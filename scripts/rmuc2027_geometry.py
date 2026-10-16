# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Source-aware geometry passes. Requires the rm-map-tools CAD Python environment.

Exact vertex packing retains every vertex attribute and triangle winding. Box
fitting is explicitly approximate, restricted to small connected components,
and measured against all source triangles in both directions. Paint selectors
are source-pinned separately; color or thinness alone never authorizes a decal.
"""
import copy
import io
import hashlib

import numpy as np


def pack_exact(doc, binary, preserve=()):
    """Index byte-identical complete vertex records, preserving hard normals/UVs."""
    from gltf_scene import accessor
    from simplify_package import append_accessor, compact
    data = bytearray(binary)
    before = after = 0
    skipped = {(n['mesh'], selector['primitive']) for selector in preserve
               for n in doc['nodes'] if n.get('name') == selector['node']}
    for mi, mesh in enumerate(doc['meshes']):
        for pi, primitive in enumerate(mesh['primitives']):
            attributes = {key: accessor(doc, data, index).copy()
                          for key, index in primitive['attributes'].items()}
            count = len(next(iter(attributes.values())))
            if (mi, pi) in skipped:
                before += count
                after += count
                continue
            if any(len(a) != count for a in attributes.values()):
                raise ValueError('vertex attribute length mismatch')
            records = np.concatenate([a.reshape(count, -1).view(np.uint8).reshape(count, -1)
                                      for a in attributes.values()], axis=1)
            packed = np.ascontiguousarray(records).view(np.dtype((np.void, records.shape[1]))).ravel()
            _, first, inverse = np.unique(packed, return_index=True, return_inverse=True)
            indices = (accessor(doc, data, primitive['indices']).copy() if 'indices' in primitive
                       else np.arange(count))
            before += count
            after += len(first)
            for key, values in attributes.items():
                old = doc['accessors'][primitive['attributes'][key]]
                new_index = append_accessor(doc, data, np.ascontiguousarray(values[first]),
                                            old['type'], old['componentType'])
                if 'normalized' in old:
                    doc['accessors'][new_index]['normalized'] = old['normalized']
                primitive['attributes'][key] = new_index
                # Verify expanded attributes exactly, not approximately.
                if not np.array_equal(values[indices], values[first][inverse[indices]]):
                    raise ValueError('exact packing changed an expanded vertex attribute')
            component = 5123 if len(first) < 65536 else 5125
            dtype = '<u2' if component == 5123 else '<u4'
            primitive['indices'] = append_accessor(doc, data, inverse[indices].astype(dtype), 'SCALAR', component)
    return compact(doc, data), {'vertices_before': before, 'vertices_after': after,
                                'expanded_attributes_exact': True}


def components(points, triangles):
    """Connected components after exact position welding, independent of colors."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    points, inverse = np.unique(points, axis=0, return_inverse=True)
    triangles = inverse[triangles]
    rows = np.repeat(triangles[:, 0], 2)
    cols = triangles[:, 1:].ravel()
    graph = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(len(points), len(points))).tocsr()
    _, labels = connected_components(graph, directed=False)
    keys = labels[triangles[:, 0]]
    order = np.argsort(keys, kind='stable')
    for group in np.split(order, np.flatnonzero(np.diff(keys[order])) + 1):
        faces = triangles[group]
        used, inverse = np.unique(faces, return_inverse=True)
        yield points[used], inverse.reshape(-1, 3)


def simplify_components(path, error_mm, sampled_limit_mm, preserve=(), component_preserve=()):
    """Simplify disconnected parts independently, then batch by original material.

    A difficult small fitting must not force an entire assembly to use its fine
    tolerance. Unlike separate render nodes/primitives, this preserves draw-call
    grouping. Every source component survives; no size-based deletion is used.
    """
    from gltf_scene import read_glb, write_glb, accessor, scene_nodes
    from simplify_package import simplify_mesh, append_accessor, compact, scene_triangles
    from mesh_quality import deviation
    doc, binary = read_glb(path)
    before = scene_triangles(doc)
    data = bytearray(binary)
    skipped = {(n['mesh'], s['primitive']) for s in preserve
               for n in doc['nodes'] if n.get('name') == s['node']}
    scales = {}
    for ni, world, _ in scene_nodes(doc):
        if 'mesh' in doc['nodes'][ni]:
            mi = doc['nodes'][ni]['mesh']
            scales[mi] = max(scales.get(mi, 0), float(np.linalg.svd(world[:3, :3], compute_uv=False).max()))
    records = []
    mesh_names = {}
    for node in doc['nodes']:
        if 'mesh' in node:
            mesh_names.setdefault(node['mesh'], set()).add(node.get('name'))
    pins = {(p['node'], p['positions_sha256']) for p in component_preserve}
    seen_pins = set()
    for mi, mesh in enumerate(doc['meshes']):
        if mi not in scales:
            continue
        for pi, primitive in enumerate(mesh['primitives']):
            if ((mi, pi) in skipped or set(primitive['attributes']) - {'POSITION', 'NORMAL'}
                    or primitive.get('extras', {}).get('rm', {}).get('layer') == 'markings'):
                continue
            p = accessor(doc, data, primitive['attributes']['POSITION']).copy()
            t = (accessor(doc, data, primitive['indices']).copy() if 'indices' in primitive
                 else np.arange(len(p))).astype(np.int64).reshape(-1, 3)
            chunks = []
            maximum_mm = 0.
            maximum_error_mm = 0.
            retained = 0
            pinned = 0
            scale = scales[mi]
            for points, faces in components(p, t):
                candidate = None
                signature = hashlib.sha256(np.asarray(points, dtype='<f4').tobytes()).hexdigest()
                matching = {(name, signature) for name in mesh_names[mi]} & pins
                if matching:
                    seen_pins.update(matching)
                    pinned += 1
                if len(faces) > 24 and not matching:
                    for attempt in range(6):
                        q, u, error = simplify_mesh(points, faces, error_mm / 1000 / scale / 2**attempt, False)
                        if len(u) >= len(faces):
                            break
                        quality = deviation(points, faces, q, u, count=2048)
                        if quality['max'] * scale * 1000 <= sampled_limit_mm:
                            candidate = q[u]
                            maximum_mm = max(maximum_mm, quality['max'] * scale * 1000)
                            maximum_error_mm = max(maximum_error_mm, error * scale * 1000)
                            break
                if candidate is None:
                    candidate = points[faces]
                    retained += 1
                chunks.append(candidate)
            flat = np.concatenate(chunks).reshape(-1, 3).astype('<f4')
            normals = np.cross(flat[1::3]-flat[0::3], flat[2::3]-flat[0::3])
            lengths = np.linalg.norm(normals, axis=1, keepdims=True)
            normals = np.where(lengths > 1e-30, normals / np.maximum(lengths, 1e-30), [0., 0., 1.])
            primitive['attributes'] = {
                'POSITION': append_accessor(doc, data, flat, 'VEC3', 5126),
                'NORMAL': append_accessor(doc, data, np.repeat(normals, 3, axis=0).astype('<f4'), 'VEC3', 5126)}
            primitive['indices'] = append_accessor(doc, data, np.arange(len(flat), dtype='<u4'), 'SCALAR', 5125)
            records.append({'mesh': mi, 'primitive': pi, 'before': len(t), 'after': len(flat)//3,
                            'components': len(chunks), 'components_retained_exact': retained,
                            'components_retained_by_recipe': pinned,
                            'max_sampled_deviation_mm': maximum_mm, 'estimated_error_mm': maximum_error_mm})
    if seen_pins != pins:
        raise ValueError('source-pinned precision component missing or changed')
    packed = compact(doc, data)
    write_glb(path, doc, packed)
    return {'method': 'meshopt-boundary-simplification-v1', 'component_policy': 'independent-refinement-v1',
            'error_limit_mm': error_mm, 'sampled_deviation_limit_mm': sampled_limit_mm,
            'estimated_error_mm': max((r['estimated_error_mm'] for r in records), default=0),
            'open_borders_locked': False, 'position_welding': 'exact',
            'deviation_triangles_per_direction': 2048,
            'deviation_triangles_per_component_per_direction': 2048, 'component_retention': 'all',
            'before_triangles': before, 'after_triangles': scene_triangles(doc),
            'max_sampled_deviation_mm': max((r['max_sampled_deviation_mm'] for r in records), default=0),
            'preserved_primitives': list(preserve), 'primitives': records}


def fitted_box(points, triangles, maximum_extent_m=0.05, deviation_limit_m=0.0005):
    """Return a small oriented box only when bidirectional surface checks pass.

    Large housings, grilles and open frames never qualify. Candidate box faces
    are sampled on a 2 mm grid as well as vertices/edges/centres; source sampling
    covers every source triangle. This is a sampled bound, not a Hausdorff proof.
    """
    from mesh_quality import deviation, distances
    if len(triangles) <= 12:
        return None
    unique = np.unique(points, axis=0)
    centered = unique - unique.mean(0)
    _, axes = np.linalg.eigh(centered.T @ centered)
    origin = unique.mean(0)
    local = (points-origin) @ axes
    lo, hi = local.min(0), local.max(0)
    span = hi - lo
    if span.max() > maximum_extent_m or span.min() < 0.0001:
        return None
    corners = np.array([[lo[0] if not i & 1 else hi[0],
                         lo[1] if not i & 2 else hi[1],
                         lo[2] if not i & 4 else hi[2]] for i in range(8)])
    faces = np.array([[0, 2, 3], [0, 3, 1], [4, 5, 7], [4, 7, 6],
                      [0, 1, 5], [0, 5, 4], [2, 6, 7], [2, 7, 3],
                      [0, 4, 6], [0, 6, 2], [1, 3, 7], [1, 7, 5]])
    box = corners @ axes.T + origin
    # A right/left handed PCA basis must not reverse the candidate winding.
    if np.linalg.det(axes) < 0:
        faces = faces[:, [0, 2, 1]]
    samples = []
    for axis in range(3):
        other = [i for i in range(3) if i != axis]
        grid = np.meshgrid(*[np.linspace(lo[i], hi[i], max(2, int(np.ceil(span[i] / .002)) + 1))
                             for i in other])
        for end in (lo[axis], hi[axis]):
            p = np.zeros((grid[0].size, 3))
            p[:, axis] = end
            for i, a in enumerate(other):
                p[:, a] = grid[i].ravel()
            samples.append(p @ axes.T + origin)
    quality = deviation(points, triangles, box, faces, count=len(triangles))
    maximum = max(quality['max'], float(distances(np.vstack(samples), points, triangles).max()))
    if maximum > deviation_limit_m:
        return None
    return box, faces, maximum


def merge_collision(doc, binary, fit_small_boxes=True):
    """Discard material partitions, retain source-node boundaries and all components."""
    from gltf_scene import accessor
    from simplify_package import append_accessor, compact
    data = bytearray(binary)
    meshes, report = [], []
    for node in doc['nodes']:
        if 'mesh' not in node:
            continue
        pieces = []
        offset = 0
        for primitive in doc['meshes'][node['mesh']]['primitives']:
            p = accessor(doc, data, primitive['attributes']['POSITION']).copy()
            t = (accessor(doc, data, primitive['indices']).copy() if 'indices' in primitive
                 else np.arange(len(p))).astype(np.int64).reshape(-1, 3)
            pieces.append((p, t + offset))
            offset += len(p)
        points = np.vstack([p for p, _ in pieces])
        triangles = np.vstack([t for _, t in pieces])
        chunks = []
        fits = []
        for p, t in components(points, triangles):
            fit = fitted_box(p, t) if fit_small_boxes else None
            if fit is not None:
                q, u, maximum = fit
                fits.append({'before': len(t), 'after': len(u), 'max_sampled_deviation_m': maximum,
                             'bounds_m': [p.min(0).tolist(), p.max(0).tolist()]})
                p, t = q, u
            chunks.append(p[t])
        flat = np.concatenate(chunks).reshape(-1, 3).astype('<f4')
        primitive = {'attributes': {'POSITION': append_accessor(doc, data, flat, 'VEC3', 5126)},
                     'indices': append_accessor(doc, data, np.arange(len(flat), dtype='<u4'), 'SCALAR', 5125),
                     'mode': 4}
        node['mesh'] = len(meshes)
        meshes.append({'name': node.get('name', ''), 'primitives': [primitive]})
        report.append({'node': node.get('name'), 'source_triangles': len(triangles),
                       'triangles_after_fitting': len(flat) // 3, 'components': len(chunks), 'box_fits': fits})
    doc['meshes'] = meshes
    doc.pop('materials', None)
    return compact(doc, data), report


def paint_tiles(points, triangles, tile_m=1.5):
    """Clip oversized planar artwork into atlas-sized tiles without changing coverage."""
    center = points.mean(0)
    _, basis = np.linalg.eigh((points-center).T @ (points-center))
    normal = basis[:, 0]
    axis = np.eye(3)[np.argmin(abs(normal))]
    u = axis - normal * (axis @ normal)
    u /= np.linalg.norm(u)
    axes = np.column_stack((u, np.cross(normal, u)))
    projected = points @ axes
    lo = projected.min(0)
    count = np.maximum(1, np.ceil(np.ptp(projected, axis=0) / tile_m).astype(int))
    if count.max() == 1:
        yield points, triangles
        return
    groups = {}
    for triangle in points[triangles]:
        uv = triangle @ axes
        first = np.maximum(0, np.floor((uv.min(0)-lo) / tile_m).astype(int))
        last = np.minimum(count-1, np.floor((uv.max(0)-lo) / tile_m).astype(int))
        for x in range(first[0], last[0]+1):
            for y in range(first[1], last[1]+1):
                polygon = list(triangle)
                for a, index in enumerate((x, y)):
                    for boundary, sign in [(lo[a]+index*tile_m, 1), (lo[a]+(index+1)*tile_m, -1)]:
                        clipped = []
                        for previous, current in zip(polygon[-1:]+polygon[:-1], polygon):
                            dp = sign * (previous @ axes[:, a] - boundary)
                            dc = sign * (current @ axes[:, a] - boundary)
                            if (dp >= 0) != (dc >= 0):
                                clipped.append(previous + (current-previous) * dp / (dp-dc))
                            if dc >= 0:
                                clipped.append(current)
                        polygon = clipped
                for i in range(1, len(polygon)-1):
                    face = np.array([polygon[0], polygon[i], polygon[i+1]])
                    if np.linalg.norm(np.cross(face[1]-face[0], face[2]-face[0])) > 1e-15:
                        groups.setdefault((x, y), []).append(face)
    for faces in groups.values():
        flat = np.asarray(faces).reshape(-1, 3)
        yield flat, np.arange(len(flat)).reshape(-1, 3)


def paint_atlas(doc, binary, selections):
    """Separate pinned paint into atlases, retaining structural backing at each cap.

    Caps become the recipe's backing color and stay present in the structural
    mesh, so converting paint never punches a hole through a deck. The original
    paint caps are cloned into sibling-local components for the upstream baker.
    """
    from gltf_scene import accessor
    from simplify_package import append_accessor, compact
    from texture_atlas import bake
    data = bytearray(binary)
    patches = []
    selection_records = []
    for selection in selections:
        nodes = [n for n in doc['nodes'] if n.get('name') == selection['node']]
        if len(nodes) != 1 or 'mesh' not in nodes[0]:
            raise ValueError('paint source node missing or ambiguous')
        node = nodes[0]
        mesh = copy.deepcopy(doc['meshes'][node['mesh']])
        node['mesh'] = len(doc['meshes'])
        doc['meshes'].append(mesh)
        matches = [p for p in mesh['primitives']
                   if doc['materials'][p['material']]['name'] == selection['material']]
        backs = [i for i, m in enumerate(doc['materials']) if m.get('name') == selection['backing_material']]
        if len(matches) != 1 or len(backs) != 1:
            raise ValueError('paint material/backing missing or ambiguous')
        original = copy.deepcopy(matches[0])
        matches[0]['material'] = backs[0]
        p = accessor(doc, data, original['attributes']['POSITION']).copy()
        t = accessor(doc, data, original['indices']).reshape(-1, 3).copy()
        # The recipe only selects planar source-painted caps, not arbitrary
        # white structures or electronic modules. Refuse thick geometry.
        center = p.mean(0)
        _, basis = np.linalg.eigh((p-center).T @ (p-center))
        thickness = float(np.ptp((p-center) @ basis[:, 0]))
        if thickness > .0015:
            raise ValueError('paint cap exceeds reviewed 1.5 mm plane tolerance')
        if selection.get('encoding') == 'geometry':
            # Sparse straight boundaries already use very few triangles.
            # Retain them exactly apart from the deliberate 0.5 mm depth offset,
            # rather than allocating mostly transparent atlas pages for lines.
            key = hashlib.sha256(selection['material'].encode()).hexdigest()[:8]
            name = selection['node'] + '/paint_geometry_' + key
            if any(n.get('name') == name for n in doc['nodes']):
                raise ValueError('paint anchor collision')
            normals = accessor(doc, data, original['attributes']['NORMAL']).copy()
            original['attributes']['POSITION'] = append_accessor(
                doc, data, np.asarray(p+normals*.0005, dtype='<f4'), 'VEC3', 5126)
            original['extras'] = {'rm': {'layer': 'markings', 'roles': ['decoration'], 'collision': False}}
            node.setdefault('children', []).append(len(doc['nodes']))
            doc['nodes'].append({'name': name, 'mesh': len(doc['meshes'])})
            doc['meshes'].append({'primitives': [original]})
            selection_records.append({**selection, 'source_triangles': len(t), 'thickness_m': thickness})
            continue
        tiled = (tile for points, faces in components(p, t) for tile in paint_tiles(points, faces))
        for index, (points, faces) in enumerate(tiled):
            if len(faces) < 1:
                continue
            key = hashlib.sha256(selection['material'].encode()).hexdigest()[:8]
            name = selection['node'] + '/paint_' + key + '_' + str(index)
            if any(n.get('name') == name for n in doc['nodes']):
                raise ValueError('paint anchor collision')
            pr = copy.deepcopy(original)
            pr['attributes'] = {'POSITION': append_accessor(doc, data, points.astype('<f4'), 'VEC3', 5126)}
            pr['indices'] = append_accessor(doc, data, faces.astype('<u4').ravel(), 'SCALAR', 5125)
            ni = len(doc['nodes'])
            node.setdefault('children', []).append(ni)
            doc['nodes'].append({'name': name, 'mesh': len(doc['meshes'])})
            doc['meshes'].append({'primitives': [pr]})
            patches.append({'node': name, 'material': selection['material']})
        selection_records.append({**selection, 'source_triangles': len(t), 'thickness_m': thickness})
    doc['buffers'][0]['byteLength'] = len(data)
    doc, packed, pages, records = bake(doc, bytes(data), patches, density=1024, size=2048,
                                      tolerance=.0015, merge_distance=.04)
    pngs = []
    for page in pages:
        stream = io.BytesIO()
        page.save(stream, format='PNG')
        pngs.append(stream.getvalue())
    return doc, compact(doc, packed), pngs, records, selection_records


def batch_paint(doc, binary):
    """Batch fixed arena artwork by atlas material into an independent visual layer.

    Only identity scene transforms are accepted. Moving equipment must retain
    its own anchors instead of entering this fixed scenery layer.
    """
    from gltf_scene import scene_nodes, accessor
    from simplify_package import append_accessor, compact
    data = bytearray(binary)
    groups = {}
    count = 0
    for ni, world, _ in list(scene_nodes(doc)):
        node = doc['nodes'][ni]
        if 'mesh' not in node:
            continue
        mesh = copy.deepcopy(doc['meshes'][node['mesh']])
        retained = []
        for primitive in mesh['primitives']:
            if primitive.get('extras', {}).get('rm', {}).get('layer') != 'markings':
                retained.append(primitive)
                continue
            if not np.array_equal(world, np.eye(4)):
                raise ValueError('fixed paint batching cannot flatten a transformed or moving anchor')
            indices = accessor(doc, data, primitive['indices'])
            arrays = {key: accessor(doc, data, index)[indices].copy()
                      for key, index in primitive['attributes'].items()}
            groups.setdefault(primitive['material'], []).append(arrays)
            count += 1
        if retained:
            node['mesh'] = len(doc['meshes'])
            doc['meshes'].append({**mesh, 'primitives': retained})
        else:
            node.pop('mesh')
    primitives = []
    for material, chunks in groups.items():
        attrs = {}
        for key in chunks[0]:
            values = np.concatenate([c[key] for c in chunks]).astype('<f4')
            attrs[key] = append_accessor(doc, data, values, 'VEC'+str(values.shape[1]), 5126)
        length = sum(len(c['POSITION']) for c in chunks)
        primitives.append({'attributes': attrs, 'indices': append_accessor(
            doc, data, np.arange(length, dtype='<u4'), 'SCALAR', 5125), 'material': material,
            'extras': {'rm': {'layer': 'decoration', 'roles': ['decoration'], 'collision': False}}})
    if groups:
        name = 'rmuc2027.decoration'
        if any(n.get('name') == name for n in doc['nodes']):
            raise ValueError('decoration layer already exists')
        doc['scenes'][doc.get('scene', 0)]['nodes'].append(len(doc['nodes']))
        doc['nodes'].append({'name': name, 'mesh': len(doc['meshes']),
                             'extras': {'rm': {'layer': 'decoration', 'roles': ['decoration'], 'collision': False}}})
        doc['meshes'].append({'name': name, 'primitives': primitives})
    return compact(doc, data), {'patches': count, 'batched_primitives': len(primitives)}
