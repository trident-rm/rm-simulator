# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Subset-only interior cleanup; no position, normal, material or pose fitting."""
import hashlib
import numpy as np


def face_components(faces):
    """Return triangle indices connected by exact coincident vertices."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    p, inv = np.unique(faces.reshape(-1, 3), axis=0, return_inverse=True)
    t = inv.reshape(-1, 3)
    graph = coo_matrix((np.ones(len(t)*2), (np.repeat(t[:, 0], 2), t[:, 1:].ravel())),
                       shape=(len(p), len(p))).tocsr()
    _, labels = connected_components(graph, directed=False)
    keys = labels[t[:, 0]]
    order = np.argsort(keys, kind='stable')
    return np.split(order, np.flatnonzero(np.diff(keys[order]))+1)


def signature(faces):
    """Hash sorted exact component positions, as pinned by the source-shell audit."""
    p = np.unique(faces.reshape(-1, 3), axis=0)
    return hashlib.sha256(np.asarray(p, dtype='<f4').tobytes()).hexdigest()


def guard_ring(faces, names, remove, rings=1):
    """Retain vertex-adjacent triangles around all retained surface boundaries."""
    keep = ~remove.copy()
    for _ in range(rings):
        for name in np.unique(names):
            ids = np.flatnonzero(names == name)
            p, inv = np.unique(faces[ids].reshape(-1, 3), axis=0, return_inverse=True)
            t = inv.reshape(-1, 3)
            vertices = np.zeros(len(p), bool)
            vertices[t[keep[ids]].ravel()] = True
            keep[ids] |= vertices[t].any(1)
    return ~keep


def duplicate_faces(faces, names):
    """Mark duplicate two-sided contact triangles within each source body."""
    remove = np.zeros(len(faces), bool)
    for name in np.unique(names):
        ids = np.flatnonzero(names == name)
        _, inv = np.unique(faces[ids].reshape(-1, 3), axis=0, return_inverse=True)
        _, first = np.unique(np.sort(inv.reshape(-1, 3), axis=1), axis=0, return_index=True)
        remove[ids] = True
        remove[ids[first]] = False
    return remove


def pinned_cavities(faces, names, selections):
    """Find every reviewed sealed-source component; fail if any signature changes."""
    remove = np.zeros(len(faces), bool)
    for selection in selections:
        ids = np.flatnonzero(names == selection['node'])
        matches = [ids[c] for c in face_components(faces[ids])
                   if signature(faces[ids[c]]) == selection['positions_sha256']]
        if len(matches) != 1 or len(matches[0]) != selection['triangles']:
            raise ValueError('source-pinned cavity component changed or missing')
        remove[matches[0]] = True
    return remove


def exterior_guard(faces, names, remove, moving, ray_mesh, directions):
    """Restore removals hit first by independent exterior orthographic views.

    Fixed-body rays omit the movable shields. No geometry changes are used to
    make the check pass. Each tested ray retains its exact original first-hit
    triangle, including source attributes. This is sampled, not a CAD proof.
    """
    import vtk
    fixed = np.array([n not in moving for n in names])
    ids = np.flatnonzero(fixed)
    source = faces[ids]
    mesh = ray_mesh(source)
    center = (source.reshape(-1, 3).min(0)+source.reshape(-1, 3).max(0))/2
    reach = float(np.linalg.norm(np.ptp(source.reshape(-1, 3), axis=0))*3+1)
    restored = np.zeros(len(faces), bool)
    t, sub, cid = vtk.reference(0.), vtk.reference(0), vtk.reference(0)
    x, pc, cell = [0.]*3, [0.]*3, vtk.vtkGenericCell()
    rays = 0
    for vi, direction in enumerate(directions):
        up = np.eye(3)[np.argmin(np.abs(direction))]
        right = np.cross(up, direction)
        right /= np.linalg.norm(right)
        up = np.cross(direction, right)
        local = (source.reshape(-1, 3)-center) @ np.array([right, up]).T
        lo, hi = local.min(0)-.002, local.max(0)+.002
        # Sample pixel centers over the projected bounds from each direction.
        grid = [np.linspace(lo[k], hi[k], 192, endpoint=False)+(hi[k]-lo[k])/384 for k in range(2)]
        for a in grid[0]:
            for b in grid[1]:
                point = center + a*right + b*up
                hit = mesh.locator.IntersectWithLine(point+direction*reach, point-direction*reach,
                                                     1e-9, t, x, pc, sub, cid, cell)
                rays += 1
                if hit and remove[ids[int(cid)]]:
                    restored[ids[int(cid)]] = True
        if vi % 16 == 0:
            print(f'exterior guard {vi+1}/{len(directions)}: restored {int(restored.sum())}', flush=True)
    return remove & ~restored, {'views': len(directions), 'pixels_per_axis': 192,
                                'rays': rays, 'triangles_restored': int(restored.sum()),
                                'moving_occluders': 'omitted', 'exact_first_hit_retained': True}


def filter_glb(path, remove):
    """Write an exact subset of expanded triangle records, retaining primitive slots."""
    from gltf_scene import read_glb, write_glb, accessor, mesh_instances
    from simplify_package import append_accessor, compact
    doc, binary = read_glb(path)
    data = bytearray(binary)
    offset = 0
    removed = 0
    records = []
    seen = set()
    for ni, pi, p, t, _ in mesh_instances(doc, binary):
        node = doc['nodes'][ni]
        key = (node['mesh'], pi)
        if key in seen:
            raise ValueError('instanced primitives require shared removal decisions')
        seen.add(key)
        primitive = doc['meshes'][node['mesh']]['primitives'][pi]
        keep = ~remove[offset:offset+len(t)]
        offset += len(t)
        # Keep semantic primitive indices and material slots stable, including
        # uncommon primitives which would otherwise become completely empty.
        if not keep.any():
            keep[:] = True
        arrays = {k: accessor(doc, data, index).copy() for k, index in primitive['attributes'].items()}
        indices = accessor(doc, data, primitive['indices']).reshape(-1, 3)
        retained = indices[keep]
        used, inverse = np.unique(retained, return_inverse=True)
        for key, values in arrays.items():
            old = doc['accessors'][primitive['attributes'][key]]
            new = append_accessor(doc, data, values[used], old['type'], old['componentType'])
            if 'normalized' in old:
                doc['accessors'][new]['normalized'] = old['normalized']
            primitive['attributes'][key] = new
            if not np.array_equal(values[retained], values[used][inverse.reshape(-1, 3)]):
                raise ValueError('retained expanded attribute changed')
        component = 5123 if len(used) < 65536 else 5125
        dtype = '<u2' if component == 5123 else '<u4'
        primitive['indices'] = append_accessor(doc, data, inverse.astype(dtype).ravel(), 'SCALAR', component)
        count = int((~keep).sum())
        removed += count
        records.append({'node': node['name'], 'primitive': pi, 'before': len(t),
                        'removed': count, 'after': len(retained)})
    if offset != len(remove):
        raise ValueError('triangle mask length mismatch')
    write_glb(path, doc, compact(doc, data))
    return {'removed_triangles': removed, 'retained_expanded_attributes_exact': True,
            'primitive_slots_retained': True, 'primitives': records}
