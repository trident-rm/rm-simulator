#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Replace the four approved center-platform vent grates with opaque panels.

This intentionally closes their visual and projectile openings. Each panel is
an extrusion of its visual source envelope; both files receive identical panel
surfaces. Optional source-pinned foot plinths close only short wall/floor gaps.
Other geometry, equipment and rule bindings remain unchanged.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def solid_panel(points, normal, tolerance_m=1e-5, thickness_m=None, outward_offset_m=0.):
    """Extrude a convex projected outline across the existing frame thickness.

    Coordinates use metres. An optional thinner panel uses the source backing
    plane as its inward cap; outward offset avoids coplanar wall overlap.
    Near-collinear hull vertices are removed within
    tolerance; the source's chamfered corners remain. Returns indexed vertices,
    outward-wound triangles and envelope measurements. Openings are filled.
    """
    import numpy as np
    from scipy.spatial import ConvexHull
    p, n = np.asarray(points, dtype=float), np.array(normal, dtype=float, copy=True)
    if (p.ndim != 2 or p.shape[1] != 3 or not np.isfinite(p).all()
            or n.shape != (3,) or not np.isfinite(n).all() or np.linalg.norm(n) < .99):
        raise ValueError('finite points and a unit panel normal required')
    n /= np.linalg.norm(n)
    u = np.cross(n, [0, 0, 1])
    if np.linalg.norm(u) < .1:
        raise ValueError('expected a near-vertical side panel')
    u /= np.linalg.norm(u)
    basis = np.column_stack([u, np.cross(n, u), n])
    center = p.mean(0)
    q = (p-center) @ basis
    lo, hi = q[:, 2].min(), q[:, 2].max()
    if not .001 <= hi-lo <= .030:
        raise ValueError('unexpected frame thickness')
    polygon = q[ConvexHull(q[:, :2]).vertices, :2].copy()
    changed = True
    while changed and len(polygon) > 3:
        changed = False
        for i in range(len(polygon)):
            a, b, c = polygon[i-1], polygon[i], polygon[(i+1) % len(polygon)]
            edge = c-a
            distance = abs(edge[0]*(b-a)[1]-edge[1]*(b-a)[0])/np.linalg.norm(edge)
            if distance <= tolerance_m:
                polygon = np.delete(polygon, i, axis=0)
                changed = True
                break
    edge = np.roll(polygon, -1, axis=0)-polygon
    delta = q[None, :, :2]-polygon[:, None, :]
    signed = (edge[:, None, 0]*delta[:, :, 1]-edge[:, None, 1]*delta[:, :, 0])/np.linalg.norm(edge, axis=1)[:, None]
    if signed.min() < -2*tolerance_m:
        raise ValueError('outline simplification exceeded envelope tolerance')
    source_thickness = float(hi-lo)
    if thickness_m is not None:
        if not np.isfinite(thickness_m) or not 0 < thickness_m <= source_thickness:
            raise ValueError('panel thickness must fit within the source frame')
        lo = hi-thickness_m
    if not np.isfinite(outward_offset_m) or not 0 <= outward_offset_m <= .001:
        raise ValueError('invalid outward surface offset')
    lo -= outward_offset_m
    hi -= outward_offset_m
    count = len(polygon)
    corners = np.vstack([np.column_stack([polygon, np.full(count, z)]) for z in [lo, hi]])
    faces = []
    for i in range(1, count-1):
        faces.extend([[0, i+1, i], [count, count+i, count+i+1]])
    for i in range(count):
        j = (i+1) % count
        faces.extend([[i, j, count+j], [i, count+j, count+i]])
    return corners @ basis.T+center, np.asarray(faces), {
        'outline_vertices': count, 'thickness_m': float(hi-lo),
        'source_frame_thickness_m': source_thickness,
        'outward_offset_m': outward_offset_m,
        'outline_inset_limit_m': tolerance_m*2,
        'source_bounds_m': [p.min(0).tolist(), p.max(0).tolist()]}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--map-tools', type=Path, required=True)
    ap.add_argument('--recipe', type=Path, default=Path(__file__).with_name('rmuc2027-solid-walls.json'))
    args = ap.parse_args()
    source, output = args.input.resolve(), args.output.resolve()
    if output.exists() or output.is_relative_to(source):
        ap.error('output must be new and outside input')
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    recipe = json.loads(args.recipe.read_text())
    for name, digest in recipe['input_manifests'].items():
        if sha(source/name) != digest:
            raise ValueError('input checkpoint mismatch')
    sys.path.insert(0, str(args.map_tools.resolve()/'python'))
    import numpy as np
    from gltf_scene import read_glb, write_glb, accessor
    from simplify_package import append_accessor, compact, scene_triangles
    spec = importlib.util.spec_from_file_location('optimize', Path(__file__).with_name('optimize-field.py'))
    optimize = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(optimize)
    manifest = json.loads((source/'manifest.json').read_text())
    asset = manifest['assets']['arena-static']
    doc, binary = read_glb(source/asset['visual'])
    replacements, measurements = {}, {}
    for name, normal in recipe['panels'].items():
        node = next(n for n in doc['nodes'] if n.get('name') == name)
        if any(k in node for k in ['matrix', 'translation', 'rotation', 'scale']):
            raise ValueError('expected baked source node')
        primitives = doc['meshes'][node['mesh']]['primitives']
        if len(primitives) != 1:
            raise ValueError('expected one visual material')
        p = accessor(doc, binary, primitives[0]['attributes']['POSITION'])
        replacements[name] = solid_panel(p, normal, thickness_m=recipe['panel_thickness_m'],
                                         outward_offset_m=recipe['outward_offset_m'])
        measurements[name] = replacements[name][2]
    if recipe.get('foot_plinths'):
        from rmuc2027_skirts import foot_plinths
        from gltf_scene import mesh_instances
        floor_entry = manifest['assets']['floor']
        floor_path = source/floor_entry['collision']
        if sha(floor_path) != floor_entry['collision_sha256']:
            raise ValueError('floor checksum mismatch')
        floor_doc, floor_binary = read_glb(floor_path)
        floor_faces = np.concatenate([p[t] for _, _, p, t, _ in mesh_instances(floor_doc, floor_binary)])
        wall_faces = [p[t] for ni, _, p, t, mi in mesh_instances(doc, binary)
                      if doc['nodes'][ni].get('name', '').startswith('centre-platform/0/')
                      and mi is not None and doc['materials'][mi].get('name') == recipe['wall_material']]
        wall_faces.extend(p[t] for p, t, _ in replacements.values())
        wall_faces = np.concatenate(wall_faces)
        for name, normal in recipe['panels'].items():
            if name not in recipe['foot_plinth_panels']:
                continue
            p, t, metrics = replacements[name]
            level = float((p @ np.asarray(normal)).min())
            extra, faces, records = foot_plinths(wall_faces, floor_faces, normal, level,
                thickness_m=recipe['panel_thickness_m'], **recipe['foot_plinths'])
            metrics['foot_plinths'] = records
            if len(extra):
                replacements[name] = (np.vstack([p, extra]), np.vstack([t, faces+len(p)]), metrics)
    if recipe.get('foot_plinths') and sum(len(m.get('foot_plinths', [])) for _, _, m in replacements.values()) != recipe['expected_foot_plinth_count']:
        raise ValueError('reviewed foot-gap coverage mismatch')
    report = {'schema_version': 1, 'recipe': recipe,
              'script_sha256': sha(Path(__file__)), 'recipe_sha256': sha(args.recipe),
              'plinth_helper_sha256': sha(Path(__file__).with_name('rmuc2027_skirts.py')),
              'policy': 'user-approved opaque center-platform side walls; vent openings and selected short foot gaps intentionally closed',
              'before': optimize.placed_counts(source, read_glb, scene_triangles), 'assets': {}}
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='solid-walls-', dir=output.parent) as tmp:
        stage = Path(tmp)/'package'
        shutil.copytree(source, stage)
        for kind in ['visual', 'collision']:
            path = stage/asset[kind]
            if sha(path) != asset[kind+'_sha256']:
                raise ValueError('input asset checksum mismatch')
            d, b = read_glb(path)
            data = bytearray(b)
            before = scene_triangles(d)
            details = []
            for name, (p, t, measurement) in replacements.items():
                nodes = [n for n in d['nodes'] if n.get('name') == name]
                if len(nodes) != 1:
                    raise ValueError('panel node selector mismatch')
                node = nodes[0]
                old = d['meshes'][node['mesh']]['primitives']
                old_count = sum(d['accessors'][pr.get('indices', pr['attributes']['POSITION'])]['count']//3 for pr in old)
                flat = np.asarray(p[t].reshape(-1, 3), dtype='<f4')
                f = flat.reshape(-1, 3, 3)
                normals = np.cross(f[:, 1]-f[:, 0], f[:, 2]-f[:, 0])
                normals /= np.linalg.norm(normals, axis=1)[:, None]
                primitive = {'attributes': {
                    'POSITION': append_accessor(d, data, flat, 'VEC3', 5126),
                    'NORMAL': append_accessor(d, data, np.repeat(normals, 3, axis=0).astype('<f4'), 'VEC3', 5126)},
                    'indices': append_accessor(d, data, np.arange(len(flat), dtype='<u2'), 'SCALAR', 5123)}
                if kind == 'visual':
                    materials = [i for i, m in enumerate(d['materials']) if m.get('name') == recipe['wall_material']]
                    if len(materials) != 1:
                        raise ValueError('wall material missing or ambiguous')
                    primitive['material'] = materials[0]
                # New mesh reference leaves any other node sharing the old one intact.
                node['mesh'] = len(d['meshes'])
                d['meshes'].append({'name': name+'/solid-wall', 'primitives': [primitive]})
                details.append(dict(node=name, before=old_count, after=len(t), **measurement))
            write_glb(path, d, compact(d, data))
            original_doc, _ = read_glb(source/asset[kind])
            preserved = [{'node': node['name'], 'primitive': pi}
                         for node in original_doc['nodes']
                         if 'mesh' in node and node.get('name') not in replacements
                         for pi in range(len(original_doc['meshes'][node['mesh']]['primitives']))]
            optimize.verify_preserved(source/asset[kind], path, preserved, read_glb, accessor)
            after = scene_triangles(d)
            asset[kind+'_sha256'] = sha(path)
            asset['triangles' if kind == 'visual' else 'collision_triangles'] = after
            report['assets'][kind] = {'before': before, 'after': after, 'panels': details, 'other_primitives_verified_exact': len(preserved)}
            print(kind, before, '->', after, flush=True)
        asset['collision_method'] = 'opaque-panel-reconstruction-v1'
        asset['mesh_reconstruction'] = {'method': 'opaque-panel-reconstruction-v1',
            'report': 'solid-wall-build.json', 'closed_openings': True,
            'visual_collision_panels_identical': True,
            'inherited_mesh_simplification': copy.deepcopy(asset['mesh_simplification'])}
        (stage/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        subprocess.run([sys.executable, str(Path(__file__).with_name('generate-minimap.py')),
                        '--cad-assets', str(stage), '--map-tools', str(args.map_tools.resolve())], check=True)
        report['after'] = optimize.placed_counts(stage, read_glb, scene_triangles)
        report['budget_passed'] = all(report['after'][k] < recipe['budgets'][k] for k in ['visual', 'collision'])
        (stage/'solid-wall-build.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        stage.rename(output)


if __name__ == '__main__':
    main()
