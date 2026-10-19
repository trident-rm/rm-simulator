#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Crop verified runtime terrain into the source-CAD assembly workbench frame.

Requires rm-map-tools' Python environment. Keeps intersecting source triangles
whole; this is a spatial extraction, not a terrain approximation.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import numpy as np


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--tools', type=Path, default=Path('../rm-map-tools'))
    p.add_argument('--field', type=Path, default=Path('field-2027'))
    p.add_argument('--package', type=Path, required=True)
    p.add_argument('--hierarchy', type=Path)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    sys.path.insert(0, str(a.tools.resolve()/'python'))
    from gltf_scene import read_glb, write_glb, mesh_instances
    hierarchy_path = a.hierarchy or a.tools/'out/rmuc2027-hierarchy/hierarchy.json'
    hierarchy = json.loads(hierarchy_path.read_text())
    manifest = json.loads((a.field/'manifest.json').read_text())
    package_manifest = json.loads((a.package/'manifest.json').read_text())
    if hierarchy['source_sha256'] != manifest['source_sha256'] or manifest['source_sha256'] != package_manifest['source_field_sha256']:
        raise ValueError('field/hierarchy/rig source mismatch')
    if sha(a.package/'assembly.json') != package_manifest['files']['config']['sha256']:
        raise ValueError('rig configuration checksum mismatch')
    occurrence = next(o for o in hierarchy['source_occurrences'] if o.get('product_id') == 1751926)
    source_to_arena = np.array(occurrence['source_world_matrix']); source_to_arena[:3, 3] /= 1000
    # Same audited native Core frame as bind_assembly.py, not the GLB renderer axes.
    core_c = np.array([[0., 0, -1], [1, 0, 0], [0, -1, 0]])
    core_o = np.array([-.1, .041, .084])
    native_to_workbench = np.eye(4)
    native_to_workbench[:3, :3] = core_c
    native_to_workbench[:3, 3] = -core_c@core_o
    arena_to_workbench = native_to_workbench@np.linalg.inv(source_to_arena)
    lo, hi = np.array([-1.5, -2.5, -.5]), np.array([3., 2.5, 2.5])
    a.out.mkdir(parents=True, exist_ok=True)
    doc = dict(asset=dict(version='2.0'), scene=0, scenes=[dict(nodes=[])], nodes=[], meshes=[], materials=[], buffers=[], bufferViews=[], accessors=[])
    binary = bytearray()
    def accessor(values, kind, component):
        while len(binary) % 4: binary.append(0)
        offset = len(binary); binary.extend(values.tobytes())
        view = len(doc['bufferViews']); doc['bufferViews'].append(dict(buffer=0, byteOffset=offset, byteLength=values.nbytes))
        entry = dict(bufferView=view, componentType=component, count=len(values), type=kind)
        if kind == 'VEC3': entry.update(min=values.min(axis=0).tolist(), max=values.max(axis=0).tolist())
        index = len(doc['accessors']); doc['accessors'].append(entry); return index
    all_points, all_triangles, sources = [], [], {}
    # Manifest terrain only: the articulated Core replaces the resting equipment.
    for key in ['floor', 'arena-static']:
        entry = manifest['assets'][key]
        for role in ['visual', 'collision']:
            file = a.field/entry[role]
            if sha(file) != entry[role+'_sha256']: raise ValueError(f'checksum mismatch: {file}')
            sources[f'{key}/{role}'] = dict(file=entry[role], sha256=sha(file))
            source, buf = read_glb(file)
            if source.get('textures') or source.get('images'): raise ValueError('textured terrain requires attribute-preserving extraction')
            material_offset = len(doc['materials'])
            if role == 'visual': doc['materials'].extend(copy.deepcopy(source.get('materials', [])))
            for _, _, points, triangles, material in mesh_instances(source, buf):
                points = points@arena_to_workbench[:3, :3].T+arena_to_workbench[:3, 3]
                tri = points[triangles]
                selected = np.all(tri.max(axis=1) >= lo, axis=1) & np.all(tri.min(axis=1) <= hi, axis=1)
                triangles = triangles[selected]
                if not len(triangles): continue
                used = np.unique(triangles); vertices = points[used]; indices = np.searchsorted(used, triangles)
                if role == 'collision':
                    base = sum(len(v) for v in all_points); all_points.append(vertices); all_triangles.append(indices+base)
                else:
                    # Native GLB axes, as in the source rig. Flat normals retain terrain creases.
                    v = vertices[indices].reshape(-1, 3)@np.array([[0., 0, 1], [1, 0, 0], [0, 1, 0]])
                    normals = np.cross(v.reshape(-1, 3, 3)[:, 1]-v.reshape(-1, 3, 3)[:, 0], v.reshape(-1, 3, 3)[:, 2]-v.reshape(-1, 3, 3)[:, 0])
                    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-30)
                    primitive = dict(attributes=dict(POSITION=accessor(v.astype('<f4'), 'VEC3', 5126), NORMAL=accessor(np.repeat(normals, 3, axis=0).astype('<f4'), 'VEC3', 5126)), mode=4)
                    if material is not None: primitive['material'] = material_offset+material
                    mesh = len(doc['meshes']); doc['meshes'].append(dict(primitives=[primitive]))
                    node = len(doc['nodes']); doc['nodes'].append(dict(name=f'assembly-area/{key}', mesh=mesh)); doc['scenes'][0]['nodes'].append(node)
    doc['buffers'] = [dict(byteLength=len(binary))]
    write_glb(a.out/'area.glb', doc, bytes(binary))
    vertices, indices = np.concatenate(all_points), np.concatenate(all_triangles)
    triangles = vertices[indices]
    normal = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-30)
    vertical = (abs(normal[:, 0]) > .9999) & (triangles[:, :, 0].mean(axis=1) > 0)
    walls = []
    for x in np.unique(np.round(triangles[vertical, :, 0].mean(axis=1), 6)):
        faces = triangles[vertical & (abs(triangles[:, :, 0].mean(axis=1)-x) < 2e-6)]
        low, high = faces.min(axis=(0, 1)), faces.max(axis=(0, 1))
        # Fig4-29's 2199 mm front wall, at the robot deck. Fig5-6 O is its
        # front/ground intersection midpoint, not the metal arm pedestal edge.
        if abs((high-low)[1]-2.199) < .01 and abs(low[2]-.25) < .001 and abs(high[2]-.45) < .001:
            walls.append((low, high))
    if len(walls) != 1: raise ValueError(f'expected unique rulebook front wall, found {len(walls)}')
    low, high = walls[0]
    origin = ((low+high)/2).tolist(); origin[2] = float(low[2])
    frame = dict(translation_m=origin, rotation_wxyz=[1., 0., 0., 0.])
    collision = dict(points_m=vertices.tolist(), triangles=indices.tolist())
    (a.out/'terrain.json').write_text(json.dumps(collision, separators=(',', ':'))+'\n')
    report = dict(schema=2, assembly_frame=frame, assembly_frame_evidence=dict(reference="UC2027 Fig4-29 / Fig5-6; front wall at robot deck", wall_bounds_m=[low.tolist(), high.tolist()], measured_span_m=float(high[1]-low[1]), expected_span_m=2.199), source_field_sha256=manifest['source_sha256'], config_sha256=sha(a.package/'assembly.json'), hierarchy_sha256=sha(hierarchy_path), occurrence_path=occurrence['occurrence_path'], arena_to_workbench=arena_to_workbench.tolist(), crop_bounds_m=[lo.tolist(), hi.tolist()], sources=sources, visual_sha256=sha(a.out/'area.glb'), collision_sha256=sha(a.out/'terrain.json'), collision_triangles=len(collision['triangles']), policy='Whole source triangles; same Core reference frame; support sampled from terrain, no arbitrary robot lift.')
    (a.out/'area.json').write_text(json.dumps(report, indent=2)+'\n')
    (a.out/'CAD-NOTICE.md').write_text((a.field/'CAD-NOTICE.md').read_text())
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
