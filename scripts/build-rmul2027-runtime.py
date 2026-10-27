#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Tessellate the checksum-pinned official RMUL fields, retaining every face/color.

Run with rm-map-tools' CAD Python environment. --map-tools supplies its existing
OCCT importer and GLB writer; no dependency on that repository enters Rust.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

SOURCES = {
    '3v3': ('RMUL2027_3V3_V1.0.0.stp', '7d47c1b6a6a0253a732e46cb122bb84410b11d5134bcb5658104299db4a48ca4', 'ThreeVsThree', -1),
    '1v1': ('RMUL2027_1V1_V1.0.0.stp', '954936e74885493836d73ba215e1c42efc3b6d40279478f5e5d626109e2cf6d1', 'Infantry', 1),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-tools', type=Path, required=True)
    parser.add_argument('--format', choices=SOURCES, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    name, source_hash, format_name, up = SOURCES[args.format]
    source = args.map_tools / 'source' / name
    if digest(source) != source_hash:
        parser.error('source SHA-256 differs from the official release')
    if args.out.exists():
        parser.error('output already exists; preserve it and choose a new directory')
    sys.path.insert(0, str(args.map_tools.resolve() / 'python'))
    import numpy as np
    from validate_parts import read_part
    from mesh_parts import tessellate
    from export_field_package import GlbWriter
    import provenance
    doc, _, error = read_part(str(source.resolve()))
    if error:
        raise ValueError(error)
    stats = {}
    faces = []
    points, triangles, color_ids, palette = tessellate(
        doc, 2., .35, stats=stats,
        face_observer=lambda index, face, count: faces.append(count))
    if stats.get('unmeshed_faces', 0):
        raise ValueError(f'source faces could not be tessellated: {stats}')
    # 3v3 is -Y up; the 1v1 covers extend along +Y. Both transforms are proper
    # rotations. Their source floor surfaces are at Y=0; no scale fitting.
    points = points[:, [0, 2, 1]] * np.array([1., -up, up]) / 1000.
    bounds = [points.min(axis=0).tolist(), points.max(axis=0).tolist()]
    points[:, 1] += 1.6243436  # Existing CAD arena-frame origin convention.
    args.out.mkdir(parents=True)
    (args.out / 'equipment').mkdir()
    asset = provenance.glb_asset('build-rmul2027-runtime.py', name, source_hash,
                                  frame='Z-up arena, metres', collision=False)
    writer = GlbWriter('official-rmul-' + args.format, asset)
    for index, color in enumerate(palette):
        selected = triangles[color_ids == index]
        writer.add_node(f'source-material-{index}', points, selected, [color] * len(selected))
    writer.write(args.out / 'floor.glb')
    GlbWriter('empty-equipment', asset).write(args.out / 'empty.glb')
    shutil.copy2(args.out / 'empty.glb', args.out / 'equipment/empty.glb')
    def entry(file):
        return {'visual': file, 'visual_sha256': digest(args.out / file),
                'placements_in_source_arena_frame': []}
    common = {'schema_version': 1, 'units': 'metres', 'season': '2027',
              'rest_pose_only': True, 'rmul2027_format': format_name,
              'source_file': name, 'source_sha256': source_hash,
              'floor_top_source_z_m': 0., 'collision_solids': False}
    dump(args.out / 'manifest.json', {**common, 'assets': {
        'floor': entry('floor.glb'), 'arena-static': entry('empty.glb'),
        'rune': entry('empty.glb'), 'outpost': entry('empty.glb')}})
    empty = {'visual': 'empty.glb', 'visual_sha256': digest(args.out / 'empty.glb'),
             'placements_in_source_arena_frame': []}
    dump(args.out / 'equipment/manifest.json', {**common, 'assets': {'base': empty, 'tech-core': empty}})
    shutil.copy2(args.map_tools / 'NOTICE.md', args.out / 'CAD-NOTICE.md')
    dump(args.out / 'runtime-build.json', {
        'source_url': 'https://bbs.robomaster.com/article/1970956',
        'source_sha256': source_hash, 'source_up_axis': '+y' if up == 1 else '-y',
        'bounds_flu_m': bounds, 'floor_top_m': 0., 'triangles': len(triangles),
        'linear_deflection_mm': 2., 'angular_deflection_rad': .35,
        'source_faces': len(faces), 'tessellation_diagnostics': stats,
        'collision': 'same source triangles as visuals; no proxy or simplification',
        'rfid': 'source-aligned approximate antenna footprints, not source geometry',
    })
    print(args.format, bounds, len(triangles), 'triangles')


if __name__ == '__main__':
    main()
