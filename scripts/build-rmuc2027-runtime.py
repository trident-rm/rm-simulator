#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Adapt a pinned RMUC2027 source export to the simulator's runtime package layout.

Uses rm-map-tools' MIT/Apache-2.0 GlbWriter/gltf_scene helpers through --map-tools.
Run with its CAD Python environment (numpy). No CAD import, simplification,
2026 geometry, reconstructed optics or inferred moving joints are used.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import sys
import tempfile


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def source_product(name):
    """Source product identity is pinned by the exporter, never a fuzzy label."""
    match = re.fullmatch(r'source_(\d+)_\d+', name)
    if not match:
        raise ValueError(f'unsupported source node identity: {name}')
    return int(match.group(1))


def validate_source(manifest, recipe):
    if (manifest.get('schema_version') != 1 or manifest.get('units') != 'metres'
            or manifest.get('source_sha256') != recipe['source_sha256']
            or manifest.get('source_file') != recipe['source_file']
            or manifest.get('source_up_axis') != 'z'
            or manifest.get('body_coverage', {}).get('complete') is not True
            or manifest.get('body_coverage', {}).get('unowned')):
        raise ValueError('source identity, units, orientation or body coverage does not match recipe')
    required = {'floor', 'base', recipe['central_asset']}
    if not required.issubset(manifest['assets']):
        raise ValueError(f'missing source assets: {required - manifest["assets"].keys()}')
    ids = [entry['product_id'] for entry in recipe['selections'].values()]
    if len(ids) != len(set(ids)):
        raise ValueError('source product selections overlap')
    for name, entry in manifest['assets'].items():
        if name != 'full-map' and not entry.get('in_full_map'):
            raise ValueError(f'{name}: unplaced component cannot be silently discarded')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True, help='optimized rmuc2027 source package')
    parser.add_argument('--out', type=Path, required=True, help='new runtime package directory')
    parser.add_argument('--map-tools', type=Path, required=True)
    parser.add_argument('--recipe', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'docs/rmuc2027-runtime.json')
    args = parser.parse_args()
    if args.out.exists():
        parser.error('output exists; choose a new directory to preserve previous packages')
    import numpy as np
    sys.path.insert(0, str(args.map_tools.resolve() / 'python'))
    from export_field_package import GlbWriter
    from gltf_scene import read_glb, read_glb_local_nodes, read_glb_nodes, local_matrix
    from export_elements import quaternion_xyzw
    import provenance

    source = args.source.resolve()
    recipe = json.loads(args.recipe.read_text())
    manifest = json.loads((source / 'manifest.json').read_text())
    validate_source(manifest, recipe)
    notice = args.map_tools / 'NOTICE.md'
    if not notice.is_file():
        raise ValueError('missing upstream CAD ownership notice')
    partitions = {v['product_id']: k for k, v in recipe['selections'].items()}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=args.out.parent) as temporary:
        stage = Path(temporary) / 'field'
        (stage / 'equipment').mkdir(parents=True)
        assets = {}
        coverage = {}
        bounds = {}
        inputs = {}
        floor_top = None
        producer_records = {}
        counts = {'visual': 0, 'collision': 0}
        for kind in ('visual', 'collision'):
            writers = {}
            placements = {}
            canonical = {}

            def writer(name):
                if name not in writers:
                    writers[name] = GlbWriter(name, provenance.glb_asset(
                        'build-rmuc2027-runtime.py', recipe['source_file'], recipe['source_sha256'],
                        frame='CAD source axes, metres; runtime placement applied once',
                        collision=kind == 'collision', runtime_mode='cad-rest-pose'))
                return writers[name]

            for asset_name, entry in manifest['assets'].items():
                if asset_name == 'full-map':
                    continue  # Already composed duplicate, not an extra source occurrence.
                path = source / entry[kind]
                if digest(path) != entry[kind + '_sha256']:
                    raise ValueError(f'{asset_name}/{kind}: source checksum mismatch')
                inputs[entry[kind]] = entry[kind + '_sha256']
                document, _ = read_glb(path)
                if document.get('images') or document.get('textures'):
                    raise ValueError('textured input requires a texture-preserving compositor')
                if any('extensions' in mat for mat in document.get('materials', [])):
                    raise ValueError('extended source materials need explicit preservation support')
                for node, points, triangles, colors, local in read_glb_local_nodes(path):
                    product = source_product(node)
                    selected = partitions.get(product) if asset_name == recipe['central_asset'] else None
                    target = selected or ('base' if asset_name == 'base' else
                                          'floor' if asset_name == 'floor' else 'arena-static')
                    for occurrence, place in enumerate(entry['placements_in_source_arena_frame']):
                        matrix = np.asarray(place['matrix_local_to_arena']) @ local
                        if (not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1])
                                or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
                                or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-6)):
                            raise ValueError('source placement must be a finite rigid transform')
                        producer_records.setdefault(target, {}).setdefault(kind, {})[asset_name] = entry['mesh_simplification'][kind]
                        key = f'{asset_name}/{occurrence}/{node}'
                        coverage.setdefault(key, {})[kind] = {'runtime_asset': target, 'triangles': len(triangles)}
                        counts[kind] += len(triangles)
                        world = points @ matrix[:3, :3].T + matrix[:3, 3]
                        bounds.setdefault(target, []).append([world.min(0).tolist(), world.max(0).tolist()])
                        if target in ('floor', 'arena-static'):
                            writer(target).add_node(key, world, triangles, colors)
                            if target == 'floor' and kind == 'visual':
                                floor_top = max(floor_top if floor_top is not None else -float('inf'), float(world[:, 2].max()))
                        else:
                            # Shared source product mesh stays in its own coordinates.
                            # The first occurrence supplies geometry, every occurrence a placement.
                            signature = hashlib.sha256(points.tobytes() + triangles.tobytes() + json.dumps(colors).encode()).hexdigest()
                            if target in canonical and canonical[target] != signature:
                                raise ValueError(f'{target}: source instances do not share canonical geometry/materials')
                            canonical[target] = signature
                            if target not in placements:
                                writer(target).add_node('static', points, triangles, colors)
                                placements[target] = []
                            placements[target].append({
                                'translation_m': matrix[:3, 3].tolist(),
                                'rotation_xyzw': quaternion_xyzw(matrix[:3, :3]),
                            })
            for name, count in [('base', 2), *[(n, s['placements']) for n, s in recipe['selections'].items()]]:
                if len(placements.get(name, [])) != count:
                    raise ValueError(f'{name}: expected {count} source placements')
            for name, output in writers.items():
                relative = ('equipment/' if name in ('base', 'tech-core') else '') + name + ('' if kind == 'visual' else '-collision') + '.glb'
                output.write(stage / relative)
                entry = assets.setdefault(name, {'placements_in_source_arena_frame': placements.get(name, [])})
                entry[kind] = relative.removeprefix('equipment/')
                entry[kind + '_sha256'] = digest(stage / relative)
                entry['triangles' if kind == 'visual' else 'collision_triangles'] = output.triangles
                # Preserve the upstream collision geometry. It remains a reproducible
                # approximation, not a fresh exact tessellation or a legacy proxy.
                entry['collision_method'] = 'meshopt-simplification-v1'
                records = list(producer_records[name][kind].values())
                if any(record['method'] != 'meshopt-simplification-v1' or not record['open_borders_locked']
                       or record['position_welding'] != 'exact' for record in records):
                    raise ValueError('runtime partition requires the reviewed locked-border source simplification')
                aggregate = {key: max(record[key] for record in records) for key in
                             ('error_limit_mm', 'estimated_error_mm', 'sampled_deviation_limit_mm', 'max_sampled_deviation_mm')}
                aggregate.update(method='meshopt-simplification-v1', open_borders_locked=True, position_welding='exact')
                entry.setdefault('mesh_simplification', {})[kind] = aggregate
            if counts[kind] != manifest['assets']['full-map']['triangles' if kind == 'visual' else 'collision_triangles']:
                raise ValueError(f'{kind}: placed geometry count differs from upstream full map')
        if len(coverage) != manifest['body_coverage']['placed_products']:
            raise ValueError('runtime source occurrence count differs from upstream coverage')
        runtime_counts = {'visual': 0, 'collision': 0}
        measured_bounds = {}
        for name, entry in assets.items():
            for kind in ('visual', 'collision'):
                prefix = 'equipment/' if name in ('base', 'tech-core') else ''
                matrices = [local_matrix({'translation': p['translation_m'], 'rotation': p['rotation_xyzw']})
                            for p in entry['placements_in_source_arena_frame']] or [np.eye(4)]
                triangle_count = 0
                for _, points, triangles, _ in read_glb_nodes(stage / prefix / entry[kind]):
                    triangle_count += len(triangles)
                    for matrix in matrices:
                        world = points @ matrix[:3, :3].T + matrix[:3, 3]
                        measured_bounds.setdefault(name, []).append([world.min(0).tolist(), world.max(0).tolist()])
                if triangle_count != entry['triangles' if kind == 'visual' else 'collision_triangles']:
                    raise ValueError('written GLB triangle count differs from its manifest')
                runtime_counts[kind] += triangle_count * len(matrices)
        if runtime_counts != counts:
            raise ValueError('written runtime package omitted or duplicated placed triangles')
        residuals = {}
        for name in assets:
            source_box = np.array([np.min([b[0] for b in bounds[name]], 0), np.max([b[1] for b in bounds[name]], 0)])
            runtime_box = np.array([np.min([b[0] for b in measured_bounds[name]], 0), np.max([b[1] for b in measured_bounds[name]], 0)])
            residuals[name] = float(np.abs(source_box - runtime_box).max())
            if residuals[name] > 0.00001:
                raise ValueError(f'{name}: runtime bounds differ from source by more than 10 micrometres')
        if floor_top is None:
            raise ValueError('missing floor geometry')
        for key, record in coverage.items():
            if set(record) != {'visual', 'collision'} or record['visual']['runtime_asset'] != record['collision']['runtime_asset']:
                raise ValueError(f'inconsistent visual/collision source coverage: {key}')
        common = {'schema_version': 1, 'units': 'metres', 'season': '2027', 'rest_pose_only': True,
                  'source_file': recipe['source_file'], 'source_sha256': recipe['source_sha256'],
                  'floor_top_source_z_m': floor_top, 'assets': {}}
        field = dict(common, assets={k: v for k, v in assets.items() if k not in ('base', 'tech-core')})
        equipment = dict(common, assets={k: v for k, v in assets.items() if k in ('base', 'tech-core')})
        dump(stage / 'manifest.json', field)
        dump(stage / 'equipment/manifest.json', equipment)
        shutil.copy2(notice, stage / 'CAD-NOTICE.md')
        dump(stage / 'runtime-build.json', {
            'source_manifest_sha256': digest(source / 'manifest.json'), 'inputs_sha256': inputs,
            'recipe': recipe, 'recipe_sha256': digest(args.recipe), 'script_sha256': digest(Path(__file__)),
            'map_tools_helpers': {name: digest(args.map_tools / 'python' / name) for name in
                                  ('gltf_scene.py', 'export_field_package.py', 'export_elements.py', 'provenance.py')},
            'placed_triangles': counts, 'verified_runtime_placed_triangles': runtime_counts,
            'runtime_bounds_residual_m': residuals, 'source_coverage': coverage,
            'source_simplification_records': producer_records,
            'bounds_source_arena_m': {k: [np.min([b[0] for b in boxes], 0).tolist(),
                                          np.max([b[1] for b in boxes], 0).tolist()] for k, boxes in bounds.items()},
            'floor_reference': 'Highest vertex of source floor mesh; crowned floor heights remain geometry-driven.',
            'motion': 'CAD rest pose only; no reviewed 2027 joint or scoring bindings.',
        })
        stage.rename(args.out)
    print(args.out.resolve())


if __name__ == '__main__':
    main()
