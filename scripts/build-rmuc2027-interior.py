#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Build a review candidate by removing interior triangles, without refitting.

Requires the rm-map-tools CAD Python environment and a pinned visibility audit.
Visual occlusion is sampled and must be reviewed. Collision removal is restricted
to pinned sealed-source cavities and exact two-sided duplicates. Never overwrites
an input, a candidate, or the installed field package.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--audit', type=Path, required=True)
    ap.add_argument('--map-tools', type=Path, required=True)
    ap.add_argument('--recipe', type=Path, default=Path(__file__).with_name('rmuc2027-interior.json'))
    args = ap.parse_args()
    source, output = args.input.resolve(), args.output.resolve()
    if output.exists() or output.is_relative_to(source):
        ap.error('output must be new and outside the input')
    recipe = json.loads(args.recipe.read_text())
    for file, expected in recipe['input_manifests'].items():
        if sha(source/file) != expected:
            raise ValueError('reviewed input checkpoint mismatch')
    if sha(args.audit/'report.json') != recipe['audit_report_sha256']:
        raise ValueError('reviewed visibility audit mismatch')
    audit_report = json.loads((args.audit/'report.json').read_text())
    sys.path.insert(0, str(args.map_tools.resolve()/'python'))
    import numpy as np
    from gltf_scene import read_glb, mesh_instances
    from simplify_package import scene_triangles
    from rmuc2027_interior import guard_ring, pinned_cavities, duplicate_faces, exterior_guard, filter_glb
    directory = Path(__file__).resolve().parent
    visibility = module('structure_visibility', directory/'analyze-structure-visibility.py')
    optimize = module('optimize_field', directory/'optimize-field.py')
    manifest = json.loads((source/'equipment/manifest.json').read_text())
    root_manifest = json.loads((source/'manifest.json').read_text())
    descriptor = root_manifest['rmuc2027_bindings']
    if sha(source/descriptor['file']) != descriptor['sha256']:
        raise ValueError('source authority checksum mismatch')
    binding = json.loads((source/descriptor['file']).read_text())
    if binding['report']['base']['source_part_sha256'] != recipe['source_part_sha256']:
        raise ValueError('source cavity provenance mismatch')
    moving = {n for joint in binding['base_motion'] for n in joint['bodies']}
    report = {'schema_version': 1, 'input': str(source), 'recipe': recipe,
              'recipe_sha256': sha(args.recipe), 'script_sha256': sha(Path(__file__)),
              'helpers_sha256': sha(directory/'rmuc2027_interior.py'),
              'audit_sha256': recipe['audit_report_sha256'], 'assets': {},
              'input_manifests': copy.deepcopy(recipe['input_manifests']),
              'source_build_sha256': sha(source/'source-detail-build.json'),
              'policy': 'exact retained surface records; sampled visual interior removal; source-pinned collision cavities and exact duplicates'}
    report['before'] = optimize.placed_counts(source, read_glb, scene_triangles)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='rmuc2027-interior-', dir=output.parent) as temporary:
        stage = Path(temporary)/'package'
        shutil.copytree(source, stage)
        asset = manifest['assets']['base']
        for kind in ['visual', 'collision']:
            file = asset[kind]
            original = source/'equipment'/file
            if sha(original) != asset[kind+'_sha256'] or sha(original) != audit_report['assets'][kind]['sha256']:
                raise ValueError('asset/audit checksum mismatch')
            doc, binary = read_glb(original)
            parts, names = [], []
            allowed = []
            for ni, pi, p, t, mi in mesh_instances(doc, binary):
                name = doc['nodes'][ni]['name']
                parts.append(p[t])
                names.extend([name]*len(t))
                eligible = name in recipe['visual_targets'] and name not in moving
                if kind == 'visual' and name in recipe['detector_housings']:
                    eligible &= doc['materials'][mi]['name'] == recipe['detector_housings'][name]
                allowed.extend([eligible]*len(t))
            faces, names = np.concatenate(parts), np.asarray(names)
            mask_path = args.audit/f'{kind}-triangles.npz'
            if sha(mask_path) != recipe['audit_masks_sha256'][kind]:
                raise ValueError('reviewed visibility mask mismatch')
            audited = np.load(mask_path)
            if not np.array_equal(audited['faces'], faces) or not np.array_equal(audited['names'], names):
                raise ValueError('audit triangle order or positions changed')
            cavities = pinned_cavities(faces, names, recipe['source_cavities'][kind])
            metrics = {'source_cavity_triangles': int(cavities.sum())}
            if kind == 'visual':
                remove = ~audited['exposed'] & np.asarray(allowed)
                remove = guard_ring(faces, names, remove, recipe['boundary_guard_rings']) | cavities
                metrics['before_exterior_guard'] = int(remove.sum())
                remove, metrics['exterior_guard'] = exterior_guard(
                    faces, names, remove, moving, visibility.RayMesh,
                    visibility.sphere_directions(recipe['exterior_guard_views']))
                if np.any(cavities & ~remove):
                    raise ValueError('a source-pinned cavity is externally visible in this checkpoint')
            else:
                duplicates = duplicate_faces(faces, names)
                remove = duplicates | cavities
                metrics['exact_duplicate_triangles'] = int(duplicates.sum())
                metrics['duplicate_cavity_overlap'] = int((duplicates & cavities).sum())
            path = stage/'equipment'/file
            cleanup = filter_glb(path, remove)
            cleanup.update(metrics)
            cleanup['input_sha256'] = sha(original)
            cleanup['output_sha256'] = sha(path)
            report['assets'][kind] = cleanup
            asset[kind+'_sha256'] = sha(path)
            asset['triangles' if kind == 'visual' else 'collision_triangles'] = len(faces)-cleanup['removed_triangles']
            print(f'{kind}: {len(faces):,} -> {len(faces)-cleanup["removed_triangles"]:,}', flush=True)
        # These records describe inherited tessellation/decimation of retained
        # source surfaces. Interior removal is a separate subset operation.
        asset['mesh_cleanup'] = {'method': 'source-pinned-interior-removal-v1',
            'input_mesh_simplification': copy.deepcopy(asset['mesh_simplification']),
            'retained_expanded_attributes_exact': True,
            'visual_removed_triangles': report['assets']['visual']['removed_triangles'],
            'collision_removed_triangles': report['assets']['collision']['removed_triangles'],
            'report': 'interior-build.json'}
        dump(stage/'equipment/manifest.json', manifest)
        minimap = json.loads((stage/'minimap.json').read_text())
        minimap['manifest_sha256'] = {file: sha(stage/file) for file in
                                       ['manifest.json', 'equipment/manifest.json']}
        minimap['interior_cleanup_reference'] = 'source-painted minimap unchanged; exterior coordinates and floor unchanged'
        dump(stage/'minimap.json', minimap)
        if sha(stage/descriptor['file']) != descriptor['sha256']:
            raise ValueError('scoring/motion authority changed')
        report['after'] = optimize.placed_counts(stage, read_glb, scene_triangles)
        report['budget_passed'] = all(report['after'][k] < recipe['budgets'][k] for k in ['visual', 'collision'])
        dump(stage/'interior-build.json', report)
        stage.rename(output)
    return 0 if report['budget_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
