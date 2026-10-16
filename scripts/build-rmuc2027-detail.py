#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Build source-aware render/collision derivatives using rm-map-tools helpers.

Run in rm-map-tools/ocpenv. Inputs are immutable, source-pinned checkpoints.
All stages and their errors are reported separately; exact packing is not
presented as lossless triangle decimation. Never overwrites an input or release.
"""
import argparse
import copy
import hashlib
import importlib.util
import importlib.metadata
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--map-tools', type=Path, required=True)
    parser.add_argument('--recipe', type=Path, default=Path(__file__).with_name('rmuc2027-detail.json'))
    args = parser.parse_args()
    source, output, producer = args.input.resolve(), args.output.resolve(), args.map_tools.resolve()
    if output.exists() or output.is_relative_to(source):
        parser.error('output must be new and outside source')
    recipe = json.loads(args.recipe.read_text())
    for scope, expected in recipe['input_manifests'].items():
        if sha(source / scope / 'manifest.json') != expected:
            raise ValueError('recipe input checkpoint mismatch; source selections require review')
    sys.path.insert(0, str(producer / 'python'))
    from gltf_scene import read_glb, write_glb, accessor
    from simplify_package import scene_triangles
    from attach_texture_atlas import attach
    from rmuc2027_geometry import pack_exact, merge_collision, paint_atlas, simplify_components, batch_paint
    if importlib.metadata.version('meshoptimizer') != '0.2.30a0':
        raise ValueError('requires meshoptimizer==0.2.30a0')
    spec = importlib.util.spec_from_file_location('optimize_field', Path(__file__).with_name('optimize-field.py'))
    optimize = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(optimize)
    protected = optimize.rmuc2027_preservation(source, read_glb, protect_moving=False)
    if not protected:
        raise ValueError('bound RMUC 2027 package required')
    report = {'schema_version': 1, 'recipe': recipe, 'input': str(source),
              'input_manifests': {}, 'assets': {}, 'script_sha256': sha(Path(__file__)),
              'geometry_helpers_sha256': sha(Path(__file__).with_name('rmuc2027_geometry.py')),
              'adapter_helpers_sha256': sha(Path(__file__).with_name('optimize-field.py')),
              'minimap_generator_sha256': sha(Path(__file__).with_name('generate-minimap.py')),
              'recipe_sha256': sha(args.recipe),
              'dependencies': {p: importlib.metadata.version(p) for p in ('meshoptimizer', 'numpy', 'scipy', 'vtk', 'Pillow')},
              'producer_files': {p.name: sha(p) for p in sorted((producer / 'python').glob('*.py'))},
              'measurement_reference': 'original bound checkpoint, not STEP; inherited errors retained',
              'policy': 'exact visual detector modules and rule bindings; approximate other geometry independently'}
    report['before'] = optimize.placed_counts(source, read_glb, scene_triangles)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='rmuc2027-detail-', dir=output.parent) as temporary:
        stage = Path(temporary) / 'package'
        shutil.copytree(source, stage)
        # Use source-painted geometry for tactical artwork: transparent atlas
        # triangles are intentionally skipped by the geometry-only map baker.
        subprocess.run([sys.executable, str(Path(__file__).with_name('generate-minimap.py')),
                        '--cad-assets', str(stage), '--map-tools', str(producer)], check=True)
        for scope in ('', 'equipment'):
            mpath = stage / scope / 'manifest.json'
            manifest = json.loads(mpath.read_text())
            report['input_manifests'][scope] = copy.deepcopy(manifest)
            for name, asset in manifest['assets'].items():
                if name == 'floor':
                    continue
                upstream = copy.deepcopy(asset.get('mesh_simplification'))
                result = {'upstream': upstream}
                for kind in ('visual', 'collision'):
                    path = stage / scope / asset[kind]
                    if sha(path) != asset[kind + '_sha256']:
                        raise ValueError('input GLB checksum mismatch')
                    doc, binary = read_glb(path)
                    before = scene_triangles(doc)
                    selections = protected.get((scope, name, kind), []) if kind == 'visual' else []
                    pngs, patches = [], []
                    extra = {}
                    if kind == 'visual' and name == 'arena-static':
                        doc, binary, pngs, patches, paints = paint_atlas(doc, binary, recipe['paint'])
                        extra['paint'] = {'selections': paints, 'pages': len(pngs), 'patches': patches,
                                          'pixels_per_metre': 1024, 'offset_m': .0005}
                    if kind == 'collision':
                        binary, fits = merge_collision(doc, binary)
                        extra['color_independent_components'] = fits
                    write_glb(path, doc, binary)
                    error = recipe[kind + '_error_mm']
                    simplification = simplify_components(path, error, recipe[kind + '_sampled_limit_mm'],
                        preserve=selections, component_preserve=(
                            recipe.get('precision_components', {}).get(asset[kind], []) if kind == 'collision' else []))
                    if selections:
                        optimize.verify_preserved(source / scope / asset[kind], path,
                                                  selections, read_glb, accessor)
                    doc, binary = read_glb(path)
                    if patches:
                        doc, binary = attach(doc, binary, pngs, patches, offset_m=.0005)
                        binary, batch = batch_paint(doc, binary)
                        extra['paint']['batching'] = batch
                        for i, png in enumerate(pngs):
                            (stage / f'paint-atlas-{i}.png').write_bytes(png)
                        atlas = {'schema_version': 1, 'files': {asset['visual']: {
                            'patches': patches, 'pages': [{'file': f'paint-atlas-{i}.png',
                            'sha256': hashlib.sha256(png).hexdigest()} for i, png in enumerate(pngs)]}},
                            'layer': 'rmuc2027.decoration', 'collision': False,
                            'structural_backing': 'original cap geometry retained with reviewed backing color'}
                        dump(stage / 'texture-atlas.json', atlas)
                        manifest['texture_atlas'] = {'sidecar': 'texture-atlas.json',
                            'sha256': sha(stage / 'texture-atlas.json'), 'embedded': True,
                            'render_sidecar_required': False, 'offset_m': .0005}
                    bytes_before_packing = len(binary)
                    binary, packing = pack_exact(doc, binary, preserve=selections)
                    packing.update(binary_bytes_before=bytes_before_packing, binary_bytes_after=len(binary))
                    write_glb(path, doc, binary)
                    if selections:
                        optimize.verify_preserved(source / scope / asset[kind], path,
                                                  selections, read_glb, accessor)
                    after = scene_triangles(doc)
                    asset[kind + '_sha256'] = sha(path)
                    asset['triangles' if kind == 'visual' else 'collision_triangles'] = after
                    result[kind] = {'input_sha256': sha(source / scope / asset[kind]),
                                    'output_sha256': sha(path), 'triangles_before': before,
                                    'triangles_after': after, 'simplification': simplification,
                                    'exact_packing': packing, **extra}
                    print(f'{scope}/{name}/{kind}: {before:,} -> {after:,}', flush=True)
                asset['mesh_simplification'] = {
                    kind: {k: v for k, v in result[kind]['simplification'].items() if k != 'primitives'}
                    for kind in ('visual', 'collision')}
                asset['collision_method'] = result['collision']['simplification']['method']
                report['assets'][f'{scope}/{name}'] = result
            dump(mpath, manifest)
        # Custom 2027 authority remains byte-identical; scenery paint nodes do
        # not alter source-body names, their transforms or motion hierarchies.
        binding = report['input_manifests']['']['rmuc2027_bindings']
        if sha(stage / binding['file']) != binding['sha256']:
            raise ValueError('2027 authority binding changed')
        minimap = json.loads((stage / 'minimap.json').read_text())
        minimap['geometry_reference'] = 'source-painted input checkpoint; shared frame and unchanged floor'
        minimap['source_manifests_sha256'] = recipe['input_manifests']
        minimap['manifest_sha256'] = {str(p.relative_to(stage)): sha(p) for p in
                                      [stage / 'manifest.json', stage / 'equipment/manifest.json']}
        dump(stage / 'minimap.json', minimap)
        report['after'] = optimize.placed_counts(stage, read_glb, scene_triangles)
        report['budget_passed'] = all(report['after'][k] < recipe['budgets'][k] for k in ('visual', 'collision'))
        dump(stage / 'source-detail-build.json', report)
        stage.rename(output)
    print(json.dumps({'output': str(output), 'counts': report['after']}, indent=2))
    if not report['budget_passed']:
        print('Candidate retained for review; triangle budget failed. Do not deploy.', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
