#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Build a reproducible render/collision derivative using rm-map-tools.

Run with rm-map-tools/ocpenv/bin/python. The input is an immutable checkpoint;
error measurements refer to that checkpoint, never to the original STEP. Upstream
simplification records are retained, so a second pass cannot masquerade as a
first pass. Outputs are new external directories, never an in-place deployment.
"""
import argparse
import copy
import hashlib
import importlib.metadata
import json
from pathlib import Path
import shutil
import sys
import subprocess
import tempfile


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def placed_counts(root, read_glb, scene_triangles):
    rows = {}
    for scope in ('', 'equipment'):
        manifest = json.loads((root / scope / 'manifest.json').read_text())
        for name, entry in manifest['assets'].items():
            copies = max(1, len(entry.get('placements_in_source_arena_frame', [])))
            counts = {}
            for kind in ('visual', 'collision'):
                doc, _ = read_glb(root / scope / entry[kind])
                counts[kind] = scene_triangles(doc) * copies
            rows[f'{scope}/{name}'] = {**counts, 'placements': copies}
    return {'assets': rows, **{k: sum(row[k] for row in rows.values()) for k in ('visual', 'collision')}}


def rmuc2027_preservation(root, read_glb, protect_moving=True):
    """Protect every moving or detector body, validating the pinned source binding.

    The custom 2027 sidecar is not an articulation descriptor. Keep it byte-for-
    byte, and retain every primitive of its moving and detector bodies instead
    of relying on the 2026 role metadata (which these source meshes lack).
    """
    manifest = json.loads((root / 'manifest.json').read_text())
    descriptor = manifest.get('rmuc2027_bindings')
    if not descriptor:
        if manifest.get('season') == '2027':
            raise ValueError('2027 optimization requires a bound source package')
        return {}
    path = (root / descriptor['file']).resolve()
    if not path.is_relative_to(root.resolve()) or digest(path) != descriptor['sha256']:
        raise ValueError('2027 binding checksum/path mismatch')
    binding = json.loads(path.read_text())
    if (binding.get('schema_version') != 1 or binding.get('ruleset') != 'rmuc2027-v1.0.0'
            or binding.get('source_sha256') != manifest.get('source_sha256')
            or binding.get('source_sha256') != 'a5ad3ede3b7674fc98f2e73b71717216383cb2b4b5dcdbe900c2038918bea8e0'):
        raise ValueError('unknown 2027 source binding')
    result = {}
    for name, scope in [('outpost', ''), ('base', 'equipment')]:
        m = json.loads((root / scope / 'manifest.json').read_text())
        report = binding['report'][name]
        motion = binding[name + '_motion']
        moving = {body for joint in motion for body in joint['bodies']}
        if moving != {f'source_body_{body}' for body in report['moving_bodies']}:
            raise ValueError('2027 motion/report body mismatch')
        protected_names = (moving if protect_moving else set()) | {
            f'source_body_{body}' for body in report['detector_bodies']}
        expected = {f'source_body_{body}' for body in report['bodies']}
        if not protected_names or not protected_names <= expected:
            raise ValueError('2027 protected body coverage mismatch')
        for kind in ('visual', 'collision'):
            doc, _ = read_glb(root / scope / m['assets'][name][kind])
            nodes = [n for n in doc['nodes'] if 'mesh' in n]
            names = [n.get('name') for n in nodes]
            if len(names) != len(set(names)) or set(names) != expected:
                raise ValueError('2027 source body coverage mismatch')
            result[(scope, name, kind)] = [
                {'node': n['name'], 'primitive': pi}
                for n in nodes if n['name'] in protected_names
                for pi in range(len(doc['meshes'][n['mesh']]['primitives']))]
    return result


def verify_preserved(source, output, selectors, read_glb, accessor):
    """Require exact retained geometry/attributes and identical scene/material data."""
    import numpy as np
    old, old_binary = read_glb(source)
    new, new_binary = read_glb(output)
    if old.get('materials') != new.get('materials'):
        raise ValueError('source materials changed')
    stripped = lambda nodes: [{k: v for k, v in n.items() if k != 'mesh'} for n in nodes]
    if stripped(old['nodes']) != stripped(new['nodes']):
        raise ValueError('source hierarchy, transforms or joint metadata changed')
    for selector in selectors:
        primitives = []
        for doc in (old, new):
            node = next(n for n in doc['nodes'] if n.get('name') == selector['node'])
            primitives.append(doc['meshes'][node['mesh']]['primitives'][selector['primitive']])
        a, b = primitives
        if ({k: v for k, v in a.items() if k not in {'attributes', 'indices'}}
                != {k: v for k, v in b.items() if k not in {'attributes', 'indices'}}
                or set(a['attributes']) != set(b['attributes'])):
            raise ValueError('protected primitive metadata changed')
        pairs = [(a['attributes'][key], b['attributes'][key]) for key in a['attributes']]
        if ('indices' in a) != ('indices' in b):
            raise ValueError('protected primitive indexing changed')
        if 'indices' in a:
            pairs.append((a['indices'], b['indices']))
        if any(not np.array_equal(accessor(old, old_binary, ai), accessor(new, new_binary, bi))
               for ai, bi in pairs):
            raise ValueError('protected moving/detector geometry changed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-tools', type=Path, required=True)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--settings', type=Path, default=Path(__file__).with_name('field-detail-standard.json'))
    args = parser.parse_args()
    source, output, producer = args.input.resolve(), args.output.resolve(), args.map_tools.resolve()
    if output.exists() or output.is_relative_to(source):
        parser.error('output must be new and outside the input checkpoint')
    sys.path.insert(0, str(producer / 'python'))
    from simplify_package import simplify_glb, safe_path, scene_triangles, protected
    from gltf_scene import read_glb, accessor
    if importlib.metadata.version('meshoptimizer') != '0.2.30a0':
        parser.error('requires meshoptimizer==0.2.30a0')
    settings = json.loads(args.settings.read_text())
    preservation = rmuc2027_preservation(source, read_glb)
    output.parent.mkdir(parents=True, exist_ok=True)
    provenance = {'schema_version': 1, 'source': str(source), 'settings': settings,
                  'script_sha256': digest(Path(__file__)),
                  'producer_files': {p.name: digest(p) for p in sorted((producer / 'python').glob('*.py'))},
                  'dependencies': {p: importlib.metadata.version(p) for p in ('meshoptimizer', 'numpy', 'scipy', 'vtk', 'Pillow')},
                  'protection_policy': ('retain all 2027 moving/detector bodies, materials and joint metadata'
                                        if preservation else 'simplify mechanical dart carriage; retain primitive optics, artwork and joint metadata'),
                  'measurement_reference': 'input checkpoint, not STEP; upstream errors are not erased',
                  'input_manifests': {}, 'assets': {}}
    with tempfile.TemporaryDirectory(prefix='field-detail-', dir=output.parent) as temporary:
        stage = Path(temporary) / 'package'
        shutil.copytree(source, stage)
        provenance['before'] = placed_counts(stage, read_glb, scene_triangles)
        processed = {}
        for scope in ('', 'equipment'):
            path = stage / scope / 'manifest.json'
            manifest = json.loads(path.read_text())
            provenance['input_manifests'][scope] = {'sha256': digest(path), 'manifest': copy.deepcopy(manifest)}
            descriptor = manifest.get('articulation')
            sidecar_path = safe_path(path.parent, descriptor['file']) if descriptor else None
            sidecar = None
            if sidecar_path:
                if digest(sidecar_path) != descriptor['sha256']:
                    raise ValueError('articulation checksum mismatch')
                sidecar = json.loads(sidecar_path.read_text())
            for name, entry in manifest['assets'].items():
                config = {**settings['defaults'], **settings.get('assets', {}).get(name, {})}
                records = {}
                upstream = copy.deepcopy(entry.get('mesh_simplification'))
                for kind in ('visual', 'collision'):
                    mesh = safe_path(path.parent, entry[kind])
                    before_hash = digest(safe_path(source / scope, entry[kind]))
                    if before_hash != entry[kind + '_sha256']:
                        raise ValueError(f'{name}/{kind} checksum mismatch')
                    if not config.get('enabled', True):
                        continue
                    binding = sidecar['assets'].get(name, {}).get('files', {}).get(kind) if sidecar else None
                    if binding and (binding['sha256'] != before_hash or binding['file'] != entry[kind]):
                        raise ValueError('articulation file mismatch')
                    selectors = [*config.get(kind + '_preserve', ()),
                                 *preservation.get((scope, name, kind), ())]
                    arguments = (config[kind + '_error_mm'], config[kind + '_sampled_limit_mm'],
                                 config['deviation_samples'], selectors,
                                 config.get(kind + '_lock_borders', False))
                    if mesh in processed:
                        previous_arguments, record = processed[mesh]
                        if previous_arguments != arguments:
                            raise ValueError('shared mesh has conflicting simplification settings')
                        record = copy.deepcopy(record)
                    else:
                        record = simplify_glb(mesh, arguments[0], binding, arguments[1],
                                              arguments[2], arguments[4], arguments[3],
                                              protection_policy=lambda metadata: protected(metadata) and not
                                              (metadata.get('id') == 'base.dart_target.carriage'))
                        processed[mesh] = (arguments, copy.deepcopy(record))
                    if preservation:
                        verify_preserved(safe_path(source / scope, entry[kind]), mesh,
                                         selectors, read_glb, accessor)
                    record['input_sha256'] = before_hash
                    record['output_sha256'] = digest(mesh)
                    records[kind] = record
                    entry[kind + '_sha256'] = record['output_sha256']
                    entry['triangles' if kind == 'visual' else 'collision_triangles'] = record['after_triangles']
                    if binding:
                        binding['sha256'] = record['output_sha256']
                    print(f'{name}/{kind}: {record["before_triangles"]:,} -> {record["after_triangles"]:,}', flush=True)
                if records:
                    entry['mesh_simplification'] = {kind: {k: v for k, v in record.items() if k != 'primitives'} for kind, record in records.items()}
                    entry['collision_method'] = records['collision']['method']
                    provenance['assets'][f'{scope}/{name}'] = {'upstream': upstream, 'pass': records}
            if sidecar:
                write_json(sidecar_path, sidecar)
                descriptor['sha256'] = digest(sidecar_path)
            manifest.pop('reference_equipment', None)
            write_json(path, manifest)
        minimap = Path(__file__).with_name('generate-minimap.py')
        subprocess.run([sys.executable, str(minimap), '--cad-assets', str(stage),
                        '--map-tools', str(producer)], check=True)
        provenance['minimap_generator_sha256'] = digest(minimap)
        provenance['after'] = placed_counts(stage, read_glb, scene_triangles)
        provenance['budget_passed'] = all(provenance['after'][k] < settings['budgets'][k] for k in ('visual', 'collision'))
        write_json(stage / 'field-detail-build.json', provenance)
        stage.rename(output)
    print(json.dumps({'output': str(output), 'counts': provenance['after'], 'budget_passed': provenance['budget_passed']}, indent=2))
    if not provenance['budget_passed']:
        print('Candidate retained for inspection; budget failed. Do not deploy.', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
