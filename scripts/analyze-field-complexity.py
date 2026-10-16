#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Read-only placed mesh/component inventory and optional simplification probes.

Run in rm-map-tools/ocpenv. Small components and coincident collision faces are
review candidates, not permission to remove geometry. Probes never write a GLB;
reported meshoptimizer error and sampled surface deviation are distinct metrics.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys


THRESHOLDS_M = (.01, .02, .05, .1)


def summarize_components(parts, copies):
    """Count nested size thresholds per primitive; material seams remain separate.

    These are connected material fragments, not necessarily independent solids.
    Copies multiply triangle counts, not the number of stored components.
    """
    return {str(size): {
        'components_per_copy': sum(c['max_extent_m'] <= size for c in parts),
        'placed_triangles': copies * sum(c['triangles'] for c in parts
                                         if c['max_extent_m'] <= size),
    } for size in THRESHOLDS_M}


def coincident_faces(points, triangles):
    """Count same-position triangle repetitions irrespective of winding.

    This ignores shading, materials and contact behavior. The result is an
    inventory, not a certified safe removal budget.
    """
    import numpy as np
    _, inverse = np.unique(points, axis=0, return_inverse=True)
    faces = np.sort(inverse[triangles], axis=1)
    return len(faces) - len(np.unique(faces, axis=0))


def probe(points, triangles, error_mm, samples):
    """Measure component-retaining simplification without modifying source data."""
    from rmuc2027_geometry import components
    from simplify_package import simplify_mesh
    from mesh_quality import deviation
    after = 0
    maximum = estimated = 0.
    for p, t in components(points, triangles):
        if len(t) <= 24:
            after += len(t)
            continue
        q, u, error = simplify_mesh(p, t, error_mm / 1000, False)
        after += len(u)
        estimated = max(estimated, error * 1000)
        if len(u) < len(t):
            maximum = max(maximum, deviation(p, t, q, u, count=samples)['max'] * 1000)
    return {'error_limit_mm': error_mm, 'triangles_before': len(triangles),
            'triangles_after': after, 'estimated_error_mm': estimated,
            'max_sampled_deviation_mm': maximum,
            'samples_per_component_per_direction': samples,
            'reference': 'input runtime mesh, not source STEP',
            'optics_normals_and_silhouette_reviewed': False}


def analyze(root, selections, errors, samples):
    """Inventory manifest-selected GLBs with placement multiplicity and checksums."""
    import numpy as np
    from gltf_scene import read_glb, mesh_instances
    from rmuc2027_geometry import components
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    report = {'schema_version': 1, 'manifests': {}, 'assets': [], 'probes': [],
              'policy': 'read-only; candidate categories overlap and are not removal budgets',
              'component_scope': 'exact-position connected fragments within each material primitive',
              'script_sha256': sha(Path(__file__))}
    found = set()
    for scope in ('', 'equipment'):
        manifest = root / scope / 'manifest.json'
        report['manifests'][scope] = sha(manifest)
        for name, entry in json.loads(manifest.read_text())['assets'].items():
            copies = max(1, len(entry.get('placements_in_source_arena_frame', [])))
            for kind in ('visual', 'collision'):
                path = root / scope / entry[kind]
                if not path.resolve().is_relative_to(root.resolve()):
                    raise ValueError('asset path escapes package')
                if sha(path) != entry[kind + '_sha256']:
                    raise ValueError(f'asset checksum mismatch: {path}')
                doc, binary = read_glb(path)
                asset = {'asset': name, 'kind': kind, 'file': str(path.relative_to(root)),
                         'sha256': sha(path), 'copies': copies, 'primitives': []}
                for ni, pi, p, t, mi in mesh_instances(doc, binary):
                    node = doc['nodes'][ni]['name']
                    parts = []
                    for cp, ct in components(p, t):
                        span = np.ptp(cp, axis=0)
                        parts.append({'triangles': len(ct), 'span_m': span.tolist(),
                                      'max_extent_m': float(max(span))})
                    primitive = {'node': node, 'primitive': pi, 'triangles_per_copy': len(t),
                                 'placed_triangles': len(t) * copies,
                                 'material': doc['materials'][mi].get('name') if mi is not None else None,
                                 'component_size_thresholds_m': summarize_components(parts, copies),
                                 'components': sorted(parts, key=lambda c: -c['triangles'])}
                    if kind == 'collision':
                        primitive['coincident_face_candidates_per_copy'] = coincident_faces(p, t)
                    asset['primitives'].append(primitive)
                    key = (name, kind, node, str(pi))
                    if key in selections:
                        found.add(key)
                        for error in errors:
                            result = dict(asset=name, kind=kind, node=node, primitive=pi,
                                          copies=copies, **probe(p, t, error, samples))
                            report['probes'].append(result)
                            print(json.dumps(result), flush=True)
                asset['placed_triangles'] = sum(p['placed_triangles'] for p in asset['primitives'])
                report['assets'].append(asset)
                print(f'{name}/{kind}: {asset["placed_triangles"]:,} placed triangles', flush=True)
    if found != selections:
        raise ValueError(f'unmatched probe selectors: {sorted(selections - found)}')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cad-assets', type=Path, required=True)
    parser.add_argument('--map-tools', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--probe', nargs=4, action='append', default=[],
                        metavar=('ASSET', 'KIND', 'NODE', 'PRIMITIVE'))
    parser.add_argument('--error-mm', nargs='+', type=float, default=[.1, .25, .5])
    parser.add_argument('--samples', type=int, default=2048)
    args = parser.parse_args()
    import math
    if any(not math.isfinite(e) or e <= 0 for e in args.error_mm) or args.samples < 96:
        parser.error('positive finite errors and at least 96 samples required')
    if args.output.exists() or args.output.resolve().is_relative_to(args.cad_assets.resolve()):
        parser.error('output must be new and outside the input package')
    sys.path.insert(0, str(args.map_tools.resolve() / 'python'))
    report = analyze(args.cad_assets.resolve(), {tuple(p) for p in args.probe},
                     args.error_mm, args.samples)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
