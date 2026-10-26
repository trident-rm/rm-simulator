#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Stage an explicit primitive-collision trial; never replace an existing package."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('source', type=Path)
    ap.add_argument('--bundle', required=True, type=Path)
    ap.add_argument('--out', required=True, type=Path)
    ap.add_argument('--asset', default='tech-core')
    args = ap.parse_args()
    source, bundle, out = args.source.resolve(), args.bundle.resolve(), args.out.resolve()
    if out.exists() or out == source or source in out.parents:
        ap.error('destination must be new and outside the source package')
    recipe = json.loads((bundle/'collision-shapes.json').read_text())
    if recipe.get('schema') != 'rm-map-tools.collision-shapes.prototype-1':
        ap.error('unsupported collision shape schema')
    manifests = [Path('manifest.json'), Path('equipment/manifest.json')]
    found = []
    for path in manifests:
        data = json.loads((source/path).read_text())
        if args.asset in data['assets']:
            found.append((path, data))
    if len(found) != 1:
        ap.error('asset must resolve uniquely in the package manifests')
    manifest, data = found[0]
    entry = data['assets'][args.asset]
    original = source/manifest.parent/entry['collision']
    if digest(original) != entry['collision_sha256'] or recipe['source']['sha256'] != entry['collision_sha256']:
        ap.error('bundle source hash does not match the package collision GLB')
    remainder = Path(recipe['retained_mesh']['file'])
    if remainder.name != str(remainder) or remainder.is_absolute() or digest(bundle/remainder) != recipe['retained_mesh']['sha256']:
        ap.error('invalid retained mesh reference/hash')
    shutil.copytree(source, out)
    relative = Path('primitive-collision')/args.asset
    target = out/manifest.parent/relative
    target.mkdir(parents=True)
    for file in ['collision-shapes.json', remainder.name]:
        shutil.copyfile(bundle/file, target/file)
    entry['collision_method'] = 'primitive-shapes-v1'
    entry['collision_shapes'] = (relative/'collision-shapes.json').as_posix()
    entry['collision_shapes_sha256'] = digest(target/'collision-shapes.json')
    (out/manifest).write_text(json.dumps(data, indent=2)+'\n')
    (out/'primitive-collision-trial.json').write_text(json.dumps({
        'scope':'Explicit trial package; source package untouched. Geometry contact approval is separate from loader support.',
        'asset':args.asset,'source_manifest_sha256':digest(source/manifest),
        'source_collision_sha256':entry['collision_sha256'],
        'collision_shapes_sha256':entry['collision_shapes_sha256'],
        'native_parts':len(recipe['native_parts']),
        'replaced_node_indices':recipe['replaced_node_indices'],
    },indent=2)+'\n')
    print(out)


if __name__ == '__main__':
    main()
