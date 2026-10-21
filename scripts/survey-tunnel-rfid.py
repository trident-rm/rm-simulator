#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Find 250 mm marking rectangles near the CAD tunnels, without CAD libraries.

Reads the installed source-derived GLB hierarchy, welds equal primitive vertices,
and reports connected components with the expected rectangular footprint. These
are geometric candidates, not a claim about radio detection or native card IDs.
Run: python3 scripts/survey-tunnel-rfid.py [field-2027]
"""
import itertools
import json
from pathlib import Path
import struct
import sys

IDENTITY = [float(i == j) for j in range(4) for i in range(4)]


def multiply(a, b):
    return [sum(a[k*4+r]*b[c*4+k] for k in range(4)) for c in range(4) for r in range(4)]


def transform(m, p):
    return [sum(m[k*4+r]*p[k] for k in range(3))+m[12+r] for r in range(3)]


def candidates(path, floor):
    raw = path.read_bytes()
    size = struct.unpack_from('<I', raw, 12)[0]
    gltf = json.loads(raw[20:20+size])
    data = raw[28+size:]

    def accessor(index):
        a = gltf['accessors'][index]
        view = gltf['bufferViews'][a['bufferView']]
        component = {5121: 'B', 5123: 'H', 5125: 'I', 5126: 'f'}[a['componentType']]
        width = {'SCALAR': 1, 'VEC3': 3}[a['type']]
        fmt = '<'+component*width
        stride = view.get('byteStride', struct.calcsize(fmt))
        start = view.get('byteOffset', 0)+a.get('byteOffset', 0)
        return [struct.unpack_from(fmt, data, start+i*stride) for i in range(a['count'])]

    def visit(index, parent):
        node = gltf['nodes'][index]
        # The approved field package records every occurrence as a full matrix.
        if any(k in node for k in ['translation', 'rotation', 'scale']):
            raise ValueError('expected matrix-based source hierarchy')
        world = multiply(parent, node.get('matrix', IDENTITY))
        if 'mesh' in node:
            mesh = gltf['meshes'][node['mesh']]
            for primitive in mesh['primitives']:
                vertices = accessor(primitive['attributes']['POSITION'])
                indices = [v[0] for v in accessor(primitive['indices'])]
                welded = {}
                vertex_ids = [welded.setdefault(tuple(round(v, 6) for v in p), len(welded)) for p in vertices]
                parents = list(range(len(welded)))

                def find(i):
                    while parents[i] != i:
                        parents[i] = parents[parents[i]]
                        i = parents[i]
                    return i

                for i in range(0, len(indices), 3):
                    a, b, c = [find(vertex_ids[q]) for q in indices[i:i+3]]
                    parents[b] = a
                    parents[c] = a
                groups = {}
                for i, p in enumerate(vertices):
                    groups.setdefault(find(vertex_ids[i]), []).append(transform(world, p))
                for points in groups.values():
                    lo = [min(p[i] for p in points) for i in range(3)]
                    hi = [max(p[i] for p in points) for i in range(3)]
                    lo[1] -= 1.6243436765
                    hi[1] -= 1.6243436765
                    lo[2] -= floor
                    hi[2] -= floor
                    dx, dy, dz = [b-a for a, b in zip(lo, hi)]
                    if (.24 < dx < .26 and .22 < dy < .25 and dz < .10
                            and -2 < lo[0] < 6 and 4 < lo[1] < 7.5):
                        yield {'asset': path.name, 'node': node['name'],
                               'bounds_m': [lo, hi],
                               'center_m': [(a+b)/2 for a, b in zip(lo, hi)]}
        for child in node.get('children', []):
            yield from visit(child, world)

    for node in gltf['scenes'][gltf.get('scene', 0)]['nodes']:
        yield from visit(node, IDENTITY)


def main():
    directory = Path(sys.argv[1] if len(sys.argv) > 1 else 'field-2027')
    manifest = json.loads((directory/'manifest.json').read_text())
    floor = manifest['floor_top_source_z_m']
    result = list(itertools.chain.from_iterable(candidates(directory/file, floor)
                  for file in ['floor.glb', 'arena-static.glb']))
    print(json.dumps({'source_sha256': manifest['source_sha256'], 'candidates': result}, indent=2))


if __name__ == '__main__':
    main()
