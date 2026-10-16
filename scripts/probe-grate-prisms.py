#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Read-only planar grate decomposition into overlapping rectangular prisms.

Source vertex coordinates define a cell grid in a selected planar face's frame.
Both corners and centers must be inside/on the source solid. Only the requested
interior region participates; the frame stays source geometry. This produces a
review budget, not a certified replacement or a runtime mesh.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import sys


def cover_cells(occupied):
    """Cover true grid cells with overlapping boxes, without covering false cells.

    Returns integer lower-inclusive/upper-exclusive bounds. Six growth orders
    produce prism candidates; greedy selection covers the most remaining cells.
    """
    import numpy as np
    candidates = set()
    for seed in np.argwhere(occupied):
        for order in itertools.permutations(range(3)):
            lo, hi = seed.copy(), seed+1
            for axis in order:
                while lo[axis] > 0:
                    test = lo.copy()
                    test[axis] -= 1
                    if not occupied[tuple(slice(a, b) for a, b in zip(test, hi))].all():
                        break
                    lo = test
                while hi[axis] < occupied.shape[axis]:
                    test = hi.copy()
                    test[axis] += 1
                    if not occupied[tuple(slice(a, b) for a, b in zip(lo, test))].all():
                        break
                    hi = test
            candidates.add(tuple(int(x) for x in lo)+tuple(int(x) for x in hi))
    remaining, boxes = occupied.copy(), []
    while remaining.any():
        _, box = max((int(remaining[tuple(slice(a, b) for a, b in zip(box[:3], box[3:]))].sum()), box)
                     for box in candidates)
        boxes.append(box)
        candidates.remove(box)
        remaining[tuple(slice(a, b) for a, b in zip(box[:3], box[3:]))] = False
    return boxes


def contained_cells(axes, inside):
    """Require all eight corners AND the center to pass an injected solid query.

    Hole corners can all lie on material boundaries while the center is empty.
    Even these nine samples do not certify arbitrary curved/concave cell shapes.
    """
    import numpy as np
    vertices = np.zeros(tuple(len(a) for a in axes), dtype=bool)
    for index in np.ndindex(vertices.shape):
        vertices[index] = inside([axes[a][i] for a, i in enumerate(index)])
    shape = tuple(n-1 for n in vertices.shape)
    occupied = np.ones(shape, dtype=bool)
    for shift in itertools.product([0, 1], repeat=3):
        occupied &= vertices[tuple(slice(s, s+n) for s, n in zip(shift, shape))]
    for index in np.argwhere(occupied):
        occupied[tuple(index)] = inside([(axes[a][i]+axes[a][i+1])/2 for a, i in enumerate(index)])
    return occupied


def covered_triangles(points_mm, triangles, boxes_mm, tolerance_mm=.01):
    """Flag triangles whose vertices, edge midpoints and centroid are in box union.

    Sampled containment is not an exact triangle/union proof. Runtime adoption
    still requires continuous coverage, normal/material and contact validation.
    """
    import numpy as np
    f = points_mm[triangles]
    samples = np.concatenate([f, f.mean(1)[:, None, :], (f+np.roll(f, 1, axis=1))/2], axis=1)
    selected = np.zeros(len(f), dtype=bool)
    maximum = 0.
    # Bound memory independently of field/primitive size.
    for start in range(0, len(f), 1024):
        sample = samples[start:start+1024].reshape(-1, 3)
        delta = np.maximum(np.maximum(boxes_mm[:, 0, None, :]-sample[None, :, :],
                                     sample[None, :, :]-boxes_mm[:, 1, None, :]), 0)
        distances = np.linalg.norm(delta, axis=2).min(0).reshape(-1, 7).max(1)
        keep = distances <= tolerance_mm
        selected[start:start+len(keep)] = keep
        if keep.any():
            maximum = max(maximum, float(distances[keep].max()))
    return selected, maximum


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-part', type=Path, required=True)
    parser.add_argument('--source-sha256', required=True)
    parser.add_argument('--face-index', type=int, default=2)
    parser.add_argument('--region-mm', nargs=4, type=float, required=True, metavar=('U0', 'U1', 'V0', 'V1'))
    parser.add_argument('--map-tools', type=Path, required=True)
    parser.add_argument('--cad-assets', type=Path, required=True)
    parser.add_argument('--runtime-node', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import numpy as np
    if args.output.exists() or args.output.resolve().is_relative_to(args.cad_assets.resolve()):
        parser.error('output must be new and outside the runtime package')
    if not np.isfinite(args.region_mm).all() or not (args.region_mm[0] < args.region_mm[1] and args.region_mm[2] < args.region_mm[3]):
        parser.error('finite increasing U/V region bounds in millimetres required')
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    if sha(args.source_part) != args.source_sha256:
        parser.error('source-part checksum mismatch')
    sys.path.insert(0, str(args.map_tools.resolve() / 'python'))
    from validate_parts import read_part
    from audit_geometry import compound, shape_map
    from gltf_scene import read_glb, mesh_instances
    from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_VERTEX, TopAbs_IN, TopAbs_ON
    from OCP.TopoDS import TopoDS
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Plane
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.gp import gp_Pnt
    doc, _, error = read_part(str(args.source_part.resolve()))
    if error:
        raise ValueError(error)
    shape = compound(doc)
    solids, faces = shape_map(shape, TopAbs_SOLID), shape_map(shape, TopAbs_FACE)
    if solids.Extent() != 1 or not 1 <= args.face_index <= faces.Extent():
        raise ValueError('one source solid and a valid face index required')
    if any(BRepAdaptor_Surface(TopoDS.Face(faces.FindKey(i))).GetType() != GeomAbs_Plane
           for i in range(1, faces.Extent()+1)):
        raise ValueError('probe requires an all-planar source solid')
    plane = BRepAdaptor_Surface(TopoDS.Face(faces.FindKey(args.face_index))).Plane()
    origin = np.array(plane.Location().Coord())
    basis = np.array([plane.Position().XDirection().Coord(), plane.Position().YDirection().Coord(),
                      plane.Axis().Direction().Coord()]).T
    vertices = shape_map(shape, TopAbs_VERTEX)
    source = (np.array([BRep_Tool.Pnt_s(TopoDS.Vertex(vertices.FindKey(i))).Coord()
                       for i in range(1, vertices.Extent()+1)])-origin) @ basis
    axes = []
    for axis in range(3):
        keys = np.round(source[:, axis], 3)
        axes.append(np.array([np.median(source[keys == key, axis]) for key in np.unique(keys)]))
    classifier = BRepClass3d_SolidClassifier(TopoDS.Solid(solids.FindKey(1)))

    def inside(local):
        classifier.Perform(gp_Pnt(*(origin+basis @ local)), 1e-5)
        return classifier.State() in (TopAbs_IN, TopAbs_ON)

    occupied = contained_cells(axes, inside)
    u0, u1, v0, v1 = args.region_mm
    occupied &= (axes[0][:-1, None, None] >= u0) & (axes[0][1:, None, None] <= u1)
    occupied &= (axes[1][None, :-1, None] >= v0) & (axes[1][None, 1:, None] <= v1)
    boxes = cover_cells(occupied)
    if not boxes:
        raise ValueError('region contains no fully sampled solid cells')
    bounds = np.array([[[axes[a][box[a]] for a in range(3)],
                        [axes[a][box[a+3]] for a in range(3)]] for box in boxes])
    report = {'schema_version': 1, 'source_part_sha256': args.source_sha256,
              'face_index': args.face_index, 'region_mm': args.region_mm,
              'origin_mm': origin.tolist(), 'basis': basis.tolist(),
              'occupied_cells': int(occupied.sum()), 'prisms': len(boxes),
              'prism_triangles': 12*len(boxes), 'bounds_local_mm': bounds.tolist(),
              'policy': 'sampled source-cell and runtime-face containment; not an approved deletion mask',
              'source_cell_samples': 'eight corners plus center', 'runtime': []}
    root = args.cad_assets.resolve()
    manifest = json.loads((root/'manifest.json').read_text())
    asset = manifest['assets']['arena-static']
    for kind in ('visual', 'collision'):
        path = root / asset[kind]
        if not path.resolve().is_relative_to(root) or sha(path) != asset[kind+'_sha256']:
            raise ValueError('runtime path/checksum mismatch')
        d, binary = read_glb(path)
        matches = [(p, t) for ni, _, p, t, _ in mesh_instances(d, binary)
                   if d['nodes'][ni].get('name') == args.runtime_node]
        if len(matches) != 1:
            raise ValueError('runtime selector must identify one single-material mesh')
        p, t = matches[0]
        local = (p*1000-origin) @ basis
        # Existing runtime occurrences have a source-frame translation. This
        # exploratory alignment is reported, not substituted for source bindings.
        offset = ((local.min(0)+local.max(0))-(source.min(0)+source.max(0)))/2
        local -= offset
        selected, deviation = covered_triangles(local, t, bounds)
        report['runtime'].append({'kind': kind, 'file_sha256': sha(path),
                                  'node': args.runtime_node, 'translation_fit_mm': offset.tolist(),
                                  'triangles_before': len(t), 'sampled_covered_triangles': int(selected.sum()),
                                  'estimated_triangles_after': len(t)-int(selected.sum())+12*len(boxes),
                                  'max_covered_sample_distance_mm': deviation})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'bounds_local_mm'}, indent=2))


if __name__ == '__main__':
    main()
