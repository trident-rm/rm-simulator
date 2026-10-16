#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Read-only sampled visibility audit of a bound RMUC 2027 base.

Run with rm-map-tools/ocpenv/bin/python. No mesh is removed or rewritten.
A hidden result is a review candidate, never proof that a face is internal or
inaccessible to a projectile. The floor and other field equipment are omitted.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time


def sphere_directions(count):
    """Deterministic Fibonacci directions, supplemented by triangle normals."""
    import numpy as np
    i = np.arange(count)
    z = 1 - 2 * (i + .5) / count
    a = i * np.pi * (3 - np.sqrt(5))
    r = np.sqrt(1 - z*z)
    return np.column_stack([r*np.cos(a), r*np.sin(a), z])


class RayMesh:
    """Opaque two-sided triangle occluder with a reusable VTK locator."""
    def __init__(self, triangles):
        import numpy as np
        import vtk
        from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray
        p = np.ascontiguousarray(triangles.reshape(-1, 3), dtype=float)
        pts = vtk.vtkPoints()
        pts.SetData(numpy_to_vtk(p, deep=True))
        t = np.arange(len(p)).reshape(-1, 3)
        cells = vtk.vtkCellArray()
        cells.ImportLegacyFormat(numpy_to_vtkIdTypeArray(
            np.column_stack([np.full(len(t), 3), t]).astype(np.int64).ravel(), deep=True))
        poly = vtk.vtkPolyData()
        poly.SetPoints(pts)
        poly.SetPolys(cells)
        self.locator = vtk.vtkStaticCellLocator()
        self.locator.SetDataSet(poly)
        self.locator.BuildLocator()
        self.t = vtk.reference(0.)
        self.x, self.pc = [0.]*3, [0.]*3
        self.sub, self.cell_id = vtk.reference(0), vtk.reference(0)
        self.cell = vtk.vtkGenericCell()
        self.rays = 0

    def clear(self, point, direction, reach_m, epsilon_m):
        """Whether an offset surface sample has an unobstructed ray to outside."""
        self.rays += 1
        return not self.locator.IntersectWithLine(
            point + direction*epsilon_m, point + direction*reach_m, 1e-9,
            self.t, self.x, self.pc, self.sub, self.cell_id, self.cell)


def audit(faces, names, moving, directions, refined_directions, epsilon_m):
    """Report assembled and motion-conservative sampled visibility per triangle."""
    import numpy as np
    fixed = np.array([name not in moving for name in names])
    whole = RayMesh(faces)
    fixed_mesh = RayMesh(faces[fixed])
    reach_m = float(np.linalg.norm(np.ptp(faces.reshape(-1, 3), axis=0))*3 + 1)
    centers = faces.mean(1)
    normals = np.cross(faces[:, 1]-faces[:, 0], faces[:, 2]-faces[:, 0])
    area2 = np.linalg.norm(normals, axis=1)
    normals /= np.maximum(area2[:, None], 1e-30)
    coarse = sphere_directions(directions)
    refined = sphere_directions(refined_directions)
    exposed = np.zeros(len(faces), dtype=bool)
    exposed_rest = np.zeros(len(faces), dtype=bool)
    coarse_hidden = np.zeros(len(faces), dtype=bool)
    rows = []
    start = time.monotonic()
    for name in dict.fromkeys(names):
        indices = np.flatnonzero(names == name)
        conservative = fixed_mesh if name not in moving else RayMesh(faces[indices])
        for i in indices:
            if area2[i] < 1e-20:
                # Degenerate surfaces are not evidence of enclosed geometry.
                exposed[i] = exposed_rest[i] = True
                continue
            trials = np.vstack([normals[i], -normals[i], coarse])
            winner = None
            for direction in trials:
                if conservative.clear(centers[i], direction, reach_m, epsilon_m):
                    winner = (centers[i], direction)
                    break
            coarse_hidden[i] = winner is None
            if winner is None:
                points = [centers[i], *(.8*faces[i] + .1*np.roll(faces[i], 1, axis=0)
                                       + .1*np.roll(faces[i], 2, axis=0))]
                for point in points:
                    for direction in np.vstack([normals[i], -normals[i], refined]):
                        if conservative.clear(point, direction, reach_m, epsilon_m):
                            winner = (point, direction)
                            break
                    if winner is not None:
                        break
            if winner is None:
                continue
            exposed[i] = True
            if whole.clear(*winner, reach_m, epsilon_m):
                exposed_rest[i] = True
            else:
                for point in [centers[i], *(.8*faces[i] + .1*np.roll(faces[i], 1, axis=0)
                                           + .1*np.roll(faces[i], 2, axis=0))]:
                    if any(whole.clear(point, direction, reach_m, epsilon_m)
                           for direction in np.vstack([normals[i], -normals[i], coarse, refined])):
                        exposed_rest[i] = True
                        break
        hidden = ~exposed[indices]
        row = {'node': str(name), 'triangles': len(indices),
               'moving': name in moving,
               'coarse_hidden_triangles': int(coarse_hidden[indices].sum()),
               'sampled_hidden_triangles': int(hidden.sum()),
               'sampled_hidden_at_rest_triangles': int((~exposed_rest[indices]).sum()),
               'surface_area_m2': float(area2[indices].sum()/2),
               'sampled_hidden_area_m2': float(area2[indices][hidden].sum()/2)}
        rows.append(row)
        print(json.dumps({**row, 'elapsed_s': round(time.monotonic()-start, 1)}), flush=True)
    return rows, exposed, exposed_rest, coarse_hidden


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--cad-assets', type=Path, required=True)
    ap.add_argument('--map-tools', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--kind', choices=['visual', 'collision', 'both'], default='both')
    ap.add_argument('--directions', type=int, default=32)
    ap.add_argument('--refine-directions', type=int, default=128)
    ap.add_argument('--epsilon-m', type=float, default=.00001)
    args = ap.parse_args()
    if min(args.directions, args.refine_directions) < 1 or not 0 < args.epsilon_m <= .0001:
        ap.error('positive direction counts and epsilon in (0, 0.1 mm] required')
    if args.output.exists() or args.output.resolve().is_relative_to(args.cad_assets.resolve()):
        ap.error('output must be new and outside the runtime package')
    import numpy as np
    sys.path.insert(0, str(args.map_tools.resolve()/'python'))
    from gltf_scene import read_glb, mesh_instances
    root = args.cad_assets.resolve()
    manifest = json.loads((root/'equipment/manifest.json').read_text())
    binding_descriptor = json.loads((root/'manifest.json').read_text())['rmuc2027_bindings']
    binding_path = root/binding_descriptor['file']
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    if digest(binding_path) != binding_descriptor['sha256']:
        raise ValueError('binding checksum mismatch')
    binding = json.loads(binding_path.read_text())
    moving = {name for joint in binding['base_motion'] for name in joint['bodies']}
    args.output.mkdir(parents=True)
    report = {'schema_version': 1, 'package': str(root), 'settings': {
        'coarse_directions': args.directions, 'refined_directions': args.refine_directions,
        'refined_barycentric_samples': 4, 'epsilon_m': args.epsilon_m,
        'two_sided': True, 'floor_and_other_equipment': False,
        'moving_occluders': 'removed for fixed bodies; self-only for moving bodies'},
        'script_sha256': digest(Path(__file__)), 'assets': {},
        'interpretation': 'sampled hidden candidates, not a certified internal/removal classification'}
    for kind in ['visual', 'collision'] if args.kind == 'both' else [args.kind]:
        entry = manifest['assets']['base']
        path = root/'equipment'/entry[kind]
        if digest(path) != entry[kind+'_sha256']:
            raise ValueError('asset checksum mismatch')
        doc, binary = read_glb(path)
        if any(m.get('alphaMode', 'OPAQUE') != 'OPAQUE' for m in doc.get('materials', [])):
            raise ValueError('transparent materials require a different visibility model')
        chunks, ids = [], []
        for ni, _, p, t, _ in mesh_instances(doc, binary):
            chunks.append(p[t])
            ids.extend([doc['nodes'][ni]['name']]*len(t))
        faces, names = np.concatenate(chunks), np.asarray(ids)
        rows, exposed, rest, coarse = audit(faces, names, moving, args.directions,
                                            args.refine_directions, args.epsilon_m)
        np.savez_compressed(args.output/f'{kind}-triangles.npz', faces=faces, names=names,
                            exposed=exposed, exposed_rest=rest, coarse_hidden=coarse)
        report['assets'][kind] = {'file': entry[kind], 'sha256': digest(path),
            'triangles': len(faces), 'coarse_hidden_triangles': int(coarse.sum()),
            'sampled_hidden_triangles': int((~exposed).sum()),
            'sampled_hidden_at_rest_triangles': int((~rest).sum()), 'bodies': rows}
        (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
