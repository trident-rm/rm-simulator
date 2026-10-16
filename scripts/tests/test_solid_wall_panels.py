# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Opaque side panels retain chamfered silhouettes and form closed solids."""
import importlib.util
from pathlib import Path
import unittest


@unittest.skipUnless(all(importlib.util.find_spec(n) for n in ('numpy', 'scipy')), 'requires CAD Python environment')
class SolidPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location(
            'solid_walls', Path(__file__).resolve().parents[1] / 'build-rmuc2027-solid-walls.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def test_chamfered_panel_is_closed_outward_wound_and_opaque(self):
        import numpy as np
        polygon = np.array([[0, 0], [1, 0], [1, .8], [.8, 1], [.2, 1], [0, .8]])
        points = np.vstack([np.column_stack([np.full(6, x), polygon]) for x in [0, .015]])
        vertices, triangles, report = self.module.solid_panel(points, [1, 0, 0])
        self.assertEqual(report['outline_vertices'], 6)
        self.assertEqual(len(triangles), 20)
        self.assertAlmostEqual(report['thickness_m'], .015)
        edges = np.sort(np.concatenate([triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]]), axis=1)
        _, counts = np.unique(edges, axis=0, return_counts=True)
        self.assertTrue((counts == 2).all())
        f = vertices[triangles]
        volume = np.einsum('ij,ij->i', f[:, 0], np.cross(f[:, 1], f[:, 2])).sum()/6
        self.assertAlmostEqual(float(volume), .96*.015)
        # Both full caps occupy the center of the former opening.
        cap_area = np.linalg.norm(np.cross(f[:, 1]-f[:, 0], f[:, 2]-f[:, 0]), axis=1)/2
        caps = np.ptp(f[:, :, 0], axis=1) < 1e-12
        self.assertAlmostEqual(float(cap_area[caps].sum()), 2*.96)

    def test_flush_skin_uses_backing_plane_and_explicit_tiny_outward_offset(self):
        import numpy as np
        points = np.array([[x, y, z] for x in [0, .015] for y in [0, 1] for z in [0, 1]])
        vertices, _, report = self.module.solid_panel(points, [1, 0, 0], thickness_m=.003, outward_offset_m=.00005)
        self.assertAlmostEqual(float(vertices[:, 0].max()), .01495)
        self.assertAlmostEqual(float(vertices[:, 0].min()), .01195)
        self.assertAlmostEqual(report['source_frame_thickness_m'], .015)

    def test_horizontal_or_thick_panel_is_refused(self):
        import numpy as np
        points = np.array([[x, y, z] for x in [0, .1] for y in [0, 1] for z in [0, 1]])
        for normal in ([1, 0, 0], [0, 0, 1]):
            with self.assertRaises(ValueError):
                self.module.solid_panel(points, normal)


if __name__ == '__main__':
    unittest.main()
