# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Foot plinths follow the floor and refuse larger passages or missing ground."""
import importlib.util
from pathlib import Path
import unittest


@unittest.skipUnless(importlib.util.find_spec('numpy'), 'requires CAD Python environment')
class PlinthTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location(
            'skirts', Path(__file__).resolve().parents[1] / 'rmuc2027_skirts.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def surfaces(self, clearance):
        import numpy as np
        floor = np.array([[-1, -1, -.01], [1, -1, -.01], [1, 3, .03], [-1, 3, .03]])
        wall = np.array([[0, 0, clearance], [0, 2, clearance+.02], [0, 2, .32], [0, 0, .3]])
        t = np.array([[0, 1, 2], [0, 2, 3]])
        return wall[t], floor[t]

    def test_six_centimetre_gap_is_one_closed_floor_following_prism(self):
        import numpy as np
        wall, floor = self.surfaces(.06)
        vertices, triangles, records = self.module.foot_plinths(wall, floor, [1, 0, 0], 0)
        self.assertEqual((len(vertices), len(triangles), len(records)), (8, 12, 1))
        np.testing.assert_allclose(records[0]['gap_height_m'], [.06, .06], atol=1e-8)
        edge = np.sort(np.concatenate([triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]]), axis=1)
        self.assertTrue((np.unique(edge, axis=0, return_counts=True)[1] == 2).all())
        faces = vertices[triangles]
        volume = np.einsum('ij,ij->i', faces[:, 0], np.cross(faces[:, 1], faces[:, 2])).sum()/6
        self.assertGreater(volume, 0)
        # The two floor corners track the 1% grade, with only the tiny overlap.
        self.assertAlmostEqual(float(vertices[2, 2]-.01*vertices[2, 1]), -1e-5)
        self.assertAlmostEqual(float(vertices[3, 2]-.01*vertices[3, 1]), -1e-5)

    def test_larger_passage_is_not_closed(self):
        wall, floor = self.surfaces(.2)
        vertices, _, records = self.module.foot_plinths(wall, floor, [1, 0, 0], 0)
        self.assertEqual(len(vertices), 0)
        self.assertEqual(records, [])

    def test_missing_floor_is_not_replaced_by_an_assumed_plane(self):
        _, floor = self.surfaces(.06)
        with self.assertRaises(ValueError):
            self.module.floor_height(floor, 10, 10)


if __name__ == '__main__':
    unittest.main()
