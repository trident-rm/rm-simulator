# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Grate prism probes must preserve voids and reject triangles spanning holes."""
import importlib.util
from pathlib import Path
import unittest


@unittest.skipUnless(importlib.util.find_spec('numpy'), 'requires CAD Python environment')
class GratePrismTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location(
            'grate_prisms', Path(__file__).resolve().parents[1] / 'probe-grate-prisms.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def test_empty_hole_with_solid_boundary_corners_stays_empty(self):
        import numpy as np
        axes = [np.arange(4.), np.arange(4.), np.arange(2.)]
        occupied = self.module.contained_cells(
            axes, lambda p: not (1 < p[0] < 2 and 1 < p[1] < 2))
        self.assertFalse(occupied[1, 1, 0])
        self.assertEqual(int(occupied.sum()), 8)
        reconstructed = np.zeros_like(occupied)
        for box in self.module.cover_cells(occupied):
            reconstructed[tuple(slice(a, b) for a, b in zip(box[:3], box[3:]))] = True
        np.testing.assert_array_equal(reconstructed, occupied)

    def test_triangles_with_vertices_on_bars_cannot_bridge_the_opening(self):
        import numpy as np
        boxes = np.array([[[0, 0, 0], [3, 1, .2]], [[0, 0, 0], [1, 3, .2]]])
        p = np.array([[0, 0, .2], [3, 1, .2], [1, 3, .2], [3, 0, .2]])
        keep, deviation = self.module.covered_triangles(p, np.array([[0, 1, 2], [0, 3, 1]]), boxes)
        np.testing.assert_array_equal(keep, [False, True])
        self.assertAlmostEqual(deviation, 0)

    def test_empty_cell_grid_has_no_prisms(self):
        import numpy as np
        self.assertEqual(self.module.cover_cells(np.zeros((2, 2, 2), dtype=bool)), [])


if __name__ == '__main__':
    unittest.main()
