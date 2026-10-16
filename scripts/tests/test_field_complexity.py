# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Check inventory units and duplicate-face classification on synthetic meshes."""
import importlib.util
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location(
    'field_complexity', Path(__file__).resolve().parents[1] / 'analyze-field-complexity.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ComponentInventoryTests(unittest.TestCase):
    def test_thresholds_include_boundary_and_multiply_only_placed_triangles(self):
        report = MODULE.summarize_components([
            {'triangles': 12, 'max_extent_m': .01},
            {'triangles': 80, 'max_extent_m': .06},
            {'triangles': 120, 'max_extent_m': 1.},
        ], 2)
        self.assertEqual(report['0.01'], {'components_per_copy': 1, 'placed_triangles': 24})
        self.assertEqual(report['0.1'], {'components_per_copy': 2, 'placed_triangles': 184})

    @unittest.skipUnless(importlib.util.find_spec('numpy'), 'requires CAD Python environment')
    def test_coincident_faces_weld_positions_and_include_opposite_winding(self):
        import numpy as np
        points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0],
                           [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, .001]])
        triangles = np.array([[0, 1, 2], [3, 4, 5], [5, 4, 3], [0, 1, 6]])
        # Two repeated faces, but the nearby non-coplanar face is distinct.
        self.assertEqual(MODULE.coincident_faces(points, triangles), 2)
        self.assertEqual(MODULE.coincident_faces(points, triangles[[0, 3]]), 0)


if __name__ == '__main__':
    unittest.main()
