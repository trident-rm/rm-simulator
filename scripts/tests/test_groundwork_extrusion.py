# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Discrete profile-error guarantees, including closed and invalid input."""
import importlib.util
from pathlib import Path
import unittest


@unittest.skipUnless(importlib.util.find_spec('numpy'), 'requires CAD Python environment')
class ProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location(
            'groundwork_probe', Path(__file__).resolve().parents[1] / 'probe-groundwork-extrusion.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def test_straight_profile_keeps_only_endpoints(self):
        self.assertEqual(self.module.profile_indices([[0, 0, 0], [1, 2, 3], [2, 4, 6]], .01), [0, 2])

    def test_closed_profile_does_not_collapse_and_respects_discrete_error(self):
        import numpy as np
        theta = np.linspace(0, 2 * np.pi, 501)
        points = np.column_stack([np.cos(theta), np.sin(theta), np.zeros(len(theta))])
        points[-1] = points[0]
        keep = self.module.profile_indices(points, .01)
        self.assertEqual((keep[0], keep[-1]), (0, 500))
        self.assertGreater(len(keep), 8)
        for a, b in zip(keep, keep[1:]):
            delta = points[b] - points[a]
            samples = points[a:b+1]
            fraction = np.clip((samples-points[a]) @ delta / (delta @ delta), 0, 1)
            distances = np.linalg.norm(samples-points[a]-fraction[:, None]*delta, axis=1)
            self.assertLessEqual(float(distances.max()), .01)

    def test_uniform_grid_keeps_peaks_has_upward_faces_and_respects_error(self):
        import numpy as np
        budget = self.module.uniform_sine_budget(70, 2084.5, 10, .25, 500)
        vertices, triangles = self.module.uniform_sine_surface_mm(70, 240.64, 2084.5, 10, .25, 500)
        self.assertEqual(budget['segments_per_period'], 28)
        self.assertEqual(budget['strips_across_width'], 5)
        self.assertEqual(len(triangles), 2800)
        self.assertAlmostEqual(float(vertices[:, 2].max()), 70)
        faces = vertices[triangles]
        self.assertTrue((np.cross(faces[:, 1]-faces[:, 0], faces[:, 2]-faces[:, 0])[:, 2] > 0).all())
        profile = vertices[::6]
        x = np.linspace(0, 2406.4, 100001)
        actual = 35*(1-np.cos(2*np.pi*x/240.64))
        observed = np.abs(np.interp(x, profile[:, 0], profile[:, 2])-actual).max()
        self.assertLessEqual(observed, budget['analytic_interpolation_bound_mm'])
        self.assertLessEqual(budget['analytic_interpolation_bound_mm'], .25)

    def test_fit_distinguishes_sine_from_semicylinders(self):
        import numpy as np
        x = np.linspace(0, 2400, 5001)
        h = 35*(1-np.cos(2*np.pi*x/240))
        report = self.module.fit_periodic_profile(np.column_stack([x, np.zeros(len(x)), h]), [0, 1, 0], 10)
        self.assertLess(report['sinusoidal']['max_sampled_residual_mm'], 1e-6)
        self.assertGreater(report['semicircular']['max_sampled_residual_mm'], 20)
        self.assertAlmostEqual(report['sinusoidal']['height_mm'], 70, places=6)

    def test_rejects_nonfinite_and_nonpositive_units(self):
        for points, tolerance in [([[0, 0, 0]], 1),
                                  ([[0, 0, 0], [1, 1, float('nan')]], 1),
                                  ([[0, 0, 0], [1, 1, 1]], 0)]:
            with self.assertRaises(ValueError):
                self.module.profile_indices(points, tolerance)


if __name__ == '__main__':
    unittest.main()
