# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Conservative visibility classifications on small synthetic closed surfaces."""
import importlib.util
from pathlib import Path
import unittest

AVAILABLE = all(importlib.util.find_spec(n) for n in ('vtk', 'numpy'))


@unittest.skipUnless(AVAILABLE, 'requires the rm-map-tools CAD Python environment')
class VisibilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location(
            'structure_visibility', Path(__file__).resolve().parents[1]/'analyze-structure-visibility.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def geometry(self):
        import numpy as np
        p = np.array([[i & 1, (i >> 1) & 1, (i >> 2) & 1] for i in range(8)], float)
        t = np.array([[0, 2, 3], [0, 3, 1], [4, 5, 7], [4, 7, 6],
                      [0, 1, 5], [0, 5, 4], [2, 6, 7], [2, 7, 3],
                      [0, 4, 6], [0, 6, 2], [1, 3, 7], [1, 7, 5]])
        inner = np.array([[[.4, .4, .5], [.6, .4, .5], [.5, .6, .5]]])
        return np.concatenate([p[t], inner])

    def test_closed_fixed_shell_hides_an_internal_triangle(self):
        import numpy as np
        import contextlib
        import io
        with contextlib.redirect_stdout(io.StringIO()):
            _, visible, rest, _ = self.module.audit(
                self.geometry(), np.array(['shell']*12 + ['inner']), set(), 8, 16, 1e-5)
        self.assertFalse(visible[-1])
        self.assertFalse(rest[-1])
        self.assertTrue(visible[:12].all())

    def test_moving_shell_cannot_authorize_removal_of_fixed_detail(self):
        import numpy as np
        import contextlib
        import io
        with contextlib.redirect_stdout(io.StringIO()):
            _, visible, rest, _ = self.module.audit(
                self.geometry(), np.array(['shell']*12 + ['inner']), {'shell'}, 8, 16, 1e-5)
        self.assertTrue(visible[-1])
        self.assertFalse(rest[-1])
        self.assertTrue(visible[:12].all())

    def test_visible_triangle_edge_overrides_hidden_centroid(self):
        import contextlib
        import io
        import numpy as np
        from unittest.mock import patch

        class PartialOccluder:
            def __init__(self, triangles):
                pass

            def clear(self, point, direction, reach_m, epsilon_m):
                return point[0] > .75

        triangle = np.array([[[0., 0, 0], [1, 0, 0], [0, 1, 0]]])
        with patch.object(self.module, 'RayMesh', PartialOccluder), \
                contextlib.redirect_stdout(io.StringIO()):
            _, visible, rest, coarse_hidden = self.module.audit(
                triangle, np.array(['panel']), set(), 8, 16, 1e-5)
        self.assertTrue(coarse_hidden[0])
        self.assertTrue(visible[0])
        self.assertTrue(rest[0])

    def test_sphere_directions_are_unit_vectors(self):
        import numpy as np
        np.testing.assert_allclose(np.linalg.norm(self.module.sphere_directions(128), axis=1), 1.)


if __name__ == '__main__':
    unittest.main()
