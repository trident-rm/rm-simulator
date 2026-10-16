# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Regression tests for exact subsets and conservative cleanup boundaries."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

PRODUCER = Path(__file__).resolve().parents[3]/'rm-map-tools/python'
AVAILABLE = (PRODUCER/'gltf_scene.py').is_file() and all(
    importlib.util.find_spec(n) for n in ('numpy', 'scipy', 'vtk'))


@unittest.skipUnless(AVAILABLE, 'requires the rm-map-tools CAD Python environment')
class InteriorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(PRODUCER))
        spec = importlib.util.spec_from_file_location(
            'rmuc2027_interior', Path(__file__).resolve().parents[1]/'rmuc2027_interior.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def test_guard_ring_preserves_border_without_discarding_interior_candidates(self):
        import numpy as np
        p = np.array([[i, i % 2, 0] for i in range(12)], float)
        faces = p[np.array([[0, 1, 2], [2, 3, 4], [4, 5, 6], [9, 10, 11]])]
        remove = np.array([False, True, True, True])
        result = self.module.guard_ring(faces, np.array(['a']*4), remove)
        np.testing.assert_array_equal(result, [False, False, True, True])

    def test_duplicate_cleanup_keeps_independently_moving_bodies(self):
        import numpy as np
        face = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0]])
        result = self.module.duplicate_faces(np.array([face, face[::-1], face]),
                                             np.array(['fixed', 'fixed', 'moving']))
        np.testing.assert_array_equal(result, [False, True, False])

    def test_cavity_requires_matching_component_and_triangle_count(self):
        import numpy as np
        face = np.array([[[0., 0, 0], [1, 0, 0], [0, 1, 0]]])
        selection = {'node': 'cavity', 'positions_sha256': self.module.signature(face), 'triangles': 1}
        np.testing.assert_array_equal(self.module.pinned_cavities(face, np.array(['cavity']),
                                                                 [selection]), [True])
        selection['triangles'] = 2
        with self.assertRaisesRegex(ValueError, 'changed or missing'):
            self.module.pinned_cavities(face, np.array(['cavity']), [selection])

    def test_subset_preserves_expanded_positions_normals_material_and_node_metadata(self):
        import numpy as np
        from gltf_scene import read_glb, accessor
        from export_field_package import GlbWriter
        p = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
        t = np.array([[0, 1, 2], [0, 3, 1]])
        writer = GlbWriter('root', {'version': '2.0'})
        writer.add_node('part', p, t, ['0.2,0.2,0.2']*2)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'part.glb'
            writer.write(path)
            before, bb = read_glb(path)
            record = self.module.filter_glb(path, np.array([False, True]))
            after, ab = read_glb(path)
        self.assertEqual(record['removed_triangles'], 1)
        self.assertEqual(before['nodes'], after['nodes'])
        self.assertEqual(before['materials'], after['materials'])
        a, b = before['meshes'][0]['primitives'][0], after['meshes'][0]['primitives'][0]
        ia = accessor(before, bb, a['indices']).reshape(-1, 3)[0]
        ib = accessor(after, ab, b['indices']).reshape(-1, 3)[0]
        for key in a['attributes']:
            np.testing.assert_array_equal(accessor(before, bb, a['attributes'][key])[ia],
                                          accessor(after, ab, b['attributes'][key])[ib])

    def test_empty_primitive_removal_is_refused_to_preserve_semantic_slots(self):
        import numpy as np
        from export_field_package import GlbWriter
        p = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0]])
        writer = GlbWriter('root', {'version': '2.0'})
        writer.add_node('part', p, np.array([[0, 1, 2]]), [None])
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'part.glb'
            writer.write(path)
            result = self.module.filter_glb(path, np.array([True]))
        self.assertEqual(result['removed_triangles'], 0)


if __name__ == '__main__':
    unittest.main()
