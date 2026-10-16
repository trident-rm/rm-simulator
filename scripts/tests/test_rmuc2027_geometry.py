# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Geometry regressions; run with rm-map-tools/ocpenv/bin/python for CAD helpers."""
import importlib.util
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest

PRODUCER = Path(__file__).resolve().parents[3] / 'rm-map-tools/python'
AVAILABLE = (PRODUCER / 'gltf_scene.py').is_file() and all(
    importlib.util.find_spec(name) for name in ('numpy', 'scipy', 'vtk', 'meshoptimizer'))


@unittest.skipUnless(AVAILABLE, 'requires the rm-map-tools Python environment')
class GeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(PRODUCER))
        spec = importlib.util.spec_from_file_location(
            'rmuc2027_geometry', Path(__file__).resolve().parents[1] / 'rmuc2027_geometry.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def cube(self, open_face=False):
        import numpy as np
        p = np.array([[i & 1, (i >> 1) & 1, (i >> 2) & 1] for i in range(8)], float)
        p *= [.02, .015, .01]
        t = np.array([[0, 2, 3], [0, 3, 1], [4, 5, 7], [4, 7, 6],
                      [0, 1, 5], [0, 5, 4], [2, 6, 7], [2, 7, 3],
                      [0, 4, 6], [0, 6, 2], [1, 3, 7], [1, 7, 5]])
        if open_face:
            t = t[:-2]
        faces = []
        for a, b, c in p[t]:
            ab, bc, ca = (a+b)/2, (b+c)/2, (c+a)/2
            faces.extend([[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]])
        flat = np.asarray(faces).reshape(-1, 3)
        return flat, np.arange(len(flat)).reshape(-1, 3)

    def test_box_fit_reduces_closed_small_component(self):
        p, t = self.cube()
        q, u, maximum = self.module.fitted_box(p, t)
        self.assertEqual(len(u), 12)
        self.assertLess(maximum, 1e-8)

    def test_box_fit_refuses_to_fill_an_opening(self):
        p, t = self.cube(open_face=True)
        self.assertIsNone(self.module.fitted_box(p, t))

    def test_large_housing_is_never_box_fitted(self):
        p, t = self.cube()
        self.assertIsNone(self.module.fitted_box(p * 20, t))

    def test_paint_tiling_preserves_area_winding_and_hole(self):
        import numpy as np
        from mesh_quality import deviation
        # Two long strips with a full gap between them must stay separate.
        p = np.array([[0, 0, 0], [5, 0, 0], [5, .1, 0], [0, .1, 0],
                      [0, .3, 0], [5, .3, 0], [5, .4, 0], [0, .4, 0]])
        t = np.array([[0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7]])
        tiles = list(self.module.paint_tiles(p, t))
        self.assertGreater(len(tiles), 1)
        faces = np.concatenate([q[u] for q, u in tiles])
        normals = np.cross(faces[:, 1]-faces[:, 0], faces[:, 2]-faces[:, 0])
        self.assertTrue((normals[:, 2] > 0).all())
        self.assertAlmostEqual(float(normals[:, 2].sum()/2), 1.)
        self.assertTrue(((faces[:, :, 1] <= .1).all(1) | (faces[:, :, 1] >= .3).all(1)).all())
        self.assertLess(deviation(p, t, faces.reshape(-1, 3),
                                  np.arange(faces.size//3).reshape(-1, 3), count=2048)['max'], 1e-8)

    def test_exact_packing_preserves_expanded_normals_uv_and_normalized_colors(self):
        import numpy as np
        from gltf_scene import accessor
        from simplify_package import append_accessor
        doc = {'nodes': [{'mesh': 0}], 'scenes': [{'nodes': [0]}],
               'meshes': [{'primitives': []}], 'bufferViews': [], 'accessors': []}
        binary = bytearray()
        # Shared position with different normals/UVs must not be merged.
        p = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]] * 2, dtype='<f4')
        normals = np.array([[0, 0, 1]] * 3 + [[0, 0, -1]] * 3, dtype='<f4')
        uv = np.array([[0, 0], [1, 0], [0, 1]] * 2, dtype='<f4')
        color = np.tile(np.array([255, 0, 0, 255], dtype='u1'), (6, 1))
        arrays = {'POSITION': p, 'NORMAL': normals, 'TEXCOORD_0': uv, 'COLOR_0': color}
        attrs = {k: append_accessor(doc, binary, a, 'VEC'+str(a.shape[1]),
                                   5121 if a.dtype == np.uint8 else 5126) for k, a in arrays.items()}
        doc['accessors'][attrs['COLOR_0']]['normalized'] = True
        doc['meshes'][0]['primitives'] = [{'attributes': attrs}]
        packed, report = self.module.pack_exact(doc, bytes(binary))
        pr = doc['meshes'][0]['primitives'][0]
        indices = accessor(doc, packed, pr['indices'])
        for key, expected in arrays.items():
            np.testing.assert_array_equal(accessor(doc, packed, pr['attributes'][key])[indices], expected)
        self.assertTrue(doc['accessors'][pr['attributes']['COLOR_0']]['normalized'])
        self.assertTrue(report['expanded_attributes_exact'])

    def test_component_simplification_keeps_a_separate_small_surface(self):
        import numpy as np
        from export_field_package import GlbWriter
        from gltf_scene import read_glb, mesh_instances
        p, t = self.cube()
        p = np.vstack([p, [[.1, 0, 0], [.101, 0, 0], [.1, .001, 0]]])
        t = np.vstack([t, [[len(p)-3, len(p)-2, len(p)-1]]])
        writer = GlbWriter('root', {'version': '2.0'})
        writer.add_node('parts', p, t, [None]*len(t))
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'parts.glb'
            writer.write(path)
            record = self.module.simplify_components(path, 2, 3)
            doc, binary = read_glb(path)
            _, _, q, u, _ = next(mesh_instances(doc, binary))
            self.assertEqual(len(list(self.module.components(q, u))), 2)
            self.assertEqual(record['component_retention'], 'all')
            self.assertFalse(record['open_borders_locked'])
            self.assertLessEqual(record['estimated_error_mm'], 2)

    def test_precision_component_mismatch_fails_closed(self):
        import numpy as np
        from export_field_package import GlbWriter
        p, t = self.cube()
        writer = GlbWriter('root', {'version': '2.0'})
        writer.add_node('precision', p, t, [None]*len(t))
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'parts.glb'
            writer.write(path)
            with self.assertRaisesRegex(ValueError, 'precision component missing'):
                self.module.simplify_components(path, 2, 3, component_preserve=[
                    {'node': 'precision', 'positions_sha256': 'changed'}])

    def test_precision_component_retains_all_source_triangles(self):
        import numpy as np
        from export_field_package import GlbWriter
        from gltf_scene import read_glb, mesh_instances
        p, t = self.cube()
        writer = GlbWriter('root', {'version': '2.0'})
        writer.add_node('precision', p, t, [None]*len(t))
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'parts.glb'
            writer.write(path)
            doc, binary = read_glb(path)
            _, _, q, u, _ = next(mesh_instances(doc, binary))
            points, _ = next(self.module.components(q, u))
            signature = hashlib.sha256(np.asarray(points, dtype='<f4').tobytes()).hexdigest()
            report = self.module.simplify_components(path, 2, 3, component_preserve=[
                {'node': 'precision', 'positions_sha256': signature}])
            self.assertEqual(report['before_triangles'], report['after_triangles'])
            self.assertEqual(report['primitives'][0]['components_retained_by_recipe'], 1)

    def test_fixed_paint_batches_without_changing_expanded_vertices(self):
        import numpy as np
        from export_field_package import GlbWriter
        from gltf_scene import read_glb, mesh_instances
        writer = GlbWriter('root', {'version': '2.0'})
        p = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0]])
        for i in range(2):
            writer.add_node(str(i), p+[i*2, 0, 0], np.array([[0, 1, 2]]), [None])
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'paint.glb'
            writer.write(path)
            doc, binary = read_glb(path)
            for mesh in doc['meshes']:
                for pr in mesh['primitives']:
                    pr['extras'] = {'rm': {'layer': 'markings'}}
            packed, result = self.module.batch_paint(doc, binary)
            faces = np.concatenate([q[u] for _, _, q, u, _ in mesh_instances(doc, packed)])
            np.testing.assert_array_equal(faces, [p, p+[2, 0, 0]])
            self.assertEqual(result, {'patches': 2, 'batched_primitives': 1})

    def test_paint_batching_refuses_a_transformed_anchor(self):
        import numpy as np
        from export_field_package import GlbWriter
        from gltf_scene import read_glb
        writer = GlbWriter('root', {'version': '2.0'})
        transform = np.eye(4)
        transform[0, 3] = 1.
        writer.add_node('moving', np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0]]),
                        np.array([[0, 1, 2]]), [None], matrix=transform)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'paint.glb'
            writer.write(path)
            doc, binary = read_glb(path)
            doc['meshes'][0]['primitives'][0]['extras'] = {'rm': {'layer': 'markings'}}
            with self.assertRaisesRegex(ValueError, 'cannot flatten'):
                self.module.batch_paint(doc, binary)

    def test_multiple_paint_colors_keep_distinct_anchors_and_solid_backing(self):
        import numpy as np
        from export_field_package import GlbWriter
        from gltf_scene import read_glb
        from simplify_package import scene_triangles
        p = np.array([[0., 0, 0], [.1, 0, 0], [0, .1, 0]])
        writer = GlbWriter('root', {'version': '2.0'})
        writer.add_node('painted-deck', np.vstack([p, p+[.2, 0, 0], p+[.4, 0, 0]]),
                        np.arange(9).reshape(-1, 3), ['1,1,1', '1,0,0', '0.2,0.2,0.2'])
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'paint.glb'
            writer.write(path)
            doc, binary = read_glb(path)
            materials = {tuple(m['extras']['step_colour_srgb']): m['name'] for m in doc['materials']}
            selections = [{'node': 'painted-deck', 'material': materials[color],
                           'backing_material': materials[(.2, .2, .2)]}
                          for color in [(1, 1, 1), (1, 0, 0)]]
            doc, binary, pngs, patches, records = self.module.paint_atlas(doc, binary, selections)
            self.assertEqual(len(records), 2)
            self.assertEqual(len(patches), 2)
            self.assertTrue(pngs)
            self.assertEqual(scene_triangles(doc), 3, 'original structural caps must not become holes')
            names = [n.get('name') for n in doc['nodes']]
            self.assertEqual(len(names), len(set(names)))

    def test_sparse_lines_retain_geometry_instead_of_allocating_an_atlas(self):
        import numpy as np
        from export_field_package import GlbWriter
        from gltf_scene import read_glb, mesh_instances
        writer = GlbWriter('root', {'version': '2.0'})
        p = np.array([[0., 0, 0], [.1, 0, 0], [0, .1, 0]])
        writer.add_node('deck', np.vstack([p, p+[.2, 0, 0]]),
                        np.arange(6).reshape(-1, 3), ['1,0,0', '0.2,0.2,0.2'])
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'lines.glb'
            writer.write(path)
            doc, binary = read_glb(path)
            mats = {tuple(m['extras']['step_colour_srgb']): m['name'] for m in doc['materials']}
            doc, binary, pngs, patches, records = self.module.paint_atlas(doc, binary, [{
                'node': 'deck', 'material': mats[(1, 0, 0)],
                'backing_material': mats[(.2, .2, .2)], 'encoding': 'geometry'}])
            self.assertEqual((pngs, patches), ([], []))
            self.assertEqual(records[0]['encoding'], 'geometry')
            painted = [q[u] for ni, _, q, u, _ in mesh_instances(doc, binary)
                       if '/paint_geometry_' in doc['nodes'][ni]['name']]
            np.testing.assert_allclose(painted, [[p+[0, 0, .0005]]], atol=1e-8)


if __name__ == '__main__':
    unittest.main()
