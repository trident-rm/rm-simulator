# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Budget accounting must include every placement, including shared GLBs."""
import importlib.util
import copy
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    'optimize_field', Path(__file__).resolve().parents[1] / 'optimize-field.py'
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class BudgetTests(unittest.TestCase):
    def test_shared_meshes_count_per_placement_and_empty_placement_is_one(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'equipment').mkdir()
            def entry(copies):
                return {'visual': 'shared.glb', 'collision': 'shared-collision.glb',
                        'placements_in_source_arena_frame': [{}] * copies}
            (root / 'manifest.json').write_text(json.dumps({'assets': {'floor': entry(0), 'a': entry(2), 'b': entry(2)}}))
            (root / 'equipment/manifest.json').write_text(json.dumps({'assets': {'base': entry(2)}}))
            result = module.placed_counts(root, lambda path: (path.name, None),
                                          lambda name: 10 if name == 'shared.glb' else 3)
            self.assertEqual(result['visual'], 70)
            self.assertEqual(result['collision'], 21)
            self.assertEqual(result['assets']['/floor']['placements'], 1)


class Rmuc2027ProtectionTests(unittest.TestCase):
    def fixture(self, root):
        binding = {'schema_version': 1, 'ruleset': 'rmuc2027-v1.0.0',
                   'source_sha256': 'a5ad3ede3b7674fc98f2e73b71717216383cb2b4b5dcdbe900c2038918bea8e0',
                   'report': {}}
        for name in ('base', 'outpost'):
            binding['report'][name] = {'bodies': [1, 2, 3], 'moving_bodies': [1],
                                       'detector_bodies': [2]}
            binding[name + '_motion'] = [{'bodies': ['source_body_1']}]
        path = root / 'bindings.json'
        path.write_text(json.dumps(binding))
        manifest = {'season': '2027', 'source_sha256': binding['source_sha256'],
                    'rmuc2027_bindings': {'file': path.name, 'sha256': module.digest(path)},
                    'assets': {name: {'visual': name + '.glb', 'collision': name + '-collision.glb'}
                               for name in ('base', 'outpost')}}
        (root / 'equipment').mkdir()
        for prefix in ('', 'equipment'):
            (root / prefix / 'manifest.json').write_text(json.dumps(manifest))
        doc = {'nodes': [{'name': f'source_body_{i + 1}', 'mesh': i} for i in range(3)],
               'meshes': [{'primitives': [{}, {}]} for _ in range(3)]}
        return doc

    def test_moving_and_detector_bodies_protected_but_fixed_body_can_reduce(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            doc = self.fixture(root)
            result = module.rmuc2027_preservation(root, lambda path: (doc, b''))
            self.assertEqual(len(result), 4)
            for selectors in result.values():
                self.assertEqual(selectors, [{'node': f'source_body_{i}', 'primitive': pi}
                                             for i in (1, 2) for pi in (0, 1)])

    def test_missing_source_body_or_duplicate_name_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            doc = self.fixture(root)
            for nodes in [doc['nodes'][:-1], [*doc['nodes'], doc['nodes'][0]]]:
                with self.assertRaisesRegex(ValueError, 'body coverage'):
                    module.rmuc2027_preservation(root, lambda path: ({**doc, 'nodes': nodes}, b''))

    def test_binding_tampering_fails_before_geometry_read(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.fixture(root)
            (root / 'bindings.json').write_text('{}')
            with self.assertRaisesRegex(ValueError, 'checksum/path'):
                module.rmuc2027_preservation(root, lambda path: self.fail('must not read geometry'))

    def test_unbound_2027_cannot_use_2026_protection_policy(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'manifest.json').write_text('{"season":"2027"}')
            with self.assertRaisesRegex(ValueError, 'requires a bound'):
                module.rmuc2027_preservation(root, lambda path: None)

    @unittest.skipUnless(importlib.util.find_spec('numpy'), 'run with rm-map-tools Python for exact-geometry check')
    def test_retained_detector_vertex_mutation_fails_even_when_metadata_matches(self):
        old = {'nodes': [{'name': 'detector', 'mesh': 0}],
               'meshes': [{'primitives': [{'attributes': {'POSITION': 0}, 'indices': 1}]}]}
        new = copy.deepcopy(old)
        selector = [{'node': 'detector', 'primitive': 0}]
        def reader(path):
            return (old, False) if path == 'source' else (new, True)
        def values(doc, changed, index):
            return [0, 1, 2] if index == 1 else [[0, 0, 0], [1, 0, 0], [0, 1, int(changed)]]
        with self.assertRaisesRegex(ValueError, 'geometry changed'):
            module.verify_preserved('source', 'output', selector, reader, values)


if __name__ == '__main__':
    unittest.main()
