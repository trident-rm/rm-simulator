# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Pinned source identities prevent mixing seasons or fuzzy CAD partitions."""
import copy
import importlib.util
from pathlib import Path
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'build-rmuc2027-runtime.py'
SPEC = importlib.util.spec_from_file_location('rmuc2027_runtime', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SourceContractTests(unittest.TestCase):
    def setUp(self):
        self.recipe = {
            'source_file': 'RMUC2027_V1.0.0.stp', 'source_sha256': 'pinned',
            'central_asset': 'centre-platform',
            'selections': {'rune': {'product_id': 1}, 'outpost': {'product_id': 2}},
        }
        self.manifest = {
            'schema_version': 1, 'units': 'metres', 'source_up_axis': 'z',
            'source_file': self.recipe['source_file'], 'source_sha256': 'pinned',
            'body_coverage': {'complete': True, 'unowned': []},
            'assets': {name: {'in_full_map': True} for name in
                       ('floor', 'base', 'centre-platform')},
        }

    def test_exact_exporter_node_identity_is_required(self):
        self.assertEqual(MODULE.source_product('source_2603236_11'), 2603236)
        for name in ('outpost', 'source_2603236', '../source_2603236_11', 'source_2603236_11_extra'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                MODULE.source_product(name)

    def test_wrong_source_axes_or_incomplete_coverage_is_rejected(self):
        MODULE.validate_source(self.manifest, self.recipe)
        for key, value in [('source_file', 'RMUC2026.stp'), ('source_sha256', 'other'),
                           ('source_up_axis', '-y'), ('units', 'millimetres'),
                           ('body_coverage', {'complete': False, 'unowned': []}),
                           ('body_coverage', {'complete': True, 'unowned': [7]})]:
            bad = copy.deepcopy(self.manifest)
            bad[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                MODULE.validate_source(bad, self.recipe)

    def test_overlapping_or_missing_partitions_are_rejected(self):
        bad = copy.deepcopy(self.recipe)
        bad['selections']['outpost']['product_id'] = 1
        with self.assertRaises(ValueError):
            MODULE.validate_source(self.manifest, bad)
        self.manifest['assets'].pop('floor')
        with self.assertRaises(ValueError):
            MODULE.validate_source(self.manifest, self.recipe)

    def test_unplaced_component_cannot_disappear_from_the_runtime(self):
        self.manifest['assets']['unplaced'] = {'in_full_map': False}
        with self.assertRaises(ValueError):
            MODULE.validate_source(self.manifest, self.recipe)
