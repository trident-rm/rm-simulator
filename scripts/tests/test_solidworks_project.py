# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Occurrence preservation and fail-closed preparation, without a native parser."""
import importlib.util
import tempfile
import unittest
from pathlib import Path

SPEC = importlib.util.spec_from_file_location("sw_project", Path(__file__).resolve().parents[1] / "prepare-solidworks-project.py")
PROJECT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROJECT)


class ProjectPreparationTests(unittest.TestCase):
    def fixture(self, root):
        for name in ["root.SLDASM", "p.SLDPRT"]:
            (root / name).write_bytes(name.encode())
        occurrence = lambda i, x: {"id": f"root/configuration:0/occurrence:{i}", "source": {
            "name": "part", "ref_config": "default", "is_hidden": True,
            "transform": PROJECT.IDENTITY[:12] + [x, 0., 0., 1.]},
            "transform_checks": {"rigid_structure": True},
            "resolution": {"document_id": "part"}}
        return {"schema": "rm-solidworks-metadata-v1", "documents": [
            {"id": "root", "source": {"path": "root.SLDASM", "sha256": PROJECT.digest(root / "root.SLDASM")},
             "configurations": [{"name": "default", "occurrences": [occurrence(0, 1.), occurrence(1, 2.)]}]},
            {"id": "part", "source": {"path": "p.SLDPRT", "sha256": PROJECT.digest(root / "p.SLDPRT")},
             "configurations": [{"name": "default", "occurrences": [], "physical_properties": {"values": {"mass": 1.}}}]}]}

    def test_repeated_hidden_occurrences_remain_distinct_and_cache_is_unreviewed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cat = self.fixture(root)
            calls = []

            def geometry(*args):
                calls.append(args)
                return [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]], [[0, 1, 2]]

            p = PROJECT.prepare(cat, root, "root.SLDASM", "default", None, geometry_reader=geometry)
            self.assertEqual(len(calls), 1)
            self.assertEqual(len(p["parts"]), 2)
            self.assertNotEqual(p["parts"][0]["occurrence"], p["parts"][1]["occurrence"])
            self.assertEqual([p["pose"]["translation_m"][0] for p in p["parts"]], [1., 2.])
            self.assertFalse(p["source_frames_and_geometry_reviewed"])
            self.assertEqual(p["groups"][0]["mass_mode"]["kind"], "unspecified")
            self.assertEqual(p["diagnostics"], [])

    def test_missing_suppressed_changed_source_and_cycle_are_explicit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cat = self.fixture(root)
            children = cat["documents"][0]["configurations"][0]["occurrences"]
            children[0]["resolution"]["document_id"] = "missing"
            children[1]["source"]["is_suppressed"] = True
            p = PROJECT.prepare(cat, root, "root.SLDASM", "default", None)
            self.assertEqual(len(p["diagnostics"]), 1)
            self.assertTrue(p["diagnostics"][0].endswith(": unresolved active reference"))
            children[0]["resolution"]["document_id"] = "root"
            p = PROJECT.prepare(cat, root, "root.SLDASM", "default", None)
            self.assertIn("cyclic", p["diagnostics"][0])
            (root / "root.SLDASM").write_text("changed")
            p = PROJECT.prepare(cat, root, "root.SLDASM", "default", None)
            self.assertIn("changed", p["diagnostics"][0])

    def test_transform_composition_and_reflection_rejection(self):
        a = PROJECT.IDENTITY[:12] + [1., 2., 3., 1.]
        b = PROJECT.IDENTITY[:12] + [-2., 0., 1., 1.]
        self.assertEqual(PROJECT.pose(PROJECT.multiply(a, b))["translation_m"], [-1., 2., 4.])
        reflected = PROJECT.IDENTITY.copy()
        reflected[0] = -1.
        with self.assertRaisesRegex(ValueError, "reflected"):
            PROJECT.pose(reflected)
        huge = PROJECT.IDENTITY.copy()
        huge[0] = 1e308
        with self.assertRaises(ValueError):
            PROJECT.pose(huge)

    def test_failed_geometry_is_not_reparsed_for_every_occurrence(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            cat=self.fixture(root)
            calls=[]

            def geometry(*args):
                calls.append(args)
                raise ValueError("unsupported source tessellation")

            p=PROJECT.prepare(cat,root,"root.SLDASM","default",None,geometry_reader=geometry)
            self.assertEqual(len(calls),1)
            self.assertEqual(len(p["diagnostics"]),2)
            self.assertEqual(p["parts"],[])


if __name__ == "__main__":
    unittest.main()
