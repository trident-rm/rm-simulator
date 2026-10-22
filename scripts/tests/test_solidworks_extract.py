# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Native catalogue resolution, trust boundaries, cycles and parser failures."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    "solidworks_extract", Path(__file__).resolve().parents[1] / "extract-solidworks.py")
EXTRACT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXTRACT)
IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


class SolidworksExtractionTests(unittest.TestCase):
    def test_rejects_reflection_and_nonfinite_but_never_claims_verified_pose(self):
        self.assertTrue(EXTRACT.matrix_checks(IDENTITY)["rigid_structure"])
        self.assertIn("unverified", EXTRACT.matrix_checks(IDENTITY)["interpretation"])
        reflected = IDENTITY.copy()
        reflected[0] = -1
        self.assertFalse(EXTRACT.matrix_checks(reflected)["rigid_structure"])
        reflected[0] = float("nan")
        self.assertFalse(EXTRACT.matrix_checks(reflected)["rigid_structure"])
        self.assertFalse(EXTRACT.matrix_checks([1])["rigid_structure"])
        huge = IDENTITY.copy()
        huge[0] = huge[5] = huge[10] = 1e308
        result = EXTRACT.matrix_checks(huge)
        self.assertFalse(result["rigid_structure"])
        json.dumps(result, allow_nan=False)

    def test_resolution_keeps_ambiguity_and_uses_source_path_suffix(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            inventory = [root / "a" / "motor.SLDPRT", root / "b" / "motor.SLDPRT"]
            for path in inventory:
                path.parent.mkdir()
                path.touch()
            owner = root / "root.SLDASM"
            target, code, candidates = EXTRACT.resolve_reference(
                "motor.SLDPRT", owner, root, inventory)
            self.assertIsNone(target)
            self.assertEqual(code, "ambiguous_reference")
            self.assertEqual(len(candidates), 2)
            target, code, _ = EXTRACT.resolve_reference(
                r"C:\old\b\MOTOR.sldprt", owner, root, inventory)
            self.assertEqual(target, inventory[1])
            self.assertEqual(code, "unique_path_suffix")
            self.assertEqual(EXTRACT.resolve_reference("", owner, root, inventory)[1],
                             "empty_reference")

    def test_parent_relative_reference_precedes_same_directory_basename(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            parent, adjacent = root / "robot/motor.SLDPRT", root / "robot/sub/motor.SLDPRT"
            for path in (parent, adjacent):
                path.parent.mkdir(parents=True, exist_ok=True)
                path.touch()
            target, method, _ = EXTRACT.resolve_reference(
                r"..\motor.SLDPRT", adjacent.parent / "owner.SLDASM", root, [parent, adjacent])
            self.assertEqual(target, parent)
            self.assertEqual(method, "owner_relative_path")
            self.assertEqual(EXTRACT.resolve_reference(
                r"..\..\..\motor.SLDPRT", adjacent.parent / "owner.SLDASM", root,
                [parent, adjacent])[1], "relative_reference_outside_search_root")

    def test_cache_is_retained_without_accepting_units_or_inventing_tensor(self):
        cache = {"mass": 17.25, "moments_of_inertia": [761, 1169, 758],
                 "products_of_inertia": [-29, -42, 30]}
        result = EXTRACT.property_checks(cache)
        self.assertEqual(result["values"], cache)
        self.assertFalse(result["units_verified"])
        self.assertFalse(result["frame_verified"])
        self.assertFalse(result["freshness_verified"])
        self.assertEqual(result["status"], "cached_unvalidated")
        result = EXTRACT.property_checks({"mass": -1, "moments_of_inertia": [1, 1, 4]})
        self.assertEqual(len(result["necessary_condition_flags"]), 2)

    def test_cycle_keeps_occurrences_configs_source_hashes_and_missing_refs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            assembly, part = root / "root.SLDASM", root / "part.SLDPRT"
            assembly.write_bytes(b"source assembly")
            part.write_bytes(b"source part")
            docs = {
                assembly: {"doc_type": "assembly", "configurations": [
                    {"name": "stowed", "preview_png_base64": "omit", "components": [
                        {"name": "part", "path": "part.SLDPRT", "transform": IDENTITY,
                         "is_suppressed": True, "is_hidden": True},
                        {"path": "missing.SLDPRT", "transform": IDENTITY}]},
                    {"name": "raised", "components": []}]},
                part: {"doc_type": "part", "configurations": [
                    {"name": "default", "components": [
                        {"path": "root.SLDASM", "transform": IDENTITY}]}]}}
            with patch.object(EXTRACT, "read_document", side_effect=lambda _, p, *_a: (docs[p], None)):
                catalogue = EXTRACT.extract([assembly], root, root / "swx_dump")
            self.assertEqual(len(catalogue["documents"]), 2)
            document = catalogue["documents"][0]
            self.assertEqual(document["source"]["sha256"], EXTRACT.digest(assembly))
            self.assertEqual(len(document["configurations"]), 2)
            config = document["configurations"][0]
            self.assertNotIn("preview_png_base64", config["metadata"])
            occurrence = config["occurrences"][0]
            self.assertTrue(occurrence["source"]["is_suppressed"])
            self.assertTrue(occurrence["source"]["is_hidden"])
            self.assertEqual(occurrence["resolution"]["document_id"],
                             catalogue["documents"][1]["id"])
            self.assertIn("missing_reference", [d["code"] for d in catalogue["diagnostics"]])
            self.assertEqual(assembly.read_bytes(), b"source assembly")

    def test_document_limit_does_not_dangle_a_falsely_resolved_document(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            a, b = root / "a.SLDASM", root / "b.SLDPRT"
            a.touch()
            b.touch()
            doc = {"configurations": [{"components": [{"path": b.name}]}]}
            with patch.object(EXTRACT, "read_document", return_value=(doc, None)):
                result = EXTRACT.extract([a], root, root / "swx_dump", max_documents=1)
            occurrence = result["documents"][0]["configurations"][0]["occurrences"][0]
            self.assertIsNone(occurrence["resolution"]["document_id"])
            self.assertEqual(occurrence["resolution"]["status"], "not_parsed_document_limit")

    def test_parser_failure_is_an_explicit_failed_document(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            source = root / "a.SLDASM"
            source.touch()
            with patch.object(EXTRACT, "read_document", return_value=(
                    None, {"code": "parser_nonzero_exit", "exit": 2})):
                result = EXTRACT.extract([source], root, root / "swx_dump")
            self.assertEqual(result["documents"][0]["status"], "failed")
            self.assertEqual(result["diagnostics"][0]["exit"], 2)
            json.dumps(result, allow_nan=False)

    def test_real_subprocess_json_limits_and_nonfinite_rejection(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "synthetic.SLDPRT"
            source.write_text('{"configurations": []}')
            document, error = EXTRACT.read_document(Path("/bin/cat"), source, 1, 1024)
            self.assertEqual(document, {"configurations": []})
            self.assertIsNone(error)
            _, error = EXTRACT.read_document(Path("/bin/cat"), source, 1, 2)
            self.assertEqual(error["code"], "parser_output_limit")
            source.write_text('{"configurations": [], "mass": NaN}')
            _, error = EXTRACT.read_document(Path("/bin/cat"), source, 1, 1024)
            self.assertEqual(error["code"], "invalid_parser_json")
            source.write_text('{"configurations": [], "mass": 1e999}')
            _, error = EXTRACT.read_document(Path("/bin/cat"), source, 1, 1024)
            self.assertEqual(error["code"], "invalid_parser_json")
            for malformed in (
                    {"configurations": [None]},
                    {"configurations": [{"components": [None]}]},
                    {"configurations": [{"mass_properties": []}]},
                    {"configurations": [{"components": [{"path": 4}]}]}):
                source.write_text(json.dumps(malformed))
                _, error = EXTRACT.read_document(Path("/bin/cat"), source, 1, 1024)
                self.assertEqual(error["code"], "invalid_parser_document")

    def test_child_configuration_is_resolved_without_defaulting_to_first(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            a, b = root / "a.SLDASM", root / "b.SLDPRT"
            a.touch()
            b.touch()
            docs = {
                a: {"configurations": [{"components": [
                    {"path": b.name, "ref_config": "second"},
                    {"path": b.name, "ref_config": "missing"}]}]},
                b: {"configurations": [{"name": "first"}, {"name": "second"}]}}
            with patch.object(EXTRACT, "read_document", side_effect=lambda _, p, *_a: (docs[p], None)):
                result = EXTRACT.extract([a], root, root / "swx_dump")
            occurrences = result["documents"][0]["configurations"][0]["occurrences"]
            self.assertTrue(occurrences[0]["resolution"]["configuration_id"].endswith("configuration:1"))
            self.assertNotIn("configuration_id", occurrences[1]["resolution"])
            self.assertEqual(result["diagnostics"][-1]["code"],
                             "missing_or_ambiguous_child_configuration")


if __name__ == "__main__":
    unittest.main()
