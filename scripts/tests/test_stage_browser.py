#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Optional browser packages retain notices and refuse unchecked or escaping files."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location("stage_browser", Path(__file__).resolve().parents[1] / "stage-browser.py")
STAGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(STAGE)


class OptionalPackages(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / "CAD-NOTICE.md").write_text("Original author and source terms\n")
        (self.root / "neutral.glb").write_bytes(b"source-payload")
        self.manifest = {"schema": 4, "profile": "pie-tunnel-heavy-v1", "files": {
            "neutral": {"file": "neutral.glb", "sha256": hashlib.sha256(b"source-payload").hexdigest()}}}
        self.write_manifest()

    def write_manifest(self):
        (self.root / "manifest.json").write_text(json.dumps(self.manifest))

    def test_checksums_and_original_notice_are_retained(self):
        files = STAGE.extra_package(f"robot={self.root}")
        self.assertEqual(set(files), {"robot/CAD-NOTICE.md", "robot/manifest.json", "robot/neutral.glb"})
        self.assertEqual(files["robot/CAD-NOTICE.md"].read_text(), "Original author and source terms\n")
        (self.root / "neutral.glb").write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "checksum mismatch"):
            STAGE.extra_package(f"robot={self.root}")

    def test_paths_cannot_escape_the_package_or_output(self):
        for name in ("../robot", "robot/other", "robot\\other", "..", "http:robot", "pkg", "index.html", "field-2027"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                STAGE.extra_package(f"{name}={self.root}")
        self.manifest["files"]["neutral"]["file"] = "../outside.glb"
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, "Invalid optional package path"):
            STAGE.extra_package(f"robot={self.root}")

    def test_original_cad_and_missing_ownership_notices_are_rejected(self):
        self.manifest["files"]["neutral"]["file"] = "source.STEP"
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, "Unsupported optional payload"):
            STAGE.extra_package(f"robot={self.root}")
        (self.root / "CAD-NOTICE.md").unlink()
        with self.assertRaisesRegex(ValueError, "ownership notice"):
            STAGE.extra_package(f"robot={self.root}")


if __name__ == "__main__":
    unittest.main()
