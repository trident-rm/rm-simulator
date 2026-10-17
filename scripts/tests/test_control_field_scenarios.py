# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Validate field-scenario sampling without CAD, sockets or rendering."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location("field_scenarios", SCRIPTS / "control-field-scenarios.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class SamplingTests(unittest.TestCase):
    def test_concave_footprint_uses_interior_not_bounding_box(self):
        zone = {"polygon_m": [[0,0], [3,0], [3,1], [1,1], [1,3], [0,3]], "floor_m": .4}
        a,b = runner.positions(zone)
        self.assertTrue(runner.inside(a, zone["polygon_m"]))
        self.assertTrue(runner.inside(b, zone["polygon_m"]))
        self.assertGreaterEqual(math.dist(a,b), .8)
        self.assertEqual(a[2], .8)
        self.assertFalse(runner.inside([2,2], zone["polygon_m"]))

    def test_small_footprint_is_rejected_instead_of_overlapping_bots(self):
        with self.assertRaises(RuntimeError):
            runner.positions({"polygon_m": [[0,0], [.3,0], [.3,.3], [0,.3]], "floor_m":0})

    def test_fortress_prefers_central_plateau_over_ramp_corners(self):
        zone = {"point":"Fortress", "polygon_m":[[-1,-1], [1,-1], [1,1], [-1,1]], "floor_m":.15}
        a,b = runner.positions(zone)
        self.assertGreaterEqual(math.dist(a,b), .8)
        self.assertLess(max(abs(p[i]) for p in [a,b] for i in [0,1]), .6)

    def test_non_referee_scenario_fails_before_mutating_match(self):
        class Client:
            def __init__(self): self.commands = []
            def call(self, command):
                self.commands.append(command)
                return {"role":"Pilot", "rmuc_2027_config":{}}
        client = Client()
        with self.assertRaisesRegex(RuntimeError, "requires a local --referee"):
            runner.run(client)
        self.assertEqual(client.commands, [{"type":"ready"}])


if __name__ == "__main__":
    unittest.main()
