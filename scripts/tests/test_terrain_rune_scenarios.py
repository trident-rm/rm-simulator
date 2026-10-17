# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Check physical scenario geometry and refusal before mutation."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest
SCRIPTS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SCRIPTS))
spec=importlib.util.spec_from_file_location("terrain_runner",SCRIPTS/"terrain-rune-scenarios.py")
runner=importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)

class ScenarioTests(unittest.TestCase):
    def test_concave_card_sample_is_inside(self):
        zone={"polygon_m":[[0,0],[4,0],[4,1],[1,1],[1,4],[0,4]],"floor_m":.2}
        point=runner.interior(zone)
        self.assertTrue(runner.field.inside(point,zone["polygon_m"]))
        self.assertAlmostEqual(point[2],.6)

    def test_quaternion_rotates_and_inverts(self):
        q=[math.sqrt(.5),0,0,math.sqrt(.5)]
        p=runner.rotate(q,[1,0,0])
        for actual,expected in zip(p,[0,1,0]):self.assertAlmostEqual(actual,expected)
        for actual,expected in zip(runner.rotate([q[0],0,0,-q[3]],p),[1,0,0]):self.assertAlmostEqual(actual,expected)

    def test_unbound_or_non_referee_refused_before_mutation(self):
        for state in [{"role":"Pilot"},{"role":"Referee","rmuc_2027_config":{"runes":[]}}]:
            class Client:
                def __init__(self):self.commands=[]
                def call(self,command):self.commands.append(command);return state
            client=Client()
            with self.assertRaisesRegex(RuntimeError,"requires embedded"):runner.run(client)
            self.assertEqual(client.commands,[{"type":"ready"}])
