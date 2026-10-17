# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Run with the CAD Python environment; default CI skips unavailable CAD deps."""
import importlib.util
from pathlib import Path
import unittest
try:
    import numpy as np
    import scipy
except ImportError:
    np=None

@unittest.skipIf(np is None,"requires rm-map-tools CAD Python (numpy/scipy)")
class BindingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec=importlib.util.spec_from_file_location("rune_binding",Path(__file__).resolve().parents[1]/"bind-rmuc2027-rune.py")
        cls.binding=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.binding)

    def test_joint_insertion_preserves_translated_rotated_scaled_rest_geometry(self):
        b=self.binding
        doc={"nodes":[{"name":"bank","children":[1]},
             {"name":"arm","translation":[2,3,4],"rotation":[0,0,.6,.8],"scale":[2,3,4],"children":[2]},
             {"name":"surface","mesh":0}],"scenes":[{"nodes":[0]}]}
        old=b.node_matrix(doc["nodes"][1])
        motion=b.bind(doc,[("bank",np.array([.3,.4,.5]),np.array([0,0,1]),"Red",["arm"])])
        new=b.node_matrix(doc["nodes"][-1])@b.node_matrix(doc["nodes"][1])
        np.testing.assert_allclose(old,new,atol=1e-12)
        self.assertEqual(motion[0]["bodies"],["arm/mesh-2"])

    def test_mixed_matrix_trs_rejected(self):
        with self.assertRaisesRegex(ValueError,"mixed"):
            self.binding.node_matrix({"matrix":np.eye(4).flatten().tolist(),"translation":[1,2,3]})
