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
            self.assertEqual(p["schema"], "rm-solidworks-project-v2")
            self.assertEqual(len(p["meshes"]), 1)
            self.assertEqual(p["parts"][0]["mesh"], p["parts"][1]["mesh"])
            self.assertNotIn("vertices_m", p["parts"][0])
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

    def test_exact_reindexing_retains_triangle_coordinates_including_signed_zero(self):
        import struct
        points=[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[-0.,0.,0.]]
        mesh={"vertices":points,"triangles":[[0,1,2],[3,2,1]]}
        g={"length_unit":"millimeter","configurations":[{"name":"default"}],"model":{"tessellations":[mesh,mesh]}}
        vertices,triangles,evidence=PROJECT.decode_geometry(g,"default")
        self.assertEqual(len(vertices),4)
        self.assertEqual(len(triangles),4)
        for t,out in zip(mesh["triangles"]*2,triangles):
            for a,b in zip(t,out):
                self.assertEqual(struct.pack("<ddd",*[v*0.001 for v in points[a]]),struct.pack("<ddd",*vertices[b]))
        self.assertEqual(evidence,"single_geometry_configuration_name")

    def test_sw_rs_geometry_requires_versioned_envelope_and_original_hash(self):
        import json
        import subprocess
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "part.SLDPRT"
            source.write_bytes(b"source")
            geometry = {"source": {"sha256": PROJECT.digest(source)},
                "length_unit": "millimeter", "configurations": [{"name": "default"}],
                "model": {"tessellations": [{"vertices": [[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                                             "triangles": [[0, 1, 2]]}]}}
            envelope = {"schema": "sw-rs-geometry-v1", "geometry": geometry}
            def run(command, **kwargs):
                self.assertEqual(command[1], "geometry")
                kwargs["stdout"].write(json.dumps(envelope).encode())
                return subprocess.CompletedProcess(command, 0)
            with patch.object(PROJECT.subprocess, "run", side_effect=run):
                vertices, triangles, _ = PROJECT.read_geometry(
                    source, source, PROJECT.digest(source), "default", 0, 1)
                self.assertEqual(vertices[1], [0.001, 0., 0.])
                self.assertEqual(triangles, [[0, 1, 2]])
                envelope.pop("schema")
                with self.assertRaisesRegex(ValueError, "schema"):
                    PROJECT.read_geometry(source, source, PROJECT.digest(source), "default")
                envelope["schema"] = "future"
                with self.assertRaisesRegex(ValueError, "schema"):
                    PROJECT.read_geometry(source, source, PROJECT.digest(source), "default")
                envelope["schema"] = "sw-rs-geometry-v1"
                geometry["source"]["sha256"] = "changed"
                with self.assertRaisesRegex(ValueError, "hash mismatch"):
                    PROJECT.read_geometry(source, source, PROJECT.digest(source), "default")

    def test_placeholder_config_name_requires_single_source_and_matching_index(self):
        g={"length_unit":"millimeter","configurations":[{"name":"Config-0","id":"sldprt:model:configuration#partition:0","source_index":0}],"model":{"tessellations":[{"vertices":[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]],"triangles":[[0,1,2]]}]}}
        _,_,evidence=PROJECT.decode_geometry(g,"默认",0,1)
        self.assertEqual(evidence,"single_source_configuration_index_alias")
        for index,count in [(0,2),(1,1),(None,1)]:
            with self.assertRaisesRegex(ValueError,"configuration mapping"):
                PROJECT.decode_geometry(g,"默认",index,count)
        g["configurations"].append({"name":"默认","active":True})
        with self.assertRaisesRegex(ValueError,"configuration mapping"):
            PROJECT.decode_geometry(g,"默认",0,1)

    def test_review_merge_keeps_existing_groups_and_stages_only_new_occurrences(self):
        import copy
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);cat=self.fixture(root)
            geometry=lambda *args: ([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]],[[0,1,2]])
            before=PROJECT.prepare(cat,root,"root.SLDASM","default",None,geometry_reader=geometry)
            before["groups"][0]["name"]="Manually reviewed chassis"
            before["parts"][0]["override_mass"]={"mass_kg":1.}
            before["source_frames_and_geometry_reviewed"]=True
            occurrence=copy.deepcopy(cat["documents"][0]["configurations"][0]["occurrences"][1]);occurrence["id"]="root/configuration:0/occurrence:2"
            cat["documents"][0]["configurations"][0]["occurrences"].append(occurrence)
            after=PROJECT.prepare(cat,root,"root.SLDASM","default",None,geometry_reader=geometry)
            summary=PROJECT.merge_review(after,before)
            self.assertEqual(summary["retained_occurrences"],2)
            self.assertEqual(summary["staged_occurrences"],1)
            self.assertEqual(after["groups"][0],before["groups"][0])
            self.assertEqual(after["parts"][0]["override_mass"],before["parts"][0]["override_mass"])
            self.assertNotEqual(after["parts"][2]["group"],before["groups"][0]["id"])
            self.assertFalse(after["groups"][1]["joint"]["reviewed"])
            self.assertFalse(after["source_frames_and_geometry_reviewed"])

    def test_review_merge_rejects_changed_occurrence_and_does_not_partially_mutate_on_error(self):
        import copy
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);cat=self.fixture(root)
            geometry=lambda *args: ([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]],[[0,1,2]])
            before=PROJECT.prepare(cat,root,"root.SLDASM","default",None,geometry_reader=geometry)
            after=copy.deepcopy(before);after["parts"][0]["pose"]["translation_m"][0]+=1.
            summary=PROJECT.merge_review(after,before)
            self.assertEqual(summary["requires_rebind"],1)
            self.assertTrue(any("explicit rebind" in d for d in after["diagnostics"]))
            bad=copy.deepcopy(before);bad["groups"][0]["id"]=2
            unchanged=copy.deepcopy(before)
            with self.assertRaises(ValueError):
                PROJECT.merge_review(unchanged,bad)
            self.assertEqual(unchanged,before)

    def test_explicit_omissions_remove_old_bodies_and_their_joint_relations(self):
        import copy
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);cat=self.fixture(root)
            geometry=lambda *args: ([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]],[[0,1,2]])
            before=PROJECT.prepare(cat,root,"root.SLDASM","default",None,geometry_reader=geometry)
            child=copy.deepcopy(before["groups"][0]);child["id"]=2
            child["joint"]={"parent":1,"motion":"fixed","pivot_m":[0.,0.,0.],"axis":[0.,0.,1.],"limits":None,"reviewed":True,"evidence":"manual"}
            before["groups"].append(child);before["parts"][1]["group"]=2
            after=copy.deepcopy(before);after["parts"]=after["parts"][1:]
            after["omitted_occurrences"]=[before["parts"][0]["occurrence"]+": unresolved active reference"]
            summary=PROJECT.merge_review(after,before)
            self.assertEqual(summary["omitted_groups"],1)
            self.assertEqual(summary["removed_joints"],1)
            self.assertEqual(summary["requires_rebind"],0)
            self.assertEqual(len(after["groups"]),1)
            self.assertEqual(after["groups"][0]["id"],2)
            self.assertIsNone(after["groups"][0]["joint"])

    def test_source_provenance_change_invalidates_joint_review_without_rewriting_it(self):
        import copy
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);cat=self.fixture(root)
            geometry=lambda *args: ([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]],[[0,1,2]])
            before=PROJECT.prepare(cat,root,"root.SLDASM","default",None,geometry_reader=geometry)
            child=copy.deepcopy(before["groups"][0]);child["id"]=2
            child["joint"]={"parent":1,"motion":"fixed","pivot_m":[0.,0.,0.],"axis":[0.,0.,1.],"limits":None,"reviewed":True,"evidence":"manual"}
            before["groups"].append(child);before["parts"][1]["group"]=2
            after=copy.deepcopy(before);after["provenance"]["root_native_sha256"]="f"*64
            summary=PROJECT.merge_review(after,before)
            self.assertEqual(summary["joint_reviews_invalidated"],1)
            self.assertFalse(after["groups"][1]["joint"]["reviewed"])
            self.assertEqual(after["groups"][1]["joint"]["evidence"],"manual")


if __name__ == "__main__":
    unittest.main()
