#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Prepare an occurrence-preserving GUI project from sw-rs native metadata and mesh output.

No mates or physical units are inferred. Cached mass remains unreviewed. Display
meshes are extracted separately from native B-Rep/edge bounds; originals are read
only. Requires the existing extract-solidworks.py catalogue and sw-rs CLI.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import struct
import tempfile


IDENTITY = [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.]


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def multiply(a, b):
    return [sum(a[k * 4 + row] * b[col * 4 + k] for k in range(4))
            for col in range(4) for row in range(4)]


def pose(m):
    if len(m) != 16 or any(isinstance(x, bool) or not isinstance(x, (int, float))
                           or not math.isfinite(x) for x in m):
        raise ValueError("invalid occurrence matrix")
    for i in range(3):
        for j in range(3):
            if abs(sum(m[4 * i + k] * m[4 * j + k] for k in range(3)) - (i == j)) > 1e-6:
                raise ValueError("non-rigid occurrence matrix")
    if any(abs(m[i] - value) > 1e-6 for i, value in ((3, 0), (7, 0), (11, 0), (15, 1))):
        raise ValueError("non-affine occurrence matrix")
    # Convert a proper rotation, never a mirrored/scaled native transform.
    a, b, c = m[0:3], m[4:7], m[8:11]
    det = a[0] * (b[1] * c[2] - b[2] * c[1]) - b[0] * (a[1] * c[2] - a[2] * c[1]) + c[0] * (a[1] * b[2] - a[2] * b[1])
    if abs(det - 1) > 1e-6:
        raise ValueError("reflected occurrence matrix")
    r = [[m[col * 4 + row] for col in range(3)] for row in range(3)]
    trace = sum(r[i][i] for i in range(3))
    if trace > 0:
        s = math.sqrt(trace + 1) * 2
        q = [s / 4, (r[2][1] - r[1][2]) / s,
             (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s]
    else:
        i = max(range(3), key=lambda j: r[j][j])
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(0., 1 + r[i][i] - r[j][j] - r[k][k])) * 2
        q = [0.] * 4
        q[0] = (r[k][j] - r[j][k]) / s
        q[i + 1] = s / 4
        q[j + 1] = (r[j][i] + r[i][j]) / s
        q[k + 1] = (r[k][i] + r[i][k]) / s
    norm = math.sqrt(sum(x * x for x in q))
    return {"translation_m": m[12:15], "rotation_wxyz": [x / norm for x in q]}


def decode_geometry(geometry, config, source_index=None, source_config_count=None):
    """Accept an unambiguous configuration and exactly reindex repeated vertices."""
    if geometry.get("length_unit") != "millimeter":
        raise ValueError("unsupported/unknown native display-mesh length unit")
    configurations = geometry.get("configurations", [])
    if len(configurations) != 1:
        raise ValueError("display tessellation configuration mapping requires manual export")
    selected = configurations[0]
    evidence = "single_geometry_configuration_name"
    if selected.get("name") != config:
        alias = (source_config_count == 1 and isinstance(source_index, int)
                 and not isinstance(source_index, bool) and source_index >= 0
                 and selected.get("source_index") == source_index
                 and selected.get("name") == f"Config-{source_index}"
                 and selected.get("id", "").endswith(f"#partition:{source_index}"))
        if not alias:
            raise ValueError("display tessellation configuration mapping requires manual export")
        evidence = "single_source_configuration_index_alias"
    vertices, triangles, indices_by_bits = [], [], {}
    for mesh in geometry.get("model", {}).get("tessellations", []):
        points, indices = mesh.get("vertices", []), mesh.get("triangles", [])
        if any(len(v) != 3 or any(isinstance(x, bool) or not isinstance(x, (int, float))
                                    or not math.isfinite(x) for x in v) for v in points):
            raise ValueError("invalid native mesh vertices")
        if any(len(t) != 3 or any(isinstance(i, bool) or not isinstance(i, int)
                                 or i < 0 or i >= len(points) for i in t) for t in indices):
            raise ValueError("invalid native mesh indices")
        remap = []
        for point in points:
            converted = [x * 0.001 for x in point]
            bits = struct.pack("<ddd", *converted)
            if bits not in indices_by_bits:
                indices_by_bits[bits] = len(vertices)
                vertices.append(converted)
            remap.append(indices_by_bits[bits])
        triangles.extend([[remap[i] for i in t] for t in indices])
        if len(vertices) > 250_000 or len(triangles) > 500_000:
            raise ValueError("native part mesh exceeds project limits")
    if not vertices or not triangles:
        raise ValueError("native Part has no supported display tessellation")
    return vertices, triangles, evidence


def read_geometry(binary, path, sha256, config, source_index=None, source_config_count=None,
                  expected_schema=None):
    with tempfile.TemporaryFile() as output, tempfile.TemporaryFile() as errors:
        run = subprocess.run([str(binary), "geometry", str(path)], stdout=output,
                             stderr=errors, timeout=60, check=False)
        if output.tell() > 128 * 1024 * 1024:
            raise ValueError("geometry parser output exceeds 128 MiB")
        output.seek(0)
        decoded = json.load(output)
    if not isinstance(decoded, dict):
        raise ValueError("invalid geometry reader document")
    if run.returncode:
        codes = ",".join(str(d.get("code", "unknown")) for d in decoded.get("diagnostics", [])[:3])
        raise ValueError(f"geometry parser exit {run.returncode}: {codes}")
    if expected_schema is not None and decoded.get("schema") != expected_schema:
        raise ValueError("unsupported geometry reader schema")
    if "schema" in decoded and decoded["schema"] != "sw-rs-geometry-v1":
        raise ValueError("unsupported geometry reader schema")
    geometry = decoded.get("geometry") or {}
    if geometry.get("source", {}).get("sha256") != sha256 or digest(path) != sha256:
        raise ValueError("native bytes changed or geometry source hash mismatch")
    return decode_geometry(geometry, config, source_index, source_config_count)


def read_sw_rs_geometry(*args):
    """Require the versioned sw-rs envelope rather than mislabelling another CLI."""
    return read_geometry(*args, expected_schema="sw-rs-geometry-v1")


def prepare(catalogue, root, document, configuration, binary, max_parts=1024, geometry_reader=read_geometry):
    if catalogue.get("schema") != "rm-solidworks-metadata-v1":
        raise ValueError("expected native metadata catalogue v1")
    root = root.resolve()
    docs = {d["id"]: d for d in catalogue["documents"]}
    matches = [d for d in docs.values() if d["source"]["path"] == document]
    if len(matches) != 1:
        raise ValueError("choose an exact catalogue document path")
    parts, diagnostics, meshes, failures = [], [], {}, {}
    total_vertices = 0
    placed_vertices = 0

    def visit(doc, name, matrix, occurrence, ancestry):
        nonlocal total_vertices, placed_vertices
        key = (doc["id"], name)
        if key in ancestry or len(ancestry) >= 64:
            diagnostics.append(occurrence + ": cyclic/too-deep assembly")
            return
        configs = [c for c in doc["configurations"] if c["name"] == name]
        if len(configs) != 1:
            diagnostics.append(occurrence + ": missing/ambiguous configuration")
            return
        config = configs[0]
        path = (root / doc["source"]["path"]).resolve()
        if not path.is_relative_to(root) or not path.is_file() or digest(path) != doc["source"]["sha256"]:
            diagnostics.append(occurrence + ": missing/changed/outside-root source")
            return
        if path.suffix.lower() == ".sldasm":
            children = config["occurrences"]
            if not children:
                diagnostics.append(occurrence + ": empty assembly is not a physical Part")
            for child in children:
                source = child["source"]
                # Hidden is visual-only, not a physical suppression flag.
                if source.get("is_suppressed"):
                    continue
                child_doc = docs.get(child["resolution"].get("document_id"))
                label = occurrence + "/" + child["id"].rsplit("/", 1)[-1]
                if child_doc is None:
                    diagnostics.append(label + ": unresolved active reference")
                    continue
                if child.get("transform_checks", {}).get("rigid_structure") is not True:
                    diagnostics.append(label + ": unchecked occurrence transform")
                    continue
                try:
                    next_matrix = multiply(matrix, source["transform"])
                    pose(next_matrix)
                    visit(child_doc, source.get("ref_config"), next_matrix, label, ancestry | {key})
                except (KeyError, ValueError) as error:
                    diagnostics.append(label + ": " + str(error))
            return
        if len(parts) >= max_parts:
            diagnostics.append(occurrence + ": part limit")
            return
        try:
            geometry_key = (doc["source"]["sha256"], name)
            mesh_id = "mesh:" + hashlib.sha256((geometry_key[0] + "\0" + name).encode()).hexdigest()
            if geometry_key in failures:
                raise ValueError(failures[geometry_key])
            if mesh_id not in meshes:
                try:
                    decoded = geometry_reader(binary, path, geometry_key[0], name,
                                              config.get("source_index"), len(doc["configurations"]))
                    vertices, triangles = decoded[:2]
                    evidence = decoded[2] if len(decoded) == 3 else "external_geometry_reader"
                    if total_vertices + len(vertices) > 2_000_000:
                        raise ValueError("project unique mesh vertex limit")
                    meshes[mesh_id] = {"source_sha256": geometry_key[0], "configuration": name,
                                       "mapping_evidence": evidence,
                                       "vertices_m": vertices, "triangles": triangles}
                    total_vertices += len(vertices)
                except (ValueError, subprocess.TimeoutExpired, OSError) as error:
                    failures[geometry_key] = str(error)
                    raise
            mesh = meshes[mesh_id]
            if placed_vertices + len(mesh["vertices_m"]) > 8_000_000:
                raise ValueError("project placed mesh vertex limit")
            placed_vertices += len(mesh["vertices_m"])
            parts.append({"id": len(parts) + 1, "name": path.stem,
                          "occurrence": occurrence, "source_sha256": doc["source"]["sha256"],
                          "pose": pose(matrix), "mesh": mesh_id, "configuration": name,
                          "group": 1, "native_cache": config.get("physical_properties", {}).get("values"),
                          "override_mass": None})
        except (ValueError, subprocess.TimeoutExpired, OSError) as error:
            diagnostics.append(occurrence + ": " + str(error))

    visit(matches[0], configuration, IDENTITY, "root", set())
    return {"schema": "rm-solidworks-project-v2", "source": document,
            "provenance": {"root_native_sha256": matches[0]["source"]["sha256"],
                           "root_configuration": configuration},
            "source_frames_and_geometry_reviewed": False, "diagnostics": diagnostics,
            "parts": parts, "meshes": meshes, "groups": [{"id": 1, "name": "Review rigid group",
                "pose": pose(IDENTITY), "fixed": True, "proxy": "per_part_boxes",
                "mass_mode": {"kind": "unspecified"}, "joint": None}]}


def geometry_signature(project, part, cache):
    """Order-independent vertex set plus ordered triangle positions; bit exact."""
    mesh = project.get("meshes", {}).get(part.get("mesh")) if part.get("mesh") else part
    if mesh is None:
        raise ValueError("previous review references a missing mesh")
    vertices, triangles = mesh["vertices_m"], mesh["triangles"]
    key = (id(vertices), id(triangles))
    if key not in cache:
        encoded = [struct.pack("<ddd", *v) for v in vertices]
        h = hashlib.sha256()
        for value in sorted(set(encoded)):
            h.update(value)
        h.update(b"triangles")
        for triangle in triangles:
            for index in triangle:
                h.update(encoded[index])
        cache[key] = h.hexdigest()
    return cache[key]


def merge_review(project, previous):
    """Retain exact occurrence reviews; stage additions without guessing joints."""
    if previous.get("schema") in {"rm-reviewed-solidworks-recipe-v1", "rm-reviewed-solidworks-recipe-v2"}:
        previous = previous["project"]
    if previous.get("schema") not in {"rm-solidworks-project-v1", "rm-solidworks-project-v2"} or previous.get("source") != project["source"]:
        raise ValueError("review merge requires the same root document and a supported project")
    original = project
    project = {**project, "parts": [dict(p) for p in project["parts"]],
               "diagnostics": list(project["diagnostics"]),
               "provenance": dict(project.get("provenance", {}))}
    groups = previous.get("groups", [])
    ids = {g["id"] for g in groups}
    if not groups or len(groups) != len(ids) or any(isinstance(i, bool) or not isinstance(i, int) or i < 0 or i >= 2 ** 64 for i in ids):
        raise ValueError("invalid previous group identities")
    old = {p["occurrence"]: p for p in previous["parts"]}
    if len(old) != len(previous["parts"]):
        raise ValueError("duplicate previous occurrence identities")
    signatures, matched, staging = {}, set(), []
    for part in project["parts"]:
        before = old.get(part["occurrence"])
        same = (before is not None and before["source_sha256"] == part["source_sha256"]
                and before["pose"] == part["pose"]
                and before.get("native_cache") == part.get("native_cache")
                and (not before.get("configuration") or before["configuration"] == part["configuration"])
                and geometry_signature(previous, before, signatures) == geometry_signature(project, part, signatures))
        if same:
            if before["group"] not in ids:
                raise ValueError("previous occurrence references a missing group")
            part["group"] = before["group"]
            part["override_mass"] = copy.deepcopy(before.get("override_mass"))
            matched.add(part["occurrence"])
        else:
            staging.append(part)
    prefixes = [d.split(": ", 1)[0] for d in project.get("omitted_occurrences", [])]
    omitted_reviews = {o for o in set(old) - matched
                       if any(o == prefix or o.startswith(prefix + "/") for prefix in prefixes)}
    removed_groups = {g["id"] for g in groups if any(p["group"] == g["id"] for p in old.values())
                      and all(p["occurrence"] in omitted_reviews for p in old.values() if p["group"] == g["id"])}
    project["groups"] = copy.deepcopy([g for g in groups if g["id"] not in removed_groups])
    removed_joints = sum(g.get("joint") is not None for g in groups if g["id"] in removed_groups)
    for group in project["groups"]:
        if group.get("joint") and group["joint"]["parent"] in removed_groups:
            group["joint"] = None
            removed_joints += 1
    if staging:
        next_id = max(ids) + 1
        if next_id >= 2 ** 64 or len(project["groups"]) >= 256:
            raise ValueError("no available staging group identity/capacity")
        roots = [g["id"] for g in project["groups"] if g.get("joint") is None]
        project["groups"].append({"id": next_id, "name": "New occurrences (review)",
            "pose": pose(IDENTITY), "fixed": False, "proxy": "per_part_boxes",
            "mass_mode": {"kind": "unspecified"}, "joint": ({"parent": roots[0],
                "motion": "fixed", "pivot_m": [0., 0., 0.], "axis": [0., 0., 1.],
                "limits": None, "reviewed": False, "evidence": ""} if len(roots) == 1 else None)})
        for part in staging:
            part["group"] = next_id
    unmatched = set(old) - matched - omitted_reviews
    for occurrence in sorted(unmatched):
        project["diagnostics"].append(occurrence + ": previous review requires explicit rebind (source/pose/geometry changed or absent)")
    before_proof, after_proof = previous.get("provenance", {}), project.get("provenance", {})
    proof_key = next((k for k in ("catalogue_sha256", "root_native_sha256")
                      if before_proof.get(k) and after_proof.get(k)), None)
    source_unchanged = bool(proof_key and before_proof[proof_key] == after_proof[proof_key])
    if before_proof.get("root_configuration") and after_proof.get("root_configuration"):
        source_unchanged = source_unchanged and before_proof["root_configuration"] == after_proof["root_configuration"]
    invalidated = 0
    if not source_unchanged:
        for group in project["groups"]:
            if group.get("joint") and group["joint"].get("reviewed"):
                group["joint"]["reviewed"] = False
                invalidated += 1
    project["source_frames_and_geometry_reviewed"] = bool(previous.get("source_frames_and_geometry_reviewed") and not staging and not unmatched and not omitted_reviews and source_unchanged)
    summary = {"retained_occurrences": len(matched), "staged_occurrences": len(staging), "requires_rebind": len(unmatched), "joint_reviews_invalidated": invalidated,
               "omitted_review_occurrences": len(omitted_reviews),
               "omitted_groups": len(removed_groups), "removed_joints": removed_joints}
    project.setdefault("provenance", {})["review_merge"] = json.dumps(summary, sort_keys=True)
    original.clear()
    original.update(project)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("catalogue", type=Path)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--document", required=True, help="exact relative catalogue path")
    parser.add_argument("--configuration", required=True)
    readers = parser.add_mutually_exclusive_group(required=True)
    readers.add_argument("--sw-rs", type=Path, help="sw-rs sw-dump metadata/geometry CLI")
    readers.add_argument("--sldkit", type=Path, help="legacy external sldkit CLI")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--merge-review", type=Path, help="retain unchanged occurrence reviews and stage new Parts")
    parser.add_argument("--omit-missing-references", action="store_true", help="explicitly omit unresolved active occurrences and their dependent relations")
    args = parser.parse_args()
    if args.output.suffix.lower() != ".json":
        parser.error("output must be a JSON sidecar")
    binary = (args.sw_rs or args.sldkit).resolve(strict=True)
    result = prepare(json.loads(args.catalogue.read_text()), args.source_root,
                     args.document, args.configuration, binary,
                     geometry_reader=read_sw_rs_geometry if args.sw_rs else read_geometry)
    result.setdefault("provenance", {}).update({"catalogue_sha256": digest(args.catalogue),
                            "geometry_reader": {"name": "sw-rs/sw-dump" if args.sw_rs else "sldkit-rs",
                                                "binary_sha256": digest(binary)},
                            "geometry": "cached_native_display_tessellation_not_BRep_mass_integration",
                            "preparer_sha256": digest(Path(__file__))})
    result["omitted_occurrences"] = []
    if args.omit_missing_references:
        omitted = [d for d in result["diagnostics"] if d.endswith(": unresolved active reference")]
        result["omitted_occurrences"] = omitted
        result["diagnostics"] = [d for d in result["diagnostics"] if d not in omitted]
    if args.merge_review:
        if args.merge_review.stat().st_size > 256 * 1024 * 1024:
            parser.error("previous review exceeds 256 MiB")
        merge_review(result, json.loads(args.merge_review.read_text()))
        result["provenance"]["merged_review_sha256"] = digest(args.merge_review)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=args.output.parent, delete=False) as out:
        temporary = Path(out.name)
        try:
            json.dump(result, out, ensure_ascii=False, allow_nan=False)
            out.write("\n")
            out.close()
            temporary.replace(args.output)
        finally:
            temporary.unlink(missing_ok=True)
    print(json.dumps({"parts": len(result["parts"]), "unique_meshes": len(result["meshes"]),
                      "unique_vertices": sum(len(m["vertices_m"]) for m in result["meshes"].values()),
                      "diagnostics": result["diagnostics"]}, ensure_ascii=False))
    return int(bool(result["diagnostics"]) or not result["parts"])


if __name__ == "__main__":
    raise SystemExit(main())
