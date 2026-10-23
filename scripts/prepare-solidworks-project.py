#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Prepare an occurrence-preserving GUI project from openswx and sldkit output.

No mates or physical units are inferred. Cached mass remains unreviewed. Display
meshes are extracted separately from native B-Rep/edge bounds; originals are read
only. Requires the existing extract-solidworks.py catalogue and sldkit-rs CLI.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
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


def read_geometry(binary, path, sha256, config):
    with tempfile.TemporaryFile() as output, tempfile.TemporaryFile() as errors:
        run = subprocess.run([str(binary), "geometry", str(path)], stdout=output,
                             stderr=errors, timeout=60, check=False)
        if run.returncode or output.tell() > 128 * 1024 * 1024:
            raise ValueError("geometry parser failed or exceeded 128 MiB")
        output.seek(0)
        decoded = json.load(output)
    geometry = decoded.get("geometry") or {}
    if geometry.get("source", {}).get("sha256") != sha256 or digest(path) != sha256:
        raise ValueError("native bytes changed or geometry source hash mismatch")
    if geometry.get("length_unit") != "millimeter":
        raise ValueError("unsupported/unknown native display-mesh length unit")
    configurations = geometry.get("configurations", [])
    # No union of different configurations: current parser cannot reliably assign
    # every display tessellation to one configuration's solid bodies.
    if len(configurations) != 1 or configurations[0].get("name") != config:
        raise ValueError("display tessellation configuration mapping requires manual export")
    vertices, triangles = [], []
    for mesh in geometry.get("model", {}).get("tessellations", []):
        offset = len(vertices)
        points, indices = mesh.get("vertices", []), mesh.get("triangles", [])
        if any(len(v) != 3 or any(isinstance(x, bool) or not isinstance(x, (int, float))
                                    or not math.isfinite(x) for x in v) for v in points):
            raise ValueError("invalid native mesh vertices")
        if any(len(t) != 3 or any(isinstance(i, bool) or not isinstance(i, int)
                                 or i < 0 or i >= len(points) for i in t) for t in indices):
            raise ValueError("invalid native mesh indices")
        vertices.extend([[x * 0.001 for x in v] for v in points])
        triangles.extend([[i + offset for i in t] for t in indices])
        if len(vertices) > 250_000 or len(triangles) > 500_000:
            raise ValueError("native part mesh exceeds project limits")
    if not vertices or not triangles:
        raise ValueError("native Part has no supported display tessellation")
    return vertices, triangles


def prepare(catalogue, root, document, configuration, binary, max_parts=1024, geometry_reader=read_geometry):
    if catalogue.get("schema") != "rm-solidworks-metadata-v1":
        raise ValueError("expected native metadata catalogue v1")
    root = root.resolve()
    docs = {d["id"]: d for d in catalogue["documents"]}
    matches = [d for d in docs.values() if d["source"]["path"] == document]
    if len(matches) != 1:
        raise ValueError("choose an exact catalogue document path")
    parts, diagnostics, cache, failures = [], [], {}, {}
    total_vertices = 0

    def visit(doc, name, matrix, occurrence, ancestry):
        nonlocal total_vertices
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
        if total_vertices >= 2_000_000:
            diagnostics.append(occurrence + ": project mesh vertex limit")
            return
        try:
            if key in failures:
                raise ValueError(failures[key])
            if key not in cache:
                try:
                    cache[key] = geometry_reader(binary, path, doc["source"]["sha256"], name)
                except (ValueError, subprocess.TimeoutExpired, OSError) as error:
                    failures[key] = str(error)
                    raise
            vertices, triangles = cache[key]
            total_vertices += len(vertices)
            if total_vertices > 2_000_000:
                raise ValueError("project mesh vertex limit")
            parts.append({"id": len(parts) + 1, "name": path.stem,
                          "occurrence": occurrence, "source_sha256": doc["source"]["sha256"],
                          "pose": pose(matrix), "vertices_m": vertices, "triangles": triangles,
                          "group": 1, "native_cache": config.get("physical_properties", {}).get("values"),
                          "override_mass": None})
        except (ValueError, subprocess.TimeoutExpired, OSError) as error:
            diagnostics.append(occurrence + ": " + str(error))

    visit(matches[0], configuration, IDENTITY, "root", set())
    return {"schema": "rm-solidworks-project-v1", "source": document,
            "source_frames_and_geometry_reviewed": False, "diagnostics": diagnostics,
            "parts": parts, "groups": [{"id": 1, "name": "Review rigid group",
                "pose": pose(IDENTITY), "fixed": True, "proxy": "per_part_boxes",
                "mass_mode": {"kind": "unspecified"}, "joint": None}]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("catalogue", type=Path)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--document", required=True, help="exact relative catalogue path")
    parser.add_argument("--configuration", required=True)
    parser.add_argument("--sldkit", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--omit-missing-references", action="store_true", help="explicitly omit unresolved active occurrences and their dependent relations")
    args = parser.parse_args()
    if args.output.suffix.lower() != ".json":
        parser.error("output must be a JSON sidecar")
    result = prepare(json.loads(args.catalogue.read_text()), args.source_root,
                     args.document, args.configuration, args.sldkit.resolve(strict=True))
    result["provenance"] = {"catalogue_sha256": digest(args.catalogue),
                            "sldkit_binary_sha256": digest(args.sldkit),
                            "geometry": "cached_native_display_tessellation_not_BRep_mass_integration"}
    result["omitted_occurrences"] = []
    if args.omit_missing_references:
        omitted = [d for d in result["diagnostics"] if d.endswith(": unresolved active reference")]
        result["omitted_occurrences"] = omitted
        result["diagnostics"] = [d for d in result["diagnostics"] if d not in omitted]
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
    print(json.dumps({"parts": len(result["parts"]), "diagnostics": result["diagnostics"]}, ensure_ascii=False))
    return int(bool(result["diagnostics"]) or not result["parts"])


if __name__ == "__main__":
    raise SystemExit(main())
