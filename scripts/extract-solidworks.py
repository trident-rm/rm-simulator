#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Extract a source-linked, unvalidated model catalogue using openswx swx_dump.

This offline metadata sidecar complements STEP geometry. It never installs mass
properties, assumes joints from mates, or converts source transforms into poses.
"""

import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path, PureWindowsPath
import subprocess
import tempfile


NATIVE = {".sldprt", ".sldasm"}


def digest(path):
    """Hash original bytes without retaining large CAD documents in memory."""
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def matrix_checks(values):
    """Check structure only; an upstream identity fallback can also pass."""
    if (not isinstance(values, list) or len(values) != 16
            or any(isinstance(x, bool) or not isinstance(x, (float, int))
                   or not math.isfinite(x) for x in values)):
        return {"rigid_structure": False, "reason": "invalid_matrix"}
    cols = [values[i:i + 3] for i in (0, 4, 8)]
    error = max(abs(sum(a * b for a, b in zip(cols[i], cols[j])) - (i == j))
                for i in range(3) for j in range(3))
    affine = max(abs(values[i] - v) for i, v in ((3, 0), (7, 0), (11, 0), (15, 1)))
    a, b, c = cols
    determinant = (a[0] * (b[1] * c[2] - b[2] * c[1])
                   - b[0] * (a[1] * c[2] - a[2] * c[1])
                   + c[0] * (a[1] * b[2] - a[2] * b[1]))
    if not all(math.isfinite(x) for x in (error, affine, determinant)):
        return {"rigid_structure": False, "reason": "matrix_arithmetic_overflow"}
    return {"rigid_structure": max(error, affine, abs(determinant - 1)) <= 1e-6,
            "basis_error": error, "homogeneous_error": affine,
            "determinant": determinant,
            "interpretation": "upstream_column_major_parent_relative_unverified",
            "source_presence": "unknown_upstream_may_default_to_identity"}


def property_checks(properties):
    """Retain cache values and flag necessary conditions, without accepting units."""
    flags = []
    mass = properties.get("mass")
    moments = properties.get("moments_of_inertia")
    if not isinstance(mass, (float, int)) or mass <= 0:
        flags.append("nonpositive_or_missing_mass")
    if not isinstance(moments, list) or len(moments) != 3:
        flags.append("missing_diagonal_inertia")
    elif any(not isinstance(x, (float, int)) or x < 0 for x in moments):
        flags.append("negative_or_invalid_diagonal_inertia")
    elif any(moments[i] > sum(moments[j] for j in range(3) if j != i) + 1e-12
             for i in range(3)):
        flags.append("diagonal_inertia_triangle_inequality")
    return {"status": "cached_unvalidated", "values": properties,
            "upstream_declared_units": "SI",
            "units_verified": False, "frame_verified": False,
            "freshness_verified": False, "necessary_condition_flags": flags}


def resolve_reference(reference, owner, root, inventory):
    """Prefer relative/suffix matches; never choose an ambiguous basename."""
    if not reference:
        return None, "empty_reference", []
    windows_path = PureWindowsPath(reference)
    parts = tuple(p.casefold() for p in windows_path.parts if p != ".")
    if not windows_path.anchor:
        relative = owner.parent.joinpath(*windows_path.parts).resolve()
        if not relative.is_relative_to(root):
            return None, "relative_reference_outside_search_root", []
        exact = [p for p in inventory
                 if str(p.relative_to(root)).casefold()
                 == str(relative.relative_to(root)).casefold()]
        if len(exact) == 1:
            return exact[0], "owner_relative_path", []
        if len(exact) > 1:
            return None, "ambiguous_reference", [str(p.relative_to(root)) for p in exact]
    name = parts[-1] if parts else ""
    candidates = [p for p in inventory if p.name.casefold() == name]
    if not candidates:
        return None, "missing_reference", []
    scored = []
    for path in candidates:
        local = tuple(p.casefold() for p in path.relative_to(root).parts)
        suffix = 0
        for left, right in zip(reversed(parts), reversed(local)):
            if left != right:
                break
            suffix += 1
        scored.append((suffix, path))
    best = max(score for score, _ in scored)
    winners = [path for score, path in scored if score == best]
    # A same-directory file is useful evidence only for a basename-only source.
    if len(parts) == 1:
        adjacent = [p for p in winners if p.parent == owner.parent]
        if len(adjacent) == 1:
            winners = adjacent
    if len(winners) == 1:
        return winners[0], "unique_path_suffix" if best > 1 else "unique_basename", []
    return None, "ambiguous_reference", [str(p.relative_to(root)) for p in winners]


def read_document(binary, path, timeout_s, max_output_bytes):
    """Bound execution and retained JSON; preserve subprocess diagnostics."""
    with tempfile.TemporaryFile() as output, tempfile.TemporaryFile() as errors:
        try:
            run = subprocess.run([str(binary), str(path)], stdout=output, stderr=errors,
                                 timeout=timeout_s, check=False)
        except (OSError, subprocess.TimeoutExpired) as error:
            return None, {"code": "parser_execution_failed", "detail": str(error)}
        errors.seek(0)
        diagnostic = errors.read(4096).decode("utf-8", errors="replace")
        if run.returncode:
            return None, {"code": "parser_nonzero_exit", "exit": run.returncode,
                          "detail": diagnostic}
        if output.tell() > max_output_bytes:
            return None, {"code": "parser_output_limit"}
        output.seek(0)
        try:
            document = json.load(output, parse_constant=reject_constant, parse_float=finite_float)
        except (ValueError, UnicodeError) as error:
            return None, {"code": "invalid_parser_json", "detail": str(error)}
        if not valid_document(document):
            return None, {"code": "invalid_parser_document"}
        return document, {"stderr": diagnostic} if diagnostic else None


def reject_constant(value):
    """JSON NaN/Infinity are not physical measurements or valid interchange."""
    raise ValueError(f"nonfinite JSON constant: {value}")


def finite_float(value):
    """Reject exponent overflow as well as nonstandard JSON constants."""
    parsed = float(value)
    if not math.isfinite(parsed):
        reject_constant(value)
    return parsed


def valid_document(document):
    """Validate every consumed nested field before mutating the catalogue."""
    if not isinstance(document, dict) or not isinstance(document.get("configurations"), list):
        return False
    for config in document["configurations"]:
        if not isinstance(config, dict) or not isinstance(config.get("components", []), list):
            return False
        properties = config.get("mass_properties")
        if properties is not None:
            if not isinstance(properties, dict):
                return False
            for key in ("mass", "volume", "surface_area"):
                if key in properties and (isinstance(properties[key], bool)
                        or not isinstance(properties[key], (int, float))):
                    return False
            for key in ("center_of_gravity", "moments_of_inertia", "products_of_inertia"):
                values = properties.get(key)
                # Upstream COM is an object; the inertia fields are arrays.
                if key == "center_of_gravity" and isinstance(values, dict):
                    values = list(values.values())
                if values is not None and (not isinstance(values, list) or len(values) != 3
                        or any(isinstance(x, bool) or not isinstance(x, (int, float)) for x in values)):
                    return False
        for component in config.get("components", []):
            if not isinstance(component, dict):
                return False
            if not isinstance(component.get("path", ""), str):
                return False
            if "ref_config" in component and not isinstance(component["ref_config"], str):
                return False
    return True


def extract(inputs, root, binary, max_documents=4096, timeout_s=30,
            max_output_bytes=64 * 1024 * 1024):
    """Build a bounded document graph, keeping unresolved occurrence evidence."""
    root = root.resolve()
    inventory = sorted({p.resolve() for p in root.rglob("*")
                        if p.suffix.lower() in NATIVE and p.is_file()
                        and p.resolve().is_relative_to(root)})
    queue = deque(sorted({p.resolve() for p in inputs}))
    if any(not p.is_relative_to(root) or p.suffix.lower() not in NATIVE for p in queue):
        raise ValueError("inputs must be native Parts/Assemblies beneath search root")
    documents, seen, diagnostics, links = [], set(), [], []
    while queue:
        path = queue.popleft()
        if path in seen:
            continue
        if len(seen) >= max_documents:
            diagnostics.append({"code": "document_limit", "remaining": len(queue) + 1})
            break
        seen.add(path)
        relative = str(path.relative_to(root))
        source_hash = digest(path)
        # Path identifies occurrences of identical bytes stored in different places.
        doc_id = "document:" + hashlib.sha256(relative.encode()).hexdigest()
        document, diagnostic = read_document(binary, path, timeout_s, max_output_bytes)
        row = {"id": doc_id, "source": {"path": relative, "sha256": source_hash},
               "configurations": []}
        documents.append(row)
        if diagnostic:
            diagnostics.append({"document": doc_id, **diagnostic})
        if digest(path) != source_hash:
            diagnostics.append({"document": doc_id, "code": "source_changed_during_parse"})
            row["status"] = "failed"
            continue
        if document is None:
            row["status"] = "failed"
            continue
        row["status"] = "metadata_only"
        row["metadata"] = {k: v for k, v in document.items()
                           if k not in ("configurations", "preview_png_base64", "sheets")}
        for ordinal, config in enumerate(document["configurations"]):
            if not isinstance(config, dict) or not isinstance(config.get("components", []), list):
                raise ValueError("invalid upstream configuration structure")
            config_id = f"{doc_id}/configuration:{ordinal}"
            item = {"id": config_id, "source_index": config.get("index"),
                    "name": config.get("name"), "metadata": {
                        k: v for k, v in config.items()
                        if k not in ("components", "preview_png_base64", "mass_properties")},
                    "occurrences": []}
            if config.get("mass_properties") is not None:
                item["physical_properties"] = property_checks(config["mass_properties"])
            row["configurations"].append(item)
            for index, component in enumerate(config.get("components", [])):
                occurrence = {"id": f"{config_id}/occurrence:{index}",
                              "source": component,
                              "transform_checks": matrix_checks(component.get("transform"))}
                target, resolution, candidates = resolve_reference(
                    component.get("path", ""), path, root, inventory)
                occurrence["resolution"] = {"method": resolution, "candidates": candidates}
                item["occurrences"].append(occurrence)
                if target:
                    queue.append(target)
                    links.append((occurrence, target))
                else:
                    diagnostics.append({"occurrence": occurrence["id"], "code": resolution})
    by_path = {d["source"]["path"]: d for d in documents}
    for occurrence, target in links:
        target_document = by_path.get(str(target.relative_to(root)))
        occurrence["resolution"]["document_id"] = target_document["id"] if target_document else None
        if target_document is None:
            occurrence["resolution"]["status"] = "not_parsed_document_limit"
        elif target_document["status"] == "failed":
            occurrence["resolution"]["status"] = "target_parse_failed"
        else:
            occurrence["resolution"]["status"] = "parsed_metadata"
            requested = occurrence["source"].get("ref_config")
            if requested is not None:
                matches = [c for c in target_document["configurations"] if c["name"] == requested]
                if len(matches) == 1:
                    occurrence["resolution"]["configuration_id"] = matches[0]["id"]
                else:
                    diagnostics.append({"occurrence": occurrence["id"],
                                        "code": "missing_or_ambiguous_child_configuration",
                                        "requested": requested})
    return {"schema": "rm-solidworks-metadata-v1", "documents": documents,
            "diagnostics": diagnostics,
            "contract": {"geometry": "external_STEP_or_checked_mesh_required",
                         "physical_properties": "cached_unvalidated_not_runtime_parameters",
                         "occurrence_transforms": "upstream_decoded_not_independently_validated",
                         "kinematics": "no_joint_or_mate_inference"}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="root native assembly/part, or inventory directory")
    parser.add_argument("--search-root", type=Path)
    parser.add_argument("--swx-dump", type=Path, required=True)
    parser.add_argument("--openswx-revision", help="declared upstream revision, not binary attestation")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-documents", type=int, default=4096)
    parser.add_argument("--timeout-s", type=float, default=30)
    parser.add_argument("--max-output-mib", type=int, default=64)
    args = parser.parse_args()
    if (args.max_documents < 1 or args.timeout_s <= 0 or not math.isfinite(args.timeout_s)
            or args.max_output_mib < 1):
        parser.error("limits must be finite and positive")
    if args.output.resolve().suffix.lower() != ".json":
        parser.error("output must be a JSON sidecar, never a native CAD file")
    source = args.input.resolve()
    root = (args.search_root or (source if source.is_dir() else source.parent)).resolve()
    inputs = ([p for p in source.rglob("*") if p.suffix.lower() in NATIVE and p.is_file()]
              if source.is_dir() else [source])
    binary = args.swx_dump.resolve(strict=True)
    result = extract(inputs, root, binary, args.max_documents, args.timeout_s,
                     args.max_output_mib * 1024 * 1024)
    result["tool"] = {"name": "openswx/swx_dump", "binary_sha256": digest(binary),
                      "declared_revision": args.openswx_revision}
    result["summary"] = {
        "documents": len(result["documents"]),
        "failed_documents": sum(d["status"] == "failed" for d in result["documents"]),
        "configurations": sum(len(d["configurations"]) for d in result["documents"]),
        "physical_property_records": sum("physical_properties" in c
            for d in result["documents"] for c in d["configurations"]),
        "occurrences": sum(len(c["occurrences"])
            for d in result["documents"] for c in d["configurations"]),
        "diagnostics": len(result["diagnostics"])}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Atomic replacement avoids leaving a seemingly valid partial catalogue.
    with tempfile.NamedTemporaryFile(mode="w", dir=args.output.parent,
                                     encoding="utf-8", delete=False) as output:
        temporary = Path(output.name)
        try:
            json.dump(result, output, ensure_ascii=False, indent=2, allow_nan=False)
            output.write("\n")
            output.close()
            temporary.replace(args.output)
        finally:
            temporary.unlink(missing_ok=True)
    print(json.dumps(result["summary"], sort_keys=True))
    return int(result["summary"]["failed_documents"] > 0
               or any(d.get("code") == "document_limit" for d in result["diagnostics"]))


if __name__ == "__main__":
    raise SystemExit(main())
