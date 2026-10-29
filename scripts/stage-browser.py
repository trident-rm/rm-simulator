#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Stage a self-contained static browser viewer from a compiled Wasm binary."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parent.parent
PACKAGES = ("field-rmul-2027-1v1", "field-rmul-2027-3v3", "field-2027")


def extra_package(specification):
    """Collect an explicitly requested package without importing original CAD."""
    name, separator, path = specification.partition("=")
    if not separator or not name or Path(name).name != name or name in (".", "..", "pkg", "index.html", *PACKAGES) or any(c in name for c in "\\:%?#"):
        raise ValueError("Extra packages use NAME=PATH with one relative URL name")
    root = Path(path).resolve()
    result = {"CAD-NOTICE.md": root / "CAD-NOTICE.md"}
    if not result["CAD-NOTICE.md"].is_file():
        raise ValueError(f"Missing upstream ownership notice: {root}")
    if (root / "manifest.json").is_file():
        manifest = json.loads((root / "manifest.json").read_bytes())
        result["manifest.json"] = root / "manifest.json"
        supported = {"team-cad-v1", "artisans-folding-hero-v1", "dreamchaser-stair-heavy-v1", "pie-tunnel-heavy-v1", "future-tunnel-infantry-v1", "ega-tunnel-sentry-v1", "pie-rmuc2027-assembly-workbench-v5"}
        if manifest.get("schema") != 1 and manifest.get("profile") not in supported:
            raise ValueError("Unsupported optional package profile")
        entries = [(entry["file"], entry["sha256"]) for entry in manifest["files"].values()]
    else:
        area = json.loads((root / "area.json").read_bytes())
        result["area.json"] = root / "area.json"
        entries = [("area.glb", area["visual_sha256"]), ("terrain.json", area["collision_sha256"])]
    for relative, expected in entries:
        if not relative or Path(relative).name != relative or relative in (".", "..") or any(c in relative for c in "\\:%?#"):
            raise ValueError(f"Invalid optional package path: {relative}")
        if Path(relative).suffix not in (".json", ".glb"):
            raise ValueError(f"Unsupported optional payload format: {relative}")
        source = root / relative
        source.resolve().relative_to(root)
        if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
            raise ValueError(f"Optional package checksum mismatch: {source}")
        result[relative] = source
    return {f"{name}/{relative}": source for relative, source in result.items()}


def stage(output, profile, extra_packages=(), extra_files=()):
    """Verify package visuals before creating a new output directory."""
    target = Path(os.environ.get("CARGO_TARGET_DIR", ROOT / "target"))
    binary = target / "wasm32-unknown-unknown" / ("debug" if profile == "dev" else profile) / "rm-simulator.wasm"
    if not binary.is_file():
        raise ValueError(f"Build the browser binary first: {binary}")
    if output.exists():
        raise ValueError(f"Output already exists; choose a new directory: {output}")
    # The coupled application uses the same verified field/collision loaders as native.
    files = {name: ROOT / name for name in ("LICENSE-MIT", "LICENSE-APACHE", "NOTICE.md", "LICENSES/MPL-2.0.txt")}
    indexes = {}
    for package in PACKAGES:
        inventory = json.loads((ROOT / "scripts" / f"release-{package}.json").read_bytes())
        indexes[package] = inventory["files"]
        root = ROOT / package
        for relative, expected in inventory["files"].items():
            source = root / relative
            source.resolve().relative_to(root.resolve())
            if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
                raise ValueError(f"Package checksum mismatch: {source}. Hydrate Git LFS first.")
            files[f"{package}/{relative}"] = source
    for specification in extra_packages:
        additions = extra_package(specification)
        if additions.keys() & files.keys():
            raise ValueError("Optional package URL collides with an existing package")
        files.update(additions)
    for specification in extra_files:
        name, separator, path = specification.partition("=")
        if not separator or not name or Path(name).name != name or name in (".", "..", "pkg", "index.html", *PACKAGES) or any(c in name for c in "\\:%?#") or name in files:
            raise ValueError("Extra files use a unique NAME=PATH")
        files[name] = Path(path)
    output.mkdir(parents=True)
    shutil.copyfile(ROOT / "web/index.html", output / "index.html")
    shutil.copyfile(ROOT / "web/assembly-worker.js", output / "assembly-worker.js")
    shutil.copyfile(ROOT / "web/prediction-worker.js", output / "prediction-worker.js")
    subprocess.run(["wasm-bindgen", "--target", "web", "--out-dir", str(output / "pkg"),
                    "--out-name", "rm_simulator", "--remove-name-section", "--remove-producers-section", str(binary)], check=True)
    for name, source in files.items():
        destination = output / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
    for package, index in indexes.items():
        (output / package / "package-files.json").write_text(json.dumps(index, sort_keys=True) + "\n")
    print(f"Staged {len(files)} files and browser application in {output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT / "local-assets/browser")
    parser.add_argument("--profile", default="web", choices=("dev", "release", "web"))
    parser.add_argument("--extra-package", action="append", default=[], metavar="NAME=PATH", help="Explicit verified robot/assembly/area package to serve with its CAD notice")
    parser.add_argument("--extra-file", action="append", default=[], metavar="NAME=PATH", help="Explicit recording or IK hints file to serve unchanged")
    args = parser.parse_args()
    stage(args.out.resolve(), args.profile, args.extra_package, args.extra_file)


if __name__ == "__main__":
    main()
