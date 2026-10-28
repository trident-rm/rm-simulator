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


def stage(output, profile):
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
    output.mkdir(parents=True)
    shutil.copyfile(ROOT / "web/index.html", output / "index.html")
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
    args = parser.parse_args()
    stage(args.out.resolve(), args.profile)


if __name__ == "__main__":
    main()
