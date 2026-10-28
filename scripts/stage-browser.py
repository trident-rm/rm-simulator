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
    # Keep both manifests and the original CAD notice. Collision recipes and
    # physical sidecars are not part of this visual-only distribution.
    files = {name: ROOT / name for name in ("LICENSE-MIT", "LICENSE-APACHE", "NOTICE.md", "LICENSES/MPL-2.0.txt")}
    for package in PACKAGES:
        root = ROOT / package
        for relative in ("manifest.json", "equipment/manifest.json"):
            path = root / relative
            manifest = json.loads(path.read_bytes())
            files[f"{package}/{relative}"] = path
            for asset in manifest["assets"].values():
                relative_visual = Path(asset["visual"])
                if relative_visual.is_absolute() or ".." in relative_visual.parts:
                    raise ValueError(f"Invalid package path: {relative_visual}")
                visual = path.parent / relative_visual
                visual.resolve().relative_to(root.resolve())
                digest = hashlib.sha256(visual.read_bytes()).hexdigest()
                if digest != asset["visual_sha256"]:
                    raise ValueError(f"Visual checksum mismatch: {visual}. Hydrate Git LFS first.")
                files[f"{package}/{visual.relative_to(root).as_posix()}"] = visual
        notice = root / "CAD-NOTICE.md"
        if not notice.is_file():
            raise ValueError(f"Missing upstream ownership notice: {notice}")
        files[f"{package}/CAD-NOTICE.md"] = notice
    output.mkdir(parents=True)
    shutil.copyfile(ROOT / "web/index.html", output / "index.html")
    subprocess.run(["wasm-bindgen", "--target", "web", "--out-dir", str(output / "pkg"),
                    "--out-name", "rm_simulator", "--remove-name-section", "--remove-producers-section", str(binary)], check=True)
    for name, source in files.items():
        destination = output / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
    print(f"Staged {len(files)} files and browser application in {output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT / "local-assets/browser")
    parser.add_argument("--profile", default="web", choices=("dev", "release", "web"))
    args = parser.parse_args()
    stage(args.out.resolve(), args.profile)


if __name__ == "__main__":
    main()
