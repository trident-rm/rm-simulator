#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Verify native 2027 competition rules with optional integrations disabled."""

from pathlib import Path
import json
import subprocess
import shutil


def main():
    """Reject season switches and exercise current rules without default features."""
    root = Path(__file__).resolve().parent.parent
    rust = "mbx" if shutil.which("mbx") else "cargo"
    metadata = json.loads(subprocess.check_output(
        ["cargo", "metadata", "--locked", "--no-deps", "--format-version", "1"],
        cwd=root, text=True,
    ))
    for package in metadata["packages"]:
        if any(name.startswith("season-") for name in package["features"]):
            raise RuntimeError(f"{package['name']} still exposes season switches")
    for package, test_filter in [
        ("rm-sim-gameplay", "season::tests"),
        ("rm-sim-gameplay", "rmuc_2027::"),
        ("rm-sim-gameplay", "rmul_2027::"),
        ("rm-sim-world", "rmul_2027::tests"),
        ("rm-sim-world", "rmuc_2027::tests"),
        ("rm-sim-server", "runtime_admission"),
        ("rm-sim-server", "rmul"),
    ]:
        subprocess.run([
            rust, "test", "--locked", "-p", package,
            "--no-default-features", "--lib", test_filter,
        ], cwd=root, check=True)
    subprocess.run([
        rust, "test", "--locked", "-p", "rm-sim-gameplay",
        "--no-default-features", "--test", "control_2027", "--test", "field_buffs_2027",
        "--test", "terrain_2027", "--test", "rune_2027", "--test", "assembly_2027",
        "--test", "adaptive_2027",
    ], cwd=root, check=True)


if __name__ == "__main__":
    main()
