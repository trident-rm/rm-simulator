#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Exercise season admission and isolated rules under each feature combination."""

from pathlib import Path
import subprocess


def check_build_graph(root, features):
    """Assert app feature forwarding without compiling native/GPU dependencies."""
    command = [
        "cargo", "tree", "--locked", "-p", "rm-simulator-app",
        "--no-default-features", "-e", "normal", "--prefix", "none",
        "--format", "{p} [{f}]",
    ]
    if features:
        command.extend(["--features", features])
    output = subprocess.check_output(command, cwd=root, text=True)
    expected = set(features.split(",")) if features else set()
    packages = {
        "rm-simulator-app", "rm-simulator-server",
        "rm-simulator-world", "rm-simulator-gameplay",
    }
    seen = set()
    for line in output.splitlines():
        package = line.split(" ", 1)[0]
        if package not in packages:
            continue
        actual = line.rsplit("[", 1)[1].split("]", 1)[0]
        actual = set(actual.split(",")) if actual else set()
        if actual != expected:
            raise RuntimeError(f"{package}: expected {expected}, got {actual}")
        seen.add(package)
    if seen != packages:
        raise RuntimeError(f"season feature graph is missing {packages - seen}")


def main():
    """Run only season admission tests when RM2026 gameplay is disabled."""
    root = Path(__file__).resolve().parent.parent
    for features in ("", "season-2026", "season-2027", "season-2026,season-2027"):
        print(f"Season feature graph: {features or '(none)'}", flush=True)
        check_build_graph(root, features)
        checks = [
            ("rm-simulator-gameplay", "season::tests"),
            ("rm-simulator-world", "season_tests"),
        ]
        if "season-2027" in features.split(","):
            checks.append(("rm-simulator-gameplay", "rmuc_2027::"))
            checks.append(("rm-simulator-world", "rmuc_2027::tests"))
        for package, test_filter in checks:
            command = [
                "cargo", "test", "--locked", "-p", package,
                "--no-default-features", "--lib", test_filter,
            ]
            if features:
                command.extend(["--features", features])
            print(f"Season rules/admission: {package}: {features or '(none)'}", flush=True)
            subprocess.run(command, cwd=root, check=True)
        if "season-2027" in features.split(","):
            subprocess.run([
                "cargo", "test", "--locked", "-p", "rm-simulator-gameplay",
                "--no-default-features", "--features", features,
                "--test", "control_2027", "--test", "field_buffs_2027",
                "--test", "terrain_2027", "--test", "rune_2027", "--test", "assembly_2027",
            ], cwd=root, check=True)


if __name__ == "__main__":
    main()
