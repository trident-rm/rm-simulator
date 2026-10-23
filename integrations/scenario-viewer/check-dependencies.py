#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Verify the viewer resolves no match/host/ROS dependency or RM render feature."""
import json
import pathlib
import subprocess

manifest = pathlib.Path(__file__).with_name("Cargo.toml")
metadata = json.loads(subprocess.check_output([
    "cargo", "metadata", "--manifest-path", str(manifest), "--locked", "--format-version", "1"
]))
packages = {package["id"]: package for package in metadata["packages"]}
for node in metadata["resolve"]["nodes"]:
    name = packages[node["id"]]["name"]
    assert name not in {"rm-simulator-assets", "rm-simulator-gameplay", "rm-simulator-world",
                        "rm-simulator-server", "rm-simulator-presentation", "rclrs", "rosidl_runtime_rs"}, name
    if name == "rm-simulator-render":
        assert "robomaster" not in node["features"], node["features"]
for name, version in [("bevy", "0.20.0"), ("wgpu", "30.0.1")]:
    versions = {package["version"] for package in packages.values() if package["name"] == name}
    assert versions == {version}, (name, versions)
print("viewer dependency boundary OK: generic rendering + engine + physics; Bevy 0.20 / wgpu 30")
