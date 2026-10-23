#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Assert architectural dependency constraints from Cargo's resolved graph."""
import json
import subprocess

metadata = json.loads(subprocess.check_output(
    ["cargo", "metadata", "--format-version", "1", "--all-features", "--locked"], text=True))
packages = {p["id"]: p for p in metadata["packages"]}
nodes = {n["id"]: n["dependencies"] for n in metadata["resolve"]["nodes"]}

# Native ROS stays in integrations/ros2's opt-in workspace. Core verification
# must not need a sourced ROS installation, even with --all-features.
assert not ({"rclrs", "ros-env", "rosidl_runtime_rs", "r2r"}
            & {p["name"] for p in packages.values()})


def dependencies(name):
    root = next(p["id"] for p in packages.values() if p["name"] == name)
    seen = set()
    pending = list(nodes[root])
    while pending:
        node = pending.pop()
        if node not in seen:
            seen.add(node)
            pending.extend(nodes[node])
    return {packages[node]["name"] for node in seen}


engine = dependencies("rm-sim-engine")
assert not any(d.startswith(("bevy", "rapier", "parry", "rm-sim-")) for d in engine), engine
assert not ({"game-networking-sockets", "rclrs", "ros-env", "rosidl_runtime_rs", "r2r"} & engine), engine

gameplay = dependencies("rm-sim-gameplay")
assert not any(d.startswith(("bevy", "rapier")) for d in gameplay), gameplay
assert not any(d.startswith("rm-sim-") for d in gameplay), gameplay
physics = dependencies("rm-sim-physics")
assert not any(d.startswith("bevy") for d in physics), physics
assert not any(d.startswith("rm-sim-") and d != "rm-sim-engine" for d in physics), physics
assets = dependencies("rm-sim-assets")
assert not any(d.startswith("bevy") for d in assets), assets
assert not ({"rm-sim-world", "rm-sim-server", "rm-sim-render",
             "game-networking-sockets"} & assets), assets
world = dependencies("rm-sim-world")
assert not any(d.startswith("bevy") for d in world), world
assert "rm-sim-control" not in world, world
control = dependencies("rm-sim-control")
assert not any(d.startswith("bevy") for d in control), control
assert not ({"rm-sim-world", "rm-sim-server", "rm-sim-render", "rm-sim-presentation",
             "rm-sim-app", "game-networking-sockets", "rclrs", "ros-env",
             "rosidl_runtime_rs", "r2r"} & control), control
server = dependencies("rm-sim-server")
assert not any(d.startswith("bevy") for d in server), server
assert "rm-sim-render" not in server, server
assert "rm-sim-control" in server, server
render = dependencies("rm-sim-render")
assert "rm-sim-physics" not in render, render
assert "rm-sim-world" not in render, render
assert "rm-sim-server" not in render, render
assert "rm-sim-gameplay" not in render, render
presentation = dependencies("rm-sim-presentation")
assert {"rm-sim-world", "rm-sim-render"} <= presentation, presentation
assert not ({"rm-sim-server", "rm-sim-app", "game-networking-sockets"} & presentation), presentation
bench = dependencies("rm-sim-bench")
assert not ({"rm-sim-physics", "rm-sim-world", "rm-sim-server", "rm-sim-gameplay", "rm-sim-app"} & bench), bench
assert not any(d.startswith("rapier") for d in bench), bench
assert not any(p["name"] in ("rm-sim", "rm-sim-api", "rm-sim-viewer", "rm-vision-world",
                             "rm-vision-runtime", "rm-vision-runner") for p in packages.values())
print("PASS: engine has no solver, application or simulator dependency; physics' only simulator dependency is engine, "
      "gameplay is independent of physics and the simulator, "
      "assets have no world, networking or renderer, "
      "controllers have no match, host, networking or renderer, "
      "world and server have no renderer, renderer has no world or server, "
      "presentation adapts world to render without hosting or transport, "
      "and no sibling simulator crate is imported")
