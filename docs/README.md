<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Documentation

Start with the [project README](../README.md) for installation, controls,
asset discovery and current server/client behavior. The guides below describe
the live implementation. Old investigations and measurements are available in
Git history.

## Current guides

| Topic | Guide |
|---|---|
| Contribution rules and review | [Contributing](../CONTRIBUTING.md) |
| Environment, `just` targets, tests and commits | [Development](development.md) |
| CI checks, validation reuse and releases | [CI and releases](releases.md) |
| Crate ownership, ticks, restore and ECS | [Simulation architecture](architecture-refactor.md) |
| Runtime/game split, two session layers and backend migration risks | [Runtime split](runtime-game-split.md) |
| Standalone physics and optional rendering | [Physics reuse](physics-reuse.md) |
| In-process sessions, clocks, controls and future adapters | [Simulation embedding](simulation-embedding.md) |
| Live-game navigation integration design, joint feedback and deferred MuJoCo | [Robotics next steps](robotics-next-steps.md) |
| Graphics presets, render limits and CPU-contention measurements | [Graphics performance](performance-graphics-20261023.md) |
| Nearest-hit geometry and ideal LiDAR acquisition | [Navigation sensors](navigation-sensors.md) |
| ROS delivery/shutdown and middleware trials | [ROS transport](ros-transport.md) |
| Optional ROS 2 / Livox Driver 2 publishing through rclrs | [ROS 2 adapter](../integrations/ros2/README.md) |
| Calibrated serial-arm IK optimization and paired benchmark | [Serial arm search](serial-arm-search.md) |
| Standalone gameplay and its live integration | [Gameplay](gameplay.md) |
| Rule sources and historical archive | [Referee rules](referee-rules.md) |
| RM2027 manuals, changes and implementation decisions | [2027 rules reference](rules-2027.md) |
| RMUL 2027 3v3/Infantry rules and practice limits | [RMUL 2027](rmul-2027.md) |
| Current runtime, 2026 archive and admission | [Runtime and archive](seasons.md) |
| Command-line options, controls and interface | [App options](app-options.md) |
| App automation, headless rendering and screenshots | [Console](console.md) |
| Transport-independent referee DTOs/mocks and custom-client link | [Referee link](referee-link.md) |
| Packet metadata and local channel diagnostics | [Network tracing](network-tracing.md) |
| RMUC 2027 source partitions, runtime build and visual review | [2027 field](rmuc2027-field.md) |
| Field package layout, discovery, composition, collision | [Field package](field-package.md) |
| Asset contracts and composition | [Semantic assets](semantic-assets.md) |
| Reproducible simplification and installation | [Field detail](field-detail.md) |
| Robot geometry and artwork assumptions | [Robot equipment](robot-equipment.md) |
| Cartesian Engineer assembly prototype and boundaries | [Assembly workbench](assembly-workbench.md) |
| CPU measurements and reproducibility | [Performance](performance.md) |
| Native rules, physical stepping and query baseline | [2026-10-21 measurements](performance-20261021.md) |
| Renderer benchmark commands and interpretation | [Render benchmark](render-benchmark.md) |
| Optional Steam integration | [Steam](steam.md) |
| License, provenance and third-party notices | [Notices](../NOTICE.md) |

- [Hardware and robot model candidates](hardware-models.md): source-derived referee modules, team CAD survey and assembled Omni prototype.

- [Tunnel RFID calibration](tunnel-rfid-calibration.md): CAD measurements, antenna approximations and live reward checks.
