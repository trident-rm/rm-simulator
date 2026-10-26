<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# rm-sim-control

Caller-driven navigation and assembly planning without a match, server, network
or renderer. Inputs are copied physical observations and shared geometry;
outputs are commands, plans and diagnostics. Callers own time, permissions,
worker scheduling, cancellation and authoritative actuation.

- `navigation`: supported-grid bake/search, explicit passage hints, bounded
  waypoints, conservative footprint checks and explicit-interval following.
- `assembly`: source-bound Cartesian rig, staged IK/collision planning, fixed
  grasp/twist freedom, cancellation/progress and 128 Hz playback retiming.
- `manipulator_plan`: deterministic IK candidate selection and bounded joint-space
  search over physical arm models and caller collision checks.
- `test-support`: optional synthetic rig fixture for diagnostic tests.

```sh
cargo run -p rm-sim-control --example navigation --locked
cargo test -p rm-sim-control --all-features --locked
```

The example drives a bare physical robot around a wall without constructing a
world referee or host. See [controller contracts](../../docs/controllers.md) for
inputs, lifecycle, remaining limitations and preserved game integration.
