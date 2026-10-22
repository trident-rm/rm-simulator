<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# rm-simulator-engine

Solver-free core for the incrementally separated robotics simulation engine.

- `Pose`: FLU translation in metres and wxyz quaternion storage.
- `time`: validated exact step durations and the existing 128 Hz default.
- `queries`: immutable validated metric rays and capture-local hit records.
- `control`: held drive modes and validated wheel-axis torque requests.
- `wheel_model`: scalar suspension, slip, motor/inertia and shared-power laws
  with caller-owned state and reusable scratch.

Dependencies are `glam` and `serde`; tests additionally use `serde_json`.
No Rapier/Parry, other simulator crates, CAD, team/match state, Bevy, networking
or wall clock is required. This crate contains contracts and reusable model
calculations; physical stepping and batch query execution remain in the existing
Rapier-backed physics crate. That crate re-exports these types for compatibility.
There is no implicit world, plugin scheduler or device context.

```sh
cargo test -p rm-simulator-engine --locked
cargo tree -p rm-simulator-engine --edges normal
```

See [engine architecture](../../docs/engine-architecture.md) for the implemented
boundary and subsequent migration stages, and the [performance contract](../../docs/engine-performance-contract.md)
for buffer, lifecycle, identity and acceptance design.

This internal workspace crate is unpublished. The separate `rm-sim` registry
package is currently a documented development placeholder.
