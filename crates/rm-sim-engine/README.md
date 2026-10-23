<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# rm-sim-engine

Solver-free core for the incrementally separated robotics simulation engine.

- `Pose`: FLU translation in metres and wxyz quaternion storage.
- `time`: validated exact step durations and the existing 128 Hz default.
- `model`: validated immutable triangle surfaces with shared source buffers,
  independent of backend cooking; validated primitive/collider/body descriptions
  retain mounting frames, explicit contact data and dynamic mass/COM/inertia.
  `ConvexHull` explicitly requests a convex envelope from at least four finite
  local FLU points in metres. It shares immutable source buffers across clones
  and preserves point order, duplicates and interior points exactly. The hull
  operation fills concavities; backend cooking owns dimensionality and numerical
  tolerances and may reject degenerate or unrepresentable clouds. No tessellation
  inference, point perturbation or fallback shape is performed. Dynamic convex
  bodies require an explicit choice of inertia or collider mass aggregation;
  CAD mass is never inferred, and dynamic triangle meshes remain rejected. `BodyPolicy` preserves explicit gravity,
  axis locks, damping, sleep and CCD choices. `ColliderPolicy` declares mass and
  contact combination; `DynamicFromColliders` opts into native shape aggregation
  using caller-selected mass/density, independently of explicit `Inertial`.
  Joint descriptions support fixed, revolute and prismatic constraints with
  body-local parent/child frames, explicit connected-body contacts and optional
  limits. Frame +X defines the free axis; frames coincide at zero displacement.
  Revolute limits lie within [-π, π], without a multi-turn guarantee. Copied
  joint state and validated motor targets/gains remain separate from descriptions;
  runtime bindings supply body identities and reject targets outside limits
  without clamping. These contracts add no serialization or checkpoint format.
  `AssemblyDescription` retains caller-ordered links and joints as shared
  immutable arrays and validates a nonempty directed tree. `AssemblyLinkId` and
  `AssemblyJointId` are separate source ID namespaces, independent of runtime
  keys. Each link supplies a validated body and initial pose, COM linear velocity
  and angular velocity in the common assembly FLU frame (m, m/s and rad/s),
  never relative to its parent. Joint frames remain body-local. No initial joint
  coordinate, inertia, density or joint placement is inferred; backend preparation
  checks locked frames and capabilities, including fixed-to-fixed constraints.
- `contacts`: copied world FLU contact geometry without participants or scoring;
  backend sampling semantics remain explicit in the runtime observation.
- Model-local `MountTable` records share immutable carrier/mount attachments for
  sensor and visual pose consumers; live bindings and measurement state stay in
  physics/adapters. Bounded compound sources retain ordered child transforms as
  one collider; backend unsupported combinations reject without flattening.
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
cargo test -p rm-sim-engine --locked
cargo tree -p rm-sim-engine --edges normal
```

See [engine architecture](../../docs/engine-architecture.md) for the implemented
boundary and subsequent migration stages, and the [performance contract](../../docs/engine-performance-contract.md)
for buffer, lifecycle, identity and acceptance design.

This internal workspace crate is unpublished. The separate `rm-sim` registry
package is currently a documented development placeholder.
