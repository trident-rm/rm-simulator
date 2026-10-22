<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Reusing physics and armor motion

`rm-simulator-physics` depends on Rapier, serde and the standalone MID360 pattern
generator. It contains chassis
suspension and drive, gimbal dynamics, projectile flight, raw armor contacts,
shared collision geometry and analytic armor motion. It does not depend on
world, gameplay, server, rendering, a CAD loader or host time. Its
[crate README](../crates/rm-simulator-physics/README.md) lists its modules and
permitted dependencies.

The library retains RoboMaster-specific physical profiles and target labels.
`Caliber` includes the existing nominal detection/damage lookup helpers for
source compatibility, but the physical step does not apply HP, buffs, activation,
detection intervals or match rules. The caller controls chassis disable state.
This is a reusable RM physical library, not a general physics-engine abstraction.

## Contents

- [Headless use](#headless-use)
- [Restore and prediction](#restore-and-prediction)
- [Optional rendering](#optional-rendering)

## Headless use

For a caller-paced bare robot, use `physics::session::RobotSession`. It owns one
`WorldPhysics`, explicit time and a repeatable initial `RobotScenario`; it does
not construct a `Field`, native match authority or host. For example:

```rust
use rm_simulator_physics::{Pose, StepDuration, Team, chassis::ChassisConfig,
    session::{RobotScenario, RobotSession, RobotCommand}};

let scenario = RobotScenario::new(vec![], 0., StepDuration::HZ_1000, |physics| {
    physics.add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 0., 0.3]))?;
    Ok(())
})?;
let mut session = RobotSession::new(scenario)?;
session.command(RobotCommand::Drive { robot: 0, command: Default::default() })?;
session.step(1000)?;
assert_eq!(session.stamp().time_ns, 1_000_000_000);
session.reset()?;
# Ok::<(), &'static str>(())
```

The runnable `robot_session` example drives a real chassis for one second at
1000 Hz and resets. `step_controlled` calls a controller before each integration
step, with a reused command buffer. Drive, direct wheel torque and fire commands
use existing physical controls and the actual muzzle. Physical power, motor and
defeat limits remain; there is no match heat, allowance or firing cadence.
`step` returns raw contacts in tick order. `step_with_targets` interpolates one
tick's prescribed target endpoints, validating membership, identity order and
poses before stepping. All batches check clock overflow before advancing.

Read `robot_control_state` for allocation-free pose/input/revision/defeat truth,
`robot_truth` for timestamped robot state including cloned model/wheel data, or
`ground_truth` for privileged geometry/query access. None captures a checkpoint. LiDAR captures
use session time through `lidar_scene_capture`; sensors and their schedules remain
caller-owned. `physics_mut` permits model/geometry changes, resolved mechanisms,
payload motion, carrier tracing and event drains through ordinary physics methods.
Do not call `step` or change the synchronization clock through that accessor.

The lower-level target and projectile example remains available:

```sh
cargo run -p rm-simulator-physics --example moving_armor --locked
```

The example creates a moving armor face, launches ten projectiles and advances
1,000 explicit ticks. It requires no CAD, window, match or server. Raw contacts
report target identity, local contact position and normal speed. Consumers may
use these for their own sensors, detection policy or scoring.

Construct `WorldPhysics` with initial `TargetFace` poses and a catch-floor height.
Supply static meshes in metres, FLU coordinates, through `add_static_mesh` or
`add_boundary_mesh`. No default arena or file path is built into the library.
Use shared `StaticGeometry` captures for collision queries or rebuilding bodies.

For each tick:

1. Write start poses with `TargetFrames::begin`.
2. Advance caller-owned motion and write end poses with `TargetFrames::update`.
3. Supply resolved `MechanismState` if the world contains articulated scenery.
4. Call `WorldPhysics::step(start_time_ns, &frames)` once, then process contacts
   in returned order before beginning another tick.

`TargetFrames` validates target identity and ordering at both endpoints. External
targets have fixed membership for the lifetime of those frames. Chassis admission
and removal use the physical world's own methods. Do not change target ordering
without rebuilding the corresponding physical target bodies.

`RotorMotion` produces outpost angles and armor poses without HP. Set
`stopped_at_ns` when the consuming application decides motion should stop.
`SinusoidalMotion` computes the Big Rune speed profile; the native 2027 authority owns
seeded selection and activation transitions. Rune pose
helpers are available under `motion::rune`. `MechanismState` carries already
resolved door states and dart-rail position. Physics never asks a referee why
those states changed.

Time arguments are simulation time. The caller owns monotonicity, overflow checks,
fixed-tick scheduling and the motion state associated with each checkpoint.
Lengths and rotations use FLU metres and normalized wxyz quaternions. Existing
physical assumptions, projectile lifetime/capacity and launch validation remain
unchanged by extraction.

## Restore and prediction

The simulator still uses `Field::restore`, which rebuilds a complete physical
world plus its rules. It restores chassis and projectiles through the extracted
library's ordinary restore methods, inserts the shared geometry and restores
identity counters before continuing normal stepping. Prediction uses that same
facade, not a reduced local physics implementation.

`RobotSession::reset` rebuilds its session-owned initial scenario recipe on a
fresh physical world, swapping it into place only after setup succeeds. The
recipe can retain shared geometry, robot models, payload shapes/initial motion,
projectile policies and bounds, prescribed mechanism state and trace enablement.
It must use repeatable inputs and must not step or change synchronization time.
Reset restarts time and identities, clears pending physical events/traces, and
discards runtime edits absent from that recipe. It does not reset caller sensor
schedules, rolling brackets, IMU state or controller RNG; reset those explicitly.

There is deliberately no arbitrary `RobotSession` checkpoint/restore API in this
first version. `chassis_snapshots`, projectile `snapshot`, `restore_chassis`,
`restore_projectile`, `insert_geometry`, `restore_identity` and `mark_synced` are
low-level reconstruction tools, not a complete physical checkpoint contract.
A correct running-world continuation must also account for per-projectile
restitution/policy history, retirement timers and touched-armor memory, payload
geometry/motion, prescribed target endpoints and mechanism state, floor and
perimeter, power/defeat/control state, time/identity counters and pending contact,
removal and trace output. Solver warm starts/manifolds are rebuilt by existing
restore paths. In particular, restoring projectile snapshots with only the current
policy can change restitution for previously launched balls. Do not infer complete
or bitwise-equivalent continuation from these individual body restore methods.
The existing match checkpoint remains the match composition's responsibility.

The world facade re-exports existing `Pose`, `Team`, chassis and projectile types.
Serialized hit reports and outpost restore fields retain their previous shape.
Presentation clearance now takes `StaticGeometry::at_mechanisms`; the world
adapter `referee::mechanism_view` converts referee snapshots into its input.

## Optional rendering

`rm-simulator-render` remains a separate Bevy library with no physics dependency.
The consumer converts physical poses and its own appearance choices into
`SceneState`. The app's `physical_scene` example demonstrates that composition
without calling `Field`, the referee, CAD loading or server APIs:

```sh
cargo run -p rm-simulator-app --example physical_scene --locked
```

This demonstration advances 16 simulation milliseconds per rendered frame.
Applications that need real-time pacing should supply their own tick accumulator.
The renderer never advances physics or rules.

Chassis snapshot ingestion builds an ID index once per update and resolves
`PresentedPose`, armor state and equipment-light state components. Transform and
material systems consume changed components. Required components keep manually
spawned chassis parts usable in headless tests; the existing ID/part markers
remain available. The projectile visual pool remains unchanged.

CAD package parsing lives in `rm-simulator-assets`, without a server dependency.
The server retains compatibility imports and runtime-specific validation. No
additional asset format or sibling repository dependency is required. See
[simulation embedding](simulation-embedding.md) for caller-owned sessions, direct
wheel torque control, configurable step durations and future adapter boundaries.

Run the crate's own tests with `cargo test -p rm-simulator-physics --locked`;
[development](development.md) lists the workspace targets. The architecture
boundaries that keep this crate reusable are stated in
[simulation architecture](architecture-refactor.md).
