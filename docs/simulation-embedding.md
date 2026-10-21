<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Embedding the simulator

The multiplayer app is one consumer of reusable libraries. In-process callers
can own a physical match without starting a server, a socket, a window or a
background worker. This guide describes the integration boundaries; it does not
add MuJoCo, ROS, Python, camera sensors, LiDAR or an RL training framework.

## Choose the layer

| Consumer | Entry point | Includes |
|---|---|---|
| Strategy evaluator | `rm-simulator-gameplay::rmuc_2027::match_rules::Match` | Native rules driven by caller observations and nanosecond durations; no physical travel/combat abstraction is invented. |
| Robot physical/control test | `rm-simulator-physics::WorldPhysics` | Rapier, wheel/suspension dynamics, gimbal, projectiles and raw contacts. |
| Complete physical match or robotics scenario | `rm-simulator-world::session::Session` | A `Field`, addressed controls, timestamps, checkpoints and explicit reset. |
| Verified map import | `rm-simulator-assets` | Manifests, checksums, collision GLBs, source bindings and articulation semantics, without server/network dependencies. |
| Scene viewer / future camera adapter | `rm-simulator-render::sync::SceneState` | Caller-owned appearance and poses; rendering never steps the world. |
| Multiplayer / external remote control | `rm-simulator-server` | Ownership, command admission, cadence, transport and real-time pacing. |

Use the lowest layer that supplies the behavior required. A strategy evaluator
does not need to load CAD; a physical test does not need to create a referee.
Asset loading preserves painted materials and source metadata; it does not
replace the existing app's robot admission and source joint binding adapters.

## Explicit timing

`StepDuration` is immutable per physical world. The default remains exactly
7,812,500 ns (128 Hz); `StepDuration::HZ_1000` selects exactly 1,000,000 ns.
`StepDuration::from_hz` supports other exact rates such as 500 or 2000 Hz;
`from_ns` accepts any positive integer-nanosecond duration up to the default
step. Coarser integration steps and fractional-nanosecond frequency conversions
are rejected. This rate selection changes resolution, not the modeled physics.
`FieldConfig::step_duration` selects the complete field's physical/rule integration
step. Standalone physics uses `WorldPhysics::with_step_duration`.

Use `field.time_ns()` and `field.step_duration()` for instance timing. The free
`tick_ns()` function remains the **default** duration for compatibility, not a
query of a particular field. A tick count alone is insufficient to compare two
sessions with different rates. Checkpoints retain the instance duration and
restore rejects inconsistent tick/time pairs.

Rule durations remain nanoseconds: a one-second timeout stays one second at
either rate. Sampling rates are independent of physical stepping. For example,
a 1000 Hz controller can run before every physical step while a caller samples
observations every 10 ms. `session::SampleSchedule` emits exact timestamps,
preserving phase across uneven caller updates. It does not synthesize historical
sensor readings: the adapter must sample at the correct step or interpolate
appropriate state. Save/reset caller-owned schedules alongside the episode.

The current multiplayer protocol, prediction and navigation retain the 128 Hz
contract. Server admission rejects nondefault fields rather than silently
interpreting a 1000 Hz tick as 7.8125 ms. Custom rates are an in-process API;
there is no CLI rate flag or environment override in this refactor. A future
network rate option must negotiate it end to end.

## Caller-owned sessions

Configure a `Field` with geometry, robots and optional native rules, then move it
into `Session::new`. There is one world and one owner. The session delegates to
the same `Field` methods used by the host; it does not implement alternate rules
or a second physics world.

```rust
use rm_simulator_world::{Field, FieldConfig, StepDuration, session::Session};

let field = Field::new(&FieldConfig {
    step_duration: StepDuration::HZ_1000,
    ..Default::default()
})?;
let mut session = Session::new(field);
session.step(1000)?;
assert_eq!(session.stamp().time_ns, 1_000_000_000);
let checkpoint = session.checkpoint();
session.step(100)?;
session.restore(&checkpoint)?;
session.reset(42)?;
assert_eq!(session.stamp().time_ns, 0);
# Ok::<(), rm_simulator_world::FieldError>(())
```

Run the complete driving example without external assets:

```sh
cargo run -p rm-simulator-world --example session --locked
```

`step_controlled` invokes a caller controller before each integration step and
reuses its command buffer. Commands apply in order. An error stops the run and
retains earlier accepted commands/steps; this is not a batch transaction.
`checkpoint` captures a branch point without changing the initial episode reset
point. `restore` is transactional. `reset(seed)` restores the original scenario
and seeds the native Rune stream. Other scenario generators and adapter RNGs
remain caller-owned and must be seeded explicitly. No random layout is generated.

Checkpoint capture shares immutable geometry rather than expanding triangles.
Restore rebuilds Rapier contacts/warm starts and touched-projectile caches, so it
has the existing physical residuals described in [physics reuse](physics-reuse.md).
It is not bit-identical solver rollback.

## Control and observation boundaries

`RobotCommand::Drive` preserves the existing velocity/gimbal/mechanism control.
`RobotCommand::WheelTorques` selects individual wheel-axis torque inputs in
newton metres, bypassing body-velocity tracking and balance assistance. Requests
still pass through motor envelopes, traction, suspension, power and defeat
constraints. A subsequent high-level drive command restores velocity control.
Gimbal/folding setpoints remain prescribed; this does not introduce a joint-level
arm or gimbal torque plant. The held drive mode is included in physical snapshots.

`RobotCommand::Fire` derives the muzzle from the actual robot. Native phase,
allowance and heat checks remain active. The trusted session does not add the
server's per-seat ownership or weapon-cadence policy: untrusted remote callers
must use the server protocol. `Session::referee` is explicitly privileged.

`ground_truth`, `robot_truth` and checkpoint access are named as privileged
truth. They must not be confused with measured sensor outputs or team-visible
Radar information. Future adapters should expose timestamped sensor observations
and keep these truth APIs available separately for labels, evaluation and debug.
This refactor creates no fake LiDAR, camera, IMU or encoder observations.

## Subsequent integrations

Camera readback/calibration, accelerated scan queries, Python environments and
robot bridges can now be developed as adapters. They should consume explicit
simulation timestamps and have caller-owned schedules, queues and RNGs. Rendering
and real-time pacing remain optional. Backend abstraction should follow an actual
second physics implementation; this refactor does not pretend that Rapier's
contact state, ray-cast wheel model and a future articulated MuJoCo model are
interchangeable.
