<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Embedding the simulator

The multiplayer app is one consumer of reusable libraries. In-process callers
can own a physical match without starting a server, a socket, a window or a
background worker. This guide describes the integration boundaries; it does not
add MuJoCo, ROS, Python, camera sensors or an RL training framework. A subsequent
passive [LiDAR foundation](navigation-sensors.md) supplies geometric rays and
ideal instantaneous scans, plus a mounted rolling MID-360 approximation with
per-point times, physical self-occlusion and seeded range effects, without an
external integration.
An [optional, separate rclrs adapter](../integrations/ros2/README.md) now publishes
those measured points as Livox Driver 2 messages and simulation `/clock` using
Pixi/RoboStack; a separate [ROS command bridge](../integrations/ros2/README.md#control-a-robot-in-an-existing-game)
can now drive a live app/server robot with leased body/world velocity or heading
vectors. Compact host clock/identity replies avoid snapshot polling, and HTTP I/O
runs on bounded workers outside ROS callbacks. Core libraries and multiplayer builds remain ROS-free.

## Choose the layer

| Consumer | Entry point | Includes |
|---|---|---|
| Strategy evaluator | `rm-simulator-gameplay::rmuc_2027::match_rules::Match` | Native rules driven by caller observations and nanosecond durations; no physical travel/combat abstraction is invented. |
| Robot physical/control test | `rm-simulator-physics::session::RobotSession` | Caller-paced physics, shared commands and sensor access; initial-scenario reset without match rules. |
| Complete physical match or robotics scenario | `rm-simulator-world::session::MatchSession` | A `Field`, addressed controls, timestamps, checkpoints and explicit reset. |
| Independent rollout collection | `physics::session::batch::RobotBatch` / `world::session::batch::MatchBatch` | Bounded sequential stepping, reusable observations and transactional selective reset; mixed per-slot durations. |
| Standalone equipment rules | `gameplay::rmuc_2027::equipment` and `rune::Rune` | Borrowed native Outpost/Base transitions and existing pure Rune authority; caller-owned explicit scheduling. |
| Verified map import | `rm-simulator-assets` | Manifests, checksums, collision GLBs, source bindings and articulation semantics, without server/network dependencies. |
| Scene viewer / future camera adapter | `rm-simulator-render::sync::SceneState` | Caller-owned appearance and poses; rendering never steps the world. |
| Multiplayer / external remote control | `rm-simulator-server` | Ownership, command admission, cadence, transport and real-time pacing. |

The two session layers share `Stamp`, `SampleSchedule`, `RobotCommand`,
`CommandResult` and `RobotTruth` from `physics::session`; the world session
re-exports them for compatibility. A `RobotCommand::Fire` request uses the actual
muzzle in both layers, but native heat/allowance/phase checks apply only through
the match composition. Physical motor, power and defeat limits apply in both.

For physics-only use, construct a `RobotScenario` and `RobotSession`; see
[the bare session example](physics-reuse.md#headless-use). It resets by rerunning
its initial setup recipe, not by restoring arbitrary running-world checkpoints.
Reset caller sensor schedules, interpolation/IMU state and controller RNG too.
Both sessions retain controller command storage across calls; sensor sampling and
returned contact storage remain caller concerns.

Use the lowest layer that supplies the behavior required. A strategy evaluator
does not need to load CAD; a physical test does not need to create a referee.
Asset loading preserves painted materials and source metadata; it does not
replace the existing app's robot admission and source joint binding adapters.

## Standalone equipment and rule adapters

[Standalone scenarios](standalone-scenarios.md) combine source-backed Outpost,
Rune and Base presets, or an explicit approximate tunnel fixture, with ordinary
`RobotSession` setup/reset. Definitions share cooked geometry across sessions.
`world::rule_adapters` supplies borrowed RFID layouts and contact detection gates;
the live world uses these same observation bridges. Pure traversal/Rune state
machines or an existing native Match remain the rule authority. No second clock,
referee scheduler or new protocol state is introduced.

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

The world entry point is named `MatchSession`: it composes physical simulation
with the Field's optional native competition authority, assembly and scoring.
`session::Session` and `session::Checkpoint` remain compatibility aliases for
`MatchSession` and `MatchCheckpoint`. Root imports also expose the explicit match
names. Existing callers may still pass a rule-free Field; the name does not
introduce an additional admission rule or a second physical world.

Configure a `Field` with geometry, robots and optional native rules, then move it
into `MatchSession::new`. There is one world and one owner. The session delegates to
the same `Field` methods used by the host; it does not implement alternate rules
or a second physics world.

```rust
use rm_simulator_world::{Field, FieldConfig, StepDuration, session::MatchSession};

let field = Field::new(&FieldConfig {
    step_duration: StepDuration::HZ_1000,
    ..Default::default()
})?;
let mut session = MatchSession::new(field);
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
retains and reuses its command buffer across calls. Commands apply in order. An error stops the run and
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
must use the server protocol. `MatchSession::referee` is explicitly privileged.

`ground_truth`, `robot_truth` and checkpoint access are named as privileged
truth. They must not be confused with measured sensor outputs or team-visible
Radar information. Future adapters should expose timestamped sensor observations
and keep these truth APIs available separately for labels, evaluation and debug.
The separate physics sensor module supplies ideal geometric LiDAR and a mounted
IMU over substep carrier traces; camera and encoder measurement models remain
separate future adapters.

The lower-level [wheel model](physics-reuse.md#backend-neutral-wheel-model)
now exposes scalar suspension, motor/inertia, slip/traction and power allocation
without a solver world. Existing sessions use it through the same Rapier chassis
adapter. Backend queries, ordered force application and stepping remain adapter
responsibilities; no backend selection or session/protocol change is introduced.

## Subsequent integrations

Camera readback/calibration, accelerated scan queries, Python environments and
robot bridges can now be developed as adapters. They should consume explicit
simulation timestamps and have caller-owned schedules, queues and RNGs. Rendering
and real-time pacing remain optional. Backend abstraction should follow an actual
second physics implementation; this refactor does not pretend that Rapier's
contact state, ray-cast wheel model and a future articulated MuJoCo model are
interchangeable.

## Batched independent sessions

The [batch facades](batched-rollouts.md) under `session::batch` retain per-slot
command buffers and fill caller-owned compact truth outputs. Slots keep their own
integration duration and timestamp. Selection/limits/overflow failures are
rejected before mutation; runtime command or integration errors report retained
partial progress. Selective reset prepares all replacements before swapping.
There is no thread pool, task/reward policy or implicit sensor reset.

For reusable multi-equipment fixtures, use [scenario composition](composable-scenarios.md).
Read [borrowed telemetry](telemetry.md) when a controller needs wheel or prescribed
joint state without cloning a complete checkpoint/model. [Scoped CPU workers](parallel-rollouts.md)
parallelize independent session requests with explicit per-request results and
no background work surviving a call; sensors and task/reward state remain external.
