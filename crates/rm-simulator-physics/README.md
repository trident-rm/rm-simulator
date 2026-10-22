<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# rm-simulator-physics

`rm-simulator-physics` is a reusable, gameplay-free Rapier library: wheel and body
dynamics, projectile flight, raw armor contacts, shared collision geometry and
prescribed mechanism motion. It knows no match rules, CAD package, socket, Bevy or
host clock; callers supply target endpoint poses and resolved mechanism state and
step it once per explicit tick. It sits under the world crate and depends on no
other simulator crate.

## Modules

| File | Owns |
|---|---|
| `src/session.rs` | Caller-paced `RobotSession` over one `WorldPhysics`, repeatable `RobotScenario` reset, addressed commands, compact truth and explicit sampling schedules. No match authority or arbitrary checkpoint restore. |
| `src/control.rs` | Validated held drive modes and per-wheel motor torque requests, independent of a transport or controller scheduler. |
| `src/sensors.rs` | Prepared ideal planar/multi-layer LiDAR, caller-stamped instantaneous acquisition and reusable output buffers over explicit geometry captures. |
| `src/sensors/imu.rs` | Mounted gyroscope/accelerometer on the chassis or actual gimbal frame, derived from opt-in solver-substep carrier traces, with restorable state and seeded noise, bias walk and saturation. |
| `src/sensors/rolling.rs` | Mounted procedural MID-360 acquisitions with interpolated robot/obstacle motion, self-occlusion, timestamps and seeded range noise/dropout. |
| `src/wheel_model.rs` | Backend-neutral scalar suspension/roller, wheel inertia, motor/slip and shared-power equations over explicit parameters, borrowed state/torques and reused buffers. No solver handles or queries. |
| `src/chassis.rs` | Compatibility facade for chassis configuration, commands, snapshots and borrowed telemetry; existing public paths remain unchanged. |
| `src/chassis/model.rs` | Physical descriptions, presets, validation and evaluated model geometry, including conversion to `WheelModel`; no solver handles or stepping. |
| `src/chassis/backend/` | Rapier body/collider construction (`construction.rs`), ordered wheel queries and forces (`dynamics.rs`), prescribed folding/gimbal adaptation (`prescribed.rs`), controls and observations (`mod.rs`). |
| `src/projectile.rs` | `WorldPhysics`, `Shot`, `Caliber`, `Contact`, `TargetFace`, `TargetFrames`, `StaticGeometry` and `ProjectilePolicy`: projectile flight, scoring faces and raw contacts in physical order. |
| `src/geometry.rs` | Shared collision captures and read-only clearance queries consumed by the world facade. |
| `src/motion.rs` | Prescribed rotor and rail motion (`RotorMotion`, `SinusoidalMotion`, `MechanismState`) and fitted armor geometry; rune pose helpers live under `motion::rune`. |

`lib.rs` also holds the clock contract: `DEFAULT_TICK_NS` and the `const fn`
`tick_ns()`, plus the FLU `Pose` and `Team`. Time arguments count ticks of
`tick_ns()`, which is 128 Hz by default (7.8125 ms per tick). Standalone callers can select immutable
`StepDuration` values, including 1000 Hz, through `WorldPhysics::with_step_duration`.
Use the instance duration to advance timestamps; `tick_ns()` returns the default. The step applies no HP, buffs, activation or
detection intervals; the world crate owns those decisions.

## Dependencies

`rapier3d-f64`, `serde` and the standalone `mid360-sim` pattern generator. The
crate never depends on `rm-simulator-world`,
`rm-simulator-gameplay`, `rm-simulator-server`, `rm-simulator-render` or Bevy,
and it never reads host time. Standalone callers can use `session::RobotSession`
without `rm-simulator-world`; match composition continues to use the same physical
plant. `rm-simulator-app` also uses it for the `physical_scene` example.

## Bare robot sessions

```sh
cargo run -p rm-simulator-physics --release --example robot_session --locked
```

`session::RobotScenario::new(faces, floor_height_m, step_duration, setup)` defines
an initial scene. Its repeatable setup recipe receives a fresh `WorldPhysics` at
time zero and installs models, shared geometry, payloads, policies and resolved
mechanisms with existing methods. `RobotSession::new(scenario)` owns that one
world. `step` advances explicit integration steps and returns ordered raw contacts;
`step_controlled` runs a controller before every tick. `step_with_targets` accepts
one tick's prescribed target endpoints. Drive, direct wheel torques and muzzle
fire use ordinary physical controls without match permissions or resource rules.

`robot_control_state` reads allocation-free pose/input/revision/defeat truth.
`stamp`, `robot_truth` and `ground_truth` expose privileged observations without
capturing a whole-world checkpoint; `robot_truth` includes cloned model and wheel
data for one robot. `lidar_scene_capture` uses session time;
`physics_mut` provides payload/mechanism control, geometry/model editing, carrier
trace hooks and event drains. Callers must keep stepping and synchronization under
the session's clock. Sensor schedules, interpolation and IMU state remain external.

`reset` transactionally rebuilds the initial recipe, including its payloads and
geometry. It resets time, IDs, events and traces; it discards later runtime edits.
The recipe must capture repeatable inputs and must not step the world. There is no
arbitrary physical checkpoint/restore API yet: body snapshots do not represent
policy history, touched-armor memory, all prescribed motion or solver continuation.
Reset caller sensor/controller state separately; do not interpolate across reset.

## Testing

`cargo test -p rm-simulator-physics --locked` runs the in-module tests in
`chassis/` and `projectile.rs` and the integration tests
`tests/projectile_retirement.rs` and `tests/robot_session.rs`. Coverage spans
chassis settling on the floor,
holonomic drive, motor limits, gimbal rate limiting and aim freezing, the shared
power budget, scoring-face acceptance, closing speed, projectile bounds and
retirement, and target-frame identity and ordering.

`lib.rs`, `chassis.rs`, `geometry.rs`, `motion.rs` and `projectile.rs` carry
doctests; the crate-level example launches from a muzzle and asserts one contact
over a 200-tick flight.

`examples/moving_armor.rs` creates a moving armor face, launches ten projectiles
and advances 1,000 explicit ticks without CAD, a window or a server:

```sh
cargo run -p rm-simulator-physics --example moving_armor --locked
```

[docs/physics-reuse.md](../../docs/physics-reuse.md) documents the headless
stepping contract, the restore procedure and optional rendering.


`mechanism::FoldingConfig` evaluates a prescribed folding arm and independently
aimed firing stage. `ChassisConfig::folding_hero()` provides the source-fitted
2026 prototype; its command holds a folded target and snapshots preserve the
actual angle. Moving collision proxies, muzzle and drawing poses share that
state. External-clearance checks stop the actuator without teleporting geometry;
this is not a motor-torque or source leg-linkage simulation.
