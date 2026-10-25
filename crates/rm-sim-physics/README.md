<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# rm-sim-physics

`rm-sim-physics` is a reusable, gameplay-free Rapier library: wheel and body
dynamics, projectile flight, raw armor contacts, shared collision geometry and
prescribed mechanism motion. It knows no match rules, CAD package, socket, Bevy or
host clock; callers supply target endpoint poses and resolved mechanism state and
step it once per explicit tick. It sits under the world crate and depends on no
simulator crate except the solver-free `rm-sim-engine` core.

## Modules

| File | Owns |
|---|---|
| `src/runtime/session.rs` | Generic `PhysicalSession`: explicit time and transactional recipe reset over a bare `PhysicalRuntime`, with typed setup and no participant roles. |
| `src/session.rs` | Caller-paced `RobotSession` over one `WorldPhysics`, repeatable `RobotScenario` reset, addressed commands, compact truth and explicit sampling schedules. No match authority or arbitrary checkpoint restore. |
| `src/control.rs` | Validated held drive modes and per-wheel motor torque requests, independent of a transport or controller scheduler. |
| `src/sensors.rs` | Prepared ideal planar/multi-layer LiDAR, caller-stamped instantaneous acquisition and reusable output buffers over explicit geometry captures. |
| `src/sensors/imu.rs` | Mounted gyroscope/accelerometer on the chassis or actual gimbal frame, derived from opt-in solver-substep carrier traces, with restorable state and seeded noise, bias walk and saturation. |
| `src/sensors/rolling.rs` | Mounted procedural MID-360 acquisitions with interpolated robot/obstacle motion, self-occlusion, timestamps and seeded range noise/dropout. |
| `src/wheel_model.rs` | Backend-neutral scalar suspension/roller, wheel inertia, motor/slip and shared-power equations over explicit parameters, borrowed state/torques and reused buffers. No solver handles or queries. |
| `src/assembly_model.rs` | Source arm/gripper/Core mechanical descriptions and forward kinematics, without attempts, rewards or playback. |
| `src/chassis.rs` | Compatibility facade for chassis configuration, commands, snapshots and borrowed telemetry; existing public paths remain unchanged. |
| `src/chassis/model.rs` | Physical descriptions, presets, validation and evaluated model geometry, including conversion to `WheelModel`; no solver handles or stepping. |
| `src/chassis/backend/` | Rapier body/collider construction (`construction.rs`), ordered wheel queries and forces (`dynamics.rs`), prescribed folding/gimbal adaptation (`prescribed.rs`), controls and observations (`mod.rs`). |
| `src/cooking.rs` / `src/cooking/` | Shared cooked mesh/body shapes and assembly resources retaining solver-free source descriptions. |
| `src/runtime/bindings.rs` | Explicit body keys, world-scoped bindings, source/state reads and generic physical control. |
| `src/runtime/directional.rs` | Opt-in regularized tread/roller friction against fixed terrain, with load-lag and contact-patch limitations documented in the wheel-contact guide. |
| `src/runtime/observations.rs` | Read-only native participant/collider identities and opt-in bounded physical contact samples, independent of scoring and command bindings. |
| `src/runtime/joints.rs` | Generic physical joint installation, world-scoped bindings, scalar state, motors and held effort. |
| `src/runtime/assembly.rs` | Transactional prepared-assembly installation/removal, placement and source-to-live binding maps. |
| `src/runtime.rs` | `PhysicalRuntime`: generic native scene, bindings, integration slices and physical observations. `projectile::WorldPhysics` composes its RoboMaster participant adapters over that scene. |
| `src/projectile.rs` | `Shot`, `Caliber`, `Contact`, `TargetFace`, `TargetFrames`, `StaticGeometry` and `ProjectilePolicy`; participant adapters and compatibility world exports. |
| `src/queries.rs` | Immutable validated ray batches and capture-local hits, with no public solver handles; callers retain output storage. |
| `src/geometry.rs` | Shared collision captures and read-only clearance queries consumed by the world facade. |
| `src/motion.rs` | Prescribed rotor and rail motion (`RotorMotion`, `SinusoidalMotion`, `MechanismState`) and fitted armor geometry; rune pose helpers live under `motion::rune`. |

`lib.rs` re-exports the engine clock contract (`DEFAULT_TICK_NS`, `tick_ns()`,
`StepDuration`) and FLU `Pose`, and retains `Team`. Session step counts use the
instance duration; low-level runtime step arguments are starting nanoseconds.
The default is 128 Hz (7.8125 ms per tick). Standalone callers can select immutable
`StepDuration` values, including 1000 Hz, through `WorldPhysics::with_step_duration`.
Use the instance duration to advance timestamps; `tick_ns()` returns the default. The step applies no HP, buffs, activation or
detection intervals; the world crate owns those decisions.

## Dependencies

`rm-sim-engine` (shared contracts and wheel laws), `rapier3d-f64`, `serde`, `glam` for source rig FK (the same locked version as
world), and the standalone `mid360-sim` pattern generator. The
crate never depends on `rm-sim-world`,
`rm-sim-gameplay`, `rm-sim-server`, `rm-sim-render` or Bevy,
and it never reads host time. Standalone callers can use `session::RobotSession`
without `rm-sim-world`; match composition continues to use the same physical
plant. `rm-sim-app` also uses it for the `physical_scene` example.

## Bare robot sessions

For a generic mechanism, use `runtime::session::PhysicalSession` over the bare
runtime. The [controller bench](../../docs/controller-bench.md) installs a small
XY positioning robot, applies caller-side force feedback, steps and resets:

```sh
cargo run -p rm-sim-physics --release --example controller_bench --locked -- 1000
```

It needs no CAD, display, server or match, and shares the game's physical runtime.
The recipe retains prepared shapes rather than cooking them again on reset.
Below, `RobotSession` adds the existing RoboMaster physical participant facade.

```sh
cargo run -p rm-sim-physics --release --example robot_session --locked
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

`cargo test -p rm-sim-physics --locked` runs the in-module tests in
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
cargo run -p rm-sim-physics --example moving_armor --locked
```

[docs/physics-reuse.md](../../docs/physics-reuse.md) documents the headless
stepping contract, the restore procedure and optional rendering.


`mechanism::FoldingConfig` evaluates a prescribed folding arm and independently
aimed firing stage. `ChassisConfig::folding_hero()` provides the source-fitted
2026 prototype; its command holds a folded target and snapshots preserve the
actual angle. Moving collision proxies, muzzle and drawing poses share that
state. External-clearance checks stop the actuator without teleporting geometry;
this is not a motor-torque or source leg-linkage simulation.

Shared poses/time, query records, drive controls and wheel laws now have one
definition in engine core. Existing physics paths re-export them. The chassis
conversion remains here, and engine core contains no solver/game dependency.

Triangle surface preparation separates `engine::model::TriangleMesh` (available
through `cooking::TriangleMesh`) from `cooking::CookedTriangleMesh`. The latter
retains source geometry and shared Rapier shape storage. Cook once and install
into multiple worlds with `WorldPhysics::add_cooked_static_mesh`; world collider
handles and state stay independent. The owned-buffer `add_static_mesh` path keeps
its existing behavior and does not retain duplicate source data. See the
[architecture guide](../../docs/engine-architecture.md#implemented-sourcecooked-surface-boundary)
for scope and remaining model-description work.

`PhysicalRuntime` owns the native scene and generic construction/integration.
`WorldPhysics` composes RoboMaster participants over that same runtime; the
`runtime::WorldPhysics` compatibility alias and root/projectile paths remain. `step_into` appends
raw contacts into retained caller storage. `RobotSession::step_into` and
`step_with_targets_into` use it directly. Sessions/match adapters retain clock,
reset and rule scheduling ownership; no extra world or scheduler is created.

`cooking::CookedRigidBody` prepares engine body descriptions once; runtime
`BodyKey`/`BodyBinding` APIs install and address world-local instances without
exposing native handles. Explicit mass/inertia, held world wrenches, dynamic
velocity and interpolated kinematic endpoints are supported. Bindings reject
other worlds/removal/reset; reacquire a key after recipe reconstruction. See
[source-described bodies](../../docs/physics-reuse.md#source-described-bodies-and-bindings)
and `examples/body_runtime.rs` for usage and current unsupported capabilities.

## Explicit convex bodies

`engine::model::ConvexHull` retains immutable source points in local FLU metres;
`ShapeDescription::ConvexHull` requests their convex envelope explicitly.
`CookedRigidBody` uses the native Rapier convex-hull path on those exact points,
rejecting geometry without a finite three-dimensional hull rather than substituting
a box or triangle surface. Collider-local placement and contact parameters remain
explicit. Cooking supplies no density, mass, COM or inertia policy.

`examples/convex_body.rs` cooks a one-link generic assembly once, installs it in a
`RobotScenario`, settles it on the floor and resets with fresh runtime bindings.
Its cube has chosen 0.2 m sides and explicit uniform-solid 2 kg mass and principal
inertia; these fixture values are not inferred from CAD:

```sh
nix develop --command mbx run -p rm-sim-physics --example convex_body --locked
nix develop --command mbx test -p rm-sim-physics --test convex_bodies --locked
```

`tests/convex_bodies.rs` compares the existing default omni chassis deck extrusion
and every bumper prism with direct Rapier hulls using exactly the same points.
Coverage includes per-face ray distances/normals, exported triangles, bounds and
volume, plus rotated/translated collider-local mounts and body placements. The
generic fixture also checks dynamic settling, projectile momentum transfer,
independent worlds and reuse through reset. This establishes shape parity; it
does not establish chassis force or mass-policy parity. Chassis construction,
wheel dynamics, game mechanisms and native snapshots keep their existing paths.

## Physical joint fixture

Generic source-described bodies can be connected by fixed, revolute and prismatic
joints, with local +X anchor-frame axes, limits, motors and held effort. Runtime
joint bindings are reacquired by logical key after recipe reset, like body
bindings. The standalone yaw/pitch turret uses explicit mass, COM and inertia and
physical revolute joints. Its engine `AssemblyDescription` keeps source link/joint
IDs and assembly-local initial states separate from runtime body/joint keys.
`CookedAssembly` prepares that source once; `install_assembly` transactionally
creates live instances at caller placements through explicit `AssemblyKeys`.
The scenario reuses the prepared resource on reset and reacquires live bindings.
The integration test also installs two independently actuated instances from one
resource with translated/rotated placement and disjoint runtime keys:

```sh
nix develop --command mbx run -p rm-sim-physics --example jointed_turret --locked
nix develop --command mbx test -p rm-sim-physics --test jointed_turret --locked
```

See [jointed mechanisms](../../docs/jointed-mechanisms.md) for frames, units,
reset behavior, tests and limitations. Current game mechanisms remain prescribed;
this fixture introduces no joint CAD import or checkpoint integration.

## Physical participant and contact observations

`runtime::ParticipantBinding` and `ColliderBinding` identify all native bodies
and colliders, including chassis, projectiles and scenery. `participants()` and
`colliders()` enumerate lazily; `participant_state` and `collider_state` return
copied privileged truth in world FLU. Collider state includes its actual parent:
parentless scenery and prescribed colliders have `None`. `body_participant`
resolves a source-model `BodyBinding` without giving observation tokens command
or wrench ownership. Tokens are opaque, process-local and not serialized;
foreign worlds, removal/slot reuse and scenario reset invalidate state access.
Reacquire bindings after reset. Caller-owned identity maps may retain old tokens.

`enable_contact_observations(capacity)` opts into a positive per-world-tick
record budget. The default has no observer allocation or contact-pair scan.
`physical_contacts()` borrows records and `physical_contacts_dropped()` reports
capacity overflow for the latest tick. `drain_physical_contacts(&mut output)`
appends while retaining observer storage and the overflow count. The next step
clears records/count; reconfiguration clears them, and disable releases storage.
Zero capacity fails before mutation. Failed steps are not transactional: preflight
errors can preserve the old buffer, while later errors can leave it cleared.

Each `PhysicalContact` retains both collider identities and their native parent
associations at capture, even if a body is subsequently removed. Records follow
native pair encounter order, with one deepest contact per active pair per solver
slice. Slice numbers start at one; `time_ns` is tick start plus
`floor(step_ns * substep / substeps)`, and `tick_end_ns` is the ordinary checked
tick endpoint. The observer owns no clock and changes no scoring/latch/retirement
semantics. These samples are neither begin/end events nor hits or measurements.

Geometry is sampled immediately after native integration, before prescribed fold
collider synchronization: cached local points are transformed by current collider
poses, while normal and signed separation come from cached native constraints.
This does not promise a fresh coherent narrow-phase projection or incoming impact
velocity. `engine::contacts::ContactGeometry` carries world points, normal from
side 1 to side 2 and signed separation, without backend or gameplay identities.
See `tests/physical_contacts.rs` and the runnable observation doctest.

## Prepared domain participants

`chassis::PreparedChassis` and `projectile::PreparedPayload` share immutable source
and cooked resources across installations and recipe resets. Native handles,
controls, wheel/attachment state and contact histories remain per instance.
Projectiles and scoring targets use the same cooked body builders; the game facade
retains its bounded projectile preparation cache and domain scheduling. Parentless
scenery shares the runtime factory without losing filtering or motion metadata.
See [the physical runtime guide](../../docs/physical-runtime.md).

The `wheel_suspension` example is an isolated closed-loop suspension bench using
recovered EGA rocker/mount dimensions and explicitly provisional mass, stroke and
spring/damping choices. It emits actual solved poses and closure residuals to CSV;
see [suspension recovery](../../docs/suspension-recovery.md#provisional-isolated-physical-fixture).
It does not replace the game's ray-wheel model or certify native CAD mates.

The `wheel_contact` example extends that linkage with a guided dynamic frame,
a driven wheel and floor/ramp contacts in the same runtime. It compares loading,
traction and obstacle response against flat and zero-friction controls; see
[guided wheel contact](../../docs/wheel-contact.md) for assumptions and commands.

Directional wheel contact is an experimental standalone option through
`runtime::DirectionalFriction` and
`runtime::PhysicalRuntime::set_directional_friction`.
It retains native normal support and couples bounded tangential damping to the
native joint solver. See [the wheel-contact guide](../../docs/wheel-contact.md#directional-contact-experiment)
for coefficients, the free-lateral test stand, reproduction and the unresolved
5 mm surface-transition stall. It is not enabled in the game.

The `whole_chassis` example attaches four reusable linkage/wheel modules to a
free six-DOF sprung body in one runtime. Axle motors produce translation/yaw;
flat support, COM load transfer, braking/coasting and asymmetric small-track
response are tested at 128/1000 Hz. The full robot layout and upper mass/inertia
remain assumed. See [the chassis experiment](../../docs/whole-chassis.md).
`PhysicalRuntime::directional_normal_load_n` exposes the preceding-slice contact
budget as privileged diagnostic truth, not a simulated load sensor.

The `servo_response` example exercises direct joint torque and bounded PD
response against known inertia and external loads. `joint_applied_effort` reads
last-slice actuator output, independently of joint position/velocity; it does
not measure constraint reaction torque. See [torque and servo response](../../docs/jointed-mechanisms.md#torque-commands-and-servo-response).
