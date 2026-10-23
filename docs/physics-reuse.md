<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Reusing physics and armor motion

`rm-simulator-physics` depends on Rapier, serde and the standalone MID360 pattern
generator. It contains chassis
suspension and drive, prescribed gimbals, physical generic joints, projectile
flight, raw armor contacts, shared collision geometry and analytic armor motion. It does not depend on
world, gameplay, server, rendering, a CAD loader or host time. Its
[crate README](../crates/rm-simulator-physics/README.md) lists its modules and
permitted dependencies.

The library retains RoboMaster-specific physical profiles and target labels.
`Caliber` includes the existing nominal detection/damage lookup helpers for
source compatibility, but the physical step does not apply HP, buffs, activation,
detection intervals or match rules. The caller controls chassis disable state.
This is a reusable RM physical library, not a general physics-engine abstraction.

## Contents

- [Backend-neutral wheel model](#backend-neutral-wheel-model)
- [Source-described bodies and bindings](#source-described-bodies-and-bindings)
- [Concrete physical runtime](#concrete-physical-runtime)
- [Headless use](#headless-use)
- [Restore and prediction](#restore-and-prediction)
- [Optional rendering](#optional-rendering)

## Backend-neutral wheel model

`wheel_model::{WheelModel, WheelState, WheelDriveScratch}` runs the existing
ray-wheel plant without constructing a `WorldPhysics` or using Rapier types.
`WheelModel::from(&ChassisConfig)` copies only scalar motor, inertia, suspension
and friction parameters; callers can also construct the record directly.
Its module doctest demonstrates unsupported motor spin without a solver.

For each backend solver slice, the adapter supplies wheel-order ground rolling
speeds in m/s and grip limits in N through the reused `scratch.contacts` buffer,
held velocity targets or a borrowed slice of wheel-axis torques in N m, a duration
in seconds and an admitted shared motor budget in W. `integrate` updates borrowed
wheel state and returns mean motor allocation and consumed power. Its reused
`forces` buffer contains mean rolling traction in N. The same eight internal
motor slices, torque envelopes, wheel inertia, slip law, non-regenerative power
accounting and angle wrapping are preserved. `suspension_load_n` and
`roller_force_n` supply the spring/damper/bump-stop and free-roller equations.
Inputs must satisfy the documented finite/positive parameter and length contracts;
this low-level API does not replace session command validation.

The backend adapter still owns query filtering and contact acceptance, wheel
geometry and moving-hub velocity, contact frames and point velocities. It computes
compression/rate, calls the scalar suspension/roller model, and supplies rolling
grip as `drive_friction * load_n`; unsupported wheels receive zero grip. It
clears unsupported contact/compression and constructs contact presentation records.
The adapter admits defeat/actuation cuts and configured power ceilings, resolves
high-level body targets, and retains balance/jump, gimbal and folding behavior.
It applies rolling forces first, then each wheel's roller and suspension forces
at the original points and in the original order before advancing the solver.
It copies traction into contact records; the model updates contact slip.

The three existing contact/traction/motor vectors retain their capacities. Initial
use or a new largest wheel count can allocate; fixed-topology calls allocate
nothing in the model after warmup. No per-wheel copy or additional wheel-state
buffer is introduced. Parameters are a small stack record, not a cloned chassis
profile. This extraction establishes reuse, not a measured speedup.

A 2026-10-23 Talos smoke comparison used seven unpaced 10,000-tick samples of
three chassis (omni, mecanum and balancing), after 128 warmup ticks, including
queries and solver work. With the development profile (workspace opt-level 1,
dependencies opt-level 3), median tick time was 971.737 µs before and 990.941 µs
after extraction (about 2% higher); ranges were 957–1001 and 959–1020 µs.
Concurrent host work and one sequential comparison limit attribution: these
samples do not isolate the refactor's effect. Existing steady-state scratch
allocation behavior is preserved.

`chassis::WheelContact` remains a compatibility export with unchanged fields and
serde representation. Chassis snapshots, drive modes and restore semantics are
unchanged. This is an f64 ray-wheel model, not an articulated roller model or a
universal engine interface. The crate still links Rapier for its existing adapter;
a future PhysX adapter must supply equivalent contacts, scheduling and force
application and validate its solver differences independently.

Regression coverage includes six one-second trajectories recorded before
extraction from `8f4f44a` on Talos (omni, mecanum, low grip, landing, direct torque
and balancing). Body pose/velocities and wheel spin/rate/compression/traction
matched exactly on the reference machine; committed comparisons allow a small
platform math tolerance. Independent scalar tests cover suspension, free spin,
contact slip reaction, direct torques, shared/zero power and buffer reuse, without
creating a solver world. Existing adapter tests cover defeat, power cuts,
contact filtering, balance/jump and restore.

## Source-described bodies and bindings

`engine::model` now supplies immutable `RigidBodyDescription` and
`ColliderDescription` records. `ShapeDescription` supports local FLU spheres,
boxes (half extents), explicit convex point hulls and indexed surfaces. Descriptions retain explicit mounting
poses and `ContactMaterial` coefficients plus symmetric membership/filter bits;
no team/projectile semantics are inferred from the bits. The current Rapier
adapter uses its default average coefficient combination. Descriptions are
in-memory values, not a new asset file or serialization format.

`BodyKind` distinguishes fixed, position-driven kinematic and dynamic bodies.
Dynamic bodies require `Inertial`: total mass in kg, principal moments in kg m²
about the COM, and a body-local COM/principal-axis frame. Positive finite mass,
realizable principal moments and finite unit poses are validated. Collider
volumes/densities never add hidden mass; CAD-native cached inertia is not imported.
Dynamic nonconvex triangle surfaces are rejected explicitly. Basic joint and
motor contracts and physical assembly trees extend these bodies (see below). Complete robot packages, actuator
electronics, damping/CCD options and body checkpoint
continuation remain future work; existing robot chassis retain their model.

`physics::cooking::CookedRigidBody` retains the immutable source and shared
backend shapes. `install_body(BodyKey, &cooked, BodyState)` creates world-local
bodies/colliders with explicit initial world FLU pose and COM velocities. Fixed
and position-driven kinematic installation requires zero initial velocity.
Generic bodies collide with existing participants and appear in geometry/sensor
captures. Fixed bodies alone permit idle stepping; dynamic/kinematic bodies keep
the ordinary integration loop active.

`BodyKey(u64)` is a caller-chosen key within the runtime, not an asset hash or
backend index. An opaque `BodyBinding` brands one installation with its runtime
and a non-reused generation. Wrong-world and removed bindings fail before
mutation; reinstalling the same key cannot revive an old token. Runtime brands
are process-local metadata and do not enter physical calculations/checkpoints.
The identity counter never wraps (exhaustion panics at world construction);
per-world generation exhaustion returns an admission error before installation.
Moving an owned world preserves bindings; recreating/resetting it invalidates them.
Use `body_binding(key)` to rebind after a successful recipe reset. The live table
retains only installed bodies, and generation counters are never recycled.

`body_state(binding)` copies privileged motion; `body_description(binding)` borrows
source metadata without cloning collider buffers. `set_body_velocity` replaces
dynamic COM linear/angular velocity in world axes. `set_body_wrench` replaces a
held world-frame COM force in N and torque in N m, remaining active until replaced
or removed; zero clears it. `set_body_endpoint` interpolates a kinematic body to
its endpoint across the next tick's ordinary solver slices and holds it there.
Repeated endpoints replace the target while retaining first-request order.
It is not a teleport API. These trusted runtime operations introduce no player
command path, controller assistance, networking, match policy or wall clock.

Rolling LiDAR brackets now also validate runtime brands, preventing interpolation
between separate worlds even if other handles/shapes coincide. Native body motion
continues through the existing per-emission interpolation. Sensor/controller
histories still require caller reset, as do ordinary RobotSession recipes.
Generic bodies currently belong to bare runtimes/RobotSession recipes; match
checkpoints and wire codecs do not serialize these new descriptions or bindings.

Run the complete headless example:

```sh
mbx run -p rm-simulator-physics --locked --example body_runtime
```

It cooks an explicitly described cube once, drops it onto the catch floor through
the ordinary runtime, then reconstructs the recipe and reacquires its binding.
No setup worker, CAD file, match or renderer is involved.

`engine::model::{JointDescription, JointLimits, JointMotor, JointState}` describe
fixed/revolute/prismatic constraints and bounded spring/damper commands without
backend handles. `install_joint` connects live body bindings and returns a
world-scoped `JointBinding`, reacquired after reset using its `JointKey`.
Removing a body also invalidates its attached joint bindings. The
[jointed mechanism guide](jointed-mechanisms.md) defines attachment frames,
coordinate units, held motor/effort replacement and the physical two-axis turret
fixture. Current game gimbals and arms remain prescribed; the
[migration plan](physics-migration.md) separates those later modeling changes
from behavior-preserving runtime extraction.

For complete rooted physical trees, `AssemblyDescription` retains source link and
joint IDs independently of live keys. `CookedAssembly` prepares shared resources;
`install_assembly` stages placed native state and validates all keys before any
world mutation. `AssemblyBinding` maps source parts to live body/joint tokens,
and `remove_assembly` preflights the whole record. The turret guide demonstrates
independent instances and recipe reuse. These physical descriptions do not yet
include robot visual/sensor references or a CAD/URDF importer.

The [convex collider guide](convex-colliders.md) describes explicit hull-envelope
semantics, backend rejection, shared resources and parity against the existing
chassis deck/bumper shapes. Dynamic hulls retain caller-supplied inertia, and
captures/debug triangle expansion include their native boundary geometry.

`BodyPolicy` describes gravity scale, world-axis constraints, damping, sleeping
and CCD. Existing defaults remain unchanged. `ColliderPolicy` adds explicit
coefficient combination and optional mass/density contributions. These are
accepted only with `BodyKind::DynamicFromColliders`, which requests backend
shape aggregation; explicit `BodyKind::Dynamic(Inertial)` keeps zero-mass
colliders. Neither mode imports CAD density or silently estimates real mass.
The chassis adapter now uses these same source/cooked policies while preserving
its native insertion order and deferred mass refresh.

## Concrete physical runtime

`physics::runtime::WorldPhysics` owns one interacting Rapier scene. Root and
`projectile::WorldPhysics` imports remain aliases of the same type. Its explicit
`step_into(prev_ns, frames, contacts)` appends raw ordered contacts into caller
storage; `step` retains the owned-output convenience API. The caller supplies
simulation time and target endpoints. This is the existing concrete runtime,
not a generic solver interface or an additional scheduler.

Physical sessions use the retained-output path directly, including
`RobotSession::step_with_targets_into` for moving targets. Match adapters keep
their existing pre/post integration rule barriers. See the
[architecture guide](engine-architecture.md#implemented-concrete-physical-runtime)
for lifecycle, compatibility and error/reset limits.

Read-only participant/collider references and optional bounded contact samples
are described in [Physical contacts](physical-contacts.md). These physical
identities do not imply gameplay ownership or generic command authority. The
RoboMaster adapter retains its independent scoring state and ordered outputs.

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
3. Supply explicit `MotionUpdate`s for instance scenery, or resolved
   `MechanismState` for legacy match equipment.
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

Independently moving instances use `motion::{MotionId, MotionFrame, MotionUpdate}`
and `WorldPhysics::add_motion_mesh`/`sync_motions`. Their IDs have caller-owned
instance/channel namespaces and no inferred team driver. Partial update batches
validate duplicate/unknown IDs, finite coordinates and translation bounds before
moving anything. Legacy `sync_mechanisms` skips explicit instance channels.
Geometry captures retain IDs, shared shapes, rest frames and current poses;
`StaticGeometry::at_motions` resolves another coordinate without double motion.
`try_insert_geometry` rejects installed ID overlap before mutation while allowing
several incoming shapes sharing one channel. Use disjoint instance namespaces for
independent captures. See [composable scenarios](composable-scenarios.md) for
source mappings, complete target tables and recipe reset contracts.

Time arguments are simulation time. The caller owns monotonicity, overflow checks,
fixed-tick scheduling and the motion state associated with each checkpoint.
Lengths and rotations use FLU metres and normalized wxyz quaternions. Existing
physical assumptions, projectile lifetime/capacity and launch validation remain
unchanged by extraction.

## Standalone arena components

`physics::arena` provides geometry without a complete `Field` or native match:

- `ZoneFootprint` borrows a polygon and vertical band. It reports occupancy;
  control capture, traversal history and buff effects remain gameplay authority.
  RMUC control, supply, Vantage and terrain-card observations use this same query.
- `OutpostModel` retains the source pivot, tilted axis and detector frames. Callers
  supply the rotor angle and map detectors to target identities. RMUC uses the
  same pose calculation while keeping spin-up, stopping, HP and damage rules.
  `world::rmuc_2027::{Tower, Detector}` remain compatibility re-exports with the
  same serialized fields.
- `TunnelModel` produces three boxes (roof and side walls) for an open-ended
  rectangular test passage. Placement and dimensions are explicit; no official
  arena dimensions are inferred. It is an approximate fixture, not a replacement
  for verified CAD, tunnel route bindings or RFID traversal rules.

Run `cargo run -p rm-simulator-physics --example arena_components --locked` for
an independent tunnel, occupancy and rotating detector example in `RobotSession`.
Detectors remain the existing physical armor housings: this example does not
load an Outpost visual shell or model its complete structural collision.

A standalone model supplies geometry, target motion and observations. A match
adapter associates these with team/card identities and submits observations to
its existing authority. It must not instantiate a separate buff/HP scheduler.
In particular, a tunnel's passage geometry and its RFID cards are independent;
a robot fitting through the opening does not automatically earn a traversal.
Reuse the existing rm-map-tools export pipeline for complete geometry:
`cad_partition.py` preserves source bodies/shells and placements;
`extract_entities.py` emits separate visual/collision GLBs; and
`export_articulated.py` exports semantically bound rigid links and joints to
SDF/URDF/USD. Work partitions describe source ownership, not automatically proven
motion or scoring roles. Articulated exports carry kinematics, not calibrated
mass/inertia or actuators. These tools require no simulator dependency on the
sibling repository; consume their versioned output packages.

The assets crate already verifies package checksums and semantic bindings, and
`collision_mesh::load_glb_selected` / `load_glb_filtered` can select primitives
and joint subtrees while preserving ancestor transforms. Server layout already
uses those readers for fixed/moving geometry. `assets::components` now exposes that composition outside server layout:
load one verified asset, retain its asset-local fixed/moving definition, and place
or install selected instances without a complete arena. Server terrain loading
uses the same builder. Native source-body loading parses the collision GLB once
for both partitions; semantic loading retains its existing primitive/subtree
filters. `ComponentModel::mechanisms` preserves binding-major, placement-minor
insertion order, matching the previous arena composition.

```sh
cargo run -p rm-simulator-assets --example outpost_component --locked -- field-2027
```

This loads one complete approved source Outpost, including fixed shell, moving
rotor geometry and three detector faces, into `RobotSession` at its existing
placement. It sets a caller-prescribed 0.5 rad rotor angle, advances and resets.
No server, complete field or native match is constructed. The visual file and
its checked semantic metadata remain available through the selected `CadAsset`;
the physics builder does not draw or reinterpret their materials.

Pass exported `CadAsset::placements` roots to `ComponentModel::placed`. To author
a new placement, use `assets::math::gltf_root_pose`, whose basis convention matches
the GLB reader; plain `Pose::at` is not an interchangeable root placement.
`PlacedComponent::install` consumes placed triangles into `WorldPhysics`;
`RobotScenario` can retain a cloneable placed definition for episode reset.
Caller target poses, detector masks and prescribed mechanism coordinates must
remain consistent. No HP/buff authority, automatic rotor scheduler or calibrated
inertial properties are created. Source geometry/verified imports remain in
rm-map-tools and assets; future rule-state extraction should preserve the single
native match clock.

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

## Complete arena without a match

The assets crate provides `arena::PreparedArena::load(&cad)` and
`PreparedArena::scenario(targets, step_duration, setup)`. The recipe installs
shared cooked source geometry, removes the catch plane, preserves projectile
bounds and runs caller setup on every construction/reset. Supply physical target
frames and mechanism coordinates explicitly; scenery does not create scoring
rules. This remains a repeatable initial reset, not a bare-world continuation
checkpoint. Example:

```sh
cargo run --locked -p rm-simulator-assets --example arena_session -- field-2027
```

Source mechanical arm calibration and FK are available directly as
`physics::assembly_model::{Model, Jaw, Auxiliary, RigPose}`. Their existing world
paths remain aliases. They require neither an assembly attempt nor a match.

## Reusable batch query boundary

`queries::RayQuery::new(origin_m, direction, max_distance_m)` validates and
normalizes a metric ray once. Its public contract contains only immutable FLU
arrays and metric scalars, with no solver handles. Reuse the request slice across
captures and call `StaticGeometry::cast_rays_into(&rays, &mut hits)` with equally
sized output. Every input has one output slot, including `None` misses; mismatched
lengths fail before modifying output. Empty batches are valid. Caller-owned
slices make capacity explicit and avoid output allocation; first use can still
initialize the capture's existing shared bounds cache. This adds no acceleration
algorithm or claimed speedup. It creates a coarse operation that future query
implementations can implement without a per-ray external call.

`queries::RayHit` retains the existing `geometry::RayHit` compatibility path.
Its shape index is an ordinal within the supplied capture, not an entity or
persistent shape identity. Requests contain no cached hits or geometry handles,
so the same batch can query another scene without stale results. They contain no
scheduler or timestamp; callers retain the capture's time/generation context.
CPU request/output slices are not device buffers or a stable FFI ABI. Rolling
LiDAR remains on its validated per-return motion path; instantaneous ray batches
do not replace motion-aware sensor acquisition.
