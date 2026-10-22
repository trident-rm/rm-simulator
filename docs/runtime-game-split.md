<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Runtime and game separation design

Status: sessions, the backend-neutral wheel model, standalone equipment presets,
borrowed structure transitions, composable scenarios, compact telemetry and bounded CPU rollout batches are implemented
(2026-10-23). Further model and solver extraction below remain proposed; the
Rapier dynamics backend is unchanged. This design extends the [current architecture](architecture-refactor.md)
and [embedding interface](simulation-embedding.md) so a second physical backend
can be evaluated without making the multiplayer app its integration layer.
The current AGENTS.md engineering contract continues to govern implementation.

The reusable product is a robot simulation runtime, with RoboMaster match rules
composed above it. The driving game, vision experiments, navigation adapters and
training environments consume that composition or the bare runtime as needed.
Keep the code in one workspace initially. Repository separation and a stable
external SDK should follow a working second consumer/backend, avoiding duplicate
asset importers, robot models and rule implementations during migration.

## Implemented first step

`physics::session::RobotSession` owns a single `WorldPhysics`, explicit time,
ordinary physical commands, compact truth and sensor access. `RobotScenario`
provides transactional initial-scenario reconstruction. Arbitrary bare physical
checkpoint continuation is deferred until its complete state can be captured;
body snapshots omit projectile policy/contact history and other continuation data.
Caller sensors/controllers retain their own state and reset responsibilities.

`world::session::MatchSession` is the explicit name for the existing Field facade;
`MatchCheckpoint` preserves its existing match restore behavior and documented
solver residuals. `Session` and `Checkpoint` remain compatibility aliases.
Physical commands, truth and timing primitives have a single definition in the
physics session and are re-exported by the world session. Both retain controller
command storage across calls.

Field still owns its existing WorldPhysics directly; it does not yet wrap a
RobotSession. The two sessions use ordinary existing physical paths without
stepping any body twice. Model/backend descriptors, moved server construction,
GPU buffers, a second backend and multiplayer backend negotiation remain later
migration steps. This first step introduces no new crate or runtime dependency.

The first model extraction now lives in `physics::wheel_model`: scalar
suspension/roller laws and the wheel inertia/motor/slip/shared-power integrator,
with explicit parameters, borrowed wheel state/torques and reusable scratch.
`physics::chassis` adapts Rapier queries and forces to it. There are no backend
handles in the model, no second backend or generic engine trait, and no changes
to physical equations, force/substep ordering, snapshots or protocol. See the
[wheel-model contract](physics-reuse.md#backend-neutral-wheel-model) for backend
responsibilities and its scope. Other concrete moves below remain future work.

## Standalone arena components

The first field-feature boundary is `physics::arena`: borrowed occupancy
footprints, source Outpost detector frames/rotation and an explicitly approximate
three-box tunnel fixture. Native RMUC adapters use these geometry calculations;
team ownership, control timers, traversal history, buffs, HP and rotor schedules
remain in the single gameplay authority. See [physics reuse](physics-reuse.md#standalone-arena-components)
for the runnable `arena_components` example and its source-CAD limitations.

This establishes reusable models and observations, not a plugin scheduler.
Standalone gameplay experiments can drive existing pure rule modules separately;
full matches continue to advance all effects through their ordinary clock.
Source partitioning and articulated export already exist in rm-map-tools;
verified package and collision-subtree readers already exist in the assets crate.
Reuse those outputs and readers. `assets::components` now composes fixed/moving models outside server layout,
and the live terrain loader uses that builder. A source Outpost can be installed
in `RobotSession` without a match. Borrowed `gameplay::rmuc_2027::equipment::{Outposts, Bases}` now share structure
transitions with the live match, and standalone Rune callers reuse its existing
pure authority. These views keep state caller-owned and add no second scheduler
or checkpoint format. See [equipment rule modules](equipment-rule-modules.md).

## Ownership

These are logical modules, not a requirement to create a crate for every row.
Retain existing crate names and compatibility exports where they remain useful.

| Layer | Owns | Excludes |
|---|---|---|
| Runtime contracts | Explicit time, stable entity/link/joint/shape identities, physical model descriptions, actuator commands, measured state, contact/query records, backend capabilities | Match roles, HP, referee permissions, Bevy, transport |
| Robot models | Mass/inertia, collision geometry, wheel/suspension/motor/slip laws, actuator limits, mounting frames, physical posture mechanisms | Ammo budgets, legal deployment slots, team victory |
| Backend implementation | Solver bodies/handles, cooked shapes, native articulation/contact state, backend queries and resource lifetime | Why a match grants power, damage policy, UI |
| Runtime session | Single physical-world owner, explicit stepping, prescribed physical motion, sensor sampling hooks, physical checkpoint/reset | Wall-clock pacing, match phase, pilot admission |
| Sensors | MID360 pattern/cadence, rolling acquisition, IMU sampling, noise and measurement timestamps over coherent runtime state | Truth presented as measurement, ROS/DDS publication, Nav2 |
| Assets | Checksums/provenance, parsed source hierarchy, geometry and articulation descriptions | Live world mutation, backend handles, competition authority |
| RoboMaster rules | Existing gameplay authorities, timers, heat/HP/economy/control/assembly/Rune/victory and legal rosters | Collision integration or drawing |
| Match composition | Runtime plus rule authority, source/zone bindings, physical-contact scoring, defeat/power effects, match checkpoint/reset | Network queues, window and host time |
| Planning/controllers | IK/path search, navigation and optical aim/sweep consuming explicit inputs and yielding commands | Match permission decisions, direct access to solver internals from UI/ROS |
| Host/transports | Seats/leases/ownership, command admission, real-time pacing, reliable ordering, codecs, HTTP/UDP | Independent rules or a second physical world |
| Presentation | Passive scene drawing, game HUD/menus/input, camera and recording, graphics preferences | Advancing physics/rules |
| Training/robotics adapters | Task/reward/termination, action cadence, observation selection, bounded environment workers, Python/ROS/robot interfaces | A copied competition engine or hidden truth in sensor inputs |

The robot-model layer can remain RoboMaster-oriented. A backend-neutral boundary
does not require turning this project into a general-purpose game engine.
The existing rm-simulator-gameplay crate remains the independent rules engine;
rm-simulator-world should become the match composition, rather than being renamed
wholesale into a generic runtime while retaining its match-specific fields.

## Concrete moves from the current code

`physics::wheel_model` now separates the reusable wheel/actuator equations
from `physics::chassis` Rapier bodies, colliders and queries. Continue extracting
other model computations only when the next consumer needs them. Preserve the existing equations,
force order, motor limits and suspension semantics in the first backend comparison.
A ray-wheel model and an articulated roller-wheel model are different plants;
benchmarking them as interchangeable would confuse model changes with backend gains.

`physics::projectile` combines dynamics with caliber labels, retirement policies,
target identity and contact detection. Keep physical drag, CCD and contact state
in the runtime/backend. The match supplies projectile profiles, maps opaque shape
labels to armor identities, and applies detection intervals/damage. Preserve the
existing public labels through compatibility adapters during migration.

`physics::motion`, mechanism trajectories and manipulator FK/IK contain reusable
physical/kinematic tools. Core assembly permissions, stages, LEDs and rewards stay
in match rules/composition. Collision-aware planning operates on an immutable
query scene; the match defines the task and the host owns its bounded worker.
Planning output is a command trajectory, never a hidden alternative world step.

`world::energy_unit` should separate a physical payload/grip attachment from its
match-specific deployment stock, spent state and scoring lifecycle. Release,
attachment and collision response belong to runtime physical state; the match
says whether a Unit may be acquired or spent.

`assets` currently depends on gameplay and physics. Split source parsing and
backend-neutral geometry descriptors from competition-specific binding validation
inside the existing crate first. Backend cooking consumes descriptors after
verification. An asset manifest must never require a live physics backend merely
to inspect it. Visual, sensor and dynamics meshes remain independently selectable
with their source hashes and approximation provenance preserved.

`server::layout` currently constructs terrain and a Field; `server::assembly_rig`
contains reusable source rig setup and planning; `server::prediction` includes
whole-match reconstruction/replay. Move construction/model loading toward assets
and match composition, and separate pure replay from transport anchors, timing
and acknowledgment policy. A bare simulation or match benchmark must eventually
load a verified arena and control a robot without linking GNS, HTTP or Bevy.
Keep layout compatibility wrappers until existing callers migrate.

`app::scene` should remain an adapter from match/runtime observations into the
renderer contract. Share immutable robot appearance descriptions and reuse pose
buffers; do not require the renderer to understand a PhysX handle or Field.
SceneState is a presentation record, not a complete checkpoint or a sensor output.

The built-in Sentry planner/controller remains optional. Its geometry query and
command interfaces can be reusable; operator map routing and team-visible contact
policy belong to the game/host. A training adapter can select a different policy
without changing the plant or granting an untrusted client direct runtime access.

## Execution and data contracts

A runtime owns exactly one authoritative dynamics backend for interacting bodies.
Do not step a robot in PhysX while also stepping its contact partners in Rapier
and copying poses between them. A separate query acceleration structure is fine
when derived from the same verified geometry and timestamped authoritative poses;
it is a measurement/query representation, not another dynamic simulation.

Use coarse operations across the backend boundary: load/cook a scene, spawn or
remove a batch, apply buffered controls/forces, query rays in a batch, advance a
physical interval, read compact observations and drain ordered contact records.
Avoid an FFI call or implicit synchronization for each body/property. Lifecycle
changes occur at explicit step boundaries, with stable IDs/generations mapping to
backend-local handles. Query result ordinals are not persistent entity IDs.

Separate immutable model data from changing state. Arena mesh/index buffers,
robot topology, joint names and physical parameters are shared by reference;
body/link poses, wheel rates and contacts occupy reusable typed buffers. Revisions
invalidate caches on geometry replacement, placement, topology changes or model
conversion. Capacities and overflow status are explicit: never silently discard
contacts, rays or articulation state because a fixed GPU buffer is full.

Support a CPU observation path first and optional device-resident batches later.
A device buffer needs declared element layout, device/context identity, ownership,
completion event and lifetime; a borrowed CPU slice cannot stand in for it.
GPU memory access remains behind the backend/adapter boundary. No pointer-based
sharing is assumed across CUDA, wgpu, Python or another process. Versioned schemas
and explicit synchronization are required even on unified-memory machines.

The match retains its current order: gather physical observations; advance rule
and mechanism state; integrate the physical tick; score contacts; apply resulting
power/defeat/mechanism changes before the next tick. Preserve the implementation's
exact ordering while extracting it. GPU substeps may contain the custom wheel
model without CPU round trips. Multiple matches may advance together, each with
its own rules, stable identity namespace and step duration. A GPU batch must not
bypass per-tick rule effects just to run several match ticks without a host barrier.
Bare physical sessions can batch longer intervals when their caller contract allows.
Action repetition for PPO never means skipping integration or intermediate rules.

The simulation clock stays caller-controlled. A real-time host paces it; a rollout
runs unpaced; sensors sample at their own explicit timestamps. Rendering can run
at 30/60 Hz regardless of physical integration frequency. Optional observation
and render updates should not force full physical checkpoints or CPU readback on
every solver substep.

## Checkpoints and multiplayer

Distinguish an ordinary observation, a physical checkpoint and a match checkpoint.
A physical checkpoint identifies backend/version, model and geometry hashes,
solver configuration, precision, integration duration, identities and restorable
state. A match checkpoint adds the complete native authority, gameplay RNG and
physical-to-rule bindings. An experiment checkpoint additionally captures caller
controller, task and sensor schedules/RNG state when reproducible continuation is
required. Sensors are not silently part of a physical checkpoint today.

Cross-backend interchange is a scenario/state initialization facility, not exact
rollback. Explicitly distinguish full native continuation from reconstruction
that rebuilds warm starts/manifolds. Current Rapier restore already has documented
residuals; neither extraction nor a new backend should promise bitwise equivalence
that the existing implementation lacks. Test determinism within the selected
backend/configuration, and physical tolerances across backends.

Keep multiplayer on its existing backend initially. Later prediction admission
must include backend/model/solver identity as well as field package and tick rate.
Clients cannot replay a PhysX host using Rapier as if it were equivalent. A
mismatched or unsupported predictor can use an explicitly supported drawing-only
mode; otherwise reject admission. Preserve the complete-world replay contract,
confirmation ordering and reliable command ownership.

## Main risks and friction

| Risk | Why it matters here | Design response |
|---|---|---|
| CPU/GPU synchronization | Wheel forces currently run every solver slice; rules consume state every match tick | Batch inputs/queries; retain state; measure wheel loop and compact per-tick readback together |
| Collision model mismatch | Tunnels have roofs/overhangs; tiny fast projectiles hit moving armor | Validate cooked static meshes, two-sided contacts, filtering, CCD and source target masks; no whole-arena convex hull |
| Precision/contact differences | Current dynamics use f64; another backend may use different precision and solvers | Declare precision and test penetration, settling, trajectories and bounded drift; preserve explicit rule time |
| Geometry identity leaks | StaticGeometry and sensor captures contain Parry shapes and collider handles | Immutable source descriptors plus backend cooked handles; stable entity/part IDs and explicit query revisions |
| Match-state duplication | Existing snapshots combine presentation records and full restore authority | Introduce compact observation APIs; share immutable configuration; keep checkpoint compatibility during transition |
| Prescribed versus dynamic joints | Gimbal/arm/folding are largely prescribed today | Model capability flags; retain current behavior first; add torque plants as a separately validated model |
| Model identification | CAD provides geometry/axes, not trustworthy link inertias, friction or motor constants | Preserve fitted current parameters; record assumptions; do not infer a validated plant merely from an imported STEP |
| FFI and packaging | Native GPU libraries, contexts and unsafe bindings differ from a Rust-only build | Optional isolated adapter; keep core safe and CPU builds usable; review any narrow unsafe-policy exception explicitly |
| Sensor continuity | Placement/topology changes invalidate rolling brackets and IMU traces | Backend-independent timestamps/generations and discontinuity tests; never interpolate across reset or missing capture intervals |
| Parallel-world memory | Duplicating arena buffers/cooked geometry per PPO world can erase capacity gains | Share immutable data where backend supports it; measure GPU/CPU memory per world, capacity limits and reset costs |
| Game/controller coupling | Assembly, optical aiming, prediction and construction still reside in server/app modules | Extract pure construction/planning/replay while retaining host authority and policy at transport boundary |
| Scope and churn | A generic rewrite can break the working game before a second backend proves useful | Same-workspace migration, compatibility facades, incremental tests and a minimal working backend prototype |

GPU backend feasibility also depends on the selected SDK's feature coverage.
PhysX offers GPU dynamics and batched state access; the referenced 5.6.1 guide
lists GPU mesh cooking requirements, CPU-side CCD/queries and fixed-buffer limits.
Validate the chosen version rather than treating GPU dynamics as acceleration of
all runtime operations. [PhysX GPU simulation](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/docs/GPURigidBodies.html).
MJWarp targets parallel throughput rather than lowest single-world latency;
it is a useful PPO comparison, not a guaranteed faster interactive plant.
[MuJoCo Warp](https://mujoco.readthedocs.io/en/latest/mjwarp/).

## Migration and acceptance

1. Establish the runtime contract using current Rapier behavior: immutable model
   descriptors, stable IDs, explicit command/state/query batches and compact
   observations. Preserve public entry points with adapters. Add dependency
   checks proving runtime users need no gameplay, networking, Bevy or host clock.
2. Separate physical session ownership from match composition. Rehome reusable
   scene/robot construction so standalone tools can load the approved field.
   Preserve rule tick ordering, snapshots and all RMUC/RMUL authorities.
3. Build the minimal second-backend experiment: one existing omni chassis,
   equivalent wheel model, representative terrain, moving armor and 17 mm shots.
   Compare Rapier and PhysX CPU/GPU with 1, 32 and 256 worlds, or stop at explicit
   resource capacity. Record complete wheel/query/solver/contact/observation cost,
   GPU memory and host/device transfer volume, after compilation and warmup.
4. Refine the interface only where the second implementation demonstrates a
   requirement. GPU batching/device observation support comes from that working
   prototype; do not freeze a universal backend trait before it exists. Evaluate
   MJWarp separately for articulated/control-heavy PPO workloads as warranted.
5. Migrate the game through the unchanged match/session facade. Extend protocol
   identity and optional prediction only when a second backend is admitted to
   multiplayer. Keep the current CPU mode usable throughout.

The prototype passes correctness gates before a speed comparison: settling/load,
acceleration/braking, free wheelspin and slip, power limits, contact filtering,
ramps/steps/tunnel clearances, moving armor and projectile CCD, defeat, physical
attachment/release, reset, rolling scan distortion and IMU cadence. Run relevant
rule/restore tests through match composition. Backend trajectories need not be
identical, but source geometry, inputs and tested physical tolerances must be
comparable and differences recorded.

A throughput win must include actual observations and required synchronization,
not just an isolated solver kernel. Report single-world p95 tick latency separately
from parallel environment-steps/s, policy action rate, reset cost and memory.
No GPU performance gain or backend replacement is established by this design.


## Next mechanical extractions

The Field layout records now live in private `config.rs`; observer/continuation
records and their capture/restore implementation live in private `snapshot.rs`.
Root re-exports preserve the existing public API and serialization field order.
The original Field regressions live in `tests.rs`. These are ownership/file moves,
not changes to rule scheduling, checkpoint guarantees or physics behavior.

The current mechanical extraction pass covers the following boundaries:

1. `PayloadSnapshot` attachment operations now own reusable grip-relative pose,
   endpoint velocity, carry/release and telemetry refresh. World Energy Units
   retain stock, spent/seating permissions and lifecycle authority. The same
   caller-owned physical record is used; no second Unit state store was added.
2. Checked PIE source-package verification, calibration checks and immutable
   collision-scene construction now live in `assets::semantics::assembly`.
   The server keeps its compatibility loader and match setup. Broader robot
   model construction, link inertial provenance and actuator descriptions remain
   follow-up work; importing native CAD caches is not physical validation.
3. Chassis configuration and fitted model geometry now live in private
   `chassis::model`; the Rapier adapter is split into construction, prescribed
   mechanisms and dynamics modules. Existing public `chassis::*` paths remain
   compatible. Wheel equations retain their existing reusable boundary. Function
   bodies and integration ordering are preserved; this is not a new backend.
4. Physics now validates reusable IK warm starts against calibrated arm FK.
   Existing generic joint-path search and collision queries remain reusable.
   Parking search, staged workbench planning, cancellation/time budgets and world
   playback types still belong to server; moving this larger task-specific
   boundary is deferred. Stages, target revelation, rewards and bounded worker
   ownership retain their existing match/host authority. Full physical arm
   dynamics are a separately validated feature.

Composed scenarios, borrowed telemetry and scoped CPU rollout workers already
provide consumers for these boundaries. The GPU experiments do not establish a
throughput win; a new engine abstraction or crate split is not a prerequisite
for this work. Each move should preserve ordinary match/restore behavior and
pass standalone and full-match checks before replacing its compatibility facade.


## Remaining coupling audit (2026-10-23)

The previous pass separated useful operations and files; it did not complete the
runtime/game split. The next extractions below are ranked by value for standalone
robotics consumers, not by file length. They are proposals, not new runtime APIs.

1. **Mechanical model data and source-package loading.** Verified robot loading
   lives in `assets::robots`, and source arm/gripper/Core calibration and FK now
   live in `physics::assembly_model`, with compatible world exports. The next
   boundary is a more general mechanical descriptor with independently verified
   inertial/actuator provenance. Native SolidWorks cache values remain unsuitable
   as automatic physical truth. Prescribed arm state and task playback remain
   distinct from a dynamically articulated arm plant.
2. **Complete arena preparation and match construction.** Physical terrain loading,
   boundaries and shared cooking now live in `assets::arena`; the complete arena
   can run in RobotSession without server/GNS/HTTP or Bevy. Native bindings,
   ownership, spawn policy and `FieldConfig` remain match/server composition.
   `PredictionScene::geometry` still constructs a temporary Field; migrating it
   requires preserving its initial rule-resolved mechanism poses.
3. **Assembly solver state versus task and worker ownership.** Source loading and
   IK-hint validation are now extracted, but `server::assembly_rig::Rig` still
   combines mechanical state, source collision queries, parking assumptions and
   manipulation task state. The app workbench includes that private server source
   with `#[path]`. Expose reusable mechanical/query inputs and planner results;
   retain staged assembly semantics in its task adapter and bounded workers,
   cancellation and wall-time measurement with the host. Keep hidden Q-target
   revelation and actual-joint entry semantics intact. Acceptance: the diagnostic
   no longer compiles private server implementation files directly.
4. **Navigation algorithms versus host service.** `server::navigation` contains
   surface queries, map baking, search and shortcutting, tunnel/posture constraints,
   following, worker queues and Field control. Extract immutable map/search and
   explicit-input follower logic first. The host keeps map-intent permissions,
   Sentry enrollment, worker bounds, failure policy and protocol presentation.
   Avoid physics depending on assets' `GroundMesh`; supply prepared data or a
   narrow query input. Acceptance: the same route/follower can run over a bare
   RobotSession without hosting a match.
5. **Instance identity versus competition motion channels.** Physical moving geometry now supports explicit `MotionId` channels alongside
   the existing `(Mechanism, Team)` adapter. Compositions can drive two same-team
   Outposts independently, retaining identities through reset/capture/insertion;
   malformed updates reject transactionally and rolling sensors check channel
   continuity. The native match still uses its existing role/team drivers.
   Migrating match bindings and serialized continuation to explicit IDs is future
   work; standalone caller-owned coordinates are not a new match authority.
6. **Reusable replay versus networking and backend-specific queries.** Server
   prediction holds the reusable whole-Field replayer alongside package discovery,
   owner anchors, sequence histories and network timing limits; the app separately
   owns projectile replay worker orchestration. Move pure continuation/replay
   operations behind the match session while keeping receipt/acknowledgment policy
   with transport. A later backend comparison must also address `StaticGeometry`
   carrying Rapier/Parry shapes and `ArmorTarget` carrying competition identities.
   Preserve compatible labels initially; do not invent a universal engine trait
   before a second backend or consumer requires it.

Keep one solver for mutually interacting bodies and one match clock/rule order.
Those are necessary coordination, not coupling to remove. No new crate, engine
rewrite, independent rule plugin schedules or GPU port follows automatically
from this audit. Start with the mechanical descriptions/package boundary, then
arena preparation; each has a concrete standalone consumer and a bounded first
implementation. Navigation can proceed independently once its inputs are explicit.

## Arena and mechanical rig extraction (2026-10-23)

`assets::arena` now owns complete terrain loading, source motion partitioning,
boundary fitting and physical installation. `PreparedArena` cooks geometry once;
its `scenario` recipe inserts shared shapes into a fresh `RobotSession` at
construction and reset. Source ground remains available for placement queries.
The standalone `arena_session` example loads the approved complete package,
places a robot and advances it without world, server, GNS, HTTP or Bevy.

The server retains compatibility `Terrain`/loader paths and its Field installer.
Its loader supplies native checked binding partitions through
`load_terrain_with_motions`; it does not convert or weaken native runtime
validation. Team ownership, detector-bank construction, legal spawn policy and
`FieldConfig` stay in match/server composition. Standalone arenas contain scenery
and physical motion channels, not an inferred match or detector authority.
Prediction's temporary Field construction remains a later migration: its initial
rule-resolved mechanism poses must be compared before switching construction.

`physics::assembly_model` now owns the existing source `Model`, `Jaw`,
`Auxiliary`, `RigPose` and frame evaluation. `world::assembly` re-exports those
types, preserving field order, serde representations and caller paths. Existing
schema-five validation is retained; it is source-profile calibration, not a
claim of trustworthy CAD inertias or a universal robot format. The extraction
uses the same already-locked glam version as world to preserve FK arithmetic.
`SourcePackage::model` decodes the shared type from retained verified bytes;
source-specific bearing/jaw guards remain in assets. Attempts, permissions,
rewards, rates, trajectory playback and recovery remain world-owned.

## First performance-conscious query contract

See the [engine performance contract](engine-performance-contract.md) for the
data ownership, lifetime, phase ordering and acceptance design.

The next incremental boundary is `physics::queries`: immutable validated
`RayQuery` requests and capture-local `RayHit` values without public solver
handles. `StaticGeometry::cast_rays_into` accepts a prepared request slice and
exact-size mutable result slice, preserving order and explicit misses. It can
reuse normalization and output storage without capturing a world per call.
Scalar `cast_ray` shares validation, and `geometry::RayHit` remains compatible.
The standalone arena example exercises the batch boundary. No universal backend
trait, GPU buffer interchange or physics acceleration follows from this API.

Keep subsequent boundaries similarly coarse: shared source model preparation,
backend cooking, buffered actuation, explicit stepping, compact observations and
ordered contact drains. Backend resource IDs must stay behind stable runtime
identities; capture-local query ordinals must never be promoted to entity IDs.
A future device path needs its own ownership, completion and layout contract,
not a CPU slice disguised as device memory. Task/controller and match adapters
continue to own cadence and permissions. Performance acceptance must include
setup/reset, cache invalidation, allocation/copy volume and complete observation
cost alongside solver time. The background suspension investigation and engine
contract review inform the next step; this query API does not preselect a new
solver or alter the current exact physical/contact schedule.
