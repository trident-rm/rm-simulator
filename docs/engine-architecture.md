<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Engine architecture and incremental migration

The target is a lightweight robotics simulation engine with explicit state and
buffered computation, composed with a small component lifecycle. RoboMaster is
one simulator built from that engine. Keep the working driving game, standalone
sessions and competition rules operational throughout migration.

This follows MuJoCo's separation of compiled model/state and caller-driven
stepping, and Gazebo's separation of component responsibilities across lifecycle
phases. It does not require their model formats or runtime plugin systems.
See [MuJoCo simulation](https://mujoco.readthedocs.io/en/stable/programming/simulation.html)
and [Gazebo system interfaces](https://gazebosim.org/api/sim/10/createsystemplugins.html).
The detailed [performance contract](engine-performance-contract.md) governs
buffer ownership, exact ordering, device boundaries and acceptance workloads.
The [physics migration plan](physics-migration.md) expands the remaining body,
joint, robot assembly, participant and sensor work into ordered acceptance gates.

## Current migration: solver-free core

`sim-engine` now owns the existing FLU `Pose`, `StepDuration`/default
timing constants, validated `RayQuery`/`RayHit`, `DriveControl`/`WheelTorques` and
scalar wheel model/state/integration. It depends only on numerical and
serialization libraries. It contains no solver, scene loader, team identity,
match authority, networking, renderer or host clock. It can be built/tested
without Rapier or a running simulator.

`rm-sim-physics` retains the concrete Rapier world, chassis construction,
source model fitting, query execution, collision captures, sensors and physical
sessions. It re-exports the moved contracts, so existing caller paths name the
same Rust types without conversion, model copies or duplicate implementations.
The chassis-to-wheel-model conversion stays beside the chassis configuration;
wheel laws and arithmetic remain in core. Query normalization uses the same
locked glam type and arithmetic as the original Rapier vector alias. Native
ray construction remains with geometry query execution in physics.

The workspace dependency check enforces this boundary. This is an ownership
extraction: it introduces no generic backend trait, independent component
scheduler, dynamic loading or physical behavior change. Team/caliber/target
identities still present in physics are compatibility debt, not engine contracts.
The separately published `rm-sim` 0.0.0-dev package is a development placeholder;
it does not ship these runtime crates or expose a simulator API yet.

## Implemented source/cooked surface boundary

`engine::model::TriangleMesh` is an immutable indexed surface in local FLU
metres. Construction takes owned buffers and validates finite vertices and index
bounds without changing winding, ordering, duplicate vertices or degenerate faces.
Clones share the original buffers. Empty surfaces remain valid source descriptions;
backend cooking decides whether they can become collision shapes.

`physics::cooking::CookedTriangleMesh` retains that description and a shared Rapier
shape/BVH. It uses the existing two-sided surface policy with duplicate-vertex
merging and degenerate-face removal. Cooking copies into backend storage once;
cloning or installing the cooked resource does not copy triangles or cook again.
`WorldPhysics::add_cooked_static_mesh` creates a world-local collider at identity
placement using the existing friction/restitution settings. Dropping the resource
wrapper does not invalidate installed colliders or retained geometry captures.
The source coordinates become world coordinates at this identity installation;
placement remains outside the surface description.

`assets::CollisionMesh::into_description` consumes converted/placed FLU buffers
without copying them. Existing `add_static_mesh` remains the owned-buffer path:
it uses the same validation and cooking implementation, consumes unique buffers
without cloning, and does not retain a second source mesh in every live world.
Moving-mesh and boundary adapters continue through that path with their existing
channels, filters and insertion order. `PreparedArena` remains a Rapier-cooked
arena that shares shapes across construction/reset, not a neutral scene format.

This surface boundary is complemented by primitive, collider, rigid-body and
explicit inertial, joint, convex-hull and rooted assembly descriptions (see below).
Actuator electronics, visual/sensor mounts and import formats remain subsequent
work. Source package hashes/provenance remain with the verified assets; mesh
buffer sharing is not a persistent content hash or topology identity. There is no
global cache, persisted cooked format or backend interchangeability guarantee.
A future cache must include source content, backend/version/precision and cooking
options; `Arc` identity alone is insufficient. No numerical speedup is claimed.

## Implemented concrete physical runtime

`physics::PhysicalRuntime` owns the single generic Rapier scene, its immutable
integration duration, body/joint bindings, query priming and physical contact
observations. The `WorldPhysics` compatibility facade composes that scene with
RoboMaster chassis, projectiles, payloads, mechanisms and scoring. Its original
slice schedule remains authoritative for those adapters. Existing root,
`runtime::WorldPhysics` and `projectile::WorldPhysics` paths identify that same
facade, not a second scene. There is no additional clock, solver trait or plugin
scheduler. See [Physical runtime](physical-runtime.md) for standalone use.

`WorldPhysics::step_into(prev_ns, frames, contacts)` appends to caller-owned contact
storage. The owned-output `step` adapter uses that implementation. Neither method
owns elapsed time: a session or match adapter supplies the starting timestamp and
owns clock advancement. Existing target validation, projectile retirement, contact
order and error behavior remain unchanged. In particular, an error is not a
transactional rollback of earlier query priming or retirement; it appends no new
contacts. Match rule barriers remain in `Field::step_with_hits`.

`RobotSession::step_into` now writes directly into the same caller buffer across
ticks, avoiding a temporary returned contact vector per tick. Moving-target callers
can use `step_with_targets_into` with identical endpoint validation and completed
stamps to `step_with_targets`. This retains buffer capacity, not every internal
solver allocation. Reset still reconstructs an initial recipe; callers reset
sensor/controller histories separately. World-scoped body bindings and basic rigid-body descriptions now extend this
runtime (see below). Broader topology revisions and capability negotiation remain
subsequent work.

## Implemented body descriptions and runtime bindings

`engine::model::{RigidBodyDescription, ColliderDescription, ShapeDescription}`
separate primitive/indexed geometry, local mounting frames and explicit contact
parameters from live pose/velocity. `BodyKind::Dynamic` requires a validated
`Inertial` with total mass, COM frame and principal moments, with no inferred
CAD mass. `BodyState` carries copied world motion. `CookedRigidBody` prepares and
shares backend shapes while retaining source descriptions. The initial shapes
are spheres, boxes, explicit convex hulls and fixed/kinematic triangle surfaces;
dynamic nonconvex surfaces are rejected.

`runtime::install_body` accepts a caller `BodyKey` and returns an opaque
`BodyBinding`. Bindings contain a process-local runtime brand and non-reused
installation generation, hiding native handles. Duplicate/invalid installations
are rejected before mutation; removal/reset invalidates previous bindings.
`body_binding(key)` supports explicit rebinding after recipe reconstruction.
Borrow source data or copy compact motion; velocity/wrench/kinematic endpoint
commands use world axes and documented units/lifetimes. Existing participant
APIs, match checkpoints and wire formats remain unchanged. Rolling scene brackets
also reject different runtime brands, preserving capture lifetime boundaries.

This implements standalone rigid bodies, not a complete robot asset file format,
solver trait or robot migration. Explicit solver capability negotiation follows
after these foundations. The
[physics reuse guide](physics-reuse.md#source-described-bodies-and-bindings)
contains units, reset/error contracts, current limitations and a runnable example.

## Joint descriptions and runtime bindings

`engine::model::JointDescription` declares fixed, revolute or prismatic relative
motion, parent/child local attachment frames, optional limits and connected-pair
contact policy. Joint-local +X is the free axis. `JointMotor` carries explicit
spring/damper targets, gains and effort saturation; `JointState` is compact scalar
physical truth. Revolute coordinates are principal angles, without a multi-turn
continuity guarantee. These records contain no native handle or match identity.

The concrete runtime connects live `BodyBinding`s through opaque `JointBinding`s.
Removal of a connected body removes its joints; stale/foreign tokens fail, and
recipe reconstruction requires rebinding by `JointKey`. Held effort and motor
commands are mutually exclusive and operate in the ordinary solver slices, with
reaction forces/torques on the connected bodies. Source body wrenches remain
independent held commands. Joint state is an observation, not continuation state.

The [standalone turret](jointed-mechanisms.md) validates a physical yaw/pitch
assembly, limits, reaction and reset. Existing game gimbals/folding/manipulation
remain prescribed. CAD joint import, complete robot asset packages,
checkpoint integration and migration of existing participants remain separate
increments in the [physics migration plan](physics-migration.md).

## Implemented physical assembly trees

`AssemblyDescription` owns a shared immutable connected tree of source links and
joints. `AssemblyLinkId`/`AssemblyJointId` identify entries within the source,
independently of live `BodyKey`/`JointKey`. Initial link poses and COM/angular
velocities are explicit in a common assembly FLU frame; joint attachments remain
body-local. Structural validation rejects cycles, disconnected links and multiple
parents while preserving caller array order. No CAD mass or joint placement is
inferred, and closed loops are currently unsupported.

`CookedAssembly` shares prepared body shapes, source arrays and resolved endpoint
indices. It validates canonical native body modes and joint conditions without
creating a world. Installation takes an explicit source-to-runtime mapping and
rigid placement, rotates initial velocities and stages native bodies/joints before
any mutation. Mapping, placement, numerical and generation-capacity errors return
without consuming live slots or generations. Commit uses the same body/joint
helpers as individual installation and preserves source order; there is no trial
world or remove-on-failure rollback. Allocation failure retains normal Rust
process behavior rather than a recoverable transaction guarantee.

`AssemblyBinding` retains source-to-live tokens for observation/control. Aggregate
removal validates all tokens before removing anything; ordinary body removal also
cascades to externally attached joints. Recipe reconstruction reuses prepared
shapes and requires fresh bindings. This adds no per-tick assembly scheduler or
serialized continuation state. The standalone turret now uses this path; game
robot descriptors remain fitted chassis packages. Visual/sensor mounts and
importer contracts are still needed before general CAD robots
can migrate.

`ShapeDescription::ConvexHull` extends physical link collision coverage without
importing tessellation or implicit inertia. Its source points remain unchanged;
backend cooking computes their envelope and rejects degenerate/unrepresentable
results. Captures and triangle expansion retain these shapes. The
[convex guide](convex-colliders.md) records parity against the current chassis
deck/bumper hulls; whole-chassis construction and mass policy are not yet migrated.

## Target ownership

| Layer | Owns | Interaction boundary |
|---|---|---|
| Engine core | Coordinates/units, time, immutable physical descriptions, identity contracts, command/query/event records, reusable model laws | Solver-free values and borrowed/reusable buffers |
| Physical backend/runtime | Cooked resources, body/link/joint state, integration, exact queries, physical contacts, actuation and resource lifetimes | One owner of each interacting world; coarse command/step/query/read operations |
| Simulator composition | Verified robot/arena packages, entity assembly, sensor/controller selection and scenario presets | Resolves model identity and binds component inputs/outputs |
| Controllers and sensors | Control policies and timed measurements over coherent state | Commands in; permitted observations and measurements out |
| Gameplay | Deterministic competition state, permissions, rewards, HP and victory | Pure rule inputs and resulting effects |
| Match adapter | Physical/rule contact mapping, exact rule/physics order and complete match continuation | Composes runtime with one competition authority |
| Renderer | Scene resources, cameras, appearance and drawing | Passive pose/appearance frames; no rules or world mutation |
| Game and other adapters | Player controls, menus/HUD, host/network authority, ROS/vision/training integration | Own consumer cadence, transport and presentation state |

These are responsibilities, not a crate for every table row. The existing assets,
gameplay, world, render, server and app crates retain their current dependencies.
Assets may prepare physical descriptions but engine core never imports assets.
Gameplay remains independent of engine/physics. Renderer can keep its existing
passive records; there is no requirement to make it inspect a physical world.

## Components cooperate through the runtime

A component is a model or behavior with explicit inputs, outputs and lifecycle.
Initially components are ordinary Rust values composed by the caller. Examples
are an omni-wheel plant, prescribed gripper, mounted LiDAR, controller or rotating
physical assembly. An Outpost's physical rotor/detectors can be composed without
its match HP or destruction policy. Team ownership and scoring remain outside
that physical component.

One runtime owns mutation of interacting physical state. Controllers read a
permitted observation and submit held commands; a plant computes forces using
current support queries; the backend integrates; sensors sample coherent
captures; match rules interpret ordered contacts; renderer consumes presentation
state. Components do not each keep a mutable pointer to the world, independently
step shared bodies or write one another's private state.

Use typed calls and batches for hot paths. Establish model indices and validated
bindings during setup; retain scratch/output capacity. Explicit records carry
contacts and other events between stages. Async messages are appropriate at
worker, network and ROS boundaries. A universal message bus for each wheel force
or joint read adds unnecessary dispatch, copies and ordering ambiguity.

## Lifecycle proposal

The following is a target composition contract, not a new scheduler API.

1. **Prepare/configure:** verify model sources, resolve component bindings, cook
   shared resources, install bodies and prepare sensor/controller buffers. Reject
   unsupported capabilities rather than silently reducing fidelity.
2. **Before physical step:** consume admitted commands in a declared order and
   resolve prescribed endpoints. Controllers may read the preceding completed
   observation. Per-slice wheel work uses the current physical scene, with no
   support result reused across an advance without a proven exact query contract.
3. **Integrate:** one backend advances the world on its explicit duration,
   preserving current substeps, force ordering and contact timestamps.
4. **After physical step:** publish coherent state and ordered physical events;
   derive scheduled sensor measurements from valid captures/carrier traces.
   Consumers retain their own slower cadence and reusable outputs.
5. **Present/export:** rendering, ROS publication, recording and training adapters
   consume the outputs they need without advancing another physical world.
6. **Reset/release:** reconstruct or restore through the declared session contract,
   invalidate retained views and reset caller sensor/controller/task state.

The match adapter can have rule barriers before and after integration. Its
current exact schedule remains normative: rules observe pre-integration poses,
mechanisms resolve, physics integrates, contacts are scored and resulting effects
are installed before the next tick. A generic lifecycle must fit that behavior;
it must not move all rules to a post-step callback or introduce another clock.

Borrowed read views prevent concurrent mutation through Rust lifetimes. Owned
captures retain old poses and shared shapes explicitly. Capture-local ordinals
remain distinct from persistent entity/part IDs. Reset epochs, topology/model
revisions and capability negotiation are future work; they are not implied by
the current timestamp or by a `RayHit::shape_index`.

## Incremental migration and gates

1. **Core contracts and wheel laws — implemented in this change.** Move existing
   definitions once; preserve facade paths and serialized layouts. Prove engine
   dependency isolation, scalar/batch query behavior, wheel trajectories, native
   rule tests, full-match restore and baseline rollout hashes.
2. **Uncooked model/resource boundary — started with triangle surfaces.** Move physical source descriptions
   out of backend-owned shape representations. Keep content identity/provenance
   and separate visual, sensor and collision choices. Keep cooked caches keyed
   by backend/version/options and share immutable resources across resets. Do not
   infer trustworthy link inertia or motors from native CAD caches. Indexed surface
   sources, shared Rapier cooking, primitive/body descriptions and explicit
   inertial data, joint descriptions and bounded spring/damper motor commands are
   implemented; rooted physical trees and transactional installation are also
   available. Complete robot asset packages, actuator electronics and cache identities
   remain subsequent work.
3. **Runtime composition and identities — concrete runtime extracted.** Introduce the minimal stable binding
   and epoch/revision contracts required by a second standalone consumer. Adapt
   existing RobotSession/MatchSession paths; preserve one world and exact clocks.
   Expose coarse preparation, buffered commands, query batches, observations and
   contact drains before considering dynamic dispatch. Read-only native body and
   collider references, with optional bounded per-slice contact observations, now
   separate physical identity from the RoboMaster scoring adapter. See
   [Physical contacts](physical-contacts.md); specialized force/motion scheduling
   and persistent checkpoint identities remain subsequent work.
4. **Controller/task extraction — navigation and assembly implemented.** Separate reusable navigation/search/following
   and manipulation algorithms from server queues, permissions and task authority.
   Remove diagnostic inclusion of private server implementation files. Keep
   bounded workers and cancellation with the application owning them. `rm-sim-control`
   now provides the existing algorithms without world/host/render dependencies;
   the server and workbench consume its public API. Task authority remains in
   world/server. See [controller contracts](controllers.md).
5. **Optional backend/plugin boundary.** Refine capability and dispatch interfaces
   only with a real additional implementation/consumer. Measure complete query,
   model-law, solver, observation and synchronization cost. CPU/device buffers
   remain different contracts; the previous GPU experiment did not establish a win.

Each step must stand alone: the existing game still builds and runs; compatibility
entry points remain; geometry, ordered contacts, sensor output and native rules
retain their validation. Checkpoints are not replaced by compact observations,
and initial recipe reset is not advertised as arbitrary physical continuation.
The migration itself claims no throughput improvement. Measure actual allocation,
copy and synchronization costs separately from structural code movement.
