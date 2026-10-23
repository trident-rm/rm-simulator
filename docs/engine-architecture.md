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

## Current migration: solver-free core

`rm-simulator-engine` now owns the existing FLU `Pose`, `StepDuration`/default
timing constants, validated `RayQuery`/`RayHit`, `DriveControl`/`WheelTorques` and
scalar wheel model/state/integration. It depends only on numerical and
serialization libraries. It contains no solver, scene loader, team identity,
match authority, networking, renderer or host clock. It can be built/tested
without Rapier or a running simulator.

`rm-simulator-physics` retains the concrete Rapier world, chassis construction,
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
explicit inertial descriptions (see below). Link/joint, convex, actuator and
sensor descriptions remain subsequent work. Source package hashes/provenance remain with the verified assets; mesh
buffer sharing is not a persistent content hash or topology identity. There is no
global cache, persisted cooked format or backend interchangeability guarantee.
A future cache must include source content, backend/version/precision and cooking
options; `Arc` identity alone is insufficient. No numerical speedup is claimed.

## Implemented concrete physical runtime

`physics::runtime::WorldPhysics` now defines the single interacting Rapier world,
its construction and immutable integration duration, query priming, solver slices,
wheel/plant force ordering, prescribed target endpoints, carrier traces and raw
contact capture. The existing root and `projectile::WorldPhysics` paths re-export
that exact type. No facade world, second clock, solver trait or plugin scheduler
is introduced. Implementation remains inside the projectile module family so its
participant adapters keep private access; separating those adapters is later work.

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
runtime (see below). Joint bindings, broader topology revisions and capability
negotiation remain subsequent work.

## Implemented body descriptions and runtime bindings

`engine::model::{RigidBodyDescription, ColliderDescription, ShapeDescription}`
separate primitive/indexed geometry, local mounting frames and explicit contact
parameters from live pose/velocity. `BodyKind::Dynamic` requires a validated
`Inertial` with total mass, COM frame and principal moments, with no inferred
CAD mass. `BodyState` carries copied world motion. `CookedRigidBody` prepares and
shares backend shapes while retaining source descriptions. The initial shapes
are spheres, boxes and fixed/kinematic surfaces; dynamic surfaces are rejected.

`runtime::install_body` accepts a caller `BodyKey` and returns an opaque
`BodyBinding`. Bindings contain a process-local runtime brand and non-reused
installation generation, hiding native handles. Duplicate/invalid installations
are rejected before mutation; removal/reset invalidates previous bindings.
`body_binding(key)` supports explicit rebinding after recipe reconstruction.
Borrow source data or copy compact motion; velocity/wrench/kinematic endpoint
commands use world axes and documented units/lifetimes. Existing participant
APIs, match checkpoints and wire formats remain unchanged. Rolling scene brackets
also reject different runtime brands, preserving capture lifetime boundaries.

This implements basic standalone rigid bodies, not a new articulation format,
solver trait or robot migration. Joint bindings, convex geometry and explicit
solver capability negotiation follow after these foundations. The
[physics reuse guide](physics-reuse.md#source-described-bodies-and-bindings)
contains units, reset/error contracts, current limitations and a runnable example.

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
   inertial data are implemented; joint/actuator descriptions and cache
   identities remain subsequent work.
3. **Runtime composition and identities — concrete runtime extracted.** Introduce the minimal stable binding
   and epoch/revision contracts required by a second standalone consumer. Adapt
   existing RobotSession/MatchSession paths; preserve one world and exact clocks.
   Expose coarse preparation, buffered commands, query batches, observations and
   contact drains before considering dynamic dispatch.
4. **Controller/task extraction.** Separate reusable navigation/search/following
   and manipulation algorithms from server queues, permissions and task authority.
   Remove diagnostic inclusion of private server implementation files. Keep
   bounded workers and cancellation with the application owning them.
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
