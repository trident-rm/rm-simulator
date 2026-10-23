<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Incremental engine dataflow and performance contract

Status: design contract, 2026-10-23. This document adds no API, backend,
dependency or runtime behavior. It refines the [runtime/game split](runtime-game-split.md)
and [performance priorities](performance-optimization-20261023.md) into reviewable
boundaries. The implementation baseline is `c53cc96aa18703e7ee4d36db9c49c450754d4db9`.
The parent checkout's uncommitted arena/arm extraction and three performance
patches were also inspected read-only; their source fingerprints appear below.
They are not commits in this document's branch. Parent integration and the full
workspace gate remain separate work.

The first core ownership extraction now lives in `rm-simulator-engine`:
poses/time, query and drive records, and wheel laws have one solver-free
definition, with physics compatibility exports. See [Engine architecture](engine-architecture.md)
for the current migration stages. The richer identity/lifecycle/device contracts
below remain design proposals.

## Start with two concrete boundaries

Extract immutable physical descriptions and a borrowed, concrete CPU query view
before designing a solver trait. A first query operation should accept a slice
of metric rays, an explicit capture stamp/filter, and reusable caller output.
Keep the current single-ray and `StaticGeometry` methods as compatibility
adapters. Keep backend handles private. Stable shape identity and capture-local
hit order must be separate fields: changing BVH traversal must not change current
equal-distance selection. The initial CPU slice is now implemented as `queries::RayQuery`,
`queries::RayHit` and `StaticGeometry::cast_rays_into`, with scalar compatibility.
It accepts immutable validated requests and exact-size caller-owned output.
Capture stamps, persistent shape keys and explicit filter descriptors remain
proposed; existing capture context and selection remain caller-owned. These
records are not a new backend trait or a stable FFI/device-buffer ABI.

The current Rapier implementation remains the reference. Introduce a coarse
backend boundary only when another implementation can honor these contracts;
initial concrete methods need no per-body vtable. A standalone user should be
able to verify/load assets, instantiate physical models, step and query them
without importing a host, match authority or renderer.

## Evidence and present implementation

| Area | Implemented evidence | Consequence for the next extraction |
|---|---|---|
| Physical sessions | `physics::session::RobotSession`, `RobotScenario`, explicit `StepDuration`, `step_into` and borrowed chassis telemetry exist | Extend these paths; do not create another world or clock |
| Match sessions | `world::session::MatchSession` and compatibility aliases preserve whole-Field checkpoints | Compact reads supplement checkpoints; checkpoint/public protocol compatibility remains |
| Model calculations | `physics::wheel_model` has scalar laws and retained scratch; prescribed mechanism/FK calculations exist | Separate equations from solver handles without changing arithmetic or force order |
| Geometry/query | `StaticGeometry` owns captured transforms with shared Rapier/Parry shapes; `RayHit::shape_index` is capture-local | This is a useful CPU capture, but not an uncooked backend-neutral model |
| Sensors | Rolling LiDAR checks coherent captures, placement revisions and topology; mounted IMU uses opt-in carrier traces | Preserve timing/interpolation and self occlusion when changing query storage |
| Parent extraction | `assets::arena::PreparedArena` prepares shared cooked geometry and reset recipes; `physics::assembly_model` owns existing arm descriptions/FK, re-exported by world | Reuse this work, but do not call `PreparedArena` backend-neutral source data |
| Parent performance patches | Contact scratch retained; fixed LiDAR broad phase shared across compatible captures; wheel remainder fast paths preserve arithmetic | Retain exact-result evidence; these are scoped changes, not a new backend |

The [headless report](performance-optimization-20261023.md#release-headless-profile-after-the-runtime-extraction)
measured 346–477 ticks/s for one loaded eight-robot world and 1,406–1,753 aggregate
ticks/s for four worlds. Checkpoint capture medians were 13–23 µs and restore
2.05–3.54 ms. Whole-process samples implicated bounds, narrow phase, contacts and
ray tests; they do not identify an exclusive per-step cost or justify weakening
geometry. The parent follow-up reports establish:

- Contact storage: 147,754 modeled fresh-list growth allocations became 315
  retained-buffer growth events; loaded-match median 8.886 → 8.875 s establishes
  no throughput improvement. All 256 checkpoint hashes matched.
- Rolling sensor: median ten-second acquisition 3,142.684 → 3,062.911 ms, with
  complete output hash `eb9ac2da54fe1702` in all three pairs. Compiler contention
  limits causal precision; the fixture settles near its start.
- Wheel scalar loop: 7.5–17.1% faster integration and 0.26–0.90% faster one-robot
  flat rollout, with bit-identical reference trajectories. Suspension queries
  were about 89% of measured chassis-adapter time. No loaded-match gain follows.

These are reported measurements, not reruns for this document. Parent-only
reports are named `performance-collision-pass-20261023.md`,
`performance-sensors-pass-20261023.md` and `performance-chassis-pass-20261023.md`.

The [experimental GPU wheel comparison](physics-gpu-wheel-loop-20261023.md)
measured forward batches of 1/32/256 replicas at 13.63/12.61/20.58 ms per tick
with readback and 13.62/15.01/21.35 ms with DirectGPU. Its matched sequential
Rapier fixture took 13.58 µs per independent world per tick at 256 worlds,
roughly 3.48 ms total. The hybrid adapter loses here. GPU replicas share one
scene; CPU worlds are independent. These are simplified boxes on a floor, not
loaded matches or an intrinsic solver ranking. Low-grip trajectories also differ;
this experiment establishes neither interchangeable plants nor rollback.

## Immutable model and cooked resources

Proposed source descriptors contain immutable FLU-metre vertex/index buffers,
primitive dimensions, topology, mounting frames, joint axes, mass/inertia,
material/filter parameters and the current calibrated actuator/model parameters.
They carry content hashes and provenance, including source face/tessellation
selection and approximation records. Model identity includes parameters as well
as geometry. CAD axes do not establish trustworthy inertia or motor dynamics.
Visual, sensor and collision mesh selections may differ only through explicit,
provenance-bearing descriptions; extraction must retain today's selections.

Backend cooking consumes those descriptors and a cooking configuration. A cooked
resource owns solver-specific shapes, acceleration structures and handles. Its
cache key includes source/model hash, backend/version, precision and cooking
options. CPU collision data can be shared across worlds where the backend permits;
world-local bodies, contacts and warm starts cannot. Device allocations additionally
belong to a device/context. A cached cooked resource must keep its source identity
available for diagnostics and checkpoint admission.

`engine::model::TriangleMesh` now provides immutable validated source buffers;
`physics::cooking::CookedTriangleMesh` retains the source and its shared Rapier
shape. Cooking is explicit and installations do not recook. No persistent cache
or cache-key scheme is implemented. See the [implemented boundary](engine-architecture.md#implemented-sourcecooked-surface-boundary).

Today `CollisionMesh`/`GroundMesh` supply source-like triangle data, while
`StaticGeometry` embeds cooked `SharedShape` objects. Parent `PreparedArena`
reuses those shapes during scenario construction/reset. Preserve that working
CPU reuse while extracting neutral descriptions; do not decode and cook meshes
again on every reset or query. Immutable sources can outlive a session; cooked
resources must outlive every capture or in-flight operation using them.

## Identity, revisions and coherent views

Proposed runtime keys are opaque world/epoch plus entity identity and, where
storage slots can be recycled, generation. Part/link/joint/shape keys resolve
through the model's stable part table. They are not pointers, backend handles,
array positions, team roles or source path-list handles. Hosted chassis IDs
remain non-reused under the existing public contract. Adding internal generations
must not renumber them or alter serialized fields.

Keep invalidation causes explicit:

| Stamp component | Changes when | Invalidates |
|---|---|---|
| World/reset epoch | Reconstruct/reset the session or replace its live world | Commands/views from the previous run, rolling brackets, query bindings |
| Topology revision | Add/remove entities, links, shapes or change connectivity | Index mappings, batch layouts and rolling interpolation |
| Model/geometry revision | Replace shape data, dimensions, filter or physical parameters | Corresponding cooked/query resources and compatibility checks |
| Placement revision | Teleport, revive or discontinuously reposition an entity | Motion interpolation and mounted sensor history for that entity |
| Completed stamp plus slice ordinal/phase | Advance integration or change prescribed poses | Pose-dependent query state and stale observation reuse |

A global monotonically changing revision can implement an initial conservative
invalidation policy. Separate revisions are worthwhile only when they avoid
measured rebuilds. Hashes identify immutable content; cheap revisions identify
live mutations. Existing placement checks are real; this complete revision scheme
is proposed. Standalone `BodyBinding` now carries a runtime brand and non-reused
installation generation, and rolling scene brackets validate runtime brands.
`RobotScenario::reset` still restarts legacy chassis identities; an adapter
retaining those unbranded keys across resets needs an external epoch. Nanoseconds
alone cannot distinguish two runs at time zero. See
[source-described bodies](physics-reuse.md#source-described-bodies-and-bindings)
for the implemented, narrower binding contract.

A borrowed truth/query view is valid only while its immutable borrow remains;
mutation/step requires dropping it. A retained owned capture pins shapes and its
own poses at a named stamp. It may be used after the live world advances, but
must never be described as current. A rolling acquisition requires two compatible
captures and emission times bracketed by them. Reject topology, placement, model
or epoch discontinuities rather than interpolate through them. Existing
`StaticGeometry` ray ordinals remain capture-local; a new persistent key is
additional metadata, not a reinterpretation of that ordinal.

## Step phases preserve the current authority

The normative behavior is the current code, especially
[`Field::step_with_hits`](../crates/rm-simulator-world/src/lib.rs),
[`advance_2027`](../crates/rm-simulator-world/src/rmuc_2027.rs) and
[`WorldPhysics::step`](../crates/rm-simulator-physics/src/runtime.rs).
The phases below document existing ordering; they are not independent scheduled
plugins. Extraction must preserve early exits and error/partial-progress behavior
as well as successful execution.

1. Validate requested end tick/time overflow. With no physical activity, native
   authority or active assembly, jump to the requested tick and sync mechanisms.
2. For an active physical world, derive start target frames from current rules
   at the old timestamp, including commands/hits already applied there.
3. Increment the Field tick. Advance RMUC, then RMUL, then assembly. RMUC first
   observes RFID using pre-integration chassis positions, advances its authority,
   then installs defeat/power budgets. Preserve each adapter's internal order.
4. Sync mechanisms; if physics was idle, continue to the next tick. Otherwise
   derive end target frames and integrate physics from the old timestamp.
5. Record measured chassis drive power into RMUC state; synchronize Energy Units.
6. Consume raw projectile contacts in their existing order, skip disabled armor,
   score at the new tick timestamp, invoke the hit observer immediately and
   append retained hit history. Then apply chassis collision damage.
7. Sync mechanisms again, sync defeats, and expire old hit history.

Within each physical tick, retain current query priming, projectile lifetime/bounds
removal, target validation and teleport handling. Each solver slice installs
interpolated kinematic endpoints, applies projectile drag, applies chassis forces,
records pre-step chassis velocities, damps payload contacts, steps Rapier, syncs
fold colliders, records carrier traces, captures projectile contacts and armor
collisions, then performs perimeter/dwell retirement. Preserve collection order
and current contact timestamps even when a slice ordinal is added internally.

At 128 Hz, `substep_count` divides the tick into eight equal slices under the
1 ms maximum; each is 0.9765625 ms. The wheel integrator retains its own eight
internal motor/slip slices. Other validated session rates use their existing
subdivision. Batch wheel rays/forces only within a coherent slice; no pose or
support result may cross a solver advance. Reordering force sums, sorting contacts,
changing meshes/CCD/substeps, or folding multiple match ticks across a rules
barrier requires separate behavioral approval and validation.

One backend owns every mutually interacting dynamic body. A timestamped query
mirror derived from it is permitted; independently stepping contact partners in
two engines is not. One native authority owns each competition's rules. A PPO
action repeated for N ticks still executes every intermediate physical/rule
phase. The host alone paces real time; physics/gameplay never read wall time.
Multiplayer stays 128 Hz, with existing command ownership and confirmation order.

## Coarse batches and reusable CPU storage

The first boundary should use concrete slice operations, with a single coarse
call per batch rather than FFI or dynamic dispatch per body/property. Suggested
operation families below are contracts, not implemented signatures:

| Operation | Inputs | Output and ordering |
|---|---|---|
| Query rays | Capture token, rays, range/filter/self-exclusion, caller output | One hit/miss result per input ordinal; stable nearest-hit ties |
| Apply controls/wrenches | Validated addressed commands or force/torque records for one phase | Explicit accepted/rejected records; declared replace/add and force lifetime |
| Advance | Immutable step duration, target/motion endpoints, retained scratch | Completed stamp; ordered contact records through the ordinary step path |
| Observe | Requested identities/fields and completed stamp | Borrowed compact truth or selected records into caller storage |
| Drain contacts | Tick/slice range and cursor, caller storage | Ordered records, continuation cursor and completion status |

Keep ray direction normalization, inclusive range, solid-origin zero hit,
miss/error distinction, collision filtering, two-sided mesh behavior and current
equal-distance order. Wheel support excludes projectiles. Rolling LiDAR retains
per-emission moving interpolation, self filtering, pattern/noise global emission
index and timestamp; IMU retains opt-in substep carrier traces. A batched API must
not replace these with one pose for a whole scan.

CPU convenience methods may grow a caller `Vec`, retaining capacity. Document
whether they append or clear: existing `RobotSession::step_into` appends contacts;
sensor `sample_into` replaces a frame after validating inputs. It does not remove
all allocations inside physics stepping. Keep these public semantics intact.
For proposed bounded methods, output capacity, required/written count, high-water
mark and overflow status are explicit. Validate ray/observation batch inputs and
capacity before publishing output; a capacity error can be retried without
advancing time. Offer reusable scratch for validation when necessary.

Contacts generated during integration require a stronger policy: preflight a
supported workload bound, retain all events in growable backend storage, or drain
in ordered chunks with a cursor while preventing the next mutation. Never discard
rule-bearing contacts, advance again to retry a full buffer, or return an ambiguous
successful partial tick. Device overflow must fail/report the affected interval
and cannot masquerade as a complete match step. A transactional whole-tick failure
is not promised by current code; introducing it would require state recovery.
Application-specific observation dropping may be allowed only through an explicit
consumer policy, never for authoritative damage/events.

For forces, state coordinates, application point/COM convention, units, accumulation
order, replace-versus-add semantics and expiry after a solver slice. Aggregate
per-body wrenches only if it preserves the existing floating-point order and
response. The measured GPU experiment's resultant wrench is an adapter choice,
not proof that changing the reference accumulation is bit-exact.

## Truth, sensors and presentation have different costs

Use existing `ChassisControlState` for compact copied control inputs and
`ChassisTelemetry<'_>` for borrowed privileged truth, including lazy wheel views.
`RobotTruth` still contains an owned `ChassisSnapshot`; its name does not imply a
zero-allocation read. A proposed multi-robot observation view should borrow
immutable configuration and wheel storage, stamp scalar values once and fill
only fields the consumer requests. Full-world snapshot construction belongs to
checkpoint/network publication cadence, not every controller query or solver
slice. Measure upstream publication construction even if a downstream owner
anchor already borrows its input.

Sensors consume coherent geometry/carrier state and produce measurements, not
privileged per-return world poses. ROS publication, vision image encoding and
training tensors are adapter costs with their own cadence, queues and reset state.
A vision renderer consumes pose/appearance data; it does not advance physics.
The app converts observations to passive render `SceneState`, reusing buffers and
shared mesh/material handles. FLU/wxyz conversion remains at the existing
presentation boundary. No renderer imports of physics/world/server are needed.

## Device memory is a distinct future contract

Current core queries and observations are CPU-side. No shared CUDA/wgpu tensor
API is implemented or implied. A future device buffer must specify element
schema/stride/precision, backend device/context identity, owner, access mode,
capacity, producer completion event and consumer release/lifetime. A borrowed CPU
slice cannot represent pending device output. Cross-stream dependencies must be
explicit; CPU reads wait for completion. Keep resources alive until consumers
finish and prevent reuse while a previous frame is in flight.

An adapter should expose CPU and device observations separately and report
unsupported combinations. Sharing a machine or GPU does not establish CUDA/wgpu,
Python or cross-process interoperability. Use an explicit supported transfer path
and count its copies, submissions, waits and bytes. A device-resident rollout is
only demonstrated when policy/sensor/control consumers actually remain resident;
DirectGPU plus CPU wheel control/readback remains hybrid. CPU rules may require
per-tick readback; retain that barrier until an equivalent authority is validated.
No device integration or unsafe exception is authorized by this design.

## Reset, reconstruction and continuation

`RobotScenario` rebuilds initial state and restarts time; it is not continuation.
Callers reset controller/task RNG, sensor schedules, rolling brackets and IMU
history themselves. Stage setup failure before replacing the live session, as
the existing transactional reset does. Share immutable geometry across resets.

Match checkpoints preserve native authority and hidden physical/rule state through
`FieldSnapshot::restore`; reconstruction rebuilds solver manifolds/warm starts.
Do not market a list of body poses as a complete checkpoint. A future physical
continuation contract must enumerate projectile retirement/contact history,
actuator state, prescribed endpoints, identity/revisions, pending records, solver
configuration and any native state the backend can actually retain. Experiment
continuation additionally captures caller sensor/controller/task state.

Backend/model/precision/version compatibility must be explicit for replay. A
cross-backend state transfer initializes a scenario; it cannot promise exact
continuation. Keep today's public restore/serde paths and residual guarantees.
Prediction continues to rebuild the whole Field at changed checkpoints and replay
ordinary commands/steps; a compact observation is never its replacement.

## Composition and migration

Logical dependency direction is verified assets → immutable runtime models →
cooked physical session → optional match composition → host/application adapter.
This is dataflow, not permission to reverse crate dependencies: physics remains
independent of assets, gameplay, world, networking and Bevy. Assets currently
depends on physics/gameplay; extract neutral descriptions inside existing modules
first. Pure gameplay stays independent; world combines it with physics. Render
accepts caller-owned presentation records; app bridges them. No crate per layer
is required.

A bare driving game selects models, controls and optional scenery without referee
rules. ROS composes the same physical or match session with timed sensor/publication
adapters. Vision composes passive rendering and sensors over that session. PPO
selects bare physics, full match, or a declared rules-only abstraction, with task,
reward and action cadence outside the engine. These are composition possibilities,
not newly supplied integrations.

Implement migration in independently reviewable steps:

1. Retain compatibility exports and exact-equivalence fixtures. Extract uncooked
   geometry/model records with hash/provenance metadata, preserving current bytes,
   calibrated parameters and cooking. Reuse parent arena/arm extraction.
2. Add the concrete CPU query view and ray batches with reusable output; adapt
   existing scalar methods. Verify ordering, invalid inputs, ties, self exclusion,
   rolling discontinuities and placement/reset behavior before changing callers.
3. Move hot consumers one at a time: suspension queries, rolling captures,
   compact controller/observation reads. Preserve each slice and authority barrier;
   compare full contacts, truth and sensor outputs, not final poses alone.
4. Extract preparation/replay operations as consumers require; keep server layout,
   world aliases and public checkpoint serialization as compatibility facades.
   Prediction construction must preserve rule-resolved initial mechanism poses.
5. Only after measured need, evaluate coarse backend dispatch with supported
   capabilities for meshes, CCD, prescribed motion, queries and continuation.
   An unsupported requirement rejects admission; it never silently reduces fidelity.

Reject a universal ECS/plugin scheduler, scene graph rewrite, trait per property,
mandatory device tensor API, generic robot format inferred from CAD, or immediate
crate/repository split. Reject whole-world snapshots as a universal observation
and independent rule plugins with their own clocks. These generalizations add
coordination and compatibility costs before proving a second consumer's need.

## Acceptance and measurement record

Every implementation step records source/dirty diff hashes, package manifest
hashes, executable/profile/toolchain/backend configuration, exact input sequence,
reset/warmup boundaries, seeds, duration and raw output hashes. Compare baseline
and candidate after builds complete, with paired order reversal and repeated runs.
On Talos use private source/target directories, a moderate build job count, the
read-only canonical `/home/hxyulin/.cache/rm-simulator-arena/field-2027`, and
serialize timing with `/tmp/rm-perf-pass-20261023.lock`. Build outside that lock;
record compiler activity/load because the lock cannot exclude external work.
Use the fleet skill for access. This document required no remote access or timing.

| Acceptance workload | Correctness gate | Performance record |
|---|---|---|
| Flat 0/1/8-robot rollout, 128 Hz and validated custom rate | Exact baseline snapshot hashes, tick partitioning, command/torque/zero-power/defeat behavior | Ticks/s, warm allocations and bytes/tick, scratch growth/high water |
| Loaded eight-robot match, 256 × 16 ticks, paired 1/4 worlds | All checkpoint hashes and ordered contacts; same package, shots, native rules and restore point | Aggregate/per-world ticks/s, batch median/p95/max, memory/world, phase costs |
| Ten-second rolling MID360, 2,000,000 emissions, plus moving-source tests | Complete return hash; tie/range/solid-origin/self filtering, noise stride, topology/placement rejection; IMU trace equivalence | Capture/acquisition/physics time separately, rays/s, allocations/bytes/capture and emission, fixed cache rebuilds |
| Suspension adapter with rotating/moving support and self exclusion | Same nearest support, normals, forces, wheel state and complete trajectories | Rays/slice, query versus wheel integration time, force calls/copies, query rebuilds |
| Reset and match restore/replay | Hidden authority/contact state, stale-key rejection, existing solver residuals | Reset/restore latency, shared versus per-world memory, cooked-resource reuse |
| Owner publication and passive scene sync | Confirmation/periodic ordering, public schema and render values unchanged | Construction and delivery allocations/copy bytes separately; changed transform/material counts |

Instrument boundary calls/batch sizes, retained capacity/growth, actual allocations
and allocated bytes, source-to-cooked duplication, CPU-to-CPU copy bytes, host↔device
bytes, submissions, completion waits and wait duration. Separate cold preparation,
first-use caches, reset and warmed stepping. Count bounded overflow, failed batches
and cache invalidations explicitly; do not infer allocator totals from vector-growth
models. For future GPU work report end-to-end consumer latency and memory/world
alongside kernel time, distinguishing independent worlds from shared-scene replicas.

For a pure extraction or allocation patch, require exact same-backend outputs
where existing fixtures support them and no timing/fidelity change. A new backend
requires separately agreed physical tolerances and scenario coverage, not relaxed
expectations for the extraction. An allocation win can be accepted without claiming
throughput if exact parity holds and repeated timings show no established gain.
Do not extrapolate scalar-wheel gains, rules-only throughput or GPU kernel time to
full-game/PPO training speed. Parent performs integration/full workspace checks;
this document's validation covers only links, formatting and source provenance.

## Inspected source fingerprints

SHA-256 of parent files read for this design, relative to its checkout. These
identify the inspected dirty content rather than a hypothetical integrated commit;
later extraction may legitimately change it. No parent file was edited.

```text
dd10bc1eb19294744ced930bcfc6eb9f57630269c5983e84e9aea5db7f637fb0  docs/runtime-game-split.md
d2e3d207e56f9ad9a53f8b59b12b4a6f859f987258e20177a5b9ad76e6b2b30a  docs/performance-optimization-20261023.md
cee8c515ac17936228ed817c6ff2e6c62083580b9a183d46b51ed518175fc5fe  crates/rm-simulator-assets/src/arena.rs
aecbe8f7651283ffe4e55c913d0d5f713b35d9a9cfe23625a9bcfd85559908d9  crates/rm-simulator-physics/src/projectile.rs
06eabeae6d2e958dda9b7bea8c346d64d5475ca94f515ae528010fe7b51910bd  crates/rm-simulator-physics/src/sensors/rolling.rs
f705a8ecde9bf622eb8af5a94f10061a41a7397406e3fc6bc2befc272201d60c  crates/rm-simulator-physics/src/wheel_model.rs
cbd7b4c1b6ea69d6d0ca89d987020fa237015fc16d215982e624069bd13267c7  crates/rm-simulator-physics/src/assembly_model.rs
```
