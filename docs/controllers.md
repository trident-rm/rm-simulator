<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Caller-driven controllers

`rm-sim-control` owns the existing navigation and assembly planning algorithms.
It depends on verified assets, physical models/queries and the assembly task's
RoboMaster pose profile. It has no world, host, transport, Bevy or renderer
dependency. It never spawns a worker, reads wall time, advances a simulation or
applies a command. These are concrete CPU algorithms over shared collision
resources, not a plugin scheduler or another physics world.

## Ownership

| Responsibility | Owner |
|---|---|
| Support layers, inflated clearance, A*, shortcuts and tunnel fitting | `control::navigation::Map` |
| Waypoint following, stow/alignment, stalled recovery and learned obstacles | `control::navigation::Controller` |
| Verified assembly rig, IK branches, swept guards, staged plans and retiming | `control::assembly::Rig` |
| Search admission, one bounded navigation worker, generations and reply delivery | Server navigation adapter |
| Assembly permissions, manually parked chassis, hidden-target reveal and worker lifetime | Server assembly adapter and world authority |
| Collision/kinematic commands, explicit stepping and actual attachments | Physical runtime and world adapter |
| Network-facing diagnostics, presentation and user controls | Existing host/app paths |

The server preserves `navigation::{Bake, Phase, View}` through re-exports, so
snapshot field layouts and protocol identities are unchanged. World playback
records remain world-owned. The server projects copied arm/payload observations
into controller inputs and converts completed playback samples back to those
existing records. Controllers cannot grant ownership, attach a physical Unit,
teleport the chassis or award an assembly result.

## Navigation without a match

Run the bare-physics example:

```sh
mbx run -p rm-sim-control --example navigation --locked
```

It cooks a wall, plans a detour, observes a robot in `WorldPhysics` and applies
controller outputs through ordinary chassis commands. No `Field`, server,
network or renderer is constructed. The bounded example must reach its goal.

Preparation and stepping are separate:

1. Supply a `Surface` with shared support geometry, catch-floor elevation,
   explicit exclusions and optional passage hints. `Map::prepare` validates
   bounded bake inputs before creating the graph. The six surveyed UC passage
   hints are an explicit `rmuc_2027_passage_hints()` preset, not implicit terrain.
2. Register a robot in `Controller`, select a goal and borrow its `Request`.
   The caller supplies current geometry, robot obstacles and actual start pose
   to `Map::plan`. The host can run this synchronous operation on its worker.
3. Mark submission and return the result with its original robot/generation.
   Stale results are ignored. Invalid route records fail before changing state;
   an observed start displaced by more than the existing tolerance blocks travel.
   Route admission is structural validation, not a collision certificate: use
   paths produced by `Map` against coherent geometry, or validate an external path.
4. Supply `RobotObservation`, live geometry, obstacle records and **elapsed
   nanoseconds** to `Controller::command`. Apply the returned command only after
   caller authority permits it. A stopped controller returns `None`.

The host retains its original update cadence: eight 128 Hz ticks per controller
update. The library's three-second stalled-recovery and ten-second preparation
windows now use explicit elapsed time rather than counting calls. A standalone
caller can use another cadence without changing those durations. The controller
is open-loop between supplied observations; it is not a replacement for a full
navigation stack, and retains the existing conservative footprints/terrain policy.

Robot IDs must not be reused while results from that controller can remain in
flight. On scenario reset, discard old worker results and rebuild/rebind controller
state. The controller does not serialize worker requests or enter match restore.

## Assembly without host-owned state

`assembly::Rig` owns copied kinematics, shared collision resources and bounded
planning hints. `ArmPose` contains actual joint/jaw angles; `PayloadObservation`
contains the caller-resolved physical pose, rigid grasp and whether that payload
is held by the selected arm. These inputs contain no player/team ownership or
live `Field` reference. The host resolves that authority before projection.

The same rig is used by the in-game automatic assembly worker and the standalone
`engineer_assembly` workbench. The workbench imports the controller library; it
no longer compiles private server source files. Verified source/terrain loading,
Cartesian jog guards, staged L1–L3 planning, the separately revealed attached
Q twist, rounded speed-limited playback and release/return validation use the
existing algorithms and search order.

`Signal` carries caller-owned cancellation/progress atomics. The library honors
cancellation at existing guard boundaries and search work remains budgeted.
The host owns the thread, cancellation decision, generation, permission checks
and trajectory admission. Wall-time measurement also belongs to callers:
`planning_elapsed_s` is zero unless filled externally. The workbench preserves
its measured timing report at that boundary.

Assembly playback retiming retains the existing **128 Hz** cadence and simulator
joint/jaw rates; it is not automatically retimed for a custom-rate physical
session. The rate constants now live with the mechanical model and keep their
old world compatibility paths. Generic dynamic actuator control and arbitrary
robot packages remain separate modeling work. The optional `test-support`
feature exposes a synthetic diagnostic fixture, never verified source CAD.

## Behavior and cost evidence

The frozen implementation at `1e5b6dd` was compared with the extraction on the
same Apple Silicon host. Navigation was fed the **same physical observations**
in lockstep: 1,536 ticks each for wall detouring and stowed tunnel traversal.
Every selected command and diagnostic record matched exactly; independent map
routes also matched. The three original host/physical integration tests still
exercise actual travel, reopening, obstruction and bounded recovery.

A synthetic assembly fixture matched the original resolved frames, mechanical
model, replay state and bounded-failure diagnostics; caller-owned wall time is
excluded from that numeric comparison. Existing rig/IK/collision/retiming tests
moved with the implementation. New public-boundary tests cover malformed and
stale results, explicit-duration timeouts, invalid observations and bounded
UTF-8 diagnostics. No checkpoint/protocol schema change is needed.

The optional approved-source preload-clearance and manual-parking tests also
pass. They exercise L1–L3 at both reference and settled parking poses. The
existing bounded unresolved L3 attached-twist case remains unresolved; the other
L3 fixture completes its revealed twist. This extraction changes ownership, not
planning coverage or physical fidelity.

The host now borrows compact chassis telemetry for following and retains its
robot-ID buffer. Dynamic obstacle collection is performed once for a following
robot and reused within that command decision; it still allocates/copies that
obstacle list. Cooked geometry is shared, and controller extraction introduces
no per-step recooking or world checkpoint copy. Completed assembly playback is
converted once at the worker/result boundary, outside physical stepping.

The small lockstep probe observed lower command-selection time after extraction,
but it was a single ordered development run with warm caches. It establishes
output parity, not a whole-game throughput improvement. Evidence and source
fingerprints are retained in ignored `local-assets/controller-extraction/`.

Sentry targeting/intelligence policy, external-control authority and match task
state remain host/world responsibilities. Portable full robot packages, generic
checkpoint continuation and another solver are not supplied by this extraction.

## Arm search and assembly modules

`rm_sim_control::manipulator_plan` owns bounded joint-space search and IK candidate
selection. It consumes the physical arm model and caller collision checks without
stepping a runtime. `PlanError` implements the standard error traits. The assembly
planner retains search/caches, with exact certificate matching in
`assembly/planner/transitions.rs` and guarded replay/retiming in
`assembly/planner/playback.rs`. Existing `Rig` method paths remain unchanged.
