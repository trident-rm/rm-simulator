<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Standalone physical runtime

`physics::PhysicalRuntime` owns one concrete Rapier scene: bodies, colliders,
source-to-live bindings, joints, held commands, query preparation and optional
physical contact observations. It has no robot team, armor scoring, projectile
caliber or match authority. It uses the same native scene and integration slices
as the game; there is no alternate reduced world or solver backend.

`physics::WorldPhysics` is the compatibility composition for RoboMaster physical
participants. It owns one `PhysicalRuntime` plus the chassis, projectile, payload,
prescribed mechanism and scoring adapters. Its existing public methods and
import paths remain available. The facade retains the original per-slice order,
including pre-impact velocity capture, payload contact damping, folding updates,
carrier traces, scoring latches and projectile retirement.

## Lifecycle and ownership

| Stage | API and owner |
|---|---|
| Describe | Solver-free engine body, collider, joint and assembly descriptions |
| Prepare | Shared `CookedRigidBody`, `CookedAssembly` and triangle resources |
| Install | Explicit `BodyKey`/`JointKey` mappings and transactional assembly installation |
| Command | Held body wrenches, joint motors/direct effort and kinematic endpoints |
| Advance | Caller-stamped `PhysicalRuntime::step(prev_ns)` at immutable `StepDuration` |
| Observe | Body/joint state, physical participant references, geometry captures and optional contacts |
| Reconstruct | `from_recipe` creates a new runtime and fresh bindings |

The bare runtime begins without a catch floor. Install one explicitly with
`install_catch_floor`, or install fixed geometry and bodies. Existing
`WorldPhysics` and `RobotSession` defaults retain their catch-floor behavior.
Gravity defaults to 9.81 m/s² downward in world FLU. Time belongs to the caller;
the runtime neither reads host time nor schedules a sensor, controller or match.

Command bindings are distinct from read-only participant references. Generic
held-wrench application acts only on its installed bodies. Chassis adapters keep
their wheel forces outside that registry, so observing a chassis cannot enroll
it in a force-reset path. Keys are logical installation identities; branded live
tokens reject foreign worlds, removal and reconstruction. Native handles remain
private. See [Physical contacts](physical-contacts.md).

## Geometry and passive consumers

`geometry_capture` retains enabled collider poses, identities and shared shapes,
including unbounded half-spaces. `PhysicalGeometry` supports metric ray requests
and exact-size caller-output batches. Ray hit ordinals refer to that capture;
use `collider_binding` to recover physical identity. Captures remain snapshots of
old state after mutation or removal. They are privileged truth, not sensor data.

Source mounts use model-local `CarrierKey` and `MountKey` values. `MountTable`
validates explicit carrier references and shares immutable attachment arrays.
Stamped carrier/mount records retain actual resolved poses, exact integration
intervals and continuity evidence; body and prescribed frames use the same
composition convention. These copied records do not grant command authority or
independently authenticate caller-supplied continuity claims. Sensor consumers
must retain the actual geometry runtime brand and topology checks as well.

## Prepared chassis compatibility

`chassis::PreparedChassis` retains an immutable configuration, cooked body and
source-order armor/prescribed-part indices behind shared ownership. Prepare once
and use `WorldPhysics::add_prepared_chassis` for multiple robots or recipe resets.
Installed native shapes share storage; body handles, wheel state, commands and
contact history remain independent. The existing `add_chassis` convenience path
uses the same construction and preserves its validation/ID semantics.

The [body runtime example](../crates/rm-sim-physics/examples/body_runtime.rs)
constructs, steps, observes and reconstructs a bare scene without robot or match
types. Run it with `mbx run -p rm-sim-physics --example body_runtime --locked`.

## Prepared carried objects and domain construction

`projectile::PreparedPayload` retains the original `PayloadShape` and cooked
single-compound body behind shared ownership. `add_prepared_payload` installs it
with independent live state; `add_payload` remains the convenience snapshot
adapter. Hull point order, mass distribution, attachment transitions and loose
object damping retain the existing behavior. Source and native shapes can be
reused across recipe reconstruction without recooking.

Projectiles and scoring targets also use cooked body descriptions. Each
`WorldPhysics` keeps at most one prepared projectile per caliber, keyed by its
restitution. A policy edit affects newly fired balls; existing bodies retain
their own material. Parentless scenery installation shares the runtime factory
while each adapter retains its filtering, motion metadata and query-priming
boundary. No generic held-wrench registration is added for domain participants.

## Reconstruction limits

The generic runtime does not promise arbitrary checkpoint continuation. A recipe
reinstalls initial geometry, models and commands; callers restart time, reacquire
bindings, and reset their sensor/controller/task history. A body/joint state read
is not a complete checkpoint: held commands, pending endpoints and adapter state
matter, as do the documented rebuilt native solver caches.

Existing match continuation stays in `Field` and the RoboMaster adapters. New
generic physical models are not automatically admitted to multiplayer prediction
or added to its codecs. The [migration plan](physics-migration.md) records that
boundary. Prescribed game gimbals, folding and manipulation remain supported;
using a new dynamic joint model is a separate fidelity change.
