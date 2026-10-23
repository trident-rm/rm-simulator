<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Physical participants and contacts

The concrete runtime separates read-only physical identity from RoboMaster
scoring ownership. `runtime::ParticipantBinding` identifies any native rigid
body; `ColliderBinding` identifies any native collider, including parentless
scenery. Neither token contains a team, armor role or model key. They are opaque,
process-local, nonserializable references, validated against the runtime brand
and native handle generation. Removal, slot reuse and recipe reset invalidate
state access. They are not persistent source identities or command authority.

`WorldPhysics::participants()` and `colliders()` enumerate live references without
allocating a registry. `participant_state` returns copied FLU body truth;
`collider_state` returns the collider pose and actual native parent, if any.
`body_participant`, `chassis_participant` and `projectile_participant` connect
existing model/domain identifiers to physical references. An armor housing's
native parent can differ from its gameplay owner: callers keep that mapping.
Generic command bindings remain separate so reading a chassis does not register
it for generic wrench application or interfere with wheel forces.

## Optional bounded observations

Call `enable_contact_observations(capacity)` with a positive record budget before
stepping. The default has no observer allocation and returns before scanning
contact pairs. Enabled observation adds a native contact-pair scan after each
solver slice, before prescribed folding collider synchronization. It does not
change integration, latches, projectile retirement or damage decisions.

`physical_contacts()` borrows at most the selected number of records for one
world tick. Each active native pair contributes its deepest cached contact per
slice, in native encounter order. This order is not a cross-backend guarantee.
Records contain both collider references and their parent associations at capture
time, world FLU points, a first-to-second normal and signed separation in metres.
The normal and separation are cached constraints; points transform cached local
contacts using current poses. These values are not a freshly recomputed,
mutually coherent narrow-phase projection. There is no inferred incoming impact
velocity, begin/end transition, sensor measurement or gameplay hit.

`time_ns` is the slice endpoint, computed as tick start plus
`floor(step_ns * substep / substeps)`; `tick_end_ns` identifies the containing
tick and `substep` is one-based. Time comes from the caller's ordinary step,
never a new clock. A record can retain a removed participant's identity while
subsequent state access correctly reports it stale.

The capacity spans the whole tick. Additional samples increment
`physical_contacts_dropped()` rather than growing storage. This explicit loss
policy applies only to optional diagnostics, never authoritative scoring.
`drain_physical_contacts` appends into caller storage and retains observer
capacity; it leaves the dropped count intact. The next step clears records and
count. Reconfiguration clears both; zero capacity rejects without mutation.
Disabling releases storage. A failed step is not transactional and can leave
previous observations preserved or cleared depending on where it failed.

## RoboMaster adapter and remaining migration

`projectile/contacts.rs` owns the scoring map, pre-integration velocity scratch,
touching scratch and armor-collision output, together with the existing scoring
and collision capture methods. Public `Contact` and `ArmorCollision` records,
ordered scoring semantics and match checkpoint formats remain unchanged. Their
time and latch semantics are independent of optional physical samples.

The runtime still schedules chassis, projectiles, payloads, prescribed targets
and their policies in one Rapier scene. This increment does not claim a fully
generic backend or migrate chassis construction. Physical tokens are not added
to checkpoint or network codecs; standalone reset remains recipe reconstruction.
See the [migration plan](physics-migration.md) for those remaining gates.

The public integration tests cover parentless floor and dynamic-pair ownership,
foreign/stale/reset references, bounded overflow and retained drains, exact slice
timestamps at 128/1000 Hz, and observer-on/off scoring, projectile state and
retirement parity over five simulated seconds. Full legacy rollouts additionally
check behavior-preserving adapter extraction.
