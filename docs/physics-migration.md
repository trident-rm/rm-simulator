<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Physical runtime migration

The destination is a reusable physical runtime composed with robot models,
controllers, sensors, a RoboMaster match adapter and a passive renderer. This is
an incremental extension of the [engine architecture](engine-architecture.md),
with the [performance contract](engine-performance-contract.md) governing hot
paths. It does not require a new repository, crate for every responsibility or
an immediate second solver.

## Current boundary

The solver-free engine owns poses, step durations, query records, scalar wheel
laws and immutable surface/body/joint/assembly descriptions. Physics cooks and installs shared
resources into one Rapier world. Source package verification and composition live
in assets. Gameplay owns competition rules, world composes those rules with
physics, server owns host authority and pacing, and render consumes presentation
records. These boundaries already work for bare sessions and match sessions.

The root `PhysicalRuntime` owns generic bodies, joints, bindings, queries and
physical observations in one native scene. `WorldPhysics` composes it with
chassis, projectiles, payloads, prescribed scoring targets and mechanism channels.
That facade retains participant-specific force/motion/contact scheduling and
compatibility methods. `ArmorTarget`, `Team`, caliber helpers and retirement
remain explicit domain concepts in the participant adapters, rather than fields
or decisions in the generic runtime. Both paths use the same cooked resources
and native integration.

`StaticGeometry`, `PreparedArena` and cooked resources retain Rapier shapes.
They are useful shared backend resources, not a portable uncooked scene.
Source robot descriptors retain fitted chassis configuration and provenance;
they do not yet describe a complete physical link/joint graph. Native CAD mass
properties remain independently unverified and must not silently become inertia
defaults. Current gimbals, folding and manipulation playback are prescribed;
adding generic joints does not automatically make those robots dynamic arms.

## Completion criteria for the physics port

The migration is complete when ordinary game physics consumes the same reusable
physical descriptions, cooked resources and runtime operations as standalone
scenarios, and the generic runtime can operate without RoboMaster team, armor,
caliber or match concepts. Compatibility facades may retain those names while
composing explicit domain adapters over the generic runtime. Moving files or
adding an alias alone does not satisfy this boundary.

The behavior-preserving construction and ownership port is implemented: chassis,
projectiles, targets and carried objects use shared source/cooked builders;
parentless scenery uses the runtime factory. Generic mounts and sensor carriers
operate without chassis IDs, and the game retains its original participant slice
sequence. Existing match continuation remains the `Field` adapter's responsibility;
its private physics ownership admits no arbitrary generic models into hosted
checkpoints. Recipe reconstruction remains distinct from continuation.

Current prescribed gimbals, folding and arm playback remain supported modeling
choices. Dynamic mechanisms, another solver, GPU integration, URDF importers,
complete robot link-graph packages and exact native solver checkpoints are
follow-on fidelity or interoperability work, not unfinished ownership extraction.

The ordered increments below record the migration sequence and future extension
points. Historical suggestions to replace prescribed game mechanisms are optional;
they do not block the reusable physical runtime boundary.

## Ordered increments

1. **Joint foundation and standalone fixture.** Add validated fixed, revolute and
   prismatic descriptions, local attachment frames, explicit collision policy,
   limits, compact state, motor commands and direct effort. Bind live joints to
   live bodies with world/generation checks. Establish removal/reset behavior and
   force signs using a two-axis turret before changing an existing game model.
   Initial articulated bodies use the existing supported primitive collision
   shapes. Arbitrary multi-turn state, physical actuator electronics and joint
   continuation checkpoints need explicit later contracts.

2. **Robot assembly descriptions and installation — physical tree implemented.** Compose existing body/joint
   descriptions into a validated immutable graph with stable source part keys,
   mount frames and separate collision, sensor and visual references. Validate
   missing references, duplicate keys and unsupported topology before installing
   any bodies. Prepare shapes once and retain dense binding tables per instance.
   Define whether loops and multiple parents are supported rather than inferring
   them from CAD assemblies. Start with a small hand-authored turret package and
   retain recipe reconstruction as its reset contract. Add a URDF importer only
   after this representation works; URDF is an input adapter, not the runtime.
   `AssemblyDescription` now validates a connected rooted tree; `CookedAssembly`
   shares physical resources, and transactional installation/removal returns
   explicit source-to-live bindings. The turret exercises this path. Visual and
   source mount tables are now available; importers and complete robot package
   integration remain optional extensions.
   Explicit convex point sources and body cooking now cover the existing deck
   and bumper shapes, with [native shape parity tests](convex-colliders.md).
   Preserve compound collider frames and filtering; do not substitute a box or
   import a nonconvex dynamic CAD surface silently. Whole-chassis construction,
   mass distribution and participant ownership were subsequently covered by the
   prepared chassis and runtime extraction increments.

3. **Participant registration and physical events — first separation implemented.** Extract runtime-owned body,
   collider, joint and query bookkeeping from projectile-specific ownership.
   Keep concrete, buffered participant adapters for chassis, projectiles and
   payloads. Record the existing per-slice sequence before moving code: target
   endpoints, forces, velocity capture, solver, folding synchronization and raw
   contact collection must retain their order. Preserve legacy public paths with
   compatibility exports. Introduce explicit part identities and contact records
   sufficient for the match adapter to map contacts to armor; physical state must
   not need to understand an Outpost or damage rule.

   Read-only branded participant/collider references now cover native objects
   without a second registration map; optional bounded per-slice observations
   remain off by default. RoboMaster scoring maps and contact scratch/capture
   live in a domain adapter with unchanged authoritative outputs. See
   [Physical contacts](physical-contacts.md). The runtime extraction retains
   participant-specific force/motion scheduling in `WorldPhysics`; existing match
   continuation remains supported, while generic models have recipe reset only.

4. **Migrate one existing robot adapter — chassis construction implemented.** First construct an existing chassis
   through shared body resources while keeping the ray-wheel model and force
   sequencing unchanged. Preserve fixed/prescribed gimbals as a supported model
   choice. Separately offer the validated dynamic turret as an opt-in robot model:
   tune actuator gains, inertia, collision exclusions and reaction forces against
   a declared reference. A dynamic joint is a modeling change, so identical
   trajectories are not an appropriate acceptance condition for that option.
   Only after this succeeds should the Engineer arm or folding linkage migrate.
   Decide explicit per-body CCD/damping and unsupported feature admission before
   migrating participants which depend on them; retain existing projectile CCD.
   Also preserve gravity scaling, axis locks, sleep policy, material coefficient
   combination and the current chassis mass/COM distribution. Convex shape parity
   alone does not establish those policies. Generic body wrenches must not erase
   a migrated chassis plant's per-slice forces when ownership is unified.

   Existing chassis presets now prepare source body/collider descriptions and
   consume the same cooked builders as generic bodies. Explicit collider-derived
   mass preserves the historical deck/turret aggregation, including zero-mass
   auxiliaries and native material combination. The adapter retains insertion
   order and deferred COM timing; it does not enter the generic held-wrench
   registry or replace wheel forces. Independent native construction parity and
   exact trajectory hashes gate this change. Public prepared-model reuse and
   generic runtime ownership are covered by the subsequent
   [standalone runtime](physical-runtime.md) increment.

5. **Sensor and presentation bindings.** Mount sensors to explicit body/part
   frames rather than only a chassis ID. Retain per-emission motion, self-occlusion,
   runtime brands, coherent topology checks and seeded effects. Introduce reusable
   pose output keyed by model parts for passive rendering without exposing solver
   handles. Visual parts can follow a physical body with a local transform; one
   CAD mesh per collider is unnecessary. Keep privileged truth distinct from
   measurements and perform FLU conversion only at the renderer/adapter boundary.

6. **Complete match adaptation and continuation.** Move team/armor scoring maps,
   projectile policies and match-specific mechanism schedules behind adapters.
   Rules retain authority over HP, heat, power permissions, defeat and victory;
   the physical plant consumes resolved physical commands. Extend match restore
   only when the new participants' persistent state is enumerated: joint commands,
   controller/plant state, payload attachments, contact latches and identity maps
   matter as well as body poses. Preserve current multiplayer restrictions until
   codecs and admission negotiate the new model contract. Recipe reset must never
   be presented as arbitrary checkpoint continuation.

7. **Capabilities and another backend.** Once these concrete consumers establish
   the interface, make unsupported shapes, joints, CCD and sensor queries explicit
   capabilities. Evaluate another backend with the same source model and complete
   workload. Keep CPU and device buffer contracts distinct. Existing PhysX trials
   did not establish a throughput win, so a GPU port is not a migration prerequisite.

Each increment remains independently reviewable. Future model/backend extensions
should build on the concrete lifecycle and preserve the same admission, identity
and scheduling contracts; a broad plugin scheduler is not required.

## Acceptance gates

For behavior-preserving extraction, compare exact existing rollout/checkpoint
hashes and ordered contact streams. Exercise native match rules, source terrain
parity, reset, prediction restore and protocol admission. Keep the ordinary game
building throughout. Primitive/joint mechanics additionally need units/signs,
limits, motor saturation, reaction/contact behavior, invalid-command no-mutation,
wrong-world/stale bindings and reset/rebinding tests.

For intentionally different physical models, report the changed assumptions and
use bounded reference trajectories, constraint error, actuator limits and contacts.
Do not demand bitwise equality with a prescribed animation or advertise CAD
inertia as measured truth. A visual approval can establish appearance, not force
accuracy.

Measure preparation/cooking, installation/reset, steady command/read/step,
sensor acquisition and observation export separately. Include ordinary 0/1/8-robot
matches and bounded multi-environment CPU batches; retain scratch capacity and
record copied/allocated bytes where instrumented. A no-op generic path should
not add per-tick allocations or materially regress the existing match workload.
Report noisy timings as inconclusive rather than calling architectural movement
a speedup. Broaden performance tests when a changed hot path or unresolved result
justifies it.

This plan preserves the current session model: callers own explicit time and
consumer cadence, one runtime owns each interacting world, and controllers,
match rules and sensors do not independently advance the solver.

## Port verification record (2026-10-23)

The completed extraction retains all six pre-port chassis trajectory hashes and
all three native match checkpoint hashes (0/1/8 robots). Frozen native-constructor
oracles cover chassis presets, projectile fire/restore, moving scoring targets,
and carried-object collision/release/reattachment. They compare native body and
collider settings as well as ordered physical outputs. The verified field package
also passes standalone-versus-game terrain installation and recipe reset.

The full isolated Talos `just verify` gate passes, including existing match
restore/prediction and protocol tests; no checkpoint schema or protocol revision
changes are introduced by this port. Generic models remain excluded from hosted
match continuation. Legacy rolling scan, IMU and serialized IMU history parity
is checked at both 128 Hz and 1000 Hz against a same-platform baseline. Exact
floating-point stream hashes are not promised across operating systems.

A bounded Apple M5 Max development-profile probe (1024 steps; existing
`rollout_cost` fixture) recorded the following non-paired samples. Background
load and timing order were not controlled, so these are regression observations,
not established speedups. Reconstruction includes fresh source validation/cooking;
callers needing repeated installation should retain prepared resources.

| Robots | Pre-port step, µs | Final step, µs | Pre-port reset, µs | Final reset, µs |
|---|---:|---:|---:|---:|
| 0 | 0.158 | 0.171 | 9.549 | 11.401 |
| 1 | 82.602 | 81.149 | 205.824 | 239.240 |
| 8 | 621.334 | 604.546 | 1511.078 | 1846.622 |

The samples show no steady-step regression for loaded fixtures, but ordinary
convenience reconstruction is slower. This port establishes reusable ownership
and preparation; it does not claim a throughput improvement or solve every reset
cost. Native solver warm starts/manifolds, projectile touching sets and chassis
armor contact-start latches still rebuild on match restore.
