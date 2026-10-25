<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Simulation architecture

The solver-free `sim-engine` owns shared poses/timing, query and drive
records, and scalar wheel laws. Physics re-exports these contracts and retains
Rapier execution. See [Engine architecture](engine-architecture.md) for the
implemented split and planned component lifecycle.

Generic execution lives in `sim-physics`; RM physics and prescribed motion live
in `rm-sim-physics`, which opts into the generic crate's Rapier extension API. The complete
`Field` facade in `rm-sim-world` coordinates them with live rules and the
referee on one explicit clock (128 Hz by default, with configurable in-process
physical/rule stepping). Rendering consumes caller-owned appearance
and has no dependency on physics or gameplay. The server host worker owns each
live simulation and roster; app systems and transports submit commands to it.

`rm-sim-control` now owns navigation search/following and source assembly
planning over explicit observations and shared geometry. Host workers, permissions
and authoritative actuation remain outside it. See [controllers](controllers.md).

The authoritative engineering rules behind this layout are in `AGENTS.md`; this
guide explains how the pieces fit together. Each crate's own `README.md` states
what it owns, its module layout and its permitted dependencies, and
`scripts/check-module-dependencies.py` enforces the boundaries in CI.

See [physics reuse](physics-reuse.md) for the public integration API and examples,
and [development](development.md) for the targets that build and test the
workspace. The world referee runs the gameplay engine as the live
rules authority; [Gameplay](gameplay.md) covers what it owns and what stays
outside the live integration.

Only one native competition authority is present in a `Field`: source-bound
RMUC, RMUL 3v3, or RMUL Infantry 1v1. RMUL practice geometry does not reuse
RMUC detector bindings. Configuration and restore reject mixed authorities;
the public view must exactly match its hidden checkpoint authority. All formats
use the same physics and explicit step clock. See [RMUL coverage](rmul-2027.md).

For the initial two-session split and proposed backend-neutral runtime migration,
see [runtime/game split](runtime-game-split.md). Sessions, reusable wheel equations,
composable equipment, compact telemetry, bounded CPU rollout workers and initial
model/source-package extractions are implemented. The remaining coupling audit
there distinguishes further proposals from the unchanged Rapier backend.

## Ownership

| Data or decision | Owner |
|---|---|
| Rigid bodies, suspension, ballistics, raw contacts | Physics library |
| Prescribed rotor/rail motion and fitted armor geometry | Physics library |
| Detection intervals, rune activation, match phase and clock | World rules and referee |
| HP, damage, heat, allowance, economy, experience, respawn, round result | Gameplay engine, run by the referee |
| Motion stop decisions and resolved mechanism positions | World, supplied to physics |
| Complete restore and tick orchestration | `Field` facade |
| Entity lifecycle, transforms and materials | Renderer |
| Snapshot-to-appearance adaptation | Presentation library |
| Presentation clock, interpolation selection, controls and UI | App |
| Roster, command authority and transport delivery | Server host |

The generic physics library depends only on the solver-free engine core among
simulator crates. RM physics additionally depends on that generic runtime; numerical helpers, serialization and the MID360 pattern remain
ordinary library dependencies. Its RM-specific caliber
lookup helpers remain available for source compatibility; physical stepping does
not apply their damage or detection policy. The renderer depends on neither
physics nor world. The presentation library bridges borrowed world records into
renderer appearances; it has no host, transport or session ownership. Generic
rendering lives in `sim-render`; `rm-sim-render` depends on it for the RM extension
and retains compatibility re-exports. See the
[presentation/render split](presentation-render-split.md). `scripts/check-module-dependencies.py` enforces these boundaries.

## Tick order

For an active physical world, `Field::step` performs these operations in order:

1. Capture target start poses from current authoritative state.
2. Advance the tick and rune state.
3. Apply referee transitions, then advance any changed rune at the same time.
4. Resolve mechanism positions and target end poses.
5. Step chassis and projectiles together through one physical tick.
6. Process contacts in returned order: disabled armor, detection, damage or
   activation, then referee observation.
7. Observe outpost destruction, resolve changed mechanisms, disable defeated
   chassis, and expire old hit reports.

With no bodies and no native authority, the field jumps to the target tick.
A referee keeps the tick loop active even when physics is idle. Mechanism state
is resolved only when articulated scenery exists. Commands and hits can change
motion at the current timestamp, so each tick rebuilds start poses from current
state. Scoring must not be deferred to another tick by an asynchronous queue.

## Restore and prediction

`Field::restore` rebuilds the complete simulation with shared collision geometry.
Both host and client prediction use the ordinary command, fire and step paths.
A checkpoint preserves hidden rule state, identities, detection history and
motion stops. Restore rebuilds solver warm starts, contact manifolds and the
projectile touched-armor cache; these retain their documented residuals.
Rebuild when the checkpoint changes, not once per rendered frame.

The extracted types retain the existing world import paths where practical.
Native match checkpoints carry geometry bindings and rules; shared presentation
records do not execute historical rules. Raw wire IDs and `ArmorTarget` remain unchanged. See the reuse
guide for the replacement of referee-dependent geometry queries with
`StaticGeometry::at_mechanisms`.

## Embedding boundaries

`physics::session::RobotSession` provides bare caller-paced physics and
repeatable initial-scenario reset without match authority.
`world::session::MatchSession` provides caller-paced match stepping, addressed controls, explicitly
privileged truth access and complete in-memory reset points. Per-field
`StepDuration` travels in checkpoints. Multiplayer admission remains 128 Hz until
rate negotiation and all peer schedulers support customization.

`rm-sim-assets` owns verified CAD import and source metadata without a
dependency on world, networking or rendering. The server retains compatibility
imports and native runtime validation. The renderer remains a separate passive
consumer. See [simulation embedding](simulation-embedding.md) for the public API,
direct wheel controls and subsequent adapter boundaries.

## App and ECS state

`PredictionState` owns the replay worker, geometry, checkpoint context, input
history, accepted prediction and counters inside `Session`. Embedded and remote
sessions both use `Client`; the embedded host still owns its simulation worker.

Chassis ingestion builds an ID index once per frame and resolves `PresentedPose`,
armor state and equipment-light state components. Transform and material systems
consume changed components. Existing part markers remain available, and absent
scene input leaves the visuals untouched. Projectile spheres retain their pool.

Tests cover unchanged transforms, first-frame poses, wheel visibility, light
states, chassis removal, native authority validation, tick partitioning and
complete-world replay. Use the [performance probes](performance.md) to measure
new builds on their target hardware.
