<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Standalone scenarios and rule adapters

A small physical scenario does not need an arena, referee, host or window.
`rm_sim_assets::scenarios::Scenario` prepares checked equipment geometry
and detector definitions, then creates ordinary `RobotSession`s. Separate
`rm_sim_world::rule_adapters` translate observations into existing gameplay
authorities. Neither layer introduces a second scheduler or physics engine.

## Presets

| Example name | Contents |
|---|---|
| `outpost-red`, `outpost-blue` | One complete approved Outpost, source rotor and three rectangular detectors |
| `base-red`, `base-blue` | One approved Base, bound shields and source detector faces |
| `rune` | Complete source Rune, both rotor channels and ten target disks |
| `tunnel-fixture` | Explicitly approximate 2 × 1 × 0.25 m three-box passage |

Source presets retain the package's existing placement, geometry and source
motion frames. They require the verified native 2027 bindings. The tunnel
fixture is illustrative and may instead be constructed with caller dimensions;
it is not an export or clearance claim about the official arena. These presets
use a flat catch floor at z=0, without importing surrounding terrain.

```sh
cargo run -p rm-sim-assets --example standalone_scenario --locked -- outpost-red field-2027
cargo run -p rm-sim-assets --example standalone_scenario --locked -- rune field-2027
cargo run -p rm-sim-assets --example standalone_scenario --locked -- tunnel-fixture
cargo run -p rm-sim-world --example standalone_rules --locked
```

The equipment example advances a caller-prescribed trajectory for 100 ms at
1000 Hz, then resets. This trajectory is an inspection policy, not native rotor
spin-up or Rune scheduling. The rule example submits three illustrative RFID
positions to the existing Tunnel traversal authority and earns ThroughTunnel.
It does not fabricate robot motion or claim physical tunnel traversal.

## Compose a physical session

Load the package through `cad_assets::load`, then call `Scenario::load` with a
`Preset`. `Scenario::session(duration, robots, initial_mechanisms)` creates a new
session. `RobotSpawn` supplies a physical team, chassis configuration and FLU
body pose; identities follow list order. Empty spawn lists are useful for
geometry, target and ray-query tests.

The definition cooks fixed/moving shapes once. New sessions and scenario resets
share immutable shape buffers; they do not reparse or retessellate CAD. Reset
rebuilds the initial bodies, initial mechanism coordinates and clock. It does not
restore an arbitrary intermediate continuation checkpoint. Controllers, RNG,
sensor acquisition state and task/reward state remain caller-owned.

`Scenario::targets_at(state, output)` resolves detector endpoints into a reused
vector. When changing prescribed motion, use the same state for collision and
targets before advancing one tick:

```rust,ignore
scenario.targets_at(&state, &mut targets)?;
session.physics_mut().sync_mechanisms(&state);
let contacts = session.step_with_targets(&targets)?;
```

The `standalone_scenario` example is the compiled version of this pattern.
Mechanism coordinates are finite radians or normalized slide fractions. Source
face IDs and original side numbering remain stable. `Scenario::targets` exposes
optional source detector rectangles; `Scenario::visual` exposes the original
checked GLB and root placement for a passive inspector. Use the package's glTF
root convention when assembling visuals; do not apply its placement twice.
No visual repainting, actuator calibration or mass/inertia inference is added.

## Modular observation adapters

`RfidLayout` borrows only the control, supply, Vantage and traversal footprints
needed for an experiment. It can be constructed without full structure bindings.
Its `terrain_cards(position, scratch)` reports raw physical entries into reusable
storage. Feed these to `gameplay::rmuc_2027::terrain::Terrain::observe(now_ns, cards)`
for a traversal-only authority, or call `RfidLayout::observe` with an existing
native `Match` for control/service effects and the normal eligibility checks.

The full world uses this same adapter. Its ordering remains all traversal
observations, then all control contacts, then own-team service observations.
Upper Highland/control roles retain their existing physical-card coalescing;
other overlapping cards remain ambiguous. Geometry queries neither grant RFID
grace themselves nor maintain buff timers. Losing a position reports no contact.

`DetectionContext::detect` applies the live source/artwork surface, caliber,
Rune ring, normal-speed and target-cadence gates, returning local y/z offsets or
the existing rejection type. Supply the source rectangle, the Rune's current
minimum ring and the target's last accepted detection timestamp as applicable.
The caller updates cadence memory only after acceptance. A detected contact is
not automatically damage: `combat_target` maps physical labels to native combat
identities, while Rune activation uses the existing Rune authority separately.

The live scorer uses these same gates and mapping. Damage, adaptive armor, team
ownership, XP, HP, detection history and every hidden rule timer remain in the
existing Field/Match state and restore format. Standalone Rune and Terrain
experiments may use their existing pure state machines with explicit time and
seeds; there is no duplicate implementation of their rules in the adapters.

A full match still observes positions, advances its authority, integrates physics
and scores contacts in the ordinary tick order. A standalone coordinator chooses
its own explicit schedule. Adapters advance neither clock. Sensor output remains
a measurement, while the true positions used for ideal referee occupancy are
privileged simulation data.

Native structure transitions are also available through borrowed
[`Outposts` and `Bases` views](equipment-rule-modules.md), used by the live Match.
These complement geometry/contact adapters; they do not replace match eligibility,
rewards or victory settlement. The existing pure Rune authority remains reusable.

## Optional inspector

The [RXUI inspector](scenario-inspector.md) is a separate optional workspace.
It consumes the source preset API for Outpost, Base and Rune, with explicit
pause/step/reset, mechanism coordinates, configurable omni spawns and physical
drive/torque/aim/fire controls. Headless scripts use the same model API and
record bounded raw armor contacts. Its orthographic collision views and
robot/projectile/detector/RFID overlays show privileged physical truth; they do not add gameplay
rules to the toolkit or replace source-material 3D rendering.

## Compose several components

[Composition](composable-scenarios.md) combines existing presets or checked
components with explicit additional FLU placements, static terrain, robot spawns
and caller-authored RFID volumes. It retains reverse source identity maps and
validates target/channel independence before preparing reusable cooked geometry.
Repeated instances cannot silently share a fixed mechanism channel.

## Independent rollout batches

[`RobotBatch` and `MatchBatch`](batched-rollouts.md) group independent sessions
with retained command/observation buffers, explicit progress reporting and
selected-set transactional resets. They support mixed integration rates without
a shared clock or extra scheduler. Cook scenario geometry once and capture shared
immutable shapes in recipes; each slot retains its own bodies, contacts and rules.
The implementation is sequential CPU execution, with measured stepping costs
comparable to ordinary per-session loops. Task/reward state and simulated sensor
sampling remain caller responsibilities.

[Parallel stepping](parallel-rollouts.md) optionally runs independent session
requests on bounded scoped CPU workers. Every admitted request is joined before
return and reports its own runtime progress; unlike sequential fail-fast stepping,
one failed slot does not cancel independent selected slots. Small dispatches may
cost more than inline stepping. [Borrowed telemetry](telemetry.md) avoids full
snapshot copies when reading wheel, payload, projectile and event truth.
