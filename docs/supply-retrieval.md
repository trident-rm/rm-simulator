<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Supply retrieval

Source-backed RMUC fields start with six Energy Units at each resource station.
The Heavy deploys empty-handed. Drive to the station, align the source gripper
with a Unit's bore, close it using J → Grip / release Unit, wait for the jaws,
and withdraw along the bore axis with the existing Cartesian jog controls.
The panel reports withdrawal travel. Assembly becomes available after extraction.
No automatic harvesting or route planner is installed.

## Rules and mechanical assumptions

RMUC 2027 V1.0.0 §4.2.6, printed p.45, Figures 4-21–4-22, specifies six
magnetically retained Units per station. Withdrawal follows the symmetry axis,
requiring at most 10 N in addition to gravity. Figure 4-39 specifies 400±50 g;
the simulator uses 400 g. Axial clocking can vary in the real installation.
The current source CAD supplies the initial clocking, not a randomized tolerance.

| Parameter | Default | Basis |
|---|---:|---|
| Magnetic resistance | 10 N | Rulebook upper bound; not a measured magnet |
| Unit mass | 0.4 kg | Rulebook nominal mass |
| Guide clearance | 20 mm | Simulator assumption |
| Grasp stiffness | 500 N/m | Simulator assumption |
| Grasp damping | 10 Ns/m | Simulator assumption |
| Arm resultant-force limit | 20 N | Simulator assumption |
| Grip resultant-force limit | 25 N | Simulator assumption |

The solver computes spring/damping force from the actual wrist displacement,
caps its resultant at the arm limit, and tests that transmitted force against
grip capacity. Equality does not slip. Gravity contributes separately to axial
acceleration. A weak arm stalls; an overloaded grip slips. Transverse force
uses the same force budget but cannot extract the Unit. Tilting the bore more
than 5° loses the grasp. Pickup still requires the existing 8 mm/5° capture
alignment and closed jaws before pulling. These tolerances are simulator policies.

Until clearance, the Unit has one translational degree of freedom. A rigid guide
reacts transverse loads and resists inward movement beyond its mounted pose.
The constant magnetic force opposes outward travel. On opening/slipping, a partly
withdrawn Unit returns along the guide on subsequent ticks without teleporting.
There is no inventory replacement during a round. RMUC referee ResetMatch returns
the same source identities to their mounts for the next round. Removal of the
carrier also releases its grip.

This is a reduced force model for retrieval. It does not solve dynamic arm
joints, contact pressure, jaw friction cones, magnetic fields or chassis reaction
loads. After clearance, the existing prescribed carry model resumes using the
actual endpoint and a newly captured wrist-to-Unit transform. Carrying and Core
insertion do not become force-controlled. The 20 mm clearance represents loss of
station engagement; a measured force-distance curve can replace that policy later.

## Ownership and source geometry

`sim-engine::retention` owns the solver-free axial law. `SupplyMount` in
`rm-sim-world::energy_unit::supply` owns RM limits, source pose and persistent
withdrawal state. The ordinary explicit world tick integrates it in slices no
longer than 1 ms. There is no extra clock, worker or physics world.

`rm-sim-assets::supply::load` checks the arena visual checksum, identifies the
authored `energy-unit/unit` components and retains their ancestor transforms.
The native Unit's +y bore and 75 mm midpoint become a centred +z extraction
frame. Exactly twelve source occurrences, six per side, are required when those
components are present. Partial records fail instead of inventing mounts.
Its convex outer proxy can fill bores and concavities, as with existing loose
payload proxies. The renderer moves the original source mesh nodes, preserving
their geometry and materials throughout pickup, carrying and discharge.

The interactive terrain loader removes those Unit nodes from both contact and
support/query scenery before installing persistent objects. The station's hub,
shafts and frame remain static. Prescribed Units remain query-visible sensor
shapes, so LiDAR sees them, while suspension rays exclude sensors. They do not
apply prescribed contact forces. Released free Units use ordinary Rapier contacts.
Standalone `PreparedArena::load` retains its original static scenery behavior;
callers selecting interactive terrain must install its `supply_units` inventory.
Server `layout::add_terrain` installs that inventory automatically.

Stable inventory IDs, mount poses, force parameters, withdrawal position/velocity,
grip/slip state and source-node identity travel in full checkpoints. Protocol 84
requires matching clients and hosts. Spent Units remain ineligible under R60.
Bare test fixtures without source inventory retain deployment preload.

## Headless checks

```sh
cargo test -p sim-engine retention
cargo test -p rm-sim-world energy_unit::supply
cargo test -p rm-sim-assets --lib supply::tests -- --ignored
cargo run -p rm-sim-server --example supply_station -- field-2027
```

The source test requires the checked-in LFS package. It checks both stations'
outward axes and that static exclusion preserves all other mesh owners. The
headless probe checks installed stock, partitioned stepping and checkpoint
continuation. Mechanical tests cover weak pulling, grip failure, gravity,
sideways pulling, actual endpoint capture, mid-pull restore and magnetic return.

One local CPU-only Mac probe measured 0.235 s startup and 22.05 ms for 128 ticks
(0.172 ms/tick), with one default physical Engineer chassis and twelve mounted
Units in the full source field. Peak resident memory was 78.5 MiB, including
the checkpoint/restore comparison worlds. Compilation, the source arm planner
and rendering are excluded. This is one sample, not a timing distribution.
