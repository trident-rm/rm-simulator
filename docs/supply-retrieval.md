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
| Grasp stiffness | 5000 N/m | Simulator assumption; limits nominal upward-load deflection to about 2.8 mm |
| Grasp damping | 10 Ns/m | Simulator assumption |
| Arm resultant-force limit | 20 N | Simulator assumption |
| Grip resultant-force limit | 25 N | Simulator assumption |

The stiffness keeps nominal upward-pull deflection near 2.8 mm, within the
tested wrist/cap clearance. It is not measured donor compliance. Jog collision
guards distinguish a retained grasp from carried cargo: jaws may contact the
gripped Unit, while its measured station-constrained pose and all other link
collisions remain checked.

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

## Record a live retrieval

Start the embedded source Heavy with a loopback listener:

```sh
cargo run -p rm-sim-app --bin rm-simulator -- \
  --listen 127.0.0.1:7700 --robot engineer --team red \
  --cad-assets field-2027 \
  --robot-assets local-assets/robot-cad-survey/bound-tunnel-heavy \
  --assembly-assets local-assets/assembly-cad/package \
  --assembly-area local-assets/assembly-cad/area \
  --console --start-paused --window-mode headless --graphics-preset low
```

Start a second app for the free spectator camera, using the same source assets:

```sh
cargo run -p rm-sim-app --bin rm-simulator -- \
  --connect 127.0.0.1:7700 --fly --console 127.0.0.1:7791 \
  --robot-assets local-assets/robot-cad-survey/bound-tunnel-heavy \
  --assembly-assets local-assets/assembly-cad/package \
  --assembly-area local-assets/assembly-cad/area \
  --window-mode headless --graphics-preset low
python3 scripts/render-supply-retrieval-video.py --park \
  --out local-assets/supply-retrieval-video/retrieval.mp4
```

The script uses operator placement before recording and a fixed side-entry path
to the red station's top Unit. It is a recording fixture, not a general retrieval
planner. The open forks enter around the 64.6 mm body from the side; the 95 mm
caps cannot pass through that opening. Ordinary bounded Cartesian jogs, jaw
actuation and force-limited withdrawal produce the motion. Pull jogs are 1.5 mm
so collision validation can use the latest actual constrained Unit pose.
The script never assigns Unit poses, attachments or force parameters.

Screenshots come from the real app renderer and advance on explicit 128 Hz ticks.
FFmpeg encodes 1280×720 H.264 at 30 fps with phase/limit captions. The adjacent
JSON retains the selected identity, mount/limits, actual joint/jaw samples,
withdrawal travel and final attachment. The runner checks the 1 rad/s actuator
limits. `--dry-run` skips the spectator, screenshots and encoding.
