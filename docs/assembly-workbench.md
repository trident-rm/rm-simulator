<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Cartesian assembly workbench

The standalone `engineer_assembly` example supports a source-preserving PIE
Engineer, the RMUC2027 Tech Core positioning arm and front mechanism, and the
complete source Energy Unit. Cartesian controls run on the ordinary 128 Hz
clock; both six-axis rigs use the physics crate's serial solver. The Engineer
follows the gripper while the Core arm positions its socket at a 0.6 m world
height by default, or at a seeded rulebook pose with `--pose-seed`. Failed IK
refuses that tool movement without projecting the target.

```sh
nix develop -c cargo run -p rm-simulator-app --example engineer_assembly -- \
  --assembly-assets local-assets/assembly-cad/package
```

The package's manifest, configuration and five GLBs are verified before loading,
including in headless scenarios. The source Engineer keeps its chassis, wheels,
armor, camera, UWB and arm; its six joint groups and two gripper jaws replace
the generic beams. The Core keeps its pedestal, six-axis positioning arm, module
case and front mechanism. Original source mesh buffers and material tables are
preserved. The only omitted Engineer leaves are the previously approved motor
leads. No source model is embedded in the binary or added to Git.

Without `--assembly-assets`, the original schematic workbench remains available.
Its `--robot-assets` option accepts the verified PIE tunnel package as a chassis
under a generic arm. The two asset flags are mutually exclusive. Neither mode
moves the chassis; the unit starts aligned between the jaws. In source mode the
jaws begin open and close before the demo starts carrying the unit.

| Key | Action |
|---|---|
| W / S | Tool −x / +x, toward / away from this fixture |
| A / D | Tool +y / −y |
| R / F | Tool +z / −z |
| Up / Down | Increase / decrease P rotation |
| Left / Right | Increase / decrease Q rotation |
| Z / X | Off-axis roll, useful for testing misalignment |
| E | Confirm assembly |
| G | Open/release or close/regrip; releasing a seated unit fails |
| Space | Pause the workbench clock |
| 1 / 2 / 3 | Restart at that difficulty |
| Escape | Exit |

W/S moves toward/away from the socket in the fixture guide frame; A/D offsets
across it. The source mode constrains an attached unit to the source carriage
and P/Q bearing geometry, so rotating P moves the grasp point along its actual
arc while IK follows it. The HUD labels these internal coordinates **Guide**;
they are not official equipment OXYZ coordinates or world tool coordinates.

Move W until seated. At difficulty two or three, lift 100 mm using R. For
difficulty three, rotate P upward 90 degrees, then rotate Q to the displayed
30-degree target and hold alignment for 400 ms. Press E to confirm. The cyan
guide shows the insertion and lifting directions. Moving sideways after seating
removes the unit; this prototype does not supply guide contact forces.

`--demo` drives the same rate-limited tool through the sequence. `--difficulty`
chooses 1–3. `--headless` runs that demonstration without a GPU, prints the
serialized final authority and exits unsuccessfully if it cannot finish:

```sh
nix develop -c cargo run -p rm-simulator-app --example engineer_assembly -- \
  --headless --difficulty 3
```

`--screenshot local-assets/assembly.png` implies the demonstration and captures
the completed scene through an offscreen GPU target. Add `--closeup` to inspect
the source gripper and front mechanism. These are example options, not `rm-simulator` CLI options.

## Mechanism authority

`gameplay::rmuc_2027::manipulation::Attempt` consumes simultaneous, authoritative
mechanism observations and explicit monotonic nanosecond timestamps. Callers
must measure insertion, lift, P/Q bearing angles, axis/guide error and seating;
these are not world Euler angles or pilot-provided certificates. The workbench
derives its observations from the actual commanded tool pose. A later physics
adapter must derive them from the source socket/bearing frames and contacts.

UC §5.4.3 / Figures 5-12–5-15 supply the ordered 100 mm insertion, 100 mm lift,
90-degree P raise, five-degree Q tolerance, confirmation and one-second rejected
confirmation lockout. The simulator selects 400 ms from the manual's 0.3–0.5 s
indication range. Three-millimetre travel/guide tolerance and five-degree initial
axis/P tolerance are workbench policies; the manual does not specify them.

Difficulty four's evaluator tracks both cores and five-second paired insertion,
lift, alignment and confirmation milestones; P raising is exempt. Five seconds
is inclusive, while the 45-second overall completion deadline is exclusive.
Continuous RFID absence fails at fifteen seconds and death/removal fails
immediately. Timer expiry precedes an observation at the same time. Lower
difficulties have no evaluator timeout, unlike the separately documented live
button prototype. The interactive workbench currently provides only one arm and
one core and consequently exposes difficulties 1–3.

The evaluator serializes its hidden dwell, timing, lockout and milestone state.
Tests exercise correct and incorrect sequences, paired boundaries, contact
renewal, withdrawal, death and restore. `physics::manipulator::Arm` implements
the independent prescribed position solver; it does not enforce gameplay.

## Source gripper

The v4 source binding fits each TPU pad's rotary stop to the **actual mid-body**
of the Energy Unit. The 95 mm cap diameter is not the gripping diameter: triangle
sections through the pad's ±11 mm axial span give a 64.6 mm body envelope.
The exporter sweeps source pad triangles about each real jaw hinge, fitting a
stop with 0.1 mm envelope clearance. This is a conservative circular contact
proxy around the rods, not a force or exact rod-to-pad contact solution.

Both jaws move at a prescribed 1.5 rad/s, a simulator setting. The default demo
waits roughly 300 ms for them to reach the fitted stops before carrying the
unit through assembly. G opens/releases or closes/regrips. Closing only attaches
when both stops have been reached and the parked unit is within 5 mm and 5° of
the grasp frame; those capture tolerances are simulator policies. Closing empty
jaws remotely cannot attach the unit. The HUD and JSONL distinguish OPEN,
CLOSING and HELD, and recordings include both jaw angles and the actual unit
pose.

A released unit remains at its last fixture pose for regripping; it no longer
follows the empty tool. The empty tool does not drive Core travel. This is a
kinematic attachment, with no gravity, mass, motor torque or gripping-force
simulation. Opening after insertion reports detachment to the ordinary
manipulation evaluator and fails the attempt; restarting restores the initial
open jaws and aligned unit. Pausing also pauses the jaw actuator.

## Source calibration and reproduction

The exporter is `rm-map-tools/benchmarks/2026-10-18-team-robot/bind_assembly.py`.
It needs the reviewed PIE `materials.glb`, STEP, hierarchy and frame records in
`local-assets/robot-cad-survey`, plus the exporter's native equipment caches for
`technology-core/{pedestal-and-shoulder,arm-and-shared,tool-head}` and
`energy-unit/unit`. Those are reproducible upstream equipment-pipeline outputs;
they are not source files to commit. From the simulator checkout:

```sh
../rm-map-tools/ocpenv/bin/python ../rm-map-tools/python/audit_joint_axes.py \
  local-assets/robot-cad-survey/pie-2026-tunnel-engineer.step \
  --out local-assets/assembly-cad/pie-axes.json
../rm-map-tools/ocpenv/bin/python \
  ../rm-map-tools/benchmarks/2026-10-18-team-robot/bind_assembly.py \
  --sources local-assets/robot-cad-survey --audit local-assets/assembly-cad/pie-axes.json \
  --out local-assets/assembly-cad/package
```

PIE axes come from the source J1–J6 bearing cylinders and occurrence ownership,
with TPU jaw envelopes defining the grasp reference. The two jaw bearings are
separate: solids 1671 and 1670; solid 1684 is another bearing on the same line as
1670. Mistaking those for two independent jaw axes would break the clamp.
The source report §4.2 identifies the six-axis serial design, transmissions and
about 180 degrees of J4 motion. Other joints use ±π numerical guards; J4 uses
±π/2 about this CAD reference. These are workbench guards, **not measured stops**.
A fixed −90-degree grasp clocking and inverse unit offset keep the demonstration
inside that J4 range without rotating the unit's reference markings.

Core positioning bearings are audited from the three checksum-pinned native STEP
components. The mechanism dimensions were rechecked in the supplied **2026
University Championship V2.1.0 English manual, printed p82, Figure 5-7**, with
steps on pp91–92. The RMUC 2027 Figure 5-7 carries the same dimensions; this
cross-check does not substitute 2026 rewards or rules for 2027.
Source arm shells partition into six rigid links. The **v3** binding fits the
front mechanism against Figures 5-7 and 5-12–5-15:

- P is dimension-fitted from Figure 5-7: 208 mm above the initial post centreline,
  15 mm in front of the panel plane. The post tip reaches 169 mm from that plane,
  so P lies 154 mm behind E along the initial normal. Neither a small transverse
  fastener nor the 150 mm curved details establish this pivot.
- Shell 1430's 11.75 mm carriage spindle supplies Q. Shell 1224 supplies the
  insertion-post line and E locator, not Q. Q begins approximately vertical in
  the source pose; P carries that axis into the panel-normal direction.
- Shell 1430 follows the P/carriage body. Shells 1223–1429 and 1431–1595 follow
  the Q output, including the square marker face and post. Other head shells
  remain on the positioning flange.

The transform order is positioning delta × fixed-P rotation × carriage lift ×
Q rotation. The lift does not translate P. After the 100 mm lift, the initial post axis
remains 108 mm below P, so the flip must sweep forward and upward. A side-view
regression checks the final 54 mm/15 mm offsets of the mounting faces above/below
P against the drawing. Earlier bindings used a much lower pivot and did not
satisfy these dimensions. Consequently Q tilts the raised unit sideways, as in
Figure 5-15, rather than spinning it about its long axis. The prescribed demo
finishes insertion/lift/P waypoints exactly before starting the next motion;
referee acceptance tolerances do not blend the motions. The fixed Engineer
parking point moved 300 mm closer to retain reach along this corrected path.

The package fits an equipment OXYZ origin to the midpoint of the pedestal's
front floor edge, with +Z upward and +X toward the reference post's outward
side (source FLU +X). This explicit CAD calibration still needs placed-arena
orientation review. E uses a source post-axis locator at 100 mm from the chosen
cylinder gauge; it is approximate, not a certified sensor measurement. Candidate
placement checks a convex hull of the complete source head against the front
plane. These checks do not certify arm self-collision or moving clearances.
All source moving-part ownership and numerical joint guards remain hypotheses
for review; no bearing housings are replaced with procedural geometry. Audit
ordinals are OCCT import ordinals, never original STEP entity IDs.

The package contains 163,701 Engineer, 136,482 Core-arm, 39,661 Core-base,
156,103 Core-head and 44,518 Unit triangles: **540,465** total. This is a CAD
inspection package, independent of the game's optimized field package. Rest
vertex positions agree within 1.2e-16 m after reference-frame conversion;
primitive buffers remain byte-identical. That numerical preservation check says
nothing about whether every inferred moving-part assignment is mechanically right.
The package records input/output hashes, bearing evidence, ownership counts,
producer hash and a CAD-NOTICE retaining PIE and DJI provenance/terms.

```sh
nix develop -c cargo build -p rm-simulator-app --example engineer_assembly
python3 scripts/assembly-cad-scenarios.py --package local-assets/assembly-cad/package
```

The source scenario matrix completes difficulties 1/2/3, rejects modified
calibration, checks final IK residuals and guards against branch jumps. The
harness rejects a largest per-tick joint change of 0.1 rad or more.
The serial solver's numerical convergence bounds are 10 µm position and 0.1 mrad
orientation; these are solver tolerances, not physical calibration accuracy.

## Offline IK animation

The default fixed fixture preserves the earlier 600 mm socket height and source
orientation for before/after mechanism comparisons. It uses a fixed 30° Q target
and is not a Table 5-10 random sample.

`gameplay::rmuc_2027::assembly_pose` now samples seeded Table 5-10 candidates in
metres/radians: x ∈ [−0.1,0], y ∈ [0.1,0.3], z ∈ [0.5,0.7]. Level 1 fixes
θ/φ/α to 0°/90°/0°; levels 2–4 sample θ ∈ [−90°,90°], φ ∈ [0°,90°], and
α ∈ [−45°,45°]. The normal is `(sinφ cosθ, sinφ sinθ, cosφ)`, with roll about
that normal, not world Euler angles. Uniform angle sampling and the roll-zero
frame `Rz(θ) Ry(φ−π/2) Rx(α)` are explicit simulator policies.

The same seed/selection/difficulty gives both teams the same equipment-local
candidate. Keep the returned targets frozen throughout an attempt. Level 4
produces two candidates with shared x/z, a second y up to 400 mm lower, and
independent orientations/Q targets; independence is a simulator policy where
paired orientations are not specified. Q targets lie within ±90°. The generator
supports level 4; the one-gripper workbench still exposes only levels 1–3.

```sh
# JSON candidates for all four difficulties, without CAD or a renderer.
target/nix/debug/examples/engineer_assembly --generate-poses 27

# Select one exact candidate and solve it, without retrying other random poses.
target/nix/debug/examples/engineer_assembly \
  --assembly-assets local-assets/assembly-cad/package --difficulty 3 \
  --pose-seed 17 --pose-index 0 --headless \
  --record local-assets/assembly-cad/random-seed17.jsonl
```

The generator supplies the full bounding domain, **not a guarantee of legality
or robot reachability**. The CAD adapter rejects head overhang, then tries the
Core and Engineer IK. It reports the exact failed candidate and never silently
resamples, shortens the movement or expands joint guards. A 48-candidate audit
(seeds 0–15 at levels 1–3) exposed positioning/Engineer IK gaps, including no
accepted level-1 sample in that scan. With v3, level-3 seed 17 completes the full sequence; seed 15 is no longer a
passing trajectory. The earlier 48-candidate scan used v2, so its successes are
not v3 coverage claims.
This is not full-domain physical support. Joint calibration, arm workspace,
gripper clocking and robot parking require further work before claiming that.
The headless solver stops at the first unreachable trajectory sample.

Solve the complete trajectory without a renderer, then render the recorded
source link poses. The renderer never runs IK or invents joint interpolation:

```sh
nix develop -c cargo build -p rm-simulator-app --example engineer_assembly --locked
target/nix/debug/examples/engineer_assembly \
  --assembly-assets local-assets/assembly-cad/package --headless --difficulty 3 \
  --record local-assets/assembly-cad/offline-d3.jsonl

../rm-map-tools/ocpenv/bin/python scripts/render-assembly-video.py \
  --package local-assets/assembly-cad/package \
  --motion local-assets/assembly-cad/offline-d3.jsonl \
  --out local-assets/assembly-cad/assembly-offline-overview.mp4
```

Add `--closeup` to the renderer for the gripper/module view. Add `--side` for an
orthographic check of the flip arc and `--mechanism-only` to hide the Engineer.
The side view marks the fixed P point and traces the post tip; it does not
change the trajectory. It requires the
rm-map-tools Python environment, its diagnostic C++ rasterizer, `clang++` and
FFmpeg. These are diffuse CAD inspection renders, not in-game recordings.
Original source material colors are retained, with simple diagnostic shading.
The overview camera fits the entire recorded motion rather than following it.

The JSONL header pins source GLB hashes and states the fixed case and simulation
clock. Samples carry the actual FK tool, joint values, absolute link frames,
mechanism angles, completion state and solver residuals. IK runs at 128 Hz with
the prior solution as its seed; recording samples every four ticks (32 Hz),
including initial/final poses. Video adds one-second endpoint holds, preserving
motion speed. Asset hash mismatches or incomplete attempts refuse rendering.
The scenario harness checks all recorded residuals, joint guards, unit
quaternions, sample timing and difficulty-3 stage coverage.

This is prescribed kinematic planning: it verifies numerical reachability and
branch continuity along the fixture guide. It does not search around obstacles,
certify arm self-collision or simulate contact/gripping forces. A successful IK
trajectory alone does not establish a mechanically feasible assembly.

## Remaining integration

This is an isolated workbench, without match networking, Heavy deployment,
progression or rewards. It cannot certify a live match. The existing referee
Start/Finish/Cancel path remains the only live assembly progression authority.

The source rigs prescribe joint positions. They do not solve motor torque,
loads, self-collision, donor wiring limits or contact forces. The source gripper
now has a fitted contact-envelope stop and gated kinematic attachment; a released
unit remains parked rather than becoming a simulated falling rigid body. Source motion currently follows the fixture's constrained mechanism
coordinates; a contact-based adapter must measure those from actual bodies and
socket/bearing frames. The default comparison fixture keeps the CAD socket orientation. Seeded targets
use the exporter's fitted equipment frame and the manual's spherical pose ranges;
not every candidate can be positioned or manipulated by the current source rigs.

Before using this with a live Heavy: review rigid ownership and moving poses,
measure actual joint limits, introduce contact/attachment and force behavior,
put commands and complete physical restore state through the existing host
protocol, and connect Heavy ownership, RFID, defeat/recovery, invincibility and
rewards. Difficulty four additionally needs two energy units and a supported
manipulation strategy. The interactive workbench currently has one gripper.
