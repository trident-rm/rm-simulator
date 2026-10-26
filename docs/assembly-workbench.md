<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Cartesian assembly workbench

The standalone `engineer_assembly` example supports a source-preserving PIE
Engineer, the RMUC2027 Tech Core positioning arm and front mechanism, and the
complete source Energy Unit. Cartesian controls run on the ordinary 128 Hz
clock; both six-axis rigs use the physics crate's serial solver. The Engineer
follows the gripper while the Core arm positions its socket at a 0.6 m world
height by default, or at a seeded rulebook pose with `--pose-seed`. Failed IK
refuses that tool movement without projecting the target. Use `--assembly-area`
for the corrected arena OXYZ and standing terrain described below; the default
flat fixture remains a legacy comparison.

```sh
nix develop -c cargo run -p rm-sim-app --example engineer_assembly -- \
  --assembly-assets local-assets/assembly-cad/package
```

The package's manifest, configuration and five GLBs are verified before loading,
including in headless scenarios. The source Engineer keeps its chassis, wheels,
armor, camera, UWB and arm; its six joint groups and two gripper jaws replace
the generic beams. The Core keeps its pedestal, six-axis positioning arm, module
case and front mechanism. Original source mesh buffers and material tables are
preserved. The explicitly omitted Engineer leaves are the previously approved motor
leads and protruding camera cable harnesses. No source model is embedded in the binary or added to Git.

Without `--assembly-assets`, the original schematic workbench remains available.
Its `--robot-assets` option accepts the verified PIE tunnel package as a chassis
under a generic arm. The two asset flags are mutually exclusive. The bare fixture keeps its chassis fixed; area-mode source planning can choose
a coordinated chassis route during lift/flip. The unit starts aligned between
the jaws. In source mode the
jaws begin open and close before the demo starts carrying the unit.

| Key | Action |
|---|---|
| W / S | Tool −x / +x, toward / away from this fixture |
| A / D | Tool +y / −y |
| R / F | Tool +z / −z |
| Up / Down | Increase / decrease P rotation |
| Left / Right | Increase / decrease Q rotation |
| Z / X | Off-axis roll, useful for testing misalignment |
| O | Plan/run all remaining stages; resume an existing plan automatically |
| P | Plan/run one stage, then hold at the next stage boundary |
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

`--demo` plans the source-CAD sequence before playback; without source assets it
drives the original schematic guide. `--assisted` prepares the fixture and runs
one stage at a time with P. O resumes automatic playback. `--difficulty`
chooses 1–3. `--headless` runs that demonstration without a GPU, prints the
serialized final authority and exits unsuccessfully if it cannot finish:

```sh
nix develop -c cargo run -p rm-sim-app --example engineer_assembly -- \
  --headless --difficulty 3
```

`--screenshot local-assets/assembly.png` implies the demonstration and captures
the completed scene through an offscreen GPU target. Add `--closeup` to inspect
the source gripper and front mechanism. These are example options, not `rm-simulator` CLI options.

For controlled source-CAD experiments, `--core-pose X Y Z THETA PHI ALPHA`
sets a frozen Table 5-10 pose (metres, then degrees), and `--q-deg DEGREES`
sets difficulty three's Q target within ±90°. Explicit Core poses and
`--pose-seed` are mutually exclusive. `--chassis-offset DX DY YAW` applies an
initial offset in rig metres/degrees after terrain placement. Add
`--fixed-chassis` to prevent both parking search and chassis motion, while still
searching arm branches, grasp clocking and camera posture. The reported absolute
initial pose is authoritative: offsets are relative to the source loader's
initial supported parking, which can vary with the requested Core pose.

`--spin-deltas APPROACH INSERT LIFT FLIP ALIGN` prescribes five axial twist
increments in degrees, each within ±180° as a workbench policy. It replaces
the automatic spin-profile family for that run. Twists accumulate continuously
through stage boundaries and retain the rigid grasp. Without this flag,
planning searches the profile family below. These controls prepare a fixture
before tick zero; they cannot move a robot that is already holding a Unit.
Explicit nonzero twists for absent stages are rejected. Alignment twist needs a
nonzero Q target; stationary twist-only segments are not represented yet.

`--ik-hints COMPLETED.jsonl` imports body-local Engineer tool/joint samples from
a completed recording with the same source configuration checksum. Import also
checks finite poses, joint bounds and FK consistency. These are numeric warm
starts, not clearance certificates. They can seed nearby Core/chassis cases;
every new target is solved and the resulting path receives ordinary collision
validation. `assembly-pose-matrix.py --hints COMPLETED.jsonl` applies the same
hint recording to each case and pins its checksum.

```sh
target/nix/debug/examples/engineer_assembly \
  --assembly-assets local-assets/assembly-cad/package \
  --assembly-area local-assets/assembly-cad/area --difficulty 3 --headless \
  --core-pose -0.05 0.2 0.5 0 90 0 --q-deg 30 \
  --fixed-chassis --chassis-offset -0.1 -0.15 15 \
  --spin-deltas 0 0 0 90 0
```

## Reusable source loading

`rm_sim_assets::semantics::assembly::SourcePackage` verifies the approved
PIE package and supplies source calibration, a source-bound area frame and an
immutable `physics::manipulator_collision::CollisionScene`. It requires no host,
match authority or renderer. Calibration is deserialized into a caller-owned
type after the existing source arm, bearing and jaw checks. The shared
`rm_sim_control::assembly::Rig` owns the loading and planning entry points.

`control::manipulator_plan::checked_ik_hint` validates recorded body/tool poses
and joint coordinates against FK before returning a body-local warm start.
This numerical check supplies no collision certificate. Parking search and staged
planning now live in `rm-sim-control`; worker lifetime, cancellation decisions
and live world playback remain in the server;
this extraction does not change their budgets, target revelation or permissions.

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

The v5 source binding retains the v4 fit of each TPU pad's rotary stop to the **actual mid-body**
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
referee acceptance tolerances do not blend the motions. The v3 fixed parking point moved 300 mm closer to retain reach along this
corrected path; v5 instead searches bounded parking offsets and IK branches
for a clear initial configuration.

The package retains the legacy comparison origin at the metal pedestal's
front floor edge. Figure 5-6 identifies the field base's front wall as the actual
OXYZ reference; area mode below supplies that corrected frame before placement.
The legacy origin alone cannot certify a rulebook pose. E uses a source post-axis locator at 100 mm from the chosen
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
nix develop -c cargo build -p rm-sim-app --example engineer_assembly
python3 scripts/assembly-cad-scenarios.py --package local-assets/assembly-cad/package
```

The source scenario matrix reports completed or collision-blocked paths for
difficulties 1/2/3, rejects modified calibration, checks retained IK residuals
and guards against branch jumps. The
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
accepted level-1 sample in that scan. With v3, the unguarded level-3 seed 17 completed the full sequence; seed 15
was no longer a passing trajectory. Those results did not check collisions and
are not v5 physical-clearance claims. The earlier 48-candidate scan used v2, so its successes are
not v3 coverage claims.
This is not full-domain physical support. Joint calibration, arm workspace,
gripper clocking and robot parking require further work before claiming that.
The headless solver stops at the first unreachable or collision-blocked
trajectory sample.

For a path that completes, solve it without a renderer, then render the recorded
source link poses. The renderer never runs IK or invents joint interpolation:

```sh
nix develop -c cargo build -p rm-sim-app --example engineer_assembly --locked
target/nix/debug/examples/engineer_assembly \
  --assembly-assets local-assets/assembly-cad/package --headless --difficulty 2 \
  --record local-assets/assembly-cad/offline-d2.jsonl

../rm-map-tools/ocpenv/bin/python scripts/render-assembly-video.py \
  --package local-assets/assembly-cad/package \
  --motion local-assets/assembly-cad/offline-d2.jsonl \
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

Recordings contain prescribed kinematics, not motor or contact dynamics. The
legacy guide verifies numerical reachability and branch continuity only; the
staged planner additionally searches branches and free-space detours and applies
collision guards. Neither numerical IK nor a finite guarded search certifies
calibrated hardware or gripping forces.

## Collision-guarded Cartesian motion

The source-CAD workbench now rejects obstructed poses and swept motion by
default. Its existing six-axis damped-least-squares IK still converts Cartesian
gripper targets into joint positions. IK establishes reachability; it does not
search around obstacles or choose a collision-free assembly strategy.

The v5 exporter adds checksummed `collision.json`, pinned to all five visual
GLBs. Engineer, jaw, unit and Core-head shapes retain source triangles and
apertures. The Core positioning arm uses one convex envelope per source leaf,
which can conservatively block a concavity. No hull spans an articulated link
chain. Runtime hull envelopes accelerate rejection of separated mesh pairs;
only the source surfaces decide a mesh collision. The fixture includes a floor
at z=0 in the bare comparison fixture; area mode replaces it with verified
terrain. Camera and UWB groups are attached to the forearm; the camera can use
its source posture during area-mode preparation, while UWB keeps its retraction. The jaw drive gears now follow their
correct source hinge sides.

`physics::manipulator_collision::CollisionScene` is renderer/gameplay-free.
It checks named poses, exact mesh intersections and conservative distance-based
rigid-frame sweeps, including rotation and moving obstacles. Unsupported queries,
invalid inputs and exhausted advancement budgets fail closed. Surface meshes
have no inferred solid interior or meaningful penetration depth. Solid contacts
use a 0.5 mm penetration tolerance; surface crossings and swept near-contact
(with a 2 µm numerical margin) still block. A swept diagnostic point is a link
origin, not an exact impact location.

The offline adapter subdivides joint changes to at most 0.01 rad and payload/lift travel
to at most 2 mm, then sweeps each frame between samples. This is a piecewise
rigid approximation of the joint path, not an exact continuous articulated
collision certificate. A transition exceeding 256 subdivisions is rejected.
Arm, jaws, payload and Core travel commit together, or remain at their previous
valid values. The HUD and JSON/JSONL report the named blocking pair. Failed
headless paths exit nonzero, and their last frame records the rejection.
`--screenshot` captures either completion or the blocked pose, including its HUD.

Exemptions are explicit: neighbouring bearings, the nested wrist stack,
coupled jaw gears, fixed auxiliary mounts and guided Core carriage interfaces.
Jaw/unit contact is allowed only during a nearby aligned grasp or while held;
unit/post contact requires the socket's narrow axial/lateral/alignment window.
These exceptions do not exempt the arm from the chassis or Core housings.

A bounded deterministic parking/IK-branch search selects a clear initial pose
when possible; it does not resample the requested Core target. Stationary
Core/pedestal/floor intersections are reported before trying robot placements,
since parking cannot repair them. If no clear
candidate exists, the initial collision is reported and motion remains blocked.
Restart restores that initial fixture. The diagnostic `--legacy-guide` is **not a collision-free plan**: source
geometry exposes chassis/J2 interference along it. The staged planner searches
other branches and grasp frames instead. A blocked demo must not be counted as a completed assembly. Detailed
surface queries also remain more expensive than IK; this is a workbench guard,
not a measured real-time full-arena controller.

```sh
target/nix/debug/examples/engineer_assembly \
  --assembly-assets local-assets/assembly-cad/package --collision-audit
python3 scripts/assembly-cad-scenarios.py --package local-assets/assembly-cad/package
# Require every prescribed path to finish, rather than reporting blocked paths:
python3 scripts/assembly-cad-scenarios.py --package local-assets/assembly-cad/package --require-completion
```

The scenario report separates completed and blocked paths, validates the retained
FK/joint state and checks checksummed assets. Synthetic tests separately verify
thin-wall tunnelling rejection, rotating links, mesh apertures, zero-depth
surface crossings and transactional rollback/recovery. This adds neither arm
contact forces nor a global motion planner. Live Heavy/network integration
remains separate.

## Staged collision-aware planning

Source `--demo`, `--headless` and `--screenshot` now use the staged planner by
default. `--legacy-guide` retains the old direct guide for collision diagnostics.
`--assisted` prepares a fixture and waits for P at stage boundaries; O switches
to automatic completion. Space pauses playback. G or Cartesian input cancels
an existing plan; an interrupted free transit holds the guide until a new O/P
return plan or restart, rather than snapping to the socket guide.

The sequence is free approach (when needed), grasp, approach to the entrance,
insertion, lift, P flip, Q alignment, dwell and referee confirmation. Levels 1/2
stop after their required subsets. Once seated, paths stay on the prescribed
mechanism guide; random joint detours are prohibited there. Confirmation still
goes through the ordinary manipulation evaluator.

`control::manipulator_plan` provides deterministic distinct IK candidates and a
bounded bidirectional joint-space RRT-Connect with swept validation and checked
shortcuts. The workbench searches grasp clockings and source-compatible axial
grip positions, continues each IK branch through the stage milestones, and then
validates every planned 128 Hz frame with the ordinary collision guard. Live
playback checks those frames again. No partial plan is applied on failure.
A 64-tick settle gives the evaluator its alignment dwell before confirmation.

Before an attempt, the prepared-fixture mode may select another Core IK branch,
robot parking offset/yaw and initial Engineer branch. The socket's selected
world pose is preserved; this is fixture preparation, **not a simulated drive or
teleport during an attempt**. The search has nine parking offsets, eight yaws,
eight grasp clockings, two axial choices, up to eight Core IK solutions and up
to twelve Engineer candidates per setup. It chooses the first clear Core branch;
it does not exhaustively combine every Core branch with every robot setup.
Candidate seeds also include a bounded 48-point low-discrepancy set. None of
these finite searches proves physical infeasibility when it fails.

The alternate grasp is 20 mm toward the exposed end of the unit. Source triangle
sections and all vertices across the entire pad band z=9..31 mm bound its radius
at 32.3 mm, so the existing jaw stops remain applicable. Grasp clocking and axial
offset change the tool/unit transform, not the unit's socket pose or source mesh.
The selected values appear in JSON/JSONL. These are kinematic grasp choices,
not friction/load certifications.

O/P in an already opened workbench, or `--plan-current`, preserves the current
chassis, Core, payload and arm poses. An open gripper may retreat in Cartesian
space, traverse a collision-checked joint path, approach and close at a new grasp.
The unit stays parked until captured. A held/seated unit cannot switch grasp
branches. An off-axis guide state outside the supported mechanism guide is
rejected explicitly. Interactive planning runs on a worker and holds the
workbench clock; pause/restart/manual cancellation never installs an old worker
result. A carried unit keeps its selected grasp for the remaining stages.

`--plan-budget N` bounds charged pose/transition checks (default 8000). Each
transition also has the collision guard's subdivision bound; RRT extensions,
connect attempts, seed sets and setup lists are finite. Numerical IK attempts
are bounded separately by those lists and the serial solver's iteration limit.
Reports include candidate/check counts, rejected stages and blocking pairs.
Planning failure exits nonzero in headless mode and leaves the live fixture
unchanged. It is distinct from an IK failure during playback.

```sh
# Prepared fixture, complete guarded playback for the source donor's level 2.
target/nix/debug/examples/engineer_assembly \
  --assembly-assets local-assets/assembly-cad/package --difficulty 2 --demo
# Prepared fixture, assisted stage-by-stage control.
target/nix/debug/examples/engineer_assembly \
  --assembly-assets local-assets/assembly-cad/package --difficulty 2 --assisted
# Preserve the existing fixture and search a free-arm approach.
target/nix/debug/examples/engineer_assembly \
  --assembly-assets local-assets/assembly-cad/package --plan-current --headless
python3 scripts/assembly-planner-scenarios.py --package local-assets/assembly-cad/package
```

The reviewed fixed CAD fixture now completes levels 1 and 2. The default level-3
fixture still yields a bounded planning failure, including P-flip IK failures;
this donor's inferred joint guards/ownership and workspace need further review.
The current-fixture free search and seeded Core poses may also fail. Exact
source-CAD free searches can take minutes; the prepared fixture is the verified
fast path, while current-pose planning remains experimental. The scenario
report separates these failures from successful assemblies. It requires levels
1/2 to complete, checks deterministic replay and rejects exhausted budgets
without moving the live rig. Numerical success remains separate from calibrated
hardware, contact forces and live Heavy integration.

## Actual Assembly Area terrain (2026-10-19)

The old flat fixture put the Engineer and metal Core pedestal on the same
plane. **That is not their relative placement in the arena.** UC §4.3.2.1,
Figures 4-28/4-29 (printed pp51–52) show assembly on the central highland under
the Rune. The supplied CAD puts the robot's deck **250 mm above the metal
pedestal's mounting floor**. Queries in the rig frame at (0,0) and
(1.4,-0.511338) return 0 and 250 mm. The metal pedestal floor itself is about
149 mm above the arena slab datum. The 250 mm difference is measured from CAD;
it is not explicitly dimensioned in those rulebook figures.

There is also an OXYZ calibration correction. UC §5.4.3 / Figure 5-6 (printed
p79) puts O at the **field base's front wall/ground intersection**, not at the
metal arm pedestal's edge. Fitting the CAD wall that matches Figure 4-29's
2199 mm span gives O=(928.000,-747.915,250.000) mm in the unchanged rig frame;
its measured span is 2198.597 mm. The calibration is source-derived and remains
a reviewable fit, not a certified survey. Its +X points out toward the robot;
+Z is up. Thus Table 5-10's z=500–700 mm corresponds to rig-frame z=750–950 mm,
**500–700 mm above the robot deck**, not above the metal pedestal floor.
Elevating the robot without converting the rulebook target frame would make the
task artificially low.

Generate a separate, disposable terrain extraction and enable it explicitly:

```sh
../rm-map-tools/ocpenv/bin/python scripts/bind-assembly-area.py \
  --package local-assets/assembly-cad/package --field field-2027 \
  --out local-assets/assembly-cad/area
nix develop -c cargo run -p rm-sim-app --example engineer_assembly -- \
  --assembly-assets local-assets/assembly-cad/package \
  --assembly-area local-assets/assembly-cad/area --difficulty 3
```

Area mode uses a fixed legal OXYZ=(-50,200,500) mm pose, horizontal shaft,
zero roll and the existing 30° Q target. It intentionally tests the lower end
of Table 5-10's height range. `--pose-seed` instead selects its explicit frozen
candidate. The fitted area frame is applied **before Core IK and the head
front-plane check**, then terrain is installed. The no-area mode retains the
old comparison frame and geometry for reproducible diagnostics; its legacy
metal-pedestal frame must not be used to certify official arena placement.

The generator verifies runtime visual/collision hashes and the common source
STEP, then uses the original pedestal occurrence's hierarchy matrix and the
native-to-rig calibration. It retains whole intersecting terrain triangles and
original material colours from `floor` and `arena-static`, without a procedural
platform or duplicate resting Core. The current extraction has 11,036 collision
triangles. Its schema-2 evidence includes the fitted OXYZ, wall bounds/span,
source transforms/checksums and output hashes. Generated files and CAD notice
stay outside Git. This is terrain context and one moving Core; a second moving
Core/Engineer remains outside the workbench's scope.

The loader verifies the extraction against the loaded rig configuration and
replaces the flat floor collision body with the actual terrain. Initial and
candidate parking sample a conservative 500×500 mm four-corner footprint,
require all four heights within 5 mm, and derive stance height from the source
chassis mesh's lowest tyre point. A 1 mm inspection clearance prevents supporting
mesh tangency from being treated as crossing; this is not suspension/contact
simulation. Initial preparation can select a clear open grasp, and area-mode
planning considers six additional farther parking offsets. Unsupported or
step-straddling stances are rejected. The fixed pedestal/plinth mount is excluded
from collision queries; all moving Core and robot versus terrain pairs remain
checked. The installed fixed pedestal's approximate hulls otherwise intersect
its plinth by about 4.7 mm.

A corrected-frame test reached alignment checks after passing flip IK, unlike
the flat fixture's flip-only failures. Its 2,936 candidates and 2,852 charged
checks included one alignment IK rejection and still found no complete guarded
path. Reaching a milestone IK/endpoint is not verification of
the complete swept path. Real-area tests therefore do not yet establish a
completed level-3 assembly. The blocked app inspection shows the actual terrain
and legal low target. Further joint/ownership and task-wide branch work remains.

### Segmented branch and placement search

Area-mode preparation tries 24 relative chassis yaws at 15° spacing and
35 parking offsets (15 original/farther positions plus 20 intermediate lateral
positions). Unsupported terrain footprints are discarded before IK. A fixed
integer permutation spreads the collision-check budget across parking, yaw and
grasp choices instead of exhausting the first parking first. This is a bounded
search, not an exhaustive feasibility proof.

The planner uses a Cartesian segment IR for approach, insertion, lift, P flip
and Q alignment. Each segment carries exact start/end guide coordinates; adjacent
boundaries agree. It solves final alignment branches backwards through individual
segments to obtain alternative grasp seeds. If that suffix fails, it solves an
earlier junction independently and traces its prefix backwards, then tries to
connect the remaining suffix forwards. Connections keep the same initial chassis
pose, prescribed route, Core positioning branch and grasp: independently reachable endpoints at different
parkings do not constitute a connected path.

Forward candidate checking rejects occupied milestone endpoints. A separate
32-query prefix-audit allowance sweeps segments at up to 5 mm / 0.02 rad guide
intervals through the ordinary articulated collision guard, assuming the closed
grasp. Successful segment witnesses survive later segment failure in the
diagnostic report; partially audited segments are not promoted to witnesses.
This allowance limits expensive terrain sweeps for incomplete candidates.
Complete candidates are still regenerated at
128 Hz, including actual jaw closure, and replayed with the execution guard.
Only a complete plan can be installed; partial witnesses never move the live rig.

Reports distinguish `connected_segments` (reverse IK only),
`milestone_witnesses` (clear endpoints), `coarse_segment_witnesses` (guarded
coarse guide prefixes) and `swept_segment_witnesses` (128 Hz execution prefixes).
Each witness pins chassis pose, grasp clocking/axial offset and endpoint joints.
Area-mode prepared plans can include the coordinated base trajectories described
below. Current-pose assisted planning retains fixed parking; it never switches
to an unrelated prepared setup after the unit is gripped.

The broadened fixed-area level-3 run tested 3,638 setup/grasp combinations and
8,485 root candidates before its 8,000-query budget expired. Its partial audit
verified an Approach sweep, but no complete plan was found. The dominant
rejection was `engineer/body / engineer/j2` (5,254 occurrences). These totals
are a bounded diagnostic, not proof that assembly is mechanically impossible.

### Source-leaf collision inspection

The ignored `audit_chassis_shoulder_collision_leaves` test isolates the dominant
pair to individual original collision leaves, using the same exact triangle
queries as the full guard. It checks both the zero-joint source pose and
independent initial IK branches. It writes pinned source hashes, link poses,
joints and intersecting leaf indices; no collision exclusion is changed.

```sh
ASSEMBLY_CAD_PACKAGE="$PWD/local-assets/assembly-cad/package" \
ASSEMBLY_CAD_AREA="$PWD/local-assets/assembly-cad/area" \
ASSEMBLY_COLLISION_AUDIT="$PWD/local-assets/assembly-cad/area-shoulder-leaf-audit.json" \
nix develop -c cargo test -p rm-sim-app --example engineer_assembly \
  --all-features --locked audit_chassis_shoulder_collision_leaves -- --ignored
../rm-map-tools/ocpenv/bin/python scripts/render-assembly-collisions.py \
  --package local-assets/assembly-cad/package \
  --audit local-assets/assembly-cad/area-shoulder-leaf-audit.json \
  --hierarchy local-assets/robot-cad-survey/pie-2026-tunnel-engineer-hierarchy.json \
  --out local-assets/assembly-cad/collision-review
```

The inspected source has one suspicious chassis-assigned leaf: collision part
1286, source solid 1693, occurrence `1/3` (`零件3^2026工程-分区赛版本`). Every isolated
body/j2 hit in the four sampled actual-area IK branches involves this leaf;
the untouched source pose also reports an intersection with L1's connection
plate, occurrence `1/2/2/3`. The two occurrences share their attachment origin within 9.5e-13 mm and
have collinear bearing axes, with the insert's local frame clocked 180° relative
to the plate. This was a rigid ownership error, rather than an inflated hull.
After user visual review, the exporter binds occurrence `1/3` to `engineer/j2`
so it moves with L1. The source geometry/material bytes, zero-pose placement,
1682 retained nodes and 163,701 Engineer triangles are unchanged. The manifest
records the ownership override and attachment evidence; the exporter rejects a
changed source attachment. No body/j2 collision exemption was added.

The corrected asset has no body/j2 collision in the zero-joint source pose.
Set `ASSEMBLY_COLLISION_REPLAY` to an earlier pinned audit JSON when running the
ignored audit to recheck exactly the earlier joint vectors. Replaying all four previously audited area-case joint vectors clears this pair
as well. Level 2 still completes. Fresh IK branch numbers can change because
the loader now chooses different clear initial grasps; comparing branch indices
alone is not a before/after geometry test. The corrected-area level-3 rerun
tested 3,057 setups and 7,443 root candidates before exhausting 8,000 guarded
queries. Its coarse prefix audit verified both Approach and Insert, but it still
found no complete path. Remaining body/j2 contacts involve other source parts;
the shoulder-disk fix does not waive them.

The renderer verifies every Engineer collision triangle against its paired
source GLB triangle within 1e-12 m before drawing. Orange marks the suspect
chassis-assigned leaf, cyan the j2/L1 link, grey the remaining robot. It exports
source and rejected poses from three viewpoints, with full context, isolated
parts and deliberately cropped joint close-ups. These are exact surface meshes;
the images do not display inflated convex broad-phase envelopes or infer contact
depths from surface intersections. `review.json` records the named source leaves.

### Remaining contact audit and camera preparation

The planner now records the first rejected **static milestone** for each named
pair in `rejected_milestones`, including the actual FK link frames and joints.
These are exact endpoint query states, not reconstructed impact points from a
sweep. The leaf audit can consume them using `ASSEMBLY_COLLISION_PLANNER` and
select a pair with `ASSEMBLY_COLLISION_PAIR='engineer/body,engineer/camera-lift'`.
The default remains body/j2. Convex or exact mesh source leaves can be inspected.

The post-disk-fix 1,200-query sample isolated:

- Body/L1: 15 exact mesh-leaf intersections, involving front armor occurrence
  `1/1/83/3` and bracket `1/1/83/2`, rather than the removed stationary-disk error.
- Body/camera lift: five intersections between armor `1/1/62/3` and camera
  hardware under `1/2/6/1`. At the same arm/chassis pose, the original source CAD
  camera posture clears all five. `ASSEMBLY_COLLISION_DEPLOYED=1` adds this
  controlled camera-only comparison; UWB and all other frames stay unchanged.
- Core/gripper: Core head shell 7 (`core/j6` collision leaf 13) intersects
  Engineer source solid 1668 (`engineer/j6` leaf 21, occurrence `1/2/8/1/4`).
  Both selected leaves are exact meshes. This sampled rejection therefore is
  not caused by a convex hull filling a Core aperture. Surface contact alone
  does not provide a penetration depth; the bracket/gripper location needs
  path and grasp review.

Area-mode fixture preparation now tries both camera postures: the existing
stowed transform and the source CAD reference posture. `camera_deployed` in
CAD reports and branch witnesses identifies the latter. This is a fixed
pre-attempt setup choice, not a simulated deployment trajectory or calibrated
servo endpoint. Current-pose planning preserves it, and the collision guard
still checks all camera/body/Core/terrain pairs. UWB retains its stowed state.
Bare-fixture preparation retains its previous camera choice. The two-posture
level-3 area rerun tested 3,049 setups and 7,453 roots before the 8,000-query
budget expired. It retained coarse Approach/Insert witnesses but still found
no complete path. Posture selection clears a specific obstruction, not the
remaining lift/flip feasibility problem.

The collision renderer accepts arbitrary Engineer pairs and selected Core
links. Exact mesh leaves are verified against source triangles; declared Core
convex leaves are rendered as their actual hulls and checked against source
vertices. `--compare-deployed` compares the sampled rejected camera pose with
its camera-only source-posture variant. Three views and context/isolated/joint
modes expose the remaining contacts without exempting them.

### Camera cable omission (2026-10-19)

User review identified the protrusions in the camera/armor contact render as
flexible cable harnesses. The exporter now omits the two complete camera cable
subassemblies `1/2/6/1/1/1/1/4` and `1/2/6/1/1/1/1/5`, including their connector
ends (source solids 1603–1610). They are absent from both the active visual scene
and generated collision asset. Camera housing, lift rails, board and mounting
structure remain. The manifest records the omission paths, source solids and
reason. Retained source buffers/materials remain unchanged.

This removes eight source mesh nodes and 802 triangles: Engineer active geometry
is now 1,674 nodes and 162,899 triangles. Replaying the original rejected stowed
camera pose gives zero camera/body leaf intersections, down from five, without
changing any chassis or arm joint. Thus deploying the camera is not necessary
to clear that sampled wire-induced obstruction. Retained camera hardware still
intersects the Unit and jaws in other sampled poses (four, one and two leaf
pairs respectively); those contacts remain enforced.

The renderer's `--first-part` option selects a reference leaf explicitly when
inspecting a historical pose which has become clear. For the body/camera replay,
`--first-part 689` highlights the unchanged armor housing. It records zero
remaining pair intersections rather than inventing a contact to obtain an image.
Level 2 and terrain-integrity checks continue to pass. Old audit/render artifacts
refer to their original source hashes; regenerate against the new package for
current collision results.

### Coordinated chassis routes

Prepared area-mode plans can move the chassis while the same Cartesian guide
continues to position the gripper. The gripper is not released or teleported to
connect segments. The bounded search includes these routes (world rig +X points
away from the Assembly Area front wall):

| Route | During lift | During P flip |
|---|---|---|
| 0 | Fixed | Fixed |
| 1 | +50 mm X | Another +100 mm X |
| 2 | +100 mm X | Hold |
| 3 | Hold | +150 mm X |
| 4 | Hold | +100 mm X and +15° chassis yaw |
| 5 | Hold | +100 mm X and −15° chassis yaw |
| 6 | −50 mm X | +150 mm X |
| 7 | −100 mm X | +150 mm X |
| 8 | −50 mm X, +50 mm Y | +150 mm X, −50 mm Y |
| 9 | −50 mm X, −50 mm Y | +150 mm X, +50 mm Y |

Difficulty two uses routes 0–2; difficulty three includes all ten. Approach and
insertion hold the initial chassis pose, while Q alignment holds the end of the
selected route. Offset/yaw progress is continuous across segment boundaries.
The search spreads its finite budget across setup/route combinations. Endpoint
IK and reverse segment continuation use the moving base pose, not the original
parking throughout. `route_trials`, `unsupported_route_samples` and the selected
`selected_route` distinguish the new search in reports.

Every planned frame carries its absolute chassis pose. Building and replaying a
complete plan limit chassis translation to 0.25 m/s and yaw to 0.35 rad/s at
128 Hz. These are workbench settings, not certified drivetrain limits. The
ordinary workbench collision guard interpolates chassis pose together with arm joints,
subdividing translation to at most 2 mm and rotation to at most 0.01 rad before
sweeping source geometry. Its level-footprint check runs at each intermediate
pose, rejecting unsupported patches and crossings to another floor height.
Ground queries reuse the server's immutable footprint BVH over the unchanged
terrain triangles. The support model remains the four-corner, 500 mm-square
inspection footprint; this is prescribed position motion without wheel forces,
rolling animation, suspension or hardware motor calibration.

Playback refuses nonfinite poses, invalid quaternions and sudden chassis jumps.
The initial chassis pose is retained separately so restarting a moved attempt
restores the original parking along with the initial arm joints. No partial
route is installed when planning fails. Tests cover a wall between clear
endpoints, a support gap between supported endpoints, rate/finite checks and
restart restoration.

The wire-free, real-area difficulty-two demonstration completed all 360 planned
128 Hz frames and reached `Confirmed`. The planner used 344 guarded checks,
selected a +30° initial chassis yaw, deployed camera posture and +135° grasp
roll, and chose route 0 (fixed chassis). This validates insertion and lift on
the source terrain; it does not demonstrate a successful moving-base assembly.
The recording is `local-assets/assembly-cad/mobile-area-d2.jsonl`; its matching
video is `mobile-area-d2.mp4` in the same scratch directory.

The difficulty-three search across all ten routes exhausted 8,000 guarded
checks after 3,077 setup/route combinations and 7,508 root candidates. It found
no complete swept route. There were 32 unsupported route samples and 187
`Flip IK` rejections; the other leading obstacles remain chassis/shoulder and
Core-head/gripper intersections. These are bounded-search results under the
workbench's assumed joint limits and route family, not proof that the physical
robot cannot assemble difficulty three. Moving-base playback has synthetic FK,
cadence, obstacle and terrain-support tests, but still needs a complete
source-CAD success case.

### Seated rotation around the assembly rod

The previous guide fixed the Unit's axial clocking throughout the attempt.
Changing `grasp_roll_rad` only changed how the gripper held the Unit before
starting; it did **not** search rotation of the Unit around the rod during
assembly. That imposed an additional orientation constraint on the arm.

The workbench now carries a separate `rod_spin_rad`. Its target is the source
head motion multiplied by the seated Unit centre and a rotation around the
Unit's local Z symmetry axis, followed by the unchanged grasp transform. The
Unit and closed gripper rotate together; this introduces neither an extra arm
joint nor slipping or regripping. The Unit centre, insertion depth and rod-axis
alignment stay fixed for a given P/Q/lift state. Off-axis tilt remains an error.
The current sensor/guide model treats axial clocking as unconstrained by the
socket. This is a simulator policy based on the cylindrical interface, not a
measured frictionless bearing: the manual describes the rotational symmetry
axis and extraction force, but does not specify axial rotational resistance
(2026 V2.1.0 §5.3.3, printed pp90–94; 2027 §5.4.3 assembly sequence).

Prepared difficulty-three area searches include eight additional linear twist
profiles: ±90° and ±180°, either during lift or **simultaneously with P flip**.
Eight more profiles vary twist across the flip, specifying spin at P=45° and
P=90°: `(0,+90)`, `(+90,+90)`, `(0,−90)`, `(−90,−90)`, `(+90,0)`, `(−90,0)`,
`(−90,+90)`, `(+90,−90)` in degrees. These test early/late twisting and twisting
then unwinding. Q alignment retains the selected final clocking. This is a
bounded profile search, not a complete continuous redundancy planner. The
existing ten chassis routes remain available separately; these added spin
profiles hold the chassis fixed.
Linear profiles retain their original deterministic setup order as piecewise
profiles are added. The search reserves 20% of its guarded query budget for the
latter, or 1,200 checks in smaller sweeps (capped at half the total budget),
checked between setup trials; a candidate already under full validation may
still use the remaining global budget. This preserves successful
linear-route searches while reserving fallback work for different twist timing.

Every planned frame and CAD report carries spin. Sampling accounts for the
maximum slope of each piecewise profile, including profiles with zero net spin.
Playback bounds spin to the workbench's 0.6 rad/s angular rate and validates the
full gripper target and ordinary swept collisions. Tests cover rod-centre/axis
invariance through P/Q motion, constant grasp transform, positive spin playback,
rate/finite rejection and restart.

`audit_rod_spin_reachability` is an ignored, source-CAD diagnostic. It takes
`ASSEMBLY_CAD_PACKAGE`, `ASSEMBLY_CAD_AREA`, `ASSEMBLY_SPIN_REFERENCE` (a completed
level-two JSON report), and `ASSEMBLY_SPIN_AUDIT` (output JSON). It compares
16 axial angles at five P/Q milestones, then samples nearby supported parking
poses. These are static IK/contact results, never executable connections.
For the completed level-two parking pose, P=60° is unreachable with spin locked
at zero, but spin from +22.5° through +112.5° in the sampled grid gives clear
solutions. P=90° remains unreachable at that parking pose for every sampled
spin. Of 21 supported nearby parking/yaw samples, four have reachable P=90°,
Q=30° endpoints only with nonzero spin; all tested endpoint solutions still
intersect the Core head with the left jaw, or the chassis with terrain. The
remaining issue therefore involves placement and clearance as well as axial
orientation freedom.

Axial twist is now a shared coordinate across **approach, insertion, lift, P
flip and Q alignment**, not a flip-only adjustment. One stage-progress function
supplies the target clocking, reverse continuation, coarse search, 128 Hz plan
building and sampling bounds. Signed Q progress works for either alignment
direction. Resumed planning anchors every candidate profile at the current
clocking, so enabling a new profile cannot jump the Unit's orientation.
Grasp closure and confirmation holds keep the preceding clocking.

The automatic family additionally tries ±90° approach/insertion/lift/alignment
increments where those stages exist, paired multi-stage increments, and +90°
flip twists combined with the existing chassis routes. These remain bounded
continuous profiles, not a complete search of all possible twist functions.
Approach twist turns the carried Unit before seating; insertion twist turns it
around the aligned rod without changing insertion depth relative to the guide.
The usual Unit/socket interface contact policy, jaw attachment gate, off-axis
error checks and swept scenery/self-collision checks are retained.

**A complete level-three case now succeeds.** The linear-spin search selected
an initial chassis translation `[1.27800005, −0.54791495, 0.38315052]` m in the
workbench frame and +195° yaw, with no chassis movement during assembly. It
performed approach, grasp, 100 mm insertion, 100 mm lift, simultaneous +90° P
flip / +90° axial twist, then +30° Q alignment and referee confirmation. Planning
used 6,279 guarded checks over 2,132 setup/route trials. All 808 playback ticks
passed; the 204 recorded samples contain no collision, at most 9.97 µm position
residual and 0.086 mrad orientation residual. This uses the existing six-axis
arm, existing joint guards and unmodified collision exclusions. The recording
and video are `local-assets/assembly-cad/rod-spin-d3.{jsonl,mp4}`. It demonstrates
one fixed, legal case; it is not a random-pose coverage claim or a calibrated
force/contact simulation.

`audit_twist_counterfactual` loads that completed JSONL through the same four
environment variables above, verifies its source manifest and terrain evidence,
and repeats IK continuation from exactly the same initial parking, grasp and
joint branch. Locking axial clocking fails to converge at P≈59.24°; adding the
simultaneous twist completes all IK segments. The locked comparison is an
IK-only branch experiment, while the successful recording separately validates
the complete swept path and referee result. The smaller 4,000-query exploratory
mixed-profile run exhausted its budget, reinforcing that a search failure alone
does not establish physical impossibility.

### Reusing verified search work

The per-query IK transposition table retains up to 512 successful target/branch
entries for an exactly matching serial-arm model. Queries try solutions from
the four nearest entries within 150 mm and 0.8 rad before ordinary branch seeds.
Those radii are search heuristics. Nearby poses are solved again, including
joint limits; failed IK is not memoized as proof of unreachability. Reports
count `ik_queries`, `nearby_ik_hints` and `exact_ik_hints`. The planner returns
the first complete valid path, without an optimality requirement.

A separate bounded table retains up to 2,048 collision-checked transitions.
Its hash only locates candidates: exact equality of all guard inputs and the
immutable collision-scene/terrain identities establishes reuse. Checks include
calibration, camera posture, joint coordinates (including winding), chassis,
tool/Unit poses, jaw and attachment state, grasp transform and P/Q/lift/twist.
A hash collision or nearby pose cannot confer clearance. Reuse is counted by
`collision_table_hits`; reused queries still consume the normal search budget.

Built plan frames retain these in-memory transition certificates. Replay still
checks finite values, joint bounds, motion cadence and Cartesian FK; it reuses
the expensive articulated sweep only if its exact before/after guard states
match. Changed state, calibration, geometry or terrain falls back to the ordinary
guard. Certificates are never serialized or trusted from a recording. One
validated stationary hold can certify repeated identical settle ticks.
`cached_guard_replays` counts successful replay reuse. Tests cover changed
joints/calibration/grasp, new blocking geometry, exact cache hits, and nearby
IK hints that must be resolved at their new target.

`scripts/assembly-pose-matrix.py` runs frozen source-CAD cases, including
explicit Core translation/orientation variations, nearby fixed chassis
placements, negative Q, two seeded poses, and prescribed twists in each moving
stage. Use `--case NAME` (repeatable) for a subset; `--jobs`, `--budget` and
`--timeout` bound the experiment. Each case retains stdout, stderr and the
sampled recording. The harness pins one executable inode for its entire sweep,
so rebuilding does not change queued cases. `report.json` pins its SHA-256,
the package manifest and terrain evidence,
and distinguishes confirmation, source/placement rejection, bounded planning
failure, playback failure and timeout. A confirmation additionally checks
residuals, gripped Unit position, finite source poses, fixed chassis and axial
spin cadence. No rejected target or failed search is resampled.

```sh
python3 scripts/assembly-pose-matrix.py \
  --package local-assets/assembly-cad/package \
  --area local-assets/assembly-cad/area
```

The 2026-10-19 cached verification sweep confirmed seven selected runs with a
fixed chassis during playback: the reference Core pose, +25 mm Core height,
a +25 mm lateral shift of both Core and initial chassis, +10° Core azimuth,
a prescribed reference twist, twist split across
lift/flip, and additional twist during Q alignment. The azimuth case selected
180° of axial twist during its 90° P flip; the others used 90° total before
alignment. Height and azimuth cases retain the reference absolute parking;
the lateral case follows the loader's Core-relative parking convention.
Lift/flip splitting used 22.5° / 67.5°; the alignment case added
22.5° while Q moved 30°. All 808 transitions per ordinary case, and all 1,143
for the azimuth case, reused exact in-memory sweep certificates during replay.
New target paths were still fully swept during planning. Results and source,
executable and hint checksums are in
`local-assets/assembly-cad/verified-matrix/report.json`.

The reference whole-run time decreased from about 432 s in the uncached
exploration to 225 s with replay reuse and IK hints. These were exploratory
runs with different concurrent workloads, not an isolated performance benchmark.
The nearby-parking retries (+50 mm rig X, +50 mm rig Y, −15° chassis yaw) still
found no complete route within 4,000 checks each. Earlier 2,500-check tests
also rejected the prescribed +22.5° approach/insertion twists, negative Q,
some Core orientation changes and one seeded target; another seeded target
was rejected by initial source placement. These bounded outcomes do not
establish physical impossibility. The initial eight-minute execution timeouts
for the successful cases were superseded by the completed cached runs.

Recordings include terrain evidence. `render-assembly-video.py --area
local-assets/assembly-cad/area ...` verifies and renders matching terrain and
still refuses incomplete recordings. Bare-fixture recordings remain valid
without `--area`.

```sh
python3 scripts/assembly-area-scenarios.py \
  --package local-assets/assembly-cad/package --area local-assets/assembly-cad/area
```

This checks elevations, frame evidence, source hashes, supported placement,
modified-file rejection and budget exhaustion without moving the live rig.

## Level 3 investigation (2026-10-19)

The earlier **flat** 600 mm comparison fixture has a **reach/clearance conflict under the
current binding and fixed chassis**, rather than a proven rule-level or physical
impossibility. The original search examined 3,710 candidates and made 3,954
charged collision checks. All 100 branches reaching the flip prefilter used the
+20 mm axial grasp; each failed IK during P travel. Centred grasps were rejected
earlier. No branch reached the Q stage in that search.

The +20 mm grasp puts the tool centre at approximately 956.985 mm at P=90°,
Q=0; the centred grasp needs 936.985 mm. This is a tight orientation-constrained
workspace boundary. A separate numerical height maximization, with free planar
parking/yaw, generous ±2π joint guards and no collisions, converged from 33
starts to approximately 956.866 mm for one sampled grasp clocking. Its 0.119 mm
shortfall is an indication of near-singular reach, **not a certified mechanical
limit**: source frames, numerical guards and mounting dimensions are not
calibrated to that precision. Different clockings require their own checks.

Representative rejected steps also expose guard effects: at P≈68.35° one branch
needs J3≈180.80°; at P≈63.80° another needs J4≈90.62°. Allowing those coordinates
locally recovers those individual IK steps. Complete planner experiments widening
J4 to ±π, other joints to ±2π, or both still found no complete path within their
8,000-check budgets. Temporary 18 mm and 19 mm grasp-offset experiments also
failed to complete. These probes retained the original source collision scene;
none changed runtime guards, meshes, tolerances or exemptions.

A reverse-continuation diagnostic starts at final alignment, unwinds Q/P and
then follows lift/insertion backwards to seed the initial arm. It found 33
continuous IK candidates in 841 sampled parking/grasp setups. The first 32 were
rechecked forwards through the ordinary swept collision guard; all were
rejected: 17 by chassis/J2, 7 by Core/chassis, and 8 by other Core/arm or
arm/gripper pairs. The first kinematic witness used a centred grasp, the chassis
300 mm closer and 225° grasp clocking. Thus IK-only level-3 trajectories do
exist, but those checked trajectories are obstructed. The 33rd candidate was
not collision checked because the diagnostic's 32-path budget had been reached.
This is not an exhaustive collision-free path search.

The PIE source report §3.1, printed p5, describes designing for levels 1–3 with
380 mm and 400 mm arm sections. Its final summary, printed p18, reports that
competition operation achieved level 2 at most, alongside electrical/control
and preparation problems. That history does not establish mechanical
inability to perform level 3.

Next investigate the chassis/J2 and Core/wrist moving-part ownership and actual
joint-zero/stop calibration. Then use backward task-wide branch selection,
finer grasp/parking choices and, if needed, collision-checked chassis motion.
The current planner freezes parking and grasp once the attempt begins and
selects only the first clear Core positioning branch. More RRT samples alone
cannot repair an unreachable constrained flip waypoint.

Two ignored, source-dependent diagnostics preserve the guard and reverse-search
experiments. Their passing test status means the experiment ran; inspect the
JSON `status` / `guarded_witness` to determine whether assembly was feasible.
The reverse diagnostic checks at most 32 complete paths and reports its first
IK-only witness separately. `ASSEMBLY_AUDIT_NEAR=1` tests closer parking offsets.

```sh
ASSEMBLY_CAD_PACKAGE="$PWD/local-assets/assembly-cad/package" \
  ASSEMBLY_AUDIT_LIMITS=original nix develop -c cargo test \
  -p rm-sim-app --example engineer_assembly audit_level_three_joint_guards \
  -- --ignored --nocapture
# Alternative experimental guards: j4, winding, all. These never change runtime.
ASSEMBLY_CAD_PACKAGE="$PWD/local-assets/assembly-cad/package" \
  nix develop -c cargo test -p rm-sim-app --example engineer_assembly \
  audit_level_three_reverse_ik -- --ignored --nocapture
```

## Planning performance and approximate collisions

Source collision geometry is partitioned into spatial chunks of at most 128
triangles without changing their coordinates or winding. Each chunk has a convex
superset for cheap distance bounds; the default mode falls back to exact source
triangles near these bounds. World AABBs and rigid motion bounds are computed
once per query. Endpoint contact results retain one exact pose/tolerance entry
per body pair, so stationary Core and terrain pairs can be reused without
assuming that nearby moving poses are clear. Allowances are checked before
cache lookup. Scene clones share this bounded cache; proxy scenes get their own.

`--fast-collisions` replaces the larger Engineer frames with their chunk convex
supersets. The wrist housing (`engineer/j6`), both jaws, complete Core, Unit and
terrain retain source collision geometry. Keeping the fitted mechanism exact
avoids proxy interference at its own joints. Visual models remain unchanged.
Proxy contacts are marked `conservative`; small gaps can become blocked, and
normal collision tolerances still apply. These are reproducible runtime proxies,
not a calibrated collision package or motor/contact simulation.

```sh
python3 scripts/assembly-pose-matrix.py \
  --package local-assets/assembly-cad/package \
  --area local-assets/assembly-cad/area \
  --case reference --case core-azimuth-10deg --case twist-reference \
  --hints local-assets/assembly-cad/rod-spin-d3.jsonl \
  --fast-collisions --jobs 1 \
  --out local-assets/assembly-cad/planner-perf/preferred-fast
```

Planner statistics now include `planning_elapsed_s`, covering search and complete
path validation, excluding loading and playback. Matrix reports retain that
value alongside total process time and collision mode. Timing is diagnostic
app-side metadata; physics and simulation clocks still use explicit ticks.

The 2026-10-20 experiment uses the same source package and reference IK hints as
the earlier verified matrix. The original forced-twist reference completed in
182.64 seconds total; chunk bounds plus exact stationary-pair caching reduced it
to 13.71 seconds planning / 14.53 seconds total in the exact mode.
Chunking alone was insufficient. See the retained local reports in
`local-assets/assembly-cad/planner-perf/` for the individual variants.

Completed recordings also supply a preferred five-stage twist schedule. The
planner tries matching existing profiles first; it never imports clearance or
adds unvalidated motion. Older recordings lacking approach/insertion/alignment
spin fields default those increments to zero. Malformed preference metadata is
ignored while joint hints still undergo calibration, finite-value, limit and FK
checks. Explicit spin inputs remain authoritative.

With the same reference recording, selective proxies and preferred-profile
ordering, four fixed scenarios completed in the final sequential run:

| Case | Planning and validation | Whole process |
|---|---:|---:|
| Reference | 7.24 s | 8.06 s |
| Core azimuth +10 degrees | 7.49 s | 8.31 s |
| Explicit +90-degree P-stage twist | 7.19 s | 8.01 s |
| Core height +25 mm | 15.32 s | 16.13 s |

These are warmed searches with imported IK/profile hints, not cold-start latency
or broad success-rate measurements. A separate reference run without imported
hints completed in 13.03 seconds planning / 14.43 seconds total. Each returned path completed ordinary
playback and reached referee confirmation; no collision waivers or cadence
changes were introduced. The higher case initially exhausted 2,500 checks in
both exact and proxy modes before preferred-profile ordering was added. Its old
recording also passes the new source contact audit. The final search found a
complete path within the same budget. A bounded failure remains distinct from
proof of infeasibility. Broader coverage and live Heavy integration remain
separate work.

## Remaining integration

### Comparison with HKUAstar planners (2026-10-19)

Read-only reviews pinned MoveIt-Engineer to
`55518d7115c89bd5adceb3e1129c39968c96cc49` and engineer-capability-map's
`feat/tech-core-feasibility` branch to
`aedd2bae8cf0231aba5c2b7d0c1b41c81b125171`. Both repositories use six
revolute arm joints; the gripper joint is fixed. These are different arm models
from the workbench's PIE donor. Six joints can satisfy a full Cartesian pose,
but ordinarily leave no continuous redundancy at that pose. A task that fixes
position and rod direction while allowing axial clocking has one fewer
orientation constraint. That freedom can help without a seventh motor; it does
not guarantee reachability or a collision-free connection.

The capability-map [trajectory generator](https://github.com/HKUAstar/engineer-capability-map/blob/aedd2bae8cf0231aba5c2b7d0c1b41c81b125171/src/capability_map/techcore/trajectory.py#L250-L330)
retains a fixed Unit orientation during approach/insertion/lift, then applies
rigid P and Q transforms. Its Q-spin is different from axial Unit clocking.
Its [dated report](https://github.com/HKUAstar/engineer-capability-map/blob/aedd2bae8cf0231aba5c2b7d0c1b41c81b125171/docs/reports/2026-09-16/technical.md#L109-L132)
already attributes the swing wall to rigid end-grasp orientation. Other
[model constraints](https://github.com/HKUAstar/engineer-capability-map/blob/aedd2bae8cf0231aba5c2b7d0c1b41c81b125171/docs/reports/2026-09-17/arm-comparison.md#L93-L118)
include shoulder 17–163°, elbow −73–60°, wrist pitch ±100° and a tool frame
45° from the final roll axis. Its latest placement-path experiment covers
[approach/insertion rather than the complete level-three sequence](https://github.com/HKUAstar/engineer-capability-map/blob/aedd2bae8cf0231aba5c2b7d0c1b41c81b125171/docs/reports/2026-09-20/path-search.md#L13-L19).

A local check using that branch's analytic IK sampled the original Engineer's
fully swung endpoint at Core `[−50,200,500]` mm / `[0,90,0]` degrees. Across
225 specified parking candidates and 73 axial clockings (−180° to +180° in
5° steps), neither fixed nor free clocking yielded an endpoint IK solution.
Clocking right-multiplied Unit-local +Y, its bore axis; it preserved the origin
and rod direction, independently of Q. This narrow 16,425-target experiment
does not test collisions, full paths, TongJi or all placements. It shows that
clocking alone does not repair that particular original-arm case. The ignored
`local-assets/external-planner-review/axial-endpoint-diagnostic.{py,json}` retain
the reproduction and complete grids.

MoveIt-Engineer likewise constructs a full frame for every
[P-flip waypoint](https://github.com/HKUAstar/MoveIt-Engineer/blob/55518d7115c89bd5adceb3e1129c39968c96cc49/src/plan/src/flow_with_pyroki.cpp#L638-L704),
without continuous axial-clock search. That may constrain otherwise valid
paths. Its [README](https://github.com/HKUAstar/MoveIt-Engineer/blob/55518d7115c89bd5adceb3e1129c39968c96cc49/README.md#L234-L238)
also documents a collision-exemption change that broke previously working
paths. Default MoveIt/Pyroki configurations load different arm descriptions
with different joint-two conventions; reconcile those models before assigning
failure to DOF. See the [MoveIt launch](https://github.com/HKUAstar/MoveIt-Engineer/blob/55518d7115c89bd5adceb3e1129c39968c96cc49/src/plan/launch/flow_with_pyroki.launch.py#L23-L24)
and [IK model default](https://github.com/HKUAstar/MoveIt-Engineer/blob/55518d7115c89bd5adceb3e1129c39968c96cc49/src/plan/scripts/pyroki_ik_node.py#L60-L65).

Its endpoint fallback is a different success criterion: it disables collision
avoidance and installs an IK endpoint, with final service tolerances as loose
as 300 mm / 90°. It also accepts 95% Cartesian-path completion. Those previews
do not establish a complete collision-free assembly. See
[fallback](https://github.com/HKUAstar/MoveIt-Engineer/blob/55518d7115c89bd5adceb3e1129c39968c96cc49/src/plan/src/flow_with_pyroki.cpp#L1789-L1852)
and [acceptance thresholds](https://github.com/HKUAstar/MoveIt-Engineer/blob/55518d7115c89bd5adceb3e1129c39968c96cc49/src/plan/scripts/pyroki_ik_node.py#L302-L315).

### Live integration still required

This is an isolated workbench, without match networking, Heavy deployment,
progression or rewards. It cannot certify a live match. The referee Start/Finish/Cancel path remains available; the in-game automatic
adapter described below separately connects accepted manipulation to progression.

The source rigs prescribe joint positions. They do not solve motor torque,
loads, donor wiring limits or contact forces. The source gripper
has a fitted contact-envelope stop and gated kinematic attachment. In standalone
offline workbench fixtures, a released Unit remains parked; the live adapter below
instead uses persistent dynamic bodies. Offline source motion follows the fixture's constrained mechanism
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

## In-game automatic assembly

The normal app now shares the Bevy-free source rig and planner with this
workbench. Launch a source PIE Heavy on the bound RMUC 2027 field:

```sh
just run --play --robot engineer --cad-assets field-2027 \
  --robot-assets local-assets/robot-cad-survey/bound-tunnel-heavy \
  --assembly-assets local-assets/assembly-cad/package \
  --assembly-area local-assets/assembly-cad/area \
  --assembly-hints local-assets/assembly-cad/rod-spin-d3.jsonl
```

Drive onto your Assembly Area, park facing the Core, and stop. Press **J**
(rebindable as Automatic assembly), then choose **L1**, **L2** or **L3**.
Escape closes the panel. Cancel or manual driving stops arm work; automatic Core
recovery continues independently once the Engineer has withdrawn. No preparation
command moves the chassis: support height, tilt, motion and reach are checked
against the current parking. An unsuccessful solve leaves it there.

The arm starts in a compact raised neutral posture and remains part of the robot
between attempts. **T** folds it for a tunnel or returns it toward neutral. Both
posture changes and assembly playback obey simulator joint/jaw speed limits of
1 rad/s (about 57°/s); these are defaults, not measured donor motor limits.
Before solving, `Positioning` moves the Core from its last actual pose (source
CAD zero joints on first use) to its selected target on explicit 128 Hz ticks.
Core rotary joints and P/Q reset use a 1 rad/s simulator limit; carriage reset
uses 0.1 m/s. The moving Core is shown immediately, with a separate progress
bar and flowing radial lights. Snapshot/restore retains the motion and elapsed
time. Arrival starts the native 45-second attempt and releases the waiting
robot-planning worker; no robot IK/collision search runs during positioning.
The Core's chosen positioning branch remains fixed through robot planning.
Core travel is prescribed joint interpolation; Core-arm collision avoidance
and overload handling remain unimplemented.

Solving holds the current joints, plans a collision-checked approach from them,
and retimes the trajectory to preserve every waypoint within those limits.
Cancellation holds the last joint pose. Successful confirmation starts
`BreakingAway`: the Unit becomes socket-attached, the jaws open while the wrist
holds still, and a swept-collision-checked unloaded path returns the arm to
neutral at the same joint/jaw speed limits. Near the flipped housing, the planner can open the pads partially,
withdraw, then finish opening in clear space. This suffix is planned before
playback; the Core and seated Unit stay fixed during its collision queries.
Confirmation settles the reward once, before return; cancellation during return
does not revoke it. The attempt deadline excludes this post-success movement.
`Recovering` then moves the Core above the source collection opening, tips its
whole wrist downward, drops the spent Unit under gravity and returns to its
manual initial pose. `Completed` means Core recovery/reset has finished. Checkpoints retain the entire
return suffix and cursor, so restoring midway resumes without a pose jump.
A blocked return refuses planning rather than executing an unchecked retreat.

The live version starts with a deployment-preloaded Unit at the actual gripper;
base storage pickup is not implemented. The grasp remains fixed through the loaded
approach and insertion. It uses the canonical frozen Core pose, with the L3 Q
target revealed only after the P flip. The same source-preserving arm, Core and
Unit meshes animate through insertion, lift, P flip and Q alignment.
It does not yet drive the live projectile collision envelopes of the six-axis
arm or replace the field's static Core collision mesh; collision-aware planning
uses its own verified source station terrain and selective convex arm envelopes.
Other robots are not obstacles in that planning scene. Live chassis/terrain
contact stays with Rapier, so tyre compression does not produce false rigid-CAD
collisions or cause the planner to adjust the parking height.

The host loads the verified rig once and solves on a bounded worker; no GPU or
simulation tick waits for IK. The solving bar reports collision-query budget
consumed, rather than estimating how close the search is to success. Actuation
advances one validated sample per 128 Hz tick; its bar reports samples applied.
Pausing holds actuation while the background solve can finish. Complete
trajectories, source calibration, evaluator state and cursor travel in checkpoints,
so prediction and restore use the same remaining motion. Motion samples use micrometre/microradian grids (0.1 micro-unit for quaternion
components), far below the collision tolerance. Those samples are column-delta
packed and compressed losslessly to fit the world-update budget. Persistent idle
arm state, posture targets and jaws also travel in checkpoints. Protocol 71
requires matching clients and hosts.

In Idle practice all three levels are available. During a running native match,
the existing difficulty unlocks, station ownership, 45-second deadline and rewards
still apply. Completion comes from the manipulation evaluator, not an unconditional
Finish button. The Heavy is an assembly-only physical prototype: its 2027 combat
HP, progression and weapons are still unsupported.

The console accepts `World` protocol commands `AutoAssembly { chassis,
difficulty }` and `CancelAutoAssembly { chassis }`. `state.arms` exposes actual/target joints and jaws. `state.assemblies` exposes
phase, checks, budget, frame cursor and failure reason for automation. Console
`spawn` remains an operator test fixture; the assembly feature never invokes it.

To record the actual game renderer and assembly panel, start the app with
`--console --start-paused --third-person --window-mode headless`, then run:

```sh
python3 scripts/render-assembly-game-video.py --level 3 \
  --out local-assets/assembly-cad/in-game/assembly-l3-in-game.mp4
```

The Heavy must already be parked; `--reference-parking` explicitly opts into
operator placement for a reproducible recording fixture. Solving is captured
on wall time. Actuation is captured by stepping the paused game clock at 128 Hz
and encoded at normal playback speed, so the HUD displays PAUSED during capture.
The script uses ordinary console commands and GPU screenshots, retains an audit
JSON, and removes intermediate PNGs after encoding. FFmpeg is required.

### Live Tech Core lighting

The articulated in-game Core uses the source CAD diffuser triangles for all
three indicator groups. Disconnected radial strips receive independent
materials; splitting their index lists preserves every source triangle, normal
and transform. Angular positions accumulate every imported shell ancestor
transform into the Core head frame before measuring angles. Only verified white diffuser faces and the two red side bars
are rebound; the housing and other painted materials retain their source colors.

Lighting follows accepted insertion/lift/P/Q milestones and confirmation,
including the team-colored target-angle strip and current-angle white strip.
The 1 Hz and 3 Hz signals use simulation time and freeze when paused. See
[rules-2027](rules-2027.md#tech-core-indicators) for the full table, simulator
presentation policies and current scope. No light changes the rules authority.

### Online L3 reveal and solver reuse

The live controller has two L3 motion segments. The first plans a carried approach,
insertion, lift and P flip using the actual initial Q=0. It neither
receives the eventual angle nor chooses its grasp/route using that angle.
Its trajectory stops at the full post-flip pose without a confirmation dwell.
Before that endpoint, `Run::twist_target_rad` is `None`; the manipulation
adapter suppresses alignment acceptance against the evaluator's zero placeholder.

At the endpoint the world samples the final Q angle, once, from the existing
Table 5-10 uniform-range sampler (seed 0, selection = job generation). This
reproducible distribution is a simulator policy. The observed angle becomes
public with `AwaitingTarget`, and the target LED appears. Only then does the
host enter `SolvingTwist` and pass it to a planner. The actual joints, jaws,
Unit attachment and Core are held while searching; the native attempt clock
continues on unpaused simulation time. The attached planner tries local
continuation on the current branch with rod pre-twist and simultaneous Q/rod
profiles, with a bounded sparse Q/clock graph fallback and dense validation of
any selected route. It cannot change grasp, parking or Core placement. A bounded failure
is reported, without resampling the angle or replaying an easier initial grasp.

The accepted prefix, reveal and remaining segment are restorable checkpoint
state. The final segment replaces the consumed trajectory, starts at the held
joints, respects the same joint/jaw rates and retains the evaluator's insertion,
lift and flip evidence. Confirmation follows only the actual Q alignment dwell.
The standalone workbench remains an offline, known-target diagnostic; it must
not be presented as an online controller without this reveal boundary.

Review of the sibling `arm-eval` project identified these useful distinctions:

- `docs/optimization.md` and `docs/articulated-campaign.md` reuse immutable
  placed geometry and Core/Unit contexts. Here the existing exact pair-contact
  cache is supplemented with two exact-pose AABB entries per body, shared only
  by clones of the same immutable collision scene. Changing a body pose misses
  the cache; changing collision geometry clears it. Allowed pairs, tolerances,
  source triangles, narrow phase and swept validation remain unchanged.
- Continuation from an attached pose is a smaller problem than global setup.
  The late-Q planner directly follows the existing branch and varies rod
  clocking. The initial planner still searches grasps and free-arm approaches.
- `arm_eval/core_analytic.py` is a geometric solver for the field Core's
  three-parallel-axis layout. It is not an Engineer solver. The ideal
  spherical-wrist Engineer solvers also cannot be applied unchanged to PIE's
  source axes: the last axis has a measured source tilt and offset. Any port
  needs structural admission checks and original source-FK validation.
- Exact intrinsic collision reuse and certified subpath reuse remain useful
  next steps. Near-pose IK hints may seed a solve, but do not certify clearance.

No sibling Python dependency or solver source was copied; the bounds cache and
attached continuation are original Rust implementations of these design ideas.

Online information limits can reduce success relative to an offline plan that
knows the final target before picking its grasp. In the source regression,
+43.7° and +4.6° revealed targets have connected continuations; −32.5° from a
different, already-fixed prefix remains unresolved after bounded search. This
is not a proof of physical impossibility. The diagnostic records such late
failures explicitly, keeps the revealed target/grasp fixed and checks that no
completion is awarded. Universal late-Q recovery is not implemented.

A single local M5 Max live L1 probe measured 16.798 s before versus 16.444 s
with exact bounds reuse and the common worker refactor. Both produced 1,202
frames, 1,950 reported guard queries, and identical serialized trajectory SHA-256
`01cb5092f37c90a59366d4bb305fcb8e6e1016c9f9e1fb814d7b85a73776f89f`.
This roughly 2% difference is a single sample, not evidence of a significant
whole-planner speedup; bounds construction is only part of the workload.
The staged +43.7° and +4.6° continuation probes took 3.709 s and 0.361 s
respectively, including source validation/retiming. Their different targets and
initial prefixes make them examples, not an average or percentile estimate.

### Persistent Energy Unit and physical pickup

The host preloads one source Unit at PIE Heavy deployment, with its own stable
identity and closed jaws. It is visible before opening J, selecting a difficulty,
Core positioning or worker planning. Starting another assembly requires the
existing Unit to be held; it never creates or replaces an object. Base storage
and fetching inventory from the supply station remain future work.

Released Units are dynamic Rapier bodies in the same scene as chassis,
projectiles and field collision. They inherit their last carried linear/angular
velocity, fall under gravity and can rest on a robot. Disconnecting a carrier
releases its cargo. Identity, shape, attachment, pose, velocities and grasp
transform survive full checkpoints and prediction restore. There is no flight
expiry or projectile allowance for an Energy Unit. The world admits at most 32
persistent Units, matching its checkpoint bound. When full, deployment preload
fails without adding a body or changing the gripper; reset the field to clear
existing Units. Carrier removal preserves released Units within this bound.

The J panel has **Grip / release Unit**, XYZ translation jogs (1 cm) and
roll/pitch/yaw jogs (5°), also available through `EnergyGrip` and `EnergyArmJog`
protocol commands. A pickup needs the Unit cylinder within 8 mm of the pad
section, bore-axis alignment within 5°, and speed at most 0.5 m/s. These are
prototype interaction allowances, not rulebook constants. Pickup preserves the
actual object pose and records its wrist-relative transform; it does not snap
or re-clock the Unit. Source collision sweeps admit jog targets before the normal
1 rad/s joint controller follows them. Manual jogs use the station/source robot
collision scene; other robots and scenery outside its terrain crop are not arm
planning obstacles. Confirmed insertions mark the same Unit spent and socket-coupled until automatic
Core discharge. UC R60 forbids reclaiming a successfully assembled Unit; grip
commands reject it even when it is still between the pads.

Held/socket-coupled Units are prescribed poses with collision response disabled;
the live six-axis arm still has no dynamic contact-force model. The insertion
planner retains its original source Unit mesh and validates the complete loaded
path. Free-body dynamics use source-leaf convex outer proxies (bores/concavities
may be filled), the nominal 0.4 kg mass from UC Figure 4-39, assumed friction 0.6
and restitution 0.05. Contact-only linear
and angular damping of 1.5/s approximate rolling resistance; free flight has none.
Micrometre canonicalization removes CAD seam noise; compact circumscribed hulls
are admitted only within a 1 mm displacement bound. The contact parameters and proxy policies are simulator assumptions, not donor
measurements.

Canonical proxy vertices travel as bounded delta-coded micrometre columns in a
separately compressed immutable geometry record. Loaded trajectories omit Unit
pose columns only when source FK plus the stored grasp reproduces every original
quantized pose exactly; otherwise they retain the ordinary explicit columns.
Decoding reconstructs the same complete world state without a geometry registry
or a prior checkpoint.

The deployment preload uses a 135° grasp clock and 20 mm axial pad offset.
After pickup, the actual full wrist-to-Unit transform replaces that preset and
stays fixed throughout planning. Offline fixture search may vary grasps, but a
live carried plan cannot. Retimed samples derive the Unit pose from joint FK and
that same rigid grasp. Only subsequent entrance/insertion travel advances the
assembly evaluator, followed by lift, flip and the delayed LED target.

Free approaches use bounded collision-checked shortcutting to reduce joint
travel and may use the configured 1 rad/s limit. Insertion and P/Q segments keep
their source-constrained paths. This is a heuristic improvement, not a global
optimality claim; late-Q target coverage remains bounded.

The source animation and LED markers sample the same accepted trajectory on the
presentation clock. Sampling never advances an evaluator, invents a Q target,
confirms a job or starts a worker. It stops at information/completion boundaries.
Table 5-13's team-colored flashing is retained; recovery flashing ends after automatic discharge/reset rather than continuing
forever after completion.

To exercise live grip, falling pickup without snapping, collision settling and
rejection of assembly with no held Unit on a fresh paused PIE session:

```sh
python3 scripts/energy-unit-game-console.py --port 7790 --screenshots
```

The test uses the deployed object and leaves it loose for manual pickup. Optional
`--recover` tries a bounded guarded approach and records reach/path failures;
it cannot retrieve an object that has rolled beyond the stationary arm's workspace.

The live console assembly harness runs one selected `--level 1|2|3` per fresh
Heavy deployment. A successfully assembled Unit is consumed and discharged into
the collection bay; another attempt requires an unused Unit from future Base
inventory, not a replacement spawn or reuse of the spent object.

The recovery fit, rulebook requirements and simulator-only route/timing policies
are documented in [the 2027 rules digest](rules-2027.md#successful-core-recovery-and-spent-units).
The discharge point is centred over the verified source bay, with a 250 ms
stationary hold before gravity release and a 500 ms clearance dwell before the
Core returns upright. Source positioning/recovery remain prescribed; no Core-arm
collision avoidance or overload response is added by this animation.

## Temporary Hero conversion for Heavy

In an RMUC 2027 running round, **J → Convert to Hero** replaces the parked
Engineer's chassis and visual model with the source folding Hero. The player
keeps its id, team and Heavy roster slot. Launcher permission follows UC §5.7.1:
L3 completion or `300 - M/50 - 2N/3` elapsed seconds, with historical maxima
of opponent-minus-own Base HP and control. Negative thresholds unlock immediately
(simulator policy); the exact fractional boundary is inclusive. No conversion
is allowed during an unfinished assembly/Core recovery, while moving, defeated
or offline. Conversion releases existing held cargo rather than deleting it.

This is an explicit gameplay approximation: conversion is available wherever
parked, without workbench installation or the confirmation shot. It installs a
600-HP Heavy (existing damage preserved), 300 heat limit, 100/s cooling, a
100-heat 42 mm launch, and the Hero's configured weapon speed/rate within host
caps. Initial ammunition remains zero. Buy 42 mm ammunition through the existing
controls: one shell for 10 gold at a service zone, or ten for 150 gold remotely
with six-second delivery; the team cap is 100. Direct hits use the native
200-HP damage table; **42 mm idle/death/link suppression remains deferred**.
Adaptive accuracy statistics continue to count only 17 mm launches/hits.

The swap is immediate, with collision geometry rebuilt and placement revision
incremented so prediction does not interpolate between shapes. Source arm meshes
are removed; the Unit remains a physical object. There is no reverse conversion.
Starting a new round retains the chosen Hero shape but clears installation,
ammunition and unlock history; use the same button after unlocking again.

### Live planner performance policy

The calibrated serial solver uses an analytical Jacobian and bounded exact IK
reuse. Collision queries use chunk BVHs, exact endpoint histories and early
first-contact rejection. Live collision geometry retains its existing policy:
Engineer j1–j5 use conservative chunk hulls; the remaining source geometry retains fine surfaces.

IK tracks at fine cadence; live collision/path keyframes up to 62.5 ms apart
use articulated subdivisions of 0.025 rad / 5 mm plus continuous rigid-frame CCD, then generate speed-limited
128 Hz commands. Grouped return-motion rejection retries the original rounded
playback samples. Offline workbench inspection retains 0.01 rad / 2 mm sweeps
and its original cadence. This is an approximate game collision policy.

The two existing parked source fixtures reach approximately 3.4–3.6 s for the
initial L3 solve, with successful revealed-Q/return jobs around 1.9–2.9 s on
the targeted cases.
L1/L2 jobs including return take approximately 3.1–3.8 s. The -32.47° Q case
remains bounded and unresolved, rather than a completed assembly.
Warm hints remain spaced by playback time. Neutral return uses 256 candidate
samples within its existing 2,500-check budget. These are fixture timings, not
coverage or worst-case guarantees. Manual parking, actual starting joints, the held Unit and post-P Q reveal remain mandatory.
See [serial arm search](serial-arm-search.md) for implementation and reproduction.
