<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# EGA suspension evidence and remaining parameters

The 2026-10-23 investigation reads the original EGA SolidWorks files, their saved
exchange data and the supplied STEP. It changes neither the source CAD nor game
physics. The current game still uses an effective ray-wheel spring/damper model
with assumed parameters; imported joint playback is a separate constraint test.
See [the import workflow](solidworks-importer.md#recover-candidate-constraints-and-play-mechanism-states).

## What the native files add

The wheel module is named `华南虎-二代舵轮`, containing `华南虎-舵下总装3.0`,
`悬挂固定板`, two `摇臂` occurrences and a `KJ85T33` damper. This identifies an
inherited module and useful hardware families, not a calibrated dynamics model.
EGA's supplied `说明文档.md` cites the
[2024 华南虎 Sentry report](https://bbs.robomaster.com/article/54116), but does not
specify the damper's stroke, spring rate, preload or damping curve. No verified
manufacturer datasheet for the exact `KJ85T33` identifier was found. In particular,
**do not interpret `T33` as 33 mm of travel without a drawing or datasheet**.

The native `COMPINSTANCETREE` marks the damper occurrence `swFlexible="YES"` in
its parent wheel module. Composing the subassembly's standalone saved transforms
is therefore not a complete in-context placement solution. This is a concrete
lead for the earlier damper-eye mismatch, rather than evidence that the whole
native model is bad or that suspension compression alone explains it.

A read-only scan of `Contents/3DExperienceExchange2` finds complete contiguous
MessagePack mate objects. In the damper it finds six distinct records:

- Lock between the lower-eye and adjacent cap/seat Parts (project IDs 16/17).
- Concentric and parallel relations between the main body and lower-eye Part
  (15/16), consistent with a candidate slider.
- Two coincident relations and a tangent relation involving coil Part 18.

The wheel scan finds 125 complete objects with 122 distinct mate IDs, including
concentric/plane references connecting the damper ends to the suspension plate,
wheel carrier and thrust bearings; rocker/bearing/shoulder-bolt connections; and
a parallel relation between the two rockers. The scan is **partial**: it is not
an UnQLite database reader, current-configuration validator or native face-reference
resolver. Repeated IDs and omitted/cross-page records cannot be silently treated
as a complete authoritative mate set. The primary binary MatesList also remains
uninterpreted. Candidate fields include suppression flags, but reading a flag does
not establish complete configuration authority.

The two distance candidates are 4 mm and 5 mm mounting offsets, not compression
limits. No stroke interval or physical spring/damper coefficient was recovered.
The `moSpringConstantUnits_c` token in configuration data describes units; it does
not provide a spring rate. Importing every mate as a rigid joint is unsafe: the
coil's end constraints may position/deform a CAD spring, while a rigid simulated
coil constrained at both ends can eliminate the intended compression freedom.

## Linkage geometry

The two rocker hole centers are 25 mm apart. Corresponding ends of the two
parallel rockers are 50 mm apart, and all four pivot lines align with the named
MF106 bearings to numerical precision in the saved geometry. Together with the
mate associations, this strongly supports a parallelogram four-bar candidate.
It does not yet establish a complete independent constraint rank or travel stops.

The next reconstruction should group hardware by rigid attachment, retain the
two rocker links and moving wheel carrier, and resolve the four bearing axes.
The main damper body connects to the wheel carrier, and the lower-eye/rod-side
Part connects to the suspension plate according to the candidate parent mates.
Resolve native entity references and compare fitted geometry before assigning
these connections as reviewed runtime joints.

An ideal parallelogram gives a useful geometric relation. If `B` is a fixed
rocker pivot, `A` its initial moving pivot, `S0` the damper eye attached to the
moving carrier and `F` its fixed eye, then
`S(q) = S0 + (R(axis,q) - I) * (A - B)` and `L(q) = |S(q) - F|`.
This converts verified damper limits into a linkage interval after checking
collisions. It supplies neither the limits nor a vehicle-up direction: the import
retains the native basis. The measured 87.55 mm eye spacing is one saved state.

The prepared assembly description currently requires a tree. A four-bar needs
an explicitly represented closure constraint in addition to its spanning tree;
the importer's present one-parent joint editor does not represent that closure.
Do not break the loop and call the resulting unconstrained tree equivalent.

## Mass, COM and inertia

Independent OpenCascade integration of the original STEP validates the local
volume, COM and normalized inertia of selected machined Parts against native
caches. Products of inertia require the tested sign conversion. This establishes
geometric consistency, not actual hardware mass or material authenticity.

| Part | Native material | Cached mass | Interpretation |
|---|---|---:|---|
| Wheel carrier (29) | 6061 alloy, implied 2700 kg/m³ | 52.870 g | Good conditional CAD mass candidate |
| Suspension plate (30) | 6061 alloy, implied 2700 kg/m³ | 77.946 g | Good conditional CAD mass candidate |
| Each rocker (31/32) | 6061 alloy, implied 2700 kg/m³ | 14.193 g | Good conditional CAD mass candidate |
| Wheel plate (44) | VMA, implied 2000 kg/m³ | 55.517 g | Geometric check passes; composite layup/density needs review |
| Damper Parts (15–17) | Unspecified, implied 1000 kg/m³ | 13.802 g combined | Placeholder-density result, unsuitable as verified hardware mass |
| Coil (18) | Unspecified | 0 g | Invalid mass basis; STEP has positive volume |

For carrier/plate/rocker/wheel plate, relative volume and mass-normalized tensor
errors against STEP are below 3e-11; COM differences are numerical. The three
non-coil damper Parts agree geometrically within 0.014% volume, 0.00023 mm COM
and 0.013% normalized tensor error. Their density is nevertheless unverified.
The original damper assembly records 44 g while the mirrored assembly records
13.802 g. These may reflect overrides/staleness; the scan does not establish the
cause. Neither value should automatically replace the Part masses or be counted
alongside them. Coil STEP volume is approximately 1487 mm³ despite a zero cache.

In the 538-occurrence prepared subset, 328 occurrences have unspecified material
and another 21 lack positive cached mass/volume. This is a partial import, so its
mass sum is not the robot's real mass. We can replace box estimates selectively
for reviewed homogeneous machined Parts now; we cannot certify a whole-robot
inertia by summing every cache. Keep density/material/override approvals separate
from geometric COM/tensor checks.

## How to obtain the missing physics

Recover geometry and mate identities first. For the remaining physical inputs:

1. Obtain the exact damper drawing or measure minimum/maximum eye spacing with
   its mechanical stops. Distinguish stroke, installed length and remaining
   bump/droop travel. Check rocker/body interference across the interval.
2. Measure force versus compression, including installed preload. A coil geometry
   estimate `k ≈ G*d^4/(8*D^3*n_active)` needs verified material shear modulus,
   wire diameter, mean coil diameter and active turns. The display coil alone does
   not establish these, its free length or whether it matches the real spring.
3. Measure force versus velocity for compression and rebound, or fit a loaded
   decay response after accounting for linkage geometry and friction. Static mates
   contain no measured damping law. Use a constant `c` only as a labelled local
   approximation when the data supports it.
4. Convert damper forces through the linkage Jacobian. Damper stroke is not wheel
   travel; effective wheel stiffness/damping vary with posture and motion ratio.

Evidence, source hashes, candidate records and reproducible read-only probes are
in ignored `local-assets/suspension-investigation-20261023/`: `probe_exchange.py`,
`measure.py`, `step_mass.py`, `geometry-mass-report.json`, `step-mass-report.json`
and `linkage-kinematics.json`. These are investigative artifacts, not runtime
inputs or a completed native mate importer.

## Provisional isolated physical fixture

`crates/rm-sim-physics/examples/wheel_suspension.rs` now installs six bodies and
seven joints into the ordinary `PhysicalRuntime`: a fixed bench frame, two
rockers, a moving carrier/test load, a damper barrel and a rod-side body. Four
hinges close the parallelogram; two more hinge the damper ends; a prismatic joint
provides compression, a spring/damper force law and end stops. It installs joints
directly, including the closure edges, without changing the tree-only assembly
schema. No pose is prescribed after setup. Gravity is active, internal collisions
are disabled, and this is a loaded bench rather than a tyre/terrain test.

The native linkage plane is rotated into bench X/Z. Choosing bench gravity −Z is
an explicit test orientation, not verification of the robot's installed-up frame.
The 5 kg moving load is a representative lumped load, not recovered wheel mass.
It approximates neither a complete quarter-car nor a freely moving whole chassis.

### Chosen values and why

| Parameter | Provisional value | Basis |
|---|---:|---|
| Damper assembly mass | 44 g | Original assembly declaration; more plausible than the mirrored 13.8 g placeholder-density sum, still unverified |
| Barrel / rod-side split | 30 / 14 g | Fixture mass partition, not independent component measurements |
| Unloaded/extension-stop eye length | 96 mm | Chosen to give sensible installed preload, not decoded from the model name |
| Usable damper stroke | 30 mm | Explicit approximate test interval; compression stop at 66 mm |
| Spring stiffness | 3000 N/m (3 N/mm) | Rounded geometric order-of-magnitude estimate, not a measured force curve |
| Symmetric viscous damping | 50 N s/m | Moderately underdamped starting point for this load and linkage |
| Moving test load | 5 kg | Representative load case, independently configurable |

As a separate plausibility check, applying assumed aluminium density to the main
barrel/cap STEP volumes and assumed steel density to the lower-eye and coil gives
about 53 g. This is not a verified bill of materials, but supports a mass on the
order of tens of grams; it does not establish that 44 g is exact. Oil, seals,
material variants and native mass overrides remain unknown.

The native coil display has roughly 2 mm wire diameter and 20.6 mm mean diameter;
a helical phase fit gives approximately 6.67 mm pitch across a 45 mm axial envelope.
Assuming spring steel with `G ≈ 79 GPa` and six active turns gives roughly
3000 N/m using the elementary close-coiled-spring formula. The material, active
turns, real free length, end conditions and correspondence to the installed spring
are unknown; this only justifies an initial scale. No steel material is silently
assigned to the zero-mass native coil cache.

At the chosen bench posture, the damper-to-carrier vertical motion ratio is
approximately 1.95. The 8.45 mm initial compression therefore gives about 25 N
of damper preload and about 49 N of generalized vertical support, comparable to
the chosen 5 kg load. A local critical damper coefficient is roughly 125 N s/m,
so 50 N s/m starts near damping ratio 0.4. These quantities vary with posture;
they are not transferable to the game wheel spring without the linkage Jacobian.
There is no separate hidden preload force: the chosen free length supplies it.

The example uses the native 14.193 g rocker masses, checked COMs and principal
inertias, with correctly oriented body frames. Other diagnostic bodies use
simple explicit uniform-box inertias. The
44 g choice does not validate the original assembly tensor. It does not override
the existing importer mass approvals or a whole-robot mass.

### Run and inspect

```sh
# An explicit output path; creates an eight-second 1000 Hz trace.
mbx run -p rm-sim-physics --example wheel_suspension --locked -- \
  /path/to/suspension.csv

# Same setup with zero or stronger damping for comparison.
mbx run -p rm-sim-physics --example wheel_suspension --locked -- \
  /path/to/undamped.csv 0
mbx run -p rm-sim-physics --example wheel_suspension --locked -- \
  /path/to/stronger-damping.csv 150
```

The trace records actual solved body frames/endpoints, slider position/speed, carrier
height/tilt and closure errors. At 3 seconds a −20 N held vertical load is added
for 0.25 seconds, then released. In the initial four-iteration run it returned within 0.1 mm
of the late settled length about 0.76 seconds after release. At 6–8 seconds its
slider-speed RMS is below 0.001 mm/s, versus about 80 mm/s with zero damping.
The zero-damping run still includes the solver's numerical effects; it is not an
ideal conservative analytic reference. At 150 N s/m the same fixture returns
within 0.1 mm in about 0.22 seconds, illustrating sensitivity rather than proving
that coefficient matches the hardware.

In that initial four-iteration pulse, maximum hinge-anchor separation was approximately
0.65 mm and maximum carrier rotation is 1.12 degrees. These are finite solver
closure errors, not measured structural flex. The coarse inferred inertias and
large load/rocker mass ratio make the loop demanding. Do not infer submillimetre
physical accuracy or certify interference clearances from this fixture. Solver
accuracy, real load distribution, end-stop contact and tyre/ground behavior need
separate validation before integrating this plant into a playable chassis.

Seven public-runtime regressions cover loaded settling/rebound at 128/1000 Hz,
damping sensitivity, both axial stops under deliberate test force, exact recipe
reset/stale tokens, invalid choices and a deliberately removed loop edge that
produces increased closure error. These are behavior tests, not native mate or
hardware validation. The existing game physics is unchanged.

## Solver convergence and source CAD binding

The follow-up keeps the 44 g approved mass assumption and physical spring/damper
choices, but raises this standalone fixture's default numerical budget to 32
iterations. The ordinary game and generic `PhysicalRuntime::new()` remain at the
existing backend default. The explicit standalone constructor is bounded to
1–64 iterations; reset reconstructs with the same choice.

| Tick rate | Iterations | Peak hinge gap, mm | Peak carrier tilt, degrees |
|---:|---:|---:|---:|
| 1000 Hz | 4 | 0.653 | 1.117 |
| 1000 Hz | 8 | 0.181 | 0.313 |
| 1000 Hz | 16 | 0.048 | 0.083 |
| 1000 Hz | 32 | 0.012 | 0.021 |
| 128 Hz | 4 | 0.626 | 1.066 |
| 128 Hz | 32 | 0.008 | 0.015 |

These are maximum residuals across the eight-second ordinary load-pulse test,
not clearance tolerances or measured flex. Both rates now have a regression
requiring less than 0.02 mm peak hinge error in that fixture. Deliberate 100 N
axial stop tests produce larger transient errors and are not covered by the
ordinary-load accuracy number. With 32 iterations, the ordinary pulse returns
within 0.1 mm about 0.66 seconds after release.

Five alternating warmed local trials measured median step-plus-truth costs of
8.42 µs/tick at four iterations and 54.81 µs/tick at 32, excluding CSV output and
setup. This Apple M5 Max development-profile fixture cost is not a full-game
benchmark; the separate STEP audit was also running. Use additional solver work
selectively instead of silently increasing the whole simulator's budget.

The CLI's optional arguments after the CSV path are damping, solver iterations,
tick frequency, then an optional diagnostic axial force. For example:

```sh
mbx run -p rm-sim-physics --example wheel_suspension --locked -- \
  /path/to/accurate.csv 50 32 1000
# Diagnostic compression sweep: replaces the passive spring by held −100 N.
mbx run -p rm-sim-physics --example wheel_suspension --locked -- \
  /path/to/compression.csv 50 32 1000 -100
```

CSV starts with the installed time-zero state and includes six actual body poses.
The passive binding probe preserves source mesh vertices and body-local offsets,
then applies recorded body poses without a second simulation. Native Parts and
bearing context were rendered in the RXUI/Bevy workbench at settled and both
software-stop states. The coil is omitted from rigid playback rather than locked
or silently stretched.

### Fused shaft discovered by interference testing

The first grouping treated original Part 15 as one rigid barrel and only moved
the lower-eye/seat Parts. Original-STEP solid intersections then showed about
476 mm³ overlap between Part 15 and the lower eye at the assumed 66 mm compression
stop; coarse samples below roughly 86 mm also intersected. This result was not
explained away as a proxy artifact.

The native Part is indeed one fused solid, including an exposed 6 mm diameter
shaft from local z = −48.65 to −70.65 mm. A separate review-only surface partition
assigns that 22 mm drawn shaft to the rod-side body, while retaining the rest as
the barrel. It preserves the original triangle membership/vertices and records
its provenance; it is not a decoded native body boundary or an approved full
import. The source CAD is unchanged. Its cut surface is an open visualization
boundary; closed cut solids are used separately for boolean analysis.

The original fused solid does not describe a verified internal bore/piston.
Expected shaft/barrel telescoping overlap must therefore be distinguished from
external interference. The review partition does **not** certify piston engagement,
real extension travel, seal contact, coil bind or mechanical stops. The 30 mm
software stroke remains provisional even if the selected external solids clear.

Follow-up evidence is in ignored `local-assets/suspension-followup-20261023/`:
solver sweep/cost logs, recorded body frames, `bind_cad.py`, GPU screenshots,
`cad-binding.json`, and the original-STEP boolean audit. These are intentionally
small investigative subsets, not a collision-free whole-robot claim.

After the shaft assignment correction, the external original-STEP audit found no
common solid volume above 0.1 mm³ in the selected different-body pairs at the
initial/settled/two actual stop states and 13 ideal geometric states spaced
2.5 mm across 66–96 mm eye length. No boolean queries failed. This compares seven
original Part definitions, with one split into two test pieces; it excludes the
coil, bearings, the rest of the wheel/robot and the explicitly unverified internal
shaft/barrel interface. It is finite pose sampling, not continuous swept-volume
clearance certification. The initial unsplit intersection report is retained
alongside the corrected report instead of hiding the grouping error.

The next [guided wheel-contact fixture](wheel-contact.md) adds a dynamic sprung
frame, a driven rolling envelope and generated terrain in the same runtime,
without changing playable robot physics.
