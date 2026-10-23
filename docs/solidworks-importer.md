<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Reviewed SolidWorks import workbench

The independent [sw-rs import workflow](sw-rs-import.md) supplies both metadata
and cached Part meshes. The older openswx/sldkit commands below remain available
for compatibility and comparison.

The optional RXUI viewer can review native Part occurrences, assign them to rigid
bodies, fit collision proxies and prepare a physical assembly with explicit mass,
COM and inertia. This is an offline import workbench, not a native SolidWorks
editor. Original CAD files are read only; project and recipe outputs are JSON
sidecars. It does not modify a multiplayer robot or install a new game profile.

## Prepare native geometry

First build an openswx catalogue with the existing
[native extraction workflow](hardware-models.md#offline-native-metadata-extractor).
The separate `sldkit-rs geometry` CLI supplies cached native display tessellation;
neither parser enters the simulator dependency graph. The geometry CLI is from
[sldkit](https://github.com/monozukuri-ai/sldkit); its binary hash and the catalogue
hash are recorded in the project, along with per-Part native SHA-256 identities.
These are provenance records, not a certificate that edited project data still
matches CAD.

```sh
python3 scripts/prepare-solidworks-project.py /path/to/catalogue.json \
  --source-root /path/to/native-files \
  --document robot.SLDASM --configuration '默认' \
  --sldkit /path/to/sldkit-rs \
  --omit-missing-references \
  --output /path/to/robot.project.json
```

`--document` is the exact relative document path recorded in the catalogue.
Referenced configurations are selected by name; assembly masses are never added
alongside their descendant Part masses. Repeated Parts retain separate occurrence
identities and full parent-relative transform composition. Suppressed occurrences
are skipped; hidden Parts remain physical. Display vertices are converted from
declared millimetres to metres; native occurrence translations retain the existing
catalogue's metre interpretation, pending explicit source-frame review.

Unresolved active references normally produce diagnostics and block physical
export. The explicit `--omit-missing-references` policy removes these occurrences
from the imported model and records every omission. Their dependent source
relations cannot be imported because the bodies are absent. The GUI offers the
same operation. Other problems—changed source hashes, reflected transforms,
missing geometry or ambiguous configuration mapping—remain diagnostics. Omissions
are included in saved reviews and exported recipes; they are not reported as a
complete original assembly. No root-assembly cached mass is reused for an omitted
component.

Surfaces use diagnostic group colors; native materials are not imported in this
pass. The native assembly basis is retained for review; callers can supply an
overall placement rotation when installing the cooked assembly.

The geometry adapter accepts a single matching geometry configuration. A
synthetic `Config-N` name may also bind by source index when the native document
has exactly one configuration; the evidence is recorded. It refuses to combine
display tessellations across multiple ambiguous configurations, even if one is
marked active.
Limits include 64 assembly levels, 1024 prepared Part occurrences, 250,000
vertices per definition, two million unique vertices, eight million placed
vertices, 256 MiB project input and
bounded parser output. Repeated geometry successes and failures are cached during
preparation so repeated occurrences do not repeatedly invoke the native decoder.
Native display meshes may be partial or stale; review them against a known export.
The bounds do not come from the parser's combined edge/mesh envelope.

### Shared geometry and incremental reimports

New projects use `rm-solidworks-project-v2`: each source/configuration mesh is
stored once, with separate occurrence poses, grouping and mass data. Exact
duplicate vertices are reindexed by their float bits, including signed zero;
triangle order, winding and coordinates are retained without welding tolerances
or decimation. Cloned Rust projects share geometry arrays, and repeated visual
occurrences share GPU mesh handles while keeping independent poses and colors.
Version 1 inline projects and recipes remain readable and editable.

Add `--merge-review /path/to/robot.reviewed.json` when preparing another import
of the same root document. Matching occurrence paths must retain their source
hash, configuration (when recorded), pose, cache and exact geometry. Their group
assignments and explicit mass overrides survive; group frames, collision choices
and joint annotations are retained. Changed/absent occurrences require explicit
rebinding rather than receiving stale review data. Source changes invalidate
joint review flags when available provenance no longer matches.

New occurrences enter a separate **New occurrences (review)** group with an
unreviewed placeholder joint and unspecified mass. Source-frame review is reset.
The placeholder is not an inferred native constraint. If no unique root remains,
it is left unattached until **Parent** explicitly supplies one.

Use **Assign part** to distribute new Parts, or select the staging group and
press **Merge** beside an existing group. Bulk merging preserves source poses,
removes the merged group's own joint and resets the target mass basis. Groups
with children require explicit reparenting first. **Remove empty groups** removes
empty leaves and their joints without orphaning populated children. Explicitly
omitted references remove their previous occurrences. A group made entirely of
omitted Parts is removed with its attached relations during review merging;
surviving child groups become unbound roots,
with no automatic replacement parent. All omissions remain logged.

## Review groups and joints

```sh
mbx run --manifest-path integrations/scenario-viewer/Cargo.toml --locked \
  --features rxui-viewer --bin rm-rxui-viewer -- \
  --solidworks-project /path/to/robot.project.json \
  --save-project /path/to/robot.reviewed.json
```

1. Select a source occurrence. Create a moving group from it, or use **Assign
   part** on an existing group. Every Part belongs to exactly one group. Moving
   Parts between groups preserves their original assembly poses.
2. Choose **Part boxes**, **Box**, **Sphere** or **Hull**. Toggle collision-proxy
   rendering to inspect the approximation against the source surfaces. Per-Part
   boxes preserve gaps between Parts, but may fill holes inside each Part. A
   whole-group hull/box fills concavities; these fits are not an error-certified
   decomposition. Box half extents are padded to at least 10 µm.
3. Choose fixed/dynamic body mode. For a non-root group, **Parent** selects its
   parent group; enter the assembly-frame pivot in metres, axis direction and
   limits, then apply **Fixed**, **Hinge** or **Slider**. Hinge limits are radians,
   slider limits metres. The engine uses joint-local +X; the importer rotates the
   entered axis into both body-local joint frames. At zero coordinate both frames
   coincide at the entered world pivot.
4. The joint slider previews one selected subtree kinematically, with other
   coordinates at zero. It changes neither the stored source poses nor the
   exported model. It is not a dynamic simulation or evidence of collision-free
   motion. A fixed joint cannot preview nonzero displacement.

openswx's current catalogue does **not expose usable mates**. Assembly hierarchy,
nearby surfaces and names are not treated as proof that parts move together.
Grouping and joints therefore require manual review; no general native mate/DOF
solver or closed-loop reduction is provided. The occurrence/group lists scroll independently; detailed controls remain in the
left review panel. Initial loading, fitting and mesh updates run synchronously;
large assemblies can pause the UI. This is not yet a progressive/background CAD
import pipeline. The physical assembly must be one
connected tree. Empty groups, cycles, unreviewed joints and unsupported fixed-to-
fixed backend joints fail export instead of being silently redirected or dropped.

## Mass properties independent of collision

Each Part can use explicit kg/metre/kg·m² overrides, a reviewed native cache or an
explicit solid-box density estimate. The GUI accepts selected-Part mass, COM and
the six central tensor entries `xx,yy,zz,xy,xz,yz`. The off-diagonals here are
**tensor entries**, not unsigned products. Overrides take precedence over a
group's mass mode. A native-cache choice explicitly selects SI interpretation
and positive or negative product-to-off-diagonal convention; it remains
user-reviewed cached data, not independently verified physical measurements.

The box-density option estimates each Part as a solid box at the chosen density;
it does not measure actual volume, voids, hardware density or composite materials.
This estimate is labelled in the report. No missing mass silently becomes zero.
Even fixed groups require a mass basis so their physical report stays complete.

For each group the importer transforms Part COMs into the group frame, rotates
each central tensor and adds parallel-axis terms about the mass-weighted group
COM. It then diagonalizes the symmetric tensor and exports principal moments
and their orientation. Nonfinite data, nonpositive mass/moments, nonsymmetric or
physically unrealizable tensors fail validation. Native root inertias are not
promoted merely because they were parsed successfully.

Changing a collision proxy does not change these inertial properties. Dynamic
engine bodies use explicit inertia and zero-density colliders, avoiding double
counting. Original cache fields and reviewed/estimated settings remain in the
project rather than being overwritten by fitted collision geometry.

## Physical export and reconstruction

**Save review** saves editable project data. **Validate + export recipe** builds
source bodies/joints, cooks them, and transactionally installs the complete
assembly in `PhysicalRuntime` before saving a recipe. It does not establish
long-horizon solver stability, exact CAD mate behavior or game admission.

```sh
# No window/GPU initialization: validate, cook, install and export.
mbx run --manifest-path integrations/scenario-viewer/Cargo.toml --locked \
  --features rxui-viewer --bin rm-rxui-viewer -- \
  --solidworks-project /path/to/robot.reviewed.json \
  --export-solidworks /path/to/robot.recipe.json

# Actual source or proxy GPU capture, with the review panels.
mbx run --manifest-path integrations/scenario-viewer/Cargo.toml --locked \
  --features rxui-viewer --bin rm-rxui-viewer -- \
  --solidworks-project /path/to/robot.reviewed.json --import-proxies \
  --frames 2 --screenshot /tmp/robot-proxies.png
```

The recipe contains the reconstruction project and computed mass reports (recipe
v2 for a shared-mesh project; legacy projects retain v1). It can
be reopened through `--solidworks-project`; computed reports are regenerated from
the project instead of trusted as serialized native state. This is recipe
reconstruction, not continuation/checkpoint support. The initial importer code
lives within the optional viewer workspace; it is not yet a stable public robot
package API or automatic URDF exporter.

## Initial validation

The EGA three-occurrence small armor module was prepared from native files with
55,256 display vertices and 38,420 triangles. It renders in the shared-device
GPU viewer and cooks/installs with three box colliders. Using explicitly reviewed
SI/negative-product assumptions gives a cached total of 0.5038 kg; an independent
NumPy aggregation agrees on mass, COM and tensor within 1e-12. This verifies the
aggregation math using the same cache, not the real module's weight or freshness.

Tests cover transformed inertia, parallel-axis aggregation, proxy-independent
mass, explicit hinge motor motion, source-preserving grouping/preview, malformed
data, missing-reference omissions, graph rejection and recipe reconstruction.
Missing parts in the full EGA source remain subject to the explicit omission
policy. This small-module check does not certify the whole robot or automatic
native articulation recovery.

A first bounded EGA pass prepared a **partial** 349-occurrence scene and recorded
122 omitted missing references. Shared definitions and exact reindexing increased
this to **538 occurrences / 131 mesh definitions**, with 594,328 unique vertices.
The uncompressed project shrank from 154.7 MiB to 62.1 MiB while including 189 more
occurrences. These are storage/coverage measurements, not simulation speedups.
Every previous occurrence retained bit-identical vertex/triangle coordinates,
pose and cache. Review merging retained all 349 and staged the 189 additions.

No vertex-budget exclusions remain in that pass. Outstanding diagnostics are
112 occurrences rejected by the native decoder's byte ledger, 41 ambiguous
configuration mappings and four without supported display tessellation. These
are not missing references and remain blocking. The scene is useful for
inspection, not yet a complete physical robot export. Native cache accuracy and
full solver-trajectory parity are not established by geometry equivalence.

### Comparison with the installed STEP-derived Sentry (2026-10-23)

The partial native scene was compared with the actual
`bound-tunnel-sentry/neutral.glb` (222,900 triangles), not a separately fitted
replacement. The runtime manifest's neutral/hierarchy hashes and original STEP
hash were verified. A single source-to-runtime rigid frame was recovered using
its declared body rotation and source envelopes; no individual Part was aligned
or repaired to improve the result.

Of 538 native occurrences, 500 have name/configuration matches in the STEP
hierarchy: 490 match placement and ten differ. Another 37 have unique exact-pose
candidates under changed connector names; these remain a separately identified
correspondence class. One light-bar connector has no verified counterpart.
Across the 527 same-pose named/candidate matches, native display-mesh envelopes
agree with the original STEP bounds to a median 0.0044 mm and maximum 0.129 mm.
Envelope agreement does not certify every internal surface or feature.

The saved-pose discrepancies affect eight components in two KJ85T33
subassemblies (about 20.6–23.5 mm and 22.0–41.8 degrees), the MHF9025 rotor
(135.06 degrees) and a brake clamp (22.84 degrees). Their cause remains
unestablished; no source transforms were changed. A relative-motion follow-up
found that the first three Parts in each KJ85T33 unit share the same rigid pose
change to within 1.8e-15 in matrix entries. The fourth changes by 20.017 degrees
about the first Part's longitudinal axis, with axial-origin separation unchanged
to about 1.2e-8 mm. These are coherent subassembly-state differences, not evidence
of eight independent bad placements. The fourth Part was subsequently identified
as the coil spring; its phase change is not an independent rotary-joint claim. The geometry suggests damper-like wheel
units, but the two saved states do not establish compression travel, mounting
constraints or spring/damping parameters.

The rotor and brake pose deltas are pure rotations about the same source-frame
axis through approximately `[0, 0.1322, 0]` metres. The author's supplied
`说明文档.md`, under `关于模型的使用`, explicitly describes special-angle mates
in assembly favorites and switching their suppression state to inspect different
poses. This supports preserving native mechanism states while reviewing joints;
it does not establish which state is authoritative. That document also identifies
the hidden `灯条连接件-0517` as an Infantry-only variant, explaining why the native
inventory can contain it without a verified counterpart in the Sentry STEP.
Detailed relative transforms are recorded in
`local-assets/native-runtime-comparison-20261023/mechanisms/motion-report.json`.

Four imported wheel-clearance
volumes are deliberately absent from the runtime and must not become physical
collision bodies. Runtime AM02 surface replacements and the removed firing-axis
reference tube account for another five deliberately modified occurrences.

For the remaining 518 same-pose retained occurrences, each direction used 96
seeded area-weighted surface samples plus six vertex extrema, with exhaustive
point-to-triangle distances. The median of per-occurrence sampled p95 distances
is 0.778 mm; 495 of 518 have p95 at or below 2 mm. This is a sampled comparison,
not a certified maximum deviation. The largest unmodified outliers are tiny
washers reduced to two triangles in the old runtime export: examples have
4–6 mm errors against runtime geometry while the native envelopes agree with
STEP within 0.007 mm. The runtime low-detail mesh is therefore unsuitable as
submillimetre ground truth for those Parts.

Read-only probes, input hashes, per-occurrence measurements and an orthographic
visibility overlay are retained in ignored
`local-assets/native-runtime-comparison-20261023/`. The overlay compares visible
sampled surfaces, not a distance field. It includes a full-model view to expose
remaining coverage gaps and a shared-subset view to compare placement. No CAD,
runtime package or review project was modified. This neutral-geometry check
establishes neither folding/joint correctness nor mass/COM/inertia accuracy;
the runtime's assumed 20 kg mass is not an independent physical reference.

## Align rigid groups to recovered mounts

A stored subassembly placement needing in-context correction is not itself a
valid motion sample. Fit its initial mounting context first; then use consistent
states for joint inference. `--mount-observation` supplies two end centers in a
selected rigid group's local frame, target centers in the common assembly frame,
and transverse source/target pin directions:

```json
{
  "schema": "rm-mount-alignment-observation-v1",
  "group": 2,
  "source_centers_group_m": [[0, 0, 0], [0.1, 0, 0]],
  "source_pin_axis_group": [0, 0, 1],
  "target_centers_m": [[1, 1, 1], [1, 1.1, 1]],
  "target_pin_axis": [0, 0, 1],
  "evidence": "Identify the native eye/bearing geometry and center measurements"
}
```

```sh
mbx run --manifest-path integrations/scenario-viewer/Cargo.toml --locked \
  --features rxui-viewer --bin rm-rxui-viewer -- \
  --solidworks-project /path/to/grouped.project.json \
  --mount-observation /path/to/mounts.json \
  --fit-mounts /path/to/mount-proposed.project.json
```

The fit rotates/translates one rigid leaf group, preserving local mesh geometry
and Part-to-group transforms. It chooses the closer orientation for an undirected
pin axis. End spacings must agree within 10 µm; no body scaling or invented stroke
compensates for a mismatch. Child-group boundaries require separate review before
this operation. The output records original member poses, supplied centers/axes and the fit under
`provenance.mount_alignment:<group>`, invalidates source-frame approval and clears
attached-joint review. It is a placement proposal, not a recovered joint, hard
stop or calibrated suspension model. Omitting `--fit-mounts` opens the fitted
proposal for GUI inspection. Keep outputs separate from the original project.

## Recover candidate constraints and play mechanism states

`--joint-observations` accepts an explicit parent/child group table with two saved
poses per connection. Poses use the project's common assembly frame (metres,
wxyz); parent motion is removed before analysis. The first sample must match the
current group frames. For example, a child initially 100 mm along parent +X
rotates by one radian about +Z:

```json
{
  "schema": "rm-relative-pose-observations-v1",
  "motions": [{
    "parent": 1,
    "child": 2,
    "evidence": "Identify the source files, configurations and saved states here",
    "mount_point_parent_m": [0, 0, 0.01],
    "states": [
      {
        "parent": {"translation_m": [0, 0, 0], "rotation_wxyz": [1, 0, 0, 0]},
        "child": {"translation_m": [0.1, 0, 0], "rotation_wxyz": [1, 0, 0, 0]}
      },
      {
        "parent": {"translation_m": [0, 0, 0], "rotation_wxyz": [1, 0, 0, 0]},
        "child": {"translation_m": [0.05403023058681398, 0.08414709848078966, 0], "rotation_wxyz": [0.8775825618903728, 0, 0, 0.479425538604203]}
      }
    ]
  }]
}
```

```sh
mbx run --manifest-path integrations/scenario-viewer/Cargo.toml --locked \
  --features rxui-viewer --bin rm-rxui-viewer -- \
  --solidworks-project /path/to/grouped.project.json \
  --joint-observations /path/to/states.json \
  --recover-joints /path/to/proposed.project.json
```

Recovery identifies a possible revolute axis or prismatic direction, or records
unexcited/screw-like motion (which may also represent combined joints). Unexcited bodies are not automatically welded, and a
screw-like delta is not forced into a supported hinge/slider. Rotation under
0.001 rad is rejected when estimating a hinge axis because the pivot is poorly
conditioned. Tests use a 10 µm linear and 1e-6 rad angular reconstruction budget;
these are numerical proposal tolerances, not hardware accuracy specifications.
Two states identify compatible motion, not a unique physical mechanism or the
complete allowed motion space.

A hinge observation identifies an **axis line**, not the bearing plane along it.
Without a mount center, the anchor is the point on that line nearest the parent's
initial origin. Optional `mount_point_parent_m` supplies an independently selected
geometry center: it must lie within 10 µm of the inferred line, and its projection
fixes the anchor along the line. Slider-line offsets remain caller choices.
Source provenance, input pose measurements, observed coordinates and fit residuals are stored under
`provenance.joint_recovery`; changes to recorded Part identities, grouping or
initial poses make proposals stale. This record is not a cryptographic geometry
certificate. Observed
angles/displacements never become hard travel limits automatically.

The command writes editable, **unreviewed** joints and preserves source placements.
Open the proposal, select each moving group, inspect its axis/pivot and explicitly
apply **Hinge** or **Slider** after review. Supply verified stops separately if
known. Existing mass, completeness and source-frame gates still apply. Original
native CAD and the input project are untouched when a separate output is used.

### Physical preview

**Start physical constraint test** cooks the reviewed project once. The slider
then commands a motor rather than prescribing a transform. **Observed pose**
commands a still-current recovered coordinate; **Pause joints**, **Run joints**
and **Reset joints** control this separate explicit 128 Hz plant. Reset reconstructs
fresh bindings at the source's initial pose while retaining cooked resources.
Group/source edits and selection changes stop playback; restart after review.
Saving/exporting always writes the source recipe, not a transient simulated pose.

This mode tests constraints using the declared/estimated inertia. Gravity and
contacts are disabled: overlapping coarse proxies, brake contact, real load,
actuator calibration, gear coupling, springs and damping laws are not validated.
Motors use inertia-scaled illustrative gains (8/s natural frequency, critical
damping, 30 rad/s² or m/s² nominal acceleration cap), not inferred robot settings.
Motor targets respect reviewed stops and the runtime's single-turn coordinate
scope. The renderer updates retained body-local instance frames without recooking
source meshes or replacing mesh/material/entity handles on each step.

```sh
# Start at the source pose and drive toward current recovered coordinates.
mbx run --manifest-path integrations/scenario-viewer/Cargo.toml --locked \
  --features rxui-viewer --bin rm-rxui-viewer -- \
  --solidworks-project /path/to/reviewed.project.json --simulate-joints

# No GPU/window: 256 presentations × four ticks = eight simulated seconds.
mbx run --manifest-path integrations/scenario-viewer/Cargo.toml --locked \
  --features rxui-viewer --bin rm-rxui-viewer -- \
  --solidworks-project /path/to/reviewed.project.json --simulate-joints \
  --mechanism-report /path/to/physical-report.json --frames 256
```

Reports contain sampled joint positions, velocities, motor targets, anchor errors
and actual body poses. Units are explicit per joint; these are diagnostic truth
records, not continuation checkpoints. Finite GPU captures use the
same four ticks per presentation; warming UI/GPU assets adds no physical ticks.

### EGA recovered-mount experiment

Two gimbal hinges were proposed from the native/STEP relative poses. The recovered
line agrees with the native 60–71–5 bearing center and thin-axis orientation;
that bearing center supplies the anchor-plane reference. A separately labelled
seven-Part bench (bearing/support/stator Parts 254, 256, 257, 258, 268; moving
rotor 255 and clamp 269) tracked the alternate 135.0607° and 22.8441° coordinates
with ideal motor playback. This fixture explicitly approves the inferred joints
only for the experiment and uses box-density inertia estimates, not verified CAD
mass. It does not make the entire robot exportable.

The KJ85T33 geometry is a coil-over damper. Anonymous Part 18 is the coil spring;
its 20° saved orientation difference is not evidence of an independent rotary
actuator or joint. Circular end-eye fits and the neighboring native thrust-bearing
axes recover both damper mounting lines. Fitting those native eye frames to the
bearing lines reproduces the held-out STEP main-body pose within 0.00001 mm and
0.00001°, using no STEP pose in the fit. Those are numerical consistency results,
not real mechanical accuracy. The native eye spacing is approximately 87.55 mm
in this saved state; it is not a recovered stroke limit.

The raw damper pin lines are offset roughly 12–18 mm and about 41.7° from their
neighboring bearings. This cannot be explained as compression alone. It suggests
stored subassembly poses need in-context mate resolution, although the original
SolidWorks solve is unavailable. A separate geometry-fitted preview preserves
all raw poses for comparison. End hinges, internal sliding travel, surrounding
wheel-arm loops and spring/damping parameters still need further reconstruction;
no automatic full suspension model is installed in the game. Probe inputs,
mount candidates, fitted previews and physical reports remain ignored under
`local-assets/recovered-mechanisms-20261023/`.

See the [suspension evidence audit](suspension-recovery.md) for native flexible-assembly
flags, candidate mate records, four-bar geometry and independently checked mass
properties, including the remaining stroke and force-law gaps.
