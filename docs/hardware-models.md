<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Referee hardware model candidates

The supplied AM02/AM12 (2019), LI01 (2020), SM01 (2019), FI02 (2019) and VT03
STEP files are processed by `../rm-map-tools/python/referee_hardware.py`.
The reproducible recipe, source hashes and review process live in that
repository's `docs/referee-hardware.md` and `rules/referee-hardware.json`.

The rough tier contains 1,513 triangles for one of each of the six modules,
compared with 551,357 source tessellation triangles. A finer coarse tier is
available for comparison. The local review package is generated with:

```sh
cd ../rm-map-tools
ocpenv/bin/python python/referee_hardware.py --sources source/referee-hardware --out out/referee-hardware
ocpenv/bin/python python/preview_referee_hardware.py out/referee-hardware
```

Source filenames are `AM02.step`, `AM12.step`, `VT03.step`, `LI01.step`,
`SM01.step` and `FI02.step`; the supplied checksums are required. Original STEP,
reference exports, candidate GLBs, NPZ and renders are regenerable local files.
No CAD is added to the simulator's embedded `assets/` or installed field packages.

Geometry and appearance remain DJI / RoboMaster source data under their terms.
AM02, AM12 and FI02 have source colors; the three remaining files are unstyled
and use an explicitly neutral fallback. No realistic paint or emission is inferred.

Rough source-solid nodes preserve their checksum-scoped native ordinals. These
are visual candidates, with no collision/scoring eligibility. They retain source
axes, recentered in metres; mounting and sensor frames are unclassified. The SM01
open housing and VT03 housing connectors are protected from envelope replacement.
Both candidate tiers omit VT03, LI01 and SM01 cables and cable-end plugs;
housing sockets remain, and source reference exports stay complete.
Native holes, small fasteners and interior details may otherwise be filled or
removed. A future robot description must bind mounting, LED and detector parts
explicitly, keeping these older hardware revisions distinct from 2027 rules.

## Team robot CAD investigation (2026-10-18)

The robot visuals can instead come from real team STEP assemblies. The
noncommercial RoboMaster team-to-team simulator use is the intended use agreed
with the user. Preserve the individual authors' terms and attribution; these
assets do not acquire this repository's software license. The approved assembled prototype can be loaded explicitly; procedural visuals remain
the default.

The following primary releases were found through `bbs.herkules.dev`. Direct
attachment links and authors are retained in the local survey JSON beside the
downloaded originals under `local-assets/robot-cad-survey/`.

| Candidate | Available geometry | Intended use in the simulator |
| --- | --- | --- |
| [Zhongzhou Zhulong 26.5 chassis, 芝士披萨k](https://bbs.herkules.dev/articles/01M0Z3C64A3ACBNKZVA1R041AV) | Direct STEP, imported locally; 637 native solid groups | Omni chassis, guards, armor carriers and suspension; no gimbal included |
| [Zhejiang University Hello World 2026 gimbal, Chenic](https://bbs.herkules.dev/articles/01M1NA853WMDH11GRJBZW6WDRE) | Downloaded ZIP with separate Omni and Balance STEP variants | Source-derived yaw/pitch assembly; source explicitly requests attribution and limits use to noncommercial RoboMaster team exchange |
| [Northeast Agricultural University TCP suspension, yep~](https://bbs.herkules.dev/articles/01M0Z3PSB96R9CHB68MEPNZ7RP) | Direct STEP, imported locally; 192 native solid groups | Independent omni wheel and compact suspension reference |
| [BISTU Xingchen 2026 omni, 内谁](https://bbs.herkules.dev/articles/01M0Z67EXHDMNRHBJS4EPC8MJY) | Direct mechanical ZIP link; archive not inspected | Complete alternative robot with direct-drive yaw/pitch; source limits use/redistribution to authorized team exchange |
| [China University of Petroleum RPS 2025 serial-leg infantry, OldWu](https://bbs.herkules.dev/articles/01M0ZGPTW46MBNS2GRBMQNVH1H) | Release describes native assembly and matching STEP | Later serial-leg balance model, not a parallel-leg substitute; noncommercial team exchange terms |
| [HKUST ENTERPRIZE 2024 mecanum, Allen_htl](https://bbs.herkules.dev/articles/01M0ZGPTW97HCSH7XK336KSX27) | Release describes full STEP and SolidWorks subassemblies; Baidu download | Later mecanum chassis and two-axis gimbal; noncommercial technical exchange terms |
| [HKUST ENTERPRIZE 2025 swerve, 港科SHK](https://bbs.herkules.dev/articles/01M0ZHB2ASR739A95PG0Z5RS8W) | Native drawings and quoted STEP part files; Baidu downloads | Later steering/drive/suspension module; source requests credit |

The three STEP trials keep every source-solid group and simplify each
solid/material region separately. Reference tessellation uses 0.25 mm / 0.3 rad;
the trial uses 1 mm / 0.8 rad plus a 1 mm absolute meshoptimizer error metric.
These are visual experiments, not a certified whole-model distance bound.

| Trial | Reference triangles | Trial triangles | Reduction |
| --- | ---: | ---: | ---: |
| Zhulong chassis | 1,885,306 | 110,875 | 94.12% |
| TCP wheel/suspension | 426,214 | 16,460 | 96.14% |
| Hello World Omni gimbal assembly | 2,105,007 | 79,280 | 96.23% |

Three matched orthographic views have foreground-mask XOR/union differences up
to 1.65%, 2.65% and 1.28%, respectively. This measures the projected foreground,
including openings, rather than all surface/shading changes. Roller faceting is
visible at this aggressive close-up tier. Nothing has yet removed fasteners,
filled holes, replaced solids with convex hulls, or baked surface detail.
The gimbal trial includes its source mounting/electronics tray and 604 solids
plus a separate non-solid geometry group. These assemblies are independently
positioned candidates, not a mounting-compatible combined robot.

The next import should preserve named source assemblies and exact placements,
then bind chassis, wheel spin, suspension, yaw and pitch frames explicitly.
Keep higher detail on wheels, armor outlines, camera and muzzle; simplify hidden
hardware and bake shallow detail separately. Merge/instance only within the
same motion group. Visual meshes must not become scoring masks or detailed
triangle colliders: existing wheel forces remain the dynamics authority until
the source dimensions and joint frames are reviewed.

Each local trial GLB embeds the team/author in `asset.copyright`, with the
source article URL, STEP SHA-256 and native solid index on its nodes. Keep the
original terms, original download URL, transformations and simplification recipe
in the eventual robot package's notice/manifest as well. Do not infer real paint
from an unstyled CAD surface or automatically turn every colored part into a
team-colored/emissive material.

## Assembled Omni prototype

`../rm-map-tools/benchmarks/2026-10-18-team-robot/` now contains the source
preparation, occurrence audit, pinned assembly recipe, exporter, review renderer
and real-asset validator. The local package is
`local-assets/robot-cad-survey/assembled/`: `neutral.glb`, `aimed.glb`,
`depressed.glb`, `animated.glb`, source/part manifests, `CAD-NOTICE.md` and
matched review images. The game can load this package with `--robot-assets`.

The assembly combines the complete Zhulong chassis (637 solids, 110,875
triangles) and the Hello World rotating gimbal plus its bearing, yaw drive,
crossbeam and tensioners (431 solids, 55,086 triangles). The duplicate lower
gimbal ammunition/electronics structure and loader are omitted; the separate
TCP wheel study is not included. Four plain 8 mm square integration posts add
48 triangles between the source mounting plate and crossbeam. These posts are
a visual fitting assumption rather than an approved manufactured adapter.

Total: **166,009 triangles**. Neutral dimensions are approximately **579 mm
wide, 546 mm long and 577 mm tall**; the two reviewed aimed poses stay below
590 mm tall. Source scale is unchanged. These are measured visual extents,
not a robot compliance determination.

The GLB uses metres, X-left/Y-up/Z-forward; convert to FLU as `[z, x, y]`.
Its floor is at Y=0. Source solids retain original occurrence paths, names,
source SHA-256 values and material metadata beneath explicit motion groups.
The eight animation channels drive yaw, pitch, its motor crank and
counter-rotating parallelogram rod, plus four independent wheel spin axes.
The pitch pivot and crank pin centers come from the source bearing geometry.
Native suspension parts remain separate at their source rest pose; spring,
damper, linkage travel, masses and inertia are not derived from the meshes.

Validation checks exact retained source material dictionaries and metadata,
complete/disjoint solid ownership and rest triangle coordinates within 0.2 µm
of the cooked inputs (observed maximum 0.015 µm from float32 rebasing).
The connecting rod closes at both pins over 299 sampled yaw/pitch poses.
Across the reviewed pitch range, moving gimbal geometry stays at least
40.95 mm above the chassis's highest source geometry. This does not certify
interference among fixed integration parts, ammunition-feed compatibility,
full mechanical travel, collision/scoring behavior or suspension dynamics.

Rebuild commands and original source identities are documented in the sibling
repository's `docs/referee-hardware.md`, under **Team robot assembly prototype**.
Keep simplified collision and armor detection definitions independent of visual
mesh triangles. Further per-part visual simplification can preserve these motion groups.


## Live game validation

```sh
just run --robot-assets local-assets/robot-cad-survey/assembled --start-paused
```

The app checks the manifest's source identities and `neutral.glb` SHA-256 before
loading it from a separate asset source. Local hosts automatically select the
source-fitted Omni preset; a headless host selects it with `--team-cad`.
Remote clients need the same optional package to display the imported model;
without it they show procedural visuals using the host's physical dimensions.
Hero, Engineer, Balance and Drone retain their existing presets.

The passive renderer binds body, yaw, pitch, motor crank, connecting rod and four
wheel groups. Independent wheel centres and spin follow the simulated wheels;
yaw, pitch and parallelogram motion follow the snapshot. Armor artwork and team
lights overlay the source modules. Front/rear scoring faces are 47 mm below the
side pair; the muzzle is 206.577 mm ahead of the pitch pivot. First-person camera
placement uses the source lens envelope. Protocol 60 carries the fitted profile.

The live console exercise loaded all nine motion groups, drove 0.88 m in one
simulated second, strafed, aimed, fired through the ordinary host path, and
captured first-person, third-person and collider views on the 2027 field. A flat
physics regression verifies settled height, all four suspension contacts and
muzzle alignment. Snapshot tests preserve the fitted configuration exactly.

Physics remains the existing independent ray-wheel spring/damper and torque/slip
model, with assumed mass and tuning. Source suspension arms and springs remain
at their rest pose while wheel hubs travel; the mechanism is not an articulated
CAD-derived suspension. Collisions use conservative simple body/guard/ammunition
and turret proxies plus the scoring housings, rather than 166,009 visual triangles.
The turret proxy does not articulate with the detailed gimbal. These are gameplay
approximations, not validated real-robot mass, inertia or contact geometry.


The imported prototype uses a user-selected dark graphite display material for
explicitly uncolored surfaces and its uniform pale blue CAD tint (sRGB
`0.7922, 0.8196, 0.9333`). The latter is an intentional presentation override,
not a claim that the STEP lacks a color. Authored white, black, metal greys and
yellow remain unchanged. Original GLBs, checksums and color metadata are retained.
This is a carbon-fiber-like color/roughness treatment without a weave texture.


## Complete tunnel Infantry and Hero candidates

The same source-solid/material simplification pipeline now has two more complete
source assemblies, pinned in rm-map-tools' `candidates.json`:

| Source | Native solids | Visual triangles | Source-pose width × length × height |
| --- | ---: | ---: | --- |
| [Qingdao University Future / 夜猫子祖先, RM2026 tunnel Infantry](https://bbs.herkules.dev/articles/01M0Z3PSBB7SCJAQM749SETCH4) | 1,233 | 190,438 | 663 × 584 × 253 mm |
| [Anhui Institute of Information Engineering Artisans / 北极, RM2026 tunnel Hero](https://bbs.herkules.dev/articles/01M0Z88C7S9FRCVHS1AHS0S5KW) | 738 | 206,777 | 787 × 835 × 423 mm |

The Future technical report explicitly describes a fixed-height omni design for
the 250 mm tunnel. The source pose's complete visual envelope is about 253 mm;
ride height and clearance still need checking under suspension load. Do not infer
compliance or scale the model to force a fit. Its hierarchy identifies four wheel
groups and the low waist-feed gimbal. Artisans' Hero has wheel-leg assemblies and
a folding side-feed launcher; its published source pose is not the folded tunnel
pose. It must not inherit the current mecanum Hero's geometry or dynamics blindly.

The source STEP colors are sparse: Artisans is entirely unstyled, while Future
has unstyled faces and the same pale CAD tint. Each local package under
`local-assets/robot-cad-survey/<source_model_id>-pilot/package/` contains:

- `model.glb`: original source appearance, named assembly hierarchy, metre units,
  a floor-aligned review frame and unchanged cooked mesh buffers.
- `graphite.glb`: the same geometry with the user-selected graphite material;
  the original material factors and color provenance remain in metadata.
- `manifest.json` and `CAD-NOTICE.md`: pinned STEP/cooked/hierarchy identities,
  output checksums, counts, author terms and explicit review-only scope.

The 797-group Infantry and 216-group Hero hierarchies retain named source
assemblies. Unclassified non-solid geometry remains present at the root rather
than being dropped. Packaging verifies unchanged mesh buffers/materials in
`model.glb`, single attachment of each cooked mesh node and identical placed
triangle counts. The two candidate packages are visual exports, not admitted by
the existing `--robot-assets` fitted profile. Their wheel, armor, camera, muzzle
and folding/pitch bindings remain to be fitted before playable integration.

Neutral grey previews make unpainted geometry easier to inspect; separate dark
previews show the graphite display policy. Neither invents source paint. The
export commands live in the sibling repository's `docs/referee-hardware.md`.

Other reviewed releases: TCR's folding omni has a complete SolidWorks assembly
but only component STEP files, Climber's directly attached ZIPs are control code
(the CAD is on Baidu), and Qiqi's directly attached complete tunnel Infantry is
SLDPRT. These were not presented as complete STEP imports.
