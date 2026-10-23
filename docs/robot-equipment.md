<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Robot equipment prototypes

The renderer constructs these meshes from boxes, chamfered housings, low-sided
cylinders and wheel rollers. It does not load the module STEP files at runtime.
The original files remain in the supplied Downloads directory. This is a visual
and driving prototype, not a competition-compliant robot design.

The `--robot` choice, the title screen's robot page and the view flags are in
[app options](app-options.md); [field package](field-package.md) owns the field.
The [documentation index](README.md) lists every guide.

## Reference audit

Sources were inspected on 2026-09-11. Page numbers below count PDF pages,
including the cover. Source filenames carry 2019/2020 labels, but the AM02,
SM01, FI02 and LI01 guides themselves identify v1.0, 2018.11.

| Module | Reference | Implemented approximation |
|---|---|---|
| AM02 small armor | `RoboMaster 装甲模块 AM02&AM12 2020 使用说明书.pdf`, pp.3–8; `AM02 2019.STEP` | Chamfered shell, smaller central dark panel, four exposed fasteners, rear supports, two rounded white plastic diffusers and SVG-derived white printing. Keeps the existing 141 × 135 × 19 mm physics housing and scoring face. The guide lists 140 × 125 mm; the STEP's axis-aligned envelope is approximately 19 × 130 × 141 mm. These are different envelopes, not grounds for silently changing the scoring constants. |
| AM12 large armor | Same guide, pp.4,6,8; `AM12 2019.STEP` | The source-bound folding Hero now fits a 235 mm-wide physical housing and source mount poses (see hardware-models.md). The guide lists 235 × 127 mm. The supplied assembly is rotated, so its world-axis bounds cannot be used directly as width/height. The legacy procedural prototypes retain AM02 armor. The source-bound Hero replaces its physical shell and artwork but retains the existing effective detector policy; the AM12 scoring region is not newly certified. |
| LI01 light bar | `RoboMaster 裁判系统灯条模块说明书（CN&EN）.pdf`, pp.3–5; `LI01 2020 .STEP` | 285 × 50 × 84.75 mm envelope including two legs, main strip and top auxiliary windows, mounted above rear armor. Ten HP segments are a visualization choice. STEP envelope includes cable/connector protrusions. |
| FI02 field interaction | `RoboMaster 场地交互模块 FI02 2020 说明书.pdf`, pp.3–6; `FI02 2019.STEP` | Drawing dimensions 121.67 × 105.67 × 17.80 mm, four edge indicators and downward-facing detection panel. Mounted beneath the central chassis enclosure with clearance. |
| SM01 speed monitor | `RoboMaster 测速模块 SM01&SM11 使用说明书-CN&EN.pdf`, pp.3–9; `SM01 2019.STEP` | Infantry muzzle shroud, 109.81 mm long, 41.20 mm high, open projectile channel and two sensor stations. Width is an approximate drawing reading. Cable and connector protrusions are omitted. |
| SM11 speed monitor | Same guide, pp.5–6 | Hero shroud, 119.70 mm long and 73.80 mm high, with a 44 mm approximate passage for 42 mm projectiles. No SM11 STEP was supplied; this uses the guide drawing. |
| VT03 transmitter | `RoboMaster裁判系统相机图传模块VT03&VT13使用说明书.pdf`, pp.4–5,12; supplied VT03 transmitter STEP | Artist-adjusted 50 mm wide × 28 mm high × 80 mm deep case, forward lens, top cooling slots, mounting ears and short rear cable tails. Flattened and elongated at user request; these proportions are not measured VT03 dimensions. Mounted on the barrel centerline, just ahead of the cradle. The first-person eye follows the visible lens. |

The paired `-1.pdf` armor and speed-monitor downloads were not needed for this
subset. No dimensions were inferred from those duplicates.

The 2026 robot manufacturing specification V2.0.0, 20260626, was also checked:
section 3.7, Figures 3-36 through 3-38, places the speed monitor at the end of the
launcher; section 3.8, Figures 3-39 through 3-41, places FI02 beneath the chassis
and describes clearance from conductive material; section 3.9, Figures 3-42 and
3-43, identifies the current camera/transmitter arrangement. These informed
placement, not a full compliance audit. The model omits wiring, fastener
threads, connector contacts, optical calibration and electromagnetic behavior.

## Chassis assumptions

Infantry uses a 22 kg chassis, a 410 mm octagonal deck and four 153 mm omni
wheels on the diagonals, 245 mm from centre (approximately ±173 mm in x and y).
The deck corner cut is 95 mm. A solid 30 mm lower extrusion joins the deck
underside, reaching 285 mm in the cardinal directions (570 mm overall), just
outside the tilted armor housings. Its deep diagonal bays leave room for the
wheels. Each wheel has a horizontal semicircular metal guard wrapping around its
footprint, with a 12 mm wide, 20 mm tall band. Both ends join the lower deck.
Twelve convex sectors describe each guard. The first eight prisms form the
lower extrusion and another 48 form the four guards; collision and rendering
share these vertices. The guards stay fixed to the chassis during suspension travel.
Armor faces sit at 250 mm reach. The front/rear pair is 58 mm above the body
centre, 28 mm above the left/right pair, following the stacked beam sets.
Chassis spacers support the raised pair’s unchanged carrier feet. Their physical
housings retain at least 5 mm of horizontal wall clearance; the lower bumper
stays below their lower edges. All bumper parts clear the wheels' swept
suspension envelopes by more than 2 mm.
Two deep bracket side carriers and a horizontal mounting foot depict the
designated support, using the same simplified prisms for collision and rendering.
The foot or its chassis spacer rests on the lower deck; only the carrier touches the module's
rear plane.
Inside the deck shell, two parallel upper beams cross two perpendicular lower
beams. Their ends support the cardinal armor bays, with short posts at the beam
crossings. Each wheel's spring sits inboard, 60% of the way from the body centre
to the hub, and terminates below the roof. A non-spinning carriage arm joins its
moving lower mount to the inner axle face. The guide and spring follow wheel
heave without following axle spin. This is a rough structural depiction using
the existing effective wheel spring/damper rates, rather than articulated joint
physics; at full droop the lower carriage can extend beneath the lower deck.
The CAD-free `omni_chassis` example accepts `open` after the terrain argument to
hide the shell and armor for an internal-frame cutaway (`bare` hides armor only).
The LI01 bar rises on chassis-mounted posts so it remains above the rear armor.
Moving the wheels onto the X diagonals frees the cardinal armor bays, allowing
plates to sit 100 mm lower. The gun pivot returns to 200 mm above the body centre.
The omni/Sentry gimbal uses a low yaw bearing and two solid triangular side
plates, with a chamfered pitch housing mounted at their tops and a small camera
seat. The hinge shaft and joint caps are hidden; there is no U-shaped bridge.
The decorative radar rack and exposed truss/flywheel/feeder detail are omitted.
Camera/SM01 mounts, the visible lens/first-person eye, muzzle and yaw/pitch
ownership remain shared with the existing controls. Collision retains the
conservative neck/cradle envelope.
The rear LI01 is mounted close to the armor: its main LED strip clears the rear
plate top by 2 mm, with the housing inboard of the armor. This lowers the default
mount approximately 20 mm relative to the previous taller posts.
The inspection example’s final `aimed` argument commands about 34° yaw and
20° elevation through the physical gimbal controller.
Extreme downward aim still has the prototype's existing
barrel/body intersection limitations.
This chassis width uses ROB 2027 V1.0.0 Table 2-7's 600 mm stowed cube as guidance
(and 800 mm extended cube, 28 kg excluding referee equipment). The supplied 2026
V2.0.0 Table 2-8 has the same envelopes and a 25 kg limit. These are reference
bounds, not implemented compliance checks: the fixed barrel has no stowing mode.

The mounting reference is ROB 2027 §3.6, S117–S118 (designated bracket), S120
(orientation), S122 (rigid connection), S126–S128 (bumper protection and no other
structure touching armor), and S129 / Figures 3-23–3-24 (unobstructed faces).
Figure 3-15 supplies the carrier outline; its 141 mm width, 15 mm rails and
86 mm mounting spacing guide the rough carrier. Other fitted depths and
clearances are model assumptions. This is a recognizable carrier approximation,
not a manufacturing model or complete angular-occlusion certification.

The user-supplied `sentry_urdf_final.zip` was inspected at zero joint angles.
Its 16 visual STL meshes total 1,156,301 triangles; the assembled bounds are
approximately 496 × 514 × 567 mm. Four roughly 152 mm wheels have diagonal
45° axes, and each `*_hanging_joint` is fixed. It therefore supplies a useful
OmniX geometry/mounting reference, but does not define dynamic suspension.
The source inertia blocks sum to about 8.98 kg and were not adopted as measured
robot mass. The prototype now uses the same diagonal layout, with independently simulated
suspension instead of the reference URDF's fixed hanging joints. Source geometry and extracted review files stay
in ignored `local-assets/`; none is embedded, redistributed or used as a collider.
The source URDF SHA-256 is
`9649bace89c5f9dd69a6b8a7de4c62d8a9923c448462e91366c7582d5b96d61f`.

Each ray wheel has independent spring/damper compression, a 120 mm ordinary travel
limit with a progressive bump stop, 1,000 N/m spring and 135 N s/m damper. A 22 kg
robot settles with about 54 mm sag and 54 N load per wheel on flat ground.
Telescoping guides and helical springs follow the actual wheel centres, including
full droop in the air. The ring supplies ordinary contact support when the robot
tips; it does not add an artificial upright controller.

Drives integrate actual axle speed and a lumped 0.008 kg m² rotational inertia.
Speed controllers request independent motor torques, capped by the assumed
60 N stall force and 3.8 m/s free-running speed; tyre slip limits ground traction
to friction times that wheel's normal load. A slipping or airborne wheel can spin
faster than ground travel. Eight internal drive slices run within each Rapier slice of the 128 Hz
physics tick; their combined mechanical output and copper losses share the
existing 120 W assumed budget. This remains a ray-wheel approximation: no
individual roller contacts, unsprung wheel rigid bodies, motor thermal model or
measured motor/tyre calibration. Reflected wheel inertia does not transfer a
separate axle reaction torque to the chassis.

Axle speed and spring compression survive protocol 59 checkpoints and RMO9
owner anchors; hubs are rebuilt at their compressed/drooped positions. Contact
loads and traction are local diagnostics, recomputed on the next physics tick.
Hero and Balance keep their body geometry and also use the inertial drive model.

Create actual physics-backed review renders without loading the arena:

```sh
nix develop -c cargo run -p rm-sim-app --example omni_chassis --locked -- local-assets/omni.png oblique flat
# Other views: top, side. Replace flat with uneven for a 35 mm front-left-wheel step.
# Append bare to hide armor panels while retaining carriers for inspection.
```

The simulator's referee hardware remains the procedural approximation described
above; the separately reviewed STEP-derived candidates are not runtime mounts yet.


Hero uses a 30 kg, 660 × 560 × 120 mm body envelope, 203 mm mecanum wheels,
65 mm wheel width and a taller, wider gimbal. These are design choices, not
rulebook dimensions. The muzzle sits 350 mm ahead of the gun pivot, leaving an exposed barrel
section behind the speed monitor. This length is an app design choice.
Both retain four small scoring modules. Hero retains its conservative box
collider; individual decorative modules and rollers do not add colliders.

Hero wheel axles are parallel. Its contact-force directions are mirrored at
45 degrees and use the hub cross rolling-direction term for yaw. Wheel spin
accounts for the projection onto that direction. This extends the existing
ideal roller/suspension model; it does not model each roller contact or a calibrated motor
controller. Renderer wheel centres follow the suspended positions on every snapshot.

Each pilot names its robot in `Hello` (protocol 33): the Hero, or the
Infantry 3, 4 or 5. The host spawns the matching chassis, records its kind
with the referee (which keeps the one configured HP) and fixes its caliber,
42 mm for the Hero and 17 mm for the infantries; a weapon update naming
another caliber is refused. The three infantries differ only in the number
painted on their armor, which the roster carries to every client; a chassis
the roster has not described yet shows 1 on a mecanum body and 3 otherwise
and is repainted when the roster arrives. The serialized chassis
configuration includes a defaulted `mecanum` flag; older snapshots without it
remain omni. Use matching client/server builds to display Hero correctly.
There are no robot-specific heat, power, ammunition or firing-rate policies.

## Lights and review fixtures

LI01 uses the referee HP fraction, rounded up to the next visible tenth.
Without a referee an alive chassis displays full HP. Defeat extinguishes LI01,
FI02, VT03 and speed-monitor indicators. Armor uses the existing per-plate hit
flash; the app's 50 ms grey flash is not a module-manual timing constant.

FI02, VT03 and speed-monitor LEDs otherwise show a static powered team color.
They do not claim a successful card read, measured velocity or radio link.
The speed monitor guide p.7 describes a waterfall effect on projectile passage;
that sequence is not implemented in live play. The comparison example stages
HP, a frozen front-plate hit flash, or defeat for visual review. The renderer
has no timer and all displayed states come from its caller.

### Armor light diffuser appearance

The supplied `RoboMaster 装甲模块 AM12 2019.STEP` was inspected with OCCT on
2026-09-11. Its rotated assembly contains paired narrow cover solids, but the
inherited solid color is the same pale blue CAD style on every component.
That styling is not a measurement of the diffuser's physical color or emission.
The off-state white plastic follows the user's reference correction.

Chassis and outpost armor now share opaque, nonmetallic, off-white diffuser
materials. Rounded shallow covers replace box-shaped or flat light strips.
The existing light-center span and tip-to-tip height remain unchanged.
Curvature, depth, roughness and reflectance are visual fits; no STEP triangles
or module geometry are embedded. The legacy procedural prototypes use small armor; the optional source-bound Hero now has fitted large physical housings.

When powered, an intensity texture gives the team-colored emission a soft
center and edge/end falloff, leaving a reflective plastic rim. The emission
multiplier is an appearance setting, not calibrated LED radiance. Existing
camera exposure and bloom are retained. A strike uses the existing grey flash
with zero emission. Defeat removes emission while the diffuser stays white;
the printed identifier stays passive white in every state.

The powered appearance was compared against original photos in the sibling
`Vision/Yolo-Detector` repository's `data/armor_image_corpus_v2/images` corpus.
That corpus is an external source and is not retained in either checkout, so the
comparison cannot be re-run from this repository. The named references include
`armor27c_curated/gkd/images/train/4263.jpg`,
`gkd_labeled_rar/已标注数据集/已标注数据集/train/image/3676.jpg` and
`gmaster_detection_zip/XJTLU_2023_Detection_ALL/images/8469.jpg`.
These show bright, near-white bar centers surrounded by team-colored edges.
The shared emission profile now reaches that brighter center through tone
mapping. The corpus spans different exposures and includes synthetic images;
only the photographed robots guided this qualitative fit. No corpus images
are embedded or redistributed, and this is not a calibrated camera or LED model.

## Robot and chassis selection

The title flow selects the robot/team first, then a compatible chassis.
`--chassis auto|omni|balance|mecanum|flight` exposes the same selection to the CLI.
Infantry 3/4/5 offers omni or balance; Sentry uses omni; Hero and Engineer use
mecanum; Drone uses Flight. The host validates the pairing. Hello, snapshots and owner anchors
carry the selection and jump state in protocol 52 (`RMI5` input batches,
`RMO9` owner anchors); use matching builds.

All new models are authored boxes, cylinders and wheel details. The supplied
`sentry_urdf_final/robot.urdf`, `combine_urdf/urdf/combine_urdf.urdf` and
`【装配体】轮腿步兵总装_V4.STEP` were inspected on 2026-09-18 only as references.
Their meshes are not copied, bundled or loaded. The Sentry reference has an
approximately 496 × 514 mm lower-body envelope and 153 mm wheels. Omni Infantry
shares the compact core, exposed motors and open corner frame, omitting
the Sentry sensor tower. The Engineer reference's chassis envelope is about
635 × 562 mm; its approximate folded arm is decorative and has no controls.
Engineer has no launcher. Sentry uses normal manual Infantry drive/aim/fire.
Its sensor rack is attached to the pitch cradle, following both gimbal axes
above and behind the camera instead of rotating across its view with the body.
Ground robots retain four small visible scoring modules; decorative arms, towers and links do not
add mesh colliders. Hero's geometry and dynamics remain unchanged.

The omni wheel contacts lie on the four diagonals, 245 mm from the centre.
Their double roller rows, spokes and motor mounts are procedural details.
The omni mock uses softer 1 kN/m springs with 80 mm unloaded extension
(about 54 mm static sag) to retain independent ground contact on ramps.
Its assumed 60 N drives and 1.0 drive friction apply to all four wheels, each
contributing through a 45-degree rolling direction during a straight run. Hero and Engineer retain their
original suspension and tyre tuning.
Engineer uses an open truss frame with shoulder, elbow and wrist housings,
paired arm plates and a canted open gripper.

Balance uses two 200 mm wheels and a narrower electronics pod inside its
360 × 480 × 180 mm collision envelope. Each leg is a serial hip–knee–wheel
chain. A four-bar parallelogram transfers the coaxial hip motor's angle to the
knee; a slim pushrod is separated from the load-bearing thigh. The drawn links
solve a two-link inverse-kinematics pose from the caller's suspension-following
wheel hub. Their lengths are 145 mm; knee-drive cranks are 45 mm. This is a
procedural visual mechanism, not independently simulated joint dynamics.

Stabilization uses a reduced continuous-time LQR for pitch and pitch rate:
`A = [[0,1],[mgh/I,0]]`, `B = [0,1/I]`, `Q = diag(1000,60)`, `R = 0.01`.
The closed-form Riccati solution computes gains once from the preset mass,
fitted inertia and centre-of-mass height. A bounded speed-error lean setpoint
encourages forward acceleration, and a 180 N m torque cap limits correction.
Settings > Controls scales this torque from 0 to 100 percent, default 100;
zero removes stabilization entirely. This reduced body-torque controller is
not the reference firmware's ten-state wheel-leg controller. No firmware code,
solved firmware gain table, mesh or sibling dependency is copied. Jump is
represented by one equivalent 2.4 m/s upward push (about 0.29 m free-flight rise)
when both wheels support an upright body. Press Space once; holding it cannot
repeat the jump. Airborne and defeated robots cannot launch. The input latch is
part of the restorable snapshot and compact owner stream.

Drone is an original four-rotor approximation: diagonal arms, motor cans,
three-blade propellers, landing skids, and twin guard hoops connected by a
crossed-wire side mesh around each rotor. Its 0.8 m square collision envelope
is approximate. Gravity, pitch/roll and vertical translation are locked for
this planar prototype; velocity feedback accelerates horizontally up to
4 m/s². The plane starts 2.0 m above the team's landing pad (read off
Figure 4-4 and the minimap artwork) and does not follow terrain. Wish velocity
is in the body frame: Q/E yaw the body and the gimbal aims separately.

The section 4.5 Aerial Safety Rope is modelled as `ChassisConfig::tether`: the
2.4 m tether's hook slides along a straight rope from the team's field edge to
the centre line (Snap Ring about 14 m from the edge) over the landing pad's
side of the field. The rope height is not given by the rulebook; 3.6 m is
assumed. Inside the reach the drone's outward wish is limited to the speed it
can shed at its 4 m/s² acceleration; past it, a horizontal spring-damper
pulls it back, capped at 20 m/s². The tether travels in the exact chassis
configuration, so restores and prediction hold it. The rope's elastic force
under 5 N and jamming after repeated turns are not modelled. Walls still collide with the body. There is no rotor aerodynamics,
altitude control or new drone-rule enforcement. A small two-axis 17 mm launcher
is mounted 140 mm below and 200 mm ahead of the body centre, clear of the rotor
plane, with the normal aim camera and muzzle. Mounted ahead, a forward shot
clears the 0.8 m frame at every pitch the flight controls allow; shots aimed
sideways or backward and upward still meet the frame, as on a real underslung
gimbal. The flying body keeps all its mass centred so its roll and pitch locks
hold. The live referee disables the air-support gate
for this prototype because its activation controls are not integrated; heat,
ammunition policy and host firing cadence remain authoritative.

Render a CAD-free comparison with:

```sh
cargo run -p rm-sim-render --example robots -- /tmp/robots.png healthy front all
```

## Seasonal RMUC rosters

The verified field package selects the displayed roster. `server::roster` is
shared by deployment, HUD and host admission; it distinguishes official equipment
from available practice prototypes. The ordinary `Robot` wire enum continues to
identify existing drivable prototypes, so an unsupported system never becomes a
fake pilot or chassis. Free practice still permits duplicate prototypes; roster
counts are a reference, not a competition registration enforcement system.

| Number | RMUC 2026 | RMUC 2027 (Table 3-1) | 2027 practice support |
|---|---|---|---|
| 1 | Hero | Heavy ×1 | Not implemented |
| 2 | Engineer | — | — |
| 3 | Infantry 3 | Infantry 3 ×1 | Omni / assisted balance |
| 4 | Infantry 4 | Infantry 4 ×1 | Omni / assisted balance |
| 5 | Infantry 5 | — | — |
| 6 | Drone | Aerial system: one hangar + four aircraft | Not implemented |
| 7 | Sentry | Sentry ×1 | Manual omni prototype |
| 8 | Dart system | One launcher + four darts | Not implemented |
| 9 | Radar | One compute module + one sensor module | Not implemented |

These are active equipment slots, excluding reserves. The 2026 list includes all
optional role slots rather than enforcing a legal lineup. The HUD always shows
all slots, including empty or unimplemented equipment. Tab lists subordinate
equipment counts. Connected deployment offers implemented prototypes and lists
unsupported systems. A 2027 host rejects Hero, Engineer, Infantry 5 and the old
Drone through both ordinary and staged spawn paths. The old Drone is not a
2027 aerial system, and Hero is not a Heavy approximation. Infantry and Sentry
remain practice prototypes. The bound RMUC 2027 field supports the limited
live control/base/outpost subset described in [the field guide](rmuc2027-field.md);
full 2027 equipment and match rules remain incomplete.

### Stair-climbing donor and mechanism tests

The optional `dreamchaser-stair-heavy-v1` package binds DreamChaser's source
front links, coupler, rear legs, sliding carriage, guide bearings and four wheel
hubs separately. The profile uses the 2026 Engineer practice slot; it does not
admit the unfinished RM2027 Heavy gameplay role. Launch with:

```sh
just run --cad-assets field --robot-assets local-assets/robot-cad-survey/bound-stair-heavy --third-person
```

T alternates raised-front stance and level travel. Raise the front wheels before
meeting a step, drive onto it, then return the linkage to travel to lift the rear
wheels. Repeat for the next step. Motion uses wheel forces and suspension in the
ordinary physics world, with no chassis pose/height injection. The lower bay
stays open in collision; the upper rails and rear electronics shelf are solid.
Moving link proxies stop actuation against external obstacles.

The closed planar linkage follows the source CAD bearings and technical report
Figures 2–3: front pin 130 mm, front wheel arm 190 mm, coupler 200 mm and rear
leg 320 mm. Travel is 5.309° absolute front-link angle; the source deployed pose
is 76.220°. The actuator is prescribed at an assumed 40°/s, with assumed 38 kg
mass and spring/motor tuning. This is not a calibrated torque/gearbox simulation.
The upper manipulation arm and auxiliary sensor linkages remain fixed; PIE
camera/UWB/arm retraction is outside this pass. The folding Hero continues to
articulate its source arm, camera, muzzle and collision proxies with T.

Reproduce the deterministic contact tests without a GPU:

```sh
nix develop -c cargo build -p rm-sim-physics --example robot_mechanisms --locked
python3 scripts/robot-mechanism-scenarios.py
```

The course has two 200 mm steps. It verifies a 400 mm rise, four final wheel
contacts and a returned travel pose. The same drive commands with no linkage
deployment stop at the first step. A separate Hero script folds and unfolds.
Unit tests cover closed lengths, invalid closures, mid-climb restore/replay,
Hero roof blocking and owner-anchor serialization of both mechanisms.

Source assembly export lives in rm-map-tools:
`benchmarks/2026-10-18-team-robot/bind_stairs.py`, with reviewed membership and
analytic frames in `python/stair_linkage.py`. It preserves source triangles
apart from the separately documented approved armor replacement. The package
retains source credits and terms; it is generated locally, not embedded in code.

Check the app input, loaded source groups, intermediate/end angles and return
through the real console and host path:

```sh
python3 scripts/robot-mechanism-console.py --out local-assets/mechanism-console
```

Run once for each package from a fresh paused session. The script saves travel,
mid-transform, deployed and returned screenshots plus a result JSON.

For an offline video of the same crossing, save the example's CSV (now including
body quaternion, all wheel centres and wheel spin), then render with the CAD
tool environment:

```sh
target/nix/debug/examples/robot_mechanisms stairs steps COMMANDS.csv > local-assets/stair-motion.csv
../rm-map-tools/ocpenv/bin/python scripts/render-stair-video.py \
  --package local-assets/robot-cad-survey/bound-stair-heavy \
  --motion local-assets/stair-motion.csv --out local-assets/stair-crossing.mp4
```

The commands are the stair scenario in `scripts/robot-mechanism-scenarios.py`.
The 32 fps side-following clip replays recorded 128 Hz physics. Source-bound
shells follow their rigid frames and the wheel centres/spin follow the physics
snapshot. It is a diffuse CAD inspection render with simplified armor artwork,
rather than an in-game screen recording. Step geometry matches the physical
fixture; the camera follows the chassis without changing its motion. Requires
rm-map-tools' Python environment, its diagnostic C++ rasterizer, clang++ and
FFmpeg. Video and preview frames stay in ignored local-assets.

### Tunnel-capable source packages

Two more optional runtime packages use checksummed `chassis.json` configurations
with source-fitted serial joints, oriented collision envelopes, armor faces and
optical mounts. The host carries this complete configuration in checkpoints and
owner anchors (protocol 64); the renderer receives named poses from the app.

| Package | Practice slot | Height on level loaded suspension | Mechanism |
| --- | --- | --- | --- |
| `bound-tunnel-heavy` (PIE) | 2026 Engineer | about 302 mm open / 228 mm stowed | T levels the arm, lowers the camera carriage and retracts UWB |
| `bound-tunnel-infantry` (Future) | Infantry, Omni | about 249 mm at neutral pitch | Fixed-height chassis; source yaw and firing pitch articulate |
| `bound-hero-armored` (Artisans) | 2026 Hero | Folded profile passes the 250 mm fixture | Existing T folding arm |

These measurements include loaded tyre placement. Future's wheel **assembly**
bounds include suspension/mounting hardware and must not be used as its tyre
centre/radius. Its small clearance is a level-floor result, not a guarantee over
bumps or with the gun elevated. No model is scaled to fit the opening.

```sh
just run --cad-assets field --robot-assets local-assets/robot-cad-survey/bound-tunnel-heavy --third-person
just run --cad-assets field --robot-assets local-assets/robot-cad-survey/bound-tunnel-infantry --third-person
```

PIE remains an Engineer practice donor, without admitting RM2027 Heavy gameplay
or manipulation/assembly. Its report §4.2 and the CAD joint axes support the
prescribed stow: shoulder levels by 7°, elbow counter-rotates, camera carriage
travels 70 mm along its 45° rail, and UWB rotates 20° at its independent hinge.
T requests an endpoint; external obstacles stop an opening arm and allow it to
retract again. The sequence is a fitted simulator policy with an assumed input
rate, not recovered firmware, torque calibration or a complete six-axis arm
controller. Source self-collision and flexible belts/cables are not simulated.
Reviewed dangling motor leads are cut, retaining the motor housings and shafts.

Future's source gimbal is rotated at its yaw bearing into a forward neutral pose.
Its barrel, camera and collision follow prescribed yaw/pitch joints. The small
pitch linkage is an approximate visual binding, not a solved four-bar mechanism.
Source materials and vertex/index buffers are retained exactly. Open/unclassified
CAD shells remain visual geometry; collision uses fitted physical solid envelopes
and the simulator's replacement armor modules. These envelopes are conservative
approximations, including an oriented box along PIE's long shoulder link to avoid
filling empty space above the stowed arm.

The temporary SaltWave Sentry is not certified for the tunnel: its fixed upper
structure is too tall and no source folding mechanism has been identified. EGA's
user-supplied STEP now has a
[fitted Sentry package](hardware-models.md#ega-tunnel-sentry-runtime-package),
including source-fitted pitch/UWB parallelograms and source-mounted armor. Its
corrected fold passes the 250 mm fixture with the assumed suspension settling,
and clears the current 650 mm passages. The endpoint and spring parameters
remain simulator policies, with measured geometry and clearance results in that guide.

Rebuild the packages with rm-map-tools' `bind_tunnel.py`. Export the ordinary
Engineer config as the documented baseline rather than copying a live snapshot:

```sh
nix develop -c cargo build -p rm-sim-app --example robot_tunnel --locked
target/nix/debug/examples/robot_tunnel export-base local-assets/tunnel-base-config.json
../rm-map-tools/ocpenv/bin/python ../rm-map-tools/benchmarks/2026-10-18-team-robot/bind_tunnel.py \
  --sources local-assets/robot-cad-survey --out local-assets/robot-cad-survey/bound-tunnel-heavy \
  --base-config local-assets/tunnel-base-config.json
# Add --source infantry and change --out to bound-tunnel-infantry for Future.
```

Run the contact fixture (2 m long, 250 mm roof clearance) without a GPU:

```sh
target/nix/debug/examples/robot_tunnel local-assets/robot-cad-survey/bound-tunnel-heavy/chassis.json stowed
target/nix/debug/examples/robot_tunnel local-assets/robot-cad-survey/bound-tunnel-heavy/chassis.json open
target/nix/debug/examples/robot_tunnel local-assets/robot-cad-survey/bound-tunnel-infantry/chassis.json stowed
target/nix/debug/examples/robot_tunnel export-hero local-assets/hero-tunnel-config.json
target/nix/debug/examples/robot_tunnel local-assets/hero-tunnel-config.json stowed
```

The open PIE control must stop before entering. The stowed PIE case also attempts
to open under the roof, verifies blocking, stows again and exits. Each run restores
a checkpoint mid-sequence and then continues through ordinary contact physics.
Use `scripts/robot-mechanism-console.py` with a fresh PIE session to test T through
the actual app/host path and save intermediate and endpoint screenshots.

Run the complete positive/negative fixture matrix with:

```sh
python3 scripts/tunnel-mechanism-scenarios.py
```

It additionally checks Future against a 240 mm roof, the Hero in both open
and folded poses, and EGA stowed versus raised under 250 mm. EGA must block
unfolding and report interior collider heights below the roof (2 mm solver
tolerance); the measured EGA peak is about 246.6 mm. The generated packages and visual reviews remain local;
committed code contains recipes and fitted simulation configuration contracts.

Record the tunnel fixture at 32 fps (every fourth 128 Hz tick) by supplying a
fourth positional argument. JSONL contains the configuration, reference joint
frames, and each frame's actual body, source joints, wheel centres and spin:

```sh
target/nix/debug/examples/robot_tunnel local-assets/robot-cad-survey/bound-tunnel-infantry/chassis.json \
  stowed .25 local-assets/future-tunnel-motion.jsonl
../rm-map-tools/ocpenv/bin/python scripts/render-tunnel-video.py \
  --package local-assets/robot-cad-survey/bound-tunnel-infantry \
  --motion local-assets/future-tunnel-motion.jsonl --out local-assets/future-tunnel.mp4 \
  --title 'Future Omni Infantry'
```

Use the PIE package/config or exported Hero config and `bound-hero-armored` for
the other clips. The offline renderer consumes Rust-evaluated source joint
frames and recorded wheel motion. The far strip of the actual roof is rendered
as a cutaway; cyan dashes show its 250 mm underside at the centre plane. This
makes small clearances visible without hiding the robot behind the full roof.
The rendered roof cutaway does not change the physical collision fixture.
