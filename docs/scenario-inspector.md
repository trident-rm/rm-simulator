<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Optional RXUI component inspector

`integrations/scenario-inspector/` is an independent Cargo workspace for inspecting
verified equipment geometry. It does not change the interactive Bevy application,
normal workspace dependencies, networking or match authority. It currently
consumes `rm_sim_assets::{components, scenarios}` and the shared
physics motion/pose helpers. Outpost, Base and Rune selections create a physical
`RobotSession` with a configurable existing omni chassis through `Scenario::session` and
`RobotSpawn`. It requires the component composition changes in
the same checkout; it is not usable against the preceding assets API.

RXUI is pinned to
[`656bf5ed460ed9604b65fa9538354fb6c746a7d4`](https://github.com/hxyulin/rxui/tree/656bf5ed460ed9604b65fa9538354fb6c746a7d4).
Its upstream workspace pins Astrelis to
`604b9fedb62009dc9600963ab98a3cc00a31fe29`. RXUI describes this API as an
unreleased rewrite; the separate inspector lockfile records the complete graph.
The native host requires Rust 1.98.1 or newer, platform window libraries and a
working wgpu backend. Rust 1.99.0 was used for validation.

## Run

From the repository root:

```sh
cargo run --locked --manifest-path integrations/scenario-inspector/Cargo.toml \
  --features native -- --asset outpost
```

The default package discovery is the ordinary `CadAssets` discovery for
`field-2027/`. Override with `--cad-assets PATH`. Loading verifies the manifests,
visual/collision checksums and native bindings; the inspector never rewrites the
package or regenerates geometry. Initial loading happens before the window opens;
subsequent selection loads on an RXUI blocking worker.

Select Outpost, Rune, Base, tech core, floor, arena or an existing `scenery-N`
entry, then select an exported placement. For assets without placements the
inspector uses the package's arena root. The asset selector scrolls horizontally.
The page scrolls vertically to mechanism controls and metadata.

- **Top / Front / Side** project world FLU onto xy / xz / yz respectively.
  The first axis points right and the second up. Zoom is centred on the fitted
  geometry; there is no free pan or perspective camera.
- **Fixed colliders** are teal, **moving colliders** amber, **RFID bands** purple.
  Detector origins are white crosses; frame axes are red +x (outward), green +y,
  blue +z. Grid spacing is one metre. These are diagnostic colors, not CAD paint.
- **Play preview** advances 16 inspection ticks per asynchronous pulse;
  **Pause** stops pulses. **Step 1 tick** pauses and adds 7.8125 ms.
  **Reset** pauses, clears time and all mechanism offsets, restores the configured
  robot pose/profile, and clears projectiles, held commands and contact history.
- Each channel slider restarts time at zero with its chosen offset and pauses.
  Source meshes on the same mechanism/team channel share that offset. Rotation
  offsets use radians; translation offsets are clamped fractions from 0 to 1.
  The displayed resolved coordinate includes elapsed inspection time. The preview
  law adds 0.5 rad/s to rotations and 0.1 fraction/s to translations, clamped at
  the end stop. These are illustrative inspection settings, not rulebook speeds.

Time is integer nanoseconds in a toolkit-independent model. A native timer merely
requests a fixed number of inspection ticks; slow frames do not catch up to wall
time. For Outpost, Base and Rune, the backend resolves `Scenario::targets_at` and
`sync_mechanisms` together before every `RobotSession::step_with_targets` at
128 Hz. Those sessions contain source geometry, targets, one optional physical robot and
its projectiles. The isolated preset catch floor is flat z=0, not the complete arena terrain. Other selections resolve prescribed geometry only. There are no
rewards, referee, HP, match clock or multiplayer input path. Reset and coordinate
edits create a fresh session from shared cooked geometry and the configured robot recipe.

## Optional source-material 3D diagnostic view

Choose **3D CPU orbit** in the native controls or pass `--view 3d`. This is a
CPU depth-rasterized orthographic diagnostic viewport, not a GPU/PBR game renderer.
Source GLB node hierarchy transforms are baked by the shared assets reader, then
the selected placement/root is applied once. Source-body names bind visual parts
to the same approved native motion channels as physics. Moving visual triangles
follow resolved coordinates on each frame, including explicit steps and reset.
Parsed, placed primitives and their motion membership are cached for the current
asset/placement; successful selection replaces the cache, while failed selection
preserves it. Wire planes remain the default and do not parse visual materials.

Yaw/elevation sliders orbit the fitted component; the existing zoom slider works
in both modes. CLI angles use radians through `--orbit-yaw` and
`--orbit-elevation`; `--zoom` accepts (0,20]. Fit includes physical robots,
projectiles and retained contacts, and optionally RFID bands. There is no pan or
perspective projection. The isolated physical catch floor remains flat z=0 and
is not drawn as invented source geometry.

The surface pass uses source linear base-color and emission factors, linear
flat directional/ambient lighting and sRGB output. It draws opaque, two-sided
triangles into a depth buffer. **Textures, alpha blending/masking, normal maps,
metallic/roughness, shadows and PBR are unsupported.** Source texture presence is
reported per picked primitive and counted in probe reports. This is factor-only
inspection, not source-render parity. Required unsupported GLB extensions cause
an explicit load error; a native failed view retains its previous image and
shows the error. No dependencies were added for this viewport.

Click the image to pick the nearest opaque source surface at that pixel centre.
The selection shows source node index, primitive index, inherited source-body
name, original material name (empty means unnamed/default), linear factors,
texture limitation, motion channel and world FLU position. The ownership and
position use the same depth test as the surface pixels. Cyan robot bounds/wheel
truth, yellow projectiles, pink contacts, detector axes and purple RFID remain
available as **x-ray diagnostic overlays**. **X-ray collision edges** adds fixed
teal/moving amber collider edges (`--collision-overlay`); their existing toggles
still control each set. Picking excludes all these overlays and selects the
underlying source surface. Hidden diagnostic lines therefore cannot steal a pick.

```sh
cargo run --release --locked --manifest-path integrations/scenario-inspector/Cargo.toml -- \
  --asset outpost --view 3d --orbit-yaw 0.7 --orbit-elevation 0.45 \
  --coordinate 0=0.7 --ticks 16 --pick 480,320 --viewport-bench 20 \
  --probe local-assets/scenario-inspector/captures/outpost-3d
```

`--pick X,Y` writes a source selection or `null` for background into the report.
`--viewport-bench N` measures 1–1000 warm viewport frames at the current state;
both flags require 3D and a probe destination to produce output. Reported minimum,
median and maximum times include vertex motion, camera fit, allocations, surface
depth rasterization and enabled overlays. They exclude source parsing, physics
stepping, PNG encoding, RXUI upload and presentation. Native controls also display
the latest viewport time. The backend is explicitly CPU; these are not GPU
measurements or whole-application frame rates. Use `--release` for interactive
inspection; unoptimized rasterization can be substantially slower. Dense arena
geometry can still be expensive, and rendering currently runs on the UI thread.

## Physical robot controls

Outpost, Base and Rune spawn one existing `ChassisConfig::default()` omni profile,
identity 0, on the isolated flat catch floor. Initial placement is two metres before
the selected component's minimum x envelope, centred in y. This is a physical
practice profile, not generated robot CAD. Spawn x/y/z (metres), yaw (radians) and
team label are editable in RXUI; each edit pauses and rebuilds at time zero. The
headless `--robot X,Y,Z,YAW_RAD` overrides initial body-centre placement;
`--no-robot` keeps a geometry/target-only session. Other component selections
remain geometry-only and reject robot commands.

Native sliders hold body forward/left velocities in m/s and yaw rate in rad/s,
and world gimbal yaw/elevation targets in radians. Actual gimbal motion follows
the physical motors; firing immediately after an aim change uses the actual
current muzzle. Play or step to let it turn. Four wheel torque sliders request
N m in the existing profile's wheel order, preserving motor, power and traction
limits. Torque control replaces velocity tracking. Editing velocity or **Brake /
velocity zero** restores velocity tracking; editing aim preserves torque mode.
Zero torque allows free rolling. Buttons fire one 17 mm shot at 25 m/s or one
42 mm shot at 12 m/s. The headless script can choose a speed in (0,40] m/s.
There is no firing cadence, heat, ammunition allowance, HP or game authority.

Cyan overlays show body/turret profile bounds, wheel hubs, physical wheel support
points (green) and the actual gun direction; these boxes do not reproduce the
full octagonal collision profile or CAD artwork. Yellow crosses are projectile
centres. Pink crosses are raw armor events. Robot, projectile, wheel and contact
positions are **privileged physics truth, not simulated sensors**. Camera fitting
includes robots, projectiles and retained contacts.

The report retains the latest 128 projectile/armor and chassis/armor events in
integration order, with tick-end timestamps in ns, target identity, projectile ID
when present, world/local contact position in metres and normal closing speed in
m/s. RXUI lists the latest eight. `contact_count` counts all such events since
reset. These are raw physics events, not scoring decisions or all rigid-body
contacts; wheel support is separate telemetry. Projectile scenery contact time
is also available in projectile truth. Event queues are drained each tick and
retained contact history is bounded. No sensor noise or observation policy is
implied by these overlays.

The toolkit-independent library accepts an explicit `RobotSpawn`, including an
existing validated `ChassisConfig`, via `Inspector::configure_robot`. A failed
configuration leaves the live episode intact. Reset retains this configured
recipe but clears runtime commands, projectiles and contacts. Selection changes
start the new component's default robot recipe. Mechanism coordinate edits also
restart the configured robot while keeping the chosen source coordinate offset.

## Headless previews and reports

The default build has no RXUI dependency enabled and needs no GPU/display:

```sh
cargo run --locked --manifest-path integrations/scenario-inspector/Cargo.toml -- \
  --asset outpost --coordinate 0=0.5 --plane xy \
  --probe local-assets/scenario-inspector/captures/outpost
cargo run --locked --manifest-path integrations/scenario-inspector/Cargo.toml -- \
  --asset rune --coordinate 0=0.5 --plane yz \
  --probe local-assets/scenario-inspector/captures/rune
cargo run --locked --manifest-path integrations/scenario-inspector/Cargo.toml -- \
  --asset base --placement 1 --coordinate 0=1 --plane xy \
  --probe local-assets/scenario-inspector/captures/base
```

Each probe writes a 960 × 640 `view.png` using the exact native image rasterizer,
and `report.json` with geometry counts, original paths, source STEP hash,
placement/root frame, source-body motion membership, resolved coordinates,
detector poses and RFID bands. `--ticks N` advances the explicit timeline before
capture. `--coordinate CHANNEL=VALUE` can repeat. `--help` lists all options.
The JSON is also printed to stdout. `--script PATH` executes an ordered JSON
action array before `--ticks N`. For example, save this array to caller scratch:

```json
[
  {"drive": [0.5, 0.0, 0.0]},
  {"step": 128},
  {"drive": [0.0, 0.0, 0.0]},
  {"aim_rad": [0.0, 0.15]},
  {"step": 32},
  {"fire": [17, 25.0]},
  {"step": 4}
]
```

```sh
cargo run --locked --manifest-path integrations/scenario-inspector/Cargo.toml -- \
  --asset outpost --script local-assets/robot-actions.json --plane xz \
  --probe local-assets/scenario-inspector/captures/robot-outpost
```

`drive` is `[forward_m_s,left_m_s,yaw_rate_rad_s]`, `aim_rad` is world
`[yaw,pitch]`, `torques_nm` is four wheel torques, `fire` is `[17|42,speed_m_s]`,
`step` is unsigned ticks and `reset` is `true`. Drive and torque requests persist.
Scripts validate operation names, shapes and finite numeric values before any
action runs; physical admission errors stop execution and retain earlier actions.
A script uses the same model methods as RXUI and reads no wall time. There is no
implicit stepping after a shot: add `step` to observe flight or contacts. Capture directories are caller-selected;
the examples use ignored scratch, never tracked assets.

## RFID boundary

The current checked package intentionally contains no RFID polygons. The live
server adds its diagram catalogue and fits heights to terrain. The inspector
reports zero footprints and an explicit unavailable message in that case; it
does not copy the server catalogue or infer antenna calibration from CAD.

Use `--rfid PATH` to import an explicit JSON array of
`rm_sim_assets::rmuc_2027::Zone` records exported by a caller. Each record
has `side`, `point`, `floor_m`, `height_m` and `polygon_m` in world FLU metres;
`point` is retained metadata and enables no rules. Import rejects nonfinite
geometry, fewer than three polygon vertices and nonpositive/overflowing height
bands. Polygons from the source bindings remain visible alongside imported ones.
Imported geometry is labelled unverified, and its path is recorded separately in
the report. **Fit field footprints** includes those polygons in the camera fit;
in component fit mode distant polygons are clipped. Front/side views show both
ends of their vertical band. Exporting a runtime catalogue remains the caller's
responsibility.

## Implementation boundary and limits

`src/model.rs` owns verified selection, placements, channel coordinates, detector
poses, optional RFID import, explicit time and the optional physical preset session. `src/raster.rs` maps
that data to pixels. `src/native.rs` owns RXUI controls, image handles and task
pacing; `src/main.rs` is the CLI/probe adapter. The library exposes `model` and
`raster` for robot/task adapters without RXUI. The initial physical preset bridge already
uses `Scenario::load`, `Scenario::session` and `Scenario::targets_at`. No dependency on server, world,
Bevy or a sibling repository was added. The assets crate retains its existing
transitive gameplay dependency.

Root placement uses `ComponentModel::placed`, `CadAssets::arena_pose` and the
existing glTF basis convention. Moving vertices use `MotionFrame::pose` and
shared quaternion helpers. Detector overlays use scenario target definitions for physical presets, with source `OutpostModel` and
Rune shaft helpers as a fallback, never independently fitted geometry. The inspector does not
change original source metadata or checksums.

Wire planes draw every original collision triangle edge. The optional 3D mode
uses `src/viewport.rs` for camera, source factors, opaque depth and picking;
`src/raster.rs` draws diagnostic overlays in that same camera. Both native images
and headless PNGs use this exact CPU path. The image is cached between changes;
changed native views allocate/upload a fresh image. No GPU framebuffer hook or
new simulation loop was introduced. See the 3D section for unsupported material
features and viewport timing scope.

## Validation

```sh
cargo fmt --manifest-path integrations/scenario-inspector/Cargo.toml -- --check
cargo test --locked --manifest-path integrations/scenario-inspector/Cargo.toml
cargo test --locked --manifest-path integrations/scenario-inspector/Cargo.toml \
  --features native -- --include-ignored
cargo clippy --locked --manifest-path integrations/scenario-inspector/Cargo.toml \
  --all-targets --features native -- -D warnings
```

The four ignored tests require the verified LFS package: one checks real source
motion, detector correspondence, end-stop clamping, atomic failed selection and
pixel-identical reset; another exercises physical robot commands, torque-preserving aim, actual muzzle firing, raw source detector contacts and robot episode reset; a third checks real Outpost/Base/Rune material previews, motion/cache/reset and independent nearest-ray/depth agreement; the native test prepares the RXUI view without a display and routes pointer clicks through Step and the 3D image picker. Set
`RM_INSPECTOR_TEST_ASSETS` for the model test's alternative package location.
Ordinary tests cover pause/pulse/step partitioning, overflow, clipping, axis
projection, script validation and a CAD-free physical drive/torque/fire/reset fixture. Native builds and these checks were exercised on Linux/Talos and
macOS. The macOS native window launched; system window capture returned
`could not create image from window`, so the saved PNGs are backend projections,
not screenshots of native window chrome. Manual native pointer/keyboard smoke testing remains outstanding. The 3D material/depth path is validated through headless tests and backend PNG probes, not a native window capture.

The robot-control expansion was checked with the default headless tests and all
ignored real-package tests on macOS, and with native tests plus warnings-denied
Clippy on an isolated Talos source/target. Native tests prepare spawn, drive, aim,
torque and fire controls and route a real pointer click through Step. Scripted
Outpost/Front, Rune/Side and Base/Top captures include the robot and a real fired
projectile after 162 explicit ticks (1.265625 s). Task evidence is under ignored
`local-assets/scenario-inspector/task1-evidence/`; source/check logs use the
`task1-` prefix in its parent directory. The macOS window server confirmed the
1120 × 949 native inspector window onscreen; its capture attempt returned
`could not create image from window`. These PNG probes therefore verify the
shared raster/model path, not native window chrome or manual robot-control input.

The CPU 3D expansion was checked on macOS/Apple M5 Max with Rust 1.99.0:
formatting, all 12 library tests including real LFS-package tests, the native
headless layout/pointer test and both doctests pass; native warnings-denied
Clippy passes. Independent ray/triangle intersections agree with the raster
surface depth for Outpost, Base and Rune. Tests cover draw-order independence,
background/degenerate/clipped triangles, camera round trips and zoom, default
white/emission factors, moving source geometry, reset and placement cache
invalidation. The native test also routes a real RXUI pointer event to a visible
source pixel and verifies source primitive selection.

Ignored scratch `local-assets/scenario-inspector/task4-evidence/` contains
`outpost-release`, `base-release`, `rune-release` and `outpost-overlay-release`
PNG/report pairs plus the robot action script. Reports include all requested
viewport parameters, source provenance, material limits and CPU timing scopes.
The Rune and Outpost overlay captures include a physical robot and one flying
projectile after 164 scripted ticks; Base uses 16 inspection ticks and channel
0 at 0.7. Warm `--release` measurements at 960 × 640 over 20 frames on that Mac
had medians of 9.9 ms for Outpost (21,406 triangles, robot disabled), 23.7 ms
for Base (29,836 triangles, robot overlay) and 17.0 ms for Rune (61,349
triangles, robot/projectile overlays, yaw −0.8 rad). Outpost with x-ray collision
edges and robot/projectile overlays measured 11.4 ms. These different fitted
scenes are diagnostic examples, not a comparative benchmark. Detailed min/max
values are retained in their JSON reports. Debug Outpost rasterization measured
roughly 0.5 seconds/frame. Logs use `task4-` in the parent scratch directory.
No native manual interaction or new native window screenshot is claimed.

## Optional dependency audit

The inspector has its own `deny.toml`; it does not inherit or widen the root
policy. Run the complete optional graph, including target-specific dependencies:

```sh
cargo deny --locked --all-features \
  --manifest-path integrations/scenario-inspector/Cargo.toml \
  --config integrations/scenario-inspector/deny.toml check
```

The policy retains the root permissive license allowlist, denies unknown
registries/git sources and wildcard dependency requirements, and warns about
multiple crate versions. Its only advisory ignore is the root's existing
maintenance-only `RUSTSEC-2026-0192` for `ttf-parser`; the root's unrelated `paste`
exception is not copied. Two version-specific Boost license exceptions cover the Windows clipboard backend. Source URLs are
explicitly restricted to RXUI, Astrelis and mid360-sim, and
`required-git-spec = "rev"` rejects unpinned git specifications. Actual revisions
remain fixed in the manifests/lockfile and are recorded beside the URL allowlist:

- RXUI: `656bf5ed460ed9604b65fa9538354fb6c746a7d4`.
- Astrelis and astrelis-winit: `604b9fedb62009dc9600963ab98a3cc00a31fe29`.
- mid360-sim: `c832a45bbde4d6cc5ef1e09afb0102b699fdc8d6`.

On 2026-10-23 HKT, cargo-deny 0.20.2 audited 399 resolved packages, all inspector
features and no target filter, using RustSec database commit
`640bf5a9f06e66d8721e8772766531fe77c8630d`. The audit used a private local Cargo
home under ignored inspector scratch and precomputed `cargo metadata --locked
--all-features` output pointing to the existing downloaded sources. Talos has no
cargo-deny installed, so no remote host packages or shared build caches were
changed. The locked dependency graph was unchanged by adding explicit version
requirements to the inspector's two path dependencies and pinned RXUI dependency.

**The initial full audit failed (exit 5) with these findings:**

- Licenses: `clipboard-win 5.4.1` and `error-code 3.4.0` require `BSL-1.0`, absent
  from the root allowlist. Both enter through RXUI → arboard 3.7.0's Windows
  clipboard backend. The inspector now allows this permissive Boost Software License for these
  exact versions only, consistent with the repository’s permissive dependency
  policy. The root policy remains unchanged. Restricting the
  audit to the current OS would hide this cross-platform requirement.
- Advisories: `zbus_names 4.3.5`
  ([RUSTSEC-2026-0371](https://rustsec.org/advisories/RUSTSEC-2026-0371)),
  `zvariant 5.15.1`
  ([RUSTSEC-2026-0370](https://rustsec.org/advisories/RUSTSEC-2026-0370)) and
  `zvariant_derive 5.15.1`
  ([RUSTSEC-2026-0372](https://rustsec.org/advisories/RUSTSEC-2026-0372)) are
  unmaintained following their migration into zbus 6. They enter through RXUI's
  zbus 5 / Linux AccessKit-AT-SPI dependencies. These are maintenance advisories,
  not reported vulnerability advisories; none was suppressed. A conservative
  resolution requires coordinated upstream RXUI/accessibility dependency
  migration, or a separately approved maintenance exception. A lockfile-only
  major-version replacement is not a compatible fix.

A follow-up license/source check on the same cross-platform metadata passes
after the two version-specific Boost allowances (`deny-license-followup.log`).
No maintenance advisory was suppressed.

Source checks pass. Ban checks pass with 23 duplicate-version warnings under the
root's warning policy. The two additional license warnings are unused root
allowlist entries (`CC0-1.0` and `MIT-0`). No vulnerability advisory was detected
in this audited graph/database snapshot. No package license expression contains
MPL-2.0, and no MPL license exception, new MPL notice or MPL source offer is
required for this inspector lockfile. Dependencies with alternative license
expressions remain satisfiable through their already-allowed permissive choice.
This finding does not change the root application's existing MPL obligations.

Evidence is retained under ignored `local-assets/scenario-inspector/`:
`audit-metadata.json`, `deny-all-features.jsonl` and `audit-summary.json` (including
lockfile SHA-256 and database revision). The full policy remains failing until the
listed maintenance advisories are resolved; it must not be described as a
passing release gate.

This cross-platform audit does not establish native runtime support. Linux was
build/layout-tested without an attached display; Windows was dependency-audited
but not launched. macOS launched a native window, while native pointer/keyboard
smoke testing and native window capture remain unverified. The shared backend PNG previews do not verify native chrome or manual input. The later CPU 3D tests separately verify source-factor rendering, depth occlusion and picking through the shared viewport path. These limits remain separate from the dependency-audit failures.
