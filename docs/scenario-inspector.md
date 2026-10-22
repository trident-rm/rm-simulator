<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Optional RXUI component inspector

`integrations/scenario-inspector/` is an independent Cargo workspace for inspecting
verified equipment geometry. It does not change the interactive Bevy application,
normal workspace dependencies, networking or match authority. It currently
consumes `rm_simulator_assets::{components, scenarios}` and the shared
physics motion/pose helpers. Outpost, Base and Rune selections create an empty
`RobotSession` from the new reusable preset API. It requires the component composition changes in
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
  **Reset** pauses, clears time and all mechanism offsets.
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
128 Hz. Those sessions contain source geometry and targets, with no robots or
projectiles. Other selections resolve prescribed geometry only. There are no
rewards, referee, HP, match clock or multiplayer input path. Reset and coordinate
edits create a fresh empty session from the shared cooked scenario geometry.

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
The JSON is also printed to stdout. Capture directories are caller-selected;
the examples use ignored scratch, never tracked assets.

## RFID boundary

The current checked package intentionally contains no RFID polygons. The live
server adds its diagram catalogue and fits heights to terrain. The inspector
reports zero footprints and an explicit unavailable message in that case; it
does not copy the server catalogue or infer antenna calibration from CAD.

Use `--rfid PATH` to import an explicit JSON array of
`rm_simulator_assets::rmuc_2027::Zone` records exported by a caller. Each record
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
`raster` for future robot or task adapters. The initial physical preset bridge already
uses `Scenario::load`, `Scenario::session` and `Scenario::targets_at`. No dependency on server, world,
Bevy or a sibling repository was added. The assets crate retains its existing
transitive gameplay dependency.

Root placement uses `ComponentModel::placed`, `CadAssets::arena_pose` and the
existing glTF basis convention. Moving vertices use `MotionFrame::pose` and
shared quaternion helpers. Detector overlays use scenario target definitions for physical presets, with source `OutpostModel` and
Rune shaft helpers as a fallback, never independently fitted geometry. The inspector does not
change original source metadata or checksums.

This is an orthographic collider wire view, with every original selected triangle
edge drawn. It does not display source visual materials/textures, hide back faces,
provide depth occlusion, simplify meshes, add picking, or spawn/configure robots and projectiles. Camera fit follows the current pose. Rune faces can overlap
in a top view; the side view is more useful. Dense geometry and software
rasterization limit interactive speed on large scenery. The 960 × 640 image is
cached between changes; component triangle data and the preset collision recipe
are currently loaded separately on selection (a possible future optimization); new pixel handles/uploads are created on changed views.
A source-material 3D viewer would need an application-owned rendering pass, which
RXUI supports through its framebuffer/graphics hooks, rather than a new match or
physics loop in the toolkit. No full 3D embedding was attempted here.

## Validation

```sh
cargo fmt --manifest-path integrations/scenario-inspector/Cargo.toml -- --check
cargo test --locked --manifest-path integrations/scenario-inspector/Cargo.toml
cargo test --locked --manifest-path integrations/scenario-inspector/Cargo.toml \
  --features native -- --include-ignored
cargo clippy --locked --manifest-path integrations/scenario-inspector/Cargo.toml \
  --all-targets --features native -- -D warnings
```

The two ignored tests require the verified LFS package: one checks real source
motion, detector correspondence, end-stop clamping, atomic failed selection and
pixel-identical reset; the other prepares the RXUI view without a display and
routes a pointer click through the actual Step button. Set
`RM_INSPECTOR_TEST_ASSETS` for the model test's alternative package location.
Ordinary tests cover pause/pulse/step partitioning, overflow, clipping and axis
projection. Native builds and these checks were exercised on Linux/Talos and
macOS. The macOS native window launched; system window capture returned
`could not create image from window`, so the saved PNGs are backend projections,
not screenshots of native window chrome. Manual native pointer/keyboard smoke
testing and full 3D/material verification remain outstanding.

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
smoke testing and native window capture remain unverified. The shared backend
PNG previews do not verify native chrome, source material rendering or 3D
occlusion. These limits remain separate from the dependency-audit failures.
