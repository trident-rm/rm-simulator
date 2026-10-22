<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Render synchronization audit: 2026-10-23

Scope: renderer changes over `a721350` with preexisting, uncommitted RMUL
work preserved. No graphics settings, rules, physics ticks, geometry, source
paint, or rendering backend change. No app files changed.

## Changes and evidence

- `chassis.rs`: compare borrowed `ChassisBuild` fields only for entities with
  build records. Previously the stale scan constructed an owned build for every
  `ChassisId`, including wheels and turret stages without build records. Only
  actual spawning now clones the body outline and bumper prisms. Retain the
  stale/existing ID sets between updates instead of reconstructing them.
- `sync.rs`: preserve Bevy's `Mut` wrapper through light helpers. Previously the
  conversion to plain `&mut MeshMaterial3d` dirtied each base, outpost and Rune
  light before its equality guard executed. Helpers now compare before mutable
  dereferencing. The headless regression counts changed components, including
  flash transitions and late-spawned lights. The ordinary public `ArmorState::apply`
  API remains available; internal ECS callers use the tracked helper.
- `targets.rs`: retain the source detector ID set between frames. Initial
  population or a larger scene can still allocate; clearing preserves capacity.
- `team_robot.rs`: filter supported chassis through a borrowed iterator instead
  of collecting a temporary `Vec<&ChassisAppearance>`. Resolve yaw only for yaw
  links and pitch only for pitch links. Wheel/fixed/fold links avoid both
  gimbal angle calculations (two quaternion inversions, two products, two vector
  rotations, one `atan2` and one `asin` per link at the source level).

The manual `borrowed_build_scan_probe` uses seven records (body, four wheels,
yaw and pitch), a four-vertex outline and twelve bumper prisms. The old source
requests fourteen nonempty Vec clones and copies 8,288 bytes per frame; the new
scan requests zero geometry clones and compares geometry once. These counts
follow directly from the fixture and code, rather than a process-wide allocator
hook; optimizer behavior and other engine allocations are outside the count.
For N chassis with this fixture multiply by N. Real robot geometry varies.
Repeated sets retain capacity after warmup; the CAD spawn iterator needs no
collection allocation. The target-light fixture changes three material
components every unchanged frame with the old helper path and zero with the
tracked path; a flash changes all three once, and a late light initializes once.

## Validation and timing

Built with `nix develop -c cargo test -p rm-simulator-render --locked --no-run
-j 2`, then ran the built test binary directly under `nix develop` with one test
thread: 39 passed, one manual probe ignored. The probe was then run separately
with `borrowed_build_scan_probe --ignored --nocapture --test-threads=1`.
For 100,000 fixture frames its old scan took 63.318916 ms and the borrowed scan
took 8.882333 ms (0.633 versus 0.089 microseconds per fixture frame; about
0.544 microseconds saved). This is one development-profile CPU microprobe,
not a release result or an end-to-end frame-time claim. Build concurrency was
limited to two Cargo jobs. Unrelated active builds and load averages around
20–23 on 18 logical CPUs limit timing interpretation. Structural copy counts
and changed-component assertions are stronger evidence than this timing.

The ordinary `nix develop -c cargo test -p rm-simulator-render --locked -j 2`
also passed: 39 unit tests and seven doctests, with the manual probe ignored.
`nix develop -c cargo clippy -p rm-simulator-render --all-targets --locked -j 2
-- -D warnings` passed. `git diff --check`, scoped rustfmt checks and
`scripts/check-module-dependencies.py` passed. Cargo operations waited for the
shared build lock rather than using a separate competing build directory.
No GUI or GPU benchmark was run for this CPU-only change.

## Remaining opportunities, in priority order

1. Reuse world-to-render buffers in `app::scene::publish_scene` and
   `chassis_appearance`. Each frame reconstructs nested wheel/armor/source-joint
   and geometry vectors; source joint names are cloned. Separate immutable robot
   description data from changing poses only with explicit ownership and model
   invalidation. `publish_scene` also builds snapshot projectiles in
   `scene_state_with_hits` before replacing them with visual projectiles.
2. Reduce hidden or unchanged HUD string formatting. Text assignment already
   compares strings, but `update_hud` formats them before that comparison; a few
   node/style assignments still mutate unconditionally. Preserve presentation
   time, input hints, RMUL data and local reservation changes when invalidating.
3. Profile prediction request construction: the full checkpoint is already
   cached behind an Arc per snapshot and the worker retains its Replayer.
   `Session::update_prediction` still clones the full chassis list and pending input list
   for requests, and shot history clones owner chassis during replay. Audit with
   the authority/anchor/context proof rules before changing those ownerships.
   Do not skip ticks or replace the whole-world prediction path.
4. CAD link lookup scans chassis IDs and source joint names linearly per link.
   A persistent lookup may help larger rosters; first count links/robots in a
   quiet, same-scene profile. Current material and transform guards already
   avoid many writes, and projectile visuals already use a shared mesh pool.

## GPU applicability

These changes reduce CPU allocations, copies and ECS change notifications;
there is no GPU compute offload. The renderer consumes caller-owned poses and
should remain passive. Match authority, Rapier stepping and prediction restore
semantics are unsuitable targets for a cheap GPU substitution.

CAD meshes use ordinary Bevy Mesh3d/material entities; this audit found no custom
instancing or culling bypass. `cad::loaded` caches replacement material handles
within a loaded instance, and projectile visuals share one mesh/material.
Procedural robot spawning creates many independent mesh/material assets, even
for repeated boxes, fasteners and rollers. Sharing identical assets across
robots could improve batching eligibility and memory use, while retaining
geometry, team paint and per-detector light identities. Source CAD material
replacements are scoped per instance; broader sharing must preserve independently
animated optical materials. Do not promise draw-call reductions without measuring
the actual Bevy batches and shadow passes.

Rank GPU investigations after a quiet full-app CPU profile: measure draw calls,
shadow-pass counts, visible mesh counts and per-pass timings on a supported
backend; then consider shared procedural assets and equivalent material-handle
reuse. Avoid disabling culling, merging independently articulated groups or
reducing CAD detail without visibility/fidelity evidence. The earlier 720p
static-scene results (High about 2.9 ms, Low without bloom about 2.3 ms, Ultra
about 4.2 ms) remain contextual evidence only; this CPU audit neither reruns them
nor attributes its gains to GPU duration. Metal timestamps were unavailable in
that benchmark. See `performance-graphics-20261023.md` for the measured limits.
