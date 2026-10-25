<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Presentation and rendering boundaries

The game, physical runtime and renderer have separate responsibilities. Rendering
consumes resolved appearance and never advances a match or queries a live solver.
The split adds no new clock, networking path or simulation world.

## Ownership

- `sim-engine` describes physical models and scalar contracts.
- `rm-sim-physics` owns physical preparation/runtime and passive sensors.
- `rm-sim-gameplay` owns deterministic competition authorities.
- `rm-sim-world` composes physical participants with those authorities.
- `rm-sim-presentation` translates borrowed RoboMaster snapshots, caller
  presentation time and detected hits into renderer appearances. It depends on
  world and render, with no host, transport, input, window or session dependency.
- `rm-sim-render::generic` owns FLU rendering conversions, camera appearance
  and shared-handle mesh/imported-scene instances with sparse pose updates.
- `rm-sim-render::rm` exposes the RoboMaster visual extension: armor,
  equipment lights, robot visuals and resolved match-scene synchronization.
- The app owns networking, prediction/interpolation selection, presentation
  scheduling, cameras, controls, HUD and menus. It selects actual presented robot
  state and hands resolved appearance to the passive renderer.

The renderer's default `robomaster` feature retains the existing public paths.
General consumers disable default features and use `generic`; the standalone
[scenario viewer](scenario-viewer.md) exercises that build without gameplay,
world, server or the app. These remain modules in one rendering crate rather than
separate repositories or a new backend abstraction.

## Hot-path contract

Static meshes, materials and imported assets are prepared once and retained by
Bevy handles. Spawning another visual shares those handles; it does not clone mesh
buffers. Stable caller visual IDs resolve to live Bevy entities. Sparse update
storage retains capacity and queued work targets the installed entity generation,
so removing/recreating an ID cannot apply an old update to a new visual.

Liveness pruning scans registered roots to remove externally despawned entities;
sparse synchronization costs O(roots + queued updates). The game does not install
this optional registry on top of its existing match-scene systems.

Pose and visibility updates compare values before mutable component access.
Unchanged inputs must not dirty `Transform`/`Visibility`, rebuild entities, swap
materials or trigger mesh uploads. Generic pose updates do not require a complete
scene snapshot. Lifecycle operations and asset changes may allocate; steady
publication is a separate measurement from loading and cooking.

The RoboMaster adapter fills caller-retained scene buffers. Live visual
projectiles and interpolated chassis are filled directly rather than first
building snapshot-derived vectors that are discarded. This does not imply every
nested robot geometry/string allocation has been eliminated. Presentation data
still includes model information; separating immutable model revisions from
changing poses is a possible subsequent optimization with explicit invalidation.

## Verification and performance interpretation

Headless tests cover scene output, time/flash behavior, visual lifecycle, handle
sharing and conditional ECS writes. The standalone viewer additionally exercises
physical recipe reset and reacquires live bindings while retaining visual assets.
Existing game scene tests and the ordinary full workspace gate remain required.

CPU adapter and synchronization probes are measured separately from end-to-end
rendering. The fixed-camera benchmark retains the same field assets, camera,
resolution, geometry, lighting and graphics configuration across builds. On Metal
its wall-frame duration includes CPU scheduling and GPU backpressure; it is not
an isolated GPU timestamp. Capture/readback is outside the sampled interval.
A screenshot checks visual output, not performance or sensor calibration.

No extra render pass, geometry approximation or quality reduction is part of this
split. Any speedup claim requires paired evidence; a new module boundary alone
establishes neither faster rendering nor lower GPU cost.

## Measured extraction checks (2026-10-23)

On Apple M5 Max, default rendering passes 43 unit tests and eight doctests;
generic-only rendering passes eight unit tests and four doctests. Both feature
modes pass all-target Clippy. The presentation adapter preserves original output
fixtures for quiet scenes, detected-hit flashes and native RMUC target/HP state.
Exact float fixture/output hashes are platform-specific; comparisons use the
original functions on the same platform rather than cross-platform identity.

The 16-chassis/32-projectile adapter probe eliminates the discarded snapshot
projectile vector and its 32 appearance writes per frame. Outer scene vectors
retain capacity; nested chassis vectors and source-joint strings still allocate
as before. Sequential developer-profile timings (~34–37 µs original, ~21–22 µs
retained) were collected under changing load, without allocator instrumentation;
these do not establish a causal speedup.

A subsequent same-process Linux comparison alternated the frozen original
adapter with the extracted adapter for five trials of 2,048 frames per fixture.
It exposed a real cross-crate cost: roughly 24–25 µs original versus 37–38 µs
extracted. A normal `#[inline]` hint on the hot chassis-appearance function
restored caller optimization without changing arithmetic: the extracted path
then measured roughly 25–26 µs versus 24–25 µs original. About 1 µs/frame remains
in this 16-robot fixture; no claim of zero boundary overhead is made. Original
output goldens are retained on both measured platforms.

A separate 1,024-visual CPU fixture measured roughly 49–55 µs/frame for direct
guarded ECS updates and 66 µs/frame for the generic sparse registry, including
publication, application scheduling and liveness pruning. Four-visual runs were
roughly 21–29 µs/frame for either path. These noisy fixture costs are not GPU
measurements and the registry is not an extra game-frame stage. Retained buffer
capacity and unchanged component ticks are tested directly.

The unchanged FLU conversion function was also compared using identical callers
and preserved binaries in alternating runs: median 12.188 ms original and
12.636 ms extracted per 4,194,304 poses, with overlapping ranges. Both functions
have 51 identical instructions apart from constant addresses; loaded SIMD
constants are byte-identical. An initial 2× difference did not reproduce.

The exact `b00cdcd` fixed-camera baseline and refactored renderer produced
pixel-identical 1280×720 images in all four comparison runs. The same approved
field package, Low/no-bloom settings, authored geometry, camera and three-second
warmup/five-second sampling were used. Alternating baseline/final/final/baseline
wall-frame medians were 8.183/8.503/9.510/8.291 ms. System load averages were
roughly 23–31 on 18 logical CPUs, with unrelated work active. This result is
inconclusive for throughput; Metal GPU timestamps are unavailable. Geometry,
render passes, quality settings and visual output were preserved.
