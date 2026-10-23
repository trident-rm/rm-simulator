<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Standalone scenario viewer

The optional `integrations/scenario-viewer` workspace opens a small physical
scene: a falling cube beside a motor-driven, jointed arm on a fixed pedestal.
It uses the same `PhysicalRuntime` and generic Bevy renderer as other callers.
It loads no RoboMaster field, match, server, network transport or ROS environment.
Dimensions, masses and motor settings are illustrative fixture choices.

From the repository root:

```sh
mbx run --manifest-path integrations/scenario-viewer/Cargo.toml --locked
```

Drag with the left mouse button to orbit; scroll to zoom. **Space** pauses or
resumes, **N** advances one tick while paused, **R** resets, and **Esc** closes.
The overlay shows running/paused state and simulation time. Reset preserves the
current pause choice and returns the scene to its original placement.

## Repeatable checks and captures

```sh
# CPU-only scene check: no window, graphics device or rasterization.
mbx run --manifest-path integrations/scenario-viewer/Cargo.toml --locked -- \
  --headless --frames 120 --benchmark

# Actual GPU-rendered PNG, with no window. Parent directory must exist.
mbx run --manifest-path integrations/scenario-viewer/Cargo.toml --locked -- \
  --screenshot /tmp/scenario-viewer.png --frames 90

# Interactive paused scene; close automatically after 120 presentations.
mbx run --manifest-path integrations/scenario-viewer/Cargo.toml --locked -- \
  --paused --frames 120

mbx test --manifest-path integrations/scenario-viewer/Cargo.toml --locked
python3 integrations/scenario-viewer/check-dependencies.py
```

Without `mbx`, substitute `cargo` (shared compilation caching will be unavailable).
The workspace has its own lockfile and target directory. When entering the root
Nix shell, run `cd integrations/scenario-viewer` and set
`CARGO_TARGET_DIR="$PWD/target/nix"` before compiling, so the shell's root target
does not mix independent workspace builds. Keep the default shared mbx cache;
no extra compiler wrapper or shared target is needed.

Ordinary interactive playback accumulates Bevy's frame delta and advances explicit
128 Hz ticks. Each update accepts at most 250 ms of elapsed time and performs at
most 32 ticks, so stalls cannot create unbounded catch-up. Pause, manual step
and reset clear the accumulator; a manual step advances one tick (7.8125 ms).

`--frames` counts presentations. Explicit frame runs, `--headless`, `--benchmark`
and `--screenshot` use a repeatable four ticks per presentation (31.25 ms) unless
paused, independently of rendering speed. In screenshot mode, frames
also provide warmup (at least 30 presentations) before the GPU readback. Physics
freezes after the requested count while loading or capture finishes. Finite
windowed runs also freeze at that count and wait for the GLB and all recursive
dependencies to load before reporting success. Every graphical mode fails on
source/dependency errors or a 120-second loading timeout. The
capture exits after saving, or fails after a 120-second timeout. Shader pipelines
compile synchronously for captures; the PNG is checked for a blank black frame.

`--benchmark` defaults to 120 presentations and exits (override with `--frames`).
It reports CPU time spent stepping and publishing poses separately.
The headless run additionally reports total app-update wall time, which includes
both phases. GPU rendering is asynchronous; these numbers are **not GPU timings**
or a graphics performance claim. Screenshot mode reports capture/readback/save
latency separately. `--headless` never captures an image and rejects GLB and
screenshot options.

Finite runs and GLB-loading modes keep the render app on the main thread so a
first-update failure can shut down cleanly before the first render. Ordinary
interactive viewing of the built-in scene keeps Bevy's pipelined renderer.
Benchmark runs therefore measure their explicit validation scheduling; they do
not establish interactive rendering throughput.

## Optional source visual

```sh
mbx run --manifest-path integrations/scenario-viewer/Cargo.toml --locked -- \
  --glb /absolute/path/to/model.glb
```

The first glTF scene is attached visually at the arm's centre. Its authored
materials, local axes and metre scale are preserved. This is a visual attachment;
the illustrated arm's box remains its collision shape and inertia. It does not
infer collision geometry or remodel a mechanism from the file. Use a self-contained
GLB with its origin and orientation prepared for this attachment.

## Ownership and validation

The viewer imports `rm_sim_render::generic` with renderer default features
disabled. It publishes `PoseFlu` under its own stable `VisualId`s, without an RM
`SceneState`. One unit-cube mesh is shared by every box; the pedestal and floor
also share a material. GLB instances use the shared source asset handle. Physics
and rendering share no live handles: body keys are rebound after reconstruction,
while visual entities, meshes, materials and scales survive reset.

Prepared `CookedRigidBody` resources are retained once and installed into a fresh
`PhysicalRuntime` on reset. The caller owns the tick and motor commands; the
renderer never steps physics. The arm is dynamic, constrained by a revolute
joint about vertical, with a bounded position motor; its displayed transform is
read from the actual body. See [Physical runtime](physical-runtime.md) and the
[`body_runtime.rs`](../crates/rm-sim-physics/examples/body_runtime.rs) and
[`jointed_turret.rs`](../crates/rm-sim-physics/examples/jointed_turret.rs)
examples for the underlying contracts.

Tests cover pause, manual step, motor motion, cube settling, stale bindings after
reset, deterministic replay, actual pose publication, visual handle reuse,
finite-run loading gates, dependency failure and timeout.
The dependency check rejects gameplay/world/server/presentation/assets/ROS crates,
the renderer's `robomaster` feature, and incompatible Bevy/wgpu versions. It uses
the committed lockfile. This workspace intentionally remains outside the ordinary
workspace and benchmark dependency graph.
