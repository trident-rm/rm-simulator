<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Performance optimization priorities, 2026-10-23

This pass follows the [graphics measurements](performance-graphics-20261023.md).
Base revision is `a721350`; the working tree also contains unrelated RMUL work.
The renderer, native gameplay, complete physical Field and multiplayer host are
separate workloads. Improvements to one do not establish a speedup in another.

## What the existing evidence says

The [2026-10-21 release baseline](performance-20261021.md) measured eight-robot
rules at 32.6 ms per seven-minute round with 128 Hz observations, versus 18.1 ms
per sixteen ticks for an eight-robot physical match over the loaded CAD field.
Full physical stepping is therefore the first throughput target for physical PPO
rollouts. Those probes were different scenarios; their ratio is not a measured
fraction of a single frame's CPU time.

That physical probe measured snapshot capture at about 4 microseconds and restore
at about 1.5 milliseconds. Copy reduction should target repeated/deep copies and
observation frequency, rather than assume every checkpoint is a bottleneck.
A reset once per long episode matters differently from reconstructing a predicted
world on every frame. Immutable geometry is already shared in checkpoints.

The later frozen renderer measured approximately 2.9 ms per frame at High and
2.3 ms with Low/no-bloom at 720p. Its full-app probes were affected by severe
external CPU contention. They cannot rank simulation against GPU costs.

## First optimization wave

Independent audits cover native gameplay/Field observation and stepping,
renderer/app synchronization, and server/sensor data flow. Favor removal of
unnecessary allocation and repeated computation without changing rule timing,
physics integration, collision fidelity or presentation semantics. Each change
needs scoped correctness checks and operation/allocation evidence; wall-clock
speedups require isolated measurements after compilation finishes.

The wheel solver has a concrete recurring allocation: each grounded chassis
builds contact, motor and traction vectors on every Rapier slice. At default
128 Hz with eight slices per tick, that is 3,072 vector allocations per robot per
simulated second. Reusing those three buffers removes their steady-state
allocation after warmup, while preserving the existing eight motor-integration
slices, force order and power limits. This count follows from the call structure;
it is not a measured whole-match speedup. A regression compares fresh and reused
storage across changing contacts, direct torque inputs and zero power budgets.
All 31 chassis tests passed in the release profile, including that regression.
Physics all-target release Clippy passed with warnings denied. The initial debug
test request was intentionally stopped while waiting behind the Bevy build; it
was replaced by the completed release-profile check.

The [gameplay/rollout report](performance-gameplay-20261023.md) records the first
completed audit: two deep roster copies per tick and temporary observation
collections were removed; terrain contacts and adapter scratch retain storage.
Before/after rollout comparison hashes matched and 200 scoped tests passed.
Rules-only timings improved in those samples, but physical stepping deteriorated
alongside unchanged checkpoint/reset timings under changing contention. These
measurements establish no causal physical speedup.

The [server/sensor report](performance-server-sensors-20261023.md) records the
second completed audit: owner anchors borrow chassis storage and reuse the
handshake configuration hash; LiDAR capture reads compact chassis state. Forty
scoped correctness tests and scoped Clippy passed. The owner microbenchmark
changed from 4.93 to 3.13 microseconds per anchor, but system load exceeded 90;
this is indicative only. Full upstream publication construction and sensor
fixed-geometry capture remain follow-up profiling targets.

The [renderer report](performance-render-20261023.md) records the third completed
audit: borrowed chassis-build comparisons remove fourteen geometry Vec clones
and 8,288 copied bytes per frame in its seven-record fixture; retained lookup
sets avoid rebuilding collections. Tracked light helpers prevent unchanged
materials from being marked dirty, and fixed/wheel CAD links skip gimbal angle
calculations. Thirty-nine renderer unit tests and seven doctests passed, as did
scoped Clippy. These CPU-side operation reductions are established; a full-app
frame-time or GPU speedup remains unmeasured.

## Follow-up order

First obtain an uncontended loaded physical rollout profile and a same-scene app
CPU profile. Keep the current structural fixes, but do not use their microprobe
ratios to predict total frame rate. Next investigate app scene-vector reuse and
compact observations for PPO; owner-only publication construction and fixed
sensor captures are the next copy targets. Benchmark bounded independent CPU
environments before selecting a training concurrency limit. GPU sensor batching
and shared procedural rendering assets follow measurements of those workloads.
A final integrated workspace check should follow the concurrent RMUL integration;
scoped validation here is not a claim that the complete workspace gate has run.

## PPO execution choices

Strategy tasks can use the standalone gameplay authority and caller-provided
observations without CAD, rendering or networking. They still need an explicit
travel/combat abstraction if those dynamics matter; the rules engine supplies no
substitute automatically. Physical tasks should use caller-owned world sessions,
share immutable map geometry, retain the world between actions and advance several
physics ticks for each policy action. Action and sensor rates need not equal the
integration rate. Privileged truth must remain distinct from simulated sensors.

Before adding a training integration, measure environments per second, physics
ticks per second, actions/observations per second, reset time and memory per
environment. Compare independent CPU workers sharing geometry at several worker
counts. Do not equate simulated-time acceleration with PPO training throughput.
Policy inference and optimization are outside current simulator measurements.

## GPU candidates and limits

The most promising experimental targets are batched LiDAR rays over persistent
scene geometry and camera observations consumed on the GPU. Keep acceleration
structures and output buffers resident, update only moving poses, and benchmark
complete acquisition including any required CPU return path. Compare against the
existing motion-aware CPU acquisition, preserving per-return timing, self filtering
and motion interpolation. No new GPU sensor backend is implemented by this pass.

Rendering is already GPU-backed. Check mesh/material handle reuse, draw counts,
visibility and CPU extraction before adding custom instancing:
[Bevy automatically instances compatible shared mesh/material handles](https://bevy.org/examples/shaders/custom-shader-instancing/).
A new custom draw path needs evidence that existing batching is insufficient.

A GPU physics/rules rewrite is a separate backend project, with collision,
precision, determinism and restore consequences. First measure CPU environment
parallelism and remove host-side allocation. Small individual operations are poor
initial offload candidates when each result is immediately needed by CPU physics:
[wgpu mapping waits for prior GPU use before CPU access](https://docs.rs/wgpu/30.0.0/wgpu/struct.Buffer.html).
Even shared-memory hardware still needs synchronization. GPU acceleration must be
judged end to end, including submission, updates, readback and consumer latency.
