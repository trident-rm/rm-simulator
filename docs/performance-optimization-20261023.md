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

## Release headless profile after the runtime extraction

A second measurement used revision `881f71e`, release optimization with
`CARGO_PROFILE_RELEASE_DEBUG=1`, on a Ryzen 9 9950X3D (32 logical CPUs, Linux).
There was no renderer, transport, wall-clock pacing or policy inference.
Existing `match_cost`, `rollout_cost` and `mounted_lidar_cost` examples supplied
workloads. The latter now accepts an optional 1–60 second acquisition duration
so sampling need not be dominated by loading a one-second scene.

The checked RMUC arena contains 187,661 ground, 94,274 fixture and 165,950
additional equipment collision triangles: 447,885 placed triangles in total.
The match probe used eight default omni robots, native rules, held drive/yaw,
17 mm firing once per robot per simulated second, explicit benchmark ammunition,
a 500-tick post-countdown warmup and 256 batches of 16 ticks. Each run advances
32 simulated seconds per match. These are default physical robots, not all
source-CAD chassis or a manipulation workload.

Across three separate unprofiled runs:

- One eight-robot world: 8.58–11.84 seconds wall time, or **346–477 world ticks/s**
  and 2.70–3.73 simulated seconds per wall second. Median 16-tick stepping cost
  was 27.86–43.80 ms; p95 was 69.77–84.94 ms. These batch statistics are not
  individual-tick latency percentiles.
- Four concurrent eight-robot worlds: 9.34–11.65 seconds wall time for the same
  32 seconds in each world, or **1,406–1,753 aggregate world ticks/s**. This is
  roughly 3.7–4.1 times paired single-world aggregate throughput. Some concurrent
  batch tails exceed the 125 ms real-time budget. These scoped-worker numbers
  do not establish a maximum environment count or PPO training speed.
- A single-world checkpoint capture had 13–23 µs median cost; restore had
  2.05–3.54 ms median cost. Restore is measured on a temporary Field, not applied
  to the retained rollout. Copy removal remains useful, but checkpoint capture
  is small relative to physical stepping in this fixture. This does not measure
  large image observations, source-backed robot checkpoints or allocation bytes.
- One 200,000-emission rolling MID360 acquisition interval: 311–347 ms for ray
  acquisition, 0.204–0.340 ms for capture and 62.6–70.8 ms for physics. The longer
  ten-second sampled probe produced 2,000,000 emissions in 3.066 s of acquisition,
  2.078 ms capture and 477.6 ms physics. The chassis is commanded to drive but
  settles against geometry near its initial location; this is a throughput
  fixture, not a high-velocity distortion or navigation test.

The first single-world repetition was slower than the next two. Background agent
builds were allowed to run, and no CPU affinity/frequency isolation was imposed;
retain the full range rather than attributing that variation to a code change.
The flat `rollout_cost` probe reported about 101 µs/tick for one robot and
833 µs/tick for eight. It has different geometry and no firing, so it is not a
controlled estimate of terrain overhead. Its three snapshot hashes still match
our established compatibility fixtures.

Linux `perf` sampled a separate 1,024-batch match run at 499 Hz with user-cycle
samples and DWARF stack capture. Self-symbol shares included approximately:
point-cloud AABB calculation 11.76%, narrow-phase pair update 10.52%, contact
constraint solve 9.33%, ray/AABB tests 8.36%, contact generation 5.06%, and
constraint updates 4.27%. These are whole-process shares, including setup,
warmup, firing, checkpoint and restore; they are not exclusive timing of
`Field::step`. Stack unwinding was incomplete for some hot optimized frames,
so a particular caller of the AABB hotspot has not been established. The match
profile retained roughly 15,000 samples without reported lost samples.

The initial one-second LiDAR process profile included substantial terrain/BVH
construction. The ten-second follow-up reduced that startup bias: ray/AABB tests
accounted for approximately 46.2% of whole-process self samples. Neither percentage
should be presented as a steady-state BVH rebuild cost. Sampling required a
privileged perf process on this host; the workload itself ran as the ordinary
user, and no system profiling settings were changed.

The measured next priorities are collision/query and contact work: attribute
repeated bounding calculations, inspect the number and shape of active contact
pairs, and distinguish wheel-support queries from projectile/CCD work before
changing shapes or substep policy. Preserve collision fidelity and current wheel
integration during any cache or filtering optimization. For dense sensor batches,
measure persistent batched ray acceleration separately. These results do not
justify a GPU rules rewrite or claim that replacing Rapier improves throughput.

Reproduce the unprofiled workload in the normal native development environment:

```sh
CARGO_PROFILE_RELEASE_DEBUG=1 cargo build --release --locked \
  -p rm-simulator-server --example match_cost --example mounted_lidar_cost \
  -p rm-simulator-world --example rollout_cost
target/release/examples/rollout_cost
target/release/examples/match_cost "$CAD_PATH" 1 8 256
target/release/examples/match_cost "$CAD_PATH" 4 8 256
target/release/examples/mounted_lidar_cost "$CAD_PATH" 10
```

Use the actual `CARGO_TARGET_DIR` when it differs from `target`. Repeated probes
must retain the same package, input sequence, profile, sampling cadence and native
library environment. Sampling can use `perf record -e cycles:u -F 499
--call-graph dwarf,8192` around the executable; inspect self samples with
`perf report --stdio --no-children`. Archive profiler overhead measurements
separately from the three unprofiled timing repetitions. Raw timing/profiling
reports, binaries, commands and `summary.json` remain ignored scratch under
`local-assets/headless-profile-20261023/` and the corresponding remote probe
directory. No integration-rate, rule, collision-fidelity or solver change was
made by this profiling pass.
