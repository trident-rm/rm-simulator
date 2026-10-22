<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Second physics backend evaluation — 2026-10-23

Evaluation base: `f9c5b02f30c1a102d8098d91b2efb1ec59f03ff5`.
This report changes no runtime behavior or workspace dependency. Experiments are
ignored scratch under `local-assets/physics-backend-eval/`; corresponding Talos
scratch is `/home/hxyulin/.cache/physics-backend-eval-20261023/`. Neither the
parent's source snapshots nor `/home/hxyulin/.cache/rm-simulator-arena/target`
were used or changed. No package, driver or system configuration was changed.

## Recommendation

Keep Rapier as the game/prediction backend. The existing `wheel_model` is a useful
boundary: a second solver ran its **identical Rust equations** successfully,
without a generic engine trait or core dependency change. This small experiment
provides no performance reason to migrate the interactive application.

For a maintained PhysX experiment, investigate NVIDIA's **ovphysx C API / Python
package**, with a thin optional adapter and coarse buffers. GPU execution works
on Talos today. Do not select the archived Embark Rust wrapper as the production
integration: it is useful for this CPU comparison, but does not provide the
current GPU SDK or safe scene-query coverage. An in-process Rust C adapter would
need an explicitly reviewed FFI safety boundary; this evaluation introduces none.
A separate process/workspace remains reasonable while that policy is unresolved.

**A concrete GPU query blocker was reproduced:** after a box falls, DirectGPU
position reads show the settled box while the deprecated raycast still hits its
initial pose (details below). GPU ray-wheel throughput is **unmeasured**. The modern package smoke test runs
boxes without wheels; its numbers cannot be compared as equivalent rollout
throughput. Establish a maintained query path, moving-body sensor consistency,
CCD/contact coverage and distribution terms before a GPU wheel implementation.
Do not replace the ray plant with articulated wheels to obtain a favorable number.

## Available APIs and licensing

* **`physx 0.19.0` + `physx-sys 0.11.5`:** downloaded and built unchanged. Its
  bundled `PxPhysicsVersion.h` identifies **PhysX 5.1.3**. `build.rs` defines
  `DISABLE_CUDA_PHYSX`; there is no GPU Cargo feature. Safe wrapper calls cover
  bodies, force/torque, poses/velocities and synchronous simulate/fetch. The
  `Scene` wrapper has no raycast method; reaching native scene queries requires
  lower-level access. `get_dynamic_actors()` allocates a vector on each call.
  The wrapper repository was archived on **2026-05-21**, despite its README still
  describing maintenance. Wrapper license: MIT OR Apache-2.0; bundled historical
  SDK: BSD-3-Clause, with its own third-party notices. Sources:
  [repository](https://github.com/EmbarkStudios/physx-rs),
  [packaged build](https://docs.rs/crate/physx-sys/0.11.5/source/build.rs).
* **`ovphysx 0.7.0.74035321`:** the current NVIDIA package tested here, using
  PhysX 5.12 according to its release. Its C API and Python API support scene
  population, explicit stepping, batched reads/writes and device arrays.
  Pre-release APIs are still changing. It is a materially more current option
  than the Rust package. See the [0.7 release](https://github.com/NVIDIA-Omniverse/PhysX/releases/tag/ovphysx-0.7.0).
* **Distribution is version-specific:** current ovphysx source is Apache-2.0,
  whereas its ready-made SDK packages and Python wheels are under the NVIDIA
  Omniverse License. Its published platforms are Linux x86_64/aarch64 and Windows
  x86_64; x86_64 binaries require AVX. A Mac application cannot assume the same
  native GPU path. Source builds and all bundled dependency licenses need review
  against `deny.toml` before shipping. See NVIDIA's
  [README, requirements and licensing](https://github.com/NVIDIA-Omniverse/PhysX/blob/main/ovphysx/README.md).

The modern Python `PhysX.raycast`/`sweep`/`overlap` methods exist but are marked
**deprecated since 0.6.0**. This was verified by inspecting the installed method
signatures/docstrings, rather than inferring absence from the old Rust wrapper.
Safe Python does not make an API stable or imply that it operates on device data.

## Talos capability and isolation

Observed through the fleet skill over ordinary SSH:

* Ryzen 9 9950X3D, 16 cores / 32 threads; approximately 123 GiB RAM.
* RTX 5090, 32607 MiB, driver 615.78.08; `nvidia-smi` reports CUDA UMD 13.4.
* Linux CachyOS x86_64; Rust 1.99.0; system Clang 23.1.1; Python 3.12.14.
* No `nvcc`, `cmake`, or installed PhysX SDK was found in the inspected command
  paths, `/opt` and `/usr/local`. This does **not** mean GPU execution is
  impossible: ovphysx and Warp wheels supply their runtime components.
* Warp 1.18.0 actually initialized `cuda:0`, RTX 5090, `sm_120`. The DirectGPU
  read returned CUDA arrays, so the smoke result is not merely driver detection
  or a CPU fallback.

The Rust probe uses a private `CARGO_TARGET_DIR` and eight build jobs. The Python
probe uses a private `ov-env`, `warp-cache`, and collider cache. Normal shared
Cargo/uv download caches were used; no shared build target or parent checkout was
used. The target occupied about 345 MiB. Installed Python package versions and
machine details are in `results/ov-freeze.txt` and `results/machine.txt` locally.
These are feasibility measurements on a shared machine, without CPU affinity,
frequency locking or exclusive scheduling.

## Matched CPU experiment

`probe/src/wheel_model.rs` is copied byte-for-byte from the base commit. The probe
uses its public suspension, free-roller, motor, wheel-inertia, slip and shared
power functions, with the default omni chassis's fitted parameters. This is not
a second implementation of those equations. The adapter reproduces fixed hub
kinematics and force-at-point torque summation from `chassis::apply_forces`.

Both backends receive the same simplified box: 22 kg, 0.41 × 0.41 × 0.10 m,
matching box inertia, initial position `(0, 0, 0.2065)` m, identity rotation,
zero velocity, gravity `(0, 0, -9.81)` m/s², damping 0.05 / 0.5, no sleeping.
Each has an actual colliding floor. This is **not** the full octagonal chassis,
turret, armor, wheel guards or CAD field. Default suspension/motor parameters
are retained, with the four diagonal hubs at radius 0.245 m. Both adapters use
one immutable Parry half-space ray-query representation of the same static floor.
There is no second dynamic world behind the PhysX adapter.

Order is read pose/velocity → four contact queries → suspension/roller laws →
shared wheel integrator → replace accumulated body forces/torques → solver step
→ read and check finite output. Scheduling matches multiplayer physics: eight
`0.0009765625` s solver slices per 128 Hz tick, with the wheel model's eight
internal motor slices inside each. Each world has separate wheel state and its
own solver scene. PhysX uses zero dispatcher workers; the outer batch is
**sequential**, not CPU-parallel or GPU-batched. PhysX stores f32 solver state;
Rapier and the shared wheel model use f64.

Each case warms for 256 ticks (2 s), then measures three consecutive 256-tick
windows, ending at 4, 6 and 8 simulated seconds. These are windows on one
trajectory, not three independent process trials. Full-loop measurements include
queries, control, force application, allocation in the safe PhysX getters,
readback, finite/traction checks and timing overhead. Engine-step measurements
surround `PhysicsWorld::step` or PhysX simulate/fetch; these include collision
processing and are **not a constraint-solver-only profile**. Per-slice timer cost
is included. No rule engine, projectile, LiDAR, networking or rendering runs.

Forward command 2 m/s, default grip, median **µs per world per 128 Hz tick**:

| Independent worlds | Rapier full loop | Rapier engine step | PhysX CPU full loop | PhysX CPU engine step |
|---:|---:|---:|---:|---:|
| 1 | 13.419 | 10.315 | 30.517 | 26.626 |
| 32 | 13.365 | 10.288 | 32.584 | 28.374 |
| 256 | 13.434 | 10.351 | 33.881 | 29.346 |

At 256 worlds this is approximately 3.44 ms / 8.67 ms for an entire sequential
batch tick. It is a fixture cost estimate, **not full-app speed or training
throughput**. PhysX is approximately 2.3–2.5× slower in this sparse fixture;
solver configuration, safe-wrapper overhead and f32/f64 differ. This does not
rank the engines for large contact-rich scenes or optimized parallel batching.

Scene creation for the forward case was approximately 0.05 ms for one PhysX
scene and several milliseconds for 256; Rapier was below 1 ms for 256.
These creation values exclude process/global foundation initialization and are
single samples, not stable startup benchmarks. The fresh-target dependency build
and first probe compile attempt took approximately 21 s by log timestamps; that
attempt ended in probe API compile errors. The subsequent successful warm build
reported 0.31 s. Neither value is an isolated successful cold build benchmark.
The measured whole CPU runs took 11.93 s / 30.11 s including all cases/warmup;
child peak RSS high-water marks were about 18 / 120 MiB, respectively.

## Correctness evidence and limits

* At rest, the predicted spring height is
  `0.2065 - 22*9.81/(4*1000) = 0.152545` m. Rapier ended at 0.15254500 m;
  PhysX at 0.15254475 m. Both are within 10 µm of prediction.
* At 8 s under forward command, positions were 14.94464063 / 14.94569492 m;
  speeds were 1.96845435 / 1.96845102 m/s. Across all sampled matching cases,
  positions differed by less than 2 mm horizontally and 1 mm vertically;
  sampled linear velocities differed by less than 0.1 mm/s.
* Reducing only the model's drive grip from 1.0 to 0.1 exercises slip-limited
  acceleration: at 4 s, forward distance decreases from about 7.071 m to 2.670 m
  and speed from about 1.968 to 1.282 m/s in both engines. Every slice checks
  finite wheel state and `abs(traction) <= available_grip`; measured motor power
  never exceeds the shared 120 W budget. This is not an experimental tyre fit.
* With wheel forces disabled, native box-floor collision settles at
  0.04988502 / 0.05000000 m, within 1 mm of the 0.05 m half-height. This exercises
  actual solver collision, unlike the supported ray-wheel cases that float
  above the box-floor contact.
* Six scratch Rust tests pass: the five unchanged wheel tests plus floor
  hit/miss and wall-ray fixtures. A separate native C++ PhysX 5.1.3 query probe
  checks the same four floor rays and wall surface. Floor distance error is at
  most `1.1920929e-7` m; wall distance is 1.89999998 m versus Parry's 1.9 m.
  Query tolerance is 1 µm. Native PhysX queries are **not** in the CPU timing loop.
* These checks do not establish general triangle-mesh/query equivalence,
  two-sided scoring contacts, moving support forces, CCD, contact ordering,
  restore equivalence, deterministic replay or real robot fidelity.

## Modern PhysX GPU smoke test

`ov-smoke.py` authors metre/kg/Z-up USDA with GPU dynamics and GPU broadphase
explicitly enabled. One scene contains 1, 32 or 256 separated colliding boxes
and a shared floor. They are spatial replicas, **not independent world scenes**.
The test warms for 1024 slices, then measures three windows of 256 synchronous
`1/1024` s steps; it reads positions once after each window. All boxes settle
within 25 mm of the expected 0.05 m centre height (observed error below 20 nm).
No wheel equations, wheel queries, forces, moving armor or projectiles run.

Median synchronous **whole-batch step time in µs per solver slice**, including
Python/native submission, completion and implicit backend work:

| Boxes in scene | GPU with CPU readback | DirectGPU | DirectGPU sampled position read + CPU copy, ms |
|---:|---:|---:|---:|
| 1 | 573.47 | 570.83 | 0.466 |
| 32 | 629.20 | 610.12 | 0.445 |
| 256 | 690.63 | 649.55 | 0.498 |

The read metric is a separate sparse observation measurement, not readback on
every solver slice or a complete control loop. It includes creation/release of
read sessions and `.numpy()` copies. DirectGPU really returns `cuda:0`; ordinary
GPU mode returns CPU arrays. Initial library/scene attach was 2.62 s on the first
process attempt; later runs were about 0.11–0.14 s per scene with warm caches.
First solver step was about 1.2 ms on later runs. First read incurred additional
initialization (71 ms on the first attempt). These startup costs are excluded
from steady step figures.

`nvidia-smi` measured this probe process at approximately 1134 MiB with one box,
1168 MiB with 32, and 1168 MiB with 256, sampled after warmup. This includes CUDA
context, runtime pools and scene capacities retained across cases; it is **not**
a linear per-world memory estimate or a peak allocation trace. Geometry here is
primitive and tiny. No meaningful CAD capacity limit was measured.

## GPU integration constraints

The separate **current-package query probe reproduces stale geometry**. After
1024 slices, the dynamic box's device position is z = 0.049999986 m. A downward
ray from `(0,0,1)` should therefore hit its top at z = 0.1 m:

* GPU with CPU readback: hit z = 0.099999987 m, distance = 0.899999976 m.
* DirectGPU: hit z = 0.256500006 m, distance = 0.743499994 m, which is the
  **initial** box top before falling. No query error is raised.

Thus a successful CUDA state read does not make the exposed scene query coherent.
Static terrain wheel rays alone could hide this failure; moving supports,
self-occlusion, armor and LiDAR would reveal it. `ov-query.py` and the retained
`ov-query-{readback,direct}.log` show the exact calls and results. This is the
strongest blocker demonstrated here, rather than an inferred API limitation.


GPU dynamics and DirectGPU are distinct choices. The documented GPU pipeline
accelerates broadphase, contact generation, body/shape management and solving;
mesh GPU cooking data and compatible contact settings matter. Do not assume
wheel queries or sensors automatically move onto CUDA. See
[GPU simulation](https://nvidia-omniverse.github.io/PhysX/physx/latest/docs/GPURigidBodies.html).

The currently published SDK DirectGPU guide says CPU poses can be stale, scene
queries depend on those poses, and ordinary CCD and vehicles are incompatible
with that mode. A complete adapter must prove current-version behavior rather
than silently substituting stale queries or disabling CCD for tiny fast shots.
See [DirectGPU restrictions](https://nvidia-omniverse.github.io/PhysX/physx/latest/docs/DirectGPUAPI.html).

The newer ovphysx 0.7 documentation additionally specifies that DirectGPU lacks
kinematic-target writes, changes contact event availability/order, and makes
some live edits unsafe or unsupported. Position teleportation is not a substitute
for a prescribed moving support's velocity. See
[0.7 mode restrictions](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/read_write/limitations.html).
Its write contract does expose world-space force/wrench input, consumes external
forces each step and uses xyzw quaternions. An adapter must convert FLU/wxyz,
respect buffer lifetimes and CUDA completion explicitly. See
[write contract](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/read_write/writable.html).

A GPU-with-readback adapter is a useful compatibility-first next experiment;
it can retain CPU wheel equations while exposing the synchronization cost.
A device-resident implementation needs equivalent wheel equations, batched
queries against coherent poses, persistent wheel state, bounded contacts and
per-tick rule effects. This evaluation deliberately stops before implementing
that larger adapter. MuJoCo/MJWarp remains an alternative for a later training
comparison, not a measured winner here; its actual batched ray API and solver
feature coverage should be checked against the chosen model. See
[MJWarp documentation](https://mujoco.readthedocs.io/en/latest/mjwarp/).

## API shape and acceptance gates

Keep explicit time and model state separate from backend handles. The next
adapter should batch spawn/remove, forces/torques, rays and compact pose/velocity
reads, retain buffers, and drain contacts with stable body/shape identities and
explicit overflow. The scratch PhysX getter allocations are evidence for this
boundary, not a reason to change the core trait hierarchy now.

Preserve one authoritative interacting world. A derived query representation
must share geometry revisions and authoritative timestamps. Rolling LiDAR and
IMU captures must not interpolate across placement/reset/topology changes.
Version every checkpoint with engine, precision, solver configuration and model
hashes; cross-backend state transfer is initialization, not exact rollback.
Solver manifolds/warm starts and hidden projectile/contact history remain
continuation concerns. Keep multiplayer prediction on matching backends; this
experiment proves neither deterministic replay nor checkpoint restoration.

Before a migration decision, add representative verified CAD, ramps/overhangs,
projectile CCD against moving armor, complete filtering/contact IDs, sensor
queries, resets and restore residuals. Then measure 1/32/256 complete control
loops with CPU observation versus device-resident observation separately,
including capacities, data movement and rules. No such full-app speedup is
claimed by this report.

## Reproduction and retained artifacts

All authored experiment code is ignored, contains no Rust unsafe code, and has
an SPDX header. The native query check is an isolated C++ SDK executable. The
only tracked addition owned by this evaluation is this report; nothing was
staged, committed or pushed.

Local artifacts:

* `local-assets/physics-backend-eval/probe/`: standalone Cargo workspace, exact
  wheel-model copy, safe adapters and resolved `Cargo.lock`.
* `run.py`, `analyze.py`: bounded CPU runner and sampled tolerance assertions.
* `native-query.cpp`: direct native floor/wall scene-query check.
* `ov-smoke.py`, `ov-query.py`: current-package GPU and query capability probes.
* `results/`: raw CSV, summaries, build/test/environment/install logs and GPU
  JSON/logs. Early failed probes are retained remotely for diagnosis.

From a copy of this scratch directory on Talos (paths are deliberately private):

```sh
cd /home/hxyulin/.cache/physics-backend-eval-20261023
export CARGO_TARGET_DIR="$PWD/target"
export CARGO_BUILD_JOBS=8
export CXX=/usr/bin/clang++
cargo build --release --locked --manifest-path probe/Cargo.toml
LD_LIBRARY_PATH=/usr/lib cargo test --release --locked --manifest-path probe/Cargo.toml
python3 run.py
# Copy CSVs into local results/ before running analyze.py locally.
uv venv --python python3 ov-env
uv pip install --python ov-env/bin/python ovphysx==0.7.0.74035321 physx-usd-schemas==25.11.2.72696984 warp-lang==1.18.0
ov-env/bin/python ov-smoke.py
ov-env/bin/python ov-smoke.py direct
ov-env/bin/python ov-query.py
ov-env/bin/python ov-query.py direct
```

The existing Talos Rust linker is Nix-provided while C++ is system Clang; its
binary initially could not find `libstdc++.so.6`. `run.py` sets
`LD_LIBRARY_PATH=/usr/lib` for the child only. No global linker or shell setting
was changed. The native query build uses the unchanged cached physx-sys SDK
headers and `target/release/build/physx-sys-*/out/libphysx.a`, linking pthread/dl;
its exact reusable command is in scratch `reproduce-native.sh`.

The wheel-model SHA-256 is
`86cbd5f4b9723bc6e09e2594e645449a61144d3003b5efe14e24ba8a99687815`
in both the base checkout and scratch. `analyze.py` passes all sampled CPU
tolerances. Only scratch tests were run; full workspace verification was not
needed for this report and would overlap the parent's concurrent implementation.
