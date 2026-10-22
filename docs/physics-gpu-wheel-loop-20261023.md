<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Batched GPU ray-wheel loop — 2026-10-23

Evaluation source base: `f9c5b02f30c1a102d8098d91b2efb1ec59f03ff5`.
This continues [the CPU comparison and GPU query investigation](physics-backend-evaluation-20261023.md).
Only this new report is intended for tracking. All probe code, raw results and
logs are ignored under `local-assets/physics-backend-eval/`, mirrored in
`/home/hxyulin/.cache/physics-backend-eval-20261023/` on Talos. No core crate,
workspace manifest/lock, branch, existing report, parent source snapshot,
shared build target, driver or system package was changed. Nothing was staged,
committed or pushed.

## Result and scope

A complete batched wheel loop ran successfully in ordinary GPU-with-readback
and DirectGPU modes for 1/32/256 robots. The wheel equations are unchanged Rust;
CUDA runs the authoritative rigid-body solver and a current-pose primitive ray
kernel. The DirectGPU query workaround passes the moving-obstacle checks, but
still copies data through CPU control. It is not a fully device-resident loop.

For forward drive, median whole-batch times are 13.63/12.61/20.58 ms per 128 Hz
tick with readback and 13.62/15.01/21.35 ms with DirectGPU, for 1/32/256 replicas.
At 256 robots that amortizes to 80.39/83.41 µs per robot per tick. The matched
sequential Rapier fixture is 13.58 µs per independent world per tick, approximately
3.48 ms for 256 worlds. This hybrid implementation gives no throughput win in
this fixture; its batch time also exceeds the 7.8125 ms real-time tick budget.
These are whole-adapter costs with different batching arrangements, not an
intrinsic solver ranking.

Settling, forward drive, grip/power limits, finite state and query coherence
checks pass. Tight cross-backend trajectory equivalence is **not established**:
low-grip samples differ from the CPU references by as much as 17.2 mm in x and
0.0116 m/s in vx. The exact wheel-law checks must not be confused with solver
trajectory equivalence.

The GPU loop contains the real ray-wheel controller, not the earlier gravity-only
box smoke. Each case has **one GPU PhysX scene with spatially separated replicas**,
2 m apart along y, and one common static floor. These are not independent scenes
or isolated worlds: collisions and queries can see other replicas. Each robot
owns independent persistent Rust wheel state. All benchmark wheel rays must hit
the floor; queries include the other dynamic boxes and exclude their own body.
The static wall is off the driving lane. A separate dynamic query test exercises
a falling, horizontally moving obstacle, including rays that do and do not
exclude that body.

Both modes use GPU dynamics and GPU broadphase on Talos's RTX 5090. `readback`
uses ordinary GPU simulation with CPU-resident observation columns; `direct`
uses suppressReadback / DirectGPU with CUDA-resident observation columns. Both
are **hybrid CPU-control loops**. Neither is a device-resident rollout benchmark.

## Fixture, equations and scheduling

The fixture retains the prior adapter's 22 kg, 0.41 × 0.41 × 0.10 m box, explicit
box inertia, initial position `(0, 2*i, 0.2065)` m, identity xyzw quaternion,
zero velocity, gravity `(0,0,-9.81)` m/s², linear/angular damping 0.05/0.5 and
sleep disabled. It is a simplified chassis fixture, not the complete robot. A separate live
read verifies both modes retain mass 22 kg, inertia diagonal
`(0.3265166581, 0.3265166581, 0.6163666844)` kg m², zero COM offset and identity
COM orientation (`gpu-loop/fixture-check.json`); inertia agrees with the
analytic box values within 1e−6 kg m².
The four diagonal hub offsets have radius 0.245 m and z = −0.05 m, ray reach
0.1565 m. Default fitted motor, wheel inertia, suspension, free-roller and grip
parameters are copied from the prior scratch adapter. The shared power budget
is 120 W. `low_grip` changes drive friction only, from 1.0 to 0.1.

The unchanged `probe/src/wheel_model.rs` is compiled into a persistent safe Rust
subprocess, `wheel-bridge`, using the existing scratch Cargo manifest and lock.
It is byte-identical to the base repository source. The adapter changes only
the source of ray distance/normal, receiving measured query results instead of
calling Parry on a half-space. Rust retains all four wheels per robot across
requests and applies the original suspension and roller forces, shared wheel
integrator, then rolling traction with the original force-at-point torque sums.
It sends one resultant force/torque pair per chassis. No articulated wheels,
new dependency or Rust unsafe code are introduced.

The original five wheel tests and a new adapter-equivalence test pass. The latter
compares both adapters for 1,024 consecutive states for each combination of
forward target 0/+2/−2 m/s and default/low grip, including roll, nonzero linear
and angular velocity, and persistent motor/spin state: 6,144 comparisons.
Forces/torques agree within 1e−12 in norm and wheel states compare exactly.
This validates the bridge adapter against the prior equations; it does not
claim equality between different physics solvers.

One 128 Hz tick always contains **eight 1/1024 s solver slices**. Every slice
retains the wheel model's **eight internal motor/slip integration slices**.
The order is completed current pose/velocity read → four wheel rays per robot →
Rust suspension/roller/motor calculation → replace world-space wrench → solver
step → fresh pose/velocity read and finite checks. The post-step read supplies
the next slice with no intervening physics or edits; it is not reused across
solver steps. Wrenches include the current COM as their application point, so
the Rust torque already includes every lever arm. External forces are consumed
by each solver step.

DirectGPU requires one bootstrap step before its read/write interface is usable.
Both modes take that single zero-wheel-force setup step, then warm up for
256 ticks (2 s) with the complete wheel loop. Three measured 64-tick windows
follow on the same trajectory, ending at 2.5009765625, 3.0009765625 and
3.5009765625 simulated seconds. They are consecutive windows, not independent
process trials. Setup includes scene population/attach, bridge startup, query buffers, bootstrap
and first read. Python imports, CUDA initialization and query compilation occur
in the preliminary query check and are not separately timed; setup figures are warm-process
scene costs. Setup and warmup are excluded from the measured medians. No
solver slices or state checks are removed from timed windows.

## Coherent primitive GPU queries

`gpu-loop/wheel_loop.py` uses maintained Warp 1.18 CUDA execution and ovphysx 0.7
session reads/writes. It never calls deprecated PhysX `raycast`/tensor bindings.
Its small authored kernel performs slab intersections in each box's local frame
and transforms normals back to world space. It tests the static floor/wall and
all current dynamic boxes, with explicit per-ray self-exclusion and nearest-hit
selection. This is a brute-force **derived primitive query representation**, not
an SDK GPU raycast or a production collision-query implementation. It uses no
initial dynamic pose cache and no independent dynamical world.

At every slice the completed authoritative state read is copied into a new
current-pose upload to persistent CUDA buffers before ray launch. The query mirror
is therefore identified by the caller's solver-slice sequence and fixed geometry
recipe. The obstacle experiment records exact slice numbers and simulation
timestamps alongside authoritative pose and measured distances. Topology,
shape revisions and reset epochs are fixed in this bounded experiment; a real
adapter would need to version and invalidate those explicitly.

The read session establishes the Warp stream dependency on its producer event;
`.numpy()` copies wait for observation data. Query launches and uploads run on
the current Warp stream, followed by an explicit device synchronization before
CPU hit copies. The wrench upload is synchronized before commit, and commit
and synchronous stepping finish before the next read. Read/write path-list
contents are checked for equal body identity and order every slice; path-list
handles themselves differ across sessions and cannot be used as identity.

Copies per slice include all 13 f32 state values per robot from PhysX to NumPy,
a 13-f32 current-pose/state upload to the query kernel, 4 × (distance + normal)
f32 hit values plus four i32 hit IDs back to CPU, f64 binary pipe input of
29 values per robot and output of eight values, and nine f32 wrench values to
PhysX. NumPy also repacks/casts arrays. DirectGPU still performs these explicit
CPU copies, including observation readback; its name does not imply otherwise.
Read/write sessions are opened and released each slice. Persistent query and
Rust wheel buffers avoid rebuilding physical scenes, but Python/session/pipe
allocation and synchronization costs remain in the totals.

## Runtime measurements

All component columns below are **milliseconds per entire batch per 128 Hz tick**; the last column alone is amortized microseconds per replica. Values are medians of the three complete windows.

| Mode | Replicas | Control | Batch ms/tick | Read | Query | Bridge | Write | Step | Other | µs/replica/tick |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| readback | 1 | settle | 11.651 | 3.484 | 1.138 | 0.149 | 2.216 | 4.351 | 0.314 | 11651.17 |
| readback | 1 | forward | 13.631 | 4.080 | 1.257 | 0.175 | 2.635 | 5.101 | 0.393 | 13630.55 |
| readback | 1 | low_grip | 11.366 | 3.378 | 1.122 | 0.145 | 2.165 | 4.246 | 0.307 | 11365.80 |
| readback | 32 | settle | 12.680 | 3.850 | 1.211 | 0.255 | 2.578 | 4.432 | 0.352 | 396.23 |
| readback | 32 | forward | 12.613 | 3.810 | 1.209 | 0.258 | 2.585 | 4.417 | 0.350 | 394.15 |
| readback | 32 | low_grip | 12.671 | 3.848 | 1.203 | 0.258 | 2.548 | 4.431 | 0.348 | 395.96 |
| readback | 256 | settle | 21.550 | 6.999 | 1.970 | 0.860 | 5.800 | 5.438 | 0.482 | 84.18 |
| readback | 256 | forward | 20.579 | 6.511 | 1.900 | 0.880 | 5.571 | 5.296 | 0.421 | 80.39 |
| readback | 256 | low_grip | 20.111 | 6.458 | 1.906 | 0.843 | 5.381 | 5.121 | 0.402 | 78.56 |
| direct | 1 | settle | 13.546 | 5.039 | 1.018 | 0.127 | 3.288 | 3.739 | 0.310 | 13545.88 |
| direct | 1 | forward | 13.617 | 5.059 | 1.034 | 0.132 | 3.323 | 3.746 | 0.314 | 13617.29 |
| direct | 1 | low_grip | 13.528 | 5.002 | 1.017 | 0.132 | 3.282 | 3.784 | 0.313 | 13527.78 |
| direct | 32 | settle | 14.902 | 5.539 | 1.132 | 0.233 | 3.682 | 3.953 | 0.351 | 465.68 |
| direct | 32 | forward | 15.011 | 5.593 | 1.156 | 0.238 | 3.711 | 3.945 | 0.353 | 469.09 |
| direct | 32 | low_grip | 15.030 | 5.583 | 1.155 | 0.237 | 3.727 | 3.966 | 0.357 | 469.70 |
| direct | 256 | settle | 21.281 | 8.119 | 1.809 | 0.784 | 5.895 | 4.266 | 0.408 | 83.13 |
| direct | 256 | forward | 21.354 | 8.079 | 1.804 | 0.824 | 5.906 | 4.360 | 0.408 | 83.41 |
| direct | 256 | low_grip | 21.354 | 8.111 | 1.805 | 0.827 | 5.924 | 4.279 | 0.411 | 83.41 |


`read` includes coherent pose/velocity collection, CPU copies, identity checks
and finite-state assertions. `query` includes state upload, wheel-ray generation,
all-shape intersections, explicit CUDA completion, and hit/ID CPU copies.
`bridge` includes NumPy input packing, both pipe transfers, unchanged Rust
controller/integrator execution and power/traction assertions. `write` includes
wrench packing, session creation, identity check, upload, synchronization and
commit. `step` is the whole synchronous PhysX step, including submission,
collision and fetch/completion, not isolated solver kernel time. `other` includes
slice/timer overhead and current ray/floor-distance validation. Component medians
need not sum exactly to the median total.

This is a shared workstation, without affinity, clock locking or exclusive GPU
access. No multi-process CPU/GPU benchmark runs were intentionally overlapped.
These wall-clock numbers measure this explicit adapter, not GPU saturation,
optimized achievable kernel performance, full-game speed or a fair isolated
CPU-versus-GPU solver contest.

## Accuracy and query checks

Setup and warmup are wall-clock costs; warmup advances 2 simulated seconds.
Final states below are sampled from the first returned robot at
3.5009765625 simulated seconds (body order is identity-checked, not assumed
numeric). All replicas have identical sampled z minima/maxima. Peak limits
include startup/warmup as well as timed windows.

| Mode | Replicas | Control | Setup ms | First step ms | Warmup s | Final x m | Final z m | Final vx m/s | Peak power W | Peak traction/grip |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| readback | 1 | settle | 109.8 | 1.077 | 2.978 | 0.0000000 | 0.1525448 | 0.0000000 | 0.0000 | 0.00000000 |
| readback | 1 | forward | 112.8 | 0.948 | 3.030 | 6.0872836 | 0.1538559 | 1.9683228 | 120.0000 | 1.00000000 |
| readback | 1 | low_grip | 121.9 | 1.151 | 3.157 | 2.0491145 | 0.1546007 | 1.1260093 | 120.0000 | 1.00000000 |
| readback | 32 | settle | 116.2 | 1.269 | 3.208 | 0.0000000 | 0.1525448 | 0.0000000 | 0.0000 | 0.00000000 |
| readback | 32 | forward | 114.4 | 1.119 | 3.245 | 6.0872836 | 0.1538559 | 1.9683228 | 120.0000 | 1.00000000 |
| readback | 32 | low_grip | 116.5 | 1.209 | 3.219 | 2.0491145 | 0.1546007 | 1.1260093 | 120.0000 | 1.00000000 |
| readback | 256 | settle | 145.0 | 1.384 | 5.117 | 0.0000000 | 0.1525448 | 0.0000000 | 0.0000 | 0.00000000 |
| readback | 256 | forward | 150.2 | 1.293 | 5.383 | 6.0872836 | 0.1538559 | 1.9683228 | 120.0000 | 1.00000000 |
| readback | 256 | low_grip | 146.2 | 1.222 | 5.158 | 2.0491145 | 0.1546007 | 1.1260093 | 120.0000 | 1.00000000 |
| direct | 1 | settle | 112.9 | 1.220 | 3.441 | 0.0000000 | 0.1525448 | 0.0000000 | 0.0000 | 0.00000000 |
| direct | 1 | forward | 113.6 | 1.309 | 3.484 | 6.0872836 | 0.1538559 | 1.9683228 | 120.0000 | 1.00000000 |
| direct | 1 | low_grip | 109.8 | 1.067 | 3.461 | 2.0491145 | 0.1546007 | 1.1260093 | 120.0000 | 1.00000000 |
| direct | 32 | settle | 119.3 | 1.220 | 3.815 | 0.0000000 | 0.1525448 | 0.0000000 | 0.0000 | 0.00000000 |
| direct | 32 | forward | 116.1 | 1.366 | 3.817 | 6.0872836 | 0.1538559 | 1.9683228 | 120.0000 | 1.00000000 |
| direct | 32 | low_grip | 115.4 | 1.334 | 3.818 | 2.0491145 | 0.1546007 | 1.1260093 | 120.0000 | 1.00000000 |
| direct | 256 | settle | 143.4 | 1.337 | 5.476 | 0.0000000 | 0.1525448 | 0.0000000 | 0.0000 | 0.00000000 |
| direct | 256 | forward | 147.1 | 1.452 | 5.494 | 6.0872836 | 0.1538559 | 1.9683228 | 120.0000 | 1.00000000 |
| direct | 256 | low_grip | 149.1 | 1.286 | 5.476 | 2.0491145 | 0.1546007 | 1.1260093 | 120.0000 | 1.00000000 |

Across both GPU modes and all replica counts, resting centre height is
0.1525447816 m versus the static spring prediction 0.152545 m, an error of
0.2185 µm. Forward speed ends at 1.9683228 m/s. Low grip reduces final distance
from 6.0872836 to 2.0491145 m and speed to 1.1260093 m/s. Maximum observed
traction/grip is 1.0000000000000002 (roundoff); peak power is
120.00000000000001 W. No non-finite states occur.

Maximum ray-distance error is 1.137e−7 m for the wheel rays and 2.385e−8 m
for the controlled obstacle/floor/wall test. The obstacle moves from
`(0.000391, 0, 1.499995)` m at slice 1 to approximately
`(0.215150, −0.000216, 0.050000)` m at slice 1024. Its downward ray hit follows
the current top throughout the fall. Excluding that same body reaches the
floor; excluding it from an upward interior ray produces a miss. Both modes
return the expected body/floor/wall IDs and normals in every sample.

The new CPU-reference binary reproduces the same bootstrap and observation
windows using the prior adapters, unchanged equations and native solver setup.
Maximum absolute differences over all three sampled windows and replica counts
are below; both GPU modes have the same listed maxima. Position/velocity results
are sampled representatives, not complete per-body trajectory captures.

| CPU reference | Control | x error mm | z error mm | vx error mm/s | vz error mm/s |
|---|---|---:|---:|---:|---:|
| rapier | settle | 0.000000 | 0.000218 | 0.000000 | 0.001569 |
| rapier | forward | 0.681289 | 0.295730 | 0.131576 | 0.073288 |
| rapier | low_grip | 16.787024 | 0.039340 | 11.499875 | 0.009971 |
| physx | settle | 0.000000 | 0.000032 | 0.000000 | 0.000271 |
| physx | forward | 1.152519 | 0.294450 | 0.128266 | 0.074328 |
| physx | low_grip | 17.196414 | 0.039220 | 11.529085 | 0.010521 |

These residuals are larger than the previous CPU-only backend comparison,
especially under traction saturation. They were retained rather than tuned
away. This experiment does not isolate how much comes from solver/version/
precision behavior versus f32 ray and adapter arithmetic. The adapter equivalence
test isolates the Rust wheel calculation; it does not bound accumulated solver
error. Do not claim interchangeable trajectories or rollback from these results.

Matched CPU forward-loop medians (µs per independent world per 128 Hz tick):

| Independent worlds | Rapier | PhysX CPU 5.1.3 |
|---:|---:|---:|
| 1 | 13.597 | 30.450 |
| 32 | 13.368 | 31.507 |
| 256 | 13.583 | 32.527 |

Every measured and warmup slice checks finite body/wheel state, mean traction
within the current available grip, motor allocation within 120 W, floor hit IDs
and ray distances against the current body quaternion and hub height. The query
test checks dynamic-box top hits at slices 1, 128, 256, 512 and 1024; the same ray
with the body excluded must reach the floor. A horizontal ray checks the wall,
an off-body ray checks static floor, and a ray starting inside the excluded body
must miss upwards. These tests distinguish current moving-body geometry from
initial-pose or static-only query success. They cover unrotated primitives and
the small rotations generated by the fixture; they are not general mesh/query
validation or a moving-support force-response test.

## DirectGPU API boundary and recommendation

The current SDK still documents CPU scene queries as dependent on stale host
poses under DirectGPU, and both sweep and speculative CCD as unsupported there.
The custom current-pose query mirror addresses the demonstrated ray-coherence
problem **for this primitive fixture only**, not PhysX scene-query or CCD support.
See NVIDIA's [DirectGPU restrictions](https://nvidia-omniverse.github.io/PhysX/physx/latest/docs/DirectGPUAPI.html).

ovphysx session reads/writes are the maintained interface used here. Its current
contract says wrenches are world-space, quaternions xyzw, external forces clear
per step, CUDA fills require producer synchronization, and writes need the
DirectGPU bootstrap. See [writable data](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/read_write/writable.html).
Kinematic-target writes are refused under DirectGPU, and pose teleportation
lacks a target's step velocity. Contact event/ordering differences also remain.
See [current session limitations](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/read_write/limitations.html).

Keep Rapier for the game and prediction. The complete hybrid loop now provides
concrete runnable GPU backend evidence, including its synchronization costs.
It does not justify an engine migration or establish compatibility with fast
projectile CCD, moving armor, kinematic platforms, CAD triangle filtering,
sensor captures, checkpoint continuation, contact identity/order or deterministic
replay. Those API/behavior limits are compatibility blockers, not speed results.
A future training-focused device-resident experiment would first need a checked
wheel-law kernel, persistent batched observations/writes, coherent mesh queries
and an explicit policy for unsupported contacts/CCD/kinematics.

## Reproduction and identities

The tested stack reuses Talos's private `ov-env`: ovphysx 0.7.0.74035321,
Warp 1.18.0, physx-usd-schemas 25.11.2.72696984, RTX 5090, driver 615.78.08,
CUDA runtime/driver 13.4, Rust 1.99.0 and scratch Rapier 0.35.3 / PhysX Rust
0.19.0 with native PhysX 5.1.3 for the CPU references. The GPU runtime is the
ovphysx package, not that older Rust SDK. Warp identifies the compiled query
module as `__main__ 482ad37`; the runner source SHA below fully identifies the
kernel and host adapter. `PhysX(config=...)` emits a constructor deprecation
warning about global configuration; it is not a deprecated scene-query call.

SHA-256 identities (also retained in `gpu-loop/identities.sha256`):

```text
8c62c2db48184779c659b8bf311f174eadfccca11f0e89305c43d73c406d4d1a  gpu-loop/wheel_loop.py
94156b41935073d0165be87ee8bd8481a64888d505362f53cfc9e37d8465aab1  probe/src/bin/wheel-bridge.rs
538b1f7b41938ea97da02565ac15ef6de19408f0e7d58d8bc02a4eae78427ee2  probe/src/bin/wheel-reference.rs
86cbd5f4b9723bc6e09e2594e645449a61144d3003b5efe14e24ba8a99687815  probe/src/wheel_model.rs
bfaa44b22ee3409ee97ee6721ec75f93a3c907b4e41ba95fe067091506b1ce83  target/release/wheel-bridge
403755d7e3ba09f54a7f2464d513b76a2c678b272ad5c60152cce351395ebc27  target/release/wheel-reference
```


Use ordinary fleet SSH. All paths below belong to this experiment, not the
parent's checkout/target. No environment creation or install is needed on the
retained Talos scratch:

```sh
ssh talos
bash
cd /home/hxyulin/.cache/physics-backend-eval-20261023
export CARGO_TARGET_DIR="$PWD/target"
export CARGO_BUILD_JOBS=8
export CXX=/usr/bin/clang++
LD_LIBRARY_PATH=/usr/lib cargo test --release --locked \
  --manifest-path probe/Cargo.toml --bin wheel-bridge
cargo build --release --locked --manifest-path probe/Cargo.toml \
  --bin wheel-bridge --bin wheel-reference
ov-env/bin/python gpu-loop/wheel_loop.py > gpu-loop/run-readback.log 2>&1
ov-env/bin/python gpu-loop/wheel_loop.py --direct > gpu-loop/run-direct.log 2>&1
LD_LIBRARY_PATH=/usr/lib target/release/wheel-reference rapier \
  > gpu-loop/reference-rapier.csv
LD_LIBRARY_PATH=/usr/lib target/release/wheel-reference physx \
  > gpu-loop/reference-physx.csv
ov-env/bin/python gpu-loop/analyze_loop.py
ov-env/bin/python gpu-loop/fixture_check.py
```

The Python runner defaults to 1/32/256 replicas, all three control cases, 256
warmup ticks and three 64-tick measurement windows. `--sizes`, `--modes`,
`--ticks` and `--warmup` allow bounded controlled reruns. The standalone reference
binary uses the same bootstrap and observation windows as this experiment,
retaining independent sequential CPU worlds. Source is in scratch
`probe/src/bin/`; original main, wheel source, Cargo manifest/lock and previous
result files remain unchanged. Raw `gpu-loop/results-{readback,direct}.json`
contains startup, warmup, all windows and dynamic query evidence. Logs retain
failed constructor/order-check attempts separately from final successful runs.
Only these scratch tests and probes were run; workspace checks would overlap the
parent's implementation and are unnecessary for a standalone report.
