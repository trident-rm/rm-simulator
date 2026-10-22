<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Rolling LiDAR broad-phase pass, 2026-10-23

Baseline: `c53cc96aa18703e7ee4d36db9c49c450754d4db9`. This pass changes
`sensors/rolling.rs` only, with a standalone `sensor_pass_cost` example for
reproducible complete-acquisition measurements. It does not change integration,
rules, collision meshes, the sensor pattern or public APIs.

## Implementation

Fixed colliders now have a sensor-owned Parry BVH over their world AABBs.
Leaves still call the original exact Rapier shape/triangle query. The immutable
acceleration is shared through `Arc`/`OnceLock` from one capture to the next
only after validating collider handles, motion channels, shared shape identity,
fixed poses and capture order. Changed ordering remains accepted but gets a
separate cache, preserving capture-local shape indices. Cloning a populated
capture shares acceleration. Half-spaces bypass the BVH to avoid infinite-bound
construction arithmetic.

Traversal is inclusive at maximum range. Exact equal-distance hits select the
lowest fixed shape index, independently of tree visitation order; fixed geometry
still wins ties against moving geometry. Moving-collider interpolation, swept
bounds, self-occlusion, sensor mounting, timestamp selection, global-index noise
and discontinuity/topology rejection remain on their existing paths. The bracket
borrows the resolved fixed-query cache, avoiding a cache check on each emission.
The old extra vector of fixed `(pose, shared shape)` pairs is also removed.
There is no new dependency, unsafe code, backend or fidelity change.

This is a collider-level broad phase, not packet traversal inside mesh BVHs.
Moving swept bounds are still constructed per bracket and tested per emission.
No physics/world lower-level API changes were needed.

## Correctness

New regressions compare the accelerated path against the original linear fixed
query plus a brute-force interpolated moving query. They compare complete
`RayHit` records exactly, including normals and shape indices, and full measured
ranges/local points after seeded angular noise, range noise and dropout. The
fixtures include 48 oriented boxes, an exact triangle mesh, the catch half-space,
stationary and translated/rotated scenes, mounts inside/above the chassis, and
strides 1, 3, 19 and 257 over 100 ms brackets. A separate moving source-fitted
`team_cad` chassis fixture verifies self-occlusion against the same reference.
This fixture uses the existing source-fitted physics model, not a new full robot
CAD mesh benchmark. Tie, inside-solid, inclusive range-edge, reordered capture
and shared-cache continuation checks are explicit. Existing rolling,
synchronized, source-motion, restore, discontinuity and sensor tests also pass.

The loaded-field probe hashes every output's index, timestamp, miss marker,
range and local point floating-point bits, outside the measured acquisition
interval. All paired baseline/candidate hashes match exactly. The probe retains
the original 200,000 emissions/s, 128 Hz capture cadence, seeded range effects,
self geometry, loaded field, warmup and commanded drive/yaw from
`mounted_lidar_cost`. Acquisition timing includes bracket validation, cache
construction/lookup, interpolation, ray queries, noise and output writes; loading,
physics, capture and hashing are separately excluded. Capture/physics timings are
reported too.

## Measurement environment and reproduction

Talos: AMD Ryzen 9 9950X3D, 32 logical CPUs, Linux, Rust 1.99.0
(`b940084d7`, 2026-09-28), repository Nix development environment, release
optimization with `CARGO_PROFILE_RELEASE_DEBUG=1`. Private source and target:
`/tmp/rm-sensors-pass-33af22c6/src` and
`/tmp/rm-sensors-pass-33af22c6/target`. The baseline is a fresh `git archive`
of the stated commit with the identical probe added. Separate baseline/candidate
executables are copied before timing; no other worker's target is used.

The canonical field `/home/hxyulin/.cache/rm-simulator-arena/field-2027` was
read-only throughout. Its manifest SHA-256 is
`8a92e1d6721be2e214cd48251fb6d9b2730c16f0df7116407dbfe819f9972586`;
`rmuc2027-bindings.json` is
`a6b7e958cae2646aa8698bb0a1143948319992d326a6475c7099d2327cff54e4`.
The ordinary verified loader checks package checksums. Loaded collision counts:
187,661 ground + 94,274 fixture + 165,950 equipment = 447,885 triangles.

Build outside the benchmark lock, separately for each source:

```sh
nix develop -c env \
  CARGO_TARGET_DIR=/tmp/rm-sensors-pass-33af22c6/target \
  CARGO_BUILD_JOBS=6 CARGO_PROFILE_RELEASE_DEBUG=1 \
  cargo build --release --locked -p rm-simulator-server --example sensor_pass_cost
```

Run inside the same native library environment, with
`LD_LIBRARY_PATH` containing the Nix protobuf/OpenSSL libdirs and the directory
of `gcc -print-file-name=libstdc++.so.6`. The measurement script holds
`flock` on `/tmp/rm-perf-pass-20261023.lock` for the entire repeated run and
executes each binary with `taskset -c 4`, using the same CPU and scene:

```sh
exec 9>/tmp/rm-perf-pass-20261023.lock
flock 9
# Alternate baseline/candidate; reverse the middle pair's order.
taskset -c 4 ./baseline /home/hxyulin/.cache/rm-simulator-arena/field-2027 10
taskset -c 4 ./candidate /home/hxyulin/.cache/rm-simulator-arena/field-2027 10
```

The guard requires one-minute system load below 12 before each run on this
32-thread host. Builds finish before measurement and never hold the timing
lock. This limits contention, but does not reserve a CPU, stop unrelated jobs,
isolate SMT siblings, or fix CPU frequency. Results are a scoped modest
improvement, not an uncontended whole-simulator throughput claim.

## Results and validation

Final-source hashes and completed scoped validation are recorded below.

Input/source SHA-256:

- Final `sensors/rolling.rs`:
  `06eabeae6d2e958dda9b7bea8c346d64d5475ca94f515ae528010fe7b51910bd`.
- Identical baseline/candidate `sensor_pass_cost.rs`:
  `20772193c4f4f5f54128ec14f1d429fc9f931d363c9cf9386d870115e8b0a4e5`.
- Baseline executable:
  `b3c4aacd5a0132160843e4ffecfb3813961b743634e7fd7bd5a97a278a527182`.
- Final candidate executable:
  `1b56d43c21ded88c7c04ce3233d331612355460edd4bdc0228b719dbf69d2699`.

Validation commands, all exit 0:

```sh
cargo clippy --release -p rm-simulator-physics --all-targets --locked -- -D warnings
cargo test --release -p rm-simulator-physics --locked
cargo clippy --release -p rm-simulator-server --example sensor_pass_cost --locked -- -D warnings
cargo fmt --all -- --check
python3 scripts/check-module-dependencies.py
git diff --check
```

Linux release tests: 146 unit tests, 11 integration tests and 82 doctests pass.
The same physics suite, all-target Clippy and formatting also pass locally on
macOS in the ordinary development profile. The module-boundary check passes.
Workspace aggregate verification belongs to the integrating parent; this report
does not claim `just verify` was run.

Raw logs, scripts and executables remain in
`/tmp/rm-sensors-pass-33af22c6/` on Talos. Text logs are also retained in the
worktree's ignored `local-assets/sensors-pass-20261023/` directory. An initial
three-pair run before the final per-bracket cache borrow measured acquisition
reductions of 1.74%, 1.75% and 1.82%, with identical output hashes. Final timings
below use the final source and binary hashes above.

Final three paired 10-second acquisitions (milliseconds; acquisition includes all
2,000,000 emissions and produces 1,041,310 returns in each run):

1. Baseline 3,100.737; candidate 3,066.026: 1.12% less acquisition time.
2. Baseline 3,168.854; candidate 3,062.911: 3.34% less acquisition time
   (candidate ran first in this pair).
3. Baseline 3,142.684; candidate 3,008.144: 4.28% less acquisition time.

The median acquisition changes from 3,142.684 to 3,062.911 ms: 2.54% less
acquisition time, or 2.60% higher acquisition throughput. Capture totals are
2.136–2.153 ms baseline and 2.038–2.047 ms candidate. Physics totals are
488.956–492.682 ms baseline and 482.148–488.461 ms candidate; no physics
speedup is inferred. Every full-rate output hash is `eb9ac2da54fe1702`.
A supplemental one-second stride-3 pair emits 66,667 points / 34,760 returns;
both hashes are `588954be48939611` (113.452 / 110.961 ms acquisition).
This supplemental pair is a compatibility check, not a repeated speed estimate.

Final-run one-minute loads were 3.86–4.41, below the guard of 12; process logs
show external compiler/linker jobs during these measurements. Our own builds
were complete and all timed probes held the shared lock, but that lock does not
prevent other workers from building. The consistent modest improvement and
eliminated repeated fixed-bound/vector construction support this focused patch;
these samples do not isolate the exact percentage causally from external
contention. The current ten-second trajectory settles against nearby geometry
(final chassis position approximately `[0.129, -0.057, 0.303]` m), so this is not
a fast-motion distortion benchmark. Moving-source and interpolation correctness
are established by differential regressions instead. Internal mesh ray traversal
is unchanged, and larger performance claims require another profiling/API pass.
