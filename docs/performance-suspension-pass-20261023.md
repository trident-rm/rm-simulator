<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Suspension query investigation, 2026-10-23

No production optimization is retained from this pass. An exact early-filtering
trial improves the one-robot flat fixture slightly, but mixed eight-robot flat
and loaded results do not establish a reliable broader gain. The original
suspension query remains unchanged. No timing, integration slices, equations,
contact geometry, self/projectile exclusion, snapshots, sensor or public API
changes are made.

## Baseline and isolation

The worktree starts at `c53cc96aa18703e7ee4d36db9c49c450754d4db9` and first
incorporates the existing foundations, in the requested order:

- `65f006d45430940ede1b2fbe851d17f6a75ba535` (fixed rolling LiDAR broad phase).
- `2dee81ce333a23e9b2f096b22446b1b5645d7c31` (contact buffers).
- `90a0bc2c9b8fd5dea7fea35e5c8b0e56983f56e0` (wheel angle wrapping).

The resulting local baseline is `edb3ed5ba2a8a41d9e39f14071fa5727006dc971`.
These are foundations, not new patches to integrate again. The parent checkout's
uncommitted arena/arm extractions were read for context, not copied or modified.
The parent owns integration and the full workspace gate.

Talos is an AMD Ryzen 9 9950X3D, x86_64 Linux. Compiler: Rust 1.99.0
(`b940084d7`), LLVM 23.1.1. Source and target are private to this pass:
`/tmp/rm-suspension-pass-20261023/source` and its `target/nix/` directory.
The Nix shell sets that target path; exports for subsequent builds occur inside
the shell. Builds use locked dependencies, `CARGO_PROFILE_RELEASE_DEBUG=1` and
`CARGO_BUILD_JOBS=4`, outside the timing lock. No shared target is modified.

Every timing/profile series takes exclusive
`flock /tmp/rm-perf-pass-20261023.lock`; this pass's own builds finish first.
Both variants use CPU 6 affinity and the same native library environment.
Other host activity continues, CPU 6 and its SMT sibling are not reserved, and
frequency is not fixed. One-minute load is 8.38 before and 5.87 after the paired
series. This is not an idle-host causal speedup measurement. No processes are
killed or system profiling settings changed.

The canonical `/home/hxyulin/.cache/rm-simulator-arena/field-2027` is read-only.
The loader verifies its checksums. Both variants load 447,885 placed collision
triangles: 187,661 ground, 94,274 fixtures and 165,950 equipment. Verified robot
packages are copied into this pass's private `robots/` directory. Their source
ownership/terms remain in their manifests; none are added to Git.

Raw reports, scripts, rejected sources and hashes are under the private remote
directory above and ignored `local-assets/suspension-pass-20261023/` locally.
Binaries and the 71.5 MB perf capture remain remote. These scratch locations are
not durable release artifacts or runtime dependencies.

## What the suspension calls actually do

The earlier [wheel pass](performance-chassis-pass-20261023.md) measured support
casts plus normal acceptance at about 89% of its instrumented chassis adapter
window. That is a fraction of the adapter, not the whole physical match.
Each grounded four-wheel chassis casts four rays per solver slice, retaining all
eight slices per default tick. The adapter's scalar wheel equations are much
cheaper than those repeated exact scene queries.

Inspection of the locked Rapier 0.35.3 / Parry 0.30.2 sources separates the work:

1. `PhysicsWorld::query_pipeline_with_filter` borrows the existing broad-phase
   tree, body/collider sets and dispatcher into a small `Copy` view. Constructing
   it neither builds nor refits acceleration. Persisting this wrapper does not
   eliminate an acceleration update.
2. `QueryPipeline` traverses the top-level collider BVH with `find_best` and
   ray/AABB bounds. At reached leaves it resolves the current collider generation
   and runs `QueryFilter::test`. This call site has no custom predicate: it excludes
   the addressed rigid body and projectile group 2, preserving all other groups.
3. Accepted leaves perform exact shape casts at current collider poses. Trimeshes
   traverse their own persistent triangle BVHs, compute exact triangle hits and
   return normals. The adapter then applies its original normal orientation and
   support-angle acceptance. It does not search through a rejected closest wall
   for another support surface.
4. Acceleration maintenance belongs to `PhysicsWorld::step` / Rapier's physics
   pipeline, separately from these read-only queries. Moving/folding colliders
   and the solver continue to update there. A persistent suspension query wrapper
   would not remove this work. Mesh BVHs are already retained; their per-ray
   traversal is not a mesh rebuild. Parry traversal uses an inline 32-entry
   SmallVec stack, not an unconditional heap allocation per cast.

A temporary diagnostic reproduces the original `find_best` query, with timers
around node tests, filtering and exact shape casts. Its loaded worker window is
1,048,576 support rays (4,096 ticks × eight slices × eight robots × four wheels):

- 56,605,542 top-level node tests: **53.98 per ray**.
- 2,144,561 accepted and 6,549,483 rejected candidate filter checks:
  **8.29 checks per ray**, of which **6.25 reject**.
- 2,097,152 exact trimesh casts: **2.00 per ray**; another 47,409 shape casts.
- Instrumented cumulative query time 3.522 s; node-test intervals 1.338 s,
  filtering intervals 0.166 s, exact trimesh intervals 0.632 s and other exact
  shape intervals 0.00582 s. Remaining time includes traversal, lookup, control
  flow and instrumentation overhead.

Per-node clocks substantially perturb this small operation. Those intervals
must not be interpreted as uninstrumented exclusive shares or added to the
previous pass's 89% figure. Counts are the stronger attribution evidence:
there is substantial top-level traversal/filter traffic as well as repeated
triangle traversal; query-object construction and rebuilding static acceleration
are not the explanation. All diagnostic loaded checkpoint hashes match baseline.

A separate baseline process profile without query timers at 499 Hz retained 8,888
user-cycle samples with zero lost samples. Whole-process self shares include
7.43% ray/AABB tests, 1.25% trimesh `find_best`, 0.59% QueryPipeline `find_best`,
11.18% point-cloud AABB calculation, 10.05% narrow-phase pair updates and 7.77%
contact solving. Broad-phase update-related traversal symbols are also present
(e.g. two-branch traversal 0.77%, single-subtree traversal 0.25%); these individual
self shares do not sum to its complete inclusive cost. Optimized stack unwinding
remains incomplete. The profile includes loading, warmup, firing, checkpoint
hashing and restore; it cannot assign every shared ray/AABB sample to suspension,
or rank all acceleration updates against wheel support precisely.

## Reuse and batching assessment

The current solver already preserves both collider and triangle acceleration.
Holding a query view across mutable solver steps is neither needed nor an
appropriate cache. Caching the previous support hit alone is insufficient:
a moving support, closer obstacle, newly inserted collider, changed shape,
projectile-group change, folding pose or placement discontinuity can change the
nearest hit while the previous triangle still intersects the ray.

A conservative candidate-region cache would need to cover the complete current
segments, preserve current collider generations, shapes, poses and filtering,
and invalidate on scene changes. The chassis adapter has no complete scene
revision contract for these changes. Rechecking a cached hit can supply a search
hint, but cannot replace searching for closer hits; exact boundary/tie behavior
also needs preservation. No temporal hit/candidate cache is introduced.

Gathering candidates once for a four-ray bounding region can share broad-phase
work, but includes geometry between the rays and loses the existing per-ray
nearest-hit pruning. Reordering exact casts may change equal-distance normal
selection. That is a prospective benchmark with a tie-preserving fallback, not
an established win in this pass. No new revision API, geometry representation,
backend or second query world is added to pursue it.

## Rejected early-filter trial and matched timing

The measured trial keeps Rapier's `find_best` traversal and exact shape casts,
but checks eligible leaf metadata before the leaf's ray/AABB test. Excluded
leaves return the rejected bound immediately. It repeats no solver work and
retains current generation lookup, self/group exclusions, solid casts, closest
hit and normal acceptance. It adds metadata/filter work for leaves a ray would
otherwise reject geometrically, so a benefit is workload dependent.

Five pairs alternate baseline/candidate order. The flat workload is the existing
`rollout_cost`. The loaded workload is `match_cost FIELD 1 8 256`, with its
500-tick post-countdown warmup, held drive/yaw, native rules and 256 accepted
shots. Both loaded binaries have the same temporary addition: hash a full
checkpoint after each 16-tick batch. Hashing is outside each step interval but
inside reported wall time. These wall times therefore differ from an unmodified
`match_cost` probe and should be compared only within this series.

- One-robot flat, microseconds/tick: baseline
  `106.772, 106.098, 106.126, 105.628, 105.629`; candidate
  `105.317, 105.385, 105.308, 105.042, 104.987`.
  All pairs improve 0.55–1.36%; medians 106.098 and 105.308.
- Eight-robot flat, microseconds/tick: baseline
  `900.091, 847.536, 868.668, 856.983, 847.563`; candidate
  `954.473, 909.659, 853.491, 844.423, 843.459`.
  Two pairs regress 6.04% and 7.33%; three improve 0.48–1.75%.
  Medians 856.983 and 853.491 conceal this inconsistency.
- Loaded wall seconds: baseline `9.153, 9.002, 9.017, 9.017, 8.978`;
  candidate `9.018, 9.019, 8.984, 8.972, 8.985`.
  Paired changes range from 0.19% slower to 1.47% faster; medians 9.017 and
  8.985 s. Across-run median 16-tick batch times are 29.083 and 29.000 ms.

The repeatable small one-robot effect does not resolve the inconsistent
multi-robot result or establish a loaded-field benefit under this host load.
The trial is removed rather than adding a specialized traversal implementation
on that evidence. This is not proof that every exact caching/batching approach
would fail, nor a claim that fewer slices or approximate surfaces are necessary.

## Compatibility checks and exact comparison hashes

All comparisons use the same compiler, inputs and foundations. Snapshot probes
hash their complete Rust Debug representations with DefaultHasher, then SHA-256
the emitted comparison stream. This is an exact same-build comparison aid, not
a portable serialization contract or exhaustive correctness proof.

- Six reference trajectories (omni, folding Hero, low grip, falling, direct
  torque, balance) have identical emitted component bits between baseline and
  trial. Combined SHA-256:
  `0991a4a5011b8abfa8830e4e10f407adaf239c9a7cb194c9b7c307c8f3be80b4`.
  Individual line SHA-256 values in that order are:
  `539edde094067ad5905af2bf5a8c629f60c91b9efb0216de0e70221dea82dc72`,
  `06ccec9b8c361344c030bde5ddedb77949c41750b7255fa52e53b64dfbdd5f40`,
  `c48f618150d15b8c62aea438ca877a30a5022bd67a62cfa195959d8630dd6d18`,
  `a65d3a9d597e3a522bb29821b8c11378579389a19ebd204a89195445ea267dc1`,
  `032125cdbe3c7a9e4affb0890149a26bcd3fa17920ec7c8bb2b8a4bfa275eafa`,
  `fc581fb434bb8492cb793908785074c05a1c16fb460b0a53e4c5834cacb9d92d`.
- Every flat repetition preserves zero/one/eight-robot final hashes
  `ff67615fbedcff6a`, `dfbce533323534c1`, `41132527cbbda422`.
- Every baseline/trial loaded repetition and the diagnostic build match all
  256 checkpoint hashes. SHA-256 of the 256-line stream:
  `6d8646fdaba0882499e49e71aa9a5a227d07221fd834a95b45cdccd9f2d1f3f2`.
  The loaded probe begins from a whole-Field restore and exercises further
  temporary restores; solver warm starts retain their existing restore limits.
- Three verified source-backed robots each run 1,024 explicit ticks with driving,
  yaw/aim and fold/unfold requests. All per-tick truth hashes match; combined
  output SHA-256:
  `bb1c59fa62aca225833af930c5207c6d5bbac4512091183870bf2b34555677e7`.
  Artisans Hero reaches 2.2165681500327987 rad; PIE Heavy reaches 1 rad.
  Future Infantry remains at zero fold, so this fixture does not validate its
  folded traversal. These are flat physical sessions, not loaded-CAD navigation
  or source-link inertial validation.
- The private differential test compares collider identity, TOI bits, normal
  bits and feature IDs for 131,072 deterministic rays. Its final version includes
  a sloped trimesh, roof and side walls, tied surfaces, moving supports,
  self colliders and a projectile. Interleaved changes replace a shape, change
  groups, remove/reinsert a collider and relocate the excluded body. All match
  the original query. No cache is added, so no cache invalidation claim is made.
- The final trial test build passes 149 physics unit tests, 11 integration tests
  and all example harnesses; physics all-target release Clippy passes with
  warnings denied. These checks concern the rejected trial. The final production
  source is byte-identical to baseline. No full workspace gate is claimed.

## Source identity and reproduction

Key SHA-256 identities (full binary/probe records are in `inputs.sha256`):

- Unchanged/final `chassis/backend/dynamics.rs`:
  `66aa307cd77a270cef95faaa66436f8c8ae8d3de8b02cef9d5ff4a578079833f`.
- Timed trial `candidate-dynamics.rs`:
  `3eb153eafeb9a2b3e7f24f2b3c8bb686afe199e802ac98cfd4941cf9c2fff23b`.
- Trial with final test-only tunnel-wall and Clippy fixes:
  `ba0e6fe428edb9cea1d1336dc2ca232ab4acbed4cf0611002aae959701527f8d`.
- `Cargo.lock`:
  `52199d56cdbd6c2ef2c680408a8af47e5014e30b361a64445309741babc23628`.
- Canonical field manifest:
  `8a92e1d6721be2e214cd48251fb6d9b2730c16f0df7116407dbfe819f9972586`.
- Common loaded hash probe `match_cost.rs`:
  `3e7cfbbcaa429cbaabc186ddff94ba250f1e85854adf0e3c432aa47bc4f53894`.
- Unchanged `rollout_cost.rs`:
  `b80dbb573ccac0ef41ed5ff54af348c6c63a4dcbfb01266344d335a19b099db5`.
- Private `source_folding.rs`:
  `970845961e34d22699f9aecb540f0614a9c3684a90749bcc136cf8c21a24f975`.
- Private `diagnostic-fn.rs`:
  `646e6db4ddc2c7abd4d10b932963fbb5f200b08f3ed195f641b48163de9d91cd`.

Verified robot descriptor identities, Hero / PIE Heavy / Future Infantry:
`7550362e5aca65812d50dbf140267c55829ddcbbce1f6b6a23bf3dd132e63cdb`,
`e0c2996e9616a6e3a249b8cb17b086433d33d05b641bb8a3d0ef79c907a7fc83`,
`0cdc262cb421a645dec24f4f9389e2777868e0f8e3d68341dea42c2a1a958867`.

Inside `nix develop`, builds use:

```sh
export CARGO_TARGET_DIR=/tmp/rm-suspension-pass-20261023/source/target/nix
export CARGO_PROFILE_RELEASE_DEBUG=1 CARGO_BUILD_JOBS=4
cargo build --release --locked -p rm-simulator-server --example match_cost \
  -p rm-simulator-world --example rollout_cost
cargo test --release --locked -p rm-simulator-physics --all-targets
cargo clippy --release --locked -p rm-simulator-physics --all-targets -- -D warnings
```

`prepare.py` applies the identical temporary checkpoint/trajectory instrumentation.
`pairs.sh` preserves both binaries, alternating order, lock, affinity and native
library paths. `followup.sh` / `resume.sh` build and run source robot comparisons,
scoped tests and the diagnostic after paired timing. `summarize.py` asserts all
comparison hashes and records all five pairs. `profile.sh` records the separate
baseline profile using `perf record -e cycles:u -F 499 --call-graph dwarf,8192`.
The rejected helper and probes remain scratch; only this report is the new patch.
