<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Gameplay and in-process rollout costs, 2026-10-23

The RMUC adapter no longer clones its robot roster twice per tick or builds an
intermediate control observation vector. Terrain contact storage and adapter
card scratch are reused. Gameplay activation arbitration masks its eligibility
vector in place instead of collecting a second vector. Rules, tick ordering,
physics and checkpoint formats are unchanged. No GPU backend or PPO integration
is added.

## Scope and evidence

Shared checkout based on `a721350`, including preexisting uncommitted RMUL work
and concurrent work in other scopes. Apple M5 Max, 18 logical CPUs, release
profile, through `nix develop -c`. Builds used two jobs and completed before each
probe. Our probes ran sequentially; other processes remained active. System
load observations fell from approximately 64 to 26 during the investigation.
These are contention-affected observations, **not controlled speedup claims**.
No full-app graphics result is inferred from these CPU probes.

Raw output is in ignored `local-assets/gameplay-performance-20261023/`.
The three original source files are retained there as `*.before.rs`.

Allocation/copy changes established by source paths and storage tests:

- `Match::resolve`: one eligibility allocation instead of eligibility plus
  resolved vectors, for nonempty rosters. A normal running `advance_ns` calls
  resolve at interval start and endpoint; each split interval also resolves.
  One ordinary unsplit call therefore eliminates two temporary vectors.
- `Terrain::observe`: previously `cards.to_vec()` allocated for each nonempty
  observation; now `contacts` retains capacity, including across empty contact
  observations. New-card detection still reads the old contacts before mutation,
  and sorting/deduplication preserve the canonical stored representation.
- `Field::advance_2027`: two deep roster clones and one collected observation
  vector are removed per tick. A deep roster clone includes each robot's
  nonempty terrain contacts and pending delivery vectors. The reused adapter
  scratch replaces one temporary card vector per robot. At sufficient retained
  capacity these adapter operations require no new vector allocations.
- The new private scratch buffer is excluded from snapshots. Construction and
  restore start with empty scratch, with the same resulting authority; it is
  neither persistent rule state nor a second observation cache.

These are source-level allocation eliminations, not global allocator counts.
Physics, native rule advancement and snapshot capture still allocate. The tests
check retained pointers/capacities and semantics, without introducing an unsafe
counting allocator or new dependencies.

## Rules-only measurements

The existing gameplay example measures five repetitions of 420-second rounds.
Fixture creation/cloning and final validation stay outside its timer. The
128 Hz fixture has 53,760 advance calls; the 1000 Hz fixture has 420,000.
Before/after median wall times per round:

- One robot, event driven: 1.335 / 0.918 ms.
- Eight robots, event driven: 3.223 / 3.070 ms.
- Eight robots, 128 Hz: 37.746 / 33.998 ms (702 / 632 ns per advance).
- Eight robots, 1000 Hz: 275.681 / 254.189 ms (656 / 605 ns per advance).
- Eight robots, mixed 128 Hz: 45.193 / 38.331 ms (841 / 713 ns per advance).
  Both produced 228 positive damage events per round.

These rules accept caller-supplied observations; they do not simulate movement,
projectile flight or sensor acquisition. Event-driven throughput does not measure
policy decisions per second. The after eight-robot 128 Hz result corresponds to
about 1.58 million advance calls per wall second in this synthetic case, excluding
policy inference, observation production and reset.

## Physical Field, checkpoint and reset measurements

The new `rollout_cost` example separates stepping from checkpoint capture and
reset. It uses an explicitly synthetic flat-ground field with valid RMUC
structure bindings and overlapping base/supply footprints. Robots are stationary
Infantry with ordinary physical bodies, native authority and RFID observations.
There is no CAD, firing, sensor, transport, renderer or training policy. Each
case starts after the five-second countdown, advances 1,024 individual 128 Hz
ticks, captures 200 checkpoints and performs 32 episode resets. Setup is untimed.
Each timing is one aggregate sample, not a percentile distribution.

Before/after microseconds per operation:

- No robots: step 0.102 / 0.098; checkpoint 0.636 / 0.845;
  reset 5.836 / 15.639.
- One robot: step 87.159 / 97.124; checkpoint 0.951 / 1.108;
  reset 204.547 / 244.944.
- Eight robots: step 764.560 / 849.290; checkpoint 3.234 / 3.751;
  reset 1,649.020 / 1,922.220.

No physical speedup is established. Timings of unchanged checkpoint/reset paths
also deteriorated. Final full snapshot Debug hashes (DefaultHasher, same compiler
and fixture) matched before/after: `73ee6b81ade05a43` for zero robots,
`f4fc46d299400300` for one, and `543f0c761b8d4061` for eight. These are local
comparison checksums, not a stable serialization contract or a solver rollback
guarantee. Separate regression tests cover restore, clock partitioning and
scratch-independent rule equivalence.

The preexisting bare `step_cost` baseline measured 90.6 us/tick for one chassis
(best of five, 20,000 ticks). It was not rerun after the changes: the optimized
adapter is absent from that bare world. For a moving, shooting CAD match use
`server/examples/match_cost.rs`; the 2026-10-21 report measured approximately
6.2x real time for its eight-robot workload. That historical observation cannot
be substituted for a current quiet-machine comparison.

Reproduction (build before timing):

```sh
nix develop -c cargo build --release --locked -j 2 \
  -p rm-simulator-gameplay --example benchmark \
  -p rm-simulator-world --example rollout_cost
nix develop -c target/nix/release/examples/benchmark
nix develop -c target/nix/release/examples/rollout_cost
```

## Embedding and remaining priorities

1. Profile the actual physical rollout and sensors on a quiet machine first.
   The rules-only sub-microsecond calls and physical hundreds-of-microseconds
   ticks measure different work. Preserve Rapier contact and wheel semantics;
   physics optimization belongs in the physics crate.
2. Use bounded CPU workers for independent environments, each owning a Field,
   sharing immutable StaticGeometry. Benchmark scaling at small worker counts
   while retaining headroom for inference, rendering and the OS. No parallel
   environment capacity was established under this contention.
3. Prefer borrowed `Session::ground_truth()` / `Field::rmuc_2027()` for privileged
   compact observations and `robot_truth()` for addressed physical state.
   `Session::step` does not capture a checkpoint per tick. Call checkpoint only
   at real branch/reset boundaries. Robot truth still builds a physical chassis
   snapshot, so it is not allocation-free. Sensor observations must remain
   separate from privileged truth.
4. `Field::snapshot` currently clones native authority/configuration into both
   presentation and restore records; checkpoints inherit that duplication.
   Removing it requires preserving existing equality, serialization and transport
   compatibility. Sharing immutable configuration or a purpose-built compact
   observation API is a follow-up, not part of this change. Episode reset rebuilds
   Rapier and intentionally does not promise warm-start-identical rollback.
5. `Session::step_controlled` reuses commands within a batch but starts a new
   command Vec each call. Persistent/caller-owned command storage could help
   adapters that call it once per tick; measure that caller pattern first.

CPU environment parallelism is the first implementation path for a future PPO
adapter. Policy inference can be batched on a GPU independently of simulation.
A GPU rules port would duplicate branch-heavy integer timers and mutable rule
state to accelerate a comparatively cheap stage, with transfer/batching costs
and substantial deterministic-equivalence work. Full physical GPU stepping would
need a different validated physics backend. GPU ray queries may be worth measuring
for large sensor batches, but are not a drop-in change to gameplay or Field.
No GPU benefit or crossover batch size is established by these measurements.

## Validation

`nix develop -c cargo test --release --locked -j 2 -p rm-simulator-gameplay
-p rm-simulator-world` passed 90 gameplay unit tests, 69 world unit tests,
one integration test and 40 doctests (200 total). This includes new contact
normalization/storage tests and scratch-independent checkpoint equivalence,
plus existing native rule partition/restore tests. The debug test request was
cancelled while waiting for another scope's shared build lock; release tests
used the already-warmed profile.

`nix develop -c cargo clippy --release --locked -j 2 -p rm-simulator-gameplay
-p rm-simulator-world --all-targets -- -D warnings` passed. Scoped rustfmt and
`git diff --check` passed. No changes were staged or committed.
