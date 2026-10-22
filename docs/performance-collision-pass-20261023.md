<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Collision/contact pass, 2026-10-23

Baseline: `c53cc96aa18703e7ee4d36db9c49c450754d4db9`. This independent pass
changes only `physics::projectile` contact collection and its private tests.
It does not change geometry, collision groups, solver settings, eight integration
slices, timing, prescribed motion, restore state or sensor acquisition.

## Caller attribution before implementation

An independent baseline release build on Talos (Ryzen 9 9950X3D, 32 logical CPUs,
Linux, rustc 1.99.0 `b940084d7`, LLVM 23.1.1) used
`CARGO_PROFILE_RELEASE_DEBUG=1 RUSTFLAGS=-Cforce-frame-pointers=yes`.
Frame pointers resolved the optimized caller that the previous DWARF profile
could not unwind. The loaded eight-robot `match_cost` ran 512 sixteen-tick
batches, with the ordinary countdown, 500-tick warmup, firing and restore probes.
`perf record -e cycles:u -F 499 --call-graph fp` recorded 8,650 match samples
without lost samples. The workload ran as the ordinary user under a privileged
profiler; no profiling settings were changed.

Point-cloud AABB calculation had 11.07% whole-process self weight; narrow-phase
`process_pair` had 10.15%, ray/AABB tests 8.34%, contact-constraint solve 8.10%,
and GJK closest-points 4.47%. Of 967
samples whose stacks contained that routine, 849 came through
`PhysicsPipeline::advance_to_final_positions` /
`Collider::compute_collision_aabb`, 115 through triangle-mesh contact manifolds,
and three through other callers. Weighted cycle estimates for those groups were
9.165, 1.207 and 0.033 billion respectively. The measured worker alone had 798
such samples (736 final-position bounds, 59 manifolds, three other), accounting
for 10.96% of its sampled cycles. The main/setup thread had 169 such samples,
including countdown and warmup stepping. This establishes recurring stepping
work rather than attributing the hotspot to package loading or query seeding.

All 700 stacks containing the hot `Aabb::cast_local_ray` routine passed through
`Chassis::apply_forces` and the world query pipeline (632 on the measured worker,
68 during setup/warmup). This ray/AABB hotspot is wheel support in this fixture,
not projectile CCD or sensor acquisition. Chassis and sensor implementations
were left to their respective passes.

The locked Parry 0.30.2 source transforms every convex-polyhedron vertex for
`ConvexPolyhedron::aabb`; Rapier 0.35.3 computes collision bounds while propagating
active bodies to their final collider positions. The default chassis deck and
bumper prisms use convex hulls. Mesh contact generation separately computes a
shape bound in mesh coordinates, retaining its interference workspace. These
are different poses/coordinate systems; caching one `StaticGeometry` capture's
world bounds cannot replace either calculation. `prime_queries` runs once per
world, captured clearance bounds are already cached, and unchanged scenery
poses already skip `set_position`.

No safe world-level bound cache or interaction filter was established. A future
experiment could investigate exact convex support/bounds evaluation inside
Parry, retaining the same hull vertices and exact bounds, with independent
equivalence tests. Replacing hulls with boxes, changing solver/contact-recycling
settings or suppressing supported interactions is outside this pass.

A separate scoped counter run inspected the same scene at world ticks 2,048
and 4,096. The contact graph held 1,236 and 1,232 candidate pairs respectively,
of which 33 and 52 had active contacts. Convex-hull/mesh pairs accounted for
1,136 candidates at both samples (20 and 41 active); cuboid/mesh pairs accounted
for 80 (seven and nine active). The rest involved balls. No sampled pair joined
colliders on the same rigid body. These bounds-overlapping but currently inactive
pairs are potential future contacts, not proven irrelevant interactions; this
does not justify filtering them away.

## Implemented storage reuse

Previously each substep allocated a new list whenever a projectile touched a
scorer or chassis armor touched a non-projectile collider. The preceding list
remained available to suppress repeated contact starts, then was discarded.
The implementation fills one retained scratch list and swaps it with the old
latch only after classification. It preserves duplicates, encounter order,
projectile ordering and the original per-chassis latch shared by its armor
plates. Empty lists are cleared before reuse, including between different owners.
The scratch is derived storage and is not checkpointed.

Projectile hits now append directly to the containing tick's output instead of
building, copying and discarding an intermediate vector on each hit-bearing
slice. Public interfaces are unchanged. Storage can grow with contact topology
and can be released when a robot/projectile leaves; there is no claim of zero
allocation for every possible scene.

The counter run also measured populated touching-list lengths and retained
scratch capacity exhaustion for the 32-second measured continuation, resetting
after template warmup. The original fresh lists would require **147,754 vector
growth allocations**; retained lists reached capacity **315 times**, a 99.79%
reduction for these lists. The original count follows Rust's `Vec<ColliderHandle>`
growth policy (minimum four entries, doubling on exhaustion) applied to each
observed list length; candidate counters increment immediately before a push
at full capacity. This is scoped operation instrumentation, not a global
allocator trace or allocation-byte measurement. It excludes intermediate hit
vectors, all other allocations, and template construction. Topology changes
and projectile retirement can require additional pool growth.
Instrumentation was absent from both timing binaries and restored out of the
private source before final validation.

## Reproduction and validation

Private Talos source, target and results live under
`/tmp/rm-collision-pass-20261023-d91cb568/{source,target,results}`. The canonical
`/home/hxyulin/.cache/rm-simulator-arena/field-2027` was read only. No other worker's
targets or processes were used. Ignored local copies of scripts and reports live
under `local-assets/collision-pass-20261023/` in this worktree.

Builds ran outside `/tmp/rm-perf-pass-20261023.lock`; all measured/profiled runs
held that shared `flock`. Unprofiled runs used `taskset -c 4`, five alternating
baseline/candidate pairs, one eight-robot world and 256 sixteen-tick batches per
run. Each run advances 32 simulated seconds. Before each run the script waited
for no `rustc` process and at most eight runnable tasks, with a five-minute
bounded wait. Per-run uptime, runnable counts and process lists are retained.
There was no frequency isolation, exclusive CPU reservation or guarantee against
new background work after each check. Profile overhead is excluded from the
paired measurements.

Both builds used the same native Nix environment, Rust compiler, Cargo.lock,
frame-pointer flag, field and benchmark input sequence. The field loader verified
the package's checksums. Input SHA-256 values:

- Field manifest: `8a92e1d6721be2e214cd48251fb6d9b2730c16f0df7116407dbfe819f9972586`.
- Cargo.lock: `52199d56cdbd6c2ef2c680408a8af47e5014e30b361a64445309741babc23628`.
- `match_cost.rs`: `f68691cd0126b2b70bbcfc79886a3ba7b3202c2a4b56e9dce6decc5b2f5dfbd3`.
- `rollout_cost.rs`: `b80dbb573ccac0ef41ed5ff54af348c6c63a4dcbfb01266344d335a19b099db5`.
- Temporary `match_parity.rs`: `598d3a4b1edbbeb75cf15f5d606243685187babf0850acc3ff2106db063c2e38`.

The temporary parity probe is the existing `match_cost` with a `DefaultHasher`
hash of the complete debug-formatted `FieldSnapshot` after every sixteen ticks.
It is built separately and excluded from timing comparisons. Full binary hashes,
compiler identification, commands, scripts and logs are retained in results.

Commands, within `nix develop` and with the private target directory:

```sh
export CARGO_PROFILE_RELEASE_DEBUG=1 CARGO_BUILD_JOBS=4
export RUSTFLAGS=-Cforce-frame-pointers=yes
cargo build --release --locked -p rm-simulator-server --example match_cost \
  --example match_parity -p rm-simulator-world --example rollout_cost
cargo clippy --release --locked -p rm-simulator-physics --all-targets -- -D warnings
cargo test --release --locked -p rm-simulator-physics
cargo test --release --locked -p rm-simulator-physics --all-targets
cargo test --release --locked -p rm-simulator-physics --doc
cargo fmt -p rm-simulator-physics -- --check
taskset -c 4 "$VERSION/match_cost" "$FIELD" 1 8 256
taskset -c 4 "$VERSION/match_parity" "$FIELD" 1 8 256
taskset -c 4 "$VERSION/rollout_cost"
```

Scoped Clippy passed with warnings denied. Physics tests passed: 144 unit tests,
three projectile-retirement tests, eight robot-session tests and 82 doctests.
The explicit doctest command also passed all 82. Formatting passed in the local
workspace and for the remote physics package. The initial remote workspace-wide
format request flagged the temporary parity example's import formatting; tracked
source formatting was clean. Aggregate workspace verification belongs to the
parent integration pass.

After removing counter instrumentation, the explicit all-target test command
passed the same 155 unit/integration tests and all six example test harnesses;
all-target Clippy and scoped formatting passed again. Remote checked source
SHA-256 values exactly matched the committed implementation files:
`aecbe8f7651283ffe4e55c913d0d5f713b35d9a9cfe23625a9bcfd85559908d9`
(`projectile.rs`) and
`f8c639f2ee56ccff6e5d740fb2be18ffb74633f74bb44822b3e772c4604be0be`
(`projectile/contact_tests.rs`).

The new regression exercises stale scratch from other owners, fresh versus
retained storage, simultaneous projectiles, chassis armor contact, separation
and recontact. It compares ordered hits, raw armor events, projectile states,
chassis states and preceding-contact latches, independently reconstructs the
final armor touching lists from the contact graph, and verifies that repeated
unchanged collections retain the same buffer allocation pool.

The loaded source-backed match parity probe matched baseline at **all 256
checkpoints**, covering the entire 4,096-tick measured continuation after restore,
including native authority state and contacts recorded in the snapshots. The
final complete snapshot hash was `21e8457487d7c6ea` for both binaries. The flat
`rollout_cost` fixture also retained all three exact final snapshot hashes:
`ff67615fbedcff6a` (zero robots), `dfbce533323534c1` (one), and
`41132527cbbda422` (eight). These are exact hash comparisons, without numerical
tolerances; they are not claims that solver caches survive a restore.
The complete 256-line checkpoint-hash outputs both had SHA-256
`9a5b2058e73056dff04dbe0984382194bc2c557c5f752dbe73a32084207981ad`.

## Paired loaded-field timings

Five paired wall times in seconds (baseline → candidate):

- Pair 1: 8.886 → 8.875.
- Pair 2: 8.912 → 8.874.
- Pair 3: 8.903 → 9.157.
- Pair 4: 8.859 → 8.818.
- Pair 5: 8.844 → 8.892.

Median wall time was 8.886 → 8.875 s, a 0.12% decrease. Paired changes ranged
from a 0.46% decrease to a 2.85% increase. Median sixteen-tick batch cost across
runs was 28.859 → 28.897 ms; median batch p95 was 71.466 → 71.529 ms. Throughput
ranges were 459.6–463.1 → 447.3–464.5 world ticks/s. All runs accepted 256 shots,
rejected none, and had no batch over the 125 ms budget. The one-minute load
averages recorded around the runs ranged from 2.50 to 4.15 on 32 logical CPUs.
These results establish **no whole-match throughput or tail-latency improvement**.
The implementation is a contact-storage allocation reduction, not a solution to
the dominant AABB/narrow-phase cost.
