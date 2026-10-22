<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Chassis performance pass, 2026-10-23

This pass starts at `c53cc96aa18703e7ee4d36db9c49c450754d4db9` and changes only
angle wrapping in the wheel model. The eight internal
motor integration slices, suspension queries, force ordering, motor power and
traction equations, geometry, explicit clocks, checkpoints and sensor paths are
preserved. No dependency or backend changes are involved.

## Choice of optimization

A fresh loaded eight-robot profile confirmed the earlier contact/collision-heavy
shape. A 512-batch process profile at 499 Hz recorded 8,496 samples without lost
samples. Point-cloud AABB self samples were 11.54%, narrow-phase pair update
10.51%, contact solve 8.33%, and ray/AABB tests 8.04%. Wheel integration accounted
for 0.52% and the chassis force adapter for 0.40% of self samples. These include
loading, warmup, firing, checkpoint and restore, and incomplete optimized stack
unwinding prevents assigning every ray/AABB sample to suspension support.

The isolated wheel integration probe exposed a smaller, safe opportunity:
`fmod` and its implementation together accounted for about 10.5% of self samples.
Every wheel spin update previously invoked a remainder, including when the
shifted angle was already between zero and one turn. The fast path skips that
identity remainder. It retains the original addition and subtraction, including
their rounding, and retains the original remainder for boundaries, large angles
and non-finite inputs. The retained candidate leaves gimbal wrapping unchanged.

## Reproduction and isolation

Talos is an AMD Ryzen 9 9950X3D Linux host with 32 logical CPUs. This pass uses
private source `/tmp/rm-chassis-pass-20261023/source` and target
`/tmp/rm-chassis-pass-20261023/target`; it does not use another worker's target.
The canonical `/home/hxyulin/.cache/rm-simulator-arena/field-2027` is read-only.
Raw reports and probe scripts live in that private directory and are copied to
ignored `local-assets/chassis-pass-20261023/` in this worktree.

Builds use the repository's `nix develop` environment, locked dependencies,
`CARGO_PROFILE_RELEASE_DEBUG=1`, and `CARGO_BUILD_JOBS=4`. Build commands run
outside the benchmark lock. Timed runs take an exclusive
`flock /tmp/rm-perf-pass-20261023.lock`; this pass finishes its own builds first.
Other workers' builds may run outside that lock. The first paired series requires
one-minute system load below 8 at admission; it has no CPU affinity/frequency
isolation, and this is not a fully idle-host experiment. No processes are killed.

The baseline and candidate binaries are saved separately before timing. Five
pairs alternate baseline/candidate execution order. Each pair runs the same
four-wheel motor microprobe (2,000,000 integrations with 1 ms adapter duration,
held 2 m/s target, 1 m/s contact speed, 50 N grip and a zero budget every third
integration, otherwise 120 W), the existing flat `rollout_cost`, and the loaded
`match_cost FIELD 1 8 256`. The latter uses the existing eight-robot drive/firing
scene, 500-tick post-countdown warmup and 256 batches of sixteen ticks. Native
library paths come from the same Nix environment for both variants. Input and
binary SHA-256 records are in `inputs.sha256`; the temporary microprobe source
is retained with the raw reports, not added to the repository.

## Rejected first candidate

The first candidate shared the fast helper with gimbal wrapping. Across five
alternating, unpinned pairs, wheel integration fell from 232.8–239.2 ns to
203.6–215.0 ns (8.5–14.7% faster per pair). Flat one/eight-robot timings varied
in both directions. Loaded match wall times were consistently worse: baseline
8.355–8.656 s and candidate 8.611–9.164 s, or 3.1–5.9% slower per pair. That
candidate was not accepted. The next candidate keeps the original gimbal helper
and uses CPU 6 affinity for both variants. The change of implementation and CPU
control means these series do not isolate the reason for the earlier slowdown.

All first-candidate compatibility checks passed: six trajectories compared
bit-for-bit with the baseline, and every flat repetition retained hashes
`ff67615fbedcff6a`, `dfbce533323534c1`, `41132527cbbda422` for zero/one/eight
robots respectively. Correctness alone did not justify retaining the slower
loaded candidate.

The measured inputs were:

- `Cargo.lock`: `52199d56cdbd6c2ef2c680408a8af47e5014e30b361a64445309741babc23628`.
- Field manifest: `8a92e1d6721be2e214cd48251fb6d9b2730c16f0df7116407dbfe819f9972586`.
- `rollout_cost.rs`: `b80dbb573ccac0ef41ed5ff54af348c6c63a4dcbfb01266344d335a19b099db5`.
- `match_cost.rs`: `f68691cd0126b2b70bbcfc79886a3ba7b3202c2a4b56e9dce6decc5b2f5dfbd3`.
- Temporary `wheel_cost.rs`: `44681a18d3fd8410513d1077648bbd2f465a6abeb16afecc8e685545b3d3004d`.

The field loader verifies the package's manifest checksums; neither variant
changes or regenerates field inputs. Both loaded variants report 187,661 ground,
94,274 fixture and 165,950 equipment triangles, or 447,885 placed triangles.

## Suspension attribution

A separate, temporary instrumented build times each support cast plus its normal
acceptance, the motor integration, and the complete chassis force adapter. It
runs the same flat and loaded probes under the shared lock. Instrumentation is
absent from both measured variants and the committed implementation. Timer
calls add overhead, so these numbers identify the work rather than establish
uninstrumented exclusive CPU costs.

In the loaded cumulative window of 327,680 chassis slices, complete adapter time
was 1.325 s, support casts/normal acceptance 1.179 s (89.0%), and motor integration
0.0660 s (5.0%): approximately 4.044, 3.598 and 0.201 microseconds per chassis
slice respectively. The four support queries therefore dominate the adapter.
The flat cumulative window of 65,536 slices measured 88.4 ms total, 63.9 ms
queries and 9.31 ms motor integration (72.2% and 10.5%). These cumulative windows
include fixture settling and warmup; flat windows can span different robot
counts. No query implementation, filtering, normal acceptance, support count or
substep duration is changed by this pass.

## Retained wheel-only candidate

The final change is confined to `wheel_model.rs`: a range check bypasses identity
remainders for wheel spin only, plus a regression against the original
expression over more than 130,000 boundary, ordinary and arbitrary-bit inputs.
Gimbal code remains at the baseline. The tested file SHA-256 is
`f705a8ecde9bf622eb8af5a94f10061a41a7397406e3fc6bc2befc272201d60c`.
Compiler: Rust 1.99.0 (`b940084d7`), LLVM 23.1.1, x86_64 Linux.

Five more alternating pairs use `taskset -c 6` for every baseline and candidate
probe. Results and binary hashes are under `round2/` in the raw report directory.
Own builds again finish before taking the shared timing lock. One-minute system load is 5.50 before and 5.52 after this series. Other host work
continues; affinity constrains these processes without reserving that CPU or
its SMT sibling, and frequency is not fixed.

- Wheel integration baseline: 238.93, 257.87, 257.27, 240.55, 240.70 ns;
  candidate: 212.76, 213.74, 213.60, 214.92, 222.70 ns. Paired improvements
  are 10.95%, 17.11%, 16.97%, 10.65%, 7.48%; medians are 240.70 and 213.74 ns.
- One-robot flat rollout baseline median: 106.002 microseconds/tick;
  candidate: 105.564. Every pair improves by 0.26–0.90%.
- Eight-robot flat rollout baseline median: 854.231 microseconds/tick;
  candidate: 841.097. Paired changes range from 0.05% slower to 2.90% faster.
- Loaded match wall times baseline: 8.886, 8.960, 8.944, 8.928, 9.008 s;
  candidate: 8.920, 9.049, 8.936, 8.934, 9.034 s. Paired changes range from
  0.09% faster to 0.99% slower; four of five are nominally slower. Median
  16-tick batch costs across runs are 29.101 and 29.058 ms. These samples
  establish no loaded-match speedup; they also do not reproduce the first
  candidate's larger slowdown. All runs accept 256 shots with no rejected
  shots or batches exceeding 125 ms.

The optimization is retained for the repeatable wheel-model and small flat
rollout gain. Its limited loaded-scene effect is consistent with integration
being a small fraction of that workload; this is not a claim of a whole-match
throughput gain. The microprobe uses a fixed four-wheel model and does not
measure support queries, source-CAD articulation workloads or training speed.
The next substantial chassis target remains exact suspension query work, whose
cost should be measured separately from projectile/CCD and contact solving.

## Validation and commands

The final source passes:

- Six existing trajectory fixtures, including omni, folding Hero, low grip,
  falling, direct torque and balance, compared bit-for-bit to a baseline build.
  A temporary test print records component bits in `six-{baseline,candidate}.bits`;
  `six.diff` is empty. The print is removed before final checks.
- All five paired flat runs retain the same three snapshot hashes listed above.
  The 2,000,000-integration microprobe also has identical final wheel state.
- Physics all-target tests: 144 unit tests and 11 integration tests passed;
  example test harnesses passed. Physics doctests: 82 passed.
- Physics all-target release Clippy with warnings denied passed.
- Workspace formatting check and local `git diff --check` passed.

Commands run inside `nix develop`, with the private target and release debug
information enabled, were:

```sh
export CARGO_TARGET_DIR=/tmp/rm-chassis-pass-20261023/target
export CARGO_PROFILE_RELEASE_DEBUG=1
export CARGO_BUILD_JOBS=4
cargo build --release --locked \
  -p rm-simulator-server --example match_cost \
  -p rm-simulator-world --example rollout_cost \
  -p rm-simulator-physics --example wheel_cost
cargo test --release --locked -p rm-simulator-physics --lib \
  wheel_adapter_preserves_reference_trajectories -- --nocapture
cargo test --release --locked -p rm-simulator-physics --all-targets
cargo test --release --locked -p rm-simulator-physics --doc
cargo clippy --release --locked -p rm-simulator-physics --all-targets -- -D warnings
cargo fmt --all -- --check
```

The temporary `wheel_cost` example is copied from the raw report into the private
source for building and removed before the final all-target and format checks.
`round2.sh` contains the exact lock, affinity, library environment and five-pair
invocations; `checks2.sh` contains the baseline comparison and scoped checks.
All mutable files and targets belong to this pass. Full workspace integration
verification and the common changelog are left to the parent task as requested.
