<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Bounded parallel CPU rollouts

`RobotBatch::step_parallel` and `MatchBatch::step_parallel` opt into bounded
standard-library CPU workers. The [sequential batch API](batched-rollouts.md)
remains available. Each slot keeps its own persistent solver, clock, immutable
integration duration, commands and native authority. Workers improve throughput
across independent environments; they do not parallelize one physical world.
There are no new dependencies, GPU requirements or changes to live match rules.

```rust
use rm_simulator_world::{Field, FieldConfig, session::{MatchSession,
    batch::{MatchBatch, BatchLimits, BatchStep, parallel::CpuWorkers}}};

let sessions = (0..8).map(|_| Field::new(&FieldConfig::default())
    .map(MatchSession::new)).collect::<Result<Vec<_>, _>>()?;
let mut batch = MatchBatch::new(sessions, BatchLimits {
    max_sessions: 8,
    max_steps_per_session: 128,
    max_commands_per_session: 4,
})?;
let requests = (0..8).map(|session| BatchStep { session, steps: 128 })
    .collect::<Vec<_>>();
let report = batch.step_parallel(CpuWorkers::new(4)?, &requests)?;
assert!(report.all_succeeded());
assert!(report.outcomes.iter().all(|item| item.after.time_ns == 1_000_000_000));
# Ok::<(), Box<dyn std::error::Error>>(())
```

## Admission, ownership and backpressure

`CpuWorkers::new` accepts 1 through 256 workers. One executes inline; larger
counts create at most the smaller of worker count and selected request count.
All ordinary batch bounds, duplicate/index checks, command counts, step limits
and clock overflows are checked before any session or queue changes.
Admission failure returns `BatchError::Invalid` without starting work.

A call borrows the batch exclusively and runs synchronously. There is no queue
or overlapping call to admit: returning waits for all scoped threads. The caller
provides backpressure by waiting for that result before submitting another call.
Threads borrow disjoint sessions; no mutable solver state is shared. Sessions
remain owned by the batch, with ordinary `session_mut` access between operations.
Task/reward state, controllers, sensors, sampling schedules and RNGs stay external.

The shared runner partitions selected slots into contiguous chunks in slot order.
There is no work stealing or CPU affinity. Mixed rates/step counts remain valid,
but uneven per-slot workloads can leave workers idle. Threads are created per
call, while sessions and solver state persist. Small one-step calls may cost more
than sequential stepping; longer rollouts amortize startup. A persistent pool
would need a separate ownership/access contract and is not provided here.

## Runtime failures and reset

The returned `ParallelReport` is marked `must_use`. `Ok(report)` means admission
succeeded and all attempts joined; callers must check `report.all_succeeded()`
or inspect every outcome for runtime errors. Each outcome records request/slot,
before/after stamps, accepted command count and the ordinary session result.
Results retain original request order regardless of completion order.

Every selected slot attempts its commands and steps even if another slot fails.
There is no completed-prefix claim or cross-worker cancellation. Within a slot,
commands retain order, stop at the first error and discard remaining commands.
Accepted commands and partial integration remain committed. A failed command may
change state without advancing the timestamp; inspect ordinary truth as needed.
Untouched slots retain queues and contacts. Physics slots expose ordered raw
contacts from their latest attempt as before. There is no work running after a
returned report or admission error. Panics propagate after threads are joined;
there is no panic-to-error conversion or promised report after a panic.

Selective reset continues through the existing synchronous `reset` operation.
It prepares all replacements before committing any selected slot and preserves
selected-set atomicity. Reset is never distributed across workers. Match
checkpoint capture/restore, ordinary observations and direct sensor acquisition
remain synchronous between stepping calls. Reset caller sensor/controller state
separately, as with ordinary sessions.

## Throughput probe

After compilation and other heavy work finish:

```sh
nix develop -c env CARGO_TARGET_DIR=target/parallel-rollouts \
  cargo run -p rm-simulator-world --example batched_rollouts --release --locked \
  -- --parallel 64 128 7
```

Repeat with `--parallel 64 1 7` and `--parallel 64 8 7` to expose
short-action dispatch overhead as well as amortized work.

Arguments are environments, 1 ms steps per environment per call and measured
repetitions. Repetition zero warms every path and should be excluded. The probe
compares the sequential batch baseline (`workers=0`) with 1/2/4/8 workers, and
alternates execution order each repetition. Construction, 256 warmup ticks per environment and queued-command
preparation are outside timers. Warming avoids charging the first solver step
after construction/restore to a one-tick action sample. Timers include scoped thread creation/join,
command application, integration, output allocation and ordered reporting.
One Drive command holds 0.5 m/s through each rollout; no per-step policy callback
is invented. CSV includes batch microseconds, microseconds per environment and
aggregate environment steps per second. These measure independent environment
throughput, never one-world latency improvement.

Both fixtures reuse the sequential probe's default chassis, catch floor and
shared static triangle. Projectile policy is the ordinary default, with no
projectiles or sensors. The match fixture has active native RMUL Infantry
authority after its actual five-second countdown; initial time is 5 s, followed by the same 256 warmup ticks. After
each worker count, the probe checks exact final robot truth and native authority
against the first execution of that repetition. Initialization is excluded;
command/control physics and native rule stepping remain included. No CAD,
recorded sensor workload, renderer or GPU is used. Results are specific to this
small fixture and host conditions.

## Recorded measurements (2026-10-23)

Talos, AMD Ryzen 9 9950X3D, 16 physical cores / 32 logical CPUs, Linux x86_64,
Rust 1.99.0, ordinary release profile. Source is base `a4b06ff` plus the worker
implementation and concurrent telemetry foundation captured in the private source
snapshot; `logs/source.sha256` records the exact tested files. All compilation
finished before timing. No CPU affinity was selected. An unrelated single-core
compiler remained active on this shared host; this was not an isolated quiet-host
run. One-minute load was 1.49 before and 1.84 after the 64-environment suite.
Process/load records are retained with the CSV. Seven measured repetitions per
configuration exclude repetition zero and alternate execution order.

Median **batch microseconds**; columns are sequential batch and worker counts.
The one-worker path runs inline, retaining parallel job/report allocation costs.

| Environments | Layer | Ticks per slot | Sequential | 1 | 2 | 4 | 8 |
|---:|---|---:|---:|---:|---:|---:|---:|
| 8 | robot | 1 | 209.2 | 210.4 | 321.1 | 214.0 | 219.2 |
| 8 | match | 1 | 264.7 | 288.9 | 386.5 | 265.8 | 246.5 |
| 8 | robot | 8 | 887.6 | 890.1 | 808.6 | 566.7 | 399.0 |
| 8 | match | 8 | 1069.0 | 1049.2 | 998.0 | 691.0 | 488.0 |
| 8 | robot | 128 | 12066.1 | 12104.6 | 6902.4 | 3834.6 | 2325.3 |
| 8 | match | 128 | 12645.2 | 12711.9 | 7528.4 | 3838.8 | 2494.8 |
| 64 | robot | 1 | 3434.2 | 3420.3 | 2156.4 | 1234.8 | 757.5 |
| 64 | match | 1 | 3885.4 | 3881.8 | 2446.5 | 1446.8 | 890.1 |
| 64 | robot | 8 | 8539.7 | 8621.6 | 5124.7 | 3077.6 | 1770.4 |
| 64 | match | 8 | 9924.2 | 9935.0 | 5965.9 | 3475.9 | 2152.1 |
| 64 | robot | 128 | 97624.8 | 97720.3 | 51741.0 | 26346.9 | 13587.4 |
| 64 | match | 128 | 101757.4 | 101834.9 | 53526.4 | 27636.0 | 14420.1 |

Divide a batch value by its environment count for microseconds per environment;
raw CSV also prints that value and aggregate environment steps per second. For
64 environments, eight workers take 11.84/13.91 µs per environment for a one-tick
robot/match action, 27.66/33.63 µs for eight ticks, and 212.30/225.31 µs for 128
ticks. At 128 ticks this is 7.18×/7.06× the sequential batch throughput.

Eight environments expose dispatch overhead: one-tick actions show no useful
consistent gain (0.95× robot / 1.07× match at eight workers), and two workers
are slower than sequential in these samples. At eight ticks, eight workers give
2.22×/2.19×; at 128 ticks, 5.19×/5.07×. With 64 environments, one-tick actions
still have enough independent work for 4.53×/4.37×. These observations support
explicit worker/chunk selection for the caller's action cadence; they do not
establish a universal crossover or a need for a persistent pool. Short calls are
particularly sensitive to cache working set, scheduling and allocation. Sessions
persist, **threads do not**. There is no persistent-worker throughput claim.

Every fixture/configuration passed exact final robot truth and native-authority
equality across sequential/1/2/4/8 execution. This small no-CAD fixture does not
predict performance with larger scenes, sensors, policy inference or combat.

Raw CSV, initial cold-start exploratory samples, load/process records, source
hashes and release build logs are retained under
`local-assets/parallel-rollouts-agent-20261023/` locally and
`/home/hxyulin/.cache/rm-parallel-workers-agent-20261023/logs/` on Talos. The
remote private `source/`, `target/` and `tmp/` directories are siblings of `logs/`;
no parent verification source or target was reused. `summary.json` locally
contains per-batch/per-environment medians and speedup for every configuration.
