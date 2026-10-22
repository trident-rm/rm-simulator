<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Batched CPU rollouts

`physics::session::batch::RobotBatch` and
`world::session::batch::MatchBatch` own bounded collections of independent
sessions. Their default `step` uses ordinary session commands and stepping,
sequentially on the calling CPU thread. Optional [bounded CPU workers](parallel-rollouts.md)
advance independent sessions with joined per-request outcomes. Each slot keeps its own physical world and, for a match,
its native authority. There is no transport, renderer, policy inference, task,
reward, GPU, worker pool or additional dependency. Batching organizes caller
work and storage; it does not automatically make the physics solver faster.

Choose a [RobotSession or MatchSession](simulation-embedding.md) first.
`RobotBatch::new` accepts owned robot sessions;
`MatchBatch::new` accepts owned complete match sessions. Neither clones a running
world. Slot indices remain stable and the collection cannot grow after creation.
A nonzero `max_sessions` permits an empty collection. The caller selects finite
limits for resident sessions, requests per call, steps per selected session and
queued commands per selected session. Zero step/command limits allow observation
or reset-only use. These limits bound facade work, not arbitrary scenario geometry
or internal solver allocations.

```rust
use rm_simulator_physics::{Pose, StepDuration, Team, chassis::ChassisConfig,
    session::{RobotCommand, RobotScenario, RobotSession,
        batch::{BatchLimits, BatchStep, RobotBatch, RobotQuery}}};

let make = || -> Result<RobotSession, &'static str> {
    RobotSession::new(RobotScenario::new(vec![], 0., StepDuration::HZ_1000, |world| {
        world.add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 0., 0.4]))?;
        Ok(())
    })?)
};
let mut batch = RobotBatch::new(vec![make()?, make()?], BatchLimits {
    max_sessions: 8, max_steps_per_session: 100, max_commands_per_session: 4,
})?;
let mut stamps = Vec::with_capacity(2);
let mut observations = Vec::with_capacity(2);
batch.commands_mut(0).unwrap().push(RobotCommand::Drive {
    robot: 0, command: Default::default(),
});
batch.step(&[BatchStep { session: 0, steps: 10 },
             BatchStep { session: 1, steps: 3 }], &mut stamps)?;
batch.observe_into(&[RobotQuery { session: 0, robot: 0 },
                     RobotQuery { session: 1, robot: 0 }], &mut observations)?;
assert_eq!(observations[0].stamp.time_ns, 10_000_000);
batch.reset(&[0])?;
assert_eq!(batch.sessions()[1].stamp().tick, 3);
# Ok::<(), Box<dyn std::error::Error>>(())
```

Commands are queued per slot and applied once, in order, before that slot's
requested advance. A zero-step request still applies commands. To run a
controller before every physical step, queue commands and request one step in
a caller loop. Commands are moved out of retained vectors, including wheel torque
requests, without cloning their storage. Unselected queues remain pending.
`observe_into` clears and fills a caller-owned vector of compact privileged
controller truth in query order; it does not clone robot models/wheel arrays or
produce sensor measurements. Unknown robot identities produce `state: None`.
Repeated observation queries are allowed within the request count limit.
`RobotBatch::contacts(slot)` exposes ordered raw contacts from that slot's latest
attempted step; untouched slots retain their previous contacts. The standalone
`RobotSession::step_into` appends contacts to reusable caller storage.

## Time and independent episodes

Mixed integration durations are supported. Each `BatchStep::steps` counts the
selected slot's immutable integration steps; it is not a shared batch clock.
Final stamps follow request order, and compact observations also include each
slot's `step_duration`. Ten steps at 1000 Hz mean 10 ms, while ten default
128 Hz steps mean 78.125 ms. Nanoseconds remain authoritative across rates.
Group sessions or split requests across calls without changing each session's
command sequence or integration sequence. This preserves deterministic output;
a controller or recipe using shared mutable external state can break independence.

Prepare expensive geometry once through the existing scenario composition APIs,
then capture cloned `StaticGeometry` in robot recipes and call
`WorldPhysics::insert_geometry`. The cooked collider shapes remain shared.
Match checkpoints already share immutable static geometry across restore/reset.
Each session still owns independent bodies, contacts, clocks and native rules;
mutable solver state is never shared across slots.

Selective resets have a transaction boundary covering the whole selected set:
all replacement worlds are constructed before any live session is swapped.
Failure leaves all sessions, pending queues, contact buffers and clocks unchanged.
Success clears only the selected slots' pending batch queues and robot contact
buffers. This temporarily retains both old and replacement worlds. External setup
callback side effects are outside the transaction; recipes should capture
immutable inputs.

`RobotBatch::reset(&[slot, ...])` reruns initial scenario recipes and restarts
those slots' clocks and physical IDs. It has no implicit random seed or arbitrary
running checkpoint. `MatchBatch::reset(&[MatchReset { session, seed }, ...])`
restores each selected initial checkpoint and seeds its native Rune stream.
An initial match checkpoint captured at a nonzero clock resets to that clock.
`MatchBatch::checkpoint_into` captures selected branch points into a retained
output vector; individual checkpoints still allocate dynamic/rule state.
Ordinary `session_mut(slot).restore(checkpoint)` retains the existing transactional
single-session restore contract and solver-cache limitations.

Task/reward state, episode seeds, controller RNGs and sensor schedules remain
caller-owned. Reset them explicitly for the same selected slots. Sensor rolling
brackets and IMU interpolation must also be reset after placement discontinuities.

## Validation and failures

Before a step changes anything, both facades validate request counts, every slot
index, duplicate slots, queued command counts, per-slot step limits and clock
overflow. Invalid input returns `BatchError::Invalid` and preserves sessions,
queues and caller output. Observation and checkpoint selection validate before
clearing output. Reset rejects duplicate/out-of-range slots before preparing any
replacement. Duplicate detection is a bounded sequential scan in the step path;
reset/checkpoint selection uses bounded temporary storage.

Ordinary command/rule/integration errors can occur after preflight. They return
`BatchError::Runtime(BatchFailure { ... })` and stop immediately. Completed earlier
requests remain committed; later slots and queues remain untouched. The successful
output vector contains exactly that completed prefix. The error includes the
failing request position, slot, number of accepted commands, before/after stamps
and original error. Remaining commands in that failing slot's queue are discarded.
Accepted commands and any partial physical/rule progress remain visible through
ordinary session truth. A timestamp alone does not describe all state changes;
inspect the failing session before deciding to resume or reset. There is no
implicit expensive checkpoint rollback. Transactional reset preparation failures
report zero completed requests because no live replacement has been committed.

## Reproducible probe

Run from the workspace, after compilation and other heavy work have finished:

```sh
nix develop -c env CARGO_TARGET_DIR=target/batched-rollouts \
  cargo run -p rm-simulator-world --example batched_rollouts --release --locked -- 16 128 5
```

Arguments are session count, one-millisecond integration steps per session and
repetitions. The probe validates bounds, warms both code paths and prints CSV
rows with microseconds per session operation. It compares direct per-session
loops with both batch facades for command+step, compact observations and reset;
match rows additionally include checkpoint capture and ordinary per-slot restore.
Both step paths retain final stamp output and compare robot truth exactly.
Match comparisons also check native rule truth. Initialization is outside timers.
Step measurement order alternates between repetitions; observation/reset/checkpoint
order stays sequential first, so their short timings are particularly sensitive
to cache/allocator order. The probe has no CAD or recorded sensor workload.

The robot fixture uses one default chassis, a catch floor and one shared triangle.
The match fixture adds active native RMUL Infantry authority, calls `StartMatch`,
advances its actual five-second countdown, asserts `Running`, and captures the
initial reset checkpoint at tick 5000 (5,000,000,000 ns). Restoring that initial
checkpoint therefore preserves a nonzero clock. Selected-set batch reset retains
all replacements until commit, whereas the direct baseline resets one at a time;
reset compares these deliberately different memory/transaction costs. Bare robot
sessions have no arbitrary running checkpoint, so no checkpoint timing is invented.

### Recorded measurement (2026-10-23)

Talos, AMD Ryzen 9 9950X3D (16 cores / 32 logical CPUs), Linux x86_64,
Rust 1.99.0, ordinary Cargo release profile. Source: base `71b4e33` plus this
batch implementation; source hashes and raw logs are retained in the private
probe directory. No CPU affinity was selected. The five measurement repetitions
used `16 128 5`; the warmup rows are excluded. Compilation had finished, and
one-minute host load was 0.63 before and after the approximately one-second run.
This is a shared host; unrelated load and frequency/cache/allocator effects are
not controlled. These measurements describe this small fixture, not general
throughput for CAD fields, sensors, combat or larger robot counts.

Median microseconds per session operation across the five repetitions:

| Layer / operation | Direct loop | Batch facade |
|---|---:|---:|
| Robot command + one step | 13.760 | 13.775 |
| Robot compact observation | 0.006 | 0.004 |
| Robot reset | 216.444 | 211.046 |
| Active match command + one step | 14.188 | 14.167 |
| Match compact observation | 0.006 | 0.007 |
| Match checkpoint capture | 0.387 | 0.393 |
| Match checkpoint restore | 220.648 | 214.999 |
| Match reset | 213.614 | 207.093 |

Stepping costs are effectively unchanged here. The small reset differences are
sensitive to execution/allocator order and are not evidence that whole-set
transactional reset is inherently faster. Observation differences are only a few
nanoseconds in this cache-resident fixture. These baseline measurements cover
the sequential API only; the [parallel probe](parallel-rollouts.md) separately
measures independent-environment throughput and scoped-thread dispatch costs.

Raw output/check logs are retained at
`local-assets/batched-rollouts-20261023/` in the development checkout and
`/tmp/rm-batch-rollouts-20261023/logs/` on Talos. The private remote source and
Cargo target directories are sibling `source/` and `target/` directories; they
do not reuse the parent's baseline/final verification source or target.

## Opt-in CPU workers

The [parallel rollout guide](parallel-rollouts.md) describes `step_parallel`,
finite worker admission, complete per-request runtime progress, synchronous
selected-set reset and the independent-environment throughput probe. Existing
sequential calls retain their completed-prefix failure behavior.
