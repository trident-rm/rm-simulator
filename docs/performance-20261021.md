<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Core baseline, 2026-10-21

Measured locally on Apple M5 Max, arm64, Rust 1.99.0, release optimization.
Base revision: `939316e`, plus the benchmark-example changes recorded alongside
this report. Benchmarks ran sequentially, without concurrent builds. No GPU,
network transport or real-time pacing was included.

Field manifest SHA-256:
`8a92e1d6721be2e214cd48251fb6d9b2730c16f0df7116407dbfe819f9972586`.
The loader counted 187,661 ground, 94,274 fixture and 165,950 equipment triangles
(447,885 placed collision triangles in total).

Raw reports, source hashes and platform metadata are in the gitignored
`local-assets/core-benchmark-20261021/`. Reproduce with the examples in
[performance checks](performance.md); these numbers are observations, not targets
or guarantees for another machine.

## Standalone game rules

Five repetitions per case. Event-driven cases run 100 rounds per repetition;
explicit-step cases run one round per repetition. Each round is 420 seconds of
running match time. Fixture setup, cloning and final validation are outside the
timed region. Timed results include stepping and retaining the resulting match.

The eight-robot roster has one Heavy, two Infantry and one Sentry per team, with
synthetic own-side control-point observations. This is a rules workload, not a
full physical match, strategy policy or search tree.

| Workload | Median wall time per round | Approximate simulated-time / wall-time |
|---|---:|---:|
| One robot, event-driven | 0.889 ms | 472,444× |
| Eight robots, event-driven | 2.649 ms | 158,525× |
| Eight robots, 128 Hz | 32.558 ms | 12,900× |
| Eight robots, 1000 Hz | 239.173 ms | 1,756× |
| Eight robots, mixed inputs at 128 Hz | 36.491 ms | 11,510× |

The fixed-step cases cost about 0.57–0.68 microseconds per advance call. The mixed
case injects accurate Radar reports and attempts launches about every 100 ms,
plus repeated damage observations. Native allowance, heat, defeat and Heavy
permission checks remain active; attempts are not all successful. It produced
228 positive damage events per round. Event-driven speed does not imply that
an agent can make 158,525 decisions per real-time second: decisions and physical
observation generation are outside that case.

The rules engine is inexpensive relative to the physical simulation below.
Optimizing it further is a lower priority than sensors, physics and the cost of
many concurrent independent environments.

## Physical simulation

Bare-world probe: best of five, 20,000 steps at 128 Hz. One flat-ground chassis
took 1,365.778 ms, or 68.3 microseconds per step: about 114× real time. The empty
field fast-forwards and is not a meaningful integration-throughput measurement.

Loaded-field probe: one match, eight Infantry, all driving and rotating, with
1,000 samples of sixteen steps each. The fixture seeds 200 rounds per robot;
all 1,000 launch attempts succeeded. Load/setup took about 2 seconds and was
excluded from the concurrent-run timer. Over 125 simulated seconds:

| Operation | Median | p95 | p99 | Maximum |
|---|---:|---:|---:|---:|
| Sixteen simulation steps | 18.056 ms | 36.766 ms | 51.766 ms | 65.791 ms |
| Snapshot capture | 0.004 ms | 0.007 ms | 0.011 ms | 0.013 ms |
| Field reconstruction | 1.475 ms | 1.537 ms | 1.537 ms | 1.537 ms |

The run took 20.236 wall seconds, approximately **6.2× real time**, including
periodic snapshots and occasional separate restore probes. No step batch exceeded
its 125 ms simulated-time budget. Restore has only sixteen samples, so its tail
percentiles have low resolution. This is not an FPS result or a capacity claim
for multiple concurrent matches.

The old probe printed 16 ms and 0.016 simulated seconds for a sixteen-tick batch;
those labels were incorrect at 128 Hz. They are now derived from `tick_ns()`.
The updated fixture also starts the match explicitly and reports accepted shots
so an unarmed run cannot silently be presented as a projectile workload.

## Static visibility-query proxy

One fixed origin, 30 m segments, cached shape AABBs and triangle BVHs; median of
seven scans after one warmup scan. This uses `segment_clear`, which exits on any
obstruction, rather than finding the nearest return.

| Query workload | Median | Maximum |
|---|---:|---:|
| 1080 planar visibility queries | 0.072 ms | 0.072 ms |
| 16 × 1800 visibility queries | 9.650 ms | 9.738 ms |

These are useful initial CPU-budget observations, but do not establish LiDAR
sensor performance. A nearest-hit implementation must search for the closest
surface, resolve robot exclusions and moving geometry, and produce timestamped
returns. Scanning from more viewpoints and measuring that implementation is the
next meaningful experiment.

## Implemented nearest-hit LiDAR

The subsequent passive sensor implementation was measured separately, with
prepared directions and reused output storage. All three origins were checked
clear of collision geometry. Each result is the median of seven acquisitions
after warmup, over the same static CAD triangles:

| Sensor origin, FLU metres | 1080 planar rays | 16 × 1800 rays |
|---|---:|---:|
| (0, 0, 0.35) | 0.652 ms | 21.622 ms |
| (4.7, 3.5, 0.35) | 0.441 ms | 14.259 ms |
| (-6.7, -3.5, 0.35) | 0.359 ms | 10.862 ms |

The observed maximum for the largest case was 22.329 ms. At a nominal 10 Hz,
these single-scanner multi-layer workloads consume approximately 11–22% of one
CPU core before dynamic-scene capture, noise, encoding and navigation. This is
not a worst-case bound. The planar case is much cheaper, and is the appropriate
first mounted navigation sensor. Raw output is `nearest-lidar.txt` in the same
scratch directory; metadata records its own source and executable hashes.

See [navigation sensors](navigation-sensors.md) for exact return semantics and
the remaining dynamic-scene/self-filtering work. This measures actual nearest
returns, not the boolean visibility proxy above.
