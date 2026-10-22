<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Engine performance checks

Runnable CPU probes and the workloads they measure. Results depend on the
compiler, hardware and asset package. [Measuring a candidate](#measuring-a-candidate)
explains how to compare builds.

See [graphics and visualization measurements](performance-graphics-20261023.md)
for the render-preset comparison, CPU-contention limits and launch controls.

## Engine microbenchmarks

Run the dependency-free examples in release mode:

```sh
cargo run --release -p rm-simulator-world --example step_cost
cargo run --release -p rm-simulator-gameplay --example benchmark
```

The world example compares 20,000 requested ticks over several runs. These
cases use no external CAD terrain; they distinguish empty fast-forward from
referee-only work and active physics, not a loaded match's frame rate.

The gameplay example measures native seven-minute rounds using one or eight
robots. It compares event-driven advancement with 128 Hz and 1000 Hz calls, plus
a synthetic mixed workload with Radar reports, launch attempts and detected hits.
It reports the median and range of five repetitions. Scenario cloning and final
validation are outside timing; no physical travel or projectile flight is simulated.
See [the recorded baseline](performance-20261021.md) for workload details and limits.

## Loaded field and concurrent matches

The server example loads the checksummed collision package and runs independent
matches on separate threads, sharing immutable geometry as prediction does:

```sh
cargo run --release -p rm-simulator-server --example match_cost -- field-2027 1 8 1000
cargo run --release -p rm-simulator-server --example match_cost -- field-2027 4 8 1000
```

Arguments are CAD path, concurrent matches, pilots per match, and sample count.
Each sample advances 16 ordinary ticks; pilots drive and rotate, and fire
17 mm shots once per simulated second per robot. The running-match fixture
seeds each Infantry with 200 rounds through the referee API and records accepted
and rejected launches. The probe reports median, p95, p99 and maximum step,
snapshot and restore time, plus step batches exceeding the correct 125 ms budget
(16 ticks at 128 Hz). Loading and startup
settling happen before sampling. Snapshot/restore measurements are separate from
step samples, but their contention is included in concurrent wall time. This is
an unpaced CPU probe, not a networking or renderer benchmark, nor a guarantee of
production server capacity. Include host/transport overhead and operating-system
headroom before selecting a matches-per-server limit.

## Measuring a candidate

Build before timing, keep the same compiler profile and checksummed field package,
and run each candidate on the target hardware without another build or renderer
competing for CPU. Record median and tail times, loading memory, and the counts
of step batches that exceed the available frame budget. Measure client GPU frame
time separately with the [render benchmark](render-benchmark.md). A headless
server probe does not establish rendering performance or network capacity.

## Navigation geometry and ideal scans

```sh
cargo run --release -p rm-simulator-server --example navigation_query_cost -- field-2027
```

This times cached static CAD segment-visibility queries for 1080 horizontal rays
and 28,800 rays over sixteen elevations, from one fixed FLU origin at 0.35 m.
It reports that boolean early-out workload separately from ideal nearest-hit
LiDAR acquisitions at three clear origins. Nearest scans reuse output storage
and include distances, normals and points. Both exclude robot motion, dynamic
obstacles, self filtering, sensor noise and output encoding.
