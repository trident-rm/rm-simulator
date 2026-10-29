<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Browser startup and memory

Measure the existing staged application on macOS:

```sh
uv run --with playwright python scripts/measure-browser.py \
  --root local-assets/browser --out local-assets/browser-measurements.json
```

Use `--executable` for an installed Chromium. The default runs three fresh browser
processes/profiles for each case: the 1v1 title screen and offline 1v1 with one
robot. Each offline trial starts the round with F5. The browser renders through
WebGPU/Metal; this is an application measurement, not a CPU-only physics benchmark.

Startup milestones use navigation-relative `performance.now()` from observed
app screen changes, field-preview text and pipeline readiness. `InMatch` alone
is a lifecycle milestone, not proof that every pipeline has compiled. Resource
entries record download completion and bytes. Each trial observes another ten
seconds after the requested screen and pipeline readiness. This does not test
long matches, sustained movement, assembly planning or memory leaks.

The sampler reads BSD `ps` every 100 ms and sums RSS for the isolated browser
process and its descendants. It records a baseline before opening the page and
an endpoint breakdown from Chromium's process inventory. RSS can count shared
pages more than once and excludes GPU allocations. It is neither private memory
nor peak physical footprint. Samples can miss shorter peaks. Wasm capacity is
recorded separately and is included within process memory, not added to RSS.

Fresh HTTP caches do not imply cold OS file or GPU driver caches. Local HTTP is
uncompressed and has no Internet latency. The report includes the Wasm SHA-256,
browser version, CPU model, viewport, source checkout commit and raw samples.
The commit identifies the checkout at measurement time; the hash identifies
the actual binary, including when measuring an older staged bundle.

## Local observations, 2026-10-29

Using the browser implementation from `ad5bd88`, Chromium 156 on an Apple M5 Max at
1600 × 1000, three fresh-process trials recorded:

| Case | Screen arrival | Sampled peak Chromium tree RSS | Wasm capacity |
|---|---|---|---|
| Title | 595–614 ms | 1,555–1,561 MiB | 143.9 MiB |
| Offline, one robot | 919–934 ms | 1,584–1,601 MiB | 156.6 MiB |

A separate detailed trial observed the title's field preview ready at 763 ms.
The offline endpoint process breakdown included about 660 MiB for the largest
renderer, 234 MiB for the GPU process and 220 MiB for the browser process.
Other renderer and utility processes account for the rest. The browser baseline
before opening the application was roughly 424–499 MiB across the repeated
trials. Do not treat subtracting that baseline as an exact application footprint.

The Wasm file is 79,968,115 bytes. A local gzip level-6 test produced 17,802,978
bytes; no serving configuration changed. In the first title trial, localhost
finished transferring Wasm at 102 ms, before the first title frame at 614 ms.
The resource timings do not separate compilation, instantiation and Bevy setup.

The next useful work is to serve compressed Wasm and measure over a controlled
network, then profile the largest renderer's code/heap allocations. Most of the
sampled process memory already exists on the title screen. These measurements
do not yet establish a CPU-only simulation budget or a deployment memory limit.

## Gameplay stall investigation, 2026-10-29

A separate local trial held forward movement and firing for eight seconds, then
observed two seconds of settling. Each case used a fresh Chromium process, the
same staged browser binary and viewport as above, `inspect=1`, and the Chrome
CPU sampling profiler. These are single trials, not repeated benchmark medians.

| Field | Prediction | p95 frame interval | p99 frame interval | Long tasks above 50 ms |
|---|---|---|---|---|
| 1v1 | On | 16.7 ms | 16.8 ms | 0 |
| 1v1 | Off | 16.7 ms | 16.8 ms | 0 |
| RMUC | On | 33.4 ms | 49.9 ms | 4 |
| RMUC | Off | 16.7 ms | 16.8 ms | 0 |

The RMUC prediction-on profile contained 6,520 CPU samples. Of these, 3,406
(52%) had chassis `Replayer` in their ancestry, 1,503 (23%) had `Field::restore`,
and 975 (15%) had convex-hull construction. These inclusive categories overlap;
they must not be added. Payload preparation accounted for 864 of the hull
samples through `WorldPhysics::add_payload`. Authoritative caller-driven host
work appeared in 951 samples (15%). Sampling percentages are approximate CPU
attribution, not per-frame durations.

The browser `PredictionWorker::exchange` runs synchronously on the UI thread.
`Replayer::advance_budgeted` restores the complete field whenever its checkpoint
identity changes. The eight-tick limit applies after restoration, so it does not
bound the cost of rebuilding the world or cooking payload hulls. Provisional
projectile prediction also restores complete fields on this thread, although it
was not the principal measured cost in this trial. Native prediction runs on
background threads.

The next fixes should cache immutable cooked payload shapes across restores and
move browser prediction into isolated workers, retaining the complete world and
existing checkpoint/cancellation contracts. Running the offline authority on a
worker is a later option; its measured cost was smaller here. Disabling prediction
is a diagnostic workaround through `?field=rmuc&play=1&prediction=0`, with the
expected loss of local chassis and provisional-shot prediction.

Maximum frame intervals still reached 83 ms with prediction disabled despite no
recorded long tasks. This means not every presentation stall is explained by
main-thread prediction. GPU submission/presentation and shader compilation need
a separate graphics trace if those stalls persist. No leak, garbage-collection
cause or network cause was established by this experiment.

Profiles used names recovered from an otherwise identical wasm-bindgen output;
the named and staged binaries had identical code-section SHA-256
`362c6d9649d46a88220d5460cfc94dfbb318df6334c05c6473e5621de9a2957b`.
