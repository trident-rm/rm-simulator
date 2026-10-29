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
