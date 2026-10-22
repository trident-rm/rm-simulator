<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Graphics and visualization costs: 2026-10-23

Use Low with bloom disabled and a modest render cap for debug visualization.
Keep higher quality available for driving and explicitly configured vision
experiments. Graphics alone do not explain the slow full-app observations in
this investigation: the workstation became CPU-saturated during measurement.

## Rendering-only comparison

The existing benchmark loaded the approved `field-2027/` visuals, normal
geometry and venue, at 1280 × 720, with camera `[10, 5, 6]` looking at
`[0, 0, 0.3]` FLU metres and 60° vertical FOV. There were no robots, physics,
networking or gameplay UI. VSync was disabled. Six settings cases each ran in
two fresh processes in shuffled order (seed 22), with six seconds warmup and
eight seconds sampling. All twelve runs completed successfully.

Machine: Apple M5 Max, 18 logical CPUs, macOS/Metal, AC power. This used the
development profile: workspace opt-level 1, dependency opt-level 3, Bevy 0.20.
The build includes the uncommitted RMUL work and launch controls over
`0da0d37`; raw reports retain binary and asset hashes. These are not release
performance claims. Metal GPU timestamp measurements are unsupported by this
benchmark; the times below measure end-to-end render-loop wall time, including
CPU scheduling and GPU backpressure, rather than isolated GPU duration.

| Setting | Mean of two frame-time medians (ms) | Individual medians (ms) |
|---|---:|---:|
| High | 2.91 | 2.81–3.00 |
| Low (bloom retained) | 2.87 | 2.39–3.34 |
| Low, bloom off | 2.26 | 2.20–2.31 |
| Ultra | 4.20 | 3.92–4.48 |
| High, shadows off | 2.42 | 2.42–2.43 |
| High, MSAA 1× | 2.93 | 2.86–3.00 |

Shadows contributed approximately 0.4–0.6 ms in this view. Disabling MSAA alone
showed no reliable gain at this resolution. Ultra cost materially more than
High. The Low result varied substantially, so its nominal preset alone does
not establish a consistent improvement. Low with bloom disabled was more
consistent at about 2.2–2.3 ms. The armor-light appearance changes without bloom.
Resolution, view and geometry can change these conclusions.

## Full app and contention

A separate app probe used the same camera position/direction and field, first empty, then
with the verified EGA Sentry spinning, paused, and recording. The app used its
normal 70° FOV rather than the benchmark’s 60°, so the two executables do not
measure identical rendered coverage. It used ordinary
host commands with no client prediction and no referee. The native sweep
controller can suppress chassis turning near obstacles; chassis motion was
not identical across these runs, which further limits comparison. App frame times are
medians of the existing diagnostic window averages, not raw per-frame tails.
Process CPU time includes the embedded host and all app threads; 100% means one
logical CPU core.

The early uncapped High run measured 5.87 ms empty, 9.28 ms with the spinning
Sentry, and 8.76 ms paused. A later High spinning run reached 22.08 ms.
Low-without-bloom measurements also deteriorated as other jobs took over the
machine. A system observation recorded load average 38.17 and 0% CPU idle on
18 logical CPUs, with unrelated build/test processes active. Memory remained
available and the OS reported no thermal warning. The full-app preset comparison
was stopped rather than treating the reversed ranking as a graphics result.
Pausing or recording measurements under this changing load cannot establish
causal physics or capture overhead either. The extra app cost needs a quiet,
same-view profile before attributing it to UI, scene synchronization or physics.

An additional Low/no-bloom run requested a 30 FPS headless cap. Its spinning
phase consumed about 104% process CPU, compared with 369% in the earlier
uncapped High phase; their different background loads prevent a controlled
speedup claim. The world tick clock advanced at roughly real time in both.
The capped recording produced 214 distinct captures over 8.60 seconds (24.9/s),
with 45 duplicated output frames and no encoder queue drops or readback-limit
skips. A requested 30 FPS limit is not a guarantee under contention, and a
30 FPS encoded video is not evidence of 30 distinct observations each second.

Raw reports, screenshots, isolated app settings and capture metadata remain in
ignored `local-assets/graphics-20261022/`. The original sweep and app scripts
are retained there. User graphics preferences were unchanged in every completed
probe. No persistent quality default was changed.

## Using the controls

```sh
just run --fly --window-mode headless \
  --graphics-preset low --no-bloom --headless-fps 30
```

`--graphics-preset` selects a clean Low/Medium/High/Ultra preset for this launch.
`--no-bloom` is an independent temporary override. Unrelated preference writes
preserve the previously saved graphics; an intentional graphics-menu edit
becomes the new saved selection. Normal launches keep existing preferences and
the High default. This allows driving and vision workflows to choose their own
quality instead of inheriting a global reduction.

`--headless-fps` defaults to the existing 60 FPS limit. Zero uncaps the headless
render loop; positive values through 1000 set a limit. It changes presentation
pacing, not physics integration or match time, and has no effect in windowed
mode. Use uncapped rendering for throughput measurements and an explicit cap
for nonessential visual monitoring.

Next measurement priorities are a quiet release-build full-app profile, raw
frame-time distributions and GPU timing on a supported backend. Graphics
quality for vision should additionally be chosen against its actual resolution,
exposure and lighting requirements; a prettier preset is not camera calibration.
