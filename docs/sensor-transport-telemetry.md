<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Profile the live sensor path

Use stage timings before choosing a new middleware. The live host supports an
opt-in diagnostic attachment that keeps acquisition, queues, owner requests and
JSON encoding distinguishable. It uses the same world, geometry, dynamic mounts,
acquisition worker, queue capacities and measured values as ordinary attachments.
No physics or acquisition clock is replaced with host time.

## Attach and poll

All routes use the existing privileged operator HTTP interface. Ordinary
`/api/sensors/attach` and `/api/sensors/poll` keep their existing JSON shapes and
do not enable the additional capture/acquisition timing reads. For diagnostics:

```sh
curl -s http://127.0.0.1:7780/api/sensors/attach-profiled \
  -H 'Content-Type: application/json' -d '{"robot":0,"stride":3,"carrier":"Gimbal"}'
# Use the returned numeric attachment ID in both requests below.
curl -si http://127.0.0.1:7780/api/sensors/poll-profiled \
  -H 'Content-Type: application/json' -d '0'
curl -s http://127.0.0.1:7780/api/sensors/detach \
  -H 'Content-Type: application/json' -d '0'
```

The same methods exist on `Simulation` and `HostHandle` as
`attach_sensors_profiled` and `poll_sensors_profiled`. A profiled poll of an
ordinary attachment fails without draining it. An ordinary poll can drain a
profiled attachment and returns only ordinary measurements. Share neither kind
of attachment between competing consumers.

Profiled JSON retains top-level `time_ns`, `frames` and `dropped_intervals`, and
adds `telemetry`. Its `frames` records align with measured frames by sequence.
Every duration below is integer **wall nanoseconds**, measured with monotonic
host intervals, not a wall-clock epoch or sensor timestamp.

| Location / field | What it includes |
|---|---|
| `telemetry.owner_request_wait_ns` | HostHandle submission/admission, mailbox and scheduling delay until owner dispatch; null for direct Simulation calls |
| `telemetry.owner_poll_wall_ns` | Owner service poll, including pruning, bounded dequeue and metadata |
| `telemetry.frames[].output_queue_wait_ns` | Ingest completion to owner dequeue; excludes JSON encoding and client reception |
| `telemetry.frames[].work.capture_wall_ns` | Summed per-attachment geometry capture, scene clone and carrier trace selection; excludes shared trace drain |
| `work.input_queue_wait_ns` | Sum of input bracket queue residence times |
| `work.input_queue_wait_max_ns` | Largest single input bracket queue residence time in this work batch |
| `work.ingest_wall_ns` | Summed worker ingest durations, including LiDAR queries and IMU assembly; wall duration, not CPU time |
| `X-RM-Sensor-Host-Call-Ns` HTTP header | Complete handler-to-HostHandle round trip, including reply scheduling |
| `X-RM-Sensor-Encode-Ns` HTTP header | JSON response encoding on the HTTP handler, outside the owner |

Work accumulates from the previous ingest that completed bundles, including
warmup or discarded partial acquisition. An ingest can cross packet boundaries
or complete multiple bundles. Consequently work is **not exact per-packet or
per-ray attribution**. Deduplicate `work.batch` per attachment: the same batch
may appear on several frames. `work.completed_frames` includes bundles dropped
at delivery, and `work.input_brackets` counts ingested brackets. Work for an
entirely dropped output batch is unavailable. This is bounded per-output metadata,
not an unbounded profiler history.

`telemetry.losses` separates cumulative `capture_intervals` (rejected/failed
capture brackets), `ingest_errors` (failed ingest calls) and `output_frames`
(completed bundles rejected by a full output queue). These are different units;
their sum preserves the existing heterogeneous `dropped_intervals` counter.
Counters are read once per poll, independently of concurrent increments, so
new losses may appear next poll. Packet sequence gaps and acquisition
`discontinuities` remain distinct signals.

## Repeat a diagnostic trial

Build before measuring. Start the actual server/app with HTTP enabled, and pick
an existing live robot from `/api/state`. The script never spawns or drives a
robot and never changes a running match's pause state or clock:

```sh
nix develop -c cargo build -p rm-sim-server --locked
# In another terminal, for example:
target/nix/debug/rm-sim-server --cad-assets /path/to/extracted/field-2027 \
  --no-referee --no-lobby --http 127.0.0.1:7780
# Once an existing live chassis is available:
python3 scripts/benchmark-sensors.py --host http://127.0.0.1:7780 --robot 0 \
  --profile --stride 1 --frames 60 --output local-assets/sensor-profile-1
```

Omit `--profile` for a control using ordinary endpoints. `--streams 4` attaches
four independent acquisitions to the selected carrier; this does not create
four worlds. `--carrier Gimbal` follows its actual dynamic motor pose; control
motion separately through the host as usual. Both modes request default 100 ms
LiDAR packets / 5 ms IMU samples and validate retained emission indices, packet
epoch and IMU cadence. The script is standard-library Python with no ROS install.

A fresh output directory is required. It holds a `summary.json` with nearest-rank
p50/p95/p99/max distributions, options, platform/load and source hashes, and
`polls.jsonl` with each poll's metadata/timings (no point arrays or world poses).
Source hashes identify the checkout, not proof that an arbitrary running host
was built from it: record the binary/build and asset identity alongside trials.
Five received frames per stream are warmup by default. Polls crossing warmup are
included in response statistics; a returned two-frame batch can exceed the
requested minimum frame count. Empty/nonempty poll durations are separate.

Requests have bounded socket/body sizes and deadlines. The run deadline is
checked between polls; an in-flight request and detach attempts may extend it.
Timeouts, invalid metadata, loss and cleanup failure produce an error report and
nonzero exit. The script always attempts to detach its own attachments. A paused
world produces no packets and will time out. For an intentional slow-consumer
trial, use e.g. `--stall-ms 600 --allow-drops`; losses remain visible in artifacts.
The stall is bounded below the attachment's five-second host-time expiry.

## Interpret the evidence

Compare nonempty JSON encoding, worker work, input/output queues, owner wait,
client time to headers, body read and JSON decode. Host-call duration contains
owner wait/poll; do not add all three as independent stages. Input queue sums are
across brackets, not a frame's single end-to-end latency. A bundle's simulated
`time_ns - end_ns` age is **simulation nanoseconds**, separate from wall queues.
There is no cross-process clock calibration requirement for durations.

Client time to headers also includes connection setup, accept/handler scheduling,
request parsing and networking. Encoding excludes socket writes. Their difference
cannot isolate listener sleep or network delay. A controlled A/B of an accept
loop, keepalive or codec change is still necessary before assigning causation.
Run repeated controls without compilation or unrelated heavy work; record CPU,
load, binary, assets, streams, motion and consumer configuration. Profiling itself
adds timing/metadata overhead, so compare it with the ordinary path. These HTTP
measurements do not validate DDS delivery or a real LIO consumer.

## Local diagnostic result, 2026-10-23

Six fresh CAD-host trials exercised ordinary/full-density acquisition, profiled
full density, stride 3, four full-density attachments, a moving gimbal, and an
intentional 600 ms consumer stall. Each fresh host used one rotating Sentry
(0.5 rad/s requested chassis yaw); the gimbal case changed world aim every
0.5 seconds. Normal trials requested 60 measured packets per stream after five
warmup packets; gimbal/stall requested 30. Emission indices, shared epochs and
all 200 Hz IMU stamps passed validation in every returned packet. The dynamic
gimbal trial delivered 35 packets with zero stage losses.

These were macOS 27.0.1 arm64 / Apple M5 Max (18 logical CPUs) debug-profile
optimized builds, with the system Python **3.9.6** consumer. Ambient one-minute
load averages ranged approximately 18–24; unrelated work continued. This differs
from the Python 3.12 consumer in the sibling mid360-sim benchmark. It is not a
controlled profiling-overhead comparison or a ROS/LIO delivery benchmark.

The one-stream profiled full-density trial reported **three rejected capture
brackets**, zero ingest errors and zero output drops. The plain control also
reported three heterogeneous losses and two packet sequence gaps. Both correctly
exited nonzero under the default loss policy. In the profiled trial, work-window
ingest p50/p95 was 29.01/64.05 ms; the maximum per-bracket input wait had
p50/p95 1.20/21.93 ms. JSON encode p50/p95 was 3.38/4.25 ms, and Python decode
21.07/49.31 ms for nonempty responses. These observations identify input loss
in that trial; they do not isolate its scheduling/compute cause.

The single-stream stride-3 trial had zero stage losses, with ingest-window
p50/p95 10.53/15.79 ms and nonempty JSON encode 1.16/1.50 ms. In the four-stream
trial, the six losses were **completed output bundles**, with no input or ingest
losses. Its sequential Python consumer decoded up to two frames per response;
output queue residence p50/p95 was 99.84/192.50 ms. It is evidence of output-path
pressure for this consumer, not a proven server-network or DDS limitation.
The deliberate stall produced four output drops and no input/ingest losses,
with output queue residence reaching 540.78 ms. Pressure trials explicitly used
`--allow-drops`; their successful exit means the diagnostic completed, not that
delivery was lossless.

This supports retaining stage diagnostics and testing the actual consumer before
changing middleware. Candidate followups are a faster client decoder/native
consumer, a controlled listener/keepalive experiment and repeated acquisition
measurements under quiet load. No replacement transport is justified solely by
these results. Work windows and header waits remain subject to the attribution
limits above; sequential runs cannot establish the overhead of profiling.

Raw metadata, source hashes and logs remain local under
`local-assets/sensor-profile-20261023/`. The tested host was based on `a4b06ff`
with the telemetry working changes, binary SHA-256
`3278f09f0488e0b3577ba552bf430a08f2b4d78ec8a3fa6d2c815876f4769bb5`,
and field manifest SHA-256
`8a92e1d6721be2e214cd48251fb6d9b2730c16f0df7116407dbfe819f9972586`.
No CAD, packet arrays or generated native artifacts are committed.
