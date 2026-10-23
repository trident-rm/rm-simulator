<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Synchronized live LiDAR and IMU

## Use the running match

The optional ROS executable can now attach to the existing headless or
interactive host's operator HTTP API. It creates **no local physical world**.
The host captures its actual integration steps, including moving equipment and
other robots. A bounded acquisition worker produces measured LiDAR/IMU bundles;
HTTP JSON encoding and ROS publication run outside the simulation worker.

Build the host normally and the ROS bridge with `pixi run build` from
`integrations/ros2/`. Start a match with HTTP enabled, for example:

```sh
just run --http 127.0.0.1:7780 --no-referee
```

Choose an existing chassis ID from `/api/state`. From `integrations/ros2/`,
using an ID of 0 as an example:

```sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
export CYCLONEDDS_URI="file://$PWD/config/cyclonedds-loopback.xml"
pixi run run --host 127.0.0.1:7780 --robot 0 --frames 600 \
  --topic /robot0/livox/lidar --frame-id robot0_lidar \
  --imu-topic /robot0/livox/imu --imu-frame-id robot0_imu --publish-clock
```

`--publish-clock` is opt-in for live mode. Enable it on exactly one bridge and
disable the control bridge's clock publication if using this clock source.
Consumers use `use_sim_time=true`. Without `--host`, the existing standalone
LiDAR-only bare-world probe remains available and owns its own clock/world.
The loopback DDS profile is local-only; see [transport configuration](ros-transport.md).

The attachment grants read-only measured sensors. It does not claim motion,
gimbal or firing control, cancel an existing planner, or change navigation into
sensor-based localization. The separate [control bridge](../integrations/ros2/README.md#control-a-robot-in-an-existing-game)
uses the existing external-drive lease. HTTP remains the privileged operator
interface; do not expose it as an untrusted robot-only API.

## Clock, frames and units

Default acquisition is nominal 200,000 LiDAR emissions/s, 100 ms LiDAR packets
and 5 ms IMU periods (200 Hz). Both use the attachment's actual simulation epoch.
LiDAR retains per-emission local FLU coordinates and timestamps, including misses
before ROS conversion. It is not deskewed. An IMU sample averages the preceding
5 ms and is stamped at that period's end. Each bundle pairs LiDAR `[start,end)`
with exactly the IMU samples in `(start,end]`.

The host records real solver substeps (at most 1 ms); the existing mounted IMU
model differentiates those poses, including mounting lever-arm effects. It needs
initial derivative history. The first incomplete bundle is discarded, so the
first normal output starts one packet after attachment. After discontinuities,
output resumes at a complete packet boundary using the original epoch and noise
indices. Missing IMU periods are never filled with synthetic values.

IMU completion lags a boundary by up to a solver substep. A packet waits for its
last IMU sample before release. The bridge publishes that bundle's IMU samples
before its LiDAR scan. This delivers IMU in packet-sized bursts, not every 5 ms
of wall time; delivery latency includes batching, acquisition and transport.
Independent ROS topics do not guarantee cross-topic callback ordering: estimators
must buffer by acquisition timestamp. Configure IMU subscription depth at least
200 for the default bursts, rather than the sensor-data default depth of 5.

- `sensor_msgs/Imu.angular_velocity`: sensor FLU **rad/s**.
- `linear_acceleration`: sensor FLU specific force **m/s²**, including the
  upright stationary reaction of approximately +9.81 on z. No conversion to g.
  Configure Livox-specific consumers that expect g explicitly at their boundary.
- Orientation is unavailable: `orientation_covariance[0] = -1`. No perfect
  chassis attitude is substituted. Zero remaining covariance matrices mean
  unknown uncertainty, not zero error.
- Neither odometry nor TF is published. Supply configured static mounting
  transforms and an external estimator separately. Measured output includes no
  robot world pose or per-point pose truth.

## Configure mounting and errors

With no configuration file, live mode uses the CLI mount height/pitch for both
sensors, with ideal IMU effects. Existing LiDAR stride, range noise, dropout,
angular noise and seed flags apply. Co-location is a configurable convenience,
not a claim about the physical MID-360 internal IMU extrinsic.

For separate mounts, an actual gimbal carrier or inertial effects, provide
`--sensor-config PATH.json`. The file is authoritative for the addressed robot,
mounts, periods, stride and effects; overlapping CLI acquisition options are not
merged. Unspecified fields take `synchronized::Config` defaults. Example:

```json
{
  "robot": 0,
  "carrier": "Gimbal",
  "lidar_mount": {"translation_m": [0.1, 0, 0.2], "rotation_wxyz": [1, 0, 0, 0]},
  "imu_mount": {"translation_m": [0.1, 0, 0.2], "rotation_wxyz": [1, 0, 0, 0]},
  "lidar_period_ns": 100000000,
  "imu_period_ns": 5000000,
  "stride": 3,
  "range_sigma_m": 0.02,
  "angular_sigma_rad": 0,
  "dropout": 0,
  "seed": 7,
  "imu_effects": {"seed": 11}
}
```

Mounts use carrier-local FLU metres and unit wxyz rotations. Compose any known
LiDAR-to-IMU extrinsic explicitly; none is inferred from appearance. The gimbal
carrier is the actual motor-limited gimbal pose, not a commanded target. Errors
are generic seeded stress models, not calibrated device measurements.

Packet periods are bounded to 10 ms–1 s; IMU periods are at least 1 ms and divide
the packet period. The assembler accepts coherent physics brackets of at most
10 ms. The hosted world remains 128 Hz; this feature does not alter multiplayer
physics timing. Callers with a different supported physics rate can use the
passive `physics::sensors::synchronized::Synchronized` assembler directly.

## Bounded live-host API

The same API is available to in-process callers through `Simulation` and
`HostHandle`, without ROS. Operator HTTP routes are:

| POST route | JSON body | Result |
|---|---|---|
| `/api/sensors/attach` | Sensor configuration (defaults permitted) | `{"id": attachment_id}` |
| `/api/sensors/poll` | Numeric attachment ID | Current simulation `time_ns`, up to two measured `frames`, cumulative `dropped_intervals` |
| `/api/sensors/detach` | Numeric attachment ID | `{"ok": true}` |
| `/api/sensors/attach-profiled` | Same sensor configuration | New attachment with opt-in host telemetry |
| `/api/sensors/poll-profiled` | Profiled attachment ID | Same measurements plus bounded telemetry and response timing headers |

See [sensor transport telemetry](sensor-transport-telemetry.md) for stage timing,
separate loss counters and a repeatable profiling script. Host durations never
replace simulation stamps; normal attachments retain their existing payloads.

At most four attachments exist. Each has four queued capture brackets and two
queued output bundles. The host uses nonblocking sends; an overloaded acquisition
worker drops capture intervals, and an overloaded consumer drops output bundles.
Capture and output losses are counted as intervals of their respective sizes,
not converted into an invented missing-point count. Packet sequence numbers and
cumulative input discontinuities expose gaps; timestamps are never compressed.
The bridge reports drop-count changes and sequence gaps on stderr.

Discontinuous captures (placement, topology/shape changes, gaps) discard partial
paired packets. Defeat or removal invalidates the attachment. Poll at least once
within five host seconds to retain it, including while paused. Abandoned
attachments expire; explicit detach releases their queues. This host-time liveness
policy never changes measurement timestamps. Pausing produces no new samples.

JSON polling is an initial integration transport, not a claimed high-bandwidth
binary sensor protocol. It preserves the bounded worker boundary so a later
native/Zenoh adapter can reuse measured bundles. The ROS client bounds each
response to 32 MiB and uses socket timeouts; normal default packets are much
smaller. Large arrays are encoded on HTTP handlers, not on the owner worker.

## Verification

Core tests check matching 100 ms / 5 ms boundaries, SI gravity reaction,
chunking/decimation consistency, schedule bounds and missing-capture recovery.
Server tests exercise real stepping and acquisition, bounded slow-consumer
behavior, expiry and detach. The end-to-end test starts a real CAD host and
checks six Livox scans with 120 actual SI IMU messages, shared stamps, exact IMU
cadence and unavailable orientation. The moving variant spawns and drives a
Sentry and requires positive measured yaw rate; it checks SI force magnitude
as well as timestamps:

```sh
nix develop -c cargo build -p rm-sim-server --locked
# From integrations/ros2/, after pixi run build:
pixi run sensors-smoke
pixi run sensors-smoke --stride 1 --moving
```

`sensors-smoke` defaults to explicit Cyclone loopback and requires the local CAD
package and `target/nix/debug/rm-sim-server`. Use `--server-bin` or
`--cad-assets` for another local build/install. It bounds the full process group
lifecycle, including native ROS teardown, and fails rather than passing forced
termination. These native runtime checks are macOS arm64 results; Linux remains
unvalidated until tested there.
