<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Optional ROS 2 / Livox integration

`rm-simulator-ros2` publishes rolling simulated LiDAR using **rclrs 0.8.0** and
actual `livox_ros_driver2/msg/CustomMsg` / `CustomPoint` interfaces. It also
publishes `rosgraph_msgs/msg/Clock` on `/clock`. The Rust package has its own
workspace and lockfile: ordinary app/server builds need no ROS libraries or
ROS Cargo features. No ROS dependency is added to physics, world or rendering.

The included executable is a standalone headless probe with one driven chassis
and the bare physical ground. It is not a switch on the multiplayer server or
interactive app, does not load the arena, and does not control a running match. The separate `control` task below can drive
an existing app/server session through its operator API.
The publisher library accepts scans from any caller-owned `Field`, including
one initialized with the CAD arena. The [mounted sensor API](../../docs/navigation-sensors.md)
supplies coherent geometry and motion without a second physical world.

## Install, build and run

Install [Pixi](https://pixi.sh) and Rust, then run from this directory:

```sh
pixi install --locked
pixi run build
pixi run run --frames 100
```

From the repository root, use
`pixi run --manifest-path integrations/ros2/pixi.toml build` and the corresponding
`run` task. RoboStack supplies ROS 2 Jazzy, the C/Python message generators and
DDS. The Pixi lock resolves Apple Silicon macOS, Linux x86_64 and Linux aarch64;
**live publishing was tested on Apple Silicon macOS only**. Rust comes from the
existing rustup installation. No system-wide ROS installation is required.

`build` first generates C/Python support for the interface-only Livox package,
then builds the opt-in Rust `ros2` feature. Generated files stay in ignored
`.ros/`, `.pixi/` and root `target/ros2/`. Interface input/environment hashes
avoid repeatedly installing already-patched macOS shared libraries. Rebuilding
changed interfaces recreates only the generated `.ros/install/` directory.

Fast DDS (`rmw_fastrtps_cpp`) is the default backend, preserving a caller-supplied
`RMW_IMPLEMENTATION` override. The smoke test now also respects that override or
an explicit `--rmw`. On this Mac, automatic interface selection produced discovery
failures and shutdown stalls with both Fast DDS and Cyclone. An explicit Cyclone
IPv4 loopback profile and a separate Zenoh RMW trial passed strict delivery and
clean shutdown checks. For local processes, after building:

```sh
CYCLONEDDS_URI="file://$PWD/config/cyclonedds-loopback.xml" \
  pixi run smoke --rmw rmw_cyclonedds_cpp --discovery-range SYSTEM_DEFAULT
```

This profile is local-only. See the [transport investigation](../../docs/ros-transport.md)
for evidence, full-rate/decimated checks, Zenoh setup and deployment limits.

`run` uses real-time pacing, 128 Hz physical integration, nominal 200,000 emissions/s
and 10 Hz message batching. It settles the chassis for one simulated second before
starting acquisition, so the first published timebase is 1,000,000,000 ns. Examples:

```sh
pixi run run --frames 20 --stride 3 --frame-id sentry_livox --lidar-id 1
pixi run run --frames 20 --range-noise-m 0 --dropout 0 --fast
pixi run run --frames 20 --angular-noise-deg 0.05 --seed 7
pixi run run --topic /livox/lidar --ros-args -r __node:=sentry_lidar
```

`--stride 3` retains every third nominal emission without changing phase/timing;
odd strides preserve all four lanes. `--mount-z-m` and `--mount-pitch-deg` configure
the mount; the library accepts a complete FLU pose. `--seed`, `--range-noise-m`
and `--dropout` configure the generic range effects. `--angular-noise-deg` opts
into independent tangent-axis emitted-ray perturbations before intersection,
using that same device seed. It defaults to zero; values must be finite and in
`[0, 180]` degrees. It is an uncalibrated stress model and retains dynamic mounting
and near-surface occlusion. Run `pixi run run --help`
for the full probe reference. `--wait-for-subscriber-ms` bounds startup discovery;
`--require-subscriber` turns a timeout into an error.

## Message and clock contract

- Header stamp and `timebase` both use the first **emission** timestamp in the
  packet, in simulation time. The first emission may have missed. `offset_time`
  preserves each retained point's nanoseconds relative to that base; misses do
  not compress or renumber time. Packets spanning more than uint32 nanoseconds
  are rejected, not wrapped. Normal 100 ms frames are comfortably bounded.
- `point_num` equals valid measured return count. No-return samples are omitted;
  an acquisition containing only misses publishes zero points with its timestamp.
  A frame with no emission timestamps is invalid.
- XYZ are float32 metres in local **sensor FLU at emission**, preserving motion
  distortion for a consumer's deskewing. No ground-truth world pose is injected.
- `line = sample_index % 4` identifies the inferred pattern lane. Exact physical
  firing timing remains a model approximation.
- `tag = 0x10` is the normal first-return simulation policy. `reflectivity = 100`
  is a constant synthetic placeholder, configurable through `LivoxConfig`.
  It is not a measured material response. Reserved bytes are zero.
- Point topics are best-effort, volatile, depth 5. `/clock` is best-effort,
  volatile, depth 1. Subscribers must accept best-effort QoS. Configure ROS
  consumers with `use_sim_time=true`; wall time never replaces point timestamps.
- No IMU, odometry or TF is published. `physics::sensors::imu` measures mounted
  IMU samples, but this adapter does not yet publish them. The existing Sentry controller remains a ground-truth baseline.

The upstream [message definitions](https://github.com/Livox-SDK/livox_ros_driver2/tree/21445540f0d100dc86a7e6df312dd70bbdb4afdf/msg)
are copied into the interface-only package, with its MIT notice. Only trailing
blank lines are normalized; message fields and comments match upstream.
No device driver, SDK, hardware access or upstream C++ implementation is required.
Use the provided package or an existing matching Livox installation as your
interface provider, avoiding duplicate overlays of different definitions.

## Embed the publisher

With the `ros2` feature enabled and ROS sourced, create a `LivoxPublisher` from
an rclrs node, then convert and publish a completed `RollingFrame`:

```rust,ignore
let publisher = rm_simulator_ros2::publisher::LivoxPublisher::new(&node, "/livox/lidar")?;
let packet = rm_simulator_ros2::LivoxPacket::from_scan(&scan, &Default::default())?;
publisher.publish(&packet)?;
publisher.publish_clock(field.time_ns())?;
```

The adapter owns no simulation, acquisition cursor, worker or scheduling thread.
Callers retain their `Field` and step it normally, sample each physics bracket,
and accumulate bounded 100 ms frames. The standalone probe demonstrates this
without changing app/protocol ownership rules. RMW publishing can block: a future
host integration should put publishing behind its own bounded queue, not inside
the authoritative simulation worker or a render system. A complete embedding
example is in [src/main.rs](src/main.rs).

## Checks

Conversion/timestamp validation is available without ROS:

```sh
cargo test --manifest-path integrations/ros2/Cargo.toml --locked --no-default-features
```

Native checks, from this directory:

```sh
pixi run test
pixi run smoke
pixi run smoke --stride 3
pixi run smoke --stride 3 --angular-noise-deg 0.05
```

The smoke test uses a separate local ROS domain and a real rclpy subscriber with
the generated Livox Python interfaces. It verifies DDS delivery, counts, ordering,
header/timebase correspondence, nanosecond offsets, lane/tag/reserved values,
finite coordinates, frame/device IDs and monotonic `/clock`. It requires process
shutdown to complete, with a 40 s process-group watchdog covering native teardown.
`--domain-id` isolates concurrent runs; `--frames 60` extends the default six-packet
check without relaxing its assertions. Point validation runs after bounded
collection so dense-packet checks do not starve the receive callback.
Tested initial full-rate run: six CustomMsg packets,
14,589 measured returns and 78 clock samples.

The 2026-10-23 angular-adoption check passed native conversion tests and Clippy,
but live DDS smoke was not clean in the tested macOS environment. Runs either
lost packets or timed out; a sampled publisher was waiting in Fast DDS 2.14.6
`MessageReceiver::removeEndpoint` during publisher destruction after acquisition.
This occurred with angular noise disabled as well. The angular model's native
geometry tests pass independently; they do not establish reliable DDS delivery
or shutdown. The subsequent investigation isolated a successful local Cyclone
interface configuration and verified Zenoh independently; see the report above.
Keep these macOS results separate from Linux runtime validation.

The optional package is outside `just verify` so that core checks remain ROS-free.
Run its native tests/Clippy and dependency checks separately when editing it.

Dependency checks use the scoped optional policy:

```sh
cargo deny --manifest-path integrations/ros2/Cargo.toml --config integrations/ros2/deny.toml check
```

rclrs 0.8.0 requires async-std 1.13, which carries maintenance advisory
[RUSTSEC-2025-0052](https://rustsec.org/advisories/RUSTSEC-2025-0052.html) because
the project was discontinued. This is not a reported memory-safety vulnerability;
no replacement is selectable through rclrs's current features. The optional
adapter's policy records this maintenance exception; core policy is unchanged.
Revisit it when rclrs replaces async-std. Dependencies remain unmodified and
permissively licensed.

## Control a robot in an existing game

The `control` task subscribes to `geometry_msgs/msg/TwistStamped` and forwards
bounded, sequenced external drive requests to the existing host. It creates no
second world and requires no ROS dependency in the app/server. This first bridge
uses the operator HTTP API at up to 20 Hz with bounded latest-value mailboxes. It is suitable
for initial navigation control, not high-rate embedded control or sensor export.

Start the interactive simulator with its local HTTP API enabled:

```sh
just run --http 127.0.0.1:7780 --no-referee
```

Use an operator-spawned Sentry so an occupied pilot seat does not keep sending
manual inputs. The console's world command or `POST /api/command` can spawn one:

```json
{"SpawnSentry":{"team":"Red","position_m":[4,-3,1],"yaw_deg":0}}
```

Read its stable chassis ID from `GET /api/state`. The same API is available from
the headless host. Run the bridge from the repository root, substituting that ID:

```sh
pixi run --manifest-path integrations/ros2/pixi.toml control --chassis 0 --mode heading --topic /sentry/cmd_vel --publish-clock
```

Enable `--publish-clock` only when this bridge is the match's single ROS clock
publisher. A dedicated worker polls the compact `/api/clock` endpoint at up to
20 Hz, including while paused; it never substitutes wall time. Consumers must use simulation time.
Do not run the separate bare-world probe as a clock source for the live game.

| Mode | Input frame | Twist interpretation |
|---|---|---|
| `body` | `base_link` | Linear x/y are body forward/left m/s; angular z is CCW rad/s. |
| `world` | `map` | Linear x/y are arena-frame m/s; angular z is CCW rad/s. |
| `heading` (default) | `map` | Nonzero linear x/y gives world heading and its magnitude gives m/s. Host turns toward it; angular z must be zero. A zero vector stops. |

`--frame-id` overrides the expected frame name; it does not transform frames.
A `map` command must already use the simulator arena's FLU axes and orientation;
apply an explicit transform in your navigation adapter if localization uses a
different map orientation. Nonplanar components and mismatched frames are refused.
Heading mode is intended for omnidirectional chassis. It can translate sideways
while turning; it is not a differential-drive controller or collision avoidance.

Publish new messages at 20 Hz, with simulation timestamps in `header.stamp` and
the exact expected `header.frame_id`. The subscription accepts best-effort QoS.
Only new ROS messages renew motion: the timer never resends a previous setpoint.
HTTP requests run on independent command and clock workers, outside ROS callbacks.
The ROS input cache and each worker retain at most one unsent value, plus one
in-flight HTTP request per worker. Newer commands replace queued ones; a slow
clock request cannot block command submission or subscription callbacks. Shutdown
drops unsent commands and waits for the in-flight HTTP operation before attempting
release. If an HTTP operation fails, the host's timestamp admission and lease
expiry remain the fallback; a client timeout cannot cancel an already queued
server operation.
Requests over 6 m/s or 4 rad/s are rejected, with heading feedback capped at
4 rad/s and gain 4/s. Native motor, power, collision and defeat restrictions still
apply. These bounds are simulator adapter policies, not RoboMaster rule constants.

The first accepted request takes motion control, cancels built-in navigation and
disables chassis/gimbal sweep. Gimbal aim, folding state and balance setting are
preserved; firing and mechanism control are separate commands. Manual pilot input,
ordinary `Chassis` commands, a new built-in navigation goal or a pause revokes the
external lease on fresh input admission, including a transition scheduled for a
future tick. Replay evidence survives external claims: duplicate pilot packets
cannot restore superseded manual motion. Team map intents do not replace an active external motion lease.
Old navigation goals never automatically resume.

The host brakes after 250 ms of simulation time without a fresh sample. Delayed,
duplicate, out-of-order and revoked requests cannot refresh a lease. The host also
checks the current pause epoch and robot placement revision. The bridge disarms
motion on a command HTTP rejection or connection error; it never automatically steals
control back after manual takeover. When enabled, clock polling continues after
motion is disarmed. Clock failures are reported separately and do not themselves
revoke motion; the host's ordinary timestamp and stale-command checks still apply.
Restart with a newer `--lease-id` after checking the
game. By default a process uses its wall-clock startup nanoseconds only as a
unique increasing lease identifier, never as a sensor/command timestamp.

On normal exit the bridge attempts a release; the host timeout handles a killed
process or lost connection. A paused world does not advance its timeout, but
entering pause revokes motion immediately and resuming requires a new lease.
An occupied player seat's ordinary input packets count as manual takeover even
when neutral, hence the recommendation to use an unpiloted robot.

For a direct non-ROS client, `POST /api/command` the same protocol request:

```json
{"ExternalDrive":{"chassis":0,"input":{"lease_id":1,"sequence":1,"sampled_time_ns":1000000000,"input_epoch":0,"placement_revision":0,"motion":{"HeadingVector":{"direction_world":[1,0],"speed_m_s":1}}}}}
```

Read time, epoch and the robot's placement revision from `GET /api/control/0`:

```json
{"time_ns":1000000000,"input_epoch":0,"paused":false,"chassis":{"id":0,"placement_revision":0,"defeated":false}}
```

`GET /api/clock` returns the same clock/epoch/pause fields with `chassis: null`.
Neither endpoint builds a full world snapshot. Use `time_ns` as a current sample
stamp when no independently synchronized ROS clock is available. The numbers
above are illustrative. Advance sequence on every
sample. Lease IDs must be nonzero and increase when reacquiring a robot. A direct
`HeadingVector` request with zero speed turns in place; signed speed permits
backing along the heading while facing it. `BodyVelocity` and `WorldVelocity`
carry `velocity_m_s: [x,y]` and `yaw_rate_rad_s` instead. Release explicitly with:

```json
{"ReleaseExternalDrive":{"chassis":0,"lease_id":1}}
```

The endpoint is privileged operator access. Keep it on loopback or an otherwise
trusted network, using the existing referee-panel deployment contract. Ordinary
network pilots cannot submit these external operator commands. Protocol 80
requires matching current app/server builds. The lease lives in the host
controller, not physical checkpoints; it does not authorize referee edits.

No live-match LiDAR/IMU/odometry/TF export is added by this control bridge. A real
planner can now actuate the in-game robot, but sensor-driven navigation still
needs those adapters. Clock/control discovery now uses compact metadata endpoints
rather than polling full match state. HTTP remains a prototype control transport;
a future telemetry stream can provide higher rates without per-request connections.

Test actual DDS input against a headless CAD host (same simulation/command path
as the game), after building the normal server binary:

```sh
nix develop -c cargo build -p rm-simulator-server --bin rm-simulator-server --locked
pixi run --manifest-path integrations/ros2/pixi.toml control-smoke
```

The smoke test starts and cleans up its own CAD host/ROS bridge in an isolated
ROS domain, checks all three control modes, physical displacement, timeout
braking while the bridge remains alive and manual takeover. It does not run
Nav2 or claim successful sensor-based autonomous navigation.

ROS-free regressions for slow I/O, latest-value replacement, command failure and
shutdown ordering run in the core Python suite:

```sh
python3 -m unittest discover -s scripts/tests -p test_control_io.py
```
