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
interactive app, does not load the arena, and does not control a running match.
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
`RMW_IMPLEMENTATION` override. An initial CycloneDDS trial on this Mac hung in
`wait_for_receive_threads` during node destruction; Fast DDS passed delivery and
clean process shutdown. The smoke test selects Fast DDS explicitly.

`run` uses real-time pacing, 128 Hz physical integration, nominal 200,000 emissions/s
and 10 Hz message batching. It settles the chassis for one simulated second before
starting acquisition, so the first published timebase is 1,000,000,000 ns. Examples:

```sh
pixi run run --frames 20 --stride 3 --frame-id sentry_livox --lidar-id 1
pixi run run --frames 20 --range-noise-m 0 --dropout 0 --fast
pixi run run --topic /livox/lidar --ros-args -r __node:=sentry_lidar
```

`--stride 3` retains every third nominal emission without changing phase/timing;
odd strides preserve all four lanes. `--mount-z-m` and `--mount-pitch-deg` configure
the mount; the library accepts a complete FLU pose. `--seed`, `--range-noise-m`
and `--dropout` configure the generic sensor effects. Run `pixi run run --help`
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
- No IMU, odometry or TF is fabricated. LiDAR/IMU pipelines still require an IMU
  sensor adapter. The existing Sentry controller remains a ground-truth baseline.

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
```

The smoke test uses a separate local ROS domain and a real rclpy subscriber with
the generated Livox Python interfaces. It verifies DDS delivery, counts, ordering,
header/timebase correspondence, nanosecond offsets, lane/tag/reserved values,
finite coordinates, frame/device IDs and monotonic `/clock`. It requires process
shutdown to complete. Tested initial full-rate run: six CustomMsg packets,
14,589 measured returns and 78 clock samples.

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
