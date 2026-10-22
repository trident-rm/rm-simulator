<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# ROS transport investigation

## Finding and scope

On 2026-10-23, the macOS arm64 ROS probe failed discovery/delivery or hung in
native middleware destruction with automatic interface selection. Explicit
Cyclone DDS IPv4 loopback configuration passed the existing strict message checks
and clean shutdown. The dense standalone test also exposed receive starvation
from validating all points inside the smoke subscriber callback; deferring those
checks until after collection restored complete delivery. Zenoh RMW with a local
router also passed, using the same
Rust publisher binary and unchanged sensor acquisition.

Use the supplied Cyclone loopback profile for local integration experiments.
Evaluate Zenoh for deployment when explicit routing is useful. Neither result
justifies writing a new DDS implementation or ROS RMW. The normal Fast DDS
runtime default is unchanged; these are opt-in transport configurations.

These are local functional tests, not throughput/latency benchmarks, network-loss
experiments, Linux validation or full navigation-stack qualification. Best-effort
QoS does not guarantee delivery in arbitrary deployments.

## Reproduction environment

- macOS 27.0.1, Apple Silicon; rclrs 0.8.0, ROS 2 Jazzy via RoboStack/Pixi.
- Base environment: rcl 9.2.11, rclpy 7.1.11, rmw 7.3.3,
  rmw_fastrtps_cpp 8.4.4 / Fast DDS 2.14.6,
  rmw_cyclonedds_cpp 2.2.3 / Cyclone DDS 0.10.5, libcxx 23.1.4.
- Zenoh used an isolated experimental environment with rmw_zenoh_cpp 0.2.10
  and libzenohc 1.9.0. The listed rcl/rclpy/rmw/DDS/libcxx versions matched
  the base environment; other dependency changes were not exhaustively isolated.
- Publisher: angular-adoption Rust release binary, built from `b96484a`.
  Best-effort, volatile LiDAR depth 5 and clock depth 1; normal 10 Hz batching,
  128 Hz physics, nominal 200,000 emissions/s. Full-rate here means no emission
  decimation, not that every emission produces a return on the bare ground.
- Each test uses a fresh ROS domain, actual generated Livox messages, a separate
  Rust process and Python subscriber. Points retain simulation timestamps.

## Evidence

| Configuration | Observed result |
|---|---|
| Fast DDS, ROS `LOCALHOST`, Python-only 3,100-point packets | One six-packet pass with clean shutdown; not consistently reproducible |
| Fast DDS, ROS `LOCALHOST`, Python-only one-point packets | No match/data and publisher destruction stalled in a later run |
| Fast DDS, Rust probe, `LOCALHOST` or `SUBNET` | Discovery failures; previous runs also lost packets or stalled after publishing |
| Cyclone DDS, automatic interfaces, `LOCALHOST` | Python-only and Rust runs hung in destruction; Rust received no data |
| Cyclone DDS, automatic interfaces, `SYSTEM_DEFAULT` | No packets; 40 s lifecycle watchdog killed the test process group |
| Cyclone DDS, explicit loopback profile | Strict six-packet and 60-packet runs passed with clean shutdown |
| Cyclone DDS, only interface pinned to `127.0.0.1`, ROS `LOCALHOST` | Strict six-packet run passed with clean shutdown |
| Cyclone DDS, loopback profile with multicast enabled | Strict six-packet run also passed |
| Zenoh RMW, loopback TCP router | Strict six-packet and 60-packet runs passed with clean shutdown |
| Dense standalone room, Cyclone loopback, inline point validation | 38 / 60 packets received |
| Dense standalone room, Zenoh, inline point validation | 47 / 60 packets received |
| Dense standalone room, same transports, validation after collection | Both received 60 / 60 packets, 1,200,000 returns and the final clock; clean shutdown |

Three additional consecutive 60-packet runs per candidate also passed, using
fresh domains each time. At full rate, each successful run delivered 188,887 measured returns,
the final clock at 7,000,000,000 ns and clean process shutdown. Both candidates
also passed stride 3 with angular sigma 0.05 degrees: six packets / 6,209 returns.
Cyclone passed stride 3 without angular noise: six packets / 6,204 returns.
The standalone room supplied the dense-payload check: 20,000 returns per 100 ms
packet at 10 Hz. Both candidates passed after the subscriber callback was
reduced to bounded collection, with unchanged point validation performed after
reception. That intervention supports receive-path starvation as the cause of
these particular dense-test losses; it does not explain the independent
no-discovery/native-teardown failures. A consumer that cannot keep up can still
drop data under best-effort QoS with either middleware.

The test still requires every expected packet, valid fields, correct emission
indices/offsets and a monotonic clock; no acceptance criteria were relaxed.

Previously captured Fast DDS stacks show publisher destruction waiting in
`MessageReceiver::removeEndpoint`, with a receive thread processing discovery
locators through `IPLocator::setIPv4` / `std::regex_match`. Cyclone stacks show
`wait_for_receive_threads` during node destruction. These locate the stalled
native paths; they do not prove a particular mutex bug, C++ ABI mismatch or
upstream defect.

The interface-only Cyclone experiment is the strongest isolation so far:
changing the selected interface while retaining ROS `LOCALHOST` is sufficient
for success. Multicast being enabled also passed on loopback. This implicates
the automatic interface/locator path on this host, rather than establishing
multicast itself as the cause. The exact failing interface and native shutdown
mechanism still need an upstream-sized reproducer and native tracing. Increasing
the drain sleep or switching angular noise off is not an established fix.

## Use Cyclone for local processes

From `integrations/ros2/`, after the normal build:

```sh
CYCLONEDDS_URI="file://$PWD/config/cyclonedds-loopback.xml" \
  pixi run smoke --rmw rmw_cyclonedds_cpp --discovery-range SYSTEM_DEFAULT
CYCLONEDDS_URI="file://$PWD/config/cyclonedds-loopback.xml" \
  pixi run smoke --rmw rmw_cyclonedds_cpp --discovery-range SYSTEM_DEFAULT \
  --stride 3 --angular-noise-deg 0.05 --domain-id 74
CYCLONEDDS_URI="file://$PWD/config/cyclonedds-loopback.xml" \
  pixi run smoke --rmw rmw_cyclonedds_cpp --discovery-range SYSTEM_DEFAULT --frames 60
```

For the probe itself, pass the same settings to every ROS process:

```sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
export CYCLONEDDS_URI="file://$PWD/config/cyclonedds-loopback.xml"
pixi run run --frames 60
```

The current locked ROS base environment includes Cyclone. The profile pins IPv4
loopback, disables multicast, provides a loopback peer and uses automatic
participant indices so peers can find predictable ports. It intentionally
cannot communicate with another machine. `SYSTEM_DEFAULT` lets the XML own
DDS discovery settings rather than layering ROS discovery overrides over it.
ROS `LOCALHOST` plus this profile also passed, but mixing policy sources is
unnecessary. See the [Cyclone configuration reference](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html)
and [Jazzy RMW discovery handling](https://github.com/ros2/rmw_cyclonedds/blob/jazzy/rmw_cyclonedds_cpp/src/rmw_node.cpp).

The smoke supervisor enforces a 40 s limit over the entire worker lifecycle,
including native publisher/subscriber destruction. Timeout kills only its own
process group and fails the test; it does not treat forced termination as a
successful shutdown. `--frames` accepts 1–100 packets within that fixed budget.
The ordinary six-packet default and all schema/timing checks remain intact.
The callback retains at most the requested number of messages (maximum 100),
then validates each point after reception. This is a bounded test collector,
not the queue/memory policy for a long-running simulator exporter.

## Zenoh trial

Zenoh preserves ROS messages and the rclrs/rclpy API through `rmw_zenoh_cpp`.
Its default discovery uses a router, and local peers can exchange data directly;
a router is not necessarily on every local data path. All ROS consumers must
use a compatible transport or an explicit bridge; a DDS-only consumer does not
become a Zenoh subscriber by changing only the simulator's environment.
See the [Jazzy setup](https://github.com/ros2/rmw_zenoh/blob/jazzy/README.md)
and [RMW design](https://github.com/ros2/rmw_zenoh/blob/jazzy/docs/design.md).

Keep the trial separate from the normal locked environment. For example, copy
this integration's `pixi.toml` and `pixi.lock` to ignored
`local-assets/transport-env/`, then run:

```sh
pixi add --manifest-path local-assets/transport-env/pixi.toml \
  'ros2-rmw-zenoh-cpp==0.2.10'
```

This experiment may resolve other packages and creates an independent runtime;
it is not a frozen replacement lockfile or a default installation instruction.
The resolved macOS trial used the versions above. Build the normal Livox
interfaces first, then use that environment to invoke this integration's
`scripts/with-ros.sh` and the same publisher binary.

Start the router in one activated experimental environment:

```sh
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false' \
  "$CONDA_PREFIX/lib/rmw_zenoh_cpp/rmw_zenohd"
```

In another terminal with the same environment activated, from `integrations/ros2/`:

```sh
ZENOH_ROUTER_CHECK_ATTEMPTS=5 bash scripts/with-ros.sh python scripts/smoke.py \
  --rmw rmw_zenoh_cpp --domain-id 75 --frames 60
```

Stop the trial router when finished. The tested package logs deprecated-config
warnings; they did not prevent these checks. No router, new package or lockfile
change is imposed on the default environment.

## Recommended integration boundary

Keep the simulation worker independent of middleware. It should hand immutable
completed sensor frames with original timestamps to a bounded delivery queue.
An adapter owns encoding, publication and shutdown. Backpressure policy and
counts of dropped frames must be explicit; transport delay cannot advance or
rewrite simulation time. ROS publishing can block even with best-effort QoS.

For a live-match exporter, start with this boundary and the existing ROS adapter.
Use Cyclone loopback for local development; qualify Zenoh with the actual LIO,
Nav2 and recording consumers before choosing it for deployment. Next checks
should cover long runs, slow consumers, restart/reconnect, missing router,
multiple sensors, remote Linux peers and bounded memory under backpressure.

If a native Rust interface is useful, add a small versioned sensor-stream
adapter over an existing transport (for example Zenoh's Rust API), with a ROS
sidecar when needed. That is a manageable application protocol. A new RMW must
also implement discovery/graph behavior, QoS, serialization/type support,
services, actions and lifecycle compatibility, far beyond streaming sensor
frames. Implementing those here has no demonstrated benefit over these working
options.
