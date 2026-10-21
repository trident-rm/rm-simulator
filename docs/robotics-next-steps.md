<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Robotics integration investigation

Investigation following `939316e`, 2026-10-21. MuJoCo implementation is deferred
at the user's request. The optional ROS adapter was added subsequently in its
own workspace; MuJoCo remains deferred.
See [the measurements](performance-20261021.md) and
[embedding contracts](simulation-embedding.md).

## Navigation simulation

The current Sentry planner has a conservative 0.25 m multi-height graph, robot
inflation, supported-ground checks, CAD-validated tunnel links and explicit
posture/heading constraints. It routes through a known ground-truth map. It is
not localization, mapping or a sensor-driven navigation stack.

The physical engine already retains shared triangle BVHs and immutable shape
captures. Its existing `segment_clear` reports boolean obstruction, not nearest
return distance, normal or object identity. The measured visibility workloads
are promising but cannot establish nearest-hit scan throughput.

The first useful reusable primitive is now a closest-hit ray query over a geometry
capture: finite origin, nonzero normalized direction, bounded metric range,
nearest range/normal/point and capture-local shape identity. Keep it passive,
independent of navigation policies and independent of sensor scheduling. Invalid
inputs must be distinguishable from a valid no-return result. A later batch API
can reuse prepared directions and buffers. The ideal instantaneous scan API also
does so; see [navigation sensors](navigation-sensors.md) for current behavior.

The mounted sensor layer now combines enabled static/dynamic shapes in coherent
per-step captures and retains the sensor's own robot for real self-occlusion.
It interpolates robot and obstacle poses at each emission and supplies local
point coordinates, per-point timestamps and seeded range error/dropout. Capture
continuity uses collider handles/shape buffers and robot placement revisions;
stable external entity/part identities and semantic layer filtering remain
future work. A capture-local shape index is not a durable robot identity. Moving
shapes use conservative swept AABBs; existing per-mesh BVHs accelerate triangle
intersections. Maintain a top-level acceleration structure if ray × shape counts
become costly in larger scenes.

The existing ideal planar/multi-elevation scan APIs remain available. The mounted
MID-360 approximation is an empirical angular fit with explicitly nominal firing
cadence and generic range effects; material-dependent intensity remains future
work. ROS LaserScan timestamps the first ray and carries
`time_increment`; preserving that convention allows a later adapter without
changing the core. [LaserScan definition](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg)

Keep truth, odometry and estimated map pose separate. A wheel encoder estimate
must integrate actual wheel rotation rather than copying body ground truth;
wheel slip should remain observable as estimator error. Odometry carries pose
and twist with their respective parent/child frames and covariance.
[Odometry definition](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg)

Use full 3D queries through tunnels and under decks. A single-height occupancy
map or heightfield cannot represent those overhangs. Ground-truth navigation
graph connectivity may remain an optional prior, but a sensor-only experiment
must not silently read it or perfect chassis poses.

The game-approved collision meshes intentionally close tiny holes and simplify
detail. They are adequate initial geometric sensor targets, not a calibrated
reproduction of physical laser reflectance. Sensor geometry, vehicle collision
geometry and visual geometry should be independently selectable with retained
source provenance. Moving robots and equipment must participate before claiming
dynamic-obstacle navigation support.

## Running a real navigation stack during a game

Integration review, 2026-10-21. **Design below, not a delivered live-game ROS
bridge.** The optional [ROS adapter](../integrations/ros2/README.md) currently
owns a separate bare-ground probe. Its DDS smoke test establishes message delivery,
not control of the CAD arena or an active multiplayer match.

Use the same external navigation executable and topic contract against both a
headless CAD match and the interactive game. The server remains the sole owner of
the physical world. Bevy only draws it. Replace the robot hardware interface with
a simulator adapter; do not port the planner into Bevy or give ROS callbacks a
mutable `Field`. A navigation stack running on another machine should see the
same ROS interfaces as one on the game's machine.

### Existing hooks and concrete gaps

- `HostHandle::apply` queues operator commands on the authoritative worker;
  `Command::Chassis` accepts body-frame forward/left speed and yaw rate.
  `POST /api/command` exposes that operator path when HTTP is enabled. This is
  useful for low-rate experiments, not the high-rate sensor/control transport.
  HTTP operator commands do not establish a pilot seat or per-robot authority.
- `StopSentry` cancels the built-in navigation goal, disables host optical
  aim/sweep and brakes. A later `NavigateSentry` or map intent can nevertheless
  restart built-in planning. Sending `Chassis` alone does not cancel navigation:
  its next tick may overwrite the command. A complete bridge needs exclusive
  ownership of the chassis motion channels.
- Raw `Chassis` requests hold their setpoint. The pilot input stream has its own
  timeout handling, but the operator path does not inherit it. A ROS subscriber
  forwarding raw commands therefore needs a host-enforced expiry, not just a
  best-effort zero command on bridge shutdown.
- `HostHandle::try_dynamic_geometry` supplies an asynchronous debug capture, not
  coherent per-tick mounted LiDAR brackets. Polling snapshots/HTTP and combining
  them with current geometry cannot reproduce rolling acquisition timestamps.
  The live host must export matching captures from its actual integration steps.

### Proposed host boundary

Add a ROS-independent external-controller attachment in the server, selected per
robot. Keep the public wire protocol versioned if these requests are exposed to
network clients. The adapter has no referee powers by default.

1. Claim an addressed robot with a generation/lease identity. A grant cancels
   built-in chassis navigation and chassis sweep; manual takeover revokes it.
   Separate motion, gimbal and firing authority so future auto-aim can coexist
   with external driving without writing the same fields.
2. Accept bounded latest-value velocity requests with sequence, lease identity
   and host-stamped expiry. Proposed initial motion timeout: 250 ms of advancing
   simulation time. Reject nonfinite values, old sequences and revoked leases;
   retain normal motor, power, defeat and match restrictions. Preserve held aim
   and posture when changing velocity; do not construct a default full command
   that unfolds the robot or resets its gimbal on every `cmd_vel` update.
3. At each 128 Hz physical step, capture the attached robot's mounted scene at
   matching boundaries. Send shared immutable captures to a bounded acquisition
   worker. Never query DDS or trace the entire LiDAR frame inside the host worker.
   Share the fixed capture geometry across attachments and retain moving robots.
4. Publish completed timestamped sensor frames through a separate bounded DDS
   queue. On overload, report dropped intervals and restart a bracket; never
   interpolate across a missing run of ticks or silently retimestamp old data.
   Track capture, acquisition and publication latency separately.
5. On defeat, removal, placement, reset, lease loss or manual takeover, revoke
   queued motion and brake. While paused, accept no new motion for later replay;
   require a fresh command on resume. A disconnected bridge cannot leave a held
   velocity indefinitely. Host transport time may detect disconnects while
   paused; the physical world still advances only on simulation ticks.

The acquisition worker needs bounded capture backlog and explicit discontinuity
handling: folding can change collider shapes, and robot spawns/removals change
scene topology. Clear partial packets at such boundaries and preserve absolute
emission phase. Observe buffer/capture age in diagnostics rather than masking
these events with a frozen scan.

### ROS-facing contract

Use robot topic namespaces and unique frame IDs for each attachment. A shared
match has one `/clock` authority. All consumers use `use_sim_time=true`.

| Interface | Owner and meaning |
|---|---|
| Livox `CustomMsg` | Simulator's existing measured, distorted local points; default 10 Hz batching. |
| `sensor_msgs/PointCloud2` | Optional conversion of the same measured points for costmaps; preserve and document time fields. |
| `sensor_msgs/Imu` | Future sampled/noisy acceleration and angular velocity; needed by LiDAR-inertial stacks. |
| Wheel feedback / `nav_msgs/Odometry` | Future encoder-derived estimate, with covariance; slip remains estimator error. |
| `/tf_static` body-to-sensor | Configured physical mounting extrinsics. |
| `/tf` odom-to-body | Exactly one odometry estimator, or an explicitly selected truth bootstrap. |
| `/tf` map-to-odom | External localization/SLAM, or explicit known-map bootstrap. |
| `geometry_msgs/TwistStamped` | External body-frame x/y velocity and z yaw rate; configurable unstamped compatibility. |
| Goal / path / status | External planner actions and diagnostics, mapped to team map intent when enabled. |
| Fold/stow request and state | Separate robot mechanism channel with achieved-state feedback. |

Simulator FLU matches the intended ROS body forward/left/up axes. `linear.x`,
`linear.y` and `angular.z` map to forward m/s, left m/s and CCW rad/s; no Bevy
coordinate transform belongs in this adapter. Enforce a configured body frame
for stamped commands. World-frame commands require an explicit transform.
Reject unsupported vertical/roll/pitch components rather than silently treating
a three-dimensional command as planar. Clamp or reject motion using documented
per-robot limits.

Nav2 expects an odometry source and the `map -> odom -> base_link` transform
chain; see its [state-estimation contract](https://docs.nav2.org/rolling/getting_started/navigation_concepts/state_estimation/).
Jazzy deployments differ in stamped velocity configuration: configure both ends
explicitly rather than assuming a default. The
[Jazzy loopback simulator reference](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_loopback_sim/)
documents `enable_stamped_cmd_vel` and its older-distribution default. Keep our
Rapier world as the plant; running that loopback simulator as another plant would
bypass the dynamics being tested.

Start with an explicitly labeled truth-localized mode to isolate controller and
bridge failures. Publish truth on evaluation-only topics in sensor-localized
mode; never also publish perfect `map -> odom` beside SLAM. A LiDAR-only planner
can use the existing scan output, but FAST-LIO-style LiDAR-inertial localization
requires the missing IMU adapter. Livox messages alone do not complete that stack.

### Arena-specific planning

An omni Sentry needs lateral velocity support. A conventional 2D navigation
stack can serve as a baseline on open ground, but the arena also needs terrain
segmentation and explicit constrained tunnel transitions. Do not project every
triangle onto one occupancy plane: roof surfaces would block usable tunnels.
Keep floor support, overhead clearance and dynamic obstacles distinct.

Expose tunnel entry/exit, alignment, fold, transit and unfold as a staged behavior
with measured completion, timeouts and cancellation. The external planner must
own route selection in this mode; the existing CAD-based tunnel planner is a
separately selected baseline. A static maximum-size footprint is conservative;
a smaller folded footprint must only activate after posture feedback confirms
clearance, with yaw restricted through narrow passages.

### Implementation order and acceptance

1. Host control lease, bounded velocity ingress and stale-command brake. Test
   manual takeover, ordering, disconnect, pause/resume and robot identity changes.
2. Live-host scan attachment and bounded acquisition/publication workers. Test
   clock alignment and discontinuities under folding, spawns and queue pressure.
3. ROS body commands, TF and an opt-in truth odometry bootstrap. Demonstrate an
   external controller driving the same robot while another player is in-game.
4. PointCloud2 conversion and a known-map navigation demo, then encoder/IMU models
   and actual external localization. Do not label truth-assisted tests as SLAM.
5. All three tunnel routes, moving opponents and mechanism-aware navigation.

The first end-to-end acceptance run should show external navigation moving the
live in-game Sentry around CAD obstacles, manual takeover stopping it, and a
killed bridge braking within the lease timeout. Record goal error, collisions,
real-time factor, command age and scan drops. A later sensor-localized run must
pass with truth topics unavailable to the external stack. These are required
future tests, not results from the existing bare-world DDS smoke test.

## Joint controls without a new backend

The existing wheel torque input and wheel speed/load/slip output are real parts
of the Rapier plant. Gimbal aim is a stabilized target with bounded prescribed
dynamics; arm motion follows prescribed joint trajectories. Neither currently
exposes a general articulated torque plant.

The next model interface should identify joints and actuators separately, with
stable IDs, local frames, axes, joint kind, limits and sign/gear conventions.
Torque, velocity and position requests need explicit distinct control modes.
Feedback should carry actual joint position/velocity, applied actuator effort,
limits/fault status and an acquisition timestamp. Rotary and prismatic units
must remain distinct. Requested torque is not measured applied effort, and
neither implies a current measurement without an electrical model.

The passive renderer already accepts body-relative named source-link poses in
`ChassisAppearance::source_joints`. That is a useful output boundary regardless
of which backend advances those links.

## Deferred MuJoCo findings

A small optional headless Rust adapter is viable. `davidhozic/mujoco-rs` exposes
safe model/data wrappers, defaults to no viewer features and declares a permissive
dual license. Its inspected manifest targets MuJoCo 3.12.0 and Rust 1.95; this
checkout used Rust 1.99. Pin matching library/binding versions and package the
native library explicitly. This is an inspected candidate, not a tested dependency.
[Binding manifest](https://github.com/davidhozic/mujoco-rs/blob/main/Cargo.toml)

Export an MJCF kinematic tree from reviewed source joints, supplemented with
link mass, center of mass, inertia tensors, joint friction/damping, motor limits
and transmission data. Existing CAD semantics supply axes and motion groups,
not that complete dynamic description. Start with a two-axis gimbal or fixed-base
arm, rather than importing the whole arena. MuJoCo supports URDF but recommends
MJCF for its richer modeling features.
[Modeling guide](https://mujoco.readthedocs.io/en/stable/modeling.html)

Control `ctrl` inputs and step explicitly with `mj_step`; read actual `qpos`,
`qvel` and sensor data. Cache named joint/actuator addresses once; free/ball
joints have different position and velocity widths. `ctrl` units depend on the
actuator/transmission. Joint-side effort uses `qfrc_actuator` or the matching
joint sensor, whereas `actuator_force` is actuator-side scalar force. Preserve
that distinction with gears or multiple actuators.
[Simulation loop](https://mujoco.readthedocs.io/en/stable/programming/simulation.html),
[joint effort sensors](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-jointactuatorfrc)

MJCF collision meshes use convex hulls. A hull of a complete arena or tunnel
assembly can fill its passages. Export primitive/convex compounds preserving
separate floors, supports and roofs; use heightfields only where a single-valued
surface is appropriate. Omni rollers still require a deliberate contact model;
changing engines alone does not validate their lateral behavior.
[Mesh collision contract](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh)

Initially use one authoritative MuJoCo rig in isolation and send its link poses
to the existing renderer. Full-match use subsequently requires a world backend
boundary for lifecycle, contacts, muzzle/armor poses and checkpoints. Avoid
advancing the same interacting robot in both Rapier and MuJoCo with pose copying;
that loses reciprocal contact dynamics. Backend checkpoints must include native
integration/actuator state, the model hash, explicit time and caller-owned
controller/sensor state. MuJoCo provides `mj_getState`/`mj_setState` and an
integration-state signature including warm starts and inputs.
[State types](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjtstate),
[state API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-getstate)
