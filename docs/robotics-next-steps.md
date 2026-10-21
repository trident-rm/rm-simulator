<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Robotics integration investigation

Investigation following `939316e`, 2026-10-21. MuJoCo implementation is deferred
at the user's request. No MuJoCo, ROS or other integration dependency was added.
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
