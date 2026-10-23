<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Standalone jointed mechanisms

The physics runtime can connect source-described generic bodies with physical
fixed, revolute or prismatic joints. The standalone two-axis turret is a small
fixture for this API. It requires no CAD, gameplay authority, renderer or network.
Current game gimbals, folding mechanisms and assembly playback remain prescribed;
this fixture does not migrate their behavior, rules or control paths. See the
[physics migration plan](physics-migration.md) for the broader work.

## Run the fixture

From the checkout with the development environment available:

```sh
nix develop --command mbx run -p rm-simulator-physics --example jointed_turret --locked
nix develop --command mbx test -p rm-simulator-physics --test jointed_turret --locked
```

`examples/jointed_turret.rs` installs a fixed pedestal, a 2 kg yaw yoke and a
1 kg pitch barrel. Each dynamic body declares its mass, central COM and uniform
box principal inertia explicitly. Colliders do not infer those properties.
The reusable scenario can instead give the pedestal a 10 kg dynamic body for
reaction and floor-contact tests. Dimensions and gains are illustrative fixture
choices, not RoboMaster specifications.

Two revolute joints constrain the bodies in the ordinary shared physics scene.
The example drives yaw to 0.6 rad and pitch to −0.25 rad, then removes the motors
and applies +5 N m held effort against the stops. It prints joint position and
velocity, a wall-time observation for 2000 explicit 1000 Hz ticks, and the initial
pitch state after recipe reset. Timing includes this tiny scene's stepping and
contact collection; it is not a throughput claim for robots or CAD arenas.
One local development-profile run on 2026-10-23 measured 8.30 ms for those
2000 ticks. It settled at yaw 0.6000 rad and pitch −0.2381 rad; the pitch
gravity error was 0.0119 rad. Under +5 N m the pitch stop held at 0.4500085 rad.
These are single-run observations, not a benchmark distribution.

## Frames, actuation and lifetime

A joint description contains parent-local and child-local anchor frames. Their
local +X is the free axis for revolute/prismatic joints. The turret rotates +X
onto pedestal +Z for yaw and yoke +Y for pitch, with matching frame orientations
on both connected bodies. At zero position the two anchor frames coincide in
world space. Positive pitch about +Y points the forward barrel down in FLU.

Limits and state are radians/radians per second for revolute joints and
metres/metres per second for prismatic joints. The turret limits yaw to ±1 rad
and pitch to ±0.45 rad. Motors specify target position and velocity, stiffness,
damping and maximum force (torque for revolute joints). The fixture uses
200 N m/rad stiffness, 10 N m s/rad damping and a 20 N m cap. Its finite stiffness
allows gravity-loaded pitch error; a target is not a prescribed pose.
`set_joint_effort` replaces the motor with held generalized effort;
`set_joint_motor` replaces held effort with the requested motor. Zero effort or
`set_joint_motor(binding, None)` clears actuation. Connected-pair
contacts are disabled in this fixture, while ordinary scenery contacts remain.

Caller-selected `BodyKey` and `JointKey` identify recipe entries. Runtime bindings
are opaque, process-local installation tokens. A recipe reset reconstructs bodies
and joints at time zero; reacquire every binding by key. Old body/joint tokens
must fail, including actuation through an old token. Reset is recipe replay,
not continuation from an arbitrary saved physical state.

## What the tests establish

`tests/jointed_turret.rs` reuses the example's recipe. It checks both-axis target
tracking and reversal, world-space barrel direction, anchor separation below
3 mm, both signs of held effort at the hard stops, and repeatable response after
reset. A dynamic-pedestal variant verifies counter-rotation under motor torque
and support of the connected assembly after floor impact. These checks use
bounded solver tolerances rather than expecting exact kinematic constraints.

There is no joint CAD import or joint checkpoint integration yet. Generic joint
state reads are privileged physical truth, not sensors. This fixture does not
supply a robot controller, competition actuator parameters, collision fitting,
or a multiplayer serialization format. Existing native integration slices,
checkpoint semantics and prescribed game mechanisms keep their current contract.
