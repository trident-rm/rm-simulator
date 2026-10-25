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
nix develop --command mbx run -p rm-sim-physics --example jointed_turret --locked
nix develop --command mbx test -p rm-sim-physics --test jointed_turret --locked
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

## Source, prepared resource and live instance

`description(dynamic_base)` builds an engine `AssemblyDescription`: three ordered
`AssemblyLink` entries with explicit rigid-body descriptions and initial states,
and two `AssemblyJoint` entries forming a rooted tree through source IDs. Initial body poses
and velocities use shared assembly-local FLU coordinates. The base starts at
z = 1 m in that frame. Joint anchor frames remain local to their own bodies;
they are not assembly-space poses.

`CookedAssembly::new` prepares the description once and retains its source.
Cooking creates reusable backend geometry, not world bodies or live solver state.
The scenario captures this immutable resource and an `AssemblyKeys` mapping.
`WorldPhysics::install_assembly` installs it transactionally at a supplied placement:
the placement maps assembly-local initial poses and velocities into world FLU.
It leaves body-local colliders, inertial frames and joint anchors local. One
prepared resource can supply multiple independently actuated live instances.

Source `AssemblyLinkId`/`AssemblyJointId` values identify entries within a reusable
description. `BodyKey`/`JointKey` values address a particular instance in a world;
`AssemblyKeys` explicitly maps between these identity spaces. The turret keeps
its original `BASE`/`YOKE`/`BARREL` and `YAW`/`PITCH` runtime keys. A second instance
uses disjoint runtime keys while sharing all source IDs and prepared geometry.
The returned `AssemblyBinding` resolves source IDs to opaque live body/joint
tokens. `remove_assembly` removes that installed instance; it does not alter the
prepared resource. Reacquire tokens after a scenario reset.

## Frames, actuation and lifetime

`BodyKind::Fixed` means immovable in the world, whereas a fixed joint locks the
relative motion of two bodies. Moving rigid CAD parts must not each be declared
as fixed world bodies. Their mass/inertia and rigid grouping need an explicit
physical model before import.

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

Caller-selected `BodyKey` and `JointKey` identify live recipe entries. Runtime bindings
are opaque, process-local installation tokens. A recipe reset reconstructs bodies
and joints at time zero; reacquire every binding by key. Old body/joint tokens
must fail, including actuation through an old token. Reset is recipe replay,
not continuation from an arbitrary saved physical state.

## What the tests establish

`tests/jointed_turret.rs` reuses the example's recipe. It checks both-axis target
tracking and reversal, world-space barrel direction, anchor separation below
3 mm, both signs of held effort at the hard stops, and repeatable response after
reset. A dynamic-pedestal variant verifies counter-rotation under motor torque
and support of the connected assembly after floor impact. A further test installs
one prepared turret resource twice with disjoint keys, translation and yaw
rotation, checks the placed body frames, actuates the instances independently,
and repeats after reset using the captured resource and fresh live tokens. These checks use
bounded solver tolerances rather than expecting exact kinematic constraints.

The optional [SolidWorks workbench](solidworks-importer.md#recover-candidate-constraints-and-play-mechanism-states)
can propose single-axis constraints from saved poses and test reviewed source
subsets with diagnostic motors. It does not decode native mates or infer travel
stops. Joint checkpoint integration remains unavailable. Generic joint
state reads are privileged physical truth, not sensors. This fixture does not
supply a robot controller, competition actuator parameters, collision fitting,
or a multiplayer serialization format. Existing native integration slices,
checkpoint semantics and prescribed game mechanisms keep their current contract.

## Torque commands and servo response

`PhysicalRuntime::set_joint_effort` accepts signed output-shaft torque in N m
(revolute) or force in N (prismatic). It replaces the PD motor and holds the
command until changed. Every solver slice applies equal and opposite effort to
the two bodies along the current parent-joint axis. Commands do not prescribe
joint position or velocity. Inertia, gravity, external loads, connected bodies,
contacts and hard stops determine the resulting motion. Direct effort has no
implicit motor torque limit; the controller must enforce its actuator limits.

`joint_state` returns position and velocity. `joint_applied_effort` reports the
held effort or saturated PD output applied at the start of the **last solver
slice**, initially zero. It allocates no output buffer. Commands do not change
this observation until stepping. A 128 Hz tick can contain several solver
slices; this observation is neither a tick average nor an integrated impulse.
A physical stop can cancel acceleration while reported actuator torque stays
nonzero. It is not a joint reaction sensor: constraint impulses, gravity and
external loads are excluded. Constraint reaction/wrench feedback is not yet
exposed through the generic joint API.

Run a centered single-rotor experiment without CAD or contacts:

```sh
cargo run -p sim-physics --example servo_response --locked -- response.csv
cargo test -p sim-physics --test servo_response --locked
```

The illustrative rotor has 0.02 kg m² principal inertia about each axis, with
its COM on the hinge. The example uses a 1000 Hz caller clock, ±0.5 rad stops,
a 4 N m/rad proportional gain, 0.4 N m s/rad damping and ±0.5 N m actuator
saturation. It changes the target from +0.3 to −0.2 rad and applies an external
−0.2 N m load for one second. CSV columns distinguish target, position, velocity,
applied actuator torque and the known external test torque.

Tests at 128 and 1000 Hz compare constant torque against
`angular_velocity = (actuator_torque + external_torque) * time / inertia`.
With +0.2 N m and no load, acceleration is 10 rad/s². An opposing −0.2 N m
load cancels it. Under the bounded PD servo, a −0.2 N m load requires a
0.05 rad steady error with the stated proportional gain: target +0.3 rad
settles at +0.25 rad while the actuator supplies +0.2 N m. Tests also cover
saturation, clearing held commands, stale bindings and pushing against stops.

This establishes a mechanical plant for in-process embedded controller tests.
It does not identify a particular servo. There is no current/voltage loop,
back-EMF or speed-dependent torque envelope, winding temperature, gearbox
loss/backlash, encoder quantization, communication delay or calibrated friction
in this fixture. An MCU-facing controller can supply bounded effort each tick;
an electrical actuator model can later convert current commands into output
shaft torque without changing the joint mechanics. Keep controller sample timing
explicit rather than treating the built-in per-slice PD law as MCU firmware.

A [matched MuJoCo comparison](mujoco-servo-comparison.md) records numerical
agreement, analytical checks, stop compliance and backend tradeoffs. Pass `128`
or `1000` after the CSV path to select the response example caller rate.
