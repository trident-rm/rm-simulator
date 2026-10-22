<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Compact physical telemetry

Robotics and training callers can inspect privileged physical truth without
cloning a complete `ChassisSnapshot`, its configuration, articulation hulls or
wheel arrays. This API uses the same physical world and explicit session clock;
it is not a measured sensor, team-visible information or a physical checkpoint.

`WorldPhysics::chassis_telemetry(id)` and `Field::chassis_telemetry(id)` return a
`chassis::ChassisTelemetry` view. `RobotSession::robot_telemetry` and
`MatchSession::robot_telemetry` forward that view at their current `stamp()`.
Unknown identities return `None`. The view borrows its world, so Rust prevents
stepping or resetting that world while the view remains in use.

```rust
use rm_simulator_physics::{WorldPhysics, Pose, Team, chassis::ChassisConfig};
let mut world = WorldPhysics::new(&[], 0.);
let id = world.add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 0., 0.4]))?;
let mut wheels = Vec::new(); // Retain this buffer across acquisitions.
let truth = world.chassis_telemetry(id).unwrap();
truth.wheels_into(&mut wheels);
assert_eq!(truth.pose, world.chassis_snapshot(id).unwrap().pose);
# Ok::<(), &'static str>(())
```

Source-backed turret extraction evaluates only the pitch joint’s ancestor chain,
bounded by the validated 64-joint limit, without building full pose vectors.
Body/turret poses, velocities, held inputs, placement revision, defeat, measured
motor power and gimbal/folding coordinates are copied scalar values.
`drive_control` borrows its held mode/request array. `wheels()` derives each
`WheelSnapshot` on demand, including its world-space hub, without allocation.
`wheels_into` clears and fills caller storage; it allocates only when capacity is
insufficient. Wheel contacts are the existing ray-cast suspension observations,
not Rapier contact manifolds or solver warm starts. Gimbal position/rate and
folding position describe the existing prescribed mechanism model; there is no
arbitrary articulated joint torque plant, folding velocity estimate or assembly
arm observation added here.

`WorldPhysics::projectile_states()` yields copied scalar `ProjectileSnapshot`
values, oldest first, without allocation. `projectile_states_into` replaces
caller output and retains its allocation when capacity suffices. Existing
`snapshot()` remains compatible and collects the same iterator.
`payload_telemetry(id)` returns copied identity/motion plus a borrowed
`PayloadShape`, avoiding cloning nested convex hull buffers.

Observation never drains events. `armor_collisions()` borrows pending raw armor
collisions in substep order; `removals()` iterates pending bounded projectile
removal records oldest first. `drain_armor_collisions` and `drain_removals`
explicitly append to caller storage and empty their queues while retaining queue
capacity. Existing `take_*` APIs keep their original behavior. Projectile armor
contacts returned by `step`/`step_into` already belong to the caller: physics has
no second pending projectile contact queue. A live Field consumes its physical
contact events through ordinary scoring; these APIs do not bypass that owner.

`carrier_trace()` borrows pending IMU samples (`None` if disabled); it does not
consume them. Existing `drain_carrier_trace` appends to caller storage and retains
the internal allocation. Existing LiDAR `sample_into`, rolling scan and IMU
sampling APIs remain the sensor output path. LiDAR scene captures still construct
owned coherent captures with shared shape buffers; compact telemetry does not
make those captures allocation-free or change sensor interpolation/reset rules.

Run the dependency-free extraction comparison with:

```sh
cargo run --release -p rm-simulator-physics --example telemetry_cost --locked
```

It compares complete snapshot cloning, scalar/borrowed view construction and a
retained wheel buffer on a folding Hero and an eight-joint fixture using the
source articulation path (no CAD import or calibrated robot dimensions).
Timings measure extraction only, not
stepping, and the view-only case does not derive wheels. Pointer/capacity checks
verify retained output storage; allocation counts are not instrumented. All new
view/iterator reads avoid heap allocation by construction; filling/draining
caller buffers may grow them. Full snapshots and physical integration can still
allocate independently.
