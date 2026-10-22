// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Mounted per-point acquisitions over a bracket of immutable physics captures.
//! Capture every integration step. Rigid translations and shortest-arc rotations
//! are interpolated; topology or shape changes require a new acquisition bracket.
//! Robot body geometry remains present, including its self-occlusion. Output is
//! in each point's sensor frame and supplies no perfect localization estimate.
use super::{
    MAX_LIDAR_RAYS,
    mid360::{Mid360Pattern, PatternWindow},
};
use crate::{
    Pose, StaticGeometry,
    geometry::RayHit,
    projectile::{pose, pose_is_valid},
};
use rapier3d_f64::prelude::{
    Aabb, ColliderHandle, Pose as Pose3, Ray, RayCast, SharedShape, Vector,
};

#[derive(Clone)]
pub(crate) struct SensorCollider {
    pub handle: ColliderHandle,
    pub pose: Pose3,
    pub shape: SharedShape,
}
/// Immutable, timestamped sensor ground truth from one physical world.
/// Capture handles are only meaningful within that live world; do not bracket
/// a reset/restore with old captures. Mesh buffers are shared and poses frozen.
#[derive(Clone)]
pub struct SceneCapture {
    robot: u32,
    placement_revision: u64,
    robot_revisions: Vec<(u32, u64)>,
    time_ns: u64,
    robot_pose: Pose3,
    fixed: Vec<SensorCollider>,
    moving: Vec<SensorCollider>,
    fixed_geometry: StaticGeometry,
}
impl SceneCapture {
    pub(crate) fn new(
        robot: u32,
        placement_revision: u64,
        time_ns: u64,
        robot_pose: Pose,
        fixed: Vec<SensorCollider>,
        moving: Vec<SensorCollider>,
        robot_revisions: Vec<(u32, u64)>,
    ) -> Self {
        let fixed_geometry = StaticGeometry {
            shapes: fixed.iter().map(|c| (c.pose, c.shape.clone())).collect(),
            ..Default::default()
        };
        Self {
            robot,
            placement_revision,
            robot_revisions,
            time_ns,
            robot_pose: pose(robot_pose),
            fixed,
            moving,
            fixed_geometry,
        }
    }
    /// Use an actual articulated carrier pose for this capture's sensor mount.
    /// Geometry, robot identity and placement revisions remain unchanged. Supply
    /// the physical carrier pose at each capture time (FLU metres, unit wxyz),
    /// never an independently advanced animation or a discontinuous placement.
    /// The ordinary acquisition interpolates this frame per emission.
    ///
    /// ```
    /// use rm_simulator_physics::{WorldPhysics, Pose, Team, chassis::ChassisConfig};
    /// let mut world = WorldPhysics::new(&[], 0.);
    /// let robot = world.add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0.,0.,0.3]))?;
    /// let head = Pose::yawed([0.,0.,0.5], 0.2);
    /// let scene = world.lidar_scene_capture(robot, 0)?.with_mount_frame(head)?;
    /// assert_eq!(scene.time_ns(), 0);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn with_mount_frame(mut self, carrier: Pose) -> Result<Self, &'static str> {
        if !pose_is_valid(carrier) {
            return Err("invalid LiDAR carrier pose");
        }
        self.robot_pose = pose(carrier);
        Ok(self)
    }
    /// Caller-supplied capture time, nanoseconds. The world facade stamps this
    /// from its actual simulation clock.
    pub fn time_ns(&self) -> u64 {
        self.time_ns
    }
}

/// Generic reproducible range effects, not a calibrated material response model.
/// Counter-based sampling is independent of acquisition chunking and misses.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RangeEffects {
    /// Sensor-specific seed; use different seeds for different simulated devices.
    pub seed: u64,
    /// Gaussian range standard deviation, metres, finite and nonnegative.
    pub standard_deviation_m: f64,
    /// Constant range offset, metres, finite; may be negative.
    pub bias_m: f64,
    /// Probability of suppressing a geometrically valid return, in [0, 1].
    pub dropout_probability: f64,
}
impl RangeEffects {
    fn validate(self) -> Result<(), &'static str> {
        if !self.standard_deviation_m.is_finite()
            || self.standard_deviation_m < 0.
            || !self.bias_m.is_finite()
            || !self.dropout_probability.is_finite()
            || !(0. ..=1.).contains(&self.dropout_probability)
        {
            return Err("invalid LiDAR range noise, bias or dropout probability");
        }
        Ok(())
    }
    fn uniform(self, index: u64, stream: u64) -> f64 {
        // SplitMix64 finalizer with disjoint counter streams, not shared RNG state.
        let mut x = index.wrapping_mul(0x9e3779b97f4a7c15) ^ self.seed ^ stream;
        x = (x ^ (x >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        x = (x ^ (x >> 27)).wrapping_mul(0x94d049bb133111eb);
        x ^= x >> 31;
        // Strictly inside (0,1); avoid log(0) and rounded 1.0.
        ((x >> 12) as f64 + 0.5) / (1u64 << 52) as f64
    }
    fn range(self, index: u64, actual_m: f64) -> Option<f64> {
        if self.uniform(index, 0x10f8ab65) < self.dropout_probability {
            return None;
        }
        let gaussian = if self.standard_deviation_m == 0. {
            0.
        } else {
            (-2. * self.uniform(index, 0x9205d34a).ln()).sqrt()
                * (std::f64::consts::TAU * self.uniform(index, 0xe38b1c79)).cos()
        };
        Some(actual_m + self.bias_m + self.standard_deviation_m * gaussian)
    }
}

/// One emission, including explicit misses. Coordinates are in the sensor frame
/// at this point's time; accumulating them requires estimated motion/deskewing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimedReturn {
    /// Global pattern index, retained through deterministic decimation.
    pub sample_index: u64,
    /// Absolute simulation emission time, nanoseconds, including the sensor epoch.
    pub time_ns: u64,
    /// Measured range in metres, absent for a miss, blind zone, dropout or noisy
    /// measurement outside configured range. No hidden surface leaks on a miss.
    pub range_m: Option<f64>,
    /// Noisy measured point in local sensor FLU metres at emission, or `None`.
    pub point_sensor_m: Option<[f64; 3]>,
}
/// Bounded acquisition storage. No per-point ground-truth robot/world poses are
/// exposed; the caller must provide its own motion estimate for deskewing.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RollingFrame {
    /// Points in requested pattern-window order, with timestamps even for misses.
    pub points: Vec<TimedReturn>,
}
/// Mounted MID-360 approximation with per-point rigid scene motion and optional
/// seeded range noise/dropout. Configuration is immutable; there is no hidden
/// pattern cursor or RNG state to checkpoint.
///
/// ```
/// use rm_simulator_physics::{Pose, Team, WorldPhysics, TargetFrames, tick_ns,
///     chassis::ChassisConfig,
///     sensors::{mid360::Mid360Pattern,
///         rolling::{MountedLidar, RangeEffects, RollingFrame}}};
/// let mut world = WorldPhysics::new(&[], 0.);
/// let robot = world.add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0.,0.,0.4]))?;
/// let sensor = MountedLidar::new(Mid360Pattern::new(0.1,40.)?,
///     Pose::at([0.,0.,0.4]), 0, RangeEffects::default())?;
/// let before = world.lidar_scene_capture(robot, 0)?;
/// world.step(0, &TargetFrames::new(vec![]))?;
/// let after = world.lidar_scene_capture(robot, tick_ns())?;
/// let window = sensor.window_between(0, tick_ns(), 1)?.unwrap();
/// let mut scan = RollingFrame::default();
/// sensor.sample_into(&before, &after, window, &mut scan)?;
/// assert_eq!(scan.points.len(), 1563);
/// assert!(scan.points.iter().all(|p| p.time_ns < tick_ns()));
/// # Ok::<(), &'static str>(())
/// ```
pub struct MountedLidar {
    pattern: Mid360Pattern,
    robot_from_sensor: Pose3,
    epoch_ns: u64,
    effects: RangeEffects,
}
impl MountedLidar {
    /// Select emissions in `[start_ns, end_ns)`, aligned to the sensor's epoch
    /// and global decimation stride. Consecutive brackets neither duplicate nor
    /// skip selected points, even at step boundaries not divisible by 5 µs.
    /// Returns `None` when no selected emission falls in the interval.
    pub fn window_between(
        &self,
        start_ns: u64,
        end_ns: u64,
        stride: u32,
    ) -> Result<Option<PatternWindow>, &'static str> {
        use super::mid360::POINT_INTERVAL_NS;
        if start_ns >= end_ns || stride == 0 {
            return Err("invalid LiDAR acquisition interval or stride");
        }
        if end_ns <= self.epoch_ns {
            return Ok(None);
        }
        let first = start_ns
            .saturating_sub(self.epoch_ns)
            .div_ceil(POINT_INTERVAL_NS);
        let stride = u64::from(stride);
        let first = first
            .div_ceil(stride)
            .checked_mul(stride)
            .ok_or("LiDAR index overflow")?;
        let end = (end_ns - self.epoch_ns).div_ceil(POINT_INTERVAL_NS);
        if first >= end {
            return Ok(None);
        }
        let count = (end - 1 - first) / stride + 1;
        if count > MAX_LIDAR_RAYS as u64 {
            return Err("LiDAR interval exceeds point budget; split captures");
        }
        Ok(Some(PatternWindow {
            first_sample: first,
            samples: count as u32,
            stride: stride as u32,
        }))
    }
    /// Mount a validated pattern on the chassis frame. Pose uses FLU metres and
    /// unit wxyz rotation. `epoch_ns` is the nominal emission time of sample zero.
    pub fn new(
        pattern: Mid360Pattern,
        robot_from_sensor: Pose,
        epoch_ns: u64,
        effects: RangeEffects,
    ) -> Result<Self, &'static str> {
        if !pose_is_valid(robot_from_sensor) {
            return Err("invalid LiDAR mounting pose");
        }
        effects.validate()?;
        Ok(Self {
            pattern,
            robot_from_sensor: pose(robot_from_sensor),
            epoch_ns,
            effects,
        })
    }
    /// Acquire points in the half-open bracket `[before.time_ns, after.time_ns)`.
    /// Both captures must belong to the same live robot/world, with identical
    /// enabled collider topology and shape buffers; geometry changes are errors.
    /// Each ray uses interpolated chassis and moving-collider poses at its own
    /// emission time. Fixed geometry retains its cached ray acceleration.
    ///
    /// Invalid inputs leave the output unchanged. At a bracket boundary use the
    /// next bracket, avoiding duplicate points. For best accuracy bracket one
    /// integration step; large rotations above π cannot be inferred from endpoints.
    pub fn sample_into(
        &self,
        before: &SceneCapture,
        after: &SceneCapture,
        window: PatternWindow,
        output: &mut RollingFrame,
    ) -> Result<(), &'static str> {
        if window.samples == 0 || window.samples as usize > MAX_LIDAR_RAYS || window.stride == 0 {
            return Err("invalid rolling LiDAR window");
        }
        let last = window
            .first_sample
            .checked_add(u64::from(window.samples - 1) * u64::from(window.stride))
            .ok_or("LiDAR index overflow")?;
        let stamp = |index| {
            Mid360Pattern::point_offset_ns(index).and_then(|v| v.checked_add(self.epoch_ns))
        };
        let first_time = stamp(window.first_sample).ok_or("LiDAR timestamp overflow")?;
        let last_time = stamp(last).ok_or("LiDAR timestamp overflow")?;
        let scene = SceneBracket::new(before, after)?;
        if first_time < before.time_ns || last_time >= after.time_ns {
            return Err("LiDAR emissions outside capture bracket");
        }
        let (min, max) = self.pattern.ranges_m();
        output.points.clear();
        output.points.reserve(window.samples as usize);
        for n in 0..window.samples {
            let index = window.first_sample + u64::from(n) * u64::from(window.stride);
            let time_ns = stamp(index).expect("validated timestamp range");
            let fraction =
                (time_ns - before.time_ns) as f64 / (after.time_ns - before.time_ns) as f64;
            let sensor =
                before.robot_pose.lerp(&after.robot_pose, fraction) * self.robot_from_sensor;
            let direction = Vector::from(self.pattern.direction(index));
            let ray = Ray::new(sensor.translation, sensor.rotation * direction);
            let range_m = scene
                .cast(&ray, max, fraction)
                .filter(|h| h.distance_m >= min)
                .and_then(|h| self.effects.range(index, h.distance_m))
                .filter(|v| v.is_finite() && *v >= min && *v <= max);
            output.points.push(TimedReturn {
                sample_index: index,
                time_ns,
                range_m,
                point_sensor_m: range_m.map(|r| (direction * r).to_array()),
            });
        }
        Ok(())
    }
}
struct SceneBracket<'a> {
    before: &'a SceneCapture,
    moving: Vec<(&'a SensorCollider, &'a SensorCollider, Aabb)>,
}
impl<'a> SceneBracket<'a> {
    fn new(before: &'a SceneCapture, after: &'a SceneCapture) -> Result<Self, &'static str> {
        if before.time_ns >= after.time_ns
            || before.robot != after.robot
            || before.placement_revision != after.placement_revision
            || before.robot_revisions != after.robot_revisions
        {
            return Err("invalid LiDAR scene times, robot identity or placement continuity");
        }
        let matching = |a: &SensorCollider, b: &SensorCollider| {
            a.handle == b.handle && std::ptr::addr_eq(a.shape.as_ref(), b.shape.as_ref())
        };
        if before.fixed.len() != after.fixed.len()
            || before.moving.len() != after.moving.len()
            || before.fixed.iter().any(|a| {
                !after
                    .fixed
                    .iter()
                    .any(|b| matching(a, b) && a.pose == b.pose)
            })
        {
            return Err("LiDAR fixed geometry or topology changed; recapture at the boundary");
        }
        let mut moving = Vec::with_capacity(before.moving.len());
        for a in &before.moving {
            let b = after
                .moving
                .iter()
                .find(|b| matching(a, b))
                .ok_or("LiDAR moving topology or shape changed")?;
            // Conservative swept bounds: the union of endpoint AABBs plus the
            // maximum rotation displacement from the start orientation. This
            // retains intermediate rotated corners, unlike endpoint union alone.
            let local = a.shape.compute_local_aabb();
            let radius = local.mins.abs().max(local.maxs.abs()).length();
            let dot = a.pose.rotation.dot(b.pose.rotation).abs().min(1.);
            let margin = Vector::splat(2. * radius * (1. - dot * dot).max(0.).sqrt() + 1e-10);
            let start = a.shape.compute_aabb(&a.pose);
            let end = b.shape.compute_aabb(&b.pose);
            let bounds = Aabb::new(
                start.mins.min(end.mins) - margin,
                start.maxs.max(end.maxs) + margin,
            );
            moving.push((a, b, bounds));
        }
        Ok(Self { before, moving })
    }
    fn cast(&self, ray: &Ray, max: f64, fraction: f64) -> Option<RayHit> {
        let mut best = self.before.fixed_geometry.cast_metric_ray(ray, max);
        for (i, (a, b, bounds)) in self.moving.iter().enumerate() {
            let limit = best.map_or(max, |h| h.distance_m);
            if bounds.cast_local_ray(ray, limit, true).is_none() {
                continue;
            }
            let transform = a.pose.lerp(&b.pose, fraction);
            if let Some(hit) = a
                .shape
                .cast_ray_and_get_normal(&transform, ray, limit, true)
                && best.is_none_or(|old| hit.time_of_impact < old.distance_m)
            {
                best = Some(RayHit {
                    distance_m: hit.time_of_impact,
                    position_m: ray.point_at(hit.time_of_impact).to_array(),
                    normal: hit.normal.to_array(),
                    shape_index: self.before.fixed.len() + i,
                });
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Team, WorldPhysics, chassis::ChassisConfig};
    fn rig() -> (WorldPhysics, u32) {
        let mut world = WorldPhysics::new(&[], 0.);
        let robot = world
            .add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 0., 0.3]))
            .unwrap();
        world.add_static_box(Pose::at([-5., 0., 3.]), [1., 100., 100.]);
        (world, robot)
    }
    fn sensor(mount: Pose, effects: RangeEffects) -> MountedLidar {
        MountedLidar::new(Mid360Pattern::new(0.1, 40.).unwrap(), mount, 0, effects).unwrap()
    }
    #[test]
    fn scene_capture_pose_and_placement_follow_live_chassis() {
        let (mut world, robot) = rig();
        for target in [Pose::yawed([1., 2., 0.3], 0.7), Pose::at([-1., 0., 0.4])] {
            world.place_chassis(robot, target).unwrap();
            let expected = world.chassis_snapshot(robot).unwrap();
            let scene = world.lidar_scene_capture(robot, 123).unwrap();
            assert_eq!(scene.robot_pose, pose(expected.pose));
            assert_eq!(scene.placement_revision, expected.placement_revision);
            assert_eq!(scene.time_ns(), 123);
            assert_eq!(
                scene.robot_revisions,
                vec![(robot, expected.placement_revision)]
            );
            assert!(!scene.moving.is_empty());
            assert!(!scene.fixed.is_empty());
        }
        assert!(world.lidar_scene_capture(u32::MAX, 0).is_err());
    }
    #[test]
    fn articulated_carrier_changes_rays_without_moving_chassis_geometry() {
        let (world, robot) = rig();
        let a = Pose::yawed([0., 0., 0.5], 0.2);
        let b = Pose::yawed([0., 0., 0.5], 0.3);
        let original = world.lidar_scene_capture(robot, 0).unwrap();
        let before = original.clone().with_mount_frame(a).unwrap();
        assert_eq!(before.moving[0].pose, original.moving[0].pose);
        let after = world
            .lidar_scene_capture(robot, 1_000_000)
            .unwrap()
            .with_mount_frame(b)
            .unwrap();
        let sensor = sensor(Pose::at([0., 0., 1.]), RangeEffects::default());
        let mut frame = RollingFrame::default();
        sensor
            .sample_into(
                &before,
                &after,
                sensor.window_between(0, 1_000_000, 1).unwrap().unwrap(),
                &mut frame,
            )
            .unwrap();
        let mut hits = 0;
        for p in frame.points {
            if let Some(local) = p.point_sensor_m {
                let f = p.time_ns as f64 / 1_000_000.;
                let world = (pose(a).lerp(&pose(b), f) * sensor.robot_from_sensor)
                    .transform_point(Vector::from(local));
                assert!((world.x + 4.5).abs() < 1e-8);
                hits += 1;
            }
        }
        assert!(hits > 20);
        assert!(
            original
                .with_mount_frame(Pose {
                    rotation_wxyz: [0.; 4],
                    ..a
                })
                .is_err()
        );
    }
    #[test]
    fn mounted_points_follow_translation_rotation_and_local_frame() {
        let (world, robot) = rig();
        let before = world.lidar_scene_capture(robot, 0).unwrap();
        let mut after = world.lidar_scene_capture(robot, 1_000_000).unwrap();
        after.robot_pose = pose(Pose::yawed([-1., 0., 0.3], 0.2));
        // Interpolate a prescribed rigid displacement of this robot's geometry.
        let delta = after.robot_pose * before.robot_pose.inverse();
        for c in &mut after.moving {
            c.pose = delta * c.pose;
        }
        let sensor = sensor(Pose::at([0., 0., 1.]), RangeEffects::default());
        let mut frame = RollingFrame::default();
        let window = sensor.window_between(0, 1_000_000, 1).unwrap().unwrap();
        sensor
            .sample_into(&before, &after, window, &mut frame)
            .unwrap();
        assert_eq!(frame.points.len(), 200);
        for point in &frame.points {
            let f = point.time_ns as f64 / 1_000_000.;
            let mounted = before.robot_pose.lerp(&after.robot_pose, f) * sensor.robot_from_sensor;
            let world_point = mounted.transform_point(Vector::from(point.point_sensor_m.unwrap()));
            assert!((world_point.x + 4.5).abs() < 1e-8);
            let direction = Vector::from(sensor.pattern.direction(point.sample_index));
            assert!(
                (Vector::from(point.point_sensor_m.unwrap()) - direction * point.range_m.unwrap())
                    .length()
                    < 1e-10
            );
        }
        // At later emissions the result differs from a frozen start-pose scan.
        let p = frame.points.last().unwrap();
        let dir = sensor.pattern.direction(p.sample_index);
        assert!((p.range_m.unwrap() - (-4.5 / dir[0])).abs() > 0.1);
    }
    #[test]
    fn own_body_occludes_and_mounting_above_it_restores_returns() {
        let (world, robot) = rig();
        let a = world.lidar_scene_capture(robot, 0).unwrap();
        let b = world.lidar_scene_capture(robot, 1_000_000).unwrap();
        let window = PatternWindow {
            first_sample: 0,
            samples: 200,
            stride: 1,
        };
        let mut inside = RollingFrame::default();
        sensor(Pose::default(), RangeEffects::default())
            .sample_into(&a, &b, window, &mut inside)
            .unwrap();
        assert!(inside.points.iter().all(|p| p.range_m.is_none()));
        let mut above = RollingFrame::default();
        sensor(Pose::at([0., 0., 1.]), RangeEffects::default())
            .sample_into(&a, &b, window, &mut above)
            .unwrap();
        assert!(above.points.iter().all(|p| p.range_m.is_some()));
    }
    #[test]
    fn moving_obstacles_are_interpolated_at_emission() {
        let (mut world, robot) = rig();
        let other = world
            .add_chassis(
                Team::Blue,
                ChassisConfig::default(),
                Pose::at([3., 0., 0.3]),
            )
            .unwrap();
        let a = world.lidar_scene_capture(robot, 0).unwrap();
        let mut b = world.lidar_scene_capture(robot, 1000).unwrap();
        world.place_chassis(other, Pose::at([5., 0., 0.3])).unwrap();
        let moved = world.lidar_scene_capture(robot, 1000).unwrap();
        b.moving = moved.moving;
        let scene = SceneBracket::new(&a, &b).unwrap();
        // Start outside the source robot, pointing along +x at the other chassis.
        let ray = Ray::new(Vector::new(1., 0., 0.3), Vector::X);
        let start = scene.cast(&ray, 10., 0.).unwrap().distance_m;
        assert!((scene.cast(&ray, 10., 0.5).unwrap().distance_m - start - 1.).abs() < 1e-8);
        assert!((scene.cast(&ray, 10., 1.).unwrap().distance_m - start - 2.).abs() < 1e-8);
    }
    #[test]
    fn rotating_corners_are_not_culled_by_endpoint_bounds() {
        let (world, robot) = rig();
        let mut a = world.lidar_scene_capture(robot, 0).unwrap();
        let mut collider = a.moving[0].clone();
        collider.shape = SharedShape::cuboid(2., 0.05, 0.05);
        collider.pose = pose(Pose::yawed([0., 0., 1.], -std::f64::consts::FRAC_PI_4));
        a.moving = vec![collider];
        let mut b = a.clone();
        b.time_ns = 1000;
        b.moving[0].pose = pose(Pose::yawed([0., 0., 1.], std::f64::consts::FRAC_PI_4));
        let scene = SceneBracket::new(&a, &b).unwrap();
        let ray = Ray::new(Vector::new(3., 0., 1.), -Vector::X);
        // At the middle orientation the rod reaches x=2, outside both endpoint
        // AABBs. A query capped at 1.2 m must still find that intermediate face.
        assert!((scene.cast(&ray, 1.2, 0.5).unwrap().distance_m - 1.).abs() < 1e-9);
    }

    #[test]
    fn acquisition_chunking_and_decimation_do_not_change_noise_or_phase() {
        let (world, robot) = rig();
        let sensor = sensor(
            Pose::at([0., 0., 1.]),
            RangeEffects {
                seed: 17,
                standard_deviation_m: 0.02,
                bias_m: 0.003,
                dropout_probability: 0.2,
            },
        );
        let a = world.lidar_scene_capture(robot, 0).unwrap();
        let b = world.lidar_scene_capture(robot, 503_333).unwrap();
        let c = world.lidar_scene_capture(robot, 1_000_000).unwrap();
        let mut whole = RollingFrame::default();
        let mut part = RollingFrame::default();
        sensor
            .sample_into(
                &a,
                &c,
                sensor.window_between(0, 1_000_000, 3).unwrap().unwrap(),
                &mut whole,
            )
            .unwrap();
        sensor
            .sample_into(
                &a,
                &b,
                sensor.window_between(0, 503_333, 3).unwrap().unwrap(),
                &mut part,
            )
            .unwrap();
        let mut joined = part.points.clone();
        sensor
            .sample_into(
                &b,
                &c,
                sensor
                    .window_between(503_333, 1_000_000, 3)
                    .unwrap()
                    .unwrap(),
                &mut part,
            )
            .unwrap();
        joined.extend(part.points);
        assert_eq!(joined, whole.points);
        assert!(whole.points.iter().any(|p| p.range_m.is_none()));
        assert!(whole.points.iter().any(|p| p.range_m.is_some()));
        let effects = RangeEffects {
            seed: 19,
            standard_deviation_m: 0.02,
            ..Default::default()
        };
        let samples: Vec<_> = (0..20000)
            .map(|i| effects.range(i, 10.).unwrap() - 10.)
            .collect();
        let mean = samples.iter().sum::<f64>() / samples.len() as f64;
        let sigma = (samples.iter().map(|v| v * v).sum::<f64>() / samples.len() as f64).sqrt();
        assert!(mean.abs() < 0.001 && (sigma - 0.02).abs() < 0.001);
        assert_ne!(
            effects.range(7, 10.),
            RangeEffects {
                seed: 20,
                ..effects
            }
            .range(7, 10.)
        );
        assert_eq!(
            RangeEffects {
                dropout_probability: 1.,
                ..Default::default()
            }
            .range(7, 10.),
            None
        );
    }
    #[test]
    fn errors_preserve_output_and_reject_teleports_or_topology_changes() {
        let (mut world, robot) = rig();
        let sensor = sensor(Pose::at([0., 0., 1.]), RangeEffects::default());
        let a = world.lidar_scene_capture(robot, 0).unwrap();
        let b = world.lidar_scene_capture(robot, 1_000_000).unwrap();
        let window = PatternWindow {
            first_sample: 0,
            samples: 1,
            stride: 1,
        };
        let mut frame = RollingFrame::default();
        sensor.sample_into(&a, &b, window, &mut frame).unwrap();
        let saved = frame.clone();
        let invalid = PatternWindow {
            first_sample: u64::MAX,
            ..window
        };
        assert!(sensor.sample_into(&a, &b, invalid, &mut frame).is_err());
        assert_eq!(frame, saved);
        let mut future = b.clone();
        future.placement_revision += 1;
        assert!(sensor.sample_into(&a, &future, window, &mut frame).is_err());
        assert_eq!(frame, saved);
        world.add_static_box(Pose::at([5., 5., 1.]), [1.; 3]);
        assert!(
            sensor
                .sample_into(
                    &a,
                    &world.lidar_scene_capture(robot, 1_000_000).unwrap(),
                    window,
                    &mut frame
                )
                .is_err()
        );
        assert_eq!(frame, saved);
        assert!(sensor.window_between(0, u64::MAX, 1).is_err());
        assert!(sensor.window_between(0, 10, 0).is_err());
        assert!(
            RangeEffects {
                standard_deviation_m: f64::NAN,
                ..Default::default()
            }
            .validate()
            .is_err()
        );
    }
}
