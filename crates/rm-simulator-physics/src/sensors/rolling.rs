// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Mounted per-point acquisitions over a bracket of immutable physics captures.
//! Capture every integration step. Rigid translations and shortest-arc rotations
//! are interpolated; topology or shape changes require a new acquisition bracket.
//! Robot body geometry remains present, including its self-occlusion. Output is
//! in each point's sensor frame and supplies no perfect localization estimate.
use super::{
    MAX_LIDAR_RAYS,
    carrier::{CarrierIdentity, StampedCarrierPose},
    mid360::{Mid360Pattern, PatternWindow},
};
use crate::{
    Pose,
    geometry::RayHit,
    projectile::{pose, pose_is_valid},
};
use mid360_sim::effects::AngularNoise;
use rapier3d_f64::parry::partitioning::{Bvh, BvhBuildStrategy, TraversalAction};
use rapier3d_f64::prelude::{
    Aabb, ColliderHandle, Pose as Pose3, Ray, RayCast, SharedShape, Vector,
};
use std::sync::{Arc, OnceLock};

#[derive(Clone)]
pub(crate) struct SensorCollider {
    pub handle: ColliderHandle,
    pub motion_channel: Option<crate::motion::MotionChannel>,
    pub pose: Pose3,
    pub shape: SharedShape,
}
/// Immutable, timestamped sensor ground truth from one physical world.
/// Capture handles are only meaningful within that live world; do not bracket
/// a reset/restore with old captures. Mesh buffers are shared and poses frozen.
#[derive(Clone)]
pub struct SceneCapture {
    runtime_identity: u64,
    // Generic captures retain the runtime's scene-wide discontinuity evidence.
    // Legacy captures retain their original complete chassis revision list.
    scene_placement_epoch: Option<u64>,
    robot: Option<u32>,
    placement_revision: Option<u64>,
    carrier_identity: Option<CarrierIdentity>,
    robot_revisions: Vec<(u32, u64)>,
    time_ns: u64,
    robot_pose: Pose3,
    fixed: Vec<SensorCollider>,
    moving: Vec<SensorCollider>,
    fixed_queries: OnceLock<Arc<FixedQueries>>,
}
impl SceneCapture {
    pub(crate) fn new(
        runtime_identity: u64,
        robot_identity: (u32, u64),
        time_ns: u64,
        robot_pose: Pose,
        fixed: Vec<SensorCollider>,
        moving: Vec<SensorCollider>,
        robot_revisions: Vec<(u32, u64)>,
    ) -> Self {
        Self {
            runtime_identity,
            scene_placement_epoch: None,
            robot: Some(robot_identity.0),
            placement_revision: Some(robot_identity.1),
            carrier_identity: None,
            robot_revisions,
            time_ns,
            robot_pose: pose(robot_pose),
            fixed,
            moving,
            fixed_queries: OnceLock::new(),
        }
    }
    /// Combine an immutable generic runtime capture with an actual resolved
    /// carrier pose. The stamp supplies simulation time; no clock is consulted.
    /// All enabled geometry, including the carrier's own colliders, occludes.
    /// The capture retains the runtime's scene-wide placement epoch so a
    /// discontinuous placement of any obstacle invalidates interpolation, even
    /// when the sensor carrier's own identity and placement remain unchanged.
    ///
    /// The caller's freely constructible world stamp must match the capture's
    /// actual runtime brand. This checks metadata consistency, not authenticity
    /// of a privileged caller's asserted carrier binding or placement revision.
    ///
    /// ```
    /// use rm_simulator_engine::model::CarrierKey;
    /// use rm_simulator_physics::{Pose, StepDuration, runtime::PhysicalRuntime,
    ///     sensors::{carrier::{CarrierIdentity, StampedCarrierPose}, rolling::SceneCapture}};
    /// let runtime = PhysicalRuntime::new(StepDuration::HZ_1000);
    /// let geometry = runtime.geometry_capture();
    /// let identity = CarrierIdentity { carrier: CarrierKey(3),
    ///     world_brand: geometry.runtime_identity(), installation_generation: 1,
    ///     discontinuity_revision: 0 };
    /// let carrier = StampedCarrierPose::new(identity, 0, 0.001, Pose::default())?;
    /// let scene = SceneCapture::from_physical_geometry(&geometry, carrier)?;
    /// assert_eq!(scene.time_ns(), 0);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn from_physical_geometry(
        geometry: &crate::runtime::PhysicalGeometry,
        carrier: StampedCarrierPose,
    ) -> Result<Self, &'static str> {
        if carrier.identity().world_brand != geometry.runtime_identity() {
            return Err("carrier stamp belongs to another physical runtime");
        }
        let (mut fixed, mut moving) = (Vec::new(), Vec::new());
        for (_, handle, pose, shape, is_fixed) in geometry.sensor_colliders() {
            let collider = SensorCollider {
                handle,
                motion_channel: None,
                pose,
                shape,
            };
            if is_fixed {
                fixed.push(collider);
            } else {
                moving.push(collider);
            }
        }
        Ok(Self {
            runtime_identity: geometry.runtime_identity(),
            scene_placement_epoch: Some(geometry.placement_epoch()),
            robot: None,
            placement_revision: None,
            carrier_identity: Some(carrier.identity()),
            robot_revisions: Vec::new(),
            time_ns: carrier.time_ns(),
            robot_pose: pose(carrier.world_from_carrier()),
            fixed,
            moving,
            fixed_queries: OnceLock::new(),
        })
    }
    /// Complete caller-stamped generic carrier identity, absent on the chassis
    /// compatibility path. The capture's actual runtime brand is kept separately.
    pub fn carrier_identity(&self) -> Option<CarrierIdentity> {
        self.carrier_identity
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
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
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
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RollingFrame {
    /// Points in requested pattern-window order, with timestamps even for misses.
    pub points: Vec<TimedReturn>,
}
/// Mounted MID-360 approximation with per-point rigid scene motion and optional
/// seeded range noise/dropout and opt-in angular perturbations. Configuration
/// is immutable; there is no hidden
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
    angular_noise: Option<AngularNoise>,
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
            angular_noise: None,
        })
    }
    /// Enable independent emitted-ray tangent noise, radians per axis in
    /// `[0, pi]`, using this sensor's existing device seed and global emission
    /// index. Zero disables it without changing the original range RNG streams.
    /// Perturbations occur before intersection and therefore affect occlusion.
    /// This generic stress model is not calibrated MID-360 encoder uncertainty.
    ///
    /// ```
    /// use rm_simulator_physics::{Pose, sensors::{
    ///     mid360::Mid360Pattern, rolling::{MountedLidar, RangeEffects},
    /// }};
    /// let sensor = MountedLidar::new(
    ///     Mid360Pattern::new(0.1, 40.)?, Pose::at([0., 0., 0.4]), 0,
    ///     RangeEffects { seed: 7, ..Default::default() },
    /// )?.with_angular_noise(0.001)?;
    /// # let _ = sensor;
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn with_angular_noise(mut self, standard_deviation_rad: f64) -> Result<Self, &'static str> {
        let noise = AngularNoise::new(standard_deviation_rad, self.effects.seed)?;
        self.angular_noise = (standard_deviation_rad > 0.).then_some(noise);
        Ok(self)
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
            let nominal = self.pattern.direction(index);
            let direction = Vector::from(match self.angular_noise {
                Some(noise) => noise
                    .perturb(index, nominal)
                    .expect("unit nominal direction"),
                None => nominal,
            });
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
// Broad phase over collider bounds only; mesh/shape ray tests remain Rapier's
// exact queries. Captures propagate this immutable cache across valid brackets.
struct FixedQueries {
    tree: Bvh,
    bounded: Vec<usize>,
    unbounded: Vec<usize>,
}
impl FixedQueries {
    fn new(colliders: &[SensorCollider]) -> Self {
        let mut unbounded = Vec::new();
        let mut bounded = Vec::new();
        let leaves: Vec<_> = colliders
            .iter()
            .enumerate()
            .filter_map(|(index, c)| {
                // Half-space bounds are effectively infinite and are unsuitable for
                // BVH construction arithmetic. Keep their original exact query.
                if c.shape.as_halfspace().is_some() {
                    unbounded.push(index);
                    None
                } else {
                    bounded.push(index);
                    Some(c.shape.compute_aabb(&c.pose))
                }
            })
            .collect();
        let tree = Bvh::from_leaves(BvhBuildStrategy::Binned, &leaves);
        Self {
            tree,
            bounded,
            unbounded,
        }
    }
    fn cast(&self, colliders: &[SensorCollider], ray: &Ray, max: f64) -> Option<RayHit> {
        let mut best: Option<RayHit> = None;
        let mut test = |index: usize| {
            let collider = &colliders[index];
            let limit = best.map_or(max, |h| h.distance_m);
            if let Some(hit) =
                collider
                    .shape
                    .cast_ray_and_get_normal(&collider.pose, ray, limit, true)
                && best.is_none_or(|old| {
                    hit.time_of_impact < old.distance_m
                        || (hit.time_of_impact == old.distance_m && index < old.shape_index)
                })
            {
                best = Some(RayHit {
                    distance_m: hit.time_of_impact,
                    position_m: ray.point_at(hit.time_of_impact).to_array(),
                    normal: hit.normal.to_array(),
                    shape_index: index,
                });
            }
        };
        // Keep pruning inclusive so equal-distance hits can retain the first
        // capture-local shape even when the BVH visits leaves out of order.
        // Bounds use the configured range, avoiding traversal-order tie loss.
        self.tree.traverse(|node| {
            if node.cast_ray(ray, max) > max {
                return TraversalAction::Prune;
            }
            if let Some(index) = node.leaf_data() {
                test(self.bounded[index as usize]);
            }
            TraversalAction::Continue
        });
        for &index in &self.unbounded {
            test(index);
        }
        best
    }
}

struct SceneBracket<'a> {
    before: &'a SceneCapture,
    fixed_queries: &'a FixedQueries,
    moving: Vec<(&'a SensorCollider, &'a SensorCollider, Aabb)>,
}
impl<'a> SceneBracket<'a> {
    fn new(before: &'a SceneCapture, after: &'a SceneCapture) -> Result<Self, &'static str> {
        if before.runtime_identity != after.runtime_identity
            || before.scene_placement_epoch != after.scene_placement_epoch
            || before.time_ns >= after.time_ns
            || before.carrier_identity != after.carrier_identity
            || before.robot != after.robot
            || before.placement_revision != after.placement_revision
            || before.robot_revisions != after.robot_revisions
        {
            return Err("invalid LiDAR scene times, robot identity or placement continuity");
        }
        let matching = |a: &SensorCollider, b: &SensorCollider| {
            a.handle == b.handle
                && a.motion_channel == b.motion_channel
                && std::ptr::addr_eq(a.shape.as_ref(), b.shape.as_ref())
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
        // Captures normally retain collider order. Only share acceleration when
        // that order (and hence every capture-local shape index) also matches.
        if before
            .fixed
            .iter()
            .zip(&after.fixed)
            .all(|(a, b)| matching(a, b) && a.pose == b.pose)
        {
            let queries = before
                .fixed_queries
                .get_or_init(|| Arc::new(FixedQueries::new(&before.fixed)));
            let _ = after.fixed_queries.set(Arc::clone(queries));
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
        let fixed_queries = before
            .fixed_queries
            .get_or_init(|| Arc::new(FixedQueries::new(&before.fixed)));
        Ok(Self {
            before,
            fixed_queries,
            moving,
        })
    }
    fn cast(&self, ray: &Ray, max: f64, fraction: f64) -> Option<RayHit> {
        let mut best = self.fixed_queries.cast(&self.before.fixed, ray, max);
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

    fn linear_cast(
        before: &SceneCapture,
        after: &SceneCapture,
        ray: &Ray,
        max: f64,
        fraction: f64,
    ) -> Option<RayHit> {
        let fixed = crate::StaticGeometry {
            shapes: before
                .fixed
                .iter()
                .map(|c| (c.pose, c.shape.clone()))
                .collect(),
            ..Default::default()
        };
        let mut best = fixed.cast_metric_ray(ray, max);
        for (i, a) in before.moving.iter().enumerate() {
            let b = after.moving.iter().find(|b| a.handle == b.handle).unwrap();
            let limit = best.map_or(max, |h| h.distance_m);
            if let Some(hit) =
                a.shape
                    .cast_ray_and_get_normal(&a.pose.lerp(&b.pose, fraction), ray, limit, true)
                && best.is_none_or(|old| hit.time_of_impact < old.distance_m)
            {
                best = Some(RayHit {
                    distance_m: hit.time_of_impact,
                    position_m: ray.point_at(hit.time_of_impact).to_array(),
                    normal: hit.normal.to_array(),
                    shape_index: before.fixed.len() + i,
                });
            }
        }
        best
    }

    #[test]
    fn accelerated_brackets_match_linear_queries_and_complete_acquisitions() {
        let (mut world, robot) = rig();
        for i in 0..48 {
            let angle = f64::from(i) * 0.7;
            world.add_static_box(
                Pose::yawed([angle.cos() * 8., angle.sin() * 8., 1.], angle),
                [0.7, 1.2, 2.],
            );
        }
        // Include exact mesh surfaces, not only primitive collider bounds.
        world
            .add_static_mesh(
                vec![[-3., -4., 0.], [-3., 4., 0.], [-3., 0., 4.]],
                vec![[0, 1, 2]],
            )
            .unwrap();
        let before = world.lidar_scene_capture(robot, 0).unwrap();
        for moving in [false, true] {
            let mut after = world.lidar_scene_capture(robot, 100_000_000).unwrap();
            if moving {
                after.robot_pose = pose(Pose::yawed([0.4, -0.1, 0.3], 0.4));
                let delta = after.robot_pose * before.robot_pose.inverse();
                for c in &mut after.moving {
                    c.pose = delta * c.pose;
                }
            }
            let scene = SceneBracket::new(&before, &after).unwrap();
            for mount in [Pose::default(), Pose::at([0., 0., 1.])] {
                let sensor = sensor(
                    mount,
                    RangeEffects {
                        seed: 17,
                        standard_deviation_m: 0.02,
                        dropout_probability: 0.1,
                        ..Default::default()
                    },
                )
                .with_angular_noise(0.001)
                .unwrap();
                for stride in [1, 3, 19, 257] {
                    let window = sensor
                        .window_between(0, after.time_ns, stride)
                        .unwrap()
                        .unwrap();
                    let mut actual = RollingFrame::default();
                    sensor
                        .sample_into(&before, &after, window, &mut actual)
                        .unwrap();
                    for point in actual.points {
                        let fraction = point.time_ns as f64 / after.time_ns as f64;
                        let transform = before.robot_pose.lerp(&after.robot_pose, fraction)
                            * sensor.robot_from_sensor;
                        let nominal = sensor.pattern.direction(point.sample_index);
                        let direction = Vector::from(
                            sensor
                                .angular_noise
                                .unwrap()
                                .perturb(point.sample_index, nominal)
                                .unwrap(),
                        );
                        let ray = Ray::new(transform.translation, transform.rotation * direction);
                        let expected = linear_cast(&before, &after, &ray, 40., fraction);
                        assert_eq!(scene.cast(&ray, 40., fraction), expected);
                        let range = expected
                            .filter(|h| h.distance_m >= 0.1)
                            .and_then(|h| sensor.effects.range(point.sample_index, h.distance_m))
                            .filter(|v| v.is_finite() && *v >= 0.1 && *v <= 40.);
                        assert_eq!(point.range_m, range);
                        assert_eq!(
                            point.point_sensor_m,
                            range.map(|r| (direction * r).to_array())
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn source_fitted_robot_self_occlusion_matches_linear_across_motion() {
        let mut world = WorldPhysics::new(&[], 0.);
        let robot = world
            .add_chassis(
                Team::Red,
                ChassisConfig::team_cad(),
                Pose::at([0., 0., 0.3]),
            )
            .unwrap();
        world.add_static_box(Pose::at([-5., 0., 3.]), [1., 100., 100.]);
        let before = world.lidar_scene_capture(robot, 0).unwrap();
        let mut after = world.lidar_scene_capture(robot, 10_000_000).unwrap();
        after.robot_pose = pose(Pose::yawed([0.2, 0., 0.3], 0.3));
        let delta = after.robot_pose * before.robot_pose.inverse();
        for c in &mut after.moving {
            c.pose = delta * c.pose;
        }
        let scene = SceneBracket::new(&before, &after).unwrap();
        let mut occluded = 0;
        let mut distant = 0;
        for height in [0., 0.15, 1.] {
            let mount = pose(Pose::at([0., 0., height]));
            for index in 0..2000 {
                let fraction = f64::from(index) / 2000.;
                let sensor = before.robot_pose.lerp(&after.robot_pose, fraction) * mount;
                let direction = Vector::from(
                    Mid360Pattern::new(0.1, 40.)
                        .unwrap()
                        .direction(index as u64),
                );
                let ray = Ray::new(sensor.translation, sensor.rotation * direction);
                let hit = scene.cast(&ray, 40., fraction);
                assert_eq!(hit, linear_cast(&before, &after, &ray, 40., fraction));
                if let Some(hit) = hit {
                    occluded += usize::from(hit.shape_index >= before.fixed.len());
                    distant += usize::from(hit.distance_m > 1.);
                }
            }
        }
        assert!(occluded > 100 && distant > 100);
    }

    #[test]
    fn fixed_cache_reuses_only_matching_order_and_preserves_ties_and_range_edges() {
        let (mut world, robot) = rig();
        world.add_static_box(Pose::at([2., 0., 1.]), [1.; 3]);
        world.add_static_box(Pose::at([2., 0., 1.]), [1.; 3]);
        let before = world.lidar_scene_capture(robot, 0).unwrap();
        let mut after = world.lidar_scene_capture(robot, 1000).unwrap();
        let scene = SceneBracket::new(&before, &after).unwrap();
        assert!(Arc::ptr_eq(
            before.fixed_queries.get().unwrap(),
            after.fixed_queries.get().unwrap()
        ));
        for ray in [
            Ray::new(Vector::new(0., 0., 1.), Vector::X),
            Ray::new(Vector::new(2., 0., 1.), Vector::X),
            Ray::new(Vector::new(0., 0., 1.), -Vector::Z),
        ] {
            for limit in [0., 1.49, 1.5, 1.51, 40.] {
                assert_eq!(
                    scene.cast(&ray, limit, 0.5),
                    linear_cast(&before, &after, &ray, limit, 0.5)
                );
            }
        }
        after.fixed.reverse();
        after.fixed_queries = OnceLock::new();
        let reordered = SceneBracket::new(&before, &after).unwrap();
        assert!(after.fixed_queries.get().is_none());
        let ray = Ray::new(Vector::new(0., 0., 1.), Vector::X);
        assert_eq!(
            reordered.cast(&ray, 40., 0.5),
            linear_cast(&before, &after, &ray, 40., 0.5)
        );
        let next = SceneCapture {
            time_ns: 2000,
            fixed_queries: OnceLock::new(),
            ..after.clone()
        };
        let _ = SceneBracket::new(&after, &next).unwrap();
        assert!(!Arc::ptr_eq(
            before.fixed_queries.get().unwrap(),
            after.fixed_queries.get().unwrap()
        ));
        assert!(Arc::ptr_eq(
            after.fixed_queries.get().unwrap(),
            next.fixed_queries.get().unwrap()
        ));
    }

    #[test]
    fn angular_configuration_preserves_disabled_measurements_and_rejects_invalid_sigma() {
        let (world, robot) = rig();
        let (a, b) = (
            world.lidar_scene_capture(robot, 0).unwrap(),
            world.lidar_scene_capture(robot, 1_000_000).unwrap(),
        );
        let mount = Pose::at([0.2, 0., 1.]);
        let effects = RangeEffects {
            seed: 17,
            standard_deviation_m: 0.02,
            dropout_probability: 0.2,
            ..Default::default()
        };
        let mut expected = RollingFrame::default();
        let original = sensor(mount, effects);
        let window = original.window_between(0, 1_000_000, 1).unwrap().unwrap();
        original.sample_into(&a, &b, window, &mut expected).unwrap();
        for sigma in [0., -0.] {
            let mut actual = RollingFrame::default();
            sensor(mount, effects)
                .with_angular_noise(0.01)
                .unwrap()
                .with_angular_noise(sigma)
                .unwrap()
                .sample_into(&a, &b, window, &mut actual)
                .unwrap();
            assert_eq!(expected, actual);
        }
        for sigma in [
            f64::NAN,
            f64::INFINITY,
            -0.001,
            std::f64::consts::PI + 0.001,
        ] {
            assert!(sensor(mount, effects).with_angular_noise(sigma).is_err());
        }
    }

    #[test]
    fn angular_noise_changes_native_edge_occlusion_and_preserves_near_blocking() {
        let nominal = Mid360Pattern::new(0.1, 40.).unwrap().direction(0);
        assert!(nominal[0] < -0.1);
        for front in [0.02, 2.] {
            let (mut world, robot) = rig();
            let edge_y = -front * nominal[1] / nominal[0] + 1e-5;
            // A thin foreground panel starts just beyond the nominal ray's
            // crossing. Angular perturbations can move it onto the panel.
            world.add_static_box(
                Pose::at([-front - 0.5e-6, edge_y + 10., 1.3]),
                [1e-6, 20., 100.],
            );
            let a = world.lidar_scene_capture(robot, 0).unwrap();
            let b = world.lidar_scene_capture(robot, 1_000_000).unwrap();
            let window = PatternWindow {
                first_sample: 0,
                samples: 1,
                stride: 1,
            };
            let mut scan = RollingFrame::default();
            let mount = Pose::at([0., 0., 1.]);
            sensor(mount, RangeEffects::default())
                .sample_into(&a, &b, window, &mut scan)
                .unwrap();
            assert!((scan.points[0].point_sensor_m.unwrap()[0] + 4.5).abs() < 1e-9);
            let (mut foreground, mut background) = (0, 0);
            for seed in 0..32 {
                sensor(
                    mount,
                    RangeEffects {
                        seed,
                        ..Default::default()
                    },
                )
                .with_angular_noise(0.02)
                .unwrap()
                .sample_into(&a, &b, window, &mut scan)
                .unwrap();
                let p = scan.points[0];
                if let Some(local) = p.point_sensor_m {
                    let emitted = AngularNoise::new(0.02, seed)
                        .unwrap()
                        .perturb(0, nominal)
                        .unwrap();
                    assert_eq!(
                        local,
                        (Vector::from(emitted) * p.range_m.unwrap()).to_array()
                    );
                    if (local[0] + 4.5).abs() < 1e-9 {
                        background += 1;
                    } else {
                        assert!(front > 0.1);
                        assert!((local[0] + front).abs() < 2e-6);
                        foreground += 1;
                    }
                } else {
                    assert!(front < 0.1);
                    foreground += 1;
                }
            }
            assert!(foreground > 0 && background > 0);
        }
    }

    #[test]
    fn shared_angular_model_follows_articulated_carrier_at_each_emission() {
        let (world, robot) = rig();
        let start = Pose::yawed([0., 0., 0.5], 0.2);
        let end = Pose::yawed([0.1, 0., 0.5], 0.3);
        let a = world
            .lidar_scene_capture(robot, 0)
            .unwrap()
            .with_mount_frame(start)
            .unwrap();
        let b = world
            .lidar_scene_capture(robot, 1_000_000)
            .unwrap()
            .with_mount_frame(end)
            .unwrap();
        let sensor = sensor(
            Pose::at([0.2, 0., 1.]),
            RangeEffects {
                seed: 17,
                ..Default::default()
            },
        )
        .with_angular_noise(0.001)
        .unwrap();
        let mut scan = RollingFrame::default();
        sensor
            .sample_into(
                &a,
                &b,
                sensor.window_between(0, 1_000_000, 1).unwrap().unwrap(),
                &mut scan,
            )
            .unwrap();
        let mut hits = 0;
        for p in scan.points {
            if let Some(local) = p.point_sensor_m {
                let fraction = p.time_ns as f64 / 1_000_000.;
                let mounted = pose(start).lerp(&pose(end), fraction) * sensor.robot_from_sensor;
                let world = mounted.transform_point(Vector::from(local));
                assert!((world.x + 4.5).abs() < 1e-8);
                let d = AngularNoise::new(0.001, 17)
                    .unwrap()
                    .perturb(p.sample_index, sensor.pattern.direction(p.sample_index))
                    .unwrap();
                assert_eq!(local, (Vector::from(d) * p.range_m.unwrap()).to_array());
                hits += 1;
            }
        }
        assert!(hits > 20);
    }

    #[test]
    fn angular_effects_preserve_chunking_decimation_and_reconstruction() {
        let (world, robot) = rig();
        let mount = Pose::at([0., 0., 1.]);
        let effects = RangeEffects {
            seed: 17,
            standard_deviation_m: 0.02,
            bias_m: 0.003,
            dropout_probability: 0.2,
        };
        let configured = || sensor(mount, effects).with_angular_noise(0.001).unwrap();
        let a = world.lidar_scene_capture(robot, 0).unwrap();
        let b = world.lidar_scene_capture(robot, 503_333).unwrap();
        let c = world.lidar_scene_capture(robot, 1_000_000).unwrap();
        let sensor = configured();
        let mut all = RollingFrame::default();
        sensor
            .sample_into(
                &a,
                &c,
                sensor.window_between(0, 1_000_000, 1).unwrap().unwrap(),
                &mut all,
            )
            .unwrap();
        for stride in [1, 2, 3, 4, 7, 31] {
            let mut chunk = RollingFrame::default();
            sensor
                .sample_into(
                    &a,
                    &b,
                    sensor.window_between(0, 503_333, stride).unwrap().unwrap(),
                    &mut chunk,
                )
                .unwrap();
            let mut joined = chunk.points.clone();
            // Reconstruct from immutable configuration, as a caller does after
            // checkpoint restore; no hidden noise state is needed.
            configured()
                .sample_into(
                    &b,
                    &c,
                    sensor
                        .window_between(503_333, 1_000_000, stride)
                        .unwrap()
                        .unwrap(),
                    &mut chunk,
                )
                .unwrap();
            joined.extend(chunk.points);
            assert_eq!(
                joined,
                all.points
                    .iter()
                    .step_by(stride as usize)
                    .copied()
                    .collect::<Vec<_>>()
            );
        }
        let mut another_device = RollingFrame::default();
        super::MountedLidar::new(
            Mid360Pattern::new(0.1, 40.).unwrap(),
            mount,
            0,
            RangeEffects {
                seed: 18,
                ..effects
            },
        )
        .unwrap()
        .with_angular_noise(0.001)
        .unwrap()
        .sample_into(
            &a,
            &c,
            sensor.window_between(0, 1_000_000, 1).unwrap().unwrap(),
            &mut another_device,
        )
        .unwrap();
        assert_ne!(all, another_device);
    }
    #[test]
    fn scene_capture_pose_and_placement_follow_live_chassis() {
        let (mut world, robot) = rig();
        for target in [Pose::yawed([1., 2., 0.3], 0.7), Pose::at([-1., 0., 0.4])] {
            world.place_chassis(robot, target).unwrap();
            let expected = world.chassis_snapshot(robot).unwrap();
            let scene = world.lidar_scene_capture(robot, 123).unwrap();
            assert_eq!(scene.robot_pose, pose(expected.pose));
            assert_eq!(scene.placement_revision, Some(expected.placement_revision));
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
    fn instance_motion_is_continuous_but_channel_swaps_require_recapture() {
        use crate::motion::{MotionChannel, MotionFrame, MotionId, MotionUpdate};
        let (mut world, robot) = rig();
        let id = MotionId {
            instance: 30,
            channel: 4,
        };
        world
            .add_motion_mesh(
                vec![[2., -1., 0.], [2., 1., 0.], [2., 1., 2.], [2., -1., 2.]],
                vec![[0, 1, 2], [0, 2, 3]],
                id,
                MotionFrame::Translation {
                    offsets_m: [[0.; 3], [2., 0., 0.]],
                },
            )
            .unwrap();
        let before = world.lidar_scene_capture(robot, 0).unwrap();
        world
            .sync_motions(&[MotionUpdate { id, coordinate: 1. }])
            .unwrap();
        let mut after = world.lidar_scene_capture(robot, 1000).unwrap();
        let bracket = SceneBracket::new(&before, &after).unwrap();
        let ray = Ray::new(Vector::new(1., 0., 1.), Vector::X);
        assert!((bracket.cast(&ray, 10., 0.5).unwrap().distance_m - 2.).abs() < 1e-9);
        let moving = after
            .moving
            .iter_mut()
            .find(|c| c.motion_channel == Some(MotionChannel::Instance(id)))
            .unwrap();
        moving.motion_channel = Some(MotionChannel::Instance(MotionId { instance: 31, ..id }));
        assert!(SceneBracket::new(&before, &after).is_err());
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
        *future.placement_revision.as_mut().unwrap() += 1;
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
    #[test]
    fn runtime_brand_rejects_brackets_even_with_identical_shared_geometry() {
        let (world, robot) = rig();
        let (other, _) = rig();
        let before = world.lidar_scene_capture(robot, 0).unwrap();
        let mut after = before.clone();
        after.time_ns = 1_000_000;
        assert!(SceneBracket::new(&before, &after).is_ok());
        after.runtime_identity = other.runtime_identity();
        assert!(SceneBracket::new(&before, &after).is_err());
    }

    #[test]
    fn bound_kinematic_geometry_is_interpolated_by_existing_sensor_queries() {
        use crate::{cooking::CookedRigidBody, runtime::BodyKey};
        use rm_simulator_engine::model::{
            BodyKind, BodyState, ColliderDescription, ContactMaterial, RigidBodyDescription,
            ShapeDescription,
        };
        let (mut world, robot) = rig();
        let model = CookedRigidBody::new(
            RigidBodyDescription::new(
                BodyKind::Kinematic,
                vec![
                    ColliderDescription::new(
                        ShapeDescription::Cuboid {
                            half_extents_m: [0.1; 3],
                        },
                        Pose::at([0.; 3]),
                        ContactMaterial {
                            friction: 0.4,
                            restitution: 0.,
                            memberships: u32::MAX,
                            filter: u32::MAX,
                        },
                    )
                    .unwrap(),
                ],
            )
            .unwrap(),
        )
        .unwrap();
        let binding = world
            .install_body(BodyKey(1), &model, BodyState::at(Pose::at([2., 0., 1.])))
            .unwrap();
        let before = world.lidar_scene_capture(robot, 0).unwrap();
        world
            .set_body_endpoint(binding, Pose::at([3., 0., 1.]))
            .unwrap();
        world.step(0, &crate::TargetFrames::new(vec![])).unwrap();
        let after = world
            .lidar_scene_capture(robot, world.step_duration().as_ns())
            .unwrap();
        let scene = SceneBracket::new(&before, &after).unwrap();
        let ray = Ray::new(Vector::new(0., 0., 1.), Vector::X);
        let hit = scene.cast(&ray, 10., 0.5).unwrap();
        assert!((hit.distance_m - 2.4).abs() < 1e-12);
    }
}
