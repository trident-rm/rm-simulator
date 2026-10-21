// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Passive LiDAR over caller-owned geometry captures. [`Lidar`] is an ideal
//! instantaneous grid scanner; [`mid360`] supplies a procedural approximation,
//! and [`rolling`] adds mounted per-point motion and seeded range effects.
//! No scheduling, localization, host time or transport is hidden in acquisition.
use crate::{
    Pose, StaticGeometry,
    geometry::RayHit,
    projectile::{pose, pose_is_valid},
};
use rapier3d_f64::prelude::{Ray, Vector};

pub mod mid360;
mod mid360_coefficients;
pub mod rolling;

/// Maximum rays per frame, a bounded allocation policy, not a rulebook constant.
pub const MAX_LIDAR_RAYS: usize = 262_144;

/// Scan directions in sensor FLU coordinates, elevation-major then azimuth-major.
#[derive(Clone, Debug, PartialEq)]
pub struct LidarConfig {
    /// Horizontal ray count per elevation, positive.
    pub azimuth_samples: u32,
    /// First horizontal angle, counterclockwise from local +x, radians.
    pub azimuth_start_rad: f64,
    /// Horizontal angle increment, radians; may be negative for clockwise scans.
    pub azimuth_step_rad: f64,
    /// Elevations above the local horizontal, radians in [-π/2, π/2], nonempty.
    pub elevations_rad: Vec<f64>,
    /// Smallest reported range, metres, nonnegative. A closer surface occludes
    /// the ray and yields no return; the query never sees through that surface.
    pub min_range_m: f64,
    /// Largest reported range, metres, finite and greater than the minimum.
    pub max_range_m: f64,
}
impl LidarConfig {
    /// Full 360-degree planar directions without a duplicated endpoint.
    /// Validate the returned configuration through [`Lidar::new`].
    pub fn planar(azimuth_samples: u32, min_range_m: f64, max_range_m: f64) -> Self {
        Self {
            azimuth_samples,
            azimuth_start_rad: 0.,
            azimuth_step_rad: std::f64::consts::TAU / f64::from(azimuth_samples.max(1)),
            elevations_rad: vec![0.],
            min_range_m,
            max_range_m,
        }
    }

    /// Validate ranges, angular arithmetic and the bounded ray count.
    pub fn validate(&self) -> Result<(), &'static str> {
        let rays = self
            .elevations_rad
            .len()
            .checked_mul(self.azimuth_samples as usize);
        if self.azimuth_samples == 0
            || self.elevations_rad.is_empty()
            || rays.is_none_or(|rays| rays > MAX_LIDAR_RAYS)
        {
            return Err("LiDAR ray count must be within the positive allocation budget");
        }
        if !self.min_range_m.is_finite()
            || self.min_range_m < 0.
            || !self.max_range_m.is_finite()
            || self.max_range_m <= self.min_range_m
        {
            return Err("LiDAR ranges must be finite with 0 <= minimum < maximum");
        }
        if !self.azimuth_start_rad.is_finite()
            || !self.azimuth_step_rad.is_finite()
            || !(self.azimuth_start_rad
                + f64::from(self.azimuth_samples - 1) * self.azimuth_step_rad)
                .is_finite()
            || self
                .elevations_rad
                .iter()
                .any(|e| !e.is_finite() || e.abs() > std::f64::consts::FRAC_PI_2)
        {
            return Err("invalid LiDAR azimuth or elevation");
        }
        Ok(())
    }
}

/// One ideal instantaneous acquisition. Misses and below-minimum returns are `None`.
/// This is geometric sensor output; it carries no hidden match or robot state.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LidarFrame {
    /// Caller-supplied simulation acquisition time, nanoseconds; all rays share it.
    pub time_ns: u64,
    /// Actual mounted sensor pose in world FLU metres and wxyz quaternion.
    pub world_from_sensor: Pose,
    /// Pattern-order returns: elevation-major then azimuth-major for [`Lidar`],
    /// or window sample order for [`mid360::Mid360Pattern`].
    /// World points/normals and shape indices refer to the supplied capture only.
    pub returns: Vec<Option<RayHit>>,
}

/// Prepared ideal scan pattern. Construction precomputes local unit directions.
/// Caller owns scheduling and supplies a coherent geometry capture and sensor pose.
///
/// ```
/// use rm_simulator_physics::{Pose, WorldPhysics, sensors::{Lidar, LidarConfig}};
/// let mut world = WorldPhysics::new(&[], 0.0);
/// world.add_static_box(Pose::at([2., 0., 1.]), [1.; 3]);
/// let sensor = Lidar::new(LidarConfig::planar(4, 0.1, 10.))?;
/// let frame = sensor.sample(&world.static_geometry_snapshot(true), Pose::at([0., 0., 1.]), 123)?;
/// assert_eq!(frame.time_ns, 123);
/// assert_eq!(frame.returns[0].unwrap().distance_m, 1.5);
/// assert!(frame.returns[2].is_none());
/// # Ok::<(), &'static str>(())
/// ```
pub struct Lidar {
    config: LidarConfig,
    directions: Vec<Vector>,
}
impl Lidar {
    /// Validate and prepare the immutable scan pattern without querying a world.
    pub fn new(config: LidarConfig) -> Result<Self, &'static str> {
        config.validate()?;
        let directions = config
            .elevations_rad
            .iter()
            .flat_map(|&elevation| {
                let (sin_e, cos_e) = elevation.sin_cos();
                let config = &config;
                (0..config.azimuth_samples).map(move |ray| {
                    let (sin_a, cos_a) = (config.azimuth_start_rad
                        + f64::from(ray) * config.azimuth_step_rad)
                        .sin_cos();
                    Vector::new(cos_e * cos_a, cos_e * sin_a, sin_e)
                })
            })
            .collect();
        Ok(Self { config, directions })
    }

    /// Read the validated configuration. Reconstruct the sensor to change its pattern.
    pub fn config(&self) -> &LidarConfig {
        &self.config
    }

    /// Acquire a new frame using the supplied simulation timestamp and mounted pose.
    pub fn sample(
        &self,
        geometry: &StaticGeometry,
        world_from_sensor: Pose,
        time_ns: u64,
    ) -> Result<LidarFrame, &'static str> {
        let mut frame = LidarFrame::default();
        self.sample_into(geometry, world_from_sensor, time_ns, &mut frame)?;
        Ok(frame)
    }

    /// Acquire into reusable output storage, preserving allocation capacity.
    /// Invalid pose leaves the prior frame intact. The capture must already omit
    /// unwanted/self geometry; durable entity filters and rolling scans are separate.
    pub fn sample_into(
        &self,
        geometry: &StaticGeometry,
        world_from_sensor: Pose,
        time_ns: u64,
        frame: &mut LidarFrame,
    ) -> Result<(), &'static str> {
        if !pose_is_valid(world_from_sensor) {
            return Err("LiDAR pose must be finite with a unit quaternion");
        }
        let transform = pose(world_from_sensor);
        frame.time_ns = time_ns;
        frame.world_from_sensor = world_from_sensor;
        frame.returns.resize(self.directions.len(), None);
        for (direction, output) in self.directions.iter().zip(&mut frame.returns) {
            let ray = Ray::new(transform.translation, transform.rotation * *direction);
            *output = geometry
                .cast_metric_ray(&ray, self.config.max_range_m)
                .filter(|hit| hit.distance_m >= self.config.min_range_m);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::WorldPhysics;

    #[test]
    fn dynamic_robot_captures_are_immutable_and_update_explicitly() {
        let mut world = WorldPhysics::new(&[], 0.);
        let robot = world
            .add_chassis(
                crate::Team::Red,
                crate::chassis::ChassisConfig::default(),
                Pose::at([2., 0., 0.3]),
            )
            .unwrap();
        let before = world.dynamic_geometry_snapshot();
        let sensor = Lidar::new(LidarConfig::planar(1, 0., 10.)).unwrap();
        let mount = Pose::at([0., 0., 0.3]);
        let first = sensor.sample(&before, mount, 0).unwrap();
        world.place_chassis(robot, Pose::at([4., 0., 0.3])).unwrap();
        let after = sensor
            .sample(&world.dynamic_geometry_snapshot(), mount, 1)
            .unwrap();
        assert!(
            (after.returns[0].unwrap().distance_m - first.returns[0].unwrap().distance_m - 2.)
                .abs()
                < 1e-9
        );
        assert_eq!(sensor.sample(&before, mount, 0).unwrap(), first);
    }

    #[test]
    fn mounted_rotation_range_occlusion_and_reused_output() {
        let mut world = WorldPhysics::new(&[], 0.);
        world.add_static_box(Pose::at([1., 2., 1.]), [1.; 3]);
        let geometry = world.static_geometry_snapshot(true);
        let sensor = Lidar::new(LidarConfig::planar(4, 0.1, 10.)).unwrap();
        let mount = Pose::yawed([1., 0., 1.], std::f64::consts::FRAC_PI_2);
        let mut frame = sensor.sample(&geometry, mount, 15).unwrap();
        let hit = frame.returns[0].unwrap();
        assert!((hit.distance_m - 1.5).abs() < 1e-9);
        assert!((hit.position_m[1] - 1.5).abs() < 1e-9);
        assert!(frame.returns[1..].iter().all(Option::is_none));
        let allocation = frame.returns.as_ptr();
        sensor
            .sample_into(&geometry, mount, 20, &mut frame)
            .unwrap();
        assert_eq!(allocation, frame.returns.as_ptr());
        assert_eq!(frame.time_ns, 20);
        let before = frame.clone();
        let mut invalid = mount;
        invalid.rotation_wxyz = [0.; 4];
        assert!(
            sensor
                .sample_into(&geometry, invalid, 30, &mut frame)
                .is_err()
        );
        assert_eq!(frame, before);
        // An object below minimum range occludes a farther one; no see-through.
        let blind = Lidar::new(LidarConfig::planar(4, 1.6, 10.)).unwrap();
        world.add_static_box(Pose::at([1., 4., 1.]), [1.; 3]);
        assert!(
            blind
                .sample(&world.static_geometry_snapshot(true), mount, 20)
                .unwrap()
                .returns[0]
                .is_none()
        );
    }

    #[test]
    fn elevation_order_and_configuration_validation() {
        let mut world = WorldPhysics::new(&[], 0.);
        world.add_static_box(Pose::at([0., 0., 2.]), [1.; 3]);
        let mut config = LidarConfig::planar(1, 0., 10.);
        config.elevations_rad = vec![0., std::f64::consts::FRAC_PI_2];
        let frame = Lidar::new(config.clone())
            .unwrap()
            .sample(&world.static_geometry_snapshot(true), Pose::default(), 0)
            .unwrap();
        assert!(frame.returns[0].is_none());
        assert!((frame.returns[1].unwrap().distance_m - 1.5).abs() < 1e-9);
        let mut invalids = vec![config.clone(); 5];
        invalids[0].max_range_m = f64::NAN;
        invalids[1].min_range_m = -1.;
        invalids[2].azimuth_start_rad = f64::INFINITY;
        invalids[3].azimuth_step_rad = f64::NAN;
        invalids[4].elevations_rad = vec![std::f64::consts::PI];
        for invalid in invalids {
            assert!(Lidar::new(invalid).is_err());
        }
        config.azimuth_samples = u32::MAX;
        assert!(Lidar::new(config.clone()).is_err());
        config.azimuth_samples = 0;
        assert!(Lidar::new(config.clone()).is_err());
        config.azimuth_samples = 1;
        config.elevations_rad = vec![];
        assert!(Lidar::new(config).is_err());
    }
}
