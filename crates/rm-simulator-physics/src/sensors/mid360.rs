// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Procedural two-clock approximation of Livox's published MID-360 angle table.
//!
//! This is an empirical angular fit, not a reverse-engineered optical design or
//! calibrated hardware emulator. See `docs/navigation-sensors.md` for provenance,
//! held-out errors and timing assumptions. Continuous phase replaces CSV wrapping;
//! the inferred rational frequencies give this approximation a 12-second period.
use super::{LidarConfig, LidarFrame, MAX_LIDAR_RAYS, mid360_coefficients::*};
use crate::{
    Pose, StaticGeometry,
    projectile::{pose, pose_is_valid},
};
use rapier3d_f64::prelude::{Ray, Vector};

/// Nominal first-return point interval, nanoseconds, from Livox MID-360 specs
/// (200,000 points/s). Exact hardware firing times are not reconstructed.
pub const POINT_INTERVAL_NS: u64 = 5_000;
/// Period of this fitted rational-clock model, samples, not a verified hardware
/// repeat period. Four interleaved lanes produce 200,000 points/s.
pub const MODEL_PERIOD_SAMPLES: u64 = 2_400_000;

/// A bounded window into the continuous sample sequence. No hidden cursor means
/// chunking, checkpoints and deterministic decimation preserve scan phase.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PatternWindow {
    /// Zero-based global point index; it does not restart at output-frame boundaries.
    pub first_sample: u64,
    /// Number of emitted rays, in `1..=MAX_LIDAR_RAYS`.
    pub samples: u32,
    /// Positive source-index increment. Decimation retains original scan phase;
    /// use odd strides to avoid repeatedly selecting only one interleaved lane.
    pub stride: u32,
}

/// Ideal range sensor using a compact Fourier approximation of the MID-360 table.
/// The nominal lane cadence is a simulation policy, not measured firing timing.
pub struct Mid360Pattern {
    min_range_m: f64,
    max_range_m: f64,
}
impl Mid360Pattern {
    pub(crate) fn ranges_m(&self) -> (f64, f64) {
        (self.min_range_m, self.max_range_m)
    }
    /// Validate ideal range limits; reflectivity-dependent detection is not modeled.
    ///
    /// ```
    /// use rm_simulator_physics::sensors::mid360::Mid360Pattern;
    /// let sensor = Mid360Pattern::new(0.1, 40.)?;
    /// let ray = sensor.direction(800_000); // Continues beyond the four-second table.
    /// assert!((ray.iter().map(|x| x*x).sum::<f64>() - 1.).abs() < 1e-12);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn new(min_range_m: f64, max_range_m: f64) -> Result<Self, &'static str> {
        LidarConfig::planar(1, min_range_m, max_range_m).validate()?;
        Ok(Self {
            min_range_m,
            max_range_m,
        })
    }

    /// Local FLU unit direction of a global point index. Four fitted lanes share
    /// each 20-microsecond phase step. Phase is reduced before floating-point
    /// evaluation to avoid precision loss at large indices.
    pub fn direction(&self, sample_index: u64) -> [f64; 3] {
        let lane = (sample_index % 4) as usize;
        let group = (sample_index % MODEL_PERIOD_SAMPLES) / 4;
        let time_s = group as f64 / 50_000.;
        let spin = std::f64::consts::TAU * (2189. / 12.) * time_s;
        let sweep = std::f64::consts::TAU * 10. * time_s;
        let mut azimuth = COEFFICIENTS_RAD[0][lane] - spin;
        let mut elevation = COEFFICIENTS_RAD[0][lane + 4];
        for (i, &(j, k)) in TERMS.iter().enumerate() {
            let (sin, cos) = (f64::from(j) * spin + f64::from(k) * sweep).sin_cos();
            let c = &COEFFICIENTS_RAD[1 + i];
            let s = &COEFFICIENTS_RAD[1 + TERMS.len() + i];
            azimuth += c[lane] * cos + s[lane] * sin;
            elevation += c[lane + 4] * cos + s[lane + 4] * sin;
        }
        let (sa, ca) = azimuth.sin_cos();
        let (se, ce) = elevation.sin_cos();
        [ce * ca, ce * sa, se]
    }

    /// Nominal emission offset from sequence start, nanoseconds, or `None` on
    /// overflow. This is a uniform simulated cadence; the CSV provides no usable
    /// per-ray timing calibration. Add a caller-owned epoch with checked arithmetic.
    pub fn point_offset_ns(sample_index: u64) -> Option<u64> {
        sample_index.checked_mul(POINT_INTERVAL_NS)
    }

    /// Query a pattern window against one frozen scene and mounted pose, reusing
    /// output storage. `time_ns` stamps the capture, not individual ray times.
    /// This does not synthesize rolling motion; use [`Self::direction`] and
    /// [`Self::point_offset_ns`] with time-aligned poses/captures for that.
    /// All arguments are validated before output changes; index wrap is an error.
    pub fn sample_into(
        &self,
        geometry: &StaticGeometry,
        world_from_sensor: Pose,
        time_ns: u64,
        window: PatternWindow,
        frame: &mut LidarFrame,
    ) -> Result<(), &'static str> {
        if !pose_is_valid(world_from_sensor) {
            return Err("LiDAR pose must be finite with a unit quaternion");
        }
        if window.samples == 0 || window.samples as usize > MAX_LIDAR_RAYS || window.stride == 0 {
            return Err("invalid MID-360 window count or stride");
        }
        let last = window
            .first_sample
            .checked_add(u64::from(window.samples - 1) * u64::from(window.stride))
            .ok_or("MID-360 sample index overflow")?;
        Self::point_offset_ns(last).ok_or("MID-360 point timestamp overflow")?;
        let transform = pose(world_from_sensor);
        frame.time_ns = time_ns;
        frame.world_from_sensor = world_from_sensor;
        frame.returns.resize(window.samples as usize, None);
        for (i, output) in frame.returns.iter_mut().enumerate() {
            let index = window.first_sample + i as u64 * u64::from(window.stride);
            let direction = Vector::from(self.direction(index));
            let ray = Ray::new(transform.translation, transform.rotation * direction);
            *output = geometry
                .cast_metric_ray(&ray, self.max_range_m)
                .filter(|hit| hit.distance_m >= self.min_range_m);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn fit_matches_reference_including_held_out_points() {
        let sensor = Mid360Pattern::new(0.1, 40.).unwrap();
        // Measurement excerpts from pinned upstream table; MIT terms in NOTICE.md.
        for (index, azimuth, zenith) in [
            (0, 268.99_f64, 37.838_f64),
            (1, 269., 40.825),
            (2, 269., 43.581),
            (3, 269., 46.152),
            (400000, 326.13, 40.04),
            (400001, 328.07, 41.516),
            (400002, 329.9, 42.926),
            (400003, 331.64, 44.277),
            (799996, 27.654, 44.261),
            (799997, 29.366, 42.921),
            (799998, 31.165, 41.523),
            (799999, 33.069, 40.061),
        ] {
            let a = azimuth.to_radians();
            let z = zenith.to_radians();
            let actual = Vector::new(z.sin() * a.cos(), z.sin() * a.sin(), z.cos());
            let predicted = Vector::from(sensor.direction(index));
            assert!(
                predicted
                    .cross(actual)
                    .length()
                    .atan2(predicted.dot(actual))
                    .to_degrees()
                    < 0.16
            );
        }
    }
    #[test]
    fn phase_continues_past_csv_and_is_bounded_at_large_indices() {
        let sensor = Mid360Pattern::new(0.1, 40.).unwrap();
        assert_ne!(sensor.direction(800000), sensor.direction(0));
        assert_eq!(sensor.direction(MODEL_PERIOD_SAMPLES), sensor.direction(0));
        for index in [799996, 799997, 799998, 799999, u64::MAX - 4] {
            let a = Vector::from(sensor.direction(index));
            let b = Vector::from(sensor.direction(index + 4));
            assert!((a.length_squared() - 1.).abs() < 1e-12);
            assert!(a.cross(b).length().atan2(a.dot(b)).to_degrees() < 1.6);
        }
        assert_eq!(Mid360Pattern::point_offset_ns(200000), Some(1_000_000_000));
        assert_eq!(Mid360Pattern::point_offset_ns(u64::MAX), None);
    }
    #[test]
    fn windows_preserve_phase_and_fail_transactionally() {
        let sensor = Mid360Pattern::new(0., 40.).unwrap();
        let mut world = crate::WorldPhysics::new(&[], 0.);
        world.add_static_box(Pose::at([0., 5., 1.]), [100., 1., 100.]);
        let geometry = world.static_geometry_snapshot(true);
        let mut whole = LidarFrame::default();
        let mut part = LidarFrame::default();
        let mount = Pose::at([0., 0., 1.]);
        sensor
            .sample_into(
                &geometry,
                mount,
                9,
                PatternWindow {
                    first_sample: 799998,
                    samples: 8,
                    stride: 3,
                },
                &mut whole,
            )
            .unwrap();
        sensor
            .sample_into(
                &geometry,
                mount,
                9,
                PatternWindow {
                    first_sample: 800010,
                    samples: 4,
                    stride: 3,
                },
                &mut part,
            )
            .unwrap();
        assert!(whole.returns.iter().all(Option::is_some));
        assert_eq!(&whole.returns[4..], &part.returns);
        let before = part.clone();
        for w in [
            PatternWindow {
                first_sample: u64::MAX,
                samples: 2,
                stride: 1,
            },
            PatternWindow {
                first_sample: 0,
                samples: 0,
                stride: 1,
            },
            PatternWindow {
                first_sample: 0,
                samples: 1,
                stride: 0,
            },
        ] {
            assert!(
                sensor
                    .sample_into(&geometry, mount, 10, w, &mut part)
                    .is_err()
            );
            assert_eq!(part, before);
        }
    }
}
