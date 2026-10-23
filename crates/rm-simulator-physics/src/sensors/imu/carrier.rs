// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Passive inertial measurements for actual body or prescribed carrier frames.
use super::*;
use crate::sensors::carrier::{CarrierIdentity, StampedCarrierPose, StampedMountPose};
use rm_simulator_engine::model::MountKey;

/// Numeric settings for a generic carrier IMU, independent of robot identities.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarrierImuConfig {
    /// Sensor pose relative to a carrier, FLU metres and unit wxyz rotation.
    /// Used by [`CarrierImu::ingest`]; resolved mount inputs already include it.
    pub carrier_from_imu: Pose,
    /// Positive output averaging period, nanoseconds.
    pub period_ns: u64,
    /// Explicit output phase origin in simulation nanoseconds.
    pub epoch_ns: u64,
    /// Caller-supplied world gravity acceleration, m/s², subtracted from motion.
    /// `None` applies no gravity correction; no runtime or global is consulted.
    pub gravity_m_s2: Option<[f64; 3]>,
    /// Explicit seeded inertial effects; ideal when all effects are zero.
    pub effects: ImuEffects,
}

/// Copied restorable numeric history and complete carrier/mount continuity.
/// Restore only with the same configuration and coherent subsequent inputs.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CarrierImuState {
    numeric: ImuState,
    identity: Option<(CarrierIdentity, Option<MountKey>)>,
}

/// Caller-clocked generic carrier IMU using the legacy sensor's numeric filter.
/// No runtime, scheduling, body lookup or artificial robot identity is owned.
///
/// ```
/// use rm_simulator_engine::model::CarrierKey;
/// use rm_simulator_physics::{Pose, sensors::{carrier::{CarrierIdentity, StampedCarrierPose},
///     imu::{CarrierImu, CarrierImuConfig}}};
/// let mut imu = CarrierImu::new(CarrierImuConfig {
///     carrier_from_imu: Pose::default(), period_ns: 5_000_000, epoch_ns: 0,
///     gravity_m_s2: Some([0., 0., -9.81]), effects: Default::default(),
/// })?;
/// let identity = CarrierIdentity { carrier: CarrierKey(2), world_brand: 1,
///     installation_generation: 4, discontinuity_revision: 0 };
/// let trace = (1..=30).map(|n| StampedCarrierPose::new(
///     identity, n * 1_000_000, 0.001, Pose::default())).collect::<Result<Vec<_>, _>>()?;
/// let mut out = Vec::new();
/// imu.ingest(&trace, &mut out)?;
/// assert!((out[0].specific_force_m_s2[2] - 9.81).abs() < 1e-9);
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Debug)]
pub struct CarrierImu {
    config: CarrierImuConfig,
    carrier_from_imu: Pose3,
    state: CarrierImuState,
}
impl CarrierImu {
    /// Validate the mount, period, effects and optional finite gravity vector.
    pub fn new(config: CarrierImuConfig) -> Result<Self, &'static str> {
        if !pose_is_valid(config.carrier_from_imu) {
            return Err("invalid IMU mounting pose");
        }
        if config.period_ns == 0 {
            return Err("IMU period must be positive");
        }
        if config
            .gravity_m_s2
            .is_some_and(|v| !v.into_iter().all(f64::is_finite))
        {
            return Err("invalid IMU gravity vector");
        }
        config.effects.validate()?;
        Ok(Self {
            carrier_from_imu: pose(config.carrier_from_imu),
            config,
            state: Default::default(),
        })
    }
    /// Validated explicit settings; no runtime defaults are inferred.
    pub fn config(&self) -> &CarrierImuConfig {
        &self.config
    }
    /// Current numeric and full identity history for exact continuation.
    pub fn state(&self) -> &CarrierImuState {
        &self.state
    }
    /// Restore history from an identically configured sensor.
    pub fn restore_state(&mut self, state: CarrierImuState) {
        self.state = state;
    }
    /// Append measurements from ordered actual carrier poses, applying the mount.
    /// All four identity fields, time ordering and the legacy two-interval gap
    /// threshold govern continuity. A change restarts differentiation/windows,
    /// retaining seeded output indices and random-walk history.
    pub fn ingest(
        &mut self,
        samples: &[StampedCarrierPose],
        out: &mut Vec<ImuSample>,
    ) -> Result<(), &'static str> {
        for sample in samples {
            let mounted = pose(sample.world_from_carrier()) * self.carrier_from_imu;
            self.push(*sample, None, mounted, out)?;
        }
        Ok(())
    }
    /// Append already resolved sensor-mount poses without composing the config
    /// mount again. Switching mount keys or input kinds restarts continuity.
    pub fn ingest_mounts(
        &mut self,
        samples: &[StampedMountPose],
        out: &mut Vec<ImuSample>,
    ) -> Result<(), &'static str> {
        for sample in samples {
            self.push(
                sample.carrier(),
                Some(sample.mount()),
                pose(sample.world_from_mount()),
                out,
            )?;
        }
        Ok(())
    }
    fn push(
        &mut self,
        sample: StampedCarrierPose,
        mount: Option<MountKey>,
        mounted: Pose3,
        out: &mut Vec<ImuSample>,
    ) -> Result<(), &'static str> {
        // Composition can overflow even though both input translations are finite.
        if !mounted.translation.is_finite() || !mounted.rotation.is_finite() {
            return Err("invalid IMU resolved mount pose");
        }
        let identity = (sample.identity(), mount);
        let h = sample.interval_s();
        let continuous = self.state.identity == Some(identity)
            && self.state.numeric.last.is_some_and(|(t, _, _)| {
                sample.time_ns() > t && ((sample.time_ns() - t) as f64) < 2e9 * h
            });
        self.state.identity = Some(identity);
        let reaction = -Vector::from(self.config.gravity_m_s2.unwrap_or([0.; 3]));
        NumericFilter {
            period_ns: self.config.period_ns,
            epoch_ns: self.config.epoch_ns,
            effects: self.config.effects,
            state: &mut self.state.numeric,
        }
        .push(
            mounted,
            sample.time_ns(),
            h,
            sample.identity().discontinuity_revision,
            continuous,
            reaction,
            out,
        );
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rm_simulator_engine::model::{CarrierKey, MountDescription};
    fn identity() -> CarrierIdentity {
        CarrierIdentity {
            carrier: CarrierKey(17),
            world_brand: 22,
            installation_generation: 31,
            discontinuity_revision: 0,
        }
    }
    fn config() -> CarrierImuConfig {
        CarrierImuConfig {
            carrier_from_imu: Pose::at([0.2, -0.1, 0.3]),
            period_ns: 5_000_000,
            epoch_ns: 0,
            gravity_m_s2: Some([0., 0., -GRAVITY_M_S2]),
            effects: ImuEffects {
                seed: 9,
                gyro_noise_density_rad_s_sqrt_hz: 0.01,
                accel_noise_density_m_s2_sqrt_hz: 0.02,
                gyro_bias_walk_rad_s2_sqrt_hz: 0.003,
                accel_bias_walk_m_s3_sqrt_hz: 0.004,
                ..Default::default()
            },
        }
    }
    fn trace() -> Vec<StampedCarrierPose> {
        let h = (1. / 128.) / 8.;
        (1..=1024)
            .map(|n| {
                let t = n as f64 * h;
                StampedCarrierPose::new(
                    identity(),
                    n * 7_812_500 / 8,
                    h,
                    Pose::yawed([t, t * t, 0.7], 4. * t),
                )
                .unwrap()
            })
            .collect()
    }
    #[test]
    fn generic_filter_matches_legacy_exactly_including_fractional_intervals_and_effects() {
        let config = config();
        let trace = trace();
        let legacy_trace: Vec<_> = trace
            .iter()
            .map(|s| CarrierSample {
                robot: 3,
                placement_revision: s.identity().discontinuity_revision,
                time_ns: s.time_ns(),
                interval_s: s.interval_s(),
                chassis: s.world_from_carrier(),
                gimbal: Pose::default(),
            })
            .collect();
        let mut legacy = MountedImu::new(ImuConfig {
            robot: 3,
            carrier: CarrierFrame::Chassis,
            carrier_from_imu: config.carrier_from_imu,
            period_ns: config.period_ns,
            epoch_ns: config.epoch_ns,
            effects: config.effects,
        })
        .unwrap();
        let mut generic = CarrierImu::new(config).unwrap();
        let (mut a, mut b) = (Vec::new(), Vec::new());
        legacy.ingest(&legacy_trace, &mut a).unwrap();
        for chunk in trace.chunks(11) {
            generic.ingest(chunk, &mut b).unwrap();
        }
        assert!(!a.is_empty());
        assert_eq!(a, b);
        assert_eq!(legacy.state(), &generic.state().numeric);
        let checkpoint = generic.state().clone();
        let mut restored = CarrierImu::new(config).unwrap();
        restored.restore_state(checkpoint.clone());
        assert_eq!(restored.state(), &checkpoint);
    }
    #[test]
    fn every_identity_field_mount_switch_time_reversal_and_gap_restart_windows() {
        let trace = trace();
        for field in 0..7 {
            let mut sensor = CarrierImu::new(config()).unwrap();
            sensor.ingest(&trace[..100], &mut Vec::new()).unwrap();
            let mut id = identity();
            match field {
                0 => id.carrier = CarrierKey(99),
                1 => id.world_brand += 1,
                2 => id.installation_generation += 1,
                3 => id.discontinuity_revision += 1,
                _ => (),
            }
            let time = match field {
                5 => trace[99].time_ns(),
                6 => trace[99].time_ns() + 10_000_000,
                _ => trace[100].time_ns(),
            };
            let sample = StampedCarrierPose::new(
                id,
                time,
                trace[100].interval_s(),
                trace[100].world_from_carrier(),
            )
            .unwrap();
            let mut out = Vec::new();
            if field == 4 {
                let mount =
                    MountDescription::new(MountKey(7), id.carrier, Pose::default()).unwrap();
                sensor
                    .ingest_mounts(&[sample.resolve_mount(&mount).unwrap()], &mut out)
                    .unwrap();
            } else {
                sensor.ingest(&[sample], &mut out).unwrap();
            }
            assert!(out.is_empty());
            assert_eq!(sensor.state.numeric.velocity, None);
            assert_eq!(sensor.state.numeric.gyro, Window::default());
            assert_eq!(sensor.state.numeric.accel, Window::default());
            assert!(sensor.state.numeric.pending_gyro.is_empty());
        }
    }
    #[test]
    fn resolved_articulated_mounts_use_actual_motion_and_explicit_gravity() {
        let mount = MountDescription::new(MountKey(1), identity().carrier, Pose::at([0.2, 0., 0.]))
            .unwrap();
        let trace: Vec<_> = (1..=200)
            .map(|n| {
                StampedCarrierPose::new(
                    identity(),
                    n * 1_000_000,
                    0.001,
                    Pose::yawed([0., 0., 0.7], n as f64 * 0.01),
                )
                .unwrap()
                .resolve_mount(&mount)
                .unwrap()
            })
            .collect();
        let mut settings = config();
        settings.effects = Default::default();
        settings.gravity_m_s2 = Some([1., 2., -3.]);
        let mut sensor = CarrierImu::new(settings).unwrap();
        let mut out = Vec::new();
        sensor.ingest_mounts(&trace, &mut out).unwrap();
        assert!(out.len() > 30);
        for sample in out {
            assert!((sample.angular_velocity_rad_s[2] - 10.).abs() < 1e-9);
            assert!((sample.specific_force_m_s2[2] - 3.).abs() < 1e-9);
        }
        settings.gravity_m_s2 = None;
        let mut no_gravity = CarrierImu::new(settings).unwrap();
        let mut out = Vec::new();
        no_gravity.ingest_mounts(&trace, &mut out).unwrap();
        assert!(out.iter().all(|s| s.specific_force_m_s2[2] == 0.));
    }
    #[test]
    fn restored_state_continues_identical_outputs_after_gap() {
        let trace = trace();
        let mut sensor = CarrierImu::new(config()).unwrap();
        sensor.ingest(&trace[..333], &mut Vec::new()).unwrap();
        let mut restored = CarrierImu::new(config()).unwrap();
        restored.restore_state(sensor.state().clone());
        let (mut a, mut b) = (Vec::new(), Vec::new());
        sensor.ingest(&trace[400..], &mut a).unwrap();
        restored.ingest(&trace[400..], &mut b).unwrap();
        assert_eq!(a, b);
        assert_eq!(sensor.state(), restored.state());
    }
}
