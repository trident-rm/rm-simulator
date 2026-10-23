// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Generic carrier input adapter over the shared packet assembler.
use super::*;
use crate::sensors::{
    carrier::{CarrierIdentity, StampedCarrierPose},
    imu::CarrierImuConfig,
};

/// Explicit settings for two sensors attached to the same generic carrier.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarrierConfig {
    /// LiDAR pose in carrier FLU metres and unit wxyz rotation.
    pub lidar_mount: Pose,
    /// Inertial mount, averaging period, epoch, effects and explicit gravity.
    /// Its epoch also establishes the LiDAR emission and packet phase.
    pub imu: CarrierImuConfig,
    /// LiDAR packet duration in nanoseconds, 10 ms through 1 s.
    pub lidar_period_ns: u64,
    /// Retain every nth emission; 1 through 1001, aligned to the common epoch.
    pub stride: u32,
    /// Minimum reported range in metres; closer geometry still occludes.
    pub min_range_m: f64,
    /// Maximum reported range in metres.
    pub max_range_m: f64,
    /// Explicit range noise, bias, dropout and device seed.
    pub range_effects: RangeEffects,
    /// Independent angular noise standard deviation, radians.
    pub angular_sigma_rad: f64,
}
/// Passive synchronized packet assembly for generic body/prescribed carriers.
/// Geometry and trace stamps must refer to the same carrier installation and
/// placement; complete packet clocks and measurement math match the legacy path.
///
/// ```
/// use rm_simulator_physics::{Pose, sensors::{imu::CarrierImuConfig,
///     synchronized::{CarrierConfig, CarrierSynchronized}}};
/// let sensor = CarrierSynchronized::new(CarrierConfig {
///     lidar_mount: Pose::default(),
///     imu: CarrierImuConfig { carrier_from_imu: Pose::default(), period_ns: 5_000_000,
///         epoch_ns: 0, gravity_m_s2: Some([0.,0.,-9.81]), effects: Default::default() },
///     lidar_period_ns: 100_000_000, stride: 3, min_range_m: 0.1, max_range_m: 40.,
///     range_effects: Default::default(), angular_sigma_rad: 0.,
/// })?;
/// # let _ = sensor;
/// # Ok::<(), &'static str>(())
/// ```
pub struct CarrierSynchronized {
    inner: Synchronized,
    last_identity: Option<CarrierIdentity>,
}
impl CarrierSynchronized {
    /// Validate caller-supplied mounts, effects and clocks without reading time.
    pub fn new(config: CarrierConfig) -> Result<Self, &'static str> {
        if !(10_000_000..=1_000_000_000).contains(&config.lidar_period_ns)
            || config.imu.period_ns < 1_000_000
            || !config.lidar_period_ns.is_multiple_of(config.imu.period_ns)
            || !(1..=1001).contains(&config.stride)
        {
            return Err("invalid synchronized sensor periods or stride");
        }
        let epoch_ns = config.imu.epoch_ns;
        let lidar = MountedLidar::new(
            Mid360Pattern::new(config.min_range_m, config.max_range_m)?,
            config.lidar_mount,
            epoch_ns,
            config.range_effects,
        )?
        .with_angular_noise(config.angular_sigma_rad)?;
        let imu = CarrierImu::new(config.imu)?;
        let boundary = epoch_ns
            .checked_add(config.lidar_period_ns)
            .ok_or("sensor clock overflow")?;
        Ok(Self {
            inner: Synchronized {
                config: PacketTiming {
                    lidar_period_ns: config.lidar_period_ns,
                    imu_period_ns: config.imu.period_ns,
                    stride: config.stride,
                },
                epoch_ns,
                lidar,
                imu: Inertial::Carrier(imu),
                last_end: None,
                boundary,
                partial: Default::default(),
                waiting: Default::default(),
                inertial: Vec::new(),
                discontinuities: 0,
            },
            last_identity: None,
        })
    }
    /// Append measured packets from one explicit bracket and its substep trace.
    /// Identity swaps, resets and missing brackets discard partial packets and
    /// resume on the original phase. Trace stamps outside the bracket or for
    /// another carrier fail before acquisition. Caller stamps are assertions,
    /// not runtime authentication; obtain body stamps from `observe_carrier`.
    pub fn ingest(
        &mut self,
        before: &SceneCapture,
        after: &SceneCapture,
        trace: &[StampedCarrierPose],
        out: &mut Vec<Frame>,
    ) -> Result<(), &'static str> {
        self.inner.begin(before, after)?;
        let Some(identity) = before.carrier_identity() else {
            self.inner.restart(after.time_ns())?;
            return Err("generic synchronized capture requires a stamped carrier");
        };
        if self
            .last_identity
            .is_some_and(|previous| previous != identity)
        {
            self.inner.restart(before.time_ns())?;
        }
        self.last_identity = Some(identity);
        if after.carrier_identity() != Some(identity)
            || trace.iter().any(|s| {
                s.identity() != identity
                    || s.time_ns() <= before.time_ns()
                    || s.time_ns() > after.time_ns()
            })
        {
            self.inner.restart(after.time_ns())?;
            return Err("synchronized carrier trace does not match capture bracket");
        }
        let Inertial::Carrier(imu) = &mut self.inner.imu else {
            unreachable!("generic constructor")
        };
        imu.ingest(trace, &mut self.inner.inertial)?;
        self.inner.acquire(before, after, out)
    }
}
