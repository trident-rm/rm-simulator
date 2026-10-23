// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Mounted inertial measurements from the solver's own substep poses.
//!
//! [`WorldPhysics`](crate::WorldPhysics) optionally records a
//! [`CarrierSample`] for every chassis at the end of every solver substep. A
//! [`MountedImu`] turns that trace into gyroscope and accelerometer samples at a
//! rigidly mounted point of the chassis body or of the actual gimbal frame:
//!
//! - angular rate is the relative rotation between consecutive mounted poses,
//!   divided by the substep (a delta-angle measurement);
//! - specific force is the second difference of the mounted point's world
//!   position, minus the solver's gravity, rotated into the sensor frame. An
//!   off-centre mount therefore sees centripetal and tangential acceleration
//!   with no separate lever-arm model.
//!
//! Both are integrated by time overlap into output periods, so the result does
//! not depend on how ticks are partitioned or on the solver step rate beyond
//! the motion it resolves. Each output is the boxcar mean over the preceding
//! period, stamped at the period's end; it therefore lags the instantaneous
//! motion by half a period. That is a modelling choice, not a measured
//! filter delay.
//!
//! Optional effects are generic white noise, turn-on bias, bias random walk
//! and saturation, indexed by output sample number. They are not calibrated to
//! any device; supply datasheet or Allan-variance values explicitly. Solver
//! contact jitter is deterministic numerical behaviour, not realistic vibration.
use crate::{
    Pose,
    projectile::{GRAVITY_M_S2, pose, pose_is_valid},
};
use rapier3d_f64::prelude::{Pose as Pose3, Vector};
use serde::{Deserialize, Serialize};

mod carrier;
pub use carrier::{CarrierImu, CarrierImuConfig, CarrierImuState};

/// Largest number of carrier samples [`WorldPhysics`](crate::WorldPhysics)
/// retains before the caller drains them, a bounded allocation policy.
/// About eight seconds of 1 kHz substeps for eight chassis.
pub const MAX_CARRIER_TRACE_SAMPLES: usize = 65_536;

/// Physical frame carrying an inertial sensor.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum CarrierFrame {
    /// The chassis body frame, including a spinning chassis' rotation.
    #[default]
    Chassis,
    /// The actual two-axis gimbal (turret) frame, including motor response,
    /// so a pitch-mounted sensor follows the real pitch and yaw motion.
    Gimbal,
}

/// One chassis' physical poses at the end of one solver substep.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CarrierSample {
    /// Chassis identity.
    pub robot: u32,
    /// Placement revision; a change marks a teleport or revival, never motion.
    pub placement_revision: u64,
    /// Simulation time at the end of the substep, nanoseconds, rounded down
    /// when a tick does not divide into whole-nanosecond substeps.
    pub time_ns: u64,
    /// Exact integration interval of this substep, seconds.
    pub interval_s: f64,
    /// Chassis body pose, world FLU metres and unit wxyz rotation.
    pub chassis: Pose,
    /// Actual gimbal pose (`ChassisSnapshot::turret`), world FLU metres.
    pub gimbal: Pose,
}
impl CarrierSample {
    /// The pose of the selected carrier frame.
    pub fn carrier(&self, frame: CarrierFrame) -> Pose {
        match frame {
            CarrierFrame::Chassis => self.chassis,
            CarrierFrame::Gimbal => self.gimbal,
        }
    }
}

/// Generic seeded inertial errors. All default to zero: an ideal sensor.
/// These are explicit stress or datasheet inputs, not a calibrated device.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ImuEffects {
    /// Device seed; use different seeds for different simulated sensors.
    pub seed: u64,
    /// Gyroscope white noise density, rad/s/√Hz, finite and nonnegative.
    pub gyro_noise_density_rad_s_sqrt_hz: f64,
    /// Accelerometer white noise density, m/s²/√Hz, finite and nonnegative.
    pub accel_noise_density_m_s2_sqrt_hz: f64,
    /// Constant gyroscope bias per sensor axis, rad/s.
    pub gyro_bias_rad_s: [f64; 3],
    /// Constant accelerometer bias per sensor axis, m/s².
    pub accel_bias_m_s2: [f64; 3],
    /// Gyroscope bias random walk, rad/s²/√Hz, finite and nonnegative.
    pub gyro_bias_walk_rad_s2_sqrt_hz: f64,
    /// Accelerometer bias random walk, m/s³/√Hz, finite and nonnegative.
    pub accel_bias_walk_m_s3_sqrt_hz: f64,
    /// Symmetric gyroscope range, rad/s; `None` never saturates.
    pub gyro_range_rad_s: Option<f64>,
    /// Symmetric accelerometer range, m/s²; `None` never saturates.
    pub accel_range_m_s2: Option<f64>,
}
impl ImuEffects {
    fn validate(&self) -> Result<(), &'static str> {
        let nonnegative = [
            self.gyro_noise_density_rad_s_sqrt_hz,
            self.accel_noise_density_m_s2_sqrt_hz,
            self.gyro_bias_walk_rad_s2_sqrt_hz,
            self.accel_bias_walk_m_s3_sqrt_hz,
        ];
        let range_ok = |r: Option<f64>| r.is_none_or(|r| r.is_finite() && r > 0.);
        if nonnegative.iter().all(|v| v.is_finite() && *v >= 0.)
            && self
                .gyro_bias_rad_s
                .iter()
                .chain(&self.accel_bias_m_s2)
                .all(|v| v.is_finite())
            && range_ok(self.gyro_range_rad_s)
            && range_ok(self.accel_range_m_s2)
        {
            Ok(())
        } else {
            Err("IMU noise, bias and range must be finite; densities nonnegative")
        }
    }
    fn uniform(&self, index: u64, stream: u64) -> f64 {
        // SplitMix64 finalizer over disjoint counter streams, as in rolling LiDAR.
        let mut x = index.wrapping_mul(0x9e3779b97f4a7c15) ^ self.seed ^ stream;
        x = (x ^ (x >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        x = (x ^ (x >> 27)).wrapping_mul(0x94d049bb133111eb);
        x ^= x >> 31;
        ((x >> 12) as f64 + 0.5) / (1u64 << 52) as f64
    }
    fn normal(&self, index: u64, stream: u64) -> f64 {
        (-2. * self.uniform(index, stream).ln()).sqrt()
            * (std::f64::consts::TAU * self.uniform(index, stream ^ 0x5bd1e995)).cos()
    }
}

/// Validated mounted inertial sensor configuration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ImuConfig {
    /// Chassis carrying the sensor.
    pub robot: u32,
    /// Frame the sensor is rigidly attached to.
    pub carrier: CarrierFrame,
    /// Sensor pose in the carrier frame, FLU metres and unit wxyz rotation.
    /// For an IMU inside a LiDAR, compose the LiDAR mount with the device's
    /// documented LiDAR-to-IMU extrinsic.
    pub carrier_from_imu: Pose,
    /// Output period, nanoseconds; positive. 5,000,000 gives 200 Hz.
    pub period_ns: u64,
    /// Simulation time origin of the output clock, nanoseconds. Sample `n`
    /// (starting at one) averages `(epoch + (n - 1)·period, epoch + n·period]`;
    /// nothing before the epoch is measured.
    pub epoch_ns: u64,
    /// Optional seeded errors; ideal by default.
    pub effects: ImuEffects,
}

/// One inertial measurement in the sensor frame (FLU axes).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ImuSample {
    /// Global output index, independent of draining or chunking.
    pub sample_index: u64,
    /// End of the averaged period, simulation nanoseconds.
    pub time_ns: u64,
    /// Measured angular rate about sensor x/y/z, rad/s.
    pub angular_velocity_rad_s: [f64; 3],
    /// Measured specific force along sensor x/y/z, m/s². A stationary upright
    /// sensor reads about +9.81 on z, the solver's gravity
    /// ([`GRAVITY_M_S2`]), not 9.80665.
    pub specific_force_m_s2: [f64; 3],
}

/// Time-overlap integrator of one piecewise-constant vector signal.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
struct Window {
    /// Index of the open window.
    index: u64,
    /// Integral of the signal over the covered part, unit·s.
    integral: [f64; 3],
    /// Covered duration of the open window, seconds.
    covered_s: f64,
}

/// Restorable sensor state: the last mounted pose, the last velocity
/// estimate, open output windows and the random-walk biases. Store it with a
/// checkpoint to continue the identical measurement sequence after restore.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ImuState {
    /// Last mounted world pose and its time and revision.
    last: Option<(u64, u64, Pose)>,
    /// Last mean world velocity of the mounted point and its interval, s.
    velocity: Option<([f64; 3], f64)>,
    gyro: Window,
    accel: Window,
    /// Completed gyro means awaiting the matching accelerometer window.
    pending_gyro: Vec<(u64, [f64; 3])>,
    /// Random-walk bias increments accumulated through `walk_index`.
    gyro_walk_rad_s: [f64; 3],
    accel_walk_m_s2: [f64; 3],
    walk_index: u64,
}

/// A rigidly mounted gyroscope and accelerometer driven by a carrier trace.
///
/// ```
/// use rm_simulator_physics::{Pose, StepDuration, Team, WorldPhysics, TargetFrames,
///     chassis::ChassisConfig,
///     sensors::imu::{CarrierFrame, ImuConfig, MountedImu}};
/// let duration = StepDuration::HZ_1000;
/// let mut world = WorldPhysics::with_step_duration(&[], 0., duration);
/// let robot = world.add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 0., 0.3]))?;
/// world.set_carrier_trace(true);
/// let mut imu = MountedImu::new(ImuConfig {
///     robot,
///     carrier: CarrierFrame::Chassis,
///     carrier_from_imu: Pose::at([0.1, 0., 0.2]),
///     period_ns: 5_000_000,
///     epoch_ns: 0,
///     effects: Default::default(),
/// })?;
/// let (mut trace, mut samples) = (Vec::new(), Vec::new());
/// for tick in 0..2_000 {
///     world.step(tick * duration.as_ns(), &TargetFrames::new(vec![]))?;
/// }
/// world.drain_carrier_trace(&mut trace);
/// imu.ingest(&trace, &mut samples)?;
/// // Settled on its wheels, the sensor reads gravity's reaction upward.
/// let last = samples.last().unwrap();
/// assert!((last.specific_force_m_s2[2] - 9.81).abs() < 0.05);
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Debug)]
pub struct MountedImu {
    config: ImuConfig,
    carrier_from_imu: Pose3,
    state: ImuState,
}
impl MountedImu {
    /// Validate a mount, period and effects; the sensor starts with no history.
    pub fn new(config: ImuConfig) -> Result<Self, &'static str> {
        if !pose_is_valid(config.carrier_from_imu) {
            return Err("invalid IMU mounting pose");
        }
        if config.period_ns == 0 {
            return Err("IMU period must be positive");
        }
        config.effects.validate()?;
        Ok(Self {
            config,
            carrier_from_imu: pose(config.carrier_from_imu),
            state: ImuState::default(),
        })
    }
    /// The validated configuration.
    pub fn config(&self) -> &ImuConfig {
        &self.config
    }
    /// Current restorable state.
    pub fn state(&self) -> &ImuState {
        &self.state
    }
    /// Continue from a state previously taken from a sensor with the same
    /// configuration, for example after restoring a world checkpoint.
    pub fn restore_state(&mut self, state: ImuState) {
        self.state = state;
    }
    /// Consume carrier samples in time order, appending completed outputs.
    ///
    /// Samples for other robots are ignored. A placement revision change, a
    /// non-increasing time or a gap longer than twice the substep restarts
    /// differentiation and discards the partially covered output periods;
    /// no measurement is fabricated across a teleport, revival, dropped
    /// trace or restore without state. Invalid poses are errors, leaving
    /// `out` unchanged by that sample.
    pub fn ingest(
        &mut self,
        samples: &[CarrierSample],
        out: &mut Vec<ImuSample>,
    ) -> Result<(), &'static str> {
        let robot = self.config.robot;
        for sample in samples.iter().filter(|s| s.robot == robot) {
            self.push(sample, out)?;
        }
        Ok(())
    }
    fn push(
        &mut self,
        sample: &CarrierSample,
        out: &mut Vec<ImuSample>,
    ) -> Result<(), &'static str> {
        let carrier = sample.carrier(self.config.carrier);
        if !pose_is_valid(carrier) || !sample.interval_s.is_finite() || sample.interval_s <= 0. {
            return Err("invalid IMU carrier sample");
        }
        let mounted = pose(carrier) * self.carrier_from_imu;
        let h = sample.interval_s;
        let continuous = self.state.last.is_some_and(|(time, revision, _)| {
            revision == sample.placement_revision
                && sample.time_ns > time
                && ((sample.time_ns - time) as f64) < 2e9 * h
        });
        NumericFilter {
            period_ns: self.config.period_ns,
            epoch_ns: self.config.epoch_ns,
            effects: self.config.effects,
            state: &mut self.state,
        }
        .push(
            mounted,
            sample.time_ns,
            h,
            sample.placement_revision,
            continuous,
            Vector::new(0., 0., GRAVITY_M_S2),
            out,
        );
        Ok(())
    }
}

// The numeric path deliberately has no chassis, carrier or mount identity.
// ImuState's field layout remains the legacy checkpoint contract.
struct NumericFilter<'a> {
    period_ns: u64,
    epoch_ns: u64,
    effects: ImuEffects,
    state: &'a mut ImuState,
}
impl NumericFilter<'_> {
    #[allow(clippy::too_many_arguments)]
    fn push(
        &mut self,
        mounted: Pose3,
        time_ns: u64,
        h: f64,
        revision: u64,
        continuous: bool,
        gravity_reaction: Vector,
        out: &mut Vec<ImuSample>,
    ) {
        let current = Pose {
            translation_m: mounted.translation.to_array(),
            rotation_wxyz: [
                mounted.rotation.w,
                mounted.rotation.x,
                mounted.rotation.y,
                mounted.rotation.z,
            ],
        };
        let previous = self.state.last.replace((time_ns, revision, current));
        let Some((t0, _, before)) = previous.filter(|_| continuous) else {
            self.restart();
            return;
        };
        let before = pose(before);
        // Gyro: body-frame relative rotation over [t0, t1].
        let delta = before.rotation.inverse() * mounted.rotation;
        let delta = if delta.w < 0. { -delta } else { delta };
        let omega = (delta.to_scaled_axis() / h).to_array();
        let t1 = t0 as f64 + h * 1e9;
        self.add_gyro(t0 as f64, t1, omega);
        // Accelerometer: second difference of the mounted point, centred at t0.
        let velocity = ((mounted.translation - before.translation) / h).to_array();
        if let Some((previous_velocity, previous_h)) = self.state.velocity.replace((velocity, h)) {
            let mean_h = (h + previous_h) / 2.;
            let acceleration = Vector::from(velocity) - Vector::from(previous_velocity);
            let world = acceleration / mean_h + gravity_reaction;
            let force = (before.rotation.inverse() * world).to_array();
            let start = t0 as f64 - previous_h * 0.5e9;
            let end = t0 as f64 + h * 0.5e9;
            self.add_accel(start, end, force, out);
        }
    }
    fn restart(&mut self) {
        self.state.velocity = None;
        self.state.pending_gyro.clear();
        self.state.gyro = Window::default();
        self.state.accel = Window::default();
    }
    /// Window bounds relative to the epoch, nanoseconds.
    fn window_bounds(&self, index: u64) -> (f64, f64) {
        let period = self.period_ns as f64;
        let end = index as f64 * period;
        (end - period, end)
    }
    /// Index of the window containing epoch-relative time `t` in `(end - period, end]`.
    fn window_of(&self, t_ns: f64) -> u64 {
        (t_ns / self.period_ns as f64).ceil().max(0.) as u64
    }
    /// Integrate `value` over `[start, end]` into consecutive windows,
    /// returning each completed, fully covered window's mean.
    fn integrate(
        &self,
        window: &mut Window,
        start_ns: f64,
        end_ns: f64,
        value: [f64; 3],
        completed: &mut Vec<(u64, [f64; 3])>,
    ) {
        let period_s = self.period_ns as f64 * 1e-9;
        // Epoch-relative times keep sub-nanosecond precision over long runs.
        let epoch = self.epoch_ns as f64;
        // Carrier timestamps are floored to integer ns, while physical substeps
        // may end on half-nanoseconds. Close a nearly complete window before
        // the next integer timestamp advances its index; otherwise every such
        // boundary silently discards a whole output period.
        let snap = |time: f64| {
            let period = self.period_ns as f64;
            let boundary = (time / period).round() * period;
            if (time - boundary).abs() <= 1. {
                boundary
            } else {
                time
            }
        };
        let (mut start, end_ns) = (snap(start_ns - epoch), snap(end_ns - epoch));
        // Begin at the window containing the piece if the open one is stale.
        let first = self.window_of(start.max(0.) + 1e-3);
        if window.index < first {
            *window = Window {
                index: first,
                ..Default::default()
            };
        }
        while start < end_ns {
            let (open_start, open_end) = self.window_bounds(window.index);
            let piece_end = end_ns.min(open_end);
            let overlap = (piece_end - start.max(open_start)).max(0.) * 1e-9;
            for (sum, v) in window.integral.iter_mut().zip(value) {
                *sum += v * overlap;
            }
            window.covered_s += overlap;
            if piece_end < open_end {
                break;
            }
            if window.index > 0 && window.covered_s >= period_s * (1. - 1e-6) {
                completed.push((window.index, window.integral.map(|v| v / window.covered_s)));
            }
            *window = Window {
                index: window.index + 1,
                ..Default::default()
            };
            start = open_end;
        }
    }
    fn add_gyro(&mut self, start: f64, end: f64, value: [f64; 3]) {
        let mut window = self.state.gyro;
        let mut done = std::mem::take(&mut self.state.pending_gyro);
        self.integrate(&mut window, start, end, value, &mut done);
        self.state.gyro = window;
        // The gyro leads the accelerometer by half a substep; keep its windows
        // until the matching accelerometer window completes.
        self.state.pending_gyro = done;
    }
    fn add_accel(&mut self, start: f64, end: f64, value: [f64; 3], out: &mut Vec<ImuSample>) {
        let mut done = Vec::new();
        let mut window = self.state.accel;
        self.integrate(&mut window, start, end, value, &mut done);
        self.state.accel = window;
        for (index, force) in done {
            self.state.pending_gyro.retain(|(gyro, _)| *gyro >= index);
            if let Some(&(gyro, rate)) = self.state.pending_gyro.first()
                && gyro == index
            {
                self.state.pending_gyro.remove(0);
                out.push(self.measure(index, rate, force));
            }
        }
    }
    fn measure(&mut self, index: u64, rate: [f64; 3], force: [f64; 3]) -> ImuSample {
        let e = self.effects;
        let period_s = self.period_ns as f64 * 1e-9;
        // Advance the random walk to this index with counter-indexed increments,
        // so the sequence depends only on the seed and emitted sample numbers.
        // Skipped periods (after a discontinuity) take one aggregated step of
        // the same variance.
        if self.state.walk_index < index {
            let steps = (index - self.state.walk_index) as f64;
            let stream = if steps == 1. { 0 } else { 0x7137449123ef65cd };
            let scale = (period_s * steps).sqrt();
            for axis in 0..3 {
                let k = axis as u64 + stream;
                self.state.gyro_walk_rad_s[axis] +=
                    e.gyro_bias_walk_rad_s2_sqrt_hz * scale * e.normal(index, 0x3c6ef372 + k);
                self.state.accel_walk_m_s2[axis] +=
                    e.accel_bias_walk_m_s3_sqrt_hz * scale * e.normal(index, 0xa54ff53a + k);
            }
            self.state.walk_index = index;
        }
        let saturate = |v: f64, range: Option<f64>| range.map_or(v, |r| v.clamp(-r, r));
        let mut gyro = [0.; 3];
        let mut accel = [0.; 3];
        for axis in 0..3 {
            let k = axis as u64;
            gyro[axis] = saturate(
                rate[axis]
                    + e.gyro_bias_rad_s[axis]
                    + self.state.gyro_walk_rad_s[axis]
                    + e.gyro_noise_density_rad_s_sqrt_hz / period_s.sqrt()
                        * e.normal(index, 0x510e527f + k),
                e.gyro_range_rad_s,
            );
            accel[axis] = saturate(
                force[axis]
                    + e.accel_bias_m_s2[axis]
                    + self.state.accel_walk_m_s2[axis]
                    + e.accel_noise_density_m_s2_sqrt_hz / period_s.sqrt()
                        * e.normal(index, 0x9b05688c + k),
                e.accel_range_m_s2,
            );
        }
        ImuSample {
            sample_index: index,
            time_ns: self
                .epoch_ns
                .saturating_add(index.saturating_mul(self.period_ns)),
            angular_velocity_rad_s: gyro,
            specific_force_m_s2: accel,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quat_z(angle: f64) -> [f64; 4] {
        let (s, c) = (angle / 2.).sin_cos();
        [c, 0., 0., s]
    }
    /// Samples at exact substep ends of `step_ns / substeps`, like the solver.
    fn trace(
        step_ns: u64,
        substeps: u64,
        ticks: u64,
        motion: impl Fn(f64) -> (Pose, Pose),
    ) -> Vec<CarrierSample> {
        let interval_s = step_ns as f64 * 1e-9 / substeps as f64;
        (0..ticks * substeps)
            .map(|k| {
                let time_ns =
                    (u128::from(step_ns) * u128::from(k + 1) / u128::from(substeps)) as u64;
                let exact_s = (k + 1) as f64 * interval_s;
                let (chassis, gimbal) = motion(exact_s);
                CarrierSample {
                    robot: 3,
                    placement_revision: 0,
                    time_ns,
                    interval_s,
                    chassis,
                    gimbal,
                }
            })
            .collect()
    }
    fn imu(carrier: CarrierFrame, mount: Pose, effects: ImuEffects) -> MountedImu {
        MountedImu::new(ImuConfig {
            robot: 3,
            carrier,
            carrier_from_imu: mount,
            period_ns: 5_000_000,
            epoch_ns: 0,
            effects,
        })
        .unwrap()
    }
    fn run(sensor: &mut MountedImu, samples: &[CarrierSample]) -> Vec<ImuSample> {
        let mut out = Vec::new();
        sensor.ingest(samples, &mut out).unwrap();
        out
    }
    #[test]
    fn fractional_nanosecond_substeps_keep_every_output_period() {
        let trace = trace(7_812_500, 8, 512, |_| {
            (Pose::at([0., 0., 0.3]), Pose::at([0., 0., 0.6]))
        });
        let mut sensor = imu(CarrierFrame::Gimbal, Pose::at([0.; 3]), Default::default());
        sensor.config.epoch_ns = 17 * 7_812_500;
        let out = run(&mut sensor, &trace);
        assert!(out.len() > 700);
        assert!(
            out.windows(2)
                .all(|w| w[1].time_ns - w[0].time_ns == 5_000_000)
        );
    }
    const OMEGA: f64 = 15.;
    /// A chassis spinning at OMEGA about world z through its origin, at 0.3 m,
    /// translating forward at 1 m/s; its gimbal holds world heading and nods
    /// in pitch at 2 Hz about a point 0.1 m ahead of the spin axis.
    fn spinning(t: f64) -> (Pose, Pose) {
        let chassis = Pose {
            translation_m: [t, 0., 0.3],
            rotation_wxyz: quat_z(OMEGA * t),
        };
        let pitch = 0.3 * (std::f64::consts::TAU * 2. * t).sin();
        let (s, c) = (-pitch / 2.).sin_cos();
        let (sy, cy) = (OMEGA * t).sin_cos();
        let gimbal = Pose {
            translation_m: [t + 0.1 * cy, 0.1 * sy, 0.45],
            rotation_wxyz: [c, 0., s, 0.],
        };
        (chassis, gimbal)
    }

    #[test]
    fn stationary_sensor_reads_solver_gravity_and_no_rate() {
        let samples = trace(1_000_000, 1, 100, |_| {
            (Pose::at([1., 2., 0.3]), Pose::default())
        });
        let out = run(
            &mut imu(CarrierFrame::Chassis, Pose::default(), Default::default()),
            &samples,
        );
        // The first period has no prior velocity, so output starts at index 2.
        assert_eq!(out.first().unwrap().sample_index, 2);
        // The centred accelerometer difference needs the next pose, so the
        // period ending at the last sample (100 ms) is still open.
        assert_eq!(out.last().unwrap().time_ns, 95_000_000);
        for s in &out {
            assert_eq!(s.time_ns, s.sample_index * 5_000_000);
            assert!(s.angular_velocity_rad_s.iter().all(|v| v.abs() < 1e-12));
            assert!((s.specific_force_m_s2[2] - GRAVITY_M_S2).abs() < 1e-9);
            assert!(s.specific_force_m_s2[..2].iter().all(|v| v.abs() < 1e-9));
        }
    }

    #[test]
    fn offset_mount_on_spinning_chassis_sees_centripetal_force() {
        let r = 0.2;
        let samples = trace(1_000_000, 1, 400, spinning);
        let out = run(
            &mut imu(
                CarrierFrame::Chassis,
                Pose::at([r, 0., 0.15]),
                Default::default(),
            ),
            &samples,
        );
        assert!(out.len() > 70);
        for s in &out {
            let [gx, gy, gz] = s.angular_velocity_rad_s;
            assert!(gx.abs() < 1e-9 && gy.abs() < 1e-9 && (gz - OMEGA).abs() < 1e-9);
            let [fx, fy, fz] = s.specific_force_m_s2;
            // Toward the spin axis, -ω²r on sensor x. Constant forward velocity
            // adds nothing; the chord estimate is low by (ωh)²/12 ≈ 2e-5.
            assert!((fx + OMEGA * OMEGA * r).abs() < 2e-3, "{fx}");
            assert!(fy.abs() < 1e-6, "{fy}");
            assert!((fz - GRAVITY_M_S2).abs() < 1e-9);
        }
    }

    #[test]
    fn gimbal_pitch_mount_follows_actual_gimbal_motion() {
        let samples = trace(1_000_000, 1, 1000, spinning);
        let out = run(
            &mut imu(CarrierFrame::Gimbal, Pose::default(), Default::default()),
            &samples,
        );
        for s in &out {
            // Mean pitch rate over the preceding 5 ms period.
            let t1 = s.time_ns as f64 * 1e-9;
            let pitch = |t: f64| 0.3 * (std::f64::consts::TAU * 2. * t).sin();
            let mean_rate = -(pitch(t1) - pitch(t1 - 5e-3)) / 5e-3;
            let [gx, gy, gz] = s.angular_velocity_rad_s;
            assert!(gx.abs() < 1e-9 && gz.abs() < 1e-9, "{gx} {gz}");
            assert!((gy - mean_rate).abs() < 1e-6, "{gy} {mean_rate}");
            // The world-stabilised gimbal does not spin; its pivot orbits the
            // spin axis, so the sensor still feels the 0.1 m centripetal term.
            let norm = s
                .specific_force_m_s2
                .iter()
                .map(|v| v * v)
                .sum::<f64>()
                .sqrt();
            let expected = (GRAVITY_M_S2.powi(2) + (OMEGA * OMEGA * 0.1).powi(2)).sqrt();
            assert!((norm - expected).abs() < 0.05, "{norm} {expected}");
        }
    }

    #[test]
    fn chunking_and_solver_rate_do_not_change_measurements() {
        let mount = Pose::at([0.2, -0.05, 0.1]);
        let fine = trace(1_000_000, 1, 1000, spinning);
        let whole = run(
            &mut imu(CarrierFrame::Chassis, mount, Default::default()),
            &fine,
        );
        let mut sensor = imu(CarrierFrame::Chassis, mount, Default::default());
        let mut chunked = Vec::new();
        for part in fine.chunks(37) {
            sensor.ingest(part, &mut chunked).unwrap();
        }
        assert_eq!(whole, chunked);
        // 128 Hz ticks split into eight 976,562.5 ns substeps.
        let coarse = trace(7_812_500, 8, 128, spinning);
        let other = run(
            &mut imu(CarrierFrame::Chassis, mount, Default::default()),
            &coarse,
        );
        let mut matched = 0;
        for a in &whole {
            if let Some(b) = other.iter().find(|b| b.sample_index == a.sample_index) {
                for k in 0..3 {
                    assert!(
                        (a.angular_velocity_rad_s[k] - b.angular_velocity_rad_s[k]).abs() < 1e-6
                    );
                    assert!((a.specific_force_m_s2[k] - b.specific_force_m_s2[k]).abs() < 0.01);
                }
                matched += 1;
            }
        }
        assert!(matched > 190, "{matched}");
    }

    #[test]
    fn discontinuities_restart_without_fabricating_motion() {
        let mut samples = trace(1_000_000, 1, 100, |_| {
            (Pose::at([0., 0., 0.3]), Pose::default())
        });
        for s in &mut samples[50..] {
            s.placement_revision = 1;
            s.chassis.translation_m[0] = 5.;
        }
        let out = run(
            &mut imu(CarrierFrame::Chassis, Pose::default(), Default::default()),
            &samples,
        );
        assert!(out.iter().all(|s| (s.specific_force_m_s2[0]).abs() < 1e-9));
        // The teleport lands at 51 ms. Period 10 ends at 50 ms but its last
        // centred difference needs the rejected 51 ms pose; period 11 is only
        // partly covered after the restart. Neither is reported.
        let indices: Vec<_> = out.iter().map(|s| s.sample_index).collect();
        assert!(indices.contains(&9) && indices.contains(&12));
        assert!(!indices.contains(&10) && !indices.contains(&11));
        // A dropped trace gap behaves the same way.
        let mut gap = trace(1_000_000, 1, 100, |_| {
            (Pose::at([0., 0., 0.3]), Pose::default())
        });
        gap.drain(40..45);
        let out = run(
            &mut imu(CarrierFrame::Chassis, Pose::default(), Default::default()),
            &gap,
        );
        let indices: Vec<_> = out.iter().map(|s| s.sample_index).collect();
        assert!(indices.contains(&7) && indices.contains(&11));
        assert!(![8, 9, 10].iter().any(|i| indices.contains(i)));
        let mut bad = gap.clone();
        bad[3].chassis.rotation_wxyz = [2., 0., 0., 0.];
        let mut sensor = imu(CarrierFrame::Chassis, Pose::default(), Default::default());
        assert!(sensor.ingest(&bad, &mut Vec::new()).is_err());
    }

    #[test]
    fn restored_state_and_seeded_effects_continue_identically() {
        let effects = ImuEffects {
            seed: 11,
            gyro_noise_density_rad_s_sqrt_hz: 2e-4,
            accel_noise_density_m_s2_sqrt_hz: 2e-3,
            gyro_bias_rad_s: [1e-3, 0., 0.],
            gyro_bias_walk_rad_s2_sqrt_hz: 1e-4,
            accel_bias_walk_m_s3_sqrt_hz: 1e-3,
            accel_range_m_s2: Some(40.),
            ..Default::default()
        };
        let samples = trace(1_000_000, 1, 2000, spinning);
        let mount = Pose::at([0.2, 0., 0.]);
        let whole = run(&mut imu(CarrierFrame::Chassis, mount, effects), &samples);
        let mut first = imu(CarrierFrame::Chassis, mount, effects);
        let mut resumed = run(&mut first, &samples[..777]);
        let state = first.state().clone();
        let mut second = imu(CarrierFrame::Chassis, mount, effects);
        second.restore_state(state);
        resumed.extend(run(&mut second, &samples[777..]));
        assert_eq!(whole, resumed);
        assert!(
            whole
                .iter()
                .all(|s| s.specific_force_m_s2.iter().all(|v| v.abs() <= 40.))
        );
        // Noise has roughly the configured discrete standard deviation.
        let ideal = run(
            &mut imu(CarrierFrame::Chassis, mount, Default::default()),
            &samples,
        );
        let residual: Vec<f64> = whole
            .iter()
            .zip(&ideal)
            .map(|(a, b)| a.specific_force_m_s2[1] - b.specific_force_m_s2[1])
            .collect();
        let mean = residual.iter().sum::<f64>() / residual.len() as f64;
        let sd = (residual.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / residual.len() as f64)
            .sqrt();
        let expected = 2e-3 / 5e-3f64.sqrt();
        assert!((sd / expected - 1.).abs() < 0.25, "{sd} {expected}");
        assert!(
            MountedImu::new(ImuConfig {
                effects: ImuEffects {
                    gyro_noise_density_rad_s_sqrt_hz: -1.,
                    ..Default::default()
                },
                ..*first.config()
            })
            .is_err()
        );
    }

    #[test]
    fn solver_spinning_chassis_matches_its_own_body_rates() {
        use crate::{
            StepDuration, TargetFrames, WorldPhysics,
            chassis::{ChassisCommand, ChassisConfig},
        };
        let duration = StepDuration::HZ_128;
        let mut world = WorldPhysics::with_step_duration(&[], 0., duration);
        let config = ChassisConfig::default();
        let robot = world
            .add_chassis(
                crate::Team::Red,
                config.clone(),
                Pose::at([0., 0., config.rest_height_m() + 0.02]),
            )
            .unwrap();
        let frames = TargetFrames::new(vec![]);
        for tick in 0..256 {
            world.step(tick * duration.as_ns(), &frames).unwrap();
        }
        world
            .command_chassis(
                robot,
                ChassisCommand {
                    yaw_rate_rad_s: 6.,
                    ..Default::default()
                },
            )
            .unwrap();
        world.set_carrier_trace(true);
        let r = 0.2;
        let mut sensor = MountedImu::new(ImuConfig {
            robot,
            carrier: CarrierFrame::Chassis,
            carrier_from_imu: Pose::at([r, 0., 0.2]),
            period_ns: 5_000_000,
            epoch_ns: 256 * duration.as_ns(),
            effects: Default::default(),
        })
        .unwrap();
        let (mut trace, mut out) = (Vec::new(), Vec::new());
        for tick in 256..1024 {
            world.step(tick * duration.as_ns(), &frames).unwrap();
            world.drain_carrier_trace(&mut trace);
            sensor.ingest(&trace, &mut out).unwrap();
            trace.clear();
        }
        assert_eq!(world.carrier_trace_dropped(), 0);
        let snapshot = world.chassis_snapshot(robot).unwrap();
        let omega = snapshot.angular_velocity_rad_s[2];
        assert!(omega > 3., "chassis did not spin up: {omega}");
        // Average the final second to suppress deterministic contact jitter.
        let tail = &out[out.len() - 200..];
        let mean = |f: &dyn Fn(&ImuSample) -> f64| tail.iter().map(f).sum::<f64>() / 200.;
        let gz = mean(&|s| s.angular_velocity_rad_s[2]);
        let fx = mean(&|s| s.specific_force_m_s2[0]);
        let fz = mean(&|s| s.specific_force_m_s2[2]);
        assert!((gz - omega).abs() < 0.05 * omega, "{gz} {omega}");
        assert!((fx + gz * gz * r).abs() < 0.5, "{fx} {}", -gz * gz * r);
        assert!((fz - GRAVITY_M_S2).abs() < 0.3, "{fz}");
    }
}
