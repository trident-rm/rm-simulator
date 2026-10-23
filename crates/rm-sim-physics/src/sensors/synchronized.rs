// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-clocked LiDAR/IMU packet assembly. No world, thread or transport is owned.
use super::{
    imu::{CarrierFrame, CarrierImu, CarrierSample, ImuConfig, ImuEffects, ImuSample, MountedImu},
    mid360::Mid360Pattern,
    rolling::{MountedLidar, RangeEffects, RollingFrame, SceneCapture},
};
use crate::Pose;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

/// Fixed mounting and scheduling in one carrier's FLU coordinates.
/// Mounts are explicit; no physical device extrinsic is assumed.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    /// Addressed chassis identity.
    pub robot: u32,
    /// Physical chassis or actual gimbal frame shared by both sensors.
    pub carrier: CarrierFrame,
    /// LiDAR mount in carrier FLU metres and unit wxyz rotation.
    pub lidar_mount: Pose,
    /// IMU mount in carrier FLU metres and unit wxyz rotation.
    pub imu_mount: Pose,
    /// LiDAR packet duration in nanoseconds, 10 ms through 1 s.
    pub lidar_period_ns: u64,
    /// IMU averaging period in nanoseconds, at least 1 ms; divides packet duration.
    pub imu_period_ns: u64,
    /// Retain every nth nominal emission; positive and at most 1001.
    pub stride: u32,
    /// Minimum reported range, metres; nearer surfaces still occlude.
    pub min_range_m: f64,
    /// Maximum reported range, metres.
    pub max_range_m: f64,
    /// LiDAR Gaussian range standard deviation, metres; uncalibrated, zero by default.
    pub range_sigma_m: f64,
    /// LiDAR independent angular standard deviation, radians; zero by default.
    pub angular_sigma_rad: f64,
    /// Independent return suppression probability, zero by default.
    pub dropout: f64,
    /// LiDAR device noise seed.
    pub seed: u64,
    /// IMU errors with explicit SI units; ideal by default.
    pub imu_effects: ImuEffects,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            robot: 0,
            carrier: CarrierFrame::Chassis,
            lidar_mount: Pose::at([0., 0., 0.4]),
            imu_mount: Pose::at([0., 0., 0.4]),
            lidar_period_ns: 100_000_000,
            imu_period_ns: 5_000_000,
            stride: 1,
            min_range_m: 0.1,
            max_range_m: 40.,
            range_sigma_m: 0.,
            angular_sigma_rad: 0.,
            dropout: 0.,
            seed: 7,
            imu_effects: Default::default(),
        }
    }
}
/// Measured output only. Acquisition times share the caller's simulation clock.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Frame {
    /// Zero-based packet ordinal since the attachment epoch; gaps remain visible.
    pub sequence: u64,
    /// Packet interval start, simulation nanoseconds (inclusive for LiDAR).
    pub start_ns: u64,
    /// Packet interval end, simulation nanoseconds (exclusive for LiDAR, inclusive for IMU).
    pub end_ns: u64,
    /// Local distorted points and explicit misses, retaining emission indices/times.
    pub lidar: RollingFrame,
    /// Completed SI inertial measurements in (start_ns, end_ns], without orientation truth.
    pub imu: Vec<ImuSample>,
    /// Cumulative input continuity failures; affected partial packets are discarded.
    pub discontinuities: u64,
}
mod carrier;
pub use carrier::{CarrierConfig, CarrierSynchronized};

struct PacketTiming {
    lidar_period_ns: u64,
    imu_period_ns: u64,
    stride: u32,
}
enum Inertial {
    Legacy(MountedImu),
    Carrier(CarrierImu),
}
impl Inertial {
    fn restart(&mut self) {
        match self {
            Self::Legacy(imu) => imu.restore_state(Default::default()),
            Self::Carrier(imu) => imu.restore_state(Default::default()),
        }
    }
}

/// Stateful packet assembler over coherent brackets and solver substep traces.
///
/// ```
/// use rm_sim_physics::sensors::synchronized::{Config, Synchronized};
/// let stream = Synchronized::new(Config::default(), 1_000_000_000)?;
/// # let _ = stream;
/// # Ok::<(), &'static str>(())
/// ```
pub struct Synchronized {
    config: PacketTiming,
    epoch_ns: u64,
    lidar: MountedLidar,
    imu: Inertial,
    last_end: Option<u64>,
    boundary: u64,
    partial: RollingFrame,
    waiting: VecDeque<Frame>,
    inertial: Vec<ImuSample>,
    discontinuities: u64,
}
impl Synchronized {
    /// Validate SI configuration and one simulation epoch in nanoseconds.
    pub fn new(config: Config, epoch_ns: u64) -> Result<Self, &'static str> {
        if !(10_000_000..=1_000_000_000).contains(&config.lidar_period_ns)
            || config.imu_period_ns < 1_000_000
            || !config.lidar_period_ns.is_multiple_of(config.imu_period_ns)
            || !(1..=1001).contains(&config.stride)
        {
            return Err("invalid synchronized sensor periods or stride");
        }
        let lidar = MountedLidar::new(
            Mid360Pattern::new(config.min_range_m, config.max_range_m)?,
            config.lidar_mount,
            epoch_ns,
            RangeEffects {
                seed: config.seed,
                standard_deviation_m: config.range_sigma_m,
                bias_m: 0.,
                dropout_probability: config.dropout,
            },
        )?
        .with_angular_noise(config.angular_sigma_rad)?;
        let imu = MountedImu::new(ImuConfig {
            robot: config.robot,
            carrier: config.carrier,
            carrier_from_imu: config.imu_mount,
            period_ns: config.imu_period_ns,
            epoch_ns,
            effects: config.imu_effects,
        })?;
        let boundary = epoch_ns
            .checked_add(config.lidar_period_ns)
            .ok_or("sensor clock overflow")?;
        Ok(Self {
            config: PacketTiming {
                lidar_period_ns: config.lidar_period_ns,
                imu_period_ns: config.imu_period_ns,
                stride: config.stride,
            },
            epoch_ns,
            lidar,
            imu: Inertial::Legacy(imu),
            last_end: None,
            boundary,
            partial: Default::default(),
            waiting: Default::default(),
            inertial: Vec::new(),
            discontinuities: 0,
        })
    }
    fn restart(&mut self, time_ns: u64) -> Result<(), &'static str> {
        self.discontinuities += 1;
        self.partial.points.clear();
        self.waiting.clear();
        self.inertial.clear();
        self.imu.restart();
        // Resume only at the next complete packet interval, retaining the epoch/phase.
        let relative = time_ns
            .checked_sub(self.epoch_ns)
            .ok_or("capture precedes epoch")?;
        let start = relative
            .div_ceil(self.config.lidar_period_ns)
            .checked_mul(self.config.lidar_period_ns)
            .and_then(|n| self.epoch_ns.checked_add(n))
            .ok_or("sensor clock overflow")?;
        self.boundary = start
            .checked_add(self.config.lidar_period_ns)
            .ok_or("sensor clock overflow")?;
        Ok(())
    }
    /// Append complete synchronized packets from one physical step.
    /// Geometry/topology errors discard partial output and return an error; the
    /// next bracket may resume. Missing input steps restart both histories.
    /// IMU completion can trail a packet boundary by one solver substep; packets
    /// wait for that measured completion, never a fabricated final IMU sample.
    pub fn ingest(
        &mut self,
        before: &SceneCapture,
        after: &SceneCapture,
        trace: &[CarrierSample],
        out: &mut Vec<Frame>,
    ) -> Result<(), &'static str> {
        self.begin(before, after)?;
        let Inertial::Legacy(imu) = &mut self.imu else {
            return Err("generic synchronized sensors require stamped carrier inputs");
        };
        imu.ingest(trace, &mut self.inertial)?;
        self.acquire(before, after, out)
    }
    fn begin(&mut self, before: &SceneCapture, after: &SceneCapture) -> Result<(), &'static str> {
        let start = before.time_ns();
        let end = after.time_ns();
        if start < self.epoch_ns || end <= start || end - start > 10_000_000 {
            return Err("synchronized capture must bracket one step of at most 10 ms");
        }
        if self.last_end.is_some_and(|last| last != start) {
            self.restart(start)?;
        }
        self.last_end = Some(end);
        Ok(())
    }
    fn acquire(
        &mut self,
        before: &SceneCapture,
        after: &SceneCapture,
        out: &mut Vec<Frame>,
    ) -> Result<(), &'static str> {
        let start = before.time_ns();
        let end = after.time_ns();
        let mut cursor = start.max(self.boundary - self.config.lidar_period_ns);
        while cursor < end {
            let stop = end.min(self.boundary);
            if let Some(window) = self
                .lidar
                .window_between(cursor, stop, self.config.stride)?
            {
                let mut chunk = RollingFrame::default();
                if let Err(error) = self.lidar.sample_into(before, after, window, &mut chunk) {
                    self.restart(end)?;
                    return Err(error);
                }
                self.partial.points.extend(chunk.points);
            }
            if stop == self.boundary {
                self.waiting.push_back(Frame {
                    sequence: (self.boundary - self.epoch_ns) / self.config.lidar_period_ns - 1,
                    start_ns: self.boundary - self.config.lidar_period_ns,
                    end_ns: self.boundary,
                    lidar: std::mem::take(&mut self.partial),
                    imu: Vec::new(),
                    discontinuities: self.discontinuities,
                });
                self.boundary = self
                    .boundary
                    .checked_add(self.config.lidar_period_ns)
                    .ok_or("sensor clock overflow")?;
            }
            cursor = stop;
        }
        while self.waiting.front().is_some_and(|frame| {
            self.inertial
                .last()
                .is_some_and(|sample| sample.time_ns >= frame.end_ns)
        }) {
            let mut frame = self.waiting.pop_front().expect("checked front");
            let split = self
                .inertial
                .partition_point(|sample| sample.time_ns <= frame.end_ns);
            frame.imu = self
                .inertial
                .drain(..split)
                .filter(|sample| sample.time_ns > frame.start_ns)
                .collect();
            let expected = self.config.lidar_period_ns / self.config.imu_period_ns;
            if frame.imu.len() != expected as usize
                || frame.imu.iter().enumerate().any(|(i, sample)| {
                    sample.time_ns != frame.start_ns + (i as u64 + 1) * self.config.imu_period_ns
                })
            {
                // Warm-up and discontinuities must not masquerade as a complete pair.
                self.discontinuities += 1;
                continue;
            }
            frame.discontinuities = self.discontinuities;
            out.push(frame);
        }
        // A missing trace cannot grow queues forever or silently release LiDAR alone.
        if self.waiting.len() > 1
            || self.inertial.len()
                > 2 * (self.config.lidar_period_ns / self.config.imu_period_ns) as usize
        {
            self.restart(end)?;
            return Err("IMU completion unavailable; partial sensor interval discarded");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{TargetFrames, Team, WorldPhysics, chassis::ChassisConfig, tick_ns};
    fn run(stride: u32, skip: Option<u64>) -> Vec<Frame> {
        run_with_topology(stride, skip, false)
    }
    fn run_with_topology(stride: u32, skip: Option<u64>, topology: bool) -> Vec<Frame> {
        let mut world = WorldPhysics::new(&[], 0.);
        let robot = world
            .add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 0., 0.4]))
            .unwrap();
        for tick in 0..256 {
            world
                .step(tick * tick_ns(), &TargetFrames::new(vec![]))
                .unwrap();
        }
        world.set_carrier_trace(true);
        let epoch = 256 * tick_ns();
        let mut sensor = Synchronized::new(
            Config {
                robot,
                stride,
                angular_sigma_rad: 0.001,
                range_sigma_m: 0.01,
                ..Default::default()
            },
            epoch,
        )
        .unwrap();
        let mut before = world.lidar_scene_capture(robot, epoch).unwrap();
        let mut out = Vec::new();
        let mut trace = Vec::new();
        for tick in 256..390 {
            if topology && tick == 290 {
                world
                    .add_chassis(
                        Team::Blue,
                        ChassisConfig::default(),
                        Pose::at([5., 5., 0.4]),
                    )
                    .unwrap();
            }
            world
                .step(tick * tick_ns(), &TargetFrames::new(vec![]))
                .unwrap();
            let after = world
                .lidar_scene_capture(robot, (tick + 1) * tick_ns())
                .unwrap();
            trace.clear();
            world.drain_carrier_trace(&mut trace);
            if skip != Some(tick) {
                let result = sensor.ingest(&before, &after, &trace, &mut out);
                if topology && tick == 290 {
                    assert!(result.is_err());
                } else {
                    result.unwrap();
                }
            }
            before = after;
        }
        out
    }
    #[test]
    fn coherent_packets_preserve_both_clocks_si_units_and_decimation() {
        let full = run(1, None);
        let sparse = run(3, None);
        assert_eq!(full.len(), 9); // Initial 100 ms needs derivative warm-up.
        assert_eq!(full.first().unwrap().sequence, 1);
        for frame in &full {
            assert_eq!(frame.lidar.points.len(), 20_000);
            assert_eq!(frame.imu.len(), 20);
            assert_eq!(frame.imu.last().unwrap().time_ns, frame.end_ns);
            assert_eq!(frame.lidar.points.first().unwrap().time_ns, frame.start_ns);
            assert!(frame.lidar.points.iter().all(|p| p.time_ns < frame.end_ns));
            for imu in &frame.imu {
                assert!(imu.angular_velocity_rad_s.iter().all(|w| w.abs() < 0.01));
                assert!((imu.specific_force_m_s2[2] - 9.81).abs() < 0.1);
            }
        }
        for (a, b) in full.iter().zip(&sparse) {
            assert_eq!(a.imu, b.imu);
            assert_eq!(
                a.lidar
                    .points
                    .iter()
                    .filter(|p| p.sample_index % 3 == 0)
                    .copied()
                    .collect::<Vec<_>>(),
                b.lidar.points
            );
        }
    }
    #[test]
    fn missing_capture_discards_partial_pairs_without_retiming() {
        let uninterrupted = run(1, None);
        let interrupted = run(1, Some(290));
        assert!(interrupted.len() < uninterrupted.len());
        assert!(interrupted.iter().any(|f| f.discontinuities > 1));
        for frame in interrupted {
            let original = uninterrupted
                .iter()
                .find(|f| f.sequence == frame.sequence)
                .unwrap();
            assert_eq!(frame.lidar, original.lidar);
            assert_eq!(frame.imu, original.imu);
        }
    }
    #[test]
    fn topology_changes_drop_the_affected_pair_and_resume_at_original_phase() {
        let frames = run_with_topology(3, None, true);
        assert!(!frames.iter().any(|f| f.sequence == 2));
        assert!(
            frames
                .iter()
                .any(|f| f.sequence > 2 && f.discontinuities > 1)
        );
        for frame in frames {
            assert_eq!(
                frame.start_ns,
                256 * tick_ns() + frame.sequence * 100_000_000
            );
            assert_eq!(frame.imu.len(), 20);
        }
    }
    #[test]
    fn rejects_unbounded_or_misaligned_schedules() {
        for config in [
            Config {
                stride: 0,
                ..Default::default()
            },
            Config {
                lidar_period_ns: 2_000_000_000,
                ..Default::default()
            },
            Config {
                imu_period_ns: 3_000_000,
                ..Default::default()
            },
            Config {
                imu_period_ns: 0,
                ..Default::default()
            },
        ] {
            assert!(Synchronized::new(config, 0).is_err());
        }
        assert!(Synchronized::new(Config::default(), u64::MAX).is_err());
    }
}
