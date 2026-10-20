// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Restorable automatic assembly motion on the normal 128 Hz field clock.
use crate::{Pose, Team};
use glam::DQuat;
use rm_simulator_physics::manipulator::SerialArm;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
/// Source-calibrated assembly geometry; units are explicit on each field.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Jaw {
    /// Source motion group name.
    pub name: String,
    /// Bearing origin in the reference body frame, metres.
    pub origin_m: [f64; 3],
    /// Unit bearing axis in the reference body frame.
    pub axis: [f64; 3],
    /// Open jaw angle in radians.
    pub open_rad: f64,
    /// Fitted grasp-stop angle in radians.
    pub closed_rad: f64,
}
/// Source-calibrated assembly geometry; units are explicit on each field.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Auxiliary {
    /// Source motion group name.
    pub name: String,
    /// Zero-pose source group frame, metres and wxyz.
    pub reference: Pose,
    /// Source-fitted stowed transform relative to the forearm.
    pub stowed_delta: Pose,
}
/// Source-calibrated assembly geometry; units are explicit on each field.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Model {
    /// Source calibration schema; five for the approved PIE binding.
    pub schema: u32,
    /// Rulebook OXYZ frame in rig coordinates, metres and wxyz.
    pub assembly_frame: Pose,
    /// Source pedestal reference points in rig metres.
    pub core_envelope_m: Vec<[f64; 3]>,
    /// Source-fitted six-axis Engineer geometry.
    pub engineer: SerialArm,
    /// Source-fitted six-axis Core positioning geometry.
    pub core: SerialArm,
    /// The two independently moving source jaws.
    pub engineer_jaws: Vec<Jaw>,
    /// Source camera and UWB attachment transforms.
    pub engineer_auxiliary: Vec<Auxiliary>,
    /// P bearing origin in the source head reference frame, metres.
    pub core_p_origin_m: [f64; 3],
    /// Unit P bearing axis.
    pub core_p_axis: [f64; 3],
    /// Q bearing origin in the source head reference frame, metres.
    pub core_q_origin_m: [f64; 3],
    /// Unit Q bearing axis.
    pub core_q_axis: [f64; 3],
    /// Unit carriage translation axis.
    pub core_lift_axis: [f64; 3],
}

impl Model {
    /// Validate bounded, finite source calibration before using its kinematics.
    pub fn validate(&self) -> Result<(), &'static str> {
        self.engineer.validate()?;
        self.core.validate()?;
        let valid = |p: Pose| {
            p.translation_m
                .into_iter()
                .chain(p.rotation_wxyz)
                .all(f64::is_finite)
                && (p.rotation_wxyz.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-6
        };
        let unit_axis = |axis: [f64; 3]| {
            axis.into_iter().all(f64::is_finite)
                && (axis.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-6
        };
        if self.engineer_jaws.len() != 2
            || self.engineer_auxiliary.len() > 8
            || self.core_envelope_m.len() > 8192
            || self.schema != 5
            || !valid(self.assembly_frame)
            || !unit_axis(self.core_p_axis)
            || !unit_axis(self.core_q_axis)
            || !unit_axis(self.core_lift_axis)
            || !self
                .core_p_origin_m
                .into_iter()
                .chain(self.core_q_origin_m)
                .chain(self.core_envelope_m.iter().flatten().copied())
                .all(f64::is_finite)
            || self.engineer_jaws.iter().any(|j| {
                j.name.len() > 64
                    || !unit_axis(j.axis)
                    || !j
                        .origin_m
                        .into_iter()
                        .chain([j.open_rad, j.closed_rad])
                        .all(f64::is_finite)
            })
            || self
                .engineer_auxiliary
                .iter()
                .any(|a| a.name.len() > 64 || !valid(a.reference) || !valid(a.stowed_delta))
        {
            return Err("invalid source assembly model");
        }
        Ok(())
    }
}

/// One source rig configuration, independent of rendering and planner internals.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigPose {
    /// Engineer body in source rig coordinates, metres and wxyz.
    pub engineer_body: Pose,
    /// Six Engineer joint angles, radians.
    pub engineer_q: [f64; 6],
    /// Six Core positioning angles, radians.
    pub core_q: [f64; 6],
    /// Unit pose in source rig coordinates, metres and wxyz.
    pub unit_pose: Pose,
    /// Two source jaw angles, radians.
    pub jaw_angles: [f64; 2],
    /// Whether the camera retains its source deployed posture.
    pub camera_deployed: bool,
    /// Carriage lift in metres.
    pub lift_m: f64,
    /// P rotation in radians.
    pub p_rad: f64,
    /// Q rotation in radians.
    pub q_rad: f64,
    /// Whether the Unit is held by the jaws.
    pub gripped: bool,
}
fn quaternion(p: Pose) -> DQuat {
    let [w, x, y, z] = p.rotation_wxyz;
    DQuat::from_xyzw(x, y, z, w)
}
fn position(p: Pose) -> glam::DVec3 {
    glam::DVec3::from_array(p.translation_m)
}
fn pose(p: glam::DVec3, q: DQuat) -> Pose {
    Pose {
        translation_m: p.to_array(),
        rotation_wxyz: [q.w, q.x, q.y, q.z],
    }
}
fn compose(a: Pose, b: Pose) -> Pose {
    let q = quaternion(a);
    pose(position(a) + q * position(b), q * quaternion(b))
}
fn inverse(a: Pose) -> Pose {
    let q = quaternion(a).inverse();
    pose(-(q * position(a)), q)
}
fn delta(origin: [f64; 3], axis: [f64; 3], angle: f64) -> Pose {
    let origin = glam::DVec3::from_array(origin);
    let q = DQuat::from_axis_angle(glam::DVec3::from_array(axis), angle);
    pose(origin - q * origin, q)
}
fn translation(v: [f64; 3]) -> Pose {
    Pose::at(v)
}
impl RigPose {
    /// Source link frames; Engineer frames are body-local, Core frames rig-local.
    pub fn frames(&self, config: &Model) -> BTreeMap<String, Pose> {
        let mut result = BTreeMap::new();
        result.insert("engineer/body".into(), Pose::default());
        result.insert("core/body".into(), Pose::default());
        for (prefix, arm, q) in [
            ("engineer", &config.engineer, self.engineer_q),
            ("core", &config.core, self.core_q),
        ] {
            for (i, p) in arm.frames(q).into_iter().enumerate() {
                result.insert(format!("{prefix}/j{}", i + 1), p);
            }
        }
        let d = compose(
            config.engineer.frames(self.engineer_q)[5],
            inverse(translation(config.engineer.axes[5].origin_m)),
        );
        for (jaw, angle) in config.engineer_jaws.iter().zip(&self.jaw_angles) {
            result.insert(
                format!("engineer/{}", jaw.name),
                compose(
                    d,
                    compose(
                        delta(jaw.origin_m, jaw.axis, *angle),
                        translation(jaw.origin_m),
                    ),
                ),
            );
        }
        let forearm = compose(
            config.engineer.frames(self.engineer_q)[2],
            inverse(translation(config.engineer.axes[2].origin_m)),
        );
        for aux in &config.engineer_auxiliary {
            result.insert(
                format!("engineer/{}", aux.name),
                compose(
                    forearm,
                    compose(
                        if aux.name == "camera-lift" && self.camera_deployed {
                            Pose::default()
                        } else {
                            aux.stowed_delta
                        },
                        aux.reference,
                    ),
                ),
            );
        }
        let l = translation(config.core_lift_axis.map(|v| v * self.lift_m));
        result.insert(
            "core/lift".into(),
            compose(
                self.core_delta(config),
                compose(l, translation(config.core_q_origin_m)),
            ),
        );
        result.insert(
            "core/p".into(),
            compose(
                self.head_delta(config, self.lift_m, self.p_rad, 0.),
                translation(config.core_p_origin_m),
            ),
        );
        result.insert(
            "core/q".into(),
            compose(
                self.head_delta(config, self.lift_m, self.p_rad, self.q_rad),
                translation(config.core_q_origin_m),
            ),
        );
        result
    }
    fn core_delta(&self, config: &Model) -> Pose {
        compose(
            config.core.frames(self.core_q)[5],
            inverse(translation(config.core.axes[5].origin_m)),
        )
    }
    fn head_delta(&self, config: &Model, lift: f64, p: f64, q: f64) -> Pose {
        compose(
            self.core_delta(config),
            compose(
                delta(config.core_p_origin_m, config.core_p_axis, p),
                compose(
                    translation(config.core_lift_axis.map(|v| v * lift)),
                    delta(config.core_q_origin_m, config.core_q_axis, q),
                ),
            ),
        )
    }
}

/// Simulator joint-rate limits, radians/second; not donor motor specifications.
pub const JOINT_SPEED_RAD_S: [f64; 6] = [1.; 6];
/// Simulator jaw-rate limit, radians/second; not a measured motor specification.
pub const JAW_SPEED_RAD_S: f64 = 1.;
/// Raised, compact neutral posture for the PIE donor, radians. Simulator preset;
/// the shoulder/forearm oppose so the empty wrist stays approximately level.
pub const NEUTRAL_JOINTS_RAD: [f64; 6] = [0., 0.35, -0.35, 0., 0., 0.];
/// Persistent body-local arm authority, independent of any assembly attempt.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArmState {
    /// Owning physical chassis identity.
    pub owner: u32,
    /// Verified source calibration.
    pub model: Model,
    /// Actual six joint angles, radians; never reset by solving or completion.
    pub joints_rad: [f64; 6],
    /// Explicit posture target, radians; holds actual joints after a trajectory.
    pub target_rad: [f64; 6],
    /// Actual two jaw angles, radians.
    pub jaws_rad: [f64; 2],
    /// Jaw target, radians.
    pub jaw_target_rad: [f64; 2],
    /// Last requested tunnel posture; only a changed request selects a new target.
    pub folded: bool,
}
impl ArmState {
    /// Start in the compact raised neutral pose with open jaws.
    pub fn neutral(owner: u32, model: Model) -> Self {
        let jaws = std::array::from_fn(|i| model.engineer_jaws.get(i).map_or(0., |j| j.open_rad));
        Self {
            owner,
            model,
            joints_rad: NEUTRAL_JOINTS_RAD,
            target_rad: NEUTRAL_JOINTS_RAD,
            jaws_rad: jaws,
            jaw_target_rad: jaws,
            folded: false,
        }
    }
    /// Validate finite bounded source geometry and joint/posture values.
    pub fn validate(&self) -> Result<(), &'static str> {
        self.model.validate()?;
        if self.model.engineer_jaws.len() != 2
            || self.model.engineer_auxiliary.len() > 8
            || self
                .joints_rad
                .into_iter()
                .zip(self.model.engineer.axes)
                .chain(self.target_rad.into_iter().zip(self.model.engineer.axes))
                .any(|(q, a)| {
                    !q.is_finite() || q < a.range_rad[0] - 1e-6 || q > a.range_rad[1] + 1e-6
                })
            || !self
                .jaws_rad
                .into_iter()
                .chain(self.jaw_target_rad)
                .all(f64::is_finite)
        {
            return Err("invalid persistent arm state");
        }
        Ok(())
    }
    /// Body-local source link frames, including continuously posed jaws.
    pub fn frames(&self) -> BTreeMap<String, Pose> {
        RigPose {
            engineer_body: Pose::default(),
            engineer_q: self.joints_rad,
            core_q: [0.; 6],
            unit_pose: Pose::default(),
            jaw_angles: self.jaws_rad,
            camera_deployed: false,
            lift_m: 0.,
            p_rad: 0.,
            q_rad: 0.,
            gripped: false,
        }
        .frames(&self.model)
    }
    fn step(&mut self) {
        let dt = crate::tick_ns() as f64 * 1e-9;
        for (i, speed) in JOINT_SPEED_RAD_S.iter().enumerate() {
            self.joints_rad[i] +=
                (self.target_rad[i] - self.joints_rad[i]).clamp(-speed * dt, speed * dt);
        }
        for i in 0..2 {
            self.jaws_rad[i] += (self.jaw_target_rad[i] - self.jaws_rad[i])
                .clamp(-JAW_SPEED_RAD_S * dt, JAW_SPEED_RAD_S * dt);
        }
    }
}
/// Automatic assembly lifecycle, shared by all clients.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Phase {
    /// The Core positioning arm is moving to its selected pose before planning.
    Positioning,
    /// A host worker is searching; the world clock is unaffected.
    Solving,
    /// A validated trajectory is being applied one frame per tick.
    Actuating,
    /// The manipulation evaluator accepted confirmation.
    Completed,
    /// Planning, parking, timeout or evaluation failed.
    Failed,
    /// Pilot cancelled or left the station.
    Cancelled,
}
/// Compact validated tick sample; fixed configuration stays in the run header.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Frame {
    /// Engineer joint angles, radians.
    pub joints_rad: [f64; 6],
    /// Jaw angles, radians.
    pub jaws_rad: [f64; 2],
    /// Unit pose in rig coordinates, metres and wxyz.
    pub unit: Pose,
    /// Logical insertion guide coordinates, metres.
    pub guide_m: [f64; 3],
    /// P, Q and off-axis guide angles, radians.
    pub angles_rad: [f64; 3],
    /// Actual kinematic grasp attachment.
    pub gripped: bool,
    /// Planner stage identifier: approach, grasp, insert, lift, flip, align, settle.
    pub stage: u8,
}
/// Restorable source Core travel before the robot planner is released.
/// Speeds are simulator defaults: 1 rad/s per rotary joint, 0.1 m/s carriage.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CorePositioning {
    /// Six source positioning joints at motion admission, radians.
    pub from_rad: [f64; 6],
    /// Six fixed target joints; planning must preserve this branch, radians.
    pub target_rad: [f64; 6],
    /// Initial carriage extension in metres, reset to zero before insertion.
    pub lift_m: f64,
    /// Initial P/Q rotations in radians, reset to zero before insertion.
    pub angles_rad: [f64; 2],
    /// Elapsed explicit simulation time, nanoseconds.
    pub elapsed_ns: u64,
    /// Quantized duration, nanoseconds; at least one normal 128 Hz tick.
    pub duration_ns: u64,
}
impl CorePositioning {
    /// Prepare bounded simultaneous travel from the current Core mechanism pose.
    pub fn new(pose: &RigPose, target_rad: [f64; 6]) -> Self {
        let seconds = pose
            .core_q
            .into_iter()
            .zip(target_rad)
            .map(|(a, b)| (a - b).abs())
            .chain([pose.p_rad.abs(), pose.q_rad.abs(), pose.lift_m.abs() / 0.1])
            .fold(0., f64::max);
        let ticks = (seconds / (crate::tick_ns() as f64 * 1e-9)).ceil().max(1.) as u64;
        Self {
            from_rad: pose.core_q,
            target_rad,
            lift_m: pose.lift_m,
            angles_rad: [pose.p_rad, pose.q_rad],
            elapsed_ns: 0,
            duration_ns: ticks.saturating_mul(crate::tick_ns()),
        }
    }
    /// Fraction of the selected motion completed, from zero through one.
    ///
    /// ```
    /// use rm_simulator_world::assembly::CorePositioning;
    /// let motion = CorePositioning {
    ///     from_rad: [0.; 6], target_rad: [0.5; 6], lift_m: 0.,
    ///     angles_rad: [0.; 2], elapsed_ns: 250_000_000,
    ///     duration_ns: 500_000_000,
    /// };
    /// assert_eq!(motion.progress(), 0.5);
    /// ```
    pub fn progress(&self) -> f64 {
        self.elapsed_ns as f64 / self.duration_ns.max(1) as f64
    }
    fn advance(&mut self, pose: &mut RigPose) -> bool {
        self.elapsed_ns = self
            .elapsed_ns
            .saturating_add(crate::tick_ns())
            .min(self.duration_ns);
        let t = self.progress();
        pose.core_q =
            std::array::from_fn(|i| self.from_rad[i] + (self.target_rad[i] - self.from_rad[i]) * t);
        pose.lift_m = self.lift_m * (1. - t);
        pose.p_rad = self.angles_rad[0] * (1. - t);
        pose.q_rad = self.angles_rad[1] * (1. - t);
        if self.elapsed_ns == self.duration_ns {
            pose.core_q = self.target_rad;
            pose.lift_m = 0.;
            pose.p_rad = 0.;
            pose.q_rad = 0.;
            true
        } else {
            false
        }
    }
    fn validate(&self, model: &Model) -> Result<(), &'static str> {
        if self.duration_ns == 0
            || self.duration_ns > 60_000_000_000
            || self.elapsed_ns > self.duration_ns
            || !self.duration_ns.is_multiple_of(crate::tick_ns())
            || !self.elapsed_ns.is_multiple_of(crate::tick_ns())
            || ![self.lift_m, self.angles_rad[0], self.angles_rad[1]]
                .into_iter()
                .all(f64::is_finite)
            || self
                .from_rad
                .iter()
                .chain(&self.target_rad)
                .any(|q| !q.is_finite())
            || self
                .from_rad
                .iter()
                .zip(&self.target_rad)
                .zip(&model.core.axes)
                .any(|((a, b), axis)| {
                    *a < axis.range_rad[0]
                        || *a > axis.range_rad[1]
                        || *b < axis.range_rad[0]
                        || *b > axis.range_rad[1]
                })
        {
            return Err("invalid Core positioning motion");
        }
        let seconds = self.duration_ns as f64 * 1e-9;
        if self
            .from_rad
            .iter()
            .zip(self.target_rad)
            .any(|(a, b)| (a - b).abs() > seconds + 1e-9)
            || self.angles_rad.iter().any(|a| a.abs() > seconds + 1e-9)
            || self.lift_m.abs() > seconds * 0.1 + 1e-9
        {
            return Err("Core positioning exceeds speed limits");
        }
        Ok(())
    }
}

/// Complete assembly authority, including the remaining trajectory for restore.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Run {
    /// Owning physical chassis id.
    pub owner: u32,
    /// Owner's team.
    pub team: Team,
    /// Monotonic host job identity; stale workers cannot replace it.
    pub generation: u64,
    /// Chassis placement/life revision at admission.
    pub placement_revision: u64,
    /// Difficulty, one through three.
    pub difficulty: u8,
    /// Current lifecycle phase.
    pub phase: Phase,
    /// Persistent pre-planning Core motion, retained for progress and restore.
    pub positioning: Option<CorePositioning>,
    /// Guarded search queries consumed, not an estimate of solution completion.
    pub checks: u32,
    /// Maximum guarded search queries.
    pub budget: u32,
    /// Human-readable phase or failure reason.
    pub message: String,
    /// Rig origin in world FLU metres and wxyz.
    pub world_from_rig: Pose,
    /// Source calibration used for all motion frames.
    pub model: Model,
    /// Current articulated state in source rig coordinates.
    pub pose: RigPose,
    /// Full validated trajectory, one sample per 128 Hz tick.
    pub frames: Vec<Frame>,
    /// Index of the next trajectory sample.
    pub cursor: u32,
    /// Field time at admission, nanoseconds.
    pub started_ns: u64,
    /// Manipulation evaluator including its dwell and confirmation state.
    pub attempt: crate::gameplay::rmuc_2027::manipulation::Attempt,
    /// Whether completion must settle a native match reward attempt.
    pub awards_match: bool,
}
impl Run {
    /// Whether planning or actuation currently owns the arm.
    pub fn active(&self) -> bool {
        matches!(
            self.phase,
            Phase::Positioning | Phase::Solving | Phase::Actuating
        )
    }
    /// Source rig Engineer body expressed in world FLU metres and wxyz.
    pub fn body_world(&self) -> Pose {
        compose(self.world_from_rig, self.pose.engineer_body)
    }
    /// Validate bounded decoded state before accepting a restore.
    pub fn validate(&self) -> Result<(), &'static str> {
        if !(1..=3).contains(&self.difficulty)
            || self.budget == 0
            || self.frames.len() > 4096
            || self.cursor as usize > self.frames.len()
            || self.message.len() > 512
            || self.model.engineer_jaws.len() != 2
            || self.model.engineer_auxiliary.len() > 8
            || self.model.core_envelope_m.len() > 8192
            || self.checks > self.budget
            || (self.phase == Phase::Actuating && self.frames.is_empty())
        {
            return Err("invalid automatic assembly state");
        }
        self.model.validate()?;
        if self.phase == Phase::Positioning && self.positioning.is_none() {
            return Err("missing Core positioning motion");
        }
        if let Some(motion) = &self.positioning {
            motion.validate(&self.model)?;
            if (self.phase == Phase::Positioning && motion.elapsed_ns == motion.duration_ns)
                || (matches!(
                    self.phase,
                    Phase::Solving | Phase::Actuating | Phase::Completed
                ) && motion.elapsed_ns != motion.duration_ns)
            {
                return Err("Core positioning phase disagrees with motion progress");
            }
        }
        let valid = |p: Pose| {
            p.translation_m
                .into_iter()
                .chain(p.rotation_wxyz)
                .all(f64::is_finite)
                && (p.rotation_wxyz.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-6
        };
        if !valid(self.world_from_rig)
            || !valid(self.pose.engineer_body)
            || !valid(self.pose.unit_pose)
            || !self
                .pose
                .engineer_q
                .into_iter()
                .chain(self.pose.core_q)
                .chain(self.pose.jaw_angles)
                .chain([self.pose.lift_m, self.pose.p_rad, self.pose.q_rad])
                .all(f64::is_finite)
        {
            return Err("invalid assembly pose");
        }
        for f in &self.frames {
            if !valid(f.unit)
                || !f
                    .joints_rad
                    .into_iter()
                    .chain(f.jaws_rad)
                    .chain(f.guide_m)
                    .chain(f.angles_rad)
                    .all(f64::is_finite)
                || f.joints_rad
                    .iter()
                    .zip(&self.model.engineer.axes)
                    .any(|(q, a)| *q < a.range_rad[0] - 1e-6 || *q > a.range_rad[1] + 1e-6)
                || f.stage > 6
            {
                return Err("invalid assembly trajectory");
            }
        }
        Ok(())
    }
    /// Apply one validated sample and observe measured manipulation on field time.
    pub fn advance(&mut self, now_ns: u64) {
        use crate::gameplay::rmuc_2027::manipulation::Sensors;
        if self.phase == Phase::Positioning {
            if let Some(motion) = &mut self.positioning
                && motion.advance(&mut self.pose)
            {
                self.phase = Phase::Solving;
                self.started_ns = now_ns;
                self.message = "Core positioned; searching for a validated assembly path".into();
            }
            return;
        }
        if self.phase != Phase::Actuating {
            return;
        }
        if let Some(f) = self.frames.get(self.cursor as usize) {
            let dt = crate::tick_ns() as f64 * 1e-9;
            if (0..6).any(|i| {
                (f.joints_rad[i] - self.pose.engineer_q[i]).abs() > JOINT_SPEED_RAD_S[i] * dt + 2e-6
            }) || (0..2).any(|i| {
                (f.jaws_rad[i] - self.pose.jaw_angles[i]).abs() > JAW_SPEED_RAD_S * dt + 2e-6
            }) {
                self.phase = Phase::Failed;
                self.message = "Trajectory exceeds joint speed limits".into();
                return;
            }
            self.pose.engineer_q = f.joints_rad;
            self.pose.jaw_angles = f.jaws_rad;
            self.pose.unit_pose = f.unit;
            self.pose.lift_m = f.guide_m[2] - 0.6;
            self.pose.p_rad = f.angles_rad[0];
            self.pose.q_rad = f.angles_rad[1];
            self.pose.gripped = f.gripped;
            let sensors = Sensors {
                attached: f.gripped,
                insertion_m: 0.05 - f.guide_m[0],
                lift_m: self.pose.lift_m,
                p_rad: self.pose.p_rad,
                q_rad: self.pose.q_rad,
                axis_error_rad: f.angles_rad[2].abs(),
                lateral_error_m: (f.guide_m[1] - 0.2).abs(),
            };
            if let Err(reason) =
                self.attempt
                    .observe(now_ns, [sensors, Sensors::default()], true, true)
            {
                self.phase = Phase::Failed;
                self.message = reason.into();
                return;
            }
            self.cursor += 1;
            self.message = [
                "Approaching",
                "Gripping",
                "Inserting",
                "Lifting",
                "Flipping",
                "Aligning",
                "Settling",
            ][f.stage as usize]
                .into();
        }
        if self.cursor as usize == self.frames.len() {
            match self.attempt.confirm(0) {
                Ok(()) => {
                    self.phase = Phase::Completed;
                    self.message = "Assembly complete".into();
                }
                Err(reason) => {
                    self.phase = Phase::Failed;
                    self.message = reason.into();
                }
            }
        }
    }
    /// Cancel without changing any physical chassis placement.
    pub fn cancel(&mut self, reason: &str) {
        self.phase = Phase::Cancelled;
        self.message = reason.chars().take(256).collect();
    }
    /// Field time remaining before the prototype 45-second attempt deadline.
    pub fn timed_out(&self, now_ns: u64) -> bool {
        self.phase != Phase::Positioning && now_ns.saturating_sub(self.started_ns) >= 45_000_000_000
    }
}

impl Run {
    /// Whether the chassis remains within the stationary fixture's 20 mm/0.03 rad
    /// allowance. This is a simulator stability allowance, not a rulebook limit.
    pub fn parking_matches(&self, actual: Pose) -> bool {
        let expected = self.body_world();
        position(expected).distance(position(actual)) <= 0.02
            && quaternion(expected).angle_between(quaternion(actual)) <= 0.03
    }
}
impl crate::Field {
    /// Persistent source arms, including idle and completed robot postures.
    pub fn arms(&self) -> &[ArmState] {
        &self.arms
    }
    /// Install a verified source arm for an existing chassis, once at deployment.
    pub fn add_arm(&mut self, arm: ArmState) -> Result<(), crate::FieldError> {
        arm.validate().map_err(crate::FieldError::Restore)?;
        if self.chassis_snapshot(arm.owner).is_none()
            || self.arms.iter().any(|a| a.owner == arm.owner)
        {
            return Err(crate::FieldError::Chassis("missing or duplicate arm owner"));
        }
        self.arms.push(arm);
        Ok(())
    }
    /// Host-owned automatic assembly jobs, including their restorable motion.
    pub fn assemblies(&self) -> &[Run] {
        &self.assemblies
    }
    /// Admit or replace an owned source-bound job after validating its trajectory.
    pub fn set_assembly(&mut self, run: Run) -> Result<(), crate::FieldError> {
        run.validate().map_err(crate::FieldError::Restore)?;
        if !self
            .chassis_snapshot(run.owner)
            .is_some_and(|c| c.team == run.team && c.placement_revision == run.placement_revision)
        {
            return Err(crate::FieldError::Chassis(
                "missing or replaced assembly chassis",
            ));
        }
        if run.phase == Phase::Actuating
            && run.cursor == 0
            && let Some(arm) = self.arms.iter().find(|a| a.owner == run.owner)
            && ((0..6).any(|i| (run.pose.engineer_q[i] - arm.joints_rad[i]).abs() > 1e-6)
                || (0..2).any(|i| (run.pose.jaw_angles[i] - arm.jaws_rad[i]).abs() > 1e-6))
        {
            return Err(crate::FieldError::Restore(
                "assembly must start from actual arm joints",
            ));
        }
        if let Some(existing) = self.assemblies.iter_mut().find(|r| r.owner == run.owner) {
            if existing.generation > run.generation {
                return Err(crate::FieldError::Restore("stale assembly generation"));
            }
            *existing = run;
        } else {
            if self
                .assemblies
                .iter()
                .any(|r| r.active() && r.team == run.team)
            {
                return Err(crate::FieldError::Chassis("assembly station is busy"));
            }
            self.assemblies.retain(|r| r.team != run.team);
            self.assemblies.push(run);
        }
        Ok(())
    }
    /// Cancel the chassis' job without moving it or awarding completion.
    pub fn cancel_assembly(&mut self, owner: u32, reason: &str) {
        if self
            .assemblies
            .iter()
            .any(|r| r.owner == owner && r.active())
            && let Some(arm) = self.arms.iter_mut().find(|a| a.owner == owner)
        {
            arm.target_rad = arm.joints_rad;
            arm.jaw_target_rad = std::array::from_fn(|i| arm.model.engineer_jaws[i].open_rad);
        }
        let team = self
            .assemblies
            .iter_mut()
            .find(|r| r.owner == owner && r.active())
            .and_then(|run| {
                let admitted = run.phase != Phase::Positioning;
                run.cancel(reason);
                (run.awards_match && admitted).then_some(run.team)
            });
        if let Some(team) = team {
            let _ = self.referee_command(crate::RefereeCommand::CancelAssembly { team });
        }
    }
    pub(crate) fn advance_assemblies(&mut self) {
        for arm in &mut self.arms {
            let Some(chassis) = self.physics.chassis_snapshot(arm.owner) else {
                continue;
            };
            if arm.folded != chassis.command.folded {
                arm.folded = chassis.command.folded;
                // Same source-fitted seven-degree shoulder/elbow stow as PIE's
                // tunnel mechanism; opening returns to the raised neutral preset.
                arm.target_rad = if arm.folded {
                    [0., -7f64.to_radians(), 7f64.to_radians(), 0., 0., 0.]
                } else {
                    NEUTRAL_JOINTS_RAD
                };
            }
            if !self
                .assemblies
                .iter()
                .any(|r| r.owner == arm.owner && r.active())
                && !chassis.defeated
            {
                arm.step();
            }
        }
        let now = self.time_ns();
        let mut settle = Vec::new();
        let mut begin = Vec::new();
        for run in &mut self.assemblies {
            if !run.active() {
                continue;
            }
            let was_positioning = run.phase == Phase::Positioning;
            let valid = self.physics.chassis_snapshot(run.owner).is_some_and(|c| {
                c.placement_revision == run.placement_revision
                    && !c.defeated
                    && run.parking_matches(c.pose)
            });
            if !valid {
                run.cancel("Parking changed or robot unavailable");
            } else if run.timed_out(now) {
                run.phase = Phase::Failed;
                run.message = "Assembly timed out".into();
            } else {
                let cursor = run.cursor;
                run.advance(now);
                if was_positioning && run.phase == Phase::Solving && run.awards_match {
                    begin.push((run.owner, run.team, run.difficulty));
                }
                if run.cursor != cursor
                    && let Some(arm) = self.arms.iter_mut().find(|a| a.owner == run.owner)
                {
                    arm.joints_rad = run.pose.engineer_q;
                    arm.target_rad = arm.joints_rad;
                    arm.jaws_rad = run.pose.jaw_angles;
                    arm.jaw_target_rad = arm.jaws_rad;
                    if run.phase == Phase::Completed {
                        arm.jaw_target_rad =
                            std::array::from_fn(|i| arm.model.engineer_jaws[i].open_rad);
                        run.pose.gripped = false;
                    }
                }
            }
            if !run.active() && run.awards_match && !was_positioning {
                settle.push((run.team, run.phase == Phase::Completed, run.owner));
            }
        }
        for (owner, team, difficulty) in begin {
            if let Err(error) =
                self.referee_command(crate::RefereeCommand::StartAssembly { team, difficulty })
                && let Some(run) = self.assemblies.iter_mut().find(|r| r.owner == owner)
            {
                run.phase = Phase::Failed;
                run.message = error.to_string();
            }
        }
        for (team, success, owner) in settle {
            let command = if success {
                crate::RefereeCommand::FinishAssembly { team }
            } else {
                crate::RefereeCommand::CancelAssembly { team }
            };
            if let Err(error) = self.referee_command(command)
                && success
            {
                let run = self
                    .assemblies
                    .iter_mut()
                    .find(|r| r.owner == owner)
                    .unwrap();
                run.phase = Phase::Failed;
                run.message = error.to_string();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ChassisConfig, ChassisPlacement, Field, RobotKind, StaticGeometry, tick_ns};
    fn fixture() -> (Field, Run) {
        let mut field = Field::new(&Default::default()).unwrap();
        let config = ChassisConfig::default();
        let body = Pose::at([0., 0., config.rest_height_m()]);
        let owner = field
            .add_chassis(&ChassisPlacement {
                config,
                spawn: body,
                team: Team::Red,
                kind: RobotKind::Engineer,
                performance: None,
            })
            .unwrap();
        let arm = SerialArm {
            axes: [rm_simulator_physics::manipulator::RevoluteAxis {
                origin_m: [0.; 3],
                axis: [0., 0., 1.],
                range_rad: [-3.2, 3.2],
            }; 6],
            tool: Pose::default(),
        };
        let model = Model {
            schema: 5,
            assembly_frame: Pose::default(),
            core_envelope_m: vec![],
            engineer: arm.clone(),
            core: arm,
            engineer_jaws: (0..2)
                .map(|i| Jaw {
                    name: format!("jaw-{i}"),
                    origin_m: [0.; 3],
                    axis: [0., 0., 1.],
                    open_rad: 0.,
                    closed_rad: 0.1,
                })
                .collect(),
            engineer_auxiliary: vec![],
            core_p_origin_m: [0.; 3],
            core_p_axis: [0., 1., 0.],
            core_q_origin_m: [0.; 3],
            core_q_axis: [0., 0., 1.],
            core_lift_axis: [0., 0., 1.],
        };
        let pose = RigPose {
            engineer_body: body,
            engineer_q: [0.; 6],
            core_q: [0.; 6],
            unit_pose: Pose::default(),
            jaw_angles: [0.; 2],
            camera_deployed: false,
            lift_m: 0.,
            p_rad: 0.,
            q_rad: 0.,
            gripped: false,
        };
        let frame = Frame {
            joints_rad: [0.; 6],
            jaws_rad: [0.; 2],
            unit: Pose::default(),
            guide_m: [-0.05, 0.2, 0.6],
            angles_rad: [0.; 3],
            gripped: true,
            stage: 2,
        };
        let mut run = Run {
            owner,
            team: Team::Red,
            generation: 1,
            placement_revision: field.chassis_revision(owner).unwrap(),
            difficulty: 1,
            phase: Phase::Actuating,
            positioning: None,
            checks: 1,
            budget: 10,
            message: String::new(),
            world_from_rig: Pose::default(),
            model,
            pose,
            frames: vec![frame; 4],
            cursor: 0,
            started_ns: 0,
            attempt: crate::gameplay::rmuc_2027::manipulation::Attempt::new(1, [0.; 2], 0).unwrap(),
            awards_match: false,
        };
        run.frames[0].guide_m[0] = 0.05;
        (field, run)
    }
    #[test]
    fn core_moves_before_solving_and_restores_mid_motion() {
        let (mut field, mut run) = fixture();
        // Admit from settled parking, as the live host requires.
        field.step(256).unwrap();
        run.pose.engineer_body = field.chassis_snapshot(run.owner).unwrap().pose;
        run.started_ns = field.time_ns();
        run.phase = Phase::Positioning;
        run.frames.clear();
        run.checks = 0;
        let target = [0.4, -0.2, 0.1, 0., 0., 0.];
        run.positioning = Some(CorePositioning::new(&run.pose, target));
        let ticks = run.positioning.as_ref().unwrap().duration_ns / crate::tick_ns();
        let robot = run.pose.engineer_q;
        field.set_assembly(run).unwrap();
        field.step(10).unwrap();
        let before = field.assemblies()[0].clone();
        assert_eq!(before.phase, Phase::Positioning);
        assert!(before.pose.core_q[0] > 0. && before.pose.core_q[0] < target[0]);
        assert_eq!(before.pose.engineer_q, robot);
        assert_eq!(before.checks, 0);
        let mut restored =
            Field::restore(&field.snapshot(), &StaticGeometry::default(), 0.).unwrap();
        for _ in 10..ticks {
            let previous = field.assemblies()[0].pose.core_q;
            field.step(1).unwrap();
            for (a, b) in previous.into_iter().zip(field.assemblies()[0].pose.core_q) {
                assert!((a - b).abs() <= crate::tick_ns() as f64 * 1e-9 + 1e-12);
            }
        }
        restored.step(ticks - 10).unwrap();
        let arrived = &field.assemblies()[0];
        assert_eq!(arrived, &restored.assemblies()[0]);
        assert_eq!(arrived.phase, Phase::Solving);
        assert_eq!(arrived.pose.core_q, target);
        assert_eq!(arrived.pose.engineer_q, robot);
        assert_eq!(arrived.started_ns, field.time_ns());
        assert_eq!(arrived.checks, 0);
    }
    #[test]
    fn cancelling_core_positioning_holds_pose_and_rejects_unsafe_motion() {
        let (mut field, mut run) = fixture();
        run.phase = Phase::Positioning;
        run.frames.clear();
        run.positioning = Some(CorePositioning::new(&run.pose, [1.; 6]));
        let mut bad = run.clone();
        bad.positioning.as_mut().unwrap().duration_ns = crate::tick_ns();
        assert!(bad.validate().is_err());
        field.set_assembly(run).unwrap();
        field.step(10).unwrap();
        field.cancel_assembly(field.assemblies()[0].owner, "test");
        let pose = field.assemblies()[0].pose.clone();
        field.step(128).unwrap();
        assert_eq!(field.assemblies()[0].pose, pose);
        assert_eq!(field.assemblies()[0].phase, Phase::Cancelled);
    }
    #[test]
    fn core_resets_previous_lift_p_and_q_at_bounded_speeds() {
        let (_, run) = fixture();
        let mut pose = run.pose;
        pose.lift_m = 0.1;
        pose.p_rad = std::f64::consts::FRAC_PI_2;
        pose.q_rad = -0.8;
        let mut motion = CorePositioning::new(&pose, [0.; 6]);
        motion.validate(&run.model).unwrap();
        while motion.elapsed_ns < motion.duration_ns {
            let old = pose.clone();
            motion.advance(&mut pose);
            let dt = crate::tick_ns() as f64 * 1e-9;
            assert!((old.lift_m - pose.lift_m).abs() <= 0.1 * dt + 1e-12);
            assert!((old.p_rad - pose.p_rad).abs() <= dt + 1e-12);
            assert!((old.q_rad - pose.q_rad).abs() <= dt + 1e-12);
        }
        assert_eq!([pose.lift_m, pose.p_rad, pose.q_rad], [0.; 3]);
    }

    #[test]
    fn restore_continues_the_same_measured_sequence() {
        let (mut field, run) = fixture();
        field.set_assembly(run).unwrap();
        field.step(2).unwrap();
        assert_eq!(field.assemblies()[0].cursor, 2);
        let snapshot = field.snapshot();
        let mut restored = Field::restore(&snapshot, &StaticGeometry::default(), 0.).unwrap();
        field.step(2).unwrap();
        restored.step(1).unwrap();
        restored.step(1).unwrap();
        assert_eq!(field.assemblies(), restored.assemblies());
        assert_eq!(field.assemblies()[0].phase, Phase::Completed);
    }
    #[test]
    fn driving_cancels_without_moving_or_confirming() {
        let (mut field, run) = fixture();
        let id = run.owner;
        field.set_assembly(run).unwrap();
        let before = field.chassis_snapshot(id).unwrap().pose;
        field
            .command_chassis(
                id,
                crate::ChassisCommand {
                    forward_m_s: 1.,
                    ..Default::default()
                },
            )
            .unwrap();
        assert_eq!(field.chassis_snapshot(id).unwrap().pose, before);
        assert_eq!(field.assemblies()[0].phase, Phase::Cancelled);
        assert!(!field.assemblies()[0].attempt.completed());
    }
    #[test]
    fn invalid_and_stale_jobs_are_rejected_and_timeouts_fail() {
        let (mut field, mut run) = fixture();
        run.model.core_p_axis = [0.; 3];
        assert!(run.validate().is_err());
        run.model.core_p_axis = [0., 1., 0.];
        field.set_assembly(run.clone()).unwrap();
        run.generation = 0;
        assert!(field.set_assembly(run.clone()).is_err());
        run.generation = 2;
        run.phase = Phase::Solving;
        run.started_ns = 0;
        field.set_assembly(run).unwrap();
        assert!(field.assemblies()[0].timed_out(45_000_000_000));
        assert!(!field.assemblies()[0].timed_out(45_000_000_000 - tick_ns()));
        field.remove_chassis(0).unwrap();
        assert_eq!(field.assemblies()[0].phase, Phase::Cancelled);
    }
    #[test]
    fn neutral_and_tunnel_targets_move_with_bounded_joint_rates_and_restore() {
        let (mut field, run) = fixture();
        let id = run.owner;
        field.add_arm(ArmState::neutral(id, run.model)).unwrap();
        field.step(8).unwrap();
        assert_eq!(field.arms()[0].joints_rad, NEUTRAL_JOINTS_RAD);
        field
            .command_chassis(
                id,
                crate::ChassisCommand {
                    folded: true,
                    ..Default::default()
                },
            )
            .unwrap();
        let before = field.arms()[0].joints_rad;
        field.step(1).unwrap();
        let after = field.arms()[0].joints_rad;
        assert!((0..6).all(|i| (after[i] - before[i]).abs()
            <= JOINT_SPEED_RAD_S[i] * tick_ns() as f64 * 1e-9 + 1e-12));
        assert_ne!(after, field.arms()[0].target_rad);
        let mut restored =
            Field::restore(&field.snapshot(), &StaticGeometry::default(), 0.).unwrap();
        field.step(128).unwrap();
        restored.step(64).unwrap();
        restored.step(64).unwrap();
        assert_eq!(field.arms(), restored.arms());
        assert_eq!(
            field.arms()[0].joints_rad,
            [0., -7f64.to_radians(), 7f64.to_radians(), 0., 0., 0.]
        );
        field
            .command_chassis(id, crate::ChassisCommand::default())
            .unwrap();
        field.step(128).unwrap();
        assert_eq!(field.arms()[0].joints_rad, NEUTRAL_JOINTS_RAD);
    }
    #[test]
    fn finishing_holds_actual_joints_and_refuses_a_prepared_pose_jump() {
        let (mut field, mut run) = fixture();
        let mut arm = ArmState::neutral(run.owner, run.model.clone());
        arm.joints_rad = run.pose.engineer_q;
        arm.target_rad = arm.joints_rad;
        arm.jaws_rad = run.pose.jaw_angles;
        arm.jaw_target_rad = arm.jaws_rad;
        field.add_arm(arm).unwrap();
        let mut invalid = run.clone();
        invalid.pose.engineer_q[0] = 1.;
        assert!(field.set_assembly(invalid).is_err());
        for (i, f) in run.frames.iter_mut().enumerate() {
            f.joints_rad[0] = (i + 1) as f64 * 0.002;
        }
        field.set_assembly(run).unwrap();
        field.step(4).unwrap();
        assert_eq!(field.assemblies()[0].phase, Phase::Completed);
        let joints = field.arms()[0].joints_rad;
        assert_eq!(joints[0], 0.008);
        field.step(128).unwrap();
        assert_eq!(field.arms()[0].joints_rad, joints);
        assert_ne!(field.arms()[0].joints_rad, NEUTRAL_JOINTS_RAD);
    }
    #[test]
    fn cancel_preserves_intermediate_joints_and_idle_restore() {
        let (mut field, mut run) = fixture();
        let mut arm = ArmState::neutral(run.owner, run.model.clone());
        arm.joints_rad = run.pose.engineer_q;
        arm.target_rad = arm.joints_rad;
        field.add_arm(arm).unwrap();
        for (i, frame) in run.frames.iter_mut().enumerate() {
            frame.joints_rad[0] = (i + 1) as f64 * 0.002;
        }
        let id = run.owner;
        field.set_assembly(run).unwrap();
        field.step(2).unwrap();
        let actual = field.arms()[0].joints_rad;
        field.cancel_assembly(id, "pilot cancelled");
        let mut restored =
            Field::restore(&field.snapshot(), &StaticGeometry::default(), 0.).unwrap();
        field.step(128).unwrap();
        restored.step(128).unwrap();
        assert_eq!(field.arms(), restored.arms());
        assert_eq!(field.arms()[0].joints_rad, actual);
        assert_eq!(field.assemblies()[0].phase, Phase::Cancelled);
    }
    #[test]
    fn persistent_arms_reject_invalid_calibration_before_kinematics() {
        let (mut field, run) = fixture();
        let mut arm = ArmState::neutral(run.owner, run.model);
        arm.model.engineer_jaws.clear();
        assert!(field.add_arm(arm).is_err());
        assert!(field.arms().is_empty());
    }
    #[test]
    fn a_speed_violation_fails_without_applying_the_joint_jump() {
        let (_, mut run) = fixture();
        run.frames[0].joints_rad[0] = 1.;
        let before = run.pose.engineer_q;
        run.advance(tick_ns());
        assert_eq!(run.phase, Phase::Failed);
        assert_eq!(run.pose.engineer_q, before);
    }
}
