// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Restorable automatic assembly motion on the owning field clock.
use crate::{Pose, Team};
use glam::DQuat;
/// Compatibility exports of the reusable source mechanical model.
pub use rm_sim_physics::assembly_model::{Auxiliary, Jaw, Model, RigPose};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
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
    fn step(&mut self, step_duration: crate::StepDuration) {
        let dt = step_duration.as_secs_f64();
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
    /// Insertion, lift and flip finished; the newly revealed LED target awaits planning.
    AwaitingTarget,
    /// Planning Q rotation from the actual attached post-flip pose.
    SolvingTwist,
    /// A validated trajectory is being applied one frame per tick.
    Actuating,
    /// Confirmation succeeded; opening jaws and following the validated return path.
    BreakingAway,
    /// Engineer is clear; the Core retracts, tips and drops the spent Unit.
    Recovering,
    /// Assembly succeeded and the Core finished recovery and reset.
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
    /// Planner stage identifier: approach, grasp, insert, lift, flip, align, settle, breakaway.
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
    /// Quantized duration, nanoseconds; at least one owning-field step.
    pub duration_ns: u64,
}
impl CorePositioning {
    /// Prepare bounded simultaneous travel from the current Core mechanism pose.
    pub fn new(pose: &RigPose, target_rad: [f64; 6]) -> Self {
        Self::with_step_duration(pose, target_rad, crate::StepDuration::default())
    }
    /// Prepare Core travel quantized to the owning field's exact duration.
    pub fn with_step_duration(
        pose: &RigPose,
        target_rad: [f64; 6],
        step_duration: crate::StepDuration,
    ) -> Self {
        let seconds = pose
            .core_q
            .into_iter()
            .zip(target_rad)
            .map(|(a, b)| (a - b).abs())
            .chain([pose.p_rad.abs(), pose.q_rad.abs(), pose.lift_m.abs() / 0.1])
            .fold(0., f64::max);
        let ticks = (seconds / step_duration.as_secs_f64()).ceil().max(1.) as u64;
        Self {
            from_rad: pose.core_q,
            target_rad,
            lift_m: pose.lift_m,
            angles_rad: [pose.p_rad, pose.q_rad],
            elapsed_ns: 0,
            duration_ns: ticks.saturating_mul(step_duration.as_ns()),
        }
    }
    /// Fraction of the selected motion completed, from zero through one.
    ///
    /// ```
    /// use rm_sim_world::assembly::CorePositioning;
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
    #[cfg(test)]
    fn advance(&mut self, pose: &mut RigPose) -> bool {
        self.advance_with_step_duration(pose, crate::StepDuration::default())
    }
    fn advance_with_step_duration(
        &mut self,
        pose: &mut RigPose,
        step_duration: crate::StepDuration,
    ) -> bool {
        self.elapsed_ns = self
            .elapsed_ns
            .saturating_add(step_duration.as_ns())
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
    #[cfg(test)]
    fn validate(&self, model: &Model) -> Result<(), &'static str> {
        self.validate_with_step_duration(model, crate::StepDuration::default())
    }
    fn validate_with_step_duration(
        &self,
        model: &Model,
        step_duration: crate::StepDuration,
    ) -> Result<(), &'static str> {
        if self.duration_ns == 0
            || self.duration_ns > 60_000_000_000
            || self.elapsed_ns > self.duration_ns
            || !self.duration_ns.is_multiple_of(step_duration.as_ns())
            || !self.elapsed_ns.is_multiple_of(step_duration.as_ns())
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

/// Prescribed post-success Core recovery. UC §5.4.3 step 6 requires recovery,
/// but its route, tip and delays here are simulator policies, not measured hardware.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CoreRecovery {
    /// Retraction to the manual initial pose, using the existing 1 rad/s, 0.1 m/s policy.
    pub motion: CorePositioning,
    /// Whole-Core wrist motion to point the shaft downward, without reversing the P hinge.
    pub tip: CorePositioning,
    /// Elapsed explicit field time, nanoseconds, through retract/tip/drop/reset.
    pub elapsed_ns: u64,
}
impl CoreRecovery {
    /// Prepare fixed idle/discharge branches; withdrawal captures the actual Core state.
    pub fn new(pose: &RigPose, idle_rad: [f64; 6], drop_rad: [f64; 6]) -> Self {
        Self::with_step_duration(pose, idle_rad, drop_rad, crate::StepDuration::default())
    }
    /// Prepare recovery motions quantized to the owning field's step.
    pub fn with_step_duration(
        pose: &RigPose,
        idle_rad: [f64; 6],
        drop_rad: [f64; 6],
        step_duration: crate::StepDuration,
    ) -> Self {
        let motion = CorePositioning::with_step_duration(pose, idle_rad, step_duration);
        let mut idle = pose.clone();
        idle.core_q = idle_rad;
        idle.lift_m = 0.;
        idle.p_rad = 0.;
        idle.q_rad = 0.;
        Self {
            motion,
            tip: CorePositioning::with_step_duration(&idle, drop_rad, step_duration),
            elapsed_ns: 0,
        }
    }
    fn begin(&mut self, pose: &RigPose, step_duration: crate::StepDuration) {
        *self = Self::with_step_duration(
            pose,
            self.motion.target_rad,
            self.tip.target_rad,
            step_duration,
        );
    }
    /// Total prescribed duration, nanoseconds, including a 250 ms stationary release hold and 500 ms gravity clearance dwell.
    pub fn duration_ns(&self) -> u64 {
        self.motion.duration_ns + 2 * self.tip.duration_ns + 750_000_000
    }
    /// Whether the shaft has tipped down and released the Unit into rigid-body physics.
    pub fn released(&self) -> bool {
        self.elapsed_ns >= self.motion.duration_ns + self.tip.duration_ns + 250_000_000
    }
    /// Fraction of the full recovery completed, inclusive zero through one.
    pub fn progress(&self) -> f32 {
        self.elapsed_ns as f32 / self.duration_ns() as f32
    }
    /// Advance one default 128 Hz tick. The Engineer and its jaws remain untouched.
    pub fn advance(&mut self, pose: &mut RigPose) -> bool {
        self.advance_with_step_duration(pose, crate::StepDuration::default())
    }
    /// Advance one owning-field step; Engineer joints and jaws remain untouched.
    pub fn advance_with_step_duration(
        &mut self,
        pose: &mut RigPose,
        step_duration: crate::StepDuration,
    ) -> bool {
        self.elapsed_ns = (self.elapsed_ns + step_duration.as_ns()).min(self.duration_ns());
        if self.elapsed_ns <= self.motion.duration_ns {
            self.motion.advance_with_step_duration(pose, step_duration);
        } else {
            let elapsed = self.elapsed_ns - self.motion.duration_ns;
            if elapsed <= self.tip.duration_ns {
                self.tip.advance_with_step_duration(pose, step_duration);
            } else if elapsed > self.tip.duration_ns + 750_000_000 {
                let t = (elapsed - self.tip.duration_ns - 750_000_000) as f64
                    / self.tip.duration_ns as f64;
                pose.core_q = std::array::from_fn(|i| {
                    self.tip.target_rad[i] + (self.tip.from_rad[i] - self.tip.target_rad[i]) * t
                });
                if self.elapsed_ns == self.duration_ns() {
                    pose.core_q = self.motion.target_rad;
                }
            }
        }
        self.elapsed_ns == self.duration_ns()
    }
    fn validate_with_step_duration(
        &self,
        model: &Model,
        step_duration: crate::StepDuration,
    ) -> Result<(), &'static str> {
        self.motion
            .validate_with_step_duration(model, step_duration)?;
        self.tip.validate_with_step_duration(model, step_duration)?;
        if self.tip.from_rad != self.motion.target_rad
            || self.tip.lift_m != 0.
            || self.tip.angles_rad != [0.; 2]
            || self.motion.elapsed_ns != self.elapsed_ns.min(self.motion.duration_ns)
            || self.tip.elapsed_ns
                != self
                    .elapsed_ns
                    .saturating_sub(self.motion.duration_ns)
                    .min(self.tip.duration_ns)
            || self.elapsed_ns > self.duration_ns()
            // The exact terminal time includes a fixed 750 ms dwell, which need
            // not lie on an arbitrary owning-field step grid. Advance clamps
            // there; intermediate elapsed values must still be step aligned.
            || (!self.elapsed_ns.is_multiple_of(step_duration.as_ns())
                && self.elapsed_ns != self.duration_ns())
        {
            return Err("invalid Core recovery time or branches");
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
    /// Optional live post-success recovery; contains its fixed idle branch and restorable progress.
    #[serde(default)]
    pub recovery: Option<CoreRecovery>,
    /// Observed Q target, radians; absent until a level-three Unit is inserted,
    /// lifted and fully flipped. No initial plan may consume this value.
    pub twist_target_rad: Option<f64>,
    /// Fixed wrist-to-Unit transform for loaded motion; absent for legacy/offline fixtures.
    #[serde(default)]
    pub unit_grasp: Option<Pose>,
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
    /// Full validated trajectory, one sample per owning-field step.
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
            Phase::Positioning
                | Phase::Solving
                | Phase::AwaitingTarget
                | Phase::SolvingTwist
                | Phase::Actuating
                | Phase::BreakingAway
        )
    }
    /// Whether this station is reserved by manipulation or automatic equipment recovery.
    pub fn station_busy(&self) -> bool {
        self.active() || self.phase == Phase::Recovering
    }
    /// Source rig Engineer body expressed in world FLU metres and wxyz.
    pub fn body_world(&self) -> Pose {
        compose(self.world_from_rig, self.pose.engineer_body)
    }
    /// Validate bounded decoded state before accepting a restore.
    pub fn validate(&self) -> Result<(), &'static str> {
        self.validate_with_step_duration(crate::StepDuration::default())
    }
    /// Validate decoded job authority against its owning field's time grid.
    pub fn validate_with_step_duration(
        &self,
        step_duration: crate::StepDuration,
    ) -> Result<(), &'static str> {
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
        if let Some(recovery) = &self.recovery {
            recovery.validate_with_step_duration(&self.model, step_duration)?;
        }
        if self.phase == Phase::Recovering && self.recovery.is_none() {
            return Err("missing Core recovery motion");
        }
        if self.phase == Phase::Positioning && self.positioning.is_none() {
            return Err("missing Core positioning motion");
        }
        if let Some(motion) = &self.positioning {
            motion.validate_with_step_duration(&self.model, step_duration)?;
            if (self.phase == Phase::Positioning && motion.elapsed_ns == motion.duration_ns)
                || (matches!(
                    self.phase,
                    Phase::Solving
                        | Phase::AwaitingTarget
                        | Phase::SolvingTwist
                        | Phase::Actuating
                        | Phase::BreakingAway
                        | Phase::Completed
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
            || self.unit_grasp.is_some_and(|g| !valid(g))
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
        if self
            .twist_target_rad
            .is_some_and(|q| !q.is_finite() || q.abs() > std::f64::consts::FRAC_PI_2)
            || (self.difficulty == 3
                && self.twist_target_rad.is_none()
                && self.attempt.target_q_rad != [0.; 2])
            || (matches!(self.phase, Phase::AwaitingTarget | Phase::SolvingTwist)
                && (self.difficulty != 3 || self.twist_target_rad.is_none()))
        {
            return Err("invalid or premature Q target");
        }
        let mut returning = false;
        for f in &self.frames {
            if f.stage == 7 {
                returning = true;
            }
            if returning && (f.stage != 7 || f.gripped) {
                return Err("invalid breakaway trajectory suffix");
            }
            if self.difficulty == 3 && self.twist_target_rad.is_none() && f.stage >= 5 {
                return Err("unobserved Q target cannot enter a planned alignment");
            }

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
                || f.stage > 7
            {
                return Err("invalid assembly trajectory");
            }
        }
        Ok(())
    }
    /// Apply one validated sample and observe measured manipulation on field time.
    pub fn advance(&mut self, now_ns: u64) {
        self.advance_with_step_duration(now_ns, crate::StepDuration::default());
    }
    /// Apply one trajectory sample using the owning field's duration for speed limits.
    pub fn advance_with_step_duration(&mut self, now_ns: u64, step_duration: crate::StepDuration) {
        use crate::gameplay::rmuc_2027::manipulation::Sensors;
        if self.phase == Phase::Recovering {
            if self
                .recovery
                .as_mut()
                .is_some_and(|r| r.advance_with_step_duration(&mut self.pose, step_duration))
            {
                self.phase = Phase::Completed;
                self.message = "Assembly complete; Unit recovered, Core idle".into();
            } else {
                self.message = if self.recovery.as_ref().is_some_and(CoreRecovery::released) {
                    "Unit dropped; resetting Core"
                } else {
                    "Recovering spent Unit"
                }
                .into();
            }
            return;
        }
        if self.phase == Phase::Positioning {
            if let Some(motion) = &mut self.positioning
                && motion.advance_with_step_duration(&mut self.pose, step_duration)
            {
                self.phase = Phase::Solving;
                self.started_ns = now_ns;
                self.message = "Core positioned; searching for a validated assembly path".into();
            }
            return;
        }
        if !matches!(self.phase, Phase::Actuating | Phase::BreakingAway) {
            return;
        }
        if let Some(f) = self.frames.get(self.cursor as usize) {
            let dt = step_duration.as_secs_f64();
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
            if self.phase == Phase::Actuating
                && let Err(reason) =
                    self.attempt
                        .observe(now_ns, [sensors, Sensors::default()], true, true)
            {
                self.phase = Phase::Failed;
                self.message = reason.into();
                return;
            }
            if self.difficulty == 3 && self.twist_target_rad.is_none() {
                // Zero is a placeholder, not a revealed target. Until the LED
                // is available it cannot satisfy alignment or its dwell.
                use crate::gameplay::rmuc_2027::manipulation::Step;
                let core = &mut self.attempt.cores[0];
                if matches!(core.step, Step::Align | Step::Ready) {
                    core.step = Step::Align;
                    core.aligned_since_ns = None;
                    core.milestones_ns[2] = None;
                }
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
                "Releasing and returning to neutral",
            ][f.stage as usize]
                .into();
        }
        if self.phase == Phase::BreakingAway {
            if self.cursor as usize == self.frames.len() {
                if let Some(recovery) = &mut self.recovery {
                    recovery.begin(&self.pose, step_duration);
                    self.phase = Phase::Recovering;
                    self.message = "Engineer clear; recovering spent Unit".into();
                } else {
                    self.phase = Phase::Completed;
                    self.message = "Assembly complete; arm neutral".into();
                }
            }
            return;
        }
        let release = self
            .frames
            .get(self.cursor as usize)
            .is_some_and(|f| f.stage == 7);
        if self.cursor as usize == self.frames.len() || release {
            if self.difficulty == 3 && self.twist_target_rad.is_none() {
                use crate::gameplay::rmuc_2027::{assembly_pose::Targets, manipulation::Step};
                let core = &self.attempt.cores[0];
                if core.step != Step::Align
                    || (self.pose.p_rad - std::f64::consts::FRAC_PI_2).abs() > 1e-5
                {
                    self.phase = Phase::Failed;
                    self.message = "Prefix ended before insertion, lift and full P flip".into();
                    return;
                }
                // Sample only at the physical reveal event. Uniform Table 5-10
                // range is a reproducible simulator policy, not an official PDF.
                let q = Targets::sample(0, self.generation, 3).unwrap().q_rad[0];
                self.twist_target_rad = Some(q);
                self.attempt.target_q_rad = [q, 0.];
                self.phase = Phase::AwaitingTarget;
                self.message = format!("LED revealed Q target {:.1} degrees", q.to_degrees());
                return;
            }
            match self.attempt.confirm(0) {
                Ok(()) => {
                    self.phase = if release {
                        Phase::BreakingAway
                    } else {
                        Phase::Completed
                    };
                    self.pose.gripped = false;
                    self.message = if release {
                        "Assembly confirmed; releasing gripper"
                    } else {
                        "Assembly complete"
                    }
                    .into();
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
        !matches!(
            self.phase,
            Phase::Positioning | Phase::BreakingAway | Phase::Recovering
        ) && now_ns.saturating_sub(self.started_ns) >= 45_000_000_000
    }
}

impl Run {
    /// Derive a carried Unit pose from calibrated joint FK and the fixed grasp.
    /// Returns `None` for a fixture without a rigid loaded grasp. No state advances.
    pub fn carried_unit_pose(&self, joints_rad: [f64; 6]) -> Option<Pose> {
        self.unit_grasp.map(|grasp| {
            compose(
                compose(
                    self.pose.engineer_body,
                    self.model.engineer.tool_pose(joints_rad),
                ),
                grasp,
            )
        })
    }
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
        run.validate_with_step_duration(self.step_duration())
            .map_err(crate::FieldError::Restore)?;
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
                .any(|r| r.station_busy() && r.team == run.team)
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
            if !self
                .energy_units
                .iter()
                .any(|u| u.attachment == crate::energy_unit::Attachment::Held(owner))
            {
                arm.jaw_target_rad = std::array::from_fn(|i| arm.model.engineer_jaws[i].open_rad);
            }
        }
        let team = self
            .assemblies
            .iter_mut()
            .find(|r| r.owner == owner && r.active())
            .and_then(|run| {
                let admitted = !matches!(
                    run.phase,
                    Phase::Positioning | Phase::BreakingAway | Phase::Recovering
                );
                run.cancel(reason);
                (run.awards_match && admitted).then_some(run.team)
            });
        if let Some(team) = team {
            let _ = self.referee_command(crate::RefereeCommand::CancelAssembly { team });
        }
    }
    pub(crate) fn advance_assemblies(&mut self) {
        let step_duration = self.step_duration();
        for arm in &mut self.arms {
            if self.rmuc_2027.as_ref().is_some_and(|s| {
                s.game.phase == crate::rmuc_2027::match_rules::Phase::Running
                    && s.game.robots.iter().any(|r| r.id == arm.owner && !r.online)
            }) {
                continue;
            }
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
                arm.step(step_duration);
            }
        }
        let now = self.time_ns();
        let mut settle = Vec::new();
        let mut begin = Vec::new();
        for run in &mut self.assemblies {
            let offline = self.rmuc_2027.as_ref().is_some_and(|s| {
                s.game.phase == crate::rmuc_2027::match_rules::Phase::Running
                    && s.game.robots.iter().any(|r| r.id == run.owner && !r.online)
            });
            if !run.station_busy() {
                continue;
            }
            let was_positioning = run.phase == Phase::Positioning;
            let was_returning = matches!(run.phase, Phase::BreakingAway | Phase::Recovering);
            let valid = self.physics.chassis_snapshot(run.owner).is_some_and(|c| {
                c.placement_revision == run.placement_revision
                    && !c.defeated
                    && run.parking_matches(c.pose)
            });
            if (offline || !valid) && run.phase != Phase::Recovering {
                run.cancel("Parking changed or robot unavailable");
            } else if run.timed_out(now) {
                run.phase = Phase::Failed;
                run.message = "Assembly timed out".into();
            } else {
                let cursor = run.cursor;
                run.advance_with_step_duration(now, step_duration);
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
            if ((!run.active() && !was_returning)
                || (!was_returning && run.phase == Phase::BreakingAway))
                && run.awards_match
                && !was_positioning
            {
                settle.push((
                    run.team,
                    matches!(run.phase, Phase::Completed | Phase::BreakingAway),
                    run.owner,
                ));
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
pub(crate) mod tests {
    use super::*;
    use crate::{ChassisConfig, ChassisPlacement, Field, RobotKind, StaticGeometry, tick_ns};
    use rm_sim_physics::manipulator::SerialArm;
    pub(crate) fn fixture() -> (Field, Run) {
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
            axes: [rm_sim_physics::manipulator::RevoluteAxis {
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
            recovery: None,
            twist_target_rad: None,
            unit_grasp: None,
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
    fn thousand_hz_playback_enforces_instance_joint_speed() {
        let (_, mut run) = fixture();
        run.frames[0].joints_rad[0] = 0.002;
        let mut default_run = run.clone();
        default_run.advance(crate::tick_ns());
        assert_ne!(default_run.phase, Phase::Failed);
        run.advance_with_step_duration(1_000_000, crate::StepDuration::HZ_1000);
        assert_eq!(run.phase, Phase::Failed);
        assert_eq!(run.pose.engineer_q[0], 0.);
    }
    #[test]
    fn autonomous_core_recovery_completes_while_engineer_is_offline() {
        use crate::rmuc_2027::{Config, Detector, Tower};
        let detector = Detector {
            pose: Pose::at([10., 0., 1.]),
            half_m: [0.045, 0.044],
        };
        let mut field = Field::new(&crate::FieldConfig {
            rmuc_2027: Some(Config {
                bases: [vec![detector.clone()], vec![detector.clone()]],
                outposts: std::array::from_fn(|_| Tower {
                    pivot_m: [10., 0., 1.],
                    axis: [0., 0., 1.],
                    detectors: vec![detector.clone(); 3],
                }),
                supply_zones: vec![],
                vantage_zones: vec![],
                terrain_zones: vec![],
                runes: vec![],
                zones: vec![],
                rotor_direction: -1,
            }),
            ..Default::default()
        })
        .unwrap();
        let (_, mut run) = fixture();
        let owner = field
            .add_chassis(&crate::ChassisPlacement {
                config: Default::default(),
                spawn: run.pose.engineer_body,
                team: Team::Red,
                kind: RobotKind::Engineer,
                performance: None,
            })
            .unwrap();
        field
            .referee_command(crate::RefereeCommand::StartMatch)
            .unwrap();
        field.step(128 * 5).unwrap();
        assert_eq!(
            field.rmuc_2027().unwrap().game.phase,
            crate::gameplay::rmuc_2027::match_rules::Phase::Running
        );
        run.owner = owner;
        run.pose.engineer_body = field.chassis_control_state(owner).unwrap().pose;
        run.placement_revision = field.chassis_revision(owner).unwrap();
        run.phase = Phase::Recovering;
        run.recovery = Some(CoreRecovery::new(&run.pose, [0.4; 6], [0.2; 6]));
        field.set_assembly(run).unwrap();
        field
            .referee_command(crate::RefereeCommand::SetRobotModules {
                robot: owner,
                main_online: false,
                speed_online: true,
                offline_armor: 0,
                supercap_online: true,
            })
            .unwrap();
        field.step(128).unwrap();
        assert_eq!(field.assemblies()[0].phase, Phase::Recovering);
        assert!(field.assemblies()[0].recovery.as_ref().unwrap().elapsed_ns > 0);
        let mut restored =
            Field::restore(&field.snapshot(), &field.fixed_geometry_snapshot(), 0.).unwrap();
        field.step(128 * 6).unwrap();
        restored.step(128 * 6).unwrap();
        assert_eq!(field.assemblies()[0].phase, Phase::Completed);
        assert_eq!(field.assemblies(), restored.assemblies());
    }

    #[test]
    fn arbitrary_step_completed_recovery_remains_restorable() {
        let (base, mut run) = fixture();
        let duration = crate::StepDuration::from_ns(1_234_567).unwrap();
        let mut checkpoint = base.snapshot();
        checkpoint.restore.as_mut().unwrap().step_duration = duration;
        let mut field = Field::restore(&checkpoint, &base.static_geometry_snapshot(), 0.).unwrap();
        let mut recovery = CoreRecovery::with_step_duration(&run.pose, [0.; 6], [0.2; 6], duration);
        assert!(!recovery.duration_ns().is_multiple_of(duration.as_ns()));
        while !recovery.advance_with_step_duration(&mut run.pose, duration) {
            recovery
                .validate_with_step_duration(&run.model, duration)
                .unwrap();
        }
        recovery
            .validate_with_step_duration(&run.model, duration)
            .unwrap();
        let mut invalid = recovery.clone();
        invalid.elapsed_ns -= 1;
        assert!(
            invalid
                .validate_with_step_duration(&run.model, duration)
                .is_err()
        );
        run.recovery = Some(recovery);
        run.phase = Phase::Completed;
        field.set_assembly(run).unwrap();
        let restored =
            Field::restore(&field.snapshot(), &field.static_geometry_snapshot(), 0.).unwrap();
        assert_eq!(field.assemblies(), restored.assemblies());
    }
    #[test]
    fn thousand_hz_arm_and_core_motion_keep_physical_rates_and_restore() {
        let (base, mut run) = fixture();
        let mut checkpoint = base.snapshot();
        checkpoint.restore.as_mut().unwrap().step_duration = crate::StepDuration::HZ_1000;
        let mut field = Field::restore(&checkpoint, &base.static_geometry_snapshot(), 0.).unwrap();
        let duration = field.step_duration();
        field.step(2000).unwrap(); // Settle parking before admitting Core motion.
        let mut arm = ArmState::neutral(run.owner, run.model.clone());
        arm.target_rad[0] = 0.5;
        field.add_arm(arm).unwrap();
        field.step(10).unwrap();
        assert!((field.arms()[0].joints_rad[0] - 0.01).abs() < 1e-12);
        run.pose.engineer_body = field.chassis_snapshot(run.owner).unwrap().pose;
        run.positioning = Some(CorePositioning::with_step_duration(
            &run.pose, [0.1; 6], duration,
        ));
        run.phase = Phase::Positioning;
        run.validate_with_step_duration(duration).unwrap();
        field.set_assembly(run).unwrap();
        field.step(40).unwrap();
        assert_eq!(
            field.assemblies()[0]
                .positioning
                .as_ref()
                .unwrap()
                .elapsed_ns,
            40_000_000
        );
        let mut restored =
            Field::restore(&field.snapshot(), &field.static_geometry_snapshot(), 0.).unwrap();
        field.step(60).unwrap();
        restored.step(20).unwrap();
        restored.step(40).unwrap();
        assert_eq!(field.assemblies(), restored.assemblies());
        assert_eq!(field.assemblies()[0].phase, Phase::Solving);
        let pose = field.assemblies()[0].pose.clone();
        let mut recovery = CoreRecovery::with_step_duration(&pose, [0.; 6], [0.2; 6], duration);
        let mut pose = pose;
        for _ in 0..50 {
            recovery.advance_with_step_duration(&mut pose, duration);
        }
        assert_eq!(recovery.elapsed_ns, 50_000_000);
        recovery
            .validate_with_step_duration(&field.assemblies()[0].model, duration)
            .unwrap();
    }
    #[test]
    fn final_q_is_revealed_only_after_the_prefix_and_holds_during_replanning() {
        use crate::gameplay::rmuc_2027::manipulation::{Attempt, Step};
        let (mut field, mut run) = fixture();
        run.difficulty = 3;
        run.attempt = Attempt::new(3, [0.; 2], 0).unwrap();
        let mut frames = vec![];
        for (stage, guide, angles) in [
            (0, [0.05, 0.2, 0.6], [0.; 3]),
            (2, [-0.05, 0.2, 0.6], [0.; 3]),
            (3, [-0.05, 0.2, 0.7], [0.; 3]),
            (4, [-0.05, 0.2, 0.7], [std::f64::consts::FRAC_PI_2, 0., 0.]),
        ] {
            let mut f = run.frames[0].clone();
            f.stage = stage;
            f.guide_m = guide;
            f.angles_rad = angles;
            frames.push(f);
        }
        run.frames = frames;
        run.validate().unwrap();
        for tick in 1..4 {
            run.advance(tick * crate::tick_ns());
            assert_eq!(run.twist_target_rad, None);
            assert_eq!(run.attempt.target_q_rad, [0.; 2]);
            assert!(!run.attempt.completed());
        }
        field.set_assembly(run.clone()).unwrap();
        let restored_field =
            Field::restore(&field.snapshot(), &StaticGeometry::default(), 0.).unwrap();
        let mut restored = restored_field.assemblies()[0].clone();
        run.advance(4 * crate::tick_ns());
        restored.advance(4 * crate::tick_ns());
        assert_eq!(run, restored);
        assert_eq!(run.phase, Phase::AwaitingTarget);
        let q = run.twist_target_rad.unwrap();
        assert!(q.abs() <= std::f64::consts::FRAC_PI_2);
        assert_eq!(run.attempt.cores[0].step, Step::Align);
        assert_eq!(run.attempt.cores[0].aligned_since_ns, None);
        assert_eq!(run.attempt.target_q_rad[0], q);
        let held = run.pose.clone();
        run.phase = Phase::SolvingTwist;
        for tick in 5..128 {
            run.advance(tick * crate::tick_ns());
        }
        assert_eq!(run.pose, held);
        assert!(run.pose.gripped);
        run.phase = Phase::Actuating;
        run.cursor = 0;
        let mut aligned = run.frames.last().unwrap().clone();
        aligned.stage = 5;
        aligned.angles_rad[1] = q;
        run.frames = vec![aligned; 64];
        run.validate().unwrap();
        for tick in 128..192 {
            run.advance(tick * crate::tick_ns());
        }
        assert_eq!(run.phase, Phase::Completed);
    }
    #[test]
    fn unknown_target_cannot_have_an_alignment_trajectory() {
        let (_, mut run) = fixture();
        run.difficulty = 3;
        run.frames[0].stage = 5;
        assert!(run.validate().is_err());
        run.phase = Phase::AwaitingTarget;
        assert!(run.validate().is_err());
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
    fn breakaway_seats_once_returns_with_limits_and_restores_mid_motion() {
        use crate::energy_unit::Attachment;
        use crate::projectile::PayloadShape;
        let (mut field, mut run) = fixture();
        field.step(256).unwrap();
        run.pose.engineer_body = field.chassis_snapshot(run.owner).unwrap().pose;
        let mut arm = ArmState::neutral(run.owner, run.model.clone());
        arm.joints_rad = run.pose.engineer_q;
        arm.target_rad = arm.joints_rad;
        field.add_arm(arm).unwrap();
        field
            .preload_energy_unit(
                run.owner,
                PayloadShape {
                    hulls_m: vec![vec![
                        [0., 0., 0.],
                        [0.01, 0., 0.],
                        [0., 0.01, 0.],
                        [0., 0., 0.01],
                    ]],
                    mass_kg: 0.25,
                    max_expansion_m: 0.,
                },
            )
            .unwrap();
        run.pose.jaw_angles = field.arms()[0].jaws_rad;
        for f in &mut run.frames {
            f.jaws_rad = run.pose.jaw_angles;
        }
        // First finish the accepted fixture, then open stationary jaws before
        // following a bounded return. The seated object must never follow it.
        let last = run.frames.last().unwrap().clone();
        for i in 0..160 {
            let mut f = last.clone();
            f.stage = 7;
            f.gripped = false;
            let t = (i as f64 / 159.).max(0.);
            f.joints_rad = NEUTRAL_JOINTS_RAD.map(|q| q * t);
            f.jaws_rad = run.pose.jaw_angles.map(|q| q * (1. - t));
            run.frames.push(f);
        }
        field.set_assembly(run).unwrap();
        field.step(4).unwrap();
        assert_eq!(field.assemblies()[0].phase, Phase::BreakingAway);
        let seated = field.energy_units()[0].clone();
        assert_eq!(seated.attachment, Attachment::Seated(field.arms()[0].owner));
        field.step(40).unwrap();
        let mut restored =
            Field::restore(&field.snapshot(), &StaticGeometry::default(), 0.).unwrap();
        for _ in 0..120 {
            let before = field.arms()[0].joints_rad;
            field.step(1).unwrap();
            restored.step(1).unwrap();
            assert!(
                (0..6).all(|i| (field.arms()[0].joints_rad[i] - before[i]).abs()
                    <= JOINT_SPEED_RAD_S[i] * tick_ns() as f64 * 1e-9 + 2e-6)
            );
            assert_eq!(field.energy_units()[0].physical.pose, seated.physical.pose);
        }
        assert_eq!(field.assemblies()[0].phase, Phase::Completed);
        assert_eq!(field.arms()[0].joints_rad, NEUTRAL_JOINTS_RAD);
        assert_eq!(field.arms(), restored.arms());
        assert_eq!(field.energy_units(), restored.energy_units());
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
