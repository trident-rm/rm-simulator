// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! UC §5.4.3, Figures 5-12–5-15: measured assembly sequencing.
//!
//! This component consumes authoritative mechanism sensors, not pilot claims.
//! Its coordinates are mechanism travel, not the manual's spherical target pose
//! angles. A physics adapter must resolve the socket, P/Q bearings and gripper
//! before reporting them. It awards no match rewards and grants no Heavy admission.
use serde::{Deserialize, Serialize};
use std::f64::consts::FRAC_PI_2;
#[cfg(test)]
mod tests;

/// Required insertion and upward travel, metres (UC §5.4.3 steps 2/3).
pub const TRAVEL_M: f64 = 0.1;
/// Q alignment tolerance in radians, inclusive (UC Table 5-10).
pub const ANGLE_RAD: f64 = 5. * std::f64::consts::PI / 180.;
/// Simulator travel tolerance, metres; not specified by the manual.
pub const POSITION_M: f64 = 0.003;
/// Simulator dwell within the manual's 0.3–0.5 s indication range.
pub const ALIGN_NS: u64 = 400_000_000;
/// Rejected confirmation lockout (UC §5.4.3 step 5), ns.
pub const RETRY_NS: u64 = 1_000_000_000;
/// Maximum separation between paired level-four steps (UC §5.4.3), ns.
pub const PAIR_NS: u64 = 5_000_000_000;
/// Continuous assembly RFID loss before failure (UC §5.4.3), ns.
pub const CONTACT_NS: u64 = 15_000_000_000;

/// Current observed mechanism travel. A detached tool cannot advance a sequence.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Sensors {
    /// Energy unit is still seated/coupled to the gripper or socket.
    pub attached: bool,
    /// Penetration along local −x, from 0 to 0.1 m.
    pub insertion_m: f64,
    /// Mechanism travel along assembly +z after insertion, metres.
    pub lift_m: f64,
    /// Upward P-bearing rotation after lift, radians.
    pub p_rad: f64,
    /// Signed Q-bearing rotation after the P step, radians.
    pub q_rad: f64,
    /// Socket/energy-unit axis mismatch, radians; nonnegative.
    pub axis_error_rad: f64,
    /// Perpendicular error from the current mechanism guide, metres; nonnegative.
    pub lateral_error_m: f64,
}
impl Sensors {
    fn valid(self) -> bool {
        [
            self.insertion_m,
            self.lift_m,
            self.p_rad,
            self.q_rad,
            self.axis_error_rad,
            self.lateral_error_m,
        ]
        .into_iter()
        .all(f64::is_finite)
            && self.axis_error_rad >= 0.
            && self.lateral_error_m >= 0.
    }
    fn seated(self) -> bool {
        self.attached
            && (self.insertion_m - TRAVEL_M).abs() <= POSITION_M
            && self.lateral_error_m <= POSITION_M
    }
    fn level(self) -> bool {
        self.p_rad.abs() <= ANGLE_RAD && self.q_rad.abs() <= ANGLE_RAD
    }
    fn lifted(self) -> bool {
        (self.lift_m - TRAVEL_M).abs() <= POSITION_M
    }
    fn raised(self) -> bool {
        (self.p_rad - FRAC_PI_2).abs() <= ANGLE_RAD
    }
}
/// A core's current required operator action.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Step {
    /// Approach the entrance at zero insertion; prevents pre-insertion shortcuts.
    #[default]
    Approach,
    /// Insert the unit 100 mm along the local −x guide.
    Insert,
    /// Lift the seated unit 100 mm along +z.
    Lift,
    /// Rotate upward 90 degrees about P.
    Raise,
    /// Hold the requested Q angle continuously for the indication dwell.
    Align,
    /// Currently aligned; a separate confirmation is still required.
    Ready,
    /// This core has accepted confirmation; no further motion is permitted.
    Confirmed,
}
/// Failure requiring a fresh workbench attempt.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Failure {
    /// Level-four exclusive 45 s deadline reached.
    Timeout,
    /// A paired insertion, lift, alignment or confirmation exceeded five seconds.
    PairTimeout,
    /// Continuous assembly-zone absence reached fifteen seconds.
    ContactLost,
    /// Attached Heavy died; recovery/disengagement is a separate physical task.
    Defeated,
    /// A seated unit was removed, or a confirmed unit was manipulated.
    Removed,
}
/// State of one core, including milestone timestamps and present sensor evidence.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Core {
    /// Current required action.
    pub step: Step,
    /// Last authoritative mechanism observation.
    pub sensors: Sensors,
    /// First insertion, lift, alignment and confirmation times in ns.
    pub milestones_ns: [Option<u64>; 4],
    /// Start of the current uninterrupted Q alignment interval.
    pub aligned_since_ns: Option<u64>,
    /// Exclusive confirmation lockout end, ns.
    pub retry_ns: u64,
}
/// Entire deterministic workbench authority. Serialize alongside physical state.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Attempt {
    /// Difficulty 1–4, selected at creation.
    pub difficulty: u8,
    /// Required Q angles for the two cores, radians, each within ±90 degrees.
    pub target_q_rad: [f64; 2],
    /// Core state; only slot zero participates below difficulty four.
    pub cores: [Core; 2],
    /// Explicit simulation timestamp of the latest observation, ns.
    pub now_ns: u64,
    /// Creation timestamp, ns. Level four expires after 45 s.
    pub started_ns: u64,
    /// Start of uninterrupted RFID absence; None means currently detected.
    pub absent_since_ns: Option<u64>,
    /// Latched failure; terminal until a new attempt is created.
    pub failure: Option<Failure>,
}
impl Attempt {
    /// Create an already-positioned workbench. Difficulty/time unlocks and shared
    /// core reservation belong to the match, not this mechanism evaluator.
    ///
    /// ```
    /// use rm_simulator_gameplay::rmuc_2027::manipulation::{Attempt, Sensors};
    /// let mut task = Attempt::new(1, [0.; 2], 0).unwrap();
    /// let mut sensors = [Sensors { attached: true, ..Default::default() }; 2];
    /// task.observe(0, sensors, true, true).unwrap(); // enter the socket
    /// sensors[0].insertion_m = 0.1;
    /// task.observe(1, sensors, true, true).unwrap();
    /// task.confirm(0).unwrap();
    /// assert!(task.completed());
    /// ```
    pub fn new(difficulty: u8, target_q_rad: [f64; 2], now_ns: u64) -> Result<Self, &'static str> {
        if !(1..=4).contains(&difficulty)
            || target_q_rad
                .iter()
                .any(|q| !q.is_finite() || q.abs() > FRAC_PI_2)
        {
            return Err("invalid assembly difficulty or Q target");
        }
        Ok(Self {
            difficulty,
            target_q_rad,
            cores: Default::default(),
            now_ns,
            started_ns: now_ns,
            absent_since_ns: None,
            failure: None,
        })
    }
    /// Number of cores participating in this attempt.
    pub fn core_count(&self) -> usize {
        if self.difficulty == 4 { 2 } else { 1 }
    }
    /// Both required confirmations succeeded and no failure occurred.
    pub fn completed(&self) -> bool {
        self.failure.is_none()
            && self.cores[..self.core_count()]
                .iter()
                .all(|c| c.step == Step::Confirmed)
    }
    /// Consume simultaneous held sensor readings at a monotonic explicit time.
    /// Timers settle before new evidence, so a late RFID renewal cannot erase
    /// fifteen seconds of absence. Invalid input leaves all authority unchanged.
    /// P and Q are mechanism encoder angles, not world Euler angles.
    pub fn observe(
        &mut self,
        now_ns: u64,
        sensors: [Sensors; 2],
        zone: bool,
        alive: bool,
    ) -> Result<(), &'static str> {
        if now_ns < self.now_ns || sensors.iter().any(|s| !s.valid()) {
            return Err("nonmonotonic assembly time or invalid sensor values");
        }
        let was_completed = self.completed();
        self.now_ns = now_ns;
        if was_completed || self.failure.is_some() {
            return Ok(());
        }
        self.settle();
        if self.failure.is_some() {
            return Ok(());
        }
        if zone {
            self.absent_since_ns = None;
        } else {
            self.absent_since_ns.get_or_insert(now_ns);
        }
        if !alive {
            self.failure = Some(Failure::Defeated);
            return Ok(());
        }
        for (i, s) in sensors.into_iter().enumerate().take(self.core_count()) {
            let c = &mut self.cores[i];
            if c.milestones_ns[0].is_some() && !s.seated() {
                self.failure = Some(Failure::Removed);
                break;
            }
            if c.step == Step::Confirmed && s != c.sensors {
                // Ignore sub-tolerance numerical drift while preserving the confirmed pose.
                if (s.lift_m - c.sensors.lift_m).abs() > POSITION_M
                    || (s.p_rad - c.sensors.p_rad).abs() > ANGLE_RAD
                    || (s.q_rad - c.sensors.q_rad).abs() > ANGLE_RAD
                {
                    self.failure = Some(Failure::Removed);
                    break;
                }
                continue;
            }
            c.sensors = s;
            // One stage per observation: skipping prerequisite poses never certifies them.
            match c.step {
                Step::Approach
                    if s.attached
                        && s.insertion_m.abs() <= POSITION_M
                        && s.lift_m.abs() <= POSITION_M
                        && s.level()
                        && s.axis_error_rad <= ANGLE_RAD
                        && s.lateral_error_m <= POSITION_M =>
                {
                    c.step = Step::Insert;
                }
                Step::Insert
                    if s.seated()
                        && s.lift_m.abs() <= POSITION_M
                        && s.level()
                        && s.axis_error_rad <= ANGLE_RAD =>
                {
                    c.milestones_ns[0] = Some(now_ns);
                    c.step = if self.difficulty == 1 {
                        Step::Ready
                    } else {
                        Step::Lift
                    };
                }
                Step::Lift if s.lifted() && s.level() && s.axis_error_rad <= ANGLE_RAD => {
                    c.milestones_ns[1].get_or_insert(now_ns);
                    c.step = if self.difficulty == 2 {
                        Step::Ready
                    } else {
                        Step::Raise
                    };
                }
                Step::Raise
                    if s.lifted()
                        && s.raised()
                        && s.q_rad.abs() <= ANGLE_RAD
                        && s.axis_error_rad <= ANGLE_RAD =>
                {
                    c.step = Step::Align;
                }
                Step::Align | Step::Ready => {
                    let aligned = s.axis_error_rad <= ANGLE_RAD
                        && match self.difficulty {
                            1 => {
                                s.lift_m.abs() <= POSITION_M
                                    && s.level()
                                    && s.axis_error_rad <= ANGLE_RAD
                            }
                            2 => s.lifted() && s.level(),
                            _ => {
                                s.lifted()
                                    && s.raised()
                                    && (s.q_rad - self.target_q_rad[i]).abs() <= ANGLE_RAD
                            }
                        };
                    if aligned {
                        let since = *c.aligned_since_ns.get_or_insert(now_ns);
                        if self.difficulty <= 2 || now_ns - since >= ALIGN_NS {
                            c.step = Step::Ready;
                            if self.difficulty >= 3 {
                                c.milestones_ns[2].get_or_insert(now_ns);
                            }
                        }
                    } else {
                        c.aligned_since_ns = None;
                        c.step = match self.difficulty {
                            1 => Step::Ready,
                            2 => Step::Lift,
                            _ => Step::Align,
                        };
                    }
                }
                _ => {}
            }
        }
        self.settle();
        Ok(())
    }
    /// Confirm one currently correct core. Incorrect confirmation imposes a
    /// one-second lockout. Repeated confirmation never awards anything twice.
    pub fn confirm(&mut self, index: usize) -> Result<(), &'static str> {
        if index >= self.core_count() {
            return Err("assembly core index out of range");
        }
        self.settle();
        if self.failure.is_some() {
            return Err("assembly attempt has failed");
        }
        let c = &mut self.cores[index];
        if self.now_ns < c.retry_ns {
            return Err("assembly confirmation is locked");
        }
        if c.step == Step::Confirmed {
            return Err("assembly core is already confirmed");
        }
        let s = c.sensors;
        let correct = s.axis_error_rad <= ANGLE_RAD
            && c.step == Step::Ready
            && s.seated()
            && match self.difficulty {
                1 => s.lift_m.abs() <= POSITION_M && s.level() && s.axis_error_rad <= ANGLE_RAD,
                2 => s.lifted() && s.level(),
                _ => {
                    s.lifted()
                        && s.raised()
                        && (s.q_rad - self.target_q_rad[index]).abs() <= ANGLE_RAD
                }
            };
        if !correct {
            c.retry_ns = self.now_ns.saturating_add(RETRY_NS);
            return Err("energy unit has not completed the required sequence");
        }
        c.step = Step::Confirmed;
        c.milestones_ns[3] = Some(self.now_ns);
        self.settle();
        Ok(())
    }
    fn settle(&mut self) {
        if self.failure.is_some() || self.completed() {
            return;
        }
        if self.difficulty == 4 && self.now_ns - self.started_ns >= super::assembly::ATTEMPT_NS {
            self.failure = Some(Failure::Timeout);
        } else if self
            .absent_since_ns
            .is_some_and(|t| self.now_ns - t >= CONTACT_NS)
        {
            self.failure = Some(Failure::ContactLost);
        } else if self.difficulty == 4 {
            for i in 0..4 {
                let a = self.cores[0].milestones_ns[i];
                let b = self.cores[1].milestones_ns[i];
                let late = match (a, b) {
                    (Some(a), Some(b)) => a.abs_diff(b) > PAIR_NS,
                    (Some(t), None) | (None, Some(t)) => self.now_ns - t > PAIR_NS,
                    _ => false,
                };
                if late {
                    self.failure = Some(Failure::PairTimeout);
                    break;
                }
            }
        }
    }
}
