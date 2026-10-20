// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared Rune presentation records and pure physical pose reconstruction.
use crate::Pose;
use serde::{Deserialize, Serialize};
/// Number of physical target blades.
pub const BLADE_COUNT: usize = 5;
/// Effective scoring disk radius, metres.
pub const EFFECTIVE_TARGET_RADIUS_M: f64 = 0.150;
/// Reference target orbit, metres.
pub const TARGET_RADIUS_M: f64 = 0.7;
/// Fitted orbit of historical source artwork, metres.
pub const CAD_TARGET_RADIUS_M: f64 = 0.6985;
/// Rune activation state. A referee-controlled rune rests Inactive until a
/// team spends an opportunity, then Activating, and holds Activated until the
/// referee takes the buff away.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RuneState {
    /// Dark and rotating; hits are ignored (section 5.5.2.1 "Inactive").
    Inactive,
    /// Targets are lit in turn and the rune accepts hits.
    Activating,
    /// Every blade is lit and the buff is held; further hits are ignored.
    Activated,
}

/// One rune frame at an authoritative tick: the held angle, which blades are
/// lit, how far activation got and where the state machine stands.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RuneSnapshot {
    /// Analytic activation motion and its simulation-time epoch, when sinusoidal.
    pub motion: Option<(BigRuneMotion, u64)>,
    /// Which mode the rune is in.
    pub kind: RuneKind,
    /// Which of the five blades are lit as targets right now.
    pub active_blades: [bool; BLADE_COUNT],
    /// Activation progress: blades hit on a Small Rune, groups completed on a Big Rune.
    pub completed_groups: u32,
    /// Angular speed at this instant; a Big Rune's varies during activation.
    pub angular_speed_rad_s: f64,
    /// Field time this frame was taken at.
    pub time_ns: u64,
    /// Face centre pose; local +x points into the wheel, away from the shooter.
    pub hub_pose: Pose,
    /// Orbit radius of the five targets, 0.7 m nominally.
    pub target_radius_m: f64,
    /// Visual counterclockwise phase, wrapped to [0, 2pi), continuous through resets.
    pub angle_rad: f64,
    /// Whether the rune is Inactive, Activating or Activated.
    pub state: RuneState,
    /// The lit blade a Small or Big Rune demands first, when one is lit.
    pub active_blade: Option<u32>,
    /// Blades already hit in the current attempt.
    pub activated: [bool; BLADE_COUNT],
    /// The five target face poses in blade order, +x the outward scoring normal.
    pub target_poses: [Pose; BLADE_COUNT],
    /// World time at which the current state began.
    pub state_since_ns: u64,
}

impl RuneSnapshot {
    /// Reconstruct motion only; unknown hits and state transitions remain authoritative.
    pub fn presentation_angle_rad(&self, time_ns: u64) -> f64 {
        let time_ns = time_ns.max(self.time_ns);
        let displacement = self.motion.map_or_else(
            || self.angular_speed_rad_s * (time_ns - self.time_ns) as f64 * 1e-9,
            |(motion, epoch)| {
                motion.displacement_rad(time_ns.saturating_sub(epoch) as f64 * 1e-9)
                    - motion.displacement_rad(self.time_ns.saturating_sub(epoch) as f64 * 1e-9)
            },
        );
        (self.angle_rad + displacement).rem_euclid(std::f64::consts::TAU)
    }
}

/// What a rune did with a hit, including the hits it refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum HitOutcome {
    /// A Big Rune target was hit: the first of the pair, or the optional bonus
    /// when `bonus` is set and it landed within one second of the first.
    GroupHit {
        /// Blade that was struck.
        blade: u32,
        /// Groups completed before this hit.
        group: u32,
        /// The pair's first target was already hit within the last second.
        bonus: bool,
    },
    /// A Small Rune target was hit; `next_blade` is now lit.
    Accepted {
        /// Blade that was struck.
        blade: u32,
        /// Blade the rune now demands.
        next_blade: u32,
    },
    /// The hit completed the rune, which now holds Activated.
    Activated {
        /// Blade that completed the sequence.
        blade: u32,
    },
    /// Resets progress, as required for hitting a non-illuminated armor module.
    WrongBlade {
        /// Blade that was struck.
        blade: u32,
        /// Blade that was lit instead.
        expected: u32,
    },
    /// No active target: the rune is Inactive or already Activated.
    Inactive,
}

/// Reasons a rune call is refused. A refused call leaves the rune unchanged.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum RuneError {
    /// The hub pose is not finite, its quaternion is zero, or a decoded rune
    /// carries a non-finite radius or angle offset.
    #[error("rune hub pose must be finite with a nonzero quaternion")]
    Pose,
    /// The requested time is before the rune's current time.
    #[error("rune time cannot move backwards")]
    TimeReversal,
    /// The blade index is not below [`BLADE_COUNT`].
    #[error("rune blade index must be in 0..5")]
    Blade,
    /// Big Rune amplitude or frequency is outside the 2026 rule bounds, or a
    /// decoded Big Rune has a non-finite epoch.
    #[error("big rune amplitude or frequency is outside the 2026 rule bounds")]
    Motion,
}

pub use rm_simulator_physics::motion::rune::{scoring_pose, target_poses};

/// Which Power Rune mode a rune runs. The referee converts every rune to Big
/// at the three-minute stage change (section 5.5.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RuneKind {
    /// Constant π/3 rad/s, one lit target at a time.
    Small,
    /// Sinusoidal speed during activation, five pairs of lit targets.
    Big,
}

/// Caller-selected, repeatable parameters within the 2026 rule bounds.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct BigRuneMotion {
    amplitude_rad_s: f64,
    frequency_rad_s: f64,
}
impl Default for BigRuneMotion {
    fn default() -> Self {
        Self {
            amplitude_rad_s: 0.9,
            frequency_rad_s: 1.94,
        }
    }
}
impl BigRuneMotion {
    /// Checked sinusoid parameters: `amplitude_rad_s` in 0.780..=1.045 and
    /// `frequency_rad_s` in 1.884..=2.000 (section 5.5.2.1). Anything else is
    /// refused with [`RuneError::Motion`].
    ///
    /// ```rust
    /// use rm_simulator_world::rune::BigRuneMotion;
    ///
    /// let motion = BigRuneMotion::new(0.9, 1.94).unwrap();
    /// // At the start of an activation the sinusoid contributes nothing, so
    /// // the speed is 2.090 - a.
    /// assert!((motion.speed_rad_s(0.0) - (2.090 - 0.9)).abs() < 1e-12);
    /// assert!(BigRuneMotion::new(1.2, 1.94).is_err());
    /// ```
    pub fn new(amplitude_rad_s: f64, frequency_rad_s: f64) -> Result<Self, RuneError> {
        if !(0.780..=1.045).contains(&amplitude_rad_s)
            || !(1.884..=2.000).contains(&frequency_rad_s)
        {
            return Err(RuneError::Motion);
        }
        Ok(Self {
            amplitude_rad_s,
            frequency_rad_s,
        })
    }
    /// Angular speed `elapsed_s` into an activation, in rad/s:
    /// `a*sin(omega*t) + 2.090 - a`.
    pub fn speed_rad_s(self, elapsed_s: f64) -> f64 {
        self.profile().speed_rad_s(elapsed_s)
    }
    /// Angle turned `elapsed_s` into an activation, in radians; the integral
    /// of [`BigRuneMotion::speed_rad_s`].
    pub fn displacement_rad(self, elapsed_s: f64) -> f64 {
        self.profile().displacement_rad(elapsed_s)
    }
    fn profile(self) -> rm_simulator_physics::motion::SinusoidalMotion {
        rm_simulator_physics::motion::SinusoidalMotion {
            amplitude_rad_s: self.amplitude_rad_s,
            frequency_rad_s: self.frequency_rad_s,
        }
    }
}

/// Shared presentation type; the native rule authority is `rmuc_2027::rune::Rune`.
pub type Rune = RuneSnapshot;
