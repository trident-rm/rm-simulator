// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Deterministic physical RoboMaster field building blocks, without match rules,
//! networking, CAD loading, Bevy or host time. Callers supply target endpoint
//! poses and resolved mechanism state, step once per explicit `tick_ns` tick, then
//! consume contacts in returned order. See the `moving_armor` example.
//!
//! ```
//! use rm_simulator_physics::{
//!     ArmorTarget, Caliber, Pose, Shot, TargetFace, TargetFrames, WorldPhysics, tick_ns,
//! };
//!
//! let face = TargetFace {
//!     target: ArmorTarget::Outpost { outpost: 0, face: 0 },
//!     // +x is the outward normal, so a half turn faces the shooter at the origin.
//!     pose: Pose::yawed([1.5, 0.0, 1.0], std::f64::consts::PI),
//! };
//! let mut physics = WorldPhysics::new(&[face], 0.0);
//! let frames = TargetFrames::new(vec![face]);
//! physics.fire(0, Pose::at([0.0, 0.0, 1.0]), Shot::at_limit(Caliber::Mm17), None)?;
//!
//! let mut contacts = 0;
//! for tick in 0..200 {
//!     contacts += physics.step(tick * tick_ns(), &frames)?.len();
//! }
//! // The 1.5 m flight takes about 60 ticks; each ball touches the housing once.
//! assert_eq!(contacts, 1);
//! # Ok::<(), &'static str>(())
//! ```
#![deny(missing_docs)]
pub mod arena;
pub mod assembly_model;
pub mod chassis;
pub mod control;
pub mod geometry;
pub mod manipulator;
pub mod manipulator_collision;
pub mod manipulator_plan;
pub mod mechanism;
pub mod motion;
pub mod projectile;
pub mod queries;
pub mod sensors;
pub mod session;
pub mod wheel_model;
pub use projectile::{
    ArmorTarget, Caliber, Contact, Shot, StaticGeometry, TargetFace, TargetFrames, WorldPhysics,
};
use serde::{Deserialize, Serialize};

/// Default simulation step, in exact nanoseconds (128 Hz).
pub const DEFAULT_TICK_NS: u64 = 7_812_500;

/// Default step in nanoseconds. Instance customization uses [`StepDuration`].
pub const fn tick_ns() -> u64 {
    DEFAULT_TICK_NS
}

/// Validated simulation step in exact integer nanoseconds.
///
/// Positive steps no coarser than the default 128 Hz step are accepted.
/// Immutable per world; no host clock is read. Mechanical calibration is separate
/// from clock precision; a higher step rate does not add missing physical effects.
/// ```
/// use rm_simulator_physics::StepDuration;
/// assert_eq!(StepDuration::from_ns(1_000_000).unwrap(), StepDuration::HZ_1000);
/// assert!(StepDuration::from_ns(0).is_err());
/// assert_eq!(StepDuration::default().as_ns(), 7_812_500);
/// assert_eq!(StepDuration::from_hz(500).unwrap().as_ns(), 2_000_000);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(try_from = "u64", into = "u64")]
pub struct StepDuration(u64);
impl StepDuration {
    /// Default 128 Hz step: exactly 7.8125 milliseconds.
    pub const HZ_128: Self = Self(DEFAULT_TICK_NS);
    /// 1000 Hz step: exactly one millisecond.
    pub const HZ_1000: Self = Self(1_000_000);
    /// Validate a positive exact nanosecond duration no coarser than 128 Hz.
    /// Larger steps are rejected to preserve the existing integration ceiling.
    pub const fn from_ns(duration_ns: u64) -> Result<Self, &'static str> {
        match duration_ns {
            1..=DEFAULT_TICK_NS => Ok(Self(duration_ns)),
            _ => Err("simulation step must be within 1..=7_812_500 nanoseconds"),
        }
    }
    /// Select a frequency in hertz with an exact integer-nanosecond period.
    /// Zero, rates below 128 Hz and fractional-nanosecond periods are rejected;
    /// use [`Self::from_ns`] when specifying an exact duration directly.
    pub const fn from_hz(frequency_hz: u64) -> Result<Self, &'static str> {
        if frequency_hz == 0 || !1_000_000_000_u64.is_multiple_of(frequency_hz) {
            return Err("frequency must have an exact integer-nanosecond period");
        }
        Self::from_ns(1_000_000_000 / frequency_hz)
    }
    /// Exact duration in nanoseconds, used for authoritative clocks.
    pub const fn as_ns(self) -> u64 {
        self.0
    }
    /// Integration duration in seconds; never used to accumulate clock time.
    pub fn as_secs_f64(self) -> f64 {
        self.0 as f64 * 1e-9
    }
}
impl Default for StepDuration {
    fn default() -> Self {
        Self::HZ_128
    }
}
impl TryFrom<u64> for StepDuration {
    type Error = &'static str;
    fn try_from(value: u64) -> Result<Self, Self::Error> {
        Self::from_ns(value)
    }
}
impl From<StepDuration> for u64 {
    fn from(value: StepDuration) -> Self {
        value.as_ns()
    }
}

#[cfg(test)]
mod step_duration_tests {
    use super::*;

    #[test]
    fn rates_are_exact_and_validation_bounds_the_solver_step() {
        assert_eq!(StepDuration::from_hz(128), Ok(StepDuration::HZ_128));
        assert_eq!(StepDuration::from_hz(1000), Ok(StepDuration::HZ_1000));
        assert_eq!(StepDuration::from_hz(2000).unwrap().as_ns(), 500_000);
        assert_eq!(StepDuration::from_ns(1_234_567).unwrap().as_ns(), 1_234_567);
        for invalid in [0, 64, 127, 333, u64::MAX] {
            assert!(StepDuration::from_hz(invalid).is_err());
        }
        for invalid in [0, DEFAULT_TICK_NS + 1, u64::MAX] {
            assert!(StepDuration::from_ns(invalid).is_err());
        }
    }
}

/// A rigid pose in forward/left/up coordinates, metres and a wxyz quaternion.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Pose {
    /// Translation in metres, forward/left/up.
    pub translation_m: [f64; 3],
    /// Rotation as a unit wxyz quaternion.
    pub rotation_wxyz: [f64; 4],
}
impl Default for Pose {
    fn default() -> Self {
        Self {
            translation_m: [0.0; 3],
            rotation_wxyz: [1.0, 0.0, 0.0, 0.0],
        }
    }
}
impl Pose {
    /// An unrotated pose at `translation_m`.
    ///
    /// ```
    /// use rm_simulator_physics::{chassis::yaw_of, Pose};
    ///
    /// let mut pose = Pose::at([1.0, -2.0, 0.5]);
    /// assert_eq!(pose.rotation_wxyz, [1.0, 0.0, 0.0, 0.0]);
    /// assert_eq!(yaw_of(pose), 0.0);
    ///
    /// // Yaw turns about world up, counter-clockwise from +x.
    /// pose = Pose::yawed(pose.translation_m, std::f64::consts::FRAC_PI_2);
    /// assert!((yaw_of(pose) - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    /// ```
    pub fn at(translation_m: [f64; 3]) -> Self {
        Self {
            translation_m,
            ..Self::default()
        }
    }
    /// A pose rotated about world up (yaw) only.
    ///
    /// ```
    /// use rm_simulator_physics::{motion::outpost::rotate, Pose};
    ///
    /// // Yaw is counter-clockwise about world up, so a quarter turn sends
    /// // forward (+x) to left (+y).
    /// let pose = Pose::yawed([0.0; 3], std::f64::consts::FRAC_PI_2);
    /// let forward = rotate(pose.rotation_wxyz, [1.0, 0.0, 0.0]);
    /// assert!(forward[0].abs() < 1e-12);
    /// assert!((forward[1] - 1.0).abs() < 1e-12);
    /// ```
    pub fn yawed(translation_m: [f64; 3], yaw_rad: f64) -> Self {
        let (s, c) = (yaw_rad / 2.0).sin_cos();
        Self {
            translation_m,
            rotation_wxyz: [c, 0.0, 0.0, s],
        }
    }
}

/// One of the two alliances. [`Team::index`] is the slot a team fills in the
/// fixed `[_; 2]` arrays that mechanism and geometry state use.
///
/// ```
/// use rm_simulator_physics::Team;
///
/// assert_eq!(Team::Red.index(), 0);
/// assert_eq!(Team::Blue.index(), 1);
/// assert_eq!(Team::Red.other(), Team::Blue);
/// assert_eq!(Team::BOTH.map(Team::name), ["red", "blue"]);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Team {
    /// The red alliance.
    Red,
    /// The blue alliance.
    Blue,
}
impl Team {
    /// Both teams, red then blue.
    pub const BOTH: [Team; 2] = [Team::Red, Team::Blue];
    /// Array slot for this team: red 0, blue 1.
    pub fn index(self) -> usize {
        match self {
            Team::Red => 0,
            Team::Blue => 1,
        }
    }
    /// The opposing team.
    pub fn other(self) -> Team {
        match self {
            Team::Red => Team::Blue,
            Team::Blue => Team::Red,
        }
    }
    /// Lower-case name, `"red"` or `"blue"`.
    pub fn name(self) -> &'static str {
        match self {
            Team::Red => "red",
            Team::Blue => "blue",
        }
    }
}
