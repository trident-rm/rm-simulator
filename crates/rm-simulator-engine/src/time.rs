// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Exact caller-paced integration time, independent of any solver or host clock.
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
/// use rm_simulator_engine::StepDuration;
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
