// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Force-limited withdrawal from an axial retaining fixture.
//!
//! This reduced model constrains an object to a straight guide until clearance.
//! The guide reacts transverse loads; adhesion opposes outward travel. It is
//! not a magnetic field model or a dynamic articulated-arm solver.
//!
//! ```
//! use sim_engine::retention::{PullOffLimits, PullOffOutcome, PullOffState};
//! let limits = PullOffLimits { mass_kg: 0.4, retention_n: 10., clearance_m: 0.02,
//!     stiffness_n_m: 500., damping_n_s_m: 10., arm_force_n: 5., grip_force_n: 25. };
//! let mut actual = PullOffState::default();
//! let result = actual.advance(limits, [0.1, 0., 0.], [1., 0., 0.], [0., 0., -9.81], 0.001)?;
//! assert_eq!(result, PullOffOutcome::Retained);
//! assert_eq!(actual.travel_m, 0.); // A 5 N arm cannot overcome 10 N retention.
//! # Ok::<(), sim_engine::retention::RetentionError>(())
//! ```
use glam::DVec3;
use serde::{Deserialize, Serialize};

/// Invalid or nonfinite retention parameters or observations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RetentionError;
impl std::fmt::Display for RetentionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("invalid axial retention model or observation")
    }
}
impl std::error::Error for RetentionError {}

/// Caller-selected mechanical parameters. No defaults imply measured hardware.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PullOffLimits {
    /// Object mass in kilograms, strictly positive.
    pub mass_kg: f64,
    /// Constant inward adhesion during withdrawal, in newtons.
    pub retention_n: f64,
    /// Travel needed to leave the retaining guide, in metres, strictly positive.
    pub clearance_m: f64,
    /// Translational grasp stiffness, in newtons per metre, strictly positive.
    pub stiffness_n_m: f64,
    /// Translational grasp damping, in newton seconds per metre.
    pub damping_n_s_m: f64,
    /// Maximum resultant force the arm can transmit, in newtons.
    pub arm_force_n: f64,
    /// Resultant force above which the grip slips, in newtons.
    pub grip_force_n: f64,
}
impl PullOffLimits {
    /// Reject nonfinite, negative or degenerate mechanical parameters.
    ///
    /// # Errors
    /// Returns [`RetentionError`] for invalid limits.
    pub fn validate(self) -> Result<(), RetentionError> {
        let values = [
            self.mass_kg,
            self.retention_n,
            self.clearance_m,
            self.stiffness_n_m,
            self.damping_n_s_m,
            self.arm_force_n,
            self.grip_force_n,
        ];
        if values.into_iter().any(|v| !v.is_finite() || v < 0.)
            || self.mass_kg == 0.
            || self.clearance_m == 0.
            || self.stiffness_n_m == 0.
        {
            return Err(RetentionError);
        }
        Ok(())
    }
}

/// Restorable axial position and velocity while retained.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PullOffState {
    /// Outward displacement from the mounted pose, in metres.
    pub travel_m: f64,
    /// Outward speed, in metres per second.
    pub velocity_m_s: f64,
}
/// Result of a force integration interval.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PullOffOutcome {
    /// Still on the guide; the pull may be stalled or making progress.
    Retained,
    /// Reached the guide clearance with the grip intact.
    Extracted,
    /// Transmitted force exceeded grip capacity; no further grip force is applied.
    Slipped,
}
impl PullOffState {
    /// Integrate a held pull for at most one millisecond. The target is the
    /// desired object-origin displacement from its mount, in world metres.
    /// `axis` is a world unit vector pointing outward; gravity is world m/s².
    /// The arm saturates before testing grip capacity. Equality does not slip.
    /// Failed validation leaves this state unchanged. The caller drops the grip
    /// on `Slipped` and releases the axial constraint on `Extracted`.
    ///
    /// # Errors
    /// Returns [`RetentionError`] for invalid limits, state, time or observations.
    pub fn advance(
        &mut self,
        limits: PullOffLimits,
        target_m: [f64; 3],
        axis: [f64; 3],
        gravity_m_s2: [f64; 3],
        dt_s: f64,
    ) -> Result<PullOffOutcome, RetentionError> {
        limits.validate()?;
        let axis = DVec3::from_array(axis);
        let target = DVec3::from_array(target_m);
        let gravity = DVec3::from_array(gravity_m_s2);
        if !axis.is_finite()
            || (axis.length_squared() - 1.).abs() > 1e-6
            || !target.is_finite()
            || !gravity.is_finite()
            || !dt_s.is_finite()
            || !(0. ..=0.001).contains(&dt_s)
            || dt_s == 0.
            || !self.travel_m.is_finite()
            || !(0. ..=limits.clearance_m).contains(&self.travel_m)
            || !self.velocity_m_s.is_finite()
        {
            return Err(RetentionError);
        }
        let desired = (target - axis * self.travel_m) * limits.stiffness_n_m
            - axis * self.velocity_m_s * limits.damping_n_s_m;
        let applied = desired.clamp_length_max(limits.arm_force_n);
        if applied.length() > limits.grip_force_n {
            return Ok(PullOffOutcome::Slipped);
        }
        let acceleration =
            (applied.dot(axis) - limits.retention_n) / limits.mass_kg + gravity.dot(axis);
        let velocity = self.velocity_m_s + acceleration * dt_s;
        let travel = self.travel_m + velocity * dt_s;
        if !velocity.is_finite() || !travel.is_finite() {
            return Err(RetentionError);
        }
        self.travel_m = travel.clamp(0., limits.clearance_m);
        self.velocity_m_s = if travel <= 0. { 0. } else { velocity };
        Ok(if travel >= limits.clearance_m {
            PullOffOutcome::Extracted
        } else {
            PullOffOutcome::Retained
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn limits() -> PullOffLimits {
        PullOffLimits {
            mass_kg: 0.4,
            retention_n: 10.,
            clearance_m: 0.02,
            stiffness_n_m: 500.,
            damping_n_s_m: 10.,
            arm_force_n: 20.,
            grip_force_n: 25.,
        }
    }
    fn pull(
        limits: PullOffLimits,
        axis: [f64; 3],
        target: [f64; 3],
    ) -> (PullOffState, PullOffOutcome) {
        let mut state = PullOffState::default();
        for _ in 0..2000 {
            let outcome = state
                .advance(limits, target, axis, [0., 0., -9.81], 0.001)
                .unwrap();
            if outcome != PullOffOutcome::Retained {
                return (state, outcome);
            }
        }
        (state, PullOffOutcome::Retained)
    }
    #[test]
    fn weak_pull_stalls_strong_pull_extracts_and_weak_grip_slips() {
        let weak = PullOffLimits {
            arm_force_n: 9.,
            ..limits()
        };
        assert_eq!(pull(weak, [1., 0., 0.], [0.1, 0., 0.]).0.travel_m, 0.);
        assert_eq!(
            pull(limits(), [1., 0., 0.], [0.1, 0., 0.]).1,
            PullOffOutcome::Extracted
        );
        let weak_grip = PullOffLimits {
            grip_force_n: 5.,
            ..limits()
        };
        assert_eq!(
            pull(weak_grip, [1., 0., 0.], [0.1, 0., 0.]).1,
            PullOffOutcome::Slipped
        );
    }
    #[test]
    fn upward_pull_must_also_overcome_weight_and_sideways_pull_cannot_extract() {
        let marginal = PullOffLimits {
            arm_force_n: 12.,
            ..limits()
        };
        assert_eq!(pull(marginal, [0., 0., 1.], [0., 0., 0.1]).0.travel_m, 0.);
        assert_eq!(
            pull(marginal, [1., 0., 0.], [0.1, 0., 0.]).1,
            PullOffOutcome::Extracted
        );
        assert_eq!(pull(limits(), [1., 0., 0.], [0., 0.1, 0.]).0.travel_m, 0.);
    }
    #[test]
    fn invalid_inputs_do_not_mutate_and_equal_load_holds() {
        let mut state = PullOffState::default();
        let initial = state;
        assert!(
            state
                .advance(limits(), [f64::NAN; 3], [1., 0., 0.], [0.; 3], 0.001)
                .is_err()
        );
        assert_eq!(state, initial);
        let equal = PullOffLimits {
            arm_force_n: 10.,
            ..limits()
        };
        assert_eq!(pull(equal, [1., 0., 0.], [0.1, 0., 0.]).0, initial);
    }
}
