// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-owned drive requests, independent of scheduling and external transports.
use serde::{Deserialize, Serialize};

/// Held wheel-axis motor torques in the caller's wheel roster order.
/// Positive torque increases the corresponding wheel's reported angular velocity.
/// These are requests in newton metres, before motor-envelope and power limiting;
/// zero requests free rolling rather than velocity-controller braking.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WheelTorques(Vec<f64>);

impl WheelTorques {
    /// Construct finite wheel-axis torque requests. Admission to a chassis also
    /// checks that the count matches its nonempty wheel roster.
    ///
    /// ```
    /// use rm_simulator_engine::control::WheelTorques;
    /// let torques = WheelTorques::new(vec![1.0, -1.0, 0.0, 0.0])?;
    /// torques.validate(4)?;
    /// assert_eq!(torques.as_nm()[1], -1.0);
    /// assert!(WheelTorques::new(vec![f64::NAN]).is_err());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn new(torques_nm: Vec<f64>) -> Result<Self, &'static str> {
        let request = Self(torques_nm);
        request.validate(request.0.len())?;
        Ok(request)
    }

    /// Requested wheel-axis torques in newton metres, in wheel roster order.
    pub fn as_nm(&self) -> &[f64] {
        &self.0
    }

    /// Check finite values and exact nonzero wheel count, including after deserialization.
    pub fn validate(&self, wheel_count: usize) -> Result<(), &'static str> {
        if wheel_count == 0 || self.0.len() != wheel_count {
            return Err("wheel torque count must match a nonempty chassis wheel roster");
        }
        if self.0.iter().any(|v| !v.is_finite()) {
            return Err("wheel motor torques must be finite");
        }
        Ok(())
    }
}

/// Held drive mode, restored with the caller's physical state. Gimbal and folding
/// requests remain caller-owned; a new high-level command selects `BodyVelocity`.
///
/// ```
/// use rm_simulator_engine::control::{DriveControl, WheelTorques};
/// let request = DriveControl::WheelTorques(WheelTorques::new(vec![0.; 4])?);
/// request.validate(4)?;
/// assert!(request.validate(2).is_err());
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum DriveControl {
    /// Built-in body-velocity tracking and optional balance/jump assistance.
    #[default]
    BodyVelocity,
    /// Direct wheel-axis torques, bypassing velocity tracking and balance/jump
    /// assistance. Suspension, traction, motor limits and authority cutoffs remain active.
    WheelTorques(WheelTorques),
}

impl DriveControl {
    /// Validate a held request against a chassis wheel roster.
    pub fn validate(&self, wheel_count: usize) -> Result<(), &'static str> {
        match self {
            Self::BodyVelocity => Ok(()),
            Self::WheelTorques(torques) => torques.validate(wheel_count),
        }
    }
}
