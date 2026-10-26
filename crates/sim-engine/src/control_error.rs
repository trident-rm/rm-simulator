// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Validation failures for control descriptions.

/// A rejected control input; no source values are repaired implicitly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ControlError {
    /// Wheel torque count must match a nonempty chassis wheel roster.
    WheelCountMismatch,
    /// Wheel motor torques must be finite.
    NonfiniteTorque,
}
impl ControlError {
    /// Original diagnostic, also used by legacy string-error adapters.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::WheelCountMismatch => {
                "wheel torque count must match a nonempty chassis wheel roster"
            }
            Self::NonfiniteTorque => "wheel motor torques must be finite",
        }
    }
}
impl std::fmt::Display for ControlError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}
impl std::error::Error for ControlError {}

// Retain ? propagation in existing adapters while new callers use typed errors.
impl From<ControlError> for &'static str {
    fn from(error: ControlError) -> Self {
        error.as_str()
    }
}
