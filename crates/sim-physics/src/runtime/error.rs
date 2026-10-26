// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Actionable runtime failures, with unchanged compatibility diagnostics.
/// A scoped runtime token was rejected before the requested operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BindingError {
    /// The token names a different runtime instance.
    Foreign(&'static str),
    /// The token no longer names a live installation.
    Stale(&'static str),
    /// The requested logical identity is already occupied.
    Duplicate(&'static str),
    /// A monotonic generation or discontinuity counter is exhausted.
    Exhausted(&'static str),
}
impl BindingError {
    /// Original diagnostic, including the affected installation kind.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Foreign(s) | Self::Stale(s) | Self::Duplicate(s) | Self::Exhausted(s) => s,
        }
    }
}
impl std::fmt::Display for BindingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}
impl std::error::Error for BindingError {}

/// Failure to admit or complete a physical runtime operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum RuntimeError {
    /// A stale, foreign, occupied or exhausted installation identity.
    Binding(BindingError),
    /// A solver-free model was rejected.
    Model(sim_engine::model::ModelError),
    /// Cooking a native shape failed.
    Cooking(crate::cooking::CookError),
    /// A carrier/mount observation was rejected.
    Carrier(crate::sensors::carrier::CarrierError),
    /// Caller values failed the operation's documented admission rules.
    InvalidInput(&'static str),
    /// Advancing would overflow the nanosecond clock.
    TimeOverflow,
    /// The requested observation storage could not be allocated.
    Allocation,
}
impl RuntimeError {
    /// Original diagnostic for compatibility participants and adapters.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Binding(e) => e.as_str(),
            Self::Model(e) => e.as_str(),
            Self::Cooking(e) => e.as_str(),
            Self::Carrier(e) => e.as_str(),
            Self::InvalidInput(s) => s,
            Self::TimeOverflow => "physics time overflow",
            Self::Allocation => "contact observation allocation failed",
        }
    }
}
impl From<&'static str> for RuntimeError {
    fn from(message: &'static str) -> Self {
        Self::InvalidInput(message)
    }
}
impl std::fmt::Display for RuntimeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}
impl std::error::Error for RuntimeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Binding(error) => Some(error),
            Self::Model(error) => Some(error),
            Self::Cooking(error) => Some(error),
            Self::Carrier(error) => Some(error),
            _ => None,
        }
    }
}
impl From<RuntimeError> for &'static str {
    fn from(error: RuntimeError) -> Self {
        error.as_str()
    }
}

impl From<sim_engine::model::ModelError> for RuntimeError {
    fn from(error: sim_engine::model::ModelError) -> Self {
        Self::Model(error)
    }
}

impl From<crate::cooking::CookError> for RuntimeError {
    fn from(error: crate::cooking::CookError) -> Self {
        Self::Cooking(error)
    }
}

impl From<crate::sensors::carrier::CarrierError> for RuntimeError {
    fn from(error: crate::sensors::carrier::CarrierError) -> Self {
        Self::Carrier(error)
    }
}
