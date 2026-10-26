// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Validation failures for queries descriptions.

/// A rejected queries input; no source values are repaired implicitly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum QueryError {
    /// Ray requires finite origin/direction and a positive finite range.
    InvalidRay,
    /// Ray direction must be nonzero.
    ZeroDirection,
}
impl QueryError {
    /// Original diagnostic, also used by legacy string-error adapters.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::InvalidRay => "ray requires finite origin/direction and a positive finite range",
            Self::ZeroDirection => "ray direction must be nonzero",
        }
    }
}
impl std::fmt::Display for QueryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}
impl std::error::Error for QueryError {}

// Retain ? propagation in existing adapters while new callers use typed errors.
impl From<QueryError> for &'static str {
    fn from(error: QueryError) -> Self {
        error.as_str()
    }
}
