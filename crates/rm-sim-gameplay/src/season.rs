// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Source season identity and the application's single supported season.
//!
//! Historical identities remain readable for asset diagnostics. They are not
//! Cargo features and do not select a live gameplay implementation.

use serde::{Deserialize, Serialize};

/// Source year recorded by field manifests; tournament and revision are bound separately.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Season {
    /// Archived RMUC 2026 source data, retained for offline tools and fixtures.
    #[serde(rename = "2026")]
    Rm2026,
    /// Active RMUC 2027 source data; live matches also require verified bindings.
    #[serde(rename = "2027")]
    Rm2027,
}

/// A source belongs to an archived runtime.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum SeasonError {
    /// The source year is no longer supported by application sessions.
    #[error(
        "RM{0} runtime is archived; use Git tag rm2026-final for 2026, or load a bound RMUC 2027 field"
    )]
    Archived(u16),
}

impl Season {
    /// The only season admitted by the application and headless CAD host.
    pub const CURRENT: Self = Self::Rm2027;

    /// Source calendar year, for diagnostics.
    pub const fn year(self) -> u16 {
        match self {
            Self::Rm2026 => 2026,
            Self::Rm2027 => 2027,
        }
    }

    /// Reject archived sources before creating a session or reading its geometry.
    ///
    /// ```
    /// use rm_sim_gameplay::season::Season;
    /// assert!(Season::Rm2026.require_current().is_err());
    /// assert!(Season::CURRENT.require_current().is_ok());
    /// ```
    pub const fn require_current(self) -> Result<(), SeasonError> {
        match self {
            Self::Rm2027 => Ok(()),
            Self::Rm2026 => Err(SeasonError::Archived(2026)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_current_runtime_is_admitted() {
        assert_eq!(Season::CURRENT, Season::Rm2027);
        assert_eq!(Season::Rm2027.require_current(), Ok(()));
        assert_eq!(
            Season::Rm2026.require_current(),
            Err(SeasonError::Archived(2026))
        );
    }

    #[test]
    fn archived_source_identity_remains_readable() {
        for (season, json) in [(Season::Rm2026, "\"2026\""), (Season::Rm2027, "\"2027\"")] {
            assert_eq!(serde_json::to_string(&season).unwrap(), json);
            assert_eq!(serde_json::from_str::<Season>(json).unwrap(), season);
        }
    }
}
