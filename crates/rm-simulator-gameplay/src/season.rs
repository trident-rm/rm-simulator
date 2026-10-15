// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Build-time season availability, separate from implemented gameplay rules.
//!
//! Features are additive: enabling both seasons keeps both identities. Enabling
//! a season does not imply its match engine is implemented. Existing gameplay and
//! field constructors use RM2026; RM2027 admission fails until its rules land.

use serde::{Deserialize, Serialize};

/// Competition season identity, independent of Cargo feature selection.
///
/// This identifies the year, not a tournament format. The current gameplay
/// implementation covers the documented RMUC 2026 subset only; RMUL variants
/// will need their own format selection rather than an alias of RMUC rules.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Season {
    /// RoboMaster 2026; current rules implement the documented RMUC subset.
    #[serde(rename = "2026")]
    Rm2026,
    /// RoboMaster 2027; isolated control rules exist, but match admission is unavailable.
    #[serde(rename = "2027")]
    Rm2027,
}

/// Why a requested season cannot start a game in this build.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum SeasonError {
    /// The season's Cargo feature was not enabled.
    #[error("RM{0} is disabled in this build; enable Cargo feature season-{0}")]
    Disabled(u16),
    /// The season is enabled, but its match engine is not ready.
    #[error("RM{0} matches are not implemented; isolated season components cannot start a match")]
    Unimplemented(u16),
}

impl Season {
    /// Seasons enabled by Cargo features, in chronological order.
    ///
    /// An entry here can still fail [`Self::require_playable`]. A build with
    /// neither feature has an empty list and cannot create a game or field.
    pub const ENABLED: &'static [Self] = &[
        #[cfg(feature = "season-2026")]
        Self::Rm2026,
        #[cfg(feature = "season-2027")]
        Self::Rm2027,
    ];

    /// Calendar year used for diagnostics and future season selection.
    pub const fn year(self) -> u16 {
        match self {
            Self::Rm2026 => 2026,
            Self::Rm2027 => 2027,
        }
    }

    /// Whether this build enables the season's infrastructure feature.
    pub const fn is_enabled(self) -> bool {
        match self {
            Self::Rm2026 => cfg!(feature = "season-2026"),
            Self::Rm2027 => cfg!(feature = "season-2027"),
        }
    }

    /// Whether a match engine exists, independently of feature selection.
    ///
    /// `true` means the simulator's documented subset exists, not that every
    /// rulebook clause is enforced. Isolated rule components do not qualify.
    /// See `docs/gameplay.md` for the integration boundary.
    pub const fn has_gameplay(self) -> bool {
        matches!(self, Self::Rm2026)
    }

    /// Reject a disabled season or one without gameplay before creating state.
    ///
    /// ```
    /// use rm_simulator_gameplay::season::{Season, SeasonError};
    /// let expected = if Season::Rm2027.is_enabled() {
    ///     SeasonError::Unimplemented(2027)
    /// } else {
    ///     SeasonError::Disabled(2027)
    /// };
    /// assert_eq!(Season::Rm2027.require_playable(), Err(expected));
    /// ```
    pub const fn require_playable(self) -> Result<(), SeasonError> {
        if !self.is_enabled() {
            Err(SeasonError::Disabled(self.year()))
        } else if !self.has_gameplay() {
            Err(SeasonError::Unimplemented(self.year()))
        } else {
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Config, Error, Game};

    #[test]
    fn season_feature_membership_and_admission() {
        for (season, enabled, implemented) in [
            (Season::Rm2026, cfg!(feature = "season-2026"), true),
            (Season::Rm2027, cfg!(feature = "season-2027"), false),
        ] {
            assert_eq!(Season::ENABLED.contains(&season), enabled);
            assert_eq!(season.is_enabled(), enabled);
            assert_eq!(season.has_gameplay(), implemented);
            let expected = if !enabled {
                Err(SeasonError::Disabled(season.year()))
            } else if !implemented {
                Err(SeasonError::Unimplemented(season.year()))
            } else {
                Ok(())
            };
            assert_eq!(season.require_playable(), expected);
            assert_eq!(
                Game::new_for_season(season, Config::default()).map(|_| ()),
                expected.map_err(Error::Season),
            );
        }
    }

    #[test]
    fn season_identity_survives_serialization_without_feature_aliasing() {
        for (season, json) in [(Season::Rm2026, "\"2026\""), (Season::Rm2027, "\"2027\"")] {
            assert_eq!(serde_json::to_string(&season).unwrap(), json);
            assert_eq!(serde_json::from_str::<Season>(json).unwrap(), season);
        }
    }

    #[test]
    fn legacy_game_constructor_explicitly_requires_2026() {
        assert_eq!(
            Game::new(Config::default()).map(|_| ()),
            Season::Rm2026.require_playable().map_err(Error::Season),
        );
    }
}
