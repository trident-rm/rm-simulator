// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Competition identity shared by admission, clients and native hosts.
use clap::ValueEnum;

/// Competition identity; each format loads its own source-backed field.
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, ValueEnum, serde::Serialize, serde::Deserialize,
)]
pub enum Competition {
    /// Source-bound RMUC 2027 V1.0.0 field and rules.
    #[default]
    #[value(name = "rmuc-2027")]
    Rmuc2027,
    /// RMUL 2027 three-versus-three on the official 12 × 8 metre field.
    #[value(name = "rmul-2027")]
    Rmul2027,
    /// RMUL 2027 Infantry confrontation on the official 5 × 5 metre field.
    #[value(name = "rmul-infantry-2027")]
    RmulInfantry2027,
}
impl Competition {
    /// Stable CLI spelling.
    pub fn id(self) -> &'static str {
        match self {
            Self::Rmuc2027 => "rmuc-2027",
            Self::Rmul2027 => "rmul-2027",
            Self::RmulInfantry2027 => "rmul-infantry-2027",
        }
    }
    /// Native RMUL format; UC has its own source-bound authority.
    pub fn rmul_format(self) -> Option<rm_sim_world::gameplay::rmul_2027::Format> {
        use rm_sim_world::gameplay::rmul_2027::Format;
        match self {
            Self::Rmuc2027 => None,
            Self::Rmul2027 => Some(Format::ThreeVsThree),
            Self::RmulInfantry2027 => Some(Format::Infantry),
        }
    }
    /// Approximate practice extent in FLU metres, excluding any official terrain claim.
    pub fn practice_bounds(self) -> [[f64; 3]; 2] {
        let [x, y] = if self == Self::RmulInfantry2027 {
            [2.5, 2.5]
        } else {
            [6.0, 4.0]
        };
        [[-x, -y, 0.0], [x, y, 3.0]]
    }
    /// Initial practice chassis position and inward heading in degrees.
    pub fn practice_spawn(self, team: rm_sim_world::Team, slot: usize) -> ([f64; 3], f64) {
        let bounds = self.practice_bounds();
        let sign = if team == rm_sim_world::Team::Red {
            1.0
        } else {
            -1.0
        };
        (
            [sign * (bounds[1][0] - 1.0), (slot as f64 - 1.0) * 1.0, 0.5],
            if sign > 0.0 { 180.0 } else { 0.0 },
        )
    }
}
