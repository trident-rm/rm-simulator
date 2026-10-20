// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Live base armor. Geometry is fitted from the verified CAD by the server.
use crate::{Pose, Team};
use serde::{Deserialize, Serialize};

/// Native 2027 base HP for shared presentation records.
pub const INITIAL_HP: u32 = rm_simulator_gameplay::rmuc_2027::match_rules::BASE_HP;
/// Native 2027 bases have no separate virtual shield.
pub const INITIAL_SHIELD_HP: u32 = 0;

/// Fitted scoring geometry for one live base: the seven plate poses and the
/// dart rail's two stops. The server reads both off the verified CAD package.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BaseConfig {
    /// Team defending this base.
    pub team: Team,
    /// Three lower plates, upper front, two other upper plates, then dart detector.
    /// +x is the outward scoring normal, +y width, +z height.
    pub plates: [Pose; 7],
    /// World translations at the dart rail's two stops, relative to its rest pose.
    pub dart_offsets_m: [[f64; 3]; 2],
}
impl BaseConfig {
    /// Reject non-finite plates, a rotation that is not unit length, or dart
    /// travel outside 1 m. The server calls this before a config reaches a field.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.plates.iter().any(|pose| {
            !pose
                .translation_m
                .iter()
                .chain(&pose.rotation_wxyz)
                .all(|v| v.is_finite())
                || (pose.rotation_wxyz.iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-6
        }) || self
            .dart_offsets_m
            .iter()
            .flatten()
            .any(|v| !v.is_finite() || v.abs() > 1.)
        {
            return Err("invalid base scoring pose or dart travel");
        }
        Ok(())
    }
}
/// One base's fitted geometry with its current HP and shield. A snapshot is
/// taken once when the field is built and then updated in place by damage.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BaseSnapshot {
    /// Plates and dart travel this state was fitted with.
    pub config: BaseConfig,
    /// Remaining HP; zero disables every plate and ends a running match.
    pub hp: u32,
    /// Remaining virtual shield, spent before HP.
    pub shield_hp: u32,
}
impl BaseSnapshot {
    /// A base at [`INITIAL_HP`] and [`INITIAL_SHIELD_HP`].
    pub fn new(config: BaseConfig) -> Self {
        Self {
            config,
            hp: INITIAL_HP,
            shield_hp: INITIAL_SHIELD_HP,
        }
    }
    /// Scoring face pose of `plate` (0 to 6). Plate 6 is the dart detector,
    /// whose translation slides between the two rail stops: `dart_position`
    /// runs from 0 to 1 and rests in the middle (see [`crate::referee::dart_target_position`]).
    pub fn pose(&self, plate: usize, dart_position: f64) -> Pose {
        let mut pose = self.config.plates[plate];
        if plate == 6 {
            let fraction = dart_position;
            for (i, point) in pose.translation_m.iter_mut().enumerate() {
                *point += self.config.dart_offsets_m[0][i] * (1. - fraction)
                    + self.config.dart_offsets_m[1][i] * fraction;
            }
        }
        pose
    }
    /// Refill HP and shield for a new match or a reset.
    pub fn reset(&mut self) {
        self.hp = INITIAL_HP;
        self.shield_hp = INITIAL_SHIELD_HP;
    }
}
