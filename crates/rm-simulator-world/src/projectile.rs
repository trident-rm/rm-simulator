// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Physical exports and the live simulation's serialized hit report.
use crate::rune::HitOutcome;
pub use rm_simulator_physics::projectile::*;
use serde::{Deserialize, Serialize};
/// Why an armor module did not register a contact.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Rejection {
    /// Normal speed at or below the Table 5-1 threshold.
    NormalSpeed,
    /// Within the module's minimum detection interval of its previous strike.
    DetectionInterval,
    /// A rune ring the referee has switched off after earlier Big Rune
    /// activations (section 5.5.2: only rings 4..10, then 7..10, detect).
    DisabledRing,
    /// Contact on the housing, edge or back rather than the scoring face.
    OutsideTarget,
    /// 42 mm projectiles are not detected by the Power Rune.
    Caliber,
}

/// One armor contact the field evaluated at a tick, whether the module
/// registered it or not. Detected hits carry damage; rejected ones name the
/// condition that failed. The field keeps the last second of them.
///
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArmorHit {
    /// Field time the contact was scored at, in nanoseconds on the world's
    /// instance step clock.
    pub time_ns: u64,
    /// Identity of the ball, as returned by `Field::fire`.
    pub projectile: u64,
    /// The chassis that fired, when a pilot did.
    pub shooter: Option<u32>,
    /// Projectile caliber that made the contact.
    pub caliber: Caliber,
    /// Which armor module was touched.
    pub target: ArmorTarget,
    /// Contact point in world FLU metres.
    pub position_m: [f64; 3],
    /// Signed width/height offset from the scoring face centre.
    pub local_offset_m: [f64; 2],
    /// Closing speed along the plate's outward normal, positive when approaching.
    pub normal_speed_m_s: f64,
    /// The module registered the strike; `rejection` explains a `false`.
    pub detected: bool,
    /// Why the module ignored the contact; `None` for a detected strike.
    pub rejection: Option<Rejection>,
    /// Rule outcome for detected rune strikes.
    pub rune_outcome: Option<HitOutcome>,
    /// HP removed from an outpost or a robot, including the centre bonus.
    pub damage: u32,
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn calibers_follow_the_rule_manual_tables() {
        assert_eq!(Caliber::Mm17.launch_speed_limit_m_s(), 25.);
        assert_eq!(Caliber::Mm42.launch_speed_limit_m_s(), 12.);
        assert_eq!(Caliber::Mm17.armor_detection_speed_m_s(), 12.);
        // Relaxed from Table 5-1's 10 m/s; see the constant.
        assert_eq!(Caliber::Mm42.armor_detection_speed_m_s(), 8.);
        assert_eq!(Caliber::Mm17.detection_interval_ns(), 50_000_000);
        assert_eq!(Caliber::Mm42.detection_interval_ns(), 200_000_000);
        assert_eq!(Shot::at_limit(Caliber::Mm42).speed_m_s, 12.);
    }
}
