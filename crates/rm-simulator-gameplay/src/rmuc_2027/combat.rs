// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native UC 2027 V1.0.0 §§5.1, 5.3–5.5 combat lifecycle state and tables.
use super::match_rules::{GroundRole, Robot};
use crate::{InfantryChassis, InfantryLauncher, Stats};
use serde::{Deserialize, Serialize};

/// Ten-hertz cooling/healing settlement period, UC §5.1.3, in nanoseconds.
pub const SETTLEMENT_NS: u64 = 100_000_000;
/// Six-second out-of-combat and remote-delivery delay, UC Table 2-1/§5.3.
pub const SIX_SECONDS_NS: u64 = 6_000_000_000;
/// Level thresholds in tenths of XP, UC Table 5-15; shared XP rounds to 0.1.
pub const EXPERIENCE_TENTHS: [u64; 10] = [
    0, 5500, 11000, 16500, 22000, 27500, 33000, 38500, 44000, 50000,
];
/// Remote logistics already paid for; HP delivery is cancelled by death.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Delivery {
    /// Running-round delivery deadline, in nanoseconds.
    pub due_ns: u64,
    /// Rounds to deliver, or zero for a 60%-maximum-HP purchase.
    pub rounds: u32,
}
/// A normal respawn's progress; paid respawn bypasses this timer (§5.3.2).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Respawn {
    /// Original progress requirement, in ns of progress, before base logistics.
    pub original_ns: u64,
    /// Accumulated progress, at one or four progress-ns per simulation-ns.
    pub progress_ns: u64,
}
/// Complete per-robot combat authority, independent of physics and host time.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Combat {
    /// Infantry chassis choice; table values are native 2027, not 2026 Stats.
    pub chassis: InfantryChassis,
    /// Infantry launcher choice; other roles ignore this field.
    pub launcher: InfantryLauncher,
    /// Chassis choice has been confirmed during this running round.
    pub chassis_locked: bool,
    /// Launcher choice has been confirmed during this running round.
    pub launcher_locked: bool,
    /// Current level, 1–10; Sentry remains level one.
    pub level: u8,
    /// Accumulated experience in tenths, including distributed awards.
    pub experience_tenths: u64,
    /// Launcher heat in tenths of a heat unit.
    pub heat_tenths: u64,
    /// Heat limit exceeded; stays latched until heat reaches zero.
    pub overheated: bool,
    /// Permanent heat lock; Figure 5-1's inclusive boundary is the adopted policy.
    pub heat_locked_for_round: bool,
    /// Temporary overspeed lock deadline, in running-round nanoseconds.
    pub overspeed_until_ns: u64,
    /// Permanent overspeed lock, independent of heat and module connectivity.
    pub overspeed_locked_for_round: bool,
    /// Speed-monitor link state; reconnection clears only this lock cause.
    pub speed_module_online: bool,
    /// Remaining 17 mm allowance (§5.4.2); Heavy/42 mm remains unavailable.
    pub allowance: u32,
    /// Time of last launch or actual HP loss; None means initially out of combat.
    pub last_combat_ns: Option<u64>,
    /// Own supply RFID expiry, including detection while defeated.
    pub supply_expires_ns: Option<u64>,
    /// Own vantage RFID expiry; currently used only for ammunition exchange.
    pub vantage_expires_ns: Option<u64>,
    /// Normal respawn progress while defeated.
    pub respawn: Option<Respawn>,
    /// Number of paid respawns before the next death.
    pub paid_respawns: u32,
    /// Weakness blocks launching and ordinary benefits.
    pub weakened: bool,
    /// Paid weakness deadline; None for normal weakness cleared by a service card.
    pub weakness_until_ns: Option<u64>,
    /// Respawn invincibility deadline, in running-round ns.
    pub invincible_until_ns: u64,
    /// Normal respawn's earliest invincibility end on service-card recovery.
    pub minimum_invincible_until_ns: u64,
    /// Paid respawn's reported doubled-power deadline; power enforcement is separate.
    pub double_power_until_ns: u64,
    /// Six-second paid ammunition/HP deliveries.
    pub deliveries: Vec<Delivery>,
    /// Sentry's last collected minute; uncollected 100-round grants accumulate.
    pub sentry_collected_minutes: u32,
    /// Fractional healing carry, in 1/200,000,000 HP, rounded cumulatively.
    pub healing_carry: i64,
}
impl Default for Combat {
    fn default() -> Self {
        Self {
            chassis: InfantryChassis::HpFocused,
            launcher: InfantryLauncher::CoolingFocused,
            chassis_locked: false,
            launcher_locked: false,
            level: 1,
            experience_tenths: 0,
            heat_tenths: 0,
            overheated: false,
            heat_locked_for_round: false,
            overspeed_until_ns: 0,
            overspeed_locked_for_round: false,
            speed_module_online: true,
            allowance: 0,
            last_combat_ns: None,
            supply_expires_ns: None,
            vantage_expires_ns: None,
            respawn: None,
            paid_respawns: 0,
            weakened: false,
            weakness_until_ns: None,
            invincible_until_ns: 0,
            minimum_invincible_until_ns: 0,
            double_power_until_ns: 0,
            deliveries: vec![],
            sentry_collected_minutes: 0,
            healing_carry: 0,
        }
    }
}
impl Combat {
    /// Whether six continuous seconds have passed without a launch or HP loss.
    pub fn out_of_combat(&self, now_ns: u64) -> bool {
        self.last_combat_ns
            .is_none_or(|t| now_ns.saturating_sub(t) >= SIX_SECONDS_NS)
    }
    /// Independent launcher locks, excluding phase, HP and allowance checks.
    pub fn launcher_locked(&self, now_ns: u64) -> bool {
        self.weakened
            || self.overheated
            || self.heat_locked_for_round
            || self.overspeed_locked_for_round
            || now_ns < self.overspeed_until_ns
            || !self.speed_module_online
    }
    /// Heat-induced first-person video cut; permanent launch lock alone does
    /// not extend the cut after heat cools to zero (UC §5.1.3).
    pub fn video_cut(&self) -> bool {
        self.overheated
    }
    /// Validate finite-range integer state before authoritative restore.
    pub fn validate(&self, now_ns: u64) -> bool {
        (1..=10).contains(&self.level)
            && u64::from(self.sentry_collected_minutes) <= now_ns / 60_000_000_000
            && self.deliveries.iter().all(|d| d.rounds <= 1000)
            && self.heat_tenths <= 1_000_000
            && self.healing_carry.unsigned_abs() <= 100_000_000
            && self.last_combat_ns.is_none_or(|t| t <= now_ns)
            && self
                .deliveries
                .iter()
                .all(|d| d.due_ns <= now_ns.saturating_add(SIX_SECONDS_NS))
            && self
                .supply_expires_ns
                .into_iter()
                .chain(self.vantage_expires_ns)
                .all(|t| t <= now_ns.saturating_add(2_000_000_000))
    }
}
impl Robot {
    /// Native UC Tables 3-6, 5-17 and 5-18, with reported paid-respawn power.
    /// The shared Stats value is a data type; no 2026 performance tables run.
    pub fn stats(&self, now_ns: u64) -> Stats {
        let c = &self.combat;
        let i = usize::from(c.level.clamp(1, 10) - 1);
        let (hp, power) = if c.chassis == InfantryChassis::PowerFocused {
            (
                [150, 175, 200, 225, 250, 275, 300, 325, 350, 400][i],
                [60, 65, 70, 75, 80, 85, 90, 95, 100, 100][i],
            )
        } else {
            (
                [200, 225, 250, 275, 300, 325, 350, 375, 400, 400][i],
                [45, 50, 55, 60, 65, 70, 75, 80, 90, 100][i],
            )
        };
        let (limit, cooling) = if c.launcher == InfantryLauncher::BurstFocused {
            (
                [170, 180, 190, 200, 210, 220, 230, 240, 250, 260][i],
                [5, 7, 9, 11, 12, 13, 14, 16, 18, 20][i],
            )
        } else {
            (
                [40, 48, 56, 64, 72, 80, 88, 96, 104, 120][i],
                [12, 14, 16, 18, 20, 22, 24, 26, 28, 30][i],
            )
        };
        let mut stats = if self.role == GroundRole::Infantry {
            Stats {
                max_hp: hp,
                chassis_power_w: power,
                heat_limit: limit,
                cooling_per_s: cooling,
            }
        } else {
            Stats {
                max_hp: if self.role == GroundRole::Heavy {
                    400
                } else {
                    200
                },
                chassis_power_w: 60,
                heat_limit: 100,
                cooling_per_s: 10,
            }
        };
        if now_ns < c.double_power_until_ns {
            stats.chassis_power_w = (stats.chassis_power_w * 2).min(200);
        }
        stats
    }
    /// Whether this live, connected robot is currently out of combat.
    pub fn out_of_combat(&self, now_ns: u64) -> bool {
        self.hp > 0 && self.online && self.combat.out_of_combat(now_ns)
    }
    /// Whether own supply detection is still within the two-second RFID grace.
    pub fn in_supply(&self, now_ns: u64) -> bool {
        self.combat.supply_expires_ns.is_some_and(|t| t > now_ns)
    }
}
