// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Deterministic referee power and module-failure authority, UC §§5.1.4–5.
use super::match_rules::{GroundRole, Match, Phase};
use serde::{Deserialize, Serialize};
/// Full 60 J referee buffer in microjoules, UC Figure 5-2.
pub const BUFFER_UJ: u64 = 60_000_000;
/// Five-second chassis power interruption, in ns, UC Figure 5-2.
pub const CUT_NS: u64 = 5_000_000_000;
/// Observed connections, sampled by the referee at 2 Hz (UC §5.1.5).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Modules {
    /// Main controller connection; false suppresses ordinary hit/module damage.
    pub main_online: bool,
    /// Speed monitor connection; independent launcher lock cause.
    pub speed_online: bool,
    /// Localization module connection; changes Radar quality rather than HP.
    pub localization_online: bool,
    /// Number of disconnected armor modules, zero through four.
    pub offline_armor: u8,
    /// Supercapacitor manager connection.
    pub supercap_online: bool,
}
impl Default for Modules {
    fn default() -> Self {
        Self {
            main_online: true,
            speed_online: true,
            localization_online: true,
            offline_armor: 0,
            supercap_online: true,
        }
    }
}
/// Complete restorable referee power meter and connectivity input.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Power {
    /// Remaining buffer in microjoules, capped at 60 J.
    pub buffer_uj: u64,
    /// Drive cut end, in running-round ns; gimbal is unaffected by over-power alone.
    pub cut_until_ns: u64,
    /// Latest non-regenerating electrical drive load, in milliwatts.
    pub actual_mw: u32,
    /// Pending connectivity input; applied only on the next 500 ms settlement.
    pub modules: Modules,
    /// Fractional offline damage carry, in 1/40 HP; always below 40.
    pub offline_hp_carry: u32,
}
impl Default for Power {
    fn default() -> Self {
        Self {
            buffer_uj: BUFFER_UJ,
            cut_until_ns: 0,
            actual_mw: 0,
            modules: Modules::default(),
            offline_hp_carry: 0,
        }
    }
}
impl Power {
    /// Validate bounded authority on restore.
    pub fn validate(&self, now_ns: u64) -> bool {
        self.buffer_uj <= BUFFER_UJ
            && self.actual_mw <= 10_000_000
            && self.modules.offline_armor <= 4
            && self.offline_hp_carry < 40
            && self.cut_until_ns <= now_ns.saturating_add(CUT_NS)
    }
}
impl Match {
    /// Replace bounded connectivity observations; effects settle at 2 Hz.
    pub fn set_modules(
        &mut self,
        id: u32,
        main_online: bool,
        speed_online: bool,
        offline_armor: u8,
        supercap_online: bool,
    ) -> Result<(), &'static str> {
        if offline_armor > 4 {
            return Err("at most four armor modules");
        }
        let r = self
            .robots
            .iter_mut()
            .find(|r| r.id == id)
            .ok_or("unknown robot")?;
        r.combat.power.modules = Modules {
            localization_online: r.combat.power.modules.localization_online,
            main_online,
            speed_online,
            offline_armor,
            supercap_online,
        };
        Ok(())
    }
    /// Replace the measured electrical load, in mW; physics must never credit regeneration.
    pub fn observe_drive_power(&mut self, id: u32, actual_mw: u32) {
        if let Some(r) = self.robots.iter_mut().find(|r| r.id == id) {
            r.combat.power.actual_mw = actual_mw.min(10_000_000);
        }
    }
    /// Rule limit in watts after paid-respawn and Sentry posture effects.
    pub fn power_limit_w(&self, id: u32) -> u32 {
        let Some(r) = self.robots.iter().find(|r| r.id == id) else {
            return 0;
        };
        let base = r.stats(self.elapsed_ns()).chassis_power_w;
        if r.role == GroundRole::Sentry {
            r.combat.sentry.power_w(base)
        } else {
            base
        }
    }
    /// Instantaneous motor budget in watts, including the remaining 100 ms buffer burst.
    /// The physical hardware budget is an independent upper bound. Zero disables drive.
    pub fn drive_budget_w(&self, id: u32) -> u32 {
        let Some(r) = self.robots.iter().find(|r| r.id == id) else {
            return 0;
        };
        if self.phase != Phase::Running {
            return u32::MAX;
        }
        if r.hp == 0 || !r.online || self.elapsed_ns() < r.combat.power.cut_until_ns {
            return 0;
        }
        self.power_limit_w(id)
            .saturating_add((r.combat.power.buffer_uj / 100_000) as u32)
    }
    pub(super) fn settle_power_modules(&mut self) {
        let now = self.elapsed_ns();
        if self.phase != Phase::Running {
            return;
        }
        if now.is_multiple_of(500_000_000) {
            let mut damage = vec![];
            for r in &mut self.robots {
                r.online = r.combat.power.modules.main_online;
                r.combat.speed_module_online = r.combat.power.modules.speed_online;
                r.combat.localization_module_online = r.combat.power.modules.localization_online;
                if r.hp == 0 {
                    continue;
                }
                let p = &mut r.combat.power;
                let loss = if !r.online {
                    p.offline_hp_carry += r.max_hp;
                    let loss = p.offline_hp_carry / 40;
                    p.offline_hp_carry %= 40;
                    loss
                } else {
                    4 * (u32::from(p.modules.offline_armor) + u32::from(!p.modules.supercap_online))
                };
                if loss > 0 {
                    damage.push((r.id, loss));
                }
            }
            for (id, loss) in damage {
                self.damage_robot(id, loss, true);
            }
        }
        if now.is_multiple_of(100_000_000) {
            let limits = self
                .robots
                .iter()
                .map(|r| self.power_limit_w(r.id))
                .collect::<Vec<_>>();
            for (r, limit) in self.robots.iter_mut().zip(limits) {
                if r.hp == 0 || !r.online {
                    continue;
                }
                let p = &mut r.combat.power;
                // Latest physical slice is the held 10 Hz meter sample (simulator policy).
                let actual = if now < p.cut_until_ns { 0 } else { p.actual_mw };
                let next = i64::try_from(p.buffer_uj).unwrap()
                    + (i64::from(limit) * 1000 - i64::from(actual)) * 100;
                p.buffer_uj = next.clamp(0, BUFFER_UJ as i64) as u64;
                if next <= 0 && actual > limit * 1000 {
                    p.cut_until_ns = now + CUT_NS;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::match_rules::COUNTDOWN_NS;
    use super::*;
    use crate::Team;
    fn game() -> Match {
        let mut g = Match::new(1);
        g.add_robot(3, Team::Red, GroundRole::Infantry);
        g.start(1);
        g.advance_ns(COUNTDOWN_NS);
        g
    }
    #[test]
    fn exact_buffer_exhaustion_cuts_only_drive_and_refills() {
        let mut g = game();
        g.observe_drive_power(3, 145_000); // 100 W above level-one 45 W.
        g.advance_ns(599_999_999);
        assert_eq!(g.robots[0].combat.power.buffer_uj, 10_000_000);
        assert!(g.drive_budget_w(3) > 0);
        g.advance_ns(1);
        assert_eq!(g.robots[0].combat.power.buffer_uj, 0);
        assert_eq!(g.drive_budget_w(3), 0);
        assert!(!g.robots[0].combat.launcher_locked(g.elapsed_ns()));
        g.observe_drive_power(3, 0);
        g.advance_ns(CUT_NS);
        assert_eq!(g.robots[0].combat.power.buffer_uj, BUFFER_UJ);
        assert!(g.drive_budget_w(3) > 0);
    }
    #[test]
    fn sampled_modules_damage_and_offline_exclusion_are_independent() {
        let mut g = game();
        g.set_modules(3, true, false, 2, false).unwrap();
        g.advance_ns(499_999_999);
        assert_eq!(g.robots[0].hp, 200);
        g.advance_ns(1);
        assert_eq!(g.robots[0].hp, 188);
        assert!(g.robots[0].combat.launcher_locked(g.elapsed_ns()));
        g.set_modules(3, false, true, 4, false).unwrap();
        g.advance_ns(500_000_000);
        assert_eq!(g.robots[0].hp, 183); // 2.5%, excludes 20 HP module loss.
        assert_eq!(g.damage_robot(3, 100, false), 0);
        assert_eq!(g.drive_budget_w(3), 0);
        g.set_modules(3, true, true, 0, true).unwrap();
        g.advance_ns(500_000_000);
        assert!(g.robots[0].online);
        assert!(!g.robots[0].combat.launcher_locked(g.elapsed_ns()));
    }
    #[test]
    fn death_refills_buffer_and_partitions_match() {
        let mut g = game();
        g.observe_drive_power(3, 200_000);
        let mut split = g.clone();
        g.advance_ns(123_456_789);
        for dt in [1, 99_999_999, 23_456_789] {
            split.advance_ns(dt);
        }
        assert_eq!(g, split);
        g.damage_robot(3, u32::MAX, true);
        assert_eq!(g.robots[0].combat.power.buffer_uj, BUFFER_UJ);
    }
}
