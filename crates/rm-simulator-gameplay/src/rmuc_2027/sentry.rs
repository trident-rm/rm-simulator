// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! UC Table 3-6 and §5.7.2/Table 5-28 Sentry authority.
use super::match_rules::{GroundRole, Match, Phase};
use serde::{Deserialize, Serialize};
/// Minimum interval between posture selections, UC §5.7.2, in ns.
pub const SWITCH_NS: u64 = 5_000_000_000;
/// Cumulative threshold above which a posture weakens, in ns (UC §5.7.2).
pub const WEAKEN_NS: u64 = 180_000_000_000;
/// Independent enhanced allowance per posture, in ns (UC §5.7.2).
pub const ENHANCED_NS: u64 = 15_000_000_000;
/// Pre-round operating mode, independent of the physical robot/controller.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Mode {
    /// Default when no pre-self-check selection was made.
    #[default]
    Automatic,
    /// Lower performance; commander interventions have no extra fee.
    SemiAutomatic,
}
/// Posture identity; array order is Attack, Defense, Movement.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Posture {
    /// Cooling focus, reduced power and vulnerability.
    Attack,
    /// Defense focus, reduced cooling and power.
    Defense,
    /// Initial posture, increased power and reduced cooling.
    #[default]
    Movement,
}
impl Posture {
    /// Stable index into cumulative budgets.
    pub fn index(self) -> usize {
        self as usize
    }
}
/// Restorable cumulative budgets. Death does not reset this state.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Sentry {
    /// Pre-round mode, retained when starting another round.
    pub mode: Mode,
    /// Normal respawn confirmation for the current death (§5.7.2).
    pub respawn_confirmed: bool,
    /// Current posture, initially Movement.
    pub posture: Posture,
    /// Whether the current posture consumes its enhanced allowance.
    pub enhanced: bool,
    /// Time spent in each posture, including enhanced time, in ns.
    pub used_ns: [u64; 3],
    /// Enhanced time spent in each posture, in ns.
    pub enhanced_ns: [u64; 3],
    /// Next voluntary posture selection time, in running-round ns.
    pub switch_after_ns: u64,
}
impl Sentry {
    /// Whether the current posture has exceeded three cumulative minutes.
    pub fn weakened(&self) -> bool {
        self.used_ns[self.posture.index()] > WEAKEN_NS
    }
    /// Next budget boundary; the caller splits advances here for partition invariance.
    pub(crate) fn boundary(&self) -> u64 {
        let i = self.posture.index();
        let weak = (WEAKEN_NS + 1).saturating_sub(self.used_ns[i]);
        let enhanced = if self.enhanced {
            ENHANCED_NS - self.enhanced_ns[i]
        } else {
            u64::MAX
        };
        if weak > 0 {
            enhanced.min(weak)
        } else {
            enhanced
        }
    }
    /// Consume running time, returning to ordinary posture at the exact allowance end.
    pub(crate) fn advance(&mut self, dt: u64) {
        let i = self.posture.index();
        self.used_ns[i] = self.used_ns[i].saturating_add(dt);
        if self.enhanced {
            self.enhanced_ns[i] = (self.enhanced_ns[i] + dt).min(ENHANCED_NS);
            if self.enhanced_ns[i] == ENHANCED_NS {
                self.enhanced = false;
            }
        }
    }
    /// Validate budgets on authoritative restore.
    pub fn validate(&self, now_ns: u64) -> bool {
        self.used_ns
            .iter()
            .try_fold(0u64, |sum, &v| sum.checked_add(v))
            .is_some_and(|sum| sum <= now_ns)
            && self
                .enhanced_ns
                .iter()
                .zip(self.used_ns)
                .all(|(&e, u)| e <= ENHANCED_NS && e <= u)
            && (!self.enhanced || self.enhanced_ns[self.posture.index()] < ENHANCED_NS)
            && self.switch_after_ns <= now_ns.saturating_add(SWITCH_NS)
    }
    /// Apply Table 5-28 after other power benefits, rounding down to integer watts.
    pub fn power_w(&self, power_w: u32) -> u32 {
        match self.posture {
            Posture::Attack | Posture::Defense => power_w / 2,
            Posture::Movement if self.enhanced => 200,
            Posture::Movement if self.weakened() => power_w * 12 / 10,
            Posture::Movement => power_w * 3 / 2,
        }
    }
}
impl Match {
    /// Select mode before simulator countdown (the self-check boundary proxy).
    ///
    /// ```
    /// use rm_simulator_gameplay::{Team, rmuc_2027::{match_rules::{Match, GroundRole}, sentry::Mode}};
    /// let mut game = Match::new(1);
    /// game.add_robot(7, Team::Red, GroundRole::Sentry);
    /// game.set_sentry_mode(7, Mode::SemiAutomatic).unwrap();
    /// assert_eq!(game.robots[0].max_hp, 200);
    /// ```
    pub fn set_sentry_mode(&mut self, id: u32, mode: Mode) -> Result<(), &'static str> {
        if self.phase != Phase::Idle {
            return Err("Sentry mode locks before countdown");
        }
        let r = self
            .robots
            .iter_mut()
            .find(|r| r.id == id && r.role == GroundRole::Sentry)
            .ok_or("not a Sentry")?;
        r.combat.sentry.mode = mode;
        r.max_hp = r.stats(0).max_hp;
        r.hp = r.max_hp;
        Ok(())
    }
    /// Select ordinary/enhanced posture; only accepted commander interventions cost gold.
    pub fn set_sentry_posture(
        &mut self,
        id: u32,
        posture: Posture,
        enhanced: bool,
        commander: bool,
    ) -> Result<(), &'static str> {
        let now = self.elapsed_ns();
        let r = self
            .robots
            .iter()
            .find(|r| r.id == id && r.role == GroundRole::Sentry)
            .ok_or("not a Sentry")?;
        if self.phase != Phase::Running || r.hp == 0 || !r.online {
            return Err("Sentry unavailable");
        }
        if now < r.combat.sentry.switch_after_ns {
            return Err("Sentry posture cooldown");
        }
        if (posture, enhanced) == (r.combat.sentry.posture, r.combat.sentry.enhanced) {
            return Err("posture already selected");
        }
        if enhanced && r.combat.sentry.enhanced_ns[posture.index()] >= ENHANCED_NS {
            return Err("enhanced posture exhausted");
        }
        self.sentry_intervention(id, commander)?;
        let r = self.robots.iter_mut().find(|r| r.id == id).unwrap();
        r.combat.sentry.posture = posture;
        r.combat.sentry.enhanced = enhanced;
        r.combat.sentry.switch_after_ns = now + SWITCH_NS;
        if posture == Posture::Attack && enhanced {
            r.combat.heat_tenths = 0;
            r.combat.overheated = false;
        }
        Ok(())
    }
    /// Charge a successfully validated commander action; autonomous actions are free.
    /// Purchase prices remain separate. Callers must apply this transactionally.
    pub fn sentry_intervention(&mut self, id: u32, commander: bool) -> Result<(), &'static str> {
        let r = self
            .robots
            .iter()
            .find(|r| r.id == id && r.role == GroundRole::Sentry)
            .ok_or("not a Sentry")?;
        if self.phase != Phase::Running || !r.online {
            return Err("Sentry unavailable");
        }
        let cost = if commander && r.combat.sentry.mode == Mode::Automatic {
            50
        } else {
            0
        };
        let gold = &mut self.gold[r.team.index()];
        if *gold < cost {
            return Err("insufficient intervention gold");
        }
        *gold -= cost;
        Ok(())
    }
}

/// Sentry-originated or commander-requested operations, UC §5.7.2.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Action {
    /// Change posture and optionally consume its enhanced allowance.
    Posture {
        /// Requested posture.
        posture: Posture,
        /// Use the remaining enhanced budget.
        enhanced: bool,
    },
    /// Purchase ammunition; normal delivery/location rules still apply.
    BuyAmmo {
        /// Number of 17 mm rounds.
        amount: u32,
    },
    /// Purchase delayed HP recovery.
    BuyHp,
    /// Purchase immediate respawn.
    InstantRespawn,
    /// Confirm a completed normal respawn bar (§5.7.2).
    ConfirmRespawn,
    /// Forward a validated minimap marker; routing belongs to the host.
    ForwardMarker,
}
impl Match {
    /// Apply a Sentry operation atomically, including its intervention fee.
    /// Invalid actions and insufficient combined purchase/fee balances change nothing.
    pub fn sentry_action(
        &mut self,
        id: u32,
        action: Action,
        commander: bool,
    ) -> Result<(), &'static str> {
        let mut next = self.clone();
        next.sentry_intervention(id, commander)?;
        match action {
            Action::Posture { posture, enhanced } => {
                next.set_sentry_posture(id, posture, enhanced, false)?
            }
            Action::BuyAmmo { amount } => next.buy_ammo(id, amount)?,
            Action::BuyHp => next.buy_hp(id)?,
            Action::InstantRespawn => next.instant_respawn(id)?,
            Action::ConfirmRespawn => {
                let r = next.robots.iter().find(|r| r.id == id).unwrap();
                if r.hp > 0
                    || r.combat.respawn.is_none()
                    || next.respawn_remaining_ns(id) != Some(0)
                {
                    return Err("normal respawn not ready");
                }
                next.robots
                    .iter_mut()
                    .find(|r| r.id == id)
                    .unwrap()
                    .combat
                    .sentry
                    .respawn_confirmed = true;
            }
            Action::ForwardMarker => (),
        }
        *self = next;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::super::match_rules::COUNTDOWN_NS;
    use super::*;
    use crate::Team;
    fn game() -> Match {
        let mut g = Match::new(1);
        g.add_robot(7, Team::Red, GroundRole::Sentry);
        g.start(1);
        g.advance_ns(COUNTDOWN_NS);
        g
    }
    #[test]
    fn modes_lock_before_countdown_and_survive_round_reset() {
        let mut g = Match::new(1);
        g.add_robot(7, Team::Red, GroundRole::Sentry);
        assert_eq!(g.robots[0].max_hp, 400);
        assert_eq!(g.robots[0].stats(0).heat_limit, 260);
        g.set_sentry_mode(7, Mode::SemiAutomatic).unwrap();
        assert_eq!(g.robots[0].max_hp, 200);
        g.start(1);
        g.advance_ns(COUNTDOWN_NS);
        assert_eq!(g.power_limit_w(7), 90);
        assert_eq!(g.field_effects(7).cooling_per_s, 3);
        assert!(g.set_sentry_mode(7, Mode::Automatic).is_err());
        assert_eq!(g.robots[0].combat.sentry.mode, Mode::SemiAutomatic);
    }
    #[test]
    fn table_5_28_all_posture_variants_and_combined_purchase_fee() {
        for (posture, weak, enhanced, power, cooling, defense, vulnerability) in [
            (Posture::Attack, false, false, 50, 90, 0, 25),
            (Posture::Attack, true, false, 50, 60, 0, 25),
            (Posture::Attack, false, true, 50, 90, 0, 25),
            (Posture::Defense, false, false, 50, 10, 50, 0),
            (Posture::Defense, true, false, 50, 10, 25, 0),
            (Posture::Defense, false, true, 50, 10, 99, 0),
            (Posture::Movement, false, false, 150, 10, 0, 25),
            (Posture::Movement, true, false, 120, 10, 0, 25),
            (Posture::Movement, false, true, 200, 10, 0, 25),
        ] {
            let mut g = game();
            let s = &mut g.robots[0].combat.sentry;
            s.posture = posture;
            s.enhanced = enhanced;
            if weak {
                s.used_ns[posture.index()] = WEAKEN_NS + 1;
            }
            let effect = g.field_effects(7);
            assert_eq!(
                (
                    g.power_limit_w(7),
                    effect.cooling_per_s,
                    effect.defense_pct,
                    effect.vulnerability_pct
                ),
                (power, cooling, defense, vulnerability)
            );
        }
        let mut g = game();
        g.gold[0] = 199; // Remote ammunition is 150 + Automatic intervention 50.
        let before = g.clone();
        assert!(
            g.sentry_action(7, Action::BuyAmmo { amount: 100 }, true)
                .is_err()
        );
        assert_eq!(g, before);
        g.gold[0] = 200;
        g.sentry_action(7, Action::BuyAmmo { amount: 100 }, true)
            .unwrap();
        assert_eq!(g.gold[0], 0);
        assert_eq!(g.robots[0].combat.deliveries.len(), 1);
    }
    #[test]
    fn enhanced_attack_heat_cooldown_and_independent_allowances() {
        let mut g = game();
        g.set_sentry_posture(7, Posture::Attack, true, false)
            .unwrap();
        g.record_launch(7, 25.);
        assert_eq!(g.robots[0].combat.heat_tenths, 0);
        assert_eq!(g.power_limit_w(7), 50);
        assert_eq!(g.field_effects(7).vulnerability_pct, 25);
        assert!(
            g.set_sentry_posture(7, Posture::Defense, true, false)
                .is_err()
        );
        g.advance_ns(ENHANCED_NS);
        assert!(!g.robots[0].combat.sentry.enhanced);
        assert!(
            g.set_sentry_posture(7, Posture::Attack, true, false)
                .is_err()
        );
        g.set_sentry_posture(7, Posture::Defense, true, false)
            .unwrap();
        assert_eq!(g.damage_robot(7, 200, false), 2);
        assert_eq!(g.field_effects(7).cooling_per_s, 10);
    }
    #[test]
    fn weakening_is_strict_and_death_preserves_usage() {
        let mut g = game();
        g.advance_ns(WEAKEN_NS);
        assert!(!g.robots[0].combat.sentry.weakened());
        assert_eq!(g.power_limit_w(7), 150);
        g.advance_ns(1);
        assert_eq!(g.power_limit_w(7), 120);
        let usage = g.robots[0].combat.sentry.clone();
        g.damage_robot(7, u32::MAX, true);
        g.advance_ns(1_000_000_000);
        assert_eq!(g.robots[0].combat.sentry, usage);
    }
    #[test]
    fn intervention_and_purchase_are_atomic_and_partition_restore_is_exact() {
        let mut g = game();
        g.gold[0] = 49;
        let before = g.clone();
        assert!(
            g.sentry_action(
                7,
                Action::Posture {
                    posture: Posture::Defense,
                    enhanced: true
                },
                true
            )
            .is_err()
        );
        assert_eq!(g, before);
        g.gold[0] = 50;
        g.sentry_action(
            7,
            Action::Posture {
                posture: Posture::Defense,
                enhanced: true,
            },
            true,
        )
        .unwrap();
        assert_eq!(g.gold[0], 0);
        let mut split = g.clone();
        g.advance_ns(WEAKEN_NS + 1);
        for dt in [ENHANCED_NS - 1, 1, WEAKEN_NS - ENHANCED_NS, 1] {
            split.advance_ns(dt);
        }
        assert_eq!(g, split);
        assert!(g.robots[0].combat.validate(g.elapsed_ns()));
    }
}

#[cfg(test)]
mod respawn_tests {
    use super::*;
    use crate::Team;
    #[test]
    fn sentry_waits_for_valid_paid_confirmation_without_busy_advancing() {
        let mut g = Match::new(1);
        g.add_robot(7, Team::Red, GroundRole::Sentry);
        g.start(1);
        g.advance_ns(super::super::match_rules::COUNTDOWN_NS);
        g.damage_robot(7, 400, true);
        let before = g.clone();
        assert!(g.sentry_action(7, Action::ConfirmRespawn, true).is_err());
        assert_eq!(before, g);
        g.advance_ns(20_000_000_000);
        assert_eq!(g.robots[0].hp, 0);
        assert_eq!(g.respawn_remaining_ns(7), Some(0));
        let gold = g.gold[0];
        g.sentry_action(7, Action::ConfirmRespawn, true).unwrap();
        assert_eq!(g.gold[0], gold - 50);
        g.advance_ns(1);
        assert_eq!(g.robots[0].hp, 40);
    }
}
