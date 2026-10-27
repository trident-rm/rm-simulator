// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native combat integration; all clocks are running-round nanoseconds.
use super::*;
use crate::rmuc_2027::combat::{
    Delivery, EXPERIENCE_TENTHS, Respawn, SETTLEMENT_NS, SIX_SECONDS_NS,
};
const SECOND: u64 = 1_000_000_000;
const HEAL_DENOM: i64 = 200_000_000;
impl Match {
    /// Check native launch locks before creating a projectile. Idle is free
    /// practice; a running round enforces 17 mm allowance and connectivity.
    pub fn check_launch(&self, id: u32) -> std::result::Result<(), &'static str> {
        let r = self
            .robots
            .iter()
            .find(|r| r.id == id)
            .ok_or("unknown 2027 robot")?;
        if !matches!(self.phase, Phase::Idle | Phase::Running) {
            return Err("2027 launcher unavailable in this phase");
        }
        if r.hp == 0 || !r.online {
            return Err("2027 robot is defeated or offline");
        }
        if self.phase == Phase::Running
            && ((r.role == GroundRole::Heavy
                && (!r.combat.heavy_installed || !self.heavy_unlocked(r.team)))
                || r.combat.launcher_locked(self.elapsed_ns())
                || (r.combat.allowance == 0
                    && self.field_effects(id).fortress_reserve
                        < if r.role == GroundRole::Heavy { 10 } else { 1 }))
        {
            return Err("2027 launcher locked or allowance exhausted");
        }
        Ok(())
    }
    /// Record an accepted physical launch, UC §§5.1.2–5.1.3/5.4.2/5.5.1.
    /// Figure 5-1's >= normal-limit+100 boundary is the user-selected policy.
    pub fn record_launch(&mut self, id: u32, speed_m_s: f64) {
        if self.phase != Phase::Running {
            return;
        }
        let now = self.elapsed_ns();
        self.resolve();
        let reserve_cost = if self
            .robots
            .iter()
            .any(|r| r.id == id && r.role == GroundRole::Heavy)
        {
            10
        } else {
            1
        };
        let reserve = self.field_effects(id).fortress_reserve >= reserve_cost;
        let Some(r) = self.robots.iter_mut().find(|r| r.id == id) else {
            return;
        };
        let stats = r.stats(now);
        if reserve {
            self.fortress_reserve_used[r.team.index()] += reserve_cost;
        } else {
            r.combat.allowance = r.combat.allowance.saturating_sub(1);
        }
        r.combat.last_combat_ns = Some(now);
        r.combat.heat_tenths += if r.role == GroundRole::Sentry
            && r.combat.sentry.enhanced
            && r.combat.sentry.posture == super::super::sentry::Posture::Attack
        {
            0
        } else if r.role == GroundRole::Heavy {
            1000
        } else {
            100
        };
        r.combat.overheated |= r.combat.heat_tenths > u64::from(stats.heat_limit) * 10;
        r.combat.heat_locked_for_round |= r.combat.heat_tenths
            >= u64::from(
                stats.heat_limit
                    + if r.role == GroundRole::Heavy {
                        200
                    } else {
                        100
                    },
            ) * 10;
        let overspeed = speed_m_s
            - if r.role == GroundRole::Heavy {
                16.5
            } else {
                25.
            };
        if r.role == GroundRole::Heavy && overspeed > 0. {
            if speed_m_s > 18. {
                r.combat.overspeed_locked_for_round = true;
            } else {
                r.combat.overspeed_until_ns = r.combat.overspeed_until_ns.max(now + 15 * SECOND);
            }
        } else if overspeed >= 10. {
            r.combat.overspeed_locked_for_round = true;
        } else if overspeed > 0. {
            r.combat.overspeed_until_ns = r.combat.overspeed_until_ns.max(
                now + if overspeed >= 5. {
                    20 * SECOND
                } else {
                    15 * SECOND
                },
            );
        }
        if r.role == GroundRole::Heavy {
            return;
        }
        self.adaptive.launches[r.team.index()] =
            self.adaptive.launches[r.team.index()].saturating_add(1);
        self.grant_experience(id, 10);
    }
    /// Native 2027 choice using the shared choice names, never the legacy tables.
    /// Each component can be selected once while running; unchanged components
    /// do not consume or bypass the other component's selection lock.
    pub fn set_performance(
        &mut self,
        id: u32,
        chassis: crate::InfantryChassis,
        launcher: crate::InfantryLauncher,
    ) -> std::result::Result<(), &'static str> {
        let now = self.elapsed_ns();
        if !matches!(self.phase, Phase::Idle | Phase::Running) {
            return Err("performance cannot change in this phase");
        }
        let r = self
            .robots
            .iter_mut()
            .find(|r| r.id == id)
            .ok_or("unknown 2027 robot")?;
        if r.role != GroundRole::Infantry {
            return Err("only Infantry selects these performance types");
        }
        let c = &mut r.combat;
        if self.phase == Phase::Idle {
            // Explicit preparation choices remain confirmed when the round starts.
            c.chassis_locked = true;
            c.launcher_locked = true;
        }
        if self.phase == Phase::Running
            && ((c.chassis_locked && c.chassis != chassis)
                || (c.launcher_locked && c.launcher != launcher))
        {
            return Err("2027 performance selection is locked");
        }
        if self.phase == Phase::Running {
            // Confirming both choices locks both; later requests may repeat them.
            let confirm_both = c.chassis == chassis && c.launcher == launcher;
            c.chassis_locked |= c.chassis != chassis || confirm_both;
            c.launcher_locked |= c.launcher != launcher || confirm_both;
        }
        c.chassis = chassis;
        c.launcher = launcher;
        let maximum = r.stats(now).max_hp;
        if r.hp > 0 {
            r.hp =
                r.hp.saturating_add(maximum.saturating_sub(r.max_hp))
                    .min(maximum);
        }
        r.max_hp = maximum;
        let limit = r.stats(now).heat_limit;
        r.combat.overheated |= r.combat.heat_tenths > u64::from(limit) * 10;
        r.combat.heat_locked_for_round |= r.combat.heat_tenths >= u64::from(limit + 100) * 10;
        Ok(())
    }
    /// Observe own supply or vantage RFID, including defeated supply detection.
    /// These cards use the same two-second grace, not a rendered overlay flag.
    pub fn observe_service(&mut self, id: u32, supply: bool, present: bool) {
        if self.phase != Phase::Running || !present {
            return;
        }
        let expiry = self.elapsed_ns() + RFID_GRACE_NS;
        let mut first_vantage = false;
        if let Some(r) = self.robots.iter_mut().find(|r| r.id == id && r.online) {
            if supply {
                r.combat.supply_expires_ns = Some(expiry);
            } else {
                r.combat.vantage_expires_ns = Some(expiry);
                first_vantage = r.hp > 0 && !r.combat.weakened && !r.terrain.experience_awarded[3];
                r.terrain.experience_awarded[3] |= first_vantage;
            }
        }
        if first_vantage {
            self.grant_experience(id, 3000);
        }
    }
    fn outpost_service(&self, r: &Robot) -> bool {
        Team::BOTH.into_iter().any(|side| {
            r.expires_ns[point_index(side, Point::Outpost)].is_some_and(|t| t > self.elapsed_ns())
                && self.control.snapshot().territories[side.index()].control
                    == ControlState::Full(r.team)
        })
    }
    fn recovery_card(&self, r: &Robot) -> bool {
        r.in_supply(self.elapsed_ns())
            || self.outpost_service(r)
            || r.expires_ns[point_index(r.team, Point::Base)].is_some_and(|t| t > self.elapsed_ns())
    }
    /// Whether an eligible, non-weakened robot can make a zone ammunition exchange.
    pub fn zone_exchange_available(&self, id: u32) -> bool {
        self.robots.iter().find(|r| r.id == id).is_some_and(|r| {
            r.hp > 0
                && r.online
                && !r.combat.weakened
                && (self.recovery_card(r)
                    || r.combat
                        .vantage_expires_ns
                        .is_some_and(|t| t > self.elapsed_ns()))
        })
    }
    pub(super) fn resolve_service_recovery(&mut self) {
        let now = self.elapsed_ns();
        let recovering = self
            .robots
            .iter()
            .filter(|r| r.hp > 0 && r.online && r.combat.weakened && self.recovery_card(r))
            .map(|r| r.id)
            .collect::<Vec<_>>();
        for r in &mut self.robots {
            if recovering.contains(&r.id) {
                r.combat.weakened = false;
                r.combat.weakness_until_ns = None;
                r.combat.invincible_until_ns = r
                    .combat
                    .invincible_until_ns
                    .min(now.max(r.combat.minimum_invincible_until_ns));
            }
            if r.combat.weakness_until_ns.is_some_and(|t| now >= t) {
                r.combat.weakened = false;
                r.combat.weakness_until_ns = None;
            }
        }
    }
    /// Purchase 17 mm allowance under Tables 5-7–5-9. Zone purchases are immediate
    /// multiples of ten; remote purchases require out-of-combat and multiples
    /// of 100, delivered six seconds later. Failed requests do not debit gold.
    pub fn buy_ammo(&mut self, id: u32, rounds: u32) -> std::result::Result<(), &'static str> {
        self.resolve_service_recovery();
        let now = self.elapsed_ns();
        if self.phase != Phase::Running {
            return Err("2027 purchases require a running round");
        }
        let r = self
            .robots
            .iter()
            .find(|r| r.id == id)
            .ok_or("unknown 2027 robot")?;
        if r.hp == 0 || !r.online || (r.role == GroundRole::Heavy && !r.combat.heavy_installed) {
            return Err("2027 robot cannot exchange ammunition");
        }
        let zone = self.zone_exchange_available(id);
        let heavy = r.role == GroundRole::Heavy;
        let unit = if heavy {
            if zone { 1 } else { 10 }
        } else if zone {
            10
        } else {
            100
        };
        if rounds == 0
            || rounds > 1000
            || !rounds.is_multiple_of(unit)
            || (!zone && !r.out_of_combat(now))
        {
            return Err("invalid 2027 exchange amount or robot is in combat");
        }
        let cost = rounds / unit * if zone { 10 } else { 150 };
        let team = r.team.index();
        let purchased = if heavy {
            &mut self.purchased_heavy_rounds[team]
        } else {
            &mut self.purchased_rounds[team]
        };
        let cap = if heavy { 100 } else { 1000 };
        if rounds > cap - *purchased || cost > self.gold[team] {
            return Err("2027 purchase cap or insufficient gold");
        }
        self.gold[team] -= cost;
        *purchased += rounds;
        let r = self.robots.iter_mut().find(|r| r.id == id).unwrap();
        if zone {
            r.combat.allowance = r.combat.allowance.saturating_add(rounds);
        } else {
            r.combat.deliveries.push(Delivery {
                due_ns: now + SIX_SECONDS_NS,
                rounds,
            });
        }
        Ok(())
    }
    /// Pay for remote HP recovery (§5.3.1/Table 5-7), delivered after six seconds.
    pub fn buy_hp(&mut self, id: u32) -> std::result::Result<(), &'static str> {
        let now = self.elapsed_ns();
        let r = self
            .robots
            .iter()
            .find(|r| r.id == id)
            .ok_or("unknown 2027 robot")?;
        if self.phase != Phase::Running || !r.out_of_combat(now) {
            return Err("remote HP requires a live out-of-combat robot");
        }
        let cost = 50 + (now * 20).div_ceil(60 * SECOND) as u32;
        let team = r.team.index();
        if self.gold[team] < cost {
            return Err("insufficient 2027 gold");
        }
        self.gold[team] -= cost;
        self.robots
            .iter_mut()
            .find(|r| r.id == id)
            .unwrap()
            .combat
            .deliveries
            .push(Delivery {
                due_ns: now + SIX_SECONDS_NS,
                rounds: 0,
            });
        Ok(())
    }
    /// Paid respawn preserves XP and allowance, restores full HP, and applies
    /// the independent 3 s weakness/invincibility and 4 s reported power boost.
    pub fn instant_respawn(&mut self, id: u32) -> std::result::Result<(), &'static str> {
        let now = self.elapsed_ns();
        let r = self
            .robots
            .iter()
            .find(|r| r.id == id)
            .ok_or("unknown 2027 robot")?;
        if self.phase != Phase::Running || r.hp > 0 || !r.online {
            return Err("2027 paid respawn requires a defeated connected robot");
        }
        let cost = now.div_ceil(60 * SECOND) as u32 * 80 + u32::from(r.combat.level) * 20;
        let team = r.team.index();
        if self.gold[team] < cost {
            return Err("insufficient 2027 gold");
        }
        self.gold[team] -= cost;
        let r = self.robots.iter_mut().find(|r| r.id == id).unwrap();
        self.radar.respawn(id);
        r.hp = r.max_hp;
        r.combat.respawn = None;
        r.combat.paid_respawns = r.combat.paid_respawns.saturating_add(1);
        r.combat.weakened = true;
        r.combat.weakness_until_ns = Some(now + 3 * SECOND);
        r.combat.invincible_until_ns = now + 3 * SECOND;
        r.combat.minimum_invincible_until_ns = now + 3 * SECOND;
        r.combat.double_power_until_ns = now + 4 * SECOND;
        r.combat.last_combat_ns = Some(now);
        Ok(())
    }
    /// Apply actual robot HP loss. Operator penalties bypass invincibility;
    /// ordinary projectile/collision damage respects it, field defense and connectivity.
    pub fn damage_robot(&mut self, id: u32, amount: u32, penalty: bool) -> u32 {
        let victim = self.robots.iter().find(|r| r.id == id).map(|r| r.team);
        let applied = self.apply_robot_damage(id, amount, penalty);
        if penalty && let Some(team) = victim {
            self.reward_attack(crate::Target::Robot(id), None, team, applied, None);
        }
        applied
    }

    // HP bookkeeping without reward routing, also used for administrative overrides.
    fn apply_robot_damage(&mut self, id: u32, amount: u32, penalty: bool) -> u32 {
        let now = self.elapsed_ns();
        self.resolve();
        let effects = self.field_effects(id);
        let amount = if penalty {
            amount
        } else {
            // UC §5.6.4.1: subtract strongest defense from vulnerability,
            // then round the final HP loss to the nearest integer.
            ((u64::from(amount) * u64::from(100 + effects.vulnerability_pct - effects.defense_pct)
                + 50)
                / 100)
                .min(u64::from(u32::MAX)) as u32
        };
        let Some(r) = self.robots.iter_mut().find(|r| r.id == id) else {
            return 0;
        };
        if !penalty && (!r.online || now < r.combat.invincible_until_ns) {
            return 0;
        }
        let applied = amount.min(r.hp);
        r.hp -= applied;
        if applied > 0 {
            r.combat.last_combat_ns = Some(now);
        }
        if r.hp == 0 && applied > 0 {
            r.combat.sentry.respawn_confirmed = false;
            r.combat.power.buffer_uj = super::super::power::BUFFER_UJ;
            r.combat.heat_tenths = 0;
            r.combat.overheated = false;
            r.combat.weakened = false;
            r.combat.weakness_until_ns = None;
            r.combat.invincible_until_ns = 0;
            r.combat.minimum_invincible_until_ns = 0;
            r.combat.double_power_until_ns = 0;
            r.combat.deliveries.retain(|d| d.rounds > 0);
            r.combat.healing_carry = 0;
            r.combat.cooling_half_carry = 0;
            r.terrain.clear_on_death();
            let progress_s =
                10 + (now + 5 * SECOND) / (10 * SECOND) + 20 * u64::from(r.combat.paid_respawns);
            r.combat.respawn = Some(Respawn {
                original_ns: progress_s.saturating_mul(SECOND),
                progress_ns: 0,
            });
            r.expires_ns = [None; 8];
        }
        self.resolve();
        applied
    }
    /// UC Table 5-2 armor collision: 2 HP, subject to invincibility, with
    /// unattributed non-projectile XP distributed under Table 5-14.
    pub fn collision(&mut self, id: u32) -> u32 {
        let Some(team) = self.robots.iter().find(|r| r.id == id).map(|r| r.team) else {
            return 0;
        };
        let applied = self.damage_robot(id, 2, false);
        self.reward_attack(crate::Target::Robot(id), None, team, applied, None);
        applied
    }
    /// Operator HP override, preserving normal death bookkeeping but granting no XP.
    pub fn set_robot_hp(&mut self, id: u32, hp: u32) -> std::result::Result<(), &'static str> {
        self.robots
            .iter()
            .find(|r| r.id == id)
            .ok_or("unknown 2027 robot")?;
        if hp == 0 {
            self.apply_robot_damage(id, u32::MAX, true);
        } else {
            let r = self.robots.iter_mut().find(|r| r.id == id).unwrap();
            if r.hp == 0 {
                self.radar.respawn(id);
            }
            r.hp = hp.min(r.max_hp);
            r.combat.respawn = None;
            r.combat.weakened = false;
            r.combat.weakness_until_ns = None;
            r.combat.invincible_until_ns = 0;
            r.combat.minimum_invincible_until_ns = 0;
            r.expires_ns = [None; 8];
        }
        Ok(())
    }
    /// Add XP in tenths; Infantry stops acquiring XP at the assembly-controlled
    /// cap. Level increases immediately add the increase in maximum HP (§5.5.2).
    pub fn grant_experience(&mut self, id: u32, amount_tenths: u64) {
        let now = self.elapsed_ns();
        let Some(r) = self
            .robots
            .iter_mut()
            .find(|r| r.id == id && r.role == GroundRole::Infantry)
        else {
            return;
        };
        let cap = self.level_caps[r.team.index()];
        if r.combat.level >= cap {
            return;
        }
        let available = super::super::combat::EXPERIENCE_TENTHS[usize::from(cap - 1)]
            .saturating_sub(r.combat.experience_tenths);
        let base = amount_tenths.min(available);
        let bonus = self.runes[r.team.index()]
            .buff
            .as_ref()
            .filter(|b| b.source == super::super::rune::Stage::Small && b.until_ns > now)
            .map_or(0, |b| {
                amount_tenths
                    .min(u64::from(12000 - b.extra_xp_tenths))
                    .min(available - base)
            });
        if let Some(buff) = &mut self.runes[r.team.index()].buff {
            buff.extra_xp_tenths += bonus as u32;
        }
        r.combat.experience_tenths = r
            .combat
            .experience_tenths
            .saturating_add(base + bonus)
            .min(EXPERIENCE_TENTHS[usize::from(cap - 1)]);
        r.combat.level = (EXPERIENCE_TENTHS
            .iter()
            .take(usize::from(cap))
            .filter(|&&t| t <= r.combat.experience_tenths)
            .count() as u8)
            .max(1);
        let maximum = r.stats(now).max_hp;
        if r.hp > 0 {
            r.hp += maximum.saturating_sub(r.max_hp);
        }
        r.max_hp = maximum;
    }
    pub(super) fn reward_attack(
        &mut self,
        target: crate::Target,
        shooter: Option<u32>,
        victim: Team,
        applied: u32,
        caliber: Option<crate::Caliber>,
    ) {
        if applied == 0 {
            return;
        }
        let eligible = |r: &&Robot| {
            r.team != victim
                && r.role == GroundRole::Infantry
                && r.hp > 0
                && r.combat.level < self.level_caps[r.team.index()]
                // Table 5-14: Infantry cannot qualify for fallback 42 mm XP.
                && caliber != Some(crate::Caliber::Mm42)
        };
        let killer =
            shooter.and_then(|id| self.robots.iter().filter(eligible).find(|r| r.id == id));
        let recipients = if let Some(r) = killer {
            vec![r.id]
        } else {
            self.robots
                .iter()
                .filter(eligible)
                .map(|r| r.id)
                .collect::<Vec<_>>()
        };
        if recipients.is_empty() {
            return;
        }
        let damage_xp = match target {
            crate::Target::Robot(_) => applied * 4,
            crate::Target::Outpost(_) => applied * 2,
            crate::Target::Base(_) => applied.div_ceil(2),
        };
        let kill = if let crate::Target::Robot(id) = target {
            self.robots
                .iter()
                .find(|r| r.id == id && r.hp == 0)
                .map(|r| r.combat.level)
        } else {
            None
        };
        let killer_level = if let Some(r) = killer {
            r.combat.level
        } else {
            let mean_xp = self
                .robots
                .iter()
                .filter(|r| recipients.contains(&r.id))
                .map(|r| r.combat.experience_tenths)
                .sum::<u64>()
                / recipients.len() as u64;
            EXPERIENCE_TENTHS.iter().filter(|&&v| v <= mean_xp).count() as u8
        };
        let kill_xp = kill.map_or(0, |level| {
            50 * u32::from(level)
                + 10 * u32::from(level) * u32::from(level.saturating_sub(killer_level))
        });
        let award = (u64::from(damage_xp + kill_xp) * 10 + recipients.len() as u64 / 2)
            / recipients.len() as u64;
        for id in recipients {
            self.grant_experience(id, award);
        }
    }
    /// Remaining normal-respawn time at current supply/base conditions, in ns.
    /// None means no pending respawn or an offline robot whose progress is paused.
    /// This projection never advances the authoritative timer.
    pub fn respawn_remaining_ns(&self, id: u32) -> Option<u64> {
        let r = self.robots.iter().find(|r| r.id == id && r.online)?;
        let p = r.combat.respawn.as_ref()?;
        let speed = if r.in_supply(self.elapsed_ns()) || self.base_hp[r.team.index()] < 4000 {
            4
        } else {
            1
        };
        Some(
            self.respawn_required_ns(r)
                .saturating_sub(p.progress_ns)
                .div_ceil(speed),
        )
    }
    fn respawn_required_ns(&self, r: &Robot) -> u64 {
        r.combat.respawn.as_ref().map_or(0, |p| {
            (u128::from(p.original_ns) * 20000)
                .div_ceil(u128::from(10000 + self.base_hp[r.team.index()]))
                .min(u128::from(u64::MAX)) as u64
        })
    }
    pub(super) fn combat_boundary_ns(&self) -> u64 {
        let now = self.elapsed_ns();
        let mut dt = SETTLEMENT_NS - now % SETTLEMENT_NS;
        for r in &self.robots {
            for deadline in r
                .combat
                .deliveries
                .iter()
                .map(|d| d.due_ns)
                .chain(r.combat.supply_expires_ns)
                .chain(r.combat.vantage_expires_ns)
                .chain(r.combat.weakness_until_ns)
                .chain(r.combat.last_combat_ns.map(|t| t + SIX_SECONDS_NS))
            {
                if deadline > now {
                    dt = dt.min(deadline - now);
                }
            }
            if r.hp == 0
                && r.online
                && let Some(p) = &r.combat.respawn
            {
                let speed = if r.in_supply(now) || self.base_hp[r.team.index()] < 4000 {
                    4
                } else {
                    1
                };
                if r.role == GroundRole::Sentry
                    && !r.combat.sentry.respawn_confirmed
                    && p.progress_ns >= self.respawn_required_ns(r)
                {
                    continue;
                }
                dt = dt.min(
                    self.respawn_required_ns(r)
                        .saturating_sub(p.progress_ns)
                        .div_ceil(speed)
                        .max(1),
                );
            }
        }
        dt
    }
    pub(super) fn advance_combat(&mut self, dt: u64) {
        let now = self.elapsed_ns();
        let before = now - dt;
        let mut respawning = vec![];
        for r in &self.robots {
            if r.hp == 0
                && r.online
                && let Some(p) = &r.combat.respawn
            {
                let speed = if r.in_supply(before) || self.base_hp[r.team.index()] < 4000 {
                    4
                } else {
                    1
                };
                if p.progress_ns.saturating_add(dt * speed) >= self.respawn_required_ns(r)
                    && (r.role != GroundRole::Sentry || r.combat.sentry.respawn_confirmed)
                {
                    respawning.push(r.id);
                }
            }
        }
        for r in &mut self.robots {
            if r.hp == 0
                && r.online
                && let Some(p) = &mut r.combat.respawn
            {
                let speed = if r.combat.supply_expires_ns.is_some_and(|t| t > before)
                    || self.base_hp[r.team.index()] < 4000
                {
                    4
                } else {
                    1
                };
                p.progress_ns = p.progress_ns.saturating_add(dt * speed);
            }
            if respawning.contains(&r.id) {
                self.radar.respawn(r.id);
                r.hp = (r.max_hp + 5) / 10;
                r.combat.respawn = None;
                r.combat.weakened = true;
                r.combat.weakness_until_ns = None;
                r.combat.invincible_until_ns = now + 30 * SECOND;
                r.combat.minimum_invincible_until_ns = now + 10 * SECOND;
                r.combat.last_combat_ns = Some(now);
            }
            let mut pending = vec![];
            for d in r.combat.deliveries.drain(..) {
                if d.due_ns > now {
                    pending.push(d);
                } else if d.rounds > 0 {
                    r.combat.allowance = r.combat.allowance.saturating_add(d.rounds);
                } else if r.hp > 0 && r.online {
                    let restored = (u64::from(r.max_hp)
                        * 60
                        * u64::from(10000 + self.base_hp[r.team.index()])
                        + 1_000_000)
                        / 2_000_000;
                    r.hp = r.max_hp.min(r.hp.saturating_add(restored as u32));
                }
            }
            r.combat.deliveries = pending;
        }
        self.resolve_service_recovery();
        self.resolve();
        if now.is_multiple_of(SETTLEMENT_NS) {
            let outpost = self
                .robots
                .iter()
                .map(|r| {
                    (
                        self.outpost_service(r),
                        self.field_effects(r.id).cooling_half_per_s,
                    )
                })
                .collect::<Vec<_>>();
            for (r, (outpost, cooling)) in self.robots.iter_mut().zip(outpost) {
                let cooling = cooling + u32::from(r.combat.cooling_half_carry);
                r.combat.cooling_half_carry = (cooling % 2) as u8;
                r.combat.heat_tenths = r.combat.heat_tenths.saturating_sub(u64::from(cooling / 2));
                if r.combat.heat_tenths == 0 {
                    r.combat.cooling_half_carry = 0;
                    r.combat.overheated = false;
                }
                if r.hp > 0 && r.online && !r.combat.weakened && (r.in_supply(now) || outpost) {
                    let mut rate = if now >= 240 * SECOND && r.out_of_combat(now) {
                        250
                    } else {
                        100
                    };
                    if !r.in_supply(now) {
                        rate /= 2;
                    }
                    let numerator = i64::from(r.max_hp)
                        * rate
                        * i64::from(10000 + self.base_hp[r.team.index()]);
                    r.combat.healing_carry += numerator;
                    let hp = (r.combat.healing_carry + HEAL_DENOM / 2) / HEAL_DENOM;
                    r.combat.healing_carry -= hp * HEAL_DENOM;
                    r.hp = r.max_hp.min(r.hp.saturating_add(hp as u32));
                    if r.hp == r.max_hp {
                        r.combat.healing_carry = 0;
                    }
                }
                if r.role == GroundRole::Sentry
                    && r.hp > 0
                    && r.online
                    && !r.combat.weakened
                    && r.in_supply(now)
                {
                    let minutes = (now / (60 * SECOND)) as u32;
                    r.combat.allowance = r
                        .combat
                        .allowance
                        .saturating_add((minutes - r.combat.sentry_collected_minutes) * 100);
                    r.combat.sentry_collected_minutes = minutes;
                }
            }
        }
        if now.is_multiple_of(60 * SECOND) {
            self.adaptive.opportunities = self.adaptive.opportunities.map(|n| n + 1);
            if now < ROUND_NS {
                for gold in &mut self.gold {
                    *gold = gold.saturating_add(if now == 360 * SECOND { 300 } else { 100 });
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Caliber, InfantryChassis as Chassis, InfantryLauncher as Launcher, Target};
    fn running() -> Match {
        let mut g = Match::new(1);
        g.add_robot(1, Team::Red, GroundRole::Infantry);
        g.add_robot(2, Team::Blue, GroundRole::Infantry);
        g.start(1);
        g.advance_ns(COUNTDOWN_NS);
        g
    }
    #[test]
    fn allowance_purchases_are_paid_delayed_and_launches_feed_stats_and_xp() {
        let mut g = running();
        assert_eq!(g.gold, [800; 2]);
        assert!(g.check_launch(1).is_err());
        assert!(g.buy_ammo(1, 10).is_err());
        let before = g.clone();
        assert!(g.buy_ammo(1, u32::MAX).is_err());
        assert_eq!(g, before);
        g.buy_ammo(1, 100).unwrap();
        assert_eq!(g.gold[0], 650);
        g.advance_ns(SIX_SECONDS_NS - 1);
        assert_eq!(g.robots[0].combat.allowance, 0);
        g.advance_ns(1);
        g.check_launch(1).unwrap();
        g.record_launch(1, 25.);
        assert_eq!(g.robots[0].combat.allowance, 99);
        assert_eq!(g.robots[0].combat.heat_tenths, 100);
        assert_eq!(g.robots[0].combat.experience_tenths, 10);
        assert_eq!(g.adaptive.launches, [1, 0]);
        assert!(!g.robots[0].out_of_combat(g.elapsed_ns()));
    }
    #[test]
    fn heat_lock_uses_inclusive_permanent_boundary_and_video_cools_to_zero() {
        let mut g = running();
        g.robots[0].combat.allowance = 100;
        for _ in 0..4 {
            g.check_launch(1).unwrap();
            g.record_launch(1, 25.);
        }
        assert!(!g.robots[0].combat.overheated);
        g.check_launch(1).unwrap();
        g.record_launch(1, 25.);
        assert!(g.robots[0].combat.video_cut());
        assert!(g.check_launch(1).is_err());
        g.advance_ns(4_100_000_000);
        assert!(g.robots[0].combat.overheated);
        g.advance_ns(SETTLEMENT_NS);
        assert!(!g.robots[0].combat.overheated);
        g.check_launch(1).unwrap();
        g.robots[0].combat.heat_tenths = 1299;
        g.record_launch(1, 25.);
        assert!(!g.robots[0].combat.heat_locked_for_round);
        g.robots[0].combat.heat_tenths = 1300;
        g.record_launch(1, 25.);
        assert!(g.robots[0].combat.heat_locked_for_round);
        g.advance_ns(20 * SECOND);
        assert!(!g.robots[0].combat.video_cut());
        assert!(g.check_launch(1).is_err());
        g.damage_robot(1, 500, true);
        assert!(g.robots[0].combat.heat_locked_for_round);
    }
    #[test]
    fn overspeed_and_module_lock_causes_do_not_clear_each_other() {
        let mut g = running();
        g.robots[0].combat.allowance = 100;
        g.record_launch(1, 29.999);
        assert_eq!(g.robots[0].combat.overspeed_until_ns, 15 * SECOND);
        g.advance_ns(15 * SECOND);
        g.check_launch(1).unwrap();
        g.record_launch(1, 30.);
        assert_eq!(g.robots[0].combat.overspeed_until_ns, 35 * SECOND);
        g.robots[0].combat.speed_module_online = false;
        g.robots[0].combat.power.modules.speed_online = false;
        g.advance_ns(20 * SECOND);
        assert!(g.check_launch(1).is_err());
        g.robots[0].combat.speed_module_online = true;
        g.robots[0].combat.power.modules.speed_online = true;
        g.check_launch(1).unwrap();
        g.record_launch(1, 35.);
        assert!(g.robots[0].combat.overspeed_locked_for_round);
        g.robots[0].combat.speed_module_online = false;
        g.robots[0].combat.power.modules.speed_online = false;
        g.robots[0].combat.speed_module_online = true;
        g.robots[0].combat.power.modules.speed_online = true;
        g.advance_ns(40 * SECOND);
        assert!(g.check_launch(1).is_err());
    }
    #[test]
    fn normal_respawn_and_service_recovery_preserve_ten_second_invincibility() {
        let mut g = running();
        g.damage_robot(1, 500, true);
        assert_eq!(
            g.robots[0].combat.respawn.as_ref().unwrap().original_ns,
            10 * SECOND
        );
        assert_eq!(g.respawn_remaining_ns(1), Some(10 * SECOND));
        g.advance_ns(10 * SECOND - 1);
        assert_eq!(g.robots[0].hp, 0);
        g.advance_ns(1);
        assert_eq!(g.robots[0].hp, 20);
        assert!(g.robots[0].combat.weakened);
        assert_eq!(g.damage_robot(1, 20, false), 0);
        g.observe(1, Team::Red, Point::Base, true);
        g.advance_ns(1);
        assert!(!g.robots[0].combat.weakened);
        assert_eq!(g.robots[0].combat.invincible_until_ns, 20 * SECOND);
        g.advance_ns(10 * SECOND - 1);
        assert_eq!(g.damage_robot(1, 20, false), 20);
    }
    #[test]
    fn supply_accelerates_dead_progress_and_offline_suspends_it() {
        let mut g = running();
        g.damage_robot(1, 500, true);
        g.robots[0].online = false;
        g.robots[0].combat.power.modules.main_online = false;
        g.advance_ns(SECOND);
        assert_eq!(g.robots[0].combat.respawn.as_ref().unwrap().progress_ns, 0);
        g.robots[0].online = true;
        g.robots[0].combat.power.modules.main_online = true;
        for _ in 0..25 {
            g.observe_service(1, true, true);
            g.advance_ns(SETTLEMENT_NS);
        }
        assert!(g.robots[0].hp >= 20);
        assert!(!g.robots[0].combat.weakened);
        assert_eq!(
            g.robots[0].combat.invincible_until_ns,
            g.elapsed_ns() + 10 * SECOND
        );
    }
    #[test]
    fn paid_respawn_costs_gold_preserves_xp_and_sets_independent_timers() {
        let mut g = running();
        g.grant_experience(1, 1200);
        g.robots[0].combat.allowance = 55;
        g.damage_robot(1, 500, true);
        g.instant_respawn(1).unwrap();
        assert_eq!(g.gold[0], 780);
        assert_eq!(g.robots[0].hp, 200);
        assert_eq!(g.robots[0].combat.allowance, 55);
        assert_eq!(g.robots[0].combat.experience_tenths, 1200);
        assert_eq!(g.robots[0].stats(0).chassis_power_w, 90);
        g.advance_ns(3 * SECOND);
        assert!(!g.robots[0].combat.weakened);
        g.advance_ns(SECOND);
        assert_eq!(g.robots[0].stats(g.elapsed_ns()).chassis_power_w, 45);
        g.damage_robot(1, 500, true);
        assert_eq!(
            g.robots[0].combat.respawn.as_ref().unwrap().original_ns,
            30 * SECOND
        );
    }
    #[test]
    fn base_logistics_scales_healing_once_and_changes_respawn_requirement() {
        let mut g = running();
        g.base_hp[0] = 5000;
        g.set_robot_hp(1, 100).unwrap();
        g.observe_service(1, true, true);
        g.advance_ns(SECOND);
        assert_eq!(g.robots[0].hp, 115);
        g.robots[0].combat.supply_expires_ns = None;
        g.observe(1, Team::Red, Point::Outpost, true);
        g.advance_ns(SECOND);
        assert_eq!(g.robots[0].hp, 123);
        g.damage_robot(1, 500, true);
        let original = g.robots[0].combat.respawn.as_ref().unwrap().original_ns;
        assert_eq!(
            g.respawn_required_ns(&g.robots[0]),
            (original * 4).div_ceil(3)
        );
    }
    #[test]
    fn remote_hp_dies_without_refund_while_ammunition_delivery_survives() {
        let mut g = running();
        g.set_robot_hp(1, 100).unwrap();
        g.buy_hp(1).unwrap();
        g.buy_ammo(1, 100).unwrap();
        assert_eq!(g.gold[0], 600);
        g.damage_robot(1, 500, true);
        assert_eq!(g.robots[0].combat.deliveries.len(), 1);
        g.advance_ns(6 * SECOND);
        assert_eq!(g.robots[0].combat.allowance, 100);
        assert_eq!(g.robots[0].hp, 0);
        assert_eq!(g.gold[0], 600);
    }
    #[test]
    fn combat_xp_level_hp_and_independent_performance_choices_use_2027_tables() {
        let mut g = running();
        g.damage_robot(1, 20, true);
        g.grant_experience(1, 5500);
        assert_eq!(
            (g.robots[0].combat.level, g.robots[0].hp, g.robots[0].max_hp),
            (2, 205, 225)
        );
        g.set_performance(1, Chassis::HpFocused, Launcher::BurstFocused)
            .unwrap();
        assert!(!g.robots[0].combat.chassis_locked);
        assert!(g.robots[0].combat.launcher_locked);
        g.set_performance(1, Chassis::PowerFocused, Launcher::BurstFocused)
            .unwrap();
        assert_eq!(g.robots[0].max_hp, 175);
        assert!(
            g.set_performance(1, Chassis::HpFocused, Launcher::BurstFocused)
                .is_err()
        );
        g.grant_experience(1, 1_000_000);
        assert_eq!(g.robots[0].combat.level, 5);
        assert_eq!(g.robots[0].combat.experience_tenths, 22000);
        assert_eq!(g.robots[0].stats(0).cooling_per_s, 12);
    }
    #[test]
    fn lethal_damage_rewards_and_posthumous_projectiles_are_not_discarded() {
        let mut g = running();
        g.set_robot_hp(2, 20).unwrap();
        assert_eq!(g.hit(Target::Robot(2), Some(1), Caliber::Mm17), 20);
        assert_eq!(g.robots[0].combat.experience_tenths, 1300);
        g.damage_robot(1, 500, true);
        g.set_robot_hp(2, 200).unwrap();
        assert_eq!(g.hit(Target::Robot(2), Some(1), Caliber::Mm17), 20);
    }
    #[test]
    fn state_restore_partitioning_covers_expiry_settlement_deliveries_and_respawn() {
        let mut a = running();
        a.buy_ammo(1, 100).unwrap();
        a.damage_robot(1, 500, true);
        a.observe_service(1, true, true);
        a.set_robot_hp(2, 100).unwrap();
        a.observe_service(2, true, true);
        let mut b: Match = serde_json::from_slice(&serde_json::to_vec(&a).unwrap()).unwrap();
        a.advance_ns(70 * SECOND);
        let mut remaining = 70 * SECOND;
        let sizes = [1, 27_000_003, 801_234_567, 9_000_000_000];
        let mut i = 0;
        while remaining > 0 {
            let dt = remaining.min(sizes[i % sizes.len()]);
            b.advance_ns(dt);
            remaining -= dt;
            i += 1;
        }
        assert_eq!(a, b);
        a.validate().unwrap();
        assert_eq!(a.adaptive.opportunities, [1; 2]);
        assert_eq!(a.gold, [750, 900]);
    }
    #[test]
    fn zone_cap_and_remote_hp_logistics_do_not_share_purchase_counters() {
        let mut g = running();
        g.gold[0] = 2000;
        g.observe(1, Team::Red, Point::Base, true);
        g.buy_ammo(1, 1000).unwrap();
        let gold = g.gold[0];
        assert!(g.buy_ammo(1, 10).is_err());
        assert_eq!(g.gold[0], gold);
        assert_eq!(g.robots[0].combat.allowance, 1000);
        g.set_robot_hp(1, 10).unwrap();
        g.base_hp[0] = 5000;
        g.buy_hp(1).unwrap();
        g.advance_ns(SIX_SECONDS_NS);
        assert_eq!(g.robots[0].hp, 100);
        assert_eq!(g.purchased_rounds[0], 1000);
    }
    #[test]
    fn late_supply_bonus_needs_six_live_seconds_without_combat() {
        let mut g = running();
        g.advance_ns(240 * SECOND);
        g.damage_robot(1, 100, true);
        for _ in 0..10 {
            g.observe_service(1, true, true);
            g.advance_ns(SETTLEMENT_NS);
        }
        assert_eq!(g.robots[0].hp, 120);
        g.robots[0].combat.supply_expires_ns = None;
        g.advance_ns(5 * SECOND);
        g.set_robot_hp(1, 100).unwrap();
        for _ in 0..10 {
            g.observe_service(1, true, true);
            g.advance_ns(SETTLEMENT_NS);
        }
        assert_eq!(g.robots[0].hp, 150);
        g.validate().unwrap();
        g.robots[0].combat.sentry_collected_minutes = 100;
        assert!(g.validate().is_err());
    }
    #[test]
    fn finished_round_telemetry_and_operator_hp_do_not_reenter_control() {
        let mut g = running();
        g.advance_ns(ROUND_NS);
        let result = g.result.clone();
        assert!(g.activations().iter().all(|(_, a)| *a == [false; 8]));
        g.set_robot_hp(1, 0).unwrap();
        g.set_robot_hp(1, 200).unwrap();
        assert_eq!(g.result, result);
        g.validate().unwrap();
    }
    #[test]
    fn sentry_grants_accumulate_until_supply_collection() {
        let mut g = running();
        g.add_robot(3, Team::Red, GroundRole::Sentry);
        assert_eq!(g.robots[2].combat.allowance, 300);
        g.advance_ns(3 * 60 * SECOND);
        g.observe_service(3, true, true);
        g.advance_ns(SETTLEMENT_NS);
        assert_eq!(g.robots[2].combat.allowance, 600);
        g.advance_ns(SETTLEMENT_NS);
        assert_eq!(g.robots[2].combat.allowance, 600);
    }
}
