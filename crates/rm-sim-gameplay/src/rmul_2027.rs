// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Standalone RMUL 2027 V1.0.0 authority (UL chapters 3–4).
//!
//! This module shares only [`crate::Team`] with other competition engines.
//! Callers resolve physical hits and RFID contacts; all timers use explicit ns.
//! See `docs/rmul-2027.md` for source clauses and settlement policies.
use crate::Team;
use serde::{Deserialize, Serialize};

const SECOND: u64 = 1_000_000_000;
const PERIOD: u64 = SECOND / 10;
const GRACE: u64 = 2 * SECOND;

/// Distinct competitions, UL chapters 3 and 4.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Format {
    /// Five-minute three-robot control match.
    ThreeVsThree,
    /// Two-minute Infantry duel.
    Infantry,
}
/// Competition and manual revision recorded in every checkpoint.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Ruleset {
    /// RMUL 3v3 2027 V1.0.0.
    Rmul2027V1,
    /// RMUL Infantry 2027 V1.0.0.
    RmulInfantry2027V1,
}
impl Format {
    /// Manual identity; neither identity admits UC state.
    pub const fn ruleset(self) -> Ruleset {
        match self {
            Self::ThreeVsThree => Ruleset::Rmul2027V1,
            Self::Infantry => Ruleset::RmulInfantry2027V1,
        }
    }
    /// Running duration in ns, UL §§3.3.7 and 4.4.
    pub const fn duration_ns(self) -> u64 {
        match self {
            Self::ThreeVsThree => 300 * SECOND,
            Self::Infantry => 120 * SECOND,
        }
    }
}
/// Native UL ground role, UL Tables 3-1 and 4-1.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Role {
    /// Heavy 1, initially without installed launcher properties.
    Heavy,
    /// Infantry 3.
    Infantry,
    /// Autonomous Sentry 7 (3v3 only).
    Sentry,
}
/// Round lifecycle; only Running advances rules.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Phase {
    /// Roster/configuration may change.
    Idle,
    /// Five-second launch countdown, UL §5.5.
    Countdown,
    /// Active round.
    Running,
    /// Frozen result.
    Finished,
}
/// UL §3.3.4's independent zones, in checkpoint slot order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Zone {
    /// Red-side highland.
    RedHighland,
    /// Blue-side highland.
    BlueHighland,
    /// Central control zone.
    Center,
}
impl Zone {
    /// All zones in state-array order.
    pub const ALL: [Self; 3] = [Self::RedHighland, Self::BlueHighland, Self::Center];
    /// Stable array slot.
    pub const fn index(self) -> usize {
        match self {
            Self::RedHighland => 0,
            Self::BlueHighland => 1,
            Self::Center => 2,
        }
    }
}
/// Why a single round ended; series administration remains caller-owned.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ResultReason {
    /// One team's control reached zero.
    ControlDepleted,
    /// An Infantry duel robot was defeated.
    Defeat,
    /// Time expired or a referee requested finish; compare UL tie breakers.
    Comparison,
}
/// Frozen single-round outcome.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RoundResult {
    /// Winner, or None for a draw.
    pub winner: Option<Team>,
    /// Resolution cause.
    pub reason: ResultReason,
    /// Running-round timestamp in ns.
    pub elapsed_ns: u64,
}
/// Fixed performance from UL Tables 3-2–3-5 and 4-2.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Stats {
    /// Maximum HP.
    pub max_hp: u32,
    /// Chassis power limit in W.
    pub chassis_power_w: u32,
    /// Normal launcher heat limit in whole heat units.
    pub heat_limit: u32,
    /// Heat cooling per second.
    pub cooling_per_s: u32,
    /// Maximum launch speed in m/s (zero for uninstalled Heavy).
    pub max_speed_m_s: f64,
}
/// Complete native robot checkpoint; hidden timers serialize with visible HP.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Robot {
    /// Caller-owned unique identity, not the manual's addressed role number.
    pub id: u32,
    /// Team ownership.
    pub team: Team,
    /// Native UL role.
    pub role: Role,
    /// Current whole HP, at most max_hp.
    pub hp: u32,
    /// Current maximum HP.
    pub max_hp: u32,
    /// Remaining allowed rounds; caliber follows role.
    pub allowance: u32,
    /// Heat in tenths of a unit.
    pub heat_tenths: u32,
    /// Heavy installation has been confirmed by one detected launch (§3.3.6).
    pub heavy_upgraded: bool,
    /// Physically installed launcher, caller verifies workbench admission.
    pub heavy_installed: bool,
    /// 3v3 Infantry power chassis instead of default HP chassis.
    pub power_chassis: bool,
    /// 3v3 Infantry burst launcher instead of default cooling launcher.
    pub burst_launcher: bool,
    /// Launcher locks until heat reaches zero; also cuts first-person video.
    pub overheated: bool,
    /// Permanent heat lock (Figure 3-12 inclusive boundary).
    pub heat_locked_for_round: bool,
    /// Permanent overspeed lock (Table 3-9).
    pub overspeed_locked_for_round: bool,
    /// Temporary overspeed lock deadline in round ns.
    pub overspeed_until_ns: u64,
    /// Weakness persists until own supply detection; not merely 30 s.
    pub weak: bool,
    /// Respawn invincibility deadline in round ns.
    pub invincible_until_ns: u64,
    /// Main-controller connectivity observation.
    pub online: bool,
    /// Speed module connectivity observation.
    pub speed_module_online: bool,
    /// Number of offline armor/supercapacitor modules, bounded to five.
    pub offline_modules: u8,
    /// Number of deaths this round.
    pub deaths: u32,
    /// Accumulated automatic respawn progress in ns.
    pub respawn_progress_ns: u64,
    /// Referee power buffer in millijoules, UL §3.3.1.4.
    pub buffer_mj: u32,
    /// Chassis power-cut deadline in round ns.
    pub power_cut_until_ns: u64,
    /// Measured motor power in milliwatts, supplied by physics.
    pub actual_power_mw: u32,
    /// Actual rounds detected this round, used by Infantry tie breakers.
    pub rounds_fired: u32,
    /// First detected launch timestamp in round ns.
    pub first_launch_ns: Option<u64>,
    /// First actual HP loss timestamp in round ns, including collisions/penalties.
    pub first_damage_ns: Option<u64>,
    format: Format,
    supply: Contact,
    zones: [Contact; 3],
    tunnel_first: Option<(Team, u8, u64)>,
    cooling_until_ns: u64,
    last_launch_ns: Option<u64>,
    defeated_at_ns: Option<u64>,
    offline_since_ns: Option<u64>,
    large_suppressed: bool,
    launcher_link_good: bool,
    postmortem_launches: u8,
    healing_remainder: u32,
    offline_remainder: u32,
}
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
struct Contact {
    detected: bool,
    expires_ns: u64,
}
impl Contact {
    fn active(&self, now: u64) -> bool {
        self.detected || now < self.expires_ns
    }
    fn observe(&mut self, detected: bool, now: u64) {
        if self.detected && !detected {
            self.expires_ns = now.saturating_add(GRACE);
        }
        self.detected = detected;
    }
}
impl Robot {
    /// Current UL table properties; no UC leveling or assembly multipliers.
    pub fn stats(&self) -> Stats {
        let (hp, power, heat, cooling, speed) = match (self.format, self.role) {
            (Format::Infantry, _) => (200, 120, 80, 12, 25.),
            (_, Role::Heavy) if self.heavy_upgraded => (450, 50, 250, 66, 16.5),
            (_, Role::Heavy) => (100, 50, 0, 0, 0.),
            (_, Role::Infantry) => (
                if self.power_chassis { 300 } else { 350 },
                if self.power_chassis { 90 } else { 75 },
                if self.burst_launcher { 230 } else { 88 },
                if self.burst_launcher { 14 } else { 24 },
                25.,
            ),
            (_, Role::Sentry) => (400, 100, 260, 30, 25.),
        };
        Stats {
            max_hp: hp,
            chassis_power_w: power,
            heat_limit: heat,
            cooling_per_s: cooling,
            max_speed_m_s: speed,
        }
    }
    fn new(format: Format, id: u32, team: Team, role: Role) -> Self {
        let mut r = Self {
            id,
            team,
            role,
            hp: 0,
            max_hp: 0,
            allowance: if format == Format::Infantry {
                200
            } else if role == Role::Sentry {
                750
            } else {
                0
            },
            heat_tenths: 0,
            heavy_upgraded: false,
            heavy_installed: false,
            power_chassis: false,
            burst_launcher: false,
            overheated: false,
            heat_locked_for_round: false,
            overspeed_locked_for_round: false,
            overspeed_until_ns: 0,
            weak: false,
            invincible_until_ns: 0,
            online: true,
            speed_module_online: true,
            offline_modules: 0,
            deaths: 0,
            respawn_progress_ns: 0,
            buffer_mj: 60_000,
            power_cut_until_ns: 0,
            actual_power_mw: 0,
            rounds_fired: 0,
            first_launch_ns: None,
            first_damage_ns: None,
            format,
            supply: Contact::default(),
            zones: Default::default(),
            tunnel_first: None,
            cooling_until_ns: 0,
            last_launch_ns: None,
            defeated_at_ns: None,
            offline_since_ns: None,
            large_suppressed: false,
            launcher_link_good: true,
            postmortem_launches: 0,
            healing_remainder: 0,
            offline_remainder: 0,
        };
        r.max_hp = r.stats().max_hp;
        r.hp = r.max_hp;
        r
    }
}
/// Team economy/control counters, UL §§3.3.3–3.3.4.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TeamState {
    /// Remaining control, initially 600 (unused in Infantry).
    pub control: u32,
    /// Ammunition-purchase currency.
    pub gold: u32,
    /// Actual enemy HP removed by projectile hits.
    pub attack_damage: u64,
    /// One-time rewards already paid for 200/400 control deficits.
    pub catch_up_awarded: [bool; 2],
}
/// Independent UL zone state; progress is ns at one progress point per second.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ZoneState {
    /// Red/blue occupation progress, bounded to 5 seconds.
    pub progress_ns: [u64; 2],
    /// Controller remains until its progress reaches zero.
    pub controller: Option<Team>,
}
/// Deterministic complete round authority, suitable for serde checkpoints.
///
/// ```
/// use rm_sim_gameplay::{Team, rmul_2027::{Match, Format, Role}};
/// let mut game = Match::new(Format::Infantry);
/// game.add_robot(3, Team::Red, Role::Infantry).unwrap();
/// game.add_robot(13, Team::Blue, Role::Infantry).unwrap();
/// game.start();
/// assert!(game.fire(3, false, 25.0));
/// assert_eq!(game.damage(Some(3), 13, false), 20);
/// ```
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Match {
    /// Explicit competition/revision identity, checked by validate.
    pub ruleset: Ruleset,
    /// Competition format.
    pub format: Format,
    /// Current lifecycle phase.
    pub phase: Phase,
    /// Whole running-round elapsed ns; frozen outside Running.
    pub elapsed_ns: u64,
    /// Five-second countdown remaining ns; zero outside Countdown.
    pub countdown_remaining_ns: u64,
    /// Robots in stable insertion order.
    pub robots: Vec<Robot>,
    /// Teams in crate::Team::BOTH order.
    pub teams: [TeamState; 2],
    /// Red highland, blue highland and center.
    pub zones: [ZoneState; 3],
    /// Frozen final result, absent until Finished.
    pub result: Option<RoundResult>,
    settled_ns: u64,
}
impl Match {
    /// Construct an idle UL authority with no robots and no UC state.
    pub fn new(format: Format) -> Self {
        Self {
            ruleset: format.ruleset(),
            format,
            phase: Phase::Idle,
            elapsed_ns: 0,
            countdown_remaining_ns: 0,
            robots: Vec::new(),
            teams: std::array::from_fn(|_| TeamState {
                control: 600,
                gold: 0,
                attack_damage: 0,
                catch_up_awarded: [false; 2],
            }),
            zones: Default::default(),
            result: None,
            settled_ns: 0,
        }
    }
    /// Add a uniquely addressed role while idle; at most one per role/team.
    pub fn add_robot(&mut self, id: u32, team: Team, role: Role) -> Result<(), &'static str> {
        if self.phase != Phase::Idle {
            return Err("RMUL roster is frozen after countdown");
        }
        if self.format == Format::Infantry && role != Role::Infantry {
            return Err("Infantry format admits Infantry only");
        }
        if self
            .robots
            .iter()
            .any(|r| r.id == id || (r.team == team && r.role == role))
        {
            return Err("duplicate RMUL robot or team role");
        }
        self.robots.push(Robot::new(self.format, id, team, role));
        Ok(())
    }
    /// Running time remaining in ns, UL §§3.3.7/4.4.
    pub fn remaining_ns(&self) -> u64 {
        self.format.duration_ns().saturating_sub(self.elapsed_ns)
    }
    /// Enter a five-second countdown from idle.
    pub fn begin_countdown(&mut self) -> Result<(), &'static str> {
        if self.phase != Phase::Idle {
            return Err("RMUL countdown requires idle state");
        }
        self.phase = Phase::Countdown;
        self.countdown_remaining_ns = 5 * SECOND;
        Ok(())
    }
    /// Start immediately from idle/countdown; repeated starts do not reset rules.
    pub fn start(&mut self) {
        if matches!(self.phase, Phase::Idle | Phase::Countdown) {
            self.phase = Phase::Running;
            self.countdown_remaining_ns = 0;
        }
    }
    /// Reset all round authority while preserving roster and Infantry choices.
    pub fn reset(&mut self) {
        let mut fresh = Self::new(self.format);
        for old in &self.robots {
            let mut r = Robot::new(self.format, old.id, old.team, old.role);
            r.power_chassis = old.power_chassis;
            r.burst_launcher = old.burst_launcher;
            r.max_hp = r.stats().max_hp;
            r.hp = r.max_hp;
            fresh.robots.push(r);
        }
        *self = fresh;
    }
    /// Freeze the current comparison result; no-op once finished.
    pub fn finish(&mut self) {
        if self.phase != Phase::Finished {
            let winner = self.compare();
            self.end(winner, ResultReason::Comparison);
        }
    }
    fn end(&mut self, winner: Option<Team>, reason: ResultReason) {
        self.phase = Phase::Finished;
        self.countdown_remaining_ns = 0;
        self.result = Some(RoundResult {
            winner,
            reason,
            elapsed_ns: self.elapsed_ns,
        });
    }
    fn robot_index(&self, id: u32) -> Option<usize> {
        self.robots.iter().position(|r| r.id == id)
    }
    /// Reject launch before physics validation; does not consume ammunition/heat.
    pub fn check_launch(&self, id: u32, large: bool) -> Result<(), &'static str> {
        if self.phase != Phase::Running {
            return Err("RMUL round is not running");
        }
        let r = self
            .robots
            .iter()
            .find(|r| r.id == id)
            .ok_or("unknown RMUL robot")?;
        if large != (r.role == Role::Heavy) {
            return Err("caliber does not match RMUL role");
        }
        if r.hp == 0
            || !r.online
            || !r.speed_module_online
            || r.weak
            || r.overheated
            || r.heat_locked_for_round
            || r.overspeed_locked_for_round
            || self.elapsed_ns < r.overspeed_until_ns
        {
            return Err("RMUL launcher is locked");
        }
        if large && !r.heavy_installed {
            return Err("Heavy launcher is not installed");
        }
        if r.allowance == 0 {
            return Err("RMUL allowance exhausted");
        }
        Ok(())
    }
    /// Accept a physically validated launch, then settle heat/speed penalties.
    /// Overspeed rounds are still actual launches; the lock affects later rounds.
    pub fn fire(&mut self, id: u32, large: bool, speed: f64) -> bool {
        if !speed.is_finite() || speed < 0. || self.check_launch(id, large).is_err() {
            return false;
        }
        self.record_launch(id, large, speed)
    }
    /// Record a referee-detected launch even if hardware bypassed a lock/allowance.
    /// This trusted path implements overfiring and permanent heat thresholds.
    pub fn record_launch(&mut self, id: u32, large: bool, speed: f64) -> bool {
        if self.phase != Phase::Running || !speed.is_finite() || speed < 0. {
            return false;
        }
        let Some(i) = self.robot_index(id) else {
            return false;
        };
        let r = &mut self.robots[i];
        if large != (r.role == Role::Heavy) || (large && !r.heavy_installed) {
            return false;
        }
        if large && r.hp > 0 && !r.heavy_upgraded {
            r.heavy_upgraded = true;
            r.max_hp = 450;
            r.hp = (r.hp + 350).min(450);
        }
        if large {
            if r.allowance == 0 {
                r.large_suppressed = true;
            }
            if r.hp == 0 {
                r.postmortem_launches = r.postmortem_launches.saturating_add(1);
                if r.postmortem_launches >= 3 {
                    r.large_suppressed = true;
                }
            }
        }
        r.allowance = r.allowance.saturating_sub(1);
        r.rounds_fired = r.rounds_fired.saturating_add(1);
        r.first_launch_ns.get_or_insert(self.elapsed_ns);
        r.last_launch_ns = Some(self.elapsed_ns);
        let stats = r.stats();
        let max = stats.max_speed_m_s;
        if speed > max {
            let duration = if large {
                if speed <= 1.1 * max {
                    15
                } else if speed <= 1.2 * max {
                    20
                } else {
                    0
                }
            } else if speed - max < 5. {
                15
            } else if speed - max < 10. {
                20
            } else {
                0
            };
            if duration == 0 {
                r.overspeed_locked_for_round = true;
            } else {
                r.overspeed_until_ns = r
                    .overspeed_until_ns
                    .max(self.elapsed_ns + duration * SECOND);
            }
        }
        r.heat_tenths = r.heat_tenths.saturating_add(if large { 1000 } else { 100 });
        if r.heat_tenths >= 10 * (stats.heat_limit + if large { 200 } else { 100 }) {
            r.heat_locked_for_round = true;
        }
        if r.heat_tenths > 10 * stats.heat_limit {
            r.overheated = true;
        }
        true
    }
    /// Apply an already detected armor projectile hit (UL Table 3-8).
    /// Caller owns speed-normal thresholds and per-plate detection intervals.
    pub fn damage(&mut self, attacker: Option<u32>, victim: u32, large: bool) -> u32 {
        if self.phase != Phase::Running {
            return 0;
        }
        let Some(v) = self.robot_index(victim) else {
            return 0;
        };
        let a = attacker.and_then(|id| self.robot_index(id));
        if let Some(a) = a
            && (self.robots[a].team == self.robots[v].team
                || large != (self.robots[a].role == Role::Heavy))
        {
            return 0;
        }
        if large {
            let source = a
                .filter(|&a| self.robots[a].role == Role::Heavy)
                .or_else(|| {
                    self.robots
                        .iter()
                        .position(|r| r.role == Role::Heavy && r.team != self.robots[v].team)
                });
            let Some(source) = source else {
                return 0;
            };
            let r = &self.robots[source];
            if !r.heavy_upgraded
                || r.last_launch_ns
                    .is_none_or(|t| self.elapsed_ns.saturating_sub(t) >= 4 * SECOND)
                || r.large_suppressed
                || !r.launcher_link_good
                || r.offline_since_ns
                    .is_some_and(|t| self.elapsed_ns.saturating_sub(t) >= 3 * SECOND)
                || r.defeated_at_ns
                    .is_some_and(|t| self.elapsed_ns.saturating_sub(t) >= 3 * SECOND)
            {
                return 0;
            }
        }
        let hp = self.hurt(v, if large { 200 } else { 20 }, false);
        if let Some(a) = a {
            self.teams[self.robots[a].team.index()].attack_damage = self.teams
                [self.robots[a].team.index()]
            .attack_damage
            .saturating_add(u64::from(hp));
            if self.format == Format::Infantry && self.robots[a].hp > 0 {
                self.robots[a].hp = (self.robots[a].hp + hp.div_ceil(2)).min(self.robots[a].max_hp);
            }
        }
        self.resolve_defeat();
        hp
    }
    /// Ordinary collision damage, 2 HP before defense/invincibility (Table 3-8).
    pub fn collision(&mut self, id: u32) -> u32 {
        if self.phase != Phase::Running {
            return 0;
        }
        let Some(i) = self.robot_index(id) else {
            return 0;
        };
        let hp = self.hurt(i, 2, false);
        self.resolve_defeat();
        hp
    }
    /// Referee penalty HP removal bypasses ordinary immunity/defense.
    pub fn penalty_damage(&mut self, id: u32, amount: u32) -> u32 {
        if self.phase != Phase::Running {
            return 0;
        }
        let Some(i) = self.robot_index(id) else {
            return 0;
        };
        let hp = self.hurt(i, amount, true);
        self.resolve_defeat();
        hp
    }
    fn hurt(&mut self, i: usize, amount: u32, bypass: bool) -> u32 {
        let now = self.elapsed_ns;
        let r = &mut self.robots[i];
        if r.hp == 0 {
            return 0;
        }
        if !bypass
            && (!r.online
                || now < r.invincible_until_ns
                || (r.role == Role::Heavy && !r.heavy_upgraded && r.supply.active(now)))
        {
            return 0;
        }
        let defended = !bypass
            && self.format == Format::ThreeVsThree
            && !r.weak
            && r.zones[..2].iter().any(|c| c.active(now));
        let loss = if defended {
            (u64::from(amount) * 75 + 50) / 100
        } else {
            u64::from(amount)
        }
        .min(u64::from(r.hp)) as u32;
        r.hp -= loss;
        if loss > 0 {
            r.first_damage_ns.get_or_insert(now);
        }
        if r.hp == 0 {
            r.deaths = r.deaths.saturating_add(1);
            r.defeated_at_ns = Some(now);
            r.postmortem_launches = 0;
            r.respawn_progress_ns = 0;
            r.heat_tenths = 0;
            r.overheated = false;
            r.buffer_mj = 60_000;
            r.healing_remainder = 0;
            r.invincible_until_ns = 0;
            if self.format == Format::ThreeVsThree && self.phase == Phase::Running {
                self.teams[r.team.index()].control =
                    self.teams[r.team.index()].control.saturating_sub(10);
            }
        }
        loss
    }
    /// Set bounded HP for operator commands; reductions still record death once.
    pub fn set_robot_hp(&mut self, id: u32, hp: u32) -> Result<(), &'static str> {
        let i = self.robot_index(id).ok_or("unknown RMUL robot")?;
        let hp = hp.min(self.robots[i].max_hp);
        if hp < self.robots[i].hp {
            self.hurt(i, self.robots[i].hp - hp, true);
        } else {
            self.robots[i].hp = hp;
            if hp > 0 {
                self.robots[i].defeated_at_ns = None;
                self.robots[i].weak = false;
                self.robots[i].invincible_until_ns = 0;
            }
        }
        if self.phase == Phase::Running {
            self.resolve_defeat();
        }
        Ok(())
    }
    /// Own-supply presence; loss has two-second grace (§3.3.5).
    /// The caller must report only this robot's own team's supply card.
    pub fn observe_supply(&mut self, id: u32, detected: bool) -> bool {
        if self.format != Format::ThreeVsThree || self.phase != Phase::Running {
            return false;
        }
        let Some(i) = self.robot_index(id) else {
            return false;
        };
        let r = &mut self.robots[i];
        r.supply.observe(detected, self.elapsed_ns);
        if detected {
            r.tunnel_first = None;
            if r.hp > 0 && r.online {
                r.weak = false;
                r.invincible_until_ns = 0;
            }
        }
        true
    }
    /// Zone presence; eligibility is resolved here, not by the physics adapter.
    pub fn observe_zone(&mut self, id: u32, zone: Zone, detected: bool) -> bool {
        if self.format != Format::ThreeVsThree || self.phase != Phase::Running {
            return false;
        }
        let Some(i) = self.robot_index(id) else {
            return false;
        };
        let r = &mut self.robots[i];
        r.zones[zone.index()].observe(detected, self.elapsed_ns);
        if detected {
            r.tunnel_first = None;
        }
        true
    }
    /// Own tunnel card entry (0/1), with no intervening unrelated card (§3.3.5.2).
    /// Repeated sampling of the same card must not count as another entry.
    pub fn observe_tunnel(&mut self, id: u32, side: Team, card: u8) -> bool {
        if self.format != Format::ThreeVsThree || self.phase != Phase::Running || card > 1 {
            return false;
        }
        let Some(i) = self.robot_index(id) else {
            return false;
        };
        let r = &mut self.robots[i];
        if !r.online || r.hp == 0 || r.team != side {
            r.tunnel_first = None;
            return false;
        }
        if r.tunnel_first.is_some_and(|(s, c, t)| {
            s == side && c != card && self.elapsed_ns.saturating_sub(t) <= 3 * SECOND
        }) {
            r.cooling_until_ns = self.elapsed_ns + 60 * SECOND;
            r.tunnel_first = None;
        } else if r
            .tunnel_first
            .is_none_or(|(s, c, _)| s != side || c != card)
        {
            r.tunnel_first = Some((side, card, self.elapsed_ns));
        }
        true
    }
    /// Cancel a tunnel pair upon any unrelated RFID entry.
    pub fn observe_other_card(&mut self, id: u32) {
        if let Some(i) = self.robot_index(id) {
            self.robots[i].tunnel_first = None;
        }
    }
    /// Install/remove Heavy's module; caller validates the physical workbench.
    pub fn set_heavy_installed(&mut self, id: u32, installed: bool) -> Result<(), &'static str> {
        if self.format != Format::ThreeVsThree || self.phase == Phase::Finished {
            return Err("Heavy installation unavailable");
        }
        let i = self.robot_index(id).ok_or("unknown RMUL robot")?;
        if self.robots[i].role != Role::Heavy {
            return Err("only Heavy installs a 42 mm module");
        }
        if self.robots[i].hp == 0 || !self.robots[i].online {
            return Err("Heavy installation requires a live online robot");
        }
        self.robots[i].heavy_installed = installed;
        Ok(())
    }
    /// Select 3v3 Infantry table profiles before countdown.
    pub fn configure_infantry(
        &mut self,
        id: u32,
        power_chassis: bool,
        burst_launcher: bool,
    ) -> Result<(), &'static str> {
        if self.format != Format::ThreeVsThree || self.phase != Phase::Idle {
            return Err("RMUL profile selection requires idle 3v3");
        }
        let i = self.robot_index(id).ok_or("unknown RMUL robot")?;
        let r = &mut self.robots[i];
        if r.role != Role::Infantry {
            return Err("profile selection requires Infantry");
        }
        r.power_chassis = power_chassis;
        r.burst_launcher = burst_launcher;
        r.max_hp = r.stats().max_hp;
        r.hp = r.max_hp;
        Ok(())
    }
    /// Purchase batches: 10 gold buys ten 17 mm or one 42 mm (Table 3-12).
    pub fn buy_ammo(&mut self, id: u32, large: bool, batches: u32) -> Result<(), &'static str> {
        if self.format != Format::ThreeVsThree || self.phase != Phase::Running {
            return Err("UL ammunition purchase unavailable");
        }
        let i = self.robot_index(id).ok_or("unknown RMUL robot")?;
        let r = &self.robots[i];
        if r.role == Role::Sentry
            || large != (r.role == Role::Heavy)
            || r.hp == 0
            || !r.online
            || !r.supply.active(self.elapsed_ns)
        {
            return Err("UL purchase requires eligible robot at own supply");
        }
        let cost = batches
            .checked_mul(10)
            .ok_or("ammunition purchase overflow")?;
        let rounds = batches
            .checked_mul(if large { 1 } else { 10 })
            .ok_or("ammunition purchase overflow")?;
        let allowance = r
            .allowance
            .checked_add(rounds)
            .ok_or("ammunition allowance overflow")?;
        let t = r.team.index();
        if cost > self.teams[t].gold {
            return Err("insufficient UL gold");
        }
        self.teams[t].gold -= cost;
        self.robots[i].allowance = allowance;
        if self.robots[i].online && self.robots[i].hp > 0 && allowance > 0 {
            self.robots[i].large_suppressed = false;
        }
        Ok(())
    }
    /// Supply measured chassis motor power in W; reject nonfinite/negative data.
    pub fn observe_power(&mut self, id: u32, actual_w: f64) -> bool {
        if !actual_w.is_finite() || actual_w < 0. || actual_w > u32::MAX as f64 / 1000. {
            return false;
        }
        let Some(i) = self.robot_index(id) else {
            return false;
        };
        self.robots[i].actual_power_mw = (actual_w * 1000.).round() as u32;
        true
    }
    /// Actuation eligibility after death, connectivity and referee power cuts.
    pub fn chassis_enabled(&self, id: u32) -> bool {
        self.phase == Phase::Running
            && self.robots.iter().any(|r| {
                r.id == id && r.hp > 0 && r.online && self.elapsed_ns >= r.power_cut_until_ns
            })
    }
    /// Drive budget in W, including the available 100 ms share of the 60 J
    /// referee buffer (UL §3.3.1.4). Physical hardware remains an upper bound.
    /// A chassis cut does not imply a gimbal or other actuation cut.
    ///
    /// ```
    /// use rm_sim_gameplay::{Team, rmul_2027::{Format, Match, Role}};
    /// let mut game = Match::new(Format::ThreeVsThree);
    /// game.add_robot(3, Team::Red, Role::Infantry)?;
    /// assert_eq!(game.drive_budget_w(3), 0); // Not running yet.
    /// game.start();
    /// assert_eq!(game.drive_budget_w(3), 675); // 75 W + 60 J / 0.1 s.
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn drive_budget_w(&self, id: u32) -> u32 {
        self.robots
            .iter()
            .find(|r| r.id == id)
            .filter(|_| self.chassis_enabled(id))
            .map_or(0, |r| {
                r.stats().chassis_power_w.saturating_add(r.buffer_mj / 100)
            })
    }
    /// Update trusted module observations; penalties settle at 2 Hz (§3.3.1.5).
    pub fn observe_modules(
        &mut self,
        id: u32,
        online: bool,
        speed_online: bool,
        offline_modules: u8,
    ) -> bool {
        if offline_modules > 5 {
            return false;
        }
        let Some(i) = self.robot_index(id) else {
            return false;
        };
        let r = &mut self.robots[i];
        if r.online && !online {
            r.offline_since_ns = Some(self.elapsed_ns);
        }
        if online {
            r.offline_since_ns = None;
        }
        r.online = online;
        r.speed_module_online = speed_online;
        r.offline_modules = offline_modules;
        true
    }
    /// Report whether Heavy launcher packet loss is below 5% (§3.3.3).
    /// The caller measures the link; other roles have no such 42 mm gate.
    pub fn observe_launcher_link(&mut self, id: u32, good: bool) -> bool {
        let Some(i) = self.robot_index(id) else {
            return false;
        };
        let r = &mut self.robots[i];
        if r.role != Role::Heavy {
            return false;
        }
        r.launcher_link_good = good;
        if !good {
            r.large_suppressed = true;
        } else if r.hp > 0 && r.online && r.allowance > 0 {
            r.large_suppressed = false;
        }
        true
    }
    /// Advance explicit time; settlements use a canonical 100 ms clock.
    /// Splitting advance calls without intervening inputs preserves exact state.
    pub fn advance(&mut self, mut dt_ns: u64) {
        if self.phase == Phase::Countdown {
            let step = dt_ns.min(self.countdown_remaining_ns);
            self.countdown_remaining_ns -= step;
            dt_ns -= step;
            if self.countdown_remaining_ns == 0 {
                self.start();
            }
        }
        if self.phase != Phase::Running {
            return;
        }
        let target = self
            .elapsed_ns
            .saturating_add(dt_ns)
            .min(self.format.duration_ns());
        while self.phase == Phase::Running && self.settled_ns + PERIOD <= target {
            self.settled_ns += PERIOD;
            self.elapsed_ns = self.settled_ns;
            self.settle();
        }
        if self.phase == Phase::Running {
            self.elapsed_ns = target;
            if target == self.format.duration_ns() {
                self.finish();
            }
        }
    }
    fn trailing_gap(&self, team: Team) -> u32 {
        self.teams[team.other().index()]
            .control
            .saturating_sub(self.teams[team.index()].control)
    }
    fn settle(&mut self) {
        let now = self.elapsed_ns;
        if self.format == Format::ThreeVsThree {
            for zone in Zone::ALL {
                let z = zone.index();
                let mut active = [false; 2];
                for r in &self.robots {
                    if r.hp > 0 && r.online && !r.weak && r.zones[z].active(now.saturating_sub(1)) {
                        active[r.team.index()] = true;
                    }
                }
                let state = &mut self.zones[z];
                match active {
                    [true, false] | [false, true] => {
                        let t = usize::from(active[1]);
                        let other = 1 - t;
                        if state.progress_ns[other] > 0 {
                            state.progress_ns[other] =
                                state.progress_ns[other].saturating_sub(PERIOD);
                        } else {
                            state.progress_ns[t] = (state.progress_ns[t] + PERIOD).min(5 * SECOND);
                        }
                    }
                    [false, false] if state.controller.is_some() => {
                        for p in &mut state.progress_ns {
                            *p = p.saturating_sub(PERIOD);
                        }
                    }
                    _ => {}
                }
                if state
                    .controller
                    .is_some_and(|t| state.progress_ns[t.index()] == 0)
                {
                    state.controller = None;
                }
                for t in Team::BOTH {
                    if state.progress_ns[t.index()] == 5 * SECOND {
                        state.controller = Some(t);
                    }
                }
            }
            if now.is_multiple_of(SECOND) {
                for z in &self.zones {
                    if let Some(t) = z.controller {
                        let other = t.other().index();
                        self.teams[other].control = self.teams[other].control.saturating_sub(1);
                    }
                }
                for (at, gold) in [(1, 200), (61, 200), (121, 200), (181, 300), (241, 300)] {
                    if now == at * SECOND {
                        for t in &mut self.teams {
                            t.gold += gold;
                        }
                    }
                }
            }
            self.resolve_defeat();
            if self.phase != Phase::Running {
                return;
            }
        }
        for i in 0..self.robots.len() {
            let gap = self.trailing_gap(self.robots[i].team);
            let r = &mut self.robots[i];
            let stats = r.stats();
            if r.hp > 0 && r.online && r.allowance > 0 && r.launcher_link_good {
                r.large_suppressed = false;
            }
            let cooling = stats.cooling_per_s * if now <= r.cooling_until_ns { 3 } else { 2 };
            // A 100 ms cooling decrement in tenths is the per-second rate;
            // every source rate is even, so the 1.5x multiplier is exact.
            r.heat_tenths = r.heat_tenths.saturating_sub(cooling / 2);
            if r.heat_tenths == 0 {
                r.overheated = false;
            }
            if r.hp == 0 {
                if self.format == Format::ThreeVsThree
                    && r.online
                    && r.defeated_at_ns.is_some_and(|t| now > t)
                {
                    let required = u64::from(r.deaths)
                        .saturating_mul(5 * SECOND)
                        .saturating_mul(if gap >= 400 {
                            50
                        } else if gap >= 200 {
                            67
                        } else {
                            100
                        })
                        / 100;
                    r.respawn_progress_ns = r.respawn_progress_ns.saturating_add(
                        PERIOD.min(now.saturating_sub(r.defeated_at_ns.unwrap_or(now))),
                    );
                    if r.respawn_progress_ns >= required {
                        r.hp = (r.max_hp + 2) / 5;
                        r.weak = true;
                        r.invincible_until_ns = now + 30 * SECOND;
                        r.defeated_at_ns = None;
                        r.respawn_progress_ns = 0;
                    }
                }
                continue;
            }
            if r.weak && r.online && r.supply.detected {
                r.weak = false;
                r.invincible_until_ns = 0;
            }
            if r.online {
                let actual_mw = if now < r.power_cut_until_ns {
                    0
                } else {
                    r.actual_power_mw
                };
                let delta = (i64::from(actual_mw) - i64::from(stats.chassis_power_w) * 1000) / 10;
                r.buffer_mj = (i64::from(r.buffer_mj) - delta).clamp(0, 60_000) as u32;
                if r.buffer_mj == 0 && delta > 0 {
                    r.power_cut_until_ns = now + 5 * SECOND;
                }
            }
            let mut damage = 0;
            if now.is_multiple_of(SECOND / 2) {
                if !r.online {
                    r.offline_remainder += r.max_hp * 5;
                    damage = r.offline_remainder / 200;
                    r.offline_remainder %= 200;
                } else {
                    damage = 4 * u32::from(r.offline_modules);
                }
            }
            if self.format == Format::ThreeVsThree
                && r.online
                && r.supply.active(now.saturating_sub(1))
            {
                r.healing_remainder += r.max_hp
                    * if gap >= 400 {
                        50
                    } else if gap >= 200 {
                        33
                    } else {
                        25
                    };
                let heal = r.healing_remainder / 1000;
                r.healing_remainder %= 1000;
                r.hp = r.hp.saturating_add(heal).min(r.max_hp);
                if r.hp == r.max_hp {
                    r.healing_remainder = 0;
                }
            }
            if damage > 0 {
                self.hurt(i, damage, true);
            }
        }
        self.resolve_defeat();
    }
    fn resolve_defeat(&mut self) {
        if self.phase != Phase::Running {
            return;
        }
        if self.format == Format::ThreeVsThree {
            for team in Team::BOTH {
                let gap = self.trailing_gap(team);
                for (i, threshold) in [200, 400].into_iter().enumerate() {
                    let state = &mut self.teams[team.index()];
                    if gap >= threshold && !state.catch_up_awarded[i] {
                        state.gold = state.gold.saturating_add(200);
                        state.catch_up_awarded[i] = true;
                    }
                }
            }
            let dead = [self.teams[0].control == 0, self.teams[1].control == 0];
            if dead.iter().any(|&v| v) {
                let winner = match dead {
                    [true, false] => Some(Team::Blue),
                    [false, true] => Some(Team::Red),
                    _ => None,
                };
                self.end(winner, ResultReason::ControlDepleted);
            }
        } else {
            let dead = Team::BOTH.map(|t| self.robots.iter().any(|r| r.team == t && r.hp == 0));
            if dead.iter().any(|&v| v) {
                let winner = match dead {
                    [true, false] => Some(Team::Blue),
                    [false, true] => Some(Team::Red),
                    _ => None,
                };
                self.end(winner, ResultReason::Defeat);
            }
        }
    }
    fn compare(&self) -> Option<Team> {
        use std::cmp::Ordering;
        let hp = Team::BOTH.map(|t| {
            self.robots
                .iter()
                .filter(|r| r.team == t)
                .map(|r| u64::from(r.hp))
                .sum::<u64>()
        });
        let ordering = if self.format == Format::ThreeVsThree {
            self.teams[0]
                .control
                .cmp(&self.teams[1].control)
                .then(
                    self.teams[0]
                        .attack_damage
                        .cmp(&self.teams[1].attack_damage),
                )
                .then(hp[0].cmp(&hp[1]))
        } else {
            let robots = Team::BOTH.map(|t| self.robots.iter().find(|r| r.team == t));
            hp[0].cmp(&hp[1]).then_with(|| {
                let [Some(red), Some(blue)] = robots else {
                    return Ordering::Equal;
                };
                if red.hp == red.max_hp && blue.hp == blue.max_hp {
                    red.rounds_fired.cmp(&blue.rounds_fired).then_with(|| {
                        blue.first_launch_ns
                            .unwrap_or(u64::MAX)
                            .cmp(&red.first_launch_ns.unwrap_or(u64::MAX))
                    })
                } else {
                    red.first_damage_ns
                        .unwrap_or(u64::MAX)
                        .cmp(&blue.first_damage_ns.unwrap_or(u64::MAX))
                }
            })
        };
        match ordering {
            Ordering::Greater => Some(Team::Red),
            Ordering::Less => Some(Team::Blue),
            Ordering::Equal => None,
        }
    }
    /// Validate trusted deserialized checkpoints before restoring an authority.
    /// This is structural validation, not authentication of referee-owned data.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.ruleset != self.format.ruleset() {
            return Err("RMUL checkpoint competition/revision mismatch");
        }
        if self.elapsed_ns > self.format.duration_ns()
            || self.settled_ns > self.elapsed_ns
            || !self.settled_ns.is_multiple_of(PERIOD)
            || self.elapsed_ns - self.settled_ns >= PERIOD
        {
            return Err("invalid RMUL clock");
        }
        if (self.phase == Phase::Finished) != self.result.is_some()
            || (self.phase == Phase::Countdown) != (self.countdown_remaining_ns > 0)
            || self.countdown_remaining_ns > 5 * SECOND
        {
            return Err("invalid RMUL phase/result");
        }
        if self
            .result
            .as_ref()
            .is_some_and(|r| r.elapsed_ns != self.elapsed_ns)
            || (matches!(self.phase, Phase::Idle | Phase::Countdown) && self.elapsed_ns != 0)
        {
            return Err("invalid RMUL lifecycle clock");
        }
        if self.teams.iter().any(|t| t.control > 600) {
            return Err("invalid UL control");
        }
        for (i, r) in self.robots.iter().enumerate() {
            if r.format != self.format
                || (self.format == Format::Infantry && r.role != Role::Infantry)
                || self.robots[..i]
                    .iter()
                    .any(|p| p.id == r.id || (p.team == r.team && p.role == r.role))
            {
                return Err("invalid RMUL roster");
            }
            if r.max_hp != r.stats().max_hp
                || r.hp > r.max_hp
                || r.buffer_mj > 60_000
                || r.offline_modules > 5
                || (r.role == Role::Sentry && r.allowance > 750)
                || r.healing_remainder >= 1000
                || r.offline_remainder >= 200
                || r.first_launch_ns.is_some_and(|t| t > self.elapsed_ns)
                || r.first_damage_ns.is_some_and(|t| t > self.elapsed_ns)
                || r.last_launch_ns.is_some_and(|t| t > self.elapsed_ns)
                || r.defeated_at_ns
                    .is_some_and(|t| t > self.elapsed_ns || r.hp > 0)
                || (r.heavy_upgraded && r.role != Role::Heavy)
                || (r.heavy_installed && r.role != Role::Heavy)
                || ((r.power_chassis || r.burst_launcher) && r.role != Role::Infantry)
            {
                return Err("invalid RMUL robot properties");
            }
            if self.format == Format::Infantry
                && (r.allowance > 200
                    || r.heavy_installed
                    || r.heavy_upgraded
                    || r.power_chassis
                    || r.burst_launcher
                    || r.weak
                    || r.supply != Contact::default()
                    || r.zones.iter().any(|c| c != &Contact::default())
                    || r.cooling_until_ns != 0
                    || r.tunnel_first.is_some())
            {
                return Err("UL Infantry checkpoint contains 3v3 state");
            }
        }
        if self.format == Format::Infantry
            && (self.zones.iter().any(|z| z != &ZoneState::default())
                || self
                    .teams
                    .iter()
                    .any(|t| t.control != 600 || t.gold != 0 || t.catch_up_awarded != [false; 2]))
        {
            return Err("UL Infantry checkpoint contains control/economy state");
        }
        for z in &self.zones {
            if z.progress_ns.iter().any(|&v| v > 5 * SECOND)
                || z.progress_ns.iter().all(|&v| v > 0)
                || z.controller.is_some_and(|t| z.progress_ns[t.index()] == 0)
            {
                return Err("invalid UL occupation progress");
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
