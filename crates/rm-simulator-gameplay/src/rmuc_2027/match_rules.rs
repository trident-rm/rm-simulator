// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! RMUC 2027 V1.0.0 combat/control subset (§§5.1–5.6, 5.8).
//! Contacts are external observations; no geometry, physics or wall clock lives here.
use super::*;
/// Native continuous field effects and read-only control diagnostics.
pub mod buffs;
mod lifecycle;

/// UC §5.6.1 base HP; no virtual shield in this season.
pub const BASE_HP: u32 = 10_000;
/// UC §5.6.1 initial outpost HP.
pub const OUTPOST_HP: u32 = 1_000;
/// UC §5.6.4.1 activation expiry delay, in nanoseconds.
pub const RFID_GRACE_NS: u64 = 2_000_000_000;
/// UC §5.6.4.9 opposing Fortress occupation to open base armor.
pub const FORTRESS_NS: u64 = 10_000_000_000;
/// UC §5.6.4.9 progress retention after death/expiry, in nanoseconds.
pub const RETAIN_NS: u64 = 3_000_000_000;
/// UC §5.6.4.9 opposing Fortress unlock, also §5.6.1 rotor stop.
pub const THREE_MIN_NS: u64 = 180_000_000_000;
/// Simulator countdown before the seven-minute running phase.
pub const COUNTDOWN_NS: u64 = 5_000_000_000;

/// Precise competition and manual identity carried in checkpoints.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Ruleset {
    /// RMUC 2027 rule manual V1.0.0 (2026-10-15), native combat/control subset.
    Rmuc2027V1,
}
/// Live phase; countdown duration is kept separately in [`Match`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Phase {
    /// Free practice, before Start.
    Idle,
    /// Five-second simulator countdown.
    Countdown,
    /// Running seven-minute round.
    Running,
    /// Result settled; rule clocks and scoring stop.
    Finished,
}
/// Roles eligible for 2027 ground RFID, distinct from 2026 Hero/Engineer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum GroundRole {
    /// Heavy; model support is supplied by the caller.
    Heavy,
    /// Infantry, using native 2027 experience and performance tables.
    Infantry,
    /// Semi-automatic Sentry (UC Table 3-6).
    Sentry,
}
/// One eligible robot and its restorable activation/progress state.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Robot {
    /// Never-reused chassis identity.
    pub id: u32,
    /// Team fixed at deployment.
    pub team: Team,
    /// Official 2027 ground role.
    pub role: GroundRole,
    /// Remaining HP; zero removes all RFID eligibility immediately.
    pub hp: u32,
    /// Current maximum HP from native 2027 role, level and chassis choice.
    pub max_hp: u32,
    /// Native 2027 heat, allowance, experience, healing and respawn state.
    pub combat: super::combat::Combat,
    /// Referee connectivity; an offline robot cannot activate RFID.
    pub online: bool,
    /// RFID expiry by side, then Outpost/CentralHighland/Base/Fortress.
    pub expires_ns: [Option<u64>; 8],
    /// Opponent Fortress opening progress per original field side, in ns.
    pub fortress_ns: [u64; 2],
    /// Last time each Fortress timer was active; controls the three-second hold.
    pub fortress_last_ns: [Option<u64>; 2],
    /// Raw traversal observations, pending sequences and timed terrain benefits.
    pub terrain: super::terrain::Terrain,
}
/// Immutable winning comparison, UC §5.8.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Result {
    /// Winning side, or `None` when all comparisons tie.
    pub winner: Option<Team>,
    /// First deciding comparison, or draw.
    pub reason: ResultReason,
}
/// Comparison order at base destruction or timeout.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ResultReason {
    /// A base reached zero: compare remaining base HP.
    BaseDestroyed,
    /// Timeout: accumulated control.
    Control,
    /// Timeout: attack damage actually applied to enemy HP.
    AttackDamage,
    /// Timeout: total living robot HP.
    RobotHp,
    /// Every comparison tied.
    Draw,
}
/// Complete serializable authority. Durations are nanoseconds, never tick counts.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Match {
    /// Ruleset identity; prevents interpreting these records as an RMUL match.
    pub ruleset: Ruleset,
    /// Running/countdown/finished state.
    pub phase: Phase,
    /// Countdown time left, in nanoseconds.
    pub countdown_ns: u64,
    /// Territory authority, including mutable physical outpost ownership.
    pub control: Control,
    /// Bases in Red/Blue order.
    pub base_hp: [u32; 2],
    /// Lowest Base HP reached this round, UC §5.6.4.9; repairs never erase it.
    pub base_lowest_hp: [u32; 2],
    /// Fortress reserve units spent this round, by defending side (§5.6.4.9).
    /// Simulator policy: one persistent pool per side; re-entry does not refill it.
    pub fortress_reserve_used: [u32; 2],
    /// Outposts by original side, not current owner.
    pub outpost_hp: [u32; 2],
    /// Open base protective armor, Red/Blue.
    pub base_open: [bool; 2],
    /// First permanent rotor stop in running-round nanoseconds, by original side.
    pub rotor_stopped_ns: [Option<u64>; 2],
    /// Shared random direction, persisted so restores never reroll it.
    pub rotor_direction: i8,
    /// Damage credited to each attacking team, excluding overkill and friendly damage.
    pub attack_damage: [u64; 2],
    /// Combat economy balances, UC Table 5-6; assembly income is not enabled.
    pub gold: [u32; 2],
    /// Rounds bought for gold by team, capped at 1,000 (Table 5-7).
    pub purchased_rounds: [u32; 2],
    /// Assembly-controlled Infantry level caps; initially five (Table 5-12).
    pub level_caps: [u8; 2],
    /// Adaptive-armor launch/hit observations; all damage still uses default armor.
    pub adaptive: super::adaptive::Adaptive,
    /// Native team Rune authority, Red then Blue, independent of the 2026 engine.
    pub runes: [super::rune::Rune; 2],
    /// Robots in ascending id order, excluding departed players.
    pub robots: Vec<Robot>,
    /// Central-highland mutual exclusion latch by original field side.
    pub highland_team: [Option<Team>; 2],
    /// First own robot occupying each Fortress.
    pub fortress_robot: [Option<u32>; 2],
    /// Settled result, present only after finishing.
    pub result: Option<Result>,
}
impl Default for Match {
    fn default() -> Self {
        Self::new(1)
    }
}
impl Match {
    /// Create idle rules. The host supplies one random sign per round; both
    /// rotors share it (UC §5.6.1). No randomness is read in the engine.
    ///
    /// ```
    /// use rm_simulator_gameplay::rmuc_2027::match_rules::{Match, Phase, COUNTDOWN_NS};
    /// let mut game = Match::new(-1);
    /// game.start(-1);
    /// game.advance_ns(COUNTDOWN_NS);
    /// assert_eq!(game.phase, Phase::Running);
    /// assert_eq!(game.elapsed_ns(), 0);
    /// ```
    pub fn new(direction: i8) -> Self {
        Self {
            ruleset: Ruleset::Rmuc2027V1,
            phase: Phase::Idle,
            countdown_ns: 0,
            control: Control::new(),
            base_hp: [BASE_HP; 2],
            base_lowest_hp: [BASE_HP; 2],
            fortress_reserve_used: [0; 2],
            outpost_hp: [OUTPOST_HP; 2],
            base_open: [false; 2],
            rotor_stopped_ns: [None; 2],
            rotor_direction: if direction < 0 { -1 } else { 1 },
            attack_damage: [0; 2],
            gold: [0; 2],
            purchased_rounds: [0; 2],
            level_caps: [5; 2],
            adaptive: super::adaptive::Adaptive::default(),
            runes: std::array::from_fn(|i| super::rune::Rune::new(direction, i as u64 + 1)),
            robots: vec![],
            highland_team: [None; 2],
            fortress_robot: [None; 2],
            result: None,
        }
    }
    /// Running-round clock, independent of the field clock and pause duration.
    pub fn elapsed_ns(&self) -> u64 {
        self.control.snapshot().elapsed_ns
    }
    /// Register a prototype. Infantry uses HP-priority level one (200 HP,
    /// UC Table 5-17); semi-automatic Sentry uses 200 HP (UC Table 3-6).
    pub fn add_robot(&mut self, id: u32, team: Team, role: GroundRole) {
        self.robots.retain(|r| r.id != id);
        self.robots.push(Robot {
            id,
            team,
            role,
            hp: if role == GroundRole::Heavy { 400 } else { 200 },
            max_hp: if role == GroundRole::Heavy { 400 } else { 200 },
            combat: super::combat::Combat {
                allowance: if role == GroundRole::Sentry { 300 } else { 0 },
                ..Default::default()
            },
            online: true,
            expires_ns: [None; 8],
            fortress_ns: [0; 2],
            fortress_last_ns: [None; 2],
            terrain: super::terrain::Terrain::default(),
        });
        self.robots.sort_by_key(|r| r.id);
    }
    /// Start a new countdown; clear stale RFID, progress, damage and results.
    pub fn start(&mut self, direction: i8) {
        let rune_seeds = self.runes.each_ref().map(|r| r.seed);
        let robots = self
            .robots
            .iter()
            .map(|r| (r.id, r.team, r.role, r.combat.chassis, r.combat.launcher))
            .collect::<Vec<_>>();
        *self = Self::new(direction);
        for (r, seed) in self.runes.iter_mut().zip(rune_seeds) {
            r.seed = seed;
        }
        for (id, team, role, chassis, launcher) in robots {
            self.add_robot(id, team, role);
            let robot = self.robots.iter_mut().find(|r| r.id == id).unwrap();
            robot.combat.chassis = chassis;
            robot.combat.launcher = launcher;
            robot.max_hp = robot.stats(0).max_hp;
            robot.hp = robot.max_hp;
        }
        self.phase = Phase::Countdown;
        self.countdown_ns = COUNTDOWN_NS;
    }
    /// Validate a decoded checkpoint before using it to restore a world.
    pub fn validate(&self) -> std::result::Result<(), &'static str> {
        Control::restore(self.control.snapshot().clone())
            .map_err(|_| "invalid 2027 control state")?;
        if (self.phase == Phase::Running && self.elapsed_ns() >= ROUND_NS)
            || self.runes.iter().any(|r| !r.validate(self.elapsed_ns()))
            || !self.adaptive.validate(self.elapsed_ns())
            || self.purchased_rounds.iter().any(|&v| v > 1000)
            || self.level_caps.iter().any(|v| ![5, 7, 10].contains(v))
            || self.robots.iter().any(|r| {
                !r.combat.validate(self.elapsed_ns())
                    || !r.terrain.validate(self.elapsed_ns())
                    || r.combat.level > self.level_caps[r.team.index()]
                    || r.max_hp != r.stats(self.elapsed_ns()).max_hp
            })
            || self.base_hp.iter().any(|&v| v > BASE_HP)
            || self
                .base_lowest_hp
                .iter()
                .zip(self.base_hp)
                .any(|(&low, hp)| low > hp)
            || Team::BOTH
                .into_iter()
                .any(|side| self.fortress_reserve_used[side.index()] > self.fortress_capacity(side))
            || self.outpost_hp.iter().any(|&v| v > OUTPOST_HP)
            || ![-1, 1].contains(&self.rotor_direction)
            || self.countdown_ns > COUNTDOWN_NS
            || self.robots.windows(2).any(|w| w[0].id >= w[1].id)
            || self.robots.iter().any(|r| {
                r.hp > r.max_hp
                    || r.max_hp == 0
                    || (r.combat.heat_tenths
                        > u64::from(r.stats(self.elapsed_ns()).heat_limit) * 10
                        && !r.combat.overheated)
                    || (r.combat.heat_tenths
                        >= u64::from(r.stats(self.elapsed_ns()).heat_limit + 100) * 10
                        && !r.combat.heat_locked_for_round)
                    || (r.hp > 0 && r.combat.respawn.is_some())
                    || r.combat
                        .respawn
                        .as_ref()
                        .is_some_and(|p| p.original_ns == 0)
                    || (r.role == GroundRole::Infantry
                        && (r.combat.experience_tenths
                            > super::combat::EXPERIENCE_TENTHS
                                [usize::from(self.level_caps[r.team.index()] - 1)]
                            || usize::from(r.combat.level)
                                != super::combat::EXPERIENCE_TENTHS
                                    .iter()
                                    .take(usize::from(self.level_caps[r.team.index()]))
                                    .filter(|&&t| t <= r.combat.experience_tenths)
                                    .count()))
                    || r.fortress_ns.iter().any(|&p| p > FORTRESS_NS)
                    || r.expires_ns
                        .iter()
                        .flatten()
                        .any(|&t| t > self.elapsed_ns() + RFID_GRACE_NS)
                    || r.fortress_last_ns
                        .iter()
                        .flatten()
                        .any(|&t| t > self.elapsed_ns())
            })
            || self
                .rotor_stopped_ns
                .iter()
                .flatten()
                .any(|&t| t > self.elapsed_ns())
            || self
                .control
                .snapshot()
                .outposts
                .iter()
                .zip(self.outpost_hp)
                .any(|(o, hp)| o.alive != (hp > 0))
            || (self.phase == Phase::Finished) != self.result.is_some()
        {
            return Err("invalid 2027 match checkpoint");
        }
        Ok(())
    }
    /// Report a physical contact at the current round time. Loss retains its
    /// previous expiry; death, disconnection and removal revoke eligibility.
    pub fn observe(&mut self, id: u32, side: Team, point: Point, present: bool) {
        let now = self.elapsed_ns();
        let index = point_index(side, point);
        if self.phase == Phase::Running
            && present
            && let Some(r) = self
                .robots
                .iter_mut()
                .find(|r| r.id == id && r.hp > 0 && r.online)
        {
            r.expires_ns[index] = Some(now + RFID_GRACE_NS);
        }
    }
    fn eligible(&self, r: &Robot, side: Team, point: Point) -> bool {
        if r.hp == 0
            || !r.online
            || r.combat.weakened
            || !r.expires_ns[point_index(side, point)].is_some_and(|t| t > self.elapsed_ns())
        {
            return false;
        }
        match point {
            Point::Outpost => {
                self.control.snapshot().territories[side.index()].control
                    == ControlState::Full(r.team)
            }
            Point::Fortress => {
                let controller = self.control.snapshot().territories[side.index()]
                    .control
                    .team();
                controller != side
                    && (r.team == side
                        || (self.elapsed_ns() >= THREE_MIN_NS && controller == r.team))
            }
            _ => true,
        }
    }
    pub(super) fn resolve(&mut self) -> Vec<[bool; 8]> {
        if self.phase != Phase::Running || self.elapsed_ns() >= ROUND_NS {
            return vec![[false; 8]; self.robots.len()];
        }

        let active = self
            .robots
            .iter()
            .map(|r| std::array::from_fn(|i| self.eligible(r, Team::BOTH[i / 4], points()[i % 4])))
            .collect::<Vec<_>>();
        for side in Team::BOTH {
            let s = side.index();
            let index = point_index(side, Point::CentralHighland);
            if !self.highland_team[s].is_some_and(|team| {
                self.robots
                    .iter()
                    .zip(&active)
                    .any(|(r, a)| r.team == team && a[index])
            }) {
                self.highland_team[s] = self
                    .robots
                    .iter()
                    .zip(&active)
                    .find(|(_, a)| a[index])
                    .map(|(r, _)| r.team);
            }
            let index = point_index(side, Point::Fortress);
            if !self.fortress_robot[s].is_some_and(|id| {
                self.robots
                    .iter()
                    .zip(&active)
                    .any(|(r, a)| r.id == id && r.team == side && a[index])
            }) {
                self.fortress_robot[s] = self
                    .robots
                    .iter()
                    .zip(&active)
                    .find(|(r, a)| r.team == side && a[index])
                    .map(|(r, _)| r.id);
            }
        }
        let resolved = self
            .robots
            .iter()
            .zip(active)
            .map(|(r, mut a)| {
                for side in Team::BOTH {
                    a[point_index(side, Point::CentralHighland)] &=
                        self.highland_team[side.index()] == Some(r.team);
                    a[point_index(side, Point::Fortress)] &=
                        r.team != side || self.fortress_robot[side.index()] == Some(r.id);
                }
                a
            })
            .collect::<Vec<_>>();
        for side in Team::BOTH {
            for point in points() {
                let i = point_index(side, point);
                let teams = Team::BOTH.map(|team| {
                    self.robots
                        .iter()
                        .zip(&resolved)
                        .any(|(r, a)| r.team == team && a[i])
                });
                self.control
                    .set_activation(side, point, teams)
                    .expect("valid constituent point");
            }
        }
        resolved
    }
    /// Advance only explicit simulation time. Expiry boundaries split intervals
    /// so one call and any partition of it have the same rule result.
    pub fn advance_ns(&mut self, mut duration: u64) {
        if self.phase == Phase::Countdown {
            let used = duration.min(self.countdown_ns);
            self.countdown_ns -= used;
            duration -= used;
            if self.countdown_ns == 0 {
                self.phase = Phase::Running;
                self.gold = [800; 2];
                for rune in &mut self.runes {
                    rune.settle(0);
                }
            }
        }
        while duration > 0 && self.phase == Phase::Running {
            let now = self.elapsed_ns();
            self.resolve_service_recovery();
            let active = self.resolve();
            let mut dt = duration
                .min(ROUND_NS - now)
                .min(SCORE_PERIOD_NS - now % SCORE_PERIOD_NS);
            dt = dt.min(self.combat_boundary_ns());
            for rune in &self.runes {
                dt = dt.min(rune.boundary(now));
            }
            // A capture changes Outpost/Fortress eligibility immediately, even
            // between score or combat settlements (UC Table 5-21, §5.6.4.9).
            for territory in &self.control.snapshot().territories {
                if territory.direction() == 1 {
                    dt = dt.min(CAPTURE_NS - territory.progress_ns);
                }
            }
            if now < THREE_MIN_NS {
                dt = dt.min(THREE_MIN_NS - now);
            }
            for (r, a) in self.robots.iter().zip(&active) {
                for expiry in r.expires_ns.iter().flatten().filter(|&&t| t > now) {
                    dt = dt.min(*expiry - now);
                }
                for side in Team::BOTH {
                    let s = side.index();
                    if r.team != side && a[point_index(side, Point::Fortress)] && !self.base_open[s]
                    {
                        dt = dt.min(FORTRESS_NS - r.fortress_ns[s]);
                    } else if let Some(last) = r.fortress_last_ns[s]
                        && last + RETAIN_NS > now
                    {
                        dt = dt.min(last + RETAIN_NS - now);
                    }
                }
            }
            if dt == 0 {
                break;
            }
            for (r, a) in self.robots.iter_mut().zip(&active) {
                for side in Team::BOTH {
                    let s = side.index();
                    if r.team != side && a[point_index(side, Point::Fortress)] && !self.base_open[s]
                    {
                        r.fortress_ns[s] = (r.fortress_ns[s] + dt).min(FORTRESS_NS);
                        r.fortress_last_ns[s] = Some(now + dt);
                        if r.fortress_ns[s] == FORTRESS_NS {
                            self.base_open[s] = true;
                        }
                    } else if r.fortress_last_ns[s].is_none_or(|last| now >= last + RETAIN_NS) {
                        r.fortress_ns[s] = 0;
                        r.fortress_last_ns[s] = None;
                    }
                }
            }
            let events = self.control.advance_ns(dt);
            self.apply_events(events);
            let at = self.elapsed_ns();
            for side in Team::BOTH {
                if self.runes[side.index()].settle(at) {
                    self.rune_big_experience(side);
                }
            }
            self.advance_combat(dt);
            let now = self.elapsed_ns();
            for robot in &mut self.robots {
                robot.terrain.expire(now);
            }
            // Publish expiry at the endpoint itself, including when the caller
            // pauses or checkpoints without making another advance.
            let now = self.elapsed_ns();
            for robot in &mut self.robots {
                for side in Team::BOTH {
                    let i = side.index();
                    if robot.fortress_last_ns[i].is_some_and(|last| now >= last + RETAIN_NS) {
                        robot.fortress_ns[i] = 0;
                        robot.fortress_last_ns[i] = None;
                    }
                }
            }
            duration -= dt;
            for side in Team::BOTH {
                let i = side.index();
                let owner = self.control.snapshot().outposts[i].owner;
                if self.elapsed_ns() >= THREE_MIN_NS || self.base_open[owner.other().index()] {
                    self.rotor_stopped_ns[i].get_or_insert(self.elapsed_ns());
                }
            }
            if self.elapsed_ns() == ROUND_NS {
                self.finish();
            }
        }
        if self.phase == Phase::Running {
            self.resolve();
        }
    }
    fn apply_events(&mut self, events: Vec<Event>) {
        for event in events {
            if let EventKind::OutpostRebuilt { side, hp, .. } = event.kind {
                self.outpost_hp[side.index()] = hp;
            }
        }
    }
    /// Spend a native team Rune opportunity in a running round (§5.6.3).
    pub fn activate_rune(&mut self, team: Team) -> std::result::Result<(), &'static str> {
        if self.phase != Phase::Running {
            return Err("Rune activation requires a running round");
        }
        self.runes[team.index()].activate(self.elapsed_ns())
    }
    /// Resolve an eligible physical Rune strike at current running time. Benefits belong
    /// to the face's team regardless of which ground robot fired (§5.6.3).
    pub fn hit_rune(&mut self, team: Team, blade: u8, ring: u8) -> super::rune::Hit {
        if self.phase != Phase::Running {
            return super::rune::Hit::Ignored;
        }
        let before = self.runes[team.index()].big_successes;
        let result = self.runes[team.index()].hit(self.elapsed_ns(), blade, ring);
        if self.runes[team.index()].big_successes > before {
            self.rune_big_experience(team);
        }
        result
    }
    fn rune_big_experience(&mut self, team: Team) {
        let ids = self
            .robots
            .iter()
            .filter(|r| r.team == team && r.hp > 0 && r.role == GroundRole::Infantry)
            .map(|r| r.id)
            .collect::<Vec<_>>();
        if !ids.is_empty() {
            let xp = (7500 + ids.len() as u64 / 2) / ids.len() as u64;
            for id in ids {
                self.grant_experience(id, xp);
            }
        }
    }
    /// Current activation flags after eligibility and central-highland arbitration.
    pub fn activations(&mut self) -> Vec<(u32, [bool; 8])> {
        let resolved = self.resolve();
        self.robots
            .iter()
            .zip(resolved)
            .map(|(r, a)| (r.id, a))
            .collect()
    }
    /// Apply Table 5-2 structure and field-adjusted robot projectile damage. The caller
    /// supplies the calibrated armor contact and caliber. 42 mm remains disabled
    /// in this subset until a Heavy launcher can report its suppression state.
    pub fn hit(
        &mut self,
        target: crate::Target,
        shooter: Option<u32>,
        caliber: crate::Caliber,
    ) -> u32 {
        if self.phase != Phase::Running || caliber != crate::Caliber::Mm17 {
            return 0;
        }
        let attack = shooter.and_then(|id| self.robots.iter().find(|r| r.id == id).map(|r| r.team));
        let attack_pct = attack
            .and_then(|t| self.runes[t.index()].buff.as_ref())
            .map_or(100, |b| b.attack_pct);
        let structure_damage = |team: Team| {
            let defense = self.runes[team.index()]
                .buff
                .as_ref()
                .map_or(0, |b| b.defense_pct);
            (20 * attack_pct * (100 - defense) + 5000) / 10000
        };
        let (team, applied) = match target {
            crate::Target::Base(team) => {
                if self.control.base_protected(team) {
                    return 0;
                }
                let damage = structure_damage(team);
                let hp = &mut self.base_hp[team.index()];
                let d = (*hp).min(damage);
                *hp -= d;
                self.base_lowest_hp[team.index()] = self.base_lowest_hp[team.index()].min(*hp);
                (team, d)
            }
            crate::Target::Outpost(side) => {
                let team = self.control.snapshot().outposts[side.index()].owner;
                let damage = structure_damage(team);
                let hp = &mut self.outpost_hp[side.index()];
                let d = (*hp).min(damage);
                *hp -= d;
                if *hp == 0 && d > 0 {
                    self.rotor_stopped_ns[side.index()].get_or_insert(self.elapsed_ns());
                    if let Ok(events) = self.control.outpost_destroyed(side) {
                        self.apply_events(events);
                    }
                }
                (team, d)
            }
            crate::Target::Robot(id) => {
                let Some(r) = self.robots.iter().find(|r| r.id == id) else {
                    return 0;
                };
                let team = r.team;
                let d = self.damage_robot(id, (20 * attack_pct + 50) / 100, false);
                (team, d)
            }
        };
        if let Some(attacker) = attack
            && attacker != team
        {
            self.attack_damage[attacker.index()] += u64::from(applied);
        }
        self.reward_attack(target, shooter, team, applied);
        // A lethal contact revokes activation in this same event, even when
        // a caller publishes or pauses before the next simulation advance.
        self.resolve();
        if self.base_hp.contains(&0) {
            self.finish();
        }
        applied
    }
    /// Set an operator HP override without crediting attack damage. A lethal
    /// outpost override still applies the ownership/destruction transition.
    pub fn set_outpost_hp(&mut self, side: Team, hp: u32) {
        let i = side.index();
        // Only the control component may rebuild a destroyed physical outpost.
        if self.outpost_hp[i] == 0 && hp > 0 {
            return;
        }
        let destroyed = self.outpost_hp[i] > 0 && hp == 0;
        self.outpost_hp[i] = hp.min(OUTPOST_HP);
        if destroyed {
            self.rotor_stopped_ns[i].get_or_insert(self.elapsed_ns());
            if let Ok(events) = self.control.outpost_destroyed(side) {
                self.apply_events(events);
            }
        }
    }
    /// Set a base's HP directly; zero ends a running round, without attack credit.
    pub fn set_base_hp(&mut self, team: Team, hp: u32) {
        self.base_hp[team.index()] = hp.min(BASE_HP);
        self.base_lowest_hp[team.index()] = self.base_lowest_hp[team.index()].min(hp.min(BASE_HP));
        if self.phase == Phase::Running && hp == 0 {
            self.finish();
        }
    }
    /// Settle UC §5.8 in order. Surviving base HP never breaks a timeout tie.
    pub fn finish(&mut self) {
        let comparison = if self.base_hp.contains(&0) {
            vec![(self.base_hp.map(u64::from), ResultReason::BaseDestroyed)]
        } else {
            vec![
                (
                    self.control.snapshot().points.map(u64::from),
                    ResultReason::Control,
                ),
                (self.attack_damage, ResultReason::AttackDamage),
                (
                    Team::BOTH.map(|team| {
                        self.robots
                            .iter()
                            .filter(|r| r.team == team)
                            .map(|r| u64::from(r.hp))
                            .sum()
                    }),
                    ResultReason::RobotHp,
                ),
            ]
        };
        let result = comparison
            .into_iter()
            .find(|(v, _)| v[0] != v[1])
            .map(|(v, reason)| Result {
                winner: Some(if v[0] > v[1] { Team::Red } else { Team::Blue }),
                reason,
            })
            .unwrap_or(Result {
                winner: None,
                reason: ResultReason::Draw,
            });
        self.result = Some(result);
        self.phase = Phase::Finished;
    }
}
/// Point slot in one side's four activation inputs.
pub const fn points() -> [Point; 4] {
    [
        Point::Outpost,
        Point::CentralHighland,
        Point::Base,
        Point::Fortress,
    ]
}
/// Flat RFID slot by original field side, then constituent point.
pub const fn point_index(side: Team, point: Point) -> usize {
    side.index() * 4
        + match point {
            Point::Outpost => 0,
            Point::CentralHighland => 1,
            Point::Base => 2,
            Point::Fortress => 3,
        }
}

#[cfg(test)]
mod tests {
    use super::*;
    const SECOND: u64 = 1_000_000_000;
    fn running() -> Match {
        let mut g = Match::new(-1);
        g.add_robot(1, Team::Red, GroundRole::Infantry);
        g.add_robot(2, Team::Blue, GroundRole::Infantry);
        g.phase = Phase::Running;
        g
    }
    fn hold(g: &mut Match, contacts: &[(u32, Team, Point)], seconds: u64) {
        for _ in 0..seconds * 10 {
            for &(id, side, point) in contacts {
                g.observe(id, side, point, true);
            }
            g.advance_ns(SECOND / 10);
        }
    }
    #[test]
    fn countdown_and_expiry_survive_roundtrip_and_partitioning() {
        let mut a = running();
        a.start(-1);
        a.advance_ns(COUNTDOWN_NS - 1);
        assert_eq!(a.phase, Phase::Countdown);
        assert_eq!(a.elapsed_ns(), 0);
        a.advance_ns(1);
        assert_eq!(a.phase, Phase::Running);
        a.observe(1, Team::Blue, Point::Base, true);
        let mut b: Match = serde_json::from_slice(&serde_json::to_vec(&a).unwrap()).unwrap();
        a.advance_ns(3 * SECOND);
        for t in [1, 123_456_789, 1_876_543_210, SECOND] {
            b.advance_ns(t);
        }
        assert_eq!(a, b);
        a.validate().unwrap();
        assert_eq!(a.control.snapshot().territories[3].progress_ns, SECOND);
        assert_eq!(
            a.control.snapshot().territories[3].activations,
            [[false; 2]; 2]
        );
    }
    #[test]
    fn temporary_control_scores_and_transferred_outpost_rebuilds_once() {
        let mut g = running();
        hold(&mut g, &[(1, Team::Blue, Point::CentralHighland)], 5);
        assert_eq!(
            g.control.snapshot().territories[1].control,
            ControlState::Temporary(Team::Red)
        );
        assert_eq!(g.control.snapshot().points, [1, 0]);
        for _ in 0..50 {
            assert_eq!(
                g.hit(
                    crate::Target::Outpost(Team::Blue),
                    Some(1),
                    crate::Caliber::Mm17
                ),
                20
            );
        }
        assert_eq!(
            g.control.snapshot().outposts[1],
            OutpostState {
                owner: Team::Red,
                alive: true
            }
        );
        assert_eq!(g.outpost_hp, [1000, 500]);
        assert_eq!(g.rotor_stopped_ns[1], Some(5 * SECOND));
        hold(&mut g, &[(1, Team::Blue, Point::CentralHighland)], 2);
        assert_eq!(g.control.snapshot().points, [3, 0]);
        assert_eq!(g.rotor_stopped_ns[1], Some(5 * SECOND));
        g.validate().unwrap();
    }
    #[test]
    fn lethal_projectile_clears_activation_without_another_advance() {
        let mut g = running();
        g.observe(1, Team::Blue, Point::Base, true);
        g.advance_ns(1);
        assert_eq!(
            g.control.snapshot().territories[3].activations[0],
            [true, false]
        );
        g.robots.iter_mut().find(|r| r.id == 1).unwrap().hp = 20;
        assert_eq!(
            g.hit(crate::Target::Robot(1), Some(2), crate::Caliber::Mm17),
            20
        );
        assert_eq!(
            g.control.snapshot().territories[3].activations,
            [[false; 2]; 2]
        );
    }
    #[test]
    fn base_protection_needs_full_own_outpost_control_and_uses_flat_2027_damage() {
        let mut g = running();
        assert_eq!(
            g.hit(
                crate::Target::Base(Team::Blue),
                Some(1),
                crate::Caliber::Mm17
            ),
            0
        );
        hold(&mut g, &[(1, Team::Blue, Point::CentralHighland)], 5);
        assert_eq!(
            g.hit(
                crate::Target::Base(Team::Blue),
                Some(1),
                crate::Caliber::Mm17
            ),
            20
        );
        assert_eq!(g.base_hp[1], 9980);
        assert_eq!(g.attack_damage[0], 20);
        assert_eq!(
            g.hit(
                crate::Target::Base(Team::Blue),
                Some(1),
                crate::Caliber::Mm42
            ),
            0
        );
    }
    #[test]
    fn central_highland_is_mutually_exclusive_but_other_points_contest() {
        let mut g = running();
        g.observe(2, Team::Blue, Point::CentralHighland, true);
        g.advance_ns(1);
        g.observe(1, Team::Blue, Point::CentralHighland, true);
        let active = g.activations();
        assert!(!active[0].1[5]);
        assert!(active[1].1[5]);
        g.observe(1, Team::Blue, Point::Base, true);
        g.observe(2, Team::Blue, Point::Base, true);
        g.advance_ns(SECOND);
        assert_eq!(g.control.snapshot().territories[3].progress_ns, 0);
        assert_eq!(
            g.control.snapshot().territories[3].activations[0],
            [true, true]
        );
        g.robots[1].hp = 0;
        g.advance_ns(1);
        assert_eq!(g.highland_team[1], Some(Team::Red));
        assert!(!g.activations()[1].1.iter().any(|a| *a));
    }
    #[test]
    fn ineligible_outpost_and_early_enemy_fortress_cannot_capture() {
        let mut g = running();
        hold(
            &mut g,
            &[
                (1, Team::Blue, Point::Outpost),
                (1, Team::Blue, Point::Fortress),
            ],
            10,
        );
        assert_eq!(
            g.control.snapshot().territories[1].control,
            ControlState::Full(Team::Blue)
        );
        assert_eq!(g.control.snapshot().territories[3].progress_ns, 0);
        assert!(!g.base_open[1]);
    }
    #[test]
    fn opposing_fortress_opens_after_ten_seconds_and_stops_rotors() {
        let mut g = running();
        g.add_robot(3, Team::Red, GroundRole::Infantry);
        g.advance_ns(THREE_MIN_NS);
        assert_eq!(g.rotor_stopped_ns, [Some(THREE_MIN_NS); 2]);
        hold(&mut g, &[(1, Team::Blue, Point::CentralHighland)], 5);
        hold(
            &mut g,
            &[
                (1, Team::Blue, Point::CentralHighland),
                (3, Team::Blue, Point::Fortress),
            ],
            9,
        );
        assert!(!g.base_open[1]);
        hold(
            &mut g,
            &[
                (1, Team::Blue, Point::CentralHighland),
                (3, Team::Blue, Point::Fortress),
            ],
            1,
        );
        assert!(g.base_open[1]);
    }
    #[test]
    fn outpost_destruction_changes_full_to_temporary_then_absence_transfers() {
        let mut g = running();
        g.set_outpost_hp(Team::Red, 0);
        assert_eq!(
            g.control.snapshot().territories[0].control,
            ControlState::Temporary(Team::Red)
        );
        assert!(!g.control.base_protected(Team::Red));
        g.advance_ns(5 * SECOND);
        assert_eq!(
            g.control.snapshot().territories[0].control,
            ControlState::Full(Team::Blue)
        );
        assert_eq!(g.outpost_hp[0], 500);
        assert_eq!(g.rotor_stopped_ns[0], Some(0));
    }
    #[test]
    fn timeout_ignores_surviving_base_hp_and_uses_attack_then_robot_hp() {
        let mut g = running();
        g.base_hp = [100, 9999];
        g.advance_ns(ROUND_NS);
        assert_eq!(
            g.result,
            Some(Result {
                winner: None,
                reason: ResultReason::Draw
            })
        );
        let mut g = running();
        g.attack_damage = [1, 0];
        g.advance_ns(ROUND_NS);
        assert_eq!(
            g.result.as_ref().unwrap().reason,
            ResultReason::AttackDamage
        );
        let mut g = running();
        g.robots[1].hp = 199;
        g.advance_ns(ROUND_NS);
        assert_eq!(g.result.as_ref().unwrap().reason, ResultReason::RobotHp);
    }
    #[test]
    fn timeout_control_precedes_attack_damage() {
        let mut g = running();
        hold(&mut g, &[(1, Team::Blue, Point::CentralHighland)], 5);
        g.attack_damage = [0, 10_000];
        g.finish();
        assert_eq!(
            g.result,
            Some(Result {
                winner: Some(Team::Red),
                reason: ResultReason::Control
            })
        );
    }
    #[test]
    fn fortress_progress_pauses_for_three_seconds_after_death_then_resets() {
        let mut g = running();
        g.add_robot(3, Team::Red, GroundRole::Infantry);
        g.advance_ns(THREE_MIN_NS);
        hold(&mut g, &[(1, Team::Blue, Point::CentralHighland)], 5);
        hold(
            &mut g,
            &[
                (1, Team::Blue, Point::CentralHighland),
                (3, Team::Blue, Point::Fortress),
            ],
            4,
        );
        let mut expired = g.clone();
        g.robots[2].hp = 0;
        hold(&mut g, &[(1, Team::Blue, Point::CentralHighland)], 2);
        assert_eq!(g.robots[2].fortress_ns[1], 4 * SECOND);
        g.robots[2].hp = 200;
        hold(
            &mut g,
            &[
                (1, Team::Blue, Point::CentralHighland),
                (3, Team::Blue, Point::Fortress),
            ],
            6,
        );
        assert!(g.base_open[1]);
        expired.robots[2].hp = 0;
        hold(&mut expired, &[(1, Team::Blue, Point::CentralHighland)], 4);
        assert_eq!(expired.robots[2].fortress_ns[1], 0);
        expired.robots[2].hp = 200;
        hold(
            &mut expired,
            &[
                (1, Team::Blue, Point::CentralHighland),
                (3, Team::Blue, Point::Fortress),
            ],
            6,
        );
        assert!(!expired.base_open[1]);
    }
    #[test]
    fn offline_robot_loses_eligibility_without_waiting_for_grace() {
        let mut g = running();
        g.observe(1, Team::Blue, Point::Base, true);
        g.advance_ns(1);
        assert_eq!(
            g.control.snapshot().territories[3].activations[0],
            [true, false]
        );
        g.robots[0].online = false;
        g.advance_ns(1);
        assert_eq!(
            g.control.snapshot().territories[3].activations[0],
            [false, false]
        );
        g.validate().unwrap();
    }
    #[test]
    fn lethal_base_override_finishes_immediately_and_round_never_runs_past_timeout() {
        let mut g = running();
        g.set_base_hp(Team::Blue, 0);
        let result = g.result.clone();
        g.advance_ns(ROUND_NS * 2);
        assert_eq!(g.elapsed_ns(), 0);
        assert_eq!(g.result, result);
        assert_eq!(result.unwrap().winner, Some(Team::Red));
        let mut g = running();
        g.advance_ns(ROUND_NS * 2);
        assert_eq!(g.elapsed_ns(), ROUND_NS);
    }
}
