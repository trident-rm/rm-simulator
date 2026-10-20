// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared referee commands and presentation records; native rules are in `rmuc_2027`.
use crate::rune::RuneKind;
use rm_simulator_gameplay as gp;
use serde::{Deserialize, Serialize};

/// Native simulator countdown duration, nanoseconds.
pub const COUNTDOWN_NS: u64 = gp::rmuc_2027::match_rules::COUNTDOWN_NS;
/// Native RMUC round duration, nanoseconds.
pub const ROUND_NS: u64 = gp::rmuc_2027::ROUND_NS;

pub use rm_simulator_physics::Team;

/// The robot classes a chassis can be recorded as. Every chassis names its
/// class in its [`crate::ChassisPlacement`]; the class fixes the gameplay
/// robot's weapon, levelling and default performance (section 5.4.2).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum RobotKind {
    /// Hero: 42 mm gun.
    Hero,
    /// Engineer: no launcher.
    Engineer,
    /// Infantry: 17 mm gun. The default class of a placement that names none.
    #[default]
    Infantry,
    /// Sentry: 17 mm gun.
    Sentry,
    /// Aerial drone: 17 mm gun that needs air support to fire.
    Drone,
}
impl RobotKind {
    /// The gameplay engine's robot type for this class.
    pub fn gameplay(self) -> gp::RobotKind {
        match self {
            Self::Hero => gp::RobotKind::Hero,
            Self::Engineer => gp::RobotKind::Engineer,
            Self::Infantry => gp::RobotKind::Infantry,
            Self::Sentry => gp::RobotKind::Sentry,
            Self::Drone => gp::RobotKind::Drone,
        }
    }
    /// Convert a shared serialized model identity into its physical kind.
    pub fn from_gameplay(kind: gp::RobotKind) -> Self {
        match kind {
            gp::RobotKind::Hero => Self::Hero,
            gp::RobotKind::Engineer => Self::Engineer,
            gp::RobotKind::Sentry => Self::Sentry,
            gp::RobotKind::Drone => Self::Drone,
            _ => Self::Infantry,
        }
    }
}

/// Performance for robots the section 5.4.2 tables do not cover. Hero,
/// Infantry, Engineer and Sentry use their rulebook values unless the
/// placement selects a performance; anything else uses `fallback`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RobotConfig {
    /// HP, power, heat limit and cooling for a robot without rulebook values.
    pub fallback: gp::Stats,
}
impl Default for RobotConfig {
    /// A 200 HP robot with a 100 heat limit cooling at 20 per second.
    fn default() -> Self {
        Self {
            fallback: gp::Stats {
                max_hp: 200,
                chassis_power_w: 0,
                heat_limit: 100,
                cooling_per_s: 20,
            },
        }
    }
}

/// Match rules for one field: who owns each rune and outpost, what every
/// chassis becomes, how long the round lasts and the seed the runes draw from.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RefereeConfig {
    /// Owner of each field rune, by rune index.
    pub rune_teams: Vec<Team>,
    /// Owner of each field outpost, by outpost index; at most one per team.
    pub outpost_teams: Vec<Team>,
    /// Performance for robot classes without rulebook values.
    pub robot: RobotConfig,
    /// Round length in ns. Section 6.6 gives 7 min; the constructor refuses
    /// zero and anything longer.
    pub round_ns: u64,
    /// Countdown before the round in ns. Section 6.5 gives 5 s.
    pub countdown_ns: u64,
    /// Seed for target selection and Big Rune motion parameters.
    pub seed: u64,
}
impl Default for RefereeConfig {
    fn default() -> Self {
        Self::alternating(1, 2)
    }
}
impl RefereeConfig {
    /// Rune and outpost `i` belong to red when `i` is even, else blue.
    pub fn alternating(runes: usize, outposts: usize) -> Self {
        let team = |i: usize| {
            if i.is_multiple_of(2) {
                Team::Red
            } else {
                Team::Blue
            }
        };
        Self::owned(
            (0..runes).map(team).collect(),
            (0..outposts).map(team).collect(),
        )
    }

    /// Explicit owners per rune and outpost index, with the default robot,
    /// clock and seed.
    pub fn owned(rune_teams: Vec<Team>, outpost_teams: Vec<Team>) -> Self {
        Self {
            rune_teams,
            outpost_teams,
            robot: RobotConfig::default(),
            round_ns: ROUND_NS,
            countdown_ns: COUNTDOWN_NS,
            seed: 0x5EED_2026,
        }
    }
}

/// Where the match is in the section 6.5 and 6.6 sequence.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum MatchPhase {
    /// No match: runes run their training policy and damage is free practice.
    Idle,
    /// A started match waiting out its countdown.
    Countdown,
    /// The round clock is running.
    Running,
    /// The round ended by timeout, base destruction or an operator stop; the
    /// round clock is frozen at that instant.
    Finished,
}

/// Which Power Rune stage the round is in. Every rune converts at 3:00
/// (section 5.5.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RuneStage {
    /// From the start until 3:00: one target at a time.
    Small,
    /// From 3:00 to the end: pairs of targets.
    Big,
}
impl RuneStage {
    /// The rune mode this stage runs.
    pub fn kind(self) -> RuneKind {
        match self {
            RuneStage::Small => RuneKind::Small,
            RuneStage::Big => RuneKind::Big,
        }
    }
}

/// An active rune buff (section 5.5.2). The gameplay engine applies the
/// attack, defense and cooling effects to the whole team; this record drives
/// the rune's presentation. Times are on the round clock
/// (`RefereeSnapshot::match_time_ns`), so a `SkipTo` ages the buff with the round.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BuffState {
    /// Which stage granted the buff.
    pub source: RuneStage,
    /// Share of incoming damage removed, in percent.
    pub defense_pct: u32,
    /// Projectile damage multiplier in percent; 100 is no attack buff.
    pub attack_pct: u32,
    /// Barrel cooling multiplier.
    pub cooling_multiplier: u32,
    /// Round clock time the buff was granted.
    pub started_ns: u64,
    /// Round clock time the buff ends.
    pub expires_ns: u64,
}

/// One team's round state: rune opportunities, activations and the current buff.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TeamSnapshot {
    /// Which team this is.
    pub team: Team,
    /// Unspent Small and Big Rune opportunities; they accumulate.
    pub rune_opportunities: u32,
    /// Big Rune activations completed, which narrows the detecting rings.
    pub big_rune_activations: u32,
    /// Round clock time the rune was triggered, while it is Activating.
    pub rune_activating_since_ns: Option<u64>,
    /// The team's active buff, when it has one.
    pub buff: Option<BuffState>,
}

/// One chassis' robot record, summarised from the gameplay engine's state.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RobotSnapshot {
    /// Chassis id the robot was opened under.
    pub id: u32,
    /// Team the chassis plays for.
    pub team: Team,
    /// Configured robot class.
    pub kind: RobotKind,
    /// Remaining HP; zero means defeated.
    pub hp: u32,
    /// Maximum HP at the robot's current level (section 5.4.2).
    pub max_hp: u32,
    /// Current level (Table 5-11).
    pub level: u8,
    /// Whether the robot is weakened after a respawn (section 5.2.2).
    pub weakened: bool,
}
impl RobotSnapshot {
    /// Whether the robot has any HP left.
    pub fn alive(&self) -> bool {
        self.hp > 0
    }
}

/// Something the referee decided or observed, kept in the snapshot's event
/// list for the panel. Events are informational; the authoritative state is
/// the snapshot's own fields.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum RefereeEvent {
    /// A base lost HP or shield.
    BaseDamaged {
        /// Team whose base was hit.
        team: Team,
        /// HP plus shield removed, after the team's defenses.
        amount: u32,
        /// Base HP after the hit.
        hp: u32,
        /// Base shield after the hit.
        shield_hp: u32,
        /// The chassis whose projectile did it; `None` otherwise.
        shooter: Option<u32>,
    },
    /// A base reached zero HP.
    BaseDestroyed {
        /// Team whose base was destroyed.
        team: Team,
    },
    /// `StartMatch` entered Countdown.
    CountdownStarted,
    /// The countdown elapsed and the round clock started.
    MatchStarted,
    /// The round ended.
    MatchFinished,
    /// The section 5.8 result of the round, or a referee's decision.
    RoundResult(gp::RoundResult),
    /// `ResetMatch` returned the field to Idle.
    MatchReset,
    /// The round reached 3:00 and the runes converted to Big.
    RuneStage(RuneStage),
    /// A team received a rune opportunity.
    RuneOpportunity {
        /// Team that received it.
        team: Team,
        /// Stage the opportunity belongs to.
        stage: RuneStage,
        /// Opportunities the team now holds.
        total: u32,
    },
    /// A team spent an opportunity and its rune began Activating.
    RuneActivating {
        /// Team that spent the opportunity.
        team: Team,
    },
    /// Twenty seconds passed without activating the rune (section 5.5.2).
    RuneActivationExpired {
        /// Team whose attempt expired.
        team: Team,
    },
    /// A rune completed and granted its buff.
    RuneActivated {
        /// Team that activated it.
        team: Team,
        /// Stage the rune was in.
        stage: RuneStage,
        /// Arms lit: blades hit on a Small Rune, rings recorded on a Big Rune.
        arms: u32,
        /// Mean ring of the recorded hits; zero when none were recorded.
        average_ring: f64,
        /// The buff that was granted.
        buff: BuffState,
    },
    /// A team's buff ran out and its rune went dark.
    BuffExpired {
        /// Team whose buff expired.
        team: Team,
    },
    /// A robot lost HP.
    RobotDamaged {
        /// Chassis id of the robot.
        robot: u32,
        /// HP removed, after defenses.
        amount: u32,
        /// Robot HP after the hit.
        hp: u32,
        /// The chassis whose projectile did it; `None` otherwise.
        shooter: Option<u32>,
    },
    /// A chassis joined the field and got a robot record.
    RobotJoined {
        /// Chassis id the robot was opened under.
        robot: u32,
        /// Team the chassis plays for.
        team: Team,
    },
    /// A chassis left the field and its robot record was dropped.
    RobotLeft {
        /// Chassis id that left.
        robot: u32,
    },
    /// A robot reached zero HP.
    RobotDefeated {
        /// Chassis id of the robot.
        robot: u32,
    },
    /// An operator restored a defeated robot.
    RobotRevived {
        /// Chassis id of the robot.
        robot: u32,
    },
    /// A defeated robot returned through its respawn timer or a paid
    /// instant respawn (section 5.2.2).
    RobotRespawned {
        /// Chassis id of the robot.
        robot: u32,
    },
    /// A robot's weakened state ended.
    WeaknessCleared {
        /// Chassis id of the robot.
        robot: u32,
    },
    /// A robot reached a new level (Table 5-11).
    LevelUp {
        /// Chassis id of the robot.
        robot: u32,
        /// New level.
        level: u8,
    },
    /// An outpost reached zero HP.
    OutpostDestroyed {
        /// Outpost index in the field's outpost list.
        outpost: u32,
        /// Team that owned it.
        team: Team,
    },
    /// A destroyed outpost was rebuilt with 750 HP (section 5.5.1).
    OutpostRebuilt {
        /// Team whose outpost was rebuilt.
        team: Team,
    },
    /// A robot completed a terrain crossing and gained its buff (section
    /// 5.5.3.5).
    TerrainCrossing {
        /// Robot that crossed.
        robot: u32,
        /// Crossing kind.
        kind: gp::ZoneKind,
    },
    /// Twenty seconds of opposing Fortress occupation expanded a team's Base
    /// Protective Armor (section 5.5.3.9).
    BaseArmorExpanded {
        /// Team whose base armor expanded.
        team: Team,
    },
}

/// One event with the field time, and the round time when a round was running.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TimedEvent {
    /// Field time the event was recorded at.
    pub time_ns: u64,
    /// Round time, when the round was running.
    pub match_time_ns: Option<u64>,
    /// What happened.
    pub event: RefereeEvent,
}

/// The referee's public state at one tick: the round clock, both teams, the
/// robot records, ownership, the gameplay state and the recent events.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RefereeSnapshot {
    /// Whether each base's protective armor is open or opening, indexed red
    /// then blue: an operator override or a Fortress capture.
    pub base_open: [bool; 2],
    /// When each base set off toward its `base_open` state, on the field
    /// clock; `None` once it rests there. See [`Self::base_open_fraction`].
    pub base_moved_ns: [Option<u64>; 2],
    /// Dart door overrides, indexed red then blue; open by default.
    pub dart_door_open: [bool; 2],
    /// When both Dart Detection Modules started sweeping along their rails,
    /// on the field clock; `None` while they rest, the default. See
    /// [`Self::dart_target_position`].
    pub dart_target_since_ns: Option<u64>,
    /// The gameplay engine's whole state: gold, HP, experience, heat,
    /// respawn timers, buffs and the round result.
    pub game: gp::Snapshot,
    /// Where the match is.
    pub phase: MatchPhase,
    /// Elapsed round time; frozen at the end of a finished match.
    pub match_time_ns: u64,
    /// Round time left, `round_ns` minus `match_time_ns`.
    pub remaining_ns: u64,
    /// Section 6.5 countdown time left while in [`MatchPhase::Countdown`];
    /// zero in every other phase.
    pub countdown_remaining_ns: u64,
    /// Small Rune stage or Big Rune stage.
    pub stage: RuneStage,
    /// Both teams, indexed red then blue.
    pub teams: [TeamSnapshot; 2],
    /// Every chassis' robot record, in join order.
    pub robots: Vec<RobotSnapshot>,
    /// Owner of each rune, by rune index.
    pub rune_teams: Vec<Team>,
    /// Owner of each outpost, by outpost index.
    pub outpost_teams: Vec<Team>,
    /// Oldest first.
    pub events: Vec<TimedEvent>,
}

pub use rm_simulator_physics::motion::{
    BASE_TRAVEL_NS, DART_TARGET_PERIOD_NS, DART_TARGET_REST, Mechanism, MechanismState,
    MotionFrame, dart_target_fraction, dart_target_position,
};

impl RefereeSnapshot {
    /// Eased travel of team `index`'s base armor at field time `time_ns`, 0
    /// shut to 1 open, moving over [`BASE_TRAVEL_NS`].
    pub fn base_open_fraction(&self, index: usize, time_ns: u64) -> f64 {
        rm_simulator_physics::motion::base_open_fraction(
            self.base_open[index],
            self.base_moved_ns[index],
            time_ns,
        )
    }
    /// Dart rail position at field time `time_ns`, 0 to 1 along the rail and
    /// [`DART_TARGET_REST`] while not sweeping.
    pub fn dart_target_position(&self, time_ns: u64) -> f64 {
        dart_target_position(self.dart_target_since_ns, time_ns)
    }
}

/// Resolve public match state for presentation clearance queries.
pub fn mechanism_view(referee: Option<&RefereeSnapshot>, time_ns: u64) -> MechanismState {
    MechanismState {
        rune_angle_rad: [0.; 2],
        outpost_angle_rad: [0.; 2],
        base_open_fraction: referee.map_or([0.0; 2], |r| {
            std::array::from_fn(|i| r.base_open_fraction(i, time_ns))
        }),
        dart_door_open: referee.map_or([true; 2], |r| r.dart_door_open),
        dart_target_fraction: referee.map_or(DART_TARGET_REST, |r| r.dart_target_position(time_ns)),
    }
}

/// Operator and organiser inputs. `Field::referee_command` mirrors base and
/// outpost HP onto the physical objects after the referee applies them.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum RefereeCommand {
    /// Set a base's HP and shield directly; zero HP in a round ends it.
    SetBaseHp {
        /// Team whose base to edit.
        team: Team,
        /// New HP, at most `base::INITIAL_HP`.
        hp: u32,
        /// New shield, at most `base::INITIAL_HP`.
        shield_hp: u32,
    },
    /// Manual CAD mechanism override, independent of match rules.
    SetBaseOpen {
        /// Team whose base cover to move.
        team: Team,
        /// Whether the cover stands open.
        open: bool,
    },
    /// Manual CAD mechanism override for the dart door.
    SetDartDoorOpen {
        /// Team whose dart door to move.
        team: Team,
        /// Whether the door stands open.
        open: bool,
    },
    /// Start or stop both Dart Detection Modules sweeping along their rails.
    /// They rest by default; a sweep sets off from rest, and stopping returns
    /// them there. An app training control; the rulebook's dart target modes
    /// are not modelled.
    SetDartTargetMoving {
        /// Whether the targets sweep.
        moving: bool,
    },
    /// Set an outpost's HP directly. In training a restored tower spins
    /// again; in a match a stopped rotor stays stopped.
    SetOutpostHp {
        /// Outpost index in the field's outpost list.
        outpost: u32,
        /// New HP, at most `outpost::INITIAL_HP`.
        hp: u32,
    },
    /// Set a team's gold.
    SetGold {
        /// Team to edit.
        team: Team,
        /// New balance.
        gold: u32,
    },
    /// Set a robot's projectile allowance, 17 mm then 42 mm.
    SetAllowance {
        /// Chassis id of the robot.
        robot: u32,
        /// New allowance in caliber order.
        allowance: [u32; 2],
    },
    /// Replace the rule switches; the live default leaves allowance
    /// unenforced and lets pilots buy ammunition anywhere.
    SetPolicy(gp::Policy),
    /// Select performance: 2026 §5.4.2 refuses changes while running;
    /// native 2027 §5.5.2 instead locks each selected component for the round.
    SetPerformance {
        /// Chassis id of the robot.
        robot: u32,
        /// New performance; it must fit the robot's class.
        performance: gp::Performance,
    },
    /// Buy rounds for a robot's allowance with team gold at its own
    /// service zone (section 5.3.2, Tables 5-7 to 5-9).
    BuyAmmo {
        /// Chassis id of the robot.
        robot: u32,
        /// Caliber bought.
        caliber: crate::Caliber,
        /// Rounds to buy; 17 mm zone units are ten, native 2027 remote units 100.
        amount: u32,
    },
    /// Buy delayed HP recovery in the native RMUC 2027 ruleset (§5.3.1).
    BuyHp {
        /// Chassis id of the purchasing robot.
        robot: u32,
    },
    /// Set a team's unspent rune opportunities.
    SetRuneOpportunities {
        /// Team to edit.
        team: Team,
        /// Opportunities the team should hold.
        opportunities: u32,
    },
    /// Operator override; zero duration clears the buff and deactivates the rune.
    SetRuneBuff {
        /// Team to grant the buff to.
        team: Team,
        /// Defense share in percent, at most 100.
        defense_pct: u32,
        /// Attack multiplier in percent, at most 1000.
        attack_pct: u32,
        /// Cooling multiplier, at most 100.
        cooling_multiplier: u32,
        /// Buff duration in ns, at most the round length.
        duration_ns: u64,
    },
    /// Idle -> Countdown -> Running.
    StartMatch,
    /// Skip the round clock forward (testing), at most to the end of the
    /// round. Missed opportunities are granted, and buffs, activation
    /// windows and every gameplay timer age with it; the runes' own hit
    /// windows keep world time.
    SkipTo {
        /// Round time to jump to, at most `round_ns`.
        match_time_ns: u64,
    },
    /// End a counting-down or running match now.
    FinishMatch,
    /// Decide a finished round the section 5.8 comparison could not.
    Adjudicate {
        /// Winning team, or `None` for a draw.
        winner: Option<Team>,
    },
    /// Back to Idle: runes return to their training policy.
    ResetMatch,
    /// Spend one opportunity to put the team's rune into Activating.
    ActivateRune {
        /// Team whose rune to trigger.
        team: Team,
    },
    /// Spend a native 2027 Radar reinforcement opportunity; referee proxy action.
    SelectReinforcement {
        /// Team whose armor receives this center and selection-time area.
        team: Team,
        /// Common bottom-left plate coordinates, right/up, in micrometres.
        center_um: [u32; 2],
    },
    /// Start a native 2027 virtual assembly; substitutes for positioning a core.
    StartAssembly {
        /// Team attempting assembly.
        team: Team,
        /// Difficulty, 1–4 (UC Table 5-11).
        difficulty: u8,
    },
    /// Complete the team's virtual assembly before its deadline.
    FinishAssembly {
        /// Team whose active attempt succeeds.
        team: Team,
    },
    /// Abandon the team's active virtual assembly without rewards.
    CancelAssembly {
        /// Team abandoning its attempt.
        team: Team,
    },
    /// Add one opportunity without spending anything.
    GrantRuneOpportunity {
        /// Team to grant it to.
        team: Team,
    },
    /// Referee penalty damage to a robot: no defense or invincibility applies.
    DamageRobot {
        /// Chassis id of the robot.
        robot: u32,
        /// HP to remove; refused for an already defeated robot.
        amount: u32,
    },
    /// Restore a robot to full HP in place, clearing its respawn timer and
    /// weakness.
    ReviveRobot {
        /// Chassis id of the robot.
        robot: u32,
    },
    /// Revive a defeated robot at once for gold (section 5.2.2).
    InstantRespawn {
        /// Chassis id of the robot.
        robot: u32,
    },
    /// End a robot's weakened state as an own service zone would.
    ClearWeakened {
        /// Chassis id of the robot.
        robot: u32,
    },
    /// Set a robot's HP, clamped to its maximum; zero defeats it.
    SetRobotHp {
        /// Chassis id of the robot.
        robot: u32,
        /// New HP.
        hp: u32,
    },
}

/// The clock an absolute referee timestamp is measured on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StampClock {
    /// The field's own tick clock (`FieldSnapshot::time_ns`).
    Field,
    /// The round clock (`RefereeSnapshot::match_time_ns`), which `SkipTo`
    /// moves and which stops when a round finishes.
    Round,
}

/// Shared presentation record retained by checkpoint/view types. There is no
/// standalone 2026 referee implementation; native rules live in `rmuc_2027`.
pub type Referee = RefereeSnapshot;

/// World team to native gameplay team.
pub fn game_team(team: Team) -> gp::Team {
    match team {
        Team::Red => gp::Team::Red,
        Team::Blue => gp::Team::Blue,
    }
}
/// Physical caliber to native gameplay caliber.
pub fn game_caliber(caliber: crate::Caliber) -> gp::Caliber {
    match caliber {
        crate::Caliber::Mm17 => gp::Caliber::Mm17,
        crate::Caliber::Mm42 => gp::Caliber::Mm42,
    }
}
