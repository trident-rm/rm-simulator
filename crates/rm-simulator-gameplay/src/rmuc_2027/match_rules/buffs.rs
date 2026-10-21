// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! UC V1.0.0 §§5.6.4.1–4, 5.6.4.9 continuous field benefits.
use super::*;

/// Effective continuous field benefits; same-type defense uses the strongest source.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Effects {
    /// Defense percentage before subtracting vulnerability, UC §5.6.4.1.
    pub defense_pct: u32,
    /// Vulnerability percentage, additive to damage after defense (§5.6.4.1).
    pub vulnerability_pct: u32,
    /// Effective heat units cooled per second, including additive Fortress cooling.
    pub cooling_per_s: u32,
    /// Exact cooling in half heat-units per second, including fractional multipliers.
    pub cooling_half_per_s: u32,
    /// Reserve rounds currently available to this defending Fortress holder.
    pub fortress_reserve: u32,
}

/// Why a robot does or does not activate one control point right now.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Activation {
    /// Eligible and selected by the point's arbitration.
    Active,
    /// No running round.
    NotRunning,
    /// Robot has zero HP.
    Defeated,
    /// Referee connectivity is absent.
    Offline,
    /// Respawn weakness suppresses ordinary benefits.
    Weakened,
    /// No unexpired physical detection, including the two-second grace.
    NoContact,
    /// The robot's team does not fully control this Outpost territory.
    OutpostNotFull,
    /// The original side still controls its Outpost territory.
    FortressDisabled,
    /// Opponent Fortress requires three elapsed minutes.
    FortressTooEarly,
    /// Another team holds the Central Highland activation latch.
    HighlandHeld,
    /// Another defending robot holds the Fortress activation latch.
    FortressHeld,
}

/// One robot's status at one original-side point; independent of geometry drawing.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PointDiagnostic {
    /// Original field side.
    pub side: Team,
    /// Constituent point.
    pub point: Point,
    /// Eligibility/arbitration result.
    pub status: Activation,
    /// Remaining RFID grace in nanoseconds; zero if no unexpired observation.
    pub remaining_ns: u64,
}

/// Read-only explanation of all point activations and continuous field benefits.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RobotDiagnostic {
    /// Never-reused chassis id.
    pub id: u32,
    /// Fixed robot team.
    pub team: Team,
    /// Eight original-side control point observations and arbitration results.
    pub points: Vec<PointDiagnostic>,
    /// Resolved defense, vulnerability, cooling and available Fortress reserve.
    pub effects: Effects,
    /// Raw card sequence and terrain-benefit authority, including cooldown/deadlines.
    pub terrain: super::super::terrain::Terrain,
}

impl Match {
    /// UC §5.6.4.9: 100 + floor(historical Base deficit / 15), capped at 500.
    pub fn fortress_capacity(&self, side: Team) -> u32 {
        (100 + (BASE_HP - self.base_lowest_hp[side.index()]) / 15).min(500)
    }

    /// Remaining shared Fortress reserve; leaving, death and repairs do not refill it.
    pub fn fortress_reserve_left(&self, side: Team) -> u32 {
        self.fortress_capacity(side)
            .saturating_sub(self.fortress_reserve_used[side.index()])
    }

    fn activation_status(&self, r: &Robot, side: Team, point: Point) -> Activation {
        use Activation::*;
        if self.phase != Phase::Running {
            return NotRunning;
        }
        if r.hp == 0 {
            return Defeated;
        }
        if !r.online {
            return Offline;
        }
        if r.combat.weakened {
            return Weakened;
        }
        if !r.expires_ns[point_index(side, point)].is_some_and(|t| t > self.elapsed_ns()) {
            return NoContact;
        }
        match point {
            Point::Outpost
                if self.control.snapshot().territories[side.index()].control
                    != ControlState::Full(r.team) =>
            {
                OutpostNotFull
            }
            Point::Fortress
                if self.control.snapshot().territories[side.index()]
                    .control
                    .team()
                    == side =>
            {
                FortressDisabled
            }
            Point::Fortress if r.team != side && self.elapsed_ns() < THREE_MIN_NS => {
                FortressTooEarly
            }
            Point::CentralHighland if self.highland_team[side.index()] != Some(r.team) => {
                HighlandHeld
            }
            Point::Fortress
                if r.team == side && self.fortress_robot[side.index()] != Some(r.id) =>
            {
                FortressHeld
            }
            _ => Active,
        }
    }

    /// Current continuous field effects after the last activation resolution.
    /// UC §§5.6.4.2–4 and .9; traversal, rune and reinforcement effects are separate.
    ///
    /// ```
    /// use rm_simulator_gameplay::{Team, rmuc_2027::{Point,
    ///     match_rules::{Match, GroundRole, COUNTDOWN_NS}}};
    /// let mut game = Match::new(1);
    /// game.add_robot(7, Team::Red, GroundRole::Infantry);
    /// game.start(1);
    /// game.advance_ns(COUNTDOWN_NS);
    /// game.observe(7, Team::Red, Point::Base, true);
    /// game.advance_ns(0); // Resolve observations without advancing time.
    /// assert_eq!(game.field_effects(7).defense_pct, 50);
    /// assert_eq!(game.damage_robot(7, 20, false), 10);
    /// ```
    pub fn field_effects(&self, id: u32) -> Effects {
        let mut effects = self.field_effects_before_posture(id);
        if let Some(r) = self
            .robots
            .iter()
            .find(|r| r.id == id && r.role == GroundRole::Sentry && r.hp > 0 && r.online)
        {
            use super::super::sentry::Posture;
            let s = &r.combat.sentry;
            match s.posture {
                Posture::Attack => {
                    effects.cooling_half_per_s = effects.cooling_half_per_s.max(
                        r.stats(self.elapsed_ns()).cooling_per_s
                            * 2
                            * if s.weakened() { 2 } else { 3 },
                    );
                    effects.vulnerability_pct = effects.vulnerability_pct.max(25);
                }
                Posture::Defense => {
                    effects.defense_pct = effects.defense_pct.max(if s.enhanced {
                        99
                    } else if s.weakened() {
                        25
                    } else {
                        50
                    });
                    effects.cooling_half_per_s = (effects.cooling_half_per_s / 6) * 2;
                }
                Posture::Movement => {
                    effects.vulnerability_pct = effects.vulnerability_pct.max(25);
                    effects.cooling_half_per_s = (effects.cooling_half_per_s / 6) * 2;
                }
            }
            effects.cooling_per_s = effects.cooling_half_per_s / 2;
        }
        effects
    }

    fn field_effects_before_posture(&self, id: u32) -> Effects {
        let Some(r) = self.robots.iter().find(|r| r.id == id) else {
            return Effects::default();
        };
        let base = r.stats(self.elapsed_ns()).cooling_per_s;
        let mut effects = Effects {
            cooling_per_s: base,
            cooling_half_per_s: base * 2,
            ..Effects::default()
        };
        if self.phase != Phase::Running || r.hp == 0 {
            return effects;
        }
        let active = |side, point| self.activation_status(r, side, point) == Activation::Active;
        if let Some(buff) = &self.runes[r.team.index()].buff {
            effects.defense_pct = buff.defense_pct;
            effects.cooling_half_per_s = base * 2 * buff.cooling;
            effects.cooling_per_s = effects.cooling_half_per_s / 2;
        }
        effects.defense_pct = effects
            .defense_pct
            .max(self.assembly[r.team.index()].defense_pct());
        // Weakness prevents RFID-point benefits (§5.3.2), not non-field Rune
        // benefits, which survive death and apply while their source exists (Table 5-26).
        if !r.online || r.combat.weakened {
            return effects;
        }
        if r.terrain.defense_until_ns > self.elapsed_ns() {
            effects.defense_pct = effects.defense_pct.max(r.terrain.defense_pct);
        }
        if active(r.team, Point::Base)
            || r.combat
                .vantage_expires_ns
                .is_some_and(|t| t > self.elapsed_ns())
        {
            effects.defense_pct = effects.defense_pct.max(50);
        }
        if Team::BOTH
            .into_iter()
            .any(|s| active(s, Point::CentralHighland))
        {
            effects.defense_pct = effects.defense_pct.max(25);
        }
        if active(r.team, Point::Fortress) {
            effects.defense_pct = effects.defense_pct.max(50);
            effects.cooling_per_s =
                base + ((BASE_HP - self.base_lowest_hp[r.team.index()]) / 80).min(75);
            effects.fortress_reserve = self.fortress_reserve_left(r.team);
        }
        if active(r.team.other(), Point::Fortress) && !self.base_open[r.team.other().index()] {
            effects.vulnerability_pct = 100;
        }
        effects.cooling_half_per_s = effects.cooling_per_s * 2;
        if r.terrain.until_ns[3] > self.elapsed_ns() {
            effects.cooling_half_per_s = effects.cooling_half_per_s.max(base * 3);
        }
        if let Some(buff) = &self.runes[r.team.index()].buff {
            effects.cooling_half_per_s = effects.cooling_half_per_s.max(base * 2 * buff.cooling);
        }
        effects.cooling_per_s = effects.cooling_half_per_s / 2;
        effects
    }

    /// Explain authoritative activations without modifying this match or its latches.
    /// A clone resolves any observations submitted since the last explicit advance.
    pub fn control_diagnostics(&self) -> Vec<RobotDiagnostic> {
        let mut resolved = self.clone();
        resolved.resolve();
        resolved
            .robots
            .iter()
            .map(|r| RobotDiagnostic {
                id: r.id,
                team: r.team,
                effects: resolved.field_effects(r.id),
                terrain: r.terrain.clone(),
                points: Team::BOTH
                    .into_iter()
                    .flat_map(|side| {
                        points().map(|point| PointDiagnostic {
                            side,
                            point,
                            status: resolved.activation_status(r, side, point),
                            remaining_ns: r.expires_ns[point_index(side, point)]
                                .map_or(0, |t| t.saturating_sub(resolved.elapsed_ns())),
                        })
                    })
                    .collect(),
            })
            .collect()
    }
}
