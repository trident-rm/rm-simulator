// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Button-driven UC §5.4.3 assembly, without manipulation or RFID simulation.
use super::{
    Team,
    match_rules::{Match, Phase},
};
use serde::{Deserialize, Serialize};

/// Prototype timeout in ns: the level-four rule, also used for levels 1–3.
pub const ATTEMPT_NS: u64 = 45_000_000_000;
/// Last attempt outcome, retained until another attempt starts.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Outcome {
    /// No attempt has started this round.
    Idle,
    /// Waiting for the operator's Finish action.
    Active,
    /// Finish arrived before the deadline.
    Completed,
    /// Deadline reached without completion.
    TimedOut,
    /// Operator abandoned the attempt.
    Cancelled,
}
/// Complete per-team assembly authority; no physical core state is inferred.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Assembly {
    /// Selected difficulty, 1–4; zero only before any attempt.
    pub difficulty: u8,
    /// Most recent attempt outcome.
    pub outcome: Outcome,
    /// Round time when the attempt started, in ns.
    pub started_ns: u64,
    /// Exclusive completion deadline in round ns.
    pub deadline_ns: u64,
    /// Completed attempts per difficulty; level four may succeed only once.
    pub completions: [u32; 4],
    /// Accumulated income per ten seconds, UC Table 5-12.
    pub income_per_10s: u32,
}
impl Default for Assembly {
    fn default() -> Self {
        Self {
            difficulty: 0,
            outcome: Outcome::Idle,
            started_ns: 0,
            deadline_ns: 0,
            completions: [0; 4],
            income_per_10s: 0,
        }
    }
}
impl Assembly {
    /// Highest completed difficulty; preceding levels must have succeeded.
    pub fn highest(&self) -> u8 {
        self.completions.iter().take_while(|&&n| n > 0).count() as u8
    }
    /// Team Infantry level cap after completed assembly, UC Table 5-12.
    pub fn level_cap(&self) -> u8 {
        match self.highest() {
            0 | 1 => 5,
            2 => 7,
            _ => 10,
        }
    }
    /// Permanent strongest defense percentage, UC Table 5-12.
    pub fn defense_pct(&self) -> u32 {
        match self.highest() {
            4 => 50,
            3 => 25,
            _ => 0,
        }
    }
    /// Reject inconsistent decoded authority before restore.
    pub fn validate(&self, now_ns: u64) -> bool {
        let h = self.highest();
        let income = [40u32, 20, 20, 40]
            .into_iter()
            .enumerate()
            .fold(0u32, |sum, (i, first)| {
                sum.saturating_add(if self.completions[i] == 0 {
                    0
                } else {
                    first.saturating_add(
                        self.completions[i]
                            .saturating_sub(1)
                            .saturating_mul([4, 8, 12, 0][i]),
                    )
                })
            });
        self.difficulty <= 4
            && self.completions[3] <= 1
            && self.completions[usize::from(h)..].iter().all(|&n| n == 0)
            && income == self.income_per_10s
            && if self.outcome == Outcome::Idle {
                self.difficulty == 0 && self.started_ns == 0 && self.deadline_ns == 0 && h == 0
            } else {
                self.difficulty > 0
                    && self.started_ns <= now_ns
                    && self.started_ns >= u64::from(self.difficulty - 1) * 60_000_000_000
                    && self.difficulty <= h + 1
                    && self.deadline_ns == self.started_ns.saturating_add(ATTEMPT_NS)
                    && (self.outcome != Outcome::Active || now_ns < self.deadline_ns)
                    && (self.outcome != Outcome::TimedOut || now_ns >= self.deadline_ns)
                    && (self.outcome != Outcome::Completed
                        || self.completions[usize::from(self.difficulty - 1)] > 0)
                    && (self.outcome != Outcome::Active
                        || self.difficulty != 4
                        || self.completions[3] == 0)
            }
    }
}
impl Match {
    /// Start a virtual positioned-core attempt, respecting Table 5-11 and shared cores.
    /// No robot, RFID contact or manipulation is required in this prototype.
    ///
    /// ```
    /// use rm_simulator_gameplay::{Team, rmuc_2027::match_rules::{Match, COUNTDOWN_NS}};
    /// let mut game = Match::new(1);
    /// game.start(1);
    /// game.advance_ns(COUNTDOWN_NS);
    /// game.start_assembly(Team::Red, 1).unwrap();
    /// game.finish_assembly(Team::Red).unwrap();
    /// assert_eq!(game.assembly[0].income_per_10s, 40);
    /// ```
    pub fn start_assembly(&mut self, team: Team, difficulty: u8) -> Result<(), &'static str> {
        let own = &self.assembly[team.index()];
        let other = &self.assembly[team.other().index()];
        if self.phase != Phase::Running || !(1..=4).contains(&difficulty) {
            return Err("assembly requires a running round and difficulty 1–4");
        }
        if own.outcome == Outcome::Active
            || (other.outcome == Outcome::Active && (difficulty == 4 || other.difficulty == 4))
        {
            return Err("assembly core is busy");
        }
        if self.elapsed_ns() < u64::from(difficulty - 1) * 60_000_000_000
            || difficulty > own.highest() + 1
            || (difficulty == 4 && own.completions[3] > 0)
        {
            return Err("assembly difficulty is locked or already exhausted");
        }
        let now = self.elapsed_ns();
        let own = &mut self.assembly[team.index()];
        own.difficulty = difficulty;
        own.started_ns = now;
        own.deadline_ns = now + ATTEMPT_NS;
        own.outcome = Outcome::Active;
        Ok(())
    }
    /// Operator Finish substitutes for all manipulation and level-four dual-core steps.
    /// Rewards are applied once; completion at the deadline is refused.
    pub fn finish_assembly(&mut self, team: Team) -> Result<(), &'static str> {
        self.settle_assembly();
        let own = &mut self.assembly[team.index()];
        if self.phase != Phase::Running || own.outcome != Outcome::Active {
            return Err("no active assembly attempt to finish");
        }
        let i = usize::from(own.difficulty - 1);
        let first = own.completions[i] == 0;
        own.completions[i] = own.completions[i].saturating_add(1);
        own.income_per_10s = own.income_per_10s.saturating_add(if first {
            [40, 20, 20, 40][i]
        } else {
            [4, 8, 12, 0][i]
        });
        own.outcome = Outcome::Completed;
        self.level_caps[team.index()] = own.level_cap();
        Ok(())
    }
    /// Cancel an active attempt without consuming progression or awarding income.
    pub fn cancel_assembly(&mut self, team: Team) -> Result<(), &'static str> {
        if self.phase != Phase::Running || self.assembly[team.index()].outcome != Outcome::Active {
            return Err("no active assembly attempt to cancel");
        }
        self.assembly[team.index()].outcome = Outcome::Cancelled;
        Ok(())
    }
    pub(super) fn settle_assembly(&mut self) {
        let now = self.elapsed_ns();
        for a in &mut self.assembly {
            if a.outcome == Outcome::Active && now >= a.deadline_ns {
                a.outcome = Outcome::TimedOut;
            }
        }
    }
}
