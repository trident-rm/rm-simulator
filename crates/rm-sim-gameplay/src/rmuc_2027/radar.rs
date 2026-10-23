// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! UC §5.7.4 Radar progress/history and independently observed team intelligence.
use super::match_rules::{Match, Phase};
use crate::Team;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Missing-data sample interval, UC Table 5-29, in nanoseconds.
pub const INTERRUPTION_NS: u64 = 500_000_000;
/// Cumulative vulnerability needed per opportunity, UC §5.7.4, in nanoseconds.
pub const OPPORTUNITY_NS: u64 = 60_000_000_000;
/// Double-vulnerability duration, UC §5.7.4, in nanoseconds.
pub const DOUBLE_NS: u64 = 30_000_000_000;
/// Simulator lifetime of externally decoded information, in nanoseconds.
/// The supplied manuals do not specify the information-wave packet rate.
pub const INFORMATION_NS: u64 = 1_000_000_000;
/// Position report quality, evaluated against the authoritative planar position.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Quality {
    /// Error strictly below 0.8 m.
    Accurate,
    /// Error from 0.8 m inclusive to 1.6 m exclusive.
    SemiAccurate,
    /// Error at least 1.6 m, or an interrupted report stream.
    Wrong,
}
/// One observer team's mark history for one ground robot, UC Tables 5-29–30.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Mark {
    /// Progress in tenths, clamped to 0..=1500.
    pub progress_tenths: i32,
    /// Signed history increment in tenths; saturation protects decoded arithmetic.
    pub increment_tenths: i32,
    /// Whether the previous sample was accurate or semi-accurate.
    pub positive: bool,
    /// Next interrupted-stream sample deadline, running-round nanoseconds.
    pub next_wrong_ns: u64,
    /// Last submitted planar FLU position, in millimetres; absent before any report.
    pub position_mm: Option<[i32; 2]>,
    /// Time of the last position report, independent of interrupted samples.
    pub reported_ns: Option<u64>,
}
impl Default for Mark {
    fn default() -> Self {
        Self {
            progress_tenths: 0,
            increment_tenths: 0,
            positive: false,
            next_wrong_ns: INTERRUPTION_NS,
            position_mm: None,
            reported_ns: None,
        }
    }
}
impl Mark {
    fn sample(&mut self, quality: Quality) {
        let positive = quality != Quality::Wrong;
        let delta = match quality {
            Quality::Accurate => 10,
            Quality::SemiAccurate => 5,
            Quality::Wrong => -8,
        };
        self.increment_tenths = if self.positive == positive {
            self.increment_tenths.saturating_add(delta)
        } else {
            delta
        };
        self.positive = positive;
        self.progress_tenths = self
            .progress_tenths
            .saturating_add(self.increment_tenths)
            .clamp(0, 1500);
    }
    /// Whether Table 5-30 permits replacing reported XY with localization XY.
    pub fn localized(&self, friendly: bool) -> bool {
        self.progress_tenths >= if friendly { 500 } else { 1000 }
    }
}
/// Decoded information-wave fields; omitted values stay unknown, never filled from truth.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct Information {
    /// Observed HP, or unknown; not inferred from a position report.
    pub hp: Option<u32>,
    /// Independently decoded HP ceiling; absent if the packet did not supply it.
    pub max_hp: Option<u32>,
    /// Observed remaining projectile allowance, or unknown.
    pub allowance: Option<u32>,
    /// Observed defense percentage, or unknown.
    pub defense_pct: Option<u32>,
    /// Last independently observed FLU position, in millimetres, or unknown.
    pub position_mm: Option<[i32; 3]>,
    /// Reception time in running-round nanoseconds; assigned by the authority.
    pub observed_ns: u64,
}
/// Entire restorable per-team Radar authority; Red then Blue.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Radar {
    /// Mark history per observing team and target identity.
    pub marks: [BTreeMap<u32, Mark>; 2],
    /// Independently decoded information per observing team and target identity.
    pub information: [BTreeMap<u32, Information>; 2],
    /// Union time with at least one living opposing robot vulnerable, nanoseconds.
    pub vulnerable_ns: [u64; 2],
    /// Opportunities spent this round, at most two per team.
    pub spent: [u8; 2],
    /// Double-vulnerability expiration, running-round nanoseconds.
    pub double_until_ns: [u64; 2],
    /// Opponent interference difficulty faced by each observer, 0..=2 (levels 1–3).
    pub interference: [u8; 2],
}
impl Radar {
    /// Available unspent opportunities; §5.7.4 caps activations at two per round.
    pub fn opportunities(&self, team: Team) -> u8 {
        ((self.vulnerable_ns[team.index()] / OPPORTUNITY_NS).min(2) as u8)
            .saturating_sub(self.spent[team.index()])
    }
    /// Effective source vulnerability, respecting the information-wave handicap.
    pub fn vulnerability_pct(&self, observer: Team, id: u32, now_ns: u64) -> u32 {
        let i = observer.index();
        if self.interference[i] > self.interference[1 - i] {
            return 0;
        }
        let progress = self.marks[i].get(&id).map_or(0, |m| m.progress_tenths);
        let base = if progress >= 1200 {
            20
        } else if progress >= 1000 {
            15
        } else {
            0
        };
        base * if now_ns < self.double_until_ns[i] {
            2
        } else {
            1
        }
    }
    pub(crate) fn respawn(&mut self, id: u32) {
        for marks in &mut self.marks {
            if let Some(m) = marks.get_mut(&id) {
                m.increment_tenths = 0;
                m.positive = false;
            }
        }
    }
    pub(crate) fn validate(&self, now: u64, ids: &[u32]) -> bool {
        self.spent
            .iter()
            .enumerate()
            .all(|(i, s)| *s <= 2 && u64::from(*s) <= self.vulnerable_ns[i] / OPPORTUNITY_NS)
            && self.interference.iter().all(|d| *d <= 2)
            && self.vulnerable_ns.iter().all(|t| *t <= now)
            && self
                .double_until_ns
                .iter()
                .all(|t| *t <= now.saturating_add(DOUBLE_NS))
            && self.marks.iter().all(|map| {
                map.len() == ids.len()
                    && map.iter().all(|(id, m)| {
                        ids.contains(id)
                            && (0..=1500).contains(&m.progress_tenths)
                            && m.next_wrong_ns > now
                            && m.next_wrong_ns <= now.saturating_add(INTERRUPTION_NS)
                            && m.reported_ns.is_none_or(|t| t <= now)
                            && m.position_mm.is_some() == m.reported_ns.is_some()
                    })
            })
            && self.information.iter().all(|map| {
                map.iter().all(|(id, info)| {
                    ids.contains(id)
                        && info.observed_ns <= now
                        && now - info.observed_ns < INFORMATION_NS
                        && info.defense_pct.is_none_or(|p| p <= 100)
                })
            })
    }
}
impl Match {
    /// Submit a position observation already classified by the world adapter.
    /// Both friendly and opposing robots can be marked (UC Table 5-30).
    ///
    /// ```
    /// use rm_sim_gameplay::{Team, rmuc_2027::{match_rules::{Match, GroundRole, COUNTDOWN_NS}, radar::Quality}};
    /// let mut game=Match::default();
    /// game.add_robot(2,Team::Blue,GroundRole::Infantry);
    /// game.start(1);game.advance_ns(COUNTDOWN_NS);
    /// game.radar_report(Team::Red,2,[1000,2000],Quality::Accurate).unwrap();
    /// assert_eq!(game.radar.marks[0][&2].progress_tenths,10);
    /// ```
    pub fn radar_report(
        &mut self,
        observer: Team,
        id: u32,
        position_mm: [i32; 2],
        mut quality: Quality,
    ) -> Result<(), &'static str> {
        if self.phase != Phase::Running {
            return Err("Radar reports require a running round");
        }
        let robot = self
            .robots
            .iter()
            .find(|r| r.id == id)
            .ok_or("unknown Radar target")?;
        // UC §5.1.5: offline localization is always accurate to enemies, wrong to allies.
        if !robot.combat.localization_module_online {
            quality = if robot.team == observer {
                Quality::Wrong
            } else {
                Quality::Accurate
            };
        }
        let now = self.elapsed_ns();
        let mark = self.radar.marks[observer.index()]
            .get_mut(&id)
            .ok_or("missing Radar target")?;
        mark.sample(quality);
        mark.position_mm = Some(position_mm);
        mark.reported_ns = Some(now);
        mark.next_wrong_ns = now + INTERRUPTION_NS;
        Ok(())
    }
    /// Accept independently decoded fields. This injection models reception only;
    /// it does not claim to implement the unpublished 2027 radio/packet decoder.
    pub fn radar_information(
        &mut self,
        observer: Team,
        id: u32,
        mut info: Information,
    ) -> Result<(), &'static str> {
        if self.phase != Phase::Running
            || !self.robots.iter().any(|r| r.id == id)
            || info.defense_pct.is_some_and(|p| p > 100)
        {
            return Err("invalid Radar information");
        }
        info.observed_ns = self.elapsed_ns();
        self.radar.information[observer.index()].insert(id, info);
        Ok(())
    }
    /// Spend a double-vulnerability opportunity; at most two activations per round.
    pub fn radar_double(&mut self, observer: Team) -> Result<(), &'static str> {
        let i = observer.index();
        if self.phase != Phase::Running
            || self.radar.opportunities(observer) == 0
            || self.elapsed_ns() < self.radar.double_until_ns[i]
        {
            return Err("Radar double unavailable");
        }
        self.radar.spent[i] += 1;
        self.radar.double_until_ns[i] = self.elapsed_ns() + DOUBLE_NS;
        Ok(())
    }
    pub(crate) fn radar_boundary_ns(&self) -> u64 {
        let now = self.elapsed_ns();
        self.radar
            .marks
            .iter()
            .flat_map(|m| m.values())
            .map(|m| m.next_wrong_ns.saturating_sub(now))
            .chain(
                self.radar
                    .information
                    .iter()
                    .flat_map(|map| map.values())
                    .map(|i| {
                        i.observed_ns
                            .saturating_add(INFORMATION_NS)
                            .saturating_sub(now)
                    }),
            )
            .chain(
                self.radar
                    .double_until_ns
                    .iter()
                    .filter(|&&t| t > now)
                    .map(|t| t - now),
            )
            .min()
            .unwrap_or(u64::MAX)
    }
    pub(crate) fn advance_radar(&mut self, dt: u64, before: u64) {
        for observer in Team::BOTH {
            let vulnerable = self.robots.iter().any(|r| {
                r.team != observer
                    && r.hp > 0
                    && self.radar.vulnerability_pct(observer, r.id, before) > 0
            });
            if vulnerable {
                self.radar.vulnerable_ns[observer.index()] += dt;
            }
        }
        let now = self.elapsed_ns();
        for marks in &mut self.radar.marks {
            for mark in marks.values_mut() {
                if mark.next_wrong_ns <= now {
                    mark.sample(Quality::Wrong);
                    mark.next_wrong_ns += INTERRUPTION_NS;
                }
            }
        }
        for information in &mut self.radar.information {
            information.retain(|_, info| now < info.observed_ns.saturating_add(INFORMATION_NS));
        }
    }
}
