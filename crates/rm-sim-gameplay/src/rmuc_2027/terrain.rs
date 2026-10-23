// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! UC V1.0.0 §5.6.4.5 / Table 5-27 physical card sequences, independent of grace.
use crate::Team;
use serde::{Deserialize, Serialize};
const SECOND: u64 = 1_000_000_000;

/// Type of physical traversal course, independent of its field side.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Course {
    /// Low then high within five seconds.
    Highland,
    /// Low then high within three seconds, fifteen-second repeat cooldown.
    Road,
    /// Either endpoint to the other, optionally through the interior, within three seconds.
    Tunnel,
}
/// A physical card identity. Repeated samples of the same held contact are not entries.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct Card {
    /// Original field side.
    pub side: Team,
    /// Course type, or None for an intervening non-traversal card.
    pub course: Option<Course>,
    /// Independent course number; Highland 0–1, Road 0, Tunnel 0–2.
    /// For non-traversal cards this is a stable caller-owned identity.
    pub index: u8,
    /// Low/high = 0/1; tunnel endpoint/interior/endpoint = 0/1/2.
    pub pad: u8,
}
impl Card {
    /// Reject identities outside the native UC course catalogue.
    pub fn valid(self) -> bool {
        match self.course {
            Some(Course::Highland) => self.index < 2 && self.pad < 2,
            Some(Course::Road) => self.index == 0 && self.pad < 2,
            Some(Course::Tunnel) => self.index < 3 && self.pad < 3,
            None => self.pad == 0,
        }
    }
}
/// The four different benefits counted by §5.6.4.5's combination rule.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Benefit {
    /// 25% defense for thirty seconds.
    Highland,
    /// 25% defense for five seconds.
    Road,
    /// Endpoint to endpoint without an interior card: 25% defense for thirty seconds.
    AcrossTunnel,
    /// Endpoint/interior/endpoint: 1.5× cooling for 120 seconds.
    ThroughTunnel,
}
impl Benefit {
    fn slot(self) -> usize {
        self as usize
    }
    fn duration(self) -> u64 {
        match self {
            Self::Road => 5 * SECOND,
            Self::ThroughTunnel => 120 * SECOND,
            _ => 30 * SECOND,
        }
    }
}
/// One in-flight sequence and its original deadline; held samples never extend it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Progress {
    /// First entered card, identifying side, course, direction and starting time.
    pub start: Card,
    /// Running-round start time, in ns.
    pub started_ns: u64,
    /// Whether this tunnel sequence has entered its interior.
    pub interior: bool,
}
impl Progress {
    /// Inclusive final detection deadline, in running-round ns.
    pub fn deadline_ns(&self) -> u64 {
        self.started_ns
            .saturating_add(if self.start.course == Some(Course::Highland) {
                5 * SECOND
            } else {
                3 * SECOND
            })
    }
}
/// Entire per-robot traversal authority, including raw held-card evidence.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Terrain {
    /// Sorted, unique physical contacts at the last observation; no RFID grace.
    pub contacts: Vec<Card>,
    /// Pending sequence, or None after completion/interruption/timeout.
    pub progress: Option<Progress>,
    /// Exclusive benefit expiries: Highland, Road, AcrossTunnel, ThroughTunnel.
    pub until_ns: [u64; 4],
    /// Defense percentage, zero, 25 or 50; expires at defense_until_ns.
    pub defense_pct: u32,
    /// Exclusive defense deadline, including combined-benefit retention.
    pub defense_until_ns: u64,
    /// Earliest next Road award. Cooldown survives death; a new round resets it.
    pub road_ready_ns: u64,
    /// First-benefit XP consumed this round: Highland, Road, Tunnel, Vantage (Table 5-14).
    pub experience_awarded: [bool; 4],
}
impl Terrain {
    /// Physical observations at one timestamp; all simultaneous contacts are reported together.
    /// Multiple distinct cards are treated as ambiguous and interrupt an attempt.
    /// Finishing exactly at a course deadline succeeds. A wrong card can start a fresh course.
    ///
    /// ```
    /// use rm_sim_gameplay::{Team, rmuc_2027::terrain::{Terrain, Card, Course, Benefit}};
    /// let mut terrain = Terrain::default();
    /// let low = Card { side: Team::Red, course: Some(Course::Road), index: 0, pad: 0 };
    /// assert_eq!(terrain.observe(0, &[low]), None);
    /// assert_eq!(terrain.observe(3_000_000_000, &[Card { pad: 1, ..low }]), Some(Benefit::Road));
    /// assert_eq!(terrain.defense_pct, 25);
    /// ```
    pub fn observe(&mut self, now: u64, cards: &[Card]) -> Option<Benefit> {
        self.expire(now);
        // Membership is independent of ordering/duplicates. Inspect the old
        // contacts before reusing their storage for this observation.
        let new = cards.iter().any(|c| !self.contacts.contains(c));
        self.contacts.clear();
        self.contacts.extend_from_slice(cards);
        self.contacts.sort();
        self.contacts.dedup();
        if !new {
            return None;
        }
        if self.contacts.len() != 1 || !self.contacts[0].valid() {
            self.progress = None;
            return None;
        }
        let card = self.contacts[0];
        let mut completed = None;
        if let Some(p) = &mut self.progress
            && card.course == p.start.course
            && card.side == p.start.side
            && card.index == p.start.index
        {
            match card.course {
                Some(Course::Highland) if card.pad == 1 => completed = Some(Benefit::Highland),
                Some(Course::Road) if card.pad == 1 && now >= self.road_ready_ns => {
                    completed = Some(Benefit::Road)
                }
                Some(Course::Tunnel) if card.pad == 1 => {
                    p.interior = true;
                    return None;
                }
                Some(Course::Tunnel) if card.pad != p.start.pad => {
                    completed = Some(if p.interior {
                        Benefit::ThroughTunnel
                    } else {
                        Benefit::AcrossTunnel
                    })
                }
                _ => {}
            }
        }
        self.progress = None;
        if let Some(benefit) = completed {
            let mixed = self
                .until_ns
                .iter()
                .enumerate()
                .any(|(i, &until)| i != benefit.slot() && until > now);
            let until = now + benefit.duration();
            self.until_ns[benefit.slot()] = self.until_ns[benefit.slot()].max(until);
            if mixed || benefit != Benefit::ThroughTunnel {
                self.defense_pct = self.defense_pct.max(if mixed { 50 } else { 25 });
                self.defense_until_ns = self.defense_until_ns.max(until);
                if mixed {
                    self.defense_until_ns = self
                        .defense_until_ns
                        .max(self.until_ns.iter().copied().max().unwrap_or(0));
                }
            }
            if benefit == Benefit::Road {
                self.road_ready_ns = now + 15 * SECOND;
            }
            return Some(benefit);
        }
        if matches!(
            (card.course, card.pad),
            (Some(Course::Highland | Course::Road), 0) | (Some(Course::Tunnel), 0 | 2)
        ) {
            self.progress = Some(Progress {
                start: card,
                started_ns: now,
                interior: false,
            });
        }
        None
    }
    /// Normalize expired progress and benefits at an explicit running timestamp.
    pub fn expire(&mut self, now: u64) {
        if self
            .progress
            .as_ref()
            .is_some_and(|p| now > p.deadline_ns())
        {
            self.progress = None;
        }
        if now >= self.defense_until_ns {
            self.defense_pct = 0;
            self.defense_until_ns = 0;
        }
        for until in &mut self.until_ns {
            if now >= *until {
                *until = 0;
            }
        }
    }
    /// Death clears progress, raw contacts and all obtained benefits, preserving Road cooldown.
    pub fn clear_on_death(&mut self) {
        let road_ready_ns = self.road_ready_ns;
        let experience_awarded = self.experience_awarded;
        *self = Self {
            road_ready_ns,
            experience_awarded,
            ..Self::default()
        };
    }
    /// Validate course identities, chronology, cooldown and benefit ranges before restore.
    pub fn validate(&self, now: u64) -> bool {
        self.contacts.iter().all(|c| c.valid())
            && self.contacts.windows(2).all(|c| c[0] < c[1])
            && [0, 25, 50].contains(&self.defense_pct)
            && self.defense_until_ns <= now.saturating_add(120 * SECOND)
            && self.road_ready_ns <= now.saturating_add(15 * SECOND)
            && self
                .until_ns
                .iter()
                .all(|&t| t <= now.saturating_add(120 * SECOND))
            && self.progress.as_ref().is_none_or(|p| {
                p.start.valid()
                    && p.started_ns <= now
                    && now <= p.deadline_ns()
                    && matches!(
                        (p.start.course, p.start.pad),
                        (Some(Course::Highland | Course::Road), 0) | (Some(Course::Tunnel), 0 | 2)
                    )
            })
    }
}

impl super::match_rules::Match {
    /// Report all raw physical cards at this timestamp; no grace is applied to sequences.
    /// Death/offline/weakness suppress sequences. Table 5-14 first-kind XP is awarded once.
    pub fn observe_terrain(&mut self, id: u32, cards: &[Card]) {
        if self.phase != super::match_rules::Phase::Running {
            return;
        }
        let now = self.elapsed_ns();
        let Some(r) = self.robots.iter_mut().find(|r| r.id == id) else {
            return;
        };
        if r.hp == 0 || !r.online || r.combat.weakened {
            r.terrain.progress = None;
            r.terrain.contacts.clear();
            return;
        }
        if let Some(benefit) = r.terrain.observe(now, cards) {
            let slot = match benefit {
                Benefit::Highland => 0,
                Benefit::Road => 1,
                _ => 2,
            };
            if !r.terrain.experience_awarded[slot] {
                r.terrain.experience_awarded[slot] = true;
                self.grant_experience(id, 3000);
            }
        }
    }
}

#[cfg(test)]
mod storage_tests {
    use super::*;

    #[test]
    fn repeated_contacts_reuse_storage_without_changing_entry_edges() {
        let low = Card {
            side: crate::Team::Red,
            course: Some(Course::Road),
            index: 0,
            pad: 0,
        };
        let high = Card { pad: 1, ..low };
        let mut terrain = Terrain::default();
        assert_eq!(terrain.observe(0, &[low, low, low]), None);
        let capacity = terrain.contacts.capacity();
        let storage = terrain.contacts.as_ptr();
        assert_eq!(terrain.contacts, [low]);
        assert_eq!(terrain.observe(1_000_000_000, &[low]), None);
        assert_eq!(
            terrain.observe(3_000_000_000, &[high, high]),
            Some(Benefit::Road)
        );
        let rewarded = terrain.clone();
        assert_eq!(terrain.observe(3_000_000_001, &[high]), None);
        assert_eq!(terrain.progress, rewarded.progress);
        // Empty contacts clear the edge state while retaining the allocation.
        assert_eq!(terrain.observe(3_000_000_002, &[]), None);
        assert!(terrain.contacts.is_empty());
        assert_eq!(terrain.contacts.capacity(), capacity);
        assert_eq!(terrain.contacts.as_ptr(), storage);
        assert!(terrain.validate(3_000_000_002));
    }

    #[test]
    fn unordered_duplicate_contacts_match_canonical_observations() {
        let low = Card {
            side: crate::Team::Red,
            course: Some(Course::Highland),
            index: 0,
            pad: 0,
        };
        let high = Card { pad: 1, ..low };
        let mut canonical = Terrain::default();
        let mut redundant = Terrain::default();
        for (time_ns, cards) in [
            (0, vec![low]),
            (1, vec![high, low]),
            (2, vec![]),
            (3, vec![low]),
            (4, vec![high]),
            (60_000_000_000, vec![high]),
        ] {
            let mut duplicates = cards.clone();
            duplicates.reverse();
            duplicates.extend_from_slice(&cards);
            assert_eq!(
                canonical.observe(time_ns, &cards),
                redundant.observe(time_ns, &duplicates)
            );
            assert_eq!(canonical, redundant);
        }
    }
}
