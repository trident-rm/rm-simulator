// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native UC V1.0.0 §5.6.3, Figure 5-17 and Tables 5-22–24. No 2026 referee.
use serde::{Deserialize, Serialize};
const S: u64 = 1_000_000_000;
/// Native Rune state, including the transient failed-attempt indication.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Phase {
    /// Dark; an opportunity can start an attempt.
    Inactive,
    /// One or two randomly selected targets accept hits.
    Activating,
    /// Completed; benefits persist until their deadline.
    Activated,
    /// A wrong hit/deadline reset progress. Resumes after a one-nanosecond marker.
    Failed,
}
/// Rune stage, selected by running time rather than a build feature.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Stage {
    /// Single targets, before three minutes.
    Small,
    /// Five pairs, from three minutes.
    Big,
}
/// Team-wide non-field benefits; death does not remove their authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Buff {
    /// Stage that awarded this benefit.
    pub source: Stage,
    /// Exclusive running-round expiry, ns.
    pub until_ns: u64,
    /// Incoming damage defense percentage.
    pub defense_pct: u32,
    /// Outgoing projectile damage multiplier, percent; 100 is unchanged.
    pub attack_pct: u32,
    /// Strongest cooling multiplier, integer 1–5.
    pub cooling: u32,
    /// Small Rune's extra XP already used, tenths, capped at 12,000.
    pub extra_xp_tenths: u32,
}
/// Result of one eligible 17 mm ground-robot strike.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Hit {
    /// State, disabled ring or target outside the wheel prevents detection.
    Ignored,
    /// An illuminated first/bonus target accepted the hit.
    Accepted,
    /// An unlit target reset this attempt's progress.
    Reset,
    /// Five groups completed and awarded a benefit.
    Completed,
}
/// Complete native Rune authority. Motion parameters are integer micro-units;
/// angles are held at transition epochs in nanoradians to preserve partitioning.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Rune {
    /// Stage currently shown.
    pub stage: Stage,
    /// Four-state activation authority.
    pub phase: Phase,
    /// Small/Big opportunities remaining, independently accumulated.
    pub opportunities: [u32; 2],
    /// Last schedule slot already granted, 0–5; grants occur at 0/90/180/255/330s.
    pub grants: u8,
    /// Successful Big activations, used to shrink the scoring area.
    pub big_successes: u8,
    /// Attempt's exclusive twenty-second endpoint, ns; zero outside an attempt.
    pub attempt_until_ns: u64,
    /// First-target or bonus inclusive hit deadline, ns.
    pub target_until_ns: u64,
    /// Current targets in blade order; two on Big first targets, one during bonus.
    pub targets: [bool; 5],
    /// Successful Small blades; all lit when a benefit is held.
    pub completed_blades: [bool; 5],
    /// Completed Big groups or Small hits, 0–5.
    pub groups: u8,
    /// Accepted rings this attempt, in hit order, at most ten.
    pub rings: Vec<u8>,
    /// Waiting for the optional second hit of a Big group.
    pub bonus: bool,
    /// Time of latest state change/reset, ns.
    pub state_since_ns: u64,
    /// Persisted target/parameter generator. Host injects a seed before Start.
    pub seed: u64,
    /// Shared physical rotation direction, -1 or 1; back face uses opposite normal.
    pub direction: i8,
    /// Motion epoch, ns; target speed t resets at every activation/reset.
    pub motion_epoch_ns: u64,
    /// Unwrapped phase at motion epoch, nanoradians.
    pub angle_epoch_nrad: i64,
    /// Sinusoidal amplitude, micro-rad/s; 780,000–1,045,000.
    pub amplitude_urad_s: u32,
    /// Sinusoid angular frequency, micro-rad/s; 1,884,000–2,000,000.
    pub omega_urad_s: u32,
    /// Current team benefit, or None when expired/not activated.
    pub buff: Option<Buff>,
}
impl Rune {
    /// Start dark with a deterministic seed; first opportunity arrives on entering Running.
    pub fn new(direction: i8, seed: u64) -> Self {
        Self {
            stage: Stage::Small,
            phase: Phase::Inactive,
            opportunities: [0; 2],
            grants: 0,
            big_successes: 0,
            attempt_until_ns: 0,
            target_until_ns: 0,
            targets: [false; 5],
            completed_blades: [false; 5],
            groups: 0,
            rings: vec![],
            bonus: false,
            state_since_ns: 0,
            seed,
            direction: if direction < 0 { -1 } else { 1 },
            motion_epoch_ns: 0,
            angle_epoch_nrad: 0,
            amplitude_urad_s: 780_000,
            omega_urad_s: 1_884_000,
            buff: None,
        }
    }
    fn random(&mut self) -> u64 {
        self.seed = self
            .seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.seed
    }
    /// Continuous analytic physical rotation at a running-round timestamp, radians.
    pub fn angle_at(&self, now: u64) -> f64 {
        let t = now.saturating_sub(self.motion_epoch_ns) as f64 * 1e-9;
        let d = if self.stage == Stage::Big
            && matches!(self.phase, Phase::Activating | Phase::Failed)
        {
            let a = f64::from(self.amplitude_urad_s) * 1e-6;
            let w = f64::from(self.omega_urad_s) * 1e-6;
            a / w * (1. - (w * t).cos()) + (2.090 - a) * t
        } else {
            std::f64::consts::PI / 3. * t
        };
        self.angle_epoch_nrad as f64 * 1e-9 + f64::from(self.direction) * d
    }
    /// Instantaneous native target speed, rad/s; simulator applies zero motor lag.
    pub fn speed_at(&self, now: u64) -> f64 {
        let t = now.saturating_sub(self.motion_epoch_ns) as f64 * 1e-9;
        let speed = if self.stage == Stage::Big
            && matches!(self.phase, Phase::Activating | Phase::Failed)
        {
            let a = f64::from(self.amplitude_urad_s) * 1e-6;
            a * (f64::from(self.omega_urad_s) * 1e-6 * t).sin() + 2.090 - a
        } else {
            std::f64::consts::PI / 3.
        };
        speed * f64::from(self.direction)
    }
    fn hold_motion(&mut self, now: u64) {
        self.angle_epoch_nrad = (self.angle_at(now) * 1e9).round() as i64;
        self.motion_epoch_ns = now;
    }
    fn select(&mut self, now: u64) {
        self.targets = [false; 5];
        if self.stage == Stage::Big {
            self.completed_blades = [false; 5];
        }
        self.bonus = false;
        self.target_until_ns = now + 5 * S / 2;
        let available = (0..5)
            .filter(|&i| self.stage == Stage::Big || !self.completed_blades[i])
            .collect::<Vec<_>>();
        if !available.is_empty() {
            let i = available[(self.random() as usize) % available.len()];
            self.targets[i] = true;
        }
        if self.stage == Stage::Big {
            let available = (0..5).filter(|&i| !self.targets[i]).collect::<Vec<_>>();
            let i = available[(self.random() as usize) % available.len()];
            self.targets[i] = true;
        }
    }
    /// Spend the current stage's opportunity; rejected requests are transactional.
    pub fn activate(&mut self, now: u64) -> Result<(), &'static str> {
        let slot = usize::from(self.stage == Stage::Big);
        if self.phase != Phase::Inactive || self.opportunities[slot] == 0 {
            return Err("Rune is busy or has no opportunity");
        }
        self.hold_motion(now);
        self.phase = Phase::Activating;
        self.opportunities[slot] -= 1;
        self.attempt_until_ns = now + 20 * S;
        self.state_since_ns = now;
        self.groups = 0;
        self.rings.clear();
        self.completed_blades = [false; 5];
        self.amplitude_urad_s = 780_000 + (self.random() % 265_001) as u32;
        self.omega_urad_s = 1_884_000 + (self.random() % 116_001) as u32;
        self.select(now);
        Ok(())
    }
    fn fail(&mut self, now: u64) {
        self.hold_motion(now);
        self.phase = Phase::Failed;
        self.state_since_ns = now;
        self.groups = 0;
        self.rings.clear();
        self.completed_blades = [false; 5];
        self.targets = [false; 5];
        self.bonus = false;
        self.amplitude_urad_s = 780_000 + (self.random() % 265_001) as u32;
        self.omega_urad_s = 1_884_000 + (self.random() % 116_001) as u32;
    }
    fn finish(&mut self, now: u64) {
        self.hold_motion(now);
        self.phase = Phase::Activated;
        self.state_since_ns = now;
        self.targets = [false; 5];
        self.completed_blades = [true; 5];
        self.attempt_until_ns = 0;
        self.target_until_ns = 0;
        self.bonus = false;
        let (attack, defense, cooling, duration) = if self.stage == Stage::Small {
            (100, 25, 1, 45 * S)
        } else {
            self.big_successes += 1;
            let sum = self.rings.iter().map(|&n| u32::from(n)).sum::<u32>();
            let n = self.rings.len() as u32;
            let (a, d, c) = if sum <= 3 * n {
                (150, 25, 1)
            } else if sum <= 7 * n {
                (150, 25, 2)
            } else if sum <= 8 * n {
                (200, 25, 2)
            } else if sum <= 9 * n {
                (200, 25, 3)
            } else {
                (300, 50, 5)
            };
            let seconds = [30, 35, 40, 45, 50, 60][self.rings.len() - 5];
            (a, d, c, seconds * S)
        };
        self.buff = Some(Buff {
            source: self.stage,
            until_ns: now + duration,
            defense_pct: defense,
            attack_pct: attack,
            cooling,
            extra_xp_tenths: 0,
        });
    }
    fn next_group(&mut self, now: u64) {
        self.groups += 1;
        if self.groups == 5 {
            self.finish(now);
        } else {
            self.select(now);
        }
    }
    /// Minimum detectable Big ring, Figure 5-17/Table 5-24; Small always accepts the full disk.
    pub fn minimum_ring(&self) -> u8 {
        if self.stage == Stage::Small {
            1
        } else {
            match self.big_successes {
                0 => 1,
                1 => 4,
                _ => 7,
            }
        }
    }
    /// Quantize radius to nearest millimetre, then assign inward-inclusive 15 mm rings.
    /// Radial precision comes from Figure 5-17; nearest-mm/boundary rounding is simulator policy.
    ///
    /// ```
    /// use rm_simulator_gameplay::rmuc_2027::rune::Rune;
    /// assert_eq!(Rune::ring(0.0), Some(10));
    /// assert_eq!(Rune::ring(0.150), Some(1));
    /// assert_eq!(Rune::ring(0.151), None);
    /// ```
    pub fn ring(radius_m: f64) -> Option<u8> {
        if !radius_m.is_finite() || !(0.0..=0.150).contains(&radius_m) {
            return None;
        }
        let mm = (radius_m * 1000.).round() as u32;
        Some((11 - mm.div_ceil(15)).clamp(1, 10) as u8)
    }
    /// Report a detected strike at the current running time, returning its native outcome.
    /// The adapter rejects non-17 mm/aerial hits before this call.
    pub fn hit(&mut self, now: u64, blade: u8, ring: u8) -> Hit {
        if self.phase != Phase::Activating || blade >= 5 || ring < self.minimum_ring() || ring > 10
        {
            return Hit::Ignored;
        }
        if !self.targets[usize::from(blade)] {
            self.fail(now);
            return Hit::Reset;
        }
        self.rings.push(ring);
        self.completed_blades[usize::from(blade)] = true;
        self.targets[usize::from(blade)] = false;
        if self.stage == Stage::Small {
            self.completed_blades[usize::from(blade)] = true;
            self.groups += 1;
            if self.groups == 5 {
                self.finish(now);
            } else {
                self.select(now);
            }
        } else if self.bonus {
            self.next_group(now);
        } else {
            self.bonus = true;
            self.target_until_ns = now + S;
        }
        if self.phase == Phase::Activated {
            Hit::Completed
        } else {
            Hit::Accepted
        }
    }
    /// Next state/schedule boundary after now, ns; lets the enclosing match split time exactly.
    pub fn boundary(&self, now: u64) -> u64 {
        let mut next = 420 * S;
        for at in [0, 90 * S, 180 * S, 255 * S, 330 * S] {
            if at > now {
                next = next.min(at);
            }
        }
        if let Some(buff) = &self.buff
            && buff.until_ns > now
        {
            next = next.min(buff.until_ns);
        }
        if matches!(self.phase, Phase::Activating | Phase::Failed) {
            next = next.min(self.attempt_until_ns);
            if self.phase == Phase::Failed {
                next = next.min(self.state_since_ns + 1);
            } else {
                next = next.min(self.target_until_ns + 1);
            }
        }
        next.saturating_sub(now)
    }
    /// Settle schedule and exact-time boundaries. Returns true on a new Big benefit.
    pub fn settle(&mut self, now: u64) -> bool {
        let was = self.big_successes;
        let schedule = [0, 90 * S, 180 * S, 255 * S, 330 * S];
        while usize::from(self.grants) < schedule.len() && schedule[usize::from(self.grants)] <= now
        {
            let slot = usize::from(self.grants >= 2);
            self.opportunities[slot] += 1;
            self.grants += 1;
        }
        if now >= 180 * S && self.stage == Stage::Small {
            self.hold_motion(180 * S);
            self.stage = Stage::Big;
            if matches!(self.phase, Phase::Activating | Phase::Failed) {
                self.phase = Phase::Inactive;
                self.groups = 0;
                self.rings.clear();
                self.completed_blades = [false; 5];
                self.targets = [false; 5];
                self.attempt_until_ns = 0;
                self.target_until_ns = 0;
                self.bonus = false;
            }
        }
        if self.buff.as_ref().is_some_and(|b| now >= b.until_ns) {
            let at = self.buff.as_ref().unwrap().until_ns;
            self.hold_motion(at);
            self.buff = None;
            self.phase = Phase::Inactive;
            self.groups = 0;
            self.rings.clear();
            self.completed_blades = [false; 5];
            self.state_since_ns = at;
        }
        if matches!(self.phase, Phase::Activating | Phase::Failed) && now >= self.attempt_until_ns {
            if self.stage == Stage::Big && self.bonus && self.groups == 4 {
                self.next_group(self.attempt_until_ns);
            } else {
                self.hold_motion(self.attempt_until_ns);
                self.phase = Phase::Inactive;
                self.groups = 0;
                self.rings.clear();
                self.completed_blades = [false; 5];
                self.targets = [false; 5];
                self.attempt_until_ns = 0;
                self.target_until_ns = 0;
                self.state_since_ns = now;
            }
        } else if self.phase == Phase::Failed && now > self.state_since_ns {
            self.phase = Phase::Activating;
            self.select(self.state_since_ns + 1);
        } else if self.phase == Phase::Activating && now > self.target_until_ns {
            let at = self.target_until_ns + 1;
            if self.stage == Stage::Big && self.bonus {
                self.next_group(at);
            } else {
                self.fail(at);
            }
        }
        self.big_successes > was
    }
    /// Validate bounded schedule, target, motion and scoring authority before restore.
    pub fn validate(&self, now: u64) -> bool {
        let active = matches!(self.phase, Phase::Activating | Phase::Failed);
        let targets = self.targets.iter().filter(|&&v| v).count();
        let progress_valid = if self.phase == Phase::Activating {
            if self.stage == Stage::Small {
                self.groups < 5 && self.rings.len() == usize::from(self.groups) && targets == 1
            } else {
                let n = usize::from(self.groups);
                let bonus = usize::from(self.bonus);
                self.groups < 5
                    && (n + bonus..=2 * n + bonus).contains(&self.rings.len())
                    && targets == if self.bonus { 1 } else { 2 }
            }
        } else if self.phase == Phase::Failed {
            self.groups == 0 && self.rings.is_empty() && targets == 0 && !self.bonus
        } else if self.phase == Phase::Activated {
            self.groups == 5 && (5..=10).contains(&self.rings.len()) && targets == 0
        } else {
            targets == 0
        };
        progress_valid
            && (self.phase == Phase::Activated) == self.buff.is_some()
            && (!active || (self.attempt_until_ns > now && self.attempt_until_ns <= now + 20 * S))
            && (active || self.attempt_until_ns == 0)
            && self.angle_epoch_nrad.unsigned_abs() <= 10_000_000_000_000
            && self.opportunities[0] <= u32::from(self.grants.min(2))
            && self.opportunities[1] <= u32::from(self.grants.saturating_sub(2))
            && self.grants <= 5
            && self.big_successes <= 3
            && self.groups <= 5
            && self.rings.len() <= 10
            && self.rings.iter().all(|&r| (1..=10).contains(&r))
            && [-1, 1].contains(&self.direction)
            && (780_000..=1_045_000).contains(&self.amplitude_urad_s)
            && (1_884_000..=2_000_000).contains(&self.omega_urad_s)
            && self.motion_epoch_ns <= now
            && self.state_since_ns <= now
            && (self.phase != Phase::Activating
                || (self.target_until_ns >= now && self.target_until_ns <= now + 5 * S / 2))
            && self
                .opportunities
                .iter()
                .map(|&n| u64::from(n))
                .sum::<u64>()
                <= 5
            && self.buff.as_ref().is_none_or(|b| {
                b.until_ns > now
                    && b.until_ns <= now + 60 * S
                    && b.extra_xp_tenths <= 12000
                    && match b.source {
                        Stage::Small => (b.attack_pct, b.defense_pct, b.cooling) == (100, 25, 1),
                        Stage::Big => {
                            self.stage == Stage::Big
                                && self.big_successes > 0
                                && b.extra_xp_tenths == 0
                                && matches!(
                                    (b.attack_pct, b.defense_pct, b.cooling),
                                    (150, 25, 1 | 2) | (200, 25, 2 | 3) | (300, 50, 5)
                                )
                        }
                    }
            })
    }
}
