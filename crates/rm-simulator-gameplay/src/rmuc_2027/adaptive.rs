// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! UC 2027 V1.0.0 §5.2 adaptive-armor observations. Reinforcement is not enabled.
//! Coordinates are quantized to micrometres for exact checkpoint equality.
use crate::{Target, Team};
use serde::{Deserialize, Serialize};

/// One detected projectile contact, including protected or invincible targets.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Impact {
    /// Round time in nanoseconds.
    pub time_ns: u64,
    /// Identified attacking team, or unknown for an unattributed projectile.
    pub attacker: Option<Team>,
    /// Physical target's owner at contact, including transferred outposts.
    pub victim: Team,
    /// Structure side or robot identity.
    pub target: Target,
    /// Detector index within the physical target.
    pub plate: u32,
    /// Bottom-left detector coordinates, horizontal right then up, in µm.
    pub position_um: [u32; 2],
    /// Detector width and height in µm, not the surrounding armor housing.
    pub size_um: [u32; 2],
}
/// Full round observations; no inferred reinforcement region changes damage.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Adaptive {
    /// Successfully launched projectiles by attacking team.
    pub launches: [u32; 2],
    /// Accumulating one-minute selection opportunities (§5.2).
    pub opportunities: [u32; 2],
    /// Contacts retained for exact coordinate medians and future victim/Radar views.
    pub impacts: Vec<Impact>,
}
/// A finite, defined team's accuracy and spread calculation (§5.2).
#[derive(Clone, Debug, PartialEq)]
pub struct Accuracy {
    /// Enemy armor detections divided by launches; not clamped to one.
    pub hit_rate: f64,
    /// Coordinate-wise median in metres from detector bottom-left.
    pub median_m: [f64; 2],
    /// Median distance from the coordinate-wise median, in metres.
    pub spread_m: f64,
    /// Hit rate divided by spread, in inverse metres.
    pub accuracy_per_m: f64,
}
/// An undefined manual case; callers must not replace it with a damage scalar.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Undefined {
    /// No launches establish an accuracy denominator.
    NoLaunches,
    /// No identified enemy hits establish medians.
    NoHits,
    /// Median spread is zero; the manual gives no finite replacement.
    ZeroSpread,
}
fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(f64::total_cmp);
    let n = values.len();
    if n.is_multiple_of(2) {
        (values[n / 2 - 1] + values[n / 2]) / 2.
    } else {
        values[n / 2]
    }
}
impl Adaptive {
    /// Count only identified hits on enemy armor; unknown/friendly hits remain
    /// in the observation history but do not invent an attacking team's accuracy.
    pub fn enemy_hits(&self, team: Team) -> impl Iterator<Item = &Impact> {
        self.impacts
            .iter()
            .filter(move |h| h.attacker == Some(team) && h.victim != team)
    }
    /// Whether the ten-hit and opportunity prerequisites hold. This does not
    /// authorize selection: clipping, boundary and singular cases remain pending.
    pub fn selection_ready(&self, team: Team) -> bool {
        self.opportunities[team.index()] > 0 && self.enemy_hits(team).count() >= 10
    }
    /// Evaluate defined statistics without choosing a reinforcement policy.
    ///
    /// ```
    /// use rm_simulator_gameplay::{Team, rmuc_2027::adaptive::{Adaptive, Undefined}};
    /// assert_eq!(Adaptive::default().accuracy(Team::Red), Err(Undefined::NoLaunches));
    /// ```
    pub fn accuracy(&self, team: Team) -> Result<Accuracy, Undefined> {
        let launches = self.launches[team.index()];
        if launches == 0 {
            return Err(Undefined::NoLaunches);
        }
        let hits = self.enemy_hits(team).collect::<Vec<_>>();
        if hits.is_empty() {
            return Err(Undefined::NoHits);
        }
        let center = std::array::from_fn(|i| {
            median(
                hits.iter()
                    .map(|h| h.position_um[i] as f64 * 1e-6)
                    .collect(),
            )
        });
        let spread = median(
            hits.iter()
                .map(|h| {
                    (h.position_um[0] as f64 * 1e-6 - center[0])
                        .hypot(h.position_um[1] as f64 * 1e-6 - center[1])
                })
                .collect(),
        );
        if spread == 0. {
            return Err(Undefined::ZeroSpread);
        }
        let rate = hits.len() as f64 / f64::from(launches);
        Ok(Accuracy {
            hit_rate: rate,
            median_m: center,
            spread_m: spread,
            accuracy_per_m: rate / spread,
        })
    }
    /// Formula-defined area share for non-singular teams, without geometric
    /// clipping or inside/outside classification. No damage uses this value.
    pub fn area_share(&self, team: Team) -> Result<f64, Undefined> {
        let rho = self.accuracy(team)?.accuracy_per_m / self.accuracy(team.other())?.accuracy_per_m;
        Ok(rho * rho / (2. + rho * rho))
    }
    /// Reject malformed coordinates and future observations before restore.
    pub fn validate(&self, elapsed_ns: u64) -> bool {
        self.opportunities
            .iter()
            .all(|&v| u64::from(v) <= elapsed_ns / 60_000_000_000)
            && self.impacts.iter().all(|h| {
                h.time_ns <= elapsed_ns
                    && (0..2).all(|i| h.size_um[i] > 0 && h.position_um[i] <= h.size_um[i])
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn impact(attacker: Option<Team>, victim: Team, x: u32, y: u32) -> Impact {
        Impact {
            time_ns: 0,
            attacker,
            victim,
            target: Target::Base(victim),
            plate: 0,
            position_um: [x, y],
            size_um: [100_000; 2],
        }
    }
    #[test]
    fn medians_unknown_and_friendly_hits_do_not_invent_accuracy() {
        let mut a = Adaptive {
            launches: [4, 4],
            ..Default::default()
        };
        a.impacts = vec![
            impact(Some(Team::Red), Team::Blue, 10_000, 20_000),
            impact(Some(Team::Red), Team::Blue, 30_000, 20_000),
            impact(None, Team::Blue, 90_000, 90_000),
            impact(Some(Team::Red), Team::Red, 99_000, 99_000),
        ];
        let accuracy = a.accuracy(Team::Red).unwrap();
        assert!((accuracy.hit_rate - 0.5).abs() < 1e-12);
        assert!((accuracy.median_m[0] - 0.02).abs() < 1e-12);
        assert!((accuracy.spread_m - 0.01).abs() < 1e-12);
        assert_eq!(a.area_share(Team::Red), Err(Undefined::NoHits));
        a.impacts
            .push(impact(Some(Team::Blue), Team::Red, 10_000, 20_000));
        assert_eq!(a.accuracy(Team::Blue), Err(Undefined::ZeroSpread));
        a.impacts
            .push(impact(Some(Team::Blue), Team::Red, 30_000, 20_000));
        assert!((a.area_share(Team::Red).unwrap() - 1. / 3.).abs() < 1e-12);
        assert!(!a.selection_ready(Team::Red));
        assert!(a.validate(0));
        a.impacts[0].position_um[0] = 100_001;
        assert!(!a.validate(0));
    }
    #[test]
    fn checkpoint_and_ten_hit_prerequisite_remain_exact() {
        let mut a = Adaptive {
            launches: [10, 0],
            opportunities: [1, 1],
            ..Default::default()
        };
        a.impacts = (0..10)
            .map(|x| impact(Some(Team::Red), Team::Blue, x * 1000, x * 2000))
            .collect();
        assert!(a.selection_ready(Team::Red));
        let b: Adaptive = serde_json::from_slice(&serde_json::to_vec(&a).unwrap()).unwrap();
        assert_eq!(a, b);
        assert!(b.validate(60_000_000_000));
    }
}
