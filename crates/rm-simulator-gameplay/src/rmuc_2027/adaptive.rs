// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! UC 2027 V1.0.0 §5.2 adaptive armor with frozen selection-time area.
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
    /// Classification at contact time; later selections never rewrite history.
    pub region: Region,
}
/// Damage class from UC Table 5-5, applied to raw projectile damage.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Region {
    /// No selected finite area: ordinary raw damage.
    #[default]
    Default,
    /// Inside or on the selected circle: half raw damage.
    Reinforced,
    /// Outside a selected finite circle: one and a half raw damage.
    Unreinforced,
}
impl Region {
    /// Exact multiplier in half units: 1, 2 or 3, UC Table 5-5.
    ///
    /// ```
    /// use rm_simulator_gameplay::rmuc_2027::adaptive::Region;
    /// assert_eq!(20 * Region::Reinforced.half_units() / 2, 10);
    /// assert_eq!(20 * Region::Unreinforced.half_units() / 2, 30);
    /// ```
    pub fn half_units(self) -> u32 {
        match self {
            Self::Default => 2,
            Self::Reinforced => 1,
            Self::Unreinforced => 3,
        }
    }
}
/// Selection-time authority, fixed until another opportunity is spent.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Selection {
    /// Shared bottom-left detector coordinates in micrometres, never normalized.
    /// Nonnegative centers may lie outside an individual smaller plate.
    pub center_um: [u32; 2],
    /// Selected plate-area fraction in parts per billion, rounded once.
    /// None freezes default armor because selection-time accuracy was undefined.
    pub area_share_ppb: Option<u32>,
    /// Running-round selection time in nanoseconds.
    pub selected_ns: u64,
}
impl Selection {
    /// Circle radius squared in square micrometres for this detector size.
    /// Circle portions beyond plate edges are discarded without resizing.
    pub fn radius_squared_um2(&self, size_um: [u32; 2]) -> Option<f64> {
        self.area_share_ppb.map(|share| {
            f64::from(size_um[0]) * f64::from(size_um[1]) * f64::from(share)
                / 1e9
                / std::f64::consts::PI
        })
    }
    /// Classify a point using the frozen area; the circle boundary is reinforced.
    ///
    /// ```
    /// use rm_simulator_gameplay::rmuc_2027::adaptive::{Selection, Region};
    /// let selection = Selection {
    ///     center_um: [45_000, 44_000], area_share_ppb: Some(333_333_333),
    ///     selected_ns: 60_000_000_000,
    /// };
    /// assert_eq!(selection.region([45_000, 44_000], [90_000, 88_000]), Region::Reinforced);
    /// ```
    pub fn region(&self, position_um: [u32; 2], size_um: [u32; 2]) -> Region {
        let Some(radius2) = self.radius_squared_um2(size_um) else {
            return Region::Default;
        };
        let [x, y] =
            std::array::from_fn(|i| f64::from(position_um[i]) - f64::from(self.center_um[i]));
        if x * x + y * y <= radius2 {
            Region::Reinforced
        } else {
            Region::Unreinforced
        }
    }
}
/// Full round observations, selected regions and accumulating opportunities.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Adaptive {
    /// Successfully launched projectiles by attacking team.
    pub launches: [u32; 2],
    /// Accumulating one-minute selection opportunities (§5.2).
    pub opportunities: [u32; 2],
    /// Total selections by team; spent opportunities never return on death or transfer.
    pub selections: [u32; 2],
    /// Frozen selected centers and area fractions, Red then Blue.
    pub selected: [Option<Selection>; 2],
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
/// An undefined accuracy formula; selection applies the documented default-armor policy.
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
    /// require finite accuracy: undefined selection freezes default armor.
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
    /// Current formula-defined area fraction for non-singular teams (§5.2).
    /// Selection snapshots this value; subsequent launches/hits do not resize it.
    pub fn area_share(&self, team: Team) -> Result<f64, Undefined> {
        let rho = self.accuracy(team)?.accuracy_per_m / self.accuracy(team.other())?.accuracy_per_m;
        Ok(rho * rho / (2. + rho * rho))
    }
    /// Spend one opportunity to select/change the common center (§5.2).
    /// The caller supplies running time; undefined accuracy freezes default armor.
    /// No opportunity is spent when the ten-hit prerequisite is absent.
    pub fn select(
        &mut self,
        team: Team,
        center_um: [u32; 2],
        now_ns: u64,
    ) -> Result<(), &'static str> {
        if !self.selection_ready(team) {
            return Err("adaptive armor needs an opportunity and ten enemy hits");
        }
        let area_share_ppb = self.area_share(team).ok().map(|v| (v * 1e9).round() as u32);
        self.opportunities[team.index()] -= 1;
        self.selections[team.index()] += 1;
        self.selected[team.index()] = Some(Selection {
            center_um,
            area_share_ppb,
            selected_ns: now_ns,
        });
        Ok(())
    }
    /// Classify the victim's current plate; ownership follows the physical target.
    pub fn region(&self, victim: Team, position_um: [u32; 2], size_um: [u32; 2]) -> Region {
        self.selected[victim.index()]
            .as_ref()
            .map_or(Region::Default, |s| s.region(position_um, size_um))
    }
    /// Reject malformed coordinates and future observations before restore.
    pub fn validate(&self, elapsed_ns: u64) -> bool {
        Team::BOTH.into_iter().all(|team| {
            let i = team.index();
            let earned = elapsed_ns / 60_000_000_000;
            let selection_valid = match &self.selected[i] {
                None => self.selections[i] == 0,
                Some(s) => {
                    self.selections[i] > 0
                        && s.selected_ns <= elapsed_ns
                        && s.selected_ns >= 60_000_000_000
                        && u64::from(self.selections[i]) <= s.selected_ns / 60_000_000_000
                        && s.area_share_ppb.is_none_or(|v| v <= 1_000_000_000)
                        && self
                            .enemy_hits(team)
                            .filter(|h| h.time_ns <= s.selected_ns)
                            .count()
                            >= 10
                }
            };
            selection_valid
                && u64::from(self.opportunities[i]) + u64::from(self.selections[i]) <= earned
        }) && self.impacts.iter().all(|h| {
            h.time_ns <= elapsed_ns
                && (0..2).all(|i| h.size_um[i] > 0 && h.position_um[i] <= h.size_um[i])
        })
    }
}

impl super::match_rules::Match {
    /// Select native team reinforcement while running (§5.2). Referee/Radar
    /// authorization belongs to the host, never the deterministic engine.
    pub fn select_reinforcement(
        &mut self,
        team: Team,
        center_um: [u32; 2],
    ) -> Result<(), &'static str> {
        if self.phase != super::match_rules::Phase::Running {
            return Err("adaptive armor selection requires a running round");
        }
        self.adaptive.select(team, center_um, self.elapsed_ns())
    }
    /// Record one calibrated physical armor hit and apply its selected region.
    /// Target ownership, attacker, timestamp and classification are resolved here;
    /// caller-supplied observation values for those fields are overwritten.
    /// Zero-damage accepted contacts still enter the statistics (§5.2).
    pub fn armor_hit(
        &mut self,
        mut impact: Impact,
        shooter: Option<u32>,
        caliber: crate::Caliber,
    ) -> u32 {
        if self.phase != super::match_rules::Phase::Running
            || !(0..2).all(|i| impact.size_um[i] > 0 && impact.position_um[i] <= impact.size_um[i])
        {
            return 0;
        }
        impact.victim = match impact.target {
            Target::Base(team) => team,
            Target::Outpost(side) => self.control.snapshot().outposts[side.index()].owner,
            Target::Robot(id) => {
                let Some(robot) = self.robots.iter().find(|r| r.id == id) else {
                    return 0;
                };
                robot.team
            }
        };
        impact.attacker =
            shooter.and_then(|id| self.robots.iter().find(|r| r.id == id).map(|r| r.team));
        impact.time_ns = self.elapsed_ns();
        impact.region = self
            .adaptive
            .region(impact.victim, impact.position_um, impact.size_um);
        let target = impact.target;
        let region = impact.region;
        self.adaptive.impacts.push(impact);
        self.hit_with_region(target, shooter, caliber, region)
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
            region: Region::Default,
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
