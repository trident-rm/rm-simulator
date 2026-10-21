// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Team-facing enemy contacts derived only from Radar/decoded observation sources.
//! Full prediction checkpoints are not an anti-cheat secrecy boundary. These
//! records define the presentation contract, independently of checkpoint truth.
use rm_simulator_world::{FieldSnapshot, Team, referee::game_team};
use serde::{Deserialize, Serialize};

/// Source of a displayed enemy position; other fields have independent provenance.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Source {
    /// Current camera-cone and line-of-sight detection by an allied Sentry.
    Optical,
    /// Last Radar-reported XY; not the authoritative robot transform.
    Reported,
    /// Table 5-30 permits authoritative localization at progress >=100.
    Localized,
    /// Independently decoded information-wave position.
    Information,
}
/// A bounded-lifetime enemy detection available to one observing team.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Contact {
    /// Stable target chassis identity, never an enemy vector index.
    pub robot: u32,
    /// Target ownership.
    pub team: Team,
    /// Reported/localized/decoded FLU metres. Reported XY has no known Z (zero).
    pub position_m: [f64; 3],
    /// Provenance of the displayed position.
    pub source: Source,
    /// Radar marking progress in tenths, 0..=1500.
    pub progress_tenths: i32,
    /// Independently decoded HP; position observations do not disclose HP.
    pub hp: Option<u32>,
    /// Independently decoded HP ceiling; never taken from hidden progression.
    pub max_hp: Option<u32>,
    /// Independently decoded remaining allowance.
    pub allowance: Option<u32>,
    /// Independently decoded defense percentage.
    pub defense_pct: Option<u32>,
    /// Position reception age in nanoseconds; localized positions are current.
    pub age_ns: u64,
}
/// Build enemy contacts for a team. Unknown HP/ammunition remain unknown;
/// expired information is never refreshed from the hidden robot state.
/// Unlocalized Radar reports disappear after two seconds (simulator UI policy).
///
/// ```
/// use rm_simulator_server::intelligence::contacts;
/// use rm_simulator_world::{Field, FieldConfig, Team};
/// let field=Field::new(&FieldConfig::default()).unwrap();
/// assert!(contacts(&field.snapshot(),Team::Red).is_empty());
/// ```
pub fn contacts(snapshot: &FieldSnapshot, observer: Team) -> Vec<Contact> {
    let Some(state) = &snapshot.rmuc_2027 else {
        return vec![];
    };
    let game = &state.game;
    let now = game.elapsed_ns();
    let i = game_team(observer).index();
    game.robots
        .iter()
        .filter(|r| r.team != game_team(observer))
        .filter_map(|r| {
            let mark = game.radar.marks[i].get(&r.id)?;
            let info = game.radar.information[i].get(&r.id).filter(|o| {
                now.saturating_sub(o.observed_ns)
                    < rm_simulator_world::gameplay::rmuc_2027::radar::INFORMATION_NS
            });
            let body = snapshot.chassis.iter().find(|c| c.id == r.id);
            let (position_m, source, age_ns) = if mark.localized(false)
                && let Some(body) = body
            {
                (body.pose.translation_m, Source::Localized, 0)
            } else if let Some(info) = info
                && let Some(p) = info.position_mm
            {
                (
                    p.map(|v| f64::from(v) / 1000.),
                    Source::Information,
                    now - info.observed_ns,
                )
            } else if let (Some(p), Some(at)) = (mark.position_mm, mark.reported_ns)
                && now.saturating_sub(at) < 2_000_000_000
            {
                (
                    [f64::from(p[0]) / 1000., f64::from(p[1]) / 1000., 0.],
                    Source::Reported,
                    now - at,
                )
            } else {
                return None;
            };
            Some(Contact {
                robot: r.id,
                team: observer.other(),
                position_m,
                source,
                progress_tenths: mark.progress_tenths,
                hp: info.and_then(|o| o.hp),
                max_hp: info.and_then(|o| o.max_hp),
                allowance: info.and_then(|o| o.allowance),
                defense_pct: info.and_then(|o| o.defense_pct),
                age_ns,
            })
        })
        .collect()
}
