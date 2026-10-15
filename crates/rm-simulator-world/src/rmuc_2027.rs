// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Geometry adapter for the RMUC 2027 V1.0.0 control/structure match subset.
pub use crate::gameplay::rmuc_2027::{Point, match_rules};
use crate::{ArmorTarget, Pose, TargetFace, Team};
use serde::{Deserialize, Serialize};

/// One fitted detector; dimensions are measured from the source CAD, not artwork.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Detector {
    /// Face at rest, in world FLU metres, +x outward.
    pub pose: Pose,
    /// Effective rectangular half-width/half-height in metres.
    pub half_m: [f64; 2],
}
/// Outpost source geometry, keyed by its original field side.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Tower {
    /// World FLU rotor pivot in metres.
    pub pivot_m: [f64; 3],
    /// Unit rotor axis in world FLU, including the source tower tilt.
    pub axis: [f64; 3],
    /// Three source detector faces at rest.
    pub detectors: Vec<Detector>,
}
/// A reviewed functional RFID footprint. Polygon x/y and floor are in FLU metres.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Zone {
    /// Original field side, independent of current controller.
    pub side: Team,
    /// Constituent control point.
    pub point: Point,
    /// Surface height, metres; supports elevated decks over tunnels.
    pub floor_m: f64,
    /// Height band above the surface in metres; simulator antenna approximation.
    pub height_m: f64,
    /// Footprint corners in world FLU x/y metres.
    pub polygon_m: Vec<[f64; 2]>,
}
impl Zone {
    /// Whether an antenna/body centre is inside the footprint and height band.
    pub fn contains(&self, p: [f64; 3]) -> bool {
        if p[2] < self.floor_m || p[2] > self.floor_m + self.height_m {
            return false;
        }
        let mut inside = false;
        for (a, b) in self
            .polygon_m
            .iter()
            .zip(self.polygon_m.iter().cycle().skip(1))
        {
            if (a[1] > p[1]) != (b[1] > p[1])
                && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
            {
                inside = !inside;
            }
        }
        inside
    }
}
/// Source-derived targets and field RFID, carried through authoritative restore.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Config {
    /// Bases, Red then Blue. Only source-present armor is bound.
    pub bases: [Vec<Detector>; 2],
    /// Outposts by original side, Red then Blue.
    pub outposts: [Tower; 2],
    /// Control-point footprints; never terrain-only traversal cards.
    pub zones: Vec<Zone>,
    /// Shared seeded direction for both rotors, either -1 or 1.
    pub rotor_direction: i8,
}
impl Config {
    /// Validate finite geometry, unit rotations, detector count and height bands.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.bases.iter().any(Vec::is_empty)
            || self.outposts.iter().any(|o| {
                o.detectors.len() != 3
                    || !o.pivot_m.iter().chain(&o.axis).all(|v| v.is_finite())
                    || (o.axis.iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-6
            })
            || ![-1, 1].contains(&self.rotor_direction)
        {
            return Err("invalid 2027 structure bindings");
        }
        for d in self
            .bases
            .iter()
            .flatten()
            .chain(self.outposts.iter().flat_map(|o| &o.detectors))
        {
            if !d
                .pose
                .translation_m
                .iter()
                .chain(&d.pose.rotation_wxyz)
                .all(|v| v.is_finite())
                || (d.pose.rotation_wxyz.iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-6
                || d.half_m
                    .iter()
                    .any(|v| !v.is_finite() || *v <= 0. || *v > 0.071)
            {
                return Err("invalid 2027 detector");
            }
        }
        if self.zones.iter().any(|z| {
            z.polygon_m.len() < 3
                || !z.floor_m.is_finite()
                || !(0.01..=0.5).contains(&z.height_m)
                || z.polygon_m.iter().flatten().any(|v| !v.is_finite())
        }) {
            return Err("invalid 2027 RFID zone");
        }
        Ok(())
    }
}
/// The entire hidden 2027 world adapter and rule authority.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct State {
    /// Immutable detector bindings and RFID geometry.
    pub config: Config,
    /// Deterministic match rules, including every expiry and progress timer.
    pub game: match_rules::Match,
}
impl State {
    /// World-facing target geometry at the current running-round time.
    pub(crate) fn faces(&self, faces: &mut Vec<TargetFace>) {
        for side in Team::BOTH {
            let s = side.index();
            for (i, d) in self.config.bases[s].iter().enumerate() {
                faces.push(TargetFace {
                    target: ArmorTarget::Base {
                        base: s as u32,
                        plate: i as u32,
                    },
                    pose: d.pose,
                });
            }
            for (i, d) in self.config.outposts[s].detectors.iter().enumerate() {
                faces.push(TargetFace {
                    target: ArmorTarget::Outpost {
                        outpost: s as u32,
                        face: i as u32,
                    },
                    pose: self.rotated(side, d.pose),
                });
            }
        }
    }
    /// Rotor angle in radians, with five-second spin-up and a permanent first stop.
    pub fn angle_rad(&self, side: Team) -> f64 {
        self.angle_at(side, self.game.elapsed_ns())
    }
    /// Presentation angle at an explicit running-round time, clamped by the
    /// permanent stop and three-minute rule; never advances the match.
    pub fn angle_at(&self, side: Team, round_ns: u64) -> f64 {
        let time = self.game.rotor_stopped_ns[side.index()]
            .unwrap_or(round_ns.min(match_rules::THREE_MIN_NS));
        let t = time as f64 * 1e-9;
        let phase = if t < 5. { t * t / 10. } else { t - 2.5 };
        phase * 0.8 * std::f64::consts::PI * f64::from(self.game.rotor_direction)
    }
    /// Rotate a source pose about this outpost's world-up rotor pivot.
    pub fn rotated(&self, side: Team, pose: Pose) -> Pose {
        self.rotated_at(side, pose, self.game.elapsed_ns())
    }
    /// Source pose at an explicit presentation round time; no rules are stepped.
    pub fn rotated_at(&self, side: Team, pose: Pose, round_ns: u64) -> Pose {
        let angle = self.angle_at(side, round_ns);
        let pivot = self.config.outposts[side.index()].pivot_m;
        let axis = self.config.outposts[side.index()].axis;
        let half = angle / 2.;
        let q = [
            half.cos(),
            axis[0] * half.sin(),
            axis[1] * half.sin(),
            axis[2] * half.sin(),
        ];
        let delta = std::array::from_fn(|i| pose.translation_m[i] - pivot[i]);
        let rotated = rm_simulator_physics::motion::outpost::rotate(q, delta);
        Pose {
            translation_m: std::array::from_fn(|i| pivot[i] + rotated[i]),
            rotation_wxyz: {
                let [a, b, c, d] = q;
                let [w, x, y, z] = pose.rotation_wxyz;
                [
                    a * w - b * x - c * y - d * z,
                    a * x + b * w + c * z - d * y,
                    a * y - b * z + c * w + d * x,
                    a * z + b * y - c * x + d * w,
                ]
            },
        }
    }
    pub(crate) fn detector(&self, target: ArmorTarget) -> Option<&Detector> {
        match target {
            ArmorTarget::Base { base, plate } => {
                self.config.bases.get(base as usize)?.get(plate as usize)
            }
            ArmorTarget::Outpost { outpost, face } => self
                .config
                .outposts
                .get(outpost as usize)?
                .detectors
                .get(face as usize),
            _ => None,
        }
    }
    pub(crate) fn disabled(&self, target: ArmorTarget) -> bool {
        match target {
            ArmorTarget::Base { base, .. } => self
                .game
                .base_hp
                .get(base as usize)
                .is_none_or(|&hp| hp == 0),
            ArmorTarget::Outpost { outpost, .. } => self
                .game
                .outpost_hp
                .get(outpost as usize)
                .is_none_or(|&hp| hp == 0),
            ArmorTarget::Chassis { chassis, .. } => self
                .game
                .robots
                .iter()
                .find(|r| r.id == chassis)
                .is_none_or(|r| r.hp == 0),
            _ => true,
        }
    }
}
impl crate::Field {
    /// Current 2027 authority, or `None` for a 2026 field/practice sandbox.
    ///
    /// ```
    /// use rm_simulator_world::{Field, FieldConfig};
    /// let field = Field::new(&FieldConfig::default()).unwrap();
    /// assert!(field.rmuc_2027().is_none());
    /// ```
    pub fn rmuc_2027(&self) -> Option<&State> {
        self.rmuc_2027.as_ref()
    }
    pub(crate) fn advance_2027(&mut self, duration_ns: u64) {
        let Some(state) = &mut self.rmuc_2027 else {
            return;
        };
        let observations = state
            .game
            .robots
            .iter()
            .flat_map(|r| {
                let p = self.physics.chassis_position_m(r.id);
                state
                    .config
                    .zones
                    .iter()
                    .map(move |z| (r.id, z.side, z.point, p.is_some_and(|p| z.contains(p))))
            })
            .collect::<Vec<_>>();
        for (id, side, point, present) in observations {
            state
                .game
                .observe(id, crate::referee::game_team(side), point, present);
        }
        state.game.advance_ns(duration_ns);
        for r in &state.game.robots {
            self.physics.set_chassis_defeated(r.id, r.hp == 0);
        }
    }
}

impl crate::FieldSnapshot {
    /// Prescribed scenery positions at an explicit presentation field time.
    /// 2027 motion uses its own running clock and never a 2026 referee record.
    pub fn mechanism_view(&self, time_ns: u64) -> crate::referee::MechanismState {
        let mut state = crate::referee::mechanism_view(self.referee.as_ref(), time_ns);
        if let Some(r) = self.rmuc_2027.as_ref() {
            let round_ns = r.game.elapsed_ns()
                + if r.game.phase == match_rules::Phase::Running {
                    time_ns.saturating_sub(self.time_ns)
                } else {
                    0
                };
            state.outpost_angle_rad = Team::BOTH.map(|side| r.angle_at(side, round_ns));
            state.base_open_fraction = r.game.base_open.map(|v| if v { 1. } else { 0. });
        }
        state
    }
}

#[cfg(all(test, feature = "season-2027"))]
mod tests {
    use super::*;
    use crate::{
        Caliber, ChassisConfig, ChassisPlacement, Field, FieldConfig, RefereeCommand, RobotKind,
        Shot, StaticGeometry, tick_ns,
    };
    fn config() -> Config {
        let detector = |x, y| Detector {
            pose: Pose::at([x, y, 1.]),
            half_m: [0.045, 0.044],
        };
        Config {
            bases: [vec![detector(7., 0.)], vec![detector(-7., 0.)]],
            outposts: [
                Tower {
                    pivot_m: [3., 0., 1.],
                    axis: [0., 0., 1.],
                    detectors: (0..3).map(|i| detector(3., i as f64)).collect(),
                },
                Tower {
                    pivot_m: [-3., 0., 1.],
                    axis: [0., 0., 1.],
                    detectors: (0..3).map(|i| detector(-3., i as f64)).collect(),
                },
            ],
            zones: vec![Zone {
                side: Team::Blue,
                point: Point::CentralHighland,
                floor_m: 0.,
                height_m: 0.45,
                polygon_m: vec![[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]],
            }],
            rotor_direction: -1,
        }
    }
    fn field() -> Field {
        Field::new(&FieldConfig {
            rmuc_2027: Some(config()),
            runes: vec![],
            outposts: vec![],
            ..Default::default()
        })
        .unwrap()
    }
    #[test]
    fn antenna_band_rejects_floor_below_deck_and_aerial_positions() {
        let zone = &config().zones[0];
        assert!(zone.contains([0., 0., 0.15]));
        assert!(!zone.contains([0., 0., -0.01]));
        assert!(!zone.contains([0., 0., 0.46]));
        assert!(!zone.contains([1.1, 0., 0.15]));
    }
    #[test]
    fn real_chassis_rfid_capture_and_restore_keep_hidden_timers() {
        let mut field = field();
        field
            .add_chassis(&ChassisPlacement {
                config: ChassisConfig::default(),
                spawn: Pose::at([0., 0., 0.15]),
                team: Team::Red,
                kind: RobotKind::Infantry,
                performance: None,
            })
            .unwrap();
        field.referee_command(RefereeCommand::StartMatch).unwrap();
        field.step(5_000_000_000 / tick_ns()).unwrap();
        field.step(3_250_000_000 / tick_ns()).unwrap();
        let snapshot = field.snapshot();
        let geometry = field.static_geometry_snapshot();
        let mut restored = Field::restore(&snapshot, &geometry, 0.).unwrap();
        assert_eq!(restored.rmuc_2027(), field.rmuc_2027());
        field.step(3_000_000_000 / tick_ns()).unwrap();
        for _ in 0..3 {
            restored.step(1_000_000_000 / tick_ns()).unwrap();
        }
        assert_eq!(restored.rmuc_2027(), field.rmuc_2027());
        assert_eq!(
            field.rmuc_2027().unwrap().game.control.snapshot().points,
            [2, 0]
        );
    }
    #[test]
    fn presentation_survives_dropping_restore_and_rejects_mismatched_authority() {
        let mut f = field();
        f.referee_command(RefereeCommand::StartMatch).unwrap();
        f.step(7_000_000_000 / tick_ns()).unwrap();
        let mut snapshot = f.snapshot();
        let mechanisms = snapshot.mechanism_view(snapshot.time_ns);
        assert_ne!(mechanisms.outpost_angle_rad, [0.; 2]);
        let saved = snapshot.restore.take();
        assert_eq!(snapshot.rmuc_2027.as_ref(), f.rmuc_2027());
        assert_eq!(snapshot.mechanism_view(snapshot.time_ns), mechanisms);
        assert!(Field::restore(&snapshot, &StaticGeometry::default(), 0.).is_err());
        snapshot.restore = saved;
        snapshot.rmuc_2027 = None;
        assert!(Field::restore(&snapshot, &StaticGeometry::default(), 0.).is_err());
    }
    #[test]
    fn source_detector_rectangle_and_flat_damage_survive_physical_hits() {
        let mut f = field();
        f.rmuc_2027.as_mut().unwrap().game.phase = match_rules::Phase::Running;
        let shot = |y| Pose::yawed([3.15, y, 1.], std::f64::consts::PI);
        f.fire(shot(0.), Shot::at_limit(Caliber::Mm17), None)
            .unwrap();
        f.step(3).unwrap();
        assert_eq!(f.rmuc_2027().unwrap().game.outpost_hp[0], 980);
        let hit = f.snapshot().hits.into_iter().find(|h| h.detected).unwrap();
        assert_eq!(hit.damage, 20);
        // A hit outside the source detector, but inside the old 2026 area,
        // must not score on the new fitted face.
        f.fire(shot(0.049), Shot::at_limit(Caliber::Mm17), None)
            .unwrap();
        f.step(12).unwrap();
        assert_eq!(f.rmuc_2027().unwrap().game.outpost_hp[0], 980);
    }
    #[test]
    fn idle_world_still_advances_live_rule_countdown_and_rejects_mixed_checkpoints() {
        let mut f = field();
        f.referee_command(RefereeCommand::StartMatch).unwrap();
        f.step(6_000_000_000 / tick_ns()).unwrap();
        assert_eq!(f.rmuc_2027().unwrap().game.elapsed_ns(), 1_000_000_000);
        let mut snapshot = f.snapshot();
        snapshot
            .restore
            .as_mut()
            .unwrap()
            .rmuc_2027
            .as_mut()
            .unwrap()
            .game
            .rotor_direction = 0;
        assert!(Field::restore(&snapshot, &StaticGeometry::default(), 0.).is_err());
    }
}
