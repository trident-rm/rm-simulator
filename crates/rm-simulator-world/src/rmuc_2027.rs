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
    /// Own supply footprints, separately calibrated; point is ignored.
    #[serde(default)]
    pub supply_zones: Vec<Zone>,
    /// Own vantage footprints for zone exchanges; point is ignored.
    #[serde(default)]
    pub vantage_zones: Vec<Zone>,
    /// Physical traversal cards; their zone's point field is unused geometry metadata.
    #[serde(default)]
    pub terrain_zones: Vec<TerrainZone>,
    /// Native Rune source geometry in Red/Blue order; empty for earlier unbound packages.
    #[serde(default)]
    pub runes: Vec<RuneGeometry>,
    /// Shared seeded direction for both rotors, either -1 or 1.
    pub rotor_direction: i8,
}
/// One native physical traversal card and its fitted antenna footprint.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TerrainZone {
    /// Exact native side/course/pad identity, independent of control activations.
    pub card: crate::gameplay::rmuc_2027::terrain::Card,
    /// Reviewed polygon/height geometry; point does not identify the traversal.
    pub zone: Zone,
}
/// Source-fitted Rune target disks and shared physical shaft frame, by face team.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RuneGeometry {
    /// Shaft pivot in world FLU metres; any point along the common axis is equivalent.
    pub pivot_m: [f64; 3],
    /// Shared unit world axis, the same physical sign for both opposing faces.
    pub axis: [f64; 3],
    /// Five source-fitted detector disk poses at rest; +x is outward.
    pub targets: [Pose; 5],
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
        if self
            .zones
            .iter()
            .chain(&self.supply_zones)
            .chain(&self.vantage_zones)
            .chain(self.terrain_zones.iter().map(|z| &z.zone))
            .any(|z| {
                z.polygon_m.len() < 3
                    || !z.floor_m.is_finite()
                    || !(0.01..=0.5).contains(&z.height_m)
                    || z.polygon_m.iter().flatten().any(|v| !v.is_finite())
            })
        {
            return Err("invalid 2027 RFID zone");
        }
        if self.terrain_zones.iter().any(|z| {
            !z.card.valid()
                || z.card.course.is_none()
                || z.card.side != crate::referee::game_team(z.zone.side)
        }) {
            return Err("invalid 2027 traversal identity");
        }
        if !self.runes.is_empty()
            && (self.runes.len() != 2
                || self.runes.iter().any(|r| {
                    r.pivot_m.iter().chain(&r.axis).any(|v| !v.is_finite())
                        || (r.axis.iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-6
                        || r.targets.iter().any(|p| {
                            p.translation_m
                                .iter()
                                .chain(&p.rotation_wxyz)
                                .any(|v| !v.is_finite())
                                || (p.rotation_wxyz.iter().map(|v| v * v).sum::<f64>() - 1.).abs()
                                    > 1e-6
                        })
                }))
        {
            return Err("invalid native Rune geometry");
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
            if let Some(rune) = self.config.runes.get(s) {
                for (blade, pose) in rune.targets.iter().enumerate() {
                    faces.push(TargetFace {
                        target: ArmorTarget::Rune {
                            rune: s as u32,
                            blade: blade as u32,
                        },
                        pose: self.rune_pose_at(side, *pose, self.game.elapsed_ns()),
                    });
                }
            }
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
    /// Rotate one source Rune target at an explicit running-round timestamp.
    pub fn rune_pose_at(&self, side: Team, pose: Pose, round_ns: u64) -> Pose {
        let Some(r) = self.config.runes.get(side.index()) else {
            return pose;
        };
        let angle = self.game.runes[side.index()].angle_at(round_ns);
        Self::rotate_source(pose, r.pivot_m, r.axis, angle)
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
        Self::rotate_source(pose, pivot, axis, angle)
    }
    fn rotate_source(pose: Pose, pivot: [f64; 3], axis: [f64; 3], angle: f64) -> Pose {
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
            ArmorTarget::Rune { rune, .. } => self.config.runes.get(rune as usize).is_none(),
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
        // Sequences consume physical entry edges, not delayed buff activations.
        for r in state.game.robots.clone() {
            use crate::gameplay::rmuc_2027::terrain::{Card, Course};
            let p = self.physics.chassis_position_m(r.id);
            let mut cards = state
                .config
                .terrain_zones
                .iter()
                .filter(|z| p.is_some_and(|p| z.zone.contains(p)))
                .map(|z| z.card)
                .collect::<Vec<_>>();
            for z in &state.config.zones {
                // Higher Highland cards also provide connected Highland/Outpost
                // detection. Coalesce those semantic roles into one physical card.
                let shared = matches!(z.point, Point::CentralHighland | Point::Outpost)
                    && cards.iter().any(|c| {
                        c.side == crate::referee::game_team(z.side)
                            && c.course == Some(Course::Highland)
                            && c.pad == 1
                    });
                if !shared && p.is_some_and(|p| z.contains(p)) {
                    cards.push(Card {
                        side: crate::referee::game_team(z.side),
                        course: None,
                        index: match z.point {
                            Point::Base => 0,
                            Point::Fortress => 1,
                            Point::Outpost => 2,
                            Point::CentralHighland => 3,
                        },
                        pad: 0,
                    });
                }
            }
            for (index, zones) in [
                (4, &state.config.supply_zones),
                (5, &state.config.vantage_zones),
            ] {
                for z in zones {
                    if p.is_some_and(|p| z.contains(p)) {
                        cards.push(Card {
                            side: crate::referee::game_team(z.side),
                            course: None,
                            index,
                            pad: 0,
                        });
                    }
                }
            }
            state.game.observe_terrain(r.id, &cards);
        }
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
        for r in &state.game.robots.clone() {
            let p = self.physics.chassis_position_m(r.id);
            for (supply, zones) in [
                (true, &state.config.supply_zones),
                (false, &state.config.vantage_zones),
            ] {
                let present = zones.iter().any(|z| {
                    crate::referee::game_team(z.side) == r.team && p.is_some_and(|p| z.contains(p))
                });
                state.game.observe_service(r.id, supply, present);
            }
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
            state.rune_angle_rad = r.game.runes.each_ref().map(|rune| rune.angle_at(round_ns));
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
            supply_zones: vec![],
            vantage_zones: vec![],
            terrain_zones: vec![],
            runes: vec![],
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
    fn heavy_conversion_keeps_identity_ammunition_damage_and_restore() {
        use crate::gameplay::rmuc_2027::match_rules::{COUNTDOWN_NS, GroundRole};
        let mut f = field();
        let config = ChassisConfig::hero();
        let id = f
            .add_chassis(&ChassisPlacement {
                config: config.clone(),
                spawn: Pose::at([0., 0., config.rest_height_m()]),
                team: Team::Red,
                kind: RobotKind::Engineer,
                performance: None,
            })
            .unwrap();
        let victim = f
            .add_chassis(&ChassisPlacement {
                config: config.clone(),
                spawn: Pose::at([3., 0., config.rest_height_m()]),
                team: Team::Blue,
                kind: RobotKind::Infantry,
                performance: None,
            })
            .unwrap();
        assert_eq!(
            f.rmuc_2027().unwrap().game.robots[0].role,
            GroundRole::Heavy
        );
        assert!(f.convert_heavy_to_hero(id).is_err());
        f.referee_command(RefereeCommand::StartMatch).unwrap();
        // Advance only rules here: physics parking remains fixed for the conversion check.
        let g = &mut f.rmuc_2027.as_mut().unwrap().game;
        g.advance_ns(COUNTDOWN_NS + 299_000_000_000);
        assert!(!g.heavy_unlocked(crate::gameplay::Team::Red));
        g.advance_ns(1_000_000_000);
        let before = f.snapshot().chassis[0].clone();
        f.convert_heavy_to_hero(id).unwrap();
        assert!(f.convert_heavy_to_hero(id).is_err());
        let after = f.snapshot().chassis[0].clone();
        assert_eq!((after.id, after.team), (before.id, before.team));
        assert_eq!(
            after.pose.translation_m[..2],
            before.pose.translation_m[..2]
        );
        assert!(after.config.folding.is_some());
        assert!(after.placement_revision > before.placement_revision);
        assert_eq!(f.rmuc_2027().unwrap().game.robots[0].max_hp, 600);
        assert!(
            f.fire(
                Pose::at([0., 0., 2.]),
                Shot::at_limit(Caliber::Mm42),
                Some(id)
            )
            .is_err()
        );
        f.buy_ammo(id, Caliber::Mm42).unwrap(); // ten remote shells
        f.rmuc_2027.as_mut().unwrap().game.advance_ns(6_000_000_000);
        f.fire(
            Pose::at([0., 0., 2.]),
            Shot {
                speed_m_s: 16.,
                ..Shot::at_limit(Caliber::Mm42)
            },
            Some(id),
        )
        .unwrap();
        let game = &mut f.rmuc_2027.as_mut().unwrap().game;
        assert_eq!(game.robots[0].combat.allowance, 9);
        assert_eq!(game.robots[0].combat.heat_tenths, 1000);
        assert_eq!(
            game.hit(
                crate::gameplay::Target::Robot(victim),
                Some(id),
                crate::gameplay::Caliber::Mm42
            ),
            200
        );
        let snapshot = f.snapshot();
        let restored = Field::restore(&snapshot, &f.static_geometry_snapshot(), 0.).unwrap();
        assert_eq!(restored.chassis_config(id), f.chassis_config(id));
        assert_eq!(restored.rmuc_2027(), f.rmuc_2027());
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
    fn virtual_assembly_commands_restore_timeout_and_finish() {
        let mut f = field();
        f.referee_command(RefereeCommand::StartMatch).unwrap();
        f.step(640).unwrap();
        f.referee_command(RefereeCommand::StartAssembly {
            team: Team::Red,
            difficulty: 1,
        })
        .unwrap();
        f.step(128).unwrap();
        let mut restored = Field::restore(&f.snapshot(), &StaticGeometry::default(), 0.).unwrap();
        for field in [&mut f, &mut restored] {
            field
                .referee_command(RefereeCommand::FinishAssembly { team: Team::Red })
                .unwrap();
            field
                .referee_command(RefereeCommand::StartAssembly {
                    team: Team::Blue,
                    difficulty: 1,
                })
                .unwrap();
            field.step(45 * 128).unwrap();
            assert!(
                field
                    .referee_command(RefereeCommand::FinishAssembly { team: Team::Blue })
                    .is_err()
            );
            let game = &field.rmuc_2027().unwrap().game;
            assert_eq!(game.assembly[0].income_per_10s, 40);
            assert_eq!(
                game.assembly[1].outcome,
                crate::gameplay::rmuc_2027::assembly::Outcome::TimedOut
            );
            game.validate().unwrap();
        }
        assert_eq!(f.snapshot().rmuc_2027, restored.snapshot().rmuc_2027);
        #[cfg(feature = "season-2026")]
        {
            let mut legacy = Field::new(&FieldConfig::default()).unwrap();
            assert!(
                legacy
                    .referee_command(RefereeCommand::StartAssembly {
                        team: Team::Red,
                        difficulty: 1
                    })
                    .is_err()
            );
        }
    }
    #[test]
    fn native_rune_collision_uses_rune_driver_and_survives_restore() {
        let mut f = field();
        let vertices = vec![[21., 0., 2.], [21., 0.1, 2.], [21., 0., 2.1]];
        let pivot = [20., 0., 2.];
        f.add_rotating_mesh(
            vertices.clone(),
            vec![[0, 1, 2]],
            crate::referee::Mechanism::Rune,
            Team::Red,
            pivot,
            [0., 0., 1.],
        )
        .unwrap();
        f.referee_command(RefereeCommand::StartMatch).unwrap();
        f.step(640 + 128).unwrap();
        let angle = f.rmuc_2027().unwrap().game.runes[0].angle_at(1_000_000_000);
        let capture = |f: &Field| {
            f.dynamic_geometry_snapshot()
                .into_triangles()
                .0
                .into_iter()
                .filter(|p| p[0] > 19.)
                .collect::<Vec<_>>()
        };
        let actual = capture(&f);
        for (original, moved) in vertices.iter().zip(&actual) {
            let x = original[0] - pivot[0];
            let y = original[1] - pivot[1];
            let expected = [
                pivot[0] + x * angle.cos() - y * angle.sin(),
                pivot[1] + x * angle.sin() + y * angle.cos(),
                original[2],
            ];
            for i in 0..3 {
                assert!(
                    (moved[i] - expected[i]).abs() < 1e-10,
                    "actual={actual:?} expected={expected:?} angle={angle}"
                );
            }
        }
        let mut restored =
            Field::restore(&f.snapshot(), &f.static_geometry_snapshot(), 0.).unwrap();
        assert_eq!(actual, capture(&restored));
        f.step(128).unwrap();
        restored.step(128).unwrap();
        assert_eq!(capture(&f), capture(&restored));
    }

    #[test]
    fn native_rune_physical_disk_scores_and_restores_without_a_legacy_referee() {
        let mut config = config();
        config.runes = Team::BOTH
            .map(|side| {
                let x = if side == Team::Red { 5. } else { -5. };
                RuneGeometry {
                    pivot_m: [x, 0., 2.],
                    axis: [1., 0., 0.],
                    targets: std::array::from_fn(|blade| {
                        let angle = blade as f64 * std::f64::consts::TAU / 5.;
                        Pose::yawed(
                            [x, 0.7 * angle.sin(), 2. + 0.7 * angle.cos()],
                            if side == Team::Red {
                                0.
                            } else {
                                std::f64::consts::PI
                            },
                        )
                    }),
                }
            })
            .to_vec();
        let mut f = Field::new(&FieldConfig {
            rmuc_2027: Some(config),
            runes: vec![],
            outposts: vec![],
            ..Default::default()
        })
        .unwrap();
        f.referee_command(RefereeCommand::StartMatch).unwrap();
        f.step(640).unwrap();
        f.referee_command(RefereeCommand::ActivateRune { team: Team::Red })
            .unwrap();
        let native = f.rmuc_2027().unwrap();
        let blade = native.game.runes[0]
            .targets
            .iter()
            .position(|&v| v)
            .unwrap();
        let p = native.config.runes[0].targets[blade].translation_m;
        f.fire(
            Pose::yawed([p[0] + 0.2, p[1], p[2]], std::f64::consts::PI),
            Shot::at_limit(Caliber::Mm17),
            None,
        )
        .unwrap();
        f.step(8).unwrap();
        assert_eq!(f.rmuc_2027().unwrap().game.runes[0].groups, 1);
        let checkpoint = f.snapshot();
        assert!(checkpoint.runes.is_empty() && checkpoint.referee.is_none());
        let mut restored = Field::restore(&checkpoint, &f.static_geometry_snapshot(), 0.).unwrap();
        f.step(512).unwrap();
        restored.step(512).unwrap();
        assert_eq!(f.rmuc_2027(), restored.rmuc_2027());
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
    fn leaving_a_physical_zone_expires_on_the_same_tick_after_restore() {
        let mut f = field();
        let id = f
            .add_chassis(&ChassisPlacement {
                config: ChassisConfig::default(),
                spawn: Pose::at([0., 0., 0.15]),
                team: Team::Red,
                kind: RobotKind::Infantry,
                performance: None,
            })
            .unwrap();
        f.referee_command(RefereeCommand::StartMatch).unwrap();
        f.step(6_000_000_000 / tick_ns()).unwrap();
        let game = &f.rmuc_2027().unwrap().game;
        let expiry = game.robots[0].expires_ns[5].unwrap();
        let until_expiry = (expiry - game.elapsed_ns()) / tick_ns();
        f.place_chassis(id, Pose::at([4., 4., 0.15])).unwrap();
        f.step(until_expiry - 1).unwrap();
        assert_eq!(
            f.rmuc_2027().unwrap().game.control.snapshot().territories[1].activations[1],
            [true, false]
        );
        let snapshot = f.snapshot();
        let geometry = f.static_geometry_snapshot();
        let mut restored = Field::restore(&snapshot, &geometry, 0.).unwrap();
        for field in [&mut f, &mut restored] {
            field.step(1).unwrap();
            let game = &field.rmuc_2027().unwrap().game;
            assert_eq!(game.elapsed_ns(), expiry);
            assert_eq!(
                game.control.snapshot().territories[1].activations,
                [[false; 2]; 2]
            );
        }
        f.step(128).unwrap();
        for _ in 0..128 {
            restored.step(1).unwrap();
        }
        assert_eq!(f.rmuc_2027(), restored.rmuc_2027());
    }

    #[test]
    fn two_physical_chassis_contest_base_until_departure_grace_expires() {
        let mut config = config();
        config.zones[0].point = Point::Base;
        let mut f = Field::new(&FieldConfig {
            rmuc_2027: Some(config),
            runes: vec![],
            outposts: vec![],
            ..Default::default()
        })
        .unwrap();
        let mut ids = vec![];
        for (team, x) in [(Team::Red, -0.5), (Team::Blue, 0.5)] {
            ids.push(
                f.add_chassis(&ChassisPlacement {
                    config: ChassisConfig::default(),
                    spawn: Pose::at([x, 0., 0.15]),
                    team,
                    kind: RobotKind::Infantry,
                    performance: None,
                })
                .unwrap(),
            );
        }
        f.referee_command(RefereeCommand::StartMatch).unwrap();
        f.step(8_000_000_000 / tick_ns()).unwrap();
        let territory = &f.rmuc_2027().unwrap().game.control.snapshot().territories[3];
        assert_eq!(territory.activations[0], [true, true]);
        assert_eq!(territory.progress_ns, 0);
        f.place_chassis(ids[1], Pose::at([4., 4., 0.15])).unwrap();
        f.step(6_000_000_000 / tick_ns()).unwrap();
        assert_eq!(
            f.rmuc_2027().unwrap().game.control.snapshot().territories[3].control,
            crate::gameplay::rmuc_2027::ControlState::Full(crate::gameplay::Team::Blue)
        );
        f.step(1_000_000_000 / tick_ns()).unwrap();
        assert_eq!(
            f.rmuc_2027().unwrap().game.control.snapshot().territories[3].control,
            crate::gameplay::rmuc_2027::ControlState::Temporary(crate::gameplay::Team::Red)
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
    fn physical_adaptive_hits_use_frozen_area_and_restore_regions() {
        use crate::gameplay::rmuc_2027::{
            adaptive::{Impact, Region},
            match_rules::GroundRole,
        };
        use crate::gameplay::{Target, Team as GTeam};
        let mut f = field();
        let g = &mut f.rmuc_2027.as_mut().unwrap().game;
        g.add_robot(1, GTeam::Red, GroundRole::Infantry);
        g.add_robot(2, GTeam::Blue, GroundRole::Infantry);
        g.start(1);
        g.advance_ns(match_rules::COUNTDOWN_NS + 60_000_000_000);
        g.adaptive.launches = [20; 2];
        for team in GTeam::BOTH {
            for i in 0..10 {
                g.armor_hit(
                    Impact {
                        time_ns: 0,
                        attacker: None,
                        victim: team.other(),
                        target: Target::Base(team.other()),
                        plate: 0,
                        position_um: [10_000 + i % 2 * 20_000, 20_000],
                        size_um: [90_000, 88_000],
                        region: Region::Default,
                    },
                    Some(if team == GTeam::Red { 1 } else { 2 }),
                    crate::gameplay::Caliber::Mm17,
                );
            }
        }
        f.referee_command(RefereeCommand::SelectReinforcement {
            team: Team::Red,
            center_um: [45_000, 44_000],
        })
        .unwrap();
        f.fire(
            Pose::yawed([3.15, 0., 1.], std::f64::consts::PI),
            Shot::at_limit(Caliber::Mm17),
            None,
        )
        .unwrap();
        f.step(8).unwrap();
        let game = &f.rmuc_2027().unwrap().game;
        assert_eq!(game.outpost_hp[0], 990);
        assert_eq!(
            game.adaptive.impacts.last().unwrap().region,
            Region::Reinforced
        );
        let mut restored = Field::restore(&f.snapshot(), &StaticGeometry::default(), 0.).unwrap();
        for field in [&mut f, &mut restored] {
            field
                .fire(
                    Pose::yawed([3.15, 0.04, 1.04], std::f64::consts::PI),
                    Shot::at_limit(Caliber::Mm17),
                    None,
                )
                .unwrap();
            field.step(4).unwrap();
            let game = &field.rmuc_2027().unwrap().game;
            assert_eq!(game.outpost_hp[0], 960);
            assert_eq!(
                game.adaptive.impacts.last().unwrap().region,
                Region::Unreinforced
            );
            game.validate().unwrap();
        }
        assert_eq!(f.rmuc_2027(), restored.rmuc_2027());
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
        let observations = &f.rmuc_2027().unwrap().game.adaptive.impacts;
        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].attacker, None);
        assert_eq!(observations[0].victim, crate::gameplay::Team::Red);
        assert_eq!(observations[0].size_um, [90000, 88000]);
        assert!(observations[0].position_um.iter().all(|&v| v > 0));
        // A hit outside the source detector, but inside the old 2026 area,
        // must not score on the new fitted face.
        f.fire(shot(0.049), Shot::at_limit(Caliber::Mm17), None)
            .unwrap();
        f.step(12).unwrap();
        assert_eq!(f.rmuc_2027().unwrap().game.outpost_hp[0], 980);
    }
    #[test]
    fn live_combat_enforces_allowance_and_restores_pending_respawn() {
        let mut f = field();
        let id = f
            .add_chassis(&ChassisPlacement {
                config: ChassisConfig::default(),
                spawn: Pose::at([0., 0., 0.16]),
                team: Team::Blue,
                kind: RobotKind::Infantry,
                performance: None,
            })
            .unwrap();
        f.referee_command(RefereeCommand::StartMatch).unwrap();
        f.step(5_000_000_000 / tick_ns()).unwrap();
        assert!(
            f.fire(
                Pose::at([0., 0., 2.]),
                Shot::at_limit(Caliber::Mm17),
                Some(id)
            )
            .is_err()
        );
        f.buy_ammo(id, Caliber::Mm17).unwrap();
        f.step(6_000_000_000 / tick_ns()).unwrap();
        f.fire(
            Pose::at([0., 0., 2.]),
            Shot::at_limit(Caliber::Mm17),
            Some(id),
        )
        .unwrap();
        let r = &f.rmuc_2027().unwrap().game.robots[0];
        assert_eq!(r.combat.allowance, 99);
        assert_eq!(r.combat.heat_tenths, 100);
        assert_eq!(f.rmuc_2027().unwrap().game.adaptive.launches, [0, 1]);
        f.referee_command(RefereeCommand::DamageRobot {
            robot: id,
            amount: 1000,
        })
        .unwrap();
        f.step(256).unwrap();
        let snapshot = f.snapshot();
        let geometry = f.static_geometry_snapshot();
        let mut restored = Field::restore(&snapshot, &geometry, 0.).unwrap();
        f.step(12_000_000_000 / tick_ns()).unwrap();
        for _ in 0..12 {
            restored.step(1_000_000_000 / tick_ns()).unwrap();
        }
        assert_eq!(f.rmuc_2027(), restored.rmuc_2027());
        let r = &f.rmuc_2027().unwrap().game.robots[0];
        assert!(r.hp > 0);
        assert!(r.combat.weakened);
        assert!(!r.combat.overheated);
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
