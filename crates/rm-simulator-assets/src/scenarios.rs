// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Reusable standalone equipment presets over one ordinary physical session.
//! Geometry is cooked once and shared across resets. No match, host or render loop.
use crate::{
    cad_assets::CadAssets,
    components::{self, ComponentRole},
};
use anyhow::{Context, ensure};
use rm_simulator_physics::{
    ArmorTarget, Pose, StaticGeometry, StepDuration, TargetFace, Team, WorldPhysics,
    arena::{Detector, OutpostModel, TunnelModel},
    chassis::ChassisConfig,
    motion::{Mechanism, MechanismState},
    session::{RobotScenario, RobotSession},
};
use std::path::PathBuf;

pub mod composition;

/// Small isolated setups; source presets retain their approved arena placement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Preset {
    /// One team's complete source Outpost and its three detectors.
    Outpost(Team),
    /// One team's complete source Base and its bound detector faces.
    Base(Team),
    /// Complete source Rune with both five-target banks.
    Rune,
    /// Illustrative 2 × 1 × 0.25 m passage, not official arena geometry.
    TunnelFixture,
}
/// Initial physical robot, with identities allocated in this list's order.
#[derive(Clone, Debug)]
pub struct RobotSpawn {
    /// Physical team label, without match admission or permissions.
    pub team: Team,
    /// Validated physical dimensions, motors and collision profile.
    pub config: ChassisConfig,
    /// Initial body-centre pose in world FLU metres.
    pub pose: Pose,
}
/// Optional source rotation for a detector; coordinate is supplied by the caller.
#[derive(Clone, Copy, Debug)]
pub struct TargetRotation {
    /// Caller-driven Outpost or Rune channel.
    pub mechanism: Mechanism,
    /// Source channel side/face, independent of a new task's team policy.
    pub team: Team,
    /// Source pivot, world FLU metres.
    pub pivot_m: [f64; 3],
    /// Unit source axis in world FLU.
    pub axis: [f64; 3],
}
/// A scoring face definition; detection and game consequences remain optional.
#[derive(Clone, Debug)]
pub struct ScenarioTarget {
    /// Physical identity and rest pose, preserving native side/face numbering.
    pub face: TargetFace,
    /// Fitted source rectangle, or None for Rune's existing ring/artwork detection.
    pub detector: Option<Detector>,
    /// Prescribed rotation, or None for a stationary source detector.
    pub rotation: Option<TargetRotation>,
}
/// Original checked visual file and its root, for a separate passive inspector.
#[derive(Clone, Debug)]
pub struct VisualInstance {
    /// Verified package visual GLB path; materials remain source-owned.
    pub file: PathBuf,
    /// Exported root pose including the loader's glTF basis convention.
    pub placement: Pose,
}
/// Immutable reusable scenario definition, distinct from a running session.
/// Source geometry is shared by reference across new sessions and recipe resets.
#[derive(Clone)]
pub struct Scenario {
    preset: Preset,
    geometry: StaticGeometry,
    targets: Vec<ScenarioTarget>,
    visual: Option<VisualInstance>,
}
impl Scenario {
    /// Load one preset from an already verified native 2027 package. Source CAD
    /// selection/placement is unchanged; TunnelFixture needs no package geometry.
    pub fn load(cad: &CadAssets, preset: Preset) -> anyhow::Result<Self> {
        if preset == Preset::TunnelFixture {
            return Self::tunnel(TunnelModel {
                pose: Pose::at([0.; 3]),
                opening_m: [2., 1., 0.25],
                wall_thickness_m: 0.1,
            });
        }
        let bindings = cad
            .rmuc_2027
            .as_ref()
            .context("source preset requires native 2027 bindings")?;
        let world = &bindings.world;
        let (asset, role, side) = match preset {
            Preset::Outpost(team) => (&cad.outpost, ComponentRole::Outpost, Some(team)),
            Preset::Base(team) => (&cad.base, ComponentRole::Equipment, Some(team)),
            Preset::Rune => (&cad.rune, ComponentRole::Rune, None),
            Preset::TunnelFixture => unreachable!(),
        };
        let placement = asset
            .placements
            .iter()
            .find(|p| side.is_none_or(|team| (p.translation_m[0] >= 0.) == (team == Team::Red)))
            .context("preset has no source placement for the requested side")?;
        let model = components::load(cad, asset, role, Some(bindings.motions(cad, asset)))?;
        let mut physics = WorldPhysics::new(&[], 0.);
        model
            .placed(placement)
            .install(&mut physics)
            .map_err(anyhow::Error::msg)?;
        let mut targets = Vec::new();
        match preset {
            Preset::Outpost(team) => {
                let source = &world.outposts[team.index()];
                source.validate().map_err(anyhow::Error::msg)?;
                for (face, d) in source.detectors.iter().enumerate() {
                    targets.push(ScenarioTarget {
                        face: TargetFace {
                            target: ArmorTarget::Outpost {
                                outpost: team.index() as u32,
                                face: face as u32,
                            },
                            pose: d.pose,
                        },
                        detector: Some(d.clone()),
                        rotation: Some(TargetRotation {
                            mechanism: Mechanism::Outpost,
                            team,
                            pivot_m: source.pivot_m,
                            axis: source.axis,
                        }),
                    });
                }
            }
            Preset::Base(team) => {
                for (plate, d) in world.bases[team.index()].iter().enumerate() {
                    targets.push(ScenarioTarget {
                        face: TargetFace {
                            target: ArmorTarget::Base {
                                base: team.index() as u32,
                                plate: plate as u32,
                            },
                            pose: d.pose,
                        },
                        detector: Some(d.clone()),
                        rotation: None,
                    });
                }
            }
            Preset::Rune => {
                ensure!(
                    world.runes.len() == 2,
                    "Rune preset requires both bound target banks"
                );
                for team in Team::BOTH {
                    let source = &world.runes[team.index()];
                    for (blade, pose) in source.targets.iter().enumerate() {
                        targets.push(ScenarioTarget {
                            face: TargetFace {
                                target: ArmorTarget::Rune {
                                    rune: team.index() as u32,
                                    blade: blade as u32,
                                },
                                pose: *pose,
                            },
                            detector: None,
                            rotation: Some(TargetRotation {
                                mechanism: Mechanism::Rune,
                                team,
                                pivot_m: source.pivot_m,
                                axis: source.axis,
                            }),
                        });
                    }
                }
            }
            Preset::TunnelFixture => unreachable!(),
        }
        Ok(Self {
            preset,
            geometry: physics.static_geometry_snapshot(true),
            targets,
            visual: Some(VisualInstance {
                file: cad.root.join(&asset.file),
                placement: *placement,
            }),
        })
    }

    /// Build a caller-dimensioned approximate passage, without CAD or a match.
    ///
    /// ```
    /// use rm_simulator_assets::scenarios::Scenario;
    /// use rm_simulator_physics::{Pose, StepDuration, arena::TunnelModel};
    /// let definition = Scenario::tunnel(TunnelModel { pose: Pose::at([0.; 3]),
    ///     opening_m: [2., 1., 0.25], wall_thickness_m: 0.1 })?;
    /// let mut session = definition.session(StepDuration::HZ_1000, vec![], Default::default())?;
    /// session.step(100).map_err(anyhow::Error::msg)?;
    /// session.reset().map_err(anyhow::Error::msg)?;
    /// assert_eq!(session.stamp().time_ns, 0);
    /// # Ok::<(), anyhow::Error>(())
    /// ```
    pub fn tunnel(model: TunnelModel) -> anyhow::Result<Self> {
        let mut physics = WorldPhysics::new(&[], 0.);
        model.install(&mut physics).map_err(anyhow::Error::msg)?;
        Ok(Self {
            preset: Preset::TunnelFixture,
            geometry: physics.static_geometry_snapshot(true),
            targets: vec![],
            visual: None,
        })
    }
    /// Preset identity; custom tunnel dimensions remain in the cooked geometry.
    pub fn preset(&self) -> Preset {
        self.preset
    }
    /// Source detector definitions in stable physical identity order.
    pub fn targets(&self) -> &[ScenarioTarget] {
        &self.targets
    }
    /// Source visual reference, absent for the synthetic tunnel fixture.
    pub fn visual(&self) -> Option<&VisualInstance> {
        self.visual.as_ref()
    }
    /// Rest-pose shared collision geometry, excluding the implicit catch plane.
    pub fn geometry(&self) -> &StaticGeometry {
        &self.geometry
    }

    /// Resolve all target endpoints with the same coordinates used for scenery.
    /// All angle inputs must be finite; normalized slide coordinates must lie in
    /// [0,1]. Validation precedes mutation of the reusable output vector.
    pub fn targets_at(
        &self,
        state: &MechanismState,
        output: &mut Vec<TargetFace>,
    ) -> anyhow::Result<()> {
        resolve_targets(&self.targets, state, output)
    }
    /// Create a fresh bare physical session with explicit timing and robot spawns.
    /// Catch ground is flat at z=0 for these isolated presets. Recipe reset retains
    /// initial coordinates and robots, sharing cooked shape buffers. No rules or
    /// automatic motion are installed; update scenery and target endpoints together
    /// through ordinary RobotSession APIs when changing mechanism coordinates.
    pub fn session(
        &self,
        duration: StepDuration,
        robots: Vec<RobotSpawn>,
        state: MechanismState,
    ) -> anyhow::Result<RobotSession> {
        let mut targets = Vec::new();
        self.targets_at(&state, &mut targets)?;
        let geometry = self.geometry.clone();
        let recipe = RobotScenario::new(targets, 0., duration, move |physics| {
            physics.insert_geometry(&geometry);
            physics.sync_mechanisms(&state);
            for robot in &robots {
                physics.add_chassis(robot.team, robot.config.clone(), robot.pose)?;
            }
            Ok(())
        })
        .map_err(anyhow::Error::msg)?;
        RobotSession::new(recipe).map_err(anyhow::Error::msg)
    }
}
fn resolve_targets(
    targets: &[ScenarioTarget],
    state: &MechanismState,
    output: &mut Vec<TargetFace>,
) -> anyhow::Result<()> {
    validate_state(state)?;
    output.clear();
    output.extend(targets.iter().map(|target| {
        let mut face = target.face;
        if let Some(rotation) = target.rotation {
            face.pose = OutpostModel::rotate_pose(
                face.pose,
                rotation.pivot_m,
                rotation.axis,
                state.fraction(rotation.mechanism, rotation.team),
            );
        }
        face
    }));
    Ok(())
}
fn validate_state(state: &MechanismState) -> anyhow::Result<()> {
    ensure!(
        state
            .outpost_angle_rad
            .iter()
            .chain(&state.rune_angle_rad)
            .all(|v| v.is_finite())
            && state
                .base_open_fraction
                .iter()
                .chain(std::iter::once(&state.dart_target_fraction))
                .all(|v| v.is_finite() && (0. ..=1.).contains(v)),
        "invalid scenario mechanism coordinates"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn tunnel() -> Scenario {
        Scenario::tunnel(TunnelModel {
            pose: Pose::at([0.; 3]),
            opening_m: [2., 1., 0.65],
            wall_thickness_m: 0.1,
        })
        .unwrap()
    }
    #[test]
    fn independent_sessions_and_recipe_reset_share_geometry_but_not_robot_state() {
        let scenario = tunnel();
        let spawns = vec![RobotSpawn {
            team: Team::Red,
            config: ChassisConfig::default(),
            pose: Pose::at([-2., 0., 0.4]),
        }];
        let mut a = scenario
            .session(StepDuration::HZ_1000, spawns.clone(), Default::default())
            .unwrap();
        let b = scenario
            .session(StepDuration::HZ_1000, spawns, Default::default())
            .unwrap();
        a.command(rm_simulator_physics::session::RobotCommand::Drive {
            robot: 0,
            command: rm_simulator_physics::chassis::ChassisCommand {
                forward_m_s: 1.,
                ..Default::default()
            },
        })
        .unwrap();
        a.step(100).unwrap();
        assert_eq!(b.stamp().time_ns, 0);
        assert_ne!(
            a.robot_truth(0).unwrap().state.pose,
            b.robot_truth(0).unwrap().state.pose
        );
        a.reset().unwrap();
        assert_eq!(a.robot_truth(0).unwrap(), b.robot_truth(0).unwrap());
        assert!(
            a.ground_truth()
                .static_geometry_snapshot(true)
                .segment_clear([-2., 0., 0.2], [2., 0., 0.2])
        );
        assert!(
            !a.ground_truth()
                .static_geometry_snapshot(true)
                .segment_clear([0., 0., 0.2], [0., 0., 1.])
        );
    }
    #[test]
    fn bad_coordinates_do_not_modify_output_or_create_a_session() {
        let scenario = tunnel();
        let target = TargetFace {
            target: ArmorTarget::Base { base: 0, plate: 0 },
            pose: Pose::at([0.; 3]),
        };
        let mut output = vec![target];
        let state = MechanismState {
            rune_angle_rad: [f64::NAN, 0.],
            ..Default::default()
        };
        assert!(scenario.targets_at(&state, &mut output).is_err());
        assert_eq!(output, [target]);
        assert!(
            scenario
                .session(StepDuration::HZ_1000, vec![], state)
                .is_err()
        );
    }
    #[test]
    fn target_rotation_uses_the_same_coordinates_as_collision_channels() {
        let mut scenario = tunnel();
        scenario.targets.push(ScenarioTarget {
            face: TargetFace {
                target: ArmorTarget::Outpost {
                    outpost: 1,
                    face: 0,
                },
                pose: Pose::at([2., 0., 1.]),
            },
            detector: None,
            rotation: Some(TargetRotation {
                mechanism: Mechanism::Outpost,
                team: Team::Blue,
                pivot_m: [1., 0., 1.],
                axis: [0., 0., 1.],
            }),
        });
        let state = MechanismState {
            outpost_angle_rad: [0., std::f64::consts::FRAC_PI_2],
            ..Default::default()
        };
        let mut output = Vec::new();
        scenario.targets_at(&state, &mut output).unwrap();
        let pose = output[0].pose;
        assert!((pose.translation_m[0] - 1.).abs() < 1e-12);
        assert!((pose.translation_m[1] - 1.).abs() < 1e-12);
        assert_eq!(output[0].target, scenario.targets[0].face.target);
    }
}
