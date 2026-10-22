// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! World snapshots into the renderer's `SceneState`: the static CAD
//! instances, the rune and outpost light overlays, projectiles, hit
//! flashes and the chassis visuals. The renderer is passive; this is the
//! only place that decides what it shows.
use bevy::math::{DQuat, DVec3};
use bevy::prelude::*;
use rm_simulator_render::{
    PoseFlu, RenderingConfig, TeamColor,
    armor::{ArmorAtlas, ArmorPattern, DiffuserProfile},
    cad::{CadInstance, CadJoint, CadRole, CadRuneFace, CadTargetLight, JointDriver},
    camera_appearance,
    chassis::ArmorOptics,
    lighting, outpost, rune,
    sync::{
        ArmorAppearance, ChassisAppearance, OutpostAppearance, ProjectileAppearance,
        RuneAppearance, SceneInput, SceneState, SourceTime, WheelAppearance,
    },
};
use rm_simulator_server::{
    cad_assets::CadAssets,
    layout::{outpost_team, rune_team},
};
use rm_simulator_world::{
    ArmorHit, ArmorTarget, ChassisSnapshot, FieldSnapshot, RefereeSnapshot, RuneKind, RuneState,
    SMALL_ARMOR_HOUSING_HALF_M, Team, outpost as outpost_rules,
};
use std::collections::HashMap;

use crate::controls::{Drive, PlayerCamera};
use crate::frames::{dquat, gimbal_poses, pose_flu, transform_of, wxyz};

use crate::session::{FlashSettings, Session};

/// Axis-aligned arena footprint in world FLU metres: the low corner then the
/// high corner. It bounds the floor and the arena-spanning scenery, and
/// `setup_stadium` sizes the venue from it.
#[derive(Resource)]
pub struct ArenaBounds(pub [[f64; 3]; 2]);

/// Rendering profile the camera, scenery and rule overlays are drawn with.
/// The app inserts the lit field profile at startup and keeps it across joins.
#[derive(Resource)]
pub struct Appearance(pub RenderingConfig);
/// Build the ordered CAD instances of a whole field: the floor and arena shell,
/// the single rune, every outpost, base and tech core, then each extra static
/// asset. The floor and arena shell take a grey scenery override; base and tech
/// core placements take the side's team colour.
///
/// `rune_faces` registers the rune asset's named face nodes so the rune angle
/// turns them; false leaves the CAD rune static, as `--no-rune` does.
/// `base_plates` hides the base asset's imported armor-plate and unclassified
/// meshes, so pass true when live bases draw those plates from rule state.
///
/// An asset that exports visual semantics gets a `CadRole::Semantic` role
/// instead, binding its joints to the rune angle, the outpost rotor and the
/// base gate, dart target and dart door mechanisms. Instance order is the
/// renderer's spawn order and must stay stable across joins.
pub fn cad_instances(cad: &CadAssets, rune_faces: bool, base_plates: bool) -> Vec<CadInstance> {
    if cad.floor.file.is_empty() {
        return Vec::new();
    }
    if cad.rest_pose_only {
        let arena = transform_of(&cad.arena_pose());
        let mut instances = Vec::new();
        for asset in [
            &cad.floor,
            &cad.arena_static,
            &cad.rune,
            &cad.outpost,
            &cad.base,
            &cad.tech_core,
        ]
        .into_iter()
        .chain(&cad.static_assets)
        {
            let transforms = if asset.placements.is_empty() {
                vec![arena]
            } else {
                asset.placements.iter().map(transform_of).collect()
            };
            instances.extend(transforms.into_iter().map(|transform| {
                let side =
                    rm_simulator_server::layout::side_team(-f64::from(transform.translation.z))
                        .index();
                let joints = cad
                    .rmuc_2027
                    .as_ref()
                    .map(|b| {
                        b.motions(cad, asset)
                            .iter()
                            .enumerate()
                            .map(|(mi, m)| rm_simulator_render::cad::CadJoint {
                                id: m.id.clone(),
                                axis: Vec3::from_array(m.axis.map(|v| v as f32)),
                                prismatic: m.limits.is_some(),
                                limits: m.limits.map(|v| v.map(|v| v as f32)),
                                driver: Some(if asset.file == cad.rune.file {
                                    JointDriver::Rune {
                                        index: mi as u32,
                                        sign: 1.,
                                    }
                                } else if m.limits.is_some() {
                                    rm_simulator_render::cad::JointDriver::BaseOpening {
                                        team: side,
                                    }
                                } else {
                                    rm_simulator_render::cad::JointDriver::Outpost {
                                        index: side as u32,
                                    }
                                }),
                            })
                            .collect::<Vec<_>>()
                    })
                    .unwrap_or_default();
                let target_lights = cad
                    .rmuc_2027
                    .as_ref()
                    .and_then(|b| {
                        if asset.file == cad.rune.file {
                            return Some(
                                b.rune_arms
                                    .iter()
                                    .enumerate()
                                    .flat_map(|(i, arms)| {
                                        arms.iter().enumerate().map(move |(blade, node)| {
                                            CadTargetLight {
                                                node: node.clone(),
                                                target: (512 + i * 5 + blade) as u64,
                                                source_color: if i == 0 {
                                                    Color::srgb(1., 0., 0.)
                                                } else {
                                                    Color::srgb(0., 0., 1.)
                                                },
                                            }
                                        })
                                    })
                                    .collect::<Vec<_>>(),
                            );
                        }
                        let (name, offset, source_color) = if asset.file == cad.base.file {
                            ("base", 0, Color::srgb(1., 0., 0.))
                        } else if asset.file == cad.outpost.file {
                            ("outpost", 128, Color::srgb(0., 1., 1.))
                        } else {
                            return None;
                        };
                        b.report.get(name).map(|report| {
                            report
                                .detector_bodies
                                .iter()
                                .enumerate()
                                .map(|(index, body)| CadTargetLight {
                                    node: format!("source_body_{body}"),
                                    target: offset + (side * 32 + index) as u64,
                                    source_color,
                                })
                                .collect::<Vec<_>>()
                        })
                    })
                    .unwrap_or_default();
                CadInstance {
                    team: (!target_lights.is_empty()).then(|| team_color(Team::BOTH[side])),
                    file: asset.file.clone(),
                    transform,
                    role: if joints.is_empty() {
                        CadRole::Static { base_color: None }
                    } else {
                        CadRole::Semantic {
                            scene: 0,
                            joints,
                            hidden_ids: vec![],
                            rune_faces: vec![],
                            target_lights,
                        }
                    },
                }
            }));
        }
        return instances;
    }

    let arena = transform_of(&cad.arena_pose());
    let mut instances = vec![
        CadInstance {
            team: None,
            file: cad.floor.file.clone(),
            transform: arena,
            role: CadRole::Static {
                base_color: Some(Color::srgb(0.24, 0.25, 0.27)),
            },
        },
        CadInstance {
            team: None,
            file: cad.arena_static.file.clone(),
            transform: arena,
            role: CadRole::Static {
                base_color: Some(Color::srgb(0.60, 0.61, 0.63)),
            },
        },
        CadInstance {
            team: None,
            file: cad.rune.file.clone(),
            transform: transform_of(&cad.rune.placements[0]),
            role: CadRole::Rune {
                faces: if rune_faces {
                    (0..2)
                        .map(|face| CadRuneFace {
                            node: format!("face_{face}"),
                            rune: face as u32,
                            color: team_color(rune_team(
                                &rm_simulator_server::layout::cad_rune_hub(
                                    &cad.rune.placements[0],
                                    face,
                                ),
                            )),
                            spin: if face == 0 { -1.0 } else { 1.0 },
                        })
                        .collect()
                } else {
                    Vec::new()
                },
            },
        },
    ];
    for (index, placement) in cad.outpost.placements.iter().enumerate() {
        instances.push(CadInstance {
            team: Some(team_color(outpost_team(placement))),
            file: cad.outpost.file.clone(),
            transform: transform_of(placement),
            role: CadRole::Outpost {
                outpost: index as u32,
            },
        });
    }
    for asset in [&cad.base, &cad.tech_core]
        .into_iter()
        .chain(&cad.static_assets)
    {
        for placement in &asset.placements {
            instances.push(CadInstance {
                team: (asset.file == cad.base.file || asset.file == cad.tech_core.file).then(
                    || {
                        team_color(rm_simulator_server::layout::side_team(
                            placement.translation_m[0],
                        ))
                    },
                ),
                file: asset.file.clone(),
                transform: transform_of(placement),
                role: CadRole::Static { base_color: None },
            });
        }
    }
    for instance in &mut instances {
        let asset = [
            &cad.floor,
            &cad.arena_static,
            &cad.rune,
            &cad.outpost,
            &cad.base,
            &cad.tech_core,
        ]
        .into_iter()
        .chain(&cad.static_assets)
        .find(|asset| asset.file == instance.file)
        .expect("CAD instance asset");
        if let Some(semantics) = asset.visual_semantics() {
            let joints = semantics
                .joints
                .iter()
                .map(|joint| {
                    let driver = match &instance.role {
                        CadRole::Rune { faces } => faces
                            .iter()
                            .find(|face| joint.id == format!("rune.face.{}.spin", face.rune))
                            .map(|face| JointDriver::Rune {
                                index: face.rune,
                                sign: face.spin,
                            }),
                        CadRole::Outpost { outpost } if joint.id == "outpost.spin" => {
                            Some(JointDriver::Outpost { index: *outpost })
                        }
                        _ => {
                            let team = rm_simulator_server::layout::side_team(-f64::from(
                                instance.transform.translation.z,
                            ))
                            .index();
                            rm_simulator_server::layout::mechanism_joint(joint).map(|kind| {
                                match kind {
                                    rm_simulator_world::referee::Mechanism::Rune => {
                                        JointDriver::Rune {
                                            index: team as u32,
                                            sign: 1.,
                                        }
                                    }
                                    rm_simulator_world::referee::Mechanism::Outpost => {
                                        JointDriver::Outpost { index: team as u32 }
                                    }
                                    rm_simulator_world::referee::Mechanism::Base => {
                                        JointDriver::BaseOpening { team }
                                    }
                                    rm_simulator_world::referee::Mechanism::DartTarget => {
                                        JointDriver::DartTarget
                                    }
                                    rm_simulator_world::referee::Mechanism::DartDoor => {
                                        JointDriver::DartDoor { team }
                                    }
                                }
                            })
                        }
                    };
                    CadJoint {
                        id: joint.id.clone(),
                        axis: Vec3::from_array(joint.axis.map(|v| v as f32)),
                        prismatic: joint.kind == "prismatic",
                        limits: joint.limits.map(|v| v.map(|x| x as f32)),
                        driver,
                    }
                })
                .collect();
            // Optical dimensions are not exported yet. Keep the existing rule
            // overlay, replacing only explicitly classified outpost armor modules.
            let mut hidden_ids: Vec<String> = if matches!(instance.role, CadRole::Outpost { .. }) {
                semantics
                    .nodes
                    .iter()
                    .filter(|node| {
                        node.metadata["roles"]
                            .as_array()
                            .is_some_and(|roles| roles.iter().any(|role| role == "armor_module"))
                    })
                    .map(|node| node.id.clone())
                    .collect()
            } else {
                Vec::new()
            };
            if base_plates && asset.file == cad.base.file {
                hidden_ids.extend((0..3).map(|i| format!("base.reconstructed.armor.{i}")));
                hidden_ids.extend((21..=29).map(|i| format!("base.unclassified.{i}")));
            }
            let rune_faces = match &instance.role {
                CadRole::Rune { faces } => faces.clone(),
                _ => Vec::new(),
            };
            instance.role = CadRole::Semantic {
                scene: semantics.scene,
                joints,
                hidden_ids,
                rune_faces,
                target_lights: vec![],
            };
        }
    }
    instances
}

/// Keep one camera through loading and gameplay so its render target stays stable.
pub fn setup_camera(
    mut commands: Commands,
    appearance: Res<Appearance>,
    mut images: ResMut<Assets<Image>>,
) {
    let rendering = appearance.0;
    let mut camera = commands.spawn((
        PlayerCamera,
        Camera3d::default(),
        lighting::skybox(&mut images),
        IsDefaultUiCamera,
        Projection::Perspective(PerspectiveProjection {
            fov: 70_f32.to_radians(),
            near: 0.01,
            far: 300.0,
            ..default()
        }),
        Transform::from_xyz(0.0, 1.0, 0.0),
    ));
    camera_appearance(&mut camera, rendering);
    lighting::spawn_lighting(&mut commands);
}

/// Practice floor and approximate RFID markings owned by one joined session.
#[derive(Component)]
pub(crate) struct PracticeFloor;

/// Spawn the venue stadium around the arena footprint, once. The two
/// `ArenaBounds` corners are world FLU metres and convert to Bevy's
/// right/up/back frame here, at the renderer boundary. A later join finds the
/// stadium already present and spawns nothing.
pub fn setup_stadium(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    bounds: Res<ArenaBounds>,
    stadium: Query<Entity, With<lighting::Stadium>>,
) {
    if stadium.is_empty() {
        let [low, high] = bounds.0;
        lighting::spawn_stadium(
            &mut commands,
            &mut meshes,
            &mut materials,
            rm_simulator_render::flu_position(low),
            rm_simulator_render::flu_position(high),
        );
    }
}

/// Spawn the rune and outpost overlays, the crosshair
/// and the HUD text.
#[allow(clippy::too_many_arguments)]
pub fn setup(
    mut commands: Commands,
    atlas: Res<ArmorAtlas>,
    diffuser: Res<DiffuserProfile>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    appearance: Res<Appearance>,
    config: Option<Res<crate::staging::StagingConfig>>,
    session: Res<Session>,
) {
    let rendering = appearance.0;
    let snapshot = &session.snapshot;
    let practice_competition = snapshot
        .rmul_2027
        .as_ref()
        .map(|game| {
            if game.format == rm_simulator_world::gameplay::rmul_2027::Format::Infantry {
                rm_simulator_server::host_args::Competition::RmulInfantry2027
            } else {
                rm_simulator_server::host_args::Competition::Rmul2027
            }
        })
        .or_else(|| {
            config
                .as_ref()
                .map(|c| c.0.host.competition)
                .filter(|c| c.rmul_format().is_some())
        });
    if let Some(competition) = practice_competition {
        let [_, high] = competition.practice_bounds();
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(
                (2.0 * high[1]) as f32,
                0.04,
                (2.0 * high[0]) as f32,
            ))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.24, 0.27, 0.28),
                ..default()
            })),
            Transform::from_xyz(0.0, -0.02, 0.0),
            PracticeFloor,
        ));
        for (x, y, hx, hy) in [
            (high[0], 0.0, 0.025, high[1]),
            (-high[0], 0.0, 0.025, high[1]),
            (0.0, high[1], high[0], 0.025),
            (0.0, -high[1], high[0], 0.025),
        ] {
            commands.spawn((
                Mesh3d(meshes.add(Cuboid::new((2.0 * hy) as f32, 1.0, (2.0 * hx) as f32))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: Color::srgb(0.12, 0.14, 0.16),
                    ..default()
                })),
                Transform::from_translation(rm_simulator_render::flu_position([x, y, 0.5])),
                PracticeFloor,
            ));
        }
        if competition == rm_simulator_server::host_args::Competition::Rmul2027 {
            for (x, y, hx, hy, color) in [
                (4.75, 2.75, 1.1, 1.0, Color::srgba(0.7, 0.12, 0.12, 0.8)),
                (-4.75, -2.75, 1.1, 1.0, Color::srgba(0.12, 0.2, 0.7, 0.8)),
                (2.3, 0.8, 0.65, 0.65, Color::srgba(0.7, 0.32, 0.12, 0.8)),
                (-2.3, -0.8, 0.65, 0.65, Color::srgba(0.12, 0.5, 0.7, 0.8)),
                (0.0, 0.0, 1.3, 1.3, Color::srgba(0.65, 0.55, 0.16, 0.8)),
            ] {
                commands.spawn((
                    Mesh3d(meshes.add(Cuboid::new(2.0 * hy, 0.002, 2.0 * hx))),
                    MeshMaterial3d(materials.add(StandardMaterial {
                        base_color: color,
                        ..default()
                    })),
                    Transform::from_translation(rm_simulator_render::flu_position([x, y, 0.002])),
                    PracticeFloor,
                ));
            }
        }
    }
    for (index, rune) in snapshot.runes.iter().enumerate() {
        let team = referee_team(snapshot, |r| &r.rune_teams, index)
            .unwrap_or_else(|| rune_team(&rune.hub_pose));
        rune::spawn_rune(
            &mut commands,
            &mut meshes,
            &mut materials,
            index as u32,
            rendering,
            team_color(team),
        );
    }
    if !snapshot.outposts.is_empty() || !snapshot.bases.is_empty() {
        let optics = outpost::OutpostOptics {
            emission_profile: diffuser.0.clone(),
            boxes: outpost_rules::HOUSING_BOXES
                .iter()
                .map(|(p, s)| (p.map(|x| x as f32), s.map(|x| x as f32)))
                .collect(),
            face_size_m: [
                outpost_rules::FACE_WIDTH_M as f32,
                outpost_rules::FACE_HEIGHT_M as f32,
            ],
            light_span_m: outpost_rules::LIGHT_SPAN_M as f32,
            light_length_m: outpost_rules::LIGHT_LENGTH_M as f32,
            rendering,
        };
        for (id, base) in snapshot.bases.iter().enumerate() {
            outpost::spawn_base(
                &mut commands,
                &mut meshes,
                &mut materials,
                id as u32,
                &optics,
                &atlas,
                team_color(base.config.team),
            );
        }
        for (id, outpost) in snapshot.outposts.iter().enumerate() {
            let team = referee_team(snapshot, |r| &r.outpost_teams, id)
                .unwrap_or_else(|| outpost_team(&outpost.origin));
            outpost::spawn(
                &mut commands,
                &mut meshes,
                &mut materials,
                id as u32,
                &optics,
                &atlas,
                team_color(team),
            );
        }
    }
    crate::hud::spawn_hud(&mut commands);
}

/// Wheel visual pose: hub in the world, local +y along the axle, spun about it.
fn wheel_pose(
    chassis: &ChassisSnapshot,
    hub_m: [f64; 2],
    wheel: &rm_simulator_world::WheelSnapshot,
) -> PoseFlu {
    let axle_yaw = if chassis.config.mecanum {
        0.0
    } else {
        hub_m[1].atan2(hub_m[0]) - std::f64::consts::FRAC_PI_2
    };
    let rotation = dquat(chassis.pose.rotation_wxyz)
        * DQuat::from_rotation_z(axle_yaw)
        * DQuat::from_rotation_y(-wheel.spin_rad);
    let centre = wheel.hub_m;
    PoseFlu {
        translation_m: centre,
        rotation_wxyz: wxyz(rotation.normalize()),
    }
}
/// The small armor module geometry the renderer draws on every chassis,
/// from the rules: the same module as the outpost's middle armor.
pub fn armor_optics() -> ArmorOptics {
    ArmorOptics {
        housing_half_m: SMALL_ARMOR_HOUSING_HALF_M.map(|v| v as f32),
        face_size_m: [
            outpost_rules::FACE_WIDTH_M as f32,
            outpost_rules::FACE_HEIGHT_M as f32,
        ],
        light_span_m: outpost_rules::LIGHT_SPAN_M as f32,
        light_length_m: outpost_rules::LIGHT_LENGTH_M as f32,
    }
}
/// The number painted on a chassis: the pilot's robot when the roster names
/// it, else 1 for a mecanum Hero and 3 for any other (a bot, or a chassis the
/// roster has not described yet).
fn armor_pattern(
    chassis: &ChassisSnapshot,
    robot: Option<rm_simulator_server::protocol::Robot>,
) -> ArmorPattern {
    use rm_simulator_server::protocol::Robot;
    match robot {
        Some(Robot::Hero) => ArmorPattern::One,
        Some(Robot::Engineer) if chassis.config.folding.is_some() => ArmorPattern::One,
        Some(Robot::Engineer) => ArmorPattern::Two,
        Some(Robot::Sentry) => ArmorPattern::GuardSmall,
        Some(Robot::Drone) => ArmorPattern::Three,
        Some(Robot::Infantry3) => ArmorPattern::Three,
        Some(Robot::Infantry4) => ArmorPattern::Four,
        Some(Robot::Infantry5) => ArmorPattern::Five,
        None if chassis.config.mecanum => ArmorPattern::One,
        None => ArmorPattern::Three,
    }
}
/// One chassis for the renderer, using the actual host or predicted motor pose. `flash`
/// lists the armor plates flashing after a strike; `robot` is what the roster
/// says its pilot drives.
pub(crate) fn chassis_appearance(
    chassis: &ChassisSnapshot,
    flash: &[bool],
    robot: Option<rm_simulator_server::protocol::Robot>,
) -> ChassisAppearance {
    let config = &chassis.config;
    let aim = pose_flu(chassis.turret);
    let (yaw_stage, turret) = gimbal_poses(pose_flu(chassis.pose), aim);
    ChassisAppearance {
        source_joints: if let Some(a) = &config.articulation {
            let q = dquat(chassis.pose.rotation_wxyz);
            let aimq = dquat(chassis.turret.rotation_wxyz);
            let d = q.inverse() * (aimq * DVec3::X);
            a.joints
                .iter()
                .zip(a.poses(
                    chassis.fold_angle_rad,
                    [d.y.atan2(d.x), d.z.clamp(-1., 1.).asin()],
                ))
                .map(|(j, p)| (j.name.clone(), pose_flu(p)))
                .collect()
        } else {
            config
                .climbing
                .as_ref()
                .map(|c| {
                    let p = c.poses(chassis.fold_angle_rad);
                    [
                        ("front", p.front),
                        ("coupler", p.coupler),
                        ("rear", p.rear),
                        ("guide", p.guide),
                        ("slider", p.slider),
                    ]
                    .into_iter()
                    .map(|(name, p)| (name.to_string(), pose_flu(p)))
                    .collect()
                })
                .unwrap_or_default()
        },
        fold_angle_rad: chassis.fold_angle_rad as f32,
        fold_collision_parts: {
            let mut parts = config.transform_collision_parts(
                chassis.pose,
                chassis.held_aim_rad,
                chassis.fold_angle_rad,
            );
            if config.climbing.is_some() {
                // Include the fixed upper proxy and rear shelf in debug/fallback drawings.
                let q = dquat(chassis.pose.rotation_wxyz);
                for (center, half) in [
                    (config.turret_center_m, config.turret_half_m),
                    ([-0.075, 0., -0.16], [0.05, 0.26, 0.006]),
                ] {
                    let p = DVec3::from_array(chassis.pose.translation_m)
                        + q * DVec3::from_array(center);
                    parts.push((
                        rm_simulator_world::Pose {
                            translation_m: p.to_array(),
                            rotation_wxyz: chassis.pose.rotation_wxyz,
                        },
                        half,
                    ));
                }
            }
            parts
                .into_iter()
                .map(|(p, h)| (pose_flu(p), h.map(|v| v as f32)))
                .collect()
        },
        model: {
            use rm_simulator_render::chassis::RobotModel;
            use rm_simulator_server::protocol::Robot;
            if config.articulation.is_some() {
                RobotModel::Articulated
            } else if config.climbing.is_some() {
                RobotModel::StairHeavy
            } else if config.folding.is_some() {
                RobotModel::FoldingHero
            } else if config.team_cad {
                RobotModel::TeamCad
            } else {
                match robot {
                    Some(Robot::Engineer) => RobotModel::Engineer,
                    Some(Robot::Drone) => RobotModel::Drone,
                    Some(Robot::Sentry) => RobotModel::Sentry,
                    Some(Robot::Hero) => RobotModel::Hero,
                    _ if config.balance_assist => RobotModel::Balance,
                    None if config.mecanum => RobotModel::Hero,
                    _ => RobotModel::Omni,
                }
            }
        },
        armor_pattern: armor_pattern(chassis, robot),
        mecanum: config.mecanum,
        hp_fraction: if chassis.defeated { 0.0 } else { 1.0 },
        id: chassis.id,
        team: team_color(chassis.team),
        pose: pose_flu(chassis.pose),
        body_half_m: config.body_half_m.map(|v| v as f32),
        body_shell_visible: true,
        frame_beam_step_m: config.frame_beam_step_m() as f32,
        body_outline_m: config
            .body_outline_m()
            .iter()
            .map(|p| p.map(|v| v as f32))
            .collect(),
        bumper_base_prisms: if config.omni_frame.is_some() {
            config.body_outline_m().len()
        } else {
            0
        },
        bumper_prisms_m: config
            .bumper_prisms_m()
            .iter()
            .map(|p| p.map(|p| p.map(|v| v as f32)))
            .collect(),
        wheels: chassis
            .wheels
            .iter()
            .zip(&config.wheel_hubs_m)
            .enumerate()
            .map(|(index, (wheel, hub))| WheelAppearance {
                pose: wheel_pose(chassis, *hub, wheel),
                suspension_anchor_m: {
                    let mounts = config
                        .suspension_mounts_at_m(
                            index,
                            wheel.suspension_compression_m,
                            chassis.fold_angle_rad,
                        )
                        .unwrap();
                    (DVec3::from_array(chassis.pose.translation_m)
                        + dquat(chassis.pose.rotation_wxyz) * DVec3::from_array(mounts[0]))
                    .to_array()
                },
                suspension_lower_m: {
                    let mounts = config
                        .suspension_mounts_at_m(
                            index,
                            wheel.suspension_compression_m,
                            chassis.fold_angle_rad,
                        )
                        .unwrap();
                    (DVec3::from_array(chassis.pose.translation_m)
                        + dquat(chassis.pose.rotation_wxyz) * DVec3::from_array(mounts[1]))
                    .to_array()
                },
                radius_m: config.wheel_radius_m as f32,
                width_m: config.wheel_width_m as f32,
            })
            .collect(),
        armor: config
            .armor_faces()
            .iter()
            .enumerate()
            .map(|(plate, face)| ArmorAppearance {
                optics: Some(ArmorOptics {
                    housing_half_m: config.armor_housing_half_m().map(|v| v as f32),
                    face_size_m: [
                        if config.folding.is_some() {
                            0.223
                        } else {
                            0.128
                        },
                        0.113,
                    ],
                    light_span_m: if config.folding.is_some() {
                        0.225
                    } else {
                        0.130
                    },
                    light_length_m: 0.056,
                }),
                plate_visible: true,
                bracket_prisms_m: config
                    .armor_bracket_prisms_m(plate)
                    .iter()
                    .map(|p| p.map(|p| p.map(|v| v as f32)))
                    .collect(),
                local: pose_flu(*face),
                hit_flash: flash.get(plate).copied().unwrap_or(false),
            })
            .collect(),
        yaw_stage,
        turret,
        turret_half_m: config.turret_half_m.map(|v| v as f32),
        pivot_above_body_m: (config.turret_center_m[2] - config.body_half_m[2]) as f32,
        barrel_length_m: config.muzzle_offset_m()[0] as f32,
        defeated: chassis.defeated,
    }
}

/// Fill the renderer's `SceneInput` from the session once a frame; the passive
/// renderer applies that caller-owned state unchanged.
///
/// The source is the session checkpoint plus locally predicted projectiles.
/// While the session is live and unpaused, rune blade,
/// outpost, base plate and dart target poses are evaluated at the time a shot
/// fired now would use, so the drawn scene matches the aim. A pending
/// screenshot or a pause uses the snapshot's own time instead.
///
/// The driven chassis uses its predicted pose while live; every other chassis
/// is sampled from the remote pose history at `remote_view_time_ns`. A pending
/// screenshot or a pause falls every chassis back to the snapshot. An HP
/// fraction from the referee's robot record overrides the default full bar, and
/// detected armor hits keep the grey strike flash for `hit_flash_ns`, an app
/// setting rather than a rule constant.
pub fn publish_scene(
    mut session: ResMut<Session>,
    drive: Option<Res<Drive>>,
    mut input: ResMut<SceneInput>,
    mut targets: ResMut<rm_simulator_render::targets::TargetInput>,
    capture: Option<Res<crate::screenshot::ScreenshotRequest>>,
) {
    if capture.is_none() {
        let now = session.time.now();
        let duration_ns = session.flash.hit_flash_ns;
        session.hit_feedback.present(now, duration_ns);
    }
    let snapshot = &session.snapshot;
    let live_presentation = capture.is_none();
    // Runes, outposts and the dart target move as functions of time, so they
    // are drawn where they will be when a shot taken now leaves the gun.
    let time_ns = if live_presentation && !session.paused {
        session.fire_time_ns()
    } else {
        snapshot.time_ns
    };
    let hits: Vec<_> = if live_presentation {
        session
            .hit_feedback
            .visible(session.time.now(), session.flash.hit_flash_ns)
            .collect()
    } else {
        flashing(snapshot, session.flash.hit_flash_ns).collect()
    };
    let mut scene = scene_state_with_hits(snapshot, session.flash, time_ns, &hits);
    scene.projectiles = session
        .visual_projectiles()
        .map(|projectile| ProjectileAppearance {
            position_m: projectile.position_m,
            radius_m: (projectile.caliber.diameter_m() / 2.0) as f32,
        })
        .collect();
    let own = drive.map(|drive| drive.chassis_id);
    let mut armor_flash: HashMap<u32, [bool; rm_simulator_world::chassis::ARMOR_COUNT]> =
        HashMap::new();
    for hit in &hits {
        if let ArmorTarget::Chassis { chassis, plate } = hit.target
            && let Some(flag) = armor_flash
                .entry(chassis)
                .or_default()
                .get_mut(plate as usize)
        {
            *flag = true;
        }
    }
    scene.chassis = snapshot
        .chassis
        .iter()
        .map(|chassis| {
            let mine = Some(chassis.id) == own;
            let flash = armor_flash.get(&chassis.id).map_or(&[][..], |f| &f[..]);
            let interpolated = if !mine && live_presentation && !session.paused {
                session
                    .remote_history
                    .sample(chassis.id, session.remote_view_time_ns)
            } else {
                None
            };
            let robot = session
                .roster
                .iter()
                .find(|p| p.chassis == Some(chassis.id))
                .and_then(|p| p.robot)
                .or_else(|| {
                    session
                        .navigation
                        .iter()
                        .any(|n| n.chassis == chassis.id)
                        .then_some(rm_simulator_server::protocol::Robot::Sentry)
                });
            let mut appearance = chassis_appearance(
                if mine && live_presentation {
                    session.presented_chassis().unwrap_or(chassis)
                } else {
                    interpolated.as_ref().unwrap_or(chassis)
                },
                flash,
                robot,
            );
            if let Some(robot) = snapshot
                .referee
                .as_ref()
                .and_then(|r| r.robots.iter().find(|r| r.id == chassis.id))
            {
                appearance.hp_fraction = robot.hp as f32 / robot.max_hp.max(1) as f32;
            }
            appearance
        })
        .collect();
    targets.0.clear();
    if let Some(state) = snapshot.rmuc_2027.as_ref() {
        let round_ns = state.game.elapsed_ns()
            + if !session.paused
                && state.game.phase == rm_simulator_world::rmuc_2027::match_rules::Phase::Running
            {
                time_ns.saturating_sub(snapshot.time_ns)
            } else {
                0
            };
        scene.base_open_fraction = state.game.base_open.map(|v| if v { 1. } else { 0. });
        scene.outposts = Team::BOTH
            .map(|side| {
                let tower = &state.config.outposts[side.index()];
                OutpostAppearance {
                    disabled: state.game.outpost_hp[side.index()] == 0,
                    origin: pose_flu(rm_simulator_world::Pose::at(tower.pivot_m)),
                    angle_rad: state.angle_at(side, round_ns),
                    armors: std::array::from_fn(|i| {
                        pose_flu(state.rotated_at(side, tower.detectors[i].pose, round_ns))
                    }),
                    hit_flash: [false; 3],
                }
            })
            .to_vec();
        scene.runes = state
            .config
            .runes
            .iter()
            .enumerate()
            .map(|(i, geometry)| {
                let r = &state.game.runes[i];
                RuneAppearance {
                    hub_pose: pose_flu(rm_simulator_world::Pose::at(geometry.pivot_m)),
                    angle_rad: r.angle_at(round_ns),
                    active_blades: r.targets,
                    activated_blades: r.completed_blades,
                    progress_stages: std::array::from_fn(|n| n < usize::from(r.groups)),
                    fully_activated: r.phase
                        == rm_simulator_world::gameplay::rmuc_2027::rune::Phase::Activated,
                    hit_flash: std::array::from_fn(|blade| {
                        hits.iter().any(|h| {
                            h.target
                                == ArmorTarget::Rune {
                                    rune: i as u32,
                                    blade: blade as u32,
                                }
                        })
                    }),
                }
            })
            .collect();
        for side in Team::BOTH {
            let i = side.index();
            if let Some(geometry) = state.config.runes.get(i) {
                let r = &state.game.runes[i];
                for (blade, rest) in geometry.targets.iter().enumerate() {
                    targets
                        .0
                        .push(rm_simulator_render::targets::TargetAppearance {
                            id: (512 + i * 5 + blade) as u64,
                            pose: pose_flu(state.rune_pose_at(side, *rest, round_ns)),
                            size_m: [0.3; 2],
                            team: team_color(side),
                            disabled: !r.targets[blade] && !r.completed_blades[blade],
                            flash: hits.iter().any(|h| {
                                h.target
                                    == ArmorTarget::Rune {
                                        rune: i as u32,
                                        blade: blade as u32,
                                    }
                            }),
                        });
                }
            }
            let owner = match state.game.control.snapshot().outposts[i].owner {
                rm_simulator_world::gameplay::Team::Red => Team::Red,
                rm_simulator_world::gameplay::Team::Blue => Team::Blue,
            };
            for (index, d) in state.config.bases[i].iter().enumerate() {
                let target = ArmorTarget::Base {
                    base: i as u32,
                    plate: index as u32,
                };
                targets
                    .0
                    .push(rm_simulator_render::targets::TargetAppearance {
                        id: (i * 32 + index) as u64,
                        pose: pose_flu(d.pose),
                        size_m: d.half_m.map(|v| (v * 2.) as f32),
                        team: team_color(side),
                        disabled: state.game.base_hp[i] == 0,
                        flash: hits.iter().any(|h| h.target == target),
                    });
            }
            for (index, d) in state.config.outposts[i].detectors.iter().enumerate() {
                let target = ArmorTarget::Outpost {
                    outpost: i as u32,
                    face: index as u32,
                };
                targets
                    .0
                    .push(rm_simulator_render::targets::TargetAppearance {
                        id: (128 + i * 32 + index) as u64,
                        pose: pose_flu(state.rotated_at(side, d.pose, round_ns)),
                        size_m: d.half_m.map(|v| (v * 2.) as f32),
                        team: team_color(owner),
                        disabled: state.game.outpost_hp[i] == 0,
                        flash: hits.iter().any(|h| h.target == target),
                    });
            }
        }
        for c in &mut scene.chassis {
            if let Some(r) = state.game.robots.iter().find(|r| r.id == c.id) {
                c.defeated = r.hp == 0;
                c.hp_fraction = r.hp as f32 / r.max_hp as f32;
            }
        }
    }
    input.0 = Some(scene);
}

/// Detected strikes still within the flash window.
fn flashing(snapshot: &FieldSnapshot, hit_flash_ns: u64) -> impl Iterator<Item = &ArmorHit> {
    snapshot.hits.iter().filter(move |hit| {
        hit.detected && snapshot.time_ns.saturating_sub(hit.time_ns) < hit_flash_ns
    })
}

/// Blink an activated rune: `flashes` on/off cycles at `flash_hz`, then lit.
/// A zero rate never blinks; a zero count blinks until the state changes.
fn rune_lit(since_ns: u64, now_ns: u64, flash_hz: f64, flashes: u32) -> bool {
    if flash_hz <= 0.0 {
        return true;
    }
    let phase = now_ns.saturating_sub(since_ns) as f64 * 1e-9 * flash_hz;
    if flashes > 0 && phase >= f64::from(flashes) {
        return true;
    }
    phase.fract() < 0.5
}

/// Owner of field object `index` according to the referee, if there is one.
fn referee_team(
    snapshot: &FieldSnapshot,
    owners: impl Fn(&RefereeSnapshot) -> &Vec<Team>,
    index: usize,
) -> Option<Team> {
    snapshot
        .referee
        .as_ref()
        .and_then(|referee| owners(referee).get(index).copied())
}

fn team_color(team: Team) -> TeamColor {
    match team {
        Team::Red => TeamColor::Red,
        Team::Blue => TeamColor::Blue,
    }
}

#[cfg(test)]
fn scene_state(snapshot: &FieldSnapshot, flash: FlashSettings) -> SceneState {
    scene_state_at(snapshot, flash, snapshot.time_ns)
}

#[cfg(test)]
fn scene_state_at(
    snapshot: &FieldSnapshot,
    flash: FlashSettings,
    presentation_ns: u64,
) -> SceneState {
    let hits: Vec<_> = flashing(snapshot, flash.hit_flash_ns).collect();
    scene_state_with_hits(snapshot, flash, presentation_ns, &hits)
}

fn scene_state_with_hits(
    snapshot: &FieldSnapshot,
    flash: FlashSettings,
    presentation_ns: u64,
    hits: &[&ArmorHit],
) -> SceneState {
    let mut rune_flash: HashMap<u32, [bool; 5]> = HashMap::new();
    let mut outpost_flash: HashMap<u32, [bool; outpost::FACE_COUNT]> = HashMap::new();
    for hit in hits {
        match hit.target {
            ArmorTarget::Rune { rune, blade } => {
                if let Some(flag) = rune_flash.entry(rune).or_default().get_mut(blade as usize) {
                    *flag = true;
                }
            }
            ArmorTarget::Outpost { outpost, face } => {
                if let Some(flag) = outpost_flash
                    .entry(outpost)
                    .or_default()
                    .get_mut(face as usize)
                {
                    *flag = true;
                }
            }
            // Chassis flashes are gathered with the chassis in `publish_scene`.
            ArmorTarget::Chassis { .. } | ArmorTarget::Base { .. } => {}
        }
    }
    let dart_position = rm_simulator_world::referee::DART_TARGET_REST;
    SceneState {
        bases: snapshot
            .bases
            .iter()
            .enumerate()
            .map(|(index, base)| rm_simulator_render::sync::BaseAppearance {
                disabled: base.hp == 0,
                plates: std::array::from_fn(|plate| pose_flu(base.pose(plate, dart_position))),
                hit_flash: std::array::from_fn(|plate| {
                    hits.iter().any(|hit| {
                        hit.target
                            == ArmorTarget::Base {
                                base: index as u32,
                                plate: plate as u32,
                            }
                    })
                }),
            })
            .collect(),
        dart_target_fraction: dart_position as f32,
        base_open_fraction: [0.; 2],
        dart_door_open: [true; 2],
        source: SourceTime {
            tick: snapshot.tick,
            time_ns: snapshot.time_ns,
        },
        runes: snapshot
            .runes
            .iter()
            .enumerate()
            .map(|(index, rune)| {
                let big = rune.kind == RuneKind::Big;
                let activated = rune.state == RuneState::Activated;
                // An activated rune blinks its arms (a visual aid, see
                // `--rune-flash-hz`); the rule text shows them lit.
                let lit = !activated
                    || rune_lit(
                        rune.state_since_ns,
                        presentation_ns,
                        flash.rune_flash_hz,
                        flash.rune_flashes,
                    );
                RuneAppearance {
                    hub_pose: pose_flu(rune.hub_pose),
                    angle_rad: rune.presentation_angle_rad(presentation_ns),
                    active_blades: rune.active_blades,
                    // Big Rune shows hit blades only once the whole rune is activated.
                    activated_blades: rune.activated.map(|hit| hit && lit && (!big || activated)),
                    fully_activated: activated && lit,
                    progress_stages: std::array::from_fn(|stage| {
                        big && lit && (stage as u32) < rune.completed_groups
                    }),
                    hit_flash: rune_flash.get(&(index as u32)).copied().unwrap_or_default(),
                }
            })
            .collect(),
        outposts: snapshot
            .outposts
            .iter()
            .enumerate()
            .map(|(index, outpost)| {
                let outpost = outpost.presentation_at(presentation_ns);
                OutpostAppearance {
                    disabled: outpost.destroyed,
                    origin: pose_flu(outpost.origin),
                    angle_rad: outpost.angle_rad,
                    armors: std::array::from_fn(|face| {
                        pose_flu(
                            outpost
                                .armors
                                .iter()
                                .find(|armor| armor.id == face as u32)
                                .expect("outpost snapshot has three faces")
                                .pose,
                        )
                    }),
                    hit_flash: outpost_flash
                        .get(&(index as u32))
                        .copied()
                        .unwrap_or_default(),
                }
            })
            .collect(),
        projectiles: snapshot
            .projectiles
            .iter()
            .map(|projectile| ProjectileAppearance {
                position_m: projectile.position_m,
                radius_m: (projectile.caliber.diameter_m() / 2.0) as f32,
            })
            .collect(),
        chassis: Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::DVec3;
    use rm_simulator_world::{Caliber, ChassisConfig, Field, Pose, tick_ns};

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        a.iter().zip(&b).all(|(a, b)| (a - b).abs() < 1e-4)
    }
    fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
        let [w, x, y, z] = q.map(|v| v as f32);
        let out = Quat::from_xyzw(x, y, z, w) * Vec3::new(v[0] as f32, v[1] as f32, v[2] as f32);
        [out.x as f64, out.y as f64, out.z as f64]
    }
    #[test]
    fn presentation_reconstructs_motion_without_changing_source_or_authority() {
        let mut checkpoint = Field::new(&Default::default()).unwrap().snapshot();
        checkpoint
            .runes
            .push(crate::test_fixtures::rune(Pose::at([6., 0., 1.6])));
        checkpoint
            .outposts
            .push(crate::test_fixtures::outpost(Pose::at([7., 0., 0.])));
        let flash = FlashSettings {
            hit_flash_ns: 50_000_000,
            rune_flash_hz: 2.,
            rune_flashes: 3,
        };
        let at = 123_000_000 / tick_ns() * tick_ns();
        let presented = scene_state_at(&checkpoint, flash, at);
        assert!(
            (presented.runes[0].angle_rad - checkpoint.runes[0].presentation_angle_rad(at)).abs()
                < 1e-12
        );
        assert_eq!(presented.source.tick, checkpoint.tick);
        assert_eq!(presented.source.time_ns, checkpoint.time_ns);
        let exact = scene_state_at(&checkpoint, flash, checkpoint.time_ns);
        assert_eq!(
            exact.outposts[0].angle_rad,
            checkpoint.outposts[0].angle_rad
        );
    }

    #[test]
    fn rune_state_masks_clear_on_failure_deactivation_and_big_completion_blinks() {
        let mut snapshot = Field::new(&Default::default()).unwrap().snapshot();
        snapshot.time_ns = 3_000_000_000;
        snapshot
            .runes
            .push(crate::test_fixtures::rune(Pose::at([6., 0., 1.6])));
        let flash = FlashSettings {
            hit_flash_ns: 50_000_000,
            rune_flash_hz: 2.,
            rune_flashes: 3,
        };
        snapshot.runes[0].activated[0] = true;
        assert!(scene_state(&snapshot, flash).runes[0].activated_blades[0]);
        snapshot.runes[0].activated = [false; 5];
        assert_eq!(
            scene_state(&snapshot, flash).runes[0].activated_blades,
            [false; 5]
        );
        let rune = &mut snapshot.runes[0];
        rune.kind = RuneKind::Big;
        rune.state = RuneState::Activated;
        rune.activated = [true; 5];
        rune.completed_groups = 5;
        rune.state_since_ns = snapshot.time_ns - 300_000_000;
        let dark = scene_state(&snapshot, flash);
        assert_eq!(dark.runes[0].activated_blades, [false; 5]);
        assert_eq!(dark.runes[0].progress_stages, [false; 5]);
        assert!(!dark.runes[0].fully_activated);
        snapshot.time_ns += 300_000_000;
        let lit = scene_state(&snapshot, flash);
        assert_eq!(lit.runes[0].progress_stages, [true; 5]);
        assert!(lit.runes[0].fully_activated);
    }

    #[test]
    fn additional_static_assets_are_drawn_once_per_placement() {
        let asset = |file: &str, count: usize| rm_simulator_server::cad_assets::CadAsset {
            semantics: None,
            file: file.to_owned(),
            placements: vec![rm_simulator_world::Pose::at([1.0, 2.0, 3.0]); count],
            collision: None,
            source_tessellated_collision: false,
        };
        let cad = CadAssets {
            rmuc_2027: None,
            season: rm_simulator_world::season::Season::Rm2026,
            rest_pose_only: false,
            root: std::path::PathBuf::new(),
            floor_top_cad_m: -1.6,
            collision_solids: true,
            floor: asset("floor", 0),
            arena_static: asset("arena", 0),
            rune: asset("rune", 1),
            outpost: asset("outpost", 2),
            base: asset("base", 2),
            tech_core: asset("tech-core", 2),
            static_assets: vec![
                asset("centre", 1),
                asset("dart", 2),
                asset("resource", 2),
                asset("footing", 2),
            ],
        };
        let instances = cad_instances(&cad, true, false);
        assert_eq!(instances.len(), 16);
        for asset in &cad.static_assets {
            let drawn: Vec<_> = instances.iter().filter(|i| i.file == asset.file).collect();
            assert_eq!(drawn.len(), asset.placements.len());
            for (instance, placement) in drawn.iter().zip(&asset.placements) {
                assert_eq!(instance.transform, transform_of(placement));
                assert!(matches!(
                    instance.role,
                    CadRole::Static { base_color: None }
                ));
            }
        }
    }
    #[test]
    fn rest_pose_cad_keeps_source_color_and_never_binds_old_animation() {
        let mut cad = crate::session::test_cad_assets();
        cad.rest_pose_only = true;
        cad.season = rm_simulator_world::season::Season::Rm2027;
        cad.floor.placements.clear();
        cad.arena_static.placements.clear();
        let instances = cad_instances(&cad, true, true);
        assert_eq!(instances.len(), 6);
        assert_eq!(instances[0].transform, transform_of(&cad.arena_pose()));
        assert!(instances.iter().all(|instance| instance.team.is_none()
            && matches!(instance.role, CadRole::Static { base_color: None })));
    }

    #[test]
    fn activated_runes_blink_at_the_configured_rate() {
        // 2 Hz: lit for the first quarter second of each half second.
        assert!(rune_lit(1_000_000_000, 1_000_000_000, 2.0, 3));
        assert!(rune_lit(1_000_000_000, 1_249_000_000, 2.0, 3));
        assert!(!rune_lit(1_000_000_000, 1_250_000_000, 2.0, 3));
        assert!(!rune_lit(1_000_000_000, 1_499_000_000, 2.0, 3));
        assert!(rune_lit(1_000_000_000, 1_500_000_000, 2.0, 3));
        // Three flashes take 1.5 s; the dark half of the fourth cycle stays lit.
        assert!(!rune_lit(1_000_000_000, 2_300_000_000, 2.0, 3));
        assert!(rune_lit(1_000_000_000, 2_800_000_000, 2.0, 3));
        assert!(rune_lit(1_000_000_000, 9_800_000_000, 2.0, 3));
        // A zero count blinks forever; a zero rate disables the blink;
        // a clock behind the activation stays lit.
        assert!(!rune_lit(1_000_000_000, 9_800_000_000, 2.0, 0));
        assert!(rune_lit(1_000_000_000, 1_300_000_000, 0.0, 3));
        assert!(rune_lit(5_000_000_000, 1_000_000_000, 2.0, 3));
        // The scene adaptation applies it to the activated arms only.
        let mut snapshot = Field::new(&Default::default()).unwrap().snapshot();
        let mut rune = crate::test_fixtures::rune(Pose::at([6., 0., 1.6]));
        rune.state = RuneState::Activated;
        rune.activated = [true; 5];
        rune.active_blades = [false; 5];
        snapshot.runes.push(rune);
        let since = snapshot.runes[0].state_since_ns;
        let flash = |hz| FlashSettings {
            hit_flash_ns: 50_000_000,
            rune_flash_hz: hz,
            rune_flashes: 3,
        };
        let lit = scene_state(&snapshot, flash(0.0));
        assert_eq!(lit.runes[0].activated_blades, [true; 5]);
        // Step into the dark half of a 2 Hz blink: the first dark moment at
        // least one step past the snapshot, wrapped forward by blink periods.
        let period = 500_000_000;
        let mut dark_at = since + 300_000_000;
        dark_at += (snapshot.time_ns.saturating_sub(dark_at) / period + 1) * period;
        snapshot.time_ns = dark_at;
        let dark = scene_state(&snapshot, flash(2.0));
        assert_eq!(dark.runes[0].activated_blades, [false; 5]);
        let still_lit = scene_state(&snapshot, flash(0.0));
        assert_eq!(still_lit.runes[0].activated_blades, [true; 5]);
    }
    #[test]
    fn hit_flash_window_follows_the_setting() {
        let mut snapshot = Field::new(&Default::default()).unwrap().snapshot();
        snapshot.time_ns = 300_000_000;
        snapshot
            .outposts
            .push(crate::test_fixtures::outpost(Pose::at([4., 0., 0.])));
        snapshot.hits.push(rm_simulator_world::ArmorHit {
            time_ns: 100_000_000,
            projectile: 0,
            shooter: None,
            caliber: Caliber::Mm17,
            target: ArmorTarget::Outpost {
                outpost: 0,
                face: 0,
            },
            position_m: [0.; 3],
            local_offset_m: [0.; 2],
            normal_speed_m_s: 25.,
            detected: true,
            rejection: None,
            rune_outcome: None,
            damage: 0,
        });
        let hit = &snapshot.hits[0];
        let age_ns = snapshot.time_ns - hit.time_ns;
        assert!(age_ns > 50_000_000, "{age_ns}");
        let short = FlashSettings {
            hit_flash_ns: 50_000_000,
            rune_flash_hz: 0.0,
            rune_flashes: 3,
        };
        let long = FlashSettings {
            hit_flash_ns: age_ns + 1,
            rune_flash_hz: 0.0,
            rune_flashes: 3,
        };
        assert_eq!(
            scene_state(&snapshot, short).outposts[0].hit_flash,
            [false; 3]
        );
        assert!(scene_state(&snapshot, long).outposts[0].hit_flash[0]);
    }
    /// On a chassis rolled 20° the yaw stage stays bolted to the body and
    /// the pitch stage still points the barrel along the world aim.
    #[test]
    fn gimbal_stages_follow_the_body_and_point_the_barrel_at_the_aim() {
        let roll = DQuat::from_rotation_x(20_f64.to_radians());
        let body = PoseFlu {
            translation_m: [0.0, 0.0, 0.2],
            rotation_wxyz: wxyz(roll),
        };
        let aim_rotation = DQuat::from_rotation_z(0.7) * DQuat::from_rotation_y(-0.3);
        let aim = PoseFlu {
            translation_m: [0.0, 0.0, 0.4],
            rotation_wxyz: wxyz(aim_rotation),
        };
        let (stage, turret) = gimbal_poses(body, aim);
        let stage_up = rotate(stage.rotation_wxyz, [0.0, 0.0, 1.0]);
        let body_up = roll * DVec3::Z;
        assert!(close(stage_up, body_up.to_array()), "{stage_up:?}");
        let barrel = rotate(turret.rotation_wxyz, [1.0, 0.0, 0.0]);
        let wanted = aim_rotation * DVec3::X;
        assert!(close(barrel, wanted.to_array()), "{barrel:?} vs {wanted:?}");
        // The pitch axis is the yaw stage's left axis: the turret's left
        // stays in the stage's plane.
        let turret_left = rotate(turret.rotation_wxyz, [0.0, 1.0, 0.0]);
        let stage_left = rotate(stage.rotation_wxyz, [0.0, 1.0, 0.0]);
        assert!(
            close(turret_left, stage_left),
            "{turret_left:?} vs {stage_left:?}"
        );
    }
    #[test]
    fn wheel_visuals_put_the_axle_on_the_hub_radius_and_the_turret_above_the_body() {
        let config = ChassisConfig::default();
        let yaw = std::f64::consts::FRAC_PI_2;
        let mut field = Field::new(&rm_simulator_world::FieldConfig {
            chassis: vec![rm_simulator_world::ChassisPlacement {
                config: config.clone(),
                spawn: Pose::yawed([1.0, 0.0, config.rest_height_m()], yaw),
                team: Team::Blue,
                kind: rm_simulator_world::RobotKind::Infantry,
                performance: None,
            }],
            ..Default::default()
        })
        .unwrap();
        field.step(1).unwrap();
        let chassis = field.snapshot().chassis.remove(0);
        for (wheel, hub) in chassis.wheels.iter().zip(&config.wheel_hubs_m) {
            let pose = wheel_pose(&chassis, *hub, wheel);
            let axle = rotate(pose.rotation_wxyz, [0.0, 1.0, 0.0]);
            // The axle is the radial direction of the hub, turned with the body.
            let radius = (hub[0].powi(2) + hub[1].powi(2)).sqrt();
            let expected = [-hub[1] / radius, hub[0] / radius, 0.0];
            assert!(close(axle, expected), "{axle:?} vs {expected:?}");
        }
        let pivot = chassis.turret.translation_m;
        assert!(
            (pivot[2] - chassis.pose.translation_m[2] - config.turret_center_m[2]).abs() < 1e-6
        );
        assert!((pivot[0] - 1.0).abs() < 1e-3 && pivot[1].abs() < 1e-3);
        // Another player's turret is drawn pointing where the world holds
        // its aim, on a gimbal that turns with the body.
        let appearance = chassis_appearance(&chassis, &[false, true], None);
        assert_eq!(appearance.turret.translation_m, pivot);
        assert_eq!(appearance.team, TeamColor::Blue);
        assert_eq!(appearance.yaw_stage.translation_m, pivot);
        let barrel = rotate(appearance.turret.rotation_wxyz, [1.0, 0.0, 0.0]);
        let aim = rotate(chassis.turret.rotation_wxyz, [1.0, 0.0, 0.0]);
        assert!(close(barrel, aim), "{barrel:?} vs {aim:?}");
        let stage_up = rotate(appearance.yaw_stage.rotation_wxyz, [0.0, 0.0, 1.0]);
        let body_up = rotate(chassis.pose.rotation_wxyz, [0.0, 0.0, 1.0]);
        assert!(close(stage_up, body_up), "{stage_up:?} vs {body_up:?}");
        assert_eq!(appearance.armor.len(), 4);
        assert!(!appearance.armor[0].hit_flash && appearance.armor[1].hit_flash);
        assert!(!appearance.defeated);
        assert!((appearance.pivot_above_body_m - 0.15).abs() < 1e-6);
    }
}
