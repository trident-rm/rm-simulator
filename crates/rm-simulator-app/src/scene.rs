// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! World snapshots into the renderer's `SceneState`: the static CAD
//! instances, the rune and outpost light overlays, projectiles, hit
//! flashes and the chassis visuals. The renderer is passive; this is the
//! only place that decides what it shows.
use bevy::prelude::*;
use rm_simulator_render::{
    RenderingConfig, TeamColor,
    armor::{ArmorAtlas, DiffuserProfile},
    cad::{CadInstance, CadJoint, CadRole, CadRuneFace, CadTargetLight, JointDriver},
    camera_appearance, lighting, outpost, rune,
    sync::{ProjectileAppearance, SceneInput},
};
use rm_simulator_server::{
    cad_assets::CadAssets,
    layout::{outpost_team, rune_team},
};
use rm_simulator_world::{
    ArmorTarget, FieldSnapshot, RefereeSnapshot, Team, outpost as outpost_rules,
};
use std::collections::HashMap;

use crate::controls::{Drive, PlayerCamera};
use crate::frames::transform_of;
pub use rm_simulator_presentation::armor_optics;
use rm_simulator_presentation::flashing;

use crate::session::Session;

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
    let scene = input.0.get_or_insert_default();
    rm_simulator_presentation::scene_state_into(snapshot, session.flash, time_ns, &hits, scene);
    scene.projectiles.clear();
    scene.projectiles.extend(
        session
            .visual_projectiles()
            .map(|projectile| ProjectileAppearance {
                position_m: projectile.position_m,
                radius_m: (projectile.caliber.diameter_m() / 2.0) as f32,
            }),
    );
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
    scene.chassis.clear();
    scene.chassis.extend(snapshot.chassis.iter().map(|chassis| {
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
    }));
    rm_simulator_presentation::native_scene_into(
        snapshot,
        time_ns,
        session.paused,
        &hits,
        scene,
        &mut targets.0,
    );
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

/// Convert transport roster identity at the application boundary.
fn presentation_robot(
    robot: rm_simulator_server::protocol::Robot,
) -> rm_simulator_presentation::Robot {
    use rm_simulator_server::protocol::Robot;
    match robot {
        Robot::Hero => rm_simulator_presentation::Robot::Hero,
        Robot::Engineer => rm_simulator_presentation::Robot::Engineer,
        Robot::Infantry3 => rm_simulator_presentation::Robot::Infantry3,
        Robot::Infantry4 => rm_simulator_presentation::Robot::Infantry4,
        Robot::Infantry5 => rm_simulator_presentation::Robot::Infantry5,
        Robot::Sentry => rm_simulator_presentation::Robot::Sentry,
        Robot::Drone => rm_simulator_presentation::Robot::Drone,
    }
}

/// Build one visual chassis from the app roster identity.
pub(crate) fn chassis_appearance(
    chassis: &rm_simulator_world::ChassisSnapshot,
    flash: &[bool],
    robot: Option<rm_simulator_server::protocol::Robot>,
) -> rm_simulator_render::sync::ChassisAppearance {
    rm_simulator_presentation::chassis_appearance(chassis, flash, robot.map(presentation_robot))
}

#[cfg(test)]
mod tests {
    use super::*;
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
}
