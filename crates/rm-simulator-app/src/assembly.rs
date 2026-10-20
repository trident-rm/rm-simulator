// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Live automatic assembly controls and source-linked articulated presentation.
mod lights;
use crate::{hud::HudState, loading::Screen, session::Session};
use bevy::{prelude::*, world_serialization::WorldInstanceReady};
use rm_simulator_server::protocol::Command;
use rm_simulator_world::{
    Pose,
    assembly::{Model, Phase, Run},
};
use std::{collections::BTreeSet, path::Path};

#[derive(Resource)]
struct Assets {
    model: Model,
}
/// Register and verify the external source package before asset plugins start.
pub fn register(app: &mut App, root: Option<&Path>) -> Result<(), String> {
    let Some(root) = root else {
        return Ok(());
    };
    let manifest: serde_json::Value = serde_json::from_slice(
        &std::fs::read(root.join("manifest.json")).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    if manifest["profile"] != "pie-rmuc2027-assembly-workbench-v5" {
        return Err("unsupported assembly source binding".into());
    }
    use sha2::{Digest, Sha256};
    for file in [
        "engineer",
        "core-base",
        "core-arm",
        "core-head",
        "unit",
        "config",
    ] {
        let entry = &manifest["files"][file];
        let name = entry["file"].as_str().ok_or("missing assembly file")?;
        if Path::new(name).components().count() != 1 {
            return Err("invalid assembly file path".into());
        }
        let bytes = std::fs::read(root.join(name)).map_err(|e| e.to_string())?;
        if entry["sha256"] != format!("{:x}", Sha256::digest(&bytes)) {
            return Err(format!("assembly {file} checksum mismatch"));
        }
    }
    let mut model: Model = serde_json::from_slice(
        &std::fs::read(root.join("assembly.json")).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    model.core_envelope_m.clear();
    let bytes = std::fs::read(root.join("engineer.glb")).map_err(|e| e.to_string())?;
    let length = u32::from_le_bytes(
        bytes
            .get(12..16)
            .ok_or("truncated Engineer GLB")?
            .try_into()
            .unwrap(),
    ) as usize;
    let scene: serde_json::Value = serde_json::from_slice(
        bytes
            .get(20..20 + length)
            .ok_or("truncated Engineer GLB JSON")?,
    )
    .map_err(|e| e.to_string())?;
    let nodes = scene["nodes"].as_array().ok_or("missing Engineer nodes")?;
    let mut names = BTreeSet::new();
    for node in nodes.iter().filter(|n| {
        n["name"]
            .as_str()
            .is_some_and(|s| s.starts_with("link/engineer/") && s != "link/engineer/body")
    }) {
        let mut children: Vec<_> = node["children"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|v| v.as_u64().map(|i| i as usize))
            .collect();
        while let Some(index) = children.pop() {
            let n = nodes.get(index).ok_or("invalid Engineer node")?;
            if n.get("mesh").is_some()
                && let Some(name) = n["name"].as_str()
            {
                names.insert(name.to_string());
            }
            children.extend(
                n["children"]
                    .as_array()
                    .into_iter()
                    .flatten()
                    .filter_map(|v| v.as_u64().map(|i| i as usize)),
            );
        }
    }
    app.insert_resource(rm_simulator_render::team_robot::ReplacedArmMeshes(names));
    let root = std::path::absolute(root).map_err(|e| e.to_string())?;
    use bevy::asset::io::{AssetSourceBuilder, file::FileAssetReader};
    app.register_asset_source(
        "assembly",
        AssetSourceBuilder::new(move || Box::new(FileAssetReader::new(root.clone()))),
    );
    app.insert_resource(Assets { model });
    Ok(())
}
/// Adds a screen-scoped panel; all actions travel through the host protocol.
pub struct AssemblyPlugin;
impl Plugin for AssemblyPlugin {
    fn build(&self, app: &mut App) {
        lights::install(app);
        app.init_resource::<VisualOwners>()
            .init_resource::<MotionViews>()
            .add_systems(OnEnter(Screen::InMatch), panel)
            .add_systems(OnExit(Screen::InMatch), cleanup)
            .add_observer(loaded)
            .add_systems(
                Update,
                (toggle, update_panel, sample_motion, visuals)
                    .chain()
                    .after(crate::session::advance_world)
                    .before(crate::scene::publish_scene)
                    .run_if(in_state(Screen::InMatch)),
            )
            .add_systems(
                Update,
                (align_arms, align_carried_units)
                    .chain()
                    .after(crate::scene::publish_scene)
                    .before(rm_simulator_render::sync::SceneSyncSet)
                    .run_if(in_state(Screen::InMatch)),
            );
    }
}
// Match the ordinary robot's presented pose, including owner prediction and
// remote interpolation. Using the last authority snapshot here detaches the
// arm from a moving chassis between network updates.
fn align_arms(
    scene: Res<rm_simulator_render::sync::SceneInput>,
    mut roots: Query<(&Root, &mut Transform)>,
) {
    let Some(scene) = &scene.0 else { return };
    for (root, mut transform) in &mut roots {
        if root.kind == 0
            && let Some(chassis) = scene.chassis.iter().find(|c| c.id == root.owner)
        {
            transform.set_if_neq(root_transform(Pose {
                translation_m: chassis.pose.translation_m,
                rotation_wxyz: chassis.pose.rotation_wxyz,
            }));
        }
    }
}
fn sample_idle_arm(
    arm: &rm_simulator_world::assembly::ArmState,
    ahead_ns: u64,
) -> rm_simulator_world::assembly::ArmState {
    let mut sampled = arm.clone();
    let dt = ahead_ns as f64 * 1e-9;
    for i in 0..6 {
        sampled.joints_rad[i] += (sampled.target_rad[i] - sampled.joints_rad[i]).clamp(-dt, dt);
    }
    for i in 0..2 {
        sampled.jaws_rad[i] += (sampled.jaw_target_rad[i] - sampled.jaws_rad[i]).clamp(-dt, dt);
    }
    sampled
}
fn align_carried_units(
    scene: Res<rm_simulator_render::sync::SceneInput>,
    session: Res<Session>,
    views: Res<MotionViews>,
    mut roots: Query<(&Root, &mut Transform)>,
) {
    let Some(scene) = &scene.0 else { return };
    let Some(restore) = &session.snapshot.restore else {
        return;
    };
    for (root, mut transform) in &mut roots {
        if root.kind != 4 {
            continue;
        }
        let Some(unit) = restore
            .energy_units
            .iter()
            .find(|u| u.physical.id == root.owner)
        else {
            continue;
        };
        let rm_simulator_world::energy_unit::Attachment::Held(owner) = unit.attachment else {
            continue;
        };
        let Some(chassis) = scene.chassis.iter().find(|c| c.id == owner) else {
            continue;
        };
        let Some(arm) = restore.arms.iter().find(|a| a.owner == owner) else {
            continue;
        };
        let body = Pose {
            translation_m: chassis.pose.translation_m,
            rotation_wxyz: chassis.pose.rotation_wxyz,
        };
        let local = if let Some(run) = restore
            .assemblies
            .iter()
            .find(|r| r.owner == owner && r.active())
            && let Some(view) = views
                .0
                .iter()
                .find(|v| v.owner == owner && v.generation == run.generation)
        {
            compose(
                arm.model.engineer.tool_pose(view.pose.engineer_q),
                unit.grasp,
            )
        } else {
            let ahead = if session.paused {
                0
            } else {
                session
                    .presentation_time_ns()
                    .saturating_sub(session.snapshot.time_ns)
                    .min(100_000_000)
            };
            compose(
                arm.model
                    .engineer
                    .tool_pose(sample_idle_arm(arm, ahead).joints_rad),
                unit.grasp,
            )
        };
        transform.set_if_neq(root_transform(compose(body, local)));
    }
}
#[derive(Component)]
struct Panel;
#[derive(Component)]
struct Status;
#[derive(Component)]
struct Bar;
#[derive(Component)]
struct Root {
    owner: u32,
    generation: u64,
    kind: u8,
    ready: bool,
}
#[derive(Component)]
struct Link {
    owner: u32,
    generation: u64,
    name: String,
}
/// Owners whose source arm is loaded and replaces the original decorative arm.
#[derive(Resource, Default)]
pub(crate) struct VisualOwners(pub BTreeSet<u32>);
fn panel(mut commands: Commands) {
    commands.spawn((Panel,Node{position_type:PositionType::Absolute,right:px(18),top:px(160),width:px(330),padding:UiRect::all(px(12)),flex_direction:FlexDirection::Column,row_gap:px(8),..default()},BackgroundColor(Color::srgba(0.025,0.04,0.055,0.94)),Visibility::Hidden,GlobalZIndex(4)))
        .with_children(|p| {
            p.spawn((Text::new("Automatic assembly"),TextFont{font_size:FontSize::Px(18.),..default()}));
            p.spawn((Status,Text::new("Park the Heavy manually and stop.\nChoose level 1, 2 or 3."),TextFont{font_size:FontSize::Px(15.),..default()}));
            p.spawn((Node{height:px(8),width:percent(100),..default()},BackgroundColor(Color::srgb(0.12,0.16,0.20)))).with_children(|p| {p.spawn((Bar,Node{height:percent(100),width:percent(0),..default()},BackgroundColor(Color::srgb(0.2,0.8,0.65))));});
            p.spawn(Node{column_gap:px(8),..default()}).with_children(|p| {
                for (n,label) in [(1,"L1"),(2,"L2"),(3,"L3"),(0,"Cancel")] {
                    p.spawn((bevy::ui_widgets::Button,bevy::ui_widgets::ActivateOnPress,Node{padding:UiRect::axes(px(14),px(8)),..default()},BackgroundColor(Color::srgb(0.15,0.22,0.28))))
                        .observe(move |_: On<bevy::ui_widgets::Activate>, mut session: ResMut<Session>| {
                            if let Some(chassis)=session.chassis_id {
                                session.apply(if n==0 {Command::CancelAutoAssembly{chassis}}else{Command::AutoAssembly{chassis,difficulty:n}});
                            }
                        })
                        .with_child((Text::new(label),TextFont{font_size:FontSize::Px(16.),..default()},Pickable::IGNORE));
                }
            });
            p.spawn(Node{column_gap:px(5),..default()}).with_children(|p| {
                for (axis,label) in [(0,"X"),(1,"Y"),(2,"Z")] {
                    for sign in [-1.,1.] {
                        p.spawn((bevy::ui_widgets::Button,bevy::ui_widgets::ActivateOnPress,Node{padding:UiRect::axes(px(6),px(6)),..default()},BackgroundColor(Color::srgb(0.15,0.22,0.28))))
                        .observe(move |_:On<bevy::ui_widgets::Activate>,mut session:ResMut<Session>| {
                            if let Some(chassis)=session.chassis_id {let mut translation_m=[0.;3];translation_m[axis]=sign*0.01;
                                session.apply(Command::EnergyArmJog{chassis,translation_m,rotation_rad:[0.;3]});}
                        }).with_child((Text::new(format!("{label}{}",if sign>0. {"+"}else{"-"})),TextFont{font_size:FontSize::Px(13.),..default()},Pickable::IGNORE));
                    }
                }
            });
            p.spawn(Node{column_gap:px(5),..default()}).with_children(|p| {
                for (axis,label) in [(0,"R"),(1,"P"),(2,"Y")] {
                    for sign in [-1.,1.] {
                        p.spawn((bevy::ui_widgets::Button,bevy::ui_widgets::ActivateOnPress,Node{padding:UiRect::axes(px(6),px(6)),..default()},BackgroundColor(Color::srgb(0.15,0.22,0.28))))
                        .observe(move |_:On<bevy::ui_widgets::Activate>,mut session:ResMut<Session>| {
                            if let Some(chassis)=session.chassis_id {let mut rotation_rad=[0.;3];rotation_rad[axis]=sign*5f64.to_radians();
                                session.apply(Command::EnergyArmJog{chassis,translation_m:[0.;3],rotation_rad});}
                        }).with_child((Text::new(format!("{label}{}",if sign>0. {"+"}else{"-"})),TextFont{font_size:FontSize::Px(13.),..default()},Pickable::IGNORE));
                    }
                }
            });
            p.spawn((bevy::ui_widgets::Button,bevy::ui_widgets::ActivateOnPress,Node{padding:UiRect::axes(px(10),px(6)),..default()},BackgroundColor(Color::srgb(0.15,0.22,0.28))))
                .observe(|_:On<bevy::ui_widgets::Activate>,mut session:ResMut<Session>| {if let Some(chassis)=session.chassis_id {session.apply(Command::EnergyGrip{chassis});}})
                .with_child((Text::new("Grip / release Unit"),TextFont{font_size:FontSize::Px(14.),..default()},Pickable::IGNORE));
            p.spawn((bevy::ui_widgets::Button,bevy::ui_widgets::ActivateOnPress,Node{padding:UiRect::axes(px(10),px(6)),..default()},BackgroundColor(Color::srgb(0.15,0.22,0.28))))
                .observe(|_:On<bevy::ui_widgets::Activate>,mut session:ResMut<Session>| {if let Some(chassis)=session.chassis_id {session.apply(Command::ConvertHeavy{chassis});}})
                .with_child((Text::new("Convert to Hero (when unlocked)"),TextFont{font_size:FontSize::Px(14.),..default()},Pickable::IGNORE));
            p.spawn((Text::new("Driving cancels arm work; Core recovery continues.\nSearch is asynchronous; motion uses game time."),TextFont{font_size:FontSize::Px(12.),..default()}));
        });
}
fn toggle(
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    session: Res<Session>,
    mut hud: ResMut<HudState>,
) {
    if hud
        .controls
        .just_pressed(crate::bindings::InputAction::Assembly, &keys, Some(&mouse))
        && !hud.rebinding
        && (session.snapshot.rmuc_2027.as_ref().is_some_and(|s| {
            s.game.robots.iter().any(|r| {
                Some(r.id) == session.chassis_id
                    && r.role == rm_simulator_world::rmuc_2027::match_rules::GroundRole::Heavy
            })
        }) || session.own_chassis().is_some_and(|c| {
            c.config
                .articulation
                .as_ref()
                .is_some_and(|a| a.profile == "pie-tunnel-heavy-v1")
        }))
    {
        hud.assembly = !hud.assembly;
        hud.consumed = true;
    }
}
#[derive(Resource, Default)]
struct MotionViews(Vec<MotionView>);
struct MotionView {
    owner: u32,
    generation: u64,
    pose: rm_simulator_world::assembly::RigPose,
    cursor: u32,
}
// Present only already supplied motion. This does not advance the evaluator,
// reveal a target, confirm assembly or start a new worker.
fn sample_motion(session: Res<Session>, mut views: ResMut<MotionViews>) {
    views.0.clear();
    let ahead = if session.paused {
        0
    } else {
        session
            .presentation_time_ns()
            .saturating_sub(session.snapshot.time_ns)
            .min(100_000_000)
    };
    for run in runs(&session) {
        let mut pose = run.pose.clone();
        let mut cursor = run.cursor;
        if run.phase == Phase::Recovering {
            if let Some(recovery) = &run.recovery {
                let mut sampled = recovery.clone();
                for _ in 0..ahead / rm_simulator_world::tick_ns() {
                    let mut next = pose.clone();
                    let finished = sampled.advance(&mut next);
                    // Do not predict the transition from prescribed attachment
                    // to unknown free rigid-body contacts.
                    if !recovery.released() && sampled.released() {
                        break;
                    }
                    pose = next;
                    if finished {
                        break;
                    }
                }
            }
        } else if run.phase == Phase::Positioning {
            if let Some(m) = &run.positioning {
                let t = (m.elapsed_ns.saturating_add(ahead).min(m.duration_ns)) as f64
                    / m.duration_ns.max(1) as f64;
                pose.core_q =
                    std::array::from_fn(|i| m.from_rad[i] + (m.target_rad[i] - m.from_rad[i]) * t);
                pose.lift_m = m.lift_m * (1. - t);
                pose.p_rad = m.angles_rad[0] * (1. - t);
                pose.q_rad = m.angles_rad[1] * (1. - t);
            }
        } else if matches!(run.phase, Phase::Actuating | Phase::BreakingAway) {
            let count = (ahead / rm_simulator_world::tick_ns()) as usize;
            // Never predict across the unknown-target or completion boundary.
            let boundary = run
                .frames
                .iter()
                .position(|f| f.stage == 7)
                .unwrap_or(run.frames.len());
            let limit = if run.phase == Phase::Actuating {
                boundary
            } else {
                run.frames.len()
            };
            let end = (run.cursor as usize + count)
                .min(limit.saturating_sub(1))
                .max(run.cursor as usize);
            if end > run.cursor as usize
                && let Some(f) = run.frames.get(end - 1)
            {
                pose.engineer_q = f.joints_rad;
                pose.jaw_angles = f.jaws_rad;
                pose.unit_pose = f.unit;
                pose.lift_m = f.guide_m[2] - 0.6;
                pose.p_rad = f.angles_rad[0];
                pose.q_rad = f.angles_rad[1];
                pose.gripped = f.gripped;
                cursor = end as u32;
            }
        }
        views.0.push(MotionView {
            owner: run.owner,
            generation: run.generation,
            pose,
            cursor,
        });
    }
}
fn runs(session: &Session) -> &[Run] {
    session
        .snapshot
        .restore
        .as_ref()
        .map_or(&[], |r| r.assemblies.as_slice())
}
fn update_panel(
    session: Res<Session>,
    hud: Res<HudState>,
    mut panels: Query<&mut Visibility, With<Panel>>,
    mut text: Query<&mut Text, With<Status>>,
    mut bars: Query<&mut Node, With<Bar>>,
) {
    let run = runs(&session)
        .iter()
        .find(|r| Some(r.owner) == session.chassis_id);
    for mut panel in &mut panels {
        panel.set_if_neq(if hud.assembly {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        });
    }
    let (mut message, progress) = if let Some(r) = run {
        let progress = match r.phase {
            Phase::Positioning => r.positioning.as_ref().map_or(0., |p| p.progress() as f32),
            Phase::Recovering => r.recovery.as_ref().map_or(0., |p| p.progress()),
            Phase::Solving | Phase::SolvingTwist => r.checks as f32 / r.budget.max(1) as f32,
            Phase::Actuating | Phase::BreakingAway | Phase::Completed => {
                r.cursor as f32 / r.frames.len().max(1) as f32
            }
            _ => 0.,
        };
        let label = if r.phase == Phase::Recovering {
            format!("Core recovery: {:.0}%", progress * 100.)
        } else if r.phase == Phase::Positioning {
            format!("Core positioning: {:.0}%", progress * 100.)
        } else if matches!(r.phase, Phase::Solving | Phase::SolvingTwist) {
            format!("Search budget: {} / {}", r.checks, r.budget)
        } else {
            format!("Actuation: {} / {} ticks", r.cursor, r.frames.len())
        };
        (
            format!(
                "L{} / {:?}\n{}\n{}",
                r.difficulty, r.phase, r.message, label
            ),
            progress,
        )
    } else {
        (
            String::from("Park the Heavy manually and stop.\nChoose level 1, 2 or 3."),
            0.,
        )
    };
    if let Some(restore) = &session.snapshot.restore {
        let held=restore.energy_units.iter().any(|u|matches!(u.attachment,rm_simulator_world::energy_unit::Attachment::Held(owner) if Some(owner)==session.chassis_id));
        message.push_str(if held {
            "\nUnit: held"
        } else {
            "\nUnit: not held"
        });
    }
    if let Some(state) = &session.snapshot.rmuc_2027
        && let Some(robot) = state.game.robots.iter().find(|r| {
            Some(r.id) == session.chassis_id
                && r.role == rm_simulator_world::rmuc_2027::match_rules::GroundRole::Heavy
        })
    {
        message.push_str(if robot.combat.heavy_installed {
            "\nHero approximation active (42 mm)."
        } else if state.game.heavy_unlocked(robot.team) {
            "\nHero conversion ready. Stop, then convert."
        } else {
            "\nHero conversion locked: L3 or catch-up timer."
        });
    }
    for mut t in &mut text {
        if t.0 != message {
            t.0 = message.clone();
        }
    }
    for mut bar in &mut bars {
        bar.width = percent(progress.clamp(0., 1.) * 100.);
    }
}
fn compose(a: Pose, b: Pose) -> Pose {
    let [w, x, y, z] = a.rotation_wxyz;
    let q = glam::DQuat::from_xyzw(x, y, z, w);
    let [w, x, y, z] = b.rotation_wxyz;
    let r = q * glam::DQuat::from_xyzw(x, y, z, w);
    Pose {
        translation_m: (glam::DVec3::from_array(a.translation_m)
            + q * glam::DVec3::from_array(b.translation_m))
        .to_array(),
        rotation_wxyz: [r.w, r.x, r.y, r.z],
    }
}
fn root_transform(p: Pose) -> Transform {
    let mut t = Transform::default();
    rm_simulator_render::apply_pose(
        &mut t,
        rm_simulator_render::PoseFlu {
            translation_m: p.translation_m,
            rotation_wxyz: p.rotation_wxyz,
        },
    );
    t.rotation *= Quat::from_rotation_y(std::f32::consts::PI);
    t
}
fn native(p: Pose) -> Transform {
    let [w, x, y, z] = p.rotation_wxyz;
    let q = Quat::from_xyzw(x as f32, y as f32, z as f32, w as f32);
    let b = Quat::from_mat3(&Mat3::from_cols(Vec3::Z, Vec3::X, Vec3::Y));
    Transform::from_translation(Vec3::new(
        p.translation_m[1] as f32,
        p.translation_m[2] as f32,
        p.translation_m[0] as f32,
    ))
    .with_rotation(b * q * b.inverse())
}
fn loaded(
    event: On<WorldInstanceReady>,
    mut roots: Query<&mut Root>,
    children: Query<&Children>,
    names: Query<&Name>,
    mut commands: Commands,
) {
    let Ok(mut root) = roots.get_mut(event.entity) else {
        return;
    };
    for entity in children.iter_descendants(event.entity) {
        if let Ok(name) = names.get(entity)
            && let Some(name) = name.as_str().strip_prefix("link/")
        {
            if root.kind == 0 && name == "engineer/body" {
                commands.entity(entity).insert(Visibility::Hidden);
            }
            commands.entity(entity).insert(Link {
                owner: root.owner,
                generation: root.generation,
                name: name.into(),
            });
        }
    }
    root.ready = true;
}
#[allow(clippy::too_many_arguments)]
fn visuals(
    mut commands: Commands,
    session: Res<Session>,
    views: Res<MotionViews>,
    assets: Option<Res<Assets>>,
    server: Res<AssetServer>,
    mut roots: Query<(Entity, &Root, &mut Transform)>,
    mut links: Query<(&Link, &mut Transform), Without<Root>>,
    mut owners: ResMut<VisualOwners>,
    mut replaced: ResMut<rm_simulator_render::team_robot::ReplacedArmOwners>,
    cad: Option<Res<rm_simulator_render::cad::CadSceneConfig>>,
    mut hidden: Option<ResMut<rm_simulator_render::cad::HiddenCadInstances>>,
) {
    owners.0.clear();
    replaced.0.clear();
    if let Some(hidden) = hidden.as_mut() {
        hidden.0.clear();
    }
    let Some(assets) = assets else {
        return;
    };
    let arms = session
        .snapshot
        .restore
        .as_ref()
        .map_or(&[][..], |r| r.arms.as_slice());
    let visible: Vec<_> = runs(&session)
        .iter()
        .filter(|r| r.model.engineer == assets.model.engineer && r.model.core == assets.model.core)
        .collect();
    let units = session
        .snapshot
        .restore
        .as_ref()
        .map_or(&[][..], |r| r.energy_units.as_slice());
    for (entity, root, mut t) in &mut roots {
        let pose = if root.kind == 0 {
            arms.iter()
                .find(|a| a.owner == root.owner)
                .and_then(|_| session.snapshot.chassis.iter().find(|c| c.id == root.owner))
                .map(|c| c.pose)
        } else if root.kind == 4 {
            units.iter().find(|u| u.physical.id == root.owner).map(|u| {
                if let rm_simulator_world::energy_unit::Attachment::Held(owner) = u.attachment
                    && let Some(run) = visible.iter().find(|r| r.owner == owner && r.active())
                    && let Some(view) = views
                        .0
                        .iter()
                        .find(|v| v.owner == owner && v.generation == run.generation)
                {
                    return compose(run.world_from_rig, view.pose.unit_pose);
                }
                if let rm_simulator_world::energy_unit::Attachment::Seated(owner) = u.attachment
                    && let Some(run) = visible
                        .iter()
                        .find(|r| r.owner == owner && r.generation == u.seated_generation)
                    && let Some(view) = views
                        .0
                        .iter()
                        .find(|v| v.owner == owner && v.generation == run.generation)
                {
                    let head = view.pose.frames(&run.model)["core/q"];
                    return compose(compose(run.world_from_rig, head), u.grasp);
                }
                u.physical.pose
            })
        } else {
            visible
                .iter()
                .find(|r| r.owner == root.owner && r.generation == root.generation)
                .map(|r| {
                    if root.kind == 4 {
                        compose(r.world_from_rig, r.pose.unit_pose)
                    } else {
                        r.world_from_rig
                    }
                })
        };
        if let Some(pose) = pose {
            let next = root_transform(pose);
            if *t != next {
                *t = next;
            }
        } else {
            commands.entity(entity).despawn();
        }
    }
    for arm in arms
        .iter()
        .filter(|a| a.model.engineer == assets.model.engineer)
    {
        let Some(chassis) = session.snapshot.chassis.iter().find(|c| c.id == arm.owner) else {
            continue;
        };
        if let Some((_, root, _)) = roots
            .iter()
            .find(|(_, r, _)| r.kind == 0 && r.owner == arm.owner)
        {
            if root.ready {
                owners.0.insert(arm.owner);
            }
        } else {
            commands.spawn((
                Root {
                    owner: arm.owner,
                    generation: 0,
                    kind: 0,
                    ready: false,
                },
                WorldAssetRoot(
                    server.load(GltfAssetLabel::Scene(0).from_asset("assembly://engineer.glb")),
                ),
                root_transform(chassis.pose),
            ));
        }
        let frames = if let Some(view) = views.0.iter().find(|v| v.owner == arm.owner).filter(|v| {
            visible
                .iter()
                .any(|r| r.owner == v.owner && r.generation == v.generation && r.active())
        }) {
            view.pose.frames(&arm.model)
        } else {
            let ahead = if session.paused || chassis.defeated {
                0
            } else {
                session
                    .presentation_time_ns()
                    .saturating_sub(session.snapshot.time_ns)
                    .min(100_000_000)
            };
            sample_idle_arm(arm, ahead).frames()
        };
        for (link, mut t) in &mut links {
            if link.owner == arm.owner
                && link.generation == 0
                && let Some(p) = frames.get(&link.name)
            {
                let next = native(*p);
                if *t != next {
                    *t = next;
                }
            }
        }
    }
    for unit in units {
        if !roots
            .iter()
            .any(|(_, r, _)| r.kind == 4 && r.owner == unit.physical.id)
        {
            commands.spawn((
                Root {
                    owner: unit.physical.id,
                    generation: 0,
                    kind: 4,
                    ready: false,
                },
                WorldAssetRoot(
                    server.load(GltfAssetLabel::Scene(0).from_asset("assembly://unit.glb")),
                ),
                root_transform(unit.physical.pose),
            ));
        }
    }
    for run in visible {
        let existing: Vec<_> = roots
            .iter()
            .filter(|(_, r, _)| {
                r.kind != 0 && r.kind != 4 && r.owner == run.owner && r.generation == run.generation
            })
            .map(|(_, r, _)| r.ready)
            .collect();
        if existing.is_empty() {
            for (kind, file) in [
                (1, "core-base.glb"),
                (2, "core-arm.glb"),
                (3, "core-head.glb"),
            ] {
                commands.spawn((
                    Root {
                        owner: run.owner,
                        generation: run.generation,
                        kind,
                        ready: false,
                    },
                    WorldAssetRoot(
                        server.load(
                            GltfAssetLabel::Scene(0).from_asset(format!("assembly://{file}")),
                        ),
                    ),
                    if kind == 4 && run.frames.is_empty() {
                        Visibility::Hidden
                    } else {
                        Visibility::Inherited
                    },
                    root_transform(if kind == 4 {
                        compose(run.world_from_rig, run.pose.unit_pose)
                    } else {
                        run.world_from_rig
                    }),
                ));
            }
        }
        if existing.len() == 3
            && existing.iter().all(|ready| *ready)
            && let (Some(cad), Some(hidden)) = (cad.as_ref(), hidden.as_mut())
        {
            let target = rm_simulator_render::flu_position(run.world_from_rig.translation_m);
            if let Some((index, _)) = cad
                .instances
                .iter()
                .enumerate()
                .filter(|(_, i)| i.file.contains("tech-core"))
                .min_by(|(_, a), (_, b)| {
                    a.transform
                        .translation
                        .distance_squared(target)
                        .total_cmp(&b.transform.translation.distance_squared(target))
                })
            {
                hidden.0.insert(index);
            }
        }
        let pose = views
            .0
            .iter()
            .find(|v| v.owner == run.owner && v.generation == run.generation)
            .map_or(&run.pose, |v| &v.pose);
        let frames = pose.frames(&run.model);
        for (link, mut t) in &mut links {
            if link.owner == run.owner
                && link.generation == run.generation
                && let Some(p) = frames.get(&link.name)
            {
                let next = native(*p);
                if *t != next {
                    *t = next;
                }
            }
        }
    }
    replaced.0.clone_from(&owners.0);
}
#[allow(clippy::type_complexity)]
fn cleanup(
    mut commands: Commands,
    entities: Query<Entity, Or<(With<Panel>, With<Root>)>>,
    mut owners: ResMut<VisualOwners>,
    mut hud: ResMut<HudState>,
    mut replaced: ResMut<rm_simulator_render::team_robot::ReplacedArmOwners>,
    mut hidden: Option<ResMut<rm_simulator_render::cad::HiddenCadInstances>>,
) {
    for e in &entities {
        commands.entity(e).despawn();
    }
    owners.0.clear();
    hud.assembly = false;
    replaced.0.clear();
    if let Some(hidden) = hidden.as_mut() {
        hidden.0.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn persistent_arm_follows_the_same_presented_chassis_as_its_body() {
        let mut field = rm_simulator_world::Field::new(&Default::default()).unwrap();
        let id = field
            .add_chassis(&rm_simulator_world::ChassisPlacement {
                spawn: Pose::at([0., 0., 0.2]),
                team: rm_simulator_world::Team::Red,
                kind: rm_simulator_world::RobotKind::Engineer,
                performance: None,
                config: Default::default(),
            })
            .unwrap();
        let mut appearance =
            crate::scene::chassis_appearance(&field.chassis_snapshot(id).unwrap(), &[], None);
        let presented = Pose::yawed([1., 2., 0.2], 0.7);
        appearance.pose = rm_simulator_render::PoseFlu {
            translation_m: presented.translation_m,
            rotation_wxyz: presented.rotation_wxyz,
        };
        let mut app = App::new();
        app.insert_resource(rm_simulator_render::sync::SceneInput(Some(
            rm_simulator_render::sync::SceneState {
                chassis: vec![appearance],
                ..Default::default()
            },
        )));
        let arm = app
            .world_mut()
            .spawn((
                Root {
                    owner: id,
                    generation: 0,
                    kind: 0,
                    ready: true,
                },
                Transform::default(),
            ))
            .id();
        let core = app
            .world_mut()
            .spawn((
                Root {
                    owner: id,
                    generation: 1,
                    kind: 1,
                    ready: true,
                },
                Transform::default(),
            ))
            .id();
        app.add_systems(Update, align_arms);
        app.update();
        assert_eq!(
            *app.world().get::<Transform>(arm).unwrap(),
            root_transform(presented)
        );
        assert_eq!(
            *app.world().get::<Transform>(core).unwrap(),
            Transform::default()
        );
    }
}
