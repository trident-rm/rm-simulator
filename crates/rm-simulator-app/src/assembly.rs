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
            .add_systems(OnEnter(Screen::InMatch), panel)
            .add_systems(OnExit(Screen::InMatch), cleanup)
            .add_observer(loaded)
            .add_systems(
                Update,
                (toggle, update_panel, visuals)
                    .chain()
                    .after(crate::session::advance_world)
                    .before(crate::scene::publish_scene)
                    .run_if(in_state(Screen::InMatch)),
            )
            .add_systems(
                Update,
                align_arms
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
    commands.spawn((Panel,Node{position_type:PositionType::Absolute,right:px(18),top:px(160),width:px(330),padding:UiRect::all(px(12)),flex_direction:FlexDirection::Column,row_gap:px(8),..default()},BackgroundColor(Color::srgba(0.025,0.04,0.055,0.94)),Visibility::Hidden,ZIndex(80)))
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
            p.spawn((Text::new("Driving cancels the operation.\nSearch is asynchronous; actuation uses game time."),TextFont{font_size:FontSize::Px(12.),..default()}));
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
        && session.own_chassis().is_some_and(|c| {
            c.config
                .articulation
                .as_ref()
                .is_some_and(|a| a.profile == "pie-tunnel-heavy-v1")
        })
    {
        hud.assembly = !hud.assembly;
        hud.consumed = true;
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
    let (message, progress) = if let Some(r) = run {
        let progress = match r.phase {
            Phase::Positioning => r.positioning.as_ref().map_or(0., |p| p.progress() as f32),
            Phase::Solving | Phase::SolvingTwist => r.checks as f32 / r.budget.max(1) as f32,
            Phase::Actuating | Phase::Completed => r.cursor as f32 / r.frames.len().max(1) as f32,
            _ => 0.,
        };
        let label = if r.phase == Phase::Positioning {
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
    assets: Option<Res<Assets>>,
    server: Res<AssetServer>,
    mut roots: Query<(Entity, &Root, &mut Transform)>,
    mut links: Query<(&Link, &mut Transform), Without<Root>>,
    mut owners: ResMut<VisualOwners>,
    mut unit_visibility: Query<&mut Visibility>,
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
    for (entity, root, mut t) in &mut roots {
        let pose = if root.kind == 0 {
            arms.iter()
                .find(|a| a.owner == root.owner)
                .and_then(|_| session.snapshot.chassis.iter().find(|c| c.id == root.owner))
                .map(|c| c.pose)
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
        if root.kind == 4 {
            let shown = visible.iter().any(|r| {
                r.owner == root.owner && r.generation == root.generation && !r.frames.is_empty()
            });
            if let Ok(mut visibility) = unit_visibility.get_mut(entity) {
                visibility.set_if_neq(if shown {
                    Visibility::Inherited
                } else {
                    Visibility::Hidden
                });
            }
        }
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
        let frames = arm.frames();
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
    for run in visible {
        let existing: Vec<_> = roots
            .iter()
            .filter(|(_, r, _)| {
                r.kind != 0 && r.owner == run.owner && r.generation == run.generation
            })
            .map(|(_, r, _)| r.ready)
            .collect();
        if existing.is_empty() {
            for (kind, file) in [
                (1, "core-base.glb"),
                (2, "core-arm.glb"),
                (3, "core-head.glb"),
                (4, "unit.glb"),
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
        if existing.len() == 4
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
        let frames = run.pose.frames(&run.model);
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
