// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Passive imported team-robot visuals, driven exclusively by caller poses.
//! Source CAD belongs to Zhongzhou Zhulong and Zhejiang University Hello World;
//! source notices and the assembly recipe are recorded in docs/hardware-models.md.
use crate::{
    PoseFlu, apply_pose,
    chassis::RobotModel,
    sync::{SceneInput, SceneSyncSet},
};
use bevy::{prelude::*, world_serialization::WorldInstanceReady};

/// Caller-provided verified robot asset. The app registers its external asset source.
#[derive(Resource)]
pub struct TeamRobotAssets {
    /// glTF path including asset source, without a scene label.
    pub file: String,
    /// Select the bound folding Hero instead of the original Omni assembly.
    pub folding_hero: bool,
    /// Select the source-bound stair Engineer instead of the Omni/Hero package.
    pub stair_heavy: bool,
    /// Source armor surfaces were removed and need complete caller-fitted modules.
    pub replace_armor: bool,
}
impl TeamRobotAssets {
    pub(crate) fn supports(&self, model: RobotModel) -> bool {
        model
            == if self.stair_heavy {
                RobotModel::StairHeavy
            } else if self.folding_hero {
                RobotModel::FoldingHero
            } else {
                RobotModel::TeamCad
            }
    }
}
/// Asynchronous robot loading state, also used by screenshot readiness.
#[derive(Resource, Default)]
pub struct TeamRobotStatus {
    /// Number of currently requested scenes.
    pub requested: usize,
    /// Number with all required motion groups bound.
    pub ready: usize,
    /// First import or binding error, if any.
    pub failed: Option<String>,
}
impl TeamRobotStatus {
    /// True once every currently requested source scene is bound successfully.
    pub fn is_ready(&self) -> bool {
        self.failed.is_none() && self.ready == self.requested
    }
}
/// Render the approved source-derived robot when its asset is supplied.
pub struct TeamRobotVisualsPlugin;
impl Plugin for TeamRobotVisualsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TeamRobotStatus>()
            .add_observer(loaded)
            .add_systems(
                Update,
                (poll_loading, spawn, sync).chain().in_set(SceneSyncSet),
            );
    }
}
#[derive(Component)]
struct RobotRoot(u32);
#[derive(Component)]
struct ReadyRobot;
#[derive(Component)]
struct AssetRoot {
    id: u32,
    owner: Entity,
}
#[derive(Component)]
struct Link {
    id: u32,
    name: String,
    rest: Transform,
}
/// Body reference in the approved GLB: source Y=85 mm above the STEP origin.
const BODY_Y_M: f32 = 0.175_675_83;
/// Camera lens front relative to the source pitch pivot, in Bevy right/up/back metres.
/// Fitted from MVL-HF0628M-6MPE's source envelope; optical center is approximate.
///
/// ```
/// let eye = rm_simulator_render::team_robot::camera_eye();
/// assert!(eye.y > 0. && eye.z < 0.); // above and ahead of the pitch pivot
/// ```
pub fn camera_eye() -> Vec3 {
    Vec3::new(0., 0.0925, -0.048_614)
}
fn q(p: PoseFlu) -> Quat {
    let [w, x, y, z] = p.rotation_wxyz;
    Quat::from_xyzw(x as f32, y as f32, z as f32, w as f32)
}
#[allow(clippy::too_many_arguments)]
fn spawn(
    input: Res<SceneInput>,
    assets: Option<Res<TeamRobotAssets>>,
    server: Option<Res<AssetServer>>,
    roots: Query<(Entity, &RobotRoot, Option<&ReadyRobot>)>,
    mut commands: Commands,
    mut status: ResMut<TeamRobotStatus>,
    config: Res<crate::chassis::Config>,
    atlas: Res<crate::armor::ArmorAtlas>,
    diffuser: Res<crate::armor::DiffuserProfile>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let Some(assets) = assets else { return };
    let Some(server) = server else { return };
    let wanted: Vec<_> = input
        .0
        .as_ref()
        .map(|s| {
            s.chassis
                .iter()
                .filter(|c| assets.supports(c.model))
                .collect()
        })
        .unwrap_or_default();
    status.requested = wanted.len();
    status.ready = roots
        .iter()
        .filter(|(_, r, ready)| ready.is_some() && wanted.iter().any(|c| c.id == r.0))
        .count();
    for (e, r, _) in &roots {
        if !wanted.iter().any(|c| c.id == r.0) {
            commands.entity(e).despawn();
        }
    }
    for c in wanted {
        if roots.iter().any(|(_, r, _)| r.0 == c.id) {
            continue;
        }
        let mut transform = Transform::default();
        apply_pose(&mut transform, c.pose);
        let owner = commands
            .spawn((RobotRoot(c.id), transform, Visibility::Inherited))
            .id();
        commands.entity(owner).with_children(|parent| {
            crate::chassis::imported_armor(
                parent,
                c,
                &config,
                assets.replace_armor,
                &atlas,
                &diffuser,
                &mut meshes,
                &mut materials,
            );
            parent.spawn((
                AssetRoot { id: c.id, owner },
                WorldAssetRoot(
                    server.load(GltfAssetLabel::Scene(0).from_asset(assets.file.clone())),
                ),
                Transform::from_xyz(
                    0.,
                    -if assets.stair_heavy {
                        0.
                    } else if assets.folding_hero {
                        0.1684552
                    } else {
                        BODY_Y_M
                    },
                    0.,
                )
                .with_rotation(Quat::from_rotation_y(std::f32::consts::PI)),
                Visibility::Inherited,
            ));
        });
    }
}
// Explicit presentation policy for this verified prototype: unstyled regions and its
// uniform pale blue CAD tint. Preserve white paint and every other source color.
fn use_graphite(extras: &str) -> bool {
    serde_json::from_str::<serde_json::Value>(extras).is_ok_and(|v| {
        if v.get("display_override").is_some() {
            return false;
        }
        (v["step_colour_origin"] == "fallback"
            && v.get("step_colour_srgb").is_some_and(|c| c.is_null()))
            || (v["step_colour_origin"] == "source"
                && v["step_colour_srgb"] == serde_json::json!([0.7922, 0.8196, 0.9333]))
    })
}
#[allow(clippy::too_many_arguments)]
fn loaded(
    event: On<WorldInstanceReady>,
    roots: Query<&AssetRoot>,
    children: Query<&Children>,
    names: Query<(&Name, &Transform)>,
    mut commands: Commands,
    mut status: ResMut<TeamRobotStatus>,
    primitives: Query<(
        &bevy::gltf::GltfMaterialExtras,
        &MeshMaterial3d<StandardMaterial>,
    )>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let Ok(root) = roots.get(event.entity) else {
        return;
    };
    let mut bound = std::collections::HashSet::new();
    let mut replacements = std::collections::HashMap::new();
    for e in children.iter_descendants(event.entity) {
        if let Ok((extras, handle)) = primitives.get(e)
            && use_graphite(&extras.value)
        {
            let replacement = replacements.entry(handle.id()).or_insert_with(|| {
                let mut material = materials.get(handle.id()).cloned().unwrap_or_default();
                // User-selected graphite appearance; never inferred as source CAD paint.
                material.base_color = Color::srgb(0.095, 0.10, 0.11);
                material.metallic = 0.1;
                material.perceptual_roughness = 0.45;
                materials.add(material)
            });
            commands
                .entity(e)
                .insert(MeshMaterial3d(replacement.clone()));
        }
        let Ok((name, transform)) = names.get(e) else {
            continue;
        };
        if let Some(name) = name.as_str().strip_prefix("link/") {
            bound.insert(name.to_owned());
            commands.entity(e).insert(Link {
                id: root.id,
                name: name.into(),
                rest: *transform,
            });
        }
    }
    let required = if bound.contains("front") {
        vec![
            "body", "front", "rear", "coupler", "guide", "slider", "wheel-0", "wheel-1", "wheel-2",
            "wheel-3",
        ]
    } else if bound.contains("fold") {
        vec![
            "body", "yaw", "fold", "pitch", "wheel-0", "wheel-1", "wheel-2", "wheel-3",
        ]
    } else {
        vec![
            "body",
            "yaw",
            "pitch",
            "pitch-crank",
            "pitch-rod",
            "wheel-0",
            "wheel-1",
            "wheel-2",
            "wheel-3",
        ]
    };
    if required.iter().all(|s| bound.contains(*s)) {
        commands.entity(root.owner).insert(ReadyRobot);
        info!(
            chassis = root.id,
            "team robot scene ready: required motion groups bound"
        );
    } else {
        status.failed = Some("team robot asset lacks required motion groups".into());
    }
}
fn poll_loading(
    server: Option<Res<AssetServer>>,
    roots: Query<&WorldAssetRoot, With<AssetRoot>>,
    mut status: ResMut<TeamRobotStatus>,
) {
    let Some(server) = server else { return };
    for root in &roots {
        if let bevy::asset::LoadState::Failed(error) = server.load_state(root.0.id()) {
            status.failed = Some(format!("team robot scene: {error}"));
        }
    }
}
fn link_transform(link: &Link, c: &crate::sync::ChassisAppearance) -> Transform {
    if c.model == RobotModel::StairHeavy
        && let Some((_, p)) = c.source_joints.iter().find(|(name, _)| name == &link.name)
    {
        let [w, x, y, z] = p.rotation_wxyz;
        return Transform::from_translation(Vec3::new(
            p.translation_m[1] as f32,
            p.translation_m[2] as f32,
            p.translation_m[0] as f32,
        ))
        .with_rotation(Quat::from_xyzw(y as f32, z as f32, x as f32, w as f32));
    }
    let mut t = link.rest;
    let relative = q(c.pose).inverse() * q(c.yaw_stage);
    let forward = relative * Vec3::X;
    let yaw = forward.y.atan2(forward.x);
    let direction = (q(c.pose).inverse() * q(c.turret)) * Vec3::X;
    let pitch = direction.z.clamp(-1., 1.).asin();
    match link.name.as_str() {
        "yaw" => t.rotation = Quat::from_rotation_y(yaw),
        "fold" => t.rotation = Quat::from_rotation_x(c.fold_angle_rad),
        "pitch" if c.model == RobotModel::FoldingHero => {
            t.rotation = Quat::from_rotation_x(-pitch - c.fold_angle_rad)
        }
        "pitch" | "pitch-crank" => t.rotation = Quat::from_rotation_x(-pitch),
        "pitch-rod" => t.rotation = Quat::from_rotation_x(pitch),
        name if name.starts_with("wheel-") => {
            if let Some(w) = name
                .strip_prefix("wheel-")
                .and_then(|v| v.parse::<usize>().ok())
                .and_then(|i| c.wheels.get(i))
            {
                let body = q(c.pose);
                let local = body.inverse()
                    * Vec3::from_array(std::array::from_fn(|i| {
                        (w.pose.translation_m[i] - c.pose.translation_m[i]) as f32
                    }));
                t.translation = Vec3::new(
                    local.y,
                    local.z
                        + if c.model == RobotModel::StairHeavy {
                            0.
                        } else if c.model == RobotModel::FoldingHero {
                            0.1684552
                        } else {
                            BODY_Y_M
                        },
                    local.x,
                );
                let angle = local.y.atan2(local.x) - std::f32::consts::FRAC_PI_2;
                let spin = (body.inverse() * q(w.pose))
                    * if matches!(c.model, RobotModel::FoldingHero | RobotModel::StairHeavy) {
                        Quat::IDENTITY
                    } else {
                        Quat::from_rotation_z(-angle)
                    };
                t.rotation = Quat::from_xyzw(spin.y, spin.z, spin.x, spin.w).normalize();
            }
        }
        _ => {}
    }
    t
}
fn sync(
    input: Res<SceneInput>,
    mut roots: Query<(&RobotRoot, &mut Transform), Without<Link>>,
    mut links: Query<(&Link, &mut Transform), Without<RobotRoot>>,
) {
    let Some(scene) = &input.0 else { return };
    for (root, mut t) in &mut roots {
        if let Some(c) = scene.chassis.iter().find(|c| c.id == root.0) {
            let mut next = *t;
            apply_pose(&mut next, c.pose);
            t.set_if_neq(next);
        }
    }
    for (link, mut t) in &mut links {
        if let Some(c) = scene.chassis.iter().find(|c| c.id == link.id) {
            t.set_if_neq(link_transform(link, c));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        TeamColor,
        sync::{ArmorAppearance, ChassisAppearance, WheelAppearance},
    };
    fn appearance(id: u32, x_m: f64) -> ChassisAppearance {
        let h = std::f64::consts::FRAC_1_SQRT_2;
        ChassisAppearance {
            source_joints: vec![],
            model: RobotModel::Omni,
            armor_pattern: crate::armor::ArmorPattern::Three,
            mecanum: false,
            hp_fraction: 1.0,
            id,
            team: TeamColor::Red,
            pose: PoseFlu {
                translation_m: [x_m, 2.0, 0.15],
                rotation_wxyz: [h, 0.0, 0.0, h],
            },
            body_half_m: [0.26, 0.26, 0.05],
            body_shell_visible: true,
            fold_angle_rad: 0.,
            fold_collision_parts: vec![],
            frame_beam_step_m: 0.0,
            body_outline_m: vec![],
            bumper_prisms_m: vec![],
            bumper_base_prisms: 0,
            wheels: vec![
                WheelAppearance {
                    pose: PoseFlu::default(),
                    suspension_anchor_m: [0.0, 0.0, 0.2],
                    suspension_lower_m: [0.0, 0.0, 0.1],
                    radius_m: 0.07,
                    width_m: 0.04,
                },
                WheelAppearance {
                    suspension_anchor_m: [0.0, 0.0, 0.2],
                    suspension_lower_m: [0.0, 0.0, 0.1],
                    pose: PoseFlu {
                        translation_m: [x_m + 0.2, 2.2, 0.1],
                        rotation_wxyz: [1.0, 0.0, 0.0, 0.0],
                    },
                    radius_m: 0.07,
                    width_m: 0.04,
                },
            ],
            armor: vec![
                ArmorAppearance {
                    optics: None,
                    plate_visible: true,
                    bracket_prisms_m: vec![],
                    local: PoseFlu {
                        translation_m: [0.28, 0.0, 0.015],
                        rotation_wxyz: [1.0, 0.0, 0.0, 0.0],
                    },
                    hit_flash: false,
                },
                ArmorAppearance {
                    optics: None,
                    plate_visible: true,
                    bracket_prisms_m: vec![],
                    local: PoseFlu {
                        translation_m: [0.0, 0.28, 0.015],
                        rotation_wxyz: [h, 0.0, 0.0, h],
                    },
                    hit_flash: true,
                },
            ],
            yaw_stage: PoseFlu {
                translation_m: [x_m, 2.0, 0.4],
                rotation_wxyz: [h, 0.0, 0.0, h],
            },
            turret: PoseFlu {
                translation_m: [x_m, 2.0, 0.4],
                rotation_wxyz: [1.0, 0.0, 0.0, 0.0],
            },
            turret_half_m: [0.06, 0.06, 0.06],
            pivot_above_body_m: 0.15,
            barrel_length_m: 0.25,
            defeated: false,
        }
    }

    #[test]
    fn graphite_policy_preserves_paint_and_requires_explicit_provenance() {
        assert!(use_graphite(
            r#"{"step_colour_origin":"fallback","step_colour_srgb":null}"#
        ));
        assert!(!use_graphite(
            r#"{"step_colour_origin":"source","step_colour_srgb":[1,1,1]}"#
        ));
        assert!(!use_graphite(
            r#"{"step_colour_origin":"source","step_colour_srgb":[0.1,0.1,0.1]}"#
        ));
        assert!(use_graphite(
            r#"{"step_colour_origin":"source","step_colour_srgb":[0.7922,0.8196,0.9333]}"#
        ));
        assert!(!use_graphite("{}"));
        assert!(!use_graphite("invalid"));
    }
    fn pose(rotation: Quat, position: Vec3) -> PoseFlu {
        PoseFlu {
            translation_m: position.to_array().map(f64::from),
            rotation_wxyz: [rotation.w, rotation.x, rotation.y, rotation.z].map(f64::from),
        }
    }
    #[test]
    fn links_follow_body_relative_aim_and_independent_axle_motion() {
        let mut c = appearance(3, 5.);
        let body = Quat::from_euler(EulerRot::ZYX, 1.1, 0.2, -0.15);
        let origin = Vec3::new(5., 2., 0.3);
        c.pose = pose(body, origin);
        c.yaw_stage = pose(body * Quat::from_rotation_z(0.7), origin);
        c.turret = pose(
            body * Quat::from_rotation_z(0.7) * Quat::from_rotation_y(-0.3),
            origin,
        );
        for (name, expected) in [
            ("yaw", Quat::from_rotation_y(0.7)),
            ("pitch", Quat::from_rotation_x(-0.3)),
            ("pitch-crank", Quat::from_rotation_x(-0.3)),
            ("pitch-rod", Quat::from_rotation_x(0.3)),
        ] {
            let rest = Transform::from_xyz(0.02, 0.3, -0.04);
            let got = link_transform(
                &Link {
                    id: 3,
                    name: name.into(),
                    rest,
                },
                &c,
            );
            assert!(got.rotation.abs_diff_eq(expected, 1e-5));
            assert_eq!(got.translation, rest.translation);
        }
        for (i, (x, y, heave, spin)) in [(0.178, 0.178, -0.07, 0.4), (0.178, -0.178, -0.09, -0.8)]
            .into_iter()
            .enumerate()
        {
            let hub = Vec3::new(x, y, heave);
            let angle = y.atan2(x) - std::f32::consts::FRAC_PI_2;
            c.wheels[i].pose = pose(
                body * Quat::from_rotation_z(angle) * Quat::from_rotation_y(-spin),
                origin + body * hub,
            );
            let got = link_transform(
                &Link {
                    id: 3,
                    name: format!("wheel-{i}"),
                    rest: Transform::default(),
                },
                &c,
            );
            assert!(
                got.translation
                    .abs_diff_eq(Vec3::new(y, heave + BODY_Y_M, x), 1e-5)
            );
            let local = Quat::from_rotation_z(angle)
                * Quat::from_rotation_y(-spin)
                * Quat::from_rotation_z(-angle);
            let expected = Quat::from_xyzw(local.y, local.z, local.x, local.w);
            assert!(got.rotation.abs_diff_eq(expected, 1e-5));
        }
    }
    #[test]
    fn stair_links_use_caller_frames_and_wheels_keep_body_local_hubs() {
        let mut c = appearance(3, 5.);
        c.model = RobotModel::StairHeavy;
        let p = pose(Quat::from_rotation_y(0.7), Vec3::new(0.12, 0., -0.03));
        c.source_joints = vec![("front".into(), p)];
        let got = link_transform(
            &Link {
                id: 3,
                name: "front".into(),
                rest: Transform::IDENTITY,
            },
            &c,
        );
        assert!(
            got.translation
                .abs_diff_eq(Vec3::new(0., -0.03, 0.12), 1e-6)
        );
        assert!(got.rotation.abs_diff_eq(Quat::from_rotation_x(0.7), 1e-6));
        let hub = Vec3::new(0.11, 0.2083, -0.20);
        let body = q(c.pose);
        let origin = Vec3::from_array(c.pose.translation_m.map(|x| x as f32));
        c.wheels[0].pose = pose(body, origin + body * hub);
        let wheel = link_transform(
            &Link {
                id: 3,
                name: "wheel-0".into(),
                rest: Transform::IDENTITY,
            },
            &c,
        );
        assert!(
            wheel
                .translation
                .abs_diff_eq(Vec3::new(hub.y, hub.z, hub.x), 1e-5)
        );
        assert!(wheel.rotation.abs_diff_eq(Quat::IDENTITY, 1e-5));
    }
}
