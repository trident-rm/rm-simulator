// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Workbench presentation around the same Bevy-free rig the host runs.
// Compile the host-owned internal implementation here to keep the standalone
// diagnostic's private inspection tests without publishing its internals.
#[path = "../../../rm-simulator-server/src/assembly_rig/mod.rs"]
#[allow(dead_code, unused_imports)]
mod rig;
use bevy::prelude::*;
pub use rig::{AssemblyPlan, PlanningFailure, Rig};
use rm_simulator_physics::Pose;
use rm_simulator_render::{PoseFlu, apply_pose};
use std::{collections::BTreeMap, path::PathBuf};
#[derive(Component)]
pub(super) struct UnitRoot;
#[derive(Resource, Default)]
pub(super) struct BoundGroups(BTreeMap<String, Entity>);
impl BoundGroups {
    pub fn ready(&self) -> bool {
        let count = |name: &str| {
            self.0
                .keys()
                .filter(|key| key.split('@').next() == Some(name))
                .count()
        };
        [
            "engineer/body",
            "engineer/camera-lift",
            "engineer/uwb",
            "engineer/jaw-left",
            "engineer/jaw-right",
            "unit/unit",
        ]
        .iter()
        .all(|name| count(name) == 1)
            && (1..=6)
                .all(|i| count(&format!("engineer/j{i}")) == 1 && count(&format!("core/j{i}")) == 3)
            && ["core/body", "core/lift", "core/p", "core/q"]
                .iter()
                .all(|name| count(name) == 3)
    }
}
/// Register this asset source before DefaultPlugins.
pub fn register(app: &mut App, root: PathBuf) {
    use bevy::asset::io::{AssetSourceBuilder, file::FileAssetReader};
    app.register_asset_source(
        "assembly",
        AssetSourceBuilder::new(move || Box::new(FileAssetReader::new(root.clone()))),
    );
    app.init_resource::<BoundGroups>();
}
pub fn spawn(commands: &mut Commands, server: &AssetServer, rig: &Rig) {
    if rig.has_area() {
        commands.spawn((
            WorldAssetRoot(
                server.load(GltfAssetLabel::Scene(0).from_asset("assembly-area://area.glb")),
            ),
            Transform::from_rotation(Quat::from_rotation_y(std::f32::consts::PI)),
        ));
    }

    for (file, body) in [
        ("engineer.glb", rig.engineer_body),
        ("core-base.glb", Pose::default()),
        ("core-arm.glb", Pose::default()),
        ("core-head.glb", Pose::default()),
    ] {
        let mut t = Transform::default();
        apply_pose(
            &mut t,
            PoseFlu {
                translation_m: body.translation_m,
                rotation_wxyz: body.rotation_wxyz,
            },
        );
        t.rotation *= Quat::from_rotation_y(std::f32::consts::PI);
        commands.spawn((
            WorldAssetRoot(
                server.load(GltfAssetLabel::Scene(0).from_asset(format!("assembly://{file}"))),
            ),
            t,
        ));
    }
    commands.spawn((
        UnitRoot,
        WorldAssetRoot(server.load(GltfAssetLabel::Scene(0).from_asset("assembly://unit.glb"))),
        Transform::default(),
    ));
}
pub fn sync(
    w: Res<super::Workbench>,
    mut groups: ResMut<BoundGroups>,
    names: Query<(Entity, &Name), Added<Name>>,
    mut transforms: Query<&mut Transform>,
    unit: Query<Entity, With<UnitRoot>>,
) {
    let Some(rig) = &w.cad else {
        return;
    };
    // A frame may occur in several component GLBs, so bind by entity, not just name.
    for (entity, name) in &names {
        if let Some(key) = name.as_str().strip_prefix("link/") {
            groups
                .0
                .insert(format!("{}@{}", key, entity.to_bits()), entity);
        }
    }
    let frames = rig.frames();
    for (name, entity) in &groups.0 {
        let key = name.split('@').next().unwrap();
        let Some(p) = frames.get(key) else {
            continue;
        };
        if let Ok(mut t) = transforms.get_mut(*entity) {
            let next = native_transform(*p);
            if *t != next {
                *t = next;
            }
        }
    }
    for entity in &unit {
        if let Ok(mut t) = transforms.get_mut(entity) {
            let mut next = Transform::default();
            apply_pose(
                &mut next,
                PoseFlu {
                    translation_m: rig.unit_pose.translation_m,
                    rotation_wxyz: rig.unit_pose.rotation_wxyz,
                },
            );
            next.rotation *= Quat::from_rotation_y(std::f32::consts::PI);
            if *t != next {
                *t = next;
            }
        }
    }
}

fn native_transform(p: Pose) -> Transform {
    let [w, x, y, z] = p.rotation_wxyz;
    let q = Quat::from_xyzw(x as f32, y as f32, z as f32, w as f32);
    // Native glTF is left/up/forward; the scene root supplies its half-turn.
    let b = Quat::from_mat3(&Mat3::from_cols(Vec3::Z, Vec3::X, Vec3::Y));
    Transform::from_translation(Vec3::new(
        p.translation_m[1] as f32,
        p.translation_m[2] as f32,
        p.translation_m[0] as f32,
    ))
    .with_rotation(b * q * b.inverse())
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DQuat, EulerRot};
    use rig::{pose, position, quaternion};
    #[test]
    fn source_group_hierarchy_matches_independent_world_flu_coordinates() {
        let body = pose(
            glam::DVec3::new(1.2, -0.7, 0.132),
            DQuat::from_euler(EulerRot::ZYX, 0.4, 0.2, -0.1),
        );
        let frame = pose(
            glam::DVec3::new(0.3, 0.1, 0.7),
            DQuat::from_euler(EulerRot::ZYX, -0.7, 0.9, 0.6),
        );
        let mut parent = Transform::default();
        apply_pose(
            &mut parent,
            PoseFlu {
                translation_m: body.translation_m,
                rotation_wxyz: body.rotation_wxyz,
            },
        );
        parent.rotation *= Quat::from_rotation_y(std::f32::consts::PI);
        let child = native_transform(frame);
        for native in [Vec3::new(0.1, 0.2, 0.3), Vec3::new(-0.4, 0.3, -0.2)] {
            let world = parent.transform_point(child.transform_point(native));
            let local = glam::DVec3::new(native.z as f64, native.x as f64, native.y as f64);
            let expected =
                position(body) + quaternion(body) * (position(frame) + quaternion(frame) * local);
            assert!(world.distance(rm_simulator_render::flu_position(expected.to_array())) < 1e-6);
        }
    }
}
