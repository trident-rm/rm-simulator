// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Move the original source Unit meshes from authoritative payload poses.
use crate::{loading::Screen, session::Session};
use bevy::{math::Affine3A, prelude::*};

#[derive(Component)]
struct SourceVisual {
    id: u32,
    rest_world: Affine3A,
}
pub(crate) struct SupplyPlugin;
impl Plugin for SupplyPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (bind, sync)
                .chain()
                .after(crate::session::advance_world)
                .run_if(in_state(Screen::InMatch)),
        );
    }
}
fn bind(
    mut commands: Commands,
    session: Res<Session>,
    nodes: Query<(Entity, &Name, &GlobalTransform), Without<SourceVisual>>,
) {
    let Some(restore) = session.snapshot.restore.as_ref() else {
        return;
    };
    for (entity, name, global) in &nodes {
        // Newly instantiated nodes need one ordinary hierarchy propagation.
        if global.translation().length_squared() < 1. {
            continue;
        }
        if let Some(unit) = restore.energy_units.iter().find(|u| {
            u.supply.as_ref().and_then(|s| s.visual_node.as_deref()) == Some(name.as_str())
        }) {
            commands.entity(entity).insert(SourceVisual {
                id: unit.physical.id,
                rest_world: global.affine(),
            });
        }
    }
}
fn sync(
    session: Res<Session>,
    mut nodes: Query<(&SourceVisual, &ChildOf, &mut Transform)>,
    parents: Query<&GlobalTransform>,
) {
    let Some(restore) = session.snapshot.restore.as_ref() else {
        return;
    };
    for (visual, parent, mut transform) in &mut nodes {
        let Some(unit) = restore
            .energy_units
            .iter()
            .find(|u| u.physical.id == visual.id)
        else {
            continue;
        };
        let Some(mount) = &unit.supply else {
            continue;
        };
        let Ok(parent) = parents.get(parent.parent()) else {
            continue;
        };
        let from = crate::frames::transform_of(&mount.pose).compute_affine();
        let to = crate::frames::transform_of(&unit.physical.pose).compute_affine();
        let local = parent.affine().inverse() * to * from.inverse() * visual.rest_world;
        let next = Transform::from_matrix(Mat4::from(local));
        if *transform != next {
            *transform = next;
        }
    }
}
