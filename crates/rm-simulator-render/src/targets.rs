// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-driven source CAD optical materials, with plain panels for non-CAD targets.
use crate::{PoseFlu, TeamColor, apply_pose};
use bevy::prelude::*;

/// Caller-owned appearance of one detector panel.
#[derive(Clone, Debug, PartialEq)]
pub struct TargetAppearance {
    /// Stable caller identity, unique within one input frame.
    pub id: u64,
    /// Detector centre in FLU metres, +x outward.
    pub pose: PoseFlu,
    /// Width and height in metres.
    pub size_m: [f32; 2],
    /// Current physical owner, independent of original field side.
    pub team: TeamColor,
    /// True for a destroyed or inactive detector.
    pub disabled: bool,
    /// Caller-resolved strike flash.
    pub flash: bool,
}
/// A CAD optical primitive whose exact source geometry replaces a broad panel.
#[derive(Component)]
pub(crate) struct CadLight {
    id: u64,
    appearance: Option<(TeamColor, bool, bool)>,
}
impl CadLight {
    pub(crate) fn new(id: u64) -> Self {
        Self {
            id,
            appearance: None,
        }
    }
}
/// Replace every frame; an empty list removes every panel.
#[derive(Resource, Default)]
pub struct TargetInput(pub Vec<TargetAppearance>);
#[derive(Component)]
struct Panel(TargetAppearance);
/// Install the passive panel renderer. Requires mesh and material assets to draw.
pub struct TargetPlugin;
impl Plugin for TargetPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TargetInput>()
            .add_systems(Update, sync.in_set(crate::sync::SceneSyncSet));
    }
}
fn transform(a: &TargetAppearance) -> Transform {
    let mut t = Transform::default();
    apply_pose(&mut t, a.pose);
    t.scale = Vec3::new(a.size_m[0], a.size_m[1], 0.002);
    t
}
fn color(a: &TargetAppearance) -> Color {
    if a.disabled {
        Color::srgb(0.025, 0.025, 0.025)
    } else if a.flash {
        Color::srgb(0.5, 0.5, 0.5)
    } else {
        a.team.light().0
    }
}
fn sync(
    mut commands: Commands,
    input: Res<TargetInput>,
    meshes: Option<ResMut<Assets<Mesh>>>,
    materials: Option<ResMut<Assets<StandardMaterial>>>,
    mut cad_lights: Query<(&mut CadLight, &MeshMaterial3d<StandardMaterial>)>,
    mut panels: Query<(
        Entity,
        &mut Panel,
        &mut Transform,
        &MeshMaterial3d<StandardMaterial>,
    )>,
) {
    let (Some(mut meshes), Some(mut materials)) = (meshes, materials) else {
        return;
    };
    let bound: std::collections::HashSet<_> =
        cad_lights.iter().map(|(light, _)| light.id).collect();
    for (mut light, material) in &mut cad_lights {
        let appearance = input.0.iter().find(|a| a.id == light.id);
        let wanted = appearance.map(|a| (a.team, a.disabled, a.flash));
        if light.appearance != wanted {
            if let Some(mut m) = materials.get_mut(&material.0) {
                m.base_color = appearance
                    .map(color)
                    .unwrap_or(Color::srgb(0.025, 0.025, 0.025));
                m.emissive = if wanted.is_some() {
                    LinearRgba::from(m.base_color) * 0.8
                } else {
                    LinearRgba::BLACK
                };
            }
            light.appearance = wanted;
        }
    }
    for (entity, mut old, mut t, material) in &mut panels {
        if bound.contains(&old.0.id) {
            commands.entity(entity).despawn();
            continue;
        }
        let Some(a) = input.0.iter().find(|a| a.id == old.0.id) else {
            commands.entity(entity).despawn();
            continue;
        };
        if old.0 != *a {
            if old.0.pose != a.pose || old.0.size_m != a.size_m {
                *t = transform(a);
            }
            if (old.0.team, old.0.disabled, old.0.flash) != (a.team, a.disabled, a.flash)
                && let Some(mut m) = materials.get_mut(&material.0)
            {
                m.base_color = color(a);
                m.emissive = LinearRgba::from(color(a)) * 0.8;
            }
            old.0 = a.clone();
        }
    }
    for a in &input.0 {
        if bound.contains(&a.id) || panels.iter().any(|(_, p, _, _)| p.0.id == a.id) {
            continue;
        }
        commands.spawn((
            Panel(a.clone()),
            Mesh3d(meshes.add(Cuboid::new(1., 1., 1.))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color(a),
                emissive: LinearRgba::from(color(a)) * 0.8,
                ..default()
            })),
            transform(a),
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn source_lights_keep_geometry_and_follow_independent_owner_flash_and_disabled_state() {
        let mut app = App::new();
        app.init_resource::<Assets<Mesh>>()
            .init_resource::<Assets<StandardMaterial>>()
            .add_plugins(TargetPlugin);
        let material = app
            .world_mut()
            .resource_mut::<Assets<StandardMaterial>>()
            .add(StandardMaterial::default());
        let other_material = app
            .world_mut()
            .resource_mut::<Assets<StandardMaterial>>()
            .add(StandardMaterial::default());
        let rest = Transform::from_xyz(2., 3., 4.);
        let first = app
            .world_mut()
            .spawn((CadLight::new(42), MeshMaterial3d(material.clone()), rest))
            .id();
        app.world_mut().spawn((
            CadLight::new(43),
            MeshMaterial3d(other_material.clone()),
            rest,
        ));
        let a = TargetAppearance {
            id: 42,
            pose: PoseFlu::default(),
            size_m: [0.1, 0.08],
            team: TeamColor::Red,
            disabled: false,
            flash: false,
        };
        let b = TargetAppearance {
            id: 43,
            team: TeamColor::Blue,
            ..a.clone()
        };
        app.world_mut().resource_mut::<TargetInput>().0 = vec![a.clone(), b.clone()];
        app.update();
        assert_eq!(
            app.world_mut().query::<&Panel>().iter(app.world()).count(),
            0
        );
        assert_eq!(*app.world().get::<Transform>(first).unwrap(), rest);
        assert_eq!(
            app.world()
                .resource::<Assets<StandardMaterial>>()
                .get(&material)
                .unwrap()
                .base_color,
            color(&a)
        );
        assert_eq!(
            app.world()
                .resource::<Assets<StandardMaterial>>()
                .get(&other_material)
                .unwrap()
                .base_color,
            color(&b)
        );
        let mut changed = a;
        changed.team = TeamColor::Blue;
        changed.flash = true;
        app.world_mut().resource_mut::<TargetInput>().0[0] = changed.clone();
        app.update();
        assert_eq!(
            app.world()
                .resource::<Assets<StandardMaterial>>()
                .get(&material)
                .unwrap()
                .base_color,
            color(&changed)
        );
        changed.flash = false;
        changed.disabled = true;
        app.world_mut().resource_mut::<TargetInput>().0[0] = changed.clone();
        app.update();
        assert_eq!(
            app.world()
                .resource::<Assets<StandardMaterial>>()
                .get(&material)
                .unwrap()
                .base_color,
            color(&changed)
        );
        assert_eq!(*app.world().get::<Transform>(first).unwrap(), rest);
        app.world_mut().resource_mut::<TargetInput>().0.clear();
        app.update();
        assert_eq!(
            app.world()
                .resource::<Assets<StandardMaterial>>()
                .get(&material)
                .unwrap()
                .emissive,
            LinearRgba::BLACK
        );
    }

    #[test]
    fn late_source_binding_removes_an_existing_filled_panel() {
        let mut app = App::new();
        app.init_resource::<Assets<Mesh>>()
            .init_resource::<Assets<StandardMaterial>>()
            .add_plugins(TargetPlugin);
        app.world_mut().resource_mut::<TargetInput>().0 = vec![TargetAppearance {
            id: 4,
            pose: PoseFlu::default(),
            size_m: [0.1, 0.08],
            team: TeamColor::Red,
            disabled: false,
            flash: false,
        }];
        app.update();
        assert_eq!(
            app.world_mut().query::<&Panel>().iter(app.world()).count(),
            1
        );
        let material = app
            .world_mut()
            .resource_mut::<Assets<StandardMaterial>>()
            .add(StandardMaterial::default());
        app.world_mut()
            .spawn((CadLight::new(4), MeshMaterial3d(material)));
        app.update();
        assert_eq!(
            app.world_mut().query::<&Panel>().iter(app.world()).count(),
            0
        );
    }
    #[test]
    fn caller_changes_owner_darkness_pose_and_removes_panels_without_rules() {
        let mut app = App::new();
        app.init_resource::<Assets<Mesh>>()
            .init_resource::<Assets<StandardMaterial>>()
            .add_plugins(TargetPlugin);
        let a = TargetAppearance {
            id: 42,
            pose: PoseFlu {
                translation_m: [1., 2., 3.],
                rotation_wxyz: [1., 0., 0., 0.],
            },
            size_m: [0.09, 0.092],
            team: TeamColor::Red,
            disabled: false,
            flash: false,
        };
        app.world_mut().resource_mut::<TargetInput>().0 = vec![a.clone()];
        app.update();
        let entity = app
            .world_mut()
            .query_filtered::<Entity, With<Panel>>()
            .single(app.world())
            .unwrap();
        assert_eq!(
            app.world().get::<Transform>(entity).unwrap().translation,
            crate::flu_position([1., 2., 3.])
        );
        let mut b = a;
        b.team = TeamColor::Blue;
        b.disabled = true;
        b.pose.translation_m[0] = 2.;
        app.world_mut().resource_mut::<TargetInput>().0 = vec![b.clone()];
        app.update();
        let material = &app
            .world()
            .get::<MeshMaterial3d<StandardMaterial>>(entity)
            .unwrap()
            .0;
        assert_eq!(
            app.world()
                .resource::<Assets<StandardMaterial>>()
                .get(material)
                .unwrap()
                .base_color,
            color(&b)
        );
        assert_eq!(
            app.world().get::<Transform>(entity).unwrap().translation,
            crate::flu_position([2., 2., 3.])
        );
        app.world_mut().resource_mut::<TargetInput>().0.clear();
        app.update();
        assert!(
            app.world_mut()
                .query::<&Panel>()
                .iter(app.world())
                .next()
                .is_none()
        );
    }
}
