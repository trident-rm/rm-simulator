// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Sparse visual publication with caller-owned identities and shared assets.
use super::{PoseFlu, apply_pose};
use bevy::{prelude::*, transform::TransformSystems, world_serialization::WorldAsset};
use std::collections::HashMap;

/// Caller-owned visual identity, independent of physics and Bevy entity IDs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VisualId(pub u64);

#[derive(Clone, Copy)]
enum Update {
    Pose(Entity, PoseFlu),
    Visibility(Entity, bool),
}

/// Retained visual roots and sparse pending changes; owns no simulation state.
///
/// Installation clones only shared asset handles. Pose/visibility publication
/// reuses a buffer, whose capacity survives every update and reset. Omitted
/// visuals retain their last pose. Duplicate IDs replace and despawn the old
/// root (including children). Queued updates address entity generations, so
/// removing and reusing a caller ID cannot apply an old update to its successor.
///
/// ```
/// use bevy::prelude::*;
/// use rm_simulator_render::generic::{PoseFlu, VisualId, VisualPlugin, Visuals};
/// let mut app = App::new();
/// app.add_plugins(VisualPlugin).add_systems(Startup,
///     |mut commands: Commands, mut visuals: ResMut<Visuals>| {
///         // Real callers pass handles from their shared Assets collections.
///         visuals.spawn_mesh(&mut commands, VisualId(1), Handle::default(),
///             Handle::default(), PoseFlu::default(), Vec3::ONE);
///         visuals.set_pose(VisualId(1), PoseFlu {
///             translation_m: [1.0, 0.0, 0.0], ..default()
///         });
///     });
/// app.update();
/// let entity = app.world().resource::<Visuals>().entity(VisualId(1)).unwrap();
/// assert_eq!(app.world().get::<Transform>(entity).unwrap().translation, Vec3::NEG_Z);
/// ```
#[derive(Resource, Default)]
pub struct Visuals {
    roots: HashMap<VisualId, Entity>,
    updates: Vec<Update>,
}

impl Visuals {
    /// Reserve space for additional roots and pending changes before publication.
    pub fn reserve(&mut self, additional_visuals: usize, additional_updates: usize) {
        self.roots.reserve(additional_visuals);
        self.updates.reserve(additional_updates);
    }

    /// Install a mesh instance, sharing the supplied mesh and material assets.
    ///
    /// Scale is in Bevy local axes and survives subsequent FLU pose updates.
    /// Replacing an ID despawns its previous root and descendants.
    #[allow(clippy::too_many_arguments)]
    pub fn spawn_mesh(
        &mut self,
        commands: &mut Commands,
        id: VisualId,
        mesh: Handle<Mesh>,
        material: Handle<StandardMaterial>,
        pose: PoseFlu,
        scale: Vec3,
    ) -> Entity {
        self.remove(commands, id);
        let entity = commands
            .spawn((
                Mesh3d(mesh),
                MeshMaterial3d(material),
                transform(pose, scale),
                Visibility::Inherited,
            ))
            .id();
        self.roots.insert(id, entity);
        entity
    }

    /// Install an imported glTF scene instance using its shared scene handle.
    ///
    /// The scene is instantiated by Bevy's WorldSerializationPlugin when loaded. Its authored
    /// mesh/material assets and child transforms stay intact. Pose/scale apply
    /// to the root; scale is in Bevy local axes. No asset loading occurs here.
    pub fn spawn_scene(
        &mut self,
        commands: &mut Commands,
        id: VisualId,
        scene: Handle<WorldAsset>,
        pose: PoseFlu,
        scale: Vec3,
    ) -> Entity {
        self.remove(commands, id);
        let entity = commands
            .spawn((
                WorldAssetRoot(scene),
                transform(pose, scale),
                Visibility::Inherited,
            ))
            .id();
        self.roots.insert(id, entity);
        entity
    }

    /// Queue an FLU pose for an installed ID; false if unknown.
    ///
    /// The caller must supply finite translation and a normalized quaternion,
    /// as for [`apply_pose`]; inputs are not validated here.
    /// Multiple publications apply in order. An externally despawned root is
    /// pruned at the next visual update; queued writes to it are ignored.
    pub fn set_pose(&mut self, id: VisualId, pose: PoseFlu) -> bool {
        let Some(&entity) = self.roots.get(&id) else {
            return false;
        };
        self.updates.push(Update::Pose(entity, pose));
        true
    }

    /// Queue root visibility; true inherits visibility, false hides descendants.
    /// Returns false for unknown IDs. Authored child visibility is preserved.
    pub fn set_visibility(&mut self, id: VisualId, visible: bool) -> bool {
        let Some(&entity) = self.roots.get(&id) else {
            return false;
        };
        self.updates.push(Update::Visibility(entity, visible));
        true
    }

    /// Return the installed root, which may await deferred spawn/despawn commands.
    /// External despawns are reflected after the next visual update.
    pub fn entity(&self, id: VisualId) -> Option<Entity> {
        self.roots.get(&id).copied()
    }

    /// Remove an ID and despawn its root and descendants; false if unknown.
    /// Removing a root already despawned externally is harmless.
    pub fn remove(&mut self, commands: &mut Commands, id: VisualId) -> bool {
        let Some(entity) = self.roots.remove(&id) else {
            return false;
        };
        commands.entity(entity).try_despawn();
        true
    }

    /// Despawn every installed root and discard pending writes, retaining storage.
    pub fn clear(&mut self, commands: &mut Commands) {
        for (_, entity) in self.roots.drain() {
            commands.entity(entity).try_despawn();
        }
        self.updates.clear();
    }

    /// Number of installed roots; external despawns are pruned during updates.
    pub fn len(&self) -> usize {
        self.roots.len()
    }

    /// Whether no roots are installed.
    pub fn is_empty(&self) -> bool {
        self.roots.is_empty()
    }

    /// Retained sparse update capacity, useful for sizing and allocation checks.
    pub fn update_capacity(&self) -> usize {
        self.updates.capacity()
    }
}

fn transform(pose: PoseFlu, scale: Vec3) -> Transform {
    let mut transform = Transform::from_scale(scale);
    apply_pose(&mut transform, pose);
    transform
}

/// Applies sparse caller publications before Bevy propagates root transforms.
/// Publish in Update, or order PostUpdate publishers before this set.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct VisualUpdateSystems;

/// Install the shared visual registry and conditional sparse update system.
/// Requires no renderer, asset loader, clock, host or game plugins.
/// Each frame checks all installed roots for external despawns, then drains
/// pending writes: O(roots + updates), with no steady storage allocation once
/// the caller has reserved enough update capacity.
pub struct VisualPlugin;
impl Plugin for VisualPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Visuals>().add_systems(
            PostUpdate,
            update_visuals
                .in_set(VisualUpdateSystems)
                .before(TransformSystems::Propagate),
        );
    }
}

fn update_visuals(
    mut visuals: ResMut<Visuals>,
    mut transforms: Query<&mut Transform>,
    mut visibility: Query<&mut Visibility>,
    entities: Query<()>,
) {
    visuals.roots.retain(|_, entity| entities.contains(*entity));
    for update in visuals.updates.drain(..) {
        match update {
            Update::Pose(entity, pose) => {
                if let Ok(mut current) = transforms.get_mut(entity) {
                    let next = transform(pose, current.scale);
                    if *current != next {
                        *current = next;
                    }
                }
            }
            Update::Visibility(entity, visible) => {
                if let Ok(mut current) = visibility.get_mut(entity) {
                    let next = if visible {
                        Visibility::Inherited
                    } else {
                        Visibility::Hidden
                    };
                    if *current != next {
                        *current = next;
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn install(app: &mut App, id: u64, pose: PoseFlu) -> Entity {
        let entity = app
            .world_mut()
            .resource_scope(|world, mut visuals: Mut<Visuals>| {
                visuals.spawn_mesh(
                    &mut world.commands(),
                    VisualId(id),
                    Handle::default(),
                    Handle::default(),
                    pose,
                    Vec3::new(2.0, 3.0, 4.0),
                )
            });
        app.world_mut().flush();
        entity
    }

    fn pose(x: f64) -> PoseFlu {
        PoseFlu {
            translation_m: [x, 2.0, 3.0],
            ..default()
        }
    }

    #[test]
    fn sparse_pose_preserves_flu_scale_handles_and_unchanged_component_ticks() {
        #[derive(Resource, Default)]
        struct Changes(usize);
        type VisualChanges = Or<(
            Changed<Transform>,
            Changed<Visibility>,
            Changed<MeshMaterial3d<StandardMaterial>>,
        )>;
        fn count(query: Query<Entity, VisualChanges>, mut changes: ResMut<Changes>) {
            changes.0 = query.iter().count();
        }
        let mut app = App::new();
        app.add_plugins(VisualPlugin)
            .init_resource::<Changes>()
            .add_systems(Last, count);
        let a = install(&mut app, 1, pose(0.0));
        let b = install(&mut app, 2, pose(1.0));
        assert_eq!(app.world().get::<Mesh3d>(a), app.world().get::<Mesh3d>(b));
        assert_eq!(
            app.world().get::<MeshMaterial3d<StandardMaterial>>(a),
            app.world().get::<MeshMaterial3d<StandardMaterial>>(b)
        );
        app.update();
        assert_eq!(app.world().resource::<Changes>().0, 2);
        for _ in 0..2 {
            let mut visuals = app.world_mut().resource_mut::<Visuals>();
            assert!(visuals.set_pose(VisualId(1), pose(0.0)));
            assert!(visuals.set_visibility(VisualId(1), true));
            app.update();
            assert_eq!(app.world().resource::<Changes>().0, 0);
        }
        app.world_mut()
            .resource_mut::<Visuals>()
            .set_pose(VisualId(1), pose(5.0));
        app.update();
        assert_eq!(app.world().resource::<Changes>().0, 1);
        let transform = app.world().get::<Transform>(a).unwrap();
        assert_eq!(transform.translation, Vec3::new(-2.0, 3.0, -5.0));
        assert_eq!(transform.scale, Vec3::new(2.0, 3.0, 4.0));
        assert_eq!(app.world().get::<Transform>(b).unwrap().translation.z, -1.0);
        app.world_mut()
            .resource_mut::<Visuals>()
            .set_visibility(VisualId(1), false);
        app.update();
        assert_eq!(
            *app.world().get::<Visibility>(a).unwrap(),
            Visibility::Hidden
        );
    }

    #[test]
    fn mesh_instances_reuse_real_assets_through_pose_and_visibility_updates() {
        let mut app = App::new();
        app.add_plugins(VisualPlugin);
        let mut meshes = Assets::<Mesh>::default();
        let mut materials = Assets::<StandardMaterial>::default();
        let mesh = meshes.add(Cuboid::new(1.0, 2.0, 3.0));
        let color = Color::srgb(0.2, 0.4, 0.8);
        let material = materials.add(StandardMaterial::from(color));
        let (a, b) = app
            .world_mut()
            .resource_scope(|world, mut visuals: Mut<Visuals>| {
                let a = visuals.spawn_mesh(
                    &mut world.commands(),
                    VisualId(1),
                    mesh.clone(),
                    material.clone(),
                    pose(0.0),
                    Vec3::ONE,
                );
                let b = visuals.spawn_mesh(
                    &mut world.commands(),
                    VisualId(2),
                    mesh.clone(),
                    material.clone(),
                    pose(1.0),
                    Vec3::ONE,
                );
                (a, b)
            });
        app.world_mut().flush();
        for frame in 0..16 {
            let mut visuals = app.world_mut().resource_mut::<Visuals>();
            visuals.set_pose(VisualId(1), pose(f64::from(frame)));
            visuals.set_visibility(VisualId(2), frame % 2 == 0);
            app.update();
            for entity in [a, b] {
                assert_eq!(app.world().get::<Mesh3d>(entity).unwrap().0.id(), mesh.id());
                assert_eq!(
                    app.world()
                        .get::<MeshMaterial3d<StandardMaterial>>(entity)
                        .unwrap()
                        .0
                        .id(),
                    material.id()
                );
            }
            assert_eq!(meshes.len(), 1);
            assert_eq!(materials.len(), 1);
            assert_eq!(materials.get(&material).unwrap().base_color, color);
        }
    }

    #[test]
    fn replacement_removal_reset_and_external_despawn_do_not_leak_or_retarget() {
        let mut app = App::new();
        app.add_plugins(VisualPlugin);
        let old = install(&mut app, 1, pose(0.0));
        let child = app
            .world_mut()
            .spawn((Transform::default(), ChildOf(old)))
            .id();
        {
            let mut visuals = app.world_mut().resource_mut::<Visuals>();
            visuals.set_pose(VisualId(1), pose(99.0));
            visuals.set_visibility(VisualId(1), false);
        }
        let replacement = install(&mut app, 1, pose(2.0));
        app.update();
        assert!(app.world().get_entity(old).is_err());
        assert!(app.world().get_entity(child).is_err());
        assert_eq!(
            app.world()
                .get::<Transform>(replacement)
                .unwrap()
                .translation
                .z,
            -2.0
        );
        assert_eq!(
            *app.world().get::<Visibility>(replacement).unwrap(),
            Visibility::Inherited
        );
        app.world_mut().despawn(replacement);
        // Reinstallation is also safe before the update system prunes external despawns.
        let next = install(&mut app, 1, pose(3.0));
        app.world_mut().despawn(next);
        app.update();
        assert!(app.world().resource::<Visuals>().is_empty());
        assert!(
            !app.world_mut()
                .resource_mut::<Visuals>()
                .set_pose(VisualId(1), pose(9.0))
        );
        let last = install(&mut app, 1, pose(4.0));
        app.world_mut()
            .resource_scope(|world, mut visuals: Mut<Visuals>| {
                visuals.set_pose(VisualId(1), pose(88.0));
                visuals.clear(&mut world.commands());
                assert!(!visuals.remove(&mut world.commands(), VisualId(1)));
            });
        let reset = install(&mut app, 1, pose(0.0));
        app.update();
        assert!(app.world().get_entity(last).is_err());
        assert_eq!(app.world().resource::<Visuals>().len(), 1);
        assert_eq!(
            app.world().get::<Transform>(reset).unwrap().translation.z,
            0.0
        );
        app.world_mut()
            .resource_scope(|world, mut visuals: Mut<Visuals>| {
                assert!(visuals.remove(&mut world.commands(), VisualId(1)));
            });
        app.update();
        assert!(app.world().get_entity(reset).is_err());
    }

    #[test]
    fn scenes_share_source_handles_and_sparse_storage_is_retained() {
        let mut app = App::new();
        app.add_plugins(VisualPlugin);
        let mut assets = Assets::<WorldAsset>::default();
        let handle = assets.add(WorldAsset::new(World::new()));
        let (a, b) = app
            .world_mut()
            .resource_scope(|world, mut visuals: Mut<Visuals>| {
                visuals.reserve(2, 4);
                let a = visuals.spawn_scene(
                    &mut world.commands(),
                    VisualId(1),
                    handle.clone(),
                    pose(0.0),
                    Vec3::ONE,
                );
                let b = visuals.spawn_scene(
                    &mut world.commands(),
                    VisualId(2),
                    handle.clone(),
                    pose(1.0),
                    Vec3::ONE,
                );
                (a, b)
            });
        app.world_mut().flush();
        assert_eq!(
            app.world().get::<WorldAssetRoot>(a).unwrap().0.id(),
            handle.id()
        );
        assert_eq!(
            app.world().get::<WorldAssetRoot>(b).unwrap().0.id(),
            handle.id()
        );
        let storage = {
            let visuals = app.world().resource::<Visuals>();
            (
                visuals.updates.as_ptr(),
                visuals.update_capacity(),
                visuals.roots.capacity(),
            )
        };
        for frame in 0..128 {
            let mut visuals = app.world_mut().resource_mut::<Visuals>();
            visuals.set_pose(VisualId(1), pose(f64::from(frame)));
            visuals.set_visibility(VisualId(2), frame % 2 == 0);
            app.update();
            let visuals = app.world().resource::<Visuals>();
            assert!(visuals.updates.is_empty());
            assert_eq!(
                (
                    visuals.updates.as_ptr(),
                    visuals.update_capacity(),
                    visuals.roots.capacity()
                ),
                storage
            );
            assert_eq!(visuals.len(), 2);
        }
        app.world_mut()
            .resource_scope(|world, mut visuals: Mut<Visuals>| {
                visuals.clear(&mut world.commands());
                assert_eq!(visuals.update_capacity(), storage.1);
                assert_eq!(visuals.roots.capacity(), storage.2);
            });
        app.update();
        assert!(app.world().get_entity(a).is_err());
        assert!(app.world().get_entity(b).is_err());
        assert_eq!(assets.len(), 1);
    }
}
