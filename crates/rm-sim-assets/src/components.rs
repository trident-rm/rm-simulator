// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Compose existing exported equipment without a Field, server or renderer.
//! Source partitions and semantic bindings remain authored by rm-map-tools.
use crate::{
    cad_assets::{CadAsset, CadAssets},
    collision_mesh::{self, CollisionMesh, CollisionPart},
    math,
    rmuc_2027::Motion,
};
use anyhow::{Context, ensure};
use rm_sim_physics::{
    Pose, Team, WorldPhysics,
    motion::{Mechanism, MotionFrame},
};

/// Native source child pose and analytic shape. Frames are asset-local before
/// placement and world FLU afterwards; quaternions use wxyz, cylinders local +Y.
pub type NativeShape = (Pose, rm_sim_physics::model::ShapeDescription);

pub(crate) fn place_shapes(shapes: &[NativeShape], placement: &Pose) -> Vec<NativeShape> {
    shapes
        .iter()
        .map(|(pose, shape)| {
            (
                Pose {
                    translation_m: math::gltf_point(placement, pose.translation_m),
                    rotation_wxyz: math::quat_multiply(
                        math::gltf_rotation(placement),
                        pose.rotation_wxyz,
                    ),
                },
                shape.clone(),
            )
        })
        .collect()
}

/// Role used to interpret the existing source motion records and old fixed nodes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ComponentRole {
    /// General static equipment or equipment with semantic prismatic joints.
    Equipment,
    /// Rune assembly; native motion slots identify Red and Blue faces.
    Rune,
    /// Outpost assembly; native rotors belong to their placed field side.
    Outpost,
}

/// A placed moving collider at its exported rest pose, in world FLU metres.
#[derive(Clone, Debug)]
pub struct MechanismMesh {
    /// Complete source triangles after one asset placement transform.
    pub mesh: CollisionMesh,
    /// Native children in world-FLU rest frames, driven by the same channel.
    pub native_shapes: Vec<NativeShape>,
    /// Caller-controlled mechanism channel; no rule state is owned here.
    pub mechanism: Mechanism,
    /// Existing channel owner, from source face order or placement side.
    pub team: Team,
    /// World translation offsets or world rotor pivot and unit axis.
    pub motion: MotionFrame,
}

#[derive(Clone, Debug)]
enum LocalFrame {
    Translation { axis: [f64; 3], limits: [f64; 2] },
    Rotation { pivot_m: [f64; 3], axis: [f64; 3] },
}
#[derive(Clone, Debug)]
struct LocalMotion {
    mesh: CollisionMesh,
    native_shapes: Vec<NativeShape>,
    source_node: Option<usize>,
    mechanism: Mechanism,
    owner: Option<Team>,
    frame: LocalFrame,
}

/// Reusable equipment definition in the original GLB's local frame.
/// Load once, then place independently. No HP, timer, rules or live body handles.
#[derive(Clone, Debug)]
pub struct ComponentModel {
    /// Optional fixed sensor-query surface for the native owners; never adds contacts.
    pub query_mesh: Option<CollisionMesh>,
    /// Fixed triangles in asset-local metres, with internal GLB transforms baked.
    pub fixed: CollisionMesh,
    /// Fixed native children in the same frame as `fixed`.
    pub native_shapes: Vec<NativeShape>,
    moving: Vec<LocalMotion>,
}

/// One placed equipment instance, independent of arena ground and match setup.
#[derive(Clone, Debug)]
pub struct PlacedComponent {
    /// Optional fixed sensor-query surface, placed once in world FLU.
    pub query_mesh: Option<CollisionMesh>,
    /// Fixed triangles in world FLU metres.
    pub fixed: CollisionMesh,
    /// Fixed native children in the same frame as `fixed`.
    pub native_shapes: Vec<NativeShape>,
    /// Source moving parts with world FLU frames, in binding order.
    pub moving: Vec<MechanismMesh>,
}
impl LocalMotion {
    fn placed(&self, placement: &Pose) -> MechanismMesh {
        let q = math::gltf_rotation(placement);
        MechanismMesh {
            mesh: self.mesh.placed(placement),
            native_shapes: place_shapes(&self.native_shapes, placement),
            mechanism: self.mechanism,
            team: self.owner.unwrap_or(if placement.translation_m[0] >= 0. {
                Team::Red
            } else {
                Team::Blue
            }),
            motion: match self.frame {
                LocalFrame::Translation { axis, limits } => MotionFrame::Translation {
                    offsets_m: limits.map(|v| math::rotate(q, axis).map(|a| a * v)),
                },
                LocalFrame::Rotation { pivot_m, axis } => MotionFrame::Rotation {
                    pivot_m: math::gltf_point(placement, pivot_m),
                    axis: math::rotate(q, axis),
                },
            },
        }
    }
}
impl ComponentModel {
    /// Apply an existing asset placement exactly once, resolving motion frames too.
    /// Use an exported `CadAsset::placements` root or `math::gltf_root_pose`;
    /// it includes the renderer basis, unlike a plain FLU `Pose::at`.
    /// Placement must be finite with a unit quaternion, as verified by the loader.
    pub fn placed(&self, placement: &Pose) -> PlacedComponent {
        PlacedComponent {
            query_mesh: self.query_mesh.as_ref().map(|mesh| mesh.placed(placement)),
            fixed: self.fixed.placed(placement),
            native_shapes: place_shapes(&self.native_shapes, placement),
            moving: self
                .moving
                .iter()
                .map(|part| part.placed(placement))
                .collect(),
        }
    }
    /// Resolve moving geometry in binding-major, placement-minor order, preserving
    /// the existing arena's collider insertion order without copying fixed meshes.
    /// Placements must be finite with unit quaternions, as checked by the loader.
    pub fn mechanisms(&self, placements: &[Pose]) -> Vec<MechanismMesh> {
        self.moving
            .iter()
            .flat_map(|part| placements.iter().map(|p| part.placed(p)))
            .collect()
    }
}
impl PlacedComponent {
    /// Install source geometry into an ordinary physical world; callers own motion
    /// updates, target faces and sensor scheduling. Meshes are installed in order;
    /// a rejected mesh leaves earlier installed geometry intact.
    ///
    /// ```
    /// use rm_sim_assets::{components::PlacedComponent, collision_mesh::CollisionMesh};
    /// use rm_sim_physics::WorldPhysics;
    /// let fixed = CollisionMesh {
    ///     vertices_m: vec![[1., -1., -1.], [1., 1., -1.], [1., 0., 1.]],
    ///     triangles: vec![[0, 1, 2]],
    /// };
    /// let mut physics = WorldPhysics::new(&[], 0.);
    /// PlacedComponent { fixed, query_mesh: None, native_shapes: vec![], moving: vec![] }.install(&mut physics)?;
    /// let scene = physics.static_geometry_snapshot(true);
    /// assert!(scene.cast_ray([0.; 3], [1., 0., 0.], 2.)?.is_some());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn install(self, physics: &mut WorldPhysics) -> Result<(), &'static str> {
        if !self.fixed.triangles.is_empty() {
            physics.add_static_mesh(self.fixed.vertices_m, self.fixed.triangles)?;
        }
        if let Some(query) = self.query_mesh {
            physics.add_static_shapes_with_queries(
                &self.native_shapes,
                query.vertices_m,
                query.triangles,
            )?;
        } else {
            physics.add_static_shapes(&self.native_shapes)?;
        }
        for part in self.moving {
            part.install(physics)?;
        }
        Ok(())
    }
}
impl MechanismMesh {
    /// Install retained triangles and native children on the same source channel.
    /// Either representation may be empty, but native geometry is never dropped.
    pub fn install(&self, physics: &mut WorldPhysics) -> Result<(), &'static str> {
        if !self.mesh.triangles.is_empty() {
            match self.motion {
                MotionFrame::Translation { offsets_m } => physics.add_mechanism_mesh(
                    self.mesh.vertices_m.clone(),
                    self.mesh.triangles.clone(),
                    self.mechanism,
                    self.team,
                    offsets_m,
                )?,
                MotionFrame::Rotation { pivot_m, axis } => physics.add_rotating_mesh(
                    self.mesh.vertices_m.clone(),
                    self.mesh.triangles.clone(),
                    self.mechanism,
                    self.team,
                    pivot_m,
                    axis,
                )?,
            }
        }
        physics.add_mechanism_shapes(
            &self.native_shapes,
            self.mechanism,
            self.team,
            self.motion,
        )?;
        Ok(())
    }
}

/// Load a verified package asset using its existing collision contract and bindings.
/// `Some(source_motions)` selects native source-body partitions (even when empty)
/// ahead of rest-pose/semantic handling. `None` uses existing semantic joints,
/// or the complete rest pose for rest-pose-only packages. This does not verify
/// arbitrary files: obtain `cad` and `asset` from the checked package loader first.
/// Returned fixed/moving geometry is asset-local; call [`ComponentModel::placed`]
/// with one of the exported placements before installing it.
pub fn load<B>(
    cad: &CadAssets<B>,
    asset: &CadAsset,
    role: ComponentRole,
    source_motions: Option<&[Motion]>,
) -> anyhow::Result<ComponentModel> {
    let mut model = load_meshes(cad, asset, role, source_motions)?;
    if let Some(shapes) = &asset.collision_shapes {
        for part in &shapes.parts {
            let shape = (
                part.pose,
                rm_sim_physics::model::ShapeDescription::Cylinder {
                    radius_m: part.radius_m,
                    half_height_m: part.half_height_m,
                },
            );
            if let Some(motions) = source_motions {
                let owners: Vec<_> = motions
                    .iter()
                    .enumerate()
                    .filter(|(_, m)| m.bodies.contains(&part.source_node_name))
                    .map(|(i, _)| i)
                    .collect();
                ensure!(
                    owners.len() <= 1,
                    "native part belongs to multiple source motions"
                );
                if let Some(&index) = owners.first() {
                    model.moving[index].native_shapes.push(shape);
                } else {
                    model.native_shapes.push(shape);
                }
            } else if cad.rest_pose_only {
                model.native_shapes.push(shape);
            } else if let Some(semantics) = asset.physics_semantics() {
                let owners: Vec<_> = semantics
                    .joints
                    .iter()
                    .filter(|j| {
                        mechanism_joint(j).is_some()
                            && part.ancestor_nodes_root_first.contains(&j.motion_node)
                    })
                    .collect();
                ensure!(
                    owners.len() <= 1,
                    "native part has overlapping semantic motion owners"
                );
                if let Some(joint) = owners.first() {
                    model
                        .moving
                        .iter_mut()
                        .find(|m| m.source_node == Some(joint.motion_node))
                        .context("missing native semantic channel")?
                        .native_shapes
                        .push(shape);
                } else {
                    ensure!(
                        role == ComponentRole::Equipment
                            || semantics
                                .fixed_primitives
                                .iter()
                                .any(|p| p.0 == part.source_node_index),
                        "native part lacks fixed/moving classification"
                    );
                    model.native_shapes.push(shape);
                }
            } else {
                ensure!(
                    role == ComponentRole::Equipment || part.source_node_name == "static",
                    "native part requires explicit source motion ownership"
                );
                model.native_shapes.push(shape);
            }
        }
    }
    ensure!(
        asset
            .collision_shapes
            .as_ref()
            .is_none_or(|s| s.query_mesh.is_none())
            || model.moving.iter().all(|m| m.native_shapes.is_empty()),
        "query mesh profiles currently require fixed native owners"
    );
    model.query_mesh = asset
        .collision_shapes
        .as_ref()
        .and_then(|s| s.query_mesh.as_deref())
        .cloned();
    Ok(model)
}

fn load_meshes<B>(
    cad: &CadAssets<B>,
    asset: &CadAsset,
    role: ComponentRole,
    source_motions: Option<&[Motion]>,
) -> anyhow::Result<ComponentModel> {
    let bytes = asset.physics_bytes(&cad.root)?;
    if let Some(motions) = source_motions {
        let mut parts = collision_mesh::load_glb_parts_bytes(&bytes)?;
        if let Some(shapes) = &asset.collision_shapes {
            for part in &shapes.parts {
                if !parts.iter().any(|p| p.name == part.source_node_name) {
                    parts.push(CollisionPart {
                        name: part.source_node_name.clone(),
                        mesh: CollisionMesh::default(),
                    });
                }
            }
        }
        return from_source_parts(parts, role, motions);
    }
    if cad.rest_pose_only {
        return Ok(ComponentModel {
            query_mesh: None,
            fixed: collision_mesh::load_glb_bytes(&bytes)?,
            native_shapes: vec![],
            moving: vec![],
        });
    }
    let Some(semantics) = asset.physics_semantics() else {
        let parts = collision_mesh::load_glb_parts_bytes(&bytes)?;
        let fixed = if role == ComponentRole::Equipment {
            combine(parts)
        } else {
            fixed_parts(parts)
        };
        return Ok(ComponentModel {
            query_mesh: None,
            fixed,
            native_shapes: vec![],
            moving: vec![],
        });
    };
    let joints: Vec<_> = semantics
        .joints
        .iter()
        .filter_map(|joint| mechanism_joint(joint).map(|mechanism| (joint, mechanism)))
        .collect();
    let fixed = if role == ComponentRole::Equipment {
        let excluded: Vec<_> = joints.iter().map(|(j, _)| j.motion_node).collect();
        collision_mesh::load_glb_filtered_bytes(
            &bytes,
            &semantics.collision_primitives,
            &excluded,
            None,
        )?
    } else {
        collision_mesh::load_glb_selected_bytes(&bytes, &semantics.fixed_primitives)?
    };
    let mut moving = Vec::new();
    for (joint, mechanism) in joints {
        let mesh = collision_mesh::load_glb_filtered_bytes(
            &bytes,
            &semantics.collision_primitives,
            &[],
            Some(joint.motion_node),
        )?;
        if mesh.triangles.is_empty()
            && !asset.collision_shapes.as_ref().is_some_and(|s| {
                s.parts
                    .iter()
                    .any(|p| p.ancestor_nodes_root_first.contains(&joint.motion_node))
            })
        {
            continue;
        }
        let axis = math::rotate(
            [
                joint.rotation_xyzw[3],
                joint.rotation_xyzw[0],
                joint.rotation_xyzw[1],
                joint.rotation_xyzw[2],
            ],
            joint.axis,
        );
        moving.push(LocalMotion {
            mesh,
            native_shapes: vec![],
            source_node: Some(joint.motion_node),
            mechanism,
            owner: None,
            frame: LocalFrame::Translation {
                axis,
                limits: joint.limits.expect("bounded mechanism"),
            },
        });
    }
    Ok(ComponentModel {
        query_mesh: None,
        fixed,
        native_shapes: vec![],
        moving,
    })
}

fn fixed_parts(parts: Vec<CollisionPart>) -> CollisionMesh {
    combine(parts.into_iter().filter(|p| p.name == "static"))
}
fn combine(parts: impl IntoIterator<Item = CollisionPart>) -> CollisionMesh {
    let mut mesh = CollisionMesh::default();
    for part in parts {
        mesh.append(part.mesh);
    }
    mesh
}
fn from_source_parts(
    parts: Vec<CollisionPart>,
    role: ComponentRole,
    motions: &[Motion],
) -> anyhow::Result<ComponentModel> {
    ensure!(
        role != ComponentRole::Rune || motions.len() <= 2,
        "Rune source motion has more than two face channels"
    );
    let mut moving = Vec::new();
    for (index, motion) in motions.iter().enumerate() {
        let mut mesh = CollisionMesh::default();
        for name in &motion.bodies {
            let part = parts
                .iter()
                .find(|p| &p.name == name)
                .with_context(|| format!("missing bound source body {name}"))?;
            mesh.append(part.mesh.clone());
        }
        let mechanism = if role == ComponentRole::Rune {
            Mechanism::Rune
        } else if motion.limits.is_none() {
            Mechanism::Outpost
        } else {
            Mechanism::Base
        };
        let frame = if let Some(limits) = motion.limits {
            LocalFrame::Translation {
                axis: motion.axis,
                limits,
            }
        } else {
            LocalFrame::Rotation {
                pivot_m: motion.pivot_m,
                axis: motion.axis,
            }
        };
        moving.push(LocalMotion {
            mesh,
            native_shapes: vec![],
            source_node: None,
            mechanism,
            frame,
            owner: (role == ComponentRole::Rune).then(|| Team::BOTH[index]),
        });
    }
    let fixed = combine(
        parts
            .into_iter()
            .filter(|part| !motions.iter().any(|m| m.bodies.contains(&part.name))),
    );
    Ok(ComponentModel {
        query_mesh: None,
        fixed,
        native_shapes: vec![],
        moving,
    })
}

/// Existing semantic prismatic channels supported by caller-prescribed physics.
/// This classifies exported IDs; it does not activate a referee or schedule motion.
pub fn mechanism_joint(joint: &crate::semantics::Joint) -> Option<Mechanism> {
    if joint.kind != "prismatic" || joint.limits.is_none() {
        return None;
    }
    match joint.id.as_str() {
        "base.shield.0.slide" | "base.shield.1.slide" | "base.shield.2.slide" => {
            Some(Mechanism::Base)
        }
        "dart-station.gate.slide" => Some(Mechanism::DartDoor),
        "base.dart_target.slide" => Some(Mechanism::DartTarget),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn part(name: &str, x: f64) -> CollisionPart {
        CollisionPart {
            name: name.into(),
            mesh: CollisionMesh {
                vertices_m: vec![[x, -1., -1.], [x, 1., -1.], [x, 0., 1.]],
                triangles: vec![[0, 1, 2]],
            },
        }
    }
    fn motion(name: &str, limits: Option<[f64; 2]>) -> Motion {
        Motion {
            id: name.into(),
            bodies: vec![name.into()],
            pivot_m: [0.; 3],
            axis: [0., 0., 1.],
            limits,
        }
    }
    #[test]
    fn native_partition_places_once_and_keeps_binding_major_order() {
        let motions = [motion("rotor", None), motion("slide", Some([-0.2, 0.3]))];
        let model = from_source_parts(
            vec![part("fixed", 3.), part("rotor", 1.), part("slide", 2.)],
            ComponentRole::Outpost,
            &motions,
        )
        .unwrap();
        assert_eq!(model.fixed.triangles.len(), 1);
        let placements = [Pose::yawed([4., 5., 6.], 0.7), Pose::at([-4., 0., 0.])];
        let moving = model.mechanisms(&placements);
        assert_eq!(moving.len(), 4);
        assert_eq!(
            moving.iter().map(|m| m.team).collect::<Vec<_>>(),
            [Team::Red, Team::Blue, Team::Red, Team::Blue]
        );
        assert_eq!(
            moving[0].mesh.vertices_m,
            part("rotor", 1.).mesh.placed(&placements[0]).vertices_m
        );
        assert_eq!(
            model.placed(&placements[0]).fixed.vertices_m,
            part("fixed", 3.).mesh.placed(&placements[0]).vertices_m
        );
        assert_eq!(
            moving[0].motion,
            MotionFrame::Rotation {
                pivot_m: math::gltf_point(&placements[0], motions[0].pivot_m),
                axis: math::rotate(math::gltf_rotation(&placements[0]), motions[0].axis),
            }
        );
        assert_eq!(
            moving[2].motion,
            MotionFrame::Translation {
                offsets_m: motions[1].limits.unwrap().map(|v| math::rotate(
                    math::gltf_rotation(&placements[0]),
                    motions[1].axis
                )
                .map(|a| a * v)),
            }
        );
    }
    #[test]
    fn rune_face_owner_does_not_follow_placement_side_and_missing_bodies_fail() {
        let motions = [motion("a", None), motion("b", None)];
        let model = from_source_parts(
            vec![part("a", 1.), part("b", 2.)],
            ComponentRole::Rune,
            &motions,
        )
        .unwrap();
        let moving = model.mechanisms(&[Pose::at([-4., 0., 0.])]);
        assert_eq!([moving[0].team, moving[1].team], Team::BOTH);
        assert!(moving.iter().all(|m| m.mechanism == Mechanism::Rune));
        assert!(from_source_parts(vec![], ComponentRole::Outpost, &motions).is_err());
        assert!(
            from_source_parts(
                vec![],
                ComponentRole::Rune,
                &[motions[0].clone(), motions[1].clone(), motions[0].clone()]
            )
            .is_err()
        );
    }
    #[test]
    fn installed_component_moves_in_a_bare_physical_world() {
        let model = from_source_parts(
            vec![part("fixed", 3.), part("rotor", 1.)],
            ComponentRole::Outpost,
            &[motion("rotor", None)],
        )
        .unwrap();
        let mut physics = WorldPhysics::new(&[], 0.);
        model
            .placed(&math::gltf_root_pose([0.; 3], [1., 0., 0., 0.]))
            .install(&mut physics)
            .unwrap();
        let before = physics.static_geometry_snapshot(true);
        assert!(
            before
                .cast_ray([0.; 3], [1., 0., 0.], 2.)
                .unwrap()
                .is_some()
        );
        let state = rm_sim_physics::motion::MechanismState {
            outpost_angle_rad: [std::f64::consts::FRAC_PI_2, 0.],
            ..Default::default()
        };
        physics.sync_mechanisms(&state);
        let after = physics.static_geometry_snapshot(true);
        assert!(after.cast_ray([0.; 3], [1., 0., 0.], 2.).unwrap().is_none());
        let hit = after.cast_ray([0.; 3], [0., 1., 0.], 2.).unwrap().unwrap();
        assert!((hit.distance_m - 1.).abs() < 1e-12);
    }
    #[test]
    fn only_the_static_node_of_a_rune_or_outpost_visual_is_fixed() {
        let tri = |z: f64| CollisionMesh {
            vertices_m: vec![[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]],
            triangles: vec![[0, 1, 2]],
        };
        let parts = vec![
            CollisionPart {
                name: "face_0/blade_0".into(),
                mesh: tri(1.0),
            },
            CollisionPart {
                name: "static".into(),
                mesh: tri(2.0),
            },
            CollisionPart {
                name: "rotor/carrier".into(),
                mesh: tri(3.0),
            },
            CollisionPart {
                name: "static".into(),
                mesh: tri(4.0),
            },
        ];
        let fixed = fixed_parts(parts);
        assert_eq!(fixed.triangles, vec![[0, 1, 2], [3, 4, 5]]);
        assert!(fixed.vertices_m.iter().all(|v| v[2] == 2.0 || v[2] == 4.0));
        assert!(fixed_parts(Vec::new()).triangles.is_empty());
    }
}

#[cfg(test)]
#[path = "components/hybrid_tests.rs"]
mod hybrid_tests;
