// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Bounded, immutable compound geometry without backend or mass semantics.
use super::{ShapeDescription, body::validate_pose};
use crate::Pose;
use std::sync::Arc;

#[derive(Debug, PartialEq)]
struct CompoundData {
    children: Vec<(Pose, ShapeDescription)>,
    depth: usize,
    nodes: usize,
    contains_triangle_surface: bool,
}

/// Ordered child shapes in local FLU mounting frames, forming one collider.
/// Clones share the complete immutable allocation. Child frames, shape order and
/// geometry are retained exactly; children carry no material or mass policy.
///
/// Sources may nest up to eight compound levels and contain at most 256 expanded
/// shape nodes, including the root and each occurrence of a shared child. These
/// bounds comfortably admit 32-leaf payloads while bounding recursion and repeated
/// cooking of shared subtrees. Cached counts make construction linear in direct
/// children. Backend capability is separate: cooking may reject nested compounds
/// or triangle children without flattening, replacing or splitting the source.
///
/// ```
/// use sim_engine::{Pose, model::{CompoundShape, ShapeDescription}};
/// let shape = CompoundShape::new(vec![
///     (Pose::at([1., 0., 0.]), ShapeDescription::Ball { radius_m: 0.1 }),
/// ])?;
/// let clone = shape.clone();
/// assert_eq!(shape.children().as_ptr(), clone.children().as_ptr());
/// assert_eq!(shape.children()[0].0.translation_m, [1., 0., 0.]);
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct CompoundShape(Arc<CompoundData>);

impl CompoundShape {
    /// Retain nonempty ordered children after validating finite unit-quaternion
    /// mounts, primitive dimensions and the eight-level/256-node source bounds.
    /// No pose normalization, geometry cooking or mass inference is performed.
    pub fn new(children: Vec<(Pose, ShapeDescription)>) -> Result<Self, &'static str> {
        if children.is_empty() {
            return Err("compound requires at least one child");
        }
        // Each direct child counts at least once; reject before inspecting a
        // caller's arbitrarily large vector. Shared descendants count per use.
        if children.len() >= 256 {
            return Err("compound exceeds 256 expanded shape nodes");
        }
        let mut depth = 1;
        let mut nodes = 1;
        let mut contains_triangle_surface = false;
        for (pose, shape) in &children {
            validate_pose(*pose)?;
            shape.validate()?;
            let (child_depth, child_nodes) = match shape {
                ShapeDescription::Compound(child) => (child.0.depth, child.0.nodes),
                _ => (0, 1),
            };
            depth = depth.max(1 + child_depth);
            nodes += child_nodes;
            if depth > 8 || nodes > 256 {
                return Err("compound exceeds eight levels or 256 expanded shape nodes");
            }
            contains_triangle_surface |= shape.contains_triangle_surface();
        }
        Ok(Self(Arc::new(CompoundData {
            children,
            depth,
            nodes,
            contains_triangle_surface,
        })))
    }

    /// Original child-local to compound-local frames and shapes, in source order.
    pub fn children(&self) -> &[(Pose, ShapeDescription)] {
        &self.0.children
    }

    pub(super) fn contains_triangle_surface(&self) -> bool {
        self.0.contains_triangle_surface
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{
        BodyKind, ColliderDescription, ColliderMass, ColliderPolicy, ContactMaterial, Inertial,
        RigidBodyDescription, TriangleMesh,
    };

    fn ball() -> ShapeDescription {
        ShapeDescription::Ball { radius_m: 0.1 }
    }

    #[test]
    fn retains_order_mount_bits_and_shared_immutable_children() {
        let pose = Pose {
            translation_m: [1., -2., 3.],
            rotation_wxyz: [1. + f64::EPSILON, 0., 0., 0.],
        };
        let children = vec![(pose, ball()), (Pose::default(), ball())];
        let pointer = children.as_ptr();
        let compound = CompoundShape::new(children.clone()).unwrap();
        let shared = compound.clone();
        assert_eq!(compound.children(), children);
        assert_eq!(compound.children().as_ptr(), shared.children().as_ptr());
        assert_eq!(
            CompoundShape::new(children).unwrap().children().as_ptr(),
            pointer
        );
        assert_eq!(
            compound.children()[0].0.rotation_wxyz[0].to_bits(),
            pose.rotation_wxyz[0].to_bits()
        );
        assert!(
            CompoundShape::new(vec![(
                Pose::default(),
                ShapeDescription::Ball { radius_m: -1. }
            )])
            .is_err()
        );
        assert_eq!(shared.children(), compound.children());
    }

    #[test]
    fn invalid_mounts_and_dimensions_are_rejected_without_changing_source() {
        let original = CompoundShape::new(vec![(Pose::default(), ball())]).unwrap();
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for axis in 0..3 {
                let mut pose = Pose::default();
                pose.translation_m[axis] = bad;
                assert!(
                    CompoundShape::new(vec![(pose, ShapeDescription::Compound(original.clone()))])
                        .is_err()
                );
            }
            for axis in 0..4 {
                let mut pose = Pose::default();
                pose.rotation_wxyz[axis] = bad;
                assert!(CompoundShape::new(vec![(pose, ball())]).is_err());
            }
        }
        for rotation_wxyz in [[0.; 4], [2., 0., 0., 0.]] {
            assert!(
                CompoundShape::new(vec![(
                    Pose {
                        rotation_wxyz,
                        ..Pose::default()
                    },
                    ball()
                )])
                .is_err()
            );
        }
        for bad in [0., -1., f64::NAN, f64::INFINITY] {
            for (radius_m, half_height_m) in [(bad, 1.), (1., bad)] {
                assert!(
                    CompoundShape::new(vec![(
                        Pose::default(),
                        ShapeDescription::Cylinder {
                            radius_m,
                            half_height_m
                        },
                    )])
                    .is_err()
                );
            }
            for axis in 0..3 {
                let mut half_extents_m = [1.; 3];
                half_extents_m[axis] = bad;
                assert!(
                    CompoundShape::new(vec![(
                        Pose::default(),
                        ShapeDescription::Cuboid { half_extents_m }
                    )])
                    .is_err()
                );
            }
        }
        assert_eq!(original.children(), &[(Pose::default(), ball())]);
    }

    #[test]
    fn source_bounds_count_repeated_subtrees_and_bound_depth() {
        assert!(CompoundShape::new(vec![]).is_err());
        assert!(CompoundShape::new(vec![(Pose::default(), ball()); 255]).is_ok());
        assert!(CompoundShape::new(vec![(Pose::default(), ball()); 256]).is_err());
        let mut nested = CompoundShape::new(vec![(Pose::default(), ball())]).unwrap();
        for _ in 1..8 {
            nested =
                CompoundShape::new(vec![(Pose::default(), ShapeDescription::Compound(nested))])
                    .unwrap();
        }
        assert!(
            CompoundShape::new(vec![(Pose::default(), ShapeDescription::Compound(nested))])
                .is_err()
        );
        let shared = CompoundShape::new(vec![(Pose::default(), ball()); 32]).unwrap();
        assert!(
            CompoundShape::new(vec![
                (
                    Pose::default(),
                    ShapeDescription::Compound(shared.clone())
                );
                7
            ])
            .is_ok()
        );
        assert!(
            CompoundShape::new(vec![
                (Pose::default(), ShapeDescription::Compound(shared));
                8
            ])
            .is_err()
        );
    }

    #[test]
    fn hidden_triangle_mesh_is_rejected_for_both_dynamic_mass_modes() {
        let mesh =
            TriangleMesh::new(vec![[0.; 3], [1., 0., 0.], [0., 1., 0.]], vec![[0, 1, 2]]).unwrap();
        let inner = CompoundShape::new(vec![(
            Pose::default(),
            ShapeDescription::TriangleMesh(mesh),
        )])
        .unwrap();
        let outer =
            CompoundShape::new(vec![(Pose::default(), ShapeDescription::Compound(inner))]).unwrap();
        let collider = ColliderDescription::new(
            ShapeDescription::Compound(outer),
            Pose::default(),
            ContactMaterial {
                friction: 0.4,
                restitution: 0.,
                memberships: u32::MAX,
                filter: u32::MAX,
            },
        )
        .unwrap();
        for kind in [BodyKind::Fixed, BodyKind::Kinematic] {
            assert!(RigidBodyDescription::new(kind, vec![collider.clone()]).is_ok());
        }
        assert_eq!(
            RigidBodyDescription::new(
                BodyKind::Dynamic(Inertial::new(2., [1.; 3], Pose::default()).unwrap()),
                vec![collider.clone()]
            )
            .unwrap_err(),
            "dynamic triangle surfaces are unsupported"
        );
        let massive = collider
            .with_policy(ColliderPolicy {
                mass: ColliderMass::MassKg(2.),
                ..Default::default()
            })
            .unwrap();
        assert_eq!(
            RigidBodyDescription::new(BodyKind::DynamicFromColliders, vec![massive]).unwrap_err(),
            "dynamic triangle surfaces are unsupported"
        );
    }
}
