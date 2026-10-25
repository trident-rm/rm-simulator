// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared backend shapes for an explicitly described rigid body.
use super::*;
use rapier3d_f64::prelude::{
    CoefficientCombineRule, ColliderBuilder, Group, InteractionGroups, InteractionTestMode,
    MassProperties, Pose as Pose3, RigidBodyBuilder, Rotation,
};
use sim_engine::model::{
    BodyKind, ColliderMass, ConvexHull, RigidBodyDescription, ShapeDescription,
};
use std::sync::Arc;

/// Source body plus shared cooked collider shapes, with no world handles/state.
/// Cooking preserves local frames, source collider order and contact parameters.
/// Explicit inertial data remains the default; collider aggregation is opt-in.
#[derive(Clone)]
pub struct CookedRigidBody {
    source: RigidBodyDescription,
    shapes: Arc<[SharedShape]>,
    aggregate_mass: Option<MassProperties>,
}
impl CookedRigidBody {
    /// Cook collider shapes once. Empty surfaces or backend geometry failures
    /// return an error before any world installation. Clones share all buffers.
    /// Compounds remain single native shapes/colliders. This backend rejects
    /// composite children (nested compounds or triangle surfaces) without fallback.
    /// Collider-derived compounds validate each child's native mass properties at
    /// the selected density before aggregation can hide overflow/underflow. Explicit
    /// collider mass follows native unit-density aggregation then rescaling; lost
    /// unit-density information cannot be recovered by choosing a smaller mass.
    ///
    /// ```
    /// use sim_engine::model::{BodyKind, RigidBodyDescription};
    /// use sim_physics::cooking::CookedRigidBody;
    /// let source = RigidBodyDescription::new(BodyKind::Fixed, vec![])?;
    /// let cooked = CookedRigidBody::new(source)?;
    /// assert!(cooked.source().colliders().is_empty());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn new(source: RigidBodyDescription) -> Result<Self, &'static str> {
        let shapes = source
            .colliders()
            .iter()
            .map(|c| cook_shape(c.shape()))
            .collect::<Result<Vec<_>, &'static str>>()?;
        let mut cooked = Self {
            source,
            shapes: shapes.into(),
            aggregate_mass: None,
        };
        cooked.aggregate_mass = cooked.validate_mass_properties()?;
        Ok(cooked)
    }
    /// Original immutable physical description, including explicit inertia.
    pub fn source(&self) -> &RigidBodyDescription {
        &self.source
    }
    /// Native construction without installing/registering or refreshing COM.
    /// The caller chooses initial state and preserves participant insertion timing.
    pub(crate) fn native_body_builder(&self) -> RigidBodyBuilder {
        let body = match self.source.kind() {
            BodyKind::Fixed => RigidBodyBuilder::fixed(),
            BodyKind::Kinematic => RigidBodyBuilder::kinematic_position_based(),
            BodyKind::DynamicFromColliders => RigidBodyBuilder::dynamic(),
            BodyKind::Dynamic(inertia) => {
                let frame = inertia.frame();
                RigidBodyBuilder::dynamic().additional_mass_properties(
                    MassProperties::with_principal_inertia_frame(
                        Vector::from_array(frame.translation_m),
                        inertia.mass_kg(),
                        Vector::from_array(inertia.principal_inertia_kg_m2()),
                        crate::runtime::math::pose(frame).rotation,
                    ),
                )
            }
        };
        let p = self.source.policy();
        let [tx, ty, tz] = p.enabled_translations;
        let [rx, ry, rz] = p.enabled_rotations;
        body.gravity_scale(p.gravity_scale)
            .enabled_translations(tx, ty, tz)
            .enabled_rotations(rx, ry, rz)
            .can_sleep(p.can_sleep)
            .linear_damping(p.linear_damping)
            .angular_damping(p.angular_damping)
            .ccd_enabled(p.ccd_enabled)
    }

    /// Native colliders in source order; retain stored quaternion bits exactly.
    /// No handles or mass refresh are performed. Explicit mass(0) remains distinct
    /// from density(0), including on massless moving auxiliaries.
    pub(crate) fn native_collider_builders(&self) -> impl Iterator<Item = ColliderBuilder> + '_ {
        self.source
            .colliders()
            .iter()
            .zip(self.shapes())
            .map(|(c, shape)| {
                let material = c.material();
                let policy = c.policy();
                let [w, x, y, z] = c.pose().rotation_wxyz;
                let builder = ColliderBuilder::new(shape.clone())
                    .position(Pose3::from_parts(
                        Vector::from_array(c.pose().translation_m),
                        Rotation::from_xyzw(x, y, z, w),
                    ))
                    .friction(material.friction)
                    .restitution(material.restitution)
                    .friction_combine_rule(native_combine(policy.friction_combine_rule))
                    .restitution_combine_rule(native_combine(policy.restitution_combine_rule))
                    .collision_groups(InteractionGroups::new(
                        Group::from_bits_retain(material.memberships),
                        Group::from_bits_retain(material.filter),
                        InteractionTestMode::And,
                    ));
                match policy.mass {
                    ColliderMass::Zero => builder.density(0.),
                    ColliderMass::MassKg(mass) => builder.mass(mass),
                    ColliderMass::DensityKgM3(density) => builder.density(density),
                }
            })
    }

    pub(crate) fn native_aggregate_mass_properties(&self) -> Option<&MassProperties> {
        self.aggregate_mass.as_ref()
    }

    fn validate_mass_properties(&self) -> Result<Option<MassProperties>, &'static str> {
        if matches!(self.source.kind(), BodyKind::Dynamic(_))
            || !self.source.colliders().iter().any(|collider| {
                matches!(
                    collider.policy().mass,
                    ColliderMass::MassKg(x) | ColliderMass::DensityKgM3(x) if x > 0.
                )
            })
        {
            return Ok(None);
        }
        // Match native insertion's ordered shape mass accumulation, without
        // touching a live world. Source frames and shape laws remain unchanged.
        let mut total = MassProperties::default();
        let mut accumulated_mass_kg = 0.;
        for ((description, shape), builder) in self
            .source
            .colliders()
            .iter()
            .zip(self.shapes())
            .zip(self.native_collider_builders())
        {
            validate_compound_child_mass(shape, description.policy().mass)?;
            let collider = builder.build();
            let mass = collider.mass_properties().transform_by(collider.position());
            let positive = matches!(description.policy().mass,
                ColliderMass::MassKg(x) | ColliderMass::DensityKgM3(x) if x > 0.);
            if (positive && (mass.mass() <= 0. || mass.inv_principal_inertia.min_element() <= 0.))
                || !mass.mass().is_finite()
                || !mass.local_com.is_finite()
                || !mass.inv_mass.is_finite()
                || !mass.inv_principal_inertia.is_finite()
                || !mass.principal_inertia_local_frame.is_finite()
            {
                return Err("unrepresentable cooked collider mass properties");
            }
            // Native accumulation stores inverse mass: an overflowing sum may
            // become zero mass, which a later collider can replace with a finite
            // value. Check the independent nonnegative sum before that information
            // is lost. Initial and auxiliary zero masses remain valid.
            accumulated_mass_kg += mass.mass();
            if !accumulated_mass_kg.is_finite() {
                return Err("unrepresentable aggregate body mass properties");
            }
            total += mass;
        }
        if !total.mass().is_finite()
            || total.mass() <= 0.
            || !total.local_com.is_finite()
            || !total.inv_principal_inertia.is_finite()
            || total.inv_principal_inertia.min_element() <= 0.
            || !total.principal_inertia_local_frame.is_finite()
        {
            return Err("unrepresentable aggregate body mass properties");
        }
        Ok(Some(total))
    }

    pub(crate) fn shapes(&self) -> &[SharedShape] {
        &self.shapes
    }
}

/// Validate before native compound summation can interpret infinite inertia's
/// zero inverse as zero inertia. This is a preflight only: the installed compound
/// and its single collider retain the backend's exact mass allocation/arithmetic.
fn validate_compound_child_mass(
    shape: &SharedShape,
    policy: ColliderMass,
) -> Result<(), &'static str> {
    let density = match policy {
        ColliderMass::DensityKgM3(density) if density > 0. => density,
        ColliderMass::MassKg(mass) if mass > 0. => 1.,
        _ => return Ok(()),
    };
    let Some(compound) = shape.as_compound() else {
        return Ok(());
    };
    let validate = |mass: &MassProperties| {
        if mass.mass() <= 0.
            || !mass.mass().is_finite()
            || !mass.local_com.is_finite()
            || !mass.inv_mass.is_finite()
            || !mass.inv_principal_inertia.is_finite()
            || !mass.principal_inertia().is_finite()
            || mass.inv_principal_inertia.min_element() <= 0.
            || !mass.principal_inertia_local_frame.is_finite()
        {
            Err("unrepresentable cooked compound child mass properties")
        } else {
            Ok(())
        }
    };
    let children = compound
        .shapes()
        .iter()
        .map(|(pose, shape)| {
            let mass = shape.mass_properties(density).transform_by(pose);
            validate(&mass)?;
            Ok(mass)
        })
        .collect::<Result<Vec<_>, &'static str>>()?;
    if let ColliderMass::MassKg(mass) = policy {
        // Match MassProperties::set_mass's inverse-inertia multiplier exactly.
        // Do not assign the whole collider mass independently to each leaf, or
        // recompute leaves at a different density than native unit-density cooking.
        let scale = mass.recip() * shape.mass_properties(1.).mass();
        for mut child in children {
            child.inv_mass *= scale;
            child.inv_principal_inertia *= scale;
            validate(&child)?;
        }
    }
    Ok(())
}

/// Preserve one native shape per source collider, including source child order.
fn cook_shape(source: &ShapeDescription) -> Result<SharedShape, &'static str> {
    Ok(match source {
        ShapeDescription::Ball { radius_m } => SharedShape::ball(*radius_m),
        ShapeDescription::Cuboid { half_extents_m: h } => SharedShape::cuboid(h[0], h[1], h[2]),
        ShapeDescription::TriangleMesh(mesh) => cook_surface(mesh)?,
        ShapeDescription::ConvexHull(hull) => cook_convex(hull)?,
        ShapeDescription::Compound(compound) => {
            let children = compound
                .children()
                .iter()
                .map(|(pose, shape)| {
                    let shape = cook_shape(shape)?;
                    // Parry panics for composite children (including triangle meshes).
                    // Reject before native construction; never flatten or split them.
                    if shape.as_composite_shape().is_some() {
                        return Err("backend does not support composite children in compounds");
                    }
                    let [w, x, y, z] = pose.rotation_wxyz;
                    Ok((
                        Pose3::from_parts(
                            Vector::from_array(pose.translation_m),
                            Rotation::from_xyzw(x, y, z, w),
                        ),
                        shape,
                    ))
                })
                .collect::<Result<Vec<_>, &'static str>>()?;
            SharedShape::compound(children)
        }
    })
}

fn native_combine(rule: sim_engine::model::CoefficientCombineRule) -> CoefficientCombineRule {
    use sim_engine::model::CoefficientCombineRule as Source;
    match rule {
        Source::Average => CoefficientCombineRule::Average,
        Source::Min => CoefficientCombineRule::Min,
        Source::Multiply => CoefficientCombineRule::Multiply,
        Source::Max => CoefficientCombineRule::Max,
    }
}

/// Hull construction uses the original points, never a box/perturbed fallback.
/// Preflight follows the backend's normalization arithmetic, rejecting singular
/// or unrepresentable input before its geometric algorithm. Volume at unit
/// density is a dimensionality check only; installation retains explicit inertia.
fn cook_convex(source: &ConvexHull) -> Result<SharedShape, &'static str> {
    if u32::try_from(source.points_m().len()).is_err() {
        return Err("too many convex hull points");
    }
    let points: Vec<_> = source
        .points_m()
        .iter()
        .copied()
        .map(Vector::from_array)
        .collect();
    let min = points
        .iter()
        .copied()
        .fold(Vector::splat(f64::INFINITY), Vector::min);
    let max = points
        .iter()
        .copied()
        .fold(Vector::splat(f64::NEG_INFINITY), Vector::max);
    let center = (min + max) * 0.5;
    let diagonal = (max - min).length();
    if !center.is_finite() || !diagonal.is_finite() || diagonal <= 0. {
        return Err("convex hull normalization is singular or unrepresentable");
    }
    // A scaled affine-rank check also avoids invoking native hull construction
    // for a point, line or plane. It does not reorder or modify the source.
    let origin = (points[0] - center) / diagonal;
    let offset = |point: Vector| (point - center) / diagonal - origin;
    let edge = points
        .iter()
        .copied()
        .map(offset)
        .max_by(|a, b| a.length_squared().total_cmp(&b.length_squared()))
        .unwrap();
    let area = points
        .iter()
        .copied()
        .map(|point| edge.cross(offset(point)))
        .max_by(|a, b| a.length_squared().total_cmp(&b.length_squared()))
        .unwrap();
    if area.length_squared() == 0.
        || !points
            .iter()
            .any(|point| area.dot(offset(*point)).abs() > 0.)
    {
        return Err("convex hull points must span three dimensions");
    }
    let shape =
        SharedShape::convex_hull(&points).ok_or("native convex hull construction failed")?;
    let hull = shape
        .as_convex_polyhedron()
        .ok_or("native convex hull is not a polyhedron")?;
    let volume = shape.mass_properties(1.).mass();
    if hull.points().len() < 4
        || hull.faces().len() < 4
        || hull.points().iter().any(|point| !point.is_finite())
        || hull.faces().iter().any(|face| {
            !face.normal.is_finite() || (face.normal.length_squared() - 1.).abs() > 1e-6
        })
        || !volume.is_finite()
        || volume <= 0.
    {
        return Err("native convex hull is degenerate or unrepresentable");
    }
    Ok(shape)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_engine::{
        Pose,
        model::{BodyKind, ColliderDescription, ContactMaterial, Inertial},
    };

    fn compound_fixture() -> (ShapeDescription, SharedShape) {
        use sim_engine::model::CompoundShape;
        // Unequal offset, rotated convex leaves expose order, frame and inertia
        // changes that symmetric primitives or identity mounts would conceal.
        let points = [
            vec![[0., 0., 0.], [0.3, 0., 0.], [0., 0.5, 0.], [0., 0., 0.7]],
            vec![[0., 0., 0.], [0.4, 0., 0.], [0., 0.2, 0.], [0., 0., 0.6]],
        ];
        let poses = [
            Pose {
                translation_m: [0.2, -0.4, 0.6],
                rotation_wxyz: [0.5, 0.5, 0.5, 0.5],
            },
            Pose {
                translation_m: [-0.3, 0.8, -0.2],
                rotation_wxyz: [1. + f64::EPSILON, 0., 0., 0.],
            },
        ];
        let source = CompoundShape::new(
            poses
                .into_iter()
                .zip(&points)
                .map(|(pose, points)| {
                    (
                        pose,
                        ShapeDescription::ConvexHull(ConvexHull::new(points.clone()).unwrap()),
                    )
                })
                .collect(),
        )
        .unwrap();
        let native = SharedShape::compound(
            poses
                .into_iter()
                .zip(&points)
                .map(|(pose, points)| {
                    let [w, x, y, z] = pose.rotation_wxyz;
                    (
                        Pose3::from_parts(
                            Vector::from_array(pose.translation_m),
                            Rotation::from_xyzw(x, y, z, w),
                        ),
                        SharedShape::convex_hull(
                            &points
                                .iter()
                                .copied()
                                .map(Vector::from_array)
                                .collect::<Vec<_>>(),
                        )
                        .unwrap(),
                    )
                })
                .collect(),
        );
        (ShapeDescription::Compound(source), native)
    }

    fn compound_collider(shape: ShapeDescription) -> ColliderDescription {
        ColliderDescription::new(
            shape,
            Pose {
                translation_m: [-1., 2., 0.3],
                rotation_wxyz: [0.5, -0.5, 0.5, 0.5],
            },
            ContactMaterial {
                friction: 0.4,
                restitution: 0.,
                memberships: u32::MAX,
                filter: u32::MAX,
            },
        )
        .unwrap()
    }

    #[test]
    fn compound_cooking_matches_native_order_frames_shapes_and_single_collider_mass() {
        use sim_engine::model::ColliderPolicy;
        let (source, native) = compound_fixture();
        for mass in [ColliderMass::MassKg(3.7), ColliderMass::DensityKgM3(11.)] {
            let collider = compound_collider(source.clone())
                .with_policy(ColliderPolicy {
                    mass,
                    ..Default::default()
                })
                .unwrap();
            let cooked = CookedRigidBody::new(
                RigidBodyDescription::new(BodyKind::DynamicFromColliders, vec![collider.clone()])
                    .unwrap(),
            )
            .unwrap();
            let shared = cooked.clone();
            assert!(Arc::ptr_eq(&cooked.shapes, &shared.shapes));
            assert!(Arc::ptr_eq(&cooked.shapes()[0].0, &shared.shapes()[0].0));
            let actual = cooked.shapes()[0].as_compound().unwrap().shapes();
            let expected = native.as_compound().unwrap().shapes();
            assert_eq!(actual.len(), 2);
            for ((actual_pose, actual_shape), (expected_pose, expected_shape)) in
                actual.iter().zip(expected)
            {
                assert_eq!(actual_pose, expected_pose);
                assert_eq!(
                    actual_pose.rotation.w.to_bits(),
                    expected_pose.rotation.w.to_bits()
                );
                let actual_hull = actual_shape.as_convex_polyhedron().unwrap();
                let expected_hull = expected_shape.as_convex_polyhedron().unwrap();
                assert_eq!(actual_hull.points(), expected_hull.points());
                assert_eq!(actual_hull.faces().len(), expected_hull.faces().len());
                assert_eq!(
                    actual_shape.mass_properties(1.),
                    expected_shape.mass_properties(1.)
                );
            }
            let builders: Vec<_> = cooked.native_collider_builders().collect();
            assert_eq!(builders.len(), 1);
            let actual = builders.into_iter().next().unwrap().build();
            let [w, x, y, z] = collider.pose().rotation_wxyz;
            let expected = ColliderBuilder::new(native.clone()).position(Pose3::from_parts(
                Vector::from_array(collider.pose().translation_m),
                Rotation::from_xyzw(x, y, z, w),
            ));
            let expected = match mass {
                ColliderMass::MassKg(mass) => expected.mass(mass),
                ColliderMass::DensityKgM3(density) => expected.density(density),
                _ => unreachable!(),
            }
            .build();
            assert_eq!(actual.mass_properties(), expected.mass_properties());
            let expected_mass = expected.mass_properties().transform_by(expected.position());
            assert_eq!(
                cooked.native_aggregate_mass_properties(),
                Some(&expected_mass)
            );
            // Native insertion refreshes body mass once for this one collider.
            let mut bodies = rapier3d_f64::prelude::RigidBodySet::new();
            let mut colliders = rapier3d_f64::prelude::ColliderSet::new();
            let body = bodies.insert(cooked.native_body_builder());
            colliders.insert_with_parent(actual, body, &mut bodies);
            let native_body = bodies.insert(RigidBodyBuilder::dynamic());
            colliders.insert_with_parent(expected, native_body, &mut bodies);
            bodies[body].recompute_mass_properties_from_colliders(&colliders);
            bodies[native_body].recompute_mass_properties_from_colliders(&colliders);
            assert_eq!(bodies[body].colliders().len(), 1);
            assert_eq!(
                bodies[body].mass_properties(),
                bodies[native_body].mass_properties()
            );
        }
    }

    fn compound_balls(radii: &[f64], policy: ColliderMass) -> RigidBodyDescription {
        use sim_engine::model::{ColliderPolicy, CompoundShape};
        let shape = ShapeDescription::Compound(
            CompoundShape::new(
                radii
                    .iter()
                    .map(|radius_m| {
                        (
                            Pose::default(),
                            ShapeDescription::Ball {
                                radius_m: *radius_m,
                            },
                        )
                    })
                    .collect(),
            )
            .unwrap(),
        );
        let collider = compound_collider(shape)
            .with_policy(ColliderPolicy {
                mass: policy,
                ..Default::default()
            })
            .unwrap();
        RigidBodyDescription::new(BodyKind::DynamicFromColliders, vec![collider]).unwrap()
    }

    #[test]
    fn compound_child_inertia_overflow_cannot_be_hidden_by_a_small_child() {
        // Native summation treats the giant ball's zero inverse inertia as zero
        // inertia, so the small ball can make the final result look valid.
        let giant = SharedShape::ball(1e100);
        let giant_mass = giant.mass_properties(1.);
        assert!(giant_mass.mass().is_finite());
        assert_eq!(giant_mass.inv_principal_inertia, Vector::ZERO);
        let native = SharedShape::compound(vec![
            (Pose3::IDENTITY, giant.clone()),
            (Pose3::IDENTITY, SharedShape::ball(1.)),
        ]);
        assert!(
            native
                .mass_properties(1.)
                .inv_principal_inertia
                .min_element()
                > 0.
        );
        for radii in [&[1e100, 1.][..], &[1., 1e100][..], &[1e100][..]] {
            for policy in [
                ColliderMass::DensityKgM3(1.),
                ColliderMass::MassKg(1.),
                ColliderMass::MassKg(1e-100),
            ] {
                assert!(
                    CookedRigidBody::new(compound_balls(radii, policy)).is_err(),
                    "{radii:?}, {policy:?}"
                );
            }
        }
        // Standalone rejection remains consistent with compound rejection.
        let mut source = compound_balls(&[1.], ColliderMass::DensityKgM3(1.));
        let collider = ColliderDescription::new(
            ShapeDescription::Ball { radius_m: 1e100 },
            Pose::default(),
            source.colliders()[0].material(),
        )
        .unwrap()
        .with_policy(source.colliders()[0].policy())
        .unwrap();
        source = RigidBodyDescription::new(BodyKind::DynamicFromColliders, vec![collider]).unwrap();
        assert!(CookedRigidBody::new(source).is_err());
    }

    #[test]
    fn compound_child_validation_uses_native_policy_scaling() {
        // Unit-density inertia overflows, but direct density cooking remains
        // representable. Rejecting all unit-density-invalid sources would lose it.
        for (radii, policy) in [
            ([1e100, 1.], ColliderMass::DensityKgM3(1e-200)),
            ([1e50, 1e50], ColliderMass::MassKg(1e-50)),
        ] {
            let cooked = CookedRigidBody::new(compound_balls(&radii, policy)).unwrap();
            let native = ColliderBuilder::new(cooked.shapes()[0].clone())
                .position(cooked.native_collider_builders().next().unwrap().position);
            let native = match policy {
                ColliderMass::DensityKgM3(density) => native.density(density),
                ColliderMass::MassKg(mass) => native.mass(mass),
                _ => unreachable!(),
            }
            .build();
            let expected = native.mass_properties().transform_by(native.position());
            assert_eq!(cooked.native_aggregate_mass_properties(), Some(&expected));
        }
        // Extreme mass scaling can underflow a tiny child's inverse inertia or
        // overflow it even when another child leaves a valid-looking aggregate.
        for (radii, mass) in [([1e-60, 1.], 1e-100), ([1e60, 1.], 1e250)] {
            assert!(
                CookedRigidBody::new(compound_balls(&radii, ColliderMass::MassKg(mass))).is_err()
            );
        }
    }

    #[test]
    fn compound_does_not_infer_mass_for_explicit_inertia_or_massless_shapes() {
        let (source, _) = compound_fixture();
        let inertia = Inertial::new(7., [2., 3., 4.], Pose::at([0.1, 0.2, 0.3])).unwrap();
        let cooked = CookedRigidBody::new(
            RigidBodyDescription::new(BodyKind::Dynamic(inertia), vec![compound_collider(source)])
                .unwrap(),
        )
        .unwrap();
        assert!(cooked.native_aggregate_mass_properties().is_none());
        let collider = cooked.native_collider_builders().next().unwrap().build();
        assert_eq!(collider.mass(), 0.);
        let expected = MassProperties::with_principal_inertia_frame(
            Vector::from_array(inertia.frame().translation_m),
            inertia.mass_kg(),
            Vector::from_array(inertia.principal_inertia_kg_m2()),
            crate::runtime::math::pose(inertia.frame()).rotation,
        );
        let mut bodies = rapier3d_f64::prelude::RigidBodySet::new();
        let mut colliders = rapier3d_f64::prelude::ColliderSet::new();
        let body = bodies.insert(cooked.native_body_builder());
        colliders.insert_with_parent(collider, body, &mut bodies);
        bodies[body].recompute_mass_properties_from_colliders(&colliders);
        let actual = &bodies[body].mass_properties().local_mprops;
        assert_eq!(actual.mass(), expected.mass());
        assert_eq!(actual.local_com, expected.local_com);
        // Native aggregation may choose equivalent principal-axis signs.
        assert_eq!(
            actual.reconstruct_inertia_matrix(),
            expected.reconstruct_inertia_matrix()
        );
    }

    #[test]
    fn unsupported_composite_children_fail_before_install_and_bad_state_is_atomic() {
        use crate::runtime::{BodyKey, PhysicalRuntime};
        use sim_engine::model::{BodyState, CompoundShape};
        let (source, _) = compound_fixture();
        let triangle = ShapeDescription::TriangleMesh(
            TriangleMesh::new(vec![[0.; 3], [1., 0., 0.], [0., 1., 0.]], vec![[0, 1, 2]]).unwrap(),
        );
        let mut world = PhysicalRuntime::new(crate::StepDuration::HZ_128);
        for kind in [BodyKind::Fixed, BodyKind::Kinematic] {
            for child in [source.clone(), triangle.clone()] {
                let shape = ShapeDescription::Compound(
                    CompoundShape::new(vec![(Pose::default(), child)]).unwrap(),
                );
                let source =
                    RigidBodyDescription::new(kind, vec![compound_collider(shape)]).unwrap();
                assert!(matches!(
                    CookedRigidBody::new(source),
                    Err("backend does not support composite children in compounds")
                ));
                assert!(world.body_binding(BodyKey(99)).is_none());
            }
        }
        // A failed child hull also propagates as an error, without substitution.
        let bad_hull = ShapeDescription::ConvexHull(ConvexHull::new(vec![[0.; 3]; 4]).unwrap());
        let bad = ShapeDescription::Compound(
            CompoundShape::new(vec![(Pose::default(), bad_hull)]).unwrap(),
        );
        assert!(
            CookedRigidBody::new(
                RigidBodyDescription::new(BodyKind::Fixed, vec![compound_collider(bad)]).unwrap()
            )
            .is_err()
        );
        let cooked = CookedRigidBody::new(
            RigidBodyDescription::new(BodyKind::Fixed, vec![compound_collider(source)]).unwrap(),
        )
        .unwrap();
        let mut bad = BodyState::at(Pose::default());
        bad.pose.rotation_wxyz = [0.; 4];
        assert!(world.install_body(BodyKey(99), &cooked, bad).is_err());
        assert!(world.body_binding(BodyKey(99)).is_none());
        assert!(
            world
                .geometry_capture()
                .cast_ray(&crate::queries::RayQuery::new([-2., 2., 0.3], [1., 0., 0.], 4.).unwrap())
                .is_none()
        );
        let binding = world
            .install_body(BodyKey(99), &cooked, BodyState::at(Pose::default()))
            .unwrap();
        assert_eq!(world.body_binding(BodyKey(99)), Some(binding));
    }

    fn aggregate(half: [f64; 3], mass: sim_engine::model::ColliderMass) -> RigidBodyDescription {
        use sim_engine::model::ColliderPolicy;
        RigidBodyDescription::new(
            BodyKind::DynamicFromColliders,
            vec![
                ColliderDescription::new(
                    ShapeDescription::Cuboid {
                        half_extents_m: half,
                    },
                    Pose::default(),
                    ContactMaterial {
                        friction: 0.4,
                        restitution: 0.,
                        memberships: u32::MAX,
                        filter: u32::MAX,
                    },
                )
                .unwrap()
                .with_policy(ColliderPolicy {
                    mass,
                    ..Default::default()
                })
                .unwrap(),
            ],
        )
        .unwrap()
    }

    #[test]
    fn cooked_mass_rejects_overflow_and_underflow_without_silent_mass_loss() {
        use sim_engine::model::ColliderMass;
        for (half, mass) in [
            ([1.; 3], ColliderMass::DensityKgM3(f64::MAX)),
            ([1e-150; 3], ColliderMass::DensityKgM3(1.)),
            ([1e150; 3], ColliderMass::MassKg(1.)),
            ([1.; 3], ColliderMass::MassKg(f64::from_bits(1))),
        ] {
            assert!(
                CookedRigidBody::new(aggregate(half, mass)).is_err(),
                "{half:?}, {mass:?}"
            );
        }
        assert!(CookedRigidBody::new(aggregate([1.; 3], ColliderMass::DensityKgM3(3.))).is_ok());
    }

    #[test]
    fn aggregate_mass_overflow_cannot_be_hidden_by_later_contributions() {
        use sim_engine::model::{ColliderMass, ColliderPolicy};
        let source = aggregate([0.5; 3], ColliderMass::MassKg(1e308));
        // Each contribution is representable on its own. Native inverse-mass
        // accumulation can turn the first overflowing pair into zero mass and
        // then report a finite final mass after adding the third collider.
        assert!(CookedRigidBody::new(source.clone()).is_ok());
        let collider = source.colliders()[0].clone();
        let overflowing = RigidBodyDescription::new(
            BodyKind::DynamicFromColliders,
            vec![collider.clone(), collider.clone(), collider.clone()],
        )
        .unwrap();
        assert!(CookedRigidBody::new(overflowing).is_err());

        let zero = collider
            .clone()
            .with_policy(ColliderPolicy {
                mass: ColliderMass::Zero,
                ..Default::default()
            })
            .unwrap();
        let explicit_zero = collider
            .clone()
            .with_policy(ColliderPolicy {
                mass: ColliderMass::MassKg(0.),
                ..Default::default()
            })
            .unwrap();
        let valid = RigidBodyDescription::new(
            BodyKind::DynamicFromColliders,
            vec![
                zero.clone(),
                explicit_zero.clone(),
                collider,
                zero,
                explicit_zero,
            ],
        )
        .unwrap();
        let cooked = CookedRigidBody::new(valid).unwrap();
        assert_eq!(
            cooked.native_aggregate_mass_properties().unwrap().mass(),
            CookedRigidBody::new(source)
                .unwrap()
                .native_aggregate_mass_properties()
                .unwrap()
                .mass()
        );
    }

    #[test]
    fn native_helpers_preserve_source_rotation_bits_and_mass_mode() {
        use sim_engine::model::{ColliderMass, ColliderPolicy};
        let p = Pose {
            translation_m: [0.1, 0.2, 0.3],
            rotation_wxyz: [1. + f64::EPSILON, 0., 0., 0.],
        };
        let mut source = aggregate([1.; 3], ColliderMass::MassKg(2.));
        let zero = ColliderDescription::new(
            ShapeDescription::Ball { radius_m: 0.1 },
            p,
            ContactMaterial {
                friction: 0.,
                restitution: 0.,
                memberships: 1,
                filter: 2,
            },
        )
        .unwrap()
        .with_policy(ColliderPolicy {
            mass: ColliderMass::MassKg(0.),
            ..Default::default()
        })
        .unwrap();
        source = RigidBodyDescription::new(
            BodyKind::DynamicFromColliders,
            vec![source.colliders()[0].clone(), zero],
        )
        .unwrap();
        let cooked = CookedRigidBody::new(source).unwrap();
        let builders: Vec<_> = cooked.native_collider_builders().collect();
        assert_eq!(
            builders[1].position.rotation.w.to_bits(),
            p.rotation_wxyz[0].to_bits()
        );
        assert!(matches!(
            builders[0].mass_properties,
            rapier3d_f64::prelude::ColliderMassProps::Mass(2.)
        ));
        assert!(matches!(
            builders[1].mass_properties,
            rapier3d_f64::prelude::ColliderMassProps::Mass(0.)
        ));
        // The builder alone deliberately retains native deferred COM timing.
        assert_eq!(cooked.native_body_builder().build().mass(), 0.);
    }

    fn body(points: Vec<[f64; 3]>) -> RigidBodyDescription {
        RigidBodyDescription::new(
            BodyKind::Dynamic(Inertial::new(2., [1.; 3], Pose::default()).unwrap()),
            vec![
                ColliderDescription::new(
                    ShapeDescription::ConvexHull(ConvexHull::new(points).unwrap()),
                    Pose::default(),
                    ContactMaterial {
                        friction: 0.4,
                        restitution: 0.,
                        memberships: u32::MAX,
                        filter: u32::MAX,
                    },
                )
                .unwrap(),
            ],
        )
        .unwrap()
    }
    #[test]
    fn lower_dimensional_and_unrepresentable_clouds_are_rejected_without_fallback() {
        for points in [
            vec![[0.; 3]; 4],
            vec![[0., 0., 0.], [1., 0., 0.], [2., 0., 0.], [3., 0., 0.]],
            vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]],
            vec![
                [0., 0., 0.],
                [1e-200, 0., 0.],
                [0., 1e-200, 0.],
                [0., 0., 1e-200],
            ],
            vec![
                [0., 0., 0.],
                [1e200, 0., 0.],
                [0., 1e200, 0.],
                [0., 0., 1e200],
            ],
            vec![
                [1e308, 0., 0.],
                [1e308, 1., 0.],
                [1e308, 0., 1.],
                [1e308, 1., 1.],
            ],
        ] {
            assert!(CookedRigidBody::new(body(points)).is_err());
        }
    }
    #[test]
    fn explicit_convex_cooking_preserves_source_points_and_shares_shapes() {
        let mut points = vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
        points.extend([[0., 0., 0.], [0.1, 0.1, 0.1]]);
        let source = body(points.clone());
        let cooked = CookedRigidBody::new(source.clone()).unwrap();
        let shared = cooked.clone();
        assert!(Arc::ptr_eq(&cooked.shapes()[0].0, &shared.shapes()[0].0));
        assert_eq!(cooked.source(), &source);
        let ShapeDescription::ConvexHull(hull) = cooked.source().colliders()[0].shape() else {
            panic!("expected source hull")
        };
        assert_eq!(hull.points_m(), points);
        assert!((cooked.shapes()[0].mass_properties(1.).mass() - 1. / 6.).abs() < 1e-12);
    }
}
