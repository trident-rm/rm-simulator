// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared backend shapes for an explicitly described rigid body.
use super::*;
use rapier3d_f64::prelude::{
    CoefficientCombineRule, ColliderBuilder, Group, InteractionGroups, InteractionTestMode,
    MassProperties, Pose as Pose3, RigidBodyBuilder, Rotation,
};
use rm_simulator_engine::model::{
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
    ///
    /// ```
    /// use rm_simulator_engine::model::{BodyKind, RigidBodyDescription};
    /// use rm_simulator_physics::cooking::CookedRigidBody;
    /// let source = RigidBodyDescription::new(BodyKind::Fixed, vec![])?;
    /// let cooked = CookedRigidBody::new(source)?;
    /// assert!(cooked.source().colliders().is_empty());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn new(source: RigidBodyDescription) -> Result<Self, &'static str> {
        let shapes = source
            .colliders()
            .iter()
            .map(|c| {
                Ok(match c.shape() {
                    ShapeDescription::Ball { radius_m } => SharedShape::ball(*radius_m),
                    ShapeDescription::Cuboid { half_extents_m: h } => {
                        SharedShape::cuboid(h[0], h[1], h[2])
                    }
                    ShapeDescription::TriangleMesh(mesh) => cook_surface(mesh)?,
                    ShapeDescription::ConvexHull(hull) => cook_convex(hull)?,
                })
            })
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
    pub(crate) fn body_builder(&self) -> RigidBodyBuilder {
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
                        crate::projectile::pose(frame).rotation,
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
    pub(crate) fn collider_builders(&self) -> impl Iterator<Item = ColliderBuilder> + '_ {
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

    pub(crate) fn aggregate_mass_properties(&self) -> Option<&MassProperties> {
        self.aggregate_mass.as_ref()
    }

    fn validate_mass_properties(&self) -> Result<Option<MassProperties>, &'static str> {
        if !matches!(self.source.kind(), BodyKind::DynamicFromColliders) {
            return Ok(None);
        }
        // Match native insertion's ordered shape mass accumulation, without
        // touching a live world. Source frames and shape laws remain unchanged.
        let mut total = MassProperties::default();
        let mut accumulated_mass_kg = 0.;
        for (description, builder) in self.source.colliders().iter().zip(self.collider_builders()) {
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

fn native_combine(
    rule: rm_simulator_engine::model::CoefficientCombineRule,
) -> CoefficientCombineRule {
    use rm_simulator_engine::model::CoefficientCombineRule as Source;
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
    use rm_simulator_engine::{
        Pose,
        model::{BodyKind, ColliderDescription, ContactMaterial, Inertial},
    };

    fn aggregate(
        half: [f64; 3],
        mass: rm_simulator_engine::model::ColliderMass,
    ) -> RigidBodyDescription {
        use rm_simulator_engine::model::ColliderPolicy;
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
        use rm_simulator_engine::model::ColliderMass;
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
        use rm_simulator_engine::model::{ColliderMass, ColliderPolicy};
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
            cooked.aggregate_mass_properties().unwrap().mass(),
            CookedRigidBody::new(source)
                .unwrap()
                .aggregate_mass_properties()
                .unwrap()
                .mass()
        );
    }

    #[test]
    fn native_helpers_preserve_source_rotation_bits_and_mass_mode() {
        use rm_simulator_engine::model::{ColliderMass, ColliderPolicy};
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
        let builders: Vec<_> = cooked.collider_builders().collect();
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
        assert_eq!(cooked.body_builder().build().mass(), 0.);
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
