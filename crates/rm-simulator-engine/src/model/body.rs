// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Validated source collider and rigid-body descriptions; no runtime handles.
use super::{ConvexHull, TriangleMesh};
use crate::Pose;
use std::sync::Arc;

/// Geometry in a collider's local FLU frame. Construction of a
/// [`ColliderDescription`] validates primitive dimensions without changing them.
#[derive(Clone, Debug, PartialEq)]
pub enum ShapeDescription {
    /// Sphere with strictly positive radius in metres.
    Ball {
        /// Radius in metres.
        radius_m: f64,
    },
    /// Box with strictly positive local x/y/z half extents in metres.
    Cuboid {
        /// Half extents in metres.
        half_extents_m: [f64; 3],
    },
    /// Indexed surface, retaining its source vertices and winding.
    TriangleMesh(TriangleMesh),
    /// Explicit convex envelope of source points, filling concavities. Backend
    /// cooking may reject degenerate or unrepresentable point clouds; no
    /// tessellation inference, point perturbation or fallback shape is implied.
    ConvexHull(ConvexHull),
}

/// Contact coefficients and symmetric membership/filter bits for a collider.
/// Two colliders interact when each membership intersects the other's filter.
/// These bits have no implicit team, sensor or projectile meaning.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactMaterial {
    /// Dimensionless nonnegative Coulomb friction coefficient.
    pub friction: f64,
    /// Dimensionless restitution in the inclusive interval 0..=1.
    pub restitution: f64,
    /// Membership bitmask.
    pub memberships: u32,
    /// Accepted membership bitmask.
    pub filter: u32,
}

/// Immutable validated collider description. Pose maps local shape coordinates
/// into the body's FLU frame; material has no implied density or mass.
#[derive(Clone, Debug, PartialEq)]
pub struct ColliderDescription {
    shape: ShapeDescription,
    pose: Pose,
    material: ContactMaterial,
}
impl ColliderDescription {
    /// Validate geometry, a finite unit-quaternion mounting pose and contact data.
    /// Empty triangle meshes and degenerate convex point clouds remain source
    /// descriptions; a backend may reject cooking.
    pub fn new(
        shape: ShapeDescription,
        pose: Pose,
        material: ContactMaterial,
    ) -> Result<Self, &'static str> {
        match &shape {
            ShapeDescription::Ball { radius_m } if !positive(*radius_m) => {
                return Err("invalid sphere radius");
            }
            ShapeDescription::Cuboid { half_extents_m }
                if !half_extents_m.iter().copied().all(positive) =>
            {
                return Err("invalid box half extents");
            }
            _ => {}
        }
        validate_pose(pose)?;
        if !material.friction.is_finite()
            || material.friction < 0.
            || !material.restitution.is_finite()
            || !(0. ..=1.).contains(&material.restitution)
        {
            return Err("invalid contact material");
        }
        Ok(Self {
            shape,
            pose,
            material,
        })
    }
    /// Original immutable local geometry.
    pub fn shape(&self) -> &ShapeDescription {
        &self.shape
    }
    /// Shape-local to body-local mounting frame, in metres and wxyz quaternion.
    pub fn pose(&self) -> Pose {
        self.pose
    }
    /// Validated coefficients and explicit symmetric filter bits.
    pub fn material(&self) -> ContactMaterial {
        self.material
    }
}

/// Explicit rigid-body mass and principal moments, without CAD-derived defaults.
/// `frame` translates to the centre of mass and rotates principal axes into the
/// body frame. Principal moments are about the centre of mass in kg m².
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inertial {
    mass_kg: f64,
    principal_inertia_kg_m2: [f64; 3],
    frame: Pose,
}
impl Inertial {
    /// Require finite positive mass/moments, finite reciprocals, a unit frame and
    /// realizable principal moments (each no greater than the sum of the others).
    pub fn new(
        mass_kg: f64,
        principal_inertia_kg_m2: [f64; 3],
        frame: Pose,
    ) -> Result<Self, &'static str> {
        validate_pose(frame)?;
        if !positive(mass_kg)
            || !mass_kg.recip().is_finite()
            || !principal_inertia_kg_m2
                .iter()
                .copied()
                .all(|x| positive(x) && x.recip().is_finite())
        {
            return Err("invalid mass or principal inertia");
        }
        let scale = principal_inertia_kg_m2.into_iter().fold(0., f64::max);
        let i = principal_inertia_kg_m2.map(|x| x / scale);
        if (0..3).any(|axis| i[axis] > i[(axis + 1) % 3] + i[(axis + 2) % 3]) {
            return Err("unrealizable principal inertia");
        }
        Ok(Self {
            mass_kg,
            principal_inertia_kg_m2,
            frame,
        })
    }
    /// Total explicit mass in kilograms, independent of collider volume.
    pub fn mass_kg(self) -> f64 {
        self.mass_kg
    }
    /// Principal moments about the centre of mass, in kg m².
    pub fn principal_inertia_kg_m2(self) -> [f64; 3] {
        self.principal_inertia_kg_m2
    }
    /// Principal-axis/centre-of-mass frame relative to the body.
    pub fn frame(self) -> Pose {
        self.frame
    }
}

/// Rigid-body integration mode. Dynamic bodies require explicit inertial data.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BodyKind {
    /// Immovable rigid body.
    Fixed,
    /// Force-driven rigid body with explicitly supplied mass and inertia.
    Dynamic(Inertial),
    /// Prescribed endpoint motion; reactions do not alter its trajectory.
    Kinematic,
}

#[derive(Debug, PartialEq)]
struct BodyData {
    kind: BodyKind,
    colliders: Vec<ColliderDescription>,
}

/// Immutable physical body source. Clones share the complete description.
/// Initial world pose/velocity and runtime identity are supplied at installation.
/// Dynamic bodies support primitives and explicit convex hulls with mandatory
/// caller-supplied inertial data; dynamic triangle meshes are explicitly rejected.
/// Geometry never synthesizes mass or inertia.
///
/// ```
/// use rm_simulator_engine::model::{BodyKind, RigidBodyDescription};
/// let anchor = RigidBodyDescription::new(BodyKind::Fixed, vec![])?;
/// assert!(anchor.colliders().is_empty());
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct RigidBodyDescription(Arc<BodyData>);
impl RigidBodyDescription {
    /// Bind validated local colliders to a body kind. Collider-free anchors are
    /// permitted. Dynamic convex hulls require explicit inertial data through
    /// [`BodyKind::Dynamic`]; dynamic triangle surfaces remain unsupported.
    pub fn new(kind: BodyKind, colliders: Vec<ColliderDescription>) -> Result<Self, &'static str> {
        if matches!(kind, BodyKind::Dynamic(_))
            && colliders
                .iter()
                .any(|c| matches!(c.shape(), ShapeDescription::TriangleMesh(_)))
        {
            return Err("dynamic triangle surfaces are unsupported");
        }
        Ok(Self(Arc::new(BodyData { kind, colliders })))
    }
    /// Integration mode and explicit dynamic inertial data.
    pub fn kind(&self) -> BodyKind {
        self.0.kind
    }
    /// Source collider order, mounting frames and coefficients.
    pub fn colliders(&self) -> &[ColliderDescription] {
        &self.0.colliders
    }
}

/// Initial or observed physical state, distinct from the immutable description.
/// Coordinates and velocities are world FLU; pose is the body frame, not its COM.
/// Runtime mutation APIs validate public values before accepting them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyState {
    /// Body-to-world pose, in metres and wxyz quaternion.
    pub pose: Pose,
    /// World velocity of the centre of mass in metres/second.
    pub velocity_m_s: [f64; 3],
    /// World angular velocity in radians/second.
    pub angular_velocity_rad_s: [f64; 3],
}
impl BodyState {
    /// Resting initial state at a supplied body pose; validation occurs at install.
    pub fn at(pose: Pose) -> Self {
        Self {
            pose,
            velocity_m_s: [0.; 3],
            angular_velocity_rad_s: [0.; 3],
        }
    }
    /// Validate a finite pose with unit rotation and finite world velocities.
    pub fn validate(self) -> Result<(), &'static str> {
        validate_pose(self.pose)?;
        if !self
            .velocity_m_s
            .into_iter()
            .chain(self.angular_velocity_rad_s)
            .all(f64::is_finite)
        {
            return Err("nonfinite body velocity");
        }
        Ok(())
    }
}

fn positive(x: f64) -> bool {
    x.is_finite() && x > 0.
}
fn validate_pose(pose: Pose) -> Result<(), &'static str> {
    let norm = pose.rotation_wxyz.iter().map(|x| x * x).sum::<f64>();
    if !pose.translation_m.into_iter().all(f64::is_finite)
        || !norm.is_finite()
        || (norm - 1.).abs() > 1e-6
    {
        return Err("invalid body or collider pose");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn material() -> ContactMaterial {
        ContactMaterial {
            friction: 0.4,
            restitution: 0.2,
            memberships: u32::MAX,
            filter: u32::MAX,
        }
    }
    #[test]
    fn source_validation_rejects_invalid_dimensions_frames_and_mass() {
        for x in [0., -1., f64::NAN, f64::INFINITY] {
            assert!(
                ColliderDescription::new(
                    ShapeDescription::Ball { radius_m: x },
                    Pose::at([0.; 3]),
                    material()
                )
                .is_err()
            );
            assert!(
                ColliderDescription::new(
                    ShapeDescription::Cuboid {
                        half_extents_m: [1., x, 1.]
                    },
                    Pose::at([0.; 3]),
                    material()
                )
                .is_err()
            );
            assert!(Inertial::new(x, [1.; 3], Pose::at([0.; 3])).is_err());
            assert!(Inertial::new(1., [1., x, 1.], Pose::at([0.; 3])).is_err());
        }
        assert!(Inertial::new(1., [1., 1., 3.], Pose::at([0.; 3])).is_err());
        assert!(Inertial::new(f64::from_bits(1), [1.; 3], Pose::at([0.; 3])).is_err());
        let bad = Pose {
            translation_m: [0.; 3],
            rotation_wxyz: [0.; 4],
        };
        assert!(Inertial::new(1., [1.; 3], bad).is_err());
        assert!(BodyState::at(bad).validate().is_err());
        let bad_material = ContactMaterial {
            friction: f64::NAN,
            ..material()
        };
        assert!(
            ColliderDescription::new(
                ShapeDescription::Ball { radius_m: 1. },
                Pose::at([0.; 3]),
                bad_material
            )
            .is_err()
        );
        let bad_material = ContactMaterial {
            restitution: 1.01,
            ..material()
        };
        assert!(
            ColliderDescription::new(
                ShapeDescription::Ball { radius_m: 1. },
                Pose::at([0.; 3]),
                bad_material
            )
            .is_err()
        );
    }
    #[test]
    fn descriptions_share_ordered_colliders_and_require_explicit_surface_policy() {
        let mesh =
            TriangleMesh::new(vec![[0.; 3], [1., 0., 0.], [0., 1., 0.]], vec![[0, 1, 2]]).unwrap();
        let collider = ColliderDescription::new(
            ShapeDescription::TriangleMesh(mesh),
            Pose::at([1., 2., 3.]),
            material(),
        )
        .unwrap();
        let dynamic = BodyKind::Dynamic(Inertial::new(1., [1.; 3], Pose::at([0.; 3])).unwrap());
        assert!(RigidBodyDescription::new(dynamic, vec![collider.clone()]).is_err());
        let fixed = RigidBodyDescription::new(BodyKind::Fixed, vec![collider.clone()]).unwrap();
        let shared = fixed.clone();
        assert_eq!(fixed.colliders().as_ptr(), shared.colliders().as_ptr());
        assert_eq!(fixed.colliders(), &[collider]);
    }

    #[test]
    fn dynamic_convex_hulls_retain_explicit_inertia_and_source_points() {
        let hull =
            ConvexHull::new(vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]).unwrap();
        let collider = ColliderDescription::new(
            ShapeDescription::ConvexHull(hull.clone()),
            Pose::at([1., 2., 3.]),
            material(),
        )
        .unwrap();
        let inertial = Inertial::new(7., [2., 3., 4.], Pose::at([0.1, 0.2, 0.3])).unwrap();
        let body =
            RigidBodyDescription::new(BodyKind::Dynamic(inertial), vec![collider.clone()]).unwrap();
        assert_eq!(body.kind(), BodyKind::Dynamic(inertial));
        assert_eq!(body.colliders(), &[collider]);
        let ShapeDescription::ConvexHull(retained) = body.colliders()[0].shape() else {
            panic!("convex source must remain explicit");
        };
        assert_eq!(retained.points_m().as_ptr(), hull.points_m().as_ptr());
    }
}
