// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared backend shapes for an explicitly described rigid body.
use super::*;
use rm_simulator_engine::model::{ConvexHull, RigidBodyDescription, ShapeDescription};
use std::sync::Arc;

/// Source body plus shared cooked collider shapes, with no world handles/state.
/// Cooking preserves local frames, source collider order and contact parameters.
/// No density-based mass estimation or inertia synthesis is performed.
#[derive(Clone)]
pub struct CookedRigidBody {
    source: RigidBodyDescription,
    shapes: Arc<[SharedShape]>,
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
        Ok(Self {
            source,
            shapes: shapes.into(),
        })
    }
    /// Original immutable physical description, including explicit inertia.
    pub fn source(&self) -> &RigidBodyDescription {
        &self.source
    }
    pub(crate) fn shapes(&self) -> &[SharedShape] {
        &self.shapes
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
