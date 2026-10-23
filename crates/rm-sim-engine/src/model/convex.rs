// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Explicit convex-envelope sources, independent of backend hull construction.
use std::sync::Arc;

/// Immutable source points for an explicitly requested convex hull, in local FLU
/// metres. Clones share the source allocation; order, duplicates and interior
/// points are retained exactly.
///
/// Cooking takes the convex envelope, filling any concavities. Source validation
/// only requires at least four finite points: full dimensionality and numerical
/// tolerances belong to the backend, which may reject degenerate or
/// unrepresentable point clouds. No hull algorithm, tessellation inference, point
/// perturbation or fallback shape is supplied here. Mass and inertia must be
/// supplied separately for dynamic bodies; they are never synthesized from points.
///
/// ```
/// use rm_sim_engine::model::{ConvexHull, ShapeDescription};
/// let hull = ConvexHull::new(vec![
///     [0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.],
/// ])?;
/// let shared = hull.clone();
/// assert_eq!(hull.points_m().as_ptr(), shared.points_m().as_ptr());
/// let shape = ShapeDescription::ConvexHull(hull);
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct ConvexHull(Arc<Vec<[f64; 3]>>);

impl ConvexHull {
    /// Take ownership of at least four finite points in local FLU metres without
    /// copying or changing them. Backend cookability is not checked here.
    pub fn new(points_m: Vec<[f64; 3]>) -> Result<Self, &'static str> {
        if points_m.len() < 4 {
            return Err("convex hull requires at least four points");
        }
        if !points_m.iter().flatten().all(|x| x.is_finite()) {
            return Err("convex hull points must be finite");
        }
        Ok(Self(Arc::new(points_m)))
    }

    /// Original source points in local FLU metres, in caller-supplied order.
    pub fn points_m(&self) -> &[[f64; 3]] {
        &self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn requires_four_points_but_leaves_dimensionality_and_scale_to_backend() {
        for count in 0..4 {
            assert!(ConvexHull::new(vec![[0.; 3]; count]).is_err());
        }
        for coordinate in [0., f64::from_bits(1), f64::MAX] {
            let points = vec![[coordinate; 3]; 4];
            assert_eq!(ConvexHull::new(points.clone()).unwrap().points_m(), points);
        }
    }

    #[test]
    fn rejects_nonfinite_values_on_every_axis() {
        for nonfinite in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for axis in 0..3 {
                let mut points = vec![[0.; 3]; 4];
                points[3][axis] = nonfinite;
                assert!(ConvexHull::new(points).is_err());
            }
        }
    }

    #[test]
    fn retains_source_order_duplicates_and_interior_points_with_shared_clones() {
        let mut source = vec![
            [0., 0., 1.],
            [0., 0., 0.],
            [0.1, 0.1, 0.1],
            [1., 0., 0.],
            [0., 1., 0.],
            [0., 0., 0.],
        ];
        let hull = ConvexHull::new(source.clone()).unwrap();
        assert_eq!(hull.points_m(), source);
        assert_ne!(hull.points_m().as_ptr(), source.as_ptr());
        source[0] = [2.; 3];
        assert_eq!(hull.points_m()[0], [0., 0., 1.]);

        let shared = hull.clone();
        assert_eq!(shared, hull);
        assert_eq!(hull.points_m().as_ptr(), shared.points_m().as_ptr());
        drop(hull);
        assert_eq!(shared.points_m()[2], [0.1, 0.1, 0.1]);
        assert_eq!(shared.points_m()[1], shared.points_m()[5]);
    }

    #[test]
    fn construction_keeps_owned_point_allocation() {
        let points = vec![[0.; 3]; 4];
        let pointer = points.as_ptr();
        let hull = ConvexHull::new(points).unwrap();
        assert_eq!(hull.points_m().as_ptr(), pointer);
    }
}
