// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Immutable query requests and capture-local results without public solver types.
//!
//! Requests validate/normalize once and can be reused across immutable scene
//! captures. Batch execution lives on [`crate::StaticGeometry::cast_rays_into`];
//! it does not step, capture a world, schedule sensors or own host time. This is
//! a CPU query contract, not a general backend trait or device-buffer interface.
use crate::projectile::vector;
use rapier3d_f64::prelude::Ray;

/// Validated finite metric ray with a normalized, dimensionless FLU direction.
/// Fields are immutable so batch execution needs no repeated validation.
/// Range is inclusive and strictly positive; an origin inside a solid hits at zero.
/// Construction allocates no storage and does not query any scene.
///
/// ```
/// use rm_simulator_physics::queries::RayQuery;
/// let ray = RayQuery::new([0., 0., 1.], [20., 0., 0.], 5.)?;
/// assert_eq!(ray.direction(), [1., 0., 0.]);
/// assert_eq!(ray.max_distance_m(), 5.);
/// assert!(RayQuery::new([0.; 3], [0.; 3], 5.).is_err());
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayQuery {
    origin_m: [f64; 3],
    direction: [f64; 3],
    max_distance_m: f64,
}
impl RayQuery {
    /// Validate finite origin/direction and positive finite range in metres.
    /// Direction scale is irrelevant. Scaling before normalization avoids
    /// overflow and underflow for extreme finite nonzero directions.
    pub fn new(
        origin_m: [f64; 3],
        direction: [f64; 3],
        max_distance_m: f64,
    ) -> Result<Self, &'static str> {
        if !origin_m.into_iter().chain(direction).all(f64::is_finite)
            || !max_distance_m.is_finite()
            || max_distance_m <= 0.
        {
            return Err("ray requires finite origin/direction and a positive finite range");
        }
        let direction = vector(direction);
        let scale = direction.abs().max_element();
        if scale == 0. {
            return Err("ray direction must be nonzero");
        }
        Ok(Self {
            origin_m,
            direction: (direction / scale).normalize().to_array(),
            max_distance_m,
        })
    }
    /// World FLU ray origin, metres.
    pub fn origin_m(&self) -> [f64; 3] {
        self.origin_m
    }
    /// Normalized, dimensionless world FLU direction.
    pub fn direction(&self) -> [f64; 3] {
        self.direction
    }
    /// Inclusive positive metric range, metres.
    pub fn max_distance_m(&self) -> f64 {
        self.max_distance_m
    }
    pub(crate) fn metric_ray(&self) -> Ray {
        Ray::new(vector(self.origin_m), vector(self.direction))
    }
}

/// Closest surface returned by a metric ray query over one geometry capture.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    /// Distance along the normalized ray, in metres, including zero if inside a solid.
    pub distance_m: f64,
    /// World FLU intersection point, metres.
    pub position_m: [f64; 3],
    /// World FLU surface normal; may be zero/unreliable for an origin inside a solid.
    pub normal: [f64; 3],
    /// Shape ordinal in this capture, stable only while that capture is unchanged.
    /// This is not a robot identity or persistent semantic part ID.
    pub shape_index: usize,
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Pose, WorldPhysics};

    #[test]
    fn batches_keep_order_ties_range_edges_and_caller_storage() {
        let mut world = WorldPhysics::new(&[], 0.);
        for _ in 0..2 {
            world.add_static_box(Pose::at([2., 0., 1.]), [1.; 3]);
        }
        let scene = world.static_geometry_snapshot(true);
        let rays = [
            RayQuery::new([0., 0., 1.], [f64::MAX, 0., 0.], 1.5).unwrap(),
            RayQuery::new([0., 0., 1.], [0., 1., 0.], 10.).unwrap(),
            RayQuery::new([2., 0., 1.], [1e-320, 0., 0.], 10.).unwrap(),
            RayQuery::new([0., 0., 1.], [1., 0., 0.], 1.49).unwrap(),
        ];
        let mut hits = vec![None; rays.len()];
        let storage = hits.as_ptr();
        for _ in 0..4 {
            scene.cast_rays_into(&rays, &mut hits).unwrap();
            assert_eq!(hits.as_ptr(), storage);
            let first = hits[0].unwrap();
            assert_eq!(first.distance_m, 1.5);
            assert_eq!(first.shape_index, 0);
            assert_eq!(first.normal, [-1., 0., 0.]);
            assert_eq!(first.position_m, [1.5, 0., 1.]);
            assert!(hits[1].is_none());
            assert_eq!(hits[2].unwrap().distance_m, 0.);
            assert!(hits[3].is_none());
        }
        let previous = hits.clone();
        assert!(scene.cast_rays_into(&rays[..1], &mut hits).is_err());
        assert_eq!(hits, previous);
        scene.cast_rays_into(&[], &mut []).unwrap();
    }

    #[test]
    fn prepared_requests_do_not_cache_scene_results_or_solver_handles() {
        let mut world = WorldPhysics::new(&[], 0.);
        world.add_static_box(Pose::at([2., 0., 1.]), [1.; 3]);
        let before = world.static_geometry_snapshot(true);
        let rays = [RayQuery::new([0., 0., 1.], [2., 0., 0.], 5.).unwrap()];
        world.add_static_box(Pose::at([1., 0., 1.]), [1.; 3]);
        let after = world.static_geometry_snapshot(true);
        let mut hits = [None];
        for (scene, distance) in [(&before, 1.5), (&after, 0.5), (&before, 1.5)] {
            scene.cast_rays_into(&rays, &mut hits).unwrap();
            assert_eq!(hits[0].unwrap().distance_m, distance);
        }
    }

    #[test]
    fn query_validation_rejects_invalid_inputs_before_execution() {
        for (origin, direction, range) in [
            ([f64::NAN, 0., 0.], [1., 0., 0.], 1.),
            ([0.; 3], [f64::INFINITY, 0., 0.], 1.),
            ([0.; 3], [0.; 3], 1.),
            ([0.; 3], [1., 0., 0.], 0.),
            ([0.; 3], [1., 0., 0.], -1.),
            ([0.; 3], [1., 0., 0.], f64::INFINITY),
        ] {
            assert!(RayQuery::new(origin, direction, range).is_err());
        }
    }
}
