// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Compatibility exports and backend integration tests for solver-free query records.
pub use sim_engine::queries::{RayHit, RayQuery};

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
