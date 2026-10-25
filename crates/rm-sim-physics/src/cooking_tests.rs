// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>

use super::*;
use sim_physics::backend::CookedMeshExt;
mod tests {
    use super::*;
    use crate::WorldPhysics;
    use std::sync::Arc;

    #[test]
    fn retains_source_and_shares_cooked_shape_between_independent_worlds() {
        let vertices = vec![[2., -1., -1.], [2., 1., -1.], [2., 0., 1.], [2., -1., -1.]];
        let triangles = vec![[3, 1, 2], [0, 0, 0]];
        let source = TriangleMesh::new(vertices.clone(), triangles.clone()).unwrap();
        let cooked = CookedTriangleMesh::new(source.clone()).unwrap();
        assert_eq!(source.vertices_m(), vertices);
        assert_eq!(source.triangles(), triangles);
        assert_eq!(
            cooked.source().vertices_m().as_ptr(),
            source.vertices_m().as_ptr()
        );
        let clone = cooked.clone();
        assert!(Arc::ptr_eq(&cooked.shape().0, &clone.shape().0));
        let mut worlds = [WorldPhysics::new(&[], -10.), WorldPhysics::new(&[], -10.)];
        let mut legacy = WorldPhysics::new(&[], -10.);
        legacy.add_static_mesh(vertices, triangles).unwrap();
        let expected = legacy
            .static_geometry_snapshot(true)
            .cast_ray([0.; 3], [1., 0., 0.], 5.)
            .unwrap();
        for world in &mut worlds {
            world.add_cooked_static_mesh(&cooked);
            let capture = world.static_geometry_snapshot(true);
            assert!(Arc::ptr_eq(&capture.shapes[0].1.0, &cooked.shape().0));
            assert_eq!(
                capture.cast_ray([0.; 3], [1., 0., 0.], 5.).unwrap(),
                expected
            );
            assert_eq!(
                capture
                    .cast_ray([4., 0., 0.], [-1., 0., 0.], 5.)
                    .unwrap()
                    .unwrap()
                    .distance_m,
                2.
            );
        }
        drop(cooked);
        drop(clone);
        drop(source);
        worlds[0].add_static_box(crate::Pose::at([1., 0., 0.]), [0.1; 3]);
        assert_eq!(worlds[1].static_geometry_snapshot(true).shapes.len(), 1);
        assert_eq!(
            worlds[1]
                .static_geometry_snapshot(true)
                .cast_ray([0.; 3], [1., 0., 0.], 5.)
                .unwrap(),
            expected
        );
    }

    #[test]
    fn source_validation_and_backend_cookability_are_separate() {
        assert!(CookedTriangleMesh::new(TriangleMesh::new(vec![], vec![]).unwrap()).is_err());
        let mut world = WorldPhysics::new(&[], -10.);
        assert_eq!(
            world
                .add_static_mesh(vec![[f64::NAN; 3]], vec![])
                .unwrap_err(),
            "mesh vertices must be finite"
        );
        assert_eq!(
            world.add_static_mesh(vec![], vec![[0; 3]]).unwrap_err(),
            "mesh triangle index out of range"
        );
        assert!(world.static_geometry_snapshot(true).shapes.is_empty());
    }
}
