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

#[cfg(test)]
mod heightfield_tests {
    use crate::cooking::CookedRigidBody;
    use crate::{Pose, WorldPhysics};
    use sim_engine::model::{BodyKind, RigidBodyDescription, ShapeDescription};
    use sim_engine::model::{ColliderDescription, ContactMaterial, HeightField};
    fn material() -> ContactMaterial {
        ContactMaterial {
            friction: 0.5,
            restitution: 0.,
            memberships: 3,
            filter: 7,
        }
    }
    #[test]
    fn finite_heightfield_queries_capture_and_diagonal_match_the_mesh() {
        let grid = HeightField::new(vec![vec![0., 0.1, 0.], vec![0.2, 0., 0.3]], 2., 1.).unwrap();
        let mounted = Pose {
            translation_m: [1., 2., 0.4],
            rotation_wxyz: [
                std::f64::consts::FRAC_1_SQRT_2,
                std::f64::consts::FRAC_1_SQRT_2,
                0.,
                0.,
            ],
        };
        let mut worlds = Vec::new();
        for shape in [
            ShapeDescription::HeightField(grid.clone()),
            ShapeDescription::TriangleMesh(grid.to_triangle_mesh()),
        ] {
            let model = CookedRigidBody::new(
                RigidBodyDescription::new(
                    BodyKind::Fixed,
                    vec![ColliderDescription::new(shape, Pose::default(), material()).unwrap()],
                )
                .unwrap(),
            )
            .unwrap();
            let mut w = WorldPhysics::new(&[], -10.);
            w.remove_catch_floor();
            w.add_fixed_model(&model, mounted).unwrap();
            worlds.push(w);
        }
        let native = worlds[0].static_geometry_snapshot(true);
        let mesh = worlds[1].static_geometry_snapshot(true);
        assert_eq!(native.clone().into_triangles().1.len(), 4);
        for x in [0.1, 0.7, 1.1, 1.8] {
            for y in [1.6, 1.9, 2.3] {
                for (z, dir) in [(3., -1.), (-1., 1.)] {
                    let a = native
                        .cast_ray([x, y, z], [0., 0., dir], 6.)
                        .unwrap()
                        .unwrap();
                    let b = mesh
                        .cast_ray([x, y, z], [0., 0., dir], 6.)
                        .unwrap()
                        .unwrap();
                    assert!((a.distance_m - b.distance_m).abs() < 1e-10);
                }
            }
        }
        assert!(
            native
                .cast_ray([4., 2., 3.], [0., 0., -1.], 6.)
                .unwrap()
                .is_none()
        );
        assert!(matches!(
            native.shapes[0].1.as_typed_shape(),
            rapier3d_f64::prelude::TypedShape::HeightField(_)
        ));
        assert_eq!(native.collision_groups, mesh.collision_groups);
    }
    #[test]
    fn heightfields_reject_dynamic_and_compound_surface_models_without_panics() {
        use sim_engine::model::CompoundShape;
        let shape =
            ShapeDescription::HeightField(HeightField::new(vec![vec![0.; 2]; 2], 2., 2.).unwrap());
        let collider =
            ColliderDescription::new(shape.clone(), Pose::default(), material()).unwrap();
        assert!(RigidBodyDescription::new(BodyKind::DynamicFromColliders, vec![collider]).is_err());
        let compound = CompoundShape::new(vec![(Pose::default(), shape)]).unwrap();
        let source = RigidBodyDescription::new(
            BodyKind::Fixed,
            vec![
                ColliderDescription::new(
                    ShapeDescription::Compound(compound),
                    Pose::default(),
                    material(),
                )
                .unwrap(),
            ],
        )
        .unwrap();
        assert!(CookedRigidBody::new(source).is_err());
    }
}
