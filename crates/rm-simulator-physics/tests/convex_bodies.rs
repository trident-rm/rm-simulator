// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Exact chassis source-shape parity and ordinary generic convex-body dynamics.
#[allow(dead_code)]
#[path = "../examples/convex_body.rs"]
mod fixture;

use rapier3d_f64::prelude::{Pose as NativePose, Ray, Rotation, SharedShape, Vector};
use rm_simulator_engine::{
    Pose,
    model::{
        BodyKind, BodyState, ColliderDescription, ContactMaterial, ConvexHull, Inertial,
        RigidBodyDescription, ShapeDescription,
    },
};
use rm_simulator_physics::{
    Caliber, Shot, StepDuration,
    chassis::ChassisConfig,
    cooking::CookedRigidBody,
    runtime::{BodyKey, WorldPhysics},
    session::RobotSession,
};

fn native_pose(pose: Pose) -> NativePose {
    let [w, x, y, z] = pose.rotation_wxyz;
    NativePose::from_parts(
        Vector::from_array(pose.translation_m),
        Rotation::from_xyzw(x, y, z, w).normalize(),
    )
}

fn body(points: Vec<[f64; 3]>, kind: BodyKind, mount: Pose) -> CookedRigidBody {
    CookedRigidBody::new(
        RigidBodyDescription::new(
            kind,
            vec![
                ColliderDescription::new(
                    ShapeDescription::ConvexHull(ConvexHull::new(points).unwrap()),
                    mount,
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
        .unwrap(),
    )
    .unwrap()
}

fn volume(vertices: &[[f64; 3]], triangles: &[[u32; 3]]) -> f64 {
    // Shift the reference near the hull to avoid cancellation after placement.
    let reference = Vector::from_array(vertices[0]);
    triangles
        .iter()
        .map(|triangle| {
            let [a, b, c] = triangle.map(|i| Vector::from_array(vertices[i as usize]) - reference);
            a.dot(b.cross(c)) / 6.
        })
        .sum::<f64>()
        .abs()
}

fn assert_parity(points: Vec<[f64; 3]>, kind: BodyKind, mount: Pose, placement: Pose) {
    // Exactly the same points and ordering as chassis construction; no fitting,
    // centering, scaling, triangulation or replacement primitive on either path.
    let native_points: Vec<_> = points.iter().copied().map(Vector::from_array).collect();
    let direct = SharedShape::convex_hull(&native_points).unwrap();
    let cooked = body(points.clone(), kind, mount);
    let ShapeDescription::ConvexHull(source) = cooked.source().colliders()[0].shape() else {
        panic!("cooked source changed shape kind");
    };
    assert_eq!(source.points_m(), points);
    let mut world = WorldPhysics::with_step_duration(&[], -100., StepDuration::HZ_1000);
    world
        .install_body(BodyKey(1), &cooked, BodyState::at(placement))
        .unwrap();
    let capture = if matches!(kind, BodyKind::Fixed) {
        world.static_geometry_snapshot(true)
    } else {
        world.dynamic_geometry_snapshot()
    };
    let placed = native_pose(placement) * native_pose(mount);
    let (vertices, triangles) = direct.as_convex_polyhedron().unwrap().to_trimesh();
    let expected: Vec<_> = vertices
        .iter()
        .map(|v| placed.transform_point(*v).to_array())
        .collect();
    let (actual, actual_triangles) = capture.clone().into_triangles();
    assert_eq!(actual_triangles, triangles);
    assert_eq!(actual.len(), expected.len());
    for (actual, expected) in actual.iter().flatten().zip(expected.iter().flatten()) {
        assert!((actual - expected).abs() < 1e-12);
    }
    let bounds = direct.compute_aabb(&placed);
    for axis in 0..3 {
        let min = actual.iter().map(|v| v[axis]).fold(f64::INFINITY, f64::min);
        let max = actual
            .iter()
            .map(|v| v[axis])
            .fold(f64::NEG_INFINITY, f64::max);
        assert!((min - bounds.mins[axis]).abs() < 1e-12);
        assert!((max - bounds.maxs[axis]).abs() < 1e-12);
    }
    let direct_volume = volume(&expected, &triangles);
    assert!(direct_volume > 0.);
    assert!((volume(&actual, &actual_triangles) - direct_volume).abs() < direct_volume * 1e-10);

    // Probe every triangular face from outside, away from edges, including the
    // small bevel/guard faces that a few axial rays would miss.
    for triangle in &triangles {
        let [a, b, c] = triangle.map(|i| Vector::from_array(expected[i as usize]));
        let normal = (b - a).cross(c - a).normalize();
        let ray = Ray::new((a + b + c) / 3. + normal, -normal);
        let expected = direct
            .cast_ray_and_get_normal(&placed, &ray, 2., true)
            .unwrap();
        let actual = capture
            .cast_ray(ray.origin.to_array(), ray.dir.to_array(), 2.)
            .unwrap()
            .unwrap();
        assert!((actual.distance_m - expected.time_of_impact).abs() < 1e-10);
        for (actual, expected) in actual.normal.into_iter().zip(expected.normal.to_array()) {
            assert!((actual - expected).abs() < 1e-9);
        }
    }
    let miss = placed.transform_point(Vector::new(5., 5., 5.));
    assert!(
        capture
            .cast_ray(miss.to_array(), [0., 0., 1.], 1.)
            .unwrap()
            .is_none()
    );
}

#[test]
fn chassis_deck_and_every_bumper_preserve_native_hulls_and_local_mounts() {
    let config = ChassisConfig::default();
    assert!(config.omni_frame.is_some());
    let hz = config.body_half_m[2];
    let deck = config
        .body_outline_m()
        .into_iter()
        .flat_map(|[x, y]| [[x, y, -hz], [x, y, hz]])
        .collect();
    let mut parts = vec![deck];
    let bumpers = config.bumper_prisms_m();
    assert!(!bumpers.is_empty());
    parts.extend(bumpers.into_iter().map(Vec::from));
    // A rotated collider-local mount as well as a rotated body placement checks
    // composition order; neither is reduced to translation or yaw-only geometry.
    let angle: f64 = 0.37;
    let mount = Pose {
        translation_m: [0.13, -0.07, 0.21],
        rotation_wxyz: [(angle / 2.).cos(), (angle / 2.).sin(), 0., 0.],
    };
    for points in parts {
        assert_parity(
            points.clone(),
            BodyKind::Fixed,
            Pose::default(),
            Pose::default(),
        );
        assert_parity(
            points,
            BodyKind::Dynamic(Inertial::new(2., [1.; 3], Pose::default()).unwrap()),
            mount,
            Pose::yawed([2., -3., 1.], 0.63),
        );
    }
}

#[test]
fn prepared_convex_assembly_reuses_source_across_worlds_settles_and_resets() {
    let prepared = fixture::prepared().unwrap();
    let original = prepared.source().clone();
    let mut a = RobotSession::new(fixture::scenario(prepared.clone()).unwrap()).unwrap();
    let b = RobotSession::new(fixture::scenario(prepared.clone()).unwrap()).unwrap();
    let old = a.ground_truth().body_binding(fixture::BODY).unwrap();
    let other = b.ground_truth().body_binding(fixture::BODY).unwrap();
    assert_ne!(old, other);
    a.step(1000).unwrap();
    let settled = a.ground_truth().body_state(old).unwrap();
    assert!(
        (settled.pose.translation_m[2] - 0.1).abs() < 0.01,
        "{settled:?}"
    );
    assert!(settled.velocity_m_s[2].abs() < 0.02, "{settled:?}");
    assert_eq!(
        b.ground_truth()
            .body_state(other)
            .unwrap()
            .pose
            .translation_m,
        [0., 0., 1.]
    );
    assert_eq!(prepared.source(), &original);
    a.reset().unwrap();
    let rebound = a.ground_truth().body_binding(fixture::BODY).unwrap();
    assert_ne!(old, rebound);
    assert!(a.ground_truth().body_state(old).is_err());
    assert_eq!(
        a.ground_truth()
            .body_state(rebound)
            .unwrap()
            .pose
            .translation_m,
        [0., 0., 1.]
    );
    a.step(1000).unwrap();
    assert_eq!(a.ground_truth().body_state(rebound).unwrap(), settled);
}

#[test]
fn projectile_hits_generic_dynamic_convex_body_and_transfers_momentum() {
    let mut session =
        RobotSession::new(fixture::scenario(fixture::prepared().unwrap()).unwrap()).unwrap();
    let binding = session.ground_truth().body_binding(fixture::BODY).unwrap();
    // Cancel weight so this test isolates an ordinary ball/body collision,
    // rather than allowing the convex cube to fall out of the projectile path.
    session
        .physics_mut()
        .set_body_wrench(binding, [0., 0., 2. * 9.81], [0.; 3])
        .unwrap();
    session
        .physics_mut()
        .fire(
            0,
            Pose::at([-0.5, 0., 1.]),
            Shot::at_limit(Caliber::Mm17),
            None,
        )
        .unwrap();
    let mut slowed = false;
    let mut forward_body_velocity: f64 = 0.;
    for _ in 0..100 {
        session.step(1).unwrap();
        slowed |= session
            .ground_truth()
            .snapshot()
            .iter()
            .any(|p| p.velocity_m_s[0].abs() < 1.);
        forward_body_velocity = forward_body_velocity.max(
            session
                .ground_truth()
                .body_state(binding)
                .unwrap()
                .velocity_m_s[0],
        );
    }
    assert!(
        slowed,
        "projectile should lose forward speed on the convex surface"
    );
    assert!(forward_body_velocity > 0.01, "{forward_body_velocity}");
}
