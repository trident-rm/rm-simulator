// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Frozen native constructor and prepared-resource regression tests.
use super::*;
use sim_physics::backend::{CookedBodyExt, RuntimeExt};

// Frozen 472b1d5 construction: deliberately does not call cooking or shared().
fn oracle_add(world: &mut WorldPhysics, state: &PayloadSnapshot) {
    let hulls = state
        .shape
        .hulls_m
        .iter()
        .map(|h| {
            let points = h
                .iter()
                .copied()
                .map(Vector::from_array)
                .collect::<Vec<_>>();
            (Pose3::IDENTITY, SharedShape::convex_hull(&points).unwrap())
        })
        .collect();
    let body = world.physical.world_mut().insert_body(
        RigidBodyBuilder::dynamic()
            .pose(pose(state.pose))
            .linvel(Vector::from_array(state.velocity_m_s))
            .angvel(Vector::from_array(state.angular_velocity_rad_s))
            .ccd_enabled(true),
    );
    let collider = world.physical.world_mut().insert_collider(
        ColliderBuilder::new(SharedShape::compound(hulls))
            .mass(state.shape.mass_kg)
            .friction(0.6)
            .restitution(0.05),
        Some(body),
    );
    world.payloads.push(Body {
        id: state.id,
        body,
        collider,
        shape: Arc::new(state.shape.clone()),
    });
    oracle_motion(
        world,
        state.id,
        state.prescribed,
        state.pose,
        state.velocity_m_s,
        state.angular_velocity_rad_s,
    );
}

// Keep the pre-port body-type transition and immediate/next pose ordering frozen.
fn oracle_motion(
    world: &mut WorldPhysics,
    id: u32,
    prescribed: bool,
    at: Pose,
    v: [f64; 3],
    w: [f64; 3],
) {
    let p = world.payloads.iter().find(|p| p.id == id).unwrap();
    let b = &mut world.physical.world_mut().bodies[p.body];
    let was = b.is_kinematic();
    b.set_body_type(
        if prescribed {
            RigidBodyType::KinematicPositionBased
        } else {
            RigidBodyType::Dynamic
        },
        true,
    );
    if prescribed {
        b.set_next_kinematic_position(pose(at));
        b.set_position(pose(at), true);
    } else if was {
        b.set_position(pose(at), true);
    }
    if prescribed || was {
        b.set_linvel(Vector::from_array(v), true);
        b.set_angvel(Vector::from_array(w), true);
    }
    world.physical.world_mut().colliders[p.collider].set_enabled(!prescribed);
}

fn fixture(prescribed: bool) -> PayloadSnapshot {
    let mut shape = PayloadShape::from_parts(
        &[CollisionPart::Box {
            half_m: [0.04, 0.03, 0.05],
        }],
        0.37,
    )
    .unwrap();
    // Asymmetric, off-origin, overlapping hulls detect COM and ordered native
    // compound mass aggregation changes. Deliberately scramble source order.
    for p in &mut shape.hulls_m[0] {
        p[0] += 0.017;
    }
    shape.hulls_m[0].reverse();
    let second = shape.hulls_m[0]
        .iter()
        .map(|p| [p[0] + 0.025, p[1] + 0.02, p[2] + 0.01])
        .collect();
    shape.hulls_m.push(second);
    PayloadSnapshot {
        id: 7,
        shape,
        pose: Pose::yawed([0.1, 0., 0.8], 0.4),
        velocity_m_s: [0.3, -0.1, 0.2],
        angular_velocity_rad_s: [0.1, 0.2, 0.4],
        prescribed,
    }
}

fn assert_equal(actual: &WorldPhysics, expected: &WorldPhysics) {
    assert_eq!(actual.payload(7), expected.payload(7));
    let a = &actual.payloads[0];
    let e = &expected.payloads[0];
    let actual_body = &actual.physical.world().bodies[a.body];
    let expected_body = &expected.physical.world().bodies[e.body];
    assert_eq!(format!("{actual_body:?}"), format!("{expected_body:?}"));
    let ca = &actual.physical.world().colliders[a.collider];
    let ce = &expected.physical.world().colliders[e.collider];
    assert_eq!(ca.mass_properties(), ce.mass_properties());
    assert_eq!(ca.is_enabled(), ce.is_enabled());
    assert_eq!(ca.friction(), ce.friction());
    assert_eq!(ca.restitution(), ce.restitution());
    let ah = ca.shape().as_compound().unwrap().shapes();
    let eh = ce.shape().as_compound().unwrap().shapes();
    assert_eq!(ah.len(), eh.len());
    for ((ap, a), (ep, e)) in ah.iter().zip(eh) {
        assert_eq!(ap, ep);
        assert_eq!(format!("{a:?}"), format!("{e:?}"));
    }
}

#[test]
fn prepared_payload_matches_frozen_native_before_after_mass_refresh_and_trajectory() {
    for prescribed in [false, true] {
        let initial = fixture(prescribed);
        let prepared = PreparedPayload::new(initial.shape.clone()).unwrap();
        let mut actual = WorldPhysics::new(&[], 0.);
        let mut expected = WorldPhysics::new(&[], 0.);
        actual.add_static_box(Pose::at([0., 0., 0.2]), [0.6, 0.6, 0.1]);
        expected.add_static_box(Pose::at([0., 0., 0.2]), [0.6, 0.6, 0.1]);
        actual
            .add_prepared_payload(
                7,
                &prepared,
                BodyState {
                    pose: initial.pose,
                    velocity_m_s: initial.velocity_m_s,
                    angular_velocity_rad_s: initial.angular_velocity_rad_s,
                },
                prescribed,
            )
            .unwrap();
        oracle_add(&mut expected, &initial);
        assert_equal(&actual, &expected);
        for world in [&mut actual, &mut expected] {
            let native = world.physical.world_mut();
            native.bodies[world.payloads[0].body]
                .recompute_mass_properties_from_colliders(&native.colliders);
        }
        assert_equal(&actual, &expected);
        let frames = TargetFrames::new(vec![]);
        for i in 0..384 {
            if [3, 180, 182].contains(&i) {
                let held = i == 180;
                let at = Pose::yawed([0.2, 0.1, 0.8], 0.7);
                let v = [0.1, 0.2, -0.3];
                let w = [0.2, 0.3, 0.4];
                actual.set_payload_motion(7, held, at, v, w).unwrap();
                oracle_motion(&mut expected, 7, held, at, v, w);
                assert_equal(&actual, &expected);
            }
            actual.step(i * tick_ns(), &frames).unwrap();
            expected.step(i * tick_ns(), &frames).unwrap();
            assert_equal(&actual, &expected);
        }
        assert!(actual.payload(7).unwrap().pose.translation_m[2] < 0.4);
    }
}

#[test]
fn prepared_clone_shares_raw_and_native_shapes_across_instances_worlds_and_reset() {
    let initial = fixture(false);
    let prepared = PreparedPayload::new(initial.shape.clone()).unwrap();
    let cloned = prepared.clone();
    assert!(std::ptr::eq(prepared.shape(), cloned.shape()));
    let sim_engine::model::ShapeDescription::Compound(compound) =
        prepared.cooked().source().colliders()[0].shape()
    else {
        panic!("single compound required")
    };
    for ((_, child), points) in compound.children().iter().zip(&initial.shape.hulls_m) {
        let sim_engine::model::ShapeDescription::ConvexHull(hull) = child else {
            panic!("convex leaf required")
        };
        assert_eq!(hull.points_m(), points);
    }
    let mut world = WorldPhysics::new(&[], 0.);
    for id in [1, 2] {
        world
            .add_prepared_payload(id, &cloned, BodyState::at(initial.pose), false)
            .unwrap();
    }
    for _ in 0..2 {
        let mut reset = WorldPhysics::new(&[], 0.);
        reset
            .add_prepared_payload(7, &prepared, BodyState::at(initial.pose), true)
            .unwrap();
        assert!(std::ptr::eq(
            world.payload_telemetry(1).unwrap().shape,
            reset.payload_telemetry(7).unwrap().shape
        ));
        for payload in world.payloads.iter().chain(&reset.payloads) {
            let scene = if payload.id == 7 { &reset } else { &world };
            let native = scene.physical.world().colliders[payload.collider].shared_shape();
            assert!(Arc::ptr_eq(&native.0, &prepared.cooked().shapes()[0].0));
        }
    }
    let owned = world.payload(1).unwrap();
    assert_eq!(owned.shape, initial.shape);
    assert!(!std::ptr::eq(&owned.shape, prepared.shape()));
}

#[test]
fn payload_preflight_rejects_invalid_resources_identity_and_state_without_mutation() {
    let initial = fixture(false);
    let prepared = PreparedPayload::new(initial.shape.clone()).unwrap();
    let mut world = WorldPhysics::new(&[], 0.);
    let before = (
        world.physical.world().bodies.len(),
        world.physical.world().colliders.len(),
    );
    let invalid = BodyState::at(Pose::at([f64::NAN, 0., 0.]));
    assert!(
        world
            .add_prepared_payload(7, &prepared, invalid, false)
            .is_err()
    );
    assert_eq!(
        before,
        (
            world.physical.world().bodies.len(),
            world.physical.world().colliders.len()
        )
    );
    let mut expansion = initial.shape.clone();
    expansion.max_expansion_m = 0.002;
    assert!(PreparedPayload::new(expansion).is_err());
    let mut mass = initial.shape.clone();
    mass.mass_kg = f64::INFINITY;
    assert!(PreparedPayload::new(mass).is_err());
    let mut degenerate = initial.shape.clone();
    degenerate.hulls_m[0] = vec![[0.; 3]; 4];
    assert!(PreparedPayload::new(degenerate).is_err());
    let mut too_many = initial.shape.clone();
    too_many.hulls_m = vec![too_many.hulls_m[0].clone(); 33];
    assert!(PreparedPayload::new(too_many).is_err());
    let mut too_large = initial.shape.clone();
    too_large.hulls_m[0] = vec![[0.; 3]; 8193];
    assert!(PreparedPayload::new(too_large).is_err());
    world.add_payload(&initial).unwrap();
    assert!(
        world
            .add_prepared_payload(7, &prepared, BodyState::at(initial.pose), false)
            .is_err()
    );
    assert_eq!(world.payload(7).unwrap(), initial);
}

#[test]
fn payload_limits_admit_flat_32_hulls_and_8192_ordered_vertices() {
    let mut shape = fixture(false).shape;
    let hull = shape.hulls_m[0].clone();
    shape.hulls_m = vec![hull.clone(); 32];
    shape.max_expansion_m = 0.001000001;
    assert_eq!(
        PreparedPayload::new(shape.clone())
            .unwrap()
            .cooked()
            .source()
            .colliders()
            .len(),
        1
    );
    shape.hulls_m = vec![hull.into_iter().cycle().take(8192).collect()];
    let prepared = PreparedPayload::new(shape.clone()).unwrap();
    assert_eq!(prepared.shape(), &shape);
    shape.hulls_m[0].push([0.; 3]);
    assert!(PreparedPayload::new(shape).is_err());
}
