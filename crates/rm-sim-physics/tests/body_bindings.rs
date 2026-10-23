// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Described bodies interact in the ordinary scene; bindings are not raw handles.
use rm_sim_engine::{
    Pose,
    model::{
        BodyKind, BodyState, ColliderDescription, ContactMaterial, Inertial, RigidBodyDescription,
        ShapeDescription,
    },
};
use rm_sim_physics::{
    StepDuration, TargetFrames,
    cooking::CookedRigidBody,
    runtime::{BodyKey, WorldPhysics},
    session::{RobotScenario, RobotSession},
};
fn model(kind: BodyKind, shape: ShapeDescription) -> CookedRigidBody {
    CookedRigidBody::new(
        RigidBodyDescription::new(
            kind,
            vec![
                ColliderDescription::new(
                    shape,
                    Pose::at([0.; 3]),
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
fn dynamic() -> BodyKind {
    BodyKind::Dynamic(Inertial::new(2., [2., 3., 4.], Pose::at([0.; 3])).unwrap())
}
fn world() -> WorldPhysics {
    WorldPhysics::with_step_duration(&[], -10., StepDuration::HZ_1000)
}
#[test]
fn described_bodies_collide_and_fixed_surfaces_are_queryable() {
    let mut world = world();
    let floor = model(
        BodyKind::Fixed,
        ShapeDescription::Cuboid {
            half_extents_m: [1., 1., 0.2],
        },
    );
    world
        .install_body(BodyKey(1), &floor, BodyState::at(Pose::at([0., 0., 0.2])))
        .unwrap();
    assert!(world.is_idle());
    let floor_scene = world.static_geometry_snapshot(true);
    let floor_distance = floor_scene
        .cast_ray([0., 0., 1.], [0., 0., -1.], 2.)
        .unwrap()
        .unwrap()
        .distance_m;
    assert!((floor_distance - 0.6).abs() < 1e-12);
    let ball = model(dynamic(), ShapeDescription::Ball { radius_m: 0.1 });
    let binding = world
        .install_body(BodyKey(2), &ball, BodyState::at(Pose::at([0., 0., 1.])))
        .unwrap();
    assert!(!world.is_idle());
    let frames = TargetFrames::new(vec![]);
    for tick in 0..1000 {
        world.step(tick * 1_000_000, &frames).unwrap();
    }
    let state = world.body_state(binding).unwrap();
    assert!(
        (state.pose.translation_m[2] - 0.5).abs() < 0.01,
        "{state:?}"
    );
    assert!(state.velocity_m_s[2].abs() < 0.02);
}
#[test]
fn bindings_reject_other_worlds_and_reused_logical_keys_without_mutation() {
    let mut a = world();
    let mut b = world();
    let shape = model(dynamic(), ShapeDescription::Ball { radius_m: 0.1 });
    let binding = a
        .install_body(BodyKey(1), &shape, BodyState::at(Pose::at([0.; 3])))
        .unwrap();
    let other = b
        .install_body(BodyKey(1), &shape, BodyState::at(Pose::at([0.; 3])))
        .unwrap();
    assert_ne!(binding, other);
    let before = b.body_state(other).unwrap();
    assert!(b.set_body_velocity(binding, [1.; 3], [0.; 3]).is_err());
    assert!(b.set_body_wrench(binding, [1.; 3], [0.; 3]).is_err());
    assert!(b.remove_bound_body(binding).is_err());
    assert_eq!(b.body_state(other).unwrap(), before);
    assert!(a.install_body(BodyKey(1), &shape, before).is_err());
    assert_eq!(a.body_state(binding).unwrap(), before);
    a.remove_bound_body(binding).unwrap();
    let replacement = a.install_body(BodyKey(1), &shape, before).unwrap();
    assert_ne!(replacement, binding);
    assert!(a.body_state(binding).is_err());
    assert!(a.remove_bound_body(binding).is_err());
    assert_eq!(a.body_binding(BodyKey(1)), Some(replacement));
    assert_eq!(a.body_state(replacement).unwrap(), before);
}
#[test]
fn held_world_wrenches_and_explicit_velocity_have_defined_units_and_lifetime() {
    let mut world = world();
    let shape = model(dynamic(), ShapeDescription::Ball { radius_m: 0.1 });
    let binding = world
        .install_body(BodyKey(1), &shape, BodyState::at(Pose::at([0.; 3])))
        .unwrap();
    world
        .set_body_wrench(binding, [4., 0., 2. * 9.81], [0., 0., 4.])
        .unwrap();
    let frames = TargetFrames::new(vec![]);
    for tick in 0..10 {
        world.step(tick * 1_000_000, &frames).unwrap();
    }
    let state = world.body_state(binding).unwrap();
    assert!((state.velocity_m_s[0] - 0.02).abs() < 1e-8, "{state:?}");
    assert!(state.velocity_m_s[2].abs() < 1e-8);
    assert!((state.angular_velocity_rad_s[2] - 0.01).abs() < 1e-8);
    assert!(
        world
            .set_body_velocity(binding, [f64::NAN, 0., 0.], [0.; 3])
            .is_err()
    );
    assert!(
        world
            .set_body_wrench(binding, [f64::NAN, 0., 0.], [0.; 3])
            .is_err()
    );
    assert_eq!(world.body_state(binding).unwrap(), state);
    world.set_body_wrench(binding, [0.; 3], [0.; 3]).unwrap();
    world.set_body_velocity(binding, [0.; 3], [0.; 3]).unwrap();
    world.step(10_000_000, &frames).unwrap();
    let state = world.body_state(binding).unwrap();
    assert!(state.velocity_m_s[0].abs() < 1e-10);
    assert!(state.velocity_m_s[2] < 0.);
}
#[test]
fn kinematic_endpoints_span_one_tick_and_hold_without_restarting() {
    let mut world = world();
    let shape = model(
        BodyKind::Kinematic,
        ShapeDescription::Cuboid {
            half_extents_m: [0.1; 3],
        },
    );
    let binding = world
        .install_body(BodyKey(1), &shape, BodyState::at(Pose::at([0.; 3])))
        .unwrap();
    let frames = TargetFrames::new(vec![]);
    world
        .set_body_endpoint(binding, Pose::at([1., 0., 0.]))
        .unwrap();
    assert_eq!(
        world.body_state(binding).unwrap().pose.translation_m,
        [0.; 3]
    );
    world.step(0, &frames).unwrap();
    assert_eq!(
        world.body_state(binding).unwrap().pose.translation_m,
        [1., 0., 0.]
    );
    world.step(1_000_000, &frames).unwrap();
    let state = world.body_state(binding).unwrap();
    assert_eq!(state.pose.translation_m, [1., 0., 0.]);
    assert_eq!(state.velocity_m_s, [0.; 3]);
    assert!(world.set_body_velocity(binding, [1.; 3], [0.; 3]).is_err());
    assert!(world.set_body_wrench(binding, [1.; 3], [0.; 3]).is_err());
}
#[test]
fn recipe_reset_rebinds_keys_and_rejects_old_tokens() {
    let shape = model(dynamic(), ShapeDescription::Ball { radius_m: 0.1 });
    let recipe = RobotScenario::new(vec![], -10., StepDuration::HZ_1000, move |world| {
        world.install_body(BodyKey(42), &shape, BodyState::at(Pose::at([0., 0., 1.])))?;
        Ok(())
    })
    .unwrap();
    let mut session = RobotSession::new(recipe).unwrap();
    let old = session.ground_truth().body_binding(BodyKey(42)).unwrap();
    session.step(10).unwrap();
    session.reset().unwrap();
    let rebound = session.ground_truth().body_binding(BodyKey(42)).unwrap();
    assert_ne!(old, rebound);
    assert!(session.ground_truth().body_state(old).is_err());
    assert_eq!(
        session
            .ground_truth()
            .body_state(rebound)
            .unwrap()
            .pose
            .translation_m,
        [0., 0., 1.]
    );
}

#[test]
fn invalid_admission_leaves_keys_and_geometry_available_for_retry() {
    let mut world = world();
    let shape = model(BodyKind::Fixed, ShapeDescription::Ball { radius_m: 0.1 });
    let mut bad = BodyState::at(Pose::at([0.; 3]));
    bad.velocity_m_s = [1., 0., 0.];
    assert!(world.install_body(BodyKey(1), &shape, bad).is_err());
    bad.velocity_m_s = [0.; 3];
    bad.pose.translation_m[0] = f64::NAN;
    assert!(world.install_body(BodyKey(1), &shape, bad).is_err());
    assert!(world.body_binding(BodyKey(1)).is_none());
    assert!(
        world
            .static_geometry_snapshot(true)
            .cast_ray([-1., 0., 0.], [1., 0., 0.], 2.)
            .unwrap()
            .is_none()
    );
    let binding = world
        .install_body(BodyKey(1), &shape, BodyState::at(Pose::at([0.; 3])))
        .unwrap();
    let frozen = world.static_geometry_snapshot(true);
    assert!(
        frozen
            .cast_ray([-1., 0., 0.], [1., 0., 0.], 2.)
            .unwrap()
            .is_some()
    );
    world.remove_bound_body(binding).unwrap();
    assert!(
        frozen
            .cast_ray([-1., 0., 0.], [1., 0., 0.], 2.)
            .unwrap()
            .is_some()
    );
    assert!(
        world
            .static_geometry_snapshot(true)
            .cast_ray([-1., 0., 0.], [1., 0., 0.], 2.)
            .unwrap()
            .is_none()
    );
}
