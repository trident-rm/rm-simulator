// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Verify recipe and installed collider sharing without sharing live robot state.
use super::*;
use sim_physics::backend::CookedBodyExt;
use std::sync::Arc;

#[test]
fn prepared_clones_and_installations_share_shapes_and_keep_robot_state_separate() {
    let prepared = PreparedChassis::new(ChassisConfig::folding_hero()).unwrap();
    let clone = prepared.clone();
    assert!(std::ptr::eq(prepared.config(), clone.config()));
    assert!(std::ptr::eq(prepared.source(), clone.source()));
    assert!(std::ptr::eq(prepared.cooked(), clone.cooked()));
    for (a, b) in prepared
        .cooked()
        .shapes()
        .iter()
        .zip(clone.cooked().shapes())
    {
        assert!(Arc::ptr_eq(&a.0, &b.0));
    }
    let mut world = PhysicsWorld::default();
    let mut a = Chassis::new_prepared(&mut world, 0, Team::Red, &prepared, Pose::at([0., 0., 0.5]))
        .unwrap();
    let b =
        Chassis::new_prepared(&mut world, 1, Team::Blue, &clone, Pose::at([4., 0., 0.5])).unwrap();
    assert_ne!(a.body, b.body);
    let a_colliders = world.bodies[a.body].colliders();
    let b_colliders = world.bodies[b.body].colliders();
    assert_eq!(a_colliders.len(), prepared.cooked().shapes().len());
    for ((a_handle, b_handle), shape) in a_colliders
        .iter()
        .zip(b_colliders)
        .zip(prepared.cooked().shapes())
    {
        assert_ne!(a_handle, b_handle);
        assert!(Arc::ptr_eq(
            &world.colliders[*a_handle].shared_shape().0,
            &shape.0
        ));
        assert!(Arc::ptr_eq(
            &world.colliders[*b_handle].shared_shape().0,
            &shape.0
        ));
    }
    let b_before = b.snapshot(&world);
    a.wheels[0].spin_rad = 1.;
    a.config.mass_kg += 1.;
    a.place(&mut world, Pose::at([1., 0., 0.5])).unwrap();
    assert_eq!(b.snapshot(&world), b_before);
    assert_eq!(prepared.config().mass_kg, b.config.mass_kg);
    a.remove(&mut world);
    assert_eq!(b.snapshot(&world), b_before);
}
