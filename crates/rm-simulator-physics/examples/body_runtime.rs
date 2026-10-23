// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Standalone source-described rigid body, cooked resources and reset-safe binding.
use rm_simulator_engine::{
    Pose,
    model::{
        BodyKind, BodyState, ColliderDescription, ContactMaterial, Inertial, RigidBodyDescription,
        ShapeDescription,
    },
};
use rm_simulator_physics::{
    StepDuration,
    cooking::CookedRigidBody,
    runtime::{BodyKey, PhysicalRuntime},
};
fn main() -> Result<(), &'static str> {
    // Explicit uniform 2 kg cube inertia about its centre (0.2 m sides), kg m².
    let inertia = Inertial::new(2., [0.013333333333333334; 3], Pose::at([0.; 3]))?;
    let source = RigidBodyDescription::new(
        BodyKind::Dynamic(inertia),
        vec![ColliderDescription::new(
            ShapeDescription::Cuboid {
                half_extents_m: [0.1; 3],
            },
            Pose::at([0.; 3]),
            ContactMaterial {
                friction: 0.4,
                restitution: 0.,
                memberships: u32::MAX,
                filter: u32::MAX,
            },
        )?],
    )?;
    let cooked = CookedRigidBody::new(source)?;
    let duration = StepDuration::HZ_1000;
    let recipe = |world: &mut PhysicalRuntime| {
        world.install_catch_floor(
            0.,
            ContactMaterial {
                friction: 0.4,
                restitution: 0.,
                memberships: u32::MAX,
                filter: u32::MAX,
            },
        )?;
        world.install_body(BodyKey(7), &cooked, BodyState::at(Pose::at([0., 0., 1.])))?;
        world.enable_contact_observations(64)?;
        Ok(())
    };
    let mut runtime = PhysicalRuntime::from_recipe(duration, recipe)?;
    let binding = runtime
        .body_binding(BodyKey(7))
        .ok_or("missing example body")?;
    let mut observed = 0;
    for tick in 0..1000 {
        runtime.step(tick * duration.as_ns())?;
        observed += runtime.physical_contacts().len();
        assert_eq!(runtime.physical_contacts_dropped(), 0);
    }
    let state = runtime.body_state(binding)?;
    println!(
        "after one second: {:?}; {observed} contact samples",
        state.pose.translation_m
    );
    assert!((state.pose.translation_m[2] - 0.1).abs() < 0.01);
    assert!(observed > 0);
    runtime = PhysicalRuntime::from_recipe(duration, recipe)?;
    assert!(runtime.body_state(binding).is_err());
    let rebound = runtime
        .body_binding(BodyKey(7))
        .ok_or("missing reset body")?;
    assert_ne!(binding, rebound);
    println!(
        "reset: {:?}",
        runtime.body_state(rebound)?.pose.translation_m
    );
    Ok(())
}
