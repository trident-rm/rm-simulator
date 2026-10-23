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
    runtime::BodyKey,
    session::{RobotScenario, RobotSession},
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
    let recipe = RobotScenario::new(vec![], 0., StepDuration::HZ_1000, move |world| {
        world.install_body(BodyKey(7), &cooked, BodyState::at(Pose::at([0., 0., 1.])))?;
        Ok(())
    })?;
    let mut session = RobotSession::new(recipe)?;
    let binding = session
        .ground_truth()
        .body_binding(BodyKey(7))
        .ok_or("missing example body")?;
    let mut contacts = Vec::new();
    session.step_into(1000, &mut contacts)?;
    let state = session.ground_truth().body_state(binding)?;
    println!("after one second: {:?}", state.pose.translation_m);
    assert!((state.pose.translation_m[2] - 0.1).abs() < 0.01);
    session.reset()?;
    assert!(session.ground_truth().body_state(binding).is_err());
    let rebound = session
        .ground_truth()
        .body_binding(BodyKey(7))
        .ok_or("missing reset body")?;
    assert_ne!(binding, rebound);
    println!(
        "reset: {:?}",
        session
            .ground_truth()
            .body_state(rebound)?
            .pose
            .translation_m
    );
    Ok(())
}
