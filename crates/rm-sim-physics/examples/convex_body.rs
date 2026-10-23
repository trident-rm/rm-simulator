// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Explicit convex fixture, cooked once and installed by a repeatable recipe.
use rm_sim_engine::{
    Pose,
    model::{
        AssemblyDescription, AssemblyLink, AssemblyLinkId, BodyKind, BodyState,
        ColliderDescription, ContactMaterial, ConvexHull, Inertial, RigidBodyDescription,
        ShapeDescription,
    },
};
use rm_sim_physics::{
    StepDuration,
    cooking::CookedAssembly,
    runtime::{AssemblyKeys, BodyKey},
    session::{RobotScenario, RobotSession},
};

/// Runtime key for the fixture; reacquire its binding after reset.
pub const BODY: BodyKey = BodyKey(7);
/// Source link identity, independent of runtime placement and keys.
pub const LINK: AssemblyLinkId = AssemblyLinkId(1);

/// One-link convex cube assembly with explicit uniform-solid mass properties.
/// The fixture has 0.2 m sides, mass 2 kg and central inertia m * side² / 6.
/// These are chosen fixture properties, not density inferred from CAD or cooking.
pub fn prepared() -> Result<CookedAssembly, &'static str> {
    let points = [-0.1, 0.1]
        .into_iter()
        .flat_map(|x| {
            [-0.1, 0.1]
                .into_iter()
                .flat_map(move |y| [-0.1, 0.1].into_iter().map(move |z| [x, y, z]))
        })
        .collect();
    let source = RigidBodyDescription::new(
        BodyKind::Dynamic(Inertial::new(
            2.,
            [2. * 0.2 * 0.2 / 6.; 3],
            Pose::default(),
        )?),
        vec![ColliderDescription::new(
            ShapeDescription::ConvexHull(ConvexHull::new(points)?),
            Pose::default(),
            ContactMaterial {
                friction: 0.4,
                restitution: 0.,
                memberships: u32::MAX,
                filter: u32::MAX,
            },
        )?],
    )?;
    CookedAssembly::new(AssemblyDescription::new(
        LINK,
        vec![AssemblyLink::new(
            LINK,
            source,
            BodyState::at(Pose::at([0., 0., 1.])),
        )?],
        vec![],
    )?)
}

/// A fresh 1000 Hz world reuses the same immutable prepared assembly on reset.
pub fn scenario(prepared: CookedAssembly) -> Result<RobotScenario, &'static str> {
    RobotScenario::new(vec![], 0., StepDuration::HZ_1000, move |world| {
        world.install_assembly(
            &prepared,
            Pose::default(),
            &AssemblyKeys {
                links: vec![(LINK, BODY)],
                joints: vec![],
            },
        )?;
        Ok(())
    })
}

fn main() -> Result<(), &'static str> {
    let mut session = RobotSession::new(scenario(prepared()?)?)?;
    let old = session
        .ground_truth()
        .body_binding(BODY)
        .ok_or("missing convex fixture")?;
    session.step(1000)?;
    let state = session.ground_truth().body_state(old)?;
    println!("settled convex cube: {:?}", state.pose.translation_m);
    assert!((state.pose.translation_m[2] - 0.1).abs() < 0.01);
    session.reset()?;
    let rebound = session
        .ground_truth()
        .body_binding(BODY)
        .ok_or("missing reset fixture")?;
    assert_ne!(old, rebound);
    assert!(session.ground_truth().body_state(old).is_err());
    assert_eq!(
        session
            .ground_truth()
            .body_state(rebound)?
            .pose
            .translation_m,
        [0., 0., 1.]
    );
    println!("reset reinstalled the prepared convex assembly");
    Ok(())
}
