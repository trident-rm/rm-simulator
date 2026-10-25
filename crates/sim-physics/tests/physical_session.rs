// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Generic session timing, runtime parity and transactional reconstruction.
use sim_engine::{
    Pose,
    model::{BodyKind, BodyState, Inertial, RigidBodyDescription},
    time::Stamp,
};
use sim_physics::{
    PhysicalRuntime, StepDuration,
    cooking::CookedRigidBody,
    runtime::{BodyKey, session::PhysicalSession},
};
use std::cell::Cell;

#[test]
fn session_matches_direct_runtime_and_partitioned_stepping_at_both_rates() {
    let cooked = CookedRigidBody::new(
        RigidBodyDescription::new(
            BodyKind::Dynamic(Inertial::new(2., [0.1; 3], Pose::default()).unwrap()),
            vec![],
        )
        .unwrap(),
    )
    .unwrap();
    for duration in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let recipe = || {
            PhysicalRuntime::from_recipe(duration, |runtime| {
                let body = runtime.install_body(
                    BodyKey(1),
                    &cooked,
                    BodyState::at(Pose::at([0., 0., 1.])),
                )?;
                runtime.set_body_wrench(body, [1., 0., 0.], [0.; 3])?;
                Ok(())
            })
        };
        let mut session = PhysicalSession::new(recipe).unwrap();
        let mut direct = recipe().unwrap();
        let binding = session.runtime().body_binding(BodyKey(1)).unwrap();
        let direct_binding = direct.body_binding(BodyKey(1)).unwrap();
        let initial = session.runtime().body_state(binding).unwrap();
        session.step(0).unwrap();
        session.step(3).unwrap();
        session.step(61).unwrap();
        for tick in 0..64 {
            direct.step(tick * duration.as_ns()).unwrap();
        }
        let expected = direct.body_state(direct_binding).unwrap();
        assert_eq!(session.runtime().body_state(binding).unwrap(), expected);
        assert_eq!(
            session.stamp(),
            Stamp {
                tick: 64,
                time_ns: 64 * duration.as_ns()
            }
        );
        let before = session.stamp();
        assert!(session.step(u64::MAX).is_err());
        assert_eq!(session.stamp(), before);
        assert_eq!(session.runtime().body_state(binding).unwrap(), expected);
        session.reset().unwrap();
        assert_eq!(
            session.stamp(),
            Stamp {
                tick: 0,
                time_ns: 0
            }
        );
        assert!(session.runtime().body_state(binding).is_err());
        let rebound = session.runtime().body_binding(BodyKey(1)).unwrap();
        assert_eq!(session.runtime().body_state(rebound).unwrap(), initial);
        session.step(64).unwrap();
        assert_eq!(session.runtime().body_state(rebound).unwrap(), expected);
    }
}

#[test]
fn failed_or_changed_rate_recipe_preserves_live_state_and_commands() {
    let fail = Cell::new(false);
    let duration = Cell::new(StepDuration::HZ_1000);
    let cooked = CookedRigidBody::new(
        RigidBodyDescription::new(
            BodyKind::Dynamic(Inertial::new(1., [0.1; 3], Pose::default()).unwrap()),
            vec![],
        )
        .unwrap(),
    )
    .unwrap();
    let recipe = || {
        PhysicalRuntime::from_recipe(duration.get(), |runtime| {
            runtime.install_body(BodyKey(1), &cooked, BodyState::at(Pose::default()))?;
            if fail.get() {
                return Err("test setup failed after installing a body");
            }
            Ok(())
        })
    };
    let mut session = PhysicalSession::new(recipe).unwrap();
    let body = session.runtime().body_binding(BodyKey(1)).unwrap();
    session
        .runtime_mut()
        .set_body_wrench(body, [2., 0., 0.], [0.; 3])
        .unwrap();
    session.step(10).unwrap();
    let before = session.runtime().body_state(body).unwrap();
    let stamp = session.stamp();
    fail.set(true);
    assert!(session.reset().is_err());
    fail.set(false);
    duration.set(StepDuration::HZ_128);
    assert!(session.reset().is_err());
    assert_eq!(session.stamp(), stamp);
    assert_eq!(session.step_duration(), StepDuration::HZ_1000);
    assert_eq!(session.runtime().body_state(body).unwrap(), before);
    session.step(10).unwrap();
    let after = session.runtime().body_state(body).unwrap();
    assert!((after.velocity_m_s[0] - before.velocity_m_s[0] - 0.02).abs() < 1e-12);
}
