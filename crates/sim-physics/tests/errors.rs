// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Public errors retain categories, causes and compatibility diagnostics.
use sim_engine::{
    Pose, StepDuration,
    model::{BodyKind, BodyState, RigidBodyDescription},
};
use sim_physics::{
    PhysicalRuntime,
    cooking::{CookError, CookedRigidBody, CookedTriangleMesh},
    runtime::{
        BindingError, BodyKey, RuntimeError,
        session::{PhysicalSession, SessionError},
    },
};
use std::error::Error;

#[test]
fn bindings_distinguish_foreign_stale_and_duplicate_without_losing_messages() {
    let model =
        CookedRigidBody::new(RigidBodyDescription::new(BodyKind::Fixed, vec![]).unwrap()).unwrap();
    let mut first = PhysicalRuntime::new(StepDuration::HZ_1000);
    let second = PhysicalRuntime::new(StepDuration::HZ_1000);
    let body = first
        .install_body(BodyKey(7), &model, BodyState::at(Pose::default()))
        .unwrap();
    let foreign = second.body_state(body).unwrap_err();
    assert!(matches!(
        foreign,
        RuntimeError::Binding(BindingError::Foreign(_))
    ));
    assert_eq!(
        foreign.source().unwrap().to_string(),
        "body binding belongs to another runtime"
    );
    assert!(matches!(
        first.install_body(BodyKey(7), &model, BodyState::at(Pose::default())),
        Err(RuntimeError::Binding(BindingError::Duplicate(_)))
    ));
    first.remove_bound_body(body).unwrap();
    assert!(matches!(
        first.body_state(body),
        Err(RuntimeError::Binding(BindingError::Stale(_)))
    ));
}

#[test]
fn cooking_and_session_errors_compose_with_standard_error_sources() {
    let empty = sim_engine::model::TriangleMesh::new(vec![], vec![]).unwrap();
    assert!(matches!(
        CookedTriangleMesh::new(empty),
        Err(CookError::Backend(_))
    ));
    let failure = RuntimeError::Binding(BindingError::Stale("stale body binding"));
    let result = PhysicalSession::new(|| Err(failure));
    let error = match result {
        Err(error) => error,
        Ok(_) => panic!("failed recipe was accepted"),
    };
    assert_eq!(error, SessionError::Recipe(failure));
    assert_eq!(
        error.source().unwrap().source().unwrap().to_string(),
        "stale body binding"
    );
    fn standard_error<T: Error + Send + Sync + 'static>() {}
    standard_error::<RuntimeError>();
    standard_error::<SessionError>();
    standard_error::<sim_engine::model::ModelError>();
    standard_error::<sim_engine::time::StepDurationError>();
}
