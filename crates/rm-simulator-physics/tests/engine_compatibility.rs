// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Existing physical paths name the same core types, without conversion or copies.
use rm_simulator_physics::{
    Pose, StepDuration,
    control::{DriveControl, WheelTorques},
    queries::RayQuery,
};

#[test]
fn compatibility_exports_have_the_same_type_identity_as_engine_contracts() {
    let pose: Pose = rm_simulator_engine::Pose::at([1., 2., 3.]);
    let duration: StepDuration = rm_simulator_engine::StepDuration::HZ_1000;
    let control: DriveControl = rm_simulator_engine::control::DriveControl::WheelTorques(
        WheelTorques::new(vec![0.; 4]).unwrap(),
    );
    let query: RayQuery =
        rm_simulator_engine::queries::RayQuery::new([0.; 3], [1., 0., 0.], 10.).unwrap();
    assert_eq!(pose.translation_m, [1., 2., 3.]);
    assert_eq!(duration.as_ns(), 1_000_000);
    control.validate(4).unwrap();
    assert_eq!(query.direction(), [1., 0., 0.]);
    let config = rm_simulator_physics::chassis::ChassisConfig::default();
    let model: rm_simulator_engine::wheel_model::WheelModel = (&config).into();
    assert_eq!(model.wheel_radius_m, config.wheel_radius_m);
}
