// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Stable serialized records retained during the solver-free core extraction.
use rm_sim_engine::{
    Pose, StepDuration,
    control::{DriveControl, WheelTorques},
};

#[test]
fn core_extraction_preserves_existing_pose_time_and_control_shapes() {
    let pose = Pose::at([1., 2., 3.]);
    assert_eq!(
        serde_json::to_string(&pose).unwrap(),
        r#"{"translation_m":[1.0,2.0,3.0],"rotation_wxyz":[1.0,0.0,0.0,0.0]}"#
    );
    assert_eq!(
        serde_json::from_str::<Pose>(&serde_json::to_string(&pose).unwrap()).unwrap(),
        pose
    );
    assert_eq!(
        serde_json::to_string(&StepDuration::HZ_128).unwrap(),
        "7812500"
    );
    assert!(serde_json::from_str::<StepDuration>("0").is_err());
    assert!(serde_json::from_str::<StepDuration>("7812501").is_err());
    let control = DriveControl::WheelTorques(WheelTorques::new(vec![1., -1.]).unwrap());
    assert_eq!(
        serde_json::to_string(&control).unwrap(),
        r#"{"WheelTorques":[1.0,-1.0]}"#
    );
    assert_eq!(
        serde_json::from_str::<DriveControl>(&serde_json::to_string(&control).unwrap()).unwrap(),
        control
    );
    assert_eq!(
        serde_json::to_string(&DriveControl::BodyVelocity).unwrap(),
        r#""BodyVelocity""#
    );
}
