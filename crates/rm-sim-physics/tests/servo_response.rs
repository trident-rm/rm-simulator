// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
#[allow(dead_code)]
#[path = "../examples/servo_response.rs"]
mod servo;
use rm_sim_engine::model::JointMotor;
use rm_sim_physics::StepDuration;
use servo::{Fixture, INERTIA_KG_M2};

#[test]
fn direct_torque_matches_inertia_and_opposing_load_at_both_rates() {
    for duration in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        for load in [0., -0.1, -0.2] {
            let mut f = Fixture::new(duration, false).unwrap();
            f.runtime.set_joint_effort(f.joint, 0.2).unwrap();
            assert_eq!(f.runtime.joint_applied_effort(f.joint).unwrap(), 0.);
            f.runtime
                .set_body_wrench(f.rotor, [0.; 3], [load, 0., 0.])
                .unwrap();
            for _ in 0..64 {
                f.step().unwrap();
            }
            let expected = (0.2 + load) / INERTIA_KG_M2 * f.time_s();
            let state = f.runtime.joint_state(f.joint).unwrap();
            assert!(
                (state.velocity - expected).abs() < 1e-8,
                "{state:?} vs {expected}"
            );
            assert_eq!(f.runtime.joint_applied_effort(f.joint).unwrap(), 0.2);
            f.runtime.set_joint_effort(f.joint, 0.).unwrap();
            assert_eq!(f.runtime.joint_applied_effort(f.joint).unwrap(), 0.2);
            f.step().unwrap();
            assert_eq!(f.runtime.joint_applied_effort(f.joint).unwrap(), 0.);
            f.runtime.remove_bound_joint(f.joint).unwrap();
            assert!(f.runtime.joint_applied_effort(f.joint).is_err());
        }
    }
}

#[test]
fn servo_saturates_and_holds_load_with_expected_pd_error() {
    for duration in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let mut f = Fixture::new(duration, true).unwrap();
        f.runtime
            .set_joint_motor(
                f.joint,
                Some(JointMotor::new(0.3, 0., 4., 0.4, 0.5).unwrap()),
            )
            .unwrap();
        f.step().unwrap();
        // At 1000 Hz this is the first-slice output; 128 Hz includes feedback substeps.
        let applied = f.runtime.joint_applied_effort(f.joint).unwrap();
        assert!(applied > 0.45 && applied <= 0.5);
        f.runtime
            .set_body_wrench(f.rotor, [0.; 3], [-0.2, 0., 0.])
            .unwrap();
        let ticks = (2_000_000_000 / duration.as_ns()) as usize;
        for _ in 0..ticks {
            f.step().unwrap();
            assert!(f.runtime.joint_applied_effort(f.joint).unwrap().abs() <= 0.5);
        }
        let state = f.runtime.joint_state(f.joint).unwrap();
        assert!((state.position - 0.25).abs() < 1e-5, "{state:?}");
        assert!((f.runtime.joint_applied_effort(f.joint).unwrap() - 0.2).abs() < 1e-5);
        // Clear motor using direct torque, then drive into the physical hard stop.
        f.runtime
            .set_body_wrench(f.rotor, [0.; 3], [0.; 3])
            .unwrap();
        f.runtime.set_joint_effort(f.joint, 0.4).unwrap();
        for _ in 0..ticks {
            f.step().unwrap();
        }
        let state = f.runtime.joint_state(f.joint).unwrap();
        assert!((state.position - 0.5).abs() < 0.002, "{state:?}");
        assert!(state.velocity.abs() < 0.01);
        // Applied actuator torque remains nonzero even when net acceleration is zero.
        assert_eq!(f.runtime.joint_applied_effort(f.joint).unwrap(), 0.4);
    }
}
