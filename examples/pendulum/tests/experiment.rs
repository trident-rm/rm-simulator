// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Closed-loop behavior, separate sampling clocks and complete extension reset.
use sim_pendulum_example::{
    CURRENT_LIMIT_A, ENCODER_RAD_PER_COUNT, Experiment, HINGE, Motor, PARTS, SUPPLY_V, Telemetry,
    reference_rad,
};

#[test]
fn motor_bounds_current_and_passively_brakes_at_zero_voltage() {
    let mut motor = Motor::default();
    for voltage in [-100., -12., 0., 12., 100.] {
        for speed in [-50., -2., 0., 2., 50.] {
            let output = motor.apply(voltage, speed).unwrap();
            assert!(output.voltage_v.abs() <= SUPPLY_V);
            assert!(output.current_a.abs() <= CURRENT_LIMIT_A);
            if voltage == 0. {
                assert!(output.torque_nm * speed <= 0.);
            }
        }
    }
    let saturated = motor.apply(100., 0.).unwrap();
    assert_eq!(saturated.current_a, CURRENT_LIMIT_A);
    assert_eq!(saturated.voltage_v, SUPPLY_V);
    assert!(motor.apply(f64::NAN, 0.).is_err());
}

#[test]
fn encoder_holds_timestamped_quantized_values_and_clocks_run_independently() {
    let mut experiment = Experiment::new().unwrap();
    assert_eq!(experiment.telemetry(), Telemetry::default());
    let initial = experiment.truth().unwrap().position;
    experiment.advance(1).unwrap();
    let first = experiment.telemetry().encoder.unwrap();
    assert_eq!(first.time_ns, 0);
    assert!((first.angle_rad() - initial).abs() <= ENCODER_RAD_PER_COUNT / 2.);
    experiment.advance(4).unwrap();
    assert_eq!(experiment.telemetry().encoder, Some(first));
    assert_ne!(experiment.truth().unwrap().position, initial);
    experiment.advance(1).unwrap();
    assert_eq!(experiment.telemetry().encoder.unwrap().time_ns, 5_000_000);
    assert_eq!(experiment.telemetry().controller_updates, 1);
    experiment.advance(994).unwrap();
    assert_eq!(experiment.telemetry().encoder_samples, 200);
    assert_eq!(experiment.telemetry().controller_updates, 100);
    assert_eq!(experiment.telemetry().encoder.unwrap().time_ns, 995_000_000);
}

#[test]
fn measured_feedback_tracks_both_gravity_loaded_targets_with_bounded_actuation() {
    let mut experiment = Experiment::new().unwrap();
    for _ in 0..2 {
        let target = reference_rad(experiment.stamp().time_ns);
        for _ in 0..4000 {
            experiment.advance(1).unwrap();
            let output = experiment.telemetry().motor;
            assert!(output.voltage_v.abs() <= SUPPLY_V);
            assert!(output.current_a.abs() <= CURRENT_LIMIT_A);
        }
        let truth = experiment.truth().unwrap();
        assert!(
            (truth.position - target).abs() < 0.005,
            "{truth:?} vs {target}"
        );
        assert!(truth.velocity.abs() < 0.05, "{truth:?}");
    }
    assert_eq!(experiment.telemetry().encoder_samples, 1600);
    assert_eq!(experiment.telemetry().controller_updates, 800);
}

#[test]
fn reset_between_sample_boundaries_clears_all_history_and_replays_the_episode() {
    let mut experiment = Experiment::new().unwrap();
    let old_joint = experiment.runtime().joint_binding(HINGE).unwrap();
    let old_body = experiment.runtime().body_binding(PARTS[1].body).unwrap();
    let mut reference = Vec::new();
    for _ in 0..1379 {
        experiment.advance(1).unwrap();
        reference.push((
            experiment.stamp(),
            experiment.telemetry(),
            experiment.truth().unwrap(),
        ));
    }
    assert_ne!(experiment.telemetry().motor.voltage_v, 0.);
    experiment.reset().unwrap();
    assert_eq!(experiment.stamp().time_ns, 0);
    assert_eq!(experiment.telemetry(), Telemetry::default());
    assert!(experiment.runtime().joint_state(old_joint).is_err());
    assert!(experiment.runtime().body_state(old_body).is_err());
    for expected in reference {
        experiment.advance(1).unwrap();
        assert_eq!(
            (
                experiment.stamp(),
                experiment.telemetry(),
                experiment.truth().unwrap()
            ),
            expected
        );
    }
}

#[test]
fn caller_partitioning_and_rejected_overflow_preserve_the_experiment() {
    let mut whole = Experiment::new().unwrap();
    let mut partitioned = Experiment::new().unwrap();
    whole.advance(2000).unwrap();
    for steps in [13, 487, 1, 1499] {
        partitioned.advance(steps).unwrap();
    }
    assert_eq!(whole.truth().unwrap(), partitioned.truth().unwrap());
    assert_eq!(whole.telemetry(), partitioned.telemetry());
    let before = (whole.stamp(), whole.telemetry(), whole.truth().unwrap());
    assert!(whole.advance(u64::MAX).is_err());
    assert_eq!(
        (whole.stamp(), whole.telemetry(), whole.truth().unwrap()),
        before
    );
}
