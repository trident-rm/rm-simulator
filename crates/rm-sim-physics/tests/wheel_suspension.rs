// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Closed-loop bench checks: numerical diagnostics, not hardware calibration.
#[allow(dead_code)]
#[path = "../examples/wheel_suspension.rs"]
mod fixture;
use fixture::{Fixture, Parameters};
use rm_sim_physics::StepDuration;

#[test]
fn loaded_loop_settles_and_rebounds_at_both_rates() {
    for duration in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let mut f = Fixture::new(Parameters::default(), duration).unwrap();
        let n = (2_000_000_000 / duration.as_ns()) as usize;
        let mut maximum_gap: f64 = 0.;
        for _ in 0..n {
            let s = f.step().unwrap();
            maximum_gap = maximum_gap.max(s.closure_error_m);
        }
        let before = f.sample().unwrap();
        assert!(before.speed_m_s.abs() < 0.001, "{duration:?} {before:?}");
        f.load(-20.).unwrap();
        for _ in 0..n / 8 {
            f.step().unwrap();
        }
        let pressed = f.sample().unwrap();
        assert!(pressed.length_m < before.length_m - 0.0005, "{pressed:?}");
        f.load(0.).unwrap();
        for _ in 0..n * 2 {
            let s = f.step().unwrap();
            maximum_gap = maximum_gap.max(s.closure_error_m);
            assert!(s.length_m.is_finite() && s.speed_m_s.is_finite());
            assert!(s.length_m >= f.parameters.free_length_m - f.parameters.stroke_m - 0.001);
            assert!(s.length_m <= f.parameters.free_length_m + 0.001);
            assert!(s.slider_error_m < 0.0001);
        }
        let end = f.sample().unwrap();
        assert!(
            (end.length_m - before.length_m).abs() < 0.0001,
            "{duration:?} {before:?} {end:?}"
        );
        assert!(end.speed_m_s.abs() < 0.0001);
        assert!(maximum_gap < 0.00002, "{duration:?} gap={maximum_gap}");
        // This explicit finite-iteration solver tolerance is recorded in the report.
        assert!(end.carrier_tilt_rad < 0.0005);
    }
}

#[test]
fn damping_reduces_residual_motion_without_prescribing_poses() {
    let mut damped = Fixture::new(Parameters::default(), StepDuration::HZ_1000).unwrap();
    let mut undamped = Fixture::new(
        Parameters {
            damping_n_s_m: 0.,
            ..Parameters::default()
        },
        StepDuration::HZ_1000,
    )
    .unwrap();
    let mut energy = [0.; 2];
    for tick in 0..6000 {
        for (i, f) in [&mut damped, &mut undamped].into_iter().enumerate() {
            if tick == 2000 {
                f.load(-20.).unwrap();
            } else if tick == 2250 {
                f.load(0.).unwrap();
            }
            let s = f.step().unwrap();
            if tick >= 4000 {
                energy[i] += s.speed_m_s.powi(2);
            }
        }
    }
    assert!(
        energy[1] > 1e-6,
        "undamped control must remain excited: {energy:?}"
    );
    assert!(energy[0] < energy[1] * 0.1, "{energy:?}");
}

#[test]
fn reset_reconstructs_the_closed_loop_and_invalidates_tokens() {
    let mut f = Fixture::new(Parameters::default(), StepDuration::HZ_1000).unwrap();
    let old = f.slider();
    let mut expected = Vec::new();
    for tick in 0..512 {
        if tick == 128 {
            f.load(-20.).unwrap();
        }
        if tick == 256 {
            f.load(0.).unwrap();
        }
        expected.push(f.step().unwrap());
    }
    f.reset().unwrap();
    assert!(f.runtime.joint_state(old).is_err());
    assert_ne!(old, f.slider());
    for (tick, want) in expected.into_iter().enumerate() {
        if tick == 128 {
            f.load(-20.).unwrap();
        }
        if tick == 256 {
            f.load(0.).unwrap();
        }
        assert_eq!(f.step().unwrap(), want);
    }
}

#[test]
fn invalid_choices_and_damping_do_not_silently_advance() {
    for parameters in [
        Parameters {
            stroke_m: -1.,
            ..Parameters::default()
        },
        Parameters {
            load_kg: f64::NAN,
            ..Parameters::default()
        },
        Parameters {
            free_length_m: 0.08,
            ..Parameters::default()
        },
    ] {
        assert!(Fixture::new(parameters, StepDuration::HZ_1000).is_err());
    }
    let mut f = Fixture::new(Parameters::default(), StepDuration::HZ_1000).unwrap();
    let before = f.sample().unwrap();
    assert!(f.damping(-1.).is_err());
    assert!(f.damping(f64::NAN).is_err());
    assert_eq!(f.sample().unwrap(), before);
    f.damping(150.).unwrap();
    assert_eq!(f.sample().unwrap(), before);
    let first = f.step().unwrap();
    f.reset().unwrap();
    assert_eq!(f.parameters.damping_n_s_m, 150.);
    assert_eq!(f.step().unwrap(), first);
}

#[test]
fn extension_and_compression_stops_bound_an_axially_forced_bench() {
    // Isolate stop behavior: replace the passive motor by a held test force.
    // This deliberate bench actuation is not a recovered robot controller.
    for (force, stop) in [(100., 0.096), (-100., 0.066)] {
        let mut f = Fixture::new(Parameters::default(), StepDuration::HZ_1000).unwrap();
        let slider = f.slider();
        f.runtime.set_joint_effort(slider, force).unwrap();
        for _ in 0..4000 {
            f.step().unwrap();
        }
        let s = f.sample().unwrap();
        assert!((s.length_m - stop).abs() < 0.00001, "force={force}: {s:?}");
        assert!(s.closure_error_m < 0.00005, "{s:?}");
    }
}

#[test]
fn omitted_loop_closure_is_detected_by_pose_residuals() {
    let mut intact = Fixture::new(Parameters::default(), StepDuration::HZ_1000).unwrap();
    let mut broken = Fixture::new(Parameters::default(), StepDuration::HZ_1000).unwrap();
    let closure = broken
        .runtime
        .joint_binding(rm_sim_physics::runtime::JointKey(4))
        .unwrap();
    broken.runtime.remove_bound_joint(closure).unwrap();
    for _ in 0..1000 {
        intact.step().unwrap();
        broken.step().unwrap();
    }
    let a = intact.sample().unwrap();
    let b = broken.sample().unwrap();
    assert!(b.closure_error_m > a.closure_error_m * 5., "{a:?} vs {b:?}");
}

#[test]
fn more_solver_work_reduces_loop_error_and_survives_reset() {
    let mut coarse =
        Fixture::with_solver_iterations(Parameters::default(), StepDuration::HZ_1000, 4).unwrap();
    let mut fine =
        Fixture::with_solver_iterations(Parameters::default(), StepDuration::HZ_1000, 32).unwrap();
    let mut gaps = [0_f64; 2];
    for _ in 0..1000 {
        for (i, f) in [&mut coarse, &mut fine].into_iter().enumerate() {
            gaps[i] = gaps[i].max(f.step().unwrap().closure_error_m);
        }
    }
    assert!(gaps[0] > 0.0001);
    assert!(gaps[1] < gaps[0] * 0.05, "{gaps:?}");
    fine.reset().unwrap();
    assert_eq!(fine.runtime.solver_iterations(), 32);
    assert!(fine.runtime.joint_state(coarse.slider()).is_err());
}
