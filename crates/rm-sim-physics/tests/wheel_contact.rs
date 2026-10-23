// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Contact/traction evidence for the guided test stand, not whole-vehicle validation.
#[allow(dead_code)]
#[path = "../examples/wheel_contact.rs"]
mod fixture;
use fixture::{Fixture, Parameters};
use rm_sim_physics::StepDuration;

#[test]
fn gravity_load_is_supported_through_the_tyre_at_both_rates() {
    for dt in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let mut f = Fixture::new(
            Parameters {
                bump_height_m: 0.,
                ..Default::default()
            },
            dt,
        )
        .unwrap();
        let initial = f.sample().unwrap();
        assert_eq!(initial.chassis_support_n, None);
        let n = (2_000_000_000 / dt.as_ns()) as usize;
        let mut sum = 0.;
        let mut count = 0;
        let mut contacts = 0;
        for tick in 0..n {
            let s = f.step().unwrap();
            assert!(s.suspension.closure_error_m < 0.0001, "{dt:?} {s:?}");
            assert!(s.minimum_separation_m > -0.001, "{dt:?} {s:?}");
            assert_eq!(s.dropped_contacts, 0);
            if tick >= n / 2 {
                sum += s.chassis_support_n.unwrap();
                count += 1;
                contacts += usize::from(s.floor_contact);
            }
        }
        let end = f.sample().unwrap();
        assert!(
            (sum / count as f64 - 5. * 9.81).abs() < 1.,
            "{dt:?} support={}",
            sum / count as f64
        );
        assert!(contacts as f64 / count as f64 > 0.95);
        assert!((end.wheel.pose.translation_m[2] - f.parameters.radius_m).abs() < 0.001);
        assert!(
            end.suspension.body_poses[0].translation_m[2]
                < initial.suspension.body_poses[0].translation_m[2] - 0.003
        );
        assert!(end.chassis_velocity_m_s[2].abs() < 0.005);
    }
}

#[test]
fn traction_drives_the_frame_but_zero_friction_only_spins_the_wheel() {
    for dt in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let mut grip = Fixture::new(
            Parameters {
                bump_height_m: 0.,
                ..Default::default()
            },
            dt,
        )
        .unwrap();
        let mut ice = Fixture::new(
            Parameters {
                bump_height_m: 0.,
                friction: 0.,
                ..Default::default()
            },
            dt,
        )
        .unwrap();
        let start = 2_000_000_000 / dt.as_ns();
        let total = 5_000_000_000 / dt.as_ns();
        for tick in 0..total {
            if tick == start {
                grip.drive(0.25).unwrap();
                ice.drive(0.25).unwrap();
            }
            grip.step().unwrap();
            ice.step().unwrap();
        }
        let a = grip.sample().unwrap();
        let b = ice.sample().unwrap();
        assert!(
            a.suspension.body_poses[0].translation_m[0] > 0.5,
            "{dt:?} {a:?}"
        );
        assert!(a.slip_m_s.abs() < 0.01);
        assert!((a.chassis_velocity_m_s[0] - 0.25).abs() < 0.03);
        assert!(
            b.suspension.body_poses[0].translation_m[0].abs() < 0.02,
            "{dt:?} {b:?}"
        );
        assert!(b.chassis_velocity_m_s[0].abs() < 0.005);
        assert!(b.rolling_speed_m_s > 0.2);
        assert!(b.slip_m_s.abs() > 0.2);
    }
}

#[test]
fn ramp_contact_raises_wheel_and_frame_and_recovers_after_passage() {
    for dt in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let mut bump = Fixture::new(Parameters::default(), dt).unwrap();
        let mut flat = Fixture::new(
            Parameters {
                bump_height_m: 0.,
                ..Default::default()
            },
            dt,
        )
        .unwrap();
        let total = 8_000_000_000 / dt.as_ns();
        let start = 2_000_000_000 / dt.as_ns();
        let mut contacts = 0;
        let mut lift = 0_f64;
        let mut frame_lift = 0_f64;
        for tick in 0..total {
            if tick == start {
                bump.drive(0.25).unwrap();
                flat.drive(0.25).unwrap();
            }
            let a = bump.step().unwrap();
            let b = flat.step().unwrap();
            assert!(!b.bump_contact);
            assert_eq!(a.dropped_contacts, 0);
            assert!(a.suspension.length_m > 0.066 && a.suspension.length_m < 0.096);
            assert!(a.suspension.closure_error_m < 0.0001, "{dt:?} {a:?}");
            contacts += usize::from(a.bump_contact);
            if tick > start {
                lift = lift.max(a.wheel.pose.translation_m[2] - b.wheel.pose.translation_m[2]);
                frame_lift = frame_lift.max(
                    a.suspension.body_poses[0].translation_m[2]
                        - b.suspension.body_poses[0].translation_m[2],
                );
            }
        }
        assert!(contacts > 10);
        assert!(lift > 0.0035 && lift < 0.008, "{dt:?} lift={lift}");
        assert!(frame_lift > 0.002, "{dt:?} frame_lift={frame_lift}");
        let a = bump.sample().unwrap();
        let b = flat.sample().unwrap();
        assert!(a.wheel.pose.translation_m[0] > fixture::BUMP_END_M + 0.2);
        assert!(
            (a.suspension.body_poses[0].translation_m[2]
                - b.suspension.body_poses[0].translation_m[2])
                .abs()
                < 0.001
        );
    }
}

#[test]
fn recipe_reset_keeps_drive_choice_and_invalidates_old_body_tokens() {
    let mut f = Fixture::new(Parameters::default(), StepDuration::HZ_1000).unwrap();
    f.drive(0.2).unwrap();
    let old = f.wheel_binding();
    let expected = (0..512).map(|_| f.step().unwrap()).collect::<Vec<_>>();
    f.reset().unwrap();
    assert!(f.suspension.runtime.body_state(old).is_err());
    assert_ne!(old, f.wheel_binding());
    for want in expected {
        assert_eq!(f.step().unwrap(), want);
    }
}

#[test]
fn bad_setup_or_drive_requests_are_rejected_without_advancing_the_plant() {
    for p in [
        Parameters {
            friction: f64::NAN,
            ..Default::default()
        },
        Parameters {
            wheel_mass_kg: 0.,
            ..Default::default()
        },
        Parameters {
            bump_height_m: -1.,
            ..Default::default()
        },
    ] {
        assert!(Fixture::new(p, StepDuration::HZ_1000).is_err());
    }
    let mut f = Fixture::new(Parameters::default(), StepDuration::HZ_1000).unwrap();
    let initial = f.sample().unwrap();
    for request in [f64::NAN, f64::INFINITY, 1.1] {
        assert!(f.drive(request).is_err());
        assert_eq!(f.sample().unwrap(), initial);
    }
}
