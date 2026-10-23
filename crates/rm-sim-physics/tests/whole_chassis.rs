// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Free-chassis regressions, not validation of real hardware or rough-terrain fidelity.
#[allow(dead_code)]
#[path = "../examples/whole_chassis.rs"]
mod fixture;
use fixture::{Fixture, Parameters, Sample};
use rm_sim_physics::StepDuration;
fn run(f: &mut Fixture, dt: StepDuration, seconds: f64) -> Sample {
    let mut last = f.sample().unwrap();
    for _ in 0..(seconds / dt.as_secs_f64()).round() as u64 {
        last = f.step().unwrap();
        assert!(
            last.body
                .pose
                .translation_m
                .into_iter()
                .chain(last.body.pose.rotation_wxyz)
                .all(f64::is_finite)
        );
        assert!(last.closure_error_m < 0.0002, "{dt:?} {last:?}");
        assert_eq!(last.dropped_contacts, 0);
    }
    last
}
fn tilt(s: Sample) -> f64 {
    let [w, x, y, z] = s.body.pose.rotation_wxyz;
    let q = glam::DQuat::from_xyzw(x, y, z, w);
    (q * glam::DVec3::Z).angle_between(glam::DVec3::Z)
}
#[test]
fn free_chassis_settles_with_four_loads_balancing_total_weight() {
    for dt in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let mut f = Fixture::new(Parameters::default(), dt).unwrap();
        assert_eq!(f.runtime.participants().count(), 25);
        assert_eq!(f.sample().unwrap().loads_n, [None; 4]);
        run(&mut f, dt, 2.);
        let mut total = 0.;
        let n = (0.5 / dt.as_secs_f64()).round() as u64;
        for _ in 0..n {
            let s = f.step().unwrap();
            for load in s.loads_n {
                let load = load.unwrap();
                assert!((30. ..80.).contains(&load));
                total += load;
            }
            assert!(tilt(s) < 0.01);
            assert_eq!(s.dropped_contacts, 0);
        }
        assert!((total / n as f64 - f.total_mass_kg() * 9.81).abs() < 3.);
        let s = f.sample().unwrap();
        assert!((0.20..0.23).contains(&s.body.pose.translation_m[2]));
        assert!(s.velocity_body_m_s.into_iter().all(|v| v.abs() < 0.005));
    }
}
#[test]
fn wheels_produce_forward_sideways_diagonal_yaw_and_combined_motion() {
    let cases = [
        [0.25, 0., 0.],
        [0., 0.25, 0.],
        [0.18, 0.18, 0.],
        [0., 0., 0.6],
        [0.15, 0.10, 0.3],
    ];
    for dt in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        for target in cases {
            let mut f = Fixture::new(Parameters::default(), dt).unwrap();
            run(&mut f, dt, 2.);
            f.command(target[0], target[1], target[2]).unwrap();
            let s = run(&mut f, dt, 2.);
            assert!(
                (s.velocity_body_m_s[0] - target[0]).abs() < 0.05,
                "{dt:?} {target:?} {s:?}"
            );
            assert!(
                (s.velocity_body_m_s[1] - target[1]).abs() < 0.05,
                "{dt:?} {target:?} {s:?}"
            );
            assert!(
                (s.angular_body_rad_s[2] - target[2]).abs() < 0.08,
                "{dt:?} {target:?} {s:?}"
            );
            assert!(tilt(s) < 0.03);
            if target[2] == 0.6 {
                assert!(s.body.pose.rotation_wxyz[3].abs() > 0.4);
            }
        }
    }
}
#[test]
fn clearing_motors_coasts_while_zero_speed_brakes_and_both_remain_stable() {
    for dt in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let mut brake = Fixture::new(Parameters::default(), dt).unwrap();
        let mut coast = Fixture::new(Parameters::default(), dt).unwrap();
        for f in [&mut brake, &mut coast] {
            run(f, dt, 2.);
            f.command(0.25, 0., 0.).unwrap();
            run(f, dt, 2.);
        }
        brake.command(0., 0., 0.).unwrap();
        coast.coast().unwrap();
        let b = run(&mut brake, dt, 0.2);
        let c = run(&mut coast, dt, 0.2);
        assert!(b.velocity_body_m_s[0].abs() < 0.07, "{dt:?} {b:?}");
        assert!(
            c.velocity_body_m_s[0] > b.velocity_body_m_s[0] + 0.07,
            "{dt:?} brake={b:?}, coast={c:?}"
        );
        for f in [&mut brake, &mut coast] {
            let s = run(f, dt, 2.);
            assert!(tilt(s) < 0.02);
            assert!(s.velocity_body_m_s[0].abs() < 0.03);
        }
    }
}
#[test]
fn asymmetric_track_moves_chassis_attitude_and_clears_both_left_wheels() {
    for dt in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let mut flat = Fixture::new(Parameters::default(), dt).unwrap();
        let mut bump = Fixture::new(
            Parameters {
                bump_height_m: 0.002,
                ..Default::default()
            },
            dt,
        )
        .unwrap();
        for f in [&mut flat, &mut bump] {
            run(f, dt, 2.);
            f.command(0.25, 0., 0.).unwrap();
        }
        let mut lift = 0_f64;
        let mut max_tilt = 0_f64;
        let mut left_lift = [0_f64; 2];
        let mut end = bump.sample().unwrap();
        for _ in 0..6_000_000_000 / dt.as_ns() {
            let a = flat.step().unwrap();
            let b = bump.step().unwrap();
            lift = lift.max(b.body.pose.translation_m[2] - a.body.pose.translation_m[2]);
            max_tilt = max_tilt.max(tilt(b));
            for (slot, i) in [0usize, 2].into_iter().enumerate() {
                left_lift[slot] = left_lift[slot]
                    .max(b.wheels[i].pose.translation_m[2] - a.wheels[i].pose.translation_m[2]);
            }
            assert!(b.closure_error_m < 0.0002 && b.dropped_contacts == 0);
            end = b;
        }
        assert!(
            left_lift.into_iter().all(|v| v > 0.001),
            "{dt:?} {left_lift:?}"
        );
        assert!(
            lift > 0.0003 && max_tilt > 0.001 && max_tilt < 0.05,
            "{dt:?} lift={lift} tilt={max_tilt}"
        );
        assert!(end.wheels[2].pose.translation_m[0] > 0.9, "{dt:?} {end:?}");
        assert!(end.velocity_body_m_s[0] > 0.18);
    }
}
#[test]
fn com_offset_shifts_actual_front_and_rear_support_and_pitch() {
    for dt in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let mut f = Fixture::new(
            Parameters {
                com_offset_m: [0.05, 0., 0.],
                ..Default::default()
            },
            dt,
        )
        .unwrap();
        run(&mut f, dt, 3.);
        let mut front = 0.;
        let mut rear = 0.;
        let n = (0.5 / dt.as_secs_f64()).round() as u64;
        for _ in 0..n {
            let s = f.step().unwrap();
            front += s.loads_n[0].unwrap() + s.loads_n[1].unwrap();
            rear += s.loads_n[2].unwrap() + s.loads_n[3].unwrap();
            assert!(tilt(s) < 0.1);
        }
        assert!((front - rear) / n as f64 > 25.);
        assert!(((front + rear) / n as f64 - f.total_mass_kg() * 9.81).abs() < 3.);
        assert!(tilt(f.sample().unwrap()) > 0.001);
    }
}
#[test]
fn reset_replays_all_modules_with_fresh_tokens_and_invalid_commands_do_not_advance() {
    for dt in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let mut f = Fixture::new(Parameters::default(), dt).unwrap();
        let episode = |f: &mut Fixture| {
            f.command(0., 0., 0.).unwrap();
            let mut states = Vec::new();
            for tick in 0..2_500_000_000 / dt.as_ns() {
                if tick == 2_000_000_000 / dt.as_ns() {
                    f.command(0.12, 0.08, 0.2).unwrap();
                }
                states.push(f.step().unwrap());
            }
            states
        };
        let first = episode(&mut f);
        let old = f.body_binding();
        f.reset().unwrap();
        assert!(f.runtime.body_state(old).is_err());
        let s = f.sample().unwrap();
        for t in [[f64::NAN, 0., 0.], [0., 0., f64::INFINITY], [10., 0., 0.]] {
            assert!(f.command(t[0], t[1], t[2]).is_err());
            assert_eq!(s, f.sample().unwrap());
        }
        assert_eq!(first, episode(&mut f));
    }
    for p in [
        Parameters {
            sprung_mass_kg: 0.,
            ..Default::default()
        },
        Parameters {
            com_offset_m: [f64::NAN, 0., 0.],
            ..Default::default()
        },
        Parameters {
            solver_iterations: 0,
            ..Default::default()
        },
        Parameters {
            roller_friction: -1.,
            ..Default::default()
        },
    ] {
        assert!(Fixture::new(p, StepDuration::HZ_1000).is_err());
    }
}
