// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Directional traction, passive slip and installation identity checks.
use rm_sim_engine::{
    Pose,
    model::{
        BodyKind, BodyPolicy, BodyState, ColliderDescription, ContactMaterial, Inertial,
        RigidBodyDescription, ShapeDescription,
    },
};
use rm_sim_physics::{
    StepDuration,
    cooking::CookedRigidBody,
    runtime::{BodyBinding, BodyKey, ColliderBinding, DirectionalFriction, PhysicalRuntime},
};

fn block(
    dt: StepDuration,
    yaw: bool,
    floor_first: bool,
) -> (PhysicalRuntime, BodyBinding, ColliderBinding) {
    let mut r = PhysicalRuntime::new(dt);
    if floor_first {
        r.install_catch_floor(
            0.,
            ContactMaterial {
                friction: 0.9,
                restitution: 0.,
                memberships: u32::MAX,
                filter: u32::MAX,
            },
        )
        .unwrap();
    }
    let shape = ColliderDescription::new(
        ShapeDescription::Cuboid {
            half_extents_m: [0.1; 3],
        },
        Pose::default(),
        ContactMaterial {
            friction: 0.9,
            restitution: 0.,
            memberships: u32::MAX,
            filter: u32::MAX,
        },
    )
    .unwrap();
    let source = RigidBodyDescription::new(
        BodyKind::Dynamic(Inertial::new(1., [0.006666666666666667; 3], Pose::default()).unwrap()),
        vec![shape],
    )
    .unwrap()
    .with_policy(BodyPolicy {
        enabled_rotations: [false; 3],
        can_sleep: false,
        ..Default::default()
    })
    .unwrap();
    let mut p = Pose::at([0., 0., 0.1001]);
    if yaw {
        p.rotation_wxyz = [
            std::f64::consts::FRAC_1_SQRT_2,
            0.,
            0.,
            std::f64::consts::FRAC_1_SQRT_2,
        ];
    }
    let b = r
        .install_body(
            BodyKey(1),
            &CookedRigidBody::new(source).unwrap(),
            BodyState::at(p),
        )
        .unwrap();
    if !floor_first {
        r.install_catch_floor(
            0.,
            ContactMaterial {
                friction: 0.9,
                restitution: 0.,
                memberships: u32::MAX,
                filter: u32::MAX,
            },
        )
        .unwrap();
    }
    let participant = r.body_participant(b).unwrap();
    let c = r
        .colliders()
        .find(|&c| r.collider_state(c).unwrap().participant == Some(participant))
        .unwrap();
    (r, b, c)
}
fn advance(r: &mut PhysicalRuntime, time: &mut u64, duration: u64) {
    for _ in 0..duration / r.step_duration().as_ns() {
        r.step(*time).unwrap();
        *time += r.step_duration().as_ns();
    }
}
#[test]
fn separate_limits_follow_rotated_collider_axes_and_pair_order() {
    for dt in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        for yaw in [false, true] {
            for first in [false, true] {
                let (mut r, b, c) = block(dt, yaw, first);
                r.set_directional_friction(
                    c,
                    Some(DirectionalFriction::new([0., 1., 0.], 0.9, 0.).unwrap()),
                )
                .unwrap();
                let mut t = 0;
                advance(&mut r, &mut t, 1_000_000_000);
                r.set_body_velocity(b, [0.3, 0.3, 0.], [0.; 3]).unwrap();
                let mut previous_energy = 0.18;
                for _ in 0..1_000_000_000 / dt.as_ns() {
                    r.step(t).unwrap();
                    t += dt.as_ns();
                    let v = r.body_state(b).unwrap().velocity_m_s;
                    let energy = v[0] * v[0] + v[1] * v[1];
                    assert!(energy <= previous_energy + 1e-8, "{dt:?} {v:?}");
                    previous_energy = energy;
                }
                let v = r.body_state(b).unwrap().velocity_m_s;
                let (stopped, free) = if yaw { (v[1], v[0]) } else { (v[0], v[1]) };
                assert!(stopped.abs() < 1e-5, "{dt:?} {yaw} {first} {v:?}");
                assert!((free - 0.3).abs() < 1e-6, "{dt:?} {v:?}");
            }
        }
    }
}
#[test]
fn finite_roller_drag_is_load_limited_and_removal_restores_native_grip() {
    for dt in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let (mut r, b, c) = block(dt, false, true);
        r.set_directional_friction(
            c,
            Some(DirectionalFriction::new([0., 1., 0.], 0.9, 0.02).unwrap()),
        )
        .unwrap();
        let mut t = 0;
        advance(&mut r, &mut t, 1_000_000_000);
        r.set_body_velocity(b, [0., 0.3, 0.], [0.; 3]).unwrap();
        advance(&mut r, &mut t, 500_000_000);
        let v = r.body_state(b).unwrap().velocity_m_s[1];
        assert!((v - (0.3 - 0.02 * 9.81 * 0.5)).abs() < 0.005, "{dt:?} {v}");
        r.set_directional_friction(c, None).unwrap();
        advance(&mut r, &mut t, 500_000_000);
        assert!(r.body_state(b).unwrap().velocity_m_s[1].abs() < 1e-5);
    }
}
#[test]
fn invalid_coefficients_and_foreign_or_removed_bindings_are_rejected() {
    for axis in [[0.; 3], [f64::NAN, 0., 0.], [2., 0., 0.]] {
        assert!(DirectionalFriction::new(axis, 0.9, 0.02).is_err());
    }
    for mu in [-1., f64::NAN, f64::INFINITY, 11.] {
        assert!(DirectionalFriction::new([0., 1., 0.], mu, 0.).is_err());
        assert!(DirectionalFriction::new([0., 1., 0.], 0.9, mu).is_err());
    }
    let (mut r, b, c) = block(StepDuration::HZ_1000, false, true);
    let (mut other, _, _) = block(StepDuration::HZ_1000, false, true);
    let law = Some(DirectionalFriction::new([0., 1., 0.], 0.9, 0.02).unwrap());
    assert!(other.set_directional_friction(c, law).is_err());
    r.set_directional_friction(c, law).unwrap();
    r.remove_bound_body(b).unwrap();
    assert!(r.set_directional_friction(c, law).is_err());
    r.step(0).unwrap();
}

#[test]
fn airborne_motion_receives_no_friction_and_invalid_damping_does_not_change_law() {
    for value in [0., -1., f64::NAN, f64::INFINITY, 1e7] {
        assert!(
            DirectionalFriction::new([0., 1., 0.], 0.9, 0.02)
                .unwrap()
                .with_slip_damping(value)
                .is_err()
        );
    }
    for dt in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let (mut r, b, c) = block(dt, false, true);
        let floor = r.colliders().find(|&x| x != c).unwrap();
        assert!(
            r.set_directional_friction(
                floor,
                Some(DirectionalFriction::new([0., 1., 0.], 0.9, 0.02).unwrap())
            )
            .is_err()
        );
        r.remove_fixed_collider(floor).unwrap();
        r.set_directional_friction(
            c,
            Some(DirectionalFriction::new([0., 1., 0.], 0.9, 0.02).unwrap()),
        )
        .unwrap();
        r.set_body_velocity(b, [0.3, 0.3, 0.], [0.; 3]).unwrap();
        let mut t = 0;
        advance(&mut r, &mut t, 500_000_000);
        let v = r.body_state(b).unwrap().velocity_m_s;
        assert!((v[0] - 0.3).abs() < 1e-12 && (v[1] - 0.3).abs() < 1e-12);
        assert!(v[2] < -4.);
    }
}
