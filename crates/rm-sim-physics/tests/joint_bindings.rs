// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Scalar joints constrain ordinary described bodies in the shared scene.
use rm_sim_engine::{
    Pose,
    model::{
        BodyKind, BodyState, ColliderDescription, ContactMaterial, Inertial, JointDescription,
        JointKind, JointLimits, JointMotor, RigidBodyDescription, ShapeDescription,
    },
};
use rm_sim_physics::{
    StepDuration, TargetFrames,
    cooking::CookedRigidBody,
    runtime::{BodyBinding, BodyKey, JointBinding, JointKey, WorldPhysics},
};

fn world(duration: StepDuration) -> WorldPhysics {
    WorldPhysics::with_step_duration(&[], -100., duration)
}
fn dynamic() -> BodyKind {
    BodyKind::Dynamic(Inertial::new(2., [2.; 3], Pose::default()).unwrap())
}
fn install(
    world: &mut WorldPhysics,
    key: u64,
    kind: BodyKind,
    state: BodyState,
    colliders: bool,
) -> BodyBinding {
    let colliders = if colliders {
        vec![
            ColliderDescription::new(
                ShapeDescription::Cuboid {
                    half_extents_m: [0.2; 3],
                },
                Pose::default(),
                ContactMaterial {
                    friction: 0.,
                    restitution: 0.,
                    memberships: u32::MAX,
                    filter: u32::MAX,
                },
            )
            .unwrap(),
        ]
    } else {
        vec![]
    };
    let model = CookedRigidBody::new(RigidBodyDescription::new(kind, colliders).unwrap()).unwrap();
    world.install_body(BodyKey(key), &model, state).unwrap()
}
fn description(kind: JointKind, limits: Option<JointLimits>) -> JointDescription {
    JointDescription::new(kind, Pose::default(), Pose::default(), limits, false).unwrap()
}
fn pair(world: &mut WorldPhysics, kind: JointKind) -> (BodyBinding, BodyBinding, JointBinding) {
    let a = install(
        world,
        1,
        BodyKind::Fixed,
        BodyState::at(Pose::default()),
        false,
    );
    let b = install(world, 2, dynamic(), BodyState::at(Pose::default()), false);
    let joint = world
        .install_joint(JointKey(1), a, b, &description(kind, None))
        .unwrap();
    (a, b, joint)
}
fn advance(world: &mut WorldPhysics, ticks: u64) {
    let frames = TargetFrames::new(vec![]);
    for tick in 0..ticks {
        world
            .step(tick * world.step_duration().as_ns(), &frames)
            .unwrap();
    }
}

#[test]
fn fixed_joint_locks_pose_and_rejects_actuation() {
    let mut w = world(StepDuration::HZ_1000);
    let (a, b, joint) = pair(&mut w, JointKind::Fixed);
    assert_eq!(joint.key(), JointKey(1));
    assert_eq!(
        *w.joint_description(joint).unwrap(),
        description(JointKind::Fixed, None)
    );
    assert!(w.set_joint_effort(joint, 1.).is_err());
    assert!(
        w.set_joint_motor(joint, Some(JointMotor::new(0., 0., 1., 1., 1.).unwrap()))
            .is_err()
    );
    w.set_joint_effort(joint, 0.).unwrap();
    w.set_joint_motor(joint, None).unwrap();
    w.set_body_wrench(b, [100., 0., 0.], [0., 5., 0.]).unwrap();
    advance(&mut w, 200);
    let state = w.joint_state(joint).unwrap();
    assert_eq!((state.position, state.velocity), (0., 0.));
    for body in [a, b] {
        let state = w.body_state(body).unwrap();
        assert!(
            state.pose.translation_m.into_iter().all(|x| x.abs() < 1e-5),
            "{state:?}"
        );
        assert!(
            state
                .angular_velocity_rad_s
                .into_iter()
                .all(|x| x.abs() < 1e-5)
        );
    }
}

#[test]
fn held_effort_is_per_slice_and_superposes_with_held_body_wrench() {
    for duration in [StepDuration::HZ_1000, StepDuration::HZ_128] {
        for kind in [JointKind::Revolute, JointKind::Prismatic] {
            let mut w = world(duration);
            let (_, b, joint) = pair(&mut w, kind);
            let (force, torque) = if kind == JointKind::Prismatic {
                ([2., 0., 0.], [0.; 3])
            } else {
                ([0.; 3], [2., 0., 0.])
            };
            w.set_body_wrench(b, force, torque).unwrap();
            w.set_joint_effort(joint, 6.).unwrap();
            advance(&mut w, 20);
            let t = duration.as_secs_f64() * 20.;
            let state = w.joint_state(joint).unwrap();
            // Mass/inertia are both 2: net scalar acceleration = (6 + 2)/2.
            assert!(
                (state.velocity - 4. * t).abs() < 2e-6,
                "{kind:?}: {state:?}, t={t}"
            );
            assert!((state.position - 2. * t * t).abs() < 0.001, "{state:?}");
            w.set_joint_effort(joint, -2.).unwrap();
            advance(&mut w, 20);
            assert!((w.joint_state(joint).unwrap().velocity - state.velocity).abs() < 2e-6);
            w.set_joint_effort(joint, 0.).unwrap();
            advance(&mut w, 20);
            assert!((w.joint_state(joint).unwrap().velocity - state.velocity - t).abs() < 2e-6);
        }
    }
}

#[test]
fn motors_replace_effort_and_validate_limits_without_mutation() {
    for kind in [JointKind::Revolute, JointKind::Prismatic] {
        let mut w = world(StepDuration::HZ_1000);
        let (a, b, old) = pair(&mut w, kind);
        w.remove_bound_joint(old).unwrap();
        let source = description(kind, Some(JointLimits::new(-0.2, 0.3).unwrap()));
        let joint = w.install_joint(JointKey(1), a, b, &source).unwrap();
        w.set_joint_effort(joint, -10.).unwrap();
        let motor = JointMotor::new(0.2, 0., 100., 20., 4.).unwrap();
        w.set_joint_motor(joint, Some(motor)).unwrap();
        assert!(
            w.set_joint_motor(joint, Some(JointMotor::new(0.31, 0., 1., 1., 1.).unwrap()))
                .is_err()
        );
        assert!(w.set_joint_effort(joint, f64::NAN).is_err());
        advance(&mut w, 10);
        // Effort saturates at +4 N or N m, unaffected by the prior -10 request.
        assert!((w.joint_state(joint).unwrap().velocity - 0.02).abs() < 2e-6);
        advance(&mut w, 1990);
        assert!((w.joint_state(joint).unwrap().position - 0.2).abs() < 0.002);
        w.set_joint_effort(joint, -4.).unwrap();
        let before = w.joint_state(joint).unwrap();
        advance(&mut w, 20);
        assert!(w.joint_state(joint).unwrap().velocity < before.velocity - 0.035);
        w.set_joint_motor(joint, None).unwrap();
        let before = w.joint_state(joint).unwrap();
        advance(&mut w, 20);
        assert!((w.joint_state(joint).unwrap().velocity - before.velocity).abs() < 2e-6);
        w.set_joint_effort(joint, 100.).unwrap();
        advance(&mut w, 1000);
        assert!(w.joint_state(joint).unwrap().position <= 0.3001);
    }
}

#[test]
fn finite_extreme_motor_terms_never_inject_nan() {
    let mut w = world(StepDuration::HZ_1000);
    let (_, body, joint) = pair(&mut w, JointKind::Prismatic);
    // Both products overflow with opposite signs in ordinary arithmetic.
    w.set_joint_motor(
        joint,
        Some(JointMotor::new(2., -2., f64::MAX, f64::MAX, 1.).unwrap()),
    )
    .unwrap();
    advance(&mut w, 10);
    let state = w.joint_state(joint).unwrap();
    assert!(
        state.position.is_finite() && state.velocity.is_finite(),
        "{state:?}"
    );
    assert!(state.velocity.abs() < 0.01);
    let body = w.body_state(body).unwrap();
    assert!(
        body.velocity_m_s
            .into_iter()
            .chain(body.angular_velocity_rad_s)
            .all(f64::is_finite)
    );
    w.set_joint_motor(
        joint,
        Some(JointMotor::new(4., -2., f64::MAX, f64::MAX, 1.).unwrap()),
    )
    .unwrap();
    advance(&mut w, 10);
    assert!(w.joint_state(joint).unwrap().velocity > state.velocity);
}

#[test]
fn cleanup_key_reuse_and_wrong_world_invalidate_only_relevant_tokens() {
    let mut w = world(StepDuration::HZ_1000);
    let (a, b, joint) = pair(&mut w, JointKind::Revolute);
    let mut other = world(StepDuration::HZ_1000);
    let (_, _, foreign) = pair(&mut other, JointKind::Revolute);
    assert_ne!(joint, foreign);
    assert!(other.joint_state(joint).is_err());
    assert!(other.joint_description(joint).is_err());
    assert!(other.set_joint_effort(joint, 1.).is_err());
    assert!(other.set_joint_motor(joint, None).is_err());
    assert!(other.remove_bound_joint(joint).is_err());
    assert!(
        other
            .install_joint(JointKey(9), a, b, &description(JointKind::Fixed, None))
            .is_err()
    );
    assert!(
        w.install_joint(JointKey(1), a, b, &description(JointKind::Fixed, None))
            .is_err()
    );
    assert!(
        w.install_joint(JointKey(2), a, a, &description(JointKind::Fixed, None))
            .is_err()
    );
    w.remove_bound_joint(joint).unwrap();
    assert!(w.joint_state(joint).is_err());
    let replacement = w
        .install_joint(JointKey(1), a, b, &description(JointKind::Revolute, None))
        .unwrap();
    assert_ne!(joint, replacement);
    let c = install(&mut w, 3, dynamic(), BodyState::at(Pose::default()), false);
    let independent = w
        .install_joint(JointKey(2), a, c, &description(JointKind::Fixed, None))
        .unwrap();
    let linked = w
        .install_joint(JointKey(3), b, c, &description(JointKind::Fixed, None))
        .unwrap();
    w.remove_bound_body(b).unwrap();
    assert!(w.joint_binding(JointKey(1)).is_none());
    assert!(w.joint_binding(JointKey(3)).is_none());
    for dead in [joint, replacement, linked] {
        assert!(w.joint_state(dead).is_err());
        assert!(w.remove_bound_joint(dead).is_err());
        assert!(w.set_joint_effort(dead, 1.).is_err());
    }
    assert!(w.joint_state(independent).is_ok());
    assert!(w.body_state(a).is_ok() && w.body_state(c).is_ok());
    let b = install(&mut w, 2, dynamic(), BodyState::at(Pose::default()), false);
    let fresh = w
        .install_joint(JointKey(1), a, b, &description(JointKind::Revolute, None))
        .unwrap();
    assert_ne!(fresh, replacement);
    // Reconstructing the world is the runtime component of scenario reset.
    w = world(StepDuration::HZ_1000);
    let (_, _, after_reset) = pair(&mut w, JointKind::Revolute);
    assert_ne!(fresh, after_reset);
    assert!(w.joint_state(fresh).is_err());
}

#[test]
fn unsupported_initial_frames_velocities_and_body_modes_fail_before_installation() {
    for kind in [JointKind::Fixed, JointKind::Revolute, JointKind::Prismatic] {
        let mut w = world(StepDuration::HZ_1000);
        let a = install(
            &mut w,
            1,
            BodyKind::Fixed,
            BodyState::at(Pose::default()),
            false,
        );
        let fixed = install(
            &mut w,
            2,
            BodyKind::Fixed,
            BodyState::at(Pose::default()),
            false,
        );
        let prescribed = install(
            &mut w,
            3,
            BodyKind::Kinematic,
            BodyState::at(Pose::default()),
            false,
        );
        for b in [fixed, prescribed] {
            assert!(
                w.install_joint(JointKey(1), a, b, &description(kind, None))
                    .is_err()
            );
        }
        let bad = install(
            &mut w,
            4,
            dynamic(),
            BodyState::at(Pose::at([0., 0.1, 0.])),
            false,
        );
        assert!(
            w.install_joint(JointKey(1), a, bad, &description(kind, None))
                .is_err()
        );
        w.remove_bound_body(bad).unwrap();
        let bad = install(
            &mut w,
            4,
            dynamic(),
            BodyState::at(Pose::yawed([0.; 3], 0.1)),
            false,
        );
        assert!(
            w.install_joint(JointKey(1), a, bad, &description(kind, None))
                .is_err()
        );
        w.remove_bound_body(bad).unwrap();
        let bad = install(
            &mut w,
            4,
            dynamic(),
            BodyState {
                velocity_m_s: [0., 1., 0.],
                ..BodyState::at(Pose::default())
            },
            false,
        );
        assert!(
            w.install_joint(JointKey(1), a, bad, &description(kind, None))
                .is_err()
        );
        w.set_body_velocity(bad, [0.; 3], [0., 1., 0.]).unwrap();
        assert!(
            w.install_joint(JointKey(1), a, bad, &description(kind, None))
                .is_err()
        );
        w.set_body_velocity(bad, [0.; 3], [0.; 3]).unwrap();
        let good = w
            .install_joint(JointKey(1), a, bad, &description(kind, None))
            .unwrap();
        assert_eq!(w.joint_binding(JointKey(1)), Some(good));
    }
}

#[test]
fn free_initial_coordinates_are_measured_and_limits_are_inclusive() {
    for kind in [JointKind::Revolute, JointKind::Prismatic] {
        let mut w = world(StepDuration::HZ_1000);
        let a = install(
            &mut w,
            1,
            BodyKind::Fixed,
            BodyState::at(Pose::default()),
            false,
        );
        let pose = if kind == JointKind::Prismatic {
            Pose::at([0.2, 0., 0.])
        } else {
            Pose {
                rotation_wxyz: [(0.1_f64).cos(), (0.1_f64).sin(), 0., 0.],
                ..Pose::default()
            }
        };
        let mut state = BodyState::at(pose);
        if kind == JointKind::Prismatic {
            state.velocity_m_s[0] = 0.4;
        } else {
            state.angular_velocity_rad_s[0] = 0.4;
        }
        let b = install(&mut w, 2, dynamic(), state, false);
        let outside = description(kind, Some(JointLimits::new(-0.1, 0.1).unwrap()));
        assert!(w.install_joint(JointKey(1), a, b, &outside).is_err());
        let source = description(kind, Some(JointLimits::new(-0.2, 0.2).unwrap()));
        let joint = w.install_joint(JointKey(1), a, b, &source).unwrap();
        let state = w.joint_state(joint).unwrap();
        assert!((state.position - 0.2).abs() < 1e-12);
        assert!((state.velocity - 0.4).abs() < 1e-12);
        w.set_joint_motor(joint, Some(JointMotor::new(0.2, 0., 1., 1., 1.).unwrap()))
            .unwrap();
        if kind == JointKind::Revolute {
            w.remove_bound_joint(joint).unwrap();
            let joint = w
                .install_joint(JointKey(1), a, b, &description(kind, None))
                .unwrap();
            assert!(
                w.set_joint_motor(joint, Some(JointMotor::new(4., 0., 1., 1., 1.).unwrap()))
                    .is_err()
            );
        }
    }
}

#[test]
fn current_world_axis_and_equal_opposite_dynamic_reaction_have_correct_sign() {
    for kind in [JointKind::Revolute, JointKind::Prismatic] {
        let mut w = world(StepDuration::HZ_1000);
        let initial = BodyState::at(Pose::yawed([0.; 3], std::f64::consts::FRAC_PI_2));
        let a = install(&mut w, 1, dynamic(), initial, false);
        let b = install(&mut w, 2, dynamic(), initial, false);
        let joint = w
            .install_joint(JointKey(1), a, b, &description(kind, None))
            .unwrap();
        w.set_joint_effort(joint, 4.).unwrap();
        advance(&mut w, 20);
        let a = w.body_state(a).unwrap();
        let b = w.body_state(b).unwrap();
        let (va, vb) = if kind == JointKind::Prismatic {
            (a.velocity_m_s, b.velocity_m_s)
        } else {
            (a.angular_velocity_rad_s, b.angular_velocity_rad_s)
        };
        assert!(va[0].abs() < 1e-8 && vb[0].abs() < 1e-8);
        assert!(
            (va[1] + 0.04).abs() < 1e-6 && (vb[1] - 0.04).abs() < 1e-6,
            "{kind:?}: {va:?} {vb:?}"
        );
        assert!((w.joint_state(joint).unwrap().velocity - 0.08).abs() < 1e-6);
    }
}

#[test]
fn contacts_block_actuated_slider_and_pair_contact_policy_is_honored() {
    fn slider(wall: bool) -> (f64, f64) {
        let mut w = world(StepDuration::HZ_1000);
        let a = install(
            &mut w,
            1,
            BodyKind::Fixed,
            BodyState::at(Pose::default()),
            false,
        );
        let b = install(&mut w, 2, dynamic(), BodyState::at(Pose::default()), true);
        if wall {
            install(
                &mut w,
                3,
                BodyKind::Fixed,
                BodyState::at(Pose::at([1., 0., 0.])),
                true,
            );
        }
        let joint = w
            .install_joint(JointKey(1), a, b, &description(JointKind::Prismatic, None))
            .unwrap();
        w.set_joint_motor(
            joint,
            Some(JointMotor::new(2., 0., 100., 20., 10.).unwrap()),
        )
        .unwrap();
        advance(&mut w, 1500);
        let state = w.joint_state(joint).unwrap();
        (state.position, state.velocity)
    }
    let free = slider(false);
    let blocked = slider(true);
    assert!(free.0 > 1.5, "{free:?}");
    assert!(
        (blocked.0 - 0.6).abs() < 0.002 && blocked.1.abs() < 0.01,
        "{blocked:?}"
    );
    let mut positions = Vec::new();
    for contacts in [false, true] {
        let mut w = world(StepDuration::HZ_1000);
        let a = install(
            &mut w,
            1,
            BodyKind::Fixed,
            BodyState::at(Pose::default()),
            true,
        );
        let b = install(
            &mut w,
            2,
            dynamic(),
            BodyState::at(Pose::at([0.3, 0., 0.])),
            true,
        );
        let source = JointDescription::new(
            JointKind::Prismatic,
            Pose::default(),
            Pose::default(),
            None,
            contacts,
        )
        .unwrap();
        let joint = w.install_joint(JointKey(1), a, b, &source).unwrap();
        advance(&mut w, 100);
        positions.push(w.joint_state(joint).unwrap().position);
    }
    assert!((positions[0] - 0.3).abs() < 1e-8, "{positions:?}");
    assert!(positions[1] > 0.38, "{positions:?}");
}

#[test]
fn installed_source_com_is_used_for_admission_and_first_slice_anchor_torque() {
    fn offset_body(w: &mut WorldPhysics, state: BodyState) -> BodyBinding {
        let inertia = Inertial::new(2., [2.; 3], Pose::at([0., 0., 1.])).unwrap();
        let model = CookedRigidBody::new(
            RigidBodyDescription::new(BodyKind::Dynamic(inertia), vec![]).unwrap(),
        )
        .unwrap();
        w.install_body(BodyKey(2), &model, state).unwrap()
    }
    let mut w = world(StepDuration::HZ_1000);
    let a = install(
        &mut w,
        1,
        BodyKind::Fixed,
        BodyState::at(Pose::default()),
        false,
    );
    // omega x (anchor - COM) = +Y cancels the incoming -Y COM velocity.
    let b = offset_body(
        &mut w,
        BodyState {
            velocity_m_s: [0., -1., 0.],
            angular_velocity_rad_s: [1., 0., 0.],
            ..BodyState::at(Pose::default())
        },
    );
    let hinge = w
        .install_joint(JointKey(1), a, b, &description(JointKind::Revolute, None))
        .unwrap();
    assert_eq!(w.joint_state(hinge).unwrap().velocity, 1.);

    let mut w = world(StepDuration::HZ_1000);
    let a = install(&mut w, 1, dynamic(), BodyState::at(Pose::default()), false);
    let b = offset_body(&mut w, BodyState::at(Pose::default()));
    let slider = w
        .install_joint(JointKey(1), a, b, &description(JointKind::Prismatic, None))
        .unwrap();
    w.set_joint_effort(slider, 4.).unwrap();
    advance(&mut w, 1);
    let a = w.body_state(a).unwrap();
    let b = w.body_state(b).unwrap();
    // +X at the child's anchor below its COM generates -Y torque. Angular
    // locking shares that torque equally between the two 2 kg m^2 bodies.
    assert!((a.angular_velocity_rad_s[1] + 0.001).abs() < 1e-7, "{a:?}");
    assert!((b.angular_velocity_rad_s[1] + 0.001).abs() < 1e-7, "{b:?}");
    assert!((a.velocity_m_s[0] + 0.002).abs() < 1e-7);
    assert!((b.velocity_m_s[0] - 0.002).abs() < 1e-7);
}

#[test]
fn held_slider_effort_follows_a_rotating_parent_axis() {
    let mut w = world(StepDuration::HZ_1000);
    let initial = BodyState {
        angular_velocity_rad_s: [0., 0., 2.],
        ..BodyState::at(Pose::default())
    };
    let a = install(&mut w, 1, dynamic(), initial, false);
    let b = install(&mut w, 2, dynamic(), initial, false);
    let joint = w
        .install_joint(JointKey(1), a, b, &description(JointKind::Prismatic, None))
        .unwrap();
    w.set_joint_effort(joint, 4.).unwrap();
    advance(&mut w, 250);
    let a = w.body_state(a).unwrap();
    let b = w.body_state(b).unwrap();
    // An axis frozen at installation would supply no net Y impulse. The
    // current rotating frame does; total linear momentum still cancels.
    assert!(
        b.velocity_m_s[1] > 0.1 && a.velocity_m_s[1] < -0.1,
        "{a:?} {b:?}"
    );
    for axis in 0..2 {
        assert!((a.velocity_m_s[axis] + b.velocity_m_s[axis]).abs() < 1e-8);
    }
    assert!(w.joint_state(joint).unwrap().velocity > 0.5);
}
