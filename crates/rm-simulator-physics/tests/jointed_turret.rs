// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! End-to-end physical two-axis fixture using the executable's scenario recipe.
#[allow(dead_code)]
#[path = "../examples/jointed_turret.rs"]
mod fixture;

use fixture::{BARREL, BASE, PITCH, PITCH_LIMIT_RAD, YAW, YAW_LIMIT_RAD, YOKE, aim, scenario};
use glam::{DQuat, DVec3};
use rm_simulator_engine::Pose;
use rm_simulator_physics::{runtime::JointKey, session::RobotSession};

fn joint(session: &RobotSession, key: JointKey) -> (f64, f64) {
    let world = session.ground_truth();
    let state = world
        .joint_state(world.joint_binding(key).unwrap())
        .unwrap();
    (state.position, state.velocity)
}

fn point(pose: Pose, local: [f64; 3]) -> DVec3 {
    let [w, x, y, z] = pose.rotation_wxyz;
    DVec3::from_array(pose.translation_m) + DQuat::from_xyzw(x, y, z, w) * DVec3::from_array(local)
}

fn assert_anchors(session: &RobotSession) {
    let world = session.ground_truth();
    let pose = |key| {
        world
            .body_state(world.body_binding(key).unwrap())
            .unwrap()
            .pose
    };
    let yaw_error = (point(pose(BASE), [0., 0., 0.1]) - point(pose(YOKE), [0., 0., -0.1])).length();
    let pitch_error =
        (point(pose(YOKE), [0., 0., 0.2]) - point(pose(BARREL), [-0.25, 0., 0.])).length();
    assert!(yaw_error < 0.003, "yaw anchor separation {yaw_error} m");
    assert!(
        pitch_error < 0.003,
        "pitch anchor separation {pitch_error} m"
    );
}

#[test]
fn physical_turret_tracks_both_targets_and_reverses_without_anchor_drift() {
    let mut session = RobotSession::new(scenario(false).unwrap()).unwrap();
    for (yaw, pitch) in [(0.6, -0.25), (-0.5, 0.2)] {
        aim(&mut session, yaw, pitch).unwrap();
        session.step(2000).unwrap();
        for (key, target) in [(YAW, yaw), (PITCH, pitch)] {
            let (position, velocity) = joint(&session, key);
            // Finite stiffness leaves a small gravity-loaded pitch error.
            assert!(
                (position - target).abs() < 0.025,
                "{key:?}: {position} vs {target}"
            );
            assert!(velocity.abs() < 0.03, "{key:?}: {velocity} rad/s");
        }
        assert_anchors(&session);
        let world = session.ground_truth();
        let barrel = world
            .body_state(world.body_binding(BARREL).unwrap())
            .unwrap()
            .pose;
        let direction = point(barrel, [1., 0., 0.]) - point(barrel, [0.; 3]);
        let expected = DVec3::new(
            yaw.cos() * pitch.cos(),
            yaw.sin() * pitch.cos(),
            -pitch.sin(),
        );
        assert!(
            (direction - expected).length() < 0.03,
            "barrel direction {direction:?}"
        );
    }
}

#[test]
fn hard_stops_hold_both_signs_under_held_effort() {
    let mut session = RobotSession::new(scenario(false).unwrap()).unwrap();
    for sign in [1., -1.] {
        for key in [YAW, PITCH] {
            let binding = session.ground_truth().joint_binding(key).unwrap();
            session
                .physics_mut()
                .set_joint_effort(binding, sign * 5.)
                .unwrap();
        }
        // Check the entire transient, not just the settled endpoint.
        for _ in 0..200 {
            session.step(10).unwrap();
            for (key, limit) in [(YAW, YAW_LIMIT_RAD), (PITCH, PITCH_LIMIT_RAD)] {
                let (position, _) = joint(&session, key);
                assert!(position.abs() <= limit + 0.025, "{key:?}: {position}");
            }
        }
        for (key, limit) in [(YAW, YAW_LIMIT_RAD), (PITCH, PITCH_LIMIT_RAD)] {
            let (position, velocity) = joint(&session, key);
            assert!(
                (position - sign * limit).abs() < 0.025,
                "{key:?}: {position}"
            );
            assert!(velocity.abs() < 0.05, "{key:?}: {velocity}");
        }
        assert_anchors(&session);
    }
}

#[test]
fn recipe_reset_invalidates_body_and_joint_tokens_and_replays_initial_response() {
    let mut session = RobotSession::new(scenario(false).unwrap()).unwrap();
    let old_joint = session.ground_truth().joint_binding(YAW).unwrap();
    let old_body = session.ground_truth().body_binding(BARREL).unwrap();
    aim(&mut session, 0.4, -0.2).unwrap();
    session.step(500).unwrap();
    let first = (joint(&session, YAW), joint(&session, PITCH));
    session.reset().unwrap();
    assert!(session.ground_truth().joint_state(old_joint).is_err());
    assert!(
        session
            .physics_mut()
            .set_joint_effort(old_joint, 5.)
            .is_err()
    );
    assert!(session.ground_truth().body_state(old_body).is_err());
    assert_ne!(
        session.ground_truth().joint_binding(YAW).unwrap(),
        old_joint
    );
    assert_ne!(
        session.ground_truth().body_binding(BARREL).unwrap(),
        old_body
    );
    assert_eq!(joint(&session, YAW), (0., 0.));
    assert_eq!(joint(&session, PITCH), (0., 0.));
    aim(&mut session, 0.4, -0.2).unwrap();
    session.step(500).unwrap();
    let replay = (joint(&session, YAW), joint(&session, PITCH));
    for (actual, expected) in [
        (replay.0.0, first.0.0),
        (replay.0.1, first.0.1),
        (replay.1.0, first.1.0),
        (replay.1.1, first.1.1),
    ] {
        assert!((actual - expected).abs() < 1e-10);
    }
}

#[test]
fn motor_reacts_on_dynamic_parent_and_floor_supports_connected_assembly() {
    let mut session = RobotSession::new(scenario(true).unwrap()).unwrap();
    aim(&mut session, 0.6, 0.).unwrap();
    session.step(100).unwrap();
    let world = session.ground_truth();
    let base = world.body_state(world.body_binding(BASE).unwrap()).unwrap();
    assert!(
        base.angular_velocity_rad_s[2] < -0.03,
        "missing yaw reaction: {base:?}"
    );
    assert!(joint(&session, YAW).0 > 0.1);
    assert_anchors(&session);
    // The complete assembly falls onto the ordinary runtime floor. A prescribed
    // child would remain aloft; an unconstrained child would separate on impact.
    aim(&mut session, 0., 0.).unwrap();
    session.step(2900).unwrap();
    let world = session.ground_truth();
    let base = world.body_state(world.body_binding(BASE).unwrap()).unwrap();
    assert!((base.pose.translation_m[2] - 0.1).abs() < 0.025, "{base:?}");
    assert!(
        DVec3::from_array(base.velocity_m_s).length() < 0.05,
        "{base:?}"
    );
    let barrel = world
        .body_state(world.body_binding(BARREL).unwrap())
        .unwrap();
    assert!(
        barrel.pose.translation_m[2] > 0.4,
        "unsupported barrel: {barrel:?}"
    );
    assert_anchors(&session);
}
