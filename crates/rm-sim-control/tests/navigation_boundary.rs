// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Public controller boundary: invalid/stale work, explicit time and bounded observations.
use rm_sim_control::navigation::{
    Controller, Map, Obstacle, Phase, RobotObservation, Route, Surface,
};
use rm_sim_physics::{
    Pose, StaticGeometry,
    chassis::{ChassisCommand, ChassisConfig},
};
fn surface() -> Surface {
    Surface {
        ground: None,
        floor: 0.,
        exclusions: vec![],
        passage_hints: vec![],
    }
}
fn following(tunnel: bool) -> Controller {
    let mut c = Controller::new(surface()).unwrap();
    c.add(7);
    c.goal(7, [2., 0.]).unwrap();
    let generation = c.request(7).unwrap().generation;
    c.accept_route(
        7,
        generation,
        Ok(Route {
            points: vec![[0.; 3], [2., 0., 0.]],
            tunnels: vec![false, tunnel],
        }),
        Some([0.; 3]),
    )
    .unwrap();
    c
}
#[test]
fn stale_and_malformed_routes_do_not_replace_the_current_request() {
    let mut c = Controller::new(surface()).unwrap();
    c.add(7);
    c.goal(7, [2., 0.]).unwrap();
    let old = c.request(7).unwrap().generation;
    c.goal(7, [3., 1.]).unwrap();
    let before = c.views();
    assert!(
        !c.accept_route(
            7,
            old,
            Ok(Route {
                points: vec![],
                tunnels: vec![]
            }),
            None
        )
        .unwrap()
    );
    assert_eq!(c.views(), before);
    let current = c.request(7).unwrap().generation;
    assert!(
        c.accept_route(
            7,
            current,
            Ok(Route {
                points: vec![[0.; 3], [1.; 3]],
                tunnels: vec![false]
            }),
            Some([0.; 3])
        )
        .is_err()
    );
    assert_eq!(c.views(), before);
    assert_eq!(c.request(7).unwrap().generation, current);
    c.stop(7);
    assert!(
        !c.accept_route(7, current, Err("late result".into()), None)
            .unwrap()
    );
    assert_eq!(c.view(7).unwrap().phase, Phase::Idle);
}
#[test]
fn preparation_timeout_uses_elapsed_nanoseconds_instead_of_controller_call_count() {
    let config = ChassisConfig::folding_hero();
    let geometry = StaticGeometry::default();
    let body = RobotObservation {
        pose: Pose::at([0., 0., config.rest_height_m()]),
        command: ChassisCommand::default(),
        fold_angle_rad: 0.,
        defeated: false,
        config: &config,
    };
    for dt in [62_500_000, 1_000_000] {
        let mut c = following(true);
        for _ in 0..10_000_000_000_u64 / dt {
            assert!(
                c.command(7, body, &geometry, &[], dt)
                    .unwrap()
                    .unwrap()
                    .folded
            );
        }
        assert_eq!(c.view(7).unwrap().phase, Phase::Following);
        let stopped = c.command(7, body, &geometry, &[], dt).unwrap().unwrap();
        assert_eq!(c.view(7).unwrap().phase, Phase::Blocked);
        assert_eq!(
            [
                stopped.forward_m_s,
                stopped.left_m_s,
                stopped.yaw_rate_rad_s
            ],
            [0.; 3]
        );
    }
}
#[test]
fn invalid_observations_fail_before_changing_controller_state() {
    let config = ChassisConfig::default();
    let geometry = StaticGeometry::default();
    let body = RobotObservation {
        pose: Pose::default(),
        command: ChassisCommand::default(),
        fold_angle_rad: 0.,
        defeated: false,
        config: &config,
    };
    let mut c = following(false);
    let before = c.views();
    assert!(c.command(7, body, &geometry, &[], 0).is_err());
    let bad = RobotObservation {
        fold_angle_rad: f64::NAN,
        ..body
    };
    assert!(c.command(7, bad, &geometry, &[], 1_000_000).is_err());
    assert!(
        c.command(
            7,
            body,
            &geometry,
            &[Obstacle {
                point: [0.; 3],
                radius: -1.
            }],
            1_000_000
        )
        .is_err()
    );
    assert_eq!(c.views(), before);
    assert!(c.goal(7, [f64::INFINITY, 0.]).is_err());
    assert_eq!(c.views(), before);
}
#[test]
fn search_admission_is_bounded_and_failure_messages_remain_valid_utf8() {
    assert!(Map::prepare(StaticGeometry::default(), surface(), [0., 0., 1e9, 1e9]).is_err());
    let map = Map::prepare(StaticGeometry::default(), surface(), [-1., -1., 1., 1.]).unwrap();
    assert!(
        map.plan(
            [0.; 3],
            [f64::NAN, 0.],
            &[],
            &StaticGeometry::default(),
            false
        )
        .is_err()
    );
    let mut c = Controller::new(surface()).unwrap();
    c.add(7);
    c.goal(7, [1., 0.]).unwrap();
    let generation = c.request(7).unwrap().generation;
    assert!(!c.submission_failed(7, generation, Phase::Following, "not a failure"));
    assert!(c.submission_failed(7, generation, Phase::Blocked, &"哨兵".repeat(100)));
    assert!(c.view(7).unwrap().valid());
    assert!(c.view(7).unwrap().message.len() <= 160);
}
