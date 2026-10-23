// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Existing chassis regression tests, moved without changing their assertions.

use super::*;
use crate::{TargetFrames, WorldPhysics, tick_ns};

#[test]
fn closed_linkage_keeps_lengths_and_level_travel() {
    let config = ChassisConfig::stair_heavy();
    config.validate().unwrap();
    let c = config.climbing.as_ref().unwrap();
    for i in 0..=128 {
        let p = c.poses(c.travel_rad() * i as f64 / 128.);
        let distance = |a: Pose, b: Pose| {
            (Vector::from_array(a.translation_m) - Vector::from_array(b.translation_m)).length()
        };
        assert!((distance(p.front, p.coupler) - c.front_pin_length_m).abs() < 1e-10);
        assert!((distance(p.coupler, p.rear) - c.coupler_length_m).abs() < 1e-10);
        assert!((p.rear.translation_m[2] - c.slider_height_m).abs() < 1e-10);
        for hub in p.hubs_m {
            assert!(hub.into_iter().all(f64::is_finite));
        }
    }
    let p = c.poses(0.);
    assert!((p.hubs_m[0][2] - p.hubs_m[2][2]).abs() < 1e-10);
    let mut invalid = c.clone();
    invalid.coupler_length_m = 0.001;
    assert!(invalid.validate().is_err());
    let mut invalid = config.clone();
    invalid.folding = ChassisConfig::folding_hero().folding;
    assert!(invalid.validate().is_err());
}

#[test]
fn moving_hubs_and_actuator_restore_mid_climb() {
    let cfg = ChassisConfig::stair_heavy();
    let mut host = WorldPhysics::new(&[], 0.);
    let id = host
        .add_chassis(Team::Red, cfg, Pose::at([0., 0., 3.]))
        .unwrap();
    host.command_chassis(
        id,
        ChassisCommand {
            folded: true,
            ..Default::default()
        },
    )
    .unwrap();
    let frames = TargetFrames::new(vec![]);
    for tick in 0..40 {
        host.step(tick * tick_ns(), &frames).unwrap();
    }
    let saved = host.chassis_snapshot(id).unwrap();
    assert!(
        saved.fold_angle_rad > 0.1 && saved.fold_angle_rad < saved.config.transform_limit_rad()
    );
    let mut restored = WorldPhysics::new(&[], 0.);
    restored.restore_chassis(&saved).unwrap();
    let copy = restored.chassis_snapshot(id).unwrap();
    assert_eq!(copy.fold_angle_rad, saved.fold_angle_rad);
    for (a, b) in saved.wheels.iter().zip(&copy.wheels) {
        for (x, y) in a.hub_m.iter().zip(b.hub_m) {
            assert!((x - y).abs() < 1e-9);
        }
    }
    for tick in 40..70 {
        host.step(tick * tick_ns(), &frames).unwrap();
        restored.step(tick * tick_ns(), &frames).unwrap();
    }
    let a = host.chassis_snapshot(id).unwrap();
    let b = restored.chassis_snapshot(id).unwrap();
    assert_eq!(a.fold_angle_rad, b.fold_angle_rad);
    for (x, y) in a.pose.translation_m.into_iter().zip(b.pose.translation_m) {
        assert!((x - y).abs() < 1e-8);
    }
}
#[test]
fn defeated_stair_actuator_stops_at_its_current_input() {
    let mut w = WorldPhysics::new(&[], 0.);
    let id = w
        .add_chassis(
            Team::Red,
            ChassisConfig::stair_heavy(),
            Pose::at([0., 0., 3.]),
        )
        .unwrap();
    w.command_chassis(
        id,
        ChassisCommand {
            folded: true,
            ..Default::default()
        },
    )
    .unwrap();
    let frames = TargetFrames::new(vec![]);
    for tick in 0..40 {
        w.step(tick * tick_ns(), &frames).unwrap();
    }
    let angle = w.chassis_snapshot(id).unwrap().fold_angle_rad;
    w.set_chassis_defeated(id, true);
    for tick in 40..80 {
        w.step(tick * tick_ns(), &frames).unwrap();
    }
    assert_eq!(w.chassis_snapshot(id).unwrap().fold_angle_rad, angle);
}
