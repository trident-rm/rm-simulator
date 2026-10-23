// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Existing chassis regression tests, moved without changing their assertions.

use super::*;
use crate::{TargetFrames, WorldPhysics, tick_ns};
fn advance(w: &mut WorldPhysics, tick: &mut u64, count: u64) {
    for _ in 0..count {
        w.step(*tick * tick_ns(), &TargetFrames::new(vec![]))
            .unwrap();
        *tick += 1;
    }
}
#[test]
fn half_second_strokes_reverse_and_resume_without_teleporting() {
    for mut config in [ChassisConfig::folding_hero(), ChassisConfig::stair_heavy()] {
        config.set_transform_duration_s(0.5).unwrap();
        let limit = config.transform_limit_rad();
        let mut world = WorldPhysics::new(&[], 0.);
        let id = world
            .add_chassis(Team::Red, config, Pose::at([0., 0., 20.]))
            .unwrap();
        let mut tick = 0;
        world
            .command_chassis(
                id,
                ChassisCommand {
                    folded: true,
                    ..Default::default()
                },
            )
            .unwrap();
        advance(&mut world, &mut tick, 32);
        assert!((world.chassis_snapshot(id).unwrap().fold_angle_rad - limit / 2.).abs() < 1e-12);
        world
            .command_chassis(id, ChassisCommand::default())
            .unwrap();
        advance(&mut world, &mut tick, 32);
        assert!(world.chassis_snapshot(id).unwrap().fold_angle_rad.abs() < 1e-12);
        world
            .command_chassis(
                id,
                ChassisCommand {
                    folded: true,
                    ..Default::default()
                },
            )
            .unwrap();
        advance(&mut world, &mut tick, 63);
        assert!(world.chassis_snapshot(id).unwrap().fold_angle_rad < limit - 0.001);
        advance(&mut world, &mut tick, 1);
        assert!((world.chassis_snapshot(id).unwrap().fold_angle_rad - limit).abs() < 1e-12);
        world
            .command_chassis(id, ChassisCommand::default())
            .unwrap();
        advance(&mut world, &mut tick, 64);
        assert!(world.chassis_snapshot(id).unwrap().fold_angle_rad.abs() < 1e-12);
    }
    let mut config = ChassisConfig::folding_hero();
    let before = config.clone();
    for invalid in [0., -1., f64::NAN, f64::INFINITY, f64::MIN_POSITIVE / 16.] {
        assert!(config.set_transform_duration_s(invalid).is_err());
        assert_eq!(config, before);
    }
}
fn setup(height: f64) -> (WorldPhysics, u32) {
    let mut w = WorldPhysics::new(&[], 0.);
    let id = w
        .add_chassis(
            Team::Red,
            ChassisConfig::folding_hero(),
            Pose::at([0., 0., height]),
        )
        .unwrap();
    (w, id)
}
fn height(w: &WorldPhysics) -> f64 {
    w.dynamic_geometry_snapshot()
        .shapes
        .iter()
        .map(|(p, s)| s.compute_aabb(p).maxs.z)
        .fold(f64::NEG_INFINITY, f64::max)
}
#[test]
fn folding_changes_physical_envelope_and_stops_unfolding_at_a_roof() {
    let (mut w, id) = setup(0.17);
    let mut tick = 0;
    advance(&mut w, &mut tick, 512);
    let before = height(&w);
    assert!(before > 0.40);
    w.command_chassis(
        id,
        ChassisCommand {
            folded: true,
            ..Default::default()
        },
    )
    .unwrap();
    advance(&mut w, &mut tick, 256);
    let folded = w.chassis_snapshot(id).unwrap();
    assert!(
        (folded.fold_angle_rad - folded.config.folding.as_ref().unwrap().folded_angle_rad).abs()
            < 1e-12
    );
    assert!(height(&w) < 0.25, "{}", height(&w));
    let muzzle = w.chassis_muzzle_pose(id).unwrap();
    assert!((muzzle.translation_m[2] - folded.turret.translation_m[2]).abs() < 0.001);
    w.add_static_box(Pose::at([0., 0., 0.5]), [2., 2., 0.5]);
    w.command_chassis(id, ChassisCommand::default()).unwrap();
    advance(&mut w, &mut tick, 256);
    let blocked = w.chassis_snapshot(id).unwrap();
    assert!(
        blocked.fold_angle_rad > 1. && blocked.fold_angle_rad < folded.fold_angle_rad,
        "blocked={} folded={} height={} pos={:?}",
        blocked.fold_angle_rad,
        folded.fold_angle_rad,
        height(&w),
        blocked.pose
    );
    assert!(height(&w) < 0.251, "{}", height(&w));
    w.command_chassis(
        id,
        ChassisCommand {
            folded: true,
            ..Default::default()
        },
    )
    .unwrap();
    advance(&mut w, &mut tick, 256);
    assert!((w.chassis_snapshot(id).unwrap().fold_angle_rad - folded.fold_angle_rad).abs() < 1e-12);
}
#[test]
fn mid_transform_restores_and_replays_at_the_same_rate() {
    let (mut host, id) = setup(3.);
    let mut tick = 0;
    host.command_chassis(
        id,
        ChassisCommand {
            folded: true,
            ..Default::default()
        },
    )
    .unwrap();
    advance(&mut host, &mut tick, 40);
    let saved = host.chassis_snapshot(id).unwrap();
    assert!(
        (saved.fold_angle_rad - saved.config.folding.as_ref().unwrap().speed_rad_s * 40. / 128.)
            .abs()
            < 1e-12,
        "angle={} command={:?}",
        saved.fold_angle_rad,
        saved.command
    );
    let mut restored = WorldPhysics::new(&[], 0.);
    restored.restore_chassis(&saved).unwrap();
    assert_eq!(
        restored.chassis_snapshot(id).unwrap().fold_angle_rad,
        saved.fold_angle_rad
    );
    let mut replay_tick = tick;
    advance(&mut host, &mut tick, 30);
    advance(&mut restored, &mut replay_tick, 30);
    assert_eq!(
        host.chassis_snapshot(id).unwrap().fold_angle_rad,
        restored.chassis_snapshot(id).unwrap().fold_angle_rad
    );
    let a = host.chassis_muzzle_pose(id).unwrap();
    let b = restored.chassis_muzzle_pose(id).unwrap();
    for i in 0..3 {
        assert!((a.translation_m[i] - b.translation_m[i]).abs() < 1e-8);
    }
}
