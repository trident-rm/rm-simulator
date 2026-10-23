// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Public prepared chassis reuse, identity and repeatable scenario coverage.
use rm_simulator_physics::{
    Pose, StepDuration, TargetFrames, Team, WorldPhysics,
    chassis::{ChassisCommand, ChassisConfig, PreparedChassis},
    session::{RobotScenario, RobotSession},
    tick_ns,
};

#[test]
fn reuse_keeps_robots_independent_and_removed_ids_stale() -> Result<(), &'static str> {
    let prepared = PreparedChassis::new(ChassisConfig::default())?;
    let spawn = Pose::at([0., 0., prepared.config().rest_height_m()]);
    let mut first = WorldPhysics::new(&[], 0.);
    let mut second = WorldPhysics::new(&[], 0.);
    let a = first.add_prepared_chassis(Team::Red, &prepared, spawn)?;
    let b = first.add_prepared_chassis(
        Team::Blue,
        &prepared.clone(),
        Pose::at([8., 0., spawn.translation_m[2]]),
    )?;
    let other = second.add_prepared_chassis(Team::Red, &prepared, spawn)?;
    let other_before = second.chassis_snapshot(other).unwrap();
    first.command_chassis(
        a,
        ChassisCommand {
            forward_m_s: 2.,
            ..Default::default()
        },
    )?;
    assert_eq!(
        first.chassis_snapshot(b).unwrap().command,
        ChassisCommand::default()
    );
    let frames = TargetFrames::new(vec![]);
    for tick in 0..128 {
        first.step(tick * tick_ns(), &frames)?;
    }
    assert!(first.chassis_snapshot(a).unwrap().pose.translation_m[0] > 0.1);
    assert!(first.chassis_snapshot(b).unwrap().pose.translation_m[0] > 7.9);
    assert_eq!(second.chassis_snapshot(other).unwrap(), other_before);
    first.remove_chassis(a)?;
    let replacement = first.add_prepared_chassis(Team::Red, &prepared, spawn)?;
    assert!(replacement > b && b > a);
    assert!(first.chassis_snapshot(a).is_none());
    assert!(first.command_chassis(a, ChassisCommand::default()).is_err());
    assert!(first.place_chassis(a, spawn).is_err());
    assert!(first.remove_chassis(a).is_err());
    let invalid = Pose::at([f64::NAN, 0., 0.]);
    assert!(
        first
            .add_prepared_chassis(Team::Red, &prepared, invalid)
            .is_err()
    );
    assert_eq!(
        first.add_prepared_chassis(
            Team::Red,
            &prepared,
            Pose::at([-8., 0., spawn.translation_m[2]])
        )?,
        replacement + 1
    );
    Ok(())
}

#[test]
fn scenario_reset_reuses_recipe_and_repeats_the_trajectory() -> Result<(), &'static str> {
    let prepared = PreparedChassis::new(ChassisConfig::default())?;
    let shared = prepared.clone();
    let spawn = Pose::at([0., 0., prepared.config().rest_height_m()]);
    let scenario = RobotScenario::new(vec![], 0., StepDuration::default(), move |world| {
        world.add_prepared_chassis(Team::Red, &shared, spawn)?;
        Ok(())
    })?;
    let mut session = RobotSession::new(scenario)?;
    let initial = session.ground_truth().chassis_snapshot(0).unwrap();
    let command = ChassisCommand {
        forward_m_s: 1.,
        ..Default::default()
    };
    session.physics_mut().command_chassis(0, command)?;
    session.step(128)?;
    let driven = session.ground_truth().chassis_snapshot(0).unwrap();
    session.reset()?;
    assert_eq!(session.stamp().time_ns, 0);
    assert_eq!(session.ground_truth().chassis_snapshot(0).unwrap(), initial);
    session.physics_mut().command_chassis(0, command)?;
    session.step(128)?;
    assert_eq!(session.ground_truth().chassis_snapshot(0).unwrap(), driven);
    assert!(std::ptr::eq(prepared.config(), prepared.clone().config()));
    Ok(())
}
