// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Bare-session integration coverage: no field, rules, transport or host clock.
use rm_simulator_physics::{
    ArmorTarget, Caliber, Pose, Shot, StepDuration, TargetFace, Team,
    chassis::{ChassisCommand, ChassisConfig},
    control::{DriveControl, WheelTorques},
    motion::{Mechanism, MechanismState},
    projectile::{PayloadShape, PayloadSnapshot, ProjectilePolicy},
    session::{CommandResult, RobotCommand, RobotScenario, RobotSession, SampleSchedule},
};

fn scenario(duration: StepDuration) -> RobotScenario {
    RobotScenario::new(vec![], 0., duration, |physics| {
        physics.add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 0., 0.3]))?;
        Ok(())
    })
    .unwrap()
}

#[test]
fn custom_rate_drives_a_real_robot_and_partitioning_preserves_state() {
    let duration = StepDuration::from_hz(1250).unwrap();
    let mut whole = RobotSession::new(scenario(duration)).unwrap();
    let mut split = RobotSession::new(scenario(duration)).unwrap();
    for session in [&mut whole, &mut split] {
        session
            .command(RobotCommand::Drive {
                robot: 0,
                command: ChassisCommand {
                    forward_m_s: 1.,
                    ..Default::default()
                },
            })
            .unwrap();
    }
    whole.step(1250).unwrap();
    split.step(600).unwrap();
    split.step(650).unwrap();
    assert_eq!(whole.stamp().tick, 1250);
    assert_eq!(whole.stamp().time_ns, 1_000_000_000);
    assert_eq!(whole.step_duration(), duration);
    let truth = whole.robot_truth(0).unwrap();
    assert!(
        truth.state.pose.translation_m[0] > 0.4,
        "{:?}",
        truth.state.pose
    );
    assert_eq!(truth, split.robot_truth(0).unwrap());
    assert!(whole.robot_truth(999).is_none());
    let compact = whole.robot_control_state(0).unwrap();
    assert_eq!(compact.pose, truth.state.pose);
    assert_eq!(compact.command, truth.state.command);
    assert!(whole.robot_control_state(999).is_none());
    assert!(whole.lidar_scene_capture(0).is_ok());
    assert!(whole.lidar_scene_capture(999).is_err());
}

#[test]
fn held_torque_controls_fire_and_per_tick_controllers_use_session_time() {
    let mut session = RobotSession::new(scenario(StepDuration::HZ_1000)).unwrap();
    let torques = WheelTorques::new(vec![0.1; 4]).unwrap();
    assert_eq!(
        session.command(RobotCommand::WheelTorques {
            robot: 0,
            torques: torques.clone()
        }),
        Ok(CommandResult::Driven)
    );
    assert!(
        session
            .command(RobotCommand::WheelTorques {
                robot: 0,
                torques: WheelTorques::new(vec![1.; 3]).unwrap()
            })
            .is_err()
    );
    let mut calls = 0;
    session
        .step_controlled(100, |session, _| {
            assert_eq!(session.stamp().time_ns, calls * 1_000_000);
            calls += 1;
        })
        .unwrap();
    assert_eq!(calls, 100);
    let truth = session.robot_truth(0).unwrap();
    assert_eq!(
        truth.state.drive_control,
        DriveControl::WheelTorques(torques)
    );
    let before = session.stamp();
    let muzzle = session.ground_truth().chassis_muzzle_pose(0).unwrap();
    assert_eq!(
        session.command(RobotCommand::Fire {
            robot: 0,
            shot: Shot::at_limit(Caliber::Mm17)
        }),
        Ok(CommandResult::Fired(0))
    );
    let ball = &session.ground_truth().snapshot()[0];
    assert_eq!(ball.position_m, muzzle.translation_m);
    assert_eq!(ball.launched_ns, before.time_ns);
    assert_eq!(ball.shooter, Some(0));
    assert_eq!(session.stamp(), before);
    assert!(
        session
            .command(RobotCommand::Fire {
                robot: 99,
                shot: Shot::at_limit(Caliber::Mm17)
            })
            .is_err()
    );
    session.physics_mut().set_chassis_defeated(0, true);
    assert!(
        session
            .command(RobotCommand::Fire {
                robot: 0,
                shot: Shot::at_limit(Caliber::Mm17)
            })
            .is_err()
    );
    session.physics_mut().set_chassis_defeated(0, false);
    session
        .command(RobotCommand::Drive {
            robot: 0,
            command: Default::default(),
        })
        .unwrap();
    assert_eq!(
        session.robot_truth(0).unwrap().state.drive_control,
        DriveControl::BodyVelocity
    );
    session.reset().unwrap();
    assert_eq!(session.stamp().time_ns, 0);
    assert_eq!(session.ground_truth().launched(), 0);
    assert_eq!(
        session.robot_truth(0).unwrap().state.command,
        ChassisCommand::default()
    );
}

fn payload() -> PayloadSnapshot {
    PayloadSnapshot {
        id: 7,
        shape: PayloadShape {
            hulls_m: vec![vec![
                [-0.05, -0.05, -0.05],
                [0.05, -0.05, -0.05],
                [-0.05, 0.05, -0.05],
                [-0.05, -0.05, 0.05],
                [0.05, 0.05, 0.05],
            ]],
            mass_kg: 0.2,
            max_expansion_m: 0.,
        },
        pose: Pose::at([3., 0., 1.]),
        velocity_m_s: [0.; 3],
        angular_velocity_rad_s: [0.; 3],
        prescribed: false,
    }
}

#[test]
fn reset_rebuilds_initial_geometry_payload_policy_and_trace_without_a_checkpoint() {
    let scenario = RobotScenario::new(vec![], 0., StepDuration::HZ_1000, |physics| {
        physics.add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 0., 0.3]))?;
        physics.add_static_box(Pose::at([5., 0., 0.5]), [1., 1., 1.]);
        physics.projectile_bounds_m = Some([[-10.; 3], [10.; 3]]);
        physics.set_projectile_policy(ProjectilePolicy::default().without_retirement())?;
        physics.add_payload(&payload())?;
        physics.add_mechanism_mesh(
            vec![[7., 0., 0.], [8., 0., 0.], [7., 1., 0.]],
            vec![[0, 1, 2]],
            Mechanism::Base,
            Team::Red,
            [[0., 0., 0.], [0., 0., 1.]],
        )?;
        physics.sync_mechanisms(&MechanismState {
            base_open_fraction: [1., 0.],
            ..Default::default()
        });
        physics.set_carrier_trace(true);
        Ok(())
    })
    .unwrap();
    let mut session = RobotSession::new(scenario).unwrap();
    let initial_geometry = session.ground_truth().static_geometry_snapshot(true);
    let initial_payload = session.ground_truth().payload(7).unwrap();
    session.step(100).unwrap();
    assert!(
        session
            .ground_truth()
            .payload(7)
            .unwrap()
            .pose
            .translation_m[2]
            < 0.99
    );
    let mut trace = Vec::new();
    session.physics_mut().drain_carrier_trace(&mut trace);
    assert!(!trace.is_empty());
    session
        .physics_mut()
        .add_static_box(Pose::at([6., 0., 0.5]), [1.; 3]);
    session
        .physics_mut()
        .sync_mechanisms(&MechanismState::default());
    session.reset().unwrap();
    assert!(session.ground_truth().has_mechanisms());
    assert_eq!(session.ground_truth().payload(7).unwrap(), initial_payload);
    assert_eq!(
        session
            .ground_truth()
            .static_geometry_snapshot(true)
            .into_triangles(),
        initial_geometry.into_triangles()
    );
    assert_eq!(
        session.ground_truth().projectile_bounds_m,
        Some([[-10.; 3], [10.; 3]])
    );
    assert_eq!(
        session.ground_truth().projectile_policy().retire_speed_m_s,
        None
    );
    let mut empty = Vec::new();
    session.physics_mut().drain_carrier_trace(&mut empty);
    assert!(empty.is_empty());
    session.step(1).unwrap();
    session.physics_mut().drain_carrier_trace(&mut empty);
    assert!(!empty.is_empty());
}

#[test]
fn failed_reset_and_invalid_target_motion_leave_live_state_intact() {
    use std::sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    };
    let reject = Arc::new(AtomicBool::new(false));
    let recipe_reject = reject.clone();
    let face = TargetFace {
        target: ArmorTarget::Base { base: 0, plate: 0 },
        pose: Pose::at([2., 0., 1.]),
    };
    let scenario = RobotScenario::new(vec![face], 0., StepDuration::HZ_1000, move |physics| {
        physics.add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 0., 0.3]))?;
        if recipe_reject.load(Ordering::Relaxed) {
            return Err("scenario unavailable");
        }
        Ok(())
    })
    .unwrap();
    let mut session = RobotSession::new(scenario).unwrap();
    session.step(10).unwrap();
    let before = session.robot_truth(0).unwrap();
    reject.store(true, Ordering::Relaxed);
    assert_eq!(session.reset(), Err("scenario unavailable"));
    assert_eq!(session.robot_truth(0).unwrap(), before);
    assert!(session.step_with_targets(&[]).is_err());
    let invalid = TargetFace {
        pose: Pose::at([f64::NAN, 0., 0.]),
        ..face
    };
    assert!(session.step_with_targets(&[invalid]).is_err());
    assert_eq!(session.robot_truth(0).unwrap(), before);
    assert!(session.step(u64::MAX).is_err());
    assert_eq!(session.robot_truth(0).unwrap(), before);
    session
        .step_with_targets(&[TargetFace {
            pose: Pose::at([2., 0.1, 1.]),
            ..face
        }])
        .unwrap();
    assert_eq!(session.stamp().tick, 11);
}

#[test]
fn projectile_contacts_are_returned_in_tick_order_from_the_same_world() {
    let face = TargetFace {
        target: ArmorTarget::Outpost {
            outpost: 0,
            face: 0,
        },
        pose: Pose::yawed([1.5, 0., 1.], std::f64::consts::PI),
    };
    let scenario = RobotScenario::new(vec![face], 0., StepDuration::HZ_128, |physics| {
        physics.fire(
            0,
            Pose::at([0., 0., 1.]),
            Shot::at_limit(Caliber::Mm17),
            None,
        )?;
        Ok(())
    })
    .unwrap();
    let mut session = RobotSession::new(scenario).unwrap();
    let contacts = session.step(200).unwrap();
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].target, face.target);
    assert_eq!(contacts[0].projectile, 0);
    assert!(contacts[0].normal_speed_m_s > 20.);
    session.reset().unwrap();
    assert_eq!(session.step(200).unwrap(), contacts);
}

#[test]
fn sample_schedule_keeps_phase_and_exhausts_on_overflow() {
    assert!(SampleSchedule::new(0, 0).is_err());
    let mut schedule = SampleSchedule::new(3, 2).unwrap();
    assert_eq!(
        (0..3).map(|_| schedule.pop_due(10)).collect::<Vec<_>>(),
        vec![Some(2), Some(5), Some(8)]
    );
    assert_eq!(schedule.pop_due(10), None);
    let mut final_sample = SampleSchedule::new(2, u64::MAX).unwrap();
    assert_eq!(final_sample.pop_due(u64::MAX), Some(u64::MAX));
    assert_eq!(final_sample.pop_due(u64::MAX), None);
}

#[test]
fn invalid_scenarios_fail_before_setup() {
    assert!(
        RobotScenario::new(vec![], f64::NAN, StepDuration::HZ_128, |_| {
            panic!("invalid scenario must not run setup");
        })
        .is_err()
    );
    let face = TargetFace {
        target: ArmorTarget::Outpost {
            outpost: 0,
            face: 0,
        },
        pose: Pose::at([1., 0., 1.]),
    };
    assert!(RobotScenario::new(vec![face, face], 0., StepDuration::HZ_128, |_| Ok(())).is_err());
}

#[test]
fn controller_command_storage_survives_repeated_calls_and_errors() {
    let mut session = RobotSession::new(scenario(StepDuration::HZ_1000)).unwrap();
    let mut storage = None;
    for index in 0..16 {
        let result = session.step_controlled(1, |_, commands| {
            commands.push(RobotCommand::Drive {
                robot: if index == 7 { 999 } else { 0 },
                command: ChassisCommand::default(),
            });
            let current = (commands.as_ptr(), commands.capacity());
            if let Some(previous) = storage {
                assert_eq!(previous, current);
            }
            storage = Some(current);
        });
        assert_eq!(result.is_err(), index == 7);
    }
    assert_eq!(session.stamp().tick, 15);
}
