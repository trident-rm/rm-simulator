// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-paced bare robot motion and scenario reset without a match or host.
use rm_sim_physics::{
    Pose, StepDuration, Team,
    chassis::{ChassisCommand, ChassisConfig},
    session::{RobotCommand, RobotScenario, RobotSession},
};

fn main() -> Result<(), &'static str> {
    let scenario = RobotScenario::new(vec![], 0., StepDuration::HZ_1000, |physics| {
        physics.add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 0., 0.3]))?;
        Ok(())
    })?;
    let mut session = RobotSession::new(scenario)?;
    session.command(RobotCommand::Drive {
        robot: 0,
        command: ChassisCommand {
            forward_m_s: 1.,
            ..Default::default()
        },
    })?;
    session.step(1000)?;
    let truth = session.robot_truth(0).ok_or("robot disappeared")?;
    println!(
        "{} ns: {:?} m",
        truth.stamp.time_ns, truth.state.pose.translation_m
    );
    assert!(truth.state.pose.translation_m[0] > 0.4);
    session.reset()?;
    assert_eq!(session.stamp().time_ns, 0);
    Ok(())
}
