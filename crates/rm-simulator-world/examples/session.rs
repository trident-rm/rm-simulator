// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-paced 1000 Hz simulation, without a server, renderer or external assets.
use rm_simulator_world::{
    ChassisCommand, ChassisConfig, ChassisPlacement, Field, FieldConfig, Pose, RobotKind,
    StepDuration, Team,
    session::{MatchSession, RobotCommand, SampleSchedule},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let field = Field::new(&FieldConfig {
        step_duration: StepDuration::HZ_1000,
        chassis: vec![ChassisPlacement {
            config: ChassisConfig::default(),
            spawn: Pose::at([0., 0., 0.4]),
            team: Team::Red,
            kind: RobotKind::Infantry,
            performance: None,
        }],
        ..Default::default()
    })?;
    let mut session = MatchSession::new(field);
    let mut samples = SampleSchedule::new(10_000_000, 0)?;
    let mut observations = 0;
    session.step_controlled(1000, |field, commands| {
        while samples.pop_due(field.time_ns()).is_some() {
            // Future sensor adapters sample here. This example reads no fake sensor.
            observations += 1;
        }
        commands.push(RobotCommand::Drive {
            robot: 0,
            command: ChassisCommand {
                forward_m_s: 0.5,
                ..Default::default()
            },
        });
    })?;
    let robot = session.robot_truth(0).expect("configured robot");
    println!(
        "{} steps, {} ns, {} sampling timestamps; robot position {:?}",
        robot.stamp.tick, robot.stamp.time_ns, observations, robot.state.pose.translation_m
    );
    let checkpoint = session.checkpoint();
    session.step(10)?;
    session.restore(&checkpoint)?;
    assert_eq!(session.stamp().time_ns, 1_000_000_000);
    session.reset(42)?;
    assert_eq!(session.stamp().time_ns, 0);
    Ok(())
}
