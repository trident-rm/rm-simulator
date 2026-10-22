// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Inspect a checked robot and step its existing physical model without a match.
use rm_simulator_assets::robots::RobotDescriptor;
use rm_simulator_physics::{
    Pose, StepDuration, Team,
    session::{RobotScenario, RobotSession},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let root = std::env::args_os()
        .nth(1)
        .ok_or("usage: robot_package PACKAGE_DIRECTORY [TRANSFORM_DURATION_S]")?;
    let robot = RobotDescriptor::load(std::path::Path::new(&root))?;
    let mut config = robot.chassis().clone();
    if let Some(duration) = std::env::args().nth(2) {
        config.set_transform_duration_s(duration.parse()?)?;
    }
    println!(
        "catalogue={} manifest={} profile={:?} configuration={:?}\nidentity={} armor={:?}\nvisual={} bytes; original transform limit={} rad",
        RobotDescriptor::VERSION,
        robot.manifest_schema(),
        robot.profile(),
        robot.configuration_origin(),
        robot.identity_sha256(),
        robot.armor(),
        robot.visual().bytes().len(),
        robot.chassis().transform_limit_rad()
    );
    let scenario = RobotScenario::new(vec![], 0., StepDuration::HZ_128, move |physics| {
        physics.add_chassis(Team::Red, config.clone(), Pose::at([0., 0., 0.4]))?;
        Ok(())
    })?;
    let mut session = RobotSession::new(scenario)?;
    session.step(128)?;
    println!(
        "{} ns: {:?} m",
        session.stamp().time_ns,
        session
            .robot_truth(0)
            .ok_or("missing robot")?
            .state
            .pose
            .translation_m
    );
    Ok(())
}
