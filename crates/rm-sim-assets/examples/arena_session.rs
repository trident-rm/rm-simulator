// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Load a complete checked arena and step a robot without a host or renderer.
use rm_sim_assets::{arena::PreparedArena, cad_assets};
use rm_sim_physics::{
    Pose, StepDuration, Team,
    chassis::ChassisConfig,
    session::{RobotCommand, RobotSession},
};
fn main() -> anyhow::Result<()> {
    let path = std::env::args_os()
        .nth(1)
        .ok_or_else(|| anyhow::anyhow!("usage: arena_session FIELD_PACKAGE"))?;
    let cad = cad_assets::load(std::path::Path::new(&path))?;
    let arena = PreparedArena::load(&cad)?;
    // Reusable engine queries are independent of the arena package and match rules.
    let rays = [
        rm_sim_physics::queries::RayQuery::new([9., 0., 2.], [0., 0., -1.], 5.)
            .map_err(anyhow::Error::msg)?,
    ];
    let mut hits = [None];
    arena
        .geometry()
        .cast_rays_into(&rays, &mut hits)
        .map_err(anyhow::Error::msg)?;
    println!(
        "example ground query: {:?}",
        hits[0].map(|hit| hit.distance_m)
    );
    let config = ChassisConfig::default();
    let floor = arena
        .terrain()
        .ground
        .ground_height_below(9., 0., 2.)
        .ok_or_else(|| anyhow::anyhow!("no ground under explicit example spawn"))?;
    let spawn = Pose::yawed(
        [9., 0., floor + config.rest_height_m()],
        std::f64::consts::PI,
    );
    let recipe = arena
        .scenario(vec![], StepDuration::HZ_128, move |physics| {
            // Explicit example posture: closed shields, open dart doors, zero rotors.
            physics.sync_mechanisms(&Default::default());
            physics.add_chassis(Team::Red, config.clone(), spawn)?;
            Ok(())
        })
        .map_err(anyhow::Error::msg)?;
    let mut session = RobotSession::new(recipe).map_err(anyhow::Error::msg)?;
    session
        .command(RobotCommand::Drive {
            robot: 0,
            command: Default::default(),
        })
        .map_err(anyhow::Error::msg)?;
    session.step(128).map_err(anyhow::Error::msg)?;
    println!("{}", arena.terrain().describe());
    println!("robot: {:?}", session.robot_truth(0));
    session.reset().map_err(anyhow::Error::msg)?;
    assert_eq!(session.stamp().tick, 0);
    Ok(())
}
