// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Exact same-platform trajectory probe for the six wheel-adapter reference cases.
//! Compare output before/after construction changes; hashes are not portable promises.
//! Pass `--prepared` to exercise reusable prepared installation with identical inputs.
use rm_simulator_physics::{
    Pose, TargetFrames, Team, WorldPhysics,
    chassis::{ChassisCommand, ChassisConfig, PreparedChassis},
    control::WheelTorques,
    tick_ns,
};
use std::{
    collections::hash_map::DefaultHasher,
    hash::{Hash, Hasher},
};

fn main() -> Result<(), &'static str> {
    let use_prepared = std::env::args().any(|arg| arg == "--prepared");
    for case in 0..6 {
        let mut config = match case {
            1 => ChassisConfig::folding_hero(),
            5 => ChassisConfig::balance(),
            _ => ChassisConfig::default(),
        };
        if case == 2 {
            config.drive_friction = 0.02;
        }
        let height = if case == 3 {
            3.
        } else {
            config.rest_height_m()
        };
        let mut world = WorldPhysics::new(&[], 0.);
        let spawn = Pose::at([0., 0., height]);
        let id = if use_prepared {
            let prepared = PreparedChassis::new(config)?;
            world.add_prepared_chassis(Team::Red, &prepared, spawn)?
        } else {
            world.add_chassis(Team::Red, config, spawn)?
        };
        world.command_chassis(
            id,
            ChassisCommand {
                forward_m_s: 2.,
                left_m_s: 0.3,
                yaw_rate_rad_s: 0.2,
                ..Default::default()
            },
        )?;
        if case == 4 {
            world.command_chassis_wheel_torques(id, WheelTorques::new(vec![0.3; 4])?)?;
        }
        let frames = TargetFrames::new(vec![]);
        let mut hash = DefaultHasher::new();
        for tick in 0..128 {
            world.step(tick * tick_ns(), &frames)?;
            // Every observable state, including held commands and wheel contacts,
            // contributes; equal final positions alone would miss intermediate drift.
            format!("{:?}", world.chassis_snapshot(id).unwrap()).hash(&mut hash);
        }
        println!("case {case}: {:016x}", hash.finish());
    }
    Ok(())
}
