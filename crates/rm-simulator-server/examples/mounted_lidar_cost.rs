// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Loaded-CAD mounted rolling LiDAR cost for one moving robot, no rendering.
//! Usage: mounted_lidar_cost CAD_PATH. Includes self-occlusion and range effects.
use rm_simulator_physics::sensors::{
    mid360::Mid360Pattern,
    rolling::{MountedLidar, RangeEffects, RollingFrame},
};
use rm_simulator_server::{cad_assets, layout};
use rm_simulator_world::{
    ChassisCommand, ChassisConfig, ChassisPlacement, Field, Pose, RobotKind, Team,
};
use std::{path::PathBuf, time::Instant};
fn main() -> anyhow::Result<()> {
    let root = PathBuf::from(std::env::args().nth(1).expect("CAD_PATH required"));
    let cad = cad_assets::load(&root)?;
    let terrain = layout::load_terrain(&cad)?;
    let mut field = Field::new(&layout::field_config(
        &cad,
        &layout::LayoutOptions {
            rune: None,
            outpost_speed_rad_s: 0.4,
            terrain: true,
            referee: false,
            projectile_policy: Default::default(),
        },
    ))?;
    layout::add_terrain(&mut field, &terrain)?;

    let robot = field.add_chassis(&ChassisPlacement {
        config: ChassisConfig::default(),
        spawn: Pose::at([0., 0., 0.4]),
        team: Team::Red,
        kind: RobotKind::Infantry,
        performance: None,
    })?;
    field.step(128)?;
    let epoch = field.time_ns();
    let sensor = MountedLidar::new(
        Mid360Pattern::new(0.1, 40.).map_err(anyhow::Error::msg)?,
        Pose::at([0., 0., 0.4]),
        epoch,
        RangeEffects {
            seed: 7,
            standard_deviation_m: 0.02,
            dropout_probability: 0.01,
            ..Default::default()
        },
    )
    .map_err(anyhow::Error::msg)?;
    field.command_chassis(
        robot,
        ChassisCommand {
            forward_m_s: 1.,
            yaw_rate_rad_s: 0.5,
            ..Default::default()
        },
    )?;
    let mut before = field
        .lidar_scene_capture(robot)
        .map_err(anyhow::Error::msg)?;
    let mut frame = RollingFrame::default();
    let (mut points, mut hits) = (0, 0);
    let (mut scan_s, mut capture_s, mut physics_s) = (0., 0., 0.);
    for _ in 0..128 {
        let start = Instant::now();
        field.step(1)?;
        physics_s += start.elapsed().as_secs_f64();
        let start = Instant::now();
        let after = field
            .lidar_scene_capture(robot)
            .map_err(anyhow::Error::msg)?;
        capture_s += start.elapsed().as_secs_f64();
        let window = sensor
            .window_between(before.time_ns(), after.time_ns(), 1)
            .map_err(anyhow::Error::msg)?
            .unwrap();
        let start = Instant::now();
        sensor
            .sample_into(&before, &after, window, &mut frame)
            .map_err(anyhow::Error::msg)?;
        scan_s += start.elapsed().as_secs_f64();
        points += frame.points.len();
        hits += frame.points.iter().filter(|p| p.range_m.is_some()).count();
        before = after;
    }
    println!("{}; CAD root={}", terrain.describe(), root.display());
    println!(
        "One simulated second, one moving robot: {points} points, {hits} returns; capture {:.3} ms, ray acquisition {:.3} ms, physics {:.3} ms; final chassis {:?}",
        capture_s * 1000.,
        scan_s * 1000.,
        physics_s * 1000.,
        field.chassis_snapshot(robot).unwrap().pose.translation_m
    );
    Ok(())
}
