// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Mounted rolling MID-360 scans from an actually driven physical chassis.
//! Usage: rolling_lidar [OUTPUT.csv]. No renderer, server or external assets.
use rm_simulator_physics::sensors::{
    mid360::Mid360Pattern,
    rolling::{MountedLidar, RangeEffects, RollingFrame},
};
use rm_simulator_world::{
    ChassisCommand, ChassisConfig, ChassisPlacement, Field, FieldConfig, Pose, RobotKind, Team,
};
use std::{
    io::{BufWriter, Write},
    time::Instant,
};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut field = Field::new(&FieldConfig {
        chassis: vec![ChassisPlacement {
            config: ChassisConfig::default(),
            spawn: Pose::at([0., 0., 0.4]),
            team: Team::Red,
            kind: RobotKind::Infantry,
            performance: None,
        }],
        ..Default::default()
    })?;
    field.step(128)?; // Settle before starting the acquisition epoch.
    let epoch = field.time_ns();
    let sensor = MountedLidar::new(
        Mid360Pattern::new(0.1, 40.)?,
        Pose::at([0., 0., 0.4]),
        epoch,
        RangeEffects {
            seed: 7,
            standard_deviation_m: 0.02,
            dropout_probability: 0.01,
            ..Default::default()
        },
    )?;
    field.command_chassis(
        0,
        ChassisCommand {
            forward_m_s: 1.,
            yaw_rate_rad_s: 0.5,
            ..Default::default()
        },
    )?;
    let mut file = std::env::args()
        .nth(1)
        .map(std::fs::File::create)
        .transpose()?
        .map(BufWriter::new);
    if let Some(f) = &mut file {
        writeln!(
            f,
            "sample_index,time_ns,range_m,sensor_x_m,sensor_y_m,sensor_z_m"
        )?;
    }
    let mut before = field.lidar_scene_capture(0)?;
    let mut frame = RollingFrame::default();
    let mut total = 0;
    let mut hits = 0;
    let mut scan_s = 0.;
    for _ in 0..128 {
        field.step(1)?;
        let after = field.lidar_scene_capture(0)?;
        if let Some(window) = sensor.window_between(before.time_ns(), after.time_ns(), 1)? {
            let start = Instant::now();
            sensor.sample_into(&before, &after, window, &mut frame)?;
            scan_s += start.elapsed().as_secs_f64();
            for p in &frame.points {
                total += 1;
                if let (Some(range), Some([x, y, z])) = (p.range_m, p.point_sensor_m) {
                    hits += 1;
                    if let Some(f) = &mut file {
                        writeln!(f, "{},{},{range},{x},{y},{z}", p.sample_index, p.time_ns)?;
                    }
                } else if let Some(f) = &mut file {
                    writeln!(f, "{},{},,,,", p.sample_index, p.time_ns)?;
                }
            }
        }
        before = after;
    }
    println!(
        "One simulated second: {total} timed points, {hits} returns, {:.3} ms ray acquisition; final chassis {:?}",
        scan_s * 1000.,
        field.chassis_snapshot(0).unwrap().pose.translation_m
    );
    Ok(())
}
