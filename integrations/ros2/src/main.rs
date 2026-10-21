// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Standalone native ROS 2 publishing probe using a real driven physical chassis.
//! An in-process consumer of the same Field/rolling LiDAR APIs as other hosts.
use anyhow::{Result, ensure};
use clap::Parser;
use rclrs::CreateBasicExecutor;
use rm_simulator_physics::sensors::{
    mid360::Mid360Pattern,
    rolling::{MountedLidar, RangeEffects, RollingFrame},
};
use rm_simulator_ros2::{LivoxConfig, LivoxPacket, publisher::LivoxPublisher};
use rm_simulator_world::{
    ChassisCommand, ChassisConfig, ChassisPlacement, Field, FieldConfig, Pose, RobotKind, Team,
};
use std::time::{Duration, Instant};
#[derive(Parser)]
#[command(
    about = "Publish a driven simulator chassis' rolling LiDAR as Livox Driver 2 CustomMsg. ROS options/remapping are accepted after --ros-args."
)]
struct Args {
    #[arg(long, default_value = "/livox/lidar")]
    topic: String,
    #[arg(long, default_value = "livox_frame")]
    frame_id: String,
    #[arg(long, default_value_t = 0)]
    lidar_id: u8,
    #[arg(long,default_value_t=20,value_parser=clap::value_parser!(u32).range(1..=36000))]
    frames: u32,
    #[arg(long,default_value_t=10,value_parser=clap::value_parser!(u32).range(1..=100))]
    rate_hz: u32,
    #[arg(long,default_value_t=1,value_parser=clap::value_parser!(u32).range(1..=1001))]
    stride: u32,
    #[arg(long, default_value_t = 0.4)]
    mount_z_m: f64,
    #[arg(long, default_value_t = 0.)]
    mount_pitch_deg: f64,
    #[arg(long, default_value_t = 0.02)]
    range_noise_m: f64,
    #[arg(long, default_value_t = 0.01)]
    dropout: f64,
    #[arg(long, default_value_t = 7)]
    seed: u64,
    #[arg(long, default_value_t = 2000)]
    wait_for_subscriber_ms: u64,
    #[arg(long)]
    require_subscriber: bool,
    #[arg(long)]
    fast: bool,
}
fn main() -> Result<()> {
    let args = Args::parse_from(rclrs::extract_non_ros_args(std::env::args())?);
    ensure!(
        1_000_000_000u64.is_multiple_of(u64::from(args.rate_hz)),
        "frame rate must divide one billion nanoseconds exactly"
    );
    let context = rclrs::Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("rm_simulator_livox")?;
    let publisher = LivoxPublisher::new(&node, &args.topic)?;
    let wait_start = Instant::now();
    while publisher.subscription_count()? == 0
        && wait_start.elapsed() < Duration::from_millis(args.wait_for_subscriber_ms)
    {
        std::thread::sleep(Duration::from_millis(10));
    }
    ensure!(
        !args.require_subscriber || publisher.subscription_count()? > 0,
        "DDS subscriber did not connect within startup timeout"
    );
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
    field.step(128)?;
    let epoch = field.time_ns();
    let half = args.mount_pitch_deg.to_radians() / 2.;
    let mount = Pose {
        translation_m: [0., 0., args.mount_z_m],
        rotation_wxyz: [half.cos(), 0., half.sin(), 0.],
    };
    let sensor = MountedLidar::new(
        Mid360Pattern::new(0.1, 40.).map_err(anyhow::Error::msg)?,
        mount,
        epoch,
        RangeEffects {
            seed: args.seed,
            standard_deviation_m: args.range_noise_m,
            dropout_probability: args.dropout,
            bias_m: 0.,
        },
    )
    .map_err(anyhow::Error::msg)?;
    let config = LivoxConfig {
        frame_id: args.frame_id,
        lidar_id: args.lidar_id,
        ..Default::default()
    };
    field.command_chassis(
        0,
        ChassisCommand {
            forward_m_s: 1.,
            yaw_rate_rad_s: 0.5,
            ..Default::default()
        },
    )?;
    let period_ns = 1_000_000_000 / u64::from(args.rate_hz);
    let finish_ns = epoch + period_ns * u64::from(args.frames);
    let mut boundary = epoch + period_ns;
    let mut before = field.lidar_scene_capture(0).map_err(anyhow::Error::msg)?;
    let mut acquisition = RollingFrame::default();
    let mut pending = RollingFrame::default();
    let mut sent = 0;
    let mut returns = 0;
    publisher.publish_clock(epoch)?;
    let wall_start = Instant::now();
    while field.time_ns() < finish_ns {
        field.step(1)?;
        let after = field.lidar_scene_capture(0).map_err(anyhow::Error::msg)?;
        if let Some(window) = sensor
            .window_between(
                before.time_ns(),
                after.time_ns().min(finish_ns),
                args.stride,
            )
            .map_err(anyhow::Error::msg)?
        {
            sensor
                .sample_into(&before, &after, window, &mut acquisition)
                .map_err(anyhow::Error::msg)?;
            for point in &acquisition.points {
                if point.time_ns >= boundary {
                    let packet =
                        LivoxPacket::from_scan(&pending, &config).map_err(anyhow::Error::msg)?;
                    publisher.publish(&packet)?;
                    returns += packet.points.len();
                    sent += 1;
                    pending.points.clear();
                    boundary += period_ns;
                }
                pending.points.push(*point);
            }
        }
        publisher.publish_clock(field.time_ns().min(finish_ns))?;
        before = after;
        if !args.fast {
            let target = Duration::from_nanos(field.time_ns().min(finish_ns) - epoch);
            if let Some(wait) = target.checked_sub(wall_start.elapsed()) {
                std::thread::sleep(wait);
            }
        }
    }
    if !pending.points.is_empty() {
        let packet = LivoxPacket::from_scan(&pending, &config).map_err(anyhow::Error::msg)?;
        publisher.publish(&packet)?;
        returns += packet.points.len();
        sent += 1;
    }
    println!(
        "Published {sent} CustomMsg packets / {returns} measured points on {}; simulation timestamps, no IMU or TF truth injected",
        args.topic
    );
    // Give best-effort DDS time to send the final sample before deleting endpoints.
    std::thread::sleep(Duration::from_millis(100));
    Ok(())
}
