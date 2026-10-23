// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Bounded operator-HTTP measured stream -> rclrs, with no local physical world.
use anyhow::{Result, ensure};
use rclrs::CreateBasicExecutor;
use rm_sim_physics::sensors::synchronized::{Config, Frame};
use rm_sim_ros2::{
    LivoxConfig, LivoxPacket,
    publisher::{ImuPublisher, LivoxPublisher},
};
use serde::Deserialize;
use std::{
    io::{Read, Write},
    net::{SocketAddr, TcpStream},
    time::{Duration, Instant},
};
#[derive(Deserialize)]
struct Poll {
    time_ns: u64,
    frames: Vec<Frame>,
    dropped_intervals: u64,
}
fn post(
    address: SocketAddr,
    path: &str,
    body: &impl serde::Serialize,
) -> Result<serde_json::Value> {
    let body = serde_json::to_vec(body)?;
    let mut socket = TcpStream::connect_timeout(&address, Duration::from_secs(1))?;
    socket.set_read_timeout(Some(Duration::from_secs(2)))?;
    socket.set_write_timeout(Some(Duration::from_secs(2)))?;
    write!(
        socket,
        "POST {path} HTTP/1.1\r\nHost: {address}\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        body.len()
    )?;
    socket.write_all(&body)?;
    let mut bytes = Vec::new();
    socket.take(32 * 1024 * 1024 + 1).read_to_end(&mut bytes)?;
    ensure!(
        bytes.len() <= 32 * 1024 * 1024,
        "sensor HTTP response exceeds 32 MiB"
    );
    let split = bytes
        .windows(4)
        .position(|b| b == b"\r\n\r\n")
        .ok_or_else(|| anyhow::anyhow!("missing HTTP headers"))?;
    let header = std::str::from_utf8(&bytes[..split])?;
    ensure!(
        header.starts_with("HTTP/1.1 200 "),
        "sensor HTTP request failed: {}",
        String::from_utf8_lossy(&bytes)
    );
    Ok(serde_json::from_slice(&bytes[split + 4..])?)
}
pub(super) fn run(args: &super::Args, address: SocketAddr) -> Result<()> {
    let mut config = if let Some(path) = &args.sensor_config {
        serde_json::from_slice::<Config>(&std::fs::read(path)?)?
    } else {
        Config {
            robot: args.robot,
            lidar_mount: rm_sim_physics::Pose::at([0., 0., args.mount_z_m]),
            imu_mount: rm_sim_physics::Pose::at([0., 0., args.mount_z_m]),
            stride: args.stride,
            range_sigma_m: args.range_noise_m,
            angular_sigma_rad: args.angular_noise_deg.to_radians(),
            dropout: args.dropout,
            seed: args.seed,
            ..Default::default()
        }
    };
    // A config file is authoritative, including its mount, robot and periods.
    if args.sensor_config.is_none() {
        let half = args.mount_pitch_deg.to_radians() / 2.;
        config.lidar_mount.rotation_wxyz = [half.cos(), 0., half.sin(), 0.];
        config.imu_mount = config.lidar_mount;
    }
    let context = rclrs::Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("rm_live_sensors")?;
    let lidar = LivoxPublisher::new(&node, &args.topic)?;
    let imu = ImuPublisher::new(&node, &args.imu_topic, &args.imu_frame_id)?;
    let begin = Instant::now();
    while (lidar.subscription_count()? == 0 || imu.subscription_count()? == 0)
        && begin.elapsed() < Duration::from_millis(args.wait_for_subscriber_ms)
    {
        std::thread::sleep(Duration::from_millis(10));
    }
    ensure!(
        !args.require_subscriber
            || (lidar.subscription_count()? > 0 && imu.subscription_count()? > 0),
        "LiDAR/IMU subscribers did not connect"
    );
    let id = post(address, "/api/sensors/attach", &config)?["id"]
        .as_u64()
        .ok_or_else(|| anyhow::anyhow!("missing sensor attachment ID"))?;
    let result = (|| -> Result<()> {
        let mut sent = 0;
        let mut drops = 0;
        let mut previous = None;
        let livox = LivoxConfig {
            frame_id: args.frame_id.clone(),
            lidar_id: args.lidar_id,
            ..Default::default()
        };
        while sent < args.frames {
            let batch: Poll = serde_json::from_value(post(address, "/api/sensors/poll", &id)?)?;
            if batch.dropped_intervals != drops {
                eprintln!("Sensor dropped intervals: {}", batch.dropped_intervals);
                drops = batch.dropped_intervals;
            }
            for frame in batch.frames {
                if let Some(last) = previous
                    && frame.sequence != last + 1
                {
                    eprintln!("Sensor packet discontinuity: {last} -> {}", frame.sequence);
                }
                previous = Some(frame.sequence);
                // Send measured IMU up to the completed scan's end before that scan.
                for sample in &frame.imu {
                    imu.publish(sample)?;
                }
                lidar.publish(
                    &LivoxPacket::from_scan(&frame.lidar, &livox).map_err(anyhow::Error::msg)?,
                )?;
                sent += 1;
                if sent == args.frames {
                    break;
                }
            }
            if args.publish_clock {
                lidar.publish_clock(batch.time_ns)?;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        println!("Published {sent} synchronized live LiDAR/IMU packets; {drops} dropped intervals");
        std::thread::sleep(Duration::from_millis(100));
        Ok(())
    })();
    let detached = post(address, "/api/sensors/detach", &id);
    result?;
    detached?;
    Ok(())
}
