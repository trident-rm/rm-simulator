// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Small flat-ground rollout, observation and reset probe; no CAD or renderer.
//! Build in release mode before running. Times include allocation, not setup.
use rm_simulator_world::{
    ChassisCommand, ChassisConfig, ChassisPlacement, Field, FieldConfig, Pose, RefereeCommand,
    RobotKind, Team,
    rmuc_2027::{Config, Detector, Point, Tower, Zone},
    session::MatchSession,
};
use std::{
    collections::hash_map::DefaultHasher,
    hash::{Hash, Hasher},
    hint::black_box,
    time::Instant,
};

fn fixture(robots: usize) -> Field {
    let detector = Detector {
        pose: Pose::at([20., 0., 1.]),
        half_m: [0.045, 0.044],
    };
    let zone = Zone {
        side: Team::Red,
        point: Point::Base,
        floor_m: 0.,
        height_m: 0.5,
        polygon_m: vec![[-15., -5.], [15., -5.], [15., 5.], [-15., 5.]],
    };
    let config = FieldConfig {
        rmuc_2027: Some(Config {
            bases: [vec![detector.clone()], vec![detector.clone()]],
            outposts: std::array::from_fn(|_| Tower {
                pivot_m: [20., 0., 1.],
                axis: [0., 0., 1.],
                detectors: vec![detector.clone(); 3],
            }),
            zones: vec![zone.clone()],
            supply_zones: vec![zone],
            vantage_zones: vec![],
            terrain_zones: vec![],
            runes: vec![],
            rotor_direction: 1,
        }),
        chassis: (0..robots)
            .map(|id| {
                let config = ChassisConfig::default();
                ChassisPlacement {
                    spawn: Pose::at([id as f64 * 2. - 7., 0., config.rest_height_m()]),
                    config,
                    team: if id % 2 == 0 { Team::Red } else { Team::Blue },
                    kind: RobotKind::Infantry,
                    performance: None,
                }
            })
            .collect(),
        ..Default::default()
    };
    let mut field = Field::new(&config).unwrap();
    field.referee_command(RefereeCommand::StartMatch).unwrap();
    field.step(640).unwrap(); // Five-second countdown at the default 128 Hz.
    for id in 0..robots as u32 {
        field
            .command_chassis(id, ChassisCommand::default())
            .unwrap();
    }
    field
}

fn main() {
    const STEPS: u64 = 1024;
    for robots in [0, 1, 8] {
        let mut session = MatchSession::new(fixture(robots));
        let start = Instant::now();
        for _ in 0..STEPS {
            black_box(session.step(1).unwrap());
        }
        println!(
            "{robots} robots: {:.3} us/step",
            start.elapsed().as_secs_f64() * 1e6 / STEPS as f64
        );
        let mut hasher = DefaultHasher::new();
        format!("{:?}", session.ground_truth().snapshot()).hash(&mut hasher);
        println!("final snapshot hash: {:016x}", hasher.finish());
        let start = Instant::now();
        for _ in 0..200 {
            black_box(session.checkpoint());
        }
        println!(
            "checkpoint: {:.3} us/capture",
            start.elapsed().as_secs_f64() * 1e6 / 200.
        );
        let start = Instant::now();
        for _ in 0..32 {
            session.reset(42).unwrap();
            black_box(session.stamp());
        }
        println!(
            "reset: {:.3} us/reset",
            start.elapsed().as_secs_f64() * 1e6 / 32.
        );
    }
}
