// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Compare privileged telemetry extraction with full chassis snapshot cloning.
//! Run with `cargo run --release -p rm-simulator-physics --example telemetry_cost --locked`.
use rm_simulator_physics::{
    Pose, Team, WorldPhysics,
    chassis::ChassisConfig,
    mechanism::{ArticulationConfig, JointDrive, SourceJoint},
};
use std::{hint::black_box, time::Instant};

fn source_chain_fixture() -> ChassisConfig {
    let mut config = ChassisConfig::default();
    // Exercise the source-backed kinematics path without loading CAD or inferring a real robot.
    config.articulation = Some(ArticulationConfig {
        profile: "telemetry-source-chain-fixture".into(),
        limit_rad: 1.,
        speed_rad_s: 0.5,
        muzzle_offset_m: [0.2, 0., 0.],
        camera_offset_m: [0.1, 0., 0.],
        armor_faces: config.armor_faces(),
        joints: (0..8)
            .map(|i| SourceJoint {
                name: if i == 7 {
                    "pitch".into()
                } else {
                    format!("joint-{i}")
                },
                parent: if i == 0 { None } else { Some(i - 1) },
                pivot_m: [0.01 * i as f64, 0.02, 0.1 + 0.02 * i as f64],
                axis: [0., 1., 0.],
                angle_rad: -1.,
                translation_m: [0.01, 0., -0.01],
                drive: if i == 7 {
                    JointDrive::Pitch
                } else {
                    JointDrive::Transform
                },
                collision: None,
            })
            .collect(),
    });
    config
}

fn measure(label: &str, config: ChassisConfig) -> Result<(), &'static str> {
    println!("{label}");
    let mut world = WorldPhysics::new(&[], 0.);
    let id = world.add_chassis(Team::Red, config, Pose::at([0., 0., 0.5]))?;
    let mut wheels = Vec::new();
    world
        .chassis_telemetry(id)
        .unwrap()
        .wheels_into(&mut wheels);
    let pointer = wheels.as_ptr();
    let capacity = wheels.capacity();
    let iterations = 100_000;
    for mode in ["snapshot", "borrowed", "wheel_buffer"] {
        let start = Instant::now();
        for _ in 0..iterations {
            let world = black_box(&world);
            match mode {
                "snapshot" => {
                    black_box(world.chassis_snapshot(black_box(id)).unwrap());
                }
                "borrowed" => {
                    black_box(world.chassis_telemetry(black_box(id)).unwrap());
                }
                _ => {
                    world
                        .chassis_telemetry(black_box(id))
                        .unwrap()
                        .wheels_into(&mut wheels);
                    black_box(&wheels);
                }
            }
        }
        println!(
            "{mode}: {:.1} ns/read ({iterations} reads)",
            start.elapsed().as_nanos() as f64 / f64::from(iterations)
        );
    }
    assert_eq!(pointer, wheels.as_ptr());
    assert_eq!(capacity, wheels.capacity());
    println!("wheel buffer retained: {capacity} slots; allocation counts are not instrumented");
    Ok(())
}

fn main() -> Result<(), &'static str> {
    measure("folding Hero", ChassisConfig::folding_hero())?;
    measure(
        "8-joint source articulation fixture",
        source_chain_fixture(),
    )
}
