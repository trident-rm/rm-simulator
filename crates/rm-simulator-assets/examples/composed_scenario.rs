// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! CAD-free composition, or checked Outposts with explicit identity remapping.
use rm_simulator_assets::{
    cad_assets,
    scenarios::{
        Preset, RobotSpawn, Scenario,
        composition::{ComponentInstance, Composition, InstanceId, RfidVolume, RobotInstance},
    },
};
use rm_simulator_physics::{
    ArmorTarget, Pose, StepDuration, Team, arena::TunnelModel, chassis::ChassisConfig,
    motion::MechanismState,
};
use std::time::Instant;

fn main() -> anyhow::Result<()> {
    let mut components = Vec::new();
    if let Some(root) = std::env::args_os().nth(1) {
        let cad = cad_assets::load(std::path::Path::new(&root))?;
        // Caller-chosen physical indices 101/102 carry no new team policy.
        for (id, team, physical) in [(10, Team::Red, 101), (20, Team::Blue, 102)] {
            let source = Scenario::load(&cad, Preset::Outpost(team))?;
            let targets = source
                .targets()
                .iter()
                .map(|t| match t.face.target {
                    ArmorTarget::Outpost { face, .. } => ArmorTarget::Outpost {
                        outpost: physical,
                        face,
                    },
                    _ => unreachable!("Outpost preset supplies Outpost targets"),
                })
                .collect();
            components.push(ComponentInstance {
                id: InstanceId(id),
                source: source.into(),
                targets,
                placement: Pose::yawed([0., 0., 0.], 0.4),
            });
        }
    } else {
        let source = Scenario::tunnel(TunnelModel {
            pose: Pose::default(),
            opening_m: [2., 1., 0.65],
            wall_thickness_m: 0.1,
        })?;
        for (id, x_m) in [(10, 0.), (20, 4.)] {
            components.push(ComponentInstance {
                id: InstanceId(id),
                source: source.clone().into(),
                placement: Pose::yawed([x_m, 0., 0.], 0.3),
                targets: vec![],
            });
        }
    }
    let start = Instant::now();
    let definition = Composition {
        components,
        robots: vec![RobotInstance {
            id: InstanceId(30),
            spawn: RobotSpawn {
                team: Team::Red,
                config: ChassisConfig::default(),
                pose: Pose::at([-2., 0., 0.4]),
            },
        }],
        rfid: vec![RfidVolume {
            id: InstanceId(40),
            label: "explicit practice antenna".into(),
            pose: Pose::at([-2., 0., 0.]),
            polygon_m: vec![[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]],
            z_range_m: [0., 1.],
        }],
        catch_floor_m: Some(0.),
        ..Default::default()
    }
    .prepare()?;
    let prepare = start.elapsed();
    let state = MechanismState {
        outpost_angle_rad: [0.4, -0.2],
        ..Default::default()
    };
    let start = Instant::now();
    let mut session = definition.session(StepDuration::HZ_1000, state)?;
    let create = start.elapsed();
    let mut targets = Vec::new();
    definition.targets_at(&state, &mut targets)?;
    for _ in 0..100 {
        session
            .step_with_targets(&targets)
            .map_err(anyhow::Error::msg)?;
    }
    let start = Instant::now();
    for _ in 0..100 {
        session.reset().map_err(anyhow::Error::msg)?;
    }
    let reset_mean = start.elapsed() / 100;
    anyhow::ensure!(session.stamp().time_ns == 0, "reset clock mismatch");
    anyhow::ensure!(
        session
            .robot_truth(definition.robot_identities()[0].physical)
            .is_some(),
        "robot identity mismatch"
    );
    println!(
        "{} components, {} targets, {} visuals, {} RFID volumes; {} prepared triangles",
        definition.components().len(),
        targets.len(),
        definition.visuals().len(),
        definition.rfid().len(),
        definition.geometry().clone().into_triangles().1.len()
    );
    println!(
        "prepare {prepare:?}; create {create:?}; mean of 100 resets {reset_mean:?}; reset passed"
    );
    Ok(())
}
