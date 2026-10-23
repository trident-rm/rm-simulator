// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Loaded-map navigation probe: `sentry_navigation FIELD CHASSIS_JSON X Y GOAL_X GOAL_Y`.
//! Uses ordinary host spawn/goal/step commands; prints phase transitions and fails unless arrived.
use rm_sim_server::{
    cad_assets, layout::LayoutOptions, navigation::Phase, protocol::Command, simulation::Simulation,
};
use rm_sim_world::{ChassisConfig, Team};
fn main() -> anyhow::Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    anyhow::ensure!(
        args.len() == 6 || args.len() == 3 && args[2] == "--survey",
        "FIELD CHASSIS_JSON X Y GOAL_X GOAL_Y (or --survey) required"
    );
    let cad = if args[0] == "fixture" {
        None
    } else {
        Some(cad_assets::load(std::path::Path::new(&args[0]))?)
    };
    let config: ChassisConfig = serde_json::from_slice(&std::fs::read(&args[1])?)?;
    config.validate().map_err(anyhow::Error::msg)?;
    let mut sim = if let Some(cad) = &cad {
        Simulation::from_cad(cad, &LayoutOptions::default(), Some(config), true, |_| {})?
    } else {
        use rm_sim_world::{Field, FieldConfig, Pose, projectile::WorldPhysics};
        let mut field = Field::new(&FieldConfig {
            runes: vec![],
            outposts: vec![],
            ..Default::default()
        })?;
        let mut physics = WorldPhysics::new(&[], 0.);
        physics.add_static_box(Pose::at([1., 0., 0.4]), [2., 18., 0.3]);
        let (v, t) = physics.static_geometry_snapshot(true).into_triangles();
        field.add_static_mesh(v, t)?;
        Simulation::new(field, true).with_spawner(rm_sim_server::layout::ChassisSpawner {
            config,
            terrain: None,
        })
    };
    if args.len() == 3 {
        let terrain = rm_sim_server::layout::load_terrain(
            cad.as_ref()
                .ok_or_else(|| anyhow::anyhow!("survey requires a loaded field"))?,
        )?;
        let geometry = sim.field().fixed_geometry_snapshot();
        let mut samples = vec![];
        for ix in -25..=70 {
            for iy in -10..=70 {
                let (x, y) = (ix as f64 * 0.1, iy as f64 * 0.1);
                for max in [-0.2, -0.1, 0.0, 0.15] {
                    if let Some(z) = terrain.ground.support_height_below(x, y, max) {
                        let low = [x, y, z + 0.139];
                        let high = [x, y, z + 0.40];
                        samples.push(serde_json::json!([
                            x,
                            y,
                            z,
                            geometry.cylinder_sweep_clear(low, low, 0.49, 0.109),
                            geometry.cylinder_sweep_clear(high, high, 0.49, 0.24)
                        ]));
                    }
                }
            }
        }
        let mut columns = vec![];
        for x in [4.5, 4.7, 4.9, -0.3, 0., 0.3] {
            for y in [5.0, 5.1, 5.2, 5.3, 5.4, 5.5, 5.6, 5.7, 5.8, 6.0, 6.3, 6.5] {
                let mut levels = vec![];
                let mut max = 1.;
                while let Some(z) = terrain.ground.ground_height_below(x, y, max) {
                    levels.push(z);
                    max = z - 0.002;
                    if levels.len() > 32 {
                        break;
                    }
                }
                columns.push(serde_json::json!([x, y, levels]));
            }
        }
        println!(
            "{}",
            serde_json::json!({"samples":samples,"columns":columns})
        );
        return Ok(());
    }
    let xy: Vec<f64> = args[2..]
        .iter()
        .map(|v| v.parse())
        .collect::<Result<_, _>>()?;
    sim.apply(&Command::SpawnSentry {
        team: Team::Red,
        position_m: [xy[0], xy[1], 1.],
        yaw_deg: 0.,
    })
    .map_err(anyhow::Error::msg)?;
    let id = sim.state().navigation[0].chassis;
    sim.step(128)?;
    sim.apply(&Command::NavigateSentry {
        chassis: id,
        position_m: [xy[2], xy[3]],
    })
    .map_err(anyhow::Error::msg)?;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(90);
    let mut previous = String::new();
    for _ in 0..2400 {
        // Poll the host worker even while paused, bounded by a wall-clock test deadline.
        loop {
            sim.step(1)?;
            let phase = sim.state().navigation[0].phase;
            if phase != Phase::Planning && phase != Phase::Baking {
                break;
            }
            anyhow::ensure!(
                std::time::Instant::now() < deadline,
                "planning deadline exceeded"
            );
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        let state = sim.state();
        let nav = &state.navigation[0];
        let message = format!("{:?}: {}", nav.phase, nav.message);
        if message != previous {
            println!(
                "{}",
                serde_json::json!({"navigation":nav,"body":sim.field().chassis_snapshot(id)})
            );
            previous = message.clone();
        }
        if nav.phase == Phase::Arrived {
            sim.step(384)?;
            println!(
                "{}",
                serde_json::json!({"arrived":true,"body":sim.field().chassis_snapshot(id)})
            );
            return Ok(());
        }
        anyhow::ensure!(
            nav.phase == Phase::Following,
            "navigation failed: {message}"
        );
        sim.step(8)?;
    }
    anyhow::bail!("did not arrive within 150 simulated seconds")
}
