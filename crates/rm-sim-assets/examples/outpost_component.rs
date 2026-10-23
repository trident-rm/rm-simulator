// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Load one actual Outpost into a physics-only session, with source rotor geometry.
use anyhow::{Context, ensure};
use rm_sim_assets::{
    cad_assets,
    components::{self, ComponentRole},
};
use rm_sim_physics::{
    ArmorTarget, StepDuration, TargetFace, Team,
    motion::MechanismState,
    session::{RobotScenario, RobotSession},
};

fn main() -> anyhow::Result<()> {
    let root = std::env::args_os()
        .nth(1)
        .map(std::path::PathBuf::from)
        .unwrap_or_else(cad_assets::default_cad_assets);
    let cad = cad_assets::load(&root)?;
    let bindings = cad
        .rmuc_2027
        .as_ref()
        .context("native 2027 source bindings required")?;
    let model = components::load(
        &cad,
        &cad.outpost,
        ComponentRole::Outpost,
        Some(bindings.motions(&cad, &cad.outpost)),
    )?;
    let placement = cad
        .outpost
        .placements
        .first()
        .context("Outpost has no placement")?;
    let team = if placement.translation_m[0] >= 0. {
        Team::Red
    } else {
        Team::Blue
    };
    let source = &bindings.world.outposts[team.index()];
    source.validate().map_err(anyhow::Error::msg)?;
    let angle_rad = 0.5;
    let faces = (0..source.detectors.len())
        .map(|face| TargetFace {
            target: ArmorTarget::Outpost {
                outpost: 0,
                face: face as u32,
            },
            pose: source.detector_pose(face, angle_rad).unwrap(),
        })
        .collect();
    let placed = model.placed(placement);
    let fixed_triangles = placed.fixed.triangles.len();
    let moving_triangles: usize = placed.moving.iter().map(|p| p.mesh.triangles.len()).sum();
    ensure!(
        fixed_triangles > 0 && moving_triangles > 0,
        "incomplete source Outpost"
    );
    let mut state = MechanismState::default();
    state.outpost_angle_rad[team.index()] = angle_rad;
    let scenario = RobotScenario::new(faces, 0., StepDuration::HZ_1000, move |physics| {
        placed.clone().install(physics)?;
        physics.sync_mechanisms(&state);
        Ok(())
    })
    .map_err(anyhow::Error::msg)?;
    let mut session = RobotSession::new(scenario).map_err(anyhow::Error::msg)?;
    session.step(10).map_err(anyhow::Error::msg)?;
    session.reset().map_err(anyhow::Error::msg)?;
    println!(
        "Source Outpost only: {fixed_triangles} fixed + {moving_triangles} moving triangles; 3 detector faces; physics-only reset passed"
    );
    Ok(())
}
