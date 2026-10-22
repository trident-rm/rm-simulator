// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Source equipment presets, caller-prescribed motion and repeatable bare reset.
use anyhow::{Context, bail};
use rm_simulator_assets::{
    cad_assets,
    scenarios::{Preset, Scenario},
};
use rm_simulator_physics::{StepDuration, Team, motion::MechanismState};

fn main() -> anyhow::Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let name = args
        .first()
        .map(|s| s.to_string_lossy())
        .unwrap_or_else(|| "outpost-red".into());
    let preset = match name.as_ref() {
        "outpost-red" => Preset::Outpost(Team::Red),
        "outpost-blue" => Preset::Outpost(Team::Blue),
        "base-red" => Preset::Base(Team::Red),
        "base-blue" => Preset::Base(Team::Blue),
        "rune" => Preset::Rune,
        "tunnel-fixture" => Preset::TunnelFixture,
        _ => bail!("choose outpost-red, outpost-blue, base-red, base-blue, rune or tunnel-fixture"),
    };
    let scenario = if preset == Preset::TunnelFixture {
        Scenario::tunnel(rm_simulator_physics::arena::TunnelModel {
            pose: rm_simulator_physics::Pose::at([0.; 3]),
            opening_m: [2., 1., 0.25],
            wall_thickness_m: 0.1,
        })?
    } else {
        let root = args
            .get(1)
            .map(std::path::PathBuf::from)
            .unwrap_or_else(cad_assets::default_cad_assets);
        Scenario::load(&cad_assets::load(&root)?, preset)?
    };
    let initial = MechanismState::default();
    let mut session = scenario.session(StepDuration::HZ_1000, vec![], initial)?;
    let mut targets = Vec::new();
    for tick in 1..=100 {
        // An illustrative caller trajectory, not native match scheduling.
        let angle = f64::from(tick) * 0.001;
        let state = MechanismState {
            outpost_angle_rad: [angle; 2],
            rune_angle_rad: [angle; 2],
            base_open_fraction: [f64::from(tick) / 100.; 2],
            ..initial
        };
        scenario.targets_at(&state, &mut targets)?;
        session.physics_mut().sync_mechanisms(&state);
        session
            .step_with_targets(&targets)
            .map_err(anyhow::Error::msg)?;
    }
    let end_ns = session.stamp().time_ns;
    session.reset().map_err(anyhow::Error::msg)?;
    anyhow::ensure!(
        session.stamp().time_ns == 0,
        "reset did not reset the clock"
    );
    if preset != Preset::TunnelFixture {
        scenario
            .visual()
            .context("source preset has no visual reference")?;
    }
    let (_, triangles) = scenario.geometry().clone().into_triangles();
    println!(
        "{preset:?}: {} cooked collision triangles, {} target faces, {end_ns} ns; reset passed",
        triangles.len(),
        scenario.targets().len()
    );
    Ok(())
}
