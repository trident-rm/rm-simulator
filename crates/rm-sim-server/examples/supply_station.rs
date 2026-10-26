// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Headless source-station installation and checkpoint continuation probe.
use rm_sim_server::{cad_assets, layout};
use rm_sim_world::{Field, energy_unit::Attachment};

fn main() -> anyhow::Result<()> {
    let path = std::env::args_os()
        .nth(1)
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::path::PathBuf::from("field-2027"));
    let start = std::time::Instant::now();
    let cad = cad_assets::load(&path)?;
    let terrain = layout::load_terrain(&cad)?;
    let mut field = Field::new(&Default::default())?;
    layout::add_terrain(&mut field, &terrain)?;
    field.add_chassis(&rm_sim_world::ChassisPlacement {
        config: Default::default(),
        spawn: rm_sim_world::Pose::at([0., 0., 0.4]),
        team: rm_sim_world::Team::Red,
        kind: rm_sim_world::RobotKind::Engineer,
        performance: None,
    })?;
    let startup_s = start.elapsed().as_secs_f64();
    let stock = field.energy_units();
    anyhow::ensure!(
        stock.len() == 12 && stock.iter().all(|u| u.attachment == Attachment::Mounted),
        "expected six mounted Units per station"
    );
    let geometry = field.static_geometry_snapshot();
    let mut restored = Field::restore(&field.snapshot(), &geometry, 0.)?;
    let stepping = std::time::Instant::now();
    field.step(128)?;
    let stepping_s = stepping.elapsed().as_secs_f64();
    for _ in 0..128 {
        restored.step(1)?;
    }
    let after = field.energy_units();
    anyhow::ensure!(
        after == restored.energy_units(),
        "partitioned/restored stock differs"
    );
    let mut max_drift_m = 0f64;
    for (before, after) in stock.iter().zip(&after) {
        anyhow::ensure!(
            before.physical.id == after.physical.id
                && before.supply == after.supply
                && after.attachment == Attachment::Mounted
                && before.physical.shape == after.physical.shape,
            "station inventory changed without a robot grip"
        );
        let drift = glam::DVec3::from_array(before.physical.pose.translation_m)
            .distance(glam::DVec3::from_array(after.physical.pose.translation_m));
        max_drift_m = max_drift_m.max(drift);
    }
    anyhow::ensure!(max_drift_m < 1e-10, "station stock moved {max_drift_m} m");
    let simulation = rm_sim_server::simulation::Simulation::new(field, true);
    let state = simulation.state();
    let node = rm_sim_server::snapshot_codec::checkpoint_node(&state)?;
    let bytes = rm_sim_server::snapshot_codec::encode_checkpoint(
        &node,
        state.field.tick,
        None,
        0,
        &mut rm_sim_server::snapshot_codec::Prediction::default(),
    )?;
    let (message, _) = rm_sim_server::snapshot_codec::decode_checkpoint(&bytes, None, 0, false)?;
    let rm_sim_server::protocol::ServerMessage::Snapshot(decoded) = message else {
        anyhow::bail!("checkpoint did not decode to a world snapshot");
    };
    let decoded = Field::restore(&decoded.field, &geometry, 0.)?;
    anyhow::ensure!(
        decoded.energy_units() == after,
        "wire checkpoint changed source stock"
    );
    println!(
        "{}",
        serde_json::json!({"robots":1,"mounted_units":stock.len(),"max_station_drift_m":max_drift_m,"startup_s":startup_s,
        "checkpoint_bytes":bytes.len(),"wire_restore_match":true,
        "one_second_step_s":stepping_s,"checkpoint_and_partition_match":true,
        "model":"axial force-limited retrieval; prescribed carry after extraction"})
    );
    Ok(())
}
