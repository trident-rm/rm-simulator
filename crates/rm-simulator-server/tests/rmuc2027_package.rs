// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Hydrated-package integration: native admission, disjoint motion and restore.
use rm_simulator_server::{cad_assets, layout};
use rm_simulator_world::{Field, referee::RefereeCommand, season::Season};

#[test]
fn reviewed_field_has_live_motion_native_rules_and_restorable_authority() -> anyhow::Result<()> {
    if !cfg!(feature = "season-2027") {
        return Ok(());
    }
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../field-2027");
    let manifest = std::fs::read(root.join("manifest.json"))?;
    if manifest.starts_with(b"version https://git-lfs.github.com/spec/") {
        eprintln!("field-2027 is not hydrated; release validation runs this integration");
        return Ok(());
    }
    let cad = cad_assets::load(&root)?;
    let bindings = cad.rmuc_2027.as_ref().expect("native bindings");
    assert_eq!(bindings.schema_version, 2);
    assert_eq!(bindings.outpost_motion[0].bodies.len(), 18);
    assert_eq!(bindings.parts.len(), 386);
    let mut invalid = bindings.clone();
    invalid.outpost_motion[0]
        .bodies
        .push("source_body_99999999".into());
    assert!(invalid.validate().is_err());
    let terrain = layout::load_terrain(&cad)?;
    assert_eq!(terrain.mechanisms.len(), 8);
    let moving: usize = terrain
        .mechanisms
        .iter()
        .map(|m| m.mesh.triangles.len())
        .sum();
    let fixed = terrain.ground.triangles.len()
        + terrain.fixtures.triangles.len()
        + terrain
            .equipment
            .iter()
            .map(|m| m.triangles.len())
            .sum::<usize>();
    assert_eq!(
        fixed + moving + terrain.boundary.triangles.len(),
        545_599 + 48
    );
    let config = layout::field_config(&cad, &layout::LayoutOptions::default());
    let mut field = Field::new_for_season(Season::Rm2027, &config)?;
    layout::add_terrain(&mut field, &terrain)?;
    let initial_motion = field.dynamic_geometry_snapshot().into_triangles();
    field.referee_command(RefereeCommand::StartMatch)?;
    field.step(6 * 128)?;
    let checkpoint = field.snapshot();
    assert_ne!(
        initial_motion,
        field.dynamic_geometry_snapshot().into_triangles()
    );
    assert!(checkpoint.referee.is_none());
    assert_eq!(
        checkpoint.rmuc_2027.as_ref().unwrap().game.phase,
        rm_simulator_world::rmuc_2027::match_rules::Phase::Running
    );
    let geometry = field.static_geometry_snapshot();
    let mut restored = Field::restore(&checkpoint, &geometry, config.floor_height_m)?;
    field.step(17)?;
    restored.step(17)?;
    assert_eq!(field.snapshot().rmuc_2027, restored.snapshot().rmuc_2027);
    Ok(())
}
