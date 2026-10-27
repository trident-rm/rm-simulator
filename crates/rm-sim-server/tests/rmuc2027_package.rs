// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Hydrated-package integration: native admission, disjoint motion and restore.
use rm_sim_server::{cad_assets, layout};
use rm_sim_world::{Field, referee::RefereeCommand};

#[test]
fn reviewed_field_has_live_motion_native_rules_and_restorable_authority() -> anyhow::Result<()> {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../field-2027");
    let manifest = std::fs::read(root.join("manifest.json"))?;
    if manifest.starts_with(b"version https://git-lfs.github.com/spec/") {
        eprintln!("field-2027 is not hydrated; release validation runs this integration");
        return Ok(());
    }
    let cad = cad_assets::load(&root)?;
    let bindings = cad.rmuc_2027.as_ref().expect("native bindings");
    assert_eq!(bindings.schema_version, 2);
    assert_eq!(bindings.outpost_motion[0].bodies.len(), 13);
    assert_eq!(bindings.parts.len(), 105);
    assert_eq!(bindings.rune_motion.len(), 2);
    assert_eq!(bindings.rune_arms.len(), 2);
    assert_eq!(bindings.config().terrain_zones.len(), 30);
    let mut invalid = bindings.clone();
    invalid.outpost_motion[0]
        .bodies
        .push("source_body_99999999".into());
    assert!(invalid.validate().is_err());
    let terrain = layout::load_terrain(&cad)?;
    let native = terrain
        .native_terrain
        .as_ref()
        .expect("native terrain deployment");
    assert_eq!(native.native_heightfields, 2);
    assert_eq!(native.parts.len(), 29);
    assert!(terrain.ground_collision.is_some());
    assert_eq!(terrain.mechanisms.len(), 10);
    assert_eq!(terrain.endpoint_bakes.len(), 2);
    let moving: usize = terrain
        .mechanisms
        .iter()
        .map(|m| m.mesh.triangles.len())
        .sum();
    // Full source ground remains available for support lookups; physical fixtures
    // use the independently reviewed remainder instead.
    // Twelve authored Units move to persistent inventory, removing their
    // 2,784 source triangles each from both static contact and support meshes.
    let unit_triangles = 12 * 2_784;
    assert_eq!(terrain.supply_units.len(), 12);
    assert_eq!(
        terrain.collision_ground().triangles.len(),
        58_028 + 4 - unit_triangles
    );
    let fixed = terrain.ground.triangles.len()
        + terrain.fixtures.triangles.len()
        + terrain
            .equipment
            .iter()
            .map(|m| m.triangles.len())
            .sum::<usize>();
    // Resource fittings, Dart panels and road/Resource markings remove 25,524 support triangles.
    assert_eq!(
        fixed + moving + terrain.boundary.triangles.len(),
        109_003 + 1_280 + 48 - unit_triangles
    );
    let config = layout::field_config(&cad, &layout::LayoutOptions::default());
    let mut field = Field::new(&config)?;
    layout::add_terrain(&mut field, &terrain)?;
    assert_eq!(field.energy_units().len(), 12);
    assert_eq!(
        field
            .static_geometry_snapshot()
            .moving_parts()
            .iter()
            .filter(|part| part.kind == rm_sim_world::referee::Mechanism::Rune)
            .count(),
        2
    );
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
        rm_sim_world::rmuc_2027::match_rules::Phase::Running
    );
    let geometry = field.static_geometry_snapshot();
    let mut restored = Field::restore(&checkpoint, &geometry, config.floor_height_m)?;
    field.step(17)?;
    restored.step(17)?;
    assert_eq!(field.snapshot().rmuc_2027, restored.snapshot().rmuc_2027);
    Ok(())
}
