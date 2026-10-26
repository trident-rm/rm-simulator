// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Exercise an opt-in hybrid package through terrain, native cooking and queries.
use rm_sim_assets::{
    arena::{PreparedArena, load_terrain},
    cad_assets,
    components::{self, ComponentRole},
    math,
};
use rm_sim_physics::{WorldPhysics, model::ShapeDescription};

fn main() -> anyhow::Result<()> {
    let root = std::env::args_os()
        .nth(1)
        .expect("usage: inspect_primitive_shapes PACKAGE");
    let cad = cad_assets::load(std::path::Path::new(&root))?;
    let terrain = load_terrain(&cad)?;
    let fixed_mesh_triangles = terrain.ground.triangles.len()
        + terrain.fixtures.triangles.len()
        + terrain
            .equipment
            .iter()
            .map(|m| m.triangles.len())
            .sum::<usize>();
    let moving_mesh_triangles = terrain
        .mechanisms
        .iter()
        .map(|m| m.mesh.triangles.len())
        .sum::<usize>();
    let native_parts = terrain.native_shapes.iter().map(Vec::len).sum::<usize>()
        + terrain
            .mechanisms
            .iter()
            .map(|m| m.native_shapes.len())
            .sum::<usize>();
    let prepared = PreparedArena::from_terrain(terrain).map_err(anyhow::Error::msg)?;
    let _ = prepared.geometry().clone().into_triangles(); // Wireframe/query export must include cylinders too.
    let asset = &cad.tech_core;
    let model = components::load(
        &cad,
        asset,
        ComponentRole::Equipment,
        cad.rmuc_2027.as_ref().map(|b| b.motions(&cad, asset)),
    )?;
    let mut checked = 0;
    for root in &asset.placements {
        for shape in model.placed(root).native_shapes {
            let ShapeDescription::Cylinder {
                radius_m,
                half_height_m,
            } = &shape.1
            else {
                anyhow::bail!("expected native cylinder")
            };
            let mut world = WorldPhysics::new(&[], 0.);
            world
                .add_static_shapes(std::slice::from_ref(&shape))
                .map_err(anyhow::Error::msg)?;
            let scene = world.static_geometry_snapshot(true);
            for (offset, direction) in [
                ([radius_m + 0.1, 0., 0.], [-1., 0., 0.]),
                ([0., half_height_m + 0.1, 0.], [0., -1., 0.]),
            ] {
                let origin = math::add(
                    shape.0.translation_m,
                    math::rotate(shape.0.rotation_wxyz, offset),
                );
                let direction = math::rotate(shape.0.rotation_wxyz, direction);
                let hit = scene
                    .cast_ray(origin, direction, 0.2)
                    .map_err(anyhow::Error::msg)?
                    .ok_or_else(|| {
                        anyhow::anyhow!("native cylinder missing from geometry capture")
                    })?;
                anyhow::ensure!(
                    (hit.distance_m - 0.1).abs() < 1e-8,
                    "native side/cap pose mismatch"
                );
                checked += 1;
            }
        }
    }
    println!(
        "{}",
        serde_json::json!({"mesh_asset_triangles":fixed_mesh_triangles+moving_mesh_triangles,"boundary_triangles":prepared.terrain().boundary.triangles.len(),"native_parts":native_parts,"native_fixed_groups":prepared.terrain().native_shapes.len(),"placed_side_cap_rays_checked":checked})
    );
    Ok(())
}
