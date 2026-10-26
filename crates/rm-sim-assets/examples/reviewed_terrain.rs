// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Compare current reviewed terrain with reconciled grids and the four-face floor.
use anyhow::{Context, Result, ensure};
use rm_sim_assets::{
    collision_mesh::load_glb_bytes,
    math::{quat_axis_angle, rotate},
    terrain_collision::{Quality, Representation, TerrainPackage},
};
use rm_sim_physics::{
    Pose, StaticGeometry, TargetFrames, Team, WorldPhysics,
    chassis::{ChassisCommand, ChassisConfig},
    cooking::CookedRigidBody,
    queries::RayQuery,
};
use sha2::{Digest, Sha256};
use sim_engine::model::{
    BodyKind, ColliderDescription, ContactMaterial, RigidBodyDescription, ShapeDescription,
};
use std::path::Path;

fn material() -> ContactMaterial {
    ContactMaterial {
        friction: 0.6,
        restitution: 0.,
        memberships: u32::MAX,
        filter: u32::MAX,
    }
}
fn model(shape: ShapeDescription) -> Result<CookedRigidBody> {
    let collider =
        ColliderDescription::new(shape, Pose::default(), material()).map_err(anyhow::Error::msg)?;
    CookedRigidBody::new(
        RigidBodyDescription::new(BodyKind::Fixed, vec![collider]).map_err(anyhow::Error::msg)?,
    )
    .map_err(anyhow::Error::msg)
}
fn world(package: &TerrainPackage) -> Result<WorldPhysics> {
    let mut world = WorldPhysics::new(&[], -10.);
    world.remove_catch_floor();
    package.install_in_world(&mut world)?;
    Ok(world)
}
fn compare(
    a: &StaticGeometry,
    b: &StaticGeometry,
    rays: &[RayQuery],
    tolerance_m: f64,
) -> Result<serde_json::Value> {
    let mut ah = vec![None; rays.len()];
    let mut bh = ah.clone();
    a.cast_rays_into(rays, &mut ah)
        .map_err(anyhow::Error::msg)?;
    b.cast_rays_into(rays, &mut bh)
        .map_err(anyhow::Error::msg)?;
    let mut changed = 0;
    let mut maximum: f64 = 0.;
    let mut maximum_normal_angle: f64 = 0.;
    for (a, b) in ah.into_iter().zip(bh) {
        match (a, b) {
            (Some(a), Some(b)) => {
                maximum = maximum.max((a.distance_m - b.distance_m).abs());
                let dot: f64 = a.normal.iter().zip(b.normal).map(|(x, y)| x * y).sum();
                maximum_normal_angle =
                    maximum_normal_angle.max(dot.clamp(-1., 1.).acos().to_degrees());
            }
            (None, None) => {}
            _ => changed += 1,
        }
    }
    ensure!(changed == 0, "{changed} hit/miss changes");
    ensure!(
        maximum <= tolerance_m,
        "maximum range change {maximum} m exceeds {tolerance_m}"
    );
    Ok(
        serde_json::json!({"rays":rays.len(),"hit_miss_changes":changed,"max_range_change_mm":maximum*1000.,"max_normal_angle_degrees":maximum_normal_angle}),
    )
}
fn bounds(package: &TerrainPackage) -> Result<[f64; 4]> {
    let floor = package
        .parts
        .iter()
        .find(|p| p.id == "floor/two-quads")
        .context("missing floor")?;
    let ShapeDescription::TriangleMesh(mesh) = floor.model.source().colliders()[0].shape() else {
        anyhow::bail!("floor must be mesh-backed");
    };
    let mut b = [
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ];
    for p in mesh.vertices_m() {
        let q = rotate(floor.placements[0].rotation_wxyz, *p);
        let p: [f64; 3] = std::array::from_fn(|i| q[i] + floor.placements[0].translation_m[i]);
        b[0] = b[0].min(p[0]);
        b[1] = b[1].max(p[0]);
        b[2] = b[2].min(p[1]);
        b[3] = b[3].max(p[1]);
    }
    Ok(b)
}
fn chassis(package: &TerrainPackage, b: [f64; 4]) -> Result<Vec<[f64; 3]>> {
    let mut world = world(package)?;
    let x = (b[0] + b[1]) / 2.;
    let y = (b[2] + b[3]) / 2. - 0.6;
    let ground = world
        .static_geometry_snapshot(true)
        .cast_ray([x, y, 3.], [0., 0., -1.], 6.)
        .map_err(anyhow::Error::msg)?
        .context("missing chassis support")?;
    let config = ChassisConfig::default();
    let height = 3. - ground.distance_m + config.rest_height_m();
    let id = world
        .add_chassis(
            Team::Red,
            config,
            Pose {
                translation_m: [x, y, height],
                rotation_wxyz: quat_axis_angle([0., 0., 1.], std::f64::consts::FRAC_PI_2),
            },
        )
        .map_err(anyhow::Error::msg)?;
    world
        .command_chassis(
            id,
            ChassisCommand {
                forward_m_s: 0.5,
                ..Default::default()
            },
        )
        .map_err(anyhow::Error::msg)?;
    let frames = TargetFrames::new(vec![]);
    let mut positions = Vec::new();
    for tick in 1..=768 {
        world
            .step(tick * 7_812_500, &frames)
            .map_err(anyhow::Error::msg)?;
        let p = world
            .chassis_snapshot(id)
            .context("missing chassis")?
            .pose
            .translation_m;
        ensure!(p.iter().all(|x| x.is_finite()), "invalid chassis pose");
        positions.push(p);
    }
    Ok(positions)
}
fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    ensure!(
        args.len() == 3,
        "usage: reviewed_terrain PACKAGE EXPECTED_MANIFEST_SHA256"
    );
    let root = Path::new(&args[1]);
    let bytes = std::fs::read(root.join("manifest.json"))?;
    ensure!(
        format!("{:x}", Sha256::digest(&bytes)) == args[2],
        "manifest checksum mismatch"
    );
    let manifest: serde_json::Value = serde_json::from_slice(&bytes)?;
    let pose = Pose {
        translation_m: [0., -1.6243436765, 1.641343676495],
        rotation_wxyz: quat_axis_angle([1., 0., 0.], std::f64::consts::FRAC_PI_2),
    };
    let mesh = TerrainPackage::load(
        root,
        Quality::Balanced,
        Representation::Mesh,
        pose,
        material(),
    )?;
    let native = TerrainPackage::load(
        root,
        Quality::Balanced,
        Representation::NativePreferred,
        pose,
        material(),
    )?;
    ensure!(
        native.native_heightfields == 2 && native.parts.len() == 30,
        "unexpected terrain inventory"
    );
    let mut baseline = mesh.clone();
    for part in &mut baseline.parts {
        let row = manifest["parts"]
            .as_array()
            .unwrap()
            .iter()
            .find(|p| p["id"] == part.id)
            .context("missing part")?;
        let variant = row["qualities"]["balanced"]
            .as_array()
            .unwrap()
            .iter()
            .find(|v| v["id"] == "balanced-reviewed-mesh")
            .context("missing reviewed control")?;
        let file = &variant["files"][0];
        let bytes = std::fs::read(root.join(file["path"].as_str().context("missing mesh path")?))?;
        ensure!(
            format!("{:x}", Sha256::digest(&bytes)) == file["sha256"].as_str().unwrap(),
            "reviewed control hash mismatch"
        );
        part.model = model(ShapeDescription::TriangleMesh(
            load_glb_bytes(&bytes)?
                .into_description()
                .map_err(anyhow::Error::msg)?,
        ))?;
    }
    let b = bounds(&baseline)?;
    let mut rays = Vec::new();
    for i in 0..257 {
        for j in 0..129 {
            let x = b[0] - 0.03 + (b[1] - b[0] + 0.06) * i as f64 / 256.;
            let y = b[2] - 0.03 + (b[3] - b[2] + 0.06) * j as f64 / 128.;
            rays.push(RayQuery::new([x, y, 3.], [0., 0., -1.], 6.).map_err(anyhow::Error::msg)?);
        }
    }
    // Explicit centre-seam and finite-footprint probes, from both sides.
    for i in 0..257 {
        let x = b[0] + (i as f64 + 0.37) / 257. * (b[1] - b[0]);
        for y in [
            b[2] - 0.001,
            b[2] + 0.001,
            (b[2] + b[3]) / 2. - 0.001,
            (b[2] + b[3]) / 2. + 0.001,
            b[3] - 0.001,
            b[3] + 0.001,
        ] {
            for (z, d) in [(3., -1.), (-3., 1.)] {
                rays.push(RayQuery::new([x, y, z], [0., 0., d], 6.).map_err(anyhow::Error::msg)?);
            }
        }
    }
    let baseline_world = world(&baseline)?;
    let mesh_world = world(&mesh)?;
    let native_world = world(&native)?;
    let full = compare(
        &baseline_world.static_geometry_snapshot(true),
        &native_world.static_geometry_snapshot(true),
        &rays,
        0.00003,
    )?;
    let representation = compare(
        &mesh_world.static_geometry_snapshot(true),
        &native_world.static_geometry_snapshot(true),
        &rays,
        0.000002,
    )?;
    let mut floor_baseline = baseline.clone();
    floor_baseline.parts.retain(|p| p.id == "floor/two-quads");
    let mut floor_candidate = mesh.clone();
    floor_candidate.parts.retain(|p| p.id == "floor/two-quads");
    let floor = compare(
        &world(&floor_baseline)?.static_geometry_snapshot(true),
        &world(&floor_candidate)?.static_geometry_snapshot(true),
        &rays,
        0.00003,
    )?;
    let a = chassis(&floor_baseline, b)?;
    let c = chassis(&floor_candidate, b)?;
    let max_position = a
        .iter()
        .zip(&c)
        .map(|(a, b)| {
            a.iter()
                .zip(b)
                .map(|(a, b)| (a - b).powi(2))
                .sum::<f64>()
                .sqrt()
        })
        .fold(0., f64::max);
    ensure!(
        max_position < 0.005,
        "floor traversal differs by {max_position} m"
    );
    println!(
        "{}",
        serde_json::to_string_pretty(
            &serde_json::json!({"full_terrain":full,"mesh_vs_native_same_quality":representation,"isolated_floor":floor,"floor_chassis_ticks":768,"max_chassis_position_change_mm":max_position*1000.,"baseline_chassis_final_m":a.last(),"candidate_chassis_final_m":c.last(),"installed":false,"performance_claim":false})
        )?
    );
    Ok(())
}
