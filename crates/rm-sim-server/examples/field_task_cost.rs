// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Rest-pose field microbenchmarks, with preparation excluded from timed work.
//! Usage: field_task_cost ROUNDS COMMON_PACKAGE LABEL PATH [LABEL PATH ...]. Paths are hydrated
//! field packages or benchmark-only RMCAD01 binary source meshes in world FLU.
use anyhow::{Context, Result, ensure};
use rm_sim_physics::{
    Pose, StaticGeometry, StepDuration, TargetFrames, WorldPhysics,
    collision_profiles::CollisionPurpose,
    cooking::CookedRigidBody,
    model::{
        BodyKind, BodyPolicy, ColliderDescription, ColliderMass, ColliderPolicy,
        RigidBodyDescription, ShapeDescription,
    },
    runtime::{BodyKey, BodyState},
    sensors::{Lidar, LidarConfig, LidarFrame},
};
use rm_sim_server::{cad_assets, layout};
use rm_sim_world::Field;
use serde_json::json;
use std::{
    fs::File,
    hint::black_box,
    io::{BufReader, Read},
    path::Path,
    time::Instant,
};

fn mesh(path: &Path, common: &Path) -> Result<StaticGeometry> {
    let mut input = BufReader::new(File::open(path)?);
    let mut magic = [0; 8];
    input.read_exact(&mut magic)?;
    ensure!(&magic == b"RMCAD01\0", "invalid mesh header");
    let mut points = Vec::new();
    let mut triangles = Vec::new();
    loop {
        let mut sizes = [0; 16];
        match input.read(&mut sizes[..1])? {
            0 => break,
            _ => input.read_exact(&mut sizes[1..])?,
        }
        let nv = usize::try_from(u64::from_le_bytes(sizes[..8].try_into()?))?;
        let nt = usize::try_from(u64::from_le_bytes(sizes[8..].try_into()?))?;
        ensure!(
            nv <= 100_000_000 && nt <= 100_000_000,
            "oversized mesh chunk"
        );
        let offset = u32::try_from(points.len())?;
        let mut raw = vec![0; nv.checked_mul(24).context("vertex overflow")?];
        input.read_exact(&mut raw)?;
        for vertex in raw.as_chunks::<24>().0 {
            let mut p = [0.; 3];
            for (j, value) in p.iter_mut().enumerate() {
                *value = f64::from_le_bytes(vertex[j * 8..j * 8 + 8].try_into()?);
            }
            points.push(p);
        }
        raw.resize(nt.checked_mul(12).context("index overflow")?, 0);
        input.read_exact(&mut raw)?;
        for triangle in raw.as_chunks::<12>().0 {
            let mut t = [0; 3];
            for (j, value) in t.iter_mut().enumerate() {
                let index = u32::from_le_bytes(triangle[j * 4..j * 4 + 4].try_into()?);
                ensure!((index as usize) < nv, "invalid source index");
                *value = offset.checked_add(index).context("index overflow")?;
            }
            triangles.push(t);
        }
    }
    eprintln!(
        "full CAD: {} vertices, {} triangles",
        points.len(),
        triangles.len()
    );
    let mut world = WorldPhysics::new(&[], 0.);
    world.remove_catch_floor();
    world
        .add_static_mesh(points, triangles)
        .map_err(anyhow::Error::msg)?;
    let cad = cad_assets::load(common)?;
    let terrain = layout::load_terrain(&cad)?;
    world
        .add_boundary_mesh(terrain.boundary.vertices_m, terrain.boundary.triangles)
        .map_err(anyhow::Error::msg)?;
    Ok(world.static_geometry_snapshot(true))
}

fn load(path: &Path, common: &Path) -> Result<StaticGeometry> {
    if path.is_file() {
        return mesh(path, common);
    }
    let cad = cad_assets::load(path)?;
    let terrain = layout::load_terrain(&cad)?;
    let mut field = Field::new(&layout::field_config(
        &cad,
        &layout::LayoutOptions {
            rune: None,
            outpost_speed_rad_s: 0.,
            terrain: true,
            referee: false,
            projectile_policy: Default::default(),
        },
    ))?;
    layout::add_terrain(&mut field, &terrain)?;
    eprintln!("{}", terrain.describe());
    Ok(field.static_geometry_snapshot())
}

fn actor(projectile: bool) -> Result<CookedRigidBody> {
    let purpose = if projectile {
        CollisionPurpose::Projectile
    } else {
        CollisionPurpose::RigidBody
    };
    let collider = ColliderDescription::new(
        if projectile {
            ShapeDescription::Ball { radius_m: 0.0085 }
        } else {
            ShapeDescription::Cuboid {
                half_extents_m: [0.12, 0.12, 0.08],
            }
        },
        Pose::default(),
        purpose
            .actor_material(0.4, 0.45)
            .map_err(anyhow::Error::msg)?,
    )
    .map_err(anyhow::Error::msg)?
    .with_policy(ColliderPolicy {
        mass: ColliderMass::MassKg(if projectile { 0.0032 } else { 1. }),
        ..Default::default()
    })
    .map_err(anyhow::Error::msg)?;
    let body = RigidBodyDescription::new(BodyKind::DynamicFromColliders, vec![collider])
        .map_err(anyhow::Error::msg)?
        .with_policy(BodyPolicy {
            ccd_enabled: projectile,
            can_sleep: false,
            gravity_scale: if projectile { 0. } else { 1. },
            ..Default::default()
        })
        .map_err(anyhow::Error::msg)?;
    CookedRigidBody::new(body).map_err(anyhow::Error::msg)
}

fn stepped(
    geometry: &StaticGeometry,
    model: &CookedRigidBody,
    projectile: bool,
) -> Result<(f64, f64)> {
    let mut world = WorldPhysics::with_step_duration(&[], 0., StepDuration::HZ_128);
    world
        .try_insert_geometry(geometry)
        .map_err(anyhow::Error::msg)?;
    let mut bodies = Vec::new();
    for i in 0..96 {
        let p = [
            -12. + (i % 16) as f64 * 1.6,
            -6. + (i / 16) as f64 * 2.4,
            if projectile {
                0.35 + (i % 3) as f64 * 0.6
            } else {
                0.7
            },
        ];
        let mut state = BodyState::at(Pose::at(p));
        if projectile {
            state.velocity_m_s = [
                if i % 2 == 0 { 25. } else { -25. },
                if i % 4 < 2 { 4. } else { -4. },
                0.,
            ];
        }
        bodies.push(
            world
                .install_body(BodyKey(i + 1), model, state)
                .map_err(anyhow::Error::msg)?,
        );
    }
    let frames = TargetFrames::new(vec![]);
    let mut contacts = Vec::new();
    for tick in 0..32 {
        world
            .step_into(tick * StepDuration::HZ_128.as_ns(), &frames, &mut contacts)
            .map_err(anyhow::Error::msg)?;
        contacts.clear();
    }
    let start = Instant::now();
    for tick in 32..160 {
        world
            .step_into(tick * StepDuration::HZ_128.as_ns(), &frames, &mut contacts)
            .map_err(anyhow::Error::msg)?;
        black_box(&contacts);
        contacts.clear();
    }
    let elapsed = start.elapsed().as_secs_f64() * 1000.;
    let checksum = bodies
        .iter()
        .map(|b| {
            world
                .body_state(*b)
                .unwrap()
                .pose
                .translation_m
                .iter()
                .sum::<f64>()
        })
        .sum();
    Ok((elapsed, checksum))
}

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let rounds: usize = args.next().context("ROUNDS required")?.parse()?;
    ensure!(rounds > 0, "rounds must be positive");
    let common = args.next().context("COMMON_PACKAGE required")?;
    let mut scenes = Vec::new();
    while let Some(label) = args.next() {
        let path = args.next().context("PATH required after label")?;
        let start = Instant::now();
        let geometry = load(Path::new(&path), Path::new(&common))?;
        eprintln!("prepared {label} in {:.3}s", start.elapsed().as_secs_f64());
        scenes.push((label, geometry));
    }
    ensure!(!scenes.is_empty(), "at least one scene required");
    let mut config = LidarConfig::planar(1800, 0.05, 30.);
    config.elevations_rad = (0..16)
        .map(|i| (-15. + 2. * i as f64).to_radians())
        .collect();
    let lidar = Lidar::new(config).map_err(anyhow::Error::msg)?;
    let origins = [[0., 0., 0.35], [4.7, 3.5, 0.35], [-6.7, -3.5, 0.35]];
    let shot = actor(true)?;
    let rigid = actor(false)?;
    // Warm BVHs and query buffers before any reported sample.
    for (_, geometry) in &scenes {
        let mut frame = LidarFrame::default();
        lidar
            .sample_into(geometry, Pose::at(origins[0]), 0, &mut frame)
            .map_err(anyhow::Error::msg)?;
    }
    for round in 0..rounds {
        for offset in 0..scenes.len() {
            let (label, geometry) = &scenes[(round + offset) % scenes.len()];
            let mut frame = LidarFrame::default();
            lidar
                .sample_into(geometry, Pose::at(origins[0]), 0, &mut frame)
                .map_err(anyhow::Error::msg)?;
            let start = Instant::now();
            let mut returns = 0;
            for origin in origins {
                lidar
                    .sample_into(geometry, Pose::at(origin), 0, &mut frame)
                    .map_err(anyhow::Error::msg)?;
                returns += frame.returns.iter().flatten().count();
                black_box(&frame);
            }
            println!(
                "{}",
                json!({"round":round,"variant":label,"task":"lidar_86400_rays","ms":start.elapsed().as_secs_f64()*1000.,"checksum":returns})
            );
            let start = Instant::now();
            let mut blocked = 0;
            for i in 0..4096 {
                let p = [
                    -13. + (i % 64) as f64 * 26. / 63.,
                    -7. + ((i / 64) % 32) as f64 * 14. / 31.,
                    if i < 2048 { 0.02 } else { 0.4 },
                ];
                blocked += usize::from(!geometry.sphere_clear(black_box(p), 0.06));
            }
            println!(
                "{}",
                json!({"round":round,"variant":label,"task":"sphere_overlap_4096","ms":start.elapsed().as_secs_f64()*1000.,"checksum":blocked})
            );
            for (task, model, projectile) in [
                ("ccd_96_spheres_128_ticks", &shot, true),
                ("contact_96_boxes_128_ticks", &rigid, false),
            ] {
                let (ms, checksum) = stepped(geometry, model, projectile)?;
                println!(
                    "{}",
                    json!({"round":round,"variant":label,"task":task,"ms":ms,"checksum":checksum})
                );
            }
        }
    }
    Ok(())
}
