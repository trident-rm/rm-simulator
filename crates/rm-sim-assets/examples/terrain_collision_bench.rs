// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Same-quality native-heightfield versus mesh benchmark, no match/render loop.
use rm_sim_assets::{
    math::{quat_axis_angle, rotate},
    terrain_collision::{Quality, Representation, TerrainPackage},
};
use rm_sim_physics::{
    PhysicalRuntime, Pose, StepDuration, TargetFrames, Team, WorldPhysics,
    chassis::{ChassisCommand, ChassisConfig},
    cooking::CookedRigidBody,
    queries::{RayHit, RayQuery},
    runtime::BodyKey,
};
use sim_engine::model::{
    BodyKind, BodyState, ColliderDescription, ContactMaterial, Inertial, RigidBodyDescription,
    ShapeDescription,
};
use std::{hint::black_box, path::Path, time::Instant};
fn material() -> ContactMaterial {
    ContactMaterial {
        friction: 0.6,
        restitution: 0.,
        memberships: u32::MAX,
        filter: u32::MAX,
    }
}
fn scene(p: &TerrainPackage) -> PhysicalRuntime {
    let mut r = PhysicalRuntime::new(StepDuration::HZ_1000);
    p.install(&mut r).unwrap();
    r
}
fn rays(bounds: [f64; 4], count: usize) -> Vec<RayQuery> {
    (0..count)
        .map(|i| {
            let u = ((i * 7919) % count) as f64 / count as f64;
            let v = ((i * 3571 + 17) % count) as f64 / count as f64;
            let x = bounds[0] + (0.01 + u * 0.98) * (bounds[1] - bounds[0]);
            let y = bounds[2] + (0.01 + v * 0.98) * (bounds[3] - bounds[2]);
            RayQuery::new([x, y, 3.], [0., 0., -1.], 6.).unwrap()
        })
        .collect()
}
fn query(r: &PhysicalRuntime, rays: &[RayQuery], repeats: usize) -> (f64, Vec<Option<RayHit>>) {
    let geometry = r.geometry_capture();
    let mut hits = vec![None; rays.len()];
    let mut times = Vec::new();
    for round in 0..repeats + 1 {
        let start = Instant::now();
        geometry.cast_rays_into(rays, &mut hits).unwrap();
        let elapsed = start.elapsed().as_secs_f64() * 1000.;
        black_box(&hits);
        if round > 0 {
            times.push(elapsed);
        }
    }
    times.sort_by(f64::total_cmp);
    (times[times.len() / 2], hits)
}
fn balls(p: &TerrainPackage, bounds: [f64; 4]) -> f64 {
    let mut r = scene(p);
    let shape = ColliderDescription::new(
        ShapeDescription::Ball { radius_m: 0.03 },
        Pose::default(),
        material(),
    )
    .unwrap();
    let model = CookedRigidBody::new(
        RigidBodyDescription::new(
            BodyKind::Dynamic(Inertial::new(0.1, [0.000036; 3], Pose::default()).unwrap()),
            vec![shape],
        )
        .unwrap(),
    )
    .unwrap();
    for i in 0..32 {
        r.install_body(
            BodyKey(i + 1),
            &model,
            BodyState::at(Pose::at([
                bounds[0] + (0.05 + (i % 8) as f64 / 8. * 0.9) * (bounds[1] - bounds[0]),
                bounds[2] + (0.1 + (i / 8) as f64 / 4. * 0.8) * (bounds[3] - bounds[2]),
                1.,
            ])),
        )
        .unwrap();
    }
    let start = Instant::now();
    for tick in 0..1500 {
        r.step(tick * 1_000_000).unwrap();
    }
    black_box(r.geometry_capture());
    start.elapsed().as_secs_f64() * 1000.
}
fn transform(pose: Pose, p: [f64; 3]) -> [f64; 3] {
    let q = rotate(pose.rotation_wxyz, p);
    std::array::from_fn(|i| q[i] + pose.translation_m[i])
}
fn bounds(package: &TerrainPackage, id: &str) -> [f64; 4] {
    let part = package.parts.iter().find(|p| p.id == id).unwrap();
    let collider = part
        .model
        .source()
        .colliders()
        .iter()
        .find(|c| matches!(c.shape(), ShapeDescription::HeightField(_)))
        .or_else(|| part.model.source().colliders().first())
        .unwrap();
    let vertices = match collider.shape() {
        ShapeDescription::HeightField(grid) => grid.to_triangle_mesh().vertices_m().to_vec(),
        ShapeDescription::TriangleMesh(mesh) => mesh.vertices_m().to_vec(),
        _ => panic!("terrain bounds require a finite surface"),
    };
    let mut b = [
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ];
    for p in &vertices {
        let p = transform(part.placements[0], transform(collider.pose(), *p));
        b[0] = b[0].min(p[0]);
        b[1] = b[1].max(p[0]);
        b[2] = b[2].min(p[1]);
        b[3] = b[3].max(p[1]);
    }
    b
}
fn static_query(package: &TerrainPackage, rays: &[RayQuery]) -> f64 {
    let mut world = WorldPhysics::new(&[], -10.);
    world.remove_catch_floor();
    package.install_in_world(&mut world).unwrap();
    let geometry = world.static_geometry_snapshot(true);
    let mut hits = vec![None; rays.len()];
    let mut times = Vec::new();
    for round in 0..6 {
        let start = Instant::now();
        geometry.cast_rays_into(rays, &mut hits).unwrap();
        if round > 0 {
            times.push(start.elapsed().as_secs_f64() * 1000.);
        }
        black_box(&hits);
    }
    times.sort_by(f64::total_cmp);
    times[2]
}
fn chassis(package: &TerrainPackage, bounds: [f64; 4]) -> (f64, [f64; 3]) {
    let mut world = WorldPhysics::with_step_duration(&[], -10., StepDuration::HZ_1000);
    world.remove_catch_floor();
    package.install_in_world(&mut world).unwrap();
    let x = bounds[0] + 0.25;
    let y = (bounds[2] + bounds[3]) / 2.;
    let ground = world
        .static_geometry_snapshot(true)
        .cast_ray([x, y, 3.], [0., 0., -1.], 6.)
        .unwrap()
        .unwrap();
    let config = ChassisConfig::default();
    let z = 3. - ground.distance_m + config.rest_height_m();
    let id = world
        .add_chassis(Team::Red, config, Pose::at([x, y, z]))
        .unwrap();
    world
        .command_chassis(
            id,
            ChassisCommand {
                forward_m_s: 0.5,
                ..Default::default()
            },
        )
        .unwrap();
    let frames = TargetFrames::new(vec![]);
    let start = Instant::now();
    for tick in 0..3000 {
        world.step(tick * 1_000_000, &frames).unwrap();
    }
    let elapsed = start.elapsed().as_secs_f64() * 1000.;
    let state = black_box(world.chassis_snapshot(id).unwrap());
    assert!(state.pose.translation_m.iter().all(|v| v.is_finite()));
    (elapsed, state.pose.translation_m)
}
fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(f64::total_cmp);
    values[values.len() / 2]
}
fn main() -> anyhow::Result<()> {
    let args = std::env::args().collect::<Vec<_>>();
    let root = Path::new(args.get(1).ok_or_else(|| {
        anyhow::anyhow!(
            "usage: terrain_collision_bench PACKAGE [coarse|fast|balanced|precise] [ray-count]"
        )
    })?);
    let quality = match args.get(2).map(String::as_str).unwrap_or("coarse") {
        "coarse" => Quality::Coarse,
        "fast" => Quality::Fast,
        "balanced" => Quality::Balanced,
        "precise" => Quality::Precise,
        _ => anyhow::bail!("invalid explicit quality"),
    };
    let count = args
        .get(3)
        .map(|v| v.parse())
        .transpose()?
        .unwrap_or(50_000);
    let root_pose = Pose {
        translation_m: [0., -1.6243436765, 1.641343676495],
        rotation_wxyz: quat_axis_angle([1., 0., 0.], std::f64::consts::FRAC_PI_2),
    };
    let mut packages = Vec::new();
    let mut results = Vec::new();
    let mut checks = Vec::new();
    let mut load_times = Vec::new();
    let order = if args.get(4).is_some_and(|s| s == "native-first") {
        [Representation::NativePreferred, Representation::Mesh]
    } else {
        [Representation::Mesh, Representation::NativePreferred]
    };
    anyhow::ensure!(count > 0, "ray count must be positive");
    for mode in order {
        let start = Instant::now();
        let package = TerrainPackage::load(root, quality, mode, root_pose, material())?;
        let load_ms = start.elapsed().as_secs_f64() * 1000.;
        load_times.push((mode, load_ms));
        packages.push(package);
    }
    let native_index = load_times
        .iter()
        .position(|(m, _)| *m == Representation::NativePreferred)
        .unwrap();
    let wave_bounds = bounds(&packages[native_index], "red-road-surface/shell-3797315");
    let field_bounds = bounds(&packages[native_index], "floor/two-quads");
    let field_rays = rays(field_bounds, count);
    let wave_rays = rays(wave_bounds, count);
    for (index, package) in packages.iter().enumerate() {
        let (mode, load_ms) = load_times[index];
        let start = Instant::now();
        let runtime = scene(package);
        let install_ms = start.elapsed().as_secs_f64() * 1000.;
        let (field_ms, field_hits) = query(&runtime, &field_rays, 5);
        let (wave_ms, wave_hits) = query(&runtime, &wave_rays, 5);
        let mut isolated = package.clone();
        isolated
            .parts
            .retain(|p| p.id == "red-road-surface/shell-3797315");
        let isolated_ms = query(&scene(&isolated), &wave_rays, 5).0;
        let sphere_ms = median((0..3).map(|_| balls(package, wave_bounds)).collect());
        let chassis_runs = (0..3)
            .map(|_| chassis(package, wave_bounds))
            .collect::<Vec<_>>();
        let chassis_ms = median(chassis_runs.iter().map(|v| v.0).collect());
        let chassis_final_position = chassis_runs[0].1;
        checks.push((field_hits, wave_hits));
        results.push(serde_json::json!({"representation":format!("{mode:?}"),"quality":quality.as_str(),"parts":package.parts.len(),"native_heightfields":package.native_heightfields,
          "rays_per_batch":count,"load_and_cook_ms":load_ms,"install_ms":install_ms,
          "physical_capture_field_rays_median_ms":field_ms,"physical_capture_wave_rays_median_ms":wave_ms,
          "static_capture_field_rays_median_ms":static_query(package,&field_rays),"static_capture_wave_rays_median_ms":static_query(package,&wave_rays),
          "isolated_wave_capture_rays_median_ms":isolated_ms,
          "sphere_scene_32_balls_1500_ticks_median_ms":sphere_ms,"chassis_3000_ticks_median_ms":chassis_ms,"chassis_final_position_m":chassis_final_position}));
    }
    let mut exact_mesh = packages[native_index].clone();
    for part in &mut exact_mesh.parts {
        let colliders = part
            .model
            .source()
            .colliders()
            .iter()
            .map(|c| {
                let shape = match c.shape() {
                    ShapeDescription::HeightField(g) => {
                        ShapeDescription::TriangleMesh(g.to_triangle_mesh())
                    }
                    x => x.clone(),
                };
                ColliderDescription::new(shape, c.pose(), c.material())
                    .unwrap()
                    .with_policy(c.policy())
                    .unwrap()
            })
            .collect();
        part.model =
            CookedRigidBody::new(RigidBodyDescription::new(BodyKind::Fixed, colliders).unwrap())
                .unwrap();
    }
    let exact_chassis = chassis(&exact_mesh, wave_bounds);
    let mut isolated = exact_mesh.clone();
    isolated
        .parts
        .retain(|p| p.id == "red-road-surface/shell-3797315");
    results.push(serde_json::json!({"representation":"ExactGridMeshControl","chassis_3000_ticks_ms":exact_chassis.0,"chassis_final_position_m":exact_chassis.1,
       "isolated_wave_capture_rays_median_ms":query(&scene(&isolated),&wave_rays,5).0,
       "sphere_scene_32_balls_1500_ticks_median_ms":median((0..3).map(|_|balls(&exact_mesh,wave_bounds)).collect()),
       "static_capture_wave_rays_median_ms":static_query(&exact_mesh,&wave_rays)}));
    for (name, quantize, combine) in [
        ("SplitFloat32Control", true, false),
        ("CombinedFloat64Control", false, true),
    ] {
        let mut control = exact_mesh.clone();
        for part in &mut control.parts {
            let mut colliders = Vec::new();
            let mut combined_points = Vec::new();
            let mut combined_triangles = Vec::new();
            let originals = part.model.source().colliders();
            for c in originals {
                let ShapeDescription::TriangleMesh(mesh) = c.shape() else {
                    unreachable!()
                };
                let points = mesh
                    .vertices_m()
                    .iter()
                    .map(|p| {
                        let p = transform(c.pose(), *p);
                        if quantize {
                            p.map(|v| v as f32 as f64)
                        } else {
                            p
                        }
                    })
                    .collect::<Vec<_>>();
                if combine {
                    let base = combined_points.len() as u32;
                    combined_points.extend(points);
                    combined_triangles.extend(mesh.triangles().iter().map(|t| t.map(|i| i + base)));
                } else {
                    colliders.push(
                        ColliderDescription::new(
                            ShapeDescription::TriangleMesh(
                                sim_engine::model::TriangleMesh::new(
                                    points,
                                    mesh.triangles().to_vec(),
                                )
                                .unwrap(),
                            ),
                            Pose::default(),
                            c.material(),
                        )
                        .unwrap()
                        .with_policy(c.policy())
                        .unwrap(),
                    );
                }
            }
            if combine {
                colliders.push(
                    ColliderDescription::new(
                        ShapeDescription::TriangleMesh(
                            sim_engine::model::TriangleMesh::new(
                                combined_points,
                                combined_triangles,
                            )
                            .unwrap(),
                        ),
                        Pose::default(),
                        originals[0].material(),
                    )
                    .unwrap()
                    .with_policy(originals[0].policy())
                    .unwrap(),
                );
            }
            part.model = CookedRigidBody::new(
                RigidBodyDescription::new(BodyKind::Fixed, colliders).unwrap(),
            )
            .unwrap();
        }
        let c = chassis(&control, wave_bounds);
        results.push(serde_json::json!({"representation":name,"chassis_3000_ticks_ms":c.0,"chassis_final_position_m":c.1}));
    }
    let mut mismatches = 0;
    let mut maximum_distance_difference = 0f64;
    let mut maximum_normal_angle = 0f64;
    for (a, b) in checks[0]
        .0
        .iter()
        .zip(&checks[1].0)
        .chain(checks[0].1.iter().zip(&checks[1].1))
    {
        match (a, b) {
            (Some(a), Some(b)) => {
                maximum_distance_difference =
                    maximum_distance_difference.max((a.distance_m - b.distance_m).abs());
                let dot = a
                    .normal
                    .iter()
                    .zip(b.normal)
                    .map(|(a, b)| a * b)
                    .sum::<f64>();
                maximum_normal_angle =
                    maximum_normal_angle.max(dot.clamp(-1., 1.).acos().to_degrees())
            }
            (None, None) => {}
            _ => mismatches += 1,
        }
    }
    println!(
        "{}",
        serde_json::to_string_pretty(
            &serde_json::json!({"results":results,"query_hit_mismatches":mismatches,"maximum_query_distance_difference_m":maximum_distance_difference,"maximum_query_normal_difference_degrees":maximum_normal_angle,"limits":"Local microbenchmark; same coarse geometry and caller material. Not full simulator FPS; sphere scene has no chassis/renderer/gameplay. Native and mesh solver contacts need not produce identical trajectories."})
        )?
    );
    anyhow::ensure!(
        mismatches == 0 && maximum_distance_difference < 0.0001,
        "native/mesh query equivalence failed"
    );
    Ok(())
}
