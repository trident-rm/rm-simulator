// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Loader-built scene benchmarks; geometry preparation stays outside timed work.
use anyhow::Result;
use rm_sim_assets::collision_profiles::PreparedProfiles;
use rm_sim_physics::{
    PhysicalRuntime, Pose, StaticGeometry, StepDuration, WorldPhysics,
    collision_profiles::CollisionPurpose,
    cooking::CookedRigidBody,
    model::{
        BodyKind, BodyPolicy, ColliderDescription, ColliderMass, ColliderPolicy, ContactMaterial,
        RigidBodyDescription, ShapeDescription,
    },
    queries::RayQuery,
    runtime::{BodyBinding, BodyKey, BodyState},
};
use serde_json::json;
use std::{hint::black_box, time::Instant};
fn origin(i: usize) -> [f64; 3] {
    [(i % 4) as f64 * 0.3, 0., (i / 4) as f64 * 0.3]
}
fn baseline(prepared: &PreparedProfiles) -> Result<CookedRigidBody> {
    let source = &prepared.model.source().colliders()[0];
    let collider = ColliderDescription::new(
        source.shape().clone(),
        source.pose(),
        ContactMaterial {
            friction: 0.4,
            restitution: 0.45,
            memberships: u32::MAX,
            filter: u32::MAX,
        },
    )
    .map_err(anyhow::Error::msg)?;
    CookedRigidBody::new(
        RigidBodyDescription::new(BodyKind::Fixed, vec![collider]).map_err(anyhow::Error::msg)?,
    )
    .map_err(anyhow::Error::msg)
}
fn capture(model: &CookedRigidBody) -> Result<StaticGeometry> {
    let mut world = WorldPhysics::new(&[], 0.);
    for i in 0..12 {
        world
            .install_body(
                BodyKey(i as u64 + 1),
                model,
                BodyState::at(Pose::at(origin(i))),
            )
            .map_err(anyhow::Error::msg)?;
    }
    Ok(world.static_geometry_snapshot(true))
}
fn rays() -> Vec<RayQuery> {
    (0..16384)
        .map(|i| {
            let o = origin(i % 12);
            let angle = (i as f64 * 2.399963229728653).rem_euclid(std::f64::consts::TAU);
            let (p, d) = if i % 4 < 2 {
                let y = -0.01 + (i * 139 % 1021) as f64 / 1021. * 0.17;
                (
                    [o[0] + 0.08 * angle.cos(), y, o[2] + 0.08 * angle.sin()],
                    [-angle.cos(), 0., -angle.sin()],
                )
            } else {
                let r = (i * 43 % 991) as f64 / 991. * 0.055;
                (
                    [
                        o[0] + r * angle.cos(),
                        if i % 4 == 2 { -0.02 } else { 0.17 },
                        o[2] + r * angle.sin(),
                    ],
                    [0., if i % 4 == 2 { 1. } else { -1. }, 0.],
                )
            };
            RayQuery::new(p, d, 0.25).unwrap()
        })
        .collect()
}
fn actor(purpose: CollisionPurpose) -> Result<CookedRigidBody> {
    let shape = if purpose == CollisionPurpose::Projectile {
        ShapeDescription::Ball { radius_m: 0.0085 }
    } else {
        ShapeDescription::Cuboid {
            half_extents_m: [0.01; 3],
        }
    };
    let c = ColliderDescription::new(
        shape,
        Pose::default(),
        purpose
            .actor_material(0.4, 0.45)
            .map_err(anyhow::Error::msg)?,
    )
    .map_err(anyhow::Error::msg)?
    .with_policy(ColliderPolicy {
        mass: ColliderMass::MassKg(if purpose == CollisionPurpose::Projectile {
            0.0032
        } else {
            1.
        }),
        ..Default::default()
    })
    .map_err(anyhow::Error::msg)?;
    let source = RigidBodyDescription::new(BodyKind::DynamicFromColliders, vec![c])
        .map_err(anyhow::Error::msg)?
        .with_policy(BodyPolicy {
            gravity_scale: 0.,
            ccd_enabled: true,
            can_sleep: false,
            ..Default::default()
        })
        .map_err(anyhow::Error::msg)?;
    CookedRigidBody::new(source).map_err(anyhow::Error::msg)
}
fn scene(
    model: &CookedRigidBody,
    actor: &CookedRigidBody,
    purpose: CollisionPurpose,
) -> Result<(PhysicalRuntime, Vec<BodyBinding>)> {
    let mut world = PhysicalRuntime::new(StepDuration::HZ_128);
    let mut bindings = Vec::new();
    for i in 0..12 {
        let o = origin(i);
        world
            .install_body(BodyKey(i as u64 + 1), model, BodyState::at(Pose::at(o)))
            .map_err(anyhow::Error::msg)?;
        let count = if purpose == CollisionPurpose::Projectile {
            4
        } else {
            1
        };
        for j in 0..count {
            let pos = [o[0], 0.02 + j as f64 * 0.035, o[2] + 0.09];
            let mut state = BodyState::at(Pose::at(pos));
            state.velocity_m_s = [0., 0., -8.];
            bindings.push(
                world
                    .install_body(BodyKey(100 + i as u64 * 4 + j as u64), actor, state)
                    .map_err(anyhow::Error::msg)?,
            );
        }
    }
    Ok((world, bindings))
}
pub(super) fn run(prepared: &PreparedProfiles) -> Result<()> {
    let base = baseline(prepared)?;
    let models = [&base, &prepared.model];
    let names = ["current-mesh", "routed-profiles"];
    let captures = [capture(models[0])?, capture(models[1])?];
    let lidar = [
        captures[0].for_collision_purpose(CollisionPurpose::Lidar),
        captures[1].for_collision_purpose(CollisionPurpose::Lidar),
    ];
    let rigid = [
        captures[0].for_collision_purpose(CollisionPurpose::RigidBody),
        captures[1].for_collision_purpose(CollisionPurpose::RigidBody),
    ];
    let rays = rays();
    let mut detailed = vec![None; rays.len()];
    rigid[0]
        .cast_rays_into(&rays, &mut detailed)
        .map_err(anyhow::Error::msg)?;
    for (purpose, sets) in [("rigid", &rigid), ("lidar", &lidar)] {
        let mut values = vec![None; rays.len()];
        sets[1]
            .cast_rays_into(&rays, &mut values)
            .map_err(anyhow::Error::msg)?;
        let mut lost = 0;
        let mut added = 0;
        let mut max = 0_f64;
        for (a, b) in detailed.iter().zip(&values) {
            match (a, b) {
                (Some(_), None) => lost += 1,
                (None, Some(_)) => added += 1,
                (Some(a), Some(b)) => max = max.max((a.distance_m - b.distance_m).abs()),
                _ => {}
            }
        }
        if purpose == "rigid" {
            anyhow::ensure!(
                lost == 0 && added == 0 && max < 1e-12,
                "rigid query geometry changed"
            );
        }
        println!(
            "{}",
            json!({"kind":"quality","purpose":purpose,"rays":rays.len(),"lost_hits":lost,"added_hits":added,"max_range_change_m":max})
        );
    }
    for metric in ["nearest-rays", "visibility"] {
        for set in &lidar {
            let mut hits = vec![None; rays.len()];
            set.cast_rays_into(&rays, &mut hits)
                .map_err(anyhow::Error::msg)?;
        }
        for round in 0..9 {
            for index in if round % 2 == 0 { [0, 1] } else { [1, 0] } {
                let mut hits = vec![None; rays.len()];
                let t = Instant::now();
                let mut count = 0;
                for _ in 0..4 {
                    if metric == "nearest-rays" {
                        lidar[index]
                            .cast_rays_into(black_box(&rays), black_box(&mut hits))
                            .map_err(anyhow::Error::msg)?;
                        count += hits.iter().filter(|x| x.is_some()).count();
                    } else {
                        for ray in &rays {
                            let a = ray.origin_m();
                            let d = ray.direction();
                            let b = std::array::from_fn(|k| a[k] + d[k] * ray.max_distance_m());
                            count += usize::from(!lidar[index].segment_clear(a, b));
                        }
                    }
                }
                let ms = t.elapsed().as_secs_f64() * 1000.;
                black_box(count);
                println!(
                    "{}",
                    json!({"kind":"timing","metric":metric,"variant":names[index],"round":round,"ms":ms/4.,"queries":rays.len(),"hits":count/4})
                );
            }
        }
    }
    for purpose in [CollisionPurpose::Projectile, CollisionPurpose::RigidBody] {
        let actor = actor(purpose)?;
        let mut reference = None;
        for round in 0..7 {
            for index in if round % 2 == 0 { [0, 1] } else { [1, 0] } {
                let mut total = 0.;
                let mut final_positions = Vec::new();
                for _ in 0..8 {
                    let (mut world, bindings) = scene(models[index], &actor, purpose)?;
                    let t = Instant::now();
                    for tick in 0..64 {
                        world
                            .step(tick * StepDuration::HZ_128.as_ns())
                            .map_err(anyhow::Error::msg)?;
                    }
                    total += t.elapsed().as_secs_f64() * 1000.;
                    final_positions = bindings
                        .iter()
                        .map(|b| world.body_state(*b).unwrap().pose.translation_m)
                        .collect();
                }
                if purpose == CollisionPurpose::RigidBody {
                    if let Some(base) = &reference {
                        let reference: &Vec<[f64; 3]> = base;
                        let max = final_positions
                            .iter()
                            .zip(reference)
                            .flat_map(|(a, b)| (0..3).map(move |k| (a[k] - b[k]).abs()))
                            .fold(0., f64::max);
                        anyhow::ensure!(max < 1e-8, "rigid actor trajectories changed by {max} m");
                    } else {
                        reference = Some(final_positions.clone());
                    }
                }
                let checksum: f64 = final_positions.iter().flatten().copied().sum();
                black_box(checksum);
                println!(
                    "{}",
                    json!({"kind":"timing","metric":purpose.as_str(),"variant":names[index],"round":round,"ms":total,"ticks":512,"scenes":8,"units_per_scene":12,"actors_per_scene":final_positions.len(),"checksum":checksum})
                );
            }
        }
    }
    Ok(())
}
