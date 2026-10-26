// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Inspect three profiles of the exported Energy rigid part, without changing a field.
use rm_sim_assets::collision_profiles::{ProfileRequest, Representation, load_part_profiles};
use rm_sim_physics::{
    Pose, WorldPhysics,
    collision_profiles::CollisionPurpose,
    runtime::{BodyKey, BodyState},
};
#[path = "collision_profiles/benchmark.rs"]
mod benchmark;
fn main() -> anyhow::Result<()> {
    let args: Vec<_> = std::env::args().collect();
    anyhow::ensure!(
        args.len() == 3 || (args.len() == 4 && args[3] == "--benchmark"),
        "usage: collision_profiles EXPORT_ROOT EXPECTED_MANIFEST_SHA256 [--benchmark]"
    );
    let requests = [
        ProfileRequest {
            purpose: CollisionPurpose::RigidBody,
            quality: "reviewed".into(),
            representation: Representation::Mesh,
            allow_experimental: false,
        },
        ProfileRequest {
            purpose: CollisionPurpose::Projectile,
            quality: "projectile-balanced".into(),
            representation: Representation::NativePreferred,
            allow_experimental: true,
        },
        ProfileRequest {
            purpose: CollisionPurpose::Lidar,
            quality: "lidar-coarse".into(),
            representation: Representation::Mesh,
            allow_experimental: true,
        },
    ];
    let prepared = load_part_profiles(
        std::path::Path::new(&args[1]),
        &args[2],
        "energy-unit/reviewed-0",
        &requests,
        0.4,
        0.45,
    )?;
    if args.len() == 4 {
        return benchmark::run(&prepared);
    }
    let mut world = WorldPhysics::new(&[], 0.);
    world
        .install_body(BodyKey(1), &prepared.model, BodyState::at(Pose::default()))
        .map_err(anyhow::Error::msg)?;
    let capture = world.static_geometry_snapshot(true);
    for (request, selected) in requests.iter().zip(&prepared.selected) {
        let geometry = capture.for_collision_purpose(request.purpose);
        let hit = geometry
            .cast_ray([0.08, 0.075, 0.], [-1., 0., 0.], 0.2)
            .map_err(anyhow::Error::msg)?;
        println!(
            "{}: {selected}, side hit distance {:?} m",
            request.purpose.as_str(),
            hit.map(|h| h.distance_m)
        );
        let socket = geometry
            .cast_ray([0., -0.001, 0.], [0., 1., 0.], 0.2)
            .map_err(anyhow::Error::msg)?
            .expect("Unit floor or filled socket");
        if request.purpose == CollisionPurpose::RigidBody {
            anyhow::ensure!(
                (socket.distance_m - 0.101).abs() < 1e-6,
                "detailed socket was lost"
            );
        } else {
            anyhow::ensure!(
                (socket.distance_m - 0.001).abs() < 1e-6,
                "coarse socket closure differs"
            );
        }
    }
    println!(
        "Detailed insertion bore retained; coarse profiles close it. {} original placements retained; preview uses product-local coordinates only.",
        prepared.placements_yup_m.len()
    );
    Ok(())
}
