// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Existing chassis regression tests, moved without changing their assertions.

use super::*;
use crate::projectile::{TargetFrames, WorldPhysics};
#[test]
fn source_mounts_settle_and_muzzle_follows_the_fitted_barrel() {
    let cfg = ChassisConfig::team_cad();
    cfg.validate().unwrap();
    let faces = cfg.armor_faces();
    assert!((faces[1].translation_m[2] - faces[0].translation_m[2] - 0.047).abs() < 1e-12);
    let mut world = WorldPhysics::new(&[], 0.);
    let id = world
        .add_chassis(
            Team::Red,
            cfg.clone(),
            Pose::at([0., 0., cfg.rest_height_m()]),
        )
        .unwrap();
    let frames = TargetFrames::new(Vec::new());
    for tick in 0..512 {
        world.step(tick * crate::tick_ns(), &frames).unwrap();
    }
    let c = &world.chassis_snapshots()[0];
    assert!((c.pose.translation_m[2] - 0.175_675_827).abs() < 0.003);
    assert!(c.wheels.iter().all(|w| w.contact.is_some()));
    let muzzle = world.chassis_muzzle_pose(id).unwrap();
    assert!((muzzle.translation_m[0] - 0.206_576_73).abs() < 0.002);
    assert!((muzzle.translation_m[2] - 0.461_125_827).abs() < 0.003);
    assert!(
        c.wheels
            .iter()
            .all(|w| w.angular_velocity_rad_s.abs() < 0.01)
    );
}
