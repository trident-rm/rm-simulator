// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Domain-owned source descriptions; native construction uses shared cooking.
use super::{Caliber, FRICTION, RESTITUTION};
use crate::{Pose, cooking::CookedRigidBody};
use rm_simulator_engine::model::{
    BodyKind, BodyPolicy, CoefficientCombineRule, ColliderDescription, ColliderMass,
    ColliderPolicy, ContactMaterial, RigidBodyDescription, ShapeDescription,
};

/// Bounded per-world preparation: one current restitution per caliber. Existing
/// colliders retain their own shared shape and immutable contact coefficients.
#[derive(Default)]
pub(super) struct ProjectileCache {
    calibers: [Option<(u64, CookedRigidBody)>; 2],
}
impl ProjectileCache {
    pub(super) fn get(&mut self, caliber: Caliber, restitution: f64) -> &CookedRigidBody {
        let index = match caliber {
            Caliber::Mm17 => 0,
            Caliber::Mm42 => 1,
        };
        let slot = &mut self.calibers[index];
        if slot
            .as_ref()
            .is_none_or(|(key, _)| *key != restitution.to_bits())
        {
            *slot = Some((restitution.to_bits(), projectile(caliber, restitution)));
        }
        &slot.as_ref().expect("projectile cache populated").1
    }
}

fn material(restitution: f64, memberships: u32) -> ContactMaterial {
    ContactMaterial {
        friction: FRICTION,
        restitution,
        memberships,
        filter: u32::MAX,
    }
}

fn projectile(caliber: Caliber, restitution: f64) -> CookedRigidBody {
    let collider = ColliderDescription::new(
        ShapeDescription::Ball {
            radius_m: caliber.diameter_m() / 2.,
        },
        Pose::default(),
        material(restitution, 2),
    )
    .expect("nominal projectile geometry and validated restitution")
    .with_policy(ColliderPolicy {
        mass: ColliderMass::MassKg(caliber.mass_kg()),
        restitution_combine_rule: CoefficientCombineRule::Min,
        ..Default::default()
    })
    .expect("nominal projectile mass");
    let source = RigidBodyDescription::new(BodyKind::DynamicFromColliders, vec![collider])
        .expect("projectile collider supplies its mass")
        .with_policy(BodyPolicy {
            ccd_enabled: true,
            can_sleep: false,
            ..Default::default()
        })
        .expect("projectile integration policy");
    CookedRigidBody::new(source).expect("nominal projectile is representable")
}

pub(super) fn target(half_extents_m: [f64; 3]) -> CookedRigidBody {
    let collider = ColliderDescription::new(
        ShapeDescription::Cuboid { half_extents_m },
        Pose::at([-half_extents_m[0], 0., 0.]),
        material(RESTITUTION, u32::MAX),
    )
    .expect("nominal target geometry")
    .with_policy(ColliderPolicy {
        mass: ColliderMass::DensityKgM3(1.),
        ..Default::default()
    })
    .expect("native default target density");
    let source = RigidBodyDescription::new(BodyKind::Kinematic, vec![collider])
        .expect("kinematic target retains native collider density")
        .with_policy(BodyPolicy {
            can_sleep: false,
            ..Default::default()
        })
        .expect("target integration policy");
    CookedRigidBody::new(source).expect("nominal target is representable")
}
