// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Describe chassis source geometry and explicit policies before backend insertion.
//!
//! The descriptions contain no solver handles. Native pose arithmetic stays at
//! this adapter boundary so existing prescribed mounts retain their exact bits.
use super::*;
use crate::cooking::CookedRigidBody;
use rm_sim_engine::model::{
    BodyKind, BodyPolicy, CoefficientCombineRule as Combine, ColliderDescription, ColliderMass,
    ColliderPolicy, ContactMaterial, ConvexHull, RigidBodyDescription, ShapeDescription,
};
use std::sync::Arc;

/// Validated chassis configuration and cooked geometry reusable across robots/worlds.
///
/// Clones share the immutable configuration, source description, cooked shapes and
/// collider role indices. Each installation owns its controls, wheels and solver
/// handles independently; this recipe contains no live world state.
#[derive(Clone)]
pub struct PreparedChassis {
    pub(super) inner: Arc<PreparedChassisData>,
}

pub(super) struct PreparedChassisData {
    pub(super) config: ChassisConfig,
    pub(super) body: CookedRigidBody,
    pub(super) armor: Vec<usize>,
    pub(super) prescribed: Vec<usize>,
}

impl PreparedChassis {
    /// Validate an owned configuration and cook every shape before installation.
    /// Returns an error for invalid configuration or geometry without changing a world.
    ///
    /// ```
    /// use rm_sim_physics::chassis::{ChassisConfig, PreparedChassis};
    /// let prepared = PreparedChassis::new(ChassisConfig::default())?;
    /// let shared = prepared.clone();
    /// assert!(std::ptr::eq(prepared.config(), shared.config()));
    /// assert!(!prepared.source().colliders().is_empty());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn new(config: ChassisConfig) -> Result<Self, &'static str> {
        config.validate()?;
        let mut colliders = Vec::new();
        let mut prescribed = Vec::new();
        // Keep the original collider-derived mass aggregation, including zero-mass
        // turret policies; never replace it with an independently fitted inertial.
        let turret_share =
            if config.planar_flight || config.folding.is_some() || config.articulation.is_some() {
                0.
            } else {
                TURRET_MASS_SHARE
            };
        let [hx, hy, hz] = config.body_half_m;
        let deck = if config.omni_frame.is_some() {
            hull(
                config
                    .body_outline_m()
                    .iter()
                    .flat_map(|&[x, y]| [[x, y, -hz], [x, y, hz]])
                    .collect(),
            )?
        } else {
            cuboid([hx, hy, hz])
        };
        colliders.push(collider(
            deck,
            Pose::default(),
            BODY_FRICTION,
            ROBOT_RESTITUTION,
            ColliderMass::MassKg(config.mass_kg * (1. - turret_share)),
            Combine::Min,
        )?);
        if config.team_cad {
            // Source-envelope gameplay proxies, not visual triangles or mass estimates.
            for (half, centre) in [
                ([0.103, 0.103, 0.079], [0., 0., -0.030]),
                ([0.10, 0.07, 0.032], [0.145, 0.16, 0.077]),
                ([0.10, 0.07, 0.032], [-0.145, 0.16, 0.077]),
                ([0.10, 0.07, 0.032], [0.145, -0.16, 0.077]),
                ([0.10, 0.07, 0.032], [-0.145, -0.16, 0.077]),
            ] {
                colliders.push(collider(
                    cuboid(half),
                    Pose::at(centre),
                    BODY_FRICTION,
                    ROBOT_RESTITUTION,
                    ColliderMass::Zero,
                    Combine::Average,
                )?);
            }
        }
        for prism in config.bumper_prisms_m() {
            colliders.push(collider(
                hull(prism.to_vec())?,
                Pose::default(),
                BODY_FRICTION,
                ROBOT_RESTITUTION,
                ColliderMass::Zero,
                Combine::Min,
            )?);
        }
        let [tx, ty, tz] = config.turret_half_m;
        let [cx, cy, cz] = config.turret_center_m;
        let turret_top = cz + tz;
        let column_bottom = if config.planar_flight {
            (cz - tz).min(hz)
        } else {
            hz
        };
        let column_top = turret_top.max(hz);
        let column_half = (column_top - column_bottom) * 0.5;
        if let Some(folding) = &config.folding {
            for (pose, half) in folding.collision_parts(Pose::default(), [0., 0.], 0.) {
                prescribed.push(colliders.len());
                colliders.push(collider(
                    cuboid(half),
                    normalized_pose(pose),
                    BODY_FRICTION,
                    ROBOT_RESTITUTION,
                    ColliderMass::Zero,
                    Combine::Average,
                )?);
            }
        } else if config.articulation.is_none() {
            colliders.push(collider(
                cuboid([tx, ty, column_half]),
                Pose::at([cx, cy, column_bottom + column_half]),
                BODY_FRICTION,
                ROBOT_RESTITUTION,
                ColliderMass::MassKg(config.mass_kg * turret_share),
                Combine::Min,
            )?);
        }
        if config.articulation.is_some() {
            for (pose, half) in config.transform_collision_parts(Pose::default(), [0., 0.], 0.) {
                prescribed.push(colliders.len());
                colliders.push(collider(
                    cuboid(half),
                    normalized_pose(pose),
                    BODY_FRICTION,
                    0.,
                    ColliderMass::Zero,
                    Combine::Average,
                )?);
            }
        }
        if config.climbing.is_some() {
            // Preserve the empty climbing entry bay beneath the electronics shelf.
            colliders.push(collider(
                cuboid([0.05, 0.26, 0.006]),
                Pose::at([-0.075, 0., -0.16]),
                BODY_FRICTION,
                ROBOT_RESTITUTION,
                ColliderMass::Zero,
                Combine::Average,
            )?);
            for (pose, half) in config.transform_collision_parts(Pose::default(), [0., 0.], 0.) {
                prescribed.push(colliders.len());
                colliders.push(collider(
                    cuboid(half),
                    normalized_pose(pose),
                    BODY_FRICTION,
                    ROBOT_RESTITUTION,
                    ColliderMass::Zero,
                    Combine::Average,
                )?);
            }
        }
        let [ax, ay, az] = config.armor_housing_half_m();
        let mut armor = Vec::with_capacity(ARMOR_COUNT);
        for face in config.armor_faces() {
            // Match the old native multiply after normalizing the scoring-face
            // quaternion. Re-normalizing the product would change shape mounts.
            let local = rapier_pose(face) * Pose3::from_translation(Vector::new(-ax, 0., 0.));
            armor.push(colliders.len());
            colliders.push(collider(
                cuboid([ax, ay, az]),
                source_pose(local),
                FRICTION,
                ROBOT_RESTITUTION,
                ColliderMass::Zero,
                Combine::Min,
            )?);
        }
        for plate in 0..ARMOR_COUNT {
            for prism in config.armor_bracket_prisms_m(plate) {
                colliders.push(collider(
                    hull(prism.to_vec())?,
                    Pose::default(),
                    BODY_FRICTION,
                    ROBOT_RESTITUTION,
                    ColliderMass::Zero,
                    Combine::Average,
                )?);
            }
        }
        let source = RigidBodyDescription::new(BodyKind::DynamicFromColliders, colliders)?
            .with_policy(BodyPolicy {
                gravity_scale: if config.planar_flight { 0. } else { 1. },
                enabled_translations: [true, true, !config.planar_flight],
                enabled_rotations: [!config.planar_flight, !config.planar_flight, true],
                linear_damping: LINEAR_DAMPING,
                angular_damping: ANGULAR_DAMPING,
                can_sleep: false,
                ccd_enabled: false,
            })?;
        Ok(Self {
            inner: Arc::new(PreparedChassisData {
                config,
                body: CookedRigidBody::new(source)?,
                armor,
                prescribed,
            }),
        })
    }

    /// Immutable validated configuration used by each installation.
    pub fn config(&self) -> &ChassisConfig {
        &self.inner.config
    }

    /// Immutable backend-neutral body description in original collider order.
    pub fn source(&self) -> &RigidBodyDescription {
        self.inner.body.source()
    }

    /// Immutable cooked body whose shapes are shared by every installation.
    pub fn cooked(&self) -> &CookedRigidBody {
        &self.inner.body
    }
}

fn cuboid(half_extents_m: [f64; 3]) -> ShapeDescription {
    ShapeDescription::Cuboid { half_extents_m }
}
fn hull(points_m: Vec<[f64; 3]>) -> Result<ShapeDescription, &'static str> {
    Ok(ShapeDescription::ConvexHull(ConvexHull::new(points_m)?))
}
fn collider(
    shape: ShapeDescription,
    pose: Pose,
    friction: f64,
    restitution: f64,
    mass: ColliderMass,
    restitution_combine_rule: Combine,
) -> Result<ColliderDescription, &'static str> {
    ColliderDescription::new(
        shape,
        pose,
        ContactMaterial {
            friction,
            restitution,
            memberships: u32::MAX,
            filter: u32::MAX,
        },
    )?
    .with_policy(ColliderPolicy {
        mass,
        restitution_combine_rule,
        ..Default::default()
    })
}
fn normalized_pose(pose: Pose) -> Pose {
    source_pose(rapier_pose(pose))
}
fn source_pose(pose: Pose3) -> Pose {
    Pose {
        translation_m: pose.translation.to_array(),
        rotation_wxyz: [
            pose.rotation.w,
            pose.rotation.x,
            pose.rotation.y,
            pose.rotation.z,
        ],
    }
}
