// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Opt-in collision-purpose policies. Legacy worlds keep their existing groups.
use crate::StaticGeometry;
use sim_engine::model::ContactMaterial;

const RIGID_ACTOR: u32 = 1;
const PROJECTILE_ACTOR: u32 = 2;
const RIGID_SURFACE: u32 = 1 << 3;
const PROJECTILE_SURFACE: u32 = 1 << 4;
const LIDAR_SURFACE: u32 = 1 << 5;

/// Which physical or query workload consumes a collision representation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CollisionPurpose {
    /// Detailed contact geometry for chassis, carried objects and other rigid bodies.
    RigidBody,
    /// Sphere/projectile contacts and continuous shape casts.
    Projectile,
    /// Read-only nearest-ray geometry; never contributes contact forces.
    Lidar,
}
impl CollisionPurpose {
    /// Stable exported profile key.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::RigidBody => "rigid-body",
            Self::Projectile => "projectile",
            Self::Lidar => "lidar",
        }
    }
    fn surface(self) -> u32 {
        match self {
            Self::RigidBody => RIGID_SURFACE,
            Self::Projectile => PROJECTILE_SURFACE,
            Self::Lidar => LIDAR_SURFACE,
        }
    }
    /// Material and contact masks for one explicitly profiled scenery surface.
    /// LiDAR surfaces have a zero contact filter and remain query-only. Actors
    /// must use [`Self::actor_material`]; default ALL memberships would opt an
    /// actor into multiple surface profiles. Bits 0, 1, 3, 4 and 5 are reserved
    /// by this standalone opt-in policy. This does not change legacy field groups.
    pub fn scenery_material(self, friction: f64, restitution: f64) -> ContactMaterial {
        ContactMaterial {
            friction,
            restitution,
            memberships: self.surface(),
            filter: match self {
                Self::RigidBody => RIGID_ACTOR,
                Self::Projectile => PROJECTILE_ACTOR,
                Self::Lidar => 0,
            },
        }
    }
    /// Actor masks that admit exactly one scenery representation, while retaining
    /// rigid/rigid, rigid/projectile and projectile/projectile interactions.
    /// LiDAR has no physical actor and returns an error. Keep caller-selected
    /// contact coefficients; construction of the body validates their values.
    pub fn actor_material(
        self,
        friction: f64,
        restitution: f64,
    ) -> Result<ContactMaterial, &'static str> {
        let memberships = match self {
            Self::RigidBody => RIGID_ACTOR,
            Self::Projectile => PROJECTILE_ACTOR,
            Self::Lidar => return Err("LiDAR profiles have no physical actor"),
        };
        Ok(ContactMaterial {
            friction,
            restitution,
            memberships,
            filter: RIGID_ACTOR | PROJECTILE_ACTOR | self.surface(),
        })
    }
}
impl StaticGeometry {
    /// Select one purpose's profiled shapes for rays, sweeps and clearance tests.
    /// Unprofiled legacy shapes remain a common fallback. Buffers are shared,
    /// motion indices are remapped, and transforms retain their captured values.
    /// The returned capture cannot double-query another purpose's surface.
    pub fn for_collision_purpose(&self, purpose: CollisionPurpose) -> Self {
        let profiled = RIGID_SURFACE | PROJECTILE_SURFACE | LIDAR_SURFACE;
        let mut out = Self {
            no_catch_floor: self.no_catch_floor,
            projectile_bounds_m: self.projectile_bounds_m,
            ..Self::default()
        };
        for (index, shape) in self.shapes.iter().enumerate() {
            let groups = self
                .collision_groups
                .get(index)
                .copied()
                .unwrap_or_default();
            let bits = groups.memberships.bits();
            // Only exact profile memberships are classified. Legacy ALL groups
            // and unrelated custom group policies remain common geometry.
            if bits & profiled != 0 && bits & !profiled == 0 && bits & purpose.surface() == 0 {
                continue;
            }
            let next = out.shapes.len();
            out.shapes.push(shape.clone());
            out.collision_groups.push(groups);
            out.mechanisms.extend(
                self.mechanisms
                    .iter()
                    .filter(|(i, ..)| *i == index)
                    .map(|(_, channel, frame)| (next, *channel, *frame)),
            );
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rapier3d_f64::prelude::*;
    fn groups(material: ContactMaterial) -> InteractionGroups {
        InteractionGroups::new(
            Group::from_bits_retain(material.memberships),
            Group::from_bits_retain(material.filter),
            InteractionTestMode::And,
        )
    }
    #[test]
    fn actors_contact_only_their_surface_profile() {
        for actor in [CollisionPurpose::RigidBody, CollisionPurpose::Projectile] {
            for surface in [
                CollisionPurpose::RigidBody,
                CollisionPurpose::Projectile,
                CollisionPurpose::Lidar,
            ] {
                let a = groups(actor.actor_material(0., 0.).unwrap());
                let b = groups(surface.scenery_material(0., 0.));
                assert_eq!(a.test(b), actor == surface);
            }
        }
        assert!(CollisionPurpose::Lidar.actor_material(0., 0.).is_err());
    }
    #[test]
    fn actual_bodies_stop_on_different_profiles_in_one_world() {
        for actor in [CollisionPurpose::RigidBody, CollisionPurpose::Projectile] {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            for (purpose, half) in [
                (CollisionPurpose::RigidBody, 0.5),
                (CollisionPurpose::Projectile, 1.),
                (CollisionPurpose::Lidar, 1.5),
            ] {
                world.insert_collider(
                    ColliderBuilder::cuboid(half, 1., 1.)
                        .translation(Vector::new(2., 0., 0.))
                        .restitution(0.)
                        .collision_groups(groups(purpose.scenery_material(0., 0.))),
                    None,
                );
            }
            let handle = world.insert_body(
                RigidBodyBuilder::dynamic()
                    .linvel(Vector::new(5., 0., 0.))
                    .ccd_enabled(true)
                    .can_sleep(false),
            );
            world.insert_collider(
                ColliderBuilder::ball(0.1)
                    .mass(1.)
                    .restitution(0.)
                    .collision_groups(groups(actor.actor_material(0., 0.).unwrap())),
                Some(handle),
            );
            for _ in 0..128 {
                world.step();
            }
            let x = world.bodies[handle].translation().x;
            let expected = if actor == CollisionPurpose::RigidBody {
                1.4
            } else {
                0.9
            };
            assert!((x - expected).abs() < 0.03, "{actor:?}: {x}");
        }
    }
    #[test]
    fn query_capture_selects_one_profile_and_retains_common_shapes() {
        let mut scene = StaticGeometry::default();
        for (purpose, x) in [
            (CollisionPurpose::RigidBody, 2.),
            (CollisionPurpose::Projectile, 3.),
            (CollisionPurpose::Lidar, 4.),
        ] {
            scene.shapes.push((
                Pose {
                    translation: Vector::new(x, 0., 0.),
                    rotation: Rotation::IDENTITY,
                },
                SharedShape::ball(0.5),
            ));
            scene
                .collision_groups
                .push(groups(purpose.scenery_material(0., 0.)));
        }
        scene.shapes.push((
            Pose {
                translation: Vector::new(8., 0., 0.),
                rotation: Rotation::IDENTITY,
            },
            SharedShape::ball(0.5),
        ));
        scene.collision_groups.push(InteractionGroups::default());
        for (purpose, x) in [
            (CollisionPurpose::RigidBody, 2.),
            (CollisionPurpose::Projectile, 3.),
            (CollisionPurpose::Lidar, 4.),
        ] {
            let selected = scene.for_collision_purpose(purpose);
            assert_eq!(selected.shapes.len(), 2);
            let hit = selected
                .cast_ray([0.; 3], [1., 0., 0.], 10.)
                .unwrap()
                .unwrap();
            assert!((hit.distance_m - (x - 0.5)).abs() < 1e-9);
        }
    }
}
