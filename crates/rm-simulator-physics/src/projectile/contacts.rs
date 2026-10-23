// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! RoboMaster contact policy, scoring bindings and retained collection buffers.
use super::*;

/// Contact policy and retained buffers owned by the RoboMaster adapter.
#[derive(Default)]
pub(super) struct State {
    /// Every scoring collider: a target face index or a chassis armor module.
    pub(super) target_of: HashMap<ColliderHandle, Scorer>,
    /// Velocities before integration, in the current projectile order.
    pub(super) velocities_before: Vec<Vector>,
    /// Incoming linear/angular velocity and centre of mass, in chassis order.
    /// Armor collision speed must be measured before the solver stops a robot.
    pub(super) chassis_velocities_before: Vec<(Vector, Vector, Vector)>,
    /// Next touching list while the preceding list still gates contact starts.
    /// Swapped with projectile/chassis latches, preserving their encounter order.
    pub(super) touching: Vec<ColliderHandle>,
    /// Armor collisions since the last [`WorldPhysics::take_armor_collisions`].
    pub(super) armor_collisions: Vec<ArmorCollision>,
}

#[derive(Clone, Copy)]
pub(super) enum Scorer {
    Face(usize),
    Armor { chassis: u32, plate: u32 },
}
/// How a struck plate was moving during the tick.
#[derive(Clone, Copy)]
enum PlateMotion {
    Kinematic { face_prev: Pose3 },
    Body(RigidBodyHandle),
}

impl WorldPhysics {
    /// Contacts first seen by the substep that just stepped, which carried the
    /// target motions between `frac_prev` and `frac_next` of the tick and
    /// integrated for `dt_s` seconds. Latches and dwell windows stamp
    /// `now_ns`, the end of the containing tick. Appends directly to the tick's
    /// ordered output, retaining its storage across slices.
    pub(super) fn capture_contacts(
        &mut self,
        frames: &TargetFrames,
        frac_prev: f64,
        frac_next: f64,
        dt_s: f64,
        now_ns: u64,
        contacts: &mut Vec<Contact>,
    ) {
        let policy = self.policy;
        for (projectile, velocity) in self
            .projectiles
            .iter_mut()
            .zip(self.rm_contacts.velocities_before.iter().copied())
        {
            self.rm_contacts.touching.clear();
            // Any touch at all starts the ball's post-contact history; only a
            // fixed body counts as the stationary scenery retirement needs.
            let mut touched_anything = false;
            let mut touching_scenery = false;
            for pair in self.world.contact_pairs_with(projectile.collider) {
                if !pair.has_any_active_contact() {
                    continue;
                }
                let other = if pair.collider1 == projectile.collider {
                    pair.collider2
                } else {
                    pair.collider1
                };
                touched_anything = true;
                // Mechanism parts are parentless colliders too, but they move;
                // a ball resting on one is not on stationary scenery.
                touching_scenery |= match self.world.colliders[other].parent() {
                    Some(body) => self.world.bodies[body].is_fixed(),
                    None => !self.mechanisms.iter().any(|(handle, ..)| *handle == other),
                };
                let Some(&scorer) = self.rm_contacts.target_of.get(&other) else {
                    continue;
                };
                self.rm_contacts.touching.push(other);
                if projectile.touching.contains(&other) {
                    continue;
                }
                // The scoring face now, and how the plate moves: a kinematic
                // face by where it was a tick ago, a chassis by its body.
                let (target_of_scorer, face_next, motion) = match scorer {
                    Scorer::Face(index) => {
                        let (start, end) = frames.motion(index);
                        let from = pose(start.pose);
                        let to = pose(end.pose);
                        (
                            self.targets[index].target,
                            from.lerp(&to, frac_next),
                            PlateMotion::Kinematic {
                                face_prev: from.lerp(&to, frac_prev),
                            },
                        )
                    }
                    Scorer::Armor { chassis, plate } => {
                        let Some(owner) = self.chassis.iter().find(|c| c.id() == chassis) else {
                            continue;
                        };
                        (
                            ArmorTarget::Chassis { chassis, plate },
                            owner.armor_pose(&self.world, plate as usize),
                            PlateMotion::Body(owner.body()),
                        )
                    }
                };
                let world_point = pair
                    .find_deepest_contact()
                    .map(|(_, contact)| {
                        let (collider, local) = if pair.collider1 == projectile.collider {
                            (pair.collider2, contact.local_p2)
                        } else {
                            (pair.collider1, contact.local_p1)
                        };
                        self.world.colliders[collider]
                            .position()
                            .transform_point(local)
                    })
                    .unwrap_or_else(|| {
                        // Project the ball centre onto the housing front.
                        let centre = self.world.bodies[projectile.body].translation();
                        let mut local = face_next.inverse_transform_point(centre);
                        local.x = local.x.min(0.);
                        face_next.transform_point(local)
                    });
                // A shot is already absorbed before an out-of-arena armor contact.
                if outside_projectile_bounds(self.projectile_bounds_m, world_point, 0.0) {
                    continue;
                }
                let local = face_next.inverse_transform_point(world_point);
                let plate_velocity = match motion {
                    PlateMotion::Kinematic { face_prev } => {
                        (face_next.transform_point(local) - face_prev.transform_point(local)) / dt_s
                    }
                    PlateMotion::Body(body) => {
                        self.world.bodies[body].velocity_at_point(world_point)
                    }
                };
                let normal = face_next.transform_vector(Vector::X);
                let normal_speed_m_s = -(velocity - plate_velocity).dot(normal);
                contacts.push(Contact {
                    projectile: projectile.id,
                    shooter: projectile.shooter,
                    caliber: projectile.caliber,
                    target: target_of_scorer,
                    position_m: world_point.to_array(),
                    local_m: local.to_array(),
                    normal_speed_m_s,
                });
            }
            std::mem::swap(&mut projectile.touching, &mut self.rm_contacts.touching);
            if touched_anything && projectile.first_contact_ns.is_none() {
                projectile.first_contact_ns = Some(now_ns);
            }
            // A spent ball has touched stationary scenery, still touches it and
            // stays below the residual speed for the whole dwell window. Any
            // separation or renewed motion reopens the window.
            let speed_m_s = self.world.bodies[projectile.body].linvel().length();
            let slow = policy
                .retire_speed_m_s
                .is_some_and(|limit| speed_m_s <= limit);
            if touching_scenery && slow {
                projectile.dwell_since_ns.get_or_insert(now_ns);
            } else {
                projectile.dwell_since_ns = None;
            }
        }
    }
    /// Record armor modules that began touching anything but a projectile this
    /// substep fast enough to register ([`ARMOR_COLLISION_SPEED_M_S`]). A
    /// module resting against an obstacle registers once, when it arrives.
    pub(super) fn capture_armor_collisions(&mut self) {
        for index in 0..self.chassis.len() {
            let chassis = &self.chassis[index];
            self.rm_contacts.touching.clear();
            for (plate, &armor) in chassis.armor_colliders().iter().enumerate() {
                for pair in self.world.contact_pairs_with(armor) {
                    if !pair.has_any_active_contact() {
                        continue;
                    }
                    let other = if pair.collider1 == armor {
                        pair.collider2
                    } else {
                        pair.collider1
                    };
                    if self.projectiles.iter().any(|p| p.collider == other) {
                        continue;
                    }
                    self.rm_contacts.touching.push(other);
                    if chassis.armor_touching.contains(&other) {
                        continue;
                    }
                    let Some((_, contact)) = pair.find_deepest_contact() else {
                        continue;
                    };
                    let local = if pair.collider1 == armor {
                        contact.local_p1
                    } else {
                        contact.local_p2
                    };
                    let point = self.world.colliders[armor]
                        .position()
                        .transform_point(local);
                    let other_body = self.world.colliders[other].parent();
                    let other_index = other_body
                        .and_then(|body| self.chassis.iter().position(|c| c.body() == body));
                    let incoming = |i: usize| {
                        let (linear, angular, com) = self.rm_contacts.chassis_velocities_before[i];
                        linear + angular.cross(point - com)
                    };
                    let other_velocity = other_index.map_or_else(
                        || {
                            other_body.map_or(Vector::ZERO, |body| {
                                self.world.bodies[body].velocity_at_point(point)
                            })
                        },
                        incoming,
                    );
                    let own_velocity = incoming(index);
                    let normal = chassis
                        .armor_pose(&self.world, plate)
                        .transform_vector(Vector::X);
                    let normal_speed_m_s = (own_velocity - other_velocity).dot(normal);
                    if normal_speed_m_s < ARMOR_COLLISION_SPEED_M_S {
                        continue;
                    }
                    let other_chassis = other_index.map(|i| self.chassis[i].id());
                    self.rm_contacts.armor_collisions.push(ArmorCollision {
                        chassis: chassis.id(),
                        plate: plate as u32,
                        other_chassis,
                        position_m: point.to_array(),
                        normal_speed_m_s,
                    });
                }
            }
            std::mem::swap(
                &mut self.chassis[index].armor_touching,
                &mut self.rm_contacts.touching,
            );
        }
    }
}
