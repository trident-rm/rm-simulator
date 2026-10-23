// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Ordered wheel queries, controller evaluation and force application per solver slice.

use super::*;

impl Chassis {
    pub(crate) fn apply_forces(&mut self, world: &mut PhysicsWorld, dt_s: f64) {
        self.step_gimbal(dt_s);
        self.step_fold(world, dt_s);
        let cfg = &self.config;
        let (pose, linvel, angvel, com) = {
            let body = &world.bodies[self.body];
            (
                *body.position(),
                body.linvel(),
                body.angvel(),
                body.center_of_mass(),
            )
        };
        let up = pose.rotation * Vector::Z;
        // A defeated robot's drive is cut: the wheels brake to a stop.
        let command = if self.defeated
            || self.actuation_cut
            || self.drive_control != DriveControl::BodyVelocity
        {
            ChassisCommand::default()
        } else {
            self.command
        };
        if cfg.planar_flight {
            let mut wish = pose.rotation * Vector::new(command.forward_m_s, command.left_m_s, 0.0);
            wish.z = 0.0;
            let mut pull = Vector::ZERO;
            if let Some(tether) = &cfg.tether {
                let hook = pose.translation;
                let anchor = vector(tether.anchor_m(hook.to_array()));
                let outward = Vector::new(hook.x - anchor.x, hook.y - anchor.y, 0.0);
                if outward.length() > 1e-9 {
                    let outward = outward.normalize();
                    let slack = tether.slack_m(hook.to_array());
                    // Brake so the drone stops where the tether goes taut; once
                    // it is past, fly back in at up to 1 m/s.
                    let allowed = if slack > 0.0 {
                        (2.0 * FLIGHT_ACCELERATION_M_S2 * slack).sqrt()
                    } else {
                        -(FLIGHT_VELOCITY_GAIN_PER_S * -slack).min(1.0)
                    };
                    let out = wish.dot(outward);
                    if out > allowed {
                        wish -= outward * (out - allowed);
                    }
                    if slack < 0.0 {
                        let stretching = linvel.dot(outward).max(0.0);
                        pull = -outward
                            * (TETHER_STIFFNESS_PER_KG * -slack
                                + TETHER_DAMPING_PER_KG * stretching)
                                .min(TETHER_MAX_PULL_M_S2);
                    }
                }
            }
            let acceleration = ((wish - linvel) * FLIGHT_VELOCITY_GAIN_PER_S)
                .clamp_length_max(FLIGHT_ACCELERATION_M_S2)
                + pull;
            let body = &mut world.bodies[self.body];
            body.reset_forces(false);
            body.reset_torques(false);
            body.add_force(
                Vector::new(acceleration.x, acceleration.y, 0.0) * cfg.mass_kg,
                true,
            );
            body.add_torque(
                Vector::Z * ((command.yaw_rate_rad_s - angvel.z) * 3.0).clamp(-8.0, 8.0),
                true,
            );
            self.jump_held = self.command.jump;
            return;
        }
        let model = WheelModel::from(cfg);
        let reach = cfg.wheel_radius_m + cfg.suspension_rest_m;
        // Projectile group 2 must not act as a suspension support. A ray spring
        // resting on a loose 17 mm ball would invent a rigid terrain column.
        let filter =
            QueryFilter::default()
                .exclude_rigid_body(self.body)
                .groups(InteractionGroups::new(
                    Group::ALL,
                    Group::ALL & !Group::GROUP_2,
                    InteractionTestMode::And,
                ));
        self.forces.clear();
        self.drive_forces.clear();
        self.wheel_drive.contacts.clear();
        for (wheel, geometry) in self.wheels.iter_mut().zip(&self.geometry) {
            let hub = pose.transform_point(geometry.hub_m);
            let axle = pose.rotation * geometry.axle;
            let target = command.forward_m_s * geometry.roll.x
                + command.left_m_s * geometry.roll.y
                + command.yaw_rate_rad_s * geometry.lever_m;
            wheel.target_m_s = target;
            let ray = Ray::new(hub, -up);
            let ground = world
                .cast_ray_and_get_normal(&ray, reach, true, filter)
                .and_then(|(_, hit)| {
                    let normal = if hit.normal.dot(up) < 0.0 {
                        -hit.normal
                    } else {
                        hit.normal
                    };
                    (normal.dot(up) > MIN_GROUND_COSINE && normal.is_finite())
                        .then_some((hit.time_of_impact, normal))
                });
            let Some((distance, normal)) = ground else {
                wheel.contact = None;
                wheel.suspension_compression_m = 0.0;
                self.wheel_drive.contacts.push((0.0, 0.0));
                self.drive_forces.push((Vector::ZERO, hub));
                continue;
            };
            let compression = reach - distance;
            wheel.suspension_compression_m = compression;
            let hub_velocity =
                linvel + angvel.cross(hub - com) + pose.rotation * geometry.velocity_m_s;
            let compression_rate = -hub_velocity.dot(up);
            let load = model.suspension_load_n(compression, compression_rate);
            let point = ray.point_at(distance);
            let contact_velocity =
                linvel + angvel.cross(point - com) + pose.rotation * geometry.velocity_m_s;
            let roll_dir = normal.cross(axle).normalize();
            let roller_dir = (axle - normal * axle.dot(normal)).normalize();
            if !roll_dir.is_finite() || !roller_dir.is_finite() {
                wheel.contact = None;
                wheel.suspension_compression_m = 0.0;
                self.wheel_drive.contacts.push((0.0, 0.0));
                self.drive_forces.push((Vector::ZERO, hub));
                continue;
            }
            let roll_speed = contact_velocity.dot(roll_dir);
            let roller_speed = contact_velocity.dot(roller_dir);
            let grip = model.drive_friction * load;
            self.wheel_drive.contacts.push((roll_speed, grip));
            let roller = model.roller_force_n(roller_speed, load);
            self.drive_forces.push((roll_dir, point));
            self.forces.push((roller_dir * roller, point));
            self.forces.push((up * load, hub));
            wheel.contact = Some(WheelContact {
                point_m: point.to_array(),
                normal: normal.to_array(),
                load_n: load,
                slip_m_s: 0.0,
                drive_force_n: 0.0,
            });
        }
        let body = &mut world.bodies[self.body];
        // Forces and torques are separate accumulators; both must be cleared.
        body.reset_forces(false);
        body.reset_torques(false);
        let budget = if self.defeated || self.actuation_cut {
            0.
        } else {
            self.drive_budget_w.min(cfg.dynamics.drive_power_w)
        };
        let torques = match &self.drive_control {
            DriveControl::WheelTorques(torques) if !self.defeated && !self.actuation_cut => {
                Some(torques)
            }
            _ => None,
        };
        let (scale, power) = self.wheel_drive.integrate(
            &model,
            &mut self.wheels,
            dt_s,
            budget,
            torques.map(WheelTorques::as_nm),
        );
        self.drive_power_w = power;
        for ((wheel, (force, _)), traction) in self
            .wheels
            .iter_mut()
            .zip(&mut self.drive_forces)
            .zip(self.wheel_drive.forces.iter().copied())
        {
            *force *= traction;
            if let Some(contact) = &mut wheel.contact {
                contact.drive_force_n = traction;
            }
        }
        let supported = self
            .wheels
            .iter()
            .filter(|wheel| wheel.contact.is_some())
            .count();
        if cfg.balance_assist
            && supported > 0
            && budget > 0.
            && self.drive_control == DriveControl::BodyVelocity
        {
            // Reduced inverted-pendulum LQR, with a small speed-error lean
            // setpoint. The motor/leg plant remains an approximation; this is
            // an assisted body torque, not a firmware wheel-leg controller.
            let lateral = pose.rotation * Vector::Y;
            let forward = pose.rotation * Vector::X;
            let pitch_error = up.cross(Vector::Z).dot(lateral).clamp(-1.0, 1.0).asin();
            let rate = angvel.dot(lateral);
            let desired_lean =
                ((command.forward_m_s - linvel.dot(forward)) * 0.045).clamp(-0.10, 0.10);
            let strength = f64::from(command.balance_control) / 100.0;
            let [kp, kd] = self.balance_gains;
            let torque = (kp * (pitch_error + scale * desired_lean) - kd * rate)
                .clamp(-180.0, 180.0)
                * strength;
            body.add_torque(lateral * torque, true);
            // A 2.4 m/s equivalent leg push gives about 0.29 m ballistic rise.
            // Both wheels must support an upright body. Holding the button,
            // replaying a checkpoint or pressing again in flight cannot stack it.
            if command.jump
                && !self.jump_held
                && supported == cfg.wheel_hubs_m.len()
                && up.dot(Vector::Z) > 0.85
                && linvel.z < 0.5
                && !self.defeated
            {
                body.apply_impulse(Vector::Z * cfg.mass_kg * 2.4, true);
            }
        }
        self.jump_held = self.command.jump;
        for (force, point) in self.drive_forces.drain(..) {
            body.add_force_at_point(force, point, true);
        }
        for (force, point) in self.forces.drain(..) {
            body.add_force_at_point(force, point, true);
        }
    }
}
