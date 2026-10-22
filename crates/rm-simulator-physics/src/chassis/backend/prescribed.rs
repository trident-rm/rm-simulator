// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Existing prescribed folding/gimbal updates and their Rapier clearance adapter.

use super::*;
use rapier3d_f64::parry::bounding_volume::BoundingVolume;

impl Chassis {
    /// Authoritative held turret pose, including the aim frozen by defeat.
    pub(crate) fn turret_pose(&self, world: &PhysicsWorld) -> Pose {
        let body = &world.bodies[self.body];
        if let Some(folding) = &self.config.folding {
            return folding
                .poses(pose_of(body), self.aim_rad, self.fold_angle_rad)
                .head;
        }
        if let Some(a) = &self.config.articulation
            && let Some(index) = a.joints.iter().position(|j| j.name == "pitch")
        {
            let local = body.position().rotation.inverse()
                * Vector::new(
                    self.aim_rad[1].cos() * self.aim_rad[0].cos(),
                    self.aim_rad[1].cos() * self.aim_rad[0].sin(),
                    self.aim_rad[1].sin(),
                );
            let frame = a.joint_pose(
                index,
                self.fold_angle_rad,
                [local.y.atan2(local.x), local.z.clamp(-1., 1.).asin()],
            );
            let head = body.position() * rapier_pose(frame);
            return Pose {
                translation_m: head.translation.to_array(),
                rotation_wxyz: [
                    head.rotation.w,
                    head.rotation.x,
                    head.rotation.y,
                    head.rotation.z,
                ],
            };
        }
        Pose {
            translation_m: body
                .position()
                .transform_point(vector(self.config.turret_center_m))
                .to_array(),
            rotation_wxyz: aim_rotation(self.aim_rad[0], self.aim_rad[1]),
        }
    }
    /// World pose of the gun muzzle: the turret pose carried `MUZZLE_FORWARD_M`
    /// along the barrel's +x, which is the launch direction `fire` uses.
    pub(crate) fn muzzle_pose(&self, world: &PhysicsWorld) -> Pose {
        let turret = self.turret_pose(world);
        let transform = rapier_pose(turret);
        Pose {
            translation_m: transform
                .transform_point(vector(self.config.muzzle_offset_m()))
                .to_array(),
            rotation_wxyz: turret.rotation_wxyz,
        }
    }
    pub(super) fn step_fold(&mut self, world: &mut PhysicsWorld, dt_s: f64) {
        let limit = self.config.transform_limit_rad();
        if limit == 0. {
            if self.config.articulation.is_some() {
                self.sync_fold_colliders(world);
            }
            return;
        }
        let speed = self
            .config
            .folding
            .as_ref()
            .map(|f| f.speed_rad_s)
            .or_else(|| self.config.articulation.as_ref().map(|a| a.speed_rad_s))
            .unwrap_or_else(|| self.config.climbing.as_ref().unwrap().speed_rad_s);
        let before = self.fold_angle_rad;
        let goal = if self.command.folded { limit } else { 0. };
        if !self.defeated && !self.actuation_cut {
            let change = speed * dt_s;
            let next = self.fold_angle_rad + (goal - self.fold_angle_rad).clamp(-change, change);
            if next == self.fold_angle_rad || self.fold_path_clear(world, next) {
                self.fold_angle_rad = next;
            }
        }
        if self.config.climbing.is_some() {
            let old = wheel_geometry_at(&self.config, before);
            self.geometry = wheel_geometry_at(&self.config, self.fold_angle_rad);
            for (g, p) in self.geometry.iter_mut().zip(old) {
                g.velocity_m_s = (g.hub_m - p.hub_m) / dt_s;
            }
        }
        self.sync_fold_colliders(world);
    }
    fn fold_path_clear(&self, world: &PhysicsWorld, next: f64) -> bool {
        let body = pose_of(&world.bodies[self.body]);
        let before = self
            .config
            .transform_collision_parts(body, self.aim_rad, self.fold_angle_rad);
        // Bound angular sampling even for a caller-supplied faster actuator.
        let samples = ((next - self.fold_angle_rad).abs() / 0.25_f64.to_radians())
            .ceil()
            .max(2.) as usize;
        for step in 1..=samples {
            let angle =
                self.fold_angle_rad + (next - self.fold_angle_rad) * step as f64 / samples as f64;
            for ((old_pose, _), (new_pose, half)) in
                before
                    .iter()
                    .copied()
                    .zip(
                        self.config
                            .transform_collision_parts(body, self.aim_rad, angle),
                    )
            {
                let shape = Cuboid::new(vector(half));
                let new_pose = rapier_pose(new_pose);
                let bounds = shape.compute_aabb(&new_pose);
                for (_, obstacle) in world.colliders.iter() {
                    if obstacle.parent() == Some(self.body)
                        || obstacle.is_sensor()
                        || !obstacle.is_enabled()
                        || !bounds.intersects(&obstacle.compute_aabb())
                    {
                        continue;
                    }
                    let penetration = |pose| {
                        rapier3d_f64::parry::query::contact(
                            obstacle.position(),
                            obstacle.shape(),
                            &pose,
                            &shape,
                            0.,
                        )
                        .map(|c| c.map_or(0., |c| (-c.dist).max(0.)))
                    };
                    let Ok(new_depth) = penetration(new_pose) else {
                        return false;
                    };
                    if new_depth <= 1e-6 {
                        continue;
                    }
                    let old_depth = penetration(rapier_pose(old_pose)).unwrap_or(0.);
                    if new_depth > old_depth + 1e-6 {
                        return false;
                    }
                }
            }
        }
        true
    }
    pub(crate) fn sync_fold_colliders(&self, world: &mut PhysicsWorld) {
        let body = pose_of(&world.bodies[self.body]);
        let inverse = world.bodies[self.body].position().inverse();
        for (handle, (pose, _)) in
            self.fold_colliders
                .iter()
                .zip(
                    self.config
                        .transform_collision_parts(body, self.aim_rad, self.fold_angle_rad),
                )
        {
            world.colliders[*handle].set_position_wrt_parent(inverse * rapier_pose(pose));
            world.colliders[*handle].set_position(rapier_pose(pose));
        }
    }
    pub(super) fn step_gimbal(&mut self, dt_s: f64) {
        if self.defeated || self.actuation_cut {
            return;
        }
        let cfg = &self.config.dynamics;
        let targets = [
            wrap_angle(self.command.aim_yaw_rad),
            self.command.aim_pitch_rad.clamp(-1.4, 1.4),
        ];
        for (axis, target) in targets.into_iter().enumerate() {
            let error = if axis == 0 {
                wrap_angle(target - self.aim_rad[axis])
            } else {
                target - self.aim_rad[axis]
            };
            let speed = (error / cfg.gimbal_response_s)
                .clamp(-cfg.gimbal_max_speed_rad_s, cfg.gimbal_max_speed_rad_s);
            let rate = &mut self.gimbal_velocity_rad_s[axis];
            *rate += (speed - *rate).clamp(
                -cfg.gimbal_max_acceleration_rad_s2 * dt_s,
                cfg.gimbal_max_acceleration_rad_s2 * dt_s,
            );
            self.aim_rad[axis] += *rate * dt_s;
            if axis == 0 {
                self.aim_rad[axis] = wrap_angle(self.aim_rad[axis]);
            } else {
                self.aim_rad[axis] = self.aim_rad[axis].clamp(-1.4, 1.4);
            }
        }
    }
}
