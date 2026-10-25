// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Insert the configured body and colliders in their original Rapier order.

use super::*;
use sim_physics::backend::CookedBodyExt;

impl Chassis {
    /// Insert the body into `world` at `spawn`. The wheels find the ground on
    /// the first step, so spawning a little above it is fine.
    pub(crate) fn new(
        world: &mut PhysicsWorld,
        id: u32,
        team: Team,
        config: ChassisConfig,
        spawn: Pose,
    ) -> Result<Self, &'static str> {
        config.validate()?;
        if !pose_is_valid(spawn) {
            return Err("chassis spawn pose must be finite with a unit rotation");
        }
        let prepared = PreparedChassis::new(config)?;
        Self::new_prepared(world, id, team, &prepared, spawn)
    }

    /// Install shared cooked geometry with independent controls and wheel state.
    pub(crate) fn new_prepared(
        world: &mut PhysicsWorld,
        id: u32,
        team: Team,
        prepared: &PreparedChassis,
        spawn: Pose,
    ) -> Result<Self, &'static str> {
        if !pose_is_valid(spawn) {
            return Err("chassis spawn pose must be finite with a unit rotation");
        }
        let config = prepared.config().clone();
        // Preparation validates and cooks every shape before touching the live
        // world. Use the cooked builders directly: generic body registration
        // would reset the wheel forces, and eager COM refresh changes timing.
        let body = world.insert_body(prepared.cooked().body_builder().pose(rapier_pose(spawn)));
        let colliders: Vec<_> = prepared
            .cooked()
            .collider_builders()
            .map(|collider| world.insert_collider(collider, Some(body)))
            .collect();
        let armor = prepared.inner.armor.iter().map(|&i| colliders[i]).collect();
        let fold_colliders = prepared
            .inner
            .prescribed
            .iter()
            .map(|&i| colliders[i])
            .collect();
        let geometry = wheel_geometry(&config);
        let wheels = vec![
            WheelState {
                spin_rad: 0.0,
                angular_velocity_rad_s: 0.0,
                suspension_compression_m: 0.0,
                target_m_s: 0.0,
                contact: None,
            };
            geometry.len()
        ];
        Ok(Self {
            fold_angle_rad: 0.,
            fold_colliders,
            placement_revision: 0,
            id,
            team,
            balance_gains: balance_lqr_gains(&config),
            config,
            body,
            armor,
            forces: Vec::with_capacity(geometry.len() * 2),
            drive_forces: Vec::with_capacity(geometry.len()),
            wheel_drive: WheelDriveScratch::default(),
            geometry,
            wheels,
            jump_held: false,
            drive_control: DriveControl::default(),
            command: ChassisCommand {
                aim_yaw_rad: yaw_of(spawn),
                ..Default::default()
            },
            aim_rad: [yaw_of(spawn), 0.0],
            gimbal_velocity_rad_s: [0.0; 2],
            drive_budget_w: f64::MAX,
            drive_power_w: 0.,
            actuation_cut: false,
            defeated: false,
            armor_touching: Vec::new(),
        })
    }
}
