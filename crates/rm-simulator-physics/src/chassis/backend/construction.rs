// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Insert the configured body and colliders in their original Rapier order.

use super::*;

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
        let body = world.insert_body(
            RigidBodyBuilder::dynamic()
                .gravity_scale(if config.planar_flight { 0.0 } else { 1.0 })
                .enabled_translations(true, true, !config.planar_flight)
                .enabled_rotations(!config.planar_flight, !config.planar_flight, true)
                .pose(rapier_pose(spawn))
                .linear_damping(LINEAR_DAMPING)
                .angular_damping(ANGULAR_DAMPING)
                .can_sleep(false),
        );
        // The turret carries a share of the mass so the centre of mass sits
        // above the body centre, as on a real robot. A flying body keeps all
        // of it centred instead: its roll and pitch locks hold only while
        // the inertia stays axis-aligned, and its gimbal hangs off-centre.
        let turret_share =
            if config.planar_flight || config.folding.is_some() || config.articulation.is_some() {
                0.0
            } else {
                TURRET_MASS_SHARE
            };
        let [hx, hy, hz] = config.body_half_m;
        let deck = if config.omni_frame.is_some() {
            let points: Vec<_> = config
                .body_outline_m()
                .iter()
                .flat_map(|&[x, y]| [Vector::new(x, y, -hz), Vector::new(x, y, hz)])
                .collect();
            ColliderBuilder::convex_hull(&points).ok_or("invalid octagonal deck")?
        } else {
            ColliderBuilder::cuboid(hx, hy, hz)
        };
        world.insert_collider(
            deck.mass(config.mass_kg * (1.0 - turret_share))
                .friction(BODY_FRICTION)
                .restitution(ROBOT_RESTITUTION)
                .restitution_combine_rule(CoefficientCombineRule::Min),
            Some(body),
        );
        if config.team_cad {
            // Source-envelope gameplay proxies, not visual triangles or mass estimates.
            // The center case extends below the nominal frame; ammunition bays sit
            // above the scoring faces and inset from their leaning upper edges.
            for (half, centre) in [
                ([0.103, 0.103, 0.079], [0., 0., -0.030]),
                ([0.10, 0.07, 0.032], [0.145, 0.16, 0.077]),
                ([0.10, 0.07, 0.032], [-0.145, 0.16, 0.077]),
                ([0.10, 0.07, 0.032], [0.145, -0.16, 0.077]),
                ([0.10, 0.07, 0.032], [-0.145, -0.16, 0.077]),
            ] {
                world.insert_collider(
                    ColliderBuilder::cuboid(half[0], half[1], half[2])
                        .translation(vector(centre))
                        .density(0.0)
                        .friction(BODY_FRICTION)
                        .restitution(ROBOT_RESTITUTION),
                    Some(body),
                );
            }
        }
        for prism in config.bumper_prisms_m() {
            let vertices = prism.map(Vector::from_array);
            world.insert_collider(
                ColliderBuilder::convex_hull(&vertices)
                    .ok_or("invalid bumper prism")?
                    .density(0.0)
                    .friction(BODY_FRICTION)
                    .restitution(ROBOT_RESTITUTION)
                    .restitution_combine_rule(CoefficientCombineRule::Min),
                Some(body),
            );
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
        let mut fold_colliders = Vec::new();
        if let Some(folding) = &config.folding {
            for (pose, half) in folding.collision_parts(Pose::default(), [0., 0.], 0.) {
                fold_colliders.push(
                    world.insert_collider(
                        ColliderBuilder::cuboid(half[0], half[1], half[2])
                            .position(rapier_pose(pose))
                            .density(0.)
                            .friction(BODY_FRICTION)
                            .restitution(ROBOT_RESTITUTION),
                        Some(body),
                    ),
                );
            }
        } else if config.articulation.is_none() {
            world.insert_collider(
                ColliderBuilder::cuboid(tx, ty, column_half)
                    .translation(Vector::new(cx, cy, column_bottom + column_half))
                    .mass(config.mass_kg * turret_share)
                    .friction(BODY_FRICTION)
                    .restitution(ROBOT_RESTITUTION)
                    .restitution_combine_rule(CoefficientCombineRule::Min),
                Some(body),
            );
        }
        if config.articulation.is_some() {
            for (p, h) in config.transform_collision_parts(Pose::default(), [0., 0.], 0.) {
                fold_colliders.push(
                    world.insert_collider(
                        ColliderBuilder::cuboid(h[0], h[1], h[2])
                            .position(rapier_pose(p))
                            .density(0.)
                            .friction(BODY_FRICTION),
                        Some(body),
                    ),
                );
            }
        }
        if config.climbing.is_some() {
            // Fixed electronics shelf is behind the front entry bay; a full
            // depth bounding box would falsely fill the empty climbing clearance.
            world.insert_collider(
                ColliderBuilder::cuboid(0.05, 0.26, 0.006)
                    .translation(Vector::new(-0.075, 0., -0.16))
                    .density(0.)
                    .friction(BODY_FRICTION)
                    .restitution(ROBOT_RESTITUTION),
                Some(body),
            );
            for (pose, half) in config.transform_collision_parts(Pose::default(), [0., 0.], 0.) {
                fold_colliders.push(
                    world.insert_collider(
                        ColliderBuilder::cuboid(half[0], half[1], half[2])
                            .position(rapier_pose(pose))
                            .density(0.)
                            .friction(BODY_FRICTION)
                            .restitution(ROBOT_RESTITUTION),
                        Some(body),
                    ),
                );
            }
        }
        // Armor housings ride the body without adding to its mass.
        let [ax, ay, az] = config.armor_housing_half_m();
        let armor = config
            .armor_faces()
            .iter()
            .map(|face| {
                let local =
                    rapier_pose(*face) * Pose3::from_translation(Vector::new(-ax, 0.0, 0.0));
                world.insert_collider(
                    ColliderBuilder::cuboid(ax, ay, az)
                        .position(local)
                        .density(0.0)
                        .friction(FRICTION)
                        .restitution(ROBOT_RESTITUTION)
                        .restitution_combine_rule(CoefficientCombineRule::Min),
                    Some(body),
                )
            })
            .collect();
        for plate in 0..ARMOR_COUNT {
            for prism in config.armor_bracket_prisms_m(plate) {
                world.insert_collider(
                    ColliderBuilder::convex_hull(&prism.map(Vector::from_array))
                        .ok_or("invalid armor carrier")?
                        .density(0.0)
                        .friction(BODY_FRICTION)
                        .restitution(ROBOT_RESTITUTION),
                    Some(body),
                );
            }
        }
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
