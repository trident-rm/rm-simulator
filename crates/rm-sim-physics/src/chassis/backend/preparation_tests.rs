// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Frozen pre-migration constructor oracle: shape order and native mass timing.
use super::*;
use crate::mechanism::{ArticulationConfig, JointDrive, JointEnvelope, SourceJoint};

// Deliberately independent native builder path from 8ec39d0. Keep this oracle
// unchanged when evolving preparation, so source adaptation cannot bless itself.
fn reference(
    world: &mut PhysicsWorld,
    config: &ChassisConfig,
    spawn: Pose,
) -> Result<(RigidBodyHandle, Vec<ColliderHandle>, Vec<ColliderHandle>), &'static str> {
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
    let armor: Vec<_> = config
        .armor_faces()
        .iter()
        .map(|face| {
            let local = rapier_pose(*face) * Pose3::from_translation(Vector::new(-ax, 0.0, 0.0));
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

    Ok((body, armor, fold_colliders))
}

fn source_articulation() -> ChassisConfig {
    let mut config = ChassisConfig::default();
    config.articulation = Some(ArticulationConfig {
        profile: "construction-parity".into(),
        limit_rad: 1.,
        speed_rad_s: 0.5,
        muzzle_offset_m: [0.2, 0., 0.],
        camera_offset_m: [0.1, 0., 0.],
        armor_faces: config.armor_faces(),
        joints: vec![SourceJoint {
            name: "pitch".into(),
            parent: None,
            pivot_m: [0.02, 0.01, 0.2],
            axis: [0., 1., 0.],
            angle_rad: -1.,
            translation_m: [0.; 3],
            drive: JointDrive::Pitch,
            collision: Some(JointEnvelope {
                rotation_wxyz: Pose::yawed([0.; 3], 0.27).rotation_wxyz,
                center_m: [0.1, 0., 0.],
                half_m: [0.12, 0.04, 0.03],
            }),
        }],
    });
    config
}

#[test]
fn prepared_sources_preserve_native_shapes_policies_roles_and_mass_timing() {
    for config in [
        ChassisConfig::default(),
        ChassisConfig::team_cad(),
        ChassisConfig::hero(),
        ChassisConfig::engineer(),
        ChassisConfig::balance(),
        ChassisConfig::drone(),
        ChassisConfig::folding_hero(),
        ChassisConfig::stair_heavy(),
        source_articulation(),
    ] {
        let spawn = Pose::yawed([1., -2., 0.5], 0.37);
        let mut old = PhysicsWorld::default();
        let mut new = PhysicsWorld::default();
        let (body, armor, prescribed) = reference(&mut old, &config, spawn).unwrap();
        let chassis = Chassis::new(&mut new, 0, Team::Red, config.clone(), spawn).unwrap();
        assert_eq!(chassis.body, body);
        assert_eq!(chassis.armor, armor);
        assert_eq!(chassis.fold_colliders, prescribed);
        assert_eq!(old.colliders.len(), new.colliders.len());
        for ((old_handle, expected), (new_handle, actual)) in
            old.colliders.iter().zip(new.colliders.iter())
        {
            assert_eq!(old_handle, new_handle);
            // Includes exact native hull point order, mounted rotation, shape
            // mass policy, combine rules, filters and all default collider flags.
            assert_eq!(
                format!("{expected:?}"),
                format!("{actual:?}"),
                "collider {old_handle:?}"
            );
        }
        // Insertion leaves the body waiting for native COM/inertia refresh.
        // Compare before and after that refresh to enforce both the timing and
        // ordered collider aggregation (not just nominal configured mass).
        assert_eq!(
            format!("{:?}", old.bodies[body]),
            format!("{:?}", new.bodies[body])
        );
        old.bodies[body].recompute_mass_properties_from_colliders(&old.colliders);
        new.bodies[body].recompute_mass_properties_from_colliders(&new.colliders);
        assert_eq!(
            format!("{:?}", old.bodies[body]),
            format!("{:?}", new.bodies[body])
        );
        assert!(new.bodies[body].mass() > 0.);

        let prepared = preparation::PreparedChassis::new(config.clone()).unwrap();
        let source = prepared.source();
        assert_eq!(source.colliders().len(), old.colliders.len());
        let mut expected_hulls = Vec::new();
        if config.omni_frame.is_some() {
            expected_hulls.push(
                config
                    .body_outline_m()
                    .iter()
                    .flat_map(|&[x, y]| {
                        [
                            [x, y, -config.body_half_m[2]],
                            [x, y, config.body_half_m[2]],
                        ]
                    })
                    .collect::<Vec<_>>(),
            );
        }
        expected_hulls.extend(config.bumper_prisms_m().into_iter().map(|p| p.to_vec()));
        for plate in 0..ARMOR_COUNT {
            expected_hulls.extend(
                config
                    .armor_bracket_prisms_m(plate)
                    .into_iter()
                    .map(|p| p.to_vec()),
            );
        }
        let actual_hulls: Vec<_> = source
            .colliders()
            .iter()
            .filter_map(|c| {
                if let rm_sim_engine::model::ShapeDescription::ConvexHull(hull) = c.shape() {
                    Some(hull.points_m())
                } else {
                    None
                }
            })
            .collect();
        assert_eq!(actual_hulls.len(), expected_hulls.len());
        for (actual, expected) in actual_hulls.iter().zip(&expected_hulls) {
            assert_eq!(*actual, expected);
        }
    }
}
