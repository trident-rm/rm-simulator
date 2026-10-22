// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Existing chassis regression tests, moved without changing their assertions.

use super::*;
use crate::{
    projectile::{Caliber, SMALL_ARMOR_HOUSING_HALF_M, Shot, TargetFrames, WorldPhysics},
    tick_ns,
};
fn integrate_wheel_drives(
    config: &ChassisConfig,
    wheels: &mut [WheelState],
    contacts: &[(f64, f64)],
    dt_s: f64,
    budget_w: f64,
    torques: Option<&WheelTorques>,
) -> (Vec<f64>, f64, f64) {
    let mut scratch = WheelDriveScratch::default();
    scratch.contacts.extend_from_slice(contacts);
    let (scale, power) = scratch.integrate(
        &WheelModel::from(config),
        wheels,
        dt_s,
        budget_w,
        torques.map(WheelTorques::as_nm),
    );
    (scratch.forces, scale, power)
}

#[test]
fn wheel_adapter_preserves_reference_trajectories() {
    // Recorded from unextracted 8f4f44a on x86_64 Talos. Values are pose,
    // linear/angular velocity, then (spin, rate, compression, traction) per wheel.
    // Tolerance permits platform math differences; extraction matched exactly
    // on the reference machine. This is not a cross-backend rollback promise.
    let expected: [&[f64]; 6] = [
        &[
            1.1278962312570662,
            0.04918715234064451,
            0.17154571812301148,
            0.9986919547605752,
            0.02022946191615501,
            -0.03518365792332064,
            0.03110078105480534,
            1.8628788938899397,
            0.30300184429688903,
            -0.0003751604487589825,
            0.017960229218667783,
            0.13821415336731366,
            0.1972271083889787,
            -0.09984539343403531,
            -14.99443540998521,
            0.015290338717257296,
            -3.558742144512922,
            -0.5876099379558508,
            -20.147895300108654,
            0.04020386381014841,
            -21.50915948818211,
            -0.1620988854003631,
            16.272654095098574,
            0.0534802530101947,
            3.6870706338657047,
            1.7078025455215107,
            21.425418111299273,
            0.028566727917303608,
            21.669369628545393,
        ],
        &[
            1.257968118263863,
            0.11327483539257603,
            0.16031875346086052,
            0.9988658670630445,
            0.0004963222141976348,
            -0.0014154591022035945,
            0.04758917688069798,
            1.9141094021285805,
            0.4210133031862688,
            -0.00039614322114683045,
            -0.0020819522601602386,
            0.003196299496580201,
            0.20303637568428812,
            -1.468225957028715,
            20.62568377048302,
            0.006039297368145152,
            -0.5294312349561168,
            2.866939237218757,
            31.0326916602818,
            0.0065115769948708435,
            9.881616806182318,
            1.3745062790297675,
            28.20581093962577,
            0.0075791602183434975,
            10.161219936598666,
            -0.2632990536429838,
            23.452565919959497,
            0.008051336820973184,
            -0.8090804173565072,
        ],
        &[
            0.008887430823029655,
            6.103680294671354e-19,
            0.152544911670361,
            0.9999999997512101,
            -2.7495839012429914e-19,
            -2.230649923469587e-5,
            -9.402552968391045e-19,
            0.01013929081454977,
            -3.309957122566318e-19,
            4.757049352022981e-6,
            5.7980457038271445e-18,
            0.00015088738271461382,
            -3.9261171512908155e-18,
            -2.2013358573974475,
            -15.049453403042229,
            0.05394733844861191,
            -1.0790059569387038,
            -1.1652719947240788,
            -20.595385276371445,
            0.05396284726134011,
            -1.0791740391666094,
            -2.861036324307832,
            16.33049548928741,
            0.05396284726134011,
            1.0791740391666094,
            2.363812639780557,
            21.876434693394238,
            0.05394733844861191,
            1.0790059569387038,
        ],
        &[
            0.04811244214069459,
            -0.041169258828440414,
            0.15194596695402218,
            0.9991945456239851,
            0.0020558032563690737,
            -0.03993553940699272,
            0.003344601698529911,
            0.5272745466006543,
            -0.17350064354100653,
            0.06788074911879097,
            0.0019413079328810966,
            -0.2884203219679275,
            -0.033823664062315725,
            -1.6819863321210926,
            -13.706827409372522,
            0.03964633363767252,
            -29.223759682335576,
            2.087596133410184,
            -6.394329587075259,
            0.06729301213770685,
            -29.80984385342221,
            1.6516994571541534,
            9.292896613301025,
            0.06862814019722556,
            27.94874998029869,
            0.9709377894795743,
            17.849207849116144,
            0.04098146169719123,
            30.44306790555788,
        ],
        &[
            -3.985211603722404e-18,
            1.454687428728153e-17,
            0.15254479104665658,
            0.4528082288575059,
            -7.216610033218708e-18,
            3.957901064715192e-18,
            0.891607933947948,
            -1.470826545201026e-17,
            1.732453152756608e-17,
            5.347019716041867e-6,
            2.1228856546778087e-17,
            -7.690961958841481e-18,
            4.220519094902586,
            0.9759939756996525,
            13.746270475767705,
            0.05395521420911864,
            2.6927838726517423,
            0.9759939756996525,
            13.746270475767702,
            0.05395521420911864,
            2.692783872651705,
            0.9759939756996525,
            13.746270475767705,
            0.05395521420911864,
            2.6927838726517423,
            0.9759939756996525,
            13.746270475767705,
            0.05395521420911864,
            2.692783872651709,
        ],
        &[
            1.4642750779796954,
            0.028713767038826582,
            0.3057431730939594,
            0.9989839328007772,
            0.0008549495126304336,
            -0.006177785459202768,
            0.044634135300722494,
            1.9839965076900623,
            0.14709372583135819,
            -0.0031226314971125398,
            -0.002721057275582049,
            0.035081789865150606,
            0.19595665781539892,
            2.679524473064732,
            -19.476674865712084,
            0.003952319035837731,
            -2.5938477128581856,
            -2.2133326737971752,
            20.432793270493676,
            0.0045073731376823745,
            2.82673967876978,
        ],
    ];
    for (case, expected) in expected.into_iter().enumerate() {
        let mut cfg = if case == 1 {
            ChassisConfig::folding_hero()
        } else if case == 5 {
            ChassisConfig::balance()
        } else {
            ChassisConfig::default()
        };
        if case == 2 {
            cfg.drive_friction = 0.02;
        }
        let height = if case == 3 { 3.0 } else { cfg.rest_height_m() };
        let mut world = chassis_world(cfg, Pose::at([0., 0., height]));
        world
            .command_chassis(
                0,
                ChassisCommand {
                    forward_m_s: 2.,
                    left_m_s: 0.3,
                    yaw_rate_rad_s: 0.2,
                    ..Default::default()
                },
            )
            .unwrap();
        if case == 4 {
            world
                .command_chassis_wheel_torques(0, WheelTorques::new(vec![0.3; 4]).unwrap())
                .unwrap();
        }
        let frames = TargetFrames::new(vec![]);
        for tick in 0..128 {
            world.step(tick * tick_ns(), &frames).unwrap();
        }
        let state = world.chassis_snapshot(0).unwrap();
        let actual: Vec<f64> = state
            .pose
            .translation_m
            .into_iter()
            .chain(state.pose.rotation_wxyz)
            .chain(state.velocity_m_s)
            .chain(state.angular_velocity_rad_s)
            .chain(state.wheels.iter().flat_map(|w| {
                [
                    w.spin_rad,
                    w.angular_velocity_rad_s,
                    w.suspension_compression_m,
                    w.contact.map_or(0., |c| c.drive_force_n),
                ]
            }))
            .collect();
        assert_eq!(actual.len(), expected.len());
        for (index, (&actual, &expected)) in actual.iter().zip(expected).enumerate() {
            assert!(
                (actual - expected).abs() < 1e-9,
                "case {case}, component {index}: {actual} != {expected}"
            );
        }
    }
}

fn chassis_world(config: ChassisConfig, spawn: Pose) -> WorldPhysics {
    let mut ballistics = WorldPhysics::new(&[], 0.0);
    ballistics.add_chassis(Team::Red, config, spawn).unwrap();
    ballistics
}

#[test]
fn reused_drive_buffers_match_fresh_storage_across_contact_and_power_changes() {
    for config in [ChassisConfig::default(), ChassisConfig::folding_hero()] {
        let count = config.wheel_hubs_m.len();
        let initial = WheelState {
            spin_rad: 0.,
            angular_velocity_rad_s: 0.,
            suspension_compression_m: 0.,
            target_m_s: 2.,
            contact: None,
        };
        let mut actual = vec![initial; count];
        let mut expected = actual.clone();
        let mut scratch = WheelDriveScratch::default();
        let torques = WheelTorques::new(vec![0.3; count]).unwrap();
        let mut storage = None;
        for slice in 0..256 {
            scratch.contacts.clear();
            for corner in 0..count {
                scratch.contacts.push(if (slice + corner) % 3 == 0 {
                    (0., 0.)
                } else {
                    (slice as f64 * 0.001, 50.)
                });
            }
            let budget = if slice % 5 == 0 { 0. } else { 40. };
            let direct = (slice % 2 == 0).then_some(&torques);
            let fresh = integrate_wheel_drives(
                &config,
                &mut expected,
                &scratch.contacts,
                0.001,
                budget,
                direct,
            );
            let result = scratch.integrate(
                &WheelModel::from(&config),
                &mut actual,
                0.001,
                budget,
                direct.map(WheelTorques::as_nm),
            );
            assert_eq!(result, (fresh.1, fresh.2));
            assert_eq!(scratch.forces, fresh.0);
            for (actual, expected) in actual.iter().zip(&expected) {
                assert_eq!(actual.spin_rad, expected.spin_rad);
                assert_eq!(
                    actual.angular_velocity_rad_s,
                    expected.angular_velocity_rad_s
                );
            }
            let pointers = (scratch.contacts.as_ptr(), scratch.forces.as_ptr());
            if let Some(previous) = storage {
                assert_eq!(
                    pointers, previous,
                    "solver buffers reallocated after warmup"
                );
            }
            storage = Some(pointers);
        }
    }
}

#[test]
fn direct_wheel_requests_validate_switch_and_restore_without_touching_aim() {
    let cfg = ChassisConfig::default();
    let mut world = chassis_world(cfg, Pose::at([0., 0., 3.]));
    let command = ChassisCommand {
        forward_m_s: 2.,
        aim_yaw_rad: 0.7,
        ..Default::default()
    };
    world.command_chassis(0, command).unwrap();
    assert!(WheelTorques::new(vec![f64::NAN; 4]).is_err());
    assert!(WheelTorques::new(vec![f64::INFINITY; 4]).is_err());
    assert!(WheelTorques::new(vec![]).is_err());
    assert!(
        world
            .command_chassis_wheel_torques(0, WheelTorques::new(vec![1.; 3]).unwrap())
            .is_err()
    );
    assert_eq!(
        world.chassis_snapshot(0).unwrap().drive_control,
        DriveControl::BodyVelocity
    );
    let torques = WheelTorques::new(vec![1., -1., 0., 0.5]).unwrap();
    assert!(
        world
            .command_chassis_wheel_torques(99, torques.clone())
            .is_err()
    );
    world
        .command_chassis_wheel_torques(0, torques.clone())
        .unwrap();
    run(&mut world, 8);
    let saved = world.chassis_snapshot(0).unwrap();
    assert_eq!(saved.command, command);
    assert_eq!(saved.drive_control, DriveControl::WheelTorques(torques));
    assert!(saved.wheels[0].angular_velocity_rad_s > 0.);
    assert!(saved.wheels[1].angular_velocity_rad_s < 0.);
    assert_eq!(saved.wheels[2].angular_velocity_rad_s, 0.);
    assert!(saved.wheels.iter().all(|w| w.target_m_s == 0.));
    let mut restored = WorldPhysics::new(&[], 0.);
    restored.restore_chassis(&saved).unwrap();
    let mut malformed = saved.clone();
    malformed.drive_control = DriveControl::WheelTorques(WheelTorques::new(vec![1.]).unwrap());
    assert!(
        WorldPhysics::new(&[], 0.)
            .restore_chassis(&malformed)
            .is_err()
    );
    let frames = TargetFrames::new(vec![]);
    for tick in 8..16 {
        world.step(tick * tick_ns(), &frames).unwrap();
        restored.step(tick * tick_ns(), &frames).unwrap();
    }
    let a = world.chassis_snapshot(0).unwrap();
    let b = restored.chassis_snapshot(0).unwrap();
    assert_eq!(a.drive_control, b.drive_control);
    for (a, b) in a.wheels.iter().zip(&b.wheels) {
        assert!((a.angular_velocity_rad_s - b.angular_velocity_rad_s).abs() < 1e-12);
    }
    world.command_chassis(0, command).unwrap();
    assert_eq!(
        world.chassis_snapshot(0).unwrap().drive_control,
        DriveControl::BodyVelocity
    );
    let mut drone = chassis_world(ChassisConfig::drone(), Pose::at([0., 0., 2.]));
    assert!(
        drone
            .command_chassis_wheel_torques(0, WheelTorques::new(vec![1.]).unwrap())
            .is_err()
    );
}

#[test]
fn direct_motor_torques_respect_axis_units_limits_and_shared_power() {
    for cfg in [ChassisConfig::default(), ChassisConfig::folding_hero()] {
        let initial = WheelState {
            spin_rad: 0.,
            angular_velocity_rad_s: 0.,
            suspension_compression_m: 0.,
            target_m_s: -100.,
            contact: None,
        };
        let contacts = vec![(0., 0.); cfg.wheel_hubs_m.len()];
        let torques = WheelTorques::new(vec![0.01; contacts.len()]).unwrap();
        let mut wheels = vec![initial; contacts.len()];
        integrate_wheel_drives(
            &cfg,
            &mut wheels,
            &contacts,
            0.001,
            f64::MAX,
            Some(&torques),
        );
        // Axis torque/inertia is independent of the mecanum contact projection.
        assert!(
            (wheels[0].angular_velocity_rad_s - 0.01 / cfg.wheel_inertia_kg_m2 * 0.001).abs()
                < 1e-12
        );
        let huge = WheelTorques::new(vec![f64::MAX; contacts.len()]).unwrap();
        let mut limited = vec![initial; contacts.len()];
        let (_, _, power) =
            integrate_wheel_drives(&cfg, &mut limited, &contacts, 0.01, 1., Some(&huge));
        assert!(power > 0. && power <= 1. + 1e-12);
        assert!(limited[0].angular_velocity_rad_s.is_finite());
        let zero = WheelTorques::new(vec![0.; contacts.len()]).unwrap();
        let before = limited[0].angular_velocity_rad_s;
        integrate_wheel_drives(&cfg, &mut limited, &contacts, 0.01, 1., Some(&zero));
        assert_eq!(limited[0].angular_velocity_rad_s, before);
        let ratio = cfg.wheel_radius_m
            / if cfg.mecanum {
                std::f64::consts::SQRT_2
            } else {
                1.
            };
        let mut overspeed = vec![
            WheelState {
                angular_velocity_rad_s: 2. * cfg.wheel_no_load_speed_m_s / ratio,
                ..initial
            };
            contacts.len()
        ];
        let before = overspeed[0].angular_velocity_rad_s;
        integrate_wheel_drives(&cfg, &mut overspeed, &contacts, 0.01, f64::MAX, Some(&huge));
        assert_eq!(overspeed[0].angular_velocity_rad_s, before);
        let brake = WheelTorques::new(vec![-0.01; contacts.len()]).unwrap();
        integrate_wheel_drives(
            &cfg,
            &mut overspeed,
            &contacts,
            0.01,
            f64::MAX,
            Some(&brake),
        );
        assert!(overspeed[0].angular_velocity_rad_s < before);
    }
}

#[test]
fn direct_drive_obeys_defeat_power_cut_and_ground_traction() {
    let cfg = ChassisConfig::default();
    for (defeated, cut, budget) in [(true, false, 80.), (false, true, 80.), (false, false, 0.)] {
        let mut world = chassis_world(cfg.clone(), Pose::at([0., 0., 3.]));
        world
            .command_chassis_wheel_torques(0, WheelTorques::new(vec![1.; 4]).unwrap())
            .unwrap();
        world.set_chassis_defeated(0, defeated);
        world.set_chassis_power(0, budget, cut);
        let saved = run(&mut world, 16);
        assert!(saved.wheels.iter().all(|w| w.angular_velocity_rad_s == 0.));
        assert_eq!(world.chassis_drive_power_w(0), Some(0.));
    }
    let mut world = chassis_world(cfg.clone(), Pose::at([0., 0., cfg.rest_height_m()]));
    // Default tangential wheel order: these signs drive body +x.
    world
        .command_chassis_wheel_torques(0, WheelTorques::new(vec![-1., -1., 1., 1.]).unwrap())
        .unwrap();
    let saved = run(&mut world, 128);
    assert!(saved.pose.translation_m[0] > 0.1, "{:?}", saved.pose);
    assert!(saved.wheels.iter().any(|w| w.contact.is_some()));

    let balance = ChassisConfig::balance();
    let mut world = chassis_world(balance.clone(), Pose::at([0., 0., balance.rest_height_m()]));
    world
        .command_chassis(
            0,
            ChassisCommand {
                jump: true,
                forward_m_s: 2.,
                ..Default::default()
            },
        )
        .unwrap();
    world
        .command_chassis_wheel_torques(0, WheelTorques::new(vec![0.; 2]).unwrap())
        .unwrap();
    let saved = run(&mut world, 1);
    assert!(
        saved.velocity_m_s[2] < 0.1,
        "direct mode must bypass assisted jump"
    );
}

#[test]
fn compact_bumper_extends_the_lower_deck_and_stays_inside_the_stowed_width() {
    let cfg = ChassisConfig::default();
    cfg.validate().unwrap();
    assert_eq!(cfg.body_outline_m().len(), 8);
    let prisms = cfg.bumper_prisms_m();
    assert_eq!(prisms.len(), 56);
    for prism in &prisms {
        for p in prism {
            assert!(p[0].abs() <= 0.285 && p[1].abs() <= 0.285);
        }
    }
    let frame = cfg.omni_frame.as_ref().unwrap();
    for prism in &prisms[8..] {
        for (i, p) in prism.iter().enumerate() {
            let z = frame.ring_center_z_m + if i < 4 { -0.5 } else { 0.5 } * frame.ring_height_m;
            assert!((p[2] - z).abs() < 1e-12);
        }
    }
    for prism in &prisms[..8] {
        for p in &prism[4..] {
            assert_eq!(p[2], -cfg.body_half_m[2]);
        }
    }
    // A ray into a diagonal corner misses the deck box's old square corner,
    // while a horizontal ray at bumper height hits the compact lower extrusion.
    let deck: Vec<_> = cfg
        .body_outline_m()
        .iter()
        .flat_map(|&[x, y]| [Vector::new(x, y, -0.05), Vector::new(x, y, 0.05)])
        .collect();
    let deck = SharedShape::convex_hull(&deck).unwrap();
    assert!(!deck.contains_point(&Pose3::IDENTITY, Vector::new(0.20, 0.20, 0.0)));
    let ring = SharedShape::compound(
        prisms
            .iter()
            .map(|p| {
                (
                    Pose3::IDENTITY,
                    SharedShape::convex_hull(&p.map(Vector::from_array)).unwrap(),
                )
            })
            .collect(),
    );
    let ray = Ray::new(Vector::new(0.5, 0.0, -0.07), -Vector::X);
    let distance = ring.cast_ray(&Pose3::IDENTITY, &ray, 1.0, true).unwrap();
    assert!((distance - 0.215).abs() < 1e-9);
    assert!(ring.contains_point(&Pose3::IDENTITY, Vector::new(0.0, 0.0, -0.07)));
}

#[test]
fn lower_bumper_and_guards_clear_every_suspended_wheel() {
    let cfg = ChassisConfig::default();
    for wheel in wheel_geometry(&cfg) {
        for prism in cfg.bumper_prisms_m() {
            let points = prism
                .iter()
                .map(|p| Vector::new(p[0], p[1], 0.0))
                .collect::<Vec<_>>();
            let mut axes = vec![wheel.axle, wheel.roll];
            for (i, point) in points.iter().enumerate() {
                for other in &points[i + 1..] {
                    let edge = other - point;
                    if edge.length_squared() > 1e-20 {
                        axes.push(Vector::new(-edge.y, edge.x, 0.0).normalize());
                    }
                }
            }
            assert!(
                axes.into_iter().any(|axis| {
                    let centre = wheel.hub_m.dot(axis);
                    let radius = 0.5 * cfg.wheel_width_m * wheel.axle.dot(axis).abs()
                        + cfg.wheel_radius_m * wheel.roll.dot(axis).abs();
                    let low = points
                        .iter()
                        .map(|p| p.dot(axis))
                        .fold(f64::INFINITY, f64::min);
                    let high = points
                        .iter()
                        .map(|p| p.dot(axis))
                        .fold(f64::NEG_INFINITY, f64::max);
                    low - (centre + radius) > 0.002 || centre - radius - high > 0.002
                }),
                "bumper clips the wheel's swept envelope"
            );
        }
    }
}

#[test]
fn armor_carriers_are_recessed_rigid_and_leave_the_ring_as_first_wall_contact() {
    let cfg = ChassisConfig::default();
    let mut armor_vertices = Vec::new();
    for (plate, face) in cfg.armor_faces().iter().enumerate() {
        let face = rapier_pose(*face);
        let prisms = cfg.armor_bracket_prisms_m(plate);
        assert_eq!(prisms.len(), if plate.is_multiple_of(2) { 4 } else { 3 });
        for prism in &prisms {
            for &point in prism {
                let local = face.inverse_transform_point(Vector::from_array(point));
                assert!(
                    local.x <= -0.019 + 1e-12,
                    "carrier protrudes through armor: {local:?}"
                );
            }
        }
        // The carrier foot is horizontal and recessed onto the lower deck.
        for p in &prisms.last().unwrap()[..4] {
            assert!((p[2] + cfg.body_half_m[2]).abs() < 1e-12);
        }
        let [hx, hy, hz] = SMALL_ARMOR_HOUSING_HALF_M;
        for x in [-2.0 * hx, 0.0] {
            for y in [-hy, hy] {
                for z in [-hz, hz] {
                    armor_vertices.push(face.transform_point(Vector::new(x, y, z)));
                }
            }
        }
    }
    let ring: Vec<_> = cfg
        .bumper_prisms_m()
        .into_iter()
        .flatten()
        .map(Vector::from_array)
        .collect();
    // Every horizontal wall orientation must meet the bumper first, even
    // after including the plate tilt and full physical housing corners.
    for degree in 0..360 {
        let a = f64::from(degree).to_radians();
        let n = Vector::new(a.cos(), a.sin(), 0.0);
        let support = |vertices: &[Vector]| {
            vertices
                .iter()
                .map(|v| v.dot(n))
                .fold(f64::NEG_INFINITY, f64::max)
        };
        assert!(support(&ring) - support(&armor_vertices) > 0.005);
    }
}

#[test]
fn diagonal_wheels_clear_the_low_cardinal_armor_through_suspension_travel() {
    let cfg = ChassisConfig::default();
    for geometry in wheel_geometry(&cfg) {
        assert!((geometry.hub_m.x.abs() - geometry.hub_m.y.abs()).abs() < 1e-12);
        assert!((geometry.hub_m.x.hypot(geometry.hub_m.y) - 0.245).abs() < 1e-12);
        // Separation in each armor plate's width direction is independent
        // of wheel height, so this holds for full droop and compression.
        for width_axis in [Vector::X, Vector::Y] {
            let tyre_near = geometry.hub_m.dot(width_axis).abs()
                - cfg.wheel_radius_m * geometry.roll.dot(width_axis).abs()
                - 0.5 * cfg.wheel_width_m * geometry.axle.dot(width_axis).abs();
            assert!(tyre_near - SMALL_ARMOR_HOUSING_HALF_M[1] > 0.025);
        }
    }
    assert_eq!(cfg.armor_height_m, 0.030);
}

#[test]
fn omni_opposing_armor_pairs_follow_the_stacked_beam_heights() {
    let cfg = ChassisConfig::default();
    let faces = cfg.armor_faces();
    assert_eq!(faces[0].translation_m[2], faces[2].translation_m[2]);
    assert_eq!(faces[1].translation_m[2], faces[3].translation_m[2]);
    assert!(
        (faces[0].translation_m[2] - faces[1].translation_m[2] - cfg.frame_beam_step_m()).abs()
            < 1e-12
    );
    let hero = ChassisConfig::hero().armor_faces();
    assert!(
        hero.iter()
            .all(|p| p.translation_m[2] == hero[0].translation_m[2])
    );
}

#[test]
fn omni_springs_are_inboard_below_the_roof_and_follow_each_wheels_heave() {
    let cfg = ChassisConfig::default();
    for (wheel, hub) in cfg.wheel_hubs_m.iter().enumerate() {
        let [upper, lower] = cfg.suspension_mounts_m(wheel, 0.05).unwrap();
        assert!(upper[0].hypot(upper[1]) + 0.014 < hub[0].hypot(hub[1]) - cfg.wheel_width_m / 2.0);
        assert!(upper[2] + 0.014 < cfg.body_half_m[2]);
        assert!(lower[2] > -0.08);
        let [next_upper, next_lower] = cfg.suspension_mounts_m(wheel, 0.075).unwrap();
        assert_eq!(upper, next_upper);
        assert_eq!(&lower[..2], &next_lower[..2]);
        assert!((next_lower[2] - lower[2] - 0.025).abs() < 1e-12);
    }
}

#[test]
fn airborne_motors_spin_up_with_inertia_and_restore_exactly() {
    let cfg = ChassisConfig::default();
    let mut physics = chassis_world(cfg.clone(), Pose::at([0.0, 0.0, 20.0]));
    physics
        .command_chassis(
            0,
            ChassisCommand {
                forward_m_s: 2.0,
                ..Default::default()
            },
        )
        .unwrap();
    let first = run(&mut physics, 1);
    let rate = first.wheels[1].angular_velocity_rad_s.abs();
    assert!(rate > 0.0 && rate < 2.0 / cfg.wheel_radius_m);
    let state = run(&mut physics, 32);
    assert!(
        state
            .wheels
            .iter()
            .all(|w| w.contact.is_none() && w.suspension_compression_m == 0.0)
    );
    assert!(state.wheels[1].angular_velocity_rad_s.abs() > rate);
    assert!(state.velocity_m_s[0].abs() < 1e-12);
    let mut restored = WorldPhysics::new(&[], 0.0);
    restored.restore_chassis(&state).unwrap();
    assert_eq!(restored.chassis_snapshots()[0], state);
    let a = run(&mut physics, 32);
    let b = run(&mut restored, 32);
    for (a, b) in a.wheels.iter().zip(b.wheels) {
        assert_eq!(a.angular_velocity_rad_s, b.angular_velocity_rad_s);
        assert_eq!(a.spin_rad, b.spin_rad);
    }
}

#[test]
fn low_grip_spins_wheels_without_inventing_traction() {
    let cfg = ChassisConfig {
        drive_friction: 0.015,
        ..Default::default()
    };
    let mut physics = chassis_world(cfg.clone(), Pose::at([0.0, 0.0, cfg.rest_height_m()]));
    run(&mut physics, 256);
    physics
        .command_chassis(
            0,
            ChassisCommand {
                forward_m_s: 2.5,
                ..Default::default()
            },
        )
        .unwrap();
    let state = run(&mut physics, 64);
    assert!(state.velocity_m_s[0] < 0.3, "{state:?}");
    for i in 0..4 {
        let w = state.wheels[i];
        let c = w.contact.unwrap();
        assert!(w.angular_velocity_rad_s.abs() * cfg.wheel_radius_m > 1.0);
        assert!(c.slip_m_s.abs() > 0.8);
        assert!(c.drive_force_n.abs() <= cfg.drive_friction * c.load_n + 1e-8);
    }
}

#[test]
fn a_loose_projectile_does_not_become_a_suspension_support() {
    let cfg = ChassisConfig::default();
    let mut physics = chassis_world(cfg.clone(), Pose::at([0.0, 0.0, cfg.rest_height_m()]));
    physics
        .fire(
            0,
            Pose::at([cfg.wheel_hubs_m[0][0], cfg.wheel_hubs_m[0][1], 0.030]),
            Shot {
                caliber: Caliber::Mm17,
                speed_m_s: 0.1,
            },
            None,
        )
        .unwrap();
    let state = run(&mut physics, 1);
    let front = state.wheels[0];
    assert!(front.suspension_compression_m < 0.001, "{front:?}");
    assert!(front.contact.unwrap().point_m[2].abs() < 1e-9);
}

#[test]
fn each_suspension_responds_to_its_own_ground_height() {
    let cfg = ChassisConfig::default();
    let mut physics = chassis_world(cfg.clone(), Pose::at([0.0, 0.0, cfg.rest_height_m()]));
    let flat = run(&mut physics, 256);
    let [x, y] = cfg.wheel_hubs_m[0];
    physics
        .add_static_mesh(
            vec![
                [x - 0.04, y - 0.04, 0.025],
                [x + 0.04, y - 0.04, 0.025],
                [x + 0.04, y + 0.04, 0.025],
                [x - 0.04, y + 0.04, 0.025],
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        )
        .unwrap();
    let raised = run(&mut physics, 1);
    assert!(
        raised.wheels[0].suspension_compression_m > flat.wheels[0].suspension_compression_m + 0.02
    );
    assert!(
        raised.wheels[0].contact.unwrap().load_n > raised.wheels[2].contact.unwrap().load_n + 15.0
    );
    assert!((raised.wheels[0].hub_m[2] - raised.wheels[2].hub_m[2] - 0.025).abs() < 0.003);
}

#[test]
fn wheel_reaction_and_shared_motor_budget_are_independent_per_corner() {
    let cfg = ChassisConfig::default();
    let initial = WheelState {
        spin_rad: 0.0,
        angular_velocity_rad_s: 0.0,
        suspension_compression_m: 0.0,
        target_m_s: 2.0,
        contact: None,
    };
    let mut wheels = vec![initial; 4];
    let contacts = [(0.0, 0.0), (0.0, 100.0), (0.0, 100.0), (0.0, 100.0)];
    for _ in 0..64 {
        integrate_wheel_drives(
            &cfg,
            &mut wheels,
            &contacts,
            1.0 / 128.0,
            cfg.dynamics.drive_power_w,
            None,
        );
    }
    assert!(wheels[0].angular_velocity_rad_s > wheels[1].angular_velocity_rad_s + 5.0);
    assert_eq!(
        wheels[1].angular_velocity_rad_s,
        wheels[2].angular_velocity_rad_s
    );
    let mut restricted = cfg.clone();
    restricted.dynamics.drive_power_w = 10.0;
    let mut low = vec![initial; 4];
    let mut high = vec![initial; 4];
    for _ in 0..16 {
        integrate_wheel_drives(
            &restricted,
            &mut low,
            &contacts,
            1.0 / 128.0,
            restricted.dynamics.drive_power_w,
            None,
        );
        integrate_wheel_drives(
            &cfg,
            &mut high,
            &contacts,
            1.0 / 128.0,
            cfg.dynamics.drive_power_w,
            None,
        );
    }
    assert!(low[0].angular_velocity_rad_s < high[0].angular_velocity_rad_s);
}

#[test]
fn gimbal_motor_is_tick_driven_rate_limited_and_restorable() {
    let config = ChassisConfig::default();
    let mut physics = chassis_world(config.clone(), Pose::at([0., 0., config.rest_height_m()]));
    physics
        .command_chassis(
            0,
            ChassisCommand {
                aim_yaw_rad: 1.,
                aim_pitch_rad: 0.4,
                ..Default::default()
            },
        )
        .unwrap();
    assert_eq!(physics.chassis_snapshots()[0].held_aim_rad, [0.; 2]);
    let frames = TargetFrames::new(Vec::new());
    let mut previous_rate = [0.; 2];
    let tick_s = tick_ns() as f64 * 1e-9;
    for tick in 0..200 {
        physics.step(tick * tick_ns(), &frames).unwrap();
        let motor = physics.chassis_snapshots()[0].gimbal_velocity_rad_s;
        for axis in 0..2 {
            assert!(motor[axis].abs() <= config.dynamics.gimbal_max_speed_rad_s + 1e-10);
            assert!(
                (motor[axis] - previous_rate[axis]).abs()
                    <= config.dynamics.gimbal_max_acceleration_rad_s2 * tick_s + 1e-10
            );
        }
        previous_rate = motor;
    }
    let saved = physics.chassis_snapshots().remove(0);
    let mut restored = chassis_world(config, saved.pose);
    restored.reset_chassis(&saved).unwrap();
    for tick in 200..500 {
        physics.step(tick * tick_ns(), &frames).unwrap();
        restored.step(tick * tick_ns(), &frames).unwrap();
        assert_eq!(
            physics.chassis_snapshots()[0].held_aim_rad,
            restored.chassis_snapshots()[0].held_aim_rad
        );
        assert_eq!(
            physics.chassis_snapshots()[0].gimbal_velocity_rad_s,
            restored.chassis_snapshots()[0].gimbal_velocity_rad_s
        );
    }
    assert!((physics.chassis_snapshots()[0].held_aim_rad[0] - 1.).abs() < 0.01);
    physics.set_chassis_defeated(0, true);
    let frozen = physics.chassis_snapshots()[0].held_aim_rad;
    physics.step(500 * tick_ns(), &frames).unwrap();
    assert_eq!(physics.chassis_snapshots()[0].held_aim_rad, frozen);
    assert_eq!(
        physics.chassis_snapshots()[0].gimbal_velocity_rad_s,
        [0.; 2]
    );
}

#[test]
fn gimbal_takes_short_yaw_path_across_wrap() {
    let config = ChassisConfig::default();
    let mut physics = chassis_world(
        config.clone(),
        Pose::yawed([0., 0., config.rest_height_m()], std::f64::consts::PI - 0.1),
    );
    physics
        .command_chassis(
            0,
            ChassisCommand {
                aim_yaw_rad: -std::f64::consts::PI + 0.1,
                ..Default::default()
            },
        )
        .unwrap();
    let state = run(&mut physics, 1);
    assert!(state.gimbal_velocity_rad_s[0] > 0.);
    let state = run(&mut physics, 1000);
    assert!(wrap_angle(state.held_aim_rad[0] - (-std::f64::consts::PI + 0.1)).abs() < 1e-6);
}

#[test]
fn landing_settles_without_gaining_bounce_height() {
    let config = ChassisConfig::default();
    let mut physics = chassis_world(config.clone(), Pose::at([0., 0., 0.6]));
    let frames = TargetFrames::new(Vec::new());
    let mut highest = 0.6_f64;
    for tick in 0..3000 {
        physics.step(tick * tick_ns(), &frames).unwrap();
        highest = highest.max(physics.chassis_snapshots()[0].pose.translation_m[2]);
    }
    let state = physics.chassis_snapshots().remove(0);
    assert!(highest <= 0.601, "landing gained height: {highest}");
    let sag = config.mass_kg * 9.81 / 4.0 / config.suspension_stiffness_n_m;
    assert!((state.pose.translation_m[2] - (config.rest_height_m() - sag)).abs() < 0.002);
    assert!(state.velocity_m_s.iter().all(|v| v.abs() < 0.01));
    assert!(state.wheels.iter().all(|w| w.contact.is_some()));
}

#[test]
fn shared_power_budget_charges_stall_losses_and_has_no_regeneration_credit() {
    for mechanical in [0., 40., 200.] {
        for copper in [0., 60., 400.] {
            let scale = drive_power_scale(mechanical, copper, 80.);
            assert!((0.0..=1.0).contains(&scale));
            assert!(scale * mechanical + scale * scale * copper <= 80. + 1e-10);
        }
    }
    assert!(drive_power_scale(0., 100., 80.) < 1.);
}

#[test]
fn muzzle_uses_rotated_body_pivot_and_freezes_held_aim_when_defeated() {
    let config = ChassisConfig::default();
    let spawn = Pose::yawed(
        [2.0, 3.0, config.rest_height_m()],
        std::f64::consts::FRAC_PI_2,
    );
    let mut physics = chassis_world(config.clone(), spawn);
    physics
        .command_chassis(
            0,
            ChassisCommand {
                aim_yaw_rad: std::f64::consts::FRAC_PI_2,
                aim_pitch_rad: 0.0,
                ..Default::default()
            },
        )
        .unwrap();
    let aimed = physics.chassis_muzzle_pose(0).unwrap();
    let expected_pivot = rapier_pose(spawn).transform_point(vector(config.turret_center_m));
    assert!((aimed.translation_m[0] - expected_pivot.x).abs() < 1e-12);
    assert!((aimed.translation_m[1] - (expected_pivot.y + MUZZLE_FORWARD_M)).abs() < 1e-12);
    assert!((aimed.translation_m[2] - expected_pivot.z).abs() < 1e-12);

    physics.set_chassis_defeated(0, true);
    physics
        .command_chassis(
            0,
            ChassisCommand {
                aim_yaw_rad: 0.0,
                aim_pitch_rad: 0.4,
                ..Default::default()
            },
        )
        .unwrap();
    assert_eq!(physics.chassis_muzzle_pose(0), Some(aimed));
    assert_eq!(physics.chassis_muzzle_pose(99), None);
}
fn run(ballistics: &mut WorldPhysics, ticks: u64) -> ChassisSnapshot {
    let frames = TargetFrames::new(Vec::new());
    for tick in 0..ticks {
        ballistics.step(tick * tick_ns(), &frames).unwrap();
    }
    ballistics.chassis_snapshots().remove(0)
}
#[test]
fn drone_stays_in_its_plane_and_restores_its_drive() {
    let config = ChassisConfig::drone();
    config.validate().unwrap();
    let mut physics = chassis_world(config, Pose::at([0.0, 0.0, 1.6]));
    physics
        .command_chassis(
            0,
            ChassisCommand {
                forward_m_s: 1.0,
                left_m_s: 0.5,
                yaw_rate_rad_s: 0.2,
                jump: true,
                ..Default::default()
            },
        )
        .unwrap();
    let state = run(&mut physics, 128);
    assert!(state.pose.translation_m[0] > 0.6);
    assert!(state.pose.translation_m[1] > 0.3);
    assert!((state.pose.translation_m[2] - 1.6).abs() < 1e-10);
    assert!(state.wheels.is_empty());
    assert!(state.pose.rotation_wxyz[1].abs() < 1e-10);
    assert!(state.pose.rotation_wxyz[2].abs() < 1e-10);
    let mut restored = WorldPhysics::new(&[], 0.0);
    restored.restore_chassis(&state).unwrap();
    let original = run(&mut physics, 128);
    let replayed = run(&mut restored, 128);
    for (a, b) in original
        .pose
        .translation_m
        .iter()
        .zip(replayed.pose.translation_m)
    {
        assert!((a - b).abs() < 1e-8);
    }
}

#[test]
fn drone_shots_clear_its_own_frame_at_every_forward_pitch() {
    let config = ChassisConfig::drone();
    let mut physics = chassis_world(config, Pose::at([0.0, 0.0, 2.0]));
    let frames = TargetFrames::new(Vec::new());
    let mut time_ns = 0;
    // The flight controls allow -1.2 rad down to 0.79 rad up.
    for step in 0..=20 {
        let pitch = -1.2 + (0.79 + 1.2) * f64::from(step) / 20.0;
        physics
            .command_chassis(
                0,
                ChassisCommand {
                    aim_pitch_rad: pitch,
                    ..Default::default()
                },
            )
            .unwrap();
        for _ in 0..128 {
            time_ns += tick_ns();
            physics.step(time_ns, &frames).unwrap();
        }
        let muzzle = physics.chassis_muzzle_pose(0).unwrap();
        let id = physics
            .fire(time_ns, muzzle, Shot::at_limit(Caliber::Mm17), Some(0))
            .unwrap();
        for _ in 0..8 {
            time_ns += tick_ns();
            physics.step(time_ns, &frames).unwrap();
        }
        let ball = physics.snapshot().into_iter().find(|b| b.id == id).unwrap();
        assert_eq!(
            ball.first_contact_ns, None,
            "pitch {pitch:.2} rad hit the drone"
        );
    }
}

#[test]
fn tethered_drone_stops_at_its_reach_and_is_pulled_back_inside() {
    let tether = Tether {
        rope_start_m: [14.0, -5.8, 3.6],
        rope_end_m: [0.0, -5.8, 3.6],
        length_m: 2.4,
    };
    let config = ChassisConfig {
        tether: Some(tether),
        ..ChassisConfig::drone()
    };
    config.validate().unwrap();
    // Facing -x from above the pad, full speed towards the Snap Ring and
    // beyond it: the drone must brake to a stop within the tether's reach.
    let spawn = Pose::yawed([10.0, -5.8, 2.0], std::f64::consts::PI);
    let mut physics = chassis_world(config.clone(), spawn);
    physics
        .command_chassis(
            0,
            ChassisCommand {
                forward_m_s: 4.0,
                aim_yaw_rad: std::f64::consts::PI,
                ..Default::default()
            },
        )
        .unwrap();
    let mut deepest = f64::INFINITY;
    let frames = TargetFrames::new(Vec::new());
    for tick in 0..128 * 8 {
        physics.step(tick * tick_ns(), &frames).unwrap();
        let hook = physics.chassis_snapshots()[0].pose.translation_m;
        deepest = deepest.min(tether.slack_m(hook));
    }
    let state = physics.chassis_snapshots().remove(0);
    // Beyond the ring by the tether's horizontal reach at 1.6 m below the rope.
    let reach = (2.4_f64.powi(2) - 1.6_f64.powi(2)).sqrt();
    assert!((state.pose.translation_m[0] + reach).abs() < 0.05);
    assert!(state.velocity_m_s[0].abs() < 0.05);
    assert!(deepest > -0.05, "overshot the tether by {deepest} m");

    // A drone placed well outside its reach is pulled back without the
    // pilot's help, and the pilot cannot hold it out.
    let mut outside = chassis_world(config, Pose::at([7.0, -1.0, 2.0]));
    outside
        .command_chassis(
            0,
            ChassisCommand {
                left_m_s: 3.0,
                ..Default::default()
            },
        )
        .unwrap();
    let state = run(&mut outside, 128 * 6);
    assert!(tether.slack_m(state.pose.translation_m) > -0.05);
    assert!((state.pose.translation_m[2] - 2.0).abs() < 1e-10);
}

#[test]
fn balance_stabilization_can_be_disabled_and_restored() {
    let pose = Pose {
        translation_m: [0.0, 0.0, 0.31],
        rotation_wxyz: [0.05_f64.cos(), 0.0, 0.05_f64.sin(), 0.0],
    };
    let mut assisted = chassis_world(ChassisConfig::balance(), pose);
    let mut unassisted = chassis_world(ChassisConfig::balance(), pose);
    unassisted
        .command_chassis(
            0,
            ChassisCommand {
                balance_control: 0,
                ..Default::default()
            },
        )
        .unwrap();
    let stable = run(&mut assisted, 128);
    let fallen = run(&mut unassisted, 128);
    assert!(stable.pose.rotation_wxyz[2].abs() < 0.025, "{stable:?}");
    assert!(fallen.pose.rotation_wxyz[2].abs() > 0.10, "{fallen:?}");
    assert!(
        unassisted
            .command_chassis(
                0,
                ChassisCommand {
                    balance_control: 101,
                    ..Default::default()
                }
            )
            .is_err()
    );
    let mut restored = WorldPhysics::new(&[], 0.0);
    restored.restore_chassis(&fallen).unwrap();
    assert_eq!(restored.chassis_snapshots()[0].command.balance_control, 0);
}

#[test]
fn balance_recovers_pitch_drives_and_jumps_once_per_press() {
    let config = ChassisConfig::balance();
    config.validate().unwrap();
    let mut spawn = Pose::at([0., 0., config.rest_height_m()]);
    let angle: f64 = 0.10;
    spawn.rotation_wxyz = [(angle / 2.).cos(), 0., (angle / 2.).sin(), 0.];
    let mut physics = chassis_world(config, spawn);
    let settled = run(&mut physics, 256);
    assert!(settled.pose.rotation_wxyz[2].abs() < 0.03, "{settled:?}");
    assert!(settled.pose.translation_m[2] > 0.20);
    physics
        .command_chassis(
            0,
            ChassisCommand {
                forward_m_s: 1.0,
                ..Default::default()
            },
        )
        .unwrap();
    let driven = run(&mut physics, 128);
    assert!(
        driven.pose.translation_m[0] > settled.pose.translation_m[0] + 0.35,
        "{driven:?}"
    );
    physics
        .command_chassis(0, ChassisCommand::default())
        .unwrap();
    let grounded = run(&mut physics, 256);
    physics
        .command_chassis(
            0,
            ChassisCommand {
                jump: true,
                ..Default::default()
            },
        )
        .unwrap();
    let airborne = run(&mut physics, 22);
    assert!(
        airborne.pose.translation_m[2] > grounded.pose.translation_m[2] + 0.12,
        "{airborne:?}"
    );
    assert!(airborne.jump_held);
    let mut restored = WorldPhysics::new(&[], 0.0);
    restored.restore_chassis(&airborne).unwrap();
    let replayed = run(&mut restored, 256);
    assert!((replayed.pose.translation_m[2] - grounded.pose.translation_m[2]).abs() < 0.03);

    let landed = run(&mut physics, 256);
    assert!(
        (landed.pose.translation_m[2] - grounded.pose.translation_m[2]).abs() < 0.03,
        "held jump repeated: {landed:?}"
    );
    physics
        .command_chassis(0, ChassisCommand::default())
        .unwrap();
    run(&mut physics, 1);
    physics
        .command_chassis(
            0,
            ChassisCommand {
                jump: true,
                ..Default::default()
            },
        )
        .unwrap();
    assert!(run(&mut physics, 22).pose.translation_m[2] > grounded.pose.translation_m[2] + 0.12);
}

#[test]
fn jump_cannot_launch_omni_or_defeated_balance() {
    for (config, defeated) in [
        (ChassisConfig::default(), false),
        (ChassisConfig::balance(), true),
    ] {
        let mut physics = chassis_world(config.clone(), Pose::at([0., 0., config.rest_height_m()]));
        let before = run(&mut physics, 128);
        physics.set_chassis_defeated(0, defeated);
        physics
            .command_chassis(
                0,
                ChassisCommand {
                    jump: true,
                    ..Default::default()
                },
            )
            .unwrap();
        let after = run(&mut physics, 22);
        assert!(after.pose.translation_m[2] < before.pose.translation_m[2] + 0.02);
    }
}

#[test]
fn source_chain_turret_telemetry_matches_full_pose_evaluation() {
    use crate::mechanism::{ArticulationConfig, JointDrive, SourceJoint};
    let mut config = ChassisConfig::default();
    config.articulation = Some(ArticulationConfig {
        profile: "telemetry-source-chain-fixture".into(),
        limit_rad: 1.,
        speed_rad_s: 0.5,
        muzzle_offset_m: [0.2, 0., 0.],
        camera_offset_m: [0.1, 0., 0.],
        armor_faces: config.armor_faces(),
        joints: vec![
            SourceJoint {
                name: "mount".into(),
                parent: None,
                pivot_m: [0.1, -0.2, 0.15],
                axis: [0., 1., 0.],
                angle_rad: 0.5,
                translation_m: [0.02, 0.01, -0.03],
                drive: JointDrive::Transform,
                collision: None,
            },
            SourceJoint {
                name: "yaw".into(),
                parent: Some(0),
                pivot_m: [0.04, 0.02, 0.2],
                axis: [0., 0., 1.],
                angle_rad: 1.,
                translation_m: [0.; 3],
                drive: JointDrive::Yaw,
                collision: None,
            },
            SourceJoint {
                name: "pitch".into(),
                parent: Some(1),
                pivot_m: [0.1, 0.03, 0.25],
                axis: [0., 1., 0.],
                angle_rad: -1.,
                translation_m: [0.; 3],
                drive: JointDrive::Pitch,
                collision: None,
            },
        ],
    });
    let mut world = crate::WorldPhysics::new(&[], 0.);
    let id = world
        .add_chassis(Team::Red, config, Pose::yawed([1., -2., 0.4], 0.7))
        .unwrap();
    world
        .command_chassis(
            id,
            ChassisCommand {
                folded: true,
                aim_yaw_rad: -0.4,
                aim_pitch_rad: 0.2,
                ..Default::default()
            },
        )
        .unwrap();
    let frames = crate::TargetFrames::new(vec![]);
    for tick in 0..8 {
        world.step(tick * crate::tick_ns(), &frames).unwrap();
    }
    let full = world.chassis_snapshot(id).unwrap();
    let articulation = full.config.articulation.as_ref().unwrap();
    let body = rapier_pose(full.pose);
    let local = body.rotation.inverse()
        * Vector::new(
            full.held_aim_rad[1].cos() * full.held_aim_rad[0].cos(),
            full.held_aim_rad[1].cos() * full.held_aim_rad[0].sin(),
            full.held_aim_rad[1].sin(),
        );
    let poses = articulation.poses(
        full.fold_angle_rad,
        [local.y.atan2(local.x), local.z.clamp(-1., 1.).asin()],
    );
    let expected = body * rapier_pose(poses[2]);
    let q = expected.rotation;
    let expected = Pose {
        translation_m: expected.translation.to_array(),
        rotation_wxyz: [q.w, q.x, q.y, q.z],
    };
    let view = world.chassis_telemetry(id).unwrap();
    assert_eq!(view.turret, expected);
    assert_eq!(view.turret, full.turret);
    assert_eq!(view.wheels().collect::<Vec<_>>(), full.wheels);
}

#[test]
fn default_configuration_is_valid_and_bad_ones_are_rejected() {
    let config = ChassisConfig::default();
    assert!(config.validate().is_ok());
    assert!((config.rest_height_m() - 0.2065).abs() < 1e-9);
    assert!(
        ChassisConfig {
            mass_kg: 0.0,
            ..config.clone()
        }
        .validate()
        .is_err()
    );
    assert!(
        ChassisConfig {
            wheel_hubs_m: vec![[0.0, 0.0]],
            ..config.clone()
        }
        .validate()
        .is_err()
    );
    assert!(
        ChassisConfig {
            roller_friction: -0.1,
            ..config.clone()
        }
        .validate()
        .is_err()
    );
    let mut world = PhysicsWorld::new();
    assert!(
        Chassis::new(
            &mut world,
            0,
            Team::Red,
            config,
            Pose {
                rotation_wxyz: [0.0; 4],
                ..Pose::default()
            }
        )
        .is_err()
    );
    assert!((yaw_of(Pose::yawed([0.0; 3], 0.7)) - 0.7).abs() < 1e-12);
    assert!((wrap_angle(7.0) - (7.0 - std::f64::consts::TAU)).abs() < 1e-12);
}
#[test]
fn chassis_settles_on_the_floor_with_all_wheels_loaded() {
    let config = ChassisConfig::default();
    let mut ballistics = chassis_world(
        config.clone(),
        Pose::at([0.0, 0.0, config.rest_height_m() + 0.05]),
    );
    let snapshot = run(&mut ballistics, 1_500);
    let z = snapshot.pose.translation_m[2];
    // Each spring carries a quarter of the weight.
    let sag = config.mass_kg * 9.81 / 4.0 / config.suspension_stiffness_n_m;
    assert!((z - (config.rest_height_m() - sag)).abs() < 0.001, "{z}");
    assert!(snapshot.velocity_m_s.iter().all(|v| v.abs() < 1e-3));
    assert_eq!(snapshot.wheels.len(), 4);
    for wheel in &snapshot.wheels {
        let contact = wheel.contact.expect("wheel on the ground");
        assert!((contact.load_n - config.mass_kg * 9.81 / 4.0).abs() < 1.0);
        assert!(contact.point_m[2].abs() < 1e-6);
        assert!((contact.normal[2] - 1.0).abs() < 1e-9);
    }
    assert!((yaw_of(snapshot.pose)).abs() < 1e-6);
}
#[test]
fn forward_left_and_yaw_commands_move_the_body_holonomically() {
    for config in [ChassisConfig::default(), ChassisConfig::hero()] {
        let spawn = Pose::at([0.0, 0.0, config.rest_height_m()]);
        for (command, check) in [
            (
                ChassisCommand {
                    forward_m_s: 1.0,
                    ..Default::default()
                },
                (|s: &ChassisSnapshot| {
                    s.pose.translation_m[0] > 1.5
                        && s.pose.translation_m[1].abs() < 0.05
                        && (s.velocity_m_s[0] - 1.0).abs() < 0.05
                        && yaw_of(s.pose).abs() < 0.05
                }) as fn(&ChassisSnapshot) -> bool,
            ),
            (
                ChassisCommand {
                    left_m_s: 1.0,
                    ..Default::default()
                },
                |s| {
                    s.pose.translation_m[1] > 1.5
                        && s.pose.translation_m[0].abs() < 0.05
                        && yaw_of(s.pose).abs() < 0.05
                },
            ),
            (
                ChassisCommand {
                    yaw_rate_rad_s: 1.0,
                    ..Default::default()
                },
                |s| {
                    s.pose.translation_m[0].abs() < 0.05
                        && s.pose.translation_m[1].abs() < 0.05
                        && (s.angular_velocity_rad_s[2] - 1.0).abs() < 0.05
                },
            ),
        ] {
            let mut ballistics = chassis_world(config.clone(), spawn);
            run(&mut ballistics, 300);
            ballistics.command_chassis(0, command).unwrap();
            let snapshot = run(&mut ballistics, 2_000);
            assert!(check(&snapshot), "{command:?}: {snapshot:?}");
            assert_eq!(snapshot.command, command);
            // Wheels spin with the ground speed they see.
            assert!(snapshot.wheels.iter().any(|w| w.spin_rad != 0.0));
        }
    }
}
#[test]
fn the_turret_rides_the_body_and_faces_the_commanded_aim() {
    let config = ChassisConfig::default();
    let mut ballistics =
        chassis_world(config.clone(), Pose::at([0.0, 0.0, config.rest_height_m()]));
    let rest = run(&mut ballistics, 300);
    let [x, y, z] = rest.turret.translation_m;
    assert!(x.abs() < 1e-6 && y.abs() < 1e-6, "{x} {y}");
    assert!((z - rest.pose.translation_m[2] - config.turret_center_m[2]).abs() < 1e-9);
    assert_eq!(rest.turret.rotation_wxyz, [1.0, 0.0, 0.0, 0.0]);
    // Aim left and up: the barrel (+x) points along +y, raised.
    ballistics
        .command_chassis(
            0,
            ChassisCommand {
                aim_yaw_rad: std::f64::consts::FRAC_PI_2,
                aim_pitch_rad: 0.5,
                ..Default::default()
            },
        )
        .unwrap();
    let aimed = run(&mut ballistics, 2_000);
    let barrel = rapier_pose(aimed.turret).rotation * Vector::X;
    assert!(barrel.x.abs() < 1e-9, "{barrel}");
    assert!((barrel.y - 0.5f64.cos()).abs() < 1e-9, "{barrel}");
    assert!((barrel.z - 0.5f64.sin()).abs() < 1e-9, "{barrel}");
    assert!(
        ballistics
            .command_chassis(
                0,
                ChassisCommand {
                    aim_pitch_rad: f64::INFINITY,
                    ..Default::default()
                }
            )
            .is_err()
    );
}
#[test]
fn motor_limit_caps_the_top_speed() {
    let config = ChassisConfig::default();
    let mut ballistics =
        chassis_world(config.clone(), Pose::at([0.0, 0.0, config.rest_height_m()]));
    run(&mut ballistics, 300);
    ballistics
        .command_chassis(
            0,
            ChassisCommand {
                forward_m_s: 10.0,
                ..Default::default()
            },
        )
        .unwrap();
    let snapshot = run(&mut ballistics, 4_000);
    let speed = snapshot.velocity_m_s[0];
    // Each diagonal wheel sees forward speed / sqrt(2).
    let top = config.wheel_no_load_speed_m_s * std::f64::consts::SQRT_2;
    assert!(speed > 0.8 * top && speed < top, "{speed}");
    assert!(
        ballistics
            .command_chassis(
                0,
                ChassisCommand {
                    forward_m_s: f64::NAN,
                    ..Default::default()
                }
            )
            .is_err()
    );
}
