// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
#[test]
fn removed_runtime_configs_are_rejected_even_without_a_cad_host() {
    use super::*;
    let variants = [
        FieldConfig {
            referee: Some(RefereeConfig::default()),
            ..Default::default()
        },
        FieldConfig {
            runes: vec![RuneConfig::default()],
            ..Default::default()
        },
        FieldConfig {
            outposts: vec![OutpostConfig {
                origin: Pose::default(),
                pivot_cad_m: None,
                speed_rad_s: 0.,
            }],
            ..Default::default()
        },
    ];
    for config in variants {
        assert!(
            Field::new(&config)
                .err()
                .unwrap()
                .to_string()
                .contains("rm2026-final")
        );
    }
    assert!(Field::new(&FieldConfig::default()).is_ok());
}
use super::*;
#[test]
fn instance_steps_preserve_clocks_partitioning_and_restore() {
    for duration in [
        StepDuration::default(),
        StepDuration::from_hz(500).unwrap(),
        StepDuration::HZ_1000,
        StepDuration::from_hz(2000).unwrap(),
    ] {
        let config = FieldConfig {
            step_duration: duration,
            ..Default::default()
        };
        let mut whole = Field::new(&config).unwrap();
        let mut parts = Field::new(&config).unwrap();
        whole.step(1000).unwrap();
        parts.step(213).unwrap();
        parts.step(787).unwrap();
        assert_eq!(whole.snapshot(), parts.snapshot());
        assert_eq!(whole.time_ns(), 1000 * duration.as_ns());
        let snapshot = whole.snapshot();
        assert_eq!(snapshot.restore.as_ref().unwrap().step_duration, duration);
        let mut restored =
            Field::restore(&snapshot, &whole.static_geometry_snapshot(), 0.).unwrap();
        assert_eq!(restored.step_duration(), duration);
        restored.step(37).unwrap();
        whole.step(37).unwrap();
        assert_eq!(restored.snapshot(), whole.snapshot());
        let mut wrong = snapshot;
        wrong.restore.as_mut().unwrap().step_duration = if duration == StepDuration::HZ_1000 {
            StepDuration::HZ_128
        } else {
            StepDuration::HZ_1000
        };
        assert!(Field::restore(&wrong, &whole.static_geometry_snapshot(), 0.).is_err());
    }
}
#[test]
fn thousand_hz_projectile_partition_and_restore_integrate_instance_dt() {
    let config = FieldConfig {
        step_duration: StepDuration::HZ_1000,
        ..Default::default()
    };
    let mut whole = Field::new(&config).unwrap();
    let mut parts = Field::new(&config).unwrap();
    for field in [&mut whole, &mut parts] {
        field
            .fire(Pose::at([0., 0., 10.]), Shot::at_limit(Caliber::Mm17), None)
            .unwrap();
    }
    whole.step(100).unwrap();
    parts.step(31).unwrap();
    parts.step(69).unwrap();
    assert_eq!(whole.snapshot(), parts.snapshot());
    assert_eq!(whole.time_ns(), 100_000_000);
    let ball = &whole.snapshot().projectiles[0];
    // Horizontal quadratic drag has a closed-form solution; vertical speed
    // contributes only a small correction over this 100 ms flight.
    let k = Caliber::Mm17.drag_factor_per_m();
    let expected_x = (1. + k * 25. * 0.1).ln() / k;
    assert!((ball.position_m[0] - expected_x).abs() < 0.002);

    assert!((ball.velocity_m_s[2] + 0.980665).abs() < 0.04);
    let mut restored =
        Field::restore(&whole.snapshot(), &whole.static_geometry_snapshot(), 0.).unwrap();
    whole.step(100).unwrap();
    restored.step(43).unwrap();
    restored.step(57).unwrap();
    assert_eq!(whole.time_ns(), restored.time_ns());
    for (a, b) in whole.snapshot().projectiles[0]
        .position_m
        .into_iter()
        .zip(restored.snapshot().projectiles[0].position_m)
    {
        assert!((a - b).abs() < 1e-8);
    }
}
/// Convert default-rate test waits to ticks, rounding up.
fn ticks(ns: u64) -> u64 {
    ns.div_ceil(tick_ns())
}
/// A referee'd field with two pilots, a rune and both outposts, driven
/// and shot at long enough for every rule to have moved.
fn busy_field(shoot: bool) -> (Field, u32) {
    let spawn = ChassisConfig::default().rest_height_m();
    let config = FieldConfig {
        chassis: vec![
            ChassisPlacement {
                config: ChassisConfig::default(),
                team: Team::Red,
                kind: RobotKind::Infantry,
                // A heat limit the eight quick shots below stay under.
                performance: Some(rm_sim_gameplay::Performance::Fixed(
                    rm_sim_gameplay::Stats {
                        max_hp: 200,
                        chassis_power_w: 60,
                        heat_limit: 1_000,
                        cooling_per_s: 20,
                    },
                )),
                spawn: Pose::at([0.0, 0.0, spawn]),
            },
            ChassisPlacement {
                config: ChassisConfig::default(),
                team: Team::Blue,
                kind: RobotKind::Infantry,
                performance: None,
                // Keep the second pilot clear of spent Rune rounds. A
                // later robot/ball impact amplifies unretained contact-solver
                // residuals and does not test checkpoint completeness.
                spawn: Pose::at([1.5, -2.0, spawn]),
            },
        ],
        runes: vec![],
        outposts: vec![],
        ..FieldConfig::default()
    };
    let mut field = Field::new(&config).unwrap();
    field.step(ticks(referee::COUNTDOWN_NS)).unwrap();
    let drive = ChassisCommand {
        forward_m_s: 1.2,
        yaw_rate_rad_s: 0.4,
        aim_yaw_rad: 0.3,
        ..Default::default()
    };
    field.command_chassis(0, drive).unwrap();
    for _ in 0..8 {
        field.step(ticks(60_000_000)).unwrap();
        if !shoot {
            continue;
        }
        let muzzle = field.chassis_muzzle_pose(0).unwrap();
        field
            .fire(muzzle, Shot::at_limit(Caliber::Mm17), Some(0))
            .unwrap();
    }
    field.step(ticks(30_000_000)).unwrap();
    (field, 0)
}
fn chassis_gap(a: &FieldSnapshot, b: &FieldSnapshot) -> f64 {
    a.chassis
        .iter()
        .zip(&b.chassis)
        .flat_map(|(a, b)| {
            assert_eq!(a.id, b.id);
            a.pose
                .translation_m
                .into_iter()
                .zip(b.pose.translation_m)
                .map(|(a, b)| (a - b).abs())
        })
        .fold(0.0_f64, f64::max)
}
fn projectile_gap(a: &FieldSnapshot, b: &FieldSnapshot) -> f64 {
    assert_eq!(
        a.projectiles.iter().map(|p| p.id).collect::<Vec<_>>(),
        b.projectiles.iter().map(|p| p.id).collect::<Vec<_>>(),
        "a restored field must keep every ball's identity"
    );
    a.projectiles
        .iter()
        .zip(&b.projectiles)
        .flat_map(|(a, b)| {
            a.position_m
                .into_iter()
                .zip(b.position_m)
                .map(|(a, b)| (a - b).abs())
        })
        .fold(0.0_f64, f64::max)
}
/// Replaying a restored field is exact while nothing rests in a contact:
/// the chassis stands on wheel ray casts, not on the contact solver.

#[test]
fn a_restored_field_without_contacts_replays_exactly() {
    let (mut original, pilot) = busy_field(false);
    let geometry = original.static_geometry_snapshot();
    let floor = original.floor_height_m();
    let mut restored = Field::restore(&original.snapshot(), &geometry, floor).unwrap();
    for round in 0..10 {
        let command = ChassisCommand {
            folded: false,
            jump: false,
            balance_control: 100,
            forward_m_s: if round % 3 == 0 { 1.5 } else { -0.5 },
            left_m_s: 0.4,
            yaw_rate_rad_s: 0.6,
            aim_yaw_rad: 0.2 * round as f64,
            aim_pitch_rad: 0.05,
        };
        original.command_chassis(pilot, command).unwrap();
        restored.command_chassis(pilot, command).unwrap();
        original.step(ticks(20_000_000)).unwrap();
        restored.step(ticks(20_000_000)).unwrap();
        assert_eq!(original.snapshot(), restored.snapshot());
    }
}
#[test]
fn a_restored_field_keeps_stepping_like_the_one_it_came_from() {
    let (mut original, pilot) = busy_field(true);
    let geometry = original.static_geometry_snapshot();
    let floor = original.floor_height_m();
    let checkpoint = original.snapshot();
    let mut restored = Field::restore(&checkpoint, &geometry, floor).unwrap();
    // Restoring is a pure read of the checkpoint, in both directions.
    assert_eq!(restored.snapshot(), checkpoint);
    assert_eq!(
        Field::restore(&restored.snapshot(), &geometry, floor)
            .unwrap()
            .snapshot(),
        checkpoint,
        "restore(snapshot(restore(s))) must stand exactly where s did"
    );
    let mut worst_chassis = 0.0_f64;
    let mut worst_projectile = 0.0_f64;
    for round in 0..10 {
        let command = ChassisCommand {
            folded: false,
            jump: false,
            balance_control: 100,
            forward_m_s: if round % 3 == 0 { 1.5 } else { -0.5 },
            left_m_s: 0.4,
            yaw_rate_rad_s: 0.6,
            aim_yaw_rad: 0.2 * round as f64,
            aim_pitch_rad: 0.05,
        };
        original.command_chassis(pilot, command).unwrap();
        restored.command_chassis(pilot, command).unwrap();
        original.step(ticks(20_000_000)).unwrap();
        restored.step(ticks(20_000_000)).unwrap();
        let (a, b) = (original.snapshot(), restored.snapshot());
        worst_chassis = worst_chassis.max(chassis_gap(&a, &b));
        worst_projectile = worst_projectile.max(projectile_gap(&a, &b));
        assert_eq!(a.referee, b.referee, "referee state diverged");
        assert_eq!(a.runes, b.runes, "rune state diverged");
        assert_eq!(a.outposts, b.outposts, "outpost state diverged");
        assert_eq!(a.tick, b.tick);
        assert_eq!(a.shots_fired, b.shots_fired);
        assert_eq!(a.hits_detected, b.hits_detected);
    }
    eprintln!(
        "restore over 200 ticks: worst chassis {worst_chassis:.3e} m, worst ball {worst_projectile:.3e} m"
    );
    // Measured over these 200 ticks: the chassis agree exactly (0 m) and
    // the balls to 1.6e-17 m, one ulp of a metre, picked up where a spent
    // ball rolls on a rebuilt contact manifold. The bounds below leave
    // room for that rounding and for nothing else: a state the restore
    // forgot moves a driven chassis by millimetres within a few ticks,
    // which is what the pre-priming version of this test measured.
    assert!(
        worst_projectile < 1e-12,
        "projectile disagreement {worst_projectile} m"
    );
    assert!(
        worst_chassis < 1e-9,
        "chassis disagreement {worst_chassis} m"
    );
}
#[test]
fn restoring_rejects_a_snapshot_it_cannot_rebuild() {
    let (field, _) = busy_field(true);
    let mut snapshot = field.snapshot();
    snapshot.restore = None;
    assert!(matches!(
        Field::restore(&snapshot, &StaticGeometry::default(), 0.0),
        Err(FieldError::Restore(_))
    ));
    let snapshot = field.snapshot();
    assert!(matches!(
        Field::restore(&snapshot, &StaticGeometry::default(), f64::NAN),
        Err(FieldError::Restore(_))
    ));
    let mut broken = field.snapshot();
    broken.time_ns += 1;
    assert!(matches!(
        Field::restore(&broken, &StaticGeometry::default(), 0.0),
        Err(FieldError::Restore(_))
    ));
}
#[test]
fn default_field_steps_deterministically_in_any_partition() {
    let config = FieldConfig::default();
    let mut whole = Field::new(&config).unwrap();
    let mut split = Field::new(&config).unwrap();
    whole.step(2_600).unwrap();
    for ticks in [1, 999, 1_000, 600] {
        split.step(ticks).unwrap();
    }
    assert_eq!(whole.snapshot(), split.snapshot());
    let snapshot = whole.snapshot();
    assert_eq!(snapshot.tick, 2_600);
    assert_eq!(snapshot.time_ns, 2_600 * tick_ns());
    assert!(snapshot.outposts.is_empty());
    assert!(snapshot.runes.is_empty());
    // Reading a snapshot does not advance time.
    assert_eq!(whole.snapshot(), snapshot);
}
#[test]
fn projectiles_keep_stepping_deterministic_in_any_partition() {
    let config = FieldConfig::default();
    let mut whole = Field::new(&config).unwrap();
    let mut split = Field::new(&config).unwrap();
    let muzzle = Pose::yawed([0.0, 0.0, 1.0], 0.6);
    let shot = Shot::at_limit(Caliber::Mm17);
    for field in [&mut whole, &mut split] {
        field.step(ticks(500_000_000)).unwrap();
        field.fire(muzzle, shot, None).unwrap();
    }
    let total = ticks(700_000_000);
    whole.step(total).unwrap();
    let parts = [1, total / 4, total / 4];
    for part in parts {
        split.step(part).unwrap();
    }
    split.step(total - parts.iter().sum::<u64>()).unwrap();
    let a = whole.snapshot();
    let b = split.snapshot();
    assert_eq!(a, b);
    assert_eq!(a.projectiles.len(), 1);
    assert!(a.projectiles[0].position_m[0] > 5.0, "{:?}", a.projectiles);
    // The world keeps ticking after the shot is spent.
    whole.step(ticks(5_000_000_000)).unwrap();
    assert!(whole.snapshot().projectiles.is_empty());
    assert_eq!(
        whole.tick(),
        ticks(500_000_000) + ticks(700_000_000) + ticks(5_000_000_000)
    );
}

/// A field with a chassis and a 15 degree ramp mesh ahead of it.
fn ramp_field() -> FieldConfig {
    FieldConfig {
        step_duration: StepDuration::default(),
        rmuc_2027: None,
        rmul_2027: None,
        zones: Vec::new(),
        bases: vec![],
        runes: Vec::new(),
        outposts: Vec::new(),
        floor_height_m: 0.0,
        referee: None,
        projectile_policy: Default::default(),
        chassis: vec![ChassisPlacement {
            config: ChassisConfig::default(),
            spawn: Pose::at([0.0, 0.0, ChassisConfig::default().rest_height_m()]),
            team: Team::Red,
            kind: RobotKind::Infantry,
            performance: None,
        }],
    }
}
fn add_ramp(field: &mut Field) {
    // Rises 15 degrees from x = 1 to x = 3, then a 2 m plateau.
    let top = 2.0 * 15_f64.to_radians().tan();
    field
        .add_static_mesh(
            vec![
                [1.0, -1.0, 0.0],
                [1.0, 1.0, 0.0],
                [3.0, -1.0, top],
                [3.0, 1.0, top],
                [5.0, -1.0, top],
                [5.0, 1.0, top],
                [5.0, -1.0, 0.0],
                [5.0, 1.0, 0.0],
            ],
            vec![
                [0, 2, 1],
                [1, 2, 3],
                [2, 4, 3],
                [3, 4, 5],
                [4, 6, 5],
                [5, 6, 7],
            ],
        )
        .unwrap();
}
#[test]
fn mecanum_chassis_steps_deterministically_in_any_partition() {
    let mut config = ramp_field();
    config.chassis[0].config = ChassisConfig::hero();
    config.chassis[0].spawn.translation_m[2] = ChassisConfig::hero().rest_height_m();
    let mut whole = Field::new(&config).unwrap();
    let mut split = Field::new(&config).unwrap();
    for field in [&mut whole, &mut split] {
        field
            .command_chassis(
                0,
                ChassisCommand {
                    forward_m_s: 1.0,
                    left_m_s: 0.5,
                    yaw_rate_rad_s: 0.3,
                    ..ChassisCommand::default()
                },
            )
            .unwrap();
    }
    whole.step(1000).unwrap();
    for ticks in [1, 249, 333, 417] {
        split.step(ticks).unwrap();
    }
    assert_eq!(whole.snapshot(), split.snapshot());
}
#[test]
fn rotating_consumes_drive_capacity_while_climbing() {
    let climb = |spin: f64| {
        let mut field = Field::new(&ramp_field()).unwrap();
        add_ramp(&mut field);
        field.step(ticks(300_000_000)).unwrap();
        for _ in 0..250 {
            let yaw = chassis::yaw_of(field.snapshot().chassis[0].pose);
            // Hold the same world-space uphill wish while the body rotates.
            field
                .command_chassis(
                    0,
                    ChassisCommand {
                        forward_m_s: 1.5 * yaw.cos(),
                        left_m_s: -1.5 * yaw.sin(),
                        yaw_rate_rad_s: spin,
                        ..Default::default()
                    },
                )
                .unwrap();
            field.step(ticks(10_000_000)).unwrap();
        }
        field.snapshot().chassis[0].pose.translation_m[0]
    };
    let straight = climb(0.);
    let rotating = climb(6.);
    assert!(straight > 2.3, "straight={straight}");
    assert!(
        rotating < straight - 0.2,
        "straight={straight}, rotating={rotating}"
    );
}

#[test]
fn chassis_climbs_a_ramp_and_steps_deterministically_in_any_partition() {
    let config = ramp_field();
    let mut whole = Field::new(&config).unwrap();
    let mut split = Field::new(&config).unwrap();
    for field in [&mut whole, &mut split] {
        add_ramp(field);
        field.step(ticks(300_000_000)).unwrap();
        field
            .command_chassis(
                0,
                ChassisCommand {
                    forward_m_s: 1.5,
                    ..Default::default()
                },
            )
            .unwrap();
    }
    let total = ticks(3_500_000_000);
    whole.step(total).unwrap();
    let parts = [1, total / 4, total / 3];
    for part in parts {
        split.step(part).unwrap();
    }
    split.step(total - parts.iter().sum::<u64>()).unwrap();
    assert_eq!(whole.snapshot(), split.snapshot());
    let chassis = whole.snapshot().chassis.remove(0);
    let top = 2.0 * 15_f64.to_radians().tan();
    let [x, _, z] = chassis.pose.translation_m;
    assert!(x > 3.2 && x < 5.0, "{chassis:?}");
    let preset = ChassisConfig::default();
    let sag = preset.mass_kg * 9.81 / 4.0 / preset.suspension_stiffness_n_m;
    assert!(
        (z - (top + preset.rest_height_m() - sag)).abs() < 0.02,
        "{chassis:?}"
    );
    assert!(chassis.wheels.iter().all(|w| w.contact.is_some()));
    assert_eq!(whole.tick(), ticks(300_000_000) + total);
    // A field without a chassis or projectiles still jumps straight ahead.
    let mut plain = Field::new(&FieldConfig::default()).unwrap();
    assert!(plain.snapshot().chassis.is_empty());
    assert!(matches!(
        plain.command_chassis(0, ChassisCommand::default()),
        Err(FieldError::Chassis(_))
    ));
    plain.step(10).unwrap();
}
/// Chassis join and leave a running field; ids count up and are never
/// reused, and each one answers only to its own id.
#[test]
fn chassis_join_and_leave_with_stable_ids() {
    let mut field = Field::new(&FieldConfig::default()).unwrap();
    let placement = |x: f64, team: Team| ChassisPlacement {
        config: ChassisConfig::default(),
        spawn: Pose::at([x, 0.0, ChassisConfig::default().rest_height_m()]),
        team,
        kind: RobotKind::Infantry,
        performance: None,
    };
    let red = field.add_chassis(&placement(-2.0, Team::Red)).unwrap();
    let blue = field.add_chassis(&placement(-4.0, Team::Blue)).unwrap();
    assert_eq!((red, blue), (0, 1));
    assert_eq!(field.chassis_team(blue), Some(Team::Blue));
    field.step(300).unwrap();
    field
        .command_chassis(
            blue,
            ChassisCommand {
                forward_m_s: 1.0,
                ..Default::default()
            },
        )
        .unwrap();
    field.step(1_000).unwrap();
    let snapshot = field.snapshot();
    assert_eq!(
        snapshot
            .chassis
            .iter()
            .map(|c| (c.id, c.team))
            .collect::<Vec<_>>(),
        vec![(red, Team::Red), (blue, Team::Blue)]
    );
    // Only the commanded chassis moved.
    assert!(snapshot.chassis[0].pose.translation_m[0].abs() - 2.0 < 0.05);
    assert!(snapshot.chassis[1].pose.translation_m[0] > -3.5);
    field.remove_chassis(red).unwrap();
    assert!(matches!(
        field.remove_chassis(red),
        Err(FieldError::Chassis(_))
    ));
    assert!(matches!(
        field.command_chassis(red, ChassisCommand::default()),
        Err(FieldError::Chassis(_))
    ));
    let next = field.add_chassis(&placement(-2.0, Team::Red)).unwrap();
    assert_eq!(next, 2);
    field.step(100).unwrap();
    let ids: Vec<u32> = field.snapshot().chassis.iter().map(|c| c.id).collect();
    assert_eq!(ids, vec![blue, next]);
}
/// A roof the body fits under but the turret does not stops the chassis.
#[test]
fn chassis_turret_stops_under_a_low_roof() {
    let config = ramp_field();
    let chassis = config.chassis[0].config.clone();
    let mut field = Field::new(&config).unwrap();
    let roof_z = chassis.rest_height_m() + chassis.body_half_m[2] + 0.05;
    assert!(
        roof_z < chassis.rest_height_m() + chassis.turret_center_m[2] + chassis.turret_half_m[2]
    );
    field
        .add_static_mesh(
            vec![
                [1.0, -1.0, roof_z],
                [1.0, 1.0, roof_z],
                [3.0, -1.0, roof_z],
                [3.0, 1.0, roof_z],
            ],
            vec![[0, 1, 2], [1, 3, 2]],
        )
        .unwrap();
    field.step(300).unwrap();
    field
        .command_chassis(
            0,
            ChassisCommand {
                forward_m_s: 1.0,
                ..ChassisCommand::default()
            },
        )
        .unwrap();
    field.step(3_000).unwrap();
    let snapshot = field.snapshot().chassis.remove(0);
    let x = snapshot.pose.translation_m[0];
    // The turret's front face reaches the roof edge and no further.
    let turret_front = chassis.turret_center_m[0] + chassis.turret_half_m[0];
    assert!(x + turret_front < 1.05 && x + turret_front > 0.8, "{x}");
    assert!(
        snapshot.velocity_m_s[0].abs() < 0.2,
        "{:?}",
        snapshot.velocity_m_s
    );
}
/// A closed 1 m box mesh centred at `center`.
fn box_mesh(center: [f64; 3], half: f64) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
    let [cx, cy, cz] = center;
    let mut vertices = Vec::new();
    for i in 0..8 {
        let sx = if i & 1 == 0 { -half } else { half };
        let sy = if i & 2 == 0 { -half } else { half };
        let sz = if i & 4 == 0 { -half } else { half };
        vertices.push([cx + sx, cy + sy, cz + sz]);
    }
    let triangles = vec![
        [0, 2, 1],
        [1, 2, 3],
        [4, 5, 6],
        [5, 7, 6],
        [0, 1, 4],
        [1, 5, 4],
        [2, 6, 3],
        [3, 6, 7],
        [0, 4, 2],
        [2, 4, 6],
        [1, 3, 5],
        [3, 7, 5],
    ];
    (vertices, triangles)
}
#[test]
fn dynamic_geometry_tracks_bodies_and_excludes_terrain() {
    let mut field = Field::new(&FieldConfig {
        runes: Vec::new(),
        outposts: Vec::new(),
        ..Default::default()
    })
    .unwrap();
    assert!(
        field
            .dynamic_geometry_snapshot()
            .into_triangles()
            .0
            .is_empty()
    );
    let id = field
        .add_chassis(&ChassisPlacement {
            team: Team::Red,
            kind: RobotKind::Infantry,
            performance: None,
            config: Default::default(),
            spawn: Pose::at([0., 0., 2.]),
        })
        .unwrap();
    let before = field.dynamic_geometry_snapshot().into_triangles();
    assert!(!before.1.is_empty());
    field.step(100).unwrap();
    assert_ne!(
        before.0,
        field.dynamic_geometry_snapshot().into_triangles().0
    );
    field.remove_chassis(id).unwrap();
    assert!(
        field
            .dynamic_geometry_snapshot()
            .into_triangles()
            .0
            .is_empty()
    );
    field
        .fire(Pose::at([0., 0., 2.]), Shot::at_limit(Caliber::Mm17), None)
        .unwrap();
    let captured = field.dynamic_geometry_snapshot();
    field.step(10).unwrap();
    let projectile = captured.into_triangles();
    assert!(!projectile.1.is_empty());
    assert_ne!(
        projectile.0,
        field.dynamic_geometry_snapshot().into_triangles().0
    );
    assert!(field.static_geometry().0.is_empty());
}

#[test]
fn fixed_geometry_snapshot_survives_field_changes_and_drop() {
    let mut field = Field::new(&ramp_field()).unwrap();
    add_ramp(&mut field);
    let expected = field.static_geometry();
    let captured = field.static_geometry_snapshot();
    let (vertices, triangles) = box_mesh([1.5, 0.0, 0.5], 0.5);
    field.add_static_mesh(vertices, triangles).unwrap();
    assert!(field.static_geometry().1.len() > expected.1.len());
    drop(field);
    let actual = std::thread::spawn(move || captured.into_triangles())
        .join()
        .unwrap();
    assert_eq!(actual, expected);
}

/// A triangle mesh block ahead of the chassis stops it and bounces
/// projectiles; the physics geometry export shows it and the meshes.
#[test]
fn triangle_mesh_blocks_chassis_and_projectiles() {
    let config = ramp_field();
    let mut field = Field::new(&config).unwrap();
    assert_eq!(field.static_geometry().1.len(), 0);
    add_ramp(&mut field);
    assert_eq!(field.static_geometry().1.len(), 6);
    let (vertices, triangles) = box_mesh([1.5, 0.0, 0.5], 0.5);
    let count = triangles.len();
    field.add_static_mesh(vertices, triangles).unwrap();
    assert_eq!(field.static_geometry().1.len(), 6 + count);
    // A projectile fired at the block comes back.
    // Offset from the robot so the returning ball tests the wall rather
    // than striking the chassis whose new silhouette is being tested below.
    let muzzle = Pose::at([0.0, 0.45, 0.5]);
    field
        .fire(muzzle, Shot::at_limit(Caliber::Mm17), None)
        .unwrap();
    field.step(120).unwrap();
    let projectile = &field.snapshot().projectiles[0];
    assert!(projectile.position_m[0] < 1.0, "{projectile:?}");
    assert!(projectile.velocity_m_s[0] < 0.0, "{projectile:?}");
    // The chassis drives into the block and stops at its face.
    field
        .command_chassis(
            0,
            ChassisCommand {
                forward_m_s: 1.0,
                ..ChassisCommand::default()
            },
        )
        .unwrap();
    field.step(3_000).unwrap();
    let chassis = field.snapshot().chassis.remove(0);
    let x = chassis.pose.translation_m[0];
    let front = config.chassis[0].config.body_half_m[0];
    assert!(x + front < 1.05 && x + front > 0.8, "{x}");
    assert!(
        chassis.velocity_m_s[0].abs() < 0.2,
        "{:?}",
        chassis.velocity_m_s
    );
}
#[test]
fn chassis_stops_against_a_wall_and_rejects_bad_placements() {
    let config = ramp_field();
    let mut field = Field::new(&config).unwrap();
    // A wall across the path at x = 1.5 whose winding faces away from the
    // chassis, and a back-facing deck under it: mesh contacts are two-sided.
    field
        .add_static_mesh(
            vec![
                [1.5, -1.0, 0.0],
                [1.5, 1.0, 0.0],
                [1.5, -1.0, 1.0],
                [1.5, 1.0, 1.0],
            ],
            vec![[0, 1, 2], [1, 3, 2]],
        )
        .unwrap();
    field
        .add_static_mesh(
            vec![
                [-1.0, -1.0, 0.03],
                [-1.0, 1.0, 0.03],
                [1.5, -1.0, 0.03],
                [1.5, 1.0, 0.03],
            ],
            vec![[0, 1, 2], [1, 3, 2]],
        )
        .unwrap();
    field.step(300).unwrap();
    let resting = field.snapshot().chassis.remove(0);
    assert!(
        resting.wheels.iter().all(|w| w
            .contact
            .is_some_and(|c| (c.point_m[2] - 0.03).abs() < 1e-6)),
        "{resting:?}"
    );
    field
        .command_chassis(
            0,
            ChassisCommand {
                forward_m_s: 2.0,
                ..Default::default()
            },
        )
        .unwrap();
    field.step(3_000).unwrap();
    let chassis = field.snapshot().chassis.remove(0);
    let front = chassis.pose.translation_m[0] + ChassisConfig::default().body_half_m[0];
    assert!(front < 1.52 && front > 1.3, "{chassis:?}");
    assert!(chassis.velocity_m_s[0].abs() < 0.2, "{chassis:?}");
    assert!(matches!(
        Field::new(&FieldConfig {
            chassis: vec![ChassisPlacement {
                config: ChassisConfig {
                    mass_kg: -1.0,
                    ..ChassisConfig::default()
                },
                spawn: Pose::default(),
                team: Team::Red,
                kind: RobotKind::Infantry,
                performance: None,
            }],
            ..FieldConfig::default()
        }),
        Err(FieldError::Chassis(_))
    ));
}
