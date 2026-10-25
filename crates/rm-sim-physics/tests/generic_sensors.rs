// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Passive generic carrier acquisition over the actual shared physical scene.
#[allow(dead_code)]
#[path = "../examples/jointed_turret.rs"]
mod fixture;
use rm_sim_physics::{
    StepDuration, TargetFrames, Team, WorldPhysics,
    chassis::ChassisConfig,
    cooking::CookedAssembly,
    runtime::{ParticipantBinding, PhysicalRuntime},
    sensors::{
        carrier::{CarrierIdentity, StampedCarrierPose},
        imu::{CarrierImu, CarrierImuConfig},
        mid360::Mid360Pattern,
        rolling::{MountedLidar, RangeEffects, RollingFrame, SceneCapture},
        synchronized::{CarrierConfig, CarrierSynchronized},
    },
};
use sim_engine::{
    Pose,
    model::{CarrierKey, JointMotor},
};

fn capture(
    runtime: &PhysicalRuntime,
    participant: ParticipantBinding,
    time_ns: u64,
) -> SceneCapture {
    let carrier = runtime
        .observe_carrier(participant, CarrierKey(71), 0, time_ns, 0.001)
        .unwrap();
    SceneCapture::from_physical_geometry(&runtime.geometry_capture(), carrier).unwrap()
}
fn lidar(mount: Pose) -> MountedLidar {
    MountedLidar::new(
        Mid360Pattern::new(0.1, 20.).unwrap(),
        mount,
        0,
        RangeEffects {
            seed: 17,
            standard_deviation_m: 0.01,
            bias_m: 0.002,
            dropout_probability: 0.05,
        },
    )
    .unwrap()
    .with_angular_noise(0.001)
    .unwrap()
}
fn scan(sensor: &MountedLidar, before: &SceneCapture, after: &SceneCapture) -> RollingFrame {
    let mut out = RollingFrame::default();
    if let Some(window) = sensor
        .window_between(before.time_ns(), after.time_ns(), 3)
        .unwrap()
    {
        sensor.sample_into(before, after, window, &mut out).unwrap();
    }
    out
}
#[test]
fn generic_capture_matches_chassis_compatibility_exactly_at_both_step_rates() {
    for duration in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let mut world = WorldPhysics::with_step_duration(&[], 0., duration);
        let robot = world
            .add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 0., 0.4]))
            .unwrap();
        world.add_static_box(Pose::at([2., 0., 0.6]), [0.2, 1., 0.6]);
        let participant = world.chassis_participant(robot).unwrap();
        let sensor = lidar(Pose::at([0., 0., 0.4]));
        for tick in 0..20 {
            let time = tick * duration.as_ns();
            let a = world.lidar_scene_capture(robot, time).unwrap();
            let generic_a = capture(world.physical_runtime(), participant, time);
            world.step(time, &TargetFrames::new(vec![])).unwrap();
            let b = world
                .lidar_scene_capture(robot, time + duration.as_ns())
                .unwrap();
            let generic_b = capture(
                world.physical_runtime(),
                participant,
                time + duration.as_ns(),
            );
            assert_eq!(scan(&sensor, &a, &b), scan(&sensor, &generic_a, &generic_b));
        }
    }
}
#[test]
fn actual_articulated_body_motion_mounts_and_self_occlusion_need_no_robot_ids() {
    let mut runtime = PhysicalRuntime::new(StepDuration::HZ_1000);
    runtime
        .install_catch_floor(
            0.,
            sim_engine::model::ContactMaterial {
                friction: 0.4,
                restitution: 0.,
                memberships: u32::MAX,
                filter: u32::MAX,
            },
        )
        .unwrap();
    let prepared = CookedAssembly::new(fixture::description(false).unwrap()).unwrap();
    let live = runtime
        .install_assembly(&prepared, Pose::default(), &fixture::keys())
        .unwrap();
    runtime
        .set_joint_motor(
            live.joint(fixture::YAW_JOINT).unwrap(),
            Some(JointMotor::new(0.6, 0., 200., 10., 20.).unwrap()),
        )
        .unwrap();
    runtime
        .set_joint_motor(
            live.joint(fixture::PITCH_JOINT).unwrap(),
            Some(JointMotor::new(-0.25, 0., 200., 10., 20.).unwrap()),
        )
        .unwrap();
    let participant = runtime
        .body_participant(live.body(fixture::BARREL_LINK).unwrap())
        .unwrap();
    let mut before = capture(&runtime, participant, 0);
    let initial = runtime.participant_state(participant).unwrap().pose;
    let inside = lidar(Pose::default());
    let outside = lidar(Pose {
        translation_m: [0., 0., 0.3],
        rotation_wxyz: [0., 1., 0., 0.],
    });
    let mut imu = CarrierImu::new(CarrierImuConfig {
        carrier_from_imu: Pose::at([0.1, 0., 0.1]),
        period_ns: 5_000_000,
        epoch_ns: 0,
        gravity_m_s2: Some([0., 0., -9.81]),
        effects: Default::default(),
    })
    .unwrap();
    let mut inertial = Vec::new();
    let mut visible = 0;
    for tick in 0..200 {
        runtime.step(tick * 1_000_000).unwrap();
        let time = (tick + 1) * 1_000_000;
        let carrier = runtime
            .observe_carrier(participant, CarrierKey(71), 0, time, 0.001)
            .unwrap();
        imu.ingest(&[carrier], &mut inertial).unwrap();
        let after = capture(&runtime, participant, time);
        assert!(
            scan(&inside, &before, &after)
                .points
                .iter()
                .all(|p| p.range_m.is_none())
        );
        visible += scan(&outside, &before, &after)
            .points
            .iter()
            .filter(|p| p.range_m.is_some())
            .count();
        before = after;
    }
    assert_ne!(
        initial,
        runtime.participant_state(participant).unwrap().pose
    );
    assert!(inertial.len() > 30);
    assert!(
        inertial
            .iter()
            .any(|s| s.angular_velocity_rad_s.iter().any(|v| v.abs() > 0.1))
    );
    assert!(visible > 0);
}
#[test]
fn all_stamp_fields_and_actual_runtime_brand_guard_rolling_brackets() {
    let runtime = PhysicalRuntime::new(StepDuration::HZ_1000);
    let geometry = runtime.geometry_capture();
    let identity = CarrierIdentity {
        carrier: CarrierKey(9),
        world_brand: geometry.runtime_identity(),
        installation_generation: 3,
        discontinuity_revision: 0,
    };
    let stamp = |id, time| StampedCarrierPose::new(id, time, 0.001, Pose::default()).unwrap();
    let a = SceneCapture::from_physical_geometry(&geometry, stamp(identity, 0)).unwrap();
    let sensor = lidar(Pose::default());
    for field in 0..4 {
        let mut changed = identity;
        match field {
            0 => changed.carrier = CarrierKey(10),
            1 => changed.world_brand += 1,
            2 => changed.installation_generation += 1,
            _ => changed.discontinuity_revision += 1,
        }
        let b = SceneCapture::from_physical_geometry(&geometry, stamp(changed, 1_000_000));
        if field == 1 {
            assert!(b.is_err());
            continue;
        }
        let mut out = RollingFrame::default();
        let window = sensor.window_between(0, 1_000_000, 1).unwrap().unwrap();
        assert!(
            sensor
                .sample_into(&a, &b.unwrap(), window, &mut out)
                .is_err()
        );
        assert!(out.points.is_empty());
    }
    let reset = PhysicalRuntime::new(StepDuration::HZ_1000);
    let mut reset_identity = identity;
    reset_identity.world_brand = reset.geometry_capture().runtime_identity();
    let b = SceneCapture::from_physical_geometry(
        &reset.geometry_capture(),
        stamp(reset_identity, 1_000_000),
    )
    .unwrap();
    let mut out = RollingFrame::default();
    assert!(
        sensor
            .sample_into(
                &a,
                &b,
                sensor.window_between(0, 1_000_000, 1).unwrap().unwrap(),
                &mut out
            )
            .is_err()
    );
}
#[test]
fn generic_packets_keep_phase_across_identity_swap_reset_and_missing_bracket() {
    let runtime = PhysicalRuntime::new(StepDuration::HZ_1000);
    let geometry = runtime.geometry_capture();
    let mut identity = CarrierIdentity {
        carrier: CarrierKey(9),
        world_brand: geometry.runtime_identity(),
        installation_generation: 3,
        discontinuity_revision: 0,
    };
    let mut sensor = CarrierSynchronized::new(CarrierConfig {
        lidar_mount: Pose::default(),
        imu: CarrierImuConfig {
            carrier_from_imu: Pose::default(),
            period_ns: 1_000_000,
            epoch_ns: 0,
            gravity_m_s2: Some([0., 0., -9.81]),
            effects: Default::default(),
        },
        lidar_period_ns: 10_000_000,
        stride: 11,
        min_range_m: 0.1,
        max_range_m: 20.,
        range_effects: Default::default(),
        angular_sigma_rad: 0.,
    })
    .unwrap();
    let mut out = Vec::new();
    for tick in 0..110 {
        if tick == 37 {
            identity.installation_generation += 1;
        }
        if tick == 67 {
            identity.discontinuity_revision += 1;
        }
        if tick == 87 {
            continue;
        }
        let stamp = |time| StampedCarrierPose::new(identity, time, 0.001, Pose::default()).unwrap();
        let before =
            SceneCapture::from_physical_geometry(&geometry, stamp(tick * 1_000_000)).unwrap();
        let after =
            SceneCapture::from_physical_geometry(&geometry, stamp((tick + 1) * 1_000_000)).unwrap();
        sensor
            .ingest(&before, &after, &[stamp((tick + 1) * 1_000_000)], &mut out)
            .unwrap();
    }
    assert!(!out.is_empty());
    assert!(!out.iter().any(|f| [3, 6, 8].contains(&f.sequence)));
    assert!(out.iter().any(|f| f.sequence > 8 && f.discontinuities > 0));
    for frame in out {
        assert_eq!(frame.start_ns, frame.sequence * 10_000_000);
        assert_eq!(frame.imu.len(), 10);
    }
}

#[test]
fn generic_geometry_topology_changes_reject_the_bracket_without_modifying_output() {
    let mut runtime = PhysicalRuntime::new(StepDuration::HZ_1000);
    let geometry = runtime.geometry_capture();
    let identity = CarrierIdentity {
        carrier: CarrierKey(1),
        world_brand: geometry.runtime_identity(),
        installation_generation: 0,
        discontinuity_revision: 0,
    };
    let stamp =
        |time| StampedCarrierPose::new(identity, time, 0.001, Pose::at([0., 0., 1.])).unwrap();
    let before = SceneCapture::from_physical_geometry(&geometry, stamp(0)).unwrap();
    let floor = runtime
        .install_catch_floor(
            0.,
            sim_engine::model::ContactMaterial {
                friction: 0.4,
                restitution: 0.,
                memberships: u32::MAX,
                filter: u32::MAX,
            },
        )
        .unwrap();
    let after = SceneCapture::from_physical_geometry(&runtime.geometry_capture(), stamp(1_000_000))
        .unwrap();
    let sensor = lidar(Pose::default());
    let window = sensor.window_between(0, 1_000_000, 1).unwrap().unwrap();
    let mut output = RollingFrame::default();
    assert!(
        sensor
            .sample_into(&before, &after, window, &mut output)
            .is_err()
    );
    assert!(output.points.is_empty());
    runtime.remove_fixed_collider(floor).unwrap();
    // A fresh capture recovers; a previously rejected bracket mutates no capture.
    let end = SceneCapture::from_physical_geometry(&runtime.geometry_capture(), stamp(1_000_000))
        .unwrap();
    sensor
        .sample_into(&before, &end, window, &mut output)
        .unwrap();
    assert_eq!(output.points.len(), 200);
}

#[test]
fn other_participant_teleport_reset_or_revival_rejects_bracket_with_unchanged_carrier() {
    for duration in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        for discontinuity in 0..3 {
            let mut world = WorldPhysics::with_step_duration(&[], 0., duration);
            let sensor_robot = world
                .add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 0., 0.4]))
                .unwrap();
            let obstacle = world
                .add_chassis(
                    Team::Blue,
                    ChassisConfig::default(),
                    Pose::at([2., 0., 0.4]),
                )
                .unwrap();
            if discontinuity == 2 {
                world.set_chassis_defeated(obstacle, true);
            }
            let participant = world.chassis_participant(sensor_robot).unwrap();
            let geometry_before = world.physical_runtime().geometry_capture();
            let before = capture(world.physical_runtime(), participant, 0);
            let sensor_before = world.chassis_snapshot(sensor_robot).unwrap();
            if discontinuity == 1 {
                let mut replacement = world.chassis_snapshot(obstacle).unwrap();
                replacement.pose = Pose::at([-2., 0., 0.4]);
                world.reset_chassis(&replacement).unwrap();
            } else if discontinuity == 2 {
                world.set_chassis_defeated(obstacle, false);
            } else {
                world
                    .place_chassis(obstacle, Pose::at([-2., 0., 0.4]))
                    .unwrap();
            }
            let geometry_after = world.physical_runtime().geometry_capture();
            let after = capture(world.physical_runtime(), participant, duration.as_ns());
            assert_eq!(sensor_before, world.chassis_snapshot(sensor_robot).unwrap());
            assert_eq!(before.carrier_identity(), after.carrier_identity());
            assert_eq!(
                geometry_before.runtime_identity(),
                geometry_after.runtime_identity()
            );
            assert_eq!(
                geometry_before
                    .colliders()
                    .map(|(id, _)| id)
                    .collect::<Vec<_>>(),
                geometry_after
                    .colliders()
                    .map(|(id, _)| id)
                    .collect::<Vec<_>>()
            );
            assert_ne!(
                geometry_before.placement_epoch(),
                geometry_after.placement_epoch()
            );
            let sensor = lidar(Pose::at([0., 0., 0.4]));
            let window = sensor
                .window_between(0, duration.as_ns(), 1)
                .unwrap()
                .unwrap();
            let mut output = RollingFrame {
                points: vec![rm_sim_physics::sensors::rolling::TimedReturn {
                    sample_index: 19,
                    time_ns: 23,
                    range_m: None,
                    point_sensor_m: None,
                }],
            };
            let previous = output.clone();
            assert!(
                sensor
                    .sample_into(&before, &after, window, &mut output)
                    .is_err()
            );
            assert_eq!(output, previous);
            // Both fresh endpoints after the teleport share its new epoch.
            let fresh = capture(world.physical_runtime(), participant, 0);
            sensor
                .sample_into(&fresh, &after, window, &mut output)
                .unwrap();
            assert!(!output.points.is_empty());
        }
    }
}

#[test]
fn ordinary_motion_of_another_participant_preserves_generic_bracket_continuity() {
    for duration in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let mut world = WorldPhysics::with_step_duration(&[], 0., duration);
        let sensor_robot = world
            .add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 0., 0.4]))
            .unwrap();
        let obstacle = world
            .add_chassis(
                Team::Blue,
                ChassisConfig::default(),
                Pose::at([2., 0., 0.8]),
            )
            .unwrap();
        world
            .command_chassis(
                obstacle,
                rm_sim_physics::chassis::ChassisCommand {
                    forward_m_s: 0.7,
                    ..Default::default()
                },
            )
            .unwrap();
        let participant = world.chassis_participant(sensor_robot).unwrap();
        let before = capture(world.physical_runtime(), participant, 0);
        let initial_epoch = world
            .physical_runtime()
            .geometry_capture()
            .placement_epoch();
        let obstacle_before = world.chassis_snapshot(obstacle).unwrap().pose;
        world.step(0, &TargetFrames::new(vec![])).unwrap();
        let after = capture(world.physical_runtime(), participant, duration.as_ns());
        assert_eq!(before.carrier_identity(), after.carrier_identity());
        assert_eq!(
            initial_epoch,
            world
                .physical_runtime()
                .geometry_capture()
                .placement_epoch()
        );
        assert_ne!(
            obstacle_before,
            world.chassis_snapshot(obstacle).unwrap().pose
        );
        assert!(
            !scan(&lidar(Pose::at([0., 0., 0.4])), &before, &after)
                .points
                .is_empty()
        );
    }
}
