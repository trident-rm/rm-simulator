// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Runtime compatibility, ordered retained output and session composition.
use rm_sim_physics::{
    ArmorTarget, Caliber, Pose, Shot, StepDuration, TargetFace, TargetFrames, Team,
    chassis::{ChassisCommand, ChassisConfig},
    projectile::Contact,
    runtime::WorldPhysics,
    session::{RobotScenario, RobotSession},
};

fn face() -> TargetFace {
    TargetFace {
        target: ArmorTarget::Outpost {
            outpost: 0,
            face: 0,
        },
        pose: Pose::yawed([1.5, 0., 1.], std::f64::consts::PI),
    }
}

fn fixture(duration: StepDuration) -> WorldPhysics {
    let mut world = WorldPhysics::with_step_duration(&[face()], 0., duration);
    let robot = world
        .add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 1., 0.3]))
        .unwrap();
    world
        .command_chassis(
            robot,
            ChassisCommand {
                forward_m_s: 0.5,
                ..Default::default()
            },
        )
        .unwrap();
    world.set_carrier_trace(true);
    world
        .fire(
            0,
            Pose::at([0., 0., 1.025]),
            Shot::at_limit(Caliber::Mm17),
            None,
        )
        .unwrap();
    world
}

#[test]
fn retained_output_matches_owned_contacts_truth_and_traces_at_both_rates() {
    for duration in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let mut owned = fixture(duration);
        let mut retained = fixture(duration);
        let mut expected = Vec::new();
        let mut contacts = Vec::with_capacity(64);
        let storage = (contacts.as_ptr(), contacts.capacity());
        let mut frames = TargetFrames::new(vec![face()]);
        for tick in 0..256 {
            let previous = tick as f64 * 0.0001;
            frames
                .begin(|faces| faces[0].pose.translation_m[1] = previous)
                .unwrap();
            frames
                .update(|faces| faces[0].pose.translation_m[1] = previous + 0.0001)
                .unwrap();
            let time_ns = tick * duration.as_ns();
            expected.extend(owned.step(time_ns, &frames).unwrap());
            retained.step_into(time_ns, &frames, &mut contacts).unwrap();
            assert_eq!(contacts, expected);
            assert_eq!(retained.snapshot(), owned.snapshot());
            assert_eq!(retained.chassis_snapshots(), owned.chassis_snapshots());
            assert_eq!(retained.carrier_trace(), owned.carrier_trace());
            assert_eq!(retained.armor_collisions(), owned.armor_collisions());
            assert_eq!(
                retained.removals().collect::<Vec<_>>(),
                owned.removals().collect::<Vec<_>>()
            );
        }
        assert!(!contacts.is_empty());
        assert_eq!((contacts.as_ptr(), contacts.capacity()), storage);
    }
}

#[test]
fn errors_preserve_existing_output_and_early_admission_preserves_state() {
    let mut world = fixture(StepDuration::HZ_128);
    let before = world.snapshot();
    let sentinel = Contact {
        projectile: u64::MAX,
        shooter: None,
        caliber: Caliber::Mm17,
        target: face().target,
        position_m: [0.; 3],
        local_m: [0.; 3],
        normal_speed_m_s: 0.,
    };
    let mut contacts = vec![sentinel.clone()];
    assert_eq!(
        world.step_into(u64::MAX, &TargetFrames::new(vec![face()]), &mut contacts),
        Err("physics time overflow")
    );
    assert_eq!(
        world.step_into(0, &TargetFrames::new(vec![]), &mut contacts),
        Err("target face count changed")
    );
    assert_eq!(contacts, [sentinel]);
    assert_eq!(world.snapshot(), before);
    let mut idle = WorldPhysics::new(&[], 0.);
    idle.step_into(0, &TargetFrames::new(vec![]), &mut contacts)
        .unwrap();
    assert_eq!(contacts.len(), 1);
}

#[test]
fn session_target_endpoints_share_runtime_output_and_reset_contract() {
    fn session() -> RobotSession {
        RobotSession::new(
            RobotScenario::new(vec![face()], 0., StepDuration::HZ_1000, |world| {
                world.fire(
                    0,
                    Pose::at([0., 0., 1.025]),
                    Shot::at_limit(Caliber::Mm17),
                    None,
                )?;
                Ok(())
            })
            .unwrap(),
        )
        .unwrap()
    }
    let mut owned = session();
    let mut retained = session();
    let mut expected = Vec::new();
    let mut contacts = Vec::with_capacity(32);
    let pointer = contacts.as_ptr();
    for _ in 0..128 {
        expected.extend(owned.step_with_targets(&[face()]).unwrap());
        retained
            .step_with_targets_into(&[face()], &mut contacts)
            .unwrap();
        assert_eq!(retained.stamp(), owned.stamp());
    }
    assert!(!contacts.is_empty());
    assert_eq!(contacts, expected);
    assert_eq!(contacts.as_ptr(), pointer);
    let stamp = retained.stamp();
    assert!(retained.step_with_targets_into(&[], &mut contacts).is_err());
    assert_eq!(retained.stamp(), stamp);
    assert_eq!(contacts, expected);
    retained.reset().unwrap();
    assert_eq!(retained.stamp().time_ns, 0);
    assert_eq!(
        retained.ground_truth().snapshot(),
        session().ground_truth().snapshot()
    );
}

#[test]
fn legacy_exports_name_the_runtime_type() {
    let root: rm_sim_physics::WorldPhysics = WorldPhysics::new(&[], 0.);
    let legacy: rm_sim_physics::projectile::WorldPhysics = root;
    assert_eq!(legacy.step_duration(), StepDuration::HZ_128);
}
