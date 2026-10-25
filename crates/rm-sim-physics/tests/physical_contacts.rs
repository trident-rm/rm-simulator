// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Public-API participant identity, bounded observation and scoring parity tests.
use rm_sim_physics::{
    ArmorTarget, Caliber, Pose, Shot, StepDuration, TargetFace, TargetFrames,
    cooking::CookedRigidBody,
    runtime::{BodyBinding, BodyKey, WorldPhysics},
    session::{RobotScenario, RobotSession},
};
use sim_engine::model::{
    BodyKind, BodyState, ColliderDescription, ContactMaterial, Inertial, RigidBodyDescription,
    ShapeDescription,
};

fn cube() -> CookedRigidBody {
    CookedRigidBody::new(
        RigidBodyDescription::new(
            BodyKind::Dynamic(Inertial::new(1., [0.01; 3], Pose::default()).unwrap()),
            vec![
                ColliderDescription::new(
                    ShapeDescription::Cuboid {
                        half_extents_m: [0.1; 3],
                    },
                    Pose::default(),
                    ContactMaterial {
                        friction: 0.4,
                        restitution: 0.,
                        memberships: u32::MAX,
                        filter: u32::MAX,
                    },
                )
                .unwrap(),
            ],
        )
        .unwrap(),
    )
    .unwrap()
}

fn install(world: &mut WorldPhysics, key: u64, position: [f64; 3]) -> BodyBinding {
    world
        .install_body(BodyKey(key), &cube(), BodyState::at(Pose::at(position)))
        .unwrap()
}

fn step(world: &mut WorldPhysics, tick: u64) {
    world
        .step(
            tick * world.step_duration().as_ns(),
            &TargetFrames::new(vec![]),
        )
        .unwrap();
}

#[test]
fn generic_floor_contacts_report_parentless_geometry_and_explicit_slice_times() {
    for (duration, slices) in [(StepDuration::HZ_128, 8), (StepDuration::HZ_1000, 1)] {
        let mut world = WorldPhysics::with_step_duration(&[], 0., duration);
        let body = install(&mut world, 1, [0., 0., 0.099]);
        let participant = world.body_participant(body).unwrap();
        world.enable_contact_observations(100).unwrap();
        step(&mut world, 7);
        assert!(!world.physical_contacts().is_empty());
        let mut previous = 0;
        for contact in world.physical_contacts() {
            assert!(contact.time_ns >= previous);
            previous = contact.time_ns;
            assert_eq!(contact.substeps, slices);
            assert!((1..=slices).contains(&contact.substep));
            assert_eq!(contact.tick_end_ns, 8 * duration.as_ns());
            assert_eq!(
                contact.time_ns,
                7 * duration.as_ns()
                    + (u128::from(duration.as_ns()) * u128::from(contact.substep)
                        / u128::from(slices)) as u64
            );
            let (body_side, floor_side) = if contact.side1.participant.is_some() {
                (contact.side1, contact.side2)
            } else {
                (contact.side2, contact.side1)
            };
            assert_eq!(body_side.participant, Some(participant));
            assert_eq!(floor_side.participant, None);
            assert_eq!(
                world
                    .collider_state(floor_side.collider)
                    .unwrap()
                    .participant,
                None
            );
            assert_eq!(
                world
                    .collider_state(body_side.collider)
                    .unwrap()
                    .participant,
                Some(participant)
            );
            assert!(
                contact
                    .geometry
                    .point1_m
                    .into_iter()
                    .chain(contact.geometry.point2_m)
                    .chain(contact.geometry.normal_from1_to2)
                    .chain([contact.geometry.separation_m])
                    .all(f64::is_finite)
            );
            let normal = contact.geometry.normal_from1_to2;
            assert!((normal.into_iter().map(|x| x * x).sum::<f64>() - 1.).abs() < 1e-9);
            // A floor-to-body normal points up; reversing native side order
            // must reverse the direction without changing caller ownership.
            let expected_z = if contact.side1.participant.is_none() {
                1.
            } else {
                -1.
            };
            assert!((normal[2] - expected_z).abs() < 1e-9);
        }
        assert_eq!(
            world.participant_state(participant).unwrap(),
            world.body_state(body).unwrap()
        );
    }
}

#[test]
fn dynamic_pair_keeps_capture_ownership_after_body_removal_and_slot_reuse() {
    let mut world = WorldPhysics::new(&[], -10.);
    let first = install(&mut world, 1, [0., 0., 1.]);
    let second = install(&mut world, 2, [0.19, 0., 1.]);
    let p1 = world.body_participant(first).unwrap();
    let p2 = world.body_participant(second).unwrap();
    world.enable_contact_observations(100).unwrap();
    step(&mut world, 0);
    let contact = *world
        .physical_contacts()
        .first()
        .expect("overlapping dynamic cubes");
    assert!([Some(p1), Some(p2)].contains(&contact.side1.participant));
    assert!([Some(p1), Some(p2)].contains(&contact.side2.participant));
    assert_ne!(contact.side1.participant, contact.side2.participant);
    let old_collider = if contact.side1.participant == Some(p1) {
        contact.side1.collider
    } else {
        contact.side2.collider
    };
    world.remove_bound_body(first).unwrap();
    let replacement = install(&mut world, 1, [3., 0., 1.]);
    assert_ne!(p1, world.body_participant(replacement).unwrap());
    assert!(world.participant_state(p1).is_err());
    assert!(world.collider_state(old_collider).is_err());
    assert!(world.body_participant(first).is_err());
    assert_eq!(world.physical_contacts()[0], contact);
    assert!(world.participant_state(p2).is_ok());
}

#[test]
fn identity_is_world_scoped_and_reset_invalidates_native_tokens() {
    let cooked = cube();
    let scenario = RobotScenario::new(vec![], 0., StepDuration::HZ_1000, move |world| {
        world.install_body(BodyKey(1), &cooked, BodyState::at(Pose::at([0., 0., 1.])))?;
        Ok(())
    })
    .unwrap();
    let mut session = RobotSession::new(scenario).unwrap();
    let binding = session.ground_truth().body_binding(BodyKey(1)).unwrap();
    let participant = session.ground_truth().body_participant(binding).unwrap();
    let colliders: Vec<_> = session.ground_truth().colliders().collect();
    assert_eq!(
        session.ground_truth().participants().collect::<Vec<_>>(),
        [participant]
    );
    let mut foreign = WorldPhysics::new(&[], 0.);
    install(&mut foreign, 1, [0., 0., 1.]);
    assert!(foreign.participant_state(participant).is_err());
    assert!(foreign.body_participant(binding).is_err());
    for collider in &colliders {
        assert!(foreign.collider_state(*collider).is_err());
    }
    session.reset().unwrap();
    assert!(
        session
            .ground_truth()
            .participant_state(participant)
            .is_err()
    );
    for collider in colliders {
        assert!(session.ground_truth().collider_state(collider).is_err());
    }
    let binding = session.ground_truth().body_binding(BodyKey(1)).unwrap();
    assert_ne!(
        session.ground_truth().body_participant(binding).unwrap(),
        participant
    );
}

#[test]
fn capacity_is_tick_wide_and_drain_appends_without_losing_storage() {
    let fixture = || {
        let mut world = WorldPhysics::new(&[], 0.);
        install(&mut world, 1, [0., 0., 0.099]);
        install(&mut world, 2, [1., 0., 0.099]);
        world
    };
    let mut bounded = fixture();
    let mut full = fixture();
    bounded.enable_contact_observations(1).unwrap();
    full.enable_contact_observations(100).unwrap();
    step(&mut bounded, 0);
    step(&mut full, 0);
    assert_eq!(bounded.physical_contacts().len(), 1);
    assert_eq!(
        bounded.physical_contacts_dropped(),
        full.physical_contacts().len() - 1
    );
    let record = bounded.physical_contacts()[0];
    let pointer = bounded.physical_contacts().as_ptr();
    assert!(bounded.enable_contact_observations(0).is_err());
    assert_eq!(bounded.physical_contacts(), [record]);
    let dropped = bounded.physical_contacts_dropped();
    let mut output = Vec::with_capacity(16);
    output.push(record);
    let caller_pointer = output.as_ptr();
    bounded.drain_physical_contacts(&mut output);
    assert_eq!(output, [record, record]);
    assert_eq!(caller_pointer, output.as_ptr());
    assert!(bounded.physical_contacts().is_empty());
    assert_eq!(bounded.physical_contacts_dropped(), dropped);
    step(&mut bounded, 1);
    assert_eq!(bounded.physical_contacts().as_ptr(), pointer);
    assert!(
        bounded
            .physical_contacts()
            .iter()
            .all(|c| c.tick_end_ns == 2 * StepDuration::HZ_128.as_ns())
    );
    bounded.disable_contact_observations();
    step(&mut bounded, 2);
    assert!(bounded.physical_contacts().is_empty());
    assert_eq!(bounded.physical_contacts_dropped(), 0);
}

#[test]
fn observer_preserves_legacy_scoring_motion_and_retirement_at_both_rates() {
    let face = TargetFace {
        target: ArmorTarget::Outpost {
            outpost: 0,
            face: 0,
        },
        pose: Pose::yawed([1.5, 0., 1.], std::f64::consts::PI),
    };
    for duration in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let fixture = || {
            let mut world = WorldPhysics::with_step_duration(&[face], 0., duration);
            world
                .fire(
                    0,
                    Pose::at([0., 0., 1.025]),
                    Shot::at_limit(Caliber::Mm17),
                    None,
                )
                .unwrap();
            world
        };
        let mut disabled = fixture();
        let mut enabled = fixture();
        enabled.enable_contact_observations(128).unwrap();
        let frames = TargetFrames::new(vec![face]);
        let mut scored = 0;
        let mut observed = 0;
        for tick in 0..(5_000_000_000 / duration.as_ns()) {
            let expected = disabled.step(tick * duration.as_ns(), &frames).unwrap();
            let actual = enabled.step(tick * duration.as_ns(), &frames).unwrap();
            scored += actual.len();
            observed += enabled.physical_contacts().len();
            assert_eq!(actual, expected);
            assert_eq!(enabled.snapshot(), disabled.snapshot());
            assert_eq!(enabled.armor_collisions(), disabled.armor_collisions());
            assert_eq!(
                enabled.removals().collect::<Vec<_>>(),
                disabled.removals().collect::<Vec<_>>()
            );
            assert!(disabled.physical_contacts().is_empty());
            assert_eq!(disabled.physical_contacts_dropped(), 0);
        }
        assert!(scored > 0);
        assert!(observed > 0);
        assert!(enabled.snapshot().is_empty());
    }
}

#[test]
fn successful_idle_step_clears_previous_observations() {
    let mut world = WorldPhysics::new(&[], 0.);
    let body = install(&mut world, 1, [0., 0., 0.099]);
    world.enable_contact_observations(1).unwrap();
    step(&mut world, 0);
    assert!(!world.physical_contacts().is_empty());
    assert!(world.physical_contacts_dropped() > 0);
    world.remove_bound_body(body).unwrap();
    step(&mut world, 1);
    assert!(world.physical_contacts().is_empty());
    assert_eq!(world.physical_contacts_dropped(), 0);
}
