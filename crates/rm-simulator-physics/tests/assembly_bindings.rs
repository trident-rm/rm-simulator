// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Source trees prepare once and install atomically into the ordinary world.
use rm_simulator_engine::{
    Pose,
    model::{
        AssemblyDescription, AssemblyJoint, AssemblyJointId, AssemblyLink, AssemblyLinkId,
        BodyKind, BodyState, ColliderDescription, ContactMaterial, Inertial, JointDescription,
        JointKind, RigidBodyDescription, ShapeDescription,
    },
};
use rm_simulator_physics::{
    cooking::CookedAssembly,
    runtime::{AssemblyKeys, BodyKey, JointKey, WorldPhysics},
};

fn body(kind: BodyKind) -> RigidBodyDescription {
    RigidBodyDescription::new(
        kind,
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
    .unwrap()
}
fn dynamic() -> BodyKind {
    BodyKind::Dynamic(Inertial::new(2., [1.; 3], Pose::at([0., 0., 0.1])).unwrap())
}
fn source(child: BodyKind, child_pose: Pose) -> AssemblyDescription {
    // Reversed source order proves installation does not assume root-first IDs.
    AssemblyDescription::new(
        AssemblyLinkId(7),
        vec![
            AssemblyLink::new(AssemblyLinkId(9), body(child), BodyState::at(child_pose)).unwrap(),
            AssemblyLink::new(
                AssemblyLinkId(7),
                body(BodyKind::Fixed),
                BodyState::at(Pose::default()),
            )
            .unwrap(),
        ],
        vec![AssemblyJoint::new(
            AssemblyJointId(6),
            AssemblyLinkId(7),
            AssemblyLinkId(9),
            JointDescription::new(
                JointKind::Revolute,
                Pose::default(),
                Pose::default(),
                None,
                false,
            )
            .unwrap(),
        )],
    )
    .unwrap()
}
fn keys(offset: u64) -> AssemblyKeys {
    // Mapping order is intentionally different from source array order.
    AssemblyKeys {
        links: vec![
            (AssemblyLinkId(7), BodyKey(offset)),
            (AssemblyLinkId(9), BodyKey(offset + 1)),
        ],
        joints: vec![(AssemblyJointId(6), JointKey(offset))],
    }
}

#[test]
fn preparation_rejects_backend_modes_and_inconsistent_frames_without_changing_source() {
    for source in [
        source(BodyKind::Fixed, Pose::default()),
        source(dynamic(), Pose::at([1., 0., 0.])),
    ] {
        let original = source.clone();
        assert!(CookedAssembly::new(source).is_err());
        assert_eq!(original.links().len(), 2);
    }
    assert!(CookedAssembly::new(source(dynamic(), Pose::default())).is_ok());
}

#[test]
fn mapping_and_placement_errors_leave_live_bodies_joints_and_available_keys_unchanged() {
    let prepared = CookedAssembly::new(source(dynamic(), Pose::default())).unwrap();
    let mut world = WorldPhysics::new(&[], -100.);
    let existing = world
        .install_assembly(&prepared, Pose::at([2., 0., 0.]), &keys(10))
        .unwrap();
    let before = world
        .body_state(existing.body(AssemblyLinkId(9)).unwrap())
        .unwrap();
    let joint_before = world
        .joint_state(existing.joint(AssemblyJointId(6)).unwrap())
        .unwrap();
    let mut missing = keys(20);
    missing.links.pop();
    let mut extra = keys(20);
    extra.links.push((AssemblyLinkId(3), BodyKey(30)));
    let mut repeated_id = keys(20);
    repeated_id.links[0].0 = AssemblyLinkId(9);
    let mut repeated_key = keys(20);
    repeated_key.links[0].1 = repeated_key.links[1].1;
    let mut unknown = keys(20);
    unknown.links[0].0 = AssemblyLinkId(300);
    let mut occupied_body = keys(20);
    occupied_body.links[1].1 = BodyKey(10);
    let mut occupied_joint = keys(20);
    occupied_joint.joints[0].1 = JointKey(10);
    let mut unknown_joint = keys(20);
    unknown_joint.joints[0].0 = AssemblyJointId(200);
    for bad in [
        missing,
        extra,
        repeated_id,
        repeated_key,
        unknown,
        occupied_body,
        occupied_joint,
        unknown_joint,
    ] {
        assert!(
            world
                .install_assembly(&prepared, Pose::default(), &bad)
                .is_err()
        );
        assert!(world.body_binding(BodyKey(20)).is_none());
        assert!(world.body_binding(BodyKey(21)).is_none());
        assert!(world.joint_binding(JointKey(20)).is_none());
        assert_eq!(
            world
                .body_state(existing.body(AssemblyLinkId(9)).unwrap())
                .unwrap(),
            before
        );
        assert_eq!(
            world
                .joint_state(existing.joint(AssemblyJointId(6)).unwrap())
                .unwrap(),
            joint_before
        );
    }
    for placement in [
        Pose::at([f64::NAN, 0., 0.]),
        Pose {
            rotation_wxyz: [0.; 4],
            ..Pose::default()
        },
    ] {
        assert!(
            world
                .install_assembly(&prepared, placement, &keys(20))
                .is_err()
        );
        assert!(world.body_binding(BodyKey(20)).is_none());
    }
    let accepted = world
        .install_assembly(&prepared, Pose::at([-2., 0., 0.]), &keys(20))
        .unwrap();
    assert_eq!(accepted.links()[0].0, AssemblyLinkId(9));
    assert_eq!(accepted.links()[1].0, AssemblyLinkId(7));
    assert_eq!(accepted.body(AssemblyLinkId(7)).unwrap().key(), BodyKey(20));
    assert_eq!(
        accepted.joint(AssemblyJointId(6)).unwrap().key(),
        JointKey(20)
    );
}

#[test]
fn placement_rotates_common_frame_motion_instead_of_adding_platform_velocity() {
    let id = AssemblyLinkId(3);
    let initial = BodyState {
        pose: Pose::at([1., 0., 2.]),
        velocity_m_s: [2., 0., 0.],
        angular_velocity_rad_s: [0., 3., 0.],
    };
    let source = AssemblyDescription::new(
        id,
        vec![AssemblyLink::new(id, body(dynamic()), initial).unwrap()],
        vec![],
    )
    .unwrap();
    let prepared = CookedAssembly::new(source).unwrap();
    let mut world = WorldPhysics::new(&[], -100.);
    let live = world
        .install_assembly(
            &prepared,
            Pose::yawed([4., 5., 6.], std::f64::consts::FRAC_PI_2),
            &AssemblyKeys {
                links: vec![(id, BodyKey(3))],
                joints: vec![],
            },
        )
        .unwrap();
    let observed = world.body_state(live.body(id).unwrap()).unwrap();
    for (actual, expected) in observed
        .pose
        .translation_m
        .into_iter()
        .zip([4., 6., 8.])
        .chain(observed.velocity_m_s.into_iter().zip([0., 2., 0.]))
        .chain(
            observed
                .angular_velocity_rad_s
                .into_iter()
                .zip([-3., 0., 0.]),
        )
    {
        assert!((actual - expected).abs() < 1e-12);
    }
    world.remove_assembly(&live).unwrap();
    assert!(world.body_state(live.body(id).unwrap()).is_err());
}

#[test]
fn removal_checks_all_tokens_before_mutation_and_cascades_external_joints() {
    let prepared = CookedAssembly::new(source(dynamic(), Pose::default())).unwrap();
    let mut world = WorldPhysics::new(&[], -100.);
    let first = world
        .install_assembly(&prepared, Pose::default(), &keys(10))
        .unwrap();
    let mut foreign = WorldPhysics::new(&[], -100.);
    assert!(foreign.remove_assembly(&first).is_err());
    // Removing one joint makes the aggregate stale but must not remove remaining bodies.
    world
        .remove_bound_joint(first.joint(AssemblyJointId(6)).unwrap())
        .unwrap();
    assert!(world.remove_assembly(&first).is_err());
    assert!(
        first
            .links()
            .iter()
            .all(|(_, binding)| world.body_state(*binding).is_ok())
    );
    for &(_, binding) in first.links() {
        world.remove_bound_body(binding).unwrap();
    }
    let replacement = world
        .install_assembly(&prepared, Pose::default(), &keys(10))
        .unwrap();
    assert!(world.remove_assembly(&first).is_err());
    assert!(
        replacement
            .links()
            .iter()
            .all(|(_, binding)| world.body_state(*binding).is_ok())
    );
    let other = world
        .install_assembly(&prepared, Pose::default(), &keys(20))
        .unwrap();
    let external = world
        .install_joint(
            JointKey(99),
            replacement.body(AssemblyLinkId(9)).unwrap(),
            other.body(AssemblyLinkId(9)).unwrap(),
            &JointDescription::new(
                JointKind::Fixed,
                Pose::default(),
                Pose::default(),
                None,
                false,
            )
            .unwrap(),
        )
        .unwrap();
    world.remove_assembly(&replacement).unwrap();
    assert!(world.joint_state(external).is_err());
    assert!(
        other
            .links()
            .iter()
            .all(|(_, binding)| world.body_state(*binding).is_ok())
    );
    assert!(
        world
            .joint_state(other.joint(AssemblyJointId(6)).unwrap())
            .is_ok()
    );
}

#[test]
fn finite_source_translation_overflow_rejects_placement_before_any_installation() {
    let id = AssemblyLinkId(1);
    let source = AssemblyDescription::new(
        id,
        vec![
            AssemblyLink::new(
                id,
                body(BodyKind::Fixed),
                BodyState::at(Pose::at([1e308, 0., 0.])),
            )
            .unwrap(),
        ],
        vec![],
    )
    .unwrap();
    let prepared = CookedAssembly::new(source).unwrap();
    let mut world = WorldPhysics::new(&[], -100.);
    let keys = AssemblyKeys {
        links: vec![(id, BodyKey(1))],
        joints: vec![],
    };
    assert!(
        world
            .install_assembly(&prepared, Pose::at([1e308, 0., 0.]), &keys)
            .is_err()
    );
    assert!(world.body_binding(BodyKey(1)).is_none());
    assert!(world.is_idle());
    assert!(
        world
            .install_assembly(&prepared, Pose::default(), &keys)
            .is_ok()
    );
}
