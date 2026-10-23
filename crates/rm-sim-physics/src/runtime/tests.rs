// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Standalone lifecycle, query and compatibility schedule parity.
use super::*;
use crate::cooking::{CookedRigidBody, CookedTriangleMesh, TriangleMesh};
use crate::queries::RayQuery;
use rm_sim_engine::model::{
    BodyKind, BodyState, CarrierKey, ColliderDescription, ContactMaterial, Inertial,
    JointDescription, JointKind, RigidBodyDescription, ShapeDescription,
};

fn material() -> ContactMaterial {
    ContactMaterial {
        friction: 0.4,
        restitution: 0.45,
        memberships: u32::MAX,
        filter: u32::MAX,
    }
}
fn cube(kind: BodyKind) -> CookedRigidBody {
    CookedRigidBody::new(
        RigidBodyDescription::new(
            kind,
            vec![
                ColliderDescription::new(
                    ShapeDescription::Cuboid {
                        half_extents_m: [0.1; 3],
                    },
                    Pose::default(),
                    material(),
                )
                .unwrap(),
            ],
        )
        .unwrap(),
    )
    .unwrap()
}
fn dynamic_cube() -> CookedRigidBody {
    cube(BodyKind::Dynamic(
        Inertial::new(1., [0.01; 3], Pose::default()).unwrap(),
    ))
}
fn hinge() -> JointDescription {
    JointDescription::new(
        JointKind::Revolute,
        Pose::default(),
        Pose::default(),
        None,
        false,
    )
    .unwrap()
}

// The same public commands exercise the generic owner and compatibility forwarders.
macro_rules! install_fixture {
    ($runtime:expr) => {{
        let runtime = &mut $runtime;
        let body = runtime
            .install_body(
                BodyKey(1),
                &dynamic_cube(),
                BodyState::at(Pose::at([0., 0., 0.099])),
            )
            .unwrap();
        let prescribed = runtime
            .install_body(
                BodyKey(2),
                &cube(BodyKind::Kinematic),
                BodyState::at(Pose::at([-0.7, 0., 0.1])),
            )
            .unwrap();
        let anchor = runtime
            .install_body(
                BodyKey(3),
                &cube(BodyKind::Fixed),
                BodyState::at(Pose::at([2., 0., 1.])),
            )
            .unwrap();
        let child = runtime
            .install_body(
                BodyKey(4),
                &dynamic_cube(),
                BodyState::at(Pose::at([2., 0., 1.])),
            )
            .unwrap();
        runtime
            .install_joint(JointKey(1), anchor, child, &hinge())
            .unwrap();
        runtime
            .set_body_wrench(body, [1., 0.5, 0.], [0., 0., 0.2])
            .unwrap();
        let joint = runtime.joint_binding(JointKey(1)).unwrap();
        runtime.set_joint_effort(joint, 0.3).unwrap();
        runtime.enable_contact_observations(256).unwrap();
        (body, prescribed, child, joint)
    }};
}

#[test]
fn standalone_owner_matches_facade_states_joints_and_ordered_observations_at_both_rates() {
    for duration in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let mut runtime = PhysicalRuntime::new(duration);
        runtime.install_catch_floor(0., material()).unwrap();
        let mut facade = WorldPhysics::with_step_duration(&[], 0., duration);
        let (body, prescribed, child, joint) = install_fixture!(runtime);
        let (facade_body, facade_prescribed, facade_child, facade_joint) = install_fixture!(facade);
        let mut observed = 0;
        for tick in 0..64 {
            let endpoint = Pose::at([-0.7 + tick as f64 * 0.005, 0., 0.1]);
            runtime.set_body_endpoint(prescribed, endpoint).unwrap();
            facade
                .set_body_endpoint(facade_prescribed, endpoint)
                .unwrap();
            runtime.step(tick * duration.as_ns()).unwrap();
            facade
                .step(tick * duration.as_ns(), &crate::TargetFrames::new(vec![]))
                .unwrap();
            for (a, b) in [
                (body, facade_body),
                (prescribed, facade_prescribed),
                (child, facade_child),
            ] {
                assert_eq!(runtime.body_state(a), facade.body_state(b));
            }
            assert_eq!(runtime.joint_state(joint), facade.joint_state(facade_joint));
            let generic_colliders: Vec<_> = runtime.colliders().collect();
            let facade_colliders: Vec<_> = facade.colliders().collect();
            let a = runtime.physical_contacts();
            let b = facade.physical_contacts();
            assert_eq!(a.len(), b.len());
            observed += a.len();
            for (a, b) in a.iter().zip(b) {
                assert_eq!(
                    (a.time_ns, a.tick_end_ns, a.substep, a.substeps, a.geometry),
                    (b.time_ns, b.tick_end_ns, b.substep, b.substeps, b.geometry)
                );
                for (left, right) in [(a.side1, b.side1), (a.side2, b.side2)] {
                    assert_eq!(
                        generic_colliders.iter().position(|&c| c == left.collider),
                        facade_colliders.iter().position(|&c| c == right.collider)
                    );
                    assert_eq!(left.participant.is_some(), right.participant.is_some());
                }
            }
        }
        assert!(observed > 0);
    }
}

#[test]
fn recipe_reconstruction_invalidates_tokens_and_native_reuse_changes_carrier_generation() {
    fn recipe(runtime: &mut PhysicalRuntime) -> Result<(), &'static str> {
        runtime.install_body(
            BodyKey(1),
            &dynamic_cube(),
            BodyState::at(Pose::at([0., 0., 1.])),
        )?;
        runtime.install_body(
            BodyKey(2),
            &cube(BodyKind::Fixed),
            BodyState::at(Pose::at([0., 0., 1.])),
        )?;
        runtime.install_joint(
            JointKey(1),
            runtime.body_binding(BodyKey(2)).unwrap(),
            runtime.body_binding(BodyKey(1)).unwrap(),
            &hinge(),
        )?;
        Ok(())
    }
    let mut runtime = PhysicalRuntime::from_recipe(StepDuration::HZ_1000, recipe).unwrap();
    let body = runtime.body_binding(BodyKey(1)).unwrap();
    let joint = runtime.joint_binding(JointKey(1)).unwrap();
    let participant = runtime.body_participant(body).unwrap();
    let carrier = runtime
        .observe_carrier(participant, CarrierKey(9), 2, 10, 0.001)
        .unwrap();
    assert_eq!(
        carrier.world_from_carrier(),
        runtime.body_state(body).unwrap().pose
    );
    assert_eq!(carrier.identity().world_brand, runtime.runtime_identity());
    assert_eq!(carrier.identity().carrier, CarrierKey(9));
    assert_eq!(carrier.identity().discontinuity_revision, 2);
    assert_eq!(carrier.interval_s().to_bits(), 0.001f64.to_bits());
    assert!(
        runtime
            .observe_carrier(participant, CarrierKey(9), 2, 10, 0.)
            .is_err()
    );
    runtime.remove_bound_body(body).unwrap();
    assert!(runtime.joint_state(joint).is_err());
    assert!(runtime.participant_state(participant).is_err());
    let replacement = runtime
        .install_body(
            BodyKey(1),
            &dynamic_cube(),
            BodyState::at(Pose::at([0., 0., 1.])),
        )
        .unwrap();
    assert_ne!(body, replacement);
    assert!(runtime.set_body_wrench(body, [0.; 3], [0.; 3]).is_err());
    let replacement_participant = runtime.body_participant(replacement).unwrap();
    let new_carrier = runtime
        .observe_carrier(replacement_participant, CarrierKey(9), 2, 10, 0.001)
        .unwrap();
    assert_ne!(
        carrier.identity().installation_generation,
        new_carrier.identity().installation_generation
    );
    // Moving the owner preserves its scene and tokens.
    let moved = runtime;
    assert!(moved.participant_state(replacement_participant).is_ok());
    let rebuilt = PhysicalRuntime::from_recipe(StepDuration::HZ_1000, recipe).unwrap();
    assert_ne!(moved.runtime_identity(), rebuilt.runtime_identity());
    assert!(rebuilt.body_state(replacement).is_err());
    assert!(
        rebuilt
            .observe_carrier(replacement_participant, CarrierKey(9), 2, 10, 0.001)
            .is_err()
    );
    assert_eq!(
        rebuilt
            .body_state(rebuilt.body_binding(BodyKey(1)).unwrap())
            .unwrap()
            .pose,
        Pose::at([0., 0., 1.])
    );
}

#[test]
fn physical_geometry_retains_floor_shared_shapes_identity_and_capture_local_ray_ties() {
    let mut runtime = PhysicalRuntime::new(StepDuration::HZ_128);
    assert!(runtime.install_catch_floor(f64::NAN, material()).is_err());
    assert_eq!(runtime.colliders().count(), 0);
    let floor = runtime.install_catch_floor(0., material()).unwrap();
    let mesh = CookedTriangleMesh::new(
        TriangleMesh::new(
            vec![[2., -2., -2.], [2., 2., -2.], [2., 0., 2.]],
            vec![[0, 1, 2]],
        )
        .unwrap(),
    )
    .unwrap();
    let first = runtime
        .install_fixed_mesh(&mesh, Pose::default(), material())
        .unwrap();
    let second = runtime
        .install_fixed_mesh(&mesh, Pose::default(), material())
        .unwrap();
    let capture = runtime.geometry_capture();
    assert_eq!(capture.runtime_identity(), runtime.runtime_identity());
    let captures: Vec<_> = capture.sensor_colliders().collect();
    assert_eq!(captures.len(), 3);
    assert!(captures.iter().all(|entry| entry.4));
    assert!(std::ptr::eq(captures[1].3.as_ref(), captures[2].3.as_ref()));
    assert!(std::ptr::eq(captures[1].3.as_ref(), mesh.shape.as_ref()));
    let downward = RayQuery::new([0., 0., 1.], [0., 0., -1.], 1.).unwrap();
    let across = RayQuery::new([0., 0., 0.5], [1., 0., 0.], 2.).unwrap();
    let floor_hit = capture.cast_ray(&downward).unwrap();
    assert_eq!(floor_hit.distance_m, 1.);
    assert_eq!(capture.collider_binding(floor_hit.shape_index), Some(floor));
    // Expanding the native traversal bound must still reject the next ULP beyond range.
    let just_short = RayQuery::new([0., 0., 0.5], [1., 0., 0.], 2f64.next_down()).unwrap();
    assert!(capture.cast_ray(&just_short).is_none());
    let enormous = RayQuery::new([0., 0., 1.], [0., 0., -1.], f64::MAX).unwrap();
    assert_eq!(capture.cast_ray(&enormous), Some(floor_hit));
    let mesh_hit = capture.cast_ray(&across).unwrap();
    assert_eq!(mesh_hit.distance_m, 2.);
    assert_eq!(capture.collider_binding(mesh_hit.shape_index), Some(first));
    let mut out = [Some(mesh_hit); 2];
    assert!(capture.cast_rays_into(&[downward], &mut out).is_err());
    assert_eq!(out, [Some(mesh_hit); 2]);
    capture
        .cast_rays_into(&[downward, across], &mut out)
        .unwrap();
    assert_eq!(out, [Some(floor_hit), Some(mesh_hit)]);
    runtime.remove_fixed_collider(first).unwrap();
    runtime.remove_fixed_collider(floor).unwrap();
    assert!(runtime.collider_state(first).is_err());
    assert!(capture.collider_state(first).is_some());
    assert_eq!(capture.cast_ray(&downward), Some(floor_hit));
    assert_eq!(runtime.geometry_capture().collider_binding(0), Some(second));
    assert!(runtime.geometry_capture().cast_ray(&downward).is_none());
    let foreign = PhysicalRuntime::new(StepDuration::HZ_128);
    assert!(foreign.collider_state(second).is_err());
    let body = runtime
        .install_body(
            BodyKey(1),
            &dynamic_cube(),
            BodyState::at(Pose::at([0., 0., 1.])),
        )
        .unwrap();
    let attached = runtime
        .geometry_capture()
        .colliders()
        .find(|(_, state)| state.participant == Some(runtime.body_participant(body).unwrap()))
        .unwrap()
        .0;
    assert!(runtime.remove_fixed_collider(attached).is_err());
    assert!(runtime.body_state(body).is_ok());
}

#[test]
fn bounded_observations_and_overflow_preserve_the_existing_step_error_contract() {
    let mut runtime = PhysicalRuntime::new(StepDuration::HZ_128);
    runtime.install_catch_floor(0., material()).unwrap();
    let body = runtime
        .install_body(
            BodyKey(1),
            &dynamic_cube(),
            BodyState::at(Pose::at([0., 0., 0.099])),
        )
        .unwrap();
    runtime.enable_contact_observations(1).unwrap();
    runtime.step(0).unwrap();
    assert_eq!(runtime.physical_contacts().len(), 1);
    assert!(runtime.physical_contacts_dropped() > 0);
    let retained = runtime.physical_contacts().to_vec();
    let state = runtime.body_state(body).unwrap();
    assert_eq!(runtime.step(u64::MAX), Err("physics time overflow"));
    assert_eq!(runtime.physical_contacts(), retained);
    assert_eq!(runtime.body_state(body).unwrap(), state);
    let dropped = runtime.physical_contacts_dropped();
    let mut drained = Vec::new();
    runtime.drain_physical_contacts(&mut drained);
    assert_eq!(drained, retained);
    assert_eq!(runtime.physical_contacts_dropped(), dropped);
    runtime.disable_contact_observations();
    assert!(runtime.physical_contacts().is_empty());
    assert_eq!(runtime.physical_contacts_dropped(), 0);
}

#[test]
fn placement_epoch_exhaustion_preserves_native_state_and_never_wraps() {
    let mut facade = WorldPhysics::new(&[], 0.);
    let id = facade
        .add_chassis(
            crate::Team::Red,
            crate::chassis::ChassisConfig::default(),
            Pose::at([0., 0., 0.3]),
        )
        .unwrap();
    let snapshot = facade.chassis_snapshot(id).unwrap();
    facade.physical.placement_epoch = u64::MAX;
    assert_eq!(
        facade.physical.mark_placement_discontinuity(),
        Err("physical placement epoch exhausted")
    );
    assert_eq!(
        facade.place_chassis(id, Pose::at([2., 0., 0.3])),
        Err("physical placement epoch exhausted")
    );
    let mut reset = snapshot.clone();
    reset.pose.translation_m[0] = 3.;
    assert_eq!(
        facade.reset_chassis(&reset),
        Err("physical placement epoch exhausted")
    );
    assert_eq!(facade.chassis_snapshot(id).unwrap(), snapshot);
    assert_eq!(facade.physical.placement_epoch(), u64::MAX);
    facade.set_chassis_defeated(id, true);
    let defeated = facade.chassis_snapshot(id).unwrap();
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        facade.set_chassis_defeated(id, false)
    }));
    assert!(result.is_err());
    assert_eq!(facade.chassis_snapshot(id).unwrap(), defeated);
    assert_eq!(facade.physical.placement_epoch(), u64::MAX);
}

#[test]
fn placement_epoch_covers_late_prediction_error_even_when_saved_revision_is_unchanged() {
    let mut facade = WorldPhysics::new(&[], 0.);
    let id = facade
        .add_chassis(
            crate::Team::Red,
            crate::chassis::ChassisConfig::default(),
            Pose::at([0., 0., 0.3]),
        )
        .unwrap();
    let mut reset = facade.chassis_snapshot(id).unwrap();
    let before = facade.physical.geometry_capture();
    reset.pose.translation_m[0] = 3.;
    reset.command.forward_m_s = f64::NAN;
    assert!(facade.reset_chassis(&reset).is_err());
    assert_eq!(facade.chassis_position_m(id).unwrap()[0], 3.);
    assert_eq!(facade.chassis_revision(id), Some(reset.placement_revision));
    assert_ne!(
        before.placement_epoch(),
        facade.physical.geometry_capture().placement_epoch()
    );
}

#[test]
fn target_epoch_changes_only_for_actual_discontinuous_placement_and_preflights_overflow() {
    let face = crate::TargetFace {
        target: crate::ArmorTarget::Rune { rune: 0, blade: 0 },
        pose: Pose::at([5., 0., 3.]),
    };
    let duration = StepDuration::HZ_1000;
    let mut facade = WorldPhysics::with_step_duration(&[face], 0., duration);
    let body = facade
        .install_body(
            BodyKey(1),
            &dynamic_cube(),
            BodyState::at(Pose::at([0., 0., 1.])),
        )
        .unwrap();
    let mut frames = crate::TargetFrames::new(vec![face]);
    facade.step(0, &frames).unwrap();
    assert_eq!(facade.physical.geometry_capture().placement_epoch(), 0);
    frames
        .update(|faces| faces[0].pose.translation_m[1] = 0.01)
        .unwrap();
    facade.step(duration.as_ns(), &frames).unwrap();
    assert_eq!(facade.physical.placement_epoch(), 0);
    frames
        .begin(|faces| faces[0].pose.translation_m[1] = 2.)
        .unwrap();
    facade.physical.placement_epoch = u64::MAX;
    let before = facade.body_state(body).unwrap();
    assert_eq!(
        facade.step(10 * duration.as_ns(), &frames),
        Err("physical placement epoch exhausted")
    );
    assert_eq!(facade.body_state(body).unwrap(), before);
    facade.physical.placement_epoch = 0;
    facade.step(10 * duration.as_ns(), &frames).unwrap();
    assert_eq!(facade.physical.placement_epoch(), 1);
    // A scene with no active participants does not place targets during step.
    facade.remove_bound_body(body).unwrap();
    facade.physical.placement_epoch = u64::MAX;
    frames
        .begin(|faces| faces[0].pose.translation_m[1] = 4.)
        .unwrap();
    facade.step(20 * duration.as_ns(), &frames).unwrap();
    assert_eq!(facade.physical.placement_epoch(), u64::MAX);
}

#[test]
fn standalone_solver_budget_is_bounded_and_preserves_default_timing() {
    let default = PhysicalRuntime::new(StepDuration::HZ_1000);
    assert_eq!(default.solver_iterations(), 4);
    for iterations in [1, 16, 32, 64] {
        let tuned =
            PhysicalRuntime::with_solver_iterations(StepDuration::HZ_1000, iterations).unwrap();
        assert_eq!(tuned.solver_iterations(), iterations);
        assert_eq!(tuned.step_duration(), default.step_duration());
        assert_eq!(
            tuned.world.integration_parameters.dt,
            default.world.integration_parameters.dt
        );
        assert_eq!(
            tuned.world.integration_parameters.max_ccd_substeps,
            default.world.integration_parameters.max_ccd_substeps
        );
    }
    for invalid in [0, 65, usize::MAX] {
        assert!(PhysicalRuntime::with_solver_iterations(StepDuration::HZ_1000, invalid).is_err());
    }
}
