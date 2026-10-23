// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Independent pre-port constructors pin native insertion and all debug properties.
use super::*;

impl WorldPhysics {
    fn oracle_add_target(&mut self, face: TargetFace) {
        let housing_half_m = match face.target {
            ArmorTarget::Outpost { .. } | ArmorTarget::Base { .. } => SMALL_ARMOR_HOUSING_HALF_M,
            ArmorTarget::Rune { .. } => RUNE_HOUSING_HALF_M,
            ArmorTarget::Chassis { .. } => SMALL_ARMOR_HOUSING_HALF_M,
        };
        let body = self.physical.world.insert_body(
            RigidBodyBuilder::kinematic_position_based()
                .pose(pose(face.pose))
                .can_sleep(false),
        );
        let [x, y, z] = housing_half_m;
        let collider = self.physical.world.insert_collider(
            ColliderBuilder::cuboid(x, y, z)
                .translation(Vector::new(-x, 0., 0.))
                .friction(FRICTION)
                .restitution(RESTITUTION),
            Some(body),
        );
        self.rm_contacts
            .target_of
            .insert(collider, Scorer::Face(self.targets.len()));
        self.targets.push(TargetBody {
            target: face.target,
            body,
        });
    }
    fn oracle_fire(
        &mut self,
        time_ns: u64,
        muzzle: Pose,
        shot: Shot,
        shooter: Option<u32>,
    ) -> Result<u64, &'static str> {
        if !pose_is_valid(muzzle) {
            return Err("muzzle pose must be finite with a unit rotation");
        }
        if let Some(id) = shooter {
            match self.chassis_defeated(id) {
                None => return Err("no chassis with that id"),
                Some(true) => return Err("a defeated robot cannot fire"),
                Some(false) => {}
            }
        }
        if !shot.speed_m_s.is_finite() || shot.speed_m_s <= 0. || shot.speed_m_s > 40. {
            return Err("launch speed must be in (0, 40] m/s");
        }
        if self.projectiles.len() >= MAX_PROJECTILES {
            let oldest = self.projectiles.remove(0);
            self.record_removal(&oldest, time_ns, RemovalReason::Capped);
            self.physical.world.remove_body(oldest.body);
        }
        let muzzle = pose(muzzle);
        let velocity = muzzle.transform_vector(Vector::new(shot.speed_m_s, 0., 0.));
        let radius = shot.caliber.diameter_m() / 2.;
        let body = self.physical.world.insert_body(
            RigidBodyBuilder::dynamic()
                .translation(muzzle.translation)
                .linvel(velocity)
                .ccd_enabled(true)
                .can_sleep(false),
        );
        let collider = self.physical.world.insert_collider(
            ColliderBuilder::ball(radius)
                .collision_groups(InteractionGroups::new(
                    Group::GROUP_2,
                    Group::ALL,
                    InteractionTestMode::And,
                ))
                .mass(shot.caliber.mass_kg())
                .friction(FRICTION)
                .restitution(self.policy.restitution)
                .restitution_combine_rule(CoefficientCombineRule::Min),
            Some(body),
        );
        let id = self.next_id;
        self.next_id += 1;
        self.launched += 1;
        self.projectiles.push(Projectile {
            id,
            shooter,
            caliber: shot.caliber,
            launched_ns: time_ns,
            body,
            collider,
            touching: Vec::new(),
            first_contact_ns: None,
            dwell_since_ns: None,
        });
        Ok(id)
    }
    fn oracle_restore_projectile(
        &mut self,
        state: &ProjectileSnapshot,
    ) -> Result<(), &'static str> {
        if !state
            .position_m
            .into_iter()
            .chain(state.velocity_m_s)
            .chain(state.angular_velocity_rad_s)
            .all(f64::is_finite)
        {
            return Err("invalid projectile checkpoint");
        }
        if self.projectiles.len() >= MAX_PROJECTILES {
            return Err("too many restored projectiles");
        }
        let radius = state.caliber.diameter_m() / 2.;
        let body = self.physical.world.insert_body(
            RigidBodyBuilder::dynamic()
                .translation(vector(state.position_m))
                .linvel(vector(state.velocity_m_s))
                .angvel(vector(state.angular_velocity_rad_s))
                .ccd_enabled(true)
                .can_sleep(false),
        );
        let collider = self.physical.world.insert_collider(
            ColliderBuilder::ball(radius)
                .collision_groups(InteractionGroups::new(
                    Group::GROUP_2,
                    Group::ALL,
                    InteractionTestMode::And,
                ))
                .mass(state.caliber.mass_kg())
                .friction(FRICTION)
                .restitution(self.policy.restitution)
                .restitution_combine_rule(CoefficientCombineRule::Min),
            Some(body),
        );
        self.projectiles.push(Projectile {
            id: state.id,
            shooter: state.shooter,
            caliber: state.caliber,
            launched_ns: state.launched_ns,
            body,
            collider,
            touching: Vec::new(),
            first_contact_ns: state.first_contact_ns,
            dwell_since_ns: state.dwell_since_ns,
        });
        Ok(())
    }
}

fn oracle_world(faces: &[TargetFace]) -> WorldPhysics {
    let mut world = WorldPhysics::new(&[], -1.);
    for face in faces {
        world.oracle_add_target(*face);
    }
    world
}

fn assert_native_equal(actual: &WorldPhysics, expected: &WorldPhysics) {
    assert_eq!(
        format!("{:?}", actual.physical.world.bodies),
        format!("{:?}", expected.physical.world.bodies)
    );
    assert_eq!(
        format!("{:?}", actual.physical.world.colliders),
        format!("{:?}", expected.physical.world.colliders)
    );
    assert_eq!(actual.snapshot(), expected.snapshot());
    assert_eq!(
        actual.removals().copied().collect::<Vec<_>>(),
        expected.removals().copied().collect::<Vec<_>>()
    );
}

#[test]
fn prepared_shots_targets_and_restore_match_native_constructors() {
    let faces = [
        TargetFace {
            target: ArmorTarget::Outpost {
                outpost: 0,
                face: 0,
            },
            pose: Pose::at([2., 0., 1.]),
        },
        TargetFace {
            target: ArmorTarget::Rune { rune: 0, blade: 0 },
            pose: Pose::at([4., 0., 1.]),
        },
    ];
    let mut actual = WorldPhysics::new(&faces, -1.);
    let mut expected = oracle_world(&faces);
    assert_native_equal(&actual, &expected);
    for restitution in [0.45, 0., 1., 0.45] {
        let policy = ProjectilePolicy {
            restitution,
            ..Default::default()
        };
        actual.set_projectile_policy(policy).unwrap();
        expected.set_projectile_policy(policy).unwrap();
        for caliber in [Caliber::Mm17, Caliber::Mm42] {
            let shot = Shot {
                caliber,
                speed_m_s: 25.,
            };
            let muzzle = Pose::yawed([0., 0., 1.], 0.3);
            assert_eq!(
                actual.fire(0, muzzle, shot, None),
                expected.oracle_fire(0, muzzle, shot, None)
            );
            assert_native_equal(&actual, &expected);
        }
    }
    let mut state = actual.snapshot()[0].clone();
    state.angular_velocity_rad_s = [1., -2., 3.];
    state.velocity_m_s = [2., 3., -4.];
    state.first_contact_ns = Some(5);
    state.dwell_since_ns = Some(8);
    let mut restored = WorldPhysics::new(&faces, -1.);
    let mut native_restored = oracle_world(&faces);
    restored.restore_projectile(&state).unwrap();
    native_restored.oracle_restore_projectile(&state).unwrap();
    assert_native_equal(&restored, &native_restored);
}

#[test]
fn prepared_cache_reuses_shapes_and_changes_only_new_collider_policy() {
    use std::sync::Arc;
    let mut world = WorldPhysics::new(&[], -1.);
    for caliber in [Caliber::Mm17, Caliber::Mm42] {
        for _ in 0..2 {
            world
                .fire(
                    0,
                    Pose::default(),
                    Shot {
                        caliber,
                        speed_m_s: 1.,
                    },
                    None,
                )
                .unwrap();
        }
    }
    let colliders = &world.physical.world.colliders;
    for pair in world.projectiles.as_chunks::<2>().0 {
        assert!(Arc::ptr_eq(
            &colliders[pair[0].collider].shared_shape().0,
            &colliders[pair[1].collider].shared_shape().0
        ));
    }
    world
        .set_projectile_policy(ProjectilePolicy {
            restitution: 0.1,
            ..Default::default()
        })
        .unwrap();
    world
        .fire(
            0,
            Pose::default(),
            Shot {
                caliber: Caliber::Mm17,
                speed_m_s: 1.,
            },
            None,
        )
        .unwrap();
    for projectile in &world.projectiles[..4] {
        assert_eq!(
            world.physical.world.colliders[projectile.collider].restitution(),
            0.45
        );
    }
    assert_eq!(
        world.physical.world.colliders[world.projectiles[4].collider].restitution(),
        0.1
    );
    world
        .set_projectile_policy(ProjectilePolicy {
            retire_dwell_ns: 123,
            ..world.projectile_policy()
        })
        .unwrap();
    world
        .fire(
            0,
            Pose::default(),
            Shot {
                caliber: Caliber::Mm17,
                speed_m_s: 1.,
            },
            None,
        )
        .unwrap();
    assert!(Arc::ptr_eq(
        &world.physical.world.colliders[world.projectiles[4].collider]
            .shared_shape()
            .0,
        &world.physical.world.colliders[world.projectiles[5].collider]
            .shared_shape()
            .0,
    ));
}

#[test]
fn prepared_flight_ordered_contacts_retirement_caps_and_moving_targets_match_native() {
    let face = TargetFace {
        target: ArmorTarget::Outpost {
            outpost: 0,
            face: 0,
        },
        pose: Pose {
            translation_m: [2., 0., 0.2],
            rotation_wxyz: [0., 0., 0., 1.],
        },
    };
    let mut actual = WorldPhysics::new(&[face], -1.);
    let mut expected = oracle_world(&[face]);
    for index in 0..MAX_PROJECTILES + 2 {
        let shot = Shot {
            caliber: if index % 2 == 0 {
                Caliber::Mm17
            } else {
                Caliber::Mm42
            },
            speed_m_s: 25.,
        };
        let muzzle = Pose::at([0., index as f64 * 0.0001, 0.2]);
        assert_eq!(
            actual.fire(0, muzzle, shot, None),
            expected.oracle_fire(0, muzzle, shot, None)
        );
    }
    assert_native_equal(&actual, &expected);
    let mut frames = TargetFrames::new(vec![face]);
    let mut contact_count = 0;
    for tick in 0..600 {
        let mut start = face;
        start.pose.translation_m[1] = (tick as f64 * 0.01).sin() * 0.02;
        let mut end = face;
        end.pose.translation_m[1] = ((tick + 1) as f64 * 0.01).sin() * 0.02;
        frames.begin(|faces| faces[0] = start).unwrap();
        frames.update(|faces| faces[0] = end).unwrap();
        let time = tick * crate::tick_ns();
        let contacts = actual.step(time, &frames).unwrap();
        contact_count += contacts.len();
        assert_eq!(contacts, expected.step(time, &frames).unwrap());
        assert_native_equal(&actual, &expected);
    }
    assert!(contact_count > 0);
    assert!(actual.projectiles.is_empty());
    assert!(
        actual
            .removals()
            .any(|removal| removal.reason == RemovalReason::Retired)
    );
    assert!(
        actual
            .removals()
            .any(|removal| matches!(removal.reason, RemovalReason::Capped))
    );
}

#[test]
fn fixed_and_kinematic_explicit_mass_is_validated_without_dynamic_inertia_overlap() {
    use rm_sim_engine::model::{
        BodyKind, ColliderDescription, ColliderMass, ColliderPolicy, ContactMaterial, Inertial,
        ShapeDescription,
    };
    let collider = |mass, radius_m, translation_m| {
        ColliderDescription::new(
            ShapeDescription::Ball { radius_m },
            Pose::at(translation_m),
            ContactMaterial {
                friction: 0.4,
                restitution: 0.45,
                memberships: u32::MAX,
                filter: u32::MAX,
            },
        )
        .unwrap()
        .with_policy(ColliderPolicy {
            mass,
            ..Default::default()
        })
        .unwrap()
    };
    for kind in [BodyKind::Fixed, BodyKind::Kinematic] {
        for mass in [ColliderMass::MassKg(2.), ColliderMass::DensityKgM3(1.)] {
            let source =
                RigidBodyDescription::new(kind, vec![collider(mass, 0.1, [0.2, 0., 0.])]).unwrap();
            let cooked = CookedRigidBody::new(source).unwrap();
            assert!(cooked.aggregate_mass_properties().unwrap().mass() > 0.);
            assert_eq!(cooked.source().kind(), kind);
        }
        let source =
            RigidBodyDescription::new(kind, vec![collider(ColliderMass::Zero, 0.1, [0.; 3])])
                .unwrap();
        assert!(
            CookedRigidBody::new(source)
                .unwrap()
                .aggregate_mass_properties()
                .is_none()
        );
        let source = RigidBodyDescription::new(
            kind,
            vec![collider(ColliderMass::DensityKgM3(1.), 1e100, [0.; 3])],
        )
        .unwrap();
        assert!(CookedRigidBody::new(source).is_err());
        // A finite local COM and finite placement can still overflow in world
        // coordinates. Generic staged installation must reject this atomically.
        let source = RigidBodyDescription::new(
            kind,
            vec![collider(ColliderMass::MassKg(1.), 0.1, [1e308, 0., 0.])],
        )
        .unwrap();
        let cooked = CookedRigidBody::new(source).unwrap();
        let mut world = WorldPhysics::new(&[], 0.);
        assert!(
            world
                .install_body(
                    BodyKey(99),
                    &cooked,
                    BodyState::at(Pose::at([1e308, 0., 0.]))
                )
                .is_err()
        );
        assert!(world.body_binding(BodyKey(99)).is_none());
    }
    let kind = BodyKind::Dynamic(Inertial::new(1., [1.; 3], Pose::default()).unwrap());
    assert!(
        RigidBodyDescription::new(kind, vec![collider(ColliderMass::MassKg(1.), 0.1, [0.; 3])])
            .is_err()
    );
}

#[test]
fn preparation_preserves_rejected_fire_and_restore_admission_and_cap_order() {
    let mut actual = WorldPhysics::new(&[], -1.);
    let mut expected = oracle_world(&[]);
    let shot = Shot {
        caliber: Caliber::Mm17,
        speed_m_s: 25.,
    };
    let invalid_pose = Pose {
        rotation_wxyz: [0.; 4],
        ..Default::default()
    };
    for (pose, speed, shooter) in [
        (invalid_pose, f64::NAN, Some(999)),
        (Pose::default(), 25., Some(999)),
        (Pose::default(), 0., None),
        (Pose::default(), 41., None),
        (Pose::default(), f64::NAN, None),
    ] {
        let shot = Shot {
            speed_m_s: speed,
            ..shot
        };
        assert_eq!(
            actual.fire(0, pose, shot, shooter),
            expected.oracle_fire(0, pose, shot, shooter)
        );
        assert_native_equal(&actual, &expected);
    }
    for index in 0..MAX_PROJECTILES {
        assert_eq!(
            actual.fire(index as u64, Pose::default(), shot, None),
            expected.oracle_fire(index as u64, Pose::default(), shot, None)
        );
    }
    let state = actual.snapshot()[0].clone();
    assert_eq!(
        actual.restore_projectile(&state),
        expected.oracle_restore_projectile(&state)
    );
    let mut invalid_state = state;
    invalid_state.angular_velocity_rad_s[1] = f64::NAN;
    assert_eq!(
        actual.restore_projectile(&invalid_state),
        expected.oracle_restore_projectile(&invalid_state)
    );
    assert_native_equal(&actual, &expected);
    assert!(actual.removals().next().is_none());
    assert_eq!(
        actual.fire(100, Pose::default(), shot, None),
        expected.oracle_fire(100, Pose::default(), shot, None)
    );
    assert_native_equal(&actual, &expected);
    assert_eq!(actual.removals().next().unwrap().projectile, 0);
}
