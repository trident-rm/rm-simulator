// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
use super::*;
use crate::components::MechanismMesh;

fn tunnel() -> ComponentDefinition {
    Scenario::tunnel(TunnelModel {
        pose: Pose::default(),
        opening_m: [2., 1., 0.65],
        wall_thickness_m: 0.1,
    })
    .unwrap()
    .into()
}
fn instance(id: u64, source: ComponentDefinition) -> ComponentInstance {
    let targets = source.targets.iter().map(|t| t.face.target).collect();
    ComponentInstance {
        id: InstanceId(id),
        source,
        placement: Pose::default(),
        targets,
    }
}
fn rotation() -> Pose {
    Pose {
        translation_m: [3., -2., 1.],
        rotation_wxyz: math::quat_multiply(
            math::quat_axis_angle([0., 1., 0.], 0.7),
            math::quat_axis_angle([0., 0., 1.], -0.4),
        ),
    }
}
fn close(a: [f64; 3], b: [f64; 3]) {
    assert!(
        a.iter().zip(b).all(|(a, b)| (a - b).abs() < 1e-9),
        "{a:?} != {b:?}"
    );
}
fn mesh() -> CollisionMesh {
    CollisionMesh {
        vertices_m: vec![[2., 0., 0.], [2., 1., 0.], [2., 0., 1.]],
        triangles: vec![[0, 1, 2]],
    }
}
fn moving(kind: Mechanism, team: Team) -> ComponentDefinition {
    let face = TargetFace {
        target: ArmorTarget::Outpost {
            outpost: 0,
            face: 0,
        },
        pose: Pose::at([2., 0., 0.]),
    };
    let targets = if kind == Mechanism::Outpost {
        vec![ScenarioTarget {
            face,
            detector: Some(Detector {
                pose: face.pose,
                half_m: [0.1, 0.2],
            }),
            rotation: Some(TargetRotation {
                mechanism: kind,
                team,
                pivot_m: [1., 0., 0.],
                axis: [0., 0., 1.],
            }),
        }]
    } else {
        vec![]
    };
    let motion = if matches!(kind, Mechanism::Outpost | Mechanism::Rune) {
        MotionFrame::Rotation {
            pivot_m: [1., 0., 0.],
            axis: [0., 0., 1.],
        }
    } else {
        MotionFrame::Translation {
            offsets_m: [[0., 0., -0.2], [0., 0., 0.6]],
        }
    };
    ComponentDefinition::new(
        PlacedComponent {
            query_mesh: None,
            fixed: CollisionMesh::default(),
            native_shapes: vec![],
            moving: vec![MechanismMesh {
                mesh: mesh(),
                native_shapes: vec![],
                mechanism: kind,
                team,
                motion,
            }],
        },
        targets,
        Some(VisualInstance {
            file: "checked-source.glb".into(),
            placement: math::gltf_root_pose([1., 2., 3.], math::quat_axis_angle([1., 0., 0.], 0.2)),
        }),
    )
    .unwrap()
}

#[test]
fn cad_free_composition_identity_topology_reset_and_occupancy() {
    let mut terrain_world = WorldPhysics::new(&[], 0.);
    terrain_world.add_static_box(Pose::at([0., 0., -0.1]), [10., 10., 0.2]);
    let mut second = instance(2, tunnel());
    second.placement = Pose::yawed([4., 0., 0.], 0.5);
    let definition = Composition {
        components: vec![instance(1, tunnel()), second],
        terrain: vec![TerrainInstance {
            id: InstanceId(3),
            geometry: terrain_world.static_geometry_snapshot(true),
            placement: Pose::default(),
        }],
        robots: vec![RobotInstance {
            id: InstanceId(4),
            spawn: RobotSpawn {
                team: Team::Blue,
                config: ChassisConfig::default(),
                pose: Pose::at([-2., 0., 0.4]),
            },
        }],
        rfid: vec![RfidVolume {
            id: InstanceId(5),
            label: "caller tunnel card".into(),
            pose: rotation(),
            polygon_m: vec![[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]],
            z_range_m: [0., 0.5],
        }],
        catch_floor_m: None,
    }
    .prepare()
    .unwrap();
    assert_eq!(
        definition.robot_identities(),
        &[RobotIdentity {
            instance: InstanceId(4),
            physical: 0
        }]
    );
    assert!(definition.rfid()[0].contains(transform_point(rotation(), [0., 0., 0.25])));
    assert!(!definition.rfid()[0].contains(transform_point(rotation(), [0., 0., 1.])));
    assert!(definition.targets().is_empty());
    let mut a = definition
        .session(StepDuration::HZ_1000, Default::default())
        .unwrap();
    let b = definition
        .session(StepDuration::HZ_1000, Default::default())
        .unwrap();
    let initial = a.robot_truth(0).unwrap();
    let before = a
        .ground_truth()
        .static_geometry_snapshot(true)
        .into_triangles();
    a.step(50).unwrap();
    assert_eq!(b.stamp().time_ns, 0);
    a.reset().unwrap();
    assert_eq!(a.robot_truth(0).unwrap(), initial);
    assert_eq!(a.stamp().time_ns, 0);
    assert!(a.robot_truth(1).is_none());
    assert_eq!(
        before,
        a.ground_truth()
            .static_geometry_snapshot(true)
            .into_triangles()
    );
}

#[test]
fn arbitrary_rigid_placement_commutes_with_motion_and_preserves_visual_basis() {
    for kind in [Mechanism::Outpost, Mechanism::Base] {
        let source = moving(kind, Team::Red);
        let source_geometry = source.geometry.clone();
        let source_visual = source.visual.clone().unwrap();
        let source_targets = source.targets.clone();
        let mut part = instance(42, source);
        part.placement = rotation();
        if !part.targets.is_empty() {
            part.targets[0] = ArmorTarget::Outpost {
                outpost: 77,
                face: 0,
            };
        }
        let definition = Composition {
            components: vec![part],
            catch_floor_m: Some(-3.),
            ..Default::default()
        }
        .prepare()
        .unwrap();
        for angle in [0., 0.3, 1.] {
            let state = MechanismState {
                outpost_angle_rad: [angle, 0.],
                base_open_fraction: [angle, 0.],
                ..Default::default()
            };
            let (before, _) = source_geometry.at_mechanisms(&state).into_triangles();
            let (after, _) = definition
                .geometry()
                .at_motions(&definition.legacy_motions(&state).unwrap())
                .unwrap()
                .into_triangles();
            assert_eq!(before.len(), after.len());
            for point in before {
                let expected = transform_point(rotation(), point);
                assert!(
                    after
                        .iter()
                        .any(|p| p.iter().zip(expected).all(|(a, b)| (a - b).abs() < 1e-9))
                );
            }
            let mut faces = Vec::new();
            definition.targets_at(&state, &mut faces).unwrap();
            if let Some(target) = source_targets.first() {
                let r = target.rotation.unwrap();
                let expected = transform_pose(
                    rotation(),
                    OutpostModel::rotate_pose(target.face.pose, r.pivot_m, r.axis, angle),
                );
                close(faces[0].pose.translation_m, expected.translation_m);
                close(
                    math::rotate(faces[0].pose.rotation_wxyz, [1., 0., 0.]),
                    math::rotate(expected.rotation_wxyz, [1., 0., 0.]),
                );
                assert_eq!(
                    definition.source_target(faces[0].target).unwrap().source,
                    target.face.target
                );
                assert_eq!(
                    definition.targets()[0].detector.as_ref().unwrap().pose,
                    transform_pose(rotation(), target.face.pose)
                );
            }
            let mut session = definition.session(StepDuration::HZ_1000, state).unwrap();
            let initial = session
                .ground_truth()
                .static_geometry_snapshot(true)
                .into_triangles();
            session.physics_mut().sync_mechanisms(&Default::default());
            session.step(1).unwrap();
            session.reset().unwrap();
            assert_eq!(
                initial,
                session
                    .ground_truth()
                    .static_geometry_snapshot(true)
                    .into_triangles()
            );
        }
        let visual = &definition.visuals()[0].visual;
        let local = [0.4, 0.2, -0.3];
        close(
            math::gltf_point(&visual.placement, local),
            transform_point(
                rotation(),
                math::gltf_point(&source_visual.placement, local),
            ),
        );
    }
}

#[test]
fn duplicate_ids_invalid_pose_cardinality_and_target_aliasing_are_rejected() {
    let mut part = instance(1, tunnel());
    let basic = Composition {
        components: vec![part.clone()],
        ..Default::default()
    };
    let mut bad = basic.clone();
    bad.components.push(part.clone());
    assert!(bad.prepare().is_err());
    part.placement.rotation_wxyz = [2., 0., 0., 0.];
    assert!(
        Composition {
            components: vec![part],
            ..Default::default()
        }
        .prepare()
        .is_err()
    );
    let mut bad = basic.clone();
    bad.catch_floor_m = Some(f64::NAN);
    assert!(bad.prepare().is_err());
    let mut part = instance(1, moving(Mechanism::Outpost, Team::Red));
    part.targets.clear();
    assert!(
        Composition {
            components: vec![part],
            ..Default::default()
        }
        .prepare()
        .is_err()
    );
    let a = instance(1, moving(Mechanism::Outpost, Team::Red));
    let b = instance(2, moving(Mechanism::Outpost, Team::Blue));
    // Different channels, but both explicitly mapped to the same physical face.
    assert!(
        Composition {
            components: vec![a, b],
            ..Default::default()
        }
        .prepare()
        .is_err()
    );
    let mut part = instance(1, moving(Mechanism::Outpost, Team::Red));
    part.targets[0] = ArmorTarget::Base { base: 9, plate: 0 };
    assert!(
        Composition {
            components: vec![part],
            ..Default::default()
        }
        .prepare()
        .is_err()
    );
    let mut bad_robot = RobotSpawn {
        team: Team::Red,
        config: ChassisConfig::default(),
        pose: Pose::default(),
    };
    bad_robot.config.mass_kg = -1.;
    assert!(
        Composition {
            robots: vec![RobotInstance {
                id: InstanceId(1),
                spawn: bad_robot
            }],
            ..Default::default()
        }
        .prepare()
        .is_err()
    );
}

#[test]
fn repeated_source_channels_receive_independent_ids_and_reject_ambiguous_legacy_control() {
    for kind in [
        Mechanism::Outpost,
        Mechanism::Rune,
        Mechanism::Base,
        Mechanism::DartTarget,
    ] {
        let a = instance(1, moving(kind, Team::Red));
        let mut b = instance(
            2,
            moving(
                kind,
                if kind == Mechanism::DartTarget {
                    Team::Blue
                } else {
                    Team::Red
                },
            ),
        );
        if !b.targets.is_empty() {
            b.targets[0] = ArmorTarget::Outpost {
                outpost: 4,
                face: 0,
            };
        }
        let definition = Composition {
            components: vec![a, b],
            ..Default::default()
        }
        .prepare()
        .unwrap();
        assert_eq!(definition.motion_identities().len(), 2);
        let mappings = definition.motion_identities();
        assert_ne!(mappings[0].physical, mappings[1].physical);
        assert!(definition.legacy_motions(&Default::default()).is_err());
        let mut coordinates = definition.default_motions();
        coordinates[0].coordinate = 0.2;
        coordinates[1].coordinate = 0.8;
        let mut session = definition
            .session_with_motions(StepDuration::HZ_1000, &coordinates)
            .unwrap();
        let initial = session.ground_truth().static_geometry_snapshot(true);
        session
            .physics_mut()
            .sync_motions(&[MotionUpdate {
                id: coordinates[0].id,
                coordinate: 0.6,
            }])
            .unwrap();
        let changed = session.ground_truth().static_geometry_snapshot(true);
        assert_ne!(initial.clone().into_triangles(), changed.into_triangles());
        let mut faces = Vec::new();
        definition
            .targets_at_motions(&coordinates, &mut faces)
            .unwrap();
        if !faces.is_empty() {
            assert_ne!(faces[0].pose, faces[1].pose);
            let previous = faces.clone();
            for invalid in [
                vec![coordinates[0]; 2],
                vec![MotionUpdate {
                    id: MotionId {
                        instance: 999,
                        channel: 0,
                    },
                    coordinate: 0.,
                }],
                vec![
                    MotionUpdate {
                        coordinate: f64::NAN,
                        ..coordinates[0]
                    },
                    coordinates[1],
                ],
            ] {
                assert!(definition.targets_at_motions(&invalid, &mut faces).is_err());
                assert_eq!(faces, previous);
            }
        }
        session.step_with_targets(&faces).unwrap();
        session.reset().unwrap();
        assert_eq!(session.stamp().time_ns, 0);
        let reset = session.ground_truth().static_geometry_snapshot(true);
        assert_eq!(
            initial.motion_channels().collect::<Vec<_>>(),
            reset.motion_channels().collect::<Vec<_>>()
        );
        assert_eq!(initial.into_triangles(), reset.into_triangles());
    }
    let a = instance(1, moving(Mechanism::Outpost, Team::Red));
    let mut b = instance(2, moving(Mechanism::Outpost, Team::Blue));
    b.targets[0] = ArmorTarget::Outpost {
        outpost: 4,
        face: 0,
    };
    assert!(
        Composition {
            components: vec![a, b],
            ..Default::default()
        }
        .prepare()
        .is_ok()
    );
}

#[test]
fn invalid_state_leaves_target_output_unchanged_and_terrain_rejects_motion() {
    let source = moving(Mechanism::Outpost, Team::Red);
    let terrain = TerrainInstance {
        id: InstanceId(2),
        geometry: source.geometry.clone(),
        placement: Pose::default(),
    };
    assert!(
        Composition {
            terrain: vec![terrain],
            ..Default::default()
        }
        .prepare()
        .is_err()
    );
    let definition = Composition {
        components: vec![instance(1, source)],
        ..Default::default()
    }
    .prepare()
    .unwrap();
    let mut output = vec![definition.targets()[0].face];
    let before = output.clone();
    let state = MechanismState {
        outpost_angle_rad: [f64::NAN, 0.],
        ..Default::default()
    };
    assert!(definition.targets_at(&state, &mut output).is_err());
    assert_eq!(output, before);
    assert!(definition.session(StepDuration::HZ_1000, state).is_err());
}

#[test]
#[ignore = "requires the verified field-2027 CAD package"]
fn checked_source_outposts_compose_with_unique_targets_and_reset() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../field-2027");
    let cad = crate::cad_assets::load(&root).unwrap();
    let mut components = Vec::new();
    for index in 0..2 {
        let source = Scenario::load(&cad, Preset::Outpost(Team::Red)).unwrap();
        let mut part = instance(100 + index as u64, source.into());
        part.placement = rotation();
        part.placement.translation_m[1] += index as f64 * 4.;
        part.targets = part
            .source
            .targets
            .iter()
            .map(|t| match t.face.target {
                ArmorTarget::Outpost { face, .. } => ArmorTarget::Outpost {
                    outpost: 500 + index as u32,
                    face,
                },
                _ => unreachable!(),
            })
            .collect();
        components.push(part);
    }
    let definition = Composition {
        components,
        catch_floor_m: Some(-5.),
        ..Default::default()
    }
    .prepare()
    .unwrap();
    assert_eq!(definition.targets().len(), 6);
    assert_eq!(definition.visuals().len(), 2);
    assert_eq!(definition.target_identities().len(), 6);
    assert!(definition.rfid().is_empty());
    assert!(
        definition
            .motion_identities()
            .iter()
            .all(|m| m.team == Team::Red)
    );
    let mut coordinates = definition.default_motions();
    coordinates[0].coordinate = 0.4;
    coordinates[1].coordinate = -0.2;
    let mut session = definition
        .session_with_motions(StepDuration::HZ_1000, &coordinates)
        .unwrap();
    let mut targets = Vec::new();
    definition
        .targets_at_motions(&coordinates, &mut targets)
        .unwrap();
    session.step_with_targets(&targets).unwrap();
    session.reset().unwrap();
    assert_eq!(session.stamp().time_ns, 0);
    for mapping in definition.target_identities() {
        assert_eq!(definition.source_target(mapping.physical), Some(*mapping));
    }
    println!(
        "source composition: {} targets, {} visuals, {} prepared triangles",
        targets.len(),
        definition.visuals().len(),
        definition.geometry().clone().into_triangles().1.len()
    );
}

#[test]
fn rfid_metadata_rejects_bad_prisms_and_self_intersecting_polygons() {
    let volume = RfidVolume {
        id: InstanceId(8),
        label: "caller".into(),
        pose: Pose::default(),
        polygon_m: vec![[0., 0.], [2., 0.], [2., 2.], [0., 2.]],
        z_range_m: [0., 1.],
    };
    let mut bad = volume.clone();
    bad.z_range_m = [1., 0.];
    assert!(
        Composition {
            rfid: vec![bad],
            ..Default::default()
        }
        .prepare()
        .is_err()
    );
    let mut bad = volume.clone();
    bad.polygon_m = vec![[0., 0.], [2., 2.], [0., 3.], [2., 0.]];
    assert!(
        Composition {
            rfid: vec![bad],
            ..Default::default()
        }
        .prepare()
        .is_err()
    );
    let mut bad = volume.clone();
    bad.polygon_m.push(bad.polygon_m[0]);
    assert!(
        Composition {
            rfid: vec![bad],
            ..Default::default()
        }
        .prepare()
        .is_err()
    );
    let mut bad = volume.clone();
    bad.polygon_m[0][0] = f64::NAN;
    assert!(
        Composition {
            rfid: vec![bad],
            ..Default::default()
        }
        .prepare()
        .is_err()
    );
    assert!(
        Composition {
            rfid: vec![volume],
            ..Default::default()
        }
        .prepare()
        .is_ok()
    );
}

#[test]
fn raw_projectile_contacts_keep_explicit_physical_and_source_identity_maps() {
    let source_face = TargetFace {
        target: ArmorTarget::Outpost {
            outpost: 0,
            face: 0,
        },
        pose: Pose::at([0., 0., 1.]),
    };
    let source = ComponentDefinition::new(
        PlacedComponent {
            query_mesh: None,
            fixed: CollisionMesh::default(),
            native_shapes: vec![],
            moving: vec![],
        },
        vec![ScenarioTarget {
            face: source_face,
            detector: Some(Detector {
                pose: source_face.pose,
                half_m: [0.1, 0.1],
            }),
            rotation: None,
        }],
        None,
    )
    .unwrap();
    let components = [(10, 101, 0.), (20, 102, 2.)]
        .into_iter()
        .map(|(id, parent, y)| ComponentInstance {
            id: InstanceId(id),
            source: source.clone(),
            placement: Pose::at([0., y, 0.]),
            targets: vec![ArmorTarget::Outpost {
                outpost: parent,
                face: 0,
            }],
        })
        .collect();
    let definition = Composition {
        components,
        catch_floor_m: Some(-5.),
        ..Default::default()
    }
    .prepare()
    .unwrap();
    let mut session = definition
        .session(StepDuration::HZ_1000, Default::default())
        .unwrap();
    for y in [0., 2.] {
        session
            .physics_mut()
            .fire(
                0,
                Pose::yawed([1., y, 1.], std::f64::consts::PI),
                rm_sim_physics::Shot::at_limit(rm_sim_physics::Caliber::Mm17),
                None,
            )
            .unwrap();
    }
    let contacts = session.step(80).unwrap();
    for (instance, physical) in [(10, 101), (20, 102)] {
        let target = ArmorTarget::Outpost {
            outpost: physical,
            face: 0,
        };
        assert!(
            contacts.iter().any(|c| c.target == target),
            "missing contact for {target:?}"
        );
        assert_eq!(
            definition.source_target(target),
            Some(TargetIdentity {
                instance: InstanceId(instance),
                source: source_face.target,
                physical: target,
            })
        );
    }
    session.reset().unwrap();
    assert!(session.step(80).unwrap().is_empty());
}

#[test]
fn motion_ids_are_structural_under_reorder_and_disjoint_instance_namespaces() {
    let a = instance(10, moving(Mechanism::Rune, Team::Red));
    let b = instance(20, moving(Mechanism::Rune, Team::Red));
    let prepare = |components| {
        Composition {
            components,
            ..Default::default()
        }
        .prepare()
        .unwrap()
    };
    let forward = prepare(vec![a.clone(), b.clone()]);
    let backward = prepare(vec![b, a]);
    for mapping in forward.motion_identities() {
        assert!(backward.motion_identities().contains(mapping));
    }
    let unrelated = prepare(vec![instance(30, moving(Mechanism::Rune, Team::Red))]);
    let mut world = WorldPhysics::new(&[], 0.);
    world.try_insert_geometry(forward.geometry()).unwrap();
    world.try_insert_geometry(unrelated.geometry()).unwrap();
    assert!(world.try_insert_geometry(backward.geometry()).is_err());
    assert_eq!(
        world
            .static_geometry_snapshot(true)
            .motion_channels()
            .count(),
        3
    );
}

#[test]
#[ignore = "requires the verified field-2027 CAD package"]
fn checked_source_runes_have_independent_banks_and_reset() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../field-2027");
    let cad = crate::cad_assets::load(&root).unwrap();
    let source: ComponentDefinition = Scenario::load(&cad, Preset::Rune).unwrap().into();
    let components = (0..2)
        .map(|index| {
            let mut component = instance(200 + index, source.clone());
            component.placement = Pose::at([index as f64 * 8., 0., 0.]);
            component.targets = component
                .source
                .targets
                .iter()
                .map(|t| match t.face.target {
                    ArmorTarget::Rune { rune, blade } => ArmorTarget::Rune {
                        rune: rune + index as u32 * 10,
                        blade,
                    },
                    _ => unreachable!(),
                })
                .collect();
            component
        })
        .collect();
    let definition = Composition {
        components,
        ..Default::default()
    }
    .prepare()
    .unwrap();
    assert_eq!(definition.motion_identities().len(), 4);
    assert_eq!(definition.targets().len(), 20);
    assert!(definition.legacy_motions(&Default::default()).is_err());
    let mut coordinates = definition.default_motions();
    for (index, coordinate) in coordinates.iter_mut().enumerate() {
        coordinate.coordinate = (index as f64 + 1.) * 0.3;
    }
    let mut session = definition
        .session_with_motions(StepDuration::HZ_1000, &coordinates)
        .unwrap();
    let initial = session.ground_truth().static_geometry_snapshot(true);
    let mut targets = Vec::new();
    definition
        .targets_at_motions(&coordinates, &mut targets)
        .unwrap();
    session
        .physics_mut()
        .sync_motions(&[MotionUpdate {
            id: coordinates[0].id,
            coordinate: -0.7,
        }])
        .unwrap();
    session.step_with_targets(&targets).unwrap();
    session.reset().unwrap();
    let reset = session.ground_truth().static_geometry_snapshot(true);
    assert_eq!(
        initial.motion_channels().collect::<Vec<_>>(),
        reset.motion_channels().collect::<Vec<_>>()
    );
    assert_eq!(initial.into_triangles(), reset.into_triangles());
}

#[test]
fn native_cylinder_is_analytic_after_composition_and_reset() {
    let shape = (
        Pose::at([2., 0., 1.]),
        rm_sim_physics::model::ShapeDescription::Cylinder {
            radius_m: 0.2,
            half_height_m: 0.5,
        },
    );
    let component = ComponentDefinition::new(
        PlacedComponent {
            query_mesh: None,
            fixed: CollisionMesh::default(),
            native_shapes: vec![shape],
            moving: vec![],
        },
        vec![],
        None,
    )
    .unwrap();
    let definition = Composition {
        components: vec![ComponentInstance {
            id: InstanceId(14),
            source: component,
            placement: Pose::at([3., 0., 0.]),
            targets: vec![],
        }],
        ..Default::default()
    }
    .prepare()
    .unwrap();
    let mut session = definition
        .session(StepDuration::HZ_1000, Default::default())
        .unwrap();
    let angle = std::f64::consts::PI / 64.;
    let radial = [angle.cos(), 0., angle.sin()];
    let origin = [5. + radial[0] * 0.3, 0., 1. + radial[2] * 0.3];
    for _ in 0..2 {
        let capture = session.ground_truth().static_geometry_snapshot(true);
        let hit = capture
            .cast_ray(origin, radial.map(|v| -v), 0.2)
            .unwrap()
            .unwrap();
        assert!(
            (hit.distance_m - 0.1).abs() < 1e-9,
            "composition tessellated the cylinder"
        );
        session.reset().unwrap();
    }
}
