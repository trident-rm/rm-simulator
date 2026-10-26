// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native scenery descriptions, grouped without changing mesh collision policy.
use super::*;
use sim_engine::model::{
    BodyKind, ColliderDescription, CompoundShape, ContactMaterial, RigidBodyDescription,
    ShapeDescription,
};

fn prepare_shapes(shapes: &[(Pose, ShapeDescription)]) -> Result<Vec<SharedShape>, &'static str> {
    // Source compounds allow 256 expanded nodes, including their root. Prepare
    // every batch before inserting any collider so malformed input is atomic.
    let material = ContactMaterial {
        friction: FRICTION,
        restitution: RESTITUTION,
        memberships: u32::MAX,
        filter: u32::MAX,
    };
    shapes
        .chunks(255)
        .map(|parts| {
            if parts.iter().any(|(_, s)| {
                matches!(
                    s,
                    ShapeDescription::TriangleMesh(_) | ShapeDescription::Compound(_)
                )
            }) {
                return Err("native scenery requires primitive or convex children");
            }
            let shape = ShapeDescription::Compound(CompoundShape::new(parts.to_vec())?);
            let source = RigidBodyDescription::new(
                BodyKind::Fixed,
                vec![ColliderDescription::new(shape, Pose::default(), material)?],
            )?;
            let cooked = crate::cooking::CookedRigidBody::new(source)?;
            Ok(cooked.shapes()[0].clone())
        })
        .collect()
}

fn check_motion(frame: crate::motion::MotionFrame) -> Result<(), &'static str> {
    match frame {
        crate::motion::MotionFrame::Translation { offsets_m }
            if !offsets_m.iter().all(|v| finite(*v)) =>
        {
            Err("nonfinite mechanism offset")
        }
        crate::motion::MotionFrame::Rotation { pivot_m, axis }
            if !pivot_m.iter().chain(&axis).all(|v| v.is_finite())
                || (axis.iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-6 =>
        {
            Err("invalid rotor pivot/axis")
        }
        _ => Ok(()),
    }
}

impl WorldPhysics {
    /// Install native primitive/convex scenery in world FLU frames, retaining
    /// analytic shapes for contacts, rays and sweeps. Child poses use wxyz.
    /// Up to 255 children share each compound collider; mesh/nested compound
    /// children are rejected. An empty list does nothing. All batches are cooked
    /// before mutation; contact material/filter match ordinary static CAD meshes.
    ///
    /// ```
    /// use rm_sim_physics::{Pose, WorldPhysics, model::ShapeDescription};
    /// let mut world = WorldPhysics::new(&[], 0.);
    /// world.add_static_shapes(&[(Pose::at([2.,0.,1.]),
    ///     ShapeDescription::Cylinder { radius_m: 0.2, half_height_m: 0.5 })])?;
    /// let hit = world.static_geometry_snapshot(true)
    ///     .cast_ray([0.,0.,1.], [1.,0.,0.], 3.)?.unwrap();
    /// assert!((hit.distance_m - 1.8).abs() < 1e-9);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn add_static_shapes(
        &mut self,
        shapes: &[(Pose, ShapeDescription)],
    ) -> Result<(), &'static str> {
        for shape in prepare_shapes(shapes)? {
            self.insert_static_mesh_shape(shape);
        }
        Ok(())
    }

    /// Install fixed native physics and a separate query-only mesh atomically.
    /// The mesh has a zero contact filter. Captures retain both representations;
    /// LiDAR selects the mesh, while contacts and clearance select native shapes.
    pub fn add_static_shapes_with_queries(
        &mut self,
        shapes: &[(Pose, ShapeDescription)],
        vertices_m: Vec<[f64; 3]>,
        triangles: Vec<[u32; 3]>,
    ) -> Result<(), &'static str> {
        let native = prepare_shapes(shapes)?;
        let query = crate::cooking::cook_buffers(vertices_m, triangles)?;
        for shape in native {
            self.physical.extension_insert_scenery(
                shape,
                Pose3::IDENTITY,
                FRICTION,
                RESTITUTION,
                crate::collision_profiles::physical_groups(),
            );
        }
        self.physical.extension_insert_scenery(
            query,
            Pose3::IDENTITY,
            FRICTION,
            RESTITUTION,
            crate::collision_profiles::query_groups(),
        );
        Ok(())
    }

    /// Install native scenery at its world-FLU rest frames on a legacy prescribed
    /// mechanism. Motion is applied to the whole compound from the same rest
    /// coordinates as mesh channels; source joint ownership remains with callers.
    pub fn add_mechanism_shapes(
        &mut self,
        shapes: &[(Pose, ShapeDescription)],
        mechanism: crate::motion::Mechanism,
        team: Team,
        frame: crate::motion::MotionFrame,
    ) -> Result<(), &'static str> {
        check_motion(frame)?;
        let rotating = matches!(frame, crate::motion::MotionFrame::Rotation { .. });
        if rotating
            != matches!(
                mechanism,
                crate::motion::Mechanism::Outpost | crate::motion::Mechanism::Rune
            )
        {
            return Err("mechanism shape motion and driver disagree");
        }
        for shape in prepare_shapes(shapes)? {
            let handle = self.insert_static_mesh_shape(shape);
            self.mechanisms.push((
                handle,
                crate::motion::MotionChannel::Legacy {
                    kind: mechanism,
                    team,
                },
                frame,
            ));
        }
        Ok(())
    }

    /// Append already cooked finite fixed geometry under one FLU rigid transform.
    /// Native shapes and collision groups are retained rather than tessellated.
    /// Catch floors, bounds and source moving channels are not imported.
    pub fn add_fixed_geometry(
        &mut self,
        geometry: &StaticGeometry,
        placement: Pose,
    ) -> Result<(), &'static str> {
        if !crate::runtime::math::pose_is_valid(placement) {
            return Err("invalid geometry placement");
        }
        if !geometry.mechanisms.is_empty() {
            return Err("fixed geometry contains moving channels");
        }
        let transform = pose(placement);
        for (index, (source, shape)) in geometry.shapes.iter().enumerate() {
            let groups = geometry
                .collision_groups
                .get(index)
                .copied()
                .unwrap_or_default();
            self.physical.extension_insert_scenery(
                shape.clone(),
                transform * *source,
                FRICTION,
                RESTITUTION,
                groups,
            );
        }
        Ok(())
    }

    /// Append an addressed moving source capture under one FLU placement.
    /// Native primitive compounds remain native; mesh vertices are transformed
    /// only for the existing mesh channel contract. Rest frames are baked once,
    /// then the explicit motion frame supplies caller-controlled displacement.
    /// Heightfields remain fixed scenery and are rejected before any insertion.
    pub fn add_motion_geometry(
        &mut self,
        geometry: &StaticGeometry,
        placement: Pose,
        id: crate::motion::MotionId,
        frame: crate::motion::MotionFrame,
    ) -> Result<(), &'static str> {
        use crate::motion::MotionChannel;
        check_motion(frame)?;
        if !crate::runtime::math::pose_is_valid(placement) {
            return Err("invalid geometry placement");
        }
        if self.mechanisms.iter().any(|(_, channel, old)| {
            *channel == MotionChannel::Instance(id)
                && std::mem::discriminant(old) != std::mem::discriminant(&frame)
        }) {
            return Err("motion channel mixes rotation and translation units");
        }
        let transform = pose(placement);
        let mut ready = Vec::new();
        for (index, (source, shape)) in geometry.shapes.iter().enumerate() {
            let mounted = transform * *source;
            let baked = match shape.as_typed_shape() {
                TypedShape::HeightField(_) => {
                    return Err("heightfields require fixed scenery installation");
                }
                TypedShape::TriMesh(mesh) => crate::cooking::cook_buffers(
                    mesh.vertices()
                        .iter()
                        .map(|v| mounted.transform_point(*v).to_array())
                        .collect(),
                    mesh.indices().to_vec(),
                )?,
                TypedShape::Compound(compound) => SharedShape::compound(
                    compound
                        .shapes()
                        .iter()
                        .map(|(child, shape)| (mounted * *child, shape.clone()))
                        .collect(),
                ),
                _ => SharedShape::compound(vec![(mounted, shape.clone())]),
            };
            ready.push((
                baked,
                geometry
                    .collision_groups
                    .get(index)
                    .copied()
                    .unwrap_or_default(),
            ));
        }
        for (shape, groups) in ready {
            let handle = self.physical.extension_insert_scenery(
                shape,
                Pose3::IDENTITY,
                FRICTION,
                RESTITUTION,
                groups,
            );
            self.mechanisms
                .push((handle, MotionChannel::Instance(id), frame));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn cylinder() -> (Pose, ShapeDescription) {
        (
            Pose::at([2., 0., 1.]),
            ShapeDescription::Cylinder {
                radius_m: 0.2,
                half_height_m: 0.5,
            },
        )
    }
    #[test]
    fn native_scenery_captures_and_restores_without_tessellation() {
        let mut world = WorldPhysics::new(&[], 0.);
        world.add_static_shapes(&[cylinder()]).unwrap();
        let scene = world.static_geometry_snapshot(true);
        assert_eq!(scene.shapes.len(), 1);
        let compound = scene.shapes[0].1.as_compound().unwrap();
        assert!(compound.shapes()[0].1.as_cylinder().is_some());
        let hit = scene
            .cast_ray([0., 0., 1.], [1., 0., 0.], 3.)
            .unwrap()
            .unwrap();
        assert!((hit.distance_m - 1.8).abs() < 1e-9);
        let mut restored = WorldPhysics::new(&[], 0.);
        restored.insert_geometry(&scene);
        assert!(
            (restored
                .static_geometry_snapshot(true)
                .cast_ray([0., 0., 1.], [1., 0., 0.], 3.)
                .unwrap()
                .unwrap()
                .distance_m
                - 1.8)
                .abs()
                < 1e-9
        );
        assert!(!scene.into_triangles().1.is_empty());
    }
    #[test]
    fn invalid_native_batches_do_not_install_an_earlier_chunk() {
        let mut world = WorldPhysics::new(&[], 0.);
        let mut shapes = vec![cylinder(); 256];
        shapes[255].1 = ShapeDescription::Cylinder {
            radius_m: f64::NAN,
            half_height_m: 1.,
        };
        assert!(world.add_static_shapes(&shapes).is_err());
        assert!(world.static_geometry_snapshot(true).shapes.is_empty());
    }
    #[test]
    fn addressed_native_motion_and_instance_placement_remain_native() {
        let mut source = WorldPhysics::new(&[], 0.);
        source.add_static_shapes(&[cylinder()]).unwrap();
        let geometry = source.static_geometry_snapshot(true);
        let mut world = WorldPhysics::new(&[], 0.);
        let id = crate::motion::MotionId {
            instance: 9,
            channel: 1,
        };
        world
            .add_motion_geometry(
                &geometry,
                Pose::at([3., 0., 0.]),
                id,
                crate::motion::MotionFrame::Translation {
                    offsets_m: [[0.; 3], [1., 0., 0.]],
                },
            )
            .unwrap();
        world
            .sync_motions(&[crate::motion::MotionUpdate {
                id,
                coordinate: 0.5,
            }])
            .unwrap();
        let capture = world.static_geometry_snapshot(true);
        let hit = capture
            .cast_ray([0., 0., 1.], [1., 0., 0.], 8.)
            .unwrap()
            .unwrap();
        assert!((hit.distance_m - 5.3).abs() < 1e-9);
        assert!(
            capture.shapes[0].1.as_compound().unwrap().shapes()[0]
                .1
                .as_cylinder()
                .is_some()
        );
        assert!(world.static_geometry_snapshot(false).shapes.is_empty());
    }
}

#[cfg(test)]
mod projectile_tests {
    use super::*;
    #[test]
    fn native_cylinder_stops_ccd_projectiles() {
        let mut world = WorldPhysics::new(&[], 0.);
        world
            .add_static_shapes(&[(
                Pose::at([2., 0., 1.]),
                ShapeDescription::Cylinder {
                    radius_m: 0.2,
                    half_height_m: 1.,
                },
            )])
            .unwrap();
        world
            .fire(
                0,
                Pose::at([0., 0., 1.]),
                Shot {
                    caliber: Caliber::Mm17,
                    speed_m_s: 20.,
                },
                None,
            )
            .unwrap();
        let frames = TargetFrames::new(vec![]);
        let mut rebound = false;
        for tick in 0..32 {
            world.step(tick * crate::tick_ns(), &frames).unwrap();
            if let Some(ball) = world.snapshot().first() {
                assert!(
                    ball.position_m[0] < 1.81,
                    "projectile passed through native cylinder"
                );
                rebound |= ball.velocity_m_s[0] < 0.;
            }
        }
        assert!(rebound);
    }
}

#[cfg(test)]
mod capture_tests {
    use super::*;
    #[test]
    fn captured_heightfields_remain_native_when_fixed_and_reject_moving_import() {
        let grid = sim_engine::model::HeightField::new(vec![vec![0.; 2]; 2], 2., 2.).unwrap();
        let model = crate::cooking::CookedRigidBody::new(
            RigidBodyDescription::new(
                BodyKind::Fixed,
                vec![
                    ColliderDescription::new(
                        ShapeDescription::HeightField(grid),
                        Pose::default(),
                        ContactMaterial {
                            friction: 0.4,
                            restitution: 0.0,
                            memberships: 3,
                            filter: 7,
                        },
                    )
                    .unwrap(),
                ],
            )
            .unwrap(),
        )
        .unwrap();
        let mut source = WorldPhysics::new(&[], -10.);
        source.add_fixed_model(&model, Pose::default()).unwrap();
        let capture = source.static_geometry_snapshot(true);
        let mut fixed = WorldPhysics::new(&[], -10.);
        fixed
            .add_fixed_geometry(&capture, Pose::at([2., 0., 0.]))
            .unwrap();
        let imported = fixed.static_geometry_snapshot(true);
        assert!(imported.shapes[0].1.as_heightfield().is_some());
        assert_eq!(capture.collision_groups, imported.collision_groups);
        assert!(
            imported
                .cast_ray([2., 1., 0.], [0., -1., 0.], 2.)
                .unwrap()
                .is_some()
        );
        let mut moving = WorldPhysics::new(&[], -10.);
        assert!(
            moving
                .add_motion_geometry(
                    &capture,
                    Pose::default(),
                    crate::motion::MotionId {
                        instance: 1,
                        channel: 1
                    },
                    crate::motion::MotionFrame::Translation {
                        offsets_m: [[0.; 3], [1., 0., 0.]]
                    }
                )
                .is_err()
        );
        assert!(moving.static_geometry_snapshot(true).shapes.is_empty());
    }

    #[test]
    fn moving_native_capture_roundtrip_preserves_contact_groups() {
        let mut world = WorldPhysics::new(&[], 0.);
        let shape = (
            Pose::at([2., 0., 1.]),
            ShapeDescription::Cylinder {
                radius_m: 0.2,
                half_height_m: 0.5,
            },
        );
        let frame = crate::motion::MotionFrame::Translation {
            offsets_m: [[0.; 3], [1., 0., 0.]],
        };
        world
            .add_mechanism_shapes(&[shape], crate::motion::Mechanism::Base, Team::Red, frame)
            .unwrap();
        let groups = InteractionGroups::new(
            Group::GROUP_3,
            Group::ALL & !Group::GROUP_2,
            InteractionTestMode::And,
        );
        let handle = world.mechanisms[0].0;
        world.physical.world_mut().colliders[handle].set_collision_groups(groups);
        let captured = world
            .static_geometry_snapshot(true)
            .motion_parts()
            .pop()
            .unwrap();
        assert_eq!(captured.geometry.collision_groups, vec![groups]);
        let mut imported = WorldPhysics::new(&[], 0.);
        imported
            .add_motion_geometry(
                &captured.geometry,
                Pose::default(),
                crate::motion::MotionId {
                    instance: 10,
                    channel: 2,
                },
                frame,
            )
            .unwrap();
        let handle = imported.mechanisms[0].0;
        assert_eq!(
            imported.physical.world().colliders[handle].collision_groups(),
            groups
        );
    }
    #[test]
    fn successful_native_batching_retains_every_moving_shape() {
        let mut world = WorldPhysics::new(&[], 0.);
        let shapes: Vec<_> = (0..256)
            .map(|i| {
                (
                    Pose::at([i as f64, 0., 1.]),
                    ShapeDescription::Cylinder {
                        radius_m: 0.2,
                        half_height_m: 0.5,
                    },
                )
            })
            .collect();
        let frame = crate::motion::MotionFrame::Translation {
            offsets_m: [[0.; 3], [0., 0., 1.]],
        };
        world
            .add_mechanism_shapes(&shapes, crate::motion::Mechanism::Base, Team::Red, frame)
            .unwrap();
        let captured = world.static_geometry_snapshot(true);
        assert_eq!(captured.shapes.len(), 2);
        assert_eq!(captured.motion_parts().len(), 2);
        assert_eq!(
            captured
                .shapes
                .iter()
                .map(|(_, s)| s.as_compound().unwrap().shapes().len())
                .sum::<usize>(),
            256
        );
    }
}

#[cfg(test)]
mod selective_query_tests {
    use super::*;
    #[test]
    fn query_mesh_does_not_stop_ccd_and_survives_capture_restore() {
        let mut world = WorldPhysics::new(&[], 0.);
        world.remove_catch_floor();
        world
            .add_static_shapes_with_queries(
                &[(
                    Pose::at([3., 0., 1.]),
                    ShapeDescription::Cylinder {
                        radius_m: 0.5,
                        half_height_m: 1.,
                    },
                )],
                vec![[1., -3., -2.], [1., 3., -2.], [1., 0., 4.]],
                vec![[0, 1, 2]],
            )
            .unwrap();
        let capture = world.static_geometry_snapshot(true);
        for scene in [&capture, &capture.fixed_only()] {
            assert!(
                (scene
                    .cast_ray([0., 0., 1.], [1., 0., 0.], 5.)
                    .unwrap()
                    .unwrap()
                    .distance_m
                    - 1.)
                    .abs()
                    < 1e-10
            );
            let rigid =
                scene.for_collision_purpose(crate::collision_profiles::CollisionPurpose::RigidBody);
            assert!(
                (rigid
                    .cast_ray([0., 0., 1.], [1., 0., 0.], 5.)
                    .unwrap()
                    .unwrap()
                    .distance_m
                    - 2.5)
                    .abs()
                    < 1e-8
            );
            assert_eq!(
                scene.clone().into_triangles().1.len(),
                124,
                "query surface leaked into physics wireframe"
            );
        }
        let mut restored = WorldPhysics::new(&[], 0.);
        restored.try_insert_geometry(&capture).unwrap();
        let again = restored.static_geometry_snapshot(true);
        assert_eq!(capture.collision_groups, again.collision_groups);
        assert_eq!(
            capture.cast_ray([0., 0., 1.], [1., 0., 0.], 5.).unwrap(),
            again.cast_ray([0., 0., 1.], [1., 0., 0.], 5.).unwrap()
        );
        restored
            .fire(
                0,
                Pose::at([0., 0., 1.]),
                Shot {
                    caliber: Caliber::Mm17,
                    speed_m_s: 20.,
                },
                None,
            )
            .unwrap();
        let frames = TargetFrames::new(vec![]);
        let mut passed_query = false;
        let mut rebound = false;
        for tick in 0..32 {
            restored.step(tick * crate::tick_ns(), &frames).unwrap();
            if let Some(ball) = restored.snapshot().first() {
                passed_query |= ball.position_m[0] > 1.5;
                rebound |= ball.velocity_m_s[0] < 0.;
                assert!(ball.position_m[0] < 2.6, "CCD ball crossed native cylinder");
            }
        }
        assert!(
            passed_query && rebound,
            "query mesh added a contact or native collision was lost"
        );
    }
}
