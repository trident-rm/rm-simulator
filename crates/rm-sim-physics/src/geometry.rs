// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared collision geometry, moving scenery and read-only clearance queries.
use crate::{
    Team,
    projectile::{pose, vector},
};
use rapier3d_f64::prelude::*;

/// An owned snapshot of physics shapes, excluding the catch half-space.
/// Mesh storage is shared with physics; capturing this does not copy triangles.
/// Triangle expansion can run after releasing the field, on a caller-owned worker.
///
/// ```
/// use rm_sim_physics::{Pose, WorldPhysics};
///
/// let mut physics = WorldPhysics::new(&[], 0.0);
/// physics.add_static_box(Pose::at([2.0, 0.0, 1.0]), [1.0, 1.0, 1.0]);
///
/// let geometry = physics.static_geometry_snapshot(true);
/// // The box blocks the sphere and the sight line through its centre.
/// assert!(!geometry.sphere_clear([2.0, 0.0, 1.0], 0.2));
/// assert!(!geometry.segment_clear([0.0, 0.0, 1.0], [4.0, 0.0, 1.0]));
/// // Both clear a path well beside it.
/// assert!(geometry.sphere_clear([2.0, 3.0, 1.0], 0.2));
/// assert!(geometry.segment_clear([0.0, 3.0, 1.0], [4.0, 3.0, 1.0]));
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Default)]
pub struct StaticGeometry {
    /// Explicit query profile; default rays use LiDAR and clearance uses rigid geometry.
    pub(crate) query_purpose: Option<crate::collision_profiles::CollisionPurpose>,
    /// True when the capture was taken from a world whose catch half-space had
    /// already been removed.
    pub(crate) no_catch_floor: bool,
    /// Convex XY slab outside which projectiles are absorbed, copied from the
    /// captured world when it declares one.
    pub(crate) projectile_bounds_m: Option<[[f64; 3]; 2]>,
    /// Per-shape AABBs, built on the first clearance query and dropped whenever
    /// a transform changes. Shared so a clone does not repeat the work.
    pub(crate) bounds: std::sync::OnceLock<std::sync::Arc<[Aabb]>>,
    /// Captured collider transforms and their shared shapes, in capture order.
    pub(crate) shapes: Vec<(rapier3d_f64::prelude::Pose, SharedShape)>,
    /// Interaction groups for [`Self::shapes`], index for index.
    pub(crate) collision_groups: Vec<InteractionGroups>,
    /// Articulated parts as shape index, role, team and typed source motion frame.
    pub(crate) endpoint_bakes: Vec<crate::projectile::endpoint_bakes::CapturedBake>,
    pub(crate) mechanisms: Vec<(
        usize,
        crate::motion::MotionChannel,
        crate::motion::MotionFrame,
    )>,
}
/// One caller-driven CAD collider with a typed motion frame. Source placement
/// is baked into `geometry`; debug renderers retain the shared rest-frame mesh.
pub struct MovingGeometry {
    /// Which mechanism moves this part.
    pub kind: crate::motion::Mechanism,
    /// Team whose [`crate::motion::MechanismState`] entry drives it.
    pub team: Team,
    /// Rest frame and prescribed translation endpoints or source rotor axis.
    pub motion: crate::motion::MotionFrame,
    /// The part's own shapes, held at the origin of its untranslated frame.
    pub geometry: StaticGeometry,
}

/// A physical moving shape with its explicit or compatibility driver.
/// Rest-frame shape buffers are shared; current coordinates live in captures/worlds.
pub struct AddressedMovingGeometry {
    /// Driver identity; instance channels are never driven by legacy team state.
    pub channel: crate::motion::MotionChannel,
    /// Prescribed frame in world FLU coordinates.
    pub motion: crate::motion::MotionFrame,
    /// Shared geometry in the frame before applying the prescribed transform.
    pub geometry: StaticGeometry,
}

/// Compatibility path for backend-independent query records.
pub use crate::queries::{RayHit, RayQuery};

impl StaticGeometry {
    fn visible(&self, index: usize, default: crate::collision_profiles::CollisionPurpose) -> bool {
        crate::collision_profiles::visible_to(
            self.collision_groups
                .get(index)
                .copied()
                .unwrap_or_default(),
            self.query_purpose.unwrap_or(default),
        )
    }

    /// Return the nearest hit up to an inclusive metric range, or a valid miss.
    /// Direction is normalized internally; its scale does not change the range.
    /// Invalid/non-finite inputs return an error, unlike a valid `Ok(None)` miss.
    /// Meshes use their exact captured triangles and cached BVHs, not convex hulls.
    /// Solid shapes containing the origin occlude at zero range. Equal-range
    /// hits retain the first capture-local shape. This never advances physics.
    ///
    /// ```
    /// use rm_sim_physics::{WorldPhysics, Pose};
    /// let mut world = WorldPhysics::new(&[], 0.0);
    /// world.add_static_box(Pose::at([2., 0., 1.]), [1.; 3]);
    /// let geometry = world.static_geometry_snapshot(true);
    /// let hit = geometry.cast_ray([0., 0., 1.], [7., 0., 0.], 3.)?.unwrap();
    /// assert!((hit.distance_m - 1.5).abs() < 1e-9);
    /// assert_eq!(hit.normal, [-1., 0., 0.]);
    /// assert!(geometry.cast_ray([0., 0., 1.], [0.; 3], 3.).is_err());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn cast_ray(
        &self,
        origin_m: [f64; 3],
        direction: [f64; 3],
        max_distance_m: f64,
    ) -> Result<Option<RayHit>, &'static str> {
        let query = RayQuery::new(origin_m, direction, max_distance_m)?;
        Ok(self.cast_metric_ray(
            &Ray::new(vector(query.origin_m()), vector(query.direction())),
            query.max_distance_m(),
        ))
    }

    /// Query a prepared batch into exact-size caller-owned output storage.
    /// Requests and results have matching order, including explicit `None` misses.
    /// Length mismatch leaves every prior output unchanged; empty batches are valid.
    /// Validated requests can be reused across captures without normalizing again.
    /// No output allocation or world snapshot is performed. The capture may lazily
    /// allocate its shared bounds cache on first use. Geometry stays immutable;
    /// result shape ordinals refer only to this capture, not durable entity IDs.
    ///
    /// ```
    /// use rm_sim_physics::{Pose, WorldPhysics, queries::RayQuery};
    /// let mut world = WorldPhysics::new(&[], 0.);
    /// world.add_static_box(Pose::at([2., 0., 1.]), [1.; 3]);
    /// let scene = world.static_geometry_snapshot(true);
    /// let rays = [RayQuery::new([0., 0., 1.], [1., 0., 0.], 10.)?];
    /// let mut hits = [None];
    /// scene.cast_rays_into(&rays, &mut hits)?;
    /// assert_eq!(hits[0].unwrap().distance_m, 1.5);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn cast_rays_into(
        &self,
        rays: &[RayQuery],
        hits: &mut [Option<RayHit>],
    ) -> Result<(), &'static str> {
        if rays.len() != hits.len() {
            return Err("ray batch output length must match request length");
        }
        for (query, hit) in rays.iter().zip(hits) {
            *hit = self.cast_metric_ray(
                &Ray::new(vector(query.origin_m()), vector(query.direction())),
                query.max_distance_m(),
            );
        }
        Ok(())
    }

    /// Internal scan path for already validated metric rays.
    pub(crate) fn cast_metric_ray(&self, ray: &Ray, max_distance_m: f64) -> Option<RayHit> {
        let bounds = self.bounds.get_or_init(|| {
            self.shapes
                .iter()
                .map(|(pose, shape)| shape.compute_aabb(pose))
                .collect()
        });
        let mut closest: Option<RayHit> = None;
        let mut limit = max_distance_m;
        for (index, ((pose, shape), bounds)) in self.shapes.iter().zip(bounds.iter()).enumerate() {
            if !self.visible(index, crate::collision_profiles::CollisionPurpose::Lidar)
                || bounds.cast_local_ray(ray, limit, true).is_none()
            {
                continue;
            }
            if let Some(hit) = shape.cast_ray_and_get_normal(pose, ray, limit, true)
                && (closest.is_none() || hit.time_of_impact < limit)
            {
                limit = hit.time_of_impact;
                closest = Some(RayHit {
                    distance_m: limit,
                    position_m: ray.point_at(limit).to_array(),
                    normal: hit.normal.to_array(),
                    shape_index: index,
                });
            }
        }
        closest
    }
    /// Separate articulated scenery from fixed geometry for debug drawing.
    pub fn fixed_only(&self) -> Self {
        Self {
            query_purpose: self.query_purpose,
            no_catch_floor: self.no_catch_floor,
            projectile_bounds_m: self.projectile_bounds_m,
            bounds: Default::default(),
            collision_groups: self
                .collision_groups
                .iter()
                .enumerate()
                .filter(|(index, _)| !self.mechanisms.iter().any(|(i, ..)| i == index))
                .map(|(_, groups)| *groups)
                .collect(),
            shapes: self
                .shapes
                .iter()
                .enumerate()
                .filter(|(index, _)| !self.mechanisms.iter().any(|(i, ..)| i == index))
                .map(|(_, shape)| shape.clone())
                .collect(),
            endpoint_bakes: Vec::new(),
            mechanisms: Vec::new(),
        }
    }

    /// Compatibility projection of legacy-driven parts in their rest frames.
    /// Explicit instance parts are available through [`Self::motion_parts`].
    pub fn moving_parts(&self) -> Vec<MovingGeometry> {
        self.motion_parts()
            .into_iter()
            .filter_map(|part| {
                let crate::motion::MotionChannel::Legacy { kind, team } = part.channel else {
                    return None;
                };
                Some(MovingGeometry {
                    kind,
                    team,
                    motion: part.motion,
                    geometry: part.geometry,
                })
            })
            .collect()
    }

    /// Drivers in captured shape order, including repeated shapes of one channel.
    /// No shape storage is cloned; callers may deduplicate IDs for inspection.
    pub fn motion_channels(&self) -> impl Iterator<Item = crate::motion::MotionChannel> + '_ {
        self.mechanisms.iter().map(|(_, channel, _)| *channel)
    }

    /// All prescribed parts with their exact driver and unchanged source frames.
    /// This includes explicit instances and shares cooked shape buffers.
    pub fn motion_parts(&self) -> Vec<AddressedMovingGeometry> {
        self.mechanisms
            .iter()
            .map(|(index, channel, frame)| {
                let (mut position, shape) = self.shapes[*index].clone();
                position.translation = Vector::ZERO;
                if matches!(frame, crate::motion::MotionFrame::Rotation { .. }) {
                    position = Default::default();
                }
                AddressedMovingGeometry {
                    channel: *channel,
                    motion: *frame,
                    geometry: Self {
                        query_purpose: self.query_purpose,
                        shapes: vec![(position, shape)],
                        collision_groups: vec![
                            self.collision_groups
                                .get(*index)
                                .copied()
                                .unwrap_or_default(),
                        ],
                        ..Self::default()
                    },
                }
            })
            .collect()
    }

    /// Conservative clearance for a small presentation correction, with shared mesh BVHs.
    pub fn sphere_clear(&self, center_m: [f64; 3], radius_m: f64) -> bool {
        if !center_m.into_iter().all(f64::is_finite) || !radius_m.is_finite() || radius_m <= 0. {
            return false;
        }
        let center = rapier3d_f64::prelude::Pose::from_translation(Vector::from_array(center_m));
        let ball = Ball::new(radius_m);
        let bounds = self.bounds.get_or_init(|| {
            self.shapes
                .iter()
                .map(|(pose, shape)| shape.compute_aabb(pose))
                .collect()
        });
        self.shapes
            .iter()
            .zip(bounds.iter())
            .enumerate()
            .all(|(index, ((pose, shape), bounds))| {
                if !self.visible(
                    index,
                    crate::collision_profiles::CollisionPurpose::RigidBody,
                ) {
                    return true;
                }
                if (0..3).any(|axis| {
                    center_m[axis] + radius_m < bounds.mins[axis]
                        || center_m[axis] - radius_m > bounds.maxs[axis]
                }) {
                    return true;
                }
                rapier3d_f64::parry::query::intersection_test(&center, &ball, pose, shape.as_ref())
                    .is_ok_and(|hit| !hit.intersecting)
            })
    }

    /// Sweep a vertical cylindrical clearance volume between FLU centres (metres).
    /// Includes both endpoints; unsupported queries fail closed. The cylinder's
    /// axis is world +z, independent of robot heading. This does not advance physics.
    ///
    /// ```
    /// use rm_sim_physics::{WorldPhysics, Pose};
    /// let mut world = WorldPhysics::new(&[], 0.);
    /// world.add_static_box(Pose::at([1.,0.,0.5]), [0.01,1.,0.5]);
    /// assert!(!world.static_geometry_snapshot(true).cylinder_sweep_clear([0.,0.,0.5],[2.,0.,0.5],0.3,0.2));
    /// ```
    pub fn cylinder_sweep_clear(
        &self,
        from_m: [f64; 3],
        to_m: [f64; 3],
        radius_m: f64,
        half_height_m: f64,
    ) -> bool {
        if !from_m
            .into_iter()
            .chain(to_m)
            .chain([radius_m, half_height_m])
            .all(f64::is_finite)
            || radius_m <= 0.
            || half_height_m <= 0.
        {
            return false;
        }
        // Parry cylinders are aligned to local Y; rotate to world Z.
        let cylinder = Cylinder::new(half_height_m, radius_m);
        let start = rapier3d_f64::prelude::Pose {
            translation: vector(from_m),
            rotation: Rotation::from_rotation_x(std::f64::consts::FRAC_PI_2),
        };
        self.shape_sweep_clear(start, &cylinder, to_m)
    }
    /// Translate an oriented FLU cuboid to `to_m` without changing its rotation.
    /// Half extents are metres in the cuboid frame. Includes both endpoints;
    /// invalid dimensions, non-unit quaternions and unsupported queries fail closed.
    ///
    /// ```
    /// use rm_sim_physics::{WorldPhysics, Pose};
    /// let mut world = WorldPhysics::new(&[], 0.);
    /// world.add_static_box(Pose::at([0.,0.5,0.2]), [4.,0.1,0.4]);
    /// let scene = world.static_geometry_snapshot(true);
    /// assert!(scene.cuboid_sweep_clear(Pose::at([-1.,0.,0.2]), [1.,0.,0.2], [0.35,0.35,0.1]));
    /// assert!(!scene.cylinder_sweep_clear([-1.,0.,0.2],[1.,0.,0.2],0.49,0.1));
    /// ```
    pub fn cuboid_sweep_clear(&self, from: crate::Pose, to_m: [f64; 3], half_m: [f64; 3]) -> bool {
        if !from
            .translation_m
            .into_iter()
            .chain(from.rotation_wxyz)
            .chain(to_m)
            .chain(half_m)
            .all(f64::is_finite)
            || half_m.iter().any(|&x| x <= 0.)
            || (from.rotation_wxyz.iter().map(|x| x * x).sum::<f64>() - 1.).abs() > 1e-6
        {
            return false;
        }
        self.shape_sweep_clear(pose(from), &Cuboid::new(vector(half_m)), to_m)
    }
    fn shape_sweep_clear(
        &self,
        start: rapier3d_f64::prelude::Pose,
        volume: &dyn Shape,
        to_m: [f64; 3],
    ) -> bool {
        let velocity = vector(to_m) - start.translation;
        let swept = volume.compute_aabb(&start).merged(&volume.compute_aabb(
            &rapier3d_f64::prelude::Pose {
                translation: vector(to_m),
                rotation: start.rotation,
            },
        ));
        let bounds = self
            .bounds
            .get_or_init(|| self.shapes.iter().map(|(p, s)| s.compute_aabb(p)).collect());
        self.shapes
            .iter()
            .zip(bounds.iter())
            .enumerate()
            .all(|(index, ((p, s), bounds))| {
                if !self.visible(
                    index,
                    crate::collision_profiles::CollisionPurpose::RigidBody,
                ) {
                    return true;
                }
                if (0..3).any(|i| swept.maxs[i] < bounds.mins[i] || swept.mins[i] > bounds.maxs[i])
                {
                    return true;
                }
                if velocity.length_squared() < 1e-16 {
                    return rapier3d_f64::parry::query::intersection_test(
                        &start,
                        volume,
                        p,
                        s.as_ref(),
                    )
                    .is_ok_and(|hit| !hit.intersecting);
                }
                rapier3d_f64::parry::query::cast_shapes(
                    &start,
                    velocity,
                    volume,
                    p,
                    Vector::ZERO,
                    s.as_ref(),
                    rapier3d_f64::parry::query::ShapeCastOptions::with_max_time_of_impact(1.),
                )
                .is_ok_and(|hit| hit.is_none())
            })
    }
    /// Test a finite sight segment against the shared CAD meshes, using their
    /// cached AABBs before querying triangle BVHs. Does not advance physics.
    pub fn segment_clear(&self, from_m: [f64; 3], to_m: [f64; 3]) -> bool {
        if !from_m.into_iter().chain(to_m).all(f64::is_finite) {
            return false;
        }
        let delta = vector(to_m) - vector(from_m);
        let length = delta.length();
        if length <= 1e-6 {
            return true;
        }
        let ray = Ray::new(vector(from_m), delta / length);
        let bounds = self.bounds.get_or_init(|| {
            self.shapes
                .iter()
                .map(|(pose, shape)| shape.compute_aabb(pose))
                .collect()
        });
        self.shapes
            .iter()
            .zip(bounds.iter())
            .all(|((pose, shape), bounds)| {
                bounds.cast_local_ray(&ray, length, true).is_none()
                    || shape.cast_ray(pose, &ray, length, true).is_none()
            })
    }

    /// Reconstruct articulated scenery from caller-resolved mechanism positions.
    /// Mesh buffers remain shared; only collider transforms are copied.
    pub fn at_mechanisms(&self, state: &crate::motion::MechanismState) -> Self {
        let mut result = self.clone();
        if !self.mechanisms.is_empty() {
            result.bounds.take();
        }
        for (index, channel, offsets) in &self.mechanisms {
            let crate::motion::MotionChannel::Legacy { kind, team } = channel else {
                continue;
            };
            let fraction = state.fraction(*kind, *team);
            result.shapes[*index].0 = pose(offsets.pose(fraction));
        }
        for bake in &mut result.endpoint_bakes {
            bake.fraction = state.fraction(bake.kind, bake.team);
        }
        result
    }

    /// Preflight a partial explicit coordinate batch without copying shape data.
    /// Repeated shapes sharing a channel are supported; duplicate update entries,
    /// unknown IDs and nonfinite/out-of-range coordinates return an error.
    pub fn validate_motions(
        &self,
        updates: &[crate::motion::MotionUpdate],
    ) -> Result<(), &'static str> {
        crate::motion::validate_updates(updates, self.mechanisms.iter().map(|(_, c, f)| (*c, *f)))
    }

    /// Resolve a partial batch of explicit coordinates, retaining all other poses.
    /// Duplicate/unknown IDs and invalid coordinates fail before any change. Mesh
    /// buffers and rest frames remain shared, including captures made after motion.
    pub fn at_motions(
        &self,
        updates: &[crate::motion::MotionUpdate],
    ) -> Result<Self, &'static str> {
        crate::motion::validate_updates(updates, self.mechanisms.iter().map(|(_, c, f)| (*c, *f)))?;
        let mut result = self.clone();
        if !updates.is_empty() {
            result.bounds.take();
        }
        for (index, channel, frame) in &self.mechanisms {
            if let crate::motion::MotionChannel::Instance(id) = channel
                && let Some(update) = updates.iter().find(|u| u.id == *id)
            {
                result.shapes[*index].0 = pose(frame.pose(update.coordinate));
            }
        }
        Ok(result)
    }

    /// Interpolate the captured collider transforms across a displayed frame interval.
    pub fn interpolate(&self, next: &Self, fraction: f64) -> Result<Self, &'static str> {
        if self.shapes.len() != next.shapes.len()
            || self.query_purpose != next.query_purpose
            || self.collision_groups != next.collision_groups
            || self.mechanisms != next.mechanisms
            || self.endpoint_bakes.len() != next.endpoint_bakes.len()
            || self
                .endpoint_bakes
                .iter()
                .zip(&next.endpoint_bakes)
                .any(|(a, b)| {
                    a.fixed != b.fixed
                        || a.moving != b.moving
                        || a.kind != b.kind
                        || a.team != b.team
                })
        {
            return Err("scenery lifecycle changed");
        }
        let mut result = next.clone();
        result.bounds.take();
        for (a, b) in self.endpoint_bakes.iter().zip(&mut result.endpoint_bakes) {
            b.fraction = a.fraction + (b.fraction - a.fraction) * fraction;
        }
        for ((a, _), (b, _)) in self.shapes.iter().zip(&mut result.shapes) {
            b.translation = a.translation.lerp(b.translation, fraction);
            b.rotation = a.rotation.slerp(b.rotation, fraction);
        }
        Ok(result)
    }
    /// Expand the captured shapes and compound children into world FLU triangles.
    /// Cylinders use a deterministic 32-segment diagnostic boundary mesh; balls
    /// use 12 longitudinal and 8 latitudinal subdivisions. Only this export is
    /// tessellated: captured native shapes still serve contacts, rays and sweeps.
    /// The default wireframe omits query-only meshes; an explicitly selected purpose
    /// exports that view instead. This may be expensive.
    pub fn into_triangles(self) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
        sim_physics::backend::triangulate_shapes(
            self.shapes
                .iter()
                .enumerate()
                .filter(|(i, _)| {
                    self.visible(*i, crate::collision_profiles::CollisionPurpose::RigidBody)
                })
                .map(|(_, shape)| shape.clone()),
        )
    }
}

#[cfg(test)]
mod clearance_tests {
    use crate::{Pose, WorldPhysics};
    fn cylinder_capture(compound: bool) -> super::StaticGeometry {
        use sim_engine::model::{
            BodyKind, BodyState, ColliderDescription, CompoundShape, ContactMaterial,
            RigidBodyDescription, ShapeDescription,
        };
        let cylinder = ShapeDescription::Cylinder {
            radius_m: 0.75,
            half_height_m: 0.5,
        };
        let child_pose = Pose {
            translation_m: [0.5, -1., 0.25],
            rotation_wxyz: [0.5, 0.5, 0.5, 0.5],
        };
        let shape = if compound {
            ShapeDescription::Compound(
                CompoundShape::new(vec![
                    (child_pose, cylinder),
                    (
                        Pose::at([-2., 0., 0.]),
                        ShapeDescription::Cuboid {
                            half_extents_m: [0.1, 0.2, 0.3],
                        },
                    ),
                ])
                .unwrap(),
            )
        } else {
            cylinder
        };
        let angle = std::f64::consts::FRAC_PI_4;
        let collider = ColliderDescription::new(
            shape,
            Pose {
                translation_m: [1., 2., 3.],
                rotation_wxyz: [angle.cos(), angle.sin(), 0., 0.],
            },
            ContactMaterial {
                friction: 0.4,
                restitution: 0.,
                memberships: u32::MAX,
                filter: u32::MAX,
            },
        )
        .unwrap();
        let cooked = crate::cooking::CookedRigidBody::new(
            RigidBodyDescription::new(BodyKind::Fixed, vec![collider]).unwrap(),
        )
        .unwrap();
        let mut world = WorldPhysics::new(&[], -10.);
        world
            .install_body(
                crate::runtime::BodyKey(1),
                &cooked,
                BodyState::at(Pose::at([2., 2., 2.])),
            )
            .unwrap();
        world.static_geometry_snapshot(true)
    }

    #[test]
    fn rotated_native_cylinder_snapshot_queries_caps_sides_and_clearances() {
        let geometry = cylinder_capture(false);
        assert!(geometry.shapes[0].1.as_cylinder().is_some());
        for (origin, direction, distance, normal) in [
            ([3., 4., 8.], [0., 0., -1.], 2.5, [0., 0., 1.]),
            ([6., 4., 5.], [-1., 0., 0.], 2.25, [1., 0., 0.]),
        ] {
            let hit = geometry.cast_ray(origin, direction, 4.).unwrap().unwrap();
            assert!((hit.distance_m - distance).abs() < 1e-10);
            for (actual, expected) in hit.normal.into_iter().zip(normal) {
                assert!((actual - expected).abs() < 1e-10);
            }
        }
        assert!(!geometry.sphere_clear([3.8, 4., 5.], 0.1));
        assert!(geometry.sphere_clear([4., 4., 5.], 0.1));
        assert!(!geometry.cylinder_sweep_clear([1., 4., 5.], [5., 4., 5.], 0.1, 0.1));
        assert!(geometry.cylinder_sweep_clear([1., 5., 5.], [5., 5., 5.], 0.1, 0.1));
        let (vertices, triangles) = geometry.into_triangles();
        assert!(!triangles.is_empty());
        for axis in 0..3 {
            let extent = if axis == 2 { 0.5 } else { 0.75 };
            let center = [3., 4., 5.][axis];
            let min = vertices
                .iter()
                .map(|v| v[axis])
                .fold(f64::INFINITY, f64::min);
            let max = vertices
                .iter()
                .map(|v| v[axis])
                .fold(f64::NEG_INFINITY, f64::max);
            assert!((min - (center - extent)).abs() < 1e-10);
            assert!((max - (center + extent)).abs() < 1e-10);
        }
    }

    #[test]
    fn compound_export_includes_cylinder_and_box_with_all_mounts() {
        use rapier3d_f64::prelude::*;
        let geometry = cylinder_capture(true);
        let (mount, shape) = &geometry.shapes[0];
        let children = shape.as_compound().unwrap().shapes();
        let cylinder_pose = *mount * children[0].0;
        let origin = cylinder_pose.transform_point(Vector::new(0., 2., 0.));
        let direction = cylinder_pose.rotation * Vector::NEG_Y;
        let hit = geometry
            .cast_ray(origin.to_array(), direction.to_array(), 3.)
            .unwrap()
            .unwrap();
        assert!((hit.distance_m - 1.5).abs() < 1e-10);
        assert!(!geometry.sphere_clear(cylinder_pose.translation.to_array(), 0.1));
        let mut expected_vertices = Vec::new();
        let mut expected_triangles = Vec::new();
        for (child_pose, shape) in children {
            let (vertices, triangles) = match shape.as_typed_shape() {
                TypedShape::Cylinder(cylinder) => cylinder.to_trimesh(32),
                TypedShape::Cuboid(cuboid) => cuboid.to_trimesh(),
                _ => panic!("unexpected child"),
            };
            let base = expected_vertices.len() as u32;
            expected_vertices.extend(vertices.into_iter().map(|v| {
                mount
                    .transform_point(child_pose.transform_point(v))
                    .to_array()
            }));
            expected_triangles.extend(triangles.into_iter().map(|t| t.map(|i| i + base)));
        }
        let (vertices, triangles) = geometry.clone().into_triangles();
        assert_eq!(triangles, expected_triangles);
        assert!(!triangles.is_empty());
        assert_eq!(vertices.len(), expected_vertices.len());
        for (actual, expected) in vertices
            .iter()
            .flatten()
            .zip(expected_vertices.iter().flatten())
        {
            assert!((actual - expected).abs() < 1e-10);
        }
        assert_eq!((vertices, triangles), geometry.into_triangles());
    }

    #[test]
    fn nearest_ray_is_metric_bounded_and_not_shape_order_dependent() {
        let mut world = WorldPhysics::new(&[], 0.);
        world.add_static_box(Pose::at([4., 0., 1.]), [1.; 3]);
        world.add_static_box(Pose::at([2., 0., 1.]), [1.; 3]);
        let geometry = world.static_geometry_snapshot(true);
        for scale in [1e-300, 1., 7., 1e300] {
            let hit = geometry
                .cast_ray([0., 0., 1.], [scale, 0., 0.], 1.5)
                .unwrap()
                .unwrap();
            assert_eq!(hit.distance_m, 1.5);
            assert_eq!(hit.position_m, [1.5, 0., 1.]);
            assert_eq!(hit.normal, [-1., 0., 0.]);
        }
        assert!(
            geometry
                .cast_ray([0., 0., 1.], [1., 0., 0.], 1.49)
                .unwrap()
                .is_none()
        );
        assert!(
            geometry
                .cast_ray([0., 0., 1.], [-1., 0., 0.], 10.)
                .unwrap()
                .is_none()
        );
        assert_eq!(
            geometry
                .cast_ray([2., 0., 1.], [1., 0., 0.], 10.)
                .unwrap()
                .unwrap()
                .distance_m,
            0.
        );
        assert!(
            geometry
                .cast_ray([f64::NAN, 0., 0.], [1., 0., 0.], 1.)
                .is_err()
        );
        assert!(geometry.cast_ray([0.; 3], [0.; 3], 1.).is_err());
        assert!(
            geometry
                .cast_ray([0.; 3], [f64::INFINITY, 0., 0.], 1.)
                .is_err()
        );
        for range in [0., -1., f64::NAN, f64::INFINITY] {
            assert!(geometry.cast_ray([0.; 3], [1., 0., 0.], range).is_err());
        }
    }

    #[test]
    fn ray_queries_preserve_triangle_gaps_instead_of_convexifying() {
        let mut world = WorldPhysics::new(&[], -10.);
        world
            .add_static_mesh(
                vec![
                    [2., -2., -1.],
                    [2., -1., -1.],
                    [2., -1., 1.],
                    [2., -2., 1.],
                    [2., 1., -1.],
                    [2., 2., -1.],
                    [2., 2., 1.],
                    [2., 1., 1.],
                ],
                vec![[0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7]],
            )
            .unwrap();
        let geometry = world.static_geometry_snapshot(true);
        assert!(
            geometry
                .cast_ray([0.; 3], [1., 0., 0.], 5.)
                .unwrap()
                .is_none()
        );
        let hit = geometry
            .cast_ray([0., 1.5, 0.], [1., 0., 0.], 5.)
            .unwrap()
            .unwrap();
        assert_eq!(hit.distance_m, 2.);
        let reverse = geometry
            .cast_ray([4., 1.5, 0.], [-1., 0., 0.], 5.)
            .unwrap()
            .unwrap();
        assert_eq!(reverse.distance_m, 2.);
    }
    #[test]
    fn oriented_box_preserves_its_heading_and_checks_the_entire_sweep() {
        let mut world = WorldPhysics::new(&[], 0.);
        world.add_static_box(Pose::at([0., 0.3, 0.2]), [4., 0.1, 0.4]);
        let scene = world.static_geometry_snapshot(true);
        let mut start = Pose::at([-1., 0., 0.2]);
        assert!(scene.cuboid_sweep_clear(start, [1., 0., 0.2], [0.4, 0.1, 0.1]));
        let half_angle = std::f64::consts::FRAC_PI_4;
        start.rotation_wxyz = [half_angle.cos(), 0., 0., half_angle.sin()];
        assert!(!scene.cuboid_sweep_clear(start, [1., 0., 0.2], [0.4, 0.1, 0.1]));
        start.rotation_wxyz = [0.; 4];
        assert!(!scene.cuboid_sweep_clear(start, [1., 0., 0.2], [0.4, 0.1, 0.1]));
        let mut world = WorldPhysics::new(&[], 0.);
        world.add_static_box(Pose::at([0., 0., 0.2]), [0.1, 1., 0.4]);
        let scene = world.static_geometry_snapshot(true);
        let start = Pose::at([-1., 0., 0.2]);
        assert!(scene.cuboid_sweep_clear(start, start.translation_m, [0.1; 3]));
        assert!(!scene.cuboid_sweep_clear(start, [1., 0., 0.2], [0.1; 3]));
    }
}
