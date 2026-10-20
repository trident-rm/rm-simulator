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
/// use rm_simulator_physics::{Pose, WorldPhysics};
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
    pub(crate) mechanisms: Vec<(
        usize,
        crate::motion::Mechanism,
        Team,
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
impl StaticGeometry {
    /// Separate articulated scenery from fixed geometry for debug drawing.
    pub fn fixed_only(&self) -> Self {
        Self {
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
            mechanisms: Vec::new(),
        }
    }

    /// Retain each moving shape in its untranslated frame, sharing CAD buffers.
    pub fn moving_parts(&self) -> Vec<MovingGeometry> {
        self.mechanisms
            .iter()
            .map(|(index, kind, team, offsets)| {
                let (mut pose, shape) = self.shapes[*index].clone();
                pose.translation = Vector::ZERO;
                if matches!(
                    kind,
                    crate::motion::Mechanism::Outpost | crate::motion::Mechanism::Rune
                ) {
                    pose = Default::default();
                }
                MovingGeometry {
                    kind: *kind,
                    team: *team,
                    motion: *offsets,
                    geometry: Self {
                        shapes: vec![(pose, shape)],
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
            .all(|((pose, shape), bounds)| {
                if (0..3).any(|axis| {
                    center_m[axis] + radius_m < bounds.mins[axis]
                        || center_m[axis] - radius_m > bounds.maxs[axis]
                }) {
                    return true;
                }
                rapier3d_f64::parry::query::intersection_test(&center, &ball, pose, shape.as_ref())
                    .is_ok_and(|hit| !hit)
            })
    }

    /// Sweep a vertical cylindrical clearance volume between FLU centres (metres).
    /// Includes both endpoints; unsupported queries fail closed. The cylinder's
    /// axis is world +z, independent of robot heading. This does not advance physics.
    ///
    /// ```
    /// use rm_simulator_physics::{WorldPhysics, Pose};
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
        let velocity = vector(to_m) - vector(from_m);
        let extents = [radius_m, radius_m, half_height_m];
        let bounds = self
            .bounds
            .get_or_init(|| self.shapes.iter().map(|(p, s)| s.compute_aabb(p)).collect());
        self.shapes
            .iter()
            .zip(bounds.iter())
            .all(|((pose, shape), bounds)| {
                if (0..3).any(|i| {
                    from_m[i].max(to_m[i]) + extents[i] < bounds.mins[i]
                        || from_m[i].min(to_m[i]) - extents[i] > bounds.maxs[i]
                }) {
                    return true;
                }
                if velocity.length_squared() < 1e-16 {
                    return rapier3d_f64::parry::query::intersection_test(
                        &start,
                        &cylinder,
                        pose,
                        shape.as_ref(),
                    )
                    .is_ok_and(|hit| !hit);
                }
                rapier3d_f64::parry::query::cast_shapes(
                    &start,
                    velocity,
                    &cylinder,
                    pose,
                    Vector::ZERO,
                    shape.as_ref(),
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
        for (index, kind, team, offsets) in &self.mechanisms {
            let fraction = state.fraction(*kind, *team);
            result.shapes[*index].0 = pose(offsets.pose(fraction));
        }
        result
    }

    /// Interpolate the captured collider transforms across a displayed frame interval.
    pub fn interpolate(&self, next: &Self, fraction: f64) -> Result<Self, &'static str> {
        if self.shapes.len() != next.shapes.len() {
            return Err("scenery lifecycle changed");
        }
        let mut result = next.clone();
        result.bounds.take();
        for ((a, _), (b, _)) in self.shapes.iter().zip(&mut result.shapes) {
            b.translation = a.translation.lerp(b.translation, fraction);
            b.rotation = a.rotation.slerp(b.rotation, fraction);
        }
        Ok(result)
    }
    /// Expand the captured shapes into world FLU triangles. This may be expensive.
    pub fn into_triangles(self) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
        let mut vertices_m = Vec::new();
        let mut triangles = Vec::new();
        for (pose, shape) in self.shapes {
            let cuboid_mesh;
            let (local, indices) = match shape.as_typed_shape() {
                TypedShape::TriMesh(mesh) => (mesh.vertices(), mesh.indices()),
                TypedShape::Ball(ball) => {
                    cuboid_mesh = ball.to_trimesh(12, 8);
                    (cuboid_mesh.0.as_slice(), cuboid_mesh.1.as_slice())
                }
                TypedShape::Cuboid(cuboid) => {
                    cuboid_mesh = cuboid.to_trimesh();
                    (cuboid_mesh.0.as_slice(), cuboid_mesh.1.as_slice())
                }
                _ => continue,
            };
            let base = vertices_m.len() as u32;
            vertices_m.extend(local.iter().map(|v| {
                let p = pose.transform_point(*v);
                [p.x, p.y, p.z]
            }));
            triangles.extend(indices.iter().map(|t| t.map(|i| i + base)));
        }
        (vertices_m, triangles)
    }
}
