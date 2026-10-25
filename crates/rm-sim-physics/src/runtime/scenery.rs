// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Neutral fixed scenery installation and immutable physical geometry captures.
use super::*;
use crate::cooking::CookedTriangleMesh;
use crate::queries::{RayHit, RayQuery};
use sim_engine::model::ContactMaterial;

#[derive(Clone)]
struct CapturedCollider {
    binding: ColliderBinding,
    state: ColliderState,
    pose: Pose3,
    shape: SharedShape,
    is_fixed: bool,
}

/// Immutable enabled geometry in native collider order, including half-spaces.
/// Shapes share backend storage; captures retain poses and runtime-branded IDs
/// even after removal or reconstruction. A ray hit's shape index addresses only
/// this capture. No participant roles, motion channels or match policy are stored.
#[derive(Clone)]
pub struct PhysicalGeometry {
    runtime: u64,
    placement_epoch: u64,
    colliders: Vec<CapturedCollider>,
}
impl PhysicalGeometry {
    /// Actual originating scene brand, retained after scene removal/reconstruction.
    /// Metadata for continuity checks, not command authority.
    pub fn runtime_identity(&self) -> u64 {
        self.runtime
    }

    /// Scene-wide discontinuity revision retained at capture time.
    /// Compare together with [`Self::runtime_identity`] before interpolating
    /// captures; a change invalidates the whole scene even for an unchanged carrier.
    pub fn placement_epoch(&self) -> u64 {
        self.placement_epoch
    }

    /// Enumerate copied collider poses and parent identities in capture order.
    pub fn colliders(
        &self,
    ) -> impl ExactSizeIterator<Item = (ColliderBinding, ColliderState)> + '_ {
        self.colliders.iter().map(|c| (c.binding, c.state))
    }

    /// Resolve capture-local shape ordinal, such as a ray hit, to its collider ID.
    pub fn collider_binding(&self, shape_index: usize) -> Option<ColliderBinding> {
        self.colliders.get(shape_index).map(|c| c.binding)
    }

    /// Read retained collider state by identity, independent of current live state.
    pub fn collider_state(&self, binding: ColliderBinding) -> Option<ColliderState> {
        self.colliders
            .iter()
            .find(|c| c.binding == binding)
            .map(|c| c.state)
    }

    /// Find the closest hit along an already validated metric ray.
    /// Inclusive range and first-capture-order ties match existing query semantics.
    /// Half-spaces are tested directly rather than rejected for unbounded AABBs.
    pub fn cast_ray(&self, query: &RayQuery) -> Option<RayHit> {
        let ray = Ray::new(vector(query.origin_m()), vector(query.direction()));
        let mut closest = None;
        let mut limit = query.max_distance_m();
        for (index, collider) in self.colliders.iter().enumerate() {
            if let Some(hit) = collider
                .shape
                // Composite-shape BVH traversal uses a strict upper bound.
                // Expand only the backend request, then enforce our inclusive range.
                .cast_ray_and_get_normal(&collider.pose, &ray, limit.next_up(), true)
                && hit.time_of_impact <= limit
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

    /// Query prepared metric rays into equally sized caller storage.
    /// A length mismatch fails before modifying output; valid misses write `None`.
    pub fn cast_rays_into(
        &self,
        rays: &[RayQuery],
        hits: &mut [Option<RayHit>],
    ) -> Result<(), &'static str> {
        if rays.len() != hits.len() {
            return Err("ray batch output length must match request length");
        }
        for (query, hit) in rays.iter().zip(hits) {
            *hit = self.cast_ray(query);
        }
        Ok(())
    }

    pub(crate) fn sensor_colliders(
        &self,
    ) -> impl ExactSizeIterator<Item = (ColliderBinding, ColliderHandle, Pose3, SharedShape, bool)> + '_
    {
        self.colliders.iter().map(|c| {
            (
                c.binding,
                c.binding.handle,
                c.pose,
                c.shape.clone(),
                c.is_fixed,
            )
        })
    }
}

impl PhysicalRuntime {
    /// Capture enabled colliders without copying shared triangle/convex buffers.
    /// Native poses retain their exact backend bits. Parentless geometry is fixed;
    /// prescribed compatibility motion metadata remains an adapter responsibility.
    pub fn geometry_capture(&self) -> PhysicalGeometry {
        let runtime = self.runtime_identity();
        PhysicalGeometry {
            runtime,
            placement_epoch: self.placement_epoch(),
            colliders: self
                .world
                .all_colliders()
                .filter(|(_, c)| c.is_enabled())
                .map(|(handle, c)| CapturedCollider {
                    binding: ColliderBinding { runtime, handle },
                    state: ColliderState {
                        pose: observations::observed_pose(c.position()),
                        participant: c.parent().and_then(|h| self.participant_for_handle(h)),
                    },
                    pose: *c.position(),
                    shape: c.shared_shape().clone(),
                    is_fixed: c.parent().is_none_or(|h| self.world.bodies[h].is_fixed()),
                })
                .collect(),
        }
    }

    // Domain adapters retain their historical validation and query-priming
    // boundaries. This factory only constructs the parentless native collider;
    // it does not normalize poses, replace filtering or enroll a dynamic body.
    pub(crate) fn insert_scenery(
        &mut self,
        shape: SharedShape,
        placement: Pose3,
        friction: f64,
        restitution: f64,
        groups: InteractionGroups,
    ) -> ColliderHandle {
        self.world.insert_collider(
            scenery_builder(shape, friction, restitution, groups).position(placement),
            None,
        )
    }

    /// Install shared cooked fixed scenery at a validated world FLU pose.
    /// Explicit friction, restitution and filtering introduce no participant roles.
    pub fn install_fixed_mesh(
        &mut self,
        mesh: &CookedTriangleMesh,
        placement: Pose,
        material: ContactMaterial,
    ) -> Result<ColliderBinding, &'static str> {
        if !pose_is_valid(placement) {
            return Err("invalid fixed scenery pose");
        }
        validate_material(material)?;
        let handle = self.insert_scenery(
            mesh.shape.clone(),
            pose(placement),
            material.friction,
            material.restitution,
            material_groups(material),
        );
        self.primed = false;
        Ok(ColliderBinding {
            runtime: self.runtime_identity(),
            handle,
        })
    }

    /// Install a catch half-space below the finite world FLU Z height in metres.
    /// The material is explicit; invalid height/material requests do not mutate.
    ///
    /// ```
    /// use sim_engine::model::ContactMaterial;
    /// use rm_sim_physics::{StepDuration, runtime::PhysicalRuntime, queries::RayQuery};
    /// let mut runtime = PhysicalRuntime::new(StepDuration::HZ_128);
    /// let floor = runtime.install_catch_floor(0., ContactMaterial {
    ///     friction: 0.4, restitution: 0., memberships: u32::MAX, filter: u32::MAX,
    /// })?;
    /// let capture = runtime.geometry_capture();
    /// let ray = RayQuery::new([0., 0., 1.], [0., 0., -1.], 1.)?;
    /// let hit = capture.cast_ray(&ray).unwrap();
    /// assert_eq!(hit.distance_m, 1.);
    /// assert_eq!(capture.collider_binding(hit.shape_index), Some(floor));
    /// runtime.remove_fixed_collider(floor)?;
    /// assert!(runtime.collider_state(floor).is_err());
    /// assert!(capture.collider_state(floor).is_some());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn install_catch_floor(
        &mut self,
        floor_height_m: f64,
        material: ContactMaterial,
    ) -> Result<ColliderBinding, &'static str> {
        if !floor_height_m.is_finite() {
            return Err("floor height must be finite");
        }
        validate_material(material)?;
        let handle = self.insert_scenery(
            SharedShape::halfspace(Vector::Z),
            Pose3::translation(0., 0., floor_height_m),
            material.friction,
            material.restitution,
            material_groups(material),
        );
        self.primed = false;
        Ok(ColliderBinding {
            runtime: self.runtime_identity(),
            handle,
        })
    }

    /// Remove live parentless scenery, invalidating its collider token.
    /// Foreign/stale bindings and body-attached colliders fail before mutation.
    pub fn remove_fixed_collider(&mut self, binding: ColliderBinding) -> Result<(), &'static str> {
        self.collider_state(binding)?;
        if self.world.colliders[binding.handle].parent().is_some() {
            return Err("scenery collider is body-attached");
        }
        self.world.remove_collider(binding.handle);
        self.primed = false;
        Ok(())
    }
}

fn validate_material(material: ContactMaterial) -> Result<(), &'static str> {
    if !material.friction.is_finite()
        || material.friction < 0.
        || !material.restitution.is_finite()
        || !(0. ..=1.).contains(&material.restitution)
    {
        return Err("invalid scenery contact material");
    }
    Ok(())
}
fn material_groups(material: ContactMaterial) -> InteractionGroups {
    InteractionGroups::new(
        Group::from_bits_retain(material.memberships),
        Group::from_bits_retain(material.filter),
        InteractionTestMode::And,
    )
}
fn scenery_builder(
    shape: SharedShape,
    friction: f64,
    restitution: f64,
    groups: InteractionGroups,
) -> ColliderBuilder {
    ColliderBuilder::new(shape)
        .friction(friction)
        .restitution(restitution)
        .collision_groups(groups)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shared_scenery_factory_preserves_native_defaults_and_filter_modes() {
        let placement = Pose3::translation(1., -2., 3.);
        let shapes = [
            SharedShape::halfspace(Vector::Z),
            SharedShape::cuboid(0.2, 0.3, 0.4),
            SharedShape::trimesh(vec![Vector::ZERO, Vector::X, Vector::Y], vec![[0, 1, 2]])
                .unwrap(),
        ];
        let mut expected = PhysicalRuntime::new(crate::StepDuration::HZ_128);
        let mut actual = PhysicalRuntime::new(crate::StepDuration::HZ_128);
        // Capture reinsertion must retain the native test mode, not rebuild an
        // And filter from membership bits. Zero density is intentionally absent.
        for (index, shape) in shapes.into_iter().enumerate() {
            let groups = if index == 0 {
                InteractionGroups::default()
            } else {
                InteractionGroups::new(Group::GROUP_3, Group::ALL, InteractionTestMode::Or)
            };
            let old = expected.world.insert_collider(
                ColliderBuilder::new(shape.clone())
                    .position(placement)
                    .friction(0.6)
                    .restitution(0.2)
                    .collision_groups(groups),
                None,
            );
            let new = actual.insert_scenery(shape, placement, 0.6, 0.2, groups);
            assert_eq!(
                format!("{:?}", expected.world.colliders[old]),
                format!("{:?}", actual.world.colliders[new])
            );
            assert_eq!(old, new);
        }
        assert_eq!(expected.primed, actual.primed);
    }
}
