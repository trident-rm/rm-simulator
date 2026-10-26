// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Explicit concrete-backend access for trusted physical participant adapters.
//!
//! This is Rapier-specific, not a solver-neutral plugin interface. Native adapters
//! own their bodies, handles and pre/post integration order in the same scene.
//! They must not remove generic bound bodies through native handles, replace the
//! scene, edit integration timing, or omit the generic command/contact phases.
//! Ordinary applications should use `PhysicalRuntime` and `PhysicalSession`.
use crate::{
    PhysicalRuntime, StepDuration,
    cooking::{CookedRigidBody, CookedTriangleMesh},
    runtime::{ParticipantBinding, PhysicalGeometry},
};
use rapier3d_f64::prelude::*;

/// Maximum duration of a native integration slice, in nanoseconds.
pub const SUBSTEP_MAX_NS: u64 = crate::runtime::SUBSTEP_MAX_NS;

/// Native slice count preserving the runtime's integration ceiling.
#[inline]
pub fn substep_count(duration: StepDuration) -> usize {
    crate::runtime::substep_count(duration)
}

/// Backend coordinate conversions shared by participant adapters.
pub mod math {
    use crate::Pose;
    use rapier3d_f64::prelude::{Pose as Pose3, Vector};
    /// Convert an FLU vector without changing its components.
    #[inline]
    pub fn vector(value: [f64; 3]) -> Vector {
        crate::runtime::math::vector(value)
    }
    /// Convert a validated FLU pose using the runtime's quaternion convention.
    #[inline]
    pub fn pose(value: Pose) -> Pose3 {
        crate::runtime::math::pose(value)
    }
    /// Whether every vector component is finite.
    #[inline]
    pub fn finite(value: [f64; 3]) -> bool {
        crate::runtime::math::finite(value)
    }
    /// Whether translation is finite and quaternion norm squared is within 1e-6 of one.
    #[inline]
    pub fn pose_is_valid(value: Pose) -> bool {
        crate::runtime::math::pose_is_valid(value)
    }
}

/// Trusted native scene access and explicit phases for a participant composition.
/// Callers preflight time/inputs, begin observations and endpoints, apply endpoint
/// fractions and participant forces, apply generic held commands, integrate, then
/// capture contacts. The caller owns the exact participant-specific barriers.
///
/// ```
/// use sim_engine::{Pose, model::{BodyKind, RigidBodyDescription}};
/// use sim_physics::{PhysicalRuntime, StepDuration, cooking::CookedRigidBody,
///     backend::{CookedBodyExt, RuntimeExt}};
/// let model = CookedRigidBody::new(RigidBodyDescription::new(BodyKind::Fixed, vec![])?)?;
/// let mut runtime = PhysicalRuntime::new(StepDuration::HZ_1000);
/// let handle = runtime.world_mut().insert_body(model.body_builder());
/// let participant = runtime.extension_participant_for_handle(handle).unwrap();
/// assert_eq!(runtime.participant_state(participant)?.pose, Pose::default());
/// # Ok::<(), &'static str>(())
/// ```
pub trait RuntimeExt {
    /// Borrow the native scene without advancing time or copying state.
    fn world(&self) -> &PhysicsWorld;
    /// Mutate adapter-owned native bodies/colliders, preserving generic bindings
    /// and the immutable integration configuration. Do not step through this reference.
    fn world_mut(&mut self) -> &mut PhysicsWorld;
    /// Check discontinuity epoch capacity before an adapter placement mutation.
    fn extension_check_placement_epoch(&self) -> Result<(), &'static str>;
    /// Prime native queries before participant work on a newly constructed scene.
    fn extension_ensure_queries_primed(&mut self);
    /// Capture generic kinematic endpoint start poses once per tick.
    fn extension_begin_body_endpoints(&mut self);
    /// Apply interpolated generic endpoints at a fraction in (0, 1] of the tick.
    fn extension_apply_body_endpoints(&mut self, fraction: f64);
    /// Apply held generic body wrenches before each integration slice.
    fn extension_apply_bound_wrenches(&mut self);
    /// Apply held generic joint efforts/motors before each integration slice.
    fn extension_apply_joint_actuation(&mut self);
    /// Advance one native slice, including opt-in directional contacts.
    fn extension_integrate_slice(&mut self);
    /// Clear the bounded physical observations at tick start.
    fn extension_begin_contact_observations(&mut self);
    /// Append native contact observations after a slice; time overflow must have
    /// been checked, and the slice index must be less than the supplied count.
    fn extension_capture_physical_contacts(
        &mut self,
        prev_ns: u64,
        substep: usize,
        substeps: usize,
    );
    /// Resolve an adapter-owned native body to a branded read-only participant.
    fn extension_participant_for_handle(
        &self,
        handle: RigidBodyHandle,
    ) -> Option<ParticipantBinding>;
    /// Install parentless native geometry with caller-selected material/filtering.
    /// Adapters retain their existing validation and query-priming boundaries.
    fn extension_insert_scenery(
        &mut self,
        shape: SharedShape,
        placement: Pose3,
        friction: f64,
        restitution: f64,
        groups: InteractionGroups,
    ) -> ColliderHandle;
}
impl RuntimeExt for PhysicalRuntime {
    #[inline]
    fn world(&self) -> &PhysicsWorld {
        &self.world
    }
    #[inline]
    fn world_mut(&mut self) -> &mut PhysicsWorld {
        &mut self.world
    }
    #[inline]
    fn extension_check_placement_epoch(&self) -> Result<(), &'static str> {
        Ok(self.check_placement_epoch()?)
    }
    #[inline]
    fn extension_ensure_queries_primed(&mut self) {
        self.ensure_queries_primed();
    }
    #[inline]
    fn extension_begin_body_endpoints(&mut self) {
        self.begin_body_endpoints();
    }
    #[inline]
    fn extension_apply_body_endpoints(&mut self, fraction: f64) {
        self.apply_body_endpoints(fraction);
    }
    #[inline]
    fn extension_apply_bound_wrenches(&mut self) {
        self.apply_bound_wrenches();
    }
    #[inline]
    fn extension_apply_joint_actuation(&mut self) {
        self.apply_joint_actuation();
    }
    #[inline]
    fn extension_integrate_slice(&mut self) {
        self.integrate_slice();
    }
    #[inline]
    fn extension_begin_contact_observations(&mut self) {
        self.begin_contact_observations();
    }
    #[inline]
    fn extension_capture_physical_contacts(
        &mut self,
        prev_ns: u64,
        substep: usize,
        substeps: usize,
    ) {
        self.capture_physical_contacts(prev_ns, substep, substeps);
    }
    #[inline]
    fn extension_participant_for_handle(
        &self,
        handle: RigidBodyHandle,
    ) -> Option<ParticipantBinding> {
        self.participant_for_handle(handle)
    }
    #[inline]
    fn extension_insert_scenery(
        &mut self,
        shape: SharedShape,
        placement: Pose3,
        friction: f64,
        restitution: f64,
        groups: InteractionGroups,
    ) -> ColliderHandle {
        self.insert_scenery(shape, placement, friction, restitution, groups)
    }
}

/// Native builders from immutable cooked resources; no installation or COM refresh.
pub trait CookedBodyExt {
    /// Build a native body with the source mass/policies, before initial placement.
    fn body_builder(&self) -> RigidBodyBuilder;
    /// Build native colliders in source order, sharing shapes and preserving poses.
    fn collider_builders(&self) -> impl Iterator<Item = ColliderBuilder> + '_;
    /// Optional validated collider-derived mass for adapters with deferred COM refresh.
    fn aggregate_mass_properties(&self) -> Option<&MassProperties>;
    /// Borrow native shapes in source collider order without cooking or copying.
    fn shapes(&self) -> &[SharedShape];
}
impl CookedBodyExt for CookedRigidBody {
    #[inline]
    fn body_builder(&self) -> RigidBodyBuilder {
        self.native_body_builder()
    }
    #[inline]
    fn collider_builders(&self) -> impl Iterator<Item = ColliderBuilder> + '_ {
        self.native_collider_builders()
    }
    #[inline]
    fn aggregate_mass_properties(&self) -> Option<&MassProperties> {
        self.native_aggregate_mass_properties()
    }
    #[inline]
    fn shapes(&self) -> &[SharedShape] {
        self.shapes()
    }
}

/// Borrow the native shape behind a prepared immutable triangle surface.
pub trait CookedMeshExt {
    /// Shared native shape; installation does not cook or duplicate its buffers.
    fn shape(&self) -> &SharedShape;
}
impl CookedMeshExt for CookedTriangleMesh {
    #[inline]
    fn shape(&self) -> &SharedShape {
        &self.shape
    }
}

/// Backend geometry data for sensor adapters; retained poses/shapes belong to
/// the capture, and native collider handles are meaningful only in its runtime.
pub trait GeometryExt {
    /// Capture-order identities, native handles, poses, shared shapes and fixed flags.
    fn sensor_colliders(
        &self,
    ) -> impl ExactSizeIterator<
        Item = (
            crate::runtime::ColliderBinding,
            ColliderHandle,
            Pose3,
            SharedShape,
            bool,
        ),
    > + '_;
}
impl GeometryExt for PhysicalGeometry {
    #[inline]
    fn sensor_colliders(
        &self,
    ) -> impl ExactSizeIterator<
        Item = (
            crate::runtime::ColliderBinding,
            ColliderHandle,
            Pose3,
            SharedShape,
            bool,
        ),
    > + '_ {
        self.sensor_colliders()
    }
}

/// Cook owned surface buffers with the runtime's existing two-sided mesh policy.
pub fn cook_buffers(
    vertices_m: Vec<[f64; 3]>,
    triangles: Vec<[u32; 3]>,
) -> Result<SharedShape, &'static str> {
    Ok(crate::cooking::cook_buffers(vertices_m, triangles)?)
}

/// Expand retained native shape boundaries with the same policy as generic captures.
/// Unsupported/unbounded shapes are omitted; this allocates diagnostic mesh buffers.
pub fn triangulate_shapes(
    shapes: impl IntoIterator<Item = (Pose3, SharedShape)>,
) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
    crate::triangles::triangulate(shapes)
}
