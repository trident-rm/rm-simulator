// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Single-owner physical scene with generic bodies, joints, queries and observations.
//!
//! Callers own time and reconstruction recipes. The compatibility facade composes
//! concrete slice operations with its participant adapters in the same native scene.
//! No backend handles, scheduling authority or mutable solver access are public.
use crate::{Pose, StepDuration};
use math::{pose, pose_is_valid, vector};
/// Backend coordinate conversion shared internally with physical adapters.
pub(crate) mod math;
use rapier3d_f64::prelude::*;
use std::collections::HashMap;

#[path = "runtime/assembly.rs"]
pub(crate) mod assembly;
#[path = "runtime/bindings.rs"]
mod bindings;
mod directional;
#[path = "runtime/joints.rs"]
mod joints;
#[path = "runtime/observations.rs"]
mod observations;
#[path = "runtime/scenery.rs"]
mod scenery;
/// Compatibility path for the RoboMaster participant facade.
pub use crate::projectile::WorldPhysics;
pub use assembly::{AssemblyBinding, AssemblyKeys};
pub use bindings::{BodyBinding, BodyKey, BodyState};
pub use directional::DirectionalFriction;
pub use joints::{JointBinding, JointKey};
pub use observations::{
    ColliderBinding, ColliderState, ContactSide, ParticipantBinding, PhysicalContact,
};
pub use scenery::PhysicalGeometry;

/// Maximum solver slice duration, in nanoseconds; preserves the existing contact tuning.
pub(crate) const SUBSTEP_MAX_NS: u64 = 1_000_000;

pub(crate) fn substep_count(step_duration: StepDuration) -> usize {
    usize::try_from(step_duration.as_ns().div_ceil(SUBSTEP_MAX_NS))
        .unwrap_or(1)
        .max(1)
}

/// One native interacting scene, independent of participant roles or match rules.
///
/// Generic command bindings own only bodies installed through this API. Read-only
/// participant tokens also identify bodies installed by compatibility adapters,
/// without enrolling those bodies in generic held-wrench actuation.
/// Reconstruction creates a new brand; old bindings cannot address the new scene.
pub struct PhysicalRuntime {
    bound_bodies: bindings::Bodies,
    bound_joints: joints::Joints,
    bound_body_endpoints: Vec<(RigidBodyHandle, Pose3, Pose3)>,
    step_duration: StepDuration,
    placement_epoch: u64,
    pub(crate) world: PhysicsWorld,
    primed: bool,
    directional: directional::DirectionalContacts,
    contact_observer: Option<observations::Observer>,
}

impl PhysicalRuntime {
    /// Create an empty scene with standard gravity (−9.81 m/s² on world Z).
    /// No floor is installed. Timing is immutable and caller-stamped.
    ///
    /// ```
    /// use rm_sim_physics::{StepDuration, runtime::PhysicalRuntime};
    /// let mut runtime = PhysicalRuntime::new(StepDuration::HZ_1000);
    /// runtime.step(0)?;
    /// assert_eq!(runtime.step_duration(), StepDuration::HZ_1000);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn new(step_duration: StepDuration) -> Self {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::new(0., 0., -9.81);
        let substep_s = step_duration.as_secs_f64() / substep_count(step_duration) as f64;
        world.integration_parameters.dt = substep_s;
        world.integration_parameters.min_ccd_dt = substep_s / 100.;
        world.integration_parameters.max_ccd_substeps = 4;
        Self {
            world,
            bound_bodies: Default::default(),
            bound_joints: Default::default(),
            bound_body_endpoints: Vec::new(),
            step_duration,
            placement_epoch: 0,
            primed: false,
            directional: Default::default(),
            contact_observer: None,
        }
    }

    /// Create a standalone scene with an explicit native constraint-solver budget.
    /// The inclusive 1..=64 iteration bound prevents unbounded setup choices.
    /// More iterations can improve closed-loop accuracy at additional CPU cost;
    /// they do not change the caller's step duration or actuation sampling rate.
    /// This is a numerical setting, not physical spring stiffness. Match/facade
    /// construction retains [`Self::new`] and its existing backend defaults.
    /// Reapply this choice in a reconstruction recipe; it is not checkpoint state.
    ///
    /// ```
    /// use rm_sim_physics::{StepDuration, runtime::PhysicalRuntime};
    /// let world = PhysicalRuntime::with_solver_iterations(StepDuration::HZ_1000, 16)?;
    /// assert_eq!(world.solver_iterations(), 16);
    /// assert_eq!(world.step_duration(), StepDuration::HZ_1000);
    /// assert!(PhysicalRuntime::with_solver_iterations(StepDuration::HZ_1000, 0).is_err());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn with_solver_iterations(
        step_duration: StepDuration,
        iterations: usize,
    ) -> Result<Self, &'static str> {
        if !(1..=64).contains(&iterations) {
            return Err("solver iterations must be within 1..=64");
        }
        let mut runtime = Self::new(step_duration);
        runtime.world.integration_parameters.num_solver_iterations = iterations;
        Ok(runtime)
    }

    /// Native constraint-solver iterations per integration slice, dimensionless.
    /// This read-only budget is fixed at construction for this runtime instance.
    pub fn solver_iterations(&self) -> usize {
        self.world.integration_parameters.num_solver_iterations
    }

    /// Construct a fresh scene using a caller-owned initial recipe.
    /// Errors discard the partial installation. Repeating a recipe reconstructs
    /// initial state, not solver caches or arbitrary checkpoint continuation.
    /// Reacquire bindings by logical keys after replacing a previous runtime.
    ///
    /// ```
    /// use rm_sim_physics::{StepDuration, runtime::PhysicalRuntime};
    /// let recipe = |_: &mut PhysicalRuntime| Ok(());
    /// let runtime = PhysicalRuntime::from_recipe(StepDuration::HZ_128, recipe)?;
    /// assert!(runtime.is_idle());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn from_recipe(
        step_duration: StepDuration,
        setup: impl FnOnce(&mut Self) -> Result<(), &'static str>,
    ) -> Result<Self, &'static str> {
        let mut runtime = Self::new(step_duration);
        setup(&mut runtime)?;
        Ok(runtime)
    }

    /// Immutable integration duration, in explicit simulation time.
    pub fn step_duration(&self) -> StepDuration {
        self.step_duration
    }

    /// Whether generic installations contain only fixed bodies.
    /// Compatibility adapters additionally account for their own active bodies.
    pub fn is_idle(&self) -> bool {
        self.bound_bodies.idle(&self.world)
    }

    /// Process-local scene brand for consumer continuity checks, not authority.
    pub fn runtime_identity(&self) -> u64 {
        self.bound_bodies.identity()
    }

    /// Scene-wide discontinuity revision for geometry interpolation.
    /// Ordinary integration and prescribed endpoints preserve this value. Compare
    /// it together with the runtime brand; reconstruction starts a new brand.
    pub fn placement_epoch(&self) -> u64 {
        self.placement_epoch
    }

    /// Invalidate geometry interpolation before a caller-driven placement jump.
    /// This changes only continuity metadata, not poses or time. Exhaustion fails
    /// before mutation; the value never wraps or aliases an earlier capture.
    /// Adapters call this for explicit teleports, not continuous endpoint motion.
    ///
    /// ```
    /// use rm_sim_physics::{StepDuration, PhysicalRuntime};
    /// let mut runtime = PhysicalRuntime::new(StepDuration::HZ_1000);
    /// let before = runtime.geometry_capture();
    /// runtime.mark_placement_discontinuity()?;
    /// let after = runtime.geometry_capture();
    /// assert_eq!(before.runtime_identity(), after.runtime_identity());
    /// assert_ne!(before.placement_epoch(), after.placement_epoch());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn mark_placement_discontinuity(&mut self) -> Result<(), &'static str> {
        self.placement_epoch = self
            .placement_epoch
            .checked_add(1)
            .ok_or("physical placement epoch exhausted")?;
        Ok(())
    }

    pub(crate) fn check_placement_epoch(&self) -> Result<(), &'static str> {
        self.placement_epoch
            .checked_add(1)
            .map(|_| ())
            .ok_or("physical placement epoch exhausted")
    }

    /// Advance one explicit tick and replace bounded physical observations.
    /// Held wrenches and joint commands apply on every solver slice; kinematic
    /// endpoints interpolate from actual tick-start poses. Time overflow fails
    /// before changing observations or native state. No wall clock is consulted.
    pub fn step(&mut self, prev_ns: u64) -> Result<(), &'static str> {
        prev_ns
            .checked_add(self.step_duration.as_ns())
            .ok_or("physics time overflow")?;
        self.begin_contact_observations();
        self.ensure_queries_primed();
        if self.is_idle() {
            return Ok(());
        }
        self.begin_body_endpoints();
        let substeps = substep_count(self.step_duration);
        for substep in 0..substeps {
            self.apply_body_endpoints((substep + 1) as f64 / substeps as f64);
            self.apply_bound_wrenches();
            self.apply_joint_actuation();
            self.integrate_slice();
            self.capture_physical_contacts(prev_ns, substep, substeps);
        }
        Ok(())
    }

    pub(crate) fn ensure_queries_primed(&mut self) {
        if self.primed {
            return;
        }
        let parameters = self.world.integration_parameters;
        let bounds: Vec<_> = self
            .world
            .colliders
            .iter()
            .filter(|(_, collider)| collider.is_enabled())
            .map(|(handle, collider)| (handle, collider.compute_aabb()))
            .filter(|(_, aabb)| aabb.mins.is_finite() && aabb.maxs.is_finite())
            .collect();
        for (handle, aabb) in bounds {
            self.world.broad_phase.set_aabb(&parameters, handle, aabb);
        }
        self.primed = true;
    }
    pub(crate) fn begin_body_endpoints(&mut self) {
        for (handle, start, _) in &mut self.bound_body_endpoints {
            *start = *self.world.bodies[*handle].position();
        }
    }
    pub(crate) fn apply_body_endpoints(&mut self, fraction: f64) {
        for (handle, start, end) in &self.bound_body_endpoints {
            self.world.bodies[*handle].set_next_kinematic_position(start.lerp(end, fraction));
        }
    }
    pub(crate) fn apply_bound_wrenches(&mut self) {
        self.bound_bodies.apply_wrenches(&mut self.world);
    }
    pub(crate) fn apply_joint_actuation(&mut self) {
        self.bound_joints.apply_actuation(&mut self.world);
    }
    pub(crate) fn integrate_slice(&mut self) {
        self.directional.step(&mut self.world);
    }
}

#[cfg(test)]
#[path = "runtime/tests.rs"]
mod tests;
