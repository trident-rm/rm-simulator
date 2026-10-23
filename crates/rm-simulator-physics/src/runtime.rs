// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Single-owner Rapier runtime for all interacting physical participants.
//!
//! The runtime owns bodies, colliders, prescribed target synchronization, wheel
//! plant integration, optional physical contact observations and sensor carrier
//! traces. RoboMaster scoring contact policy and buffers belong to the projectile
//! adapter; stepping still composes the specialized chassis/projectile adapters.
//! It executes one explicit caller-stamped tick, preserving the ordinary solver
//! slices and force/contact order. Sessions and match adapters own clocks,
//! command admission, reset recipes and rule barriers; this module adds no clock,
//! match authority, scheduler, plugin interface or alternate physical world.
//!
//! [`WorldPhysics::step_into`] appends ordered raw contacts into retained caller
//! storage. [`WorldPhysics::step`] is the owned-output compatibility adapter.
//! Existing root and projectile import paths name this exact same type.
use super::*;
#[path = "runtime/assembly.rs"]
pub(crate) mod assembly;
#[path = "runtime/bindings.rs"]
mod bindings;
#[path = "runtime/joints.rs"]
mod joints;
#[path = "runtime/observations.rs"]
mod observations;
pub use assembly::{AssemblyBinding, AssemblyKeys};
pub use bindings::{BodyBinding, BodyKey, BodyState};
pub use joints::{JointBinding, JointKey};
pub use observations::{
    ColliderBinding, ColliderState, ContactSide, ParticipantBinding, PhysicalContact,
};

/// Longest integration slice of a world tick, in nanoseconds. A 17 mm shot at
/// the 25 m/s limit crosses further in one 128 Hz tick than the thin armour
/// housings are deep, and Rapier CCD does not recover the hit, so the ballistic
/// world never integrates more than this at once whatever the tick length, and
/// a tick no longer than this is a single slice.
pub(super) const SUBSTEP_MAX_NS: u64 = 1_000_000;

/// Solver runs per world tick: the tick split into [`SUBSTEP_MAX_NS`] slices,
/// so a coarse tick still integrates ballistics at the fine step the contact
/// margins were tuned at.
fn substep_count(step_duration: crate::StepDuration) -> usize {
    usize::try_from(step_duration.as_ns().div_ceil(SUBSTEP_MAX_NS))
        .unwrap_or(1)
        .max(1)
}

/// The field's single Rapier scene and its dynamic physics participants.
///
/// One world holds the static collision geometry, the target faces it can
/// score on, the projectiles in flight and every chassis. Stepping is
/// deterministic for a fixed tick schedule.
pub struct WorldPhysics {
    bound_bodies: bindings::Bodies,
    bound_joints: joints::Joints,
    bound_body_endpoints: Vec<(RigidBodyHandle, Pose3, Pose3)>,
    pub(super) step_duration: crate::StepDuration,
    /// Convex XY slab outside which projectiles are absorbed, in world FLU
    /// metres. `None` means no perimeter absorption. A hit beyond the slab
    /// scores nothing, which also catches a fast shot that crosses a whole
    /// wall in one tick.
    pub projectile_bounds_m: Option<[[f64; 3]; 2]>,
    /// Restitution, flight limit and low-speed retirement rule applied to
    /// projectiles created from now on. Changing it does not alter balls
    /// already in flight.
    pub(super) policy: ProjectilePolicy,
    /// Recent removals, oldest first, bounded to [`REMOVAL_MEMORY`].
    pub(super) removals: std::collections::VecDeque<Removal>,
    pub(super) payloads: Vec<payload::Body>,
    pub(super) world: PhysicsWorld,
    pub(super) projectiles: Vec<Projectile>,
    pub(super) rm_contacts: contacts::State,
    contact_observer: Option<observations::Observer>,
    /// Opt-in substep carrier poses for inertial sensors; observation output,
    /// never restored world state.
    pub(super) carrier_trace: Option<Vec<crate::sensors::imu::CarrierSample>>,
    pub(super) carrier_trace_dropped: u64,
    pub(super) chassis: Vec<Chassis>,
    pub(super) next_chassis_id: u32,
    pub(super) targets: Vec<TargetBody>,
    pub(super) mechanisms: Vec<(
        ColliderHandle,
        crate::motion::MotionChannel,
        crate::motion::MotionFrame,
    )>,
    /// Field time the target bodies currently stand at, if any.
    pub(super) synced_ns: Option<u64>,
    /// The broad-phase tree has been filled at least once.
    pub(super) primed: bool,
    pub(super) next_id: u64,
    pub(super) launched: u64,
}

impl WorldPhysics {
    /// A world with a flat floor at `floor_height_m`. Target bodies are created
    /// for every face; their poses are supplied at each step.
    ///
    /// ```
    /// use rm_simulator_physics::{Caliber, Pose, Shot, tick_ns, TargetFrames, WorldPhysics};
    ///
    /// let mut physics = WorldPhysics::new(&[], 0.0);
    /// let frames = TargetFrames::new(Vec::new());
    ///
    /// // A slow ball falls, rolls and settles one ball radius above the floor.
    /// physics.fire(
    ///     0,
    ///     Pose::at([0.0, 0.0, 1.0]),
    ///     Shot { caliber: Caliber::Mm42, speed_m_s: 3.0 },
    ///     None,
    /// )?;
    /// assert!(!physics.is_idle());
    /// for tick in 0..2_000_000_000 / tick_ns() {
    ///     physics.step(tick * tick_ns(), &frames)?;
    /// }
    ///
    /// let resting = physics.snapshot();
    /// let radius_m = Caliber::Mm42.diameter_m() / 2.0;
    /// let resting_z_m = resting[0].position_m[2];
    /// assert!((resting_z_m - radius_m).abs() < 0.01, "{resting_z_m}");
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn new(faces: &[TargetFace], floor_height_m: f64) -> Self {
        Self::with_step_duration(faces, floor_height_m, crate::StepDuration::default())
    }
    /// Build standalone physics with an immutable, validated integration step.
    /// Call [`Self::step`] once per duration, passing its starting time in nanoseconds.
    /// ```
    /// use rm_simulator_physics::{StepDuration, TargetFrames, WorldPhysics};
    /// let duration = StepDuration::HZ_1000;
    /// let mut physics = WorldPhysics::with_step_duration(&[], 0., duration);
    /// physics.step(0, &TargetFrames::new(vec![])).unwrap();
    /// assert_eq!(physics.step_duration().as_ns(), 1_000_000);
    /// ```
    pub fn with_step_duration(
        faces: &[TargetFace],
        floor_height_m: f64,
        step_duration: crate::StepDuration,
    ) -> Self {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::new(0., 0., -GRAVITY_M_S2);
        let substep_s = step_duration.as_secs_f64() / substep_count(step_duration) as f64;
        world.integration_parameters.dt = substep_s;
        world.integration_parameters.min_ccd_dt = substep_s / 100.;
        world.integration_parameters.max_ccd_substeps = 4;
        world.insert_collider(
            ColliderBuilder::new(SharedShape::halfspace(Vector::Z))
                .translation(Vector::new(0., 0., floor_height_m))
                .friction(FRICTION)
                .restitution(RESTITUTION),
            None,
        );
        let mut ballistics = Self {
            bound_bodies: Default::default(),
            bound_joints: Default::default(),
            bound_body_endpoints: Vec::new(),
            step_duration,
            payloads: Vec::new(),
            world,
            projectile_bounds_m: None,
            policy: ProjectilePolicy::default(),
            removals: std::collections::VecDeque::new(),
            projectiles: Vec::new(),
            rm_contacts: Default::default(),
            contact_observer: None,
            carrier_trace: None,
            carrier_trace_dropped: 0,
            chassis: Vec::new(),
            next_chassis_id: 0,
            targets: Vec::new(),
            mechanisms: Vec::new(),
            synced_ns: None,
            primed: false,
            next_id: 0,
            launched: 0,
        };
        for face in faces {
            ballistics.add_target(*face);
        }
        ballistics
    }
    pub(crate) fn runtime_identity(&self) -> u64 {
        self.bound_bodies.identity()
    }

    /// No active physical participants. Generic fixed anchors alone do not
    /// require integration; dynamic/kinematic bound bodies keep stepping active.
    pub fn is_idle(&self) -> bool {
        self.projectiles.is_empty()
            && self.chassis.is_empty()
            && self.payloads.is_empty()
            && self.bound_bodies.idle(&self.world)
    }

    /// Immutable integration duration of this physical world.
    pub fn step_duration(&self) -> crate::StepDuration {
        self.step_duration
    }

    /// Wheel rays read the broad-phase tree, which the pipeline fills while
    /// stepping. A world that has never stepped has an empty one, so the first
    /// tick would find no ground under any wheel; seed it with what is there.
    fn prime_queries(&mut self) {
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
    }
    /// Declare the target bodies already standing at `time_ns`, so the first
    /// restored tick moves them instead of teleporting them into place.
    pub fn mark_synced(&mut self, time_ns: u64) {
        self.synced_ns = Some(time_ns);
    }
    /// Advance one tick from `prev_ns`, taking the target face poses at the
    /// start and end of the tick from `frames`.
    ///
    /// The solver runs [`SUBSTEP_MAX_NS`]-bounded slices, interpolating the
    /// target poses across the tick, so a coarse tick still carries fast shots
    /// past no armour between solver runs. Returns every contact first seen
    /// this tick, oldest projectile first. Applying detection intervals,
    /// damage and buffs is the caller's business. Errors when the face count or
    /// identity order no longer matches the targets the world was built with.
    ///
    /// ```
    /// use rm_simulator_physics::{
    ///     ArmorTarget, Caliber, Pose, Shot, TargetFace, TargetFrames, WorldPhysics, tick_ns,
    /// };
    ///
    /// let face = TargetFace {
    ///     target: ArmorTarget::Outpost { outpost: 0, face: 0 },
    ///     pose: Pose::yawed([1.5, 0.0, 1.0], std::f64::consts::PI),
    /// };
    /// let mut physics = WorldPhysics::new(&[face], 0.0);
    /// let frames = TargetFrames::new(vec![face]);
    ///
    /// // Two shots one tick apart, side by side so neither can strike the other.
    /// let first = physics.fire(
    ///     0, Pose::at([0.0, 0.0, 1.025]), Shot::at_limit(Caliber::Mm17), None)?;
    /// let second = physics.fire(
    ///     tick_ns(), Pose::at([0.0, 0.04, 1.025]), Shot::at_limit(Caliber::Mm17), None)?;
    ///
    /// let mut seen = Vec::new();
    /// for tick in 0..200 {
    ///     for contact in physics.step(tick * tick_ns(), &frames)? {
    ///         assert_eq!(contact.target, face.target);
    ///         assert!(contact.normal_speed_m_s > 20.0, "{contact:?}");
    ///         seen.push(contact.projectile);
    ///     }
    /// }
    /// assert_eq!(seen, vec![first, second]);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn step(
        &mut self,
        prev_ns: u64,
        frames: &TargetFrames,
    ) -> Result<Vec<Contact>, &'static str> {
        let mut contacts = Vec::new();
        self.step_into(prev_ns, frames, &mut contacts)?;
        Ok(contacts)
    }

    /// Advance one caller-stamped tick and append ordered raw contacts.
    /// Existing output is retained, including on errors. Capacity is reused;
    /// internal solver/contact storage may still allocate. Uses the exact same
    /// slices, synchronization and retirement behavior as [`Self::step`].
    ///
    /// Errors do not promise transactional world rollback: ordinary query priming
    /// and projectile retirement can precede target validation. No new contacts
    /// are appended on error; already completed calls remain committed. The caller
    /// owns the clock and must not advance/step interacting bodies elsewhere.
    ///
    /// ```
    /// use rm_simulator_physics::{runtime::WorldPhysics, TargetFrames};
    /// let mut world = WorldPhysics::new(&[], 0.);
    /// let frames = TargetFrames::new(vec![]);
    /// let mut contacts = Vec::with_capacity(64);
    /// world.step_into(0, &frames, &mut contacts)?;
    /// world.step_into(world.step_duration().as_ns(), &frames, &mut contacts)?;
    /// assert!(contacts.is_empty());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn step_into(
        &mut self,
        prev_ns: u64,
        frames: &TargetFrames,
        contacts: &mut Vec<Contact>,
    ) -> Result<(), &'static str> {
        let next_ns = prev_ns
            .checked_add(self.step_duration.as_ns())
            .ok_or("physics time overflow")?;
        if frames.start.len() != self.targets.len() {
            return Err("target face count changed");
        }
        // Observation output is not transactional: later validation failures may
        // leave the preceding output cleared. Basic preflight above preserves it.
        self.begin_contact_observations();
        if !self.primed {
            self.prime_queries();
            self.primed = true;
        }
        // Discard spent shots before they cost physics time.
        let mut index = 0;
        while index < self.projectiles.len() {
            let projectile = &self.projectiles[index];
            let position = self.world.bodies[projectile.body].translation();
            let reason =
                if next_ns.saturating_sub(projectile.launched_ns) > self.policy.max_flight_ns {
                    Some(RemovalReason::Expired)
                } else if position.z < -1. || position.length() > 200. || !position.is_finite() {
                    Some(RemovalReason::OutOfWorld)
                } else if outside_projectile_bounds(
                    self.projectile_bounds_m,
                    position,
                    projectile.caliber.diameter_m() / 2.0,
                ) {
                    Some(RemovalReason::OutOfBounds)
                } else {
                    None
                };
            if let Some(reason) = reason {
                let projectile = self.projectiles.remove(index);
                self.record_removal(&projectile, next_ns, reason);
                self.world.remove_body(projectile.body);
            } else {
                index += 1;
            }
        }
        if self.is_idle() {
            self.synced_ns = None;
            return Ok(());
        }
        let teleport = self.synced_ns != Some(prev_ns);
        for (target, (start, end)) in self.targets.iter().zip(frames.motions()) {
            if start.target != target.target || end.target != target.target {
                return Err("target face order changed");
            }
            if !pose_is_valid(start.pose) || !pose_is_valid(end.pose) {
                return Err("target face pose must be finite with a unit rotation");
            }
        }
        for (handle, start, _) in &mut self.bound_body_endpoints {
            *start = *self.world.bodies[*handle].position();
        }
        let policy = self.policy;
        let substeps = substep_count(self.step_duration);
        let substep_s = self.step_duration.as_secs_f64() / substeps as f64;
        for substep in 0..substeps {
            let frac_prev = substep as f64 / substeps as f64;
            let frac_next = (substep + 1) as f64 / substeps as f64;
            for (target, (start, end)) in self.targets.iter().zip(frames.motions()) {
                let body = &mut self.world.bodies[target.body];
                if teleport && substep == 0 {
                    body.set_position(pose(start.pose), true);
                }
                body.set_next_kinematic_position(pose(start.pose).lerp(&pose(end.pose), frac_next));
            }
            for (handle, start, end) in &self.bound_body_endpoints {
                self.world.bodies[*handle].set_next_kinematic_position(start.lerp(end, frac_next));
            }
            self.rm_contacts.velocities_before.clear();
            for projectile in &self.projectiles {
                let body = &mut self.world.bodies[projectile.body];
                let velocity = body.linvel();
                let area = std::f64::consts::PI * (projectile.caliber.diameter_m() / 2.).powi(2);
                let drag = -0.5 * AIR_DENSITY_KG_M3 * DRAG_COEFFICIENT * area * velocity.length();
                body.reset_forces(false);
                body.add_force(velocity * drag, false);
                self.rm_contacts.velocities_before.push(velocity);
            }
            for chassis in &mut self.chassis {
                chassis.apply_forces(&mut self.world, substep_s);
            }
            self.rm_contacts.chassis_velocities_before.clear();
            for chassis in &self.chassis {
                let body = &self.world.bodies[chassis.body()];
                self.rm_contacts.chassis_velocities_before.push((
                    body.linvel(),
                    body.angvel(),
                    body.center_of_mass(),
                ));
            }
            self.bound_bodies.apply_wrenches(&mut self.world);
            self.bound_joints.apply_actuation(&mut self.world);
            self.damp_payload_contacts();
            self.world.step();
            self.capture_physical_contacts(prev_ns, substep, substeps);
            for chassis in &self.chassis {
                chassis.sync_fold_colliders(&mut self.world);
            }
            self.record_carriers(prev_ns, substep, substeps, substep_s);
            self.capture_contacts(frames, frac_prev, frac_next, substep_s, next_ns, contacts);
            self.capture_armor_collisions();
            // Absorb perimeter shots the slice they cross the wall, so a fast
            // shot cannot reach past the perimeter and bounce back inside. The
            // convex XY volume also catches one that traverses an entire wall
            // in a single slice.
            let mut index = 0;
            while index < self.projectiles.len() {
                let shot = &self.projectiles[index];
                let position = self.world.bodies[shot.body].translation();
                let reason = if outside_projectile_bounds(
                    self.projectile_bounds_m,
                    position,
                    shot.caliber.diameter_m() / 2.0,
                ) {
                    Some(RemovalReason::OutOfBounds)
                } else if policy.retire_speed_m_s.is_some()
                    && shot.dwell_since_ns.is_some_and(|since| {
                        next_ns.saturating_sub(since) >= policy.retire_dwell_ns
                    })
                {
                    Some(RemovalReason::Retired)
                } else {
                    None
                };
                if let Some(reason) = reason {
                    let shot = self.projectiles.remove(index);
                    self.record_removal(&shot, next_ns, reason);
                    self.world.remove_body(shot.body);
                } else {
                    index += 1;
                }
            }
        }
        self.synced_ns = Some(next_ns);
        Ok(())
    }
    fn record_carriers(&mut self, prev_ns: u64, substep: usize, substeps: usize, substep_s: f64) {
        let Some(trace) = &mut self.carrier_trace else {
            return;
        };
        let duration = u128::from(self.step_duration.as_ns());
        let offset = duration * (substep as u128 + 1) / substeps as u128;
        let time_ns = prev_ns.saturating_add(offset as u64);
        for chassis in &self.chassis {
            if trace.len() >= crate::sensors::imu::MAX_CARRIER_TRACE_SAMPLES {
                self.carrier_trace_dropped += 1;
                continue;
            }
            let body = self.world.bodies[chassis.body()].position();
            trace.push(crate::sensors::imu::CarrierSample {
                robot: chassis.id(),
                placement_revision: chassis.revision(),
                time_ns,
                interval_s: substep_s,
                chassis: Pose {
                    translation_m: body.translation.to_array(),
                    rotation_wxyz: [
                        body.rotation.w,
                        body.rotation.x,
                        body.rotation.y,
                        body.rotation.z,
                    ],
                },
                gimbal: chassis.turret_pose(&self.world),
            });
        }
    }
}
