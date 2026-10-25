// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Read-only native participant identities and opt-in cached contact observations.
use super::*;
use sim_engine::contacts::ContactGeometry;

/// Opaque process-local identity of any native rigid body in one runtime.
/// Native generations reject removed/reused slots; the runtime brand rejects
/// other worlds and resets. This is neither a model key nor a command binding,
/// and is deliberately not serializable.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ParticipantBinding {
    pub(super) runtime: u64,
    pub(super) handle: RigidBodyHandle,
}

/// Opaque process-local identity of any native collider in one runtime.
/// Removal, slot reuse, another world and reset invalidate state access.
/// This read-only token is deliberately not serializable.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ColliderBinding {
    pub(super) runtime: u64,
    pub(super) handle: ColliderHandle,
}

/// Copied privileged collider truth, without roles or collision policy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ColliderState {
    /// Current collider pose in world FLU metres and a wxyz unit quaternion.
    pub pose: Pose,
    /// Actual native parent body; parentless scenery/prescribed geometry is `None`.
    pub participant: Option<ParticipantBinding>,
}

/// One contact side, retaining its actual parent association at capture time.
/// Tokens remain useful for caller identity maps after removal, though subsequent
/// state access then fails rather than resolving a replacement body/collider.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ContactSide {
    /// Collider identity at capture time.
    pub collider: ColliderBinding,
    /// Native parent at capture time, or `None` for a parentless collider.
    pub participant: Option<ParticipantBinding>,
}

/// One deepest cached native contact per active pair per integration slice.
/// This is privileged backend observation, not a begin/end event, hit, measured
/// sensor or checkpoint. Pair order is native encounter order. Points use current
/// collider poses and cached local contacts; normal and separation are cached
/// constraints. They need not form a freshly coherent narrow-phase projection.
/// Incoming impact velocity and gameplay ownership are not inferred.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhysicalContact {
    /// Slice-end time: tick start plus floor(step_ns * substep / substeps).
    pub time_ns: u64,
    /// End of the ordinary caller-stamped world tick, in nanoseconds.
    pub tick_end_ns: u64,
    /// One-based integration slice within the world tick.
    pub substep: u32,
    /// Total integration slices within the world tick.
    pub substeps: u32,
    /// First side in native pair order.
    pub side1: ContactSide,
    /// Second side in native pair order.
    pub side2: ContactSide,
    /// Backend geometry in world FLU metres, sampled immediately after integration.
    pub geometry: ContactGeometry,
}

pub(super) struct Observer {
    retained: Vec<PhysicalContact>,
    limit: usize,
    dropped: usize,
}

impl PhysicalRuntime {
    /// Lazily enumerate all native rigid bodies, without registering command keys.
    /// Enumeration order is backend storage order, not a stable semantic order.
    pub fn participants(&self) -> impl Iterator<Item = ParticipantBinding> + '_ {
        let runtime = self.bound_bodies.identity();
        self.world
            .bodies
            .iter()
            .map(move |(handle, _)| ParticipantBinding { runtime, handle })
    }

    /// Lazily enumerate all native colliders, including parentless scenery.
    /// Enumeration order is backend storage order, not a stable semantic order.
    pub fn colliders(&self) -> impl Iterator<Item = ColliderBinding> + '_ {
        let runtime = self.bound_bodies.identity();
        self.world
            .colliders
            .iter()
            .map(move |(handle, _)| ColliderBinding { runtime, handle })
    }

    /// Read copied world FLU pose and velocities for a live participant.
    /// Foreign worlds, reset and native removal/reuse return an error.
    pub fn participant_state(
        &self,
        binding: ParticipantBinding,
    ) -> Result<BodyState, &'static str> {
        if binding.runtime != self.bound_bodies.identity() {
            return Err("participant binding belongs to another runtime");
        }
        let body = self
            .world
            .bodies
            .get(binding.handle)
            .ok_or("stale participant binding")?;
        let p = body.position();
        Ok(BodyState {
            pose: observed_pose(p),
            velocity_m_s: body.linvel().to_array(),
            angular_velocity_rad_s: body.angvel().to_array(),
        })
    }

    /// Read a live collider's world pose and actual parent body association.
    /// Foreign worlds, reset and native removal/reuse return an error.
    pub fn collider_state(&self, binding: ColliderBinding) -> Result<ColliderState, &'static str> {
        if binding.runtime != self.bound_bodies.identity() {
            return Err("collider binding belongs to another runtime");
        }
        let collider = self
            .world
            .colliders
            .get(binding.handle)
            .ok_or("stale collider binding")?;
        Ok(ColliderState {
            pose: observed_pose(collider.position()),
            participant: collider
                .parent()
                .and_then(|handle| self.participant_for_handle(handle)),
        })
    }

    /// Resolve an installed model command binding to its read-only participant.
    /// This does not register other native bodies for wrench/command ownership.
    pub fn body_participant(
        &self,
        binding: BodyBinding,
    ) -> Result<ParticipantBinding, &'static str> {
        self.participant_for_handle(self.bound_bodies.handle(binding)?)
            .ok_or("stale participant binding")
    }

    pub(crate) fn participant_for_handle(
        &self,
        handle: RigidBodyHandle,
    ) -> Option<ParticipantBinding> {
        self.world.bodies.get(handle).map(|_| ParticipantBinding {
            runtime: self.bound_bodies.identity(),
            handle,
        })
    }

    /// Copy actual participant pose with runtime-derived continuity evidence.
    /// The source carrier and discontinuity revision are caller labels; the scene
    /// brand and native slot/generation come from this validated live participant.
    /// Time is caller-stamped and the interval must be finite and positive.
    /// This is privileged truth, not a measured sensor or command authority.
    ///
    /// ```
    /// use sim_engine::model::{BodyKind, BodyState, CarrierKey, RigidBodyDescription};
    /// use sim_physics::{Pose, StepDuration, cooking::CookedRigidBody,
    ///     runtime::{BodyKey, PhysicalRuntime}};
    /// let model = CookedRigidBody::new(RigidBodyDescription::new(BodyKind::Fixed, vec![])?)?;
    /// let mut runtime = PhysicalRuntime::new(StepDuration::HZ_1000);
    /// let body = runtime.install_body(BodyKey(1), &model, BodyState::at(Pose::default()))?;
    /// let participant = runtime.body_participant(body)?;
    /// let sample = runtime.observe_carrier(participant, CarrierKey(7), 0, 1_000_000, 0.001)?;
    /// assert_eq!(sample.identity().world_brand, runtime.runtime_identity());
    /// assert_eq!(sample.world_from_carrier(), Pose::default());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn observe_carrier(
        &self,
        binding: ParticipantBinding,
        carrier: sim_engine::model::CarrierKey,
        discontinuity_revision: u64,
        time_ns: u64,
        interval_s: f64,
    ) -> Result<crate::sensors::carrier::StampedCarrierPose, &'static str> {
        let state = self.participant_state(binding)?;
        let (index, generation) = binding.handle.into_raw_parts();
        crate::sensors::carrier::StampedCarrierPose::new(
            crate::sensors::carrier::CarrierIdentity {
                carrier,
                world_brand: self.runtime_identity(),
                installation_generation: (u64::from(generation) << 32) | u64::from(index),
                discontinuity_revision,
            },
            time_ns,
            interval_s,
            state.pose,
        )
    }

    /// Enable bounded observations with a positive caller-selected record budget.
    /// Reconfiguration clears records/dropped count; zero or allocation failure
    /// returns an error before mutation. No observer is allocated by default.
    ///
    /// ```
    /// use sim_physics::PhysicalRuntime;
    /// let mut world = PhysicalRuntime::new(sim_physics::StepDuration::HZ_128);
    /// world.enable_contact_observations(64)?;
    /// world.step(0)?;
    /// let mut retained = Vec::new();
    /// world.drain_physical_contacts(&mut retained);
    /// world.disable_contact_observations();
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn enable_contact_observations(&mut self, capacity: usize) -> Result<(), &'static str> {
        if capacity == 0 {
            return Err("contact observation capacity must be positive");
        }
        let mut retained = Vec::new();
        retained
            .try_reserve_exact(capacity)
            .map_err(|_| "contact observation allocation failed")?;
        self.contact_observer = Some(Observer {
            retained,
            limit: capacity,
            dropped: 0,
        });
        Ok(())
    }

    /// Disable observations and release their storage and records.
    pub fn disable_contact_observations(&mut self) {
        self.contact_observer = None;
    }

    /// Borrow records retained until the next step, drain or reconfiguration.
    /// The budget spans a whole tick. A failed step is not transactional: earlier
    /// preflight failures may preserve records, later failures may leave them cleared.
    pub fn physical_contacts(&self) -> &[PhysicalContact] {
        self.contact_observer
            .as_ref()
            .map_or(&[], |observer| observer.retained.as_slice())
    }

    /// Records omitted for capacity during the latest tick, saturating at usize max.
    /// Drain leaves this count intact; the next step or reconfiguration clears it.
    pub fn physical_contacts_dropped(&self) -> usize {
        self.contact_observer
            .as_ref()
            .map_or(0, |observer| observer.dropped)
    }

    /// Append observations into caller storage, retaining the observer allocation.
    /// Existing caller records are preserved; this does not reset the dropped count.
    pub fn drain_physical_contacts(&mut self, output: &mut Vec<PhysicalContact>) {
        if let Some(observer) = &mut self.contact_observer {
            output.append(&mut observer.retained);
        }
    }

    pub(crate) fn begin_contact_observations(&mut self) {
        if let Some(observer) = &mut self.contact_observer {
            observer.retained.clear();
            observer.dropped = 0;
        }
    }

    pub(crate) fn capture_physical_contacts(
        &mut self,
        prev_ns: u64,
        substep: usize,
        substeps: usize,
    ) {
        // The disabled path must return before scanning native contact pairs.
        let Some(observer) = &mut self.contact_observer else {
            return;
        };
        let runtime = self.bound_bodies.identity();
        let duration = u128::from(self.step_duration.as_ns());
        let offset = duration * (substep as u128 + 1) / substeps as u128;
        // Ordinary step preflight already checked the tick endpoint for overflow.
        let tick_end_ns = prev_ns + self.step_duration.as_ns();
        let time_ns = prev_ns + offset as u64;
        for pair in self.world.contact_pairs() {
            if !pair.has_any_active_contact() {
                continue;
            }
            let Some((manifold, contact)) = pair.find_deepest_contact() else {
                continue;
            };
            if observer.retained.len() >= observer.limit {
                observer.dropped = observer.dropped.saturating_add(1);
                continue;
            }
            let collider1 = &self.world.colliders[pair.collider1];
            let collider2 = &self.world.colliders[pair.collider2];
            let side = |handle, collider: &Collider| ContactSide {
                collider: ColliderBinding { runtime, handle },
                participant: collider.parent().and_then(|handle| {
                    self.world
                        .bodies
                        .get(handle)
                        .map(|_| ParticipantBinding { runtime, handle })
                }),
            };
            observer.retained.push(PhysicalContact {
                time_ns,
                tick_end_ns,
                substep: (substep + 1) as u32,
                substeps: substeps as u32,
                side1: side(pair.collider1, collider1),
                side2: side(pair.collider2, collider2),
                geometry: ContactGeometry {
                    point1_m: collider1
                        .position()
                        .transform_point(contact.local_p1)
                        .to_array(),
                    point2_m: collider2
                        .position()
                        .transform_point(contact.local_p2)
                        .to_array(),
                    normal_from1_to2: manifold.data.normal.to_array(),
                    separation_m: contact.dist,
                },
            });
        }
    }
}

pub(super) fn observed_pose(pose: &Pose3) -> Pose {
    Pose {
        translation_m: pose.translation.to_array(),
        rotation_wxyz: [
            pose.rotation.w,
            pose.rotation.x,
            pose.rotation.y,
            pose.rotation.z,
        ],
    }
}
