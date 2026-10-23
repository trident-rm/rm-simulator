// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! World-scoped non-reused bindings for source-described physical bodies.
use super::*;
use crate::cooking::CookedRigidBody;
use rm_sim_engine::model::{BodyKind, RigidBodyDescription};
use std::sync::atomic::{AtomicU64, Ordering};

/// Solver-free copied body state, also available in engine model descriptions.
pub use rm_sim_engine::model::BodyState;

/// Caller-selected body identity within a runtime, independent of backend slots.
/// Scenario recipes may reuse keys after reconstruction; bindings must be reacquired.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BodyKey(pub u64);

/// Opaque binding to one installation in one runtime. Copying/moving a runtime
/// preserves bindings; removal, replacement or scenario reset invalidates them.
/// Values are process-local, not serializable checkpoint/model identities.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BodyBinding {
    runtime: u64,
    key: BodyKey,
    generation: u64,
}
impl BodyBinding {
    /// Caller-selected logical identity; this key alone does not validate a binding.
    pub fn key(self) -> BodyKey {
        self.key
    }
}

struct Entry {
    generation: u64,
    handle: RigidBodyHandle,
    model: CookedRigidBody,
    force_n: Vector,
    torque_nm: Vector,
}

pub(super) struct Bodies {
    runtime: u64,
    next_generation: u64,
    entries: HashMap<BodyKey, Entry>,
    dynamic_keys: Vec<BodyKey>,
}
impl Default for Bodies {
    fn default() -> Self {
        // Identity branding does not enter physics calculations or checkpoints.
        // A fresh world/reset must never reuse another live/captured token.
        static NEXT: AtomicU64 = AtomicU64::new(1);
        let runtime = NEXT
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |x| x.checked_add(1))
            .expect("physical runtime identity exhausted");
        Self {
            runtime,
            next_generation: 0,
            entries: HashMap::new(),
            dynamic_keys: Vec::new(),
        }
    }
}
impl Bodies {
    pub(super) fn identity(&self) -> u64 {
        self.runtime
    }
    fn entry(&self, binding: BodyBinding) -> Result<&Entry, &'static str> {
        if binding.runtime != self.runtime {
            return Err("body binding belongs to another runtime");
        }
        self.entries
            .get(&binding.key)
            .filter(|e| e.generation == binding.generation)
            .ok_or("stale body binding")
    }
    pub(super) fn contains(&self, key: BodyKey) -> bool {
        self.entries.contains_key(&key)
    }
    pub(super) fn check_capacity(&self, count: usize) -> Result<(), &'static str> {
        let count = u64::try_from(count).map_err(|_| "too many bodies")?;
        self.next_generation
            .checked_add(count)
            .ok_or("body binding generation exhausted")?;
        Ok(())
    }
    pub(super) fn handle(&self, binding: BodyBinding) -> Result<RigidBodyHandle, &'static str> {
        Ok(self.entry(binding)?.handle)
    }
    pub(super) fn apply_wrenches(&self, world: &mut PhysicsWorld) {
        for key in &self.dynamic_keys {
            let entry = &self.entries[key];
            let body = &mut world.bodies[entry.handle];
            if body.is_dynamic() {
                body.reset_forces(false);
                body.reset_torques(false);
                body.add_force(entry.force_n, false);
                body.add_torque(entry.torque_nm, false);
            }
        }
    }
    fn binding(&self, key: BodyKey, entry: &Entry) -> BodyBinding {
        BodyBinding {
            runtime: self.runtime,
            key,
            generation: entry.generation,
        }
    }
    pub(super) fn idle(&self, world: &PhysicsWorld) -> bool {
        self.entries
            .values()
            .all(|e| world.bodies[e.handle].is_fixed())
    }
}

impl PhysicalRuntime {
    /// Install a cooked body under an explicit unique key and initial world state.
    /// Validation/duplicate checks precede mutation. Dynamic mass/COM/inertia come
    /// only from the source: explicit inertia or opt-in collider aggregation. Fixed and
    /// position-driven kinematic bodies require zero initial velocities.
    ///
    /// ```
    /// use rm_sim_engine::{Pose, model::{BodyKind, BodyState, RigidBodyDescription}};
    /// use rm_sim_physics::{runtime::{WorldPhysics, BodyKey}, cooking::CookedRigidBody};
    /// let anchor = CookedRigidBody::new(RigidBodyDescription::new(BodyKind::Fixed, vec![])?)?;
    /// let mut world = WorldPhysics::new(&[], 0.);
    /// let binding = world.install_body(BodyKey(7), &anchor, BodyState::at(Pose::at([0.; 3])))?;
    /// assert_eq!(world.body_binding(BodyKey(7)), Some(binding));
    /// world.remove_bound_body(binding)?;
    /// assert!(world.body_state(binding).is_err());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn install_body(
        &mut self,
        key: BodyKey,
        model: &CookedRigidBody,
        state: BodyState,
    ) -> Result<BodyBinding, &'static str> {
        state.validate()?;
        if self.bound_bodies.entries.contains_key(&key) {
            return Err("duplicate body key");
        }
        self.bound_bodies.check_capacity(1)?;
        let body = prepare_body(model, state)?;
        Ok(self.commit_body(key, model, body))
    }

    pub(super) fn commit_body(
        &mut self,
        key: BodyKey,
        model: &CookedRigidBody,
        body: RigidBody,
    ) -> BodyBinding {
        let handle = self.world.insert_body(body);
        for (description, builder) in model
            .source()
            .colliders()
            .iter()
            .zip(model.collider_builders())
        {
            // Retain the generic API's historical normalization, while participant
            // adapters may consume exact stored source rotations from the cooker.
            self.world
                .insert_collider(builder.position(pose(description.pose())), Some(handle));
        }
        // Rapier otherwise defers this to the next integration. Joint admission
        // and first-slice anchor forces require the installed source COM now.
        self.world.bodies[handle].recompute_mass_properties_from_colliders(&self.world.colliders);
        let entry = Entry {
            handle,
            generation: self.bound_bodies.next_generation,
            model: model.clone(),
            force_n: Vector::ZERO,
            torque_nm: Vector::ZERO,
        };
        let binding = self.bound_bodies.binding(key, &entry);
        self.bound_bodies.entries.insert(key, entry);
        if self.world.bodies[handle].is_dynamic() {
            self.bound_bodies.dynamic_keys.push(key);
        }
        self.bound_bodies.next_generation += 1;
        self.primed = false;
        binding
    }

    /// Rebind a logical scenario key after a successful reconstruction/reset.
    pub fn body_binding(&self, key: BodyKey) -> Option<BodyBinding> {
        self.bound_bodies
            .entries
            .get(&key)
            .map(|e| self.bound_bodies.binding(key, e))
    }

    /// Borrow the installed immutable source without cloning collider buffers.
    pub fn body_description(
        &self,
        binding: BodyBinding,
    ) -> Result<&RigidBodyDescription, &'static str> {
        Ok(self.bound_bodies.entry(binding)?.model.source())
    }

    /// Read copied privileged motion, not a measured sensor or checkpoint.
    pub fn body_state(&self, binding: BodyBinding) -> Result<BodyState, &'static str> {
        let body = &self.world.bodies[self.bound_bodies.entry(binding)?.handle];
        let p = body.position();
        Ok(BodyState {
            pose: Pose {
                translation_m: p.translation.to_array(),
                rotation_wxyz: [p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z],
            },
            velocity_m_s: body.linvel().to_array(),
            angular_velocity_rad_s: body.angvel().to_array(),
        })
    }

    /// Replace dynamic world COM velocity and angular velocity; wake the body.
    /// Nonfinite requests or bindings to fixed/kinematic bodies fail before mutation.
    pub fn set_body_velocity(
        &mut self,
        binding: BodyBinding,
        velocity_m_s: [f64; 3],
        angular_velocity_rad_s: [f64; 3],
    ) -> Result<(), &'static str> {
        if !velocity_m_s
            .into_iter()
            .chain(angular_velocity_rad_s)
            .all(f64::is_finite)
        {
            return Err("nonfinite body velocity");
        }
        let handle = self.bound_bodies.entry(binding)?.handle;
        let body = &mut self.world.bodies[handle];
        if !body.is_dynamic() {
            return Err("body is not dynamic");
        }
        body.set_linvel(vector(velocity_m_s), true);
        body.set_angvel(vector(angular_velocity_rad_s), true);
        Ok(())
    }

    /// Replace a held world-frame force (N) at the COM and torque (N m).
    /// Remains active across solver slices/ticks until replaced or removed; zero
    /// clears it. This does not change other participants' actuation or advance time.
    pub fn set_body_wrench(
        &mut self,
        binding: BodyBinding,
        force_n: [f64; 3],
        torque_nm: [f64; 3],
    ) -> Result<(), &'static str> {
        if !force_n.into_iter().chain(torque_nm).all(f64::is_finite) {
            return Err("nonfinite body wrench");
        }
        let handle = self.bound_bodies.entry(binding)?.handle;
        let body = &mut self.world.bodies[handle];
        if !body.is_dynamic() {
            return Err("body is not dynamic");
        }
        body.reset_forces(false);
        body.reset_torques(false);
        body.add_force(vector(force_n), true);
        body.add_torque(vector(torque_nm), true);
        let entry = self
            .bound_bodies
            .entries
            .get_mut(&binding.key)
            .expect("validated binding");
        entry.force_n = vector(force_n);
        entry.torque_nm = vector(torque_nm);
        Ok(())
    }

    /// Set the end pose for the next runtime tick on a prescribed body. The
    /// runtime interpolates from its actual current pose across solver slices.
    /// Only kinematic bodies accept endpoints; no teleport/placement API is implied.
    /// Repeated requests replace an endpoint; first-request order is retained.
    pub fn set_body_endpoint(
        &mut self,
        binding: BodyBinding,
        endpoint: Pose,
    ) -> Result<(), &'static str> {
        BodyState::at(endpoint).validate()?;
        let handle = self.bound_bodies.entry(binding)?.handle;
        if !self.world.bodies[handle].is_kinematic() {
            return Err("body is not kinematic");
        }
        if let Some((_, _, end)) = self
            .bound_body_endpoints
            .iter_mut()
            .find(|(h, _, _)| *h == handle)
        {
            *end = pose(endpoint);
        } else {
            self.bound_body_endpoints.push((
                handle,
                *self.world.bodies[handle].position(),
                pose(endpoint),
            ));
        }
        Ok(())
    }

    /// Remove one generic body, its colliders and attached described joints.
    /// Invalidates the body binding and every attached joint binding.
    /// Reinstalling the same logical key produces a different generation.
    pub fn remove_bound_body(&mut self, binding: BodyBinding) -> Result<(), &'static str> {
        let handle = self.bound_bodies.entry(binding)?.handle;
        self.bound_joints.remove_attached(handle, &mut self.world);
        self.bound_bodies.entries.remove(&binding.key);
        self.bound_bodies
            .dynamic_keys
            .retain(|key| *key != binding.key);
        self.bound_body_endpoints.retain(|(h, _, _)| *h != handle);
        self.world.remove_body(handle);
        self.primed = false;
        Ok(())
    }
}

/// Prepare native state without consuming world handles or binding generations.
pub(super) fn prepare_body(
    model: &CookedRigidBody,
    state: BodyState,
) -> Result<RigidBody, &'static str> {
    state.validate()?;
    let mut body = model.body_builder();
    if !matches!(
        model.source().kind(),
        BodyKind::Dynamic(_) | BodyKind::DynamicFromColliders
    ) && state
        .velocity_m_s
        .into_iter()
        .chain(state.angular_velocity_rad_s)
        .any(|x| x != 0.)
    {
        return Err("non-dynamic body requires zero initial velocities");
    }
    body = body
        .pose(pose(state.pose))
        .linvel(vector(state.velocity_m_s))
        .angvel(vector(state.angular_velocity_rad_s));
    let mut body = body.build();
    body.recompute_mass_properties_from_colliders(&ColliderSet::new());
    let aggregate_com_is_finite = model
        .aggregate_mass_properties()
        .is_none_or(|mass| body.position().transform_point(mass.local_com).is_finite());
    if !aggregate_com_is_finite
        || !body.center_of_mass().is_finite()
        || model.source().colliders().iter().any(|collider| {
            let world = body.position() * pose(collider.pose());
            !world.translation.is_finite() || !world.rotation.is_finite()
        })
    {
        return Err("nonfinite placed body mass or collider frame");
    }
    Ok(body)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TargetFrames;
    use rm_sim_engine::model::{ColliderDescription, ContactMaterial, Inertial, ShapeDescription};
    use std::sync::Arc;

    fn aggregate_source() -> RigidBodyDescription {
        use rm_sim_engine::model::{CoefficientCombineRule as Rule, ColliderMass, ColliderPolicy};
        let colliders = [
            (3., [0., 0., 0.]),
            (1., [0.1, 0., 0.5]),
            (0., [0., 0., -0.5]),
        ]
        .into_iter()
        .map(|(mass, center)| {
            ColliderDescription::new(
                ShapeDescription::Cuboid {
                    half_extents_m: [0.2; 3],
                },
                Pose::at(center),
                ContactMaterial {
                    friction: 0.4,
                    restitution: 0.2,
                    memberships: u32::MAX,
                    filter: u32::MAX,
                },
            )
            .unwrap()
            .with_policy(ColliderPolicy {
                mass: ColliderMass::MassKg(mass),
                friction_combine_rule: Rule::Multiply,
                restitution_combine_rule: Rule::Min,
            })
            .unwrap()
        })
        .collect();
        RigidBodyDescription::new(BodyKind::DynamicFromColliders, colliders).unwrap()
    }

    #[test]
    fn native_policy_and_ordered_mass_aggregation_match_direct_builders() {
        use rm_sim_engine::model::BodyPolicy;
        let policy = BodyPolicy {
            gravity_scale: 0.,
            enabled_translations: [true, true, false],
            enabled_rotations: [false, false, true],
            can_sleep: false,
            linear_damping: 0.3,
            angular_damping: 0.6,
            ccd_enabled: true,
        };
        let model = CookedRigidBody::new(aggregate_source().with_policy(policy).unwrap()).unwrap();
        let mut generic = WorldPhysics::new(&[], -10.);
        let mut native = WorldPhysics::new(&[], -10.);
        let state = BodyState {
            pose: Pose::at([0., 0., 2.]),
            velocity_m_s: [1., 2., 0.],
            angular_velocity_rad_s: [0., 0., 0.5],
        };
        let binding = generic.install_body(BodyKey(1), &model, state).unwrap();
        let a = generic.physical.bound_bodies.handle(binding).unwrap();
        let b = native.physical.world.insert_body(
            RigidBodyBuilder::dynamic()
                .gravity_scale(0.)
                .enabled_translations(true, true, false)
                .enabled_rotations(false, false, true)
                .can_sleep(false)
                .linear_damping(0.3)
                .angular_damping(0.6)
                .ccd_enabled(true)
                .pose(pose(state.pose))
                .linvel(vector(state.velocity_m_s))
                .angvel(vector(state.angular_velocity_rad_s)),
        );
        for (mass, center) in [
            (3., [0., 0., 0.]),
            (1., [0.1, 0., 0.5]),
            (0., [0., 0., -0.5]),
        ] {
            native.physical.world.insert_collider(
                ColliderBuilder::cuboid(0.2, 0.2, 0.2)
                    .translation(vector(center))
                    .mass(mass)
                    .friction(0.4)
                    .restitution(0.2)
                    .friction_combine_rule(CoefficientCombineRule::Multiply)
                    .restitution_combine_rule(CoefficientCombineRule::Min),
                Some(b),
            );
        }
        native.physical.world.bodies[b]
            .recompute_mass_properties_from_colliders(&native.physical.world.colliders);
        assert_eq!(
            generic.physical.world.bodies[a]
                .mass_properties()
                .local_mprops,
            native.physical.world.bodies[b]
                .mass_properties()
                .local_mprops
        );
        assert_eq!(generic.physical.world.bodies[a].mass(), 4.);
        assert_eq!(
            generic.physical.world.bodies[a].locked_axes(),
            native.physical.world.bodies[b].locked_axes()
        );
        assert_eq!(
            generic.physical.world.bodies[a].is_ccd_enabled(),
            native.physical.world.bodies[b].is_ccd_enabled()
        );
        for (&ca, &cb) in generic.physical.world.bodies[a]
            .colliders()
            .iter()
            .zip(native.physical.world.bodies[b].colliders())
        {
            let ca = &generic.physical.world.colliders[ca];
            let cb = &native.physical.world.colliders[cb];
            assert_eq!(ca.mass_properties(), cb.mass_properties());
            assert_eq!(ca.friction_combine_rule(), cb.friction_combine_rule());
            assert_eq!(ca.restitution_combine_rule(), cb.restitution_combine_rule());
        }
        for _ in 0..128 {
            generic.physical.world.step();
            native.physical.world.step();
            assert_eq!(
                generic.physical.world.bodies[a].position(),
                native.physical.world.bodies[b].position()
            );
            assert_eq!(
                generic.physical.world.bodies[a].linvel(),
                native.physical.world.bodies[b].linvel()
            );
            assert_eq!(
                generic.physical.world.bodies[a].angvel(),
                native.physical.world.bodies[b].angvel()
            );
        }
    }

    #[test]
    fn unchanged_generic_constructors_match_original_native_defaults() {
        let inertial = Inertial::new(2., [1., 1.5, 2.], Pose::at([0.1, 0., 0.2])).unwrap();
        let source = RigidBodyDescription::new(
            BodyKind::Dynamic(inertial),
            vec![
                ColliderDescription::new(
                    ShapeDescription::Ball { radius_m: 0.1 },
                    Pose::default(),
                    ContactMaterial {
                        friction: 0.4,
                        restitution: 0.2,
                        memberships: u32::MAX,
                        filter: u32::MAX,
                    },
                )
                .unwrap(),
            ],
        )
        .unwrap();
        let model = CookedRigidBody::new(source).unwrap();
        let mut generic = WorldPhysics::new(&[], -10.);
        let mut native = WorldPhysics::new(&[], -10.);
        let initial = Pose::at([0., 0., 2.]);
        let live = generic
            .install_body(BodyKey(1), &model, BodyState::at(initial))
            .unwrap();
        let a = generic.physical.bound_bodies.handle(live).unwrap();
        let b = native.physical.world.insert_body(
            RigidBodyBuilder::dynamic()
                .additional_mass_properties(MassProperties::with_principal_inertia_frame(
                    vector([0.1, 0., 0.2]),
                    2.,
                    vector([1., 1.5, 2.]),
                    Rotation::IDENTITY,
                ))
                .pose(pose(initial)),
        );
        native.physical.world.insert_collider(
            ColliderBuilder::ball(0.1)
                .density(0.)
                .friction(0.4)
                .restitution(0.2),
            Some(b),
        );
        native.physical.world.bodies[b]
            .recompute_mass_properties_from_colliders(&native.physical.world.colliders);
        for _ in 0..128 {
            assert_eq!(
                generic.physical.world.bodies[a].position(),
                native.physical.world.bodies[b].position()
            );
            assert_eq!(
                generic.physical.world.bodies[a].linvel(),
                native.physical.world.bodies[b].linvel()
            );
            assert_eq!(
                generic.physical.world.bodies[a].angvel(),
                native.physical.world.bodies[b].angvel()
            );
            assert_eq!(
                generic.physical.world.bodies[a].is_sleeping(),
                native.physical.world.bodies[b].is_sleeping()
            );
            generic.physical.world.step();
            native.physical.world.step();
        }
    }

    #[test]
    fn rejected_aggregate_initial_state_preserves_world_and_generation() {
        let model = CookedRigidBody::new(aggregate_source()).unwrap();
        let mut world = WorldPhysics::new(&[], -10.);
        let before = (
            world.physical.world.bodies.len(),
            world.physical.world.colliders.len(),
            world.physical.bound_bodies.next_generation,
        );
        assert!(
            world
                .install_body(
                    BodyKey(1),
                    &model,
                    BodyState {
                        velocity_m_s: [f64::NAN, 0., 0.],
                        ..BodyState::at(Pose::default())
                    }
                )
                .is_err()
        );
        assert_eq!(
            (
                world.physical.world.bodies.len(),
                world.physical.world.colliders.len(),
                world.physical.bound_bodies.next_generation
            ),
            before
        );
        let live = world
            .install_body(BodyKey(1), &model, BodyState::at(Pose::default()))
            .unwrap();
        let state = world.body_state(live).unwrap();
        assert!(
            world
                .set_body_wrench(live, [0., f64::INFINITY, 0.], [0.; 3])
                .is_err()
        );
        assert_eq!(world.body_state(live).unwrap(), state);
        assert_eq!(
            world.physical.bound_bodies.entry(live).unwrap().force_n,
            Vector::ZERO
        );
        assert_eq!(live.generation, 0);
    }

    fn prepared_anchor() -> crate::cooking::CookedAssembly {
        use rm_sim_engine::model::{AssemblyDescription, AssemblyLink, AssemblyLinkId};
        let source = RigidBodyDescription::new(
            BodyKind::Fixed,
            vec![
                ColliderDescription::new(
                    ShapeDescription::Cuboid {
                        half_extents_m: [0.1; 3],
                    },
                    Pose::default(),
                    ContactMaterial {
                        friction: 0.4,
                        restitution: 0.,
                        memberships: u32::MAX,
                        filter: u32::MAX,
                    },
                )
                .unwrap(),
            ],
        )
        .unwrap();
        crate::cooking::CookedAssembly::new(
            AssemblyDescription::new(
                AssemblyLinkId(1),
                vec![
                    AssemblyLink::new(AssemblyLinkId(1), source, BodyState::at(Pose::default()))
                        .unwrap(),
                ],
                vec![],
            )
            .unwrap(),
        )
        .unwrap()
    }

    #[test]
    fn assembly_rejection_consumes_no_body_generation_or_native_slots() {
        use rm_sim_engine::model::AssemblyLinkId;
        let prepared = prepared_anchor();
        let keys = super::super::AssemblyKeys {
            links: vec![(AssemblyLinkId(1), BodyKey(1))],
            joints: vec![],
        };
        let mut world = WorldPhysics::new(&[], 0.);
        let before = (
            world.physical.world.bodies.len(),
            world.physical.world.colliders.len(),
        );
        assert!(
            world
                .install_assembly(&prepared, Pose::at([f64::NAN, 0., 0.]), &keys)
                .is_err()
        );
        assert_eq!(world.physical.bound_bodies.next_generation, 0);
        assert_eq!(
            (
                world.physical.world.bodies.len(),
                world.physical.world.colliders.len()
            ),
            before
        );
        world.physical.bound_bodies.next_generation = u64::MAX;
        assert!(
            world
                .install_assembly(&prepared, Pose::default(), &keys)
                .is_err()
        );
        assert_eq!(world.physical.bound_bodies.next_generation, u64::MAX);
        assert_eq!(
            (
                world.physical.world.bodies.len(),
                world.physical.world.colliders.len()
            ),
            before
        );
        world.physical.bound_bodies.next_generation = 0;
        let live = world
            .install_assembly(&prepared, Pose::default(), &keys)
            .unwrap();
        let binding = live.body(AssemblyLinkId(1)).unwrap();
        assert_eq!(binding.generation, 0);
        let mut control = WorldPhysics::new(&[], 0.);
        let direct = control
            .install_body(
                BodyKey(1),
                &prepared.bodies()[0],
                BodyState::at(Pose::default()),
            )
            .unwrap();
        assert_eq!(
            world.physical.bound_bodies.handle(binding).unwrap(),
            control.physical.bound_bodies.handle(direct).unwrap()
        );
    }

    #[test]
    fn assembly_instances_share_source_and_cooked_shapes() {
        use rm_sim_engine::model::AssemblyLinkId;
        let prepared = prepared_anchor();
        let shared = prepared.clone();
        assert_eq!(
            prepared.source().links().as_ptr(),
            shared.source().links().as_ptr()
        );
        let mut world = WorldPhysics::new(&[], 0.);
        let bindings: Vec<_> = [1, 2]
            .into_iter()
            .map(|key| {
                world
                    .install_assembly(
                        &shared,
                        Pose::at([key as f64, 0., 0.]),
                        &super::super::AssemblyKeys {
                            links: vec![(AssemblyLinkId(1), BodyKey(key))],
                            joints: vec![],
                        },
                    )
                    .unwrap()
                    .body(AssemblyLinkId(1))
                    .unwrap()
            })
            .collect();
        let shape = |binding| {
            let handle = world.physical.bound_bodies.handle(binding).unwrap();
            &world.physical.world.colliders[world.physical.world.bodies[handle].colliders()[0]]
                .shared_shape()
                .0
        };
        assert!(Arc::ptr_eq(shape(bindings[0]), shape(bindings[1])));
        let retained = world.static_geometry_snapshot(false);
        for binding in bindings {
            world.remove_bound_body(binding).unwrap();
        }
        assert!(!retained.sphere_clear([1., 0., 0.], 0.05));
    }
    #[test]
    fn installation_uses_explicit_mass_frame_and_shares_shapes() {
        let frame = Pose::yawed([0.1, 0.2, 0.3], 0.4);
        let inertia = Inertial::new(2., [2., 3., 4.], frame).unwrap();
        let source = RigidBodyDescription::new(
            BodyKind::Dynamic(inertia),
            vec![
                ColliderDescription::new(
                    ShapeDescription::Cuboid {
                        half_extents_m: [0.1; 3],
                    },
                    Pose::at([0.; 3]),
                    ContactMaterial {
                        friction: 0.4,
                        restitution: 0.2,
                        memberships: 0x1,
                        filter: 0x4,
                    },
                )
                .unwrap(),
            ],
        )
        .unwrap();
        let model = CookedRigidBody::new(source).unwrap();
        let mut a = WorldPhysics::new(&[], -10.);
        let mut b = WorldPhysics::new(&[], -10.);
        let left_binding = a
            .install_body(BodyKey(7), &model, BodyState::at(Pose::at([0.; 3])))
            .unwrap();
        let right_binding = b
            .install_body(BodyKey(7), &model, BodyState::at(Pose::at([0.; 3])))
            .unwrap();
        let left_handle = a.physical.bound_bodies.entry(left_binding).unwrap().handle;
        let right_handle = b.physical.bound_bodies.entry(right_binding).unwrap().handle;
        let ca = &a.physical.world.colliders[a.physical.world.bodies[left_handle].colliders()[0]];
        let cb = &b.physical.world.colliders[b.physical.world.bodies[right_handle].colliders()[0]];
        assert!(Arc::ptr_eq(&ca.shared_shape().0, &cb.shared_shape().0));
        assert_eq!(ca.collision_groups().memberships.bits(), 1);
        assert_eq!(ca.collision_groups().filter.bits(), 4);
        a.step(0, &TargetFrames::new(vec![])).unwrap();
        let m = &a.physical.world.bodies[left_handle]
            .mass_properties()
            .local_mprops;
        assert_eq!(a.physical.world.bodies[left_handle].mass(), 2.);
        assert!((m.local_com - vector(frame.translation_m)).length() < 1e-12);
        assert!((m.inv_principal_inertia - vector([0.5, 1. / 3., 0.25])).length() < 1e-12);
        assert!(
            (m.principal_inertia_local_frame
                .dot(pose(frame).rotation)
                .abs()
                - 1.)
                .abs()
                < 1e-12
        );
    }
}
