// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Opt-in regularized friction patches for directional rolling proxies.
use super::*;

/// An approximate regularized anisotropic friction law against fixed scenery.
/// The unit roller axis is collider-local; its projection onto each contact plane
/// is the low-grip direction. The perpendicular tangent is the tread direction.
/// Coefficients are dimensionless and independently bound tangential impulses by
/// mu times the preceding slice's native normal impulse (a rectangular limit).
/// One pressure-centered patch per collider omits distributed twisting friction.
/// A near-parallel axis/normal uses native isotropic tread friction instead.
/// This is not a model of individual rollers or measured bearing resistance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DirectionalFriction {
    axis: Vector,
    tread: f64,
    roller: f64,
    slip_damping_n_s_m: f64,
}
impl DirectionalFriction {
    /// Validate a unit local roller axis and coefficients in the inclusive 0..=10 range.
    /// Default implicit tangential damping is 500 N s/m per patch, an assumed
    /// regularization rather than a measured tyre property.
    ///
    /// ```
    /// use rm_sim_physics::runtime::DirectionalFriction;
    /// let law = DirectionalFriction::new([0., 1., 0.], 0.9, 0.02)?;
    /// assert!(DirectionalFriction::new([0.; 3], 0.9, 0.02).is_err());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn new(roller_axis: [f64; 3], tread: f64, roller: f64) -> Result<Self, &'static str> {
        let axis = vector(roller_axis);
        if !axis.is_finite()
            || (axis.length_squared() - 1.).abs() > 1e-6
            || ![tread, roller]
                .into_iter()
                .all(|v| v.is_finite() && (0. ..=10.).contains(&v))
        {
            return Err("invalid directional friction");
        }
        Ok(Self {
            axis: axis.normalize(),
            tread,
            roller,
            slip_damping_n_s_m: 500.,
        })
    }
    /// Set assumed implicit tangential damping in N s/m, inclusive 1..=1e6.
    /// Finite damping permits load-dependent slip below the Coulomb limit.
    /// Higher values approach sticking but can aggravate contact overconstraint.
    ///
    /// ```
    /// use rm_sim_physics::runtime::DirectionalFriction;
    /// let law = DirectionalFriction::new([0., 1., 0.], 0.9, 0.02)?;
    /// let compliant = law.with_slip_damping(250.)?;
    /// assert!(law.with_slip_damping(0.).is_err());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn with_slip_damping(mut self, damping_n_s_m: f64) -> Result<Self, &'static str> {
        if !damping_n_s_m.is_finite() || !(1. ..=1e6).contains(&damping_n_s_m) {
            return Err("invalid directional slip damping");
        }
        self.slip_damping_n_s_m = damping_n_s_m;
        Ok(self)
    }
}
#[derive(Default)]
pub(super) struct DirectionalContacts {
    entries: Vec<(ColliderHandle, DirectionalFriction)>,
    contacts: Vec<Constraint>,
    anchor: Option<RigidBodyHandle>,
    joints: Vec<(RigidBodyHandle, ImpulseJointHandle)>,
}
struct Constraint {
    body: RigidBodyHandle,
    collider: ColliderHandle,
    normal: Vector,
    load: f64,
    damping_n_s_m: f64,
    point: Vector,
    axes: [Vector; 2],
    limits: [f64; 2],
}
impl DirectionalContacts {
    fn selected(
        &self,
        a: ColliderHandle,
        b: ColliderHandle,
        colliders: &ColliderSet,
        bodies: &RigidBodySet,
    ) -> Option<(ColliderHandle, DirectionalFriction, bool)> {
        let fixed = |h| {
            colliders
                .get(h)
                .is_some_and(|c| c.parent().is_none_or(|p| bodies[p].is_fixed()))
        };
        self.entries.iter().find_map(|&(h, law)| {
            if h == a && fixed(b) {
                Some((h, law, true))
            } else if h == b && fixed(a) {
                Some((h, law, false))
            } else {
                None
            }
        })
    }
    pub(super) fn step(&mut self, world: &mut PhysicsWorld) {
        self.entries.retain(|(h, _)| world.colliders.contains(*h));
        if self.entries.is_empty() {
            for (_, h) in self.joints.drain(..) {
                world.impulse_joints.remove(h, true);
            }
            world.step();
            return;
        }
        self.contacts.clear();
        for pair in world.contact_pairs() {
            if !pair.has_any_active_contact() {
                continue;
            }
            let Some((h, law, first)) = self.selected(
                pair.collider1,
                pair.collider2,
                &world.colliders,
                &world.bodies,
            ) else {
                continue;
            };
            let collider = &world.colliders[h];
            let Some(body) = collider.parent() else {
                continue;
            };
            let axis = collider.position().rotation * law.axis;
            for manifold in &pair.manifolds {
                let tangent = axis - manifold.data.normal * axis.dot(manifold.data.normal);
                if tangent.length_squared() < 1e-12 {
                    continue;
                }
                let n = if first {
                    manifold.data.normal
                } else {
                    -manifold.data.normal
                };
                for solver in &manifold.data.solver_contacts {
                    let index = (solver.contact_id[0] & 0x7fff_ffff) as usize;
                    let Some(contact) = manifold.points.get(index) else {
                        continue;
                    };
                    let normal = contact.data.impulse;
                    if !normal.is_finite() || normal <= 0. {
                        continue;
                    }
                    let local = if first {
                        contact.local_p1
                    } else {
                        contact.local_p2
                    };
                    let point = collider.position().transform_point(local);
                    if let Some(patch) = self.contacts.iter_mut().find(|p| p.collider == h) {
                        let total = patch.load + normal;
                        if !total.is_finite() {
                            continue;
                        }
                        let weight = normal / total;
                        patch.point = patch.point.lerp(point, weight);
                        patch.normal = patch.normal.lerp(n, weight);
                        patch.load = total;
                    } else {
                        self.contacts.push(Constraint {
                            body,
                            collider: h,
                            point,
                            normal: n,
                            load: normal,
                            damping_n_s_m: law.slip_damping_n_s_m,
                            axes: [Vector::ZERO; 2],
                            limits: [law.tread, law.roller],
                        });
                    }
                }
            }
        }
        self.contacts.retain_mut(|patch| {
            let Some((_, law)) = self.entries.iter().find(|(h, _)| *h == patch.collider) else {
                return false;
            };
            let axis = world.colliders[patch.collider].position().rotation * law.axis;
            if patch.normal.length_squared() < 1e-16 {
                return false;
            }
            let n = patch.normal.normalize();
            let lateral = axis - n * axis.dot(n);
            if lateral.length_squared() < 1e-12 {
                return false;
            }
            let lateral = lateral.normalize();
            patch.axes = [lateral.cross(n), lateral];
            patch.limits = patch.limits.map(|mu| mu * patch.load);
            patch.point.is_finite()
                && patch
                    .limits
                    .iter()
                    .all(|v| (v / world.integration_parameters.dt).is_finite())
        });
        // Native velocity constraints couple tangential resistance to the same
        // joint solve as the suspension. Force limits use the preceding slice's
        // normal impulses: a documented one-slice load lag, not guessed weight.
        let anchor = if self.contacts.is_empty() {
            self.anchor
        } else {
            Some(
                *self
                    .anchor
                    .get_or_insert_with(|| world.bodies.insert(RigidBodyBuilder::fixed())),
            )
        };
        let dt = world.integration_parameters.dt;
        for (index, contact) in self.contacts.iter().enumerate() {
            let n = contact.axes[0].cross(contact.axes[1]);
            let rotation =
                Rotation::from_mat3(&glam::DMat3::from_cols(contact.axes[0], contact.axes[1], n));
            let frame = Pose3::from_parts(contact.point, rotation);
            let local = world.bodies[contact.body].position().inverse() * frame;
            let mut joint = GenericJointBuilder::new(JointAxesMask::empty())
                .local_frame1(frame)
                .local_frame2(local)
                .build();
            for (axis, limit) in [JointAxis::LinX, JointAxis::LinY]
                .into_iter()
                .zip(contact.limits)
            {
                joint.set_motor_model(axis, MotorModel::ForceBased);
                joint.set_motor_velocity(axis, 0., contact.damping_n_s_m);
                joint.set_motor_max_force(axis, limit / dt);
            }
            if index < self.joints.len()
                && self.joints[index].0 == contact.body
                && let Some(live) = world.impulse_joints.get_mut(self.joints[index].1, true)
            {
                live.data = joint;
            } else {
                if index < self.joints.len() {
                    world.impulse_joints.remove(self.joints[index].1, true);
                }
                let handle = world.impulse_joints.insert(
                    anchor.expect("nonempty friction patches have an anchor"),
                    contact.body,
                    joint,
                    true,
                );
                if index < self.joints.len() {
                    self.joints[index] = (contact.body, handle);
                } else {
                    self.joints.push((contact.body, handle));
                }
            }
        }
        for (_, h) in self.joints.drain(self.contacts.len()..) {
            world.impulse_joints.remove(h, true);
        }
        world.step_with_events(self, &());
    }
}
impl PhysicsHooks for DirectionalContacts {
    fn modify_solver_contacts(&self, context: &mut ContactModificationContext<'_>) {
        if let Some((h, law, _)) = self.selected(
            context.collider1,
            context.collider2,
            context.colliders,
            context.bodies,
        ) {
            let axis = context.colliders[h].position().rotation * law.axis;
            let tangent = axis - *context.normal * axis.dot(*context.normal);
            *context.friction = if tangent.length_squared() < 1e-12 {
                law.tread
            } else {
                0.
            };
        }
    }
}
impl PhysicalRuntime {
    /// Read the normal load used by an active directional friction patch, N.
    /// This privileged diagnostic is the sum of native normal-impulse magnitudes
    /// from the preceding integration slice divided by its duration. It is not a
    /// vertical load, fresh contact impulse, force sensor or independent ground
    /// truth. `None` means no supported directional patch; stale/foreign tokens
    /// return an error. See [`Self::set_directional_friction`] for a runnable setup.
    pub fn directional_normal_load_n(
        &self,
        collider: ColliderBinding,
    ) -> Result<Option<f64>, &'static str> {
        self.collider_state(collider)?;
        if !self
            .directional
            .entries
            .iter()
            .any(|(h, _)| *h == collider.handle)
        {
            return Ok(None);
        }
        Ok(self
            .directional
            .contacts
            .iter()
            .find(|c| c.collider == collider.handle)
            .map(|c| c.load / self.world.integration_parameters.dt))
    }
    /// Opt a dynamic collider into directional contact against fixed scenery.
    /// `None` restores its ordinary material law. Other dynamic/kinematic contacts
    /// retain the native isotropic law. The law is installation-local, not serialized.
    /// Native normal contacts remain unchanged. Before each integration slice,
    /// preceding normal impulses define a pressure-centered patch and independent
    /// force limits. Two implicit zero-velocity damping constraints solve alongside
    /// native joints/contacts; no target chassis velocity is prescribed.
    /// There is one slice of load lag, no tangential warm start and no twisting
    /// friction. A retained inertialess fixed anchor represents stationary ground.
    /// This experimental approximation can overconstrain transitions between
    /// simultaneous surfaces; it is not a substitute for detailed roller geometry.
    /// Reapply during recipe reset. Stale/foreign/non-dynamic bindings fail before mutation.
    ///
    /// ```
    /// use rm_sim_physics::{StepDuration, runtime::{PhysicalRuntime, BodyKey, DirectionalFriction}};
    /// # use sim_engine::{Pose, model::{BodyKind, BodyState, ColliderDescription, ContactMaterial, Inertial, RigidBodyDescription, ShapeDescription}};
    /// # use rm_sim_physics::cooking::CookedRigidBody;
    /// let mut runtime = PhysicalRuntime::new(StepDuration::HZ_1000);
    /// # let material = ContactMaterial { friction: 0.9, restitution: 0., memberships: 1, filter: 1 };
    /// # let shape = ColliderDescription::new(ShapeDescription::Cuboid { half_extents_m: [0.1; 3] }, Pose::default(), material)?;
    /// # let body = CookedRigidBody::new(RigidBodyDescription::new(BodyKind::Dynamic(Inertial::new(1., [0.01; 3], Pose::default())?), vec![shape])?)?;
    /// # let body = runtime.install_body(BodyKey(1), &body, BodyState::at(Pose::at([0., 0., 0.1])))?;
    /// # runtime.install_catch_floor(0., material)?;
    /// let participant = runtime.body_participant(body)?;
    /// let tyre = runtime.colliders().find(|&c| {
    ///     runtime.collider_state(c).is_ok_and(|s| s.participant == Some(participant))
    /// }).ok_or("missing tyre")?;
    /// runtime.set_directional_friction(tyre,
    ///     Some(DirectionalFriction::new([0., 1., 0.], 0.9, 0.02)?))?;
    /// assert_eq!(runtime.directional_normal_load_n(tyre)?, None);
    /// runtime.step(0)?;
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn set_directional_friction(
        &mut self,
        collider: ColliderBinding,
        law: Option<DirectionalFriction>,
    ) -> Result<(), &'static str> {
        self.collider_state(collider)?;
        let native = &self.world.colliders[collider.handle];
        if !native
            .parent()
            .is_some_and(|p| self.world.bodies[p].is_dynamic())
        {
            return Err("directional friction requires a dynamic collider");
        }
        self.directional
            .entries
            .retain(|(h, _)| *h != collider.handle && self.world.colliders.contains(*h));
        let native = &mut self.world.colliders[collider.handle];
        if let Some(law) = law {
            self.directional.entries.push((collider.handle, law));
            native.set_active_hooks(native.active_hooks() | ActiveHooks::MODIFY_SOLVER_CONTACTS);
        } else {
            native.set_active_hooks(native.active_hooks() - ActiveHooks::MODIFY_SOLVER_CONTACTS);
        }
        Ok(())
    }
}
