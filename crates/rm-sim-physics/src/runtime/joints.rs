// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Described impulse joints in the existing runtime, with held scalar actuation.
use super::*;
use sim_engine::model::{JointDescription, JointKind, JointMotor, JointState};
use std::f64::consts::PI;

/// Caller-selected joint identity within one runtime, independent of solver slots.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct JointKey(pub u64);

/// Opaque process-local installation token, branded by runtime and generation.
/// Removing either body or the joint invalidates it; reconstruction/reset requires
/// rebinding the logical key. This is not serializable checkpoint state.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct JointBinding {
    runtime: u64,
    key: JointKey,
    generation: u64,
}
impl JointBinding {
    /// Logical identity; the key alone does not validate an installation.
    pub fn key(self) -> JointKey {
        self.key
    }
}

struct Entry {
    key: JointKey,
    generation: u64,
    handle: ImpulseJointHandle,
    parent: RigidBodyHandle,
    child: RigidBodyHandle,
    description: JointDescription,
    motor: Option<JointMotor>,
    effort: f64,
    last_applied_effort: f64,
}

/// Retains installation order for deterministic force accumulation and removal.
#[derive(Default)]
pub(super) struct Joints {
    next_generation: u64,
    entries: Vec<Entry>,
}
impl Joints {
    pub(super) fn contains(&self, key: JointKey) -> bool {
        self.entries.iter().any(|e| e.key == key)
    }
    pub(super) fn check_capacity(&self, count: usize) -> Result<(), &'static str> {
        let count = u64::try_from(count).map_err(|_| "too many joints")?;
        self.next_generation
            .checked_add(count)
            .ok_or("joint binding generation exhausted")?;
        Ok(())
    }
    pub(super) fn check_binding(
        &self,
        runtime: u64,
        binding: JointBinding,
    ) -> Result<(), &'static str> {
        self.entry(runtime, binding).map(|_| ())
    }
    fn entry(&self, runtime: u64, binding: JointBinding) -> Result<&Entry, &'static str> {
        if binding.runtime != runtime {
            return Err("joint binding belongs to another runtime");
        }
        self.entries
            .iter()
            .find(|e| e.key == binding.key && e.generation == binding.generation)
            .ok_or("stale joint binding")
    }
    fn entry_mut(
        &mut self,
        runtime: u64,
        binding: JointBinding,
    ) -> Result<&mut Entry, &'static str> {
        self.entry(runtime, binding)?;
        Ok(self
            .entries
            .iter_mut()
            .find(|e| e.key == binding.key)
            .expect("validated binding"))
    }
    pub(super) fn remove_attached(&mut self, body: RigidBodyHandle, world: &mut PhysicsWorld) {
        self.entries.retain(|entry| {
            if entry.parent == body || entry.child == body {
                world.impulse_joints.remove(entry.handle, true);
                false
            } else {
                true
            }
        });
    }
    pub(super) fn apply_actuation(&mut self, world: &mut PhysicsWorld) {
        for entry in &mut self.entries {
            let effort = if let Some(motor) = entry.motor {
                let state = state(entry, world);
                motor_effort(motor, state)
            } else {
                entry.effort
            };
            entry.last_applied_effort = effort;
            if effort == 0. {
                continue;
            }
            let parent_frame =
                world.bodies[entry.parent].position() * pose(entry.description.parent_frame());
            let child_frame =
                world.bodies[entry.child].position() * pose(entry.description.child_frame());
            let axis = parent_frame.rotation * Vector::X;
            match entry.description.kind() {
                JointKind::Revolute => {
                    world.bodies[entry.parent].add_torque(-axis * effort, true);
                    world.bodies[entry.child].add_torque(axis * effort, true);
                }
                JointKind::Prismatic => {
                    world.bodies[entry.parent].add_force_at_point(
                        -axis * effort,
                        parent_frame.translation,
                        true,
                    );
                    world.bodies[entry.child].add_force_at_point(
                        axis * effort,
                        child_frame.translation,
                        true,
                    );
                }
                JointKind::Fixed => {}
            }
        }
    }
}

// Ordinary finite arithmetic retains its usual rounding. Overflowing errors or
// spring/damper terms are recomputed at a common scale before final saturation,
// so finite admitted commands cannot inject NaN into Rapier.
fn motor_effort(motor: JointMotor, state: JointState) -> f64 {
    let spring = motor.stiffness() * (motor.target_position() - state.position);
    let damper = motor.damping() * (motor.target_velocity() - state.velocity);
    let raw = spring + damper;
    let effort = if !raw.is_finite() {
        let gains = motor.stiffness().max(motor.damping());
        let scale = motor
            .target_position()
            .abs()
            .max(state.position.abs())
            .max(motor.target_velocity().abs())
            .max(state.velocity.abs());
        if gains == 0. || scale == 0. {
            return 0.;
        }
        let normalized = (motor.stiffness() / gains)
            * (motor.target_position() / scale - state.position / scale)
            + (motor.damping() / gains)
                * (motor.target_velocity() / scale - state.velocity / scale);
        // Multiply by the smaller factor first, avoiding an overflowing
        // intermediate when the saturated result is representable.
        (normalized * gains.min(scale)) * gains.max(scale)
    } else {
        raw
    };
    effort.clamp(-motor.max_force(), motor.max_force())
}

fn angle(parent: Pose3, child: Pose3) -> f64 {
    let rotation = parent.rotation.inverse() * child.rotation;
    // Match Rapier's principal hinge coordinate, including quaternion sign.
    let sign = if parent.rotation.dot(child.rotation) < 0. {
        -1.
    } else {
        1.
    };
    sign * 2. * rotation.x.clamp(-1., 1.).asin()
}
fn state(entry: &Entry, world: &PhysicsWorld) -> JointState {
    state_for_bodies(
        &entry.description,
        &world.bodies[entry.parent],
        &world.bodies[entry.child],
    )
}
fn state_for_bodies(description: &JointDescription, a: &RigidBody, b: &RigidBody) -> JointState {
    let parent = a.position() * pose(description.parent_frame());
    let child = b.position() * pose(description.child_frame());
    let axis = parent.rotation * Vector::X;
    match description.kind() {
        JointKind::Fixed => JointState {
            position: 0.,
            velocity: 0.,
        },
        JointKind::Revolute => JointState {
            position: angle(parent, child),
            velocity: (b.angvel() - a.angvel()).dot(axis),
        },
        JointKind::Prismatic => {
            let delta = child.translation - parent.translation;
            JointState {
                position: delta.dot(axis),
                velocity: (b.velocity_at_point(child.translation)
                    - a.velocity_at_point(parent.translation))
                .dot(axis)
                    + delta.dot(a.angvel().cross(axis)),
            }
        }
    }
}

impl PhysicalRuntime {
    /// Install a described impulse joint between distinct, live bound bodies.
    /// At least one body must be dynamic: Rapier cannot enforce a constraint
    /// between two prescribed/fixed bodies. Initial locked coordinates must agree
    /// within 1e-6 m/rad and locked relative velocities within 1e-6 m/s or rad/s.
    /// Nonzero free position/speed is supported; position must satisfy limits.
    /// Validation precedes mutation. The free axis is parent-joint +X, with
    /// positive child-relative displacement and right-hand angular direction.
    ///
    /// ```
    /// use sim_engine::{Pose, model::{BodyKind, BodyState, Inertial,
    ///     RigidBodyDescription, JointDescription, JointKind}};
    /// use rm_sim_physics::{cooking::CookedRigidBody,
    ///     runtime::{WorldPhysics, BodyKey, JointKey}};
    /// let mut world = WorldPhysics::new(&[], -10.);
    /// let fixed = CookedRigidBody::new(RigidBodyDescription::new(BodyKind::Fixed, vec![])?)?;
    /// let dynamic = CookedRigidBody::new(RigidBodyDescription::new(
    ///     BodyKind::Dynamic(Inertial::new(1., [1.; 3], Pose::default())?), vec![])?)?;
    /// let a = world.install_body(BodyKey(1), &fixed, BodyState::at(Pose::default()))?;
    /// let b = world.install_body(BodyKey(2), &dynamic, BodyState::at(Pose::default()))?;
    /// let source = JointDescription::new(JointKind::Revolute,
    ///     Pose::default(), Pose::default(), None, false)?;
    /// let joint = world.install_joint(JointKey(1), a, b, &source)?;
    /// assert_eq!(world.joint_state(joint)?.position, 0.);
    /// world.remove_bound_body(b)?;
    /// assert!(world.joint_state(joint).is_err());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn install_joint(
        &mut self,
        key: JointKey,
        parent: BodyBinding,
        child: BodyBinding,
        description: &JointDescription,
    ) -> Result<JointBinding, &'static str> {
        let parent = self.bound_bodies.handle(parent)?;
        let child = self.bound_bodies.handle(child)?;
        if parent == child {
            return Err("joint requires distinct bodies");
        }
        if self.bound_joints.entries.iter().any(|e| e.key == key) {
            return Err("duplicate joint key");
        }
        self.bound_joints.check_capacity(1)?;
        let joint = prepare_joint(
            &self.world.bodies[parent],
            &self.world.bodies[child],
            description,
        )?;
        Ok(self.commit_joint(key, parent, child, description, joint))
    }

    pub(super) fn commit_joint(
        &mut self,
        key: JointKey,
        parent: RigidBodyHandle,
        child: RigidBodyHandle,
        description: &JointDescription,
        joint: GenericJoint,
    ) -> JointBinding {
        let generation = self.bound_joints.next_generation + 1;
        let handle = self.world.impulse_joints.insert(parent, child, joint, true);
        self.bound_joints.entries.push(Entry {
            key,
            generation,
            handle,
            parent,
            child,
            description: *description,
            motor: None,
            effort: 0.,
            last_applied_effort: 0.,
        });
        self.bound_joints.next_generation = generation;
        self.primed = false;
        JointBinding {
            runtime: self.runtime_identity(),
            key,
            generation,
        }
    }

    /// Acquire the current world-scoped installation token for a logical key.
    pub fn joint_binding(&self, key: JointKey) -> Option<JointBinding> {
        self.bound_joints
            .entries
            .iter()
            .find(|e| e.key == key)
            .map(|e| JointBinding {
                runtime: self.runtime_identity(),
                key,
                generation: e.generation,
            })
    }

    /// Borrow the immutable installed source; stale/foreign bindings fail.
    pub fn joint_description(
        &self,
        binding: JointBinding,
    ) -> Result<&JointDescription, &'static str> {
        Ok(&self
            .bound_joints
            .entry(self.runtime_identity(), binding)?
            .description)
    }

    /// Read a compact scalar observation without allocating. Fixed joints report
    /// zero; revolute angles are principal values in [-pi, pi], without turns.
    pub fn joint_state(&self, binding: JointBinding) -> Result<JointState, &'static str> {
        Ok(state(
            self.bound_joints.entry(self.runtime_identity(), binding)?,
            &self.world,
        ))
    }

    /// Actuator effort applied in the last completed solver slice: N m for a
    /// revolute joint, N for a prismatic joint. Initially zero. Setting a command
    /// does not change this observation until stepping; clearing a command likewise
    /// leaves the previous observation until the next slice.
    ///
    /// This is held effort or the saturated PD output computed at slice start,
    /// not a measured net torque, constraint reaction, tick average, or electrical
    /// motor estimate. External loads and hard-stop reactions are excluded.
    ///
    /// ```
    /// use sim_engine::{Pose, model::{BodyKind, BodyState, Inertial,
    ///     RigidBodyDescription, JointDescription, JointKind}};
    /// use rm_sim_physics::{StepDuration, cooking::CookedRigidBody,
    ///     runtime::{PhysicalRuntime, BodyKey, JointKey}};
    /// # fn main() -> Result<(), &'static str> {
    /// let mut runtime = PhysicalRuntime::new(StepDuration::HZ_1000);
    /// let fixed = CookedRigidBody::new(RigidBodyDescription::new(BodyKind::Fixed, vec![])?)?;
    /// let rotor = CookedRigidBody::new(RigidBodyDescription::new(
    ///     BodyKind::Dynamic(Inertial::new(1., [0.02; 3], Pose::default())?), vec![])?)?;
    /// let a = runtime.install_body(BodyKey(1), &fixed, BodyState::at(Pose::default()))?;
    /// let b = runtime.install_body(BodyKey(2), &rotor, BodyState::at(Pose::default()))?;
    /// let joint = runtime.install_joint(JointKey(1), a, b, &JointDescription::new(
    ///     JointKind::Revolute, Pose::default(), Pose::default(), None, false)?)?;
    /// runtime.set_joint_effort(joint, 0.2)?;
    /// assert_eq!(runtime.joint_applied_effort(joint)?, 0.);
    /// runtime.step(0)?;
    /// assert_eq!(runtime.joint_applied_effort(joint)?, 0.2);
    /// # Ok(()) }
    /// ```
    pub fn joint_applied_effort(&self, binding: JointBinding) -> Result<f64, &'static str> {
        Ok(self
            .bound_joints
            .entry(self.runtime_identity(), binding)?
            .last_applied_effort)
    }

    /// Replace held effort with an optional held spring/damper motor. Each solver
    /// slice applies `clamp(k*(target-position) + d*(target_velocity-velocity),
    /// -max_force, max_force)` as N (prismatic) or N m (revolute).
    /// `None` clears both modes. Fixed joints reject motors. Targets outside
    /// inclusive limits fail; even unlimited revolute targets must be in [-pi, pi].
    /// Angular errors use ordinary principal-coordinate subtraction, not a
    /// shortest-path wrap or multi-turn target. No time advances here.
    pub fn set_joint_motor(
        &mut self,
        binding: JointBinding,
        motor: Option<JointMotor>,
    ) -> Result<(), &'static str> {
        let runtime = self.runtime_identity();
        let entry = self.bound_joints.entry_mut(runtime, binding)?;
        if let Some(motor) = motor {
            if entry.description.kind() == JointKind::Fixed {
                return Err("fixed joint cannot have a motor");
            }
            if entry.description.kind() == JointKind::Revolute
                && !(-PI..=PI).contains(&motor.target_position())
            {
                return Err("revolute motor target must lie within [-pi, pi]");
            }
            if entry.description.limits().is_some_and(|l| {
                motor.target_position() < l.min() || motor.target_position() > l.max()
            }) {
                return Err("joint motor target outside limits");
            }
        }
        entry.motor = motor;
        entry.effort = 0.;
        Ok(())
    }

    /// Replace the motor with held signed force in N (prismatic) or torque in
    /// N m (revolute), applied every slice along the current parent-joint +X.
    /// Equal/opposite forces act at the two anchors, including COM torque;
    /// hinge torques are equal/opposite world torques. Zero clears both modes.
    /// Fixed joints reject nonzero effort; nonfinite effort fails before mutation.
    pub fn set_joint_effort(
        &mut self,
        binding: JointBinding,
        effort: f64,
    ) -> Result<(), &'static str> {
        if !effort.is_finite() {
            return Err("nonfinite joint effort");
        }
        let runtime = self.runtime_identity();
        let entry = self.bound_joints.entry_mut(runtime, binding)?;
        if entry.description.kind() == JointKind::Fixed && effort != 0. {
            return Err("fixed joint cannot have effort");
        }
        entry.motor = None;
        entry.effort = effort;
        Ok(())
    }

    /// Remove this constraint and its held actuation, invalidating its token.
    /// Logical key reuse creates a fresh generation; body bindings remain valid.
    pub fn remove_bound_joint(&mut self, binding: JointBinding) -> Result<(), &'static str> {
        self.bound_joints.entry(self.runtime_identity(), binding)?;
        let index = self
            .bound_joints
            .entries
            .iter()
            .position(|e| e.key == binding.key)
            .expect("validated binding");
        let entry = self.bound_joints.entries.remove(index);
        self.world.impulse_joints.remove(entry.handle, true);
        self.primed = false;
        Ok(())
    }
}

fn validate_initial(
    description: &JointDescription,
    parent: &RigidBody,
    child: &RigidBody,
) -> Result<(), &'static str> {
    let a = parent.position() * pose(description.parent_frame());
    let b = child.position() * pose(description.child_frame());
    let rotation = a.rotation.inverse() * b.rotation;
    let offset = a.inverse_transform_point(b.translation);
    let axis = a.rotation * Vector::X;
    let angular = child.angvel() - parent.angvel();
    let linear = child.velocity_at_point(b.translation) - parent.velocity_at_point(a.translation);
    let (translation_error, rotation_error, linear_error, angular_error) = match description.kind()
    {
        JointKind::Fixed => (
            offset.length(),
            Vector::new(rotation.x, rotation.y, rotation.z).length(),
            linear.length(),
            angular.length(),
        ),
        JointKind::Revolute => (
            offset.length(),
            Vector::new(0., rotation.y, rotation.z).length(),
            linear.length(),
            (angular - axis * angular.dot(axis)).length(),
        ),
        JointKind::Prismatic => {
            // Frame separation rotates with the parent even on a valid slider.
            let relative = linear - parent.angvel().cross(b.translation - a.translation);
            (
                Vector::new(0., offset.y, offset.z).length(),
                Vector::new(rotation.x, rotation.y, rotation.z).length(),
                (relative - axis * relative.dot(axis)).length(),
                angular.length(),
            )
        }
    };
    if [
        translation_error,
        rotation_error * 2.,
        linear_error,
        angular_error,
    ]
    .into_iter()
    .any(|x| !x.is_finite() || x > 1e-6)
    {
        return Err("initial joint locked frames or velocities disagree");
    }
    let position = state_for_bodies(description, parent, child).position;
    if description
        .limits()
        .is_some_and(|l| position < l.min() || position > l.max())
    {
        return Err("initial joint position outside limits");
    }
    Ok(())
}

/// Validate native staged bodies and construct a constraint without world mutation.
pub(super) fn prepare_joint(
    parent: &RigidBody,
    child: &RigidBody,
    description: &JointDescription,
) -> Result<GenericJoint, &'static str> {
    if !parent.is_dynamic() && !child.is_dynamic() {
        return Err("joint requires at least one dynamic body");
    }
    validate_initial(description, parent, child)?;
    let (mask, axis) = match description.kind() {
        JointKind::Fixed => (JointAxesMask::LOCKED_FIXED_AXES, JointAxis::LinX),
        JointKind::Revolute => (JointAxesMask::LOCKED_REVOLUTE_AXES, JointAxis::AngX),
        JointKind::Prismatic => (JointAxesMask::LOCKED_PRISMATIC_AXES, JointAxis::LinX),
    };
    let mut joint = GenericJointBuilder::new(mask)
        .local_frame1(pose(description.parent_frame()))
        .local_frame2(pose(description.child_frame()))
        .contacts_enabled(description.contacts_enabled());
    if let Some(limits) = description.limits() {
        joint = joint.limits(axis, [limits.min(), limits.max()]);
    }
    Ok(joint.build())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn assembly_joint_capacity_failure_precedes_all_body_and_joint_mutations() {
        use sim_engine::model::{
            AssemblyDescription, AssemblyJoint, AssemblyJointId, AssemblyLink, AssemblyLinkId,
            BodyKind, Inertial, RigidBodyDescription,
        };
        let source = AssemblyDescription::new(
            AssemblyLinkId(1),
            vec![
                AssemblyLink::new(
                    AssemblyLinkId(1),
                    RigidBodyDescription::new(BodyKind::Fixed, vec![]).unwrap(),
                    BodyState::at(Pose::default()),
                )
                .unwrap(),
                AssemblyLink::new(
                    AssemblyLinkId(2),
                    RigidBodyDescription::new(
                        BodyKind::Dynamic(Inertial::new(1., [1.; 3], Pose::default()).unwrap()),
                        vec![],
                    )
                    .unwrap(),
                    BodyState::at(Pose::default()),
                )
                .unwrap(),
            ],
            vec![AssemblyJoint::new(
                AssemblyJointId(1),
                AssemblyLinkId(1),
                AssemblyLinkId(2),
                JointDescription::new(
                    JointKind::Revolute,
                    Pose::default(),
                    Pose::default(),
                    None,
                    false,
                )
                .unwrap(),
            )],
        )
        .unwrap();
        let model = crate::cooking::CookedAssembly::new(source).unwrap();
        let keys = super::super::AssemblyKeys {
            links: vec![
                (AssemblyLinkId(1), BodyKey(1)),
                (AssemblyLinkId(2), BodyKey(2)),
            ],
            joints: vec![(AssemblyJointId(1), JointKey(1))],
        };
        let mut world = WorldPhysics::new(&[], 0.);
        world.physical.bound_joints.next_generation = u64::MAX;
        assert!(
            world
                .install_assembly(&model, Pose::default(), &keys)
                .is_err()
        );
        assert_eq!(world.physical.world.bodies.len(), 0);
        assert_eq!(world.physical.world.colliders.len(), 1);
        assert_eq!(world.physical.world.impulse_joints.len(), 0);
        assert_eq!(world.physical.bound_joints.next_generation, u64::MAX);
        assert!(world.body_binding(BodyKey(1)).is_none());
        world.physical.bound_joints.next_generation = 0;
        let live = world
            .install_assembly(&model, Pose::default(), &keys)
            .unwrap();
        assert_eq!(live.joint(AssemblyJointId(1)).unwrap().generation, 1);
    }

    #[test]
    fn extreme_finite_motor_arithmetic_saturates_without_nan_or_spurious_overflow() {
        let state = JointState {
            position: -f64::MAX,
            velocity: 0.,
        };
        let motor = JointMotor::new(f64::MAX, 0., 1e-308, 0., 10.).unwrap();
        let effort = motor_effort(motor, state);
        assert!((effort - 2. * (f64::MAX * 1e-308)).abs() < 1e-14);
        let motor = JointMotor::new(f64::MAX, 1., 0., 1., 10.).unwrap();
        assert!((motor_effort(motor, state) - 1.).abs() < 1e-14);
        let state = JointState {
            position: 0.,
            velocity: 0.,
        };
        let motor = JointMotor::new(2., -2., f64::MAX, f64::MAX, 10.).unwrap();
        assert_eq!(motor_effort(motor, state), 0.);
        for (target, velocity, expected) in [(4., -2., 10.), (-4., 2., -10.)] {
            let motor = JointMotor::new(target, velocity, f64::MAX, f64::MAX, 10.).unwrap();
            assert_eq!(motor_effort(motor, state), expected);
        }
    }
}
