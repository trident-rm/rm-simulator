// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Prepare, preflight and commit complete source assembly installations.
use super::*;
use crate::cooking::CookedAssembly;
use sim_engine::model::{AssemblyJointId, AssemblyLinkId};

/// Complete caller-selected mapping from source IDs to world-local keys.
/// Link/joint namespaces are separate. Installation rejects missing, extra,
/// duplicate source IDs, duplicate destination keys and keys already in use.
#[derive(Clone, Debug)]
pub struct AssemblyKeys {
    /// One runtime body key for each source link, in any order.
    pub links: Vec<(AssemblyLinkId, BodyKey)>,
    /// One runtime joint key for each source joint, in any order.
    pub joints: Vec<(AssemblyJointId, JointKey)>,
}

/// Opaque immutable source-to-live bindings in source array order. Dropping this
/// record does not remove bodies. Removal/reset invalidates its individual tokens;
/// reacquire keys after reconstruction. This is not a serialized checkpoint.
#[derive(Clone, Debug)]
pub struct AssemblyBinding {
    links: Vec<(AssemblyLinkId, BodyBinding)>,
    joints: Vec<(AssemblyJointId, JointBinding)>,
}
impl AssemblyBinding {
    /// Find the live body installation for a source link; no world validation.
    pub fn body(&self, id: AssemblyLinkId) -> Option<BodyBinding> {
        self.links
            .iter()
            .find(|(source, _)| *source == id)
            .map(|(_, binding)| *binding)
    }
    /// Find the live joint installation for a source joint; no world validation.
    pub fn joint(&self, id: AssemblyJointId) -> Option<JointBinding> {
        self.joints
            .iter()
            .find(|(source, _)| *source == id)
            .map(|(_, binding)| *binding)
    }
    /// Borrow source-ordered body bindings without allocating.
    pub fn links(&self) -> &[(AssemblyLinkId, BodyBinding)] {
        &self.links
    }
    /// Borrow source-ordered joint bindings without allocating.
    pub fn joints(&self) -> &[(AssemblyJointId, JointBinding)] {
        &self.joints
    }
}

struct Staged {
    bodies: Vec<RigidBody>,
    joints: Vec<GenericJoint>,
}

/// Native staged bodies have explicit COMs, but no native handles or colliders.
/// Their attached colliders have zero density, so installation cannot change mass.
fn stage(model: &CookedAssembly, placement: Pose) -> Result<Staged, &'static str> {
    BodyState::at(placement).validate()?;
    let placement = pose(placement);
    let bodies = model
        .source()
        .links()
        .iter()
        .zip(model.bodies())
        .map(|(link, cooked)| {
            let initial = link.initial();
            let transformed = placement * pose(initial.pose);
            let state = BodyState {
                pose: Pose {
                    translation_m: transformed.translation.to_array(),
                    rotation_wxyz: [
                        transformed.rotation.w,
                        transformed.rotation.x,
                        transformed.rotation.y,
                        transformed.rotation.z,
                    ],
                },
                velocity_m_s: (placement.rotation * vector(initial.velocity_m_s)).to_array(),
                angular_velocity_rad_s: (placement.rotation
                    * vector(initial.angular_velocity_rad_s))
                .to_array(),
            };
            bindings::prepare_body(cooked, state)
        })
        .collect::<Result<Vec<_>, _>>()?;
    let joints = model
        .source()
        .joints()
        .iter()
        .zip(model.endpoints())
        .map(|(joint, &(parent, child))| {
            joints::prepare_joint(&bodies[parent], &bodies[child], joint.description())
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok(Staged { bodies, joints })
}

pub(crate) fn validate_cooked(model: &CookedAssembly) -> Result<(), &'static str> {
    stage(model, Pose::default()).map(|_| ())
}

impl PhysicalRuntime {
    /// Install all bodies/joints under explicit keys at an assembly-to-world pose.
    /// Source poses are in one common assembly frame; placement rotates COM and
    /// angular velocities without adding platform velocity. All recoverable errors
    /// are checked before mutation, including transformed native joint conditions,
    /// duplicate keys and binding-generation capacity. No trial world or rollback
    /// consumes handles. Allocation failure follows ordinary Rust process behavior.
    /// Cooked resources are shared; installation stages O(links + joints) native
    /// state and commits in source order. It introduces no per-tick processing.
    ///
    /// ```
    /// use sim_engine::{Pose, model::{AssemblyDescription, AssemblyLink,
    ///     AssemblyLinkId, BodyKind, BodyState, RigidBodyDescription}};
    /// use sim_physics::{cooking::CookedAssembly,
    ///     runtime::{PhysicalRuntime, AssemblyKeys, BodyKey}};
    /// let id = AssemblyLinkId(4);
    /// let source = AssemblyDescription::new(id, vec![AssemblyLink::new(id,
    ///     RigidBodyDescription::new(BodyKind::Fixed, vec![])?,
    ///     BodyState::at(Pose::default()))?], vec![])?;
    /// let prepared = CookedAssembly::new(source)?;
    /// let keys = AssemblyKeys { links: vec![(id, BodyKey(20))], joints: vec![] };
    /// let mut world = PhysicalRuntime::new(sim_physics::StepDuration::HZ_128);
    /// let live = world.install_assembly(&prepared, Pose::at([1., 0., 0.]), &keys)?;
    /// assert_eq!(world.body_state(live.body(id).unwrap())?.pose.translation_m, [1., 0., 0.]);
    /// world.remove_assembly(&live)?;
    /// assert!(world.body_state(live.body(id).unwrap()).is_err());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn install_assembly(
        &mut self,
        model: &CookedAssembly,
        placement: Pose,
        keys: &AssemblyKeys,
    ) -> Result<AssemblyBinding, &'static str> {
        let links: HashMap<_, _> = keys.links.iter().copied().collect();
        let joints: HashMap<_, _> = keys.joints.iter().copied().collect();
        if links.len() != keys.links.len()
            || links.len() != model.source().links().len()
            || joints.len() != keys.joints.len()
            || joints.len() != model.source().joints().len()
        {
            return Err("assembly requires each source ID exactly once");
        }
        let link_keys = model
            .source()
            .links()
            .iter()
            .map(|link| {
                links
                    .get(&link.id())
                    .copied()
                    .ok_or("missing source link mapping")
            })
            .collect::<Result<Vec<_>, _>>()?;
        let joint_keys = model
            .source()
            .joints()
            .iter()
            .map(|joint| {
                joints
                    .get(&joint.id())
                    .copied()
                    .ok_or("missing source joint mapping")
            })
            .collect::<Result<Vec<_>, _>>()?;
        let unique_links: std::collections::HashSet<_> = link_keys.iter().copied().collect();
        let unique_joints: std::collections::HashSet<_> = joint_keys.iter().copied().collect();
        if unique_links.len() != link_keys.len()
            || unique_joints.len() != joint_keys.len()
            || link_keys.iter().any(|key| self.bound_bodies.contains(*key))
            || joint_keys
                .iter()
                .any(|key| self.bound_joints.contains(*key))
        {
            return Err("duplicate or occupied assembly runtime key");
        }
        self.bound_bodies.check_capacity(link_keys.len())?;
        self.bound_joints.check_capacity(joint_keys.len())?;
        let staged = stage(model, placement)?;
        let mut live = AssemblyBinding {
            links: Vec::with_capacity(link_keys.len()),
            joints: Vec::with_capacity(joint_keys.len()),
        };
        for (((link, cooked), key), body) in model
            .source()
            .links()
            .iter()
            .zip(model.bodies())
            .zip(link_keys)
            .zip(staged.bodies)
        {
            live.links
                .push((link.id(), self.commit_body(key, cooked, body)));
        }
        for (((joint, &(parent, child)), key), native) in model
            .source()
            .joints()
            .iter()
            .zip(model.endpoints())
            .zip(joint_keys)
            .zip(staged.joints)
        {
            let parent = self
                .bound_bodies
                .handle(live.links[parent].1)
                .expect("preflighted body");
            let child = self
                .bound_bodies
                .handle(live.links[child].1)
                .expect("preflighted body");
            live.joints.push((
                joint.id(),
                self.commit_joint(key, parent, child, joint.description(), native),
            ));
        }
        Ok(live)
    }

    /// Remove all mapped installations after validating every token. If any part
    /// is stale/foreign, nothing is removed. Other joints attached externally to
    /// these bodies are removed by ordinary body-removal cascading. Source IDs
    /// and cloned binding records do not keep removed native objects alive.
    pub fn remove_assembly(&mut self, live: &AssemblyBinding) -> Result<(), &'static str> {
        for &(_, binding) in &live.links {
            self.bound_bodies.handle(binding)?;
        }
        for &(_, binding) in &live.joints {
            self.bound_joints
                .check_binding(self.runtime_identity(), binding)?;
        }
        for &(_, binding) in &live.joints {
            self.remove_bound_joint(binding).expect("preflighted joint");
        }
        for &(_, binding) in &live.links {
            self.remove_bound_body(binding).expect("preflighted body");
        }
        Ok(())
    }
}
