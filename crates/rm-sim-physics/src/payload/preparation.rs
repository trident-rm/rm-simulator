// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Immutable payload resources without world handles or attachment state.
use super::PayloadShape;
use crate::{Pose, cooking::CookedRigidBody};
use sim_engine::model::{
    BodyKind, BodyPolicy, ColliderDescription, ColliderMass, ColliderPolicy, CompoundShape,
    ContactMaterial, ConvexHull, RigidBodyDescription, ShapeDescription,
};
use std::sync::Arc;

/// Reusable source envelopes and cooked native compound, independent of a world.
/// Clones share source hull buffers and native shapes; each installation has its
/// own identity, pose, motion, attachment state and contact damping.
#[derive(Clone)]
pub struct PreparedPayload {
    shape: Arc<PayloadShape>,
    cooked: CookedRigidBody,
}
impl PreparedPayload {
    /// Validate and cook an owned payload without changing source point order,
    /// fitting hulls again, or allocating any live world handles.
    ///
    /// ```
    /// use sim_engine::model::BodyState;
    /// use rm_sim_physics::{Pose, WorldPhysics, manipulator_collision::CollisionPart,
    ///     projectile::{PayloadShape, PreparedPayload}};
    /// let shape = PayloadShape::from_parts(&[CollisionPart::Box { half_m: [0.03; 3] }], 0.25)?;
    /// let prepared = PreparedPayload::new(shape)?;
    /// let mut world = WorldPhysics::new(&[], 0.);
    /// world.add_prepared_payload(1, &prepared, BodyState::at(Pose::at([0., 0., 1.])), false)?;
    /// assert!(std::ptr::eq(world.payload_telemetry(1).unwrap().shape, prepared.shape()));
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn new(shape: PayloadShape) -> Result<Self, &'static str> {
        let cooked = cook(&shape)?;
        Ok(Self {
            shape: Arc::new(shape),
            cooked,
        })
    }
    /// Original immutable payload envelopes, mass in kilograms and expansion in metres.
    pub fn shape(&self) -> &PayloadShape {
        &self.shape
    }
    /// Shared body description and native shapes, with no instance state or handles.
    pub fn cooked(&self) -> &CookedRigidBody {
        &self.cooked
    }
    pub(super) fn shared_shape(&self) -> Arc<PayloadShape> {
        Arc::clone(&self.shape)
    }
}

pub(super) fn cook(shape: &PayloadShape) -> Result<CookedRigidBody, &'static str> {
    if !shape.max_expansion_m.is_finite()
        || !(0.0..=0.001000001).contains(&shape.max_expansion_m)
        || !shape.mass_kg.is_finite()
        || shape.mass_kg <= 0.
        || shape.hulls_m.is_empty()
        || shape.hulls_m.len() > 32
        || shape.hulls_m.iter().map(Vec::len).sum::<usize>() > 8192
    {
        return Err("invalid payload shape");
    }
    let children = shape
        .hulls_m
        .iter()
        .map(|hull| {
            Ok((
                Pose::default(),
                ShapeDescription::ConvexHull(ConvexHull::new(hull.clone())?),
            ))
        })
        .collect::<Result<Vec<_>, &'static str>>()?;
    let collider = ColliderDescription::new(
        ShapeDescription::Compound(CompoundShape::new(children)?),
        Pose::default(),
        ContactMaterial {
            friction: 0.6,
            restitution: 0.05,
            memberships: u32::MAX,
            filter: u32::MAX,
        },
    )?
    .with_policy(ColliderPolicy {
        mass: ColliderMass::MassKg(shape.mass_kg),
        ..Default::default()
    })?;
    let source = RigidBodyDescription::new(BodyKind::DynamicFromColliders, vec![collider])?
        .with_policy(BodyPolicy {
            ccd_enabled: true,
            ..Default::default()
        })?;
    CookedRigidBody::new(source)
}
