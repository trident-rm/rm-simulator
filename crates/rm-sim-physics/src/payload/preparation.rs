// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Immutable payload resources without world handles or attachment state.
use super::PayloadShape;
use crate::{Pose, cooking::CookedRigidBody};
use sim_engine::model::{
    BodyKind, BodyPolicy, BodyState, ColliderDescription, ColliderMass, ColliderPolicy,
    CompoundShape, ContactMaterial, ConvexHull, RigidBodyDescription, ShapeDescription,
};
use std::{cell::RefCell, collections::VecDeque, sync::Arc};

// Each native replay thread and browser worker owns a bounded resource cache.
// Only immutable source descriptions/native shapes are retained; no world handles,
// body motion or checkpoint authority enters this cache.
const CACHE_CAPACITY: usize = 16;
#[derive(Default)]
struct PayloadCache(VecDeque<PreparedPayload>);
impl PayloadCache {
    fn prepare(&mut self, shape: &PayloadShape) -> Result<PreparedPayload, &'static str> {
        if let Some(index) = self
            .0
            .iter()
            .position(|entry| exact_shape(entry.shape(), shape))
        {
            let prepared = self.0.remove(index).expect("located cache entry");
            self.0.push_front(prepared.clone());
            return Ok(prepared);
        }
        // Validation/cooking precedes insertion, so rejected shapes never enter
        // the cache. Every scalar, vertex, hull boundary and source order is key.
        let prepared = PreparedPayload::new(shape.clone())?;
        if self.0.len() == CACHE_CAPACITY {
            self.0.pop_back();
        }
        self.0.push_front(prepared.clone());
        Ok(prepared)
    }
}
fn exact_shape(a: &PayloadShape, b: &PayloadShape) -> bool {
    a.mass_kg.to_bits() == b.mass_kg.to_bits()
        && a.max_expansion_m.to_bits() == b.max_expansion_m.to_bits()
        && a.hulls_m.len() == b.hulls_m.len()
        && a.hulls_m.iter().zip(&b.hulls_m).all(|(a, b)| {
            a.len() == b.len()
                && a.iter()
                    .zip(b)
                    .all(|(a, b)| a.iter().zip(b).all(|(a, b)| a.to_bits() == b.to_bits()))
        })
}
thread_local! {
    static CACHE: RefCell<PayloadCache> = RefCell::new(PayloadCache::default());
}
pub(super) fn cached(shape: &PayloadShape) -> Result<PreparedPayload, &'static str> {
    CACHE.with(|cache| cache.borrow_mut().prepare(shape))
}

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
    /// Validate motion and placed mass frame without installing a live body.
    /// Batch callers can preflight every placement before mutating their world.
    ///
    /// # Errors
    /// Rejects invalid motion or a nonfinite transformed mass frame.
    pub fn validate_placement(&self, state: BodyState) -> Result<(), &'static str> {
        self.body_at(state).map(|_| ())
    }
    pub(super) fn body_at(
        &self,
        state: BodyState,
    ) -> Result<rapier3d_f64::prelude::RigidBody, &'static str> {
        use rapier3d_f64::prelude::Vector;
        use sim_physics::backend::CookedBodyExt;
        state.validate()?;
        if !super::pose_is_valid(state.pose) {
            return Err("invalid payload motion");
        }
        let body = self
            .cooked
            .body_builder()
            .pose(super::pose(state.pose))
            .linvel(Vector::from_array(state.velocity_m_s))
            .angvel(Vector::from_array(state.angular_velocity_rad_s))
            .build();
        if self
            .cooked
            .aggregate_mass_properties()
            .is_some_and(|mass| !body.position().transform_point(mass.local_com).is_finite())
        {
            return Err("nonfinite placed payload mass frame");
        }
        Ok(body)
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
    Ok(CookedRigidBody::new(source)?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::manipulator_collision::CollisionPart;
    use sim_physics::backend::CookedBodyExt;

    fn shape() -> PayloadShape {
        PayloadShape::from_parts(&[CollisionPart::Box { half_m: [0.03; 3] }], 0.25).unwrap()
    }
    fn shared(a: &PreparedPayload, b: &PreparedPayload) -> bool {
        Arc::ptr_eq(&a.cooked.shapes()[0].0, &b.cooked.shapes()[0].0)
    }
    #[test]
    fn cache_keys_every_source_value_and_preserves_order_and_mass() {
        let mut cache = PayloadCache::default();
        let source = shape();
        let first = cache.prepare(&source).unwrap();
        assert!(shared(&first, &cache.prepare(&source).unwrap()));
        let mut changed = source.clone();
        changed.mass_kg = 0.5;
        let heavier = cache.prepare(&changed).unwrap();
        assert!(!shared(&first, &heavier));
        assert_eq!(heavier.shape().mass_kg, 0.5);
        assert_ne!(
            first.cooked.aggregate_mass_properties(),
            heavier.cooked.aggregate_mass_properties()
        );
        changed = source.clone();
        changed.hulls_m[0][0][0] += 0.001;
        assert!(!shared(&first, &cache.prepare(&changed).unwrap()));
        changed = source.clone();
        changed.hulls_m[0].reverse();
        let reordered = cache.prepare(&changed).unwrap();
        assert!(!shared(&first, &reordered));
        assert_eq!(reordered.shape(), &changed);
        changed = source.clone();
        changed.max_expansion_m += 1e-6;
        assert!(!shared(&first, &cache.prepare(&changed).unwrap()));
    }
    #[test]
    fn cache_is_bounded_lru_and_rejects_invalid_shapes_without_eviction() {
        let mut cache = PayloadCache::default();
        let mut source = shape();
        let oldest = cache.prepare(&source).unwrap();
        for i in 1..CACHE_CAPACITY {
            source.mass_kg = i as f64 + 1.;
            cache.prepare(&source).unwrap();
        }
        let newest = cache.prepare(&source).unwrap();
        let second = cache.0[CACHE_CAPACITY - 2].clone();
        let before = cache.0.len();
        for invalid in [f64::NAN, f64::INFINITY, 0.] {
            let mut invalid_shape = source.clone();
            invalid_shape.mass_kg = invalid;
            assert!(cache.prepare(&invalid_shape).is_err());
        }
        let mut invalid = source.clone();
        invalid.hulls_m[0][0][0] = f64::NAN;
        assert!(cache.prepare(&invalid).is_err());
        assert_eq!(cache.0.len(), before);
        assert!(shared(&newest, &cache.prepare(&source).unwrap()));
        assert!(shared(&oldest, &cache.prepare(oldest.shape()).unwrap()));
        source.mass_kg = 99.;
        cache.prepare(&source).unwrap();
        assert_eq!(cache.0.len(), CACHE_CAPACITY);
        assert!(cache.0.iter().any(|entry| shared(entry, &oldest)));
        assert!(!cache.0.iter().any(|entry| shared(entry, &second)));
        assert!(cache.0.iter().any(|entry| shared(entry, &newest)));
    }
}
