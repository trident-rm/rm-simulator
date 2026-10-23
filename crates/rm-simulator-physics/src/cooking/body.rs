// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared backend shapes for an explicitly described rigid body.
use super::*;
use rm_simulator_engine::model::{RigidBodyDescription, ShapeDescription};
use std::sync::Arc;

/// Source body plus shared cooked collider shapes, with no world handles/state.
/// Cooking preserves local frames, source collider order and contact parameters.
/// No density-based mass estimation or inertia synthesis is performed.
#[derive(Clone)]
pub struct CookedRigidBody {
    source: RigidBodyDescription,
    shapes: Arc<[SharedShape]>,
}
impl CookedRigidBody {
    /// Cook collider shapes once. Empty surfaces or backend geometry failures
    /// return an error before any world installation. Clones share all buffers.
    ///
    /// ```
    /// use rm_simulator_engine::model::{BodyKind, RigidBodyDescription};
    /// use rm_simulator_physics::cooking::CookedRigidBody;
    /// let source = RigidBodyDescription::new(BodyKind::Fixed, vec![])?;
    /// let cooked = CookedRigidBody::new(source)?;
    /// assert!(cooked.source().colliders().is_empty());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn new(source: RigidBodyDescription) -> Result<Self, &'static str> {
        let shapes = source
            .colliders()
            .iter()
            .map(|c| {
                Ok(match c.shape() {
                    ShapeDescription::Ball { radius_m } => SharedShape::ball(*radius_m),
                    ShapeDescription::Cuboid { half_extents_m: h } => {
                        SharedShape::cuboid(h[0], h[1], h[2])
                    }
                    ShapeDescription::TriangleMesh(mesh) => cook_surface(mesh)?,
                })
            })
            .collect::<Result<Vec<_>, &'static str>>()?;
        Ok(Self {
            source,
            shapes: shapes.into(),
        })
    }
    /// Original immutable physical description, including explicit inertia.
    pub fn source(&self) -> &RigidBodyDescription {
        &self.source
    }
    pub(crate) fn shapes(&self) -> &[SharedShape] {
        &self.shapes
    }
}
