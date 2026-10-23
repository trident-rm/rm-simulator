// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Immutable prepared assembly resources; no caller-world installations.
use super::CookedRigidBody;
use rm_simulator_engine::model::AssemblyDescription;
use std::{collections::HashMap, sync::Arc};

struct Prepared {
    source: AssemblyDescription,
    bodies: Vec<CookedRigidBody>,
    endpoints: Vec<(usize, usize)>,
}

/// Shared source assembly, cooked shapes and resolved source link indices.
/// Preparation validates backend body modes and initial joint constraints. It
/// creates no world, live native handles or persistent model identity. Cloning
/// shares all arrays and shapes; installation supplies runtime keys and placement.
#[derive(Clone)]
pub struct CookedAssembly(Arc<Prepared>);
impl CookedAssembly {
    /// Cook all body resources and validate canonical initial state once.
    /// Unsupported geometry/body pairs or inconsistent joint frames/velocities
    /// return an error without touching a caller's runtime. The source tree's
    /// order is preserved; no joint placement or inertial properties are inferred.
    pub fn new(source: AssemblyDescription) -> Result<Self, &'static str> {
        let indices: HashMap<_, _> = source
            .links()
            .iter()
            .enumerate()
            .map(|(index, link)| (link.id(), index))
            .collect();
        let bodies = source
            .links()
            .iter()
            .map(|link| CookedRigidBody::new(link.body().clone()))
            .collect::<Result<Vec<_>, _>>()?;
        let endpoints = source
            .joints()
            .iter()
            .map(|joint| (indices[&joint.parent()], indices[&joint.child()]))
            .collect();
        let prepared = Self(Arc::new(Prepared {
            source,
            bodies,
            endpoints,
        }));
        crate::runtime::assembly::validate_cooked(&prepared)?;
        Ok(prepared)
    }
    /// Original immutable source, with assembly-local initial states and IDs.
    pub fn source(&self) -> &AssemblyDescription {
        &self.0.source
    }
    pub(crate) fn bodies(&self) -> &[CookedRigidBody] {
        &self.0.bodies
    }
    pub(crate) fn endpoints(&self) -> &[(usize, usize)] {
        &self.0.endpoints
    }
}
