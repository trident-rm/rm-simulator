// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Rapier-specific immutable resources prepared from solver-free descriptions.
//!
//! Cooking allocates mesh/BVH storage once. Clones and world installations share
//! that storage; body/collider handles, transforms and contacts remain world-local.
//! No global or persistent cooking cache is installed by this module.
/// A source model or its concrete backend representation was rejected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CookError {
    /// A solver-free description failed admission.
    Model(sim_engine::model::ModelError),
    /// The backend cannot represent the admitted source geometry or mass.
    Backend(&'static str),
}
impl CookError {
    /// Original diagnostic for compatibility adapters.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Model(e) => e.as_str(),
            Self::Backend(s) => s,
        }
    }
}
impl std::fmt::Display for CookError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}
impl std::error::Error for CookError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Model(e) => Some(e),
            Self::Backend(_) => None,
        }
    }
}
impl From<sim_engine::model::ModelError> for CookError {
    fn from(error: sim_engine::model::ModelError) -> Self {
        Self::Model(error)
    }
}
impl From<CookError> for &'static str {
    fn from(error: CookError) -> Self {
        error.as_str()
    }
}

use rapier3d_f64::prelude::{SharedShape, TriMeshFlags, Vector};

/// Compatibility access to the solver-free source mesh type.
pub use sim_engine::model::TriangleMesh;
mod body;
pub use body::CookedRigidBody;
mod assembly;
pub use assembly::CookedAssembly;

/// A source surface and its cooked Rapier shape, shared across worlds/resets.
/// The current policy merges duplicate vertices and removes degenerate faces,
/// with two-sided contacts and no internal-edge fixing. Source buffers retain
/// their original order and topology even when the cooked representation differs.
/// This resource contains no placement, collision filter, body or world handle.
///
/// ```
/// use sim_physics::{PhysicalRuntime, cooking::{TriangleMesh, CookedTriangleMesh}};
/// let source = TriangleMesh::new(
///     vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]], vec![[0, 1, 2]],
/// )?;
/// let cooked = CookedTriangleMesh::new(source)?;
/// let mut first = PhysicalRuntime::new(sim_physics::StepDuration::HZ_128);
/// let mut second = PhysicalRuntime::new(sim_physics::StepDuration::HZ_128);
/// let material = sim_engine::model::ContactMaterial {
///     friction: 0.4, restitution: 0., memberships: u32::MAX, filter: u32::MAX,
/// };
/// first.install_fixed_mesh(&cooked, sim_physics::Pose::default(), material)?;
/// second.install_fixed_mesh(&cooked, sim_physics::Pose::default(), material)?;
/// assert_eq!(cooked.source().triangles(), &[[0, 1, 2]]);
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Debug)]
pub struct CookedTriangleMesh {
    source: TriangleMesh,
    pub(crate) shape: SharedShape,
}

impl CookedTriangleMesh {
    /// Cook a validated surface once using the existing Rapier mesh policy.
    /// Copies source buffers into backend storage while retaining the immutable
    /// source. Backend rejection (including an empty surface) creates no world state.
    ///
    /// # Errors
    /// Returns `CookError::Backend` when Rapier rejects the surface, including an empty mesh.
    pub fn new(source: TriangleMesh) -> Result<Self, CookError> {
        let shape = cook_surface(&source)?;
        Ok(Self { source, shape })
    }

    /// Original solver-free description, not reconstructed tessellation from a BVH.
    pub fn source(&self) -> &TriangleMesh {
        &self.source
    }
}

fn cook_surface(source: &TriangleMesh) -> Result<SharedShape, CookError> {
    cook_vectors(
        source
            .vertices_m()
            .iter()
            .copied()
            .map(Vector::from_array)
            .collect(),
        source.triangles().to_vec(),
    )
}

/// Consume already validated buffers. Kept separate so legacy owned-buffer
/// installation does not retain/copy source buffers merely to cook them.
#[cfg(feature = "rapier-extension")]
pub(crate) fn cook_buffers(
    vertices_m: Vec<[f64; 3]>,
    triangles: Vec<[u32; 3]>,
) -> Result<SharedShape, CookError> {
    cook_vectors(
        vertices_m.into_iter().map(Vector::from_array).collect(),
        triangles,
    )
}

fn cook_vectors(vertices: Vec<Vector>, triangles: Vec<[u32; 3]>) -> Result<SharedShape, CookError> {
    SharedShape::trimesh_with_flags(
        vertices,
        triangles,
        TriMeshFlags::MERGE_DUPLICATE_VERTICES | TriMeshFlags::DELETE_DEGENERATE_TRIANGLES,
    )
    .map_err(|_| CookError::Backend("mesh must hold at least one triangle"))
}
