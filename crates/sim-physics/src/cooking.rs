// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Rapier-specific immutable resources prepared from solver-free descriptions.
//!
//! Cooking allocates mesh/BVH storage once. Clones and world installations share
//! that storage; body/collider handles, transforms and contacts remain world-local.
//! No global or persistent cooking cache is installed by this module.
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
#[derive(Clone)]
pub struct CookedTriangleMesh {
    source: TriangleMesh,
    pub(crate) shape: SharedShape,
}

impl CookedTriangleMesh {
    /// Cook a validated surface once using the existing Rapier mesh policy.
    /// Copies source buffers into backend storage while retaining the immutable
    /// source. Backend rejection (including an empty surface) creates no world state.
    pub fn new(source: TriangleMesh) -> Result<Self, &'static str> {
        let shape = cook_surface(&source)?;
        Ok(Self { source, shape })
    }

    /// Original solver-free description, not reconstructed tessellation from a BVH.
    pub fn source(&self) -> &TriangleMesh {
        &self.source
    }
}

fn cook_surface(source: &TriangleMesh) -> Result<SharedShape, &'static str> {
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
) -> Result<SharedShape, &'static str> {
    cook_vectors(
        vertices_m.into_iter().map(Vector::from_array).collect(),
        triangles,
    )
}

fn cook_vectors(
    vertices: Vec<Vector>,
    triangles: Vec<[u32; 3]>,
) -> Result<SharedShape, &'static str> {
    SharedShape::trimesh_with_flags(
        vertices,
        triangles,
        TriMeshFlags::MERGE_DUPLICATE_VERTICES | TriMeshFlags::DELETE_DEGENERATE_TRIANGLES,
    )
    .map_err(|_| "mesh must hold at least one triangle")
}
