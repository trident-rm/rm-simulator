// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Immutable physical descriptions, independent of cooked shapes and world state.
use std::sync::Arc;
mod assembly;
mod body;
mod compound;
mod convex;
pub use compound::CompoundShape;
mod joint;
mod mount;
pub use mount::{CarrierKey, MountDescription, MountKey, MountTable};
mod policy;
pub use assembly::{
    AssemblyDescription, AssemblyJoint, AssemblyJointId, AssemblyLink, AssemblyLinkId,
};
pub use body::{
    BodyKind, BodyState, ColliderDescription, ContactMaterial, Inertial, RigidBodyDescription,
    ShapeDescription,
};
pub use convex::ConvexHull;
pub use joint::{JointDescription, JointKind, JointLimits, JointMotor, JointState};
pub use policy::{BodyPolicy, CoefficientCombineRule, ColliderMass, ColliderPolicy};

#[derive(Clone, Debug, PartialEq)]
struct MeshData {
    vertices_m: Vec<[f64; 3]>,
    triangles: Vec<[u32; 3]>,
}

/// A validated indexed triangle surface in local FLU metres.
/// Cloning shares immutable source buffers. Placement, material, collision filters,
/// provenance and backend cooking policy belong to the enclosing model/adapter.
/// Validation preserves vertex/index order, duplicates and degenerate triangles;
/// whether a surface is cookable is a separate backend decision.
///
/// ```
/// use rm_simulator_engine::model::TriangleMesh;
/// let mesh = TriangleMesh::new(
///     vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]], vec![[0, 1, 2]],
/// )?;
/// let shared = mesh.clone();
/// assert_eq!(mesh.vertices_m().as_ptr(), shared.vertices_m().as_ptr());
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct TriangleMesh(Arc<MeshData>);

impl TriangleMesh {
    /// Take ownership without copying buffers; reject nonfinite vertices and
    /// invalid indices. Empty surfaces are valid descriptions, not necessarily
    /// valid collision shapes. No simplification or winding repair is performed.
    pub fn new(vertices_m: Vec<[f64; 3]>, triangles: Vec<[u32; 3]>) -> Result<Self, &'static str> {
        if !vertices_m.iter().flatten().all(|v| v.is_finite()) {
            return Err("mesh vertices must be finite");
        }
        if vertices_m.len() > u32::MAX as usize {
            return Err("mesh has too many vertices");
        }
        if triangles
            .iter()
            .flatten()
            .any(|&i| i as usize >= vertices_m.len())
        {
            return Err("mesh triangle index out of range");
        }
        Ok(Self(Arc::new(MeshData {
            vertices_m,
            triangles,
        })))
    }

    /// Source vertices in local FLU metres, in original order.
    pub fn vertices_m(&self) -> &[[f64; 3]] {
        &self.0.vertices_m
    }

    /// Original vertex indices, including source winding and degenerate faces.
    pub fn triangles(&self) -> &[[u32; 3]] {
        &self.0.triangles
    }

    /// Recover owned buffers, reusing their allocations when uniquely owned.
    /// Shared descriptions copy buffers so other users remain immutable.
    pub fn into_buffers(self) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
        let data = Arc::unwrap_or_clone(self.0);
        (data.vertices_m, data.triangles)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validates_without_repairing_and_reuses_unique_buffers() {
        let vertices = vec![[0.; 3], [1., 0., 0.], [0., 1., 0.], [0.; 3]];
        let triangles = vec![[0, 1, 2], [3, 2, 1], [0, 0, 0]];
        let pointers = (vertices.as_ptr(), triangles.as_ptr());
        let mesh = TriangleMesh::new(vertices, triangles).unwrap();
        let shared = mesh.clone();
        assert_eq!(mesh.vertices_m().as_ptr(), shared.vertices_m().as_ptr());
        assert_eq!(mesh.triangles(), &[[0, 1, 2], [3, 2, 1], [0, 0, 0]]);
        let copied = shared.into_buffers();
        assert_ne!(copied.0.as_ptr(), pointers.0);
        let recovered = mesh.into_buffers();
        assert_eq!((recovered.0.as_ptr(), recovered.1.as_ptr()), pointers);
    }

    #[test]
    fn rejects_invalid_data_but_accepts_empty_source() {
        assert!(TriangleMesh::new(vec![[f64::NAN, 0., 0.]], vec![]).is_err());
        assert!(TriangleMesh::new(vec![[f64::INFINITY, 0., 0.]], vec![]).is_err());
        assert!(TriangleMesh::new(vec![[0.; 3]], vec![[0, 1, 0]]).is_err());
        assert!(TriangleMesh::new(vec![], vec![]).is_ok());
    }
}
