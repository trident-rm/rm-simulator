// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Rapier-specific immutable resources prepared from solver-free descriptions.
//!
//! Cooking allocates mesh/BVH storage once. Clones and world installations share
//! that storage; body/collider handles, transforms and contacts remain world-local.
//! No global or persistent cooking cache is installed by this module.
use rapier3d_f64::prelude::{SharedShape, TriMeshFlags, Vector};

/// Compatibility access to the solver-free source mesh type.
pub use rm_simulator_engine::model::TriangleMesh;
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
/// use rm_simulator_physics::{WorldPhysics, cooking::{TriangleMesh, CookedTriangleMesh}};
/// let source = TriangleMesh::new(
///     vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]], vec![[0, 1, 2]],
/// )?;
/// let cooked = CookedTriangleMesh::new(source)?;
/// let mut first = WorldPhysics::new(&[], -1.);
/// let mut second = WorldPhysics::new(&[], -1.);
/// first.add_cooked_static_mesh(&cooked);
/// second.add_cooked_static_mesh(&cooked);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::WorldPhysics;
    use std::sync::Arc;

    #[test]
    fn retains_source_and_shares_cooked_shape_between_independent_worlds() {
        let vertices = vec![[2., -1., -1.], [2., 1., -1.], [2., 0., 1.], [2., -1., -1.]];
        let triangles = vec![[3, 1, 2], [0, 0, 0]];
        let source = TriangleMesh::new(vertices.clone(), triangles.clone()).unwrap();
        let cooked = CookedTriangleMesh::new(source.clone()).unwrap();
        assert_eq!(source.vertices_m(), vertices);
        assert_eq!(source.triangles(), triangles);
        assert_eq!(
            cooked.source().vertices_m().as_ptr(),
            source.vertices_m().as_ptr()
        );
        let clone = cooked.clone();
        assert!(Arc::ptr_eq(&cooked.shape.0, &clone.shape.0));
        let mut worlds = [WorldPhysics::new(&[], -10.), WorldPhysics::new(&[], -10.)];
        let mut legacy = WorldPhysics::new(&[], -10.);
        legacy.add_static_mesh(vertices, triangles).unwrap();
        let expected = legacy
            .static_geometry_snapshot(true)
            .cast_ray([0.; 3], [1., 0., 0.], 5.)
            .unwrap();
        for world in &mut worlds {
            world.add_cooked_static_mesh(&cooked);
            let capture = world.static_geometry_snapshot(true);
            assert!(Arc::ptr_eq(&capture.shapes[0].1.0, &cooked.shape.0));
            assert_eq!(
                capture.cast_ray([0.; 3], [1., 0., 0.], 5.).unwrap(),
                expected
            );
            assert_eq!(
                capture
                    .cast_ray([4., 0., 0.], [-1., 0., 0.], 5.)
                    .unwrap()
                    .unwrap()
                    .distance_m,
                2.
            );
        }
        drop(cooked);
        drop(clone);
        drop(source);
        worlds[0].add_static_box(crate::Pose::at([1., 0., 0.]), [0.1; 3]);
        assert_eq!(worlds[1].static_geometry_snapshot(true).shapes.len(), 1);
        assert_eq!(
            worlds[1]
                .static_geometry_snapshot(true)
                .cast_ray([0.; 3], [1., 0., 0.], 5.)
                .unwrap(),
            expected
        );
    }

    #[test]
    fn source_validation_and_backend_cookability_are_separate() {
        assert!(CookedTriangleMesh::new(TriangleMesh::new(vec![], vec![]).unwrap()).is_err());
        let mut world = WorldPhysics::new(&[], -10.);
        assert_eq!(
            world
                .add_static_mesh(vec![[f64::NAN; 3]], vec![])
                .unwrap_err(),
            "mesh vertices must be finite"
        );
        assert_eq!(
            world.add_static_mesh(vec![], vec![[0; 3]]).unwrap_err(),
            "mesh triangle index out of range"
        );
        assert!(world.static_geometry_snapshot(true).shapes.is_empty());
    }
}
