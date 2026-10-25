// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared diagnostic boundary tessellation for retained physical geometry.
use rapier3d_f64::prelude::*;
pub(crate) fn triangulate(
    shapes: impl IntoIterator<Item = (Pose3, SharedShape)>,
) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
    let mut vertices_m = Vec::new();
    let mut triangles = Vec::new();
    for (pose, shape) in shapes {
        let boundary_mesh;
        let (local, indices) = match shape.as_typed_shape() {
            TypedShape::TriMesh(mesh) => (mesh.vertices(), mesh.indices()),
            TypedShape::Ball(ball) => {
                boundary_mesh = ball.to_trimesh(12, 8);
                (boundary_mesh.0.as_slice(), boundary_mesh.1.as_slice())
            }
            TypedShape::Cuboid(cuboid) => {
                boundary_mesh = cuboid.to_trimesh();
                (boundary_mesh.0.as_slice(), boundary_mesh.1.as_slice())
            }
            TypedShape::ConvexPolyhedron(hull) => {
                boundary_mesh = hull.to_trimesh();
                (boundary_mesh.0.as_slice(), boundary_mesh.1.as_slice())
            }
            _ => continue,
        };
        let base = vertices_m.len() as u32;
        vertices_m.extend(local.iter().map(|v| {
            let p = pose.transform_point(*v);
            [p.x, p.y, p.z]
        }));
        triangles.extend(indices.iter().map(|t| t.map(|i| i + base)));
    }
    (vertices_m, triangles)
}
