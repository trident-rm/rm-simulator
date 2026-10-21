// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Verified CAD manifests, collision meshes, articulation semantics and source bindings.
//! No networking, rendering, match runtime or host-clock dependency is required.
//! Physics supplies FLU poses; gameplay supplies serialized season and rule identities.
#![deny(missing_docs)]
pub mod cad_assets;
pub mod collision_mesh;
pub mod math;
pub mod rmuc_2027;
pub mod semantics;
