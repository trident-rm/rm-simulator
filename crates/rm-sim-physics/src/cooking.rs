// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Compatibility exports of generic cooked resources used by RM participants.
pub(crate) use sim_physics::backend::cook_buffers;
pub use sim_physics::cooking::{
    CookError, CookedAssembly, CookedRigidBody, CookedTriangleMesh, TriangleMesh,
};
#[cfg(test)]
#[path = "cooking_tests.rs"]
mod regressions;
