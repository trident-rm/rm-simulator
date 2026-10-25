// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Generic physical runtime and the RoboMaster participant compatibility facade.
/// Compatibility path for RoboMaster physical participants.
pub use crate::projectile::WorldPhysics;
pub(crate) use sim_physics::backend::{math, substep_count};
pub use sim_physics::runtime::*;
/// Native integration slice ceiling, in nanoseconds.
pub const SUBSTEP_MAX_NS: u64 = sim_physics::backend::SUBSTEP_MAX_NS;
#[cfg(test)]
#[path = "runtime_tests.rs"]
mod tests;
