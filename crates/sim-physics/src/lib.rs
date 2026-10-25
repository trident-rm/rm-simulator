// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Generic concrete Rapier physics, without competition participants or rules.
//! Models are described in `sim-engine`; callers own time, controls and sensors.
//! Native participant adapters opt into the explicit `rapier-extension` feature.
#![deny(missing_docs)]
pub mod cooking;
pub mod runtime;
mod triangles;
/// Solver-free metric query requests and capture-local results.
pub mod queries {
    pub use sim_engine::queries::{RayHit, RayQuery};
}
/// Copied generic physical frame observations for passive consumers.
pub mod sensors {
    pub mod carrier;
}
/// Opt-in trusted adapter access to the concrete Rapier backend and slice phases.
#[cfg(feature = "rapier-extension")]
pub mod backend;
pub use runtime::PhysicalRuntime;
pub use sim_engine::{Pose, StepDuration};

/// Fault injection for downstream tests; never required by ordinary consumers.
#[cfg(feature = "test-support")]
pub mod test_support {
    /// Replace the discontinuity epoch for exhaustion tests, without moving bodies.
    /// This deliberately bypasses continuity guarantees and must not be used in a
    /// running simulation or with retained sensor interpolation history.
    pub fn set_placement_epoch(runtime: &mut crate::PhysicalRuntime, epoch: u64) {
        runtime.set_test_placement_epoch(epoch);
    }
}
