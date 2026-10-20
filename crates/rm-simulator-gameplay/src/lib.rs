// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native RMUC 2027 gameplay and shared serialized records.
//! Independent of physics, CAD and Bevy.
//!
//! Live rounds use [`rmuc_2027::match_rules::Match`] and explicit nanosecond
//! durations. Shared serialized state and physical model descriptors remain
//! available for transport and presentation; no 2026 match engine is compiled.
#![deny(missing_docs)]
pub mod coverage;
mod performance;
/// RMUC 2027 control and the native control/structure match subset.
pub mod rmuc_2027;
pub mod season;
mod state;
pub use performance::*;
pub use state::*;

/// Historical serialized tick unit; the current world clock is 128 Hz.
/// Native rules accept nanoseconds directly.
pub const TICK_NS: u64 = 1_000_000;
/// Ticks in one second at the 1 ms resolution, so rule durations multiply
/// cleanly.
pub const SECOND_TICKS: u64 = 1_000;
