// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Published state and control identities shared by every transport.
use rm_sim_world::FieldSnapshot;
use serde::{Deserialize, Serialize};

/// What clients see: the field plus the host's pacing state.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SimulationState {
    /// Chassis ids of the training bots, ascending.
    pub bots: Vec<u32>,
    /// Host controller diagnostics and paths; not additional physical rule state.
    pub navigation: Vec<rm_sim_control::navigation::View>,
    /// Opted-in Sentry scan/aim state and current optical detections.
    pub sentry_tracking: Vec<crate::sentry_view::View>,
    /// Host publication identity; zero for an unpublished operator capture.
    pub snapshot_id: u64,
    /// Counter bumped on every pause transition. Inputs and shots stamped with
    /// another epoch are refused.
    pub input_epoch: u64,
    /// Each shooter's newest 32 results, newest first.
    pub shot_results: Vec<crate::protocol::ShotResult>,
    /// Whether the host is paused.
    pub paused: bool,
    /// The whole field at the publication tick.
    pub field: FieldSnapshot,
}

/// Small control/clock reply, captured on the authoritative worker without a world snapshot.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ControlStatus {
    /// Current simulation time in nanoseconds.
    pub time_ns: u64,
    /// Current pause epoch for input admission.
    pub input_epoch: u64,
    /// Whether simulation pacing is paused.
    pub paused: bool,
    /// Optional addressed robot's life/defeat identity; absent for clock-only queries.
    pub chassis: Option<ControlIdentity>,
}
/// Minimal robot identity needed to start an external control lease.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ControlIdentity {
    /// Stable simulator chassis ID, never a referee ID.
    pub id: u32,
    /// Revision changed by placement or revival.
    pub placement_revision: u64,
    /// Defeated robots cannot accept an external lease.
    pub defeated: bool,
}

/// Minimum world-time span of a manual paused step, in nanoseconds.
pub const STEP_NS: u64 = 16_000_000;
/// Whole ticks spanning at least one manual step.
pub fn step_ticks() -> u64 {
    STEP_NS.div_ceil(rm_sim_world::tick_ns()).max(1)
}
