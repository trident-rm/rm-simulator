// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Published Sentry camera observations, without controller ownership.
use serde::{Deserialize, Serialize};

/// One optical detection; no HP/ammunition is inferred from a visible model.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Detection {
    /// Observed target chassis identity.
    pub robot: u32,
    /// Observed enemy FLU body position in metres.
    pub position_m: [f64; 3],
}
/// Controller diagnostics, independent of physical/rule restore authority.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct View {
    /// Controlled Sentry identity.
    pub chassis: u32,
    /// Explicit operator opt-in; disabled controllers never rewrite commands.
    pub enabled: bool,
    /// Last optical sampling tick, on the authoritative 128 Hz field clock.
    pub observed_tick: u64,
    /// Current visible target, or none while sweeping.
    pub target: Option<u32>,
    /// Chassis rotation/gimbal aiming yielded to narrow-passage posture.
    pub heading_locked: bool,
    /// Current camera detections, bounded by the live roster.
    pub detections: Vec<Detection>,
}
