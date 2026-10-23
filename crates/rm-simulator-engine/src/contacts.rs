// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Geometric contact observations without participants, scoring or solver handles.

/// Copied geometry reported by a backend contact sample, in world FLU.
/// This is privileged physical observation, not a collision command, damage
/// decision, transition event or continuation checkpoint. Backend sampling phase
/// determines whether points/normal/separation are cached narrow-phase values;
/// signed separation need not equal a projection recomputed after integration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactGeometry {
    /// Surface point on the first collider, in world metres.
    pub point1_m: [f64; 3],
    /// Surface point on the second collider, in world metres.
    pub point2_m: [f64; 3],
    /// Backend-reported world normal pointing from the first collider to second.
    pub normal_from1_to2: [f64; 3],
    /// Backend-reported signed separation in metres; negative means penetration.
    pub separation_m: f64,
}
