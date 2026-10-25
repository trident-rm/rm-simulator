// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Compatibility paths for backend-neutral ray-wheel equations and state.

#[cfg(test)]
pub(crate) use sim_engine::wheel_model::drive_power_scale;
pub use sim_engine::wheel_model::{WheelContact, WheelDriveScratch, WheelModel, WheelState};
