// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Solver-free simulation contracts and reusable physical model calculations.
//!
//! This core owns FLU poses, exact step durations, validated query and actuation
//! records and scalar wheel laws. Backends own integration, cooked geometry and
//! resource handles; applications own scenarios, rules, scheduling and rendering.
//! There is no implicit world, wall clock, plugin scheduler or device context.
#![deny(missing_docs)]
pub mod contacts;
pub mod control;
pub mod model;
pub mod pose;
pub mod queries;
pub mod time;
pub mod wheel_model;
pub use pose::Pose;
pub use time::{DEFAULT_TICK_NS, StepDuration, tick_ns};
