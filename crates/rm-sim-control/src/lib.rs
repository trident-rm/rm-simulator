// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-driven navigation and assembly planning without a match, host or renderer.
//! Consumers own admission, worker scheduling, wall time and authoritative actuation.
#![deny(missing_docs)]

pub mod assembly;

pub mod navigation;

/// Caller-driven numerical IK candidate selection and bounded joint-space search.
pub mod manipulator_plan;
