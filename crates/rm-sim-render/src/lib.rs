// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Passive Bevy rendering from caller-owned FLU poses and shared asset handles.
//!
//! [`generic`] works without the default `robomaster` feature. With that feature,
//! [`rm`] groups the game appearance/plugins and the original root paths remain
//! available for compatibility.
//! This crate never advances rules or reads a simulation clock.
#![deny(missing_docs)]
#[cfg(feature = "robomaster")]
pub mod armor;
#[cfg(feature = "robomaster")]
pub mod cad;
#[cfg(feature = "robomaster")]
pub mod chassis;
#[cfg(feature = "robomaster")]
mod equipment;
pub mod graphics_settings;
pub mod lighting;
#[cfg(feature = "robomaster")]
pub mod outpost;
#[cfg(feature = "robomaster")]
pub mod projectile;
/// Shared generated-light material registration, implemented by the generic renderer.
pub use sim_render::quality;
#[cfg(feature = "robomaster")]
pub mod rune;
#[cfg(feature = "robomaster")]
pub mod sync;
#[cfg(feature = "robomaster")]
pub mod targets;
#[cfg(feature = "robomaster")]
pub mod team_robot;

/// Compatibility path for the independent generic renderer.
pub use sim_render as generic;
/// RoboMaster-specific appearance and plugins; existing root paths remain available.
#[cfg(feature = "robomaster")]
pub mod rm;

#[cfg(feature = "robomaster")]
use bevy::prelude::*;
pub use generic::{
    FLOOR_Y_M, PoseFlu, RenderingConfig, RenderingProfile, apply_pose, camera_appearance,
    flu_position, flu_vector, pose_from_transform,
};

/// Team colour of a rune or armor light.
#[cfg(feature = "robomaster")]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TeamColor {
    /// Red team; the red half of the field is FLU +x.
    Red,
    /// Blue team; the blue half of the field is FLU -x.
    #[default]
    Blue,
}
#[cfg(feature = "robomaster")]
impl TeamColor {
    /// Base colour and linear emission of a lit light bar or rune target.
    pub fn light(self) -> (Color, LinearRgba) {
        match self {
            TeamColor::Red => (
                Color::srgb(1.0, 0.12, 0.08),
                LinearRgba::rgb(1.0, 0.03, 0.01),
            ),
            TeamColor::Blue => (Color::srgb(0.0, 0.35, 1.0), LinearRgba::rgb(0.0, 0.2, 1.0)),
        }
    }
}
