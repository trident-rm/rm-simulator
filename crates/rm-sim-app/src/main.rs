// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native and browser entry points for the same protocol-driven application.
mod args;
mod assembly;
mod auto_aim;
mod bindings;
#[cfg(not(target_arch = "wasm32"))]
mod console;
mod controls;
mod debug;
mod debug_panel;
mod enemy_intel;
mod frames;
mod graphics;
mod hit_feedback;
mod hud;
mod interpolation;
mod loading;
mod minimap;
#[cfg(test)]
#[cfg(not(target_arch = "wasm32"))]
mod net_harness;
mod network_hud;
mod prediction;
mod preferences;
mod presentation;
mod projectile_prediction;
#[cfg(feature = "referee-link")]
#[cfg(not(target_arch = "wasm32"))]
mod referee_link;
#[cfg(feature = "referee-link")]
#[cfg(not(target_arch = "wasm32"))]
mod referee_video;
mod robot_assets;
mod scene;
mod screenshot;
mod sentry;
mod session;
mod staging;
#[cfg(feature = "steam")]
#[cfg(not(target_arch = "wasm32"))]
mod steam_support;
mod supply;
#[cfg(test)]
#[cfg(not(target_arch = "wasm32"))]
mod test_fixtures;
mod title;
mod ui_theme;
#[cfg(not(target_arch = "wasm32"))]
mod video_capture;
mod zone_debug;

#[cfg(target_arch = "wasm32")]
mod browser;
#[cfg(target_arch = "wasm32")]
mod browser_assembly_worker;
#[cfg(target_arch = "wasm32")]
mod browser_assets;
#[cfg(target_arch = "wasm32")]
mod browser_client;
#[cfg(target_arch = "wasm32")]
mod browser_prediction_worker;
mod native;

#[cfg(not(target_arch = "wasm32"))]
fn main() -> bevy::app::AppExit {
    native::run()
}

#[cfg(target_arch = "wasm32")]
fn main() {
    browser::start();
}
