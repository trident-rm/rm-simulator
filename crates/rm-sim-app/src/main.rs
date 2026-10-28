// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native simulator and browser field-view entry points.
//! Native modules retain their protocol-driven sessions. The browser module
//! asynchronously loads verified HTTP visuals without starting a native host.
#[cfg(not(target_arch = "wasm32"))]
mod args;
#[cfg(not(target_arch = "wasm32"))]
mod assembly;
#[cfg(not(target_arch = "wasm32"))]
mod auto_aim;
#[cfg(not(target_arch = "wasm32"))]
mod bindings;
#[cfg(not(target_arch = "wasm32"))]
mod console;
#[cfg(not(target_arch = "wasm32"))]
mod controls;
#[cfg(not(target_arch = "wasm32"))]
mod debug;
#[cfg(not(target_arch = "wasm32"))]
mod debug_panel;
#[cfg(not(target_arch = "wasm32"))]
mod enemy_intel;
#[cfg(not(target_arch = "wasm32"))]
mod frames;
#[cfg(not(target_arch = "wasm32"))]
mod graphics;
#[cfg(not(target_arch = "wasm32"))]
mod hit_feedback;
#[cfg(not(target_arch = "wasm32"))]
mod hud;
#[cfg(not(target_arch = "wasm32"))]
mod interpolation;
#[cfg(not(target_arch = "wasm32"))]
mod loading;
#[cfg(not(target_arch = "wasm32"))]
mod minimap;
#[cfg(test)]
#[cfg(not(target_arch = "wasm32"))]
mod net_harness;
#[cfg(not(target_arch = "wasm32"))]
mod network_hud;
#[cfg(not(target_arch = "wasm32"))]
mod prediction;
#[cfg(not(target_arch = "wasm32"))]
mod preferences;
#[cfg(not(target_arch = "wasm32"))]
mod presentation;
#[cfg(not(target_arch = "wasm32"))]
mod projectile_prediction;
#[cfg(feature = "referee-link")]
#[cfg(not(target_arch = "wasm32"))]
mod referee_link;
#[cfg(feature = "referee-link")]
#[cfg(not(target_arch = "wasm32"))]
mod referee_video;
#[cfg(not(target_arch = "wasm32"))]
mod robot_assets;
#[cfg(not(target_arch = "wasm32"))]
mod scene;
#[cfg(not(target_arch = "wasm32"))]
mod screenshot;
#[cfg(not(target_arch = "wasm32"))]
mod sentry;
#[cfg(not(target_arch = "wasm32"))]
mod session;
#[cfg(not(target_arch = "wasm32"))]
mod staging;
#[cfg(feature = "steam")]
#[cfg(not(target_arch = "wasm32"))]
mod steam_support;
#[cfg(not(target_arch = "wasm32"))]
mod supply;
#[cfg(test)]
#[cfg(not(target_arch = "wasm32"))]
mod test_fixtures;
#[cfg(not(target_arch = "wasm32"))]
mod title;
#[cfg(not(target_arch = "wasm32"))]
mod ui_theme;
#[cfg(not(target_arch = "wasm32"))]
mod video_capture;
#[cfg(not(target_arch = "wasm32"))]
mod zone_debug;

#[cfg(target_arch = "wasm32")]
mod browser;
#[cfg(not(target_arch = "wasm32"))]
mod native;

#[cfg(not(target_arch = "wasm32"))]
fn main() -> bevy::app::AppExit {
    native::run()
}

#[cfg(target_arch = "wasm32")]
fn main() {
    browser::start();
}
