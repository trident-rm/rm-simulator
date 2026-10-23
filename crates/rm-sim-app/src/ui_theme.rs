// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared visual tokens for the application shell, controls and HUD.
use bevy::{
    feathers::{dark_theme::create_dark_theme, theme::UiTheme, tokens::semantic},
    prelude::*,
};

pub(crate) const SURFACE: Color = Color::srgb(0.045, 0.063, 0.086);
pub(crate) const PANEL: Color = SURFACE;
pub(crate) const BORDER: Color = Color::srgb(0.16, 0.21, 0.27);
pub(crate) const TEXT: Color = Color::srgb(0.91, 0.94, 0.97);
pub(crate) const MUTED: Color = Color::srgb(0.57, 0.65, 0.73);
pub(crate) const ACCENT: Color = Color::srgb(0.25, 0.79, 0.76);
pub(crate) const SPACE_PX: f32 = 8.;
pub(crate) const RADIUS_PX: f32 = 12.;
pub(crate) const CONTROL_HEIGHT_PX: f32 = 40.;

/// Consistent control height and hit target; width belongs to the caller.
pub(crate) fn button_node() -> Node {
    Node {
        min_height: px(CONTROL_HEIGHT_PX),
        align_items: AlignItems::Center,
        border_radius: BorderRadius::all(px(6)),
        flex_shrink: 0.,
        padding: UiRect::horizontal(px(SPACE_PX * 2.)),
        ..default()
    }
}
/// A padded vertical surface; callers set size, placement and overflow.
pub(crate) fn panel_node() -> Node {
    Node {
        padding: UiRect::all(px(SPACE_PX * 3.)),
        flex_direction: FlexDirection::Column,
        row_gap: px(SPACE_PX * 2.),
        border: UiRect::all(px(1.)),
        border_radius: BorderRadius::all(px(RADIUS_PX)),
        ..default()
    }
}
/// Feathers controls use the same palette as the application surfaces.
pub(crate) fn theme() -> UiTheme {
    let mut theme = create_dark_theme();
    for (token, color) in [
        (semantic::SURFACE_WINDOW, SURFACE),
        (semantic::SURFACE_DIALOG, SURFACE),
        (semantic::BORDER_DEFAULT, BORDER),
        (semantic::TEXT_DEFAULT, TEXT),
        (semantic::TEXT_DIM, MUTED),
        (semantic::FILL_SOLID_DEFAULT, Color::srgb(0.11, 0.15, 0.20)),
        (semantic::FILL_SOLID_HOVER, Color::srgb(0.16, 0.22, 0.28)),
        (semantic::FILL_SOLID_PRESSED, Color::srgb(0.08, 0.12, 0.16)),
        (semantic::FILL_ACCENT_DEFAULT, Color::srgb(0.08, 0.42, 0.42)),
        (semantic::FILL_ACCENT_HOVER, Color::srgb(0.12, 0.48, 0.48)),
        (semantic::TEXT_ON_ACCENT, TEXT),
    ] {
        theme.semantic_base.insert(token, color);
    }
    UiTheme(theme)
}
/// Replace Bevy's fallback monospace face, retaining explicit fonts and sizes.
/// Flair and Feathers remain responsible for their own inherited font choices.
pub(crate) fn typography(
    assets: Option<Res<AssetServer>>,
    mut regular: Local<Option<Handle<Font>>>,
    mut fonts: Query<&mut TextFont, Changed<TextFont>>,
) {
    let Some(assets) = assets else {
        return;
    };
    let regular =
        regular.get_or_insert_with(|| assets.load(bevy::feathers::constants::fonts::REGULAR));
    for mut font in &mut fonts {
        if font.font == bevy::text::FontSource::default() {
            font.font = regular.clone().into();
        }
    }
}
