// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Explicit choice after map clicks; 0x0303 keys are controller-defined data.
use super::{FieldMap, HudState};
use crate::session::Session;
use bevy::prelude::*;
use rm_simulator_server::{
    map_interaction::{Interaction, Recipient, Target},
    protocol::{Command, MapMarkerKind},
};
#[derive(Component)]
pub(crate) struct Popup;
#[derive(Component)]
pub(crate) struct Summary;
#[derive(Clone, Copy)]
pub(super) enum Action {
    Marker,
    Sentry,
    Team,
    TeamWithSentry,
    NoKey,
    SelectedKey,
    Cancel,
}
/// Apply a user choice once; confirmed interactions use the ordinary host command path.
pub(super) fn choose(action: Action, ui: &mut HudState, session: &mut Session) {
    let Some((position_m, key_code)) = ui.pending_map else {
        return;
    };
    ui.consumed = true;
    match action {
        Action::NoKey => {
            ui.pending_map = Some((position_m, 0));
            return;
        }
        Action::SelectedKey => {
            let key = match ui.map_marker {
                MapMarkerKind::Attack => b'A',
                MapMarkerKind::Defend => b'B',
                MapMarkerKind::Alert => b'I',
                MapMarkerKind::Custom(c) => c,
            };
            ui.pending_map = Some((position_m, key));
            return;
        }
        Action::Marker => {
            ui.last_marker = Some((ui.map_marker, position_m));
            session.apply(Command::MapMarker {
                kind: ui.map_marker,
                position_m,
            });
        }
        Action::Sentry | Action::Team | Action::TeamWithSentry => {
            session.apply(Command::MapInteraction(Interaction {
                recipient: match action {
                    Action::Team => Recipient::Team {
                        include_sentry: false,
                    },
                    Action::TeamWithSentry => Recipient::Team {
                        include_sentry: true,
                    },
                    _ => Recipient::Sentry,
                },
                target: Target::Position(position_m),
                kind: ui.map_marker,
                key_code,
            }))
        }
        Action::Cancel => {}
    }
    ui.pending_map = None;
}
/// Picking supplies locations for both window and image-target UI cameras.
#[allow(clippy::too_many_arguments)]
pub(crate) fn press(
    event: On<Pointer<Press>>,
    player: Res<crate::controls::Player>,
    mut ui: ResMut<HudState>,
    session: Res<Session>,
    map: Query<(&ComputedNode, &UiGlobalTransform), With<FieldMap>>,
    cameras: Query<&Camera>,
    artwork: Option<Res<crate::minimap::Minimap>>,
) {
    use bevy::picking::pointer::PointerButton;
    if event.button != PointerButton::Primary
        || player.captured
        || ui.consumed
        || ui.pending_map.is_some()
        || (!ui.large_map && ui.modal_open())
        || session.chassis_id.is_none()
        || session.role != rm_simulator_server::protocol::Role::Pilot
    {
        return;
    }
    let (Ok((node, transform)), Ok(camera)) = (map.single(), cameras.get(event.hit.camera)) else {
        return;
    };
    let point = event.pointer_location.position * camera.target_scaling_factor().unwrap_or(1.)
        - camera
            .physical_viewport_rect()
            .map_or(Vec2::ZERO, |rect| rect.min.as_vec2());
    let Some(normalized) = node.normalize_point(*transform, point) else {
        return;
    };
    let bounds = artwork
        .as_ref()
        .map_or([-15., -9., 15., 9.], |map| map.bounds_m);
    ui.pending_map = Some((crate::minimap::point(normalized, bounds), 0));
    ui.large_map = true;
    ui.consumed = true;
}
/// Create/update the choice card as a child of the match-owned map.
pub(crate) fn sync(
    mut commands: Commands,
    ui: Res<HudState>,
    map: Query<Entity, With<FieldMap>>,
    popups: Query<Entity, With<Popup>>,
    mut summaries: Query<&mut Text, With<Summary>>,
) {
    let Some((position, key)) = ui.pending_map else {
        for entity in &popups {
            commands.entity(entity).despawn();
        }
        return;
    };
    let text = format!(
        "Map point: {:.2}, {:.2} m\nInteraction key: {}\nDelivery sends intent; autonomy is not enabled.",
        position[0],
        position[1],
        if key == 0 {
            "none".into()
        } else {
            char::from(key).to_string()
        }
    );
    if popups.is_empty() {
        let Ok(parent) = map.single() else {
            return;
        };
        commands
            .spawn((
                Popup,
                ChildOf(parent),
                GlobalZIndex(90),
                Node {
                    position_type: PositionType::Absolute,
                    left: px(12),
                    top: px(12),
                    width: px(310),
                    padding: UiRect::all(px(12)),
                    row_gap: px(8),
                    flex_direction: FlexDirection::Column,
                    ..default()
                },
                BackgroundColor(Color::srgba(0.025, 0.04, 0.055, 0.98)),
            ))
            .with_children(|p| {
                p.spawn((
                    Summary,
                    Text::new(text.clone()),
                    TextFont {
                        font_size: FontSize::Px(15.),
                        ..default()
                    },
                    Pickable::IGNORE,
                ));
                for (label, action) in [
                    ("Place local marker", Action::Marker),
                    ("Send to Sentry", Action::Sentry),
                    ("Send to team (except Sentry)", Action::Team),
                    ("Send to team + Sentry", Action::TeamWithSentry),
                    ("No key", Action::NoKey),
                    ("Use selected map letter", Action::SelectedKey),
                    ("Cancel", Action::Cancel),
                ] {
                    p.spawn((
                        bevy::ui_widgets::Button,
                        bevy::ui_widgets::ActivateOnPress,
                        Node {
                            padding: UiRect::axes(px(10), px(6)),
                            ..default()
                        },
                        BackgroundColor(Color::srgb(0.15, 0.22, 0.28)),
                    ))
                    .observe(
                        move |_: On<bevy::ui_widgets::Activate>,
                              mut ui: ResMut<HudState>,
                              mut session: ResMut<Session>| {
                            choose(action, &mut ui, &mut session)
                        },
                    )
                    .with_child((
                        Text::new(label),
                        TextFont {
                            font_size: FontSize::Px(15.),
                            ..default()
                        },
                        Pickable::IGNORE,
                    ));
                }
            });
    } else {
        for mut summary in &mut summaries {
            summary.set_if_neq(Text::new(text.clone()));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cancelling_sends_nothing_and_confirmed_intents_keep_selected_key() {
        let mut session = crate::session::test_session(true);
        session.ready();
        let mut ui = HudState {
            pending_map: Some(([1., 2.], 0)),
            ..Default::default()
        };
        choose(Action::Cancel, &mut ui, &mut session);
        assert!(ui.pending_map.is_none());
        assert!(session.map_interactions.is_empty());
        ui.pending_map = Some(([1., 2.], 0));
        ui.map_marker = MapMarkerKind::Defend;
        choose(Action::SelectedKey, &mut ui, &mut session);
        assert_eq!(ui.pending_map, Some(([1., 2.], b'B')));
        choose(Action::Sentry, &mut ui, &mut session);
        crate::session::wait_for_session(&mut session, |s| {
            s.commands_confirmed() && !s.map_interactions.is_empty()
        });
        assert_eq!(session.map_interactions.len(), 1);
        let event = &session.map_interactions[0];
        assert_eq!(event.interaction.kind, MapMarkerKind::Defend);
        assert_eq!(event.interaction.key_code, b'B');
        assert_eq!(event.interaction.target, Target::Position([1., 2.]));
        assert!(event.recipients.is_empty());
        assert!(ui.pending_map.is_none());
    }
}
