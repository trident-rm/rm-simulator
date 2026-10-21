// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Owned Sentry mode/posture controls; all operations go through the host protocol.
use crate::{hud::HudState, loading::Screen, session::Session};
use bevy::prelude::*;
use rm_simulator_server::protocol::Command;
use rm_simulator_world::gameplay::rmuc_2027::{
    match_rules::{GroundRole, Phase},
    sentry::{Action, Mode, Posture},
};
#[derive(Component)]
struct SentryControls;
#[derive(Component)]
struct ModeControls;
#[derive(Component)]
struct AutoAimStatus;
/// Install the role-specific control panel during a running session.
pub struct SentryPlugin;
impl Plugin for SentryPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(Screen::InMatch), spawn)
            .add_systems(OnExit(Screen::InMatch), despawn)
            .add_systems(Update, update.run_if(in_state(Screen::InMatch)));
    }
}
fn despawn(mut commands: Commands, panels: Query<Entity, With<SentryControls>>) {
    for e in &panels {
        commands.entity(e).despawn();
    }
}
fn spawn(mut commands: Commands) {
    commands
        .spawn((
            SentryControls,
            Node {
                position_type: PositionType::Absolute,
                left: px(20),
                top: percent(25),
                flex_direction: FlexDirection::Column,
                row_gap: px(4),
                padding: UiRect::all(px(8)),
                display: Display::None,
                ..default()
            },
            BackgroundColor(Color::srgba(0.04, 0.06, 0.09, 0.95)),
            GlobalZIndex(3),
        ))
        .with_children(|p| {
            p.spawn((
                Text::new("SENTRY"),
                TextFont {
                    font_size: FontSize::Px(14.),
                    ..default()
                },
            ));
            p.spawn((
                ModeControls,
                Node {
                    column_gap: px(4),
                    ..default()
                },
            ))
            .with_children(|row| {
                for (mode, label) in [
                    (Mode::Automatic, "Automatic"),
                    (Mode::SemiAutomatic, "Semi-auto"),
                ] {
                    row.spawn((
                        bevy::ui_widgets::Button,
                        bevy::ui_widgets::ActivateOnPress,
                        Node {
                            padding: UiRect::all(px(6)),
                            ..default()
                        },
                        BackgroundColor(Color::srgb(0.15, 0.22, 0.28)),
                    ))
                    .observe(
                        move |_: On<bevy::ui_widgets::Activate>,
                              mut s: ResMut<Session>,
                              mut ui: ResMut<HudState>| {
                            ui.consumed = true;
                            if let Some(chassis) = s.chassis_id {
                                s.apply(Command::SetSentryMode { chassis, mode });
                            }
                        },
                    )
                    .with_child((
                        Text::new(label),
                        TextFont {
                            font_size: FontSize::Px(13.),
                            ..default()
                        },
                        Pickable::IGNORE,
                    ));
                }
            });
            for (enabled, label) in [(true, "Auto aim + sweep"), (false, "Manual aim")] {
                p.spawn((
                    bevy::ui_widgets::Button,
                    bevy::ui_widgets::ActivateOnPress,
                    Node {
                        padding: UiRect::all(px(5)),
                        ..default()
                    },
                    BackgroundColor(Color::srgb(0.15, 0.22, 0.28)),
                ))
                .observe(
                    move |_: On<bevy::ui_widgets::Activate>,
                          mut session: ResMut<Session>,
                          mut ui: ResMut<HudState>| {
                        ui.consumed = true;
                        if let Some(chassis) = session.chassis_id {
                            session.apply(Command::SetSentryAutoAim { chassis, enabled });
                        }
                    },
                )
                .with_child((
                    Text::new(label),
                    TextFont {
                        font_size: FontSize::Px(13.),
                        ..default()
                    },
                    Pickable::IGNORE,
                ));
            }
            p.spawn((
                AutoAimStatus,
                Text::new("Manual aim"),
                TextFont {
                    font_size: FontSize::Px(12.),
                    ..default()
                },
                Pickable::IGNORE,
            ));
            for posture in [Posture::Attack, Posture::Defense, Posture::Movement] {
                p.spawn(Node {
                    column_gap: px(4),
                    ..default()
                })
                .with_children(|row| {
                    for enhanced in [false, true] {
                        row.spawn((
                            bevy::ui_widgets::Button,
                            bevy::ui_widgets::ActivateOnPress,
                            Node {
                                padding: UiRect::all(px(6)),
                                ..default()
                            },
                            BackgroundColor(Color::srgb(0.15, 0.22, 0.28)),
                        ))
                        .observe(
                            move |_: On<bevy::ui_widgets::Activate>,
                                  mut s: ResMut<Session>,
                                  mut ui: ResMut<HudState>| {
                                ui.consumed = true;
                                if let Some(chassis) = s.chassis_id {
                                    s.apply(Command::SentryAction {
                                        chassis,
                                        action: Action::Posture { posture, enhanced },
                                    });
                                }
                            },
                        )
                        .with_child((
                            Text::new(format!("{posture:?}{}", if enhanced { " +" } else { "" })),
                            TextFont {
                                font_size: FontSize::Px(13.),
                                ..default()
                            },
                            Pickable::IGNORE,
                        ));
                    }
                });
            }
        });
}
fn update(
    session: Res<Session>,
    ui: Res<HudState>,
    mut panels: Query<&mut Node, (With<SentryControls>, Without<ModeControls>)>,
    mut modes: Query<&mut Node, (With<ModeControls>, Without<SentryControls>)>,
    mut status: Query<&mut Text, With<AutoAimStatus>>,
) {
    let tracking = session
        .sentry_tracking
        .iter()
        .find(|v| Some(v.chassis) == session.chassis_id && v.enabled);
    let caption = tracking.map_or_else(
        || "Manual aim".into(),
        |v| {
            if v.heading_locked {
                "Tunnel heading locked".into()
            } else if let Some(id) = v.target {
                format!("Tracking #{id} | manual fire")
            } else {
                "Sweeping | manual fire".into()
            }
        },
    );
    for mut text in &mut status {
        if text.0 != caption {
            text.0 = caption.clone();
        }
    }
    let game = session.rmuc_2027().map(|s| &s.game);
    let owned = game.is_some_and(|g| {
        g.robots
            .iter()
            .any(|r| Some(r.id) == session.chassis_id && r.role == GroundRole::Sentry)
    });
    for mut node in &mut panels {
        let display = if owned && !ui.modal_open() {
            Display::Flex
        } else {
            Display::None
        };
        if node.display != display {
            node.display = display;
        }
    }
    for mut node in &mut modes {
        let display = if game.is_some_and(|g| g.phase == Phase::Idle) {
            Display::Flex
        } else {
            Display::None
        };
        if node.display != display {
            node.display = display;
        }
    }
}
