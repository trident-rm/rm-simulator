// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Enemy labels consume team contact records, never hidden robot health/allowance.
use crate::{controls::PlayerCamera, hud::HudState, loading::Screen, session::Session};
use bevy::prelude::*;
use rm_simulator_server::intelligence::{Contact, Source};
#[derive(Component)]
struct IntelLabel(u32);
/// Install screen-space labels for known enemy locations during a match.
pub struct EnemyIntelPlugin;
impl Plugin for EnemyIntelPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, update.run_if(in_state(Screen::InMatch)))
            .add_systems(OnExit(Screen::InMatch), cleanup);
    }
}
fn cleanup(mut commands: Commands, labels: Query<Entity, With<IntelLabel>>) {
    for e in &labels {
        commands.entity(e).despawn();
    }
}
/// Format known values independently; absent fields are explicitly unknown.
pub(crate) fn caption(c: &Contact, detail: bool) -> String {
    let source = match c.source {
        Source::Reported => "REPORT",
        Source::Localized => "RADAR",
        Source::Information => "INFO",
        Source::Optical => "VISION",
    };
    let hp = c.hp.map_or_else(
        || "?".into(),
        |p| {
            c.max_hp
                .map_or_else(|| p.to_string(), |m| format!("{p}/{m}"))
        },
    );
    if detail {
        format!(
            "#{id} · {source} · HP {hp} · AMMO {ammo} · DEF {def} · P {progress:.1}",
            id = c.robot,
            ammo = c.allowance.map_or_else(|| "?".into(), |a| a.to_string()),
            def = c
                .defense_pct
                .map_or_else(|| "?".into(), |d| format!("{d}%")),
            progress = f64::from(c.progress_tenths) / 10.
        )
    } else {
        format!("#{} {source} HP {hp}", c.robot)
    }
}
fn update(
    mut commands: Commands,
    session: Res<Session>,
    ui: Res<HudState>,
    cameras: Query<(&Camera, &GlobalTransform), With<PlayerCamera>>,
    mut labels: Query<(Entity, &IntelLabel, &mut Node, &mut Text)>,
) {
    let contacts = rm_simulator_server::sentry_controller::contacts(
        &session.snapshot,
        &session.sentry_tracking,
        session.team,
    );
    let camera = cameras.iter().next();
    for (entity, label, mut node, mut text) in &mut labels {
        let Some(c) = contacts
            .iter()
            .find(|c| c.robot == label.0 && c.source != Source::Reported)
        else {
            commands.entity(entity).despawn();
            continue;
        };
        let point = rm_simulator_render::flu_position([
            c.position_m[0],
            c.position_m[1],
            c.position_m[2] + 0.7,
        ]);
        let projected =
            camera.and_then(|(camera, transform)| camera.world_to_viewport(transform, point).ok());
        let display = if projected.is_some() && !ui.modal_open() {
            Display::Flex
        } else {
            Display::None
        };
        if node.display != display {
            node.display = display;
        }
        if let Some(p) = projected {
            node.left = px(p.x);
            node.top = px(p.y);
        }
        let caption = caption(c, true);
        if text.0 != caption {
            text.0 = caption;
        }
    }
    for c in contacts
        .iter()
        .filter(|c| c.source != Source::Reported)
        .take(32)
    {
        if labels.iter().any(|(_, l, _, _)| l.0 == c.robot) {
            continue;
        }
        commands.spawn((
            IntelLabel(c.robot),
            Text::new(caption(c, true)),
            TextFont {
                font_size: FontSize::Px(13.),
                ..default()
            },
            TextColor(Color::srgb(1., 0.75, 0.35)),
            Node {
                position_type: PositionType::Absolute,
                display: Display::None,
                padding: UiRect::all(px(4)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.02, 0.03, 0.04, 0.8)),
            GlobalZIndex(2),
            Pickable::IGNORE,
        ));
    }
}
/// HP and ceiling known to this pilot, or unrestricted referee/allied authority.
pub(crate) fn health(session: &Session, id: u32) -> (Option<u32>, Option<u32>) {
    if let Some(game) = &session.snapshot.rmul_2027 {
        // UL has no UC Radar information authority. The native round scoreboard
        // reports its robots directly, consistently with the team HP display.
        return game
            .robots
            .iter()
            .find(|r| r.id == id)
            .map_or((None, None), |r| (Some(r.hp), Some(r.max_hp)));
    }
    let robot = session
        .rmuc_2027()
        .and_then(|s| s.game.robots.iter().find(|r| r.id == id));
    if session.role.referees()
        || session
            .snapshot
            .chassis
            .iter()
            .any(|b| b.id == id && b.team == session.team)
    {
        return robot.map_or((None, None), |r| (Some(r.hp), Some(r.max_hp)));
    }
    session
        .rmuc_2027()
        .and_then(|s| {
            s.game.radar.information[rm_simulator_world::referee::game_team(session.team).index()]
                .get(&id)
                .filter(|info| {
                    s.game.elapsed_ns().saturating_sub(info.observed_ns)
                        < rm_simulator_world::gameplay::rmuc_2027::radar::INFORMATION_NS
                })
        })
        .map_or((None, None), |info| (info.hp, info.max_hp))
}
/// Numeric health label with explicit unknown fields for the roster view.
pub(crate) fn health_text(session: &Session, id: u32) -> String {
    let (hp, max) = health(session, id);
    match (hp, max) {
        (Some(h), Some(m)) => format!("{h} / {m} HP"),
        (Some(h), None) => format!("{h} HP"),
        _ => "HP ?".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn unknown_fields_are_not_invented() {
        let c = Contact {
            robot: 2,
            team: rm_simulator_world::Team::Blue,
            position_m: [0.; 3],
            source: Source::Localized,
            progress_tenths: 1200,
            hp: None,
            max_hp: None,
            allowance: None,
            defense_pct: Some(50),
            age_ns: 0,
        };
        let s = caption(&c, true);
        assert!(s.contains("HP ?"));
        assert!(s.contains("AMMO ?"));
        assert!(s.contains("DEF 50%"));
    }
}
