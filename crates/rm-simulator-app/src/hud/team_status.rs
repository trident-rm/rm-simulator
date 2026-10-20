// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Competitor manual (July 2026), main interface pp. 1 and 3: mirrored
//! per-robot health slots, with unoccupied and defeated robots dimmed.
use super::*;
use rm_simulator_world::{RobotKind, RobotSnapshot};

use rm_simulator_server::roster::{Slot, slots};
use rm_simulator_world::season::Season;
const DIM: Color = Color::srgb(0.38, 0.42, 0.46);

/// One team's robot slot row, holding a chassis id per slot or `None` when empty.
#[derive(Component)]
pub(super) struct TeamSlots {
    team: Team,
    season: Season,
    // Bind to chassis identity rather than position in a received snapshot.
    occupants: Vec<Option<u32>>,
}
/// Per-team, per-slot text node showing one robot's kind, id and HP.
#[derive(Component)]
pub(super) struct SlotText {
    team: Team,
    index: usize,
}
/// Per-team, per-slot health bar fill for one robot.
#[derive(Component)]
pub(super) struct SlotFill {
    team: Team,
    index: usize,
}
#[derive(Clone)]
struct RobotStatus {
    id: u32,
    kind: RobotKind,
    hp: Option<(u32, u32)>,
    number: Option<u8>,
}

/// Spawn one team's complete official equipment slots under the given score card.
pub(super) fn spawn(commands: &mut Commands, parent: Entity, team: Team) {
    let row = container(
        commands,
        parent,
        if team == Team::Red {
            "robot-slots red-slots"
        } else {
            "robot-slots"
        },
    );
    commands.entity(row).insert(TeamSlots {
        team,
        season: Season::Rm2026,
        occupants: vec![None; slots(Season::Rm2026).len()],
    });
    for index in 0..slots(Season::Rm2026).len() {
        spawn_slot(commands, row, team, index);
    }
}
fn spawn_slot(commands: &mut Commands, row: Entity, team: Team, index: usize) {
    let card = container(commands, row, "team-robot");
    commands.spawn((
        ChildOf(card),
        SlotText { team, index },
        Text::new(""),
        text_style(12.),
        TextColor(DIM),
        ClassList::new("team-robot-text"),
        Pickable::IGNORE,
    ));
    bar(commands, card, SlotFill { team, index }, DIM);
}
fn kind_name(kind: RobotKind) -> &'static str {
    match kind {
        RobotKind::Hero => "HERO",
        RobotKind::Engineer => "ENG",
        RobotKind::Infantry => "INF",
        RobotKind::Drone => "DRONE",
        RobotKind::Sentry => "SENTRY",
    }
}
fn matches(slot: &Slot, robot: &RobotStatus) -> bool {
    slot.prototype.is_some_and(|p| {
        p.kind() == robot.kind && robot.number.is_none_or(|number| slot.number == number)
    })
}
fn reconcile(occupants: &mut Vec<Option<u32>>, robots: &[RobotStatus], slots: &[Slot]) {
    for (index, occupant) in occupants.iter_mut().enumerate() {
        if !robots
            .iter()
            .any(|r| Some(r.id) == *occupant && (index >= slots.len() || matches(&slots[index], r)))
        {
            *occupant = None;
        }
    }
    let mut ordered: Vec<_> = robots.iter().collect();
    ordered.sort_by_key(|r| r.id);
    for robot in ordered {
        if occupants.contains(&Some(robot.id)) {
            continue;
        }
        let index = (0..occupants.len())
            .find(|&i| occupants[i].is_none() && (i >= slots.len() || matches(&slots[i], robot)));
        if let Some(index) = index {
            occupants[index] = Some(robot.id);
        } else {
            occupants.push(Some(robot.id));
        }
    }
}
fn presentation(
    index: usize,
    robot: Option<&RobotStatus>,
    team: Team,
    slots: &[Slot],
) -> (String, f32, Color) {
    if let Some(robot) = robot {
        let health = robot
            .hp
            .map_or_else(|| "HP --".into(), |(hp, max)| format!("{hp}/{max}"));
        let status = if robot.hp.is_some_and(|(hp, _)| hp == 0) {
            "DOWN"
        } else if slots.get(index).is_some_and(|s| s.name == "Heavy") {
            "HEAVY"
        } else {
            kind_name(robot.kind)
        };
        let slot = slots
            .get(index)
            .map_or_else(|| "+".into(), |slot| slot.number.to_string());
        let text = format!("{slot} {status}\n{health}");
        let fraction = robot.hp.map_or(0., |(hp, max)| {
            (hp as f32 / max.max(1) as f32).clamp(0., 1.)
        });
        let color = if robot.hp.is_some_and(|(hp, _)| hp == 0) {
            DIM
        } else {
            team_color(team)
        };
        (text, fraction, color)
    } else {
        let title = slots.get(index).map_or_else(
            || "+ EXTRA".into(),
            |slot| {
                format!(
                    "{} {}",
                    slot.number,
                    match slot.name {
                        "Hero" => "HERO",
                        "Engineer" => "ENG",
                        "Heavy" => "HEAVY",
                        "Infantry 3" | "Infantry 4" | "Infantry 5" => "INF",
                        "Drone" => "DRONE",
                        "Aerial system" => "AIR",
                        "Sentry" => "SENTRY",
                        "Dart system" => "DART",
                        _ => "RADAR",
                    }
                )
            },
        );
        let status = if slots.get(index).is_some_and(|s| s.prototype.is_none()) {
            "NO SIM"
        } else {
            "EMPTY"
        };
        (format!("{title}\n{status}"), 0., DIM)
    }
}

/// Reconcile each team's slot occupants against the latest snapshot, spawn a
/// slot for any extra chassis, and write each slot's text, tint and bar width.
/// Runs after `session::advance_world`, so the health shown matches the frame.
pub(super) fn update(
    mut commands: Commands,
    session: Option<Res<Session>>,
    mut rows: Query<(Entity, &mut TeamSlots)>,
    mut texts: Query<(&SlotText, &mut Text, &mut TextColor)>,
    mut fills: Query<(&SlotFill, &mut Node, &mut BackgroundColor)>,
) {
    let Some(session) = session else {
        return;
    };
    for (entity, mut row) in &mut rows {
        let slots = slots(session.season);
        if row.season != session.season {
            commands.entity(entity).despawn_children();
            row.season = session.season;
            row.occupants = vec![None; slots.len()];
            for index in 0..slots.len() {
                spawn_slot(&mut commands, entity, row.team, index);
            }
            continue;
        }
        // The chassis and HP come from the same authoritative snapshot. A roster
        // update can arrive separately; spectators never create health slots.
        let robots: Vec<_> = session
            .snapshot
            .chassis
            .iter()
            .filter(|c| c.team == row.team)
            .map(|c| {
                let health: Option<&RobotSnapshot> = session
                    .referee()
                    .and_then(|r| r.robots.iter().find(|r| r.id == c.id && r.team == row.team));
                let native_kind = session
                    .rmuc_2027()
                    .and_then(|s| s.game.robots.iter().find(|r| r.id == c.id))
                    .map(|r| match r.role {
                        rm_simulator_world::rmuc_2027::match_rules::GroundRole::Heavy => {
                            RobotKind::Engineer
                        }
                        rm_simulator_world::rmuc_2027::match_rules::GroundRole::Infantry => {
                            RobotKind::Infantry
                        }
                        rm_simulator_world::rmuc_2027::match_rules::GroundRole::Sentry => {
                            RobotKind::Sentry
                        }
                    });
                RobotStatus {
                    id: c.id,
                    kind: health.map_or_else(
                        || {
                            session
                                .roster
                                .iter()
                                .find(|p| p.chassis == Some(c.id))
                                .and_then(|p| p.robot)
                                .map_or(native_kind.unwrap_or(RobotKind::Infantry), |r| r.kind())
                        },
                        |r| r.kind,
                    ),
                    number: session
                        .roster
                        .iter()
                        .find(|p| p.chassis == Some(c.id))
                        .and_then(|p| p.robot)
                        .map(|r| {
                            slots
                                .iter()
                                .find(|s| s.prototype == Some(r))
                                .map_or(r.number(), |s| s.number)
                        }),
                    hp: health.map(|r| (r.hp, r.max_hp)).or_else(|| {
                        session
                            .rmuc_2027()
                            .and_then(|s| s.game.robots.iter().find(|r| r.id == c.id))
                            .map(|r| (r.hp, r.max_hp))
                    }),
                }
            })
            .collect();
        let old_len = row.occupants.len();
        reconcile(&mut row.occupants, &robots, slots);
        for index in old_len..row.occupants.len() {
            spawn_slot(&mut commands, entity, row.team, index);
        }
        for (slot, mut text, mut color) in &mut texts {
            if slot.team != row.team {
                continue;
            }
            let robot = row.occupants[slot.index].and_then(|id| robots.iter().find(|r| r.id == id));
            let (value, _, tint) = presentation(slot.index, robot, row.team, slots);
            if text.0 != value {
                text.0 = value;
            }
            if color.0 != tint {
                color.0 = tint;
            }
        }
        for (slot, mut node, mut color) in &mut fills {
            if slot.team != row.team {
                continue;
            }
            let robot = row.occupants[slot.index].and_then(|id| robots.iter().find(|r| r.id == id));
            let (_, fraction, tint) = presentation(slot.index, robot, row.team, slots);
            let width = percent(fraction * 100.);
            if node.width != width {
                node.width = width;
            }
            if color.0 != tint {
                color.0 = tint;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn populated_session() -> Session {
        let mut session = crate::session::test_session(true);
        let template = session.snapshot.chassis[0].clone();
        session.snapshot.chassis.clear();
        let mut field = rm_simulator_world::Field::new(&rm_simulator_world::FieldConfig {
            referee: Some(rm_simulator_world::RefereeConfig::default()),
            ..default()
        })
        .unwrap()
        .snapshot();
        let referee = field.referee.as_mut().unwrap();
        for (id, team, kind, hp) in [
            (0, Team::Red, RobotKind::Infantry, 150),
            (1, Team::Blue, RobotKind::Hero, 200),
            (2, Team::Red, RobotKind::Infantry, 0),
            (3, Team::Blue, RobotKind::Infantry, 40),
            (4, Team::Red, RobotKind::Infantry, 200),
        ] {
            let mut chassis = template.clone();
            chassis.id = id;
            chassis.team = team;
            session.snapshot.chassis.push(chassis);
            referee.robots.push(RobotSnapshot {
                id,
                team,
                kind,
                hp,
                max_hp: 200,
                level: 1,
                weakened: false,
            });
        }
        session.snapshot.referee = field.referee;
        session.roster.clear(); // Simulate a roster message that has not arrived yet.
        session
    }
    #[test]
    fn both_teams_update_without_roster_and_disconnect_clears_health() {
        let mut app = App::new();
        app.insert_resource(populated_session())
            .add_systems(Startup, |mut commands: Commands| {
                let root = commands.spawn_empty().id();
                spawn(&mut commands, root, Team::Red);
                spawn(&mut commands, root, Team::Blue);
            })
            .add_systems(Update, update);
        app.update();
        app.update();
        let texts: Vec<_> = app
            .world_mut()
            .query::<&Text>()
            .iter(app.world())
            .map(|t| t.0.clone())
            .collect();
        let occupants: Vec<_> = app
            .world_mut()
            .query::<&TeamSlots>()
            .iter(app.world())
            .flat_map(|r| r.occupants.iter().flatten().copied())
            .collect();
        for id in 0..5 {
            assert!(occupants.contains(&id));
        }
        assert!(texts.iter().any(|t| t.contains("DOWN")));
        assert!(texts.iter().any(|t| t.contains("150/200")));
        {
            let mut session = app.world_mut().resource_mut::<Session>();
            session.snapshot.chassis.retain(|c| c.id != 0);
            session
                .snapshot
                .referee
                .as_mut()
                .unwrap()
                .robots
                .iter_mut()
                .find(|r| r.id == 3)
                .unwrap()
                .hp = 10;
        }
        app.update();
        let texts: Vec<_> = app
            .world_mut()
            .query::<&Text>()
            .iter(app.world())
            .map(|t| t.0.clone())
            .collect();
        assert!(
            !app.world_mut()
                .query::<&TeamSlots>()
                .iter(app.world())
                .any(|r| r.occupants.contains(&Some(0)))
        );
        assert!(texts.iter().any(|t| t.contains("10/200")));
    }

    #[test]
    fn official_number_takes_precedence_over_same_kind_order() {
        let mut infantry4 = robot(2, RobotKind::Infantry, 200);
        infantry4.number = Some(4);
        let mut occupants = vec![None; 7];
        reconcile(&mut occupants, &[infantry4], slots(Season::Rm2027));
        assert_eq!(occupants[1], None);
        assert_eq!(occupants[2], Some(2));
    }

    #[test]
    fn season_2027_has_complete_equipment_without_legacy_hero_alias() {
        let mut session = populated_session();
        session.season = Season::Rm2027;
        let mut app = App::new();
        app.insert_resource(session)
            .add_systems(Startup, |mut commands: Commands| {
                let root = commands.spawn_empty().id();
                spawn(&mut commands, root, Team::Red);
                spawn(&mut commands, root, Team::Blue);
            })
            .add_systems(Update, update);
        for _ in 0..4 {
            app.update();
        }
        let texts: Vec<_> = app
            .world_mut()
            .query::<&Text>()
            .iter(app.world())
            .map(|t| t.0.clone())
            .collect();
        for name in [
            "1 HEAVY", "3 INF", "4 INF", "6 AIR", "7 SENTRY", "8 DART", "9 RADAR",
        ] {
            assert!(
                texts.iter().any(|t| t.starts_with(name)),
                "missing {name}: {texts:?}"
            );
        }
        assert!(texts.iter().any(|t| t.starts_with("1 HEAVY\nEMPTY")));
        // A stale Hero snapshot remains an extra practice chassis, never Heavy.
        assert!(!texts.iter().any(|t| t.starts_with("1 HERO")));
    }

    #[test]
    fn tab_shows_both_teams_while_held_and_toolbar_can_keep_it_open() {
        let mut app = App::new();
        app.insert_resource(populated_session())
            .init_resource::<HudState>()
            .init_resource::<ButtonInput<KeyCode>>()
            .add_systems(Startup, |mut commands: Commands| spawn_hud(&mut commands))
            .add_systems(Update, update_panel_rows);
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::Tab);
        for _ in 0..3 {
            app.update();
        }
        let rows = app
            .world_mut()
            .query::<&PanelRows>()
            .single(app.world())
            .unwrap();
        assert!(rows.0.iter().any(|r| r[0] == "RED"));
        assert!(rows.0.iter().any(|r| r[0] == "BLUE"));
        for id in 0..5 {
            assert!(rows.0.iter().any(|r| r[0] == format!("Robot #{id}")));
        }
        let frame = app
            .world_mut()
            .query_filtered::<Entity, With<PanelFrame>>()
            .single(app.world())
            .unwrap();
        assert_eq!(
            app.world().get::<Node>(frame).unwrap().display,
            Display::Flex
        );
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .release(KeyCode::Tab);
        app.update();
        assert_eq!(
            app.world().get::<Node>(frame).unwrap().display,
            Display::None
        );
        app.world_mut().resource_mut::<HudState>().roster = true;
        app.update();
        assert_eq!(
            app.world().get::<Node>(frame).unwrap().display,
            Display::Flex
        );
        app.world_mut().resource_mut::<HudState>().close();
        app.world_mut().resource_mut::<HudState>().pause_menu = true;
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::Tab);
        app.update();
        assert_eq!(
            app.world().get::<Node>(frame).unwrap().display,
            Display::None
        );
    }

    fn robot(id: u32, kind: RobotKind, hp: u32) -> RobotStatus {
        RobotStatus {
            id,
            kind,
            hp: Some((hp, 200)),
            number: None,
        }
    }
    #[test]
    fn duplicate_types_overflow_and_departure_does_not_move_survivors() {
        let mut occupants = vec![None; 9];
        let slots = slots(Season::Rm2026);
        let robots = vec![
            robot(7, RobotKind::Infantry, 200),
            robot(2, RobotKind::Infantry, 100),
            robot(9, RobotKind::Infantry, 0),
            robot(1, RobotKind::Hero, 200),
        ];
        reconcile(&mut occupants, &robots, slots);
        assert_eq!(
            occupants,
            vec![
                Some(1),
                None,
                Some(2),
                Some(7),
                Some(9),
                None,
                None,
                None,
                None
            ]
        );
        reconcile(
            &mut occupants,
            &[robots[0].clone(), robots[2].clone()],
            slots,
        );
        assert_eq!(
            occupants,
            vec![None, None, None, Some(7), Some(9), None, None, None, None]
        );
        reconcile(
            &mut occupants,
            &[
                robots[0].clone(),
                robots[2].clone(),
                robot(12, RobotKind::Infantry, 200),
            ],
            slots,
        );
        assert_eq!(occupants[2], Some(12));
    }
    #[test]
    fn empty_dead_and_damaged_are_distinct() {
        let (empty, fill, color) = presentation(0, None, Team::Red, slots(Season::Rm2026));
        assert!(empty.contains("EMPTY"));
        assert_eq!(fill, 0.);
        assert_eq!(color, DIM);
        let (dead, fill, color) = presentation(
            0,
            Some(&robot(4, RobotKind::Hero, 0)),
            Team::Red,
            slots(Season::Rm2026),
        );
        assert!(dead.contains("DOWN"));
        assert!(dead.starts_with("1 DOWN"));
        assert_eq!(fill, 0.);
        assert_eq!(color, DIM);
        let (_, fill, color) = presentation(
            0,
            Some(&robot(4, RobotKind::Hero, 50)),
            Team::Blue,
            slots(Season::Rm2026),
        );
        assert_eq!(fill, 0.25);
        assert_eq!(color, team_color(Team::Blue));
    }
}
