// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Client-only inspection of seasonal buff/RFID footprints and traversal order.
use crate::{controls::PlayerCamera, debug_panel::DebugOptions, session::Session};
use bevy::prelude::*;
use rm_simulator_render::flu_position;
use rm_simulator_server::zone_catalog::{Area, areas};
use rm_simulator_world::Team;

/// Installs seasonal zone drawing; it never sends commands to the referee.
pub struct ZoneDebugPlugin;
#[derive(Default, Reflect, bevy::gizmos::config::GizmoConfigGroup)]
struct ZoneGizmos;
#[derive(Component)]
struct ZoneLabel(usize);
#[derive(Component)]
struct ZoneLegend;
#[derive(Resource)]
struct Catalog {
    season: rm_simulator_world::season::Season,
    live: bool,
    areas: Vec<Area>,
}
impl Plugin for ZoneDebugPlugin {
    fn build(&self, app: &mut App) {
        app.init_gizmo_group::<ZoneGizmos>()
            .add_systems(Startup, |mut config: ResMut<GizmoConfigStore>| {
                config.config_mut::<ZoneGizmos>().0.depth_bias = -1.;
            })
            .add_systems(
                Update,
                (prepare, draw)
                    .chain()
                    .run_if(in_state(crate::loading::Screen::InMatch)),
            )
            .add_systems(OnExit(crate::loading::Screen::InMatch), clear);
    }
}
#[allow(clippy::type_complexity)]
fn clear(mut commands: Commands, labels: Query<Entity, Or<(With<ZoneLabel>, With<ZoneLegend>)>>) {
    for entity in &labels {
        commands.entity(entity).despawn();
    }
    commands.remove_resource::<Catalog>();
}
#[allow(clippy::type_complexity)]
fn prepare(
    mut commands: Commands,
    session: Res<Session>,
    catalog: Option<Res<Catalog>>,
    labels: Query<Entity, Or<(With<ZoneLabel>, With<ZoneLegend>)>>,
) {
    if catalog
        .is_some_and(|c| c.season == session.season && c.live == session.rmuc_2027().is_some())
    {
        return;
    }
    for entity in &labels {
        commands.entity(entity).despawn();
    }
    let mut areas = areas(session.season);
    if let Some(native) = session.rmuc_2027() {
        for area in &mut areas {
            if let Some(point) = area.control_point()
                && let Some(zone) = native
                    .config
                    .zones
                    .iter()
                    .find(|z| z.side == area.side && z.point == point)
            {
                area.floor_m = zone.floor_m;
                area.polygon_m = zone.polygon_m.clone();
            }
        }
    }
    for (index, area) in areas.iter().enumerate() {
        commands.spawn((
            ZoneLabel(index),
            Node {
                position_type: PositionType::Absolute,
                display: Display::None,
                padding: UiRect::all(px(3.)),
                ..default()
            },
            GlobalZIndex(15),
            Text::new(format!("{}\n{}", area.id, area.label)),
            TextFont {
                font_size: bevy::text::FontSize::Px(12.),
                ..default()
            },
            TextColor(Color::WHITE),
            BackgroundColor(Color::srgba(0., 0., 0., 0.85)),
            Pickable::IGNORE,
        ));
    }
    commands.spawn((ZoneLegend, Node { position_type: PositionType::Absolute, left:px(18.), bottom:px(125.), display:Display::None, padding:UiRect::all(px(7.)), ..default() },
        GlobalZIndex(15), Text::new(if session.rmuc_2027().is_some() {
            "2027 LIVE CONTROL | Red/blue: active | Amber: contested\nLabels: occupants, expiry, blocking reasons and field benefits\nDiagram RFID (~10 cm); traversal overlays remain inspection only"
        } else if session.season == rm_simulator_world::season::Season::Rm2027 {
            "2027 ZONE INSPECTION | Green: RFID | Boxes: height band\nFigure 5-23 estimates (~10 cm); tunnel interiors inferred\nX-ray outlines; no 2027 buff/control enforcement"
        } else { "2026 ZONE INSPECTION | Green: referee RFID footprints\nBoxes: height band | X-ray outlines" }),
        TextFont {font_size:bevy::text::FontSize::Px(13.),..default()}, TextColor(Color::WHITE), BackgroundColor(Color::srgba(0.,0.,0.,0.85)), Pickable::IGNORE));
    commands.insert_resource(Catalog {
        season: session.season,
        live: session.rmuc_2027().is_some(),
        areas,
    });
}
fn visible(area: &Area, options: &DebugOptions) -> bool {
    options.zones || (area.rfid && options.rfid_zones)
}
fn draw(
    options: Res<DebugOptions>,
    session: Res<Session>,
    catalog: Option<Res<Catalog>>,
    camera: Query<(&Camera, &GlobalTransform), With<PlayerCamera>>,
    mut gizmos: Gizmos<ZoneGizmos>,
    mut labels: Query<(&ZoneLabel, &mut Node, &mut Text), Without<ZoneLegend>>,
    mut legend: Query<&mut Node, (With<ZoneLegend>, Without<ZoneLabel>)>,
) {
    let Some(catalog) = catalog else {
        return;
    };
    let camera = camera.single().ok();
    let diagnostics = session
        .rmuc_2027()
        .filter(|_| options.zone_labels)
        .map(|s| s.game.control_diagnostics())
        .unwrap_or_default();
    let captions = catalog
        .areas
        .iter()
        .map(|area| {
            let mut caption = format!("{}\n{}", area.id, area.label);
            if options.zone_labels
                && let Some(state) = session.rmuc_2027()
                && let Some(point) = area.control_point()
            {
                caption.push('\n');
                caption.push_str(&point_details(&state.game, area.side, point, &diagnostics));
            }
            caption
        })
        .collect::<Vec<_>>();
    for mut node in &mut legend {
        node.display = if options.zones || options.rfid_zones {
            Display::Flex
        } else {
            Display::None
        };
    }
    for area in &catalog.areas {
        if !visible(area, &options) {
            continue;
        }
        let active = session.rmuc_2027().and_then(|r| {
            area.control_point().map(|point| {
                let (territory, index) = match point {
                    rm_simulator_world::rmuc_2027::Point::Outpost => (area.side.index(), 0),
                    rm_simulator_world::rmuc_2027::Point::CentralHighland => (area.side.index(), 1),
                    rm_simulator_world::rmuc_2027::Point::Base => (2 + area.side.index(), 0),
                    rm_simulator_world::rmuc_2027::Point::Fortress => (2 + area.side.index(), 1),
                };
                r.game.control.snapshot().territories[territory].activations[index]
            })
        });
        let color = if active == Some([true, true]) {
            Color::srgb(1., 0.7, 0.1)
        } else if active == Some([true, false]) {
            Color::srgb(1., 0.2, 0.1)
        } else if active == Some([false, true]) {
            Color::srgb(0.1, 0.5, 1.)
        } else if area.rfid {
            Color::srgb(0.1, 1., 0.65)
        } else if area.side == Team::Red {
            Color::srgb(1., 0.3, 0.15)
        } else {
            Color::srgb(0.2, 0.6, 1.)
        };
        let z = area.floor_m + 0.035;
        for (a, b) in area
            .polygon_m
            .iter()
            .zip(area.polygon_m.iter().cycle().skip(1))
        {
            gizmos.line(
                flu_position([a[0], a[1], z]),
                flu_position([b[0], b[1], z]),
                color,
            );
            // An elevated duplicate and uprights keep covered tunnel pads visible
            // as inspection volumes without pretending they are upper deck cards.
            if options.zones {
                gizmos.line(
                    flu_position([a[0], a[1], z + 0.45]),
                    flu_position([b[0], b[1], z + 0.45]),
                    color,
                );
                gizmos.line(
                    flu_position([a[0], a[1], z]),
                    flu_position([a[0], a[1], z + 0.45]),
                    color,
                );
            }
        }
    }
    // Prefer nearby labels and suppress overlaps; full identifiers remain in
    // the catalog and become visible when the camera moves closer.
    let mut projected = Vec::new();
    if options.zone_labels
        && let Some((camera, transform)) = camera
    {
        for (index, area) in catalog.areas.iter().enumerate() {
            if !visible(area, &options) {
                continue;
            }
            let n = area.polygon_m.len() as f64;
            let center = flu_position([
                area.polygon_m.iter().map(|p| p[0]).sum::<f64>() / n,
                area.polygon_m.iter().map(|p| p[1]).sum::<f64>() / n,
                area.floor_m + 0.55,
            ]);
            if let Ok(point) = camera.world_to_viewport(transform, center)
                && let Some(size) = camera.logical_viewport_size()
                && point.x >= 0.
                && point.y >= 0.
                && point.x < size.x
                && point.y < size.y
            {
                projected.push((
                    index,
                    point,
                    center.distance_squared(transform.translation()),
                    area.control_point().is_some_and(|point| {
                        diagnostics.iter().any(|r| {
                            r.points.iter().any(|p| {
                                p.point == point
                                    && p.side.index() == area.side.index()
                                    && p.remaining_ns > 0
                            })
                        })
                    }),
                ));
            }
        }
    }
    // Occupied control cards take precedence over nearby empty inspection areas.
    projected.sort_by(|a, b| b.3.cmp(&a.3).then_with(|| a.2.total_cmp(&b.2)));
    let mut occupied: Vec<Rect> = Vec::new();
    let mut accepted = std::collections::HashMap::new();
    for (index, mut point, _, _) in projected {
        let lines = captions[index].lines().collect::<Vec<_>>();
        let width = lines.iter().map(|s| s.len()).max().unwrap_or(0) as f32 * 7. + 8.;
        let extent = Vec2::new(width, lines.len() as f32 * 16. + 8.);
        if let Some((camera, _)) = camera
            && let Some(size) = camera.logical_viewport_size()
        {
            point = point.min((size - extent).max(Vec2::ZERO));
        }
        let bounds = Rect::from_corners(point, point + extent);
        if occupied.iter().any(|r| !r.intersect(bounds).is_empty()) {
            continue;
        }
        occupied.push(bounds);
        accepted.insert(index, point);
    }
    for (label, mut node, mut text) in &mut labels {
        if let Some(position) = accepted.get(&label.0) {
            if text.0 != captions[label.0] {
                text.0.clone_from(&captions[label.0]);
            }
            node.display = Display::Flex;
            node.left = px(position.x);
            node.top = px(position.y);
        } else {
            node.display = Display::None;
        }
    }
}

fn point_details(
    game: &rm_simulator_world::rmuc_2027::match_rules::Match,
    side: Team,
    point: rm_simulator_world::rmuc_2027::Point,
    robots: &[rm_simulator_world::rmuc_2027::match_rules::buffs::RobotDiagnostic],
) -> String {
    use rm_simulator_world::gameplay::{
        Team as RuleTeam,
        rmuc_2027::{ControlState, Point},
    };
    use rm_simulator_world::rmuc_2027::match_rules::buffs::Activation;
    let side = if side == Team::Red {
        RuleTeam::Red
    } else {
        RuleTeam::Blue
    };
    let territory = &game.control.snapshot().territories[side.index()
        + if matches!(point, Point::Base | Point::Fortress) {
            2
        } else {
            0
        }];
    let presence = territory
        .activations
        .iter()
        .fold([false; 2], |a, b| [a[0] || b[0], a[1] || b[1]]);
    let controller = territory.control.team();
    let motion = if presence == [true; 2] {
        "CONTESTED / PAUSED"
    } else {
        match territory.control {
            ControlState::Full(_) if presence[controller.other().index()] => "CAPTURING",
            ControlState::Temporary(_) if !presence[controller.index()] => "RECOVERING",
            _ if territory.progress_ns > 0 => "ROLLING BACK",
            _ => "HOLDING",
        }
    };
    let mut lines = vec![format!(
        "{:?} | {motion} {:.2}/5s",
        territory.control,
        territory.progress_ns as f64 * 1e-9
    )];
    for robot in robots {
        let Some(p) = robot
            .points
            .iter()
            .find(|p| p.side == side && p.point == point && p.remaining_ns > 0)
        else {
            continue;
        };
        let reason = match p.status {
            Activation::Active => "ACTIVE",
            Activation::NotRunning => "round not running",
            Activation::Defeated => "defeated",
            Activation::Offline => "offline",
            Activation::Weakened => "weakened",
            Activation::NoContact => "no RFID contact",
            Activation::OutpostNotFull => "needs own FULL outpost control",
            Activation::FortressDisabled => "own side still controls outpost",
            Activation::FortressTooEarly => "enemy Fortress unlocks at 3:00",
            Activation::HighlandHeld => "other team holds highland",
            Activation::FortressHeld => "another defender holds Fortress",
        };
        lines.push(format!(
            "#{} {:?}: {reason} | RFID {:.2}s",
            robot.id,
            robot.team,
            p.remaining_ns as f64 * 1e-9
        ));
        if p.status == Activation::Active {
            let e = &robot.effects;
            lines.push(format!(
                "  DEF {}% / VUL {}% / COOL {}/s / RES {}",
                e.defense_pct, e.vulnerability_pct, e.cooling_per_s, e.fortress_reserve
            ));
            if point == Point::Fortress
                && robot.team != side
                && let Some(r) = game.robots.iter().find(|r| r.id == robot.id)
            {
                lines.push(format!(
                    "  ARMOR {} | {:.2}/10s",
                    if game.base_open[side.index()] {
                        "OPEN"
                    } else {
                        "CLOSED"
                    },
                    r.fortress_ns[side.index()] as f64 * 1e-9
                ));
            }
        }
    }
    if lines.len() == 1 {
        lines.push("No unexpired robot detections".into());
    }
    lines.join("\n")
}

#[cfg(test)]
mod tests {
    use super::*;
    use rm_simulator_world::gameplay::{
        Team as RuleTeam,
        rmuc_2027::match_rules::{COUNTDOWN_NS, GroundRole, Match},
    };
    #[test]
    fn labels_explain_contest_expiry_and_eligibility() {
        let mut game = Match::new(1);
        game.add_robot(1, RuleTeam::Red, GroundRole::Infantry);
        game.add_robot(2, RuleTeam::Blue, GroundRole::Infantry);
        game.start(1);
        game.advance_ns(COUNTDOWN_NS);
        for id in [1, 2] {
            game.observe(
                id,
                RuleTeam::Blue,
                rm_simulator_world::rmuc_2027::Point::Base,
                true,
            );
        }
        game.advance_ns(0);
        let text = point_details(
            &game,
            Team::Blue,
            rm_simulator_world::rmuc_2027::Point::Base,
            &game.control_diagnostics(),
        );
        assert!(text.contains("CONTESTED / PAUSED"));
        assert!(text.contains("#1 Red: ACTIVE | RFID 2.00s"));
        assert!(text.contains("DEF 50%"));
        game.observe(
            1,
            RuleTeam::Blue,
            rm_simulator_world::rmuc_2027::Point::Outpost,
            true,
        );
        let text = point_details(
            &game,
            Team::Blue,
            rm_simulator_world::rmuc_2027::Point::Outpost,
            &game.control_diagnostics(),
        );
        assert!(text.contains("needs own FULL outpost control"));
    }
}
