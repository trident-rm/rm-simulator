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
            "2027 LIVE CONTROL | Red/blue: active | Amber: contested\nDiagram RFID (~10 cm); terrain/supply overlays inspection only\nBoxes show antenna height; hidden tunnel pads inferred"
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
    mut labels: Query<(&ZoneLabel, &mut Node), Without<ZoneLegend>>,
    mut legend: Query<&mut Node, (With<ZoneLegend>, Without<ZoneLabel>)>,
) {
    let Some(catalog) = catalog else {
        return;
    };
    let camera = camera.single().ok();
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
                ));
            }
        }
    }
    projected.sort_by(|a, b| a.2.total_cmp(&b.2));
    let mut occupied: Vec<Rect> = Vec::new();
    let mut accepted = std::collections::HashMap::new();
    for (index, point, _) in projected {
        let area = &catalog.areas[index];
        let width = area.id.len().max(area.label.len()) as f32 * 7. + 8.;
        let bounds = Rect::from_corners(point, point + Vec2::new(width, 38.));
        if occupied.iter().any(|r| !r.intersect(bounds).is_empty()) {
            continue;
        }
        occupied.push(bounds);
        accepted.insert(index, point);
    }
    for (label, mut node) in &mut labels {
        if let Some(position) = accepted.get(&label.0) {
            node.display = Display::Flex;
            node.left = px(position.x);
            node.top = px(position.y);
        } else {
            node.display = Display::None;
        }
    }
}
