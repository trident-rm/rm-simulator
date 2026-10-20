// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Source diffuser bindings and UC2027 §5.4.3 Table 5-13 indication.
use super::{Root, runs};
use crate::session::Session;
use bevy::{
    mesh::{Indices, VertexAttributeValues},
    prelude::*,
    world_serialization::WorldInstanceReady,
};
use rm_simulator_world::{Team, assembly::Phase, gameplay::rmuc_2027::manipulation::Step};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Ink {
    Off,
    White,
    Red,
    Blue,
    Green,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Indication {
    Idle,
    Positioning,
    Ready,
    Inserted,
    Lifted,
    Raised,
    Aligning,
    Aligned,
    Confirmed,
}
#[derive(Component)]
pub(super) struct Lamp {
    owner: u32,
    generation: u64,
    group: u8,
    angle_rad: f32,
}
#[derive(Resource)]
struct Palette([Handle<StandardMaterial>; 5]);

pub(super) fn install(app: &mut App) {
    app.add_observer(bind)
        .add_systems(Startup, palette)
        .add_systems(
            Update,
            update
                .after(super::visuals)
                .run_if(in_state(crate::loading::Screen::InMatch)),
        );
}
fn palette(mut commands: Commands, mut materials: ResMut<Assets<StandardMaterial>>) {
    commands.insert_resource(Palette(
        [
            Color::srgb(0.045, 0.05, 0.055),
            Color::WHITE,
            Color::srgb(1., 0.025, 0.01),
            Color::srgb(0.015, 0.12, 1.),
            Color::srgb(0.02, 1., 0.04),
        ]
        .map(|color| {
            materials.add(StandardMaterial {
                base_color: color,
                emissive: LinearRgba::from(color)
                    * if color == Color::srgb(0.045, 0.05, 0.055) {
                        0.
                    } else {
                        2.
                    },
                unlit: true,
                double_sided: true,
                cull_mode: None,
                ..default()
            })
        }),
    ));
}

// Source shells can contain several disconnected LED strips. Split only their
// index lists; all source vertices, normals and triangle winding are preserved.
fn components(mesh: &Mesh) -> Vec<(Mesh, Vec3)> {
    let Some(VertexAttributeValues::Float32x3(positions)) =
        mesh.attribute(Mesh::ATTRIBUTE_POSITION)
    else {
        return vec![];
    };
    let indices: Vec<_> = mesh
        .indices()
        .map_or_else(|| (0..positions.len()).collect(), |v| v.iter().collect());
    if !indices.len().is_multiple_of(3) || indices.iter().any(|i| *i >= positions.len()) {
        return vec![];
    }
    let key = |i: usize| positions[i].map(|v| (v as f64 * 1e6).round() as i64);
    let mut remaining: BTreeSet<_> = (0..indices.len() / 3).collect();
    let mut result = vec![];
    while let Some(first) = remaining.pop_first() {
        let mut triangles = vec![first];
        let mut vertices: BTreeSet<_> = indices[first * 3..first * 3 + 3]
            .iter()
            .map(|i| key(*i))
            .collect();
        loop {
            let found: Vec<_> = remaining
                .iter()
                .copied()
                .filter(|t| {
                    indices[t * 3..t * 3 + 3]
                        .iter()
                        .any(|i| vertices.contains(&key(*i)))
                })
                .collect();
            if found.is_empty() {
                break;
            }
            for t in found {
                remaining.remove(&t);
                triangles.push(t);
                vertices.extend(indices[t * 3..t * 3 + 3].iter().map(|i| key(*i)));
            }
        }
        let selected: Vec<_> = triangles
            .into_iter()
            .flat_map(|t| indices[t * 3..t * 3 + 3].iter().map(|i| *i as u32))
            .collect();
        let low = selected.iter().fold(Vec3::splat(f32::INFINITY), |p, i| {
            p.min(Vec3::from(positions[*i as usize]))
        });
        let high = selected
            .iter()
            .fold(Vec3::splat(f32::NEG_INFINITY), |p, i| {
                p.max(Vec3::from(positions[*i as usize]))
            });
        let mut part = mesh.clone();
        part.insert_indices(Indices::U32(selected));
        result.push((part, (low + high) * 0.5));
    }
    result
}

// Accumulate every imported shell transform up to the articulated link.
// Primitive transforms alone omit the source node's rotated/offset CAD frame.
fn in_link(
    entity: Entity,
    link: Entity,
    transforms: &Query<&Transform>,
    parents: &Query<&ChildOf>,
) -> Option<Mat4> {
    let mut current = entity;
    let mut matrix = Mat4::IDENTITY;
    for _ in 0..64 {
        if current == link {
            return Some(matrix);
        }
        matrix = transforms.get(current).ok()?.to_matrix() * matrix;
        current = parents.get(current).ok()?.parent();
    }
    None
}

#[allow(clippy::too_many_arguments)]
fn bind(
    event: On<WorldInstanceReady>,
    roots: Query<&Root>,
    children: Query<&Children>,
    names: Query<&Name>,
    primitives: Query<(
        &Mesh3d,
        &MeshMaterial3d<StandardMaterial>,
        Option<&Transform>,
    )>,
    transforms: Query<&Transform>,
    parents: Query<&ChildOf>,
    mut meshes: ResMut<Assets<Mesh>>,
    materials: Res<Assets<StandardMaterial>>,
    palette: Res<Palette>,
    mut commands: Commands,
) {
    let Ok(root) = roots.get(event.entity) else {
        return;
    };
    if root.kind != 3 {
        return;
    }
    let mut count = [0; 3];
    for link in children.iter_descendants(event.entity) {
        let Ok(name) = names.get(link) else { continue };
        let group = match name.as_str() {
            "link/core/j6" => 1,
            "link/core/q" => 3,
            _ => continue,
        };
        let mut low = Vec3::splat(f32::INFINITY);
        let mut high = Vec3::splat(f32::NEG_INFINITY);
        for entity in children.iter_descendants(link) {
            if let Ok((mesh, material, _)) = primitives.get(entity)
                && let Some(material) = materials.get(&material.0)
                && material.base_color.to_srgba().red > 0.99
                && material.base_color.to_srgba().green > 0.99
                && material.base_color.to_srgba().blue > 0.99
                && let Some(frame) = in_link(entity, link, &transforms, &parents)
                && let Some(mesh) = meshes.get(&mesh.0)
                && let Some(VertexAttributeValues::Float32x3(positions)) =
                    mesh.attribute(Mesh::ATTRIBUTE_POSITION)
            {
                for position in positions {
                    let point = frame.transform_point3(Vec3::from(*position));
                    low = low.min(point);
                    high = high.max(point);
                }
            }
        }
        // The fan spans a semicircle: its bottom edge is the diameter,
        // and the midpoint of its lateral bounds is the angular origin.
        let reference = Vec3::new((low.x + high.x) * 0.5, low.y, 0.);
        for entity in children.iter_descendants(link) {
            let Ok((mesh, material, transform)) = primitives.get(entity) else {
                continue;
            };
            let Some(material) = materials.get(&material.0) else {
                continue;
            };
            // White source diffuser faces (groups 1/3), and only the two red
            // source side bars (group 2). Other painted CAD remains untouched.
            let rgba = material.base_color.to_srgba();
            let white = rgba.red > 0.99 && rgba.green > 0.99 && rgba.blue > 0.99;
            let red = rgba.red > 0.99 && rgba.green < 0.01 && rgba.blue < 0.01;
            let group = if red && group == 1 {
                2
            } else if white {
                group
            } else {
                continue;
            };
            let Some(frame) = in_link(entity, link, &transforms, &parents) else {
                continue;
            };
            let Some(source) = meshes.get(&mesh.0) else {
                continue;
            };
            let parts = components(source);
            if parts.is_empty() {
                continue;
            }
            commands.entity(entity).insert(Visibility::Hidden);
            // Use a sibling with the same transform, rather than a hidden
            // parent's child. No overlay offset, duplicate faces or z fighting.
            let parent = children
                .iter_descendants(link)
                .find(|p| children.get(*p).is_ok_and(|c| c.contains(&entity)))
                .unwrap_or(link);
            for (part, center) in parts {
                // Native glTF axes are left/up/forward. Source diffuser
                // bounds provide the origin; no source dimensions are baked in.
                let center = frame.transform_point3(center);
                let angle_rad = (center.x - reference.x).atan2(center.y - reference.y);
                commands
                    .spawn((
                        Lamp {
                            owner: root.owner,
                            generation: root.generation,
                            group,
                            angle_rad,
                        },
                        Mesh3d(meshes.add(part)),
                        MeshMaterial3d(palette.0[0].clone()),
                        transform.copied().unwrap_or_default(),
                    ))
                    .insert(ChildOf(parent));
                count[(group - 1) as usize] += 1;
            }
        }
    }
    info!(?count, "Bound source Tech Core light groups");
}

fn indication(phase: Phase, step: Step, difficulty: u8) -> Indication {
    match phase {
        Phase::Failed | Phase::Cancelled => Indication::Idle,
        Phase::Positioning => Indication::Positioning,
        Phase::Solving => Indication::Ready,
        Phase::Completed => Indication::Confirmed,
        Phase::Actuating => match step {
            Step::Approach | Step::Insert => Indication::Ready,
            Step::Lift => Indication::Inserted,
            Step::Raise => Indication::Lifted,
            Step::Align => Indication::Aligning,
            Step::Ready => match difficulty {
                1 => Indication::Inserted,
                2 => Indication::Lifted,
                _ => Indication::Aligned,
            },
            Step::Confirmed => Indication::Confirmed,
        },
    }
}
fn blink(now_ns: u64, hz: u64) -> bool {
    ((now_ns % 1_000_000_000) * hz / 500_000_000).is_multiple_of(2)
}
fn ink(
    state: Indication,
    group: u8,
    team: Team,
    now_ns: u64,
    selected: bool,
    current: bool,
    flowing: bool,
) -> Ink {
    let team = match team {
        Team::Red => Ink::Red,
        Team::Blue => Ink::Blue,
    };
    let flash = |ink, hz| if blink(now_ns, hz) { ink } else { Ink::Off };
    if group == 1 {
        match state {
            Indication::Idle => Ink::Off,
            Indication::Positioning => {
                if flowing {
                    Ink::White
                } else {
                    Ink::Off
                }
            }
            Indication::Aligning => {
                if selected {
                    team
                } else if current {
                    Ink::White
                } else {
                    Ink::Off
                }
            }
            Indication::Aligned => {
                if selected {
                    Ink::Green
                } else {
                    Ink::Off
                }
            }
            Indication::Confirmed => flash(Ink::White, 3),
            _ => flash(Ink::White, 1),
        }
    } else {
        match state {
            Indication::Ready => flash(team, 1),
            Indication::Inserted => flash(team, if group == 3 { 3 } else { 1 }),
            Indication::Lifted | Indication::Confirmed => flash(team, 3),
            _ => team,
        }
    }
}
fn nearest(angles: &[f32], target: f32) -> Option<f32> {
    angles
        .iter()
        .copied()
        .min_by(|a, b| (a - target).abs().total_cmp(&(b - target).abs()))
}
fn update(
    session: Res<Session>,
    palette: Res<Palette>,
    mut lamps: Query<(&Lamp, &mut MeshMaterial3d<StandardMaterial>)>,
) {
    let mut angles: BTreeMap<(u32, u64), Vec<f32>> = BTreeMap::new();
    for (lamp, _) in &lamps {
        if lamp.group == 1 {
            angles
                .entry((lamp.owner, lamp.generation))
                .or_default()
                .push(lamp.angle_rad);
        }
    }
    for (lamp, mut material) in &mut lamps {
        let Some(run) = runs(&session)
            .iter()
            .find(|r| r.owner == lamp.owner && r.generation == lamp.generation)
        else {
            continue;
        };
        let core = &run.attempt.cores[0];
        let mut state = indication(run.phase, core.step, run.difficulty);
        // Between finishing P and starting Q, Table 5-13's step-4 indication.
        if state == Indication::Aligning
            && run
                .frames
                .get(run.cursor.saturating_sub(1) as usize)
                .is_some_and(|f| f.stage == 4)
        {
            state = Indication::Raised;
        }
        let a = angles
            .get(&(lamp.owner, lamp.generation))
            .map_or(&[][..], Vec::as_slice);
        let selected = nearest(a, run.attempt.target_q_rad[0] as f32) == Some(lamp.angle_rad);
        let current = nearest(a, core.sensors.q_rad as f32) == Some(lamp.angle_rad);
        // Chase speed is a presentation policy, absent from Table 5-13.
        let chase =
            ((session.snapshot.time_ns % 1_000_000_000) as f32 / 1e9 - 0.5) * std::f32::consts::PI;
        let flowing = nearest(a, chase) == Some(lamp.angle_rad);
        let color = ink(
            state,
            lamp.group,
            run.team,
            session.snapshot.time_ns,
            selected,
            current,
            flowing,
        );
        material.set_if_neq(MeshMaterial3d(palette.0[color as usize].clone()));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn angular_binding_includes_rotated_source_shell_ancestors() {
        let mut app = App::new();
        let link = app
            .world_mut()
            .spawn(Transform::from_xyz(20., 30., 40.))
            .id();
        let shell = app
            .world_mut()
            .spawn((
                Transform::from_translation(Vec3::new(0.6, -0.9, -0.25))
                    .with_rotation(Quat::from_rotation_x(std::f32::consts::PI)),
                ChildOf(link),
            ))
            .id();
        let primitive = app
            .world_mut()
            .spawn((Transform::default(), ChildOf(shell)))
            .id();
        let mut query = app.world_mut().query::<(&Transform, &ChildOf)>();
        assert!(query.get(app.world(), primitive).is_ok());
        let mut state = bevy::ecs::system::SystemState::<(Query<&Transform>, Query<&ChildOf>)>::new(
            app.world_mut(),
        );
        let (transforms, parents) = state.get(app.world()).unwrap();
        let matrix = in_link(primitive, link, &transforms, &parents).unwrap();
        let expected = Vec3::new(0.1, 0.2, 0.);
        let native = matrix.inverse().transform_point3(expected);
        let measured = matrix.transform_point3(native);
        assert!(measured.distance(expected) < 1e-6);
        assert!((measured.x.atan2(measured.y) - 0.4636476).abs() < 1e-6);
        assert!(native.distance(measured) > 0.5);
    }

    #[test]
    fn diffuser_split_preserves_source_triangles_and_attributes() {
        use bevy::{asset::RenderAssetUsages, mesh::PrimitiveTopology};
        let positions = vec![
            [0., 0., 0.],
            [1., 0., 0.],
            [1., 1., 0.],
            [0., 1., 0.],
            [3., 0., 0.],
            [4., 0., 0.],
            [4., 1., 0.],
            [3., 1., 0.],
        ];
        let mut source = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
        source.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
        source.insert_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0., 0., 1.]; 8]);
        source.insert_indices(Indices::U32(vec![0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7]));
        let parts = components(&source);
        assert_eq!(parts.len(), 2);
        let mut triangles = vec![];
        for (mesh, _) in parts {
            assert_eq!(
                mesh.attribute(Mesh::ATTRIBUTE_POSITION),
                source.attribute(Mesh::ATTRIBUTE_POSITION)
            );
            assert_eq!(
                mesh.attribute(Mesh::ATTRIBUTE_NORMAL),
                source.attribute(Mesh::ATTRIBUTE_NORMAL)
            );
            triangles.extend(
                mesh.indices()
                    .unwrap()
                    .iter()
                    .collect::<Vec<_>>()
                    .as_chunks::<3>()
                    .0
                    .iter()
                    .map(|t| t.to_vec()),
            );
        }
        triangles.sort();
        assert_eq!(
            triangles,
            vec![vec![0, 1, 2], vec![0, 2, 3], vec![4, 5, 6], vec![4, 6, 7]]
        );
        assert_eq!(
            indication(Phase::Actuating, Step::Ready, 1),
            Indication::Inserted
        );
        assert_eq!(
            indication(Phase::Actuating, Step::Ready, 2),
            Indication::Lifted
        );
        assert_eq!(
            indication(Phase::Actuating, Step::Ready, 3),
            Indication::Aligned
        );
    }

    #[test]
    fn table_5_13_patterns_and_exact_simulation_frequencies() {
        for team in [Team::Red, Team::Blue] {
            let color = if team == Team::Red {
                Ink::Red
            } else {
                Ink::Blue
            };
            for group in [2, 3] {
                assert_eq!(
                    ink(Indication::Idle, group, team, 0, false, false, false),
                    color
                );
            }
            assert_eq!(
                ink(Indication::Ready, 1, team, 0, false, false, false),
                Ink::White
            );
            assert_eq!(
                ink(Indication::Ready, 1, team, 500_000_000, false, false, false),
                Ink::Off
            );
            assert_eq!(
                ink(
                    Indication::Inserted,
                    2,
                    team,
                    200_000_000,
                    false,
                    false,
                    false
                ),
                color
            );
            assert_eq!(
                ink(
                    Indication::Inserted,
                    3,
                    team,
                    200_000_000,
                    false,
                    false,
                    false
                ),
                Ink::Off
            );
            assert_eq!(
                ink(
                    Indication::Lifted,
                    2,
                    team,
                    200_000_000,
                    false,
                    false,
                    false
                ),
                Ink::Off
            );
            assert_eq!(
                ink(
                    Indication::Raised,
                    2,
                    team,
                    700_000_000,
                    false,
                    false,
                    false
                ),
                color
            );
            assert_eq!(
                ink(Indication::Aligning, 1, team, 0, true, true, false),
                color
            );
            assert_eq!(
                ink(Indication::Aligning, 1, team, 0, false, true, false),
                Ink::White
            );
            assert_eq!(
                ink(Indication::Aligned, 1, team, 0, true, false, false),
                Ink::Green
            );
            assert_eq!(
                ink(
                    Indication::Confirmed,
                    1,
                    team,
                    200_000_000,
                    false,
                    false,
                    false
                ),
                Ink::Off
            );
            assert_eq!(
                ink(Indication::Positioning, 1, team, 0, false, false, true),
                Ink::White
            );
        }
        for hz in [1, 3] {
            assert!(blink(0, hz));
            assert!(!blink(500_000_000 / hz + 1, hz));
            assert!(blink(1_000_000_000, hz));
        }
        assert_eq!(
            indication(Phase::Solving, Step::Approach, 3),
            Indication::Ready
        );
    }
}
