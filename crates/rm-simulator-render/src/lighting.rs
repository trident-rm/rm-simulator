// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Field lighting: ambient plus a shadowed key and an unshadowed fill.
//! Neutral white lamps preserve the CAD paint colours. Levels are assumed for
//! an indoor venue, not measured.
use bevy::prelude::*;

/// The venue key light; the fill stays unshadowed at every graphics preset.
#[derive(Component)]
pub struct ShadowKeyLight;

/// Spawn the venue lamps and set white global ambient light. The shadowed key
/// light is the only one that casts shadows; illuminances are assumed for an
/// indoor venue, not measured.
pub fn spawn_lighting(commands: &mut Commands) {
    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: 70.0,
        ..default()
    });
    commands.spawn((
        ShadowKeyLight,
        DirectionalLight {
            illuminance: 1700.0,
            color: Color::WHITE,
            shadow_maps_enabled: true,
            ..default()
        },
        // The 29 m field fits within 40 m of the camera; keep the 10 m near cascade.
        bevy::light::CascadeShadowConfigBuilder {
            num_cascades: 2,
            first_cascade_far_bound: 10.0,
            maximum_distance: 40.0,
            ..default()
        }
        .build(),
        Transform::from_xyz(-4.0, 8.0, 1.0)
            .looking_at(Vec3::new(0.0, crate::FLOOR_Y_M, -3.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 450.0,
            color: Color::WHITE,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_xyz(5.0, 4.0, -5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

/// Indoor hall cubemap baked once from an artistic venue at its central viewpoint.
/// Distant seating, walls and roof have no runtime geometry or parallax.
pub fn skybox(images: &mut Assets<Image>) -> bevy::light::Skybox {
    use bevy::{
        asset::RenderAssetUsages,
        render::render_resource::{
            Extent3d, TextureDimension, TextureFormat, TextureViewDescriptor, TextureViewDimension,
        },
    };
    const SIZE: u32 = 512;
    let mut pixels = Vec::with_capacity((SIZE * SIZE * 6 * 4) as usize);
    for face in 0..6 {
        for y in 0..SIZE {
            for x in 0..SIZE {
                let u = 2.0 * (x as f32 + 0.5) / SIZE as f32 - 1.0;
                let v = 2.0 * (y as f32 + 0.5) / SIZE as f32 - 1.0;
                let direction = match face {
                    0 => Vec3::new(1.0, -v, -u),
                    1 => Vec3::new(-1.0, -v, u),
                    2 => Vec3::new(u, 1.0, v),
                    3 => Vec3::new(u, -1.0, -v),
                    4 => Vec3::new(u, -v, 1.0),
                    _ => Vec3::new(-u, -v, -1.0),
                }
                .normalize();
                let color = hall_color(direction);
                pixels.extend([color.x as u8, color.y as u8, color.z as u8, 255]);
            }
        }
    }
    let mut image = Image::new(
        Extent3d {
            width: SIZE,
            height: SIZE * 6,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        pixels,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    );
    image.sampler = bevy::image::ImageSampler::linear();
    image
        .reinterpret_stacked_2d_as_array(6)
        .expect("six square skybox faces");
    image.texture_view_descriptor = Some(TextureViewDescriptor {
        dimension: Some(TextureViewDimension::Cube),
        ..default()
    });
    bevy::light::Skybox {
        image: Some(images.add(image)),
        brightness: 500.0,
        ..default()
    }
}

// An intentionally indistinct hall: broad tonal shapes give enclosure without
// repeated stripes or bright fixtures competing with the field's target lights.
fn hall_color(direction: Vec3) -> Vec3 {
    let origin = Vec3::new(0.0, 1.0, 0.0);
    let wall_distance = 70.0 / direction.x.abs().max(direction.z.abs()).max(1e-6);
    let roof_distance = if direction.y > 0.0 {
        30.0 / direction.y
    } else {
        f32::INFINITY
    };
    let floor_distance = if direction.y < 0.0 {
        -1.0 / direction.y
    } else {
        f32::INFINITY
    };
    let distance = wall_distance.min(roof_distance).min(floor_distance);
    let point = origin + direction * distance;
    if distance == roof_distance {
        let mut color = Vec3::new(24.0, 27.0, 32.0);
        // Just three low-contrast roof beams, softened in the bake itself.
        for x in [-43.0, 8.0, 49.0] {
            let beam = (-((point.x - x) / 0.8).powi(2)).exp();
            color -= Vec3::splat(5.0 * beam);
        }
        // Broad diffused fixtures, deliberately sparse and staggered. A soft
        // superellipse avoids sharp luminous rectangles and cubemap aliasing.
        for (x, z) in [(-27.0, -23.0), (24.0, -8.0), (-6.0, 34.0)] {
            let falloff =
                (-0.5 * (((point.x - x) / 5.0).powi(4) + ((point.z - z) / 9.0).powi(4))).exp();
            color += Vec3::new(58.0, 59.0, 60.0) * falloff;
        }
        color
    } else if distance == floor_distance {
        Vec3::new(12.0, 15.0, 20.0)
    } else {
        // Seating reads as one dim bank rather than individually striped rows.
        let seating = (-(point.y / 7.0).powi(4)).exp();
        let upper_fade = (point.y / 31.0).clamp(0.0, 1.0);
        Vec3::new(20.0, 23.0, 28.0) - Vec3::splat(3.0 * seating + 2.0 * upper_fade)
    }
}

/// Decorative venue, retained with the CAD across matches.
#[derive(Component)]
pub struct Stadium;

/// Nearby welded mesh fencing and floor apron; the distant venue is in the cubemap.
/// Bounds come from the same grounded CAD scenery
/// used to place the collision walls. Venue dimensions are artistic settings.
pub fn spawn_stadium(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    corner_a: Vec3,
    corner_b: Vec3,
) {
    let low = corner_a.min(corner_b);
    let high = corner_a.max(corner_b);
    let center = (low + high) * 0.5;
    let half = (high - low) * 0.5;
    let mut steel = None;
    // Four local wall frames: x runs along the fence, z points out of the field.
    for (offset, rotation, length) in [
        (Vec3::new(0.0, 0.0, half.z + 0.05), 0.0, half.x * 2.0),
        (
            Vec3::new(0.0, 0.0, -half.z - 0.05),
            std::f32::consts::PI,
            half.x * 2.0,
        ),
        (
            Vec3::new(half.x + 0.05, 0.0, 0.0),
            std::f32::consts::FRAC_PI_2,
            half.z * 2.0,
        ),
        (
            Vec3::new(-half.x - 0.05, 0.0, 0.0),
            -std::f32::consts::FRAC_PI_2,
            half.z * 2.0,
        ),
    ] {
        let rotation = Quat::from_rotation_y(rotation);
        let origin = Vec3::new(center.x, low.y, center.z) + offset;
        let height = high.y + 2.0 - low.y;
        let bar = |batch: &mut Option<Mesh>, size: Vec3, position: Vec3| {
            let mesh = Mesh::from(Cuboid::from_size(size)).transformed_by(Transform {
                translation: origin + rotation * position,
                rotation,
                ..default()
            });
            if let Some(batch) = batch {
                batch.merge(&mesh).expect("cuboids share vertex attributes");
            } else {
                *batch = Some(mesh);
            }
        };
        let panels = (length / 2.0).ceil() as u32;
        for i in 0..=panels {
            bar(
                &mut steel,
                Vec3::new(0.055, height, 0.055),
                Vec3::new(
                    -length / 2.0 + length * i as f32 / panels as f32,
                    height / 2.0,
                    0.0,
                ),
            );
        }
        for y in [0.0, height] {
            bar(
                &mut steel,
                Vec3::new(length, 0.045, 0.045),
                Vec3::new(0.0, y, 0.0),
            );
        }
        // Actual open geometry, rather than a transparent solid panel.
        let wires = (length / 0.16).ceil() as u32;
        for i in 0..=wires {
            bar(
                &mut steel,
                Vec3::new(0.009, height, 0.009),
                Vec3::new(
                    -length / 2.0 + length * i as f32 / wires as f32,
                    height / 2.0,
                    0.0,
                ),
            );
        }
        let rows = (height / 0.16).ceil() as u32;
        for i in 1..rows {
            bar(
                &mut steel,
                Vec3::new(length, 0.009, 0.009),
                Vec3::new(0.0, height * i as f32 / rows as f32, 0.0),
            );
        }
    }
    // Retain a ground plane for camera translation and the fence's shadows.
    let stands = Some(
        Mesh::from(Cuboid::new(
            half.x * 2.0 + 130.0,
            0.04,
            half.z * 2.0 + 130.0,
        ))
        .transformed_by(Transform::from_xyz(center.x, low.y - 0.02, center.z)),
    );
    for (index, (mesh, color, emission)) in [
        (steel, Color::srgb(0.28, 0.32, 0.36), 0.0),
        (stands, Color::srgb(0.025, 0.033, 0.05), 0.0),
    ]
    .into_iter()
    .enumerate()
    {
        let mut entity = commands.spawn((
            Stadium,
            Mesh3d(meshes.add(mesh.expect("venue batch"))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                emissive: color.to_linear() * emission,
                perceptual_roughness: 0.8,
                ..default()
            })),
            Transform::default(),
        ));
        if index == 1 {
            // Venue shells must not occlude the directional lights used as
            // indoor field lamps. Fence posts still cast their own shadows.
            entity.insert(bevy::light::NotShadowCaster);
        }
    }
}
