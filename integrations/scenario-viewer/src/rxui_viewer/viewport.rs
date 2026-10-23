// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! One caller-driven Bevy render app; RXUI samples its GPU target directly.
use super::importer::Project;
use crate::simulation::{HALF_EXTENTS_M, KEYS, Simulation};
use bevy::{
    prelude::*,
    render::{
        RenderApp, RenderPlugin,
        render_asset::RenderAssets,
        renderer::{RenderAdapter, RenderDevice, RenderInstance, RenderQueue},
        texture::GpuImage,
    },
};
use rm_sim_render::generic::{PoseFlu, VisualId, VisualPlugin, Visuals};
use rxui::astrelis::{GraphicsContext, wgpu};

pub struct Viewport {
    pub app: App,
    target: Handle<Image>,
    camera: Entity,
    size: [u32; 2],
    center: Vec3,
    radius: f32,
    imported: Vec<(VisualId, Handle<Mesh>, Handle<StandardMaterial>)>,
}

impl Viewport {
    pub fn new(plant: &Simulation) -> Self {
        let mut app = App::new();
        app.add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: None,
                    exit_condition: bevy::window::ExitCondition::DontExit,
                    ..default()
                })
                .set(RenderPlugin {
                    synchronous_pipeline_compilation: true,
                    ..default()
                })
                .disable::<bevy::winit::WinitPlugin>()
                .disable::<bevy::render::pipelined_rendering::PipelinedRenderingPlugin>(),
        )
        .add_plugins(VisualPlugin)
        .insert_resource(ClearColor(Color::srgb(0.035, 0.05, 0.075)));
        while app.plugins_state() != bevy::app::PluginsState::Ready {
            bevy::tasks::tick_global_task_pools_on_main_thread();
            std::thread::yield_now();
        }
        app.finish();
        app.cleanup();
        let size = [800, 560];
        let target =
            app.world_mut()
                .resource_mut::<Assets<Image>>()
                .add(Image::new_target_texture(
                    size[0],
                    size[1],
                    wgpu::TextureFormat::Rgba8UnormSrgb,
                    None,
                ));
        let camera = app
            .world_mut()
            .spawn((
                Camera3d::default(),
                bevy::camera::RenderTarget::Image(target.clone().into()),
                Transform::from_xyz(3.5, 2.8, 4.).looking_at(Vec3::new(0., 0.55, 0.), Vec3::Y),
            ))
            .id();
        app.world_mut().spawn((
            DirectionalLight {
                illuminance: 12000.,
                shadow_maps_enabled: false,
                ..default()
            },
            Transform::from_xyz(3., 5., 4.).looking_at(Vec3::ZERO, Vec3::Y),
        ));
        app.insert_resource(GlobalAmbientLight {
            color: Color::WHITE,
            brightness: 250.,
            ..default()
        });
        let mesh = app
            .world_mut()
            .resource_mut::<Assets<Mesh>>()
            .add(Cuboid::new(1., 1., 1.));
        let mut commands = app.world_mut().commands();
        commands.queue(move |world: &mut World| {
            let colors = [
                Color::srgb(0.98, 0.48, 0.16),
                Color::srgb(0.16, 0.24, 0.32),
                Color::srgb(0.12, 0.7, 0.76),
            ];
            world.resource_scope(|world, mut visuals: Mut<Visuals>| {
                for i in 0..4 {
                    let material =
                        world
                            .resource_mut::<Assets<StandardMaterial>>()
                            .add(StandardMaterial {
                                base_color: colors[if i == 3 { 1 } else { i }],
                                perceptual_roughness: 0.8,
                                ..default()
                            });
                    let (id, half, pose) = if i < 3 {
                        (KEYS[i].0, HALF_EXTENTS_M[i], PoseFlu::default())
                    } else {
                        (
                            10,
                            [4., 4., 0.025],
                            PoseFlu {
                                translation_m: [0., 0., -0.025],
                                ..default()
                            },
                        )
                    };
                    visuals.spawn_mesh(
                        &mut world.commands(),
                        VisualId(id),
                        mesh.clone(),
                        material,
                        pose,
                        Vec3::new(
                            2. * half[1] as f32,
                            2. * half[2] as f32,
                            2. * half[0] as f32,
                        ),
                    );
                }
            });
        });
        app.world_mut().flush();
        let mut this = Self {
            app,
            target,
            camera,
            size,
            center: Vec3::new(0., 0.55, 0.),
            radius: 5.,
            imported: Vec::new(),
        };
        // Initial uploads and pipeline setup, without advancing the plant.
        for _ in 0..3 {
            this.draw(plant, size, 0.65, true);
        }
        this
    }

    pub fn graphics(&self) -> GraphicsContext {
        let world = self.app.sub_app(RenderApp).world();
        GraphicsContext::from_wgpu(
            world.resource::<RenderInstance>().clone().into_inner(),
            world.resource::<RenderAdapter>().clone().into_inner(),
            world.resource::<RenderDevice>().wgpu_device().clone(),
            world.resource::<RenderQueue>().clone().into_inner(),
        )
    }

    pub fn draw(&mut self, plant: &Simulation, size: [u32; 2], yaw: f32, floor: bool) {
        let resized = size != self.size;
        if resized {
            self.app
                .world_mut()
                .resource_mut::<Assets<Image>>()
                .get_mut(&self.target)
                .unwrap()
                .resize(bevy::render::render_resource::Extent3d {
                    width: size[0],
                    height: size[1],
                    depth_or_array_layers: 1,
                });
            self.size = size;
        }
        let mut visuals = self.app.world_mut().resource_mut::<Visuals>();
        for key in KEYS {
            let p = plant.pose(key);
            visuals.set_pose(
                VisualId(key.0),
                PoseFlu {
                    translation_m: p.translation_m,
                    rotation_wxyz: p.rotation_wxyz,
                },
            );
        }
        visuals.set_visibility(VisualId(10), floor);
        let eye = self.center + self.radius * Vec3::new(yaw.sin(), 0.56, yaw.cos());
        *self
            .app
            .world_mut()
            .get_mut::<Transform>(self.camera)
            .unwrap() = Transform::from_translation(eye).looking_at(self.center, Vec3::Y);
        self.app.update();
        // Asset modifications are extracted before rendering. A second update
        // makes the replacement target available before RXUI binds its view.
        if resized {
            self.app.update();
        }
    }

    /// Rebuild only after an explicit import edit, never during ordinary painting.
    pub fn import_scene(
        &mut self,
        project: &Project,
        selected: u64,
        proxy: bool,
    ) -> Result<(), String> {
        use rm_sim_engine::model::{
            BodyKind, BodyState, ColliderDescription, ContactMaterial, RigidBodyDescription,
        };
        use rm_sim_physics::{WorldPhysics, cooking::CookedRigidBody, runtime::BodyKey};
        use rm_sim_render::generic::flu_position;
        let mut descriptions = Vec::new();
        if proxy {
            for group in &project.groups {
                let colliders = project
                    .proxies(group)?
                    .into_iter()
                    .map(|(pose, shape)| {
                        ColliderDescription::new(
                            shape,
                            pose,
                            ContactMaterial {
                                friction: 0.5,
                                restitution: 0.,
                                memberships: u32::MAX,
                                filter: u32::MAX,
                            },
                        )
                    })
                    .collect::<Result<Vec<_>, _>>()
                    .map_err(str::to_owned)?;
                let body = CookedRigidBody::new(
                    RigidBodyDescription::new(BodyKind::Fixed, colliders).map_err(str::to_owned)?,
                )
                .map_err(str::to_owned)?;
                let mut world = WorldPhysics::new(&[], 0.);
                world
                    .install_body(BodyKey(1), &body, BodyState::at(Default::default()))
                    .map_err(str::to_owned)?;
                let (vertices, triangles) = world.static_geometry_snapshot(false).into_triangles();
                descriptions.push((group.id, group.pose, vertices, triangles));
            }
        } else {
            for part in &project.parts {
                descriptions.push((
                    part.group,
                    part.pose,
                    part.vertices_m.clone(),
                    part.triangles.clone(),
                ));
            }
        }
        // Prepare all geometry before changing the visible scene.
        let mut lo = Vec3::splat(f32::INFINITY);
        let mut hi = -lo;
        let mut meshes = Vec::new();
        for (group, pose, vertices, triangles) in descriptions {
            for &v in &vertices {
                let p = flu_position(super::importer::transform(pose, v).to_array());
                lo = lo.min(p);
                hi = hi.max(p);
            }
            let positions: Vec<_> = vertices
                .iter()
                .map(|&v| flu_position(v).to_array())
                .collect();
            let mut mesh = Mesh::new(
                bevy::mesh::PrimitiveTopology::TriangleList,
                bevy::asset::RenderAssetUsages::default(),
            );
            mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
            mesh.insert_indices(bevy::mesh::Indices::U32(
                triangles.into_iter().flatten().collect(),
            ));
            mesh.duplicate_vertices();
            mesh.compute_flat_normals();
            meshes.push((group, pose, mesh));
        }
        if !lo.is_finite() || !hi.is_finite() || !(hi - lo).length().is_finite() {
            return Err("no supported import geometry".into());
        }
        let world = self.app.world_mut();
        world.resource_scope(|world, mut visuals: Mut<Visuals>| {
            let mut commands = world.commands();
            for id in [1, 2, 3, 10] {
                visuals.remove(&mut commands, VisualId(id));
            }
            for (id, _, _) in &self.imported {
                visuals.remove(&mut commands, *id);
            }
        });
        world.flush();
        for (_, mesh, material) in self.imported.drain(..) {
            world.resource_mut::<Assets<Mesh>>().remove(mesh.id());
            world
                .resource_mut::<Assets<StandardMaterial>>()
                .remove(material.id());
        }
        for (i, (group, p, mesh)) in meshes.into_iter().enumerate() {
            let mesh = world.resource_mut::<Assets<Mesh>>().add(mesh);
            let color = if group == selected {
                Color::srgb(0.98, 0.58, 0.18)
            } else {
                Color::srgb(0.18, 0.6, 0.7)
            };
            let material = world
                .resource_mut::<Assets<StandardMaterial>>()
                .add(StandardMaterial {
                    base_color: color,
                    cull_mode: None,
                    perceptual_roughness: 0.8,
                    ..default()
                });
            let id = VisualId(1000 + i as u64);
            world.resource_scope(|world, mut visuals: Mut<Visuals>| {
                visuals.spawn_mesh(
                    &mut world.commands(),
                    id,
                    mesh.clone(),
                    material.clone(),
                    PoseFlu {
                        translation_m: p.translation_m,
                        rotation_wxyz: p.rotation_wxyz,
                    },
                    Vec3::ONE,
                );
            });
            self.imported.push((id, mesh, material));
        }
        world.flush();
        self.center = (lo + hi) * 0.5;
        self.radius = ((hi - lo).length() * 1.2).max(0.15);
        Ok(())
    }

    pub fn image(&self) -> Result<rxui::Image, Box<dyn std::error::Error>> {
        let images = self
            .app
            .sub_app(RenderApp)
            .world()
            .resource::<RenderAssets<GpuImage>>();
        let image = images
            .get(&self.target)
            .ok_or("viewport GPU image not ready")?;
        let view: &wgpu::TextureView = &image.texture_view;
        Ok(rxui::Image::from_view(view.clone(), self.size)?)
    }
}

/// Bounded backing size; minimized windows never request a zero-sized texture.
pub fn backing_size(logical: [f32; 2], scale: f32) -> [u32; 2] {
    [logical[0] - 288., logical[1] - 140.]
        .map(|v| (v.max(1.) * scale).round().clamp(64., 2048.) as u32)
}
