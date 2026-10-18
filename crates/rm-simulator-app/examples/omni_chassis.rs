// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! CAD-free physical omni fixture. Arguments: [output.png] [oblique|top|side]
//! [flat|uneven] [closed|bare|open] [aimed]. Uses the same config, wheel dynamics and renderer as the app.
use bevy::{
    prelude::*,
    render::view::screenshot::{Screenshot, save_to_disk},
};
use rm_simulator_physics::{
    Pose, TargetFrames, Team, WorldPhysics,
    chassis::{ChassisConfig, ChassisSnapshot},
    tick_ns,
};
use rm_simulator_render::{
    chassis::{ArmorOptics, ChassisVisualsPlugin, RobotModel},
    sync::*,
    *,
};

#[derive(Resource)]
struct Capture {
    path: Option<String>,
    target: Handle<Image>,
    frames: u32,
}
fn pose(p: Pose) -> PoseFlu {
    PoseFlu {
        translation_m: p.translation_m,
        rotation_wxyz: p.rotation_wxyz,
    }
}
fn appearance(c: &ChassisSnapshot) -> ChassisAppearance {
    let cfg = &c.config;
    let body_q = bevy::math::DQuat::from_array([
        c.pose.rotation_wxyz[1],
        c.pose.rotation_wxyz[2],
        c.pose.rotation_wxyz[3],
        c.pose.rotation_wxyz[0],
    ]);
    let aim_q = bevy::math::DQuat::from_array([
        c.turret.rotation_wxyz[1],
        c.turret.rotation_wxyz[2],
        c.turret.rotation_wxyz[3],
        c.turret.rotation_wxyz[0],
    ]);
    let direction = body_q.inverse() * (aim_q * bevy::math::DVec3::X);
    let yaw = body_q * bevy::math::DQuat::from_rotation_z(direction.y.atan2(direction.x));
    ChassisAppearance {
        source_joints: vec![],
        model: RobotModel::Omni,
        armor_pattern: armor::ArmorPattern::Three,
        mecanum: false,
        hp_fraction: 1.0,
        id: c.id,
        team: TeamColor::Blue,
        pose: pose(c.pose),
        body_half_m: cfg.body_half_m.map(|v| v as f32),
        body_shell_visible: true,
        fold_angle_rad: 0.,
        fold_collision_parts: vec![],
        frame_beam_step_m: cfg.frame_beam_step_m() as f32,
        body_outline_m: cfg
            .body_outline_m()
            .iter()
            .map(|p| p.map(|v| v as f32))
            .collect(),
        bumper_base_prisms: if cfg.omni_frame.is_some() {
            cfg.body_outline_m().len()
        } else {
            0
        },
        bumper_prisms_m: cfg
            .bumper_prisms_m()
            .iter()
            .map(|p| p.map(|p| p.map(|v| v as f32)))
            .collect(),
        wheels: c
            .wheels
            .iter()
            .zip(&cfg.wheel_hubs_m)
            .enumerate()
            .map(|(index, (w, hub))| {
                let yaw = hub[1].atan2(hub[0]) - std::f64::consts::FRAC_PI_2;
                let q = bevy::math::DQuat::from_array([
                    c.pose.rotation_wxyz[1],
                    c.pose.rotation_wxyz[2],
                    c.pose.rotation_wxyz[3],
                    c.pose.rotation_wxyz[0],
                ]);
                let r = q
                    * bevy::math::DQuat::from_rotation_z(yaw)
                    * bevy::math::DQuat::from_rotation_y(-w.spin_rad);
                let mounts = cfg
                    .suspension_mounts_m(index, w.suspension_compression_m)
                    .unwrap();
                let position = bevy::math::DVec3::from_array(c.pose.translation_m);
                let anchor = position + q * bevy::math::DVec3::from_array(mounts[0]);
                let lower = position + q * bevy::math::DVec3::from_array(mounts[1]);
                WheelAppearance {
                    pose: PoseFlu {
                        translation_m: w.hub_m,
                        rotation_wxyz: [r.w, r.x, r.y, r.z],
                    },
                    suspension_anchor_m: anchor.to_array(),
                    suspension_lower_m: lower.to_array(),
                    radius_m: cfg.wheel_radius_m as f32,
                    width_m: cfg.wheel_width_m as f32,
                }
            })
            .collect(),
        armor: cfg
            .armor_faces()
            .iter()
            .enumerate()
            .map(|(plate, p)| ArmorAppearance {
                optics: None,
                plate_visible: true,
                bracket_prisms_m: cfg
                    .armor_bracket_prisms_m(plate)
                    .iter()
                    .map(|p| p.map(|p| p.map(|v| v as f32)))
                    .collect(),
                local: pose(*p),
                hit_flash: false,
            })
            .collect(),
        yaw_stage: PoseFlu {
            translation_m: c.turret.translation_m,
            rotation_wxyz: [yaw.w, yaw.x, yaw.y, yaw.z],
        },
        turret: pose(c.turret),
        turret_half_m: cfg.turret_half_m.map(|v| v as f32),
        pivot_above_body_m: (cfg.turret_center_m[2] - cfg.body_half_m[2]) as f32,
        barrel_length_m: rm_simulator_physics::chassis::MUZZLE_FORWARD_M as f32,
        defeated: false,
    }
}
fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                visible: false,
                ..default()
            }),
            ..default()
        }))
        .add_plugins((
            SceneSyncPlugin,
            ChassisVisualsPlugin {
                rendering: RenderingConfig::field(),
                armor: ArmorOptics {
                    housing_half_m: [0.0095, 0.0705, 0.0675],
                    face_size_m: [0.128, 0.113],
                    light_span_m: 0.130,
                    light_length_m: 0.056,
                },
            },
        ))
        .add_systems(Startup, setup)
        .add_systems(Update, capture)
        .run();
}
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    mut scene: ResMut<SceneInput>,
) {
    let args: Vec<_> = std::env::args().collect();
    let uneven = args.get(3).is_some_and(|s| s == "uneven");
    let cfg = ChassisConfig::default();
    let mut physics = WorldPhysics::new(&[], 0.0);
    let [x, y] = cfg.wheel_hubs_m[0];
    let block = ([x - 0.06, y - 0.06, 0.0], [x + 0.06, y + 0.06, 0.035]);
    if uneven {
        let (lo, hi) = block;
        physics
            .add_static_mesh(
                vec![
                    [lo[0], lo[1], hi[2]],
                    [hi[0], lo[1], hi[2]],
                    [hi[0], hi[1], hi[2]],
                    [lo[0], hi[1], hi[2]],
                ],
                vec![[0, 1, 2], [0, 2, 3]],
            )
            .unwrap();
        let center = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, hi[2] / 2.0];
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(
                (hi[1] - lo[1]) as f32,
                hi[2] as f32,
                (hi[0] - lo[0]) as f32,
            ))),
            MeshMaterial3d(materials.add(Color::srgb(0.5, 0.36, 0.17))),
            Transform::from_translation(flu_position(center)),
        ));
    }
    physics
        .add_chassis(
            Team::Blue,
            cfg.clone(),
            Pose::at([0.0, 0.0, cfg.rest_height_m() + 0.04]),
        )
        .unwrap();
    if args.get(5).is_some_and(|s| s == "aimed") {
        physics
            .command_chassis(
                0,
                rm_simulator_physics::chassis::ChassisCommand {
                    aim_yaw_rad: 0.6,
                    aim_pitch_rad: 0.35,
                    ..default()
                },
            )
            .unwrap();
    }
    let frames = TargetFrames::new(vec![]);
    for tick in 0..384 {
        physics.step(tick * tick_ns(), &frames).unwrap();
    }
    let c = physics.chassis_snapshots().remove(0);
    println!(
        "wheel loads N: {:?}; compression mm: {:?}",
        c.wheels
            .iter()
            .map(|w| w.contact.map(|c| c.load_n))
            .collect::<Vec<_>>(),
        c.wheels
            .iter()
            .map(|w| w.suspension_compression_m * 1000.0)
            .collect::<Vec<_>>()
    );
    let mut visual = appearance(&c);
    if args.get(4).is_some_and(|s| s == "open") {
        visual.body_shell_visible = false;
    }
    if args.get(4).is_some_and(|s| s == "bare" || s == "open") {
        for armor in &mut visual.armor {
            armor.plate_visible = false;
        }
    }
    scene.0 = Some(SceneState {
        chassis: vec![visual],
        ..default()
    });
    let target = images.add(Image::new_target_texture(
        1200,
        1000,
        bevy::render::render_resource::TextureFormat::Rgba8UnormSrgb,
        None,
    ));
    let eye = match args.get(2).map(String::as_str) {
        Some("top") => Vec3::new(0.0, 1.9, 0.001),
        Some("side") => Vec3::new(1.3, 0.24, 0.001),
        _ => Vec3::new(1.0, 0.9, -1.15),
    };
    let mut camera = commands.spawn((
        Camera3d::default(),
        bevy::camera::RenderTarget::Image(target.clone().into()),
        Transform::from_translation(eye).looking_at(Vec3::new(0.0, 0.20, 0.0), Vec3::Y),
    ));
    camera_appearance(&mut camera, RenderingConfig::field());
    commands.insert_resource(Capture {
        path: args.get(1).cloned(),
        target,
        frames: 0,
    });
    for (eye, lux) in [
        (Vec3::new(2.0, 5.0, -3.0), 8500.0),
        (Vec3::new(-3.0, 2.0, 1.0), 5000.0),
    ] {
        commands.spawn((
            DirectionalLight {
                illuminance: lux,
                shadow_maps_enabled: true,
                ..default()
            },
            Transform::from_translation(eye).looking_at(Vec3::ZERO, Vec3::Y),
        ));
    }
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(10.0, 0.02, 10.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.22, 0.25, 0.30))),
        Transform::from_xyz(0.0, -0.01, 0.0),
    ));
}
fn capture(mut commands: Commands, mut capture: ResMut<Capture>) {
    capture.frames += 1;
    if capture.frames == 100
        && let Some(path) = capture.path.take()
    {
        commands
            .spawn(Screenshot::image(capture.target.clone()))
            .observe(save_to_disk(path))
            .observe(
                |_: On<bevy::render::view::screenshot::ScreenshotCaptured>,
                 mut exit: MessageWriter<AppExit>| {
                    exit.write(AppExit::Success);
                },
            );
    }
}
