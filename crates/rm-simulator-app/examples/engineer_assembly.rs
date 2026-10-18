// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Standalone Cartesian assembly workbench, not a multiplayer match or Heavy admission.
//! `cargo run -p rm-simulator-app --example engineer_assembly -- --help`
use bevy::render::{Render, RenderApp, RenderSystems, render_resource::PipelineCache};
use bevy::{
    prelude::*,
    render::view::screenshot::{Screenshot, save_to_disk},
};
use clap::Parser;
use rm_simulator_physics::{Pose, manipulator::Arm, tick_ns};
use rm_simulator_render::{PoseFlu, apply_pose, flu_position};
use rm_simulator_world::gameplay::rmuc_2027::manipulation::{Attempt, Sensors, Step};
use std::{
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
};
#[derive(Resource, Clone, Default)]
struct CaptureReady(Arc<AtomicBool>);
struct CapturePlugin;
impl Plugin for CapturePlugin {
    fn build(&self, app: &mut App) {
        let ready = CaptureReady::default();
        app.insert_resource(ready.clone());
        app.sub_app_mut(RenderApp)
            .insert_resource(ready)
            .add_systems(Render, pipeline_ready.in_set(RenderSystems::Cleanup));
    }
}
fn pipeline_ready(cache: Res<PipelineCache>, ready: Res<CaptureReady>) {
    ready
        .0
        .store(cache.waiting_pipelines().count() == 0, Ordering::Release);
}
#[allow(dead_code)]
#[path = "../src/robot_assets.rs"]
mod robot_assets;

#[derive(Parser, Resource)]
#[command(about = "Cartesian Engineer assembly workbench (prototype geometry, no match rewards)")]
struct Options {
    /// Initial difficulty, 1–3. Paired-core level four is evaluator-only for now.
    #[arg(long, default_value_t=1, value_parser=clap::value_parser!(u8).range(1..=3))]
    difficulty: u8,
    /// Verified PIE source package: chassis/wheels only; arm uses a generic IK proxy.
    #[arg(long)]
    robot_assets: Option<PathBuf>,
    /// Demonstrate the full sequence with rate-limited Cartesian movements.
    #[arg(long)]
    demo: bool,
    /// Run the same demonstration without rendering, print state and exit.
    #[arg(long)]
    headless: bool,
    /// Save the completed demonstration and exit (implies --demo).
    #[arg(long)]
    screenshot: Option<PathBuf>,
}
const SOCKET: [f64; 3] = [-0.05, 0.2, 0.6];
const ENTRANCE_X: f64 = 0.05;
const SPEED: f64 = 0.12;
const RATE: f64 = 0.6;
const ARM: Arm = Arm {
    shoulder_m: [0.55, 0., 0.35],
    lengths_m: [0.4, 0.4],
};
#[derive(Resource)]
struct Workbench {
    task: Attempt,
    tool: Pose,
    angles: [f64; 3], // P, Q, off-guide roll
    attached: bool,
    tick: u64,
    paused: bool,
    message: String,
    unreachable: bool,
    capture_frames: u32,
    confirm_requested: bool,
}
impl Workbench {
    fn new(d: u8) -> Self {
        Self {
            task: Attempt::new(d, [30f64.to_radians(); 2], 0).unwrap(),
            tool: Pose::at([0.09, 0.2, 0.6]),
            angles: [0.; 3],
            attached: true,
            tick: 0,
            paused: false,
            message: String::new(),
            unreachable: false,
            capture_frames: 0,
            confirm_requested: false,
        }
    }
    fn step(&mut self, movement: [f64; 3], rotation: [f64; 3]) {
        if self.paused || self.task.failure.is_some() || self.task.completed() {
            return;
        }
        let dt = tick_ns() as f64 * 1e-9;
        let mut tool = self.tool;
        let angles = std::array::from_fn(|i| self.angles[i] + rotation[i] * RATE * dt);
        for (i, v) in movement.into_iter().enumerate() {
            tool.translation_m[i] += v * SPEED * dt;
        }
        let p = Quat::from_rotation_y(-angles[0] as f32)
            * Quat::from_rotation_x(angles[1] as f32)
            * Quat::from_rotation_z(angles[2] as f32);
        let q = DQuat::from_xyzw(p.x as f64, p.y as f64, p.z as f64, p.w as f64).normalize();
        tool.rotation_wxyz = [q.w, q.x, q.y, q.z];
        self.unreachable = ARM.solve(tool).is_err() || tool.translation_m[2] < 0.15;
        if !self.unreachable {
            self.tool = tool;
            self.angles = angles;
        }
        self.tick += 1;
        let [x, y, z] = self.tool.translation_m;
        let s = Sensors {
            attached: self.attached,
            insertion_m: ENTRANCE_X - x,
            lift_m: z - SOCKET[2],
            p_rad: self.angles[0],
            q_rad: self.angles[1],
            axis_error_rad: self.angles[2].abs(),
            lateral_error_m: (y - SOCKET[1]).abs(),
        };
        self.task
            .observe(self.tick * tick_ns(), [s, Sensors::default()], true, true)
            .unwrap();
    }
    fn confirm(&mut self) {
        self.message = match self.task.confirm(0) {
            Ok(()) => "Assembly confirmed".into(),
            Err(reason) => reason.into(),
        };
    }
    fn demo(&mut self) {
        let mut target = SOCKET;
        let mut angles = [0.; 3];
        match self.task.cores[0].step {
            Step::Approach => target[0] = ENTRANCE_X,
            Step::Insert => {}
            Step::Lift => target[2] += 0.1,
            Step::Raise => {
                target[2] += 0.1;
                angles[0] = std::f64::consts::FRAC_PI_2;
            }
            Step::Align => {
                target[2] += 0.1;
                angles = [std::f64::consts::FRAC_PI_2, self.task.target_q_rad[0], 0.];
            }
            Step::Ready => {
                self.confirm();
                return;
            }
            Step::Confirmed => return,
        }
        let dt = tick_ns() as f64 * 1e-9;
        let movement = std::array::from_fn(|i| {
            ((target[i] - self.tool.translation_m[i]) / (SPEED * dt)).clamp(-1., 1.)
        });
        let rotation =
            std::array::from_fn(|i| ((angles[i] - self.angles[i]) / (RATE * dt)).clamp(-1., 1.));
        self.step(movement, rotation);
    }
}
use bevy::math::DQuat;
#[derive(Resource, Default)]
struct CaptureImage(Option<Handle<Image>>);
#[derive(Component)]
struct Segment(u8);
#[derive(Component)]
struct Tool;
#[derive(Component)]
struct Status;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut options = Options::parse();
    options.demo |= options.screenshot.is_some();
    if options.headless {
        let mut w = Workbench::new(options.difficulty);
        for _ in 0..10000 {
            w.demo();
            if w.task.completed() {
                break;
            }
        }
        println!("{}", serde_json::to_string_pretty(&w.task)?);
        if !w.task.completed() {
            return Err("assembly demonstration did not complete".into());
        }
        return Ok(());
    }
    if let Some(path) = &mut options.robot_assets {
        *path = path.canonicalize()?;
        robot_assets::verify(path)?;
        let cfg = robot_assets::source_chassis(path)?.ok_or("source chassis required")?;
        if cfg
            .articulation
            .as_ref()
            .is_none_or(|a| a.profile != "pie-tunnel-heavy-v1")
        {
            return Err("workbench currently supports the PIE Engineer chassis only".into());
        }
    }
    let asset_path = options
        .robot_assets
        .as_ref()
        .map(|p| p.display().to_string())
        .unwrap_or_else(|| "assets".into());
    let offscreen = options.screenshot.is_some();
    let mut plugins = DefaultPlugins
        .set(AssetPlugin {
            file_path: asset_path,
            ..default()
        })
        .set(WindowPlugin {
            primary_window: Some(Window {
                title: "Engineer Cartesian assembly workbench".into(),
                resolution: (1280, 800).into(),
                ..default()
            }),
            ..default()
        });
    if offscreen {
        plugins = plugins.disable::<bevy::winit::WinitPlugin>();
    }
    let mut app = App::new();
    app.init_resource::<CaptureImage>()
        .insert_resource(Workbench::new(options.difficulty))
        .insert_resource(options)
        .add_plugins(plugins);
    if offscreen {
        app.add_plugins(bevy::app::ScheduleRunnerPlugin::run_loop(
            std::time::Duration::from_secs_f64(1. / 60.),
        ));
    }
    app.add_plugins(CapturePlugin)
        .insert_resource(bevy::winit::WinitSettings::continuous())
        .insert_resource(Time::<Fixed>::from_hz(128.))
        .insert_resource(ClearColor(Color::srgb(0.035, 0.045, 0.055)))
        .add_systems(Startup, setup)
        .add_systems(FixedUpdate, advance)
        .add_systems(Update, (actions, hide_source_arm, draw, capture).chain())
        .run();
    Ok(())
}
fn block(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    position: [f64; 3],
    size: [f32; 3],
    color: Color,
) -> Entity {
    commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(size[0], size[1], size[2]))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                perceptual_roughness: 0.6,
                ..default()
            })),
            Transform::from_translation(flu_position(position)),
        ))
        .id()
}
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    assets: Res<AssetServer>,
    options: Res<Options>,
    mut images: ResMut<Assets<Image>>,
    mut capture: ResMut<CaptureImage>,
) {
    let target = if options.screenshot.is_some() {
        let image = images.add(Image::new_target_texture(
            1280,
            800,
            bevy::render::render_resource::TextureFormat::Rgba8UnormSrgb,
            None,
        ));
        capture.0 = Some(image.clone());
        bevy::camera::RenderTarget::Image(image.into())
    } else {
        default()
    };
    commands.spawn((
        target,
        Camera3d::default(),
        bevy::camera::Hdr,
        bevy::camera::Exposure { ev100: 9. },
        IsDefaultUiCamera,
        Transform::from_translation(flu_position([1.45, 1.65, 1.4]))
            .looking_at(flu_position([0.25, 0.1, 0.42]), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(1., 3., 2.).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    block(
        &mut commands,
        &mut meshes,
        &mut materials,
        [0.2, 0., -0.04],
        [2.4, 0.08, 2.4],
        Color::srgb(0.12, 0.14, 0.16),
    );
    // Fixture is intentionally schematic: source Tech Core motion is not bound yet.
    block(
        &mut commands,
        &mut meshes,
        &mut materials,
        [-0.24, 0.2, 0.35],
        [0.22, 0.7, 0.18],
        Color::srgb(0.2, 0.23, 0.27),
    );
    block(
        &mut commands,
        &mut meshes,
        &mut materials,
        [-0.05, 0.2, 0.6],
        [0.02, 0.04, 0.02],
        Color::srgb(0.1, 0.8, 0.8),
    );
    if options.robot_assets.is_some() {
        commands.spawn((
            WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset("neutral.glb"))),
            Transform::from_translation(flu_position([0.55, 0., 0.132]))
                .with_rotation(Quat::from_rotation_y(std::f32::consts::PI)),
        ));
        // Package armor was removed for runtime replacement; add simple workbench housings.
        for p in [
            [0.88, 0., 0.16],
            [0.22, 0., 0.16],
            [0.55, 0.29, 0.16],
            [0.55, -0.29, 0.16],
        ] {
            block(
                &mut commands,
                &mut meshes,
                &mut materials,
                p,
                [0.08, 0.08, 0.08],
                Color::srgb(0.45, 0.05, 0.04),
            );
        }
    } else {
        block(
            &mut commands,
            &mut meshes,
            &mut materials,
            [0.55, 0., 0.14],
            [0.5, 0.22, 0.55],
            Color::srgb(0.16, 0.18, 0.2),
        );
    }
    block(
        &mut commands,
        &mut meshes,
        &mut materials,
        [0.55, 0., 0.275],
        [0.08, 0.15, 0.08],
        Color::srgb(0.25, 0.28, 0.3),
    );
    for i in 0..2 {
        let e = block(
            &mut commands,
            &mut meshes,
            &mut materials,
            [0.; 3],
            [0.035, 1., 0.035],
            Color::srgb(0.55, 0.59, 0.63),
        );
        commands.entity(e).insert(Segment(i));
    }
    let tool = block(
        &mut commands,
        &mut meshes,
        &mut materials,
        [0.; 3],
        [0.065, 0.065, 0.065],
        Color::srgb(0.9, 0.57, 0.08),
    );
    commands.entity(tool).insert(Tool);
    commands.spawn((
        Text::new(""),
        TextFont {
            font_size: bevy::text::FontSize::Px(19.),
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            top: px(16.),
            left: px(16.),
            ..default()
        },
        Status,
    ));
}
fn hide_source_arm(mut q: Query<(&Name, &mut Visibility), Added<Name>>) {
    for (name, mut visibility) in &mut q {
        if [
            "link/shoulder",
            "link/elbow",
            "link/camera-lift",
            "link/uwb",
        ]
        .contains(&name.as_str())
        {
            *visibility = Visibility::Hidden;
        }
    }
}
fn actions(
    keys: Res<ButtonInput<KeyCode>>,
    mut w: ResMut<Workbench>,
    mut exit: MessageWriter<AppExit>,
) {
    for (key, d) in [
        (KeyCode::Digit1, 1),
        (KeyCode::Digit2, 2),
        (KeyCode::Digit3, 3),
    ] {
        if keys.just_pressed(key) {
            *w = Workbench::new(d);
        }
    }
    if keys.just_pressed(KeyCode::Space) {
        w.paused = !w.paused;
    }
    if keys.just_pressed(KeyCode::KeyG) {
        w.attached = !w.attached;
    }
    if keys.just_pressed(KeyCode::KeyE) {
        w.confirm_requested = true;
    }
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}
fn advance(keys: Res<ButtonInput<KeyCode>>, options: Res<Options>, mut w: ResMut<Workbench>) {
    if options.demo {
        w.demo();
        return;
    }
    let axis = |positive, negative| {
        f64::from(u8::from(keys.pressed(positive))) - f64::from(u8::from(keys.pressed(negative)))
    };
    w.step(
        [
            axis(KeyCode::KeyS, KeyCode::KeyW),
            axis(KeyCode::KeyA, KeyCode::KeyD),
            axis(KeyCode::KeyR, KeyCode::KeyF),
        ],
        [
            axis(KeyCode::ArrowUp, KeyCode::ArrowDown),
            axis(KeyCode::ArrowLeft, KeyCode::ArrowRight),
            axis(KeyCode::KeyZ, KeyCode::KeyX),
        ],
    );
    if !w.paused && std::mem::take(&mut w.confirm_requested) {
        w.confirm();
    }
}
fn draw(
    w: Res<Workbench>,
    mut segments: Query<(&Segment, &mut Transform), Without<Tool>>,
    mut tool: Query<&mut Transform, (With<Tool>, Without<Segment>)>,
    mut text: Query<&mut Text, With<Status>>,
    mut gizmos: Gizmos,
) {
    let arm = ARM.solve(w.tool).unwrap();
    let points = [arm.shoulder_m, arm.elbow_m, arm.tool.translation_m];
    for (segment, mut t) in &mut segments {
        let i = usize::from(segment.0);
        let a = flu_position(points[i]);
        let b = flu_position(points[i + 1]);
        t.translation = (a + b) * 0.5;
        t.rotation = Quat::from_rotation_arc(Vec3::Y, (b - a).normalize());
        t.scale.y = (b - a).length();
    }
    for mut t in &mut tool {
        apply_pose(
            &mut t,
            PoseFlu {
                translation_m: w.tool.translation_m,
                rotation_wxyz: w.tool.rotation_wxyz,
            },
        );
    }
    let color = if w.task.completed() {
        Color::srgb(0.1, 1., 0.3)
    } else {
        Color::srgb(0.1, 0.8, 0.9)
    };
    gizmos.line(
        flu_position([ENTRANCE_X, 0.2, 0.6]),
        flu_position(SOCKET),
        color,
    );
    gizmos.line(flu_position(SOCKET), flu_position([-0.05, 0.2, 0.7]), color);
    gizmos.sphere(
        Isometry3d::from_translation(flu_position(SOCKET)),
        0.012,
        color,
    );
    let [x, y, z] = w.tool.translation_m;
    for mut t in &mut text {
        t.0 = format!(
            "ASSEMBLY WORKBENCH | difficulty {} | {:?} | {:.2}s{}\n\
            WASD: horizontal  R/F: height  Arrows: P/Q  Z/X: off-axis roll\n\
            E: confirm  G: release/grip  Space: pause  1/2/3: restart difficulty\n\
            Tool [{x:.3}, {y:.3}, {z:.3}] m  P {:.1}°  Q {:.1}° (target {:.1}°)\n\
            {} {} {}\n\
            Generic position-controlled arm + schematic socket; no match rewards",
            w.task.difficulty,
            w.task.cores[0].step,
            w.tick as f64 / 128.,
            if w.paused { " PAUSED" } else { "" },
            w.angles[0].to_degrees(),
            w.angles[1].to_degrees(),
            w.task.target_q_rad[0].to_degrees(),
            if w.task.completed() {
                "COMPLETE"
            } else if w.unreachable {
                "UNREACHABLE TARGET"
            } else {
                ""
            },
            w.message,
            w.task
                .failure
                .map_or_else(String::new, |f| format!("Failed: {f:?}"))
        );
    }
}
fn capture(
    image: Res<CaptureImage>,
    options: Res<Options>,
    ready: Res<CaptureReady>,
    mut w: ResMut<Workbench>,
    mut commands: Commands,
) {
    if !ready.0.load(Ordering::Acquire) {
        w.capture_frames = 0;
        return;
    }
    if let Some(path) = &options.screenshot
        && w.task.completed()
    {
        w.capture_frames += 1;
        if w.capture_frames == 60 {
            commands
                .spawn(
                    image
                        .0
                        .clone()
                        .map_or_else(Screenshot::primary_window, Screenshot::image),
                )
                .observe(save_to_disk(path.clone()))
                .observe(
                    |_: On<bevy::render::view::screenshot::ScreenshotCaptured>,
                     mut exit: MessageWriter<AppExit>| {
                        exit.write(AppExit::Success);
                    },
                );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rate_limit_pause_and_unreachable_target() {
        let mut w = Workbench::new(1);
        let x = w.tool.translation_m[0];
        w.step([1., 0., 0.], [0.; 3]);
        assert!((w.tool.translation_m[0] - x - SPEED / 128.).abs() < 1e-12);
        w.paused = true;
        let tick = w.tick;
        let tool = w.tool;
        w.step([1.; 3], [1.; 3]);
        assert_eq!(w.tick, tick);
        assert_eq!(w.tool, tool);
        w.paused = false;
        // Move away from the fixture until the arm's reach, without entering it.
        for _ in 0..3000 {
            w.step([0., 1., 0.], [0.; 3]);
        }
        assert!(w.unreachable);
        assert!(ARM.solve(w.tool).is_ok());
    }
    #[test]
    fn cartesian_demo_completes_each_interactive_difficulty() {
        for d in 1..=3 {
            let mut w = Workbench::new(d);
            for _ in 0..1280 {
                w.demo();
                if w.task.completed() {
                    break;
                }
            }
            assert!(w.task.completed(), "difficulty {d}: {:?}", w.task);
            assert_eq!(w.task.failure, None);
            assert!(ARM.solve(w.tool).is_ok());
        }
    }
}
