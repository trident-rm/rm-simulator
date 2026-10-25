// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Small standalone physical scenario viewer.
mod simulation;
use bevy::{
    input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll},
    prelude::*,
    render::view::screenshot::{Screenshot, ScreenshotCaptured},
};
use clap::Parser;
use rm_sim_render::generic::{PoseFlu, VisualId, VisualPlugin, Visuals};
use simulation::{HALF_EXTENTS_M, KEYS, Simulation};
use std::{
    path::PathBuf,
    time::{Duration, Instant},
};

#[derive(Parser, Resource, Clone)]
#[command(
    about = "Watch a falling cube and a motor-driven arm. Space pauses, N steps, R resets; drag to orbit and scroll to zoom."
)]
struct Options {
    /// Check the scene without opening a window or using a GPU.
    #[arg(long, conflicts_with_all=["screenshot","glb"])]
    headless: bool,
    /// Save an actual rendered PNG without opening a window, then exit.
    #[arg(long)]
    screenshot: Option<PathBuf>,
    /// Run this many presentations; each advances four 128 Hz ticks unless paused.
    #[arg(long)]
    frames: Option<u32>,
    /// Start paused.
    #[arg(long)]
    paused: bool,
    /// Attach the first scene of this GLB to the arm (metres, native glTF axes).
    #[arg(long)]
    glb: Option<PathBuf>,
    /// Report CPU stepping and pose timings; headless also reports app updates.
    #[arg(long)]
    benchmark: bool,
}
impl Options {
    fn frame_limit(&self) -> Option<u32> {
        self.frames.or_else(|| self.screenshot.as_ref().map(|_| 90))
    }
}
#[derive(Resource)]
struct Plant(Simulation);
#[derive(Resource, Default)]
struct PlaybackClock(Duration);
impl PlaybackClock {
    fn ticks(&mut self, delta: Duration, tick: Duration) -> u32 {
        self.0 += delta.min(Duration::from_millis(250));
        let ticks = (self.0.as_nanos() / tick.as_nanos()).min(32) as u32;
        self.0 -= tick * ticks;
        ticks
    }
    fn clear(&mut self) {
        self.0 = Duration::ZERO;
    }
}
#[derive(Resource, Default)]
struct Stats {
    frames: u32,
    physics: Duration,
    publish: Duration,
}
#[derive(Resource)]
struct Capture {
    target: Handle<Image>,
    requested: bool,
    started: Instant,
}
#[derive(Resource)]
struct Attachment {
    scene: Handle<bevy::world_serialization::WorldAsset>,
    started: Instant,
}
const LOAD_TIMEOUT: Duration = Duration::from_secs(120);
#[derive(Resource, Default)]
enum AttachmentProgress {
    #[default]
    Ready,
    Pending,
    Failed(String),
}
fn loading_progress(ready: bool, failure: Option<String>, elapsed: Duration) -> AttachmentProgress {
    if let Some(error) = failure {
        AttachmentProgress::Failed(error)
    } else if ready {
        AttachmentProgress::Ready
    } else if elapsed >= LOAD_TIMEOUT {
        AttachmentProgress::Failed("source visual loading timed out".into())
    } else {
        AttachmentProgress::Pending
    }
}
fn poll_attachment(
    assets: Res<AssetServer>,
    attachment: Option<Res<Attachment>>,
    mut progress: ResMut<AttachmentProgress>,
) {
    let Some(attachment) = attachment else {
        return;
    };
    let id = attachment.scene.id();
    let failure = match assets.load_state(id) {
        bevy::asset::LoadState::Failed(error) => Some(error.to_string()),
        _ => match assets.recursive_dependency_load_state(id) {
            bevy::asset::RecursiveDependencyLoadState::Failed(error) => Some(error.to_string()),
            _ => None,
        },
    };
    *progress = loading_progress(
        assets.is_loaded_with_dependencies(id),
        failure,
        attachment.started.elapsed(),
    );
}
#[derive(Component)]
struct Orbit {
    azimuth: f32,
    elevation: f32,
    radius: f32,
}
#[derive(Component)]
struct Status;
fn pose(p: sim_engine::Pose) -> PoseFlu {
    PoseFlu {
        translation_m: p.translation_m,
        rotation_wxyz: p.rotation_wxyz,
    }
}
fn main() -> AppExit {
    let mut options = Options::parse();
    if options.benchmark && options.frames.is_none() && options.screenshot.is_none() {
        options.frames = Some(120);
    }
    if options.frames == Some(0) {
        eprintln!("--frames must be positive");
        return AppExit::error();
    }
    if let Some(path) = &options.glb
        && !path.is_file()
    {
        eprintln!("GLB does not exist: {}", path.display());
        return AppExit::error();
    }
    let mut app = App::new();
    app.insert_resource(Plant(
        Simulation::new(options.paused).expect("prepare scene"),
    ))
    .insert_resource(options.clone())
    .init_resource::<Stats>()
    .init_resource::<PlaybackClock>()
    .init_resource::<AttachmentProgress>();
    if options.headless {
        app.add_plugins(MinimalPlugins)
            .init_resource::<Assets<Mesh>>()
            .init_resource::<Assets<StandardMaterial>>()
            .add_plugins(VisualPlugin)
            .add_systems(Startup, setup_visuals)
            .add_systems(Update, advance);
        app.finish();
        app.cleanup();
        let count = options.frames.unwrap_or(120);
        let began = Instant::now();
        for _ in 0..count {
            app.update();
        }
        let plant = app.world().resource::<Plant>();
        println!(
            "headless: {count} presentations, {} ticks, cube z={:.4} m; GPU unused",
            plant.0.tick,
            plant.0.pose(simulation::CUBE).translation_m[2]
        );
        report(&app, began.elapsed());
        return AppExit::Success;
    }
    let source_root = options.glb.as_ref().map(|p| {
        std::fs::canonicalize(p)
            .expect("GLB path")
            .parent()
            .unwrap()
            .to_string_lossy()
            .into_owned()
    });
    let mut plugins = DefaultPlugins
        .set(AssetPlugin {
            file_path: source_root.unwrap_or_else(|| "assets".into()),
            ..default()
        })
        .set(bevy::render::RenderPlugin {
            synchronous_pipeline_compilation: options.screenshot.is_some(),
            ..default()
        })
        .set(WindowPlugin {
            exit_condition: if options.screenshot.is_some() {
                bevy::window::ExitCondition::DontExit
            } else {
                bevy::window::ExitCondition::OnAllClosed
            },
            primary_window: options.screenshot.is_none().then(|| Window {
                title: "Scenario viewer".into(),
                resolution: (1100, 760).into(),
                ..default()
            }),
            ..default()
        });
    // Validation can exit before the first pipelined render is submitted;
    // avoid the early-exit RenderAppChannels teardown wait in those modes.
    // Ordinary interactive viewing of the built-in scene keeps pipelining.
    if options.frame_limit().is_some() || options.glb.is_some() {
        plugins = plugins.disable::<bevy::render::pipelined_rendering::PipelinedRenderingPlugin>();
    }
    if options.screenshot.is_some() {
        plugins = plugins.disable::<bevy::winit::WinitPlugin>();
    }
    app.add_plugins(plugins)
        .add_plugins(VisualPlugin)
        .insert_resource(bevy::winit::WinitSettings::continuous())
        .insert_resource(ClearColor(Color::srgb(0.035, 0.05, 0.075)))
        .add_systems(Startup, (setup_visuals, setup_view))
        .add_systems(
            Update,
            (
                controls,
                advance,
                orbit,
                update_status,
                poll_attachment,
                capture,
            )
                .chain(),
        );
    if options.screenshot.is_some() {
        app.add_plugins(bevy::app::ScheduleRunnerPlugin::run_loop(
            Duration::from_millis(8),
        ));
    }
    app.run()
}
fn setup_visuals(
    mut commands: Commands,
    mut visuals: ResMut<Visuals>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    plant: Res<Plant>,
) {
    visuals.reserve(5, 4);
    // All cuboids share one unit mesh. Each root keeps its scale through reset.
    let mesh = meshes.add(Cuboid::new(1., 1., 1.));
    let colors = [
        Color::srgb(0.98, 0.48, 0.16),
        Color::srgb(0.16, 0.24, 0.32),
        Color::srgb(0.12, 0.7, 0.76),
    ];
    let mut pedestal_material = Handle::default();
    for i in 0..3 {
        let material = materials.add(StandardMaterial {
            base_color: colors[i],
            perceptual_roughness: 0.65,
            ..default()
        });
        if i == 1 {
            pedestal_material = material.clone();
        }
        let [x, y, z] = HALF_EXTENTS_M[i];
        visuals.spawn_mesh(
            &mut commands,
            VisualId(KEYS[i].0),
            mesh.clone(),
            material,
            pose(plant.0.pose(KEYS[i])),
            Vec3::new(2. * y as f32, 2. * z as f32, 2. * x as f32),
        );
    }
    let floor = pedestal_material;
    visuals.spawn_mesh(
        &mut commands,
        VisualId(10),
        mesh,
        floor,
        PoseFlu {
            translation_m: [0., 0., -0.025],
            ..default()
        },
        Vec3::new(8., 0.05, 8.),
    );
}
fn setup_view(
    mut commands: Commands,
    options: Res<Options>,
    assets: Res<AssetServer>,
    mut visuals: ResMut<Visuals>,
    mut images: ResMut<Assets<Image>>,
    plant: Res<Plant>,
) {
    let camera = commands
        .spawn((
            Camera3d::default(),
            Transform::from_xyz(3.5, 2.8, 4.0).looking_at(Vec3::new(0., 0.55, 0.), Vec3::Y),
            Orbit {
                azimuth: -0.65,
                elevation: 0.5,
                radius: 5.8,
            },
        ))
        .id();
    if options.screenshot.is_some() {
        let target = images.add(Image::new_target_texture(
            1100,
            760,
            bevy::render::render_resource::TextureFormat::Rgba8UnormSrgb,
            None,
        ));
        commands
            .entity(camera)
            .insert(bevy::camera::RenderTarget::Image(target.clone().into()));
        commands.insert_resource(Capture {
            target,
            requested: false,
            started: Instant::now(),
        });
    }
    commands.spawn((
        DirectionalLight {
            illuminance: 12000.,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(3., 5., 4.).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: 250.,
        ..default()
    });
    // Source scenes keep their glTF local axes. The stable root tracks the arm.
    if let Some(path) = &options.glb {
        let path = std::fs::canonicalize(path).expect("GLB path");
        let scene: Handle<bevy::world_serialization::WorldAsset> = assets.load(
            bevy::gltf::GltfAssetLabel::Scene(0)
                .from_asset(path.file_name().unwrap().to_string_lossy().into_owned()),
        );
        commands.insert_resource(Attachment {
            scene: scene.clone(),
            started: Instant::now(),
        });
        commands.insert_resource(AttachmentProgress::Pending);
        visuals.spawn_scene(
            &mut commands,
            VisualId(20),
            scene,
            pose(plant.0.pose(simulation::ARM)),
            Vec3::ONE,
        );
    }
    commands.spawn((
        UiTargetCamera(camera),
        Text::new("Scenario viewer"),
        TextFont {
            font_size: FontSize::Px(19.),
            ..default()
        },
        TextColor(Color::srgb(0.9, 0.94, 0.98)),
        Node {
            position_type: PositionType::Absolute,
            left: px(22),
            top: px(20),
            ..default()
        },
        Status,
    ));
}
fn controls(
    keys: Res<ButtonInput<KeyCode>>,
    mut plant: ResMut<Plant>,
    mut exit: MessageWriter<AppExit>,
    mut clock: ResMut<PlaybackClock>,
) {
    if keys.just_pressed(KeyCode::Space) {
        plant.0.paused = !plant.0.paused;
        clock.clear();
    }
    if keys.just_pressed(KeyCode::KeyR) {
        plant.0.reset().expect("reset scene");
        clock.clear();
    }
    if keys.just_pressed(KeyCode::KeyN) && plant.0.paused {
        plant.0.step().expect("step scene");
        clock.clear();
    }
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}
fn advance(
    mut plant: ResMut<Plant>,
    mut visuals: ResMut<Visuals>,
    mut stats: ResMut<Stats>,
    options: Res<Options>,
    time: Res<Time>,
    mut clock: ResMut<PlaybackClock>,
) {
    let start = Instant::now();
    if options
        .frame_limit()
        .is_none_or(|limit| stats.frames < limit)
    {
        let ticks = if options.headless
            || options.frames.is_some()
            || options.screenshot.is_some()
            || options.benchmark
        {
            4
        } else if plant.0.paused {
            clock.clear();
            0
        } else {
            clock.ticks(
                time.delta(),
                Duration::from_nanos(plant.0.runtime.step_duration().as_ns()),
            )
        };
        plant.0.advance(ticks).expect("step scene");
    }
    stats.physics += start.elapsed();
    let start = Instant::now();
    for key in KEYS {
        visuals.set_pose(VisualId(key.0), pose(plant.0.pose(key)));
    }
    visuals.set_pose(VisualId(20), pose(plant.0.pose(simulation::ARM)));
    stats.publish += start.elapsed();
    stats.frames += 1;
}
fn orbit(
    buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    mut cameras: Query<(&mut Transform, &mut Orbit)>,
) {
    for (mut transform, mut orbit) in &mut cameras {
        if buttons.pressed(MouseButton::Left) {
            orbit.azimuth -= motion.delta.x * 0.006;
            orbit.elevation = (orbit.elevation + motion.delta.y * 0.006).clamp(0.1, 1.45);
        }
        orbit.radius = (orbit.radius - scroll.delta.y * 0.25).clamp(2., 14.);
        let (s, c) = orbit.azimuth.sin_cos();
        let (se, ce) = orbit.elevation.sin_cos();
        let next = Transform::from_translation(Vec3::new(
            orbit.radius * ce * s,
            0.55 + orbit.radius * se,
            orbit.radius * ce * c,
        ))
        .looking_at(Vec3::new(0., 0.55, 0.), Vec3::Y);
        if *transform != next {
            *transform = next;
        }
    }
}
fn update_status(plant: Res<Plant>, mut text: Single<&mut Text, With<Status>>) {
    let next = format!(
        "SCENARIO VIEWER\n{}  |  {:.2} s\nSpace  pause / resume    N  step    R  reset\nDrag  orbit    Scroll  zoom    Esc  close",
        if plant.0.paused { "Paused" } else { "Running" },
        plant.0.tick as f64 / 128.
    );
    if text.0 != next {
        text.0 = next;
    }
}
fn report(app: &App, elapsed: Duration) {
    let stats = app.world().resource::<Stats>();
    if app.world().resource::<Options>().benchmark {
        println!(
            "CPU totals: physics={:?}, pose publication={:?}, app updates={elapsed:?}; presentations={}. App updates include physics/publication. No GPU timing.",
            stats.physics, stats.publish, stats.frames
        );
    }
}
fn capture(
    mut commands: Commands,
    options: Res<Options>,
    stats: Res<Stats>,
    capture: Option<ResMut<Capture>>,
    progress: Res<AttachmentProgress>,
    mut exit: MessageWriter<AppExit>,
) {
    match &*progress {
        AttachmentProgress::Failed(error) => {
            error!("cannot load source visual: {error}");
            exit.write(AppExit::error());
            return;
        }
        AttachmentProgress::Pending => return,
        AttachmentProgress::Ready => {}
    }
    let limit = options.frame_limit().unwrap_or(90);
    let Some(mut capture) = capture else {
        if options.frames.is_some() && stats.frames >= limit {
            if options.benchmark {
                println!(
                    "CPU totals: physics={:?}, pose publication={:?}; presentations={}; no GPU timing",
                    stats.physics, stats.publish, stats.frames
                );
            }
            exit.write(AppExit::Success);
        }
        return;
    };
    if capture.started.elapsed() > Duration::from_secs(120) {
        error!("capture timed out");
        exit.write(AppExit::error());
        return;
    }
    if stats.frames < limit.max(30) || capture.requested {
        return;
    }
    if options.benchmark {
        println!(
            "CPU totals: physics={:?}, pose publication={:?}; presentations={}; no GPU timing",
            stats.physics, stats.publish, stats.frames
        );
    }
    capture.requested = true;
    let path = options.screenshot.clone().unwrap();
    let started = Instant::now();
    commands
        .spawn(Screenshot::image(capture.target.clone()))
        .observe(
            move |event: On<ScreenshotCaptured>, mut exit: MessageWriter<AppExit>| {
                let result = event
                    .image
                    .clone()
                    .try_into_dynamic()
                    .map_err(|e| e.to_string())
                    .and_then(|image| {
                        if image.to_rgb8().pixels().all(|p| p.0 == [0, 0, 0]) {
                            return Err("blank GPU readback".into());
                        }
                        image.save(&path).map_err(|e| e.to_string())
                    });
                match result {
                    Ok(()) => {
                        println!(
                            "saved {} (capture/readback {:?})",
                            path.display(),
                            started.elapsed()
                        );
                        exit.write(AppExit::Success);
                    }
                    Err(e) => {
                        error!("{e}");
                        exit.write(AppExit::error());
                    }
                }
            },
        );
}

#[cfg(test)]
mod tests {
    use super::*;
    fn app() -> App {
        let mut app = App::new();
        app.insert_resource(Options::parse_from([
            "viewer", "--paused", "--frames", "120",
        ]))
        .add_plugins(MinimalPlugins)
        .add_plugins(VisualPlugin)
        .init_resource::<Assets<Mesh>>()
        .init_resource::<Assets<StandardMaterial>>()
        .insert_resource(Plant(Simulation::new(true).unwrap()))
        .init_resource::<Stats>()
        .init_resource::<PlaybackClock>()
        .init_resource::<ButtonInput<KeyCode>>()
        .init_resource::<AttachmentProgress>()
        .add_message::<AppExit>()
        .add_systems(Startup, setup_visuals)
        .add_systems(Update, (controls, advance).chain());
        app
    }
    #[test]
    fn finite_run_waits_for_recursive_loading_without_advancing_more_physics() {
        let mut app = app();
        app.world_mut().resource_mut::<Options>().frames = Some(1);
        app.world_mut().resource_mut::<Plant>().0.paused = false;
        app.insert_resource(AttachmentProgress::Pending)
            .add_systems(Update, capture.after(advance));
        for _ in 0..5 {
            app.update();
        }
        assert_eq!(app.world().resource::<Plant>().0.tick, 4);
        assert!(app.world().resource::<Messages<AppExit>>().is_empty());
        app.insert_resource(AttachmentProgress::Ready);
        app.update();
        assert_eq!(app.world().resource::<Plant>().0.tick, 4);
        assert!(matches!(
            app.world_mut()
                .resource_mut::<Messages<AppExit>>()
                .drain()
                .next(),
            Some(AppExit::Success)
        ));
    }
    #[test]
    fn loading_failure_fails_finite_and_continuous_runs_before_capture_warmup() {
        for arguments in [
            vec!["viewer", "--frames", "1"],
            vec!["viewer"],
            vec![
                "viewer",
                "--screenshot",
                "/tmp/unused-viewer.png",
                "--frames",
                "1",
            ],
        ] {
            let mut app = app();
            app.insert_resource(Options::parse_from(arguments));
            app.insert_resource(loading_progress(
                false,
                Some("recursive dependency failed".into()),
                Duration::ZERO,
            ));
            app.add_systems(Update, capture.after(advance));
            app.update();
            assert!(matches!(
                app.world_mut()
                    .resource_mut::<Messages<AppExit>>()
                    .drain()
                    .next(),
                Some(AppExit::Error(_))
            ));
            assert_eq!(
                app.world_mut()
                    .query::<&Screenshot>()
                    .iter(app.world())
                    .count(),
                0
            );
        }
        assert!(matches!(
            loading_progress(false, None, LOAD_TIMEOUT),
            AttachmentProgress::Failed(_)
        ));
        assert!(matches!(
            loading_progress(false, None, LOAD_TIMEOUT - Duration::from_nanos(1)),
            AttachmentProgress::Pending
        ));
    }
    #[test]
    fn playback_clock_is_partition_independent_and_bounds_stalls() {
        let tick = Duration::from_nanos(7_812_500);
        let run = |frames: u64| {
            let mut clock = PlaybackClock::default();
            let mut ticks = 0;
            for i in 0..frames {
                let start = i * 1_000_000_000 / frames;
                let end = (i + 1) * 1_000_000_000 / frames;
                ticks += clock.ticks(Duration::from_nanos(end - start), tick);
            }
            assert_eq!(clock.0, Duration::ZERO);
            ticks
        };
        assert_eq!(run(60), 128);
        assert_eq!(run(144), 128);
        let mut clock = PlaybackClock::default();
        assert_eq!(clock.ticks(Duration::from_secs(10), tick), 32);
        assert_eq!(clock.0, Duration::ZERO);
        assert_eq!(clock.ticks(Duration::from_millis(4), tick), 0);
        clock.clear();
        assert_eq!(clock.ticks(Duration::from_millis(4), tick), 0);
    }
    #[test]
    fn controls_publish_real_poses_and_reset_preserves_visual_assets() {
        let mut app = app();
        app.update();
        let entity = app
            .world()
            .resource::<Visuals>()
            .entity(VisualId(1))
            .unwrap();
        let mesh = app.world().get::<Mesh3d>(entity).unwrap().0.clone();
        let material = app
            .world()
            .get::<MeshMaterial3d<StandardMaterial>>(entity)
            .unwrap()
            .0
            .clone();
        let before = app.world().get::<Transform>(entity).unwrap().translation;
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyN);
        app.update();
        assert_eq!(app.world().resource::<Plant>().0.tick, 1);
        assert!(app.world().get::<Transform>(entity).unwrap().translation.y < before.y);
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .clear();
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyR);
        app.update();
        assert_eq!(app.world().resource::<Plant>().0.tick, 0);
        assert_eq!(
            app.world().get::<Transform>(entity).unwrap().translation,
            before
        );
        assert_eq!(app.world().get::<Mesh3d>(entity).unwrap().0, mesh);
        assert_eq!(
            app.world()
                .get::<MeshMaterial3d<StandardMaterial>>(entity)
                .unwrap()
                .0,
            material
        );
        assert_eq!(app.world().resource::<Visuals>().len(), 4);
        assert_eq!(app.world().resource::<Assets<Mesh>>().len(), 1);
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .clear();
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::Space);
        app.update();
        assert_eq!(app.world().resource::<Plant>().0.tick, 4);
    }
}
