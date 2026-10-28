// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Browser entry point: fetch verified visuals, then start the passive renderer.
use bevy::{
    asset::{
        AssetMetaCheck,
        io::{AssetSourceBuilder, memory::MemoryAssetReader},
    },
    input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll},
    prelude::*,
};
use rm_sim_assets::cad_assets::visual::VisualPackage;
use rm_sim_render::{
    PoseFlu, RenderingConfig, apply_pose,
    cad::{CadInstance, CadRole, CadScenePlugin, CadSceneStatus},
    sync::SceneSyncPlugin,
};
use std::path::Path;
use wasm_bindgen::JsCast;
use wasm_bindgen_futures::JsFuture;

#[path = "browser_session.rs"]
mod session;

const MAX_FILE_BYTES: usize = 64 * 1024 * 1024;
const MAX_PACKAGE_BYTES: usize = 256 * 1024 * 1024;

pub(crate) fn start() {
    wasm_bindgen_futures::spawn_local(async {
        if let Err(error) = load().await {
            status(&format!("Cannot load field: {error:#}"), "error");
        }
    });
}
fn status(text: &str, state: &str) {
    if let Some(element) = web_sys::window()
        .and_then(|w| w.document())
        .and_then(|d| d.get_element_by_id("status"))
    {
        element.set_text_content(Some(text));
        let _ = element.set_attribute("data-state", state);
    }
}
async fn fetch(path: &str, limit: usize) -> anyhow::Result<Vec<u8>> {
    let window = web_sys::window().ok_or_else(|| anyhow::anyhow!("missing browser window"))?;
    let response: web_sys::Response = JsFuture::from(window.fetch_with_str(path))
        .await
        .map_err(|e| anyhow::anyhow!("fetch {path}: {e:?}"))?
        .dyn_into()
        .map_err(|_| anyhow::anyhow!("invalid HTTP response for {path}"))?;
    anyhow::ensure!(response.ok(), "{path}: HTTP {}", response.status());
    if let Some(length) = response
        .headers()
        .get("Content-Length")
        .ok()
        .flatten()
        .and_then(|v| v.parse::<usize>().ok())
    {
        anyhow::ensure!(length <= limit, "{path}: response exceeds byte limit");
    }
    let buffer = response
        .array_buffer()
        .map_err(|e| anyhow::anyhow!("{path}: {e:?}"))?;
    let buffer = JsFuture::from(buffer)
        .await
        .map_err(|e| anyhow::anyhow!("reading {path}: {e:?}"))?;
    let array = js_sys::Uint8Array::new(&buffer);
    anyhow::ensure!(
        array.length() as usize <= limit,
        "{path}: response exceeds byte limit"
    );
    Ok(array.to_vec())
}
async fn load() -> anyhow::Result<()> {
    let window = web_sys::window().ok_or_else(|| anyhow::anyhow!("missing browser window"))?;
    let query =
        web_sys::UrlSearchParams::new_with_str(&window.location().search().unwrap_or_default())
            .map_err(|_| anyhow::anyhow!("invalid URL query"))?;
    let server = query.get("server").filter(|s| !s.is_empty());
    let (root, radius) = match query.get("field").as_deref().unwrap_or("1v1") {
        "1v1" => ("field-rmul-2027-1v1", 9.),
        "3v3" => ("field-rmul-2027-3v3", 20.),
        "rmuc" => ("field-2027", 43.),
        _ => anyhow::bail!("unknown field; choose 1v1, 3v3 or rmuc"),
    };
    status("Loading field manifests…", "loading");
    let arena = fetch(&format!("{root}/manifest.json"), 1024 * 1024).await?;
    let equipment = fetch(&format!("{root}/equipment/manifest.json"), 1024 * 1024).await?;
    let package = VisualPackage::from_manifests(&arena, &equipment)?;
    let memory = MemoryAssetReader::default();
    let mut total = 0;
    let mut instances = Vec::new();
    for (index, asset) in package.assets.iter().enumerate() {
        status(
            &format!("Loading field {}/{}…", index + 1, package.assets.len()),
            "loading",
        );
        let bytes = fetch(
            &format!("{root}/{}", asset.file),
            MAX_FILE_BYTES.min(MAX_PACKAGE_BYTES - total),
        )
        .await?;
        total += bytes.len();
        asset.verify(&bytes)?;
        memory.root.insert_asset(Path::new(&asset.file), bytes);
        for pose in &asset.placements {
            let mut transform = Transform::default();
            apply_pose(
                &mut transform,
                PoseFlu {
                    translation_m: pose.translation_m,
                    rotation_wxyz: pose.rotation_wxyz,
                },
            );
            instances.push(CadInstance {
                file: format!("verified://{}", asset.file),
                transform,
                team: None,
                role: CadRole::Static { base_color: None },
            });
        }
    }
    status("Preparing field…", "loading");
    let mut app = App::new();
    app.register_asset_source(
        "verified",
        AssetSourceBuilder::new(move || Box::new(memory.clone())),
    );
    app.add_plugins(
        DefaultPlugins
            .set(WindowPlugin {
                primary_window: Some(Window {
                    title: "RoboMaster field".into(),
                    canvas: Some("#sim".into()),
                    fit_canvas_to_parent: true,
                    ..default()
                }),
                ..default()
            })
            .set(AssetPlugin {
                meta_check: AssetMetaCheck::Never,
                ..default()
            }),
    )
    .insert_resource(ClearColor(Color::srgb(0.13, 0.15, 0.19)))
    .insert_resource(Orbit {
        yaw: 0.7,
        pitch: 0.85,
        radius,
    })
    .add_plugins(SceneSyncPlugin)
    .add_plugins(CadScenePlugin {
        instances,
        rendering: RenderingConfig::default(),
    })
    .add_systems(Startup, setup)
    .add_systems(Update, (orbit, loading_status));
    if let Some(url) = server {
        session::install(&mut app, &url, &query)?;
    }
    app.run();
    Ok(())
}
#[derive(Resource)]
struct Orbit {
    yaw: f32,
    pitch: f32,
    radius: f32,
}
#[derive(Component)]
struct FieldCamera;
fn setup(mut commands: Commands, orbit: Res<Orbit>) {
    commands.insert_resource(GlobalAmbientLight {
        brightness: 200.,
        ..default()
    });
    commands.spawn((
        DirectionalLight {
            illuminance: 2000.,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_xyz(8., 15., 10.).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Camera3d::default(),
        bevy::camera::Exposure { ev100: 9. },
        Msaa::Off,
        camera_transform(&orbit),
        FieldCamera,
    ));
}
fn camera_transform(orbit: &Orbit) -> Transform {
    Transform::from_xyz(
        orbit.radius * orbit.pitch.cos() * orbit.yaw.sin(),
        orbit.radius * orbit.pitch.sin(),
        orbit.radius * orbit.pitch.cos() * orbit.yaw.cos(),
    )
    .looking_at(Vec3::ZERO, Vec3::Y)
}
fn orbit(
    mut state: ResMut<Orbit>,
    buttons: Res<ButtonInput<MouseButton>>,
    mouse: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    mut camera: Query<&mut Transform, With<FieldCamera>>,
) {
    let dragging = buttons.pressed(MouseButton::Left) && mouse.delta != Vec2::ZERO;
    if !dragging && scroll.delta.y == 0. {
        return;
    }
    if dragging {
        state.yaw -= mouse.delta.x * 0.005;
        state.pitch = (state.pitch + mouse.delta.y * 0.005).clamp(0.1, 1.5);
    }
    state.radius = (state.radius * (-scroll.delta.y * 0.1).exp()).clamp(2., 90.);
    if state.is_changed() {
        for mut transform in &mut camera {
            *transform = camera_transform(&state);
        }
    }
}
fn loading_status(
    scene: Res<CadSceneStatus>,
    mut ready: Local<bool>,
    session: Option<NonSend<session::BrowserSession>>,
) {
    if session.is_some() {
        return;
    }
    if *ready {
        return;
    }
    if let Some(error) = &scene.failed {
        status(&format!("Cannot display field: {error}"), "error");
        *ready = true;
    } else if scene.ready() {
        status("Drag to orbit · Scroll to zoom", "ready");
        *ready = true;
    }
}
