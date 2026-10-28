// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native command-line startup and application wiring.
use bevy::prelude::*;
use clap::Parser;
use rm_sim_render::{
    RenderingConfig,
    cad::CadScenePlugin,
    chassis::ChassisVisualsPlugin,
    projectile::ProjectileVisualsPlugin,
    sync::{SceneSyncPlugin, SceneSyncSet},
};

use crate::args::Args;
use crate::controls::{
    Drive, Gun, drive_camera, drive_chassis, fire_gun, fly_camera, mouse_capture,
};
use crate::debug::collision_view;
use crate::hud::update_hud;
use crate::scene::{Appearance, publish_scene};
use crate::screenshot::{ScreenshotRequest, take_screenshot};
use crate::session::advance_world;

pub(crate) fn run() -> AppExit {
    #[cfg(feature = "steam")]
    let steam_runtime = crate::steam_support::SteamRuntime::initialize();
    let mut args = Args::parse();
    if let Err(error) = args.resolve_asset_path() {
        eprintln!("resolving assets: {error}");
        return AppExit::error();
    }
    let mut app = App::new();
    #[cfg(feature = "steam")]
    crate::steam_support::install(&mut app, steam_runtime);
    if let Some(path) = &args.hero_assets {
        use bevy::asset::io::{AssetSourceBuilder, file::FileAssetReader};
        let root = path.clone();
        app.register_asset_source(
            "hero-robot",
            AssetSourceBuilder::new(move || Box::new(FileAssetReader::new(root.clone()))),
        );
    }
    if let Some(path) = &args.robot_assets {
        use bevy::asset::io::{AssetSourceBuilder, file::FileAssetReader};
        let root = std::path::absolute(path).expect("robot asset path");
        app.register_asset_source(
            "robots",
            AssetSourceBuilder::new(move || Box::new(FileAssetReader::new(root.clone()))),
        );
        app.insert_resource(rm_sim_render::team_robot::TeamRobotAssets {
            file: "robots://neutral.glb".into(),
            hero_file: args
                .hero_assets
                .as_ref()
                .map(|_| "hero-robot://neutral.glb".into()),
            folding_hero: args.host.folding_hero,
            stair_heavy: args.host.stair_heavy,
            articulated: args.host.source_chassis.is_some(),
            required_groups: args
                .host
                .source_chassis
                .as_ref()
                .and_then(|c| c.articulation.as_ref())
                .map(|a| {
                    a.joints
                        .iter()
                        .map(|j| j.name.clone())
                        .chain((0..4).map(|i| format!("wheel-{i}")))
                        .collect()
                })
                .unwrap_or_default(),
            replace_armor: matches!(
                args.robot_descriptor
                    .as_ref()
                    .expect("resolved robot package")
                    .armor(),
                rm_sim_assets::robots::ArmorIdentity::Replacement { .. }
            ),
        });
    }
    if let Err(error) = crate::assembly::register(&mut app, args.host.assembly_assets.as_deref()) {
        eprintln!("assembly assets: {error}");
        return AppExit::error();
    }
    let mut plugins = DefaultPlugins
        .set(WindowPlugin {
            primary_window: Some(Window {
                title: "rm-simulator".into(),
                focused: args.window_mode == crate::args::WindowMode::Normal,
                ..default()
            }),
            ..default()
        })
        .set(AssetPlugin {
            file_path: args.host.cad_assets_path().to_string_lossy().into_owned(),
            ..default()
        });
    if args.window_mode == crate::args::WindowMode::Headless {
        plugins = plugins.disable::<bevy::winit::WinitPlugin>();
    }
    // The referee link's broker logs every connection and subscription.
    #[cfg(feature = "referee-link")]
    {
        plugins = plugins.set(bevy::log::LogPlugin {
            filter: format!("{},rumqttd=warn", bevy::log::DEFAULT_FILTER),
            ..default()
        });
    }
    app.insert_resource(args.window_mode);
    app.add_plugins(plugins)
        .add_plugins(rm_sim_render::team_robot::TeamRobotVisualsPlugin)
        .add_plugins(crate::presentation::PresentationPlugin {
            headless_fps: args.headless_fps,
        })
        .add_plugins(crate::screenshot::ScreenshotReadinessPlugin)
        .add_plugins(crate::video_capture::VideoCapturePlugin)
        .add_plugins(CadScenePlugin {
            instances: Vec::new(),
            rendering: RenderingConfig::field(),
        })
        // Venue-like haze instead of a black void beyond the field edge.
        .insert_resource(ClearColor(Color::srgb(0.13, 0.15, 0.19)))
        .add_plugins(SceneSyncPlugin)
        .insert_resource(Appearance(RenderingConfig::field()))
        .add_plugins(ProjectileVisualsPlugin {
            rendering: RenderingConfig::field(),
        })
        .add_plugins(ChassisVisualsPlugin {
            rendering: RenderingConfig::field(),
            armor: crate::scene::armor_optics(),
        })
        .insert_resource(Gun::new(args.shot(), args.fire_interval_ns()))
        .insert_resource(crate::hud::HudState {
            debug: args.debug_panel,
            network_stats: args.network_stats,
            ..default()
        })
        .init_resource::<crate::auto_aim::AutoAim>()
        .add_plugins(crate::preferences::PreferencesPlugin {
            graphics_preset: args
                .graphics_preset
                .map(crate::args::GraphicsPreset::quality),
            no_bloom: args.no_bloom,
        })
        .add_plugins(crate::graphics::GraphicsPlugin)
        .add_plugins(crate::hud::HudPlugin)
        .add_plugins(crate::assembly::AssemblyPlugin)
        .add_plugins(crate::supply::SupplyPlugin)
        .add_plugins(crate::sentry::SentryPlugin)
        .add_plugins(crate::enemy_intel::EnemyIntelPlugin)
        .add_plugins(crate::debug_panel::DebugPanelPlugin)
        .add_plugins(crate::zone_debug::ZoneDebugPlugin)
        .insert_resource(crate::debug_panel::DebugOptions {
            stats: args.render_stats,
            zones: args.zones,
            rfid_zones: args.rfid_zones,
            zone_labels: args.zone_labels,
        })
        .insert_resource(bevy::pbr::wireframe::WireframeConfig {
            global: args.wireframe,
            ..default()
        })
        .add_systems(
            Update,
            (crate::hud::panel_input, crate::hud::menu_input)
                .chain()
                .run_if(not(in_state(crate::loading::Screen::Staging)))
                .before(fire_gun)
                .before(drive_chassis)
                .before(fly_camera)
                .before(mouse_capture)
                .before(update_hud)
                .before(crate::hud::update_map)
                .before(crate::hud::update_panel_rows),
        )
        .add_plugins(crate::loading::LoadingPlugin)
        .add_plugins(crate::title::TitlePlugin)
        .add_plugins(crate::staging::StagingPlugin)
        .add_systems(
            Update,
            crate::hud::sync_menus
                .after(mouse_capture)
                .after(crate::hud::menu_input),
        )
        .add_systems(Startup, crate::scene::setup_camera)
        // Poll, sample mouse/assist, submit controls and shots, then exchange
        // prediction and place the physical camera. A capturing click cannot fire.
        .add_systems(
            Update,
            (
                advance_world,
                crate::controls::sample_drive_aim.run_if(resource_exists::<Drive>),
                fly_camera.run_if(not(resource_exists::<Drive>)),
                crate::auto_aim::update,
                drive_chassis.run_if(resource_exists::<Drive>),
                crate::session::advance_shots,
                fire_gun,
                crate::session::predict_frame,
                drive_camera.run_if(resource_exists::<Drive>),
                publish_scene,
            )
                .chain()
                .run_if(in_state(crate::loading::Screen::InMatch))
                .before(SceneSyncSet)
                .before(mouse_capture)
                .before(update_hud)
                .before(crate::hud::update_map),
        )
        .add_systems(
            Update,
            (
                mouse_capture,
                collision_view,
                crate::debug::dynamic_colliders
                    .after(collision_view)
                    .after(publish_scene)
                    .before(SceneSyncSet),
                update_hud,
                crate::hud::click_map.before(mouse_capture),
                crate::hud::sync_map_interaction.after(crate::hud::click_map),
                crate::hud::update_map,
                crate::hud::navigation_paths,
                crate::network_hud::update,
                crate::hud::update_panel_rows,
            )
                .run_if(in_state(crate::loading::Screen::InMatch)),
        )
        .add_systems(
            Update,
            take_screenshot
                .after(collision_view)
                .after(SceneSyncSet)
                .run_if(resource_exists::<ScreenshotRequest>)
                .run_if(
                    in_state(crate::loading::Screen::InMatch)
                        .or_else(in_state(crate::loading::Screen::Staging))
                        .or_else(in_state(crate::loading::Screen::Title)),
                ),
        );
    if let Some(path) = &args.screenshot {
        app.insert_resource(ScreenshotRequest::new(path.clone()));
    }
    if let Some(address) = args.console {
        match crate::console::Console::bind(address) {
            Ok(console) => {
                app.insert_resource(console)
                    .add_plugins(crate::console::ConsolePlugin);
            }
            Err(error) => {
                eprintln!("cannot start console: {error}");
                return AppExit::error();
            }
        }
    }
    #[cfg(feature = "referee-link")]
    if let Some(address) = args.referee_link {
        match crate::referee_link::start(address) {
            Ok(link) => {
                app.insert_resource(link)
                    .add_plugins(crate::referee_link::RefereeLinkPlugin);
            }
            Err(error) => {
                eprintln!("cannot start referee link on {address}: {error}");
                return AppExit::error();
            }
        }
    }
    #[cfg(feature = "referee-link")]
    if let Some(address) = args.referee_video {
        match crate::referee_video::start(address) {
            Ok(video) => {
                app.insert_resource(video)
                    .add_plugins(crate::referee_video::RefereeVideoPlugin);
            }
            Err(error) => {
                eprintln!("cannot start referee video to {address}: {error:#}");
                return AppExit::error();
            }
        }
    }
    // A command line that already names a match starts on the splash.
    if args.auto_join() {
        app.insert_resource(crate::loading::JoinRequest(args.clone()))
            .insert_state(crate::loading::Screen::Loading);
    } else {
        app.insert_state(crate::loading::Screen::Title);
    }
    app.insert_resource(crate::title::BaseArgs(args));
    app.run()
}
