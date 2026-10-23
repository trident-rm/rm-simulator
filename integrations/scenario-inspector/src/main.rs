// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Optional standalone component inspector; use --help for native/probe commands.
#[cfg(feature = "native")]
mod native;
use anyhow::{Context, bail};
use model::Inspector;
use raster::{Options, Plane};
use rm_scenario_inspector::{model, raster};
use std::path::PathBuf;

fn main() -> anyhow::Result<()> {
    let mut pick: Option<[u32; 2]> = None;
    let mut bench = 0usize;
    let mut spawn = None;
    let mut robot_enabled = true;
    let mut script: Option<PathBuf> = None;
    let mut rfid: Option<PathBuf> = None;
    let mut root = rm_sim_assets::cad_assets::default_cad_assets();
    let mut asset = "outpost".to_string();
    let mut placement = 0;
    let mut probe: Option<PathBuf> = None;
    let mut ticks = 0;
    let mut coordinates = vec![];
    let mut options = Options::default();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || args.next().context(format!("{arg} needs a value"));
        match arg.as_str() {
            "--view" => {
                options.orbit = match value()?.as_str() {
                    "3d" => true,
                    "wire" => false,
                    _ => bail!("view must be wire or 3d"),
                }
            }
            "--orbit-yaw" => options.yaw_rad = value()?.parse()?,
            "--orbit-elevation" => options.elevation_rad = value()?.parse()?,
            "--zoom" => options.zoom = value()?.parse()?,
            "--collision-overlay" => options.collision_overlay = true,
            "--viewport-bench" => bench = value()?.parse()?,
            "--pick" => {
                let v = value()?
                    .split(',')
                    .map(str::parse::<u32>)
                    .collect::<Result<Vec<_>, _>>()?;
                anyhow::ensure!(
                    v.len() == 2 && v[0] < raster::WIDTH && v[1] < raster::HEIGHT,
                    "pick needs X,Y inside 960x640"
                );
                pick = Some([v[0], v[1]]);
            }
            "--no-robot" => robot_enabled = false,
            "--robot" => {
                let v = value()?
                    .split(',')
                    .map(str::parse::<f64>)
                    .collect::<Result<Vec<_>, _>>()?;
                anyhow::ensure!(
                    v.len() == 4 && v.iter().all(|v| v.is_finite()),
                    "robot needs finite X,Y,Z,YAW_RAD"
                );
                spawn = Some(v);
            }
            "--script" => script = Some(value()?.into()),
            "--rfid" => rfid = Some(value()?.into()),
            "--cad-assets" => root = value()?.into(),
            "--asset" => asset = value()?,
            "--placement" => placement = value()?.parse()?,
            "--probe" => probe = Some(value()?.into()),
            "--ticks" => ticks = value()?.parse()?,
            "--coordinate" => {
                let s = value()?;
                let (i, v) = s.split_once('=').context("expected CHANNEL=VALUE")?;
                coordinates.push((i.parse::<usize>()?, v.parse::<f64>()?));
            }
            "--plane" => {
                options.plane = match value()?.as_str() {
                    "xy" => Plane::Top,
                    "xz" => Plane::Front,
                    "yz" => Plane::Side,
                    _ => bail!("plane must be xy, xz or yz"),
                }
            }
            "--field" => options.field = true,
            "--help" | "-h" => {
                println!(
                    "rm-scenario-inspector [--cad-assets PATH] [--asset outpost|rune|base|tech-core|floor|arena|scenery-N]\n  [--placement N] [--coordinate CHANNEL=VALUE] [--ticks N] [--plane xy|xz|yz] [--field]\n  [--view wire|3d] [--orbit-yaw RAD] [--orbit-elevation RAD] [--zoom FACTOR]\n  [--collision-overlay] [--pick X,Y] [--viewport-bench FRAMES]\n  [--rfid ZONES_JSON] [--probe OUTPUT_DIRECTORY]\n  [--robot X,Y,Z,YAW_RAD | --no-robot] [--script ACTIONS_JSON]\n\nWithout --probe, opens RXUI (build with --features native). Probe writes view.png and report.json.\nCoordinates are rad for rotation, 0..1 fractions for translation. Preview law: +0.5 rad/s or +0.1 fraction/s.\nExplicit 128 Hz time; Outpost/Base/Rune spawn a physical omni robot. Scripts drive/aim/fire/step/reset. Privileged truth, no referee or networking."
                );
                return Ok(());
            }
            _ => bail!("unknown argument {arg}"),
        }
    }
    anyhow::ensure!(
        options.yaw_rad.is_finite()
            && options.elevation_rad.is_finite()
            && options.zoom.is_finite()
            && options.zoom > 0.
            && options.zoom <= 20.,
        "camera needs finite angles and zoom in (0,20]"
    );
    anyhow::ensure!(bench <= 1000, "viewport benchmark capped at 1000 frames");
    anyhow::ensure!(
        options.orbit || (pick.is_none() && bench == 0),
        "picking/viewport benchmark require --view 3d"
    );
    anyhow::ensure!(
        probe.is_some() || (pick.is_none() && bench == 0),
        "picking/viewport benchmark need --probe OUTPUT_DIRECTORY"
    );
    let mut inspector = Inspector::load(&root, &asset, placement)?;
    if let Some(path) = rfid {
        inspector.load_zones(&path)?;
    }
    for (i, v) in coordinates {
        inspector.set_coordinate(i, v)?;
    }
    if !robot_enabled && inspector.robot_spawn().is_some() {
        inspector.configure_robot(None)?;
    } else if let Some(v) = spawn {
        let config = rm_sim_physics::chassis::ChassisConfig::default();
        inspector.configure_robot(Some(rm_sim_assets::scenarios::RobotSpawn {
            team: rm_sim_physics::Team::Red,
            config,
            pose: rm_sim_physics::Pose {
                translation_m: [v[0], v[1], v[2]],
                rotation_wxyz: [(v[3] / 2.).cos(), 0., 0., (v[3] / 2.).sin()],
            },
        }))?;
    }
    if let Some(path) = script {
        inspector.run_script(&serde_json::from_slice(&std::fs::read(path)?)?)?;
    }
    inspector.step(ticks)?;
    if let Some(path) = probe {
        std::fs::create_dir_all(&path)?;
        let mut report = inspector.report();
        let pixels = if options.orbit {
            let frame = rm_scenario_inspector::viewport::render(&inspector, &options)?;
            report["viewport"] = serde_json::json!({"backend":"CPU depth-rasterized orthographic diagnostic 3D", "width":raster::WIDTH,"height":raster::HEIGHT,"triangles":frame.triangles,"render_ms":frame.elapsed_ms,"yaw_rad":options.yaw_rad,"elevation_rad":options.elevation_rad,"zoom":options.zoom,"collision_overlay":options.collision_overlay,"materials":"source base color/emission factors; flat lighting; textures/PBR/alpha unsupported; opaque two-sided surfaces; x-ray diagnostics", "textured_primitives":inspector.visuals()?.iter().filter(|v|v.part.textured).count()});
            if let Some([x, y]) = pick {
                report["pick"] = frame
                    .pick(&inspector, x, y)
                    .unwrap_or(serde_json::Value::Null);
            }
            if bench > 0 {
                let mut times = Vec::with_capacity(bench);
                for _ in 0..bench {
                    times.push(
                        rm_scenario_inspector::viewport::render(&inspector, &options)?.elapsed_ms,
                    );
                }
                times.sort_by(f64::total_cmp);
                report["viewport_benchmark"] = serde_json::json!({"frames":bench,"min_ms":times[0],"median_ms":times[bench/2],"max_ms":times[bench-1],"scope":"warm parsed visuals; transforms, fit, allocation, depth raster and enabled overlays; excludes physics, parsing, PNG, RXUI upload and presentation", "backend":"CPU; no GPU timing"});
            }
            frame.pixels
        } else {
            raster::render(&inspector, &options)
        };
        raster::write_png(&path.join("view.png"), &pixels)?;
        std::fs::write(
            path.join("report.json"),
            serde_json::to_vec_pretty(&report)?,
        )?;
        println!("{}", serde_json::to_string_pretty(&report)?);
        return Ok(());
    }
    #[cfg(feature = "native")]
    {
        native::run(inspector, options)?;
        Ok(())
    }
    #[cfg(not(feature = "native"))]
    {
        bail!("native host disabled: use --features native, or --probe OUTPUT_DIRECTORY")
    }
}
