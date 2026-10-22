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
    let mut rfid: Option<PathBuf> = None;
    let mut root = rm_simulator_assets::cad_assets::default_cad_assets();
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
                    "rm-scenario-inspector [--cad-assets PATH] [--asset outpost|rune|base|tech-core|floor|arena|scenery-N]\n  [--placement N] [--coordinate CHANNEL=VALUE] [--ticks N] [--plane xy|xz|yz] [--field]\n  [--rfid ZONES_JSON] [--probe OUTPUT_DIRECTORY]\n\nWithout --probe, opens RXUI (build with --features native). Probe writes view.png and report.json.\nCoordinates are rad for rotation, 0..1 fractions for translation. Preview law: +0.5 rad/s or +0.1 fraction/s.\nExplicit 128 Hz time; source presets step an empty physical session. No referee, robots or networking."
                );
                return Ok(());
            }
            _ => bail!("unknown argument {arg}"),
        }
    }
    let mut inspector = Inspector::load(&root, &asset, placement)?;
    if let Some(path) = rfid {
        inspector.load_zones(&path)?;
    }
    for (i, v) in coordinates {
        inspector.set_coordinate(i, v)?;
    }
    inspector.step(ticks)?;
    if let Some(path) = probe {
        std::fs::create_dir_all(&path)?;
        raster::write_png(
            &path.join("view.png"),
            &raster::render(&inspector, &options),
        )?;
        std::fs::write(
            path.join("report.json"),
            serde_json::to_vec_pretty(&inspector.report())?,
        )?;
        println!("{}", serde_json::to_string_pretty(&inspector.report())?);
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
