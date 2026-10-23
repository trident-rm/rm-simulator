// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Static CAD visibility and ideal instantaneous nearest-hit LiDAR costs.
//! Usage: navigation_query_cost CAD_PATH. No rendering, sensor noise or dynamic robots.
use rm_sim_physics::sensors::{Lidar, LidarConfig, LidarFrame};
use rm_sim_server::{cad_assets, layout};
use rm_sim_world::{Field, Pose};
use std::{hint::black_box, path::PathBuf, time::Instant};

fn main() -> anyhow::Result<()> {
    let root = PathBuf::from(std::env::args().nth(1).expect("CAD_PATH required"));
    let cad = cad_assets::load(&root)?;
    let terrain = layout::load_terrain(&cad)?;
    let mut field = Field::new(&layout::field_config(
        &cad,
        &layout::LayoutOptions {
            rune: None,
            outpost_speed_rad_s: 0.4,
            terrain: true,
            referee: false,
            projectile_policy: Default::default(),
        },
    ))?;
    layout::add_terrain(&mut field, &terrain)?;
    let geometry = field.static_geometry_snapshot();
    println!("{}; CAD root={}", terrain.describe(), root.display());
    println!(
        "Boolean visibility only: cached per-shape AABBs and mesh BVHs, no nearest distances, labels, scan motion or dynamic bodies."
    );
    for (name, horizontal, layers) in [("planar", 1080, 1), ("sixteen layers", 1800, 16)] {
        let origin = [0., 0., 0.35];
        let endpoints: Vec<[f64; 3]> = (0..layers)
            .flat_map(|layer| {
                let elevation = if layers == 1 {
                    0.
                } else {
                    (-15. + 30. * f64::from(layer) / f64::from(layers - 1)).to_radians()
                };
                (0..horizontal).map(move |ray| {
                    let yaw = f64::from(ray) / f64::from(horizontal) * std::f64::consts::TAU;
                    [
                        origin[0] + 30. * elevation.cos() * yaw.cos(),
                        origin[1] + 30. * elevation.cos() * yaw.sin(),
                        origin[2] + 30. * elevation.sin(),
                    ]
                })
            })
            .collect();
        let scan = || {
            endpoints
                .iter()
                .filter(|&&end| !geometry.segment_clear(origin, black_box(end)))
                .count()
        };
        black_box(scan()); // Populate immutable geometry bounds before timing.
        let mut times = Vec::new();
        let mut blocked = 0;
        for _ in 0..7 {
            let start = Instant::now();
            blocked = black_box(scan());
            times.push(start.elapsed().as_secs_f64());
        }
        times.sort_by(f64::total_cmp);
        println!(
            "{name}: rays={}; median={:.3} ms; max={:.3} ms; {:.0} queries/s; blocked={blocked}",
            endpoints.len(),
            times[3] * 1000.,
            times[6] * 1000.,
            endpoints.len() as f64 / times[3]
        );
        let mut config = LidarConfig::planar(horizontal, 0.05, 30.);
        if layers > 1 {
            config.elevations_rad = (0..layers)
                .map(|i| (-15. + 30. * f64::from(i) / f64::from(layers - 1)).to_radians())
                .collect();
        }
        let lidar = Lidar::new(config).map_err(anyhow::Error::msg)?;
        for origin in [[0., 0., 0.35], [4.7, 3.5, 0.35], [-6.7, -3.5, 0.35]] {
            anyhow::ensure!(
                geometry.sphere_clear(origin, 0.01),
                "scan origin overlaps geometry: {origin:?}"
            );
            let mut frame = LidarFrame::default();
            let mount = Pose::at(origin);
            lidar
                .sample_into(&geometry, mount, 0, &mut frame)
                .map_err(anyhow::Error::msg)?;
            let mut times = Vec::new();
            for _ in 0..7 {
                let start = Instant::now();
                lidar
                    .sample_into(&geometry, mount, 0, &mut frame)
                    .map_err(anyhow::Error::msg)?;
                black_box(&frame);
                times.push(start.elapsed().as_secs_f64());
            }
            times.sort_by(f64::total_cmp);
            let hits = frame.returns.iter().flatten().count();
            println!(
                "{name} nearest-hit at {origin:?}: rays={}; median={:.3} ms; max={:.3} ms; returns={hits}",
                frame.returns.len(),
                times[3] * 1000.,
                times[6] * 1000.
            );
        }
    }
    Ok(())
}
