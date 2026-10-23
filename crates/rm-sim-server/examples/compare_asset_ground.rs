// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Compare loaded ground surfaces, including lower tunnel floors and upper decks.
//!
//! Usage: `compare_asset_ground BEFORE AFTER`. This deterministic grid check
//! complements the producer's sampled surface deviations; it is not a maximum
//! geometric error proof. Moving bodies and fixed equipment are checked by the
//! producer separately, rather than included in terrain-height queries.
use anyhow::{Context, ensure};
use rm_sim_server::{cad_assets, layout};

fn main() -> anyhow::Result<()> {
    let paths: Vec<_> = std::env::args_os().skip(1).collect();
    ensure!(paths.len() == 2, "usage: compare_asset_ground BEFORE AFTER");
    let before = cad_assets::load(std::path::Path::new(&paths[0]))?;
    let after = cad_assets::load(std::path::Path::new(&paths[1]))?;
    ensure!(
        before.floor_top_cad_m == after.floor_top_cad_m,
        "floor reference changed"
    );
    let before = layout::load_terrain(&before)?;
    let after = layout::load_terrain(&after)?;
    ensure!(
        before.mechanisms.len() == after.mechanisms.len(),
        "mechanism coverage changed"
    );
    // Simulator derivative policy: 7 mm sampled collision limit plus 3 mm for
    // probing numerical seams. This is not a competition rulebook tolerance.
    let tolerance_m = 0.010;
    let mut checked = 0;
    let mut failures = 0;
    let mut maximum_m: f64 = 0.;
    let mut examples = Vec::new();
    for (direction, source, target) in [
        ("before-to-after", &before.ground, &after.ground),
        ("after-to-before", &after.ground, &before.ground),
    ] {
        for ix in 0..140 {
            for iy in 0..80 {
                // Offset the grid from regular CAD edges instead of probing
                // exclusively at joints and shared triangle vertices.
                let x = -14. + (f64::from(ix) + 0.371) * 0.2;
                let y = -8. + (f64::from(iy) + 0.613) * 0.2;
                for ceiling_m in [0.1, 0.4, 0.8, 1.5, 3.] {
                    let Some(z) = source.ground_height_below(x, y, ceiling_m) else {
                        continue;
                    };
                    checked += 1;
                    let candidate = target.ground_height_below(x, y, z + tolerance_m);
                    let delta = candidate.map(|h| (h - z).abs());
                    if let Some(delta) = delta {
                        maximum_m = maximum_m.max(delta);
                    }
                    if delta.is_none_or(|d| d > tolerance_m) {
                        failures += 1;
                        if examples.len() < 20 {
                            examples.push(serde_json::json!({"direction": direction,
                                "position_m": [x, y, z], "candidate_m": candidate}));
                        }
                    }
                }
            }
        }
    }
    println!(
        "{}",
        serde_json::to_string_pretty(&serde_json::json!({
            "samples": checked, "failures": failures, "maximum_height_delta_m": maximum_m,
            "tolerance_m": tolerance_m, "examples": examples,
            "before": before.describe(), "after": after.describe()
        }))?
    );
    ensure!(checked > 0, "no comparable terrain surfaces");
    ensure!(failures == 0, "ground surface comparison failed");
    std::io::Write::flush(&mut std::io::stdout()).context("flush comparison report")?;
    Ok(())
}
