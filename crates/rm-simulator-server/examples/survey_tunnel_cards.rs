// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Print source CAD surface stacks along the two Highland tunnel lanes.
fn main() -> anyhow::Result<()> {
    let cad = rm_simulator_server::cad_assets::load(std::path::Path::new("field-2027"))?;
    let terrain = rm_simulator_server::layout::load_terrain(&cad)?;
    for y in [5.93, 6.93] {
        for i in -20..=20 {
            let x = f64::from(i) * 0.1;
            let mut levels = vec![];
            let mut top = 1.;
            while let Some(z) = terrain.ground.ground_height_below(x, y, top) {
                levels.push(z);
                top = z - 0.003;
                if levels.len() >= 10 {
                    break;
                }
            }
            println!("{x:.2},{y:.2},{levels:?}");
        }
    }
    Ok(())
}
