// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! CPU-only FLU conversion fixture; independent of GPU rendering and asset loading.
use bevy::prelude::*;
use rm_sim_render::{PoseFlu, apply_pose};
use std::{hint::black_box, time::Instant};

fn main() {
    const VISUALS: usize = 1024;
    const FRAMES: usize = 4096;
    let mut transforms = vec![Transform::default(); VISUALS];
    for trial in 0..5 {
        let start = Instant::now();
        for frame in 0..FRAMES {
            for (index, transform) in transforms.iter_mut().enumerate() {
                apply_pose(
                    transform,
                    black_box(PoseFlu {
                        translation_m: [index as f64 * 0.01, frame as f64 * 0.001, 0.5],
                        rotation_wxyz: [1.0, 0.0, 0.0, 0.0],
                    }),
                );
            }
            black_box(&transforms);
        }
        println!(
            "trial {trial}: {} poses, {:?}",
            VISUALS * FRAMES,
            start.elapsed()
        );
    }
}
