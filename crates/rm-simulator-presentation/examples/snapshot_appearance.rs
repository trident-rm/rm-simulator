// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Bounded CPU-only snapshot adaptation probe; excludes simulation, ECS and drawing.
//! Run with `mbx run -p rm-simulator-presentation --example snapshot_appearance`.
//! `fresh` reproduces the former allocation pattern; `retained` uses the app path.
use rm_simulator_render::sync::SceneState;
use std::{hint::black_box, time::Instant};
#[path = "../tests/support/mod.rs"]
mod support;

fn main() {
    let iterations = 256;
    for (name, native, flashing) in [
        ("quiet", false, false),
        ("flash", false, true),
        ("native", true, true),
    ] {
        let snapshot = support::fixture(native, flashing);
        let fixture_hash = support::hash(serde_json::to_vec(&snapshot).unwrap());
        let mut expected = None;
        for retained in [false, true] {
            let mut scene = SceneState::default();
            let mut targets = Vec::new();
            for _ in 0..16 {
                support::adapt(&snapshot, retained, &mut scene, &mut targets);
            }
            let start = Instant::now();
            for _ in 0..iterations {
                support::adapt(black_box(&snapshot), retained, &mut scene, &mut targets);
                black_box((&scene, &targets));
            }
            let elapsed = start.elapsed();
            let hash = support::hash(format!("{scene:?}{targets:?}"));
            if let Some(expected) = &expected {
                assert_eq!(&hash, expected);
            } else {
                expected = Some(hash.clone());
            }
            println!(
                "{name} mode={} iterations={iterations} ns_per_frame={} fixture_sha256={fixture_hash} output_sha256={hash}",
                if retained { "retained" } else { "fresh" },
                elapsed.as_nanos() / iterations
            );
        }
    }
}
