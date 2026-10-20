// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Coarse native 2027 stepping benchmark; measurements are not rule inputs.
use rm_simulator_gameplay::{
    Team,
    rmuc_2027::match_rules::{COUNTDOWN_NS, GroundRole, Match},
};
use std::{hint::black_box, time::Instant};
fn main() {
    let start = Instant::now();
    for _ in 0..1_000 {
        let mut game = Match::default();
        game.add_robot(1, Team::Red, GroundRole::Infantry);
        game.start(1);
        game.advance_ns(COUNTDOWN_NS + 420_000_000_000);
        black_box(game);
    }
    println!("1,000 native rounds: {:?}", start.elapsed());
}
