// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Run with `just gameplay-demo`. Native RMUC 2027; no CAD or host clock.
use rm_sim_gameplay::{
    Team,
    rmuc_2027::{
        ControlState, Point, Territory,
        match_rules::{COUNTDOWN_NS, GroundRole, Match, Phase},
    },
};

fn main() {
    let mut game = Match::default();
    game.add_robot(1, Team::Red, GroundRole::Infantry);
    game.add_robot(2, Team::Blue, GroundRole::Sentry);
    game.start(1);
    game.advance_ns(COUNTDOWN_NS);
    assert_eq!(game.phase, Phase::Running);

    // Referee HP overrides and RFID observations stand in for physical contacts.
    game.set_outpost_hp(Team::Blue, 0);
    for _ in 0..16 {
        game.observe(1, Team::Blue, Point::Outpost, true);
        game.advance_ns(1_000_000_000);
    }
    let territory = &game.control.snapshot().territories[Territory::BlueOutpost.index()];
    assert_eq!(territory.control, ControlState::Full(Team::Red));
    println!("Captured Blue Outpost territory: {:?}", territory.control);
    println!("Control scores: {:?}", game.control.snapshot().points);
    game.validate().expect("valid native authority");

    game.set_base_hp(Team::Blue, 0);
    assert_eq!(game.phase, Phase::Finished);
    println!("RMUC 2027 round result: {:?}", game.result);
}
