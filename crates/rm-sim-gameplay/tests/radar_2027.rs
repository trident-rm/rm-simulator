// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
use rm_sim_gameplay::{
    Team,
    rmuc_2027::{
        match_rules::{COUNTDOWN_NS, GroundRole, Match},
        radar::{Information, Quality},
    },
};
fn game() -> Match {
    let mut g = Match::default();
    g.add_robot(1, Team::Red, GroundRole::Infantry);
    g.add_robot(2, Team::Blue, GroundRole::Infantry);
    g.add_robot(3, Team::Blue, GroundRole::Infantry);
    g.start(1);
    g.advance_ns(COUNTDOWN_NS);
    g
}
#[test]
fn rulebook_history_example_and_friendly_threshold_and_respawn() {
    let mut g = game();
    for _ in 0..15 {
        g.advance_ns(200_000_000);
        g.radar_report(Team::Red, 2, [0, 0], Quality::Accurate)
            .unwrap();
    }
    assert_eq!(g.radar.marks[0][&2].progress_tenths, 1200);
    assert_eq!(g.field_effects(2).vulnerability_pct, 20);
    for _ in 0..5 {
        g.advance_ns(200_000_000);
        g.radar_report(Team::Red, 2, [0, 0], Quality::Wrong)
            .unwrap();
    }
    assert_eq!(g.radar.marks[0][&2].progress_tenths, 1080);
    g.advance_ns(1_000_000_000);
    assert_eq!(g.radar.marks[0][&2].progress_tenths, 976);
    g.radar_report(Team::Red, 2, [0, 0], Quality::Accurate)
        .unwrap();
    g.radar_report(Team::Red, 2, [0, 0], Quality::SemiAccurate)
        .unwrap();
    assert_eq!(g.radar.marks[0][&2].progress_tenths, 1001);
    for _ in 0..10 {
        g.radar_report(Team::Red, 1, [0, 0], Quality::Accurate)
            .unwrap();
    }
    assert!(g.radar.marks[0][&1].localized(true));
    assert!(!g.radar.marks[0][&1].localized(false));
    let p = g.radar.marks[0][&2].progress_tenths;
    g.set_robot_hp(2, 0).unwrap();
    g.gold[1] = 10000;
    g.instant_respawn(2).unwrap();
    assert_eq!(g.radar.marks[0][&2].progress_tenths, p);
    assert_eq!(g.radar.marks[0][&2].increment_tenths, 0);
    g.validate().unwrap();
}
#[test]
fn union_timer_interference_double_and_partition_restore() {
    let mut g = game();
    for id in [2, 3] {
        for _ in 0..17 {
            g.radar_report(Team::Red, id, [0, 0], Quality::Accurate)
                .unwrap();
        }
    }
    for _ in 0..300 {
        g.advance_ns(200_000_000);
        for id in [2, 3] {
            g.radar_report(Team::Red, id, [0, 0], Quality::Accurate)
                .unwrap();
        }
    }
    assert_eq!(g.radar.vulnerable_ns[0], 60_000_000_000);
    assert_eq!(g.radar.opportunities(Team::Red), 1);
    g.radar_double(Team::Red).unwrap();
    assert_eq!(g.field_effects(2).vulnerability_pct, 40);
    assert!(g.radar_double(Team::Red).is_err());
    g.radar.interference[0] = 1;
    assert_eq!(g.field_effects(2).vulnerability_pct, 0);
    let mut split = g.clone();
    g.advance_ns(1_110_000_000);
    for _ in 0..111 {
        split.advance_ns(10_000_000);
    }
    assert_eq!(g, split);
    g.validate().unwrap();
    assert_eq!(g.radar.vulnerable_ns[0], 60_000_000_000);
}
#[test]
fn information_expires_without_disclosing_unobserved_fields_and_reset_clears_it() {
    let mut g = game();
    g.advance_ns(7_812_500);
    g.radar_information(
        Team::Red,
        2,
        Information {
            hp: Some(123),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(g.radar.information[0][&2].allowance, None);
    let mut split = g.clone();
    g.advance_ns(1_000_000_000);
    for _ in 0..128 {
        split.advance_ns(7_812_500);
    }
    assert_eq!(g, split);
    assert!(g.radar.information[0].is_empty());
    g.start(1);
    assert_eq!(g.radar.marks[0][&2].progress_tenths, 0);
}
