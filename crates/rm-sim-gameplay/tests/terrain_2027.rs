// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native UC §5.6.4.5 and Table 5-27 sequence and lifetime boundaries.
use rm_sim_gameplay::{
    InfantryChassis, InfantryLauncher, Team,
    rmuc_2027::{
        match_rules::{COUNTDOWN_NS, GroundRole, Match},
        terrain::{Benefit, Card, Course, Terrain},
    },
};
const S: u64 = 1_000_000_000;
fn card(side: Team, course: Course, pad: u8) -> Card {
    Card {
        side,
        course: Some(course),
        index: 0,
        pad,
    }
}
#[test]
fn directed_courses_accept_inclusive_deadline_and_reject_late_or_reverse() {
    for side in Team::BOTH {
        for (course, window, benefit) in [
            (Course::Road, 3 * S, Benefit::Road),
            (Course::Highland, 5 * S, Benefit::Highland),
        ] {
            let low = card(side, course, 0);
            let high = card(side, course, 1);
            for delta in [window - 1, window, window + 1] {
                let mut t = Terrain::default();
                t.observe(0, &[low]);
                assert_eq!(
                    t.observe(delta, &[high]),
                    if delta <= window { Some(benefit) } else { None }
                );
            }
            let mut t = Terrain::default();
            t.observe(0, &[high]);
            assert_eq!(t.observe(S, &[low]), None);
            assert!(t.progress.is_some());
        }
    }
}
#[test]
fn tunnel_both_directions_and_all_three_courses_keep_crossing_types_distinct() {
    for side in Team::BOTH {
        for index in 0..3 {
            for start in [0, 2] {
                for interior in [false, true] {
                    let mut t = Terrain::default();
                    let c = |pad| Card {
                        index,
                        ..card(side, Course::Tunnel, pad)
                    };
                    t.observe(0, &[c(start)]);
                    if interior {
                        t.observe(S, &[c(1)]);
                    }
                    assert_eq!(
                        t.observe(3 * S, &[c(2 - start)]),
                        Some(if interior {
                            Benefit::ThroughTunnel
                        } else {
                            Benefit::AcrossTunnel
                        })
                    );
                    assert_eq!(t.defense_pct, if interior { 0 } else { 25 });
                    assert_eq!(t.until_ns[3], if interior { 123 * S } else { 0 });
                }
            }
        }
    }
}
#[test]
fn held_cards_intervening_cards_wrong_side_course_and_simultaneous_cards() {
    let low = card(Team::Red, Course::Road, 0);
    let high = card(Team::Red, Course::Road, 1);
    let mut t = Terrain::default();
    t.observe(0, &[low]);
    t.observe(2 * S, &[low]);
    assert_eq!(t.progress.as_ref().unwrap().started_ns, 0);
    assert_eq!(t.observe(4 * S, &[high]), None);
    for blocker in [
        Card {
            course: None,
            index: 4,
            pad: 0,
            side: Team::Red,
        },
        card(Team::Blue, Course::Road, 1),
        card(Team::Red, Course::Highland, 1),
    ] {
        let mut t = Terrain::default();
        t.observe(0, &[low]);
        t.observe(S, &[blocker]);
        assert_eq!(t.observe(2 * S, &[high]), None);
    }
    let mut t = Terrain::default();
    t.observe(0, &[low, high]);
    assert!(t.progress.is_none());
    assert_eq!(t.observe(S, &[high]), None);
}
fn road(t: &mut Terrain, now: u64) -> Option<Benefit> {
    t.observe(now, &[]);
    t.observe(now, &[card(Team::Red, Course::Road, 0)]);
    t.observe(now + S, &[card(Team::Red, Course::Road, 1)])
}
#[test]
fn road_cooldown_and_death_and_mixed_benefit_lifetimes() {
    let mut t = Terrain::default();
    assert_eq!(road(&mut t, 0), Some(Benefit::Road));
    assert_eq!(road(&mut t, 2 * S), None);
    t.clear_on_death();
    assert_eq!(t.road_ready_ns, 16 * S);
    assert_eq!(road(&mut t, 15 * S), Some(Benefit::Road));
    t.observe(17 * S, &[card(Team::Red, Course::Tunnel, 0)]);
    t.observe(18 * S, &[card(Team::Red, Course::Tunnel, 1)]);
    assert_eq!(
        t.observe(19 * S, &[card(Team::Red, Course::Tunnel, 2)]),
        Some(Benefit::ThroughTunnel)
    );
    assert_eq!((t.defense_pct, t.defense_until_ns), (50, 139 * S));
    t.expire(139 * S);
    assert_eq!(t.defense_pct, 0);
    assert_eq!(t.until_ns, [0; 4]);
}
fn running() -> Match {
    let mut g = Match::new(1);
    g.add_robot(1, Team::Red, GroundRole::Infantry);
    g.start(1);
    g.advance_ns(COUNTDOWN_NS);
    g
}
#[test]
fn first_kind_xp_survives_death_and_cooling_preserves_half_units_across_restore() {
    let mut g = running();
    g.set_performance(
        1,
        InfantryChassis::HpFocused,
        InfantryLauncher::BurstFocused,
    )
    .unwrap();
    for pad in [0, 1, 2] {
        g.observe_terrain(1, &[card(Team::Red, Course::Tunnel, pad)]);
    }
    assert_eq!(g.robots[0].combat.experience_tenths, 3000);
    assert_eq!(g.field_effects(1).cooling_half_per_s, 15); // Burst base 5/s -> 7.5/s.
    g.robots[0].combat.heat_tenths = 100;
    let mut restored: Match = serde_json::from_slice(&serde_json::to_vec(&g).unwrap()).unwrap();
    g.advance_ns(S);
    for _ in 0..128 {
        restored.advance_ns(S / 128);
    }
    assert_eq!(g, restored);
    assert_eq!(g.robots[0].combat.heat_tenths, 25);
    g.set_robot_hp(1, 0).unwrap();
    assert_eq!(g.robots[0].terrain.until_ns, [0; 4]);
    assert!(g.robots[0].terrain.experience_awarded[2]);
    g.set_robot_hp(1, 200).unwrap();
    for pad in [0, 2] {
        g.observe_terrain(1, &[card(Team::Red, Course::Tunnel, pad)]);
    }
    assert_eq!(g.robots[0].combat.experience_tenths, 3000);
    g.validate().unwrap();
}
#[test]
fn cooling_then_defense_retains_the_longer_original_cooling_duration() {
    let mut t = Terrain::default();
    for (now, pad) in [(0, 0), (S, 1), (2 * S, 2)] {
        t.observe(now, &[card(Team::Red, Course::Tunnel, pad)]);
    }
    assert_eq!(t.defense_pct, 0);
    let original_end = t.until_ns[3];
    t.observe(3 * S, &[card(Team::Red, Course::Road, 0)]);
    t.observe(4 * S, &[card(Team::Red, Course::Road, 1)]);
    assert_eq!(t.defense_pct, 50);
    assert_eq!(t.defense_until_ns, original_end);
    assert!(t.validate(4 * S));
}
