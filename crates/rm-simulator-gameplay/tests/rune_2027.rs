// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native 2027 Rune schedules, ring tables, pair timing, benefits and exact restore.
use rm_simulator_gameplay::{
    Caliber, Target, Team,
    rmuc_2027::{
        match_rules::{COUNTDOWN_NS, GroundRole, Match},
        rune::{Hit, Phase, Rune, Stage},
    },
};
const S: u64 = 1_000_000_000;
fn running() -> Match {
    let mut g = Match::new(1);
    g.add_robot(1, Team::Red, GroundRole::Infantry);
    g.add_robot(2, Team::Blue, GroundRole::Infantry);
    g.start(1);
    g.advance_ns(COUNTDOWN_NS);
    g
}
fn target(g: &Match, team: Team) -> u8 {
    g.runes[team.index()]
        .targets
        .iter()
        .position(|&b| b)
        .unwrap() as u8
}
fn small(g: &mut Match, team: Team) {
    g.activate_rune(team).unwrap();
    for _ in 0..5 {
        let blade = target(g, team);
        g.hit_rune(team, blade, 10);
    }
}
fn big(g: &mut Match, team: Team, ring: u8, arms: usize) {
    g.activate_rune(team).unwrap();
    for group in 0..5 {
        let blade = target(g, team);
        g.hit_rune(team, blade, ring);
        if group < arms - 5 {
            let blade = target(g, team);
            g.hit_rune(team, blade, ring);
        } else {
            g.advance_ns(S + 1);
        }
    }
}
#[test]
fn schedule_stage_boundary_busy_attempt_timeout_and_transactional_rejection() {
    let mut g = running();
    assert_eq!(g.runes[0].opportunities, [1, 0]);
    g.advance_ns(90 * S);
    assert_eq!(g.runes[0].opportunities, [2, 0]);
    g.activate_rune(Team::Red).unwrap();
    let before = g.clone();
    assert!(g.activate_rune(Team::Red).is_err());
    assert_eq!(g, before);
    g.advance_ns(20 * S);
    assert_eq!(g.runes[0].phase, Phase::Inactive);
    g.advance_ns(70 * S);
    assert_eq!(g.runes[0].stage, Stage::Big);
    assert_eq!(g.runes[0].opportunities, [1, 1]);
    g.advance_ns(150 * S);
    assert_eq!(g.runes[0].opportunities, [1, 3]);
}
#[test]
fn exact_first_bonus_deadlines_wrong_target_and_failed_marker() {
    let mut g = running();
    g.advance_ns(180 * S);
    g.activate_rune(Team::Red).unwrap();
    g.advance_ns(5 * S / 2);
    let blade = target(&g, Team::Red);
    assert_eq!(g.hit_rune(Team::Red, blade, 10), Hit::Accepted);
    g.advance_ns(S);
    assert!(g.runes[0].bonus);
    let blade = target(&g, Team::Red);
    assert_eq!(g.hit_rune(Team::Red, blade, 10), Hit::Accepted);
    assert_eq!(g.runes[0].groups, 1);
    let bad = g.runes[0].targets.iter().position(|b| !b).unwrap() as u8;
    assert_eq!(g.hit_rune(Team::Red, bad, 10), Hit::Reset);
    assert_eq!(g.runes[0].phase, Phase::Failed);
    assert!(g.runes[0].rings.is_empty());
    g.advance_ns(1);
    assert_eq!(g.runes[0].phase, Phase::Activating);
    g.advance_ns(5 * S / 2 + 1);
    assert_eq!(g.runes[0].phase, Phase::Failed);
}
#[test]
fn all_big_tables_arms_and_both_teams() {
    for team in Team::BOTH {
        for arms in 5..=10 {
            for (ring, attack, defense, cooling) in [
                (1, 150, 25, 1),
                (3, 150, 25, 1),
                (4, 150, 25, 2),
                (7, 150, 25, 2),
                (8, 200, 25, 2),
                (9, 200, 25, 3),
                (10, 300, 50, 5),
            ] {
                let mut g = running();
                g.advance_ns(180 * S);
                big(&mut g, team, ring, arms);
                let r = &g.runes[team.index()];
                assert_eq!(r.phase, Phase::Activated);
                let b = r.buff.as_ref().unwrap();
                assert_eq!(
                    (b.attack_pct, b.defense_pct, b.cooling),
                    (attack, defense, cooling)
                );
                assert_eq!(
                    b.until_ns - g.elapsed_ns(),
                    [30, 35, 40, 45, 50, 60][arms - 5] * S
                );
                assert_eq!(r.rings.len(), arms);
                assert_eq!(r.big_successes, 1);
                g.validate().unwrap();
            }
        }
    }
}
#[test]
fn small_shared_xp_cap_survives_death_and_applies_to_structures_after_transfer() {
    let mut g = running();
    g.add_robot(3, Team::Red, GroundRole::Infantry);
    small(&mut g, Team::Red);
    for id in [1, 3] {
        g.grant_experience(id, 6000);
    }
    assert_eq!(g.runes[0].buff.as_ref().unwrap().extra_xp_tenths, 12000);
    g.grant_experience(1, 100);
    assert_eq!(g.robots[0].combat.experience_tenths, 12100);
    g.set_robot_hp(1, 0).unwrap();
    assert!(g.runes[0].buff.is_some());
    assert_eq!(g.field_effects(3).defense_pct, 25);
    g.set_outpost_hp(Team::Blue, 0);
    g.advance_ns(5 * S);
    assert_eq!(g.outpost_hp[1], 500);
    assert_eq!(
        g.hit(Target::Outpost(Team::Blue), Some(2), Caliber::Mm17),
        15
    );
    g.set_outpost_hp(Team::Red, 0);
    assert_eq!(g.hit(Target::Base(Team::Red), Some(2), Caliber::Mm17), 15);
}
#[test]
fn small_buff_crosses_big_stage_and_motion_does_not_jump() {
    let mut g = running();
    g.advance_ns(179 * S);
    small(&mut g, Team::Red);
    let before = g.runes[0].angle_at(180 * S);
    g.advance_ns(S);
    assert!((g.runes[0].angle_at(180 * S) - before).abs() < 1e-9);
    assert_eq!(g.runes[0].buff.as_ref().unwrap().source, Stage::Small);
    assert_eq!(g.runes[0].stage, Stage::Big);
    g.advance_ns(44 * S);
    assert_eq!(g.runes[0].phase, Phase::Inactive);
    g.activate_rune(Team::Red).unwrap();
}
#[test]
fn ring_boundaries_shrink_only_after_big_success_and_ignore_disabled_strikes() {
    for (mm, ring) in [
        (0, 10),
        (15, 10),
        (16, 9),
        (30, 9),
        (135, 2),
        (136, 1),
        (150, 1),
    ] {
        assert_eq!(Rune::ring(f64::from(mm) / 1000.), Some(ring));
    }
    assert_eq!(Rune::ring(f64::NAN), None);
    assert_eq!(Rune::ring(0.151), None);
    let mut g = running();
    g.advance_ns(180 * S);
    big(&mut g, Team::Red, 10, 10);
    assert_eq!(g.runes[0].minimum_ring(), 4);
    g.advance_ns(75 * S);
    g.activate_rune(Team::Red).unwrap();
    let blade = target(&g, Team::Red);
    let before = g.clone();
    assert_eq!(g.hit_rune(Team::Red, blade, 3), Hit::Ignored);
    assert_eq!(g, before);
}
#[test]
fn attempt_deadline_and_bonus_restore_are_partition_independent() {
    let mut a = running();
    a.advance_ns(180 * S);
    a.activate_rune(Team::Red).unwrap();
    let blade = target(&a, Team::Red);
    a.hit_rune(Team::Red, blade, 9);
    let mut b: Match = serde_json::from_slice(&serde_json::to_vec(&a).unwrap()).unwrap();
    a.advance_ns(20 * S);
    for _ in 0..2560 {
        b.advance_ns(S / 128);
    }
    assert_eq!(a, b);
    a.validate().unwrap();
    assert_eq!(a.runes[0].phase, Phase::Inactive);
}
#[test]
fn rune_benefits_survive_weakness_and_disconnection_but_not_expiry() {
    let mut g = running();
    small(&mut g, Team::Red);
    g.robots[0].combat.weakened = true;
    g.robots[0].online = false;
    g.robots[0].combat.power.modules.main_online = false;
    assert_eq!(g.field_effects(1).defense_pct, 25);
    g.advance_ns(45 * S);
    assert_eq!(g.field_effects(1).defense_pct, 0);
}
#[test]
fn fractional_mean_rings_and_malformed_restore_are_checked() {
    let mut g = running();
    g.advance_ns(180 * S);
    g.activate_rune(Team::Red).unwrap();
    // Mean 7.2 belongs to (7,8], not the integer-truncated (3,7] tier.
    for ring in [7, 7, 7, 7, 8] {
        let blade = target(&g, Team::Red);
        g.hit_rune(Team::Red, blade, ring);
        g.advance_ns(S + 1);
    }
    assert_eq!(g.runes[0].buff.as_ref().unwrap().attack_pct, 200);
    g.validate().unwrap();
    let original = g.clone();
    g.runes[0].buff.as_mut().unwrap().defense_pct = u32::MAX;
    assert!(g.validate().is_err());
    g = original;
    g.runes[0].opportunities = [u32::MAX; 2];
    assert!(g.validate().is_err());
}
