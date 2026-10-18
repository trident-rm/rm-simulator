// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Virtual assembly timing, progression, arbitration and restore regressions.
use rm_simulator_gameplay::{
    Team,
    rmuc_2027::{
        assembly::{ATTEMPT_NS, Outcome},
        match_rules::{COUNTDOWN_NS, GroundRole, Match, Phase},
    },
};
const S: u64 = 1_000_000_000;
fn running() -> Match {
    let mut g = Match::new(1);
    g.add_robot(1, Team::Red, GroundRole::Infantry);
    g.start(1);
    g.advance_ns(COUNTDOWN_NS);
    g
}
fn complete(g: &mut Match, team: Team, difficulty: u8) {
    g.start_assembly(team, difficulty).unwrap();
    g.finish_assembly(team).unwrap();
    g.validate().unwrap();
}
#[test]
fn unlocks_previous_completion_and_duplicate_actions_are_transactional() {
    let mut g = Match::new(1);
    g.add_robot(1, Team::Red, GroundRole::Infantry);
    assert!(g.start_assembly(Team::Red, 1).is_err());
    g.start(1);
    assert!(g.start_assembly(Team::Red, 1).is_err());
    g.advance_ns(COUNTDOWN_NS);
    for difficulty in [0, 2, 3, 4, 255] {
        let before = g.clone();
        assert!(g.start_assembly(Team::Red, difficulty).is_err());
        assert_eq!(g, before);
    }
    complete(&mut g, Team::Red, 1);
    let before = g.clone();
    assert!(g.finish_assembly(Team::Red).is_err());
    assert!(g.start_assembly(Team::Red, 2).is_err());
    assert_eq!(g, before);
    g.advance_ns(60 * S - 1);
    assert!(g.start_assembly(Team::Red, 2).is_err());
    g.advance_ns(1);
    complete(&mut g, Team::Red, 2);
    assert_eq!(g.level_caps, [7, 5]);
    g.advance_ns(60 * S);
    complete(&mut g, Team::Red, 3);
    assert_eq!(g.level_caps, [10, 5]);
    assert_eq!(g.field_effects(1).defense_pct, 25, "{g:?}");
    g.advance_ns(60 * S);
    complete(&mut g, Team::Red, 4);
    assert_eq!(g.field_effects(1).defense_pct, 50);
    assert!(g.start_assembly(Team::Red, 4).is_err());
}
#[test]
fn timeout_boundary_and_finish_before_it_apply_once_on_both_sides() {
    for team in Team::BOTH {
        for delay in [ATTEMPT_NS - 1, ATTEMPT_NS, ATTEMPT_NS + 1] {
            let mut g = running();
            g.start_assembly(team, 1).unwrap();
            g.advance_ns(delay);
            assert_eq!(g.finish_assembly(team).is_ok(), delay < ATTEMPT_NS);
            let a = &g.assembly[team.index()];
            assert_eq!(
                a.outcome,
                if delay < ATTEMPT_NS {
                    Outcome::Completed
                } else {
                    Outcome::TimedOut
                }
            );
            assert_eq!(a.completions[0], u32::from(delay < ATTEMPT_NS));
            assert_eq!(a.income_per_10s, if delay < ATTEMPT_NS { 40 } else { 0 });
            g.validate().unwrap();
        }
    }
}
#[test]
fn timeout_releases_cores_and_retry_keeps_completed_progress() {
    let mut g = running();
    complete(&mut g, Team::Red, 1);
    g.start_assembly(Team::Red, 1).unwrap();
    assert!(g.start_assembly(Team::Red, 1).is_err());
    g.advance_ns(ATTEMPT_NS);
    assert_eq!(g.assembly[0].completions, [1, 0, 0, 0]);
    complete(&mut g, Team::Red, 1);
    assert_eq!(g.assembly[0].income_per_10s, 44);
    g.start_assembly(Team::Red, 1).unwrap();
    g.cancel_assembly(Team::Red).unwrap();
    assert_eq!(g.assembly[0].outcome, Outcome::Cancelled);
    assert!(g.cancel_assembly(Team::Red).is_err());
    assert_eq!(g.assembly[0].income_per_10s, 44);
}
#[test]
fn lower_difficulties_overlap_but_level_four_reserves_both_cores() {
    let mut g = running();
    for difficulty in 1..=3 {
        if difficulty > 1 {
            g.advance_ns(60 * S);
        }
        for team in Team::BOTH {
            complete(&mut g, team, difficulty);
        }
    }
    g.advance_ns(60 * S);
    g.start_assembly(Team::Blue, 1).unwrap();
    assert!(g.start_assembly(Team::Red, 4).is_err());
    g.start_assembly(Team::Red, 2).unwrap();
    g.cancel_assembly(Team::Red).unwrap();
    g.cancel_assembly(Team::Blue).unwrap();
    g.start_assembly(Team::Red, 4).unwrap();
    for difficulty in 1..=4 {
        assert!(g.start_assembly(Team::Blue, difficulty).is_err());
    }
    g.advance_ns(ATTEMPT_NS);
    complete(&mut g, Team::Blue, 4);
    complete(&mut g, Team::Red, 4);
}
#[test]
fn recurring_income_matches_manual_example_and_round_aligned_settlements() {
    let mut g = running();
    complete(&mut g, Team::Red, 1);
    let gold = g.gold;
    g.advance_ns(10 * S - 1);
    assert_eq!(g.gold, gold);
    g.advance_ns(1);
    assert_eq!(g.gold, [gold[0] + 40, gold[1]]);
    g.advance_ns(50 * S);
    complete(&mut g, Team::Red, 2);
    g.advance_ns(60 * S);
    complete(&mut g, Team::Red, 3);
    complete(&mut g, Team::Red, 2);
    complete(&mut g, Team::Red, 1);
    assert_eq!(g.assembly[0].income_per_10s, 92);
    let gold = g.gold;
    g.advance_ns(10 * S);
    assert_eq!(g.gold, [gold[0] + 92, gold[1]]);
}
#[test]
fn checkpoint_restore_partitioning_reset_and_round_finish() {
    let mut g = running();
    g.start_assembly(Team::Red, 1).unwrap();
    g.advance_ns(7 * S + 12);
    let bytes = serde_json::to_vec(&g).unwrap();
    let mut restored: Match = serde_json::from_slice(&bytes).unwrap();
    restored.validate().unwrap();
    g.advance_ns(ATTEMPT_NS);
    for dt in [1, 999_999_999, 13 * S, 31 * S] {
        restored.advance_ns(dt);
    }
    assert_eq!(g, restored);
    assert_eq!(g.assembly[0].outcome, Outcome::TimedOut);
    g.start_assembly(Team::Red, 1).unwrap();
    g.finish();
    assert_eq!(g.phase, Phase::Finished);
    assert_eq!(g.assembly[0].outcome, Outcome::Cancelled);
    assert!(g.finish_assembly(Team::Red).is_err());
    g.validate().unwrap();
    g.start(1);
    assert_eq!(g.assembly[0].outcome, Outcome::Idle);
    assert_eq!(g.assembly[0].completions, [0; 4]);
}
#[test]
fn malformed_checkpoints_reject_income_and_shared_core_conflicts() {
    let mut g = running();
    complete(&mut g, Team::Red, 1);
    g.assembly[0].income_per_10s += 1;
    assert!(g.validate().is_err());
    g.assembly[0].income_per_10s -= 1;
    g.assembly[0].completions[3] = 1;
    assert!(g.validate().is_err());
    let mut g = running();
    for difficulty in 1..=3 {
        if difficulty > 1 {
            g.advance_ns(60 * S);
        }
        for team in Team::BOTH {
            complete(&mut g, team, difficulty);
        }
    }
    g.advance_ns(60 * S);
    g.start_assembly(Team::Red, 4).unwrap();
    g.assembly[1] = g.assembly[0].clone();
    assert!(g.validate().is_err());
}

#[test]
fn permanent_defense_applies_to_robot_and_owned_structures_and_survives_respawn() {
    use rm_simulator_gameplay::{Caliber, Target};
    let mut g = running();
    for difficulty in 1..=4 {
        if difficulty > 1 {
            g.advance_ns(60 * S);
        }
        complete(&mut g, Team::Red, difficulty);
        let damage = match difficulty {
            3 => 15,
            4 => 10,
            _ => 20,
        };
        assert_eq!(
            g.hit(Target::Outpost(Team::Red), None, Caliber::Mm17),
            damage
        );
        assert_eq!(g.hit(Target::Outpost(Team::Blue), None, Caliber::Mm17), 20);
    }
    assert_eq!(g.damage_robot(1, 20, false), 10);
    g.set_robot_hp(1, 0).unwrap();
    g.set_robot_hp(1, u32::MAX).unwrap();
    g.robots[0].combat.weakened = true;
    assert_eq!(g.field_effects(1).defense_pct, 50);
    assert_eq!(g.damage_robot(1, 20, false), 10);
    g.validate().unwrap();
}
