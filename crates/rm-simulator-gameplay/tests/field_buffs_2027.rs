// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Continuous UC 2027 field benefits and their authoritative persistence.
use rm_simulator_gameplay::{
    Team,
    rmuc_2027::{
        CAPTURE_NS, Point,
        match_rules::{
            COUNTDOWN_NS, GroundRole, Match, RFID_GRACE_NS, THREE_MIN_NS, buffs::Activation,
        },
    },
};

fn running(team: Team) -> Match {
    let mut g = Match::new(1);
    g.add_robot(1, team, GroundRole::Infantry);
    g.add_robot(2, team.other(), GroundRole::Infantry);
    g.start(1);
    g.advance_ns(COUNTDOWN_NS);
    g
}
fn activate(g: &mut Match, id: u32, side: Team, point: Point) {
    g.observe(id, side, point, true);
    g.advance_ns(0);
}

#[test]
fn base_vantage_and_highland_defense_use_strongest_source_and_exact_expiry() {
    for team in Team::BOTH {
        let mut g = running(team);
        activate(&mut g, 1, team, Point::CentralHighland);
        assert_eq!(g.field_effects(1).defense_pct, 25);
        assert_eq!(g.damage_robot(1, 20, false), 15);
        activate(&mut g, 1, team, Point::Base);
        assert_eq!(g.field_effects(1).defense_pct, 50);
        assert_eq!(g.damage_robot(1, 20, false), 10);
        assert_eq!(g.collision(1), 1);
        assert_eq!(g.damage_robot(1, 20, true), 20);
        g.advance_ns(RFID_GRACE_NS - 1);
        assert_eq!(g.field_effects(1).defense_pct, 50);
        g.advance_ns(1);
        assert_eq!(g.field_effects(1).defense_pct, 0);
        g.observe_service(1, false, true);
        assert_eq!(g.field_effects(1).defense_pct, 50);
        g.robots[0].combat.weakened = true;
        assert_eq!(g.field_effects(1).defense_pct, 0);
        g.robots[0].combat.weakened = false;
        g.robots[0].online = false;
        g.robots[0].combat.power.modules.main_online = false;
        assert_eq!(g.field_effects(1).defense_pct, 0);
    }
}

#[test]
fn highland_defense_follows_exclusive_latch_not_raw_contacts() {
    let mut g = running(Team::Red);
    activate(&mut g, 2, Team::Blue, Point::CentralHighland);
    activate(&mut g, 1, Team::Blue, Point::CentralHighland);
    assert_eq!(g.field_effects(1).defense_pct, 0);
    assert_eq!(g.field_effects(2).defense_pct, 25);
    assert_eq!(
        g.control_diagnostics()[0].points[5].status,
        Activation::HighlandHeld
    );
    g.robots[1].online = false;
    g.robots[1].combat.power.modules.main_online = false;
    g.advance_ns(0);
    assert_eq!(g.field_effects(1).defense_pct, 25);
}

#[test]
fn fortress_uses_historical_deficit_and_shared_reserve_before_personal_allowance() {
    for team in Team::BOTH {
        let mut g = running(team);
        g.add_robot(3, team, GroundRole::Sentry);
        g.set_outpost_hp(team, 0);
        g.advance_ns(CAPTURE_NS);
        g.set_base_hp(team, 4000);
        g.set_base_hp(team, 10000);
        activate(&mut g, 1, team, Point::Fortress);
        let effects = g.field_effects(1);
        assert_eq!(effects.defense_pct, 50);
        assert_eq!(effects.cooling_per_s, 12 + 75);
        assert_eq!(effects.fortress_reserve, 500);
        g.check_launch(1).unwrap();
        g.record_launch(1, 25.);
        assert_eq!(g.robots[0].combat.allowance, 0);
        assert_eq!(g.fortress_reserve_used[team.index()], 1);
        g.advance_ns(100_000_000);
        assert_eq!(g.robots[0].combat.heat_tenths, 100 - 87);
        activate(&mut g, 3, team, Point::Fortress);
        assert_eq!(g.field_effects(3).fortress_reserve, 0);
        g.robots[0].online = false;
        g.robots[0].combat.power.modules.main_online = false;
        g.advance_ns(0);
        assert_eq!(g.field_effects(3).fortress_reserve, 499);
        // The same reserve is shared across holders, not refilled by handover.
        g.fortress_reserve_used[team.index()] = 500;
        g.robots[2].combat.allowance = 0;
        assert!(g.check_launch(3).is_err());
        g.robots[2].combat.allowance = 3;
        g.check_launch(3).unwrap();
        g.record_launch(3, 25.);
        assert_eq!(g.robots[2].combat.allowance, 2);
        let restored: Match = serde_json::from_slice(&serde_json::to_vec(&g).unwrap()).unwrap();
        restored.validate().unwrap();
        assert_eq!(restored, g);
        assert_eq!(restored.fortress_reserve_left(team), 0);
        g.start(1);
        assert_eq!(g.base_lowest_hp, [10000; 2]);
        assert_eq!(g.fortress_reserve_used, [0; 2]);
    }
}

#[test]
fn fortress_thresholds_round_down_and_preserve_maximum_loss() {
    let mut g = running(Team::Red);
    for (loss, capacity) in [
        (0, 100),
        (14, 100),
        (15, 101),
        (5999, 499),
        (6000, 500),
        (9999, 500),
    ] {
        g.set_base_hp(Team::Red, 10000 - loss);
        assert_eq!(g.fortress_capacity(Team::Red), capacity);
    }
    g.set_base_hp(Team::Red, 10000);
    assert_eq!(g.fortress_capacity(Team::Red), 500);
    g.base_lowest_hp[0] = 10001;
    assert!(g.validate().is_err());
}

#[test]
fn enemy_fortress_vulnerability_combines_with_defense_and_ends_on_opening() {
    for team in Team::BOTH {
        let side = team.other();
        let mut g = running(team);
        g.set_outpost_hp(side, 0);
        g.advance_ns(THREE_MIN_NS);
        activate(&mut g, 1, side, Point::Fortress);
        assert_eq!(g.field_effects(1).vulnerability_pct, 100);
        assert_eq!(g.damage_robot(1, 20, false), 40);
        activate(&mut g, 1, team, Point::Base);
        assert_eq!(g.damage_robot(1, 20, false), 30);
        // Hold only the enemy Fortress; ten seconds opens its armor.
        for _ in 0..10 {
            activate(&mut g, 1, side, Point::Fortress);
            g.advance_ns(1_000_000_000);
        }
        assert!(g.base_open[side.index()]);
        assert_eq!(g.field_effects(1).vulnerability_pct, 0);
        assert_eq!(g.damage_robot(1, 20, false), 20);
    }
}

#[test]
fn diagnostics_explain_blocks_without_changing_authority() {
    let mut g = running(Team::Red);
    activate(&mut g, 1, Team::Blue, Point::Outpost);
    activate(&mut g, 1, Team::Blue, Point::Fortress);
    let before = g.clone();
    let diagnostics = g.control_diagnostics();
    assert_eq!(g, before);
    assert_eq!(diagnostics[0].points[4].status, Activation::OutpostNotFull);
    assert_eq!(
        diagnostics[0].points[7].status,
        Activation::FortressDisabled
    );
    g.set_outpost_hp(Team::Blue, 0);
    g.advance_ns(CAPTURE_NS);
    activate(&mut g, 1, Team::Blue, Point::Fortress);
    assert_eq!(
        g.control_diagnostics()[0].points[7].status,
        Activation::FortressTooEarly
    );
}
