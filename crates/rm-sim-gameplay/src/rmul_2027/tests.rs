// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
use super::*;

#[test]
fn infantry_damage_and_lifesteal() {
    let mut g = Match::new(Format::Infantry);
    g.add_robot(3, Team::Red, Role::Infantry).unwrap();
    g.add_robot(13, Team::Blue, Role::Infantry).unwrap();
    g.start();
    g.set_robot_hp(3, 150).unwrap();
    assert_eq!(g.damage(Some(3), 13, false), 20);
    assert_eq!(g.robots[0].hp, 160);
    assert_eq!(g.robots[1].hp, 180);
}

fn duel() -> Match {
    let mut g = Match::new(Format::Infantry);
    g.add_robot(3, Team::Red, Role::Infantry).unwrap();
    g.add_robot(13, Team::Blue, Role::Infantry).unwrap();
    g.start();
    g
}
fn three() -> Match {
    let mut g = Match::new(Format::ThreeVsThree);
    for (id, team, role) in [
        (1, Team::Red, Role::Heavy),
        (3, Team::Red, Role::Infantry),
        (7, Team::Red, Role::Sentry),
        (11, Team::Blue, Role::Heavy),
        (13, Team::Blue, Role::Infantry),
        (17, Team::Blue, Role::Sentry),
    ] {
        g.add_robot(id, team, role).unwrap();
    }
    g.start();
    g
}
fn heavy_ready() -> Match {
    let mut g = three();
    g.set_heavy_installed(1, true).unwrap();
    g.observe_supply(1, true);
    g.advance(SECOND);
    g.buy_ammo(1, true, 4).unwrap();
    g
}

#[test]
fn roster_identity_and_profiles_are_ul_only() {
    let mut g = Match::new(Format::ThreeVsThree);
    g.add_robot(3, Team::Red, Role::Infantry).unwrap();
    assert!(g.add_robot(4, Team::Red, Role::Infantry).is_err());
    assert!(g.add_robot(3, Team::Blue, Role::Heavy).is_err());
    assert_eq!(g.robots[0].max_hp, 350);
    g.configure_infantry(3, true, true).unwrap();
    assert_eq!(
        g.robots[0].stats(),
        Stats {
            max_hp: 300,
            chassis_power_w: 90,
            heat_limit: 230,
            cooling_per_s: 14,
            max_speed_m_s: 25.
        }
    );
    g.start();
    assert!(g.configure_infantry(3, false, false).is_err());
    assert!(g.add_robot(7, Team::Red, Role::Sentry).is_err());
    g.reset();
    assert_eq!(g.robots[0].max_hp, 300);
    assert_eq!(g.phase, Phase::Idle);
    let mut d = Match::new(Format::Infantry);
    assert!(d.add_robot(1, Team::Red, Role::Heavy).is_err());
    d.add_robot(3, Team::Red, Role::Infantry).unwrap();
    assert_eq!(d.robots[0].allowance, 200);
    assert_eq!(d.robots[0].stats().chassis_power_w, 120);
    assert_ne!(d.ruleset, g.ruleset);
}

#[test]
fn countdown_and_timeout_freeze_exactly() {
    let mut g = Match::new(Format::Infantry);
    g.begin_countdown().unwrap();
    g.advance(4 * SECOND);
    assert_eq!(g.countdown_remaining_ns, SECOND);
    assert_eq!(g.elapsed_ns, 0);
    g.advance(2 * SECOND);
    assert_eq!(g.phase, Phase::Running);
    assert_eq!(g.elapsed_ns, SECOND);
    g.advance(u64::MAX);
    assert_eq!(g.elapsed_ns, 120 * SECOND);
    assert_eq!(g.phase, Phase::Finished);
    assert_eq!(g.remaining_ns(), 0);
    assert_eq!(g.result.as_ref().unwrap().winner, None);
    assert!(g.validate().is_ok());
    let frozen = g.clone();
    g.advance(u64::MAX);
    g.start();
    g.finish();
    assert_eq!(g, frozen);
}

#[test]
fn lifesteal_is_actual_enemy_projectile_loss_only_and_rounds_half_up() {
    let mut g = duel();
    g.set_robot_hp(3, 100).unwrap();
    g.set_robot_hp(13, 1).unwrap();
    assert_eq!(g.damage(Some(3), 13, false), 1);
    assert_eq!(g.robots[0].hp, 101);
    assert_eq!(g.teams[0].attack_damage, 1);
    assert_eq!(g.result.as_ref().unwrap().winner, Some(Team::Red));
    let mut g = three();
    assert_eq!(g.damage(Some(3), 1, false), 0);
    assert_eq!(g.robots[0].hp, 100);
    assert_eq!(g.damage(None, 3, false), 20);
    assert_eq!(g.teams[0].attack_damage, 0);
    assert_eq!(g.collision(3), 2);
}

#[test]
fn infantry_tiebreaks_use_specific_full_hp_branch_after_healing() {
    let mut g = duel();
    g.damage(Some(13), 3, false);
    g.advance(SECOND);
    g.damage(Some(3), 13, false);
    // Red was damaged earlier but both have healed; specific full-HP clauses win.
    g.set_robot_hp(3, 200).unwrap();
    g.set_robot_hp(13, 200).unwrap();
    assert!(g.fire(3, false, 25.));
    g.finish();
    assert_eq!(g.result.as_ref().unwrap().winner, Some(Team::Red));
    let mut g = duel();
    g.fire(3, false, 25.);
    g.advance(SECOND);
    g.fire(13, false, 25.);
    g.finish();
    assert_eq!(g.result.as_ref().unwrap().winner, Some(Team::Red));
    let mut g = duel();
    g.damage(Some(13), 3, false);
    g.advance(SECOND);
    g.damage(Some(3), 13, false);
    // Both at 190 after red's lifesteal: blue's first damage was later.
    g.set_robot_hp(3, 180).unwrap();
    g.set_robot_hp(13, 180).unwrap();
    g.finish();
    assert_eq!(g.result.as_ref().unwrap().winner, Some(Team::Blue));
}

#[test]
fn small_overspeed_bands_are_independent_of_heat_and_connectivity() {
    for (speed, seconds, permanent) in [
        (25., 0, false),
        (29.99, 15, false),
        (30., 20, false),
        (34.99, 20, false),
        (35., 0, true),
    ] {
        let mut g = duel();
        assert!(g.fire(3, false, speed));
        assert_eq!(g.robots[0].overspeed_until_ns, seconds * SECOND);
        assert_eq!(g.robots[0].overspeed_locked_for_round, permanent);
        g.observe_modules(3, true, false, 0);
        assert!(g.check_launch(3, false).is_err());
        g.observe_modules(3, true, true, 0);
        if seconds > 0 {
            g.advance(seconds * SECOND - 1);
            assert!(g.check_launch(3, false).is_err());
            g.advance(1);
            assert!(g.check_launch(3, false).is_ok());
        }
    }
    let mut g = duel();
    let before = g.clone();
    assert!(!g.fire(3, false, f64::NAN));
    assert!(!g.fire(3, true, 16.5));
    assert_eq!(g, before);
}

#[test]
fn large_overspeed_uses_ul_ratios_and_installation_is_separate() {
    for (speed, seconds, permanent) in [
        (16.5, 0, false),
        (18.15, 15, false),
        (18.16, 20, false),
        (19.8, 20, false),
        (19.81, 0, true),
    ] {
        let mut g = heavy_ready();
        assert_eq!(g.robots[0].max_hp, 100);
        assert!(g.fire(1, true, speed));
        assert_eq!(g.robots[0].max_hp, 450);
        assert_eq!(
            g.robots[0].overspeed_until_ns,
            if seconds == 0 {
                0
            } else {
                (1 + seconds) * SECOND
            }
        );
        assert_eq!(g.robots[0].overspeed_locked_for_round, permanent);
    }
}

#[test]
fn heat_lock_cools_to_zero_and_inclusive_permanent_threshold_survives_death() {
    let mut g = duel();
    for _ in 0..9 {
        assert!(g.fire(3, false, 25.));
    }
    assert_eq!(g.robots[0].heat_tenths, 900);
    assert!(g.robots[0].overheated);
    assert!(!g.fire(3, false, 25.));
    g.advance(SECOND);
    assert!(g.robots[0].overheated);
    g.advance(7 * SECOND);
    assert!(!g.robots[0].overheated);
    assert!(g.check_launch(3, false).is_ok());
    for _ in 0..18 {
        assert!(g.record_launch(3, false, 25.));
    }
    assert_eq!(g.robots[0].heat_tenths, 1800);
    assert!(g.robots[0].heat_locked_for_round);
    let mut g = three();
    for _ in 0..36 {
        g.record_launch(7, false, 25.);
    }
    assert!(g.robots[2].heat_locked_for_round);
    g.penalty_damage(7, 400);
    assert_eq!(g.robots[2].heat_tenths, 0);
    g.advance(5 * SECOND);
    g.observe_supply(7, true);
    assert!(g.check_launch(7, false).is_err());
}

#[test]
fn last_legal_large_round_scores_but_overfiring_suppresses() {
    let mut g = heavy_ready();
    g.robots[0].allowance = 1;
    assert!(g.fire(1, true, 16.5));
    assert_eq!(g.robots[0].allowance, 0);
    assert_eq!(g.damage(Some(1), 13, true), 200);
    assert!(g.record_launch(1, true, 16.5));
    assert_eq!(g.damage(Some(1), 17, true), 0);
    g.buy_ammo(1, true, 1).unwrap();
    assert_eq!(g.damage(Some(1), 17, true), 200);
}

#[test]
fn large_inactivity_death_and_link_suppression_are_separate_gates() {
    let mut g = heavy_ready();
    g.fire(1, true, 16.5);
    g.advance(4 * SECOND);
    assert_eq!(g.damage(Some(1), 13, true), 0);
    g.fire(1, true, 16.5);
    assert!(g.observe_launcher_link(1, false));
    assert_eq!(g.damage(Some(1), 13, true), 0);
    g.observe_launcher_link(1, true);
    assert_eq!(g.damage(Some(1), 13, true), 200);
    g.penalty_damage(1, 450);
    g.advance(3 * SECOND);
    assert_eq!(g.damage(Some(1), 17, true), 0);
    let mut g = heavy_ready();
    g.fire(1, true, 16.5);
    g.penalty_damage(1, 450);
    for _ in 0..3 {
        assert!(g.record_launch(1, true, 16.5));
    }
    assert_eq!(g.damage(Some(1), 17, true), 0);
}

#[test]
fn supply_income_purchase_and_sentry_allowance() {
    let mut g = three();
    assert!(g.buy_ammo(3, false, 1).is_err());
    g.advance(SECOND - 1);
    assert_eq!(g.teams[0].gold, 0);
    g.advance(1);
    assert_eq!(g.teams[0].gold, 200);
    g.observe_supply(3, true);
    g.buy_ammo(3, false, 2).unwrap();
    assert_eq!(g.teams[0].gold, 180);
    assert_eq!(g.robots[1].allowance, 20);
    g.observe_supply(7, true);
    assert!(g.buy_ammo(7, false, 1).is_err());
    g.advance(240 * SECOND);
    assert_eq!(g.teams[0].gold, 1180);
    let mut d = duel();
    assert!(!d.observe_supply(3, true));
    assert!(d.buy_ammo(3, false, 1).is_err());
    d.advance(SECOND);
    assert_eq!(d.teams[0].gold, 0);
}

#[test]
fn control_capture_contest_reversal_and_grace() {
    let mut g = three();
    g.observe_zone(3, Zone::Center, true);
    g.advance(5 * SECOND);
    assert_eq!(g.zones[2].controller, Some(Team::Red));
    assert_eq!(g.teams[1].control, 599);
    g.observe_zone(13, Zone::Center, true);
    g.advance(SECOND);
    assert_eq!(g.zones[2].progress_ns, [5 * SECOND, 0]);
    g.observe_zone(3, Zone::Center, false);
    g.advance(2 * SECOND);
    assert_eq!(g.zones[2].progress_ns, [5 * SECOND, 0]);
    g.advance(5 * SECOND);
    assert_eq!(g.zones[2].controller, None);
    g.advance(5 * SECOND);
    assert_eq!(g.zones[2].controller, Some(Team::Blue));
    g.observe_zone(13, Zone::Center, false);
    g.advance(7 * SECOND);
    assert_eq!(g.zones[2], ZoneState::default());
}

#[test]
fn highland_defense_supply_immunity_and_grace() {
    let mut g = three();
    g.observe_zone(3, Zone::RedHighland, true);
    assert_eq!(g.damage(Some(13), 3, false), 15);
    assert_eq!(g.collision(3), 2);
    g.observe_zone(3, Zone::RedHighland, false);
    g.advance(2 * SECOND - 1);
    assert_eq!(g.damage(Some(13), 3, false), 15);
    g.advance(1);
    assert_eq!(g.damage(Some(13), 3, false), 20);
    g.observe_supply(1, true);
    assert_eq!(g.damage(Some(13), 1, false), 0);
    g.penalty_damage(1, 1);
    assert_eq!(g.robots[0].hp, 99);
    g.set_heavy_installed(1, true).unwrap();
    g.advance(SECOND);
    g.buy_ammo(1, true, 1).unwrap();
    g.fire(1, true, 16.5);
    assert_eq!(g.damage(Some(13), 1, false), 20);
}

#[test]
fn respawn_weakness_is_not_the_thirty_second_invincibility_timer() {
    let mut g = three();
    g.penalty_damage(3, 350);
    assert_eq!(g.teams[0].control, 590);
    g.advance(5 * SECOND - 1);
    assert_eq!(g.robots[1].hp, 0);
    g.advance(1);
    assert_eq!(g.robots[1].hp, 70);
    assert!(g.robots[1].weak);
    assert_eq!(g.damage(Some(13), 3, false), 0);
    g.observe_zone(3, Zone::Center, true);
    g.advance(30 * SECOND);
    assert_eq!(g.zones[2].progress_ns, [0, 0]);
    assert!(g.robots[1].weak);
    assert_eq!(g.damage(Some(13), 3, false), 20);
    g.observe_supply(3, true);
    assert!(!g.robots[1].weak);
    g.penalty_damage(3, 350);
    g.advance(10 * SECOND - 1);
    assert_eq!(g.robots[1].hp, 0);
    g.advance(1);
    assert_eq!(g.robots[1].hp, 70);
}

#[test]
fn respawn_after_off_grid_death_never_gets_early_progress() {
    let mut g = three();
    g.advance(PERIOD / 2);
    g.penalty_damage(3, 350);
    g.advance(5 * SECOND);
    assert_eq!(g.robots[1].hp, 0);
    g.advance(PERIOD / 2);
    assert_eq!(g.robots[1].hp, 70);
}

#[test]
fn catchup_is_dynamic_and_gold_is_once_per_threshold() {
    let mut g = three();
    g.teams[0].control = 400;
    g.advance(PERIOD);
    assert_eq!(g.teams[0].gold, 200);
    assert_eq!(g.teams[0].catch_up_awarded, [true, false]);
    g.teams[0].control = 200;
    g.advance(PERIOD);
    assert_eq!(g.teams[0].gold, 400);
    g.penalty_damage(3, 350);
    g.advance(2_500_000_000);
    assert_eq!(g.robots[1].hp, 70);
    g.observe_supply(3, true);
    g.advance(SECOND);
    assert_eq!(g.robots[1].hp, 245);
    g.advance(SECOND);
    assert_eq!(g.teams[0].gold, 600);
}

#[test]
fn tunnel_pair_is_own_side_bounded_and_cancelled_by_other_cards() {
    let mut g = three();
    assert!(!g.observe_tunnel(3, Team::Blue, 0));
    assert!(g.observe_tunnel(3, Team::Red, 0));
    g.observe_zone(3, Zone::Center, true);
    g.observe_tunnel(3, Team::Red, 1);
    assert_eq!(g.robots[1].cooling_until_ns, 0);
    g.observe_other_card(3);
    g.observe_tunnel(3, Team::Red, 0);
    g.advance(3 * SECOND);
    g.observe_tunnel(3, Team::Red, 1);
    assert_eq!(g.robots[1].cooling_until_ns, 63 * SECOND);
    g.record_launch(3, false, 25.);
    g.advance(PERIOD);
    assert_eq!(g.robots[1].heat_tenths, 64);
    g.advance(60 * SECOND);
    g.record_launch(3, false, 25.);
    g.advance(PERIOD);
    assert_eq!(g.robots[1].heat_tenths, 76);
}

#[test]
fn power_buffer_cut_and_offline_module_penalties() {
    let mut g = duel();
    assert!(g.observe_power(3, 220.));
    g.advance(600_000_000);
    assert_eq!(g.robots[0].buffer_mj, 0);
    assert!(!g.chassis_enabled(3));
    g.observe_power(3, 0.);
    g.advance(5 * SECOND);
    assert!(g.chassis_enabled(3));
    g.observe_modules(3, true, true, 2);
    g.advance(500_000_000);
    assert_eq!(g.robots[0].hp, 192);
    g.observe_modules(3, false, true, 2);
    assert_eq!(g.damage(Some(13), 3, false), 0);
    g.advance(SECOND);
    assert_eq!(g.robots[0].hp, 182);
    assert!(!g.chassis_enabled(3));
    assert!(!g.observe_power(3, f64::NAN));
    assert!(!g.observe_power(3, -1.));
}

#[test]
fn zero_control_and_timeout_comparisons() {
    let mut g = three();
    g.teams[0].control = 10;
    g.penalty_damage(3, 350);
    assert_eq!(g.phase, Phase::Finished);
    assert_eq!(g.result.as_ref().unwrap().winner, Some(Team::Blue));
    let mut g = three();
    g.damage(Some(3), 13, false);
    g.advance(300 * SECOND);
    assert_eq!(g.result.as_ref().unwrap().winner, Some(Team::Red));
}

#[test]
fn checkpoints_keep_all_hidden_clocks_and_reject_identity_leaks() {
    let mut g = heavy_ready();
    g.fire(1, true, 18.16);
    g.observe_zone(3, Zone::Center, true);
    g.observe_tunnel(7, Team::Red, 0);
    g.penalty_damage(13, 350);
    g.advance(1_234_567_891);
    let encoded = serde_json::to_string(&g).unwrap();
    let mut copy: Match = serde_json::from_str(&encoded).unwrap();
    assert_eq!(g, copy);
    copy.validate().unwrap();
    g.advance(87_654_321_123);
    for _ in 0..87 {
        copy.advance(SECOND);
    }
    copy.advance(654_321_123);
    assert_eq!(g, copy);
    let mut bad = duel();
    bad.ruleset = Ruleset::Rmul2027V1;
    assert!(bad.validate().is_err());
    let mut bad = duel();
    bad.teams[0].gold = 1;
    assert!(bad.validate().is_err());
    let mut bad = duel();
    bad.zones[0].progress_ns[0] = 1;
    assert!(bad.validate().is_err());
    let mut bad = duel();
    bad.robots[0].first_launch_ns = Some(1);
    assert!(bad.validate().is_err());
    let mut bad = duel();
    bad.finish();
    bad.result.as_mut().unwrap().elapsed_ns = 1;
    assert!(bad.validate().is_err());
    let mut bad = three();
    bad.robots[0].buffer_mj = 60_001;
    assert!(bad.validate().is_err());
}
