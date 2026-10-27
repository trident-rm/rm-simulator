// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Independent regression cases from the RMUC/RMUL V1.0.0 rule audit.
use rm_sim_world::{
    ChassisCommand, ChassisConfig, ChassisPlacement, Field, FieldConfig, Pose, RefereeCommand,
    RobotKind, Team as PhysicalTeam,
    gameplay::{
        Caliber, InfantryChassis, InfantryLauncher, Target, Team,
        rmuc_2027::match_rules::{COUNTDOWN_NS, GroundRole, Match as UcMatch},
        rmul_2027::{Format, Match as UlMatch, Role},
    },
    tick_ns,
};

const SECOND: u64 = 1_000_000_000;

fn uc() -> UcMatch {
    let mut g = UcMatch::new(1);
    g.add_robot(1, Team::Red, GroundRole::Heavy);
    g.add_robot(3, Team::Red, GroundRole::Infantry);
    g.add_robot(4, Team::Blue, GroundRole::Infantry);
    g.start(1);
    g.advance_ns(COUNTDOWN_NS);
    g
}

#[test]
fn uc_heavy_42mm_damage_cannot_award_xp_to_17mm_infantry() {
    // UC Table 5-14 restricts fallback projectile recipients to robots able
    // to deal that caliber's damage AND eligible to acquire experience.
    let mut g = uc();
    g.advance_ns(300 * SECOND);
    let heavy = g.robots.iter_mut().find(|r| r.id == 1).unwrap();
    heavy.combat.heavy_installed = true;
    heavy.max_hp = 600;
    g.robots
        .iter_mut()
        .find(|r| r.id == 3)
        .unwrap()
        .combat
        .allowance = 100;
    assert!(g.heavy_unlocked(Team::Red));
    assert_eq!(g.hit(Target::Robot(4), Some(1), Caliber::Mm42), 200);
    let xp = g
        .robots
        .iter()
        .find(|r| r.id == 3)
        .unwrap()
        .combat
        .experience_tenths;
    assert_eq!(xp, 0, "17mm Infantry cannot receive fallback 42mm XP");
}

#[test]
fn uc_non_projectile_death_distributes_unidentified_kill_xp() {
    // UC Table 5-14: deaths from causes other than armor attacks have no
    // identified killer; non-projectile kill experience is shared among live Infantry.
    let mut g = uc();
    assert_eq!(g.damage_robot(4, 200, true), 200);
    let xp = g
        .robots
        .iter()
        .find(|r| r.id == 3)
        .unwrap()
        .combat
        .experience_tenths;
    assert!(
        xp >= 500,
        "level-one death must award at least its 50 XP kill bonus, got {xp}/10 XP"
    );
}

#[test]
fn uc_explicit_pre_round_performance_cannot_change_after_start() {
    // UC §5.5.2: selected performance types cannot change in the running round.
    let mut g = UcMatch::new(1);
    g.add_robot(3, Team::Red, GroundRole::Infantry);
    g.set_performance(
        3,
        InfantryChassis::PowerFocused,
        InfantryLauncher::BurstFocused,
    )
    .unwrap();
    g.start(1);
    g.advance_ns(COUNTDOWN_NS);
    assert!(
        g.set_performance(
            3,
            InfantryChassis::HpFocused,
            InfantryLauncher::CoolingFocused
        )
        .is_err()
    );
}

#[test]
fn ul_power_buffer_recharges_during_chassis_cut() {
    // UL §3.3.1.4/Figure 3-13 settles Z -= (Pr-Pl)*0.1 at 10 Hz.
    let mut g = UlMatch::new(Format::ThreeVsThree);
    g.add_robot(3, Team::Red, Role::Infantry).unwrap();
    g.start();
    assert!(g.observe_power(3, 675.)); // 600 W above limit for 100 ms = 60 J.
    g.advance(SECOND / 10);
    assert_eq!(g.robots[0].buffer_mj, 0);
    assert!(!g.chassis_enabled(3));
    assert!(g.observe_power(3, 0.));
    g.advance(SECOND);
    assert_eq!(
        g.robots[0].buffer_mj, 60_000,
        "zero measured power should refill the buffer during a cut"
    );
}

#[test]
fn ul_over_power_cut_does_not_cut_gimbal_actuation() {
    // UL §3.3.1.4 cuts chassis power only; §3.3.1.5 separately cuts all
    // actuation when main-controller offline. Exercise the real Field adapter.
    let mut field = Field::new(&FieldConfig {
        rmul_2027: Some(Format::ThreeVsThree),
        ..Default::default()
    })
    .unwrap();
    let config = ChassisConfig::default();
    let id = field
        .add_chassis(&ChassisPlacement {
            spawn: Pose::at([0., 3., config.rest_height_m()]),
            config,
            kind: RobotKind::Infantry,
            team: PhysicalTeam::Red,
            performance: None,
        })
        .unwrap();
    field.referee_command(RefereeCommand::StartMatch).unwrap();
    field.step(5 * SECOND / tick_ns()).unwrap();
    let geometry = field.static_geometry_snapshot();
    let mut checkpoint = field.snapshot();
    let mut uncut = Field::restore(&checkpoint, &geometry, 0.).unwrap();
    uncut
        .command_chassis(
            id,
            ChassisCommand {
                aim_yaw_rad: 1.,
                ..Default::default()
            },
        )
        .unwrap();
    uncut.step(SECOND / tick_ns()).unwrap();
    assert!(
        uncut.chassis_snapshot(id).unwrap().held_aim_rad[0] > 0.1,
        "positive control must move the gimbal"
    );
    let game = checkpoint.rmul_2027.as_mut().unwrap();
    game.robots[0].buffer_mj = 0;
    game.robots[0].power_cut_until_ns = game.elapsed_ns + 5 * SECOND;
    checkpoint.restore.as_mut().unwrap().rmul_2027 = Some(game.clone());
    let mut field = Field::restore(&checkpoint, &geometry, 0.).unwrap();
    field
        .command_chassis(
            id,
            ChassisCommand {
                aim_yaw_rad: 1.,
                ..Default::default()
            },
        )
        .unwrap();
    field.step(SECOND / tick_ns()).unwrap();
    let aim = field.chassis_snapshot(id).unwrap().held_aim_rad[0];
    assert!(
        aim > 0.1,
        "chassis-only power cut froze the gimbal at {aim}"
    );
}
