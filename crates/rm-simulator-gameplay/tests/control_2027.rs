// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! UC V1.0.0 Table 5-21 and §§5.6.2, 5.6.4 regression scenarios.
use rm_simulator_gameplay::{
    Team,
    rmuc_2027::{
        CAPTURE_NS, Control, ControlState, EventKind, OutpostState, Point, SCORE_PERIOD_NS,
        Territory,
        match_rules::{
            COUNTDOWN_NS, GroundRole, Match, Phase, RFID_GRACE_NS, ResultReason, THREE_MIN_NS,
            point_index,
        },
    },
};

const SECOND: u64 = SCORE_PERIOD_NS;

// Enumerate both points independently: contention on different constituent
// points must pause just as contention on the same point does.
#[test]
fn table_5_21_all_presence_combinations_and_control_states() {
    for territory in Territory::ALL {
        let side = territory.initial_team();
        let points = if territory.index() < 2 {
            [Point::Outpost, Point::CentralHighland]
        } else {
            [Point::Base, Point::Fortress]
        };
        let states = if territory.index() < 2 {
            vec![
                ControlState::Full(side),
                ControlState::Full(side.other()),
                ControlState::Temporary(side),
                ControlState::Temporary(side.other()),
            ]
        } else {
            vec![
                ControlState::Full(side),
                ControlState::Temporary(side.other()),
            ]
        };
        for state in states {
            for mask in 0..16 {
                let mut snapshot = Control::new().snapshot().clone();
                snapshot.territories[territory.index()].control = state;
                snapshot.territories[territory.index()].progress_ns = 2 * SECOND;
                if territory.index() < 2 {
                    snapshot.outposts[side.index()] = match state {
                        ControlState::Full(owner) => OutpostState { owner, alive: true },
                        ControlState::Temporary(owner) => OutpostState {
                            owner: owner.other(),
                            alive: true,
                        },
                    };
                }
                let mut control = Control::restore(snapshot).unwrap();
                let presence: [[bool; 2]; 2] = std::array::from_fn(|p| {
                    std::array::from_fn(|t| mask & (1 << (2 * p + t)) != 0)
                });
                for (point, teams) in points.into_iter().zip(presence) {
                    control.set_activation(side, point, teams).unwrap();
                }
                let own = presence.iter().any(|p| p[state.team().index()]);
                let enemy = presence.iter().any(|p| p[state.team().other().index()]);
                // Independent truth table: full needs an uncontested challenger;
                // temporary recovers whenever its controller is absent.
                let progress = match (state, own, enemy) {
                    (_, true, true) => 2,
                    (ControlState::Full(_), false, true)
                    | (ControlState::Temporary(_), false, _) => 3,
                    _ => 1,
                };
                control.advance_ns(SECOND);
                let entry = &control.snapshot().territories[territory.index()];
                assert_eq!(entry.control, state);
                assert_eq!(
                    entry.progress_ns,
                    progress * SECOND,
                    "{territory:?} {state:?} mask={mask}"
                );
                Control::restore(control.snapshot().clone()).unwrap();
            }
        }
    }
}

#[test]
fn switching_constituent_points_keeps_progress_and_transitions_before_scoring() {
    for team in Team::BOTH {
        let side = team.other();
        let mut g = Control::new();
        let active = Team::BOTH.map(|t| t == team);
        g.set_activation(side, Point::Outpost, active).unwrap();
        g.set_activation(side, Point::Base, active).unwrap();
        g.advance_ns(2 * SECOND);
        g.set_activation(side, Point::Outpost, [false; 2]).unwrap();
        g.set_activation(side, Point::CentralHighland, active)
            .unwrap();
        g.set_activation(side, Point::Base, [false; 2]).unwrap();
        g.set_activation(side, Point::Fortress, active).unwrap();
        g.advance_ns(3 * SECOND - 1);
        assert_eq!(g.snapshot().points, [0; 2]);
        let events = g.advance_ns(1);
        assert_eq!(events.len(), 3);
        assert!(events.iter().all(|e| e.at_ns == CAPTURE_NS));
        assert!(matches!(events[0].kind, EventKind::ControlChanged { .. }));
        assert!(matches!(events[1].kind, EventKind::ControlChanged { .. }));
        assert_eq!(events[2].kind, EventKind::PointsAwarded { team, amount: 4 });
        assert!(!g.base_protected(side));
    }
}

fn running(team: Team, role: GroundRole) -> Match {
    let mut g = Match::new(1);
    g.add_robot(1, team, role);
    g.start(1);
    g.advance_ns(COUNTDOWN_NS);
    g
}

fn hold(g: &mut Match, contacts: &[(u32, Team, Point)], duration: u64) {
    let mut remaining = duration;
    while remaining > 0 {
        for &(id, side, point) in contacts {
            g.observe(id, side, point, true);
        }
        let dt = remaining.min(SECOND / 8);
        g.advance_ns(dt);
        remaining -= dt;
    }
}

#[test]
fn all_ground_roles_capture_and_rfid_expires_at_exactly_two_seconds() {
    for team in Team::BOTH {
        for role in [GroundRole::Heavy, GroundRole::Infantry, GroundRole::Sentry] {
            let side = team.other();
            let mut g = running(team, role);
            g.observe(1, side, Point::Base, true);
            g.advance_ns(RFID_GRACE_NS - 1);
            assert!(g.activations()[0].1[point_index(side, Point::Base)]);
            // A false observation must not refresh (or immediately cancel) expiry.
            g.observe(1, side, Point::Base, false);
            g.advance_ns(1);
            assert!(!g.activations()[0].1[point_index(side, Point::Base)]);
            assert_eq!(
                g.control.snapshot().territories[2 + side.index()].progress_ns,
                RFID_GRACE_NS
            );
            g.advance_ns(RFID_GRACE_NS);
            assert_eq!(
                g.control.snapshot().territories[2 + side.index()].progress_ns,
                0
            );
            hold(&mut g, &[(1, side, Point::Base)], CAPTURE_NS);
            assert_eq!(
                g.control.snapshot().territories[2 + side.index()].control,
                ControlState::Temporary(team)
            );
            g.validate().unwrap();
        }
    }
}

#[test]
fn death_offline_weakness_and_removal_revoke_control_input() {
    for team in Team::BOTH {
        for reason in 0..4 {
            let mut g = running(team, GroundRole::Infantry);
            g.observe(1, team.other(), Point::Base, true);
            g.advance_ns(SECOND);
            match reason {
                0 => g.robots[0].hp = 0,
                1 => g.robots[0].online = false,
                2 => g.robots[0].combat.weakened = true,
                _ => g.robots.clear(),
            }
            assert!(g.activations().iter().all(|(_, a)| !a.iter().any(|v| *v)));
            assert_eq!(
                g.control.snapshot().territories[2 + team.other().index()].activations,
                [[false; 2]; 2]
            );
        }
    }
}

#[test]
fn highland_latch_hands_over_at_expiry_and_simultaneous_ties_use_robot_id() {
    for team in Team::BOTH {
        let mut g = running(team, GroundRole::Infantry);
        g.add_robot(2, team.other(), GroundRole::Infantry);
        g.observe(2, team, Point::CentralHighland, true);
        g.advance_ns(SECOND);
        g.observe(1, team, Point::CentralHighland, true);
        assert_eq!(g.highland_team[team.index()], Some(team.other()));
        g.advance_ns(SECOND - 1);
        assert!(!g.activations()[0].1[point_index(team, Point::CentralHighland)]);
        g.advance_ns(1);
        assert!(g.activations()[0].1[point_index(team, Point::CentralHighland)]);
        assert_eq!(g.highland_team[team.index()], Some(team));
        g.advance_ns(SECOND);
        assert_eq!(g.highland_team[team.index()], None);
        // Deliberately observe the higher id first at the same timestamp.
        g.observe(2, team, Point::CentralHighland, true);
        g.observe(1, team, Point::CentralHighland, true);
        assert!(g.activations()[0].1[point_index(team, Point::CentralHighland)]);
        assert!(!g.activations()[1].1[point_index(team, Point::CentralHighland)]);
    }
}

#[test]
fn fortress_eligibility_and_opening_are_symmetric_and_exact() {
    for team in Team::BOTH {
        let side = team.other();
        let mut g = running(team, GroundRole::Infantry);
        g.add_robot(2, team, GroundRole::Infantry);
        let highland = (1, side, Point::CentralHighland);
        let fortress = (2, side, Point::Fortress);
        hold(&mut g, &[highland], CAPTURE_NS);
        // Temporary enemy-outpost control qualifies, but not before 3:00.
        hold(&mut g, &[highland, fortress], THREE_MIN_NS - CAPTURE_NS - 1);
        assert_eq!(g.robots[1].fortress_ns[side.index()], 0);
        hold(&mut g, &[highland, fortress], 1);
        assert!(g.activations()[1].1[point_index(side, Point::Fortress)]);
        hold(&mut g, &[highland, fortress], 10 * SECOND - 1);
        assert!(!g.base_open[side.index()]);
        hold(&mut g, &[highland, fortress], 1);
        assert!(g.base_open[side.index()]);
        g.validate().unwrap();
    }
}

#[test]
fn observation_script_is_identical_across_partitions_and_json_restore() {
    let mut coarse = running(Team::Red, GroundRole::Infantry);
    coarse.add_robot(2, Team::Blue, GroundRole::Sentry);
    let mut fine = coarse.clone();
    for n in 0..160 {
        for g in [&mut coarse, &mut fine] {
            for id in 1..=2 {
                for side in Team::BOTH {
                    for point in [Point::Base, Point::CentralHighland] {
                        g.observe(id, side, point, (n + id) % 7 < 4);
                    }
                }
            }
            if n == 70 {
                g.set_outpost_hp(Team::Blue, 0);
            }
        }
        coarse.advance_ns(137_000_003);
        for dt in [1, 7_812_500, 129_187_502] {
            fine.advance_ns(dt);
        }
        fine = serde_json::from_slice(&serde_json::to_vec(&fine).unwrap()).unwrap();
        fine.validate().unwrap();
        assert_eq!(coarse, fine, "script interval {n}");
    }
    assert_eq!(coarse.phase, Phase::Running);
}

#[test]
fn fractional_capture_boundary_rechecks_fortress_eligibility() {
    let mut coarse = running(Team::Red, GroundRole::Infantry);
    coarse.add_robot(2, Team::Red, GroundRole::Infantry);
    coarse.advance_ns(THREE_MIN_NS + 237_000_001);
    hold(
        &mut coarse,
        &[(1, Team::Blue, Point::CentralHighland)],
        9 * SECOND / 2,
    );
    coarse.observe(1, Team::Blue, Point::CentralHighland, true);
    coarse.observe(2, Team::Blue, Point::Fortress, true);
    let mut fine = coarse.clone();
    coarse.advance_ns(4 * SECOND / 5);
    fine.advance_ns(SECOND / 2);
    fine.advance_ns(3 * SECOND / 10);
    assert_eq!(coarse.robots[1].fortress_ns[1], 3 * SECOND / 10);
    assert_eq!(coarse, fine);
}

#[test]
fn fortress_retention_clears_at_exact_boundary_without_an_extra_tick() {
    let mut g = running(Team::Red, GroundRole::Infantry);
    g.add_robot(2, Team::Red, GroundRole::Infantry);
    g.advance_ns(THREE_MIN_NS);
    hold(
        &mut g,
        &[(1, Team::Blue, Point::CentralHighland)],
        CAPTURE_NS,
    );
    hold(
        &mut g,
        &[
            (1, Team::Blue, Point::CentralHighland),
            (2, Team::Blue, Point::Fortress),
        ],
        SECOND,
    );
    g.robots[1].online = false;
    hold(
        &mut g,
        &[(1, Team::Blue, Point::CentralHighland)],
        3 * SECOND - 1,
    );
    assert_eq!(g.robots[1].fortress_ns[1], SECOND);
    g.advance_ns(1);
    assert_eq!(g.robots[1].fortress_ns[1], 0);
    assert_eq!(g.robots[1].fortress_last_ns[1], None);
}

#[test]
fn transferred_outpost_and_first_defending_fortress_robot_use_current_ownership() {
    for attacker in Team::BOTH {
        let side = attacker.other();
        let mut g = running(attacker, GroundRole::Infantry);
        g.add_robot(2, side, GroundRole::Infantry);
        g.add_robot(3, side, GroundRole::Sentry);
        // Destruction and absence transfer full control to the attacker.
        g.set_outpost_hp(side, 0);
        g.advance_ns(CAPTURE_NS);
        for id in 1..=3 {
            g.observe(id, side, Point::Outpost, true);
            g.observe(id, side, Point::Fortress, true);
        }
        let active = g.activations();
        let outpost = point_index(side, Point::Outpost);
        let fortress = point_index(side, Point::Fortress);
        assert!(active[0].1[outpost]);
        assert!(!active[1].1[outpost]);
        assert!(!active[2].1[outpost]);
        assert!(!active[0].1[fortress]); // Attacker still needs the 3:00 unlock.
        assert!(active[1].1[fortress]);
        assert!(!active[2].1[fortress]);
        g.robots[1].online = false;
        assert!(g.activations()[2].1[fortress]);
        g.robots[1].online = true;
        // A returning lower id cannot steal the still-active defender's latch.
        assert!(!g.activations()[1].1[fortress]);
        assert_eq!(g.fortress_robot[side.index()], Some(3));
    }
}

#[test]
fn control_wins_timeout_but_base_destruction_precedes_every_comparison() {
    for winner in Team::BOTH {
        let mut g = running(winner, GroundRole::Infantry);
        g.add_robot(2, winner.other(), GroundRole::Heavy);
        hold(
            &mut g,
            &[(1, winner.other(), Point::CentralHighland)],
            CAPTURE_NS,
        );
        g.attack_damage[winner.other().index()] = 1_000_000;
        let mut destroyed = g.clone();
        destroyed.set_base_hp(winner, 0);
        let result = destroyed.result.as_ref().unwrap();
        assert_eq!(result.winner, Some(winner.other()));
        assert_eq!(result.reason, ResultReason::BaseDestroyed);
        g.advance_ns(rm_simulator_gameplay::rmuc_2027::ROUND_NS);
        let result = g.result.as_ref().unwrap();
        assert_eq!(result.winner, Some(winner));
        assert_eq!(result.reason, ResultReason::Control);
        let finished = g.clone();
        g.advance_ns(u64::MAX);
        assert_eq!(g, finished);
    }
}
