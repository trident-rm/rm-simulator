// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Standalone equipment/live authority parity with explicit running time.
use rm_sim_gameplay::{
    Caliber, Target, Team,
    rmuc_2027::{
        CAPTURE_NS, Control, Point,
        equipment::{Bases, Outposts},
        match_rules::{COUNTDOWN_NS, GroundRole, Match, Phase},
        rune::{Hit, Rune},
    },
};

const S: u64 = 1_000_000_000;

fn running() -> Match {
    let mut game = Match::new(-1);
    game.add_robot(1, Team::Blue, GroundRole::Infantry);
    game.start(-1);
    game.advance_ns(COUNTDOWN_NS);
    game
}

#[test]
fn outpost_destruction_rebuild_and_permanent_stop_match_live() {
    for side in Team::BOTH {
        let mut game = running();
        game.advance_ns(2 * S);
        let mut control = game.control.clone();
        let mut hp = game.outpost_hp;
        let mut stopped = game.rotor_stopped_ns;
        assert_eq!(
            Outposts::new(&mut control, &mut hp, &mut stopped).damage(side, 1000),
            1000
        );
        game.set_outpost_hp(side, 0);
        assert_eq!(control, game.control);
        assert_eq!(hp, game.outpost_hp);
        assert_eq!(stopped, game.rotor_stopped_ns);

        // An operator cannot resurrect a destroyed outpost.
        Outposts::new(&mut control, &mut hp, &mut stopped).set_hp(side, 1000);
        game.set_outpost_hp(side, 1000);
        assert_eq!(hp[side.index()], 0);
        let events = control.advance_ns(CAPTURE_NS);
        Outposts::new(&mut control, &mut hp, &mut stopped).apply_events(&events);
        game.advance_ns(CAPTURE_NS);
        assert_eq!(control, game.control);
        assert_eq!(hp, game.outpost_hp);
        assert_eq!(hp[side.index()], 500);
        assert_eq!(stopped, game.rotor_stopped_ns);
        assert_eq!(stopped[side.index()], Some(2 * S));
        assert_eq!(
            control.snapshot().outposts[side.index()].owner,
            side.other()
        );

        // Rebuilding does not restart the rotor, even after a JSON checkpoint.
        let mut restored: Match =
            serde_json::from_str(&serde_json::to_string(&game).unwrap()).unwrap();
        game.advance_ns(180 * S);
        restored.advance_ns(180 * S);
        // Standalone scheduling visits the three-minute rotor boundary just
        // as Match does, then advances the remaining time.
        for dt_ns in [180 * S - control.snapshot().elapsed_ns, 7 * S] {
            let events = control.advance_ns(dt_ns);
            let mut view = Outposts::new(&mut control, &mut hp, &mut stopped);
            view.apply_events(&events);
            view.settle_rotors(game.base_open);
        }
        assert_eq!(game, restored);
        assert_eq!(hp, game.outpost_hp);
        assert_eq!(stopped, game.rotor_stopped_ns);
        game.validate().unwrap();
    }
}

#[test]
fn captured_outpost_lethal_hit_credits_old_owner_and_rebuilds_immediately() {
    let mut game = running();
    // Use the same control authority to prepare a temporary enemy capture.
    game.control
        .set_activation(Team::Red, Point::Outpost, [false, true])
        .unwrap();
    game.control.advance_ns(CAPTURE_NS);
    let mut control = game.control.clone();
    let mut hp = [20, 1000];
    let mut stopped = game.rotor_stopped_ns;
    game.outpost_hp = hp;
    assert_eq!(
        Outposts::new(&mut control, &mut hp, &mut stopped).damage(Team::Red, 20),
        20
    );
    assert_eq!(
        game.hit(Target::Outpost(Team::Red), Some(1), Caliber::Mm17),
        20
    );
    assert_eq!(hp, game.outpost_hp);
    assert_eq!(hp[0], 500);
    assert_eq!(stopped, game.rotor_stopped_ns);
    assert_eq!(
        control.snapshot().outposts,
        game.control.snapshot().outposts
    );
    assert_eq!(game.attack_damage, [0, 20]);
}

#[test]
fn base_protection_overkill_and_minimum_history_match_live() {
    for side in Team::BOTH {
        let mut game = running();
        let mut hp = game.base_hp;
        let mut lowest = game.base_lowest_hp;
        assert_eq!(
            Bases::new(&game.control, &mut hp, &mut lowest).damage(side, 20),
            0
        );
        assert_eq!(game.hit(Target::Base(side), Some(1), Caliber::Mm17), 0);
        game.set_outpost_hp(side, 0);
        for value in [15, 10_001, 19] {
            Bases::new(&game.control, &mut hp, &mut lowest).set_hp(side, value);
            game.set_base_hp(side, value);
            assert_eq!(hp, game.base_hp);
            assert_eq!(lowest, game.base_lowest_hp);
        }
        let expected = Bases::new(&game.control, &mut hp, &mut lowest).damage(side, 20);
        assert_eq!(
            game.hit(Target::Base(side), Some(1), Caliber::Mm17),
            expected
        );
        assert_eq!(expected, 19);
        assert_eq!(hp, game.base_hp);
        assert_eq!(lowest, game.base_lowest_hp);
        assert_eq!(game.phase, Phase::Finished);
        assert_eq!(game.result.as_ref().unwrap().winner, Some(side.other()));
        // Standalone damage has no reward authority; Match keeps enemy-only credit.
        assert_eq!(
            game.attack_damage[1],
            if side == Team::Red { 19 } else { 0 }
        );
    }
}

#[test]
fn opposing_base_open_and_three_minute_rotor_boundaries_match_live() {
    for opening in [false, true] {
        let mut game = running();
        let mut control = Control::new();
        let mut hp = game.outpost_hp;
        let mut stopped = game.rotor_stopped_ns;
        game.base_open[1] = opening;
        for dt_ns in [1, S - 1, 179 * S - 1, 1, S] {
            let events = control.advance_ns(dt_ns);
            let mut outposts = Outposts::new(&mut control, &mut hp, &mut stopped);
            outposts.apply_events(&events);
            // Match can split at earlier native boundaries. Align the first
            // explicit opening observation with its next settlement at one nanosecond.
            game.advance_ns(dt_ns);
            outposts.settle_rotors(game.base_open);
            assert_eq!(stopped, game.rotor_stopped_ns);
        }
    }
}

fn advance_rune(rune: &mut Rune, now_ns: &mut u64, duration_ns: u64) {
    let target_ns = *now_ns + duration_ns;
    while *now_ns < target_ns {
        *now_ns += rune.boundary(*now_ns).min(target_ns - *now_ns);
        rune.settle(*now_ns);
    }
}

#[test]
fn existing_rune_authority_matches_live_attempts_benefits_and_hidden_timers() {
    let mut game = running();
    let mut rune = game.runes[1].clone();
    let mut now_ns = game.elapsed_ns();
    for stage_time_ns in [0, 180 * S] {
        advance_rune(&mut rune, &mut now_ns, stage_time_ns - game.elapsed_ns());
        game.advance_ns(stage_time_ns - game.elapsed_ns());
        assert_eq!(rune, game.runes[1]);
        rune.activate(now_ns).unwrap();
        game.activate_rune(Team::Blue).unwrap();
        loop {
            let blade = rune.targets.iter().position(|&lit| lit).unwrap() as u8;
            let hit = rune.hit(now_ns, blade, 10);
            assert_eq!(game.hit_rune(Team::Blue, blade, 10), hit);
            assert_eq!(rune, game.runes[1]);
            if hit == Hit::Completed {
                break;
            }
            advance_rune(&mut rune, &mut now_ns, S / 10);
            game.advance_ns(S / 10);
            assert_eq!(rune, game.runes[1]);
        }
        if stage_time_ns > 0 {
            assert!(game.robots[0].combat.experience_tenths > 0);
        }
        let remaining_ns = rune.buff.as_ref().unwrap().until_ns - now_ns;
        advance_rune(&mut rune, &mut now_ns, remaining_ns);
        game.advance_ns(remaining_ns);
        assert_eq!(rune, game.runes[1]);
        assert!(rune.buff.is_none());
        game.validate().unwrap();
    }
}
