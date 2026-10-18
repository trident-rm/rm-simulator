// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Frozen selection-time areas and Table 5-5 damage integration.
use rm_simulator_gameplay::{
    Caliber, Target, Team,
    rmuc_2027::{
        adaptive::{Impact, Region, Selection},
        match_rules::{COUNTDOWN_NS, GroundRole, Match},
    },
};
const S: u64 = 1_000_000_000;
fn running() -> Match {
    let mut g = Match::new(1);
    g.add_robot(1, Team::Red, GroundRole::Infantry);
    g.add_robot(2, Team::Blue, GroundRole::Infantry);
    g.start(1);
    g.advance_ns(COUNTDOWN_NS + 60 * S);
    g.adaptive.launches = [20; 2];
    g
}
fn impact(target: Target, p: [u32; 2]) -> Impact {
    Impact {
        time_ns: 0,
        attacker: None,
        victim: Team::Red,
        target,
        plate: 0,
        position_um: p,
        size_um: [90_000, 88_000],
        region: Region::Default,
    }
}
fn populate(g: &mut Match, team: Team) {
    for i in 0..10 {
        // Protected Base contacts still establish accuracy; no damage is needed.
        assert_eq!(
            g.armor_hit(
                impact(
                    Target::Base(team.other()),
                    [10_000 + i % 2 * 20_000, 20_000]
                ),
                Some(if team == Team::Red { 1 } else { 2 }),
                Caliber::Mm17
            ),
            0
        );
    }
}
fn ready() -> Match {
    let mut g = running();
    for t in Team::BOTH {
        populate(&mut g, t);
    }
    g
}
#[test]
fn prerequisites_spending_and_rejection_preserve_state() {
    let mut g = running();
    let before = g.clone();
    assert!(g.select_reinforcement(Team::Red, [45_000, 44_000]).is_err());
    assert_eq!(g, before);
    populate(&mut g, Team::Red);
    g.select_reinforcement(Team::Red, [45_000, 44_000]).unwrap();
    assert_eq!(g.adaptive.opportunities, [0, 1]);
    assert_eq!(g.adaptive.selections, [1, 0]);
    let before = g.clone();
    assert!(g.select_reinforcement(Team::Red, [0, 0]).is_err());
    assert_eq!(g, before);
    g.validate().unwrap();
    g.finish();
    assert!(g.select_reinforcement(Team::Blue, [0, 0]).is_err());
}
#[test]
fn area_is_frozen_until_another_selection_and_history_is_immutable() {
    let mut g = ready();
    g.select_reinforcement(Team::Red, [45_000, 44_000]).unwrap();
    let selection = g.adaptive.selected[0].clone().unwrap();
    assert_eq!(selection.area_share_ppb, Some(333_333_333));
    let hit = impact(Target::Outpost(Team::Red), [45_000, 44_000]);
    assert_eq!(g.armor_hit(hit.clone(), Some(2), Caliber::Mm17), 10);
    let past = g.adaptive.impacts.last().unwrap().clone();
    g.adaptive.launches[0] *= 10;
    assert!(g.adaptive.area_share(Team::Red).unwrap() < 0.05);
    g.advance_ns(60 * S);
    assert_eq!(g.adaptive.selected[0].as_ref(), Some(&selection));
    assert_eq!(g.armor_hit(hit, Some(2), Caliber::Mm17), 10);
    let new_share = (g.adaptive.area_share(Team::Red).unwrap() * 1e9).round() as u32;
    g.select_reinforcement(Team::Red, [0, 0]).unwrap();
    assert_eq!(
        g.adaptive.selected[0].as_ref().unwrap().area_share_ppb,
        Some(new_share)
    );
    assert_eq!(&g.adaptive.impacts[20], &past);
    assert_eq!(g.adaptive.opportunities[0], 0);
    g.validate().unwrap();
}
#[test]
fn undefined_selection_freezes_default_even_after_statistics_become_defined() {
    let mut g = running();
    populate(&mut g, Team::Red);
    g.select_reinforcement(Team::Red, [45_000, 44_000]).unwrap();
    assert_eq!(
        g.adaptive.selected[0].as_ref().unwrap().area_share_ppb,
        None
    );
    populate(&mut g, Team::Blue);
    assert!(g.adaptive.area_share(Team::Red).is_ok());
    assert_eq!(
        g.armor_hit(
            impact(Target::Outpost(Team::Red), [45_000, 44_000]),
            Some(2),
            Caliber::Mm17
        ),
        20
    );
    assert_eq!(g.adaptive.impacts.last().unwrap().region, Region::Default);
    g.advance_ns(60 * S);
    g.select_reinforcement(Team::Red, [45_000, 44_000]).unwrap();
    assert_eq!(
        g.armor_hit(
            impact(Target::Outpost(Team::Red), [45_000, 44_000]),
            Some(2),
            Caliber::Mm17
        ),
        10
    );
}
#[test]
fn circles_scale_by_detector_area_are_clipped_without_resizing_and_include_boundary() {
    let s = Selection {
        center_um: [0, 0],
        area_share_ppb: Some(333_333_333),
        selected_ns: 60 * S,
    };
    let radius = s.radius_squared_um2([100_000; 2]).unwrap().sqrt();
    assert_eq!(
        s.region([radius.floor() as u32, 0], [100_000; 2]),
        Region::Reinforced
    );
    assert_eq!(
        s.region([radius.ceil() as u32, 0], [100_000; 2]),
        Region::Unreinforced
    );
    assert_eq!(s.region([25_000, 0], [100_000; 2]), Region::Reinforced);
    assert_eq!(s.region([25_000, 0], [50_000; 2]), Region::Unreinforced);
    // A zero-area circle still contains its center, exercising exact equality.
    let point = Selection {
        area_share_ppb: Some(0),
        ..s
    };
    assert_eq!(point.region([0, 0], [100_000; 2]), Region::Reinforced);
    assert_eq!(point.region([1, 0], [100_000; 2]), Region::Unreinforced);
}
#[test]
fn regions_multiply_field_defense_and_apply_to_both_sides() {
    for team in Team::BOTH {
        let mut g = ready();
        g.select_reinforcement(team, [45_000, 44_000]).unwrap();
        let robot = if team == Team::Red { 1 } else { 2 };
        let shooter = if team == Team::Red { 2 } else { 1 };
        assert_eq!(
            g.armor_hit(
                impact(Target::Robot(robot), [45_000, 44_000]),
                Some(shooter),
                Caliber::Mm17
            ),
            10
        );
        assert_eq!(
            g.armor_hit(
                impact(Target::Robot(robot), [90_000, 88_000]),
                Some(shooter),
                Caliber::Mm17
            ),
            30
        );
        g.observe(
            robot,
            team,
            rm_simulator_gameplay::rmuc_2027::Point::Base,
            true,
        );
        g.advance_ns(0);
        assert_eq!(
            g.armor_hit(
                impact(Target::Robot(robot), [90_000, 88_000]),
                Some(shooter),
                Caliber::Mm17
            ),
            15
        );
        let last = g.adaptive.impacts.last().unwrap();
        assert_eq!(last.region, Region::Unreinforced);
        assert_eq!(last.attacker, Some(team.other()));
        assert_eq!(last.victim, team);
    }
}
#[test]
fn restore_death_respawn_round_reset_and_malformed_checkpoints() {
    let mut g = ready();
    g.select_reinforcement(Team::Red, [45_000, 44_000]).unwrap();
    g.set_robot_hp(1, 0).unwrap();
    g.set_robot_hp(1, u32::MAX).unwrap();
    let mut restored: Match = serde_json::from_slice(&serde_json::to_vec(&g).unwrap()).unwrap();
    restored.validate().unwrap();
    let hit = impact(Target::Robot(1), [45_000, 44_000]);
    assert_eq!(g.armor_hit(hit.clone(), None, Caliber::Mm17), 10);
    assert_eq!(restored.armor_hit(hit, None, Caliber::Mm17), 10);
    g.advance_ns(3 * S);
    for _ in 0..3 {
        restored.advance_ns(S);
    }
    assert_eq!(g, restored);
    restored.adaptive.selected[0]
        .as_mut()
        .unwrap()
        .area_share_ppb = Some(1_000_000_001);
    assert!(restored.validate().is_err());
    restored = g.clone();
    restored.adaptive.impacts.clear();
    assert!(restored.validate().is_err());
    restored = g.clone();
    restored.adaptive.opportunities[1] = 100;
    assert!(restored.validate().is_err());
    restored = g.clone();
    restored.adaptive.selected[0].as_mut().unwrap().selected_ns = 64 * S;
    assert!(restored.validate().is_err());
    g.start(1);
    assert_eq!(g.adaptive.selected, [None, None]);
    assert_eq!(g.adaptive.selections, [0; 2]);
    assert!(g.adaptive.impacts.is_empty());
}

#[test]
fn transferred_outpost_uses_new_owners_region_and_base_uses_its_own() {
    let mut g = ready();
    g.select_reinforcement(Team::Red, [0, 0]).unwrap();
    g.select_reinforcement(Team::Blue, [45_000, 44_000])
        .unwrap();
    for _ in 0..5 {
        g.observe(
            2,
            Team::Red,
            rm_simulator_gameplay::rmuc_2027::Point::CentralHighland,
            true,
        );
        g.advance_ns(S);
    }
    g.set_outpost_hp(Team::Red, 0);
    assert_eq!(g.control.snapshot().outposts[0].owner, Team::Blue);
    let p = [45_000, 44_000];
    assert_eq!(
        g.armor_hit(impact(Target::Outpost(Team::Red), p), None, Caliber::Mm17),
        10
    );
    assert_eq!(g.adaptive.impacts.last().unwrap().victim, Team::Blue);
    assert_eq!(
        g.adaptive.impacts.last().unwrap().region,
        Region::Reinforced
    );
    assert_eq!(
        g.armor_hit(impact(Target::Base(Team::Red), p), None, Caliber::Mm17),
        30
    );
    assert_eq!(
        g.adaptive.impacts.last().unwrap().region,
        Region::Unreinforced
    );
    g.validate().unwrap();
}

#[test]
fn singular_statistics_freeze_default_and_missing_coordinates_do_not_invent_regions() {
    for mode in 0..3 {
        let mut g = ready();
        match mode {
            0 => g.adaptive.launches[0] = 0,
            1 => g
                .adaptive
                .impacts
                .retain(|h| h.attacker != Some(Team::Blue)),
            _ => {
                for h in &mut g.adaptive.impacts {
                    h.position_um = [20_000; 2];
                }
            }
        }
        g.select_reinforcement(Team::Red, [45_000, 44_000]).unwrap();
        assert_eq!(
            g.adaptive.selected[0].as_ref().unwrap().area_share_ppb,
            None
        );
        assert_eq!(
            g.armor_hit(
                impact(Target::Outpost(Team::Red), [0, 0]),
                None,
                Caliber::Mm17
            ),
            20
        );
        g.validate().unwrap();
    }
    let mut g = ready();
    g.select_reinforcement(Team::Red, [45_000, 44_000]).unwrap();
    // The legacy coordinate-free API explicitly retains default damage.
    assert_eq!(g.hit(Target::Outpost(Team::Red), None, Caliber::Mm17), 20);
    let before = g.clone();
    assert_eq!(
        g.armor_hit(
            impact(Target::Outpost(Team::Red), [90_001, 0]),
            None,
            Caliber::Mm17
        ),
        0
    );
    assert_eq!(g, before);
}

#[test]
fn rune_attack_and_defense_multiply_the_adaptive_raw_damage() {
    let mut g = ready();
    g.select_reinforcement(Team::Red, [45_000, 44_000]).unwrap();
    g.advance_ns(120 * S);
    for team in Team::BOTH {
        g.activate_rune(team).unwrap();
        for _ in 0..10 {
            let blade = g.runes[team.index()]
                .targets
                .iter()
                .position(|&v| v)
                .unwrap() as u8;
            g.hit_rune(team, blade, 5);
        }
    }
    assert_eq!(g.runes[1].buff.as_ref().unwrap().attack_pct, 150);
    assert_eq!(g.field_effects(1).defense_pct, 25);
    // Outside: 20 * 1.5 armor * 1.5 attack * 0.75 defense = 33.75 -> 34.
    assert_eq!(
        g.armor_hit(
            impact(Target::Robot(1), [90_000, 88_000]),
            Some(2),
            Caliber::Mm17
        ),
        34
    );
    assert_eq!(
        g.armor_hit(
            impact(Target::Outpost(Team::Red), [90_000, 88_000]),
            Some(2),
            Caliber::Mm17
        ),
        34
    );
    // Inside: 20 * 0.5 * 1.5 * 0.75 = 11.25 -> 11.
    assert_eq!(
        g.armor_hit(
            impact(Target::Robot(1), [45_000, 44_000]),
            Some(2),
            Caliber::Mm17
        ),
        11
    );
    g.validate().unwrap();
}
