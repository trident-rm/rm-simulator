// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native rules throughput, without CAD, physical motion, networking or rendering.
use rm_simulator_gameplay::{
    Caliber, Target, Team,
    rmuc_2027::{
        Point, ROUND_NS,
        match_rules::{COUNTDOWN_NS, GroundRole, Match, Phase},
        radar::Quality,
    },
};
use std::{hint::black_box, time::Instant};

fn fixture(robots: u32) -> Match {
    let mut game = Match::default();
    for id in 0..robots {
        let team = if id % 2 == 0 { Team::Red } else { Team::Blue };
        let role = match id / 2 {
            0 => GroundRole::Heavy,
            3 => GroundRole::Sentry,
            _ => GroundRole::Infantry,
        };
        game.add_robot(id, team, role);
    }
    game.start(1);
    game.advance_ns(COUNTDOWN_NS);
    for id in 0..robots {
        let side = if id % 2 == 0 { Team::Red } else { Team::Blue };
        game.observe(
            id,
            side,
            [
                Point::Outpost,
                Point::CentralHighland,
                Point::Base,
                Point::Fortress,
            ][(id / 2) as usize],
            true,
        );
    }
    game
}

fn run(mut game: Match, dt_ns: u64, mixed: bool) -> (Match, u64, u64) {
    let mut calls = 0;
    let mut positive_hits = 0;
    let mut next_input_ns = 0;
    while game.phase == Phase::Running {
        let now = game.elapsed_ns();
        if mixed && now >= next_input_ns {
            let second = now / 1_000_000_000;
            for id in 0..8 {
                let side = if id % 2 == 0 { Team::Red } else { Team::Blue };
                game.radar_report(side.other(), id, [id as i32 * 1000, 0], Quality::Accurate)
                    .unwrap();
                if game.check_launch(id).is_ok() {
                    game.record_launch(id, 25.);
                }
                if second.is_multiple_of(2) {
                    positive_hits +=
                        u64::from(game.hit(Target::Robot(id), Some(id ^ 1), Caliber::Mm17) > 0);
                }
            }
            next_input_ns = now + 100_000_000;
        }
        game.advance_ns(dt_ns.min(ROUND_NS - now));
        calls += 1;
    }
    (game, calls, positive_hits)
}

fn main() {
    const REPEATS: usize = 5;
    println!(
        "Native 2027 rules only; five repetitions, fixture/cloning/validation outside timing; round=420s"
    );
    for (name, robots, dt_ns, rounds, mixed) in [
        ("one robot / event driven", 1, ROUND_NS, 100, false),
        ("eight robots / event driven", 8, ROUND_NS, 100, false),
        ("eight robots / 128 Hz", 8, 7_812_500, 1, false),
        ("eight robots / 1000 Hz", 8, 1_000_000, 1, false),
        ("eight robots / mixed 128 Hz", 8, 7_812_500, 1, true),
    ] {
        let base = fixture(robots);
        let mut times = Vec::new();
        let mut calls = 0;
        let mut hits = 0;
        for _ in 0..REPEATS {
            let inputs: Vec<_> = (0..rounds).map(|_| base.clone()).collect();
            let start = Instant::now();
            let results: Vec<_> = inputs
                .into_iter()
                .map(|game| black_box(run(game, dt_ns, mixed)))
                .collect();
            times.push(start.elapsed().as_secs_f64());
            calls = results.iter().map(|(_, calls, _)| calls).sum::<u64>();
            hits = results.iter().map(|(_, _, hits)| hits).sum::<u64>();
            for (game, _, _) in results {
                assert_eq!(game.phase, Phase::Finished);
                assert_eq!(game.elapsed_ns(), ROUND_NS);
                game.validate().unwrap();
            }
        }
        times.sort_by(f64::total_cmp);
        let median = times[REPEATS / 2];
        println!(
            "{name}: median={:.3} ms/round; {:.1} ns/advance; {:.1}x realtime; calls/round={}; positive_hits/round={}; range={:.3}..{:.3} ms/round",
            median * 1000. / rounds as f64,
            median * 1e9 / calls as f64,
            rounds as f64 * 420. / median,
            calls / rounds,
            hits / rounds,
            times[0] * 1000. / rounds as f64,
            times[REPEATS - 1] * 1000. / rounds as f64
        );
    }
}
