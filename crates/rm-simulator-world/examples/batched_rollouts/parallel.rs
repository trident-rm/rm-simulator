// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Independent-environment throughput with checked equality after timed work.
use super::*;
use rm_simulator_physics::session::batch::parallel::CpuWorkers;

fn row(layer: &str, repetition: usize, workers: usize, count: usize, steps: usize, time: Duration) {
    println!(
        "{layer},{repetition},{workers},{count},{steps},{:.3},{:.3},{:.3}",
        time.as_secs_f64() * 1e6,
        time.as_secs_f64() * 1e6 / count as f64,
        count as f64 * steps as f64 / time.as_secs_f64()
    );
}

pub(super) fn run(args: &[String]) -> Result<()> {
    if args.len() > 3 {
        return Err("usage: batched_rollouts --parallel [sessions] [steps] [repetitions]".into());
    }
    let read = |index: usize, default: usize| -> Result<usize> {
        Ok(args
            .get(index)
            .map(|item| item.parse())
            .transpose()?
            .unwrap_or(default))
    };
    let count = read(0, 64)?;
    let steps = read(1, 512)?;
    let repetitions = read(2, 5)?;
    if !(1..=1024).contains(&count)
        || !(1..=100_000).contains(&steps)
        || !(1..=100).contains(&repetitions)
    {
        return Err("require sessions 1..=1024, steps 1..=100000, repetitions 1..=100".into());
    }
    let limits = BatchLimits {
        max_sessions: count,
        max_steps_per_session: steps as u64,
        max_commands_per_session: 1,
    };
    let requests: Vec<_> = (0..count)
        .map(|session| BatchStep {
            session,
            steps: steps as u64,
        })
        .collect();
    println!(
        "# 1000 Hz; one held Drive(0.5 m/s) per environment before a rollout; no CAD/sensors/GPU"
    );
    println!(
        "# Robot: default chassis, catch floor, shared triangle, default projectile policy, no authority"
    );
    println!(
        "# Match: same geometry; active native RMUL Infantry authority, countdown completed at 5 s"
    );
    println!(
        "# workers=0: sequential batch baseline; scoped workers include spawn/join and report allocation"
    );
    println!(
        "layer,repetition,workers,environments,steps_per_environment,batch_us,environment_us,environment_steps_per_second"
    );
    println!("# Each environment warms for 256 ticks before timed command application/integration");
    for repetition in 0..=repetitions {
        // Repetition zero warms every worker count; exclude it from summaries.
        let order = if repetition % 2 == 0 {
            [0, 1, 2, 4, 8]
        } else {
            [8, 4, 2, 1, 0]
        };
        let mut expected_robot = None;
        let mut expected_match = None;
        for workers in order {
            let mut batch = RobotBatch::new(robots(count)?, limits)?;
            for slot in 0..count {
                let _ = batch.session_mut(slot).unwrap().step(256)?;
                batch.commands_mut(slot).unwrap().push(command());
            }
            let mut stamps = Vec::with_capacity(count);
            let begin = Instant::now();
            let robot_report = if workers == 0 {
                batch.step(&requests, &mut stamps)?;
                None
            } else {
                Some(batch.step_parallel(CpuWorkers::new(workers)?, &requests)?)
            };
            let duration = begin.elapsed();
            if let Some(report) = robot_report {
                assert!(report.all_succeeded(), "{report:?}");
            }
            let truth: Vec<_> = batch
                .sessions()
                .iter()
                .map(|session| session.robot_truth(0))
                .collect();
            if let Some(expected) = &expected_robot {
                assert_eq!(&truth, expected);
            } else {
                expected_robot = Some(truth);
            }
            row("robot", repetition, workers, count, steps, duration);

            let mut batch = MatchBatch::new(matches(count)?, limits)?;
            for slot in 0..count {
                let _ = batch.session_mut(slot).unwrap().step(256)?;
                batch.commands_mut(slot).unwrap().push(command());
            }
            let begin = Instant::now();
            let match_report = if workers == 0 {
                batch.step(&requests, &mut stamps)?;
                None
            } else {
                Some(batch.step_parallel(CpuWorkers::new(workers)?, &requests)?)
            };
            let duration = begin.elapsed();
            if let Some(report) = match_report {
                assert!(report.all_succeeded(), "{report:?}");
            }
            let truth: Vec<_> = batch
                .sessions()
                .iter()
                .map(|session| {
                    (
                        session.robot_truth(0),
                        session.ground_truth().rmul_2027().cloned(),
                    )
                })
                .collect();
            if let Some(expected) = &expected_match {
                assert_eq!(&truth, expected);
            } else {
                expected_match = Some(truth);
            }
            row("match", repetition, workers, count, steps, duration);
        }
    }
    println!(
        "# Exact final robot truth/native authority equality passed across sequential/1/2/4/8 workers."
    );
    Ok(())
}
