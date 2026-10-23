// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! CPU rollout probe: ordinary loops versus sequential batch facades, without CAD.
//! Run in release mode; arguments are sessions, integration steps, repetitions.
use rm_sim_physics::{
    WorldPhysics,
    session::{RobotScenario, RobotSession, batch::RobotBatch},
};
use rm_sim_world::{
    ChassisConfig, ChassisPlacement, Field, FieldConfig, Pose, RobotKind, StepDuration, Team,
    session::{
        MatchCheckpoint, MatchSession, RobotCommand,
        batch::{BatchLimits, BatchStep, ControlObservation, MatchBatch, MatchReset, RobotQuery},
    },
};
use std::{
    error::Error,
    hint::black_box,
    time::{Duration, Instant},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn command() -> RobotCommand {
    RobotCommand::Drive {
        robot: 0,
        command: rm_sim_world::ChassisCommand {
            forward_m_s: 0.5,
            ..Default::default()
        },
    }
}
fn elapsed(mut run: impl FnMut() -> Result<()>) -> Result<Duration> {
    let begin = Instant::now();
    run()?;
    Ok(begin.elapsed())
}
fn report(layer: &str, operation: &str, sequential: Duration, batch: Duration, count: usize) {
    println!(
        "{layer},{operation},{count},{:.3},{:.3},{:.3}",
        sequential.as_secs_f64() * 1e6 / count as f64,
        batch.as_secs_f64() * 1e6 / count as f64,
        sequential.as_secs_f64() / batch.as_secs_f64()
    );
}

fn robots(count: usize) -> Result<Vec<RobotSession>> {
    // Cook once and share the immutable collider geometry through every recipe.
    let mut source = WorldPhysics::new(&[], 0.);
    source.add_static_mesh(
        vec![[10., 0., 0.], [11., 0., 0.], [10., 1., 0.]],
        vec![[0, 1, 2]],
    )?;
    let geometry = source.static_geometry_snapshot(false);
    (0..count)
        .map(|_| {
            let geometry = geometry.clone();
            Ok(RobotSession::new(RobotScenario::new(
                vec![],
                0.,
                StepDuration::HZ_1000,
                move |world| {
                    world.insert_geometry(&geometry);
                    world.add_chassis(
                        Team::Red,
                        ChassisConfig::default(),
                        Pose::at([0., 0., 0.4]),
                    )?;
                    Ok(())
                },
            )?)?)
        })
        .collect()
}
fn matches(count: usize) -> Result<Vec<MatchSession>> {
    let mut field = Field::new(&FieldConfig {
        step_duration: StepDuration::HZ_1000,
        rmul_2027: Some(rm_sim_world::gameplay::rmul_2027::Format::Infantry),
        chassis: vec![ChassisPlacement {
            config: ChassisConfig::default(),
            spawn: Pose::at([0., 0., 0.4]),
            team: Team::Red,
            kind: RobotKind::Infantry,
            performance: None,
        }],
        ..Default::default()
    })?;
    field.add_static_mesh(
        vec![[10., 0., 0.], [11., 0., 0.], [10., 1., 0.]],
        vec![[0, 1, 2]],
    )?;
    field.referee_command(rm_sim_world::RefereeCommand::StartMatch)?;
    let countdown_ns = field.rmul_2027().unwrap().countdown_remaining_ns;
    assert_eq!(countdown_ns % field.step_duration().as_ns(), 0);
    field.step(countdown_ns / field.step_duration().as_ns())?;
    assert_eq!(
        field.rmul_2027().unwrap().phase,
        rm_sim_world::gameplay::rmul_2027::Phase::Running
    );
    // Reset/checkpoint measurements deliberately retain this nonzero initial clock.
    let recipe = MatchCheckpoint::capture(&field);
    (0..count)
        .map(|_| Ok(MatchSession::new(recipe.restore()?)))
        .collect()
}

fn robot_probe(count: usize, steps: usize, limits: BatchLimits, reverse: bool) -> Result<()> {
    let mut sequential = robots(count)?;
    let mut batch = RobotBatch::new(robots(count)?, limits)?;
    let requests: Vec<_> = (0..count)
        .map(|session| BatchStep { session, steps: 1 })
        .collect();
    let queries: Vec<_> = (0..count)
        .map(|session| RobotQuery { session, robot: 0 })
        .collect();
    let indices: Vec<_> = (0..count).collect();
    let mut stamps = Vec::with_capacity(count);
    let mut contacts: Vec<Vec<_>> = (0..count).map(|_| Vec::new()).collect();
    let mut sequential_stamps = Vec::with_capacity(count);
    let mut seq_step = || {
        elapsed(|| {
            for _ in 0..steps {
                sequential_stamps.clear();
                for (session, contacts) in sequential.iter_mut().zip(&mut contacts) {
                    session.command(command())?;
                    contacts.clear();
                    session.step_into(1, contacts)?;
                    sequential_stamps.push(session.stamp());
                }
            }
            Ok(())
        })
    };
    let mut batch_step = || {
        elapsed(|| {
            for _ in 0..steps {
                for index in 0..count {
                    batch.commands_mut(index).unwrap().push(command());
                }
                batch.step(&requests, &mut stamps)?;
            }
            Ok(())
        })
    };
    let (seq, bat) = if reverse {
        let bat = batch_step()?;
        (seq_step()?, bat)
    } else {
        let seq = seq_step()?;
        (seq, batch_step()?)
    };
    report("robot", "command+step", seq, bat, count * steps);
    for (left, right) in sequential.iter().zip(batch.sessions()) {
        assert_eq!(left.robot_truth(0), right.robot_truth(0));
    }
    let mut observed = Vec::with_capacity(count);
    let seq = elapsed(|| {
        for _ in 0..steps {
            observed.clear();
            observed.extend(sequential.iter().enumerate().map(|(session, item)| {
                ControlObservation {
                    session,
                    robot: 0,
                    stamp: item.stamp(),
                    step_duration: item.step_duration(),
                    state: item.robot_control_state(0),
                }
            }));
            black_box(&observed);
        }
        Ok(())
    })?;
    let bat = elapsed(|| {
        for _ in 0..steps {
            batch.observe_into(&queries, &mut observed)?;
            black_box(&observed);
        }
        Ok(())
    })?;
    report("robot", "observation", seq, bat, count * steps);
    let seq = elapsed(|| {
        for session in &mut sequential {
            session.reset()?;
        }
        Ok(())
    })?;
    let bat = elapsed(|| {
        batch.reset(&indices)?;
        Ok(())
    })?;
    report("robot", "reset", seq, bat, count);
    Ok(())
}

fn match_probe(count: usize, steps: usize, limits: BatchLimits, reverse: bool) -> Result<()> {
    let mut sequential = matches(count)?;
    let mut batch = MatchBatch::new(matches(count)?, limits)?;
    let requests: Vec<_> = (0..count)
        .map(|session| BatchStep { session, steps: 1 })
        .collect();
    let queries: Vec<_> = (0..count)
        .map(|session| RobotQuery { session, robot: 0 })
        .collect();
    let resets: Vec<_> = (0..count)
        .map(|session| MatchReset {
            session,
            seed: session as u64,
        })
        .collect();
    let indices: Vec<_> = (0..count).collect();
    let mut stamps = Vec::with_capacity(count);
    let mut sequential_stamps = Vec::with_capacity(count);
    let mut seq_step = || {
        elapsed(|| {
            for _ in 0..steps {
                sequential_stamps.clear();
                for session in &mut sequential {
                    session.command(command())?;
                    sequential_stamps.push(session.step(1)?);
                }
            }
            Ok(())
        })
    };
    let mut batch_step = || {
        elapsed(|| {
            for _ in 0..steps {
                for index in 0..count {
                    batch.commands_mut(index).unwrap().push(command());
                }
                batch.step(&requests, &mut stamps)?;
            }
            Ok(())
        })
    };
    let (seq, bat) = if reverse {
        let bat = batch_step()?;
        (seq_step()?, bat)
    } else {
        let seq = seq_step()?;
        (seq, batch_step()?)
    };
    report("match", "command+step", seq, bat, count * steps);
    for (left, right) in sequential.iter().zip(batch.sessions()) {
        assert_eq!(left.robot_truth(0), right.robot_truth(0));
        assert_eq!(
            left.ground_truth().rmul_2027(),
            right.ground_truth().rmul_2027()
        );
    }
    let mut observed = Vec::with_capacity(count);
    let seq = elapsed(|| {
        for _ in 0..steps {
            observed.clear();
            observed.extend(sequential.iter().enumerate().map(|(session, item)| {
                ControlObservation {
                    session,
                    robot: 0,
                    stamp: item.stamp(),
                    step_duration: item.ground_truth().step_duration(),
                    state: item.robot_control_state(0),
                }
            }));
            black_box(&observed);
        }
        Ok(())
    })?;
    let bat = elapsed(|| {
        for _ in 0..steps {
            batch.observe_into(&queries, &mut observed)?;
            black_box(&observed);
        }
        Ok(())
    })?;
    report("match", "observation", seq, bat, count * steps);
    let mut checkpoints = Vec::with_capacity(count);
    let seq = elapsed(|| {
        for _ in 0..steps {
            checkpoints.clear();
            checkpoints.extend(sequential.iter().map(MatchSession::checkpoint));
            black_box(&checkpoints);
        }
        Ok(())
    })?;
    let bat = elapsed(|| {
        for _ in 0..steps {
            batch.checkpoint_into(&indices, &mut checkpoints)?;
            black_box(&checkpoints);
        }
        Ok(())
    })?;
    report("match", "checkpoint", seq, bat, count * steps);
    let seq = elapsed(|| {
        for (session, checkpoint) in sequential.iter_mut().zip(&checkpoints) {
            session.restore(checkpoint)?;
        }
        Ok(())
    })?;
    let bat = elapsed(|| {
        for (index, checkpoint) in checkpoints.iter().enumerate() {
            batch.session_mut(index).unwrap().restore(checkpoint)?;
        }
        Ok(())
    })?;
    report("match", "checkpoint-restore", seq, bat, count);
    let seq = elapsed(|| {
        for (index, session) in sequential.iter_mut().enumerate() {
            session.reset(index as u64)?;
        }
        Ok(())
    })?;
    let bat = elapsed(|| {
        batch.reset(&resets)?;
        Ok(())
    })?;
    report("match", "reset", seq, bat, count);
    Ok(())
}

#[path = "batched_rollouts/parallel.rs"]
mod parallel;

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.first().is_some_and(|value| value == "--parallel") {
        return parallel::run(&args[1..]);
    }
    if args.len() > 3 {
        return Err("usage: batched_rollouts [sessions] [steps] [repetitions]".into());
    }
    let read = |index: usize, default: usize| -> Result<usize> {
        Ok(args
            .get(index)
            .map(|value| value.parse())
            .transpose()?
            .unwrap_or(default))
    };
    let count = read(0, 16)?;
    let steps = read(1, 128)?;
    let repetitions = read(2, 5)?;
    if count == 0
        || steps == 0
        || repetitions == 0
        || count > 1024
        || steps > 100_000
        || repetitions > 100
    {
        return Err("require sessions 1..=1024, steps 1..=100000, repetitions 1..=100".into());
    }
    let limits = BatchLimits {
        max_sessions: count,
        max_steps_per_session: steps as u64,
        max_commands_per_session: 1,
    };
    // Warm the same code paths before measurement; constructors are outside timers.
    robot_probe(1, 4, limits, false)?;
    match_probe(1, 4, limits, false)?;
    println!(
        "# {count} sessions, {steps} x 1 ms steps, {repetitions} repetitions; sequential CPU, no CAD; warmup above"
    );
    println!(
        "layer,operation,session_operations,sequential_us_per_op,batch_us_per_op,sequential_over_batch"
    );
    for repetition in 0..repetitions {
        println!("# repetition {repetition}");
        robot_probe(count, steps, limits, repetition % 2 == 1)?;
        match_probe(count, steps, limits, repetition % 2 == 1)?;
    }
    println!(
        "# Active RMUL Infantry authority; match start/reset clock: tick 5000, 5000000000 ns."
    );
    println!(
        "# Times include wall-clock contention; reset batches retain all replacements until commit."
    );
    println!(
        "# Bare RobotSession has no arbitrary running checkpoint; that cost is intentionally unavailable."
    );
    Ok(())
}
