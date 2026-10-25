// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Headless voltage-controlled pendulum with optional timestamped CSV output.
use sim_pendulum_example::{Experiment, reference_rad};
use std::{io::Write, time::Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut seconds: u64 = 8;
    let mut csv_path = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--seconds" => {
                seconds = args
                    .next()
                    .ok_or("--seconds needs a positive integer")?
                    .parse()?
            }
            "--csv" => csv_path = Some(args.next().ok_or("--csv needs a path")?),
            _ => return Err("usage: sim-pendulum-example [--seconds N] [--csv PATH]".into()),
        }
    }
    let ticks = seconds
        .checked_mul(1000)
        .filter(|&n| n > 0)
        .ok_or("duration must be positive and fit the clock")?;
    ticks
        .checked_mul(1_000_000)
        .ok_or("duration must fit the clock")?;
    let mut csv = csv_path
        .map(|p| std::fs::File::create(p).map(std::io::BufWriter::new))
        .transpose()?;
    if let Some(csv) = &mut csv {
        writeln!(
            csv,
            "time_ns,encoder_time_ns,encoder_count,target_rad,voltage_v,current_a,torque_nm,truth_angle_rad,truth_velocity_rad_s"
        )?;
    }
    let setup_start = Instant::now();
    let mut experiment = Experiment::new()?;
    let setup = setup_start.elapsed();
    let start = Instant::now();
    for _ in 0..ticks {
        experiment.advance(1)?;
        if let Some(csv) = &mut csv {
            let state = experiment.truth()?;
            let telemetry = experiment.telemetry();
            let encoder = telemetry.encoder.ok_or("missing encoder")?;
            let target = reference_rad(experiment.stamp().time_ns - 1_000_000);
            writeln!(
                csv,
                "{},{},{},{target},{},{},{},{},{}",
                experiment.stamp().time_ns,
                encoder.time_ns,
                encoder.count,
                telemetry.motor.voltage_v,
                telemetry.motor.current_a,
                telemetry.motor.torque_nm,
                state.position,
                state.velocity
            )?;
        }
    }
    if let Some(csv) = &mut csv {
        csv.flush()?;
    }
    let elapsed = start.elapsed();
    let truth = experiment.truth()?;
    let telemetry = experiment.telemetry();
    let target = reference_rad(experiment.stamp().time_ns - 1_000_000);
    println!(
        "simulation_ns={} setup_ms={:.3} loop_ms={:.3} us_per_tick={:.3} csv={}",
        experiment.stamp().time_ns,
        setup.as_secs_f64() * 1e3,
        elapsed.as_secs_f64() * 1e3,
        elapsed.as_secs_f64() * 1e6 / ticks as f64,
        csv.is_some()
    );
    println!(
        "encoder_samples={} controller_updates={} target_rad={target} truth_angle_rad={} error_rad={} encoder={:?}",
        telemetry.encoder_samples,
        telemetry.controller_updates,
        truth.position,
        target - truth.position,
        telemetry.encoder
    );
    Ok(())
}
