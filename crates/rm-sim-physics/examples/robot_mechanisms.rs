// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Deterministic mechanism runner: `robot_mechanisms hero|stairs flat|steps commands.csv [spawn_x]`.
//! Commands are tick,forward_m_s,deployed (0/1); output is CSV on stdout.
use rm_sim_physics::{
    Pose, TargetFrames, Team, WorldPhysics,
    chassis::{ChassisCommand, ChassisConfig},
    tick_ns,
};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let cfg = match args.first().map(String::as_str) {
        Some("hero") => ChassisConfig::folding_hero(),
        Some("stairs") => ChassisConfig::stair_heavy(),
        _ => return Err("robot must be hero or stairs".into()),
    };
    cfg.validate()?;
    let commands = std::fs::read_to_string(args.get(2).ok_or("commands CSV required")?)?;
    let mut script = Vec::new();
    for line in commands
        .lines()
        .filter(|s| !s.trim().is_empty() && !s.starts_with('#'))
    {
        let v: Vec<_> = line.split(',').collect();
        if v.len() != 3 {
            return Err("expected tick,forward_m_s,deployed".into());
        }
        let tick = v[0].parse::<u64>()?;
        let speed = v[1].parse::<f64>()?;
        let deployed = v[2].parse::<u8>()?;
        if !speed.is_finite()
            || deployed > 1
            || tick > 60_000
            || script
                .last()
                .is_some_and(|row: &(u64, f64, bool)| row.0 >= tick)
        {
            return Err(
                "commands need finite speeds, 0/1 modes and increasing ticks <= 60000".into(),
            );
        }
        script.push((tick, speed, deployed == 1));
    }
    let mut world = WorldPhysics::new(&[], 0.);
    match args.get(1).map(String::as_str) {
        Some("steps") => {
            world.add_static_box(Pose::at([0.8, 0., 0.1]), [1.6, 3., 0.2]);
            world.add_static_box(Pose::at([2.4, 0., 0.2]), [1.6, 3., 0.4]);
        }
        Some("flat") => {}
        _ => return Err("terrain must be flat or steps".into()),
    }
    let x = args
        .get(3)
        .map(|v| v.parse::<f64>())
        .transpose()?
        .unwrap_or(-0.6);
    let id = world.add_chassis(
        Team::Red,
        cfg.clone(),
        Pose::at([x, 0., cfg.rest_height_m()]),
    )?;
    let frames = TargetFrames::new(vec![]);
    print!(
        "tick,x_m,z_m,pitch_rad,input_rad,front_x_m,front_z_m,rear_x_m,rear_z_m,contacts,y_m,qw,qx,qy,qz"
    );
    for i in 0..4 {
        print!(",wheel_{i}_x,wheel_{i}_y,wheel_{i}_z,wheel_{i}_spin");
    }
    println!();
    let end = script.last().ok_or("empty command script")?.0;
    let mut cursor = 0;
    for tick in 0..=end {
        while cursor < script.len() && script[cursor].0 == tick {
            let (_, speed, folded) = script[cursor];
            world.command_chassis(
                id,
                ChassisCommand {
                    forward_m_s: speed,
                    folded,
                    ..Default::default()
                },
            )?;
            cursor += 1;
        }
        world.step(tick * tick_ns(), &frames)?;
        if tick % 4 == 0 || tick == end {
            let s = world.chassis_snapshot(id).unwrap();
            let q = s.pose.rotation_wxyz;
            let pitch = (2. * (q[0] * q[2] - q[3] * q[1])).clamp(-1., 1.).asin();
            print!(
                "{tick},{:.8},{:.8},{pitch:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{}",
                s.pose.translation_m[0],
                s.pose.translation_m[2],
                s.fold_angle_rad,
                s.wheels[0].hub_m[0],
                s.wheels[0].hub_m[2],
                s.wheels[2].hub_m[0],
                s.wheels[2].hub_m[2],
                s.wheels.iter().filter(|w| w.contact.is_some()).count()
            );
            print!(
                ",{:.8},{:.10},{:.10},{:.10},{:.10}",
                s.pose.translation_m[1], q[0], q[1], q[2], q[3]
            );
            for w in &s.wheels {
                print!(
                    ",{:.8},{:.8},{:.8},{:.8}",
                    w.hub_m[0], w.hub_m[1], w.hub_m[2], w.spin_rad
                );
            }
            println!();
        }
    }
    Ok(())
}
