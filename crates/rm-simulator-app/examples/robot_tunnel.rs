// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Contact-driven source profile test: `robot_tunnel chassis.json stowed|open [roof_m] [record.jsonl]`.
//! Roof spans x=0..2 m. Prints samples and fails if the stowed robot cannot exit.
use rm_simulator_physics::{
    Pose, TargetFrames, Team, WorldPhysics,
    chassis::{ChassisCommand, ChassisConfig},
    tick_ns,
};
use std::io::Write;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args
        .first()
        .is_some_and(|s| s == "export-base" || s == "export-hero")
    {
        std::fs::write(
            args.get(1).ok_or("output JSON path required")?,
            serde_json::to_vec_pretty(&if args[0] == "export-hero" {
                ChassisConfig::folding_hero()
            } else {
                ChassisConfig::engineer()
            })?,
        )?;
        return Ok(());
    }
    let cfg: ChassisConfig = serde_json::from_slice(&std::fs::read(
        args.first().ok_or("chassis.json required")?,
    )?)?;
    cfg.validate()?;
    let stowed = match args.get(1).map(String::as_str) {
        Some("stowed") => true,
        Some("open") => false,
        _ => return Err("stowed or open required".into()),
    };
    let roof = args
        .get(2)
        .map(|s| s.parse::<f64>())
        .transpose()?
        .unwrap_or(0.25);
    if !roof.is_finite() || roof <= 0. {
        return Err("positive roof height required".into());
    }
    let mut record = args.get(3).map(std::fs::File::create).transpose()?;
    if let Some(file) = &mut record {
        writeln!(
            file,
            "{}",
            serde_json::json!({"type":"header","config":cfg,"roof_m":roof,"reference_frames":source_frames(&cfg,0.,[0.,0.])})
        )?;
    }
    let mut w = WorldPhysics::new(&[], 0.);
    w.add_static_box(Pose::at([1., 0., roof + 0.05]), [2., 2., 0.1]);
    let id = w.add_chassis(
        Team::Red,
        cfg.clone(),
        Pose::at([-0.7, 0., cfg.rest_height_m()]),
    )?;
    let frames = TargetFrames::new(vec![]);
    let mut restored = false;
    let check_stow = stowed && cfg.articulation.as_ref().is_some_and(|a| a.limit_rad > 0.);
    let mut blocked_unfold = false;
    for tick in 0..=1800 {
        if tick == 0 || tick == 384 {
            w.command_chassis(
                id,
                ChassisCommand {
                    folded: stowed,
                    forward_m_s: if tick == 0 { 0. } else { 0.5 },
                    ..Default::default()
                },
            )?;
        }
        w.step(tick * tick_ns(), &frames)?;
        if check_stow && tick == 700 {
            w.command_chassis(id, ChassisCommand::default())?;
        }
        if check_stow && tick == 950 {
            let angle = w.chassis_snapshot(id).unwrap().fold_angle_rad;
            assert!(
                angle > 0.01 && angle < cfg.transform_limit_rad(),
                "roof must block the opening arm: {angle}"
            );
            blocked_unfold = true;
        }
        if check_stow && tick == 1000 {
            w.command_chassis(
                id,
                ChassisCommand {
                    folded: true,
                    forward_m_s: 0.5,
                    ..Default::default()
                },
            )?;
        }
        if tick == 96 {
            let s = w.chassis_snapshot(id).unwrap();
            let mut copy = WorldPhysics::new(&[], 0.);
            copy.add_static_box(Pose::at([1., 0., roof + 0.05]), [2., 2., 0.1]);
            copy.restore_chassis(&s)?;
            assert_eq!(
                copy.chassis_snapshot(id).unwrap().fold_angle_rad,
                s.fold_angle_rad
            );
            w = copy;
            restored = true;
        }
        if tick % 4 == 0
            && let Some(file) = &mut record
        {
            let s = w.chassis_snapshot(id).unwrap();
            let [w, x, y, z] = s.pose.rotation_wxyz;
            let direction =
                bevy::math::DQuat::from_xyzw(x, y, z, w).inverse() * bevy::math::DVec3::X;
            let aim = [
                direction.y.atan2(direction.x),
                direction.z.clamp(-1., 1.).asin(),
            ];
            writeln!(
                file,
                "{}",
                serde_json::json!({"tick":tick,"pose":s.pose,"wheels":s.wheels,"input_rad":s.fold_angle_rad,"source_frames":source_frames(&cfg,s.fold_angle_rad,aim)})
            )?;
        }
        if tick % 128 == 0 {
            let s = w.chassis_snapshot(id).unwrap();
            println!(
                "{tick},{:0.5},{:0.5},{:0.5}",
                s.pose.translation_m[0], s.pose.translation_m[2], s.fold_angle_rad
            );
        }
    }
    let s = w.chassis_snapshot(id).unwrap();
    let passed = s.pose.translation_m[0] > 2.5;
    println!(
        "{}",
        serde_json::json!({"passed":passed,"stowed":stowed,"roof_m":roof,"restored_mid_transform":restored,"blocked_unfold":blocked_unfold,"final_x_m":s.pose.translation_m[0],"final_z_m":s.pose.translation_m[2],"input_rad":s.fold_angle_rad})
    );
    if stowed && !passed {
        return Err("stowed robot did not exit the tunnel".into());
    }
    if !stowed && cfg.transform_limit_rad() > 0. && passed {
        return Err("open robot unexpectedly passed the low roof".into());
    }
    Ok(())
}

fn source_frames(cfg: &ChassisConfig, angle: f64, aim: [f64; 2]) -> Vec<(String, Pose)> {
    if let Some(a) = &cfg.articulation {
        a.joints
            .iter()
            .zip(a.poses(angle, aim))
            .map(|(j, p)| (j.name.clone(), p))
            .collect()
    } else if let Some(f) = &cfg.folding {
        let p = f.poses(Pose::default(), aim, angle);
        vec![
            ("body".into(), Pose::default()),
            ("yaw".into(), p.yaw),
            ("fold".into(), p.arm),
            ("pitch".into(), p.head),
        ]
    } else {
        vec![]
    }
}
