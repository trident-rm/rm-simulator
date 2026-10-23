// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Drive a bare physical robot around a wall without a Field, host, network or renderer.
use rm_sim_control::navigation::{Controller, Map, Phase, RobotObservation, Surface};
use rm_sim_physics::{Pose, TargetFrames, Team, WorldPhysics, chassis::ChassisConfig, tick_ns};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = WorldPhysics::new(&[], 0.);
    world.add_static_box(Pose::at([0., 0., 0.5]), [0.03, 2.5, 1.]);
    let geometry = world.static_geometry_snapshot(true);
    let surface = Surface {
        ground: None,
        floor: 0.,
        exclusions: vec![],
        passage_hints: vec![],
    };
    let map = Map::prepare(geometry.clone(), surface.clone(), [-3., -3., 3., 3.])?;
    let mut controller = Controller::new(surface)?;
    controller.set_summary(map.summary().clone());
    let config = ChassisConfig::default();
    let id = world.add_chassis(
        Team::Red,
        config.clone(),
        Pose::at([-2., 0., config.rest_height_m()]),
    )?;
    controller.add(id);
    controller.goal(id, [2., 0.])?;
    let request = controller.request(id).unwrap();
    let generation = request.generation;
    let route = map.plan([-2., 0., 0.], request.goal_m, &[], &geometry, false)?;
    controller.submitted(id, generation);
    controller.accept_route(id, generation, Ok(route), Some([-2., 0., 0.]))?;
    let frames = TargetFrames::new(vec![]);
    let mut contacts = Vec::new();
    for tick in 0..128 * 20 {
        if tick % 8 == 0 {
            let robot = world.chassis_telemetry(id).unwrap();
            let observed = RobotObservation {
                pose: robot.pose,
                command: robot.command,
                fold_angle_rad: robot.fold_angle_rad,
                defeated: robot.defeated,
                config: &config,
            };
            if let Some(command) =
                controller.command(id, observed, &geometry, &[], 8 * tick_ns())?
            {
                // The caller may reject or modify this result; the controller cannot actuate.
                world.command_chassis(id, command)?;
            }
        }
        contacts.clear();
        world.step_into(tick * tick_ns(), &frames, &mut contacts)?;
        if controller.view(id).unwrap().phase == Phase::Arrived {
            println!(
                "Arrived after {:.3}s at {:?}",
                (tick + 1) as f64 * tick_ns() as f64 * 1e-9,
                world.chassis_control_state(id).unwrap().pose.translation_m
            );
            return Ok(());
        }
    }
    Err("navigation did not arrive within the bounded example".into())
}
