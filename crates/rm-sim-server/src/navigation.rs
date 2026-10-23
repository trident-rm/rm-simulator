// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Bounded host navigation worker and authoritative Field adapter.
use crate::collision_mesh::GroundMesh;
pub(crate) use rm_sim_control::navigation::tunnel_capable;
pub use rm_sim_control::navigation::{Bake, CELL_M, Phase, RADIUS_M, View};
use rm_sim_control::navigation::{
    Controller, Exclusion, Map, Obstacle, RobotObservation, Route, Surface,
};
use rm_sim_world::{Field, StaticGeometry};
use std::sync::mpsc;
pub(crate) fn brake(field: &mut Field, id: u32) -> Result<(), String> {
    let mut command = field
        .chassis_control_state(id)
        .ok_or("missing chassis")?
        .command;
    command.forward_m_s = 0.;
    command.left_m_s = 0.;
    command.yaw_rate_rad_s = 0.;
    field
        .command_chassis(id, command)
        .map_err(|e| e.to_string())
}
pub(crate) fn rotation_clear(field: &Field, id: u32, geometry: &StaticGeometry) -> bool {
    let Some(body) = field.chassis_control_state(id) else {
        return false;
    };
    let Some(config) = field.chassis_config(id) else {
        return false;
    };
    rm_sim_control::navigation::rotation_clear(body.pose, config, geometry)
}
/// Approximate Road card envelope from the registered 2027 diagram, with 15 cm
/// survey allowance. This is a navigation exclusion, never a referee zone.
pub(crate) fn road_exclusions() -> Vec<Exclusion> {
    use rm_sim_world::Team;
    let areas = crate::zone_catalog::areas();
    [Team::Red, Team::Blue]
        .into_iter()
        .map(|side| {
            let mut region = Exclusion {
                min: [f64::INFINITY; 2],
                max: [f64::NEG_INFINITY; 2],
            };
            for p in areas
                .iter()
                .filter(|a| {
                    a.side == side && (a.id.ends_with("road-low") || a.id.ends_with("road-high"))
                })
                .flat_map(|a| &a.polygon_m)
            {
                for (i, value) in p.iter().enumerate() {
                    region.min[i] = region.min[i].min(*value - 0.15);
                    region.max[i] = region.max[i].max(*value + 0.15);
                }
            }
            region
        })
        .collect()
}
fn obstacles(field: &Field, own: u32) -> Vec<Obstacle> {
    field
        .chassis_snapshots()
        .iter()
        .filter(|c| c.id != own)
        .map(|c| Obstacle {
            point: c.pose.translation_m,
            radius: 0.42,
        })
        .collect()
}
struct Job {
    id: u32,
    generation: u64,
    start: [f64; 3],
    goal: [f64; 2],
    obstacles: Vec<Obstacle>,
    live: StaticGeometry,
    allow_tunnel: bool,
}
enum Reply {
    Baked(Bake),
    Route(u32, u64, Result<Route, String>),
}
/// One bounded worker; search/controller state is implemented in rm-sim-control.
pub(crate) struct Service {
    sender: mpsc::SyncSender<Job>,
    receiver: mpsc::Receiver<Reply>,
    controller: Controller,
    ids: Vec<u32>,
}
impl Service {
    pub(crate) fn new(
        geometry: StaticGeometry,
        ground: Option<GroundMesh>,
        floor: f64,
        bounds: [f64; 4],
        exclusions: Vec<Exclusion>,
    ) -> Result<Self, String> {
        let surface = Surface {
            ground: ground.map(std::sync::Arc::new),
            floor,
            exclusions,
            passage_hints: rm_sim_control::navigation::rmuc_2027_passage_hints(),
        };
        Map::validate_inputs(&surface, bounds)?;
        let worker_surface = surface.clone();
        let (sender, jobs) = mpsc::sync_channel::<Job>(1);
        let (replies, receiver) = mpsc::sync_channel(4);
        std::thread::Builder::new()
            .name("rm-navigation".into())
            .spawn(move || {
                let map = Map::prepare(geometry, worker_surface, bounds)
                    .expect("admitted navigation bake");
                if replies.send(Reply::Baked(map.summary().clone())).is_err() {
                    return;
                }
                while let Ok(job) = jobs.recv() {
                    let path = map.plan(
                        job.start,
                        job.goal,
                        &job.obstacles,
                        &job.live,
                        job.allow_tunnel,
                    );
                    if replies
                        .send(Reply::Route(job.id, job.generation, path))
                        .is_err()
                    {
                        break;
                    }
                }
            })
            .map_err(|e| e.to_string())?;
        Ok(Self {
            sender,
            receiver,
            controller: Controller::new(surface)?,
            ids: Vec::new(),
        })
    }
    pub(crate) fn add(&mut self, id: u32) {
        self.controller.add(id);
    }
    pub(crate) fn owns(&self, id: u32) -> bool {
        self.controller.owns(id)
    }
    pub(crate) fn remove(&mut self, id: u32) {
        self.controller.remove(id);
    }
    pub(crate) fn stop(&mut self, id: u32) {
        self.controller.stop(id);
    }
    pub(crate) fn goal(&mut self, id: u32, goal: [f64; 2]) -> Result<(), String> {
        self.controller.goal(id, goal)
    }
    pub(crate) fn views(&self) -> Vec<View> {
        self.controller.views()
    }
    pub(crate) fn poll(&mut self, field: &Field) {
        while let Ok(reply) = self.receiver.try_recv() {
            match reply {
                Reply::Baked(summary) => self.controller.set_summary(summary),
                Reply::Route(id, generation, path) => {
                    let position = field
                        .chassis_control_state(id)
                        .map(|b| b.pose.translation_m);
                    let _ = self.controller.accept_route(id, generation, path, position);
                }
            }
        }
        self.controller.ids_into(&mut self.ids);
        for &id in &self.ids {
            let Some(request) = self.controller.request(id) else {
                continue;
            };
            let generation = request.generation;
            let Some(body) = field.chassis_control_state(id) else {
                continue;
            };
            let p = body.pose.translation_m;
            let Some(z) = self.controller.surface().height(p[0], p[1], p[2] + 0.05) else {
                self.controller.submission_failed(
                    id,
                    generation,
                    Phase::Unreachable,
                    "No support beneath Sentry",
                );
                continue;
            };
            let job = Job {
                id,
                generation,
                start: [p[0], p[1], z],
                goal: request.goal_m,
                obstacles: obstacles(field, id)
                    .into_iter()
                    .chain(request.failed.iter().cloned())
                    .collect(),
                live: field.static_geometry_snapshot(),
                allow_tunnel: field.chassis_config(id).is_some_and(tunnel_capable),
            };
            match self.sender.try_send(job) {
                Ok(()) => self.controller.submitted(id, generation),
                Err(mpsc::TrySendError::Full(_)) => break,
                Err(_) => {
                    self.controller.submission_failed(
                        id,
                        generation,
                        Phase::Blocked,
                        "Navigation worker stopped",
                    );
                }
            }
        }
    }
    pub(crate) fn tick(&mut self, field: &mut Field) {
        if !field.tick().is_multiple_of(8) {
            return;
        }
        let live = field.static_geometry_snapshot();
        self.controller.ids_into(&mut self.ids);
        for &id in &self.ids {
            if self
                .controller
                .view(id)
                .is_some_and(|v| v.phase == Phase::Idle)
            {
                continue;
            }
            let Some(body) = field.chassis_telemetry(id) else {
                continue;
            };
            let Some(config) = field.chassis_config(id) else {
                continue;
            };
            let observation = RobotObservation {
                pose: body.pose,
                command: body.command,
                fold_angle_rad: body.fold_angle_rad,
                defeated: body.defeated,
                config,
            };
            let obstacles = if self
                .controller
                .view(id)
                .is_some_and(|v| v.phase == Phase::Following)
            {
                obstacles(field, id)
            } else {
                Vec::new()
            };
            let command = self.controller.command(
                id,
                observation,
                &live,
                &obstacles,
                field.step_duration().as_ns() * 8,
            );
            if let Ok(Some(command)) = command {
                let _ = field.command_chassis(id, command);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
        (a[0] - b[0]).hypot(a[1] - b[1])
    }
    use rm_sim_world::{
        ChassisConfig, ChassisPlacement, FieldConfig, Pose, RobotKind, Team, tick_ns,
    };
    fn wall() -> StaticGeometry {
        let mut physics = rm_sim_world::projectile::WorldPhysics::new(&[], 0.);
        physics.add_static_box(Pose::at([0., 0., 0.5]), [0.03, 2.5, 1.0]);
        physics.static_geometry_snapshot(true)
    }
    #[test]
    fn road_policy_regions_are_finite_and_ordered() {
        let roads = road_exclusions();
        assert_eq!(roads.len(), 2);
        assert!(
            roads
                .iter()
                .all(|r| (0..2).all(|i| r.min[i].is_finite() && r.min[i] < r.max[i]))
        );
    }
    #[test]
    fn preparing_does_not_replan_against_a_future_translation() {
        let mut field = Field::new(&FieldConfig {
            runes: vec![],
            outposts: vec![],
            ..Default::default()
        })
        .unwrap();
        let mut physics = rm_sim_world::projectile::WorldPhysics::new(&[], 0.);
        physics.add_static_box(Pose::at([0.7, 0., 0.5]), [0.05, 4., 1.]);
        let (v, t) = physics.static_geometry_snapshot(true).into_triangles();
        field.add_static_mesh(v, t).unwrap();
        let config = ChassisConfig::folding_hero();
        let id = field
            .add_chassis(&ChassisPlacement {
                spawn: Pose {
                    translation_m: [0., 0., config.rest_height_m()],
                    rotation_wxyz: crate::math::quat_axis_angle(
                        [0., 0., 1.],
                        std::f64::consts::FRAC_PI_2,
                    ),
                },
                config,
                team: Team::Red,
                kind: RobotKind::Sentry,
                performance: None,
            })
            .unwrap();
        field.step(128).unwrap();
        let mut service = Service::new(
            StaticGeometry::default(),
            None,
            0.,
            [-3., -3., 3., 3.],
            vec![],
        )
        .unwrap();
        service.add(id);
        service.goal(id, [2., 0.]).unwrap();
        let generation = service.controller.request(id).unwrap().generation;
        service
            .controller
            .accept_route(
                id,
                generation,
                Ok(Route {
                    points: vec![[0., 0., 0.], [2., 0., 0.]],
                    tunnels: vec![false, true],
                }),
                Some([0.; 3]),
            )
            .unwrap();
        for _ in 0..64 {
            service.tick(&mut field);
            field.step(1).unwrap();
            assert_eq!(service.views()[0].phase, Phase::Following);
            assert_eq!(service.views()[0].phase, Phase::Following);
            let command = field.chassis_snapshot(id).unwrap().command;
            assert_eq!([command.forward_m_s, command.left_m_s], [0., 0.]);
        }
        assert!(field.chassis_snapshot(id).unwrap().command.folded);
    }
    #[test]
    fn tunnel_follower_stows_aligns_crosses_and_reopens_with_actual_physics() {
        let mut field = Field::new(&FieldConfig {
            runes: vec![],
            outposts: vec![],
            ..Default::default()
        })
        .unwrap();
        let mut physics = rm_sim_world::projectile::WorldPhysics::new(&[], 0.);
        physics.add_static_box(Pose::at([0., 0., 0.4]), [1., 8., 0.3]);
        let geometry = physics.static_geometry_snapshot(true);
        let (v, t) = geometry.clone().into_triangles();
        field.add_static_mesh(v, t).unwrap();
        let config = ChassisConfig::folding_hero();
        let id = field
            .add_chassis(&ChassisPlacement {
                spawn: Pose {
                    translation_m: [-2.5, 0., config.rest_height_m()],
                    rotation_wxyz: crate::math::quat_axis_angle(
                        [0., 0., 1.],
                        std::f64::consts::FRAC_PI_2,
                    ),
                },
                config,
                team: Team::Red,
                kind: RobotKind::Sentry,
                performance: None,
            })
            .unwrap();
        field.step(128).unwrap();
        let map = Map::prepare(
            geometry.clone(),
            Surface {
                ground: None,
                floor: 0.,
                exclusions: vec![],
                passage_hints: rm_sim_control::navigation::rmuc_2027_passage_hints(),
            },
            [-3., -2., 3., 2.],
        )
        .unwrap();
        let route = map
            .plan([-2.5, 0., 0.], [2.5, 0.], &[], &geometry, true)
            .unwrap();
        let mut service = Service::new(
            StaticGeometry::default(),
            None,
            0.,
            [-3., -2., 3., 2.],
            vec![],
        )
        .unwrap();
        service.add(id);
        service.goal(id, [2.5, 0.]).unwrap();
        let generation = service.controller.request(id).unwrap().generation;
        service
            .controller
            .accept_route(id, generation, Ok(route), Some([-2.5, 0., 0.]))
            .unwrap();
        let mut prepared = false;
        let mut crossed = false;
        for _ in 0..128 * 45 {
            service.tick(&mut field);
            field.step(1).unwrap();
            let body = field.chassis_snapshot(id).unwrap();
            if service.views()[0].message == "Stowing and aligning before tunnel" {
                prepared = true;
                assert!(body.command.forward_m_s.hypot(body.command.left_m_s) <= 0.17);
            }
            if !crossed && body.pose.translation_m[0].abs() < 0.3 {
                crossed = true;
                assert!(body.command.folded);
                assert!((body.fold_angle_rad - body.config.transform_limit_rad()).abs() < 0.001);
                // Braking, including an operator stop, preserves safe stow.
                brake(&mut field, id).unwrap();
                assert!(field.chassis_snapshot(id).unwrap().command.folded);
            }
            if service.views()[0].phase != Phase::Following {
                break;
            }
        }
        assert!(
            prepared && crossed,
            "{:?}, {:?}",
            service.views(),
            field.chassis_snapshot(id).unwrap().pose
        );
        assert_eq!(
            service.views()[0].phase,
            Phase::Arrived,
            "{:?}",
            service.views()
        );
        field.step(512).unwrap();
        assert!(field.chassis_snapshot(id).unwrap().fold_angle_rad < 0.001);
    }
    #[test]
    fn follower_drives_actual_chassis_and_replans_for_new_obstacles() {
        let mut field = Field::new(&FieldConfig {
            runes: vec![],
            outposts: vec![],
            ..Default::default()
        })
        .unwrap();
        let geometry = wall();
        let (vertices, triangles) = geometry.clone().into_triangles();
        field.add_static_mesh(vertices, triangles).unwrap();
        let config = ChassisConfig::default();
        let id = field
            .add_chassis(&ChassisPlacement {
                config: config.clone(),
                spawn: Pose::at([-2., 0., config.rest_height_m()]),
                team: Team::Red,
                kind: RobotKind::Sentry,
                performance: None,
            })
            .unwrap();
        field.step(128).unwrap();
        let mut service = Service::new(geometry, None, 0., [-3., -3., 3., 3.], vec![]).unwrap();
        service.add(id);
        service.goal(id, [2., 0.]).unwrap();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(15);
        loop {
            service.poll(&field);
            if service.views()[0].phase == Phase::Following {
                break;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "{:?}",
                service.views()
            );
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        for _ in 0..(25_000_000_000 / tick_ns()) {
            service.tick(&mut field);
            field.step(1).unwrap();
            if service.views()[0].phase != Phase::Following {
                break;
            }
        }
        let view = &service.views()[0];
        let body = field.chassis_snapshot(id).unwrap();
        assert_eq!(
            view.phase,
            Phase::Arrived,
            "{view:?}, body={:?}",
            body.pose.translation_m
        );
        assert!(distance(body.pose.translation_m, [2., 0., 0.]) < 0.22);
        service.goal(id, [-2., 0.]).unwrap();
        loop {
            service.poll(&field);
            if service.views()[0].phase == Phase::Following {
                break;
            }
            assert!(std::time::Instant::now() < deadline);
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        let p = field.chassis_snapshot(id).unwrap().pose.translation_m;
        let other = field
            .add_chassis(&ChassisPlacement {
                config: config.clone(),
                spawn: Pose::at([p[0] - 0.9, p[1], config.rest_height_m()]),
                team: Team::Blue,
                kind: RobotKind::Infantry,
                performance: None,
            })
            .unwrap();
        for _ in 0..16 {
            service.tick(&mut field);
            field.step(1).unwrap();
        }
        assert_eq!(service.views()[0].phase, Phase::Planning);
        assert_eq!(field.chassis_snapshot(id).unwrap().command.forward_m_s, 0.);
        assert!(field.chassis_snapshot(other).is_some());
        loop {
            service.poll(&field);
            if service.views()[0].phase != Phase::Planning {
                break;
            }
            assert!(std::time::Instant::now() < deadline);
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        assert_eq!(
            service.views()[0].phase,
            Phase::Following,
            "{:?}",
            service.views()
        );
        for _ in 0..(35_000_000_000 / tick_ns()) {
            service.tick(&mut field);
            field.step(1).unwrap();
            while service.views()[0].phase == Phase::Planning {
                service.poll(&field);
                assert!(std::time::Instant::now() < deadline);
                std::thread::sleep(std::time::Duration::from_millis(2));
            }
            if service.views()[0].phase != Phase::Following {
                break;
            }
        }
        assert_eq!(
            service.views()[0].phase,
            Phase::Arrived,
            "{:?}",
            service.views()
        );
    }
}
