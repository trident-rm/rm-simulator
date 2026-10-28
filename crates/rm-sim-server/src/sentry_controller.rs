// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Optional host-side Sentry scanning/aiming. No automatic trigger or firing.
//! Optical detections use a finite camera cone and shared scenery line of sight.
use rm_sim_world::Team;
#[cfg(feature = "native-host")]
use rm_sim_world::{ChassisSnapshot, Field};
#[cfg(feature = "native-host")]
use std::collections::BTreeMap;

pub use crate::sentry_view::{Detection, View};

/// One host-owned controller per opted-in Sentry; never owns another physics world.
#[derive(Default)]
#[cfg(feature = "native-host")]
pub(crate) struct Service {
    agents: BTreeMap<u32, View>,
    outputs: BTreeMap<u32, ([f64; 2], f64)>,
}
#[cfg(feature = "native-host")]
fn angle(a: f64) -> f64 {
    a.sin().atan2(a.cos())
}
#[cfg(feature = "native-host")]
fn point(body: &ChassisSnapshot, local: [f64; 3]) -> [f64; 3] {
    let rotated = crate::math::rotate(body.pose.rotation_wxyz, local);
    std::array::from_fn(|i| body.pose.translation_m[i] + rotated[i])
}
#[cfg(feature = "native-host")]
fn aim(from: [f64; 3], to: [f64; 3]) -> [f64; 2] {
    let d: [f64; 3] = std::array::from_fn(|i| to[i] - from[i]);
    [d[1].atan2(d[0]), d[2].atan2(d[0].hypot(d[1]))]
}
#[cfg(feature = "native-host")]
impl Service {
    pub(crate) fn set(&mut self, id: u32, enabled: bool) {
        self.outputs.remove(&id);
        self.agents.insert(
            id,
            View {
                chassis: id,
                enabled,
                observed_tick: 0,
                target: None,
                heading_locked: false,
                detections: vec![],
            },
        );
    }
    pub(crate) fn remove(&mut self, id: u32) {
        self.agents.remove(&id);
        self.outputs.remove(&id);
    }
    pub(crate) fn active(&self) -> bool {
        self.agents.values().any(|a| a.enabled)
    }
    pub(crate) fn views(&self) -> Vec<View> {
        self.agents.values().cloned().collect()
    }
    pub(crate) fn tick(&mut self, field: &mut Field, navigation: &[crate::navigation::View]) {
        if !self.active() {
            return;
        }
        let stopped = field.rmuc_2027().is_some_and(|s| {
            matches!(
                s.game.phase,
                rm_sim_world::gameplay::rmuc_2027::match_rules::Phase::Countdown
                    | rm_sim_world::gameplay::rmuc_2027::match_rules::Phase::Finished
            )
        });
        if stopped {
            self.outputs.clear();
            for view in self.agents.values_mut().filter(|v| v.enabled) {
                view.target = None;
                view.detections.clear();
                if let Some(body) = field.chassis_snapshot(view.chassis) {
                    let mut command = body.command;
                    command.yaw_rate_rad_s = 0.;
                    command.aim_yaw_rad = body.held_aim_rad[0];
                    command.aim_pitch_rad = body.held_aim_rad[1];
                    let _ = field.command_chassis(body.id, command);
                }
            }
            return;
        }
        if !field.tick().is_multiple_of(8) {
            self.apply(field);
            return;
        }
        self.outputs.clear();
        let bodies = field.chassis_snapshots();
        self.agents
            .retain(|id, _| bodies.iter().any(|b| b.id == *id));
        let geometry = field.static_geometry_snapshot();
        let time_s = field.time_ns() as f64 * 1e-9;
        for view in self.agents.values_mut().filter(|a| a.enabled) {
            let Some(body) = bodies.iter().find(|b| b.id == view.chassis) else {
                continue;
            };
            view.detections.clear();
            view.observed_tick = field.tick();
            view.heading_locked = false;
            let previous = view.target.take();
            let native = field
                .rmuc_2027()
                .and_then(|s| s.game.robots.iter().find(|r| r.id == body.id));
            if body.defeated
                || native.is_some_and(|r| !r.online)
                || field.rmuc_2027().is_some_and(|s| {
                    matches!(
                        s.game.phase,
                        rm_sim_world::gameplay::rmuc_2027::match_rules::Phase::Countdown
                            | rm_sim_world::gameplay::rmuc_2027::match_rules::Phase::Finished
                    )
                })
            {
                continue;
            }
            let mut command = body.command;
            let nav = navigation.iter().find(|n| n.chassis == body.id);
            let in_passage = command.folded
                || body.fold_angle_rad > 0.001
                || nav.is_some_and(|n| {
                    n.phase == crate::navigation::Phase::Following
                        && n.tunnel_segments.get(n.waypoint).copied().unwrap_or(false)
                });
            view.heading_locked = in_passage;
            // Stow pitch and passage-frame alignment always outrank combat scanning.
            if in_passage {
                if nav.is_none_or(|n| n.phase != crate::navigation::Phase::Following) {
                    command.yaw_rate_rad_s = 0.;
                    command.aim_yaw_rad = rm_sim_world::chassis::yaw_of(body.pose);
                    command.aim_pitch_rad =
                        crate::math::rotate(body.pose.rotation_wxyz, [1., 0., 0.])[2]
                            .clamp(-1., 1.)
                            .asin();
                    let _ = field.command_chassis(body.id, command);
                }
                continue;
            }
            let origin = field
                .chassis_muzzle_pose(body.id)
                .unwrap_or(body.turret)
                .translation_m;
            let mut candidates = vec![];
            for enemy in bodies.iter().filter(|e| e.team != body.team && !e.defeated) {
                let delta: [f64; 3] =
                    std::array::from_fn(|i| enemy.pose.translation_m[i] - origin[i]);
                let distance = delta.iter().map(|x| x * x).sum::<f64>().sqrt();
                if distance > 20. {
                    continue;
                }
                let visible = enemy
                    .config
                    .armor_faces()
                    .into_iter()
                    .filter_map(|face| {
                        let p = point(enemy, face.translation_m);
                        let target = aim(origin, p);
                        if angle(target[0] - body.held_aim_rad[0]).abs() > 0.70
                            || (target[1] - body.held_aim_rad[1]).abs() > 0.5
                            || !geometry.segment_clear(origin, p)
                        {
                            return None;
                        }
                        // Other robot bodies can block the ray; exclude the observer/target.
                        let ray: [f64; 3] = std::array::from_fn(|i| p[i] - origin[i]);
                        let length2: f64 = ray.iter().map(|v| v * v).sum();
                        if bodies
                            .iter()
                            .filter(|b| b.id != body.id && b.id != enemy.id)
                            .any(|b| {
                                let t = (0..3)
                                    .map(|i| (b.pose.translation_m[i] - origin[i]) * ray[i])
                                    .sum::<f64>()
                                    / length2.max(1e-9);
                                t > 0.
                                    && t < 1.
                                    && (0..3)
                                        .map(|i| {
                                            let d =
                                                origin[i] + t * ray[i] - b.pose.translation_m[i];
                                            d * d
                                        })
                                        .sum::<f64>()
                                        < 0.28_f64.powi(2)
                            })
                        {
                            return None;
                        }
                        Some(target)
                    })
                    .next();
                if let Some(target) = visible {
                    view.detections.push(Detection {
                        robot: enemy.id,
                        position_m: enemy.pose.translation_m,
                    });
                    candidates.push((
                        enemy.id != previous.unwrap_or(u32::MAX),
                        distance,
                        enemy.id,
                        target,
                    ));
                }
            }
            candidates.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.total_cmp(&b.1)).then(a.2.cmp(&b.2)));
            if let Some((_, _, id, target)) = candidates.first() {
                view.target = Some(*id);
                // Track a measured armor center; motor speed/acceleration limits remain physical.
                command.aim_yaw_rad = target[0];
                command.aim_pitch_rad = target[1];
            } else {
                command.aim_yaw_rad = angle(time_s * 5.);
                command.aim_pitch_rad = -0.04 + 0.14 * (time_s * 0.7).sin();
            }
            let nav_stopped = nav.is_some_and(|n| {
                matches!(
                    n.phase,
                    crate::navigation::Phase::Blocked
                        | crate::navigation::Phase::Planning
                        | crate::navigation::Phase::Baking
                        | crate::navigation::Phase::Unreachable
                )
            });
            let turning_room = crate::navigation::rotation_clear(field, body.id, &geometry)
                && !bodies.iter().filter(|b| b.id != body.id).any(|b| {
                    let p = body.pose.translation_m;
                    let q = b.pose.translation_m;
                    (p[0] - q[0]).hypot(p[1] - q[1]) < 1.0 && (p[2] - q[2]).abs() < 0.7
                });
            if !nav_stopped && turning_room {
                command.yaw_rate_rad_s = -0.8;
            } else {
                command.yaw_rate_rad_s = 0.;
                view.heading_locked = true;
            }
            self.outputs.insert(
                body.id,
                (
                    [command.aim_yaw_rad, command.aim_pitch_rad],
                    command.yaw_rate_rad_s,
                ),
            );
            let _ = field.command_chassis(body.id, command);
        }
    }
    fn apply(&self, field: &mut Field) {
        for (&id, &(aim, rate)) in &self.outputs {
            let Some(body) = field.chassis_snapshot(id) else {
                continue;
            };
            if body.defeated
                || body.command.folded
                || body.fold_angle_rad > 0.001
                || field
                    .rmuc_2027()
                    .is_some_and(|s| s.game.robots.iter().any(|r| r.id == id && !r.online))
            {
                continue;
            }
            let mut command = body.command;
            command.aim_yaw_rad = aim[0];
            command.aim_pitch_rad = aim[1];
            command.yaw_rate_rad_s = rate;
            if command != body.command {
                let _ = field.command_chassis(id, command);
            }
        }
    }
}

/// Contacts visible to one team through enabled allied Sentry cameras.
/// Position detections contain no inferred health or ammunition information.
pub fn contacts(
    snapshot: &rm_sim_world::FieldSnapshot,
    views: &[View],
    observer: Team,
) -> Vec<crate::intelligence::Contact> {
    let mut contacts = crate::intelligence::contacts(snapshot, observer);
    for view in views.iter().filter(|v| {
        v.enabled
            && snapshot.tick.saturating_sub(v.observed_tick) <= 32
            && snapshot
                .chassis
                .iter()
                .any(|b| b.id == v.chassis && b.team == observer)
    }) {
        for d in &view.detections {
            if let Some(contact) = contacts.iter_mut().find(|c| c.robot == d.robot) {
                contact.position_m = d.position_m;
                contact.source = crate::intelligence::Source::Optical;
                contact.age_ns = 0;
            } else {
                // Status and position may arrive from different sensors: the
                // information-wave record need not contain its own coordinates.
                let information = snapshot.rmuc_2027.as_ref().and_then(|s| {
                    s.game.radar.information[rm_sim_world::referee::game_team(observer).index()]
                        .get(&d.robot)
                        .filter(|info| {
                            s.game.elapsed_ns().saturating_sub(info.observed_ns)
                                < rm_sim_world::gameplay::rmuc_2027::radar::INFORMATION_NS
                        })
                });
                contacts.push(crate::intelligence::Contact {
                    robot: d.robot,
                    team: observer.other(),
                    position_m: d.position_m,
                    source: crate::intelligence::Source::Optical,
                    progress_tenths: 0,
                    hp: information.and_then(|i| i.hp),
                    max_hp: information.and_then(|i| i.max_hp),
                    allowance: information.and_then(|i| i.allowance),
                    defense_pct: information.and_then(|i| i.defense_pct),
                    age_ns: 0,
                });
            }
        }
    }
    contacts.sort_by_key(|c| c.robot);
    contacts
}

#[cfg(all(test, feature = "native-host"))]
mod tests {
    use super::*;
    use rm_sim_world::{
        ChassisCommand, ChassisConfig, ChassisPlacement, FieldConfig, Pose, RobotKind,
    };
    fn robot(field: &mut Field, team: Team, position: [f64; 3]) -> u32 {
        field
            .add_chassis(&ChassisPlacement {
                team,
                config: ChassisConfig::default(),
                spawn: Pose::at(position),
                kind: RobotKind::Sentry,
                performance: None,
            })
            .unwrap()
    }
    #[test]
    fn acquisition_ignores_friends_and_blocked_targets_and_never_fires() {
        let mut field = Field::new(&FieldConfig::default()).unwrap();
        let own = robot(&mut field, Team::Red, [0., 0., 1.]);
        let enemy = robot(&mut field, Team::Blue, [3., 0., 1.]);
        robot(&mut field, Team::Red, [0., 3., 1.]);
        let mut service = Service::default();
        service.set(own, true);
        service.tick(&mut field, &[]);
        assert_eq!(service.views()[0].target, Some(enemy));
        assert_eq!(service.views()[0].detections.len(), 1);
        let before = field.snapshot().shots_fired;
        for _ in 0..8 {
            field.step(8).unwrap();
            service.tick(&mut field, &[]);
        }
        assert_eq!(field.snapshot().shots_fired, before);
        // A barrier intersects every armor ray, including the nearest/front plate.
        let mut physics = rm_sim_world::projectile::WorldPhysics::new(&[], 0.);
        physics.add_static_box(Pose::at([1.5, 0., 1.]), [0.05, 2., 2.]);
        let mut blocked = Field::new(&FieldConfig::default()).unwrap();
        let (vertices, triangles) = physics.static_geometry_snapshot(true).into_triangles();
        blocked.add_static_mesh(vertices, triangles).unwrap();
        let own = robot(&mut blocked, Team::Red, [0., 0., 1.]);
        robot(&mut blocked, Team::Blue, [3., 0., 1.]);
        service = Service::default();
        service.set(own, true);
        service.tick(&mut blocked, &[]);
        assert!(service.views()[0].target.is_none());
        assert!(service.views()[0].detections.is_empty());
        assert_eq!(
            blocked
                .chassis_snapshot(own)
                .unwrap()
                .command
                .yaw_rate_rad_s,
            -0.8
        );
    }
    #[test]
    fn optical_position_joins_independent_status_without_radar_coordinates() {
        use rm_sim_world::{RefereeCommand, gameplay::rmuc_2027::radar::Information};
        let mut field = Field::new(&crate::workload::field_config()).unwrap();
        let own = robot(&mut field, Team::Red, [0., 0., 1.]);
        let enemy = robot(&mut field, Team::Blue, [3., 0., 1.]);
        field.referee_command(RefereeCommand::StartMatch).unwrap();
        field.step(640).unwrap();
        field
            .referee_command(RefereeCommand::RadarInformation {
                team: Team::Red,
                robot: enemy,
                information: Information {
                    hp: Some(123),
                    max_hp: Some(400),
                    allowance: Some(77),
                    ..Default::default()
                },
            })
            .unwrap();
        assert!(crate::intelligence::contacts(&field.snapshot(), Team::Red).is_empty());
        let mut view = View {
            chassis: own,
            enabled: true,
            observed_tick: field.tick(),
            target: Some(enemy),
            heading_locked: false,
            detections: vec![Detection {
                robot: enemy,
                position_m: field.chassis_snapshot(enemy).unwrap().pose.translation_m,
            }],
        };
        let merged = contacts(&field.snapshot(), std::slice::from_ref(&view), Team::Red);
        assert_eq!(merged.len(), 1);
        assert_eq!(merged[0].source, crate::intelligence::Source::Optical);
        assert_eq!(merged[0].hp, Some(123));
        assert_eq!(merged[0].max_hp, Some(400));
        assert_eq!(merged[0].allowance, Some(77));
        assert_eq!(merged[0].defense_pct, None);
        assert!(contacts(&field.snapshot(), std::slice::from_ref(&view), Team::Blue).is_empty());
        field.step(128).unwrap();
        view.observed_tick = field.tick();
        let expired = contacts(&field.snapshot(), &[view], Team::Red);
        assert_eq!(expired.len(), 1);
        assert_eq!(expired[0].hp, None);
        assert_eq!(expired[0].allowance, None);
    }
    #[test]
    fn pilot_packets_do_not_cancel_host_axes_between_optical_samples() {
        let mut field = Field::new(&FieldConfig::default()).unwrap();
        let own = robot(&mut field, Team::Red, [0., 0., 1.]);
        let mut service = Service::default();
        service.set(own, true);
        service.tick(&mut field, &[]);
        let output = field.chassis_snapshot(own).unwrap().command;
        field.step(1).unwrap();
        field
            .command_chassis(
                own,
                ChassisCommand {
                    forward_m_s: 1.,
                    aim_yaw_rad: 2.,
                    ..Default::default()
                },
            )
            .unwrap();
        service.tick(&mut field, &[]);
        let merged = field.chassis_snapshot(own).unwrap().command;
        assert_eq!(merged.forward_m_s, 1.);
        assert_eq!(merged.aim_yaw_rad, output.aim_yaw_rad);
        assert_eq!(merged.yaw_rate_rad_s, output.yaw_rate_rad_s);
    }
    #[test]
    fn stow_and_manual_opt_out_preserve_the_navigation_command() {
        let mut field = Field::new(&FieldConfig::default()).unwrap();
        let own = robot(&mut field, Team::Red, [0., 0., 1.]);
        let command = ChassisCommand {
            folded: true,
            aim_pitch_rad: 0.1,
            yaw_rate_rad_s: 0.2,
            ..Default::default()
        };
        field.command_chassis(own, command).unwrap();
        let mut service = Service::default();
        service.set(own, true);
        service.tick(&mut field, &[]);
        let held = field.chassis_snapshot(own).unwrap().command;
        assert!(held.folded);
        assert_eq!(held.yaw_rate_rad_s, 0.);
        assert_eq!(held.aim_pitch_rad, 0.);
        assert!(service.views()[0].heading_locked);
        let nav = crate::navigation::View {
            chassis: own,
            phase: crate::navigation::Phase::Following,
            goal_m: Some([1., 0.]),
            path_m: vec![[0., 0., 0.], [1., 0., 0.]],
            tunnel_segments: vec![false, true],
            waypoint: 1,
            message: String::new(),
            bake: Default::default(),
        };
        field.command_chassis(own, command).unwrap();
        service.tick(&mut field, &[nav]);
        assert_eq!(field.chassis_snapshot(own).unwrap().command, command);
        service.set(own, false);
        field
            .command_chassis(own, ChassisCommand::default())
            .unwrap();
        service.tick(&mut field, &[]);
        assert_eq!(
            field.chassis_snapshot(own).unwrap().command,
            ChassisCommand::default()
        );
    }
}
