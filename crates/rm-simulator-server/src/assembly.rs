// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Bounded background assembly planning; the host alone installs completed jobs.
use crate::assembly_rig::{Rig, Signal};
use rm_simulator_world::{
    Pose,
    assembly::{Phase, Run},
};
use std::{
    collections::BTreeMap,
    path::Path,
    sync::{Arc, atomic::Ordering, mpsc},
};

struct Solved {
    start: Rig,
    frames: Vec<rm_simulator_world::assembly::Frame>,
}
struct Job {
    generation: u64,
    signal: Arc<Signal>,
    receiver: mpsc::Receiver<Result<Solved, String>>,
    begin: Option<mpsc::SyncSender<()>>,
}
pub(crate) struct Service {
    template: Rig,
    origins: [Pose; 2],
    generation: u64,
    jobs: BTreeMap<u32, Job>,
    warm: Option<Rig>,
}
fn compose(a: Pose, b: Pose) -> Pose {
    crate::assembly_rig::compose(a, b)
}
fn inverse(a: Pose) -> Pose {
    let q = crate::math::quat_conjugate(a.rotation_wxyz);
    Pose {
        translation_m: crate::math::rotate(q, a.translation_m.map(|v| -v)),
        rotation_wxyz: q,
    }
}
impl Service {
    pub(crate) fn load(
        package: &Path,
        area: &Path,
        hints: Option<&Path>,
        floor: f64,
    ) -> Result<Self, String> {
        let target = rm_simulator_world::gameplay::rmuc_2027::assembly_pose::TargetPose {
            position_m: [-0.05, 0.2, 0.5],
            theta_rad: 0.,
            phi_rad: std::f64::consts::FRAC_PI_2,
            alpha_rad: 0.,
        };
        let mut template = Rig::load_in_area(package, target, area).map_err(|e| e.to_string())?;
        template.use_live_collisions();
        template.select_live_core_branch().map_err(str::to_string)?;
        if let Some(hints) = hints {
            template.load_ik_hints(hints).map_err(|e| e.to_string())?;
        }
        let evidence: serde_json::Value = serde_json::from_slice(
            &std::fs::read(area.join("area.json")).map_err(|e| e.to_string())?,
        )
        .map_err(|e| e.to_string())?;
        let m: [[f64; 4]; 4] = serde_json::from_value(evidence["arena_to_workbench"].clone())
            .map_err(|e| e.to_string())?;
        let r = glam::DMat3::from_cols_array(&[
            m[0][0], m[1][0], m[2][0], m[0][1], m[1][1], m[2][1], m[0][2], m[1][2], m[2][2],
        ]);
        let q = glam::DQuat::from_mat3(&r);
        let arena_to_rig = Pose {
            translation_m: [m[0][3], m[1][3], m[2][3]],
            rotation_wxyz: [q.w, q.x, q.y, q.z],
        };
        let mut origin = inverse(arena_to_rig);
        origin.translation_m = crate::cad_assets::cad_point_to_flu(origin.translation_m, floor);
        let mirror = compose(Pose::yawed([0.; 3], std::f64::consts::PI), origin);
        let origins = if origin.translation_m[0] >= 0. {
            [origin, mirror]
        } else {
            [mirror, origin]
        };
        Ok(Self {
            template,
            origins,
            generation: 0,
            jobs: Default::default(),
            warm: None,
        })
    }
    pub(crate) fn start(
        &mut self,
        difficulty: u8,
        chassis: &rm_simulator_world::ChassisSnapshot,
        now_ns: u64,
        awards_match: bool,
        arm: &rm_simulator_world::assembly::ArmState,
        previous: Option<&rm_simulator_world::assembly::RigPose>,
    ) -> Result<Run, String> {
        let owner = arm.owner;
        let team = chassis.team;
        if !(1..=3).contains(&difficulty) {
            return Err("Choose assembly level 1, 2 or 3".into());
        }
        if self.jobs.len() >= 2 || self.jobs.contains_key(&owner) {
            return Err("Previous assembly worker is still stopping".into());
        }
        if chassis
            .velocity_m_s
            .into_iter()
            .chain(chassis.angular_velocity_rad_s)
            .any(|v| v.abs() > 0.05)
        {
            return Err("Stop the Heavy before solving assembly".into());
        }
        let origin = self.origins[team.index()];
        let mut rig = self.template.clone();
        if let Some(warm) = &self.warm {
            rig.reuse_runtime_hints(warm);
        }
        rig.set_live_parking(compose(inverse(origin), chassis.pose))
            .map_err(str::to_string)?;
        self.generation = self
            .generation
            .checked_add(1)
            .ok_or("assembly job identity exhausted")?;
        let generation = self.generation;
        let signal = Arc::new(Signal::default());
        let (sender, receiver) = mpsc::sync_channel(1);
        let (begin, begin_receiver) = mpsc::sync_channel(1);
        let mut run = Run {
            owner,
            team,
            generation,
            placement_revision: chassis.placement_revision,
            difficulty,
            phase: Phase::Positioning,
            positioning: None,
            checks: 0,
            budget: 5000,
            message: "Moving Tech Core into position".into(),
            world_from_rig: origin,
            model: rig.runtime_model(),
            pose: rig.runtime_pose(),
            frames: Vec::new(),
            cursor: 0,
            started_ns: now_ns,
            attempt: rm_simulator_world::gameplay::rmuc_2027::manipulation::Attempt::new(
                difficulty,
                [std::f64::consts::PI / 6.; 2],
                now_ns,
            )
            .map_err(str::to_string)?,
            awards_match,
        };
        let target_core = run.pose.core_q;
        if let Some(previous) = previous {
            run.pose.core_q = previous.core_q;
            run.pose.lift_m = previous.lift_m;
            run.pose.p_rad = previous.p_rad;
            run.pose.q_rad = previous.q_rad;
        } else {
            run.pose.core_q = [0.; 6];
        }
        run.pose.gripped = false;
        run.positioning = Some(rm_simulator_world::assembly::CorePositioning::new(
            &run.pose,
            target_core,
        ));
        run.pose.engineer_q = arm.joints_rad;
        run.pose.jaw_angles = arm.jaws_rad;
        rig.set_live_entry(arm);
        let worker_signal = signal.clone();
        std::thread::Builder::new()
            .name("assembly-planner".into())
            .spawn(move || {
                // No robot IK, collision checks or trajectory work before the
                // host sees the Core reach its target on simulation time.
                if begin_receiver.recv().is_err() {
                    return;
                }
                let result = rig
                    .prepare_live_gripper()
                    .map_err(str::to_string)
                    .and_then(|()| {
                        rig.plan_assembly_observed(
                            difficulty,
                            std::f64::consts::PI / 6.,
                            5000,
                            worker_signal.clone(),
                        )
                        .map_err(|f| f.reason)
                    })
                    .and_then(|(start, plan)| {
                        if worker_signal.cancelled.load(Ordering::Relaxed) {
                            return Err("Assembly cancelled".into());
                        }
                        let frames = start
                            .runtime_limited_frames(&plan)
                            .map_err(str::to_string)?;
                        if frames.len() > 4096 {
                            return Err(
                                "Arm trajectory exceeds the 32-second playback capacity".into()
                            );
                        }
                        let mut start = start;
                        start.remember_runtime_plan(&plan);
                        Ok(Solved { start, frames })
                    });
                let _ = sender.send(result);
            })
            .map_err(|e| e.to_string())?;
        self.jobs.insert(
            owner,
            Job {
                generation,
                signal,
                receiver,
                begin: Some(begin),
            },
        );
        Ok(run)
    }
    pub(crate) fn poll(&mut self, field: &mut rm_simulator_world::Field) {
        for chassis in field.chassis_snapshots() {
            if chassis
                .config
                .articulation
                .as_ref()
                .is_some_and(|a| a.profile == "pie-tunnel-heavy-v1")
                && !field.arms().iter().any(|a| a.owner == chassis.id)
            {
                let _ = field.add_arm(rm_simulator_world::assembly::ArmState::neutral(
                    chassis.id,
                    self.template.runtime_model(),
                ));
            }
        }
        let ids: Vec<_> = self.jobs.keys().copied().collect();
        for owner in ids {
            let job = self.jobs.get_mut(&owner).unwrap();
            let current = field
                .assemblies()
                .iter()
                .find(|r| {
                    r.owner == owner
                        && r.generation == job.generation
                        && matches!(r.phase, Phase::Positioning | Phase::Solving)
                })
                .cloned();
            if current.is_none() {
                job.signal.cancelled.store(true, Ordering::Relaxed);
                job.begin.take();
            }
            if current.as_ref().is_some_and(|r| r.phase == Phase::Solving)
                && let Some(begin) = job.begin.take()
            {
                let _ = begin.send(());
            }
            match job.receiver.try_recv() {
                Err(mpsc::TryRecvError::Empty) => {
                    if let Some(mut run) = current {
                        run.checks = job.signal.checks.load(Ordering::Relaxed) as u32;
                        let _ = field.set_assembly(run);
                    }
                }
                result => {
                    self.jobs.remove(&owner);
                    let Some(mut run) = current else {
                        continue;
                    };
                    match result
                        .unwrap_or_else(|_| Err("Assembly worker stopped unexpectedly".into()))
                    {
                        Ok(solved) => {
                            let current = field.chassis_snapshot(owner);
                            if !current.as_ref().is_some_and(|c| {
                                c.placement_revision == run.placement_revision
                                    && !c.defeated
                                    && run.parking_matches(c.pose)
                            }) {
                                field.cancel_assembly(owner, "Parking changed while solving");
                                continue;
                            }
                            let Some(arm) = field.arms().iter().find(|a| a.owner == owner) else {
                                field.cancel_assembly(owner, "Arm unavailable");
                                continue;
                            };
                            let start_pose = solved.start.runtime_pose();
                            if start_pose.core_q != run.pose.core_q {
                                field.cancel_assembly(
                                    owner,
                                    "Planner changed the positioned Core branch",
                                );
                                continue;
                            }
                            run.pose = start_pose;
                            run.pose.engineer_q = arm.joints_rad;
                            run.pose.jaw_angles = arm.jaws_rad;
                            run.frames = solved.frames;
                            run.phase = Phase::Actuating;
                            run.message = "Actuating validated path".into();
                            run.attempt=rm_simulator_world::gameplay::rmuc_2027::manipulation::Attempt::new(run.difficulty,[std::f64::consts::PI/6.;2],field.time_ns()).unwrap();
                            self.warm = Some(solved.start);
                            if let Err(error) = field.set_assembly(run) {
                                field.cancel_assembly(owner, &error.to_string());
                            }
                        }
                        Err(error) => {
                            field.cancel_assembly(owner, &error);
                            run.phase = Phase::Failed;
                            run.message = error.chars().take(256).collect();
                            let _ = field.set_assembly(run);
                        }
                    }
                }
            }
        }
    }
}
impl Drop for Service {
    fn drop(&mut self) {
        for job in self.jobs.values() {
            job.signal.cancelled.store(true, Ordering::Relaxed);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rm_simulator_world::Team;
    #[test]
    #[ignore = "requires generated source CAD and assembly terrain"]
    fn source_live_manual_parking_all_levels() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../local-assets/assembly-cad");
        let mut service = Service::load(
            &root.join("package"),
            &root.join("area"),
            Some(&root.join("rod-spin-d3.jsonl")),
            -1.640343675041,
        )
        .unwrap();
        let body = Pose::yawed(
            [1.278000049957, -0.547914948432, 0.383150523114],
            195f64.to_radians(),
        );
        let settled = Pose {
            translation_m: [1.2778059251617133, -0.5475544096622016, 0.38181622579516405],
            rotation_wxyz: [
                -0.13052596475502815,
                -2.0574039813800683e-05,
                5.414436070459979e-05,
                0.9914448896281963,
            ],
        };
        for (body, difficulty) in [body, settled]
            .into_iter()
            .flat_map(|b| (1..=3).map(move |d| (b, d)))
        {
            let config = rm_simulator_world::chassis::ChassisConfig::default();
            let mut field = rm_simulator_world::Field::new(&Default::default()).unwrap();
            let world_body = compose(service.origins[0], body);
            let id = field
                .add_chassis(&rm_simulator_world::ChassisPlacement {
                    spawn: world_body,
                    team: Team::Red,
                    kind: rm_simulator_world::RobotKind::Engineer,
                    performance: None,
                    config,
                })
                .unwrap();
            let chassis = field.chassis_snapshot(id).unwrap();
            let arm = rm_simulator_world::assembly::ArmState::neutral(
                id,
                service.template.runtime_model(),
            );
            field.add_arm(arm.clone()).unwrap();
            let mut run = service
                .start(difficulty, &chassis, 0, false, &arm, None)
                .unwrap();
            assert!(
                run.body_world()
                    .translation_m
                    .into_iter()
                    .zip(world_body.translation_m)
                    .all(|(a, b)| (a - b).abs() < 1e-12)
            );
            assert_eq!(run.phase, Phase::Positioning);
            assert_eq!(service.jobs[&id].signal.checks.load(Ordering::Relaxed), 0);
            assert!(matches!(
                service.jobs[&id].receiver.try_recv(),
                Err(mpsc::TryRecvError::Empty)
            ));
            let duration = run.positioning.as_ref().unwrap().duration_ns;
            for tick in 1..=duration / rm_simulator_world::tick_ns() {
                run.advance(tick * rm_simulator_world::tick_ns());
            }
            assert_eq!(run.phase, Phase::Solving);
            service
                .jobs
                .get_mut(&id)
                .unwrap()
                .begin
                .take()
                .unwrap()
                .send(())
                .unwrap();
            let start = std::time::Instant::now();
            let solved = service
                .jobs
                .remove(&id)
                .unwrap()
                .receiver
                .recv_timeout(std::time::Duration::from_secs(120))
                .unwrap()
                .unwrap();
            run.pose = solved.start.runtime_pose();
            run.pose.engineer_q = arm.joints_rad;
            run.pose.jaw_angles = arm.jaws_rad;
            run.frames = solved.frames;
            run.phase = Phase::Actuating;
            assert!(run.parking_matches(world_body));
            run.validate().unwrap();
            field.set_assembly(run.clone()).unwrap();
            if difficulty == 3 {
                let other = field
                    .add_chassis(&rm_simulator_world::ChassisPlacement {
                        config: rm_simulator_world::ChassisConfig::default(),
                        spawn: compose(service.origins[1], body),
                        team: Team::Blue,
                        kind: rm_simulator_world::RobotKind::Engineer,
                        performance: None,
                    })
                    .unwrap();
                let mut other_run = run.clone();
                other_run.owner = other;
                other_run.team = Team::Blue;
                other_run.world_from_rig = service.origins[1];
                other_run.placement_revision = field.chassis_revision(other).unwrap();
                field.set_assembly(other_run).unwrap();
            }
            let state = crate::simulation::Simulation::new(field, true).state();
            let bytes = crate::snapshot_codec::encode_player_message(
                &crate::protocol::ServerMessage::Snapshot(Box::new(state)),
            );
            assert!(
                bytes.len() < 60 * 1024,
                "assembly checkpoint is {} bytes",
                bytes.len()
            );
            let crate::protocol::ServerMessage::Snapshot(decoded) =
                crate::snapshot_codec::decode_player_message(&bytes).unwrap()
            else {
                panic!("snapshot")
            };
            assert_eq!(decoded.field.restore.as_ref().unwrap().assemblies[0], run);
            eprintln!("L{difficulty} checkpoint: {} bytes", bytes.len());
            let mut restored: Run =
                serde_json::from_slice(&serde_json::to_vec(&run).unwrap()).unwrap();
            for i in 0..run.frames.len() {
                let now = (i as u64 + 1) * rm_simulator_world::tick_ns();
                run.advance(now);
                restored.advance(now);
            }
            assert_eq!(run, restored);
            assert_eq!(
                run.phase,
                Phase::Completed,
                "L{difficulty}: {}",
                run.message
            );
            eprintln!(
                "L{difficulty}: {:.3}s, {} frames",
                start.elapsed().as_secs_f64(),
                run.frames.len()
            );
            service.warm = Some(solved.start);
        }
    }
}
