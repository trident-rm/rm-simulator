// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Bounded background assembly planning; the host alone installs completed jobs.
use crate::assembly_rig::{Rig, Signal};
use rm_sim_world::{
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
    end: Rig,
    frames: Vec<rm_sim_world::assembly::Frame>,
}
struct Job {
    generation: u64,
    signal: Arc<Signal>,
    receiver: mpsc::Receiver<Result<Solved, String>>,
    begin: Option<mpsc::SyncSender<()>>,
    twist: bool,
}
pub(crate) struct Service {
    template: Rig,
    unit_shape: rm_sim_world::projectile::PayloadShape,
    recovery_core_rad: [[f64; 6]; 2],
    origins: [Pose; 2],
    generation: u64,
    jobs: BTreeMap<u32, Job>,
    warm: Option<Rig>,
    resumes: BTreeMap<u32, (u64, Rig)>,
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
        let target = rm_sim_world::gameplay::rmuc_2027::assembly_pose::TargetPose {
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
        let collision: rm_sim_physics::manipulator_collision::CollisionAsset =
            serde_json::from_slice(
                &std::fs::read(package.join("collision.json")).map_err(|e| e.to_string())?,
            )
            .map_err(|e| e.to_string())?;
        let unit = collision
            .bodies
            .iter()
            .find(|b| b.frame == "unit/unit")
            .ok_or("missing source Unit collision")?;
        let recovery_core_rad = template.recovery_core_joints().map_err(str::to_string)?;
        // UC §4.3, Figure 4-39: Unit mass 400±50 g; use the nominal mass.
        let unit_shape = rm_sim_world::projectile::PayloadShape::from_parts(&unit.parts, 0.4)
            .map_err(str::to_string)?;
        Ok(Self {
            template,
            unit_shape,
            recovery_core_rad,
            origins,
            generation: 0,
            jobs: Default::default(),
            warm: None,
            resumes: Default::default(),
        })
    }
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn start(
        &mut self,
        difficulty: u8,
        chassis: &rm_sim_world::ChassisSnapshot,
        now_ns: u64,
        awards_match: bool,
        arm: &rm_sim_world::assembly::ArmState,
        previous: Option<&rm_sim_world::assembly::RigPose>,
        unit: &rm_sim_world::energy_unit::EnergyUnit,
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
        if unit.attachment != rm_sim_world::energy_unit::Attachment::Held(owner) {
            return Err("Assembly requires the existing held Unit".into());
        }
        rig.use_held_unit(&crate::assembly_rig::payload(unit, owner));
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
            recovery: Some(rm_sim_world::assembly::CoreRecovery::new(
                &rig.runtime_pose(),
                self.recovery_core_rad[0],
                self.recovery_core_rad[1],
            )),
            twist_target_rad: None,
            unit_grasp: Some(unit.grasp),
            checks: 0,
            budget: 7500,
            message: "Moving Tech Core into position".into(),
            world_from_rig: origin,
            model: rig.runtime_model(),
            pose: rig
                .carried_start(&crate::assembly_rig::arm_pose(arm))
                .runtime_pose(),
            frames: Vec::new(),
            cursor: 0,
            started_ns: now_ns,
            attempt: rm_sim_world::gameplay::rmuc_2027::manipulation::Attempt::new(
                difficulty, [0.; 2], now_ns,
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
        run.pose.gripped = true;
        run.positioning = Some(rm_sim_world::assembly::CorePositioning::new(
            &run.pose,
            target_core,
        ));
        run.pose.engineer_q = arm.joints_rad;
        run.pose.jaw_angles = arm.jaws_rad;
        rig.set_live_entry(&crate::assembly_rig::arm_pose(arm));
        let worker_signal = signal.clone();
        std::thread::Builder::new()
            .name("assembly-planner".into())
            .spawn(move || {
                // No robot IK, collision checks or trajectory work before the
                // host sees the Core reach its target on simulation time.
                if begin_receiver.recv().is_err() {
                    return;
                }
                let result = solve_segment(rig, difficulty, None, worker_signal);
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
                twist: false,
            },
        );
        Ok(run)
    }
    pub(crate) fn jog(
        &self,
        field: &mut rm_sim_world::Field,
        owner: u32,
        translation_m: [f64; 3],
        rotation_rad: [f64; 3],
    ) -> Result<(), String> {
        let target = field
            .energy_arm_jog_target(owner, translation_m, rotation_rad)
            .map_err(|e| e.to_string())?;
        let chassis = field
            .chassis_snapshot(owner)
            .ok_or("missing source robot")?;
        let arm = field
            .arms()
            .iter()
            .find(|a| a.owner == owner)
            .ok_or("missing source arm")?;
        let units = field.energy_units();
        let wrist = compose(chassis.pose, arm.model.engineer.tool_pose(arm.joints_rad));
        let unit = units
            .iter()
            .find(|u| u.attachment == rm_sim_world::energy_unit::Attachment::Held(owner))
            .or_else(|| {
                units.iter().min_by(|a, b| {
                    let distance = |u: &rm_sim_world::energy_unit::EnergyUnit| {
                        u.physical
                            .pose
                            .translation_m
                            .iter()
                            .zip(wrist.translation_m)
                            .map(|(a, b)| (a - b).powi(2))
                            .sum::<f64>()
                    };
                    distance(a).total_cmp(&distance(b))
                })
            });
        let origin = self.origins[chassis.team.index()];
        let station = field
            .assemblies()
            .iter()
            .find(|r| r.team == chassis.team)
            .map(|r| &r.pose);
        self.template
            .validate_live_jog(
                compose(inverse(origin), chassis.pose),
                &crate::assembly_rig::arm_pose(arm),
                target,
                unit.map(|u| crate::assembly_rig::payload(u, owner))
                    .as_ref(),
                origin,
                station,
            )
            .map_err(str::to_string)?;
        field
            .command_energy_arm(owner, target)
            .map_err(|e| e.to_string())
    }
    pub(crate) fn poll(&mut self, field: &mut rm_sim_world::Field) {
        for chassis in field.chassis_snapshots() {
            if chassis
                .config
                .articulation
                .as_ref()
                .is_some_and(|a| a.profile == "pie-tunnel-heavy-v1")
                && !field.arms().iter().any(|a| a.owner == chassis.id)
                && field
                    .add_arm(rm_sim_world::assembly::ArmState::neutral(
                        chassis.id,
                        self.template.runtime_model(),
                    ))
                    .is_ok()
            {
                let result = field.preload_energy_unit(chassis.id, self.unit_shape.clone());
                if let Err(error) = result {
                    eprintln!("Could not initialize Energy Unit: {error}");
                }
            }
        }
        let waiting: Vec<_> = field
            .assemblies()
            .iter()
            .filter(|r| r.phase == Phase::AwaitingTarget)
            .cloned()
            .collect();
        for mut run in waiting {
            let Some((generation, mut rig)) = self.resumes.remove(&run.owner) else {
                field.cancel_assembly(run.owner, "Missing post-flip planner context");
                continue;
            };
            if generation != run.generation {
                field.cancel_assembly(run.owner, "Stale post-flip planner context");
                continue;
            }
            if let Err(error) = rig.adopt_runtime_pose(&run.pose) {
                field.cancel_assembly(run.owner, error);
                continue;
            }
            let Some(target) = run.twist_target_rad else {
                field.cancel_assembly(run.owner, "Q target not observed");
                continue;
            };
            let signal = Arc::new(Signal::default());
            let worker_signal = signal.clone();
            let (sender, receiver) = mpsc::sync_channel(1);
            let difficulty = run.difficulty;
            let spawned = std::thread::Builder::new()
                .name("assembly-twist-planner".into())
                .spawn(move || {
                    let result = solve_segment(rig, difficulty, Some(target), worker_signal);
                    let _ = sender.send(result);
                });
            if let Err(error) = spawned {
                field.cancel_assembly(run.owner, &error.to_string());
                continue;
            }
            self.jobs.insert(
                run.owner,
                Job {
                    generation,
                    signal,
                    receiver,
                    begin: None,
                    twist: true,
                },
            );
            run.phase = Phase::SolvingTwist;
            run.checks = 0;
            run.message = format!(
                "Solving observed Q target {:.1} degrees",
                target.to_degrees()
            );
            let _ = field.set_assembly(run);
        }
        self.resumes.retain(|owner, (generation, _)| {
            field
                .assemblies()
                .iter()
                .any(|r| r.owner == *owner && r.generation == *generation && r.active())
        });
        let ids: Vec<_> = self.jobs.keys().copied().collect();
        for owner in ids {
            let job = self.jobs.get_mut(&owner).unwrap();
            let twist = job.twist;
            let current = field
                .assemblies()
                .iter()
                .find(|r| {
                    r.owner == owner
                        && r.generation == job.generation
                        && matches!(
                            r.phase,
                            Phase::Positioning | Phase::Solving | Phase::SolvingTwist
                        )
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
                            if twist
                                && ((0..6).any(|i| {
                                    (start_pose.engineer_q[i] - run.pose.engineer_q[i]).abs() > 1e-6
                                }) || (start_pose.p_rad - run.pose.p_rad).abs() > 1e-6
                                    || (start_pose.q_rad - run.pose.q_rad).abs() > 1e-6)
                            {
                                field.cancel_assembly(
                                    owner,
                                    "Twist plan changed its actual start state",
                                );
                                continue;
                            }
                            run.pose = start_pose;
                            run.pose.engineer_q = arm.joints_rad;
                            run.pose.jaw_angles = arm.jaws_rad;
                            run.frames = solved.frames;
                            run.cursor = 0;
                            run.phase = Phase::Actuating;
                            run.message = "Actuating validated path".into();
                            if !twist {
                                run.attempt =
                                    rm_sim_world::gameplay::rmuc_2027::manipulation::Attempt::new(
                                        run.difficulty,
                                        [0.; 2],
                                        field.time_ns(),
                                    )
                                    .unwrap();
                                if run.difficulty == 3 {
                                    self.resumes.insert(owner, (run.generation, solved.end));
                                }
                            }
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
fn solve_segment(
    mut rig: Rig,
    difficulty: u8,
    twist: Option<f64>,
    signal: Arc<Signal>,
) -> Result<Solved, String> {
    if twist.is_none() {
        rig.prepare_live_gripper().map_err(str::to_string)?;
    }
    let (mut start, plan) = rig
        .plan_runtime_segment(difficulty, twist, 5000, signal.clone())
        .map_err(|f| {
            #[cfg(test)]
            eprintln!(
                "runtime planning failure: {}",
                serde_json::to_string(&f).unwrap()
            );
            f.reason
        })?;
    if signal.cancelled.load(Ordering::Relaxed) {
        return Err("Assembly cancelled".into());
    }
    let mut frames = start
        .runtime_limited_frames(&plan)
        .map_err(str::to_string)?;
    if frames.len() > 4096 {
        return Err("Arm trajectory exceeds the 32-second playback capacity".into());
    }
    let mut end = start.runtime_endpoint(&plan).map_err(str::to_string)?;
    start.remember_runtime_plan(&plan);
    end.reuse_runtime_hints(&start);
    if difficulty != 3 || twist.is_some() {
        let last = frames.last().ok_or("empty assembly trajectory")?;
        frames.extend(
            end.runtime_breakaway_frames(last, signal)
                .map_err(str::to_string)?,
        );
        if frames.len() > 4096 {
            return Err("Assembly and return exceed the playback capacity".into());
        }
    }
    Ok(Solved {
        start,
        end,
        frames: frames
            .into_iter()
            .map(crate::assembly_rig::playback)
            .collect(),
    })
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
    use rm_sim_world::Team;
    #[test]
    #[ignore = "requires generated source CAD and assembly terrain"]
    fn source_preloaded_unit_starts_clear_at_actual_wrist() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../local-assets/assembly-cad");
        let service = Service::load(
            &root.join("package"),
            &root.join("area"),
            None,
            -1.640343675041,
        )
        .unwrap();
        eprintln!(
            "Unit convex hull vertices: {:?}",
            service
                .unit_shape
                .hulls_m
                .iter()
                .map(Vec::len)
                .collect::<Vec<_>>()
        );
        let mut rig = service.template.clone();
        rig.set_live_parking(Pose::yawed(
            [1.278000049957, -0.547914948432, 0.383150523114],
            195f64.to_radians(),
        ))
        .unwrap();
        let arm = rm_sim_world::assembly::ArmState::neutral(0, rig.runtime_model());
        let carried = rig.carried_start(&crate::assembly_rig::arm_pose(&arm));
        assert!(
            carried.collision_contacts().unwrap().is_empty(),
            "loaded collisions: {:?}",
            carried.collision_contacts().unwrap()
        );
        assert!(carried.runtime_pose().gripped);
        assert_eq!(carried.runtime_pose().engineer_q, arm.joints_rad);
    }
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
            if std::env::var("RM_ASSEMBLY_TEST_LEVEL")
                .ok()
                .is_some_and(|s| s != difficulty.to_string())
            {
                continue;
            }
            for (q, target, normal) in [
                (service.recovery_core_rad[0], [-0.2, 0.2, 0.8], [1., 0., 0.]),
                (
                    service.recovery_core_rad[1],
                    [-0.45, 0., 0.65],
                    [0., 0., -1.],
                ),
            ] {
                let local = compose(
                    inverse(service.template.runtime_model().assembly_frame),
                    service.template.runtime_model().core.tool_pose(q),
                );
                assert!(
                    local
                        .translation_m
                        .into_iter()
                        .zip(target)
                        .all(|(a, b)| (a - b).abs() < 1e-4)
                );
                let axis = crate::math::rotate(local.rotation_wxyz, [1., 0., 0.]);
                assert!(
                    axis.into_iter()
                        .zip(normal)
                        .map(|(a, b)| a * b)
                        .sum::<f64>()
                        > 0.99999
                );
            }
            let config = rm_sim_world::chassis::ChassisConfig::default();
            let mut field = rm_sim_world::Field::new(&Default::default()).unwrap();
            let world_body = compose(service.origins[0], body);
            let id = field
                .add_chassis(&rm_sim_world::ChassisPlacement {
                    spawn: world_body,
                    team: Team::Red,
                    kind: rm_sim_world::RobotKind::Engineer,
                    performance: None,
                    config,
                })
                .unwrap();
            let chassis = field.chassis_snapshot(id).unwrap();
            let arm =
                rm_sim_world::assembly::ArmState::neutral(id, service.template.runtime_model());
            field.add_arm(arm.clone()).unwrap();
            field
                .preload_energy_unit(id, service.unit_shape.clone())
                .unwrap();
            let arm = field.arms()[0].clone();
            let unit = field.energy_units().remove(0);
            let mut run = service
                .start(difficulty, &chassis, 0, false, &arm, None, &unit)
                .unwrap();
            assert!(
                run.body_world()
                    .translation_m
                    .into_iter()
                    .zip(world_body.translation_m)
                    .all(|(a, b)| (a - b).abs() < 1e-12)
            );
            assert!(run.pose.gripped);
            let mut carried = service.template.clone();
            carried.set_live_parking(run.pose.engineer_body).unwrap();
            let carried = carried.carried_start(&crate::assembly_rig::arm_pose(&arm));
            assert_eq!(run.pose.unit_pose, carried.runtime_pose().unit_pose);
            let loaded_unit = run.pose.unit_pose;
            assert_eq!(
                run.attempt.cores[0].step,
                rm_sim_world::gameplay::rmuc_2027::manipulation::Step::Approach
            );
            assert_eq!(run.phase, Phase::Positioning);
            assert_eq!(service.jobs[&id].signal.checks.load(Ordering::Relaxed), 0);
            assert!(matches!(
                service.jobs[&id].receiver.try_recv(),
                Err(mpsc::TryRecvError::Empty)
            ));
            let duration = run.positioning.as_ref().unwrap().duration_ns;
            for tick in 1..=duration / rm_sim_world::tick_ns() {
                run.advance(tick * rm_sim_world::tick_ns());
            }
            assert_eq!(run.pose.unit_pose, loaded_unit);
            assert!(run.pose.gripped);
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
            eprintln!(
                "L{difficulty} initial solve: {:.3}s",
                start.elapsed().as_secs_f64()
            );
            run.pose = solved.start.runtime_pose();
            run.pose.engineer_q = arm.joints_rad;
            run.pose.jaw_angles = arm.jaws_rad;
            run.frames = solved.frames;
            assert!(run.frames.iter().all(|f| f.gripped == (f.stage < 7)));
            for f in run.frames.iter().filter(|f| f.stage < 2) {
                assert!(f.guide_m[0] >= 0.05 - 1e-6);
                assert_eq!(&f.guide_m[1..], &[0.2, 0.6]);
            }
            for f in run.frames.iter().filter(|f| f.gripped) {
                let carried = solved.start.clone();
                // Source replay and retiming must keep the payload rigid to the wrist.
                let arm = rm_sim_world::assembly::ArmState {
                    joints_rad: f.joints_rad,
                    ..arm.clone()
                };
                let expected = carried
                    .carried_start(&crate::assembly_rig::arm_pose(&arm))
                    .runtime_pose()
                    .unit_pose;
                assert!(
                    expected
                        .translation_m
                        .iter()
                        .zip(f.unit.translation_m)
                        .all(|(a, b)| (a - b).abs() < 2e-6)
                );
            }
            run.phase = Phase::Actuating;
            assert!(run.parking_matches(world_body));
            run.validate().unwrap();
            field.set_assembly(run.clone()).unwrap();
            if difficulty == 3 {
                let other = field
                    .add_chassis(&rm_sim_world::ChassisPlacement {
                        config: rm_sim_world::ChassisConfig::default(),
                        spawn: compose(service.origins[1], body),
                        team: Team::Blue,
                        kind: rm_sim_world::RobotKind::Engineer,
                        performance: None,
                    })
                    .unwrap();
                field
                    .add_arm(rm_sim_world::assembly::ArmState::neutral(
                        other,
                        run.model.clone(),
                    ))
                    .unwrap();
                field
                    .preload_energy_unit(other, service.unit_shape.clone())
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
                let now = (i as u64 + 1) * rm_sim_world::tick_ns();
                run.advance(now);
                restored.advance(now);
                if run.frames[i].stage < 2 {
                    assert!(
                        run.attempt.cores[0]
                            .milestones_ns
                            .iter()
                            .all(Option::is_none)
                    );
                }
            }
            assert_eq!(run, restored);
            if difficulty == 3 {
                assert_eq!(run.phase, Phase::AwaitingTarget);
                let target = run.twist_target_rad.unwrap();
                let mut endpoint = solved.end;
                endpoint.adopt_runtime_pose(&run.pose).unwrap();
                let twist_start = std::time::Instant::now();
                let held = endpoint.runtime_pose();
                let twist = match solve_segment(
                    endpoint.clone(),
                    difficulty,
                    Some(target),
                    Arc::new(Signal::default()),
                ) {
                    Ok(twist) => twist,
                    Err(error) => {
                        // An unknown target cannot be used to choose an earlier
                        // grasp. A bounded late failure is a valid controller
                        // outcome, never a completed assembly or a resample.
                        assert_eq!(
                            error,
                            "No attached Q/rod continuation found within the local search budget"
                        );
                        assert_eq!(endpoint.runtime_pose(), held);
                        assert_eq!(run.phase, Phase::AwaitingTarget);
                        assert_eq!(run.twist_target_rad, Some(target));
                        assert!(run.pose.gripped && !run.attempt.completed());
                        eprintln!(
                            "L3 revealed {:.2} degrees: bounded unresolved continuation after {:.3}s",
                            target.to_degrees(),
                            twist_start.elapsed().as_secs_f64()
                        );
                        service.warm = Some(solved.start);
                        continue;
                    }
                };
                run.pose = twist.start.runtime_pose();
                run.frames = twist.frames;
                run.cursor = 0;
                run.phase = Phase::Actuating;
                run.validate().unwrap();
                restored = serde_json::from_slice(&serde_json::to_vec(&run).unwrap()).unwrap();
                let base_now = run.attempt.now_ns;
                for i in 0..run.frames.len() {
                    let now = base_now + (i as u64 + 1) * rm_sim_world::tick_ns();
                    run.advance(now);
                    restored.advance(now);
                }
                assert_eq!(run, restored);
                eprintln!(
                    "L3 revealed {:.2} degrees: twist {:.3}s, {} frames",
                    target.to_degrees(),
                    twist_start.elapsed().as_secs_f64(),
                    run.frames.len()
                );
            }
            let mut recovery_tick = 0;
            while run.phase == Phase::Recovering {
                recovery_tick += 1;
                assert!(recovery_tick < 8192);
                let now = run.attempt.now_ns + recovery_tick * rm_sim_world::tick_ns();
                run.advance(now);
                restored.advance(now);
            }
            assert_eq!(run, restored);
            assert!(run.recovery.as_ref().unwrap().released());
            assert_eq!(
                run.phase,
                Phase::Completed,
                "L{difficulty}: {}",
                run.message
            );
            assert!((0..6).all(|i| {
                (run.pose.engineer_q[i] - rm_sim_world::assembly::NEUTRAL_JOINTS_RAD[i]).abs()
                    < 1e-6
            }));
            assert!(!run.pose.gripped);
            eprintln!(
                "L{difficulty}: {:.3}s, {} frames",
                start.elapsed().as_secs_f64(),
                run.frames.len()
            );
            service.warm = Some(solved.start);
        }
    }
}
