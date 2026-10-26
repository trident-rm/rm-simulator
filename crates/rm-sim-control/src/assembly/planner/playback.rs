// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Guarded plan playback and explicit physical retiming.
use super::{
    Frame, IkTable, JointSearch, Plan, PlanError, Rig, Search, Signal, Stage, Statistics,
    TransitionTable, at, breakaway_edge, chassis_cadence, compose, inverse, joint_path, pose,
    position, quaternion,
};
impl Rig {
    /// Validate and apply one precomputed frame through the ordinary atomic guard.
    pub fn follow_plan(&mut self, f: &Frame) -> Result<(), &'static str> {
        if !(1..=8).contains(&f.ticks) {
            return Err("invalid planned sample duration");
        }
        if f.q
            .iter()
            .zip(self.config.engineer.axes)
            .any(|(q, a)| !q.is_finite() || !(a.range_rad[0]..=a.range_rad[1]).contains(q))
        {
            return Err("invalid planned joints");
        }
        chassis_cadence(self.engineer_body, f.chassis)?;
        let mut next = self.clone();
        next.engineer_body = f.chassis;
        if !f.rod_spin_rad.is_finite()
            || (f.rod_spin_rad - self.rod_spin_rad).abs()
                > super::RATE * rm_sim_physics::tick_ns() as f64 * 1e-9 * f.ticks as f64 + 1e-9
        {
            return Err("planned rod spin exceeds workbench cadence");
        }
        next.rod_spin_rad = f.rod_spin_rad;
        next.advance_gripper(
            f.close,
            rm_sim_physics::tick_ns() as f64 * 1e-9 * f.ticks as f64,
        );
        next = at(next, f.q);
        if !f.free {
            let expected = next.target(f.logical, f.angles);
            next.position_error_m = position(next.tool).distance(position(expected));
            next.angle_error_rad = quaternion(next.tool).angle_between(quaternion(expected));
            if next.position_error_m > 1e-5 || next.angle_error_rad > 1e-4 {
                return Err("planned FK diverged from Cartesian guide");
            }
            if next.gripped {
                next.lift_m = f.logical[2] - 0.6;
                next.p_rad = f.angles[0];
                next.q_rad = f.angles[1];
            }
        }
        next.peak_joint_step_rad = next.peak_joint_step_rad.max(
            (0..6)
                .map(|i| (self.engineer_q[i] - f.q[i]).abs())
                .fold(0., f64::max),
        );
        if f.validation
            .as_ref()
            .is_some_and(|v| v.matches(self, &next))
        {
            next.last_collision = None;
            next.cached_guard_replays += 1;
            *self = next;
            Ok(())
        } else {
            self.accept_candidate(next)
        }
    }
    /// Replay validated keyframes to a kinematic endpoint without actuating a world.
    pub fn runtime_endpoint(&self, plan: &Plan) -> Result<Rig, &'static str> {
        let mut end = self.clone();
        for frame in &plan.frames {
            end.follow_plan(frame)?;
        }
        Ok(end)
    }
    /// Release the seated Unit, clear the socket and return through swept-clear motion.
    pub fn runtime_breakaway_frames(
        &self,
        last: &super::PlaybackFrame,
        signal: std::sync::Arc<Signal>,
    ) -> Result<Vec<super::PlaybackFrame>, &'static str> {
        use super::NEUTRAL_JOINTS_RAD;
        let mut search = Search {
            stats: Statistics::default(),
            budget: 2500,
            ik_table: IkTable::default(),
            transitions: TransitionTable::default(),
            last: None,
            last_checked: None,
            stage: Stage::Approach,
            signal: Some(signal),
        };
        let mut root = at(self.clone(), last.joints_rad);
        root.jaw_angles = last.jaws_rad.to_vec();
        root.unit_pose = pose(position(last.unit), quaternion(last.unit).normalize());
        root.gripped = false;
        root.grip_requested = false;
        // Opening fully beside the flipped Core can strike its housing. Search
        // a partial pad opening, withdraw, then finish opening in clear space.
        for fraction in [1., 0.5, 0.25, 0.125] {
            let mut opened = root.clone();
            let mut opening = Vec::new();
            let mut target = root.clone();
            for (i, jaw) in root.config.engineer_jaws.iter().enumerate() {
                target.jaw_angles[i] =
                    root.jaw_angles[i] + (jaw.open_rad - root.jaw_angles[i]) * fraction;
            }
            if breakaway_edge(&mut opened, target, &mut search, last, &mut opening).is_err() {
                continue;
            }
            let rotation = quaternion(opened.tool);
            for escape in [
                None,
                Some(rotation * glam::DVec3::Z),
                Some(-(rotation * glam::DVec3::Z)),
                Some(-(rotation * glam::DVec3::X)),
                Some(rotation * glam::DVec3::X),
                Some(glam::DVec3::Z),
            ] {
                let mut trial = opened.clone();
                let mut frames = opening.clone();
                let mut clear = true;
                if let Some(direction) = escape {
                    for i in 1..=40 {
                        let desired = pose(
                            position(opened.tool) + direction * (0.12 * i as f64 / 40.),
                            rotation,
                        );
                        let Ok(q) = trial.config.engineer.solve(
                            compose(inverse(trial.engineer_body), desired),
                            trial.engineer_q,
                        ) else {
                            clear = false;
                            break;
                        };
                        let next = at(trial.clone(), q);
                        if breakaway_edge(&mut trial, next, &mut search, last, &mut frames).is_err()
                        {
                            clear = false;
                            break;
                        }
                    }
                }
                if !clear {
                    continue;
                }
                let mut target = trial.clone();
                for (a, j) in target
                    .jaw_angles
                    .iter_mut()
                    .zip(&target.config.engineer_jaws)
                {
                    *a = j.open_rad;
                }
                if breakaway_edge(&mut trial, target, &mut search, last, &mut frames).is_err() {
                    continue;
                }
                let model = trial.clone();
                let path = joint_path(
                    model.config.engineer.axes.map(|a| a.range_rad),
                    model.engineer_q,
                    NEUTRAL_JOINTS_RAD,
                    JointSearch {
                        // Preserve difficult post-flip return paths; the shared
                        // collision-check budget still bounds this search.
                        samples: 256,
                        ..Default::default()
                    },
                    |a, b| {
                        let n = ((0..6).map(|j| (b[j] - a[j]).abs()).fold(0., f64::max) / 0.1)
                            .ceil()
                            .max(1.) as usize;
                        let mut previous = at(model.clone(), a);
                        for i in 1..=n {
                            let next = at(
                                model.clone(),
                                std::array::from_fn(|j| a[j] + (b[j] - a[j]) * i as f64 / n as f64),
                            );
                            match search.check(&previous, next) {
                                Ok(r) => previous = r,
                                Err("motion blocked by collision") => return Ok(false),
                                Err(_) => return Err(PlanError::CollisionQuery),
                            }
                        }
                        Ok(true)
                    },
                );
                let Ok(path) = path else {
                    continue;
                };
                for q in path.into_iter().skip(1) {
                    let next = at(trial.clone(), q);
                    if breakaway_edge(&mut trial, next, &mut search, last, &mut frames).is_err() {
                        clear = false;
                        break;
                    }
                }
                if clear {
                    return Ok(frames);
                }
            }
        }
        Err("no collision-free breakaway and neutral return found")
    }
    /// Retime validated motion along its existing joint path. No IK goal or
    /// endpoint is projected; slow joints lengthen the corresponding segment.
    pub fn runtime_limited_frames(
        &self,
        plan: &Plan,
    ) -> Result<Vec<super::PlaybackFrame>, &'static str> {
        use super::{JAW_SPEED_RAD_S, JOINT_SPEED_RAD_S, PlaybackFrame as Output};
        let source = self.runtime_frames(plan)?;
        let mut previous = Output {
            joints_rad: self.engineer_q,
            jaws_rad: [self.jaw_angles[0], self.jaw_angles[1]],
            unit: self.unit_pose,
            guide_m: if self.gripped && self.live_entry.is_none() {
                [-0.05, 0.2, 0.6 + self.lift_m]
            } else {
                [0.09, 0.2, 0.6]
            },
            angles_rad: [self.p_rad, self.q_rad, 0.],
            gripped: self.gripped,
            stage: 0,
        };
        let dt = rm_sim_physics::tick_ns() as f64 * 1e-9;
        let mut output = Vec::new();
        for (next, planned) in source.into_iter().zip(&plan.frames) {
            let count = (0..6)
                .map(|i| {
                    ((next.joints_rad[i] - previous.joints_rad[i]).abs()
                        / (JOINT_SPEED_RAD_S[i] * dt))
                        .ceil() as usize
                })
                .chain((0..2).map(|i| {
                    ((next.jaws_rad[i] - previous.jaws_rad[i]).abs() / (JAW_SPEED_RAD_S * dt))
                        .ceil() as usize
                }))
                .max()
                .unwrap_or(1)
                .max(planned.ticks as usize);
            for i in 1..=count {
                let t = i as f64 / count as f64;
                let mix = |a: f64, b: f64| ((a + (b - a) * t) * 1e6).round() / 1e6;
                let mut frame = next.clone();
                frame.joints_rad =
                    std::array::from_fn(|j| mix(previous.joints_rad[j], next.joints_rad[j]));
                frame.jaws_rad =
                    std::array::from_fn(|j| mix(previous.jaws_rad[j], next.jaws_rad[j]));
                frame.guide_m = std::array::from_fn(|j| mix(previous.guide_m[j], next.guide_m[j]));
                frame.angles_rad =
                    std::array::from_fn(|j| mix(previous.angles_rad[j], next.angles_rad[j]));
                let q = quaternion(previous.unit).slerp(quaternion(next.unit), t);
                frame.unit = if previous.gripped && next.gripped {
                    // Retiming must retain the rigid grasp even between source
                    // samples: interpolating payload poses separately drifts.
                    at(self.clone(), frame.joints_rad).tool_unit_pose()
                } else {
                    pose(position(previous.unit).lerp(position(next.unit), t), q)
                };
                frame.unit.translation_m =
                    frame.unit.translation_m.map(|v| (v * 1e6).round() / 1e6);
                frame.unit.rotation_wxyz =
                    frame.unit.rotation_wxyz.map(|v| (v * 1e7).round() / 1e7);
                frame.gripped = if i == count {
                    next.gripped
                } else {
                    previous.gripped
                };
                output.push(frame);
            }
            previous = next;
        }
        Ok(output)
    }
}
