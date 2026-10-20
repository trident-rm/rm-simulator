// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Staged workbench planning. Setup search is allowed only before an attempt;
//! assisted planning preserves the current chassis, Core, payload and arm state.
use super::*;
use rm_simulator_physics::manipulator_plan::{JointSearch, PlanError, ik_candidates, joint_path};
use serde::Serialize;

// Source terrain sweeps are expensive. Reserve a small diagnostic allowance for
// partial prefixes; complete candidate execution always gets its full guard.
const PREFIX_AUDIT_CHECKS: usize = 32;
// Workbench playback limits, not measured drivetrain or competition limits.
const CHASSIS_SPEED_M_S: f64 = 0.25;
const CHASSIS_YAW_RAD_S: f64 = 0.35;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum Stage {
    Approach,
    Grasp,
    Insert,
    Lift,
    Flip,
    Align,
    Settle,
}
#[derive(Clone)]
pub struct Frame {
    /// Absolute chassis pose in world FLU metres and wxyz orientation.
    pub chassis: Pose,
    /// Unit rotation around the insertion rod, radians; rigid grip is retained.
    pub rod_spin_rad: f64,
    pub q: [f64; 6],
    pub logical: [f64; 3],
    pub angles: [f64; 3],
    pub close: bool,
    pub stage: Stage,
    pub free: bool,
    validation: Option<std::sync::Arc<ValidatedTransition>>,
}
#[derive(Clone, Default, Serialize)]
pub struct Statistics {
    /// Wall time spent searching and validating this plan, excluding load/playback.
    pub planning_elapsed_s: f64,
    pub candidates: usize,
    pub ik_queries: usize,
    pub nearby_ik_hints: usize,
    pub exact_ik_hints: usize,
    pub collision_checks: usize,
    pub collision_table_hits: usize,
    pub frames: usize,
    pub setup_changed: bool,
    pub reverse_roots: usize,
    pub tested_setups: usize,
    pub route_trials: BTreeMap<usize, usize>,
    pub unsupported_route_samples: usize,
    pub segment_checks: usize,
    pub connected_segments: BTreeMap<String, usize>,
    pub selected_chassis: Option<Pose>,
    /// Chosen prescribed base trajectory, available only after a complete plan.
    pub selected_route: Option<ChassisRoute>,
    pub milestone_witnesses: BTreeMap<String, Witness>,
    pub swept_segment_witnesses: BTreeMap<String, Witness>,
    pub coarse_segment_witnesses: BTreeMap<String, Witness>,
    pub rejected_pairs: BTreeMap<String, usize>,
    pub rejected_milestones: BTreeMap<String, serde_json::Value>,
    pub ik_rejections: usize,
    pub rejected_stages: BTreeMap<String, usize>,
}
/// Reproducible branch/placement evidence, never an executable partial plan.
#[derive(Clone, Serialize)]
pub struct Witness {
    chassis: Pose,
    grasp_roll_rad: f64,
    rod_spin_rad: f64,
    grasp_axial_m: f64,
    joints_rad: [f64; 6],
    camera_deployed: bool,
}
impl Witness {
    fn at(rig: &Rig) -> Self {
        Self {
            chassis: rig.engineer_body,
            grasp_roll_rad: rig.grasp_roll_rad,
            rod_spin_rad: rig.rod_spin_rad,
            grasp_axial_m: rig.grasp_axial_m,
            joints_rad: rig.engineer_q,
            camera_deployed: rig.camera_deployed,
        }
    }
}
#[derive(Clone, Serialize)]
pub struct Failure {
    pub reason: String,
    pub stage: Stage,
    pub collision: Option<Box<CollisionHit>>,
    pub statistics: Box<Statistics>,
}
pub struct Plan {
    pub frames: Vec<Frame>,
    pub statistics: Statistics,
}
/// Exact, in-memory certificate for an already swept transition. Never serialized.
#[derive(Clone)]
struct ValidatedTransition {
    from: Rig,
    to: Rig,
}
fn same_arc<T>(a: &Option<std::sync::Arc<T>>, b: &Option<std::sync::Arc<T>>) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(a), Some(b)) => std::sync::Arc::ptr_eq(a, b),
        _ => false,
    }
}
fn same_guard_state(a: &Rig, b: &Rig) -> bool {
    a.config == b.config
        && same_arc(&a.collision, &b.collision)
        && same_arc(&a.area, &b.area)
        && a.camera_deployed == b.camera_deployed
        && a.engineer_body == b.engineer_body
        && a.engineer_q == b.engineer_q
        && a.core_q == b.core_q
        && a.tool == b.tool
        && a.unit_pose == b.unit_pose
        && a.lift_m == b.lift_m
        && a.p_rad == b.p_rad
        && a.q_rad == b.q_rad
        && a.jaw_angles == b.jaw_angles
        && a.gripped == b.gripped
        && a.grip_requested == b.grip_requested
        && a.grasp_roll_rad == b.grasp_roll_rad
        && a.grasp_axial_m == b.grasp_axial_m
        && a.rod_spin_rad == b.rod_spin_rad
}
impl ValidatedTransition {
    fn matches(&self, from: &Rig, to: &Rig) -> bool {
        same_guard_state(&self.from, from) && same_guard_state(&self.to, to)
    }
}
#[derive(Default)]
struct IkTable {
    entries: Vec<(SerialArm, Pose, Vec<[f64; 6]>)>,
}
impl IkTable {
    fn insert(&mut self, arm: &SerialArm, target: Pose, solutions: Vec<[f64; 6]>) {
        if solutions.is_empty() {
            return;
        }
        if let Some(entry) = self
            .entries
            .iter_mut()
            .find(|(model, p, _)| model == arm && *p == target)
        {
            for q in solutions {
                if !entry.2.contains(&q) {
                    entry.2.push(q);
                }
            }
            entry.2.truncate(8);
        } else {
            if self.entries.len() >= 512 {
                self.entries.remove(0);
            }
            self.entries.push((arm.clone(), target, solutions));
        }
    }
    fn seeds(&self, arm: &SerialArm, target: Pose) -> (Vec<[f64; 6]>, bool) {
        let mut nearby: Vec<_> = self
            .entries
            .iter()
            .filter(|(model, _, _)| model == arm)
            .filter_map(|(_, p, q)| {
                let distance = position(*p).distance(position(target));
                let angle = quaternion(*p).angle_between(quaternion(target));
                (distance <= 0.15 && angle <= 0.8).then_some((
                    distance / 0.15 + angle / 0.8,
                    *p == target,
                    q,
                ))
            })
            .collect();
        nearby.sort_by(|a, b| a.0.total_cmp(&b.0));
        let exact = nearby.first().is_some_and(|v| v.1);
        (
            nearby
                .into_iter()
                .take(4)
                .flat_map(|(_, _, q)| q.iter().copied())
                .collect(),
            exact,
        )
    }
}
#[derive(Default)]
struct TransitionTable {
    entries: std::collections::HashMap<u64, Vec<std::sync::Arc<ValidatedTransition>>>,
    order: std::collections::VecDeque<u64>,
}
fn transition_key(from: &Rig, to: &Rig) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hash = std::collections::hash_map::DefaultHasher::new();
    for r in [from, to] {
        for value in r
            .engineer_q
            .into_iter()
            .chain(r.core_q)
            .chain(r.engineer_body.translation_m)
            .chain(r.engineer_body.rotation_wxyz)
            .chain(r.unit_pose.translation_m)
            .chain(r.unit_pose.rotation_wxyz)
            .chain([r.lift_m, r.p_rad, r.q_rad, r.rod_spin_rad])
        {
            value.to_bits().hash(&mut hash);
        }
        r.gripped.hash(&mut hash);
    }
    hash.finish()
}
impl TransitionTable {
    fn get(&self, from: &Rig, to: &Rig) -> Option<std::sync::Arc<ValidatedTransition>> {
        self.entries
            .get(&transition_key(from, to))?
            .iter()
            .find(|v| v.matches(from, to))
            .cloned()
    }
    fn insert(&mut self, entry: std::sync::Arc<ValidatedTransition>) {
        if self.order.len() >= 2048 {
            self.entries.remove(&self.order.pop_front().unwrap());
        }
        let key = transition_key(&entry.from, &entry.to);
        self.entries.entry(key).or_default().push(entry);
        self.order.push_back(key);
    }
}
struct Search {
    stats: Statistics,
    budget: usize,
    ik_table: IkTable,
    transitions: TransitionTable,
    last: Option<CollisionHit>,
    last_checked: Option<std::sync::Arc<ValidatedTransition>>,
    stage: Stage,
    signal: Option<std::sync::Arc<Signal>>,
}
#[derive(Default)]
pub(crate) struct Signal {
    pub checks: std::sync::atomic::AtomicUsize,
    pub cancelled: std::sync::atomic::AtomicBool,
}
impl Search {
    fn ik(
        &mut self,
        arm: &SerialArm,
        target: Pose,
        seeds: impl IntoIterator<Item = [f64; 6]>,
        maximum: usize,
    ) -> Vec<[f64; 6]> {
        self.stats.ik_queries += 1;
        let (warm, exact) = self.ik_table.seeds(arm, target);
        self.stats.nearby_ik_hints += usize::from(!warm.is_empty() && !exact);
        self.stats.exact_ik_hints += usize::from(exact);
        let found = ik_candidates(arm, target, warm.into_iter().chain(seeds), maximum);
        self.ik_table.insert(arm, target, found.clone());
        found
    }
    fn charge(&mut self) -> Result<(), &'static str> {
        if let Some(signal) = &self.signal {
            use std::sync::atomic::Ordering;
            if signal.cancelled.load(Ordering::Relaxed) {
                return Err("assembly planning cancelled");
            }
            signal.checks.fetch_add(1, Ordering::Relaxed);
        }
        if self.stats.collision_checks >= self.budget {
            return Err("planning collision-check budget exhausted");
        }
        self.stats.collision_checks += 1;
        Ok(())
    }
    fn check(&mut self, from: &Rig, to: Rig) -> Result<Rig, &'static str> {
        self.charge()?;
        if let Some(validation) = self.transitions.get(from, &to) {
            self.stats.collision_table_hits += 1;
            self.last_checked = Some(validation);
            let mut accepted = to;
            accepted.last_collision = None;
            return Ok(accepted);
        }
        let mut tested = from.clone();
        match tested.accept_candidate(to) {
            Ok(()) => {
                self.last_checked = Some(std::sync::Arc::new(ValidatedTransition {
                    from: from.clone(),
                    to: tested.clone(),
                }));
                self.transitions
                    .insert(self.last_checked.as_ref().unwrap().clone());
                Ok(tested)
            }
            Err(e) => {
                self.last_checked = None;
                self.last = tested.last_collision;
                self.count_hit();
                Err(e)
            }
        }
    }
    fn frame(&self, r: &Rig, p: [f64; 3], a: [f64; 3], stage: Stage, free: bool) -> Frame {
        let mut frame = sample_frame(r, p, a, stage, free);
        frame.validation = self.last_checked.clone();
        frame
    }
    fn count_hit(&mut self) {
        *self
            .stats
            .rejected_stages
            .entry(format!("{:?}", self.stage))
            .or_default() += 1;
        if let Some(h) = &self.last {
            *self
                .stats
                .rejected_pairs
                .entry(format!("{} / {}", h.first, h.second))
                .or_default() += 1;
        }
    }
    fn failure(&self, reason: &str) -> Failure {
        let stage = [
            Stage::Settle,
            Stage::Align,
            Stage::Flip,
            Stage::Lift,
            Stage::Insert,
            Stage::Grasp,
            Stage::Approach,
        ]
        .into_iter()
        .find(|s| {
            self.stats.rejected_stages.contains_key(&format!("{s:?}"))
                || self
                    .stats
                    .rejected_stages
                    .contains_key(&format!("{s:?} IK"))
                || self
                    .stats
                    .rejected_stages
                    .contains_key(&format!("{s:?} support"))
        })
        .unwrap_or(self.stage);
        Failure {
            reason: reason.into(),
            stage,
            collision: if stage == self.stage {
                self.last.clone().map(Box::new)
            } else {
                None
            },
            statistics: Box::new(self.stats.clone()),
        }
    }
}
fn goals(d: u8, q: f64, logical: [f64; 3], angles: [f64; 3]) -> Vec<(Stage, [f64; 3], [f64; 3])> {
    let mut result = vec![];
    if logical[0] > super::ENTRANCE_X + 1e-9 {
        result.push((Stage::Approach, [super::ENTRANCE_X, 0.2, 0.6], [0.; 3]));
    }
    if logical[0] > super::SOCKET[0] + 1e-9 {
        result.push((Stage::Insert, super::SOCKET, [0.; 3]));
    }
    if d >= 2 && logical[2] < 0.7 - 1e-9 {
        result.push((Stage::Lift, [-0.05, 0.2, 0.7], [0.; 3]));
    }
    if d >= 3 && angles[0] < std::f64::consts::FRAC_PI_2 - 1e-9 {
        result.push((
            Stage::Flip,
            [-0.05, 0.2, 0.7],
            [std::f64::consts::FRAC_PI_2, 0., 0.],
        ));
    }
    if d >= 3 && (angles[1] - q).abs() > 1e-9 {
        result.push((
            Stage::Align,
            [-0.05, 0.2, 0.7],
            [std::f64::consts::FRAC_PI_2, q, 0.],
        ));
    }
    result
}
/// A Cartesian-constrained segment. IK endpoints alone are never a connection:
/// continuation must follow the guide, and execution separately sweeps collisions.
#[derive(Clone, Copy)]
struct Segment {
    stage: Stage,
    from: ([f64; 3], [f64; 3]),
    to: ([f64; 3], [f64; 3]),
}
impl Segment {
    fn samples(self, linear_m: f64, angular_rad: f64) -> usize {
        (0..3)
            .map(|i| (self.to.0[i] - self.from.0[i]).abs() / linear_m)
            .chain((0..3).map(|i| (self.to.1[i] - self.from.1[i]).abs() / angular_rad))
            .fold(0., f64::max)
            .ceil()
            .max(1.) as usize
    }
    fn point(self, t: f64) -> ([f64; 3], [f64; 3]) {
        (
            std::array::from_fn(|i| self.from.0[i] + (self.to.0[i] - self.from.0[i]) * t),
            std::array::from_fn(|i| self.from.1[i] + (self.to.1[i] - self.from.1[i]) * t),
        )
    }
}
fn segments(d: u8, q: f64, logical: [f64; 3], angles: [f64; 3]) -> Vec<Segment> {
    let mut from = (logical, angles);
    goals(d, q, logical, angles)
        .into_iter()
        .map(|(stage, p, a)| {
            let segment = Segment {
                stage,
                from,
                to: (p, a),
            };
            from = (p, a);
            segment
        })
        .collect()
}
/// Discover initial branches by solving each constrained segment backwards.
/// No branch jump, release, fixture motion or collision waiver is introduced.
#[cfg(test)]
fn reverse_roots(
    setup: &Rig,
    task: &[Segment],
    endpoints: &[[f64; 6]],
    stats: &mut Statistics,
) -> Vec<[f64; 6]> {
    reverse_route_roots(setup, task, endpoints, stats, ChassisRoute::default())
}
fn reverse_route_roots(
    setup: &Rig,
    task: &[Segment],
    endpoints: &[[f64; 6]],
    stats: &mut Statistics,
    route: ChassisRoute,
) -> Vec<[f64; 6]> {
    let mut roots = vec![];
    for &q in endpoints {
        let mut probe = at(setup.clone(), q);
        let mut connected = true;
        for &segment in task.iter().rev() {
            let n = route.samples(segment, 0.005, 0.02);
            for i in (0..n).rev() {
                let (p, a) = segment.point(i as f64 / n as f64);
                if let Err(reason) = route_step(&mut probe, setup.engineer_body, route, p, a) {
                    if reason == "chassis route has no level support" {
                        stats.unsupported_route_samples += 1;
                    }
                    connected = false;
                    break;
                }
            }
            if !connected {
                break;
            }
            *stats
                .connected_segments
                .entry(format!("{:?} reverse IK", segment.stage))
                .or_default() += 1;
        }
        if connected
            && !roots
                .iter()
                .any(|r: &[f64; 6]| (0..6).all(|i| (r[i] - probe.engineer_q[i]).abs() < 0.001))
        {
            roots.push(probe.engineer_q);
        }
    }
    stats.reverse_roots += roots.len();
    roots
}
/// Prescribed, preselected workbench route; not a drivetrain controller.
#[derive(Clone, Copy, Default, Serialize)]
pub struct ChassisRoute {
    lift_offset_m: [f64; 2],
    flip_offset_m: [f64; 2],
    flip_yaw_rad: f64,
    initial_spin_rad: f64,
    approach_spin_rad: f64,
    insert_spin_rad: f64,
    align_spin_rad: f64,
    alignment_target_rad: f64,
    lift_spin_rad: f64,
    flip_spin_rad: f64,
    /// Optional spin at the midpoint of P motion; supports twisting then unwinding.
    flip_mid_spin_rad: Option<f64>,
}
impl ChassisRoute {
    fn spin_deltas(self) -> [f64; 5] {
        [
            self.approach_spin_rad,
            self.insert_spin_rad,
            self.lift_spin_rad,
            self.flip_spin_rad,
            self.align_spin_rad,
        ]
    }
    /// Progress coordinates for the five moving stages, independent of pose azimuth.
    fn phases(self, p: [f64; 3], a: [f64; 3]) -> [f64; 5] {
        [
            ((0.09 - p[0]) / 0.04).clamp(0., 1.),
            ((0.05 - p[0]) / 0.1).clamp(0., 1.),
            ((p[2] - 0.6) / 0.1).clamp(0., 1.),
            (a[0] / std::f64::consts::FRAC_PI_2).clamp(0., 1.),
            if self.alignment_target_rad.abs() > 1e-9 {
                (a[1] / self.alignment_target_rad).clamp(0., 1.)
            } else {
                0.
            },
        ]
    }
    fn anchored(mut self, spin: f64, p: [f64; 3], a: [f64; 3], target_q: f64) -> Self {
        self.alignment_target_rad = target_q;
        self.initial_spin_rad = 0.;
        self.initial_spin_rad = spin - self.spin(p, a);
        self
    }
    fn deltas(self) -> [f64; 5] {
        [
            self.approach_spin_rad,
            self.insert_spin_rad,
            self.lift_spin_rad,
            self.flip_spin_rad,
            self.align_spin_rad,
        ]
    }
    fn spin(self, logical: [f64; 3], angles: [f64; 3]) -> f64 {
        let phases = self.phases(logical, angles);
        let mut value = self.initial_spin_rad;
        for (i, (t, delta)) in phases.into_iter().zip(self.deltas()).enumerate() {
            value += match (i, self.flip_mid_spin_rad) {
                (3, Some(mid)) if t <= 0.5 => 2. * t * mid,
                (3, Some(mid)) => mid + (2. * t - 1.) * (delta - mid),
                _ => t * delta,
            };
        }
        value
    }
    fn samples(self, segment: Segment, linear_m: f64, angular_rad: f64) -> usize {
        let from = self.phases(segment.from.0, segment.from.1);
        let to = self.phases(segment.to.0, segment.to.1);
        let spin_bound: f64 = (0..5)
            .map(|i| {
                let delta = self.deltas()[i];
                let slope = if i == 3 {
                    self.flip_mid_spin_rad
                        .map_or(delta.abs(), |mid| 2. * mid.abs().max((delta - mid).abs()))
                } else {
                    delta.abs()
                };
                (to[i] - from[i]).abs() * slope
            })
            .sum();
        segment
            .samples(linear_m, angular_rad)
            .max((spin_bound / angular_rad).ceil() as usize)
    }
    fn place(self, rig: &mut Rig, origin: Pose, p: [f64; 3], a: [f64; 3]) {
        rig.engineer_body = self.body(origin, p, a);
        rig.rod_spin_rad = self.spin(p, a);
    }

    fn body(self, origin: Pose, logical: [f64; 3], angles: [f64; 3]) -> Pose {
        let lift = ((logical[2] - 0.6) / 0.1).clamp(0., 1.);
        let flip = (angles[0] / std::f64::consts::FRAC_PI_2).clamp(0., 1.);
        pose(
            position(origin)
                + glam::DVec3::new(
                    lift * self.lift_offset_m[0] + flip * self.flip_offset_m[0],
                    lift * self.lift_offset_m[1] + flip * self.flip_offset_m[1],
                    0.,
                ),
            quaternion(origin) * DQuat::from_rotation_z(flip * self.flip_yaw_rad),
        )
    }
}
fn route_step(
    rig: &mut Rig,
    origin: Pose,
    route: ChassisRoute,
    p: [f64; 3],
    a: [f64; 3],
) -> Result<(), &'static str> {
    route.place(rig, origin, p, a);
    if !rig.support_parking() {
        return Err("chassis route has no level support");
    }
    rig.solve_tool(p, a)
}
fn chassis_cadence(from: Pose, to: Pose) -> Result<(), &'static str> {
    let dt = rm_simulator_physics::tick_ns() as f64 * 1e-9;
    if !to
        .translation_m
        .into_iter()
        .chain(to.rotation_wxyz)
        .all(f64::is_finite)
        || (to.rotation_wxyz.into_iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-8
    {
        return Err("invalid planned chassis pose");
    }
    if position(from).distance(position(to)) > CHASSIS_SPEED_M_S * dt + 1e-9
        || quaternion(from).angle_between(quaternion(to)) > CHASSIS_YAW_RAD_S * dt + 1e-9
    {
        return Err("planned chassis motion exceeds workbench cadence");
    }
    Ok(())
}
fn at(mut rig: Rig, q: [f64; 6]) -> Rig {
    rig.engineer_q = q;
    rig.tool = compose(rig.engineer_body, rig.config.engineer.tool_pose(q));
    if rig.gripped {
        rig.unit_pose = rig.tool_unit_pose();
    }
    rig
}
fn sample_frame(r: &Rig, logical: [f64; 3], angles: [f64; 3], stage: Stage, free: bool) -> Frame {
    Frame {
        chassis: r.engineer_body,
        rod_spin_rad: r.rod_spin_rad,
        q: r.engineer_q,
        logical,
        angles,
        stage,
        free,
        close: r.grip_requested,
        validation: None,
    }
}
pub(super) fn branch_seeds(arm: &SerialArm) -> Vec<[f64; 6]> {
    let mut seeds = initial_seeds();
    for n in 1..=48 {
        seeds.push(std::array::from_fn(|i| {
            let base = [2, 3, 5, 7, 11, 13][i];
            let mut x = n;
            let mut f = 1.;
            let mut u = 0.;
            while x > 0 {
                f /= base as f64;
                u += f * (x % base) as f64;
                x /= base;
            }
            let [a, b] = arm.axes[i].range_rad;
            a + (b - a) * u
        }));
    }
    seeds
}
impl Rig {
    /// Validate and apply one precomputed frame through the ordinary atomic guard.
    pub fn follow_plan(&mut self, f: &Frame) -> Result<(), &'static str> {
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
                > super::RATE * rm_simulator_physics::tick_ns() as f64 * 1e-9 + 1e-9
        {
            return Err("planned rod spin exceeds workbench cadence");
        }
        next.rod_spin_rad = f.rod_spin_rad;
        next.advance_gripper(f.close, rm_simulator_physics::tick_ns() as f64 * 1e-9);
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
    /// Search root IK branches and validate the complete task before returning a plan.
    /// Fixture preparation may change initial parking/Core joints, never a running attempt.
    pub fn plan_assembly(
        &self,
        difficulty: u8,
        target_q: f64,
        logical: [f64; 3],
        angles: [f64; 3],
        prepare: bool,
        budget: usize,
    ) -> Result<(Rig, Plan), Failure> {
        let started = std::time::Instant::now();
        let mut result =
            self.plan_assembly_inner(difficulty, target_q, logical, angles, prepare, budget, None);
        let elapsed = started.elapsed().as_secs_f64();
        match &mut result {
            Ok((_, plan)) => plan.statistics.planning_elapsed_s = elapsed,
            Err(failure) => failure.statistics.planning_elapsed_s = elapsed,
        }
        result
    }
    pub(crate) fn plan_assembly_observed(
        &self,
        difficulty: u8,
        target_q: f64,
        budget: usize,
        signal: std::sync::Arc<Signal>,
    ) -> Result<(Rig, Plan), Failure> {
        self.plan_assembly_inner(
            difficulty,
            target_q,
            [0.09, 0.2, 0.6],
            [0.; 3],
            true,
            budget,
            Some(signal),
        )
    }
    pub(crate) fn plan_runtime_segment(
        &self,
        difficulty: u8,
        twist: Option<f64>,
        budget: usize,
        signal: std::sync::Arc<Signal>,
    ) -> Result<(Rig, Plan), Failure> {
        let result = if let Some(target) = twist {
            self.plan_attached_twist(target, budget, signal)
        } else {
            // Initial Q is physically zero. The future LED target does not
            // exist yet and cannot influence grasp, branch or route selection.
            self.plan_assembly_observed(difficulty, 0., budget, signal)
        };
        result.map(|(start, mut plan)| {
            if difficulty == 3 && twist.is_none() {
                plan.frames.retain(|f| f.stage != Stage::Settle);
            }
            (start, plan)
        })
    }
    fn plan_attached_twist(
        &self,
        target: f64,
        budget: usize,
        signal: std::sync::Arc<Signal>,
    ) -> Result<(Rig, Plan), Failure> {
        let mut search = Search {
            stats: Statistics::default(),
            budget,
            ik_table: IkTable::default(),
            transitions: TransitionTable::default(),
            last: None,
            last_checked: None,
            stage: Stage::Align,
            signal: Some(signal),
        };
        if !target.is_finite()
            || target.abs() > std::f64::consts::FRAC_PI_2
            || !self.gripped
            || (self.p_rad - std::f64::consts::FRAC_PI_2).abs() > 1e-5
        {
            return Err(
                search.failure("Attached Q planning requires a revealed target and full P flip")
            );
        }
        let dt = rm_simulator_physics::tick_ns() as f64 * 1e-9;
        let half = std::f64::consts::FRAC_PI_2;
        // Once attached, stay on the actual branch. Short local continuation
        // avoids a global grasp/parking/end-branch search that cannot be applied
        // without releasing the Unit. Rod clocking may lead or accompany Q.
        for (pre, delta) in [
            (0., 0.),
            (0., half),
            (0., -half),
            (half / 2., 0.),
            (-half / 2., 0.),
            (half, 0.),
            (-half, 0.),
            (0., half / 2.),
            (0., -half / 2.),
            (half, -half),
            (-half, half),
            (half, half),
            (-half, -half),
            (0., 2. * half),
            (0., -2. * half),
        ] {
            let mut state = self.clone();
            let mut frames = Vec::new();
            let mut clear = true;
            search.stats.candidates += 1;
            for (q0, q1, s0, s1) in [
                (
                    self.q_rad,
                    self.q_rad,
                    self.rod_spin_rad,
                    self.rod_spin_rad + pre,
                ),
                (
                    self.q_rad,
                    target,
                    self.rod_spin_rad + pre,
                    self.rod_spin_rad + pre + delta,
                ),
            ] {
                let count =
                    (((q1 - q0).abs().max((s1 - s0).abs())) / (super::RATE * dt)).ceil() as usize;
                for i in 1..=count {
                    let t = i as f64 / count as f64;
                    let angles = [self.p_rad, q0 + (q1 - q0) * t, 0.];
                    let logical = [-0.05, 0.2, 0.6 + self.lift_m];
                    let mut next = state.clone();
                    next.rod_spin_rad = s0 + (s1 - s0) * t;
                    if let Err(error) = next.solve_tool(logical, angles) {
                        search.stats.ik_rejections += 1;
                        *search
                            .stats
                            .rejected_stages
                            .entry("Align IK".into())
                            .or_default() += 1;
                        if search.stats.collision_checks >= budget {
                            return Err(search.failure(error));
                        }
                        clear = false;
                        break;
                    }
                    match search.check(&state, next) {
                        Ok(next) => state = next,
                        Err(error) if search.stats.collision_checks >= budget => {
                            return Err(search.failure(error));
                        }
                        Err(_) => {
                            clear = false;
                            break;
                        }
                    }
                    frames.push(search.frame(&state, logical, angles, Stage::Align, false));
                }
                if !clear {
                    break;
                }
            }
            if clear {
                let logical = [-0.05, 0.2, 0.6 + self.lift_m];
                let angles = [self.p_rad, target, 0.];
                for _ in 0..64 {
                    frames.push(search.frame(&state, logical, angles, Stage::Settle, false));
                }
                search.stats.frames = frames.len();
                return Ok((
                    self.clone(),
                    Plan {
                        frames,
                        statistics: search.stats,
                    },
                ));
            }
        }
        // A sparse Q/clock graph can route around obstacles that no straight
        // pre-twist/profile reaches. These are search waypoints only; every
        // selected route is then re-solved and checked at execution cadence.
        struct Node {
            rig: Rig,
            q: usize,
            clock: i32,
            parent: usize,
        }
        let q_steps = ((target - self.q_rad).abs() / 5f64.to_radians())
            .ceil()
            .max(1.) as usize;
        let logical = [-0.05, 0.2, 0.6 + self.lift_m];
        let mut nodes = vec![Node {
            rig: self.clone(),
            q: 0,
            clock: 0,
            parent: 0,
        }];
        let mut open = vec![0usize];
        let mut cells: BTreeMap<(usize, i32), Vec<[f64; 6]>> = BTreeMap::new();
        cells.insert((0, 0), vec![self.engineer_q]);
        while !open.is_empty() && nodes.len() < 384 && search.stats.collision_checks < budget {
            let at = open
                .iter()
                .enumerate()
                .min_by_key(|(_, n)| {
                    (q_steps - nodes[**n].q) * 100 + nodes[**n].clock.unsigned_abs() as usize
                })
                .unwrap()
                .0;
            let index = open.swap_remove(at);
            let node = &nodes[index];
            if node.q == q_steps {
                let mut path = vec![index];
                let mut k = index;
                while k != 0 {
                    k = nodes[k].parent;
                    path.push(k);
                }
                path.reverse();
                let mut state = self.clone();
                let mut frames = vec![];
                let mut clear = true;
                for edge in path.windows(2) {
                    let a = &nodes[edge[0]];
                    let b = &nodes[edge[1]];
                    let q0 = self.q_rad + (target - self.q_rad) * a.q as f64 / q_steps as f64;
                    let q1 = self.q_rad + (target - self.q_rad) * b.q as f64 / q_steps as f64;
                    let s0 = self.rod_spin_rad + a.clock as f64 * 15f64.to_radians();
                    let s1 = self.rod_spin_rad + b.clock as f64 * 15f64.to_radians();
                    let n = ((q1 - q0).abs().max((s1 - s0).abs()) / (super::RATE * dt))
                        .ceil()
                        .max(1.) as usize;
                    for i in 1..=n {
                        let t = i as f64 / n as f64;
                        let angles = [self.p_rad, q0 + (q1 - q0) * t, 0.];
                        let mut next = state.clone();
                        next.rod_spin_rad = s0 + (s1 - s0) * t;
                        if next.solve_tool(logical, angles).is_err() {
                            clear = false;
                            break;
                        }
                        match search.check(&state, next) {
                            Ok(next) => state = next,
                            Err(_) => {
                                clear = false;
                                break;
                            }
                        }
                        frames.push(search.frame(&state, logical, angles, Stage::Align, false));
                    }
                    if !clear {
                        break;
                    }
                }
                if clear {
                    for _ in 0..64 {
                        frames.push(search.frame(
                            &state,
                            logical,
                            [self.p_rad, target, 0.],
                            Stage::Settle,
                            false,
                        ));
                    }
                    search.stats.frames = frames.len();
                    return Ok((
                        self.clone(),
                        Plan {
                            frames,
                            statistics: search.stats,
                        },
                    ));
                }
                continue;
            }
            let q = node.q as i32;
            let clock = node.clock;
            let from = node.rig.clone();
            for (next_q, next_clock) in [
                (q + 1, clock),
                (q + 1, clock + 1),
                (q + 1, clock - 1),
                (q, clock + 1),
                (q, clock - 1),
                (q - 1, clock),
            ] {
                if !(0..=q_steps as i32).contains(&next_q) || !(-12..=12).contains(&next_clock) {
                    continue;
                }
                let key = (next_q as usize, next_clock);
                if cells.get(&key).is_some_and(|branches| branches.len() >= 3) {
                    continue;
                }
                let mut next = from.clone();
                next.rod_spin_rad = self.rod_spin_rad + next_clock as f64 * 15f64.to_radians();
                let angles = [
                    self.p_rad,
                    self.q_rad + (target - self.q_rad) * next_q as f64 / q_steps as f64,
                    0.,
                ];
                if next.solve_tool(logical, angles).is_err() {
                    continue;
                }
                if cells.get(&key).is_some_and(|branches| {
                    branches
                        .iter()
                        .any(|q| (0..6).all(|j| (q[j] - next.engineer_q[j]).abs() < 0.1))
                }) {
                    continue;
                }
                let Ok(next) = search.check(&from, next) else {
                    continue;
                };
                cells.entry(key).or_default().push(next.engineer_q);
                let id = nodes.len();
                nodes.push(Node {
                    rig: next,
                    q: key.0,
                    clock: next_clock,
                    parent: index,
                });
                open.push(id);
                if search.stats.collision_checks >= budget {
                    break;
                }
            }
        }
        Err(search.failure("No attached Q/rod continuation found within the local search budget"))
    }
    pub(crate) fn runtime_endpoint(&self, plan: &Plan) -> Result<Rig, &'static str> {
        let mut end = self.clone();
        for frame in &plan.frames {
            end.follow_plan(frame)?;
        }
        Ok(end)
    }
    #[allow(clippy::too_many_arguments)]
    fn plan_assembly_inner(
        &self,
        difficulty: u8,
        target_q: f64,
        logical: [f64; 3],
        angles: [f64; 3],
        prepare: bool,
        budget: usize,
        signal: Option<std::sync::Arc<Signal>>,
    ) -> Result<(Rig, Plan), Failure> {
        let mut search = Search {
            stats: Statistics::default(),
            budget,
            ik_table: IkTable::default(),
            transitions: TransitionTable::default(),
            last: None,
            last_checked: None,
            stage: Stage::Approach,
            signal,
        };
        for (target, q) in self.ik_hints.iter() {
            search
                .ik_table
                .insert(&self.config.engineer, *target, vec![*q]);
        }
        if !(1..=3).contains(&difficulty) || !target_q.is_finite() || budget == 0 {
            return Err(search.failure("invalid planner request"));
        }
        if !logical.into_iter().chain(angles).all(f64::is_finite)
            || (logical[1] - 0.2).abs() > 1e-8
            || angles[2].abs() > 1e-8
            || logical[0] < -0.05 - 1e-8
            || !(0.6 - 1e-8..=0.7 + 1e-8).contains(&logical[2])
            || !(0.0..=std::f64::consts::FRAC_PI_2 + 1e-8).contains(&angles[0])
            || (prepare && (self.gripped || logical != [0.09, 0.2, 0.6] || angles != [0.; 3]))
        {
            return Err(search.failure(
                "planning requires an aligned guide state; fixture setup is initial-only",
            ));
        }
        let mut setups = vec![self.clone()];
        if prepare {
            // Preserve the selected socket pose. Only numerical Core IK branches change.
            // Live Core positioning has finished before this robot search.
            // Its selected numerical IK branch is physical, not a setup freedom.
            let mut clear_core = self.live_parking.then(|| self.clone());
            let core_candidates = if self.live_parking {
                vec![]
            } else {
                search.ik(&self.config.core, self.socket, initial_seeds(), 8)
            };
            for q in core_candidates {
                let mut trial = self.clone();
                trial.core_q = q;
                search.charge().map_err(|e| search.failure(e))?;
                let contacts = trial.collision_contacts().map_err(|e| search.failure(e))?;
                if !contacts.iter().any(|h| {
                    [&h.first, &h.second]
                        .into_iter()
                        .all(|n| !n.starts_with("engineer/") && n.as_str() != "unit/unit")
                }) {
                    clear_core = Some(trial);
                    break;
                }
            }
            let Some(base) = clear_core else {
                return Err(search.failure("no clear Core positioning branch within setup budget"));
            };
            setups.clear();
            let mut parking = vec![
                (0., 0.),
                (0.15, 0.),
                (-0.15, 0.),
                (0.3, 0.),
                (-0.3, 0.),
                (0., 0.3),
                (0., -0.3),
                (0.15, 0.3),
                (0.15, -0.3),
            ];
            if self.area.is_some() {
                parking.extend([
                    (0.45, 0.),
                    (0.6, 0.),
                    (0.45, 0.3),
                    (0.45, -0.3),
                    (0.6, 0.3),
                    (0.6, -0.3),
                ]);
            }
            if self.area.is_some() {
                for forward in [-0.1, 0.05, 0.2, 0.35, 0.5] {
                    for lateral in [-0.45, -0.15, 0.15, 0.45] {
                        parking.push((forward, lateral));
                    }
                }
            }
            let mut yaws = vec![0., 90., -90., 180., 45., -45., 135., -135.];
            if self.area.is_some() {
                yaws.extend([
                    15., -15., 30., -30., 60., -60., 75., -75., 105., -105., 120., -120., 150.,
                    -150., 165., -165.,
                ]);
            }
            if self.fixed_chassis {
                parking = vec![(0., 0.)];
                yaws = vec![0.];
            }
            for (forward, lateral) in parking {
                for &yaw_degrees in &yaws {
                    let yaw = yaw_degrees * std::f64::consts::PI / 180.;
                    for roll in [
                        base.grasp_roll_rad,
                        -base.grasp_roll_rad,
                        0.,
                        std::f64::consts::PI,
                        std::f64::consts::FRAC_PI_4,
                        -std::f64::consts::FRAC_PI_4,
                        3. * std::f64::consts::FRAC_PI_4,
                        -3. * std::f64::consts::FRAC_PI_4,
                    ] {
                        let mut trial = base.clone();
                        trial.engineer_body.translation_m[0] += forward;
                        trial.engineer_body.translation_m[1] += lateral;
                        trial.engineer_body = pose(
                            position(trial.engineer_body),
                            quaternion(base.engineer_body) * DQuat::from_rotation_z(yaw),
                        );
                        if !trial.support_parking() {
                            continue;
                        }
                        if self.live_parking {
                            trial.engineer_body = base.engineer_body;
                        }
                        trial.grasp_roll_rad = roll;
                        // Source triangle sections preserve the 32.3 mm radial
                        // envelope through the pads' +/-11 mm span at +20 mm.
                        trial.grasp_axial_m = 0.02;
                        setups.push(trial.clone());
                        trial.grasp_axial_m = 0.;
                        setups.push(trial);
                    }
                }
            }
        }
        if self.live_entry.is_some() {
            // A preloaded Unit cannot be re-clocked or shifted in the closed
            // gripper by setup search. Those are offline fixture freedoms only.
            setups.retain(|r| {
                r.grasp_roll_rad == self.grasp_roll_rad && r.grasp_axial_m == self.grasp_axial_m
            });
        }
        if !prepare && !self.gripped {
            setups.clear();
            for roll in [
                self.grasp_roll_rad,
                -self.grasp_roll_rad,
                0.,
                std::f64::consts::PI,
                std::f64::consts::FRAC_PI_4,
                -std::f64::consts::FRAC_PI_4,
                3. * std::f64::consts::FRAC_PI_4,
                -3. * std::f64::consts::FRAC_PI_4,
            ] {
                for axial in [0.02, 0.] {
                    let mut trial = self.clone();
                    trial.grasp_roll_rad = roll;
                    trial.grasp_axial_m = axial;
                    setups.push(trial);
                }
            }
        }
        if prepare && self.area.is_some() {
            // Spread the finite budget across parking, yaw and grasp choices;
            // exhausting all branches at the first parking wastes alternatives.
            let postures = setups.into_iter().flat_map(|rig| {
                let mut deployed = rig.clone();
                deployed.camera_deployed = !rig.camera_deployed;
                if self.live_parking {
                    vec![rig]
                } else {
                    vec![rig, deployed]
                }
            });
            let mut ranked: Vec<_> = postures.enumerate().collect();
            ranked.sort_by_key(|(i, _)| (*i as u64).wrapping_mul(0x9e3779b97f4a7c15));
            setups = ranked.into_iter().map(|(_, rig)| rig).collect();
        }
        let mut routes = vec![ChassisRoute {
            initial_spin_rad: self.rod_spin_rad,
            ..Default::default()
        }];
        if prepare && !self.fixed_chassis && self.area.is_some() && difficulty >= 2 {
            routes.extend([
                ChassisRoute {
                    lift_offset_m: [0.05, 0.],
                    flip_offset_m: [0.1, 0.],
                    flip_yaw_rad: 0.,
                    ..Default::default()
                },
                ChassisRoute {
                    lift_offset_m: [0.1, 0.],
                    ..Default::default()
                },
            ]);
        }
        if prepare && !self.fixed_chassis && self.area.is_some() && difficulty >= 3 {
            routes.extend([
                ChassisRoute {
                    flip_offset_m: [0.15, 0.],
                    ..Default::default()
                },
                ChassisRoute {
                    flip_offset_m: [0.1, 0.],
                    flip_yaw_rad: std::f64::consts::PI / 12.,
                    ..Default::default()
                },
                ChassisRoute {
                    flip_offset_m: [0.1, 0.],
                    flip_yaw_rad: -std::f64::consts::PI / 12.,
                    ..Default::default()
                },
                ChassisRoute {
                    lift_offset_m: [-0.05, 0.],
                    flip_offset_m: [0.15, 0.],
                    ..Default::default()
                },
                ChassisRoute {
                    lift_offset_m: [-0.1, 0.],
                    flip_offset_m: [0.15, 0.],
                    ..Default::default()
                },
                ChassisRoute {
                    lift_offset_m: [-0.05, 0.05],
                    flip_offset_m: [0.15, -0.05],
                    ..Default::default()
                },
                ChassisRoute {
                    lift_offset_m: [-0.05, -0.05],
                    flip_offset_m: [0.15, 0.05],
                    ..Default::default()
                },
            ]);
        }
        if self.requested_spin_deltas_rad.is_none() && self.area.is_some() && difficulty >= 3 {
            // The Unit may turn around the rod while staying seated and rigidly
            // gripped. These are continuous routes, not changes to grasp clocking.
            for spin in [
                -std::f64::consts::PI,
                -std::f64::consts::FRAC_PI_2,
                std::f64::consts::FRAC_PI_2,
                std::f64::consts::PI,
            ] {
                routes.push(ChassisRoute {
                    lift_spin_rad: spin,
                    ..Default::default()
                });
                routes.push(ChassisRoute {
                    flip_spin_rad: spin,
                    ..Default::default()
                });
            }
        }
        let linear_route_count = routes.len();
        if self.requested_spin_deltas_rad.is_none() && self.area.is_some() && difficulty >= 3 {
            let half = std::f64::consts::FRAC_PI_2;
            for (mid, end) in [
                (0., half),
                (half, half),
                (0., -half),
                (-half, -half),
                (half, 0.),
                (-half, 0.),
                (-half, half),
                (half, -half),
            ] {
                routes.push(ChassisRoute {
                    flip_mid_spin_rad: Some(mid),
                    flip_spin_rad: end,
                    ..Default::default()
                });
            }
        }
        if self.requested_spin_deltas_rad.is_none() && self.area.is_some() {
            let half = std::f64::consts::FRAC_PI_2;
            for spin in [-half, half] {
                routes.push(ChassisRoute {
                    approach_spin_rad: spin,
                    ..Default::default()
                });
                routes.push(ChassisRoute {
                    insert_spin_rad: spin,
                    ..Default::default()
                });
                if difficulty >= 2 {
                    routes.push(ChassisRoute {
                        lift_spin_rad: spin,
                        ..Default::default()
                    });
                }
                if difficulty >= 3 {
                    routes.push(ChassisRoute {
                        align_spin_rad: spin,
                        ..Default::default()
                    });
                    // Plan twist across multiple stages, not as independent endpoint choices.
                    routes.push(ChassisRoute {
                        insert_spin_rad: spin,
                        flip_spin_rad: -spin,
                        align_spin_rad: spin,
                        ..Default::default()
                    });
                    routes.push(ChassisRoute {
                        lift_spin_rad: spin,
                        flip_spin_rad: spin,
                        align_spin_rad: -spin,
                        ..Default::default()
                    });
                    routes.push(ChassisRoute {
                        approach_spin_rad: spin,
                        insert_spin_rad: -spin,
                        flip_spin_rad: spin,
                        ..Default::default()
                    });
                }
            }
            if prepare && difficulty >= 3 {
                let moving: Vec<_> = routes
                    .iter()
                    .copied()
                    .filter(|r| {
                        r.lift_offset_m != [0.; 2]
                            || r.flip_offset_m != [0.; 2]
                            || r.flip_yaw_rad != 0.
                    })
                    .collect();
                for mut route in moving {
                    route.flip_spin_rad = half;
                    routes.push(route);
                }
            }
        }
        for route in &mut routes {
            if let Some([approach, insert, lift, flip, align]) = self.requested_spin_deltas_rad {
                route.approach_spin_rad = approach;
                route.insert_spin_rad = insert;
                route.lift_spin_rad = lift;
                route.flip_spin_rad = flip;
                route.align_spin_rad = align;
                route.flip_mid_spin_rad = None;
            }
            // A resumed plan starts at the current clocking, not at a profile's
            // hypothetical value before insertion/lift/flip had begun.
            *route = route.anchored(self.rod_spin_rad, logical, angles, target_q);
        }
        let mut schedule: Vec<_> = (0..setups.len())
            .flat_map(|i| (0..routes.len()).map(move |j| (i, j)))
            .enumerate()
            .collect();
        if routes.len() > 1 {
            schedule.sort_by_key(|(_, (i, j))| {
                (
                    self.preferred_spin_deltas_rad
                        .is_some_and(|hint| routes[*j].spin_deltas() != hint),
                    route_order(*i, *j, linear_route_count, routes.len()),
                )
            });
        }
        for (_, (setup_index, route_index)) in schedule {
            if search
                .signal
                .as_ref()
                .is_some_and(|s| s.cancelled.load(std::sync::atomic::Ordering::Relaxed))
            {
                return Err(Failure {
                    reason: "assembly planning cancelled".into(),
                    stage: Stage::Approach,
                    collision: None,
                    statistics: Box::new(search.stats),
                });
            }
            if routes.len() > linear_route_count
                && route_index < linear_route_count
                && search.stats.collision_checks >= linear_search_budget(budget)
            {
                continue;
            }
            let setup = &setups[setup_index];
            let route = routes[route_index];
            *search.stats.route_trials.entry(route_index).or_default() += 1;
            search.stats.tested_setups += 1;
            if search.stats.tested_setups.is_multiple_of(256) {
                eprintln!(
                    "assembly search: {} setups, {} reverse-derived seeds, {} guarded checks",
                    search.stats.tested_setups,
                    search.stats.reverse_roots,
                    search.stats.collision_checks
                );
            }
            let target = setup.target(logical, angles);
            let seeds = branch_seeds(&setup.config.engineer);
            let endpoint = goals(difficulty, target_q, logical, angles).last().copied();
            let goal_seeds = endpoint
                .map(|(_, p, a)| {
                    let mut endpoint = setup.clone();
                    route.place(&mut endpoint, setup.engineer_body, p, a);
                    if !endpoint.support_parking() {
                        return vec![];
                    }
                    search.ik(
                        &setup.config.engineer,
                        compose(inverse(endpoint.engineer_body), endpoint.target(p, a)),
                        seeds.clone(),
                        8,
                    )
                })
                .unwrap_or_default();
            let task = segments(difficulty, target_q, logical, angles);
            let mut backward = vec![];
            if self.area.is_some() {
                backward = reverse_route_roots(setup, &task, &goal_seeds, &mut search.stats, route);
                // If the complete suffix cannot yet connect, solve an earlier
                // junction independently and propagate its branch back to the
                // grasp. Forward continuation then tries to connect the suffix.
                if backward.is_empty() {
                    for end in (1..task.len()).rev() {
                        let (p, a) = task[end - 1].to;
                        let mut junction = setup.clone();
                        route.place(&mut junction, setup.engineer_body, p, a);
                        if !junction.support_parking() {
                            continue;
                        }
                        let junctions = search.ik(
                            &setup.config.engineer,
                            compose(inverse(junction.engineer_body), junction.target(p, a)),
                            seeds.clone(),
                            4,
                        );
                        backward.extend(reverse_route_roots(
                            setup,
                            &task[..end],
                            &junctions,
                            &mut search.stats,
                            route,
                        ));
                        if !backward.is_empty() {
                            break;
                        }
                    }
                }
            }
            let mut candidates = search.ik(
                &setup.config.engineer,
                compose(inverse(setup.engineer_body), target),
                backward
                    .into_iter()
                    .chain(std::iter::once(setup.engineer_q))
                    .chain(goal_seeds)
                    .chain(seeds),
                12,
            );
            if let Some((actual, _)) = self.live_entry {
                candidates.sort_by(|a, b| {
                    let distance =
                        |q: &[f64; 6]| (0..6).map(|i| (q[i] - actual[i]).powi(2)).sum::<f64>();
                    distance(a).total_cmp(&distance(b))
                });
            }
            for q in candidates {
                search.stats.candidates += 1;
                search.stage = Stage::Approach;
                let mut root = at(setup.clone(), q);
                if prepare {
                    root.initial_engineer_q = q;
                    root.initial_engineer_body = root.engineer_body;
                    root.unit_pose = root.tool_unit_pose();
                    root.last_collision = None;
                }
                if search.stats.collision_checks >= budget {
                    return Err(search.failure("planning collision-check budget exhausted"));
                }
                // Cheap milestone rejection precedes the full 128 Hz validation.
                let mut probe = root.clone();
                probe.gripped = true;
                probe.grip_requested = true;
                probe.jaw_angles = probe
                    .config
                    .engineer_jaws
                    .iter()
                    .map(|j| j.closed_rad)
                    .collect();
                let mut viable = true;
                for &segment in &task {
                    let stage = segment.stage;
                    search.stage = stage;
                    let beginning = probe.clone();
                    let count = route.samples(segment, 0.005, 0.02);
                    for i in 1..=count {
                        let (x, r) = segment.point(i as f64 / count as f64);
                        if let Err(reason) = route_step(&mut probe, root.engineer_body, route, x, r)
                        {
                            if reason == "chassis route has no level support" {
                                search.stats.unsupported_route_samples += 1;
                            } else {
                                search.stats.ik_rejections += 1;
                            }
                            *search
                                .stats
                                .rejected_stages
                                .entry(format!(
                                    "{:?} {}",
                                    stage,
                                    if reason == "chassis route has no level support" {
                                        "support"
                                    } else {
                                        "IK"
                                    }
                                ))
                                .or_default() += 1;
                            viable = false;
                            break;
                        }
                    }
                    if !viable {
                        break;
                    }
                    search.charge().map_err(|e| search.failure(e))?;
                    match probe.collision_contacts() {
                        Ok(h) if h.is_empty() => {
                            search
                                .stats
                                .milestone_witnesses
                                .entry(format!("{stage:?}"))
                                .or_insert_with(|| Witness::at(&probe));
                        }
                        Ok(h) => {
                            search.last = h.into_iter().next();
                            if let Some(hit) = &search.last {
                                search.stats.rejected_milestones.entry(format!("{} / {}",hit.first,hit.second)).or_insert_with(|| serde_json::json!({"stage":stage,"hit":hit,"q_rad":probe.engineer_q,"frames":probe.world_frames(),"query":"static-endpoint"}));
                            }
                            search.count_hit();
                            viable = false;
                            break;
                        }
                        Err(e) => return Err(search.failure(e)),
                    }
                    if self.area.is_some() && search.stats.segment_checks < PREFIX_AUDIT_CHECKS {
                        // Validate and retain each prefix independently before
                        // attempting the next segment. These are swept source
                        // paths, not merely reachable milestone endpoints.
                        let mut prefix = beginning;
                        let mut complete = true;
                        for i in 1..=count {
                            if search.stats.segment_checks >= PREFIX_AUDIT_CHECKS {
                                complete = false;
                                break;
                            }
                            search.stats.segment_checks += 1;
                            let (p, a) = segment.point(i as f64 / count as f64);
                            let mut next = prefix.clone();
                            if route_step(&mut next, root.engineer_body, route, p, a).is_err() {
                                viable = false;
                                break;
                            }
                            match search.check(&prefix, next) {
                                Ok(accepted) => prefix = accepted,
                                Err(e) if search.stats.collision_checks >= budget => {
                                    return Err(search.failure(e));
                                }
                                Err(_) => {
                                    viable = false;
                                    break;
                                }
                            }
                        }
                        if !viable {
                            break;
                        }
                        if complete {
                            probe = prefix;
                            search
                                .stats
                                .coarse_segment_witnesses
                                .entry(format!("{stage:?}"))
                                .or_insert_with(|| Witness::at(&probe));
                        }
                    }
                }
                if !viable {
                    continue;
                }
                match build(
                    self,
                    &root,
                    difficulty,
                    target_q,
                    logical,
                    angles,
                    prepare,
                    &mut search,
                    route,
                ) {
                    Ok((start, frames)) => {
                        search.stats.frames = frames.len();
                        search.stats.selected_chassis = Some(start.engineer_body);
                        search.stats.selected_route = Some(route);
                        search.stats.setup_changed = prepare
                            && (start.engineer_body != self.engineer_body
                                || start.engineer_q != self.engineer_q
                                || start.core_q != self.core_q
                                || start.grasp_roll_rad != self.grasp_roll_rad
                                || start.grasp_axial_m != self.grasp_axial_m
                                || start.camera_deployed != self.camera_deployed);
                        return Ok((
                            start,
                            Plan {
                                frames,
                                statistics: search.stats,
                            },
                        ));
                    }
                    Err(e) if search.stats.collision_checks >= budget => {
                        return Err(search.failure(e));
                    }
                    Err(_) => {}
                }
            }
        }
        Err(search
            .failure("no complete collision-free branch/path found within bounded setup search"))
    }
}
// Reserve enough queries for a full 128 Hz candidate, even in smaller sweeps.
fn linear_search_budget(budget: usize) -> usize {
    budget - (budget / 5).max(1200).min(budget / 2)
}
// Keep earlier linear-route searches reproducible as nonlinear families grow.
fn route_order(setup: usize, route: usize, linear: usize, total: usize) -> (bool, u64) {
    let nonlinear = route >= linear;
    let ordinal = if nonlinear {
        setup * (total - linear) + route - linear
    } else {
        setup * linear + route
    };
    (nonlinear, (ordinal as u64).wrapping_mul(0x9e3779b97f4a7c15))
}
// Escape the narrow grasp neighbourhood in Cartesian space before asking
// a global joint sampler to choose directions. A held payload follows the wrist;
// an ungripped offline fixture stays parked.
fn free_retreat(
    original: &Rig,
    logical: [f64; 3],
    angles: [f64; 3],
    search: &mut Search,
) -> Result<(Rig, Vec<Frame>), &'static str> {
    let rotation = quaternion(original.tool);
    let directions = if original.gripped {
        // Back out along the bore before trying sideways clearance with a load.
        [
            rotation * glam::DVec3::Z,
            glam::DVec3::Z,
            -(rotation * glam::DVec3::X),
            rotation * glam::DVec3::X,
        ]
    } else {
        [
            -(rotation * glam::DVec3::X),
            rotation * glam::DVec3::Z,
            glam::DVec3::Z,
            rotation * glam::DVec3::X,
        ]
    };
    for direction in directions {
        let mut state = original.clone();
        let mut frames = vec![];
        let mut clear = true;
        let n =
            (0.12 / (super::SPEED * rm_simulator_physics::tick_ns() as f64 * 1e-9)).ceil() as usize;
        for i in 1..=n {
            let desired = pose(
                position(original.tool) + direction * (0.12 * i as f64 / n as f64),
                rotation,
            );
            let Ok(q) = state.config.engineer.solve(
                compose(inverse(state.engineer_body), desired),
                state.engineer_q,
            ) else {
                clear = false;
                break;
            };
            match search.check(&state, at(state.clone(), q)) {
                Ok(next) => {
                    state = next;
                    frames.push(search.frame(&state, logical, angles, Stage::Approach, true));
                }
                Err(e) if search.stats.collision_checks >= search.budget => return Err(e),
                Err(_) => {
                    clear = false;
                    break;
                }
            }
        }
        if clear {
            return Ok((state, frames));
        }
    }
    Ok((original.clone(), vec![]))
}
// Prefer a loaded Cartesian clearance route over a broad six-joint search.
// Every accepted edge still passes the ordinary articulated swept guard.
fn carried_clearance_path(
    original: &Rig,
    goal: &Rig,
    search: &mut Search,
) -> Result<Option<Vec<[f64; 6]>>, &'static str> {
    for height_m in [0.08, 0.16, 0.24, 0.] {
        let mut state = original.clone();
        let mut path = vec![state.engineer_q];
        let waypoints = if height_m == 0. {
            vec![goal.tool]
        } else {
            vec![
                pose(
                    position(original.tool) + glam::DVec3::Z * height_m,
                    quaternion(original.tool),
                ),
                pose(
                    position(goal.tool) + glam::DVec3::Z * height_m,
                    quaternion(goal.tool),
                ),
                goal.tool,
            ]
        };
        let mut clear = true;
        for target in waypoints {
            let from = state.tool;
            let count = (position(from).distance(position(target)) / 0.015)
                .max(quaternion(from).angle_between(quaternion(target)) / 0.06)
                .ceil()
                .max(1.) as usize;
            for i in 1..=count {
                let t = i as f64 / count as f64;
                let tool = pose(
                    position(from).lerp(position(target), t),
                    quaternion(from).slerp(quaternion(target), t),
                );
                let Ok(q) = state.config.engineer.solve(
                    compose(inverse(state.engineer_body), tool),
                    state.engineer_q,
                ) else {
                    clear = false;
                    break;
                };
                match search.check(&state, at(state.clone(), q)) {
                    Ok(next) => {
                        state = next;
                        path.push(q);
                    }
                    Err(e) if search.stats.collision_checks >= search.budget => return Err(e),
                    Err(_) => {
                        clear = false;
                        break;
                    }
                }
            }
            if !clear {
                break;
            }
        }
        // Cartesian continuation can reach another IK branch. Only a matching
        // endpoint can join the already validated insertion/flip suffix.
        if clear && (0..6).all(|i| (state.engineer_q[i] - goal.engineer_q[i]).abs() < 1e-4) {
            path.push(goal.engineer_q);
            return Ok(Some(path));
        }
    }
    Ok(None)
}
#[allow(clippy::too_many_arguments)]
fn build(
    current: &Rig,
    root: &Rig,
    d: u8,
    q: f64,
    mut logical: [f64; 3],
    mut angles: [f64; 3],
    prepare: bool,
    search: &mut Search,
    route: ChassisRoute,
) -> Result<(Rig, Vec<Frame>), &'static str> {
    let dt = rm_simulator_physics::tick_ns() as f64 * 1e-9;
    let mut state = if prepare {
        root.clone()
    } else {
        current.clone()
    };
    state.grasp_roll_rad = root.grasp_roll_rad;
    state.grasp_axial_m = root.grasp_axial_m;
    let entry = if prepare { current.live_entry } else { None };
    if let Some((q, jaws)) = entry {
        state = at(state, q);
        state.jaw_angles = jaws.to_vec();
        state.gripped = true;
        state.grip_requested = true;
        state.unit_pose = state.tool_unit_pose();
    }
    let start = state.clone();
    let carried_entry = entry.is_some();
    let mut frames = vec![];
    if carried_entry {
        search.stage = Stage::Grasp;
        for _ in 0..128 {
            if state
                .jaw_angles
                .iter()
                .zip(&state.config.engineer_jaws)
                .all(|(a, j)| (a - j.closed_rad).abs() < 1e-10)
            {
                break;
            }
            let mut next = state.clone();
            next.advance_gripper(true, dt);
            state = search.check(&state, next)?;
            frames.push(search.frame(&state, logical, angles, Stage::Grasp, true));
        }
    }
    if (!prepare || entry.is_some()) && !state.gripped {
        search.stage = Stage::Approach;
        for _ in 0..64 {
            if state
                .jaw_angles
                .iter()
                .zip(&state.config.engineer_jaws)
                .all(|(a, j)| (a - j.open_rad).abs() < 1e-10)
            {
                break;
            }
            let mut next = state.clone();
            next.advance_gripper(false, dt);
            state = search.check(&state, next)?;
            frames.push(search.frame(&state, logical, angles, Stage::Approach, true));
        }
    }
    if (!prepare || entry.is_some())
        && (0..6)
            .map(|i| (state.engineer_q[i] - root.engineer_q[i]).abs())
            .fold(0., f64::max)
            > 1e-6
    {
        if state.gripped && !carried_entry {
            return Err("cannot change grasp branch with a carried or seated unit");
        }
        search.stage = Stage::Approach;
        let (retreated, retreat_frames) = free_retreat(&state, logical, angles, search)?;
        state = retreated;
        frames.extend(retreat_frames);
        let mut grasp = root.clone();
        grasp.gripped = carried_entry;
        grasp.grip_requested = carried_entry;
        grasp.jaw_angles = state.jaw_angles.clone();
        grasp.unit_pose = if carried_entry {
            grasp.tool_unit_pose()
        } else {
            state.unit_pose
        };
        let (pregrasp, arrival) = free_retreat(&grasp, logical, angles, search)?;
        let mut approach_joints = vec![grasp.engineer_q];
        approach_joints.extend(arrival.iter().map(|f| f.q));
        let model = state.clone();
        let cartesian = if carried_entry {
            carried_clearance_path(&model, &pregrasp, search)?
        } else {
            None
        };
        let path = if let Some(path) = cartesian {
            path
        } else {
            joint_path(
                model.config.engineer.axes.map(|a| a.range_rad),
                model.engineer_q,
                pregrasp.engineer_q,
                JointSearch {
                    // Try other endpoint branches before spending the entire
                    // global budget on a trapped loaded-arm connection.
                    samples: if carried_entry { 16 } else { 128 },
                    ..Default::default()
                },
                |a, b| {
                    // Long RRT shortcuts still use the same small articulated
                    // intervals as execution, rather than exhausting one guard call.
                    let n = ((0..6).map(|i| (b[i] - a[i]).abs()).fold(0., f64::max) / 0.1)
                        .ceil()
                        .max(1.) as usize;
                    let mut previous = at(model.clone(), a);
                    for i in 1..=n {
                        let t = i as f64 / n as f64;
                        let next = at(
                            model.clone(),
                            std::array::from_fn(|j| a[j] + (b[j] - a[j]) * t),
                        );
                        match search.check(&previous, next) {
                            Ok(accepted) => previous = accepted,
                            Err("motion blocked by collision") => return Ok(false),
                            Err(_) => return Err(PlanError::CollisionQuery),
                        }
                    }
                    Ok(true)
                },
            )
            .map_err(|_| "free approach joint search exhausted or obstructed")?
        };
        for pair in path.windows(2) {
            let n = ((0..6)
                .map(|i| (pair[1][i] - pair[0][i]).abs())
                .fold(0., f64::max)
                / (0.6 * dt))
                .ceil()
                .max(1.) as usize;
            for i in 1..=n {
                let t = i as f64 / n as f64;
                let next = at(
                    state.clone(),
                    std::array::from_fn(|j| pair[0][j] + (pair[1][j] - pair[0][j]) * t),
                );
                state = search.check(&state, next)?;
                frames.push(search.frame(&state, logical, angles, Stage::Approach, true));
            }
        }
        for q in approach_joints.into_iter().rev().skip(1) {
            state = search.check(&state, at(state.clone(), q))?;
            frames.push(search.frame(&state, logical, angles, Stage::Approach, true));
        }
    }
    search.stage = Stage::Grasp;
    for _ in 0..64 {
        if state.gripped {
            break;
        }
        let mut next = state.clone();
        next.advance_gripper(true, dt);
        state = search.check(&state, next)?;
        frames.push(search.frame(&state, logical, angles, Stage::Grasp, false));
    }
    if !state.gripped {
        return Err("gripper could not establish an aligned grasp");
    }
    for segment in segments(d, q, logical, angles) {
        let stage = segment.stage;
        search.stage = stage;
        let n = route.samples(segment, super::SPEED * dt, super::RATE * dt);
        for i in 1..=n {
            let (x, r) = segment.point(i as f64 / n as f64);
            let mut next = state.clone();
            route_step(&mut next, root.engineer_body, route, x, r)?;
            chassis_cadence(state.engineer_body, next.engineer_body)?;
            state = search.check(&state, next)?;
            frames.push(search.frame(&state, x, r, stage, false));
        }
        search
            .stats
            .swept_segment_witnesses
            .entry(format!("{stage:?}"))
            .or_insert_with(|| Witness::at(&state));
        logical = segment.to.0;
        angles = segment.to.1;
    }
    state = search.check(&state, state.clone())?;
    for _ in 0..64 {
        frames.push(search.frame(&state, logical, angles, Stage::Settle, false));
    }
    Ok((start, frames))
}

impl Rig {
    /// Retime validated motion along its existing joint path. No IK goal or
    /// endpoint is projected; slow joints lengthen the corresponding segment.
    pub(crate) fn runtime_limited_frames(
        &self,
        plan: &Plan,
    ) -> Result<Vec<rm_simulator_world::assembly::Frame>, &'static str> {
        use rm_simulator_world::assembly::{Frame as Output, JAW_SPEED_RAD_S, JOINT_SPEED_RAD_S};
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
        let dt = rm_simulator_physics::tick_ns() as f64 * 1e-9;
        let mut output = Vec::new();
        for next in source {
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
                .max(1);
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

#[cfg(test)]
impl Rig {
    pub(crate) fn test_playback_fixture() -> (Self, Plan) {
        let mut rig = super::tests::mechanism_fixture();
        let logical = [0.09, 0.2, 0.6];
        let target = rig.target(logical, [0.; 3]);
        rig.config.engineer.tool = target;
        rig.tool = target;
        rig.unit_pose = rig.tool_unit_pose();
        let frames = [Stage::Approach, Stage::Approach, Stage::Grasp]
            .into_iter()
            .map(|stage| sample_frame(&rig, logical, [0.; 3], stage, false))
            .collect();
        (
            rig,
            Plan {
                frames,
                statistics: Statistics::default(),
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn preloaded_payload_retiming_preserves_the_actual_grasp() {
        let mut rig = super::super::tests::mechanism_fixture();
        rig.config.engineer_jaws = (0..2)
            .map(|i| Jaw {
                name: format!("jaw-{i}"),
                origin_m: [0.; 3],
                axis: [0., 0., 1.],
                open_rad: 0.,
                closed_rad: 0.2,
            })
            .collect();
        let mut arm = rm_simulator_world::assembly::ArmState::neutral(1, rig.runtime_model());
        arm.jaws_rad = [0.2; 2];
        rig.set_live_entry(&arm);
        let carried = rig.carried_start(&arm);
        assert_eq!(carried.engineer_q, arm.joints_rad);
        assert!(carried.gripped);
        let mut goal = arm.joints_rad;
        goal[0] += 0.7;
        let next = at(carried.clone(), goal);
        let frame = sample_frame(&next, [0.09, 0.2, 0.6], [0.; 3], Stage::Approach, true);
        let plan = Plan {
            frames: vec![frame],
            statistics: Statistics::default(),
        };
        let samples = carried.runtime_limited_frames(&plan).unwrap();
        assert!(samples.len() > 1);
        for f in samples {
            assert!(f.gripped);
            assert_eq!(f.guide_m, [0.09, 0.2, 0.6]);
            let expected = at(carried.clone(), f.joints_rad).tool_unit_pose();
            assert!(position(expected).distance(position(f.unit)) < 2e-6);
            assert!(quaternion(expected).angle_between(quaternion(f.unit).normalize()) < 2e-6);
        }
    }
    #[test]
    #[ignore = "requires source CAD and a completed recording"]
    fn audit_recorded_source_contacts() {
        let package = std::env::var("ASSEMBLY_CAD_PACKAGE").unwrap();
        let area = std::env::var("ASSEMBLY_CAD_AREA").unwrap();
        let input =
            std::fs::read_to_string(std::env::var("ASSEMBLY_CONTACT_RECORDING").unwrap()).unwrap();
        let mut rows = input.lines();
        let header: serde_json::Value = serde_json::from_str(rows.next().unwrap()).unwrap();
        let requested = serde_json::from_value(header["requested_pose"].clone()).unwrap();
        let mut rig = Rig::load_in_area(Path::new(&package), requested, Path::new(&area)).unwrap();
        for row in rows {
            let row: serde_json::Value = serde_json::from_str(row).unwrap();
            let cad = &row["cad"];
            rig.tool = serde_json::from_value(cad["tool"].clone()).unwrap();
            rig.unit_pose = serde_json::from_value(cad["unit_pose"].clone()).unwrap();
            rig.core_q = serde_json::from_value(cad["core_q_rad"].clone()).unwrap();
            rig.gripped = cad["gripped"].as_bool().unwrap();
            rig.grip_requested = cad["grip_requested"].as_bool().unwrap();
            rig.grasp_roll_rad = cad["grasp_roll_rad"].as_f64().unwrap();
            rig.grasp_axial_m = cad["grasp_axial_m"].as_f64().unwrap();
            rig.lift_m = row["guide_m"][2].as_f64().unwrap() - 0.6;
            rig.p_rad = row["angles_rad"][0].as_f64().unwrap();
            rig.q_rad = row["angles_rad"][1].as_f64().unwrap();
            let frames = serde_json::from_value(row["frames_world_flu"].clone()).unwrap();
            let hits = rig
                .collision
                .as_ref()
                .unwrap()
                .contacts(&frames, &rig.allowed_contacts(), 0.0005)
                .unwrap();
            assert!(hits.is_empty(), "tick {}: {hits:?}", row["tick"]);
        }
    }
    /// Isolate the dominant collision to original source leaves. Diagnostic only:
    /// neither ownership nor collision exclusions are changed by this audit.
    #[test]
    #[ignore = "requires source CAD; isolates source-leaf collision pairs"]
    fn audit_chassis_shoulder_collision_leaves() {
        let package = PathBuf::from(std::env::var("ASSEMBLY_CAD_PACKAGE").unwrap());
        let rig = if let Ok(area) = std::env::var("ASSEMBLY_CAD_AREA") {
            Rig::load_in_area(
                &package,
                TargetPose {
                    position_m: [-0.05, 0.2, 0.5],
                    theta_rad: 0.,
                    phi_rad: std::f64::consts::FRAC_PI_2,
                    alpha_rad: 0.,
                },
                Path::new(&area),
            )
            .unwrap()
        } else {
            Rig::load(&package).unwrap()
        };
        let asset: rm_simulator_physics::manipulator_collision::CollisionAsset =
            serde_json::from_slice(&std::fs::read(package.join("collision.json")).unwrap())
                .unwrap();
        let pair = std::env::var("ASSEMBLY_COLLISION_PAIR")
            .unwrap_or_else(|_| "engineer/body,engineer/j2".into());
        let (first, second) = pair
            .split_once(',')
            .expect("pair must name two frames separated by comma");
        let body = asset.bodies.iter().find(|b| b.frame == first).unwrap();
        let shoulder = asset.bodies.iter().find(|b| b.frame == second).unwrap();
        let mut cases = vec![
            ("source-zero".to_string(), [0.; 6]),
            ("loaded".into(), rig.engineer_q),
        ];
        cases.extend(
            ik_candidates(
                &rig.config.engineer,
                compose(
                    inverse(rig.engineer_body),
                    rig.target([0.09, 0.2, 0.6], [0.; 3]),
                ),
                branch_seeds(&rig.config.engineer),
                12,
            )
            .into_iter()
            .enumerate()
            .map(|(i, q)| (format!("initial-branch-{i}"), q)),
        );
        if let Ok(path) = std::env::var("ASSEMBLY_COLLISION_REPLAY") {
            let old: serde_json::Value =
                serde_json::from_slice(&std::fs::read(path).unwrap()).unwrap();
            for case in old["cases"].as_array().unwrap() {
                if case["label"]
                    .as_str()
                    .unwrap()
                    .starts_with("initial-branch-")
                {
                    cases.push((
                        format!("replay-{}", case["label"].as_str().unwrap()),
                        serde_json::from_value(case["q_rad"].clone()).unwrap(),
                    ));
                }
            }
        }
        let mut cases: Vec<_> = cases
            .into_iter()
            .map(|(label, q)| {
                let candidate = at(rig.clone(), q);
                (label, q, candidate.world_frames())
            })
            .collect();
        if let Ok(path) = std::env::var("ASSEMBLY_COLLISION_PLANNER") {
            let old: serde_json::Value =
                serde_json::from_slice(&std::fs::read(path).unwrap()).unwrap();
            let witnesses = &old["planner"]["failure"]["statistics"]["rejected_milestones"];
            for (key, witness) in witnesses.as_object().unwrap() {
                let hit = &witness["hit"];
                if (hit["first"] == first && hit["second"] == second)
                    || (hit["first"] == second && hit["second"] == first)
                {
                    cases.push((
                        format!("planner-{key}"),
                        serde_json::from_value(witness["q_rad"].clone()).unwrap(),
                        serde_json::from_value(witness["frames"].clone()).unwrap(),
                    ));
                }
            }
        }
        if std::env::var_os("ASSEMBLY_COLLISION_DEPLOYED").is_some() {
            let deployed: Vec<_> = cases
                .iter()
                .map(|(label, q, frames)| {
                    let mut frames = frames.clone();
                    let forearm = compose(
                        frames["engineer/j3"],
                        inverse(translation(rig.config.engineer.axes[2].origin_m)),
                    );
                    for aux in rig
                        .config
                        .engineer_auxiliary
                        .iter()
                        .filter(|a| a.name == "camera-lift")
                    {
                        frames.insert(
                            format!("engineer/{}", aux.name),
                            compose(forearm, aux.reference),
                        );
                    }
                    (format!("{label}-aux-deployed"), *q, frames)
                })
                .collect();
            cases.extend(deployed);
        }
        let mut report = vec![];
        for (label, q, frames) in cases {
            let bounds = |part: &rm_simulator_physics::manipulator_collision::CollisionPart,
                          frame: Pose| {
                let points_m = match part {
                    rm_simulator_physics::manipulator_collision::CollisionPart::Mesh {
                        points_m,
                        ..
                    }
                    | rm_simulator_physics::manipulator_collision::CollisionPart::Convex {
                        points_m,
                    } => points_m,
                    _ => panic!("source audit requires mesh or convex source leaves"),
                };
                points_m
                    .iter()
                    .map(|p| position(frame) + quaternion(frame) * glam::DVec3::from_array(*p))
                    .fold(
                        (
                            glam::DVec3::splat(f64::INFINITY),
                            glam::DVec3::splat(f64::NEG_INFINITY),
                        ),
                        |(lo, hi), p| (lo.min(p), hi.max(p)),
                    )
            };
            let aa: Vec<_> = body
                .parts
                .iter()
                .map(|p| bounds(p, frames[&body.frame]))
                .collect();
            let bb: Vec<_> = shoulder
                .parts
                .iter()
                .map(|p| bounds(p, frames[&shoulder.frame]))
                .collect();
            let mut pairs = vec![];
            let mut broad = 0;
            for (i, (body_low, body_high)) in aa.iter().enumerate() {
                for (j, (arm_low, arm_high)) in bb.iter().enumerate() {
                    if !body_low.cmple(*arm_high).all() || !arm_low.cmple(*body_high).all() {
                        continue;
                    }
                    broad += 1;
                    let scene = CollisionScene::new(
                        rm_simulator_physics::manipulator_collision::CollisionAsset {
                            schema: 1,
                            excluded_pairs: vec![],
                            bodies: vec![
                                rm_simulator_physics::manipulator_collision::CollisionBody {
                                    frame: body.frame.clone(),
                                    parts: vec![body.parts[i].clone()],
                                },
                                rm_simulator_physics::manipulator_collision::CollisionBody {
                                    frame: shoulder.frame.clone(),
                                    parts: vec![shoulder.parts[j].clone()],
                                },
                            ],
                        },
                    )
                    .unwrap();
                    let hits = scene.contacts(&frames, &[], 0.0005).unwrap();
                    if !hits.is_empty() {
                        pairs.push(serde_json::json!({"body_part":i,"j2_part":j,"first_frame":first,"second_frame":second,"overlap_bounds_m":[body_low.max(*arm_low).to_array(),body_high.min(*arm_high).to_array()],"hits":hits}));
                    }
                }
            }
            report.push(serde_json::json!({"label":label,"q_rad":q,"frames":frames,"broad_pairs":broad,"intersecting_leaves":pairs}));
        }
        let out = std::env::var("ASSEMBLY_COLLISION_AUDIT").unwrap();
        std::fs::write(out,serde_json::to_vec_pretty(&serde_json::json!({"schema":1,"first_frame":first,"second_frame":second,"manifest_sha256":format!("{:x}",Sha256::digest(std::fs::read(package.join("manifest.json")).unwrap())),"collision_sha256":format!("{:x}",Sha256::digest(std::fs::read(package.join("collision.json")).unwrap())),"cases":report})).unwrap()).unwrap();
    }
    #[test]
    #[ignore = "requires generated CAD; reverse IK diagnostic"]
    fn audit_level_three_reverse_ik() {
        let package = std::env::var("ASSEMBLY_CAD_PACKAGE")
            .unwrap_or_else(|_| "../../local-assets/assembly-cad/package".into());
        let rig = Rig::load(Path::new(&package)).unwrap();
        let mut counts = BTreeMap::<String, usize>::new();
        let mut witness = None;
        let mut first_kinematic = None;
        let offsets = if std::env::var_os("ASSEMBLY_AUDIT_NEAR").is_some() {
            vec![-0.4, -0.5, -0.6]
        } else {
            vec![-0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3]
        };
        'setups: for dx in offsets {
            for dy in [0., -0.2, 0.2] {
                for yaw in (0..24).map(|i| i as f64 * std::f64::consts::TAU / 24.) {
                    for (roll, axial) in (0..8).flat_map(|i| {
                        [0., 0.02].map(move |a| (i as f64 * std::f64::consts::TAU / 8., a))
                    }) {
                        let mut trial = rig.clone();
                        trial.engineer_body.translation_m[0] += dx;
                        trial.engineer_body.translation_m[1] += dy;
                        trial.engineer_body = pose(
                            position(trial.engineer_body),
                            quaternion(rig.engineer_body) * DQuat::from_rotation_z(yaw),
                        );
                        trial.grasp_roll_rad = roll;
                        trial.grasp_axial_m = axial;
                        let milestones =
                            goals(3, std::f64::consts::FRAC_PI_6, [0.09, 0.2, 0.6], [0.; 3]);
                        let (_, end, angles) = *milestones.last().unwrap();
                        let straight_flip = ik_candidates(
                            &trial.config.engineer,
                            compose(
                                inverse(trial.engineer_body),
                                trial.target(end, [std::f64::consts::FRAC_PI_2, 0., 0.]),
                            ),
                            branch_seeds(&trial.config.engineer),
                            1,
                        );
                        *counts
                            .entry("straight_flip_reachable_setups".into())
                            .or_default() += usize::from(!straight_flip.is_empty());
                        if counts.get("setups").copied().unwrap_or(0) == 0 {
                            println!(
                                "TASK_AUDIT {}",
                                serde_json::json!({"body":trial.engineer_body,"socket":trial.socket,
                                "lift":trial.target(end,[0.;3]),"flip":trial.target(end,[std::f64::consts::FRAC_PI_2,0.,0.]),
                                "align":trial.target(end,angles),"axes":trial.config.engineer.axes})
                            );
                        }
                        let candidates = ik_candidates(
                            &trial.config.engineer,
                            compose(inverse(trial.engineer_body), trial.target(end, angles)),
                            branch_seeds(&trial.config.engineer),
                            8,
                        );
                        *counts.entry("setups".into()).or_default() += 1;
                        for q in candidates {
                            *counts.entry("end_solutions".into()).or_default() += 1;
                            let mut probe = at(trial.clone(), q);
                            let mut previous = (end, angles);
                            let mut viable = true;
                            for (stage, p, a) in milestones.iter().rev().copied().chain(
                                std::iter::once((Stage::Approach, [0.09, 0.2, 0.6], [0.; 3])),
                            ) {
                                let count = (0..3)
                                    .map(|i| (p[i] - previous.0[i]).abs() / 0.005)
                                    .chain((0..3).map(|i| (a[i] - previous.1[i]).abs() / 0.02))
                                    .fold(0., f64::max)
                                    .ceil()
                                    .max(1.) as usize;
                                for i in 1..=count {
                                    let t = i as f64 / count as f64;
                                    let x = std::array::from_fn(|j| {
                                        previous.0[j] + (p[j] - previous.0[j]) * t
                                    });
                                    let r = std::array::from_fn(|j| {
                                        previous.1[j] + (a[j] - previous.1[j]) * t
                                    });
                                    if probe.solve_tool(x, r).is_err() {
                                        *counts
                                            .entry(format!("reverse {stage:?} IK"))
                                            .or_default() += 1;
                                        viable = false;
                                        break;
                                    }
                                }
                                if !viable {
                                    break;
                                }
                                previous = (p, a);
                            }
                            if viable {
                                *counts.entry("continuous_ik".into()).or_default() += 1;
                                let setup = serde_json::json!({"dx":dx,"dy":dy,"yaw":yaw,"roll":roll,"axial":axial,"root_q":probe.engineer_q});
                                first_kinematic.get_or_insert_with(|| setup.clone());
                                if counts.get("guarded_paths").copied().unwrap_or(0) >= 32 {
                                    break 'setups;
                                }
                                *counts.entry("guarded_paths".into()).or_default() += 1;
                                let mut root = at(trial.clone(), probe.engineer_q);
                                root.initial_engineer_q = root.engineer_q;
                                root.unit_pose = root.tool_unit_pose();
                                let mut search = Search {
                                    stats: Statistics::default(),
                                    budget: 8000,
                                    ik_table: IkTable::default(),
                                    transitions: TransitionTable::default(),
                                    last: None,
                                    last_checked: None,
                                    stage: Stage::Approach,
                                    signal: None,
                                };
                                match build(
                                    &rig,
                                    &root,
                                    3,
                                    std::f64::consts::FRAC_PI_6,
                                    [0.09, 0.2, 0.6],
                                    [0.; 3],
                                    true,
                                    &mut search,
                                    ChassisRoute::default(),
                                ) {
                                    Ok((_, frames)) => {
                                        witness = Some(
                                            serde_json::json!({"setup":setup,"frames":frames.len()}),
                                        );
                                        break 'setups;
                                    }
                                    Err(e) => {
                                        *counts
                                            .entry(format!(
                                                "guarded {e}: {:?}",
                                                search.last.as_ref().map(|h| (&h.first, &h.second))
                                            ))
                                            .or_default() += 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        println!(
            "REVERSE_AUDIT {}",
            serde_json::json!({"counts":counts,"first_kinematic_witness":first_kinematic,"guarded_witness":witness})
        );
    }
    /// Explicit source-CAD calibration experiment; never changes runtime guards.
    #[test]
    #[ignore = "requires generated CAD; bounded searches can take minutes"]
    fn audit_level_three_joint_guards() {
        let package = std::env::var("ASSEMBLY_CAD_PACKAGE")
            .unwrap_or_else(|_| "../../local-assets/assembly-cad/package".into());
        let mode = std::env::var("ASSEMBLY_AUDIT_LIMITS").unwrap_or_else(|_| "original".into());
        assert!(["original", "j4", "winding", "all"].contains(&mode.as_str()));
        let mut rig = Rig::load(Path::new(&package)).unwrap();
        for (i, axis) in rig.config.engineer.axes.iter_mut().enumerate() {
            if i == 3 && ["j4", "all"].contains(&mode.as_str()) {
                axis.range_rad = [-std::f64::consts::PI, std::f64::consts::PI];
            } else if i != 3 && ["winding", "all"].contains(&mode.as_str()) {
                axis.range_rad = [-2. * std::f64::consts::PI, 2. * std::f64::consts::PI];
            }
        }
        let result = match rig.plan_assembly(
            3,
            std::f64::consts::FRAC_PI_6,
            [0.09, 0.2, 0.6],
            [0.; 3],
            true,
            8000,
        ) {
            Ok((mut start, plan)) => {
                let mut ranges = [[f64::INFINITY, f64::NEG_INFINITY]; 6];
                for f in &plan.frames {
                    start.follow_plan(f).unwrap();
                    for (i, range) in ranges.iter_mut().enumerate() {
                        range[0] = range[0].min(f.q[i]);
                        range[1] = range[1].max(f.q[i]);
                    }
                }
                serde_json::json!({"status":"ready","statistics":plan.statistics,
                    "joint_ranges_rad":ranges,"final":start.report()})
            }
            Err(failure) => serde_json::json!({"status":"failed","failure":failure}),
        };
        println!(
            "GUARD_AUDIT {}",
            serde_json::json!({"mode":mode,"result":result})
        );
    }
    #[test]
    fn segment_boundaries_join_exactly_and_subdivision_preserves_the_guide() {
        let start = ([0.09, 0.2, 0.6], [0.; 3]);
        let task = segments(3, 0.5, start.0, start.1);
        assert_eq!(task.first().unwrap().from, start);
        for pair in task.windows(2) {
            assert_eq!(pair[0].to, pair[1].from);
        }
        assert_eq!(
            task.last().unwrap().to,
            ([-0.05, 0.2, 0.7], [std::f64::consts::FRAC_PI_2, 0.5, 0.])
        );
        for segment in task {
            assert_eq!(segment.point(0.), segment.from);
            assert_eq!(segment.point(1.), segment.to);
            let n = segment.samples(0.005, 0.02);
            let mut previous = segment.from;
            for i in 1..=n {
                let point = segment.point(i as f64 / n as f64);
                for j in 0..3 {
                    assert!((point.0[j] - previous.0[j]).abs() <= 0.005 + 1e-12);
                    assert!((point.1[j] - previous.1[j]).abs() <= 0.02 + 1e-12);
                }
                previous = point;
            }
        }
    }
    /// Controlled IK comparison from exactly the same successful grasp and parking.
    #[test]
    #[ignore = "requires generated CAD and a completed twisting level-three recording"]
    fn audit_twist_counterfactual() {
        let package = std::env::var("ASSEMBLY_CAD_PACKAGE").unwrap();
        let area = std::env::var("ASSEMBLY_CAD_AREA").unwrap();
        let motion =
            std::fs::read_to_string(std::env::var("ASSEMBLY_SPIN_REFERENCE").unwrap()).unwrap();
        let records: Vec<serde_json::Value> = motion
            .lines()
            .map(|l| serde_json::from_str(l).unwrap())
            .collect();
        assert_eq!(
            records[0]["manifest"],
            serde_json::from_slice::<serde_json::Value>(
                &std::fs::read(Path::new(&package).join("manifest.json")).unwrap()
            )
            .unwrap()
        );
        assert_eq!(records.last().unwrap()["completed"], true);
        let cad = &records[1]["cad"];
        let requested = serde_json::from_value(cad["requested_pose"].clone()).unwrap();
        let mut rig = Rig::load_in_area(Path::new(&package), requested, Path::new(&area)).unwrap();
        assert_eq!(rig.report()["area"], cad["area"]);
        rig.engineer_body = serde_json::from_value(cad["engineer_body"].clone()).unwrap();
        rig.engineer_q = serde_json::from_value(cad["engineer_q_rad"].clone()).unwrap();
        rig.camera_deployed = cad["camera_deployed"].as_bool().unwrap();
        rig.grasp_roll_rad = cad["grasp_roll_rad"].as_f64().unwrap();
        rig.grasp_axial_m = cad["grasp_axial_m"].as_f64().unwrap();
        let mut results = vec![];
        for twist in [false, true] {
            let mut probe = rig.clone();
            let route = ChassisRoute {
                flip_spin_rad: if twist {
                    std::f64::consts::FRAC_PI_2
                } else {
                    0.
                },
                ..Default::default()
            };
            let mut failure = None;
            'segments: for segment in
                segments(3, std::f64::consts::FRAC_PI_6, [0.09, 0.2, 0.6], [0.; 3])
            {
                let n = route.samples(segment, 0.005, 0.02);
                for i in 1..=n {
                    let (p, a) = segment.point(i as f64 / n as f64);
                    if let Err(reason) = route_step(&mut probe, rig.engineer_body, route, p, a) {
                        failure = Some(
                            serde_json::json!({"stage":segment.stage,"angles_rad":a,"reason":reason}),
                        );
                        break 'segments;
                    }
                }
            }
            results.push(serde_json::json!({"twist":twist,"complete_ik":failure.is_none(),"failure":failure}));
        }
        assert_eq!(results[1]["complete_ik"], true);
        let path = std::env::var("ASSEMBLY_SPIN_AUDIT").unwrap();
        std::fs::write(path,serde_json::to_vec_pretty(&serde_json::json!({"purpose":"same initial configuration; IK-only counterfactual; full swept success is in the recording","cases":results})).unwrap()).unwrap();
    }
    /// Reproduce the completed real-area lift, then compare locked and free rod clocking.
    #[test]
    #[ignore = "requires generated CAD and a completed level-two area recording"]
    fn audit_rod_spin_reachability() {
        let package = std::env::var("ASSEMBLY_CAD_PACKAGE").unwrap();
        let area = std::env::var("ASSEMBLY_CAD_AREA").unwrap();
        let input = std::env::var("ASSEMBLY_SPIN_REFERENCE").unwrap();
        let reference: serde_json::Value =
            serde_json::from_slice(&std::fs::read(input).unwrap()).unwrap();
        assert_eq!(reference["planner"]["status"], "ready");
        assert_eq!(reference["cores"][0]["step"], "Confirmed");
        assert_eq!(reference["difficulty"], 2);
        let cad = &reference["cad"];
        let requested = serde_json::from_value(cad["requested_pose"].clone()).unwrap();
        let mut rig = Rig::load_in_area(Path::new(&package), requested, Path::new(&area)).unwrap();
        assert_eq!(
            rig.report()["area"],
            cad["area"],
            "reference terrain/configuration must match"
        );
        rig.engineer_body = serde_json::from_value(cad["engineer_body"].clone()).unwrap();
        rig.engineer_q = serde_json::from_value(cad["engineer_q_rad"].clone()).unwrap();
        rig.camera_deployed = cad["camera_deployed"].as_bool().unwrap();
        rig.grasp_roll_rad = cad["grasp_roll_rad"].as_f64().unwrap();
        rig.grasp_axial_m = cad["grasp_axial_m"].as_f64().unwrap();
        let mut seeds = vec![rig.engineer_q];
        seeds.extend(branch_seeds(&rig.config.engineer));
        let mut rows = vec![];
        for p_degrees in [0., 30., 60., 90.] {
            for q_degrees in if p_degrees == 90. {
                vec![0., 30.]
            } else {
                vec![0.]
            } {
                let mut reachable = vec![];
                let mut clear = vec![];
                let mut rejected = BTreeMap::<String, usize>::new();
                for spin_degrees in (-8..8).map(|i| i as f64 * 22.5) {
                    rig.rod_spin_rad = spin_degrees.to_radians();
                    let logical = [-0.05, 0.2, 0.7];
                    let angles = [
                        p_degrees * std::f64::consts::PI / 180.,
                        q_degrees * std::f64::consts::PI / 180.,
                        0.,
                    ];
                    let solutions = ik_candidates(
                        &rig.config.engineer,
                        compose(inverse(rig.engineer_body), rig.target(logical, angles)),
                        seeds.clone(),
                        8,
                    );
                    if !solutions.is_empty() {
                        reachable.push(spin_degrees);
                    }
                    let mut has_clear = false;
                    for q in solutions {
                        let mut probe = at(rig.clone(), q);
                        probe.gripped = true;
                        probe.grip_requested = true;
                        probe.lift_m = 0.1;
                        probe.p_rad = angles[0];
                        probe.q_rad = angles[1];
                        probe.unit_pose = probe.tool_unit_pose();
                        match probe.collision_contacts().unwrap().first() {
                            None => has_clear = true,
                            Some(hit) => {
                                *rejected
                                    .entry(format!("{} / {}", hit.first, hit.second))
                                    .or_default() += 1
                            }
                        }
                    }
                    if has_clear {
                        clear.push(spin_degrees);
                    }
                }
                rows.push(serde_json::json!({"p_degrees":p_degrees,"q_degrees":q_degrees,
                    "reachable_spin_degrees":reachable,"clear_spin_degrees":clear,"rejected_pairs":rejected}));
            }
        }
        let body = rig.engineer_body;
        let mut placements = vec![];
        for dx in [-0.3, -0.15, 0., 0.15] {
            for dy in [-0.15, 0., 0.15] {
                for yaw_degrees in [-30., 0., 30.] {
                    rig.engineer_body = pose(
                        position(body) + glam::DVec3::new(dx, dy, 0.),
                        quaternion(body)
                            * DQuat::from_rotation_z(yaw_degrees * std::f64::consts::PI / 180.),
                    );
                    if !rig.support_parking() {
                        continue;
                    }
                    let mut reachable = vec![];
                    let mut clear = vec![];
                    let mut witness = None;
                    let mut rejected = BTreeMap::<String, usize>::new();
                    for spin_degrees in [0., -45., 45., -90., 90., -135., 135., 180.] {
                        rig.rod_spin_rad = spin_degrees * std::f64::consts::PI / 180.;
                        let angles = [std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_6, 0.];
                        let solutions = ik_candidates(
                            &rig.config.engineer,
                            compose(
                                inverse(rig.engineer_body),
                                rig.target([-0.05, 0.2, 0.7], angles),
                            ),
                            seeds.clone(),
                            4,
                        );
                        if !solutions.is_empty() {
                            reachable.push(spin_degrees);
                        }
                        for q in solutions {
                            let mut probe = at(rig.clone(), q);
                            probe.gripped = true;
                            probe.grip_requested = true;
                            probe.lift_m = 0.1;
                            probe.p_rad = angles[0];
                            probe.q_rad = angles[1];
                            probe.unit_pose = probe.tool_unit_pose();
                            let hits = probe.collision_contacts().unwrap();
                            for hit in &hits {
                                *rejected
                                    .entry(format!("{} / {}", hit.first, hit.second))
                                    .or_default() += 1;
                            }
                            if hits.is_empty() {
                                clear.push(spin_degrees);
                                witness.get_or_insert_with(|| Witness::at(&probe));
                                break;
                            }
                        }
                    }
                    placements.push(serde_json::json!({"dx_m":dx,"dy_m":dy,"yaw_delta_degrees":yaw_degrees,
                        "reachable_spin_degrees":reachable,"clear_spin_degrees":clear,"witness":witness,"rejected_pairs":rejected}));
                }
            }
        }
        let output = serde_json::json!({"purpose":"static milestone comparison, not a connected executable plan", "reference":cad, "rows":rows,"placements_at_p90_q30":placements});
        let path = std::env::var("ASSEMBLY_SPIN_AUDIT").unwrap();
        std::fs::write(path, serde_json::to_vec_pretty(&output).unwrap()).unwrap();
    }
    #[test]
    fn axial_spin_preserves_rod_position_axis_and_rigid_grasp_through_flip() {
        let mut rig = super::super::tests::mechanism_fixture();
        rig.grasp_axial_m = 0.02;
        for angles in [
            [0.; 3],
            [0.7, 0., 0.],
            [std::f64::consts::FRAC_PI_2, 0.5, 0.],
        ] {
            let logical = [-0.05, 0.2, 0.7];
            rig.rod_spin_rad = 0.;
            rig.tool = rig.target(logical, angles);
            let zero = rig.tool_unit_pose();
            let grip = compose(inverse(rig.tool), zero);
            let axis = quaternion(zero) * glam::DVec3::Z;
            for spin in [-3., -1.5, 0.4, 2.8] {
                rig.rod_spin_rad = spin;
                rig.tool = rig.target(logical, angles);
                let unit = rig.tool_unit_pose();
                assert!(position(unit).distance(position(zero)) < 1e-12);
                assert!((quaternion(unit) * glam::DVec3::Z - axis).length() < 1e-12);
                assert!(quaternion(unit).angle_between(quaternion(zero)) > 0.3);
                let actual_grip = compose(inverse(rig.tool), unit);
                assert!(position(actual_grip).distance(position(grip)) < 1e-12);
                assert!(quaternion(actual_grip).angle_between(quaternion(grip)) < 1e-7);
            }
        }
    }
    #[test]
    fn rod_spin_route_is_continuous_and_replay_enforces_its_rate() {
        let route = ChassisRoute {
            lift_spin_rad: 1.2,
            flip_spin_rad: -2.,
            ..Default::default()
        };
        let at_insert = route.spin([-0.05, 0.2, 0.6], [0.; 3]);
        assert_eq!(at_insert, 0.);
        let at_lift = route.spin([-0.05, 0.2, 0.7], [0.; 3]);
        assert!((at_lift - 1.2).abs() < 1e-12);
        let at_flip = route.spin([-0.05, 0.2, 0.7], [std::f64::consts::FRAC_PI_2, 0., 0.]);
        assert!((at_flip + 0.8).abs() < 1e-12);
        assert_eq!(
            at_flip,
            route.spin([-0.05, 0.2, 0.7], [std::f64::consts::FRAC_PI_2, 0.5, 0.])
        );
        let mut rig = super::super::tests::mechanism_fixture();
        let previous = rig.report();
        let mut frame = sample_frame(&rig, [-0.05, 0.2, 0.7], [0.; 3], Stage::Lift, true);
        for spin in [0.1, f64::NAN] {
            frame.rod_spin_rad = spin;
            assert!(rig.follow_plan(&frame).is_err());
            assert_eq!(rig.report(), previous);
        }
        rig.rod_spin_rad = 0.4;
        rig.restart();
        assert_eq!(rig.rod_spin_rad, 0.);
    }
    #[test]
    fn axial_freedom_covers_every_moving_stage_and_negative_q() {
        let route = ChassisRoute {
            approach_spin_rad: 0.1,
            insert_spin_rad: 0.2,
            lift_spin_rad: 0.3,
            flip_spin_rad: 0.4,
            align_spin_rad: 0.5,
            alignment_target_rad: -0.6,
            ..Default::default()
        };
        let task = segments(3, -0.6, [0.09, 0.2, 0.6], [0.; 3]);
        let mut previous = 0.;
        for (segment, delta) in task.into_iter().zip([0.1, 0.2, 0.3, 0.4, 0.5]) {
            assert!((route.spin(segment.from.0, segment.from.1) - previous).abs() < 1e-12);
            let spin = route.spin(segment.to.0, segment.to.1);
            assert!((spin - previous - delta).abs() < 1e-12);
            let dt = rm_simulator_physics::tick_ns() as f64 * 1e-9;
            let n = route.samples(segment, super::super::SPEED * dt, super::super::RATE * dt);
            let mut last = previous;
            for i in 1..=n {
                let (p, a) = segment.point(i as f64 / n as f64);
                let v = route.spin(p, a);
                assert!((v - last).abs() <= super::super::RATE * dt + 1e-9);
                last = v;
            }
            previous = spin;
        }
        assert!((previous - 1.5).abs() < 1e-12);
    }
    #[test]
    fn resumed_axial_profiles_preserve_current_clocking_at_every_stage() {
        let route = ChassisRoute {
            approach_spin_rad: 0.2,
            insert_spin_rad: -0.3,
            lift_spin_rad: 0.4,
            flip_spin_rad: 0.5,
            align_spin_rad: -0.6,
            ..Default::default()
        };
        for segment in segments(3, -0.6, [0.09, 0.2, 0.6], [0.; 3]) {
            let (p, a) = segment.point(0.4);
            let anchored = route.anchored(0.75, p, a, -0.6);
            assert!((anchored.spin(p, a) - 0.75).abs() < 1e-12);
        }
    }
    #[test]
    fn nearby_ik_table_reuses_seeds_but_resolves_the_new_pose_and_model() {
        let mut arm = super::super::tests::mechanism_fixture().config.engineer;
        arm.tool = Pose::at([0.2, 0., 0.]);
        let q = [0.05; 6];
        let old = arm.tool_pose(q);
        let target = arm.tool_pose([0.052; 6]);
        let mut table = IkTable::default();
        table.insert(&arm, old, vec![q]);
        let (warm, exact) = table.seeds(&arm, target);
        assert!(!exact);
        assert_eq!(warm, vec![q]);
        let solved = arm.solve(target, warm[0]).unwrap();
        assert_ne!(solved, q);
        assert!(position(arm.tool_pose(solved)).distance(position(target)) < 1e-5);
        assert!(quaternion(arm.tool_pose(solved)).angle_between(quaternion(target)) < 1e-4);
        assert!(table.seeds(&arm, old).1);
        arm.axes[0].range_rad = [-1., 1.];
        assert!(table.seeds(&arm, target).0.is_empty());
    }
    #[test]
    fn exact_transition_cache_rejects_mutated_state_configuration_and_geometry() {
        let rig = super::super::tests::mechanism_fixture();
        let mut search = Search {
            stats: Statistics::default(),
            budget: 8,
            ik_table: IkTable::default(),
            transitions: TransitionTable::default(),
            last: None,
            last_checked: None,
            stage: Stage::Approach,
            signal: None,
        };
        let checked = search.check(&rig, rig.clone()).unwrap();
        search.check(&rig, rig.clone()).unwrap();
        assert_eq!(search.stats.collision_table_hits, 1);
        let frame = search.frame(&checked, [0.09, 0.2, 0.6], [0.; 3], Stage::Approach, true);
        let mut replay = rig.clone();
        replay.follow_plan(&frame).unwrap();
        assert_eq!(replay.cached_guard_replays, 1);
        let mut changed = frame.clone();
        changed.q[0] = 0.001;
        let mut replay = rig.clone();
        replay.follow_plan(&changed).unwrap();
        assert_eq!(replay.cached_guard_replays, 0);
        let mut changed = rig.clone();
        changed.config.engineer.axes[0].origin_m[0] += 0.001;
        assert!(
            !frame
                .validation
                .as_ref()
                .unwrap()
                .matches(&changed, &checked)
        );
        changed = rig.clone();
        changed.core_q[0] = 0.001;
        assert!(
            !frame
                .validation
                .as_ref()
                .unwrap()
                .matches(&changed, &checked)
        );
        changed = rig.clone();
        changed.grasp_roll_rad = 0.1;
        assert!(
            !frame
                .validation
                .as_ref()
                .unwrap()
                .matches(&changed, &checked)
        );
        use rm_simulator_physics::manipulator_collision::{
            CollisionAsset, CollisionBody, CollisionPart,
        };
        let mut obstructed = rig.clone();
        obstructed.collision = Some(std::sync::Arc::new(
            CollisionScene::new(CollisionAsset {
                schema: 1,
                excluded_pairs: vec![],
                bodies: vec![
                    CollisionBody {
                        frame: "unit/unit".into(),
                        parts: vec![CollisionPart::Box { half_m: [0.01; 3] }],
                    },
                    CollisionBody {
                        frame: "floor".into(),
                        parts: vec![CollisionPart::Box { half_m: [0.1; 3] }],
                    },
                ],
            })
            .unwrap(),
        ));
        assert!(obstructed.follow_plan(&frame).is_err());
        assert_eq!(obstructed.cached_guard_replays, 0);
        assert!(obstructed.last_collision.is_some());
    }
    #[test]
    fn fallback_reserves_room_for_playback_validation_in_small_sweeps() {
        assert_eq!(linear_search_budget(8000), 6400);
        assert_eq!(linear_search_budget(2500), 1300);
        assert_eq!(linear_search_budget(4000), 2800);
        assert!(linear_search_budget(2500) + 808 < 2500);
    }
    #[test]
    fn nonlinear_routes_preserve_linear_setup_order() {
        let mut old: Vec<_> = (0..100)
            .flat_map(|i| (0..18).map(move |j| (i, j)))
            .collect();
        old.sort_by_key(|(i, j)| route_order(*i, *j, 18, 18));
        let mut expanded: Vec<_> = (0..100)
            .flat_map(|i| (0..26).map(move |j| (i, j)))
            .collect();
        expanded.sort_by_key(|(i, j)| route_order(*i, *j, 18, 26));
        assert_eq!(&expanded[..old.len()], old.as_slice());
        assert!(expanded[old.len()..].iter().all(|(_, j)| *j >= 18));
    }
    #[test]
    fn twist_then_unwind_sampling_respects_rate_despite_zero_net_spin() {
        let route = ChassisRoute {
            flip_mid_spin_rad: Some(std::f64::consts::FRAC_PI_2),
            ..Default::default()
        };
        let segment = Segment {
            stage: Stage::Flip,
            from: ([-0.05, 0.2, 0.7], [0.; 3]),
            to: ([-0.05, 0.2, 0.7], [std::f64::consts::FRAC_PI_2, 0., 0.]),
        };
        let dt = rm_simulator_physics::tick_ns() as f64 * 1e-9;
        let n = route.samples(segment, super::super::SPEED * dt, super::super::RATE * dt);
        let mut previous = 0.;
        let mut maximum: f64 = 0.;
        for i in 1..=n {
            let (p, a) = segment.point(i as f64 / n as f64);
            let spin = route.spin(p, a);
            assert!((spin - previous).abs() <= super::super::RATE * dt + 1e-9);
            maximum = maximum.max(spin);
            previous = spin;
        }
        assert!(maximum > 1.56);
        assert_eq!(previous, 0.);
    }
    #[test]
    fn seated_spin_playback_moves_the_unit_and_gripper_together() {
        let mut rig = super::super::tests::mechanism_fixture();
        let logical = [-0.05, 0.2, 0.7];
        rig.grasp_axial_m = 0.02;
        rig.tool = rig.target(logical, [0.; 3]);
        rig.unit_pose = rig.tool_unit_pose();
        rig.config.engineer.tool = rig.tool;
        let unit = rig.unit_pose;
        let normal = quaternion(unit) * glam::DVec3::Z;
        for axis in &mut rig.config.engineer.axes {
            axis.origin_m = unit.translation_m;
            axis.axis = normal.to_array();
        }
        rig.gripped = true;
        rig.grip_requested = true;
        let grip = compose(inverse(rig.tool), unit);
        let mut frame = sample_frame(&rig, logical, [0.; 3], Stage::Lift, false);
        frame.rod_spin_rad = 0.001;
        frame.q[0] = 0.001;
        rig.follow_plan(&frame).unwrap();
        assert!(position(rig.unit_pose).distance(position(unit)) < 1e-12);
        assert!((quaternion(rig.unit_pose) * glam::DVec3::Z - normal).length() < 1e-12);
        let spun_grip = compose(inverse(rig.tool), rig.unit_pose);
        assert!(position(spun_grip).distance(position(grip)) < 1e-12);
        assert!(quaternion(spun_grip).angle_between(quaternion(grip)) < 1e-7);
        assert!((quaternion(rig.unit_pose).angle_between(quaternion(unit)) - 0.001).abs() < 1e-9);
    }
    #[test]
    fn chassis_route_is_continuous_and_playback_rejects_teleporting() {
        let route = ChassisRoute {
            lift_offset_m: [0.05, 0.],
            flip_offset_m: [0.1, 0.],
            flip_yaw_rad: 0.2,
            ..Default::default()
        };
        let origin = Pose::yawed([1., 0., 0.3], 0.7);
        assert_eq!(route.body(origin, [-0.05, 0.2, 0.6], [0.; 3]), origin);
        let lifted = route.body(origin, [-0.05, 0.2, 0.7], [0.; 3]);
        assert!((lifted.translation_m[0] - 1.05).abs() < 1e-12);
        let final_body = route.body(
            origin,
            [-0.05, 0.2, 0.7],
            [std::f64::consts::FRAC_PI_2, 0., 0.],
        );
        assert_eq!(
            final_body,
            route.body(
                origin,
                [-0.05, 0.2, 0.7],
                [std::f64::consts::FRAC_PI_2, 0.5, 0.]
            )
        );
        let mut rig = super::super::tests::mechanism_fixture();
        let previous = rig.report();
        let mut frame = sample_frame(&rig, [0.09, 0.2, 0.6], [0.; 3], Stage::Approach, true);
        frame.chassis.translation_m[0] = 0.1;
        assert!(rig.follow_plan(&frame).is_err());
        assert_eq!(rig.report(), previous);
        frame.chassis.translation_m[0] = f64::NAN;
        assert!(rig.follow_plan(&frame).is_err());
        assert_eq!(rig.report(), previous);
    }
    #[test]
    fn moving_base_playback_keeps_the_held_gripper_on_its_world_guide() {
        let mut rig = super::super::tests::mechanism_fixture();
        let logical = [0.09, 0.2, 0.6];
        let expected = rig.target(logical, [0.; 3]);
        rig.config.engineer.tool = expected;
        rig.tool = expected;
        rig.unit_pose = rig.tool_unit_pose();
        rig.gripped = true;
        rig.grip_requested = true;
        let unit = rig.unit_pose;
        let mut frame = sample_frame(&rig, logical, [0.; 3], Stage::Lift, false);
        frame.chassis = Pose::yawed([0.; 3], 0.001);
        frame.q[0] = -0.001;
        rig.follow_plan(&frame).unwrap();
        assert_eq!(rig.engineer_body, frame.chassis);
        assert!(position(rig.tool).distance(position(expected)) < 1e-12);
        assert!(quaternion(rig.tool).angle_between(quaternion(expected)) < 1e-7);
        assert!(position(rig.unit_pose).distance(position(unit)) < 1e-12);
        assert!(rig.gripped);
    }
    #[test]
    fn chassis_motion_sweeps_an_obstacle_and_restart_restores_initial_parking() {
        use rm_simulator_physics::manipulator_collision::{
            CollisionAsset, CollisionBody, CollisionPart,
        };
        let mut rig = super::super::tests::mechanism_fixture();
        rig.engineer_body = Pose::at([-0.05, 0., 0.]);
        rig.initial_engineer_body = rig.engineer_body;
        rig.collision = Some(std::sync::Arc::new(
            CollisionScene::new(CollisionAsset {
                schema: 1,
                excluded_pairs: vec![],
                bodies: vec![
                    CollisionBody {
                        frame: "engineer/body".into(),
                        parts: vec![CollisionPart::Box { half_m: [0.01; 3] }],
                    },
                    CollisionBody {
                        frame: "floor".into(),
                        parts: vec![CollisionPart::Box {
                            half_m: [0.005, 0.2, 0.2],
                        }],
                    },
                ],
            })
            .unwrap(),
        ));
        let origin = rig.engineer_body;
        let mut next = rig.clone();
        next.engineer_body.translation_m[0] = 0.05;
        assert!(rig.collision_contacts().unwrap().is_empty());
        assert!(next.collision_contacts().unwrap().is_empty());
        assert!(rig.accept_candidate(next).is_err());
        assert_eq!(rig.engineer_body, origin);
        assert!(rig.last_collision.is_some());
        rig.engineer_body.translation_m[0] = 0.4;
        rig.restart();
        assert_eq!(rig.engineer_body, origin);
    }
    #[test]
    fn camera_preparation_changes_only_the_camera_link_and_is_preserved_by_clone() {
        let mut rig = super::super::tests::mechanism_fixture();
        rig.config.engineer_auxiliary.push(Auxiliary {
            name: "camera-lift".into(),
            reference: Pose::at([0.1, 0., 0.3]),
            stowed_delta: Pose::at([0., 0., -0.07]),
        });
        rig.engineer_q[0] = 0.7;
        let stowed = rig.world_frames();
        rig.camera_deployed = true;
        let deployed = rig.clone().world_frames();
        assert!(
            (position(deployed["engineer/camera-lift"])
                .distance(position(stowed["engineer/camera-lift"]))
                - 0.07)
                .abs()
                < 1e-12
        );
        for (name, frame) in &stowed {
            if name != "engineer/camera-lift" {
                assert_eq!(*frame, deployed[name]);
            }
        }
        assert_eq!(rig.report()["camera_deployed"], true);
    }
    #[test]
    fn reverse_branch_connects_a_junction_and_can_continue_forwards() {
        let mut rig = super::super::tests::mechanism_fixture();
        let from = ([0.09, 0.2, 0.6], [0.; 3]);
        let segment = Segment {
            stage: Stage::Align,
            from,
            to: (from.0, [0., 0.15, 0.]),
        };
        rig.config.engineer.tool = rig.target(from.0, from.1);
        let goal = rig
            .config
            .engineer
            .solve(rig.target(segment.to.0, segment.to.1), [0.025; 6])
            .unwrap();
        let mut stats = Statistics::default();
        let roots = reverse_roots(&rig, &[segment], &[goal], &mut stats);
        assert_eq!(roots.len(), 1);
        assert_eq!(stats.connected_segments["Align reverse IK"], 1);
        let mut probe = at(rig, roots[0]);
        let n = segment.samples(0.005, 0.02);
        for i in 0..=n {
            let (p, a) = segment.point(i as f64 / n as f64);
            probe.solve_tool(p, a).unwrap();
            assert!(probe.position_error_m <= 1e-5);
            assert!(probe.angle_error_rad <= 1e-4);
        }
    }
    #[test]
    fn reverse_segments_reject_unreachable_junction_without_mutating_the_fixture() {
        let rig = super::super::tests::mechanism_fixture();
        let before = rig.report();
        let task = segments(3, 0.5, [0.09, 0.2, 0.6], [0.; 3]);
        let mut stats = Statistics::default();
        assert!(reverse_roots(&rig, &task, &[[0.; 6]], &mut stats).is_empty());
        assert_eq!(stats.reverse_roots, 0);
        assert_eq!(rig.report(), before);
    }
    #[test]
    fn task_goals_preserve_order_and_resume_without_withdrawing_a_seated_unit() {
        let full = goals(3, 0.5, [0.09, 0.2, 0.6], [0.; 3]);
        assert_eq!(
            full.iter().map(|g| g.0).collect::<Vec<_>>(),
            vec![
                Stage::Approach,
                Stage::Insert,
                Stage::Lift,
                Stage::Flip,
                Stage::Align
            ]
        );
        let resumed = goals(3, 0.5, [-0.05, 0.2, 0.7], [0.3, 0., 0.]);
        assert_eq!(
            resumed.iter().map(|g| g.0).collect::<Vec<_>>(),
            vec![Stage::Flip, Stage::Align]
        );
        assert!(goals(1, 0.5, [-0.05, 0.2, 0.6], [0.; 3]).is_empty());
    }
    #[test]
    fn failed_planning_and_invalid_frames_never_mutate_the_live_rig() {
        let mut rig = super::super::tests::mechanism_fixture();
        let before = rig.report();
        assert!(
            rig.plan_assembly(3, 0., [0.09, 0.2, 0.6], [0.; 3], false, 0)
                .is_err()
        );
        assert_eq!(rig.report(), before);
        let frame = Frame {
            validation: None,
            chassis: rig.engineer_body,
            rod_spin_rad: rig.rod_spin_rad,
            q: [99.; 6],
            logical: [0.09, 0.2, 0.6],
            angles: [0.; 3],
            close: true,
            stage: Stage::Grasp,
            free: false,
        };
        assert!(rig.follow_plan(&frame).is_err());
        assert_eq!(rig.report(), before);
    }
    #[test]
    fn grasp_transform_roundtrips_clocking_and_axial_offsets() {
        let mut rig = super::super::tests::mechanism_fixture();
        rig.config.engineer.tool = rig.target([0.09, 0.2, 0.6], [0.; 3]);
        let original = rig.config.engineer.tool;
        for roll in [0., 1.2, -1.5] {
            for axial in [0., 0.02] {
                rig.grasp_roll_rad = roll;
                rig.grasp_axial_m = axial;
                rig.tool = rig.target([0.09, 0.2, 0.6], [0.; 3]);
                let unit = rig.tool_unit_pose();
                let expected = pose(
                    position(original),
                    quaternion(original) * DQuat::from_rotation_z(-GRASP_ROLL_RAD),
                );
                assert!(position(unit).distance(position(expected)) < 1e-12);
                assert!(quaternion(unit).angle_between(quaternion(expected)) < 1e-7);
            }
        }
    }
}
