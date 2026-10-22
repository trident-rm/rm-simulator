// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared Bevy-free CAD rig and bounded automatic assembly planner.
pub(super) const RATE: f64 = 0.6;
pub(super) const SPEED: f64 = 0.12;
pub(super) const ENTRANCE_X: f64 = 0.05;
pub(super) const SOCKET: [f64; 3] = [-0.05, 0.2, 0.6];
mod area;
mod planner;
use glam::DQuat;
pub(crate) use planner::Signal;
pub use planner::{Failure as PlanningFailure, Plan as AssemblyPlan};
use rm_simulator_physics::{
    Pose,
    manipulator::SerialArm,
    manipulator_collision::{CollisionHit, CollisionScene},
};
use rm_simulator_world::gameplay::rmuc_2027::assembly_pose::TargetPose;
use serde::Deserialize;
// Fixed cylinder grasp clocking; the unit gets the inverse offset at rendering.
const GRASP_ROLL_RAD: f64 = -std::f64::consts::FRAC_PI_2;
#[cfg(test)]
use rm_simulator_physics::manipulator_collision::CollisionAsset;
use rm_simulator_world::assembly::Model as Config;
#[cfg(test)]
use rm_simulator_world::assembly::{Auxiliary, Jaw};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};
#[derive(Clone)]
pub struct Rig {
    config: Config,
    source_config_sha256: Option<String>,
    area: Option<std::sync::Arc<area::Area>>,
    camera_deployed: bool,
    fixed_chassis: bool,
    live_parking: bool,
    live_entry: Option<([f64; 6], [f64; 2])>,
    held_grasp: Option<Pose>,
    ik_hints: std::sync::Arc<Vec<(Pose, [f64; 6])>>,
    requested_spin_deltas_rad: Option<[f64; 5]>,
    preferred_spin_deltas_rad: Option<[f64; 5]>,
    initial_engineer_body: Pose,
    engineer_q: [f64; 6],
    core_q: [f64; 6],
    initial_engineer_q: [f64; 6],
    pub engineer_body: Pose,
    pub tool: Pose,
    pub socket: Pose,
    pub requested_pose: Option<TargetPose>,
    pub position_error_m: f64,
    pub angle_error_rad: f64,
    peak_joint_step_rad: f64,
    cached_guard_replays: usize,
    lift_m: f64,
    p_rad: f64,
    q_rad: f64,
    jaw_angles: Vec<f64>,
    gripped: bool,
    grasp_roll_rad: f64,
    rod_spin_rad: f64,
    grasp_axial_m: f64,
    grip_requested: bool,
    pub unit_pose: Pose,
    collision: Option<std::sync::Arc<CollisionScene>>,
    pub last_collision: Option<CollisionHit>,
}
// Earlier recordings predate the approach/insertion/alignment spin fields.
fn recorded_spin_deltas(route: &serde_json::Value) -> Option<[f64; 5]> {
    let route = route.as_object()?;
    [
        "approach_spin_rad",
        "insert_spin_rad",
        "lift_spin_rad",
        "flip_spin_rad",
        "align_spin_rad",
    ]
    .map(|key| {
        route
            .get(key)
            .map_or(Some(0.), |v| v.as_f64().filter(|v| v.is_finite()))
    })
    .into_iter()
    .collect::<Option<Vec<_>>>()?
    .try_into()
    .ok()
}
/// Validate recorded numerics before quaternion inversion or FK evaluation.
fn checked_ik_hint(
    arm: &SerialArm,
    body: Pose,
    tool: Pose,
    q: [f64; 6],
) -> Result<(Pose, [f64; 6]), &'static str> {
    rm_simulator_physics::manipulator_plan::checked_ik_hint(arm, body, tool, q)
}
pub(super) fn quaternion(p: Pose) -> DQuat {
    let [w, x, y, z] = p.rotation_wxyz;
    DQuat::from_xyzw(x, y, z, w)
}
pub(super) fn position(p: Pose) -> glam::DVec3 {
    glam::DVec3::from_array(p.translation_m)
}
pub(super) fn pose(p: glam::DVec3, q: DQuat) -> Pose {
    Pose {
        translation_m: p.to_array(),
        rotation_wxyz: [q.w, q.x, q.y, q.z],
    }
}
pub(super) fn compose(a: Pose, b: Pose) -> Pose {
    let q = quaternion(a);
    pose(position(a) + q * position(b), q * quaternion(b))
}
fn inverse(a: Pose) -> Pose {
    let q = quaternion(a).inverse();
    pose(-(q * position(a)), q)
}
fn delta(origin: [f64; 3], axis: [f64; 3], angle: f64) -> Pose {
    let origin = glam::DVec3::from_array(origin);
    let q = DQuat::from_axis_angle(glam::DVec3::from_array(axis), angle);
    pose(origin - q * origin, q)
}
fn translation(v: [f64; 3]) -> Pose {
    Pose::at(v)
}
fn initial_seeds() -> Vec<[f64; 6]> {
    let mut seeds = vec![
        [0.; 6],
        [0., 1., -1., 0., -1.5, 0.],
        [0., 1., -1., -1., -1.5, 0.],
        [0., 0.8, -1.8, 0., 1., 0.],
        [0., 1.6, 2.2, 0., -1.6, 0.],
        [0., -1.6, 2.2, 0., 1.6, 0.],
        [std::f64::consts::PI, 0.8, -1.8, 0., 1., 0.],
        [-std::f64::consts::PI, 0.8, -1.8, 0., 1., 0.],
    ];
    for shoulder in [-1.8, 1.8] {
        for elbow in [-2.2, 2.2] {
            for wrist in [-1.4, 1.4] {
                seeds.push([0., shoulder, elbow, wrist, -1.5, 0.]);
                seeds.push([0., shoulder, elbow, wrist, 1.5, 2.5]);
            }
        }
    }
    seeds
}
fn solve_initial(arm: &SerialArm, target: Pose) -> Result<[f64; 6], &'static str> {
    for seed in planner::branch_seeds(arm) {
        if let Ok(q) = arm.solve(target, seed) {
            return Ok(q);
        }
    }
    Err("source CAD arm cannot reach the workbench's initial tool pose")
}
impl Rig {
    pub fn has_area(&self) -> bool {
        self.area.is_some()
    }
    /// Use conservative per-chunk convex collision geometry; visuals stay unchanged.
    pub fn use_convex_collisions(&mut self) {
        self.collision = self.collision.as_ref().map(|scene| {
            std::sync::Arc::new(scene.convex_approximation(|name| {
                name.starts_with("engineer/")
                    && name != "engineer/j6"
                    && !name.starts_with("engineer/jaw-")
            }))
        });
    }
    pub(crate) fn use_live_collisions(&mut self) {
        self.collision = self.collision.as_ref().map(|scene| {
            std::sync::Arc::new(scene.convex_approximation(|name| {
                matches!(
                    name,
                    "engineer/j1" | "engineer/j2" | "engineer/j3" | "engineer/j4" | "engineer/j5"
                )
            }))
        });
    }
    pub fn report(&self) -> serde_json::Value {
        serde_json::json!({"initial_engineer_body":self.initial_engineer_body,"camera_deployed":self.camera_deployed,"fixed_chassis":self.fixed_chassis,"ik_hint_count":self.ik_hints.len(),"requested_spin_deltas_rad":self.requested_spin_deltas_rad,"area":self.area.as_ref().map(|a| &a.evidence),"engineer_body":self.engineer_body,"engineer_q_rad":self.engineer_q,"core_q_rad":self.core_q,"tool":self.tool,
            "fast_collisions":self.collision.as_ref().is_some_and(|s| s.uses_convex_approximation()),
            "preferred_spin_deltas_rad":self.preferred_spin_deltas_rad,
            "position_error_m":self.position_error_m,"angle_error_rad":self.angle_error_rad,
            "cached_guard_replays":self.cached_guard_replays,"peak_joint_step_rad":self.peak_joint_step_rad,"requested_pose":self.requested_pose,
            "rod_spin_rad":self.rod_spin_rad,"grasp_axial_m":self.grasp_axial_m,"grasp_roll_rad":self.grasp_roll_rad,"gripped":self.gripped,"grip_requested":self.grip_requested,"collision":self.last_collision,
            "jaw_angles_rad":self.jaw_angles,"unit_pose":self.unit_pose,
            "p_origin_world_m":compose(self.core_delta(), translation(self.config.core_p_origin_m)).translation_m,
            "post_tip_world_m":compose(self.head_delta(self.lift_m,self.p_rad,self.q_rad), self.config.core.tool).translation_m})
    }
    pub fn load(root: &Path) -> Result<Self, Box<dyn std::error::Error>> {
        Self::load_impl(root, None, None)
    }
    pub fn load_target(
        root: &Path,
        requested_pose: TargetPose,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        Self::load_impl(root, Some(requested_pose), None)
    }
    pub fn load_in_area(
        root: &Path,
        requested_pose: TargetPose,
        area: &Path,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let mut rig = Self::load_impl(root, Some(requested_pose), Some(area))?;
        rig.load_area(root, area)?;
        Ok(rig)
    }
    fn load_impl(
        root: &Path,
        requested_pose: Option<TargetPose>,
        area: Option<&Path>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let source = rm_simulator_assets::semantics::assembly::SourcePackage::load(root)?;
        let mut config: Config = source.calibration()?;
        if let Some(area) = area {
            config.assembly_frame = source.area_frame(area)?;
        }
        let goal = if let Some(requested) = requested_pose {
            let r = requested.rotation_matrix();
            let rotation = glam::DMat3::from_cols_array(&[
                r[0][0], r[1][0], r[2][0], r[0][1], r[1][1], r[2][1], r[0][2], r[1][2], r[2][2],
            ]);
            let goal = compose(
                config.assembly_frame,
                pose(
                    glam::DVec3::from_array(requested.position_m),
                    DQuat::from_mat3(&rotation),
                ),
            );
            // Table 5-10 is a bounding domain, not a guarantee that a pose is legal.
            let placement = compose(
                inverse(config.assembly_frame),
                compose(goal, inverse(config.core.tool)),
            );
            if config.core_envelope_m.is_empty()
                || config.core_envelope_m.iter().any(|p| {
                    let x =
                        position(placement) + quaternion(placement) * glam::DVec3::from_array(*p);
                    !x.is_finite() || x.x > 1e-6
                })
            {
                return Err(
                    "sample violates the Core head's pedestal-front plane; no resampling performed"
                        .into(),
                );
            }
            goal
        } else {
            // Preserve the reviewed fixed fixture for before/after mechanism comparisons.
            let mut goal = config.core.tool;
            goal.translation_m[2] = 0.6;
            goal
        };
        let core_q =
            solve_initial(&config.core, goal).map_err(|e| format!("Core positioning: {e}"))?;
        let socket = config.core.tool_pose(core_q);
        let body = if let Some(requested) = requested_pose {
            compose(
                config.assembly_frame,
                Pose::yawed([0.45, requested.position_m[1], 0.132], std::f64::consts::PI),
            )
        } else {
            Pose::yawed([0.95, socket.translation_m[1], 0.132], std::f64::consts::PI)
        };
        let mut rig = Self {
            collision: None,
            last_collision: None,
            jaw_angles: config.engineer_jaws.iter().map(|j| j.open_rad).collect(),
            config,
            source_config_sha256: source.config_sha256().map(str::to_owned),
            area: None,
            camera_deployed: false,
            fixed_chassis: false,
            live_parking: false,
            live_entry: None,
            held_grasp: None,
            ik_hints: Default::default(),
            requested_spin_deltas_rad: None,
            preferred_spin_deltas_rad: None,
            unit_pose: Pose::default(),
            gripped: false,
            grasp_roll_rad: GRASP_ROLL_RAD,
            rod_spin_rad: 0.,
            grasp_axial_m: 0.,
            grip_requested: false,
            engineer_q: [0.; 6],
            initial_engineer_q: [0.; 6],
            core_q,
            engineer_body: body,
            initial_engineer_body: body,
            tool: Pose::default(),
            socket,
            requested_pose,
            position_error_m: 0.,
            angle_error_rad: 0.,
            peak_joint_step_rad: 0.,
            cached_guard_replays: 0,
            lift_m: 0.,
            p_rad: 0.,
            q_rad: 0.,
        };
        let target = rig.target([0.09, 0.2, 0.6], [0.; 3]);
        rig.engineer_q = solve_initial(&rig.config.engineer, compose(inverse(body), target))
            .map_err(|e| format!("Engineer positioning: {e}"))?;
        rig.tool = compose(body, rig.config.engineer.tool_pose(rig.engineer_q));
        rig.initial_engineer_q = rig.engineer_q;
        rig.unit_pose = rig.tool_unit_pose();
        rig.collision = Some(std::sync::Arc::new(source.collision_scene()?));
        // Parking/Engineer IK cannot repair a collision between fixed Core
        // positioning links, its head and the floor. Do not repeat that query
        // for every robot placement or silently resample the equipment pose.
        if let Some(hit) = rig.collision_contacts()?.into_iter().find(|h| {
            [&h.first, &h.second]
                .into_iter()
                .all(|name| !name.starts_with("engineer/") && name.as_str() != "unit/unit")
        }) {
            rig.last_collision = Some(hit);
            return Ok(rig);
        }
        let original = rig.clone();
        for (forward, lateral) in [0., -0.15, 0.15, -0.3, 0.3]
            .into_iter()
            .flat_map(|x| [0., 0.15, -0.15, 0.3, -0.3, 0.5, -0.5].map(|y| (x, y)))
        {
            rig.engineer_body = original.engineer_body;
            rig.engineer_body.translation_m[0] += forward;
            rig.engineer_body.translation_m[1] += lateral;
            let local = compose(inverse(rig.engineer_body), target);
            for seed in initial_seeds() {
                if let Ok(q) = rig.config.engineer.solve(local, seed) {
                    rig.engineer_q = q;
                    rig.initial_engineer_q = q;
                    rig.tool = compose(rig.engineer_body, rig.config.engineer.tool_pose(q));
                    rig.unit_pose = rig.tool_unit_pose();
                    if rig.collision_contacts()?.is_empty() {
                        rig.initial_engineer_body = rig.engineer_body;
                        return Ok(rig);
                    }
                }
            }
        }
        rig = original;
        rig.last_collision = rig.collision_contacts()?.into_iter().next();
        Ok(rig)
    }
    fn allowed_contacts(&self) -> Vec<[&str; 2]> {
        let mut allowed = Vec::new();
        // Live Rapier wheels and bumper own body/terrain contact. Their tyre
        // compression differs slightly from the static source envelope; retain
        // every arm, payload and Core versus terrain query here.
        if self.live_parking {
            allowed.push(["engineer/body", "floor"]);
        }
        if self.live_parking && !self.gripped {
            // A loose Unit's existing support contacts are stationary during
            // an arm jog; they cannot obstruct motion of a different link.
            allowed.push(["unit/unit", "engineer/body"]);
            allowed.push(["unit/unit", "floor"]);
        }
        let target = self.tool_unit_pose();
        let aligned = if self.live_parking {
            (quaternion(target) * glam::DVec3::Z).dot(quaternion(self.unit_pose) * glam::DVec3::Z)
                >= 5f64.to_radians().cos()
        } else {
            quaternion(target).angle_between(quaternion(self.unit_pose)) <= 5f64.to_radians()
        };
        if self.gripped
            || (self.grip_requested || self.live_parking)
                && position(target).distance(position(self.unit_pose))
                    <= if self.live_parking { 0.008 } else { 0.005 }
                && aligned
        {
            allowed.push(["engineer/jaw-left", "unit/unit"]);
            allowed.push(["engineer/jaw-right", "unit/unit"]);
        }
        let socket = compose(
            self.head_delta(self.lift_m, self.p_rad, self.q_rad),
            self.config.core.tool,
        );
        let axis = quaternion(socket) * glam::DVec3::X;
        let offset = position(self.unit_pose) - position(socket);
        let axial = offset.dot(axis);
        let lateral = (offset - axis * axial).length();
        let normal = quaternion(self.unit_pose) * glam::DVec3::Z;
        if lateral <= 0.003
            && (-0.028..=0.078).contains(&axial)
            && normal.dot(axis) >= 5f64.to_radians().cos()
        {
            allowed.push(["unit/unit", "core/q"]);
        }
        allowed
    }
    pub fn collision_contacts(&self) -> Result<Vec<CollisionHit>, &'static str> {
        self.collision.as_ref().map_or_else(
            || Ok(vec![]),
            |scene| scene.contacts(&self.world_frames(), &self.allowed_contacts(), 0.0005),
        )
    }
    fn first_collision(&self) -> Result<Option<CollisionHit>, &'static str> {
        self.collision.as_ref().map_or(Ok(None), |scene| {
            scene.first_contact(&self.world_frames(), &self.allowed_contacts(), 0.0005)
        })
    }
    fn tool_unit_pose(&self) -> Pose {
        if let Some(grasp) = self.held_grasp {
            return compose(self.tool, grasp);
        }
        pose(
            position(self.tool) - quaternion(self.tool) * glam::DVec3::Z * self.grasp_axial_m,
            quaternion(self.tool) * DQuat::from_rotation_z(-self.grasp_roll_rad),
        )
    }
    /// Import numeric IK warm starts from a completed recording of the same source calibration.
    /// Hints never confer collision clearance; every proposed new path is checked normally.
    pub fn load_ik_hints(&mut self, path: &Path) -> Result<(), Box<dyn std::error::Error>> {
        let text = std::fs::read_to_string(path)?;
        let rows: Vec<serde_json::Value> = text
            .lines()
            .map(serde_json::from_str)
            .collect::<Result<_, _>>()?;
        if self.source_config_sha256.is_none()
            || rows.len() < 2
            || rows[0]["manifest"]["files"]["config"]["sha256"].as_str()
                != self.source_config_sha256.as_deref()
            || rows.last().unwrap()["completed"] != true
        {
            return Err(
                "IK hints require a completed recording of the same source calibration".into(),
            );
        }
        let mut hints = vec![];
        for row in &rows[1..] {
            let body: Pose = serde_json::from_value(row["cad"]["engineer_body"].clone())?;
            let tool: Pose = serde_json::from_value(row["cad"]["tool"].clone())?;
            let q: [f64; 6] = serde_json::from_value(row["cad"]["engineer_q_rad"].clone())?;
            hints.push(checked_ik_hint(&self.config.engineer, body, tool, q)?);
        }
        self.ik_hints = std::sync::Arc::new(hints);
        let route = &rows[0]["planner"]["statistics"]["selected_route"];
        self.preferred_spin_deltas_rad = recorded_spin_deltas(route);
        Ok(())
    }
    /// Prescribe five stage twist increments (approach, insertion, lift, P, Q), radians.
    /// Each is bounded to +/-pi as an inspection policy, not a measured motor limit.
    pub fn configure_spin(&mut self, deltas: [f64; 5]) -> Result<(), &'static str> {
        if self.gripped
            || !deltas
                .into_iter()
                .all(|v| v.is_finite() && v.abs() <= std::f64::consts::PI)
        {
            return Err(
                "initial spin increments require finite angles within +/-pi before gripping",
            );
        }
        self.requested_spin_deltas_rad = Some(deltas);
        Ok(())
    }
    /// Configure an initial relative planar parking pose before planning or time advances.
    /// X/Y are rig metres, yaw is radians; fixed parking still permits arm/grasp setup.
    pub fn configure_parking(&mut self, offset: [f64; 3], fixed: bool) -> Result<(), &'static str> {
        if self.gripped {
            return Err("initial parking cannot change while gripping");
        }
        if !offset.into_iter().all(f64::is_finite) {
            return Err("invalid chassis parking offset");
        }
        let mut candidate = self.clone();
        candidate.engineer_body = pose(
            position(self.engineer_body) + glam::DVec3::new(offset[0], offset[1], 0.),
            quaternion(self.engineer_body) * DQuat::from_rotation_z(offset[2]),
        );
        if !candidate.support_parking() {
            return Err("requested chassis parking has no level support");
        }
        candidate.initial_engineer_body = candidate.engineer_body;
        candidate.fixed_chassis = fixed;
        candidate.tool = compose(
            candidate.engineer_body,
            candidate.config.engineer.tool_pose(candidate.engineer_q),
        );
        *self = candidate;
        Ok(())
    }
    pub fn gripped(&self) -> bool {
        self.gripped
    }
    /// Prescribed jaw actuator and proximity-gated attachment, advanced on explicit ticks.
    /// A released unit stays at its last pose; this fixture does not simulate a falling load.
    fn advance_gripper(&mut self, close: bool, dt_s: f64) {
        const JAW_RATE_RAD_S: f64 = 1.5; // prototype servo policy, not a hardware measurement
        self.grip_requested = close;
        if !close {
            self.gripped = false;
        }
        for (angle, jaw) in self.jaw_angles.iter_mut().zip(&self.config.engineer_jaws) {
            let target = if close { jaw.closed_rad } else { jaw.open_rad };
            let travel = JAW_RATE_RAD_S * dt_s;
            let remaining = target - *angle;
            *angle = if remaining.abs() <= travel {
                target
            } else {
                *angle + remaining.signum() * travel
            };
        }
        let closed = close
            && !self.jaw_angles.is_empty()
            && self
                .jaw_angles
                .iter()
                .zip(&self.config.engineer_jaws)
                .all(|(a, j)| (a - j.closed_rad).abs() < 1e-10);
        let target = self.tool_unit_pose();
        if closed
            && position(target).distance(position(self.unit_pose)) <= 0.005
            && quaternion(target).angle_between(quaternion(self.unit_pose)) <= 5f64.to_radians()
        {
            self.gripped = true;
            self.unit_pose = target;
        }
    }
    pub fn step_gripper(&mut self, close: bool, dt_s: f64) -> Result<(), &'static str> {
        self.last_collision = None;
        if !dt_s.is_finite() || dt_s < 0. {
            return Err("invalid gripper time step");
        }
        let mut candidate = self.clone();
        candidate.advance_gripper(close, dt_s);
        self.accept_candidate(candidate)
    }
    /// Propose arm, payload, mechanism and jaw motion as one atomic change.
    pub fn drive(
        &mut self,
        logical: [f64; 3],
        angles: [f64; 3],
        close: bool,
        dt_s: f64,
    ) -> Result<(), &'static str> {
        self.last_collision = None;
        if !dt_s.is_finite() || dt_s < 0. {
            return Err("invalid gripper time step");
        }
        let mut candidate = self.clone();
        candidate.advance_gripper(close, dt_s);
        candidate.solve_tool(logical, angles)?;
        self.accept_candidate(candidate)
    }
    fn accept_candidate(&mut self, mut candidate: Self) -> Result<(), &'static str> {
        let body = candidate.engineer_body;
        if !body
            .translation_m
            .into_iter()
            .chain(body.rotation_wxyz)
            .all(f64::is_finite)
            || (body.rotation_wxyz.into_iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-8
        {
            return Err("invalid chassis pose");
        }
        let moving_body = self.engineer_body != body;
        if moving_body && (quaternion(body) * glam::DVec3::Z - glam::DVec3::Z).length() > 1e-8 {
            return Err("chassis route requires an upright body");
        }
        if let Some(scene) = &self.collision {
            let joint_travel = self
                .engineer_q
                .iter()
                .zip(candidate.engineer_q)
                .chain(self.core_q.iter().zip(candidate.core_q))
                .chain(
                    self.jaw_angles
                        .iter()
                        .zip(candidate.jaw_angles.iter().copied()),
                )
                .map(|(a, b)| (a - b).abs())
                .fold(0., f64::max)
                .max((self.p_rad - candidate.p_rad).abs())
                .max((self.q_rad - candidate.q_rad).abs())
                .max(quaternion(self.engineer_body).angle_between(quaternion(body)));
            let linear_travel = (self.lift_m - candidate.lift_m)
                .abs()
                .max(position(self.unit_pose).distance(position(candidate.unit_pose)))
                .max(position(self.engineer_body).distance(position(body)));
            // Simulator policy: live source-CAD sweeps use 1.43-degree / 5 mm
            // articulated subdivisions, with continuous rigid-frame CCD inside
            // each subdivision. Offline inspection retains 0.01 rad / 2 mm.
            let (angular_step_rad, linear_step_m) = if self.live_parking {
                (0.025, 0.005)
            } else {
                (0.01, 0.002)
            };
            let count = (joint_travel / angular_step_rad)
                .max(linear_travel / linear_step_m)
                .ceil()
                .max(1.) as usize;
            if count > 256 {
                return Err("motion exceeds collision subdivision budget");
            }
            let mut previous = self.world_frames();
            for i in 1..=count {
                let t = i as f64 / count as f64;
                let mix = |a: f64, b: f64| a + (b - a) * t;
                let mut sample = candidate.clone();
                sample.engineer_body = pose(
                    position(self.engineer_body).lerp(position(body), t),
                    quaternion(self.engineer_body).slerp(quaternion(body), t),
                );
                if moving_body {
                    let expected = sample.engineer_body;
                    if !sample.support_parking()
                        || (sample.engineer_body.translation_m[2] - expected.translation_m[2]).abs()
                            > 1e-6
                    {
                        return Err("chassis route leaves its level support patch");
                    }
                    sample.engineer_body = expected;
                }
                sample.engineer_q =
                    std::array::from_fn(|j| mix(self.engineer_q[j], candidate.engineer_q[j]));
                sample.core_q = std::array::from_fn(|j| mix(self.core_q[j], candidate.core_q[j]));
                sample.jaw_angles = self
                    .jaw_angles
                    .iter()
                    .zip(&candidate.jaw_angles)
                    .map(|(a, b)| mix(*a, *b))
                    .collect();
                sample.lift_m = mix(self.lift_m, candidate.lift_m);
                sample.p_rad = mix(self.p_rad, candidate.p_rad);
                sample.q_rad = mix(self.q_rad, candidate.q_rad);
                sample.tool = compose(
                    sample.engineer_body,
                    sample.config.engineer.tool_pose(sample.engineer_q),
                );
                sample.unit_pose = if self.gripped && candidate.gripped {
                    sample.tool_unit_pose()
                } else {
                    pose(
                        position(self.unit_pose).lerp(position(candidate.unit_pose), t),
                        quaternion(self.unit_pose).slerp(quaternion(candidate.unit_pose), t),
                    )
                };
                let next = sample.world_frames();
                if let Some(hit) =
                    scene.check_motion(&previous, &next, &sample.allowed_contacts(), 0.0005)?
                {
                    self.last_collision = Some(hit);
                    return Err("motion blocked by collision");
                }
                previous = next;
            }
        }
        candidate.last_collision = None;
        *self = candidate;
        Ok(())
    }
    pub fn restart(&mut self) {
        self.last_collision = None;
        self.cached_guard_replays = 0;
        self.gripped = false;
        self.grip_requested = false;
        self.jaw_angles = self
            .config
            .engineer_jaws
            .iter()
            .map(|j| j.open_rad)
            .collect();
        self.engineer_body = self.initial_engineer_body;
        self.rod_spin_rad = 0.;
        self.engineer_q = self.initial_engineer_q;
        self.lift_m = 0.;
        self.p_rad = 0.;
        self.q_rad = 0.;
        self.position_error_m = 0.;
        self.angle_error_rad = 0.;
        self.peak_joint_step_rad = 0.;
        self.tool = compose(
            self.engineer_body,
            self.config.engineer.tool_pose(self.engineer_q),
        );
        self.unit_pose = self.tool_unit_pose();
    }
    fn core_delta(&self) -> Pose {
        compose(
            self.config.core.frames(self.core_q)[5],
            inverse(translation(self.config.core.axes[5].origin_m)),
        )
    }
    fn head_delta(&self, lift: f64, p: f64, q: f64) -> Pose {
        let l = translation(self.config.core_lift_axis.map(|v| v * lift));
        compose(
            self.core_delta(),
            compose(
                delta(self.config.core_p_origin_m, self.config.core_p_axis, p),
                compose(
                    l,
                    delta(self.config.core_q_origin_m, self.config.core_q_axis, q),
                ),
            ),
        )
    }
    fn target(&self, logical: [f64; 3], angles: [f64; 3]) -> Pose {
        let insertion = ENTRANCE_X - logical[0];
        let center = compose(
            self.config.core.tool,
            Pose::at([0.075 - insertion, logical[1] - 0.2, 0.]),
        );
        let rotation = pose(
            glam::DVec3::ZERO,
            DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2)
                * DQuat::from_rotation_z(
                    self.rod_spin_rad
                        + if self.held_grasp.is_some() {
                            0.
                        } else {
                            self.grasp_roll_rad
                        },
                )
                * DQuat::from_rotation_x(angles[2]),
        );
        let unit = compose(
            self.head_delta(logical[2] - 0.6, angles[0], angles[1]),
            compose(center, rotation),
        );
        if let Some(grasp) = self.held_grasp {
            compose(unit, inverse(grasp))
        } else {
            compose(unit, Pose::at([0., 0., self.grasp_axial_m]))
        }
    }
    fn solve_tool(&mut self, logical: [f64; 3], angles: [f64; 3]) -> Result<(), &'static str> {
        let target = self.target(logical, angles);
        let q = self.config.engineer.solve(
            compose(inverse(self.engineer_body), target),
            self.engineer_q,
        )?;
        let actual = compose(self.engineer_body, self.config.engineer.tool_pose(q));
        self.position_error_m = position(actual).distance(position(target));
        self.angle_error_rad = quaternion(actual).angle_between(quaternion(target));
        self.peak_joint_step_rad = self.peak_joint_step_rad.max(
            q.into_iter()
                .zip(self.engineer_q)
                .map(|(a, b)| (a - b).abs())
                .fold(0., f64::max),
        );
        self.engineer_q = q;
        self.tool = actual;
        if self.gripped {
            self.unit_pose = self.tool_unit_pose();
            self.lift_m = logical[2] - 0.6;
            self.p_rad = angles[0];
            self.q_rad = angles[1];
        }
        Ok(())
    }
    /// Absolute source link frames, shared by offline inspection and GPU presentation.
    pub fn world_frames(&self) -> BTreeMap<String, Pose> {
        let mut frames = self.frames();
        for (name, frame) in &mut frames {
            if name.starts_with("engineer/") {
                *frame = compose(self.engineer_body, *frame);
            }
        }
        frames.insert("unit/unit".into(), self.unit_pose);
        frames.insert(
            "floor".into(),
            if self.area.is_some() {
                Pose::default()
            } else {
                Pose::at([0., 0., -0.05])
            },
        );
        frames
    }
    pub fn frames(&self) -> BTreeMap<String, Pose> {
        self.runtime_pose().frames(&self.config)
    }
}
impl Rig {
    pub(crate) fn runtime_pose(&self) -> rm_simulator_world::assembly::RigPose {
        rm_simulator_world::assembly::RigPose {
            engineer_body: self.engineer_body,
            engineer_q: self.engineer_q,
            core_q: self.core_q,
            unit_pose: self.unit_pose,
            jaw_angles: [
                self.jaw_angles.first().copied().unwrap_or(0.),
                self.jaw_angles.get(1).copied().unwrap_or(0.),
            ],
            camera_deployed: self.camera_deployed,
            lift_m: self.lift_m,
            p_rad: self.p_rad,
            q_rad: self.q_rad,
            gripped: self.gripped,
        }
    }
    pub(crate) fn runtime_model(&self) -> Config {
        let mut model = self.config.clone();
        // Source envelope witnesses are load-time checks, not motion geometry.
        model.core_envelope_m.clear();
        model
    }
    pub(crate) fn set_live_parking(&mut self, body: Pose) -> Result<(), &'static str> {
        let mut candidate = self.clone();
        candidate.engineer_body = body;
        if !candidate.support_parking()
            || (candidate.engineer_body.translation_m[2] - body.translation_m[2]).abs() > 0.02
        {
            return Err("Park on the level assembly deck before solving");
        }
        if (quaternion(body) * glam::DVec3::Z - glam::DVec3::Z).length() > 0.03 {
            return Err("Park the Heavy level before solving");
        }
        candidate.engineer_body = body;
        candidate.initial_engineer_body = body;
        candidate.fixed_chassis = true;
        candidate.live_parking = true;
        // Preloaded fixture uses the source-validated 20 mm pad offset, fixed
        // before the payload appears or any setup search begins.
        candidate.grasp_axial_m = 0.02;
        candidate.grasp_roll_rad = 3. * std::f64::consts::FRAC_PI_4;
        *self = candidate;
        Ok(())
    }
    pub(crate) fn recovery_core_joints(&self) -> Result<[[f64; 6]; 2], &'static str> {
        // UC §5.4.3 initial OXYZ pose is the return destination. The discharge
        // point is fitted above the central recessed collection opening behind
        // Fig4-29's curved front wall: 450 mm behind O, centred laterally, 650 mm
        // above the standing deck. These recovery offsets are simulator policy.
        let idle = compose(
            self.config.assembly_frame,
            Pose::at(TargetPose::INITIAL.position_m),
        );
        let drop = compose(
            self.config.assembly_frame,
            pose(
                glam::DVec3::new(-0.45, 0., 0.65),
                DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2),
            ),
        );
        let idle_q = self
            .config
            .core
            .solve(idle, self.core_q)
            .or_else(|_| solve_initial(&self.config.core, idle))?;
        let drop_q = self
            .config
            .core
            .solve(drop, idle_q)
            .or_else(|_| solve_initial(&self.config.core, drop))?;
        Ok([idle_q, drop_q])
    }
    pub(crate) fn select_live_core_branch(&mut self) -> Result<(), &'static str> {
        for q in rm_simulator_physics::manipulator_plan::ik_candidates(
            &self.config.core,
            self.socket,
            initial_seeds(),
            8,
        ) {
            let mut trial = self.clone();
            trial.core_q = q;
            if !trial.collision_contacts()?.iter().any(|h| {
                [&h.first, &h.second]
                    .into_iter()
                    .all(|n| !n.starts_with("engineer/") && n.as_str() != "unit/unit")
            }) {
                self.core_q = q;
                return Ok(());
            }
        }
        Err("no clear Core positioning branch within setup budget")
    }
    pub(crate) fn validate_live_jog(
        &self,
        body: Pose,
        arm: &rm_simulator_world::assembly::ArmState,
        target: [f64; 6],
        unit: Option<&rm_simulator_world::energy_unit::EnergyUnit>,
        world_from_rig: Pose,
        station: Option<&rm_simulator_world::assembly::RigPose>,
    ) -> Result<(), &'static str> {
        let mut from = self.clone();
        from.engineer_body = body;
        from.live_parking = true;
        from.engineer_q = arm.joints_rad;
        from.jaw_angles = arm.jaws_rad.to_vec();
        from.tool = compose(body, from.config.engineer.tool_pose(from.engineer_q));
        from.gripped = false;
        from.unit_pose = Pose::at([100.; 3]);
        if let Some(unit) = unit {
            from.unit_pose = compose(inverse(world_from_rig), unit.physical.pose);
            from.gripped =
                unit.attachment == rm_simulator_world::energy_unit::Attachment::Held(arm.owner);
            if from.gripped {
                from.held_grasp = Some(unit.grasp);
            }
        }
        if let Some(station) = station {
            from.core_q = station.core_q;
            from.lift_m = station.lift_m;
            from.p_rad = station.p_rad;
            from.q_rad = station.q_rad;
        }
        let mut next = from.clone();
        next.engineer_q = target;
        next.tool = compose(body, next.config.engineer.tool_pose(target));
        if next.gripped {
            next.unit_pose = next.tool_unit_pose();
        }
        from.accept_candidate(next)
    }
    pub(crate) fn use_held_unit(&mut self, unit: &rm_simulator_world::energy_unit::EnergyUnit) {
        self.held_grasp = Some(unit.grasp);
    }
    pub(crate) fn set_live_entry(&mut self, arm: &rm_simulator_world::assembly::ArmState) {
        self.live_entry = Some((arm.joints_rad, arm.jaws_rad));
    }
    // Operator-loaded payload fixture: its grasp is fixed before any planning.
    // This does not seat the Unit on the Core or advance insertion evidence.
    pub(crate) fn carried_start(&self, arm: &rm_simulator_world::assembly::ArmState) -> Self {
        let mut carried = self.clone();
        carried.engineer_q = arm.joints_rad;
        carried.jaw_angles = arm.jaws_rad.to_vec();
        carried.tool = compose(
            carried.engineer_body,
            carried.config.engineer.tool_pose(carried.engineer_q),
        );
        carried.unit_pose = carried.tool_unit_pose();
        carried.gripped = true;
        carried.grip_requested = true;
        carried
    }
    pub(crate) fn adopt_runtime_pose(
        &mut self,
        pose: &rm_simulator_world::assembly::RigPose,
    ) -> Result<(), &'static str> {
        self.engineer_body = pose.engineer_body;
        self.engineer_q = pose.engineer_q;
        self.core_q = pose.core_q;
        self.jaw_angles = pose.jaw_angles.to_vec();
        self.lift_m = pose.lift_m;
        self.p_rad = pose.p_rad;
        self.q_rad = pose.q_rad;
        self.gripped = pose.gripped;
        self.unit_pose = pose.unit_pose;
        self.tool = compose(
            self.engineer_body,
            self.config.engineer.tool_pose(self.engineer_q),
        );
        self.live_entry = None;
        if !self.gripped {
            return Err("post-flip planning requires the existing rigid grasp");
        }
        let expected = self.tool_unit_pose();
        if position(expected).distance(position(self.unit_pose)) > 1e-4
            || quaternion(expected)
                .normalize()
                .angle_between(quaternion(self.unit_pose).normalize())
                > 1e-4
        {
            return Err("actual attached Unit disagrees with the retained grasp");
        }
        self.unit_pose = expected;
        Ok(())
    }
    pub(crate) fn prepare_live_gripper(&mut self) -> Result<(), &'static str> {
        self.engineer_q = solve_initial(
            &self.config.engineer,
            compose(
                inverse(self.engineer_body),
                self.target([0.09, 0.2, 0.6], [0.; 3]),
            ),
        )?;
        self.initial_engineer_q = self.engineer_q;
        self.tool = compose(
            self.engineer_body,
            self.config.engineer.tool_pose(self.engineer_q),
        );
        self.unit_pose = self.tool_unit_pose();
        Ok(())
    }
    pub(crate) fn runtime_frames(
        &self,
        plan: &AssemblyPlan,
    ) -> Result<Vec<rm_simulator_world::assembly::Frame>, &'static str> {
        let mut rig = self.clone();
        plan.frames
            .iter()
            .map(|frame| {
                if frame.chassis != self.engineer_body {
                    return Err("runtime plan must preserve manual parking");
                }
                rig.follow_plan(frame)?;
                Ok(rm_simulator_world::assembly::Frame {
                    // Micrometre/microradian motion samples keep the complete trajectory
                    // small; below a micrometre/radian-millionth on this source rig.
                    joints_rad: rig.engineer_q.map(|v| (v * 1e6).round() / 1e6),
                    jaws_rad: [rig.jaw_angles[0], rig.jaw_angles[1]]
                        .map(|v| (v * 1e6).round() / 1e6),
                    unit: Pose {
                        translation_m: rig.unit_pose.translation_m.map(|v| (v * 1e6).round() / 1e6),
                        rotation_wxyz: rig.unit_pose.rotation_wxyz.map(|v| (v * 1e7).round() / 1e7),
                    },
                    guide_m: frame.logical.map(|v| (v * 1e6).round() / 1e6),
                    angles_rad: frame.angles.map(|v| (v * 1e6).round() / 1e6),
                    gripped: rig.gripped,
                    stage: frame.stage as u8,
                })
            })
            .collect()
    }
}
impl Rig {
    pub(crate) fn remember_runtime_plan(&mut self, plan: &AssemblyPlan) {
        let mut hints = (*self.ik_hints).clone();
        let mut previous = self.engineer_q;
        let mut tick = 0usize;
        for frame in &plan.frames {
            for i in 1..=frame.ticks {
                if tick.is_multiple_of(4) {
                    let t = i as f64 / frame.ticks as f64;
                    let q = std::array::from_fn(|j| previous[j] + (frame.q[j] - previous[j]) * t);
                    hints.push((self.config.engineer.tool_pose(q), q));
                }
                tick += 1;
            }
            previous = frame.q;
        }
        hints.truncate(4096);
        self.ik_hints = std::sync::Arc::new(hints);
    }
    pub(crate) fn reuse_runtime_hints(&mut self, previous: &Self) {
        if self.config == previous.config {
            let mut hints = (*self.ik_hints).clone();
            hints.extend(previous.ik_hints.iter().copied());
            hints.truncate(4096);
            self.ik_hints = std::sync::Arc::new(hints);
        }
        // Positive P-stage axial twist is a known useful profile, not a clearance
        // certificate. Every new request still searches and validates its path.
        self.preferred_spin_deltas_rad = Some([0., 0., 0., std::f64::consts::FRAC_PI_2, 0.]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    pub(super) fn mechanism_fixture() -> Rig {
        use rm_simulator_physics::manipulator::RevoluteAxis;
        let arm = SerialArm {
            axes: [RevoluteAxis {
                origin_m: [0.; 3],
                axis: [0., 0., 1.],
                range_rad: [-3.2, 3.2],
            }; 6],
            tool: Pose::default(),
        };
        Rig {
            source_config_sha256: None,
            area: None,
            camera_deployed: false,
            fixed_chassis: false,
            live_parking: false,
            live_entry: None,
            held_grasp: None,
            ik_hints: Default::default(),
            requested_spin_deltas_rad: None,
            preferred_spin_deltas_rad: None,
            config: Config {
                schema: 5,
                assembly_frame: Pose::default(),
                core_envelope_m: vec![],
                engineer: arm.clone(),
                core: arm,
                engineer_jaws: vec![],
                engineer_auxiliary: vec![],
                core_p_origin_m: [0., 0., 0.1],
                core_p_axis: [0., -1., 0.],
                core_q_origin_m: [0.; 3],
                core_q_axis: [0., 0., 1.],
                core_lift_axis: [0., 0., 1.],
            },
            engineer_q: [0.; 6],
            core_q: [0.; 6],
            initial_engineer_q: [0.; 6],
            engineer_body: Pose::default(),
            initial_engineer_body: Pose::default(),
            tool: Pose::default(),
            socket: Pose::default(),
            requested_pose: None,
            position_error_m: 0.,
            angle_error_rad: 0.,
            peak_joint_step_rad: 0.,
            cached_guard_replays: 0,
            lift_m: 0.,
            p_rad: 0.,
            q_rad: 0.,
            jaw_angles: vec![],
            gripped: false,
            grasp_roll_rad: GRASP_ROLL_RAD,
            rod_spin_rad: 0.,
            grasp_axial_m: 0.,
            grip_requested: false,
            unit_pose: Pose::default(),
            collision: None,
            last_collision: None,
        }
    }
    #[test]
    fn recorded_twist_preference_accepts_legacy_fields_and_rejects_bad_metadata() {
        assert_eq!(
            recorded_spin_deltas(&serde_json::json!({"flip_spin_rad":1.5,"lift_spin_rad":0.})),
            Some([0., 0., 0., 1.5, 0.])
        );
        assert_eq!(
            recorded_spin_deltas(
                &serde_json::json!({"approach_spin_rad":0.2,"insert_spin_rad":0.3,"lift_spin_rad":0.4,"flip_spin_rad":0.5,"align_spin_rad":0.6})
            ),
            Some([0.2, 0.3, 0.4, 0.5, 0.6])
        );
        assert_eq!(recorded_spin_deltas(&serde_json::Value::Null), None);
        assert_eq!(
            recorded_spin_deltas(&serde_json::json!({"flip_spin_rad":"invalid"})),
            None
        );
    }
    #[test]
    fn imported_ik_hints_reject_bad_quaternions_limits_and_inconsistent_fk() {
        let arm = mechanism_fixture().config.engineer;
        let tool = arm.tool_pose([0.; 6]);
        assert!(checked_ik_hint(&arm, Pose::default(), tool, [0.; 6]).is_ok());
        let invalid = Pose {
            rotation_wxyz: [0.; 4],
            ..Pose::default()
        };
        assert!(checked_ik_hint(&arm, invalid, tool, [0.; 6]).is_err());
        assert!(checked_ik_hint(&arm, Pose::default(), tool, [f64::NAN; 6]).is_err());
        assert!(checked_ik_hint(&arm, Pose::default(), tool, [100.; 6]).is_err());
        assert!(checked_ik_hint(&arm, Pose::default(), Pose::at([0.1, 0., 0.]), [0.; 6]).is_err());
    }
    #[test]
    fn configuring_initial_parking_preserves_the_unit_and_rejects_live_teleports() {
        let mut rig = mechanism_fixture();
        let unit = rig.unit_pose;
        rig.configure_parking([0.1, -0.2, 0.3], true).unwrap();
        assert_eq!(rig.unit_pose, unit);
        assert!(rig.fixed_chassis);
        assert!(rig.configure_spin([f64::NAN; 5]).is_err());
        rig.configure_spin([0., 0., 0., 1., 0.]).unwrap();
        assert_eq!(rig.requested_spin_deltas_rad, Some([0., 0., 0., 1., 0.]));
        assert_eq!(rig.initial_engineer_body, rig.engineer_body);
        rig.gripped = true;
        let before = rig.report();
        assert!(rig.configure_parking([0.1, 0., 0.], true).is_err());
        assert_eq!(rig.report(), before);
    }
    #[test]
    fn blocked_transition_rolls_back_payload_arm_and_jaws_and_can_recover() {
        use rm_simulator_physics::manipulator_collision::{CollisionBody, CollisionPart};
        let mut rig = mechanism_fixture();
        rig.unit_pose = Pose::at([-0.1, 0., 0.]);
        rig.collision = Some(std::sync::Arc::new(
            CollisionScene::new(CollisionAsset {
                schema: 1,
                bodies: vec![
                    CollisionBody {
                        frame: "unit/unit".into(),
                        parts: vec![CollisionPart::Box { half_m: [0.005; 3] }],
                    },
                    CollisionBody {
                        frame: "core/body".into(),
                        parts: vec![CollisionPart::Box {
                            half_m: [0.0005, 0.04, 0.04],
                        }],
                    },
                ],
                excluded_pairs: vec![],
            })
            .unwrap(),
        ));
        let mut candidate = rig.clone();
        candidate.unit_pose = Pose::at([0.1, 0., 0.]); // both endpoints clear
        candidate.engineer_q[0] = 0.2;
        candidate.jaw_angles = vec![0.1];
        candidate.grip_requested = true;
        assert!(rig.accept_candidate(candidate).is_err());
        assert_eq!(rig.unit_pose, Pose::at([-0.1, 0., 0.]));
        assert_eq!(rig.engineer_q, [0.; 6]);
        assert!(rig.jaw_angles.is_empty());
        assert!(!rig.grip_requested);
        assert_eq!(rig.last_collision.as_ref().unwrap().first, "unit/unit");
        let mut safe = rig.clone();
        safe.unit_pose = Pose::at([-0.1, 0.01, 0.]);
        rig.accept_candidate(safe).unwrap();
        assert!(rig.last_collision.is_none());
        assert_eq!(rig.unit_pose, Pose::at([-0.1, 0.01, 0.]));
    }
    #[test]
    fn flip_uses_fixed_hinge_and_q_tilts_the_raised_unit() {
        use std::f64::consts::{FRAC_PI_2, FRAC_PI_6};
        let rig = mechanism_fixture();
        let raised = rig.head_delta(0.1, FRAC_PI_2, 0.);
        // A carriage point lifted into P stays on that hinge during the flip.
        assert!(position(raised).distance(glam::DVec3::new(0., 0., 0.1)) < 1e-12);
        let center = position(raised) + quaternion(raised) * glam::DVec3::new(0.075, 0., 0.);
        assert!(center.distance(glam::DVec3::new(0., 0., 0.175)) < 1e-12);
        let up = quaternion(raised) * glam::DVec3::X;
        assert!(up.distance(glam::DVec3::Z) < 1e-12);
        let tilted = rig.head_delta(0.1, FRAC_PI_2, FRAC_PI_6);
        let normal = quaternion(tilted) * glam::DVec3::X;
        // Fig 5-15: Q changes the shaft normal; it does not roll around it.
        assert!(normal.distance(glam::DVec3::new(0., 0.5, 3f64.sqrt() / 2.)) < 1e-12);
    }
    #[test]
    fn rulebook_side_view_matches_208_169_54_and_15_mm_dimensions() {
        let mut rig = mechanism_fixture();
        // 2026 EN V2.1.0 Fig 5-7, printed p82; same dimensioned drawing in
        // RMUC2027 Fig 5-7. Source panel face is x=0, initial post is z=0.
        rig.config.core_p_origin_m = [0.015, 0., 0.208];
        let lifted = rig.head_delta(0.1, 0., 0.);
        let flipped = rig.head_delta(0.1, std::f64::consts::FRAC_PI_2, 0.);
        let point = |d: Pose, p: [f64; 3]| position(d) + quaternion(d) * glam::DVec3::from_array(p);
        assert!(point(lifted, [0.169, 0., 0.]).distance(glam::DVec3::new(0.169, 0., 0.1)) < 1e-12);
        let tip = point(flipped, [0.169, 0., 0.]);
        assert!(tip.distance(glam::DVec3::new(0.123, 0., 0.362)) < 1e-12);
        let front = point(flipped, [0.069, 0., 0.]); // 169 mm reach minus 100 mm insertion
        assert!((front.z - 0.208 - 0.054).abs() < 1e-12);
        let back = point(flipped, [0., 0., 0.]);
        assert!((0.208 - back.z - 0.015).abs() < 1e-12);
        assert!((front.x - 0.123).abs() < 1e-12);
    }
    #[test]
    fn jaw_actuation_requires_alignment_then_releases_and_regrips() {
        let mut rig = mechanism_fixture();
        rig.config.engineer_jaws = vec![
            Jaw {
                name: "jaw-left".into(),
                origin_m: [0.; 3],
                axis: [0., 0., 1.],
                open_rad: -0.5,
                closed_rad: -0.1,
            },
            Jaw {
                name: "jaw-right".into(),
                origin_m: [0.; 3],
                axis: [0., 0., 1.],
                open_rad: 0.6,
                closed_rad: 0.15,
            },
        ];
        rig.jaw_angles = vec![-0.5, 0.6];
        rig.unit_pose = rig.tool_unit_pose();
        rig.step_gripper(true, 0.01).unwrap();
        assert!(!rig.gripped());
        assert!((rig.jaw_angles[0] + 0.485).abs() < 1e-12);
        rig.step_gripper(true, 1.).unwrap();
        assert!(rig.gripped());
        assert_eq!(rig.jaw_angles, vec![-0.1, 0.15]);
        let held = rig.unit_pose;
        rig.step_gripper(false, 0.01).unwrap();
        assert!(!rig.gripped());
        rig.tool.translation_m[0] += 0.02;
        rig.step_gripper(false, 1.).unwrap();
        assert_eq!(rig.unit_pose, held); // released unit does not follow an empty gripper
        assert_eq!(rig.jaw_angles, vec![-0.5, 0.6]);
        rig.step_gripper(true, 1.).unwrap();
        assert!(!rig.gripped()); // closing remotely cannot claim attachment
        rig.tool.translation_m[0] -= 0.02;
        rig.tool.rotation_wxyz = [0., 1., 0., 0.];
        rig.step_gripper(true, 1.).unwrap();
        assert!(!rig.gripped()); // proximity alone does not align the shaft
        rig.tool.rotation_wxyz = [1., 0., 0., 0.];
        rig.step_gripper(true, 1.).unwrap();
        assert!(rig.gripped());
        assert_eq!(rig.unit_pose, held);
        rig.restart();
        assert!(!rig.gripped());
        assert_eq!(rig.jaw_angles, vec![-0.5, 0.6]);
    }
}
