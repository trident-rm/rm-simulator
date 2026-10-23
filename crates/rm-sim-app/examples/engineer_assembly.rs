// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Standalone Cartesian assembly workbench, not a multiplayer match or Heavy admission.
//! `cargo run -p rm-sim-app --example engineer_assembly -- --help`
use bevy::render::{Render, RenderApp, RenderSystems, render_resource::PipelineCache};
use bevy::{
    prelude::*,
    render::view::screenshot::{Screenshot, save_to_disk},
};
use clap::Parser;
use rm_sim_physics::{Pose, manipulator::Arm, tick_ns};
use rm_sim_render::{PoseFlu, apply_pose, flu_position};
use rm_sim_world::gameplay::rmuc_2027::{
    assembly_pose::Targets,
    manipulation::{Attempt, Sensors, Step},
};
use std::{
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
};
#[derive(Resource, Clone, Default)]
struct CaptureReady(Arc<AtomicBool>);
struct CapturePlugin;
impl Plugin for CapturePlugin {
    fn build(&self, app: &mut App) {
        let ready = CaptureReady::default();
        app.insert_resource(ready.clone());
        app.sub_app_mut(RenderApp)
            .insert_resource(ready)
            .add_systems(Render, pipeline_ready.in_set(RenderSystems::Cleanup));
    }
}
fn pipeline_ready(cache: Res<PipelineCache>, ready: Res<CaptureReady>) {
    ready
        .0
        .store(cache.waiting_pipelines().count() == 0, Ordering::Release);
}
mod assembly_cad;
#[allow(dead_code)]
#[path = "../src/robot_assets.rs"]
mod robot_assets;

#[derive(Parser, Resource)]
#[command(about = "Cartesian Engineer assembly workbench (prototype geometry, no match rewards)")]
struct Options {
    /// Initial difficulty, 1–3. Paired-core level four is evaluator-only for now.
    #[arg(long, default_value_t=1, value_parser=clap::value_parser!(u8).range(1..=3))]
    difficulty: u8,
    /// Verified PIE source package: chassis/wheels only; arm uses a generic IK proxy.
    #[arg(long)]
    robot_assets: Option<PathBuf>,
    /// Source-preserved Engineer arm, Tech Core and Energy Unit workbench package.
    #[arg(long, conflicts_with = "robot_assets")]
    assembly_assets: Option<PathBuf>,
    /// Verified central-highland terrain extraction, in the source rig frame.
    #[arg(long, requires = "assembly_assets")]
    assembly_area: Option<PathBuf>,
    /// Use conservative convex collision chunks instead of source triangles.
    #[arg(long, requires = "assembly_assets")]
    fast_collisions: bool,
    /// Demonstrate the full sequence with rate-limited Cartesian movements.
    #[arg(long)]
    demo: bool,
    /// Run the same demonstration without rendering, print state and exit.
    #[arg(long)]
    headless: bool,
    /// Save the completed or blocked demonstration and exit (implies --demo).
    #[arg(long)]
    screenshot: Option<PathBuf>,
    /// Write solved source-CAD frames at 32 Hz, including the initial and final poses.
    #[arg(long, requires_all = ["headless", "assembly_assets"])]
    record: Option<PathBuf>,
    /// Reproducible rulebook pose selection; failed CAD/IK checks never resample.
    #[arg(long, requires = "assembly_assets")]
    pose_seed: Option<u64>,
    /// Explicit rulebook x/y/z metres and theta/phi/alpha degrees for a frozen Core pose.
    #[arg(
        long,
        num_args = 6,
        allow_hyphen_values = true,
        requires = "assembly_assets",
        conflicts_with = "pose_seed"
    )]
    core_pose: Option<Vec<f64>>,
    /// Explicit Q target in degrees, within +/-90 (difficulty three only).
    #[arg(long, allow_hyphen_values = true, requires = "assembly_assets")]
    q_deg: Option<f64>,
    /// Initial relative chassis dx/dy metres and yaw degrees in the workbench frame.
    #[arg(
        long,
        num_args = 3,
        allow_hyphen_values = true,
        requires = "assembly_area"
    )]
    chassis_offset: Option<Vec<f64>>,
    /// Completed same-calibration recording used only for IK warm starts.
    #[arg(long, requires = "assembly_assets")]
    ik_hints: Option<PathBuf>,
    /// Prescribed twist increments in degrees: approach, insertion, lift, P flip, Q alignment.
    #[arg(
        long,
        num_args = 5,
        allow_hyphen_values = true,
        requires = "assembly_area"
    )]
    spin_deltas: Option<Vec<f64>>,
    /// Keep chassis parking fixed while allowing arm, grasp and axial-spin planning.
    #[arg(long, requires = "assembly_area")]
    fixed_chassis: bool,
    /// Explicit pose selection number, frozen throughout the attempt.
    #[arg(long, default_value_t = 0, requires = "pose_seed")]
    pose_index: u64,
    /// Export all four difficulty candidates as JSON, without loading CAD or solving IK.
    #[arg(long, conflicts_with_all = ["assembly_assets", "robot_assets", "record"])]
    generate_poses: Option<u64>,
    /// Print initial named collision contacts and exit without moving the fixture.
    #[arg(long, requires = "assembly_assets")]
    collision_audit: bool,
    /// Use the original Cartesian guide without offline path selection (diagnostics).
    #[arg(long, requires = "assembly_assets")]
    legacy_guide: bool,
    /// Prepare a staged source-CAD plan and pause between stages (P advances, O runs all).
    #[arg(long,requires="assembly_assets",conflicts_with_all=["demo","headless","legacy_guide","screenshot"])]
    assisted: bool,
    /// Preserve the current fixture; plan a real free-arm approach instead of setup search.
    #[arg(long, requires = "assembly_assets", conflicts_with = "legacy_guide")]
    plan_current: bool,
    /// Maximum guarded transitions tested by the staged planner.
    #[arg(long, default_value_t=8000, value_parser=clap::value_parser!(u32).range(1..=100000))]
    plan_budget: u32,
    /// Inspect the gripper and Core front mechanism at closer range.
    #[arg(long)]
    closeup: bool,
}
/// Validate explicit Table 5-10 coordinates before invoking any source-CAD solver.
fn explicit_core_pose(
    values: &[f64],
    difficulty: u8,
) -> Result<rm_sim_world::gameplay::rmuc_2027::assembly_pose::TargetPose, &'static str> {
    if values.len() != 6
        || !values.iter().all(|v| v.is_finite())
        || !(-0.1..=0.).contains(&values[0])
        || !(0.1..=0.3).contains(&values[1])
        || !(0.5..=0.7).contains(&values[2])
        || !(-90. ..=90.).contains(&values[3])
        || !(0. ..=90.).contains(&values[4])
        || !(-45. ..=45.).contains(&values[5])
        || difficulty == 1 && (values[3] != 0. || values[4] != 90. || values[5] != 0.)
    {
        return Err("explicit Core pose is outside the selected Table 5-10 range");
    }
    Ok(
        rm_sim_world::gameplay::rmuc_2027::assembly_pose::TargetPose {
            position_m: [values[0], values[1], values[2]],
            theta_rad: values[3].to_radians(),
            phi_rad: values[4].to_radians(),
            alpha_rad: values[5].to_radians(),
        },
    )
}
/// Reject requested twists for absent stages rather than silently dropping them.
fn validate_spin_stages(difficulty: u8, q: f64, values: &[f64]) -> Result<(), &'static str> {
    if values.len() != 5
        || difficulty < 2 && values[2] != 0.
        || difficulty < 3 && (values[3] != 0. || values[4] != 0.)
        || q.abs() <= 1e-9 && values[4] != 0.
    {
        return Err(
            "requested twist requires a moving stage at this difficulty and a nonzero Q target for alignment",
        );
    }
    Ok(())
}
const SOCKET: [f64; 3] = [-0.05, 0.2, 0.6];
const ENTRANCE_X: f64 = 0.05;
const SPEED: f64 = 0.12;
const RATE: f64 = 0.6;
const ARM: Arm = Arm {
    shoulder_m: [0.55, 0., 0.35],
    lengths_m: [0.4, 0.4],
};
type PlannedAssembly =
    Result<(assembly_cad::Rig, assembly_cad::AssemblyPlan), assembly_cad::PlanningFailure>;
type PendingPlan = Arc<std::sync::Mutex<Option<PlannedAssembly>>>;
#[derive(Resource)]
struct Workbench {
    task: Attempt,
    tool: Pose,
    angles: [f64; 3], // P, Q, off-guide roll
    attached: bool,
    tick: u64,
    paused: bool,
    message: String,
    unreachable: bool,
    capture_frames: u32,
    confirm_requested: bool,
    cad: Option<assembly_cad::Rig>,
    plan: Option<assembly_cad::AssemblyPlan>,
    plan_cursor: usize,
    automatic: bool,
    stage_run: bool,
    guide_detached: bool,
    planning_report: serde_json::Value,
    pending_plan: Option<PendingPlan>,
}
impl Workbench {
    fn new(d: u8) -> Self {
        Self {
            task: Attempt::new(d, [30f64.to_radians(); 2], 0).unwrap(),
            tool: Pose::at([0.09, 0.2, 0.6]),
            angles: [0.; 3],
            attached: true,
            tick: 0,
            paused: false,
            message: String::new(),
            unreachable: false,
            capture_frames: 0,
            confirm_requested: false,
            cad: None,
            plan: None,
            plan_cursor: 0,
            automatic: false,
            stage_run: false,
            guide_detached: false,
            planning_report: serde_json::Value::Null,
            pending_plan: None,
        }
    }
    fn step(&mut self, movement: [f64; 3], rotation: [f64; 3]) {
        if self.paused {
            return;
        }
        let dt = tick_ns() as f64 * 1e-9;
        if self.guide_detached {
            if let Some(cad) = &mut self.cad {
                match cad.step_gripper(self.attached, dt) {
                    Ok(()) => self.unreachable = false,
                    Err(reason) => {
                        self.unreachable = true;
                        self.message = reason.into();
                        return;
                    }
                }
            }
            self.tick += 1;
            self.observe();
            self.message =
                "Off guide: O/P to plan a return, or restart; Cartesian guide held".into();
            return;
        }
        if self.task.failure.is_some() || self.task.completed() {
            if let Some(cad) = &mut self.cad {
                self.unreachable = cad.step_gripper(self.attached, dt).is_err();
            }
            return;
        }
        let mut tool = self.tool;
        let angles = std::array::from_fn(|i| self.angles[i] + rotation[i] * RATE * dt);
        for (i, v) in movement.into_iter().enumerate() {
            tool.translation_m[i] += v * SPEED * dt;
        }
        let p = Quat::from_rotation_y(-angles[0] as f32)
            * Quat::from_rotation_x(angles[1] as f32)
            * Quat::from_rotation_z(angles[2] as f32);
        let q = DQuat::from_xyzw(p.x as f64, p.y as f64, p.z as f64, p.w as f64).normalize();
        tool.rotation_wxyz = [q.w, q.x, q.y, q.z];
        self.unreachable = tool.translation_m[2] < 0.15
            || if let Some(cad) = &mut self.cad {
                match cad.drive(tool.translation_m, angles, self.attached, dt) {
                    Ok(()) => false,
                    Err(reason) => {
                        self.message = reason.into();
                        true
                    }
                }
            } else {
                ARM.solve(tool).is_err()
            };
        if !self.unreachable {
            self.tool = tool;
            self.angles = angles;
        }
        self.tick += 1;
        self.observe();
    }
    fn observe(&mut self) {
        let [x, y, z] = self.tool.translation_m;
        let s = Sensors {
            attached: self.cad.as_ref().map_or(self.attached, |c| c.gripped()),
            insertion_m: ENTRANCE_X - x,
            lift_m: z - SOCKET[2],
            p_rad: self.angles[0],
            q_rad: self.angles[1],
            axis_error_rad: self.angles[2].abs()
                + self.cad.as_ref().map_or(0., |c| c.angle_error_rad),
            lateral_error_m: (y - SOCKET[1]).abs()
                + self.cad.as_ref().map_or(0., |c| c.position_error_m),
        };
        self.task
            .observe(self.tick * tick_ns(), [s, Sensors::default()], true, true)
            .unwrap();
    }
    fn start_plan(&mut self, prepare: bool, automatic: bool, budget: usize) {
        let Some(rig) = &self.cad else {
            self.message = "Source CAD required for collision-aware planning".into();
            return;
        };
        let result = rig.plan_assembly(
            self.task.difficulty,
            self.task.target_q_rad[0],
            self.tool.translation_m,
            self.angles,
            prepare,
            budget,
        );
        self.apply_plan(result, automatic);
    }
    fn apply_plan(&mut self, result: PlannedAssembly, automatic: bool) {
        match result {
            Ok((start, plan)) => {
                self.planning_report =
                    serde_json::json!({"status":"ready","statistics":plan.statistics});
                self.cad = Some(start);
                self.plan = Some(plan);
                self.plan_cursor = 0;
                self.automatic = automatic;
                self.stage_run = true;
                self.unreachable = false;
                self.message = "Collision-checked plan ready".into();
            }
            Err(failure) => {
                self.message =
                    format!("Planning failed at {:?}: {}", failure.stage, failure.reason);
                self.planning_report = serde_json::json!({"status":"failed","failure":failure});
                self.unreachable = true;
                self.automatic = false;
                self.stage_run = false;
            }
        }
    }
    fn cancel_plan(&mut self) {
        self.pending_plan = None;
        self.plan = None;
        self.automatic = false;
        self.stage_run = false;
        self.unreachable = false;
        if !self.planning_report.is_null() {
            self.planning_report = serde_json::json!({"status":"cancelled"});
        }
        self.message = "Plan cancelled; manual controls".into();
    }
    fn request_plan(&mut self, automatic: bool, budget: usize) {
        if self.pending_plan.is_some() {
            return;
        }
        let Some(rig) = self.cad.clone() else {
            self.message = "Source CAD required for planning".into();
            return;
        };
        let result = Arc::new(std::sync::Mutex::new(None));
        self.pending_plan = Some(result.clone());
        self.automatic = automatic;
        self.message = "Planning on worker; workbench clock held".into();
        let (d, q, logical, angles) = (
            self.task.difficulty,
            self.task.target_q_rad[0],
            self.tool.translation_m,
            self.angles,
        );
        std::thread::spawn(move || {
            let plan = rig.plan_assembly(d, q, logical, angles, false, budget);
            *result.lock().unwrap() = Some(plan);
        });
    }
    fn planned_step(&mut self) {
        if self.paused
            || (!self.automatic && !self.stage_run)
            || self.task.completed()
            || self.task.failure.is_some()
            || self.unreachable
        {
            return;
        }
        let Some(plan) = &self.plan else {
            return;
        };
        let Some(frame) = plan.frames.get(self.plan_cursor).cloned() else {
            if self.automatic {
                self.confirm();
                self.automatic = false;
            }
            self.stage_run = false;
            return;
        };
        match self.cad.as_mut().unwrap().follow_plan(&frame) {
            Ok(()) => {}
            Err(reason) => {
                self.unreachable = true;
                self.message = reason.into();
                return;
            }
        }
        self.guide_detached = frame.free;
        self.tool.translation_m = frame.logical;
        self.angles = frame.angles;
        self.plan_cursor += 1;
        self.tick += 1;
        self.observe();
        self.message = format!(
            "Plan {:?} | {}/{}",
            frame.stage,
            self.plan_cursor,
            self.plan.as_ref().unwrap().frames.len()
        );
        if !self.automatic
            && self
                .plan
                .as_ref()
                .unwrap()
                .frames
                .get(self.plan_cursor)
                .is_none_or(|next| next.stage != frame.stage)
        {
            self.stage_run = false;
            self.message.push_str(" | P: next stage, O: automatic");
        }
    }
    fn confirm(&mut self) {
        self.message = match self.task.confirm(0) {
            Ok(()) => "Assembly confirmed".into(),
            Err(reason) => reason.into(),
        };
    }
    fn demo(&mut self) {
        if self.plan.is_some() {
            self.planned_step();
            return;
        }
        if self.planning_report["status"] == "failed" {
            return;
        }
        if self.cad.as_ref().is_some_and(|c| !c.gripped()) && self.attached {
            self.step([0.; 3], [0.; 3]);
            return;
        }
        let mut target = SOCKET;
        let mut angles = [0.; 3];
        match self.task.cores[0].step {
            Step::Approach => target[0] = ENTRANCE_X,
            Step::Insert => {}
            Step::Lift => target[2] += 0.1,
            Step::Raise => {
                target[2] += 0.1;
                angles[0] = std::f64::consts::FRAC_PI_2;
            }
            Step::Align => {
                target[2] += 0.1;
                angles = [std::f64::consts::FRAC_PI_2, self.task.target_q_rad[0], 0.];
            }
            Step::Ready => {
                if self.task.difficulty >= 2 {
                    target[2] += 0.1;
                }
                if self.task.difficulty >= 3 {
                    angles = [std::f64::consts::FRAC_PI_2, self.task.target_q_rad[0], 0.];
                }
            }
            Step::Confirmed => return,
        }
        // Finish each exact waypoint before starting the next physical motion.
        // Referee tolerances may advance the evaluator a few millimetres/degrees
        // early; they must not blend lifting with flipping or P with Q.
        if target[2] > SOCKET[2] && (self.tool.translation_m[0] - SOCKET[0]).abs() > 1e-9 {
            target[2] = SOCKET[2];
            angles = [0.; 3];
        } else if angles[0] > 0. && (self.tool.translation_m[2] - target[2]).abs() > 1e-9 {
            angles = [0.; 3];
        } else if angles[1] != 0. && (self.angles[0] - angles[0]).abs() > 1e-9 {
            angles[1] = 0.;
        }
        if self.task.cores[0].step == Step::Ready
            && (0..3).all(|i| {
                (self.tool.translation_m[i] - target[i]).abs() < 1e-9
                    && (self.angles[i] - angles[i]).abs() < 1e-9
            })
        {
            self.confirm();
            return;
        }
        let dt = tick_ns() as f64 * 1e-9;
        let movement = std::array::from_fn(|i| {
            ((target[i] - self.tool.translation_m[i]) / (SPEED * dt)).clamp(-1., 1.)
        });
        let rotation =
            std::array::from_fn(|i| ((angles[i] - self.angles[i]) / (RATE * dt)).clamp(-1., 1.));
        self.step(movement, rotation);
    }
}
use bevy::math::DQuat;
#[derive(Resource, Default)]
struct CaptureImage(Option<Handle<Image>>);
#[derive(Component)]
struct Segment(u8);
#[derive(Component)]
struct Tool;
#[derive(Component)]
struct Status;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut options = Options::parse();
    options.demo |= options.screenshot.is_some();
    if let Some(seed) = options.generate_poses {
        let targets: Vec<_> = (1..=4)
            .map(|d| Targets::sample(seed, 0, d).unwrap())
            .collect();
        println!("{}", serde_json::to_string_pretty(&targets)?);
        return Ok(());
    }
    let mut workbench = Workbench::new(options.difficulty);
    if let Some(area) = &mut options.assembly_area {
        *area = area.canonicalize()?;
    }
    if let Some(path) = &mut options.assembly_assets {
        *path = path.canonicalize()?;
        let requested = if let Some(values) = &options.core_pose {
            Some(explicit_core_pose(values, options.difficulty)?)
        } else if let Some(seed) = options.pose_seed {
            let targets = Targets::sample(seed, options.pose_index, options.difficulty)?;
            workbench.task = Attempt::new(options.difficulty, [targets.q_rad[0]; 2], 0)?;
            Some(targets.cores[0])
        } else if options.assembly_area.is_some() {
            Some(
                rm_sim_world::gameplay::rmuc_2027::assembly_pose::TargetPose {
                    position_m: [-0.05, 0.2, 0.5],
                    theta_rad: 0.,
                    phi_rad: std::f64::consts::FRAC_PI_2,
                    alpha_rad: 0.,
                },
            )
        } else {
            None
        };
        workbench.cad = Some(if let Some(area) = &options.assembly_area {
            assembly_cad::Rig::load_in_area(path, requested.unwrap(), area)?
        } else if let Some(requested) = requested {
            assembly_cad::Rig::load_target(path, requested)?
        } else {
            assembly_cad::Rig::load(path)?
        });
    }
    if let Some(degrees) = options.q_deg {
        if options.difficulty != 3 || !degrees.is_finite() || degrees.abs() > 90. {
            return Err(
                "--q-deg requires difficulty three and a finite angle within +/-90 degrees".into(),
            );
        }
        workbench.task = Attempt::new(options.difficulty, [degrees.to_radians(); 2], 0)?;
    }
    if let Some(rig) = &mut workbench.cad {
        if options.fast_collisions {
            rig.use_convex_collisions();
        }
        if let Some(path) = &options.ik_hints {
            rig.load_ik_hints(path)?;
        }
        if let Some(values) = &options.spin_deltas {
            validate_spin_stages(options.difficulty, workbench.task.target_q_rad[0], values)?;
            rig.configure_spin(std::array::from_fn(|i| values[i].to_radians()))?;
        }
        let offset = options.chassis_offset.as_deref().unwrap_or(&[0.; 3]);
        if options.fixed_chassis || options.chassis_offset.is_some() {
            rig.configure_parking(
                [offset[0], offset[1], offset[2].to_radians()],
                options.fixed_chassis,
            )?;
        }
    }
    if options.collision_audit {
        println!(
            "{}",
            serde_json::to_string_pretty(&workbench.cad.as_ref().unwrap().collision_contacts()?)?
        );
        return Ok(());
    }
    if workbench.cad.is_some()
        && !options.legacy_guide
        && (options.headless || options.demo || options.assisted)
    {
        workbench.start_plan(
            !options.plan_current,
            !options.assisted,
            options.plan_budget as usize,
        );
    }
    if options.headless {
        let mut w = workbench;
        let mut recording = if let Some(path) = &options.record {
            use std::io::Write;
            if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
                std::fs::create_dir_all(parent)?;
            }
            let mut file = std::io::BufWriter::new(std::fs::File::create(path)?);
            let manifest: serde_json::Value = serde_json::from_slice(&std::fs::read(
                options
                    .assembly_assets
                    .as_ref()
                    .unwrap()
                    .join("manifest.json"),
            )?)?;
            writeln!(
                file,
                "{}",
                serde_json::json!({"schema":1,"tick_ns":tick_ns(),
                "sample_ticks":4,"difficulty":options.difficulty,"manifest":manifest,
                "case":"insert 100 mm, lift 100 mm, P 90 deg, then requested Q",
                "pose_seed":options.pose_seed,"pose_index":options.pose_index,
                "core_pose":options.core_pose,"chassis_offset":options.chassis_offset,"fixed_chassis":options.fixed_chassis,"spin_deltas":options.spin_deltas,"fast_collisions":options.fast_collisions,
                "target_q_rad":w.task.target_q_rad,
                "requested_pose":w.cad.as_ref().unwrap().requested_pose,
                "area":w.cad.as_ref().unwrap().report()["area"],
                "mode":if options.legacy_guide { "legacy guide with collision guards" } else { "staged collision-aware IK and free-motion joint planner" },
                "planner":w.planning_report})
            )?;
            write_frame(&mut file, &w)?;
            Some(file)
        } else {
            None
        };
        for _ in 0..10000 {
            w.demo();
            if let Some(file) = &mut recording
                && (w.tick.is_multiple_of(4) || w.task.completed() || w.unreachable)
            {
                write_frame(file, &w)?;
            }
            if w.task.completed() || w.unreachable || w.task.failure.is_some() {
                break;
            }
        }
        if let Some(file) = &mut recording {
            std::io::Write::flush(file)?;
        }
        let mut report = serde_json::to_value(&w.task)?;
        report["ik_unreachable"] = (w.unreachable
            && w.planning_report.is_null()
            && !w.cad.as_ref().is_some_and(|c| c.last_collision.is_some()))
        .into();
        report["motion_blocked"] = w.unreachable.into();
        report["diagnostic"] = w.message.clone().into();
        report["planner"] = w.planning_report.clone();
        if let Some(cad) = &w.cad {
            report["cad"] = cad.report();
        }
        println!("{}", serde_json::to_string_pretty(&report)?);
        if !w.task.completed() {
            return Err("assembly demonstration did not complete".into());
        }
        return Ok(());
    }
    if let Some(path) = &mut options.robot_assets {
        *path = path.canonicalize()?;
        robot_assets::verify(path)?;
        let cfg = robot_assets::source_chassis(path)?.ok_or("source chassis required")?;
        if cfg
            .articulation
            .as_ref()
            .is_none_or(|a| a.profile != "pie-tunnel-heavy-v1")
        {
            return Err("workbench currently supports the PIE Engineer chassis only".into());
        }
    }
    let asset_path = options
        .robot_assets
        .as_ref()
        .map(|p| p.display().to_string())
        .unwrap_or_else(|| "assets".into());
    let offscreen = options.screenshot.is_some();
    let mut plugins = DefaultPlugins
        .set(AssetPlugin {
            file_path: asset_path,
            ..default()
        })
        .set(WindowPlugin {
            primary_window: Some(Window {
                title: "Engineer Cartesian assembly workbench".into(),
                resolution: (1280, 800).into(),
                ..default()
            }),
            ..default()
        });
    if offscreen {
        plugins = plugins.disable::<bevy::winit::WinitPlugin>();
    }
    let mut app = App::new();
    if let Some(root) = &options.assembly_assets {
        assembly_cad::register(&mut app, root.clone());
    }
    if let Some(root) = &options.assembly_area {
        use bevy::asset::io::{AssetSourceBuilder, file::FileAssetReader};
        let root = root.clone();
        app.register_asset_source(
            "assembly-area",
            AssetSourceBuilder::new(move || Box::new(FileAssetReader::new(root.clone()))),
        );
    }
    app.init_resource::<CaptureImage>()
        .init_resource::<assembly_cad::BoundGroups>()
        .insert_resource(workbench)
        .insert_resource(options)
        .add_plugins(plugins);
    if offscreen {
        app.add_plugins(bevy::app::ScheduleRunnerPlugin::run_loop(
            std::time::Duration::from_secs_f64(1. / 60.),
        ));
    }
    app.add_plugins(CapturePlugin)
        .insert_resource(bevy::winit::WinitSettings::continuous())
        .insert_resource(Time::<Fixed>::from_hz(128.))
        .insert_resource(ClearColor(Color::srgb(0.035, 0.045, 0.055)))
        .add_systems(Startup, setup)
        .add_systems(FixedUpdate, advance)
        .add_systems(
            Update,
            (
                poll_plan,
                actions,
                hide_source_arm,
                assembly_cad::sync,
                draw,
                capture,
            )
                .chain(),
        )
        .run();
    Ok(())
}
fn write_frame(file: &mut impl std::io::Write, w: &Workbench) -> std::io::Result<()> {
    let rig = w.cad.as_ref().expect("record requires source CAD");
    writeln!(
        file,
        "{}",
        serde_json::json!({"tick":w.tick,
        "step":w.task.cores[0].step,"completed":w.task.completed(),
        "guide_m":w.tool.translation_m,"angles_rad":w.angles,
        "frames_world_flu":rig.world_frames(),"cad":rig.report(),"plan_cursor":w.plan_cursor})
    )
}

fn block(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    position: [f64; 3],
    size: [f32; 3],
    color: Color,
) -> Entity {
    commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(size[0], size[1], size[2]))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                perceptual_roughness: 0.6,
                ..default()
            })),
            Transform::from_translation(flu_position(position)),
        ))
        .id()
}
#[allow(clippy::too_many_arguments)] // Independent Bevy resources, matching the scene setup boundary.
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    assets: Res<AssetServer>,
    options: Res<Options>,
    workbench: Res<Workbench>,
    mut images: ResMut<Assets<Image>>,
    mut capture: ResMut<CaptureImage>,
) {
    let target = if options.screenshot.is_some() {
        let image = images.add(Image::new_target_texture(
            1280,
            800,
            bevy::render::render_resource::TextureFormat::Rgba8UnormSrgb,
            None,
        ));
        capture.0 = Some(image.clone());
        bevy::camera::RenderTarget::Image(image.into())
    } else {
        default()
    };
    let (eye, look) = if options.closeup && workbench.cad.is_some() {
        ([1.5, 0.35, 1.12], [0.77, -0.51, 0.75])
    } else if workbench.cad.is_some() {
        ([2.5, 1.65, 1.65], [0.68, -0.1, 0.5])
    } else {
        ([1.45, 1.65, 1.4], [0.25, 0.1, 0.42])
    };
    commands.spawn((
        target,
        Camera3d::default(),
        bevy::camera::Hdr,
        bevy::camera::Exposure { ev100: 9. },
        IsDefaultUiCamera,
        Transform::from_translation(flu_position(eye)).looking_at(flu_position(look), Vec3::Y),
    ));
    commands.insert_resource(GlobalAmbientLight {
        brightness: 90.,
        color: Color::WHITE,
        ..default()
    });
    commands.spawn((
        DirectionalLight {
            illuminance: 2200.,
            ..default()
        },
        Transform::from_translation(flu_position([3., 2., 2.]))
            .looking_at(flu_position([0.7, -0.5, 0.5]), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(1., 3., 2.).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    if options.assembly_area.is_none() {
        block(
            &mut commands,
            &mut meshes,
            &mut materials,
            [0.7, 0., -0.04],
            [3., 0.08, 3.],
            Color::srgb(0.12, 0.14, 0.16),
        );
    }
    if let Some(rig) = &workbench.cad {
        assembly_cad::spawn(&mut commands, &assets, rig);
    } else {
        // Schematic fallback remains available without source CAD.
        block(
            &mut commands,
            &mut meshes,
            &mut materials,
            [-0.24, 0.2, 0.35],
            [0.22, 0.7, 0.18],
            Color::srgb(0.2, 0.23, 0.27),
        );
        block(
            &mut commands,
            &mut meshes,
            &mut materials,
            [-0.05, 0.2, 0.6],
            [0.02, 0.04, 0.02],
            Color::srgb(0.1, 0.8, 0.8),
        );
        if options.robot_assets.is_some() {
            commands.spawn((
                WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset("neutral.glb"))),
                Transform::from_translation(flu_position([0.55, 0., 0.132]))
                    .with_rotation(Quat::from_rotation_y(std::f32::consts::PI)),
            ));
            // Package armor was removed for runtime replacement; add simple workbench housings.
            for p in [
                [0.88, 0., 0.16],
                [0.22, 0., 0.16],
                [0.55, 0.29, 0.16],
                [0.55, -0.29, 0.16],
            ] {
                block(
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    p,
                    [0.08, 0.08, 0.08],
                    Color::srgb(0.45, 0.05, 0.04),
                );
            }
        } else {
            block(
                &mut commands,
                &mut meshes,
                &mut materials,
                [0.55, 0., 0.14],
                [0.5, 0.22, 0.55],
                Color::srgb(0.16, 0.18, 0.2),
            );
        }
        block(
            &mut commands,
            &mut meshes,
            &mut materials,
            [0.55, 0., 0.275],
            [0.08, 0.15, 0.08],
            Color::srgb(0.25, 0.28, 0.3),
        );
        for i in 0..2 {
            let e = block(
                &mut commands,
                &mut meshes,
                &mut materials,
                [0.; 3],
                [0.035, 1., 0.035],
                Color::srgb(0.55, 0.59, 0.63),
            );
            commands.entity(e).insert(Segment(i));
        }
        let tool = block(
            &mut commands,
            &mut meshes,
            &mut materials,
            [0.; 3],
            [0.065, 0.065, 0.065],
            Color::srgb(0.9, 0.57, 0.08),
        );
        commands.entity(tool).insert(Tool);
    }
    commands.spawn((
        Text::new(""),
        TextFont {
            font_size: bevy::text::FontSize::Px(19.),
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            top: px(16.),
            left: px(16.),
            ..default()
        },
        Status,
    ));
}
fn hide_source_arm(mut q: Query<(&Name, &mut Visibility), Added<Name>>) {
    for (name, mut visibility) in &mut q {
        if [
            "link/shoulder",
            "link/elbow",
            "link/camera-lift",
            "link/uwb",
        ]
        .contains(&name.as_str())
        {
            *visibility = Visibility::Hidden;
        }
    }
}
fn actions(
    options: Res<Options>,
    keys: Res<ButtonInput<KeyCode>>,
    mut w: ResMut<Workbench>,
    mut exit: MessageWriter<AppExit>,
) {
    for (key, d) in [
        (KeyCode::Digit1, 1),
        (KeyCode::Digit2, 2),
        (KeyCode::Digit3, 3),
    ] {
        if keys.just_pressed(key) {
            if options.pose_seed.is_some() && d != w.task.difficulty {
                w.message =
                    "Seeded poses depend on difficulty; launch with the requested --difficulty"
                        .into();
                continue;
            }
            let target_q = w.task.target_q_rad;
            let mut cad = w.cad.take();
            if let Some(rig) = &mut cad {
                rig.restart();
            }
            *w = Workbench {
                task: Attempt::new(d, target_q, 0).unwrap(),
                cad,
                ..Workbench::new(d)
            };
        }
    }
    if keys.just_pressed(KeyCode::Space) {
        w.paused = !w.paused;
    }
    if keys.just_pressed(KeyCode::KeyG) {
        w.cancel_plan();
        w.attached = !w.attached;
    }
    if keys.just_pressed(KeyCode::KeyO) || keys.just_pressed(KeyCode::KeyP) {
        let automatic = keys.just_pressed(KeyCode::KeyO);
        if w.plan.is_some() {
            w.automatic = automatic;
            w.stage_run = true;
        } else {
            w.request_plan(automatic, options.plan_budget as usize);
        }
    }
    if [
        KeyCode::KeyW,
        KeyCode::KeyS,
        KeyCode::KeyA,
        KeyCode::KeyD,
        KeyCode::KeyR,
        KeyCode::KeyF,
        KeyCode::KeyZ,
        KeyCode::KeyX,
        KeyCode::ArrowUp,
        KeyCode::ArrowDown,
        KeyCode::ArrowLeft,
        KeyCode::ArrowRight,
    ]
    .into_iter()
    .any(|k| keys.just_pressed(k))
    {
        w.cancel_plan();
    }
    if keys.just_pressed(KeyCode::KeyE) {
        w.confirm_requested = true;
    }
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}
fn poll_plan(mut w: ResMut<Workbench>) {
    let result = w
        .pending_plan
        .as_ref()
        .and_then(|pending| pending.lock().unwrap().take());
    if let Some(result) = result {
        let automatic = w.automatic;
        w.pending_plan = None;
        w.apply_plan(result, automatic);
    }
}
fn advance(keys: Res<ButtonInput<KeyCode>>, options: Res<Options>, mut w: ResMut<Workbench>) {
    if w.pending_plan.is_some() {
        return;
    }
    if w.plan.is_some() {
        if w.automatic || w.stage_run {
            w.planned_step();
        }
        if !w.paused && std::mem::take(&mut w.confirm_requested) {
            w.confirm();
        }
        return;
    }
    if options.demo && w.planning_report["status"] == "failed" {
        return;
    }
    if options.demo && (w.cad.is_none() || w.planning_report.is_null()) {
        if !w.unreachable {
            w.demo();
        }
        return;
    }
    let axis = |positive, negative| {
        f64::from(u8::from(keys.pressed(positive))) - f64::from(u8::from(keys.pressed(negative)))
    };
    w.step(
        [
            axis(KeyCode::KeyS, KeyCode::KeyW),
            axis(KeyCode::KeyA, KeyCode::KeyD),
            axis(KeyCode::KeyR, KeyCode::KeyF),
        ],
        [
            axis(KeyCode::ArrowUp, KeyCode::ArrowDown),
            axis(KeyCode::ArrowLeft, KeyCode::ArrowRight),
            axis(KeyCode::KeyZ, KeyCode::KeyX),
        ],
    );
    if !w.paused && std::mem::take(&mut w.confirm_requested) {
        w.confirm();
    }
}
fn draw(
    w: Res<Workbench>,
    mut segments: Query<(&Segment, &mut Transform), Without<Tool>>,
    mut tool: Query<&mut Transform, (With<Tool>, Without<Segment>)>,
    mut text: Query<&mut Text, With<Status>>,
    mut gizmos: Gizmos,
) {
    if w.cad.is_none() {
        let arm = ARM.solve(w.tool).unwrap();
        let points = [arm.shoulder_m, arm.elbow_m, arm.tool.translation_m];
        for (segment, mut t) in &mut segments {
            let i = usize::from(segment.0);
            let a = flu_position(points[i]);
            let b = flu_position(points[i + 1]);
            t.translation = (a + b) * 0.5;
            t.rotation = Quat::from_rotation_arc(Vec3::Y, (b - a).normalize());
            t.scale.y = (b - a).length();
        }
        for mut t in &mut tool {
            apply_pose(
                &mut t,
                PoseFlu {
                    translation_m: w.tool.translation_m,
                    rotation_wxyz: w.tool.rotation_wxyz,
                },
            );
        }
    }
    let color = if w.task.completed() {
        Color::srgb(0.1, 1., 0.3)
    } else {
        Color::srgb(0.1, 0.8, 0.9)
    };
    if let Some(cad) = &w.cad {
        let [qw, qx, qy, qz] = cad.socket.rotation_wxyz;
        let q = DQuat::from_xyzw(qx, qy, qz, qw);
        let p = bevy::math::DVec3::from_array(cad.socket.translation_m);
        gizmos.line(
            flu_position(p.to_array()),
            flu_position((p + q * bevy::math::DVec3::X * 0.1).to_array()),
            color,
        );
        gizmos.sphere(
            Isometry3d::from_translation(flu_position(p.to_array())),
            0.012,
            color,
        );
    } else {
        gizmos.line(
            flu_position([ENTRANCE_X, 0.2, 0.6]),
            flu_position(SOCKET),
            color,
        );
        gizmos.line(flu_position(SOCKET), flu_position([-0.05, 0.2, 0.7]), color);
        gizmos.sphere(
            Isometry3d::from_translation(flu_position(SOCKET)),
            0.012,
            color,
        );
    }
    let [x, y, z] = w.tool.translation_m;
    for mut t in &mut text {
        t.0 = format!(
            "ASSEMBLY WORKBENCH | difficulty {} | {:?} | {:.2}s{}\n\
            WASD: horizontal  R/F: height  Arrows: P/Q  Z/X: off-axis roll\n\
            O: automatic  P: next stage  E: confirm  G: grip  Space: pause  1/2/3: restart\n\
            {} [{x:.3}, {y:.3}, {z:.3}] m  P {:.1}°  Q {:.1}° (target {:.1}°)\n\
            {} {} {}\n\
            {}",
            w.task.difficulty,
            w.task.cores[0].step,
            w.tick as f64 / 128.,
            if w.paused { " PAUSED" } else { "" },
            if w.cad.is_some() { "Guide" } else { "Tool" },
            w.angles[0].to_degrees(),
            w.angles[1].to_degrees(),
            w.task.target_q_rad[0].to_degrees(),
            if w.task.completed() {
                "COMPLETE"
            } else if w.pending_plan.is_some() {
                "PLANNING"
            } else if w.planning_report["status"] == "failed" {
                "PLANNING FAILED"
            } else if w.cad.as_ref().is_some_and(|c| c.last_collision.is_some()) {
                "COLLISION BLOCKED"
            } else if w.unreachable {
                "UNREACHABLE TARGET"
            } else {
                ""
            },
            w.cad
                .as_ref()
                .filter(|_| w.pending_plan.is_none() && w.planning_report["status"] != "failed")
                .and_then(|c| c.last_collision.as_ref())
                .map_or_else(
                    || w.message.clone(),
                    |h| format!(
                        "{} / {}{}",
                        h.first,
                        h.second,
                        if h.conservative {
                            " (sweep budget)"
                        } else {
                            ""
                        }
                    )
                ),
            w.task
                .failure
                .map_or_else(String::new, |f| format!("Failed: {f:?}")),
            if w.cad.is_some() {
                if w.cad.as_ref().is_some_and(|c| c.gripped()) {
                    "Source CAD | GRIPPED | constrained assembly guide | no match rewards"
                } else if w.attached {
                    "Source CAD | CLOSING / NO GRASP | no match rewards"
                } else {
                    "Source CAD | OPEN / RELEASED | no match rewards"
                }
            } else {
                "Generic position-controlled arm + schematic socket; no match rewards"
            }
        );
    }
}
fn capture(
    image: Res<CaptureImage>,
    groups: Res<assembly_cad::BoundGroups>,
    options: Res<Options>,
    ready: Res<CaptureReady>,
    mut w: ResMut<Workbench>,
    mut commands: Commands,
) {
    if !ready.0.load(Ordering::Acquire) {
        w.capture_frames = 0;
        return;
    }
    if w.cad.is_some() && !groups.ready() {
        w.capture_frames = 0;
        return;
    }
    if let Some(path) = &options.screenshot
        && (w.task.completed() || w.unreachable)
    {
        w.capture_frames += 1;
        if w.capture_frames == 60 {
            commands
                .spawn(
                    image
                        .0
                        .clone()
                        .map_or_else(Screenshot::primary_window, Screenshot::image),
                )
                .observe(save_to_disk(path.clone()))
                .observe(
                    |_: On<bevy::render::view::screenshot::ScreenshotCaptured>,
                     mut exit: MessageWriter<AppExit>| {
                        exit.write(AppExit::Success);
                    },
                );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn assisted_playback_stops_at_boundaries_and_obeys_pause() {
        let (rig, plan) = assembly_cad::Rig::test_playback_fixture();
        let mut w = crate::Workbench::new(1);
        w.plan = Some(plan);
        w.cad = Some(rig);
        w.stage_run = true;
        w.paused = true;
        w.planned_step();
        assert_eq!(w.tick, 0);
        w.paused = false;
        w.planned_step();
        assert!(w.stage_run);
        w.planned_step();
        assert!(!w.stage_run);
        assert_eq!(w.plan_cursor, 2);
        w.planned_step();
        assert_eq!(w.plan_cursor, 2);
        w.stage_run = true;
        w.planned_step();
        assert_eq!(w.plan_cursor, 3);
        assert!(!w.stage_run);
        // Exhausting an arbitrary path never bypasses the manipulation referee.
        w.automatic = true;
        w.planned_step();
        assert!(!w.task.completed());
    }
    #[test]
    fn cancelled_worker_result_is_never_installed() {
        let mut w = crate::Workbench::new(1);
        let pending = std::sync::Arc::new(std::sync::Mutex::new(None));
        w.pending_plan = Some(pending.clone());
        w.unreachable = true;
        w.planning_report = serde_json::json!({"status":"failed"});
        w.cancel_plan();
        let (rig, _) = assembly_cad::Rig::test_playback_fixture();
        *pending.lock().unwrap() =
            Some(rig.plan_assembly(1, 0., [0.09, 0.2, 0.6], [0.; 3], false, 0));
        let mut app = App::new();
        app.insert_resource(w).add_systems(Update, crate::poll_plan);
        app.update();
        let w = app.world().resource::<crate::Workbench>();
        assert_eq!(w.planning_report["status"], "cancelled");
        assert!(!w.unreachable);
        assert!(w.plan.is_none());
    }
    #[test]
    fn rate_limit_pause_and_unreachable_target() {
        let mut w = Workbench::new(1);
        let x = w.tool.translation_m[0];
        w.step([1., 0., 0.], [0.; 3]);
        assert!((w.tool.translation_m[0] - x - SPEED / 128.).abs() < 1e-12);
        w.paused = true;
        let tick = w.tick;
        let tool = w.tool;
        w.step([1.; 3], [1.; 3]);
        assert_eq!(w.tick, tick);
        assert_eq!(w.tool, tool);
        w.paused = false;
        // Move away from the fixture until the arm's reach, without entering it.
        for _ in 0..3000 {
            w.step([0., 1., 0.], [0.; 3]);
        }
        assert!(w.unreachable);
        assert!(ARM.solve(w.tool).is_ok());
    }
    #[test]
    fn explicit_pose_and_parking_arguments_are_bounded_and_unambiguous() {
        let args = Options::try_parse_from([
            "workbench",
            "--assembly-assets",
            "package",
            "--assembly-area",
            "area",
            "--core-pose",
            "-0.05",
            "0.2",
            "0.5",
            "10",
            "80",
            "20",
            "--chassis-offset",
            "-0.1",
            "-0.15",
            "15",
            "--fixed-chassis",
            "--q-deg",
            "-30",
        ])
        .unwrap();
        assert!(args.fixed_chassis);
        assert_eq!(args.q_deg, Some(-30.));
        let p = explicit_core_pose(args.core_pose.as_ref().unwrap(), 3).unwrap();
        assert!((p.theta_rad - 10f64.to_radians()).abs() < 1e-12);
        assert!(explicit_core_pose(&[-0.05, 0.2, 0.5, 10., 80., 20.], 1).is_err());
        assert!(explicit_core_pose(&[-0.05, 0.2, 0.8, 0., 90., 0.], 3).is_err());
        assert!(explicit_core_pose(&[f64::NAN; 6], 3).is_err());
        assert!(Options::try_parse_from(["workbench", "--fixed-chassis"]).is_err());
        assert!(validate_spin_stages(1, 0., &[0., 0., 1., 0., 0.]).is_err());
        assert!(validate_spin_stages(3, 0., &[0., 0., 0., 90., 20.]).is_err());
        assert!(validate_spin_stages(3, -0.5, &[0., 0., 0., 90., 20.]).is_ok());
    }
    #[test]
    fn cartesian_demo_completes_each_interactive_difficulty() {
        for d in 1..=3 {
            let mut w = Workbench::new(d);
            for _ in 0..1280 {
                let prior_p = w.angles[0];
                let prior_q = w.angles[1];
                let prior_z = w.tool.translation_m[2];
                w.demo();
                if w.angles[0] != prior_p {
                    assert!((prior_z - SOCKET[2] - 0.1).abs() < 1e-9);
                }
                if w.angles[1] != prior_q {
                    assert!((prior_p - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
                }
                if w.task.completed() {
                    break;
                }
            }
            assert!(w.task.completed(), "difficulty {d}: {:?}", w.task);
            assert_eq!(w.task.failure, None);
            assert!((w.tool.translation_m[0] - SOCKET[0]).abs() < 1e-9);
            if d >= 2 {
                assert!((w.tool.translation_m[2] - SOCKET[2] - 0.1).abs() < 1e-9);
            }
            if d >= 3 {
                assert!((w.angles[0] - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
            }
            assert!(ARM.solve(w.tool).is_ok());
        }
    }
}
