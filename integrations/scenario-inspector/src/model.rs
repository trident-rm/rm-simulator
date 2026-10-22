// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Toolkit-independent selection and explicit physical preview. No rules or wall-clock reads.
use anyhow::{Context, ensure};
use rm_simulator_assets::{
    cad_assets::{self, CadAsset, CadAssets},
    components::{self, ComponentRole, PlacedComponent},
    math,
    scenarios::{Preset, RobotSpawn, Scenario},
};
use rm_simulator_physics::{
    Caliber, Pose, Shot, StepDuration, TargetFace, Team,
    arena::OutpostModel,
    chassis::{ChassisCommand, ChassisConfig, ChassisSnapshot},
    control::WheelTorques,
    motion::{Mechanism, MechanismState, MotionFrame},
    projectile::ProjectileSnapshot,
    session::{RobotCommand, RobotSession},
};
use serde_json::{Value, json};
use std::path::Path;

/// Fixed inspection/physical preview step in nanoseconds (128 Hz).
pub const TICK_NS: u64 = 7_812_500;

/// Explicit inspection pacing, with no wall-clock reads or match authority.
/// Use Inspector's step methods when it owns a physical preset session.
///
/// ```
/// use rm_scenario_inspector::model::{Timeline, TICK_NS};
/// let mut clock = Timeline::default();
/// clock.step(2)?;
/// assert_eq!(clock.time_ns, 2 * TICK_NS);
/// # Ok::<(), anyhow::Error>(())
/// ```
#[derive(Default)]
pub struct Timeline {
    /// Elapsed explicit time in nanoseconds.
    pub time_ns: u64,
    /// Whether asynchronous pulses should request explicit steps.
    pub playing: bool,
}
impl Timeline {
    /// Add ticks, rejecting arithmetic overflow before changing the clock.
    pub fn step(&mut self, ticks: u64) -> anyhow::Result<()> {
        self.time_ns = self
            .time_ns
            .checked_add(
                ticks
                    .checked_mul(TICK_NS)
                    .context("inspection tick overflow")?,
            )
            .context("inspection time overflow")?;
        Ok(())
    }
    /// Advance sixteen ticks only while playback is enabled.
    pub fn pulse(&mut self) -> anyhow::Result<()> {
        if self.playing {
            self.step(16)?;
        }
        Ok(())
    }
    /// Clear elapsed time and pause.
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// One bounded diagnostic contact, from physics rather than a sensor or referee.
#[derive(Clone, Debug, PartialEq)]
pub struct PhysicalContact {
    /// End of the integration tick in nanoseconds.
    pub time_ns: u64,
    /// Physical event family: projectile armor contact or chassis armor collision.
    pub kind: &'static str,
    /// Physics target identity, with no detection/scoring decision applied.
    pub target: String,
    /// Physical projectile identity, absent for chassis armor collisions.
    pub projectile: Option<u64>,
    /// Contact location in world FLU metres.
    pub position_m: [f64; 3],
    /// Contact location in the target frame in metres, when supplied by physics.
    pub local_m: Option<[f64; 3]>,
    /// Closing speed along the armor normal, metres per second.
    pub normal_speed_m_s: f64,
}
impl PhysicalContact {
    fn report(&self) -> Value {
        json!({"time_ns": self.time_ns, "kind": self.kind, "target": self.target,
            "projectile": self.projectile, "position_m": self.position_m,
            "local_m": self.local_m, "normal_speed_m_s": self.normal_speed_m_s})
    }
}

struct PhysicalPreview {
    scenario: Scenario,
    session: RobotSession,
    targets: Vec<TargetFace>,
    spawn: Option<RobotSpawn>,
    contacts: Vec<PhysicalContact>,
    contact_count: u64,
}

impl PhysicalPreview {
    fn advance(&mut self, end: u64, state: &MechanismState) -> anyhow::Result<()> {
        self.scenario.targets_at(state, &mut self.targets)?;
        self.session.physics_mut().sync_mechanisms(state);
        let contacts = self
            .session
            .step_with_targets(&self.targets)
            .map_err(anyhow::Error::msg)?;
        for contact in contacts {
            self.contacts.push(PhysicalContact {
                time_ns: end,
                kind: "projectile-armor",
                target: format!("{:?}", contact.target),
                projectile: Some(contact.projectile),
                position_m: contact.position_m,
                local_m: Some(contact.local_m),
                normal_speed_m_s: contact.normal_speed_m_s,
            });
            self.contact_count += 1;
        }
        for contact in self.session.physics_mut().take_armor_collisions() {
            self.contacts.push(PhysicalContact {
                time_ns: end,
                kind: "chassis-armor",
                target: format!(
                    "chassis {} plate {} other {:?}",
                    contact.chassis, contact.plate, contact.other_chassis
                ),
                projectile: None,
                position_m: contact.position_m,
                local_m: None,
                normal_speed_m_s: contact.normal_speed_m_s,
            });
            self.contact_count += 1;
        }
        // Event queues and retained diagnostics stay bounded during long previews.
        self.session.physics_mut().take_removals();
        if self.contacts.len() > 128 {
            self.contacts.drain(..self.contacts.len() - 128);
        }
        Ok(())
    }
}

/// Caller-owned inspection state; mutate time/coordinates through its methods
/// to keep the optional physical session synchronized with displayed geometry.
pub struct Inspector {
    /// Checksum-verified source package and original metadata.
    pub cad: CadAssets,
    /// Stable UI selection names for this package.
    pub names: Vec<String>,
    /// Current index in names, valid after successful load/selection.
    pub selected: usize,
    /// Original exported placement index, or zero for the arena root.
    pub placement: usize,
    /// Fixed/moving world FLU triangles, retained without simplification.
    pub placed: PlacedComponent,
    /// Explicit pacing state; use step/reset to change its time.
    pub timeline: Timeline,
    /// Per-channel offsets at inspection t=0; rotations rad, translations fraction.
    pub coordinates: Vec<f64>,
    /// Optional caller-exported RFID geometry; never treated as package-verified.
    pub overlay_zones: Vec<rm_simulator_assets::rmuc_2027::Zone>,
    /// Caller-supplied overlay path, explicitly outside package verification.
    pub overlay_source: Option<std::path::PathBuf>,
    physical: Option<PhysicalPreview>,
}
impl Inspector {
    /// Verify a package and construct one source selection, paused at time zero.
    pub fn load(root: &Path, name: &str, placement: usize) -> anyhow::Result<Self> {
        let cad = cad_assets::load(root)?;
        let mut names: Vec<String> = ["outpost", "rune", "base", "tech-core", "floor", "arena"]
            .into_iter()
            .map(String::from)
            .collect();
        names.extend((0..cad.static_assets.len()).map(|i| format!("scenery-{i}")));
        let selected = names
            .iter()
            .position(|s| s == name)
            .context("unknown asset name")?;
        let placed = Self::compose(&cad, name, placement)?;
        let coordinates = vec![0.; placed.moving.len()];
        let physical = Self::physical(&cad, name, placement)?;
        Ok(Self {
            cad,
            names,
            selected,
            placement,
            placed,
            timeline: Timeline::default(),
            coordinates,
            overlay_zones: vec![],
            overlay_source: None,
            physical,
        })
    }
    fn physical(
        cad: &CadAssets,
        name: &str,
        placement: usize,
    ) -> anyhow::Result<Option<PhysicalPreview>> {
        let asset = Self::lookup(cad, name);
        let Some(root) = asset.placements.get(placement) else {
            return Ok(None);
        };
        let side = if root.translation_m[0] >= 0. {
            Team::Red
        } else {
            Team::Blue
        };
        let preset = match name {
            "outpost" => Preset::Outpost(side),
            "base" => Preset::Base(side),
            "rune" => Preset::Rune,
            _ => return Ok(None),
        };
        if cad.rmuc_2027.is_none() {
            return Ok(None);
        }
        let scenario = Scenario::load(cad, preset)?;
        // Never silently substitute another placement of the same side.
        if scenario.visual().is_none_or(|v| v.placement != *root) {
            return Ok(None);
        }
        // Spawn outside the selected source envelope on the preset's flat catch floor.
        // This is the existing physical omni profile, not source CAD or a match robot.
        let placed = Self::compose(cad, name, placement)?;
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for point in placed
            .fixed
            .vertices_m
            .iter()
            .chain(placed.moving.iter().flat_map(|p| &p.mesh.vertices_m))
        {
            min_x = min_x.min(point[0]);
            min_y = min_y.min(point[1]);
            max_y = max_y.max(point[1]);
        }
        let config = ChassisConfig::default();
        let spawn = RobotSpawn {
            team: Team::Red,
            pose: Pose::at([
                if min_x.is_finite() { min_x - 2. } else { 0. },
                if min_y.is_finite() {
                    (min_y + max_y) / 2.
                } else {
                    0.
                },
                config.rest_height_m(),
            ]),
            config,
        };
        let session = scenario.session(
            StepDuration::HZ_128,
            vec![spawn.clone()],
            MechanismState::default(),
        )?;
        Ok(Some(PhysicalPreview {
            scenario,
            session,
            targets: vec![],
            spawn: Some(spawn),
            contacts: vec![],
            contact_count: 0,
        }))
    }
    /// Resolve physical mechanism channels at the current explicit time.
    pub fn mechanism_state(&self) -> MechanismState {
        self.state_at(self.timeline.time_ns)
    }
    fn state_at(&self, time_ns: u64) -> MechanismState {
        let mut state = MechanismState::default();
        for (i, part) in self.placed.moving.iter().enumerate() {
            let value = self.coordinate_at(i, time_ns);
            match part.mechanism {
                Mechanism::Base => state.base_open_fraction[part.team.index()] = value,
                Mechanism::Outpost => state.outpost_angle_rad[part.team.index()] = value,
                Mechanism::Rune => state.rune_angle_rad[part.team.index()] = value,
                Mechanism::DartDoor => state.dart_door_open[part.team.index()] = value >= 0.5,
                Mechanism::DartTarget => state.dart_target_fraction = value,
            }
        }
        state
    }
    /// Advance explicit ticks through the ordinary physical robot/target session.
    /// Add ticks, rejecting arithmetic overflow before changing the clock.
    pub fn step(&mut self, ticks: u64) -> anyhow::Result<()> {
        let mut checked = Timeline {
            time_ns: self.timeline.time_ns,
            playing: false,
        };
        checked.step(ticks)?;
        if self.physical.is_none() {
            self.timeline.time_ns = checked.time_ns;
            return Ok(());
        }
        for _ in 0..ticks {
            let end = self.timeline.time_ns + TICK_NS;
            let state = self.state_at(end);
            let preview = self.physical.as_mut().unwrap();
            preview.advance(end, &state)?;
            self.timeline.time_ns = end;
            debug_assert_eq!(preview.session.stamp().time_ns, end);
        }
        Ok(())
    }
    /// Advance sixteen ticks only while playback is enabled.
    pub fn pulse(&mut self) -> anyhow::Result<()> {
        if self.timeline.playing {
            self.step(16)?;
        }
        Ok(())
    }
    /// Restart shared geometry and the configured initial physical robot.
    fn restart_session(&mut self) -> anyhow::Result<()> {
        let state = self.mechanism_state();
        if let Some(preview) = &mut self.physical {
            preview.session = preview.scenario.session(
                StepDuration::HZ_128,
                preview.spawn.clone().into_iter().collect(),
                state,
            )?;
            preview.contacts.clear();
            preview.contact_count = 0;
        }
        Ok(())
    }
    /// Initial robot recipe retained by reset; absent for geometry-only selections.
    pub fn robot_spawn(&self) -> Option<&RobotSpawn> {
        self.physical.as_ref().and_then(|p| p.spawn.as_ref())
    }
    /// Rebuild at time zero with an explicit physical robot recipe (or no robot).
    /// Invalid configurations leave the current session and timeline intact.
    pub fn configure_robot(&mut self, spawn: Option<RobotSpawn>) -> anyhow::Result<()> {
        let state = self.state_at(0);
        let preview = self
            .physical
            .as_mut()
            .context("selection has no physical scenario")?;
        let session = preview.scenario.session(
            StepDuration::HZ_128,
            spawn.clone().into_iter().collect(),
            state,
        )?;
        preview.session = session;
        preview.spawn = spawn;
        preview.contacts.clear();
        preview.contact_count = 0;
        self.timeline.reset();
        Ok(())
    }
    /// Current state at the inspection time: privileged truth, not a simulated sensor.
    pub fn robot_truth(&self) -> Option<ChassisSnapshot> {
        self.physical
            .as_ref()?
            .session
            .robot_truth(0)
            .map(|t| t.state)
    }
    /// Current projectile truth in world FLU metres; no sensing model is applied.
    pub fn projectiles(&self) -> Vec<ProjectileSnapshot> {
        self.physical
            .as_ref()
            .map(|p| p.session.ground_truth().snapshot())
            .unwrap_or_default()
    }
    /// Most recent 128 raw projectile/armor or chassis/armor events, in tick order.
    pub fn contacts(&self) -> &[PhysicalContact] {
        self.physical
            .as_ref()
            .map(|p| p.contacts.as_slice())
            .unwrap_or_default()
    }
    /// Hold body velocity (m/s, rad/s) and world gimbal aim (rad) using real motors.
    /// A drive request selects velocity control, replacing a held torque request.
    pub fn drive(&mut self, command: ChassisCommand) -> anyhow::Result<()> {
        self.robot_command(RobotCommand::Drive { robot: 0, command })
            .map(|_| ())
    }
    /// Set world yaw/elevation in radians while preserving the current drive mode.
    pub fn aim(&mut self, yaw_rad: f64, pitch_rad: f64) -> anyhow::Result<()> {
        let state = self.robot_truth().context("no robot spawned")?;
        self.drive(ChassisCommand {
            aim_yaw_rad: yaw_rad,
            aim_pitch_rad: pitch_rad,
            ..state.command
        })?;
        if let rm_simulator_physics::control::DriveControl::WheelTorques(torques) =
            state.drive_control
        {
            self.robot_command(RobotCommand::WheelTorques { robot: 0, torques })?;
        }
        Ok(())
    }
    /// Hold wheel torques in newton metres in physical wheel roster order.
    pub fn wheel_torques(&mut self, torques_nm: Vec<f64>) -> anyhow::Result<()> {
        let torques = WheelTorques::new(torques_nm).map_err(anyhow::Error::msg)?;
        self.robot_command(RobotCommand::WheelTorques { robot: 0, torques })
            .map(|_| ())
    }
    /// Launch once from the actual muzzle; speed is metres per second in (0,40].
    /// Physical capacity/defeat limits apply; no cadence, heat or HP rules exist.
    pub fn fire(&mut self, caliber: Caliber, speed_m_s: f64) -> anyhow::Result<()> {
        self.robot_command(RobotCommand::Fire {
            robot: 0,
            shot: Shot { caliber, speed_m_s },
        })
        .map(|_| ())
    }
    fn robot_command(
        &mut self,
        command: RobotCommand,
    ) -> anyhow::Result<rm_simulator_physics::session::CommandResult> {
        self.physical
            .as_mut()
            .context("selection has no physical scenario")?
            .session
            .command(command)
            .map_err(anyhow::Error::msg)
    }
    /// Execute an ordered headless JSON action array through the same UI methods.
    /// Actions use explicit ticks; drive persists until replaced. No wall time is read.
    pub fn run_script(&mut self, script: &Value) -> anyhow::Result<()> {
        validate_script(script)?;
        for action in script.as_array().unwrap() {
            let (key, value) = action.as_object().unwrap().iter().next().unwrap();
            let numbers = || {
                value
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|v| v.as_f64().unwrap())
                    .collect::<Vec<_>>()
            };
            match key.as_str() {
                "step" => self.step(value.as_u64().unwrap())?,
                "reset" => self.reset()?,
                "drive" => {
                    let v = numbers();
                    let command = self.robot_truth().context("no robot spawned")?.command;
                    self.drive(ChassisCommand {
                        forward_m_s: v[0],
                        left_m_s: v[1],
                        yaw_rate_rad_s: v[2],
                        ..command
                    })?;
                }
                "aim_rad" => {
                    let v = numbers();
                    self.aim(v[0], v[1])?;
                }
                "torques_nm" => self.wheel_torques(numbers())?,
                "fire" => {
                    let v = numbers();
                    self.fire(
                        if v[0] == 17. {
                            Caliber::Mm17
                        } else {
                            Caliber::Mm42
                        },
                        v[1],
                    )?;
                }
                _ => unreachable!(),
            }
        }
        Ok(())
    }
    /// Original source asset descriptor of the current selection.
    pub fn asset(&self) -> &CadAsset {
        Self::lookup(&self.cad, &self.names[self.selected])
    }
    fn lookup<'a>(cad: &'a CadAssets, name: &str) -> &'a CadAsset {
        match name {
            "outpost" => &cad.outpost,
            "rune" => &cad.rune,
            "base" => &cad.base,
            "tech-core" => &cad.tech_core,
            "floor" => &cad.floor,
            "arena" => &cad.arena_static,
            _ => {
                &cad.static_assets[name
                    .strip_prefix("scenery-")
                    .unwrap()
                    .parse::<usize>()
                    .unwrap()]
            }
        }
    }
    fn compose(cad: &CadAssets, name: &str, placement: usize) -> anyhow::Result<PlacedComponent> {
        let asset = Self::lookup(cad, name);
        let pose = if asset.placements.is_empty() {
            ensure!(placement == 0, "unplaced asset has only arena root index 0");
            cad.arena_pose()
        } else {
            *asset
                .placements
                .get(placement)
                .context("placement index out of range")?
        };
        let role = match name {
            "outpost" => ComponentRole::Outpost,
            "rune" => ComponentRole::Rune,
            _ => ComponentRole::Equipment,
        };
        let motions = cad.rmuc_2027.as_ref().map(|b| b.motions(cad, asset));
        Ok(components::load(cad, asset, role, motions)?.placed(&pose))
    }
    /// Load another selection; failure leaves the current model intact.
    pub fn select(&mut self, selected: usize, placement: usize) -> anyhow::Result<()> {
        let name = self
            .names
            .get(selected)
            .context("asset index out of range")?;
        let placed = Self::compose(&self.cad, name, placement)?;
        let physical = Self::physical(&self.cad, name, placement)?;
        self.physical = physical;
        self.coordinates = vec![0.; placed.moving.len()];
        self.placed = placed;
        self.selected = selected;
        self.placement = placement;
        self.timeline.reset();
        Ok(())
    }
    /// Pause and restore time, offsets and configured robot; clear shots and contacts.
    pub fn reset(&mut self) -> anyhow::Result<()> {
        self.timeline.reset();
        self.coordinates.fill(0.);
        self.restart_session()
    }
    /// Restart at zero time with one shared channel offset: rad or fraction.
    pub fn set_coordinate(&mut self, channel: usize, value: f64) -> anyhow::Result<()> {
        ensure!(value.is_finite(), "coordinate must be finite");
        let part = self
            .placed
            .moving
            .get(channel)
            .context("missing mechanism channel")?;
        let value = match part.motion {
            MotionFrame::Rotation { .. } => value,
            MotionFrame::Translation { .. } => value.clamp(0., 1.),
        };
        let key = (part.mechanism, part.team);
        // Multiple source bodies on one physical channel share its coordinate.
        for (i, p) in self.placed.moving.iter().enumerate() {
            if (p.mechanism, p.team) == key {
                self.coordinates[i] = value;
            }
        }
        self.timeline.reset();
        self.restart_session()
    }
    /// Current resolved channel value: radians for rotation, otherwise fraction.
    pub fn coordinate(&self, channel: usize) -> f64 {
        self.coordinate_at(channel, self.timeline.time_ns)
    }
    fn coordinate_at(&self, channel: usize, time_ns: u64) -> f64 {
        // A deliberately illustrative preview law, not a competition rule speed.
        let t = time_ns as f64 / 1e9;
        match self.placed.moving[channel].motion {
            MotionFrame::Rotation { .. } => self.coordinates[channel] + t * 0.5,
            MotionFrame::Translation { .. } => (self.coordinates[channel] + t * 0.1).clamp(0., 1.),
        }
    }
    /// World FLU point after the shared mechanism transform, in metres.
    pub fn moved_point(&self, channel: usize, point: [f64; 3]) -> [f64; 3] {
        let pose = self.placed.moving[channel]
            .motion
            .pose(self.coordinate(channel));
        math::add(pose.translation_m, math::rotate(pose.rotation_wxyz, point))
    }
    /// Current source detector frames in world FLU, with +x outward.
    pub fn detectors(&self) -> Vec<Pose> {
        if let Some(preview) = &self.physical {
            let mut targets = Vec::new();
            preview
                .scenario
                .targets_at(&self.mechanism_state(), &mut targets)
                .expect("validated inspection coordinates");
            return targets.into_iter().map(|t| t.pose).collect();
        }
        let Some(b) = &self.cad.rmuc_2027 else {
            return vec![];
        };
        let team = if self.root().translation_m[0] >= 0. {
            Team::Red
        } else {
            Team::Blue
        };
        match self.names[self.selected].as_str() {
            "outpost" => {
                let source = &b.world.outposts[team.index()];
                let angle = self
                    .placed
                    .moving
                    .iter()
                    .position(|p| p.team == team)
                    .map(|i| self.coordinate(i))
                    .unwrap_or(0.);
                (0..source.detectors.len())
                    .filter_map(|i| source.detector_pose(i, angle))
                    .collect()
            }
            "base" => b.world.bases[team.index()].iter().map(|d| d.pose).collect(),
            "rune" => b
                .world
                .runes
                .iter()
                .enumerate()
                .flat_map(|(i, r)| {
                    let angle = self
                        .placed
                        .moving
                        .iter()
                        .position(|p| p.team.index() == i)
                        .map(|c| self.coordinate(c))
                        .unwrap_or(0.);
                    r.targets
                        .map(|p| OutpostModel::rotate_pose(p, r.pivot_m, r.axis, angle))
                })
                .collect(),
            _ => vec![],
        }
    }
    /// Import explicit caller geometry without enabling any RFID rules.
    pub fn load_zones(&mut self, path: &Path) -> anyhow::Result<()> {
        let zones: Vec<rm_simulator_assets::rmuc_2027::Zone> =
            serde_json::from_slice(&std::fs::read(path)?)?;
        ensure!(
            zones.iter().all(|z| z.polygon_m.len() >= 3
                && z.polygon_m.iter().flatten().all(|v| v.is_finite())
                && z.floor_m.is_finite()
                && z.height_m.is_finite()
                && z.height_m > 0.
                && (z.floor_m + z.height_m).is_finite()),
            "invalid RFID overlay geometry"
        );
        self.overlay_zones = zones;
        self.overlay_source = Some(path.to_path_buf());
        Ok(())
    }
    /// Source and optional caller RFID footprints; no occupancy/rule evaluation.
    pub fn zones(&self) -> Vec<&rm_simulator_assets::rmuc_2027::Zone> {
        let mut result = self.overlay_zones.iter().collect::<Vec<_>>();
        if let Some(b) = &self.cad.rmuc_2027 {
            result.extend(
                b.world
                    .zones
                    .iter()
                    .chain(&b.world.supply_zones)
                    .chain(&b.world.vantage_zones)
                    .chain(b.world.terrain_zones.iter().map(|z| &z.zone)),
            );
        }
        result
    }
    /// Exported root pose including the glTF basis, applied exactly once.
    pub fn root(&self) -> Pose {
        self.asset()
            .placements
            .get(self.placement)
            .copied()
            .unwrap_or_else(|| self.cad.arena_pose())
    }
    /// Inspection JSON with provenance, counts, poses, clocks and limitations.
    pub fn report(&self) -> Value {
        let asset = self.asset();
        let source = self.cad.rmuc_2027.as_ref();
        json!({
            "asset": self.names[self.selected], "package_root": self.cad.root,
            "visual": self.cad.root.join(&asset.file), "collision": asset.physics_file(&self.cad.root),
            "verified_collision_contract": asset.source_tessellated_collision,
            "source_step_sha256": source.map(|b| &b.source_sha256),
            "placement_index": self.placement, "root_pose": self.root(),
            "frame": "world FLU metres; root includes glTF basis, apply only through shared helpers",
            "time_ns": self.timeline.time_ns, "tick_ns": TICK_NS, "playing": self.timeline.playing,
            "physical_preset": self.physical.as_ref().map(|p| format!("{:?}", p.scenario.preset())),
            "physical_time_ns": self.physical.as_ref().map(|p| p.session.stamp().time_ns),
            "truth_label": "privileged physics truth; not simulated sensors; raw contacts, no scoring or HP",
            "robot_spawn": self.robot_spawn().map(|r| json!({"team": r.team, "config": r.config, "pose": r.pose})),
            "robot_truth": self.robot_truth(), "projectiles": self.projectiles(),
            "contacts": self.contacts().iter().map(PhysicalContact::report).collect::<Vec<_>>(),
            "contact_count": self.physical.as_ref().map(|p| p.contact_count),
            "contact_retention": 128,
            "preview_law": "rotation +0.5 rad/s; translation +0.1 fraction/s, clamped; illustrative",
            "fixed_triangles": self.placed.fixed.triangles.len(),
            "moving": self.placed.moving.iter().enumerate().map(|(i,p)| json!({
                "channel": i, "mechanism": format!("{:?}", p.mechanism), "team": format!("{:?}", p.team),
                "frame": format!("{:?}", p.motion), "coordinate": self.coordinate(i),
                "triangles": p.mesh.triangles.len(),
            })).collect::<Vec<_>>(),
            "source_motions": source.map(|b| b.motions(&self.cad, asset).iter().map(|m| json!({
                "id": m.id, "bodies": m.bodies, "axis": m.axis, "pivot_m": m.pivot_m, "limits": m.limits
            })).collect::<Vec<_>>()),
            "detector_poses": self.detectors(), "rfid_footprints": self.zones(),
            "rfid_overlay_source_unverified": self.overlay_source,
            "limitations": "collider wire projection only; no visual materials/textures, occlusion, RF or match authority; isolated preset catch floor is flat z=0"
        })
    }
}

/// Validate a script before executing any actions; finite values and exact shapes
/// are required. Physics still validates command admission during execution.
///
/// ```
/// use rm_scenario_inspector::model::validate_script;
/// let script = serde_json::json!([{"step": 128}, {"aim_rad": [0.0, 0.1]}]);
/// validate_script(&script)?;
/// # Ok::<(), anyhow::Error>(())
/// ```
pub fn validate_script(script: &Value) -> anyhow::Result<()> {
    for action in script
        .as_array()
        .context("script must be a JSON action array")?
    {
        let object = action.as_object().context("action must be an object")?;
        ensure!(object.len() == 1, "action must have exactly one operation");
        let (key, value) = object.iter().next().unwrap();
        let len = match key.as_str() {
            "step" => {
                ensure!(value.as_u64().is_some(), "step must be unsigned ticks");
                continue;
            }
            "reset" => {
                ensure!(value == &Value::Bool(true), "reset must be true");
                continue;
            }
            "drive" => 3,
            "aim_rad" | "fire" => 2,
            "torques_nm" => 4,
            _ => anyhow::bail!("unknown script action {key}"),
        };
        let values = value.as_array().context("command needs numeric array")?;
        ensure!(
            values.len() == len
                && values
                    .iter()
                    .all(|v| v.as_f64().is_some_and(f64::is_finite)),
            "invalid {key} values"
        );
        if key == "fire" {
            ensure!(
                matches!(values[0].as_f64(), Some(17. | 42.))
                    && values[1].as_f64().is_some_and(|v| v > 0. && v <= 40.),
                "fire needs [17 or 42, speed in (0,40]]"
            );
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn paused_pulses_manual_step_reset_and_partitioning() {
        let mut a = Timeline::default();
        a.pulse().unwrap();
        assert_eq!(a.time_ns, 0);
        a.step(1).unwrap();
        assert_eq!(a.time_ns, TICK_NS);
        a.playing = true;
        a.pulse().unwrap();
        let mut b = Timeline::default();
        b.step(17).unwrap();
        assert_eq!(a.time_ns, b.time_ns);
        a.reset();
        assert_eq!(a.time_ns, 0);
        assert!(!a.playing);
    }
    #[test]
    fn script_validation_rejects_bad_shapes_before_execution() {
        validate_script(&json!([{"drive":[1.,0.,0.]},{"torques_nm":[0.,1.,0.,-1.]},{"aim_rad":[0.,0.1]},{"fire":[17,25.]},{"step":128},{"reset":true}])).unwrap();
        for invalid in [
            json!({}),
            json!([{"step":-1}]),
            json!([{"step":1,"reset":true}]),
            json!([{"fire":[18,25]}]),
            json!([{"fire":[42,41]}]),
            json!([{"drive":[1,2]}]),
            json!([{"torques_nm":[1,2]}]),
            json!([{"reset":false}]),
        ] {
            assert!(validate_script(&invalid).is_err(), "{invalid}");
        }
    }
    #[test]
    fn cad_free_physical_preview_drives_fires_and_rebuilds_robot() {
        use rm_simulator_physics::arena::TunnelModel;
        let scenario = Scenario::tunnel(TunnelModel {
            pose: Pose::at([10., 0., 0.]),
            opening_m: [2., 1., 0.65],
            wall_thickness_m: 0.1,
        })
        .unwrap();
        let config = ChassisConfig::default();
        let spawn = RobotSpawn {
            team: Team::Red,
            pose: Pose::at([0., 0., config.rest_height_m()]),
            config,
        };
        let session = scenario
            .session(
                StepDuration::HZ_128,
                vec![spawn.clone()],
                MechanismState::default(),
            )
            .unwrap();
        let mut preview = PhysicalPreview {
            scenario,
            session,
            spawn: Some(spawn),
            targets: vec![],
            contacts: vec![],
            contact_count: 0,
        };
        let initial = preview.session.robot_truth(0).unwrap().state;
        preview
            .session
            .command(RobotCommand::Drive {
                robot: 0,
                command: ChassisCommand {
                    forward_m_s: 1.,
                    ..Default::default()
                },
            })
            .unwrap();
        for tick in 1..=128 {
            preview
                .advance(tick * TICK_NS, &MechanismState::default())
                .unwrap();
        }
        assert!(
            preview
                .session
                .robot_truth(0)
                .unwrap()
                .state
                .pose
                .translation_m[0]
                > 0.1
        );
        preview
            .session
            .command(RobotCommand::WheelTorques {
                robot: 0,
                torques: WheelTorques::new(vec![0.5; 4]).unwrap(),
            })
            .unwrap();
        preview
            .advance(129 * TICK_NS, &MechanismState::default())
            .unwrap();
        assert!(matches!(
            preview.session.robot_truth(0).unwrap().state.drive_control,
            rm_simulator_physics::control::DriveControl::WheelTorques(_)
        ));
        let muzzle = preview
            .session
            .ground_truth()
            .chassis_muzzle_pose(0)
            .unwrap();
        preview
            .session
            .command(RobotCommand::Fire {
                robot: 0,
                shot: Shot {
                    caliber: Caliber::Mm17,
                    speed_m_s: 25.,
                },
            })
            .unwrap();
        assert_eq!(
            preview.session.ground_truth().snapshot()[0].position_m,
            muzzle.translation_m
        );
        preview.session.reset().unwrap();
        assert_eq!(preview.session.robot_truth(0).unwrap().state, initial);
        assert!(preview.session.ground_truth().snapshot().is_empty());
    }
    #[test]
    fn overflow_is_atomic() {
        let mut a = Timeline {
            time_ns: u64::MAX,
            playing: true,
        };
        assert!(a.step(1).is_err());
        assert_eq!(a.time_ns, u64::MAX);
        assert!(a.step(u64::MAX).is_err());
        assert_eq!(a.time_ns, u64::MAX);
    }
}

#[cfg(test)]
mod package_tests {
    use super::*;
    #[test]
    #[ignore = "requires the verified field-2027 LFS package"]
    fn source_robot_commands_contacts_and_reset() {
        let root = std::env::var_os("RM_INSPECTOR_TEST_ASSETS")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(cad_assets::default_cad_assets);
        let mut m = Inspector::load(&root, "outpost", 0).unwrap();
        let initial = m.robot_truth().unwrap();
        assert!(m.run_script(&json!([{"step":1},{"oops":1}])).is_err());
        assert_eq!(m.timeline.time_ns, 0);
        m.run_script(&json!([{"drive":[0.5,0.,0.]},{"step":64},{"torques_nm":[0.1,0.2,0.3,0.4]},{"aim_rad":[0.1,0.2]},{"step":16},{"fire":[17,25]}])).unwrap();
        assert_ne!(m.robot_truth().unwrap().pose, initial.pose);
        assert!(matches!(
            m.robot_truth().unwrap().drive_control,
            rm_simulator_physics::control::DriveControl::WheelTorques(_)
        ));
        let muzzle = m
            .physical
            .as_ref()
            .unwrap()
            .session
            .ground_truth()
            .chassis_muzzle_pose(0)
            .unwrap();
        assert_eq!(m.projectiles()[0].position_m, muzzle.translation_m);
        assert_eq!(m.projectiles()[0].shooter, Some(0));
        assert!(m.wheel_torques(vec![1.; 2]).is_err());
        assert!(m.aim(f64::NAN, 0.).is_err());
        // Controlled contact fixture immediately outside the real source detector.
        // The extra shot isolates contact reporting from chassis parking/aim accuracy.
        let face = m.detectors()[0];
        let normal = math::rotate(face.rotation_wxyz, [1., 0., 0.]);
        let position = std::array::from_fn(|i| face.translation_m[i] + normal[i] * 0.06);
        // A half-turn about local z reverses the local muzzle +x.
        let rotation = math::quat_multiply(face.rotation_wxyz, [0., 0., 0., 1.]);
        let time = m.timeline.time_ns;
        m.physical
            .as_mut()
            .unwrap()
            .session
            .physics_mut()
            .fire(
                time,
                Pose {
                    translation_m: position,
                    rotation_wxyz: rotation,
                },
                Shot {
                    caliber: Caliber::Mm17,
                    speed_m_s: 25.,
                },
                Some(0),
            )
            .unwrap();
        m.step(4).unwrap();
        assert!(m.contacts().iter().any(|c| c.kind == "projectile-armor"));
        assert!(m.contacts().iter().all(|c| c.time_ns <= m.timeline.time_ns));
        m.reset().unwrap();
        assert_eq!(m.robot_truth().unwrap(), initial);
        assert!(m.projectiles().is_empty());
        assert!(m.contacts().is_empty());
        assert_eq!(m.timeline.time_ns, 0);
        let old = m.report();
        let mut bad = m.robot_spawn().unwrap().clone();
        bad.pose.translation_m[0] = f64::NAN;
        assert!(m.configure_robot(Some(bad)).is_err());
        assert_eq!(m.report(), old);
        m.configure_robot(None).unwrap();
        assert!(m.robot_truth().is_none());
        assert!(m.fire(Caliber::Mm17, 25.).is_err());
    }
    #[test]
    #[ignore = "requires the verified field-2027 LFS package"]
    fn source_components_motion_detector_frames_and_reset() {
        let root = std::env::var_os("RM_INSPECTOR_TEST_ASSETS")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(cad_assets::default_cad_assets);
        let mut m = Inspector::load(&root, "outpost", 0).unwrap();
        assert!(!m.placed.fixed.triangles.is_empty());
        assert_eq!(m.placed.moving.len(), 1);
        assert!(!m.placed.moving[0].mesh.triangles.is_empty());
        assert_eq!(m.detectors().len(), 3);
        let before = crate::raster::render(&m, &Default::default());
        let pose = m.detectors()[0];
        m.set_coordinate(0, 0.5).unwrap();
        assert_ne!(pose, m.detectors()[0]);
        let source =
            &m.cad.rmuc_2027.as_ref().unwrap().world.outposts[m.placed.moving[0].team.index()];
        assert_eq!(m.detectors()[0], source.detector_pose(0, 0.5).unwrap());
        m.step(128).unwrap();
        assert_eq!(m.coordinate(0), 1.0);
        assert_eq!(
            m.physical.as_ref().unwrap().session.stamp().time_ns,
            m.timeline.time_ns
        );
        assert!(m.set_coordinate(0, f64::NAN).is_err());
        assert_eq!(m.coordinate(0), 1.0);
        m.reset().unwrap();
        assert_eq!(before, crate::raster::render(&m, &Default::default()));
        let rune = m.names.iter().position(|n| n == "rune").unwrap();
        m.select(rune, 0).unwrap();
        assert_eq!(m.detectors().len(), 10);
        let base = m.names.iter().position(|n| n == "base").unwrap();
        m.select(base, 1).unwrap();
        assert_eq!(m.placed.moving.len(), 3);
        m.set_coordinate(0, 2.).unwrap();
        assert_eq!(m.coordinate(0), 1.);
        m.set_coordinate(0, -2.).unwrap();
        assert_eq!(m.coordinate(0), 0.);
        assert!(!m.detectors().is_empty());
        // The package intentionally leaves RFID empty; the runtime adds its catalogue.
        assert!(m.zones().is_empty());
        let old = m.report();
        assert!(m.select(base, usize::MAX).is_err());
        assert_eq!(old, m.report());
    }
}
