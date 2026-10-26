// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native RoboMaster field and shared physics on an explicit instance-owned clock.
//! No renderer, host clock, transport or background threads. Projectiles fly in
//! the shared `rm-sim-physics` world. This facade coordinates its contacts and
//! motion with live rules, scoring, referee decisions and complete restore.
#![deny(missing_docs)]

pub mod assembly;
pub mod base;
pub mod energy_unit;
pub use base::{BaseConfig, BaseSnapshot};
pub mod chassis;
mod config;
pub use config::{ChassisPlacement, FieldConfig, OutpostConfig, RuneConfig};
mod snapshot;
pub use snapshot::{ArmorSnapshot, FieldRestore, FieldSnapshot};
pub mod outpost;
pub mod projectile;
pub mod referee;
pub mod rmuc_2027;
mod rmul_2027;
pub mod rule_adapters;
pub mod rune;
mod scoring;
pub mod session;
pub mod zones;

pub use chassis::{ChassisCommand, ChassisConfig, ChassisSnapshot, WheelContact, WheelSnapshot};
pub use outpost::{Outpost, OutpostSnapshot};
pub use projectile::{
    ArmorHit, ArmorTarget, Caliber, ProjectileSnapshot, Rejection, SMALL_ARMOR_HOUSING_HALF_M,
    Shot, StaticGeometry,
};
use projectile::{Contact, HIT_MEMORY_NS, TargetFace, TargetFrames, WorldPhysics};
pub use referee::{
    BuffState, MatchPhase, Referee, RefereeCommand, RefereeConfig, RefereeEvent, RefereeSnapshot,
    RobotConfig, RobotKind, RobotSnapshot, RuneStage, StampClock, Team, TeamSnapshot, TimedEvent,
};
pub use rm_sim_physics::control::{DriveControl, WheelTorques};
use rune::RuneError;
pub use rune::{BigRuneMotion, HitOutcome, Rune, RuneKind, RuneSnapshot, RuneState};
pub use session::{MatchCheckpoint, MatchSession};

/// Native gameplay authority and shared serialized records.
pub use rm_sim_gameplay as gameplay;
/// Season identities and admission checks shared with standalone gameplay.
pub use rm_sim_gameplay::season;
pub use rm_sim_physics::{Pose, StepDuration, tick_ns};

/// Everything a `Field` can refuse: an invalid layout, a launch the referee
/// blocks, a physics step that cannot resolve a target frame, or a restore
/// whose snapshot disagrees with the geometry on hand.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum FieldError {
    /// A rune pose or motion parameter failed validation (section 5.5.2 bounds).
    #[error("invalid rune configuration: {0}")]
    Rune(#[from] RuneError),
    /// An outpost origin, pivot or speed failed validation.
    #[error("invalid outpost configuration: {0}")]
    Outpost(&'static str),
    /// The shot could not be launched, for example at a defeated shooter.
    #[error("invalid shot: {0}")]
    Shot(&'static str),
    /// An unknown chassis id or a rejected chassis placement.
    #[error("invalid chassis: {0}")]
    Chassis(&'static str),
    /// A collision mesh or projectile bound was rejected.
    #[error("invalid collision mesh: {0}")]
    Mesh(&'static str),
    /// The physics step could not resolve its scoring target frames.
    #[error("projectile stepping failed: {0}")]
    Ballistics(&'static str),
    /// `step` would carry the tick counter past the range of the nanosecond clock.
    #[error("field tick overflow")]
    TickOverflow,
    /// The referee refused the command, or the field has no referee.
    #[error("referee: {0}")]
    Referee(&'static str),
    /// The snapshot cannot rebuild a field: rule state, floor height or HP is wrong.
    #[error("cannot restore the field: {0}")]
    Restore(&'static str),
}

/// The single admitted authority. Wire DTOs retain their historical optional fields.
enum Authority {
    Bare,
    Rmuc(Box<rmuc_2027::State>),
    Rmul(gameplay::rmul_2027::Match),
}
impl Authority {
    fn from_records(
        uc: Option<rmuc_2027::State>,
        ul: Option<gameplay::rmul_2027::Match>,
    ) -> Result<Self, FieldError> {
        match (uc, ul) {
            (None, None) => Ok(Self::Bare),
            (Some(state), None) => Ok(Self::Rmuc(Box::new(state))),
            (None, Some(game)) => Ok(Self::Rmul(game)),
            (Some(_), Some(_)) => Err(FieldError::Restore("mixed competition configuration")),
        }
    }
    fn rmuc(&self) -> Option<&rmuc_2027::State> {
        if let Self::Rmuc(state) = self {
            Some(state)
        } else {
            None
        }
    }
    fn rmuc_mut(&mut self) -> Option<&mut rmuc_2027::State> {
        if let Self::Rmuc(state) = self {
            Some(state)
        } else {
            None
        }
    }
    fn rmul(&self) -> Option<&gameplay::rmul_2027::Match> {
        if let Self::Rmul(game) = self {
            Some(game)
        } else {
            None
        }
    }
    fn rmul_mut(&mut self) -> Option<&mut gameplay::rmul_2027::Match> {
        if let Self::Rmul(game) = self {
            Some(game)
        } else {
            None
        }
    }
}

/// The complete field: one physics world holding projectiles and chassis, with
/// the runes, outposts, bases and optional referee on top. Reading a snapshot
/// never advances time.
///
/// ```rust
/// use rm_sim_world::{Field, FieldConfig, tick_ns};
///
/// let mut field = Field::new(&FieldConfig::default()).unwrap();
/// assert_eq!(field.tick(), 0);
/// assert_eq!(field.time_ns(), 0);
/// // Time moves only through explicit ticks of tick_ns() nanoseconds.
/// field.step(1).unwrap();
/// assert_eq!(field.tick(), 1);
/// assert_eq!(field.time_ns(), tick_ns());
/// ```
pub struct Field {
    // Reusable adapter scratch, deliberately excluded from checkpoints.
    rmuc_terrain_cards: Vec<gameplay::rmuc_2027::terrain::Card>,
    energy_units: Vec<energy_unit::EnergyUnit>,
    arms: Vec<assembly::ArmState>,
    assemblies: Vec<assembly::Run>,
    authority: Authority,
    floor_height_m: f64,
    tick: u64,
    physics: WorldPhysics,
    target_frames: TargetFrames,
    hits: Vec<ArmorHit>,
    hits_detected: u64,
    last_detection_ns: std::collections::HashMap<ArmorTarget, u64>,
}
impl Field {
    /// Build a native match from source bindings, or a bare physical world.
    /// Historical rule configurations are rejected with the archive tag.
    ///
    /// ```
    /// use rm_sim_world::{Field, FieldConfig};
    /// let field = Field::new(&FieldConfig::default()).unwrap();
    /// assert!(field.rmuc_2027().is_none());
    /// assert!(field.snapshot().chassis.is_empty());
    /// ```
    pub fn new(config: &FieldConfig) -> Result<Self, FieldError> {
        if config.referee.is_some()
            || !config.runes.is_empty()
            || !config.outposts.is_empty()
            || !config.bases.is_empty()
            || !config.zones.is_empty()
        {
            return Err(FieldError::Restore(
                "2026 runtime removed; use rm2026-final",
            ));
        }
        if config.rmuc_2027.is_some() && config.rmul_2027.is_some() {
            return Err(FieldError::Restore("mixed competition configuration"));
        }
        if let Some(native) = &config.rmuc_2027 {
            native.validate().map_err(FieldError::Restore)?;
        }
        let mut faces = Vec::new();
        let rmuc_2027 = config.rmuc_2027.clone().map(|config| rmuc_2027::State {
            game: rmuc_2027::match_rules::Match::new(config.rotor_direction),
            config,
        });
        Self::write_target_faces(rmuc_2027.as_ref(), &mut faces);
        let mut field = Self {
            energy_units: Vec::new(),
            arms: Vec::new(),
            assemblies: Vec::new(),
            authority: Authority::from_records(
                rmuc_2027,
                config.rmul_2027.map(gameplay::rmul_2027::Match::new),
            )?,
            floor_height_m: config.floor_height_m,
            tick: 0,
            physics: {
                let mut physics = WorldPhysics::with_step_duration(
                    &faces,
                    config.floor_height_m,
                    config.step_duration,
                );
                physics
                    .set_projectile_policy(config.projectile_policy)
                    .map_err(FieldError::Shot)?;
                physics
            },
            rmuc_terrain_cards: Vec::new(),
            target_frames: TargetFrames::new(faces),
            hits: Vec::new(),
            hits_detected: 0,
            last_detection_ns: std::collections::HashMap::new(),
        };
        for chassis in &config.chassis {
            field.add_chassis(chassis)?;
        }
        Ok(field)
    }
    /// Refill scoring faces in their fixed construction order, keeping capacity.
    fn write_target_faces(native: Option<&rmuc_2027::State>, faces: &mut Vec<TargetFace>) {
        faces.clear();
        if let Some(state) = native {
            state.faces(faces);
        }
    }
    /// Install a cooked fixed model at an ordinary FLU pose, preserving native grids.
    pub fn add_fixed_model(
        &mut self,
        model: &rm_sim_physics::cooking::CookedRigidBody,
        placement: Pose,
    ) -> Result<(), FieldError> {
        self.physics
            .add_fixed_model(model, placement)
            .map(|_| ())
            .map_err(FieldError::Mesh)
    }
    /// Add a fixed collision mesh (world FLU metres), for example the field CAD.
    pub fn add_static_mesh(
        &mut self,
        vertices_m: Vec<[f64; 3]>,
        triangles: Vec<[u32; 3]>,
    ) -> Result<(), FieldError> {
        self.physics
            .add_static_mesh(vertices_m, triangles)
            .map(|_| ())
            .map_err(FieldError::Mesh)
    }
    /// Insert shared cooked scenery and its motion/endpoint-cache records.
    /// The caller prepares collision geometry; this field retains scoring bodies
    /// and resolves source channels at its current mechanism state without
    /// advancing time. This uses the same insertion path as checkpoint restore.
    pub fn add_prepared_geometry(&mut self, geometry: &StaticGeometry) -> Result<(), FieldError> {
        self.physics
            .try_insert_geometry(geometry)
            .map_err(FieldError::Mesh)?;
        self.sync_mechanisms();
        Ok(())
    }
    /// Add fixed native scenery in world-FLU child frames with the same contact
    /// policy as field meshes. Native shapes remain analytic through captures.
    pub fn add_static_shapes(
        &mut self,
        shapes: &[(Pose, rm_sim_physics::model::ShapeDescription)],
    ) -> Result<(), FieldError> {
        self.physics
            .add_static_shapes(shapes)
            .map_err(FieldError::Mesh)
    }
    /// Add native children to a prescribed source scenery channel at rest.
    /// Translations are metres; rotor axes are unit FLU vectors. Shape children
    /// retain their rest frames and follow the supplied mesh channel's motion.
    pub fn add_mechanism_shapes(
        &mut self,
        shapes: &[(Pose, rm_sim_physics::model::ShapeDescription)],
        mechanism: referee::Mechanism,
        team: Team,
        frame: referee::MotionFrame,
    ) -> Result<(), FieldError> {
        self.physics
            .add_mechanism_shapes(shapes, mechanism, team, frame)
            .map_err(FieldError::Mesh)
    }
    /// Register an operator-driven mesh at its exported rest pose, with world
    /// FLU translations for the closed and open endpoints. No inferred rules.
    pub fn add_mechanism_mesh(
        &mut self,
        vertices_m: Vec<[f64; 3]>,
        triangles: Vec<[u32; 3]>,
        mechanism: referee::Mechanism,
        team: Team,
        offsets_m: [[f64; 3]; 2],
    ) -> Result<(), FieldError> {
        self.physics
            .add_mechanism_mesh(vertices_m, triangles, mechanism, team, offsets_m)
            .map_err(FieldError::Mesh)?;
        self.sync_mechanisms();
        Ok(())
    }
    /// Register a source rotor's exact rest-frame world triangles, pivot in
    /// metres and dimensionless unit axis. Rules supply the angle each tick.
    pub fn add_rotor_mesh(
        &mut self,
        vertices_m: Vec<[f64; 3]>,
        triangles: Vec<[u32; 3]>,
        team: Team,
        pivot_m: [f64; 3],
        axis: [f64; 3],
    ) -> Result<(), FieldError> {
        self.physics
            .add_rotor_mesh(vertices_m, triangles, team, pivot_m, axis)
            .map_err(FieldError::Mesh)
    }

    /// Register an explicitly driven Outpost/Rune rotor from world rest-frame
    /// triangles, a pivot in metres and a dimensionless unit source axis.
    pub fn add_rotating_mesh(
        &mut self,
        vertices_m: Vec<[f64; 3]>,
        triangles: Vec<[u32; 3]>,
        mechanism: referee::Mechanism,
        team: Team,
        pivot_m: [f64; 3],
        axis: [f64; 3],
    ) -> Result<(), FieldError> {
        self.physics
            .add_rotating_mesh(vertices_m, triangles, mechanism, team, pivot_m, axis)
            .map_err(FieldError::Mesh)?;
        self.sync_mechanisms();
        Ok(())
    }

    /// The fixed collision geometry the physics actually holds (meshes
    /// and boxes; not the catch floor), as one triangle soup in
    /// world FLU metres, for debugging views.
    pub fn static_geometry(&self) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
        self.static_geometry_snapshot().into_triangles()
    }
    /// Robot containment mesh ignored by projectile collision queries.
    /// Pair with `set_projectile_bounds` to discard shots at the perimeter.
    pub fn add_boundary_mesh(
        &mut self,
        vertices_m: Vec<[f64; 3]>,
        triangles: Vec<[u32; 3]>,
    ) -> Result<(), FieldError> {
        self.physics
            .add_boundary_mesh(vertices_m, triangles)
            .map_err(FieldError::Mesh)
    }
    /// Absorb projectiles touching or leaving the arena's XY perimeter at any height.
    /// This is a simulator setting. Robot collision walls remain ordinary geometry.
    pub fn set_projectile_bounds(&mut self, bounds_m: [[f64; 3]; 2]) -> Result<(), FieldError> {
        if !bounds_m.iter().flatten().all(|v| v.is_finite())
            || (0..2).any(|i| bounds_m[0][i] >= bounds_m[1][i])
        {
            return Err(FieldError::Mesh("invalid projectile bounds"));
        }
        self.physics.projectile_bounds_m = Some(bounds_m);
        Ok(())
    }
    /// CAD-backed arenas use their own floor mesh and perimeter walls.
    pub fn remove_catch_floor(&mut self) {
        self.physics.remove_catch_floor();
    }
    /// Configured catch-plane height; CAD arenas may have removed that plane.
    pub fn floor_height_m(&self) -> f64 {
        self.floor_height_m
    }
    /// Capture shared shapes without copying mesh buffers or advancing time.
    pub fn static_geometry_snapshot(&self) -> StaticGeometry {
        self.physics.static_geometry_snapshot(true)
    }
    /// Current chassis, armor and projectile collider shapes in world coordinates.
    /// Fixed debug geometry excludes articulated equipment captured each frame.
    pub fn fixed_geometry_snapshot(&self) -> StaticGeometry {
        self.physics.static_geometry_snapshot(false)
    }
    /// Chassis, armor, projectile and mechanism collider shapes at the current
    /// tick, in world FLU metres. Fixed terrain is excluded.
    pub fn dynamic_geometry_snapshot(&self) -> StaticGeometry {
        self.physics.dynamic_geometry_snapshot()
    }

    /// Capture enabled geometry and this robot's actual pose at the current
    /// simulation time for a mounted rolling LiDAR. Own-robot occluders remain.
    /// This is trusted ground truth for sensor generation, not a wire command.
    pub fn lidar_scene_capture(
        &self,
        robot: u32,
    ) -> Result<rm_sim_physics::sensors::rolling::SceneCapture, &'static str> {
        self.physics.lidar_scene_capture(robot, self.time_ns())
    }

    /// Start or stop recording every chassis' body and actual gimbal pose at
    /// the end of every solver substep, for mounted inertial sensors
    /// ([`rm_sim_physics::sensors::imu::MountedImu`]). Either call
    /// discards undrained samples. This is trusted ground truth for sensor
    /// generation; it is not carried by checkpoints, so re-enable it after
    /// [`Self::restore`] and restore each sensor's own state.
    ///
    /// ```
    /// use rm_sim_world::{ChassisConfig, ChassisPlacement, Field, FieldConfig,
    ///     Pose, RobotKind, Team};
    /// let config = FieldConfig {
    ///     chassis: vec![ChassisPlacement {
    ///         config: ChassisConfig::default(),
    ///         spawn: Pose::at([0., 0., 0.4]),
    ///         team: Team::Red,
    ///         kind: RobotKind::Infantry,
    ///         performance: None,
    ///     }],
    ///     ..Default::default()
    /// };
    /// let (mut whole, mut split) = (Field::new(&config)?, Field::new(&config)?);
    /// whole.set_carrier_trace(true);
    /// split.set_carrier_trace(true);
    /// whole.step(6)?;
    /// let (mut a, mut b) = (Vec::new(), Vec::new());
    /// whole.drain_carrier_trace(&mut a);
    /// for _ in 0..3 {
    ///     split.step(2)?;
    ///     split.drain_carrier_trace(&mut b);
    /// }
    /// // Eight substeps per 128 Hz tick, independent of tick partitioning.
    /// assert_eq!(a.len(), 48);
    /// assert_eq!(a, b);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn set_carrier_trace(&mut self, enabled: bool) {
        self.physics.set_carrier_trace(enabled);
    }

    /// Move recorded substep carrier samples, in time order, into `out`.
    /// Drain at least once per few seconds of simulated time; see
    /// [`rm_sim_physics::sensors::imu::MAX_CARRIER_TRACE_SAMPLES`].
    pub fn drain_carrier_trace(
        &mut self,
        out: &mut Vec<rm_sim_physics::sensors::imu::CarrierSample>,
    ) {
        self.physics.drain_carrier_trace(out);
    }

    /// Carrier samples dropped because the trace was not drained in time.
    pub fn carrier_trace_dropped(&self) -> u64 {
        self.physics.carrier_trace_dropped()
    }

    /// Launch a projectile in the shared physical world, with an optional shooter.
    /// Native matches enforce phase, HP, launcher locks, heat and allowance.
    /// Returns the projectile identity.
    ///
    /// ```
    /// use rm_sim_world::{Caliber, Field, FieldConfig, Pose, Shot};
    /// let mut field = Field::new(&FieldConfig::default()).unwrap();
    /// field.fire(Pose::at([0.,0.,1.]), Shot::at_limit(Caliber::Mm17), None).unwrap();
    /// assert_eq!(field.snapshot().shots_fired, 1);
    /// ```
    pub fn fire(
        &mut self,
        muzzle: Pose,
        shot: Shot,
        shooter: Option<u32>,
    ) -> Result<u64, FieldError> {
        if let Some(state) = self.authority.rmuc() {
            if shot.caliber == Caliber::Mm42
                && !shooter.is_some_and(|id| {
                    state.game.robots.iter().any(|r| {
                        r.id == id
                            && r.role == rmuc_2027::match_rules::GroundRole::Heavy
                            && r.combat.heavy_installed
                    })
                })
            {
                return Err(FieldError::Referee("42 mm requires a converted Heavy"));
            }
            if let Some(id) = shooter {
                let heavy = state
                    .game
                    .robots
                    .iter()
                    .any(|r| r.id == id && r.role == rmuc_2027::match_rules::GroundRole::Heavy);
                if (shot.caliber == Caliber::Mm42) != heavy {
                    return Err(FieldError::Referee("wrong robot caliber"));
                }
                state.game.check_launch(id).map_err(FieldError::Referee)?;
            } else if !matches!(
                state.game.phase,
                rmuc_2027::match_rules::Phase::Idle | rmuc_2027::match_rules::Phase::Running
            ) {
                return Err(FieldError::Referee(
                    "2027 launcher unavailable in this phase",
                ));
            }
        }
        if let Some(game) = self.authority.rmul() {
            let shooter = shooter.ok_or(FieldError::Referee("RMUL launch requires a robot"))?;
            game.check_launch(shooter, shot.caliber == Caliber::Mm42)
                .map_err(FieldError::Referee)?;
        }
        let id = self
            .physics
            .fire(self.time_ns(), muzzle, shot, shooter)
            .map_err(FieldError::Shot)?;
        if let (Some(state), Some(id)) = (self.authority.rmuc_mut(), shooter) {
            state.game.record_launch(id, shot.speed_m_s);
        }
        if let (Some(game), Some(shooter)) = (self.authority.rmul_mut(), shooter) {
            game.fire(shooter, shot.caliber == Caliber::Mm42, shot.speed_m_s);
        }
        Ok(id)
    }
    /// Buy one pilot ammunition unit during a round. Legacy 2026 uses ten
    /// 17 mm rounds or one 42 mm round; native 2027 chooses ten zone rounds or
    /// 100 remote rounds with six-second delivery (Tables 5-7–5-9).
    pub fn buy_ammo(&mut self, chassis: u32, caliber: Caliber) -> Result<(), FieldError> {
        if let Some(state) = self.authority.rmuc_mut() {
            let heavy =
                state.game.robots.iter().any(|r| {
                    r.id == chassis && r.role == rmuc_2027::match_rules::GroundRole::Heavy
                });
            if (caliber == Caliber::Mm42) != heavy {
                return Err(FieldError::Referee("wrong robot caliber"));
            }
            let amount = if heavy {
                if state.game.zone_exchange_available(chassis) {
                    1
                } else {
                    10
                }
            } else if state.game.zone_exchange_available(chassis) {
                10
            } else {
                100
            };
            return state
                .game
                .buy_ammo(chassis, amount)
                .map_err(FieldError::Referee);
        }
        self.referee_command(RefereeCommand::BuyAmmo {
            robot: chassis,
            caliber,
            amount: match caliber {
                Caliber::Mm17 => 10,
                Caliber::Mm42 => 1,
            },
        })
    }

    /// Put a chassis on the field and return its id; ids count up and are
    /// never reused. The referee, when there is one, opens a robot record
    /// under the same id.
    ///
    /// ```rust
    /// use rm_sim_world::{
    ///     ChassisConfig, ChassisPlacement, Field, FieldConfig, Pose, RobotKind, Team,
    /// };
    ///
    /// let mut field = Field::new(&FieldConfig {
    ///     runes: vec![],
    ///     outposts: vec![],
    ///     ..FieldConfig::default()
    /// })
    /// .unwrap();
    /// let placement = |x_m| ChassisPlacement {
    ///     config: ChassisConfig::default(),
    ///     spawn: Pose::at([x_m, 0.0, ChassisConfig::default().rest_height_m()]),
    ///     team: Team::Blue,
    ///     kind: RobotKind::Infantry,
    ///     performance: None,
    /// };
    /// assert_eq!(field.add_chassis(&placement(0.0)).unwrap(), 0);
    /// assert_eq!(field.add_chassis(&placement(1.0)).unwrap(), 1);
    /// field.remove_chassis(0).unwrap();
    /// // A freed id is never handed out again.
    /// assert_eq!(field.add_chassis(&placement(2.0)).unwrap(), 2);
    /// assert_eq!(field.chassis_team(2), Some(Team::Blue));
    /// ```
    pub fn add_chassis(&mut self, placement: &ChassisPlacement) -> Result<u32, FieldError> {
        if self.authority.rmuc().is_some()
            && !matches!(
                placement.kind,
                RobotKind::Infantry | RobotKind::Sentry | RobotKind::Engineer
            )
        {
            return Err(FieldError::Referee("unsupported 2027 ground role"));
        }
        if self.authority.rmuc().is_some()
            && placement.performance.is_some_and(|p| {
                !matches!(
                    (placement.kind, p),
                    (RobotKind::Infantry, gameplay::Performance::Infantry { .. })
                )
            })
        {
            return Err(FieldError::Referee(
                "native 2027 deployment accepts only Infantry performance choices",
            ));
        }
        if let Some(game) = self.authority.rmul() {
            self.check_rmul_placement(game, placement)?;
        }
        let id = self
            .physics
            .add_chassis(placement.team, placement.config.clone(), placement.spawn)
            .map_err(FieldError::Chassis)?;
        if let Some(state) = self.authority.rmuc_mut() {
            let role = match placement.kind {
                RobotKind::Infantry => rmuc_2027::match_rules::GroundRole::Infantry,
                RobotKind::Sentry => rmuc_2027::match_rules::GroundRole::Sentry,
                RobotKind::Engineer => rmuc_2027::match_rules::GroundRole::Heavy,
                _ => return Err(FieldError::Referee("unsupported 2027 ground role")),
            };
            state
                .game
                .add_robot(id, referee::game_team(placement.team), role);
            if let Some(gameplay::Performance::Infantry { chassis, launcher }) =
                placement.performance
            {
                let r = state.game.robots.iter_mut().find(|r| r.id == id).unwrap();
                r.combat.chassis = chassis;
                r.combat.launcher = launcher;
                r.max_hp = r.stats(state.game.control.snapshot().elapsed_ns).max_hp;
                r.hp = r.max_hp;
            }
        }
        if let Some(game) = self.authority.rmul_mut() {
            game.add_robot(
                id,
                referee::game_team(placement.team),
                Self::rmul_role(placement.kind)?,
            )
            .map_err(FieldError::Referee)?;
            if let Some(gameplay::Performance::Infantry { chassis, launcher }) =
                placement.performance
            {
                game.configure_infantry(
                    id,
                    chassis == gameplay::InfantryChassis::PowerFocused,
                    launcher == gameplay::InfantryLauncher::BurstFocused,
                )
                .map_err(FieldError::Referee)?;
            }
        }
        Ok(id)
    }
    /// Take a chassis off the field, with its robot record.
    pub fn remove_chassis(&mut self, id: u32) -> Result<(), FieldError> {
        if let Some(game) = self.authority.rmul_mut() {
            game.robots.retain(|r| r.id != id);
        }
        self.cancel_assembly(id, "Assembly owner disconnected");
        self.release_energy_units(id);
        self.arms.retain(|a| a.owner != id);
        self.physics
            .remove_chassis(id)
            .map_err(FieldError::Chassis)?;
        if let Some(state) = self.authority.rmuc_mut() {
            state.game.robots.retain(|r| r.id != id);
            for marks in &mut state.game.radar.marks {
                marks.remove(&id);
            }
            for info in &mut state.game.radar.information {
                info.remove(&id);
            }
            if state.game.phase == rmuc_2027::match_rules::Phase::Running {
                state.game.activations();
            }
        }
        Ok(())
    }
    /// Reposition an existing chassis for inspection without changing its HP or id.
    pub fn place_chassis(&mut self, id: u32, spawn: Pose) -> Result<(), FieldError> {
        self.physics
            .place_chassis(id, spawn)
            .map_err(FieldError::Chassis)
    }
    /// Placement revision of a chassis id; it changes on placement or revival,
    /// so a client never interpolates across robot lives. `None` for an unknown id.
    pub fn chassis_revision(&self, id: u32) -> Option<u64> {
        self.physics.chassis_revision(id)
    }
    /// Whether the referee has the chassis' robot at zero HP; `None` for an
    /// unknown id. A defeated robot cannot drive, aim or fire and absorbs no damage.
    pub fn chassis_defeated(&self, id: u32) -> Option<bool> {
        self.physics.chassis_defeated(id)
    }
    /// Cut the drive of every chassis whose robot the referee has defeated
    /// and restore the revived ones.
    fn sync_defeats(&mut self) {
        if let Some(game) = self.authority.rmul() {
            for robot in &game.robots {
                self.physics.set_chassis_defeated(robot.id, robot.hp == 0);
            }
        }
        if let Some(state) = self.authority.rmuc() {
            for robot in &state.game.robots {
                self.physics.set_chassis_defeated(robot.id, robot.hp == 0);
            }
        }
    }
    /// Set a chassis' desired body velocity; it holds until replaced. The
    /// omni inverse kinematics derives wheel speeds from it, and the motors
    /// decide how much of the wish the drivetrain budget allows.
    ///
    /// ```rust
    /// use rm_sim_world::{
    ///     ChassisCommand, ChassisConfig, ChassisPlacement, Field, FieldConfig, Pose, RobotKind,
    ///     Team,
    /// };
    ///
    /// let config = FieldConfig {
    ///     chassis: vec![ChassisPlacement {
    ///         config: ChassisConfig::default(),
    ///         spawn: Pose::at([0.0, 0.0, ChassisConfig::default().rest_height_m()]),
    ///         team: Team::Red,
    ///         kind: RobotKind::Infantry,
    ///         performance: None,
    ///     }],
    ///     runes: vec![],
    ///     outposts: vec![],
    ///     ..FieldConfig::default()
    /// };
    /// let mut field = Field::new(&config).unwrap();
    /// field
    ///     .command_chassis(
    ///         0,
    ///         ChassisCommand {
    ///             forward_m_s: 2.0,
    ///             ..ChassisCommand::default()
    ///         },
    ///     )
    ///     .unwrap();
    /// field.step(1_000).unwrap();
    /// let moved = field.chassis_positions_m().next().unwrap();
    /// assert!(moved[0] > 0.5, "{moved:?}");
    /// ```
    pub fn command_chassis(&mut self, id: u32, command: ChassisCommand) -> Result<(), FieldError> {
        if command.forward_m_s.abs() > 0.01
            || self
                .arms
                .iter()
                .any(|a| a.owner == id && a.folded != command.folded)
            || command.left_m_s.abs() > 0.01
            || command.yaw_rate_rad_s.abs() > 0.01
        {
            self.cancel_assembly(id, "Cancelled by manual driving");
        }
        self.physics
            .command_chassis(id, command)
            .map_err(FieldError::Chassis)
    }
    /// Hold direct wheel-axis motor torques through the ordinary physical plant.
    /// Valid admission cancels automatic assembly; gimbal and folding targets
    /// are preserved. Native referee power/defeat constraints remain in force.
    /// Use [`Self::command_chassis`] to resume body-velocity control.
    ///
    /// ```
    /// use rm_sim_world::{Field, FieldConfig, ChassisPlacement, ChassisConfig,
    ///     Pose, Team, RobotKind, WheelTorques, DriveControl};
    /// let mut field = Field::new(&FieldConfig::default()).unwrap();
    /// let id = field.add_chassis(&ChassisPlacement {
    ///     config: ChassisConfig::default(), spawn: Pose::at([0., 0., 0.3]),
    ///     team: Team::Red, kind: RobotKind::Infantry, performance: None,
    /// }).unwrap();
    /// field.command_chassis_wheel_torques(id, WheelTorques::new(vec![0.; 4]).unwrap()).unwrap();
    /// assert!(matches!(field.chassis_snapshot(id).unwrap().drive_control,
    ///     DriveControl::WheelTorques(_)));
    /// ```
    pub fn command_chassis_wheel_torques(
        &mut self,
        id: u32,
        torques: WheelTorques,
    ) -> Result<(), FieldError> {
        self.physics
            .command_chassis_wheel_torques(id, torques)
            .map_err(FieldError::Chassis)?;
        self.cancel_assembly(id, "Cancelled by direct wheel driving");
        Ok(())
    }
    /// Install a source Hero practice chassis on an eligible native Heavy.
    /// RMUC conversion bypasses workbench installation and confirmation.
    /// RMUL requires a parked chassis at its practice workbench; upgraded
    /// properties remain locked until the confirmation shot.
    pub fn convert_heavy_to_hero(&mut self, id: u32) -> Result<(), FieldError> {
        if self.authority.rmul().is_some() {
            return self.convert_rmul_heavy(id);
        }
        let state = self
            .authority
            .rmuc()
            .ok_or(FieldError::Referee("conversion requires RMUC 2027"))?;
        let robot = state
            .game
            .robots
            .iter()
            .find(|r| r.id == id && r.role == rmuc_2027::match_rules::GroundRole::Heavy)
            .ok_or(FieldError::Referee("only Heavy can convert"))?;
        if !state.game.heavy_unlocked(robot.team) || robot.hp == 0 || !robot.online {
            return Err(FieldError::Referee(
                "Heavy launcher permission is locked or robot unavailable",
            ));
        }
        if robot.combat.heavy_installed {
            return Err(FieldError::Referee("Heavy already converted"));
        }
        if self
            .assemblies
            .iter()
            .any(|r| r.owner == id && r.station_busy())
        {
            return Err(FieldError::Referee(
                "finish assembly and Core recovery before converting",
            ));
        }
        let mut body = self
            .physics
            .chassis_snapshot(id)
            .ok_or(FieldError::Chassis("unknown chassis"))?;
        if body.velocity_m_s.iter().any(|v| v.abs() > 0.1)
            || body.angular_velocity_rad_s.iter().any(|v| v.abs() > 0.1)
        {
            return Err(FieldError::Referee("stop before converting Heavy"));
        }
        let config = ChassisConfig::folding_hero();
        body.pose.translation_m[2] += config.rest_height_m() - body.config.rest_height_m();
        body.config = config;
        body.fold_angle_rad = 0.;
        body.command = ChassisCommand::default();
        for wheel in &mut body.wheels {
            wheel.contact = None;
            wheel.angular_velocity_rad_s = 0.;
        }
        body.placement_revision = body.placement_revision.saturating_add(1);
        self.physics
            .replace_chassis(&body)
            .map_err(FieldError::Chassis)?;
        self.cancel_assembly(id, "Converted to Hero approximation");
        self.release_energy_units(id);
        self.arms.retain(|a| a.owner != id);
        let state = self.authority.rmuc_mut().unwrap();
        let robot = state.game.robots.iter_mut().find(|r| r.id == id).unwrap();
        robot.combat.heavy_installed = true;
        robot.hp = robot.hp.saturating_add(600 - robot.max_hp).min(600);
        robot.max_hp = 600;
        Ok(())
    }
    /// The preset a chassis id was placed with; `None` for an unknown id.
    pub fn chassis_config(&self, id: u32) -> Option<&ChassisConfig> {
        self.physics.chassis_config(id)
    }
    /// The team a chassis id plays for; `None` for an unknown id.
    pub fn chassis_team(&self, id: u32) -> Option<Team> {
        self.physics.chassis_team(id)
    }
    /// Authoritative muzzle pose for a chassis' held turret aim.
    /// The muzzle is 0.35 m along the turret's +x barrel line.
    pub fn chassis_muzzle_pose(&self, id: u32) -> Option<Pose> {
        self.physics.chassis_muzzle_pose(id)
    }
    /// The ball restitution, flight limit and retirement rule this field runs.
    pub fn projectile_policy(&self) -> projectile::ProjectilePolicy {
        self.physics.projectile_policy()
    }
    /// Take the recent projectile removal records, oldest first. Lifetime
    /// instrumentation; no rule reads them.
    pub fn take_projectile_removals(&mut self) -> Vec<projectile::Removal> {
        self.physics.take_removals()
    }
    /// Every ball in flight, without building a whole field snapshot; a
    /// provisional-shot replay reads its own balls and nothing else.
    pub fn projectile_snapshots(&self) -> Vec<ProjectileSnapshot> {
        self.physics.snapshot()
    }
    /// Allocation-free state for an addressed controller, excluding geometry and wheels.
    ///
    /// ```
    /// let field = rm_sim_world::Field::new(&Default::default())?;
    /// assert!(field.chassis_control_state(123).is_none());
    /// # Ok::<(), rm_sim_world::FieldError>(())
    /// ```
    pub fn chassis_control_state(&self, id: u32) -> Option<chassis::ChassisControlState> {
        self.physics.chassis_control_state(id)
    }
    /// Borrow compact body/gimbal state and lazily derived wheel telemetry.
    /// This is privileged physical truth, without copying model geometry or a checkpoint.
    pub fn chassis_telemetry(&self, id: u32) -> Option<chassis::ChassisTelemetry<'_>> {
        self.physics.chassis_telemetry(id)
    }

    /// One chassis' snapshot on its own, without building a whole field
    /// snapshot; a replay reads its own robot after every tick.
    pub fn chassis_snapshot(&self, id: u32) -> Option<ChassisSnapshot> {
        self.physics.chassis_snapshot(id)
    }
    /// All current physical chassis, without copying match or trajectory state.
    pub fn chassis_snapshots(&self) -> Vec<ChassisSnapshot> {
        self.physics.chassis_snapshots()
    }
    /// Current chassis centres in world FLU metres, in id order, without snapshots.
    pub fn chassis_positions_m(&self) -> impl Iterator<Item = [f64; 3]> + '_ {
        self.physics.chassis_positions_m()
    }
    /// Shared presentation compatibility view; native towers live in `rmuc_2027`.
    pub fn outposts(&self) -> &[OutpostSnapshot] {
        &[]
    }
    /// Ticks elapsed since the field was built.
    pub fn tick(&self) -> u64 {
        self.tick
    }
    /// Immutable duration of one physical and rule integration step.
    pub fn step_duration(&self) -> StepDuration {
        self.physics.step_duration()
    }
    /// `tick` times the instance step duration: never the host clock.
    pub fn time_ns(&self) -> u64 {
        self.tick * self.step_duration().as_ns()
    }
    /// Shared presentation compatibility view; native Runes live in `rmuc_2027`.
    pub fn runes(&self) -> &[RuneSnapshot] {
        &[]
    }
    /// Shared compatibility view; always absent. Read `rmuc_2027` for live rules.
    pub fn referee(&self) -> Option<&RefereeSnapshot> {
        None
    }
    /// Supply the host's random shared rotor direction for the next 2027 round.
    /// The selected sign is carried into the match checkpoint at Start.
    pub fn set_rmuc_2027_rotor_direction(&mut self, direction: i8) {
        if let Some(state) = self.authority.rmuc_mut() {
            state.config.rotor_direction = if direction < 0 { -1 } else { 1 };
        }
    }
    /// Host-provided random target/motion seed for the next native Rune round.
    /// The generator is persisted in checkpoints and never rerolled by restore.
    pub fn set_rmuc_2027_rune_seed(&mut self, seed: u64) {
        if let Some(state) = self.authority.rmuc_mut() {
            for (i, rune) in state.game.runes.iter_mut().enumerate() {
                rune.seed = seed.wrapping_add(i as u64);
            }
        }
    }
    /// Apply an operator command to native authority at the current field time.
    /// Bare physics worlds reject rule commands.
    ///
    /// ```
    /// use rm_sim_world::{Field, FieldConfig, RefereeCommand};
    /// let mut field = Field::new(&FieldConfig::default()).unwrap();
    /// assert!(field.referee_command(RefereeCommand::StartMatch).is_err());
    /// ```
    pub fn referee_command(&mut self, command: RefereeCommand) -> Result<(), FieldError> {
        if self.authority.rmul().is_some() {
            return self.rmul_command(command);
        }
        if matches!(&command, RefereeCommand::ResetMatch) && self.authority.rmuc().is_some() {
            self.reset_supply_units()?;
        }
        if let Some(state) = self.authority.rmuc_mut() {
            use rmuc_2027::match_rules::{Match, Phase};
            match command {
                RefereeCommand::StartMatch => {
                    if state.game.phase != Phase::Idle {
                        return Err(FieldError::Referee(
                            "reset the 2027 match before starting again",
                        ));
                    }
                    state.game.start(state.config.rotor_direction);
                }
                RefereeCommand::ResetMatch => {
                    self.assemblies.clear();
                    let robots = state.game.robots.clone();
                    state.game = Match::new(state.config.rotor_direction);
                    for r in robots {
                        state.game.add_robot(r.id, r.team, r.role);
                    }
                }
                RefereeCommand::SelectReinforcement { team, center_um } => state
                    .game
                    .select_reinforcement(referee::game_team(team), center_um)
                    .map_err(FieldError::Referee)?,
                RefereeCommand::StartAssembly { team, difficulty } => state
                    .game
                    .start_assembly(referee::game_team(team), difficulty)
                    .map_err(FieldError::Referee)?,
                RefereeCommand::FinishAssembly { team } => state
                    .game
                    .finish_assembly(referee::game_team(team))
                    .map_err(FieldError::Referee)?,
                RefereeCommand::CancelAssembly { team } => state
                    .game
                    .cancel_assembly(referee::game_team(team))
                    .map_err(FieldError::Referee)?,
                RefereeCommand::ActivateRune { team } => {
                    if state.config.runes.len() != 2 {
                        return Err(FieldError::Referee(
                            "2027 Rune source geometry is not bound",
                        ));
                    }
                    state
                        .game
                        .activate_rune(referee::game_team(team))
                        .map_err(FieldError::Referee)?;
                }
                RefereeCommand::FinishMatch => {
                    if !matches!(state.game.phase, Phase::Countdown | Phase::Running) {
                        return Err(FieldError::Referee("no active 2027 round to finish"));
                    }
                    state.game.finish();
                }
                RefereeCommand::SetBaseHp {
                    team,
                    hp,
                    shield_hp,
                } => {
                    if shield_hp != 0 || hp > rmuc_2027::match_rules::BASE_HP {
                        return Err(FieldError::Referee(
                            "2027 bases have at most 10000 HP and no virtual shield",
                        ));
                    }
                    state.game.set_base_hp(referee::game_team(team), hp)
                }
                RefereeCommand::SetOutpostHp { outpost, hp } => {
                    let side = *Team::BOTH
                        .get(outpost as usize)
                        .ok_or(FieldError::Referee("invalid 2027 outpost"))?;
                    if hp > 1000 || (hp > 0 && state.game.outpost_hp[side.index()] == 0) {
                        return Err(FieldError::Referee(
                            "2027 outpost rebuild requires a control transfer; HP maximum is 1000",
                        ));
                    }
                    state.game.set_outpost_hp(referee::game_team(side), hp);
                }
                RefereeCommand::SetBaseOpen { team, open } => {
                    state.game.base_open[team.index()] = open
                }
                RefereeCommand::SetRobotHp { robot, hp } => state
                    .game
                    .set_robot_hp(robot, hp)
                    .map_err(FieldError::Referee)?,
                RefereeCommand::ReviveRobot { robot } => state
                    .game
                    .set_robot_hp(robot, u32::MAX)
                    .map_err(FieldError::Referee)?,
                RefereeCommand::DamageRobot { robot, amount } => {
                    if !state.game.robots.iter().any(|r| r.id == robot) {
                        return Err(FieldError::Referee("unknown 2027 robot"));
                    }
                    state.game.damage_robot(robot, amount, true);
                }
                RefereeCommand::SetGold { team, gold } => state.game.gold[team.index()] = gold,
                RefereeCommand::SetAllowance { robot, allowance } => {
                    if allowance[1] != 0 {
                        return Err(FieldError::Referee("2027 Heavy is unavailable"));
                    }
                    state
                        .game
                        .robots
                        .iter_mut()
                        .find(|r| r.id == robot)
                        .ok_or(FieldError::Referee("unknown 2027 robot"))?
                        .combat
                        .allowance = allowance[0];
                }
                RefereeCommand::SetPerformance {
                    robot,
                    performance: gameplay::Performance::Infantry { chassis, launcher },
                } => state
                    .game
                    .set_performance(robot, chassis, launcher)
                    .map_err(FieldError::Referee)?,
                RefereeCommand::BuyAmmo {
                    robot,
                    caliber,
                    amount,
                } => {
                    if caliber != Caliber::Mm17 {
                        return Err(FieldError::Referee("2027 Heavy is unavailable"));
                    }
                    state
                        .game
                        .buy_ammo(robot, amount)
                        .map_err(FieldError::Referee)?;
                }
                RefereeCommand::SetLocalizationModule { robot, online } => {
                    let r = state
                        .game
                        .robots
                        .iter_mut()
                        .find(|r| r.id == robot)
                        .ok_or(FieldError::Referee("unknown robot"))?;
                    r.combat.power.modules.localization_online = online;
                }
                RefereeCommand::RadarReport {
                    team,
                    robot,
                    position_m,
                } => {
                    if position_m.iter().any(|v| !v.is_finite() || v.abs() > 1000.) {
                        return Err(FieldError::Referee("invalid Radar position"));
                    }
                    let actual = self
                        .physics
                        .chassis_snapshot(robot)
                        .ok_or(FieldError::Referee("unknown Radar target"))?
                        .pose
                        .translation_m;
                    // Compare integer millimetres at exact Table 5-29 boundaries.
                    let reported = position_m.map(|v| (v * 1000.).round() as i32);
                    let actual = [actual[0], actual[1]].map(|v| (v * 1000.).round() as i32);
                    let error2 = (0..2)
                        .map(|i| {
                            let delta = i64::from(reported[i]) - i64::from(actual[i]);
                            delta * delta
                        })
                        .sum::<i64>();
                    use gameplay::rmuc_2027::radar::Quality;
                    let quality = if error2 < 800 * 800 {
                        Quality::Accurate
                    } else if error2 < 1600 * 1600 {
                        Quality::SemiAccurate
                    } else {
                        Quality::Wrong
                    };
                    state
                        .game
                        .radar_report(referee::game_team(team), robot, reported, quality)
                        .map_err(FieldError::Referee)?;
                }
                RefereeCommand::RadarInformation {
                    team,
                    robot,
                    information,
                } => state
                    .game
                    .radar_information(referee::game_team(team), robot, information)
                    .map_err(FieldError::Referee)?,
                RefereeCommand::RadarDouble { team } => state
                    .game
                    .radar_double(referee::game_team(team))
                    .map_err(FieldError::Referee)?,
                RefereeCommand::RadarInterference { team, difficulty } => {
                    if difficulty > 2 {
                        return Err(FieldError::Referee("invalid Radar difficulty"));
                    }
                    state.game.radar.interference[referee::game_team(team).index()] = difficulty;
                }
                RefereeCommand::SetSentryMode { robot, mode } => state
                    .game
                    .set_sentry_mode(robot, mode)
                    .map_err(FieldError::Referee)?,
                RefereeCommand::SentryAction {
                    robot,
                    action,
                    commander,
                } => state
                    .game
                    .sentry_action(robot, action, commander)
                    .map_err(FieldError::Referee)?,
                RefereeCommand::SetRobotModules {
                    robot,
                    main_online,
                    speed_online,
                    offline_armor,
                    supercap_online,
                } => state
                    .game
                    .set_modules(
                        robot,
                        main_online,
                        speed_online,
                        offline_armor,
                        supercap_online,
                    )
                    .map_err(FieldError::Referee)?,
                RefereeCommand::BuyHp { robot } => {
                    state.game.buy_hp(robot).map_err(FieldError::Referee)?
                }
                RefereeCommand::InstantRespawn { robot } => state
                    .game
                    .instant_respawn(robot)
                    .map_err(FieldError::Referee)?,
                _ => {
                    return Err(FieldError::Referee(
                        "command is outside the 2027 control/structure subset",
                    ));
                }
            }
            if state.game.phase == Phase::Running {
                state.game.activations();
            }
            for r in &state.game.robots {
                self.physics
                    .set_chassis_defeated(r.id, r.hp == 0 && state.game.phase != Phase::Idle);
            }
            self.sync_mechanisms();
            return Ok(());
        }
        Err(FieldError::Referee("no native RMUC 2027 match is bound"))
    }
    /// Advance exactly `ticks` instance steps. Stepping
    /// in pieces equals one large step.
    /// Projectiles and chassis are integrated tick by tick. With no active
    /// bodies and no referee, rune state advances directly to the target time.
    ///
    /// ```rust
    /// use rm_sim_world::{Field, FieldConfig, tick_ns};
    ///
    /// let mut whole = Field::new(&FieldConfig::default()).unwrap();
    /// let mut split = Field::new(&FieldConfig::default()).unwrap();
    /// whole.step(1_000).unwrap();
    /// for ticks in [1, 249, 333, 417] {
    ///     split.step(ticks).unwrap();
    /// }
    /// assert_eq!(whole.tick(), 1_000);
    /// assert_eq!(whole.time_ns(), 1_000 * tick_ns());
    /// // Partitioning the ticks does not change the result.
    /// assert_eq!(whole.snapshot().runes, split.snapshot().runes);
    /// ```
    pub fn step(&mut self, ticks: u64) -> Result<(), FieldError> {
        self.step_with_hits(ticks, &mut |_| {})
    }

    /// Step `ticks` instance steps, reporting each
    /// scored contact in order.
    /// The observer runs after scoring and before snapshot retention can remove
    /// the contact. It sees only new contacts, never restored history, and must
    /// not block. No event queue is retained by the field.
    ///
    /// ```
    /// use rm_sim_world::{Field, FieldConfig};
    /// let mut field = Field::new(&FieldConfig::default()).unwrap();
    /// let mut contacts = Vec::new();
    /// field.step_with_hits(2_000, &mut |hit| contacts.push(hit.clone())).unwrap();
    /// assert_eq!(field.tick(), 2_000);
    /// assert!(contacts.is_empty());
    /// ```
    pub fn step_with_hits(
        &mut self,
        ticks: u64,
        observer: &mut dyn FnMut(&ArmorHit),
    ) -> Result<(), FieldError> {
        let end = self
            .tick
            .checked_add(ticks)
            .filter(|tick| tick.checked_mul(self.step_duration().as_ns()).is_some())
            .ok_or(FieldError::TickOverflow)?;
        while self.tick < end {
            let idle = self.physics.is_idle();
            if idle
                && self.authority.rmuc().is_none()
                && self.authority.rmul().is_none()
                && !self.assemblies.iter().any(assembly::Run::active)
            {
                self.tick = end;
                self.sync_mechanisms();
                break;
            }
            let prev_ns = self.time_ns();
            if !idle {
                // Rule commands and scored hits may change a target at this same
                // timestamp, so derive the start frame from authoritative rules.
                self.target_frames
                    .begin(|faces| Self::write_target_faces(self.authority.rmuc(), faces))
                    .map_err(FieldError::Ballistics)?;
            }
            self.tick += 1;
            let next_ns = self.time_ns();
            self.advance_2027(self.step_duration().as_ns());
            self.advance_rmul(self.step_duration().as_ns());
            self.advance_assemblies();
            self.sync_mechanisms();
            if idle {
                continue;
            }
            self.target_frames
                .update(|faces| Self::write_target_faces(self.authority.rmuc(), faces))
                .map_err(FieldError::Ballistics)?;
            let contacts = self
                .physics
                .step(prev_ns, &self.target_frames)
                .map_err(FieldError::Ballistics)?;
            if let Some(state) = self.authority.rmuc_mut() {
                for r in &mut state.game.robots {
                    r.combat.power.actual_mw =
                        (self.physics.chassis_drive_power_w(r.id).unwrap_or(0.) * 1000.)
                            .round()
                            .clamp(0., 10_000_000.) as u32;
                }
            }
            self.sync_energy_units();
            for contact in contacts {
                if self.armor_disabled(contact.target) {
                    continue;
                }
                let hit = self.score(next_ns, contact)?;
                observer(&hit);
                self.hits.push(hit);
            }
            self.apply_collisions(next_ns);
            self.sync_mechanisms();
            self.sync_defeats();
            self.hits
                .retain(|hit| next_ns.saturating_sub(hit.time_ns) < HIT_MEMORY_NS);
        }
        Ok(())
    }
    /// Table 5-2 collision damage for chassis armor modules that struck
    /// something during a running round, at most once per module per 17 mm
    /// detection interval.
    fn apply_collisions(&mut self, time_ns: u64) {
        let collisions = self.physics.take_armor_collisions();
        if let Some(game) = self.authority.rmul_mut() {
            for collision in collisions {
                let target = ArmorTarget::Chassis {
                    chassis: collision.chassis,
                    plate: collision.plate,
                };
                if self.last_detection_ns.get(&target).is_some_and(|last| {
                    time_ns.saturating_sub(*last) < Caliber::Mm17.detection_interval_ns()
                }) {
                    continue;
                }
                self.last_detection_ns.insert(target, time_ns);
                game.collision(collision.chassis);
            }
            return;
        }
        if let Some(state) = self.authority.rmuc_mut() {
            if state.game.phase != rmuc_2027::match_rules::Phase::Running {
                return;
            }
            for collision in collisions {
                let target = ArmorTarget::Chassis {
                    chassis: collision.chassis,
                    plate: collision.plate,
                };
                if self.last_detection_ns.get(&target).is_some_and(|last| {
                    time_ns.saturating_sub(*last) < Caliber::Mm17.detection_interval_ns()
                }) {
                    continue;
                }
                self.last_detection_ns.insert(target, time_ns);
                state.game.collision(collision.chassis);
            }
        }
    }
    /// Buff point footprints this field reports contacts for.
    pub fn zones(&self) -> &[zones::ZoneArea] {
        &[]
    }
    /// Resolve match state only when articulated scenery actually needs it.
    fn sync_mechanisms(&mut self) {
        if self.physics.has_mechanisms() {
            let mut state = referee::mechanism_view(None, self.time_ns());
            if let Some(r) = self.authority.rmuc() {
                state.outpost_angle_rad = Team::BOTH.map(|t| r.angle_rad(t));
                state.rune_angle_rad = r
                    .game
                    .runes
                    .each_ref()
                    .map(|rune| rune.angle_at(r.game.elapsed_ns()));
                state.base_open_fraction = r.game.base_open.map(|v| if v { 1. } else { 0. });
            }
            self.physics.sync_mechanisms(&state);
        }
    }
    /// Whether native authority disables this physical scoring face.
    /// Unknown targets are disabled; physical chassis remain usable in bare worlds.
    pub fn armor_disabled(&self, target: ArmorTarget) -> bool {
        if let Some(state) = self.authority.rmuc() {
            return state.disabled(target);
        }
        match target {
            ArmorTarget::Chassis { chassis, .. } => {
                self.physics.chassis_defeated(chassis).unwrap_or(true)
            }
            _ => true,
        }
    }
    /// Put one chassis back on a captured state, for a client replaying its
    /// own robot against estimated peers. The authoritative field never does
    /// this; nothing else about the field moves.
    pub fn reset_chassis(&mut self, state: &ChassisSnapshot) -> Result<(), FieldError> {
        self.physics
            .reset_chassis(state)
            .map_err(FieldError::Chassis)
    }
}

#[cfg(test)]
mod tests;
