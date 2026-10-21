// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native RoboMaster field and shared physics on an explicit instance-owned clock.
//! No renderer, host clock, transport or background threads. Projectiles fly in
//! the shared `rm-simulator-physics` world. This facade coordinates its contacts and
//! motion with live rules, scoring, referee decisions and complete restore.
#![deny(missing_docs)]

pub mod assembly;
pub mod base;
pub mod energy_unit;
pub use base::{BaseConfig, BaseSnapshot};
pub mod chassis;
pub mod outpost;
pub mod projectile;
pub mod referee;
pub mod rmuc_2027;
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
pub use rm_simulator_physics::control::{DriveControl, WheelTorques};
use rune::RuneError;
pub use rune::{BigRuneMotion, HitOutcome, Rune, RuneKind, RuneSnapshot, RuneState};
use serde::{Deserialize, Serialize};

/// Native gameplay authority and shared serialized records.
pub use rm_simulator_gameplay as gameplay;
/// Season identities and admission checks shared with standalone gameplay.
pub use rm_simulator_gameplay::season;
pub use rm_simulator_physics::{Pose, StepDuration, tick_ns};

/// One armor module pose on a rotating mechanism, keyed by face index.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ArmorSnapshot {
    /// Face index in the mechanism's own order.
    pub id: u32,
    /// Face centre in world FLU metres; +x is the outward scoring normal.
    pub pose: Pose,
}

/// One Power Rune in the layout: which mode it starts in and where its hub sits.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RuneConfig {
    /// Rune mode. The referee's three-minute stage change converts every rune
    /// to Big (section 5.5.2).
    pub kind: RuneKind,
    /// Face centre; local +x points into the wheel, away from the shooter.
    /// The scoring front faces local -x; local +z is up.
    pub hub_pose: Pose,
    /// Use the measured 698.5 mm CAD target orbit instead of the nominal 700 mm.
    pub cad_orbit: bool,
}
impl Default for RuneConfig {
    fn default() -> Self {
        Self {
            kind: RuneKind::Small,
            hub_pose: Pose::at([6.0, 0.0, 1.6]),
            cad_orbit: false,
        }
    }
}

/// One outpost in the layout: where the tower base stands, how fast it turns
/// and, when the asset supplied one, its verified rotor pivot.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct OutpostConfig {
    /// Optional verified pivot in CAD glTF metres; legacy configurations use the fitted pivot.
    #[serde(default)]
    pub pivot_cad_m: Option<[f64; 3]>,
    /// Tower base pose: translation is the base on the floor, rotation maps the
    /// tower's local FLU frame (forward = CAD +x) into the world.
    pub origin: Pose,
    /// Rotor speed counter-clockwise about the tower's up axis. Section 5.5.1
    /// gives 0.8π rad/s; the constructor rejects magnitudes above 10 rad/s.
    pub speed_rad_s: f64,
}

/// A driven chassis, whose team it plays for and where it starts.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChassisPlacement {
    /// Preset dimensions, suspension, armor geometry and drivetrain budget.
    pub config: ChassisConfig,
    /// Body centre pose; the wheels find the ground on the first tick.
    pub spawn: Pose,
    /// Team whose referee record, base and buffs this chassis belongs to.
    pub team: Team,
    /// Robot class the referee records for it; a placement that names none
    /// is an infantry, which is what older serialized layouts hold.
    #[serde(default)]
    pub kind: RobotKind,
    /// Section 5.4.2 performance type the robot selected; `None` takes the
    /// class default (a long-range Hero, an HP-focused cooling-focused Infantry).
    #[serde(default)]
    pub performance: Option<rm_simulator_gameplay::Performance>,
}

/// Static field layout. Everything here is placed once; motion comes from the clock.
///
/// ```rust
/// use rm_simulator_world::FieldConfig;
///
/// // Defaults provide bare physics; native bindings opt into match rules.
/// let config = FieldConfig::default();
/// assert!(config.runes.is_empty());
/// assert!(config.outposts.is_empty());
/// assert_eq!(config.floor_height_m, 0.0);
/// assert!(config.bases.is_empty());
/// ```
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FieldConfig {
    /// Immutable physical/rules step. Custom rates are API-only; server sessions
    /// accept only the default until protocol prediction/planning supports them.
    #[serde(default)]
    pub step_duration: StepDuration,
    /// Native RMUC 2027 rules and source geometry; absent for bare physics.
    #[serde(default)]
    pub rmuc_2027: Option<rmuc_2027::Config>,
    /// Historical layout record; must be empty. Native bases use `rmuc_2027`.
    #[serde(default)]
    pub bases: Vec<BaseConfig>,
    /// Historical layout record; must be empty. Native Runes use `rmuc_2027`.
    pub runes: Vec<RuneConfig>,
    /// Historical layout record; must be empty. Native towers use `rmuc_2027`.
    pub outposts: Vec<OutpostConfig>,
    /// Height of the flat floor plane. Zero is the playing floor; a field
    /// that supplies its own ground mesh lowers it to a catch surface.
    pub floor_height_m: f64,
    /// Chassis present from the start, in id order; more can join and
    /// leave with [`Field::add_chassis`] and [`Field::remove_chassis`].
    #[serde(default)]
    pub chassis: Vec<ChassisPlacement>,
    /// Historical configuration record; must be `None`.
    #[serde(default)]
    pub referee: Option<RefereeConfig>,
    /// Ball restitution, hard flight limit and low-speed retirement rule. The
    /// default is the simulator's control behaviour. A host and every client
    /// predicting for it must use the same value, so it travels in the
    /// snapshot's [`FieldRestore`] as well.
    #[serde(default)]
    pub projectile_policy: projectile::ProjectilePolicy,
    /// Historical zone records; must be empty. Native zones use `rmuc_2027`.
    #[serde(default)]
    pub zones: Vec<zones::ZoneArea>,
}

impl Default for FieldConfig {
    fn default() -> Self {
        Self {
            step_duration: StepDuration::default(),
            rmuc_2027: None,
            zones: Vec::new(),
            bases: Vec::new(),
            floor_height_m: 0.0,
            chassis: Vec::new(),
            referee: None,
            projectile_policy: projectile::ProjectilePolicy::default(),
            runes: Vec::new(),
            outposts: Vec::new(),
        }
    }
}

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

/// The rules' own bookkeeping, beyond what an observer needs: rune activation
/// progress and its seeded stream, outpost rotor stops, the referee's schedule
/// and configuration, and the physics identity counters. [`Field::restore`]
/// needs it; a snapshot that lost it can still be drawn.
///
/// What it deliberately cannot carry is solver state. Rapier's contact
/// manifolds, warm starts and each ball's touched-armor set are rebuilt from
/// the restored bodies, so a restored field's first ticks differ from the
/// original's by the solver's warm-start residual, and a ball already resting
/// against armor registers that touch once more.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FieldRestore {
    /// Exact simulation step, preserved when restoring the authoritative clock.
    #[serde(default)]
    pub step_duration: StepDuration,
    /// Restorable automatic assembly jobs and validated remaining trajectories.
    #[serde(default)]
    pub assemblies: Vec<assembly::Run>,
    /// Persistent articulated arm authority for deployed source Heavy robots.
    #[serde(default)]
    pub arms: Vec<assembly::ArmState>,
    /// Persistent physical Energy Units, including loose and socket-seated objects.
    #[serde(default)]
    pub energy_units: Vec<energy_unit::EnergyUnit>,
    /// Complete RMUC 2027 authority and geometry, absent for bare physics.
    pub rmuc_2027: Option<rmuc_2027::State>,
    /// Next id [`Field::add_chassis`] would hand out; ids are never reused.
    pub next_chassis_id: u32,
    /// Per-module time of the last detected strike, oldest kept, in target
    /// order, so the detection intervals of section 5.1.1 survive a restore.
    pub last_detection_ns: Vec<(ArmorTarget, u64)>,
    /// The projectile restitution, flight limit and retirement rule the field
    /// runs, so a restored field spends and retires balls exactly as the field
    /// it was captured from does.
    pub projectile_policy: projectile::ProjectilePolicy,
}

/// One authoritative tick of the whole field, as an observer or a peer
/// receives it. `Field::restore` rebuilds a field from one, so it carries the
/// rules' hidden state as well as what a viewer draws.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FieldSnapshot {
    /// Native 2027 view, retained even when a drawing-only decoder drops restore.
    /// The checkpoint codec derives this from the matching authority record.
    pub rmuc_2027: Option<rmuc_2027::State>,
    /// Every base's fitted geometry, HP and shield.
    pub bases: Vec<BaseSnapshot>,
    /// Ticks elapsed since the field was built.
    pub tick: u64,
    /// `tick` times the restore's step duration, the field clock in nanoseconds.
    pub time_ns: u64,
    /// One frame per rune, in referee index order.
    pub runes: Vec<RuneSnapshot>,
    /// One frame per outpost, in referee index order.
    pub outposts: Vec<OutpostSnapshot>,
    /// Balls still in flight at this tick.
    pub projectiles: Vec<ProjectileSnapshot>,
    /// Every chassis on the field, in id order.
    pub chassis: Vec<ChassisSnapshot>,
    /// Armor contacts from the last second, oldest first.
    pub hits: Vec<ArmorHit>,
    /// Balls launched since the field was built.
    pub shots_fired: u64,
    /// Contacts that passed every detection condition, rejected ones excluded.
    pub hits_detected: u64,
    /// The match state, when the field has a referee.
    pub referee: Option<RefereeSnapshot>,
    /// Hidden rule state, for [`Field::restore`]. Present on every snapshot a
    /// field takes; a decoder that drops it gives up restoring, not drawing.
    pub restore: Option<FieldRestore>,
}

/// The complete field: one physics world holding projectiles and chassis, with
/// the runes, outposts, bases and optional referee on top. Reading a snapshot
/// never advances time.
///
/// ```rust
/// use rm_simulator_world::{Field, FieldConfig, tick_ns};
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
    energy_units: Vec<energy_unit::EnergyUnit>,
    arms: Vec<assembly::ArmState>,
    assemblies: Vec<assembly::Run>,
    rmuc_2027: Option<rmuc_2027::State>,
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
    /// use rm_simulator_world::{Field, FieldConfig};
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
            rmuc_2027,
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

    /// Launch a projectile in the shared physical world, with an optional shooter.
    /// Native matches enforce phase, HP, launcher locks, heat and allowance.
    /// Returns the projectile identity.
    ///
    /// ```
    /// use rm_simulator_world::{Caliber, Field, FieldConfig, Pose, Shot};
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
        if let Some(state) = &self.rmuc_2027 {
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
        let id = self
            .physics
            .fire(self.time_ns(), muzzle, shot, shooter)
            .map_err(FieldError::Shot)?;
        if let (Some(state), Some(id)) = (&mut self.rmuc_2027, shooter) {
            state.game.record_launch(id, shot.speed_m_s);
        }
        Ok(id)
    }
    /// Buy one pilot ammunition unit during a round. Legacy 2026 uses ten
    /// 17 mm rounds or one 42 mm round; native 2027 chooses ten zone rounds or
    /// 100 remote rounds with six-second delivery (Tables 5-7–5-9).
    pub fn buy_ammo(&mut self, chassis: u32, caliber: Caliber) -> Result<(), FieldError> {
        if let Some(state) = &mut self.rmuc_2027 {
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
    /// use rm_simulator_world::{
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
        if self.rmuc_2027.is_some()
            && !matches!(
                placement.kind,
                RobotKind::Infantry | RobotKind::Sentry | RobotKind::Engineer
            )
        {
            return Err(FieldError::Referee("unsupported 2027 ground role"));
        }
        if self.rmuc_2027.is_some()
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
        let id = self
            .physics
            .add_chassis(placement.team, placement.config.clone(), placement.spawn)
            .map_err(FieldError::Chassis)?;
        if let Some(state) = &mut self.rmuc_2027 {
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
        Ok(id)
    }
    /// Take a chassis off the field, with its robot record.
    pub fn remove_chassis(&mut self, id: u32) -> Result<(), FieldError> {
        self.cancel_assembly(id, "Assembly owner disconnected");
        self.release_energy_units(id);
        self.arms.retain(|a| a.owner != id);
        self.physics
            .remove_chassis(id)
            .map_err(FieldError::Chassis)?;
        if let Some(state) = &mut self.rmuc_2027 {
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
        if let Some(state) = &self.rmuc_2027 {
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
    /// use rm_simulator_world::{
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
    /// use rm_simulator_world::{Field, FieldConfig, ChassisPlacement, ChassisConfig,
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
    /// Convert an eligible native Heavy to a source Hero chassis in place.
    /// Simulator approximation: no workbench installation or confirmation shot.
    pub fn convert_heavy_to_hero(&mut self, id: u32) -> Result<(), FieldError> {
        let state = self
            .rmuc_2027
            .as_ref()
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
        let state = self.rmuc_2027.as_mut().unwrap();
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
        if let Some(state) = &mut self.rmuc_2027 {
            state.config.rotor_direction = if direction < 0 { -1 } else { 1 };
        }
    }
    /// Host-provided random target/motion seed for the next native Rune round.
    /// The generator is persisted in checkpoints and never rerolled by restore.
    pub fn set_rmuc_2027_rune_seed(&mut self, seed: u64) {
        if let Some(state) = &mut self.rmuc_2027 {
            for (i, rune) in state.game.runes.iter_mut().enumerate() {
                rune.seed = seed.wrapping_add(i as u64);
            }
        }
    }
    /// Apply an operator command to native authority at the current field time.
    /// Bare physics worlds reject rule commands.
    ///
    /// ```
    /// use rm_simulator_world::{Field, FieldConfig, RefereeCommand};
    /// let mut field = Field::new(&FieldConfig::default()).unwrap();
    /// assert!(field.referee_command(RefereeCommand::StartMatch).is_err());
    /// ```
    pub fn referee_command(&mut self, command: RefereeCommand) -> Result<(), FieldError> {
        if let Some(state) = &mut self.rmuc_2027 {
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
    /// use rm_simulator_world::{Field, FieldConfig, tick_ns};
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
    /// use rm_simulator_world::{Field, FieldConfig};
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
                && self.rmuc_2027.is_none()
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
                    .begin(|faces| Self::write_target_faces(self.rmuc_2027.as_ref(), faces))
                    .map_err(FieldError::Ballistics)?;
            }
            self.tick += 1;
            let next_ns = self.time_ns();
            self.advance_2027(self.step_duration().as_ns());
            self.advance_assemblies();
            self.sync_mechanisms();
            if idle {
                continue;
            }
            self.target_frames
                .update(|faces| Self::write_target_faces(self.rmuc_2027.as_ref(), faces))
                .map_err(FieldError::Ballistics)?;
            let contacts = self
                .physics
                .step(prev_ns, &self.target_frames)
                .map_err(FieldError::Ballistics)?;
            if let Some(state) = &mut self.rmuc_2027 {
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
        if let Some(state) = &mut self.rmuc_2027 {
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
            if let Some(r) = &self.rmuc_2027 {
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
        if let Some(state) = &self.rmuc_2027 {
            return state.disabled(target);
        }
        match target {
            ArmorTarget::Chassis { chassis, .. } => {
                self.physics.chassis_defeated(chassis).unwrap_or(true)
            }
            _ => true,
        }
    }
    /// A frame of everything an observer needs at the current tick: bases,
    /// runes, outposts, projectiles, chassis, recent hits and the referee.
    /// Taking one never advances the clock, and it always carries the hidden
    /// rule state a `restore` needs.
    ///
    /// ```rust
    /// use rm_simulator_world::{Field, FieldConfig, tick_ns};
    ///
    /// let mut field = Field::new(&FieldConfig::default()).unwrap();
    /// field.step(250).unwrap();
    /// let snapshot = field.snapshot();
    /// assert_eq!(snapshot.tick, 250);
    /// assert_eq!(snapshot.time_ns, 250 * tick_ns());
    /// assert!(snapshot.runes.is_empty());
    /// assert!(snapshot.restore.is_some());
    /// ```
    pub fn snapshot(&self) -> FieldSnapshot {
        let time_ns = self.time_ns();
        FieldSnapshot {
            rmuc_2027: self.rmuc_2027.clone(),
            bases: Vec::new(),
            tick: self.tick,
            time_ns,
            runes: Vec::new(),
            outposts: Vec::new(),
            projectiles: self.physics.snapshot(),
            chassis: self.physics.chassis_snapshots(),
            hits: self.hits.clone(),
            shots_fired: self.physics.launched(),
            hits_detected: self.hits_detected,
            referee: None,
            restore: Some(FieldRestore {
                step_duration: self.step_duration(),
                assemblies: self.assemblies.clone(),
                arms: self.arms.clone(),
                energy_units: self.energy_units(),
                rmuc_2027: self.rmuc_2027.clone(),
                next_chassis_id: self.physics.next_chassis_id(),
                projectile_policy: self.physics.projectile_policy(),
                last_detection_ns: {
                    // A hash map has no order; sort so equal fields snapshot equal.
                    let mut detections: Vec<_> = self
                        .last_detection_ns
                        .iter()
                        .map(|(target, time_ns)| (*target, *time_ns))
                        .collect();
                    detections.sort_unstable();
                    detections
                },
            }),
        }
    }
    /// Rebuild the field the snapshot was taken from, sharing `geometry`'s
    /// collision shapes instead of reloading any CAD, and standing at the
    /// snapshot's own tick. Stepping the result applies the same rules as the
    /// original: this is one field, not a reduced presentation world.
    ///
    /// What is restored: every rune, outpost and the referee exactly (from
    /// [`FieldRestore`]); every chassis with its pose, velocities, held aim,
    /// command, defeat and wheel spin; every projectile with its identity,
    /// age, position, velocity and spin; the armor detection intervals, the
    /// recent hits and the shot and detection counters.
    ///
    /// What cannot be restored is solver state, as [`FieldRestore`] records:
    /// contact manifolds and warm starts are rebuilt, and a projectile's set
    /// of already-touched armor colliders starts empty. Wheel contacts and
    /// tyre targets are recomputed on the first tick, so they need no
    /// checkpoint.
    ///
    /// ```rust
    /// use rm_simulator_world::{Field, FieldConfig};
    ///
    /// let mut field = Field::new(&FieldConfig::default()).unwrap();
    /// field.step(500).unwrap();
    /// let checkpoint = field.snapshot();
    /// let geometry = field.static_geometry_snapshot();
    ///
    /// let mut restored = Field::restore(&checkpoint, &geometry, 0.0).unwrap();
    /// assert_eq!(restored.tick(), checkpoint.tick);
    /// // The restored field keeps stepping under the same rules as the original.
    /// field.step(10).unwrap();
    /// restored.step(10).unwrap();
    /// assert_eq!(restored.snapshot().runes, field.snapshot().runes);
    /// ```
    pub fn restore(
        snapshot: &FieldSnapshot,
        geometry: &StaticGeometry,
        floor_height_m: f64,
    ) -> Result<Self, FieldError> {
        // The explicit native authority chooses restore admission; never infer
        // a season from a team, HP value or CAD position.
        if snapshot.referee.is_some()
            || !snapshot.runes.is_empty()
            || !snapshot.outposts.is_empty()
            || !snapshot.bases.is_empty()
        {
            return Err(FieldError::Restore(
                "2026 runtime/checkpoints removed; use rm2026-final",
            ));
        }
        let incoming_2027 = snapshot.restore.as_ref().and_then(|r| r.rmuc_2027.as_ref());
        if let Some(state) = incoming_2027 {
            state.config.validate().map_err(FieldError::Restore)?;
            state.game.validate().map_err(FieldError::Restore)?;
            if !snapshot.bases.is_empty()
                || !snapshot.outposts.is_empty()
                || !snapshot.runes.is_empty()
                || snapshot.referee.is_some()
            {
                return Err(FieldError::Restore("mixed season checkpoint"));
            }
        }
        if snapshot.rmuc_2027.as_ref() != incoming_2027 {
            return Err(FieldError::Restore("2027 view differs from its authority"));
        }
        let rules = snapshot
            .restore
            .as_ref()
            .ok_or(FieldError::Restore("the snapshot carries no rule state"))?;
        if rules.arms.len() > 64 {
            return Err(FieldError::Restore("too many arm owners"));
        }
        let mut arm_ids = std::collections::BTreeSet::new();
        for arm in &rules.arms {
            arm.validate().map_err(FieldError::Restore)?;
            if !arm_ids.insert(arm.owner) || !snapshot.chassis.iter().any(|c| c.id == arm.owner) {
                return Err(FieldError::Restore("missing or duplicate arm owner"));
            }
        }
        if rules.assemblies.len() > 2 {
            return Err(FieldError::Restore("too many assembly jobs"));
        }
        if rules.assemblies.len() == 2
            && (rules.assemblies[0].owner == rules.assemblies[1].owner
                || rules.assemblies[0].team == rules.assemblies[1].team)
        {
            return Err(FieldError::Restore("duplicate assembly owner or station"));
        }
        for run in &rules.assemblies {
            run.validate_with_step_duration(rules.step_duration)
                .map_err(FieldError::Restore)?;
            if run.active()
                && !snapshot.chassis.iter().any(|c| {
                    c.id == run.owner
                        && c.team == run.team
                        && c.placement_revision == run.placement_revision
                })
            {
                return Err(FieldError::Restore("missing assembly owner"));
            }
        }
        if !floor_height_m.is_finite() {
            return Err(FieldError::Restore("the floor height must be finite"));
        }
        if snapshot.tick.checked_mul(rules.step_duration.as_ns()) != Some(snapshot.time_ns) {
            return Err(FieldError::Restore("the snapshot tick and time disagree"));
        }
        for base in &snapshot.bases {
            base.config.validate().map_err(FieldError::Restore)?;
            if base.hp > base::INITIAL_HP || base.shield_hp > base::INITIAL_HP {
                return Err(FieldError::Restore("base HP or shield out of range"));
            }
        }
        let time_ns = snapshot.time_ns;
        let mut faces = Vec::new();
        Self::write_target_faces(rules.rmuc_2027.as_ref(), &mut faces);
        let mut physics =
            WorldPhysics::with_step_duration(&faces, floor_height_m, rules.step_duration);
        physics
            .set_projectile_policy(rules.projectile_policy)
            .map_err(FieldError::Restore)?;
        physics.insert_geometry(geometry);
        let mut field = Self {
            assemblies: rules.assemblies.clone(),
            arms: rules.arms.clone(),
            energy_units: rules.energy_units.clone(),
            rmuc_2027: rules.rmuc_2027.clone(),
            floor_height_m,
            tick: snapshot.tick,
            physics,
            target_frames: TargetFrames::new(faces),
            hits: snapshot.hits.clone(),
            hits_detected: snapshot.hits_detected,
            last_detection_ns: rules.last_detection_ns.iter().copied().collect(),
        };
        for chassis in &snapshot.chassis {
            field
                .physics
                .restore_chassis(chassis)
                .map_err(FieldError::Chassis)?;
        }
        field.restore_energy_units()?;
        for projectile in &snapshot.projectiles {
            field
                .physics
                .restore_projectile(projectile)
                .map_err(FieldError::Shot)?;
        }
        field
            .physics
            .restore_identity(rules.next_chassis_id, snapshot.shots_fired);
        field.physics.mark_synced(time_ns);
        field.sync_mechanisms();
        field.sync_defeats();
        Ok(field)
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
mod tests {
    #[test]
    fn removed_runtime_configs_are_rejected_even_without_a_cad_host() {
        use super::*;
        let variants = [
            FieldConfig {
                referee: Some(RefereeConfig::default()),
                ..Default::default()
            },
            FieldConfig {
                runes: vec![RuneConfig::default()],
                ..Default::default()
            },
            FieldConfig {
                outposts: vec![OutpostConfig {
                    origin: Pose::default(),
                    pivot_cad_m: None,
                    speed_rad_s: 0.,
                }],
                ..Default::default()
            },
        ];
        for config in variants {
            assert!(
                Field::new(&config)
                    .err()
                    .unwrap()
                    .to_string()
                    .contains("rm2026-final")
            );
        }
        assert!(Field::new(&FieldConfig::default()).is_ok());
    }
    use super::*;
    #[test]
    fn instance_steps_preserve_clocks_partitioning_and_restore() {
        for duration in [
            StepDuration::default(),
            StepDuration::from_hz(500).unwrap(),
            StepDuration::HZ_1000,
            StepDuration::from_hz(2000).unwrap(),
        ] {
            let config = FieldConfig {
                step_duration: duration,
                ..Default::default()
            };
            let mut whole = Field::new(&config).unwrap();
            let mut parts = Field::new(&config).unwrap();
            whole.step(1000).unwrap();
            parts.step(213).unwrap();
            parts.step(787).unwrap();
            assert_eq!(whole.snapshot(), parts.snapshot());
            assert_eq!(whole.time_ns(), 1000 * duration.as_ns());
            let snapshot = whole.snapshot();
            assert_eq!(snapshot.restore.as_ref().unwrap().step_duration, duration);
            let mut restored =
                Field::restore(&snapshot, &whole.static_geometry_snapshot(), 0.).unwrap();
            assert_eq!(restored.step_duration(), duration);
            restored.step(37).unwrap();
            whole.step(37).unwrap();
            assert_eq!(restored.snapshot(), whole.snapshot());
            let mut wrong = snapshot;
            wrong.restore.as_mut().unwrap().step_duration = if duration == StepDuration::HZ_1000 {
                StepDuration::HZ_128
            } else {
                StepDuration::HZ_1000
            };
            assert!(Field::restore(&wrong, &whole.static_geometry_snapshot(), 0.).is_err());
        }
    }
    #[test]
    fn thousand_hz_projectile_partition_and_restore_integrate_instance_dt() {
        let config = FieldConfig {
            step_duration: StepDuration::HZ_1000,
            ..Default::default()
        };
        let mut whole = Field::new(&config).unwrap();
        let mut parts = Field::new(&config).unwrap();
        for field in [&mut whole, &mut parts] {
            field
                .fire(Pose::at([0., 0., 10.]), Shot::at_limit(Caliber::Mm17), None)
                .unwrap();
        }
        whole.step(100).unwrap();
        parts.step(31).unwrap();
        parts.step(69).unwrap();
        assert_eq!(whole.snapshot(), parts.snapshot());
        assert_eq!(whole.time_ns(), 100_000_000);
        let ball = &whole.snapshot().projectiles[0];
        // Horizontal quadratic drag has a closed-form solution; vertical speed
        // contributes only a small correction over this 100 ms flight.
        let k = Caliber::Mm17.drag_factor_per_m();
        let expected_x = (1. + k * 25. * 0.1).ln() / k;
        assert!((ball.position_m[0] - expected_x).abs() < 0.002);

        assert!((ball.velocity_m_s[2] + 0.980665).abs() < 0.04);
        let mut restored =
            Field::restore(&whole.snapshot(), &whole.static_geometry_snapshot(), 0.).unwrap();
        whole.step(100).unwrap();
        restored.step(43).unwrap();
        restored.step(57).unwrap();
        assert_eq!(whole.time_ns(), restored.time_ns());
        for (a, b) in whole.snapshot().projectiles[0]
            .position_m
            .into_iter()
            .zip(restored.snapshot().projectiles[0].position_m)
        {
            assert!((a - b).abs() < 1e-8);
        }
    }
    /// Convert default-rate test waits to ticks, rounding up.
    fn ticks(ns: u64) -> u64 {
        ns.div_ceil(tick_ns())
    }
    /// A referee'd field with two pilots, a rune and both outposts, driven
    /// and shot at long enough for every rule to have moved.
    fn busy_field(shoot: bool) -> (Field, u32) {
        let spawn = ChassisConfig::default().rest_height_m();
        let config = FieldConfig {
            chassis: vec![
                ChassisPlacement {
                    config: ChassisConfig::default(),
                    team: Team::Red,
                    kind: RobotKind::Infantry,
                    // A heat limit the eight quick shots below stay under.
                    performance: Some(rm_simulator_gameplay::Performance::Fixed(
                        rm_simulator_gameplay::Stats {
                            max_hp: 200,
                            chassis_power_w: 60,
                            heat_limit: 1_000,
                            cooling_per_s: 20,
                        },
                    )),
                    spawn: Pose::at([0.0, 0.0, spawn]),
                },
                ChassisPlacement {
                    config: ChassisConfig::default(),
                    team: Team::Blue,
                    kind: RobotKind::Infantry,
                    performance: None,
                    // Keep the second pilot clear of spent Rune rounds. A
                    // later robot/ball impact amplifies unretained contact-solver
                    // residuals and does not test checkpoint completeness.
                    spawn: Pose::at([1.5, -2.0, spawn]),
                },
            ],
            runes: vec![],
            outposts: vec![],
            ..FieldConfig::default()
        };
        let mut field = Field::new(&config).unwrap();
        field.step(ticks(referee::COUNTDOWN_NS)).unwrap();
        let drive = ChassisCommand {
            forward_m_s: 1.2,
            yaw_rate_rad_s: 0.4,
            aim_yaw_rad: 0.3,
            ..Default::default()
        };
        field.command_chassis(0, drive).unwrap();
        for _ in 0..8 {
            field.step(ticks(60_000_000)).unwrap();
            if !shoot {
                continue;
            }
            let muzzle = field.chassis_muzzle_pose(0).unwrap();
            field
                .fire(muzzle, Shot::at_limit(Caliber::Mm17), Some(0))
                .unwrap();
        }
        field.step(ticks(30_000_000)).unwrap();
        (field, 0)
    }
    fn chassis_gap(a: &FieldSnapshot, b: &FieldSnapshot) -> f64 {
        a.chassis
            .iter()
            .zip(&b.chassis)
            .flat_map(|(a, b)| {
                assert_eq!(a.id, b.id);
                a.pose
                    .translation_m
                    .into_iter()
                    .zip(b.pose.translation_m)
                    .map(|(a, b)| (a - b).abs())
            })
            .fold(0.0_f64, f64::max)
    }
    fn projectile_gap(a: &FieldSnapshot, b: &FieldSnapshot) -> f64 {
        assert_eq!(
            a.projectiles.iter().map(|p| p.id).collect::<Vec<_>>(),
            b.projectiles.iter().map(|p| p.id).collect::<Vec<_>>(),
            "a restored field must keep every ball's identity"
        );
        a.projectiles
            .iter()
            .zip(&b.projectiles)
            .flat_map(|(a, b)| {
                a.position_m
                    .into_iter()
                    .zip(b.position_m)
                    .map(|(a, b)| (a - b).abs())
            })
            .fold(0.0_f64, f64::max)
    }
    /// Replaying a restored field is exact while nothing rests in a contact:
    /// the chassis stands on wheel ray casts, not on the contact solver.

    #[test]
    fn a_restored_field_without_contacts_replays_exactly() {
        let (mut original, pilot) = busy_field(false);
        let geometry = original.static_geometry_snapshot();
        let floor = original.floor_height_m();
        let mut restored = Field::restore(&original.snapshot(), &geometry, floor).unwrap();
        for round in 0..10 {
            let command = ChassisCommand {
                folded: false,
                jump: false,
                balance_control: 100,
                forward_m_s: if round % 3 == 0 { 1.5 } else { -0.5 },
                left_m_s: 0.4,
                yaw_rate_rad_s: 0.6,
                aim_yaw_rad: 0.2 * round as f64,
                aim_pitch_rad: 0.05,
            };
            original.command_chassis(pilot, command).unwrap();
            restored.command_chassis(pilot, command).unwrap();
            original.step(ticks(20_000_000)).unwrap();
            restored.step(ticks(20_000_000)).unwrap();
            assert_eq!(original.snapshot(), restored.snapshot());
        }
    }
    #[test]
    fn a_restored_field_keeps_stepping_like_the_one_it_came_from() {
        let (mut original, pilot) = busy_field(true);
        let geometry = original.static_geometry_snapshot();
        let floor = original.floor_height_m();
        let checkpoint = original.snapshot();
        let mut restored = Field::restore(&checkpoint, &geometry, floor).unwrap();
        // Restoring is a pure read of the checkpoint, in both directions.
        assert_eq!(restored.snapshot(), checkpoint);
        assert_eq!(
            Field::restore(&restored.snapshot(), &geometry, floor)
                .unwrap()
                .snapshot(),
            checkpoint,
            "restore(snapshot(restore(s))) must stand exactly where s did"
        );
        let mut worst_chassis = 0.0_f64;
        let mut worst_projectile = 0.0_f64;
        for round in 0..10 {
            let command = ChassisCommand {
                folded: false,
                jump: false,
                balance_control: 100,
                forward_m_s: if round % 3 == 0 { 1.5 } else { -0.5 },
                left_m_s: 0.4,
                yaw_rate_rad_s: 0.6,
                aim_yaw_rad: 0.2 * round as f64,
                aim_pitch_rad: 0.05,
            };
            original.command_chassis(pilot, command).unwrap();
            restored.command_chassis(pilot, command).unwrap();
            original.step(ticks(20_000_000)).unwrap();
            restored.step(ticks(20_000_000)).unwrap();
            let (a, b) = (original.snapshot(), restored.snapshot());
            worst_chassis = worst_chassis.max(chassis_gap(&a, &b));
            worst_projectile = worst_projectile.max(projectile_gap(&a, &b));
            assert_eq!(a.referee, b.referee, "referee state diverged");
            assert_eq!(a.runes, b.runes, "rune state diverged");
            assert_eq!(a.outposts, b.outposts, "outpost state diverged");
            assert_eq!(a.tick, b.tick);
            assert_eq!(a.shots_fired, b.shots_fired);
            assert_eq!(a.hits_detected, b.hits_detected);
        }
        eprintln!(
            "restore over 200 ticks: worst chassis {worst_chassis:.3e} m, worst ball {worst_projectile:.3e} m"
        );
        // Measured over these 200 ticks: the chassis agree exactly (0 m) and
        // the balls to 1.6e-17 m, one ulp of a metre, picked up where a spent
        // ball rolls on a rebuilt contact manifold. The bounds below leave
        // room for that rounding and for nothing else: a state the restore
        // forgot moves a driven chassis by millimetres within a few ticks,
        // which is what the pre-priming version of this test measured.
        assert!(
            worst_projectile < 1e-12,
            "projectile disagreement {worst_projectile} m"
        );
        assert!(
            worst_chassis < 1e-9,
            "chassis disagreement {worst_chassis} m"
        );
    }
    #[test]
    fn restoring_rejects_a_snapshot_it_cannot_rebuild() {
        let (field, _) = busy_field(true);
        let mut snapshot = field.snapshot();
        snapshot.restore = None;
        assert!(matches!(
            Field::restore(&snapshot, &StaticGeometry::default(), 0.0),
            Err(FieldError::Restore(_))
        ));
        let snapshot = field.snapshot();
        assert!(matches!(
            Field::restore(&snapshot, &StaticGeometry::default(), f64::NAN),
            Err(FieldError::Restore(_))
        ));
        let mut broken = field.snapshot();
        broken.time_ns += 1;
        assert!(matches!(
            Field::restore(&broken, &StaticGeometry::default(), 0.0),
            Err(FieldError::Restore(_))
        ));
    }
    #[test]
    fn default_field_steps_deterministically_in_any_partition() {
        let config = FieldConfig::default();
        let mut whole = Field::new(&config).unwrap();
        let mut split = Field::new(&config).unwrap();
        whole.step(2_600).unwrap();
        for ticks in [1, 999, 1_000, 600] {
            split.step(ticks).unwrap();
        }
        assert_eq!(whole.snapshot(), split.snapshot());
        let snapshot = whole.snapshot();
        assert_eq!(snapshot.tick, 2_600);
        assert_eq!(snapshot.time_ns, 2_600 * tick_ns());
        assert!(snapshot.outposts.is_empty());
        assert!(snapshot.runes.is_empty());
        // Reading a snapshot does not advance time.
        assert_eq!(whole.snapshot(), snapshot);
    }
    #[test]
    fn projectiles_keep_stepping_deterministic_in_any_partition() {
        let config = FieldConfig::default();
        let mut whole = Field::new(&config).unwrap();
        let mut split = Field::new(&config).unwrap();
        let muzzle = Pose::yawed([0.0, 0.0, 1.0], 0.6);
        let shot = Shot::at_limit(Caliber::Mm17);
        for field in [&mut whole, &mut split] {
            field.step(ticks(500_000_000)).unwrap();
            field.fire(muzzle, shot, None).unwrap();
        }
        let total = ticks(700_000_000);
        whole.step(total).unwrap();
        let parts = [1, total / 4, total / 4];
        for part in parts {
            split.step(part).unwrap();
        }
        split.step(total - parts.iter().sum::<u64>()).unwrap();
        let a = whole.snapshot();
        let b = split.snapshot();
        assert_eq!(a, b);
        assert_eq!(a.projectiles.len(), 1);
        assert!(a.projectiles[0].position_m[0] > 5.0, "{:?}", a.projectiles);
        // The world keeps ticking after the shot is spent.
        whole.step(ticks(5_000_000_000)).unwrap();
        assert!(whole.snapshot().projectiles.is_empty());
        assert_eq!(
            whole.tick(),
            ticks(500_000_000) + ticks(700_000_000) + ticks(5_000_000_000)
        );
    }

    /// A field with a chassis and a 15 degree ramp mesh ahead of it.
    fn ramp_field() -> FieldConfig {
        FieldConfig {
            step_duration: StepDuration::default(),
            rmuc_2027: None,
            zones: Vec::new(),
            bases: vec![],
            runes: Vec::new(),
            outposts: Vec::new(),
            floor_height_m: 0.0,
            referee: None,
            projectile_policy: Default::default(),
            chassis: vec![ChassisPlacement {
                config: ChassisConfig::default(),
                spawn: Pose::at([0.0, 0.0, ChassisConfig::default().rest_height_m()]),
                team: Team::Red,
                kind: RobotKind::Infantry,
                performance: None,
            }],
        }
    }
    fn add_ramp(field: &mut Field) {
        // Rises 15 degrees from x = 1 to x = 3, then a 2 m plateau.
        let top = 2.0 * 15_f64.to_radians().tan();
        field
            .add_static_mesh(
                vec![
                    [1.0, -1.0, 0.0],
                    [1.0, 1.0, 0.0],
                    [3.0, -1.0, top],
                    [3.0, 1.0, top],
                    [5.0, -1.0, top],
                    [5.0, 1.0, top],
                    [5.0, -1.0, 0.0],
                    [5.0, 1.0, 0.0],
                ],
                vec![
                    [0, 2, 1],
                    [1, 2, 3],
                    [2, 4, 3],
                    [3, 4, 5],
                    [4, 6, 5],
                    [5, 6, 7],
                ],
            )
            .unwrap();
    }
    #[test]
    fn mecanum_chassis_steps_deterministically_in_any_partition() {
        let mut config = ramp_field();
        config.chassis[0].config = ChassisConfig::hero();
        config.chassis[0].spawn.translation_m[2] = ChassisConfig::hero().rest_height_m();
        let mut whole = Field::new(&config).unwrap();
        let mut split = Field::new(&config).unwrap();
        for field in [&mut whole, &mut split] {
            field
                .command_chassis(
                    0,
                    ChassisCommand {
                        forward_m_s: 1.0,
                        left_m_s: 0.5,
                        yaw_rate_rad_s: 0.3,
                        ..ChassisCommand::default()
                    },
                )
                .unwrap();
        }
        whole.step(1000).unwrap();
        for ticks in [1, 249, 333, 417] {
            split.step(ticks).unwrap();
        }
        assert_eq!(whole.snapshot(), split.snapshot());
    }
    #[test]
    fn rotating_consumes_drive_capacity_while_climbing() {
        let climb = |spin: f64| {
            let mut field = Field::new(&ramp_field()).unwrap();
            add_ramp(&mut field);
            field.step(ticks(300_000_000)).unwrap();
            for _ in 0..250 {
                let yaw = chassis::yaw_of(field.snapshot().chassis[0].pose);
                // Hold the same world-space uphill wish while the body rotates.
                field
                    .command_chassis(
                        0,
                        ChassisCommand {
                            forward_m_s: 1.5 * yaw.cos(),
                            left_m_s: -1.5 * yaw.sin(),
                            yaw_rate_rad_s: spin,
                            ..Default::default()
                        },
                    )
                    .unwrap();
                field.step(ticks(10_000_000)).unwrap();
            }
            field.snapshot().chassis[0].pose.translation_m[0]
        };
        let straight = climb(0.);
        let rotating = climb(6.);
        assert!(straight > 2.3, "straight={straight}");
        assert!(
            rotating < straight - 0.2,
            "straight={straight}, rotating={rotating}"
        );
    }

    #[test]
    fn chassis_climbs_a_ramp_and_steps_deterministically_in_any_partition() {
        let config = ramp_field();
        let mut whole = Field::new(&config).unwrap();
        let mut split = Field::new(&config).unwrap();
        for field in [&mut whole, &mut split] {
            add_ramp(field);
            field.step(ticks(300_000_000)).unwrap();
            field
                .command_chassis(
                    0,
                    ChassisCommand {
                        forward_m_s: 1.5,
                        ..Default::default()
                    },
                )
                .unwrap();
        }
        let total = ticks(3_500_000_000);
        whole.step(total).unwrap();
        let parts = [1, total / 4, total / 3];
        for part in parts {
            split.step(part).unwrap();
        }
        split.step(total - parts.iter().sum::<u64>()).unwrap();
        assert_eq!(whole.snapshot(), split.snapshot());
        let chassis = whole.snapshot().chassis.remove(0);
        let top = 2.0 * 15_f64.to_radians().tan();
        let [x, _, z] = chassis.pose.translation_m;
        assert!(x > 3.2 && x < 5.0, "{chassis:?}");
        let preset = ChassisConfig::default();
        let sag = preset.mass_kg * 9.81 / 4.0 / preset.suspension_stiffness_n_m;
        assert!(
            (z - (top + preset.rest_height_m() - sag)).abs() < 0.02,
            "{chassis:?}"
        );
        assert!(chassis.wheels.iter().all(|w| w.contact.is_some()));
        assert_eq!(whole.tick(), ticks(300_000_000) + total);
        // A field without a chassis or projectiles still jumps straight ahead.
        let mut plain = Field::new(&FieldConfig::default()).unwrap();
        assert!(plain.snapshot().chassis.is_empty());
        assert!(matches!(
            plain.command_chassis(0, ChassisCommand::default()),
            Err(FieldError::Chassis(_))
        ));
        plain.step(10).unwrap();
    }
    /// Chassis join and leave a running field; ids count up and are never
    /// reused, and each one answers only to its own id.
    #[test]
    fn chassis_join_and_leave_with_stable_ids() {
        let mut field = Field::new(&FieldConfig::default()).unwrap();
        let placement = |x: f64, team: Team| ChassisPlacement {
            config: ChassisConfig::default(),
            spawn: Pose::at([x, 0.0, ChassisConfig::default().rest_height_m()]),
            team,
            kind: RobotKind::Infantry,
            performance: None,
        };
        let red = field.add_chassis(&placement(-2.0, Team::Red)).unwrap();
        let blue = field.add_chassis(&placement(-4.0, Team::Blue)).unwrap();
        assert_eq!((red, blue), (0, 1));
        assert_eq!(field.chassis_team(blue), Some(Team::Blue));
        field.step(300).unwrap();
        field
            .command_chassis(
                blue,
                ChassisCommand {
                    forward_m_s: 1.0,
                    ..Default::default()
                },
            )
            .unwrap();
        field.step(1_000).unwrap();
        let snapshot = field.snapshot();
        assert_eq!(
            snapshot
                .chassis
                .iter()
                .map(|c| (c.id, c.team))
                .collect::<Vec<_>>(),
            vec![(red, Team::Red), (blue, Team::Blue)]
        );
        // Only the commanded chassis moved.
        assert!(snapshot.chassis[0].pose.translation_m[0].abs() - 2.0 < 0.05);
        assert!(snapshot.chassis[1].pose.translation_m[0] > -3.5);
        field.remove_chassis(red).unwrap();
        assert!(matches!(
            field.remove_chassis(red),
            Err(FieldError::Chassis(_))
        ));
        assert!(matches!(
            field.command_chassis(red, ChassisCommand::default()),
            Err(FieldError::Chassis(_))
        ));
        let next = field.add_chassis(&placement(-2.0, Team::Red)).unwrap();
        assert_eq!(next, 2);
        field.step(100).unwrap();
        let ids: Vec<u32> = field.snapshot().chassis.iter().map(|c| c.id).collect();
        assert_eq!(ids, vec![blue, next]);
    }
    /// A roof the body fits under but the turret does not stops the chassis.
    #[test]
    fn chassis_turret_stops_under_a_low_roof() {
        let config = ramp_field();
        let chassis = config.chassis[0].config.clone();
        let mut field = Field::new(&config).unwrap();
        let roof_z = chassis.rest_height_m() + chassis.body_half_m[2] + 0.05;
        assert!(
            roof_z
                < chassis.rest_height_m() + chassis.turret_center_m[2] + chassis.turret_half_m[2]
        );
        field
            .add_static_mesh(
                vec![
                    [1.0, -1.0, roof_z],
                    [1.0, 1.0, roof_z],
                    [3.0, -1.0, roof_z],
                    [3.0, 1.0, roof_z],
                ],
                vec![[0, 1, 2], [1, 3, 2]],
            )
            .unwrap();
        field.step(300).unwrap();
        field
            .command_chassis(
                0,
                ChassisCommand {
                    forward_m_s: 1.0,
                    ..ChassisCommand::default()
                },
            )
            .unwrap();
        field.step(3_000).unwrap();
        let snapshot = field.snapshot().chassis.remove(0);
        let x = snapshot.pose.translation_m[0];
        // The turret's front face reaches the roof edge and no further.
        let turret_front = chassis.turret_center_m[0] + chassis.turret_half_m[0];
        assert!(x + turret_front < 1.05 && x + turret_front > 0.8, "{x}");
        assert!(
            snapshot.velocity_m_s[0].abs() < 0.2,
            "{:?}",
            snapshot.velocity_m_s
        );
    }
    /// A closed 1 m box mesh centred at `center`.
    fn box_mesh(center: [f64; 3], half: f64) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
        let [cx, cy, cz] = center;
        let mut vertices = Vec::new();
        for i in 0..8 {
            let sx = if i & 1 == 0 { -half } else { half };
            let sy = if i & 2 == 0 { -half } else { half };
            let sz = if i & 4 == 0 { -half } else { half };
            vertices.push([cx + sx, cy + sy, cz + sz]);
        }
        let triangles = vec![
            [0, 2, 1],
            [1, 2, 3],
            [4, 5, 6],
            [5, 7, 6],
            [0, 1, 4],
            [1, 5, 4],
            [2, 6, 3],
            [3, 6, 7],
            [0, 4, 2],
            [2, 4, 6],
            [1, 3, 5],
            [3, 7, 5],
        ];
        (vertices, triangles)
    }
    #[test]
    fn dynamic_geometry_tracks_bodies_and_excludes_terrain() {
        let mut field = Field::new(&FieldConfig {
            runes: Vec::new(),
            outposts: Vec::new(),
            ..Default::default()
        })
        .unwrap();
        assert!(
            field
                .dynamic_geometry_snapshot()
                .into_triangles()
                .0
                .is_empty()
        );
        let id = field
            .add_chassis(&ChassisPlacement {
                team: Team::Red,
                kind: RobotKind::Infantry,
                performance: None,
                config: Default::default(),
                spawn: Pose::at([0., 0., 2.]),
            })
            .unwrap();
        let before = field.dynamic_geometry_snapshot().into_triangles();
        assert!(!before.1.is_empty());
        field.step(100).unwrap();
        assert_ne!(
            before.0,
            field.dynamic_geometry_snapshot().into_triangles().0
        );
        field.remove_chassis(id).unwrap();
        assert!(
            field
                .dynamic_geometry_snapshot()
                .into_triangles()
                .0
                .is_empty()
        );
        field
            .fire(Pose::at([0., 0., 2.]), Shot::at_limit(Caliber::Mm17), None)
            .unwrap();
        let captured = field.dynamic_geometry_snapshot();
        field.step(10).unwrap();
        let projectile = captured.into_triangles();
        assert!(!projectile.1.is_empty());
        assert_ne!(
            projectile.0,
            field.dynamic_geometry_snapshot().into_triangles().0
        );
        assert!(field.static_geometry().0.is_empty());
    }

    #[test]
    fn fixed_geometry_snapshot_survives_field_changes_and_drop() {
        let mut field = Field::new(&ramp_field()).unwrap();
        add_ramp(&mut field);
        let expected = field.static_geometry();
        let captured = field.static_geometry_snapshot();
        let (vertices, triangles) = box_mesh([1.5, 0.0, 0.5], 0.5);
        field.add_static_mesh(vertices, triangles).unwrap();
        assert!(field.static_geometry().1.len() > expected.1.len());
        drop(field);
        let actual = std::thread::spawn(move || captured.into_triangles())
            .join()
            .unwrap();
        assert_eq!(actual, expected);
    }

    /// A triangle mesh block ahead of the chassis stops it and bounces
    /// projectiles; the physics geometry export shows it and the meshes.
    #[test]
    fn triangle_mesh_blocks_chassis_and_projectiles() {
        let config = ramp_field();
        let mut field = Field::new(&config).unwrap();
        assert_eq!(field.static_geometry().1.len(), 0);
        add_ramp(&mut field);
        assert_eq!(field.static_geometry().1.len(), 6);
        let (vertices, triangles) = box_mesh([1.5, 0.0, 0.5], 0.5);
        let count = triangles.len();
        field.add_static_mesh(vertices, triangles).unwrap();
        assert_eq!(field.static_geometry().1.len(), 6 + count);
        // A projectile fired at the block comes back.
        // Offset from the robot so the returning ball tests the wall rather
        // than striking the chassis whose new silhouette is being tested below.
        let muzzle = Pose::at([0.0, 0.45, 0.5]);
        field
            .fire(muzzle, Shot::at_limit(Caliber::Mm17), None)
            .unwrap();
        field.step(120).unwrap();
        let projectile = &field.snapshot().projectiles[0];
        assert!(projectile.position_m[0] < 1.0, "{projectile:?}");
        assert!(projectile.velocity_m_s[0] < 0.0, "{projectile:?}");
        // The chassis drives into the block and stops at its face.
        field
            .command_chassis(
                0,
                ChassisCommand {
                    forward_m_s: 1.0,
                    ..ChassisCommand::default()
                },
            )
            .unwrap();
        field.step(3_000).unwrap();
        let chassis = field.snapshot().chassis.remove(0);
        let x = chassis.pose.translation_m[0];
        let front = config.chassis[0].config.body_half_m[0];
        assert!(x + front < 1.05 && x + front > 0.8, "{x}");
        assert!(
            chassis.velocity_m_s[0].abs() < 0.2,
            "{:?}",
            chassis.velocity_m_s
        );
    }
    #[test]
    fn chassis_stops_against_a_wall_and_rejects_bad_placements() {
        let config = ramp_field();
        let mut field = Field::new(&config).unwrap();
        // A wall across the path at x = 1.5 whose winding faces away from the
        // chassis, and a back-facing deck under it: mesh contacts are two-sided.
        field
            .add_static_mesh(
                vec![
                    [1.5, -1.0, 0.0],
                    [1.5, 1.0, 0.0],
                    [1.5, -1.0, 1.0],
                    [1.5, 1.0, 1.0],
                ],
                vec![[0, 1, 2], [1, 3, 2]],
            )
            .unwrap();
        field
            .add_static_mesh(
                vec![
                    [-1.0, -1.0, 0.03],
                    [-1.0, 1.0, 0.03],
                    [1.5, -1.0, 0.03],
                    [1.5, 1.0, 0.03],
                ],
                vec![[0, 1, 2], [1, 3, 2]],
            )
            .unwrap();
        field.step(300).unwrap();
        let resting = field.snapshot().chassis.remove(0);
        assert!(
            resting.wheels.iter().all(|w| w
                .contact
                .is_some_and(|c| (c.point_m[2] - 0.03).abs() < 1e-6)),
            "{resting:?}"
        );
        field
            .command_chassis(
                0,
                ChassisCommand {
                    forward_m_s: 2.0,
                    ..Default::default()
                },
            )
            .unwrap();
        field.step(3_000).unwrap();
        let chassis = field.snapshot().chassis.remove(0);
        let front = chassis.pose.translation_m[0] + ChassisConfig::default().body_half_m[0];
        assert!(front < 1.52 && front > 1.3, "{chassis:?}");
        assert!(chassis.velocity_m_s[0].abs() < 0.2, "{chassis:?}");
        assert!(matches!(
            Field::new(&FieldConfig {
                chassis: vec![ChassisPlacement {
                    config: ChassisConfig {
                        mass_kg: -1.0,
                        ..ChassisConfig::default()
                    },
                    spawn: Pose::default(),
                    team: Team::Red,
                    kind: RobotKind::Infantry,
                    performance: None,
                }],
                ..FieldConfig::default()
            }),
            Err(FieldError::Chassis(_))
        ));
    }
}
