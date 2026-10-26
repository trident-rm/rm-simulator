// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! The authoritative field with pause and pacing state, and the chassis
//! spawner that gives players a body. The host clock is converted to ticks
//! here, never inside the world crate.
use crate::cad_assets::CadAssets;
use crate::layout::{
    ChassisSpawner, LayoutOptions, SPAWN_SLOT_SPACING_M, add_terrain, field_config, load_terrain,
};
use crate::protocol::{Command, Robot, WeaponConfig};
use rm_sim_world::{ChassisConfig, Field, FieldError, FieldSnapshot, RobotKind, Team, tick_ns};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, VecDeque};

/// Catch-up cap so a stalled host does not spiral the world clock.
pub const MAX_ADVANCE_NS: u64 = 250_000_000;
/// World time one manual step covers while paused, in nanoseconds. The step is
/// a duration, not a tick count, so it covers at least 16 ms at every physics
/// rate; a rate whose ticks do not divide it rounds up to the next boundary.
pub const STEP_NS: u64 = 16_000_000;

/// Ticks in one manual step while paused: the first tick boundary at or after
/// [`STEP_NS`] at the current rate, so a step is never shorter than 16 ms.
///
/// ```
/// use rm_sim_server::simulation::{STEP_NS, step_ticks};
///
/// let tick_ns = rm_sim_world::tick_ns();
/// assert!(step_ticks() * tick_ns >= STEP_NS);
/// assert!((step_ticks() - 1) * tick_ns < STEP_NS);
/// ```
pub fn step_ticks() -> u64 {
    STEP_NS.div_ceil(tick_ns()).max(1)
}

/// A world-time interval schedule that yields whole ticks without losing time.
///
/// A fixed tick cannot advance by a fraction of itself, so a caller that
/// schedules intervals shorter than one tick — a measurement harness pacing a
/// codec every 2 ms, say — must accumulate the remainder. Rounding every
/// interval to a tick on its own would make the simulated duration disagree
/// with the duration the caller reports, and a 2 ms interval rounded up to a
/// 7.8125 ms tick runs the world 3.9 times too fast.
///
/// [`Simulation::advance_observed`] keeps the same accumulator for host time;
/// this type exposes that arithmetic to callers that pace their own clock.
///
/// ```
/// use rm_sim_server::simulation::TickSchedule;
///
/// let tick_ns = rm_sim_world::tick_ns();
/// let mut schedule = TickSchedule::default();
/// // Eight 2 ms intervals cover 16 ms, which is two whole 128 Hz ticks.
/// let ticks: u64 = (0..8).map(|_| schedule.advance(2_000_000)).sum();
/// assert_eq!(ticks, 16_000_000 / tick_ns);
/// assert_eq!(schedule.elapsed_ns(), 16_000_000);
/// assert_eq!(schedule.issued_ticks(), ticks);
/// ```
#[derive(Clone, Copy, Debug, Default)]
pub struct TickSchedule {
    elapsed_ns: u64,
    issued_ticks: u64,
}

impl TickSchedule {
    /// Advance the schedule by `duration_ns` and return the whole ticks that
    /// became due. The result is zero for an interval that did not complete a
    /// tick; the remainder stays in [`TickSchedule::elapsed_ns`] and is charged
    /// to a later interval, so a caller that steps by the returned count keeps
    /// simulated time equal to scheduled time.
    pub fn advance(&mut self, duration_ns: u64) -> u64 {
        self.elapsed_ns += duration_ns;
        let due = self.elapsed_ns / tick_ns();
        let ticks = due - self.issued_ticks;
        self.issued_ticks = due;
        ticks
    }

    /// World time the schedule has covered so far, in nanoseconds. This is the
    /// duration a caller has scheduled, not the duration it has simulated;
    /// `issued_ticks * tick_ns()` is the latter.
    pub fn elapsed_ns(&self) -> u64 {
        self.elapsed_ns
    }

    /// Whole ticks issued so far, which is how far the world has advanced.
    pub fn issued_ticks(&self) -> u64 {
        self.issued_ticks
    }

    /// Ticks an interval of `duration_ns` starting now covers: the sample
    /// interval a frame submitted at this point should declare. At least one,
    /// because an input frame may not declare a zero-tick duration.
    pub fn span_ticks(&self, duration_ns: u64) -> u64 {
        let span = (self.elapsed_ns + duration_ns) / tick_ns() - self.elapsed_ns / tick_ns();
        span.max(1)
    }
}

/// What clients see: the field plus the host's pacing state.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SimulationState {
    /// Chassis ids of the training bots, ascending.
    pub bots: Vec<u32>,
    /// Host controller diagnostics and paths; not additional physical rule state.
    pub navigation: Vec<crate::navigation::View>,
    /// Opted-in Sentry scan/aim state and current optical detections.
    pub sentry_tracking: Vec<crate::sentry_controller::View>,
    /// Host publication identity; zero for an unpublished operator capture.
    pub snapshot_id: u64,
    /// Counter bumped on every pause transition. Inputs and shots stamped with
    /// another epoch are refused.
    pub input_epoch: u64,
    /// Each shooter's newest 32 results, newest first.
    pub shot_results: Vec<crate::protocol::ShotResult>,
    /// Whether the host is paused.
    pub paused: bool,
    /// The whole field at the publication tick.
    pub field: FieldSnapshot,
}

/// Small control/clock reply, captured on the authoritative worker without a world snapshot.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ControlStatus {
    /// Current simulation time in nanoseconds.
    pub time_ns: u64,
    /// Current pause epoch for input admission.
    pub input_epoch: u64,
    /// Whether simulation pacing is paused.
    pub paused: bool,
    /// Optional addressed robot's life/defeat identity; absent for clock-only queries.
    pub chassis: Option<ControlIdentity>,
}
/// Minimal robot identity needed to start an external control lease.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ControlIdentity {
    /// Stable simulator chassis ID, never a referee ID.
    pub id: u32,
    /// Revision changed by placement or revival.
    pub placement_revision: u64,
    /// Defeated robots cannot accept an external lease.
    pub defeated: bool,
}

/// Construction events shared by the window's loading screen and headless host.
#[derive(Debug, PartialEq)]
pub enum BuildProgress {
    /// Terrain files are being read.
    ReadingTerrain,
    /// The physics world and the rules are being built.
    BuildingPhysics,
    /// Terrain finished loading; the string describes the loaded mesh.
    TerrainReady(String),
    /// The chassis spawner is being prepared.
    PreparingChassis,
}

/// Accepted shot journal, bounded to the latest 256 shots. Times are simulation times
/// except the explicitly client-relative clock. Projectile ID links to impact records.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FireRecord {
    /// Actual initial speed after sampling, in m/s.
    pub launch_speed_m_s: f64,
    /// Chassis id of the shooter.
    pub shooter: u32,
    /// Id of the projectile in the field snapshot.
    pub projectile_id: u64,
    /// Simulation time at which the shot fired, in ns.
    pub accepted_time_ns: u64,
    /// Muzzle pose the host fired from, in FLU metres and wxyz.
    pub authoritative_muzzle_pose: rm_sim_world::Pose,
}

/// Host records with the lifetime of one addressed chassis, including bots.
#[derive(Default)]
struct ChassisState {
    spawn: Option<rm_sim_world::Pose>,
    robot: Option<Robot>,
    weapon: Option<WeaponConfig>,
    shots: ShotState,
}
#[derive(Default)]
struct ShotState {
    last_id: u64,
    results: VecDeque<crate::protocol::ShotResult>,
    intended_ns: VecDeque<u64>,
}

/// The authoritative field with the host's pause, pacing, input and shot state.
/// One host worker owns the only live value. World time moves only through
/// [`advance`](Simulation::advance) and [`step`](Simulation::step), and only in
/// whole fixed 128 Hz ticks.
pub struct Simulation {
    sensors: crate::sensor_stream::Service,
    competition: crate::host_args::Competition,
    external_control: crate::external_control::Service,
    navigation: Option<crate::navigation::Service>,
    sentry_tracking: crate::sentry_controller::Service,
    assembly: Option<crate::assembly::Service>,
    /// Lobby password every remote seat must present. The embedded owner seat
    /// is exempt because it already holds the simulation.
    pub(crate) password: String,
    bots: std::collections::BTreeSet<u32>,
    /// Ordered so that snapshot bytes never depend on hash iteration order.
    pub(crate) input_streams: BTreeMap<u32, crate::input_stream::InputStream>,
    chassis_state: BTreeMap<u32, ChassisState>,
    pending_shots: Vec<Command>,
    completed_shots: VecDeque<crate::protocol::ShotResult>,
    prediction_scene: Option<crate::prediction::PredictionScene>,
    fire_records: VecDeque<FireRecord>,
    field: Field,
    paused: bool,
    input_epoch: u64,
    /// Host time not yet turned into a whole tick.
    carry_ns: u64,
    /// How players get a chassis; without one everybody spectates.
    spawner: Option<ChassisSpawner>,
    weapon: WeaponConfig,
    weapon_limits: crate::protocol::WeaponLimits,
    /// Starting settings and caps for a 42 mm (Hero) chassis.
    hero_weapon: WeaponConfig,
    hero_weapon_limits: crate::protocol::WeaponLimits,
}

impl Simulation {
    /// Construct an explicitly approximate RMUL practice floor without loading UC CAD.
    /// The native match remains the rules authority; terrain and equipment are absent.
    ///
    /// ```
    /// use rm_sim_server::{host_args::Competition, simulation::Simulation};
    /// let simulation = Simulation::rmul_practice(
    ///     Competition::RmulInfantry2027, true, Some(Default::default()), false,
    /// ).unwrap();
    /// assert!(simulation.field().rmul_2027().is_some());
    /// assert!(simulation.field().rmuc_2027().is_none());
    /// ```
    pub fn rmul_practice(
        competition: crate::host_args::Competition,
        referee: bool,
        chassis: Option<ChassisConfig>,
        paused: bool,
    ) -> anyhow::Result<Self> {
        let format = competition
            .rmul_format()
            .ok_or_else(|| anyhow::anyhow!("RMUL practice requires an RMUL competition"))?;
        let mut field = Field::new(&rm_sim_world::FieldConfig {
            rmul_2027: referee.then_some(format),
            ..Default::default()
        })?;
        let [low, high] = competition.practice_bounds();
        field.set_projectile_bounds([low, high])?;
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        for (a, b) in [
            ([low[0], low[1]], [high[0], low[1]]),
            ([high[0], low[1]], [high[0], high[1]]),
            ([high[0], high[1]], [low[0], high[1]]),
            ([low[0], high[1]], [low[0], low[1]]),
        ] {
            let i = vertices.len() as u32;
            vertices.extend([
                [a[0], a[1], 0.0],
                [b[0], b[1], 0.0],
                [b[0], b[1], 1.0],
                [a[0], a[1], 1.0],
            ]);
            triangles.extend([[i, i + 1, i + 2], [i, i + 2, i + 3]]);
        }
        field.add_boundary_mesh(vertices, triangles)?;
        let mut simulation = Self::new(field, paused);
        simulation.competition = competition;
        if let Some(config) = chassis {
            simulation = simulation.with_spawner(ChassisSpawner {
                config,
                terrain: None,
            });
        }
        Ok(simulation)
    }
    /// Competition selected by this host, independent of referee enablement.
    pub fn competition(&self) -> crate::host_args::Competition {
        self.competition
    }
    /// Build the CAD field and optional chassis spawner without opening sockets
    /// or advancing time. The caller chooses whether to run this on a worker.
    /// No player is added until a spawn method is called.
    pub fn from_cad(
        cad: &CadAssets,
        options: &LayoutOptions,
        chassis: Option<ChassisConfig>,
        paused: bool,
        progress: impl FnMut(BuildProgress),
    ) -> anyhow::Result<Self> {
        cad.season.require_current()?;
        anyhow::ensure!(
            !options.referee || cad.rmuc_2027.is_some(),
            "RMUC 2027 matches require a bound field package; use --no-referee for rest-pose practice"
        );
        Self::build_cad(cad, options, chassis, paused, progress)
    }

    // Geometry construction is shared with historical regression fixtures. Only
    // from_cad is public and admits application sessions.
    fn build_cad(
        cad: &CadAssets,
        options: &LayoutOptions,
        chassis: Option<ChassisConfig>,
        paused: bool,
        mut progress: impl FnMut(BuildProgress),
    ) -> anyhow::Result<Self> {
        let terrain = if options.terrain {
            progress(BuildProgress::ReadingTerrain);
            Some(load_terrain(cad)?)
        } else {
            None
        };
        progress(BuildProgress::BuildingPhysics);
        anyhow::ensure!(
            cad.rmuc_2027.is_none() || !options.referee || options.terrain,
            "live RMUC 2027 contacts require the verified terrain; use --no-referee for bare practice"
        );
        let mut config = field_config(cad, options);
        if let (Some(native), Some(terrain)) = (&mut config.rmuc_2027, &terrain) {
            crate::rmuc_2027::fit_zone_heights(native, &terrain.ground);
        }
        let mut field = Field::new(&config)?;
        if let Some(terrain) = &terrain {
            add_terrain(&mut field, terrain)?;
            progress(BuildProgress::TerrainReady(terrain.describe()));
        }
        let mut simulation = Self::new(field, paused);
        if let Some(config) = chassis {
            progress(BuildProgress::PreparingChassis);
            simulation = simulation.with_spawner(ChassisSpawner { config, terrain });
        }
        simulation.prediction_scene = crate::prediction::PredictionScene::from_cad(cad, options);
        Ok(simulation)
    }

    /// Wrap a field with empty pacing, input and shot state. No player is added
    /// and no prediction scene is advertised.
    ///
    /// # Panics
    /// Panics for a nondefault field step; use [`Self::try_new`] for admission.
    pub fn new(field: Field, paused: bool) -> Self {
        Self::try_new(field, paused).expect("server simulation requires the default 128 Hz field")
    }
    /// Admit a default-rate field. Custom steps are currently API-only in the
    /// world/physics crates; network prediction and assembly planning use 128 Hz.
    pub fn try_new(field: Field, paused: bool) -> Result<Self, &'static str> {
        if field.step_duration() != rm_sim_world::StepDuration::default() {
            return Err("server simulation supports only the default 128 Hz field step");
        }
        Ok(Self {
            competition: field.rmul_2027().map_or(Default::default(), |game| {
                if game.format == rm_sim_world::gameplay::rmul_2027::Format::Infantry {
                    crate::host_args::Competition::RmulInfantry2027
                } else {
                    crate::host_args::Competition::Rmul2027
                }
            }),
            sensors: Default::default(),
            external_control: Default::default(),
            navigation: None,
            sentry_tracking: Default::default(),
            assembly: None,
            password: String::new(),
            bots: Default::default(),
            input_epoch: 0,
            input_streams: BTreeMap::new(),
            chassis_state: BTreeMap::new(),
            pending_shots: Vec::new(),
            completed_shots: VecDeque::new(),
            prediction_scene: None,
            field,
            paused,
            carry_ns: 0,
            spawner: None,
            weapon: WeaponConfig::default(),
            weapon_limits: Default::default(),
            hero_weapon: WeaponConfig::hero(),
            hero_weapon_limits: crate::protocol::WeaponLimits::hero(),
            fire_records: VecDeque::new(),
        })
    }
    /// Attach measured LiDAR/IMU to the existing world; no motion authority is granted.
    /// Returns an attachment ID; poll within five host seconds to retain it.
    pub fn attach_sensors(
        &mut self,
        config: rm_sim_physics::sensors::synchronized::Config,
    ) -> Result<u64, String> {
        self.sensors.attach(&mut self.field, config)
    }
    /// Attach the same measured sensors with opt-in server wall-duration telemetry.
    /// Timing metadata stays outside physics and never changes simulation ns stamps.
    pub fn attach_sensors_profiled(
        &mut self,
        config: rm_sim_physics::sensors::synchronized::Config,
    ) -> Result<u64, String> {
        self.sensors.attach_profiled(&mut self.field, config)
    }
    /// Drain up to two measured bundles and their host durations in nanoseconds.
    /// Requires a profiled attachment; direct calls have no owner mailbox timing.
    pub fn poll_sensors_profiled(
        &mut self,
        id: u64,
    ) -> Result<crate::sensor_stream::ProfiledPoll, String> {
        self.sensors.poll_profiled(&mut self.field, id)
    }
    /// Drain at most two synchronized measured packets; timestamps are simulation ns.
    pub fn poll_sensors(&mut self, id: u64) -> Result<crate::sensor_stream::Poll, String> {
        self.sensors.poll(&mut self.field, id)
    }
    /// Remove a read-only sensor attachment and its bounded queues.
    pub fn detach_sensors(&mut self, id: u64) {
        self.sensors.detach(&mut self.field, id);
    }
    /// Poll the bounded worker without advancing the world, including while paused.
    pub(crate) fn is_sentry_bot(&self, id: u32) -> bool {
        self.bots.contains(&id) && self.robot(id) == Some(Robot::Sentry)
    }
    fn revoke_external_control(&mut self, id: u32) {
        if self.external_control.revoke(id) {
            let _ = crate::navigation::brake(&mut self.field, id);
        }
    }
    pub(crate) fn poll_navigation(&mut self) {
        if let Some(nav) = &mut self.navigation {
            nav.poll(&self.field);
        }
    }
    pub(crate) fn accept_navigation_intent(&mut self, event: &crate::map_interaction::Event) {
        if let crate::map_interaction::Target::Position(position_m) = event.interaction.target {
            for &id in &event.recipients {
                if self.external_control.active(id) {
                    continue;
                }
                if let Some(nav) = &mut self.navigation
                    && nav.owns(id)
                {
                    let _ = nav.goal(id, position_m);
                    let _ = crate::navigation::brake(&mut self.field, id);
                }
            }
        }
    }
    fn spawn_sentry(
        &mut self,
        team: Team,
        position_m: [f64; 3],
        yaw_deg: f64,
    ) -> Result<(), String> {
        crate::protocol::check_map_marker(
            crate::protocol::MapMarkerKind::Alert,
            [position_m[0], position_m[1]],
        )?;
        if !position_m.into_iter().chain([yaw_deg]).all(f64::is_finite) {
            return Err("Sentry placement must be finite".into());
        }
        if self
            .chassis_state
            .iter()
            .filter_map(|(&id, state)| state.robot.map(|robot| (id, robot)))
            .any(|(id, robot)| robot == Robot::Sentry && self.field.chassis_team(id) == Some(team))
        {
            return Err("team already has a Sentry".into());
        }
        if self.bots.len() >= 32 {
            return Err("at most 32 training bots".into());
        }
        self.validate_roster(Robot::Sentry)?;
        let config = self.robot_config(Robot::Sentry, crate::protocol::Chassis::Auto)?;
        let placement = self
            .spawner
            .as_ref()
            .ok_or("this field offers no chassis")?
            .at_with(config, RobotKind::Sentry, team, position_m, yaw_deg);
        let support = self
            .spawner
            .as_ref()
            .and_then(|s| s.terrain.as_ref())
            .and_then(|t| {
                t.ground
                    .ground_height_below(position_m[0], position_m[1], position_m[2])
            })
            .unwrap_or(self.field.floor_height_m());
        if !self.field.static_geometry_snapshot().cylinder_sweep_clear(
            [position_m[0], position_m[1], support + 0.40],
            [position_m[0], position_m[1], support + 0.40],
            crate::navigation::RADIUS_M,
            0.24,
        ) || self
            .field
            .chassis_positions_m()
            .any(|p| (p[0] - position_m[0]).hypot(p[1] - position_m[1]) < 0.9)
        {
            return Err("Sentry spawn is obstructed or occupied".into());
        }
        if self.navigation.is_none() {
            let ground = self
                .spawner
                .as_ref()
                .and_then(|s| s.terrain.as_ref())
                .map(|t| t.ground.clone());
            let bounds = self
                .spawner
                .as_ref()
                .and_then(|s| s.terrain.as_ref())
                .and_then(|t| t.projectile_bounds_m)
                .map_or([-14.5, -8., 14.5, 8.], |b| {
                    [b[0][0], b[0][1], b[1][0], b[1][1]]
                });
            self.navigation = Some(crate::navigation::Service::new(
                self.field.fixed_geometry_snapshot(),
                ground,
                self.field.floor_height_m(),
                bounds,
                crate::navigation::road_exclusions(),
            )?);
        }
        let id = self.place(placement, Some(Robot::Sentry))?;
        self.bots.insert(id);
        self.navigation.as_mut().unwrap().add(id);
        self.sentry_tracking.set(id, true);
        Ok(())
    }
    fn spawn_bot(&mut self, team: Team, spin_rad_s: f64) -> Result<(), String> {
        if !spin_rad_s.is_finite() || spin_rad_s.abs() > 20. {
            return Err("bot spin must be finite and within +/-20 rad/s".into());
        }
        if self.bots.len() >= 32 {
            return Err("at most 32 training bots".into());
        }
        let id = self.spawn_chassis(team)?;
        self.field
            .command_chassis(
                id,
                rm_sim_world::ChassisCommand {
                    yaw_rate_rad_s: spin_rad_s,
                    ..Default::default()
                },
            )
            .map_err(|e| e.to_string())?;
        self.bots.insert(id);
        Ok(())
    }

    /// Let players spawn chassis.
    /// Require this password for every remote seat, including spectators.
    pub fn with_password(mut self, password: String) -> Self {
        self.password = password;
        self
    }

    /// Let players spawn chassis. Without a spawner, `spawn_chassis` fails and
    /// everybody spectates.
    pub fn with_spawner(mut self, spawner: ChassisSpawner) -> Self {
        self.spawner = Some(spawner);
        self
    }
    /// Set host starting settings and match the caps to their rate and speed.
    /// Call `with_weapon_limits` afterward to allow higher values. Fails when
    /// the configuration is invalid.
    pub fn with_weapon(mut self, weapon: WeaponConfig) -> Result<Self, String> {
        self.weapon = weapon.validate().map_err(str::to_string)?;
        self.weapon_limits = crate::protocol::WeaponLimits {
            max_speed_m_s: weapon.shot.speed_m_s,
            min_interval_ns: weapon.interval_ns,
        };
        Ok(self)
    }
    /// Set independent host caps, refusing limits that exclude the defaults.
    ///
    /// ```
    /// use rm_sim_server::{simulation::Simulation, protocol::WeaponLimits};
    /// use rm_sim_world::{Field, FieldConfig};
    /// let simulation = Simulation::new(Field::new(&FieldConfig::default()).unwrap(), false)
    ///     .with_weapon_limits(WeaponLimits::default()).unwrap();
    /// assert_eq!(simulation.weapon_limits().max_speed_m_s, 30.0);
    /// ```
    pub fn with_weapon_limits(
        mut self,
        limits: crate::protocol::WeaponLimits,
    ) -> Result<Self, String> {
        limits
            .admit(self.weapon.shot.caliber, self.weapon)
            .map_err(str::to_string)?;
        self.weapon_limits = limits;
        Ok(self)
    }
    /// Set the Hero's 42 mm starting settings and match its caps to their
    /// rate and speed, like [`Self::with_weapon`]. Call
    /// `with_hero_weapon_limits` afterward to allow higher values.
    pub fn with_hero_weapon(mut self, mut weapon: WeaponConfig) -> Result<Self, String> {
        weapon.shot.caliber = rm_sim_world::Caliber::Mm42;
        self.hero_weapon = weapon.validate().map_err(str::to_string)?;
        self.hero_weapon_limits = crate::protocol::WeaponLimits {
            max_speed_m_s: weapon.shot.speed_m_s,
            min_interval_ns: weapon.interval_ns,
        };
        Ok(self)
    }
    /// Set the Hero's caps, refusing limits that exclude its defaults.
    ///
    /// ```
    /// use rm_sim_server::{simulation::Simulation, protocol::{WeaponConfig, WeaponLimits}};
    /// use rm_sim_world::{Caliber, Field, FieldConfig};
    /// let simulation = Simulation::new(Field::new(&FieldConfig::default()).unwrap(), false)
    ///     .with_hero_weapon(WeaponConfig::hero()).unwrap()
    ///     .with_hero_weapon_limits(WeaponLimits::hero()).unwrap();
    /// assert_eq!(simulation.hero_weapon().shot.caliber, Caliber::Mm42);
    /// assert_eq!(simulation.limits_for_caliber(Caliber::Mm42).max_speed_m_s, 16.0);
    /// ```
    pub fn with_hero_weapon_limits(
        mut self,
        limits: crate::protocol::WeaponLimits,
    ) -> Result<Self, String> {
        limits
            .admit(rm_sim_world::Caliber::Mm42, self.hero_weapon)
            .map_err(str::to_string)?;
        self.hero_weapon_limits = limits;
        Ok(self)
    }
    /// Caps offered to every 17 mm pilot independently of their starting settings.
    pub fn weapon_limits(&self) -> crate::protocol::WeaponLimits {
        self.weapon_limits
    }
    /// Host starting settings for a 42 mm (Hero) chassis.
    pub fn hero_weapon(&self) -> WeaponConfig {
        self.hero_weapon
    }
    /// The caps for a launcher of `caliber`: the Hero's for 42 mm.
    pub fn limits_for_caliber(
        &self,
        caliber: rm_sim_world::Caliber,
    ) -> crate::protocol::WeaponLimits {
        let mut limits = match caliber {
            rm_sim_world::Caliber::Mm17 => self.weapon_limits,
            rm_sim_world::Caliber::Mm42 => self.hero_weapon_limits,
        };
        if self.field.rmuc_2027().is_some() && caliber == rm_sim_world::Caliber::Mm17 {
            // UC 2027 Tables 3-5/3-6; do not inherit the legacy 30 m/s configurable cap.
            limits.max_speed_m_s = limits.max_speed_m_s.min(25.);
        }
        limits
    }
    /// The caps a chassis' pilot may configure within, by its caliber.
    pub fn weapon_limits_for(&self, chassis: u32) -> crate::protocol::WeaponLimits {
        self.limits_for_caliber(self.caliber(chassis))
    }
    /// The scene clients need to predict, or None when the field was not built
    /// from a verifiable CAD package.
    pub fn prediction_scene(&self) -> Option<crate::prediction::PredictionScene> {
        self.prediction_scene.clone()
    }
    /// Host starting weapon settings.
    pub fn weapon(&self) -> WeaponConfig {
        let mut weapon = self.weapon;
        weapon.shot.speed_m_s = weapon
            .shot
            .speed_m_s
            .min(self.limits_for_caliber(weapon.shot.caliber).max_speed_m_s);
        weapon
    }
    /// The live field.
    pub fn field(&self) -> &Field {
        &self.field
    }
    /// Tests only: mutate the field directly. Prediction is disabled because the
    /// host can no longer promise clients an identical world.
    #[cfg(test)]
    pub(crate) fn test_field_mut(&mut self) -> &mut Field {
        self.prediction_scene = None;
        &mut self.field
    }
    /// Whether the host is paused.
    pub fn paused(&self) -> bool {
        self.paused
    }
    /// The current input epoch, bumped on every pause transition.
    pub fn input_epoch(&self) -> u64 {
        self.input_epoch
    }
    /// Set the pause flag. A change bumps the input epoch, commands every held
    /// input to neutral, cancels queued shots and discards un-ticked host time,
    /// so no input or shot crosses the pause.
    pub fn set_paused(&mut self, paused: bool) {
        if paused != self.paused {
            let ids: Vec<_> = self.field.snapshot().chassis.iter().map(|b| b.id).collect();
            for id in ids {
                self.revoke_external_control(id);
            }
            self.input_epoch = self
                .input_epoch
                .checked_add(1)
                .expect("input epoch exhausted");
            for (id, stream) in &mut self.input_streams {
                if let Some(neutral) = stream.cancel() {
                    let _ = self.field.command_chassis(*id, neutral);
                }
            }
            self.cancel_pending_shots(None, "shot cancelled by pause or epoch change");
            self.input_streams.clear();
            self.carry_ns = 0;
        }
        self.paused = paused;
    }
    /// The chassis configuration a chassis gets when no robot is named for
    /// it, when the field offers chassis at all.
    pub fn spawner_config(&self) -> Option<&ChassisConfig> {
        self.spawner.as_ref().map(|spawner| &spawner.config)
    }
    /// Give a player of `team` a chassis of the spawner's default
    /// configuration in the team's first spawn slot that no chassis is
    /// standing in. It is no particular robot, so its gun fires the host's
    /// default caliber; training bots take this path.
    pub fn spawn_chassis(&mut self, team: Team) -> Result<u32, String> {
        let config = self.spawner_config().cloned();
        self.spawn_in_slot(team, config, RobotKind::Infantry, None)
    }
    /// Give a pilot of `team` the chassis of `robot` in the team's first spawn
    /// slot that no chassis is standing in. The robot fixes the chassis
    /// preset, the class the referee records and the caliber its gun fires.
    pub fn spawn_robot(&mut self, team: Team, robot: Robot) -> Result<u32, String> {
        self.spawn_robot_with_chassis(team, robot, crate::protocol::Chassis::Auto)
    }
    /// Spawn a validated robot/drivetrain combination in a team slot.
    pub fn spawn_robot_with_chassis(
        &mut self,
        team: Team,
        robot: Robot,
        chassis: crate::protocol::Chassis,
    ) -> Result<u32, String> {
        self.validate_roster(robot)?;
        self.validate_rmul_slot(team, robot)?;
        let config = self.robot_config(robot, chassis)?;
        self.spawn_in_slot(team, Some(config), robot.kind(), Some(robot))
    }
    fn robot_config(
        &self,
        robot: Robot,
        chassis: crate::protocol::Chassis,
    ) -> Result<ChassisConfig, String> {
        let config = chassis.config(robot)?;
        let ega = self
            .spawner_config()
            .is_some_and(crate::navigation::tunnel_capable);
        if chassis == crate::protocol::Chassis::TunnelOmni && !ega {
            return Err("EGA tunnel Sentry requires its verified --robot-assets package".into());
        }
        Ok(
            if self
                .spawner_config()
                .and_then(|c| c.articulation.as_ref())
                .is_some_and(|a| match a.profile.as_str() {
                    "pie-tunnel-heavy-v1" => robot == Robot::Engineer,
                    "ega-tunnel-sentry-v1" => {
                        robot == Robot::Sentry && chassis != crate::protocol::Chassis::Omni
                    }
                    "future-tunnel-infantry-v1" => matches!(
                        robot,
                        Robot::Infantry3 | Robot::Infantry4 | Robot::Infantry5
                    ),
                    _ => false,
                })
            {
                self.spawner_config().unwrap().clone()
            } else if robot == Robot::Engineer
                && self.spawner_config().is_some_and(|c| c.climbing.is_some())
            {
                ChassisConfig::stair_heavy()
            } else if robot == Robot::Hero
                && self.spawner_config().is_some_and(|c| c.folding.is_some())
            {
                ChassisConfig::folding_hero()
            } else if self.spawner_config().is_some_and(|c| c.team_cad)
                && !config.mecanum
                && !config.balance_assist
                && !config.planar_flight
            {
                ChassisConfig::team_cad()
            } else {
                config
            },
        )
    }
    /// Admit a manually piloted role. UL §3.1.3 requires automatic Sentries;
    /// the host's optional optical controller does not implement autonomous firing.
    pub fn validate_pilot_robot(&self, robot: Robot) -> Result<(), String> {
        self.validate_roster(robot)?;
        if self.competition.rmul_format().is_some() && robot == Robot::Sentry {
            return Err(
                "RMUL Sentry must be fully autonomous; manual Sentry pilots are unavailable".into(),
            );
        }
        Ok(())
    }
    fn validate_rmul_slot(&self, team: Team, robot: Robot) -> Result<(), String> {
        if self.competition.rmul_format().is_some()
            && self
                .chassis_state
                .iter()
                .filter_map(|(id, state)| state.robot.as_ref().map(|robot| (id, robot)))
                .any(|(id, existing)| {
                    *existing == robot && self.field.chassis_team(*id) == Some(team)
                })
        {
            return Err("the team's RMUL equipment slot is occupied".into());
        }
        Ok(())
    }
    fn validate_roster(&self, robot: Robot) -> Result<(), String> {
        if crate::roster::supports_for(self.competition, robot) {
            Ok(())
        } else {
            Err(format!(
                "{} is not available in the {} roster",
                robot.name(),
                self.competition.id()
            ))
        }
    }
    fn spawn_in_slot(
        &mut self,
        team: Team,
        config: Option<ChassisConfig>,
        kind: RobotKind,
        robot: Option<Robot>,
    ) -> Result<u32, String> {
        let spawner = self
            .spawner
            .as_ref()
            .ok_or("this field offers no chassis")?;
        let config = config.unwrap_or_else(|| spawner.config.clone());
        let clear = SPAWN_SLOT_SPACING_M / 2.0;
        let limit = if self.competition.rmul_format().is_some() {
            3
        } else {
            1024
        };
        let placement = (0..limit)
            .map(|slot| {
                if self.competition.rmul_format().is_some() {
                    let (spawn, yaw) = self.competition.practice_spawn(team, slot);
                    spawner.at_with(config.clone(), kind, team, spawn, yaw)
                } else {
                    spawner.slot_with(config.clone(), kind, team, slot)
                }
            })
            .find(|placement| {
                let [x, y, _] = placement.spawn.translation_m;
                self.field
                    .chassis_positions_m()
                    .all(|[px, py, _]| (px - x).hypot(py - y) > clear)
            })
            .ok_or("no unoccupied spawn position is available")?;
        self.place(placement, robot)
    }
    /// Place a local pilot at a requested FLU position and heading in the
    /// spawner's default configuration, using the same ground lookup as
    /// joining network pilots. Like `spawn_chassis` it is no particular robot.
    pub fn spawn_chassis_at(
        &mut self,
        team: Team,
        spawn_m: [f64; 3],
        yaw_deg: f64,
    ) -> Result<u32, String> {
        let spawner = self
            .spawner
            .as_ref()
            .ok_or("this field offers no chassis")?;
        let placement = spawner.at(team, spawn_m, yaw_deg);
        self.place(placement, None)
    }
    /// Place a local pilot's `robot` at a requested FLU position and heading,
    /// using the same chassis preset and ground lookup as a joining network
    /// pilot who chose it.
    pub fn spawn_robot_at(
        &mut self,
        team: Team,
        robot: Robot,
        spawn_m: [f64; 3],
        yaw_deg: f64,
    ) -> Result<u32, String> {
        self.spawn_robot_at_with_chassis(
            team,
            robot,
            crate::protocol::Chassis::Auto,
            spawn_m,
            yaw_deg,
        )
    }
    /// Place a validated robot/drivetrain combination at an owner's spawn.
    pub fn spawn_robot_at_with_chassis(
        &mut self,
        team: Team,
        robot: Robot,
        chassis: crate::protocol::Chassis,
        spawn_m: [f64; 3],
        yaw_deg: f64,
    ) -> Result<u32, String> {
        self.validate_roster(robot)?;
        self.validate_rmul_slot(team, robot)?;
        let config = self.robot_config(robot, chassis)?;
        let spawner = self
            .spawner
            .as_ref()
            .ok_or("this field offers no chassis")?;
        let placement = spawner.at_with(config, robot.kind(), team, spawn_m, yaw_deg);
        self.place(placement, Some(robot))
    }
    /// Validate and create a staged pilot. Placement is a host policy: the
    /// selected base ring, a compatible drivetrain and an unoccupied position.
    /// Drone retains its existing aerial pad and tether instead of ground placement.
    pub(crate) fn deploy_robot(
        &mut self,
        team: Team,
        robot: Robot,
        chassis: crate::protocol::Chassis,
        position_m: [f64; 2],
    ) -> Result<u32, String> {
        self.validate_roster(robot)?;
        self.validate_rmul_slot(team, robot)?;
        let config = self.robot_config(robot, chassis)?;
        if !position_m.into_iter().all(f64::is_finite) {
            return Err("spawn position must be finite".into());
        }
        let (spawn, yaw) = if robot == Robot::Drone {
            let (spawn, yaw) = crate::layout::drone_slot(team, 0);
            if (spawn[0] - position_m[0]).hypot(spawn[1] - position_m[1]) > 0.001 {
                return Err("Drone must deploy at its aerial pad".into());
            }
            (spawn, yaw)
        } else {
            if !(if self.competition.rmul_format().is_some() {
                let [low, high] = self.competition.practice_bounds();
                position_m[0].abs() < high[0] - 0.4
                    && position_m[1] > low[1] + 0.4
                    && position_m[1] < high[1] - 0.4
                    && crate::layout::side_team(position_m[0]) == team
            } else {
                crate::deployment::contains(team, position_m)
            }) {
                return Err("choose a position inside your base ring, outside the platform".into());
            }
            (
                [position_m[0], position_m[1], 1.0],
                if team == Team::Red { 180.0 } else { 0.0 },
            )
        };
        let clearance = (config.body_half_m[0].hypot(config.body_half_m[1]) * 2.0).max(0.9);
        if self
            .field
            .chassis_positions_m()
            .any(|p| (p[0] - spawn[0]).hypot(p[1] - spawn[1]) < clearance)
        {
            return Err("that position is occupied; choose another point in the ring".into());
        }
        self.spawn_robot_at_with_chassis(team, robot, chassis, spawn, yaw)
    }

    fn place(
        &mut self,
        placement: rm_sim_world::ChassisPlacement,
        robot: Option<Robot>,
    ) -> Result<u32, String> {
        let id = self
            .field
            .add_chassis(&placement)
            .map_err(|e| e.to_string())?;
        let state = self.chassis_state.entry(id).or_default();
        state.spawn = Some(placement.spawn);
        state.robot = robot;
        Ok(id)
    }
    /// The robot a chassis was spawned as; `None` for a bot or a chassis
    /// spawned by configuration alone.
    pub fn robot(&self, chassis: u32) -> Option<Robot> {
        self.chassis_state
            .get(&chassis)
            .and_then(|state| state.robot)
    }
    /// The caliber a chassis' gun fires: its robot's, or the host default
    /// weapon's for a chassis that is no particular robot.
    pub fn caliber(&self, chassis: u32) -> rm_sim_world::Caliber {
        if self
            .field
            .chassis_config(chassis)
            .is_some_and(|c| c.folding.is_some())
            && self.robot(chassis) == Some(Robot::Engineer)
        {
            rm_sim_world::Caliber::Mm42
        } else {
            self.robot(chassis)
                .map_or(self.weapon.shot.caliber, Robot::caliber)
        }
    }
    /// The weapon a chassis fires with: the pilot's admitted settings, else
    /// the host defaults for its caliber (the Hero's for 42 mm), always at the
    /// chassis' own caliber.
    pub fn weapon_for(&self, chassis: u32) -> WeaponConfig {
        let caliber = self.caliber(chassis);
        let mut weapon = self
            .chassis_state
            .get(&chassis)
            .and_then(|state| state.weapon)
            .unwrap_or(match caliber {
                rm_sim_world::Caliber::Mm17 => self.weapon,
                rm_sim_world::Caliber::Mm42 => self.hero_weapon,
            });
        weapon.shot.caliber = caliber;
        weapon.shot.speed_m_s = weapon
            .shot
            .speed_m_s
            .min(self.limits_for_caliber(caliber).max_speed_m_s);
        weapon
    }

    /// Remove a chassis with its input stream, pending shots, results, bot mark
    /// and spawn record. A later spawn takes a fresh id.
    pub fn remove_chassis(&mut self, id: u32) -> Result<(), String> {
        self.external_control.remove(id);
        self.input_streams.remove(&id);
        self.cancel_pending_shots(Some(id), "shooter disconnected");
        self.chassis_state.remove(&id);
        self.bots.remove(&id);
        self.sentry_tracking.remove(id);
        if let Some(nav) = &mut self.navigation {
            nav.remove(id);
        }
        self.field.remove_chassis(id).map_err(|e| e.to_string())
    }
    /// A copy of the accepted shot journal, oldest first, at most 256 records.
    pub fn fire_records(&self) -> Vec<FireRecord> {
        self.fire_records.iter().cloned().collect()
    }
    /// The whole field snapshot.
    pub fn snapshot(&self) -> FieldSnapshot {
        self.field.snapshot()
    }

    /// Read the clock and optionally one robot's identity without cloning match,
    /// geometry, configuration or trajectory state. Unknown requested IDs fail.
    ///
    /// ```
    /// let (simulation, ids) = rm_sim_server::workload::simulation(1);
    /// let clock = simulation.control_status(None)?;
    /// assert!(clock.chassis.is_none());
    /// let robot = simulation.control_status(Some(ids[0]))?;
    /// assert_eq!(robot.time_ns, clock.time_ns);
    /// assert_eq!(robot.chassis.unwrap().id, ids[0]);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn control_status(&self, chassis: Option<u32>) -> Result<ControlStatus, &'static str> {
        let chassis = chassis
            .map(|id| {
                let body = self
                    .field
                    .chassis_control_state(id)
                    .ok_or("missing chassis")?;
                Ok::<_, &'static str>(ControlIdentity {
                    id,
                    placement_revision: body.placement_revision,
                    defeated: body.defeated,
                })
            })
            .transpose()?;
        Ok(ControlStatus {
            time_ns: self.field.time_ns(),
            input_epoch: self.input_epoch,
            paused: self.paused,
            chassis,
        })
    }

    /// The publication state: field, bots, epoch, pause flag and recent shot
    /// results. `snapshot_id` is left at zero for the host to stamp.
    pub fn state(&self) -> SimulationState {
        SimulationState {
            bots: self.bots.iter().copied().collect(),
            navigation: self
                .navigation
                .as_ref()
                .map_or_else(Vec::new, |nav| nav.views()),
            sentry_tracking: self.sentry_tracking.views(),
            snapshot_id: 0,
            input_epoch: self.input_epoch,
            shot_results: self
                .chassis_state
                .values()
                .flat_map(|state| state.shots.results.iter().rev().take(32).cloned())
                .collect(),
            paused: self.paused,
            field: self.snapshot(),
        }
    }
    /// Turn `delta_ns` of host time into ticks (capped at [`MAX_ADVANCE_NS`])
    /// and step them unless paused. Returns the ticks stepped.
    pub fn advance(&mut self, delta_ns: u64) -> Result<u64, FieldError> {
        self.advance_observed(delta_ns, &mut |_| {})
    }
    /// Advance host time while reporting contacts before their snapshot history expires.
    pub(crate) fn advance_observed(
        &mut self,
        delta_ns: u64,
        observer: &mut dyn FnMut(&rm_sim_world::ArmorHit),
    ) -> Result<u64, FieldError> {
        if let Some(service) = &mut self.assembly {
            service.poll(&mut self.field);
        }
        self.sensors.prune(&mut self.field);
        if self.paused {
            return Ok(0);
        }
        self.carry_ns += delta_ns.min(MAX_ADVANCE_NS);
        let ticks = self.carry_ns / tick_ns();
        self.carry_ns -= ticks * tick_ns();
        if ticks > 0 {
            self.step_observed(ticks, observer)?;
        }
        Ok(ticks)
    }
    /// Step exactly `ticks`, paused or not.
    pub fn step(&mut self, ticks: u64) -> Result<(), FieldError> {
        self.step_observed(ticks, &mut |_| {})
    }
    /// Keep input and shot scheduling on the ordinary ticks while streaming contacts.
    fn step_observed(
        &mut self,
        ticks: u64,
        observer: &mut dyn FnMut(&rm_sim_world::ArmorHit),
    ) -> Result<(), FieldError> {
        if let Some(service) = &mut self.assembly {
            service.poll(&mut self.field);
        }
        self.poll_navigation();
        if !self.sensors.active()
            && !self.external_control.any_active()
            && self.navigation.is_none()
            && !self.sentry_tracking.active()
            && self.input_streams.is_empty()
            && self.pending_shots.is_empty()
        {
            return self.field.step_with_hits(ticks, observer);
        }
        for _ in 0..ticks {
            let now = self.field.time_ns();
            self.input_streams.retain(|id, stream| {
                let valid = stream.belongs_to(self.field.chassis_revision(*id))
                    && self.field.chassis_defeated(*id) == Some(false);
                if !valid && let Some(neutral) = stream.cancel() {
                    let _ = self.field.command_chassis(*id, neutral);
                }
                valid
            });
            for (id, stream) in &mut self.input_streams {
                if let Some(neutral) = stream.advance(now) {
                    self.external_control.revoke(*id);
                    let _ = self.field.command_chassis(*id, neutral);
                }
            }
            if let Some(nav) = &mut self.navigation {
                nav.tick(&mut self.field);
            }
            let navigation = self
                .navigation
                .as_ref()
                .map_or_else(Vec::new, |n| n.views());
            self.sentry_tracking.tick(&mut self.field, &navigation);
            self.external_control.tick(&mut self.field);
            self.execute_pending_shots();
            self.field.step_with_hits(1, observer)?;
            self.sensors.capture(&mut self.field);
        }
        Ok(())
    }
    /// Pilot recovery is requested explicitly outside a match; respawning
    /// changes health only.
    fn recover_pilot(&mut self, id: u32, reset: bool) -> Result<(), String> {
        let spawn = self
            .chassis_state
            .get(&id)
            .and_then(|state| state.spawn)
            .ok_or("no pilot with that chassis")?;
        if !reset && self.field.chassis_defeated(id) != Some(true) {
            return Err("robot is not defeated".into());
        }
        if self.field.rmul_2027().is_some_and(|game| {
            matches!(
                game.phase,
                rm_sim_world::gameplay::rmul_2027::Phase::Countdown
                    | rm_sim_world::gameplay::rmul_2027::Phase::Running
            )
        }) {
            return Err("pilot reset/recovery is unavailable during an RMUL round".into());
        }
        if self.field.rmuc_2027().is_some_and(|s| {
            matches!(
                s.game.phase,
                rm_sim_world::rmuc_2027::match_rules::Phase::Countdown
                    | rm_sim_world::rmuc_2027::match_rules::Phase::Running
            )
        }) {
            return Err(
                "robots recover on the 2027 match's respawn timer; use a paid respawn request"
                    .into(),
            );
        }
        self.revoke_external_control(id);
        // A no-referee practice robot can still be repositioned through debug.
        if self.field.rmuc_2027().is_some() || self.field.rmul_2027().is_some() {
            self.field
                .referee_command(rm_sim_world::RefereeCommand::ReviveRobot { robot: id })
                .map_err(|e| e.to_string())?;
        }
        if let Some(mut stream) = self.input_streams.remove(&id)
            && let Some(neutral) = stream.cancel()
        {
            let _ = self.field.command_chassis(id, neutral);
        }
        if reset {
            self.field
                .place_chassis(id, spawn)
                .map_err(|e| e.to_string())?;
        }
        Ok(())
    }
    /// The recorded result of one shot, while the shooter's last 128 results
    /// still hold it.
    pub fn shot_result(&self, shooter: u32, shot_id: u64) -> Option<&crate::protocol::ShotResult> {
        self.chassis_state
            .get(&shooter)?
            .shots
            .results
            .iter()
            .find(|result| result.shot_id == shot_id)
    }
    fn validate_shot(
        &self,
        shooter: u32,
        shot_id: u64,
        input: crate::input_stream::InputFrame,
    ) -> Result<(), String> {
        if shot_id == 0
            || self.chassis_state.get(&shooter).is_some_and(|state| {
                state.shots.last_id != 0 && shot_id <= state.shots.last_id.saturating_sub(128)
            })
        {
            return Err("shot id is stale".into());
        }
        let revision = self
            .field
            .chassis_revision(shooter)
            .ok_or("no chassis with that id")?;
        let now = self.field.time_ns();
        if self.paused
            || self.field.chassis_defeated(shooter) != Some(false)
            || input.input_epoch != self.input_epoch
            || input.placement_revision != revision
            || !input.command.is_finite()
            || input.sequence == 0
            || now.saturating_sub(input.sampled_time_ns) > 250_000_000
            || input.sampled_time_ns > now.saturating_add(200_000_000)
        {
            return Err("shot expired or belongs to another life".into());
        }
        Ok(())
    }
    fn remember_shot(
        &mut self,
        shooter: u32,
        shot_id: u64,
        result: Result<u64, String>,
        notify: bool,
    ) {
        let shots = &mut self.chassis_state.entry(shooter).or_default().shots;
        shots.last_id = shots.last_id.max(shot_id);
        let launch_speed_m_s = result
            .as_ref()
            .ok()
            .and_then(|id| {
                self.fire_records
                    .iter()
                    .find(|record| record.projectile_id == *id)
            })
            .map(|record| record.launch_speed_m_s);
        let result = crate::protocol::ShotResult {
            launch_speed_m_s,
            executed_time_ns: result.as_ref().ok().map(|_| self.field.time_ns()),
            shooter,
            shot_id,
            result,
        };
        let history = &mut self.chassis_state.entry(shooter).or_default().shots.results;
        if history.len() == 128 {
            history.pop_front();
        }
        history.push_back(result.clone());
        if notify {
            // At most 32 pending shots per live chassis can complete in one advance.
            if self
                .completed_shots
                .iter()
                .filter(|r| r.shooter == shooter)
                .count()
                >= 128
                && let Some(index) = self
                    .completed_shots
                    .iter()
                    .position(|r| r.shooter == shooter)
            {
                self.completed_shots.remove(index);
            }
            self.completed_shots.push_back(result);
        }
    }
    /// Remove and return the shot results completed since the last call, in
    /// completion order.
    pub(crate) fn take_completed_shots(&mut self) -> VecDeque<crate::protocol::ShotResult> {
        std::mem::take(&mut self.completed_shots)
    }
    /// Whether a shot with that id and shooter is still queued.
    pub(crate) fn is_shot_scheduled(&self, shooter: u32, shot_id: u64) -> bool {
        self.pending_shots.iter().any(|c| {
            matches!(c, Command::FireAimed { shooter: owner, shot_id: id, .. }
                if *owner == shooter && *id == shot_id)
        })
    }
    fn cancel_pending_shots(&mut self, shooter: Option<u32>, reason: &str) {
        let pending = std::mem::take(&mut self.pending_shots);
        for command in pending {
            if let Command::FireAimed {
                shooter: owner,
                shot_id,
                ..
            } = command
            {
                if shooter.is_none_or(|id| id == owner) {
                    self.remember_shot(owner, shot_id, Err(reason.into()), true);
                } else {
                    self.pending_shots.push(command);
                }
            }
        }
    }
    fn execute_pending_shots(&mut self) {
        let mut index = 0;
        while index < self.pending_shots.len() {
            let Command::FireAimed {
                shooter,
                shot_id,
                input,
            } = self.pending_shots[index]
            else {
                unreachable!()
            };
            let result = match self.validate_shot(shooter, shot_id, input) {
                Err(reason) => Err(reason),
                Ok(()) if input.sampled_time_ns <= self.field.time_ns() => {
                    self.fire_aimed(shooter, shot_id, input)
                }
                Ok(()) => {
                    index += 1;
                    continue;
                }
            };
            self.pending_shots.remove(index);
            self.remember_shot(shooter, shot_id, result, true);
        }
    }
    fn fire_aimed(
        &mut self,
        shooter: u32,
        shot_id: u64,
        input: crate::input_stream::InputFrame,
    ) -> Result<u64, String> {
        self.validate_shot(shooter, shot_id, input)?;
        let state = self
            .field
            .chassis_control_state(shooter)
            .ok_or("no chassis with that id")?;
        // Only the shot's exact aim is applied; held movement and its lease are untouched.
        let aim = rm_sim_world::ChassisCommand {
            aim_yaw_rad: input.command.aim_yaw_rad,
            aim_pitch_rad: input.command.aim_pitch_rad,
            ..state.command
        };
        self.field
            .command_chassis(shooter, aim)
            .map_err(|e| e.to_string())?;
        let result = self.fire_now(shooter, input.sampled_time_ns);
        let _ = self.field.command_chassis(shooter, state.command);
        let projectile_id = result?;
        let shots = &mut self.chassis_state.entry(shooter).or_default().shots;
        shots.last_id = shots.last_id.max(shot_id);
        Ok(projectile_id)
    }
    /// Fire the shooter's gun at the current tick. The gun's rate is enforced
    /// on `intended_ns`, the pilot's timeline: no two shots may be intended
    /// within one interval of each other. A scheduled shot that arrived late,
    /// or behind a newer one, still fires, but it neither pushes the cooldown
    /// onto the correctly spaced shot behind it nor forms a burst.
    fn fire_now(&mut self, shooter: u32, intended_ns: u64) -> Result<u64, String> {
        if matches!(self.robot(shooter), Some(Robot::Engineer))
            && self.caliber(shooter) != rm_sim_world::Caliber::Mm42
        {
            return Err("this robot has no launcher".into());
        }
        let weapon = self.weapon_for(shooter);
        let fired = &mut self
            .chassis_state
            .entry(shooter)
            .or_default()
            .shots
            .intended_ns;
        if fired
            .iter()
            .any(|&t| t.abs_diff(intended_ns) < weapon.interval_ns)
        {
            return Err("weapon is cooling down".into());
        }
        let muzzle = self
            .field
            .chassis_muzzle_pose(shooter)
            .ok_or("no chassis with that id")?;
        let muzzle = weapon.spread.apply(muzzle, shooter, intended_ns);
        let shot = weapon.sample_shot(
            shooter,
            intended_ns,
            self.weapon_limits_for(shooter).max_speed_m_s,
        );
        let projectile_id = self
            .field
            .fire(muzzle, shot, Some(shooter))
            .map_err(|e| e.to_string())?;
        if self.fire_records.len() == 256 {
            self.fire_records.pop_front();
        }
        self.fire_records.push_back(FireRecord {
            launch_speed_m_s: shot.speed_m_s,
            shooter,
            projectile_id,
            accepted_time_ns: self.field.time_ns(),
            authoritative_muzzle_pose: muzzle,
        });
        let fired = &mut self
            .chassis_state
            .entry(shooter)
            .or_default()
            .shots
            .intended_ns;
        if fired.len() == 32 {
            fired.pop_front();
        }
        fired.push_back(intended_ns);
        Ok(projectile_id)
    }
    /// Apply a player's or operator's command at the current tick.
    pub fn apply(&mut self, command: &Command) -> Result<(), String> {
        self.apply_observed(command, &mut |_| {})
    }
    /// Apply a command, streaming contacts from explicit steps to the host.
    pub(crate) fn apply_observed(
        &mut self,
        command: &Command,
        observer: &mut dyn FnMut(&rm_sim_world::ArmorHit),
    ) -> Result<(), String> {
        match command {
            Command::ExternalDrive { chassis, input } => {
                if self.paused || input.input_epoch != self.input_epoch {
                    return Err("external input requires an unpaused current epoch".into());
                }
                let body = self
                    .field
                    .chassis_control_state(*chassis)
                    .ok_or("missing chassis")?;
                if body.placement_revision != input.placement_revision || body.defeated {
                    return Err("external input belongs to an unavailable chassis life".into());
                }
                self.external_control
                    .receive(*chassis, *input, self.field.time_ns())?;
                // Cancel held/pending pilot motion but retain sequence evidence, so
                // a replay cannot retake control after an explicit external claim.
                if let Some(stream) = self.input_streams.get_mut(chassis) {
                    stream.supersede();
                }
                if let Some(nav) = &mut self.navigation {
                    nav.stop(*chassis);
                }
                if self.robot(*chassis) == Some(Robot::Sentry) {
                    self.sentry_tracking.set(*chassis, false);
                }
                let command = input.motion.resolve(body.pose, body.command)?;
                self.field
                    .command_chassis(*chassis, command)
                    .map_err(|e| e.to_string())
            }
            Command::ReleaseExternalDrive { chassis, lease_id } => {
                self.external_control.release(*chassis, *lease_id)?;
                crate::navigation::brake(&mut self.field, *chassis)
            }
            Command::SpawnSentry {
                team,
                position_m,
                yaw_deg,
            } => self.spawn_sentry(*team, *position_m, *yaw_deg),
            Command::NavigateSentry {
                chassis,
                position_m,
            } => {
                crate::protocol::check_map_marker(
                    crate::protocol::MapMarkerKind::Alert,
                    *position_m,
                )?;
                self.navigation
                    .as_mut()
                    .ok_or("no navigation Sentry")?
                    .goal(*chassis, *position_m)?;
                self.revoke_external_control(*chassis);
                crate::navigation::brake(&mut self.field, *chassis)
            }
            Command::StopSentry { chassis } => {
                let nav = self
                    .navigation
                    .as_mut()
                    .filter(|n| n.owns(*chassis))
                    .ok_or("not a navigation Sentry")?;
                nav.stop(*chassis);
                self.revoke_external_control(*chassis);
                self.sentry_tracking.set(*chassis, false);
                crate::navigation::brake(&mut self.field, *chassis)
            }
            Command::MapInteraction(_) => {
                Err("map interactions require a connected team pilot".into())
            }
            Command::SetSentryAutoAim { chassis, enabled } => {
                if *enabled && self.external_control.active(*chassis) {
                    return Err("release external control before enabling chassis sweep".into());
                }
                if self.robot(*chassis) != Some(Robot::Sentry) {
                    return Err("auto aim requires a Sentry".into());
                }
                self.sentry_tracking.set(*chassis, *enabled);
                if !enabled && let Some(body) = self.field.chassis_snapshot(*chassis) {
                    let mut command = body.command;
                    command.yaw_rate_rad_s = 0.;
                    command.aim_yaw_rad = body.held_aim_rad[0];
                    command.aim_pitch_rad = body.held_aim_rad[1];
                    self.field
                        .command_chassis(*chassis, command)
                        .map_err(|e| e.to_string())?;
                }
                Ok(())
            }
            Command::ConvertHeavy { chassis } => {
                self.field
                    .convert_heavy_to_hero(*chassis)
                    .map_err(|e| e.to_string())?;
                if let Some(state) = self.chassis_state.get_mut(chassis) {
                    state.weapon = None;
                }
                self.input_streams.remove(chassis);
                self.cancel_pending_shots(Some(*chassis), "Heavy converted");
                Ok(())
            }
            Command::EnergyGrip { chassis } => self
                .field
                .toggle_energy_grip(*chassis)
                .map_err(|e| e.to_string()),
            Command::EnergyArmJog {
                chassis,
                translation_m,
                rotation_rad,
            } => self
                .assembly
                .as_ref()
                .ok_or("No source assembly assets installed")?
                .jog(&mut self.field, *chassis, *translation_m, *rotation_rad),
            Command::AutoAssembly {
                chassis,
                difficulty,
            } => self.start_auto_assembly(*chassis, *difficulty),
            Command::CancelAutoAssembly { chassis } => {
                self.field.cancel_assembly(*chassis, "Cancelled by pilot");
                Ok(())
            }
            Command::Deploy { .. } => Err("deployment requires a connected peer".into()),
            Command::ConfigureWeapon { chassis, weapon } => {
                if self.field.chassis_muzzle_pose(*chassis).is_none() {
                    return Err("no chassis with that id".into());
                }
                let weapon = self
                    .weapon_limits_for(*chassis)
                    .admit(self.caliber(*chassis), *weapon)
                    .map_err(str::to_string)?;
                self.chassis_state.entry(*chassis).or_default().weapon = Some(weapon);
                Ok(())
            }
            Command::FireAimed {
                shooter,
                shot_id,
                input,
            } => {
                if let Some(previous) = self.shot_result(*shooter, *shot_id) {
                    return previous.result.as_ref().map(|_| ()).map_err(Clone::clone);
                }
                if let Some(previous) = self.pending_shots.iter().find(|c| matches!(c,
                    Command::FireAimed { shooter: owner, shot_id: id, .. } if owner == shooter && id == shot_id)) {
                    return if previous == command { Ok(()) } else { Err("shot id changed contents".into()) };
                }
                let admission = self.validate_shot(*shooter, *shot_id, *input);
                if let Err(reason) = admission {
                    self.remember_shot(*shooter, *shot_id, Err(reason.clone()), false);
                    return Err(reason);
                }
                if input.sampled_time_ns > self.field.time_ns() {
                    if self.pending_shots.iter().filter(|c| matches!(c, Command::FireAimed { shooter: owner, .. } if owner == shooter)).count() >= 32 {
                        let reason = "shot schedule full".to_string();
                        self.remember_shot(*shooter, *shot_id, Err(reason.clone()), false);
                        return Err(reason);
                    }
                    self.pending_shots.push(*command);
                    self.pending_shots.sort_by_key(|c| match c {
                        Command::FireAimed {
                            input,
                            shooter,
                            shot_id,
                            ..
                        } => (input.sampled_time_ns, *shooter, *shot_id),
                        _ => unreachable!(),
                    });
                    return Ok(());
                }
                let result = self.fire_aimed(*shooter, *shot_id, *input);
                self.remember_shot(*shooter, *shot_id, result.clone(), false);
                result.map(|_| ())
            }
            Command::PilotInput { chassis, frame } => {
                if frame.input_epoch != self.input_epoch {
                    return Err("input epoch expired".into());
                }
                let revision = self
                    .field
                    .chassis_revision(*chassis)
                    .ok_or("no chassis with that id")?;
                let now = self.field.time_ns();
                let stream = self.input_streams.entry(*chassis).or_default();
                let previous_received = stream.telemetry.received_sequence;
                let accepted = stream
                    .receive(*frame, now, revision)
                    .map_err(str::to_string)?;
                let fresh = stream.telemetry.received_sequence > previous_received;
                // Manual ownership changes on admission, even when the actual
                // transition is scheduled for a future tick.
                if fresh {
                    self.revoke_external_control(*chassis);
                }
                if let Some(command) = accepted {
                    self.field
                        .command_chassis(*chassis, command)
                        .map_err(|e| e.to_string())?;
                }
                Ok(())
            }
            Command::PlaceChassis {
                chassis,
                position_m,
                yaw_deg,
            }
            | Command::PlaceBot {
                chassis,
                position_m,
                yaw_deg,
            } => {
                if matches!(command, Command::PlaceBot { .. }) && !self.bots.contains(chassis) {
                    return Err("not a training bot".into());
                }
                if !position_m.iter().all(|v| v.is_finite()) || !yaw_deg.is_finite() {
                    return Err("placement must be finite".into());
                }
                let team = self
                    .field
                    .chassis_team(*chassis)
                    .ok_or("no chassis with that id")?;
                let spawner = self
                    .spawner
                    .as_ref()
                    .ok_or("this field offers no chassis")?;
                let placement = spawner.at(team, *position_m, *yaw_deg);
                self.revoke_external_control(*chassis);
                self.input_streams.remove(chassis);
                if let Some(nav) = &mut self.navigation {
                    nav.stop(*chassis);
                }
                self.field
                    .place_chassis(*chassis, placement.spawn)
                    .map_err(|e| e.to_string())
            }
            Command::Chassis { chassis, command } => {
                if !command.is_finite() {
                    return Err("invalid chassis command".into());
                }
                self.revoke_external_control(*chassis);
                self.field
                    .command_chassis(*chassis, *command)
                    .map_err(|e| e.to_string())
            }
            Command::SpawnBot { team, spin_rad_s } => self.spawn_bot(*team, *spin_rad_s),
            Command::RemoveBot { chassis } => {
                if !self.bots.contains(chassis) {
                    return Err("not a training bot".into());
                }
                self.remove_chassis(*chassis)
            }
            Command::ActivateRune { team } => self
                .field
                .referee_command(rm_sim_world::RefereeCommand::ActivateRune { team: *team })
                .map_err(|e| e.to_string()),
            Command::ClearBots => {
                for id in self.bots.clone() {
                    self.remove_chassis(id)?;
                }
                Ok(())
            }
            Command::Respawn { chassis } => self.recover_pilot(*chassis, false),
            Command::ResetRobot { chassis } => self.recover_pilot(*chassis, true),
            Command::BuyAmmo { chassis, caliber } => self
                .field
                .buy_ammo(*chassis, *caliber)
                .map_err(|e| e.to_string()),
            Command::SetSentryMode { chassis, mode } => self
                .field
                .referee_command(rm_sim_world::RefereeCommand::SetSentryMode {
                    robot: *chassis,
                    mode: *mode,
                })
                .map_err(|e| e.to_string()),
            Command::SentryAction { chassis, action } => self
                .field
                .referee_command(rm_sim_world::RefereeCommand::SentryAction {
                    robot: *chassis,
                    action: *action,
                    commander: false,
                })
                .map_err(|e| e.to_string()),
            Command::BuyHp { chassis } => self
                .field
                .referee_command(rm_sim_world::RefereeCommand::BuyHp { robot: *chassis })
                .map_err(|e| e.to_string()),
            Command::InstantRespawn { chassis } => self
                .field
                .referee_command(rm_sim_world::RefereeCommand::InstantRespawn { robot: *chassis })
                .map_err(|e| e.to_string()),
            Command::SetPerformance {
                performance: rm_sim_world::gameplay::Performance::Fixed(_),
                ..
            } => Err("pilots choose a Hero or Infantry performance type".into()),
            Command::SetPerformance {
                chassis,
                performance,
            } => self
                .field
                .referee_command(rm_sim_world::RefereeCommand::SetPerformance {
                    robot: *chassis,
                    performance: *performance,
                })
                .map_err(|e| e.to_string()),
            Command::Fire { shooter } => {
                let now = self.field.time_ns();
                self.fire_now(*shooter, now).map(|_| ())
            }
            Command::SpawnProjectile { muzzle, shot } => self
                .field
                .fire(*muzzle, *shot, None)
                .map(|_| ())
                .map_err(|e| e.to_string()),
            Command::Referee(command) => {
                if matches!(command, rm_sim_world::RefereeCommand::StartMatch)
                    && self.field.rmuc_2027().is_some()
                {
                    use std::hash::{BuildHasher, Hasher};
                    let random = std::collections::hash_map::RandomState::new()
                        .build_hasher()
                        .finish();
                    self.field.set_rmuc_2027_rune_seed(random);
                    self.field
                        .set_rmuc_2027_rotor_direction(if random & 1 == 0 { -1 } else { 1 });
                }
                self.field
                    .referee_command(*command)
                    .map_err(|e| e.to_string())?;
                Ok(())
            }
            Command::Pause { paused } => {
                self.set_paused(*paused);
                Ok(())
            }
            Command::Step { ticks } => {
                if *ticks == 0 || *ticks > 60_000 {
                    return Err("step between 1 and 60000 ticks".into());
                }
                self.step_observed(*ticks, observer)
                    .map_err(|e| e.to_string())
            }
            // Markers go to the host's record; the world is unchanged.
            Command::MapMarker { kind, position_m } => {
                crate::protocol::check_map_marker(*kind, *position_m).map_err(Into::into)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rm_sim_world::{FieldConfig, RefereeCommand};

    #[test]
    fn chassis_removal_clears_shot_state_without_reordering_other_results() {
        let (mut sim, first) = pilot_simulation();
        let second = sim.spawn_chassis(Team::Blue).unwrap();
        sim.remember_shot(second, 4, Err("second rejection".into()), false);
        sim.remember_shot(first, 2, Err("first rejection".into()), false);
        sim.remember_shot(first, 3, Err("later rejection".into()), false);
        let ids: Vec<_> = sim
            .state()
            .shot_results
            .iter()
            .map(|s| (s.shooter, s.shot_id))
            .collect();
        assert_eq!(ids, vec![(first, 3), (first, 2), (second, 4)]);
        sim.remove_chassis(first).unwrap();
        assert!(sim.shot_result(first, 3).is_none());
        assert!(sim.shot_result(second, 4).is_some());
        assert_eq!(sim.state().shot_results.len(), 1);
    }

    #[test]
    fn rmul_practice_admits_only_its_roster_and_bounds_spawns() {
        use crate::host_args::Competition;
        for referee in [false, true] {
            let mut sim = Simulation::rmul_practice(
                Competition::RmulInfantry2027,
                referee,
                Some(ChassisConfig::default()),
                false,
            )
            .unwrap();
            assert!(sim.validate_pilot_robot(Robot::Sentry).is_err());
            let id = sim.spawn_robot(Team::Red, Robot::Infantry3).unwrap();
            let p = sim
                .snapshot()
                .chassis
                .iter()
                .find(|c| c.id == id)
                .unwrap()
                .pose
                .translation_m;
            assert!(p[0].abs() < 2.5 && p[1].abs() < 2.5);
            assert!(sim.spawn_robot(Team::Red, Robot::Infantry3).is_err());
            assert!(sim.spawn_robot(Team::Blue, Robot::Engineer).is_err());
            assert!(sim.spawn_robot(Team::Blue, Robot::Infantry3).is_ok());
            assert!(sim.snapshot().rmuc_2027.is_none());
            assert_eq!(sim.snapshot().rmul_2027.is_some(), referee);
        }
    }

    fn cad_without_files() -> CadAssets {
        let asset = |placements| crate::cad_assets::CadAsset {
            collision_shapes: None,
            semantics: None,
            file: "missing-builder-test.glb".into(),
            placements,
            collision: None,
            source_tessellated_collision: false,
            bake_motion_endpoints: false,
        };
        CadAssets {
            terrain_collision: None,
            rmuc_2027: None,
            season: rm_sim_world::season::Season::Rm2026,
            rest_pose_only: false,
            root: std::path::PathBuf::from("missing-builder-test-package"),
            floor_top_cad_m: crate::cad_assets::FLOOR_TOP_CAD_M,
            collision_solids: false,
            floor: asset(Vec::new()),
            arena_static: asset(Vec::new()),
            rune: asset(vec![rm_sim_world::Pose::at([5.0, 0.0, 2.0])]),
            outpost: asset(vec![rm_sim_world::Pose::at([3.0, 0.0, 0.0])]),
            base: asset(Vec::new()),
            tech_core: asset(Vec::new()),
            static_assets: Vec::new(),
        }
    }
    fn build_options() -> LayoutOptions {
        LayoutOptions {
            rune: Some(rm_sim_world::RuneKind::Big),
            outpost_speed_rad_s: 1.0,
            terrain: false,
            referee: true,
            projectile_policy: Default::default(),
        }
    }

    #[test]
    fn runtime_admission_rejects_archived_fields_before_geometry() {
        for referee in [false, true] {
            let mut options = build_options();
            options.referee = referee;
            options.terrain = true;
            let mut events = Vec::new();
            let error =
                Simulation::from_cad(&cad_without_files(), &options, None, false, |event| {
                    events.push(event)
                })
                .err()
                .expect("2026 must be archived");
            assert!(error.to_string().contains("rm2026-final"));
            assert!(events.is_empty());
        }
    }

    #[test]
    fn runtime_admission_requires_bindings_for_matches_but_allows_bare_practice() {
        let mut cad = cad_without_files();
        cad.season = rm_sim_world::season::Season::CURRENT;
        cad.rest_pose_only = true;
        let mut options = build_options();
        let mut events = Vec::new();
        let error = Simulation::from_cad(&cad, &options, None, false, |event| events.push(event))
            .err()
            .expect("matches need native bindings");
        assert!(error.to_string().contains("bound field package"));
        assert!(events.is_empty());
        options.referee = false;
        let simulation = Simulation::from_cad(&cad, &options, None, false, |_| {}).unwrap();
        let snapshot = simulation.snapshot();
        assert!(snapshot.referee.is_none());
        assert!(snapshot.runes.is_empty() && snapshot.outposts.is_empty());
    }

    #[test]
    fn rest_pose_cad_has_no_old_referee_targets_or_zones() {
        let mut cad = cad_without_files();
        cad.season = rm_sim_world::season::Season::Rm2027;
        cad.rest_pose_only = true;
        let options = build_options(); // Explicitly asks for old targets and referee.
        let config = field_config(&cad, &options);
        assert!(config.runes.is_empty() && config.outposts.is_empty());
        assert!(config.bases.is_empty() && config.zones.is_empty());
        assert!(config.referee.is_none());
        let mut simulation = Simulation::build_cad(&cad, &options, None, true, |_| {}).unwrap();
        let snapshot = simulation.snapshot();
        assert!(snapshot.runes.is_empty() && snapshot.outposts.is_empty());
        assert!(snapshot.referee.is_none());
        assert!(
            simulation
                .apply(&Command::Referee(RefereeCommand::StartMatch))
                .is_err()
        );
    }

    #[test]
    fn placement_preserves_identity_and_hp_and_clears_velocity() {
        let mut simulation = Simulation::build_cad(
            &cad_without_files(),
            &build_options(),
            Some(ChassisConfig::default()),
            true,
            |_| {},
        )
        .unwrap();
        let id = simulation.spawn_chassis(Team::Red).unwrap();
        simulation
            .apply(&Command::Chassis {
                chassis: id,
                command: rm_sim_world::ChassisCommand {
                    forward_m_s: 2.0,
                    ..Default::default()
                },
            })
            .unwrap();
        simulation.step(100).unwrap();
        let robots = simulation.snapshot().rmuc_2027.map(|s| s.game.robots);
        simulation
            .apply(&Command::PlaceChassis {
                chassis: id,
                position_m: [2.0, 3.0, 10.0],
                yaw_deg: 90.0,
            })
            .unwrap();
        let snapshot = simulation.snapshot();
        assert_eq!(snapshot.rmuc_2027.map(|s| s.game.robots), robots);
        assert_eq!(snapshot.chassis.len(), 1);
        let chassis = &snapshot.chassis[0];
        assert_eq!(chassis.id, id);
        assert_eq!(
            chassis.pose.translation_m,
            [2.0, 3.0, chassis.config.rest_height_m() + 0.01]
        );
        assert_eq!(chassis.velocity_m_s, [0.0; 3]);
        assert_eq!(chassis.angular_velocity_rad_s, [0.0; 3]);
        assert!(chassis.wheels.iter().all(|w| w.contact.is_none()));
        assert!(
            simulation
                .apply(&Command::PlaceChassis {
                    chassis: id,
                    position_m: [f64::NAN, 0.0, 0.0],
                    yaw_deg: 0.0
                })
                .is_err()
        );
    }
    #[test]
    fn cad_construction_preserves_pause_and_rules_without_creating_a_pilot() {
        let mut events = Vec::new();
        let config = ChassisConfig::default();
        let mut simulation = Simulation::build_cad(
            &cad_without_files(),
            &build_options(),
            Some(config.clone()),
            true,
            |event| events.push(event),
        )
        .unwrap();
        // No terrain files exist: disabling collision must skip their loading.
        assert_eq!(
            events,
            [
                BuildProgress::BuildingPhysics,
                BuildProgress::PreparingChassis
            ]
        );
        assert!(simulation.paused());
        assert_eq!(simulation.advance(1_000_000).unwrap(), 0);
        let snapshot = simulation.snapshot();
        assert!(snapshot.chassis.is_empty());
        assert!(snapshot.runes.is_empty());
        assert!(snapshot.outposts.is_empty());
        assert!(snapshot.referee.is_none());
        assert!(snapshot.rmuc_2027.is_none());

        let local = simulation
            .spawn_chassis_at(Team::Blue, [2.0, 3.0, 10.0], 90.0)
            .unwrap();
        let peer = simulation.spawn_chassis(Team::Red).unwrap();
        let snapshot = simulation.snapshot();
        let chassis = snapshot.chassis.iter().find(|c| c.id == local).unwrap();
        assert_eq!(chassis.team, Team::Blue);
        assert_eq!(
            chassis.pose.translation_m,
            [2.0, 3.0, config.rest_height_m() + 0.01]
        );
        assert!((chassis.pose.rotation_wxyz[0] - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-9);
        assert!((chassis.pose.rotation_wxyz[3] - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-9);
        assert_eq!(chassis.config, config);
        assert!(
            snapshot
                .chassis
                .iter()
                .any(|c| c.id == peer && c.team == Team::Red)
        );
    }

    #[test]
    fn cad_construction_can_disable_chassis_runes_and_referee() {
        let mut options = build_options();
        options.rune = None;
        options.referee = false;
        let mut simulation =
            Simulation::build_cad(&cad_without_files(), &options, None, false, |_| {}).unwrap();
        assert!(!simulation.paused());
        assert!(simulation.spawner_config().is_none());
        assert!(simulation.spawn_chassis(Team::Red).is_err());
        assert!(
            simulation
                .spawn_chassis_at(Team::Red, [0.0; 3], 0.0)
                .is_err()
        );
        let snapshot = simulation.snapshot();
        assert!(snapshot.runes.is_empty());
        assert!(snapshot.referee.is_none());
        assert!(snapshot.chassis.is_empty());
    }

    #[test]
    fn cad_construction_propagates_terrain_and_physics_failures() {
        let mut options = build_options();
        options.terrain = true;
        let mut events = Vec::new();
        let result = Simulation::build_cad(&cad_without_files(), &options, None, false, |event| {
            events.push(event)
        });
        assert!(result.is_err());
        assert_eq!(events, [BuildProgress::ReadingTerrain]);
    }

    fn simulation() -> Simulation {
        let config = crate::workload::field_config();
        Simulation::new(Field::new(&config).unwrap(), false)
            .with_weapon(WeaponConfig {
                interval_ns: 100_000_000,
                speed_variation_m_s: 0.,
                spread: Default::default(),
                ..Default::default()
            })
            .unwrap()
    }
    fn pilot_simulation() -> (Simulation, u32) {
        let mut simulation = simulation().with_spawner(ChassisSpawner {
            config: ChassisConfig::default(),
            terrain: None,
        });
        let id = simulation.spawn_chassis(Team::Red).unwrap();
        (simulation, id)
    }

    fn external_input(sim: &Simulation, id: u32, lease_id: u64, sequence: u64) -> Command {
        Command::ExternalDrive {
            chassis: id,
            input: crate::external_control::Input {
                lease_id,
                sequence,
                sampled_time_ns: sim.field.time_ns(),
                input_epoch: sim.input_epoch,
                placement_revision: sim.field.chassis_revision(id).unwrap(),
                motion: crate::external_control::Motion::HeadingVector {
                    direction_world: [0., 1.],
                    speed_m_s: 1.,
                },
            },
        }
    }
    #[test]
    fn external_drive_expires_preserves_channels_and_requires_fresh_lease() {
        let (mut sim, id) = pilot_simulation();
        sim.apply(&Command::Chassis {
            chassis: id,
            command: rm_sim_world::ChassisCommand {
                folded: true,
                aim_yaw_rad: 0.8,
                aim_pitch_rad: 0.2,
                ..Default::default()
            },
        })
        .unwrap();
        let input = external_input(&sim, id, 1, 1);
        sim.apply(&input).unwrap();
        sim.step(16).unwrap();
        assert!(sim.external_control.active(id));
        assert!(sim.apply(&input).is_err());
        sim.step(17).unwrap();
        assert!(!sim.external_control.active(id));
        let body = sim.field.chassis_snapshot(id).unwrap();
        assert_eq!(
            (
                body.command.forward_m_s,
                body.command.left_m_s,
                body.command.yaw_rate_rad_s
            ),
            (0., 0., 0.)
        );
        assert!(body.command.folded);
        assert_eq!(
            (body.command.aim_yaw_rad, body.command.aim_pitch_rad),
            (0.8, 0.2)
        );
        assert!(sim.apply(&external_input(&sim, id, 1, 2)).is_err());
        sim.apply(&external_input(&sim, id, 2, 1)).unwrap();
        sim.apply(&Command::Chassis {
            chassis: id,
            command: Default::default(),
        })
        .unwrap();
        assert!(!sim.external_control.active(id));
        assert!(sim.apply(&external_input(&sim, id, 2, 2)).is_err());
    }
    #[test]
    fn rejected_update_at_expiry_still_brakes_and_scheduled_manual_input_takes_over() {
        let (mut sim, id) = pilot_simulation();
        sim.apply(&external_input(&sim, id, 1, 1)).unwrap();
        sim.step(32).unwrap();
        assert!(sim.apply(&external_input(&sim, id, 1, 2)).is_err());
        sim.step(1).unwrap();
        assert_eq!(
            sim.field.chassis_snapshot(id).unwrap().command.forward_m_s,
            0.
        );
        assert_eq!(sim.field.chassis_snapshot(id).unwrap().command.left_m_s, 0.);
        sim.apply(&external_input(&sim, id, 2, 1)).unwrap();
        let now = sim.field.time_ns();
        sim.apply(&Command::PilotInput {
            chassis: id,
            frame: crate::input_stream::InputFrame {
                input_epoch: sim.input_epoch,
                sequence: 1,
                sampled_time_ns: now + tick_ns(),
                duration_ticks: 1,
                placement_revision: sim.field.chassis_revision(id).unwrap(),
                command: rm_sim_world::ChassisCommand {
                    forward_m_s: 0.3,
                    ..Default::default()
                },
            },
        })
        .unwrap();
        assert!(!sim.external_control.active(id));
        // An intervening refresh of the revoked lease must not erase the
        // scheduled manual transition.
        assert!(sim.apply(&external_input(&sim, id, 2, 2)).is_err());
        sim.step(2).unwrap();
        assert!(!sim.external_control.active(id));
        assert_eq!(
            sim.field.chassis_snapshot(id).unwrap().command.forward_m_s,
            0.3
        );
    }

    #[test]
    fn replayed_pilot_input_cannot_revoke_a_new_external_lease() {
        let (mut sim, id) = pilot_simulation();
        let frame = crate::input_stream::InputFrame {
            input_epoch: sim.input_epoch,
            sequence: 1,
            sampled_time_ns: sim.field.time_ns(),
            duration_ticks: 1,
            placement_revision: sim.field.chassis_revision(id).unwrap(),
            command: rm_sim_world::ChassisCommand {
                forward_m_s: 0.3,
                ..Default::default()
            },
        };
        sim.apply(&Command::PilotInput { chassis: id, frame })
            .unwrap();
        sim.apply(&external_input(&sim, id, 1, 1)).unwrap();
        sim.apply(&Command::PilotInput { chassis: id, frame })
            .unwrap();
        assert!(sim.external_control.active(id));
        let fresh = crate::input_stream::InputFrame {
            sequence: 2,
            sampled_time_ns: sim.field.time_ns() + 2 * tick_ns(),
            ..frame
        };
        sim.apply(&Command::PilotInput {
            chassis: id,
            frame: fresh,
        })
        .unwrap();
        assert!(!sim.external_control.active(id));
        assert!(sim.apply(&external_input(&sim, id, 1, 2)).is_err());
        sim.step(3).unwrap();
        assert_eq!(
            sim.field
                .chassis_control_state(id)
                .unwrap()
                .command
                .forward_m_s,
            0.3
        );
        let future = crate::input_stream::InputFrame {
            sequence: 3,
            sampled_time_ns: sim.field.time_ns() + 2 * tick_ns(),
            ..frame
        };
        sim.apply(&Command::PilotInput {
            chassis: id,
            frame: future,
        })
        .unwrap();
        sim.apply(&external_input(&sim, id, 2, 1)).unwrap();
        sim.apply(&Command::PilotInput {
            chassis: id,
            frame: future,
        })
        .unwrap();
        sim.step(3).unwrap();
        assert!(sim.external_control.active(id));
        assert_ne!(
            sim.field
                .chassis_control_state(id)
                .unwrap()
                .command
                .forward_m_s,
            0.3
        );
    }

    #[test]
    fn external_drive_pause_and_life_boundaries_reject_stale_packets() {
        let (mut sim, id) = pilot_simulation();
        let input = external_input(&sim, id, 1, 1);
        sim.apply(&input).unwrap();
        sim.set_paused(true);
        assert!(!sim.external_control.active(id));
        assert!(sim.apply(&external_input(&sim, id, 2, 1)).is_err());
        sim.set_paused(false);
        assert!(sim.apply(&input).is_err());
        let input = external_input(&sim, id, 2, 1);
        sim.apply(&input).unwrap();
        sim.apply(&Command::PlaceChassis {
            chassis: id,
            position_m: [1., 1., 0.5],
            yaw_deg: 90.,
        })
        .unwrap();
        assert!(!sim.external_control.active(id));
        assert!(sim.apply(&input).is_err());
        sim.apply(&external_input(&sim, id, 3, 1)).unwrap();
        let stale_release = Command::ReleaseExternalDrive {
            chassis: id,
            lease_id: 2,
        };
        assert!(sim.apply(&stale_release).is_err());
        assert!(sim.external_control.active(id));
        sim.apply(&Command::ReleaseExternalDrive {
            chassis: id,
            lease_id: 3,
        })
        .unwrap();
        assert!(!sim.external_control.active(id));
    }

    fn reference_robot(
        sim: &mut Simulation,
        team: Team,
        robot: Robot,
        chassis: crate::protocol::Chassis,
    ) -> u32 {
        let config = sim.robot_config(robot, chassis).unwrap();
        sim.spawn_in_slot(team, Some(config), robot.kind(), Some(robot))
            .unwrap()
    }

    #[test]
    fn team_cad_profile_survives_slot_and_explicit_spawning() {
        use crate::protocol::Chassis;
        let mut sim = simulation().with_spawner(ChassisSpawner {
            config: ChassisConfig::team_cad(),
            terrain: None,
        });
        let slot = sim.spawn_robot(Team::Red, Robot::Infantry3).unwrap();
        let explicit = sim
            .spawn_robot_at_with_chassis(
                Team::Blue,
                Robot::Infantry4,
                Chassis::Omni,
                [-5., 0., 0.],
                0.,
            )
            .unwrap();
        let state = sim.snapshot();
        for id in [slot, explicit] {
            assert_eq!(
                state.chassis.iter().find(|c| c.id == id).unwrap().config,
                ChassisConfig::team_cad()
            );
        }
        // Deployment shares this selector, while other drivetrains keep their presets.
        for (robot, chassis) in [
            (Robot::Hero, Chassis::Auto),
            (Robot::Engineer, Chassis::Auto),
            (Robot::Drone, Chassis::Auto),
            (Robot::Infantry3, Chassis::Balance),
        ] {
            assert_eq!(
                sim.robot_config(robot, chassis).unwrap(),
                chassis.config(robot).unwrap()
            );
        }
        assert!(sim.robot_config(Robot::Hero, Chassis::Balance).is_err());
    }

    #[test]
    fn weapon_updates_are_per_pilot_leave_flying_bullets_alone_and_keep_cooldown() {
        let (mut sim, shooter) = pilot_simulation();
        let other = sim.spawn_chassis(Team::Blue).unwrap();
        sim.apply(&Command::Fire { shooter }).unwrap();
        let before = sim.snapshot().projectiles;
        let mut weapon = sim.weapon();
        weapon.shot.speed_m_s = 10.;
        weapon.speed_variation_m_s = 1.;
        weapon.spread.angle_rad = 0.1;
        weapon.spread.distribution = crate::protocol::SpreadDistribution::Gaussian;
        sim.apply(&Command::ConfigureWeapon {
            chassis: shooter,
            weapon,
        })
        .unwrap();
        assert_eq!(sim.snapshot().projectiles, before);
        assert_eq!(
            sim.apply(&Command::Fire { shooter }).unwrap_err(),
            "weapon is cooling down"
        );
        sim.apply(&Command::Fire { shooter: other }).unwrap();
        let other_bullet = sim.snapshot().projectiles.last().unwrap().clone();
        let speed = |v: [f64; 3]| v.iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!((speed(other_bullet.velocity_m_s) - sim.weapon().shot.speed_m_s).abs() < 1e-9);
        sim.step(100).unwrap();
        let expected = weapon.spread.apply(
            sim.field().chassis_muzzle_pose(shooter).unwrap(),
            shooter,
            sim.field().time_ns(),
        );
        sim.apply(&Command::Fire { shooter }).unwrap();
        assert_eq!(
            sim.fire_records().last().unwrap().authoritative_muzzle_pose,
            expected
        );
        assert!(
            (speed(sim.snapshot().projectiles.last().unwrap().velocity_m_s)
                - weapon
                    .sample_shot(shooter, sim.field().time_ns(), sim.weapon().shot.speed_m_s)
                    .speed_m_s)
                .abs()
                < 1e-9
        );
        let invalid = WeaponConfig {
            interval_ns: 1,
            ..weapon
        };
        assert!(
            sim.apply(&Command::ConfigureWeapon {
                chassis: shooter,
                weapon: invalid
            })
            .is_err()
        );
        assert_eq!(sim.chassis_state[&shooter].weapon.unwrap(), weapon);
        sim.remove_chassis(shooter).unwrap();
        assert!(!sim.chassis_state.contains_key(&shooter));
    }

    #[test]
    fn ega_profile_is_selected_for_sentries_only() {
        let mut config = ChassisConfig::default();
        config.omni_frame = None;
        config.articulation = Some(
            serde_json::from_value(serde_json::json!({
                "profile": "ega-tunnel-sentry-v1", "limit_rad": 0., "speed_rad_s": 0.65,
                "muzzle_offset_m": [0.2,0.,0.], "camera_offset_m": [0.1,0.,0.],
                "armor_faces": config.armor_faces(),
                "joints": [{"name": "source-body", "parent": null,
                    "pivot_m": [0.,0.,0.], "axis": [0.,0.,1.], "angle_rad": 0.,
                    "translation_m": [0.,0.,0.], "drive": "Transform", "collision": null}]
            }))
            .unwrap(),
        );
        config.validate().unwrap();
        let sim = simulation().with_spawner(ChassisSpawner {
            config: config.clone(),
            terrain: None,
        });
        assert_eq!(
            sim.robot_config(Robot::Sentry, crate::protocol::Chassis::Auto)
                .unwrap(),
            config
        );
        assert_eq!(
            sim.robot_config(Robot::Sentry, crate::protocol::Chassis::TunnelOmni)
                .unwrap(),
            config
        );
        assert!(
            sim.robot_config(Robot::Sentry, crate::protocol::Chassis::Omni)
                .unwrap()
                .articulation
                .is_none()
        );
        assert!(
            simulation()
                .robot_config(Robot::Sentry, crate::protocol::Chassis::TunnelOmni)
                .is_err()
        );
        for robot in [Robot::Infantry3, Robot::Engineer] {
            assert!(
                sim.robot_config(robot, crate::protocol::Chassis::Auto)
                    .unwrap()
                    .articulation
                    .is_none()
            );
        }
    }

    #[test]
    fn selected_chassis_and_new_robot_roles_are_host_owned() {
        use crate::protocol::Chassis;
        let mut sim = simulation().with_spawner(ChassisSpawner {
            config: ChassisConfig::default(),
            terrain: None,
        });
        assert!(
            sim.spawn_robot_with_chassis(Team::Red, Robot::Hero, Chassis::Balance)
                .is_err()
        );
        let balance = sim
            .spawn_robot_with_chassis(Team::Red, Robot::Infantry3, Chassis::Balance)
            .unwrap();
        let sentry = sim.spawn_robot(Team::Blue, Robot::Sentry).unwrap();
        let engineer = sim.spawn_robot(Team::Red, Robot::Engineer).unwrap();
        assert!(sim.field().chassis_config(balance).unwrap().balance_assist);
        assert!(sim.field().chassis_config(engineer).unwrap().mecanum);
        sim.apply(&Command::Fire { shooter: sentry }).unwrap();
        assert!(
            sim.apply(&Command::Fire { shooter: engineer })
                .unwrap_err()
                .contains("no launcher")
        );
        let state = sim.state();
        let message = crate::protocol::ServerMessage::Snapshot(Box::new(state));
        let bytes = crate::snapshot_codec::encode_player_message(&message);
        let crate::protocol::ServerMessage::Snapshot(decoded) =
            crate::snapshot_codec::decode_player_message(&bytes).unwrap()
        else {
            panic!("snapshot expected")
        };
        assert!(
            decoded
                .field
                .chassis
                .iter()
                .find(|c| c.id == balance)
                .unwrap()
                .config
                .balance_assist
        );
    }

    #[test]
    fn a_hero_fires_42_mm_and_an_infantry_17_mm_whatever_the_host_default_says() {
        let mut sim = Simulation::new(Field::new(&FieldConfig::default()).unwrap(), false)
            .with_spawner(ChassisSpawner {
                config: ChassisConfig::default(),
                terrain: None,
            });
        let hero = reference_robot(
            &mut sim,
            Team::Red,
            Robot::Hero,
            crate::protocol::Chassis::Auto,
        );
        let infantry = sim.spawn_robot(Team::Blue, Robot::Infantry4).unwrap();
        assert_eq!(sim.robot(hero), Some(Robot::Hero));
        assert_eq!(sim.robot(infantry), Some(Robot::Infantry4));
        assert_eq!(sim.weapon().shot.caliber, rm_sim_world::Caliber::Mm17);
        assert_eq!(sim.caliber(hero), rm_sim_world::Caliber::Mm42);
        assert_eq!(sim.caliber(infantry), rm_sim_world::Caliber::Mm17);
        assert_eq!(
            sim.weapon_for(hero).shot.caliber,
            rm_sim_world::Caliber::Mm42
        );
        let placed = sim.field().chassis_config(hero).unwrap();
        assert!(placed.mecanum);
        assert!(!sim.field().chassis_config(infantry).unwrap().mecanum);
        // The hero starts on its own weapon, slower than the host default.
        assert_eq!(sim.weapon_for(hero), WeaponConfig::hero());
        let mut weapon = sim.weapon_for(hero);
        weapon.shot = rm_sim_world::Shot::at_limit(rm_sim_world::Caliber::Mm17);
        assert_eq!(
            sim.apply(&Command::ConfigureWeapon {
                chassis: hero,
                weapon
            })
            .unwrap_err(),
            "caliber follows the robot"
        );
        weapon.shot = rm_sim_world::Shot::at_limit(rm_sim_world::Caliber::Mm42);
        sim.apply(&Command::ConfigureWeapon {
            chassis: hero,
            weapon,
        })
        .unwrap();
        assert_eq!(sim.weapon_for(hero), weapon);

        for shooter in [hero, infantry] {
            sim.apply(&Command::Fire { shooter }).unwrap();
        }
        let calibers: Vec<_> = sim
            .snapshot()
            .projectiles
            .iter()
            .map(|p| p.caliber)
            .collect();
        assert_eq!(
            calibers,
            [rm_sim_world::Caliber::Mm42, rm_sim_world::Caliber::Mm17]
        );
        sim.remove_chassis(hero).unwrap();
        assert_eq!(sim.robot(hero), None);
        assert_eq!(sim.caliber(hero), rm_sim_world::Caliber::Mm17);
    }

    #[test]
    fn scheduled_launch_uses_settings_current_at_execution() {
        let (mut sim, shooter) = pilot_simulation();
        let input = crate::input_stream::InputFrame {
            input_epoch: 0,
            sequence: 1,
            sampled_time_ns: 16_000_000,
            duration_ticks: 16,
            placement_revision: 0,
            command: Default::default(),
        };
        sim.apply(&Command::FireAimed {
            shooter,
            shot_id: 1,
            input,
        })
        .unwrap();
        assert_eq!(sim.snapshot().shots_fired, 0);
        let mut weapon = sim.weapon();
        weapon.shot.speed_m_s = 7.;
        sim.apply(&Command::ConfigureWeapon {
            chassis: shooter,
            weapon,
        })
        .unwrap();
        sim.step(17).unwrap();
        let v = sim.snapshot().projectiles[0].velocity_m_s;
        // Spawn solver noise can shave a little speed; this only separates the
        // settings at execution (7) from the settings at scheduling (30).
        assert!((v.iter().map(|v| v * v).sum::<f64>().sqrt() - 7.).abs() < 0.5);
        assert_eq!(sim.snapshot().shots_fired, 1);
    }

    #[test]
    fn bots_spin_pause_and_remove_without_touching_pilots() {
        let (mut sim, pilot) = pilot_simulation();
        assert!(
            sim.apply(&Command::SpawnBot {
                team: Team::Blue,
                spin_rad_s: f64::NAN
            })
            .is_err()
        );
        sim.apply(&Command::SpawnBot {
            team: Team::Blue,
            spin_rad_s: 3.,
        })
        .unwrap();
        let bot = sim.state().bots[0];
        sim.step(1500).unwrap();
        let robot = sim
            .snapshot()
            .chassis
            .into_iter()
            .find(|c| c.id == bot)
            .unwrap();
        assert!(robot.angular_velocity_rad_s[2].abs() > 0.5);
        sim.set_paused(true);
        let before = sim.snapshot();
        assert_eq!(sim.advance(100_000_000).unwrap(), 0);
        assert_eq!(sim.snapshot(), before);
        assert!(sim.apply(&Command::RemoveBot { chassis: pilot }).is_err());
        sim.apply(&Command::ClearBots).unwrap();
        assert!(sim.state().bots.is_empty());
        assert_eq!(sim.snapshot().chassis.len(), 1);
        assert_eq!(sim.snapshot().chassis[0].id, pilot);
        sim.apply(&Command::SpawnBot {
            team: Team::Red,
            spin_rad_s: -3.,
        })
        .unwrap();
        assert!(sim.state().bots[0] > bot);
    }

    #[test]
    fn training_placement_rejects_pilots_and_invalid_poses_without_mutation() {
        let (mut sim, pilot) = pilot_simulation();
        let placement = |chassis, x| Command::PlaceBot {
            chassis,
            position_m: [x, 0., 2.],
            yaw_deg: 90.,
        };
        let before = sim.snapshot();
        assert_eq!(
            sim.apply(&placement(pilot, 1.)),
            Err("not a training bot".into())
        );
        assert_eq!(sim.snapshot(), before);
        sim.apply(&Command::SpawnBot {
            team: Team::Blue,
            spin_rad_s: 0.,
        })
        .unwrap();
        let bot = sim.state().bots[0];
        let before = sim.snapshot();
        assert!(sim.apply(&placement(bot, f64::NAN)).is_err());
        assert_eq!(sim.snapshot(), before);
        sim.apply(&placement(bot, 2.)).unwrap();
        let after = sim.snapshot();
        let robot = after.chassis.iter().find(|c| c.id == bot).unwrap();
        assert!((robot.pose.translation_m[0] - 2.).abs() < 1e-9);
        assert!(
            robot.placement_revision
                > before
                    .chassis
                    .iter()
                    .find(|c| c.id == bot)
                    .unwrap()
                    .placement_revision
        );
        assert_eq!(
            after.chassis.iter().find(|c| c.id == pilot),
            before.chassis.iter().find(|c| c.id == pilot)
        );
        assert_eq!(after.referee, before.referee);
    }

    #[test]
    fn pilot_purchases_charge_team_gold_and_reject_atomically() {
        use rm_sim_world::Caliber;
        let (mut sim, id) = pilot_simulation();
        let buy = |caliber| Command::BuyAmmo {
            chassis: id,
            caliber,
        };
        assert!(sim.apply(&buy(Caliber::Mm17)).is_err());
        sim.apply(&Command::Referee(RefereeCommand::StartMatch))
            .unwrap();
        sim.step(6_001_000_000 / rm_sim_world::tick_ns()).unwrap();
        let game = |sim: &Simulation| sim.snapshot().rmuc_2027.unwrap().game;
        sim.apply(&Command::Referee(RefereeCommand::SetGold {
            team: Team::Red,
            gold: 1000,
        }))
        .unwrap();
        let before = game(&sim);
        // Native Table 5-7 remote exchange: 100 rounds for 150 gold after six seconds.
        sim.apply(&buy(Caliber::Mm17)).unwrap();
        sim.step(768).unwrap();
        let after = game(&sim);
        assert_eq!(after.gold[0], before.gold[0] - 150);
        assert_eq!(
            after.robots[0].combat.allowance,
            before.robots[0].combat.allowance + 100
        );
        // An Infantry has no 42 mm launcher.
        let bought = sim.snapshot();
        assert!(sim.apply(&buy(Caliber::Mm42)).is_err());
        assert_eq!(sim.snapshot(), bought);
        sim.apply(&Command::Referee(RefereeCommand::SetGold {
            team: Team::Red,
            gold: 149,
        }))
        .unwrap();
        let empty = sim.snapshot();
        assert!(sim.apply(&buy(Caliber::Mm17)).is_err());
        assert_eq!(sim.snapshot(), empty);
        assert!(
            sim.apply(&Command::BuyAmmo {
                chassis: id + 1,
                caliber: Caliber::Mm17
            })
            .is_err()
        );
        assert_eq!(sim.snapshot(), empty);
    }

    #[test]
    fn pilots_pick_a_performance_type_before_the_round_runs() {
        use rm_sim_world::gameplay::{InfantryChassis, InfantryLauncher, Performance, Stats};
        let (mut sim, id) = pilot_simulation();
        let power = Performance::Infantry {
            chassis: InfantryChassis::PowerFocused,
            launcher: InfantryLauncher::BurstFocused,
        };
        let pick = |performance| Command::SetPerformance {
            chassis: id,
            performance,
        };
        sim.apply(&pick(power)).unwrap();
        // Native Table 5-17: a level-one power-focused chassis has 150 HP.
        assert_eq!(sim.snapshot().rmuc_2027.unwrap().game.robots[0].max_hp, 150);
        let fixed = Performance::Fixed(Stats {
            max_hp: 10_000,
            chassis_power_w: 1,
            heat_limit: 1,
            cooling_per_s: 1,
        });
        assert!(sim.apply(&pick(fixed)).is_err());
        assert!(
            sim.apply(&pick(Performance::Hero(Default::default())))
                .is_err()
        );
        sim.apply(&Command::Referee(RefereeCommand::StartMatch))
            .unwrap();
        sim.step(6_001_000_000 / rm_sim_world::tick_ns()).unwrap();
        // Native 2027 allows one running selection, then locks changed choices.
        sim.apply(&pick(power)).unwrap();
        let running = sim.snapshot();
        assert!(
            sim.apply(&pick(
                Performance::default_for(rm_sim_world::gameplay::RobotKind::Infantry).unwrap()
            ))
            .is_err()
        );
        assert_eq!(sim.snapshot(), running);
    }

    #[test]
    fn debug_reset_returns_owned_robot_to_spawn_without_resetting_economy() {
        let (mut sim, id) = pilot_simulation();
        let before = sim.snapshot();
        sim.test_field_mut()
            .place_chassis(id, rm_sim_world::Pose::at([50.0, 50.0, -2.0]))
            .unwrap();
        sim.apply(&Command::ResetRobot { chassis: id }).unwrap();
        let after = sim.snapshot();
        assert_eq!(after.chassis[0].pose, before.chassis[0].pose);
        assert_eq!(after.chassis[0].placement_revision, 2);
        assert_eq!(
            after.rmuc_2027.unwrap().game,
            before.rmuc_2027.unwrap().game
        );
        assert!(sim.apply(&Command::ResetRobot { chassis: id + 1 }).is_err());
    }

    #[test]
    fn referee_revival_preserves_pose_and_allowance() {
        let (mut sim, id) = pilot_simulation();
        sim.apply(&Command::Referee(RefereeCommand::StartMatch))
            .unwrap();
        sim.step(640).unwrap();
        let before = sim.snapshot();
        sim.apply(&Command::Referee(RefereeCommand::SetRobotHp {
            robot: id,
            hp: 0,
        }))
        .unwrap();
        assert!(sim.snapshot().chassis[0].defeated);
        assert!(sim.apply(&Command::Respawn { chassis: id }).is_err());
        sim.apply(&Command::Referee(RefereeCommand::ReviveRobot { robot: id }))
            .unwrap();
        let after = sim.snapshot();
        assert!(!after.chassis[0].defeated);
        assert_eq!(after.chassis[0].pose, before.chassis[0].pose);
        assert_eq!(
            after.rmuc_2027.unwrap().game.robots[0].combat.allowance,
            before.rmuc_2027.unwrap().game.robots[0].combat.allowance
        );
    }

    #[test]
    fn host_time_becomes_whole_ticks_with_a_cap_and_pause() {
        let mut simulation = simulation();
        let tick = rm_sim_world::tick_ns();
        assert_eq!(simulation.advance(tick + tick / 2).unwrap(), 1);
        assert_eq!(simulation.advance(tick + tick / 2).unwrap(), 2);
        assert_eq!(simulation.field().tick(), 3);
        let capped = simulation.advance(10_000_000_000).unwrap();
        assert_eq!(capped, MAX_ADVANCE_NS / tick);
        simulation.apply(&Command::Pause { paused: true }).unwrap();
        assert_eq!(simulation.advance(1_000_000_000).unwrap(), 0);
        simulation.apply(&Command::Step { ticks: 16 }).unwrap();
        assert_eq!(simulation.field().tick(), 3 + capped + 16);
        assert!(simulation.apply(&Command::Step { ticks: 0 }).is_err());
        simulation.apply(&Command::Pause { paused: false }).unwrap();
        assert_eq!(simulation.advance(tick - 1).unwrap(), 0);
        assert_eq!(simulation.advance(1).unwrap(), 1);
    }
    #[test]
    fn commands_reach_the_field_and_report_rejections() {
        let mut simulation = simulation();
        simulation
            .apply(&Command::Referee(RefereeCommand::StartMatch))
            .unwrap();
        assert_eq!(
            simulation.snapshot().rmuc_2027.unwrap().game.phase,
            rm_sim_world::rmuc_2027::match_rules::Phase::Countdown
        );
        let rejected = simulation
            .apply(&Command::Referee(RefereeCommand::StartMatch))
            .unwrap_err();
        assert!(rejected.contains("reset"), "{rejected}");
        // No chassis on this field, and no spawner to make one.
        assert!(
            simulation
                .apply(&Command::Chassis {
                    chassis: 0,
                    command: Default::default()
                })
                .is_err()
        );
        assert!(simulation.spawn_chassis(Team::Red).is_err());
        simulation.step(640).unwrap();
        simulation
            .apply(&Command::SpawnProjectile {
                muzzle: rm_sim_world::Pose::at([0.0, 0.0, 1.0]),
                shot: rm_sim_world::Shot::at_limit(rm_sim_world::Caliber::Mm17),
            })
            .unwrap();
        assert_eq!(simulation.snapshot().shots_fired, 1);
    }
    #[test]
    fn the_spawner_fills_team_slots_and_frees_them_on_removal() {
        let mut simulation = simulation().with_spawner(ChassisSpawner {
            config: ChassisConfig::default(),
            terrain: None,
        });
        let red = simulation.spawn_chassis(Team::Red).unwrap();
        let red2 = simulation.spawn_chassis(Team::Red).unwrap();
        let blue = simulation.spawn_chassis(Team::Blue).unwrap();
        let chassis = simulation.snapshot().chassis;
        assert_eq!(chassis.len(), 3);
        assert!(chassis[0].pose.translation_m[0] > 0.0 && chassis[2].pose.translation_m[0] < 0.0);
        // The second red chassis sits beside the first.
        assert!((chassis[1].pose.translation_m[1] - chassis[0].pose.translation_m[1]).abs() > 0.5);
        simulation
            .apply(&Command::Chassis {
                chassis: blue,
                command: Default::default(),
            })
            .unwrap();
        simulation.remove_chassis(red).unwrap();
        assert!(simulation.remove_chassis(red).is_err());
        // The freed slot is reused by the next red player; ids are not.
        let red3 = simulation.spawn_chassis(Team::Red).unwrap();
        assert!(red3 > red2);
        let chassis = simulation.snapshot().chassis;
        assert_eq!(chassis.len(), 3);
        assert!((chassis[2].pose.translation_m[1] - chassis[0].pose.translation_m[1]).abs() > 0.5);
    }

    #[test]
    fn fire_journal_is_bounded_and_rejections_do_not_create_shots() {
        let mut simulation = Simulation::new(Field::new(&FieldConfig::default()).unwrap(), false)
            .with_spawner(ChassisSpawner {
                config: ChassisConfig::default(),
                terrain: None,
            })
            .with_weapon(WeaponConfig {
                interval_ns: rm_sim_world::tick_ns(),
                ..Default::default()
            })
            .unwrap();
        let shooter = simulation.spawn_chassis(Team::Red).unwrap();
        for _ in 0..257 {
            simulation.apply(&Command::Fire { shooter }).unwrap();
            assert!(simulation.apply(&Command::Fire { shooter }).is_err());
            simulation.step(1).unwrap();
        }
        let records = simulation.fire_records();
        assert_eq!(records.len(), 256);
        assert_eq!(
            records.first().unwrap().accepted_time_ns,
            rm_sim_world::tick_ns()
        );
        assert_eq!(
            records.last().unwrap().accepted_time_ns,
            256 * rm_sim_world::tick_ns()
        );
    }

    #[test]
    fn pilot_fire_uses_the_authoritative_muzzle_and_cadence() {
        let weapon = WeaponConfig {
            shot: rm_sim_world::Shot {
                caliber: rm_sim_world::Caliber::Mm17,
                speed_m_s: 30.0,
            },
            interval_ns: 10_000_000,
            speed_variation_m_s: 0.,
            spread: Default::default(),
        };
        let mut simulation = simulation()
            .with_spawner(ChassisSpawner {
                config: ChassisConfig::default(),
                terrain: None,
            })
            .with_weapon(weapon)
            .unwrap();
        let chassis = simulation.spawn_chassis(Team::Red).unwrap();
        let other = simulation.spawn_chassis(Team::Blue).unwrap();
        simulation
            .apply(&Command::Chassis {
                chassis,
                command: rm_sim_world::ChassisCommand {
                    aim_yaw_rad: 0.7,
                    aim_pitch_rad: 0.2,
                    ..Default::default()
                },
            })
            .unwrap();
        simulation.step(1).unwrap();
        let expected = simulation.field().chassis_muzzle_pose(chassis).unwrap();
        simulation
            .apply(&Command::Fire { shooter: chassis })
            .unwrap();
        let records = simulation.fire_records();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].authoritative_muzzle_pose, expected);
        assert_eq!(records[0].accepted_time_ns, rm_sim_world::tick_ns());
        let snapshot = simulation.snapshot();
        assert_eq!(records[0].projectile_id, snapshot.projectiles[0].id);
        assert_eq!(snapshot.projectiles[0].position_m, expected.translation_m);
        assert_eq!(snapshot.projectiles[0].caliber, weapon.shot.caliber);
        let speed = snapshot.projectiles[0]
            .velocity_m_s
            .iter()
            .map(|v| v * v)
            .sum::<f64>()
            .sqrt();
        assert!((speed - 25.0).abs() < 1e-9);
        assert!(
            simulation
                .apply(&Command::Fire { shooter: chassis })
                .is_err()
        );
        simulation.set_paused(true);
        assert_eq!(simulation.advance(u64::MAX).unwrap(), 0);
        assert!(
            simulation
                .apply(&Command::Fire { shooter: chassis })
                .is_err()
        );
        simulation.apply(&Command::Fire { shooter: other }).unwrap();
        assert_eq!(simulation.snapshot().shots_fired, 2);
        // One tick past the shot the gun is still cooling; the interval is
        // longer than one tick, and two ticks clear it.
        simulation.step(1).unwrap();
        assert!(
            simulation
                .apply(&Command::Fire { shooter: chassis })
                .is_err()
        );
        simulation.step(1).unwrap();
        simulation
            .apply(&Command::Fire { shooter: chassis })
            .unwrap();
        assert_eq!(simulation.snapshot().shots_fired, 3);
    }

    #[test]
    fn rejected_fire_does_not_consume_a_cooldown() {
        let mut simulation = simulation().with_spawner(ChassisSpawner {
            config: ChassisConfig::default(),
            terrain: None,
        });
        let chassis = simulation.spawn_chassis(Team::Blue).unwrap();
        simulation
            .test_field_mut()
            .referee_command(RefereeCommand::SetRobotHp {
                robot: chassis,
                hp: 0,
            })
            .unwrap();
        assert!(
            simulation
                .apply(&Command::Fire { shooter: chassis })
                .is_err()
        );
        simulation
            .apply(&Command::Referee(RefereeCommand::ReviveRobot {
                robot: chassis,
            }))
            .unwrap();
        simulation
            .apply(&Command::Fire { shooter: chassis })
            .unwrap();
        assert_eq!(simulation.snapshot().shots_fired, 1);
    }
    #[test]
    fn cooldown_rejection_is_terminal_and_retries_cannot_schedule_a_shot() {
        let (mut sim, shooter) = pilot_simulation();
        let input = crate::input_stream::InputFrame {
            input_epoch: 0,
            sequence: 1,
            sampled_time_ns: 0,
            duration_ticks: 16,
            placement_revision: sim.field().chassis_revision(shooter).unwrap(),
            command: Default::default(),
        };
        let fire = |shot_id| Command::FireAimed {
            shooter,
            shot_id,
            input,
        };
        sim.apply(&fire(1)).unwrap();
        assert_eq!(sim.apply(&fire(2)).unwrap_err(), "weapon is cooling down");
        assert!(sim.shot_result(shooter, 2).unwrap().result.is_err());
        assert_eq!(sim.snapshot().shots_fired, 1);
        sim.step(sim.weapon.interval_ns.div_ceil(rm_sim_world::tick_ns()))
            .unwrap();
        assert_eq!(sim.apply(&fire(2)).unwrap_err(), "weapon is cooling down");
        assert_eq!(sim.snapshot().shots_fired, 1);
        // A fresh intent spaced one interval behind the first on the pilot's
        // clock fires; one that reuses the first shot's time is still cooling.
        assert_eq!(sim.apply(&fire(3)).unwrap_err(), "weapon is cooling down");
        sim.apply(&Command::FireAimed {
            shooter,
            shot_id: 4,
            input: crate::input_stream::InputFrame {
                sequence: 2,
                sampled_time_ns: sim.weapon.interval_ns,
                ..input
            },
        })
        .unwrap();
        assert_eq!(sim.snapshot().shots_fired, 2);
    }

    #[test]
    fn a_late_shot_does_not_reject_the_correctly_spaced_shot_behind_it() {
        let (mut sim, shooter) = pilot_simulation();
        let interval = sim.weapon.interval_ns;
        let placement_revision = sim.field().chassis_revision(shooter).unwrap();
        let frame = move |sequence, sampled_time_ns| crate::input_stream::InputFrame {
            input_epoch: 0,
            sequence,
            sampled_time_ns,
            duration_ticks: 16,
            placement_revision,
            command: Default::default(),
        };
        // The first intent arrives well past its intended time and fires late.
        sim.step(sim.weapon.interval_ns.div_ceil(rm_sim_world::tick_ns()))
            .unwrap();
        sim.apply(&Command::FireAimed {
            shooter,
            shot_id: 1,
            input: frame(1, 0),
        })
        .unwrap();
        // The second was spaced one interval behind the first on the pilot's
        // clock and arrives on time; it is also overdue, so it fires at once.
        sim.apply(&Command::FireAimed {
            shooter,
            shot_id: 2,
            input: frame(2, interval),
        })
        .unwrap();
        assert_eq!(sim.snapshot().shots_fired, 2);
        // A third, overdue intent between the two on the pilot's clock is refused.
        assert_eq!(
            sim.apply(&Command::FireAimed {
                shooter,
                shot_id: 3,
                input: frame(3, interval / 2),
            })
            .unwrap_err(),
            "weapon is cooling down"
        );
        assert_eq!(sim.snapshot().shots_fired, 2);
    }

    #[test]
    fn scheduled_fire_uses_the_same_tick_moving_muzzle_once() {
        let (mut sim, shooter) = pilot_simulation();
        sim.apply(&Command::Referee(RefereeCommand::StartMatch))
            .unwrap();
        sim.step(6_001_000_000 / rm_sim_world::tick_ns()).unwrap();
        sim.apply(&Command::Referee(RefereeCommand::SetGold {
            team: Team::Red,
            gold: 1000,
        }))
        .unwrap();
        sim.apply(&Command::BuyAmmo {
            chassis: shooter,
            caliber: rm_sim_world::Caliber::Mm17,
        })
        .unwrap();
        sim.step(768).unwrap();
        let frame = crate::input_stream::InputFrame {
            input_epoch: sim.input_epoch(),
            sequence: 1,
            sampled_time_ns: sim.field.time_ns() + 64_000_000,
            duration_ticks: 16,
            placement_revision: sim.field.chassis_revision(shooter).unwrap(),
            command: rm_sim_world::ChassisCommand {
                forward_m_s: 2.,
                aim_yaw_rad: 0.4,
                ..Default::default()
            },
        };
        sim.apply(&Command::Chassis {
            chassis: shooter,
            command: frame.command,
        })
        .unwrap();
        let fire = Command::FireAimed {
            shooter,
            shot_id: 1,
            input: frame,
        };
        let allowance = sim.snapshot().rmuc_2027.unwrap().game.robots[0]
            .combat
            .allowance;
        sim.apply(&fire).unwrap();
        sim.apply(&fire).unwrap();
        assert_eq!(
            sim.snapshot().rmuc_2027.unwrap().game.robots[0]
                .combat
                .allowance,
            allowance
        );
        assert_eq!(sim.pending_shots.len(), 1);
        assert_eq!(sim.snapshot().shots_fired, 0);
        let mut changed = frame;
        changed.command.aim_yaw_rad = 1.;
        assert!(
            sim.apply(&Command::FireAimed {
                shooter,
                shot_id: 1,
                input: changed,
            })
            .is_err()
        );
        sim.step(64_000_000_u64.div_ceil(rm_sim_world::tick_ns()) - 1)
            .unwrap();
        assert!(sim.shot_result(shooter, 1).is_none());
        sim.apply(&Command::PilotInput {
            chassis: shooter,
            frame,
        })
        .unwrap();
        // Advance to the tick before the shot's sampled time; the state there
        // is exactly where the muzzle will be read.
        sim.step(1).unwrap();
        let expected = crate::prediction::muzzle_for(&sim.snapshot().chassis[0], frame.command);
        // The pending shot executes at the start of the first tick whose time
        // has reached the sampled time.
        sim.step(1).unwrap();
        let actual = sim.fire_records()[0].authoritative_muzzle_pose;
        assert!(
            actual
                .translation_m
                .into_iter()
                .zip(expected.translation_m)
                .all(|(a, b)| (a - b).abs() < 1e-12)
        );
        assert!(
            actual
                .rotation_wxyz
                .into_iter()
                .zip(expected.rotation_wxyz)
                .all(|(a, b)| (a - b).abs() < 1e-12)
        );
        // The shot executes at the first tick whose time reaches the sampled
        // time, so the recorded execution is that tick's own clock reading.
        let executed =
            frame.sampled_time_ns.div_ceil(rm_sim_world::tick_ns()) * rm_sim_world::tick_ns();
        assert_eq!(
            sim.shot_result(shooter, 1).unwrap().executed_time_ns,
            Some(executed)
        );
        sim.apply(&fire).unwrap();
        assert_eq!(sim.snapshot().shots_fired, 1);
        assert_eq!(sim.take_completed_shots().len(), 1);
        let after = sim.snapshot().rmuc_2027.unwrap().game.robots[0]
            .combat
            .allowance;
        assert_eq!(after, allowance - 1);
    }
    #[test]
    fn shot_queue_is_bounded_and_cancels_across_pause_placement_and_defeat() {
        for change in 0..3 {
            let (mut sim, shooter) = pilot_simulation();
            let input = crate::input_stream::InputFrame {
                input_epoch: sim.input_epoch(),
                sequence: 1,
                sampled_time_ns: 100_000_000,
                duration_ticks: 16,
                placement_revision: sim.field.chassis_revision(shooter).unwrap(),
                command: Default::default(),
            };
            for shot_id in 1..=32 {
                sim.apply(&Command::FireAimed {
                    shooter,
                    shot_id,
                    input,
                })
                .unwrap();
            }
            assert_eq!(
                sim.apply(&Command::FireAimed {
                    shooter,
                    shot_id: 33,
                    input,
                })
                .unwrap_err(),
                "shot schedule full"
            );
            match change {
                0 => {
                    sim.set_paused(true);
                    sim.set_paused(false);
                }
                1 => {
                    sim.apply(&Command::PlaceChassis {
                        chassis: shooter,
                        position_m: [0., 0., 1.],
                        yaw_deg: 0.,
                    })
                    .unwrap();
                }
                _ => {
                    sim.apply(&Command::Referee(
                        rm_sim_world::RefereeCommand::SetRobotHp {
                            robot: shooter,
                            hp: 0,
                        },
                    ))
                    .unwrap();
                }
            }
            sim.step(150).unwrap();
            assert!(sim.pending_shots.is_empty());
            assert_eq!(sim.snapshot().shots_fired, 0);
            assert_eq!(sim.take_completed_shots().len(), 32);
            assert!(sim.shot_result(shooter, 1).unwrap().result.is_err());
        }
    }
    #[test]
    fn same_tick_queued_shots_do_not_form_a_burst_and_partitioning_is_stable() {
        fn run(parts: &[u64]) -> (Vec<FireRecord>, Vec<crate::protocol::ShotResult>) {
            let (mut sim, shooter) = pilot_simulation();
            for shot_id in (1..=3).rev() {
                let input = crate::input_stream::InputFrame {
                    input_epoch: sim.input_epoch(),
                    sequence: shot_id,
                    sampled_time_ns: 50_000_000,
                    duration_ticks: 16,
                    placement_revision: sim.field.chassis_revision(shooter).unwrap(),
                    command: Default::default(),
                };
                sim.apply(&Command::FireAimed {
                    shooter,
                    shot_id,
                    input,
                })
                .unwrap();
            }
            for part in parts {
                sim.step(*part).unwrap();
            }
            (sim.fire_records(), sim.take_completed_shots().into())
        }
        let a = run(&[100]);
        let b = run(&[25, 25, 1, 49]);
        assert_eq!(a, b);
        assert_eq!(a.0.len(), 1);
        assert_eq!(a.1.iter().filter(|r| r.result.is_err()).count(), 2);
    }

    #[test]
    fn reordered_fresh_shots_are_deduplicated_without_sequence_head_of_line_loss() {
        let (mut sim, shooter) = pilot_simulation();
        let frame = crate::input_stream::InputFrame {
            input_epoch: 0,
            sequence: 1,
            sampled_time_ns: 0,
            duration_ticks: 16,
            placement_revision: sim.field().chassis_revision(shooter).unwrap(),
            command: Default::default(),
        };
        // Each shot has its own intended time; the newer one arrives first.
        let interval = sim.weapon.interval_ns;
        let command = move |shot_id| Command::FireAimed {
            shooter,
            shot_id,
            input: crate::input_stream::InputFrame {
                sequence: shot_id,
                sampled_time_ns: (shot_id - 1) * interval,
                ..frame
            },
        };
        sim.apply(&command(2)).unwrap();
        sim.step(2 * interval / rm_sim_world::tick_ns()).unwrap();
        assert_eq!(sim.snapshot().shots_fired, 1);
        sim.apply(&command(1)).unwrap();
        sim.apply(&command(1)).unwrap();
        sim.apply(&command(2)).unwrap();
        assert_eq!(sim.snapshot().shots_fired, 2);
    }

    #[test]
    fn outage_shots_expire_without_backlog_bursts_or_movement_refresh() {
        let (mut sim, shooter) = pilot_simulation();
        let mut input = crate::input_stream::InputFrame {
            input_epoch: 0,
            sequence: 1,
            sampled_time_ns: 0,
            duration_ticks: 16,
            placement_revision: sim.field().chassis_revision(shooter).unwrap(),
            command: rm_sim_world::ChassisCommand {
                forward_m_s: 2.,
                aim_yaw_rad: 0.5,
                ..Default::default()
            },
        };
        sim.step(300).unwrap();
        assert!(
            sim.apply(&Command::FireAimed {
                shooter,
                shot_id: 1,
                input,
            })
            .is_err()
        );
        assert_eq!(sim.snapshot().shots_fired, 0);
        input.sampled_time_ns = sim.field().time_ns();
        let fire = Command::FireAimed {
            shooter,
            shot_id: 2,
            input,
        };
        sim.apply(&fire).unwrap();
        sim.apply(&fire).unwrap();
        assert_eq!(sim.snapshot().shots_fired, 1);
        assert_eq!(sim.snapshot().chassis[0].command.forward_m_s, 0.);
        assert!(sim.input_streams.is_empty());
        assert!(
            sim.apply(&Command::FireAimed {
                shooter,
                shot_id: 3,
                input,
            })
            .is_err()
        );
        assert_eq!(sim.snapshot().shots_fired, 1);
        assert!(!sim.snapshot().projectiles.is_empty());
    }

    #[test]
    fn revival_without_fresh_packets_neutralizes_the_previous_drive() {
        {
            let (mut sim, chassis) = pilot_simulation();
            sim.apply(&Command::Referee(RefereeCommand::StartMatch))
                .unwrap();
            sim.step(640).unwrap();
            let frame = crate::input_stream::InputFrame {
                input_epoch: sim.input_epoch(),
                sequence: 1,
                sampled_time_ns: sim.field().time_ns(),
                duration_ticks: 16,
                placement_revision: sim.field().chassis_revision(chassis).unwrap(),
                command: rm_sim_world::ChassisCommand {
                    forward_m_s: 2.,
                    ..Default::default()
                },
            };
            sim.apply(&Command::PilotInput { chassis, frame }).unwrap();
            sim.apply(&Command::Referee(RefereeCommand::SetRobotHp {
                robot: chassis,
                hp: 0,
            }))
            .unwrap();
            sim.apply(&Command::Referee(RefereeCommand::ReviveRobot {
                robot: chassis,
            }))
            .unwrap();
            sim.step(300).unwrap();
            assert_eq!(sim.snapshot().chassis[0].command.forward_m_s, 0.);
        }
    }

    #[test]
    fn queued_and_delayed_inputs_and_shots_cannot_cross_pause_epochs() {
        let (mut sim, chassis) = pilot_simulation();
        let mut frame = crate::input_stream::InputFrame {
            input_epoch: 0,
            sequence: 1,
            sampled_time_ns: 100_000_000,
            duration_ticks: 16,
            placement_revision: sim.field().chassis_revision(chassis).unwrap(),
            command: rm_sim_world::ChassisCommand {
                forward_m_s: 2.,
                ..Default::default()
            },
        };
        sim.apply(&Command::PilotInput { chassis, frame }).unwrap();
        sim.set_paused(true);
        sim.set_paused(false);
        assert!(sim.apply(&Command::PilotInput { chassis, frame }).is_err());
        assert!(
            sim.apply(&Command::FireAimed {
                shooter: chassis,
                shot_id: 1,
                input: frame,
            })
            .is_err()
        );
        sim.step(150).unwrap();
        assert_eq!(sim.snapshot().chassis[0].command.forward_m_s, 0.);
        frame.input_epoch = sim.input_epoch();
        frame.sampled_time_ns = sim.field().time_ns();
        frame.sequence += 1;
        sim.apply(&Command::PilotInput { chassis, frame }).unwrap();
        assert_eq!(sim.snapshot().chassis[0].command.forward_m_s, 2.);
    }

    #[test]
    fn future_controls_cannot_cross_a_reset_or_revived_life() {
        let (mut sim, chassis) = pilot_simulation();
        let frame = crate::input_stream::InputFrame {
            input_epoch: 0,
            sequence: 1,
            sampled_time_ns: 100_000_000,
            duration_ticks: 16,
            placement_revision: sim.field().chassis_revision(chassis).unwrap(),
            command: rm_sim_world::ChassisCommand {
                forward_m_s: 2.,
                ..Default::default()
            },
        };
        sim.apply(&Command::PilotInput { chassis, frame }).unwrap();
        sim.apply(&Command::ResetRobot { chassis }).unwrap();
        sim.step(150).unwrap();
        assert_eq!(sim.snapshot().chassis[0].command.forward_m_s, 0.);
        assert!(sim.apply(&Command::PilotInput { chassis, frame }).is_err());
    }

    #[test]
    fn aimed_fire_uses_its_own_input_without_rewinding_newer_held_aim() {
        let (mut simulation, shooter) = pilot_simulation();
        let revision = simulation.snapshot().chassis[0].placement_revision;
        let input = |sequence, yaw| crate::input_stream::InputFrame {
            input_epoch: 0,
            sequence,
            sampled_time_ns: simulation.field().time_ns(),
            duration_ticks: 16,
            placement_revision: revision,
            command: rm_sim_world::ChassisCommand {
                aim_yaw_rad: yaw,
                ..Default::default()
            },
        };
        let earlier = input(1, 0.25);
        let mut newer = input(2, 1.25);
        newer.command.forward_m_s = 2.;
        newer.command.left_m_s = -0.5;
        newer.command.yaw_rate_rad_s = 0.3;
        simulation
            .apply(&Command::PilotInput {
                chassis: shooter,
                frame: newer,
            })
            .unwrap();
        let held_state = simulation.field().chassis_control_state(shooter).unwrap();
        let fire = Command::FireAimed {
            shooter,
            shot_id: 1,
            input: earlier,
        };
        simulation.apply(&fire).unwrap();
        let shot = simulation.fire_records()[0].clone();
        let expected =
            crate::prediction::muzzle_for(&simulation.snapshot().chassis[0], earlier.command);
        assert!(
            shot.authoritative_muzzle_pose
                .translation_m
                .into_iter()
                .zip(expected.translation_m)
                .all(|(a, b)| (a - b).abs() < 1e-12)
        );
        assert!(
            shot.authoritative_muzzle_pose
                .rotation_wxyz
                .into_iter()
                .zip(expected.rotation_wxyz)
                .all(|(a, b)| (a - b).abs() < 1e-12)
        );
        assert_eq!(simulation.snapshot().chassis[0].command, newer.command);
        assert_eq!(
            simulation.field().chassis_control_state(shooter).unwrap(),
            held_state
        );
        assert_eq!(
            simulation
                .apply(&Command::FireAimed {
                    shooter,
                    shot_id: 2,
                    input: earlier,
                })
                .unwrap_err(),
            "weapon is cooling down"
        );
        assert_eq!(
            simulation.field().chassis_control_state(shooter).unwrap(),
            held_state
        );
        simulation.apply(&fire).unwrap();
        assert_eq!(simulation.fire_records().len(), 1);
        assert_eq!(simulation.snapshot().shots_fired, 1);
    }
}

#[cfg(test)]
mod roster_tests {
    use super::*;
    #[test]
    fn host_rejects_removed_roles_before_creating_a_chassis() {
        let mut simulation = Simulation::new(Field::new(&Default::default()).unwrap(), true);
        for robot in [Robot::Hero, Robot::Infantry5, Robot::Drone] {
            assert!(
                simulation
                    .spawn_robot(Team::Red, robot)
                    .unwrap_err()
                    .contains("roster")
            );
            assert!(
                simulation
                    .spawn_robot_at(Team::Red, robot, [0., 0., 1.], 0.)
                    .unwrap_err()
                    .contains("roster")
            );
        }
        assert!(simulation.field.snapshot().chassis.is_empty());
    }
}

impl Simulation {
    /// Enable verified automatic assembly with a source rig, matching terrain and
    /// optional completed-recording hints. No robot is moved by this installation.
    pub fn with_assembly_assets(
        mut self,
        package: Option<&std::path::Path>,
        area: Option<&std::path::Path>,
        hints: Option<&std::path::Path>,
        floor_top_cad_m: f64,
    ) -> Result<Self, String> {
        let Some(package) = package else {
            return Ok(self);
        };
        let area = area.ok_or("--assembly-assets requires --assembly-area")?;
        {
            self.assembly = Some(crate::assembly::Service::load(
                package,
                area,
                hints,
                floor_top_cad_m,
            )?);
            Ok(self)
        }
    }
    fn start_auto_assembly(&mut self, owner: u32, difficulty: u8) -> Result<(), String> {
        {
            if self.robot(owner) != Some(Robot::Engineer) {
                return Err("Automatic assembly requires the Engineer/Heavy pilot".into());
            }
            let chassis = self
                .field
                .chassis_snapshot(owner)
                .ok_or("Assembly chassis is unavailable")?;
            if chassis.defeated {
                return Err("A defeated robot cannot assemble".into());
            }
            if chassis
                .config
                .articulation
                .as_ref()
                .is_none_or(|a| a.profile != "pie-tunnel-heavy-v1")
            {
                return Err(
                    "Automatic assembly currently needs the source PIE tunnel Heavy".into(),
                );
            }
            if self
                .field
                .assemblies()
                .iter()
                .any(|r| r.station_busy() && (r.owner == owner || r.team == chassis.team))
            {
                return Err("Assembly station is busy; cancel before restarting".into());
            }
            let service = self
                .assembly
                .as_mut()
                .ok_or("Host has no verified assembly assets installed")?;
            let awards = self.field.rmuc_2027().is_some_and(|s| {
                s.game.phase == rm_sim_world::rmuc_2027::match_rules::Phase::Running
            });
            if let Some(state) = self.field.rmuc_2027() {
                if awards {
                    let mut game = state.game.clone();
                    game.start_assembly(
                        if chassis.team == Team::Red {
                            rm_sim_world::gameplay::Team::Red
                        } else {
                            rm_sim_world::gameplay::Team::Blue
                        },
                        difficulty,
                    )
                    .map_err(str::to_string)?;
                }
            } else {
                return Err("Automatic assembly requires the bound RMUC 2027 field".into());
            }
            let arm = self
                .field
                .arms()
                .iter()
                .find(|a| a.owner == owner)
                .cloned()
                .ok_or("Source arm has not finished initialization")?;
            let previous = self
                .field
                .assemblies()
                .iter()
                .find(|r| r.team == chassis.team)
                .map(|r| r.pose.clone());
            let unit = self
                .field
                .energy_units()
                .into_iter()
                .find(|u| u.attachment == rm_sim_world::energy_unit::Attachment::Held(owner))
                .ok_or("Pick up an Energy Unit before starting assembly")?;
            let run = service.start(
                difficulty,
                &chassis,
                self.field.time_ns(),
                awards,
                &arm,
                previous.as_ref(),
                &unit,
            )?;
            // Native 45-second assembly admission starts when the Core arrives,
            // not while its positioning mechanism is still moving.
            self.input_streams.remove(&owner);
            self.field
                .command_chassis(
                    owner,
                    rm_sim_world::ChassisCommand {
                        forward_m_s: 0.,
                        left_m_s: 0.,
                        yaw_rate_rad_s: 0.,
                        jump: false,
                        ..chassis.command
                    },
                )
                .map_err(|e| e.to_string())?;
            self.field.set_assembly(run).map_err(|e| e.to_string())
        }
    }
}
