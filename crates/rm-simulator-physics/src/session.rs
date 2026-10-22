// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-paced bare robot simulation over one [`WorldPhysics`].
//!
//! No match authority, transport or host clock is involved. A repeatable initial
//! [`RobotScenario`] supports reset; arbitrary physical checkpoints are not yet
//! supported. Individual body snapshots omit solver caches, projectile contact
//! memory and policy history, prescribed target/mechanism state and pending
//! events. They must not be presented as a complete continuation checkpoint.
//! Sensor schedules, rolling brackets and IMU state remain caller-owned and must
//! be reset separately after an episode reset or discontinuous placement.

use crate::{
    Shot, StepDuration, TargetFace, TargetFrames, WorldPhysics,
    chassis::{ChassisCommand, ChassisSnapshot},
    control::WheelTorques,
    projectile::Contact,
};

/// Simulation timestamp; nanoseconds are authoritative across integration rates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Stamp {
    /// Completed physical integration steps since scenario construction.
    pub tick: u64,
    /// Elapsed simulation nanoseconds, independent of host time.
    pub time_ns: u64,
}

/// An addressed physical request for a trusted in-process controller.
#[derive(Clone, Debug, PartialEq)]
pub enum RobotCommand {
    /// Body velocity, stabilized gimbal target and mechanism setpoints.
    Drive {
        /// Existing physical robot identity.
        robot: u32,
        /// Request validated through the ordinary physical control path.
        command: ChassisCommand,
    },
    /// Hold wheel-axis torques, retaining gimbal and folding targets.
    WheelTorques {
        /// Existing physical robot identity.
        robot: u32,
        /// Newton metre requests in configured wheel order.
        torques: WheelTorques,
    },
    /// Launch from the robot's actual muzzle. The receiving session applies its
    /// authority policy; [`RobotSession`] adds no match cadence or ammo rules.
    Fire {
        /// Existing physical robot identity.
        robot: u32,
        /// Caliber and speed in metres per second.
        shot: Shot,
    },
}

/// Result of an accepted physical request; accepting commands advances no time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandResult {
    /// A held drive request was accepted.
    Driven,
    /// A projectile was launched with this physical identity.
    Fired(u64),
}

/// Timestamped privileged robot truth, distinct from sensor measurements.
#[derive(Clone, Debug, PartialEq)]
pub struct RobotTruth {
    /// Time at which state was read.
    pub stamp: Stamp,
    /// Actual physical state and wheel telemetry, without a world checkpoint.
    pub state: ChassisSnapshot,
}

type Setup = dyn Fn(&mut WorldPhysics) -> Result<(), &'static str> + Send + Sync;

/// Repeatable initial physical scenario, not a checkpoint of a running world.
///
/// The setup recipe runs on a fresh world at time zero on construction and every
/// reset. It may install shared geometry, models, payloads, initial projectiles,
/// policies, prescribed mechanisms and carrier trace recording through ordinary
/// physics methods. It must not step the world or change its synchronization
/// clock. Capture immutable inputs; mutable external state can make resets differ.
/// There is no implicit randomization or seed. Reset restarts the identity space.
/// Runtime edits are discarded unless represented in this initial recipe.
pub struct RobotScenario {
    faces: Vec<TargetFace>,
    floor_height_m: f64,
    step_duration: StepDuration,
    setup: Box<Setup>,
}
impl RobotScenario {
    /// Define validated initial targets, a catch-floor height in metres and timing.
    /// The recipe is invoked by [`RobotSession::new`] and [`RobotSession::reset`].
    pub fn new(
        faces: Vec<TargetFace>,
        floor_height_m: f64,
        step_duration: StepDuration,
        setup: impl Fn(&mut WorldPhysics) -> Result<(), &'static str> + Send + Sync + 'static,
    ) -> Result<Self, &'static str> {
        if !floor_height_m.is_finite() {
            return Err("floor height must be finite");
        }
        validate_faces(&faces)?;
        for (index, face) in faces.iter().enumerate() {
            if faces[..index]
                .iter()
                .any(|other| other.target == face.target)
            {
                return Err("duplicate target identity");
            }
        }
        Ok(Self {
            faces,
            floor_height_m,
            step_duration,
            setup: Box::new(setup),
        })
    }

    fn build(&self) -> Result<WorldPhysics, &'static str> {
        let mut physics =
            WorldPhysics::with_step_duration(&self.faces, self.floor_height_m, self.step_duration);
        (self.setup)(&mut physics)?;
        Ok(physics)
    }
}

fn validate_faces(faces: &[TargetFace]) -> Result<(), &'static str> {
    if faces
        .iter()
        .any(|face| !crate::projectile::pose_is_valid(face.pose))
    {
        return Err("target poses must be finite with unit rotations");
    }
    Ok(())
}

/// A single-owner, caller-paced physical session with repeatable scenario reset.
///
/// All interacting bodies use the same world and ordinary physical step. Commands
/// retain physical motor, power and defeat limits; there is no HP, heat, referee,
/// task/reward model or wall-clock pacing. Compact truth reads do not capture a
/// checkpoint. See [`RobotScenario`] for the deliberately limited reset contract.
///
/// ```
/// use rm_simulator_physics::{Pose, StepDuration, Team, chassis::ChassisConfig,
///     session::{RobotScenario, RobotSession, RobotCommand}};
/// let scenario = RobotScenario::new(vec![], 0., StepDuration::HZ_1000, |world| {
///     world.add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 0., 0.3]))?;
///     Ok(())
/// })?;
/// let mut session = RobotSession::new(scenario)?;
/// session.command(RobotCommand::Drive { robot: 0, command: Default::default() })?;
/// session.step(10)?;
/// assert_eq!(session.stamp().time_ns, 10_000_000);
/// assert!(session.robot_truth(0).is_some());
/// session.reset()?;
/// assert_eq!(session.stamp().tick, 0);
/// # Ok::<(), &'static str>(())
/// ```
pub struct RobotSession {
    physics: WorldPhysics,
    scenario: RobotScenario,
    faces: Vec<TargetFace>,
    frames: TargetFrames,
    stamp: Stamp,
    commands: Vec<RobotCommand>,
}
impl RobotSession {
    /// Build the initial scenario, returning setup errors without a partial session.
    pub fn new(scenario: RobotScenario) -> Result<Self, &'static str> {
        let physics = scenario.build()?;
        Ok(Self {
            physics,
            commands: Vec::new(),
            faces: scenario.faces.clone(),
            frames: TargetFrames::new(scenario.faces.clone()),
            scenario,
            stamp: Stamp {
                tick: 0,
                time_ns: 0,
            },
        })
    }

    /// Read completed physical steps and elapsed nanoseconds without copying state.
    pub fn stamp(&self) -> Stamp {
        self.stamp
    }

    /// Immutable integration duration used by this session.
    pub fn step_duration(&self) -> StepDuration {
        self.physics.step_duration()
    }

    /// Inspect physical truth and query geometry. This is privileged, not a sensor.
    pub fn ground_truth(&self) -> &WorldPhysics {
        &self.physics
    }

    /// Access ordinary physical setup, payload/mechanism control and sensor hooks.
    ///
    /// Do not call `step` or `mark_synced` here: the session owns time. Placement,
    /// model/topology changes and resets invalidate caller sensor interpolation.
    /// Edits here do not change the initial scenario recipe used by reset.
    pub fn physics_mut(&mut self) -> &mut WorldPhysics {
        &mut self.physics
    }

    /// Read allocation-free controller truth without model geometry or wheel arrays.
    /// The returned pose, input, revision and defeat state share [`Self::stamp`].
    pub fn robot_control_state(&self, robot: u32) -> Option<crate::chassis::ChassisControlState> {
        self.physics.chassis_control_state(robot)
    }

    /// Read one robot's truth without constructing a full physical checkpoint.
    /// This clones its model configuration and wheel arrays; use
    /// [`Self::robot_control_state`] for an allocation-free controller observation.
    pub fn robot_truth(&self, robot: u32) -> Option<RobotTruth> {
        self.physics
            .chassis_snapshot(robot)
            .map(|state| RobotTruth {
                stamp: self.stamp,
                state,
            })
    }

    /// Capture the current LiDAR scene with the session's authoritative timestamp.
    /// Acquisition, interpolation and simulated sensor state remain caller-owned.
    pub fn lidar_scene_capture(
        &self,
        robot: u32,
    ) -> Result<crate::sensors::rolling::SceneCapture, &'static str> {
        self.physics.lidar_scene_capture(robot, self.stamp.time_ns)
    }

    /// Apply a request immediately through existing controls, advancing no time.
    pub fn command(&mut self, request: RobotCommand) -> Result<CommandResult, &'static str> {
        match request {
            RobotCommand::Drive { robot, command } => {
                self.physics.command_chassis(robot, command)?;
                Ok(CommandResult::Driven)
            }
            RobotCommand::WheelTorques { robot, torques } => {
                self.physics.command_chassis_wheel_torques(robot, torques)?;
                Ok(CommandResult::Driven)
            }
            RobotCommand::Fire { robot, shot } => {
                let muzzle = self
                    .physics
                    .chassis_muzzle_pose(robot)
                    .ok_or("unknown chassis")?;
                self.physics
                    .fire(self.stamp.time_ns, muzzle, shot, Some(robot))
                    .map(CommandResult::Fired)
            }
        }
    }

    /// Advance physical steps with stationary external targets; return ordered contacts.
    /// Armor collisions and projectile removal events remain in the ordinary world's
    /// queues, drained through [`Self::physics_mut`]. On error, prior steps remain.
    pub fn step(&mut self, steps: u64) -> Result<Vec<Contact>, &'static str> {
        self.check_advance(steps)?;
        let mut contacts = Vec::new();
        for _ in 0..steps {
            contacts.extend(self.integrate(None)?);
        }
        Ok(contacts)
    }

    fn check_advance(&self, steps: u64) -> Result<(), &'static str> {
        self.stamp
            .tick
            .checked_add(steps)
            .ok_or("physics tick overflow")?;
        steps
            .checked_mul(self.step_duration().as_ns())
            .and_then(|elapsed| self.stamp.time_ns.checked_add(elapsed))
            .ok_or("physics time overflow")?;
        Ok(())
    }

    fn integrate(&mut self, end: Option<&[TargetFace]>) -> Result<Vec<Contact>, &'static str> {
        if let Some(end) = end {
            validate_faces(end)?;
            self.frames.update(|faces| {
                faces.clear();
                faces.extend_from_slice(end);
            })?;
        }
        if end.is_none() {
            self.frames.update(|faces| faces.clone_from(&self.faces))?;
        }
        self.frames.begin(|faces| faces.clone_from(&self.faces))?;
        let contacts = self.physics.step(self.stamp.time_ns, &self.frames)?;
        if let Some(end) = end {
            self.faces.clone_from_slice(end);
        }
        self.stamp.tick += 1;
        self.stamp.time_ns += self.step_duration().as_ns();
        Ok(contacts)
    }

    /// Advance one tick interpolating external targets from their last endpoint.
    /// Count, identity order and finite unit poses are checked before integration.
    /// Resolve prescribed mechanisms through [`Self::physics_mut`] before stepping.
    pub fn step_with_targets(&mut self, end: &[TargetFace]) -> Result<Vec<Contact>, &'static str> {
        self.check_advance(1)?;
        self.integrate(Some(end))
    }

    /// Run a controller before every physical tick, applying its requests in order.
    /// Command storage is retained across calls. Errors retain earlier accepted
    /// requests/steps and stop.
    /// The controller sees privileged truth; sensor adapters select their own inputs.
    pub fn step_controlled(
        &mut self,
        steps: u64,
        mut controller: impl FnMut(&Self, &mut Vec<RobotCommand>),
    ) -> Result<Vec<Contact>, &'static str> {
        self.check_advance(steps)?;
        let mut commands = std::mem::take(&mut self.commands);
        let result = (|| {
            let mut contacts = Vec::new();
            for _ in 0..steps {
                commands.clear();
                controller(self, &mut commands);
                for request in commands.drain(..) {
                    self.command(request)?;
                }
                contacts.extend(self.integrate(None)?);
            }
            Ok(contacts)
        })();
        commands.clear();
        self.commands = commands;
        result
    }

    /// Rebuild the initial recipe transactionally, discarding all runtime edits.
    /// Failure leaves live bodies, time and event/trace buffers untouched. Success
    /// restarts time and IDs; caller sensor/controller state must be reset separately.
    /// This is scenario reconstruction, not arbitrary checkpoint continuation.
    pub fn reset(&mut self) -> Result<(), &'static str> {
        let physics = self.scenario.build()?;
        self.physics = physics;
        self.faces.clone_from(&self.scenario.faces);
        self.frames = TargetFrames::new(self.faces.clone());
        self.stamp = Stamp {
            tick: 0,
            time_ns: 0,
        };
        Ok(())
    }

    /// Transfer the sole physical world to another caller, ending session ownership.
    pub fn into_physics(self) -> WorldPhysics {
        self.physics
    }
}

/// Explicit phase-preserving nanosecond sampling cadence, with no implicit sensor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SampleSchedule {
    period_ns: u64,
    next_ns: Option<u64>,
}
impl SampleSchedule {
    /// Set a positive sampling period and first timestamp, in nanoseconds.
    pub fn new(period_ns: u64, first_ns: u64) -> Result<Self, &'static str> {
        if period_ns == 0 {
            return Err("sample period must be positive");
        }
        Ok(Self {
            period_ns,
            next_ns: Some(first_ns),
        })
    }

    /// Consume one due timestamp; drain repeatedly to catch up. Overflow exhausts it.
    ///
    /// ```
    /// use rm_simulator_physics::session::SampleSchedule;
    /// let mut schedule = SampleSchedule::new(3, 2)?;
    /// assert_eq!(schedule.pop_due(5), Some(2));
    /// assert_eq!(schedule.pop_due(5), Some(5));
    /// assert_eq!(schedule.pop_due(5), None);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn pop_due(&mut self, now_ns: u64) -> Option<u64> {
        let next = self.next_ns.filter(|&next| next <= now_ns)?;
        self.next_ns = next.checked_add(self.period_ns);
        Some(next)
    }
}
