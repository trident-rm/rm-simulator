// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! In-process simulation sessions without transport, pacing, rendering or workers.
//!
//! This is a trusted embedding API, not a multiplayer authority boundary. External
//! pilots must still use server commands and their ownership checks. Observations
//! here are explicitly ground truth; simulated sensors belong in caller adapters.
use crate::{
    ChassisCommand, ChassisSnapshot, Field, FieldError, FieldSnapshot, RefereeCommand, Shot,
    StaticGeometry,
};

/// Simulation timestamp. Nanoseconds are authoritative across different step rates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Stamp {
    /// Completed simulation integration steps since field construction.
    pub tick: u64,
    /// Elapsed simulation nanoseconds, independent of the host clock.
    pub time_ns: u64,
}

/// Complete in-memory reset point, including shared immutable collision geometry.
///
/// Physics solver caches are rebuilt on restore, with the same limitations as
/// [`Field::restore`]. Capture is explicit; normal stepping does not clone it.
#[derive(Clone)]
pub struct Checkpoint {
    snapshot: FieldSnapshot,
    geometry: StaticGeometry,
    floor_height_m: f64,
}
impl Checkpoint {
    /// Capture the entire physical field and native match authority.
    pub fn capture(field: &Field) -> Self {
        Self {
            snapshot: field.snapshot(),
            geometry: field.static_geometry_snapshot(),
            floor_height_m: field.floor_height_m(),
        }
    }

    /// Inspect checkpoint truth, including hidden rule state; this is not a sensor view.
    pub fn snapshot(&self) -> &FieldSnapshot {
        &self.snapshot
    }

    /// Reconstruct an independent field sharing immutable collision meshes.
    pub fn restore(&self) -> Result<Field, FieldError> {
        Field::restore(&self.snapshot, &self.geometry, self.floor_height_m)
    }
}

/// An addressed high-level robot request for a trusted in-process controller.
#[derive(Clone, Debug, PartialEq)]
pub enum RobotCommand {
    /// Body velocity, stabilized gimbal target and mechanism setpoints.
    Drive {
        /// Existing field-assigned robot identity.
        robot: u32,
        /// Validated through the ordinary field control path.
        command: ChassisCommand,
    },
    /// Bypass velocity tracking with wheel-axis torque requests. Physical motor,
    /// power and defeat limits still apply; gimbal/folding targets are retained.
    WheelTorques {
        /// Existing field-assigned robot identity.
        robot: u32,
        /// Newton metre requests in the configured wheel roster order.
        torques: rm_simulator_physics::control::WheelTorques,
    },
    /// Launch from the robot's actual authoritative muzzle, applying native rules.
    Fire {
        /// Existing field-assigned robot identity.
        robot: u32,
        /// Caliber and speed in metres per second.
        shot: Shot,
    },
}

/// Result of an accepted robot request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandResult {
    /// A drive setpoint was accepted; no time has advanced.
    Driven,
    /// A projectile was launched, identified by its field identity.
    Fired(u64),
}

/// Timestamped physical robot truth, without the full match checkpoint.
///
/// Encoders, IMU estimates, camera frames and radio information must be derived
/// separately. Reading this object grants perfect pose and velocity information.
#[derive(Clone, Debug, PartialEq)]
pub struct RobotTruth {
    /// Time at which the state was read.
    pub stamp: Stamp,
    /// Actual physical state, wheel telemetry and last control request.
    pub state: ChassisSnapshot,
}

/// A caller-paced, single-owner simulation with explicit reset and truth access.
///
/// Configure geometry and robots on a [`Field`] before moving it here. This
/// facade uses the same physical/rule paths as the multiplayer host, without a
/// socket, host worker, wall clock or renderer. It does not impose a task/reward
/// model. One call to [`Self::step`] can run faster or slower than real time.
///
/// ```
/// use rm_simulator_world::{Field, FieldConfig, session::Session};
/// let field = Field::new(&FieldConfig::default())?;
/// let mut session = Session::new(field);
/// session.step(128)?;
/// assert_eq!(session.stamp().time_ns, 1_000_000_000);
/// session.reset(42)?;
/// assert_eq!(session.stamp().time_ns, 0);
/// # Ok::<(), rm_simulator_world::FieldError>(())
/// ```
pub struct Session {
    field: Field,
    reset_point: Checkpoint,
}
impl Session {
    /// Own an already configured field and capture its initial reset point.
    pub fn new(field: Field) -> Self {
        let reset_point = Checkpoint::capture(&field);
        Self { field, reset_point }
    }

    /// Read simulation time without advancing or copying the world.
    pub fn stamp(&self) -> Stamp {
        Stamp {
            tick: self.field.tick(),
            time_ns: self.field.time_ns(),
        }
    }

    /// Read native field truth. This grants privileged visibility, not sensor access.
    pub fn ground_truth(&self) -> &Field {
        &self.field
    }

    /// Read one robot's actual state; unknown identities return `None`.
    pub fn robot_truth(&self, robot: u32) -> Option<RobotTruth> {
        self.field.chassis_snapshot(robot).map(|state| RobotTruth {
            stamp: self.stamp(),
            state,
        })
    }

    /// Apply one request immediately, without advancing time.
    ///
    /// This is trusted control: transport ownership and weapon cadence belong
    /// to the server. Native phase, heat, allowance and defeat checks still run.
    pub fn command(&mut self, request: RobotCommand) -> Result<CommandResult, FieldError> {
        match request {
            RobotCommand::Drive { robot, command } => {
                self.field.command_chassis(robot, command)?;
                Ok(CommandResult::Driven)
            }
            RobotCommand::WheelTorques { robot, torques } => {
                self.field.command_chassis_wheel_torques(robot, torques)?;
                Ok(CommandResult::Driven)
            }
            RobotCommand::Fire { robot, shot } => {
                let muzzle = self
                    .field
                    .chassis_muzzle_pose(robot)
                    .ok_or(FieldError::Chassis("unknown chassis"))?;
                self.field
                    .fire(muzzle, shot, Some(robot))
                    .map(CommandResult::Fired)
            }
        }
    }

    /// Apply a privileged native referee operation, without advancing time.
    pub fn referee(&mut self, command: RefereeCommand) -> Result<(), FieldError> {
        self.field.referee_command(command)
    }

    /// Advance a count of the field's configured integration steps.
    pub fn step(&mut self, steps: u64) -> Result<Stamp, FieldError> {
        self.field.step(steps)?;
        Ok(self.stamp())
    }

    /// Call an in-process controller before every physical step, then integrate.
    ///
    /// The command buffer is cleared and reused each step. Requests apply in
    /// order; an error retains previously accepted requests/steps and stops the
    /// run. The controller sees truth deliberately; sensor adapters should
    /// provide their own observations instead of exposing this field directly.
    pub fn step_controlled(
        &mut self,
        steps: u64,
        mut controller: impl FnMut(&Field, &mut Vec<RobotCommand>),
    ) -> Result<Stamp, FieldError> {
        let mut commands = Vec::new();
        for _ in 0..steps {
            commands.clear();
            controller(&self.field, &mut commands);
            for request in commands.drain(..) {
                self.command(request)?;
            }
            self.field.step(1)?;
        }
        Ok(self.stamp())
    }

    /// Capture a branch point; does not replace the episode reset point.
    pub fn checkpoint(&self) -> Checkpoint {
        Checkpoint::capture(&self.field)
    }

    /// Restore a branch point transactionally. Failure leaves this session intact.
    pub fn restore(&mut self, checkpoint: &Checkpoint) -> Result<(), FieldError> {
        self.field = checkpoint.restore()?;
        Ok(())
    }

    /// Restore the initial scenario and seed the native Rune stream.
    ///
    /// Other randomized scenarios remain caller-owned and must use their own
    /// explicit seeds. The initial clock is restored, even for a reset point
    /// originally captured after time zero. No randomized layout is invented.
    pub fn reset(&mut self, seed: u64) -> Result<(), FieldError> {
        let mut field = self.reset_point.restore()?;
        field.set_rmuc_2027_rune_seed(seed);
        self.field = field;
        Ok(())
    }

    /// Recover the field for further setup or transfer to the multiplayer host.
    pub fn into_field(self) -> Field {
        self.field
    }
}

/// An explicit nanosecond sampling cadence for future sensor/controller adapters.
///
/// This schedules timestamps only; it does not render, simulate sensors or read
/// host time. The caller must sample/interpolate the world at the returned time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SampleSchedule {
    period_ns: u64,
    next_ns: Option<u64>,
}
impl SampleSchedule {
    /// Create a schedule with a positive period and first timestamp, in nanoseconds.
    pub fn new(period_ns: u64, first_ns: u64) -> Result<Self, &'static str> {
        if period_ns == 0 {
            return Err("sample period must be positive");
        }
        Ok(Self {
            period_ns,
            next_ns: Some(first_ns),
        })
    }

    /// Consume one due timestamp at or before `now_ns`, preserving exact phase.
    /// Call again to drain missed samples; overflow exhausts the schedule.
    ///
    /// ```
    /// use rm_simulator_world::session::SampleSchedule;
    /// let mut camera = SampleSchedule::new(10_000_000, 0)?;
    /// assert_eq!(camera.pop_due(15_000_000), Some(0));
    /// assert_eq!(camera.pop_due(15_000_000), Some(10_000_000));
    /// assert_eq!(camera.pop_due(15_000_000), None);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn pop_due(&mut self, now_ns: u64) -> Option<u64> {
        let next = self.next_ns.filter(|&next| next <= now_ns)?;
        self.next_ns = next.checked_add(self.period_ns);
        Some(next)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ChassisConfig, ChassisPlacement, FieldConfig, Pose, RobotKind, Team};

    fn scenario() -> Field {
        Field::new(&FieldConfig {
            chassis: vec![ChassisPlacement {
                config: ChassisConfig::default(),
                spawn: Pose::at([0., 0., 0.4]),
                team: Team::Red,
                kind: RobotKind::Infantry,
                performance: None,
            }],
            ..Default::default()
        })
        .unwrap()
    }

    #[test]
    fn control_runs_before_each_step_and_reset_preserves_scenario() {
        let mut session = Session::new(scenario());
        let mut called = Vec::new();
        session
            .step_controlled(8, |field, commands| {
                called.push(field.tick());
                commands.push(RobotCommand::Drive {
                    robot: 0,
                    command: ChassisCommand {
                        forward_m_s: 1.,
                        ..Default::default()
                    },
                });
            })
            .unwrap();
        assert_eq!(called, (0..8).collect::<Vec<_>>());
        assert_eq!(
            session.robot_truth(0).unwrap().state.command.forward_m_s,
            1.
        );
        let branch = session.checkpoint();
        session.step(4).unwrap();
        session.restore(&branch).unwrap();
        assert_eq!(session.stamp().tick, 8);
        session.reset(17).unwrap();
        assert_eq!(session.stamp().tick, 0);
        assert_eq!(
            session.robot_truth(0).unwrap().state.command.forward_m_s,
            0.
        );
        assert!(session.robot_truth(999).is_none());
    }

    #[test]
    fn checkpoint_retains_geometry_and_failed_restore_is_transactional() {
        let mut field = scenario();
        field
            .add_static_mesh(
                vec![[1., 0., 0.], [2., 0., 0.], [1., 1., 0.]],
                vec![[0, 1, 2]],
            )
            .unwrap();
        let mut session = Session::new(field);
        session.step(3).unwrap();
        let mut checkpoint = session.checkpoint();
        checkpoint.snapshot.time_ns += 1;
        let before = session.stamp();
        assert!(session.restore(&checkpoint).is_err());
        assert_eq!(session.stamp(), before);
        session.reset(0).unwrap();
        assert_eq!(session.ground_truth().static_geometry().1.len(), 1);
    }

    #[test]
    fn sampling_is_phase_preserving_and_overflow_safe() {
        assert!(SampleSchedule::new(0, 0).is_err());
        let mut schedule = SampleSchedule::new(3, 2).unwrap();
        let mut due = Vec::new();
        while let Some(time) = schedule.pop_due(10) {
            due.push(time);
        }
        assert_eq!(due, vec![2, 5, 8]);
        assert_eq!(schedule.pop_due(11), Some(11));
        let mut schedule = SampleSchedule::new(2, u64::MAX).unwrap();
        assert_eq!(schedule.pop_due(u64::MAX), Some(u64::MAX));
        assert_eq!(schedule.pop_due(u64::MAX), None);
    }

    #[test]
    fn high_rate_control_and_held_torque_checkpoint_share_the_field_clock() {
        use crate::{DriveControl, StepDuration, WheelTorques};
        let mut config = FieldConfig {
            step_duration: StepDuration::HZ_1000,
            ..Default::default()
        };
        config.chassis = scenario()
            .snapshot()
            .chassis
            .into_iter()
            .map(|robot| ChassisPlacement {
                config: robot.config,
                spawn: robot.pose,
                team: robot.team,
                kind: RobotKind::Infantry,
                performance: None,
            })
            .collect();
        let mut session = Session::new(Field::new(&config).unwrap());
        let torques = WheelTorques::new(vec![0.1; 4]).unwrap();
        session
            .command(RobotCommand::WheelTorques {
                robot: 0,
                torques: torques.clone(),
            })
            .unwrap();
        let mut calls = 0;
        session
            .step_controlled(1000, |field, _| {
                assert_eq!(field.time_ns(), calls * 1_000_000);
                calls += 1;
            })
            .unwrap();
        assert_eq!(calls, 1000);
        assert_eq!(session.stamp().time_ns, 1_000_000_000);
        let checkpoint = session.checkpoint();
        session
            .command(RobotCommand::Drive {
                robot: 0,
                command: Default::default(),
            })
            .unwrap();
        session.restore(&checkpoint).unwrap();
        assert_eq!(
            session.robot_truth(0).unwrap().state.drive_control,
            DriveControl::WheelTorques(torques)
        );
        assert_eq!(
            session.ground_truth().step_duration(),
            StepDuration::HZ_1000
        );
        session.reset(42).unwrap();
        assert_eq!(session.stamp().time_ns, 0);
        assert_eq!(
            session.ground_truth().step_duration(),
            StepDuration::HZ_1000
        );
        assert_eq!(
            session.robot_truth(0).unwrap().state.drive_control,
            DriveControl::BodyVelocity
        );
    }
}
