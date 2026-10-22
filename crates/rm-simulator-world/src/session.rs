// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Match sessions composing physical simulation and rules without transport or pacing.
//!
//! This is a trusted embedding API, not a multiplayer authority boundary. External
//! pilots must still use server commands and their ownership checks. Observations
//! here are explicitly ground truth; simulated sensors belong in caller adapters.
/// Bounded stepping over independent complete match sessions.
pub mod batch;

use crate::{Field, FieldError, FieldSnapshot, RefereeCommand, StaticGeometry};

/// Shared physical commands, observations and timing primitives used by both session layers.
pub use rm_simulator_physics::session::{
    CommandResult, RobotCommand, RobotTruth, SampleSchedule, Stamp,
};

/// Complete in-memory reset point, including shared immutable collision geometry.
///
/// Physics solver caches are rebuilt on restore, with the same limitations as
/// [`Field::restore`]. Capture is explicit; normal stepping does not clone it.
#[derive(Clone)]
pub struct MatchCheckpoint {
    snapshot: FieldSnapshot,
    geometry: StaticGeometry,
    floor_height_m: f64,
}
impl MatchCheckpoint {
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

/// A caller-paced match composition with physics, optional native rules and reset.
///
/// Unlike a bare physical robot session, this facade owns a [`Field`] and its
/// complete rule/assembly state. A rule-free Field remains accepted for existing
/// embedding callers; new physics-only consumers should use the physics session.
///
/// Configure geometry and robots on a [`Field`] before moving it here. This
/// facade uses the same physical/rule paths as the multiplayer host, without a
/// socket, host worker, wall clock or renderer. It does not impose a task/reward
/// model. One call to [`Self::step`] can run faster or slower than real time.
///
/// ```
/// use rm_simulator_world::{Field, FieldConfig, session::MatchSession};
/// let field = Field::new(&FieldConfig::default())?;
/// let mut session = MatchSession::new(field);
/// session.step(128)?;
/// assert_eq!(session.stamp().time_ns, 1_000_000_000);
/// session.reset(42)?;
/// assert_eq!(session.stamp().time_ns, 0);
/// # Ok::<(), rm_simulator_world::FieldError>(())
/// ```
pub struct MatchSession {
    field: Field,
    reset_point: MatchCheckpoint,
    commands: Vec<RobotCommand>,
}
impl MatchSession {
    /// Own an already configured field and capture its initial reset point.
    pub fn new(field: Field) -> Self {
        let reset_point = MatchCheckpoint::capture(&field);
        Self {
            field,
            reset_point,
            commands: Vec::new(),
        }
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

    /// Read allocation-free controller truth without geometry or wheel arrays.
    /// This is privileged pose/input/revision/defeat state at [`Self::stamp`].
    pub fn robot_control_state(&self, robot: u32) -> Option<crate::chassis::ChassisControlState> {
        self.field.chassis_control_state(robot)
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
        Self::apply_command(&mut self.field, request)
    }

    fn apply_command(
        field: &mut Field,
        request: RobotCommand,
    ) -> Result<CommandResult, FieldError> {
        match request {
            RobotCommand::Drive { robot, command } => {
                field.command_chassis(robot, command)?;
                Ok(CommandResult::Driven)
            }
            RobotCommand::WheelTorques { robot, torques } => {
                field.command_chassis_wheel_torques(robot, torques)?;
                Ok(CommandResult::Driven)
            }
            RobotCommand::Fire { robot, shot } => {
                let muzzle = field
                    .chassis_muzzle_pose(robot)
                    .ok_or(FieldError::Chassis("unknown chassis"))?;
                field
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
    /// The command buffer is retained across calls, cleared and reused each step.
    /// Requests apply in order; an error retains earlier accepted requests/steps
    /// and stops the run. The controller sees truth deliberately; sensor adapters should
    /// provide their own observations instead of exposing this field directly.
    pub fn step_controlled(
        &mut self,
        steps: u64,
        mut controller: impl FnMut(&Field, &mut Vec<RobotCommand>),
    ) -> Result<Stamp, FieldError> {
        for _ in 0..steps {
            self.commands.clear();
            controller(&self.field, &mut self.commands);
            for request in self.commands.drain(..) {
                Self::apply_command(&mut self.field, request)?;
            }
            self.field.step(1)?;
        }
        Ok(self.stamp())
    }

    /// Capture a branch point; does not replace the episode reset point.
    pub fn checkpoint(&self) -> MatchCheckpoint {
        MatchCheckpoint::capture(&self.field)
    }

    /// Restore a branch point transactionally. Failure leaves this session intact.
    pub fn restore(&mut self, checkpoint: &MatchCheckpoint) -> Result<(), FieldError> {
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

/// Compatibility name for [`MatchSession`]. New callers should name the session layer explicitly.
pub type Session = MatchSession;

/// Compatibility name for [`MatchCheckpoint`], including complete native match state.
pub type Checkpoint = MatchCheckpoint;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        ChassisCommand, ChassisConfig, ChassisPlacement, FieldConfig, Pose, RobotKind, Team,
    };

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
        let mut session = MatchSession::new(field);
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
    fn match_checkpoint_retains_authority_and_legacy_names_remain_compatible() {
        use crate::gameplay::rmul_2027::{Format, Phase};
        let field = Field::new(&FieldConfig {
            rmul_2027: Some(Format::Infantry),
            ..Default::default()
        })
        .unwrap();
        let mut session: Session = MatchSession::new(field);
        session.referee(RefereeCommand::StartMatch).unwrap();
        session.step(32).unwrap();
        let checkpoint: Checkpoint = session.checkpoint();
        let expected = session.ground_truth().rmul_2027().unwrap().clone();
        assert_eq!(expected.phase, Phase::Countdown);
        session.step(16).unwrap();
        assert_ne!(session.ground_truth().rmul_2027(), Some(&expected));
        session.restore(&checkpoint).unwrap();
        assert_eq!(session.stamp().tick, 32);
        assert_eq!(session.ground_truth().rmul_2027(), Some(&expected));
        session.reset(123).unwrap();
        assert_eq!(session.stamp().tick, 0);
        assert_eq!(
            session.ground_truth().rmul_2027().unwrap().phase,
            Phase::Idle
        );
    }

    #[test]
    fn controller_storage_survives_calls_and_failed_commands_preserve_order() {
        let mut session = MatchSession::new(scenario());
        let drive = |robot, forward_m_s| RobotCommand::Drive {
            robot,
            command: ChassisCommand {
                forward_m_s,
                ..Default::default()
            },
        };
        assert!(
            session
                .step_controlled(1, |_, commands| {
                    commands.push(drive(0, 1.));
                    commands.push(drive(999, 2.));
                    commands.push(drive(0, 3.));
                })
                .is_err()
        );
        assert_eq!(session.stamp().tick, 0);
        assert_eq!(
            session.robot_truth(0).unwrap().state.command.forward_m_s,
            1.
        );
        assert!(session.commands.is_empty());
        let storage = session.commands.as_ptr();
        let capacity = session.commands.capacity();
        for _ in 0..16 {
            session
                .step_controlled(1, |_, commands| {
                    commands.push(drive(0, 0.5));
                    commands.push(drive(0, 0.));
                })
                .unwrap();
            assert_eq!(session.commands.as_ptr(), storage);
            assert_eq!(session.commands.capacity(), capacity);
            assert_eq!(
                session.robot_truth(0).unwrap().state.command.forward_m_s,
                0.
            );
        }
        assert_eq!(session.stamp().tick, 16);
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
