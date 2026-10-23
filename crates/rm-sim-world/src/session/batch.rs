// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Bounded CPU rollouts through the ordinary complete-match sessions.
//!
//! Each slot owns its physics, clock and native authority. Task/reward state,
//! sensor schedules and RNGs remain caller-owned. Sequential execution remains
//! the default; [`parallel`] supplies opt-in scoped workers without a transport.

/// Opt-in bounded CPU execution through ordinary match sessions.
pub mod parallel;

use super::{MatchCheckpoint, MatchSession, RobotCommand, Stamp};
use crate::FieldError;

use rm_sim_physics::session::batch::validate_selection;
/// Shared bounds, requests, observations and explicit partial-progress errors.
pub use rm_sim_physics::session::batch::{
    BatchError, BatchFailure, BatchLimits, BatchStep, ControlObservation, RobotQuery,
};

/// Select an initial match checkpoint and its explicit native Rune seed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MatchReset {
    /// Stable session slot; duplicates in one reset are rejected.
    pub session: usize,
    /// Seed for the ordinary native Rune stream; no scenario RNG is invented.
    pub seed: u64,
}

/// Bounded single-owner collection of independent complete match sessions.
///
/// Mixed integration durations are supported. Queued commands apply before each
/// selected advance. Preflight failure changes nothing. Runtime errors preserve
/// the completed prefix and report the failing slot's before/after timestamps;
/// remaining commands in its queue are discarded and later queues are untouched.
/// Native phase, heat, allowance and defeat checks remain ordinary match checks.
///
/// ```
/// use rm_sim_world::{Field, FieldConfig, session::{MatchSession,
///     batch::{MatchBatch, MatchReset, BatchLimits, BatchStep}}};
/// let mut batch = MatchBatch::new(vec![MatchSession::new(
///     Field::new(&FieldConfig::default())?)], BatchLimits {
///     max_sessions: 8, max_steps_per_session: 128, max_commands_per_session: 4,
/// })?;
/// let mut stamps = Vec::new();
/// batch.step(&[BatchStep { session: 0, steps: 128 }], &mut stamps)?;
/// assert_eq!(stamps[0].time_ns, 1_000_000_000);
/// batch.reset(&[MatchReset { session: 0, seed: 42 }])?;
/// assert_eq!(batch.sessions()[0].stamp().tick, 0);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub struct MatchBatch {
    sessions: Vec<MatchSession>,
    commands: Vec<Vec<RobotCommand>>,
    limits: BatchLimits,
}
impl MatchBatch {
    /// Admit owned sessions without copying worlds. No sessions can be added later.
    pub fn new(
        sessions: Vec<MatchSession>,
        limits: BatchLimits,
    ) -> Result<Self, BatchError<FieldError>> {
        if limits.max_sessions == 0 || sessions.len() > limits.max_sessions {
            return Err(BatchError::Invalid {
                request: 0,
                reason: "invalid session capacity",
            });
        }
        Ok(Self {
            commands: (0..sessions.len()).map(|_| Vec::new()).collect(),
            sessions,
            limits,
        })
    }

    /// Inspect independent session truth; slot order remains stable for this batch.
    pub fn sessions(&self) -> &[MatchSession] {
        &self.sessions
    }

    /// Access a slot for ordinary commands and caller-owned sensor acquisition.
    /// Changes do not affect other slots or queued batch commands.
    pub fn session_mut(&mut self, session: usize) -> Option<&mut MatchSession> {
        self.sessions.get_mut(session)
    }

    /// Reuse a slot's pending command buffer. Limits are checked by [`Self::step`].
    /// Queued commands are applied once before that slot advances, including zero steps.
    pub fn commands_mut(&mut self, session: usize) -> Option<&mut Vec<RobotCommand>> {
        self.commands.get_mut(session)
    }

    /// Apply queued commands and advance selected slots in request order.
    /// All indices, duplicates, work bounds and clock overflows are checked first.
    /// Success output contains one final stamp per request, in request order.
    pub fn step(
        &mut self,
        requests: &[BatchStep],
        output: &mut Vec<Stamp>,
    ) -> Result<(), BatchError<FieldError>> {
        self.validate_steps(requests)?;
        output.clear();
        for (request, item) in requests.iter().enumerate() {
            let session = &mut self.sessions[item.session];
            let before = session.stamp();
            let command_count = self.commands[item.session].len();
            for (accepted_commands, command) in self.commands[item.session].drain(..).enumerate() {
                if let Err(source) = session.command(command) {
                    return Err(BatchError::Runtime(BatchFailure {
                        request,
                        session: item.session,
                        completed_requests: output.len(),
                        accepted_commands,
                        before,
                        after: session.stamp(),
                        source,
                    }));
                }
            }
            if let Err(source) = session.step(item.steps) {
                return Err(BatchError::Runtime(BatchFailure {
                    request,
                    session: item.session,
                    completed_requests: output.len(),
                    accepted_commands: command_count,
                    before,
                    after: session.stamp(),
                    source,
                }));
            }
            output.push(session.stamp());
        }
        Ok(())
    }

    fn validate_steps(&self, requests: &[BatchStep]) -> Result<(), BatchError<FieldError>> {
        if requests.len() > self.limits.max_sessions {
            return Err(BatchError::Invalid {
                request: 0,
                reason: "too many requests",
            });
        }
        for (request, item) in requests.iter().enumerate() {
            let invalid = |reason| BatchError::Invalid { request, reason };
            let session = self
                .sessions
                .get(item.session)
                .ok_or_else(|| invalid("session index out of bounds"))?;
            if requests[..request]
                .iter()
                .any(|other| other.session == item.session)
            {
                return Err(invalid("duplicate session index"));
            }
            if item.steps > self.limits.max_steps_per_session {
                return Err(invalid("step limit exceeded"));
            }
            if self.commands[item.session].len() > self.limits.max_commands_per_session {
                return Err(invalid("command limit exceeded"));
            }
            session
                .stamp()
                .tick
                .checked_add(item.steps)
                .and_then(|tick| tick.checked_mul(session.ground_truth().step_duration().as_ns()))
                .ok_or_else(|| invalid("session clock overflow"))?;
        }
        Ok(())
    }

    /// Fill caller-owned observation storage in query order, retaining its capacity.
    /// Invalid indices/counts leave output unchanged; unknown robots produce `None`.
    pub fn observe_into(
        &self,
        queries: &[RobotQuery],
        output: &mut Vec<ControlObservation>,
    ) -> Result<(), BatchError<FieldError>> {
        if queries.len() > self.limits.max_sessions {
            return Err(BatchError::Invalid {
                request: 0,
                reason: "too many observations",
            });
        }
        for (request, query) in queries.iter().enumerate() {
            if query.session >= self.sessions.len() {
                return Err(BatchError::Invalid {
                    request,
                    reason: "session index out of bounds",
                });
            }
        }
        output.clear();
        output.extend(queries.iter().map(|query| {
            let session = &self.sessions[query.session];
            ControlObservation {
                session: query.session,
                robot: query.robot,
                stamp: session.stamp(),
                step_duration: session.ground_truth().step_duration(),
                state: session.robot_control_state(query.robot),
            }
        }));
        Ok(())
    }

    /// Capture selected match branch points, sharing immutable geometry.
    /// Invalid indices/duplicates leave output unchanged. The output vector retains
    /// capacity, but each checkpoint captures newly allocated dynamic/rule state.
    pub fn checkpoint_into(
        &self,
        indices: &[usize],
        output: &mut Vec<MatchCheckpoint>,
    ) -> Result<(), BatchError<FieldError>> {
        validate_selection(indices.iter().copied(), self.sessions.len(), self.limits)?;
        output.clear();
        output.extend(
            indices
                .iter()
                .map(|&index| self.sessions[index].checkpoint()),
        );
        Ok(())
    }

    /// Transactionally restore each selected initial match and seed its Rune stream.
    /// All replacement fields are built before swapping; failure leaves every live
    /// slot, command queue and clock unchanged. Peak memory includes replacements.
    /// Initial checkpoint clocks can be nonzero. Caller sensors/RNGs remain separate.
    pub fn reset(&mut self, requests: &[MatchReset]) -> Result<(), BatchError<FieldError>> {
        validate_selection(
            requests.iter().map(|request| request.session),
            self.sessions.len(),
            self.limits,
        )?;
        let mut prepared = Vec::with_capacity(requests.len());
        for (request, item) in requests.iter().enumerate() {
            let session = &self.sessions[item.session];
            let mut field = session.reset_point.restore().map_err(|source| {
                BatchError::Runtime(BatchFailure {
                    request,
                    session: item.session,
                    completed_requests: 0,
                    accepted_commands: 0,
                    before: session.stamp(),
                    after: session.stamp(),
                    source,
                })
            })?;
            field.set_rmuc_2027_rune_seed(item.seed);
            prepared.push(field);
        }
        for (item, field) in requests.iter().zip(prepared) {
            self.sessions[item.session].field = field;
            self.commands[item.session].clear();
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn session(duration: crate::StepDuration) -> MatchSession {
        MatchSession::new(
            crate::Field::new(&crate::FieldConfig {
                step_duration: duration,
                rmul_2027: Some(crate::gameplay::rmul_2027::Format::Infantry),
                chassis: vec![crate::ChassisPlacement {
                    config: crate::ChassisConfig::default(),
                    spawn: crate::Pose::at([0., 0., 0.4]),
                    team: crate::Team::Red,
                    kind: crate::RobotKind::Infantry,
                    performance: None,
                }],
                ..Default::default()
            })
            .unwrap(),
        )
    }
    fn limits() -> BatchLimits {
        BatchLimits {
            max_sessions: 4,
            max_steps_per_session: u64::MAX,
            max_commands_per_session: 4,
        }
    }
    fn batch() -> MatchBatch {
        MatchBatch::new(
            vec![
                session(crate::StepDuration::default()),
                session(crate::StepDuration::HZ_1000),
            ],
            limits(),
        )
        .unwrap()
    }
    fn drive(robot: u32, speed: f64) -> RobotCommand {
        RobotCommand::Drive {
            robot,
            command: crate::chassis::ChassisCommand {
                forward_m_s: speed,
                ..Default::default()
            },
        }
    }
    #[test]
    fn mixed_durations_grouping_and_buffers_preserve_independent_results() {
        let mut grouped = batch();
        let mut split = batch();
        for index in 0..2 {
            grouped.commands_mut(index).unwrap().push(drive(0, 0.5));
            split.commands_mut(index).unwrap().push(drive(0, 0.5));
        }
        let mut output = Vec::with_capacity(2);
        let storage = output.as_ptr();
        grouped
            .step(
                &[
                    BatchStep {
                        session: 1,
                        steps: 12,
                    },
                    BatchStep {
                        session: 0,
                        steps: 12,
                    },
                ],
                &mut output,
            )
            .unwrap();
        assert_eq!(output[0].time_ns, 12_000_000);
        assert_eq!(
            output[1].time_ns,
            12 * crate::StepDuration::default().as_ns()
        );
        for steps in [2, 3, 7] {
            for index in 0..2 {
                split
                    .step(
                        &[BatchStep {
                            session: index,
                            steps,
                        }],
                        &mut output,
                    )
                    .unwrap();
            }
        }
        assert_eq!(output.as_ptr(), storage);
        for index in 0..2 {
            assert_eq!(
                grouped.sessions()[index].robot_truth(0),
                split.sessions()[index].robot_truth(0)
            );
        }
        let queries = [
            RobotQuery {
                session: 0,
                robot: 0,
            },
            RobotQuery {
                session: 1,
                robot: 999,
            },
        ];
        let mut observed = Vec::with_capacity(2);
        grouped.observe_into(&queries, &mut observed).unwrap();
        let storage = observed.as_ptr();
        grouped.observe_into(&queries, &mut observed).unwrap();
        assert_eq!(storage, observed.as_ptr());
        assert!(observed[0].state.is_some());
        assert!(observed[1].state.is_none());
        assert_eq!(observed[1].step_duration, crate::StepDuration::HZ_1000);
        let before = observed.clone();
        assert!(
            grouped
                .observe_into(
                    &[RobotQuery {
                        session: 99,
                        robot: 0
                    }],
                    &mut observed
                )
                .is_err()
        );
        assert_eq!(observed, before);
    }
    #[test]
    fn preflight_rejects_all_work_before_mutating_any_slot_or_output() {
        let mut batch = batch();
        batch.commands_mut(0).unwrap().push(drive(0, 0.5));
        let mut output = vec![Stamp {
            tick: 99,
            time_ns: 99,
        }];
        for requests in [
            vec![
                BatchStep {
                    session: 0,
                    steps: 1,
                },
                BatchStep {
                    session: 9,
                    steps: 1,
                },
            ],
            vec![
                BatchStep {
                    session: 0,
                    steps: 1,
                },
                BatchStep {
                    session: 0,
                    steps: 1,
                },
            ],
            vec![
                BatchStep {
                    session: 0,
                    steps: 1,
                },
                BatchStep {
                    session: 1,
                    steps: u64::MAX,
                },
            ],
        ] {
            assert!(matches!(
                batch.step(&requests, &mut output),
                Err(BatchError::Invalid { .. })
            ));
            assert_eq!(batch.sessions()[0].stamp().tick, 0);
            assert_eq!(batch.commands_mut(0).unwrap().len(), 1);
            assert_eq!(output[0].tick, 99);
        }
        batch.limits.max_steps_per_session = 0;
        assert!(
            batch
                .step(
                    &[BatchStep {
                        session: 0,
                        steps: 1
                    }],
                    &mut output
                )
                .is_err()
        );
        batch.limits.max_steps_per_session = 1;
        batch.limits.max_commands_per_session = 0;
        assert!(
            batch
                .step(
                    &[BatchStep {
                        session: 0,
                        steps: 0
                    }],
                    &mut output
                )
                .is_err()
        );
        assert_eq!(
            batch.sessions()[0]
                .robot_truth(0)
                .unwrap()
                .state
                .command
                .forward_m_s,
            0.
        );
    }
    #[test]
    fn runtime_command_failure_exposes_prefix_and_discards_only_current_queue() {
        let mut batch = batch();
        batch
            .commands_mut(1)
            .unwrap()
            .extend([drive(0, 1.), drive(999, 2.), drive(0, 3.)]);
        let capacity = batch.commands_mut(1).unwrap().capacity();
        let mut output = Vec::new();
        let error = batch
            .step(
                &[
                    BatchStep {
                        session: 0,
                        steps: 2,
                    },
                    BatchStep {
                        session: 1,
                        steps: 3,
                    },
                ],
                &mut output,
            )
            .unwrap_err();
        let BatchError::Runtime(failure) = error else {
            panic!("expected runtime failure")
        };
        assert_eq!(failure.completed_requests, 1);
        assert_eq!(failure.accepted_commands, 1);
        assert_eq!(failure.request, 1);
        assert_eq!(failure.before, failure.after);
        assert_eq!(output.len(), 1);
        assert_eq!(batch.sessions()[0].stamp().tick, 2);
        assert_eq!(batch.sessions()[1].stamp().tick, 0);
        assert_eq!(
            batch.sessions()[1]
                .robot_truth(0)
                .unwrap()
                .state
                .command
                .forward_m_s,
            1.
        );
        assert!(batch.commands_mut(1).unwrap().is_empty());
        assert_eq!(batch.commands_mut(1).unwrap().capacity(), capacity);
        // Later requests and their queued input remain untouched.
        batch.commands_mut(0).unwrap().push(drive(0, 2.));
        batch.commands_mut(1).unwrap().push(drive(999, 0.));
        assert!(
            batch
                .step(
                    &[
                        BatchStep {
                            session: 1,
                            steps: 1
                        },
                        BatchStep {
                            session: 0,
                            steps: 1
                        }
                    ],
                    &mut output
                )
                .is_err()
        );
        assert_eq!(batch.commands_mut(0).unwrap().len(), 1);
        assert_eq!(batch.sessions()[0].stamp().tick, 2);
    }
    #[test]
    fn selective_reset_does_not_change_other_episode_or_queued_input() {
        let mut batch = batch();
        let mut output = Vec::new();
        batch
            .step(
                &[
                    BatchStep {
                        session: 0,
                        steps: 4,
                    },
                    BatchStep {
                        session: 1,
                        steps: 5,
                    },
                ],
                &mut output,
            )
            .unwrap();
        let untouched = batch.sessions()[1].robot_truth(0);
        batch.commands_mut(0).unwrap().push(drive(0, 1.));
        batch.commands_mut(1).unwrap().push(drive(0, 2.));
        batch
            .reset(&[MatchReset {
                session: 0,
                seed: 42,
            }])
            .unwrap();
        assert_eq!(batch.sessions()[0].stamp().tick, 0);
        assert!(batch.commands_mut(0).unwrap().is_empty());
        assert_eq!(batch.sessions()[1].robot_truth(0), untouched);
        assert_eq!(batch.commands_mut(1).unwrap().len(), 1);
        assert!(
            batch
                .reset(&[
                    MatchReset {
                        session: 1,
                        seed: 0
                    },
                    MatchReset {
                        session: 1,
                        seed: 1
                    }
                ])
                .is_err()
        );
        assert_eq!(batch.sessions()[1].robot_truth(0), untouched);
    }
    #[test]
    fn invalid_reset_checkpoint_is_transactional_across_selected_set() {
        let mut batch = batch();
        let mut output = Vec::new();
        batch
            .step(
                &[
                    BatchStep {
                        session: 0,
                        steps: 3,
                    },
                    BatchStep {
                        session: 1,
                        steps: 4,
                    },
                ],
                &mut output,
            )
            .unwrap();
        let before = batch.sessions()[0].robot_truth(0);
        batch.commands_mut(0).unwrap().push(drive(0, 0.5));
        batch.sessions[1].reset_point.snapshot.time_ns += 1;
        let error = batch
            .reset(&[
                MatchReset {
                    session: 0,
                    seed: 42,
                },
                MatchReset {
                    session: 1,
                    seed: 43,
                },
            ])
            .unwrap_err();
        let BatchError::Runtime(failure) = error else {
            panic!("expected restore error")
        };
        assert_eq!(failure.completed_requests, 0);
        assert_eq!(failure.request, 1);
        assert_eq!(batch.sessions()[0].robot_truth(0), before);
        assert_eq!(batch.sessions()[1].stamp().tick, 4);
        assert_eq!(batch.commands_mut(0).unwrap().len(), 1);
    }

    #[test]
    fn match_checkpoint_buffers_and_nonzero_initial_clock_are_preserved() {
        let mut original = session(crate::StepDuration::HZ_1000);
        original.step(7).unwrap();
        let mut batch =
            MatchBatch::new(vec![MatchSession::new(original.into_field())], limits()).unwrap();
        let mut checkpoints = Vec::with_capacity(1);
        batch.checkpoint_into(&[0], &mut checkpoints).unwrap();
        let storage = checkpoints.as_ptr();
        batch
            .step(
                &[BatchStep {
                    session: 0,
                    steps: 2,
                }],
                &mut Vec::new(),
            )
            .unwrap();
        batch
            .session_mut(0)
            .unwrap()
            .restore(&checkpoints[0])
            .unwrap();
        assert_eq!(batch.sessions()[0].stamp().tick, 7);
        batch.checkpoint_into(&[0], &mut checkpoints).unwrap();
        assert_eq!(checkpoints.as_ptr(), storage);
        assert!(batch.checkpoint_into(&[99], &mut checkpoints).is_err());
        assert_eq!(checkpoints[0].snapshot().tick, 7);
        batch
            .reset(&[MatchReset {
                session: 0,
                seed: 5,
            }])
            .unwrap();
        assert_eq!(batch.sessions()[0].stamp().time_ns, 7_000_000);
    }
}
