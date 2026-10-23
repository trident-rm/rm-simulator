// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Bounded CPU rollouts over independent physical worlds.
//!
//! Slots keep their own immutable step duration and clock. Grouping independent
//! sessions changes neither their commands nor their integration order. Task,
//! reward, seeds, sensors and sampling schedules remain caller-owned. This facade
//! uses sequential execution by default; [`parallel`] adds opt-in scoped workers.

/// Opt-in bounded CPU execution and complete per-request progress.
pub mod parallel;

use super::{RobotCommand, RobotSession, Stamp};
use crate::{StepDuration, chassis::ChassisControlState, projectile::Contact};

/// Caller-selected finite work limits, enforced before any batch operation mutates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BatchLimits {
    /// Maximum resident sessions and requests in one operation; must be positive.
    pub max_sessions: usize,
    /// Maximum integration steps for each selected session in one call.
    pub max_steps_per_session: u64,
    /// Maximum queued commands for each selected session in one call.
    pub max_commands_per_session: usize,
}

/// One slot's advance in its own integration steps, never wall-clock time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BatchStep {
    /// Stable zero-based session slot; duplicates in one step call are rejected.
    pub session: usize,
    /// Number of that session's immutable integration durations to advance.
    pub steps: u64,
}

/// One privileged controller observation request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RobotQuery {
    /// Zero-based session slot.
    pub session: usize,
    /// Physical robot identity within that session.
    pub robot: u32,
}

/// Allocation-free privileged truth; this is neither a sensor nor a checkpoint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlObservation {
    /// Session slot queried.
    pub session: usize,
    /// Robot identity queried.
    pub robot: u32,
    /// This session's observation timestamp, independent of other slot clocks.
    pub stamp: Stamp,
    /// Immutable integration duration; mixed rates are explicitly supported.
    pub step_duration: StepDuration,
    /// Compact physical state, or `None` for an unknown robot identity.
    pub state: Option<ChassisControlState>,
}

/// A runtime failure with explicit retained progress; no implicit rollback occurs.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BatchFailure<E> {
    /// Request position at which execution stopped.
    pub request: usize,
    /// Session slot which failed.
    pub session: usize,
    /// Fully completed requests, also the length of successful step output.
    /// Transactional reset failures always report zero.
    pub completed_requests: usize,
    /// Commands accepted in the failing request before the error.
    pub accepted_commands: usize,
    /// Failing session timestamp before executing this request.
    pub before: Stamp,
    /// Failing session timestamp after the error; inspect its truth for state changes.
    pub after: Stamp,
    /// Original ordinary-session error.
    pub source: E,
}

/// Batch admission failure or explicitly reported runtime partial progress.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BatchError<E> {
    /// Validation rejected the operation before changing sessions, queues or output.
    Invalid {
        /// Offending request position, or zero for construction/overall limits.
        request: usize,
        /// Rejected invariant.
        reason: &'static str,
    },
    /// Stepping retains its completed prefix; reset preparation changes no live slot.
    Runtime(BatchFailure<E>),
}
impl<E: std::fmt::Display> std::fmt::Display for BatchError<E> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Invalid { request, reason } => write!(f, "batch request {request}: {reason}"),
            Self::Runtime(failure) => write!(
                f,
                "batch request {} (session {}), {} complete, {} commands accepted, time {} -> {} ns: {}",
                failure.request,
                failure.session,
                failure.completed_requests,
                failure.accepted_commands,
                failure.before.time_ns,
                failure.after.time_ns,
                failure.source
            ),
        }
    }
}
impl<E: std::fmt::Debug + std::fmt::Display> std::error::Error for BatchError<E> {}

/// Validate bounded, unique session indices without mutating live sessions.
/// Exposed for the complete-match facade; repeated observation queries need not
/// use this helper because observations are read-only.
pub fn validate_selection<E>(
    indices: impl IntoIterator<Item = usize>,
    session_count: usize,
    limits: BatchLimits,
) -> Result<(), BatchError<E>> {
    // Bounded temporary storage is used for resets/checkpoint selection only,
    // outside the stepping hot path.
    let mut seen = Vec::new();
    for (request, session) in indices.into_iter().enumerate() {
        if request >= limits.max_sessions {
            return Err(BatchError::Invalid {
                request,
                reason: "too many requests",
            });
        }
        if session >= session_count {
            return Err(BatchError::Invalid {
                request,
                reason: "session index out of bounds",
            });
        }
        if seen.contains(&session) {
            return Err(BatchError::Invalid {
                request,
                reason: "duplicate session index",
            });
        }
        seen.push(session);
    }
    Ok(())
}

/// Bounded single-owner collection of independent [`RobotSession`] instances.
///
/// Command queues retain capacity. `step` clears/reuses output only after
/// successful preflight. A runtime command error discards the rest of that slot's
/// queue, preserves accepted commands and completed steps, and leaves later
/// slots/queues untouched. Contacts from attempted slots remain available through
/// [`Self::contacts`]. Recipes should capture shared immutable geometry, never
/// mutable cross-session state. Mixed integration durations are supported.
///
/// ```
/// use rm_sim_physics::{StepDuration, session::{RobotScenario, RobotSession,
///     batch::{RobotBatch, BatchLimits, BatchStep}}};
/// let session = RobotSession::new(RobotScenario::new(vec![], 0.,
///     StepDuration::HZ_1000, |_| Ok(()))?)?;
/// let mut batch = RobotBatch::new(vec![session], BatchLimits {
///     max_sessions: 8, max_steps_per_session: 100, max_commands_per_session: 4,
/// })?;
/// let mut stamps = Vec::new();
/// batch.step(&[BatchStep { session: 0, steps: 10 }], &mut stamps)?;
/// assert_eq!(stamps[0].time_ns, 10_000_000);
/// batch.reset(&[0])?;
/// assert_eq!(batch.sessions()[0].stamp().tick, 0);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub struct RobotBatch {
    sessions: Vec<RobotSession>,
    commands: Vec<Vec<RobotCommand>>,
    contacts: Vec<Vec<Contact>>,
    limits: BatchLimits,
}
impl RobotBatch {
    /// Admit owned sessions without copying worlds. No sessions can be added later.
    pub fn new(
        sessions: Vec<RobotSession>,
        limits: BatchLimits,
    ) -> Result<Self, BatchError<&'static str>> {
        if limits.max_sessions == 0 || sessions.len() > limits.max_sessions {
            return Err(BatchError::Invalid {
                request: 0,
                reason: "invalid session capacity",
            });
        }
        Ok(Self {
            commands: (0..sessions.len()).map(|_| Vec::new()).collect(),
            contacts: (0..sessions.len()).map(|_| Vec::new()).collect(),
            sessions,
            limits,
        })
    }

    /// Inspect independent session truth; slot order remains stable for this batch.
    pub fn sessions(&self) -> &[RobotSession] {
        &self.sessions
    }

    /// Access a slot for ordinary commands and caller-owned sensor acquisition.
    /// Changes do not affect other slots or queued batch commands.
    pub fn session_mut(&mut self, session: usize) -> Option<&mut RobotSession> {
        self.sessions.get_mut(session)
    }

    /// Reuse a slot's pending command buffer. Limits are checked by [`Self::step`].
    /// Queued commands are applied once before that slot advances, including zero steps.
    pub fn commands_mut(&mut self, session: usize) -> Option<&mut Vec<RobotCommand>> {
        self.commands.get_mut(session)
    }

    /// Ordered raw contacts from this slot's latest attempted batch step.
    /// Untouched slots retain their previous contacts until stepped or reset.
    pub fn contacts(&self, session: usize) -> Option<&[Contact]> {
        self.contacts.get(session).map(Vec::as_slice)
    }

    /// Apply queued commands and advance selected slots in request order.
    /// All indices, duplicates, work bounds and clock overflows are checked first.
    /// Success output contains one final stamp per request, in request order.
    pub fn step(
        &mut self,
        requests: &[BatchStep],
        output: &mut Vec<Stamp>,
    ) -> Result<(), BatchError<&'static str>> {
        self.validate_steps(requests)?;
        output.clear();
        for (request, item) in requests.iter().enumerate() {
            let session = &mut self.sessions[item.session];
            let before = session.stamp();
            self.contacts[item.session].clear();
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
            if let Err(source) = session.step_into(item.steps, &mut self.contacts[item.session]) {
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

    fn validate_steps(&self, requests: &[BatchStep]) -> Result<(), BatchError<&'static str>> {
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
                .check_advance(item.steps)
                .map_err(|_| invalid("session clock overflow"))?;
        }
        Ok(())
    }

    /// Fill caller-owned observation storage in query order, retaining its capacity.
    /// Invalid indices/counts leave output unchanged; unknown robots produce `None`.
    pub fn observe_into(
        &self,
        queries: &[RobotQuery],
        output: &mut Vec<ControlObservation>,
    ) -> Result<(), BatchError<&'static str>> {
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
                step_duration: session.step_duration(),
                state: session.robot_control_state(query.robot),
            }
        }));
        Ok(())
    }

    /// Transactionally rebuild the selected recipes; failure changes no live slot,
    /// queue, contact buffer or clock. Success clears only selected pending queues
    /// and contacts, and restarts their clocks/IDs. Caller sensor/RNG state is separate.
    /// All replacements are constructed before swapping; peak memory includes them.
    pub fn reset(&mut self, indices: &[usize]) -> Result<(), BatchError<&'static str>> {
        validate_selection(indices.iter().copied(), self.sessions.len(), self.limits)?;
        let mut prepared = Vec::with_capacity(indices.len());
        for (request, &index) in indices.iter().enumerate() {
            let session = &self.sessions[index];
            let physics = session.scenario.build().map_err(|source| {
                BatchError::Runtime(BatchFailure {
                    request,
                    session: index,
                    completed_requests: 0,
                    accepted_commands: 0,
                    before: session.stamp(),
                    after: session.stamp(),
                    source,
                })
            })?;
            prepared.push(physics);
        }
        for (&index, physics) in indices.iter().zip(prepared) {
            let session = &mut self.sessions[index];
            session.physics = physics;
            session.faces.clone_from(&session.scenario.faces);
            session.frames = crate::TargetFrames::new(session.faces.clone());
            session.stamp = Stamp {
                tick: 0,
                time_ns: 0,
            };
            self.commands[index].clear();
            self.contacts[index].clear();
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn session(duration: crate::StepDuration) -> RobotSession {
        RobotSession::new(
            super::super::RobotScenario::new(vec![], 0., duration, |world| {
                world.add_chassis(
                    crate::Team::Red,
                    crate::chassis::ChassisConfig::default(),
                    crate::Pose::at([0., 0., 0.4]),
                )?;
                Ok(())
            })
            .unwrap(),
        )
        .unwrap()
    }
    fn limits() -> BatchLimits {
        BatchLimits {
            max_sessions: 4,
            max_steps_per_session: u64::MAX,
            max_commands_per_session: 4,
        }
    }
    fn batch() -> RobotBatch {
        RobotBatch::new(
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
        batch.reset(&[0]).unwrap();
        assert_eq!(batch.sessions()[0].stamp().tick, 0);
        assert!(batch.commands_mut(0).unwrap().is_empty());
        assert_eq!(batch.sessions()[1].robot_truth(0), untouched);
        assert_eq!(batch.commands_mut(1).unwrap().len(), 1);
        assert!(batch.reset(&[1, 1]).is_err());
        assert_eq!(batch.sessions()[1].robot_truth(0), untouched);
    }
    #[test]
    fn failed_reset_preparation_leaves_every_selected_world_and_queue_unchanged() {
        use std::sync::{
            Arc,
            atomic::{AtomicBool, Ordering},
        };
        let reject = Arc::new(AtomicBool::new(false));
        let flag = reject.clone();
        let failing = RobotSession::new(
            super::super::RobotScenario::new(
                vec![],
                0.,
                crate::StepDuration::default(),
                move |_| {
                    if flag.load(Ordering::Relaxed) {
                        Err("setup rejected")
                    } else {
                        Ok(())
                    }
                },
            )
            .unwrap(),
        )
        .unwrap();
        let mut batch = RobotBatch::new(
            vec![session(crate::StepDuration::default()), failing],
            limits(),
        )
        .unwrap();
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
        batch.commands_mut(0).unwrap().push(drive(0, 0.5));
        let before = batch.sessions()[0].robot_truth(0);
        reject.store(true, Ordering::Relaxed);
        let error = batch.reset(&[0, 1]).unwrap_err();
        let BatchError::Runtime(failure) = error else {
            panic!("expected setup error")
        };
        assert_eq!(failure.completed_requests, 0);
        assert_eq!(failure.request, 1);
        assert_eq!(batch.sessions()[0].robot_truth(0), before);
        assert_eq!(batch.sessions()[1].stamp().tick, 4);
        assert_eq!(batch.commands_mut(0).unwrap().len(), 1);
    }

    #[test]
    fn recipe_geometry_is_shared_between_worlds_and_after_reset() {
        let mut source = crate::WorldPhysics::new(&[], 0.);
        source
            .add_static_mesh(
                vec![[10., 0., 0.], [11., 0., 0.], [10., 1., 0.]],
                vec![[0, 1, 2]],
            )
            .unwrap();
        let geometry = source.static_geometry_snapshot(false);
        let make = || {
            let geometry = geometry.clone();
            RobotSession::new(
                super::super::RobotScenario::new(
                    vec![],
                    0.,
                    crate::StepDuration::default(),
                    move |world| {
                        world.insert_geometry(&geometry);
                        Ok(())
                    },
                )
                .unwrap(),
            )
            .unwrap()
        };
        let mut batch = RobotBatch::new(vec![make(), make()], limits()).unwrap();
        batch.reset(&[1]).unwrap();
        for session in batch.sessions() {
            assert_eq!(
                session
                    .ground_truth()
                    .static_geometry_snapshot(false)
                    .into_triangles(),
                geometry.clone().into_triangles()
            );
        }
    }
}
