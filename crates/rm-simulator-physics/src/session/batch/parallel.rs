// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Synchronous CPU workers borrowing independent persistent sessions.
//!
//! Every admitted request finishes its attempt before return. Runtime errors are
//! reported per request, without cancelling independent requests or rolling back.
//! Scoped threads propagate panics; no panic is converted to a normal error.

use super::{BatchError, BatchStep, RobotBatch};
use crate::session::Stamp;

/// Explicit finite CPU worker bound, independent of the host's logical CPU count.
///
/// One worker executes inline. Larger counts use at most `min(count, requests)`
/// scoped threads per call; sessions and solver state persist between calls.
/// There is no background queue, worker lifetime or asynchronous admission.
///
/// ```
/// use rm_simulator_physics::session::batch::parallel::CpuWorkers;
/// assert!(CpuWorkers::new(0).is_err());
/// assert_eq!(CpuWorkers::new(4)?.count(), 4);
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CpuWorkers(usize);
impl CpuWorkers {
    /// Admit 1 through 256 workers; larger counts are rejected before any work.
    /// The ceiling bounds thread creation even when a batch admits many sessions.
    pub fn new(count: usize) -> Result<Self, &'static str> {
        if !(1..=256).contains(&count) {
            return Err("CPU worker count must be between 1 and 256");
        }
        Ok(Self(count))
    }

    /// Maximum CPU workers used by a call; one runs on the calling thread.
    pub fn count(self) -> usize {
        self.0
    }

    /// Shared orchestration for trusted physics/world adapters after admission.
    /// Jobs are partitioned into exclusive chunks. Every chunk is joined before
    /// return or panic propagation; outputs retain job order. No queues are used.
    ///
    /// ```
    /// use rm_simulator_physics::session::batch::parallel::CpuWorkers;
    /// let mut jobs = [1, 2, 3];
    /// let results = CpuWorkers::new(2)?.run(&mut jobs, |value| {
    ///     *value *= 2;
    ///     *value
    /// });
    /// assert_eq!(jobs, [2, 4, 6]);
    /// assert_eq!(results, [2, 4, 6]);
    /// # Ok::<(), &'static str>(())
    /// ```
    #[doc(hidden)]
    pub fn run<T: Send, R: Send>(self, jobs: &mut [T], run: impl Fn(&mut T) -> R + Sync) -> Vec<R> {
        if self.0 == 1 || jobs.len() <= 1 {
            return jobs.iter_mut().map(run).collect();
        }
        let job_count = jobs.len();
        let chunk_size = job_count.div_ceil(self.0);
        std::thread::scope(|scope| {
            let handles: Vec<_> = jobs
                .chunks_mut(chunk_size)
                .map(|chunk| {
                    let run = &run;
                    scope.spawn(move || chunk.iter_mut().map(run).collect::<Vec<_>>())
                })
                .collect();
            let mut output = Vec::with_capacity(job_count);
            // Collect all joins before propagating any panic. Unwinding the
            // scope also joins threads if allocation or another operation panics.
            let mut panic = None;
            for handle in handles {
                match handle.join() {
                    Ok(results) => output.extend(results),
                    Err(payload) => {
                        if panic.is_none() {
                            panic = Some(payload);
                        }
                    }
                }
            }
            if let Some(payload) = panic {
                std::panic::resume_unwind(payload);
            }
            output
        })
    }
}

/// Exact progress of one admitted request, including ordinary runtime errors.
#[derive(Debug)]
pub struct ParallelOutcome<E> {
    /// Position in the original request slice, independent of worker order.
    pub request: usize,
    /// Stable session slot.
    pub session: usize,
    /// Timestamp before commands or integration.
    pub before: Stamp,
    /// Timestamp after the attempt, including partial integration on failure.
    pub after: Stamp,
    /// Commands accepted before integration or the first rejected command.
    pub accepted_commands: usize,
    /// Ordinary command/integration result; accepted changes are never rolled back.
    pub result: Result<(), E>,
}

/// Joined operation results in original request order, with no work left running.
///
/// Admission failures use `BatchError::Invalid` and return no report. An admitted
/// operation returns this report even when individual requests fail. Callers must
/// inspect [`Self::all_succeeded`] or each outcome before using results.
#[derive(Debug)]
#[must_use = "inspect all_succeeded() or the per-request runtime results"]
pub struct ParallelReport<E> {
    /// One outcome per selected slot, in request order, including every failure.
    pub outcomes: Vec<ParallelOutcome<E>>,
}
impl<E> ParallelReport<E> {
    /// Whether every admitted request accepted all commands and completed its steps.
    pub fn all_succeeded(&self) -> bool {
        self.outcomes.iter().all(|outcome| outcome.result.is_ok())
    }
}

impl RobotBatch {
    /// Apply queued commands once, then advance each selected independent world.
    ///
    /// Uses the same admission checks as [`Self::step`]. Rejected admission leaves
    /// every session, command queue and contact buffer unchanged. After admission,
    /// all selected slots attempt their work, including after another slot fails.
    /// A failed command discards that slot's remaining queued commands. Untouched
    /// slots retain queues/contacts. Outputs report all progress in request order.
    /// Calls require exclusive batch access, so there is no pending-work queue.
    /// Reset remains the existing synchronous selected-set transaction.
    ///
    /// ```
    /// use rm_simulator_physics::{StepDuration, session::{RobotScenario, RobotSession,
    ///     batch::{RobotBatch, BatchLimits, BatchStep, parallel::CpuWorkers}}};
    /// let session = RobotSession::new(RobotScenario::new(vec![], 0.,
    ///     StepDuration::HZ_1000, |_| Ok(()))?)?;
    /// let mut batch = RobotBatch::new(vec![session], BatchLimits {
    ///     max_sessions: 8, max_steps_per_session: 100, max_commands_per_session: 4,
    /// })?;
    /// let report = batch.step_parallel(CpuWorkers::new(2)?,
    ///     &[BatchStep { session: 0, steps: 10 }])?;
    /// assert!(report.all_succeeded());
    /// assert_eq!(report.outcomes[0].after.time_ns, 10_000_000);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn step_parallel(
        &mut self,
        workers: CpuWorkers,
        requests: &[BatchStep],
    ) -> Result<ParallelReport<&'static str>, BatchError<&'static str>> {
        self.validate_steps(requests)?;
        let mut jobs: Vec<_> = self
            .sessions
            .iter_mut()
            .zip(&mut self.commands)
            .zip(&mut self.contacts)
            .enumerate()
            .filter_map(|(index, ((session, commands), contacts))| {
                requests
                    .iter()
                    .enumerate()
                    .find(|(_, item)| item.session == index)
                    .map(|(request, item)| (request, *item, session, commands, contacts))
            })
            .collect();
        let mut outcomes =
            workers.run(&mut jobs, |(request, item, session, commands, contacts)| {
                let before = session.stamp();
                contacts.clear();
                let mut accepted_commands = 0;
                let result = commands
                    .drain(..)
                    .try_for_each(|command| {
                        session.command(command)?;
                        accepted_commands += 1;
                        Ok(())
                    })
                    .and_then(|()| session.step_into(item.steps, contacts));
                ParallelOutcome {
                    request: *request,
                    session: item.session,
                    before,
                    after: session.stamp(),
                    accepted_commands,
                    result,
                }
            });
        outcomes.sort_unstable_by_key(|outcome| outcome.request);
        Ok(ParallelReport { outcomes })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::session::RobotCommand;
    use crate::session::batch::BatchLimits;

    fn batch() -> RobotBatch {
        let sessions = (0..8)
            .map(|index| {
                let duration = if index % 2 == 0 {
                    crate::StepDuration::HZ_1000
                } else {
                    crate::StepDuration::default()
                };
                crate::session::RobotSession::new(
                    crate::session::RobotScenario::new(vec![], 0., duration, |world| {
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
            })
            .collect();
        RobotBatch::new(
            sessions,
            BatchLimits {
                max_sessions: 8,
                max_steps_per_session: 100,
                max_commands_per_session: 4,
            },
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
    fn worker_counts_match_sequential_truth_and_request_order() {
        for count in [1, 2, 4, 8] {
            let mut sequential = batch();
            let mut parallel = batch();
            let requests: Vec<_> = (0..8)
                .rev()
                .map(|session| BatchStep {
                    session,
                    steps: 12 + session as u64,
                })
                .collect();
            for steps in [0, 1, 2] {
                for slot in 0..8 {
                    for target in [&mut sequential, &mut parallel] {
                        target
                            .commands_mut(slot)
                            .unwrap()
                            .extend([drive(0, 0.1), drive(0, 0.4)]);
                    }
                }
                let requests: Vec<_> = requests
                    .iter()
                    .map(|item| BatchStep {
                        session: item.session,
                        steps: item.steps * steps,
                    })
                    .collect();
                let mut stamps = Vec::new();
                sequential.step(&requests, &mut stamps).unwrap();
                let report = parallel
                    .step_parallel(CpuWorkers::new(count).unwrap(), &requests)
                    .unwrap();
                assert!(report.all_succeeded());
                for (index, outcome) in report.outcomes.iter().enumerate() {
                    assert_eq!(outcome.request, index);
                    assert_eq!(outcome.session, requests[index].session);
                    assert_eq!(outcome.after, stamps[index]);
                    assert_eq!(outcome.accepted_commands, 2);
                }
                for (left, right) in sequential.sessions().iter().zip(parallel.sessions()) {
                    assert_eq!(left.robot_truth(0), right.robot_truth(0));
                }
            }
        }
    }
    #[test]
    fn all_attempts_join_and_errors_report_every_slots_progress() {
        let mut batch = batch();
        batch
            .commands_mut(0)
            .unwrap()
            .extend([drive(0, 0.3), drive(999, 0.4), drive(0, 0.8)]);
        batch.commands_mut(2).unwrap().push(drive(999, 0.1));
        batch.commands_mut(7).unwrap().push(drive(0, 0.2));
        let requests = [
            BatchStep {
                session: 2,
                steps: 10,
            },
            BatchStep {
                session: 1,
                steps: 11,
            },
            BatchStep {
                session: 0,
                steps: 12,
            },
        ];
        let report = batch
            .step_parallel(CpuWorkers::new(4).unwrap(), &requests)
            .unwrap();
        assert!(!report.all_succeeded());
        assert_eq!(report.outcomes.len(), 3);
        assert!(report.outcomes[0].result.is_err());
        assert_eq!(report.outcomes[0].accepted_commands, 0);
        assert_eq!(report.outcomes[1].after.tick, 11);
        assert!(report.outcomes[1].result.is_ok());
        assert_eq!(report.outcomes[2].accepted_commands, 1);
        assert_eq!(report.outcomes[2].before, report.outcomes[2].after);
        assert!(report.outcomes[2].result.is_err());
        assert!(batch.commands_mut(0).unwrap().is_empty());
        assert!(batch.commands_mut(2).unwrap().is_empty());
        assert_eq!(batch.commands_mut(7).unwrap().len(), 1);
        // Ordinary synchronous reset still supports a whole selected-set transaction.
        batch.reset(&[0, 1]).unwrap();
        assert_eq!(batch.sessions()[0].stamp().tick, 0);
        assert_eq!(batch.sessions()[1].stamp().tick, 0);
    }
    #[test]
    fn admission_errors_preserve_all_slots_queues_and_contacts() {
        let mut batch = batch();
        batch.commands_mut(0).unwrap().push(drive(0, 0.4));
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
            vec![BatchStep {
                session: 0,
                steps: 101,
            }],
        ] {
            assert!(matches!(
                batch.step_parallel(CpuWorkers::new(2).unwrap(), &requests),
                Err(BatchError::Invalid { .. })
            ));
            assert!(batch.sessions().iter().all(|slot| slot.stamp().tick == 0));
            assert_eq!(batch.commands_mut(0).unwrap().len(), 1);
        }
        let report = batch
            .step_parallel(CpuWorkers::new(8).unwrap(), &[])
            .unwrap();
        assert!(report.all_succeeded());
        assert!(report.outcomes.is_empty());
        assert!(CpuWorkers::new(0).is_err());
        assert!(CpuWorkers::new(257).is_err());
    }
}
