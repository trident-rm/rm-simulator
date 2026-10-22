// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Bounded scoped CPU workers over independent ordinary match sessions.

use super::{BatchError, BatchStep, MatchBatch};
use crate::FieldError;
/// Shared worker admission and per-request joined progress.
pub use rm_simulator_physics::session::batch::parallel::{
    CpuWorkers, ParallelOutcome, ParallelReport,
};

impl MatchBatch {
    /// Apply commands once and advance selected matches on bounded CPU workers.
    ///
    /// All indices, duplicates, command/step limits and clock overflow are checked
    /// before mutation. Admission errors leave every slot and queue untouched.
    /// Every admitted slot finishes its attempt before return, even when another
    /// slot fails; inspect the returned report for all runtime errors and progress.
    /// Failed commands discard the rest of that slot's queue. Native match checks
    /// and per-slot command/integration order remain ordinary session behavior.
    /// No background work or queue survives the call. Panics propagate after join.
    /// Selective reset remains the existing synchronous transaction.
    ///
    /// ```
    /// use rm_simulator_world::{Field, FieldConfig, session::{MatchSession,
    ///     batch::{MatchBatch, BatchLimits, BatchStep, parallel::CpuWorkers}}};
    /// let mut batch = MatchBatch::new(vec![MatchSession::new(
    ///     Field::new(&FieldConfig::default())?)], BatchLimits {
    ///     max_sessions: 8, max_steps_per_session: 128, max_commands_per_session: 4,
    /// })?;
    /// let report = batch.step_parallel(CpuWorkers::new(2)?,
    ///     &[BatchStep { session: 0, steps: 128 }])?;
    /// assert!(report.all_succeeded());
    /// assert_eq!(report.outcomes[0].after.time_ns, 1_000_000_000);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn step_parallel(
        &mut self,
        workers: CpuWorkers,
        requests: &[BatchStep],
    ) -> Result<ParallelReport<FieldError>, BatchError<FieldError>> {
        self.validate_steps(requests)?;
        let mut jobs: Vec<_> = self
            .sessions
            .iter_mut()
            .zip(&mut self.commands)
            .enumerate()
            .filter_map(|(index, (session, commands))| {
                requests
                    .iter()
                    .enumerate()
                    .find(|(_, item)| item.session == index)
                    .map(|(request, item)| (request, *item, session, commands))
            })
            .collect();
        let mut outcomes = workers.run(&mut jobs, |(request, item, session, commands)| {
            let before = session.stamp();
            let mut accepted_commands = 0;
            let result = commands
                .drain(..)
                .try_for_each(|command| {
                    session.command(command)?;
                    accepted_commands += 1;
                    Ok(())
                })
                .and_then(|()| session.step(item.steps).map(|_| ()));
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

    fn batch() -> MatchBatch {
        let sessions = (0..8)
            .map(|index| {
                let duration = if index % 2 == 0 {
                    crate::StepDuration::HZ_1000
                } else {
                    crate::StepDuration::default()
                };
                crate::session::MatchSession::new(
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
            })
            .collect();
        MatchBatch::new(
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
                    assert_eq!(
                        left.ground_truth().rmul_2027(),
                        right.ground_truth().rmul_2027()
                    );
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
        batch
            .reset(&[
                super::super::MatchReset {
                    session: 0,
                    seed: 1,
                },
                super::super::MatchReset {
                    session: 1,
                    seed: 2,
                },
            ])
            .unwrap();
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
