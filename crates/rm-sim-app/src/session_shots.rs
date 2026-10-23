// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Provisional shots with bounded retries and independent authoritative outcomes.
use super::*;
use crate::projectile_prediction::{Flight, Request, Worker};
use rm_sim_server::protocol::ShotResult;
use std::{
    collections::BTreeMap,
    sync::Arc,
    time::{Duration, Instant},
};
struct LocalFlight {
    flight: Flight,
    spread: rm_sim_server::protocol::BulletSpread,
    command: Command,
    created: Instant,
    reserved: bool,
    scheduled: bool,
    muzzle_resolved: bool,
    sent: Instant,
    projectile: Option<rm_sim_world::ProjectileSnapshot>,
}
/// Provisional shots this client fired and how the host answered them. A
/// provisional ball is drawn under a client-owned id, never the host's.
#[derive(Default)]
pub(super) struct LocalShots {
    /// Wall time from sending a shot to the host accepting it, in ms.
    pub last_confirmation_ms: Option<f64>,
    /// Individual confirmation delays, retained independently of console polling.
    pub confirmation_samples: rm_sim_server::network_trace::EventSamples,
    /// Individual available host execution offsets, in ms.
    pub execution_samples: rm_sim_server::network_trace::EventSamples,
    /// Own launches confirmed exactly once, including snapshot recovery.
    pub confirmed_launches: u64,
    /// Host execution time minus the client's predicted launch time, in ms.
    pub last_execution_offset_ms: Option<f64>,
    /// Worker that flies unconfirmed shots through a restored field, or `None`
    /// when prediction is off.
    pub worker: Option<Worker>,
    flights: BTreeMap<u64, LocalFlight>,
    /// Lazily copied once per replay checkpoint, shared by frame requests.
    checkpoint: Option<(u64, u64, Arc<FieldSnapshot>)>,
    next_launch_ns: u64,
    last_launch: Option<(u64, u64, f64)>,
    /// Shots that expired after 5 s with no authoritative outcome.
    pub unresolved_outcomes: u64,
}
impl Session {
    /// Actual initial speed of this pilot's latest confirmed launch in m/s.
    /// Rejected shots and changes to settings do not change this reading.
    pub fn last_bullet_speed_m_s(&self) -> Option<f64> {
        self.shots.last_launch.map(|(_, _, speed)| speed)
    }
    /// Send a `FireAimed` for the own chassis from the newest input frame and
    /// record a provisional flight. A shot inside the weapon cadence is
    /// skipped; missing input, a full pending set or an exhausted id is an
    /// error the caller shows as a notice.
    pub(super) fn begin_local_shot(&mut self, shooter: u32) -> Result<(), String> {
        if self.weapon_update_pending {
            if !self.commands_confirmed() {
                return Ok(());
            }
            self.weapon_update_pending = false;
        }
        if self.prediction_limited() {
            return Ok(());
        }
        if let Some((chassis, command)) = self.sampled_drive {
            if chassis != shooter {
                return Err("pilot input belongs to another chassis".into());
            }
            if self.last_input_frame.is_none_or(|frame| {
                frame.command != command || frame.sampled_time_ns != self.input_time_ns()
            }) {
                self.apply(Command::Chassis { chassis, command });
            }
        }
        let input = self.last_input_frame.ok_or("waiting for pilot input")?;
        let own = self.presented_chassis().ok_or("pilot unavailable")?;
        if self.pending_shots() >= 32 || self.shots.flights.len() >= 128 {
            return Err("too many pending shots".into());
        }
        if input.sampled_time_ns < self.shots.next_launch_ns {
            return Ok(());
        }
        let muzzle = crate::projectile_prediction::muzzle(own, input.command);
        let muzzle = self
            .weapon
            .spread
            .apply(muzzle, shooter, input.sampled_time_ns);
        let shot_id = self.next_shot_id;
        self.next_shot_id = self
            .next_shot_id
            .checked_add(1)
            .ok_or("shot id exhausted")?;
        let command = Command::FireAimed {
            shooter,
            shot_id,
            input,
        };
        self.client.send(command).map_err(|e| e.to_string())?;
        self.shots.next_launch_ns = input
            .sampled_time_ns
            .saturating_add(self.weapon.interval_ns);
        let now = self.time.now();
        self.shots.flights.insert(
            shot_id,
            LocalFlight {
                spread: self.weapon.spread,
                flight: Flight {
                    id: shot_id,
                    muzzle,
                    launched_ns: input.sampled_time_ns,
                    shot: self.weapon.sample_shot(
                        shooter,
                        input.sampled_time_ns,
                        self.weapon_limits.max_speed_m_s,
                    ),
                    authoritative: None,
                },
                command,
                created: now,
                reserved: true,
                scheduled: false,
                muzzle_resolved: false,
                sent: now,
                projectile: None,
            },
        );
        Ok(())
    }
    /// Unconfirmed shots whose muzzle the prediction worker has not captured
    /// yet, one sample per shot.
    pub(super) fn pending_shot_samples(&self) -> Vec<crate::prediction::ShotSample> {
        self.shots
            .flights
            .iter()
            .filter_map(|(id, flight)| {
                if flight.muzzle_resolved || flight.flight.authoritative.is_some() {
                    return None;
                }
                let Command::FireAimed { input, .. } = flight.command else {
                    return None;
                };
                Some(crate::prediction::ShotSample {
                    id: *id,
                    time_ns: input.sampled_time_ns,
                    aim: input.command,
                })
            })
            .collect()
    }
    /// Apply muzzle poses the prediction worker captured, dropping captures
    /// from an older prediction epoch.
    pub(super) fn resolve_shot_muzzles(&mut self, captures: Vec<(u64, u64, rm_sim_world::Pose)>) {
        for (epoch, id, muzzle) in captures {
            if epoch == self.prediction.epoch
                && let Some(flight) = self.shots.flights.get_mut(&id)
            {
                if let Command::FireAimed { shooter, input, .. } = flight.command {
                    flight.flight.muzzle =
                        flight.spread.apply(muzzle, shooter, input.sampled_time_ns);
                }
                flight.muzzle_resolved = true;
            }
        }
    }
    /// Forget every provisional flight and reopen the cadence gate. Called on
    /// pause, disconnect and any life change.
    pub(super) fn cancel_local_shots(&mut self) {
        self.shots.flights.clear();
        self.shots.next_launch_ns = 0;
    }
    fn shot_result(&mut self, result: ShotResult) {
        if Some(result.shooter) != self.chassis_id {
            return;
        }
        if let (Ok(projectile), Some(executed), Some(speed)) = (
            &result.result,
            result.executed_time_ns,
            result.launch_speed_m_s,
        ) && self
            .shots
            .last_launch
            .is_none_or(|(time, id, _)| (executed, *projectile) > (time, id))
        {
            self.shots.last_launch = Some((executed, *projectile, speed));
        }
        match result.result {
            Ok(id) => {
                if let Some(flight) = self.shots.flights.get_mut(&result.shot_id) {
                    if flight.flight.authoritative.is_none() {
                        self.shots.confirmed_launches += 1;
                        self.shots.last_confirmation_ms =
                            Some(self.time.since(flight.created).as_secs_f64() * 1000.);
                        self.shots.last_execution_offset_ms = result
                            .executed_time_ns
                            .map(|t| (t as f64 - flight.flight.launched_ns as f64) / 1e6);
                        self.shots
                            .confirmation_samples
                            .record(self.shots.last_confirmation_ms.unwrap());
                        if let Some(offset) = self.shots.last_execution_offset_ms {
                            self.shots.execution_samples.record(offset);
                        }
                    }
                    flight.flight.authoritative = Some(id);
                    if let Some(executed) = result.executed_time_ns {
                        flight.flight.launched_ns = executed;
                        self.shots.next_launch_ns = self
                            .shots
                            .next_launch_ns
                            .max(executed.saturating_add(self.weapon.interval_ns));
                    }
                }
            }
            Err(reason) => {
                if self.shots.flights.remove(&result.shot_id).is_some() {
                    self.shot_rejection_count += 1;
                    self.last_shot_rejection = Some(format!("shot {}: {reason}", result.shot_id));
                }
            }
        }
    }
    /// Fold a checkpoint's shot results into the flights and stop reserving
    /// ammo for accepted shots. A confirmed flight is kept only while its
    /// projectile is still in the snapshot.
    pub(super) fn reconcile_shots(&mut self, state: &rm_sim_server::simulation::SimulationState) {
        for result in &state.shot_results {
            self.shot_result(result.clone());
            if Some(result.shooter) == self.chassis_id
                && result.result.is_ok()
                && let Some(flight) = self.shots.flights.get_mut(&result.shot_id)
            {
                flight.reserved = false;
            }
        }
        self.shots.flights.retain(|id, flight| {
            if let Some(authoritative) = flight.flight.authoritative {
                // A snapshot containing acceptance also tells us whether the ball still exists.
                if state.shot_results.iter().any(|r| {
                    Some(r.shooter) == self.chassis_id
                        && r.shot_id == *id
                        && r.result == Ok(authoritative)
                }) {
                    return state
                        .field
                        .projectiles
                        .iter()
                        .any(|p| p.id == authoritative);
                }
            }
            true
        });
    }
    /// Take scheduled and answered shot messages from the client and update
    /// the flights accordingly.
    pub(super) fn poll_shot_results(&mut self) {
        for (shooter, id) in self.client.take_scheduled_shots() {
            if Some(shooter) == self.chassis_id
                && let Some(flight) = self.shots.flights.get_mut(&id)
            {
                flight.scheduled = true;
            }
        }
        for result in self.client.take_shot_results() {
            self.shot_result(result);
        }
    }
    /// Retire flights older than 5 s, resend unconfirmed shots every 40 ms for
    /// at most 250 ms, then step the provisional projectile worker.
    pub(crate) fn advance_local_shots(&mut self) {
        let connected = self.client.disconnected().is_none();
        let now = self.time.now();
        self.shots.flights.retain(|_, flight| {
            let expired = now.saturating_duration_since(flight.created) >= Duration::from_secs(5);
            if expired && flight.flight.authoritative.is_none() {
                self.shots.unresolved_outcomes += 1;
            }
            !expired
        });
        if connected {
            for flight in self.shots.flights.values_mut() {
                if flight.flight.authoritative.is_none()
                    && !flight.scheduled
                    && now.saturating_duration_since(flight.created) < Duration::from_millis(250)
                    && now.saturating_duration_since(flight.sent) >= Duration::from_millis(40)
                {
                    let _ = self.client.send(flight.command);
                    flight.sent = now;
                }
            }
        }
        if self.shots.flights.is_empty() {
            self.shots.checkpoint = None;
            return;
        }
        if self.paused || self.prediction_limited() {
            return;
        }
        let Some(own) = self.presented_chassis().cloned() else {
            return;
        };
        let Some(worker) = &self.shots.worker else {
            return;
        };
        let checkpoint = self.shots.checkpoint.get_or_insert_with(|| {
            (
                self.prediction.epoch,
                self.snapshot_id,
                Arc::new(self.snapshot.clone()),
            )
        });
        if (checkpoint.0, checkpoint.1) != (self.prediction.epoch, self.snapshot_id) {
            *checkpoint = (
                self.prediction.epoch,
                self.snapshot_id,
                Arc::new(self.snapshot.clone()),
            );
        }
        let snapshot = Arc::clone(&checkpoint.2);
        let peers = snapshot
            .chassis
            .iter()
            .filter(|chassis| Some(chassis.id) != self.chassis_id)
            .map(|chassis| {
                self.remote_history
                    .sample(chassis.id, self.remote_view_time_ns)
                    .unwrap_or_else(|| chassis.clone())
            })
            .collect();
        if let Some(output) = worker.exchange(Request {
            epoch: self.prediction.epoch,
            snapshot_id: self.snapshot_id,
            snapshot,
            peers,
            own,
            time_ns: self.prediction.time_ns.max(self.frame_time_ns),
            flights: self
                .shots
                .flights
                .values()
                .map(|f| f.flight.clone())
                .collect(),
        }) && output.epoch == self.prediction.epoch
        {
            for (id, flight) in &mut self.shots.flights {
                flight.projectile = output.projectiles.get(id).cloned();
            }
        }
    }
    /// Shots of `caliber` begun locally but not yet accepted by the host. The
    /// HUD subtracts them from the host's ammo count.
    pub fn reserved_ammo(&self, caliber: rm_sim_world::Caliber) -> u32 {
        self.shots
            .flights
            .values()
            .filter(|f| f.reserved && f.flight.shot.caliber == caliber)
            .count() as u32
    }
    /// The local robot's barrel heat in tenths, with the heat of shots the
    /// host has not accepted yet added while the round runs (the only phase
    /// that counts heat), and its heat limit in whole units; `None` without
    /// a referee record for the local chassis.
    pub fn predicted_heat(&self) -> Option<(u64, u32)> {
        use rm_sim_world::referee::game_caliber;
        if let Some(game) = &self.snapshot.rmul_2027 {
            let robot = game.robots.iter().find(|r| Some(r.id) == self.chassis_id)?;
            let pending = if game.phase == rm_sim_world::gameplay::rmul_2027::Phase::Running {
                u64::from(self.reserved_ammo(self.weapon.shot.caliber))
                    * game_caliber(self.weapon.shot.caliber).launch_heat_tenths()
            } else {
                0
            };
            return Some((
                u64::from(robot.heat_tenths) + pending,
                robot.stats().heat_limit,
            ));
        }
        if let Some(s) = self.rmuc_2027() {
            let r = s
                .game
                .robots
                .iter()
                .find(|r| Some(r.id) == self.chassis_id)?;
            let pending = if s.game.phase == rm_sim_world::rmuc_2027::match_rules::Phase::Running {
                u64::from(self.reserved_ammo(self.weapon.shot.caliber))
                    * game_caliber(self.weapon.shot.caliber).launch_heat_tenths()
            } else {
                0
            };
            return Some((
                r.combat.heat_tenths + pending,
                r.stats(s.game.elapsed_ns()).heat_limit,
            ));
        }
        None
    }
    /// Whether the pilot has no launcher or the host would refuse a launch for heat (section 5.1.3), so no
    /// local shot should begin: the latest snapshot has the barrel overheated
    /// or locked for the round, or its heat plus the shots still on their way
    /// to the host is already past the limit. The snapshot's heat predates
    /// its cooling since, so under lag this errs toward holding a shot.
    pub fn barrel_blocked(&self) -> bool {
        if let Some(game) = &self.snapshot.rmul_2027 {
            let Some(id) = self.chassis_id else {
                return true;
            };
            return game
                .check_launch(id, self.weapon.shot.caliber == rm_sim_world::Caliber::Mm42)
                .is_err()
                || game
                    .robots
                    .iter()
                    .find(|r| r.id == id)
                    .is_none_or(|r| r.allowance <= self.reserved_ammo(self.weapon.shot.caliber))
                || self
                    .predicted_heat()
                    .is_some_and(|(heat, limit)| heat > u64::from(limit) * 10);
        }
        if self.rmuc_2027().is_none()
            && self
                .client
                .welcome()
                .chassis
                .as_ref()
                .is_some_and(|c| matches!(c.robot, rm_sim_server::protocol::Robot::Engineer))
        {
            return true;
        }
        if let Some(s) = self.rmuc_2027() {
            let Some(r) = s.game.robots.iter().find(|r| Some(r.id) == self.chassis_id) else {
                return true;
            };
            return s.game.check_launch(r.id).is_err()
                || (s.game.phase == rm_sim_world::rmuc_2027::match_rules::Phase::Running
                    && (r.combat.allowance.saturating_add(
                        s.game.field_effects(r.id).fortress_reserve
                            / if self.weapon.shot.caliber == rm_sim_world::Caliber::Mm42 {
                                10
                            } else {
                                1
                            },
                    ) <= self.reserved_ammo(self.weapon.shot.caliber)
                        || self
                            .predicted_heat()
                            .is_some_and(|(heat, limit)| heat > u64::from(limit) * 10)));
        }
        false
    }
    /// Flights with no authoritative outcome yet.
    pub fn pending_shots(&self) -> usize {
        self.shots
            .flights
            .values()
            .filter(|f| f.flight.authoritative.is_none())
            .count()
    }
    /// Projectile states for drawing, merging local flights without copying
    /// the full field checkpoint on every frame that a shot is pending.
    pub fn visual_projectiles(&self) -> impl Iterator<Item = &rm_sim_world::ProjectileSnapshot> {
        let official = self.snapshot.projectiles.iter().map(|projectile| {
            self.shots
                .flights
                .values()
                .find(|flight| flight.flight.authoritative == Some(projectile.id))
                .and_then(|flight| flight.projectile.as_ref())
                .unwrap_or(projectile)
        });
        let provisional = self.shots.flights.values().filter_map(|flight| {
            let already_present = flight.flight.authoritative.is_some_and(|id| {
                self.snapshot
                    .projectiles
                    .iter()
                    .any(|projectile| projectile.id == id)
            });
            (!already_present)
                .then_some(flight.projectile.as_ref())
                .flatten()
        });
        official.chain(provisional)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rmul_heat_and_launcher_admission_use_native_authority() {
        use rm_sim_world::gameplay::{
            Team,
            rmul_2027::{Format, Match, Role},
        };
        let mut session = crate::session::test_session(false);
        let id = session.chassis_id.unwrap();
        let mut game = Match::new(Format::Infantry);
        game.add_robot(id, Team::Red, Role::Infantry).unwrap();
        session.snapshot.rmul_2027 = Some(game);
        assert!(session.barrel_blocked());
        session.snapshot.rmul_2027.as_mut().unwrap().start();
        assert!(!session.barrel_blocked());
        let robot = &mut session.snapshot.rmul_2027.as_mut().unwrap().robots[0];
        robot.heat_tenths = 270;
        assert_eq!(session.predicted_heat(), Some((270, 80)));
        session.snapshot.rmul_2027.as_mut().unwrap().robots[0].allowance = 0;
        assert!(session.barrel_blocked());
    }

    #[test]
    fn projectile_replay_is_idle_without_flights_and_shares_active_checkpoint() {
        let mut session = crate::session::test_session(false);
        let field = rm_sim_world::Field::new(&Default::default()).unwrap();
        session.shots.worker = Some(Worker::new(field.static_geometry_snapshot(), 0.).unwrap());
        session.advance_local_shots();
        assert!(session.shots.checkpoint.is_none());
        let shooter = session.chassis_id.unwrap();
        session.number_input(Command::Chassis {
            chassis: shooter,
            command: Default::default(),
        });
        session.begin_local_shot(shooter).unwrap();
        session.advance_local_shots();
        let first = Arc::clone(&session.shots.checkpoint.as_ref().unwrap().2);
        session.advance_local_shots();
        assert!(Arc::ptr_eq(
            &first,
            &session.shots.checkpoint.as_ref().unwrap().2
        ));
        session.snapshot_id += 1;
        session.advance_local_shots();
        assert!(!Arc::ptr_eq(
            &first,
            &session.shots.checkpoint.as_ref().unwrap().2
        ));
        session.cancel_local_shots();
        session.advance_local_shots();
        assert!(session.shots.checkpoint.is_none());
    }

    #[test]
    fn fortress_reserve_unblocks_zero_personal_ammo_and_pending_shots_reserve_it() {
        use rm_sim_world::gameplay::Team;
        use rm_sim_world::{
            Pose,
            rmuc_2027::{
                Config, Detector, Point, State, Tower,
                match_rules::{COUNTDOWN_NS, GroundRole, Match},
            },
        };
        let mut session = crate::session::test_session(false);
        let shooter = session.chassis_id.unwrap();
        let mut game = Match::new(1);
        game.add_robot(shooter, Team::Red, GroundRole::Infantry);
        game.start(1);
        game.advance_ns(COUNTDOWN_NS);
        game.set_outpost_hp(Team::Red, 0);
        game.advance_ns(5_000_000_000);
        game.observe(shooter, Team::Red, Point::Fortress, true);
        game.advance_ns(0);
        game.fortress_reserve_used[0] = 99;
        assert_eq!(game.robots[0].combat.allowance, 0);
        let detector = Detector {
            pose: Pose::at([3., 0., 1.]),
            half_m: [0.045, 0.044],
        };
        session.snapshot.referee = None;
        session.snapshot.rmuc_2027 = Some(State {
            game,
            config: Config {
                bases: [vec![detector.clone()], vec![detector.clone()]],
                outposts: std::array::from_fn(|_| Tower {
                    pivot_m: [3., 0., 1.],
                    axis: [0., 0., 1.],
                    detectors: vec![detector.clone(); 3],
                }),
                zones: vec![],
                supply_zones: vec![],
                vantage_zones: vec![],
                terrain_zones: vec![],
                runes: vec![],
                rotor_direction: 1,
            },
        });
        assert!(!session.barrel_blocked());
        session.number_input(Command::Chassis {
            chassis: shooter,
            command: Default::default(),
        });
        session.begin_local_shot(shooter).unwrap();
        assert_eq!(session.reserved_ammo(rm_sim_world::Caliber::Mm17), 1);
        assert!(session.barrel_blocked());
    }

    #[test]
    fn weapon_update_waits_for_confirmation_and_old_predictions_keep_their_spread() {
        let mut session = crate::session::test_session(false);
        let shooter = session.chassis_id.unwrap();
        session.number_input(Command::Chassis {
            chassis: shooter,
            command: Default::default(),
        });
        session.begin_local_shot(shooter).unwrap();
        let id = *session.shots.flights.first_key_value().unwrap().0;
        let mut weapon = session.weapon;
        weapon.spread.angle_rad = 0.1;
        session.set_weapon(weapon).unwrap();
        assert!(session.weapon_update_pending);
        assert!(!session.commands_confirmed());
        session.begin_local_shot(shooter).unwrap();
        assert_eq!(session.shots.flights.len(), 1);
        let muzzle = rm_sim_world::Pose::at([1., 2., 3.]);
        session.resolve_shot_muzzles(vec![(session.prediction.epoch, id, muzzle)]);
        let expected = session.shots.flights[&id].spread.apply(
            muzzle,
            shooter,
            session.last_input_frame.unwrap().sampled_time_ns,
        );
        assert_eq!(session.shots.flights[&id].flight.muzzle, expected);
        assert_eq!(session.weapon, weapon);
    }

    #[test]
    fn execution_binds_actual_tick_and_snapshot_releases_ammo_once() {
        let mut session = crate::session::test_session(false);
        let shooter = session.chassis_id.unwrap();
        session.number_input(Command::Chassis {
            chassis: shooter,
            command: Default::default(),
        });
        session.begin_local_shot(shooter).unwrap();
        let (&shot_id, flight) = session.shots.flights.first_key_value().unwrap();
        let executed = flight.flight.launched_ns + 10_000_000;
        let result = ShotResult {
            launch_speed_m_s: Some(24.87),
            executed_time_ns: Some(executed),
            shooter,
            shot_id,
            result: Ok(999),
        };
        session.shot_result(result.clone());
        session.shot_result(result.clone());
        assert_eq!(
            session.shots.flights[&shot_id].flight.authoritative,
            Some(999)
        );
        assert_eq!(session.shots.flights[&shot_id].flight.launched_ns, executed);
        assert_eq!(session.reserved_ammo(session.weapon.shot.caliber), 1);
        assert_eq!(session.shots.last_execution_offset_ms, Some(10.));
        assert_eq!(session.last_bullet_speed_m_s(), Some(24.87));
        let mut older = result.clone();
        older.executed_time_ns = Some(executed - 1);
        older.launch_speed_m_s = Some(20.);
        session.shot_result(older);
        assert_eq!(session.last_bullet_speed_m_s(), Some(24.87));
        let state = rm_sim_server::simulation::SimulationState {
            bots: vec![],
            navigation: Vec::new(),
            sentry_tracking: Vec::new(),
            snapshot_id: session.snapshot_id,
            input_epoch: session.input_epoch,
            shot_results: vec![result],
            paused: false,
            field: session.snapshot.clone(),
        };
        session.reconcile_shots(&state);
        session.reconcile_shots(&state);
        let confirmations = serde_json::to_value(&session.shots.confirmation_samples).unwrap();
        let executions = serde_json::to_value(&session.shots.execution_samples).unwrap();
        assert_eq!(confirmations["total"], 1);
        assert_eq!(executions["values"], serde_json::json!([[1, 10.]]));
        assert_eq!(session.reserved_ammo(session.weapon.shot.caliber), 0);
        assert_eq!(session.pending_shots(), 0);
    }

    #[test]
    fn accepted_bullets_do_not_fill_the_pending_shot_limit() {
        let mut session = crate::session::test_session(false);
        let shooter = session.chassis_id.unwrap();
        session.number_input(Command::Chassis {
            chassis: shooter,
            command: Default::default(),
        });
        let input = session.last_input_frame.unwrap();
        let command = Command::FireAimed {
            shooter,
            shot_id: 1,
            input,
        };
        for id in 1..=32 {
            session.shots.flights.insert(
                id,
                LocalFlight {
                    spread: Default::default(),
                    flight: Flight {
                        id,
                        muzzle: rm_sim_world::Pose::at([0.; 3]),
                        launched_ns: 0,
                        shot: session.weapon.shot,
                        authoritative: Some(id),
                    },
                    command,
                    created: Instant::now(),
                    sent: Instant::now(),
                    reserved: false,
                    scheduled: false,
                    muzzle_resolved: false,
                    projectile: None,
                },
            );
        }
        session.next_shot_id = 33;
        session.begin_local_shot(shooter).unwrap();
        assert_eq!(session.shots.flights.len(), 33);
        assert_eq!(session.pending_shots(), 1);
        for flight in session.shots.flights.values_mut() {
            flight.flight.authoritative = None;
        }
        assert_eq!(
            session.begin_local_shot(shooter).unwrap_err(),
            "too many pending shots"
        );
    }
}
