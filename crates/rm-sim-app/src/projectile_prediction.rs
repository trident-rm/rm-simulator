// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Provisional balls, fired into a field restored from the last snapshot.
//! Results are presentation only and never mutate authoritative state.
use rm_sim_server::prediction::muzzle_for;
use rm_sim_world::{
    ChassisSnapshot, Field, FieldSnapshot, ProjectileSnapshot, Shot, StaticGeometry, tick_ns,
};
use std::{
    collections::BTreeMap,
    sync::{Arc, Condvar, Mutex},
};

/// Native requests replay no more than one second at once. Browser exchanges
/// also impose a four-tick integration budget.
const MAX_REPLAY_NS: u64 = 1_000_000_000;
/// The provisional ball ids a client draws are its own, never the host's.
/// `session_shots` marks an authoritative ball replaced by a prediction with the
/// same bit, so one constant states that contract.
pub(crate) const PROVISIONAL_BIT: u64 = 1 << 63;

/// One provisional ball the client draws for its own unconfirmed shot. It is
/// presentation only; the host's accepted ball replaces it by shot id.
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Flight {
    /// Client shot id, unique for the session and never reused. It keys the
    /// flight in a [`Request`] and the ball in an [`Output`].
    pub id: u64,
    /// Muzzle pose in world FLU metres. A provisional flight uses [`muzzle`];
    /// the host's captured muzzle overwrites it once the shot is accepted.
    pub muzzle: rm_sim_world::Pose,
    /// World time in ns at which the ball appears, so a lagging replay still
    /// launches it at the sampled tick. The host's executed time replaces the
    /// client's estimate on acceptance.
    pub launched_ns: u64,
    /// Weapon configuration the ball flies with: caliber and launch speed in
    /// metres per second along the muzzle's +x.
    pub shot: Shot,
    /// Host projectile id once the host accepted the shot, `None` while the
    /// flight is still a guess. A set value makes the client draw the host's own
    /// ball rather than its replay.
    pub authoritative: Option<u64>,
}
/// One replay request: the checkpoint to restore, the shooter's own prediction
/// and the provisional flights to launch into that restore.
#[derive(serde::Serialize, serde::Deserialize)]
pub struct Request {
    /// Prediction epoch. A change makes the worker rebuild its field, and lets
    /// the caller drop an output that an older epoch produced.
    pub epoch: u64,
    /// Id of the checkpoint in `snapshot`. A change also forces a rebuild.
    pub snapshot_id: u64,
    /// Last authoritative checkpoint, the rules' hidden state included, that the
    /// worker restores a whole `Field` from. Provisional balls meet the same
    /// terrain, armor and moving equipment as the host's ball.
    pub snapshot: Arc<FieldSnapshot>,
    /// Sampled peer poses applied when this checkpoint is restored. Commands
    /// are cleared so peers coast just as they do in the presented scene.
    pub peers: Vec<ChassisSnapshot>,
    /// The shooter's presented chassis. The restore applies it with the client's
    /// own command, so the local prediction drives it, while every other chassis
    /// is placed with a cleared command and coasts on the sampled pose.
    pub own: ChassisSnapshot,
    /// Client time in ns to advance the restored field to. The worker clamps the
    /// replay to `MAX_REPLAY_NS` past its current field time.
    pub time_ns: u64,
    /// Provisional flights to launch and report. The worker reads at most the
    /// first 128.
    pub flights: Vec<Flight>,
}
/// Provisional ball states one request produced, for the caller to draw.
#[derive(serde::Serialize, serde::Deserialize)]
pub struct Output {
    /// Epoch of the request that produced this output. The caller drops the
    /// output when the prediction epoch has moved on.
    pub epoch: u64,
    /// Ball state keyed by [`Flight::id`]. Each ball's own id also carries
    /// `PROVISIONAL_BIT`, so a provisional ball cannot be mistaken for a host
    /// projectile id.
    pub projectiles: BTreeMap<u64, ProjectileSnapshot>,
}
#[derive(Default)]
struct Mailbox {
    #[cfg(not(target_arch = "wasm32"))]
    request: Option<Request>,
    output: Option<Output>,
    stopped: bool,
    #[cfg(target_arch = "wasm32")]
    error: Option<String>,
    completed: u64,
    replay_ticks: u64,
}
/// One restored `Field` for provisional trajectories, never authoritative state.
/// Native execution uses a detached background thread. Browser execution uses
/// an isolated worker with at most four integration ticks per request.
pub struct Worker {
    shared: Arc<(Mutex<Mailbox>, Condvar)>,
    #[cfg(target_arch = "wasm32")]
    worker: Mutex<Option<crate::browser_prediction_worker::Handle>>,
    #[cfg(target_arch = "wasm32")]
    idle: std::sync::atomic::AtomicBool,
}
/// One restored field the client fires its unconfirmed shots into. It is the
/// whole field, so a provisional ball meets the same terrain, armor and moving
/// equipment the host's ball will. It is rebuilt on each new snapshot, which is
/// also how a confirmed shot is corrected: the host's own ball arrives in the
/// snapshot and replaces the guess.
struct Scene {
    field: Field,
    epoch: u64,
    snapshot_id: u64,
    time_ns: u64,
    /// Flight id to the ball this field gave it.
    fired: BTreeMap<u64, u64>,
}
impl Scene {
    fn restore(request: &Request, geometry: &StaticGeometry, floor_height_m: f64) -> Option<Self> {
        let mut field = Field::restore(&request.snapshot, geometry, floor_height_m).ok()?;
        // The owner stands where its own prediction puts it; peers coast on the
        // pose the caller sampled, with no command of their own to drive them.
        let _ = field.reset_chassis(&request.own);
        for state in &request.peers {
            if state.id != request.own.id {
                let mut peer = state.clone();
                peer.command = Default::default();
                let _ = field.reset_chassis(&peer);
            }
        }
        Some(Self {
            time_ns: field.time_ns(),
            field,
            epoch: request.epoch,
            snapshot_id: request.snapshot_id,
            fired: BTreeMap::new(),
        })
    }
    /// Launch every unconfirmed shot at its own tick and step to `end`.
    fn advance(&mut self, flights: &[&Flight], end: u64, max_ticks: usize) {
        let mut ticks = 0;
        loop {
            for flight in flights {
                if flight.authoritative.is_none()
                    && flight.launched_ns <= self.time_ns
                    && !self.fired.contains_key(&flight.id)
                    && let Ok(id) = self.field.fire(flight.muzzle, flight.shot, None)
                {
                    self.fired.insert(flight.id, id);
                }
            }
            if ticks >= max_ticks
                || self.time_ns.saturating_add(tick_ns()) > end
                || self.field.step(1).is_err()
            {
                return;
            }
            self.time_ns = self.time_ns.saturating_add(tick_ns());
            ticks += 1;
        }
    }
}
impl Worker {
    /// Spawn the worker with the client's verified collision geometry and the
    /// floor height in metres; `Field::restore` needs both and both stay fixed
    /// for the worker's life. Returns an error when the thread cannot be spawned.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn new(geometry: StaticGeometry, floor: f64) -> std::io::Result<Self> {
        let shared = Arc::new((Mutex::new(Mailbox::default()), Condvar::new()));
        let thread = shared.clone();
        std::thread::Builder::new()
            .name("projectile-prediction".into())
            .spawn(move || {
                let mut driver = ProjectileDriver::new(geometry, floor);
                loop {
                    let request = {
                        let Ok(mut mailbox) = thread.0.lock() else {
                            return;
                        };
                        while mailbox.request.is_none() && !mailbox.stopped {
                            let Ok(next) = thread.1.wait(mailbox) else {
                                return;
                            };
                            mailbox = next;
                        }
                        if mailbox.stopped {
                            return;
                        }
                        mailbox.request.take().expect("prediction request")
                    };
                    let output = driver.run(&request, usize::MAX);
                    if let Ok(mut mailbox) = thread.0.lock() {
                        mailbox.completed += 1;
                        mailbox.replay_ticks = driver.last_replay_ticks;
                        mailbox.output = Some(output);
                    }
                }
            })?;
        Ok(Self { shared })
    }
    /// Prepare lazy isolated browser trajectory replay with four ticks per request.
    #[cfg(target_arch = "wasm32")]
    pub fn new(_geometry: StaticGeometry, _floor: f64) -> std::io::Result<Self> {
        Ok(Self {
            shared: Arc::new((Mutex::new(Mailbox::default()), Condvar::new())),
            worker: Mutex::new(None),
            idle: std::sync::atomic::AtomicBool::new(true),
        })
    }
    /// Whether isolated execution has failed and its last output must be discarded.
    pub fn failed(&self) -> bool {
        #[cfg(target_arch = "wasm32")]
        {
            self.shared
                .0
                .try_lock()
                .ok()
                .is_some_and(|mailbox| mailbox.error.is_some())
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            false
        }
    }
    /// Completed trajectory replays and the last integration tick count.
    pub fn diagnostics(&self) -> Option<serde_json::Value> {
        self.shared.0.try_lock().ok().map(|mailbox| {
            let value=serde_json::json!({"completed":mailbox.completed,"last_replay_ticks":mailbox.replay_ticks});
            #[cfg(target_arch = "wasm32")]
            let value={
                let mut value=value;
                value["execution"]=serde_json::json!("web-worker");
                value["error"]=serde_json::json!(mailbox.error);
                value["worker"]=self.worker.lock().ok().and_then(|w|w.as_ref().map(|w|w.diagnostics())).unwrap_or(serde_json::Value::Null);
                value
            };
            value
        })
    }

    /// Hand a request to the worker and take the output of the previous one
    /// without blocking. Returns `None` while the worker holds the lock, so a
    /// caller that fell behind skips a frame instead of stalling the render
    /// thread, and a request that replaces an unread one is dropped. An empty
    /// flight list clears queued work and returns an empty output without replay.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn exchange(&self, request: Request) -> Option<Output> {
        let Ok(mut mailbox) = self.shared.0.try_lock() else {
            return None;
        };
        if request.flights.is_empty() {
            mailbox.request = None;
            mailbox.output = None;
            return Some(Output {
                epoch: request.epoch,
                projectiles: BTreeMap::new(),
            });
        }
        mailbox.request = Some(request);
        let result = mailbox.output.take();
        self.shared.1.notify_one();
        result
    }
    /// Lazily start an isolated browser trajectory worker on the first provisional flight.
    #[cfg(target_arch = "wasm32")]
    pub fn exchange(&self, request: Request) -> Option<Output> {
        let mut worker = self.worker.lock().ok()?;
        if request.flights.is_empty() {
            if !self.idle.swap(true, std::sync::atomic::Ordering::Relaxed)
                && let Some(worker) = worker.as_ref()
            {
                let _ = worker.post(crate::browser_prediction_worker::Request::ClearProjectile {
                    epoch: request.epoch,
                });
            }
            self.shared.0.lock().ok()?.output = None;
            return Some(Output {
                epoch: request.epoch,
                projectiles: BTreeMap::new(),
            });
        }
        self.idle.store(false, std::sync::atomic::Ordering::Relaxed);
        if worker.is_none() {
            let output = Arc::clone(&self.shared);
            match crate::browser_prediction_worker::Handle::new(Box::new(move |reply| {
                if let Ok(mut mailbox) = output.0.lock() {
                    match reply {
                        Ok(crate::browser_prediction_worker::Response::Projectile {
                            output,
                            ticks,
                        }) => {
                            mailbox.completed += 1;
                            mailbox.replay_ticks = ticks;
                            mailbox.output = Some(output);
                        }
                        Ok(crate::browser_prediction_worker::Response::ClearedProjectile) => {
                            mailbox.output = None
                        }
                        Ok(_) => mailbox.error = Some("wrong trajectory worker response".into()),
                        Err(error) => mailbox.error = Some(error),
                    }
                }
            })) {
                Ok(handle) => *worker = Some(handle),
                Err(error) => {
                    self.shared.0.lock().ok()?.error = Some(error.to_string());
                    return None;
                }
            }
        }
        let previous = self.shared.0.lock().ok()?.output.take();
        if let Err(error) =
            worker
                .as_ref()?
                .post(crate::browser_prediction_worker::Request::Projectile(
                    Box::new(request),
                ))
        {
            self.shared.0.lock().ok()?.error = Some(error);
        }
        previous
    }
}
impl Drop for Worker {
    fn drop(&mut self) {
        if let Ok(mut m) = self.shared.0.lock() {
            m.stopped = true;
            #[cfg(not(target_arch = "wasm32"))]
            self.shared.1.notify_one();
        }
    }
}

/// Full-field trajectory state used by both native and browser schedulers.
pub(crate) struct ProjectileDriver {
    scene: Option<Scene>,
    geometry: StaticGeometry,
    floor: f64,
    pub(crate) last_replay_ticks: u64,
}
impl ProjectileDriver {
    pub(crate) fn new(geometry: StaticGeometry, floor: f64) -> Self {
        Self {
            scene: None,
            geometry,
            floor,
            last_replay_ticks: 0,
        }
    }
    #[cfg(target_arch = "wasm32")]
    pub(crate) fn clear(&mut self) {
        self.scene = None;
    }
    pub(crate) fn run(&mut self, request: &Request, max_ticks: usize) -> Output {
        self.last_replay_ticks = 0;
        if self.scene.as_ref().is_none_or(|scene| {
            scene.epoch != request.epoch || scene.snapshot_id != request.snapshot_id
        }) {
            self.scene = Scene::restore(request, &self.geometry, self.floor);
        }
        let mut projectiles = BTreeMap::new();
        if let Some(scene) = &mut self.scene {
            let flights: Vec<&Flight> = request.flights.iter().take(128).collect();
            let end = request
                .time_ns
                .max(scene.time_ns)
                .min(scene.time_ns.saturating_add(MAX_REPLAY_NS));
            let before = scene.time_ns;
            scene.advance(&flights, end, max_ticks);
            self.last_replay_ticks = scene.time_ns.saturating_sub(before) / tick_ns();
            let balls = scene.field.projectile_snapshots();
            for flight in flights {
                let ball = flight
                    .authoritative
                    .or_else(|| scene.fired.get(&flight.id).copied())
                    .and_then(|id| balls.iter().find(|ball| ball.id == id));
                if let Some(ball) = ball {
                    let mut state = ball.clone();
                    state.id = PROVISIONAL_BIT | flight.id;
                    state.launched_ns = flight.launched_ns;
                    projectiles.insert(flight.id, state);
                }
            }
        }

        Output {
            epoch: request.epoch,
            projectiles,
        }
    }
}

/// Muzzle pose in world FLU metres for `own`, matching the pose the host derives
/// for the same turret and barrel. A provisional flight starts here until the
/// host's captured muzzle arrives. `command` is passed through for call-site
/// symmetry with the fire path and is not read.
pub fn muzzle(own: &ChassisSnapshot, command: rm_sim_world::ChassisCommand) -> rm_sim_world::Pose {
    muzzle_for(own, command)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rm_sim_world::{Caliber, ChassisConfig, ChassisPlacement, FieldConfig, Pose, Team};
    #[test]
    fn worker_json_roundtrip_preserves_accepted_and_provisional_flights() {
        let mut field = Field::new(&FieldConfig {
            chassis: vec![ChassisPlacement {
                config: ChassisConfig::default(),
                team: Team::Red,
                kind: rm_sim_world::RobotKind::Infantry,
                performance: None,
                spawn: Pose::at([-5., 0., 1.]),
            }],
            ..Default::default()
        })
        .unwrap();
        let shot = Shot::at_limit(Caliber::Mm17);
        let muzzle = Pose::at([0., 0., 2.]);
        let accepted = field.fire(muzzle, shot, None).unwrap();
        let snapshot = Arc::new(field.snapshot());
        let request = Request {
            epoch: 4,
            snapshot_id: 3,
            snapshot: Arc::clone(&snapshot),
            peers: snapshot.chassis.clone(),
            own: snapshot.chassis[0].clone(),
            time_ns: 4 * tick_ns(),
            flights: vec![
                Flight {
                    id: 1,
                    muzzle,
                    launched_ns: 0,
                    shot,
                    authoritative: Some(accepted),
                },
                Flight {
                    id: 2,
                    muzzle,
                    launched_ns: 0,
                    shot,
                    authoritative: None,
                },
            ],
        };
        let decoded: Request =
            serde_json::from_slice(&serde_json::to_vec(&request).unwrap()).unwrap();
        assert_eq!(*decoded.snapshot, *snapshot);
        assert_eq!(decoded.flights[0].authoritative, Some(accepted));
        assert_eq!(decoded.flights[1].authoritative, None);
        let geometry = field.static_geometry_snapshot();
        let expected = ProjectileDriver::new(geometry.clone(), 0.).run(&request, 4);
        let actual = ProjectileDriver::new(geometry, 0.).run(&decoded, 4);
        let transferred: Output =
            serde_json::from_slice(&serde_json::to_vec(&actual).unwrap()).unwrap();
        assert_eq!(transferred.epoch, 4);
        assert_eq!(transferred.projectiles, expected.projectiles);
        assert_eq!(transferred.projectiles.len(), 2);
    }

    #[test]
    fn caller_budget_matches_native_trajectory_and_epoch_discards_old_flights() {
        let field = Field::new(&FieldConfig {
            chassis: vec![ChassisPlacement {
                config: ChassisConfig::default(),
                team: Team::Red,
                kind: rm_sim_world::RobotKind::Infantry,
                performance: None,
                spawn: Pose::at([-5., 0., 1.]),
            }],
            ..Default::default()
        })
        .unwrap();
        let snapshot = Arc::new(field.snapshot());
        let tick = tick_ns();
        let mut request = Request {
            epoch: 1,
            snapshot_id: 1,
            snapshot: Arc::clone(&snapshot),
            peers: snapshot.chassis.clone(),
            own: snapshot.chassis[0].clone(),
            time_ns: 20 * tick,
            flights: vec![Flight {
                id: 1,
                muzzle: Pose::at([0., 0., 2.]),
                launched_ns: 0,
                shot: Shot::at_limit(Caliber::Mm17),
                authoritative: None,
            }],
        };
        let geometry = field.static_geometry_snapshot();
        let mut expected = ProjectileDriver::new(geometry.clone(), 0.);
        let expected = expected.run(&request, usize::MAX);
        let mut driver = ProjectileDriver::new(geometry.clone(), 0.);
        for part in 1..=5 {
            let output = driver.run(&request, 4);
            assert_eq!(
                driver.last_replay_ticks, 4,
                "trajectory integration budget, part {part}"
            );
            assert_eq!(driver.scene.as_ref().unwrap().time_ns, part * 4 * tick);
            if part == 5 {
                assert_eq!(
                    output.projectiles, expected.projectiles,
                    "bounded trajectory matches native physics"
                );
            }
        }
        request.epoch = 2;
        request.flights[0].id = 2;
        let output = driver.run(&request, 4);
        let fresh = ProjectileDriver::new(geometry, 0.).run(&request, 4);
        assert_eq!(output.epoch, 2);
        assert!(
            !output.projectiles.contains_key(&1),
            "old flight cannot survive an epoch change"
        );
        assert_eq!(output.projectiles, fresh.projectiles);
        assert_eq!(
            field.snapshot(),
            *snapshot,
            "trajectory replay never mutates authority"
        );
    }

    #[test]
    fn restore_uses_sampled_peer_poses_without_mutating_shared_checkpoint() {
        let placement = ChassisPlacement {
            config: ChassisConfig::default(),
            team: Team::Red,
            kind: rm_sim_world::RobotKind::Infantry,
            performance: None,
            spawn: Pose::at([-5., 0., 1.]),
        };
        let field = Field::new(&FieldConfig {
            chassis: vec![placement.clone(), placement],
            ..Default::default()
        })
        .unwrap();
        let snapshot = Arc::new(field.snapshot());
        let mut peer = snapshot.chassis[1].clone();
        peer.pose.translation_m[0] = 5.;
        let request = Request {
            epoch: 1,
            snapshot_id: 1,
            snapshot: Arc::clone(&snapshot),
            peers: vec![peer.clone()],
            own: snapshot.chassis[0].clone(),
            time_ns: 0,
            flights: vec![],
        };
        let scene = Scene::restore(&request, &field.static_geometry_snapshot(), 0.).unwrap();
        assert_eq!(scene.field.snapshot().chassis[1].pose, peer.pose);
        assert_ne!(snapshot.chassis[1].pose, peer.pose);
        // An idle request never enters the worker mailbox: there is no field
        // to restore or step until a flight exists.
        let worker = Worker::new(field.static_geometry_snapshot(), 0.).unwrap();
        assert!(worker.exchange(request).unwrap().projectiles.is_empty());
        assert!(worker.shared.0.lock().unwrap().request.is_none());
    }

    #[test]
    fn authoritative_checkpoint_replaces_provisional_trajectory_and_epoch_clears_it() {
        let field = Field::new(&FieldConfig {
            chassis: vec![ChassisPlacement {
                config: ChassisConfig::default(),
                team: Team::Red,
                kind: rm_sim_world::RobotKind::Infantry,
                performance: None,
                spawn: Pose::at([-5., 0., 1.]),
            }],
            ..Default::default()
        })
        .unwrap();
        let worker = Worker::new(field.static_geometry_snapshot(), 0.).unwrap();
        let mut snapshot = field.snapshot();
        let mut flight = Flight {
            id: 1,
            muzzle: Pose::at([0., 0., 2.]),
            launched_ns: 0,
            shot: Shot::at_limit(Caliber::Mm17),
            authoritative: None,
        };
        let wait = |snapshot: &FieldSnapshot, flight: &Flight, epoch, accepted| {
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
            loop {
                // Ask for 100 ms of flight, aligned up so a whole tick of the
                // fixed clock reaches it.
                let time_ns =
                    100_000_000_u64.div_ceil(rm_sim_world::tick_ns()) * rm_sim_world::tick_ns();
                if let Some(output) = worker.exchange(Request {
                    epoch,
                    snapshot_id: if accepted { 2 } else { 1 },
                    snapshot: Arc::new(snapshot.clone()),
                    peers: snapshot.chassis.clone(),
                    own: snapshot.chassis[0].clone(),
                    time_ns,
                    flights: if epoch == 2 {
                        vec![]
                    } else {
                        vec![flight.clone()]
                    },
                }) && output.epoch == epoch
                {
                    if epoch == 2 {
                        assert!(output.projectiles.is_empty());
                        break None;
                    }
                    if let Some(p) = output.projectiles.get(&1)
                        && (!accepted || p.position_m[1] > 1.9)
                    {
                        break Some(p.clone());
                    }
                }
                assert!(std::time::Instant::now() < deadline);
                std::thread::sleep(std::time::Duration::from_millis(1));
            }
        };
        let provisional = wait(&snapshot, &flight, 1, false).unwrap();
        assert!(provisional.position_m[0] > 1.);
        flight.authoritative = Some(7);
        snapshot.tick = 100_000_000_u64.div_ceil(rm_sim_world::tick_ns());
        snapshot.time_ns = snapshot.tick * rm_sim_world::tick_ns();
        snapshot.projectiles.push(ProjectileSnapshot {
            id: 7,
            caliber: Caliber::Mm17,
            launched_ns: 0,
            position_m: [0., 2., 2.],
            velocity_m_s: [0., 20., 0.],
            angular_velocity_rad_s: [0.; 3],
            shooter: None,
            first_contact_ns: None,
            dwell_since_ns: None,
        });
        let corrected = wait(&snapshot, &flight, 1, true).unwrap();
        assert_eq!(corrected.id, provisional.id);
        assert_eq!(corrected.position_m, [0., 2., 2.]);
        wait(&snapshot, &flight, 2, true);
        assert_eq!(field.snapshot().shots_fired, 0);
    }
}
