// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Conservative Sentry navigation prototype: shared CAD bake, bounded A* and chassis input.
//! This is host-side control, not a new physics world or a firing/targeting policy.
use crate::collision_mesh::GroundMesh;
use rm_simulator_world::{ChassisCommand, Field, StaticGeometry};
use serde::{Deserialize, Serialize};
use std::{
    cmp::Reverse,
    collections::{BTreeMap, BinaryHeap},
    sync::{Arc, mpsc},
};

/// Bake grid pitch, metres; a simulator navigation setting, not a rule constant.
pub const CELL_M: f64 = 0.25;
/// Conservative omnidirectional robot radius including bumper and clearance, metres.
pub const RADIUS_M: f64 = 0.37;
const GRADE: f64 = 0.213; // tan(12 degrees), conservative omni terrain limit.
const SPEED_M_S: f64 = 0.65;
const PLANNING_RADIUS_M: f64 = RADIUS_M + 0.12;
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn center(p: [f64; 3]) -> [f64; 3] {
    [p[0], p[1], p[2] + 0.40]
}

/// Shared baked-map statistics, independent of how many Sentries use it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Bake {
    /// Number of sampled XY columns.
    pub columns: usize,
    /// Collision-clear supported nodes, including separate height layers.
    pub nodes: usize,
    /// Directed, collision-checked neighbor edges.
    pub edges: usize,
}
/// Host navigation lifecycle; receiving an intent alone never means arrival.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Phase {
    /// Shared map bake is still on a worker.
    Baking,
    /// Spawned and stopped, without a destination.
    Idle,
    /// Waiting for the worker's bounded path search.
    Planning,
    /// Following validated waypoints with ordinary wheel commands.
    Following,
    /// Reached the destination and stopped.
    Arrived,
    /// Cannot safely continue; a new intent can request another path.
    Blocked,
    /// Search could not connect the exact start and goal safely.
    Unreachable,
}
/// Small caller-visible navigation state, carried with authoritative snapshots.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct View {
    /// Stable autonomous Sentry chassis id.
    pub chassis: u32,
    /// Current controller phase.
    pub phase: Phase,
    /// Most recent requested XY location, world FLU metres.
    pub goal_m: Option<[f64; 2]>,
    /// Smoothed supported path in world FLU metres; at most 256 points.
    pub path_m: Vec<[f64; 3]>,
    /// Next waypoint index.
    pub waypoint: usize,
    /// User-readable acceptance/failure explanation, at most 160 characters.
    pub message: String,
    /// Shared map summary, once baked.
    pub bake: Bake,
}
impl View {
    pub(crate) fn valid(&self) -> bool {
        self.path_m.len() <= 256
            && self.waypoint <= self.path_m.len()
            && self.message.len() <= 160
            && self
                .path_m
                .iter()
                .flatten()
                .chain(self.goal_m.iter().flatten())
                .all(|x| x.is_finite())
    }
}
/// Explicit operator policy footprint, before robot-radius inflation.
#[derive(Clone, Debug)]
pub(crate) struct Exclusion {
    min: [f64; 2],
    max: [f64; 2],
}
impl Exclusion {
    fn crosses(&self, a: [f64; 3], b: [f64; 3]) -> bool {
        let mut enter: f64 = 0.;
        let mut leave: f64 = 1.;
        for i in 0..2 {
            let lo = self.min[i] - PLANNING_RADIUS_M;
            let hi = self.max[i] + PLANNING_RADIUS_M;
            let d = b[i] - a[i];
            if d.abs() < 1e-12 {
                if a[i] < lo || a[i] > hi {
                    return false;
                }
            } else {
                let u = (lo - a[i]) / d;
                let v = (hi - a[i]) / d;
                enter = enter.max(u.min(v));
                leave = leave.min(u.max(v));
                if enter > leave {
                    return false;
                }
            }
        }
        true
    }
}
/// Approximate Road card envelope from the registered 2027 diagram, with 15 cm
/// survey allowance. This is a navigation exclusion, never a referee zone.
pub(crate) fn road_exclusions(season: rm_simulator_world::season::Season) -> Vec<Exclusion> {
    use rm_simulator_world::{Team, season::Season};
    if season != Season::Rm2027 {
        return vec![];
    }
    let areas = crate::zone_catalog::areas(season);
    [Team::Red, Team::Blue]
        .into_iter()
        .map(|side| {
            let mut region = Exclusion {
                min: [f64::INFINITY; 2],
                max: [f64::NEG_INFINITY; 2],
            };
            for p in areas
                .iter()
                .filter(|a| {
                    a.side == side && (a.id.ends_with("road-low") || a.id.ends_with("road-high"))
                })
                .flat_map(|a| &a.polygon_m)
            {
                for (i, value) in p.iter().enumerate() {
                    region.min[i] = region.min[i].min(*value - 0.15);
                    region.max[i] = region.max[i].max(*value + 0.15);
                }
            }
            region
        })
        .collect()
}
#[derive(Clone)]
struct Surface {
    ground: Option<Arc<GroundMesh>>,
    floor: f64,
    exclusions: Vec<Exclusion>,
}
impl Surface {
    fn height(&self, x: f64, y: f64, max: f64) -> Option<f64> {
        self.ground.as_ref().map_or_else(
            || (self.floor <= max).then_some(self.floor),
            |g| g.ground_height_below(x, y, max),
        )
    }
    fn supported(&self, p: [f64; 3]) -> bool {
        if self.exclusions.iter().any(|r| r.crosses(p, p)) {
            return false;
        }
        let Some(center_z) = self.height(p[0], p[1], p[2] + 0.30) else {
            return false;
        };
        // Sample the inflated footprint, including wheel diagonals. Reject a
        // height discontinuity under the bumper even if the centre is smooth.
        (0..8).all(|i| {
            let angle = i as f64 * std::f64::consts::FRAC_PI_4;
            let x = PLANNING_RADIUS_M * angle.cos();
            let y = PLANNING_RADIUS_M * angle.sin();
            self.height(p[0] + x, p[1] + y, center_z + 0.30)
                .is_some_and(|z| (z - center_z).abs() <= 0.012 + PLANNING_RADIUS_M * GRADE)
        })
    }
}
struct Map {
    nodes: Vec<[f64; 3]>,
    edges: Vec<Vec<(usize, u32)>>,
    geometry: StaticGeometry,
    surface: Surface,
    summary: Bake,
}
#[derive(Clone)]
struct Obstacle {
    point: [f64; 3],
    radius: f64,
}
fn obstacles(field: &Field, own: u32) -> Vec<Obstacle> {
    field
        .chassis_snapshots()
        .iter()
        .filter(|c| c.id != own)
        .map(|c| Obstacle {
            point: c.pose.translation_m,
            radius: 0.42,
        })
        .collect()
}
fn avoids(a: [f64; 3], b: [f64; 3], obstacles: &[Obstacle]) -> bool {
    obstacles.iter().all(|o| {
        if (o.point[2] - ((a[2] + b[2]) * 0.5 + 0.25)).abs() > 0.8 {
            return true;
        }
        let d = [b[0] - a[0], b[1] - a[1]];
        let len = d[0] * d[0] + d[1] * d[1];
        let t = if len < 1e-12 {
            0.
        } else {
            ((o.point[0] - a[0]) * d[0] + (o.point[1] - a[1]) * d[1]) / len
        }
        .clamp(0., 1.);
        (o.point[0] - a[0] - t * d[0]).hypot(o.point[1] - a[1] - t * d[1])
            > RADIUS_M + o.radius + 0.08
    })
}
impl Map {
    fn line_clear(&self, a: [f64; 3], b: [f64; 3]) -> bool {
        let length = distance(a, b);
        if self.surface.exclusions.iter().any(|r| r.crosses(a, b))
            || (a[2] - b[2]).abs() > 0.015 + length * GRADE
        {
            return false;
        }
        let n = (length / 0.03).ceil().max(1.) as usize;
        let mut previous = [None::<f64>; 9];
        for i in 0..=n {
            let t = i as f64 / n as f64;
            let p = std::array::from_fn(|j| a[j] + t * (b[j] - a[j]));
            if !self.surface.supported(p) {
                return false;
            }
            for (track, old) in previous.iter_mut().enumerate() {
                let angle = track as f64 * std::f64::consts::FRAC_PI_4;
                let r = if track == 8 { 0. } else { PLANNING_RADIUS_M };
                let Some(height) = self.surface.height(
                    p[0] + r * angle.cos(),
                    p[1] + r * angle.sin(),
                    p[2] + 0.30,
                ) else {
                    return false;
                };
                // Four mm allows CAD sheet/seam noise; curbs under any part of
                // the footprint must not disappear between coarse grid nodes.
                if old.is_some_and(|z| (height - z).abs() > GRADE * length / n as f64 + 0.004) {
                    return false;
                }
                *old = Some(height);
            }
        }
        self.geometry
            .cylinder_sweep_clear(center(a), center(b), PLANNING_RADIUS_M, 0.24)
    }

    fn bake(geometry: StaticGeometry, surface: Surface, bounds: [f64; 4]) -> Self {
        let nx = ((bounds[2] - bounds[0]) / CELL_M).floor() as usize + 1;
        let ny = ((bounds[3] - bounds[1]) / CELL_M).floor() as usize + 1;
        let mut columns = vec![Vec::<usize>::new(); nx * ny];
        let mut map = Self {
            nodes: vec![],
            edges: vec![],
            geometry,
            surface,
            summary: Bake {
                columns: nx * ny,
                ..Default::default()
            },
        };
        for y in 0..ny {
            for x in 0..nx {
                let xy = [bounds[0] + x as f64 * CELL_M, bounds[1] + y as f64 * CELL_M];
                for max in [0.10, 0.35, 0.65, 1.05, 1.45] {
                    let Some(z) = map.surface.height(xy[0], xy[1], max) else {
                        continue;
                    };
                    let p = [xy[0], xy[1], z];
                    if columns[y * nx + x]
                        .iter()
                        .any(|&i| (map.nodes[i][2] - z).abs() < 0.04)
                    {
                        continue;
                    }
                    if map.surface.supported(p)
                        && map.geometry.cylinder_sweep_clear(
                            center(p),
                            center(p),
                            PLANNING_RADIUS_M,
                            0.24,
                        )
                    {
                        columns[y * nx + x].push(map.nodes.len());
                        map.nodes.push(p);
                        map.edges.push(vec![]);
                    }
                }
            }
        }
        for y in 0..ny {
            for x in 0..nx {
                for (dx, dy) in [(1, 0), (0, 1), (1, 1), (-1, 1)] {
                    let (xx, yy) = (x as isize + dx, y as isize + dy);
                    if xx < 0 || xx >= nx as isize || yy >= ny as isize {
                        continue;
                    }
                    for &a in &columns[y * nx + x] {
                        for &b in &columns[yy as usize * nx + xx as usize] {
                            if map.line_clear(map.nodes[a], map.nodes[b]) {
                                let cost = (distance(map.nodes[a], map.nodes[b]) * 1000.
                                    + (map.nodes[a][2] - map.nodes[b][2]).abs() * 500.)
                                    .ceil() as u32;
                                map.edges[a].push((b, cost));
                                map.edges[b].push((a, cost));
                            }
                        }
                    }
                }
            }
        }
        map.summary.nodes = map.nodes.len();
        map.summary.edges = map.edges.iter().map(Vec::len).sum();
        map
    }
    fn plan(
        &self,
        start: [f64; 3],
        goal: [f64; 2],
        dynamic: &[Obstacle],
        live: &StaticGeometry,
    ) -> Result<Vec<[f64; 3]>, String> {
        let clear = |a, b| {
            self.line_clear(a, b)
                && avoids(a, b, dynamic)
                && live.cylinder_sweep_clear(center(a), center(b), PLANNING_RADIUS_M, 0.24)
        };
        let begin = self
            .nodes
            .iter()
            .enumerate()
            .filter(|(_, p)| distance(start, **p) <= 0.65 && clear(start, **p))
            .min_by(|a, b| distance(start, *a.1).total_cmp(&distance(start, *b.1)))
            .map(|(i, _)| i)
            .ok_or("start has no safe connection to the baked map")?;
        let mut goals = BTreeMap::new();
        for (i, &p) in self.nodes.iter().enumerate() {
            if (p[0] - goal[0]).hypot(p[1] - goal[1]) > 0.50 {
                continue;
            }
            if let Some(z) = self.surface.height(goal[0], goal[1], p[2] + 0.15) {
                let end = [goal[0], goal[1], z];
                if clear(p, end) {
                    goals.insert(i, end);
                }
            }
        }
        if goals.is_empty() {
            return Err("goal has no collision-clear supported location".into());
        }
        let heuristic = |i: usize| {
            goals
                .keys()
                .map(|&g| (distance(self.nodes[i], self.nodes[g]) * 1000.) as u32)
                .min()
                .unwrap_or(0)
        };
        let mut costs = vec![u32::MAX; self.nodes.len()];
        let mut parent = vec![usize::MAX; self.nodes.len()];
        let mut heap = BinaryHeap::new();
        costs[begin] = 0;
        heap.push((Reverse(heuristic(begin)), Reverse(begin), 0u32));
        let mut end = None;
        while let Some((_, Reverse(a), cost)) = heap.pop() {
            if cost != costs[a] {
                continue;
            }
            if goals.contains_key(&a) {
                end = Some(a);
                break;
            }
            for &(b, w) in &self.edges[a] {
                let next = cost.saturating_add(w);
                if next < costs[b]
                    && avoids(self.nodes[a], self.nodes[b], dynamic)
                    && live.cylinder_sweep_clear(
                        center(self.nodes[a]),
                        center(self.nodes[b]),
                        PLANNING_RADIUS_M,
                        0.24,
                    )
                {
                    costs[b] = next;
                    parent[b] = a;
                    heap.push((Reverse(next.saturating_add(heuristic(b))), Reverse(b), next));
                }
            }
        }
        let mut at = end.ok_or("no route connects the start and goal")?;
        let finish = goals[&at];
        let mut raw = vec![finish];
        loop {
            raw.push(self.nodes[at]);
            if at == begin {
                break;
            }
            at = parent[at];
        }
        raw.push(start);
        raw.reverse();
        let mut result = vec![start];
        let mut cursor = 0;
        while cursor + 1 < raw.len() {
            let mut next = raw.len() - 1;
            while next > cursor + 1 && !clear(raw[cursor], raw[next]) {
                next -= 1;
            }
            result.push(raw[next]);
            cursor = next;
        }
        if result.len() > 256 {
            return Err("route exceeds prototype waypoint budget".into());
        }
        Ok(result)
    }
}
struct Job {
    id: u32,
    generation: u64,
    start: [f64; 3],
    goal: [f64; 2],
    obstacles: Vec<Obstacle>,
    live: StaticGeometry,
}
enum Reply {
    Baked(Bake),
    Route(u32, u64, Result<Vec<[f64; 3]>, String>),
}
struct Agent {
    view: View,
    generation: u64,
    pending: bool,
    last: [f64; 3],
    stalled: u64,
    retries: u8,
    failed: Vec<Obstacle>,
}
impl Agent {
    fn replan(&mut self, from: [f64; 3], toward: [f64; 3], learn: bool) {
        if self.retries >= 3 {
            self.pending = false;
            self.view.phase = Phase::Blocked;
            self.view.message = "Stopped after three recovery attempts; send a new goal".into();
            return;
        }
        if learn {
            // Block a small region ahead, not the occupied start: the next
            // search must be able to retreat around the failed approach.
            let d = distance(from, toward).max(0.001);
            let t = 0.75 / d;
            let point = [
                from[0] + t * (toward[0] - from[0]),
                from[1] + t * (toward[1] - from[1]),
                from[2] + 0.25,
            ];
            if self.failed.len() == 64 {
                self.failed.remove(0);
            }
            self.failed.push(Obstacle {
                point,
                radius: 0.05,
            });
        }
        self.retries += 1;
        self.generation += 1;
        self.pending = true;
        self.stalled = 0;
        self.view.phase = Phase::Planning;
        self.view.message = format!("Replanning around obstruction (attempt {}/3)", self.retries);
        self.view.path_m.clear();
        self.view.waypoint = 0;
    }
}
/// One worker and shared baked map for all host-owned navigation Sentries.
pub(crate) struct Service {
    sender: mpsc::SyncSender<Job>,
    receiver: mpsc::Receiver<Reply>,
    agents: BTreeMap<u32, Agent>,
    surface: Surface,
    summary: Option<Bake>,
}
impl Service {
    pub(crate) fn new(
        geometry: StaticGeometry,
        ground: Option<GroundMesh>,
        floor: f64,
        bounds: [f64; 4],
        exclusions: Vec<Exclusion>,
    ) -> Result<Self, String> {
        if !bounds.into_iter().chain([floor]).all(f64::is_finite)
            || bounds[2] <= bounds[0]
            || bounds[3] <= bounds[1]
            || bounds[2] - bounds[0] > 40.
            || bounds[3] - bounds[1] > 25.
        {
            return Err("invalid or oversized navigation bake bounds".into());
        }
        let surface = Surface {
            ground: ground.map(Arc::new),
            floor,
            exclusions,
        };
        let worker_surface = surface.clone();
        let (sender, jobs) = mpsc::sync_channel::<Job>(1);
        let (replies, receiver) = mpsc::sync_channel(4);
        std::thread::Builder::new()
            .name("rm-navigation".into())
            .spawn(move || {
                let map = Map::bake(geometry, worker_surface, bounds);
                if replies.send(Reply::Baked(map.summary.clone())).is_err() {
                    return;
                }
                while let Ok(job) = jobs.recv() {
                    let path = map.plan(job.start, job.goal, &job.obstacles, &job.live);
                    if replies
                        .send(Reply::Route(job.id, job.generation, path))
                        .is_err()
                    {
                        break;
                    }
                }
            })
            .map_err(|e| e.to_string())?;
        Ok(Self {
            sender,
            receiver,
            agents: BTreeMap::new(),
            surface,
            summary: None,
        })
    }
    pub(crate) fn add(&mut self, id: u32) {
        self.agents.insert(
            id,
            Agent {
                view: View {
                    chassis: id,
                    phase: if self.summary.is_some() {
                        Phase::Idle
                    } else {
                        Phase::Baking
                    },
                    goal_m: None,
                    path_m: vec![],
                    waypoint: 0,
                    message: "Spawned; send a map location".into(),
                    bake: self.summary.clone().unwrap_or_default(),
                },
                generation: 0,
                pending: false,
                last: [0.; 3],
                stalled: 0,
                retries: 0,
                failed: vec![],
            },
        );
    }
    pub(crate) fn owns(&self, id: u32) -> bool {
        self.agents.contains_key(&id)
    }
    pub(crate) fn remove(&mut self, id: u32) {
        self.agents.remove(&id);
    }
    pub(crate) fn stop(&mut self, id: u32) {
        if let Some(a) = self.agents.get_mut(&id) {
            a.generation += 1;
            a.pending = false;
            a.view.phase = Phase::Idle;
            a.view.goal_m = None;
            a.view.path_m.clear();
            a.view.waypoint = 0;
            a.view.message = "Stopped by operator".into();
        }
    }
    pub(crate) fn goal(&mut self, id: u32, goal: [f64; 2]) -> Result<(), String> {
        let baking = self.summary.is_none();
        let a = self.agents.get_mut(&id).ok_or("not a navigation Sentry")?;
        a.generation += 1;
        a.pending = true;
        a.view.goal_m = Some(goal);
        a.view.phase = if baking {
            Phase::Baking
        } else {
            Phase::Planning
        };
        a.view.path_m.clear();
        a.view.waypoint = 0;
        a.view.message = "Planning from actual chassis position".into();
        a.stalled = 0;
        a.retries = 0;
        Ok(())
    }
    pub(crate) fn views(&self) -> Vec<View> {
        self.agents.values().map(|a| a.view.clone()).collect()
    }
    pub(crate) fn poll(&mut self, field: &Field) {
        while let Ok(reply) = self.receiver.try_recv() {
            match reply {
                Reply::Baked(summary) => {
                    self.summary = Some(summary.clone());
                    for a in self.agents.values_mut() {
                        a.view.bake = summary.clone();
                        if a.view.phase == Phase::Baking {
                            a.view.phase = if a.view.goal_m.is_some() {
                                Phase::Planning
                            } else {
                                Phase::Idle
                            };
                        }
                    }
                }
                Reply::Route(id, generation, path) => {
                    let Some(a) = self
                        .agents
                        .get_mut(&id)
                        .filter(|a| a.generation == generation)
                    else {
                        continue;
                    };
                    match path {
                        Ok(path) => {
                            let moved = field
                                .chassis_snapshot(id)
                                .is_none_or(|b| distance(b.pose.translation_m, path[0]) > 0.3);
                            if moved {
                                a.view.phase = Phase::Blocked;
                                a.view.message =
                                    "Chassis moved during planning; send the goal again".into();
                            } else {
                                a.last = path[0];
                                a.view.path_m = path;
                                a.view.waypoint = 1;
                                a.view.phase = Phase::Following;
                                a.view.message = "Following collision-checked path".into();
                            }
                        }
                        Err(message) => {
                            a.view.phase = Phase::Unreachable;
                            a.view.message = message;
                        }
                    }
                }
            }
        }
        for (&id, a) in &mut self.agents {
            if !a.pending {
                continue;
            }
            let Some(body) = field.chassis_snapshot(id) else {
                continue;
            };
            let p = body.pose.translation_m;
            let Some(z) = self.surface.height(p[0], p[1], p[2] + 0.05) else {
                a.pending = false;
                a.view.phase = Phase::Unreachable;
                a.view.message = "No support beneath Sentry".into();
                continue;
            };
            let job = Job {
                id,
                generation: a.generation,
                start: [p[0], p[1], z],
                goal: a.view.goal_m.unwrap(),
                obstacles: obstacles(field, id)
                    .into_iter()
                    .chain(a.failed.clone())
                    .collect(),
                live: field.static_geometry_snapshot(),
            };
            match self.sender.try_send(job) {
                Ok(()) => a.pending = false,
                Err(mpsc::TrySendError::Full(_)) => break,
                Err(_) => {
                    a.pending = false;
                    a.view.phase = Phase::Blocked;
                    a.view.message = "Navigation worker stopped".into();
                }
            }
        }
    }
    pub(crate) fn tick(&mut self, field: &mut Field) {
        if !field.tick().is_multiple_of(8) {
            return;
        }
        let live = field.static_geometry_snapshot();
        for (&id, a) in &mut self.agents {
            let Some(body) = field.chassis_snapshot(id) else {
                continue;
            };
            let mut command = ChassisCommand::default();
            if a.view.phase == Phase::Following && !body.defeated {
                let p = body.pose.translation_m;
                let last = *a.view.path_m.last().unwrap();
                while a.view.waypoint + 1 < a.view.path_m.len()
                    && distance(p, a.view.path_m[a.view.waypoint]) < 0.08
                {
                    a.view.waypoint += 1;
                }
                if distance(p, last) < 0.12 {
                    a.view.phase = Phase::Arrived;
                    a.view.message = "Arrived; wheels stopped".into();
                } else {
                    let waypoint = a.view.path_m[a.view.waypoint];
                    let d = distance(p, waypoint);
                    let t = (0.45 / d.max(0.001)).min(1.);
                    let floor = self.surface.height(p[0], p[1], p[2] + 0.05).unwrap_or(p[2]);
                    let from = [p[0], p[1], floor];
                    let to = [
                        p[0] + t * (waypoint[0] - p[0]),
                        p[1] + t * (waypoint[1] - p[1]),
                        floor + t * (waypoint[2] - floor),
                    ];
                    if !avoids(from, to, &obstacles(field, id))
                        || !live.cylinder_sweep_clear(center(from), center(to), RADIUS_M, 0.24)
                    {
                        a.replan(from, to, false);
                    } else {
                        let speed = SPEED_M_S.min(distance(p, last) * 1.6).min(d * 2.0);
                        let vx = (waypoint[0] - p[0]) / d.max(0.001) * speed;
                        let vy = (waypoint[1] - p[1]) / d.max(0.001) * speed;
                        let [w, x, y, z] = body.pose.rotation_wxyz;
                        let yaw = (2. * (w * z + x * y)).atan2(1. - 2. * (y * y + z * z));
                        command.forward_m_s = vx * yaw.cos() + vy * yaw.sin();
                        command.left_m_s = -vx * yaw.sin() + vy * yaw.cos();
                        if distance(p, a.last) > 0.025 {
                            a.last = p;
                            a.stalled = 0;
                        } else {
                            a.stalled += 8;
                        }
                        if a.stalled > 384 {
                            a.replan(from, waypoint, true);
                            command = ChassisCommand::default();
                        }
                    }
                }
            } else if body.defeated && a.view.phase == Phase::Following {
                a.view.phase = Phase::Blocked;
                a.view.message = "Sentry defeated; stopped".into();
            }
            let _ = field.command_chassis(id, command);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rm_simulator_world::{
        ChassisConfig, ChassisPlacement, FieldConfig, Pose, RobotKind, Team, tick_ns,
    };
    fn wall() -> StaticGeometry {
        let mut physics = rm_simulator_world::projectile::WorldPhysics::new(&[], 0.);
        physics.add_static_box(Pose::at([0., 0., 0.5]), [0.03, 2.5, 1.0]);
        physics.static_geometry_snapshot(true)
    }
    #[test]
    fn bake_and_shortcut_detour_around_a_wall_and_reject_occupied_goal() {
        let geometry = wall();
        let map = Map::bake(
            geometry.clone(),
            Surface {
                ground: None,
                floor: 0.,
                exclusions: vec![],
            },
            [-3., -3., 3., 3.],
        );
        let path = map.plan([-2., 0., 0.], [2., 0.], &[], &geometry).unwrap();
        assert!(path.len() >= 3);
        assert!(path.iter().any(|p| p[1].abs() > 1.25 + RADIUS_M));
        assert!(path.windows(2).all(|p| map.line_clear(p[0], p[1])));
        assert!(map.plan([-2., 0., 0.], [0., 0.], &[], &geometry).is_err());
        let blocked = vec![Obstacle {
            point: [2., 0., 0.2],
            radius: 0.5,
        }];
        assert!(
            map.plan([-2., 0., 0.], [2., 0.], &blocked, &geometry)
                .is_err()
        );
    }
    #[test]
    fn abrupt_low_curb_is_rejected_while_a_continuous_ramp_connects() {
        use crate::collision_mesh::CollisionMesh;
        for ramp in [false, true] {
            let z0 = if ramp { 0. } else { 0.04 };
            let z1 = if ramp { 0.20 } else { 0.04 };
            let mesh = CollisionMesh {
                vertices_m: vec![
                    [-2., -2., 0.],
                    [0., -2., 0.],
                    [0., 2., 0.],
                    [-2., 2., 0.],
                    [0., -2., z0],
                    [2., -2., z1],
                    [2., 2., z1],
                    [0., 2., z0],
                ],
                triangles: vec![[0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7]],
            };
            let map = Map::bake(
                StaticGeometry::default(),
                Surface {
                    ground: Some(Arc::new(mesh.into())),
                    floor: 0.,
                    exclusions: vec![],
                },
                [-1.5, -1.5, 1.5, 1.5],
            );
            assert_eq!(
                map.plan([-1., 0., 0.], [1., 0.], &[], &StaticGeometry::default())
                    .is_ok(),
                ramp
            );
        }
    }
    #[test]
    fn inflated_exclusions_survive_path_shortcutting() {
        let region = Exclusion {
            min: [-0.2, -0.8],
            max: [0.2, 0.8],
        };
        let geometry = StaticGeometry::default();
        let map = Map::bake(
            geometry.clone(),
            Surface {
                ground: None,
                floor: 0.,
                exclusions: vec![region.clone()],
            },
            [-3., -3., 3., 3.],
        );
        let path = map.plan([-2., 0., 0.], [2., 0.], &[], &geometry).unwrap();
        assert!(path.len() >= 3);
        assert!(path.windows(2).all(|p| !region.crosses(p[0], p[1])));
        assert!(map.plan([-2., 0., 0.], [0., 0.], &[], &geometry).is_err());
        assert!(region.crosses([0.6, 0., 0.], [0.6, 0., 0.]));
        let roads = road_exclusions(rm_simulator_world::season::Season::Rm2027);
        assert_eq!(roads.len(), 2);
        assert!(
            roads
                .iter()
                .all(|r| (0..2).all(|i| r.min[i].is_finite() && r.min[i] < r.max[i]))
        );
        assert!(road_exclusions(rm_simulator_world::season::Season::Rm2026).is_empty());
    }
    #[test]
    fn failed_approach_is_remembered_and_recovery_is_bounded() {
        let mut service = Service::new(
            StaticGeometry::default(),
            None,
            0.,
            [-3., -3., 3., 3.],
            vec![],
        )
        .unwrap();
        service.add(1);
        service.goal(1, [2., 0.]).unwrap();
        let a = service.agents.get_mut(&1).unwrap();
        for attempt in 1..=3 {
            a.replan([0., 0., 0.], [2., 0., 0.], true);
            assert_eq!(a.retries, attempt);
            assert_eq!(a.view.phase, Phase::Planning);
            assert!(a.pending);
        }
        assert!(avoids([0., 0., 0.], [0., 0., 0.], &a.failed));
        assert!(!avoids([0., 0., 0.], [2., 0., 0.], &a.failed));
        let geometry = StaticGeometry::default();
        let map = Map::bake(
            geometry.clone(),
            Surface {
                ground: None,
                floor: 0.,
                exclusions: vec![],
            },
            [-3., -3., 3., 3.],
        );
        let path = map
            .plan([0., 0., 0.], [2., 0.], &a.failed, &geometry)
            .unwrap();
        assert!(path.len() >= 3);
        assert!(path.windows(2).all(|p| avoids(p[0], p[1], &a.failed)));
        a.replan([0., 0., 0.], [2., 0., 0.], true);
        assert_eq!(a.view.phase, Phase::Blocked);
        assert_eq!(a.failed.len(), 3);
        service.goal(1, [2., 0.]).unwrap();
        assert_eq!(service.agents[&1].retries, 0);
        assert_eq!(service.agents[&1].failed.len(), 3);
    }
    #[test]
    fn support_gap_is_not_bridged_by_a_shortcut() {
        use crate::collision_mesh::CollisionMesh;
        let mesh = CollisionMesh {
            vertices_m: vec![
                [-3., -2., 0.],
                [-0.3, -2., 0.],
                [-0.3, 2., 0.],
                [-3., 2., 0.],
                [0.3, -2., 0.],
                [3., -2., 0.],
                [3., 2., 0.],
                [0.3, 2., 0.],
            ],
            triangles: vec![[0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7]],
        };
        let map = Map::bake(
            StaticGeometry::default(),
            Surface {
                ground: Some(Arc::new(mesh.into())),
                floor: 0.,
                exclusions: vec![],
            },
            [-2.5, -1.5, 2.5, 1.5],
        );
        assert!(
            map.plan([-2., 0., 0.], [2., 0.], &[], &StaticGeometry::default())
                .is_err()
        );
    }
    #[test]
    fn follower_drives_actual_chassis_and_replans_for_new_obstacles() {
        let mut field = Field::new(&FieldConfig {
            runes: vec![],
            outposts: vec![],
            ..Default::default()
        })
        .unwrap();
        let geometry = wall();
        let (vertices, triangles) = geometry.clone().into_triangles();
        field.add_static_mesh(vertices, triangles).unwrap();
        let config = ChassisConfig::default();
        let id = field
            .add_chassis(&ChassisPlacement {
                config: config.clone(),
                spawn: Pose::at([-2., 0., config.rest_height_m()]),
                team: Team::Red,
                kind: RobotKind::Sentry,
                performance: None,
            })
            .unwrap();
        field.step(128).unwrap();
        let mut service = Service::new(geometry, None, 0., [-3., -3., 3., 3.], vec![]).unwrap();
        service.add(id);
        service.goal(id, [2., 0.]).unwrap();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(15);
        loop {
            service.poll(&field);
            if service.views()[0].phase == Phase::Following {
                break;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "{:?}",
                service.views()
            );
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        for _ in 0..(25_000_000_000 / tick_ns()) {
            service.tick(&mut field);
            field.step(1).unwrap();
            if service.views()[0].phase != Phase::Following {
                break;
            }
        }
        let view = &service.views()[0];
        let body = field.chassis_snapshot(id).unwrap();
        assert_eq!(
            view.phase,
            Phase::Arrived,
            "{view:?}, body={:?}",
            body.pose.translation_m
        );
        assert!(distance(body.pose.translation_m, [2., 0., 0.]) < 0.22);
        service.goal(id, [-2., 0.]).unwrap();
        loop {
            service.poll(&field);
            if service.views()[0].phase == Phase::Following {
                break;
            }
            assert!(std::time::Instant::now() < deadline);
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        let p = field.chassis_snapshot(id).unwrap().pose.translation_m;
        let other = field
            .add_chassis(&ChassisPlacement {
                config: config.clone(),
                spawn: Pose::at([p[0] - 0.9, p[1], config.rest_height_m()]),
                team: Team::Blue,
                kind: RobotKind::Infantry,
                performance: None,
            })
            .unwrap();
        for _ in 0..16 {
            service.tick(&mut field);
            field.step(1).unwrap();
        }
        assert_eq!(service.views()[0].phase, Phase::Planning);
        assert_eq!(field.chassis_snapshot(id).unwrap().command.forward_m_s, 0.);
        assert!(field.chassis_snapshot(other).is_some());
        loop {
            service.poll(&field);
            if service.views()[0].phase != Phase::Planning {
                break;
            }
            assert!(std::time::Instant::now() < deadline);
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        assert_eq!(
            service.views()[0].phase,
            Phase::Following,
            "{:?}",
            service.views()
        );
        for _ in 0..(35_000_000_000 / tick_ns()) {
            service.tick(&mut field);
            field.step(1).unwrap();
            while service.views()[0].phase == Phase::Planning {
                service.poll(&field);
                assert!(std::time::Instant::now() < deadline);
                std::thread::sleep(std::time::Duration::from_millis(2));
            }
            if service.views()[0].phase != Phase::Following {
                break;
            }
        }
        assert_eq!(
            service.views()[0].phase,
            Phase::Arrived,
            "{:?}",
            service.views()
        );
    }
}
