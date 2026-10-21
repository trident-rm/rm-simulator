// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Conservative Sentry navigation prototype: shared CAD bake, bounded A* and chassis input.
//! This is host-side control, not a new physics world or a firing/targeting policy.
use crate::collision_mesh::GroundMesh;
use rm_simulator_world::{ChassisCommand, ChassisConfig, Field, StaticGeometry};
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
const TUNNEL_GRADE: f64 = 0.287; // tan(16 degrees), admits the UC §4.4 15-degree approach.
const GRADE: f64 = 0.213; // tan(12 degrees), conservative omni terrain limit.
const SPEED_M_S: f64 = 1.0;
const TUNNEL_SPEED_M_S: f64 = 0.85;
const PLANNING_RADIUS_M: f64 = RADIUS_M + 0.12;
pub(crate) fn brake(field: &mut Field, id: u32) -> Result<(), String> {
    let mut command = field.chassis_snapshot(id).ok_or("missing Sentry")?.command;
    command.forward_m_s = 0.;
    command.left_m_s = 0.;
    command.yaw_rate_rad_s = 0.;
    field
        .command_chassis(id, command)
        .map_err(|e| e.to_string())
}
pub(crate) fn rotation_clear(field: &Field, id: u32, geometry: &StaticGeometry) -> bool {
    let Some(body) = field.chassis_snapshot(id) else {
        return false;
    };
    let p = body.pose.translation_m;
    let radius = body.config.body_half_m[0]
        .hypot(body.config.body_half_m[1])
        .max(PLANNING_RADIUS_M);
    // Ground support is below the center; preserve a small margin above the deck.
    geometry.cylinder_sweep_clear(
        [p[0], p[1], p[2] + 0.32],
        [p[0], p[1], p[2] + 0.32],
        radius,
        0.3,
    )
}
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
// Fitted EGA loaded envelope is 245.5 mm high; retain 2.5 mm CAD clearance.
const STOW_TOP_M: f64 = 0.248;
// Source body +x is the resting barrel, not necessarily chassis forward.
// The fixed body's source-axis collision box supplies that frame: align it
// with the passage so the real wheel arrangement approaches the ramp in X.
// An omni robot may strafe through a narrow exit connector. Hold the longest
// segment's heading throughout a contiguous stowed section; rotating to face a
// short lateral waypoint can wedge an otherwise clear exit.
fn passage_heading(path: &[[f64; 3]], tunnels: &[bool], waypoint: usize) -> f64 {
    let mut first = waypoint;
    while first > 1 && tunnels[first - 1] {
        first -= 1;
    }
    let mut last = waypoint;
    while tunnels[waypoint] && last + 1 < path.len() && tunnels[last + 1] {
        last += 1;
    }
    let edge = (first..=last)
        .filter(|&i| tunnels[i])
        .max_by(|&a, &b| distance(path[a - 1], path[a]).total_cmp(&distance(path[b - 1], path[b])))
        .unwrap_or(waypoint);
    (path[edge][1] - path[edge - 1][1]).atan2(path[edge][0] - path[edge - 1][0])
}
fn tunnel_heading_offset(config: &ChassisConfig) -> f64 {
    config
        .articulation
        .as_ref()
        .filter(|_| tunnel_capable(config))
        .and_then(|a| a.joints.iter().find(|j| j.name == "source-body"))
        .and_then(|j| j.collision.as_ref())
        .map_or(0., |c| {
            -rm_simulator_world::chassis::yaw_of(rm_simulator_world::Pose {
                rotation_wxyz: c.rotation_wxyz,
                ..Default::default()
            })
        })
}
pub(crate) fn tunnel_capable(config: &ChassisConfig) -> bool {
    config
        .articulation
        .as_ref()
        .is_some_and(|a| a.profile == "ega-tunnel-sentry-v1")
}
fn clearance(
    surface: &Surface,
    geometry: &StaticGeometry,
    a: [f64; 3],
    b: [f64; 3],
    tunnel: bool,
    radius: f64,
) -> bool {
    if tunnel {
        let heading = (b[1] - a[1]).atan2(b[0] - a[0]);
        stowed_clear(surface, geometry, a, b, heading)
    } else {
        geometry.cylinder_sweep_clear(center(a), center(b), radius, 0.24)
    }
}
// EGA source body/wheels fit a 670 mm square; retain 15 mm per side.
// Like the ordinary cylinder, this tests the upper-body/headroom band. Floor
// support tracks handle lower terrain and curbs; live source colliders remain authoritative.
fn stowed_clear(
    surface: &Surface,
    geometry: &StaticGeometry,
    a: [f64; 3],
    b: [f64; 3],
    yaw: f64,
) -> bool {
    // The crowned CAD floor and its tunnel roofs share a slope. A world-upright
    // box falsely loses ~9 mm of headroom across a 700 mm footprint at 1.5°.
    // Follow the support plane, as the real independently suspended chassis does.
    let n = (distance(a, b) / 0.10).ceil().max(1.) as usize;
    let pose = |p: [f64; 3]| {
        let z = surface.height(p[0], p[1], p[2] + 0.17)?;
        if (z - p[2]).abs() > 0.025 {
            return None;
        }
        let h = |dx, dy| surface.height(p[0] + dx, p[1] + dy, z + 0.17);
        let gx = (h(0.35, 0.)? - h(-0.35, 0.)?) / 0.70;
        let gy = (h(0., 0.35)? - h(0., -0.35)?) / 0.70;
        if gx.hypot(gy) > TUNNEL_GRADE + 0.01 {
            return None;
        }
        let up = glam::DVec3::new(-gx, -gy, 1.).normalize();
        let forward =
            glam::DVec3::new(yaw.cos(), yaw.sin(), gx * yaw.cos() + gy * yaw.sin()).normalize();
        let left = up.cross(forward).normalize();
        let q = glam::DQuat::from_mat3(&glam::DMat3::from_cols(forward, left, up)).to_array();
        let center = glam::DVec3::new(p[0], p[1], z) + up * ((0.14 + STOW_TOP_M) / 2.);
        Some(rm_simulator_world::Pose {
            translation_m: center.to_array(),
            rotation_wxyz: [q[3], q[0], q[1], q[2]],
        })
    };
    let mut previous = match pose(a) {
        Some(p) => p,
        None => return false,
    };
    for i in 1..=n {
        let t = i as f64 / n as f64;
        let p = std::array::from_fn(|j| a[j] + t * (b[j] - a[j]));
        let Some(next) = pose(p) else {
            return false;
        };
        let half = [0.35, 0.35, (STOW_TOP_M - 0.14) / 2.];
        if !geometry.cuboid_sweep_clear(previous, next.translation_m, half)
            || !geometry.cuboid_sweep_clear(next, previous.translation_m, half)
        {
            return false;
        }
        previous = next;
    }
    true
}
// The Road uses Figure 5-23; the two adjacent Highland lanes use surveyed CAD
// hints (runtime RMUC2027_V1.0.0.stp, arena FLU): X ±1.45 m, Y 5.93/6.93 m.
// Passage staging seeds remain separate from the calibrated RFID antenna pads.
// These are seeds, not geometry: fitted paths require complete
// source support/headroom validation. Off-grid sampling avoids missing an 800 mm
// opening with a 700 mm robot on a 250 mm navigation grid.
fn tunnel_seeds() -> Vec<[[f64; 3]; 2]> {
    let mut seeds = vec![
        [[4.695806, 4.666273, 0.], [4.695806, 5.936208, 0.]],
        [[-4.695806, -4.666273, 0.], [-4.695806, -5.936208, 0.]],
    ];
    for sign in [1., -1.] {
        for y in [5.93, 6.93] {
            seeds.push([[sign * -1.45, sign * y, 0.], [sign * 1.45, sign * y, 0.]]);
        }
    }
    seeds
}
fn footprint(tunnel: bool, yaw: f64) -> [[f64; 2]; 9] {
    std::array::from_fn(|i| {
        if i == 8 {
            return [0.; 2];
        }
        let a = i as f64 * std::f64::consts::FRAC_PI_4;
        let r = if tunnel {
            if i % 2 == 0 {
                0.35
            } else {
                0.35 * 2_f64.sqrt()
            }
        } else {
            PLANNING_RADIUS_M
        };
        [r * (a + yaw).cos(), r * (a + yaw).sin()]
    })
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
    /// Undirected low-clearance graph edges requiring explicit stowed traversal.
    pub tunnel_links: usize,
    /// CAD-validated off-grid passages fitted near six Road/Highland survey hints.
    pub passages: usize,
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
    /// For each waypoint, whether the incoming segment requires aligned stowed travel.
    /// Empty while stopped/planning; otherwise the same length as `path_m`, first false.
    pub tunnel_segments: Vec<bool>,
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
            && self.tunnel_segments.len() == self.path_m.len()
            && !self.tunnel_segments.first().copied().unwrap_or(false)
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
    #[cfg(test)]
    fn crosses(&self, a: [f64; 3], b: [f64; 3]) -> bool {
        self.crosses_with(a, b, PLANNING_RADIUS_M)
    }
    fn crosses_with(&self, a: [f64; 3], b: [f64; 3], padding: f64) -> bool {
        let mut enter: f64 = 0.;
        let mut leave: f64 = 1.;
        for i in 0..2 {
            let lo = self.min[i] - padding;
            let hi = self.max[i] + padding;
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
pub(crate) fn road_exclusions() -> Vec<Exclusion> {
    use rm_simulator_world::Team;
    let areas = crate::zone_catalog::areas();
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
            |g| g.support_height_below(x, y, max),
        )
    }
    fn supported(&self, p: [f64; 3]) -> bool {
        self.supported_with(p, false)
    }
    fn supported_with(&self, p: [f64; 3], tunnel: bool) -> bool {
        self.supported_heading(p, tunnel, 0.)
    }
    fn supported_heading(&self, p: [f64; 3], tunnel: bool, yaw: f64) -> bool {
        let above = if tunnel { 0.17 } else { 0.30 };
        let grade = if tunnel { TUNNEL_GRADE } else { GRADE };
        if self
            .exclusions
            .iter()
            .any(|r| r.crosses_with(p, p, if tunnel { 0.35 } else { PLANNING_RADIUS_M }))
        {
            return false;
        }
        let Some(center_z) = self.height(p[0], p[1], p[2] + above) else {
            return false;
        };
        if (center_z - p[2]).abs() > 0.025 {
            return false;
        }
        // Edge-direction slope alone lets A* zigzag across a surface that is
        // too steep for the vehicle. Check the native support face's full
        // gradient, including when travelling approximately along a contour.
        if self
            .ground
            .as_ref()
            .and_then(|g| g.support_grade_below(p[0], p[1], p[2] + above))
            .is_some_and(|actual| actual > grade + 0.001)
        {
            return false;
        }
        // Sample the inflated footprint, including wheel diagonals. Reject a
        // height discontinuity under the bumper even if the centre is smooth.
        footprint(tunnel, yaw).iter().all(|&[x, y]| {
            self.height(p[0] + x, p[1] + y, center_z + above)
                .is_some_and(|z| (z - center_z).abs() <= 0.012 + PLANNING_RADIUS_M * grade)
        })
    }
}
struct Map {
    nodes: Vec<[f64; 3]>,
    low: Vec<bool>,
    edges: Vec<Vec<(usize, u32, bool)>>,
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
        self.segment_clear(a, b, false)
    }
    fn segment_clear(&self, a: [f64; 3], b: [f64; 3], tunnel: bool) -> bool {
        let length = distance(a, b);
        let grade = if tunnel { TUNNEL_GRADE } else { GRADE };
        let heading = (b[1] - a[1]).atan2(b[0] - a[0]);
        let padding = if tunnel {
            0.35 * (heading.cos().abs() + heading.sin().abs())
        } else {
            PLANNING_RADIUS_M
        };
        if self
            .surface
            .exclusions
            .iter()
            .any(|r| r.crosses_with(a, b, padding))
            || (a[2] - b[2]).abs() > 0.015 + length * grade
        {
            return false;
        }
        let n = (length / 0.03).ceil().max(1.) as usize;
        let mut previous = [None::<f64>; 9];
        for i in 0..=n {
            let t = i as f64 / n as f64;
            let p = std::array::from_fn(|j| a[j] + t * (b[j] - a[j]));
            if !self.surface.supported_heading(p, tunnel, heading) {
                return false;
            }
            for ([x, y], old) in footprint(tunnel, heading).into_iter().zip(&mut previous) {
                let Some(height) = self.surface.height(
                    p[0] + x,
                    p[1] + y,
                    p[2] + if tunnel { 0.17 } else { 0.30 },
                ) else {
                    return false;
                };
                // Four mm allows CAD sheet/seam noise; curbs under any part of
                // the footprint must not disappear between coarse grid nodes.
                if old.is_some_and(|z| (height - z).abs() > grade * length / n as f64 + 0.004) {
                    return false;
                }
                *old = Some(height);
            }
        }
        clearance(
            &self.surface,
            &self.geometry,
            a,
            b,
            tunnel,
            PLANNING_RADIUS_M,
        )
    }

    fn bake(geometry: StaticGeometry, surface: Surface, bounds: [f64; 4]) -> Self {
        let nx = ((bounds[2] - bounds[0]) / CELL_M).floor() as usize + 1;
        let ny = ((bounds[3] - bounds[1]) / CELL_M).floor() as usize + 1;
        let mut columns = vec![Vec::<usize>::new(); nx * ny];
        let mut map = Self {
            nodes: vec![],
            low: vec![],
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
                for max in [-0.25, -0.10, 0., 0.10, 0.35, 0.65, 1.05, 1.45] {
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
                    let high = map.surface.supported(p)
                        && clearance(&map.surface, &map.geometry, p, p, false, PLANNING_RADIUS_M);
                    let low = !high
                        && map.surface.supported_with(p, true)
                        && (0..4).any(|i| {
                            stowed_clear(
                                &map.surface,
                                &map.geometry,
                                p,
                                p,
                                i as f64 * std::f64::consts::FRAC_PI_4,
                            )
                        });
                    if high || low {
                        columns[y * nx + x].push(map.nodes.len());
                        map.nodes.push(p);
                        map.low.push(low);
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
                            let tunnel = map.low[a] || map.low[b];
                            if map.segment_clear(map.nodes[a], map.nodes[b], tunnel) {
                                let cost = (distance(map.nodes[a], map.nodes[b]) * 1000.
                                    + (map.nodes[a][2] - map.nodes[b][2]).abs() * 500.)
                                    .ceil() as u32;
                                let cost = cost * if tunnel { 2 } else { 1 }
                                    + if map.low[a] != map.low[b] { 900 } else { 0 };
                                map.edges[a].push((b, cost, tunnel));
                                map.edges[b].push((a, cost, tunnel));
                                if tunnel {
                                    map.summary.tunnel_links += 1;
                                }
                            }
                        }
                    }
                }
            }
        }
        map.add_passages(&tunnel_seeds());
        map.summary.nodes = map.nodes.len();
        map.summary.edges = map.edges.iter().map(Vec::len).sum();
        map
    }
    fn add_passages(&mut self, seeds: &[[[f64; 3]; 2]]) {
        for &[a, b] in seeds {
            let length = distance(a, b);
            if length < 0.1 {
                continue;
            }
            let d = [(b[0] - a[0]) / length, (b[1] - a[1]) / length];
            let mut accepted = None;
            // The Road's sloped entry needs its full run-up for wheel alignment.
            // Flat Highland lanes can use shorter collision-checked connectors.
            let extensions: &[f64] = if d[1].abs() > 0.9 {
                &[0.8, 1.05, 1.3]
            } else {
                &[0.35, 0.5, 0.65, 0.8, 1.05, 1.3]
            };
            'candidate: for &extension in extensions {
                for offset in [
                    0., -0.05, 0.05, -0.1, 0.1, -0.15, 0.15, -0.2, 0.2, -0.25, 0.25,
                ] {
                    let mut ends = [a, b];
                    for (i, p) in ends.iter_mut().enumerate() {
                        let sign = if i == 0 { -1. } else { 1. };
                        p[0] += sign * extension * d[0] - offset * d[1];
                        p[1] += sign * extension * d[1] + offset * d[0];
                        p[2] = self.surface.height(p[0], p[1], 0.15).unwrap_or(f64::NAN);
                    }
                    if !ends.iter().flatten().all(|v| v.is_finite())
                        || ends.iter().any(|&p| !self.segment_clear(p, p, true))
                    {
                        continue;
                    }
                    let n = (distance(ends[0], ends[1]) / 0.10).ceil() as usize;
                    let mut path = vec![ends[0]];
                    for i in 1..=n {
                        let t = i as f64 / n as f64;
                        let x = ends[0][0] + t * (ends[1][0] - ends[0][0]);
                        let y = ends[0][1] + t * (ends[1][1] - ends[0][1]);
                        let last = *path.last().unwrap();
                        let Some(z) = self.surface.height(x, y, last[2] + 0.10) else {
                            path.clear();
                            break;
                        };
                        let p = [x, y, z];
                        if !self.segment_clear(last, p, true) {
                            path.clear();
                            break;
                        }
                        path.push(p);
                    }
                    if path.is_empty()
                        || (path.last().unwrap()[2] - ends[1][2]).abs() > 0.025
                        || path.iter().all(|&p| {
                            clearance(
                                &self.surface,
                                &self.geometry,
                                p,
                                p,
                                false,
                                PLANNING_RADIUS_M,
                            )
                        })
                    {
                        continue;
                    }
                    // Prefer a fitted off-grid staging point over a grid node
                    // near a wall. If the endpoint is still narrow, keep stow
                    // through its connector to a wider staging node instead.
                    let staging = ends.map(|p| {
                        self.line_clear(p, p)
                            && clearance(
                                &self.surface,
                                &self.geometry,
                                p,
                                p,
                                false,
                                PLANNING_RADIUS_M + 0.06,
                            )
                    });
                    let connections: Vec<Vec<usize>> = ends
                        .iter()
                        .enumerate()
                        .map(|(side, &p)| {
                            self.nodes
                                .iter()
                                .enumerate()
                                .filter(|(i, q)| {
                                    !self.low[*i]
                                        && distance(p, **q) < 0.75
                                        && if staging[side] {
                                            self.line_clear(p, **q)
                                        } else {
                                            clearance(
                                                &self.surface,
                                                &self.geometry,
                                                **q,
                                                **q,
                                                false,
                                                PLANNING_RADIUS_M + 0.06,
                                            ) && self.segment_clear(p, **q, true)
                                        }
                                })
                                .map(|(i, _)| i)
                                .collect()
                        })
                        .collect();
                    if connections.iter().any(Vec::is_empty) {
                        continue;
                    }
                    accepted = Some((path, connections, staging));
                    break 'candidate;
                }
            }
            if let Some((path, connections, staging)) = accepted {
                let first = self.nodes.len();
                let last = first + path.len() - 1;
                for (i, p) in path.into_iter().enumerate() {
                    let id = self.nodes.len();
                    self.nodes.push(p);
                    self.low.push(if id == first {
                        !staging[0]
                    } else if id == last {
                        !staging[1]
                    } else {
                        true
                    });
                    self.edges.push(vec![]);
                    if i > 0 {
                        let cost = (distance(self.nodes[id - 1], p) * 2000.).ceil() as u32
                            + if id == last || i == 1 { 900 } else { 0 };
                        self.edges[id - 1].push((id, cost, true));
                        self.edges[id].push((id - 1, cost, true));
                        self.summary.tunnel_links += 1;
                    }
                }
                for ((id, neighbors), stage) in
                    [first, last].into_iter().zip(connections).zip(staging)
                {
                    for n in neighbors {
                        let cost = (distance(self.nodes[id], self.nodes[n]) * 1000.).ceil() as u32;
                        let cost = if stage { cost } else { cost * 2 + 900 };
                        self.edges[id].push((n, cost, !stage));
                        self.edges[n].push((id, cost, !stage));
                        if !stage {
                            self.summary.tunnel_links += 1;
                        }
                    }
                }
                self.summary.passages += 1;
            }
        }
    }
    fn plan(
        &self,
        start: [f64; 3],
        goal: [f64; 2],
        dynamic: &[Obstacle],
        live: &StaticGeometry,
        allow_tunnel: bool,
    ) -> Result<Route, String> {
        let clear = |a, b| {
            self.line_clear(a, b)
                && avoids(a, b, dynamic)
                && live.cylinder_sweep_clear(center(a), center(b), PLANNING_RADIUS_M, 0.24)
        };
        let normal_begin = self
            .nodes
            .iter()
            .enumerate()
            .filter(|(_, p)| distance(start, **p) <= 0.65 && clear(start, **p))
            .min_by(|a, b| distance(start, *a.1).total_cmp(&distance(start, *b.1)))
            .map(|(i, _)| (i, false));
        let (begin, start_tunnel) = normal_begin
            .or_else(|| {
                allow_tunnel
                    .then(|| {
                        self.nodes
                            .iter()
                            .enumerate()
                            .filter(|(_, p)| {
                                distance(start, **p) <= 0.65
                                    && self.segment_clear(start, **p, true)
                                    && avoids(start, **p, dynamic)
                                    && clearance(
                                        &self.surface,
                                        live,
                                        start,
                                        **p,
                                        true,
                                        PLANNING_RADIUS_M,
                                    )
                            })
                            .min_by(|a, b| distance(start, *a.1).total_cmp(&distance(start, *b.1)))
                            .map(|(i, _)| (i, true))
                    })
                    .flatten()
            })
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
        let mut incoming = vec![false; self.nodes.len()];
        incoming[begin] = start_tunnel;
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
            for &(b, w, tunnel) in &self.edges[a] {
                if tunnel && !allow_tunnel {
                    continue;
                }
                let next = cost.saturating_add(w);
                if next < costs[b]
                    && avoids(self.nodes[a], self.nodes[b], dynamic)
                    && clearance(
                        &self.surface,
                        live,
                        self.nodes[a],
                        self.nodes[b],
                        tunnel,
                        PLANNING_RADIUS_M,
                    )
                {
                    costs[b] = next;
                    parent[b] = a;
                    incoming[b] = tunnel;
                    heap.push((Reverse(next.saturating_add(heuristic(b))), Reverse(b), next));
                }
            }
        }
        let mut at = end.ok_or("no route connects the start and goal")?;
        let finish = goals[&at];
        let mut raw = vec![(finish, false)];
        loop {
            raw.push((self.nodes[at], incoming[at]));
            if at == begin {
                break;
            }
            at = parent[at];
        }
        raw.push((start, false));
        raw.reverse();
        let mut result = Route {
            points: vec![start],
            tunnels: vec![false],
        };
        let mut cursor = 0;
        while cursor + 1 < raw.len() {
            let tunnel = raw[cursor + 1].1;
            let mut next = cursor + 1;
            while next + 1 < raw.len() && raw[next + 1].1 == tunnel {
                next += 1;
            }
            while next > cursor + 1
                && !(self.segment_clear(raw[cursor].0, raw[next].0, tunnel)
                    && avoids(raw[cursor].0, raw[next].0, dynamic)
                    && clearance(
                        &self.surface,
                        live,
                        raw[cursor].0,
                        raw[next].0,
                        tunnel,
                        PLANNING_RADIUS_M,
                    ))
            {
                next -= 1;
            }
            result.points.push(raw[next].0);
            result.tunnels.push(raw[next].1);
            cursor = next;
        }
        if result.points.len() > 256 {
            return Err("route exceeds prototype waypoint budget".into());
        }
        Ok(result)
    }
}
#[derive(Debug)]
struct Route {
    points: Vec<[f64; 3]>,
    tunnels: Vec<bool>,
}
struct Job {
    id: u32,
    generation: u64,
    start: [f64; 3],
    goal: [f64; 2],
    obstacles: Vec<Obstacle>,
    live: StaticGeometry,
    allow_tunnel: bool,
}
enum Reply {
    Baked(Bake),
    Route(u32, u64, Result<Route, String>),
}
struct Agent {
    view: View,
    generation: u64,
    pending: bool,
    last: [f64; 3],
    stalled: u64,
    retries: u8,
    failed: Vec<Obstacle>,
    // Retain the passage frame until a full-height rotation envelope is clear.
    passage_yaw: Option<f64>,
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
        self.view.tunnel_segments.clear();
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
                    let path = map.plan(
                        job.start,
                        job.goal,
                        &job.obstacles,
                        &job.live,
                        job.allow_tunnel,
                    );
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
                    tunnel_segments: vec![],
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
                passage_yaw: None,
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
            a.view.tunnel_segments.clear();
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
        a.view.tunnel_segments.clear();
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
                        Ok(route) => {
                            let path = route.points;
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
                                a.view.tunnel_segments = route.tunnels;
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
                allow_tunnel: field.chassis_config(id).is_some_and(tunnel_capable),
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
            let mut command = ChassisCommand {
                folded: body.command.folded,
                aim_yaw_rad: body.command.aim_yaw_rad,
                aim_pitch_rad: body.command.aim_pitch_rad,
                ..Default::default()
            };
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
                    command.folded = body.command.folded
                        && !clearance(
                            &self.surface,
                            &live,
                            [p[0], p[1], last[2]],
                            [p[0], p[1], last[2]],
                            false,
                            PLANNING_RADIUS_M,
                        );
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
                    let tunnel = a.view.tunnel_segments[a.view.waypoint]
                        || (body.command.folded
                            && !clearance(
                                &self.surface,
                                &live,
                                from,
                                from,
                                false,
                                PLANNING_RADIUS_M,
                            ));
                    let yaw = rm_simulator_world::chassis::yaw_of(body.pose);
                    let entry = a.view.path_m[a.view.waypoint - 1];
                    let offset = tunnel_heading_offset(field.chassis_config(id).unwrap());
                    if !body.command.folded && body.fold_angle_rad < 0.001 {
                        a.passage_yaw = None;
                    }
                    let heading = if tunnel {
                        *a.passage_yaw.get_or_insert_with(|| {
                            passage_heading(
                                &a.view.path_m,
                                &a.view.tunnel_segments,
                                a.view.waypoint,
                            ) + offset
                        })
                    } else {
                        a.passage_yaw = None;
                        (waypoint[1] - entry[1]).atan2(waypoint[0] - entry[0]) + offset
                    };
                    let error = (heading - yaw).sin().atan2((heading - yaw).cos());
                    let limit = field.chassis_config(id).unwrap().transform_limit_rad();
                    let ready = !tunnel
                        || ((body.fold_angle_rad - limit).abs() < 0.001 && error.abs() < 0.04);
                    command.folded = tunnel;
                    if tunnel {
                        command.aim_yaw_rad = yaw;
                        command.aim_pitch_rad =
                            crate::math::rotate(body.pose.rotation_wxyz, [1., 0., 0.])[2]
                                .clamp(-1., 1.)
                                .asin();
                        let yaw_speed = if clearance(
                            &self.surface,
                            &live,
                            from,
                            from,
                            false,
                            PLANNING_RADIUS_M,
                        ) {
                            1.5
                        } else {
                            0.5
                        };
                        command.yaw_rate_rad_s = (error * 4.).clamp(-yaw_speed, yaw_speed);
                        let target = yaw + (error * 0.25).clamp(-0.05, 0.05);
                        if (0..=4).any(|i| {
                            !stowed_clear(
                                &self.surface,
                                &live,
                                from,
                                from,
                                yaw - offset + (target - yaw) * i as f64 / 4.,
                            )
                        }) {
                            command.yaw_rate_rad_s = 0.;
                        }
                        a.view.message = if ready {
                            "Traversing tunnel (stowed)"
                        } else {
                            "Stowing and aligning before tunnel"
                        }
                        .into();
                    } else {
                        a.view.message = "Following collision-checked path".into();
                    }
                    let length = distance(entry, waypoint).max(0.001);
                    let u = [
                        (waypoint[0] - entry[0]) / length,
                        (waypoint[1] - entry[1]) / length,
                    ];
                    let along =
                        ((p[0] - entry[0]) * u[0] + (p[1] - entry[1]) * u[1]).clamp(0., length);
                    let line = [entry[0] + u[0] * along, entry[1] + u[1] * along, floor];
                    if !ready {
                        // Evaluate travel only after fold/alignment, retaining
                        // bounded staging correction below. A future sweep at
                        // an unaligned heading must not consume replans.
                        // Counter suspension/tyre drift on the crowned floor.
                        // Keep the checked staging position while folding and
                        // aligning, using only a clear full-height correction.
                        if distance(from, entry) > 0.01 {
                            let target = [entry[0], entry[1], floor];
                            if clearance(&self.surface, &live, from, target, false, RADIUS_M)
                                && avoids(from, target, &obstacles(field, id))
                            {
                                let vx = ((entry[0] - p[0]) * 2.).clamp(-0.12, 0.12);
                                let vy = ((entry[1] - p[1]) * 2.).clamp(-0.12, 0.12);
                                command.forward_m_s = vx * yaw.cos() + vy * yaw.sin();
                                command.left_m_s = -vx * yaw.sin() + vy * yaw.cos();
                            }
                        }
                        a.stalled += 8;
                        if a.stalled > 1280 {
                            a.view.phase = Phase::Blocked;
                            a.view.message = "Tunnel preparation failed; wheels stopped".into();
                            command.yaw_rate_rad_s = 0.;
                        }
                    } else if !avoids(from, to, &obstacles(field, id))
                        || !(if tunnel {
                            stowed_clear(&self.surface, &live, from, to, yaw - offset)
                        } else {
                            clearance(&self.surface, &live, from, to, false, RADIUS_M)
                        })
                    {
                        if tunnel
                            && ready
                            && distance(from, line) > 0.003
                            && stowed_clear(&self.surface, &live, from, line, yaw - offset)
                            && avoids(from, line, &obstacles(field, id))
                            && a.stalled < 384
                        {
                            a.view.message = "Centering before narrow tunnel approach".into();
                            let vx = ((line[0] - p[0]) * 4.).clamp(-0.12, 0.12);
                            let vy = ((line[1] - p[1]) * 4.).clamp(-0.12, 0.12);
                            command.forward_m_s = vx * yaw.cos() + vy * yaw.sin();
                            command.left_m_s = -vx * yaw.sin() + vy * yaw.cos();
                            a.stalled += 8;
                        } else {
                            a.replan(from, to, false);
                        }
                    } else {
                        let speed = (if tunnel && (waypoint[2] - entry[2]).abs() / length > 0.08 {
                            0.6
                        } else if tunnel {
                            TUNNEL_SPEED_M_S
                        } else {
                            SPEED_M_S
                        })
                        .min(distance(p, last) * 1.6)
                        .min(d * 2.0);
                        let mut vx = (waypoint[0] - p[0]) / d.max(0.001) * speed;
                        let mut vy = (waypoint[1] - p[1]) / d.max(0.001) * speed;
                        if tunnel {
                            vx = u[0] * speed + ((line[0] - p[0]) * 4.).clamp(-0.12, 0.12);
                            vy = u[1] * speed + ((line[1] - p[1]) * 4.).clamp(-0.12, 0.12);
                        }
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
                            command.forward_m_s = 0.;
                            command.left_m_s = 0.;
                            command.yaw_rate_rad_s = 0.;
                        }
                    }
                }
            } else if body.defeated && a.view.phase == Phase::Following {
                a.view.phase = Phase::Blocked;
                a.view.message = "Sentry defeated; stopped".into();
            }
            if a.view.phase != Phase::Following {
                command.forward_m_s = 0.;
                command.left_m_s = 0.;
                command.yaw_rate_rad_s = 0.;
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
        let path = map
            .plan([-2., 0., 0.], [2., 0.], &[], &geometry, false)
            .unwrap()
            .points;
        assert!(path.len() >= 3);
        assert!(path.iter().any(|p| p[1].abs() > 1.25 + RADIUS_M));
        assert!(path.windows(2).all(|p| map.line_clear(p[0], p[1])));
        assert!(
            map.plan([-2., 0., 0.], [0., 0.], &[], &geometry, false)
                .is_err()
        );
        let blocked = vec![Obstacle {
            point: [2., 0., 0.2],
            radius: 0.5,
        }];
        assert!(
            map.plan([-2., 0., 0.], [2., 0.], &blocked, &geometry, false)
                .is_err()
        );
    }
    #[test]
    fn tunnel_links_are_explicit_capability_gated_and_survive_shortcuts() {
        let mut world = rm_simulator_world::projectile::WorldPhysics::new(&[], 0.);
        // Roof fully separates the high-clearance graph, but leaves a 250 mm passage.
        world.add_static_box(rm_simulator_world::Pose::at([0., 0., 0.4]), [1., 8., 0.3]);
        let geometry = world.static_geometry_snapshot(true);
        let map = Map::bake(
            geometry.clone(),
            Surface {
                ground: None,
                floor: 0.,
                exclusions: vec![],
            },
            [-3., -2., 3., 2.],
        );
        assert!(
            map.plan([-2.5, 0., 0.], [2.5, 0.], &[], &geometry, false)
                .is_err()
        );
        assert!(map.summary.tunnel_links > 0);
        assert!(
            map.plan([-2.5, 0., 0.], [2.5, 0.], &[], &geometry, false)
                .is_err()
        );
        for (start, end) in [([-2.5, 0., 0.], [2.5, 0.]), ([2.5, 0., 0.], [-2.5, 0.])] {
            let route = map.plan(start, end, &[], &geometry, true).unwrap();
            assert_eq!(route.points.len(), route.tunnels.len());
            assert_eq!(route.tunnels.iter().filter(|&&t| t).count(), 1);
            for i in 1..route.points.len() {
                assert!(map.segment_clear(route.points[i - 1], route.points[i], route.tunnels[i]));
            }
        }
        let mut obstruction = rm_simulator_world::projectile::WorldPhysics::new(&[], 0.);
        obstruction.add_static_box(rm_simulator_world::Pose::at([0., 0., 0.1]), [0.1, 8., 0.2]);
        assert!(
            map.plan(
                [-2.5, 0., 0.],
                [2.5, 0.],
                &[],
                &obstruction.static_geometry_snapshot(true),
                true
            )
            .is_err()
        );
    }
    #[test]
    fn source_collision_frame_aligns_both_front_wheels_for_the_ramp() {
        let mut config = ChassisConfig::default();
        let angle = 32_f64.to_radians();
        config.articulation = Some(
            serde_json::from_value(serde_json::json!({
                "profile":"ega-tunnel-sentry-v1","limit_rad":1.,"speed_rad_s":0.65,
                "muzzle_offset_m":[0.2,0.,0.],"camera_offset_m":[0.1,0.,0.],
                "armor_faces":config.armor_faces(),
                "joints":[{"name":"source-body","parent":null,"pivot_m":[0.,0.,0.],
                    "axis":[0.,0.,1.],"angle_rad":0.,"translation_m":[0.,0.,0.],"drive":"Transform",
                    "collision":{"center_m":[0.,0.,0.],"half_m":[0.292,0.292,0.1],
                        "rotation_wxyz":crate::math::quat_axis_angle([0.,0.,1.],-angle)}}]
            }))
            .unwrap(),
        );
        let offset = tunnel_heading_offset(&config);
        assert!((offset - angle).abs() < 1e-12);
        // Source-fitted wheel 0/3 become the front pair in chassis forward.
        let forward = |x: f64, y: f64| x * offset.cos() - y * offset.sin();
        assert!((forward(0.2636288, 0.0598945) - forward(0.0616634, -0.2633672)).abs() < 0.002);
        config.articulation.as_mut().unwrap().profile = "other-source".into();
        assert_eq!(tunnel_heading_offset(&config), 0.);
    }
    #[test]
    fn stowed_section_keeps_its_heading_on_short_lateral_exit_connectors() {
        let path = [
            [3., 0., 0.],
            [2., 0., 0.],
            [-2., 0., 0.],
            [-2., -0.2, 0.],
            [-3., -0.2, 0.],
        ];
        let tunnels = [false, false, true, true, false];
        assert_eq!(passage_heading(&path, &tunnels, 2), std::f64::consts::PI);
        assert_eq!(passage_heading(&path, &tunnels, 3), std::f64::consts::PI);
    }
    #[test]
    fn stowed_sweep_follows_crowned_support_but_rejects_a_lower_roof() {
        use crate::collision_mesh::CollisionMesh;
        let slope: f64 = 0.03;
        let surface = Surface {
            ground: Some(Arc::new(
                CollisionMesh {
                    vertices_m: vec![
                        [-3., -3., -3. * slope],
                        [3., -3., -3. * slope],
                        [3., 3., 3. * slope],
                        [-3., 3., 3. * slope],
                    ],
                    triangles: vec![[0, 1, 2], [0, 2, 3]],
                }
                .into(),
            )),
            floor: 0.,
            exclusions: vec![],
        };
        let rotation = crate::math::quat_axis_angle([1., 0., 0.], slope.atan());
        let up = crate::math::rotate(rotation, [0., 0., 1.]);
        for (height, expected) in [(0.25, true), (0.24, false)] {
            let mut world = rm_simulator_world::projectile::WorldPhysics::new(&[], 0.);
            world.add_static_box(
                Pose {
                    translation_m: up.map(|v| v * (height + 0.15)),
                    rotation_wxyz: rotation,
                },
                [2., 4., 0.3],
            );
            let geometry = world.static_geometry_snapshot(true);
            assert_eq!(
                stowed_clear(&surface, &geometry, [-0.5, 0., 0.], [0.5, 0., 0.], 0.),
                expected
            );
            assert_eq!(
                stowed_clear(
                    &surface,
                    &geometry,
                    [0.5, 0., 0.],
                    [-0.5, 0., 0.],
                    std::f64::consts::PI
                ),
                expected
            );
            if expected {
                assert!(!geometry.cuboid_sweep_clear(
                    Pose::at([0., 0., (0.14 + STOW_TOP_M) / 2.]),
                    [0., 0., (0.14 + STOW_TOP_M) / 2.],
                    [0.35, 0.35, (STOW_TOP_M - 0.14) / 2.]
                ));
            }
        }
    }
    #[test]
    fn tunnel_hints_cover_the_adjacent_highland_lanes_and_their_mirrors() {
        let seeds = tunnel_seeds();
        assert_eq!(seeds.len(), 6);
        for sign in [1., -1.] {
            for y in [5.93, 6.93] {
                assert!(
                    seeds.contains(&[[sign * -1.45, sign * y, 0.], [sign * 1.45, sign * y, 0.]])
                );
            }
        }
    }
    #[test]
    fn off_grid_passage_fitting_connects_an_opening_the_coarse_grid_misses() {
        let mut world = rm_simulator_world::projectile::WorldPhysics::new(&[], 0.);
        world.add_static_box(Pose::at([0., 0.1, 0.4]), [1., 0.8, 0.3]);
        world.add_static_box(Pose::at([0., 2.02, 0.5]), [3.4, 3., 1.]);
        world.add_static_box(Pose::at([0., -1.82, 0.5]), [3.4, 3., 1.]);
        // The narrow side walls continue past both fitted endpoints. Connect
        // to full-height staging nodes with checked stowed edges, rather than
        // demanding the ordinary 980 mm footprint inside this 840 mm lane.
        let geometry = world.static_geometry_snapshot(true);
        let mut map = Map::bake(
            geometry.clone(),
            Surface {
                ground: None,
                floor: 0.,
                exclusions: vec![],
            },
            [-4., -2., 4., 2.],
        );
        assert!(
            map.plan([-3.5, 0.1, 0.], [3.5, 0.1], &[], &geometry, true)
                .is_err()
        );
        map.add_passages(&[[[-0.6, 0.1, 0.], [0.6, 0.1, 0.]]]);
        assert_eq!(map.summary.passages, 1);
        let route = map
            .plan([-3.5, 0.1, 0.], [3.5, 0.1], &[], &geometry, true)
            .unwrap();
        assert!(route.tunnels.iter().any(|&t| t));
        assert!(
            map.plan([-3.5, 0.1, 0.], [3.5, 0.1], &[], &geometry, false)
                .is_err()
        );
        let resumed = map
            .plan([0., 0.1, 0.], [3.5, 0.1], &[], &geometry, true)
            .unwrap();
        assert!(
            resumed.tunnels[1],
            "resuming underneath must preserve the stow action"
        );
    }
    #[test]
    fn source_grade_limits_apply_even_when_travelling_across_the_slope() {
        use crate::collision_mesh::CollisionMesh;
        for (degrees, ordinary, stowed) in [
            (11_f64, true, true),
            (15., false, true),
            (17., false, false),
        ] {
            let slope = degrees.to_radians().tan();
            let surface = Surface {
                ground: Some(Arc::new(
                    CollisionMesh {
                        vertices_m: vec![
                            [-3., -3., -3. * slope],
                            [3., -3., 3. * slope],
                            [3., 3., 3. * slope],
                            [-3., 3., -3. * slope],
                        ],
                        triangles: vec![[0, 1, 2], [0, 2, 3]],
                    }
                    .into(),
                )),
                floor: 0.,
                exclusions: vec![],
            };
            for heading in [0., std::f64::consts::FRAC_PI_2] {
                assert_eq!(
                    surface.supported_heading([0.; 3], false, heading),
                    ordinary,
                    "{degrees}"
                );
                assert_eq!(
                    surface.supported_heading([0.; 3], true, heading),
                    stowed,
                    "{degrees}"
                );
            }
        }
    }
    #[test]
    fn tunnel_approach_accepts_15_degree_ramp_with_fitted_road_padding() {
        use crate::collision_mesh::CollisionMesh;
        let slope = 15_f64.to_radians().tan();
        let map = Map::bake(
            StaticGeometry::default(),
            Surface {
                ground: Some(Arc::new(
                    CollisionMesh {
                        vertices_m: vec![
                            [-3., -3., -3. * slope],
                            [3., -3., 3. * slope],
                            [3., 3., 3. * slope],
                            [-3., 3., -3. * slope],
                        ],
                        triangles: vec![[0, 1, 2], [0, 2, 3]],
                    }
                    .into(),
                )),
                floor: 0.,
                exclusions: vec![],
            },
            [-2., -2., 2., 2.],
        );
        let a = [-1., 0., -slope];
        let b = [1., 0., slope];
        assert!(!map.segment_clear(a, b, false));
        assert!(map.segment_clear(a, b, true));
        let road = Exclusion {
            min: [0., -1.],
            max: [1., 1.],
        };
        assert!(road.crosses([-0.4, 0., 0.], [-0.4, 0., 0.]));
        assert!(!road.crosses_with([-0.4, 0., 0.], [-0.4, 0., 0.], 0.35));
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
                map.plan(
                    [-1., 0., 0.],
                    [1., 0.],
                    &[],
                    &StaticGeometry::default(),
                    false
                )
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
        let path = map
            .plan([-2., 0., 0.], [2., 0.], &[], &geometry, false)
            .unwrap()
            .points;
        assert!(path.len() >= 3);
        assert!(path.windows(2).all(|p| !region.crosses(p[0], p[1])));
        assert!(
            map.plan([-2., 0., 0.], [0., 0.], &[], &geometry, false)
                .is_err()
        );
        assert!(region.crosses([0.6, 0., 0.], [0.6, 0., 0.]));
        let roads = road_exclusions();
        assert_eq!(roads.len(), 2);
        assert!(
            roads
                .iter()
                .all(|r| (0..2).all(|i| r.min[i].is_finite() && r.min[i] < r.max[i]))
        );
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
            .plan([0., 0., 0.], [2., 0.], &a.failed, &geometry, false)
            .unwrap()
            .points;
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
            map.plan(
                [-2., 0., 0.],
                [2., 0.],
                &[],
                &StaticGeometry::default(),
                false
            )
            .is_err()
        );
    }
    #[test]
    fn preparing_does_not_replan_against_a_future_translation() {
        let mut field = Field::new(&FieldConfig {
            runes: vec![],
            outposts: vec![],
            ..Default::default()
        })
        .unwrap();
        let mut physics = rm_simulator_world::projectile::WorldPhysics::new(&[], 0.);
        physics.add_static_box(Pose::at([0.7, 0., 0.5]), [0.05, 4., 1.]);
        let (v, t) = physics.static_geometry_snapshot(true).into_triangles();
        field.add_static_mesh(v, t).unwrap();
        let config = ChassisConfig::folding_hero();
        let id = field
            .add_chassis(&ChassisPlacement {
                spawn: Pose {
                    translation_m: [0., 0., config.rest_height_m()],
                    rotation_wxyz: crate::math::quat_axis_angle(
                        [0., 0., 1.],
                        std::f64::consts::FRAC_PI_2,
                    ),
                },
                config,
                team: Team::Red,
                kind: RobotKind::Sentry,
                performance: None,
            })
            .unwrap();
        field.step(128).unwrap();
        let mut service = Service::new(
            StaticGeometry::default(),
            None,
            0.,
            [-3., -3., 3., 3.],
            vec![],
        )
        .unwrap();
        service.add(id);
        let a = service.agents.get_mut(&id).unwrap();
        a.view.path_m = vec![[0., 0., 0.], [2., 0., 0.]];
        a.view.tunnel_segments = vec![false, true];
        a.view.waypoint = 1;
        a.view.phase = Phase::Following;
        for _ in 0..64 {
            service.tick(&mut field);
            field.step(1).unwrap();
            assert_eq!(service.agents[&id].retries, 0);
            assert_eq!(service.agents[&id].view.phase, Phase::Following);
            let command = field.chassis_snapshot(id).unwrap().command;
            assert_eq!([command.forward_m_s, command.left_m_s], [0., 0.]);
        }
        assert!(field.chassis_snapshot(id).unwrap().command.folded);
    }
    #[test]
    fn tunnel_follower_stows_aligns_crosses_and_reopens_with_actual_physics() {
        let mut field = Field::new(&FieldConfig {
            runes: vec![],
            outposts: vec![],
            ..Default::default()
        })
        .unwrap();
        let mut physics = rm_simulator_world::projectile::WorldPhysics::new(&[], 0.);
        physics.add_static_box(Pose::at([0., 0., 0.4]), [1., 8., 0.3]);
        let geometry = physics.static_geometry_snapshot(true);
        let (v, t) = geometry.clone().into_triangles();
        field.add_static_mesh(v, t).unwrap();
        let config = ChassisConfig::folding_hero();
        let id = field
            .add_chassis(&ChassisPlacement {
                spawn: Pose {
                    translation_m: [-2.5, 0., config.rest_height_m()],
                    rotation_wxyz: crate::math::quat_axis_angle(
                        [0., 0., 1.],
                        std::f64::consts::FRAC_PI_2,
                    ),
                },
                config,
                team: Team::Red,
                kind: RobotKind::Sentry,
                performance: None,
            })
            .unwrap();
        field.step(128).unwrap();
        let map = Map::bake(
            geometry.clone(),
            Surface {
                ground: None,
                floor: 0.,
                exclusions: vec![],
            },
            [-3., -2., 3., 2.],
        );
        let route = map
            .plan([-2.5, 0., 0.], [2.5, 0.], &[], &geometry, true)
            .unwrap();
        let mut service = Service::new(
            StaticGeometry::default(),
            None,
            0.,
            [-3., -2., 3., 2.],
            vec![],
        )
        .unwrap();
        service.add(id);
        let a = service.agents.get_mut(&id).unwrap();
        a.view.path_m = route.points;
        a.view.tunnel_segments = route.tunnels;
        a.view.phase = Phase::Following;
        a.view.waypoint = 1;
        let mut prepared = false;
        let mut crossed = false;
        for _ in 0..128 * 45 {
            service.tick(&mut field);
            field.step(1).unwrap();
            let body = field.chassis_snapshot(id).unwrap();
            if service.views()[0].message == "Stowing and aligning before tunnel" {
                prepared = true;
                assert!(body.command.forward_m_s.hypot(body.command.left_m_s) <= 0.17);
            }
            if !crossed && body.pose.translation_m[0].abs() < 0.3 {
                crossed = true;
                assert!(body.command.folded);
                assert!((body.fold_angle_rad - body.config.transform_limit_rad()).abs() < 0.001);
                // Braking, including an operator stop, preserves safe stow.
                brake(&mut field, id).unwrap();
                assert!(field.chassis_snapshot(id).unwrap().command.folded);
            }
            if service.views()[0].phase != Phase::Following {
                break;
            }
        }
        assert!(
            prepared && crossed,
            "{:?}, {:?}",
            service.views(),
            field.chassis_snapshot(id).unwrap().pose
        );
        assert_eq!(
            service.views()[0].phase,
            Phase::Arrived,
            "{:?}",
            service.views()
        );
        field.step(512).unwrap();
        assert!(field.chassis_snapshot(id).unwrap().fold_angle_rad < 0.001);
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
