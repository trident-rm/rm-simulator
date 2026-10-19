// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Bounded deterministic planning for prescribed arms. The caller supplies swept
//! collision validation; these searches do not advance time or apply forces.
use crate::{Pose, manipulator::SerialArm};

/// Search limits and deterministic sampling identity.
#[derive(Clone, Copy, Debug)]
pub struct JointSearch {
    /// Maximum random samples, excluding the direct connection attempt.
    pub samples: usize,
    /// Largest change of any joint per tree extension, radians.
    pub step_rad: f64,
    /// Reproducible pseudorandom stream; never a host clock seed.
    pub seed: u64,
}
impl Default for JointSearch {
    fn default() -> Self {
        Self {
            samples: 512,
            step_rad: 0.15,
            seed: 1,
        }
    }
}
/// A bounded search failed; this does not prove that no physical path exists.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlanError {
    /// Invalid limits, poses, endpoints or options.
    InvalidInput,
    /// A caller collision query failed; never interpreted as clearance.
    CollisionQuery,
    /// Either endpoint is obstructed.
    BlockedEndpoint,
    /// No connection was found within the supplied sampling budget.
    Exhausted,
}
/// Distinct numerical IK solutions, ordered by the caller's seed preference.
/// Joint coordinates are not wrapped across their calibrated range boundaries.
///
/// ```
/// use rm_simulator_physics::{Pose,manipulator::*,manipulator_plan::ik_candidates};
/// let axis=RevoluteAxis{origin_m:[0.;3],axis:[0.,0.,1.],range_rad:[-3.,3.]};
/// let arm=SerialArm{axes:[axis;6],tool:Pose::at([0.2,0.,0.])};
/// assert_eq!(ik_candidates(&arm,arm.tool,[ [0.;6] ],4).len(),1);
/// ```
pub fn ik_candidates(
    arm: &SerialArm,
    target: Pose,
    seeds: impl IntoIterator<Item = [f64; 6]>,
    maximum: usize,
) -> Vec<[f64; 6]> {
    let mut found: Vec<[f64; 6]> = Vec::new();
    for seed in seeds {
        if found.len() >= maximum {
            break;
        }
        if let Ok(q) = arm.solve(target, seed)
            && !found.iter().any(|p| distance(*p, q) < 0.001)
        {
            found.push(q);
        }
    }
    found
}
fn distance(a: [f64; 6], b: [f64; 6]) -> f64 {
    (0..6).map(|i| (a[i] - b[i]).abs()).fold(0., f64::max)
}
#[derive(Clone)]
struct Node {
    q: [f64; 6],
    parent: usize,
}
fn branch(tree: &[Node], mut i: usize) -> Vec<[f64; 6]> {
    let mut path = vec![];
    loop {
        path.push(tree[i].q);
        if tree[i].parent == i {
            break;
        }
        i = tree[i].parent;
    }
    path.reverse();
    path
}
fn extend(
    tree: &mut Vec<Node>,
    target: [f64; 6],
    step: f64,
    edge: &mut impl FnMut([f64; 6], [f64; 6]) -> Result<bool, PlanError>,
) -> Result<Option<usize>, PlanError> {
    let nearest = (0..tree.len())
        .min_by(|a, b| distance(tree[*a].q, target).total_cmp(&distance(tree[*b].q, target)))
        .unwrap();
    let from = tree[nearest].q;
    let d = distance(from, target);
    if d < 1e-10 {
        return Ok(Some(nearest));
    }
    let t = (step / d).min(1.);
    let q = std::array::from_fn(|i| from[i] + (target[i] - from[i]) * t);
    if !edge(from, q)? {
        return Ok(None);
    }
    let i = tree.len();
    tree.push(Node { q, parent: nearest });
    Ok(Some(i))
}
/// Bidirectional joint-space RRT-Connect for **free** arm travel. Start/goal are
/// checked first. Every returned edge has passed the caller's swept validator,
/// including shortcuts. No Cartesian or socket constraint is inferred: use a
/// constrained IK guide once the unit enters a socket.
///
/// ```
/// use rm_simulator_physics::manipulator_plan::*;
/// let path=joint_path([[-1.,1.];6],[0.;6],[0.5;6],JointSearch::default(),|_,_|Ok(true)).unwrap();
/// assert_eq!(path,vec![[0.;6],[0.5;6]]);
/// ```
pub fn joint_path(
    limits: [[f64; 2]; 6],
    start: [f64; 6],
    goal: [f64; 6],
    options: JointSearch,
    mut edge: impl FnMut([f64; 6], [f64; 6]) -> Result<bool, PlanError>,
) -> Result<Vec<[f64; 6]>, PlanError> {
    if !options.step_rad.is_finite()
        || options.step_rad <= 0.
        || limits.iter().enumerate().any(|(i, [a, b])| {
            !a.is_finite()
                || !b.is_finite()
                || a >= b
                || !start[i].is_finite()
                || !goal[i].is_finite()
                || !(*a..=*b).contains(&start[i])
                || !(*a..=*b).contains(&goal[i])
        })
    {
        return Err(PlanError::InvalidInput);
    }
    if !edge(start, start)? || !edge(goal, goal)? {
        return Err(PlanError::BlockedEndpoint);
    }
    if edge(start, goal)? {
        return Ok(vec![start, goal]);
    }
    let mut trees = [
        vec![Node {
            q: start,
            parent: 0,
        }],
        vec![Node { q: goal, parent: 0 }],
    ];
    let mut random = options.seed;
    for n in 0..options.samples {
        let side = n % 2;
        let target = if n % 5 == 0 {
            trees[1 - side][0].q
        } else {
            std::array::from_fn(|i| {
                random = random.wrapping_add(0x9e3779b97f4a7c15);
                let mut x = random;
                x = (x ^ (x >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
                x = (x ^ (x >> 27)).wrapping_mul(0x94d049bb133111eb);
                x ^= x >> 31;
                limits[i][0]
                    + (limits[i][1] - limits[i][0]) * ((x >> 11) as f64 / (1u64 << 53) as f64)
            })
        };
        let Some(a) = extend(&mut trees[side], target, options.step_rad, &mut edge)? else {
            continue;
        };
        let q = trees[side][a].q;
        // One extension takes at most step_rad. Bound connect work as well as samples.
        for _ in 0..128 {
            let Some(b) = extend(&mut trees[1 - side], q, options.step_rad, &mut edge)? else {
                break;
            };
            if distance(trees[1 - side][b].q, q) < 1e-10 {
                let mut left = branch(&trees[side], a);
                let mut right = branch(&trees[1 - side], b);
                if side == 1 {
                    std::mem::swap(&mut left, &mut right);
                }
                right.reverse();
                left.extend(right.into_iter().skip(1));
                let mut path = vec![left[0]];
                let mut at = 0;
                while at + 1 < left.len() {
                    let mut next = at + 1;
                    for i in (at + 2..left.len()).rev() {
                        if edge(left[at], left[i])? {
                            next = i;
                            break;
                        }
                    }
                    path.push(left[next]);
                    at = next;
                }
                // Validate forward execution even though the goal tree grew backwards.
                for p in path.windows(2) {
                    if !edge(p[0], p[1])? {
                        return Err(PlanError::Exhausted);
                    }
                }
                return Ok(path);
            }
        }
    }
    Err(PlanError::Exhausted)
}
#[cfg(test)]
mod tests {
    use super::*;
    fn clear(a: [f64; 6], b: [f64; 6]) -> Result<bool, PlanError> {
        // A finite wall separates the endpoints, with space around either end.
        Ok((0..=100).all(|i| {
            let t = i as f64 / 100.;
            let x = a[0] + (b[0] - a[0]) * t;
            let y = a[1] + (b[1] - a[1]) * t;
            !(x.abs() < 0.15 && y.abs() < 0.55)
        }))
    }
    #[test]
    fn routes_around_obstacle_and_replays_deterministically() {
        let a = [-0.8, 0., 0., 0., 0., 0.];
        let b = [0.8, 0., 0., 0., 0., 0.];
        let opt = JointSearch {
            samples: 512,
            ..Default::default()
        };
        let p = joint_path([[-1., 1.]; 6], a, b, opt, clear).unwrap();
        assert!(p.len() > 2);
        assert_eq!(p.first(), Some(&a));
        assert_eq!(p.last(), Some(&b));
        assert!(p.windows(2).all(|w| clear(w[0], w[1]).unwrap()));
        assert_eq!(p, joint_path([[-1., 1.]; 6], a, b, opt, clear).unwrap());
    }
    #[test]
    fn routed_edges_pass_the_real_swept_collision_query() {
        use crate::{Pose, manipulator_collision::*};
        use std::collections::BTreeMap;
        let body = |name: &str, half_m| CollisionBody {
            frame: name.into(),
            parts: vec![CollisionPart::Box { half_m }],
        };
        let scene = CollisionScene::new(CollisionAsset {
            schema: 1,
            bodies: vec![body("tool", [0.025; 3]), body("wall", [0.1, 0.5, 0.5])],
            excluded_pairs: vec![],
        })
        .unwrap();
        // Two configuration coordinates move a tool proxy in a horizontal plane.
        let poses = |q: [f64; 6]| {
            BTreeMap::from([
                ("tool".into(), Pose::at([q[0], q[1], 0.])),
                ("wall".into(), Pose::default()),
            ])
        };
        let mut edge = |a, b| {
            scene
                .check_motion(&poses(a), &poses(b), &[], 0.)
                .map(|h| h.is_none())
                .map_err(|_| PlanError::CollisionQuery)
        };
        let start = [-0.8, 0., 0., 0., 0., 0.];
        let end = [0.8, 0., 0., 0., 0., 0.];
        assert!(!edge(start, end).unwrap());
        let path = joint_path(
            [[-1., 1.]; 6],
            start,
            end,
            JointSearch::default(),
            &mut edge,
        )
        .unwrap();
        assert!(path.len() > 2);
        for pair in path.windows(2) {
            assert!(edge(pair[0], pair[1]).unwrap());
        }
    }
    #[test]
    fn budgets_invalid_inputs_and_query_errors_fail_closed() {
        assert_eq!(
            joint_path(
                [[-1., 1.]; 6],
                [0.; 6],
                [0.8; 6],
                JointSearch::default(),
                |_, _| Ok(false)
            ),
            Err(PlanError::BlockedEndpoint)
        );
        assert_eq!(
            joint_path(
                [[-1., 1.]; 6],
                [0.; 6],
                [0.8; 6],
                JointSearch::default(),
                |_, _| Err(PlanError::CollisionQuery)
            ),
            Err(PlanError::CollisionQuery)
        );
        let a = [-0.8, 0., 0., 0., 0., 0.];
        let b = [0.8, 0., 0., 0., 0., 0.];
        assert_eq!(
            joint_path(
                [[-1., 1.]; 6],
                a,
                b,
                JointSearch {
                    samples: 0,
                    ..Default::default()
                },
                clear
            ),
            Err(PlanError::Exhausted)
        );
        assert_eq!(
            joint_path(
                [[-1., 1.]; 6],
                [2.; 6],
                [0.; 6],
                JointSearch::default(),
                clear
            ),
            Err(PlanError::InvalidInput)
        );
    }
}
