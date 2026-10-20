// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Named source-link collision queries for prescribed manipulators, without CAD
//! loading, rendering or match rules. These checks reject motion; they apply no forces.
use crate::{Pose, manipulator::rigid_pose};
use rapier3d_f64::{
    math::Vector,
    parry::{
        bounding_volume::{Aabb, BoundingVolume},
        query::{self, NonlinearRigidMotion},
    },
    prelude::SharedShape,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

/// Serializable collision geometry in one source link's local FLU coordinates.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum CollisionPart {
    /// Conservative hull of one source leaf; never a hull spanning several joints.
    Convex {
        /// Finite hull vertices in metres; must span a volume.
        points_m: Vec<[f64; 3]>,
    },
    /// Exact source triangles, preserving housing and jaw apertures.
    Mesh {
        /// Finite vertices in metres.
        points_m: Vec<[f64; 3]>,
        /// Vertex-index triples; no out-of-bounds or repeated indices.
        triangles: Vec<[u32; 3]>,
    },
    /// Analytic cuboid centred on the frame origin.
    Box {
        /// Strictly positive half extents in metres.
        half_m: [f64; 3],
    },
}
/// Shapes rigidly attached to one named link frame.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CollisionBody {
    /// Stable caller frame name, unique within an asset.
    pub frame: String,
    /// Source-local shapes, at least one per body.
    pub parts: Vec<CollisionPart>,
}
/// Rebuildable collision input; the caller verifies its source checksums.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CollisionAsset {
    /// Format identity; currently 1.
    pub schema: u32,
    /// Bodies in deterministic query order.
    pub bodies: Vec<CollisionBody>,
    /// Explicit joint/interface contacts which this model does not constrain.
    pub excluded_pairs: Vec<[String; 2]>,
}
/// Evidence for an obstructed pose or an impact during a rigid-frame sweep.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CollisionHit {
    /// First colliding source frame.
    pub first: String,
    /// Second colliding source frame.
    pub second: String,
    /// Diagnostic point in world FLU metres: contact witness for solid queries,
    /// first frame origin for surface intersections and conservative swept queries.
    pub point_m: [f64; 3],
    /// Nonnegative penetration depth in metres, zero for a swept impact.
    pub penetration_m: f64,
    /// Fraction of the proposed transition at impact; 1 for a pose query.
    pub fraction: f64,
    /// True for a proxy obstruction or exhausted conservative advancement;
    /// this is an unresolved source obstruction, never a proof of clearance.
    pub conservative: bool,
}
#[derive(Clone)]
struct Body {
    name: String,
    shapes: Vec<SharedShape>,
    envelopes: Vec<SharedShape>,
    part_tree: std::sync::Arc<PartTree>,
    radius_m: f64,
    approximate: bool,
}
// A BVH across source chunks, in addition to the BVH inside each chunk. This
// avoids testing every terrain chunk against every small robot part.
#[derive(Clone)]
struct PartTree {
    bounds: Aabb,
    children: Option<Box<[PartTree; 2]>>,
    indices: Vec<usize>,
}
impl PartTree {
    fn build(bounds: &[Aabb], mut indices: Vec<usize>) -> Self {
        let total = indices
            .iter()
            .map(|&i| bounds[i])
            .reduce(|a, b| a.merged(&b))
            .unwrap();
        if indices.len() <= 8 {
            return Self {
                bounds: total,
                children: None,
                indices,
            };
        }
        let size = total.extents();
        let axis = if size.x >= size.y && size.x >= size.z {
            0
        } else if size.y >= size.z {
            1
        } else {
            2
        };
        let mid = indices.len() / 2;
        indices.select_nth_unstable_by(mid, |&a, &b| {
            bounds[a].center()[axis].total_cmp(&bounds[b].center()[axis])
        });
        let right = indices.split_off(mid);
        Self {
            bounds: total,
            children: Some(Box::new([
                Self::build(bounds, indices),
                Self::build(bounds, right),
            ])),
            indices: vec![],
        }
    }
    fn query(&self, bounds: &Aabb, out: &mut Vec<usize>) {
        if !self.bounds.intersects(bounds) {
            return;
        }
        if let Some(children) = &self.children {
            for child in children.iter() {
                child.query(bounds, out);
            }
        } else {
            out.extend_from_slice(&self.indices);
        }
    }
}
impl Body {
    fn candidates(
        &self,
        world_bounds: Aabb,
        pose: rapier3d_f64::math::Pose,
        travel_m: f64,
    ) -> Vec<usize> {
        if self.shapes.len() <= 8 {
            return (0..self.shapes.len()).collect();
        }
        let query = world_bounds
            .loosened(travel_m + 1e-9)
            .transform_by(&pose.inverse());
        let mut result = vec![];
        self.part_tree.query(&query, &mut result);
        result.sort_unstable(); // Preserve deterministic first-contact ordering.
        result
    }
}
/// Shared, immutable BVHs and convex shapes, cheap to clone for transactional IK.
#[derive(Clone)]
pub struct CollisionScene {
    bodies: Vec<Body>,
    excluded: Vec<Vec<bool>>,
    // Bounded exact endpoint history per pair. Revisited branches retain source
    // checks; winding/poses and tolerance must match, with no nearby-pose reuse.
    contacts_cache: std::sync::Arc<std::sync::Mutex<ContactHistory>>,
    // Two exact placed states per body retain both endpoints of short sweeps.
    // Geometry belongs to this immutable scene; convex replacements reset it.
    bounds_cache: std::sync::Arc<std::sync::Mutex<Vec<Vec<PlacedBounds>>>>,
    approximate: bool,
}
type ContactHistory = BTreeMap<(usize, usize), VecDeque<PairContacts>>;
#[derive(Clone)]
struct PlacedBounds {
    pose: rapier3d_f64::math::Pose,
    bounds: std::sync::Arc<[Aabb]>,
}
#[derive(Clone)]
struct PairContacts {
    poses: [rapier3d_f64::math::Pose; 2],
    tolerance_m: f64,
    hits: Vec<CollisionHit>,
}
fn pair(a: &str, b: &str) -> (String, String) {
    if a < b {
        (a.into(), b.into())
    } else {
        (b.into(), a.into())
    }
}
fn valid_pose(p: Pose) -> bool {
    p.translation_m
        .into_iter()
        .chain(p.rotation_wxyz)
        .all(f64::is_finite)
        && (p.rotation_wxyz.into_iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-8
}
// Spatially partition source triangles, preserving each triangle exactly. Small
// chunks avoid nested whole-link/terrain distance traversals during short sweeps.
fn mesh_chunks(
    vertices: &[Vector],
    mut triangles: Vec<[u32; 3]>,
    out: &mut Vec<SharedShape>,
) -> Result<(), &'static str> {
    if triangles.len() > 128 {
        let centre = |t: &[u32; 3]| {
            (vertices[t[0] as usize] + vertices[t[1] as usize] + vertices[t[2] as usize]) / 3.
        };
        let mut lo = centre(&triangles[0]);
        let mut hi = lo;
        for t in &triangles {
            lo = lo.min(centre(t));
            hi = hi.max(centre(t));
        }
        let size = hi - lo;
        let axis = if size.x >= size.y && size.x >= size.z {
            0
        } else if size.y >= size.z {
            1
        } else {
            2
        };
        let coordinate = |v: Vector| [v.x, v.y, v.z][axis];
        let middle = triangles.len() / 2;
        triangles.select_nth_unstable_by(middle, |a, b| {
            coordinate(centre(a)).total_cmp(&coordinate(centre(b)))
        });
        let right = triangles.split_off(middle);
        mesh_chunks(vertices, triangles, out)?;
        return mesh_chunks(vertices, right, out);
    }
    let mut indices = BTreeMap::new();
    let mut points = Vec::new();
    let triangles = triangles
        .into_iter()
        .map(|t| {
            t.map(|old| {
                *indices.entry(old).or_insert_with(|| {
                    let new = points.len() as u32;
                    points.push(vertices[old as usize]);
                    new
                })
            })
        })
        .collect();
    out.push(SharedShape::trimesh(points, triangles).map_err(|_| "invalid source triangle mesh")?);
    Ok(())
}
fn shape_envelope(shape: &SharedShape) -> SharedShape {
    let Some(mesh) = shape.as_trimesh() else {
        return shape.clone();
    };
    SharedShape::convex_hull(mesh.vertices()).unwrap_or_else(|| {
        let bounds = shape.compute_local_aabb();
        let centre = bounds.center();
        let half = bounds.half_extents();
        SharedShape::compound(vec![(
            rapier3d_f64::math::Pose::translation(centre.x, centre.y, centre.z),
            SharedShape::cuboid(half.x.max(1e-8), half.y.max(1e-8), half.z.max(1e-8)),
        )])
    })
}
impl CollisionScene {
    /// Build shared geometry, rejecting malformed input instead of dropping parts.
    ///
    /// ```
    /// use std::collections::BTreeMap;
    /// use rm_simulator_physics::{Pose,manipulator_collision::*};
    /// let body=|name:&str|CollisionBody{frame:name.into(),parts:vec![CollisionPart::Box{half_m:[0.1;3]}]};
    /// let scene=CollisionScene::new(CollisionAsset{schema:1,bodies:vec![body("tool"),body("wall")],excluded_pairs:vec![]}).unwrap();
    /// let frames=BTreeMap::from([("tool".into(),Pose::default()),("wall".into(),Pose::at([0.15,0.,0.]))]);
    /// assert_eq!(scene.contacts(&frames,&[],0.).unwrap().len(),1);
    /// assert_eq!(scene.first_contact(&frames,&[],0.).unwrap(), scene.contacts(&frames,&[],0.).unwrap().into_iter().next());
    /// let fast=scene.convex_approximation(|name| name == "tool");
    /// assert!(fast.uses_convex_approximation());
    /// assert!(fast.contacts(&frames,&[],0.).unwrap()[0].conservative);
    /// ```
    pub fn new(asset: CollisionAsset) -> Result<Self, &'static str> {
        if asset.schema != 1 || asset.bodies.is_empty() {
            return Err("invalid collision asset schema or empty scene");
        }
        let mut bodies = Vec::new();
        let mut names = BTreeSet::new();
        for body in asset.bodies {
            if body.frame.is_empty() || !names.insert(body.frame.clone()) || body.parts.is_empty() {
                return Err("duplicate or empty collision body");
            }
            let mut convex = Vec::new();
            let mut vertices = Vec::new();
            let mut indices = Vec::new();
            let mut radius_m: f64 = 0.;
            for part in body.parts {
                match part {
                    CollisionPart::Convex { points_m } => {
                        if !points_m.iter().flatten().all(|x| x.is_finite()) {
                            return Err("nonfinite convex collision geometry");
                        }
                        let points: Vec<_> = points_m.into_iter().map(Vector::from_array).collect();
                        for p in &points {
                            radius_m = radius_m.max(p.length());
                        }
                        convex.push((
                            Default::default(),
                            SharedShape::convex_hull(&points)
                                .ok_or("degenerate source collision hull")?,
                        ));
                    }
                    CollisionPart::Box { half_m } => {
                        if half_m.iter().any(|v| !v.is_finite() || *v <= 0.) {
                            return Err("invalid collision box extents");
                        }
                        radius_m = radius_m.max(Vector::from_array(half_m).length());
                        convex.push((
                            Default::default(),
                            SharedShape::cuboid(half_m[0], half_m[1], half_m[2]),
                        ));
                    }
                    CollisionPart::Mesh {
                        points_m,
                        triangles,
                    } => {
                        if points_m.is_empty()
                            || triangles.is_empty()
                            || !points_m.iter().flatten().all(|x| x.is_finite())
                            || triangles.iter().any(|t| {
                                t.iter().any(|i| *i as usize >= points_m.len())
                                    || t[0] == t[1]
                                    || t[1] == t[2]
                                    || t[0] == t[2]
                            })
                        {
                            return Err("invalid source collision mesh");
                        }
                        let offset = u32::try_from(vertices.len())
                            .map_err(|_| "collision mesh too large")?;
                        for p in points_m {
                            let v = Vector::from_array(p);
                            radius_m = radius_m.max(v.length());
                            vertices.push(v);
                        }
                        for t in triangles {
                            indices.push(
                                t.map(|i| {
                                    i.checked_add(offset).ok_or("collision mesh index overflow")
                                })
                                .into_iter()
                                .collect::<Result<Vec<_>, _>>()?
                                .try_into()
                                .unwrap(),
                            );
                        }
                    }
                }
            }
            let mut shapes = Vec::new();
            if !convex.is_empty() {
                shapes.push(SharedShape::compound(convex));
            }
            if !vertices.is_empty() {
                mesh_chunks(&vertices, indices, &mut shapes)?;
            }
            let local_bounds: Vec<_> = shapes
                .iter()
                .map(|shape| shape.compute_local_aabb())
                .collect();
            let part_tree =
                std::sync::Arc::new(PartTree::build(&local_bounds, (0..shapes.len()).collect()));
            bodies.push(Body {
                part_tree,
                name: body.frame,
                envelopes: shapes.iter().map(shape_envelope).collect(),
                shapes,
                radius_m,
                approximate: false,
            });
        }
        let mut excluded = BTreeSet::new();
        for [a, b] in asset.excluded_pairs {
            if a == b || !names.contains(&a) || !names.contains(&b) {
                return Err("unknown or reflexive collision exclusion");
            }
            excluded.insert(pair(&a, &b));
        }
        let excluded = bodies
            .iter()
            .map(|a| {
                bodies
                    .iter()
                    .map(|b| excluded.contains(&pair(&a.name, &b.name)))
                    .collect()
            })
            .collect();
        Ok(Self {
            bodies,
            excluded,
            contacts_cache: Default::default(),
            bounds_cache: Default::default(),
            approximate: false,
        })
    }
    /// Replace selected frames' spatial triangle chunks with convex envelopes.
    /// Geometry consists of convex supersets; normal contact tolerances still
    /// apply. Proxies may block small source apertures. Contact points/depths
    /// describe proxies, not CAD.
    /// The returned scene has independent caches and unchanged frame/exclusions.
    /// Unselected frames retain their source triangles, allowing precise gripper
    /// and socket geometry alongside approximate chassis and arm links.
    pub fn convex_approximation(&self, select: impl Fn(&str) -> bool) -> Self {
        let mut scene = self.clone();
        for body in &mut scene.bodies {
            if select(&body.name) {
                body.shapes.clone_from(&body.envelopes);
                let bounds: Vec<_> = body.shapes.iter().map(|s| s.compute_local_aabb()).collect();
                body.part_tree =
                    std::sync::Arc::new(PartTree::build(&bounds, (0..body.shapes.len()).collect()));
                body.approximate = true;
            }
        }
        scene.contacts_cache = Default::default();
        scene.bounds_cache = Default::default();
        scene.approximate = scene.bodies.iter().any(|b| b.approximate);
        scene
    }
    /// Whether triangle chunks have been replaced by conservative convex proxies.
    pub fn uses_convex_approximation(&self) -> bool {
        self.approximate
    }
    fn poses(
        &self,
        frames: &BTreeMap<String, Pose>,
    ) -> Result<Vec<rapier3d_f64::math::Pose>, &'static str> {
        self.bodies
            .iter()
            .map(|b| {
                let p = *frames.get(&b.name).ok_or("missing collision link frame")?;
                if !valid_pose(p) {
                    return Err("invalid collision link pose");
                }
                Ok(rigid_pose(p))
            })
            .collect()
    }
    fn skipped(&self, i: usize, j: usize, allowed: &[[&str; 2]]) -> bool {
        let a = &self.bodies[i];
        let b = &self.bodies[j];
        self.excluded[i][j]
            || allowed
                .iter()
                .any(|[x, y]| (a.name == *x && b.name == *y) || (a.name == *y && b.name == *x))
    }
    fn placed_bounds(
        &self,
        poses: &[rapier3d_f64::math::Pose],
    ) -> Result<Vec<std::sync::Arc<[Aabb]>>, &'static str> {
        let mut cache = self
            .bounds_cache
            .lock()
            .map_err(|_| "collision bounds cache poisoned")?;
        cache.resize_with(self.bodies.len(), Vec::new);
        let mut result = Vec::with_capacity(self.bodies.len());
        for ((body, pose), entries) in self.bodies.iter().zip(poses).zip(cache.iter_mut()) {
            if let Some(entry) = entries.iter().find(|entry| entry.pose == *pose) {
                result.push(entry.bounds.clone());
                continue;
            }
            let bounds: std::sync::Arc<[Aabb]> = body
                .shapes
                .iter()
                .map(|shape| shape.compute_aabb(pose))
                .collect();
            if entries.len() == 2 {
                entries.remove(0);
            }
            entries.push(PlacedBounds {
                pose: *pose,
                bounds: bounds.clone(),
            });
            result.push(bounds);
        }
        Ok(result)
    }
    /// Report solid penetrations exceeding a nonnegative tolerance in metres,
    /// and exact mesh/mesh surface intersections (no volume depth is inferred).
    /// Every body frame must be supplied, including stationary bodies. Query errors
    /// are returned, never interpreted as clearance.
    pub fn contacts(
        &self,
        frames: &BTreeMap<String, Pose>,
        allowed: &[[&str; 2]],
        tolerance_m: f64,
    ) -> Result<Vec<CollisionHit>, &'static str> {
        self.contacts_until(frames, allowed, tolerance_m, false)
    }
    /// First contact in the same deterministic order as [`Self::contacts`].
    /// Stops after the first obstructing pair; clearance still checks every pair.
    /// Units, required frames and tolerance semantics match [`Self::contacts`].
    pub fn first_contact(
        &self,
        frames: &BTreeMap<String, Pose>,
        allowed: &[[&str; 2]],
        tolerance_m: f64,
    ) -> Result<Option<CollisionHit>, &'static str> {
        Ok(self
            .contacts_until(frames, allowed, tolerance_m, true)?
            .into_iter()
            .next())
    }
    fn contacts_until(
        &self,
        frames: &BTreeMap<String, Pose>,
        allowed: &[[&str; 2]],
        tolerance_m: f64,
        first_only: bool,
    ) -> Result<Vec<CollisionHit>, &'static str> {
        if !tolerance_m.is_finite() || tolerance_m < 0. {
            return Err("invalid collision tolerance");
        }
        let poses = self.poses(frames)?;
        let bounds = self.placed_bounds(&poses)?;
        let body_bounds: Vec<_> = bounds
            .iter()
            .map(|parts| parts.iter().copied().reduce(|a, b| a.merged(&b)).unwrap())
            .collect();
        let mut result = Vec::new();
        for (i, a) in self.bodies.iter().enumerate() {
            for (j, b) in self.bodies.iter().enumerate().skip(i + 1) {
                if self.skipped(i, j, allowed)
                    || !body_bounds[i].intersects(&body_bounds[j])
                    || (poses[i].translation - poses[j].translation).length()
                        > a.radius_m + b.radius_m
                {
                    continue;
                }
                let cached = self
                    .contacts_cache
                    .lock()
                    .map_err(|_| "collision cache poisoned")?
                    .get(&(i, j))
                    .and_then(|entries| {
                        entries.iter().rev().find(|c| {
                            c.poses == [poses[i], poses[j]] && c.tolerance_m == tolerance_m
                        })
                    })
                    .cloned();
                if let Some(cached) = cached {
                    result.extend(cached.hits);
                    if first_only && !result.is_empty() {
                        return Ok(result);
                    }
                    continue;
                }
                let first_hit = result.len();
                let mut deepest = None;
                let mut surface_hit = false;
                for (xi, x) in a.shapes.iter().enumerate() {
                    for yi in b.candidates(bounds[i][xi], poses[j], 0.) {
                        let y = &b.shapes[yi];
                        if !bounds[i][xi].intersects(&bounds[j][yi]) {
                            continue;
                        }
                        if !query::intersection_test(
                            &poses[i],
                            a.envelopes[xi].as_ref(),
                            &poses[j],
                            b.envelopes[yi].as_ref(),
                        )
                        .map_err(|_| "unsupported collision envelope query")?
                        {
                            continue;
                        }
                        if x.as_trimesh().is_some() && y.as_trimesh().is_some() {
                            if surface_hit {
                                continue;
                            }
                            if query::intersection_test(
                                &poses[i],
                                x.as_ref(),
                                &poses[j],
                                y.as_ref(),
                            )
                            .map_err(|_| "unsupported source surface query")?
                            {
                                surface_hit = true;
                                result.push(CollisionHit {
                                    first: a.name.clone(),
                                    second: b.name.clone(),
                                    point_m: poses[i].translation.to_array(),
                                    penetration_m: 0.,
                                    fraction: 1.,
                                    conservative: a.approximate || b.approximate,
                                });
                            }
                            continue;
                        }
                        if let Some(hit) =
                            query::contact(&poses[i], x.as_ref(), &poses[j], y.as_ref(), 0.)
                                .map_err(|_| "unsupported collision contact pair")?
                            && hit.dist < -tolerance_m
                            && deepest
                                .as_ref()
                                .is_none_or(|h: &query::Contact| hit.dist < h.dist)
                        {
                            deepest = Some(hit);
                        }
                    }
                }
                if let Some(hit) = deepest {
                    result.push(CollisionHit {
                        first: a.name.clone(),
                        second: b.name.clone(),
                        point_m: hit.point1.to_array(),
                        penetration_m: -hit.dist,
                        fraction: 1.,
                        conservative: a.approximate || b.approximate,
                    });
                }
                let mut cache = self
                    .contacts_cache
                    .lock()
                    .map_err(|_| "collision cache poisoned")?;
                let entries = cache.entry((i, j)).or_default();
                if entries.len() >= 32 {
                    entries.pop_front();
                }
                entries.push_back(PairContacts {
                    poses: [poses[i], poses[j]],
                    tolerance_m,
                    hits: result[first_hit..].to_vec(),
                });
                if first_only && !result.is_empty() {
                    return Ok(result);
                }
            }
        }
        Ok(result)
    }
    /// First obstructing contact or swept impact between two rigid-frame states.
    /// Translation and shortest-arc rotation are swept, including moving obstacles.
    /// Callers with articulated trajectories subdivide in joint space before this
    /// query: rigid-frame interpolation is not an exact multi-joint trajectory.
    pub fn check_motion(
        &self,
        from: &BTreeMap<String, Pose>,
        to: &BTreeMap<String, Pose>,
        allowed: &[[&str; 2]],
        tolerance_m: f64,
    ) -> Result<Option<CollisionHit>, &'static str> {
        if let Some(hit) = self.first_contact(to, allowed, tolerance_m)? {
            return Ok(Some(hit));
        }
        let before = self.poses(from)?;
        let after = self.poses(to)?;
        let motions: Vec<_> = (0..self.bodies.len())
            .map(|k| {
                let mut q = after[k].rotation * before[k].rotation.inverse();
                if q.w < 0. {
                    q = -q;
                }
                let v = Vector::new(q.x, q.y, q.z);
                let n = v.length();
                let angular = if n > 1e-12 {
                    v * (2. * n.atan2(q.w) / n)
                } else {
                    Vector::ZERO
                };
                NonlinearRigidMotion::new(
                    before[k],
                    Vector::ZERO,
                    after[k].translation - before[k].translation,
                    angular,
                )
            })
            .collect();

        let before_bounds = self.placed_bounds(&before)?;
        let after_bounds = self.placed_bounds(&after)?;
        let bounds: Vec<Vec<_>> = self
            .bodies
            .iter()
            .enumerate()
            .map(|(i, body)| {
                body.shapes
                    .iter()
                    .enumerate()
                    .map(|(index, _)| {
                        // A point travels at most radius * angle under rotation.
                        before_bounds[i][index]
                            .merged(&after_bounds[i][index])
                            .loosened(body.radius_m * motions[i].angvel.length())
                    })
                    .collect()
            })
            .collect();

        let body_bounds: Vec<_> = bounds
            .iter()
            .map(|parts| parts.iter().copied().reduce(|a, b| a.merged(&b)).unwrap())
            .collect();

        for (i, a) in self.bodies.iter().enumerate() {
            for (j, b) in self.bodies.iter().enumerate().skip(i + 1) {
                if self.skipped(i, j, allowed)
                    || !body_bounds[i].intersects(&body_bounds[j])
                    || (before[i] == after[i] && before[j] == after[j])
                {
                    continue;
                }
                // Origin-centred spheres enclose all orientations over the sweep.
                let travel = (after[i].translation - before[i].translation).length()
                    + (after[j].translation - before[j].translation).length();
                if (before[i].translation - before[j].translation).length()
                    > a.radius_m + b.radius_m + travel
                {
                    continue;
                }
                let ma = &motions[i];
                let mb = &motions[j];
                for (xi, x) in a.shapes.iter().enumerate() {
                    for yi in b.candidates(
                        bounds[i][xi],
                        before[j],
                        motions[j].linvel.length() + b.radius_m * motions[j].angvel.length(),
                    ) {
                        let y = &b.shapes[yi];
                        if !bounds[i][xi].intersects(&bounds[j][yi]) {
                            continue;
                        }
                        // Distance is Lipschitz with the sum of maximum point speeds.
                        // This conservative advancement keeps mesh apertures intact and
                        // avoids treating a bounding-volume impact as a surface hit.
                        let speed = ma.linvel.length()
                            + mb.linvel.length()
                            + a.radius_m * ma.angvel.length()
                            + b.radius_m * mb.angvel.length();
                        if speed <= 1e-14 {
                            continue;
                        }
                        let mut t = 0.;
                        let mut stopped = false;
                        for iteration in 0..512 {
                            let pa = ma.position_at_time(t);
                            let pb = mb.position_at_time(t);
                            // A convex superset gives a cheap distance lower bound.
                            // Only near its surface do we need the exact triangles.
                            let lower = query::distance(
                                &pa,
                                a.envelopes[xi].as_ref(),
                                &pb,
                                b.envelopes[yi].as_ref(),
                            )
                            .map_err(|_| "unsupported collision envelope distance")?;
                            let distance = if lower > 2e-6 {
                                lower
                            } else {
                                query::distance(&pa, x.as_ref(), &pb, y.as_ref())
                                    .map_err(|_| "unsupported collision distance query")?
                            };
                            if !distance.is_finite() {
                                return Err("nonfinite collision distance");
                            }
                            if distance <= 2e-6 || iteration == 511 {
                                return Ok(Some(CollisionHit {
                                    first: a.name.clone(),
                                    second: b.name.clone(),
                                    point_m: pa.translation.to_array(),
                                    penetration_m: 0.,
                                    fraction: t,
                                    conservative: a.approximate || b.approximate || distance > 2e-6,
                                }));
                            }
                            let advance = 0.9 * (distance - 1e-6) / speed;
                            if t + advance >= 1. {
                                stopped = true;
                                break;
                            }
                            t += advance;
                        }
                        if !stopped {
                            return Err("collision advancement did not settle");
                        }
                    }
                }
            }
        }
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn body(name: &str, half_m: [f64; 3]) -> CollisionBody {
        CollisionBody {
            frame: name.into(),
            parts: vec![CollisionPart::Box { half_m }],
        }
    }
    fn scene(bodies: Vec<CollisionBody>) -> CollisionScene {
        CollisionScene::new(CollisionAsset {
            schema: 1,
            bodies,
            excluded_pairs: vec![],
        })
        .unwrap()
    }
    fn frames(a: Pose, b: Pose) -> BTreeMap<String, Pose> {
        BTreeMap::from([("arm".into(), a), ("wall".into(), b)])
    }
    #[test]
    fn chunk_tree_contains_all_endpoint_and_swept_candidates() {
        let shapes: Vec<_> = (0..80)
            .map(|i| {
                SharedShape::compound(vec![(
                    rapier3d_f64::math::Pose::from_translation(Vector::new(
                        i as f64 * 0.03 - 1.2,
                        0.,
                        0.,
                    )),
                    SharedShape::cuboid(0.01, 0.01, 0.01),
                )])
            })
            .collect();
        let bounds: Vec<_> = shapes.iter().map(|s| s.compute_local_aabb()).collect();
        let b = Body {
            name: "chunks".into(),
            part_tree: std::sync::Arc::new(PartTree::build(&bounds, (0..shapes.len()).collect())),
            envelopes: shapes.clone(),
            shapes,
            radius_m: 1.3,
            approximate: false,
        };
        for angle in [0., 0.4, 1.5, 3.] {
            let before = rigid_pose(Pose::yawed([0.1, -0.2, 0.3], angle));
            let query = Aabb::new(Vector::new(-0.3, -0.4, 0.2), Vector::new(0.4, 0.2, 0.4));
            let candidates = b.candidates(query, before, 0.);
            for (i, shape) in b.shapes.iter().enumerate() {
                if shape.compute_aabb(&before).intersects(&query) {
                    assert!(candidates.contains(&i));
                }
            }
            let travel = 0.08 + b.radius_m * 0.3;
            let candidates = b.candidates(query, before, travel);
            for sample in 0..=10 {
                let t = sample as f64 / 10.;
                let pose = rigid_pose(Pose::yawed([0.1 + 0.08 * t, -0.2, 0.3], angle + 0.3 * t));
                for (i, shape) in b.shapes.iter().enumerate() {
                    if shape.compute_aabb(&pose).intersects(&query) {
                        assert!(candidates.contains(&i));
                    }
                }
            }
        }
    }
    #[test]
    fn first_contact_and_bounded_history_preserve_order_and_policy() {
        let cached = scene(vec![body("arm", [0.1; 3]), body("wall", [0.1; 3])]);
        for i in 0..50 {
            let f = frames(Pose::at([i as f64 * 0.001, 0., 0.]), Pose::default());
            let all = cached.contacts(&f, &[], 0.).unwrap();
            assert_eq!(
                cached.first_contact(&f, &[], 0.).unwrap(),
                all.into_iter().next()
            );
            assert_eq!(
                cached.first_contact(&f, &[["arm", "wall"]], 0.).unwrap(),
                None
            );
        }
        assert!(
            cached
                .contacts_cache
                .lock()
                .unwrap()
                .values()
                .all(|v| v.len() <= 32)
        );
        for tolerance in [0., 0.3] {
            let f = frames(Pose::default(), Pose::default());
            let reference = scene(vec![body("arm", [0.1; 3]), body("wall", [0.1; 3])]);
            assert_eq!(
                cached.first_contact(&f, &[], tolerance).unwrap(),
                reference.first_contact(&f, &[], tolerance).unwrap()
            );
        }
        assert!(cached.first_contact(&BTreeMap::new(), &[], 0.).is_err());
        assert!(
            cached
                .first_contact(&frames(Pose::default(), Pose::default()), &[], f64::NAN)
                .is_err()
        );
    }
    #[test]
    fn placed_bounds_reuse_exact_endpoints_and_keep_decisions_unchanged() {
        let cached = scene(vec![body("arm", [0.1; 3]), body("wall", [0.1; 3])]);
        let mut reference = cached.clone();
        reference.contacts_cache = Default::default();
        reference.bounds_cache = Default::default();
        let mut previous = frames(Pose::at([-0.4, 0., 0.]), Pose::default());
        for i in 0..9 {
            let next = frames(
                Pose::yawed([-0.4 + i as f64 * 0.1, 0., 0.], i as f64 * 0.1),
                Pose::default(),
            );
            reference.contacts_cache = Default::default();
            reference.bounds_cache = Default::default();
            assert_eq!(
                cached.contacts(&next, &[], 0.0005).unwrap(),
                reference.contacts(&next, &[], 0.0005).unwrap()
            );
            assert_eq!(
                cached.check_motion(&previous, &next, &[], 0.0005).unwrap(),
                reference
                    .check_motion(&previous, &next, &[], 0.0005)
                    .unwrap()
            );
            let poses = cached.poses(&next).unwrap();
            let a = cached.placed_bounds(&poses).unwrap();
            let b = cached.placed_bounds(&poses).unwrap();
            assert!(a.iter().zip(&b).all(|(a, b)| std::sync::Arc::ptr_eq(a, b)));
            assert!(
                cached
                    .bounds_cache
                    .lock()
                    .unwrap()
                    .iter()
                    .all(|entries| entries.len() <= 2)
            );
            previous = next;
        }
        assert!(
            cached
                .convex_approximation(|_| true)
                .bounds_cache
                .lock()
                .unwrap()
                .is_empty()
        );
    }
    #[test]
    fn solid_contacts_exclusions_and_missing_frames_fail_closed() {
        let s = scene(vec![body("arm", [0.1; 3]), body("wall", [0.1; 3])]);
        let p = frames(Pose::default(), Pose::at([0.15, 0., 0.]));
        let hit = s.contacts(&p, &[], 0.0005).unwrap();
        assert_eq!(hit.len(), 1);
        assert!((hit[0].penetration_m - 0.05).abs() < 1e-6);
        assert!(s.contacts(&p, &[["arm", "wall"]], 0.).unwrap().is_empty());
        assert!(s.contacts(&BTreeMap::new(), &[], 0.).is_err());
        assert!(s.contacts(&p, &[], f64::NAN).is_err());
        // Tolerance, allowances and both absolute poses are part of the query.
        assert!(s.contacts(&p, &[], 0.1).unwrap().is_empty());
        assert_eq!(s.contacts(&p, &[], 0.0005).unwrap(), hit);
        let clear = frames(Pose::at([-1., 0., 0.]), Pose::at([0.15, 0., 0.]));
        assert!(s.contacts(&clear, &[], 0.).unwrap().is_empty());
        assert_eq!(s.contacts(&p, &[], 0.0005).unwrap(), hit);
        let proxies = s.convex_approximation(|name| name == "arm");
        assert!(proxies.uses_convex_approximation());
        let proxy_hit = proxies.contacts(&p, &[], 0.0005).unwrap();
        assert!(proxy_hit[0].conservative);
        assert!(!s.contacts(&p, &[], 0.0005).unwrap()[0].conservative);
    }
    #[test]
    fn sweep_blocks_a_thin_wall_even_with_clear_endpoints() {
        let s = scene(vec![
            body("arm", [0.01; 3]),
            body("wall", [0.0005, 0.3, 0.3]),
        ]);
        let a = frames(Pose::at([-0.2, 0., 0.]), Pose::default());
        let b = frames(Pose::at([0.2, 0., 0.]), Pose::default());
        assert!(s.contacts(&a, &[], 0.).unwrap().is_empty());
        assert!(s.contacts(&b, &[], 0.).unwrap().is_empty());
        let h = s.check_motion(&a, &b, &[], 0.).unwrap().unwrap();
        assert!((0.4..0.6).contains(&h.fraction));
        assert!(
            s.check_motion(&a, &b, &[["arm", "wall"]], 0.)
                .unwrap()
                .is_none()
        );
    }
    #[test]
    fn rotation_is_swept_not_just_the_tool_centre() {
        let s = scene(vec![
            body("arm", [1., 0.02, 0.02]),
            body("wall", [0.03, 0.03, 0.1]),
        ]);
        let a = frames(
            Pose::yawed([0.; 3], -std::f64::consts::FRAC_PI_4),
            Pose::at([0.8, 0., 0.]),
        );
        let b = frames(
            Pose::yawed([0.; 3], std::f64::consts::FRAC_PI_4),
            Pose::at([0.8, 0., 0.]),
        );
        assert!(s.contacts(&a, &[], 0.).unwrap().is_empty());
        assert!(s.contacts(&b, &[], 0.).unwrap().is_empty());
        assert!(s.check_motion(&a, &b, &[], 0.).unwrap().is_some());
    }
    fn sheet(aperture: bool) -> CollisionBody {
        let mut points = vec![];
        let mut triangles = vec![];
        let rectangles = if aperture {
            vec![
                [-1., -0.2, -1., 1.],
                [0.2, 1., -1., 1.],
                [-0.2, 0.2, -1., -0.2],
                [-0.2, 0.2, 0.2, 1.],
            ]
        } else {
            vec![[-1., 1., -1., 1.]]
        };
        for [y0, y1, z0, z1] in rectangles {
            let n = points.len() as u32;
            points.extend([[0., y0, z0], [0., y1, z0], [0., y1, z1], [0., y0, z1]]);
            triangles.extend([[n, n + 1, n + 2], [n, n + 2, n + 3]]);
        }
        CollisionBody {
            frame: "wall".into(),
            parts: vec![CollisionPart::Mesh {
                points_m: points,
                triangles,
            }],
        }
    }
    #[test]
    fn exact_apertures_remain_open_and_sheets_block_sideways_crossing() {
        let s = scene(vec![body("arm", [0.05; 3]), sheet(true)]);
        let a = frames(Pose::at([-0.2, 0., 0.]), Pose::default());
        let b = frames(Pose::at([0.2, 0., 0.]), Pose::default());
        let clear = s.check_motion(&a, &b, &[], 0.).unwrap();
        assert!(clear.is_none(), "{clear:?}");
        let a = frames(Pose::at([-0.2, 0.4, 0.]), Pose::default());
        let b = frames(Pose::at([0.2, 0.4, 0.]), Pose::default());
        assert!(s.check_motion(&a, &b, &[], 0.).unwrap().is_some());
    }
    #[test]
    fn mesh_surface_crossings_are_not_dropped_for_zero_volume_depth() {
        let mut arm = sheet(false);
        arm.frame = "arm".into();
        let s = scene(vec![arm, sheet(false)]);
        let p = frames(
            Pose::yawed([0.; 3], std::f64::consts::FRAC_PI_2),
            Pose::default(),
        );
        let h = s.contacts(&p, &[], 0.001).unwrap();
        assert_eq!(h.len(), 1);
        assert_eq!(h[0].penetration_m, 0.);
    }
    #[test]
    fn partition_preserves_triangles_apertures_and_swept_contacts() {
        let mut points = Vec::new();
        let mut triangles = Vec::new();
        for y in 0..20 {
            for z in 0..20 {
                if (8..12).contains(&y) && (8..12).contains(&z) {
                    continue;
                }
                let y0 = y as f64 * 0.1 - 1.;
                let z0 = z as f64 * 0.1 - 1.;
                let n = points.len() as u32;
                points.extend([
                    Vector::new(0., y0, z0),
                    Vector::new(0., y0 + 0.1, z0),
                    Vector::new(0., y0 + 0.1, z0 + 0.1),
                    Vector::new(0., y0, z0 + 0.1),
                ]);
                triangles.extend([[n, n + 1, n + 2], [n, n + 2, n + 3]]);
            }
        }
        let mut chunks = Vec::new();
        mesh_chunks(&points, triangles.clone(), &mut chunks).unwrap();
        assert!(chunks.len() > 1);
        let key = |t: [Vector; 3]| t.map(|p| p.to_array().map(f64::to_bits));
        let mut expected: Vec<_> = triangles
            .iter()
            .map(|t| key(t.map(|i| points[i as usize])))
            .collect();
        let mut actual = Vec::new();
        for shape in &chunks {
            let mesh = shape.as_trimesh().unwrap();
            actual.extend(
                mesh.indices()
                    .iter()
                    .map(|t| key(t.map(|i| mesh.vertices()[i as usize]))),
            );
        }
        expected.sort();
        actual.sort();
        assert_eq!(actual, expected);
        let wall = CollisionBody {
            frame: "wall".into(),
            parts: vec![CollisionPart::Mesh {
                points_m: points.iter().map(|p| p.to_array()).collect(),
                triangles: triangles.clone(),
            }],
        };
        let s = scene(vec![body("arm", [0.025; 3]), wall]);
        // Compare against an unpartitioned copy of exactly the same geometry.
        let mut reference = s.clone();
        reference.contacts_cache = Default::default();
        reference.bounds_cache = Default::default();
        reference.bodies[1].shapes = vec![SharedShape::trimesh(points, triangles).unwrap()];
        reference.bodies[1].envelopes = reference.bodies[1].shapes.clone();
        for y in [0., 0.15, 0.3, 0.5, 0.95, 1.1] {
            let a = frames(Pose::at([-0.2, y, 0.]), Pose::default());
            let b = frames(Pose::at([0.2, y, 0.]), Pose::default());
            assert_eq!(
                s.check_motion(&a, &b, &[], 0.).unwrap().is_some(),
                reference.check_motion(&a, &b, &[], 0.).unwrap().is_some()
            );
            let c = frames(Pose::at([0., y, 0.]), Pose::default());
            assert_eq!(
                s.contacts(&c, &[], 0.).unwrap().is_empty(),
                reference.contacts(&c, &[], 0.).unwrap().is_empty()
            );
        }
    }
}
