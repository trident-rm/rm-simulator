// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Named source-link collision queries for prescribed manipulators, without CAD
//! loading, rendering or match rules. These checks reject motion; they apply no forces.
use crate::{Pose, manipulator::rigid_pose};
use rapier3d_f64::{
    math::Vector,
    parry::{
        bounding_volume::BoundingVolume,
        query::{self, NonlinearRigidMotion},
    },
    prelude::SharedShape,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

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
    /// True when conservative advancement exhausted its iteration budget;
    /// this is an unresolved obstruction, never a proof of clearance.
    pub conservative: bool,
}
#[derive(Clone)]
struct Body {
    name: String,
    shapes: Vec<SharedShape>,
    radius_m: f64,
    envelope: SharedShape,
}
/// Shared, immutable BVHs and convex shapes, cheap to clone for transactional IK.
#[derive(Clone)]
pub struct CollisionScene {
    bodies: Vec<Body>,
    excluded: BTreeSet<(String, String)>,
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
            let mut envelopes = Vec::new();
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
                        let points: Vec<_> =
                            points_m.iter().copied().map(Vector::from_array).collect();
                        let hull = SharedShape::convex_hull(&points).unwrap_or_else(|| {
                            let mut min = points[0];
                            let mut max = points[0];
                            for p in &points {
                                min = min.min(*p);
                                max = max.max(*p);
                            }
                            let half = (max - min) * 0.5;
                            SharedShape::compound(vec![(
                                rapier3d_f64::math::Pose::translation(
                                    (min.x + max.x) * 0.5,
                                    (min.y + max.y) * 0.5,
                                    (min.z + max.z) * 0.5,
                                ),
                                SharedShape::cuboid(
                                    half.x.max(1e-8),
                                    half.y.max(1e-8),
                                    half.z.max(1e-8),
                                ),
                            )])
                        });
                        // A compound cannot contain another compound. Flatten the
                        // planar fallback's translated cuboid into this envelope.
                        if let Some(parts) = hull.as_compound() {
                            envelopes.extend(parts.shapes().iter().cloned());
                        } else {
                            envelopes.push((Default::default(), hull));
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
            envelopes.extend(convex.iter().cloned());
            let envelope = SharedShape::compound(envelopes);
            let mut shapes = Vec::new();
            if !convex.is_empty() {
                shapes.push(SharedShape::compound(convex));
            }
            if !vertices.is_empty() {
                shapes.push(
                    SharedShape::trimesh(vertices, indices)
                        .map_err(|_| "invalid source triangle mesh")?,
                );
            }
            bodies.push(Body {
                name: body.frame,
                shapes,
                radius_m,
                envelope,
            });
        }
        let mut excluded = BTreeSet::new();
        for [a, b] in asset.excluded_pairs {
            if a == b || !names.contains(&a) || !names.contains(&b) {
                return Err("unknown or reflexive collision exclusion");
            }
            excluded.insert(pair(&a, &b));
        }
        Ok(Self { bodies, excluded })
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
    fn skipped(&self, a: &Body, b: &Body, allowed: &[[&str; 2]]) -> bool {
        self.excluded.contains(&pair(&a.name, &b.name))
            || allowed
                .iter()
                .any(|[x, y]| (a.name == *x && b.name == *y) || (a.name == *y && b.name == *x))
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
        if !tolerance_m.is_finite() || tolerance_m < 0. {
            return Err("invalid collision tolerance");
        }
        let poses = self.poses(frames)?;
        let mut result = Vec::new();
        for (i, a) in self.bodies.iter().enumerate() {
            for (j, b) in self.bodies.iter().enumerate().skip(i + 1) {
                if self.skipped(a, b, allowed)
                    || (poses[i].translation - poses[j].translation).length()
                        > a.radius_m + b.radius_m
                {
                    continue;
                }
                if !query::intersection_test(
                    &poses[i],
                    a.envelope.as_ref(),
                    &poses[j],
                    b.envelope.as_ref(),
                )
                .map_err(|_| "unsupported collision envelope query")?
                {
                    continue;
                }
                let mut deepest = None;
                for x in &a.shapes {
                    for y in &b.shapes {
                        if !x
                            .compute_aabb(&poses[i])
                            .intersects(&y.compute_aabb(&poses[j]))
                        {
                            continue;
                        }
                        if x.as_trimesh().is_some() && y.as_trimesh().is_some() {
                            if query::intersection_test(
                                &poses[i],
                                x.as_ref(),
                                &poses[j],
                                y.as_ref(),
                            )
                            .map_err(|_| "unsupported source surface query")?
                            {
                                result.push(CollisionHit {
                                    first: a.name.clone(),
                                    second: b.name.clone(),
                                    point_m: poses[i].translation.to_array(),
                                    penetration_m: 0.,
                                    fraction: 1.,
                                    conservative: false,
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
                        conservative: false,
                    });
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
        if let Some(hit) = self.contacts(to, allowed, tolerance_m)?.into_iter().next() {
            return Ok(Some(hit));
        }
        let before = self.poses(from)?;
        let after = self.poses(to)?;
        for (i, a) in self.bodies.iter().enumerate() {
            for (j, b) in self.bodies.iter().enumerate().skip(i + 1) {
                if self.skipped(a, b, allowed) || (before[i] == after[i] && before[j] == after[j]) {
                    continue;
                }
                let motion = |k: usize| {
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
                };
                // Origin-centred spheres enclose all orientations over the sweep.
                let travel = (after[i].translation - before[i].translation).length()
                    + (after[j].translation - before[j].translation).length();
                if (before[i].translation - before[j].translation).length()
                    > a.radius_m + b.radius_m + travel
                {
                    continue;
                }
                let ma = motion(i);
                let mb = motion(j);
                for x in &a.shapes {
                    for y in &b.shapes {
                        // Endpoint AABBs enlarged by the maximum rotational point
                        // displacement enclose every intermediate orientation.
                        let aa = x
                            .compute_aabb(&before[i])
                            .merged(&x.compute_aabb(&after[i]))
                            .loosened(a.radius_m * ma.angvel.length());
                        let bb = y
                            .compute_aabb(&before[j])
                            .merged(&y.compute_aabb(&after[j]))
                            .loosened(b.radius_m * mb.angvel.length());
                        if !aa.intersects(&bb) {
                            continue;
                        }
                        // Distance is Lipschitz with the sum of maximum point speeds.
                        // This conservative advancement keeps mesh apertures intact and
                        // avoids treating a composite bounding-volume TOI as a surface hit.
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
                            let envelope_distance =
                                query::distance(&pa, a.envelope.as_ref(), &pb, b.envelope.as_ref())
                                    .map_err(|_| "unsupported collision envelope distance")?;
                            let distance = if envelope_distance > 2e-6 {
                                envelope_distance
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
                                    conservative: distance > 2e-6,
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
    fn solid_contacts_exclusions_and_missing_frames_fail_closed() {
        let s = scene(vec![body("arm", [0.1; 3]), body("wall", [0.1; 3])]);
        let p = frames(Pose::default(), Pose::at([0.15, 0., 0.]));
        let hit = s.contacts(&p, &[], 0.0005).unwrap();
        assert_eq!(hit.len(), 1);
        assert!((hit[0].penetration_m - 0.05).abs() < 1e-6);
        assert!(s.contacts(&p, &[["arm", "wall"]], 0.).unwrap().is_empty());
        assert!(s.contacts(&BTreeMap::new(), &[], 0.).is_err());
        assert!(s.contacts(&p, &[], f64::NAN).is_err());
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
}
