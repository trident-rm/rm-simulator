// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Persistent rigid payloads in the same Rapier scene as robots and terrain.
//!
//! ```
//! use rm_simulator_physics::{WorldPhysics,Pose,TargetFrames};
//! use rm_simulator_physics::projectile::{PayloadShape,PayloadSnapshot};
//! use rm_simulator_physics::manipulator_collision::CollisionPart;
//! let shape=PayloadShape::from_parts(&[CollisionPart::Box{half_m:[0.03;3]}],0.25)?;
//! let mut physics=WorldPhysics::new(&[],0.);
//! let at=Pose::at([0.,0.,1.]);
//! physics.add_payload(&PayloadSnapshot{id:1,shape,pose:at,velocity_m_s:[0.;3],
//!     angular_velocity_rad_s:[0.;3],prescribed:true})?;
//! physics.set_payload_motion(1,false,at,[0.;3],[0.;3])?;
//! physics.step(0,&TargetFrames::new(vec![]))?;
//! assert!(physics.payload(1).unwrap().pose.translation_m[2]<1.);
//! # Ok::<(), &'static str>(())
//! ```
use super::*;
use crate::manipulator_collision::CollisionPart;

// A circumscribed compact hull, admitted only when every original hull vertex
// is enclosed and the maximum displacement of its selected source vertices is
// at most 1 mm. Convexity bounds the entire outer surface by that displacement.
fn compact_hull(points: &[Vector]) -> (Vec<Vector>, f64) {
    if points.len() <= 128 {
        return (points.to_vec(), 0.);
    }
    for count in [64, 128, 256, 512] {
        let mut selected = Vec::new();
        for i in 0..count {
            let z = 1. - 2. * (i as f64 + 0.5) / count as f64;
            let angle = i as f64 * std::f64::consts::PI * (3. - 5f64.sqrt());
            let r = (1. - z * z).sqrt();
            let d = Vector::new(r * angle.cos(), r * angle.sin(), z);
            let p = *points
                .iter()
                .max_by(|a, b| a.dot(d).total_cmp(&b.dot(d)))
                .unwrap();
            if !selected.contains(&p) {
                selected.push(p);
            }
        }
        let Some(hull) = SharedShape::convex_hull(&selected) else {
            continue;
        };
        let h = hull.as_convex_polyhedron().unwrap();
        let center =
            h.points().iter().copied().fold(Vector::ZERO, |a, b| a + b) / h.points().len() as f64;
        let mut factor = 1f64;
        let mut valid = true;
        for face in h.faces() {
            let p =
                h.points()[h.vertices_adj_to_face()[face.first_vertex_or_edge as usize] as usize];
            let plane = (p - center).dot(face.normal);
            if plane <= 1e-12 {
                valid = false;
                break;
            }
            let support = points
                .iter()
                .map(|p| (*p - center).dot(face.normal))
                .fold(f64::NEG_INFINITY, f64::max);
            factor = factor.max(support / plane);
        }
        let expansion = h
            .points()
            .iter()
            .map(|p| (*p - center).length() * (factor - 1.))
            .fold(0., f64::max);
        if valid && expansion <= 0.000998 {
            return (
                h.points()
                    .iter()
                    .map(|p| center + (*p - center) * (factor + 1e-12))
                    .collect(),
                expansion,
            );
        }
    }
    (points.to_vec(), 0.)
}
/// Source-leaf convex outer envelopes for a small carried object.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PayloadShape {
    /// Each leaf's convex vertices, in object-local FLU metres. Apertures may be filled.
    pub hulls_m: Vec<Vec<[f64; 3]>>,
    /// Assumed object mass, kilograms; strictly positive.
    pub mass_kg: f64,
    /// Maximum admitted outward source-vertex displacement, metres; at most 1 mm.
    pub max_expansion_m: f64,
}
impl PayloadShape {
    /// Reduce source leaves to convex outer envelopes, preserving their local frames.
    pub fn from_parts(parts: &[CollisionPart], mass_kg: f64) -> Result<Self, &'static str> {
        let mut hulls_m = Vec::new();
        let mut max_expansion_m = 0.000002f64;
        for part in parts {
            let points: Vec<Vector> = match part {
                CollisionPart::Mesh { points_m, .. } | CollisionPart::Convex { points_m } => {
                    points_m.iter().copied().map(Vector::from_array).collect()
                }
                CollisionPart::Box { half_m } => [-1., 1.]
                    .into_iter()
                    .flat_map(|x| {
                        [-1., 1.].into_iter().flat_map(move |y| {
                            [-1., 1.].into_iter().map(move |z| {
                                Vector::new(x * half_m[0], y * half_m[1], z * half_m[2])
                            })
                        })
                    })
                    .collect(),
            };
            if points.iter().any(|p| !p.is_finite()) {
                return Err("nonfinite payload geometry");
            }
            // CAD face seams carry sub-micrometre noise which prevents flat
            // caps from merging. Canonicalize only the free-body proxy; the
            // source insertion planner retains its original mesh.
            let points: Vec<_> = points
                .into_iter()
                .map(|p| {
                    Vector::new(
                        (p.x * 1e6).round() / 1e6,
                        (p.y * 1e6).round() / 1e6,
                        (p.z * 1e6).round() / 1e6,
                    )
                })
                .collect();
            let hull = SharedShape::convex_hull(&points).ok_or("degenerate payload hull")?;
            let (compact, expansion) = compact_hull(
                hull.as_convex_polyhedron()
                    .ok_or("invalid payload hull")?
                    .points(),
            );
            max_expansion_m = max_expansion_m.max(expansion + 1e-6);
            let mut vertices: Vec<_> = compact
                .iter()
                .map(|p| p.to_array().map(|v| (v * 1e6).round() / 1e6))
                .collect();
            vertices.sort_by(|a, b| {
                a[0].total_cmp(&b[0])
                    .then(a[1].total_cmp(&b[1]))
                    .then(a[2].total_cmp(&b[2]))
            });
            vertices.dedup();
            hulls_m.push(vertices);
        }
        let shape = Self {
            hulls_m,
            mass_kg,
            max_expansion_m,
        };
        shape.shared()?;
        Ok(shape)
    }
    fn shared(&self) -> Result<SharedShape, &'static str> {
        if !self.max_expansion_m.is_finite()
            || !(0.0..=0.001000001).contains(&self.max_expansion_m)
            || !self.mass_kg.is_finite()
            || self.mass_kg <= 0.
            || self.hulls_m.is_empty()
            || self.hulls_m.len() > 32
            || self.hulls_m.iter().map(Vec::len).sum::<usize>() > 8192
        {
            return Err("invalid payload shape");
        }
        let hulls = self
            .hulls_m
            .iter()
            .map(|h| {
                if h.iter().flatten().any(|v| !v.is_finite()) {
                    return Err("nonfinite payload hull");
                }
                SharedShape::convex_hull(
                    &h.iter()
                        .copied()
                        .map(Vector::from_array)
                        .collect::<Vec<_>>(),
                )
                .map(|s| (Pose3::IDENTITY, s))
                .ok_or("degenerate payload hull")
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(SharedShape::compound(hulls))
    }
}
/// Restorable object identity, geometry, pose and physical motion.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PayloadSnapshot {
    /// Stable caller-owned identity, never recycled within a world.
    pub id: u32,
    /// Immutable source-leaf outer envelopes and assumed mass.
    pub shape: PayloadShape,
    /// Object-local to world FLU transform.
    pub pose: Pose,
    /// World linear velocity, metres/second.
    pub velocity_m_s: [f64; 3],
    /// World angular velocity, radians/second.
    pub angular_velocity_rad_s: [f64; 3],
    /// True while held or socket-coupled; colliders are disabled to avoid pushing its carrier.
    pub prescribed: bool,
}
pub(super) struct Body {
    id: u32,
    body: RigidBodyHandle,
    collider: ColliderHandle,
    shape: PayloadShape,
}
impl WorldPhysics {
    pub(super) fn damp_payload_contacts(&mut self) {
        for payload in &self.payloads {
            let touching = self
                .world
                .narrow_phase
                .contact_pairs_with(payload.collider)
                .any(|p| p.has_any_active_contact());
            let body = &mut self.world.bodies[payload.body];
            // Prototype rolling resistance, per second, applied only during
            // contact. Free flight keeps ordinary gravity and no artificial drag.
            let damping = if touching && body.is_dynamic() {
                1.5
            } else {
                0.
            };
            body.set_linear_damping(damping);
            body.set_angular_damping(damping);
        }
    }
    /// Install a persistent object; duplicate identities and invalid states are rejected.
    pub fn add_payload(&mut self, state: &PayloadSnapshot) -> Result<(), &'static str> {
        if self.payloads.iter().any(|b| b.id == state.id)
            || !pose_is_valid(state.pose)
            || state
                .velocity_m_s
                .into_iter()
                .chain(state.angular_velocity_rad_s)
                .any(|v| !v.is_finite())
        {
            return Err("invalid or duplicate payload");
        }
        let shape = state.shape.shared()?;
        let body = self.world.insert_body(
            RigidBodyBuilder::dynamic()
                .pose(pose(state.pose))
                .linvel(Vector::from_array(state.velocity_m_s))
                .angvel(Vector::from_array(state.angular_velocity_rad_s))
                .ccd_enabled(true),
        );
        let collider = self.world.insert_collider(
            ColliderBuilder::new(shape)
                .mass(state.shape.mass_kg)
                .friction(0.6)
                .restitution(0.05),
            Some(body),
        );
        self.payloads.push(Body {
            id: state.id,
            body,
            collider,
            shape: state.shape.clone(),
        });
        self.set_payload_motion(
            state.id,
            state.prescribed,
            state.pose,
            state.velocity_m_s,
            state.angular_velocity_rad_s,
        )?;
        Ok(())
    }
    /// Current physical state of an installed payload, including resting objects.
    pub fn payload(&self, id: u32) -> Option<PayloadSnapshot> {
        let p = self.payloads.iter().find(|p| p.id == id)?;
        let body = &self.world.bodies[p.body];
        let q = body.rotation();
        Some(PayloadSnapshot {
            id,
            shape: p.shape.clone(),
            pose: Pose {
                translation_m: body.translation().to_array(),
                rotation_wxyz: [q.w, q.x, q.y, q.z],
            },
            velocity_m_s: body.linvel().to_array(),
            angular_velocity_rad_s: body.angvel().to_array(),
            prescribed: body.is_kinematic(),
        })
    }
    /// Carry or release an existing object, preserving identity. Released velocity is caller supplied.
    pub fn set_payload_motion(
        &mut self,
        id: u32,
        prescribed: bool,
        at: Pose,
        velocity_m_s: [f64; 3],
        angular_velocity_rad_s: [f64; 3],
    ) -> Result<(), &'static str> {
        if !pose_is_valid(at)
            || velocity_m_s
                .into_iter()
                .chain(angular_velocity_rad_s)
                .any(|v| !v.is_finite())
        {
            return Err("invalid payload motion");
        }
        let p = self
            .payloads
            .iter()
            .find(|p| p.id == id)
            .ok_or("missing payload")?;
        let body = &mut self.world.bodies[p.body];
        let was = body.is_kinematic();
        body.set_body_type(
            if prescribed {
                RigidBodyType::KinematicPositionBased
            } else {
                RigidBodyType::Dynamic
            },
            true,
        );
        if prescribed {
            body.set_next_kinematic_position(pose(at));
            // Set immediately too: checkpoints and held objects must agree at tick boundaries.
            body.set_position(pose(at), true);
        } else if was {
            body.set_position(pose(at), true);
        }
        if prescribed || was {
            body.set_linvel(Vector::from_array(velocity_m_s), true);
            body.set_angvel(Vector::from_array(angular_velocity_rad_s), true);
        }
        self.world.colliders[p.collider].set_enabled(!prescribed);
        Ok(())
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn compact_hulls_enclose_source_vertices_with_bounded_expansion() {
        let points: Vec<_> = (0..512)
            .flat_map(|i| {
                let a = i as f64 * std::f64::consts::TAU / 512.;
                [-0.05, 0.05].map(|z| Vector::new(0.0323 * a.cos(), 0.0323 * a.sin(), z))
            })
            .collect();
        let (reduced, error) = compact_hull(&points);
        assert!(reduced.len() < points.len());
        assert!(error <= 0.001);
        let hull = SharedShape::convex_hull(&reduced).unwrap();
        let h = hull.as_convex_polyhedron().unwrap();
        for f in h.faces() {
            let p = h.points()[h.vertices_adj_to_face()[f.first_vertex_or_edge as usize] as usize];
            assert!(points.iter().all(|v| (*v - p).dot(f.normal) <= 1e-9));
        }
    }
    #[test]
    fn payload_falls_rests_on_a_support_and_restores_velocity() {
        let shape =
            PayloadShape::from_parts(&[CollisionPart::Box { half_m: [0.03; 3] }], 0.25).unwrap();
        let mut world = WorldPhysics::new(&[], 0.);
        world.add_static_box(Pose::at([0., 0., 0.2]), [0.5, 0.5, 0.1]);
        world
            .add_payload(&PayloadSnapshot {
                id: 4,
                shape,
                pose: Pose::at([0., 0., 0.7]),
                velocity_m_s: [0.; 3],
                angular_velocity_rad_s: [0.; 3],
                prescribed: true,
            })
            .unwrap();
        world
            .set_payload_motion(4, false, Pose::at([0., 0., 0.7]), [0.; 3], [0.; 3])
            .unwrap();
        let frames = TargetFrames::new(vec![]);
        for i in 0..256 {
            world.step(i * tick_ns(), &frames).unwrap();
        }
        let state = world.payload(4).unwrap();
        assert!(
            (state.pose.translation_m[2] - 0.28).abs() < 0.004,
            "{state:?}"
        );
        assert!(state.velocity_m_s[2].abs() < 0.05);
        let mut restored = WorldPhysics::new(&[], 0.);
        restored.add_payload(&state).unwrap();
        assert_eq!(restored.payload(4).unwrap(), state);
        assert!(!world.is_idle());
    }
}
