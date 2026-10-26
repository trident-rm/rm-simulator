// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Cache complete translation endpoints; keep the original meshes during travel.
use super::*;
use crate::motion::{Mechanism, MotionChannel, MotionFrame};

#[derive(Clone)]
pub(crate) struct CapturedBake {
    pub fixed: usize,
    pub moving: Vec<usize>,
    pub endpoints: [SharedShape; 2],
    pub kind: Mechanism,
    pub team: Team,
    pub fraction: f64,
}

pub(super) struct EndpointBake {
    pub fixed: ColliderHandle,
    pub moving: Vec<ColliderHandle>,
    pub cached: ColliderHandle,
    pub endpoints: [SharedShape; 2],
    pub kind: Mechanism,
    pub team: Team,
    pub fraction: f64,
}

impl WorldPhysics {
    /// Cache two whole-assembly meshes for a bounded legacy translation channel.
    /// `fixed` identifies its complete stationary body. All channel meshes must
    /// have identical collision groups and finite translation frames. Cooking
    /// precedes mutation. At exact endpoints only one complete mesh is enabled;
    /// during travel only the fixed body and original moving meshes are enabled.
    /// This leaves target/scoring bodies untouched and preserves capture/reset.
    pub fn bake_translation_endpoints(
        &mut self,
        fixed: ColliderHandle,
        kind: Mechanism,
        team: Team,
    ) -> Result<(), &'static str> {
        let channel = MotionChannel::Legacy { kind, team };
        let parts: Vec<_> = self
            .mechanisms
            .iter()
            .filter(|(_, c, _)| *c == channel)
            .copied()
            .collect();
        if parts.is_empty()
            || parts
                .iter()
                .any(|(_, _, f)| !matches!(f, MotionFrame::Translation { .. }))
        {
            return Err("endpoint bake requires bounded translation meshes");
        }
        let colliders = &self.physical.world().colliders;
        let fixed_collider = colliders
            .get(fixed)
            .ok_or("missing endpoint fixed collider")?;
        if self.endpoint_bakes.iter().any(|b| {
            b.fixed == fixed || b.moving.contains(&fixed) || (b.kind == kind && b.team == team)
        }) || self.mechanisms.iter().any(|(h, ..)| *h == fixed)
        {
            return Err("overlapping endpoint bake");
        }
        let starts_closed = parts
            .iter()
            .all(|(h, _, f)| *colliders[*h].position() == pose(f.pose(0.)));
        let starts_open = parts
            .iter()
            .all(|(h, _, f)| *colliders[*h].position() == pose(f.pose(1.)));
        if !starts_closed && !starts_open {
            return Err("install endpoint bake at an exact endpoint");
        }
        let groups = fixed_collider.collision_groups();
        let mut endpoints = Vec::new();
        for endpoint in [0., 1.] {
            let mut points = Vec::new();
            let mut triangles = Vec::new();
            for (handle, transform) in std::iter::once((fixed, *fixed_collider.position())).chain(
                parts
                    .iter()
                    .map(|(h, _, frame)| (*h, pose(frame.pose(endpoint)))),
            ) {
                let collider = &colliders[handle];
                if collider.collision_groups() != groups || collider.parent().is_some() {
                    return Err("endpoint bake requires matching standalone scenery groups");
                }
                let mesh = collider
                    .shape()
                    .as_trimesh()
                    .ok_or("endpoint bake requires triangle meshes")?;
                let offset = points.len() as u32;
                points.extend(mesh.vertices().iter().map(|p| (transform * *p).to_array()));
                triangles.extend(mesh.indices().iter().map(|t| t.map(|i| i + offset)));
            }
            endpoints.push(crate::cooking::cook_buffers(points, triangles)?);
        }
        let endpoints: [SharedShape; 2] =
            endpoints.try_into().map_err(|_| "missing endpoint mesh")?;
        let fraction = if starts_open { 1. } else { 0. };
        let cached = self.physical.extension_insert_scenery(
            endpoints[fraction as usize].clone(),
            Default::default(),
            FRICTION,
            RESTITUTION,
            groups,
        );
        self.endpoint_bakes.push(EndpointBake {
            fixed,
            moving: parts.iter().map(|(h, ..)| *h).collect(),
            cached,
            endpoints,
            kind,
            team,
            fraction: f64::NAN,
        });
        self.sync_endpoint_bakes_for(kind, team, fraction);
        Ok(())
    }

    pub(super) fn sync_endpoint_bakes_for(&mut self, kind: Mechanism, team: Team, fraction: f64) {
        for bake in &mut self.endpoint_bakes {
            if bake.kind != kind || bake.team != team || bake.fraction == fraction {
                continue;
            }
            let endpoint = if fraction == 0. {
                Some(0)
            } else if fraction == 1. {
                Some(1)
            } else {
                None
            };
            let colliders = &mut self.physical.world_mut().colliders;
            // This batch finishes before any query/step; never expose both forms.
            colliders[bake.cached].set_enabled(false);
            colliders[bake.fixed].set_enabled(endpoint.is_none());
            for handle in &bake.moving {
                colliders[*handle].set_enabled(endpoint.is_none());
            }
            if let Some(index) = endpoint {
                colliders[bake.cached].set_shape(bake.endpoints[index].clone());
                colliders[bake.cached].set_enabled(true);
            }
            bake.fraction = fraction;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world() -> WorldPhysics {
        let mut w = WorldPhysics::new(&[], -10.);
        w.remove_catch_floor();
        let vertices = vec![[2., -1., 0.], [2., 1., 0.], [2., 0., 2.]];
        let fixed = w
            .add_static_mesh(
                vertices.iter().map(|p| [p[0], p[1] + 3., p[2]]).collect(),
                vec![[0, 1, 2]],
            )
            .unwrap();
        w.add_mechanism_mesh(
            vertices,
            vec![[0, 1, 2]],
            Mechanism::Base,
            Team::Red,
            [[0.; 3], [1., 0., 0.]],
        )
        .unwrap();
        w.bake_translation_endpoints(fixed, Mechanism::Base, Team::Red)
            .unwrap();
        w
    }

    #[test]
    fn endpoints_enable_one_whole_mesh_and_transition_uses_sources() {
        let mut w = world();
        for (fraction, count) in [(0., 1), (0.5, 2), (1., 1), (0.5, 2), (0., 1)] {
            let state = crate::motion::MechanismState {
                base_open_fraction: [fraction, 0.],
                ..Default::default()
            };
            w.sync_mechanisms(&state);
            assert_eq!(
                w.physical
                    .world()
                    .all_colliders()
                    .filter(|(_, c)| c.is_enabled())
                    .count(),
                count
            );
            let capture = w.static_geometry_snapshot(true);
            let hit = capture
                .cast_ray([0., 0., 1.], [1., 0., 0.], 5.)
                .unwrap()
                .unwrap();
            assert_eq!(hit.distance_m, 2. + fraction);
            let ray = Ray::new(Vector::new(0., 0., 1.), Vector::X);
            let live_hit = w
                .physical
                .world()
                .all_colliders()
                .filter(|(_, c)| c.is_enabled())
                .filter_map(|(_, c)| c.shape().cast_ray(c.position(), &ray, 5., true))
                .min_by(f64::total_cmp)
                .unwrap();
            assert_eq!(live_hit, 2. + fraction);
            let mut restored = WorldPhysics::new(&[], -10.);
            restored.try_insert_geometry(&capture).unwrap();
            assert_eq!(
                restored
                    .physical
                    .world()
                    .all_colliders()
                    .filter(|(_, c)| c.is_enabled())
                    .count(),
                count
            );
            restored.sync_mechanisms(&state);
            assert_eq!(
                restored
                    .physical
                    .world()
                    .all_colliders()
                    .filter(|(_, c)| c.is_enabled())
                    .count(),
                count
            );
            assert_eq!(restored.endpoint_bakes.len(), 1);
        }
    }

    #[test]
    fn masked_groups_and_interpolated_capture_survive_import() {
        let mut w = world();
        let groups =
            InteractionGroups::new(Group::GROUP_3, Group::GROUP_2, InteractionTestMode::And);
        for (_, collider) in w.physical.world_mut().colliders.iter_mut() {
            collider.set_collision_groups(groups);
        }
        let closed = w.static_geometry_snapshot(true);
        let mut without_bake = closed.clone();
        without_bake.endpoint_bakes.clear();
        assert!(closed.interpolate(&without_bake, 0.5).is_err());
        let open = closed.at_mechanisms(&crate::motion::MechanismState {
            base_open_fraction: [1., 0.],
            ..Default::default()
        });
        let midpoint = closed.interpolate(&open, 0.5).unwrap();
        let mut restored = WorldPhysics::new(&[], -10.);
        restored.try_insert_geometry(&midpoint).unwrap();
        assert_eq!(restored.endpoint_bakes[0].fraction, 0.5);
        assert_eq!(
            restored
                .physical
                .world()
                .all_colliders()
                .filter(|(_, c)| c.is_enabled())
                .count(),
            2
        );
        assert!(
            restored
                .physical
                .world()
                .all_colliders()
                .all(|(_, c)| c.collision_groups() == groups)
        );
    }

    #[test]
    fn ccd_stops_projectiles_at_closed_open_and_travelling_meshes() {
        for fraction in [0., 0.5, 1.] {
            let mut w = world();
            w.sync_mechanisms(&crate::motion::MechanismState {
                base_open_fraction: [fraction, 0.],
                ..Default::default()
            });
            w.fire(
                0,
                Pose::at([0., 0., 1.]),
                Shot {
                    caliber: Caliber::Mm17,
                    speed_m_s: 25.,
                },
                None,
            )
            .unwrap();
            let frames = TargetFrames::new(vec![]);
            let mut contact = false;
            for tick in 0..40 {
                w.step(tick * crate::tick_ns(), &frames).unwrap();
                for ball in w.snapshot() {
                    assert!(ball.position_m[0] < 2. + fraction + 0.02);
                    contact |= ball.first_contact_ns.is_some();
                }
            }
            assert!(contact);
        }
    }
}
