// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Constraint-only motor playback. Gravity and contacts are explicitly disabled.
use super::importer::{Project, quat, transform};
use bevy::math::DVec3;
use rm_sim_physics::{
    StepDuration,
    cooking::CookedAssembly,
    runtime::{AssemblyKeys, BodyKey, JointKey, PhysicalRuntime},
};
use serde::Serialize;
use sim_engine::{Pose, model::*};
use std::collections::BTreeMap;
type Result<T> = std::result::Result<T, String>;
#[derive(Serialize)]
pub struct Reading {
    pub group: u64,
    pub coordinate_unit: &'static str,
    pub velocity_unit: &'static str,
    pub position: f64,
    pub velocity: f64,
    pub target: f64,
    pub anchor_error_m: f64,
}
pub struct Playback {
    model: CookedAssembly,
    keys: AssemblyKeys,
    pub runtime: PhysicalRuntime,
    pub tick: u64,
    pub poses: BTreeMap<u64, Pose>,
    gains: BTreeMap<u64, f64>,
    targets: BTreeMap<u64, f64>,
}
fn generalized_inertia(
    kind: BodyKind,
    pose: Pose,
    pivot: [f64; 3],
    axis: DVec3,
    prismatic: bool,
) -> Option<f64> {
    let BodyKind::Dynamic(i) = kind else {
        return None;
    };
    if prismatic {
        return Some(i.mass_kg());
    }
    let com = transform(pose, i.frame().translation_m);
    let principal = quat(i.frame()).conjugate() * quat(pose).conjugate() * axis;
    let diagonal = DVec3::from_array(i.principal_inertia_kg_m2());
    Some(
        (principal * principal).dot(diagonal)
            + i.mass_kg()
                * (com - DVec3::from_array(pivot))
                    .cross(axis)
                    .length_squared(),
    )
}
impl Playback {
    pub fn new(project: &Project) -> Result<Self> {
        let (source, _) = project.description()?;
        let mut links = Vec::new();
        for link in source.links() {
            let body = link.body();
            let colliders = body
                .colliders()
                .iter()
                .map(|c| {
                    let mut material = c.material();
                    material.filter = 0;
                    ColliderDescription::new(c.shape().clone(), c.pose(), material)?
                        .with_policy(c.policy())
                })
                .collect::<std::result::Result<Vec<_>, _>>()
                .map_err(str::to_owned)?;
            let mut policy = body.policy();
            policy.gravity_scale = 0.;
            let body = RigidBodyDescription::new(body.kind(), colliders)
                .and_then(|b| b.with_policy(policy))
                .map_err(str::to_owned)?;
            links.push(AssemblyLink::new(link.id(), body, link.initial()).map_err(str::to_owned)?);
        }
        let description = AssemblyDescription::new(source.root(), links, source.joints().to_vec())
            .map_err(str::to_owned)?;
        let model = CookedAssembly::new(description).map_err(str::to_owned)?;
        let keys = AssemblyKeys {
            links: model
                .source()
                .links()
                .iter()
                .map(|l| (l.id(), BodyKey(l.id().0)))
                .collect(),
            joints: model
                .source()
                .joints()
                .iter()
                .map(|j| (j.id(), JointKey(j.id().0)))
                .collect(),
        };
        let mut gains = BTreeMap::new();
        for joint in model.source().joints() {
            if joint.description().kind() == JointKind::Fixed {
                continue;
            }
            let child = project
                .groups
                .iter()
                .find(|g| g.id == joint.child().0)
                .unwrap();
            let parent = project
                .groups
                .iter()
                .find(|g| g.id == joint.parent().0)
                .unwrap();
            let j = child.joint.as_ref().unwrap();
            let axis = DVec3::from_array(j.axis).normalize();
            let cb = model
                .source()
                .links()
                .iter()
                .find(|l| l.id() == joint.child())
                .unwrap()
                .body()
                .kind();
            let pb = model
                .source()
                .links()
                .iter()
                .find(|l| l.id() == joint.parent())
                .unwrap()
                .body()
                .kind();
            let prism = joint.description().kind() == JointKind::Prismatic;
            let ci = generalized_inertia(cb, child.pose, j.pivot_m, axis, prism);
            let pi = generalized_inertia(pb, parent.pose, j.pivot_m, axis, prism);
            let inertia = match (ci, pi) {
                (Some(c), Some(p)) => c.min(p) * 0.5,
                (Some(c), None) => c,
                (None, Some(p)) => p,
                _ => return Err("motor requires a dynamic participant".into()),
            };
            if !inertia.is_finite() || inertia <= 0. {
                return Err("invalid generalized motor inertia".into());
            }
            gains.insert(joint.id().0, inertia);
        }
        if gains.is_empty() {
            return Err("no movable reviewed joints in this project".into());
        }
        let mut p = Self {
            model,
            keys,
            runtime: PhysicalRuntime::new(StepDuration::HZ_128),
            tick: 0,
            poses: BTreeMap::new(),
            gains,
            targets: BTreeMap::new(),
        };
        p.reset()?;
        Ok(p)
    }
    pub fn reset(&mut self) -> Result<()> {
        let mut next = PhysicalRuntime::new(StepDuration::HZ_128);
        next.install_assembly(&self.model, Pose::default(), &self.keys)
            .map_err(str::to_owned)?;
        self.runtime = next;
        self.tick = 0;
        self.targets.clear();
        self.refresh()?;
        for id in self.gains.keys().copied().collect::<Vec<_>>() {
            self.target(id, 0.)?;
        }
        Ok(())
    }
    pub fn target_value(&self, group: u64) -> Option<f64> {
        self.targets.get(&group).copied()
    }
    pub fn target(&mut self, group: u64, coordinate: f64) -> Result<()> {
        let binding = self
            .runtime
            .joint_binding(JointKey(group))
            .ok_or("selected group has no live joint")?;
        let joint = self
            .model
            .source()
            .joints()
            .iter()
            .find(|j| j.id().0 == group)
            .ok_or("missing joint")?;
        let desc = joint.description();
        if !coordinate.is_finite()
            || desc
                .limits()
                .is_some_and(|l| coordinate < l.min() || coordinate > l.max())
            || (desc.kind() == JointKind::Revolute && coordinate.abs() > std::f64::consts::PI)
        {
            return Err("target outside bounded single-turn playback or reviewed limits".into());
        }
        let scale = *self
            .gains
            .get(&group)
            .ok_or("fixed joint cannot be motorized")?;
        // Demonstration gains: natural frequency 8/s, critically damped, capped
        // at 30 units/s² of generalized acceleration. Not robot motor calibration.
        let motor = JointMotor::new(coordinate, 0., scale * 64., scale * 16., scale * 30.)
            .map_err(str::to_owned)?;
        self.runtime
            .set_joint_motor(binding, Some(motor))
            .map_err(str::to_owned)?;
        self.targets.insert(group, coordinate);
        Ok(())
    }
    pub fn advance(&mut self, count: u32) -> Result<()> {
        if count > 32 {
            return Err("bounded playback accepts at most 32 ticks per update".into());
        }
        let end = self
            .tick
            .checked_add(u64::from(count))
            .ok_or("playback clock overflow")?;
        end.checked_mul(StepDuration::HZ_128.as_ns())
            .ok_or("playback time overflow")?;
        while self.tick < end {
            self.runtime
                .step(self.tick * StepDuration::HZ_128.as_ns())
                .map_err(str::to_owned)?;
            self.tick += 1;
        }
        self.refresh()
    }
    fn refresh(&mut self) -> Result<()> {
        for &(id, key) in &self.keys.links {
            let binding = self
                .runtime
                .body_binding(key)
                .ok_or("missing installed body")?;
            self.poses.insert(
                id.0,
                self.runtime
                    .body_state(binding)
                    .map_err(str::to_owned)?
                    .pose,
            );
        }
        Ok(())
    }
    pub fn readings(&self) -> Result<Vec<Reading>> {
        self.model
            .source()
            .joints()
            .iter()
            .map(|j| {
                let binding = self
                    .runtime
                    .joint_binding(JointKey(j.id().0))
                    .ok_or("missing live joint")?;
                let state = self.runtime.joint_state(binding).map_err(str::to_owned)?;
                let a = transform(
                    self.poses[&j.parent().0],
                    j.description().parent_frame().translation_m,
                );
                let b = transform(
                    self.poses[&j.child().0],
                    j.description().child_frame().translation_m,
                );
                // Prismatic anchors intentionally separate along the free axis.
                let error = if j.description().kind() == JointKind::Prismatic {
                    let axis = quat(self.poses[&j.parent().0])
                        * quat(j.description().parent_frame())
                        * DVec3::X;
                    ((b - a) - axis * axis.dot(b - a)).length()
                } else {
                    (b - a).length()
                };
                Ok(Reading {
                    group: j.id().0,
                    coordinate_unit: match j.description().kind() {
                        JointKind::Revolute => "rad",
                        JointKind::Prismatic => "m",
                        JointKind::Fixed => "none",
                    },
                    velocity_unit: match j.description().kind() {
                        JointKind::Revolute => "rad/s",
                        JointKind::Prismatic => "m/s",
                        JointKind::Fixed => "none",
                    },
                    position: state.position,
                    velocity: state.velocity,
                    target: *self.targets.get(&j.id().0).unwrap_or(&0.),
                    anchor_error_m: error,
                })
            })
            .collect()
    }
}
#[cfg(test)]
pub(super) mod tests {
    use super::super::importer::{Joint, Motion};
    use super::*;
    pub fn source() -> Project {
        let mut p = super::super::importer::tests::shared_fixture();
        p.parts[1].group = 2;
        let mut g = p.groups[0].clone();
        g.id = 2;
        g.fixed = false;
        g.pose = p.parts[1].pose;
        g.joint = Some(Joint {
            parent: 1,
            motion: Motion::Revolute,
            pivot_m: [0., 0., 0.],
            axis: [0., 0., 1.],
            limits: Some([-1., 1.]),
            reviewed: true,
            evidence: "explicit test fixture".into(),
        });
        p.groups.push(g);
        p
    }
    #[test]
    fn physical_tracking_reversal_reset_and_local_geometry_preservation() {
        let p = source();
        let mut sim = Playback::new(&p).unwrap();
        let before = serde_json::to_string(&p).unwrap();
        let old = sim.runtime.body_binding(BodyKey(2)).unwrap();
        for target in [0.6, -0.4] {
            sim.target(2, target).unwrap();
            for _ in 0..128 {
                sim.advance(4).unwrap();
            }
            let r = sim.readings().unwrap();
            assert!((r[0].position - target).abs() < 0.003, "{}", r[0].position);
            assert!(r[0].anchor_error_m < 0.0001);
        }
        assert!(sim.target(2, 2.).is_err());
        assert!(sim.advance(33).is_err());
        sim.reset().unwrap();
        assert!(sim.runtime.body_state(old).is_err());
        assert_eq!(sim.tick, 0);
        sim.target(2, 0.6).unwrap();
        for _ in 0..128 {
            sim.advance(4).unwrap();
        }
        let first = sim.poses[&2];
        sim.reset().unwrap();
        sim.target(2, 0.6).unwrap();
        for _ in 0..128 {
            sim.advance(4).unwrap();
        }
        assert_eq!(first, sim.poses[&2]);
        assert_eq!(before, serde_json::to_string(&p).unwrap());
    }
}
