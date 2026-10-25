// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Candidate constraints from relative saved poses, not SolidWorks mate decoding.
#[path = "recovery/mounts.rs"]
pub mod mounts;
use super::importer::{Joint, Motion, Project, compose, inverse, quat, transform};
use bevy::math::DVec3;
use serde::{Deserialize, Serialize};
use sim_engine::Pose;
use std::{collections::BTreeSet, io::Read, path::Path};
type Result<T> = std::result::Result<T, String>;
const LINEAR_TOL_M: f64 = 1e-5;
const ANGULAR_TOL_RAD: f64 = 1e-6;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sample {
    pub parent: Pose,
    pub child: Pose,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Observation {
    pub parent: u64,
    pub child: u64,
    pub states: [Sample; 2],
    pub evidence: String,
    /// Optional independently selected mounting center in parent-local metres.
    #[serde(default)]
    pub mount_point_parent_m: Option<[f64; 3]>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Observations {
    pub schema: String,
    pub motions: Vec<Observation>,
}
fn load_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T> {
    let file = std::fs::File::open(path).map_err(|e| e.to_string())?;
    let mut bytes = Vec::new();
    file.take(1024 * 1024 + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() > 1024 * 1024 {
        return Err("observation table exceeds 1 MiB".into());
    }
    serde_json::from_slice(&bytes).map_err(|e| e.to_string())
}
impl Observations {
    pub fn load(path: &Path) -> Result<Self> {
        load_json(path)
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    Unexcited,
    Revolute,
    Prismatic,
    ScrewLike,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct PartIdentity {
    id: u64,
    sha256: String,
    configuration: String,
    pose: Pose,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Candidate {
    pub parent: u64,
    pub child: u64,
    pub kind: Kind,
    /// A point on the recovered line, nearest the parent's initial origin.
    pub pivot_parent_m: [f64; 3],
    pub axis_parent: [f64; 3],
    /// Observed radians (rotation) or metres (translation), not a hardware limit.
    pub observed_coordinate: f64,
    pub axial_advance_m: f64,
    pub mount_reference_parent_m: Option<[f64; 3]>,
    pub fit_translation_error_m: f64,
    pub fit_rotation_error_rad: f64,
    pub evidence: String,
    initial: Sample,
    parts: Vec<PartIdentity>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Report {
    pub schema: String,
    pub candidates: Vec<Candidate>,
    /// Original measurements remain available for auditing the inferred frames.
    #[serde(default)]
    pub observations: Vec<Observation>,
}
fn valid_pose(p: Pose) -> bool {
    DVec3::from_array(p.translation_m).is_finite()
        && quat(p).is_finite()
        && (quat(p).length_squared() - 1.).abs() < 1e-6
}
fn near(a: Pose, b: Pose) -> bool {
    (DVec3::from_array(a.translation_m) - DVec3::from_array(b.translation_m)).length() < 1e-9
        && quat(a).angle_between(quat(b)) < 1e-8
}
fn identities(project: &Project, parent: u64, child: u64) -> Vec<PartIdentity> {
    project
        .parts
        .iter()
        .filter(|p| p.group == parent || p.group == child)
        .map(|p| PartIdentity {
            id: p.id,
            sha256: p.source_sha256.clone(),
            configuration: p.configuration.clone(),
            pose: p.pose,
        })
        .collect()
}
impl Candidate {
    fn current(&self, p: &Project) -> bool {
        let parent = p.groups.iter().find(|g| g.id == self.parent);
        let child = p.groups.iter().find(|g| g.id == self.child);
        parent.zip(child).is_some_and(|(a, b)| {
            near(a.pose, self.initial.parent) && near(b.pose, self.initial.child)
        }) && self.parts == identities(p, self.parent, self.child)
    }
    pub fn apply(&self, project: &mut Project) -> Result<()> {
        if !self.current(project) {
            return Err(
                "recovery proposal is stale: grouping, source identity or initial pose changed"
                    .into(),
            );
        }
        if !DVec3::from_array(self.axis_parent).is_finite()
            || (DVec3::from_array(self.axis_parent).length_squared() - 1.).abs() > 1e-6
            || !DVec3::from_array(self.pivot_parent_m).is_finite()
            || !self.observed_coordinate.is_finite()
        {
            return Err("invalid candidate axis, pivot or coordinate".into());
        }
        let motion = match self.kind {
            Kind::Revolute => Motion::Revolute,
            Kind::Prismatic => Motion::Prismatic,
            _ => return Err(
                "unexcited or screw-like motion does not identify a supported single-axis constraint"
                    .into(),
            ),
        };
        let mut ancestor = Some(self.parent);
        let mut seen = BTreeSet::new();
        while let Some(id) = ancestor {
            if id == self.child || !seen.insert(id) {
                return Err("candidate parent creates a joint cycle".into());
            }
            ancestor = project
                .groups
                .iter()
                .find(|g| g.id == id)
                .and_then(|g| g.joint.as_ref().map(|j| j.parent));
        }
        let pivot = transform(self.initial.parent, self.pivot_parent_m).to_array();
        let axis = (quat(self.initial.parent) * DVec3::from_array(self.axis_parent)).to_array();
        let group = project
            .groups
            .iter_mut()
            .find(|g| g.id == self.child)
            .unwrap();
        group.joint = Some(Joint {
            parent: self.parent,
            motion,
            pivot_m: pivot,
            axis,
            limits: None,
            reviewed: false,
            evidence: format!(
                "Inferred from two relative saved poses, not a decoded SolidWorks mate. Anchor is either the axis-line gauge or the projection of a caller-supplied mount center; mounting geometry needs explicit review. No travel stops inferred. {}",
                self.evidence
            ),
        });
        Ok(())
    }
    pub fn observed_target(&self, project: &Project) -> Option<f64> {
        if !self.current(project) {
            return None;
        }
        let j = project
            .groups
            .iter()
            .find(|g| g.id == self.child)?
            .joint
            .as_ref()?;
        let pivot = transform(self.initial.parent, self.pivot_parent_m);
        let axis = quat(self.initial.parent) * DVec3::from_array(self.axis_parent);
        let kind = matches!(
            (self.kind, j.motion),
            (Kind::Revolute, Motion::Revolute) | (Kind::Prismatic, Motion::Prismatic)
        );
        (kind
            && j.parent == self.parent
            && (DVec3::from_array(j.pivot_m) - pivot).length() < 1e-8
            && (DVec3::from_array(j.axis).normalize() - axis).length() < 1e-8)
            .then_some(self.observed_coordinate)
    }
}
impl Report {
    pub fn recover(project: &Project, input: &Observations) -> Result<Self> {
        project.validate()?;
        if input.schema != "rm-relative-pose-observations-v1"
            || input.motions.is_empty()
            || input.motions.len() > 256
        {
            return Err("invalid or unbounded pose observation table".into());
        }
        let mut seen = BTreeSet::new();
        let mut candidates = Vec::new();
        for o in &input.motions {
            if o.parent == o.child || !seen.insert(o.child) || o.evidence.trim().is_empty() {
                return Err("duplicate/self motion or missing observation provenance".into());
            }
            let parent = project
                .groups
                .iter()
                .find(|g| g.id == o.parent)
                .ok_or("missing parent group")?;
            let child = project
                .groups
                .iter()
                .find(|g| g.id == o.child)
                .ok_or("missing child group")?;
            if !o
                .states
                .iter()
                .all(|s| valid_pose(s.parent) && valid_pose(s.child))
                || !near(parent.pose, o.states[0].parent)
                || !near(child.pose, o.states[0].child)
            {
                return Err(
                    "invalid poses or first observation does not match the reviewed source frames"
                        .into(),
                );
            }
            let a = compose(inverse(o.states[0].parent), o.states[0].child);
            let b = compose(inverse(o.states[1].parent), o.states[1].child);
            let d = compose(b, inverse(a));
            let mut q = quat(d);
            if q.w < 0. {
                q = -q;
            }
            let vector = DVec3::new(q.x, q.y, q.z);
            let angle = 2. * vector.length().atan2(q.w);
            let t = DVec3::from_array(d.translation_m);
            let (kind, axis, mut pivot, coordinate, advance) = if angle <= ANGULAR_TOL_RAD {
                if t.length() <= LINEAR_TOL_M {
                    (Kind::Unexcited, DVec3::X, DVec3::ZERO, 0., 0.)
                } else {
                    (
                        Kind::Prismatic,
                        t.normalize(),
                        DVec3::ZERO,
                        t.length(),
                        t.length(),
                    )
                }
            } else {
                if angle < 1e-3 {
                    return Err("rotation too small to condition a mounting-axis estimate".into());
                }
                let axis = vector.normalize();
                let advance = axis.dot(t);
                let perpendicular = t - axis * advance;
                let pivot = (perpendicular + axis.cross(perpendicular) / (angle * 0.5).tan()) * 0.5;
                (
                    if advance.abs() <= LINEAR_TOL_M {
                        Kind::Revolute
                    } else {
                        Kind::ScrewLike
                    },
                    axis,
                    pivot,
                    angle,
                    advance,
                )
            };
            if let Some(reference) = o.mount_point_parent_m {
                let reference = DVec3::from_array(reference);
                if !reference.is_finite() {
                    return Err("nonfinite mount geometry reference".into());
                }
                if matches!(kind, Kind::Revolute | Kind::ScrewLike) {
                    if (reference - pivot).cross(axis).length() > LINEAR_TOL_M {
                        return Err(
                            "mount geometry center does not lie on the inferred axis".into()
                        );
                    }
                    pivot += axis * (reference - pivot).dot(axis);
                } else if kind == Kind::Prismatic {
                    pivot = reference;
                }
            }
            let predicted = match kind {
                Kind::Unexcited => a,
                Kind::Prismatic => compose(Pose::at((axis * coordinate).to_array()), a),
                Kind::Revolute | Kind::ScrewLike => {
                    let r = bevy::math::DQuat::from_axis_angle(axis, coordinate);
                    compose(
                        Pose {
                            translation_m: (pivot - r * pivot
                                + if kind == Kind::ScrewLike {
                                    axis * advance
                                } else {
                                    DVec3::ZERO
                                })
                            .to_array(),
                            rotation_wxyz: [r.w, r.x, r.y, r.z],
                        },
                        a,
                    )
                }
            };
            let fit_translation_error_m = (DVec3::from_array(predicted.translation_m)
                - DVec3::from_array(b.translation_m))
            .length();
            let fit_rotation_error_rad = quat(predicted).angle_between(quat(b));
            if !pivot.is_finite()
                || !axis.is_finite()
                || fit_translation_error_m > LINEAR_TOL_M * 1.01
                || fit_rotation_error_rad > ANGULAR_TOL_RAD * 1.01
            {
                return Err("relative-motion reconstruction failed".into());
            }
            candidates.push(Candidate {
                parent: o.parent,
                child: o.child,
                kind,
                pivot_parent_m: pivot.to_array(),
                axis_parent: axis.to_array(),
                observed_coordinate: coordinate,
                axial_advance_m: advance,
                mount_reference_parent_m: o.mount_point_parent_m,
                fit_translation_error_m,
                fit_rotation_error_rad,
                evidence: o.evidence.clone(),
                initial: o.states[0].clone(),
                parts: identities(project, o.parent, o.child),
            });
        }
        Ok(Self {
            schema: "rm-inferred-joint-proposals-v1".into(),
            candidates,
            observations: input.motions.clone(),
        })
    }
    /// Apply supported proposals to a clone, leaving all inferred joints unreviewed.
    pub fn apply(&self, project: &mut Project) -> Result<()> {
        let mut next = project.clone();
        for c in &self.candidates {
            if matches!(c.kind, Kind::Revolute | Kind::Prismatic) {
                c.apply(&mut next)?;
            }
        }
        next.provenance.insert(
            "joint_recovery".into(),
            serde_json::to_string(self)
                .map_err(|e| e.to_string())?
                .into(),
        );
        next.validate()?;
        *project = next;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    pub(super) fn fixture() -> Project {
        let mut p = super::super::importer::tests::shared_fixture();
        p.parts[1].group = 2;
        let mut g = p.groups[0].clone();
        g.id = 2;
        g.fixed = false;
        g.pose = p.parts[1].pose;
        g.joint = None;
        p.groups.push(g);
        p
    }
    fn input(p: &Project, delta: Pose) -> Observations {
        let a = Sample {
            parent: p.groups[0].pose,
            child: p.groups[1].pose,
        };
        Observations {
            schema: "rm-relative-pose-observations-v1".into(),
            motions: vec![Observation {
                parent: 1,
                child: 2,
                states: [
                    a.clone(),
                    Sample {
                        parent: a.parent,
                        child: compose(delta, a.child),
                    },
                ],
                evidence: "synthetic analytic fixture".into(),
                mount_point_parent_m: None,
            }],
        }
    }
    #[test]
    fn hinge_recovers_axis_line_and_preserves_initial_geometry() {
        let mut p = fixture();
        let before = p.parts[1].pose;
        let pivot = DVec3::new(0.4, -0.2, 0.);
        let q = bevy::math::DQuat::from_rotation_z(0.7);
        let delta = Pose {
            translation_m: (pivot - q * pivot).to_array(),
            rotation_wxyz: [q.w, q.x, q.y, q.z],
        };
        let report = Report::recover(&p, &input(&p, delta)).unwrap();
        let c = &report.candidates[0];
        assert_eq!(c.kind, Kind::Revolute);
        assert!((DVec3::from_array(c.pivot_parent_m) - pivot).length() < 1e-10);
        assert!((c.observed_coordinate - 0.7).abs() < 1e-10);
        report.apply(&mut p).unwrap();
        assert_eq!(p.parts[1].pose, before);
        assert!(!p.groups[1].joint.as_ref().unwrap().reviewed);
        assert!(p.assembly().is_err());
        assert!(p.groups[1].joint.as_ref().unwrap().limits.is_none());
        p.groups[1].joint.as_mut().unwrap().reviewed = true;
        assert!(p.assembly().is_ok());
        p.parts[1].source_sha256 = "changed".into();
        let before = serde_json::to_string(&p).unwrap();
        assert!(report.apply(&mut p).is_err());
        assert_eq!(before, serde_json::to_string(&p).unwrap());
    }
    #[test]
    fn mount_center_resolves_axis_gauge_and_rejects_off_axis_geometry() {
        let p = fixture();
        let pivot = DVec3::new(0.4, -0.2, 0.);
        let q = bevy::math::DQuat::from_rotation_z(std::f64::consts::PI);
        let delta = Pose {
            translation_m: (pivot - q * pivot).to_array(),
            rotation_wxyz: [q.w, q.x, q.y, q.z],
        };
        let mut i = input(&p, delta);
        i.motions[0].mount_point_parent_m = Some([0.4, -0.2, 0.3]);
        let r = Report::recover(&p, &i).unwrap();
        let c = &r.candidates[0];
        assert!((DVec3::from_array(c.pivot_parent_m) - DVec3::new(0.4, -0.2, 0.3)).length() < 1e-9);
        assert!((c.observed_coordinate - std::f64::consts::PI).abs() < 1e-12);
        i.motions[0].mount_point_parent_m = Some([0.5, -0.2, 0.3]);
        assert!(Report::recover(&p, &i).is_err());
    }

    #[test]
    fn moving_parent_is_removed_and_unexcited_or_screw_is_not_welded() {
        let p = fixture();
        let mut i = input(&p, Pose::at([0.02, 0., 0.]));
        let r = bevy::math::DQuat::from_rotation_y(0.4);
        let common = Pose {
            translation_m: [2., 3., 4.],
            rotation_wxyz: [r.w, r.x, r.y, r.z],
        };
        i.motions[0].states[1].parent = compose(common, i.motions[0].states[1].parent);
        i.motions[0].states[1].child = compose(common, i.motions[0].states[1].child);
        let c = Report::recover(&p, &i).unwrap().candidates.remove(0);
        assert_eq!(c.kind, Kind::Prismatic);
        assert!((c.observed_coordinate - 0.02).abs() < 1e-12);
        let q = bevy::math::DQuat::from_rotation_z(0.8);
        let report = Report::recover(
            &p,
            &input(
                &p,
                Pose {
                    translation_m: [0., 0., 0.03],
                    rotation_wxyz: [q.w, q.x, q.y, q.z],
                },
            ),
        )
        .unwrap();
        assert_eq!(report.candidates[0].kind, Kind::ScrewLike);
        let mut copy = p.clone();
        report.apply(&mut copy).unwrap();
        assert!(copy.groups[1].joint.is_none());
        assert_eq!(
            Report::recover(&p, &input(&p, Pose::default()))
                .unwrap()
                .candidates[0]
                .kind,
            Kind::Unexcited
        );
    }
    #[test]
    fn rejects_invalid_observations_and_cyclic_application() {
        let mut p = fixture();
        let mut i = input(&p, Pose::at([0.1, 0., 0.]));
        i.motions[0].states[1].child.rotation_wxyz = [0.; 4];
        assert!(Report::recover(&p, &i).is_err());
        let tiny = bevy::math::DQuat::from_rotation_z(1e-4);
        assert!(
            Report::recover(
                &p,
                &input(
                    &p,
                    Pose {
                        translation_m: [0.; 3],
                        rotation_wxyz: [tiny.w, tiny.x, tiny.y, tiny.z]
                    }
                )
            )
            .is_err()
        );
        let r = Report::recover(&p, &input(&p, Pose::at([0.1, 0., 0.]))).unwrap();
        p.groups[0].joint = Some(Joint {
            parent: 2,
            motion: Motion::Fixed,
            pivot_m: [0.; 3],
            axis: [1., 0., 0.],
            limits: None,
            reviewed: true,
            evidence: "test".into(),
        });
        assert!(r.apply(&mut p).is_err());
    }
}
