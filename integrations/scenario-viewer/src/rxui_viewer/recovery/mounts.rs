// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Review-only rigid placement from two end centers and a transverse pin axis.
use super::super::importer::{Project, compose, inverse, quat};
use super::{PartIdentity, Result, identities, near};
use bevy::math::{DMat3, DQuat, DVec3};
use rm_sim_engine::Pose;
use serde::{Deserialize, Serialize};
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Observation {
    pub schema: String,
    pub group: u64,
    pub source_centers_group_m: [[f64; 3]; 2],
    pub source_pin_axis_group: [f64; 3],
    pub target_centers_m: [[f64; 3]; 2],
    pub target_pin_axis: [f64; 3],
    pub evidence: String,
}
impl Observation {
    pub fn load(path: &std::path::Path) -> Result<Self> {
        super::load_json(path)
    }
}
#[derive(Serialize)]
pub struct Alignment {
    pub group: u64,
    pub initial_pose: Pose,
    pub fitted_pose: Pose,
    pub center_error_m: f64,
    pub pin_axis_error_rad: f64,
    pub evidence: String,
    pub mount_observation: Observation,
    members: Vec<PartIdentity>,
}
impl Alignment {
    pub fn fit(project: &Project, o: &Observation) -> Result<Self> {
        project.validate()?;
        if o.schema != "rm-mount-alignment-observation-v1" || o.evidence.trim().is_empty() {
            return Err("invalid mount observation or missing geometry provenance".into());
        }
        let group = project
            .groups
            .iter()
            .find(|g| g.id == o.group)
            .ok_or("missing mount group")?;
        let [a, b] = o.source_centers_group_m.map(DVec3::from_array);
        let [x, y] = o.target_centers_m.map(DVec3::from_array);
        let pin = DVec3::from_array(o.source_pin_axis_group);
        let target_pin = DVec3::from_array(o.target_pin_axis);
        if [a, b, x, y, pin, target_pin].iter().any(|v| !v.is_finite())
            || pin.length() < 1e-10
            || target_pin.length() < 1e-10
        {
            return Err("invalid mount centers or axes".into());
        }
        let length = (b - a).length();
        let target_length = (y - x).length();
        if length < 1e-5 || target_length < 1e-5 || (length - target_length).abs() > 1e-5 {
            return Err("end-center distances disagree; mounting fit never scales geometry".into());
        }
        let u = (b - a) / length;
        let v = (y - x) / target_length;
        let pin = pin.normalize();
        let target_pin = target_pin.normalize();
        if pin.dot(u).abs() > 1e-3 || target_pin.dot(v).abs() > 1e-3 {
            return Err("mount fit expects transverse, parallel pin geometry".into());
        }
        let p = (pin - u * pin.dot(u)).normalize();
        let q = (target_pin - v * target_pin.dot(v)).normalize();
        let source = DMat3::from_cols(u, p, u.cross(p));
        let mut best = None;
        let mut angle = f64::INFINITY;
        for sign in [-1., 1.] {
            let q = q * sign;
            let target = DMat3::from_cols(v, q, v.cross(q));
            let rotation = DQuat::from_mat3(&(target * source.transpose())).normalize();
            let change = quat(group.pose).normalize().angle_between(rotation);
            if change < angle {
                angle = change;
                best = Some(rotation);
            }
        }
        let rotation = best.ok_or("mount orientation cannot be resolved")?;
        let translation = x - rotation * a;
        let center_error_m = (rotation * b + translation - y).length();
        let pin_axis_error_rad = (rotation * pin).dot(target_pin).abs().clamp(-1., 1.).acos();
        if !translation.is_finite() || center_error_m > 1e-5 || pin_axis_error_rad > 1e-3 {
            return Err("mount closure failed".into());
        }
        Ok(Self {
            group: o.group,
            initial_pose: group.pose,
            fitted_pose: Pose {
                translation_m: translation.to_array(),
                rotation_wxyz: [rotation.w, rotation.x, rotation.y, rotation.z],
            },
            center_error_m,
            pin_axis_error_rad,
            evidence: o.evidence.clone(),
            mount_observation: o.clone(),
            members: identities(project, o.group, o.group),
        })
    }
    /// Move a single rigid leaf group, preserving meshes and invalidating review.
    pub fn apply(&self, project: &mut Project) -> Result<()> {
        let group = project
            .groups
            .iter()
            .find(|g| g.id == self.group)
            .ok_or("missing mount group")?;
        if !near(group.pose, self.initial_pose)
            || self.members != identities(project, self.group, self.group)
        {
            return Err("mount fit is stale".into());
        }
        if project
            .groups
            .iter()
            .any(|g| g.joint.as_ref().is_some_and(|j| j.parent == self.group))
        {
            return Err(
                "mount fitting requires a rigid leaf group; review child boundaries first".into(),
            );
        }
        let delta = compose(self.fitted_pose, inverse(self.initial_pose));
        let mut next = project.clone();
        for p in &mut next.parts {
            if p.group == self.group {
                p.pose = compose(delta, p.pose);
            }
        }
        let group = next.groups.iter_mut().find(|g| g.id == self.group).unwrap();
        group.pose = self.fitted_pose;
        if let Some(j) = &mut group.joint {
            j.reviewed = false;
            j.evidence = format!(
                "Placement changed by geometry fit; re-check this joint. {}",
                j.evidence
            );
        }
        next.source_frames_and_geometry_reviewed = false;
        next.provenance.insert(
            format!("mount_alignment:{}", self.group),
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
    #[test]
    fn end_mount_fit_preserves_scale_and_rejects_stale_or_incompatible_anchors() {
        let mut p = super::super::tests::fixture();
        let raw = p.clone();
        let before_mass = raw.mass(&raw.groups[1]).unwrap().0;
        let o = Observation {
            schema: "rm-mount-alignment-observation-v1".into(),
            group: 2,
            source_centers_group_m: [[0., 0., 0.], [0.1, 0., 0.]],
            source_pin_axis_group: [0., 0., 1.],
            target_centers_m: [[1., 1., 1.], [1., 1.1, 1.]],
            target_pin_axis: [0., 0., 1.],
            evidence: "analytic two-pin fixture".into(),
        };
        let fit = Alignment::fit(&p, &o).unwrap();
        assert!(fit.center_error_m < 1e-12);
        fit.apply(&mut p).unwrap();
        let g = p.groups.iter().find(|g| g.id == 2).unwrap();
        for (local, target) in o.source_centers_group_m.into_iter().zip(o.target_centers_m) {
            assert!(
                (super::super::super::importer::transform(g.pose, local)
                    - DVec3::from_array(target))
                .length()
                    < 1e-10
            );
        }
        assert!(!p.source_frames_and_geometry_reviewed);
        let after_mass = p.mass(&p.groups[1]).unwrap().0;
        assert_eq!(before_mass.mass_kg, after_mass.mass_kg);
        for (a, b) in before_mass
            .com_m
            .into_iter()
            .chain(before_mass.inertia_kg_m2.into_iter().flatten())
            .zip(
                after_mass
                    .com_m
                    .into_iter()
                    .chain(after_mass.inertia_kg_m2.into_iter().flatten()),
            )
        {
            assert!((a - b).abs() < 1e-12);
        }

        for key in p.meshes.keys() {
            assert!(std::sync::Arc::ptr_eq(
                &p.meshes[key].vertices_m,
                &raw.meshes[key].vertices_m
            ));
        }
        assert!(fit.apply(&mut p).is_err());
        let mut attached = raw.clone();
        attached.groups[1].joint = Some(super::super::super::importer::Joint {
            parent: 1,
            motion: super::super::super::importer::Motion::Revolute,
            pivot_m: [0.; 3],
            axis: [0., 0., 1.],
            limits: None,
            reviewed: true,
            evidence: "existing reviewed joint".into(),
        });
        fit.apply(&mut attached).unwrap();
        assert!(!attached.groups[1].joint.as_ref().unwrap().reviewed);
        attached.source_frames_and_geometry_reviewed = true;
        assert!(attached.assembly().is_err());
        attached.groups[1].joint.as_mut().unwrap().reviewed = true;
        assert!(attached.assembly().is_ok());
        let mut bad = o;
        bad.target_centers_m[1][1] = 1.2;
        assert!(Alignment::fit(&raw, &bad).is_err());
        bad.target_centers_m[1][1] = 1.1;
        bad.source_pin_axis_group = [1., 0., 0.];
        assert!(Alignment::fit(&raw, &bad).is_err());
        assert_eq!(
            raw.parts[1].pose,
            super::super::tests::fixture().parts[1].pose
        );
    }
}
