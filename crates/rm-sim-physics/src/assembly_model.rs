// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Source-calibrated arm, gripper and positioning-rig geometry and forward kinematics.
//! No match attempts, permissions, rewards, playback or host clock are owned here.
//!
//! ```
//! use rm_sim_physics::assembly_model::Jaw;
//! let jaw = Jaw {
//!     name: "jaw-left".into(), origin_m: [0., 0., 0.1], axis: [0., 0., 1.],
//!     open_rad: 0.4, closed_rad: 0.1,
//! };
//! assert!(jaw.closed_rad < jaw.open_rad);
//! ```
/// Simulator joint-rate limits, radians/second; not donor motor specifications.
pub const JOINT_SPEED_RAD_S: [f64; 6] = [1.; 6];
/// Simulator jaw-rate limit, radians/second; not a measured motor specification.
pub const JAW_SPEED_RAD_S: f64 = 1.;
/// Raised, compact neutral posture for the PIE donor, radians. Simulator preset;
/// the shoulder/forearm oppose so the empty wrist stays approximately level.
pub const NEUTRAL_JOINTS_RAD: [f64; 6] = [0., 0.35, -0.35, 0., 0., 0.];

use crate::{Pose, manipulator::SerialArm};
use glam::DQuat;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
/// Source-calibrated assembly geometry; units are explicit on each field.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Jaw {
    /// Source motion group name.
    pub name: String,
    /// Bearing origin in the reference body frame, metres.
    pub origin_m: [f64; 3],
    /// Unit bearing axis in the reference body frame.
    pub axis: [f64; 3],
    /// Open jaw angle in radians.
    pub open_rad: f64,
    /// Fitted grasp-stop angle in radians.
    pub closed_rad: f64,
}
/// Source-calibrated assembly geometry; units are explicit on each field.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Auxiliary {
    /// Source motion group name.
    pub name: String,
    /// Zero-pose source group frame, metres and wxyz.
    pub reference: Pose,
    /// Source-fitted stowed transform relative to the forearm.
    pub stowed_delta: Pose,
}
/// Source-calibrated assembly geometry; units are explicit on each field.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Model {
    /// Source calibration schema; five for the approved PIE binding.
    pub schema: u32,
    /// Rulebook OXYZ frame in rig coordinates, metres and wxyz.
    pub assembly_frame: Pose,
    /// Source pedestal reference points in rig metres.
    pub core_envelope_m: Vec<[f64; 3]>,
    /// Source-fitted six-axis Engineer geometry.
    pub engineer: SerialArm,
    /// Source-fitted six-axis Core positioning geometry.
    pub core: SerialArm,
    /// The two independently moving source jaws.
    pub engineer_jaws: Vec<Jaw>,
    /// Source camera and UWB attachment transforms.
    pub engineer_auxiliary: Vec<Auxiliary>,
    /// P bearing origin in the source head reference frame, metres.
    pub core_p_origin_m: [f64; 3],
    /// Unit P bearing axis.
    pub core_p_axis: [f64; 3],
    /// Q bearing origin in the source head reference frame, metres.
    pub core_q_origin_m: [f64; 3],
    /// Unit Q bearing axis.
    pub core_q_axis: [f64; 3],
    /// Unit carriage translation axis.
    pub core_lift_axis: [f64; 3],
}

impl Model {
    /// Validate bounded, finite source calibration before using its kinematics.
    pub fn validate(&self) -> Result<(), &'static str> {
        self.engineer.validate()?;
        self.core.validate()?;
        let valid = |p: Pose| {
            p.translation_m
                .into_iter()
                .chain(p.rotation_wxyz)
                .all(f64::is_finite)
                && (p.rotation_wxyz.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-6
        };
        let unit_axis = |axis: [f64; 3]| {
            axis.into_iter().all(f64::is_finite)
                && (axis.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-6
        };
        if self.engineer_jaws.len() != 2
            || self.engineer_auxiliary.len() > 8
            || self.core_envelope_m.len() > 8192
            || self.schema != 5
            || !valid(self.assembly_frame)
            || !unit_axis(self.core_p_axis)
            || !unit_axis(self.core_q_axis)
            || !unit_axis(self.core_lift_axis)
            || !self
                .core_p_origin_m
                .into_iter()
                .chain(self.core_q_origin_m)
                .chain(self.core_envelope_m.iter().flatten().copied())
                .all(f64::is_finite)
            || self.engineer_jaws.iter().any(|j| {
                j.name.len() > 64
                    || !unit_axis(j.axis)
                    || !j
                        .origin_m
                        .into_iter()
                        .chain([j.open_rad, j.closed_rad])
                        .all(f64::is_finite)
            })
            || self
                .engineer_auxiliary
                .iter()
                .any(|a| a.name.len() > 64 || !valid(a.reference) || !valid(a.stowed_delta))
        {
            return Err("invalid source assembly model");
        }
        Ok(())
    }
}

/// One source rig configuration, independent of rendering and planner internals.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigPose {
    /// Engineer body in source rig coordinates, metres and wxyz.
    pub engineer_body: Pose,
    /// Six Engineer joint angles, radians.
    pub engineer_q: [f64; 6],
    /// Six Core positioning angles, radians.
    pub core_q: [f64; 6],
    /// Unit pose in source rig coordinates, metres and wxyz.
    pub unit_pose: Pose,
    /// Two source jaw angles, radians.
    pub jaw_angles: [f64; 2],
    /// Whether the camera retains its source deployed posture.
    pub camera_deployed: bool,
    /// Carriage lift in metres.
    pub lift_m: f64,
    /// P rotation in radians.
    pub p_rad: f64,
    /// Q rotation in radians.
    pub q_rad: f64,
    /// Whether the Unit is held by the jaws.
    pub gripped: bool,
}
fn quaternion(p: Pose) -> DQuat {
    let [w, x, y, z] = p.rotation_wxyz;
    DQuat::from_xyzw(x, y, z, w)
}
fn position(p: Pose) -> glam::DVec3 {
    glam::DVec3::from_array(p.translation_m)
}
fn pose(p: glam::DVec3, q: DQuat) -> Pose {
    Pose {
        translation_m: p.to_array(),
        rotation_wxyz: [q.w, q.x, q.y, q.z],
    }
}
fn compose(a: Pose, b: Pose) -> Pose {
    let q = quaternion(a);
    pose(position(a) + q * position(b), q * quaternion(b))
}
fn inverse(a: Pose) -> Pose {
    let q = quaternion(a).inverse();
    pose(-(q * position(a)), q)
}
fn delta(origin: [f64; 3], axis: [f64; 3], angle: f64) -> Pose {
    let origin = glam::DVec3::from_array(origin);
    let q = DQuat::from_axis_angle(glam::DVec3::from_array(axis), angle);
    pose(origin - q * origin, q)
}
fn translation(v: [f64; 3]) -> Pose {
    Pose::at(v)
}
impl RigPose {
    /// Source link frames; Engineer frames are body-local, Core frames rig-local.
    pub fn frames(&self, config: &Model) -> BTreeMap<String, Pose> {
        let mut result = BTreeMap::new();
        result.insert("engineer/body".into(), Pose::default());
        result.insert("core/body".into(), Pose::default());
        for (prefix, arm, q) in [
            ("engineer", &config.engineer, self.engineer_q),
            ("core", &config.core, self.core_q),
        ] {
            for (i, p) in arm.frames(q).into_iter().enumerate() {
                result.insert(format!("{prefix}/j{}", i + 1), p);
            }
        }
        let d = compose(
            config.engineer.frames(self.engineer_q)[5],
            inverse(translation(config.engineer.axes[5].origin_m)),
        );
        for (jaw, angle) in config.engineer_jaws.iter().zip(&self.jaw_angles) {
            result.insert(
                format!("engineer/{}", jaw.name),
                compose(
                    d,
                    compose(
                        delta(jaw.origin_m, jaw.axis, *angle),
                        translation(jaw.origin_m),
                    ),
                ),
            );
        }
        let forearm = compose(
            config.engineer.frames(self.engineer_q)[2],
            inverse(translation(config.engineer.axes[2].origin_m)),
        );
        for aux in &config.engineer_auxiliary {
            result.insert(
                format!("engineer/{}", aux.name),
                compose(
                    forearm,
                    compose(
                        if aux.name == "camera-lift" && self.camera_deployed {
                            Pose::default()
                        } else {
                            aux.stowed_delta
                        },
                        aux.reference,
                    ),
                ),
            );
        }
        let l = translation(config.core_lift_axis.map(|v| v * self.lift_m));
        result.insert(
            "core/lift".into(),
            compose(
                self.core_delta(config),
                compose(l, translation(config.core_q_origin_m)),
            ),
        );
        result.insert(
            "core/p".into(),
            compose(
                self.head_delta(config, self.lift_m, self.p_rad, 0.),
                translation(config.core_p_origin_m),
            ),
        );
        result.insert(
            "core/q".into(),
            compose(
                self.head_delta(config, self.lift_m, self.p_rad, self.q_rad),
                translation(config.core_q_origin_m),
            ),
        );
        result
    }
    fn core_delta(&self, config: &Model) -> Pose {
        compose(
            config.core.frames(self.core_q)[5],
            inverse(translation(config.core.axes[5].origin_m)),
        )
    }
    fn head_delta(&self, config: &Model, lift: f64, p: f64, q: f64) -> Pose {
        compose(
            self.core_delta(config),
            compose(
                delta(config.core_p_origin_m, config.core_p_axis, p),
                compose(
                    translation(config.core_lift_axis.map(|v| v * lift)),
                    delta(config.core_q_origin_m, config.core_q_axis, q),
                ),
            ),
        )
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::manipulator::RevoluteAxis;

    #[test]
    fn source_frames_preserve_jaw_pivots_lift_and_camera_postures() {
        let arm = SerialArm {
            axes: [RevoluteAxis {
                origin_m: [0.; 3],
                axis: [0., 0., 1.],
                range_rad: [-3.2, 3.2],
            }; 6],
            tool: Pose::default(),
        };
        let model = Model {
            schema: 5,
            assembly_frame: Pose::default(),
            core_envelope_m: vec![],
            engineer: arm.clone(),
            core: arm,
            engineer_jaws: ["left", "right"]
                .map(|name| Jaw {
                    name: name.into(),
                    origin_m: [0.1, 0., 0.],
                    axis: [0., 0., 1.],
                    open_rad: 0.,
                    closed_rad: 0.1,
                })
                .to_vec(),
            engineer_auxiliary: vec![Auxiliary {
                name: "camera-lift".into(),
                reference: Pose::at([0., 0., 0.2]),
                stowed_delta: Pose::at([0., 0., -0.1]),
            }],
            core_p_origin_m: [0.; 3],
            core_p_axis: [0., 1., 0.],
            core_q_origin_m: [0.; 3],
            core_q_axis: [0., 0., 1.],
            core_lift_axis: [0., 0., 1.],
        };
        model.validate().unwrap();
        let mut rig = RigPose {
            engineer_body: Pose::at([2., 3., 4.]),
            engineer_q: [0.; 6],
            core_q: [0.; 6],
            unit_pose: Pose::default(),
            jaw_angles: [0.5, -0.5],
            camera_deployed: false,
            lift_m: 0.25,
            p_rad: 0.,
            q_rad: 0.,
            gripped: false,
        };
        let frames = rig.frames(&model);
        assert_eq!(frames["core/lift"].translation_m, [0., 0., 0.25]);
        assert_eq!(frames["engineer/camera-lift"].translation_m, [0., 0., 0.1]);
        for name in ["left", "right"] {
            let p = frames[&format!("engineer/{name}")];
            assert!((p.translation_m[0] - 0.1).abs() < 1e-14);
            assert!(p.translation_m[1].abs() < 1e-14);
        }
        rig.camera_deployed = true;
        assert_eq!(
            rig.frames(&model)["engineer/camera-lift"].translation_m,
            [0., 0., 0.2]
        );
    }
}
