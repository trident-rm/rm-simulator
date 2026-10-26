// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Copied controller inputs; these records grant no match or actuator authority.
use rm_sim_physics::{Pose, assembly_model::Model};
use serde::{Deserialize, Serialize};

/// Actual six arm joints and two jaws, radians, sampled by the caller.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmPose {
    /// Actual rotary joint coordinates, radians.
    pub joints_rad: [f64; 6],
    /// Actual jaw coordinates, radians.
    pub jaws_rad: [f64; 2],
}
impl ArmPose {
    /// Nominal raised neutral configuration with source-defined open jaws.
    pub fn neutral(model: &Model) -> Self {
        Self {
            joints_rad: super::NEUTRAL_JOINTS_RAD,
            jaws_rad: std::array::from_fn(|i| {
                model.engineer_jaws.get(i).map_or(0., |j| j.open_rad)
            }),
        }
    }
}
/// How the observed payload couples to the selected gripper.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PayloadAttachment {
    /// No grasp by this arm; the observed payload remains an obstacle.
    Free,
    /// A real grasp exists, but a station still constrains the payload pose.
    Retained,
    /// The payload follows the arm's fixed grasp transform.
    Carried,
}
/// Caller-resolved payload pose and attachment relative to the selected arm.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PayloadObservation {
    /// Current world FLU pose, metres and wxyz quaternion.
    pub pose: Pose,
    /// Actual grasp and station constraint, resolved by the owner.
    pub attachment: PayloadAttachment,
    /// Fixed tool-to-payload attachment transform, metres and wxyz quaternion.
    pub grasp: Pose,
}

/// Compact validated tick sample; fixed configuration stays in the run header.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlaybackFrame {
    /// Engineer joint angles, radians.
    pub joints_rad: [f64; 6],
    /// Jaw angles, radians.
    pub jaws_rad: [f64; 2],
    /// Unit pose in rig coordinates, metres and wxyz.
    pub unit: Pose,
    /// Logical insertion guide coordinates, metres.
    pub guide_m: [f64; 3],
    /// P, Q and off-axis guide angles, radians.
    pub angles_rad: [f64; 3],
    /// Actual kinematic grasp attachment.
    pub gripped: bool,
    /// Planner stage identifier: approach, grasp, insert, lift, flip, align, settle, breakaway.
    pub stage: u8,
}
