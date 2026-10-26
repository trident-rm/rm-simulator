// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Host-side conversions for caller-driven assembly planning.
use rm_sim_control::assembly::{ArmPose, PayloadAttachment, PayloadObservation, PlaybackFrame};
pub(crate) use rm_sim_control::assembly::{Rig, Signal, compose};

pub(crate) fn arm_pose(arm: &rm_sim_world::assembly::ArmState) -> ArmPose {
    ArmPose {
        joints_rad: arm.joints_rad,
        jaws_rad: arm.jaws_rad,
    }
}
pub(crate) fn payload(
    unit: &rm_sim_world::energy_unit::EnergyUnit,
    owner: u32,
) -> PayloadObservation {
    PayloadObservation {
        pose: unit.physical.pose,
        grasp: unit.grasp,
        attachment: match unit.attachment {
            rm_sim_world::energy_unit::Attachment::Held(id) if id == owner => {
                PayloadAttachment::Carried
            }
            rm_sim_world::energy_unit::Attachment::Extracting(id) if id == owner => {
                PayloadAttachment::Retained
            }
            _ => PayloadAttachment::Free,
        },
    }
}
pub(crate) fn playback(frame: PlaybackFrame) -> rm_sim_world::assembly::Frame {
    rm_sim_world::assembly::Frame {
        joints_rad: frame.joints_rad,
        jaws_rad: frame.jaws_rad,
        unit: frame.unit,
        guide_m: frame.guide_m,
        angles_rad: frame.angles_rad,
        gripped: frame.gripped,
        stage: frame.stage,
    }
}
