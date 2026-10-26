// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Exact swept-transition certificates and matching.
use super::Rig;
/// Exact, in-memory certificate for an already swept transition. Never serialized.
#[derive(Clone)]
pub(super) struct ValidatedTransition {
    pub(super) from: Rig,
    pub(super) to: Rig,
}
fn same_arc<T>(a: &Option<std::sync::Arc<T>>, b: &Option<std::sync::Arc<T>>) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(a), Some(b)) => std::sync::Arc::ptr_eq(a, b),
        _ => false,
    }
}
fn same_guard_state(a: &Rig, b: &Rig) -> bool {
    a.config == b.config
        && same_arc(&a.collision, &b.collision)
        && same_arc(&a.area, &b.area)
        && a.camera_deployed == b.camera_deployed
        && a.engineer_body == b.engineer_body
        && a.engineer_q == b.engineer_q
        && a.core_q == b.core_q
        && a.tool == b.tool
        && a.unit_pose == b.unit_pose
        && a.held_grasp == b.held_grasp
        && a.lift_m == b.lift_m
        && a.p_rad == b.p_rad
        && a.q_rad == b.q_rad
        && a.jaw_angles == b.jaw_angles
        && a.gripped == b.gripped
        && a.retained_grip == b.retained_grip
        && a.grip_requested == b.grip_requested
        && a.grasp_roll_rad == b.grasp_roll_rad
        && a.grasp_axial_m == b.grasp_axial_m
        && a.rod_spin_rad == b.rod_spin_rad
}
impl ValidatedTransition {
    pub(super) fn matches(&self, from: &Rig, to: &Rig) -> bool {
        same_guard_state(&self.from, from) && same_guard_state(&self.to, to)
    }
}
