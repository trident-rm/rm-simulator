// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Passive presentation fixtures. No archived gameplay state machine is required.
use rm_simulator_world::{OutpostSnapshot, Pose, RuneKind, RuneSnapshot, RuneState};

pub fn rune(hub_pose: Pose) -> RuneSnapshot {
    RuneSnapshot {
        motion: None,
        kind: RuneKind::Small,
        active_blades: [true, false, false, false, false],
        completed_groups: 0,
        angular_speed_rad_s: std::f64::consts::PI / 3.,
        time_ns: 0,
        hub_pose,
        target_radius_m: 0.7,
        angle_rad: 0.,
        state: RuneState::Activating,
        active_blade: Some(0),
        activated: [false; 5],
        target_poses: rm_simulator_world::rune::target_poses(hub_pose, 0.7, 0.),
        state_since_ns: 0,
    }
}
pub fn outpost(origin: Pose) -> OutpostSnapshot {
    let mut snapshot = OutpostSnapshot {
        speed_rad_s: rm_simulator_world::outpost::DEFAULT_SPEED_RAD_S,
        pivot_cad_m: rm_simulator_world::outpost::PIVOT_CAD_M,
        time_ns: 0,
        origin,
        angle_rad: 0.,
        armors: vec![],
        hp: 1000,
        destroyed: false,
        started_ns: None,
        stopped_ns: None,
        homing: false,
    };
    snapshot.rebuild_armors();
    snapshot
}
