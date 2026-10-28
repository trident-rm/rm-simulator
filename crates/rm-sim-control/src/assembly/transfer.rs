// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Mutable planner state for transfer between isolated workers.
use super::*;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Serializable mutable rig state, bound to the exact reconstructed mechanical model.
/// Collision acceleration and source geometry are rebuilt from verified package bytes.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RigState {
    model_sha256: String,
    source_config_sha256: Option<String>,
    camera_deployed: bool,
    fixed_chassis: bool,
    live_parking: bool,
    live_entry: Option<([f64; 6], [f64; 2])>,
    held_grasp: Option<Pose>,
    ik_hints: Vec<(Pose, [f64; 6])>,
    requested_spin_deltas_rad: Option<[f64; 5]>,
    preferred_spin_deltas_rad: Option<[f64; 5]>,
    initial_engineer_body: Pose,
    engineer_q: [f64; 6],
    core_q: [f64; 6],
    initial_engineer_q: [f64; 6],
    engineer_body: Pose,
    tool: Pose,
    socket: Pose,
    requested_pose: Option<TargetPose>,
    position_error_m: f64,
    angle_error_rad: f64,
    peak_joint_step_rad: f64,
    cached_guard_replays: usize,
    lift_m: f64,
    p_rad: f64,
    q_rad: f64,
    jaw_angles: Vec<f64>,
    gripped: bool,
    retained_grip: bool,
    grasp_roll_rad: f64,
    rod_spin_rad: f64,
    grasp_axial_m: f64,
    grip_requested: bool,
    unit_pose: Pose,
    last_collision: Option<CollisionHit>,
}
impl Rig {
    /// Snapshot mutable joints, grasp, planner hints and source identity without geometry.
    pub fn transfer_state(&self) -> Result<RigState, serde_json::Error> {
        Ok(RigState {
            model_sha256: format!("{:x}", Sha256::digest(serde_json::to_vec(&self.config)?)),
            source_config_sha256: self.source_config_sha256.clone(),
            camera_deployed: self.camera_deployed,
            fixed_chassis: self.fixed_chassis,
            live_parking: self.live_parking,
            live_entry: self.live_entry,
            held_grasp: self.held_grasp,
            ik_hints: (*self.ik_hints).clone(),
            requested_spin_deltas_rad: self.requested_spin_deltas_rad,
            preferred_spin_deltas_rad: self.preferred_spin_deltas_rad,
            initial_engineer_body: self.initial_engineer_body,
            engineer_q: self.engineer_q,
            core_q: self.core_q,
            initial_engineer_q: self.initial_engineer_q,
            engineer_body: self.engineer_body,
            tool: self.tool,
            socket: self.socket,
            requested_pose: self.requested_pose,
            position_error_m: self.position_error_m,
            angle_error_rad: self.angle_error_rad,
            peak_joint_step_rad: self.peak_joint_step_rad,
            cached_guard_replays: self.cached_guard_replays,
            lift_m: self.lift_m,
            p_rad: self.p_rad,
            q_rad: self.q_rad,
            jaw_angles: self.jaw_angles.clone(),
            gripped: self.gripped,
            retained_grip: self.retained_grip,
            grasp_roll_rad: self.grasp_roll_rad,
            rod_spin_rad: self.rod_spin_rad,
            grasp_axial_m: self.grasp_axial_m,
            grip_requested: self.grip_requested,
            unit_pose: self.unit_pose,
            last_collision: self.last_collision.clone(),
        })
    }
    /// Apply a worker snapshot only to the same verified model, rejecting malformed state.
    pub fn restore_transfer_state(&mut self, state: RigState) -> Result<(), String> {
        let identity = format!(
            "{:x}",
            Sha256::digest(serde_json::to_vec(&self.config).map_err(|e| e.to_string())?)
        );
        if state.model_sha256 != identity || state.source_config_sha256 != self.source_config_sha256
        {
            return Err("assembly worker state belongs to another source/model".into());
        }
        let valid_pose = |pose: Pose| {
            pose.translation_m
                .iter()
                .chain(&pose.rotation_wxyz)
                .all(|v| v.is_finite())
                && (pose.rotation_wxyz.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-5
        };
        let valid_joints = |arm: &SerialArm, q: [f64; 6]| {
            q.iter().zip(&arm.axes).all(|(v, axis)| {
                v.is_finite() && *v >= axis.range_rad[0] - 1e-6 && *v <= axis.range_rad[1] + 1e-6
            })
        };
        if ![
            state.initial_engineer_body,
            state.engineer_body,
            state.tool,
            state.socket,
            state.unit_pose,
        ]
        .into_iter()
        .all(valid_pose)
            || state.held_grasp.is_some_and(|p| !valid_pose(p))
            || !valid_joints(&self.config.engineer, state.engineer_q)
            || !valid_joints(&self.config.engineer, state.initial_engineer_q)
            || !valid_joints(&self.config.core, state.core_q)
            || state.ik_hints.len() > 4096
            || state.jaw_angles.len() != 2
            || !state
                .ik_hints
                .iter()
                .all(|(p, q)| valid_pose(*p) && valid_joints(&self.config.engineer, *q))
            || !state
                .jaw_angles
                .iter()
                .all(|v| v.is_finite() && v.abs() <= std::f64::consts::TAU)
            || ![
                state.position_error_m,
                state.angle_error_rad,
                state.peak_joint_step_rad,
                state.lift_m,
                state.p_rad,
                state.q_rad,
                state.grasp_roll_rad,
                state.rod_spin_rad,
                state.grasp_axial_m,
            ]
            .into_iter()
            .all(f64::is_finite)
        {
            return Err("invalid assembly worker state".into());
        }
        self.source_config_sha256 = state.source_config_sha256;
        self.camera_deployed = state.camera_deployed;
        self.fixed_chassis = state.fixed_chassis;
        self.live_parking = state.live_parking;
        self.live_entry = state.live_entry;
        self.held_grasp = state.held_grasp;
        self.ik_hints = std::sync::Arc::new(state.ik_hints);
        self.requested_spin_deltas_rad = state.requested_spin_deltas_rad;
        self.preferred_spin_deltas_rad = state.preferred_spin_deltas_rad;
        self.initial_engineer_body = state.initial_engineer_body;
        self.engineer_q = state.engineer_q;
        self.core_q = state.core_q;
        self.initial_engineer_q = state.initial_engineer_q;
        self.engineer_body = state.engineer_body;
        self.tool = state.tool;
        self.socket = state.socket;
        self.requested_pose = state.requested_pose;
        self.position_error_m = state.position_error_m;
        self.angle_error_rad = state.angle_error_rad;
        self.peak_joint_step_rad = state.peak_joint_step_rad;
        self.cached_guard_replays = state.cached_guard_replays;
        self.lift_m = state.lift_m;
        self.p_rad = state.p_rad;
        self.q_rad = state.q_rad;
        self.jaw_angles = state.jaw_angles;
        self.gripped = state.gripped;
        self.retained_grip = state.retained_grip;
        self.grasp_roll_rad = state.grasp_roll_rad;
        self.rod_spin_rad = state.rod_spin_rad;
        self.grasp_axial_m = state.grasp_axial_m;
        self.grip_requested = state.grip_requested;
        self.unit_pose = state.unit_pose;
        self.last_collision = state.last_collision;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn mutable_state_roundtrip_keeps_grasp_and_rejects_a_different_model() {
        let mut rig = super::super::tests::mechanism_fixture();
        rig.held_grasp = Some(Pose::yawed([0.01, 0.02, 0.03], 0.4));
        rig.retained_grip = true;
        rig.jaw_angles = vec![0., 0.];
        let state = rig.transfer_state().unwrap();
        let mut restored = super::super::tests::mechanism_fixture();
        restored
            .restore_transfer_state(
                serde_json::from_slice(&serde_json::to_vec(&state).unwrap()).unwrap(),
            )
            .unwrap();
        assert_eq!(restored.held_grasp, rig.held_grasp);
        assert!(restored.retained_grip);
        assert_eq!(
            serde_json::to_value(restored.transfer_state().unwrap()).unwrap(),
            serde_json::to_value(state.clone()).unwrap()
        );
        restored.config.core.tool.translation_m[0] += 0.01;
        assert!(restored.restore_transfer_state(state).is_err());
    }

    #[test]
    #[ignore = "requires RM_ASSEMBLY_PACKAGE and RM_ASSEMBLY_AREA checked CAD packages"]
    fn source_rig_worker_state_preserves_grasp_hints_and_identity() {
        let package = std::env::var_os("RM_ASSEMBLY_PACKAGE").expect("set RM_ASSEMBLY_PACKAGE");
        let area = std::env::var_os("RM_ASSEMBLY_AREA").expect("set RM_ASSEMBLY_AREA");
        let target = TargetPose {
            position_m: [-0.05, 0.2, 0.5],
            theta_rad: 0.,
            phi_rad: std::f64::consts::FRAC_PI_2,
            alpha_rad: 0.,
        };
        let mut rig = Rig::load_in_area(Path::new(&package), target, Path::new(&area)).unwrap();
        rig.use_live_collisions();
        rig.select_live_core_branch().unwrap();
        let grasp = Pose::yawed([0.012, -0.005, 0.031], 0.37);
        rig.use_held_unit(&PayloadObservation {
            pose: rig.unit_pose,
            attachment: PayloadAttachment::Carried,
            grasp,
        });
        rig.set_live_entry(&ArmPose {
            joints_rad: rig.engineer_q,
            jaws_rad: [rig.jaw_angles[0], rig.jaw_angles[1]],
        });
        rig.retained_grip = true;
        rig.ik_hints = std::sync::Arc::new(vec![(
            rig.config.engineer.tool_pose(rig.engineer_q),
            rig.engineer_q,
        )]);
        let state = rig.transfer_state().unwrap();
        let bytes = serde_json::to_vec(&state).unwrap();
        let decoded: RigState = serde_json::from_slice(&bytes).unwrap();
        let mut copy = Rig::load_in_area(Path::new(&package), target, Path::new(&area)).unwrap();
        copy.use_live_collisions();
        copy.restore_transfer_state(decoded).unwrap();
        assert_eq!(copy.runtime_pose(), rig.runtime_pose());
        assert_eq!(copy.held_grasp, Some(grasp));
        assert!(copy.retained_grip);
        assert_eq!(copy.live_entry, rig.live_entry);
        assert_eq!(copy.ik_hints, rig.ik_hints);
        assert_eq!(copy.source_config_sha256, rig.source_config_sha256);
        assert_eq!(
            serde_json::to_value(copy.transfer_state().unwrap()).unwrap(),
            serde_json::to_value(state.clone()).unwrap()
        );
        let mut wrong_source = state.clone();
        wrong_source.source_config_sha256 = Some("0".repeat(64));
        assert!(copy.restore_transfer_state(wrong_source).is_err());
        let mut invalid = state;
        invalid.engineer_body.rotation_wxyz = [0.; 4];
        assert!(copy.restore_transfer_state(invalid).is_err());
    }
}
