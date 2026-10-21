// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Persistent Energy Unit ownership, proximity grips and physical release.
use crate::{Field, FieldError, Pose};
use glam::{DQuat, DVec3};
use rm_simulator_physics::projectile::{PayloadShape, PayloadSnapshot};
use serde::{Deserialize, Serialize};

// Admission and restore share this bound; Units persist until the field resets.
const MAX_ENERGY_UNITS: usize = 32;
fn q(p: Pose) -> DQuat {
    let [w, x, y, z] = p.rotation_wxyz;
    DQuat::from_xyzw(x, y, z, w)
}
fn compose(a: Pose, b: Pose) -> Pose {
    let r = q(a) * q(b);
    Pose {
        translation_m: (DVec3::from_array(a.translation_m)
            + q(a) * DVec3::from_array(b.translation_m))
        .to_array(),
        rotation_wxyz: [r.w, r.x, r.y, r.z],
    }
}
fn inverse(p: Pose) -> Pose {
    let r = q(p).inverse();
    Pose {
        translation_m: (r * -DVec3::from_array(p.translation_m)).to_array(),
        rotation_wxyz: [r.w, r.x, r.y, r.z],
    }
}
/// A Unit's current mechanical attachment; gripping is distinct from Core seating.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Attachment {
    /// Carried by the named robot's actual wrist.
    Held(u32),
    /// Freely simulated by Rapier; may rest on a robot or terrain.
    Free,
    /// Coupled to the Core after the named robot's completed insertion.
    Seated(u32),
}
/// One persistent source Unit. Its physics identity survives release and pickup.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EnergyUnit {
    /// Rigid-body geometry, world pose, velocities and stable identity.
    pub physical: PayloadSnapshot,
    /// Current carrier or station attachment.
    pub attachment: Attachment,
    /// Whether successful assembly consumed this Unit; UC §6 R60 forbids its reuse.
    #[serde(default)]
    pub spent: bool,
    /// Attachment-to-Unit transform, metres and wxyz: wrist when held, Core Q frame when seated.
    pub grasp: Pose,
    /// Last assembly generation that seated this Unit, preventing repeated capture after recovery.
    pub seated_generation: u64,
}
impl Field {
    /// Current Energy Units, including authoritative Rapier poses for loose objects.
    ///
    /// ```
    /// use rm_simulator_world::Field;
    /// let field=Field::new(&Default::default())?;
    /// assert!(field.energy_units().is_empty());
    /// # Ok::<(),rm_simulator_world::FieldError>(())
    /// ```
    pub fn energy_units(&self) -> Vec<EnergyUnit> {
        self.energy_units
            .iter()
            .cloned()
            .map(|mut u| {
                if let Some(mut p) = self.physics.payload(u.physical.id) {
                    if u.attachment != Attachment::Free {
                        p.velocity_m_s = u.physical.velocity_m_s;
                        p.angular_velocity_rad_s = u.physical.angular_velocity_rad_s;
                    }
                    u.physical = p;
                }
                u
            })
            .collect()
    }
    fn wrist(&self, owner: u32) -> Option<Pose> {
        let arm = self.arms.iter().find(|a| a.owner == owner)?;
        Some(compose(
            self.chassis_control_state(owner)?.pose,
            arm.model.engineer.tool_pose(arm.joints_rad),
        ))
    }
    /// Preload exactly one Unit at deployment, with closed jaws. Never called by assembly start.
    /// Rejects admission without mutation when 32 persistent Units already exist;
    /// released Units survive carrier removal and are cleared by resetting the field.
    pub fn preload_energy_unit(
        &mut self,
        owner: u32,
        shape: PayloadShape,
    ) -> Result<(), FieldError> {
        if self.energy_units.iter().any(|u| u.physical.id == owner) {
            return Err(FieldError::Restore("duplicate Energy Unit identity"));
        }
        if self.energy_units.len() >= MAX_ENERGY_UNITS {
            return Err(FieldError::Restore("too many Energy Units"));
        }
        let wrist = self
            .wrist(owner)
            .ok_or(FieldError::Chassis("missing Unit carrier"))?;
        let rotation = DQuat::from_rotation_z(-3. * std::f64::consts::FRAC_PI_4);
        let grasp = Pose {
            translation_m: [0., 0., -0.02],
            rotation_wxyz: [rotation.w, rotation.x, rotation.y, rotation.z],
        };
        let physical = PayloadSnapshot {
            id: owner,
            shape,
            pose: compose(wrist, grasp),
            velocity_m_s: [0.; 3],
            angular_velocity_rad_s: [0.; 3],
            prescribed: true,
        };
        self.physics
            .add_payload(&physical)
            .map_err(FieldError::Restore)?;
        self.energy_units.push(EnergyUnit {
            physical,
            attachment: Attachment::Held(owner),
            spent: false,
            grasp,
            seated_generation: 0,
        });
        if let Some(arm) = self.arms.iter_mut().find(|a| a.owner == owner) {
            arm.jaws_rad = std::array::from_fn(|i| arm.model.engineer_jaws[i].closed_rad);
            arm.jaw_target_rad = arm.jaws_rad;
        }
        Ok(())
    }
    /// Open/release or close/pick up a nearby Unit. Does not create or teleport objects.
    pub fn toggle_energy_grip(&mut self, owner: u32) -> Result<(), FieldError> {
        let wrist = self
            .wrist(owner)
            .ok_or(FieldError::Chassis("missing source arm"))?;
        if self.chassis_snapshot(owner).is_none_or(|c| c.defeated) {
            return Err(FieldError::Chassis("robot unavailable"));
        }
        if let Some(index) = self
            .energy_units
            .iter()
            .position(|u| u.attachment == Attachment::Held(owner))
        {
            self.cancel_assembly(owner, "Cancelled by gripper control");
            let unit = &mut self.energy_units[index];
            let p = unit.physical.clone();
            self.physics
                .set_payload_motion(
                    p.id,
                    false,
                    p.pose,
                    p.velocity_m_s,
                    p.angular_velocity_rad_s,
                )
                .map_err(FieldError::Restore)?;
            unit.attachment = Attachment::Free;
            unit.physical = self.physics.payload(p.id).unwrap();
            if let Some(arm) = self.arms.iter_mut().find(|a| a.owner == owner) {
                arm.jaw_target_rad = std::array::from_fn(|i| arm.model.engineer_jaws[i].open_rad);
            }
            return Ok(());
        }
        // The pads grip the cylinder at the calibrated 20 mm section. A seated
        // Unit remains ineligible after successful assembly (R60).
        let expected = compose(
            wrist,
            Pose {
                translation_m: [0., 0., -0.02],
                rotation_wxyz: [1., 0., 0., 0.],
            },
        );
        let index = self
            .energy_units()
            .iter()
            .enumerate()
            .filter(|(_, u)| !u.spent && !matches!(u.attachment, Attachment::Held(_)))
            .filter_map(|(i, u)| {
                let distance = DVec3::from_array(u.physical.pose.translation_m)
                    .distance(DVec3::from_array(expected.translation_m));
                let axis = q(wrist) * DVec3::Z;
                let bore = q(u.physical.pose) * DVec3::Z;
                (distance <= 0.008
                    && axis.dot(bore) >= 5f64.to_radians().cos()
                    && DVec3::from_array(u.physical.velocity_m_s).length() <= 0.5)
                    .then_some((i, distance))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i)
            .ok_or(FieldError::Chassis(
                "no aligned Unit within the gripper pads",
            ))?;
        self.cancel_assembly(owner, "Cancelled by gripper control");
        let mut unit = self.energy_units()[index].clone();
        unit.grasp = compose(inverse(wrist), unit.physical.pose);
        unit.attachment = Attachment::Held(owner);
        self.physics
            .set_payload_motion(unit.physical.id, true, unit.physical.pose, [0.; 3], [0.; 3])
            .map_err(FieldError::Restore)?;
        unit.physical = self.physics.payload(unit.physical.id).unwrap();
        self.energy_units[index] = unit;
        if let Some(arm) = self.arms.iter_mut().find(|a| a.owner == owner) {
            arm.jaw_target_rad = std::array::from_fn(|i| arm.model.engineer_jaws[i].closed_rad);
        }
        Ok(())
    }
    /// Compute a wrist jog target in robot-local metres/radians, without moving the arm. Each command is bounded to 20 mm/0.1 rad.
    pub fn energy_arm_jog_target(
        &self,
        owner: u32,
        translation_m: [f64; 3],
        rotation_rad: [f64; 3],
    ) -> Result<[f64; 6], FieldError> {
        if translation_m
            .into_iter()
            .any(|v| !v.is_finite() || v.abs() > 0.02)
            || rotation_rad
                .into_iter()
                .any(|v| !v.is_finite() || v.abs() > 0.1)
        {
            return Err(FieldError::Chassis("invalid arm jog"));
        }
        if self.chassis_snapshot(owner).is_none_or(|c| c.defeated) {
            return Err(FieldError::Chassis("robot unavailable"));
        }
        let arm = self
            .arms
            .iter()
            .find(|a| a.owner == owner)
            .ok_or(FieldError::Chassis("missing source arm"))?;
        let mut desired = arm.model.engineer.tool_pose(arm.target_rad);
        desired.translation_m =
            std::array::from_fn(|i| desired.translation_m[i] + translation_m[i]);
        let r = DQuat::from_euler(
            glam::EulerRot::XYZ,
            rotation_rad[0],
            rotation_rad[1],
            rotation_rad[2],
        ) * q(desired);
        desired.rotation_wxyz = [r.w, r.x, r.y, r.z];
        arm.model
            .engineer
            .solve(desired, arm.target_rad)
            .map_err(FieldError::Chassis)
    }
    /// Set a host-validated source joint target; motion retains the normal joint-rate limits.
    pub fn command_energy_arm(
        &mut self,
        owner: u32,
        target_rad: [f64; 6],
    ) -> Result<(), FieldError> {
        let arm = self
            .arms
            .iter()
            .find(|a| a.owner == owner)
            .ok_or(FieldError::Chassis("missing source arm"))?;
        if target_rad
            .iter()
            .zip(arm.model.engineer.axes)
            .any(|(q, a)| !q.is_finite() || !(a.range_rad[0]..=a.range_rad[1]).contains(q))
        {
            return Err(FieldError::Chassis("invalid source joint target"));
        }
        self.cancel_assembly(owner, "Cancelled by arm control");
        self.arms
            .iter_mut()
            .find(|a| a.owner == owner)
            .unwrap()
            .target_rad = target_rad;
        Ok(())
    }
    pub(crate) fn sync_energy_units(&mut self) {
        let dt = self.step_duration().as_secs_f64();
        for index in 0..self.energy_units.len() {
            // Only attachment metadata is copied. Hull buffers are immutable and
            // never participate in prescribed motion synchronization.
            let unit = &self.energy_units[index];
            let attachment = unit.attachment;
            let grasp = unit.grasp;
            let seated_generation = unit.seated_generation;
            let (at, prescribed) = match attachment {
                Attachment::Free => continue,
                Attachment::Seated(owner) => {
                    let Some(run) = self
                        .assemblies
                        .iter()
                        .find(|r| r.owner == owner && r.generation == seated_generation)
                    else {
                        continue;
                    };
                    let Some(recovery) = &run.recovery else {
                        continue;
                    };
                    if recovery.elapsed_ns == 0 {
                        continue;
                    }
                    let head = run.pose.frames(&run.model)["core/q"];
                    (
                        compose(compose(run.world_from_rig, head), grasp),
                        !recovery.released(),
                    )
                }
                Attachment::Held(owner) => {
                    let Some(wrist) = self.wrist(owner) else {
                        let unit = &mut self.energy_units[index];
                        let p = &mut unit.physical;
                        self.physics
                            .set_payload_motion(
                                p.id,
                                false,
                                p.pose,
                                p.velocity_m_s,
                                p.angular_velocity_rad_s,
                            )
                            .expect("validated held Unit");
                        p.prescribed = false;
                        unit.attachment = Attachment::Free;
                        continue;
                    };
                    let at = compose(wrist, grasp);
                    if let Some(run) = self.assemblies.iter().find(|r| {
                        r.owner == owner
                            && (r.active() || r.phase == crate::assembly::Phase::Completed)
                    }) && matches!(
                        run.phase,
                        crate::assembly::Phase::Completed | crate::assembly::Phase::BreakingAway
                    ) && !run.pose.gripped
                        && seated_generation != run.generation
                        && run.cursor > 0
                    {
                        let unit = &mut self.energy_units[index];
                        unit.attachment = Attachment::Seated(owner);
                        unit.spent = true;
                        let head =
                            compose(run.world_from_rig, run.pose.frames(&run.model)["core/q"]);
                        unit.grasp = compose(inverse(head), at);
                        unit.seated_generation = run.generation;
                    }
                    (at, true)
                }
            };
            let unit = &mut self.energy_units[index];
            // Prescribed Units retain their last pose here; free Units are read
            // from physics on pickup, before switching back to prescribed motion.
            let previous = &mut unit.physical;
            let velocity = ((DVec3::from_array(at.translation_m)
                - DVec3::from_array(previous.pose.translation_m))
                / dt)
                .to_array();
            let angular = ((q(at) * q(previous.pose).inverse()).to_scaled_axis() / dt).to_array();
            self.physics
                .set_payload_motion(previous.id, prescribed, at, velocity, angular)
                .expect("validated attached Unit");
            previous.pose = at;
            previous.prescribed = prescribed;
            previous.velocity_m_s = velocity;
            previous.angular_velocity_rad_s = angular;
            if !prescribed {
                unit.attachment = Attachment::Free;
            }
        }
    }
    pub(crate) fn release_energy_units(&mut self, owner: u32) {
        for unit in &mut self.energy_units {
            if unit.attachment == Attachment::Held(owner) {
                let p = &unit.physical;
                let _ = self.physics.set_payload_motion(
                    p.id,
                    false,
                    p.pose,
                    p.velocity_m_s,
                    p.angular_velocity_rad_s,
                );
                unit.physical.prescribed = false;
                unit.attachment = Attachment::Free;
            }
        }
    }
    pub(crate) fn restore_energy_units(&mut self) -> Result<(), FieldError> {
        if self.energy_units.len() > MAX_ENERGY_UNITS {
            return Err(FieldError::Restore("too many Energy Units"));
        }
        for unit in &self.energy_units {
            let g = unit.grasp;
            if g.translation_m
                .into_iter()
                .chain(g.rotation_wxyz)
                .any(|v| !v.is_finite())
                || g.translation_m.into_iter().map(|v| v * v).sum::<f64>()
                    > if unit.spent { 1. } else { 0.1 * 0.1 }
                || (g.rotation_wxyz.into_iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-6
            {
                return Err(FieldError::Restore("invalid Energy Unit grasp"));
            }
            if (unit.spent && matches!(unit.attachment, Attachment::Held(_)))
                || (!unit.spent && matches!(unit.attachment, Attachment::Seated(_)))
            {
                return Err(FieldError::Restore("invalid spent Unit attachment"));
            }
            if matches!(unit.attachment, Attachment::Free) == unit.physical.prescribed {
                return Err(FieldError::Restore(
                    "Unit attachment disagrees with dynamics",
                ));
            }
            if let Attachment::Held(owner) = unit.attachment
                && self.arms.iter().all(|a| a.owner != owner)
            {
                return Err(FieldError::Restore("missing Unit holder"));
            }
            self.physics
                .add_payload(&unit.physical)
                .map_err(FieldError::Restore)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::assembly::{ArmState, NEUTRAL_JOINTS_RAD};
    fn fixture() -> (Field, u32) {
        let (mut field, run) = crate::assembly::tests::fixture();
        let mut model = run.model;
        model.engineer.tool = Pose::at([0., 0., 0.65]);
        field.add_arm(ArmState::neutral(run.owner, model)).unwrap();
        let shape = PayloadShape::from_parts(
            &[
                rm_simulator_physics::manipulator_collision::CollisionPart::Box {
                    half_m: [0.03; 3],
                },
            ],
            0.25,
        )
        .unwrap();
        field.preload_energy_unit(run.owner, shape).unwrap();
        (field, run.owner)
    }
    #[test]
    fn repeated_deployments_refuse_excess_units_without_breaking_restore() {
        let (mut field, owner) = fixture();
        let model = field.arms()[0].model.clone();
        let shape = field.energy_units()[0].physical.shape.clone();
        field.remove_chassis(owner).unwrap();
        for _ in 1..MAX_ENERGY_UNITS {
            let owner = field
                .add_chassis(&crate::ChassisPlacement {
                    config: Default::default(),
                    spawn: Pose::at([0., 0., 0.4]),
                    team: crate::Team::Red,
                    kind: crate::RobotKind::Engineer,
                    performance: None,
                })
                .unwrap();
            field
                .add_arm(ArmState::neutral(owner, model.clone()))
                .unwrap();
            field.preload_energy_unit(owner, shape.clone()).unwrap();
            field.remove_chassis(owner).unwrap();
        }
        let owner = field
            .add_chassis(&crate::ChassisPlacement {
                config: Default::default(),
                spawn: Pose::at([0., 0., 0.4]),
                team: crate::Team::Red,
                kind: crate::RobotKind::Engineer,
                performance: None,
            })
            .unwrap();
        field.add_arm(ArmState::neutral(owner, model)).unwrap();
        let before = field.snapshot();
        assert!(matches!(
            field.preload_energy_unit(owner, shape),
            Err(FieldError::Restore("too many Energy Units"))
        ));
        assert!(field.physics.payload(owner).is_none());
        assert_eq!(field.energy_units.len(), MAX_ENERGY_UNITS);
        assert_eq!(field.snapshot(), before);
        let restored = Field::restore(&before, &field.fixed_geometry_snapshot(), 0.).unwrap();
        assert_eq!(restored.energy_units(), field.energy_units());
    }

    #[test]
    fn core_recovery_follows_socket_drops_spent_unit_and_survives_disconnect() {
        use crate::assembly::{CoreRecovery, Phase};
        let (mut field, id) = fixture();
        field.step(256).unwrap();
        let (_, mut run) = crate::assembly::tests::fixture();
        run.model = field.arms()[0].model.clone();
        run.model.core_p_origin_m = [0., 0., 0.6];
        run.model.core_q_origin_m = [0., 0., 0.6];
        run.pose.engineer_body = field.chassis_snapshot(id).unwrap().pose;
        run.pose.engineer_q = field.arms()[0].joints_rad;
        run.pose.jaw_angles = field.arms()[0].jaws_rad;
        for f in &mut run.frames {
            f.joints_rad = run.pose.engineer_q;
            f.jaws_rad = run.pose.jaw_angles;
        }
        let last = run.frames.last().unwrap().clone();
        for i in 1..=16 {
            let mut f = last.clone();
            f.stage = 7;
            f.gripped = false;
            f.jaws_rad = last.jaws_rad.map(|q| q * (1. - i as f64 / 16.));
            run.frames.push(f);
        }
        run.recovery = Some(CoreRecovery::new(
            &run.pose,
            [0.4, 0., 0., 0., 0., 0.],
            [0.5, 0., 0., 0., 0., 0.],
        ));
        field.set_assembly(run).unwrap();
        field.step(4).unwrap();
        let seated = field.energy_units()[0].clone();
        assert!(seated.spent);
        assert_eq!(seated.attachment, Attachment::Seated(id));
        // Even while still exactly between the pads, R60 forbids reclaiming it.
        assert!(field.toggle_energy_grip(id).is_err());
        assert_eq!(field.assemblies()[0].phase, Phase::BreakingAway);
        field.step(36).unwrap();
        assert_eq!(field.assemblies()[0].phase, Phase::Recovering);
        assert_ne!(field.energy_units()[0].physical.pose, seated.physical.pose);
        let mut restored =
            Field::restore(&field.snapshot(), &crate::StaticGeometry::default(), 0.).unwrap();
        field.cancel_assembly(id, "Pilot drives away");
        assert_eq!(field.assemblies()[0].phase, Phase::Recovering);
        field.remove_chassis(id).unwrap();
        restored.remove_chassis(id).unwrap();
        for _ in 0..512 {
            field.step(1).unwrap();
            restored.step(1).unwrap();
            assert_eq!(field.assemblies(), restored.assemblies());
            let a = field.energy_units()[0].clone();
            let b = restored.energy_units()[0].clone();
            assert_eq!(a.attachment, b.attachment);
            for (x, y) in a
                .physical
                .pose
                .translation_m
                .into_iter()
                .chain(a.physical.pose.rotation_wxyz)
                .chain(a.physical.velocity_m_s)
                .chain(a.physical.angular_velocity_rad_s)
                .zip(
                    b.physical
                        .pose
                        .translation_m
                        .into_iter()
                        .chain(b.physical.pose.rotation_wxyz)
                        .chain(b.physical.velocity_m_s)
                        .chain(b.physical.angular_velocity_rad_s),
                )
            {
                assert!((x - y).abs() < 1e-9);
            }
            if field.energy_units()[0].attachment == Attachment::Free {
                break;
            }
        }
        let released = field.energy_units()[0].clone();
        assert_eq!(released.attachment, Attachment::Free);
        assert!(!released.physical.prescribed && released.spent);
        assert!(
            released
                .physical
                .velocity_m_s
                .into_iter()
                .chain(released.physical.angular_velocity_rad_s)
                .all(|v| v.abs() < 1e-9),
            "stationary discharge must not throw the Unit"
        );
        field.step(4).unwrap();
        assert!(
            field.energy_units()[0].physical.pose.translation_m[2]
                < released.physical.pose.translation_m[2]
        );
        field.step(512).unwrap();
        assert_eq!(field.assemblies()[0].phase, Phase::Completed);
        assert_eq!(field.assemblies()[0].pose.p_rad, 0.);
        assert_eq!(field.energy_units().len(), 1);
        let reloaded =
            Field::restore(&field.snapshot(), &crate::StaticGeometry::default(), 0.).unwrap();
        assert!(reloaded.energy_units()[0].spent);
    }
    #[test]
    fn ticking_prescribed_and_free_units_retains_immutable_hull_buffers() {
        let (mut field, owner) = fixture();
        let hulls = |field: &Field| {
            field.energy_units[0]
                .physical
                .shape
                .hulls_m
                .iter()
                .map(|h| h.as_ptr())
                .collect::<Vec<_>>()
        };
        let before = hulls(&field);
        field.step(1).unwrap();
        assert_eq!(hulls(&field), before);
        field.toggle_energy_grip(owner).unwrap();
        let before = hulls(&field);
        field.step(1).unwrap();
        assert_eq!(hulls(&field), before);
    }

    #[test]
    fn a_deployed_unit_is_held_visible_and_can_release_and_regrip_without_spawning() {
        let (mut field, id) = fixture();
        assert!(field.assemblies().is_empty());
        let before = field.energy_units();
        assert_eq!(before[0].attachment, Attachment::Held(id));
        assert_eq!(field.arms()[0].joints_rad, NEUTRAL_JOINTS_RAD);
        field.toggle_energy_grip(id).unwrap();
        assert_eq!(field.energy_units()[0].attachment, Attachment::Free);
        assert_eq!(
            field.energy_units()[0].physical.pose,
            before[0].physical.pose
        );
        field.toggle_energy_grip(id).unwrap();
        let after = field.energy_units();
        assert_eq!(after.len(), 1);
        assert_eq!(after[0].physical.id, before[0].physical.id);
        assert_eq!(after[0].physical.pose, before[0].physical.pose);
        assert_eq!(after[0].attachment, Attachment::Held(id));
        assert!(
            field
                .preload_energy_unit(id, before[0].physical.shape.clone())
                .is_err()
        );
    }
    #[test]
    fn a_resting_unit_on_a_reachable_support_can_be_picked_up_without_snapping() {
        let (mut field, id) = fixture();
        field.step(256).unwrap();
        let at = field.energy_units()[0].physical.pose;
        field.physics.add_static_box(
            Pose::at([
                at.translation_m[0],
                at.translation_m[1],
                at.translation_m[2] - 0.084,
            ]),
            [0.3, 0.3, 0.1],
        );
        field.toggle_energy_grip(id).unwrap();
        field.step(128).unwrap();
        let before = field.energy_units()[0].physical.clone();
        assert!((before.pose.translation_m[2] - at.translation_m[2]).abs() > 0.001);
        assert!(before.velocity_m_s.into_iter().map(|v| v * v).sum::<f64>() < 0.0025);
        field.toggle_energy_grip(id).unwrap();
        let after = field.energy_units()[0].clone();
        assert_eq!(after.attachment, Attachment::Held(id));
        assert_eq!(before.id, after.physical.id);
        assert_eq!(before.pose, after.physical.pose);
    }
    #[test]
    fn releasing_a_moving_wrist_keeps_angular_velocity() {
        let (mut field, id) = fixture();
        let mut target = field.arms()[0].joints_rad;
        target[0] += 0.3;
        field.command_energy_arm(id, target).unwrap();
        field.step(8).unwrap();
        let held = field.energy_units()[0].clone();
        assert!(held.physical.angular_velocity_rad_s[2] > 0.9);
        field.toggle_energy_grip(id).unwrap();
        let free = field.energy_units()[0].clone();
        assert_eq!(
            free.physical.angular_velocity_rad_s,
            held.physical.angular_velocity_rad_s
        );
        assert_eq!(free.physical.pose, held.physical.pose);
    }
    #[test]
    fn dropped_unit_hits_the_robot_and_keeps_identity_through_restore_and_disconnect() {
        let (mut field, id) = fixture();
        field.toggle_energy_grip(id).unwrap();
        let before = field.energy_units()[0].physical.pose.translation_m[2];
        field.step(256).unwrap();
        let units = field.energy_units();
        let unit = &units[0];
        assert_eq!(unit.attachment, Attachment::Free);
        let z = unit.physical.pose.translation_m[2];
        assert!(
            z < before - 0.1 && z > 0.1,
            "must rest on robot, not fall through it: {unit:?}"
        );
        assert!(field.toggle_energy_grip(id).is_err());
        let snapshot = field.snapshot();
        let restored = Field::restore(&snapshot, &field.fixed_geometry_snapshot(), 0.).unwrap();
        assert_eq!(restored.energy_units(), units);
        field.remove_chassis(id).unwrap();
        assert_eq!(field.energy_units().len(), 1);
        field.step(256).unwrap();
        assert!(field.energy_units()[0].physical.pose.translation_m[2] < z - 0.05);
        assert_eq!(field.energy_units()[0].physical.id, id);
    }
}
