// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Finite supply inventory and force-limited axial pickup on the explicit clock.
use super::*;
use sim_engine::retention::{PullOffLimits, PullOffOutcome, PullOffState};

/// Upper bound on magnetic withdrawal resistance, excluding gravity.
/// RMUC 2027 V1.0.0 §4.2.6, printed p.45, Figures 4-21–4-22.
pub const MAX_RETENTION_N: f64 = 10.;

/// A source Unit mount and its restorable constrained withdrawal state.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SupplyMount {
    /// Original Unit-centred world pose. Local +z is the outward bore axis.
    pub pose: Pose,
    /// Source mesh-node name, when the source scene itself renders this Unit.
    pub visual_node: Option<String>,
    /// Mechanical limits; stiffness, damping, clearance and robot forces are simulator policies.
    pub limits: PullOffLimits,
    /// Actual axial displacement and velocity, not commanded wrist motion.
    pub pull: PullOffState,
    /// Whether the last extraction failed because the grip slipped.
    pub slipped: bool,
}
impl SupplyMount {
    /// Prepare a mounted nominal 400 g Unit. Uses the 10 N rulebook upper bound.
    /// The 20 mm guide clearance, 5000 N/m stiffness, 10 Ns/m damping,
    /// 20 N arm limit and 25 N grip capacity are explicit simulator assumptions.
    pub fn new(pose: Pose) -> Self {
        Self {
            pose,
            visual_node: None,
            limits: PullOffLimits {
                mass_kg: 0.4,
                retention_n: MAX_RETENTION_N,
                clearance_m: 0.02,
                stiffness_n_m: 5000.,
                damping_n_s_m: 10.,
                arm_force_n: 20.,
                grip_force_n: 25.,
            },
            pull: PullOffState::default(),
            slipped: false,
        }
    }
    fn validate(&self) -> Result<(), FieldError> {
        self.limits
            .validate()
            .map_err(|_| FieldError::Restore("invalid supply force limits"))?;
        self.pose
            .validate_with(sim_engine::pose::PoseTolerance::Runtime)
            .map_err(|_| FieldError::Restore("invalid supply mount pose"))?;
        if self.limits.retention_n > MAX_RETENTION_N
            || self.limits.clearance_m > 0.1
            || self
                .visual_node
                .as_ref()
                .is_some_and(|n| n.is_empty() || n.len() > 256)
            || !self.pull.travel_m.is_finite()
            || !(0. ..=self.limits.clearance_m).contains(&self.pull.travel_m)
            || !self.pull.velocity_m_s.is_finite()
        {
            return Err(FieldError::Restore("invalid supply mount"));
        }
        Ok(())
    }
}
pub(super) fn validate_unit(unit: &EnergyUnit) -> Result<(), FieldError> {
    if let Some(mount) = &unit.supply {
        mount.validate()?;
        if mount.limits.mass_kg != unit.physical.shape.mass_kg {
            return Err(FieldError::Restore("supply mass disagrees with payload"));
        }
        if matches!(
            unit.attachment,
            Attachment::Mounted | Attachment::Extracting(_)
        ) {
            let axis = q(mount.pose) * DVec3::Z;
            let position = DVec3::from_array(mount.pose.translation_m) + axis * mount.pull.travel_m;
            if unit.spent
                || !unit.physical.prescribed
                || position.distance(DVec3::from_array(unit.physical.pose.translation_m)) > 1e-8
                || q(mount.pose).angle_between(q(unit.physical.pose)).abs() > 1e-6
            {
                return Err(FieldError::Restore("invalid retained supply Unit"));
            }
        }
    } else if matches!(
        unit.attachment,
        Attachment::Mounted | Attachment::Extracting(_)
    ) {
        return Err(FieldError::Restore("missing supply mount"));
    }
    Ok(())
}
impl Field {
    pub(crate) fn reset_supply_units(&mut self) -> Result<(), FieldError> {
        for unit in &mut self.energy_units {
            let Some(mount) = &mut unit.supply else {
                continue;
            };
            self.physics
                .set_payload_motion(unit.physical.id, true, mount.pose, [0.; 3], [0.; 3])
                .map_err(FieldError::Restore)?;
            unit.physical.pose = mount.pose;
            unit.physical.velocity_m_s = [0.; 3];
            unit.physical.angular_velocity_rad_s = [0.; 3];
            unit.physical.prescribed = true;
            unit.attachment = Attachment::Mounted;
            unit.spent = false;
            unit.grasp = Pose::default();
            unit.seated_generation = 0;
            mount.pull = PullOffState::default();
            mount.slipped = false;
        }
        Ok(())
    }
    /// Install exactly six persistent Units at caller-verified source mount poses.
    /// IDs must be unused across the inventory. No Unit is replaced or replenished.
    /// Validation precedes insertion; a failed admission leaves the field unchanged.
    ///
    /// ```
    /// use rm_sim_world::{Field, Pose, energy_unit::supply::SupplyMount};
    /// use rm_sim_world::projectile::PayloadShape;
    /// use rm_sim_physics::manipulator_collision::CollisionPart;
    /// let mut field = Field::new(&Default::default())?;
    /// let shape = PayloadShape::from_parts(&[CollisionPart::Box { half_m: [0.03; 3] }], 0.4)?;
    /// let mounts = std::array::from_fn(|i| (100 + i as u32,
    ///     SupplyMount::new(Pose::at([i as f64, 0., 1.]))));
    /// field.add_supply_station(mounts, shape)?;
    /// assert_eq!(field.energy_units().len(), 6);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    ///
    /// # Errors
    /// Rejects duplicate IDs, invalid shapes/mounts or the 32-Unit inventory bound.
    pub fn add_supply_station(
        &mut self,
        units: [(u32, SupplyMount); 6],
        shape: PayloadShape,
    ) -> Result<(), FieldError> {
        if self.energy_units.len() + 6 > MAX_ENERGY_UNITS {
            return Err(FieldError::Restore("too many Energy Units"));
        }
        let prepared = rm_sim_physics::projectile::PreparedPayload::new(shape.clone())
            .map_err(FieldError::Restore)?;
        for (i, (id, mount)) in units.iter().enumerate() {
            mount.validate()?;
            prepared
                .validate_placement(sim_engine::model::BodyState::at(mount.pose))
                .map_err(FieldError::Restore)?;
            if mount.pull != PullOffState::default()
                || mount.limits.mass_kg != shape.mass_kg
                || self.energy_units.iter().any(|u| u.physical.id == *id)
                || units[..i].iter().any(|(previous, _)| previous == id)
            {
                return Err(FieldError::Restore("invalid or duplicate supply Unit"));
            }
        }
        for (id, mount) in units {
            self.physics
                .add_prepared_payload(
                    id,
                    &prepared,
                    sim_engine::model::BodyState {
                        pose: mount.pose,
                        velocity_m_s: [0.; 3],
                        angular_velocity_rad_s: [0.; 3],
                    },
                    true,
                )
                .map_err(FieldError::Restore)?;
            self.physics
                .enable_payload_queries(id)
                .map_err(FieldError::Restore)?;
            self.energy_units.push(EnergyUnit {
                physical: PayloadSnapshot {
                    id,
                    shape: shape.clone(),
                    pose: mount.pose,
                    velocity_m_s: [0.; 3],
                    angular_velocity_rad_s: [0.; 3],
                    prescribed: true,
                },
                attachment: Attachment::Mounted,
                spent: false,
                grasp: Pose::default(),
                seated_generation: 0,
                supply: Some(mount),
            });
        }
        Ok(())
    }
    pub(super) fn release_supply_grip(&mut self, index: usize) {
        // Losing the grip does not remove the magnet or guide. Return on ticks.
        self.energy_units[index].attachment = Attachment::Mounted;
    }
    pub(super) fn return_supply_unit(&mut self, index: usize) {
        let duration_ns = self.step_duration().as_ns();
        let unit = &mut self.energy_units[index];
        let mount = unit.supply.as_mut().expect("retained supply mount");
        let axis = q(mount.pose) * DVec3::Z;
        let gravity = DVec3::new(0., 0., -crate::projectile::GRAVITY_M_S2);
        if mount.pull == PullOffState::default()
            && mount.limits.mass_kg * gravity.dot(axis) <= mount.limits.retention_n
        {
            return;
        }
        let slices = duration_ns.div_ceil(1_000_000);
        let limits = PullOffLimits {
            arm_force_n: 0.,
            ..mount.limits
        };
        let mut outcome = PullOffOutcome::Retained;
        for _ in 0..slices {
            outcome = mount
                .pull
                .advance(
                    limits,
                    [0.; 3],
                    axis.to_array(),
                    gravity.to_array(),
                    duration_ns as f64 * 1e-9 / slices as f64,
                )
                .expect("validated returning supply Unit");
            if outcome == PullOffOutcome::Extracted {
                break;
            }
        }
        write_endpoint(unit, &mut self.physics);
        if outcome == PullOffOutcome::Extracted {
            unit.physical
                .release_attachment(&mut self.physics)
                .expect("installed supply Unit");
            unit.attachment = Attachment::Free;
        }
    }
    pub(super) fn advance_supply_grip(&mut self, index: usize, owner: u32) {
        if self.chassis_defeated(owner).unwrap_or(true) {
            self.release_supply_grip(index);
            return;
        }
        let Some(wrist) = self.wrist(owner) else {
            self.release_supply_grip(index);
            return;
        };
        let duration_ns = self.step_duration().as_ns();
        let arm = self
            .arms
            .iter()
            .find(|a| a.owner == owner)
            .expect("wrist has source arm");
        let closed =
            (0..2).all(|i| (arm.jaws_rad[i] - arm.model.engineer_jaws[i].closed_rad).abs() <= 1e-3);
        let unit = &mut self.energy_units[index];
        let mount = unit.supply.as_mut().expect("retained supply mount");
        let axis = q(mount.pose) * DVec3::Z;
        let target_pose = PayloadSnapshot::attached_pose(wrist, unit.grasp);
        let target = DVec3::from_array(target_pose.translation_m)
            - DVec3::from_array(mount.pose.translation_m);
        if (q(target_pose) * DVec3::Z).dot(axis) < 5f64.to_radians().cos()
            || (!closed && (target - axis * mount.pull.travel_m).length() > 0.008)
        {
            mount.slipped = true;
            self.release_supply_grip(index);
            return;
        }
        if !closed {
            self.return_supply_unit(index);
            return;
        }
        let slices = duration_ns.div_ceil(1_000_000);
        let dt_s = duration_ns as f64 * 1e-9 / slices as f64;
        let mut outcome = PullOffOutcome::Retained;
        for _ in 0..slices {
            outcome = mount
                .pull
                .advance(
                    mount.limits,
                    target.to_array(),
                    axis.to_array(),
                    [0., 0., -crate::projectile::GRAVITY_M_S2],
                    dt_s,
                )
                .expect("validated supply pull");
            if outcome != PullOffOutcome::Retained {
                break;
            }
        }
        write_endpoint(unit, &mut self.physics);
        match outcome {
            PullOffOutcome::Retained => {}
            PullOffOutcome::Slipped => {
                unit.supply.as_mut().expect("supply mount").slipped = true;
                self.release_supply_grip(index);
                if let Some(arm) = self.arms.iter_mut().find(|a| a.owner == owner) {
                    arm.jaw_target_rad =
                        std::array::from_fn(|i| arm.model.engineer_jaws[i].open_rad);
                }
            }
            PullOffOutcome::Extracted => {
                // Capture the actual endpoint; never snap to the commanded wrist.
                unit.grasp = PayloadSnapshot::attachment_offset(wrist, unit.physical.pose);
                unit.attachment = Attachment::Held(owner);
            }
        }
    }
}

fn write_endpoint(unit: &mut EnergyUnit, physics: &mut crate::WorldPhysics) {
    let mount = unit.supply.as_ref().expect("supply mount");
    let axis = q(mount.pose) * DVec3::Z;
    let position = DVec3::from_array(mount.pose.translation_m) + axis * mount.pull.travel_m;
    let at = Pose {
        translation_m: position.to_array(),
        ..mount.pose
    };
    let velocity = (axis * mount.pull.velocity_m_s).to_array();
    physics
        .set_payload_motion(unit.physical.id, true, at, velocity, [0.; 3])
        .expect("installed supply Unit");
    unit.physical.pose = at;
    unit.physical.velocity_m_s = velocity;
    unit.physical.angular_velocity_rad_s = [0.; 3];
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture(arm_n: f64, grip_n: f64) -> (Field, u32) {
        let (mut field, owner) = super::super::tests::fixture();
        let unit = &mut field.energy_units[0];
        let mut mount = SupplyMount::new(unit.physical.pose);
        mount.limits.arm_force_n = arm_n;
        mount.limits.grip_force_n = grip_n;
        mount.limits.mass_kg = unit.physical.shape.mass_kg;
        unit.supply = Some(mount);
        unit.attachment = Attachment::Mounted;
        field.toggle_energy_grip(owner).unwrap();
        assert_eq!(
            field.energy_units[0].attachment,
            Attachment::Extracting(owner)
        );
        (field, owner)
    }
    fn move_wrist(field: &mut Field, owner: u32, distance_m: f64) {
        let mount = field.energy_units[0].supply.as_ref().unwrap();
        let axis = q(mount.pose) * DVec3::Z;
        let mut pose = field.chassis_snapshot(owner).unwrap().pose;
        pose.translation_m = (DVec3::from_array(pose.translation_m) + axis * distance_m).to_array();
        field.place_chassis(owner, pose).unwrap();
    }
    #[test]
    fn force_limits_gate_extraction_and_grip_failure_keeps_inventory() {
        let (mut weak, owner) = fixture(5., 25.);
        move_wrist(&mut weak, owner, 0.08);
        for _ in 0..128 {
            weak.advance_supply_grip(0, owner);
        }
        assert_eq!(
            weak.energy_units[0].supply.as_ref().unwrap().pull.travel_m,
            0.
        );
        assert_eq!(
            weak.energy_units[0].attachment,
            Attachment::Extracting(owner)
        );

        let (mut slipping, owner) = fixture(20., 5.);
        move_wrist(&mut slipping, owner, 0.08);
        slipping.advance_supply_grip(0, owner);
        assert_eq!(slipping.energy_units[0].attachment, Attachment::Mounted);
        assert!(slipping.energy_units[0].supply.as_ref().unwrap().slipped);
        assert_eq!(slipping.energy_units.len(), 1);

        let (mut strong, owner) = fixture(20., 25.);
        move_wrist(&mut strong, owner, 0.08);
        let id = strong.energy_units[0].physical.id;
        for _ in 0..128 {
            strong.advance_supply_grip(0, owner);
            if strong.energy_units[0].attachment == Attachment::Held(owner) {
                break;
            }
        }
        assert_eq!(strong.energy_units[0].attachment, Attachment::Held(owner));
        assert_eq!(strong.energy_units[0].physical.id, id);
        let unit = &strong.energy_units[0];
        let at = PayloadSnapshot::attached_pose(strong.wrist(owner).unwrap(), unit.grasp);
        assert!(
            DVec3::from_array(at.translation_m)
                .distance(DVec3::from_array(unit.physical.pose.translation_m))
                < 1e-10
        );
    }
    #[test]
    fn mid_pull_restore_continues_identically_and_release_returns_without_teleport() {
        let (mut field, owner) = fixture(20., 25.);
        move_wrist(&mut field, owner, 0.08);
        field.advance_supply_grip(0, owner);
        assert!(field.energy_units[0].supply.as_ref().unwrap().pull.travel_m > 0.);
        assert_eq!(
            field.energy_units[0].attachment,
            Attachment::Extracting(owner)
        );
        let geometry = field.static_geometry_snapshot();
        let mut restored = Field::restore(&field.snapshot(), &geometry, 0.).unwrap();
        for _ in 0..4 {
            field.advance_supply_grip(0, owner);
            restored.advance_supply_grip(0, owner);
        }
        assert_eq!(restored.energy_units(), field.energy_units());

        let (mut field, owner) = fixture(20., 25.);
        move_wrist(&mut field, owner, 0.08);
        field.advance_supply_grip(0, owner);
        let at = field.energy_units[0].physical.pose;
        field.toggle_energy_grip(owner).unwrap();
        assert_eq!(field.energy_units[0].physical.pose, at);
        assert_eq!(field.energy_units[0].attachment, Attachment::Mounted);
        for _ in 0..128 {
            field.return_supply_unit(0);
        }
        assert_eq!(
            field.energy_units[0].supply.as_ref().unwrap().pull,
            PullOffState::default()
        );
    }
    #[test]
    fn station_admission_is_atomic_and_spent_units_cannot_be_stock() {
        let mut field = Field::new(&Default::default()).unwrap();
        let shape = PayloadShape::from_parts(
            &[rm_sim_physics::manipulator_collision::CollisionPart::Box { half_m: [0.03; 3] }],
            0.4,
        )
        .unwrap();
        let station = std::array::from_fn(|i| {
            (
                100 + i as u32,
                SupplyMount::new(Pose::at([i as f64, 0., 1.])),
            )
        });
        let mut invalid = station.clone();
        invalid[5].0 = invalid[0].0;
        assert!(field.add_supply_station(invalid, shape.clone()).is_err());
        assert!(field.energy_units().is_empty());
        field.add_supply_station(station, shape).unwrap();
        assert_eq!(field.energy_units().len(), 6);
        assert!(
            field
                .physics
                .dynamic_geometry_snapshot()
                .cast_ray([0., 0., 2.], [0., 0., -1.], 1.5)
                .unwrap()
                .is_some(),
            "mounted stock must remain query-visible"
        );
        let geometry = field.static_geometry_snapshot();
        let restored = Field::restore(&field.snapshot(), &geometry, 0.).unwrap();
        assert_eq!(restored.energy_units(), field.energy_units());
        field.energy_units[0].spent = true;
        assert!(Field::restore(&field.snapshot(), &geometry, 0.).is_err());
        let ids: Vec<_> = field.energy_units.iter().map(|u| u.physical.id).collect();
        field.reset_supply_units().unwrap();
        assert!(
            field
                .energy_units
                .iter()
                .all(|u| !u.spent && u.attachment == Attachment::Mounted)
        );
        assert_eq!(
            field
                .energy_units
                .iter()
                .map(|u| u.physical.id)
                .collect::<Vec<_>>(),
            ids
        );
    }

    #[test]
    fn insufficient_magnetic_support_drops_a_downward_unit_without_a_grip() {
        let (mut field, _) = fixture(20., 25.);
        let unit = &mut field.energy_units[0];
        unit.attachment = Attachment::Mounted;
        let mount = unit.supply.as_mut().unwrap();
        mount.limits.retention_n = 0.;
        let rotation = DQuat::from_rotation_x(std::f64::consts::PI);
        mount.pose.rotation_wxyz = [rotation.w, rotation.x, rotation.y, rotation.z];
        write_endpoint(unit, &mut field.physics);
        for _ in 0..128 {
            field.return_supply_unit(0);
            if field.energy_units[0].attachment == Attachment::Free {
                break;
            }
        }
        let unit = &field.energy_units[0];
        assert_eq!(unit.attachment, Attachment::Free);
        assert!(unit.physical.velocity_m_s[2] < 0.);
        assert!(!unit.physical.prescribed);
    }
}
