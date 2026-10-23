// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-owned attachment transforms and motion, without grip or seating policy.
use super::PayloadSnapshot;
use crate::{Pose, StepDuration, WorldPhysics};
use rapier3d_f64::prelude::{Rotation, Vector};

fn rotation(p: Pose) -> Rotation {
    let [w, x, y, z] = p.rotation_wxyz;
    Rotation::from_xyzw(x, y, z, w)
}

impl PayloadSnapshot {
    /// Compose a world attachment frame with an attachment-to-object offset.
    /// Inputs use FLU metres and normalized wxyz quaternions. No snapping,
    /// collision query or permission check is performed.
    ///
    /// ```
    /// use rm_sim_physics::{Pose, projectile::PayloadSnapshot};
    /// let at = PayloadSnapshot::attached_pose(Pose::at([1., 0., 0.]), Pose::at([0., 0., 0.2]));
    /// assert_eq!(at.translation_m, [1., 0., 0.2]);
    /// ```
    pub fn attached_pose(frame: Pose, offset: Pose) -> Pose {
        let r = rotation(frame) * rotation(offset);
        Pose {
            translation_m: (Vector::from_array(frame.translation_m)
                + rotation(frame) * Vector::from_array(offset.translation_m))
            .to_array(),
            rotation_wxyz: [r.w, r.x, r.y, r.z],
        }
    }

    /// Capture an attachment-to-object offset without changing the world pose.
    /// Inputs use FLU metres and normalized wxyz quaternions. The caller retains
    /// this offset and decides when the attachment exists.
    ///
    /// ```
    /// use rm_sim_physics::{Pose, projectile::PayloadSnapshot};
    /// let frame = Pose::at([1., 0., 0.]);
    /// let at = Pose::at([1., 0., 0.2]);
    /// let offset = PayloadSnapshot::attachment_offset(frame, at);
    /// assert_eq!(PayloadSnapshot::attached_pose(frame, offset), at);
    /// ```
    pub fn attachment_offset(frame: Pose, at: Pose) -> Pose {
        let r = rotation(frame).inverse();
        let inverse = Pose {
            translation_m: (r * -Vector::from_array(frame.translation_m)).to_array(),
            rotation_wxyz: [r.w, r.x, r.y, r.z],
        };
        Self::attached_pose(inverse, at)
    }

    /// Synchronize an installed payload and this caller-owned record to a resolved pose.
    /// Linear velocity is the world object-origin displacement in metres divided
    /// by `step`; angular velocity is the shortest quaternion delta in radians
    /// divided by `step`. Compute the object pose after applying its attachment
    /// offset, so rotating offsets contribute to release linear velocity.
    /// `prescribed = false` releases at that endpoint with those velocities.
    /// The previous record must describe the preceding attachment endpoint;
    /// free-body truth must be refreshed before beginning a new attachment.
    /// Retain this record's endpoint velocities until release: kinematic solver
    /// telemetry can report different velocities between integration steps.
    /// Failed motion validation leaves this record unchanged.
    ///
    /// ```
    /// use rm_sim_physics::{Pose, StepDuration, WorldPhysics, projectile::{PayloadShape, PayloadSnapshot},
    ///     manipulator_collision::CollisionPart};
    /// # let shape = PayloadShape::from_parts(&[CollisionPart::Box { half_m: [0.03; 3] }], 0.25)?;
    /// let mut object = PayloadSnapshot { id: 7, shape, pose: Pose::at([0., 0., 1.]),
    ///     velocity_m_s: [0.; 3], angular_velocity_rad_s: [0.; 3], prescribed: true };
    /// let mut physics = WorldPhysics::new(&[], 0.);
    /// physics.add_payload(&object)?;
    /// object.sync_attached_pose(&mut physics, Pose::at([0.001, 0., 1.]), true, StepDuration::HZ_1000)?;
    /// object.release_attachment(&mut physics)?;
    /// assert_eq!(physics.payload_telemetry(7).unwrap().velocity_m_s, [1., 0., 0.]);
    /// assert!(!object.prescribed);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn sync_attached_pose(
        &mut self,
        physics: &mut WorldPhysics,
        at: Pose,
        prescribed: bool,
        step: StepDuration,
    ) -> Result<(), &'static str> {
        let dt = step.as_secs_f64();
        let velocity = ((Vector::from_array(at.translation_m)
            - Vector::from_array(self.pose.translation_m))
            / dt)
            .to_array();
        let angular =
            ((rotation(at) * rotation(self.pose).inverse()).to_scaled_axis() / dt).to_array();
        physics.set_payload_motion(self.id, prescribed, at, velocity, angular)?;
        self.pose = at;
        self.prescribed = prescribed;
        self.velocity_m_s = velocity;
        self.angular_velocity_rad_s = angular;
        Ok(())
    }

    /// Release an installed attachment using this record's last pose and world velocities.
    /// Does not recompute velocity at the attachment origin or discard angular
    /// velocity. The caller decides whether release is permitted. For an already
    /// dynamic body, the existing physics motion is left untouched by the setter.
    /// See [`Self::sync_attached_pose`] for a runnable carry/release example.
    pub fn release_attachment(&mut self, physics: &mut WorldPhysics) -> Result<(), &'static str> {
        physics.set_payload_motion(
            self.id,
            false,
            self.pose,
            self.velocity_m_s,
            self.angular_velocity_rad_s,
        )?;
        self.prescribed = false;
        Ok(())
    }

    /// Refresh pose and motion from installed physics without cloning hull buffers.
    /// Unknown identities return an error and leave this record unchanged.
    /// Geometry and identity remain caller-owned and are not replaced.
    ///
    /// ```
    /// # use rm_sim_physics::{Pose, WorldPhysics, projectile::{PayloadShape, PayloadSnapshot}, manipulator_collision::CollisionPart};
    /// # let shape = PayloadShape::from_parts(&[CollisionPart::Box { half_m: [0.03; 3] }], 0.25)?;
    /// # let mut object = PayloadSnapshot { id: 1, shape, pose: Pose::at([0., 0., 1.]), velocity_m_s: [0.; 3], angular_velocity_rad_s: [0.; 3], prescribed: true };
    /// # let mut physics = WorldPhysics::new(&[], 0.);
    /// physics.add_payload(&object)?;
    /// object.refresh_motion(&physics)?;
    /// assert_eq!(object.pose, physics.payload_telemetry(1).unwrap().pose);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn refresh_motion(&mut self, physics: &WorldPhysics) -> Result<(), &'static str> {
        let truth = physics
            .payload_telemetry(self.id)
            .ok_or("missing payload")?;
        self.pose = truth.pose;
        self.velocity_m_s = truth.velocity_m_s;
        self.angular_velocity_rad_s = truth.angular_velocity_rad_s;
        self.prescribed = truth.prescribed;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{TargetFrames, manipulator_collision::CollisionPart, projectile::PayloadShape};

    fn object() -> PayloadSnapshot {
        PayloadSnapshot {
            id: 8,
            shape: PayloadShape::from_parts(&[CollisionPart::Box { half_m: [0.03; 3] }], 0.25)
                .unwrap(),
            pose: Pose::at([1., 0., 2.]),
            velocity_m_s: [0.; 3],
            angular_velocity_rad_s: [0.; 3],
            prescribed: true,
        }
    }

    #[test]
    fn offset_rotation_releases_object_origin_velocity_and_restores_without_policy() {
        let mut object = object();
        let mut physics = WorldPhysics::new(&[], 0.);
        physics.add_payload(&object).unwrap();
        let hull = object.shape.hulls_m[0].as_ptr();
        let physical_hull = physics.payload_telemetry(8).unwrap().shape.hulls_m[0].as_ptr();
        let frame = Pose::at([0., 0., 2.]);
        let offset = PayloadSnapshot::attachment_offset(frame, object.pose);
        assert_eq!(offset.translation_m, [1., 0., 0.]);
        let dt = StepDuration::HZ_128.as_secs_f64();
        let angle = 0.01;
        let at = PayloadSnapshot::attached_pose(Pose::yawed(frame.translation_m, angle), offset);
        object
            .sync_attached_pose(&mut physics, at, true, StepDuration::HZ_128)
            .unwrap();
        let expected = [(angle.cos() - 1.) / dt, angle.sin() / dt, 0.];
        assert_eq!(object.velocity_m_s, expected);
        assert!((object.angular_velocity_rad_s[2] - angle / dt).abs() < 1e-14);
        let before = object.clone();
        object.release_attachment(&mut physics).unwrap();
        let released = physics.payload(8).unwrap();
        assert_eq!(released.pose, before.pose);
        assert_eq!(released.velocity_m_s, before.velocity_m_s);
        assert_eq!(
            released.angular_velocity_rad_s,
            before.angular_velocity_rad_s
        );
        assert!(!released.prescribed);
        assert_eq!(object.shape.hulls_m[0].as_ptr(), hull);
        assert_eq!(
            physics.payload_telemetry(8).unwrap().shape.hulls_m[0].as_ptr(),
            physical_hull
        );
        let mut restored = WorldPhysics::new(&[], 0.);
        restored.add_payload(&released).unwrap();
        assert_eq!(restored.payload(8).unwrap(), released);
        physics.step(0, &TargetFrames::new(vec![])).unwrap();
        restored.step(0, &TargetFrames::new(vec![])).unwrap();
        assert_eq!(physics.payload(8), restored.payload(8));
        object.refresh_motion(&physics).unwrap();
        let frame = Pose::yawed([0.2, -0.3, 2.], 0.7);
        let offset = PayloadSnapshot::attachment_offset(frame, object.pose);
        let recaptured = PayloadSnapshot::attached_pose(frame, offset);
        for (actual, expected) in recaptured
            .translation_m
            .into_iter()
            .zip(object.pose.translation_m)
        {
            assert!((actual - expected).abs() < 1e-15);
        }
    }

    #[test]
    fn endpoint_release_uses_same_delta_and_failed_updates_leave_record_unchanged() {
        let mut object = object();
        let mut physics = WorldPhysics::new(&[], 0.);
        physics.add_payload(&object).unwrap();
        object
            .sync_attached_pose(
                &mut physics,
                Pose::at([1.001, 0., 2.]),
                false,
                StepDuration::HZ_1000,
            )
            .unwrap();
        assert!(!object.prescribed);
        assert_eq!(object.velocity_m_s, [(1.001 - 1.) / 0.001, 0., 0.]);
        assert_eq!(
            physics.payload(8).unwrap().velocity_m_s,
            object.velocity_m_s
        );
        let before = object.clone();
        assert!(
            object
                .sync_attached_pose(
                    &mut physics,
                    Pose::at([f64::NAN, 0., 0.]),
                    true,
                    StepDuration::HZ_128
                )
                .is_err()
        );
        assert_eq!(object, before);
        let mut absent = object.clone();
        absent.id = 99;
        let before = absent.clone();
        assert_eq!(absent.refresh_motion(&physics), Err("missing payload"));
        assert_eq!(
            absent.release_attachment(&mut physics),
            Err("missing payload")
        );
        assert_eq!(absent, before);
    }
}
