// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Prescribed folding linkage geometry, shared by collision, muzzle and drawing callers.
use crate::Pose;
use rapier3d_f64::math::{Pose as Pose3, Rotation, Vector};
use serde::{Deserialize, Serialize};

/// A folding arm with a counter-rotating firing stage. All mounts are body FLU metres.
/// This is a kinematic actuator, not a torque, gearbox or flexible ammunition-chain model.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FoldingConfig {
    /// Body-relative yaw bearing centre in metres.
    pub yaw_pivot_m: [f64; 3],
    /// Body-relative lower folding hinge centre in metres.
    pub lower_pivot_m: [f64; 3],
    /// Body-relative upper firing hinge centre in the unfolded pose, in metres.
    pub upper_pivot_m: [f64; 3],
    /// Arm proxy centre relative to the lower hinge in its unfolded frame, metres.
    pub arm_center_m: [f64; 3],
    /// Arm collision cuboid half extents in metres.
    pub arm_half_m: [f64; 3],
    /// Firing-stage proxy centre relative to its hinge in metres.
    pub head_center_m: [f64; 3],
    /// Firing-stage collision cuboid half extents in metres.
    pub head_half_m: [f64; 3],
    /// Requested folded angle about the yaw frame's +left, radians; zero is unfolded.
    pub folded_angle_rad: f64,
    /// Prescribed actuator speed in radians per second; positive.
    pub speed_rad_s: f64,
    /// Barrel exit relative to the firing hinge, in local FLU metres.
    pub muzzle_offset_m: [f64; 3],
    /// Approximate lens front relative to the firing hinge, in local FLU metres.
    pub camera_offset_m: [f64; 3],
}
/// World FLU poses of the three source motion groups at an explicit mechanism angle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FoldingPoses {
    /// Yaw bearing pose, before folding or firing pitch.
    pub yaw: Pose,
    /// Lower folding hinge pose, including its current angle.
    pub arm: Pose,
    /// Upper firing hinge pose, counter-rotated to maintain the requested aim.
    pub head: Pose,
}
fn rigid(p: Pose) -> Pose3 {
    let [w, x, y, z] = p.rotation_wxyz;
    Pose3::from_parts(
        Vector::from_array(p.translation_m),
        Rotation::from_xyzw(x, y, z, w),
    )
}
fn pose(p: Pose3) -> Pose {
    Pose {
        translation_m: p.translation.to_array(),
        rotation_wxyz: [p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z],
    }
}
impl FoldingConfig {
    /// Check finite mounts, positive cuboid sizes/rate and a fold angle in (0, pi].
    pub fn validate(&self) -> Result<(), &'static str> {
        if ![
            self.yaw_pivot_m,
            self.lower_pivot_m,
            self.upper_pivot_m,
            self.arm_center_m,
            self.head_center_m,
            self.muzzle_offset_m,
            self.camera_offset_m,
        ]
        .into_iter()
        .flatten()
        .all(f64::is_finite)
            || !self
                .arm_half_m
                .into_iter()
                .chain(self.head_half_m)
                .all(|v| v.is_finite() && v > 0.)
            || !self.speed_rad_s.is_finite()
            || self.speed_rad_s <= 0.
            || !self.folded_angle_rad.is_finite()
            || self.folded_angle_rad <= 0.
            || self.folded_angle_rad > std::f64::consts::PI
        {
            return Err("invalid folding mechanism geometry or actuator limits");
        }
        Ok(())
    }
    /// Evaluate the linkage without advancing it. The head holds its aim while its hinge moves.
    ///
    /// ```
    /// use rm_simulator_physics::{chassis::ChassisConfig, Pose};
    /// let config = ChassisConfig::folding_hero();
    /// let linkage = config.folding.as_ref().unwrap();
    /// let upright = linkage.poses(Pose::default(), [0., 0.], 0.);
    /// let folded = linkage.poses(Pose::default(), [0., 0.], linkage.folded_angle_rad);
    /// assert!(folded.head.translation_m[2] < upright.head.translation_m[2]);
    /// assert_eq!(folded.head.rotation_wxyz, upright.head.rotation_wxyz);
    /// ```
    pub fn poses(&self, body: Pose, aim_rad: [f64; 2], angle_rad: f64) -> FoldingPoses {
        let body = rigid(body);
        let desired =
            Rotation::from_rotation_z(aim_rad[0]) * Rotation::from_rotation_y(-aim_rad[1]);
        let direction = body.rotation.inverse() * (desired * Vector::X);
        let yaw = Rotation::from_rotation_z(direction.y.atan2(direction.x));
        let pitch = direction.z.atan2(direction.x.hypot(direction.y));
        let base = body * Pose3::from_parts(Vector::from_array(self.yaw_pivot_m), yaw);
        let lower = Vector::from_array(self.lower_pivot_m) - Vector::from_array(self.yaw_pivot_m);
        let arm = base * Pose3::from_parts(lower, Rotation::from_rotation_y(angle_rad));
        let upper = arm.transform_point(
            Vector::from_array(self.upper_pivot_m) - Vector::from_array(self.lower_pivot_m),
        );
        FoldingPoses {
            yaw: pose(base),
            arm: pose(arm),
            head: pose(Pose3::from_parts(
                upper,
                base.rotation * Rotation::from_rotation_y(-pitch),
            )),
        }
    }
    /// The two collision cuboid poses and half extents at an explicit angle, in world FLU.
    pub fn collision_parts(
        &self,
        body: Pose,
        aim_rad: [f64; 2],
        angle_rad: f64,
    ) -> [(Pose, [f64; 3]); 2] {
        let poses = self.poses(body, aim_rad, angle_rad);
        [
            (
                pose(
                    rigid(poses.arm)
                        * Pose3::from_translation(Vector::from_array(self.arm_center_m)),
                ),
                self.arm_half_m,
            ),
            (
                pose(
                    rigid(poses.head)
                        * Pose3::from_translation(Vector::from_array(self.head_center_m)),
                ),
                self.head_half_m,
            ),
        ]
    }
}
