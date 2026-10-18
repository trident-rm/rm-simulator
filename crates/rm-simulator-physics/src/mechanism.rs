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

/// Closed single-input stair linkage, fitted to DreamChaser's source bearings.
/// Lengths follow the team's 2026 technical report Figures 2–3. Actuator speed
/// is a simulator assumption; this is prescribed geometry with effective tyres,
/// not a calibrated torque/gearbox or flexible suspension linkage simulation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ClimbingConfig {
    /// Fixed front hinge [forward, up] in body metres.
    pub front_pivot_m: [f64; 2],
    /// Fixed rear sliding-guide hinge [forward, up] in body metres.
    pub guide_m: [f64; 2],
    /// Height of the horizontal rear-top carriage above the body origin, metres.
    pub slider_height_m: f64,
    /// Front hinge to coupler pin length, metres (report Figure 3: 130 mm).
    pub front_pin_length_m: f64,
    /// Front hinge to wheel centre length, metres (report Figure 3: 190 mm).
    pub front_wheel_length_m: f64,
    /// Coupler pin-to-pin length, metres (report Figure 3: 200 mm).
    pub coupler_length_m: f64,
    /// Rear carriage to wheel centre length, metres (report Figure 3: 320 mm).
    pub rear_wheel_length_m: f64,
    /// Front-link absolute angle from downward at the level travel pose, radians.
    pub travel_angle_rad: f64,
    /// Front-link absolute angle at the raised-front endpoint, radians.
    pub deployed_angle_rad: f64,
    /// Maximum prescribed input angular speed, radians per second.
    pub speed_rad_s: f64,
    /// Left/right wheel offsets from the centre line, metres.
    pub wheel_left_m: [f64; 2],
    /// Spring anchor offset above the ideal linkage wheel centre, metres.
    pub spring_anchor_offset_m: f64,
}
/// Body-local closed-linkage frames and ideal wheel centres, before spring travel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClimbingPoses {
    /// Front arm at its fixed hinge; local -up points toward the wheel.
    pub front: Pose,
    /// Coupler at its front pin; local -up points toward its rear pin.
    pub coupler: Pose,
    /// Rear leg at the sliding upper carriage.
    pub rear: Pose,
    /// Fixed guide, following the rear leg's angle without its translation.
    pub guide: Pose,
    /// Rear upper carriage; translates horizontally without rotating.
    pub slider: Pose,
    /// Ideal wheel centres: front left/right then rear left/right, body FLU metres.
    pub hubs_m: [[f64; 3]; 4],
}
impl ClimbingConfig {
    /// Available input travel from level driving to raised-front stance, radians.
    pub fn travel_rad(&self) -> f64 {
        self.deployed_angle_rad - self.travel_angle_rad
    }
    /// Reject nonfinite dimensions, impossible closures and nonpositive rates.
    pub fn validate(&self) -> Result<(), &'static str> {
        if !self
            .front_pivot_m
            .into_iter()
            .chain(self.guide_m)
            .chain(self.wheel_left_m)
            .chain([
                self.slider_height_m,
                self.travel_angle_rad,
                self.deployed_angle_rad,
                self.spring_anchor_offset_m,
            ])
            .all(f64::is_finite)
            || ![
                self.front_pin_length_m,
                self.front_wheel_length_m,
                self.coupler_length_m,
                self.rear_wheel_length_m,
                self.speed_rad_s,
            ]
            .into_iter()
            .all(|v| v.is_finite() && v > 0.)
            || self.travel_rad() <= 0.
            || self.travel_rad() > std::f64::consts::PI
        {
            return Err("invalid climbing linkage dimensions or actuator rate");
        }
        for i in 0..=128 {
            let p = self.poses(self.travel_rad() * i as f64 / 128.);
            if !p.hubs_m.into_iter().flatten().all(f64::is_finite) {
                return Err("climbing linkage cannot close across its travel");
            }
        }
        Ok(())
    }
    /// Evaluate the exact planar closure. Input zero is level travel.
    ///
    /// ```
    /// use rm_simulator_physics::chassis::ChassisConfig;
    /// let config = ChassisConfig::stair_heavy();
    /// let linkage = config.climbing.as_ref().unwrap();
    /// let drive = linkage.poses(0.);
    /// let climb = linkage.poses(linkage.travel_rad());
    /// assert!((drive.hubs_m[0][2] - drive.hubs_m[2][2]).abs() < 1e-6);
    /// assert!(climb.hubs_m[0][2] - climb.hubs_m[2][2] > 0.27);
    /// ```
    pub fn poses(&self, input_rad: f64) -> ClimbingPoses {
        let theta = self.travel_angle_rad + input_rad;
        let a = Vector::new(self.front_pivot_m[0], 0., self.front_pivot_m[1]);
        let direction = Vector::new(-theta.sin(), 0., -theta.cos());
        let b = a + direction * self.front_pin_length_m;
        let height = b.z - self.slider_height_m;
        let c = Vector::new(
            b.x - (self.coupler_length_m.powi(2) - height * height).sqrt(),
            0.,
            self.slider_height_m,
        );
        let d = Vector::new(self.guide_m[0], 0., self.guide_m[1]);
        let rear_dir = (d - c).normalize();
        let rear_angle = (-rear_dir.x).atan2(-rear_dir.z);
        let bar_dir = c - b;
        let bar_angle = (-bar_dir.x).atan2(-bar_dir.z);
        let frame = |origin: Vector, angle| {
            pose(Pose3::from_parts(origin, Rotation::from_rotation_y(angle)))
        };
        let front_hub = a + direction * self.front_wheel_length_m;
        let rear_hub = c + rear_dir * self.rear_wheel_length_m;
        ClimbingPoses {
            front: frame(a, theta),
            coupler: frame(b, bar_angle),
            rear: frame(c, rear_angle),
            guide: frame(d, rear_angle),
            slider: frame(c, 0.),
            hubs_m: std::array::from_fn(|i| {
                let v = if i < 2 { front_hub } else { rear_hub };
                [v.x, self.wheel_left_m[i % 2], v.z]
            }),
        }
    }
    /// Conservative moving arm boxes, body-local, for external collision checks.
    /// Source self-contact and bearing shapes are deliberately excluded.
    pub fn collision_parts(&self, input_rad: f64) -> [(Pose, [f64; 3]); 4] {
        let p = self.poses(input_rad);
        std::array::from_fn(|i| {
            let (frame, length) = if i < 2 {
                (p.front, self.front_wheel_length_m)
            } else {
                (p.rear, self.rear_wheel_length_m)
            };
            let local =
                Pose3::from_translation(Vector::new(0., self.wheel_left_m[i % 2], -length / 2.));
            (pose(rigid(frame) * local), [0.012, 0.035, length / 2.])
        })
    }
}
