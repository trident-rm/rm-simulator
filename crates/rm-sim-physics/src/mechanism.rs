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
    /// use rm_sim_physics::{chassis::ChassisConfig, Pose};
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
    /// use rm_sim_physics::chassis::ChassisConfig;
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

/// Source-fitted rigid joint chain. All reference mounts are body-local FLU metres.
/// One primary input drives a prescribed stow sequence; no actuator torque is inferred.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArticulationConfig {
    /// Fitted muzzle offset from the pitch pivot, local FLU metres.
    pub muzzle_offset_m: [f64; 3],
    /// Fitted camera eye offset from the pitch pivot, local FLU metres.
    pub camera_offset_m: [f64; 3],
    /// Package profile identity retained in snapshots and used by asset adapters.
    pub profile: String,
    /// Maximum primary input, radians; zero for a fixed-height source robot.
    pub limit_rad: f64,
    /// Prescribed primary input rate, radians per second; positive.
    pub speed_rad_s: f64,
    /// Topologically ordered joints. Parents must precede children. A joint named
    /// `pitch` is the firing hinge: its composed pose carries the muzzle and eye.
    /// Without it, the chassis retains its fixed turret-center mounting convention.
    pub joints: Vec<SourceJoint>,
    /// Four source-fitted armor scoring face poses, body-local FLU.
    pub armor_faces: [Pose; 4],
}
/// Rigid source group driven by a parent, the primary transform, or gimbal aim.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SourceJoint {
    /// Source GLB motion-group name, unique within this configuration.
    pub name: String,
    /// Earlier joint index, or no parent for a body attachment.
    pub parent: Option<usize>,
    /// Reference hinge position, body-local FLU metres.
    pub pivot_m: [f64; 3],
    /// Unit rotation axis in the reference body frame.
    pub axis: [f64; 3],
    /// Rotation at the stowed endpoint, radians (or gain for an aim joint).
    pub angle_rad: f64,
    /// Translation at the stowed endpoint, in reference body FLU metres.
    pub translation_m: [f64; 3],
    /// Input source: primary transform, body-relative yaw, or body-relative pitch.
    pub drive: JointDrive,
    /// Optional conservative source envelope, relative to the reference hinge.
    pub collision: Option<JointEnvelope>,
}
/// Input used by a source joint.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum JointDrive {
    /// Normalized primary input, from zero to one.
    Transform,
    /// Body-relative gimbal yaw, radians.
    Yaw,
    /// Body-relative gimbal elevation, radians.
    Pitch,
}
/// Cuboid envelope of a source group, in its hinge-local reference frame.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointEnvelope {
    /// Cuboid rotation relative to its reference hinge, wxyz unit quaternion.
    pub rotation_wxyz: [f64; 4],
    /// Centre relative to its hinge, metres.
    pub center_m: [f64; 3],
    /// Positive half extents, metres.
    pub half_m: [f64; 3],
}
impl ArticulationConfig {
    /// Reject cycles, duplicate names, nonunit axes and nonfinite source geometry.
    pub fn validate(&self) -> Result<(), &'static str> {
        let finite = |v: [f64; 3]| v.into_iter().all(f64::is_finite);
        if !finite(self.muzzle_offset_m)
            || !finite(self.camera_offset_m)
            || self.profile.is_empty()
            || self.joints.is_empty()
            || self.joints.len() > 64
            || !self.limit_rad.is_finite()
            || !(0. ..=std::f64::consts::PI).contains(&self.limit_rad)
            || !self.speed_rad_s.is_finite()
            || self.speed_rad_s <= 0.
        {
            return Err("invalid source articulation input");
        }
        let mut names = std::collections::HashSet::new();
        for (i, j) in self.joints.iter().enumerate() {
            if j.name.is_empty()
                || !names.insert(&j.name)
                || j.parent.is_some_and(|p| p >= i)
                || !finite(j.pivot_m)
                || !finite(j.axis)
                || !finite(j.translation_m)
                || (Vector::from_array(j.axis).length() - 1.).abs() > 1e-6
                || !j.angle_rad.is_finite()
                || j.collision.is_some_and(|c| {
                    !finite(c.center_m)
                        || c.half_m.into_iter().any(|v| !v.is_finite() || v <= 0.)
                        || !c.rotation_wxyz.into_iter().all(f64::is_finite)
                        || (c.rotation_wxyz.into_iter().map(|v| v * v).sum::<f64>() - 1.).abs()
                            > 1e-6
                })
            {
                return Err("invalid source articulation joint");
            }
        }
        for p in self.armor_faces {
            if !finite(p.translation_m)
                || !p.rotation_wxyz.into_iter().all(f64::is_finite)
                || (p.rotation_wxyz.into_iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-6
            {
                return Err("invalid source armor mount");
            }
        }
        Ok(())
    }
    /// Body-local rigid frames for the bounded primary input and local gimbal aim.
    /// Validate the graph before evaluation; parent frames are composed exactly.
    ///
    /// ```
    /// use rm_sim_physics::mechanism::*;
    /// # let config = ArticulationConfig {
    /// #   profile: "example".into(), limit_rad: 1., speed_rad_s: 0.5,
    /// #   muzzle_offset_m: [0.2,0.,0.], camera_offset_m: [0.1,0.,0.],
    /// #   armor_faces: rm_sim_physics::chassis::ChassisConfig::default().armor_faces(),
    /// #   joints: vec![SourceJoint { name: "camera".into(), parent: None,
    /// #     pivot_m: [0.,0.,0.2], axis: [0.,1.,0.], angle_rad: 0.,
    /// #     translation_m: [0.,0.,-0.05], drive: JointDrive::Transform, collision: None }],
    /// # };
    /// config.validate()?;
    /// let stowed = config.poses(1., [0., 0.]);
    /// assert!((stowed[0].translation_m[2] - 0.15).abs() < 1e-12);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn poses(&self, input_rad: f64, aim_rad: [f64; 2]) -> Vec<Pose> {
        let mut deltas: Vec<Pose3> = Vec::with_capacity(self.joints.len());
        self.joints
            .iter()
            .map(|j| {
                let pivot = Pose3::from_translation(Vector::from_array(j.pivot_m));
                let delta = self.joint_delta(
                    j,
                    j.parent.map_or(Pose3::IDENTITY, |p| deltas[p]),
                    input_rad,
                    aim_rad,
                );
                deltas.push(delta);
                pose(delta * pivot)
            })
            .collect()
    }
    /// Evaluate one frame without allocating the full pose chain. The graph must
    /// already be validated: chassis construction enforces <=64 topologically ordered joints.
    pub(crate) fn joint_pose(&self, index: usize, input_rad: f64, aim_rad: [f64; 2]) -> Pose {
        fn delta(
            config: &ArticulationConfig,
            index: usize,
            input_rad: f64,
            aim_rad: [f64; 2],
        ) -> Pose3 {
            let joint = &config.joints[index];
            let parent = joint
                .parent
                .map_or(Pose3::IDENTITY, |p| delta(config, p, input_rad, aim_rad));
            config.joint_delta(joint, parent, input_rad, aim_rad)
        }
        pose(
            delta(self, index, input_rad, aim_rad)
                * Pose3::from_translation(Vector::from_array(self.joints[index].pivot_m)),
        )
    }

    fn joint_delta(
        &self,
        joint: &SourceJoint,
        parent: Pose3,
        input_rad: f64,
        aim_rad: [f64; 2],
    ) -> Pose3 {
        let value = match joint.drive {
            JointDrive::Transform => {
                if self.limit_rad > 0. {
                    (input_rad / self.limit_rad).clamp(0., 1.)
                } else {
                    0.
                }
            }
            JointDrive::Yaw => aim_rad[0],
            JointDrive::Pitch => aim_rad[1],
        };
        let pivot = Pose3::from_translation(Vector::from_array(joint.pivot_m));
        let rotation =
            Rotation::from_scaled_axis(Vector::from_array(joint.axis) * (joint.angle_rad * value));
        let change = Pose3::from_parts(Vector::from_array(joint.translation_m) * value, rotation);
        parent * pivot * change * pivot.inverse()
    }
    /// Body-local conservative envelopes in the same poses as their source shells.
    pub fn collision_parts(&self, input_rad: f64, aim_rad: [f64; 2]) -> Vec<(Pose, [f64; 3])> {
        self.joints
            .iter()
            .zip(self.poses(input_rad, aim_rad))
            .filter_map(|(j, p)| {
                j.collision.map(|c| {
                    (
                        pose(
                            rigid(p)
                                * rigid(Pose {
                                    translation_m: c.center_m,
                                    rotation_wxyz: c.rotation_wxyz,
                                }),
                        ),
                        c.half_m,
                    )
                })
            })
            .collect()
    }
}

#[cfg(test)]
mod serial_tests {
    use super::*;
    fn fixture() -> ArticulationConfig {
        ArticulationConfig {
            muzzle_offset_m: [0.2, 0., 0.],
            camera_offset_m: [0.1, 0., 0.],
            profile: "test".into(),
            limit_rad: 1.,
            speed_rad_s: 0.5,
            armor_faces: crate::chassis::ChassisConfig::default().armor_faces(),
            joints: vec![
                SourceJoint {
                    name: "arm".into(),
                    parent: None,
                    pivot_m: [0., 0., 0.2],
                    axis: [0., 1., 0.],
                    angle_rad: std::f64::consts::FRAC_PI_2,
                    translation_m: [0.; 3],
                    drive: JointDrive::Transform,
                    collision: Some(JointEnvelope {
                        center_m: [0.1, 0., 0.],
                        half_m: [0.1, 0.02, 0.02],
                        rotation_wxyz: [1., 0., 0., 0.],
                    }),
                },
                SourceJoint {
                    name: "carriage".into(),
                    parent: Some(0),
                    pivot_m: [0.2, 0., 0.2],
                    axis: [0., 0., 1.],
                    angle_rad: 0.,
                    translation_m: [0., 0., -0.1],
                    drive: JointDrive::Transform,
                    collision: None,
                },
            ],
        }
    }
    #[test]
    fn serial_motion_composes_parent_rotation_and_carriage_translation() {
        let c = fixture();
        c.validate().unwrap();
        let p = c.poses(1., [0.; 2]);
        for (a, b) in p[1].translation_m.into_iter().zip([-0.1, 0., 0.]) {
            assert!((a - b).abs() < 1e-12);
        }
        let b = c.collision_parts(1., [0.; 2]);
        assert_eq!(b.len(), 1);
        assert!((b[0].0.translation_m[2] - 0.1).abs() < 1e-12);
        assert_eq!(c.poses(2., [0.; 2]), p);
    }
    #[test]
    fn single_joint_matches_full_chain_for_nested_branches_and_maximum_depth() {
        let mut c = fixture();
        c.joints[0].pivot_m = [0.12, -0.25, 0.3];
        c.joints[0].translation_m = [0.01, 0.02, -0.03];
        c.joints[1].drive = JointDrive::Yaw;
        c.joints[1].angle_rad = 1.;
        let mut branch = c.joints[1].clone();
        branch.name = "branch".into();
        branch.parent = Some(0);
        branch.pivot_m = [-0.1, 0.24, 0.17];
        branch.drive = JointDrive::Pitch;
        c.joints.push(branch);
        // Extend one branch to the maximum admitted depth while retaining the sibling.
        for index in 3..64 {
            let mut joint = c.joints[2].clone();
            joint.name = format!("joint-{index}");
            joint.parent = Some(index - 1);
            joint.pivot_m[0] += index as f64 * 0.001;
            c.joints.push(joint);
        }
        c.validate().unwrap();
        for input in [-0.5, 0., 0.4, 1., 2.] {
            for aim in [[0.; 2], [0.6, -0.2], [-1., 0.5]] {
                for (index, expected) in c.poses(input, aim).into_iter().enumerate() {
                    assert_eq!(c.joint_pose(index, input, aim), expected);
                }
            }
        }
    }
    #[test]
    fn invalid_graph_and_collision_frames_are_rejected() {
        let mut c = fixture();
        c.joints[0].parent = Some(1);
        assert!(c.validate().is_err());
        let mut c = fixture();
        c.joints[1].name = "arm".into();
        assert!(c.validate().is_err());
        let mut c = fixture();
        c.joints[0].axis = [0.; 3];
        assert!(c.validate().is_err());
        let mut c = fixture();
        c.joints[0].collision.as_mut().unwrap().rotation_wxyz = [0.; 4];
        assert!(c.validate().is_err());
    }
    #[test]
    fn gimbal_drives_are_independent_from_primary_input() {
        let mut c = fixture();
        c.joints[0].drive = JointDrive::Yaw;
        assert_eq!(c.poses(0., [0.; 2])[0], c.poses(1., [0.; 2])[0]);
        assert_ne!(c.poses(0., [0.; 2])[0], c.poses(0., [0.5, 0.])[0]);
    }
    #[test]
    fn source_pitch_muzzle_follows_the_moving_hinge_and_survives_restore() {
        use crate::{TargetFrames, Team, WorldPhysics, chassis::ChassisConfig, tick_ns};
        let mut source = fixture();
        source.joints[1].name = "pitch".into();
        let config = ChassisConfig {
            articulation: Some(source.clone()),
            omni_frame: None,
            ..Default::default()
        };
        let mut world = WorldPhysics::new(&[], 0.);
        let id = world
            .add_chassis(Team::Red, config, Pose::at([0., 0., 3.]))
            .unwrap();
        let mut snapshot = world.chassis_snapshot(id).unwrap();
        snapshot.fold_angle_rad = 0.7;
        world = WorldPhysics::new(&[], 0.);
        world.restore_chassis(&snapshot).unwrap();
        world.step(tick_ns(), &TargetFrames::new(vec![])).unwrap();
        let snapshot = world.chassis_snapshot(id).unwrap();
        let head = rigid(snapshot.pose) * rigid(source.poses(snapshot.fold_angle_rad, [0.; 2])[1]);
        let expected = head * rigid(Pose::at(source.muzzle_offset_m));
        let actual = rigid(world.chassis_muzzle_pose(id).unwrap());
        assert!((expected.translation - actual.translation).length() < 1e-8);
        assert!(expected.rotation.dot(actual.rotation).abs() > 1. - 1e-12);
        assert!((rigid(snapshot.turret).translation - head.translation).length() < 1e-8);
    }
    #[test]
    fn serial_chassis_restore_replays_the_current_primary_input() {
        use crate::{
            TargetFrames, Team, WorldPhysics,
            chassis::{ChassisCommand, ChassisConfig},
            tick_ns,
        };
        let cfg = ChassisConfig {
            articulation: Some(fixture()),
            omni_frame: None,
            ..Default::default()
        };
        cfg.validate().unwrap();
        let mut a = WorldPhysics::new(&[], 0.);
        let id = a
            .add_chassis(Team::Red, cfg, Pose::at([0., 0., 3.]))
            .unwrap();
        a.command_chassis(
            id,
            ChassisCommand {
                folded: true,
                ..Default::default()
            },
        )
        .unwrap();
        let frames = TargetFrames::new(vec![]);
        for t in 0..40 {
            a.step(t * tick_ns(), &frames).unwrap();
        }
        let snapshot = a.chassis_snapshot(id).unwrap();
        let mut b = WorldPhysics::new(&[], 0.);
        b.restore_chassis(&snapshot).unwrap();
        for t in 40..70 {
            a.step(t * tick_ns(), &frames).unwrap();
            b.step(t * tick_ns(), &frames).unwrap();
        }
        let x = a.chassis_snapshot(id).unwrap();
        let y = b.chassis_snapshot(id).unwrap();
        assert_eq!(x.fold_angle_rad, y.fold_angle_rad);
        for (a, b) in x.pose.translation_m.into_iter().zip(y.pose.translation_m) {
            assert!((a - b).abs() < 1e-8);
        }
    }
}
