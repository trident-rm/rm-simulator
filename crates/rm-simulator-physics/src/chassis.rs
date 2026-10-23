// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! A four-wheel omni chassis driven on ray-cast wheels inside the field's rapier
//! world. The omni body is an octagonal deck and compact lower bumper, with a cuboid for the turret
//! that reaches down to the body top (so low roofs stop the chassis and
//! nothing slips between the two). Each wheel is a ray from its hub along
//! the body's -z that carries a spring/damper suspension and an ideal omni tyre:
//! independent motor torque, wheel inertia, slip-limited traction and a small roller
//! resistance along the axle, so the four wheels together move the body
//! holonomically. Ground is whatever the ray hits: the floor plane, the field
//! CAD (bumps, ramps, tunnels) or the tower proxies. Four small armor
//! modules ride the body sides as extra colliders, so projectiles strike
//! them where the body is at the moment of contact; a defeated robot's
//! drive is cut and its aim frozen.
//!
//! The rule manual constrains chassis power and envelope, not the mechanism;
//! the wheel, motor and suspension figures below are typical of a RoboMaster
//! infantry with 153 mm omni wheels and M3508 drives and are assumed, not
//! measured.
//!
//! Physical descriptions live in `model`; `backend` adapts them to Rapier.
//! These private modules retain the existing public `chassis::*` API.
//!
//! A model can supply the existing backend-neutral wheel equations without
//! creating a Rapier world:
//!
//! ```
//! use rm_simulator_physics::{chassis::ChassisConfig, wheel_model::WheelModel};
//!
//! let config = ChassisConfig::default();
//! config.validate()?;
//! let wheels = WheelModel::from(&config);
//! assert_eq!(wheels.wheel_radius_m, config.wheel_radius_m);
//! # Ok::<(), &'static str>(())
//! ```

mod backend;
mod model;

use crate::{
    Pose, Team,
    control::{DriveControl, WheelTorques},
};
pub(crate) use backend::Chassis;
pub use backend::PreparedChassis;
pub use model::{ChassisConfig, ChassisDynamics, OmniFrame, Tether};
use model::{WheelGeometry, wheel_geometry, wheel_geometry_at};
use rapier3d_f64::prelude::{PhysicsWorld, Pose3, Rotation, Vector};
use serde::{Deserialize, Serialize};

/// Armor modules per chassis: front, left, back and right, in that order.
pub const ARMOR_COUNT: usize = 4;
/// Gun muzzle offset from the stabilised turret pivot along its +x view line.
/// This 0.35 m barrel length is an application-level infantry geometry assumption.
pub const MUZZLE_FORWARD_M: f64 = 0.35;

/// Desired body-frame velocity and gun aim. The wheels are commanded from
/// the velocity by the omni inverse kinematics; the motors decide how much
/// of it they reach. The aim is where the stabilised gun pivot points in the
/// world; the gimbal is assumed to hold it exactly, so the turret pose in
/// the snapshot follows it at once.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChassisCommand {
    /// Request the deployed transform endpoint: folded Hero or raised-front stair stance.
    /// Fixed robots ignore it; this is a held target, not a repeated toggle.
    #[serde(default)]
    pub folded: bool,
    /// Desired body forward speed, in metres per second.
    pub forward_m_s: f64,
    /// Desired body left speed, in metres per second.
    pub left_m_s: f64,
    /// Counter-clockwise about the body's up axis.
    pub yaw_rate_rad_s: f64,
    /// Gun heading, counter-clockwise about world up from +x.
    #[serde(default)]
    pub aim_yaw_rad: f64,
    /// Gun elevation above the horizon.
    #[serde(default)]
    pub aim_pitch_rad: f64,
    /// Held jump request. Balance chassis launch once per press while supported;
    /// other chassis ignore it. Release before requesting another jump.
    #[serde(default)]
    pub jump: bool,
    /// Stabilization strength in percent, 0 (unassisted) through 100 (full LQR).
    /// Ignored by other chassis. Carried in input and restorable snapshots.
    #[serde(default = "full_balance_control")]
    pub balance_control: u8,
}
fn full_balance_control() -> u8 {
    100
}
impl Default for ChassisCommand {
    fn default() -> Self {
        Self {
            folded: false,
            forward_m_s: 0.,
            left_m_s: 0.,
            yaw_rate_rad_s: 0.,
            aim_yaw_rad: 0.,
            aim_pitch_rad: 0.,
            jump: false,
            balance_control: 100,
        }
    }
}
impl ChassisCommand {
    /// Validate numeric input before admitting it to a prediction history.
    pub fn is_finite(&self) -> bool {
        [
            self.forward_m_s,
            self.left_m_s,
            self.yaw_rate_rad_s,
            self.aim_yaw_rad,
            self.aim_pitch_rad,
        ]
        .iter()
        .all(|v| v.is_finite())
            && self.balance_control <= 100
    }
}

/// Compatibility export of the backend-neutral wheel contact record.
pub use crate::wheel_model::WheelContact;
/// One wheel's state at the end of a tick.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct WheelSnapshot {
    /// Suspended wheel centre in world FLU metres, including droop/compression.
    pub hub_m: [f64; 3],
    /// Accumulated rolling angle, wrapped to one turn. Positive travels along
    /// the configured rolling direction; physical axle rotation has opposite sign.
    pub spin_rad: f64,
    /// Actual axle speed, in radians per second; independent of body velocity.
    #[serde(default)]
    pub angular_velocity_rad_s: f64,
    /// Spring compression from full droop, in metres; zero when airborne.
    #[serde(default)]
    pub suspension_compression_m: f64,
    /// Surface speed the inverse kinematics asked of this wheel; zero in direct torque mode.
    pub target_m_s: f64,
    /// Ground contact this tick, or `None` when the ray found no ground.
    pub contact: Option<WheelContact>,
}
/// One chassis' observable state, complete enough to draw it or to rebuild a
/// prediction at a checkpoint.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChassisSnapshot {
    /// Primary transform input angle in radians (folding or stair linkage); zero for fixed robots.
    #[serde(default)]
    pub fold_angle_rad: f64,
    /// Changes on placement or revival, so clients never interpolate across robot lives.
    pub placement_revision: u64,
    /// Field-assigned identity, stable for the chassis' life.
    pub id: u32,
    /// Team the chassis plays for.
    pub team: Team,
    /// Its dimensions, so a viewer can draw any chassis it sees.
    pub config: ChassisConfig,
    /// Body centre pose in world FLU.
    pub pose: Pose,
    /// Gun pivot pose: the configured turret centre carried by the body,
    /// facing the actual motor aim with +x along the barrel (world-stabilised,
    /// so the body's roll and pitch do not reach it).
    pub turret: Pose,
    /// Body linear velocity in metres per second.
    pub velocity_m_s: [f64; 3],
    /// Body angular velocity in radians per second.
    pub angular_velocity_rad_s: [f64; 3],
    /// Last high-level request; drive components are dormant in direct torque mode.
    pub command: ChassisCommand,
    /// Held drive mode and direct wheel requests, preserved across restoration.
    #[serde(default)]
    pub drive_control: DriveControl,
    /// Previous physics slice jump state, retained to prevent replayed launches.
    #[serde(default)]
    pub jump_held: bool,
    /// Actual stabilized gimbal heading/elevation, preserved while defeated.
    pub held_aim_rad: [f64; 2],
    /// Actual motor rates, needed to continue acceleration-limited replay.
    pub gimbal_velocity_rad_s: [f64; 2],
    /// Wheel states in `ChassisConfig::wheel_hubs_m` order.
    pub wheels: Vec<WheelSnapshot>,
    /// The referee has this robot at zero HP: the drive is cut and the aim
    /// holds where it was.
    pub defeated: bool,
}

/// Borrowed privileged chassis truth, without cloning configuration or wheel storage.
///
/// Scalar poses and motion are copied from the current body. Wheel hubs are derived
/// lazily; this view does not expose solver contacts or warm starts. Gimbal and
/// folding coordinates describe the existing prescribed mechanisms, not articulated
/// joint torque dynamics. Holding the view prevents mutation of its physical world.
pub struct ChassisTelemetry<'a> {
    /// Stable chassis identity.
    pub id: u32,
    /// Assigned team.
    pub team: Team,
    /// Body pose in world FLU metres and wxyz quaternion.
    pub pose: Pose,
    /// Stabilized gun pivot pose in world FLU.
    pub turret: Pose,
    /// World linear velocity in metres per second.
    pub velocity_m_s: [f64; 3],
    /// World angular velocity in radians per second.
    pub angular_velocity_rad_s: [f64; 3],
    /// Actual prescribed gimbal heading and elevation, in radians.
    pub held_aim_rad: [f64; 2],
    /// Actual prescribed gimbal motor rates, in radians per second.
    pub gimbal_velocity_rad_s: [f64; 2],
    /// Actual prescribed folding/linkage coordinate, in radians.
    pub fold_angle_rad: f64,
    /// Held high-level input; drive components may be dormant in torque mode.
    pub command: ChassisCommand,
    /// Borrowed drive mode and wheel torque requests; no request array is cloned.
    pub drive_control: &'a DriveControl,
    /// Placement/revival revision, for detecting discontinuities.
    pub placement_revision: u64,
    /// Whether defeat disables actuation.
    pub defeated: bool,
    /// Measured drive motor load, in watts.
    pub drive_power_w: f64,
    chassis: &'a Chassis,
    world: &'a PhysicsWorld,
}

/// Allocation-free state for a controller; excludes geometry, configuration and wheel arrays.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChassisControlState {
    /// Current body pose in world FLU metres and unit wxyz quaternion.
    pub pose: Pose,
    /// Held high-level input, including aim, folding and balance channels.
    pub command: ChassisCommand,
    /// Placement/revival revision used to reject controls for an old life.
    pub placement_revision: u64,
    /// Whether native defeat has disabled actuation.
    pub defeated: bool,
}

fn vector(v: [f64; 3]) -> Vector {
    Vector::new(v[0], v[1], v[2])
}
fn rapier_pose(p: Pose) -> Pose3 {
    let pose = exact_rapier_pose(p);
    Pose3::from_parts(pose.translation, pose.rotation.normalize())
}
/// The pose bit for bit. Rapier integrates rotations without renormalising,
/// so a live body sits an ulp or two off unit length; a restore must keep that
/// or its replay drifts from the host's.
fn exact_rapier_pose(p: Pose) -> Pose3 {
    let [w, x, y, z] = p.rotation_wxyz;
    Pose3::from_parts(vector(p.translation_m), Rotation::from_xyzw(x, y, z, w))
}
fn pose_is_valid(p: Pose) -> bool {
    let norm = p.rotation_wxyz.iter().map(|v| v * v).sum::<f64>();
    p.translation_m.iter().all(|v| v.is_finite()) && norm.is_finite() && (norm - 1.).abs() < 1e-6
}

impl ChassisSnapshot {
    /// Recompute the wheel values that follow from the rest of the snapshot:
    /// each hub centre from `pose` and the configured hub layout, and each
    /// tyre target from the inverse kinematics of `command` (zero while
    /// `defeated`, whose drive is cut). A codec carrying wheel compression
    /// calls this after decoding. The hub matches the host within the pose's
    /// own precision. The target matches whenever the command has not changed
    /// since the last tick; a restored chassis recomputes both before using
    /// them, so neither affects stepping. Contacts are left alone. Wheels
    /// beyond the configured hubs are left unchanged.
    ///
    /// ```rust
    /// use rm_simulator_physics::chassis::{ChassisCommand, ChassisConfig, ChassisSnapshot};
    /// use rm_simulator_physics::{Pose, Team};
    ///
    /// let config = ChassisConfig::default();
    /// let wheels = config.wheel_hubs_m.len();
    /// let mut snapshot = ChassisSnapshot {
    ///     fold_angle_rad: 0.,
    ///     placement_revision: 0,
    ///     id: 0,
    ///     team: Team::Red,
    ///     config,
    ///     pose: Pose::at([1.0, 2.0, 0.3]),
    ///     turret: Pose::at([1.0, 2.0, 0.5]),
    ///     velocity_m_s: [0.0; 3],
    ///     angular_velocity_rad_s: [0.0; 3],
    ///     drive_control: Default::default(),
    ///     jump_held: false,
    ///     command: ChassisCommand { forward_m_s: 1.0, ..Default::default() },
    ///     held_aim_rad: [0.0; 2],
    ///     gimbal_velocity_rad_s: [0.0; 2],
    ///     wheels: vec![Default::default(); wheels],
    ///     defeated: false,
    /// };
    /// snapshot.derive_wheel_kinematics();
    /// let hub = snapshot.config.wheel_hubs_m[0];
    /// assert!((snapshot.wheels[0].hub_m[0] - (1.0 + hub[0])).abs() < 1e-12);
    /// let projected = std::f64::consts::FRAC_1_SQRT_2;
    /// assert!((snapshot.wheels[0].target_m_s + projected).abs() < 1e-12);
    /// assert!((snapshot.wheels[2].target_m_s - projected).abs() < 1e-12);
    /// ```
    pub fn derive_wheel_kinematics(&mut self) {
        let pose = rapier_pose(self.pose);
        let command = if self.defeated || self.drive_control != DriveControl::BodyVelocity {
            ChassisCommand::default()
        } else {
            self.command
        };
        for (wheel, geometry) in self
            .wheels
            .iter_mut()
            .zip(wheel_geometry_at(&self.config, self.fold_angle_rad))
        {
            wheel.hub_m = pose
                .transform_point(
                    geometry.hub_m
                        + Vector::Z
                            * (wheel.suspension_compression_m - self.config.suspension_rest_m),
                )
                .to_array();
            wheel.target_m_s = command.forward_m_s * geometry.roll.x
                + command.left_m_s * geometry.roll.y
                + command.yaw_rate_rad_s * geometry.lever_m;
        }
    }
}

/// Gun rotation as wxyz: yaw about world up, then elevation, which is a
/// negative rotation about the gun's own left (+y) axis.
fn aim_rotation(yaw_rad: f64, pitch_rad: f64) -> [f64; 4] {
    let (sy, cy) = (yaw_rad / 2.0).sin_cos();
    let (sp, cp) = (-pitch_rad / 2.0).sin_cos();
    // (cy, 0, 0, sy) * (cp, 0, sp, 0)
    [cy * cp, -sy * sp, cy * sp, cp * sy]
}

fn wrap_angle(angle: f64) -> f64 {
    use std::f64::consts::{PI, TAU};
    (angle + PI).rem_euclid(TAU) - PI
}

/// Yaw of a pose about world up, counter-clockwise from +x.
pub fn yaw_of(pose: Pose) -> f64 {
    let [w, x, y, z] = pose.rotation_wxyz;
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

#[cfg(test)]
mod climbing_tests;
#[cfg(test)]
mod folding_tests;
#[cfg(test)]
mod team_cad_tests;
#[cfg(test)]
mod tests;

#[cfg(test)]
use crate::wheel_model::{WheelDriveScratch, WheelModel, WheelState, drive_power_scale};

#[cfg(test)]
use rapier3d_f64::prelude::*;
