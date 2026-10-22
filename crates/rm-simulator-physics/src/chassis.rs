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
use crate::{
    Pose, Team,
    control::{DriveControl, WheelTorques},
    projectile::{FRICTION, SMALL_ARMOR_HOUSING_HALF_M},
};
use rapier3d_f64::parry::bounding_volume::BoundingVolume;
use rapier3d_f64::prelude::*;
use serde::{Deserialize, Serialize};

/// Low robot contact rebound. Assumed, not a measured material property.
const ROBOT_RESTITUTION: f64 = 0.05;
/// Body collider friction against walls and the ground.
const BODY_FRICTION: f64 = 0.3;
/// Damping applied by rapier to the body velocities each step; keeps a
/// chassis that has lost wheel contact from spinning for ever.
const LINEAR_DAMPING: f64 = 0.05;
const ANGULAR_DAMPING: f64 = 0.5;
/// Surfaces steeper than this against the body's up axis are walls, not ground.
const MIN_GROUND_COSINE: f64 = 0.3;
/// Fraction of the mass carried by the turret collider (gimbal and gun).
const TURRET_MASS_SHARE: f64 = 0.15;
/// Armor modules per chassis: front, left, back and right, in that order.
pub const ARMOR_COUNT: usize = 4;
/// Gun muzzle offset from the stabilised turret pivot along its +x view line.
/// This 0.35 m barrel length is an application-level infantry geometry assumption.
pub const MUZZLE_FORWARD_M: f64 = 0.35;

/// Horizontal acceleration a flying chassis may use to reach its commanded
/// velocity, in metres per second squared. Assumed, not a rulebook figure.
const FLIGHT_ACCELERATION_M_S2: f64 = 4.0;
/// Velocity-tracking gain of the flight controller, per second.
const FLIGHT_VELOCITY_GAIN_PER_S: f64 = 5.0;
/// Stiffness of a taut tether per kilogram of drone, in newtons per metre per
/// kilogram. A 30 rad/s pull-back keeps a full-speed overshoot to about 0.13 m.
/// Assumed; the rulebook specifies the tether's length, not its elasticity.
const TETHER_STIFFNESS_PER_KG: f64 = 900.0;
/// Near-critical damping of a taut tether per kilogram, in newtons per metre
/// per second per kilogram. It resists only motion that stretches the tether.
const TETHER_DAMPING_PER_KG: f64 = 60.0;
/// Cap on the taut tether's pull-back acceleration, in metres per second
/// squared, so a chassis placed far outside its reach returns smoothly.
const TETHER_MAX_PULL_M_S2: f64 = 20.0;

/// The Aerial Safety Rope a Drone flies on (rule manual V2.1.0 section 4.5,
/// Flight Zone). A retractable tether box rides a wire rope and cannot pass
/// the rope's Snap Ring; a soft tether of `length_m` runs from the box to the
/// drone's hook. The drone may therefore go anywhere within `length_m` of the
/// rope span from `rope_start_m` to `rope_end_m`, and nowhere else.
///
/// The hook is taken at the body centre. The tether's constant elastic force
/// (under 5 N static, section 4.5) is not modelled: the drone feels nothing
/// until the tether is taut. The box's inability to follow many turns in one
/// direction is not modelled either.
///
/// ```
/// use rm_simulator_physics::chassis::Tether;
///
/// let tether = Tether {
///     rope_start_m: [14.0, -5.8, 3.6],
///     rope_end_m: [0.0, -5.8, 3.6],
///     length_m: 2.4,
/// };
/// tether.validate()?;
/// // Directly below the rope and 1.6 m under it: 0.8 m of tether to spare.
/// assert!((tether.slack_m([7.0, -5.8, 2.0]) - 0.8).abs() < 1e-12);
/// // Past the Snap Ring the box stays at the rope's end.
/// assert_eq!(tether.anchor_m([-3.0, -5.8, 2.0]), [0.0, -5.8, 3.6]);
/// assert!(tether.slack_m([-3.0, -5.8, 2.0]) < 0.0);
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Tether {
    /// World FLU position of the near end of the span the tether box can
    /// travel, in metres: the end above the team's own Landing Pad.
    pub rope_start_m: [f64; 3],
    /// World FLU position of the Snap Ring, in metres; the box cannot pass it.
    pub rope_end_m: [f64; 3],
    /// Tether length from box to hook, in metres; 2.4 m in section 4.5.
    pub length_m: f64,
}
impl Tether {
    /// Where the tether box sits for a hook at `point_m`: the closest point of
    /// the rope span, in world FLU metres.
    pub fn anchor_m(&self, point_m: [f64; 3]) -> [f64; 3] {
        let start = vector(self.rope_start_m);
        let span = vector(self.rope_end_m) - start;
        let along = if span.length_squared() > 0.0 {
            ((vector(point_m) - start).dot(span) / span.length_squared()).clamp(0.0, 1.0)
        } else {
            0.0
        };
        (start + span * along).to_array()
    }
    /// Tether left before it is taut with the hook at `point_m`, in metres:
    /// positive inside the reach, negative by how far the hook is beyond it.
    pub fn slack_m(&self, point_m: [f64; 3]) -> f64 {
        self.length_m - (vector(point_m) - vector(self.anchor_m(point_m))).length()
    }
    /// Check that every coordinate is finite and the length positive.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self
            .rope_start_m
            .iter()
            .chain(&self.rope_end_m)
            .all(|v| v.is_finite())
            && self.length_m.is_finite()
            && self.length_m > 0.0
        {
            Ok(())
        } else {
            Err("tether rope must be finite and its length positive")
        }
    }
}

/// Assumed actuator and suspension tuning, not rulebook limits.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ChassisDynamics {
    /// Gimbal speed-loop time constant in seconds: the commanded rate is the
    /// aim error divided by this.
    pub gimbal_response_s: f64,
    /// Cap on the gimbal rate, in radians per second.
    pub gimbal_max_speed_rad_s: f64,
    /// Cap on how fast the gimbal rate may change, in radians per second squared.
    pub gimbal_max_acceleration_rad_s2: f64,
    /// Shared drivetrain power budget in watts. The four drives scale down
    /// together once mechanical output plus copper loss exceeds it.
    pub drive_power_w: f64,
    /// Copper loss at stall force, per motor; braking does not return power.
    pub motor_stall_loss_w: f64,
    /// Maximum ordinary spring travel, followed by a progressive bump stop.
    pub suspension_travel_m: f64,
    /// Extra spring rate once compression passes `suspension_travel_m`, in
    /// newtons per metre.
    pub bump_stop_stiffness_n_m: f64,
}
impl Default for ChassisDynamics {
    fn default() -> Self {
        Self {
            gimbal_response_s: 0.025,
            gimbal_max_speed_rad_s: 12.0,
            gimbal_max_acceleration_rad_s2: 240.0,
            drive_power_w: 120.0,
            motor_stall_loss_w: 25.0,
            suspension_travel_m: 0.04,
            bump_stop_stiffness_n_m: 80_000.0,
        }
    }
}

/// Octagonal deck, close-fitting lower bumper and wheel guards. Dimensions are
/// prototype assumptions, sized within ROB 2027 Table 2-7's 600 mm stowed cube.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OmniFrame {
    /// Equal corner cut on the deck's rectangular footprint, in metres.
    pub corner_cut_m: f64,
    /// Lower bumper's outer cardinal reach in metres (historical ring parameter).
    pub ring_half_m: f64,
    /// Corner cut of the lower bumper, leaving diagonal wheel bays, in metres.
    pub ring_corner_cut_m: f64,
    /// Width of the wheel guard bars in their horizontal plane, in metres.
    pub ring_width_m: f64,
    /// Wheel guard vertical thickness, in metres.
    pub ring_height_m: f64,
    /// Wheel guard attachment centre height relative to the body origin, in metres.
    pub ring_center_z_m: f64,
}

/// Counterclockwise chamfered rectangle in body FLU metres.
fn octagon(hx: f64, hy: f64, cut: f64) -> [[f64; 2]; 8] {
    [
        [hx, hy - cut],
        [hx - cut, hy],
        [-hx + cut, hy],
        [-hx, hy - cut],
        [-hx, -hy + cut],
        [-hx + cut, -hy],
        [hx - cut, -hy],
        [hx, -hy + cut],
    ]
}

/// Physical description of the chassis. All lengths are in the body frame:
/// origin at the body centre, +x forward, +y left, +z up.
///
/// Presets cover omni Infantry/Sentry, mecanum Hero/Engineer and two-wheel
/// Balance Infantry. All use assumed values, not rulebook dimensions.
///
/// ```
/// use rm_simulator_physics::chassis::ChassisConfig;
///
/// let infantry = ChassisConfig::default();
/// infantry.validate()?;
/// assert_eq!(infantry.mass_kg, 22.0);
/// assert!(!infantry.mecanum);
/// // Hub drop, wheel radius and rest suspension add up to the body height.
/// assert!((infantry.rest_height_m() - 0.2065).abs() < 1e-9);
///
/// let hero = ChassisConfig::hero();
/// hero.validate()?;
/// assert_eq!(hero.mass_kg, 30.0);
/// assert!(hero.mecanum);
/// assert!(hero.wheel_radius_m > infantry.wheel_radius_m);
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChassisConfig {
    /// Source-fitted serial joints and armor mounts, loaded from a verified package.
    #[serde(default)]
    pub articulation: Option<crate::mechanism::ArticulationConfig>,
    /// Optional source-fitted single-input wheel linkage for stair traversal.
    #[serde(default)]
    pub climbing: Option<crate::mechanism::ClimbingConfig>,
    /// Optional prescribed folding arm; its collider and muzzle geometry share one state.
    #[serde(default)]
    pub folding: Option<crate::mechanism::FoldingConfig>,
    /// Source-fitted Zhulong/Hello World Omni prototype; selects its measured mounts.
    #[serde(default)]
    pub team_cad: bool,
    /// Actuator and suspension tuning.
    pub dynamics: ChassisDynamics,
    /// Drive layout: `true` is mecanum rollers, `false` ideal omni wheels.
    pub mecanum: bool,
    /// Simplified ground-contact pitch controller for the two-wheel prototype.
    /// Reduced-model LQR body assistance is not a full wheel-leg dynamics model.
    #[serde(default)]
    pub balance_assist: bool,
    /// Fixed-altitude horizontal flight. Gravity and vertical motion are disabled.
    #[serde(default)]
    pub planar_flight: bool,
    /// Aerial Safety Rope limiting where a flying chassis may go, set by the
    /// field layout when the drone is placed. `None` flies unrestrained.
    #[serde(default)]
    pub tether: Option<Tether>,
    /// Total chassis mass in kilograms, armour modules excluded.
    pub mass_kg: f64,
    /// Deck collider half extents in metres. The omni ring covers its wheels;
    /// other presets use a conservative box around their footprint.
    pub body_half_m: [f64; 3],
    /// Optional octagonal deck and compact lower bumper; other presets keep their box.
    #[serde(default)]
    pub omni_frame: Option<OmniFrame>,
    /// Turret centre in the body frame (the gun pivot) and half extents. The
    /// collider is a column of this footprint from the body top to the turret
    /// top, standing in for the gimbal neck.
    pub turret_center_m: [f64; 3],
    /// Half extents of that turret column, in metres.
    pub turret_half_m: [f64; 3],
    /// Wheel hubs sit this far below the body centre.
    pub hub_drop_m: f64,
    /// Wheel hub positions in the body's horizontal plane. Each wheel rolls
    /// along the counter-clockwise tangent about the body centre and its
    /// rollers run along the radial axle.
    pub wheel_hubs_m: Vec<[f64; 2]>,
    /// Wheel radius in metres; it sets both the suspension reach and the
    /// rolling speed the spin integrates.
    pub wheel_radius_m: f64,
    /// Axle width of one wheel, in metres.
    pub wheel_width_m: f64,
    /// Wheel and reflected drivetrain rotational inertia, in kg m². Assumed.
    #[serde(default = "default_wheel_inertia")]
    pub wheel_inertia_kg_m2: f64,
    /// Gap between the wheel bottom and the ground with the suspension fully
    /// extended. Compression beyond it is resisted by the same spring. The
    /// static sag (weight over stiffness) is the margin a wheel keeps before
    /// it lifts off on twisted ground, so a chassis that is too stiff rocks on
    /// two or three wheels. The default uses compliant, near-critically damped
    /// springs so the edge-centered wheels retain traction on ramp entries.
    pub suspension_rest_m: f64,
    /// Spring rate of one wheel suspension, in newtons per metre.
    pub suspension_stiffness_n_m: f64,
    /// Damper rate of one wheel suspension, in newtons per metre per second.
    pub suspension_damping_n_s_m: f64,
    /// Tyre force one drive can produce at standstill.
    pub wheel_stall_force_n: f64,
    /// Wheel surface speed at which the drive torque reaches zero.
    pub wheel_no_load_speed_m_s: f64,
    /// Tyre force per unit of slip speed before the friction or motor limit.
    pub slip_stiffness_n_s_m: f64,
    /// Coulomb friction coefficient along the rolling direction.
    pub drive_friction: f64,
    /// Coulomb friction coefficient along the axle: the rollers' resistance.
    pub roller_friction: f64,
    /// One small armor module on each side of the body (front, left, back,
    /// right): its scoring face stands this far outside the body side, its
    /// centre this high above the body centre, and it leans back by this
    /// angle so the outward normal tilts up. ROB 2027 S120 (2026 S149)
    /// requires a normal 75° from vertical: 15° above the horizontal.
    pub armor_standoff_m: f64,
    /// Lower armor-pair centre height above the body centre, in metres.
    /// Omni front/rear faces add `frame_beam_step_m()`; other frames use this for all faces.
    pub armor_height_m: f64,
    /// Lean angle of the plate in radians; the outward normal tilts up by this.
    pub armor_tilt_rad: f64,
}
fn default_wheel_inertia() -> f64 {
    0.008
}

impl Default for ChassisConfig {
    /// A typical infantry: 22 kg (competition robots weigh about 22 to 32 kg),
    /// 0.41 m octagonal deck and 0.556 m lower bumper, 153 mm omni wheels at the
    /// four diagonals (245 mm from centre), prototype drives (about 4.6 N m at the
    /// wheel, 3.8 m/s free running). Assumed values, see the module notes.
    fn default() -> Self {
        let d = 0.245 * std::f64::consts::FRAC_1_SQRT_2;
        Self {
            articulation: None,
            climbing: None,
            folding: None,
            team_cad: false,
            dynamics: ChassisDynamics {
                suspension_travel_m: 0.12,
                ..Default::default()
            },
            mecanum: false,
            balance_assist: false,
            planar_flight: false,
            tether: None,
            mass_kg: 22.0,
            body_half_m: [0.205, 0.205, 0.05],
            omni_frame: Some(OmniFrame {
                corner_cut_m: 0.095,
                ring_half_m: 0.285,
                ring_corner_cut_m: 0.257,
                ring_width_m: 0.012,
                ring_height_m: 0.020,
                ring_center_z_m: -0.070,
            }),
            turret_center_m: [0.0, 0.0, 0.2],
            turret_half_m: [0.06, 0.06, 0.06],
            hub_drop_m: 0.05,
            wheel_hubs_m: vec![[d, d], [-d, d], [-d, -d], [d, -d]],
            wheel_radius_m: 0.0765,
            wheel_width_m: 0.04,
            wheel_inertia_kg_m2: default_wheel_inertia(),
            suspension_rest_m: 0.08,
            // Compliant independent suspension retains contact on twisted ground.
            suspension_stiffness_n_m: 1_000.0,
            // About 0.9 of critical for a quarter of 22 kg on 1 kN/m.
            suspension_damping_n_s_m: 135.0,
            // All four drives contribute through their 45-degree rolling directions.
            wheel_stall_force_n: 60.0,
            wheel_no_load_speed_m_s: 3.8,
            slip_stiffness_n_s_m: 150.0,
            drive_friction: 1.0,
            roller_friction: 0.03,
            // Recess the carrier and face inside the compact lower bumper.
            armor_standoff_m: 0.045,
            armor_height_m: 0.030,
            armor_tilt_rad: 15_f64.to_radians(),
        }
    }
}
impl ChassisConfig {
    /// Source-fitted Zhulong chassis / Hello World gimbal, at original scale.
    /// Mounts come from the approved STEP assembly recipe (2026-10-18), not
    /// rulebook dimensions. Mass, springs and motors retain assumed tuning.
    ///
    /// ```
    /// use rm_simulator_physics::chassis::ChassisConfig;
    /// let robot = ChassisConfig::team_cad();
    /// assert!(robot.team_cad);
    /// assert_eq!(robot.wheel_hubs_m.len(), 4);
    /// ```
    pub fn team_cad() -> Self {
        let d = 0.178_306_329_501_305;
        Self {
            team_cad: true,
            omni_frame: Some(OmniFrame {
                ring_half_m: 0.290,
                ring_height_m: 0.030,
                ring_center_z_m: -0.085,
                ..Self::default().omni_frame.unwrap()
            }),
            wheel_hubs_m: vec![[d, d], [d, -d], [-d, d], [-d, -d]],
            wheel_radius_m: 0.0775,
            // Source axle at Y=-13.1776 mm relative to body reference Y=85 mm.
            // Account for the assumed 22 kg / four 1 kN/m springs at static sag.
            hub_drop_m: 0.098_177_595_154_746 - (0.08 - 22.0 * 9.81 / 4_000.0),
            turret_center_m: [0., 0., 0.24095 + 0.1295 - 0.085],
            turret_half_m: [0.145, 0.075, 0.12],
            armor_height_m: 0.007_695_17,
            armor_standoff_m: 0.057_919_39,
            ..Self::default()
        }
    }
    /// Source-fitted Artisans Hero with mecanum wheels and a counter-rotating folding head.
    /// Joint centres come from the 2026 source STEP; 127 degrees and a 500 ms full stroke
    /// are simulator actuator assumptions. Legs retain effective ray suspension.
    pub fn folding_hero() -> Self {
        let reference_z = 0.0775;
        Self {
            folding: Some(crate::mechanism::FoldingConfig {
                yaw_pivot_m: [0., 0., 0.076 - reference_z],
                lower_pivot_m: [0., 0., 0.122 - reference_z],
                upper_pivot_m: [-0.0091, 0., 0.23647 - reference_z],
                arm_center_m: [-0.00455, 0.06, 0.057235],
                arm_half_m: [0.03, 0.135, 0.07],
                head_center_m: [0.0755, -0.019, 0.0275],
                head_half_m: [0.1505, 0.095, 0.069],
                folded_angle_rad: 127_f64.to_radians(),
                speed_rad_s: 127_f64.to_radians() / 0.5,
                muzzle_offset_m: [0.23582, -0.00141, 0.],
                camera_offset_m: [0.14243, -0.00676, 0.07753],
            }),
            body_half_m: [0.28, 0.235, 0.0775],
            turret_center_m: [-0.0091, 0., 0.23647 - reference_z],
            turret_half_m: [0.1505, 0.095, 0.069],
            wheel_hubs_m: vec![
                [0.29, 0.27505],
                [0.29, -0.27505],
                [-0.244978, 0.27499],
                [-0.244978, -0.27499],
            ],
            wheel_radius_m: 0.077,
            wheel_width_m: 0.042,
            hub_drop_m: 0.01293 + reference_z - 0.03,
            armor_standoff_m: 0.055,
            armor_height_m: -0.005,
            ..Self::hero()
        }
    }

    /// DreamChaser Engineer donor with its closed stair-climbing wheel linkage.
    /// Geometry follows the supplied 2026 STEP/report; 38 kg, spring/motor tuning
    /// and a 500 ms full actuator stroke are simulator assumptions. This profile
    /// does not admit the unavailable RM2027 Heavy gameplay role.
    pub fn stair_heavy() -> Self {
        let linkage = crate::mechanism::ClimbingConfig {
            front_pivot_m: [0.215, -0.0152747187334],
            guide_m: [-0.135, -0.1452747187334],
            slider_height_m: -0.0252747187334,
            front_pin_length_m: 0.130,
            front_wheel_length_m: 0.190,
            coupler_length_m: 0.200,
            rear_wheel_length_m: 0.320,
            travel_angle_rad: 0.0926526307467,
            deployed_angle_rad: 1.3302776523948,
            speed_rad_s: (1.3302776523948 - 0.0926526307467) / 0.5,
            wheel_left_m: [0.2083, -0.2083],
            spring_anchor_offset_m: 0.03 - 38. * 9.81 / 40_000.,
        };
        let hubs = linkage.poses(0.).hubs_m;
        Self {
            climbing: Some(linkage.clone()),
            mass_kg: 38.,
            body_half_m: [0.285, 0.270, 0.015],
            omni_frame: None,
            wheel_hubs_m: hubs.iter().map(|p| [p[0], p[1]]).collect(),
            hub_drop_m: -hubs[0][2] - linkage.spring_anchor_offset_m,
            wheel_radius_m: 0.077,
            wheel_width_m: 0.094,
            suspension_rest_m: 0.03,
            suspension_stiffness_n_m: 10_000.,
            suspension_damping_n_s_m: 500.,
            wheel_stall_force_n: 180.,
            turret_center_m: [0.08, 0., 0.32],
            turret_half_m: [0.23, 0.16, 0.30],
            armor_height_m: 0.0,
            armor_standoff_m: 0.060,
            ..Self::engineer()
        }
    }
    /// Maximum primary transform input angle, radians; zero for fixed robots.
    /// The legacy `fold_angle_rad` snapshot field carries this one input for
    /// either mutually exclusive folding or stair-climbing configurations.
    pub fn transform_limit_rad(&self) -> f64 {
        self.articulation
            .as_ref()
            .map(|a| a.limit_rad)
            .or_else(|| self.folding.as_ref().map(|f| f.folded_angle_rad))
            .or_else(|| self.climbing.as_ref().map(|c| c.travel_rad()))
            .unwrap_or(0.)
    }
    /// Set a full mechanism stroke duration in seconds, retaining source geometry.
    /// This simulator tuning applies to either direction; a reversal travels only
    /// the remaining distance. Obstruction, defeat and module outages can delay it.
    /// Fixed robots are unchanged. Rejects nonpositive/nonfinite durations or rates.
    ///
    /// ```
    /// use rm_simulator_physics::chassis::ChassisConfig;
    /// let mut robot = ChassisConfig::folding_hero();
    /// robot.set_transform_duration_s(0.5).unwrap();
    /// assert_eq!(robot.folding.as_ref().unwrap().speed_rad_s,
    ///            robot.transform_limit_rad() / 0.5);
    /// ```
    pub fn set_transform_duration_s(&mut self, duration_s: f64) -> Result<(), &'static str> {
        let limit = self.transform_limit_rad();
        let speed = limit / duration_s;
        if !duration_s.is_finite()
            || duration_s <= 0.
            || !speed.is_finite()
            || (limit > 0. && speed <= 0.)
        {
            return Err("invalid transform duration");
        }
        if limit == 0. {
            return Ok(());
        }
        if let Some(a) = &mut self.articulation {
            a.speed_rad_s = speed;
        }
        if let Some(f) = &mut self.folding {
            f.speed_rad_s = speed;
        }
        if let Some(c) = &mut self.climbing {
            c.speed_rad_s = speed;
        }
        Ok(())
    }
    /// World collision poses for the primary transform at an explicit input angle.
    pub fn transform_collision_parts(
        &self,
        body: Pose,
        aim_rad: [f64; 2],
        angle_rad: f64,
    ) -> Vec<(Pose, [f64; 3])> {
        if let Some(a) = &self.articulation {
            let local = rapier_pose(body).rotation.inverse()
                * Vector::new(
                    aim_rad[1].cos() * aim_rad[0].cos(),
                    aim_rad[1].cos() * aim_rad[0].sin(),
                    aim_rad[1].sin(),
                );
            let aim = [local.y.atan2(local.x), local.z.clamp(-1., 1.).asin()];
            a.collision_parts(angle_rad, aim)
                .into_iter()
                .map(|(p, h)| {
                    (
                        {
                            let v = rapier_pose(body) * rapier_pose(p);
                            Pose {
                                translation_m: v.translation.to_array(),
                                rotation_wxyz: [
                                    v.rotation.w,
                                    v.rotation.x,
                                    v.rotation.y,
                                    v.rotation.z,
                                ],
                            }
                        },
                        h,
                    )
                })
                .collect()
        } else if let Some(folding) = &self.folding {
            folding.collision_parts(body, aim_rad, angle_rad).to_vec()
        } else if let Some(climbing) = &self.climbing {
            climbing
                .collision_parts(angle_rad)
                .into_iter()
                .map(|(p, h)| {
                    (
                        {
                            let p = rapier_pose(body) * rapier_pose(p);
                            Pose {
                                translation_m: p.translation.to_array(),
                                rotation_wxyz: [
                                    p.rotation.w,
                                    p.rotation.x,
                                    p.rotation.y,
                                    p.rotation.z,
                                ],
                            }
                        },
                        h,
                    )
                })
                .collect()
        } else {
            vec![]
        }
    }

    /// Gun muzzle offset from the pitch pivot in local FLU metres.
    /// The source SM01 front is 206.577 mm ahead of the gimbal pivot.
    ///
    /// ```
    /// use rm_simulator_physics::chassis::ChassisConfig;
    /// assert_eq!(ChassisConfig::team_cad().muzzle_offset_m(), [0.20657673, 0., 0.]);
    /// ```
    pub fn muzzle_offset_m(&self) -> [f64; 3] {
        if let Some(a) = &self.articulation {
            a.muzzle_offset_m
        } else if let Some(folding) = &self.folding {
            folding.muzzle_offset_m
        } else if self.team_cad {
            [0.206_576_73, 0., 0.]
        } else {
            [MUZZLE_FORWARD_M, 0., 0.]
        }
    }

    /// Deck footprint in body FLU metres, counterclockwise. A box has four corners.
    ///
    /// ```
    /// use rm_simulator_physics::chassis::ChassisConfig;
    /// assert_eq!(ChassisConfig::default().body_outline_m().len(), 8);
    /// assert_eq!(ChassisConfig::hero().body_outline_m().len(), 4);
    /// ```
    pub fn body_outline_m(&self) -> Vec<[f64; 2]> {
        let [x, y, _] = self.body_half_m;
        self.omni_frame.as_ref().map_or_else(
            || vec![[x, y], [-x, y], [-x, -y], [x, -y]],
            |frame| octagon(x, y, frame.corner_cut_m).to_vec(),
        )
    }
    /// Convex lower bumper and wheel guard prisms in body FLU metres. Each has
    /// four vertices per end face, shared by collision and rendering.
    /// The first eight prisms form a solid lower-deck extrusion. Each wheel
    /// then adds a horizontal semicircular guard whose ends join the lower deck.
    /// The diagonal bays keep every guard clear of the suspended tyre envelope.
    ///
    /// ```
    /// use rm_simulator_physics::chassis::ChassisConfig;
    /// assert_eq!(ChassisConfig::default().bumper_prisms_m().len(), 56);
    /// assert!(ChassisConfig::hero().bumper_prisms_m().is_empty());
    /// ```
    pub fn bumper_prisms_m(&self) -> Vec<[[f64; 3]; 8]> {
        let Some(f) = &self.omni_frame else {
            return Vec::new();
        };
        let outer = octagon(f.ring_half_m, f.ring_half_m, f.ring_corner_cut_m);
        let low = f.ring_center_z_m - 0.5 * f.ring_height_m;
        let top = -self.body_half_m[2];
        let prism = |xy: [[f64; 2]; 4], lo: f64, hi: f64| {
            std::array::from_fn(|k| [xy[k % 4][0], xy[k % 4][1], if k < 4 { lo } else { hi }])
        };
        let midpoint = |a: [f64; 2], b: [f64; 2]| [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
        // A fan of convex quads fills the bottom without a surrounding moat.
        let mut parts: Vec<_> = (0..8)
            .map(|i| {
                prism(
                    [
                        [0.0, 0.0],
                        midpoint(outer[(i + 7) % 8], outer[i]),
                        outer[i],
                        midpoint(outer[i], outer[(i + 1) % 8]),
                    ],
                    low,
                    top,
                )
            })
            .collect();
        for geometry in wheel_geometry(self) {
            let radial = geometry.axle;
            let tangent = geometry.roll;
            let radius = geometry.hub_m.x.hypot(geometry.hub_m.y);
            let centre = 0.40 * radius;
            // Enclose the whole tyre footprint, including its outer corners.
            let inner =
                (radius + 0.5 * self.wheel_width_m - centre).hypot(self.wheel_radius_m) + 0.008;
            let outer = inner + f.ring_width_m;
            // A horizontal bow around the wheel, with both ends embedded in the
            // lower deck. It stays below the armor and clear of vertical travel.
            for sector in 0..12 {
                let angle = |i: usize| std::f64::consts::PI * i as f64 / 12.0;
                let a = angle(sector);
                let b = angle(sector + 1);
                let xy = [(inner, a), (outer, a), (outer, b), (inner, b)].map(|(r, angle)| {
                    let p = radial * (centre + r * angle.sin()) + tangent * (r * angle.cos());
                    [p.x, p.y]
                });
                parts.push(prism(xy, low, low + f.ring_height_m));
            }
        }
        parts
    }
    /// Fixed upper and moving lower spring mounts in body-local FLU metres.
    /// Omni mounts are inboard of the tyre, between the crossed frame beams.
    /// The lower carriage follows wheel heave without rotating with its axle.
    /// `compression_m` is suspension compression from full droop, in metres.
    /// These depict the existing effective wheel spring; they add no joint dynamics.
    /// Returns `None` for an absent wheel.
    ///
    /// ```
    /// use rm_simulator_physics::chassis::ChassisConfig;
    /// let cfg = ChassisConfig::default();
    /// let [upper, lower] = cfg.suspension_mounts_m(0, 0.05).unwrap();
    /// assert!(upper[2] < cfg.body_half_m[2]);
    /// assert!(lower[2] < upper[2]);
    /// assert!(cfg.suspension_mounts_m(99, 0.0).is_none());
    /// ```
    pub fn suspension_mounts_m(&self, wheel: usize, compression_m: f64) -> Option<[[f64; 3]; 2]> {
        self.suspension_mounts_at_m(wheel, compression_m, 0.)
    }
    /// Suspension mounts at an explicit primary linkage angle, in body metres.
    pub fn suspension_mounts_at_m(
        &self,
        wheel: usize,
        compression_m: f64,
        angle_rad: f64,
    ) -> Option<[[f64; 3]; 2]> {
        if let Some(c) = &self.climbing {
            let p = *c.poses(angle_rad).hubs_m.get(wheel)?;
            return Some([
                [p[0], p[1], p[2] + c.spring_anchor_offset_m],
                [
                    p[0],
                    p[1],
                    p[2] + c.spring_anchor_offset_m - self.suspension_rest_m + compression_m,
                ],
            ]);
        }
        let &[x, y] = self.wheel_hubs_m.get(wheel)?;
        let inboard = if self.omni_frame.is_some() { 0.60 } else { 1.0 };
        let upper_z = if self.omni_frame.is_some() {
            self.body_half_m[2] - 0.022
        } else {
            self.body_half_m[2]
        };
        let lower_z = -self.hub_drop_m - self.suspension_rest_m
            + compression_m
            + if self.omni_frame.is_some() {
                0.025
            } else {
                0.0
            };
        Some([
            [x * inboard, y * inboard, upper_z],
            [x * inboard, y * inboard, lower_z],
        ])
    }

    /// Assumed Hero prototype with 203 mm mecanum wheels. These are design
    /// choices, not rulebook dimensions; HP and weapon limits remain configured separately.
    pub fn hero() -> Self {
        Self {
            armor_standoff_m: 0.02,
            armor_height_m: 0.015,
            omni_frame: None,
            dynamics: ChassisDynamics {
                drive_power_w: 160.0,
                ..Default::default()
            },
            mecanum: true,
            mass_kg: 30.0,
            body_half_m: [0.33, 0.28, 0.06],
            turret_center_m: [0.0, 0.0, 0.25],
            turret_half_m: [0.09, 0.09, 0.075],
            wheel_hubs_m: vec![[0.22, 0.22], [-0.22, 0.22], [-0.22, -0.22], [0.22, -0.22]],
            wheel_radius_m: 0.1015,
            wheel_width_m: 0.065,
            wheel_stall_force_n: 60.0,
            drive_friction: 0.7,
            suspension_rest_m: 0.03,
            suspension_stiffness_n_m: 10_000.0,
            // About 0.9 of critical for a quarter of 30 kg on 10 kN/m.
            suspension_damping_n_s_m: 500.0,
            ..Self::default()
        }
    }

    /// Approximate two-wheel infantry with passive leg suspension. Dimensions and controller
    /// gains are prototype assumptions, not rulebook constants. It drives
    /// forward/backward and turns; it cannot strafe. Jump uses an equivalent
    /// leg push impulse; joint-level leg dynamics are not simulated.
    ///
    /// ```
    /// use rm_simulator_physics::chassis::ChassisConfig;
    /// let config = ChassisConfig::balance();
    /// config.validate()?;
    /// assert_eq!(config.wheel_hubs_m.len(), 2);
    /// assert!(config.balance_assist);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn balance() -> Self {
        Self {
            armor_standoff_m: 0.02,
            armor_height_m: 0.015,
            omni_frame: None,
            balance_assist: true,
            dynamics: ChassisDynamics::default(),
            suspension_rest_m: 0.03,
            body_half_m: [0.18, 0.24, 0.09],
            hub_drop_m: 0.18,
            wheel_hubs_m: vec![[0.0, 0.24], [0.0, -0.24]],
            wheel_radius_m: 0.10,
            wheel_width_m: 0.045,
            suspension_stiffness_n_m: 20_000.0,
            suspension_damping_n_s_m: 840.0,
            roller_friction: 0.7,
            drive_friction: 0.7,
            wheel_stall_force_n: 80.0,
            ..Self::default()
        }
    }
    /// Approximate engineer platform carrying a fixed arm. The envelope,
    /// mass and drive tuning are design assumptions; arm dynamics are absent.
    ///
    /// ```
    /// use rm_simulator_physics::chassis::ChassisConfig;
    /// let config = ChassisConfig::engineer();
    /// config.validate()?;
    /// assert!(config.mecanum);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn engineer() -> Self {
        Self {
            body_half_m: [0.36, 0.29, 0.08],
            wheel_hubs_m: vec![
                [0.24, 0.255],
                [-0.24, 0.255],
                [-0.24, -0.255],
                [0.24, -0.255],
            ],
            turret_center_m: [-0.12, 0.0, 0.36],
            turret_half_m: [0.13, 0.13, 0.1],
            ..Self::hero()
        }
    }

    /// Guarded quadcopter prototype constrained to the horizontal plane it is
    /// placed in; `rest_height_m` offers 1.6 m above the ground for a bare
    /// placement, and the field layout sets its own flight height and tether.
    /// It flies in its body frame and yaws on command, independent of the
    /// gimbal aim. No rotor aerodynamics or altitude controls.
    ///
    /// The underslung gimbal hangs 0.20 m ahead of the body centre, so a
    /// forward shot's muzzle clears the 0.8 m frame at every pitch the flight
    /// controls allow; a centred gimbal put upward shots into the frame.
    pub fn drone() -> Self {
        Self {
            armor_standoff_m: 0.02,
            armor_height_m: 0.015,
            omni_frame: None,
            planar_flight: true,
            mass_kg: 5.0,
            body_half_m: [0.40, 0.40, 0.065],
            turret_center_m: [0.20, 0.0, -0.14],
            turret_half_m: [0.035; 3],
            wheel_hubs_m: Vec::new(),
            ..Self::default()
        }
    }
    /// Height of the body centre above flat ground with the springs unloaded.
    pub fn rest_height_m(&self) -> f64 {
        if self.planar_flight {
            return 1.6;
        }
        self.hub_drop_m + self.wheel_radius_m + self.suspension_rest_m
    }
    /// Assumed separation between the stacked omni frame beam sets, in metres.
    /// The front/rear armor pair follows the upper beams. This 28 mm prototype
    /// dimension is inspired by the reference assembly, not a rulebook requirement.
    ///
    /// ```
    /// use rm_simulator_physics::chassis::ChassisConfig;
    /// assert_eq!(ChassisConfig::default().frame_beam_step_m(), 0.028);
    /// assert_eq!(ChassisConfig::hero().frame_beam_step_m(), 0.0);
    /// ```
    pub fn frame_beam_step_m(&self) -> f64 {
        if self.omni_frame.is_some() {
            0.028
        } else {
            0.0
        }
    }

    /// Body-frame pose of each armor scoring face (+x the outward normal,
    /// +y along the width, +z along the height): front, left, back, right.
    ///
    /// ```
    /// use rm_simulator_physics::{chassis::ChassisConfig, motion::outpost::rotate};
    ///
    /// let faces = ChassisConfig::default().armor_faces();
    /// assert_eq!(faces.len(), 4);
    ///
    /// let normals: Vec<[f64; 3]> = faces
    ///     .iter()
    ///     .map(|face| rotate(face.rotation_wxyz, [1.0, 0.0, 0.0]))
    ///     .collect();
    /// // Front points along +x, left +y, back -x, right -y.
    /// assert!(normals[0][0] > 0.96);
    /// assert!(normals[1][1] > 0.96);
    /// assert!(normals[2][0] < -0.96);
    /// assert!(normals[3][1] < -0.96);
    /// // Every plate leans back 15 degrees, so each normal tilts up.
    /// assert!(normals.iter().all(|normal| normal[2] > 0.2));
    /// ```
    pub fn armor_faces(&self) -> [Pose; ARMOR_COUNT] {
        if let Some(a) = &self.articulation {
            return a.armor_faces;
        }
        if self.climbing.is_some() {
            return [
                [0.32379092, 0., 0.01032271],
                [-0.01399811, 0.34581087, -0.05815303],
                [-0.28285879, 0., -0.08268737],
                [-0.01400001, -0.34573989, -0.05814805],
            ]
            .into_iter()
            .enumerate()
            .map(|(i, translation_m)| {
                let q = Rotation::from_rotation_z(i as f64 * std::f64::consts::FRAC_PI_2)
                    * Rotation::from_rotation_y(-15_f64.to_radians());
                Pose {
                    translation_m,
                    rotation_wxyz: [q.w, q.x, q.y, q.z],
                }
            })
            .collect::<Vec<_>>()
            .try_into()
            .unwrap();
        }
        // Source-fitted replacement fronts from robot-armor-v1. Order stays
        // front/left/back/right even where the source arms are asymmetric.
        if self.folding.is_some() {
            return [
                (
                    [0.36365273, 0.00000001, -0.01655840],
                    [0.99138091, -0.00000317, -0.13101104, -0.00002400],
                ),
                (
                    [0.05999869, 0.35522290, -0.01806917],
                    [0.70111012, 0.09284380, -0.09287754, 0.70085547],
                ),
                (
                    [-0.31373668, -0.00000149, -0.06053783],
                    [0.00020390, 0.13139881, -0.00002703, 0.99132957],
                ),
                (
                    [0.06000125, -0.35524141, -0.01806626],
                    [-0.70078373, 0.09286484, 0.09281132, 0.70118781],
                ),
            ]
            .map(|(translation_m, q)| {
                let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
                Pose {
                    translation_m,
                    rotation_wxyz: q.map(|v| v / norm),
                }
            });
        }
        let [hx, hy, _] = self.body_half_m;
        std::array::from_fn(|plate| {
            // Exact axis directions, so a plate never picks up a rounding yaw.
            let (cos, sin) = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)][plate];
            let reach = if self.team_cad {
                if plate.is_multiple_of(2) {
                    0.248_201_21
                } else {
                    0.262_919_39
                }
            } else {
                (if plate.is_multiple_of(2) { hx } else { hy }) + self.armor_standoff_m
            };
            let yaw = plate as f64 * std::f64::consts::FRAC_PI_2;
            let rotation =
                Rotation::from_rotation_z(yaw) * Rotation::from_rotation_y(-self.armor_tilt_rad);
            Pose {
                translation_m: [
                    reach * cos,
                    reach * sin,
                    self.armor_height_m
                        + if self.team_cad && plate.is_multiple_of(2) {
                            -0.047
                        } else if plate.is_multiple_of(2) {
                            self.frame_beam_step_m()
                        } else {
                            0.0
                        },
                ],
                rotation_wxyz: [rotation.w, rotation.x, rotation.y, rotation.z],
            }
        })
    }
    /// Physical housing half extents in local FLU metres, behind the face.
    /// The source Artisans AM12 uses the guide's 235 × 127 mm outline; AM02 keeps the fitted Outpost
    /// housing contract. Dimensions are CAD fits, not effective scoring areas.
    ///
    /// ```
    /// use rm_simulator_physics::chassis::ChassisConfig;
    /// assert_eq!(ChassisConfig::folding_hero().armor_housing_half_m()[1], 0.1175);
    /// ```
    pub fn armor_housing_half_m(&self) -> [f64; 3] {
        if self.folding.is_some() {
            [0.0095, 0.1175, 0.0635]
        } else {
            SMALL_ARMOR_HOUSING_HALF_M
        }
    }

    /// Rough designated-carrier geometry behind an omni armor module, in body
    /// FLU metres. Two deep side carriers and a horizontal mounting foot share
    /// their vertices with rendering; they are unscored rigid chassis colliders.
    /// The raised pair also includes a chassis spacer beneath its carrier foot.
    /// ROB 2027 §3.6, S117–S118 and Figure 3-15 require the official bracket.
    /// These fitted solids depict it; they are not official replacement drawings.
    /// Invalid plate indices and non-omni frames return no geometry.
    ///
    /// ```
    /// use rm_simulator_physics::chassis::ChassisConfig;
    /// let cfg = ChassisConfig::default();
    /// assert_eq!(cfg.armor_bracket_prisms_m(0).len(), 4);
    /// assert!(cfg.armor_bracket_prisms_m(4).is_empty());
    /// ```
    pub fn armor_bracket_prisms_m(&self, plate: usize) -> Vec<[[f64; 3]; 8]> {
        if self.omni_frame.is_none() || plate >= ARMOR_COUNT {
            return Vec::new();
        }
        if self.team_cad {
            return Vec::new();
        }
        let face = self.armor_faces()[plate];
        let yaw = plate as f64 * std::f64::consts::FRAC_PI_2;
        let rotation = Rotation::from_rotation_z(yaw);
        let center = Vector::from_array(face.translation_m);
        let (sin, cos) = self.armor_tilt_rad.sin_cos();
        // Front carrier edges touch only the housing's rear plane. A small
        // clearance keeps the foot below the armor rather than touching its lip.
        let front = |z: f64| [-0.019 * cos - z * sin, -0.019 * sin + z * cos];
        let lower = front(-0.060);
        let upper = front(0.060);
        let profile = [lower, upper, [-0.066, upper[1]], [-0.066, -0.076]];
        let mut parts: Vec<_> = [-0.043, 0.043]
            .into_iter()
            .map(|y| {
                std::array::from_fn(|k| {
                    let [x, z] = profile[k % 4];
                    (center
                        + rotation * Vector::new(x, y + if k < 4 { -0.0075 } else { 0.0075 }, z))
                    .to_array()
                })
            })
            .collect();
        parts.push(std::array::from_fn(|k| {
            let corners = [
                [-0.066, -0.0705],
                [-0.010, -0.0705],
                [-0.010, 0.0705],
                [-0.066, 0.0705],
            ];
            let [x, y] = corners[k % 4];
            (center + rotation * Vector::new(x, y, if k < 4 { -0.080 } else { -0.072 })).to_array()
        }));
        let step = face.translation_m[2] - self.armor_height_m;
        if step > 0.0 {
            // A chassis spacer supports the unchanged designated bracket foot.
            // It touches the carrier only, never the armor module itself.
            parts.push(std::array::from_fn(|k| {
                let corners = [
                    [-0.066, -0.0705],
                    [-0.048, -0.0705],
                    [-0.048, 0.0705],
                    [-0.066, 0.0705],
                ];
                let [x, y] = corners[k % 4];
                (center + rotation * Vector::new(x, y, if k < 4 { -0.080 - step } else { -0.080 }))
                    .to_array()
            }));
        }
        parts
    }

    /// Validate dimensions and physical parameters before constructing a chassis.
    pub fn validate(&self) -> Result<(), &'static str> {
        if let Some(a) = &self.articulation {
            a.validate()?;
            if self.folding.is_some()
                || self.climbing.is_some()
                || self.planar_flight
                || self.balance_assist
                || self.team_cad
            {
                return Err("source articulation requires an exclusive ground chassis");
            }
        }
        if self.climbing.is_some()
            && (self.folding.is_some()
                || self.planar_flight
                || self.balance_assist
                || self.wheel_hubs_m.len() != 4)
        {
            return Err("climbing needs four ground wheels and one exclusive mechanism");
        }
        if let Some(climbing) = &self.climbing {
            climbing.validate()?;
        }
        if let Some(folding) = &self.folding {
            folding.validate()?;
        }
        if self.planar_flight && (self.balance_assist || !self.wheel_hubs_m.is_empty()) {
            return Err("planar flight cannot have wheels or balance assist");
        }
        if let Some(tether) = &self.tether {
            if !self.planar_flight {
                return Err("only a flying chassis has a tether");
            }
            tether.validate()?;
        }
        let positive = [
            self.dynamics.gimbal_response_s,
            self.dynamics.gimbal_max_speed_rad_s,
            self.dynamics.gimbal_max_acceleration_rad_s2,
            self.dynamics.drive_power_w,
            self.dynamics.motor_stall_loss_w,
            self.dynamics.suspension_travel_m,
            self.dynamics.bump_stop_stiffness_n_m,
            self.mass_kg,
            self.hub_drop_m,
            self.wheel_radius_m,
            self.wheel_width_m,
            self.wheel_inertia_kg_m2,
            self.suspension_rest_m,
            self.suspension_stiffness_n_m,
            self.suspension_damping_n_s_m,
            self.wheel_stall_force_n,
            self.wheel_no_load_speed_m_s,
            self.slip_stiffness_n_s_m,
            self.drive_friction,
        ];
        if positive.iter().any(|v| !v.is_finite() || *v <= 0.0) {
            return Err("chassis quantities must be finite and positive");
        }
        if !self.roller_friction.is_finite() || self.roller_friction < 0.0 {
            return Err("roller friction must be finite and non-negative");
        }
        if self.body_half_m.iter().any(|v| !v.is_finite() || *v <= 0.0) {
            return Err("body half extents must be finite and positive");
        }
        if let Some(f) = &self.omni_frame
            && ([
                f.corner_cut_m,
                f.ring_half_m,
                f.ring_corner_cut_m,
                f.ring_width_m,
                f.ring_height_m,
            ]
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.0)
                || !f.ring_center_z_m.is_finite()
                || f.corner_cut_m >= self.body_half_m[0].min(self.body_half_m[1])
                || f.ring_corner_cut_m >= f.ring_half_m
                || f.ring_width_m >= f.ring_corner_cut_m
                || f.ring_half_m - f.ring_width_m <= self.body_half_m[0].max(self.body_half_m[1])
                || f.ring_center_z_m - 0.5 * f.ring_height_m >= -self.body_half_m[2])
        {
            return Err("invalid octagonal frame or ring dimensions");
        }
        if self
            .turret_half_m
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.0)
            || self.turret_center_m.iter().any(|v| !v.is_finite())
        {
            return Err("turret extents must be finite and positive");
        }
        if self.turret_center_m[2] + self.turret_half_m[2] <= self.body_half_m[2]
            && !(self.planar_flight
                && self.turret_center_m[2] - self.turret_half_m[2] < -self.body_half_m[2])
        {
            return Err("turret top must be above the body top");
        }
        if self.wheel_hubs_m.is_empty() && !self.planar_flight {
            return Err("chassis needs at least one wheel");
        }
        if self
            .wheel_hubs_m
            .iter()
            .any(|[x, y]| !x.is_finite() || !y.is_finite() || x.hypot(*y) < 1e-3)
        {
            return Err("wheel hubs must be finite and off the body centre");
        }
        if !self.armor_standoff_m.is_finite()
            || self.armor_standoff_m < 0.0
            || !self.armor_height_m.is_finite()
            || !self.armor_tilt_rad.is_finite()
            || self.armor_tilt_rad.abs() >= std::f64::consts::FRAC_PI_2
        {
            return Err(
                "armor standoff must be finite and non-negative, its height finite and its tilt under a quarter turn",
            );
        }
        Ok(())
    }
}

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

/// Ground contact of one wheel at the end of a tick.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct WheelContact {
    /// Contact point in world FLU metres.
    pub point_m: [f64; 3],
    /// Ground normal, turned to face the body's up axis.
    pub normal: [f64; 3],
    /// Suspension force pressing the wheel onto the ground.
    pub load_n: f64,
    /// Wheel tread minus ground surface speed along the rolling direction.
    pub slip_m_s: f64,
    /// Applied rolling traction at this contact, in newtons.
    pub drive_force_n: f64,
}
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

/// Body-frame wheel geometry in `ChassisConfig::wheel_hubs_m` order.
fn wheel_geometry(config: &ChassisConfig) -> Vec<WheelGeometry> {
    wheel_geometry_at(config, 0.)
}
fn wheel_geometry_at(config: &ChassisConfig, angle_rad: f64) -> Vec<WheelGeometry> {
    let hubs = config.climbing.as_ref().map(|c| c.poses(angle_rad).hubs_m);
    config
        .wheel_hubs_m
        .iter()
        .enumerate()
        .map(|(index, &[x, y])| {
            let [x, y, z] = hubs.map_or([x, y, -config.hub_drop_m], |h| {
                let mut p = h[index];
                p[2] += config.climbing.as_ref().unwrap().spring_anchor_offset_m;
                p
            });
            let lever_m = x.hypot(y);
            // Mecanum contact force is perpendicular to the free roller,
            // at 45 degrees to the wheel plane, with mirrored handedness.
            let roll = if config.mecanum {
                Vector::new(1.0, -(x * y).signum(), 0.0).normalize()
            } else {
                Vector::new(-y / lever_m, x / lever_m, 0.0)
            };
            WheelGeometry {
                hub_m: Vector::new(x, y, z),
                velocity_m_s: Vector::ZERO,
                roll,
                axle: Vector::new(roll.y, -roll.x, 0.0),
                lever_m: x * roll.y - y * roll.x,
            }
        })
        .collect()
}

/// Fixed body-frame geometry of one wheel.
struct WheelGeometry {
    hub_m: Vector,
    velocity_m_s: Vector,
    /// Unit rolling direction in the body's horizontal plane.
    roll: Vector,
    /// Free roller direction in the contact plane. Radial for omni wheels.
    axle: Vector,
    /// Signed contact speed per unit yaw rate, from the hub cross rolling direction.
    lever_m: f64,
}
#[derive(Clone, Copy)]
struct WheelState {
    spin_rad: f64,
    angular_velocity_rad_s: f64,
    suspension_compression_m: f64,
    target_m_s: f64,
    contact: Option<WheelContact>,
}

/// Reusable per-wheel solver storage; none of it is physical restore state.
#[derive(Default)]
struct WheelDriveScratch {
    contacts: Vec<(f64, f64)>,
    forces: Vec<f64>,
    motors: Vec<f64>,
}
impl WheelDriveScratch {
    /// Integrate motor torque and equal/opposite tyre reaction, with shared power.
    /// Eight internal slices per Rapier slice resolve the stiff slip law without
    /// changing the caller-selected integration duration.
    fn integrate(
        &mut self,
        config: &ChassisConfig,
        wheels: &mut [WheelState],
        dt_s: f64,
        budget_w: f64,
        torques: Option<&WheelTorques>,
    ) -> (f64, f64) {
        let contacts = &self.contacts;
        let ratio = config.wheel_radius_m
            / if config.mecanum {
                std::f64::consts::SQRT_2
            } else {
                1.0
            };
        let slices = 8;
        let dt = dt_s / f64::from(slices);
        self.forces.resize(wheels.len(), 0.0);
        self.forces.fill(0.0);
        self.motors.resize(wheels.len(), 0.0);
        let forces = &mut self.forces;
        let motors = &mut self.motors;
        let mut average_scale = 0.0;
        let mut power_w = 0.0;
        for _ in 0..slices {
            let mut mechanical_w = 0.0;
            let mut copper_w = 0.0;
            for (index, (wheel, motor)) in wheels.iter().zip(motors.iter_mut()).enumerate() {
                let speed = wheel.angular_velocity_rad_s * ratio;
                let requested_force = torques.map_or_else(
                    || 4.0 * config.slip_stiffness_n_s_m * (wheel.target_m_s - speed),
                    |torques| torques.as_nm()[index] / ratio,
                );
                let limit = config.wheel_stall_force_n
                    * if requested_force * speed > 0.0 {
                        (1.0 - speed.abs() / config.wheel_no_load_speed_m_s).clamp(0.0, 1.0)
                    } else {
                        1.0
                    };
                *motor = requested_force.clamp(-limit, limit);
                mechanical_w += (*motor * speed).max(0.0);
                copper_w += config.dynamics.motor_stall_loss_w
                    * (*motor / config.wheel_stall_force_n).powi(2);
            }
            let scale = drive_power_scale(mechanical_w, copper_w, budget_w);
            average_scale += scale / f64::from(slices);
            power_w += (mechanical_w * scale + copper_w * scale * scale) / f64::from(slices);
            for (((wheel, &motor), &(ground_speed, grip)), force) in wheels
                .iter_mut()
                .zip(motors.iter())
                .zip(contacts)
                .zip(forces.iter_mut())
            {
                let speed = wheel.angular_velocity_rad_s * ratio;
                let traction =
                    (config.slip_stiffness_n_s_m * (speed - ground_speed)).clamp(-grip, grip);
                // Tyre reaction resists the motor even when the body cannot move.
                wheel.angular_velocity_rad_s +=
                    (motor * scale - traction) * ratio / config.wheel_inertia_kg_m2 * dt;
                wheel.spin_rad = wrap_angle(wheel.spin_rad + wheel.angular_velocity_rad_s * dt);
                *force += traction / f64::from(slices);
            }
        }
        for (wheel, &(ground_speed, _)) in wheels.iter_mut().zip(contacts) {
            if let Some(contact) = &mut wheel.contact {
                contact.slip_m_s = wheel.angular_velocity_rad_s * ratio - ground_speed;
            }
        }
        (average_scale, power_w)
    }
}

/// One chassis inside a physics world: its Rapier bodies and colliders, the
/// wheel geometry resolved from its configuration, and the drive and gimbal
/// state carried between ticks.
pub(crate) struct Chassis {
    fold_angle_rad: f64,
    fold_colliders: Vec<ColliderHandle>,
    placement_revision: u64,
    id: u32,
    team: Team,
    config: ChassisConfig,
    body: RigidBodyHandle,
    /// Armor housing colliders on the body, in `armor_faces` order.
    armor: Vec<ColliderHandle>,
    geometry: Vec<WheelGeometry>,
    wheels: Vec<WheelState>,
    /// Reused each tick; collect forces before mutably borrowing the body.
    forces: Vec<(Vector, Vector)>,
    drive_forces: Vec<(Vector, Vector)>,
    /// Per-wheel contacts and motor integration buffers reused across solver slices.
    wheel_drive: WheelDriveScratch,
    command: ChassisCommand,
    drive_control: DriveControl,
    jump_held: bool,
    balance_gains: [f64; 2],
    /// Actual motor heading and elevation; defeat freezes them.
    aim_rad: [f64; 2],
    gimbal_velocity_rad_s: [f64; 2],
    defeated: bool,
    drive_budget_w: f64,
    drive_power_w: f64,
    actuation_cut: bool,
    /// Colliders the armor housings touched after the previous substep, so a
    /// resting contact registers one collision rather than one per substep.
    pub(crate) armor_touching: Vec<ColliderHandle>,
}

fn pose_of(body: &RigidBody) -> Pose {
    let p = body.position();
    Pose {
        translation_m: p.translation.to_array(),
        rotation_wxyz: [p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z],
    }
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

impl Chassis {
    /// Insert the body into `world` at `spawn`. The wheels find the ground on
    /// the first step, so spawning a little above it is fine.
    pub(crate) fn new(
        world: &mut PhysicsWorld,
        id: u32,
        team: Team,
        config: ChassisConfig,
        spawn: Pose,
    ) -> Result<Self, &'static str> {
        config.validate()?;
        if !pose_is_valid(spawn) {
            return Err("chassis spawn pose must be finite with a unit rotation");
        }
        let body = world.insert_body(
            RigidBodyBuilder::dynamic()
                .gravity_scale(if config.planar_flight { 0.0 } else { 1.0 })
                .enabled_translations(true, true, !config.planar_flight)
                .enabled_rotations(!config.planar_flight, !config.planar_flight, true)
                .pose(rapier_pose(spawn))
                .linear_damping(LINEAR_DAMPING)
                .angular_damping(ANGULAR_DAMPING)
                .can_sleep(false),
        );
        // The turret carries a share of the mass so the centre of mass sits
        // above the body centre, as on a real robot. A flying body keeps all
        // of it centred instead: its roll and pitch locks hold only while
        // the inertia stays axis-aligned, and its gimbal hangs off-centre.
        let turret_share =
            if config.planar_flight || config.folding.is_some() || config.articulation.is_some() {
                0.0
            } else {
                TURRET_MASS_SHARE
            };
        let [hx, hy, hz] = config.body_half_m;
        let deck = if config.omni_frame.is_some() {
            let points: Vec<_> = config
                .body_outline_m()
                .iter()
                .flat_map(|&[x, y]| [Vector::new(x, y, -hz), Vector::new(x, y, hz)])
                .collect();
            ColliderBuilder::convex_hull(&points).ok_or("invalid octagonal deck")?
        } else {
            ColliderBuilder::cuboid(hx, hy, hz)
        };
        world.insert_collider(
            deck.mass(config.mass_kg * (1.0 - turret_share))
                .friction(BODY_FRICTION)
                .restitution(ROBOT_RESTITUTION)
                .restitution_combine_rule(CoefficientCombineRule::Min),
            Some(body),
        );
        if config.team_cad {
            // Source-envelope gameplay proxies, not visual triangles or mass estimates.
            // The center case extends below the nominal frame; ammunition bays sit
            // above the scoring faces and inset from their leaning upper edges.
            for (half, centre) in [
                ([0.103, 0.103, 0.079], [0., 0., -0.030]),
                ([0.10, 0.07, 0.032], [0.145, 0.16, 0.077]),
                ([0.10, 0.07, 0.032], [-0.145, 0.16, 0.077]),
                ([0.10, 0.07, 0.032], [0.145, -0.16, 0.077]),
                ([0.10, 0.07, 0.032], [-0.145, -0.16, 0.077]),
            ] {
                world.insert_collider(
                    ColliderBuilder::cuboid(half[0], half[1], half[2])
                        .translation(vector(centre))
                        .density(0.0)
                        .friction(BODY_FRICTION)
                        .restitution(ROBOT_RESTITUTION),
                    Some(body),
                );
            }
        }
        for prism in config.bumper_prisms_m() {
            let vertices = prism.map(Vector::from_array);
            world.insert_collider(
                ColliderBuilder::convex_hull(&vertices)
                    .ok_or("invalid bumper prism")?
                    .density(0.0)
                    .friction(BODY_FRICTION)
                    .restitution(ROBOT_RESTITUTION)
                    .restitution_combine_rule(CoefficientCombineRule::Min),
                Some(body),
            );
        }
        let [tx, ty, tz] = config.turret_half_m;
        let [cx, cy, cz] = config.turret_center_m;
        let turret_top = cz + tz;
        let column_bottom = if config.planar_flight {
            (cz - tz).min(hz)
        } else {
            hz
        };
        let column_top = turret_top.max(hz);
        let column_half = (column_top - column_bottom) * 0.5;
        let mut fold_colliders = Vec::new();
        if let Some(folding) = &config.folding {
            for (pose, half) in folding.collision_parts(Pose::default(), [0., 0.], 0.) {
                fold_colliders.push(
                    world.insert_collider(
                        ColliderBuilder::cuboid(half[0], half[1], half[2])
                            .position(rapier_pose(pose))
                            .density(0.)
                            .friction(BODY_FRICTION)
                            .restitution(ROBOT_RESTITUTION),
                        Some(body),
                    ),
                );
            }
        } else if config.articulation.is_none() {
            world.insert_collider(
                ColliderBuilder::cuboid(tx, ty, column_half)
                    .translation(Vector::new(cx, cy, column_bottom + column_half))
                    .mass(config.mass_kg * turret_share)
                    .friction(BODY_FRICTION)
                    .restitution(ROBOT_RESTITUTION)
                    .restitution_combine_rule(CoefficientCombineRule::Min),
                Some(body),
            );
        }
        if config.articulation.is_some() {
            for (p, h) in config.transform_collision_parts(Pose::default(), [0., 0.], 0.) {
                fold_colliders.push(
                    world.insert_collider(
                        ColliderBuilder::cuboid(h[0], h[1], h[2])
                            .position(rapier_pose(p))
                            .density(0.)
                            .friction(BODY_FRICTION),
                        Some(body),
                    ),
                );
            }
        }
        if config.climbing.is_some() {
            // Fixed electronics shelf is behind the front entry bay; a full
            // depth bounding box would falsely fill the empty climbing clearance.
            world.insert_collider(
                ColliderBuilder::cuboid(0.05, 0.26, 0.006)
                    .translation(Vector::new(-0.075, 0., -0.16))
                    .density(0.)
                    .friction(BODY_FRICTION)
                    .restitution(ROBOT_RESTITUTION),
                Some(body),
            );
            for (pose, half) in config.transform_collision_parts(Pose::default(), [0., 0.], 0.) {
                fold_colliders.push(
                    world.insert_collider(
                        ColliderBuilder::cuboid(half[0], half[1], half[2])
                            .position(rapier_pose(pose))
                            .density(0.)
                            .friction(BODY_FRICTION)
                            .restitution(ROBOT_RESTITUTION),
                        Some(body),
                    ),
                );
            }
        }
        // Armor housings ride the body without adding to its mass.
        let [ax, ay, az] = config.armor_housing_half_m();
        let armor = config
            .armor_faces()
            .iter()
            .map(|face| {
                let local =
                    rapier_pose(*face) * Pose3::from_translation(Vector::new(-ax, 0.0, 0.0));
                world.insert_collider(
                    ColliderBuilder::cuboid(ax, ay, az)
                        .position(local)
                        .density(0.0)
                        .friction(FRICTION)
                        .restitution(ROBOT_RESTITUTION)
                        .restitution_combine_rule(CoefficientCombineRule::Min),
                    Some(body),
                )
            })
            .collect();
        for plate in 0..ARMOR_COUNT {
            for prism in config.armor_bracket_prisms_m(plate) {
                world.insert_collider(
                    ColliderBuilder::convex_hull(&prism.map(Vector::from_array))
                        .ok_or("invalid armor carrier")?
                        .density(0.0)
                        .friction(BODY_FRICTION)
                        .restitution(ROBOT_RESTITUTION),
                    Some(body),
                );
            }
        }
        let geometry = wheel_geometry(&config);
        let wheels = vec![
            WheelState {
                spin_rad: 0.0,
                angular_velocity_rad_s: 0.0,
                suspension_compression_m: 0.0,
                target_m_s: 0.0,
                contact: None,
            };
            geometry.len()
        ];
        Ok(Self {
            fold_angle_rad: 0.,
            fold_colliders,
            placement_revision: 0,
            id,
            team,
            balance_gains: balance_lqr_gains(&config),
            config,
            body,
            armor,
            forces: Vec::with_capacity(geometry.len() * 2),
            drive_forces: Vec::with_capacity(geometry.len()),
            wheel_drive: WheelDriveScratch::default(),
            geometry,
            wheels,
            jump_held: false,
            drive_control: DriveControl::default(),
            command: ChassisCommand {
                aim_yaw_rad: yaw_of(spawn),
                ..Default::default()
            },
            aim_rad: [yaw_of(spawn), 0.0],
            gimbal_velocity_rad_s: [0.0; 2],
            drive_budget_w: f64::MAX,
            drive_power_w: 0.,
            actuation_cut: false,
            defeated: false,
            armor_touching: Vec::new(),
        })
    }
    /// Field-assigned identity of this chassis.
    pub(crate) fn id(&self) -> u32 {
        self.id
    }
    /// Handle of the dynamic body that carries the chassis.
    pub(crate) fn body(&self) -> RigidBodyHandle {
        self.body
    }
    /// The four armor housing colliders, in `ChassisConfig::armor_faces` order.
    pub(crate) fn armor_colliders(&self) -> &[ColliderHandle] {
        &self.armor
    }
    /// World pose of armor scoring face `plate` right now.
    pub(crate) fn armor_pose(&self, world: &PhysicsWorld, plate: usize) -> Pose3 {
        *world.bodies[self.body].position() * rapier_pose(self.config.armor_faces()[plate])
    }
    /// Authoritative held turret pose, including the aim frozen by defeat.
    pub(crate) fn turret_pose(&self, world: &PhysicsWorld) -> Pose {
        let body = &world.bodies[self.body];
        if let Some(folding) = &self.config.folding {
            return folding
                .poses(pose_of(body), self.aim_rad, self.fold_angle_rad)
                .head;
        }
        if let Some(a) = &self.config.articulation
            && let Some(index) = a.joints.iter().position(|j| j.name == "pitch")
        {
            let local = body.position().rotation.inverse()
                * Vector::new(
                    self.aim_rad[1].cos() * self.aim_rad[0].cos(),
                    self.aim_rad[1].cos() * self.aim_rad[0].sin(),
                    self.aim_rad[1].sin(),
                );
            let frames = a.poses(
                self.fold_angle_rad,
                [local.y.atan2(local.x), local.z.clamp(-1., 1.).asin()],
            );
            let head = body.position() * rapier_pose(frames[index]);
            return Pose {
                translation_m: head.translation.to_array(),
                rotation_wxyz: [
                    head.rotation.w,
                    head.rotation.x,
                    head.rotation.y,
                    head.rotation.z,
                ],
            };
        }
        Pose {
            translation_m: body
                .position()
                .transform_point(vector(self.config.turret_center_m))
                .to_array(),
            rotation_wxyz: aim_rotation(self.aim_rad[0], self.aim_rad[1]),
        }
    }
    /// World pose of the gun muzzle: the turret pose carried `MUZZLE_FORWARD_M`
    /// along the barrel's +x, which is the launch direction `fire` uses.
    pub(crate) fn muzzle_pose(&self, world: &PhysicsWorld) -> Pose {
        let turret = self.turret_pose(world);
        let transform = rapier_pose(turret);
        Pose {
            translation_m: transform
                .transform_point(vector(self.config.muzzle_offset_m()))
                .to_array(),
            rotation_wxyz: turret.rotation_wxyz,
        }
    }
    /// Placement revision, bumped by `place` and by revival.
    pub(crate) fn revision(&self) -> u64 {
        self.placement_revision
    }
    /// Whether the referee has this robot at zero HP.
    pub(crate) fn set_power(&mut self, budget_w: f64, actuation_cut: bool) {
        self.drive_budget_w = budget_w.max(0.);
        self.actuation_cut = actuation_cut;
    }
    pub(crate) fn drive_power_w(&self) -> f64 {
        self.drive_power_w
    }
    pub(crate) fn is_defeated(&self) -> bool {
        self.defeated
    }
    /// Cut or restore the drive. A revived robot picks up its pilot's
    /// current aim again.
    pub(crate) fn set_defeated(&mut self, defeated: bool) {
        if self.defeated && !defeated {
            self.placement_revision = self
                .placement_revision
                .checked_add(1)
                .expect("chassis revision overflow");
        }
        self.defeated = defeated;
        if defeated {
            self.jump_held = false;
            self.gimbal_velocity_rad_s = [0.0; 2];
        }
    }
    /// Team this chassis plays for.
    pub(crate) fn team(&self) -> Team {
        self.team
    }
    /// Configuration the chassis was built with.
    pub(crate) fn config(&self) -> &ChassisConfig {
        &self.config
    }
    /// Take the body and its colliders out of `world`.
    pub(crate) fn remove(self, world: &mut PhysicsWorld) {
        world.remove_body(self.body);
    }
    /// Reposition for local inspection, preserving identity and defeat state.
    pub(crate) fn place(
        &mut self,
        world: &mut PhysicsWorld,
        spawn: Pose,
    ) -> Result<(), &'static str> {
        if !pose_is_valid(spawn) {
            return Err("chassis spawn pose must be finite with a unit rotation");
        }
        self.placement_revision = self
            .placement_revision
            .checked_add(1)
            .ok_or("placement revision exhausted")?;
        let body = &mut world.bodies[self.body];
        body.set_position(rapier_pose(spawn), true);
        body.set_linvel(Vector::ZERO, true);
        body.set_angvel(Vector::ZERO, true);
        body.reset_forces(true);
        body.reset_torques(true);
        self.jump_held = false;
        self.drive_control = DriveControl::default();
        self.command = ChassisCommand {
            aim_yaw_rad: yaw_of(spawn),
            ..Default::default()
        };
        self.gimbal_velocity_rad_s = [0.0; 2];
        if !self.defeated {
            self.aim_rad = [yaw_of(spawn), 0.0];
        }
        for wheel in &mut self.wheels {
            wheel.contact = None;
            wheel.target_m_s = 0.0;
            wheel.angular_velocity_rad_s = 0.0;
            wheel.suspension_compression_m = 0.0;
        }
        world
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut world.colliders);
        Ok(())
    }
    /// Store a command already checked by the caller; rejects non-finite input.
    pub(crate) fn set_command(&mut self, command: ChassisCommand) -> Result<(), &'static str> {
        if !command.is_finite() {
            return Err("chassis command must be finite");
        }
        self.command = command;
        self.drive_control = DriveControl::BodyVelocity;
        Ok(())
    }
    /// Select direct wheel drive without changing the held gimbal/folding request.
    pub(crate) fn set_wheel_torques(&mut self, torques: WheelTorques) -> Result<(), &'static str> {
        torques.validate(self.wheels.len())?;
        self.drive_control = DriveControl::WheelTorques(torques);
        Ok(())
    }
    /// Restore observable rigid-body and wheel state for presentation replay.
    /// Contact solver caches are deliberately rebuilt, not copied from the host.
    pub(crate) fn restore_prediction(
        &mut self,
        world: &mut PhysicsWorld,
        state: &ChassisSnapshot,
    ) -> Result<(), &'static str> {
        if !state
            .velocity_m_s
            .into_iter()
            .chain(state.angular_velocity_rad_s)
            .all(f64::is_finite)
            || state
                .held_aim_rad
                .iter()
                .chain(&state.gimbal_velocity_rad_s)
                .any(|v| !v.is_finite())
            || state.wheels.len() != self.wheels.len()
            || state.wheels.iter().any(|w| {
                !w.spin_rad.is_finite()
                    || !w.angular_velocity_rad_s.is_finite()
                    || !w.suspension_compression_m.is_finite()
                    || w.suspension_compression_m < 0.0
            })
        {
            return Err("non-finite prediction state");
        }
        state.drive_control.validate(self.wheels.len())?;
        self.place(world, state.pose)?;
        world.bodies[self.body].set_position(exact_rapier_pose(state.pose), true);
        world
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut world.colliders);
        let max_angle = self.config.transform_limit_rad();
        if !state.fold_angle_rad.is_finite() || !(0. ..=max_angle).contains(&state.fold_angle_rad) {
            return Err("invalid folding state");
        }
        self.fold_angle_rad = state.fold_angle_rad;
        self.geometry = wheel_geometry_at(&self.config, self.fold_angle_rad);
        self.placement_revision = state.placement_revision;
        self.set_command(state.command)?;
        self.drive_control = state.drive_control.clone();
        self.defeated = state.defeated;
        self.jump_held = state.jump_held;
        self.aim_rad = state.held_aim_rad;
        self.gimbal_velocity_rad_s = state.gimbal_velocity_rad_s;
        self.sync_fold_colliders(world);
        let body = &mut world.bodies[self.body];
        body.set_linvel(Vector::from_array(state.velocity_m_s), true);
        body.set_angvel(Vector::from_array(state.angular_velocity_rad_s), true);
        for (wheel, saved) in self.wheels.iter_mut().zip(&state.wheels) {
            wheel.spin_rad = saved.spin_rad;
            wheel.angular_velocity_rad_s = saved.angular_velocity_rad_s;
            wheel.suspension_compression_m = saved.suspension_compression_m;
            // Both are outputs of the previous tick, recomputed before they
            // are used again; carrying them makes a restore snapshot equal.
            wheel.target_m_s = saved.target_m_s;
            wheel.contact = saved.contact;
        }
        Ok(())
    }
    /// Cast the wheel rays and load the body with this slice's suspension and
    /// tyre forces, integrating the gimbal and wheel spin by `dt_s`. Call once
    /// per solver slice before the world steps; forces are replaced, not
    /// accumulated, so repeated calls over one tick are safe.
    pub(crate) fn apply_forces(&mut self, world: &mut PhysicsWorld, dt_s: f64) {
        self.step_gimbal(dt_s);
        self.step_fold(world, dt_s);
        let cfg = &self.config;
        let (pose, linvel, angvel, com) = {
            let body = &world.bodies[self.body];
            (
                *body.position(),
                body.linvel(),
                body.angvel(),
                body.center_of_mass(),
            )
        };
        let up = pose.rotation * Vector::Z;
        // A defeated robot's drive is cut: the wheels brake to a stop.
        let command = if self.defeated
            || self.actuation_cut
            || self.drive_control != DriveControl::BodyVelocity
        {
            ChassisCommand::default()
        } else {
            self.command
        };
        if cfg.planar_flight {
            let mut wish = pose.rotation * Vector::new(command.forward_m_s, command.left_m_s, 0.0);
            wish.z = 0.0;
            let mut pull = Vector::ZERO;
            if let Some(tether) = &cfg.tether {
                let hook = pose.translation;
                let anchor = vector(tether.anchor_m(hook.to_array()));
                let outward = Vector::new(hook.x - anchor.x, hook.y - anchor.y, 0.0);
                if outward.length() > 1e-9 {
                    let outward = outward.normalize();
                    let slack = tether.slack_m(hook.to_array());
                    // Brake so the drone stops where the tether goes taut; once
                    // it is past, fly back in at up to 1 m/s.
                    let allowed = if slack > 0.0 {
                        (2.0 * FLIGHT_ACCELERATION_M_S2 * slack).sqrt()
                    } else {
                        -(FLIGHT_VELOCITY_GAIN_PER_S * -slack).min(1.0)
                    };
                    let out = wish.dot(outward);
                    if out > allowed {
                        wish -= outward * (out - allowed);
                    }
                    if slack < 0.0 {
                        let stretching = linvel.dot(outward).max(0.0);
                        pull = -outward
                            * (TETHER_STIFFNESS_PER_KG * -slack
                                + TETHER_DAMPING_PER_KG * stretching)
                                .min(TETHER_MAX_PULL_M_S2);
                    }
                }
            }
            let acceleration = ((wish - linvel) * FLIGHT_VELOCITY_GAIN_PER_S)
                .clamp_length_max(FLIGHT_ACCELERATION_M_S2)
                + pull;
            let body = &mut world.bodies[self.body];
            body.reset_forces(false);
            body.reset_torques(false);
            body.add_force(
                Vector::new(acceleration.x, acceleration.y, 0.0) * cfg.mass_kg,
                true,
            );
            body.add_torque(
                Vector::Z * ((command.yaw_rate_rad_s - angvel.z) * 3.0).clamp(-8.0, 8.0),
                true,
            );
            self.jump_held = self.command.jump;
            return;
        }
        let reach = cfg.wheel_radius_m + cfg.suspension_rest_m;
        // Projectile group 2 must not act as a suspension support. A ray spring
        // resting on a loose 17 mm ball would invent a rigid terrain column.
        let filter =
            QueryFilter::default()
                .exclude_rigid_body(self.body)
                .groups(InteractionGroups::new(
                    Group::ALL,
                    Group::ALL & !Group::GROUP_2,
                    InteractionTestMode::And,
                ));
        self.forces.clear();
        self.drive_forces.clear();
        self.wheel_drive.contacts.clear();
        for (wheel, geometry) in self.wheels.iter_mut().zip(&self.geometry) {
            let hub = pose.transform_point(geometry.hub_m);
            let axle = pose.rotation * geometry.axle;
            let target = command.forward_m_s * geometry.roll.x
                + command.left_m_s * geometry.roll.y
                + command.yaw_rate_rad_s * geometry.lever_m;
            wheel.target_m_s = target;
            let ray = Ray::new(hub, -up);
            let ground = world
                .cast_ray_and_get_normal(&ray, reach, true, filter)
                .and_then(|(_, hit)| {
                    let normal = if hit.normal.dot(up) < 0.0 {
                        -hit.normal
                    } else {
                        hit.normal
                    };
                    (normal.dot(up) > MIN_GROUND_COSINE && normal.is_finite())
                        .then_some((hit.time_of_impact, normal))
                });
            let Some((distance, normal)) = ground else {
                wheel.contact = None;
                wheel.suspension_compression_m = 0.0;
                self.wheel_drive.contacts.push((0.0, 0.0));
                self.drive_forces.push((Vector::ZERO, hub));
                continue;
            };
            let compression = reach - distance;
            wheel.suspension_compression_m = compression;
            let hub_velocity =
                linvel + angvel.cross(hub - com) + pose.rotation * geometry.velocity_m_s;
            let compression_rate = -hub_velocity.dot(up);
            let overtravel = (compression - cfg.dynamics.suspension_travel_m).max(0.0);
            let bump_stop = cfg.dynamics.bump_stop_stiffness_n_m
                * overtravel
                * (1.0 + overtravel / cfg.dynamics.suspension_travel_m);
            let load = (bump_stop
                + cfg.suspension_stiffness_n_m * compression
                + cfg.suspension_damping_n_s_m * compression_rate)
                .max(0.0);
            let point = ray.point_at(distance);
            let contact_velocity =
                linvel + angvel.cross(point - com) + pose.rotation * geometry.velocity_m_s;
            let roll_dir = normal.cross(axle).normalize();
            let roller_dir = (axle - normal * axle.dot(normal)).normalize();
            if !roll_dir.is_finite() || !roller_dir.is_finite() {
                wheel.contact = None;
                wheel.suspension_compression_m = 0.0;
                self.wheel_drive.contacts.push((0.0, 0.0));
                self.drive_forces.push((Vector::ZERO, hub));
                continue;
            }
            let roll_speed = contact_velocity.dot(roll_dir);
            let roller_speed = contact_velocity.dot(roller_dir);
            let grip = cfg.drive_friction * load;
            self.wheel_drive.contacts.push((roll_speed, grip));
            let roller_grip = cfg.roller_friction * load;
            let roller =
                (-cfg.slip_stiffness_n_s_m * roller_speed).clamp(-roller_grip, roller_grip);
            self.drive_forces.push((roll_dir, point));
            self.forces.push((roller_dir * roller, point));
            self.forces.push((up * load, hub));
            wheel.contact = Some(WheelContact {
                point_m: point.to_array(),
                normal: normal.to_array(),
                load_n: load,
                slip_m_s: 0.0,
                drive_force_n: 0.0,
            });
        }
        let body = &mut world.bodies[self.body];
        // Forces and torques are separate accumulators; both must be cleared.
        body.reset_forces(false);
        body.reset_torques(false);
        let budget = if self.defeated || self.actuation_cut {
            0.
        } else {
            self.drive_budget_w.min(cfg.dynamics.drive_power_w)
        };
        let torques = match &self.drive_control {
            DriveControl::WheelTorques(torques) if !self.defeated && !self.actuation_cut => {
                Some(torques)
            }
            _ => None,
        };
        let (scale, power) =
            self.wheel_drive
                .integrate(cfg, &mut self.wheels, dt_s, budget, torques);
        self.drive_power_w = power;
        for ((wheel, (force, _)), traction) in self
            .wheels
            .iter_mut()
            .zip(&mut self.drive_forces)
            .zip(self.wheel_drive.forces.iter().copied())
        {
            *force *= traction;
            if let Some(contact) = &mut wheel.contact {
                contact.drive_force_n = traction;
            }
        }
        let supported = self
            .wheels
            .iter()
            .filter(|wheel| wheel.contact.is_some())
            .count();
        if cfg.balance_assist
            && supported > 0
            && budget > 0.
            && self.drive_control == DriveControl::BodyVelocity
        {
            // Reduced inverted-pendulum LQR, with a small speed-error lean
            // setpoint. The motor/leg plant remains an approximation; this is
            // an assisted body torque, not a firmware wheel-leg controller.
            let lateral = pose.rotation * Vector::Y;
            let forward = pose.rotation * Vector::X;
            let pitch_error = up.cross(Vector::Z).dot(lateral).clamp(-1.0, 1.0).asin();
            let rate = angvel.dot(lateral);
            let desired_lean =
                ((command.forward_m_s - linvel.dot(forward)) * 0.045).clamp(-0.10, 0.10);
            let strength = f64::from(command.balance_control) / 100.0;
            let [kp, kd] = self.balance_gains;
            let torque = (kp * (pitch_error + scale * desired_lean) - kd * rate)
                .clamp(-180.0, 180.0)
                * strength;
            body.add_torque(lateral * torque, true);
            // A 2.4 m/s equivalent leg push gives about 0.29 m ballistic rise.
            // Both wheels must support an upright body. Holding the button,
            // replaying a checkpoint or pressing again in flight cannot stack it.
            if command.jump
                && !self.jump_held
                && supported == cfg.wheel_hubs_m.len()
                && up.dot(Vector::Z) > 0.85
                && linvel.z < 0.5
                && !self.defeated
            {
                body.apply_impulse(Vector::Z * cfg.mass_kg * 2.4, true);
            }
        }
        self.jump_held = self.command.jump;
        for (force, point) in self.drive_forces.drain(..) {
            body.add_force_at_point(force, point, true);
        }
        for (force, point) in self.forces.drain(..) {
            body.add_force_at_point(force, point, true);
        }
    }
    fn step_fold(&mut self, world: &mut PhysicsWorld, dt_s: f64) {
        let limit = self.config.transform_limit_rad();
        if limit == 0. {
            if self.config.articulation.is_some() {
                self.sync_fold_colliders(world);
            }
            return;
        }
        let speed = self
            .config
            .folding
            .as_ref()
            .map(|f| f.speed_rad_s)
            .or_else(|| self.config.articulation.as_ref().map(|a| a.speed_rad_s))
            .unwrap_or_else(|| self.config.climbing.as_ref().unwrap().speed_rad_s);
        let before = self.fold_angle_rad;
        let goal = if self.command.folded { limit } else { 0. };
        if !self.defeated && !self.actuation_cut {
            let change = speed * dt_s;
            let next = self.fold_angle_rad + (goal - self.fold_angle_rad).clamp(-change, change);
            if next == self.fold_angle_rad || self.fold_path_clear(world, next) {
                self.fold_angle_rad = next;
            }
        }
        if self.config.climbing.is_some() {
            let old = wheel_geometry_at(&self.config, before);
            self.geometry = wheel_geometry_at(&self.config, self.fold_angle_rad);
            for (g, p) in self.geometry.iter_mut().zip(old) {
                g.velocity_m_s = (g.hub_m - p.hub_m) / dt_s;
            }
        }
        self.sync_fold_colliders(world);
    }
    fn fold_path_clear(&self, world: &PhysicsWorld, next: f64) -> bool {
        let body = pose_of(&world.bodies[self.body]);
        let before = self
            .config
            .transform_collision_parts(body, self.aim_rad, self.fold_angle_rad);
        // Bound angular sampling even for a caller-supplied faster actuator.
        let samples = ((next - self.fold_angle_rad).abs() / 0.25_f64.to_radians())
            .ceil()
            .max(2.) as usize;
        for step in 1..=samples {
            let angle =
                self.fold_angle_rad + (next - self.fold_angle_rad) * step as f64 / samples as f64;
            for ((old_pose, _), (new_pose, half)) in
                before
                    .iter()
                    .copied()
                    .zip(
                        self.config
                            .transform_collision_parts(body, self.aim_rad, angle),
                    )
            {
                let shape = Cuboid::new(vector(half));
                let new_pose = rapier_pose(new_pose);
                let bounds = shape.compute_aabb(&new_pose);
                for (_, obstacle) in world.colliders.iter() {
                    if obstacle.parent() == Some(self.body)
                        || obstacle.is_sensor()
                        || !obstacle.is_enabled()
                        || !bounds.intersects(&obstacle.compute_aabb())
                    {
                        continue;
                    }
                    let penetration = |pose| {
                        rapier3d_f64::parry::query::contact(
                            obstacle.position(),
                            obstacle.shape(),
                            &pose,
                            &shape,
                            0.,
                        )
                        .map(|c| c.map_or(0., |c| (-c.dist).max(0.)))
                    };
                    let Ok(new_depth) = penetration(new_pose) else {
                        return false;
                    };
                    if new_depth <= 1e-6 {
                        continue;
                    }
                    let old_depth = penetration(rapier_pose(old_pose)).unwrap_or(0.);
                    if new_depth > old_depth + 1e-6 {
                        return false;
                    }
                }
            }
        }
        true
    }
    pub(crate) fn sync_fold_colliders(&self, world: &mut PhysicsWorld) {
        let body = pose_of(&world.bodies[self.body]);
        let inverse = world.bodies[self.body].position().inverse();
        for (handle, (pose, _)) in
            self.fold_colliders
                .iter()
                .zip(
                    self.config
                        .transform_collision_parts(body, self.aim_rad, self.fold_angle_rad),
                )
        {
            world.colliders[*handle].set_position_wrt_parent(inverse * rapier_pose(pose));
            world.colliders[*handle].set_position(rapier_pose(pose));
        }
    }
    fn step_gimbal(&mut self, dt_s: f64) {
        if self.defeated || self.actuation_cut {
            return;
        }
        let cfg = &self.config.dynamics;
        let targets = [
            wrap_angle(self.command.aim_yaw_rad),
            self.command.aim_pitch_rad.clamp(-1.4, 1.4),
        ];
        for (axis, target) in targets.into_iter().enumerate() {
            let error = if axis == 0 {
                wrap_angle(target - self.aim_rad[axis])
            } else {
                target - self.aim_rad[axis]
            };
            let speed = (error / cfg.gimbal_response_s)
                .clamp(-cfg.gimbal_max_speed_rad_s, cfg.gimbal_max_speed_rad_s);
            let rate = &mut self.gimbal_velocity_rad_s[axis];
            *rate += (speed - *rate).clamp(
                -cfg.gimbal_max_acceleration_rad_s2 * dt_s,
                cfg.gimbal_max_acceleration_rad_s2 * dt_s,
            );
            self.aim_rad[axis] += *rate * dt_s;
            if axis == 0 {
                self.aim_rad[axis] = wrap_angle(self.aim_rad[axis]);
            } else {
                self.aim_rad[axis] = self.aim_rad[axis].clamp(-1.4, 1.4);
            }
        }
    }
    pub(crate) fn control_state(&self, world: &PhysicsWorld) -> ChassisControlState {
        ChassisControlState {
            pose: {
                let body = &world.bodies[self.body];
                let q = body.rotation();
                Pose {
                    translation_m: body.translation().to_array(),
                    rotation_wxyz: [q.w, q.x, q.y, q.z],
                }
            },
            command: self.command,
            placement_revision: self.placement_revision,
            defeated: self.defeated,
        }
    }
    /// Read the body, wheel and gimbal state into a serializable snapshot.
    pub(crate) fn snapshot(&self, world: &PhysicsWorld) -> ChassisSnapshot {
        let body = &world.bodies[self.body];
        let rotation = body.rotation();
        let pose = *body.position();
        let turret = self.turret_pose(world);
        ChassisSnapshot {
            fold_angle_rad: self.fold_angle_rad,
            placement_revision: self.placement_revision,
            id: self.id,
            team: self.team,
            config: self.config.clone(),
            pose: Pose {
                translation_m: body.translation().to_array(),
                rotation_wxyz: [rotation.w, rotation.x, rotation.y, rotation.z],
            },
            turret,
            velocity_m_s: body.linvel().to_array(),
            angular_velocity_rad_s: body.angvel().to_array(),
            command: self.command,
            drive_control: self.drive_control.clone(),
            jump_held: self.jump_held,
            held_aim_rad: self.aim_rad,
            gimbal_velocity_rad_s: self.gimbal_velocity_rad_s,
            wheels: self
                .wheels
                .iter()
                .zip(&self.geometry)
                .map(|(wheel, geometry)| WheelSnapshot {
                    hub_m: pose
                        .transform_point(
                            geometry.hub_m
                                + Vector::Z
                                    * (wheel.suspension_compression_m
                                        - self.config.suspension_rest_m),
                        )
                        .to_array(),
                    spin_rad: wheel.spin_rad,
                    angular_velocity_rad_s: wheel.angular_velocity_rad_s,
                    suspension_compression_m: wheel.suspension_compression_m,
                    target_m_s: wheel.target_m_s,
                    contact: wheel.contact,
                })
                .collect(),
            defeated: self.defeated,
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

/// Uniform motor allocation: mechanical output scales linearly and copper loss quadratically.
// Closed-form continuous LQR for x=[pitch, pitch_rate], A=[[0,1],[mgh/I,0]],
// B=[0,1/I], Q=diag(1000,60), R=0.01. These cost weights and the fitted
// centre-of-mass height are prototype choices. Solve once per chassis creation.
fn balance_lqr_gains(config: &ChassisConfig) -> [f64; 2] {
    let h = config.hub_drop_m + TURRET_MASS_SHARE * config.turret_center_m[2];
    let inertia = config.mass_kg
        * ((config.body_half_m[0].powi(2) + config.body_half_m[2].powi(2)) / 3.0 + h * h);
    let gravity = config.mass_kg * 9.81 * h;
    let kp = gravity + (gravity * gravity + 1000.0 / 0.01).sqrt();
    let kd = (2.0 * inertia * kp + 60.0 / 0.01).sqrt();
    [kp, kd]
}

fn drive_power_scale(mechanical_w: f64, copper_w: f64, budget_w: f64) -> f64 {
    if budget_w <= 0.0 {
        return 0.0;
    }
    if mechanical_w + copper_w <= budget_w {
        return 1.0;
    }
    if copper_w <= f64::EPSILON {
        return (budget_w / mechanical_w).min(1.0);
    }
    (2.0 * budget_w
        / (mechanical_w + (mechanical_w * mechanical_w + 4.0 * copper_w * budget_w).sqrt()))
    .min(1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        projectile::{Caliber, Shot, TargetFrames, WorldPhysics},
        tick_ns,
    };
    fn integrate_wheel_drives(
        config: &ChassisConfig,
        wheels: &mut [WheelState],
        contacts: &[(f64, f64)],
        dt_s: f64,
        budget_w: f64,
        torques: Option<&WheelTorques>,
    ) -> (Vec<f64>, f64, f64) {
        let mut scratch = WheelDriveScratch {
            contacts: contacts.to_vec(),
            ..Default::default()
        };
        let (scale, power) = scratch.integrate(config, wheels, dt_s, budget_w, torques);
        (scratch.forces, scale, power)
    }

    fn chassis_world(config: ChassisConfig, spawn: Pose) -> WorldPhysics {
        let mut ballistics = WorldPhysics::new(&[], 0.0);
        ballistics.add_chassis(Team::Red, config, spawn).unwrap();
        ballistics
    }

    #[test]
    fn reused_drive_buffers_match_fresh_storage_across_contact_and_power_changes() {
        for config in [ChassisConfig::default(), ChassisConfig::folding_hero()] {
            let count = config.wheel_hubs_m.len();
            let initial = WheelState {
                spin_rad: 0.,
                angular_velocity_rad_s: 0.,
                suspension_compression_m: 0.,
                target_m_s: 2.,
                contact: None,
            };
            let mut actual = vec![initial; count];
            let mut expected = actual.clone();
            let mut scratch = WheelDriveScratch::default();
            let torques = WheelTorques::new(vec![0.3; count]).unwrap();
            let mut storage = None;
            for slice in 0..256 {
                scratch.contacts.clear();
                for corner in 0..count {
                    scratch.contacts.push(if (slice + corner) % 3 == 0 {
                        (0., 0.)
                    } else {
                        (slice as f64 * 0.001, 50.)
                    });
                }
                let budget = if slice % 5 == 0 { 0. } else { 40. };
                let direct = (slice % 2 == 0).then_some(&torques);
                let fresh = integrate_wheel_drives(
                    &config,
                    &mut expected,
                    &scratch.contacts,
                    0.001,
                    budget,
                    direct,
                );
                let result = scratch.integrate(&config, &mut actual, 0.001, budget, direct);
                assert_eq!(result, (fresh.1, fresh.2));
                assert_eq!(scratch.forces, fresh.0);
                for (actual, expected) in actual.iter().zip(&expected) {
                    assert_eq!(actual.spin_rad, expected.spin_rad);
                    assert_eq!(
                        actual.angular_velocity_rad_s,
                        expected.angular_velocity_rad_s
                    );
                }
                let pointers = (
                    scratch.contacts.as_ptr(),
                    scratch.forces.as_ptr(),
                    scratch.motors.as_ptr(),
                );
                if let Some(previous) = storage {
                    assert_eq!(
                        pointers, previous,
                        "solver buffers reallocated after warmup"
                    );
                }
                storage = Some(pointers);
            }
        }
    }

    #[test]
    fn direct_wheel_requests_validate_switch_and_restore_without_touching_aim() {
        let cfg = ChassisConfig::default();
        let mut world = chassis_world(cfg, Pose::at([0., 0., 3.]));
        let command = ChassisCommand {
            forward_m_s: 2.,
            aim_yaw_rad: 0.7,
            ..Default::default()
        };
        world.command_chassis(0, command).unwrap();
        assert!(WheelTorques::new(vec![f64::NAN; 4]).is_err());
        assert!(WheelTorques::new(vec![f64::INFINITY; 4]).is_err());
        assert!(WheelTorques::new(vec![]).is_err());
        assert!(
            world
                .command_chassis_wheel_torques(0, WheelTorques::new(vec![1.; 3]).unwrap())
                .is_err()
        );
        assert_eq!(
            world.chassis_snapshot(0).unwrap().drive_control,
            DriveControl::BodyVelocity
        );
        let torques = WheelTorques::new(vec![1., -1., 0., 0.5]).unwrap();
        assert!(
            world
                .command_chassis_wheel_torques(99, torques.clone())
                .is_err()
        );
        world
            .command_chassis_wheel_torques(0, torques.clone())
            .unwrap();
        run(&mut world, 8);
        let saved = world.chassis_snapshot(0).unwrap();
        assert_eq!(saved.command, command);
        assert_eq!(saved.drive_control, DriveControl::WheelTorques(torques));
        assert!(saved.wheels[0].angular_velocity_rad_s > 0.);
        assert!(saved.wheels[1].angular_velocity_rad_s < 0.);
        assert_eq!(saved.wheels[2].angular_velocity_rad_s, 0.);
        assert!(saved.wheels.iter().all(|w| w.target_m_s == 0.));
        let mut restored = WorldPhysics::new(&[], 0.);
        restored.restore_chassis(&saved).unwrap();
        let mut malformed = saved.clone();
        malformed.drive_control = DriveControl::WheelTorques(WheelTorques::new(vec![1.]).unwrap());
        assert!(
            WorldPhysics::new(&[], 0.)
                .restore_chassis(&malformed)
                .is_err()
        );
        let frames = TargetFrames::new(vec![]);
        for tick in 8..16 {
            world.step(tick * tick_ns(), &frames).unwrap();
            restored.step(tick * tick_ns(), &frames).unwrap();
        }
        let a = world.chassis_snapshot(0).unwrap();
        let b = restored.chassis_snapshot(0).unwrap();
        assert_eq!(a.drive_control, b.drive_control);
        for (a, b) in a.wheels.iter().zip(&b.wheels) {
            assert!((a.angular_velocity_rad_s - b.angular_velocity_rad_s).abs() < 1e-12);
        }
        world.command_chassis(0, command).unwrap();
        assert_eq!(
            world.chassis_snapshot(0).unwrap().drive_control,
            DriveControl::BodyVelocity
        );
        let mut drone = chassis_world(ChassisConfig::drone(), Pose::at([0., 0., 2.]));
        assert!(
            drone
                .command_chassis_wheel_torques(0, WheelTorques::new(vec![1.]).unwrap())
                .is_err()
        );
    }

    #[test]
    fn direct_motor_torques_respect_axis_units_limits_and_shared_power() {
        for cfg in [ChassisConfig::default(), ChassisConfig::folding_hero()] {
            let initial = WheelState {
                spin_rad: 0.,
                angular_velocity_rad_s: 0.,
                suspension_compression_m: 0.,
                target_m_s: -100.,
                contact: None,
            };
            let contacts = vec![(0., 0.); cfg.wheel_hubs_m.len()];
            let torques = WheelTorques::new(vec![0.01; contacts.len()]).unwrap();
            let mut wheels = vec![initial; contacts.len()];
            integrate_wheel_drives(
                &cfg,
                &mut wheels,
                &contacts,
                0.001,
                f64::MAX,
                Some(&torques),
            );
            // Axis torque/inertia is independent of the mecanum contact projection.
            assert!(
                (wheels[0].angular_velocity_rad_s - 0.01 / cfg.wheel_inertia_kg_m2 * 0.001).abs()
                    < 1e-12
            );
            let huge = WheelTorques::new(vec![f64::MAX; contacts.len()]).unwrap();
            let mut limited = vec![initial; contacts.len()];
            let (_, _, power) =
                integrate_wheel_drives(&cfg, &mut limited, &contacts, 0.01, 1., Some(&huge));
            assert!(power > 0. && power <= 1. + 1e-12);
            assert!(limited[0].angular_velocity_rad_s.is_finite());
            let zero = WheelTorques::new(vec![0.; contacts.len()]).unwrap();
            let before = limited[0].angular_velocity_rad_s;
            integrate_wheel_drives(&cfg, &mut limited, &contacts, 0.01, 1., Some(&zero));
            assert_eq!(limited[0].angular_velocity_rad_s, before);
            let ratio = cfg.wheel_radius_m
                / if cfg.mecanum {
                    std::f64::consts::SQRT_2
                } else {
                    1.
                };
            let mut overspeed = vec![
                WheelState {
                    angular_velocity_rad_s: 2. * cfg.wheel_no_load_speed_m_s / ratio,
                    ..initial
                };
                contacts.len()
            ];
            let before = overspeed[0].angular_velocity_rad_s;
            integrate_wheel_drives(&cfg, &mut overspeed, &contacts, 0.01, f64::MAX, Some(&huge));
            assert_eq!(overspeed[0].angular_velocity_rad_s, before);
            let brake = WheelTorques::new(vec![-0.01; contacts.len()]).unwrap();
            integrate_wheel_drives(
                &cfg,
                &mut overspeed,
                &contacts,
                0.01,
                f64::MAX,
                Some(&brake),
            );
            assert!(overspeed[0].angular_velocity_rad_s < before);
        }
    }

    #[test]
    fn direct_drive_obeys_defeat_power_cut_and_ground_traction() {
        let cfg = ChassisConfig::default();
        for (defeated, cut, budget) in [(true, false, 80.), (false, true, 80.), (false, false, 0.)]
        {
            let mut world = chassis_world(cfg.clone(), Pose::at([0., 0., 3.]));
            world
                .command_chassis_wheel_torques(0, WheelTorques::new(vec![1.; 4]).unwrap())
                .unwrap();
            world.set_chassis_defeated(0, defeated);
            world.set_chassis_power(0, budget, cut);
            let saved = run(&mut world, 16);
            assert!(saved.wheels.iter().all(|w| w.angular_velocity_rad_s == 0.));
            assert_eq!(world.chassis_drive_power_w(0), Some(0.));
        }
        let mut world = chassis_world(cfg.clone(), Pose::at([0., 0., cfg.rest_height_m()]));
        // Default tangential wheel order: these signs drive body +x.
        world
            .command_chassis_wheel_torques(0, WheelTorques::new(vec![-1., -1., 1., 1.]).unwrap())
            .unwrap();
        let saved = run(&mut world, 128);
        assert!(saved.pose.translation_m[0] > 0.1, "{:?}", saved.pose);
        assert!(saved.wheels.iter().any(|w| w.contact.is_some()));

        let balance = ChassisConfig::balance();
        let mut world = chassis_world(balance.clone(), Pose::at([0., 0., balance.rest_height_m()]));
        world
            .command_chassis(
                0,
                ChassisCommand {
                    jump: true,
                    forward_m_s: 2.,
                    ..Default::default()
                },
            )
            .unwrap();
        world
            .command_chassis_wheel_torques(0, WheelTorques::new(vec![0.; 2]).unwrap())
            .unwrap();
        let saved = run(&mut world, 1);
        assert!(
            saved.velocity_m_s[2] < 0.1,
            "direct mode must bypass assisted jump"
        );
    }

    #[test]
    fn compact_bumper_extends_the_lower_deck_and_stays_inside_the_stowed_width() {
        let cfg = ChassisConfig::default();
        cfg.validate().unwrap();
        assert_eq!(cfg.body_outline_m().len(), 8);
        let prisms = cfg.bumper_prisms_m();
        assert_eq!(prisms.len(), 56);
        for prism in &prisms {
            for p in prism {
                assert!(p[0].abs() <= 0.285 && p[1].abs() <= 0.285);
            }
        }
        let frame = cfg.omni_frame.as_ref().unwrap();
        for prism in &prisms[8..] {
            for (i, p) in prism.iter().enumerate() {
                let z =
                    frame.ring_center_z_m + if i < 4 { -0.5 } else { 0.5 } * frame.ring_height_m;
                assert!((p[2] - z).abs() < 1e-12);
            }
        }
        for prism in &prisms[..8] {
            for p in &prism[4..] {
                assert_eq!(p[2], -cfg.body_half_m[2]);
            }
        }
        // A ray into a diagonal corner misses the deck box's old square corner,
        // while a horizontal ray at bumper height hits the compact lower extrusion.
        let deck: Vec<_> = cfg
            .body_outline_m()
            .iter()
            .flat_map(|&[x, y]| [Vector::new(x, y, -0.05), Vector::new(x, y, 0.05)])
            .collect();
        let deck = SharedShape::convex_hull(&deck).unwrap();
        assert!(!deck.contains_point(&Pose3::IDENTITY, Vector::new(0.20, 0.20, 0.0)));
        let ring = SharedShape::compound(
            prisms
                .iter()
                .map(|p| {
                    (
                        Pose3::IDENTITY,
                        SharedShape::convex_hull(&p.map(Vector::from_array)).unwrap(),
                    )
                })
                .collect(),
        );
        let ray = Ray::new(Vector::new(0.5, 0.0, -0.07), -Vector::X);
        let distance = ring.cast_ray(&Pose3::IDENTITY, &ray, 1.0, true).unwrap();
        assert!((distance - 0.215).abs() < 1e-9);
        assert!(ring.contains_point(&Pose3::IDENTITY, Vector::new(0.0, 0.0, -0.07)));
    }

    #[test]
    fn lower_bumper_and_guards_clear_every_suspended_wheel() {
        let cfg = ChassisConfig::default();
        for wheel in wheel_geometry(&cfg) {
            for prism in cfg.bumper_prisms_m() {
                let points = prism
                    .iter()
                    .map(|p| Vector::new(p[0], p[1], 0.0))
                    .collect::<Vec<_>>();
                let mut axes = vec![wheel.axle, wheel.roll];
                for (i, point) in points.iter().enumerate() {
                    for other in &points[i + 1..] {
                        let edge = other - point;
                        if edge.length_squared() > 1e-20 {
                            axes.push(Vector::new(-edge.y, edge.x, 0.0).normalize());
                        }
                    }
                }
                assert!(
                    axes.into_iter().any(|axis| {
                        let centre = wheel.hub_m.dot(axis);
                        let radius = 0.5 * cfg.wheel_width_m * wheel.axle.dot(axis).abs()
                            + cfg.wheel_radius_m * wheel.roll.dot(axis).abs();
                        let low = points
                            .iter()
                            .map(|p| p.dot(axis))
                            .fold(f64::INFINITY, f64::min);
                        let high = points
                            .iter()
                            .map(|p| p.dot(axis))
                            .fold(f64::NEG_INFINITY, f64::max);
                        low - (centre + radius) > 0.002 || centre - radius - high > 0.002
                    }),
                    "bumper clips the wheel's swept envelope"
                );
            }
        }
    }

    #[test]
    fn armor_carriers_are_recessed_rigid_and_leave_the_ring_as_first_wall_contact() {
        let cfg = ChassisConfig::default();
        let mut armor_vertices = Vec::new();
        for (plate, face) in cfg.armor_faces().iter().enumerate() {
            let face = rapier_pose(*face);
            let prisms = cfg.armor_bracket_prisms_m(plate);
            assert_eq!(prisms.len(), if plate.is_multiple_of(2) { 4 } else { 3 });
            for prism in &prisms {
                for &point in prism {
                    let local = face.inverse_transform_point(Vector::from_array(point));
                    assert!(
                        local.x <= -0.019 + 1e-12,
                        "carrier protrudes through armor: {local:?}"
                    );
                }
            }
            // The carrier foot is horizontal and recessed onto the lower deck.
            for p in &prisms.last().unwrap()[..4] {
                assert!((p[2] + cfg.body_half_m[2]).abs() < 1e-12);
            }
            let [hx, hy, hz] = SMALL_ARMOR_HOUSING_HALF_M;
            for x in [-2.0 * hx, 0.0] {
                for y in [-hy, hy] {
                    for z in [-hz, hz] {
                        armor_vertices.push(face.transform_point(Vector::new(x, y, z)));
                    }
                }
            }
        }
        let ring: Vec<_> = cfg
            .bumper_prisms_m()
            .into_iter()
            .flatten()
            .map(Vector::from_array)
            .collect();
        // Every horizontal wall orientation must meet the bumper first, even
        // after including the plate tilt and full physical housing corners.
        for degree in 0..360 {
            let a = f64::from(degree).to_radians();
            let n = Vector::new(a.cos(), a.sin(), 0.0);
            let support = |vertices: &[Vector]| {
                vertices
                    .iter()
                    .map(|v| v.dot(n))
                    .fold(f64::NEG_INFINITY, f64::max)
            };
            assert!(support(&ring) - support(&armor_vertices) > 0.005);
        }
    }

    #[test]
    fn diagonal_wheels_clear_the_low_cardinal_armor_through_suspension_travel() {
        let cfg = ChassisConfig::default();
        for geometry in wheel_geometry(&cfg) {
            assert!((geometry.hub_m.x.abs() - geometry.hub_m.y.abs()).abs() < 1e-12);
            assert!((geometry.hub_m.x.hypot(geometry.hub_m.y) - 0.245).abs() < 1e-12);
            // Separation in each armor plate's width direction is independent
            // of wheel height, so this holds for full droop and compression.
            for width_axis in [Vector::X, Vector::Y] {
                let tyre_near = geometry.hub_m.dot(width_axis).abs()
                    - cfg.wheel_radius_m * geometry.roll.dot(width_axis).abs()
                    - 0.5 * cfg.wheel_width_m * geometry.axle.dot(width_axis).abs();
                assert!(tyre_near - SMALL_ARMOR_HOUSING_HALF_M[1] > 0.025);
            }
        }
        assert_eq!(cfg.armor_height_m, 0.030);
    }

    #[test]
    fn omni_opposing_armor_pairs_follow_the_stacked_beam_heights() {
        let cfg = ChassisConfig::default();
        let faces = cfg.armor_faces();
        assert_eq!(faces[0].translation_m[2], faces[2].translation_m[2]);
        assert_eq!(faces[1].translation_m[2], faces[3].translation_m[2]);
        assert!(
            (faces[0].translation_m[2] - faces[1].translation_m[2] - cfg.frame_beam_step_m()).abs()
                < 1e-12
        );
        let hero = ChassisConfig::hero().armor_faces();
        assert!(
            hero.iter()
                .all(|p| p.translation_m[2] == hero[0].translation_m[2])
        );
    }

    #[test]
    fn omni_springs_are_inboard_below_the_roof_and_follow_each_wheels_heave() {
        let cfg = ChassisConfig::default();
        for (wheel, hub) in cfg.wheel_hubs_m.iter().enumerate() {
            let [upper, lower] = cfg.suspension_mounts_m(wheel, 0.05).unwrap();
            assert!(
                upper[0].hypot(upper[1]) + 0.014 < hub[0].hypot(hub[1]) - cfg.wheel_width_m / 2.0
            );
            assert!(upper[2] + 0.014 < cfg.body_half_m[2]);
            assert!(lower[2] > -0.08);
            let [next_upper, next_lower] = cfg.suspension_mounts_m(wheel, 0.075).unwrap();
            assert_eq!(upper, next_upper);
            assert_eq!(&lower[..2], &next_lower[..2]);
            assert!((next_lower[2] - lower[2] - 0.025).abs() < 1e-12);
        }
    }

    #[test]
    fn airborne_motors_spin_up_with_inertia_and_restore_exactly() {
        let cfg = ChassisConfig::default();
        let mut physics = chassis_world(cfg.clone(), Pose::at([0.0, 0.0, 20.0]));
        physics
            .command_chassis(
                0,
                ChassisCommand {
                    forward_m_s: 2.0,
                    ..Default::default()
                },
            )
            .unwrap();
        let first = run(&mut physics, 1);
        let rate = first.wheels[1].angular_velocity_rad_s.abs();
        assert!(rate > 0.0 && rate < 2.0 / cfg.wheel_radius_m);
        let state = run(&mut physics, 32);
        assert!(
            state
                .wheels
                .iter()
                .all(|w| w.contact.is_none() && w.suspension_compression_m == 0.0)
        );
        assert!(state.wheels[1].angular_velocity_rad_s.abs() > rate);
        assert!(state.velocity_m_s[0].abs() < 1e-12);
        let mut restored = WorldPhysics::new(&[], 0.0);
        restored.restore_chassis(&state).unwrap();
        assert_eq!(restored.chassis_snapshots()[0], state);
        let a = run(&mut physics, 32);
        let b = run(&mut restored, 32);
        for (a, b) in a.wheels.iter().zip(b.wheels) {
            assert_eq!(a.angular_velocity_rad_s, b.angular_velocity_rad_s);
            assert_eq!(a.spin_rad, b.spin_rad);
        }
    }

    #[test]
    fn low_grip_spins_wheels_without_inventing_traction() {
        let cfg = ChassisConfig {
            drive_friction: 0.015,
            ..Default::default()
        };
        let mut physics = chassis_world(cfg.clone(), Pose::at([0.0, 0.0, cfg.rest_height_m()]));
        run(&mut physics, 256);
        physics
            .command_chassis(
                0,
                ChassisCommand {
                    forward_m_s: 2.5,
                    ..Default::default()
                },
            )
            .unwrap();
        let state = run(&mut physics, 64);
        assert!(state.velocity_m_s[0] < 0.3, "{state:?}");
        for i in 0..4 {
            let w = state.wheels[i];
            let c = w.contact.unwrap();
            assert!(w.angular_velocity_rad_s.abs() * cfg.wheel_radius_m > 1.0);
            assert!(c.slip_m_s.abs() > 0.8);
            assert!(c.drive_force_n.abs() <= cfg.drive_friction * c.load_n + 1e-8);
        }
    }

    #[test]
    fn a_loose_projectile_does_not_become_a_suspension_support() {
        let cfg = ChassisConfig::default();
        let mut physics = chassis_world(cfg.clone(), Pose::at([0.0, 0.0, cfg.rest_height_m()]));
        physics
            .fire(
                0,
                Pose::at([cfg.wheel_hubs_m[0][0], cfg.wheel_hubs_m[0][1], 0.030]),
                Shot {
                    caliber: Caliber::Mm17,
                    speed_m_s: 0.1,
                },
                None,
            )
            .unwrap();
        let state = run(&mut physics, 1);
        let front = state.wheels[0];
        assert!(front.suspension_compression_m < 0.001, "{front:?}");
        assert!(front.contact.unwrap().point_m[2].abs() < 1e-9);
    }

    #[test]
    fn each_suspension_responds_to_its_own_ground_height() {
        let cfg = ChassisConfig::default();
        let mut physics = chassis_world(cfg.clone(), Pose::at([0.0, 0.0, cfg.rest_height_m()]));
        let flat = run(&mut physics, 256);
        let [x, y] = cfg.wheel_hubs_m[0];
        physics
            .add_static_mesh(
                vec![
                    [x - 0.04, y - 0.04, 0.025],
                    [x + 0.04, y - 0.04, 0.025],
                    [x + 0.04, y + 0.04, 0.025],
                    [x - 0.04, y + 0.04, 0.025],
                ],
                vec![[0, 1, 2], [0, 2, 3]],
            )
            .unwrap();
        let raised = run(&mut physics, 1);
        assert!(
            raised.wheels[0].suspension_compression_m
                > flat.wheels[0].suspension_compression_m + 0.02
        );
        assert!(
            raised.wheels[0].contact.unwrap().load_n
                > raised.wheels[2].contact.unwrap().load_n + 15.0
        );
        assert!((raised.wheels[0].hub_m[2] - raised.wheels[2].hub_m[2] - 0.025).abs() < 0.003);
    }

    #[test]
    fn wheel_reaction_and_shared_motor_budget_are_independent_per_corner() {
        let cfg = ChassisConfig::default();
        let initial = WheelState {
            spin_rad: 0.0,
            angular_velocity_rad_s: 0.0,
            suspension_compression_m: 0.0,
            target_m_s: 2.0,
            contact: None,
        };
        let mut wheels = vec![initial; 4];
        let contacts = [(0.0, 0.0), (0.0, 100.0), (0.0, 100.0), (0.0, 100.0)];
        for _ in 0..64 {
            integrate_wheel_drives(
                &cfg,
                &mut wheels,
                &contacts,
                1.0 / 128.0,
                cfg.dynamics.drive_power_w,
                None,
            );
        }
        assert!(wheels[0].angular_velocity_rad_s > wheels[1].angular_velocity_rad_s + 5.0);
        assert_eq!(
            wheels[1].angular_velocity_rad_s,
            wheels[2].angular_velocity_rad_s
        );
        let mut restricted = cfg.clone();
        restricted.dynamics.drive_power_w = 10.0;
        let mut low = vec![initial; 4];
        let mut high = vec![initial; 4];
        for _ in 0..16 {
            integrate_wheel_drives(
                &restricted,
                &mut low,
                &contacts,
                1.0 / 128.0,
                restricted.dynamics.drive_power_w,
                None,
            );
            integrate_wheel_drives(
                &cfg,
                &mut high,
                &contacts,
                1.0 / 128.0,
                cfg.dynamics.drive_power_w,
                None,
            );
        }
        assert!(low[0].angular_velocity_rad_s < high[0].angular_velocity_rad_s);
    }

    #[test]
    fn gimbal_motor_is_tick_driven_rate_limited_and_restorable() {
        let config = ChassisConfig::default();
        let mut physics = chassis_world(config.clone(), Pose::at([0., 0., config.rest_height_m()]));
        physics
            .command_chassis(
                0,
                ChassisCommand {
                    aim_yaw_rad: 1.,
                    aim_pitch_rad: 0.4,
                    ..Default::default()
                },
            )
            .unwrap();
        assert_eq!(physics.chassis_snapshots()[0].held_aim_rad, [0.; 2]);
        let frames = TargetFrames::new(Vec::new());
        let mut previous_rate = [0.; 2];
        let tick_s = tick_ns() as f64 * 1e-9;
        for tick in 0..200 {
            physics.step(tick * tick_ns(), &frames).unwrap();
            let motor = physics.chassis_snapshots()[0].gimbal_velocity_rad_s;
            for axis in 0..2 {
                assert!(motor[axis].abs() <= config.dynamics.gimbal_max_speed_rad_s + 1e-10);
                assert!(
                    (motor[axis] - previous_rate[axis]).abs()
                        <= config.dynamics.gimbal_max_acceleration_rad_s2 * tick_s + 1e-10
                );
            }
            previous_rate = motor;
        }
        let saved = physics.chassis_snapshots().remove(0);
        let mut restored = chassis_world(config, saved.pose);
        restored.reset_chassis(&saved).unwrap();
        for tick in 200..500 {
            physics.step(tick * tick_ns(), &frames).unwrap();
            restored.step(tick * tick_ns(), &frames).unwrap();
            assert_eq!(
                physics.chassis_snapshots()[0].held_aim_rad,
                restored.chassis_snapshots()[0].held_aim_rad
            );
            assert_eq!(
                physics.chassis_snapshots()[0].gimbal_velocity_rad_s,
                restored.chassis_snapshots()[0].gimbal_velocity_rad_s
            );
        }
        assert!((physics.chassis_snapshots()[0].held_aim_rad[0] - 1.).abs() < 0.01);
        physics.set_chassis_defeated(0, true);
        let frozen = physics.chassis_snapshots()[0].held_aim_rad;
        physics.step(500 * tick_ns(), &frames).unwrap();
        assert_eq!(physics.chassis_snapshots()[0].held_aim_rad, frozen);
        assert_eq!(
            physics.chassis_snapshots()[0].gimbal_velocity_rad_s,
            [0.; 2]
        );
    }

    #[test]
    fn gimbal_takes_short_yaw_path_across_wrap() {
        let config = ChassisConfig::default();
        let mut physics = chassis_world(
            config.clone(),
            Pose::yawed([0., 0., config.rest_height_m()], std::f64::consts::PI - 0.1),
        );
        physics
            .command_chassis(
                0,
                ChassisCommand {
                    aim_yaw_rad: -std::f64::consts::PI + 0.1,
                    ..Default::default()
                },
            )
            .unwrap();
        let state = run(&mut physics, 1);
        assert!(state.gimbal_velocity_rad_s[0] > 0.);
        let state = run(&mut physics, 1000);
        assert!(wrap_angle(state.held_aim_rad[0] - (-std::f64::consts::PI + 0.1)).abs() < 1e-6);
    }

    #[test]
    fn landing_settles_without_gaining_bounce_height() {
        let config = ChassisConfig::default();
        let mut physics = chassis_world(config.clone(), Pose::at([0., 0., 0.6]));
        let frames = TargetFrames::new(Vec::new());
        let mut highest = 0.6_f64;
        for tick in 0..3000 {
            physics.step(tick * tick_ns(), &frames).unwrap();
            highest = highest.max(physics.chassis_snapshots()[0].pose.translation_m[2]);
        }
        let state = physics.chassis_snapshots().remove(0);
        assert!(highest <= 0.601, "landing gained height: {highest}");
        let sag = config.mass_kg * 9.81 / 4.0 / config.suspension_stiffness_n_m;
        assert!((state.pose.translation_m[2] - (config.rest_height_m() - sag)).abs() < 0.002);
        assert!(state.velocity_m_s.iter().all(|v| v.abs() < 0.01));
        assert!(state.wheels.iter().all(|w| w.contact.is_some()));
    }

    #[test]
    fn shared_power_budget_charges_stall_losses_and_has_no_regeneration_credit() {
        for mechanical in [0., 40., 200.] {
            for copper in [0., 60., 400.] {
                let scale = drive_power_scale(mechanical, copper, 80.);
                assert!((0.0..=1.0).contains(&scale));
                assert!(scale * mechanical + scale * scale * copper <= 80. + 1e-10);
            }
        }
        assert!(drive_power_scale(0., 100., 80.) < 1.);
    }

    #[test]
    fn muzzle_uses_rotated_body_pivot_and_freezes_held_aim_when_defeated() {
        let config = ChassisConfig::default();
        let spawn = Pose::yawed(
            [2.0, 3.0, config.rest_height_m()],
            std::f64::consts::FRAC_PI_2,
        );
        let mut physics = chassis_world(config.clone(), spawn);
        physics
            .command_chassis(
                0,
                ChassisCommand {
                    aim_yaw_rad: std::f64::consts::FRAC_PI_2,
                    aim_pitch_rad: 0.0,
                    ..Default::default()
                },
            )
            .unwrap();
        let aimed = physics.chassis_muzzle_pose(0).unwrap();
        let expected_pivot = rapier_pose(spawn).transform_point(vector(config.turret_center_m));
        assert!((aimed.translation_m[0] - expected_pivot.x).abs() < 1e-12);
        assert!((aimed.translation_m[1] - (expected_pivot.y + MUZZLE_FORWARD_M)).abs() < 1e-12);
        assert!((aimed.translation_m[2] - expected_pivot.z).abs() < 1e-12);

        physics.set_chassis_defeated(0, true);
        physics
            .command_chassis(
                0,
                ChassisCommand {
                    aim_yaw_rad: 0.0,
                    aim_pitch_rad: 0.4,
                    ..Default::default()
                },
            )
            .unwrap();
        assert_eq!(physics.chassis_muzzle_pose(0), Some(aimed));
        assert_eq!(physics.chassis_muzzle_pose(99), None);
    }
    fn run(ballistics: &mut WorldPhysics, ticks: u64) -> ChassisSnapshot {
        let frames = TargetFrames::new(Vec::new());
        for tick in 0..ticks {
            ballistics.step(tick * tick_ns(), &frames).unwrap();
        }
        ballistics.chassis_snapshots().remove(0)
    }
    #[test]
    fn drone_stays_in_its_plane_and_restores_its_drive() {
        let config = ChassisConfig::drone();
        config.validate().unwrap();
        let mut physics = chassis_world(config, Pose::at([0.0, 0.0, 1.6]));
        physics
            .command_chassis(
                0,
                ChassisCommand {
                    forward_m_s: 1.0,
                    left_m_s: 0.5,
                    yaw_rate_rad_s: 0.2,
                    jump: true,
                    ..Default::default()
                },
            )
            .unwrap();
        let state = run(&mut physics, 128);
        assert!(state.pose.translation_m[0] > 0.6);
        assert!(state.pose.translation_m[1] > 0.3);
        assert!((state.pose.translation_m[2] - 1.6).abs() < 1e-10);
        assert!(state.wheels.is_empty());
        assert!(state.pose.rotation_wxyz[1].abs() < 1e-10);
        assert!(state.pose.rotation_wxyz[2].abs() < 1e-10);
        let mut restored = WorldPhysics::new(&[], 0.0);
        restored.restore_chassis(&state).unwrap();
        let original = run(&mut physics, 128);
        let replayed = run(&mut restored, 128);
        for (a, b) in original
            .pose
            .translation_m
            .iter()
            .zip(replayed.pose.translation_m)
        {
            assert!((a - b).abs() < 1e-8);
        }
    }

    #[test]
    fn drone_shots_clear_its_own_frame_at_every_forward_pitch() {
        let config = ChassisConfig::drone();
        let mut physics = chassis_world(config, Pose::at([0.0, 0.0, 2.0]));
        let frames = TargetFrames::new(Vec::new());
        let mut time_ns = 0;
        // The flight controls allow -1.2 rad down to 0.79 rad up.
        for step in 0..=20 {
            let pitch = -1.2 + (0.79 + 1.2) * f64::from(step) / 20.0;
            physics
                .command_chassis(
                    0,
                    ChassisCommand {
                        aim_pitch_rad: pitch,
                        ..Default::default()
                    },
                )
                .unwrap();
            for _ in 0..128 {
                time_ns += tick_ns();
                physics.step(time_ns, &frames).unwrap();
            }
            let muzzle = physics.chassis_muzzle_pose(0).unwrap();
            let id = physics
                .fire(time_ns, muzzle, Shot::at_limit(Caliber::Mm17), Some(0))
                .unwrap();
            for _ in 0..8 {
                time_ns += tick_ns();
                physics.step(time_ns, &frames).unwrap();
            }
            let ball = physics.snapshot().into_iter().find(|b| b.id == id).unwrap();
            assert_eq!(
                ball.first_contact_ns, None,
                "pitch {pitch:.2} rad hit the drone"
            );
        }
    }

    #[test]
    fn tethered_drone_stops_at_its_reach_and_is_pulled_back_inside() {
        let tether = Tether {
            rope_start_m: [14.0, -5.8, 3.6],
            rope_end_m: [0.0, -5.8, 3.6],
            length_m: 2.4,
        };
        let config = ChassisConfig {
            tether: Some(tether),
            ..ChassisConfig::drone()
        };
        config.validate().unwrap();
        // Facing -x from above the pad, full speed towards the Snap Ring and
        // beyond it: the drone must brake to a stop within the tether's reach.
        let spawn = Pose::yawed([10.0, -5.8, 2.0], std::f64::consts::PI);
        let mut physics = chassis_world(config.clone(), spawn);
        physics
            .command_chassis(
                0,
                ChassisCommand {
                    forward_m_s: 4.0,
                    aim_yaw_rad: std::f64::consts::PI,
                    ..Default::default()
                },
            )
            .unwrap();
        let mut deepest = f64::INFINITY;
        let frames = TargetFrames::new(Vec::new());
        for tick in 0..128 * 8 {
            physics.step(tick * tick_ns(), &frames).unwrap();
            let hook = physics.chassis_snapshots()[0].pose.translation_m;
            deepest = deepest.min(tether.slack_m(hook));
        }
        let state = physics.chassis_snapshots().remove(0);
        // Beyond the ring by the tether's horizontal reach at 1.6 m below the rope.
        let reach = (2.4_f64.powi(2) - 1.6_f64.powi(2)).sqrt();
        assert!((state.pose.translation_m[0] + reach).abs() < 0.05);
        assert!(state.velocity_m_s[0].abs() < 0.05);
        assert!(deepest > -0.05, "overshot the tether by {deepest} m");

        // A drone placed well outside its reach is pulled back without the
        // pilot's help, and the pilot cannot hold it out.
        let mut outside = chassis_world(config, Pose::at([7.0, -1.0, 2.0]));
        outside
            .command_chassis(
                0,
                ChassisCommand {
                    left_m_s: 3.0,
                    ..Default::default()
                },
            )
            .unwrap();
        let state = run(&mut outside, 128 * 6);
        assert!(tether.slack_m(state.pose.translation_m) > -0.05);
        assert!((state.pose.translation_m[2] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn balance_stabilization_can_be_disabled_and_restored() {
        let pose = Pose {
            translation_m: [0.0, 0.0, 0.31],
            rotation_wxyz: [0.05_f64.cos(), 0.0, 0.05_f64.sin(), 0.0],
        };
        let mut assisted = chassis_world(ChassisConfig::balance(), pose);
        let mut unassisted = chassis_world(ChassisConfig::balance(), pose);
        unassisted
            .command_chassis(
                0,
                ChassisCommand {
                    balance_control: 0,
                    ..Default::default()
                },
            )
            .unwrap();
        let stable = run(&mut assisted, 128);
        let fallen = run(&mut unassisted, 128);
        assert!(stable.pose.rotation_wxyz[2].abs() < 0.025, "{stable:?}");
        assert!(fallen.pose.rotation_wxyz[2].abs() > 0.10, "{fallen:?}");
        assert!(
            unassisted
                .command_chassis(
                    0,
                    ChassisCommand {
                        balance_control: 101,
                        ..Default::default()
                    }
                )
                .is_err()
        );
        let mut restored = WorldPhysics::new(&[], 0.0);
        restored.restore_chassis(&fallen).unwrap();
        assert_eq!(restored.chassis_snapshots()[0].command.balance_control, 0);
    }

    #[test]
    fn balance_recovers_pitch_drives_and_jumps_once_per_press() {
        let config = ChassisConfig::balance();
        config.validate().unwrap();
        let mut spawn = Pose::at([0., 0., config.rest_height_m()]);
        let angle: f64 = 0.10;
        spawn.rotation_wxyz = [(angle / 2.).cos(), 0., (angle / 2.).sin(), 0.];
        let mut physics = chassis_world(config, spawn);
        let settled = run(&mut physics, 256);
        assert!(settled.pose.rotation_wxyz[2].abs() < 0.03, "{settled:?}");
        assert!(settled.pose.translation_m[2] > 0.20);
        physics
            .command_chassis(
                0,
                ChassisCommand {
                    forward_m_s: 1.0,
                    ..Default::default()
                },
            )
            .unwrap();
        let driven = run(&mut physics, 128);
        assert!(
            driven.pose.translation_m[0] > settled.pose.translation_m[0] + 0.35,
            "{driven:?}"
        );
        physics
            .command_chassis(0, ChassisCommand::default())
            .unwrap();
        let grounded = run(&mut physics, 256);
        physics
            .command_chassis(
                0,
                ChassisCommand {
                    jump: true,
                    ..Default::default()
                },
            )
            .unwrap();
        let airborne = run(&mut physics, 22);
        assert!(
            airborne.pose.translation_m[2] > grounded.pose.translation_m[2] + 0.12,
            "{airborne:?}"
        );
        assert!(airborne.jump_held);
        let mut restored = WorldPhysics::new(&[], 0.0);
        restored.restore_chassis(&airborne).unwrap();
        let replayed = run(&mut restored, 256);
        assert!((replayed.pose.translation_m[2] - grounded.pose.translation_m[2]).abs() < 0.03);

        let landed = run(&mut physics, 256);
        assert!(
            (landed.pose.translation_m[2] - grounded.pose.translation_m[2]).abs() < 0.03,
            "held jump repeated: {landed:?}"
        );
        physics
            .command_chassis(0, ChassisCommand::default())
            .unwrap();
        run(&mut physics, 1);
        physics
            .command_chassis(
                0,
                ChassisCommand {
                    jump: true,
                    ..Default::default()
                },
            )
            .unwrap();
        assert!(
            run(&mut physics, 22).pose.translation_m[2] > grounded.pose.translation_m[2] + 0.12
        );
    }

    #[test]
    fn jump_cannot_launch_omni_or_defeated_balance() {
        for (config, defeated) in [
            (ChassisConfig::default(), false),
            (ChassisConfig::balance(), true),
        ] {
            let mut physics =
                chassis_world(config.clone(), Pose::at([0., 0., config.rest_height_m()]));
            let before = run(&mut physics, 128);
            physics.set_chassis_defeated(0, defeated);
            physics
                .command_chassis(
                    0,
                    ChassisCommand {
                        jump: true,
                        ..Default::default()
                    },
                )
                .unwrap();
            let after = run(&mut physics, 22);
            assert!(after.pose.translation_m[2] < before.pose.translation_m[2] + 0.02);
        }
    }

    #[test]
    fn default_configuration_is_valid_and_bad_ones_are_rejected() {
        let config = ChassisConfig::default();
        assert!(config.validate().is_ok());
        assert!((config.rest_height_m() - 0.2065).abs() < 1e-9);
        assert!(
            ChassisConfig {
                mass_kg: 0.0,
                ..config.clone()
            }
            .validate()
            .is_err()
        );
        assert!(
            ChassisConfig {
                wheel_hubs_m: vec![[0.0, 0.0]],
                ..config.clone()
            }
            .validate()
            .is_err()
        );
        assert!(
            ChassisConfig {
                roller_friction: -0.1,
                ..config.clone()
            }
            .validate()
            .is_err()
        );
        let mut world = PhysicsWorld::new();
        assert!(
            Chassis::new(
                &mut world,
                0,
                Team::Red,
                config,
                Pose {
                    rotation_wxyz: [0.0; 4],
                    ..Pose::default()
                }
            )
            .is_err()
        );
        assert!((yaw_of(Pose::yawed([0.0; 3], 0.7)) - 0.7).abs() < 1e-12);
        assert!((wrap_angle(7.0) - (7.0 - std::f64::consts::TAU)).abs() < 1e-12);
    }
    #[test]
    fn chassis_settles_on_the_floor_with_all_wheels_loaded() {
        let config = ChassisConfig::default();
        let mut ballistics = chassis_world(
            config.clone(),
            Pose::at([0.0, 0.0, config.rest_height_m() + 0.05]),
        );
        let snapshot = run(&mut ballistics, 1_500);
        let z = snapshot.pose.translation_m[2];
        // Each spring carries a quarter of the weight.
        let sag = config.mass_kg * 9.81 / 4.0 / config.suspension_stiffness_n_m;
        assert!((z - (config.rest_height_m() - sag)).abs() < 0.001, "{z}");
        assert!(snapshot.velocity_m_s.iter().all(|v| v.abs() < 1e-3));
        assert_eq!(snapshot.wheels.len(), 4);
        for wheel in &snapshot.wheels {
            let contact = wheel.contact.expect("wheel on the ground");
            assert!((contact.load_n - config.mass_kg * 9.81 / 4.0).abs() < 1.0);
            assert!(contact.point_m[2].abs() < 1e-6);
            assert!((contact.normal[2] - 1.0).abs() < 1e-9);
        }
        assert!((yaw_of(snapshot.pose)).abs() < 1e-6);
    }
    #[test]
    fn forward_left_and_yaw_commands_move_the_body_holonomically() {
        for config in [ChassisConfig::default(), ChassisConfig::hero()] {
            let spawn = Pose::at([0.0, 0.0, config.rest_height_m()]);
            for (command, check) in [
                (
                    ChassisCommand {
                        forward_m_s: 1.0,
                        ..Default::default()
                    },
                    (|s: &ChassisSnapshot| {
                        s.pose.translation_m[0] > 1.5
                            && s.pose.translation_m[1].abs() < 0.05
                            && (s.velocity_m_s[0] - 1.0).abs() < 0.05
                            && yaw_of(s.pose).abs() < 0.05
                    }) as fn(&ChassisSnapshot) -> bool,
                ),
                (
                    ChassisCommand {
                        left_m_s: 1.0,
                        ..Default::default()
                    },
                    |s| {
                        s.pose.translation_m[1] > 1.5
                            && s.pose.translation_m[0].abs() < 0.05
                            && yaw_of(s.pose).abs() < 0.05
                    },
                ),
                (
                    ChassisCommand {
                        yaw_rate_rad_s: 1.0,
                        ..Default::default()
                    },
                    |s| {
                        s.pose.translation_m[0].abs() < 0.05
                            && s.pose.translation_m[1].abs() < 0.05
                            && (s.angular_velocity_rad_s[2] - 1.0).abs() < 0.05
                    },
                ),
            ] {
                let mut ballistics = chassis_world(config.clone(), spawn);
                run(&mut ballistics, 300);
                ballistics.command_chassis(0, command).unwrap();
                let snapshot = run(&mut ballistics, 2_000);
                assert!(check(&snapshot), "{command:?}: {snapshot:?}");
                assert_eq!(snapshot.command, command);
                // Wheels spin with the ground speed they see.
                assert!(snapshot.wheels.iter().any(|w| w.spin_rad != 0.0));
            }
        }
    }
    #[test]
    fn the_turret_rides_the_body_and_faces_the_commanded_aim() {
        let config = ChassisConfig::default();
        let mut ballistics =
            chassis_world(config.clone(), Pose::at([0.0, 0.0, config.rest_height_m()]));
        let rest = run(&mut ballistics, 300);
        let [x, y, z] = rest.turret.translation_m;
        assert!(x.abs() < 1e-6 && y.abs() < 1e-6, "{x} {y}");
        assert!((z - rest.pose.translation_m[2] - config.turret_center_m[2]).abs() < 1e-9);
        assert_eq!(rest.turret.rotation_wxyz, [1.0, 0.0, 0.0, 0.0]);
        // Aim left and up: the barrel (+x) points along +y, raised.
        ballistics
            .command_chassis(
                0,
                ChassisCommand {
                    aim_yaw_rad: std::f64::consts::FRAC_PI_2,
                    aim_pitch_rad: 0.5,
                    ..Default::default()
                },
            )
            .unwrap();
        let aimed = run(&mut ballistics, 2_000);
        let barrel = rapier_pose(aimed.turret).rotation * Vector::X;
        assert!(barrel.x.abs() < 1e-9, "{barrel}");
        assert!((barrel.y - 0.5f64.cos()).abs() < 1e-9, "{barrel}");
        assert!((barrel.z - 0.5f64.sin()).abs() < 1e-9, "{barrel}");
        assert!(
            ballistics
                .command_chassis(
                    0,
                    ChassisCommand {
                        aim_pitch_rad: f64::INFINITY,
                        ..Default::default()
                    }
                )
                .is_err()
        );
    }
    #[test]
    fn motor_limit_caps_the_top_speed() {
        let config = ChassisConfig::default();
        let mut ballistics =
            chassis_world(config.clone(), Pose::at([0.0, 0.0, config.rest_height_m()]));
        run(&mut ballistics, 300);
        ballistics
            .command_chassis(
                0,
                ChassisCommand {
                    forward_m_s: 10.0,
                    ..Default::default()
                },
            )
            .unwrap();
        let snapshot = run(&mut ballistics, 4_000);
        let speed = snapshot.velocity_m_s[0];
        // Each diagonal wheel sees forward speed / sqrt(2).
        let top = config.wheel_no_load_speed_m_s * std::f64::consts::SQRT_2;
        assert!(speed > 0.8 * top && speed < top, "{speed}");
        assert!(
            ballistics
                .command_chassis(
                    0,
                    ChassisCommand {
                        forward_m_s: f64::NAN,
                        ..Default::default()
                    }
                )
                .is_err()
        );
    }
}

#[cfg(test)]
mod team_cad_tests {
    use super::*;
    use crate::projectile::{TargetFrames, WorldPhysics};
    #[test]
    fn source_mounts_settle_and_muzzle_follows_the_fitted_barrel() {
        let cfg = ChassisConfig::team_cad();
        cfg.validate().unwrap();
        let faces = cfg.armor_faces();
        assert!((faces[1].translation_m[2] - faces[0].translation_m[2] - 0.047).abs() < 1e-12);
        let mut world = WorldPhysics::new(&[], 0.);
        let id = world
            .add_chassis(
                Team::Red,
                cfg.clone(),
                Pose::at([0., 0., cfg.rest_height_m()]),
            )
            .unwrap();
        let frames = TargetFrames::new(Vec::new());
        for tick in 0..512 {
            world.step(tick * crate::tick_ns(), &frames).unwrap();
        }
        let c = &world.chassis_snapshots()[0];
        assert!((c.pose.translation_m[2] - 0.175_675_827).abs() < 0.003);
        assert!(c.wheels.iter().all(|w| w.contact.is_some()));
        let muzzle = world.chassis_muzzle_pose(id).unwrap();
        assert!((muzzle.translation_m[0] - 0.206_576_73).abs() < 0.002);
        assert!((muzzle.translation_m[2] - 0.461_125_827).abs() < 0.003);
        assert!(
            c.wheels
                .iter()
                .all(|w| w.angular_velocity_rad_s.abs() < 0.01)
        );
    }
}

#[cfg(test)]
mod folding_tests {
    use super::*;
    use crate::{TargetFrames, WorldPhysics, tick_ns};
    fn advance(w: &mut WorldPhysics, tick: &mut u64, count: u64) {
        for _ in 0..count {
            w.step(*tick * tick_ns(), &TargetFrames::new(vec![]))
                .unwrap();
            *tick += 1;
        }
    }
    #[test]
    fn half_second_strokes_reverse_and_resume_without_teleporting() {
        for mut config in [ChassisConfig::folding_hero(), ChassisConfig::stair_heavy()] {
            config.set_transform_duration_s(0.5).unwrap();
            let limit = config.transform_limit_rad();
            let mut world = WorldPhysics::new(&[], 0.);
            let id = world
                .add_chassis(Team::Red, config, Pose::at([0., 0., 20.]))
                .unwrap();
            let mut tick = 0;
            world
                .command_chassis(
                    id,
                    ChassisCommand {
                        folded: true,
                        ..Default::default()
                    },
                )
                .unwrap();
            advance(&mut world, &mut tick, 32);
            assert!(
                (world.chassis_snapshot(id).unwrap().fold_angle_rad - limit / 2.).abs() < 1e-12
            );
            world
                .command_chassis(id, ChassisCommand::default())
                .unwrap();
            advance(&mut world, &mut tick, 32);
            assert!(world.chassis_snapshot(id).unwrap().fold_angle_rad.abs() < 1e-12);
            world
                .command_chassis(
                    id,
                    ChassisCommand {
                        folded: true,
                        ..Default::default()
                    },
                )
                .unwrap();
            advance(&mut world, &mut tick, 63);
            assert!(world.chassis_snapshot(id).unwrap().fold_angle_rad < limit - 0.001);
            advance(&mut world, &mut tick, 1);
            assert!((world.chassis_snapshot(id).unwrap().fold_angle_rad - limit).abs() < 1e-12);
            world
                .command_chassis(id, ChassisCommand::default())
                .unwrap();
            advance(&mut world, &mut tick, 64);
            assert!(world.chassis_snapshot(id).unwrap().fold_angle_rad.abs() < 1e-12);
        }
        let mut config = ChassisConfig::folding_hero();
        let before = config.clone();
        for invalid in [0., -1., f64::NAN, f64::INFINITY, f64::MIN_POSITIVE / 16.] {
            assert!(config.set_transform_duration_s(invalid).is_err());
            assert_eq!(config, before);
        }
    }
    fn setup(height: f64) -> (WorldPhysics, u32) {
        let mut w = WorldPhysics::new(&[], 0.);
        let id = w
            .add_chassis(
                Team::Red,
                ChassisConfig::folding_hero(),
                Pose::at([0., 0., height]),
            )
            .unwrap();
        (w, id)
    }
    fn height(w: &WorldPhysics) -> f64 {
        w.dynamic_geometry_snapshot()
            .shapes
            .iter()
            .map(|(p, s)| s.compute_aabb(p).maxs.z)
            .fold(f64::NEG_INFINITY, f64::max)
    }
    #[test]
    fn folding_changes_physical_envelope_and_stops_unfolding_at_a_roof() {
        let (mut w, id) = setup(0.17);
        let mut tick = 0;
        advance(&mut w, &mut tick, 512);
        let before = height(&w);
        assert!(before > 0.40);
        w.command_chassis(
            id,
            ChassisCommand {
                folded: true,
                ..Default::default()
            },
        )
        .unwrap();
        advance(&mut w, &mut tick, 256);
        let folded = w.chassis_snapshot(id).unwrap();
        assert!(
            (folded.fold_angle_rad - folded.config.folding.as_ref().unwrap().folded_angle_rad)
                .abs()
                < 1e-12
        );
        assert!(height(&w) < 0.25, "{}", height(&w));
        let muzzle = w.chassis_muzzle_pose(id).unwrap();
        assert!((muzzle.translation_m[2] - folded.turret.translation_m[2]).abs() < 0.001);
        w.add_static_box(Pose::at([0., 0., 0.5]), [2., 2., 0.5]);
        w.command_chassis(id, ChassisCommand::default()).unwrap();
        advance(&mut w, &mut tick, 256);
        let blocked = w.chassis_snapshot(id).unwrap();
        assert!(
            blocked.fold_angle_rad > 1. && blocked.fold_angle_rad < folded.fold_angle_rad,
            "blocked={} folded={} height={} pos={:?}",
            blocked.fold_angle_rad,
            folded.fold_angle_rad,
            height(&w),
            blocked.pose
        );
        assert!(height(&w) < 0.251, "{}", height(&w));
        w.command_chassis(
            id,
            ChassisCommand {
                folded: true,
                ..Default::default()
            },
        )
        .unwrap();
        advance(&mut w, &mut tick, 256);
        assert!(
            (w.chassis_snapshot(id).unwrap().fold_angle_rad - folded.fold_angle_rad).abs() < 1e-12
        );
    }
    #[test]
    fn mid_transform_restores_and_replays_at_the_same_rate() {
        let (mut host, id) = setup(3.);
        let mut tick = 0;
        host.command_chassis(
            id,
            ChassisCommand {
                folded: true,
                ..Default::default()
            },
        )
        .unwrap();
        advance(&mut host, &mut tick, 40);
        let saved = host.chassis_snapshot(id).unwrap();
        assert!(
            (saved.fold_angle_rad
                - saved.config.folding.as_ref().unwrap().speed_rad_s * 40. / 128.)
                .abs()
                < 1e-12,
            "angle={} command={:?}",
            saved.fold_angle_rad,
            saved.command
        );
        let mut restored = WorldPhysics::new(&[], 0.);
        restored.restore_chassis(&saved).unwrap();
        assert_eq!(
            restored.chassis_snapshot(id).unwrap().fold_angle_rad,
            saved.fold_angle_rad
        );
        let mut replay_tick = tick;
        advance(&mut host, &mut tick, 30);
        advance(&mut restored, &mut replay_tick, 30);
        assert_eq!(
            host.chassis_snapshot(id).unwrap().fold_angle_rad,
            restored.chassis_snapshot(id).unwrap().fold_angle_rad
        );
        let a = host.chassis_muzzle_pose(id).unwrap();
        let b = restored.chassis_muzzle_pose(id).unwrap();
        for i in 0..3 {
            assert!((a.translation_m[i] - b.translation_m[i]).abs() < 1e-8);
        }
    }
}

#[cfg(test)]
mod climbing_tests {
    use super::*;
    use crate::{TargetFrames, WorldPhysics, tick_ns};

    #[test]
    fn closed_linkage_keeps_lengths_and_level_travel() {
        let config = ChassisConfig::stair_heavy();
        config.validate().unwrap();
        let c = config.climbing.as_ref().unwrap();
        for i in 0..=128 {
            let p = c.poses(c.travel_rad() * i as f64 / 128.);
            let distance = |a: Pose, b: Pose| {
                (Vector::from_array(a.translation_m) - Vector::from_array(b.translation_m)).length()
            };
            assert!((distance(p.front, p.coupler) - c.front_pin_length_m).abs() < 1e-10);
            assert!((distance(p.coupler, p.rear) - c.coupler_length_m).abs() < 1e-10);
            assert!((p.rear.translation_m[2] - c.slider_height_m).abs() < 1e-10);
            for hub in p.hubs_m {
                assert!(hub.into_iter().all(f64::is_finite));
            }
        }
        let p = c.poses(0.);
        assert!((p.hubs_m[0][2] - p.hubs_m[2][2]).abs() < 1e-10);
        let mut invalid = c.clone();
        invalid.coupler_length_m = 0.001;
        assert!(invalid.validate().is_err());
        let mut invalid = config.clone();
        invalid.folding = ChassisConfig::folding_hero().folding;
        assert!(invalid.validate().is_err());
    }

    #[test]
    fn moving_hubs_and_actuator_restore_mid_climb() {
        let cfg = ChassisConfig::stair_heavy();
        let mut host = WorldPhysics::new(&[], 0.);
        let id = host
            .add_chassis(Team::Red, cfg, Pose::at([0., 0., 3.]))
            .unwrap();
        host.command_chassis(
            id,
            ChassisCommand {
                folded: true,
                ..Default::default()
            },
        )
        .unwrap();
        let frames = TargetFrames::new(vec![]);
        for tick in 0..40 {
            host.step(tick * tick_ns(), &frames).unwrap();
        }
        let saved = host.chassis_snapshot(id).unwrap();
        assert!(
            saved.fold_angle_rad > 0.1 && saved.fold_angle_rad < saved.config.transform_limit_rad()
        );
        let mut restored = WorldPhysics::new(&[], 0.);
        restored.restore_chassis(&saved).unwrap();
        let copy = restored.chassis_snapshot(id).unwrap();
        assert_eq!(copy.fold_angle_rad, saved.fold_angle_rad);
        for (a, b) in saved.wheels.iter().zip(&copy.wheels) {
            for (x, y) in a.hub_m.iter().zip(b.hub_m) {
                assert!((x - y).abs() < 1e-9);
            }
        }
        for tick in 40..70 {
            host.step(tick * tick_ns(), &frames).unwrap();
            restored.step(tick * tick_ns(), &frames).unwrap();
        }
        let a = host.chassis_snapshot(id).unwrap();
        let b = restored.chassis_snapshot(id).unwrap();
        assert_eq!(a.fold_angle_rad, b.fold_angle_rad);
        for (x, y) in a.pose.translation_m.into_iter().zip(b.pose.translation_m) {
            assert!((x - y).abs() < 1e-8);
        }
    }
    #[test]
    fn defeated_stair_actuator_stops_at_its_current_input() {
        let mut w = WorldPhysics::new(&[], 0.);
        let id = w
            .add_chassis(
                Team::Red,
                ChassisConfig::stair_heavy(),
                Pose::at([0., 0., 3.]),
            )
            .unwrap();
        w.command_chassis(
            id,
            ChassisCommand {
                folded: true,
                ..Default::default()
            },
        )
        .unwrap();
        let frames = TargetFrames::new(vec![]);
        for tick in 0..40 {
            w.step(tick * tick_ns(), &frames).unwrap();
        }
        let angle = w.chassis_snapshot(id).unwrap().fold_angle_rad;
        w.set_chassis_defeated(id, true);
        for tick in 40..80 {
            w.step(tick * tick_ns(), &frames).unwrap();
        }
        assert_eq!(w.chassis_snapshot(id).unwrap().fold_angle_rad, angle);
    }
}
