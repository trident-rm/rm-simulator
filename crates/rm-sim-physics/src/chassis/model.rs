// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Immutable physical descriptions, presets and evaluated body-local model geometry.
//! No solver bodies, handles, queries or time advancement live here.

use super::{ARMOR_COUNT, MUZZLE_FORWARD_M, rapier_pose, vector};
use crate::{Pose, projectile::SMALL_ARMOR_HOUSING_HALF_M, wheel_model::WheelModel};
use rapier3d_f64::prelude::{Rotation, Vector};
use serde::{Deserialize, Serialize};

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
/// use rm_sim_physics::chassis::Tether;
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
/// use rm_sim_physics::chassis::ChassisConfig;
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
    /// use rm_sim_physics::chassis::ChassisConfig;
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
    /// use rm_sim_physics::chassis::ChassisConfig;
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
    /// use rm_sim_physics::chassis::ChassisConfig;
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
    /// use rm_sim_physics::chassis::ChassisConfig;
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
    /// use rm_sim_physics::chassis::ChassisConfig;
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
    /// use rm_sim_physics::chassis::ChassisConfig;
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
    /// use rm_sim_physics::chassis::ChassisConfig;
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
    /// use rm_sim_physics::chassis::ChassisConfig;
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
    /// use rm_sim_physics::chassis::ChassisConfig;
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
    /// use rm_sim_physics::{chassis::ChassisConfig, motion::outpost::rotate};
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
    /// use rm_sim_physics::chassis::ChassisConfig;
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
    /// use rm_sim_physics::chassis::ChassisConfig;
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

impl From<&ChassisConfig> for WheelModel {
    fn from(config: &ChassisConfig) -> Self {
        Self {
            wheel_radius_m: config.wheel_radius_m,
            wheel_inertia_kg_m2: config.wheel_inertia_kg_m2,
            slip_stiffness_n_s_m: config.slip_stiffness_n_s_m,
            wheel_stall_force_n: config.wheel_stall_force_n,
            wheel_no_load_speed_m_s: config.wheel_no_load_speed_m_s,
            motor_stall_loss_w: config.dynamics.motor_stall_loss_w,
            suspension_travel_m: config.dynamics.suspension_travel_m,
            bump_stop_stiffness_n_m: config.dynamics.bump_stop_stiffness_n_m,
            suspension_stiffness_n_m: config.suspension_stiffness_n_m,
            suspension_damping_n_s_m: config.suspension_damping_n_s_m,
            drive_friction: config.drive_friction,
            roller_friction: config.roller_friction,
            mecanum: config.mecanum,
        }
    }
}

/// Body-frame wheel geometry in `ChassisConfig::wheel_hubs_m` order.
pub(super) fn wheel_geometry(config: &ChassisConfig) -> Vec<WheelGeometry> {
    wheel_geometry_at(config, 0.)
}
pub(super) fn wheel_geometry_at(config: &ChassisConfig, angle_rad: f64) -> Vec<WheelGeometry> {
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
pub(super) struct WheelGeometry {
    pub(super) hub_m: Vector,
    pub(super) velocity_m_s: Vector,
    /// Unit rolling direction in the body's horizontal plane.
    pub(super) roll: Vector,
    /// Free roller direction in the contact plane. Radial for omni wheels.
    pub(super) axle: Vector,
    /// Signed contact speed per unit yaw rate, from the hub cross rolling direction.
    pub(super) lever_m: f64,
}
