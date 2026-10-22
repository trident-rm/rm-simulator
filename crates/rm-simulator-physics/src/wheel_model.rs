// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Backend-neutral ray-wheel equations. The adapter supplies contact kinematics,
//! admitted controls and power, retains wheel state, and applies returned forces
//! before stepping its solver. No queries, handles, clock or solver are used here.
//!
//! ```
//! use rm_simulator_physics::{chassis::ChassisConfig,
//!     wheel_model::{WheelModel, WheelState, WheelDriveScratch}};
//! let model = WheelModel::from(&ChassisConfig::default());
//! let mut wheels = [WheelState { target_m_s: 2., ..Default::default() }; 4];
//! let mut scratch = WheelDriveScratch::default();
//! // No ground contact: motors spin the wheels without applying body traction.
//! scratch.contacts.resize(4, (0., 0.));
//! let (_, power_w) = scratch.integrate(&model, &mut wheels, 0.001, 40., None);
//! assert!(power_w <= 40. + 1e-10);
//! assert!(wheels[0].angular_velocity_rad_s > 0.);
//! assert_eq!(scratch.forces, [0.; 4]);
//! ```
use serde::{Deserialize, Serialize};

/// Assumed wheel/motor/suspension tuning, copied from a validated chassis profile.
/// These are the existing fitted model parameters, not rulebook limits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelModel {
    /// Wheel radius in metres.
    pub wheel_radius_m: f64,
    /// Lumped wheel-axis inertia in kg m².
    pub wheel_inertia_kg_m2: f64,
    /// Tread and roller slip stiffness in N s/m.
    pub slip_stiffness_n_s_m: f64,
    /// Maximum motor force at zero tread speed in N.
    pub wheel_stall_force_n: f64,
    /// Motor no-load tread speed in m/s.
    pub wheel_no_load_speed_m_s: f64,
    /// Copper loss per motor at stall force in W.
    pub motor_stall_loss_w: f64,
    /// Compression before bump stop engagement in metres.
    pub suspension_travel_m: f64,
    /// Bump stop spring stiffness in N/m.
    pub bump_stop_stiffness_n_m: f64,
    /// Suspension spring stiffness in N/m.
    pub suspension_stiffness_n_m: f64,
    /// Suspension damping in N s/m.
    pub suspension_damping_n_s_m: f64,
    /// Dimensionless rolling grip coefficient.
    pub drive_friction: f64,
    /// Dimensionless free-roller grip coefficient.
    pub roller_friction: f64,
    /// Mecanum rolling-force projection (radius divided by sqrt(2)).
    pub mecanum: bool,
}

impl WheelModel {
    /// Suspension load in N from compression in m and compression rate in m/s.
    /// Negative total load clamps to zero: a ray spring cannot pull on the ground.
    pub fn suspension_load_n(&self, compression_m: f64, compression_rate_m_s: f64) -> f64 {
        let overtravel = (compression_m - self.suspension_travel_m).max(0.0);
        let bump_stop = self.bump_stop_stiffness_n_m
            * overtravel
            * (1.0 + overtravel / self.suspension_travel_m);
        (bump_stop
            + self.suspension_stiffness_n_m * compression_m
            + self.suspension_damping_n_s_m * compression_rate_m_s)
            .max(0.0)
    }

    /// Free-roller drag in N, limited by its share of suspension load in N.
    pub fn roller_force_n(&self, roller_speed_m_s: f64, load_n: f64) -> f64 {
        let roller_grip = self.roller_friction * load_n;
        (-self.slip_stiffness_n_s_m * roller_speed_m_s).clamp(-roller_grip, roller_grip)
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
/// Persistent wheel state plus the adapter's current contact and target.
/// No solver state or handles; existing chassis snapshots remain the restore format.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WheelState {
    /// Rolling angle in radians, wrapped to [-pi, pi).
    pub spin_rad: f64,
    /// Wheel-axis angular velocity in radians per second.
    pub angular_velocity_rad_s: f64,
    /// Compression from full droop in metres; zero without contact.
    pub suspension_compression_m: f64,
    /// Held velocity-controller tread target in metres per second.
    pub target_m_s: f64,
    /// Adapter-owned ground contact, or None when unsupported.
    pub contact: Option<WheelContact>,
}

/// Reusable per-wheel model storage; none of it is physical restore state.
/// Contact and output vectors and private motor storage retain capacity across
/// calls. The first call or a new largest wheel count can grow storage; a fixed
/// topology allocates nothing here after warmup. No per-wheel objects are allocated.
#[derive(Default)]
pub struct WheelDriveScratch {
    /// One (rolling ground speed in m/s, available grip in N) per wheel.
    /// The adapter replaces these each solver slice; (0, 0) denotes no contact.
    pub contacts: Vec<(f64, f64)>,
    /// Mean rolling traction in N, in wheel order, replaced by each integration.
    pub forces: Vec<f64>,
    motors: Vec<f64>,
}
impl WheelDriveScratch {
    /// Integrate motor torque and equal/opposite tyre reaction, with shared power.
    /// Eight internal slices per backend slice resolve the stiff slip law without
    /// changing the caller-selected integration duration. Returns (mean motor
    /// allocation scale, mean consumed power in W); braking earns no power credit.
    /// Inputs must be finite, dt_s positive, budget_w nonnegative, and model
    /// parameters physically valid (positive radii/inertia/stall force/speed).
    /// Contacts and optional axis torques in N m must have one entry per wheel.
    /// `forces` is resized to that count. Ground grip must be nonnegative.
    /// `None` selects velocity tracking; `Some` bypasses that controller only.
    /// The adapter admits defeat/actuation/power limits before calling, resets
    /// unsupported contacts, and uses the returned allocation scale for assistance.
    /// Panics on mismatched lengths. Storage is reused after capacity warmup.
    pub fn integrate(
        &mut self,
        config: &WheelModel,
        wheels: &mut [WheelState],
        dt_s: f64,
        budget_w: f64,
        torques: Option<&[f64]>,
    ) -> (f64, f64) {
        assert_eq!(self.contacts.len(), wheels.len());
        if let Some(torques) = torques {
            assert_eq!(torques.len(), wheels.len());
        }
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
                    |torques| torques[index] / ratio,
                );
                let limit = config.wheel_stall_force_n
                    * if requested_force * speed > 0.0 {
                        (1.0 - speed.abs() / config.wheel_no_load_speed_m_s).clamp(0.0, 1.0)
                    } else {
                        1.0
                    };
                *motor = requested_force.clamp(-limit, limit);
                mechanical_w += (*motor * speed).max(0.0);
                copper_w +=
                    config.motor_stall_loss_w * (*motor / config.wheel_stall_force_n).powi(2);
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

pub(crate) fn drive_power_scale(mechanical_w: f64, copper_w: f64, budget_w: f64) -> f64 {
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

/// Wrap radians with the original remainder rounding, skipping identity remainders.
fn wrap_angle(angle: f64) -> f64 {
    use std::f64::consts::{PI, TAU};
    let shifted = angle + PI;
    // Most integration slices remain within one turn. Keep the original
    // add/subtract rounding, but skip the remainder when it is the identity.
    let wrapped = if (0.0..TAU).contains(&shifted) {
        shifted
    } else {
        shifted.rem_euclid(TAU)
    };
    wrapped - PI
}

#[cfg(test)]
mod tests {
    use super::*;

    // An explicit model, not a ChassisConfig or a Rapier world.
    fn model() -> WheelModel {
        WheelModel {
            wheel_radius_m: 0.1,
            wheel_inertia_kg_m2: 0.01,
            slip_stiffness_n_s_m: 100.,
            wheel_stall_force_n: 20.,
            wheel_no_load_speed_m_s: 10.,
            motor_stall_loss_w: 5.,
            suspension_travel_m: 0.05,
            bump_stop_stiffness_n_m: 1000.,
            suspension_stiffness_n_m: 1000.,
            suspension_damping_n_s_m: 20.,
            drive_friction: 0.5,
            roller_friction: 0.1,
            mecanum: false,
        }
    }

    #[test]
    fn angle_fast_path_matches_remainder_bits_at_boundaries_and_large_inputs() {
        use std::f64::consts::{PI, TAU};
        let check = |angle: f64| {
            let expected = (angle + PI).rem_euclid(TAU) - PI;
            let actual = wrap_angle(angle);
            if expected.is_nan() {
                assert!(actual.is_nan());
            } else {
                assert_eq!(actual.to_bits(), expected.to_bits(), "angle={angle:?}");
            }
        };
        for turns in -32..=32 {
            for boundary in [-PI, -0.0, 0.0, PI] {
                let angle = f64::from(turns) * TAU + boundary;
                check(angle.next_down());
                check(angle);
                check(angle.next_up());
            }
        }
        // Exercise normal slice increments as well as arbitrary finite values,
        // infinities, subnormals and NaNs without depending on a random library.
        let mut bits = 0x1234_5678_9abc_def0_u64;
        for i in 0..65_536 {
            bits ^= bits << 13;
            bits ^= bits >> 7;
            bits ^= bits << 17;
            check(f64::from_bits(bits));
            check((f64::from(i) - 32_768.) * PI / 32_768.);
        }
        for angle in [
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NAN,
            f64::MAX,
            f64::MIN,
        ] {
            check(angle);
        }
    }

    #[test]
    fn suspension_damping_bump_stop_and_roller_limits_need_no_solver() {
        let model = model();
        assert_eq!(model.suspension_load_n(0.02, 0.5), 30.);
        assert_eq!(model.suspension_load_n(0.02, -2.), 0.);
        assert!((model.suspension_load_n(0.07, -0.2) - 94.).abs() < 1e-12);
        assert_eq!(model.roller_force_n(2., 30.), -3.);
        assert_eq!(model.roller_force_n(-2., 30.), 3.);
        assert_eq!(model.roller_force_n(0.01, 30.), -1.);
        assert_eq!(model.roller_force_n(2., 0.), 0.);
    }

    #[test]
    fn airborne_axis_torque_integrates_inertia_and_eight_spin_slices() {
        for mecanum in [false, true] {
            let model = WheelModel { mecanum, ..model() };
            let mut wheels = [WheelState::default()];
            let mut scratch = WheelDriveScratch::default();
            scratch.contacts.push((0., 0.));
            let (scale, power) = scratch.integrate(&model, &mut wheels, 0.008, 100., Some(&[0.01]));
            assert_eq!(scale, 1.);
            assert!(power > 0.);
            assert!((wheels[0].angular_velocity_rad_s - 0.008).abs() < 1e-14);
            assert!((wheels[0].spin_rad - 0.000036).abs() < 1e-14);
            assert_eq!(scratch.forces, [0.]);
            assert_eq!(wheels[0].contact, None);
        }
    }

    #[test]
    fn unpowered_slipping_wheel_still_transfers_reaction_to_ground() {
        let model = model();
        let mut wheels = [WheelState {
            angular_velocity_rad_s: 20.,
            contact: Some(WheelContact {
                point_m: [0.; 3],
                normal: [0., 0., 1.],
                load_n: 2.,
                slip_m_s: 0.,
                drive_force_n: 0.,
            }),
            ..Default::default()
        }];
        let mut scratch = WheelDriveScratch::default();
        scratch.contacts.push((0., model.drive_friction * 2.));
        assert_eq!(
            scratch.integrate(&model, &mut wheels, 0.008, 0., Some(&[10.])),
            (0., 0.)
        );
        assert_eq!(scratch.forces, [1.]);
        assert!((wheels[0].angular_velocity_rad_s - 19.92).abs() < 1e-12);
        assert!((wheels[0].contact.unwrap().slip_m_s - 1.992).abs() < 1e-12);
    }

    #[test]
    fn shared_power_and_zero_budget_reuse_all_buffers_after_warmup() {
        let model = model();
        let mut scratch = WheelDriveScratch::default();
        let mut storage = None;
        for slice in 0..256 {
            // Smaller active slices must not discard the largest retained buffers.
            let count = if slice % 2 == 0 { 4 } else { 2 };
            let mut wheels = [WheelState {
                target_m_s: 3.,
                ..Default::default()
            }; 4];
            scratch.contacts.clear();
            scratch.contacts.resize(count, (0., 0.));
            let budget = if slice % 3 == 0 { 0. } else { 1. };
            let torques = [1., 2., 3., 4.];
            let direct = (slice % 5 == 0).then_some(&torques[..count]);
            let (scale, power) =
                scratch.integrate(&model, &mut wheels[..count], 0.001, budget, direct);
            assert!(power <= budget + 1e-12);
            assert_eq!(scratch.forces, vec![0.; count]);
            if budget == 0. {
                assert_eq!((scale, power), (0., 0.));
                assert!(
                    wheels[..count]
                        .iter()
                        .all(|w| w.angular_velocity_rad_s == 0.)
                );
            } else {
                assert!(scale > 0. && scale < 1.);
                assert!(power > 0.);
                assert!(
                    wheels[..count]
                        .iter()
                        .all(|w| w.angular_velocity_rad_s > 0.)
                );
                if direct.is_none() {
                    assert_eq!(wheels[0], wheels[1]);
                }
            }
            let pointers = (
                scratch.contacts.as_ptr(),
                scratch.forces.as_ptr(),
                scratch.motors.as_ptr(),
            );
            if let Some(previous) = storage {
                assert_eq!(pointers, previous);
            }
            storage = Some(pointers);
        }
    }

    #[test]
    #[should_panic(expected = "assertion `left == right` failed")]
    fn mismatched_contact_count_cannot_silently_skip_wheels() {
        WheelDriveScratch::default().integrate(
            &model(),
            &mut [WheelState::default()],
            0.001,
            1.,
            None,
        );
    }
}
