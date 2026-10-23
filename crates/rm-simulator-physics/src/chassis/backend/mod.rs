// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Rapier chassis adapter: solver ownership, controls and observation access.
//! Scalar tyre and motor equations remain in `crate::wheel_model`.

use super::*;
use crate::{
    Pose,
    projectile::FRICTION,
    wheel_model::{WheelDriveScratch, WheelModel, WheelState},
};
use rapier3d_f64::prelude::*;

mod construction;
mod dynamics;
mod preparation;
mod prescribed;

pub use preparation::PreparedChassis;

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

impl ChassisTelemetry<'_> {
    /// Derive compact wheel values in configured hub order, without allocating.
    /// Contact values are the wheel model's stored ray observations, not solver state.
    pub fn wheels(&self) -> impl ExactSizeIterator<Item = WheelSnapshot> + '_ {
        let pose = *self.world.bodies[self.chassis.body].position();
        self.chassis
            .wheels
            .iter()
            .zip(&self.chassis.geometry)
            .map(move |(wheel, geometry)| WheelSnapshot {
                hub_m: pose
                    .transform_point(
                        geometry.hub_m
                            + Vector::Z
                                * (wheel.suspension_compression_m
                                    - self.chassis.config.suspension_rest_m),
                    )
                    .to_array(),
                spin_rad: wheel.spin_rad,
                angular_velocity_rad_s: wheel.angular_velocity_rad_s,
                suspension_compression_m: wheel.suspension_compression_m,
                target_m_s: wheel.target_m_s,
                contact: wheel.contact,
            })
    }

    /// Replace caller wheel output, retaining its allocation when capacity suffices.
    pub fn wheels_into(&self, out: &mut Vec<WheelSnapshot>) {
        out.clear();
        out.extend(self.wheels());
    }
}

impl Chassis {
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
    pub(crate) fn telemetry<'a>(&'a self, world: &'a PhysicsWorld) -> ChassisTelemetry<'a> {
        let body = &world.bodies[self.body];
        ChassisTelemetry {
            id: self.id,
            team: self.team,
            pose: pose_of(body),
            turret: self.turret_pose(world),
            velocity_m_s: body.linvel().to_array(),
            angular_velocity_rad_s: body.angvel().to_array(),
            held_aim_rad: self.aim_rad,
            gimbal_velocity_rad_s: self.gimbal_velocity_rad_s,
            fold_angle_rad: self.fold_angle_rad,
            command: self.command,
            drive_control: &self.drive_control,
            placement_revision: self.placement_revision,
            defeated: self.defeated,
            drive_power_w: self.drive_power_w,
            chassis: self,
            world,
        }
    }
    /// Read the body, wheel and gimbal state into a serializable snapshot.
    pub(crate) fn snapshot(&self, world: &PhysicsWorld) -> ChassisSnapshot {
        let truth = self.telemetry(world);
        ChassisSnapshot {
            fold_angle_rad: truth.fold_angle_rad,
            placement_revision: truth.placement_revision,
            id: truth.id,
            team: truth.team,
            config: self.config.clone(),
            pose: truth.pose,
            turret: truth.turret,
            velocity_m_s: truth.velocity_m_s,
            angular_velocity_rad_s: truth.angular_velocity_rad_s,
            command: truth.command,
            drive_control: truth.drive_control.clone(),
            jump_held: self.jump_held,
            held_aim_rad: truth.held_aim_rad,
            gimbal_velocity_rad_s: truth.gimbal_velocity_rad_s,
            wheels: truth.wheels().collect(),
            defeated: truth.defeated,
        }
    }
}

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

#[cfg(test)]
mod preparation_tests;

#[cfg(test)]
mod reuse_tests;
