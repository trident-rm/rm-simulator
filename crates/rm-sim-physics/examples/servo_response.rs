// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Torque-controlled centered rotor: illustrative inertia, no electrical model.
use rm_sim_physics::{
    StepDuration,
    cooking::CookedRigidBody,
    runtime::{BodyBinding, BodyKey, JointBinding, JointKey, PhysicalRuntime},
};
use sim_engine::{
    Pose,
    model::{
        BodyKind, BodyState, Inertial, JointDescription, JointKind, JointLimits, JointMotor,
        RigidBodyDescription,
    },
};

/// Explicit rotor inertia about every principal axis, kg m² (fixture choice).
pub const INERTIA_KG_M2: f64 = 0.02;

/// A centered rotor hinged about world +X to a fixed support, without contacts.
/// Gravity passes through the anchor, so there is no gravity moment.
pub struct Fixture {
    /// Caller-owned physical plant; torque and external loads use normal APIs.
    pub runtime: PhysicalRuntime,
    /// Rotor for external wrench commands.
    pub rotor: BodyBinding,
    /// Hinge for torque/servo commands and scalar observations.
    pub joint: JointBinding,
    time_ns: u64,
}
impl Fixture {
    /// Construct at a supplied rate, optionally with symmetric ±0.5 rad stops.
    pub fn new(duration: StepDuration, stops: bool) -> Result<Self, &'static str> {
        let mut runtime = PhysicalRuntime::new(duration);
        let fixed = CookedRigidBody::new(RigidBodyDescription::new(BodyKind::Fixed, vec![])?)?;
        let rotor = CookedRigidBody::new(RigidBodyDescription::new(
            BodyKind::Dynamic(Inertial::new(5., [INERTIA_KG_M2; 3], Pose::default())?),
            vec![],
        )?)?;
        let parent = runtime.install_body(BodyKey(1), &fixed, BodyState::at(Pose::default()))?;
        let rotor = runtime.install_body(BodyKey(2), &rotor, BodyState::at(Pose::default()))?;
        let limits = if stops {
            Some(JointLimits::new(-0.5, 0.5)?)
        } else {
            None
        };
        let joint = runtime.install_joint(
            JointKey(1),
            parent,
            rotor,
            &JointDescription::new(
                JointKind::Revolute,
                Pose::default(),
                Pose::default(),
                limits,
                false,
            )?,
        )?;
        Ok(Self {
            runtime,
            rotor,
            joint,
            time_ns: 0,
        })
    }
    /// Advance exactly one caller tick, holding the latest command.
    pub fn step(&mut self) -> Result<(), &'static str> {
        self.runtime.step(self.time_ns)?;
        self.time_ns += self.runtime.step_duration().as_ns();
        Ok(())
    }
    /// Elapsed simulation time in seconds.
    pub fn time_s(&self) -> f64 {
        self.time_ns as f64 * 1e-9
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use std::io::Write;
    let path = std::env::args()
        .nth(1)
        .ok_or("usage: servo_response OUTPUT.csv [128|1000]")?;
    let hz: u64 = std::env::args()
        .nth(2)
        .unwrap_or_else(|| "1000".into())
        .parse()?;
    let duration = match hz {
        128 => StepDuration::HZ_128,
        1000 => StepDuration::HZ_1000,
        _ => return Err("expected 128 or 1000 Hz".into()),
    };
    let mut csv = std::io::BufWriter::new(std::fs::File::create(path)?);
    writeln!(
        csv,
        "time_s,target_rad,position_rad,velocity_rad_s,applied_torque_nm,external_torque_nm"
    )?;
    let mut fixture = Fixture::new(duration, true)?;
    for tick in 0..3 * hz {
        let target = if tick < 3 * hz / 2 { 0.3 } else { -0.2 };
        let load = if (hz..2 * hz).contains(&tick) {
            -0.2
        } else {
            0.
        };
        fixture.runtime.set_joint_motor(
            fixture.joint,
            Some(JointMotor::new(target, 0., 4., 0.4, 0.5)?),
        )?;
        fixture
            .runtime
            .set_body_wrench(fixture.rotor, [0.; 3], [load, 0., 0.])?;
        fixture.step()?;
        let state = fixture.runtime.joint_state(fixture.joint)?;
        writeln!(
            csv,
            "{},{target},{},{},{},{load}",
            fixture.time_s(),
            state.position,
            state.velocity,
            fixture.runtime.joint_applied_effort(fixture.joint)?
        )?;
    }
    csv.flush()?;
    Ok(())
}
