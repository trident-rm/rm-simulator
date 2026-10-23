// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-owned physical scene; no renderer, match, transport or host clock.
use rm_sim_engine::{
    Pose,
    model::{
        BodyKind, BodyState, ColliderDescription, ContactMaterial, Inertial, JointDescription,
        JointKind, JointLimits, JointMotor, RigidBodyDescription, ShapeDescription,
    },
};
use rm_sim_physics::{
    StepDuration,
    cooking::CookedRigidBody,
    runtime::{BodyKey, JointKey, PhysicalRuntime},
};

/// Stable viewer key for the falling cube.
pub const CUBE: BodyKey = BodyKey(1);
/// Stable viewer key for the fixed pedestal.
pub const BASE: BodyKey = BodyKey(2);
/// Stable viewer key for the dynamic arm.
pub const ARM: BodyKey = BodyKey(3);
/// Physical bodies in visual installation order.
pub const KEYS: [BodyKey; 3] = [CUBE, BASE, ARM];
/// Illustrative body half extents in FLU metres, matching `KEYS`.
pub const HALF_EXTENTS_M: [[f64; 3]; 3] = [[0.2; 3], [0.3, 0.3, 0.35], [0.6, 0.1, 0.1]];
const YAW: JointKey = JointKey(1);
const INITIAL: [[f64; 3]; 3] = [[-0.8, 0., 1.5], [0.7, 0., 0.35], [0.7, 0., 0.85]];
fn material() -> ContactMaterial {
    ContactMaterial {
        friction: 0.5,
        restitution: 0.15,
        memberships: u32::MAX,
        filter: u32::MAX,
    }
}
fn model(half: [f64; 3], mass: Option<f64>) -> Result<CookedRigidBody, &'static str> {
    let kind = if let Some(mass) = mass {
        let [x, y, z] = half;
        BodyKind::Dynamic(Inertial::new(
            mass,
            [y * y + z * z, x * x + z * z, x * x + y * y].map(|v| mass * v / 3.),
            Pose::default(),
        )?)
    } else {
        BodyKind::Fixed
    };
    CookedRigidBody::new(RigidBodyDescription::new(
        kind,
        vec![ColliderDescription::new(
            ShapeDescription::Cuboid {
                half_extents_m: half,
            },
            Pose::default(),
            material(),
        )?],
    )?)
}

/// Prepared resources survive resets; all live bindings are reacquired by key.
pub struct Simulation {
    /// One generic scene, rebuilt on reset.
    pub runtime: PhysicalRuntime,
    models: [CookedRigidBody; 3],
    /// Completed 128 Hz ticks since reconstruction.
    pub tick: u64,
    /// Whether automatic advance is suspended; explicit step remains available.
    pub paused: bool,
}
impl Simulation {
    /// Prepare models once and install the initial scene.
    pub fn new(paused: bool) -> Result<Self, &'static str> {
        let models = [
            model(HALF_EXTENTS_M[0], Some(2.))?,
            model(HALF_EXTENTS_M[1], None)?,
            model(HALF_EXTENTS_M[2], Some(1.))?,
        ];
        let runtime = Self::install(&models)?;
        Ok(Self {
            runtime,
            models,
            tick: 0,
            paused,
        })
    }
    fn install(models: &[CookedRigidBody; 3]) -> Result<PhysicalRuntime, &'static str> {
        PhysicalRuntime::from_recipe(StepDuration::default(), |runtime| {
            runtime.install_catch_floor(0., material())?;
            for i in 0..3 {
                runtime.install_body(KEYS[i], &models[i], BodyState::at(Pose::at(INITIAL[i])))?;
            }
            let h = std::f64::consts::FRAC_1_SQRT_2;
            let frame = |translation_m| Pose {
                translation_m,
                rotation_wxyz: [h, 0., -h, 0.],
            };
            let description = JointDescription::new(
                JointKind::Revolute,
                frame([0., 0., 0.5]),
                frame([0.; 3]),
                Some(JointLimits::new(-1., 1.)?),
                false,
            )?;
            runtime.install_joint(
                YAW,
                runtime.body_binding(BASE).unwrap(),
                runtime.body_binding(ARM).unwrap(),
                &description,
            )?;
            Ok(())
        })
    }
    /// Read a privileged world FLU pose, rebinding the logical key each time.
    pub fn pose(&self, key: BodyKey) -> Pose {
        self.runtime
            .body_state(self.runtime.body_binding(key).unwrap())
            .unwrap()
            .pose
    }
    /// Set bounded motor torque control and advance exactly one explicit tick.
    pub fn step(&mut self) -> Result<(), &'static str> {
        let now = self.tick * self.runtime.step_duration().as_ns();
        let target = (now as f64 * 1e-9 * 0.8).sin() * 0.8;
        let binding = self.runtime.joint_binding(YAW).unwrap();
        self.runtime
            .set_joint_motor(binding, Some(JointMotor::new(target, 0., 40., 5., 8.)?))?;
        self.runtime.step(now)?;
        self.tick += 1;
        Ok(())
    }
    /// Advance the requested number of ticks unless paused.
    pub fn advance(&mut self, ticks: u32) -> Result<(), &'static str> {
        if !self.paused {
            for _ in 0..ticks {
                self.step()?;
            }
        }
        Ok(())
    }
    /// Reconstruct from retained prepared resources, preserving pause choice.
    pub fn reset(&mut self) -> Result<(), &'static str> {
        self.runtime = Self::install(&self.models)?;
        self.tick = 0;
        Ok(())
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn pause_step_reset_rebinds_and_replays_identically() {
        let mut sim = Simulation::new(true).unwrap();
        let old = sim.runtime.body_binding(CUBE).unwrap();
        sim.advance(128).unwrap();
        assert_eq!(sim.tick, 0);
        sim.step().unwrap();
        assert_eq!(sim.tick, 1);
        assert!(sim.pose(CUBE).translation_m[2] < 1.5);
        sim.paused = false;
        sim.advance(255).unwrap();
        let cube = sim.pose(CUBE);
        let arm = sim.pose(ARM);
        assert!((cube.translation_m[2] - 0.2).abs() < 0.02);
        assert!(arm.rotation_wxyz[3].abs() > 0.1);
        sim.paused = true;
        sim.reset().unwrap();
        assert!(sim.paused);
        assert_eq!(sim.tick, 0);
        assert!(sim.runtime.body_state(old).is_err());
        assert_eq!(sim.pose(CUBE).translation_m, INITIAL[0]);
        sim.paused = false;
        sim.advance(256).unwrap();
        assert_eq!(sim.pose(CUBE), cube);
        assert_eq!(sim.pose(ARM), arm);
    }
}
