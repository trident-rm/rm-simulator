// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Viewer-owned selection/pacing over two independently owned physical examples.
use crate::simulation::{self, Simulation};
use sim_engine::{Pose, StepDuration};
use sim_pendulum_example::{Experiment, PARTS, VisualPart, reference_rad};
use sim_physics::runtime::BodyKey;

const FIXTURE_PARTS: [VisualPart; 3] = [
    VisualPart {
        body: simulation::KEYS[0],
        half_extents_m: simulation::HALF_EXTENTS_M[0],
        color_srgb: [0.98, 0.48, 0.16],
    },
    VisualPart {
        body: simulation::KEYS[1],
        half_extents_m: simulation::HALF_EXTENTS_M[1],
        color_srgb: [0.16, 0.24, 0.32],
    },
    VisualPart {
        body: simulation::KEYS[2],
        half_extents_m: simulation::HALF_EXTENTS_M[2],
        color_srgb: [0.12, 0.7, 0.76],
    },
];
enum Kind {
    Fixture(Simulation),
    Pendulum(Experiment),
}

/// One selected scene. Pause/presentation pacing belong only to this viewer.
pub struct Scene {
    inner: Kind,
    /// Whether automatic physical advance is suspended.
    pub paused: bool,
}
impl Scene {
    /// Construct either the original fixture or the independent pendulum example.
    pub fn new(paused: bool, pendulum: bool) -> Result<Self, &'static str> {
        let inner = if pendulum {
            Kind::Pendulum(Experiment::new()?)
        } else {
            Kind::Fixture(Simulation::new(false)?)
        };
        Ok(Self { inner, paused })
    }
    /// Caller-owned visual recipes; no second physical model is constructed here.
    pub fn parts(&self) -> &'static [VisualPart] {
        match self.inner {
            Kind::Fixture(_) => &FIXTURE_PARTS,
            Kind::Pendulum(_) => &PARTS,
        }
    }
    /// Fixed support whose material is also used for the presentation floor.
    pub fn support_body(&self) -> BodyKey {
        match self.inner {
            Kind::Fixture(_) => simulation::BASE,
            Kind::Pendulum(_) => PARTS[0].body,
        }
    }
    /// Completed ticks, independent of the render frame count.
    pub fn tick(&self) -> u64 {
        match &self.inner {
            Kind::Fixture(s) => s.tick,
            Kind::Pendulum(s) => s.stamp().tick,
        }
    }
    /// The selected model's physical integration duration.
    pub fn step_duration(&self) -> StepDuration {
        match &self.inner {
            Kind::Fixture(s) => s.runtime.step_duration(),
            Kind::Pendulum(s) => s.runtime().step_duration(),
        }
    }
    /// Privileged physical pose used only by presentation.
    pub fn pose(&self, key: BodyKey) -> Pose {
        match &self.inner {
            Kind::Fixture(s) => s.pose(key),
            Kind::Pendulum(s) => {
                s.runtime()
                    .body_state(s.runtime().body_binding(key).expect("visual body key"))
                    .expect("live visual body")
                    .pose
            }
        }
    }
    /// Optional existing GLB attachment point, absent for the pendulum scene.
    pub fn attachment_pose(&self) -> Option<Pose> {
        match &self.inner {
            Kind::Fixture(s) => Some(s.pose(simulation::ARM)),
            Kind::Pendulum(_) => None,
        }
    }
    /// Advance exactly one model tick, including its actual sensor/controller work.
    pub fn step(&mut self) -> Result<(), &'static str> {
        match &mut self.inner {
            Kind::Fixture(s) => s.step(),
            Kind::Pendulum(s) => s.advance(1),
        }
    }
    /// Advance requested ticks unless paused, using the same path as manual stepping.
    pub fn advance(&mut self, ticks: u32) -> Result<(), &'static str> {
        if !self.paused {
            match &mut self.inner {
                Kind::Fixture(s) => s.advance(ticks)?,
                Kind::Pendulum(s) => s.advance(u64::from(ticks))?,
            }
        }
        Ok(())
    }
    /// Reset physical and extension state, preserving the viewer's pause choice.
    pub fn reset(&mut self) -> Result<(), &'static str> {
        match &mut self.inner {
            Kind::Fixture(s) => s.reset(),
            Kind::Pendulum(s) => s.reset(),
        }
    }
    /// Compact scene telemetry for the diagnostic HUD and headless output.
    pub fn summary(&self) -> String {
        match &self.inner {
            Kind::Fixture(s) => {
                format!("cube z={:.4} m", s.pose(simulation::CUBE).translation_m[2])
            }
            Kind::Pendulum(s) => {
                let telemetry = s.telemetry();
                let target = reference_rad(
                    s.stamp()
                        .time_ns
                        .saturating_sub(s.runtime().step_duration().as_ns()),
                );
                let angle = telemetry
                    .encoder
                    .map(|sample| format!("{:.3}", sample.angle_rad()))
                    .unwrap_or_else(|| "--".into());
                format!(
                    "Pendulum  encoder={angle} rad  target={target:.2} rad  voltage={:.2} V",
                    telemetry.motor.voltage_v
                )
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn viewer_and_headless_share_poses_clocks_and_complete_reset() {
        let mut viewer = Scene::new(true, true).unwrap();
        let mut headless = Experiment::new().unwrap();
        viewer.advance(20).unwrap();
        assert_eq!(viewer.tick(), 0);
        viewer.step().unwrap();
        headless.advance(1).unwrap();
        viewer.paused = false;
        for _ in 0..250 {
            viewer.advance(4).unwrap();
            headless.advance(4).unwrap();
        }
        for part in PARTS {
            let binding = headless.runtime().body_binding(part.body).unwrap();
            assert_eq!(
                viewer.pose(part.body),
                headless.runtime().body_state(binding).unwrap().pose
            );
        }
        assert_eq!(viewer.tick(), headless.stamp().tick);
        viewer.paused = true;
        viewer.reset().unwrap();
        headless.reset().unwrap();
        assert!(viewer.paused);
        assert_eq!(viewer.tick(), 0);
        assert!(viewer.summary().contains("encoder=--"));
        viewer.step().unwrap();
        headless.advance(1).unwrap();
        assert!(viewer.summary().contains("voltage="));
        assert_eq!(
            viewer.pose(PARTS[1].body),
            headless
                .runtime()
                .body_state(headless.runtime().body_binding(PARTS[1].body).unwrap())
                .unwrap()
                .pose
        );
    }
}
