// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Prescribed outpost rotation and shared target/guard geometry. No renderer or scheduler.
//! Geometry was fitted from the extracted RMUC2026 CAD in `rm-vision-sim`.
use crate::{ArmorSnapshot, Pose};
use serde::{Deserialize, Serialize};

use rm_sim_physics::motion::RotorMotion;
pub use rm_sim_physics::motion::outpost::*;
/// RMUC 2027 V1.0.0 initial Outpost HP (§5.6.1).
pub const INITIAL_HP: u32 = 1000;

/// One outpost frame at an authoritative tick: the rotor angle and the three
/// armor poses it implies. HP and destruction travel with it so a client can
/// freeze a destroyed tower without recomputing anything.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OutpostSnapshot {
    /// Configured motion and fitted pivot for client pose reconstruction.
    pub speed_rad_s: f64,
    /// CAD-space rotor pivot in metres; the fitted pivot when the asset did
    /// not supply a verified one.
    pub pivot_cad_m: [f64; 3],
    /// Field time this frame was taken at.
    pub time_ns: u64,
    /// Tower base pose in FLU metres.
    pub origin: Pose,
    /// Rotor angle at `time_ns`, radians counter-clockwise about the tower's up axis.
    pub angle_rad: f64,
    /// The three armor modules in face order, front first.
    pub armors: Vec<ArmorSnapshot>,
    /// Remaining outpost HP, at most [`INITIAL_HP`].
    pub hp: u32,
    /// HP reached zero; the middle armor stopped where it was.
    pub destroyed: bool,
    /// Field time the match's rotor spin-up began, when a match runs it.
    pub started_ns: Option<u64>,
    /// Field time the rotor stopped, when it has.
    pub stopped_ns: Option<u64>,
    /// Whether the stop returns the armor to its initial position.
    pub homing: bool,
}
impl OutpostSnapshot {
    /// Reconstruct armor geometry from the transmitted rotor pose, including
    /// a destroyed outpost frozen between ticks. HP is never recomputed.
    pub fn rebuild_armors(&mut self) {
        self.armors = armor_poses_at(self.origin, self.pivot_cad_m, self.angle_rad)
            .into_iter()
            .enumerate()
            .map(|(id, pose)| ArmorSnapshot {
                id: id as u32,
                pose,
            })
            .collect();
    }
    /// Presentation only: retain authoritative destruction and HP.
    pub fn presentation_at(&self, time_ns: u64) -> Self {
        if self.destroyed || time_ns <= self.time_ns {
            return self.clone();
        }
        let motion = RotorMotion {
            pivot_cad_m: self.pivot_cad_m,
            origin: self.origin,
            speed_rad_s: self.speed_rad_s,
            stopped_at_ns: self.stopped_ns,
            started_ns: self.started_ns,
            homing: self.homing,
        };
        let mut presented = self.clone();
        presented.angle_rad = motion.angle_at(time_ns);
        presented.time_ns = time_ns;
        presented.rebuild_armors();
        presented
    }
}

/// Shared presentation type; live Outpost HP and rotation belong to native 2027 rules.
pub type Outpost = OutpostSnapshot;
