// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! RoboMaster participant adapters over `sim-physics`, without match
//! rules, networking, CAD loading, Bevy or host time. Callers supply target endpoint
//! poses and resolved mechanism state, step once per explicit `tick_ns` tick, then
//! consume contacts in returned order. See the `moving_armor` example.
//!
//! ```
//! use rm_sim_physics::{
//!     ArmorTarget, Caliber, Pose, Shot, TargetFace, TargetFrames, WorldPhysics, tick_ns,
//! };
//!
//! let face = TargetFace {
//!     target: ArmorTarget::Outpost { outpost: 0, face: 0 },
//!     // +x is the outward normal, so a half turn faces the shooter at the origin.
//!     pose: Pose::yawed([1.5, 0.0, 1.0], std::f64::consts::PI),
//! };
//! let mut physics = WorldPhysics::new(&[face], 0.0);
//! let frames = TargetFrames::new(vec![face]);
//! physics.fire(0, Pose::at([0.0, 0.0, 1.0]), Shot::at_limit(Caliber::Mm17), None)?;
//!
//! let mut contacts = 0;
//! for tick in 0..200 {
//!     contacts += physics.step(tick * tick_ns(), &frames)?.len();
//! }
//! // The 1.5 m flight takes about 60 ticks; each ball touches the housing once.
//! assert_eq!(contacts, 1);
//! # Ok::<(), &'static str>(())
//! ```
#![deny(missing_docs)]
pub mod arena;
pub mod assembly_model;
pub mod chassis;
pub mod collision_profiles;
pub mod control;
pub mod cooking;
pub mod geometry;
pub mod manipulator;
pub mod manipulator_collision;
pub mod mechanism;
pub mod motion;
pub mod projectile;
pub mod queries;
/// Single-owner physical runtime, also available through compatibility exports.
pub mod runtime;
/// Standalone physical scene without competition participant adapters.
pub use runtime::PhysicalRuntime;
pub mod sensors;
pub mod session;
pub mod wheel_model;
pub use projectile::{
    ArmorTarget, Caliber, Contact, Shot, StaticGeometry, TargetFace, TargetFrames, WorldPhysics,
};
use serde::{Deserialize, Serialize};

/// Solver-free source geometry, body and collider descriptions.
pub use sim_engine::model;
/// Shared solver-free contracts; existing physical import paths stay compatible.
pub use sim_engine::{DEFAULT_TICK_NS, Pose, StepDuration, tick_ns};

/// One of the two alliances. [`Team::index`] is the slot a team fills in the
/// fixed `[_; 2]` arrays that mechanism and geometry state use.
///
/// ```
/// use rm_sim_physics::Team;
///
/// assert_eq!(Team::Red.index(), 0);
/// assert_eq!(Team::Blue.index(), 1);
/// assert_eq!(Team::Red.other(), Team::Blue);
/// assert_eq!(Team::BOTH.map(Team::name), ["red", "blue"]);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Team {
    /// The red alliance.
    Red,
    /// The blue alliance.
    Blue,
}
impl Team {
    /// Both teams, red then blue.
    pub const BOTH: [Team; 2] = [Team::Red, Team::Blue];
    /// Array slot for this team: red 0, blue 1.
    pub fn index(self) -> usize {
        match self {
            Team::Red => 0,
            Team::Blue => 1,
        }
    }
    /// The opposing team.
    pub fn other(self) -> Team {
        match self {
            Team::Red => Team::Blue,
            Team::Blue => Team::Red,
        }
    }
    /// Lower-case name, `"red"` or `"blue"`.
    pub fn name(self) -> &'static str {
        match self {
            Team::Red => "red",
            Team::Blue => "blue",
        }
    }
}
