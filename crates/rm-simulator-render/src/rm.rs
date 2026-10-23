// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! RoboMaster appearance, equipment and match-scene compatibility surface.
//!
//! These plugins consume resolved caller appearances; they do not run a game.
//! General viewers should use [`crate::generic`] instead. The original root
//! modules remain public aliases of these same implementations.
pub use crate::{
    TeamColor, armor, cad, chassis, outpost, projectile, rune, sync, targets, team_robot,
};
