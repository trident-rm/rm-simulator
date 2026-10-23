// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared performance selection and statistics records. Historical source labels
//! remain serialized; native 2027 combat owns all live balance tables.
use serde::{Deserialize, Serialize};

/// Highest robot level (Table 5-11).
pub const MAX_LEVEL: u8 = 10;

/// Hero type (Table 5-12). Section 5.4.2 defaults an unselected Hero to
/// long-range attack-focused.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum HeroType {
    /// Melee-focused: more HP and power, slower cooling.
    MeleeFocused,
    /// Long-range attack-focused, the section 5.4.2 default.
    #[default]
    LongRangeFocused,
}

/// Infantry chassis type (Table 5-13). Section 5.4.2 defaults an unselected
/// chassis to HP-focused.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum InfantryChassis {
    /// Power-focused: less HP, more chassis power.
    PowerFocused,
    /// HP-focused, the section 5.4.2 default.
    #[default]
    HpFocused,
}

/// Infantry 17 mm launching mechanism type (Table 5-14). Section 5.4.2
/// defaults an unselected launcher to cooling-focused.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum InfantryLauncher {
    /// Burst-focused: high heat limit, slow cooling.
    BurstFocused,
    /// Cooling-focused, the section 5.4.2 default.
    #[default]
    CoolingFocused,
}

/// A robot's performance values at one level.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Stats {
    /// Maximum HP; positive.
    pub max_hp: u32,
    /// Chassis power limit in watts; reported, not enforced (section 5.1.4).
    pub chassis_power_w: u32,
    /// Barrel heat limit Q0 in heat units (section 5.1.3).
    pub heat_limit: u32,
    /// Barrel heat cooling in heat units per second (section 5.1.3).
    pub cooling_per_s: u32,
}

/// Shared performance selection descriptor. Native rules resolve its values.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Performance {
    /// Table 5-12 by Hero type.
    Hero(HeroType),
    /// Table 5-13 chassis and Table 5-14 launcher.
    Infantry {
        /// Chassis type, which sets HP and power.
        chassis: InfantryChassis,
        /// Launching mechanism type, which sets heat and cooling.
        launcher: InfantryLauncher,
    },
    /// Caller-supplied values for robots the performance system does not cover
    /// (section 5.4.2: Engineer, Sentry, Drone HP) and for scenarios. Levels do
    /// not change them.
    Fixed(Stats),
}

impl Performance {
    /// Default selection descriptor for Infantry or a reference Hero model.
    /// Other roles have native role-specific statistics, not historical tables.
    pub fn default_for(kind: crate::RobotKind) -> Option<Self> {
        match kind {
            crate::RobotKind::Hero => Some(Self::Hero(HeroType::default())),
            crate::RobotKind::Infantry => Some(Self::Infantry {
                chassis: InfantryChassis::default(),
                launcher: InfantryLauncher::default(),
            }),
            _ => None,
        }
    }
    /// Whether this source may describe a robot of `kind`: Hero tables for a
    /// Hero, Infantry tables for Infantry, fixed values for anything, and a
    /// positive maximum HP in every case.
    pub fn fits(self, kind: crate::RobotKind) -> bool {
        match self {
            Self::Hero(_) => kind == crate::RobotKind::Hero,
            Self::Infantry { .. } => kind == crate::RobotKind::Infantry,
            Self::Fixed(stats) => stats.max_hp > 0,
        }
    }
}
