// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared polygon/height records used by diagnostic imports. Native 2027 RFID
//! occupation and traversal rules live in `rmuc_2027`; no 2026 zone catalog remains.
use crate::Team;
use rm_simulator_gameplay::{self as gp, ZoneKind};
use serde::{Deserialize, Serialize};

/// Height of the body centre above a point's floor, in metres, within which
/// the chassis counts as standing on it. Wide enough for the chassis' ride
/// height and suspension travel, and narrow enough that a deck above a tunnel
/// pad or a floor below a highland point does not count. An application
/// setting, not a rule constant.
pub const ZONE_BODY_BAND_M: f64 = 0.45;

/// One buff point card area on the field.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ZoneArea {
    /// Buff point kind.
    pub kind: ZoneKind,
    /// Team the point belongs to, or the side it lies on for points any team
    /// may use.
    pub owner: Team,
    /// Pad number within a terrain crossing course; 0 otherwise
    /// ([`gp::Zone::pad`]).
    pub pad: u8,
    /// Floor height under the point, in metres.
    pub floor_m: f64,
    /// Footprint corners in field x and y, in metres, in either winding.
    pub polygon_m: Vec<[f64; 2]>,
}

impl ZoneArea {
    /// The gameplay zone this area reports.
    pub fn zone(&self) -> gp::Zone {
        gp::Zone {
            kind: self.kind,
            owner: crate::referee::game_team(self.owner),
            pad: self.pad,
        }
    }
    /// Whether a chassis body centre at `position_m` stands on this area.
    ///
    /// ```
    /// use rm_simulator_world::{Team, zones::ZoneArea, gameplay::ZoneKind};
    ///
    /// let area = ZoneArea {
    ///     kind: ZoneKind::Base,
    ///     owner: Team::Red,
    ///     pad: 0,
    ///     floor_m: 0.2,
    ///     polygon_m: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
    /// };
    /// assert!(area.contains([0.5, 0.5, 0.4]));
    /// assert!(!area.contains([1.5, 0.5, 0.4]));
    /// // Below the floor or high above it does not count.
    /// assert!(!area.contains([0.5, 0.5, 0.1]));
    /// assert!(!area.contains([0.5, 0.5, 0.9]));
    /// ```
    pub fn contains(&self, position_m: [f64; 3]) -> bool {
        let [x, y, z] = position_m;
        if !(self.floor_m..=self.floor_m + ZONE_BODY_BAND_M).contains(&z) {
            return false;
        }
        let mut inside = false;
        let n = self.polygon_m.len();
        for i in 0..n {
            let [xi, yi] = self.polygon_m[i];
            let [xj, yj] = self.polygon_m[(i + n - 1) % n];
            if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi {
                inside = !inside;
            }
        }
        inside
    }
}
