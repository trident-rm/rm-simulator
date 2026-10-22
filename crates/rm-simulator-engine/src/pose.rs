// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Solver-independent FLU poses with wxyz quaternion storage.
use serde::{Deserialize, Serialize};

/// A rigid pose in forward/left/up coordinates, metres and a wxyz quaternion.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Pose {
    /// Translation in metres, forward/left/up.
    pub translation_m: [f64; 3],
    /// Rotation as a unit wxyz quaternion.
    pub rotation_wxyz: [f64; 4],
}
impl Default for Pose {
    fn default() -> Self {
        Self {
            translation_m: [0.0; 3],
            rotation_wxyz: [1.0, 0.0, 0.0, 0.0],
        }
    }
}
impl Pose {
    /// An unrotated pose at `translation_m`.
    ///
    /// ```
    /// use rm_simulator_engine::Pose;
    ///
    /// let mut pose = Pose::at([1.0, -2.0, 0.5]);
    /// assert_eq!(pose.rotation_wxyz, [1.0, 0.0, 0.0, 0.0]);
    ///
    /// // Yaw turns about world up, counter-clockwise from +x.
    /// pose = Pose::yawed(pose.translation_m, std::f64::consts::FRAC_PI_2);
    /// assert!((2. * pose.rotation_wxyz[3].atan2(pose.rotation_wxyz[0])
    ///     - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    /// ```
    pub fn at(translation_m: [f64; 3]) -> Self {
        Self {
            translation_m,
            ..Self::default()
        }
    }
    /// A pose rotated about world up (yaw) only.
    ///
    /// ```
    /// use rm_simulator_engine::Pose;
    ///
    /// // Yaw is counter-clockwise about world up, so a quarter turn sends
    /// // forward (+x) to left (+y).
    /// let pose = Pose::yawed([0.0; 3], std::f64::consts::FRAC_PI_2);
    /// let [w, _, _, z] = pose.rotation_wxyz;
    /// let forward = [1. - 2. * z * z, 2. * w * z, 0.];
    /// assert!(forward[0].abs() < 1e-12);
    /// assert!((forward[1] - 1.0).abs() < 1e-12);
    /// ```
    pub fn yawed(translation_m: [f64; 3], yaw_rad: f64) -> Self {
        let (s, c) = (yaw_rad / 2.0).sin_cos();
        Self {
            translation_m,
            rotation_wxyz: [c, 0.0, 0.0, s],
        }
    }
}
