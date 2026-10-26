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
    /// use sim_engine::Pose;
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
    /// use sim_engine::Pose;
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

/// The established admission rule for a pose, independent of backend conversion.
/// Tolerances are explicit because source, runtime and asset contracts differ.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PoseTolerance {
    /// Squared quaternion norm error at most 1e-6, for source models and mounts.
    Model,
    /// Squared quaternion norm error strictly below 1e-6, for live runtime input.
    Runtime,
    /// Quaternion length error at most 1e-6, for composed asset placements.
    Asset,
    /// Squared quaternion norm error at most 1e-8, for recorded IK input.
    IkHint,
}

/// A pose cannot represent a finite rigid transform under its admission policy.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PoseError {
    /// At least one translation component is not finite.
    NonfiniteTranslation,
    /// Quaternion norm is nonfinite or outside the selected unit-length tolerance.
    NonunitRotation,
}
impl std::fmt::Display for PoseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::NonfiniteTranslation => "pose translation must be finite",
            Self::NonunitRotation => "pose rotation must have unit length",
        })
    }
}
impl std::error::Error for PoseError {}
impl Pose {
    /// Check finite translation and the selected quaternion norm rule without repair.
    ///
    /// # Errors
    /// Returns a translation or rotation error; normalization is never implicit.
    ///
    /// ```
    /// use sim_engine::{Pose, pose::PoseTolerance};
    /// Pose::default().validate_with(PoseTolerance::Model)?;
    /// # Ok::<(), sim_engine::pose::PoseError>(())
    /// ```
    pub fn validate_with(self, tolerance: PoseTolerance) -> Result<(), PoseError> {
        if !self.translation_m.into_iter().all(f64::is_finite) {
            return Err(PoseError::NonfiniteTranslation);
        }
        let norm_squared = self.rotation_wxyz.iter().map(|x| x * x).sum::<f64>();
        let accepted = norm_squared.is_finite()
            && match tolerance {
                PoseTolerance::Model => (norm_squared - 1.).abs() <= 1e-6,
                PoseTolerance::Runtime => (norm_squared - 1.).abs() < 1e-6,
                PoseTolerance::Asset => (norm_squared.sqrt() - 1.).abs() <= 1e-6,
                PoseTolerance::IkHint => (norm_squared - 1.).abs() <= 1e-8,
            };
        if !accepted {
            return Err(PoseError::NonunitRotation);
        }
        Ok(())
    }
}
#[cfg(test)]
mod validation_tests {
    use super::*;
    #[test]
    fn policies_preserve_historical_acceptance_without_normalization() {
        for w in [
            0.,
            -1.,
            1.,
            f64::NAN,
            f64::INFINITY,
            1. + 0.75e-6,
            1. + 1e-8,
        ] {
            let p = Pose {
                rotation_wxyz: [w, 0., 0., 0.],
                ..Pose::default()
            };
            let n = w * w;
            for (policy, expected) in [
                (
                    PoseTolerance::Model,
                    n.is_finite() && (n - 1.).abs() <= 1e-6,
                ),
                (
                    PoseTolerance::Runtime,
                    n.is_finite() && (n - 1.).abs() < 1e-6,
                ),
                (
                    PoseTolerance::Asset,
                    n.is_finite() && (n.sqrt() - 1.).abs() <= 1e-6,
                ),
                (
                    PoseTolerance::IkHint,
                    n.is_finite() && (n - 1.).abs() <= 1e-8,
                ),
            ] {
                assert_eq!(p.validate_with(policy).is_ok(), expected, "{policy:?}: {w}");
            }
        }
        assert_eq!(
            Pose::at([f64::NAN, 0., 0.]).validate_with(PoseTolerance::Model),
            Err(PoseError::NonfiniteTranslation)
        );
    }
}
