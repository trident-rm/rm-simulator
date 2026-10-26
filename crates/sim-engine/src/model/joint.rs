// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Solver-free joint descriptions and scalar runtime observations/commands.
use super::ModelError;

use crate::Pose;
use std::f64::consts::PI;

/// Relative motion permitted between two body-local joint frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    /// Lock all translation and rotation; scalar position and velocity are zero.
    Fixed,
    /// Rotate about joint-local +X, positive by the right-hand rule, in radians.
    /// Angular observations have no multi-turn continuity guarantee.
    Revolute,
    /// Translate along joint-local +X in metres, with rotation locked.
    Prismatic,
}

/// Immutable inclusive scalar position bounds, in radians for a revolute joint
/// or metres for a prismatic joint. A joint description validates kind-specific
/// restrictions; equal endpoints are permitted.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointLimits {
    min: f64,
    max: f64,
}

impl JointLimits {
    /// Require finite endpoints with `min <= max`, without changing the values.
    ///
    /// # Errors
    /// Rejects invalid limits, frames or motor targets; fixed joints cannot carry limits.
    pub fn new(min: f64, max: f64) -> Result<Self, ModelError> {
        if !min.is_finite() || !max.is_finite() || min > max {
            return Err(ModelError::InvalidJointLimits);
        }
        Ok(Self { min, max })
    }

    /// Inclusive lower position bound, in radians or metres according to kind.
    pub fn min(&self) -> f64 {
        self.min
    }

    /// Inclusive upper position bound, in radians or metres according to kind.
    pub fn max(&self) -> f64 {
        self.max
    }
}

/// Immutable validated constraint source, without body identities or solver keys.
/// Each frame maps joint-local coordinates into its respective body's local FLU
/// coordinates (metres and wxyz quaternion). At zero displacement the two frames
/// coincide in world space. Positive displacement rotates/translates the child
/// frame relative to the parent frame about/along parent-frame +X.
///
/// ```
/// use sim_engine::{Pose, model::{JointDescription, JointKind, JointLimits}};
/// let hinge = JointDescription::new(
///     JointKind::Revolute, Pose::at([0., 0., 0.2]), Pose::default(),
///     Some(JointLimits::new(-0.5, 0.5)?), false,
/// )?;
/// assert_eq!(hinge.limits().unwrap().max(), 0.5);
/// assert!(!hinge.contacts_enabled());
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointDescription {
    kind: JointKind,
    parent_frame: Pose,
    child_frame: Pose,
    limits: Option<JointLimits>,
    contacts_enabled: bool,
}

impl JointDescription {
    /// Require finite translations and unit quaternions (squared norm tolerance
    /// 1e-6), without normalization. Fixed joints reject limits; revolute bounds
    /// must lie within inclusive [-π, π]. `None` leaves the free axis unlimited,
    /// but does not promise multi-turn revolute coordinates. Contact policy is
    /// explicit and applies to the connected pair, subject to collider filters.
    ///
    /// # Errors
    /// Rejects invalid limits, frames or motor targets; fixed joints cannot carry limits.
    pub fn new(
        kind: JointKind,
        parent_frame: Pose,
        child_frame: Pose,
        limits: Option<JointLimits>,
        contacts_enabled: bool,
    ) -> Result<Self, ModelError> {
        validate_frame(parent_frame)?;
        validate_frame(child_frame)?;
        if let Some(limits) = limits {
            match kind {
                JointKind::Fixed => return Err(ModelError::FixedJointLimits),
                JointKind::Revolute if limits.min < -PI || limits.max > PI => {
                    return Err(ModelError::RevoluteLimitsOutOfRange);
                }
                _ => {}
            }
        }
        Ok(Self {
            kind,
            parent_frame,
            child_frame,
            limits,
            contacts_enabled,
        })
    }

    /// Permitted relative motion and its scalar coordinate units.
    pub fn kind(&self) -> JointKind {
        self.kind
    }

    /// Joint-to-parent-body local frame, in metres and wxyz quaternion.
    pub fn parent_frame(&self) -> Pose {
        self.parent_frame
    }

    /// Joint-to-child-body local frame, in metres and wxyz quaternion.
    pub fn child_frame(&self) -> Pose {
        self.child_frame
    }

    /// Optional inclusive position bounds, in radians or metres according to kind.
    pub fn limits(&self) -> Option<JointLimits> {
        self.limits
    }

    /// Whether the connected bodies may contact, subject to collider filters.
    pub fn contacts_enabled(&self) -> bool {
        self.contacts_enabled
    }
}

/// Copied scalar observation, without a solver handle or mutable backend access.
/// Fixed joints report zero for both fields. Revolute coordinates have no
/// multi-turn continuity guarantee; this record is not a solver checkpoint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointState {
    /// Relative displacement in radians (revolute) or metres (prismatic).
    pub position: f64,
    /// Relative speed in radians/second (revolute) or metres/second (prismatic).
    pub velocity: f64,
}

/// Immutable finite motor targets and nonnegative spring/damper gains.
/// Position/velocity use rad and rad/s for revolute joints, m and m/s for
/// prismatic joints. Stiffness uses N m/rad or N/m; damping uses N m s/rad or
/// N s/m. Maximum effort is torque in N m for revolute joints or force in N for
/// prismatic joints. The runtime validates applicability and position limits
/// against its joint description; it must reject invalid targets, never clamp.
///
/// ```
/// use sim_engine::model::JointMotor;
/// let motor = JointMotor::new(0.2, 0., 10., 1., 5.)?;
/// assert_eq!(motor.target_position(), 0.2);
/// assert_eq!(motor.max_force(), 5.);
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointMotor {
    target_position: f64,
    target_velocity: f64,
    stiffness: f64,
    damping: f64,
    max_force: f64,
}

impl JointMotor {
    /// Require finite targets, finite nonnegative gains and finite positive
    /// maximum effort. No reciprocal-finiteness restriction is imposed.
    ///
    /// # Errors
    /// Rejects invalid limits, frames or motor targets; fixed joints cannot carry limits.
    pub fn new(
        target_position: f64,
        target_velocity: f64,
        stiffness: f64,
        damping: f64,
        max_force: f64,
    ) -> Result<Self, ModelError> {
        if ![
            target_position,
            target_velocity,
            stiffness,
            damping,
            max_force,
        ]
        .into_iter()
        .all(f64::is_finite)
            || stiffness < 0.
            || damping < 0.
            || max_force <= 0.
        {
            return Err(ModelError::InvalidJointMotor);
        }
        Ok(Self {
            target_position,
            target_velocity,
            stiffness,
            damping,
            max_force,
        })
    }

    /// Desired position in radians (revolute) or metres (prismatic).
    pub fn target_position(&self) -> f64 {
        self.target_position
    }

    /// Desired speed in rad/s (revolute) or m/s (prismatic).
    pub fn target_velocity(&self) -> f64 {
        self.target_velocity
    }

    /// Spring gain in N m/rad (revolute) or N/m (prismatic).
    pub fn stiffness(&self) -> f64 {
        self.stiffness
    }

    /// Damping gain in N m s/rad (revolute) or N s/m (prismatic).
    pub fn damping(&self) -> f64 {
        self.damping
    }

    /// Positive effort limit in N m (revolute) or N (prismatic).
    pub fn max_force(&self) -> f64 {
        self.max_force
    }
}

fn validate_frame(frame: Pose) -> Result<(), ModelError> {
    frame
        .validate_with(crate::pose::PoseTolerance::Model)
        .map_err(|_| ModelError::InvalidJointFrame)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn description(
        kind: JointKind,
        limits: Option<JointLimits>,
    ) -> Result<JointDescription, ModelError> {
        JointDescription::new(kind, Pose::default(), Pose::default(), limits, false)
    }

    #[test]
    fn limits_validate_finiteness_order_and_kind_specific_bounds() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(JointLimits::new(value, 1.).is_err());
            assert!(JointLimits::new(-1., value).is_err());
        }
        assert!(JointLimits::new(1., -1.).is_err());
        let equal = JointLimits::new(0.3, 0.3).unwrap();
        assert_eq!((equal.min(), equal.max()), (0.3, 0.3));
        assert!(description(JointKind::Revolute, Some(equal)).is_ok());
        assert!(description(JointKind::Fixed, Some(equal)).is_err());
        let full = JointLimits::new(-PI, PI).unwrap();
        assert!(description(JointKind::Revolute, Some(full)).is_ok());
        for (min, max) in [(-PI - 1e-12, PI), (-PI, PI + 1e-12)] {
            let limits = Some(JointLimits::new(min, max).unwrap());
            assert!(description(JointKind::Revolute, limits).is_err());
            assert!(description(JointKind::Prismatic, limits).is_ok());
        }
        for kind in [JointKind::Fixed, JointKind::Revolute, JointKind::Prismatic] {
            assert!(description(kind, None).is_ok());
        }
    }

    #[test]
    fn frames_and_explicit_contact_policy_are_preserved() {
        let parent = Pose::yawed([1., 2., 3.], 0.7);
        let child = Pose::yawed([-0.2, 0.3, 0.], -0.4);
        for contacts in [false, true] {
            let joint =
                JointDescription::new(JointKind::Prismatic, parent, child, None, contacts).unwrap();
            assert_eq!(joint.kind(), JointKind::Prismatic);
            assert_eq!(joint.parent_frame(), parent);
            assert_eq!(joint.child_frame(), child);
            assert_eq!(joint.limits(), None);
            assert_eq!(joint.contacts_enabled(), contacts);
        }
        for bad in [
            Pose {
                translation_m: [f64::NAN, 0., 0.],
                ..parent
            },
            Pose {
                translation_m: [0., f64::INFINITY, 0.],
                ..parent
            },
            Pose {
                rotation_wxyz: [0.; 4],
                ..parent
            },
            Pose {
                rotation_wxyz: [2., 0., 0., 0.],
                ..parent
            },
            Pose {
                rotation_wxyz: [f64::NAN, 0., 0., 0.],
                ..parent
            },
            Pose {
                rotation_wxyz: [f64::MAX, 0., 0., 0.],
                ..parent
            },
        ] {
            assert!(JointDescription::new(JointKind::Fixed, bad, child, None, false).is_err());
            assert!(JointDescription::new(JointKind::Fixed, parent, bad, None, false).is_err());
        }
        // Both signs represent the same valid unit rotation, without normalization.
        let negative_identity = Pose {
            rotation_wxyz: [-1., 0., 0., 0.],
            ..Pose::default()
        };
        assert!(
            JointDescription::new(JointKind::Fixed, negative_identity, child, None, false).is_ok()
        );
    }

    #[test]
    fn motor_rejects_nonfinite_values_and_invalid_gains_or_effort() {
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for index in 0..5 {
                let mut values = [0., 0., 1., 1., 1.];
                values[index] = invalid;
                let [p, v, k, d, f] = values;
                assert!(JointMotor::new(p, v, k, d, f).is_err());
            }
        }
        assert!(JointMotor::new(0., 0., -1., 1., 1.).is_err());
        assert!(JointMotor::new(0., 0., 1., -1., 1.).is_err());
        for effort in [0., -1.] {
            assert!(JointMotor::new(0., 0., 1., 1., effort).is_err());
        }
        let tiny = f64::from_bits(1);
        let motor = JointMotor::new(-10., -20., 0., 0., tiny).unwrap();
        assert_eq!(motor.target_position(), -10.);
        assert_eq!(motor.target_velocity(), -20.);
        assert_eq!(motor.stiffness(), 0.);
        assert_eq!(motor.damping(), 0.);
        assert_eq!(motor.max_force(), tiny);
        // Large finite values are retained; only runtime binding knows limits.
        assert!(JointMotor::new(f64::MAX, f64::MIN, f64::MAX, f64::MAX, f64::MAX).is_ok());
    }
}
