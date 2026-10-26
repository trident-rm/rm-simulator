// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Explicit integration and collider mass/contact policies, independent of a solver.
use super::ModelError;

/// Coefficient combination for a contacting pair. If the two rules differ, the
/// higher priority wins: Average < Min < Multiply < Max.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CoefficientCombineRule {
    /// Arithmetic mean of the two coefficients.
    #[default]
    Average,
    /// Smaller coefficient.
    Min,
    /// Product of the coefficients.
    Multiply,
    /// Larger coefficient.
    Max,
}

/// Explicit contribution of one collider to its body's mass properties.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum ColliderMass {
    /// No mass or inertia contribution (zero density).
    #[default]
    Zero,
    /// Shape-derived inertia scaled to this nonnegative mass in kilograms.
    MassKg(f64),
    /// Shape-derived mass and inertia at this nonnegative density in kg/m³.
    DensityKgM3(f64),
}
impl ColliderMass {
    pub(super) fn validate(self) -> Result<(), ModelError> {
        match self {
            Self::Zero => Ok(()),
            Self::MassKg(x) | Self::DensityKgM3(x) if x.is_finite() && x >= 0. => Ok(()),
            _ => Err(ModelError::InvalidColliderMass),
        }
    }
    pub(super) fn contributes(self) -> bool {
        matches!(self, Self::MassKg(x) | Self::DensityKgM3(x) if x > 0.)
    }
}

/// Collider mass and contact combination policy. Defaults preserve generic
/// source bodies: zero density and arithmetic-mean coefficients.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ColliderPolicy {
    /// Explicit mass contribution; never inferred from CAD material metadata.
    pub mass: ColliderMass,
    /// Friction coefficient combination.
    pub friction_combine_rule: CoefficientCombineRule,
    /// Restitution coefficient combination.
    pub restitution_combine_rule: CoefficientCombineRule,
}

/// Immutable-body integration settings, validated when attached to a description.
/// Defaults preserve generic body behavior: gravity at scale one, all axes free,
/// sleeping allowed, zero damping, and CCD disabled.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyPolicy {
    /// Dimensionless finite gravity multiplier; negative scales reverse gravity.
    pub gravity_scale: f64,
    /// Enabled world translation axes x/y/z; false locks that axis.
    pub enabled_translations: [bool; 3],
    /// Enabled world rotation axes x/y/z; false locks that axis.
    pub enabled_rotations: [bool; 3],
    /// Whether the solver may put this body to sleep.
    pub can_sleep: bool,
    /// Nonnegative linear velocity damping rate in inverse seconds.
    pub linear_damping: f64,
    /// Nonnegative angular velocity damping rate in inverse seconds.
    pub angular_damping: f64,
    /// Enable continuous collision detection for this body.
    pub ccd_enabled: bool,
}
impl Default for BodyPolicy {
    fn default() -> Self {
        Self {
            gravity_scale: 1.,
            enabled_translations: [true; 3],
            enabled_rotations: [true; 3],
            can_sleep: true,
            linear_damping: 0.,
            angular_damping: 0.,
            ccd_enabled: false,
        }
    }
}
impl BodyPolicy {
    pub(super) fn validate(self) -> Result<(), ModelError> {
        if !self.gravity_scale.is_finite()
            || ![self.linear_damping, self.angular_damping]
                .into_iter()
                .all(|x| x.is_finite() && x >= 0.)
        {
            return Err(ModelError::InvalidBodyPolicy);
        }
        Ok(())
    }
}
