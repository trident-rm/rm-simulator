// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Validation failures for model descriptions.

/// A rejected model input; no source values are repaired implicitly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ModelError {
    /// Collider mass or density must be finite and nonnegative.
    InvalidColliderMass,
    /// Invalid body gravity or damping policy.
    InvalidBodyPolicy,
    /// Invalid sphere radius.
    InvalidSphereRadius,
    /// Invalid cylinder radius or half height.
    InvalidCylinderRadiusOrHalfHeight,
    /// Invalid box half extents.
    InvalidBoxHalfExtents,
    /// Invalid contact material.
    InvalidContactMaterial,
    /// Invalid mass or principal inertia.
    InvalidMassOrPrincipalInertia,
    /// Unrealizable principal inertia.
    UnrealizablePrincipalInertia,
    /// Dynamic triangle surfaces are unsupported.
    DynamicSurface,
    /// Collider-derived dynamic body requires a positive mass contribution.
    MissingColliderMass,
    /// Collider mass requires explicit collider-derived dynamic body mode.
    UnexpectedColliderMass,
    /// Nonfinite body velocity.
    NonfiniteBodyVelocity,
    /// Invalid body or collider pose.
    InvalidPose,
    /// Invalid mount pose.
    InvalidMountPose,
    /// Duplicate carrier key.
    DuplicateCarrierKey,
    /// Duplicate mount key.
    DuplicateMountKey,
    /// Mount carrier is missing.
    MissingMountCarrier,
    /// Invalid finite heightfield.
    InvalidHeightfield,
    /// Joint limits must be finite and ordered.
    InvalidJointLimits,
    /// Fixed joints cannot have limits.
    FixedJointLimits,
    /// Revolute limits must lie within [-pi, pi].
    RevoluteLimitsOutOfRange,
    /// Invalid joint motor targets, gains or maximum effort.
    InvalidJointMotor,
    /// Invalid joint frame.
    InvalidJointFrame,
    /// Convex hull requires at least four points.
    TooFewHullPoints,
    /// Convex hull points must be finite.
    NonfiniteHullPoint,
    /// Assembly must contain at least one link.
    EmptyAssembly,
    /// Duplicate assembly link ID.
    DuplicateAssemblyLinkId,
    /// Assembly root link is missing.
    MissingAssemblyRoot,
    /// Duplicate assembly joint ID.
    DuplicateAssemblyJointId,
    /// Assembly joint parent link is missing.
    MissingJointParent,
    /// Assembly joint child link is missing.
    MissingJointChild,
    /// Assembly joint cannot connect a link to itself.
    SelfJoint,
    /// Assembly root cannot be a child.
    RootHasParent,
    /// Assembly link has multiple parents.
    MultipleParents,
    /// Assembly contains a directed cycle.
    AssemblyCycle,
    /// Assembly links are disconnected from the root.
    DisconnectedAssembly,
    /// Compound requires at least one child.
    EmptyCompound,
    /// Compound exceeds 256 expanded shape nodes.
    TooManyCompoundNodes,
    /// Compound exceeds eight levels or 256 expanded shape nodes.
    CompoundDepthOrSize,
    /// Mesh vertices must be finite.
    NonfiniteMeshVertex,
    /// Mesh has too many vertices.
    TooManyMeshVertices,
    /// Mesh triangle index out of range.
    MeshIndexOutOfRange,
}
impl ModelError {
    /// Original diagnostic, also used by legacy string-error adapters.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::InvalidColliderMass => "collider mass or density must be finite and nonnegative",
            Self::InvalidBodyPolicy => "invalid body gravity or damping policy",
            Self::InvalidSphereRadius => "invalid sphere radius",
            Self::InvalidCylinderRadiusOrHalfHeight => "invalid cylinder radius or half height",
            Self::InvalidBoxHalfExtents => "invalid box half extents",
            Self::InvalidContactMaterial => "invalid contact material",
            Self::InvalidMassOrPrincipalInertia => "invalid mass or principal inertia",
            Self::UnrealizablePrincipalInertia => "unrealizable principal inertia",
            Self::DynamicSurface => "dynamic triangle surfaces are unsupported",
            Self::MissingColliderMass => {
                "collider-derived dynamic body requires a positive mass contribution"
            }
            Self::UnexpectedColliderMass => {
                "collider mass requires explicit collider-derived dynamic body mode"
            }
            Self::NonfiniteBodyVelocity => "nonfinite body velocity",
            Self::InvalidPose => "invalid body or collider pose",
            Self::InvalidMountPose => "invalid mount pose",
            Self::DuplicateCarrierKey => "duplicate carrier key",
            Self::DuplicateMountKey => "duplicate mount key",
            Self::MissingMountCarrier => "mount carrier is missing",
            Self::InvalidHeightfield => "invalid finite heightfield",
            Self::InvalidJointLimits => "joint limits must be finite and ordered",
            Self::FixedJointLimits => "fixed joints cannot have limits",
            Self::RevoluteLimitsOutOfRange => "revolute limits must lie within [-pi, pi]",
            Self::InvalidJointMotor => "invalid joint motor targets, gains or maximum effort",
            Self::InvalidJointFrame => "invalid joint frame",
            Self::TooFewHullPoints => "convex hull requires at least four points",
            Self::NonfiniteHullPoint => "convex hull points must be finite",
            Self::EmptyAssembly => "assembly must contain at least one link",
            Self::DuplicateAssemblyLinkId => "duplicate assembly link ID",
            Self::MissingAssemblyRoot => "assembly root link is missing",
            Self::DuplicateAssemblyJointId => "duplicate assembly joint ID",
            Self::MissingJointParent => "assembly joint parent link is missing",
            Self::MissingJointChild => "assembly joint child link is missing",
            Self::SelfJoint => "assembly joint cannot connect a link to itself",
            Self::RootHasParent => "assembly root cannot be a child",
            Self::MultipleParents => "assembly link has multiple parents",
            Self::AssemblyCycle => "assembly contains a directed cycle",
            Self::DisconnectedAssembly => "assembly links are disconnected from the root",
            Self::EmptyCompound => "compound requires at least one child",
            Self::TooManyCompoundNodes => "compound exceeds 256 expanded shape nodes",
            Self::CompoundDepthOrSize => {
                "compound exceeds eight levels or 256 expanded shape nodes"
            }
            Self::NonfiniteMeshVertex => "mesh vertices must be finite",
            Self::TooManyMeshVertices => "mesh has too many vertices",
            Self::MeshIndexOutOfRange => "mesh triangle index out of range",
        }
    }
}
impl std::fmt::Display for ModelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}
impl std::error::Error for ModelError {}

// Retain ? propagation in existing adapters while new callers use typed errors.
impl From<ModelError> for &'static str {
    fn from(error: ModelError) -> Self {
        error.as_str()
    }
}
