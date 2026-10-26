// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared backend conversion helpers, without participant or scoring concepts.
use crate::Pose;
use rapier3d_f64::prelude::{Pose as Pose3, Rotation, Vector};

/// Convert an FLU array in metres to a Rapier vector.
pub(crate) fn vector(v: [f64; 3]) -> Vector {
    Vector::new(v[0], v[1], v[2])
}
pub(crate) fn pose(p: Pose) -> Pose3 {
    let [w, x, y, z] = p.rotation_wxyz;
    Pose3::from_parts(
        vector(p.translation_m),
        Rotation::from_xyzw(x, y, z, w).normalize(),
    )
}
#[cfg(feature = "rapier-extension")]
pub(crate) fn finite(v: [f64; 3]) -> bool {
    v.iter().all(|x| x.is_finite())
}
pub(crate) fn pose_is_valid(p: Pose) -> bool {
    p.validate_with(sim_engine::pose::PoseTolerance::Runtime)
        .is_ok()
}
