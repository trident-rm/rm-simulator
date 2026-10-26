// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Immutable validated CPU query records without solver handles or resource ownership.
//! Backends execute batches against caller-selected coherent captures. Hit ordinals
//! are capture-local; device buffers, filtering and scheduling belong to adapters.
#[path = "queries_error.rs"]
mod queries_error;
pub use queries_error::QueryError;

/// Validated finite metric ray with a normalized, dimensionless FLU direction.
/// Fields are immutable so batch execution needs no repeated validation.
/// Range is inclusive and strictly positive; an origin inside a solid hits at zero.
/// Construction allocates no storage and does not query any scene.
///
/// ```
/// use sim_engine::queries::RayQuery;
/// let ray = RayQuery::new([0., 0., 1.], [20., 0., 0.], 5.)?;
/// assert_eq!(ray.direction(), [1., 0., 0.]);
/// assert_eq!(ray.max_distance_m(), 5.);
/// assert!(RayQuery::new([0.; 3], [0.; 3], 5.).is_err());
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayQuery {
    origin_m: [f64; 3],
    direction: [f64; 3],
    max_distance_m: f64,
}
impl RayQuery {
    /// Validate finite origin/direction and positive finite range in metres.
    /// Direction scale is irrelevant. Scaling before normalization avoids
    /// overflow and underflow for extreme finite nonzero directions.
    ///
    /// # Errors
    /// Rejects nonfinite origins/directions, zero directions, and nonpositive or nonfinite range.
    pub fn new(
        origin_m: [f64; 3],
        direction: [f64; 3],
        max_distance_m: f64,
    ) -> Result<Self, QueryError> {
        if !origin_m.into_iter().chain(direction).all(f64::is_finite)
            || !max_distance_m.is_finite()
            || max_distance_m <= 0.
        {
            return Err(QueryError::InvalidRay);
        }
        let direction = glam::DVec3::new(direction[0], direction[1], direction[2]);
        let scale = direction.abs().max_element();
        if scale == 0. {
            return Err(QueryError::ZeroDirection);
        }
        Ok(Self {
            origin_m,
            direction: (direction / scale).normalize().to_array(),
            max_distance_m,
        })
    }
    /// World FLU ray origin, metres.
    #[inline]
    pub fn origin_m(&self) -> [f64; 3] {
        self.origin_m
    }
    /// Normalized, dimensionless world FLU direction.
    #[inline]
    pub fn direction(&self) -> [f64; 3] {
        self.direction
    }
    /// Inclusive positive metric range, metres.
    #[inline]
    pub fn max_distance_m(&self) -> f64 {
        self.max_distance_m
    }
}

/// Closest surface returned by a metric ray query over one geometry capture.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    /// Distance along the normalized ray, in metres, including zero if inside a solid.
    pub distance_m: f64,
    /// World FLU intersection point, metres.
    pub position_m: [f64; 3],
    /// World FLU surface normal; may be zero/unreliable for an origin inside a solid.
    pub normal: [f64; 3],
    /// Shape ordinal in this capture, stable only while that capture is unchanged.
    /// This is not a robot identity or persistent semantic part ID.
    pub shape_index: usize,
}
