// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Standalone arena geometry and spatial queries, without a match authority.
//! Callers supply placement, dimensions and rotor angles; no rewards, HP,
//! timers or automatic scheduling are owned here.
//!
//! ```
//! use rm_sim_physics::{Pose, WorldPhysics, arena::{TunnelModel, ZoneFootprint}};
//! let mut physics = WorldPhysics::new(&[], 0.);
//! TunnelModel { pose: Pose::at([0.; 3]), opening_m: [2., 1., 0.25],
//!     wall_thickness_m: 0.1 }.install(&mut physics)?;
//! let zone = ZoneFootprint { polygon_m: &[[-1., -0.5], [1., -0.5],
//!     [1., 0.5], [-1., 0.5]], floor_m: 0., height_m: 0.25 };
//! assert!(zone.contains([0., 0., 0.1]));
//! # Ok::<(), &'static str>(())
//! ```
use crate::{Pose, WorldPhysics};
use serde::{Deserialize, Serialize};

/// Borrowed polygon and inclusive vertical band, in world FLU metres.
/// Occupancy uses the existing ray-crossing policy; polygon edges are not
/// universally inclusive. This is a simulator antenna approximation, not RF.
#[derive(Clone, Copy, Debug)]
pub struct ZoneFootprint<'a> {
    /// Footprint vertices in either winding.
    pub polygon_m: &'a [[f64; 2]],
    /// Surface height in metres.
    pub floor_m: f64,
    /// Accepted height above the surface, in metres.
    pub height_m: f64,
}
impl ZoneFootprint<'_> {
    /// Test a body/antenna centre without allocation or mutable state.
    /// Non-finite positions and footprints with fewer than three vertices miss.
    pub fn contains(&self, p: [f64; 3]) -> bool {
        if !p.into_iter().all(f64::is_finite)
            || !self.floor_m.is_finite()
            || !self.height_m.is_finite()
            || self.height_m < 0.
            || !(self.floor_m + self.height_m).is_finite()
            || self.polygon_m.len() < 3
            || p[2] < self.floor_m
            || p[2] > self.floor_m + self.height_m
        {
            return false;
        }
        let mut inside = false;
        for (a, b) in self
            .polygon_m
            .iter()
            .zip(self.polygon_m.iter().cycle().skip(1))
        {
            if (a[1] > p[1]) != (b[1] > p[1])
                && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
            {
                inside = !inside;
            }
        }
        inside
    }
}

/// One fitted detector; dimensions are measured from the source CAD, not artwork.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Detector {
    /// Face at rest, in world FLU metres, +x outward.
    pub pose: Pose,
    /// Effective rectangular half-width/half-height in metres.
    pub half_m: [f64; 2],
}
/// Source-fitted Outpost geometry, independent of team ownership and match state.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OutpostModel {
    /// World FLU rotor pivot in metres.
    pub pivot_m: [f64; 3],
    /// Unit rotor axis in world FLU, including the source tower tilt.
    pub axis: [f64; 3],
    /// Three source detector faces at rest.
    pub detectors: Vec<Detector>,
}
impl OutpostModel {
    /// Validate an independently loaded source model: three detector faces,
    /// finite pivot, unit axis and positive rectangular half extents.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.detectors.len() != 3
            || !self.pivot_m.iter().chain(&self.axis).all(|v| v.is_finite())
            || (self.axis.iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-6
            || self.detectors.iter().any(|d| {
                !crate::projectile::pose_is_valid(d.pose)
                    || d.half_m.iter().any(|v| !v.is_finite() || *v <= 0.)
            })
        {
            return Err("invalid source Outpost model");
        }
        Ok(())
    }
    /// Resolve one detector at a caller-supplied finite angle in radians.
    /// Invalid angles or detector indices return `None`; validate source geometry
    /// before use. Rotation speed, stops and destruction belong to the caller.
    pub fn detector_pose(&self, detector: usize, angle_rad: f64) -> Option<Pose> {
        if !angle_rad.is_finite() {
            return None;
        }
        self.detectors
            .get(detector)
            .map(|d| Self::rotate_pose(d.pose, self.pivot_m, self.axis, angle_rad))
    }
    /// Rotate a source pose around a finite pivot and unit FLU axis.
    /// Inputs must be validated by the caller; preserves the source pose frame.
    pub fn rotate_pose(pose: Pose, pivot: [f64; 3], axis: [f64; 3], angle: f64) -> Pose {
        let half = angle / 2.;
        let q = [
            half.cos(),
            axis[0] * half.sin(),
            axis[1] * half.sin(),
            axis[2] * half.sin(),
        ];
        let delta = std::array::from_fn(|i| pose.translation_m[i] - pivot[i]);
        let rotated = crate::motion::outpost::rotate(q, delta);
        Pose {
            translation_m: std::array::from_fn(|i| pivot[i] + rotated[i]),
            rotation_wxyz: {
                let [a, b, c, d] = q;
                let [w, x, y, z] = pose.rotation_wxyz;
                [
                    a * w - b * x - c * y - d * z,
                    a * x + b * w + c * z - d * y,
                    a * y - b * z + c * w + d * x,
                    a * z + b * y - c * x + d * w,
                ]
            },
        }
    }
}

/// A deliberately approximate, open-ended rectangular tunnel test fixture.
/// Source CAD remains the authority for competition geometry. Local +x runs
/// along the tunnel; the pose is the opening's floor-centre at mid-length.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TunnelModel {
    /// World FLU placement, with a unit quaternion.
    pub pose: Pose,
    /// Clear passage length, width and height, in metres.
    pub opening_m: [f64; 3],
    /// Roof and side-wall thickness in metres; no floor slab is added.
    pub wall_thickness_m: f64,
}
impl TunnelModel {
    /// Resolve roof and two side walls as world poses and full box sizes.
    /// Validation completes before any geometry can be installed.
    pub fn collision_boxes(&self) -> Result<[(Pose, [f64; 3]); 3], &'static str> {
        if !crate::projectile::pose_is_valid(self.pose)
            || self.opening_m.iter().any(|v| !v.is_finite() || *v <= 0.)
            || !self.wall_thickness_m.is_finite()
            || self.wall_thickness_m <= 0.
        {
            return Err("invalid tunnel fixture dimensions or pose");
        }
        let [length, width, height] = self.opening_m;
        let t = self.wall_thickness_m;
        let local = [
            ([0., 0., height + t / 2.], [length, width + 2. * t, t]),
            ([0., -(width + t) / 2., height / 2.], [length, t, height]),
            ([0., (width + t) / 2., height / 2.], [length, t, height]),
        ];
        let boxes = local.map(|(offset, size)| {
            let delta = crate::motion::outpost::rotate(self.pose.rotation_wxyz, offset);
            (
                Pose {
                    translation_m: std::array::from_fn(|i| self.pose.translation_m[i] + delta[i]),
                    rotation_wxyz: self.pose.rotation_wxyz,
                },
                size,
            )
        });
        if boxes
            .iter()
            .any(|(p, s)| !p.translation_m.iter().chain(s).all(|v| v.is_finite()))
        {
            return Err("tunnel fixture geometry overflow");
        }
        Ok(boxes)
    }
    /// Install the validated three-box fixture into the caller's physical world.
    pub fn install(&self, physics: &mut WorldPhysics) -> Result<(), &'static str> {
        for (pose, size) in self.collision_boxes()? {
            physics.add_static_box(pose, size);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn footprints_preserve_height_winding_and_concavity() {
        let polygon = [[0., 0.], [2., 0.], [2., 1.], [1., 1.], [1., 2.], [0., 2.]];
        let zone = ZoneFootprint {
            polygon_m: &polygon,
            floor_m: 0.2,
            height_m: 0.45,
        };
        assert!(zone.contains([0.5, 1.5, 0.2]));
        assert!(zone.contains([0.5, 1.5, 0.65]));
        assert!(!zone.contains([1.5, 1.5, 0.3]));
        assert!(!zone.contains([0.5, 0.5, 0.199]));
        assert!(!zone.contains([0.5, 0.5, 0.651]));
        assert!(!zone.contains([f64::NAN, 0.5, 0.3]));
        let reverse: Vec<_> = polygon.into_iter().rev().collect();
        let reversed = ZoneFootprint {
            polygon_m: &reverse,
            ..zone
        };
        for x in 0..20 {
            for y in 0..20 {
                let p = [f64::from(x) / 10. + 0.03, f64::from(y) / 10. + 0.03, 0.3];
                assert_eq!(zone.contains(p), reversed.contains(p));
            }
        }
        assert!(
            !ZoneFootprint {
                floor_m: f64::NAN,
                ..zone
            }
            .contains([0.5, 0.5, 0.3])
        );
    }

    #[test]
    fn source_rotor_preserves_distances_and_detector_frames() {
        let model = OutpostModel {
            pivot_m: [1., 2., 3.],
            axis: [0., 0., 1.],
            detectors: (0..3)
                .map(|i| Detector {
                    pose: Pose::at([2., 2., 3. + f64::from(i)]),
                    half_m: [0.1, 0.05],
                })
                .collect(),
        };
        model.validate().unwrap();
        let p = model.detector_pose(0, std::f64::consts::FRAC_PI_2).unwrap();
        for (actual, expected) in p.translation_m.into_iter().zip([1., 3., 3.]) {
            assert!((actual - expected).abs() < 1e-12);
        }
        let forward = crate::motion::outpost::rotate(p.rotation_wxyz, [1., 0., 0.]);
        assert!(forward[0].abs() < 1e-12);
        assert!((forward[1] - 1.).abs() < 1e-12);
        assert!(model.detector_pose(3, 0.).is_none());
        assert!(model.detector_pose(0, f64::NAN).is_none());
        let mut invalid = model;
        invalid.axis = [0.; 3];
        assert!(invalid.validate().is_err());
    }

    #[test]
    fn tunnel_is_open_ended_and_blocks_roof_and_walls_after_placement() {
        let tunnel = TunnelModel {
            pose: Pose::at([2., 3., 0.]),
            opening_m: [2., 1., 0.25],
            wall_thickness_m: 0.1,
        };
        let mut physics = WorldPhysics::new(&[], 0.);
        tunnel.install(&mut physics).unwrap();
        let geometry = physics.static_geometry_snapshot(true);
        assert!(geometry.segment_clear([0., 3., 0.1], [4., 3., 0.1]));
        assert!(!geometry.segment_clear([2., 3., 0.1], [2., 3., 0.5]));
        assert!(!geometry.segment_clear([2., 3., 0.1], [2., 4., 0.1]));
        let before = geometry.clone().into_triangles().1.len();
        assert!(
            TunnelModel {
                wall_thickness_m: f64::NAN,
                ..tunnel
            }
            .install(&mut physics)
            .is_err()
        );
        assert_eq!(
            physics
                .static_geometry_snapshot(true)
                .into_triangles()
                .1
                .len(),
            before
        );
    }
}
