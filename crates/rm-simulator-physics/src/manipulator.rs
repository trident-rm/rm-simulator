// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Cartesian workbench arm inverse kinematics. This is a prescribed position
//! actuator, without torque, payload, self-collision or source-donor calibration.
use crate::Pose;
use rapier3d_f64::math::{Rotation, Vector};

/// A yaw bearing and planar serial two-link arm, followed by a free wrist.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Arm {
    /// Shoulder bearing in body FLU metres.
    pub shoulder_m: [f64; 3],
    /// Upper-arm and forearm lengths in metres, both strictly positive.
    pub lengths_m: [f64; 2],
}
/// Exact elbow-up solution; all points are body-local FLU metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Solution {
    /// Shoulder position, metres.
    pub shoulder_m: [f64; 3],
    /// Solved elbow position, metres.
    pub elbow_m: [f64; 3],
    /// Solved gripper pose; it equals the requested pose without projection.
    pub tool: Pose,
    /// Yaw, shoulder elevation and relative elbow bend, radians.
    pub joints_rad: [f64; 3],
    /// Wrist rotation relative to the forearm, wxyz unit quaternion.
    pub wrist_wxyz: [f64; 4],
}
impl Arm {
    /// Solve an exact Cartesian pose. Unreachable/nonfinite targets and invalid
    /// geometry/quaternions are rejected; never silently shorten or teleport.
    ///
    /// ```
    /// use rm_simulator_physics::{Pose, manipulator::Arm};
    /// let arm = Arm { shoulder_m: [0.; 3], lengths_m: [0.4; 2] };
    /// let solution = arm.solve(Pose::at([0.5, 0., 0.2])).unwrap();
    /// assert_eq!(solution.tool.translation_m, [0.5, 0., 0.2]);
    /// assert!(arm.solve(Pose::at([1., 0., 0.])).is_err());
    /// ```
    pub fn solve(self, tool: Pose) -> Result<Solution, &'static str> {
        if !self
            .shoulder_m
            .into_iter()
            .chain(tool.translation_m)
            .all(f64::is_finite)
            || self.lengths_m.iter().any(|l| !l.is_finite() || *l <= 0.)
            || !tool.rotation_wxyz.iter().all(|v| v.is_finite())
            || (tool.rotation_wxyz.iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-8
        {
            return Err("invalid Cartesian arm geometry or target pose");
        }
        let [x, y, z] = std::array::from_fn(|i| tool.translation_m[i] - self.shoulder_m[i]);
        let r = x.hypot(y);
        let distance = r.hypot(z);
        let [a, b] = self.lengths_m;
        if distance <= 1e-9 || distance > a + b || distance < (a - b).abs() {
            return Err("Cartesian arm target is unreachable");
        }
        let yaw = y.atan2(x);
        let elbow = -((distance * distance - a * a - b * b) / (2. * a * b))
            .clamp(-1., 1.)
            .acos();
        let shoulder = z.atan2(r) - (b * elbow.sin()).atan2(a + b * elbow.cos());
        let elbow_m = [
            self.shoulder_m[0] + a * shoulder.cos() * yaw.cos(),
            self.shoulder_m[1] + a * shoulder.cos() * yaw.sin(),
            self.shoulder_m[2] + a * shoulder.sin(),
        ];
        let forearm = Rotation::from_scaled_axis(Vector::new(0., 0., yaw))
            * Rotation::from_scaled_axis(Vector::new(0., -(shoulder + elbow), 0.));
        let [w, x, y, z] = tool.rotation_wxyz;
        let wrist = forearm.inverse() * Rotation::from_xyzw(x, y, z, w);
        Ok(Solution {
            shoulder_m: self.shoulder_m,
            elbow_m,
            tool,
            joints_rad: [yaw, shoulder, elbow],
            wrist_wxyz: [wrist.w, wrist.x, wrist.y, wrist.z],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn exact_lengths_and_wrist_orientation_across_workspace() {
        let arm = Arm {
            shoulder_m: [0.1, -0.2, 0.3],
            lengths_m: [0.4, 0.35],
        };
        for p in [
            [0.5, 0., 0.5],
            [-0.3, 0.1, 0.4],
            [0.1, -0.2, 0.9],
            [0.1, -0.2, -0.3],
        ] {
            let tool = Pose {
                translation_m: p,
                rotation_wxyz: [0.5; 4],
            };
            let s = arm.solve(tool).unwrap();
            for (a, b, l) in [(s.shoulder_m, s.elbow_m, 0.4), (s.elbow_m, p, 0.35)] {
                let distance = a
                    .into_iter()
                    .zip(b)
                    .map(|(x, y)| (x - y).powi(2))
                    .sum::<f64>()
                    .sqrt();
                assert!((distance - l).abs() < 1e-12);
            }
            let [yaw, shoulder, elbow] = s.joints_rad;
            let [w, x, y, z] = s.wrist_wxyz;
            let q = Rotation::from_scaled_axis(Vector::new(0., 0., yaw))
                * Rotation::from_scaled_axis(Vector::new(0., -(shoulder + elbow), 0.))
                * Rotation::from_xyzw(x, y, z, w);
            assert!((q.w.abs() - 0.5).abs() < 1e-12);
            assert!((q.x.abs() - 0.5).abs() < 1e-12);
        }
    }
    #[test]
    fn rejects_unreachable_and_invalid_targets() {
        let arm = Arm {
            shoulder_m: [0.; 3],
            lengths_m: [0.4, 0.2],
        };
        for p in [[0.; 3], [0.1, 0., 0.], [0.7, 0., 0.], [f64::NAN, 0., 0.]] {
            assert!(arm.solve(Pose::at(p)).is_err());
        }
        assert!(
            arm.solve(Pose {
                rotation_wxyz: [0.; 4],
                ..Pose::at([0.3, 0., 0.])
            })
            .is_err()
        );
    }
}
