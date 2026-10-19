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

/// One reference-pose revolute axis. Limits are caller-owned, not inferred by IK.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RevoluteAxis {
    /// Point on the bearing line in reference body FLU metres.
    pub origin_m: [f64; 3],
    /// Unit bearing direction in the same reference frame.
    pub axis: [f64; 3],
    /// Inclusive minimum/maximum relative-to-reference rotation in radians.
    pub range_rad: [f64; 2],
}
/// Six-axis serial CAD chain using space-frame reference bearings (product of exponentials).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SerialArm {
    /// Ordered base-to-tool axes, all expressed in the same zero-pose body frame.
    pub axes: [RevoluteAxis; 6],
    /// Tool reference in the zero-pose body frame, metres and wxyz unit quaternion.
    pub tool: Pose,
}
fn rigid_pose(p: Pose) -> rapier3d_f64::math::Pose {
    let [w, x, y, z] = p.rotation_wxyz;
    rapier3d_f64::math::Pose::from_parts(
        Vector::from_array(p.translation_m),
        Rotation::from_xyzw(x, y, z, w),
    )
}
fn public_pose(p: rapier3d_f64::math::Pose) -> Pose {
    Pose {
        translation_m: p.translation.to_array(),
        rotation_wxyz: [p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z],
    }
}
impl SerialArm {
    /// Reject nonfinite geometry, non-unit bearings/quaternions and reversed limits.
    pub fn validate(&self) -> Result<(), &'static str> {
        let p = self.tool;
        if !p
            .translation_m
            .into_iter()
            .chain(p.rotation_wxyz)
            .all(f64::is_finite)
            || (p.rotation_wxyz.into_iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-8
            || self.axes.iter().any(|a| {
                !a.origin_m
                    .into_iter()
                    .chain(a.axis)
                    .chain(a.range_rad)
                    .all(f64::is_finite)
                    || (a.axis.into_iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-8
                    || a.range_rad[0] >= a.range_rad[1]
            })
        {
            return Err("invalid serial arm geometry or limits");
        }
        Ok(())
    }
    /// Zero-pose bearing frames transformed by the serial joint deltas.
    /// Call [`Self::validate`] before evaluating imported configurations.
    pub fn frames(&self, q: [f64; 6]) -> [Pose; 6] {
        let mut delta = rapier3d_f64::math::Pose::IDENTITY;
        std::array::from_fn(|i| {
            let a = self.axes[i];
            let origin = Vector::from_array(a.origin_m);
            let rotation = Rotation::from_scaled_axis(Vector::from_array(a.axis) * q[i]);
            delta *= rapier3d_f64::math::Pose::from_parts(origin - rotation * origin, rotation);
            public_pose(delta * rapier3d_f64::math::Pose::from_translation(origin))
        })
    }
    /// Cartesian tool pose at the supplied relative joint angles, in body FLU.
    pub fn tool_pose(&self, q: [f64; 6]) -> Pose {
        let frame = rigid_pose(self.frames(q)[5]);
        let rest =
            rapier3d_f64::math::Pose::from_translation(Vector::from_array(self.axes[5].origin_m));
        public_pose(frame * rest.inverse() * rigid_pose(self.tool))
    }
    /// Warm-started damped least-squares IK, preserving the requested pose.
    /// Converges within 10 micrometres and 0.1 milliradians or returns an error;
    /// never clamps the Cartesian target. No torque or collision model is implied.
    ///
    /// ```
    /// use rm_simulator_physics::{Pose, manipulator::{SerialArm,RevoluteAxis}};
    /// let joint=RevoluteAxis {origin_m:[0.;3],axis:[0.,0.,1.],range_rad:[-3.,3.]};
    /// let arm=SerialArm {axes:[joint;6],tool:Pose::at([0.2,0.,0.])};
    /// let q=arm.solve(arm.tool_pose([0.;6]),[0.;6]).unwrap();
    /// assert_eq!(q,[0.;6]);
    /// ```
    pub fn solve(&self, target: Pose, seed: [f64; 6]) -> Result<[f64; 6], &'static str> {
        self.validate()?;
        if !target
            .translation_m
            .into_iter()
            .chain(target.rotation_wxyz)
            .chain(seed)
            .all(f64::is_finite)
            || (target.rotation_wxyz.into_iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-8
        {
            return Err("invalid serial arm target or seed");
        }
        let residual = |q| {
            let p = rigid_pose(self.tool_pose(q));
            let t = rigid_pose(target);
            let position = (p.translation - t.translation) * 4.;
            let mut rotation = p.rotation * t.rotation.inverse();
            if rotation.w < 0. {
                rotation = -rotation;
            }
            let vector = Vector::new(rotation.x, rotation.y, rotation.z);
            let n = vector.length();
            let angle = 2. * n.atan2(rotation.w);
            let v = if n > 1e-12 {
                vector * (angle / n)
            } else {
                vector * 2.
            };
            [position.x, position.y, position.z, v.x, v.y, v.z]
        };
        let norm = |r: [f64; 6]| r.into_iter().map(|v| v * v).sum::<f64>();
        let mut q = std::array::from_fn(|i| {
            seed[i].clamp(self.axes[i].range_rad[0], self.axes[i].range_rad[1])
        });
        for _ in 0..120 {
            let r = residual(q);
            if r[..3].iter().map(|v| v * v).sum::<f64>() < 1.6e-9
                && r[3..].iter().map(|v| v * v).sum::<f64>() < 1e-8
            {
                return Ok(q);
            }
            let jac: [[f64; 6]; 6] = std::array::from_fn(|col| {
                let mut trial = q;
                trial[col] += 1e-5;
                let b = residual(trial);
                std::array::from_fn(|row| (b[row] - r[row]) / 1e-5)
            });
            let mut system = [[0.; 7]; 6];
            for i in 0..6 {
                for j in 0..6 {
                    system[i][j] = (0..6).map(|k| jac[i][k] * jac[j][k]).sum::<f64>()
                        + if i == j { 1e-6 } else { 0. };
                }
                system[i][6] = -(0..6).map(|k| jac[i][k] * r[k]).sum::<f64>();
            }
            let step = solve_six(system).ok_or("singular serial IK step")?;
            let scale = (0.15 / step.into_iter().map(f64::abs).fold(0., f64::max)).min(1.);
            let mut improved = false;
            for factor in [1., 0.5, 0.25, 0.125, 0.0625] {
                let trial = std::array::from_fn(|i| {
                    (q[i] + step[i] * scale * factor)
                        .clamp(self.axes[i].range_rad[0], self.axes[i].range_rad[1])
                });
                if norm(residual(trial)) < norm(r) {
                    q = trial;
                    improved = true;
                    break;
                }
            }
            if !improved {
                break;
            }
        }
        Err("serial arm target did not converge within its joint limits")
    }
}
fn solve_six(mut a: [[f64; 7]; 6]) -> Option<[f64; 6]> {
    for col in 0..6 {
        let pivot = (col..6).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[pivot][col].abs() < 1e-15 {
            return None;
        }
        a.swap(col, pivot);
        let divisor = a[col][col];
        for value in &mut a[col][col..] {
            *value /= divisor;
        }
        let pivot_row = a[col];
        for (i, row) in a.iter_mut().enumerate() {
            if i != col {
                let factor = row[col];
                for (value, pivot) in row[col..].iter_mut().zip(&pivot_row[col..]) {
                    *value -= factor * pivot;
                }
            }
        }
    }
    Some(std::array::from_fn(|i| a[i][6]))
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
    fn serial_fixture() -> SerialArm {
        let origins = [
            [0., 0., 0.],
            [0., 0., 0.2],
            [0.4, 0., 0.2],
            [0.7, 0., 0.2],
            [0.7, 0., 0.2],
            [0.8, 0., 0.2],
        ];
        let axes = [
            [0., 0., 1.],
            [0., 1., 0.],
            [0., 1., 0.],
            [1., 0., 0.],
            [0., 1., 0.],
            [1., 0., 0.],
        ];
        SerialArm {
            axes: std::array::from_fn(|i| RevoluteAxis {
                origin_m: origins[i],
                axis: axes[i],
                range_rad: [-3., 3.],
            }),
            tool: Pose::at([0.9, 0., 0.2]),
        }
    }
    #[test]
    fn serial_ik_tracks_position_orientation_and_antipodal_quaternions() {
        let arm = serial_fixture();
        let desired = [0.2, 0.4, -0.8, 0.3, 0.7, -0.2];
        let target = arm.tool_pose(desired);
        let q = arm.solve(target, desired.map(|v| v + 0.05)).unwrap();
        let actual = arm.tool_pose(q);
        assert!(
            Vector::from_array(actual.translation_m)
                .distance(Vector::from_array(target.translation_m))
                < 1e-5
        );
        let mut antipodal = target;
        antipodal.rotation_wxyz = target.rotation_wxyz.map(|v| -v);
        assert_eq!(arm.solve(antipodal, q).unwrap(), q);
        assert!(arm.solve(Pose::at([10., 0., 0.]), q).is_err());
    }
    #[test]
    fn serial_frames_follow_reference_axis_lines() {
        let arm = serial_fixture();
        let angles = [0.1, 0.2, -0.3, 0.4, -0.5, 0.6];
        let mut delta = rapier3d_f64::math::Pose::IDENTITY;
        for (i, a) in arm.axes.into_iter().enumerate() {
            let p = Vector::from_array(a.origin_m);
            let r = Rotation::from_scaled_axis(Vector::from_array(a.axis) * angles[i]);
            delta *= rapier3d_f64::math::Pose::from_parts(p - r * p, r);
            let f = rigid_pose(arm.frames(angles)[i]);
            assert!(
                (f.translation
                    - (delta * rapier3d_f64::math::Pose::from_translation(p)).translation)
                    .length()
                    < 1e-12
            );
        }
        let mut invalid = arm;
        invalid.axes[0].axis = [0.; 3];
        assert!(invalid.validate().is_err());
    }
}
