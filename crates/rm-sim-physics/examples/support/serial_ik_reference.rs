// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
// Frozen baseline from manipulator.rs at c642c0cb9ed5e894e1b9a64c40055ae6e4366aaa.
// Used only for differential tests and timing, never production planning.
#![allow(dead_code)]
use rapier3d_f64::math::{Rotation, Vector};
use rm_sim_physics::{Pose, manipulator::SerialArm};
fn rigid_pose(p: Pose) -> rapier3d_f64::math::Pose {
    let [w, x, y, z] = p.rotation_wxyz;
    rapier3d_f64::math::Pose::from_parts(
        Vector::from_array(p.translation_m),
        Rotation::from_xyzw(x, y, z, w),
    )
}
pub fn tool_pose(arm: &SerialArm, q: [f64; 6]) -> Pose {
    let frame = rigid_pose(arm.frames(q)[5]);
    let rest = rapier3d_f64::math::Pose::from_translation(Vector::from_array(arm.axes[5].origin_m));
    let p = frame * rest.inverse() * rigid_pose(arm.tool);
    Pose {
        translation_m: p.translation.to_array(),
        rotation_wxyz: [p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z],
    }
}
pub fn solve(arm: &SerialArm, target: Pose, seed: [f64; 6]) -> Result<[f64; 6], &'static str> {
    arm.validate()?;
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
        let p = rigid_pose(tool_pose(arm, q));
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
    let mut q =
        std::array::from_fn(|i| seed[i].clamp(arm.axes[i].range_rad[0], arm.axes[i].range_rad[1]));
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
                    .clamp(arm.axes[i].range_rad[0], arm.axes[i].range_rad[1])
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
