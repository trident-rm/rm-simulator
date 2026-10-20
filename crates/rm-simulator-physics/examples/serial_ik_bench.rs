// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Paired actual-chain numerical IK benchmark; geometry supplied as 55 numbers.
//! No CAD, asset identity, spherical wrist or joint limits are invented here.
#[path = "support/serial_ik_reference.rs"]
mod reference;
use rm_simulator_physics::{
    Pose,
    manipulator::{RevoluteAxis, SerialArm},
    manipulator_plan::ik_candidates,
};
use std::{hint::black_box, time::Instant};

fn random(state: &mut u64) -> f64 {
    *state = state
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    (*state >> 11) as f64 / (1u64 << 53) as f64
}
fn roots(arm: &SerialArm, t: Pose, seeds: &[[f64; 6]]) -> Vec<[f64; 6]> {
    let mut out: Vec<[f64; 6]> = vec![];
    for &seed in seeds {
        if out.len() >= 4 {
            break;
        }
        if let Ok(q) = reference::solve(arm, t, seed)
            && !out.iter().any(|p| {
                p.iter()
                    .zip(q)
                    .map(|(a, b)| (a - b).abs())
                    .fold(0., f64::max)
                    < 0.001
            })
        {
            out.push(q);
        }
    }
    out
}
fn validate(arm: &SerialArm, target: Pose, q: [f64; 6]) {
    assert!(
        q.iter()
            .zip(arm.axes)
            .all(|(v, a)| v.is_finite() && *v >= a.range_rad[0] && *v <= a.range_rad[1])
    );
    let p = arm.tool_pose(q);
    assert!(
        p.translation_m
            .iter()
            .zip(target.translation_m)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt()
            < 1e-5
    );
    let dot = p
        .rotation_wxyz
        .iter()
        .zip(target.rotation_wxyz)
        .map(|(a, b)| a * b)
        .sum::<f64>()
        .abs()
        .clamp(0., 1.);
    assert!(2. * dot.acos() < 1e-4 + 1e-9);
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .ok_or("usage: serial_ik_bench ARM.txt")?;
    let values = std::fs::read_to_string(path)?
        .split_whitespace()
        .map(str::parse::<f64>)
        .collect::<Result<Vec<_>, _>>()?;
    if values.len() != 55 {
        return Err("expected 6*(origin3+axis3+limits2)+tool xyz3+wxyz4".into());
    }
    let arm = SerialArm {
        axes: std::array::from_fn(|i| {
            let a = &values[i * 8..];
            RevoluteAxis {
                origin_m: a[..3].try_into().unwrap(),
                axis: a[3..6].try_into().unwrap(),
                range_rad: a[6..8].try_into().unwrap(),
            }
        }),
        tool: Pose {
            translation_m: values[48..51].try_into()?,
            rotation_wxyz: values[51..55].try_into()?,
        },
    };
    arm.validate()?;
    let mut rng = 20261020;
    let mut cases = vec![];
    for i in 0..768 {
        let q = std::array::from_fn(|j| {
            let [lo, hi] = arm.axes[j].range_rad;
            lo + (0.05 + 0.9 * random(&mut rng)) * (hi - lo)
        });
        let mut t = reference::tool_pose(&arm, q);
        let seed = if i % 3 == 0 {
            std::array::from_fn(|j| {
                (q[j] + (random(&mut rng) - 0.5) * 0.2)
                    .clamp(arm.axes[j].range_rad[0], arm.axes[j].range_rad[1])
            })
        } else {
            std::array::from_fn(|j| {
                let [lo, hi] = arm.axes[j].range_rad;
                lo + random(&mut rng) * (hi - lo)
            })
        };
        if i % 3 == 2 {
            t.translation_m[0] += 3.;
        }
        if i % 11 == 0 {
            t.rotation_wxyz = t.rotation_wxyz.map(|x| -x);
        }
        cases.push((t, seed));
    }
    // Warm both implementations, then rotate order over five timed repeats.
    for &(t, s) in &cases[..16] {
        let _ = black_box(reference::solve(&arm, t, s));
        let _ = black_box(arm.solve(t, s));
    }
    let mut sums = [0.; 2];
    let mut success = [0; 2];
    let mut changed_outcomes = 0;
    let mut identical = true;
    let mut equal = 0;
    for repeat in 0..5 {
        let mut outputs = [vec![], vec![]];
        for m in if repeat % 2 == 0 { [0, 1] } else { [1, 0] } {
            let begin = Instant::now();
            outputs[m] = cases
                .iter()
                .map(|&(t, s)| {
                    if m == 0 {
                        reference::solve(&arm, t, s)
                    } else {
                        arm.solve(t, s)
                    }
                })
                .collect();
            black_box(&outputs[m]);
            sums[m] += begin.elapsed().as_secs_f64();
        }
        for ((_, b), (target, _)) in outputs[0].iter().zip(&outputs[1]).zip(&cases) {
            if let Ok(q) = b {
                validate(&arm, *target, *q);
            }
        }
        identical &= outputs[0] == outputs[1];
        if repeat == 0 {
            equal = outputs[0].len();
            success = std::array::from_fn(|i| outputs[i].iter().filter(|x| x.is_ok()).count());
            changed_outcomes = outputs[0]
                .iter()
                .zip(&outputs[1])
                .filter(|(a, b)| a.is_ok() != b.is_ok())
                .count();
        }
    }
    let seeds: Vec<[f64; 6]> = (0..16)
        .map(|_| {
            std::array::from_fn(|j| {
                let [lo, hi] = arm.axes[j].range_rad;
                lo + random(&mut rng) * (hi - lo)
            })
        })
        .collect();
    let mut root_s = [0.; 2];
    let mut found = 0;
    for repeat in 0..3 {
        let mut outputs = [vec![], vec![]];
        for m in if repeat % 2 == 0 { [0, 1] } else { [1, 0] } {
            let begin = Instant::now();
            outputs[m] = cases
                .iter()
                .step_by(24)
                .map(|(t, _)| {
                    if m == 0 {
                        roots(&arm, *t, &seeds)
                    } else {
                        ik_candidates(&arm, *t, seeds.iter().copied(), 4)
                    }
                })
                .collect();
            black_box(&outputs[m]);
            root_s[m] += begin.elapsed().as_secs_f64();
        }
        identical &= outputs[0] == outputs[1];
        for (roots, (target, _)) in outputs[1].iter().zip(cases.iter().step_by(24)) {
            for q in roots {
                validate(&arm, *target, *q);
            }
        }
        found = outputs[1].iter().map(Vec::len).sum();
    }
    println!(
        "{{\"single_seed_cases\":{equal},\"baseline_successes\":{},\"optimized_successes\":{},\"changed_outcomes\":{changed_outcomes},\"single_seed_repeats\":5,\"baseline_s\":{},\"optimized_s\":{},\"speedup\":{},\"multi_seed_targets\":32,\"multi_seed_repeats\":3,\"multi_seed_roots\":{found},\"baseline_multi_s\":{},\"optimized_multi_s\":{},\"multi_speedup\":{},\"all_results_identical\":{identical},\"optimized_solutions_valid\":true}}",
        success[0],
        success[1],
        sums[0],
        sums[1],
        sums[0] / sums[1],
        root_s[0],
        root_s[1],
        root_s[0] / root_s[1]
    );
    Ok(())
}
