// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! UC §5.4.3, Table 5-10 and Figure 5-10: seeded assembly pose candidates.
//!
//! These are equipment OXYZ coordinates, never world Euler angles. Sampling
//! angles uniformly is a simulator policy; the manual specifies ranges, not a
//! probability distribution. A CAD adapter must reject structural overhang and
//! check positioning-arm reach separately, without silently changing a sample.
use serde::{Deserialize, Serialize};
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

/// One requested core pose, before insertion, lifting or P/Q manipulation.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TargetPose {
    /// Front assembly-post centre E in equipment OXYZ, metres.
    pub position_m: [f64; 3],
    /// Azimuth of the shaft normal about +Z from +X, radians.
    pub theta_rad: f64,
    /// Polar angle of the shaft normal from +Z, radians.
    pub phi_rad: f64,
    /// Right-handed roll about the outward shaft normal, radians.
    pub alpha_rad: f64,
}
impl TargetPose {
    /// Manual idle pose (§5.4.3), outside the selectable assembly-position range.
    pub const INITIAL: Self = Self {
        position_m: [-0.2, 0.2, 0.8],
        theta_rad: 0.,
        phi_rad: FRAC_PI_2,
        alpha_rad: 0.,
    };
    /// Row-major rotation, with local +X the outward shaft normal.
    ///
    /// This is `Rz(theta) Ry(phi−π/2) Rx(alpha)`. The roll-zero convention
    /// continues the upright difficulty-one frame to the pole, where azimuth
    /// still determines its tangent axes. It never reinterprets φ as pitch.
    ///
    /// ```
    /// use rm_sim_gameplay::rmuc_2027::assembly_pose::TargetPose;
    /// let r = TargetPose::INITIAL.rotation_matrix();
    /// assert!((r[0][0] - 1.).abs() < 1e-12); // horizontal normal
    /// assert!(r[2][0].abs() < 1e-12);
    /// ```
    pub fn rotation_matrix(self) -> [[f64; 3]; 3] {
        let (s, c) = self.theta_rad.sin_cos();
        let (sp, cp) = self.phi_rad.sin_cos();
        let (sa, ca) = self.alpha_rad.sin_cos();
        [
            [c * sp, -s * ca - c * cp * sa, s * sa - c * cp * ca],
            [s * sp, c * ca - s * cp * sa, -c * sa - s * cp * ca],
            [cp, sp * sa, sp * ca],
        ]
    }
}
/// Shared requested targets for either team's equipment-local frame.
///
/// Same seed, selection index and difficulty produce identical targets for Red
/// and Blue (§5.4.3). Generate a new selection only when rules permit movement;
/// keep the returned value frozen throughout insertion and retries.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Targets {
    /// Requested difficulty, 1–4.
    pub difficulty: u8,
    /// Round seed, for reproduction and restore.
    pub seed: u64,
    /// Explicit selection number, independent of host time or sample rejection.
    pub selection: u64,
    /// One target below difficulty four; two at difficulty four.
    pub cores: Vec<TargetPose>,
    /// Q targets in radians, one per participating core, within ±π/2.
    pub q_rad: Vec<f64>,
}
impl Targets {
    /// Sample Table 5-10 candidates with deterministic SplitMix64 bits.
    ///
    /// Level four shares x/z; the second y lies up to 400 mm
    /// below the first (§5.4.3). Independent orientations are a simulator policy where no paired angular
    /// relationship is specified. Independent Q targets follow the manual's random
    /// rotation requirement. No unstated minimum core separation is enforced.
    /// Candidates may be physically infeasible and must then be reported as such.
    pub fn sample(seed: u64, selection: u64, difficulty: u8) -> Result<Self, &'static str> {
        if !(1..=4).contains(&difficulty) {
            return Err("assembly pose difficulty must be 1–4");
        }
        let mut state = seed
            ^ selection.wrapping_mul(0x9e3779b97f4a7c15)
            ^ u64::from(difficulty).wrapping_mul(0xd1b54a32d192ed03);
        let mut uniform = |lo: f64, hi: f64| {
            state = state.wrapping_add(0x9e3779b97f4a7c15);
            let mut z = state;
            z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
            z ^= z >> 31;
            lo + (hi - lo) * ((z >> 11) as f64 / ((1u64 << 53) as f64))
        };
        let mut pose = TargetPose {
            position_m: [uniform(-0.1, 0.), uniform(0.1, 0.3), uniform(0.5, 0.7)],
            theta_rad: uniform(-FRAC_PI_2, FRAC_PI_2),
            phi_rad: uniform(0., FRAC_PI_2),
            alpha_rad: uniform(-FRAC_PI_4, FRAC_PI_4),
        };
        if difficulty == 1 {
            pose.theta_rad = 0.;
            pose.phi_rad = FRAC_PI_2;
            pose.alpha_rad = 0.;
        }
        let mut cores = vec![pose];
        let mut q_rad = vec![if difficulty >= 3 {
            uniform(-PI / 2., PI / 2.)
        } else {
            0.
        }];
        if difficulty == 4 {
            let mut other = pose;
            other.position_m[1] -= uniform(0., 0.4);
            other.theta_rad = uniform(-FRAC_PI_2, FRAC_PI_2);
            other.phi_rad = uniform(0., FRAC_PI_2);
            other.alpha_rad = uniform(-FRAC_PI_4, FRAC_PI_4);
            cores.push(other);
            q_rad.push(uniform(-FRAC_PI_2, FRAC_PI_2));
        }
        Ok(Self {
            difficulty,
            seed,
            selection,
            cores,
            q_rad,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn spherical_normal_roll_and_poles() {
        for theta in [-FRAC_PI_2, -0.2, 0., FRAC_PI_2] {
            for phi in [0., 0.3, FRAC_PI_2] {
                for alpha in [-FRAC_PI_4, 0., FRAC_PI_4] {
                    let r = TargetPose {
                        position_m: [0.; 3],
                        theta_rad: theta,
                        phi_rad: phi,
                        alpha_rad: alpha,
                    }
                    .rotation_matrix();
                    let normal = [phi.sin() * theta.cos(), phi.sin() * theta.sin(), phi.cos()];
                    for i in 0..3 {
                        assert!((r[i][0] - normal[i]).abs() < 1e-12);
                    }
                    for i in 0..3 {
                        for j in 0..3 {
                            let dot: f64 = (0..3).map(|k| r[k][i] * r[k][j]).sum();
                            assert!((dot - f64::from(u8::from(i == j))).abs() < 1e-12);
                        }
                    }
                    let cross = [
                        r[1][0] * r[2][1] - r[2][0] * r[1][1],
                        r[2][0] * r[0][1] - r[0][0] * r[2][1],
                        r[0][0] * r[1][1] - r[1][0] * r[0][1],
                    ];
                    for i in 0..3 {
                        assert!((cross[i] - r[i][2]).abs() < 1e-12);
                    }
                }
            }
        }
    }
    #[test]
    fn deterministic_ranges_and_paired_coordinates() {
        for d in 1..=4 {
            for seed in 0..1000 {
                let a = Targets::sample(seed, 3, d).unwrap();
                assert_eq!(a, Targets::sample(seed, 3, d).unwrap());
                assert_ne!(a, Targets::sample(seed, 4, d).unwrap());
                let p = a.cores[0];
                for (v, (lo, hi)) in
                    p.position_m
                        .into_iter()
                        .zip([(-0.1, 0.), (0.1, 0.3), (0.5, 0.7)])
                {
                    assert!((lo..=hi).contains(&v));
                }
                assert!((-FRAC_PI_2..=FRAC_PI_2).contains(&p.theta_rad));
                assert!((0. ..=FRAC_PI_2).contains(&p.phi_rad));
                assert!((-FRAC_PI_4..=FRAC_PI_4).contains(&p.alpha_rad));
                assert!(a.q_rad.iter().all(|q| q.abs() <= FRAC_PI_2));
                if d == 1 {
                    assert_eq!([p.theta_rad, p.phi_rad, p.alpha_rad], [0., FRAC_PI_2, 0.]);
                }
                if d == 4 {
                    let b = a.cores[1];
                    assert_eq!(p.position_m[0], b.position_m[0]);
                    assert_eq!(p.position_m[2], b.position_m[2]);
                    assert!((0. ..=0.4).contains(&(p.position_m[1] - b.position_m[1])));
                }
            }
        }
        assert!(Targets::sample(0, 0, 0).is_err());
        assert!(Targets::sample(0, 0, 5).is_err());
    }
}
