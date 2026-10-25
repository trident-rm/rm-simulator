// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Copied privileged frame observations for passive sensors and pose consumers.
//!
//! Callers supply actual resolved body or prescribed-frame poses and explicit
//! continuity evidence. These values neither access a runtime nor measure a
//! sensor, advance time, validate a live binding or establish scene topology.

use crate::Pose;
use glam::{DQuat, DVec3};
use sim_engine::model::{CarrierKey, MountDescription, MountKey};

/// Caller-stamped identity/continuity evidence for one resolved source carrier.
/// Equality compares every field. This freely constructible record is not an
/// unforgeable runtime token and does not validate a body binding. Adapters must
/// stamp it from validated runtime observations; opaque integration comes later.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CarrierIdentity {
    /// Model-local source carrier. The caller scopes it to this installation.
    pub carrier: CarrierKey,
    /// Caller-stamped world/reset evidence, not an unforgeable runtime brand.
    /// Change when the supplying world is replaced or reconstructed.
    pub world_brand: u64,
    /// Caller-stamped installation generation; change when this carrier's model
    /// instance is removed/reinstalled, including reuse of the same source key.
    pub installation_generation: u64,
    /// Caller-stamped placement revision; change on teleport or other motion
    /// discontinuity. Ordinary continuous body or prescribed motion retains it.
    pub discontinuity_revision: u64,
}

/// Immutable copied carrier pose at an explicitly supplied simulation stamp.
/// Exact integration intervals are retained separately from integer timestamps:
/// substeps may end at fractional nanoseconds rounded down in `time_ns`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StampedCarrierPose {
    identity: CarrierIdentity,
    time_ns: u64,
    interval_s: f64,
    world_from_carrier: Pose,
}

impl StampedCarrierPose {
    /// Copy an actual resolved pose in world FLU metres and wxyz rotation.
    /// Require a finite positive interval in seconds, including the first sample;
    /// initialization without a preceding interval belongs to consumer history.
    /// Validate finite translation and unit rotation (squared norm tolerance
    /// 1e-6) without normalization. Preserve the interval's exact supplied bits;
    /// do not derive it from rounded timestamp differences.
    ///
    /// ```
    /// use sim_engine::model::CarrierKey;
    /// use rm_sim_physics::{Pose, sensors::carrier::{CarrierIdentity, StampedCarrierPose}};
    /// let identity = CarrierIdentity {
    ///     carrier: CarrierKey(3), world_brand: 8,
    ///     installation_generation: 1, discontinuity_revision: 0,
    /// };
    /// let interval_s = (1. / 128.) / 8.;
    /// let sample = StampedCarrierPose::new(identity, 976_562, interval_s, Pose::default())?;
    /// assert_eq!(sample.interval_s().to_bits(), interval_s.to_bits());
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn new(
        identity: CarrierIdentity,
        time_ns: u64,
        interval_s: f64,
        world_from_carrier: Pose,
    ) -> Result<Self, &'static str> {
        if !interval_s.is_finite() || interval_s <= 0. {
            return Err("carrier interval must be finite and positive");
        }
        validate_pose(world_from_carrier)?;
        Ok(Self {
            identity,
            time_ns,
            interval_s,
            world_from_carrier,
        })
    }

    /// Copied caller-stamped carrier identity and continuity evidence.
    pub fn identity(&self) -> CarrierIdentity {
        self.identity
    }

    /// End stamp in simulation nanoseconds, possibly rounded down by the caller.
    pub fn time_ns(&self) -> u64 {
        self.time_ns
    }

    /// Exact caller-supplied positive integration interval in seconds.
    pub fn interval_s(&self) -> f64 {
        self.interval_s
    }

    /// Actual resolved carrier-local to world FLU transform in metres and wxyz.
    pub fn world_from_carrier(&self) -> Pose {
        self.world_from_carrier
    }

    /// Whether all supplied identity/continuity fields match. Does not establish
    /// temporal adjacency, valid scene topology, or authenticity of either stamp.
    pub fn same_continuity(&self, other: &Self) -> bool {
        self.identity == other.identity
    }

    /// Compose `world_from_carrier * carrier_from_mount` in FLU using ordinary
    /// quaternion/vector arithmetic without normalizing inputs or output.
    /// Reject mismatched source carriers and nonfinite or nonunit results (for
    /// example overflow or compounded input norm error). No runtime is queried.
    /// The result preserves carrier evidence, timestamp and exact interval.
    ///
    /// ```
    /// use sim_engine::model::{CarrierKey, MountKey, MountDescription};
    /// use rm_sim_physics::{Pose, sensors::carrier::{CarrierIdentity, StampedCarrierPose}};
    /// let identity = CarrierIdentity {
    ///     carrier: CarrierKey(3), world_brand: 8,
    ///     installation_generation: 1, discontinuity_revision: 0,
    /// };
    /// let carrier = StampedCarrierPose::new(identity, 0, 0.001, Pose::at([1., 0., 0.]))?;
    /// let mount = MountDescription::new(MountKey(4), CarrierKey(3), Pose::at([0., 0., 0.2]))?;
    /// let resolved = carrier.resolve_mount(&mount)?;
    /// assert_eq!(resolved.world_from_mount().translation_m, [1., 0., 0.2]);
    /// assert_eq!(resolved.carrier(), carrier);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn resolve_mount(
        &self,
        mount: &MountDescription,
    ) -> Result<StampedMountPose, &'static str> {
        if mount.carrier() != self.identity.carrier {
            return Err("mount belongs to another source carrier");
        }
        let local = mount.carrier_from_mount();
        let rotation = quaternion(self.world_from_carrier);
        let translation = DVec3::from_array(self.world_from_carrier.translation_m)
            + rotation * DVec3::from_array(local.translation_m);
        let [x, y, z, w] = (rotation * quaternion(local)).to_array();
        let world_from_mount = Pose {
            translation_m: translation.to_array(),
            rotation_wxyz: [w, x, y, z],
        };
        validate_pose(world_from_mount)?;
        Ok(StampedMountPose {
            mount: mount.key(),
            carrier: *self,
            world_from_mount,
        })
    }
}

/// Copied privileged mount pose for a passive sensor or visual adapter.
/// World FLU conversion for rendering belongs to that adapter. This is not a
/// sensor measurement, live binding, or arbitrary continuation checkpoint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StampedMountPose {
    mount: MountKey,
    carrier: StampedCarrierPose,
    world_from_mount: Pose,
}

impl StampedMountPose {
    /// Model-local source mount key.
    pub fn mount(&self) -> MountKey {
        self.mount
    }

    /// Original copied carrier input, including its stamp and continuity evidence.
    pub fn carrier(&self) -> StampedCarrierPose {
        self.carrier
    }

    /// Mount-local to world FLU transform in metres and wxyz rotation.
    pub fn world_from_mount(&self) -> Pose {
        self.world_from_mount
    }
}

fn quaternion(pose: Pose) -> DQuat {
    let [w, x, y, z] = pose.rotation_wxyz;
    DQuat::from_xyzw(x, y, z, w)
}

fn validate_pose(pose: Pose) -> Result<(), &'static str> {
    let norm = pose.rotation_wxyz.iter().map(|x| x * x).sum::<f64>();
    if !pose.translation_m.into_iter().all(f64::is_finite)
        || !norm.is_finite()
        || (norm - 1.).abs() > 1e-6
    {
        return Err("invalid resolved carrier or mount pose");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity() -> CarrierIdentity {
        CarrierIdentity {
            carrier: CarrierKey(2),
            world_brand: 3,
            installation_generation: 4,
            discontinuity_revision: 5,
        }
    }

    fn sample(identity: CarrierIdentity, pose: Pose) -> StampedCarrierPose {
        StampedCarrierPose::new(identity, 976_562, (1. / 128.) / 8., pose).unwrap()
    }

    #[test]
    fn composes_translated_rotated_mount_and_retains_all_input_evidence() {
        let input = sample(
            identity(),
            Pose::yawed([1., 2., 3.], std::f64::consts::FRAC_PI_2),
        );
        let mount = MountDescription::new(
            MountKey(9),
            identity().carrier,
            Pose::yawed([2., 0., 1.], std::f64::consts::FRAC_PI_2),
        )
        .unwrap();
        let output = input.resolve_mount(&mount).unwrap();
        assert_eq!(output.mount(), MountKey(9));
        assert_eq!(output.carrier(), input);
        assert_eq!(input.identity(), identity());
        assert_eq!(input.time_ns(), 976_562);
        assert_eq!(
            input.interval_s().to_bits(),
            ((1_f64 / 128.) / 8.).to_bits()
        );
        assert_eq!(input.world_from_carrier().translation_m, [1., 2., 3.]);
        for (actual, expected) in output
            .world_from_mount()
            .translation_m
            .into_iter()
            .zip([1., 4., 4.])
        {
            assert!((actual - expected).abs() < 1e-12);
        }
        assert!(output.world_from_mount().rotation_wxyz[0].abs() < 1e-12);
        assert!((output.world_from_mount().rotation_wxyz[3] - 1.).abs() < 1e-12);
    }

    #[test]
    fn every_identity_field_is_required_for_continuity_but_time_is_separate() {
        let first = sample(identity(), Pose::default());
        for changed in [
            CarrierIdentity {
                carrier: CarrierKey(9),
                ..identity()
            },
            CarrierIdentity {
                world_brand: 9,
                ..identity()
            },
            CarrierIdentity {
                installation_generation: 9,
                ..identity()
            },
            CarrierIdentity {
                discontinuity_revision: 9,
                ..identity()
            },
        ] {
            assert!(!first.same_continuity(&sample(changed, Pose::default())));
        }
        let later = StampedCarrierPose::new(identity(), 0, 0.1, Pose::at([9., 0., 0.])).unwrap();
        assert!(first.same_continuity(&later));
        let foreign = MountDescription::new(MountKey(1), CarrierKey(9), Pose::default()).unwrap();
        assert!(first.resolve_mount(&foreign).is_err());
    }

    #[test]
    fn composition_preserves_noncommuting_rotation_order() {
        let input = sample(
            identity(),
            Pose::yawed([1., 2., 3.], std::f64::consts::FRAC_PI_2),
        );
        let half = std::f64::consts::FRAC_1_SQRT_2;
        let mount = MountDescription::new(
            MountKey(1),
            identity().carrier,
            Pose {
                translation_m: [0., 2., 1.],
                rotation_wxyz: [half, 0., half, 0.],
            },
        )
        .unwrap();
        let resolved = input.resolve_mount(&mount).unwrap().world_from_mount();
        for (actual, expected) in resolved.translation_m.into_iter().zip([-1., 2., 4.]) {
            assert!((actual - expected).abs() < 1e-12);
        }
        for (actual, expected) in resolved
            .rotation_wxyz
            .into_iter()
            .zip([0.5, -0.5, 0.5, 0.5])
        {
            assert!((actual - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn rejects_invalid_intervals_and_poses_including_initial_zero_interval() {
        for interval in [0., -0., -1., f64::NAN, f64::INFINITY] {
            assert!(StampedCarrierPose::new(identity(), 0, interval, Pose::default()).is_err());
        }
        for pose in [
            Pose::at([f64::NAN, 0., 0.]),
            Pose::at([f64::INFINITY, 0., 0.]),
            Pose {
                rotation_wxyz: [0.; 4],
                ..Pose::default()
            },
            Pose {
                rotation_wxyz: [f64::NAN; 4],
                ..Pose::default()
            },
        ] {
            assert!(StampedCarrierPose::new(identity(), 0, 1., pose).is_err());
        }
    }

    #[test]
    fn composition_never_normalizes_and_rejects_overflow_or_excess_norm_error() {
        let pose = Pose {
            rotation_wxyz: [-1.0000001, 0., 0., 0.],
            ..Pose::default()
        };
        let input = sample(identity(), pose);
        let mount =
            MountDescription::new(MountKey(1), identity().carrier, Pose::default()).unwrap();
        assert_eq!(
            input.resolve_mount(&mount).unwrap().world_from_mount(),
            pose
        );
        let huge = sample(identity(), Pose::at([f64::MAX, 0., 0.]));
        let mount = MountDescription::new(
            MountKey(1),
            identity().carrier,
            Pose::at([f64::MAX, 0., 0.]),
        )
        .unwrap();
        assert!(huge.resolve_mount(&mount).is_err());
        let near = Pose {
            rotation_wxyz: [1.0000004, 0., 0., 0.],
            ..Pose::default()
        };
        let mount = MountDescription::new(MountKey(1), identity().carrier, near).unwrap();
        assert!(sample(identity(), near).resolve_mount(&mount).is_err());
    }
}
