// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Immutable model-local frames shared by sensor and visual adapters.
//!
//! A carrier may describe a physical body or a prescribed frame. Source keys
//! neither imply a dynamic model nor identify an installation in a live world.

use crate::Pose;
use std::{collections::BTreeSet, sync::Arc};

/// Caller-assigned mount identity, unique within one model's mount table.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MountKey(pub u64);

/// Caller-assigned carrier identity within one model, independent of live bodies.
/// The caller defines which body or prescribed frame supplies its resolved pose.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CarrierKey(pub u64);

/// Immutable rigid attachment in a carrier's local FLU frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MountDescription {
    key: MountKey,
    carrier: CarrierKey,
    carrier_from_mount: Pose,
}

impl MountDescription {
    /// Validate finite translation in metres and a unit wxyz quaternion
    /// (squared norm tolerance 1e-6). Preserve supplied values without repair,
    /// normalization or inference of a body, sensor type or visual resource.
    pub fn new(
        key: MountKey,
        carrier: CarrierKey,
        carrier_from_mount: Pose,
    ) -> Result<Self, &'static str> {
        let norm = carrier_from_mount
            .rotation_wxyz
            .iter()
            .map(|x| x * x)
            .sum::<f64>();
        if !carrier_from_mount
            .translation_m
            .into_iter()
            .all(f64::is_finite)
            || !norm.is_finite()
            || (norm - 1.).abs() > 1e-6
        {
            return Err("invalid mount pose");
        }
        Ok(Self {
            key,
            carrier,
            carrier_from_mount,
        })
    }

    /// Source mount key, scoped to the enclosing model.
    pub fn key(&self) -> MountKey {
        self.key
    }

    /// Source carrier whose actual resolved pose the caller supplies.
    pub fn carrier(&self) -> CarrierKey {
        self.carrier
    }

    /// Transform from mount-local to carrier-local FLU, in metres and wxyz.
    pub fn carrier_from_mount(&self) -> Pose {
        self.carrier_from_mount
    }
}

#[derive(Debug, PartialEq)]
struct MountData {
    carriers: Vec<CarrierKey>,
    mounts: Vec<MountDescription>,
}

/// Shared immutable source mounts and explicit carrier list in caller order.
/// Keys have separate namespaces and no runtime, solver or domain meaning.
/// Cloning shares both arrays; setup may resolve their ordinals once so hot
/// consumers can use borrowed slices without searching or allocating.
///
/// ```
/// use rm_sim_engine::{Pose, model::{CarrierKey, MountKey, MountDescription, MountTable}};
/// let carrier = CarrierKey(7);
/// let mount = MountDescription::new(MountKey(7), carrier, Pose::at([0.1, 0., 0.2]))?;
/// let table = MountTable::new(vec![carrier], vec![mount])?;
/// let shared = table.clone();
/// assert_eq!(table.mounts().as_ptr(), shared.mounts().as_ptr());
/// assert_eq!(table.carriers().as_ptr(), shared.carriers().as_ptr());
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct MountTable(Arc<MountData>);

impl MountTable {
    /// Take ownership without copying or reordering either array. Reject duplicate
    /// carrier keys, duplicate mount keys and references absent from `carriers`.
    /// Empty tables and explicitly listed carriers without mounts are valid.
    pub fn new(
        carriers: Vec<CarrierKey>,
        mounts: Vec<MountDescription>,
    ) -> Result<Self, &'static str> {
        let mut carrier_keys = BTreeSet::new();
        for &carrier in &carriers {
            if !carrier_keys.insert(carrier) {
                return Err("duplicate carrier key");
            }
        }
        let mut mount_keys = BTreeSet::new();
        for mount in &mounts {
            if !mount_keys.insert(mount.key) {
                return Err("duplicate mount key");
            }
            if !carrier_keys.contains(&mount.carrier) {
                return Err("mount carrier is missing");
            }
        }
        Ok(Self(Arc::new(MountData { carriers, mounts })))
    }

    /// Explicit unique source carriers in original caller order.
    pub fn carriers(&self) -> &[CarrierKey] {
        &self.0.carriers
    }

    /// Validated source mounts in original caller order.
    pub fn mounts(&self) -> &[MountDescription] {
        &self.0.mounts
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mount(key: u64, carrier: u64) -> MountDescription {
        MountDescription::new(MountKey(key), CarrierKey(carrier), Pose::default()).unwrap()
    }

    #[test]
    fn retains_order_values_and_owned_allocations_and_shares_both_arrays() {
        let pose = Pose {
            rotation_wxyz: [-1., 0., 0., 0.],
            ..Pose::at([1., 2., 3.])
        };
        let first = MountDescription::new(MountKey(9), CarrierKey(4), pose).unwrap();
        let carriers = vec![CarrierKey(4), CarrierKey(1), CarrierKey(9)];
        let mounts = vec![first, mount(2, 1), mount(4, 4)];
        let pointers = (carriers.as_ptr(), mounts.as_ptr());
        let table = MountTable::new(carriers, mounts).unwrap();
        let shared = table.clone();
        assert_eq!(
            (table.carriers().as_ptr(), table.mounts().as_ptr()),
            pointers
        );
        assert_eq!(
            (shared.carriers().as_ptr(), shared.mounts().as_ptr()),
            pointers
        );
        assert_eq!(
            table.carriers(),
            &[CarrierKey(4), CarrierKey(1), CarrierKey(9)]
        );
        assert_eq!(table.mounts(), &[first, mount(2, 1), mount(4, 4)]);
        assert_eq!(first.key(), MountKey(9));
        assert_eq!(first.carrier(), CarrierKey(4));
        assert_eq!(first.carrier_from_mount(), pose);
        drop(table);
        assert_eq!(shared.mounts()[0], first);
    }

    #[test]
    fn rejects_duplicate_keys_and_missing_references() {
        assert_eq!(
            MountTable::new(vec![CarrierKey(1); 2], vec![]).unwrap_err(),
            "duplicate carrier key"
        );
        assert_eq!(
            MountTable::new(vec![CarrierKey(1)], vec![mount(2, 1); 2]).unwrap_err(),
            "duplicate mount key"
        );
        assert_eq!(
            MountTable::new(vec![], vec![mount(2, 1)]).unwrap_err(),
            "mount carrier is missing"
        );
        assert!(MountTable::new(vec![], vec![]).is_ok());
        assert!(MountTable::new(vec![CarrierKey(1)], vec![]).is_ok());
    }

    #[test]
    fn rejects_invalid_poses_without_normalizing_accepted_values() {
        for pose in [
            Pose::at([f64::NAN, 0., 0.]),
            Pose::at([0., f64::INFINITY, 0.]),
            Pose {
                rotation_wxyz: [0.; 4],
                ..Pose::default()
            },
            Pose {
                rotation_wxyz: [2., 0., 0., 0.],
                ..Pose::default()
            },
            Pose {
                rotation_wxyz: [f64::NAN, 0., 0., 0.],
                ..Pose::default()
            },
            Pose {
                rotation_wxyz: [f64::MAX, 0., 0., 0.],
                ..Pose::default()
            },
        ] {
            assert!(MountDescription::new(MountKey(0), CarrierKey(0), pose).is_err());
        }
        let pose = Pose {
            rotation_wxyz: [1.0000001, 0., 0., 0.],
            ..Pose::default()
        };
        assert_eq!(
            MountDescription::new(MountKey(0), CarrierKey(0), pose)
                .unwrap()
                .carrier_from_mount(),
            pose
        );
    }
}
