// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Immutable source assembly trees, without backend identities or inferred state.

use super::{BodyState, JointDescription, RigidBodyDescription};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

/// Caller-assigned link identity, unique within one assembly description.
/// This source ID is independent of runtime body keys and installation order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AssemblyLinkId(pub u64);

/// Caller-assigned joint identity, unique within one assembly description.
/// Link and joint IDs have separate namespaces; this is not a runtime joint key.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AssemblyJointId(pub u64);

/// Immutable body source and validated initial state for one assembly link.
/// The initial body pose, COM linear velocity and angular velocity are expressed
/// in the common assembly FLU frame, never relative to the parent link.
#[derive(Clone, Debug, PartialEq)]
pub struct AssemblyLink {
    id: AssemblyLinkId,
    body: RigidBodyDescription,
    initial: BodyState,
}

impl AssemblyLink {
    /// Bind a source ID and body to a finite initial state with unit rotation.
    /// Pose uses metres and wxyz quaternion; COM linear velocity uses m/s and
    /// angular velocity uses rad/s, all in the common assembly FLU frame.
    pub fn new(
        id: AssemblyLinkId,
        body: RigidBodyDescription,
        initial: BodyState,
    ) -> Result<Self, &'static str> {
        initial.validate()?;
        Ok(Self { id, body, initial })
    }

    /// Caller-assigned source identity, independent of runtime keys.
    pub fn id(&self) -> AssemblyLinkId {
        self.id
    }

    /// Validated body description with explicit collider and inertial data.
    pub fn body(&self) -> &RigidBodyDescription {
        &self.body
    }

    /// Copied source state in common assembly FLU: pose in m, COM velocity in
    /// m/s and angular velocity in rad/s. Pose describes the body frame.
    pub fn initial(&self) -> BodyState {
        self.initial
    }
}

/// Directed source edge binding a validated joint description to two link IDs.
/// Joint frames remain body-local; the assembly does not infer displacement or
/// check locked-frame compatibility against the supplied initial body states.
#[derive(Clone, Debug, PartialEq)]
pub struct AssemblyJoint {
    id: AssemblyJointId,
    parent: AssemblyLinkId,
    child: AssemblyLinkId,
    description: JointDescription,
}

impl AssemblyJoint {
    /// Bind source IDs without changing the already validated local description.
    /// Endpoint existence and tree structure are checked by the assembly.
    pub fn new(
        id: AssemblyJointId,
        parent: AssemblyLinkId,
        child: AssemblyLinkId,
        description: JointDescription,
    ) -> Self {
        Self {
            id,
            parent,
            child,
            description,
        }
    }

    /// Caller-assigned source identity, independent of runtime keys.
    pub fn id(&self) -> AssemblyJointId {
        self.id
    }

    /// Parent source link ID in the directed assembly tree.
    pub fn parent(&self) -> AssemblyLinkId {
        self.parent
    }

    /// Child source link ID in the directed assembly tree.
    pub fn child(&self) -> AssemblyLinkId {
        self.child
    }

    /// Original joint description, with body-local frames in metres and wxyz
    /// quaternion and scalar limits in radians or metres according to kind.
    pub fn description(&self) -> &JointDescription {
        &self.description
    }
}

#[derive(Debug, PartialEq)]
struct AssemblyData {
    root: AssemblyLinkId,
    links: Vec<AssemblyLink>,
    joints: Vec<AssemblyJoint>,
}

/// Validated nonempty directed tree of immutable source bodies and joints.
/// Clones share the complete link and joint arrays in caller order. Source IDs
/// are scoped to this description, without runtime keys or persistent hashes.
/// Initial states use a common assembly FLU frame. No initial joint coordinate,
/// mass, inertia, CAD density or joint placement is inferred. Backend preparation
/// validates capabilities and locked frames, including fixed-to-fixed support.
///
/// ```
/// use rm_sim_engine::{Pose, model::{
///     AssemblyDescription, AssemblyJoint, AssemblyJointId, AssemblyLink,
///     AssemblyLinkId, BodyKind, BodyState, JointDescription, JointKind,
///     RigidBodyDescription,
/// }};
/// let root = AssemblyLinkId(10);
/// let child = AssemblyLinkId(20);
/// let body = RigidBodyDescription::new(BodyKind::Fixed, vec![])?;
/// let joint = JointDescription::new(
///     JointKind::Fixed, Pose::default(), Pose::default(), None, false,
/// )?;
/// let assembly = AssemblyDescription::new(root, vec![
///     AssemblyLink::new(child, body.clone(), BodyState::at(Pose::default()))?,
///     AssemblyLink::new(root, body, BodyState::at(Pose::default()))?,
/// ], vec![AssemblyJoint::new(AssemblyJointId(10), root, child, joint)])?;
/// assert_eq!(assembly.links()[0].id(), child); // Caller order is retained.
/// let shared = assembly.clone();
/// assert_eq!(assembly.links().as_ptr(), shared.links().as_ptr());
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct AssemblyDescription(Arc<AssemblyData>);

impl AssemblyDescription {
    /// Take ownership of ordered arrays and require a connected directed tree.
    /// Link and joint IDs must each be unique, every endpoint must exist, and
    /// the root must exist and never be a child. Self edges, multiple parents,
    /// cycles and disconnected links are rejected. A single link without joints
    /// is valid. No body-kind or initial joint-frame compatibility is imposed.
    pub fn new(
        root: AssemblyLinkId,
        links: Vec<AssemblyLink>,
        joints: Vec<AssemblyJoint>,
    ) -> Result<Self, &'static str> {
        if links.is_empty() {
            return Err("assembly must contain at least one link");
        }
        let mut indices = BTreeMap::new();
        for (index, link) in links.iter().enumerate() {
            if indices.insert(link.id, index).is_some() {
                return Err("duplicate assembly link ID");
            }
        }
        if !indices.contains_key(&root) {
            return Err("assembly root link is missing");
        }
        let mut joint_ids = BTreeSet::new();
        let mut has_parent = vec![false; links.len()];
        let mut children = vec![Vec::new(); links.len()];
        for joint in &joints {
            if !joint_ids.insert(joint.id) {
                return Err("duplicate assembly joint ID");
            }
            let Some(&parent) = indices.get(&joint.parent) else {
                return Err("assembly joint parent link is missing");
            };
            let Some(&child) = indices.get(&joint.child) else {
                return Err("assembly joint child link is missing");
            };
            if parent == child {
                return Err("assembly joint cannot connect a link to itself");
            }
            if joint.child == root {
                return Err("assembly root cannot be a child");
            }
            if has_parent[child] {
                return Err("assembly link has multiple parents");
            }
            has_parent[child] = true;
            children[parent].push(child);
        }

        // Remove every acyclic component, including components away from root,
        // so a disconnected cycle cannot hide behind an edge-count check.
        let mut pending: Vec<usize> = (0..links.len()).filter(|&i| !has_parent[i]).collect();
        let mut visited = 0;
        while visited < pending.len() {
            pending.extend_from_slice(&children[pending[visited]]);
            visited += 1;
        }
        if visited != links.len() {
            return Err("assembly contains a directed cycle");
        }
        // An acyclic graph with at most one parent per link is a forest. With
        // n-1 edges it has exactly one component, rooted at the supplied root.
        if joints.len() != links.len() - 1 {
            return Err("assembly links are disconnected from the root");
        }
        Ok(Self(Arc::new(AssemblyData {
            root,
            links,
            joints,
        })))
    }

    /// Source ID of the tree's sole link without a parent.
    pub fn root(&self) -> AssemblyLinkId {
        self.0.root
    }

    /// Complete immutable link array in caller order, without topological sorting.
    pub fn links(&self) -> &[AssemblyLink] {
        &self.0.links
    }

    /// Complete immutable joint array in caller order.
    pub fn joints(&self) -> &[AssemblyJoint] {
        &self.0.joints
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Pose,
        model::{BodyKind, Inertial, JointKind},
    };

    fn link(id: u64) -> AssemblyLink {
        AssemblyLink::new(
            AssemblyLinkId(id),
            RigidBodyDescription::new(BodyKind::Fixed, vec![]).unwrap(),
            BodyState::at(Pose::default()),
        )
        .unwrap()
    }

    fn joint(id: u64, parent: u64, child: u64) -> AssemblyJoint {
        AssemblyJoint::new(
            AssemblyJointId(id),
            AssemblyLinkId(parent),
            AssemblyLinkId(child),
            JointDescription::new(
                JointKind::Fixed,
                Pose::default(),
                Pose::default(),
                None,
                false,
            )
            .unwrap(),
        )
    }

    fn graph(
        links: &[u64],
        edges: &[(u64, u64, u64)],
    ) -> Result<AssemblyDescription, &'static str> {
        AssemblyDescription::new(
            AssemblyLinkId(0),
            links.iter().copied().map(link).collect(),
            edges.iter().map(|&(id, p, c)| joint(id, p, c)).collect(),
        )
    }

    #[test]
    fn single_link_and_separate_source_id_namespaces_are_valid() {
        let single = graph(&[0], &[]).unwrap();
        assert_eq!(single.root(), AssemblyLinkId(0));
        assert_eq!(single.links().len(), 1);
        assert!(single.joints().is_empty());
        let assembly = graph(&[2, 0, 1], &[(0, 1, 2), (1, 0, 1)]).unwrap();
        assert_eq!(assembly.root(), AssemblyLinkId(0));
        assert_eq!(
            assembly
                .links()
                .iter()
                .map(AssemblyLink::id)
                .collect::<Vec<_>>(),
            vec![AssemblyLinkId(2), AssemblyLinkId(0), AssemblyLinkId(1)]
        );
        assert_eq!(assembly.joints()[0].parent(), AssemblyLinkId(1));
        assert_eq!(assembly.joints()[0].child(), AssemblyLinkId(2));
        assert_eq!(assembly.joints()[1].id(), AssemblyJointId(1));
    }

    #[test]
    fn rejects_empty_duplicate_ids_and_missing_root_or_endpoints() {
        for (links, edges, error) in [
            (vec![], vec![], "assembly must contain at least one link"),
            (vec![0, 0], vec![], "duplicate assembly link ID"),
            (vec![1], vec![], "assembly root link is missing"),
            (
                vec![0, 1, 2],
                vec![(5, 0, 1), (5, 0, 2)],
                "duplicate assembly joint ID",
            ),
            (
                vec![0, 1],
                vec![(0, 9, 1)],
                "assembly joint parent link is missing",
            ),
            (
                vec![0, 1],
                vec![(0, 0, 9)],
                "assembly joint child link is missing",
            ),
        ] {
            assert_eq!(graph(&links, &edges).unwrap_err(), error);
        }
    }

    #[test]
    fn rejects_self_edges_root_children_and_multiple_parents() {
        for (edges, error) in [
            (
                vec![(0, 1, 1)],
                "assembly joint cannot connect a link to itself",
            ),
            (vec![(0, 1, 0)], "assembly root cannot be a child"),
            (
                vec![(0, 0, 2), (1, 1, 2)],
                "assembly link has multiple parents",
            ),
            (
                vec![(0, 0, 1), (1, 0, 1)],
                "assembly link has multiple parents",
            ),
        ] {
            assert_eq!(graph(&[0, 1, 2], &edges).unwrap_err(), error);
        }
    }

    #[test]
    fn detects_cycles_away_from_root_even_with_exact_tree_edge_count() {
        assert_eq!(
            graph(&[0, 1, 2], &[(0, 1, 2), (1, 2, 1)]).unwrap_err(),
            "assembly contains a directed cycle"
        );
        assert_eq!(
            graph(&[0, 1, 2, 3], &[(0, 1, 2), (1, 2, 1)]).unwrap_err(),
            "assembly contains a directed cycle"
        );
        assert_eq!(
            graph(&[0, 1, 2, 3], &[(0, 0, 1), (1, 2, 3)]).unwrap_err(),
            "assembly links are disconnected from the root"
        );
        assert_eq!(
            graph(&[0, 1], &[]).unwrap_err(),
            "assembly links are disconnected from the root"
        );
    }

    #[test]
    fn validates_initial_state_without_repair() {
        let body = link(0).body().clone();
        for initial in [
            BodyState::at(Pose::at([f64::NAN, 0., 0.])),
            BodyState::at(Pose {
                rotation_wxyz: [0.; 4],
                ..Pose::default()
            }),
            BodyState {
                velocity_m_s: [0., f64::INFINITY, 0.],
                ..BodyState::at(Pose::default())
            },
            BodyState {
                angular_velocity_rad_s: [f64::NAN, 0., 0.],
                ..BodyState::at(Pose::default())
            },
        ] {
            assert!(AssemblyLink::new(AssemblyLinkId(0), body.clone(), initial).is_err());
        }
    }

    #[test]
    fn clones_share_full_arrays_and_retain_common_frame_states_and_body_data() {
        let inertial = Inertial::new(2., [1., 2., 2.], Pose::at([0.1, 0.2, 0.3])).unwrap();
        let body = RigidBodyDescription::new(BodyKind::Dynamic(inertial), vec![]).unwrap();
        let initial = BodyState {
            pose: Pose::at([5., 6., 7.]),
            velocity_m_s: [1., -2., 3.],
            angular_velocity_rad_s: [-0.4, 0.5, 0.6],
        };
        let child = AssemblyLink::new(AssemblyLinkId(1), body.clone(), initial).unwrap();
        let links = vec![child, link(0)];
        let joints = vec![joint(4, 0, 1)];
        let pointers = (links.as_ptr(), joints.as_ptr());
        let assembly = AssemblyDescription::new(AssemblyLinkId(0), links, joints).unwrap();
        let shared = assembly.clone();
        assert_eq!(
            (assembly.links().as_ptr(), assembly.joints().as_ptr()),
            pointers
        );
        assert_eq!(
            (shared.links().as_ptr(), shared.joints().as_ptr()),
            pointers
        );
        assert_eq!(shared.links()[0].body(), &body);
        assert_eq!(shared.links()[0].initial(), initial);
        let mut copied = shared.links()[0].initial();
        copied.pose.translation_m[0] = 99.;
        assert_ne!(copied, shared.links()[0].initial());
        assert_eq!(
            shared.joints()[0].description(),
            &joint(4, 0, 1).description
        );
    }

    #[test]
    fn fixed_fixed_and_inconsistent_locked_frames_remain_backend_decisions() {
        let mut child = link(1);
        child.initial.pose = Pose::at([10., 20., 30.]);
        let assembly = AssemblyDescription::new(
            AssemblyLinkId(0),
            vec![link(0), child],
            vec![joint(0, 0, 1)],
        )
        .unwrap();
        assert_eq!(
            assembly.links()[1].initial().pose.translation_m,
            [10., 20., 30.]
        );
    }
}
