// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Checked composition of immutable presets, terrain, robots and explicit RFID.
//! Preparation expands and cooks geometry once; sessions/resets share the result.
//! Each instance receives structural physical motion IDs; competition roles remain
//! inspectable source metadata and never implicitly drive repeated equipment.
use super::*;
#[cfg(test)]
use crate::collision_mesh::CollisionMesh;
use crate::{components::PlacedComponent, math};
use rm_sim_physics::{
    arena::ZoneFootprint,
    motion::{MotionChannel, MotionFrame, MotionId, MotionUpdate},
};
use std::collections::HashSet;

/// Caller-owned stable identity, independent of insertion order and team policy.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct InstanceId(pub u64);

/// Immutable geometry and metadata from an existing preset or checked component.
#[derive(Clone)]
pub struct ComponentDefinition {
    geometry: StaticGeometry,
    targets: Vec<ScenarioTarget>,
    visual: Option<VisualInstance>,
}
impl From<Scenario> for ComponentDefinition {
    fn from(source: Scenario) -> Self {
        Self {
            geometry: source.geometry,
            targets: source.targets,
            visual: source.visual,
        }
    }
}
impl ComponentDefinition {
    /// Shared source rest geometry, before the additional instance placement.
    pub fn geometry(&self) -> &StaticGeometry {
        &self.geometry
    }
    /// Original target order used by explicit instance identity mappings.
    pub fn targets(&self) -> &[ScenarioTarget] {
        &self.targets
    }
    /// Original checked visual root, if supplied by the source component.
    pub fn visual(&self) -> Option<&VisualInstance> {
        self.visual.as_ref()
    }
    /// Cook a placed component using the existing collision contract. Target
    /// frames and visual root must already use the same world-FLU source frame.
    /// This checks physical metadata; it does not verify arbitrary file provenance.
    /// Obtain source components through the checked package loader.
    pub fn new(
        component: PlacedComponent,
        targets: Vec<ScenarioTarget>,
        visual: Option<VisualInstance>,
    ) -> anyhow::Result<Self> {
        validate_targets(&targets)?;
        if let Some(visual) = &visual {
            validate_pose(visual.placement)?;
        }
        let mut world = WorldPhysics::new(&[], 0.);
        component.install(&mut world).map_err(anyhow::Error::msg)?;
        Ok(Self {
            geometry: world.static_geometry_snapshot(true),
            targets,
            visual,
        })
    }
}

/// One preset plus an additional world-FLU rigid transform.
#[derive(Clone)]
pub struct ComponentInstance {
    /// Stable instance identity, unique across components, terrain, robots and RFID.
    pub id: InstanceId,
    /// Checked, already source-placed immutable preset.
    pub source: ComponentDefinition,
    /// Additional FLU rigid transform, applied after the source placement.
    pub placement: Pose,
    /// Explicit collision identities, one per source target in source order.
    /// Kind and local plate/face/blade number must match the source; parent
    /// indices may be renamed freely and carry no inferred match team meaning.
    pub targets: Vec<ArmorTarget>,
}

/// Static terrain supplied by the caller, excluding implicit floors/bounds.
#[derive(Clone)]
pub struct TerrainInstance {
    /// Stable identity in the composition-wide namespace.
    pub id: InstanceId,
    /// Finite triangle/box geometry; moving channels are rejected.
    /// Only explicit shapes are imported; catch planes and bounds are not copied.
    pub geometry: StaticGeometry,
    /// Additional FLU rigid transform.
    pub placement: Pose,
}

/// Initial robot with explicit stable identity and an ordinary physical spawn.
#[derive(Clone, Debug)]
pub struct RobotInstance {
    /// Stable identity in the composition-wide namespace.
    pub id: InstanceId,
    /// Physical initial state in composition world FLU; no extra transform applies.
    pub spawn: RobotSpawn,
}

/// Caller-supplied antenna approximation, without implicit rule/card ownership.
#[derive(Clone, Debug)]
pub struct RfidVolume {
    /// Stable identity in the composition-wide namespace.
    pub id: InstanceId,
    /// Opaque caller label; no competition role or team is inferred.
    pub label: String,
    /// Local-prism to world-FLU rigid transform.
    pub pose: Pose,
    /// Simple polygon in local XY metres, in either winding.
    pub polygon_m: Vec<[f64; 2]>,
    /// Inclusive local z range in metres, lower then upper.
    pub z_range_m: [f64; 2],
}
impl RfidVolume {
    /// Test an antenna/body point using the existing polygon edge policy.
    /// Arbitrary tilt is supported by transforming the query into local FLU.
    ///
    /// ```
    /// use rm_sim_assets::scenarios::composition::{InstanceId, RfidVolume};
    /// use rm_sim_physics::Pose;
    /// let volume = RfidVolume { id: InstanceId(9), label: "practice".into(),
    ///     pose: Pose::at([3., 0., 0.]), polygon_m: vec![[-1.,-1.], [1.,-1.],
    ///     [1.,1.], [-1.,1.]], z_range_m: [0.,0.5] };
    /// assert!(volume.contains([3., 0., 0.25]));
    /// assert!(!volume.contains([0., 0., 0.25]));
    /// ```
    pub fn contains(&self, position_m: [f64; 3]) -> bool {
        let delta = std::array::from_fn(|i| position_m[i] - self.pose.translation_m[i]);
        let local = math::rotate(math::quat_conjugate(self.pose.rotation_wxyz), delta);
        ZoneFootprint {
            polygon_m: &self.polygon_m,
            floor_m: self.z_range_m[0],
            height_m: self.z_range_m[1] - self.z_range_m[0],
        }
        .contains(local)
    }
}

/// Reverse mapping of a collision target to its original source identity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TargetIdentity {
    /// Stable owning instance.
    pub instance: InstanceId,
    /// Original source identity; never reinterpreted as new task team policy.
    pub source: ArmorTarget,
    /// Unique identity used by physical target faces and raw contacts.
    pub physical: ArmorTarget,
}

/// Inspectable source role/side to independently driven physical channel mapping.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotionIdentity {
    /// Owning component; its ID supplies the physical namespace.
    pub instance: InstanceId,
    /// Original source role, retained solely as caller interpretation metadata.
    pub kind: Mechanism,
    /// Canonical original source side (Red for the shared DartTarget).
    pub team: Team,
    /// Structural physical identity, stable across preparation order and resets.
    pub physical: MotionId,
}

/// Mapping from caller robot identity to the reset-stable physical chassis ID.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RobotIdentity {
    /// Caller robot identity.
    pub instance: InstanceId,
    /// List-allocated physics chassis ID, stable across recipe resets.
    pub physical: u32,
}

/// Source visual with its transformed root and stable owning instance.
#[derive(Clone, Debug)]
pub struct PlacedVisual {
    /// Owning component instance.
    pub instance: InstanceId,
    /// Checked GLB path and root; the glTF basis remains applied exactly once.
    pub visual: VisualInstance,
}

/// Stable identity and additional rigid placement of prepared scenery.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InstancePlacement {
    /// Caller-owned instance identity.
    pub id: InstanceId,
    /// Additional source-world to composition-world FLU rigid transform.
    pub placement: Pose,
}

/// Editable preparation inputs; call [`Self::prepare`] for a checked immutable definition.
#[derive(Clone, Default)]
pub struct Composition {
    /// Ordered checked component instances.
    pub components: Vec<ComponentInstance>,
    /// Ordered explicit static terrain instances.
    pub terrain: Vec<TerrainInstance>,
    /// Ordered robot spawns; order determines mapped chassis IDs.
    pub robots: Vec<RobotInstance>,
    /// Explicit caller metadata; presets contribute no inferred RFID.
    pub rfid: Vec<RfidVolume>,
    /// Flat catch-floor height in world metres, or None when terrain owns ground.
    pub catch_floor_m: Option<f64>,
}

/// Checked immutable composition; geometry is cooked only during preparation.
#[derive(Clone)]
pub struct ComposedScenario {
    geometry: StaticGeometry,
    targets: Vec<ScenarioTarget>,
    robots: Vec<RobotSpawn>,
    robot_ids: Vec<RobotIdentity>,
    target_ids: Vec<TargetIdentity>,
    motion_ids: Vec<MotionIdentity>,
    target_motion_ids: Vec<Option<MotionId>>,
    visuals: Vec<PlacedVisual>,
    rfid: Vec<RfidVolume>,
    catch_floor_m: Option<f64>,
    components: Vec<InstancePlacement>,
    terrain: Vec<InstancePlacement>,
}

impl Composition {
    /// Validate identities, poses and cardinality, allocate instance channels, then cook.
    /// Preparation reuses native shapes and transforms retained mesh vertices once;
    /// it is deliberately outside session creation/reset and stepping.
    ///
    /// ```
    /// use rm_sim_assets::scenarios::{Scenario, composition::*};
    /// use rm_sim_physics::{Pose, StepDuration, arena::TunnelModel};
    /// let source = Scenario::tunnel(TunnelModel { pose: Pose::default(),
    ///     opening_m: [2., 1., 0.65], wall_thickness_m: 0.1 })?;
    /// let definition = Composition { components: vec![ComponentInstance {
    ///     id: InstanceId(7), source: source.into(), placement: Pose::at([3., 0., 0.]), targets: vec![],
    /// }], catch_floor_m: Some(0.), ..Default::default() }.prepare()?;
    /// let mut session = definition.session(StepDuration::HZ_1000, Default::default())?;
    /// session.step(10).map_err(anyhow::Error::msg)?;
    /// session.reset().map_err(anyhow::Error::msg)?;
    /// assert_eq!(session.stamp().time_ns, 0);
    /// # Ok::<(), anyhow::Error>(())
    /// ```
    pub fn prepare(self) -> anyhow::Result<ComposedScenario> {
        ensure!(
            self.catch_floor_m.is_none_or(f64::is_finite),
            "nonfinite catch floor"
        );
        let mut ids = HashSet::new();
        for id in self
            .components
            .iter()
            .map(|x| x.id)
            .chain(self.terrain.iter().map(|x| x.id))
            .chain(self.robots.iter().map(|x| x.id))
            .chain(self.rfid.iter().map(|x| x.id))
        {
            ensure!(ids.insert(id), "duplicate instance ID {id:?}");
        }
        let mut physical_ids = HashSet::new();
        let mut motion_ids = Vec::new();
        let mut target_motion_ids = Vec::new();
        let mut targets = Vec::new();
        let mut target_ids = Vec::new();
        let mut visuals = Vec::new();
        let mut physics = WorldPhysics::new(&[], 0.);
        for instance in &self.components {
            validate_pose(instance.placement)?;
            validate_targets(&instance.source.targets)?;
            ensure!(
                instance.targets.len() == instance.source.targets.len(),
                "target mapping cardinality mismatch"
            );
            let parts = instance.source.geometry.moving_parts();
            let mut own_channels = Vec::new();
            for (kind, team) in parts.iter().map(|p| (p.kind, p.team)).chain(
                instance
                    .source
                    .targets
                    .iter()
                    .filter_map(|t| t.rotation.map(|r| (r.mechanism, r.team))),
            ) {
                let channel = canonical_channel(kind, team);
                if !own_channels.contains(&channel) {
                    own_channels.push(channel);
                }
            }
            for (kind, team) in &own_channels {
                motion_ids.push(MotionIdentity {
                    instance: instance.id,
                    kind: *kind,
                    team: *team,
                    physical: physical_motion_id(instance.id, *kind, *team),
                });
            }
            for (source, physical) in instance.source.targets.iter().zip(&instance.targets) {
                ensure!(
                    same_surface(source.face.target, *physical),
                    "target mapping changes source kind or local surface index"
                );
                ensure!(
                    physical_ids.insert(*physical),
                    "duplicate physical target identity"
                );
                let mut target = source.clone();
                target.face.target = *physical;
                target.face.pose = transform_pose(instance.placement, target.face.pose);
                validate_pose(target.face.pose)?;
                if let Some(detector) = &mut target.detector {
                    detector.pose = transform_pose(instance.placement, detector.pose);
                }
                if let Some(rotation) = &mut target.rotation {
                    rotation.pivot_m = transform_point(instance.placement, rotation.pivot_m);
                    rotation.axis = math::rotate(instance.placement.rotation_wxyz, rotation.axis);
                }
                target_motion_ids.push(
                    source
                        .rotation
                        .map(|r| physical_motion_id(instance.id, r.mechanism, r.team)),
                );
                targets.push(target);
                target_ids.push(TargetIdentity {
                    instance: instance.id,
                    source: source.face.target,
                    physical: *physical,
                });
            }
            if let Some(visual) = &instance.source.visual {
                let placement = transform_pose(instance.placement, visual.placement);
                validate_pose(placement)?;
                visuals.push(PlacedVisual {
                    instance: instance.id,
                    visual: VisualInstance {
                        file: visual.file.clone(),
                        placement,
                    },
                });
            }
            physics
                .add_fixed_geometry(&instance.source.geometry.fixed_only(), instance.placement)
                .map_err(anyhow::Error::msg)?;
            for part in parts {
                let frame = match part.motion {
                    MotionFrame::Rotation { pivot_m, axis } => MotionFrame::Rotation {
                        pivot_m: transform_point(instance.placement, pivot_m),
                        axis: math::rotate(instance.placement.rotation_wxyz, axis),
                    },
                    MotionFrame::Translation { offsets_m } => MotionFrame::Translation {
                        offsets_m: offsets_m
                            .map(|v| math::rotate(instance.placement.rotation_wxyz, v)),
                    },
                };
                physics
                    .add_motion_geometry(
                        &part.geometry,
                        instance.placement,
                        physical_motion_id(instance.id, part.kind, part.team),
                        frame,
                    )
                    .map_err(anyhow::Error::msg)?;
            }
        }
        for terrain in &self.terrain {
            validate_pose(terrain.placement)?;
            ensure!(
                terrain.geometry.motion_parts().is_empty(),
                "terrain contains moving channels"
            );
            physics
                .add_fixed_geometry(&terrain.geometry, terrain.placement)
                .map_err(anyhow::Error::msg)?;
        }
        for volume in &self.rfid {
            validate_rfid(volume)?;
        }
        let mut robot_ids = Vec::new();
        for (index, robot) in self.robots.iter().enumerate() {
            validate_pose(robot.spawn.pose)?;
            robot_ids.push(RobotIdentity {
                instance: robot.id,
                physical: u32::try_from(index).context("too many robot spawns")?,
            });
        }
        let definition = ComposedScenario {
            geometry: physics.static_geometry_snapshot(true),
            targets,
            robots: self.robots.into_iter().map(|r| r.spawn).collect(),
            robot_ids,
            target_ids,
            motion_ids,
            target_motion_ids,
            visuals,
            rfid: self.rfid,
            catch_floor_m: self.catch_floor_m,
            components: self
                .components
                .iter()
                .map(|x| InstancePlacement {
                    id: x.id,
                    placement: x.placement,
                })
                .collect(),
            terrain: self
                .terrain
                .iter()
                .map(|x| InstancePlacement {
                    id: x.id,
                    placement: x.placement,
                })
                .collect(),
        };
        // Validate robot configurations and the complete target topology before publishing.
        definition.session_with_motions(StepDuration::HZ_1000, &definition.default_motions())?;
        Ok(definition)
    }
}

impl ComposedScenario {
    /// Configured flat catch-floor height in metres, or None for terrain-owned ground.
    pub fn catch_floor_m(&self) -> Option<f64> {
        self.catch_floor_m
    }
    /// Component identities and additional placements, including geometry-only fixtures.
    pub fn components(&self) -> &[InstancePlacement] {
        &self.components
    }
    /// Static terrain identities and additional placements.
    pub fn terrain(&self) -> &[InstancePlacement] {
        &self.terrain
    }
    /// Initial physical robot definitions, in mapped chassis-ID order.
    pub fn robots(&self) -> &[RobotSpawn] {
        &self.robots
    }
    /// Prepared rest geometry; moving coordinates are caller-owned.
    pub fn geometry(&self) -> &StaticGeometry {
        &self.geometry
    }
    /// Transformed detector definitions with their physical identities.
    pub fn targets(&self) -> &[ScenarioTarget] {
        &self.targets
    }
    /// Complete reverse target map, in component/source target order.
    pub fn target_identities(&self) -> &[TargetIdentity] {
        &self.target_ids
    }
    /// Reset-stable robot identity map, in spawn order.
    pub fn robot_identities(&self) -> &[RobotIdentity] {
        &self.robot_ids
    }
    /// Checked source visuals with exactly one original glTF basis.
    pub fn visuals(&self) -> &[PlacedVisual] {
        &self.visuals
    }
    /// Explicit RFID metadata; no implicit preset footprints are generated.
    pub fn rfid(&self) -> &[RfidVolume] {
        &self.rfid
    }
    /// Resolve a raw contact identity without interpreting competition team policy.
    pub fn source_target(&self, physical: ArmorTarget) -> Option<TargetIdentity> {
        self.target_ids
            .iter()
            .find(|x| x.physical == physical)
            .copied()
    }
    /// Resolve prescribed endpoints transactionally into caller-owned storage.
    pub fn targets_at(
        &self,
        state: &MechanismState,
        output: &mut Vec<TargetFace>,
    ) -> anyhow::Result<()> {
        let updates = self.legacy_motions(state)?;
        self.targets_at_motions(&updates, output)
    }
    /// Create an independent session sharing prepared shapes and initial topology.
    /// Reset preserves IDs, robots, targets and initial coordinates. Caller sensor,
    /// controller, RNG and task state remain external to the session recipe.
    pub fn session(
        &self,
        duration: StepDuration,
        state: MechanismState,
    ) -> anyhow::Result<RobotSession> {
        let updates = self.legacy_motions(&state)?;
        self.session_with_motions(duration, &updates)
    }

    /// Structural source-to-instance motion mappings. Disjoint component instance
    /// IDs yield disjoint physical namespaces, including across compositions.
    pub fn motion_identities(&self) -> &[MotionIdentity] {
        &self.motion_ids
    }

    /// Complete rest coordinates (zero angle/fraction, open legacy DartDoors).
    /// This is explicit initial data; no automatic motion is scheduled.
    pub fn default_motions(&self) -> Vec<MotionUpdate> {
        self.motion_ids
            .iter()
            .map(|m| MotionUpdate {
                id: m.physical,
                coordinate: f64::from(m.kind == Mechanism::DartDoor),
            })
            .collect()
    }

    /// Explicitly translate unambiguous source roles into instance coordinates.
    /// Repeated source channels reject rather than silently synchronize instances.
    pub fn legacy_motions(&self, state: &MechanismState) -> anyhow::Result<Vec<MotionUpdate>> {
        validate_state(state)?;
        for (index, channel) in self.motion_ids.iter().enumerate() {
            ensure!(
                !self.motion_ids[..index]
                    .iter()
                    .any(|other| (other.kind, other.team) == (channel.kind, channel.team)),
                "ambiguous legacy mechanism channel; use explicit motion IDs"
            );
        }
        Ok(self
            .motion_ids
            .iter()
            .map(|m| MotionUpdate {
                id: m.physical,
                coordinate: state.fraction(m.kind, m.team),
            })
            .collect())
    }

    fn validate_motions(&self, updates: &[MotionUpdate]) -> anyhow::Result<()> {
        ensure!(
            updates.len() == self.motion_ids.len(),
            "complete motion coordinates required"
        );
        for (index, update) in updates.iter().enumerate() {
            ensure!(
                !updates[..index].iter().any(|u| u.id == update.id),
                "duplicate motion ID"
            );
            let identity = self
                .motion_ids
                .iter()
                .find(|m| m.physical == update.id)
                .context("unknown motion ID")?;
            ensure!(update.coordinate.is_finite(), "nonfinite motion coordinate");
            ensure!(
                matches!(identity.kind, Mechanism::Outpost | Mechanism::Rune)
                    || (0. ..=1.).contains(&update.coordinate),
                "translation coordinate outside 0..=1"
            );
        }
        // Also validate resolved source-frame translations before publishing targets.
        self.geometry
            .validate_motions(
                updates
                    .iter()
                    .filter(|u| self.geometry_channels().any(|id| id == u.id))
                    .copied()
                    .collect::<Vec<_>>()
                    .as_slice(),
            )
            .map_err(anyhow::Error::msg)?;
        Ok(())
    }

    fn geometry_channels(&self) -> impl Iterator<Item = MotionId> + '_ {
        self.geometry
            .motion_channels()
            .filter_map(|channel| match channel {
                MotionChannel::Instance(id) => Some(id),
                MotionChannel::Legacy { .. } => None,
            })
    }

    /// Resolve detector endpoints from a complete explicit coordinate table.
    /// Unknown/duplicate/missing IDs and invalid coordinates leave output unchanged.
    pub fn targets_at_motions(
        &self,
        updates: &[MotionUpdate],
        output: &mut Vec<TargetFace>,
    ) -> anyhow::Result<()> {
        self.validate_motions(updates)?;
        let mut resolved = Vec::with_capacity(self.targets.len());
        for (target, id) in self.targets.iter().zip(&self.target_motion_ids) {
            let mut face = target.face;
            if let (Some(rotation), Some(id)) = (target.rotation, id) {
                let coordinate = updates
                    .iter()
                    .find(|u| u.id == *id)
                    .expect("validated complete coordinates")
                    .coordinate;
                face.pose = OutpostModel::rotate_pose(
                    face.pose,
                    rotation.pivot_m,
                    rotation.axis,
                    coordinate,
                );
                validate_pose(face.pose)?;
            }
            resolved.push(face);
        }
        *output = resolved;
        Ok(())
    }

    /// Create a session with explicit initial coordinates, retained by recipe reset.
    /// Subsequent partial physical updates use `physics_mut().sync_motions`; target
    /// endpoint resolution needs a complete caller-owned coordinate table. IDs and
    /// rest frames survive reset, captures and checked insertion into another world.
    pub fn session_with_motions(
        &self,
        duration: StepDuration,
        updates: &[MotionUpdate],
    ) -> anyhow::Result<RobotSession> {
        let mut targets = Vec::new();
        self.targets_at_motions(updates, &mut targets)?;
        let geometry_updates: Vec<_> = updates
            .iter()
            .filter(|u| self.geometry_channels().any(|id| id == u.id))
            .copied()
            .collect();
        let geometry = self
            .geometry
            .at_motions(&geometry_updates)
            .map_err(anyhow::Error::msg)?;
        let robots = self.robots.clone();
        let floor = self.catch_floor_m;
        let recipe = RobotScenario::new(targets, floor.unwrap_or(0.), duration, move |world| {
            if floor.is_none() {
                world.remove_catch_floor();
            }
            world.try_insert_geometry(&geometry)?;
            for robot in &robots {
                world.add_chassis(robot.team, robot.config.clone(), robot.pose)?;
            }
            Ok(())
        })
        .map_err(anyhow::Error::msg)?;
        RobotSession::new(recipe).map_err(anyhow::Error::msg)
    }
}

fn physical_motion_id(instance: InstanceId, kind: Mechanism, team: Team) -> MotionId {
    let (kind, team) = canonical_channel(kind, team);
    let channel = match kind {
        Mechanism::Outpost => 0,
        Mechanism::Rune => 2,
        Mechanism::Base => 4,
        Mechanism::DartDoor => 6,
        Mechanism::DartTarget => 8,
    } + team.index() as u64;
    MotionId {
        instance: instance.0,
        channel,
    }
}

fn canonical_channel(kind: Mechanism, team: Team) -> (Mechanism, Team) {
    (
        kind,
        if kind == Mechanism::DartTarget {
            Team::Red
        } else {
            team
        },
    )
}
fn same_surface(a: ArmorTarget, b: ArmorTarget) -> bool {
    match (a, b) {
        (ArmorTarget::Base { plate: a, .. }, ArmorTarget::Base { plate: b, .. }) => a == b,
        (ArmorTarget::Outpost { face: a, .. }, ArmorTarget::Outpost { face: b, .. }) => a == b,
        (ArmorTarget::Rune { blade: a, .. }, ArmorTarget::Rune { blade: b, .. }) => a == b,
        _ => false,
    }
}
fn validate_pose(pose: Pose) -> anyhow::Result<()> {
    ensure!(
        pose.translation_m
            .iter()
            .chain(&pose.rotation_wxyz)
            .all(|v| v.is_finite())
            && (math::quat_length(pose.rotation_wxyz) - 1.).abs() <= 1e-6,
        "pose must be finite with a unit quaternion"
    );
    Ok(())
}
fn transform_point(placement: Pose, point: [f64; 3]) -> [f64; 3] {
    math::add(
        placement.translation_m,
        math::rotate(placement.rotation_wxyz, point),
    )
}
fn transform_pose(placement: Pose, pose: Pose) -> Pose {
    Pose {
        translation_m: transform_point(placement, pose.translation_m),
        rotation_wxyz: math::quat_multiply(placement.rotation_wxyz, pose.rotation_wxyz),
    }
}
fn validate_rfid(volume: &RfidVolume) -> anyhow::Result<()> {
    validate_pose(volume.pose)?;
    ensure!(
        !volume.label.is_empty()
            && volume.polygon_m.len() >= 3
            && volume
                .polygon_m
                .iter()
                .flatten()
                .chain(&volume.z_range_m)
                .all(|v| v.is_finite())
            && volume.z_range_m[0] <= volume.z_range_m[1]
            && (volume.z_range_m[1] - volume.z_range_m[0]).is_finite(),
        "invalid RFID prism metadata"
    );
    let area: f64 = volume
        .polygon_m
        .iter()
        .zip(volume.polygon_m.iter().cycle().skip(1))
        .map(|(a, b)| a[0] * b[1] - b[0] * a[1])
        .sum();
    ensure!(
        area.is_finite() && area.abs() > 1e-12,
        "degenerate RFID polygon"
    );
    let count = volume.polygon_m.len();
    for (i, a) in volume.polygon_m.iter().enumerate() {
        ensure!(
            !volume.polygon_m[..i].contains(a),
            "repeated RFID polygon vertex"
        );
        let b = volume.polygon_m[(i + 1) % count];
        for j in i + 1..count {
            if j == i + 1 || (i == 0 && j == count - 1) {
                continue;
            }
            let c = volume.polygon_m[j];
            let d = volume.polygon_m[(j + 1) % count];
            ensure!(
                !segments_intersect(*a, b, c, d)?,
                "self-intersecting RFID polygon"
            );
        }
    }
    Ok(())
}

fn segments_intersect(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> anyhow::Result<bool> {
    let cross = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    let values = [
        cross(a, b, c),
        cross(a, b, d),
        cross(c, d, a),
        cross(c, d, b),
    ];
    ensure!(
        values.iter().all(|v| v.is_finite()),
        "RFID polygon extent overflow"
    );
    let on = |p: [f64; 2], a: [f64; 2], b: [f64; 2]| {
        (0..2).all(|i| p[i] >= a[i].min(b[i]) && p[i] <= a[i].max(b[i]))
    };
    let opposite = |a: f64, b: f64| (a > 0. && b < 0.) || (a < 0. && b > 0.);
    Ok(
        (opposite(values[0], values[1]) && opposite(values[2], values[3]))
            || (values[0] == 0. && on(c, a, b))
            || (values[1] == 0. && on(d, a, b))
            || (values[2] == 0. && on(a, c, d))
            || (values[3] == 0. && on(b, c, d)),
    )
}

fn validate_targets(targets: &[ScenarioTarget]) -> anyhow::Result<()> {
    let mut ids = HashSet::new();
    for target in targets {
        ensure!(
            ids.insert(target.face.target),
            "duplicate source target identity"
        );
        validate_pose(target.face.pose)?;
        ensure!(
            !matches!(target.face.target, ArmorTarget::Chassis { .. }),
            "component target cannot use chassis identities"
        );
        if let Some(detector) = &target.detector {
            validate_pose(detector.pose)?;
            ensure!(
                detector.pose == target.face.pose
                    && detector.half_m.iter().all(|v| v.is_finite() && *v > 0.),
                "invalid or inconsistent detector metadata"
            );
        }
        if let Some(rotation) = target.rotation {
            ensure!(
                matches!(rotation.mechanism, Mechanism::Outpost | Mechanism::Rune)
                    && rotation
                        .pivot_m
                        .iter()
                        .chain(&rotation.axis)
                        .all(|v| v.is_finite())
                    && (rotation.axis.iter().map(|v| v * v).sum::<f64>() - 1.).abs() <= 1e-6,
                "invalid target rotation frame"
            );
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests;
