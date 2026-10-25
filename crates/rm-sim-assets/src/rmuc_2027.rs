// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Source-binding DTOs and validation, independent of the live world adapter.
use crate::cad_assets::{CadAsset, CadAssets};
use anyhow::{Context, ensure};
use rm_sim_gameplay::rmuc_2027::Point;
use rm_sim_physics::{Pose, Team};
use serde::{Deserialize, Serialize};

/// Shared physical source geometry; runtime rules do not own these definitions.
pub use rm_sim_physics::arena::{Detector, OutpostModel as Tower};
/// A reviewed functional RFID footprint. Polygon x/y and floor are in FLU metres.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Zone {
    /// Original field side, independent of current controller.
    pub side: Team,
    /// Constituent control point.
    pub point: Point,
    /// Surface height, metres; supports elevated decks over tunnels.
    pub floor_m: f64,
    /// Height band above the surface in metres; simulator antenna approximation.
    pub height_m: f64,
    /// Footprint corners in world FLU x/y metres.
    pub polygon_m: Vec<[f64; 2]>,
}
/// Serialized source-derived targets and field RFID, without a match runtime.
/// This DTO mirrors the world adapter schema; runtime calibration stays with the caller.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Config {
    /// Bases, Red then Blue. Only source-present armor is bound.
    pub bases: [Vec<Detector>; 2],
    /// Outposts by original side, Red then Blue.
    pub outposts: [Tower; 2],
    /// Control-point footprints; never terrain-only traversal cards.
    pub zones: Vec<Zone>,
    /// Own supply footprints, separately calibrated; point is ignored.
    #[serde(default)]
    pub supply_zones: Vec<Zone>,
    /// Own vantage footprints for zone exchanges; point is ignored.
    #[serde(default)]
    pub vantage_zones: Vec<Zone>,
    /// Physical traversal cards; their zone's point field is unused geometry metadata.
    #[serde(default)]
    pub terrain_zones: Vec<TerrainZone>,
    /// Native Rune source geometry in Red/Blue order; empty for earlier unbound packages.
    #[serde(default)]
    pub runes: Vec<RuneGeometry>,
    /// Shared seeded direction for both rotors, either -1 or 1.
    pub rotor_direction: i8,
}
/// One native physical traversal card and its fitted antenna footprint.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TerrainZone {
    /// Exact native side/course/pad identity, independent of control activations.
    pub card: rm_sim_gameplay::rmuc_2027::terrain::Card,
    /// Reviewed polygon/height geometry; point does not identify the traversal.
    pub zone: Zone,
}
/// Source-fitted Rune target disks and shared physical shaft frame, by face team.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RuneGeometry {
    /// Shaft pivot in world FLU metres; any point along the common axis is equivalent.
    pub pivot_m: [f64; 3],
    /// Shared unit world axis, the same physical sign for both opposing faces.
    pub axis: [f64; 3],
    /// Five source-fitted detector disk poses at rest; +x is outward.
    pub targets: [Pose; 5],
}
impl Config {
    /// Validate finite geometry, unit rotations, detector count and height bands.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.bases.iter().any(Vec::is_empty)
            || self.outposts.iter().any(|o| {
                o.detectors.len() != 3
                    || !o.pivot_m.iter().chain(&o.axis).all(|v| v.is_finite())
                    || (o.axis.iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-6
            })
            || ![-1, 1].contains(&self.rotor_direction)
        {
            return Err("invalid 2027 structure bindings");
        }
        for d in self
            .bases
            .iter()
            .flatten()
            .chain(self.outposts.iter().flat_map(|o| &o.detectors))
        {
            if !d
                .pose
                .translation_m
                .iter()
                .chain(&d.pose.rotation_wxyz)
                .all(|v| v.is_finite())
                || (d.pose.rotation_wxyz.iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-6
                || d.half_m
                    .iter()
                    .any(|v| !v.is_finite() || *v <= 0. || *v > 0.071)
            {
                return Err("invalid 2027 detector");
            }
        }
        if self
            .zones
            .iter()
            .chain(&self.supply_zones)
            .chain(&self.vantage_zones)
            .chain(self.terrain_zones.iter().map(|z| &z.zone))
            .any(|z| {
                z.polygon_m.len() < 3
                    || !z.floor_m.is_finite()
                    || !(0.01..=0.5).contains(&z.height_m)
                    || z.polygon_m.iter().flatten().any(|v| !v.is_finite())
            })
        {
            return Err("invalid 2027 RFID zone");
        }
        if self.terrain_zones.iter().any(|z| {
            !z.card.valid()
                || z.card.course.is_none()
                || z.card.side
                    != match z.zone.side {
                        Team::Red => rm_sim_gameplay::Team::Red,
                        Team::Blue => rm_sim_gameplay::Team::Blue,
                    }
        }) {
            return Err("invalid 2027 traversal identity");
        }
        if !self.runes.is_empty()
            && (self.runes.len() != 2
                || self.runes.iter().any(|r| {
                    r.pivot_m.iter().chain(&r.axis).any(|v| !v.is_finite())
                        || (r.axis.iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-6
                        || r.targets.iter().any(|p| {
                            p.translation_m
                                .iter()
                                .chain(&p.rotation_wxyz)
                                .any(|v| !v.is_finite())
                                || (p.rotation_wxyz.iter().map(|v| v * v).sum::<f64>() - 1.).abs()
                                    > 1e-6
                        })
                }))
        {
            return Err("invalid native Rune geometry");
        }
        Ok(())
    }
}
/// Exact source-body partition for one driven joint.
#[derive(Clone, Debug, PartialEq, Deserialize)]
pub struct Motion {
    /// Exported `extras.rm.id` of the moving node.
    pub id: String,
    /// Exact mesh-node names, naming source BREP bodies or surface-model shells.
    pub bodies: Vec<String>,
    /// Unit axis in the source asset frame.
    pub axis: [f64; 3],
    /// Joint origin in source asset metres.
    pub pivot_m: [f64; 3],
    /// Prismatic travel in metres, or `None` for the outpost's continuous rotation.
    pub limits: Option<[f64; 2]>,
}
/// Ordered source bodies used to fit one structure's optical detectors.
///
/// ```
/// use rm_sim_assets::rmuc_2027::SourceReport;
/// let report: SourceReport = serde_json::from_str(r#"{"detector_bodies":[17,23,31]}"#)?;
/// assert_eq!(report.detector_bodies, [17, 23, 31]);
/// # Ok::<(), serde_json::Error>(())
/// ```
#[derive(Clone, Debug, PartialEq, Deserialize)]
pub struct SourceReport {
    /// STEP body identifiers, in the same order as the fitted detector faces.
    pub detector_bodies: Vec<u64>,
}
/// One source-owned runtime part, preserving distinct drawing/collision membership.
///
/// ```
/// use rm_sim_assets::rmuc_2027::Part;
/// let part: Part = serde_json::from_str(
///     r#"{"id":"outpost/source_body_2200540","asset":"outpost","source_kind":"body"}"#,
/// )?;
/// assert_eq!(part.asset, "outpost");
/// assert!(part.motion_id.is_none());
/// # Ok::<(), serde_json::Error>(())
/// ```
#[derive(Clone, Debug, PartialEq, Deserialize)]
pub struct Part {
    /// Stable identity scoped by runtime asset; never a reconstructed face ID.
    pub id: String,
    /// Runtime asset containing this part (`base` or `outpost`).
    pub asset: String,
    /// Source-owned unit classification, including explicitly derived units.
    pub source_kind: String,
    /// Original source entity or locally scoped native-unit ordinal, when known.
    pub source_id: Option<u64>,
    /// Hierarchical work-part identity retained by the source exporter.
    pub component_id: Option<String>,
    /// Unique visual mesh-node name, when the visual retains this part.
    pub visual_node: Option<String>,
    /// Unique collision mesh-node name, when collision retains this part.
    pub collision_node: Option<String>,
    /// Owning joint identity; absent for stationary geometry.
    pub motion_id: Option<String>,
}
/// Source-pinned 2027 target geometry, motion partitions and part catalogue.
#[derive(Clone, Debug, PartialEq, Deserialize)]
pub struct Bindings<C = Config> {
    /// Binding schema: one for legacy bodies, two for structured parts and shells.
    pub schema_version: u32,
    /// Competition and manual revision, never just a year.
    pub ruleset: String,
    /// Exact DJI source STEP hash.
    pub source_sha256: String,
    /// World FLU detector poses and rotor axes.
    pub world: C,
    /// Outpost armor-module assemblies, excluding the shared fixed frame.
    pub outpost_motion: Vec<Motion>,
    /// Three base protective-armor source bodies.
    pub base_motion: Vec<Motion>,
    /// Optional exact native Rune arm-subtree partitions, Red/Blue order.
    #[serde(default)]
    pub rune_motion: Vec<Motion>,
    /// Source arm node names used by passive CAD light bindings, Red/Blue order.
    #[serde(default)]
    pub rune_arms: Vec<[String; 5]>,
    /// Source partitions retained by the binder, keyed by structure name.
    pub report: std::collections::BTreeMap<String, SourceReport>,
    /// Structured source parts, required by schema two; legacy schema one has none.
    #[serde(default)]
    pub parts: Vec<Part>,
}
impl Bindings {
    /// Validate source identity and all fitted geometry before package admission.
    pub fn validate(&self) -> anyhow::Result<()> {
        self.world.validate().map_err(anyhow::Error::msg)?;
        self.validate_source(self.world.bases[0].len(), self.world.runes.len())
    }
}
impl<C> Bindings<C> {
    /// Validate source partitions against the runtime's validated geometry counts.
    /// `base_detectors` is the red base detector count; `runes` is zero or two.
    /// Callers with a custom configuration must also validate its geometry.
    pub fn validate_source(&self, base_detectors: usize, runes: usize) -> anyhow::Result<()> {
        ensure!(
            matches!(self.schema_version, 1 | 2)
                && self.ruleset == "rmuc2027-v1.0.0"
                && self.source_sha256
                    == "a5ad3ede3b7674fc98f2e73b71717216383cb2b4b5dcdbe900c2038918bea8e0",
            "unknown 2027 source/ruleset binding"
        );
        ensure!(
            (self.rune_motion.is_empty() && self.rune_arms.is_empty() && runes == 0)
                || (self.rune_motion.len() == 2
                    && self.rune_arms.len() == 2
                    && runes == 2
                    && self.rune_motion.iter().all(|m| m.limits.is_none()
                        && !m.bodies.is_empty()
                        && m.axis.iter().chain(&m.pivot_m).all(|v| v.is_finite())
                        && (m.axis.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-6)),
            "incomplete native Rune binding"
        );
        let mut rune_bodies = std::collections::BTreeSet::new();
        for (index, (motion, arms)) in self.rune_motion.iter().zip(&self.rune_arms).enumerate() {
            let hub_prefix = format!(
                "rune/banks/bank-{}/hub-cluster/mesh-",
                if index == 0 { "a" } else { "b" }
            );
            let expected = if index == 0 {
                "rune.red.spin"
            } else {
                "rune.blue.spin"
            };
            let unique_arms: std::collections::BTreeSet<_> = arms.iter().collect();
            ensure!(
                motion.id == expected
                    && unique_arms.len() == 5
                    && arms.iter().all(|arm| !arm.is_empty()
                        && motion
                            .bodies
                            .iter()
                            .any(|body| body.starts_with(&format!("{arm}/mesh-"))))
                    && motion.bodies.iter().all(|body| rune_bodies.insert(body)
                        && (body.starts_with(&hub_prefix)
                            || arms
                                .iter()
                                .any(|arm| body.starts_with(&format!("{arm}/mesh-"))))),
                "duplicate or unowned native Rune source geometry"
            );
        }
        for (name, count) in [("base", base_detectors), ("outpost", 3)] {
            let report = self
                .report
                .get(name)
                .context("missing detector source report")?;
            let unique: std::collections::BTreeSet<_> = report.detector_bodies.iter().collect();
            ensure!(
                report.detector_bodies.len() == count && unique.len() == count,
                "incomplete/duplicate detector source bodies"
            );
        }
        ensure!(
            self.outpost_motion.len() == 1 && self.base_motion.len() == 3,
            "incomplete 2027 joints"
        );
        for m in self.outpost_motion.iter().chain(&self.base_motion) {
            ensure!(
                !m.bodies.is_empty()
                    && m.bodies.iter().all(|b| {
                        b.strip_prefix("source_body_")
                            .or_else(|| {
                                (self.schema_version == 2)
                                    .then(|| b.strip_prefix("source_shell_"))
                                    .flatten()
                            })
                            .is_some_and(|id| id.parse::<u64>().is_ok_and(|id| id > 0))
                    })
                    && m.pivot_m.iter().chain(&m.axis).all(|v| v.is_finite())
                    && (m.axis.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-6,
                "invalid source motion binding"
            );
            if let Some(limits) = m.limits {
                ensure!(limits == [0., 0.25], "unknown base armor travel");
            }
        }
        if self.schema_version == 2 {
            let unique: std::collections::BTreeSet<_> = self.parts.iter().map(|p| &p.id).collect();
            ensure!(
                !self.parts.is_empty() && unique.len() == self.parts.len(),
                "missing/duplicate source parts"
            );
            for (asset, motions) in [
                ("base", &self.base_motion),
                ("outpost", &self.outpost_motion),
            ] {
                for motion in motions {
                    let names: std::collections::BTreeSet<_> = motion.bodies.iter().collect();
                    ensure!(
                        names.len() == motion.bodies.len(),
                        "duplicate moving source part"
                    );
                    for name in &motion.bodies {
                        ensure!(
                            self.parts.iter().any(|p| p.asset == asset
                                && p.visual_node.as_ref() == Some(name)
                                && p.collision_node.as_ref() == Some(name)
                                && p.motion_id.as_ref() == Some(&motion.id)),
                            "motion has no paired source part"
                        );
                    }
                }
            }
        }
        Ok(())
    }
    /// Source motion records for this runtime asset.
    pub fn motions<'a, B>(&'a self, cad: &CadAssets<B>, asset: &CadAsset) -> &'a [Motion] {
        if asset.file == cad.outpost.file {
            &self.outpost_motion
        } else if asset.file == cad.rune.file {
            &self.rune_motion
        } else if asset.file == cad.base.file {
            &self.base_motion
        } else {
            &[]
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn binding() -> Bindings {
        let detector = serde_json::json!({
            "pose": Pose::at([0., 0., 1.]), "half_m": [0.045, 0.044]
        });
        let tower = serde_json::json!({
            "pivot_m": [0., 0., 0.], "axis": [0., 0., 1.],
            "detectors": [detector.clone(), detector.clone(), detector.clone()]
        });
        let motion = serde_json::json!({
            "id": "outpost.spin", "bodies": ["source_body_1"],
            "axis": [0., 0., 1.], "pivot_m": [0., 0., 0.], "limits": null
        });
        let mut base_motion = motion.clone();
        base_motion["limits"] = serde_json::json!([0., 0.25]);
        serde_json::from_value(serde_json::json!({
            "schema_version": 1, "ruleset": "rmuc2027-v1.0.0",
            "source_sha256": "a5ad3ede3b7674fc98f2e73b71717216383cb2b4b5dcdbe900c2038918bea8e0",
            "world": {
                "bases": [[detector.clone()], [detector]],
                "outposts": [tower.clone(), tower], "zones": [], "rotor_direction": 1
            },
            "outpost_motion": [motion],
            "base_motion": [base_motion.clone(), base_motion.clone(), base_motion],
            "report": {
                "base": {"detector_bodies": [1]},
                "outpost": {"detector_bodies": [1, 2, 3]}
            }
        }))
        .unwrap()
    }

    #[test]
    fn standalone_bindings_validate_geometry_and_source_identity() {
        let valid = binding();
        valid.validate().unwrap();
        let mut invalid = valid.clone();
        invalid.world.bases[0][0].half_m[0] = 0.072;
        assert!(invalid.validate().is_err());
        let mut invalid = valid.clone();
        invalid.ruleset = "rmul2027-v1.0.0".into();
        assert!(invalid.validate().is_err());
        let mut invalid = valid.clone();
        invalid.source_sha256 = "unreviewed".into();
        assert!(invalid.validate().is_err());
        let mut invalid = valid;
        invalid.report.get_mut("outpost").unwrap().detector_bodies = vec![1, 1, 3];
        assert!(invalid.validate().is_err());
    }

    #[test]
    fn standalone_bindings_reject_incomplete_motion_and_unpaired_source_parts() {
        let valid = binding();
        let mut invalid = valid.clone();
        invalid.rune_motion.push(valid.outpost_motion[0].clone());
        assert!(invalid.validate().is_err());
        let mut invalid = valid.clone();
        invalid.base_motion[0].limits = Some([0., 0.5]);
        assert!(invalid.validate().is_err());
        let mut invalid = valid.clone();
        invalid.schema_version = 2;
        assert!(invalid.validate().is_err());
        let mut invalid = valid;
        invalid.outpost_motion[0].bodies = vec!["unknown".into()];
        assert!(invalid.validate().is_err());
    }

    #[test]
    fn rune_motion_accepts_its_hub_and_rejects_foreign_or_duplicate_parts() {
        let mut valid = binding();
        for (bank, id) in [("a", "rune.red.spin"), ("b", "rune.blue.spin")] {
            let arms = std::array::from_fn(|i| format!("rune/banks/bank-{bank}/arm-{i}"));
            let mut bodies: Vec<_> = arms.iter().map(|arm| format!("{arm}/mesh-1")).collect();
            bodies.push(format!("rune/banks/bank-{bank}/hub-cluster/mesh-2"));
            valid.rune_arms.push(arms);
            valid.rune_motion.push(Motion {
                id: id.into(),
                bodies,
                axis: [0., 0., 1.],
                pivot_m: [0.; 3],
                limits: None,
            });
        }
        valid.validate_source(1, 2).unwrap();
        let mut invalid = valid.clone();
        invalid.rune_motion[0]
            .bodies
            .push("rune/banks/bank-b/hub-cluster/mesh-3".into());
        assert!(invalid.validate_source(1, 2).is_err());
        let mut invalid = valid.clone();
        invalid.rune_motion[0]
            .bodies
            .push(valid.rune_motion[0].bodies[0].clone());
        assert!(invalid.validate_source(1, 2).is_err());
        let mut invalid = valid;
        invalid.rune_motion[0]
            .bodies
            .push("rune/banks/bank-a/fixed-logo/mesh-4".into());
        assert!(invalid.validate_source(1, 2).is_err());
    }
}
