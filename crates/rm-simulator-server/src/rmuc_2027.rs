// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Checksum-bound RMUC 2027 V1.0.0 source-body detector and motion bindings.
use crate::{
    cad_assets::{CadAsset, CadAssets},
    collision_mesh::{self, CollisionMesh},
    math,
};
use anyhow::{Context, ensure};
use rm_simulator_world::{
    referee::Mechanism,
    rmuc_2027::{Config, Zone},
};
use serde::Deserialize;

/// Exact source-body partition for one driven joint.
#[derive(Clone, Debug, PartialEq, Deserialize)]
pub struct Motion {
    /// Exported `extras.rm.id` of the moving node.
    pub id: String,
    /// Exact mesh-node names, each naming one source BREP body.
    pub bodies: Vec<String>,
    /// Unit axis in the source asset frame.
    pub axis: [f64; 3],
    /// Joint origin in source asset metres.
    pub pivot_m: [f64; 3],
    /// Prismatic travel in metres, or `None` for the outpost's continuous rotation.
    pub limits: Option<[f64; 2]>,
}
/// Source-pinned 2027 target geometry and motion partitions.
#[derive(Clone, Debug, PartialEq, Deserialize)]
pub struct Bindings {
    /// Binding schema; currently one.
    pub schema_version: u32,
    /// Competition and manual revision, never just a year.
    pub ruleset: String,
    /// Exact DJI source STEP hash.
    pub source_sha256: String,
    /// World FLU detector poses and rotor axes.
    pub world: Config,
    /// Outpost armor-module assemblies, excluding the shared fixed frame.
    pub outpost_motion: Vec<Motion>,
    /// Three base protective-armor source bodies.
    pub base_motion: Vec<Motion>,
}
impl Bindings {
    /// Validate identity, geometry and supported motion contracts before admission.
    pub fn validate(&self) -> anyhow::Result<()> {
        ensure!(
            self.schema_version == 1
                && self.ruleset == "rmuc2027-v1.0.0"
                && self.source_sha256
                    == "a5ad3ede3b7674fc98f2e73b71717216383cb2b4b5dcdbe900c2038918bea8e0",
            "unknown 2027 source/ruleset binding"
        );
        self.world.validate().map_err(anyhow::Error::msg)?;
        ensure!(
            self.outpost_motion.len() == 1 && self.base_motion.len() == 3,
            "incomplete 2027 joints"
        );
        for m in self.outpost_motion.iter().chain(&self.base_motion) {
            ensure!(
                !m.bodies.is_empty()
                    && m.bodies.iter().all(|b| b.starts_with("source_body_"))
                    && m.pivot_m.iter().chain(&m.axis).all(|v| v.is_finite())
                    && (m.axis.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-6,
                "invalid source motion binding"
            );
            if let Some(limits) = m.limits {
                ensure!(limits == [0., 0.25], "unknown base armor travel");
            }
        }
        Ok(())
    }
    /// Geometry with the reviewed figure footprints installed as antenna approximations.
    pub fn config(&self) -> Config {
        let mut config = self.world.clone();
        config.zones = crate::zone_catalog::areas(rm_simulator_world::season::Season::Rm2027)
            .into_iter()
            .filter_map(|a| {
                let point = a.control_point()?;
                Some(Zone {
                    side: a.side,
                    point,
                    floor_m: a.floor_m,
                    height_m: 0.45,
                    polygon_m: a.polygon_m,
                })
            })
            .collect();
        for a in crate::zone_catalog::areas(rm_simulator_world::season::Season::Rm2027) {
            let supply = a.id.ends_with("supply");
            if !supply && !a.id.ends_with("vantage") {
                continue;
            }
            let zone = Zone {
                side: a.side,
                point: rm_simulator_world::rmuc_2027::Point::Base,
                floor_m: a.floor_m,
                height_m: 0.45,
                polygon_m: a.polygon_m,
            };
            if supply {
                config.supply_zones.push(zone);
            } else {
                config.vantage_zones.push(zone);
            }
        }
        config
    }
    /// Source motion records for this runtime asset.
    pub fn motions<'a>(&'a self, cad: &CadAssets, asset: &CadAsset) -> &'a [Motion] {
        if asset.file == cad.outpost.file {
            &self.outpost_motion
        } else if asset.file == cad.base.file {
            &self.base_motion
        } else {
            &[]
        }
    }
}
/// Fit each approximate RFID height band to plausible source ground under its
/// footprint. Minima retain crowned/ramped edges; filtering by the diagram's
/// level avoids selecting a lower tunnel instead of its upper deck. The 10 mm
/// tolerance is a simulator antenna setting, not a rulebook detection range.
pub fn fit_zone_heights(config: &mut Config, ground: &crate::collision_mesh::GroundMesh) {
    for zone in config
        .zones
        .iter_mut()
        .chain(&mut config.supply_zones)
        .chain(&mut config.vantage_zones)
    {
        let n = zone.polygon_m.len() as f64;
        let center = [
            zone.polygon_m.iter().map(|p| p[0]).sum::<f64>() / n,
            zone.polygon_m.iter().map(|p| p[1]).sum::<f64>() / n,
        ];
        let mut samples = vec![center];
        for (a, b) in zone
            .polygon_m
            .iter()
            .zip(zone.polygon_m.iter().cycle().skip(1))
        {
            samples.push(*a);
            samples.push([(a[0] + b[0]) / 2., (a[1] + b[1]) / 2.]);
        }
        let heights = samples
            .into_iter()
            .filter_map(|p| ground.ground_height_below(p[0], p[1], zone.floor_m + 0.15))
            .filter(|&height| height >= zone.floor_m - 0.30)
            .collect::<Vec<_>>();
        if let Some(height) = heights.into_iter().min_by(f64::total_cmp) {
            zone.floor_m = height - 0.01;
        }
    }
}
/// Load only fixed source bodies, leaving driven bodies for their own colliders.
pub fn fixed_collision(cad: &CadAssets, asset: &CadAsset) -> anyhow::Result<CollisionMesh> {
    let motions = cad
        .rmuc_2027
        .as_ref()
        .map(|b| b.motions(cad, asset))
        .unwrap_or(&[]);
    let mut result = CollisionMesh::default();
    for part in collision_mesh::load_glb_parts(&asset.physics_file(&cad.root))? {
        if !motions.iter().any(|m| m.bodies.contains(&part.name)) {
            result.append(part.mesh);
        }
    }
    Ok(result)
}
/// Place exact driven source-body triangles with the same pivots as drawing/scoring.
pub fn mechanisms(
    cad: &CadAssets,
    asset: &CadAsset,
    output: &mut Vec<crate::layout::MechanismMesh>,
) -> anyhow::Result<()> {
    let Some(bindings) = &cad.rmuc_2027 else {
        return Ok(());
    };
    let parts = collision_mesh::load_glb_parts(&asset.physics_file(&cad.root))?;
    for motion in bindings.motions(cad, asset) {
        let mut mesh = CollisionMesh::default();
        for name in &motion.bodies {
            let part = parts
                .iter()
                .find(|p| &p.name == name)
                .with_context(|| format!("missing bound source body {name}"))?;
            mesh.append(part.mesh.clone());
        }
        for placement in &asset.placements {
            let mechanism = if motion.limits.is_none() {
                Mechanism::Outpost
            } else {
                Mechanism::Base
            };
            let frame = if let Some(limits) = motion.limits {
                let axis = math::rotate(math::gltf_rotation(placement), motion.axis);
                rm_simulator_world::referee::MotionFrame::Translation {
                    offsets_m: limits.map(|limit| axis.map(|v| v * limit)),
                }
            } else {
                rm_simulator_world::referee::MotionFrame::Rotation {
                    pivot_m: math::gltf_point(placement, motion.pivot_m),
                    axis: math::rotate(math::gltf_rotation(placement), motion.axis),
                }
            };
            output.push(crate::layout::MechanismMesh {
                mesh: mesh.placed(placement),
                mechanism,
                team: crate::layout::side_team(placement.translation_m[0]),
                motion: frame,
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use rm_simulator_world::{
        Pose, Team,
        rmuc_2027::{Detector, Point, Tower},
    };
    #[test]
    fn crowned_base_edges_count_but_lower_floor_cannot_activate_upper_highland() {
        let d = Detector {
            pose: Pose::at([0., 0., 1.]),
            half_m: [0.045, 0.044],
        };
        let mut config = Config {
            bases: [vec![d.clone()], vec![d.clone()]],
            outposts: std::array::from_fn(|_| Tower {
                pivot_m: [0.; 3],
                axis: [0., 0., 1.],
                detectors: vec![d.clone(); 3],
            }),
            rotor_direction: 1,
            supply_zones: vec![],
            vantage_zones: vec![],
            zones: vec![
                Zone {
                    side: Team::Red,
                    point: Point::Base,
                    floor_m: 0.2,
                    height_m: 0.45,
                    polygon_m: vec![[0., 0.], [1., 0.], [1., 1.], [0., 1.]],
                },
                Zone {
                    side: Team::Red,
                    point: Point::CentralHighland,
                    floor_m: 0.4,
                    height_m: 0.45,
                    polygon_m: vec![[3., 0.], [4., 0.], [4., 1.], [3., 1.]],
                },
            ],
        };
        let ground = crate::collision_mesh::GroundMesh::from(CollisionMesh {
            vertices_m: vec![
                [-1., -1., 0.],
                [5., -1., 0.],
                [5., 2., 0.],
                [-1., 2., 0.],
                [2., -1., 0.3],
                [5., -1., 0.3],
                [5., 2., 0.3],
                [2., 2., 0.3],
            ],
            triangles: vec![[0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7]],
        });
        assert!(!config.zones[0].contains([0.5, 0.5, 0.15]));
        fit_zone_heights(&mut config, &ground);
        assert!(config.zones[0].contains([0.5, 0.5, 0.15]));
        assert!(config.zones[1].contains([3.5, 0.5, 0.45]));
        assert!(!config.zones[1].contains([3.5, 0.5, 0.15]));
    }
}
