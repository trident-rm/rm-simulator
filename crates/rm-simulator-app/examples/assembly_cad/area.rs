// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Verified terrain extraction in the rig's unchanged equipment frame.
use super::*;
use rm_simulator_physics::manipulator_collision::CollisionPart;
use rm_simulator_server::collision_mesh::{CollisionMesh, GroundMesh};

#[derive(Clone)]
pub(super) struct Area {
    pub mesh: std::sync::Arc<GroundMesh>,
    pub evidence: serde_json::Value,
    stance_offset_m: f64,
}
impl Rig {
    pub(super) fn load_area(
        &mut self,
        package: &Path,
        directory: &Path,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let evidence: serde_json::Value =
            serde_json::from_slice(&std::fs::read(directory.join("area.json"))?)?;
        let hash = |p: PathBuf| -> Result<String, std::io::Error> {
            Ok(format!("{:x}", Sha256::digest(std::fs::read(p)?)))
        };
        let manifest: serde_json::Value =
            serde_json::from_slice(&std::fs::read(package.join("manifest.json"))?)?;
        if self.source_config_sha256.as_deref() != evidence["config_sha256"].as_str()
            || evidence["schema"] != 2
            || evidence["source_field_sha256"] != manifest["source_field_sha256"]
            || evidence["config_sha256"] != hash(package.join("assembly.json"))?
            || evidence["visual_sha256"] != hash(directory.join("area.glb"))?
            || evidence["collision_sha256"] != hash(directory.join("terrain.json"))?
            || manifest["files"]["collision"]["sha256"] != hash(package.join("collision.json"))?
        {
            return Err("assembly area source/checksum mismatch".into());
        }
        #[derive(Deserialize)]
        struct Terrain {
            points_m: Vec<[f64; 3]>,
            triangles: Vec<[u32; 3]>,
        }
        let terrain: Terrain =
            serde_json::from_slice(&std::fs::read(directory.join("terrain.json"))?)?;
        let mut collision: CollisionAsset =
            serde_json::from_slice(&std::fs::read(package.join("collision.json"))?)?;
        let bottom = collision
            .bodies
            .iter()
            .find(|b| b.frame == "engineer/body")
            .ok_or("missing chassis collision body")?
            .parts
            .iter()
            .flat_map(|p| match p {
                CollisionPart::Mesh { points_m, .. } | CollisionPart::Convex { points_m } => {
                    points_m.iter().map(|p| p[2]).collect::<Vec<_>>()
                }
                CollisionPart::Box { half_m } => vec![-half_m[2]],
            })
            .fold(f64::INFINITY, f64::min);
        // Static inspection stance: source tyre envelope plus 1 mm clearance.
        // Surface queries cannot distinguish supporting tangency from crossing.
        let stance_offset_m = -bottom + 0.001;
        if !stance_offset_m.is_finite() || !(0.05..0.3).contains(&stance_offset_m) {
            return Err("invalid source tyre envelope".into());
        }
        let floor = collision
            .bodies
            .iter_mut()
            .find(|b| b.frame == "floor")
            .ok_or("missing source floor body")?;
        floor.parts = vec![CollisionPart::Mesh {
            points_m: terrain.points_m.clone(),
            triangles: terrain.triangles.clone(),
        }];
        // The fixed pedestal is source-mounted into its plinth. It never moves;
        // preserve all moving Core/robot versus terrain checks.
        collision
            .excluded_pairs
            .push(["core/body".into(), "floor".into()]);
        let scene = CollisionScene::new(collision)?;
        let mut candidate = self.clone();
        candidate.area = Some(std::sync::Arc::new(Area {
            mesh: std::sync::Arc::new(GroundMesh::from(CollisionMesh {
                vertices_m: terrain.points_m,
                triangles: terrain.triangles,
            })),
            evidence,
            stance_offset_m,
        }));
        candidate.collision = Some(std::sync::Arc::new(scene));
        let initial_body = candidate.engineer_body;
        let mut supported = false;
        for forward in [0., 0.15, 0.3, 0.45, 0.6] {
            candidate.engineer_body = initial_body;
            candidate.engineer_body.translation_m[0] += forward;
            if candidate.support_parking() {
                supported = true;
                break;
            }
        }
        if !supported {
            return Err("no level assembly terrain beneath initial parking candidates".into());
        }
        let target = candidate.target([0.09, 0.2, 0.6], [0.; 3]);
        candidate.engineer_q = solve_initial(
            &candidate.config.engineer,
            compose(inverse(candidate.engineer_body), target),
        )?;
        candidate.initial_engineer_q = candidate.engineer_q;
        candidate.initial_engineer_body = candidate.engineer_body;
        candidate.tool = compose(
            candidate.engineer_body,
            candidate.config.engineer.tool_pose(candidate.engineer_q),
        );
        candidate.unit_pose = candidate.tool_unit_pose();
        // Prepare an open, source-compatible initial grasp. The unit stays at
        // the selected guide pose; this is before tick zero, not a live teleport.
        let mut clear = None;
        'grasps: for roll in [
            candidate.grasp_roll_rad,
            -candidate.grasp_roll_rad,
            0.,
            std::f64::consts::PI,
            std::f64::consts::FRAC_PI_4,
            -std::f64::consts::FRAC_PI_4,
            3. * std::f64::consts::FRAC_PI_4,
            -3. * std::f64::consts::FRAC_PI_4,
        ] {
            for axial in [0.02, 0.] {
                let mut trial = candidate.clone();
                trial.grasp_roll_rad = roll;
                trial.grasp_axial_m = axial;
                let target = compose(
                    inverse(trial.engineer_body),
                    trial.target([0.09, 0.2, 0.6], [0.; 3]),
                );
                for q in rm_simulator_physics::manipulator_plan::ik_candidates(
                    &trial.config.engineer,
                    target,
                    std::iter::once(trial.engineer_q)
                        .chain(planner::branch_seeds(&trial.config.engineer)),
                    12,
                ) {
                    trial.engineer_q = q;
                    trial.tool = compose(trial.engineer_body, trial.config.engineer.tool_pose(q));
                    trial.unit_pose = trial.tool_unit_pose();
                    if trial.collision_contacts()?.is_empty() {
                        clear = Some(trial);
                        break 'grasps;
                    }
                }
            }
        }
        if let Some(clear) = clear {
            candidate = clear;
        }
        candidate.initial_engineer_q = candidate.engineer_q;
        candidate.initial_engineer_body = candidate.engineer_body;
        candidate.last_collision = candidate.collision_contacts()?.into_iter().next();
        *self = candidate;
        Ok(())
    }
    /// Place a fixed planar chassis on a level terrain support patch; no driving is implied.
    pub(super) fn support_parking(&mut self) -> bool {
        let Some(area) = &self.area else {
            return true;
        };
        let mut heights = Vec::new();
        // Conservative four-corner wheel support footprint, a workbench policy.
        for x in [-0.25, 0.25] {
            for y in [-0.25, 0.25] {
                let p = position(self.engineer_body)
                    + quaternion(self.engineer_body) * bevy::math::DVec3::new(x, y, 0.);
                let Some(z) = area.mesh.ground_height_below(p.x, p.y, 0.6) else {
                    return false;
                };
                heights.push(z);
            }
        }
        let low = heights.iter().copied().fold(f64::INFINITY, f64::min);
        let high = heights.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        if high - low > 0.005 {
            return false;
        }
        self.engineer_body.translation_m[2] = high + area.stance_offset_m;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn moving_chassis_requires_support_between_clear_endpoints() {
        use rm_simulator_physics::manipulator_collision::{
            CollisionAsset, CollisionBody, CollisionPart,
        };
        let mut rig = super::super::tests::mechanism_fixture();
        let mut mesh = CollisionMesh::default();
        for (low, high) in [(-1., 0.24), (0.26, 1.)] {
            let base = mesh.vertices_m.len() as u32;
            mesh.vertices_m.extend([
                [low, -1., 0.],
                [high, -1., 0.],
                [high, 1., 0.],
                [low, 1., 0.],
            ]);
            mesh.triangles
                .extend([[base, base + 1, base + 2], [base, base + 2, base + 3]]);
        }
        rig.area = Some(std::sync::Arc::new(Area {
            mesh: std::sync::Arc::new(mesh.into()),
            evidence: serde_json::Value::Null,
            stance_offset_m: 0.133,
        }));
        rig.engineer_body.translation_m[0] = -0.05;
        assert!(rig.support_parking());
        rig.collision = Some(std::sync::Arc::new(
            CollisionScene::new(CollisionAsset {
                schema: 1,
                excluded_pairs: vec![],
                bodies: vec![
                    CollisionBody {
                        frame: "engineer/body".into(),
                        parts: vec![CollisionPart::Box { half_m: [0.01; 3] }],
                    },
                    CollisionBody {
                        frame: "floor".into(),
                        parts: vec![CollisionPart::Box {
                            half_m: [1., 1., 0.001],
                        }],
                    },
                ],
            })
            .unwrap(),
        ));
        let origin = rig.engineer_body;
        let mut next = rig.clone();
        next.engineer_body.translation_m[0] = 0.05;
        assert!(next.support_parking());
        assert!(rig.accept_candidate(next).unwrap_err().contains("support"));
        assert_eq!(rig.engineer_body, origin);
    }
    #[test]
    fn parking_requires_one_level_support_patch_and_preserves_rejected_pose() {
        let mut rig = super::super::tests::mechanism_fixture();
        let mut mesh = CollisionMesh::default();
        for (low, high, z) in [(-2., 0.3, 0.), (0.3, 2., 0.25)] {
            let base = mesh.vertices_m.len() as u32;
            mesh.vertices_m
                .extend([[low, -2., z], [high, -2., z], [high, 2., z], [low, 2., z]]);
            mesh.triangles
                .extend([[base, base + 1, base + 2], [base, base + 2, base + 3]]);
        }
        rig.area = Some(std::sync::Arc::new(Area {
            mesh: std::sync::Arc::new(mesh.into()),
            evidence: serde_json::Value::Null,
            stance_offset_m: 0.133,
        }));
        assert!(rig.support_parking());
        assert!((rig.engineer_body.translation_m[2] - 0.133).abs() < 1e-12);
        rig.engineer_body.translation_m[0] = 0.2;
        let previous = rig.engineer_body;
        assert!(!rig.support_parking());
        assert_eq!(rig.engineer_body, previous);
        rig.engineer_body.translation_m[0] = 0.6;
        assert!(rig.support_parking());
        assert!((rig.engineer_body.translation_m[2] - 0.383).abs() < 1e-12);
        rig.engineer_body.translation_m[0] = 3.;
        assert!(!rig.support_parking());
    }
}
