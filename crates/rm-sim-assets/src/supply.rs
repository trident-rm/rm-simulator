// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Source-bound resource-station Unit placements, separate from static scenery.
use crate::{cad_assets::CadAssets, collision_mesh::CollisionMesh};
use anyhow::{Context, ensure};
use glam::{DMat4, DQuat, DVec3};
use rm_sim_physics::{Pose, manipulator_collision::CollisionPart, projectile::PayloadShape};

/// One source-placed persistent Unit, with an outward +z bore axis.
#[derive(Clone, Debug)]
pub struct SourceUnit {
    /// Exact source mesh-node name, used to move the existing visual instance.
    pub visual_node: String,
    /// Unit-centred world FLU pose, metres and wxyz quaternion.
    pub pose: Pose,
    /// Source convex outer proxy, nominal 400 g; the bore may be filled.
    pub shape: PayloadShape,
}

/// Separate the twelve authored Units from a verified 2027 arena visual snapshot.
/// Absent Unit components leave older/non-RM packages unchanged. Present bindings
/// must contain six Units on each side; malformed partial bindings fail closed.
/// Local source +y is the bore axis and its midpoint is 75 mm from the origin,
/// from the standalone 150 mm Unit CAD (UC Figure 4-39).
///
/// # Errors
/// Rejects changed file hashes, malformed transforms, partial stock or invalid proxies.
pub fn load<B>(cad: &CadAssets<B>) -> anyhow::Result<Vec<SourceUnit>> {
    let manifest: serde_json::Value =
        serde_json::from_slice(&crate::package_io::read(cad.root.join("manifest.json"))?)?;
    let entry = &manifest["assets"]["arena-static"];
    let expected = entry["visual_sha256"]
        .as_str()
        .context("missing arena visual checksum")?;
    let bytes =
        crate::cad_assets::read_verified_bytes(&cad.root.join(&cad.arena_static.file), expected)?;
    let length =
        u32::from_le_bytes(bytes.get(12..16).context("truncated GLB")?.try_into()?) as usize;
    let document: serde_json::Value = serde_json::from_slice(
        bytes
            .get(20..20usize.checked_add(length).context("GLB size overflow")?)
            .context("truncated GLB JSON")?,
    )?;
    let nodes = document["nodes"].as_array().context("missing GLB nodes")?;
    let scene = document["scene"].as_u64().unwrap_or(0) as usize;
    let roots = document["scenes"][scene]["nodes"]
        .as_array()
        .context("missing GLB scene")?;
    let mut stack = roots
        .iter()
        .rev()
        .map(|n| {
            Ok((
                n.as_u64().context("invalid scene node")? as usize,
                DMat4::IDENTITY,
                0,
            ))
        })
        .collect::<anyhow::Result<Vec<_>>>()?;
    let mut frames = Vec::new();
    while let Some((index, parent, depth)) = stack.pop() {
        ensure!(depth < 64, "cyclic or deep supply scene");
        let node = nodes.get(index).context("invalid supply node")?;
        let local = if let Some(matrix) = node.get("matrix") {
            let columns: [f64; 16] = serde_json::from_value(matrix.clone())?;
            DMat4::from_cols_array(&columns)
        } else {
            let scale = node
                .get("scale")
                .cloned()
                .map(serde_json::from_value::<[f64; 3]>)
                .transpose()?
                .unwrap_or([1.; 3]);
            let rotation = node
                .get("rotation")
                .cloned()
                .map(serde_json::from_value::<[f64; 4]>)
                .transpose()?
                .unwrap_or([0., 0., 0., 1.]);
            let translation = node
                .get("translation")
                .cloned()
                .map(serde_json::from_value::<[f64; 3]>)
                .transpose()?
                .unwrap_or([0.; 3]);
            DMat4::from_scale_rotation_translation(
                DVec3::from_array(scale),
                DQuat::from_array(rotation),
                DVec3::from_array(translation),
            )
        };
        let world = parent * local;
        if node["extras"]["component_id"] == "energy-unit/unit" && node.get("mesh").is_some() {
            let name = node["name"]
                .as_str()
                .context("unnamed source Unit")?
                .to_owned();
            let (scale, _, _) = world.to_scale_rotation_translation();
            ensure!(
                world.is_finite() && scale.distance(DVec3::ONE) < 1e-6,
                "nonrigid source Unit"
            );
            let frame = world
                * DMat4::from_translation(DVec3::new(0., 0.075, 0.))
                * DMat4::from_quat(DQuat::from_rotation_x(-std::f64::consts::FRAC_PI_2));
            let (_, rotation, translation) = frame.to_scale_rotation_translation();
            // Arena GLBs store source CAD coordinates directly, not glTF Y-up.
            let pose = Pose {
                translation_m: crate::cad_assets::cad_point_to_flu(
                    translation.to_array(),
                    cad.floor_top_cad_m,
                ),
                rotation_wxyz: [rotation.w, rotation.x, rotation.y, rotation.z],
            };
            pose.validate_with(sim_engine::pose::PoseTolerance::Asset)?;
            frames.push((name, pose));
        }
        if let Some(children) = node["children"].as_array() {
            for child in children.iter().rev() {
                stack.push((
                    child.as_u64().context("invalid child")? as usize,
                    world,
                    depth + 1,
                ));
            }
        }
    }
    if frames.is_empty() {
        return Ok(Vec::new());
    }
    ensure!(
        frames.len() == 12
            && frames
                .iter()
                .filter(|(_, p)| p.translation_m[0] > 0.)
                .count()
                == 6,
        "supply source must contain six Units per side"
    );
    ensure!(
        frames
            .iter()
            .map(|(name, _)| name)
            .collect::<std::collections::BTreeSet<_>>()
            .len()
            == 12,
        "duplicate source Unit names"
    );
    let parts = crate::collision_mesh::load_glb_parts_bytes(&bytes)?;
    frames
        .into_iter()
        .map(|(visual_node, pose)| {
            let source = parts
                .iter()
                .find(|p| p.name == visual_node)
                .context("missing Unit source triangles")?;
            let rotation = DQuat::from_xyzw(
                pose.rotation_wxyz[1],
                pose.rotation_wxyz[2],
                pose.rotation_wxyz[3],
                pose.rotation_wxyz[0],
            );
            let points_m = source
                .mesh
                .clone()
                .into_flu(cad.floor_top_cad_m)
                .vertices_m
                .into_iter()
                .map(|p| {
                    (rotation.inverse()
                        * (DVec3::from_array(p) - DVec3::from_array(pose.translation_m)))
                    .to_array()
                })
                .collect();
            let shape = PayloadShape::from_parts(&[CollisionPart::Convex { points_m }], 0.4)
                .map_err(anyhow::Error::msg)?;
            Ok(SourceUnit {
                visual_node,
                pose,
                shape,
            })
        })
        .collect()
}

/// Decode scenery while excluding precisely the source Unit mesh nodes.
/// Call only after [`load`] has admitted the complete source inventory.
pub(crate) fn without_units(bytes: &[u8], units: &[SourceUnit]) -> anyhow::Result<CollisionMesh> {
    let mut mesh = CollisionMesh::default();
    let mut removed = std::collections::BTreeSet::new();
    for part in crate::collision_mesh::load_glb_parts_bytes(bytes)? {
        if units.iter().any(|unit| unit.visual_node == part.name) {
            ensure!(
                removed.insert(part.name),
                "duplicate supply collision owner"
            );
        } else {
            mesh.append(part.mesh);
        }
    }
    ensure!(
        removed.len() == units.len(),
        "supply collision owners differ from source visuals"
    );
    Ok(mesh)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    #[ignore = "requires the verified field-2027 CAD package"]
    fn source_axes_stock_and_static_exclusion_match_authored_units() {
        let path = std::env::var_os("RM_ARENA_PACKAGE")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|| std::path::PathBuf::from("../../field-2027"));
        let cad = crate::cad_assets::load(&path).unwrap();
        let units = load(&cad).unwrap();
        assert_eq!(units.len(), 12);
        for station in units.as_chunks::<6>().0.iter() {
            let center = station
                .iter()
                .map(|u| DVec3::from_array(u.pose.translation_m))
                .sum::<DVec3>()
                / 6.;
            for unit in station {
                let [w, x, y, z] = unit.pose.rotation_wxyz;
                let axis = DQuat::from_xyzw(x, y, z, w) * DVec3::Z;
                let radial = (DVec3::from_array(unit.pose.translation_m) - center).normalize();
                assert!(
                    axis.dot(radial) > 0.99,
                    "{}: axis {axis:?}, radial {radial:?}",
                    unit.visual_node
                );
                assert_eq!(unit.shape.mass_kg, 0.4);
            }
        }
        for bytes in [
            std::fs::read(path.join(&cad.arena_static.file)).unwrap(),
            cad.arena_static.physics_bytes(&path).unwrap().into_owned(),
        ] {
            let parts = crate::collision_mesh::load_glb_parts_bytes(&bytes).unwrap();
            let retained = without_units(&bytes, &units).unwrap();
            let expected: usize = parts
                .iter()
                .filter(|p| !p.name.starts_with("energy-unit/unit/"))
                .map(|p| p.mesh.triangles.len())
                .sum();
            assert_eq!(retained.triangles.len(), expected);
            assert_eq!(
                parts
                    .iter()
                    .filter(|p| p.name.starts_with("energy-unit/unit/"))
                    .count(),
                12
            );
            assert!(parts.iter().map(|p| p.mesh.triangles.len()).sum::<usize>() > expected);
        }
    }
}
