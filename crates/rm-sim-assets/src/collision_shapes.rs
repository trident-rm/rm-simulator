// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Verified opt-in hybrid collision recipes. Loading a prototype grants no contact approval.
use crate::cad_assets::read_verified_bytes;
use crate::collision_mesh::{load_glb_bytes, load_glb_selected_bytes};
use crate::math::{quat_axis_angle, quat_length, quat_multiply, quat_normalize, rotate};
use anyhow::{Context, anyhow, ensure};
use rm_sim_physics::Pose;
use serde::Deserialize;
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Component, Path, PathBuf};

/// Verified retained mesh and native parts of one unplaced asset.
#[derive(Clone, Debug, PartialEq)]
pub struct CollisionShapes {
    /// Provenance of the retained GLB. Loading uses the retained snapshot, never this path.
    pub remainder_file: PathBuf,
    /// Immutable snapshot whose hash, metadata and geometry were verified together.
    pub(crate) remainder_bytes: std::sync::Arc<[u8]>,
    /// Native solid cylinders, retaining their original motion ownership.
    pub parts: Vec<CylinderPart>,
    /// Optional verified mesh of replaced fixed owners, used only for sensor queries.
    pub(crate) query_mesh: Option<std::sync::Arc<crate::collision_mesh::CollisionMesh>>,
}

/// A cylinder in the original active GLB scene, before placement or CAD-to-FLU conversion.
#[derive(Clone, Debug, PartialEq)]
pub struct CylinderPart {
    /// Original mesh-node index; names alone are not unique selectors.
    pub source_node_index: usize,
    /// Exact raw source node name.
    pub source_node_name: String,
    /// Active-scene ancestry, including the selected node last.
    pub ancestor_nodes_root_first: Vec<usize>,
    /// Nearest source ancestor's component ownership, including the node itself.
    pub component_id: Option<String>,
    /// Metres and wxyz rotation, with native cylinder axis local +Y.
    /// This is an ordinary rigid pose, not the renderer's `gltf_root_pose` convention.
    pub pose: Pose,
    /// Positive finite radius in metres.
    pub radius_m: f64,
    /// Positive finite half-height in metres.
    pub half_height_m: f64,
}

#[derive(Deserialize)]
struct Recipe {
    schema: String,
    units: String,
    coordinate_frame: String,
    cylinder_local_axis: String,
    quaternion_order: String,
    source: FileHash,
    retained_mesh: RetainedMesh,
    #[serde(default)]
    query_mesh: Option<RetainedMesh>,
    replaced_node_indices: Vec<usize>,
    removed_mesh_triangles: usize,
    native_parts: Vec<NativePart>,
}
#[derive(Deserialize)]
struct FileHash {
    file: String,
    sha256: String,
}
#[derive(Deserialize)]
struct RetainedMesh {
    file: String,
    sha256: String,
    triangles: usize,
}
#[derive(Deserialize)]
struct NativePart {
    id: String,
    source_node_index: usize,
    source_node_name: String,
    ancestor_nodes_root_first: Vec<usize>,
    component_id: Option<String>,
    shape: Cylinder,
}
#[derive(Deserialize)]
struct Cylinder {
    kind: String,
    radius_m: f64,
    length_m: f64,
    pose: ExportPose,
    axis_unit_vector: [f64; 3],
    solid: bool,
}
#[derive(Deserialize)]
struct ExportPose {
    translation_m: [f64; 3],
    rotation_xyzw: [f64; 4],
}

/// Resolve a strictly relative local file, rejecting traversal and symlink escapes.
pub(crate) fn local_file(directory: &Path, file: &str) -> anyhow::Result<PathBuf> {
    ensure!(
        !file.is_empty()
            && !file.contains('\\')
            && Path::new(file)
                .components()
                .all(|c| matches!(c, Component::Normal(_))),
        "collision shape path must be local: {file:?}"
    );
    crate::verified::confined_path(directory, Path::new(file))
        .map(|_| directory.join(file))
        .with_context(|| {
            format!("collision shape path escapes its directory or is missing: {file:?}")
        })
}

fn check_hash(hash: &str) -> anyhow::Result<()> {
    ensure!(
        hash.len() == 64 && hash.bytes().all(|c| c.is_ascii_hexdigit()),
        "invalid SHA-256"
    );
    Ok(())
}

/// Verify a prototype recipe against the original collision GLB and manifest hashes.
/// The source remains authoritative for selectors; the retained mesh must preserve
/// all other node metadata, scene structure, motion records and collision geometry.
/// Returned poses use native local +Y and remain in the unplaced source scene.
pub fn load_verified(
    source_file: &Path,
    source_sha256: &str,
    sidecar_file: &Path,
    sidecar_sha256: &str,
) -> anyhow::Result<CollisionShapes> {
    check_hash(source_sha256)?;
    check_hash(sidecar_sha256)?;
    let sidecar_bytes = read_verified_bytes(sidecar_file, sidecar_sha256)?;
    let recipe: Recipe =
        serde_json::from_slice(&sidecar_bytes).context("collision shapes prototype contract")?;
    ensure!(
        ((recipe.schema == "rm-map-tools.collision-shapes.prototype-1"
            && recipe.query_mesh.is_none())
            || (recipe.schema == "rm-map-tools.collision-shapes.prototype-2"
                && recipe.query_mesh.is_some()))
            && recipe.units == "m"
            && recipe.coordinate_frame == "source_glb_active_scene"
            && recipe.cylinder_local_axis == "+Z"
            && recipe.quaternion_order == "xyzw",
        "unsupported collision shapes frame/schema"
    );
    // source.file is provenance, not a second source to load relative to the recipe.
    ensure!(
        !recipe.source.file.is_empty()
            && !recipe.source.file.contains('\\')
            && Path::new(&recipe.source.file)
                .components()
                .all(|c| matches!(c, Component::Normal(_))),
        "invalid source GLB provenance path"
    );
    check_hash(&recipe.source.sha256)?;
    ensure!(
        recipe.source.sha256.eq_ignore_ascii_case(source_sha256),
        "source GLB hash differs from manifest"
    );
    let source_bytes = read_verified_bytes(source_file, source_sha256)?;
    check_hash(&recipe.retained_mesh.sha256)?;
    let remainder_file = local_file(
        sidecar_file
            .parent()
            .ok_or_else(|| anyhow!("sidecar has no directory"))?,
        &recipe.retained_mesh.file,
    )?;
    let remainder_bytes = read_verified_bytes(&remainder_file, &recipe.retained_mesh.sha256)?;
    let source = document(&source_bytes)?;
    let remainder = document(&remainder_bytes)?;
    let ancestry = active_ancestry(&source)?;
    let replaced: BTreeSet<_> = recipe.replaced_node_indices.iter().copied().collect();
    ensure!(
        !replaced.is_empty() && replaced.len() == recipe.replaced_node_indices.len(),
        "empty or duplicate replaced node selectors"
    );
    let mut ids = BTreeSet::new();
    let mut covered = BTreeSet::new();
    let mut parts = Vec::new();
    for part in recipe.native_parts {
        ensure!(
            !part.id.is_empty() && ids.insert(part.id),
            "empty or duplicate native part ID"
        );
        let index = part.source_node_index;
        let chain = ancestry
            .get(&index)
            .ok_or_else(|| anyhow!("selected node is not in active scene"))?;
        let node = &source["nodes"][index];
        ensure!(
            replaced.contains(&index)
                && node["mesh"].as_u64().is_some()
                && node["name"].as_str() == Some(&part.source_node_name)
                && chain == &part.ancestor_nodes_root_first,
            "source node identity/ancestry differs"
        );
        let component = chain.iter().rev().find_map(|&i| {
            source["nodes"][i]["extras"]["component_id"]
                .as_str()
                .filter(|s| !s.is_empty())
        });
        ensure!(
            component == part.component_id.as_deref(),
            "source component ownership differs"
        );
        let shape = part.shape;
        ensure!(
            shape.kind == "cylinder",
            "unsupported native collision kind {}",
            shape.kind
        );
        ensure!(
            shape.solid
                && shape.radius_m.is_finite()
                && shape.radius_m > 0.0
                && shape.length_m.is_finite()
                && shape.length_m > 0.0
                && shape.length_m / 2.0 > 0.0,
            "cylinder must have positive finite solid dimensions"
        );
        let [x, y, z, w] = shape.pose.rotation_xyzw;
        let q = [w, x, y, z];
        ensure!(
            q.iter()
                .chain(&shape.pose.translation_m)
                .chain(&shape.axis_unit_vector)
                .all(|v| v.is_finite())
                && (quat_length(q) - 1.0).abs() <= 1e-8,
            "invalid cylinder pose/quaternion"
        );
        let axis = rotate(q, [0.0, 0.0, 1.0]);
        ensure!(
            axis.iter()
                .zip(shape.axis_unit_vector)
                .all(|(a, b)| (a - b).abs() <= 1e-8)
                && (shape
                    .axis_unit_vector
                    .iter()
                    .map(|v| v * v)
                    .sum::<f64>()
                    .sqrt()
                    - 1.0)
                    .abs()
                    <= 1e-8,
            "cylinder axis disagrees with pose"
        );
        covered.insert(index);
        parts.push(CylinderPart {
            source_node_index: index,
            source_node_name: part.source_node_name,
            ancestor_nodes_root_first: part.ancestor_nodes_root_first,
            component_id: part.component_id,
            pose: Pose {
                translation_m: shape.pose.translation_m,
                rotation_wxyz: quat_normalize(quat_multiply(
                    q,
                    quat_axis_angle([1.0, 0.0, 0.0], std::f64::consts::FRAC_PI_2),
                )),
            },
            radius_m: shape.radius_m,
            half_height_m: shape.length_m / 2.0,
        });
    }
    ensure!(
        covered == replaced,
        "native parts do not cover replaced nodes exactly"
    );
    verify_remainder(&source, &remainder, &replaced)?;
    let retained = load_glb_bytes(&remainder_bytes)?;
    let mut retained_selectors = BTreeSet::new();
    let mut removed_selectors = BTreeSet::new();
    for &index in ancestry.keys() {
        for primitive in 0..primitives(&source, index)?.len() {
            if replaced.contains(&index) {
                removed_selectors.insert((index, primitive));
            } else {
                retained_selectors.insert((index, primitive));
            }
        }
    }
    ensure!(
        load_glb_selected_bytes(&source_bytes, &retained_selectors)? == retained,
        "remainder changed retained collision geometry"
    );
    ensure!(
        retained.triangles.len() == recipe.retained_mesh.triangles
            && load_glb_selected_bytes(&source_bytes, &removed_selectors)?
                .triangles
                .len()
                == recipe.removed_mesh_triangles,
        "remainder/removed triangle counts differ"
    );
    let query_mesh = recipe
        .query_mesh
        .map(|query| -> anyhow::Result<_> {
            check_hash(&query.sha256)?;
            let path = local_file(
                sidecar_file.parent().context("sidecar has no directory")?,
                &query.file,
            )?;
            let bytes = read_verified_bytes(&path, &query.sha256)?;
            let mut doc = document(&bytes)?;
            ensure!(
                active_ancestry(&doc)? == ancestry,
                "query mesh changed source ancestry"
            );
            let owners: BTreeSet<_> = ancestry
                .keys()
                .copied()
                .filter(|i| doc["nodes"][*i].get("mesh").is_some())
                .collect();
            ensure!(
                owners == replaced,
                "query mesh must contain exactly the replaced owners"
            );
            let mesh = load_glb_bytes(&bytes)?;
            ensure!(
                query.triangles > 0 && mesh.triangles.len() == query.triangles,
                "query mesh triangle count differs"
            );
            for node in doc["nodes"].as_array_mut().context("missing query nodes")? {
                node.as_object_mut()
                    .context("invalid query node")?
                    .remove("mesh");
            }
            let all_nodes = (0..source["nodes"]
                .as_array()
                .context("missing source nodes")?
                .len())
                .collect();
            verify_remainder(&source, &doc, &all_nodes)?;
            Ok(std::sync::Arc::new(mesh))
        })
        .transpose()?;
    Ok(CollisionShapes {
        query_mesh,
        remainder_file,
        remainder_bytes: remainder_bytes.into(),
        parts,
    })
}

// Read only GLB framing/JSON here; triangle payload decoding belongs to collision_mesh.
fn document(bytes: &[u8]) -> anyhow::Result<Value> {
    let word = |offset| -> anyhow::Result<u32> {
        Ok(u32::from_le_bytes(
            bytes
                .get(offset..offset + 4)
                .ok_or_else(|| anyhow!("truncated GLB"))?
                .try_into()?,
        ))
    };
    ensure!(
        word(0)? == 0x4654_6c67 && word(4)? == 2 && word(8)? as usize == bytes.len(),
        "invalid GLB header"
    );
    ensure!(word(16)? == 0x4e4f_534a, "GLB first chunk must be JSON");
    Ok(serde_json::from_slice(
        bytes
            .get(20..20 + word(12)? as usize)
            .ok_or_else(|| anyhow!("truncated GLB JSON"))?,
    )?)
}

fn active_ancestry(doc: &Value) -> anyhow::Result<BTreeMap<usize, Vec<usize>>> {
    let nodes = doc["nodes"]
        .as_array()
        .ok_or_else(|| anyhow!("missing nodes"))?;
    let scene = doc.get("scene").map(index).transpose()?.unwrap_or(0);
    let roots = doc["scenes"][scene]["nodes"]
        .as_array()
        .ok_or_else(|| anyhow!("missing active scene"))?;
    let mut stack = roots
        .iter()
        .map(|v| Ok((index(v)?, Vec::new())))
        .collect::<anyhow::Result<Vec<_>>>()?;
    let mut result = BTreeMap::new();
    while let Some((i, mut chain)) = stack.pop() {
        ensure!(
            chain.len() < 64 && !result.contains_key(&i),
            "cyclic or multiply instanced source nodes"
        );
        let node = nodes.get(i).ok_or_else(|| anyhow!("invalid node index"))?;
        chain.push(i);
        if let Some(children) = node.get("children") {
            for child in children
                .as_array()
                .ok_or_else(|| anyhow!("invalid children"))?
            {
                stack.push((index(child)?, chain.clone()));
            }
        }
        result.insert(i, chain);
    }
    Ok(result)
}
fn index(value: &Value) -> anyhow::Result<usize> {
    value
        .as_u64()
        .and_then(|i| usize::try_from(i).ok())
        .ok_or_else(|| anyhow!("invalid glTF index"))
}
fn primitives(doc: &Value, node: usize) -> anyhow::Result<&Vec<Value>> {
    if let Some(mesh) = doc["nodes"][node].get("mesh") {
        doc["meshes"][index(mesh)?]["primitives"]
            .as_array()
            .ok_or_else(|| anyhow!("invalid mesh primitives"))
    } else {
        static EMPTY: Vec<Value> = Vec::new();
        Ok(&EMPTY)
    }
}
fn verify_remainder(
    source: &Value,
    remainder: &Value,
    replaced: &BTreeSet<usize>,
) -> anyhow::Result<()> {
    let mut expected = source.clone();
    let mut actual = remainder.clone();
    let source_nodes = source["nodes"]
        .as_array()
        .ok_or_else(|| anyhow!("missing source nodes"))?;
    ensure!(
        remainder["nodes"]
            .as_array()
            .is_some_and(|n| n.len() == source_nodes.len()),
        "remainder node count differs"
    );
    for (i, node) in source_nodes.iter().enumerate() {
        let before = primitives(source, i)?;
        let after = primitives(remainder, i)?;
        if replaced.contains(&i) {
            ensure!(
                remainder["nodes"][i].get("mesh").is_none(),
                "replaced mesh still present"
            );
        } else {
            ensure!(
                node.get("mesh").is_some() == remainder["nodes"][i].get("mesh").is_some()
                    && before.len() == after.len(),
                "retained mesh occurrence differs"
            );
            for (a, b) in before.iter().zip(after) {
                ensure!(
                    primitive_metadata(source, a)? == primitive_metadata(remainder, b)?,
                    "retained primitive metadata differs"
                );
            }
        }
        expected["nodes"][i]
            .as_object_mut()
            .ok_or_else(|| anyhow!("invalid source node"))?
            .remove("mesh");
        actual["nodes"][i]
            .as_object_mut()
            .ok_or_else(|| anyhow!("invalid remainder node"))?
            .remove("mesh");
    }
    for key in ["meshes", "accessors", "bufferViews", "buffers"] {
        expected
            .as_object_mut()
            .ok_or_else(|| anyhow!("invalid GLB document"))?
            .remove(key);
        actual
            .as_object_mut()
            .ok_or_else(|| anyhow!("invalid GLB document"))?
            .remove(key);
    }
    ensure!(
        expected == actual,
        "remainder changed source scene/node/motion metadata"
    );
    Ok(())
}
fn primitive_metadata(doc: &Value, primitive: &Value) -> anyhow::Result<Value> {
    let mut result = primitive.clone();
    if let Some(attributes) = result.get_mut("attributes") {
        for accessor in attributes
            .as_object_mut()
            .ok_or_else(|| anyhow!("invalid attributes"))?
            .values_mut()
        {
            *accessor = accessor_metadata(doc, accessor)?;
        }
    }
    if let Some(accessor) = result.get_mut("indices") {
        *accessor = accessor_metadata(doc, accessor)?;
    }
    Ok(result)
}
fn accessor_metadata(doc: &Value, reference: &Value) -> anyhow::Result<Value> {
    let mut accessor = doc["accessors"][index(reference)?].clone();
    let object = accessor
        .as_object_mut()
        .ok_or_else(|| anyhow!("invalid accessor"))?;
    object.remove("bufferView");
    object.remove("byteOffset");
    Ok(accessor)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use sha2::{Digest, Sha256};
    use std::sync::atomic::{AtomicU64, Ordering};

    fn hash(path: &Path) -> String {
        format!("{:x}", Sha256::digest(std::fs::read(path).unwrap()))
    }
    fn write_glb(path: &Path, doc: &Value) {
        let mut data = serde_json::to_vec(doc).unwrap();
        while !data.len().is_multiple_of(4) {
            data.push(b' ');
        }
        let mut bin = Vec::new();
        for value in [0.0_f32, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0] {
            bin.extend(value.to_le_bytes());
        }
        for i in [0_u32, 1, 2] {
            bin.extend(i.to_le_bytes());
        }
        let mut bytes = Vec::new();
        for word in [
            0x4654_6c67_u32,
            2,
            (12 + 8 + data.len() + 8 + bin.len()) as u32,
            data.len() as u32,
            0x4e4f_534a,
        ] {
            bytes.extend(word.to_le_bytes());
        }
        bytes.extend(data);
        bytes.extend((bin.len() as u32).to_le_bytes());
        bytes.extend(0x004e_4942_u32.to_le_bytes());
        bytes.extend(bin);
        std::fs::write(path, bytes).unwrap();
    }
    struct Fixture {
        directory: PathBuf,
        recipe: Value,
        source: Value,
        remainder: Value,
    }
    impl Fixture {
        fn new() -> Self {
            static NEXT: AtomicU64 = AtomicU64::new(0);
            let directory = std::env::temp_dir().join(format!(
                "rm-shapes-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir_all(&directory).unwrap();
            let source = json!({
                "asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0]}],
                "nodes":[{"name":"motion-owner","children":[1,2],"extras":{"component_id":"arm"}},
                    {"name":"repeated","mesh":0}, {"name":"repeated","mesh":0,"translation":[2,0,0]}],
                "meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1}]}],
                "accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"},
                    {"bufferView":1,"componentType":5125,"count":3,"type":"SCALAR"}],
                "bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":36},{"buffer":0,"byteOffset":36,"byteLength":12}],
                "buffers":[{"byteLength":48}]
            });
            let mut remainder = source.clone();
            remainder["nodes"][1]
                .as_object_mut()
                .unwrap()
                .remove("mesh");
            write_glb(&directory.join("source.glb"), &source);
            write_glb(&directory.join("remainder.glb"), &remainder);
            let recipe = json!({
                "schema":"rm-map-tools.collision-shapes.prototype-1","units":"m",
                "coordinate_frame":"source_glb_active_scene","cylinder_local_axis":"+Z","quaternion_order":"xyzw",
                "source":{"file":"source.glb","sha256":hash(&directory.join("source.glb"))},
                "retained_mesh":{"file":"remainder.glb","sha256":hash(&directory.join("remainder.glb")),"triangles":1},
                "replaced_node_indices":[1],"removed_mesh_triangles":1,
                "native_parts":[{"id":"node-1-0","source_node_index":1,"source_node_name":"repeated",
                    "ancestor_nodes_root_first":[0,1],"component_id":"arm",
                    "shape":{"kind":"cylinder","radius_m":0.2,"length_m":0.6,"solid":true,
                        "pose":{"translation_m":[1,2,3],"rotation_xyzw":[0,0,0,1]},"axis_unit_vector":[0,0,1]}}]
            });
            Self {
                directory,
                recipe,
                source,
                remainder,
            }
        }
        fn load(&self) -> anyhow::Result<CollisionShapes> {
            let sidecar = self.directory.join("shapes.json");
            std::fs::write(&sidecar, serde_json::to_vec(&self.recipe)?).unwrap();
            load_verified(
                &self.directory.join("source.glb"),
                self.recipe["source"]["sha256"].as_str().unwrap(),
                &sidecar,
                &hash(&sidecar),
            )
        }
        fn rewrite_remainder(&mut self) {
            let file = self.directory.join("remainder.glb");
            write_glb(&file, &self.remainder);
            self.recipe["retained_mesh"]["sha256"] = hash(&file).into();
        }
    }
    impl Drop for Fixture {
        fn drop(&mut self) {
            std::fs::remove_dir_all(&self.directory).unwrap();
        }
    }
    #[test]
    fn query_mesh_is_source_owned_verified_once_and_cannot_add_other_owners() {
        let mut f = Fixture::new();
        let path = f.directory.join("query.glb");
        let mut query = f.source.clone();
        query["nodes"][2].as_object_mut().unwrap().remove("mesh");
        write_glb(&path, &query);
        f.recipe["schema"] = json!("rm-map-tools.collision-shapes.prototype-2");
        f.recipe["query_mesh"] = json!({"file":"query.glb","sha256":hash(&path),"triangles":1});
        let loaded = f.load().unwrap();
        let recipe = f.recipe.clone();
        f.recipe["schema"] = json!("rm-map-tools.collision-shapes.prototype-1");
        assert!(
            f.load().is_err(),
            "query payload was accepted under the legacy contract"
        );
        f.recipe = recipe;
        assert_eq!(loaded.query_mesh.as_ref().unwrap().triangles.len(), 1);
        std::fs::write(&path, b"replaced after loading").unwrap();
        assert_eq!(loaded.query_mesh.as_ref().unwrap().triangles.len(), 1);
        assert!(f.load().is_err(), "changed query bytes were accepted");
        query["nodes"][2]["mesh"] = json!(0);
        write_glb(&path, &query);
        f.recipe["query_mesh"]["sha256"] = hash(&path).into();
        assert!(f.load().is_err(), "unselected owner was added to queries");
        query["nodes"][2].as_object_mut().unwrap().remove("mesh");
        query["nodes"][1]["translation"] = json!([1., 0., 0.]);
        write_glb(&path, &query);
        f.recipe["query_mesh"]["sha256"] = hash(&path).into();
        assert!(f.load().is_err(), "query owner transform changed");
    }

    #[test]
    fn verified_remainder_survives_source_sidecar_and_file_replacement() {
        let f = Fixture::new();
        let shapes = f.load().unwrap();
        let expected = load_glb_bytes(&shapes.remainder_bytes).unwrap();
        let asset = crate::cad_assets::CadAsset {
            file: "unused.glb".into(),
            semantics: None,
            placements: vec![],
            collision: Some(f.directory.join("source.glb")),
            collision_shapes: Some(shapes.clone()),
            source_tessellated_collision: true,
            bake_motion_endpoints: false,
        };
        for name in ["source.glb", "shapes.json", "remainder.glb"] {
            std::fs::write(f.directory.join(name), b"replaced after verification").unwrap();
        }
        assert_eq!(asset.physics_mesh(&f.directory).unwrap(), expected);
        let parts = crate::collision_mesh::load_glb_parts_bytes(
            &asset.physics_bytes(&f.directory).unwrap(),
        )
        .unwrap();
        assert_eq!(parts.len(), 1);
        std::fs::remove_file(&shapes.remainder_file).unwrap();
        assert_eq!(asset.physics_mesh(&f.directory).unwrap(), expected);
        assert!(
            load_verified(
                &f.directory.join("source.glb"),
                f.recipe["source"]["sha256"].as_str().unwrap(),
                &f.directory.join("shapes.json"),
                &hash(&f.directory.join("shapes.json"))
            )
            .is_err()
        );
    }

    #[test]
    fn shared_mesh_occurrence_is_removed_once_and_multiple_cylinders_cover_it() {
        let mut f = Fixture::new();
        let mut second = f.recipe["native_parts"][0].clone();
        second["id"] = "node-1-1".into();
        f.recipe["native_parts"]
            .as_array_mut()
            .unwrap()
            .push(second);
        let shapes = f.load().unwrap();
        assert_eq!(shapes.parts.len(), 2);
        assert_eq!(shapes.parts[0].half_height_m, 0.3);
        assert_eq!(shapes.parts[0].pose.translation_m, [1.0, 2.0, 3.0]);
        assert_eq!(shapes.parts[0].ancestor_nodes_root_first, [0, 1]);
        let axis = rotate(shapes.parts[0].pose.rotation_wxyz, [0.0, 1.0, 0.0]);
        assert!(
            axis.iter()
                .zip([0.0, 0.0, 1.0])
                .all(|(a, b)| (a - b).abs() < 1e-12)
        );
    }
    #[test]
    fn manifest_opt_in_loads_verified_remainder_and_malformed_recipe_never_falls_back() {
        let f = Fixture::new();
        f.load().unwrap();
        std::fs::create_dir_all(f.directory.join("equipment")).unwrap();
        std::fs::copy(
            f.directory.join("source.glb"),
            f.directory.join("visual.glb"),
        )
        .unwrap();
        std::fs::copy(
            f.directory.join("source.glb"),
            f.directory.join("equipment/visual.glb"),
        )
        .unwrap();
        let entry = |count: usize| {
            json!({
                "visual":"visual.glb", "visual_sha256":hash(&f.directory.join("source.glb")),
                "placements_in_source_arena_frame":vec![json!({"translation_m":[0,0,0],"rotation_xyzw":[0,0,0,1]});count]
            })
        };
        let mut arena = json!({"schema_version":1,"units":"metres","season":"2027","rest_pose_only":true,
            "assets":{"floor":entry(0),"arena-static":entry(0),"rune":entry(1),"outpost":entry(2)}});
        let equipment = json!({"schema_version":1,"units":"metres","season":"2027","rest_pose_only":true,
            "assets":{"base":entry(2),"tech-core":entry(2)}});
        let selected = &mut arena["assets"]["arena-static"];
        selected["collision"] = json!("source.glb");
        selected["collision_sha256"] = json!(hash(&f.directory.join("source.glb")));
        selected["collision_method"] = json!("primitive-shapes-v1");
        selected["collision_shapes"] = json!("shapes.json");
        selected["collision_shapes_sha256"] = json!(hash(&f.directory.join("shapes.json")));
        std::fs::write(
            f.directory.join("equipment/manifest.json"),
            equipment.to_string(),
        )
        .unwrap();
        std::fs::write(f.directory.join("manifest.json"), arena.to_string()).unwrap();
        let cad = crate::cad_assets::load(&f.directory).unwrap();
        assert!(cad.arena_static.collision_shapes.is_some());
        assert_eq!(
            cad.arena_static.collision.as_deref(),
            Some(f.directory.join("source.glb").as_path())
        );
        assert_eq!(
            cad.arena_static.physics_file(&f.directory),
            f.directory.join("remainder.glb")
        );
        assert!(cad.floor.collision_shapes.is_none());
        std::fs::write(
            f.directory.join("shapes.json"),
            b"malformed recognized recipe",
        )
        .unwrap();
        assert!(crate::cad_assets::load(&f.directory).is_err());
        // Legacy entries must not discover or attempt to load an undeclared sidecar.
        arena["assets"]["arena-static"]
            .as_object_mut()
            .unwrap()
            .remove("collision_method");
        std::fs::write(f.directory.join("manifest.json"), arena.to_string()).unwrap();
        let cad = crate::cad_assets::load(&f.directory).unwrap();
        assert!(cad.arena_static.collision_shapes.is_none());
        assert_eq!(
            cad.arena_static.physics_file(&f.directory),
            f.directory.join("visual.glb")
        );
    }

    #[test]
    fn compacted_mesh_indices_preserve_occurrences_and_geometry() {
        let mut f = Fixture::new();
        let second_mesh = f.source["meshes"][0].clone();
        f.source["meshes"].as_array_mut().unwrap().push(second_mesh);
        f.source["nodes"][2]["mesh"] = json!(1);
        write_glb(&f.directory.join("source.glb"), &f.source);
        f.recipe["source"]["sha256"] = hash(&f.directory.join("source.glb")).into();
        // The remainder has compacted mesh 1 to mesh 0 and discarded mesh 0.
        assert!(f.load().is_ok());
    }
    #[test]
    fn manifest_to_sidecar_source_hash_mismatch_fails() {
        let mut f = Fixture::new();
        let source_hash = hash(&f.directory.join("source.glb"));
        f.recipe["source"]["sha256"] = json!("0".repeat(64));
        let sidecar = f.directory.join("shapes.json");
        std::fs::write(&sidecar, serde_json::to_vec(&f.recipe).unwrap()).unwrap();
        assert!(
            load_verified(
                &f.directory.join("source.glb"),
                &source_hash,
                &sidecar,
                &hash(&sidecar)
            )
            .is_err()
        );
    }

    #[test]
    fn rotated_export_z_becomes_native_y_without_root_pose_conversion() {
        let mut f = Fixture::new();
        let q = quat_axis_angle([0.0, 1.0, 0.0], 0.73);
        f.recipe["native_parts"][0]["shape"]["pose"]["rotation_xyzw"] =
            json!([q[1], q[2], q[3], q[0]]);
        f.recipe["native_parts"][0]["shape"]["axis_unit_vector"] =
            json!(rotate(q, [0.0, 0.0, 1.0]));
        let p = f.load().unwrap().parts.remove(0);
        let got = rotate(p.pose.rotation_wxyz, [0.0, 1.0, 0.0]);
        assert!(
            got.iter()
                .zip(rotate(q, [0.0, 0.0, 1.0]))
                .all(|(a, b)| (a - b).abs() < 1e-12)
        );
    }
    #[test]
    fn malformed_dimensions_pose_axis_and_unsupported_kind_fail() {
        for (key, value) in [
            ("radius_m", json!(0)),
            ("radius_m", json!(-1)),
            ("length_m", json!(0)),
            ("length_m", json!("NaN")),
            ("solid", json!(false)),
            ("kind", json!("capsule")),
            ("axis_unit_vector", json!([1, 0, 0])),
            ("axis_unit_vector", json!([0, 0, 2])),
            (
                "pose",
                json!({"translation_m":[1,2,3],"rotation_xyzw":[0,0,0,2]}),
            ),
        ] {
            let mut fixture = Fixture::new();
            fixture.recipe["native_parts"][0]["shape"][key] = value;
            assert!(fixture.load().is_err(), "accepted invalid {key}");
        }
    }
    #[test]
    fn checksums_bind_sidecar_source_and_remainder() {
        let f = Fixture::new();
        f.load().unwrap();
        let sidecar = f.directory.join("shapes.json");
        assert!(
            load_verified(
                &f.directory.join("source.glb"),
                f.recipe["source"]["sha256"].as_str().unwrap(),
                &sidecar,
                &"0".repeat(64)
            )
            .is_err()
        );
        std::fs::write(f.directory.join("source.glb"), b"tampered source").unwrap();
        assert!(f.load().is_err());
        write_glb(&f.directory.join("source.glb"), &f.source);
        std::fs::write(f.directory.join("remainder.glb"), b"tampered remainder").unwrap();
        assert!(f.load().is_err());
    }
    #[test]
    fn duplicate_missing_or_forged_selectors_fail() {
        for case in 0..8 {
            let mut f = Fixture::new();
            match case {
                0 => f.recipe["replaced_node_indices"] = json!([1, 1]),
                1 => f.recipe["native_parts"] = json!([]),
                2 => {
                    let p = f.recipe["native_parts"][0].clone();
                    f.recipe["native_parts"].as_array_mut().unwrap().push(p);
                }
                3 => f.recipe["native_parts"][0]["source_node_index"] = json!(2),
                4 => f.recipe["native_parts"][0]["ancestor_nodes_root_first"] = json!([1]),
                5 => f.recipe["native_parts"][0]["component_id"] = json!("wrong-owner"),
                6 => f.recipe["native_parts"][0]["source_node_name"] = json!("wrong-name"),
                _ => f.recipe["replaced_node_indices"] = json!([1, 2]),
            }
            assert!(f.load().is_err(), "accepted selector case {case}");
        }
    }
    #[test]
    fn remainder_must_preserve_names_motion_geometry_and_exact_removal() {
        for case in 0..6 {
            let mut f = Fixture::new();
            match case {
                0 => f.remainder["nodes"][1]["mesh"] = json!(0),
                1 => {
                    f.remainder["nodes"][2]
                        .as_object_mut()
                        .unwrap()
                        .remove("mesh");
                }
                2 => f.remainder["nodes"][0]["name"] = json!("renamed-motion"),
                3 => f.remainder["nodes"][2]["translation"] = json!([3, 0, 0]),
                4 => f.recipe["retained_mesh"]["triangles"] = json!(2),
                _ => f.recipe["removed_mesh_triangles"] = json!(2),
            }
            f.rewrite_remainder();
            assert!(f.load().is_err(), "accepted remainder case {case}");
        }
        // Re-sign changed geometry: identity verification must still detect it.
        let mut f = Fixture::new();
        let path = f.directory.join("remainder.glb");
        let mut bytes = std::fs::read(&path).unwrap();
        let end = bytes.len();
        bytes[end - 48..end - 44].copy_from_slice(&0.5_f32.to_le_bytes());
        std::fs::write(&path, bytes).unwrap();
        f.recipe["retained_mesh"]["sha256"] = hash(&path).into();
        assert!(f.load().is_err());
    }
    #[test]
    fn frame_schema_and_nonlocal_paths_fail() {
        for (key, value) in [
            ("schema", json!("assembly-collision-v1")),
            ("units", json!("mm")),
            ("coordinate_frame", json!("FLU")),
            ("cylinder_local_axis", json!("+Y")),
            ("quaternion_order", json!("wxyz")),
        ] {
            let mut f = Fixture::new();
            f.recipe[key] = value;
            assert!(f.load().is_err());
        }
        for path in [
            "../remainder.glb",
            "/tmp/remainder.glb",
            "nested/../../remainder.glb",
            "..\\remainder.glb",
        ] {
            let mut f = Fixture::new();
            f.recipe["retained_mesh"]["file"] = json!(path);
            assert!(f.load().is_err());
        }
    }
    #[cfg(unix)]
    #[test]
    fn symlink_escape_is_not_a_local_sidecar_path() {
        let f = Fixture::new();
        std::os::unix::fs::symlink(std::env::temp_dir(), f.directory.join("outside")).unwrap();
        assert!(local_file(&f.directory, "outside").is_err());
    }
}
