// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Visual-only package parsing from caller-owned bytes, without filesystem I/O.
//! This validates package identity and placements, not collision or match admission.
use super::{
    FLOOR_TOP_CAD_M, Manifest, arena_pose, parse_manifest, placement_pose, validate_identity,
};
use anyhow::{Context, ensure};
use rm_sim_physics::Pose;
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;

/// One checksummed, self-contained visual file and its world placements.
#[derive(Clone, Debug)]
pub struct VisualAsset {
    /// Forward-slash path relative to the package root, suitable for an HTTP URL.
    pub file: String,
    /// World FLU poses in metres, including the package's floor offset.
    pub placements: Vec<Pose>,
    sha256: String,
}
impl VisualAsset {
    /// Verify the exact GLB bytes a renderer will consume.
    /// Rejects hash mismatches and external buffer/image URIs: every dependency
    /// must be inside these verified bytes, rather than fetched later unchecked.
    pub fn verify(&self, bytes: &[u8]) -> anyhow::Result<()> {
        ensure!(
            format!("{:x}", Sha256::digest(bytes)).eq_ignore_ascii_case(&self.sha256),
            "{}: visual checksum mismatch",
            self.file
        );
        ensure!(
            bytes.len() >= 20 && &bytes[..4] == b"glTF",
            "{}: expected a GLB",
            self.file
        );
        let word = |offset| u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap());
        ensure!(
            word(4) == 2 && word(8) as usize == bytes.len() && &bytes[16..20] == b"JSON",
            "{}: invalid GLB header",
            self.file
        );
        let end = 20usize
            .checked_add(word(12) as usize)
            .context("GLB JSON length overflow")?;
        let json: serde_json::Value =
            serde_json::from_slice(bytes.get(20..end).context("truncated GLB JSON")?)?;
        for kind in ["buffers", "images"] {
            if let Some(entries) = json[kind].as_array() {
                ensure!(
                    entries.iter().all(|entry| entry.get("uri").is_none()),
                    "{}: external {kind} are not verified",
                    self.file
                );
            }
        }
        Ok(())
    }
}

/// Render plan for a source-backed 2027 package, independent of its carrier.
#[derive(Clone, Debug)]
pub struct VisualPackage {
    /// Native RMUL format, or none for RMUC. This does not admit a match.
    pub rmul_format: Option<rm_sim_gameplay::rmul_2027::Format>,
    /// Unique visual files with all placements; duplicate files are fetched once.
    pub assets: Vec<VisualAsset>,
}
impl VisualPackage {
    /// Parse the arena and equipment manifest bytes received from one package.
    /// Shares schema, season/source identity and pose validation with the native
    /// loader. No collision recipes, bindings or source files are read.
    ///
    /// ```
    /// use rm_sim_assets::cad_assets::visual::VisualPackage;
    /// assert!(VisualPackage::from_manifests(b"{}", b"{}").is_err());
    /// ```
    pub fn from_manifests(arena: &[u8], equipment: &[u8]) -> anyhow::Result<Self> {
        let arena = parse_manifest(arena, "manifest.json")?;
        let equipment = parse_manifest(equipment, "equipment/manifest.json")?;
        let season = validate_identity(&arena, &equipment)?;
        ensure!(
            season == rm_sim_gameplay::season::Season::Rm2027,
            "browser visuals require a 2027 package"
        );
        let floor = arena.floor_top_source_z_m.unwrap_or(FLOOR_TOP_CAD_M);
        ensure!(
            (-3.0..=0.0).contains(&floor),
            "invalid manifest floor height"
        );
        let mut files = BTreeMap::<String, VisualAsset>::new();
        let mut add =
            |manifest: &Manifest, name: &str, prefix: &str, count: usize| -> anyhow::Result<()> {
                let asset = manifest
                    .assets
                    .get(name)
                    .with_context(|| format!("missing visual asset {name}"))?;
                ensure!(
                    asset.placements_in_source_arena_frame.len() == count,
                    "{name}: unexpected placement count"
                );
                if count == 0 && !["floor", "arena-static"].contains(&name) {
                    return Ok(());
                }
                ensure!(valid_path(&asset.visual), "{name}: invalid visual path");
                ensure!(
                    asset.visual_sha256.len() == 64
                        && asset.visual_sha256.bytes().all(|b| b.is_ascii_hexdigit()),
                    "{name}: invalid visual checksum"
                );
                let file = format!("{prefix}{}", asset.visual);
                let placements = if count == 0 {
                    vec![arena_pose(floor)]
                } else {
                    asset
                        .placements_in_source_arena_frame
                        .iter()
                        .map(|p| placement_pose(p, floor))
                        .collect::<anyhow::Result<Vec<_>>>()?
                };
                if let Some(previous) = files.get_mut(&file) {
                    ensure!(
                        previous.sha256.eq_ignore_ascii_case(&asset.visual_sha256),
                        "conflicting checksums for {file}"
                    );
                    previous.placements.extend(placements);
                } else {
                    files.insert(
                        file.clone(),
                        VisualAsset {
                            file,
                            sha256: asset.visual_sha256.clone(),
                            placements,
                        },
                    );
                }
                Ok(())
            };
        add(&arena, "floor", "", 0)?;
        // RMUL's complete field lives in floor.glb; placeholders are not rendered.
        if arena.rmul2027_format.is_none() {
            add(&arena, "arena-static", "", 0)?;
            add(&arena, "rune", "", 1)?;
            for (manifest, name, prefix) in [
                (&arena, "outpost", ""),
                (&equipment, "base", "equipment/"),
                (&equipment, "tech-core", "equipment/"),
            ] {
                add(manifest, name, prefix, 2)?;
            }
        }
        let mut names = std::collections::BTreeSet::new();
        for name in &arena.static_assets {
            ensure!(
                ![
                    "floor",
                    "arena-static",
                    "rune",
                    "outpost",
                    "base",
                    "tech-core"
                ]
                .contains(&name.as_str())
                    && names.insert(name),
                "duplicate or reserved static asset {name}"
            );
            let count = arena
                .assets
                .get(name)
                .context("missing static asset")?
                .placements_in_source_arena_frame
                .len();
            ensure!(count > 0, "static asset has no placements");
            add(&arena, name, "", count)?;
        }
        Ok(Self {
            rmul_format: arena.rmul2027_format,
            assets: files.into_values().collect(),
        })
    }
}
fn valid_path(file: &str) -> bool {
    file.ends_with(".glb")
        && !file.contains(['\\', ':', '?', '#', '%'])
        && file
            .split('/')
            .all(|part| !part.is_empty() && part != "." && part != "..")
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn glb(uri: Option<&str>) -> Vec<u8> {
        let mut scene = json!({"asset":{"version":"2.0"},"scenes":[{}],"scene":0});
        if let Some(uri) = uri {
            scene["buffers"] = json!([{"uri":uri,"byteLength":4}]);
        }
        let mut json = serde_json::to_vec(&scene).unwrap();
        while !json.len().is_multiple_of(4) {
            json.push(b' ');
        }
        let mut bytes = b"glTF".to_vec();
        bytes.extend(2u32.to_le_bytes());
        bytes.extend((20u32 + json.len() as u32).to_le_bytes());
        bytes.extend((json.len() as u32).to_le_bytes());
        bytes.extend(b"JSON");
        bytes.extend(json);
        bytes
    }
    fn manifests(file: &str, bytes: &[u8]) -> (Vec<u8>, Vec<u8>) {
        let asset = json!({"visual":file,"visual_sha256":format!("{:x}",Sha256::digest(bytes)),"placements_in_source_arena_frame":[]});
        let arena = json!({"schema_version":1,"units":"metres","season":"2027","rest_pose_only":true,"rmul2027_format":"Infantry","source_sha256":"954936e74885493836d73ba215e1c42efc3b6d40279478f5e5d626109e2cf6d1","floor_top_source_z_m":0.,"assets":{"floor":asset}});
        (
            serde_json::to_vec(&arena).unwrap(),
            serde_json::to_vec(&arena).unwrap(),
        )
    }
    #[test]
    fn supplied_bytes_are_verified_without_reopening_a_file() {
        let bytes = glb(None);
        let (arena, equipment) = manifests("floor.glb", &bytes);
        let plan = VisualPackage::from_manifests(&arena, &equipment).unwrap();
        assert_eq!(plan.assets.len(), 1);
        plan.assets[0].verify(&bytes).unwrap();
        assert_eq!(plan.assets[0].placements[0], arena_pose(0.));
        let mut changed = bytes;
        *changed.last_mut().unwrap() ^= 1;
        assert!(
            plan.assets[0]
                .verify(&changed)
                .unwrap_err()
                .to_string()
                .contains("checksum")
        );
    }
    #[test]
    fn url_escape_and_identity_mismatches_are_rejected_before_fetching() {
        for file in [
            "../floor.glb",
            "/floor.glb",
            "https://host/floor.glb",
            "a%2fb.glb",
            "a\\b.glb",
            "a.glb?x",
            "a//b.glb",
        ] {
            let (arena, equipment) = manifests(file, &glb(None));
            assert!(
                VisualPackage::from_manifests(&arena, &equipment).is_err(),
                "{file}"
            );
        }
        let (arena, equipment) = manifests("floor.glb", &glb(None));
        let mut wrong: serde_json::Value = serde_json::from_slice(&equipment).unwrap();
        wrong["rmul2027_format"] = json!("ThreeVsThree");
        assert!(
            VisualPackage::from_manifests(&arena, &serde_json::to_vec(&wrong).unwrap()).is_err()
        );
    }
    #[test]
    fn matching_hash_does_not_authorize_external_dependencies_or_truncated_glbs() {
        for bytes in [glb(Some("unverified.bin")), b"glTF".to_vec()] {
            let (arena, equipment) = manifests("floor.glb", &bytes);
            let plan = VisualPackage::from_manifests(&arena, &equipment).unwrap();
            assert!(plan.assets[0].verify(&bytes).is_err());
        }
    }
}
