// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Compatibility helpers for the shared checked robot package loader.
#[allow(unused_imports, reason = "preserve the app helper paths")]
pub use rm_sim_assets::robots::{is_stair_heavy, replacement_armor, verify};
use std::path::Path;

/// Read checked source configuration with the app's existing 500 ms transform
/// tuning. The shared assets loader retains the unmodified source configuration.
#[allow(dead_code, reason = "preserve the app helper path")]
pub fn source_chassis(root: &Path) -> std::io::Result<Option<rm_sim_world::ChassisConfig>> {
    let mut config = rm_sim_assets::robots::source_chassis(root)?;
    if let Some(c) = &mut config {
        c.set_transform_duration_s(0.5)
            .map_err(std::io::Error::other)?;
    }
    Ok(config)
}

/// Apply the app's existing transform tuning to a copy of a verified source
/// configuration. Legacy fitted presets remain selected by the host as before.
pub fn tuned_source_chassis(
    descriptor: &rm_sim_assets::robots::RobotDescriptor,
) -> std::io::Result<Option<rm_sim_world::ChassisConfig>> {
    if descriptor.source_configuration().is_none() {
        return Ok(None);
    }
    let mut config = descriptor.chassis().clone();
    config
        .set_transform_duration_s(0.5)
        .map_err(std::io::Error::other)?;
    Ok(Some(config))
}

#[cfg(test)]
mod tests {
    use super::*;
    use sha2::{Digest, Sha256};

    #[test]
    fn replacement_mounts_must_match_the_physical_profile() {
        let root = std::env::temp_dir().join(format!("rm-armor-asset-{}", std::process::id()));
        std::fs::create_dir_all(&root).unwrap();
        for hero in [false, true] {
            let config = if hero {
                rm_sim_world::chassis::ChassisConfig::folding_hero()
            } else {
                rm_sim_world::chassis::ChassisConfig::team_cad()
            };
            let modules: Vec<_> = config
                .armor_faces()
                .into_iter()
                .enumerate()
                .map(|(id, pose)| {
                    serde_json::json!({
                        "id": id, "kind": if hero { "AM12" } else { "AM02" },
                        "body_face_pose_flu": pose, "housing_half_m": config.armor_housing_half_m()
                    })
                })
                .collect();
            let mut manifest = serde_json::json!({"profile": if hero { "artisans-folding-hero-v1" } else { "omni" },
                "armor": {"schema": "robot-armor-v1", "runtime_replacement": true, "modules": modules}});
            let write = |value: &serde_json::Value| {
                std::fs::write(root.join("manifest.json"), value.to_string()).unwrap()
            };
            write(&manifest);
            assert!(replacement_armor(&root).unwrap());
            manifest["armor"]["modules"][0]["body_face_pose_flu"]["translation_m"][0] = 12.into();
            write(&manifest);
            assert!(replacement_armor(&root).is_err());
        }
        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn profile_and_mesh_integrity_are_both_required() {
        let root = std::env::temp_dir().join(format!("rm-team-asset-{}", std::process::id()));
        std::fs::create_dir_all(&root).unwrap();
        let mesh = b"test mesh bytes";
        std::fs::write(root.join("neutral.glb"), mesh).unwrap();
        let mut manifest = serde_json::json!({
            "schema": 1,
            "sources": {
                "chassis": {"cad_sha256": "b5a603acd8222295096184b68824eacf7302c7955e67943568dbc702a0611357"},
                "gimbal": {"cad_sha256": "47e6aab908c3fbb443b9f3d950e77144ea2bc7baf6e6aa9d8a754174e822ed7f"}
            },
            "files": {"neutral": {"file": "neutral.glb", "sha256": format!("{:x}", Sha256::digest(mesh))}}
        });
        let write = |m: &serde_json::Value| {
            std::fs::write(root.join("manifest.json"), m.to_string()).unwrap()
        };
        write(&manifest);
        verify(&root).unwrap();
        std::fs::write(root.join("neutral.glb"), b"changed").unwrap();
        assert!(verify(&root).unwrap_err().to_string().contains("checksum"));
        manifest["sources"]["chassis"]["cad_sha256"] = "wrong source".into();
        write(&manifest);
        assert!(verify(&root).unwrap_err().to_string().contains("profile"));
        std::fs::remove_dir_all(root).unwrap();
    }
}
