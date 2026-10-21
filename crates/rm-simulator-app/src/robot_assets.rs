// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Verify the external, source-fitted robot package before the GPU imports it.
use sha2::{Digest, Sha256};
use std::path::Path;
/// The fitted mount profile is scoped to these exact source STEP releases.
pub fn verify(root: &Path) -> std::io::Result<bool> {
    let manifest: serde_json::Value =
        serde_json::from_slice(&std::fs::read(root.join("manifest.json"))?)?;
    let chassis = "b5a603acd8222295096184b68824eacf7302c7955e67943568dbc702a0611357";
    let gimbal = "47e6aab908c3fbb443b9f3d950e77144ea2bc7baf6e6aa9d8a754174e822ed7f";
    let folding_hero = manifest["schema"] == 2
        && manifest["profile"] == "artisans-folding-hero-v1"
        && manifest["source"]["cad_sha256"]
            == "71d6ddb07672028613c03ab2abca508cd616f93a3e2b5481b6696a063b86dd6e";
    let stair = manifest["schema"] == 3
        && manifest["profile"] == "dreamchaser-stair-heavy-v1"
        && manifest["source"]["cad_sha256"]
            == "dd98239dce94e35cb2ccc43171f4edc6e35c77f3f0659f91890750b98bad0e01";
    let tunnel = manifest["schema"] == 4
        && manifest["profile"] == "pie-tunnel-heavy-v1"
        && manifest["source"]["cad_sha256"]
            == "bfb91b499f17488ed1deb523568869f2fcb29436a53cbc39ec5e64ac4a822c47";
    let future = manifest["schema"] == 4
        && manifest["profile"] == "future-tunnel-infantry-v1"
        && manifest["source"]["cad_sha256"]
            == "d82b2c6950ec29d1d9609e1b5973d7981500be95bfb5f22b41b5eb68ced4bbbd";
    let sentry = manifest["schema"] == 4
        && manifest["profile"] == "ega-tunnel-sentry-v1"
        && manifest["source"]["cad_sha256"]
            == "c3aafbe2b06bd678a66cb769f2fcc825a851c56b4644d6b1d84a6389ad73b65b";
    if !folding_hero
        && !stair
        && !tunnel
        && !future
        && !sentry
        && (manifest["schema"] != 1
            || manifest["sources"]["chassis"]["cad_sha256"] != chassis
            || manifest["sources"]["gimbal"]["cad_sha256"] != gimbal
            || manifest["files"]["neutral"]["file"] != "neutral.glb")
    {
        return Err(std::io::Error::other(
            "robot package does not match the fitted team CAD profile",
        ));
    }
    let actual = format!(
        "{:x}",
        Sha256::digest(std::fs::read(root.join("neutral.glb"))?)
    );
    if manifest["files"]["neutral"]["sha256"] != actual {
        return Err(std::io::Error::other("robot neutral.glb checksum mismatch"));
    }
    source_chassis(root)?;
    replacement_armor(root)?;
    Ok(folding_hero)
}

/// Whether the verified package selects the source-bound stair Engineer.
pub fn is_stair_heavy(root: &Path) -> std::io::Result<bool> {
    let m: serde_json::Value = serde_json::from_slice(&std::fs::read(root.join("manifest.json"))?)?;
    Ok(m["profile"] == "dreamchaser-stair-heavy-v1")
}

/// Validate the optional replacement contract against the host's fitted profile.
/// Legacy packages retain CAD surfaces and receive artwork/light overlays only.
pub fn replacement_armor(root: &Path) -> std::io::Result<bool> {
    let manifest: serde_json::Value =
        serde_json::from_slice(&std::fs::read(root.join("manifest.json"))?)?;
    let Some(armor) = manifest.get("armor") else {
        return Ok(false);
    };
    let invalid =
        || std::io::Error::other("robot armor bindings do not match the fitted collision profile");
    if armor["schema"] != "robot-armor-v1" || armor["runtime_replacement"] != true {
        return Err(invalid());
    }
    let hero = manifest["profile"] == "artisans-folding-hero-v1";
    let config = if let Some(c) = source_chassis(root)? {
        c
    } else if manifest["profile"] == "dreamchaser-stair-heavy-v1" {
        rm_simulator_world::chassis::ChassisConfig::stair_heavy()
    } else if hero {
        rm_simulator_world::chassis::ChassisConfig::folding_hero()
    } else {
        rm_simulator_world::chassis::ChassisConfig::team_cad()
    };
    let modules = armor["modules"].as_array().ok_or_else(invalid)?;
    if modules.len() != 4 {
        return Err(invalid());
    }
    for ((index, module), face) in modules.iter().enumerate().zip(config.armor_faces()) {
        if module["id"] != index || module["kind"] != if hero { "AM12" } else { "AM02" } {
            return Err(invalid());
        }
        let pose: rm_simulator_world::Pose =
            serde_json::from_value(module["body_face_pose_flu"].clone()).map_err(|_| invalid())?;
        let half: [f64; 3] =
            serde_json::from_value(module["housing_half_m"].clone()).map_err(|_| invalid())?;
        if pose
            .translation_m
            .iter()
            .zip(face.translation_m)
            .any(|(a, b)| !a.is_finite() || (a - b).abs() > 0.0002)
            || half
                .iter()
                .zip(config.armor_housing_half_m())
                .any(|(a, b)| !a.is_finite() || (a - b).abs() > 0.000001)
            || pose.rotation_wxyz.iter().any(|a| !a.is_finite())
        {
            return Err(invalid());
        }
        let dot = pose
            .rotation_wxyz
            .iter()
            .zip(face.rotation_wxyz)
            .map(|(a, b)| a * b)
            .sum::<f64>();
        let norm = pose.rotation_wxyz.iter().map(|a| a * a).sum::<f64>();
        if (norm - 1.).abs() > 0.00001 || dot.abs() < 0.99999 {
            return Err(invalid());
        }
    }
    Ok(true)
}

/// Read and validate a checksummed source-fitted chassis configuration, if present.
pub fn source_chassis(root: &Path) -> std::io::Result<Option<rm_simulator_world::ChassisConfig>> {
    let m: serde_json::Value = serde_json::from_slice(&std::fs::read(root.join("manifest.json"))?)?;
    if m["schema"] != 4 {
        return Ok(None);
    }
    let bytes = std::fs::read(root.join("chassis.json"))?;
    if m["files"]["chassis"]["file"] != "chassis.json"
        || m["files"]["chassis"]["sha256"] != format!("{:x}", Sha256::digest(&bytes))
    {
        return Err(std::io::Error::other("source chassis checksum mismatch"));
    }
    let mut c: rm_simulator_world::ChassisConfig = serde_json::from_slice(&bytes)?;
    c.validate().map_err(std::io::Error::other)?;
    if c.articulation
        .as_ref()
        .is_none_or(|a| Some(a.profile.as_str()) != m["profile"].as_str())
    {
        return Err(std::io::Error::other(
            "source articulation profile mismatch",
        ));
    }
    // Runtime actuator tuning, applied after verifying the unmodified source package.
    c.set_transform_duration_s(0.5)
        .map_err(std::io::Error::other)?;
    Ok(Some(c))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replacement_mounts_must_match_the_physical_profile() {
        let root = std::env::temp_dir().join(format!("rm-armor-asset-{}", std::process::id()));
        std::fs::create_dir_all(&root).unwrap();
        for hero in [false, true] {
            let config = if hero {
                rm_simulator_world::chassis::ChassisConfig::folding_hero()
            } else {
                rm_simulator_world::chassis::ChassisConfig::team_cad()
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
