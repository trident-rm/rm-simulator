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
    if !folding_hero
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
    Ok(folding_hero)
}

#[cfg(test)]
mod tests {
    use super::*;

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
