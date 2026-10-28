// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Source-package construction for the approved PIE assembly binding.
//!
//! This loader verifies source identity and checksums, reads caller-owned
//! calibration types, and constructs an immutable physics collision scene.
//! It has no match authority, target sampling, worker, clock or playback state.
use rm_sim_physics::{
    Pose,
    manipulator::SerialArm,
    manipulator_collision::{CollisionAsset, CollisionScene},
};
use serde::{Deserialize, de::DeserializeOwned};
use sha2::{Digest, Sha256};
use std::path::Path;

/// Verified snapshot of source identity, file inventory and calibration/collision bytes.
/// Source calibration checks preserve the approved workbench bearing/jaw guards.
pub struct SourcePackage {
    config_bytes: Vec<u8>,
    collision_bytes: Vec<u8>,
    manifest: serde_json::Value,
}
impl SourcePackage {
    /// Verify all seven package files and retain the checked configuration/collision bytes.
    /// Later changes to the source files do not change this snapshot.
    pub fn load(root: &Path) -> Result<Self, Box<dyn std::error::Error>> {
        let m: serde_json::Value =
            serde_json::from_slice(&crate::package_io::read(root.join("manifest.json"))?)?;
        if m["schema"] != 1
            || m["profile"] != "pie-rmuc2027-assembly-workbench-v5"
            || m["source_step_sha256"]
                != "bfb91b499f17488ed1deb523568869f2fcb29436a53cbc39ec5e64ac4a822c47"
            || m["source_field_sha256"]
                != "a5ad3ede3b7674fc98f2e73b71717216383cb2b4b5dcdbe900c2038918bea8e0"
        {
            return Err("unsupported assembly CAD profile or source release".into());
        }
        let mut config_bytes = Vec::new();
        let mut collision_bytes = Vec::new();
        for (key, file) in [
            ("config", "assembly.json"),
            ("collision", "collision.json"),
            ("engineer", "engineer.glb"),
            ("core-base", "core-base.glb"),
            ("core-arm", "core-arm.glb"),
            ("core-head", "core-head.glb"),
            ("unit", "unit.glb"),
        ] {
            let bytes = crate::package_io::read(root.join(file))?;
            let hash = format!("{:x}", Sha256::digest(&bytes));
            if m["files"][key]["file"] != file || m["files"][key]["sha256"] != hash {
                return Err(format!("assembly CAD checksum mismatch: {file}").into());
            }
            match key {
                "config" => config_bytes = bytes,
                "collision" => collision_bytes = bytes,
                _ => {}
            }
        }
        Ok(Self {
            config_bytes,
            collision_bytes,
            manifest: m,
        })
    }
    /// Decode the shared mechanical model and validate both source and mechanical guards.
    /// No match attempt, playback state or runtime authority is constructed.
    pub fn model(
        &self,
    ) -> Result<rm_sim_physics::assembly_model::Model, Box<dyn std::error::Error>> {
        let model: rm_sim_physics::assembly_model::Model = self.calibration()?;
        model.validate()?;
        Ok(model)
    }

    /// Construct caller-owned calibration from verified bytes, checking arm/bearing/jaw guards.
    /// Runtime admission and task permissions remain the caller's responsibility.
    pub fn calibration<T: DeserializeOwned>(&self) -> Result<T, Box<dyn std::error::Error>> {
        let result = serde_json::from_slice(&self.config_bytes)?;
        let config: Calibration = serde_json::from_slice(&self.config_bytes)?;
        config.engineer.validate()?;
        config.core.validate()?;
        if config.schema != 5 || config.engineer_jaws.len() != 2 {
            return Err("invalid assembly calibration schema".into());
        }
        for (p, a) in [
            (config.core_p_origin_m, config.core_p_axis),
            (config.core_q_origin_m, config.core_q_axis),
            ([0.; 3], config.core_lift_axis),
        ]
        .into_iter()
        .chain(config.engineer_jaws.iter().map(|j| (j.origin_m, j.axis)))
        {
            if !p.into_iter().chain(a).all(f64::is_finite)
                || (a.into_iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-8
            {
                return Err("invalid assembly bearing calibration".into());
            }
        }
        if config.engineer_jaws.iter().any(|j| {
            !j.open_rad.is_finite()
                || !j.closed_rad.is_finite()
                || j.closed_rad.abs() >= j.open_rad.abs()
                || j.closed_rad * j.open_rad < 0.
                || !matches!(j.name.as_str(), "jaw-left" | "jaw-right")
        }) {
            return Err("invalid gripper jaw calibration".into());
        }
        Ok(result)
    }
    /// Configuration checksum used by recordings and area-frame bindings.
    pub fn config_sha256(&self) -> Option<&str> {
        self.manifest["files"]["config"]["sha256"].as_str()
    }
    /// Read a separately supplied source-bound area frame, rejecting nonfinite or nonunit poses.
    pub fn area_frame(&self, area: &Path) -> Result<Pose, Box<dyn std::error::Error>> {
        let evidence: serde_json::Value =
            serde_json::from_slice(&crate::package_io::read(area.join("area.json"))?)?;
        if evidence["schema"] != 2
            || evidence["config_sha256"] != self.manifest["files"]["config"]["sha256"]
            || evidence["source_field_sha256"] != self.manifest["source_field_sha256"]
        {
            return Err("assembly area frame/source mismatch".into());
        }
        let p: Pose = serde_json::from_value(evidence["assembly_frame"].clone())?;
        if !p
            .translation_m
            .into_iter()
            .chain(p.rotation_wxyz)
            .all(f64::is_finite)
            || (p.rotation_wxyz.into_iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-8
        {
            return Err("invalid assembly area frame".into());
        }
        Ok(p)
    }
    /// Build collision queries from verified bytes, checking collision/visual source identity.
    pub fn collision_scene(&self) -> Result<CollisionScene, Box<dyn std::error::Error>> {
        let collision_value: serde_json::Value = serde_json::from_slice(&self.collision_bytes)?;
        for key in ["engineer", "core-base", "core-arm", "core-head", "unit"] {
            let file = self.manifest["files"][key]["file"]
                .as_str()
                .ok_or("missing collision source")?;
            if collision_value["source_glbs"][file] != self.manifest["files"][key]["sha256"] {
                return Err("collision/visual source mismatch".into());
            }
        }
        Ok(CollisionScene::new(serde_json::from_value::<
            CollisionAsset,
        >(collision_value)?)?)
    }
}

// Read only the source mechanical fields needed for construction guards. The
// caller's complete calibration type remains independent of world snapshots.
#[derive(Deserialize)]
struct Calibration {
    schema: u32,
    engineer: SerialArm,
    core: SerialArm,
    engineer_jaws: Vec<Jaw>,
    core_p_origin_m: [f64; 3],
    core_p_axis: [f64; 3],
    core_q_origin_m: [f64; 3],
    core_q_axis: [f64; 3],
    core_lift_axis: [f64; 3],
}
#[derive(Deserialize)]
struct Jaw {
    name: String,
    origin_m: [f64; 3],
    axis: [f64; 3],
    open_rad: f64,
    closed_rad: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore = "requires RM_ASSEMBLY_PACKAGE and RM_ASSEMBLY_AREA source packages"]
    fn actual_source_package_validates_visual_model_collision_and_area() {
        let root = std::env::var_os("RM_ASSEMBLY_PACKAGE").expect("set RM_ASSEMBLY_PACKAGE");
        let area = std::env::var_os("RM_ASSEMBLY_AREA").expect("set RM_ASSEMBLY_AREA");
        let package = SourcePackage::load(Path::new(&root)).unwrap();
        let model = package.model().unwrap();
        model.validate().unwrap();
        package.collision_scene().unwrap();
        package.area_frame(Path::new(&area)).unwrap();
    }

    #[test]
    fn verified_snapshot_survives_source_replacement_and_deletion() {
        struct Fixture(std::path::PathBuf);
        impl Drop for Fixture {
            fn drop(&mut self) {
                let _ = std::fs::remove_dir_all(&self.0);
            }
        }
        let fixture = Fixture(std::env::temp_dir().join(format!(
            "rm-assembly-package-snapshot-{}",
            std::process::id()
        )));
        std::fs::create_dir(&fixture.0).unwrap();
        let root = &fixture.0;
        let arm = SerialArm {
            axes: [rm_sim_physics::manipulator::RevoluteAxis {
                origin_m: [0.; 3],
                axis: [0., 0., 1.],
                range_rad: [-3., 3.],
            }; 6],
            tool: Pose::default(),
        };
        let config = serde_json::json!({
            "schema": 5, "engineer": arm, "core": arm,
            "engineer_jaws": [
                {"name":"jaw-left", "origin_m":[0.,0.,0.], "axis":[0.,0.,1.], "open_rad":-0.5, "closed_rad":-0.1},
                {"name":"jaw-right", "origin_m":[0.,0.,0.], "axis":[0.,0.,1.], "open_rad":0.5, "closed_rad":0.1}
            ],
            "core_p_origin_m":[0.,0.,0.], "core_p_axis":[0.,0.,1.],
            "core_q_origin_m":[0.,0.,0.], "core_q_axis":[0.,0.,1.],
            "core_lift_axis":[0.,0.,1.]
        });
        let mut manifest = serde_json::json!({
            "schema":1, "profile":"pie-rmuc2027-assembly-workbench-v5",
            "source_step_sha256":"bfb91b499f17488ed1deb523568869f2fcb29436a53cbc39ec5e64ac4a822c47",
            "source_field_sha256":"a5ad3ede3b7674fc98f2e73b71717216383cb2b4b5dcdbe900c2038918bea8e0",
            "files":{}
        });
        let mut source_glbs = serde_json::Map::new();
        for key in ["engineer", "core-base", "core-arm", "core-head", "unit"] {
            let file = format!("{key}.glb");
            // This loader checks source identity, not GLB interpretation.
            let bytes = b"source geometry checksum fixture";
            std::fs::write(root.join(&file), bytes).unwrap();
            let hash = format!("{:x}", Sha256::digest(bytes));
            manifest["files"][key] = serde_json::json!({"file":file, "sha256":hash});
            source_glbs.insert(file, hash.into());
        }
        let collision = serde_json::json!({
            "schema":1, "source_glbs": source_glbs, "excluded_pairs":[],
            "bodies":[{"frame":"fixture/body", "parts":[{"kind":"box", "half_m":[0.1,0.1,0.1]}]}]
        });
        for (key, file, value) in [
            ("config", "assembly.json", &config),
            ("collision", "collision.json", &collision),
        ] {
            let bytes = serde_json::to_vec(value).unwrap();
            let hash = format!("{:x}", Sha256::digest(&bytes));
            std::fs::write(root.join(file), bytes).unwrap();
            manifest["files"][key] = serde_json::json!({"file":file, "sha256":hash});
        }
        std::fs::write(
            root.join("manifest.json"),
            serde_json::to_vec(&manifest).unwrap(),
        )
        .unwrap();
        let source = SourcePackage::load(root).unwrap();
        let checksum = source.config_sha256().unwrap().to_owned();
        assert_eq!(source.calibration::<serde_json::Value>().unwrap(), config);
        source.collision_scene().unwrap();

        std::fs::write(root.join("assembly.json"), b"{}").unwrap();
        std::fs::write(root.join("collision.json"), b"{}").unwrap();
        assert!(SourcePackage::load(root).is_err());
        assert_eq!(source.calibration::<serde_json::Value>().unwrap(), config);
        source.collision_scene().unwrap();
        assert_eq!(source.config_sha256(), Some(checksum.as_str()));

        std::fs::remove_file(root.join("assembly.json")).unwrap();
        std::fs::remove_file(root.join("collision.json")).unwrap();
        assert_eq!(source.calibration::<serde_json::Value>().unwrap(), config);
        source.collision_scene().unwrap();
        assert_eq!(source.config_sha256(), Some(checksum.as_str()));
    }
}
