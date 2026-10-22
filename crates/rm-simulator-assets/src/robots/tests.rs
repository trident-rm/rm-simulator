// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
use super::*;
use rm_simulator_physics::mechanism::{ArticulationConfig, JointDrive, SourceJoint};
use std::{
    path::PathBuf,
    sync::atomic::{AtomicU64, Ordering},
};

struct Package(PathBuf);
impl Package {
    fn new() -> Self {
        static SERIAL: AtomicU64 = AtomicU64::new(0);
        let root = std::env::temp_dir().join(format!(
            "rm-robot-descriptor-{}-{}",
            std::process::id(),
            SERIAL.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::create_dir_all(&root).unwrap();
        Self(root)
    }
    fn manifest(&self) -> Value {
        read_manifest(&self.0).unwrap()
    }
    fn write_manifest(&self, manifest: &Value) {
        std::fs::write(
            self.0.join("manifest.json"),
            serde_json::to_vec(manifest).unwrap(),
        )
        .unwrap();
    }
    fn team() -> Self {
        let p = Self::new();
        std::fs::write(
            p.0.join("neutral.glb"),
            b"checksum fixture, not parsed geometry",
        )
        .unwrap();
        p.write_manifest(&serde_json::json!({
            "schema": 1, "sources": {
                "chassis": {"cad_sha256": "b5a603acd8222295096184b68824eacf7302c7955e67943568dbc702a0611357"},
                "gimbal": {"cad_sha256": "47e6aab908c3fbb443b9f3d950e77144ea2bc7baf6e6aa9d8a754174e822ed7f"}},
            "files": {"neutral": {"file": "neutral.glb", "sha256": format!("{:x}", Sha256::digest(b"checksum fixture, not parsed geometry"))}}
        }));
        p
    }
    fn source() -> Self {
        let p = Self::team();
        // Synthetic validator fixture only: this joint is not inferred native CAD.
        let base = ChassisConfig::default();
        let config = ChassisConfig {
            articulation: Some(ArticulationConfig {
                profile: "pie-tunnel-heavy-v1".into(),
                limit_rad: 1.0,
                speed_rad_s: 0.25,
                muzzle_offset_m: [0.; 3],
                camera_offset_m: [0.; 3],
                armor_faces: base.armor_faces(),
                joints: vec![SourceJoint {
                    name: "test-only-transform".into(),
                    parent: None,
                    pivot_m: [0.; 3],
                    axis: [0., 1., 0.],
                    angle_rad: 1.,
                    translation_m: [0.; 3],
                    drive: JointDrive::Transform,
                    collision: None,
                }],
            }),
            ..base
        };
        config.validate().unwrap();
        let bytes = serde_json::to_vec(&config).unwrap();
        std::fs::write(p.0.join("chassis.json"), &bytes).unwrap();
        let mut m = p.manifest();
        m["schema"] = 4.into();
        m["profile"] = "pie-tunnel-heavy-v1".into();
        m["source"] = serde_json::json!({"cad_sha256": "bfb91b499f17488ed1deb523568869f2fcb29436a53cbc39ec5e64ac4a822c47"});
        m["files"]["chassis"] = serde_json::json!({"file": "chassis.json", "sha256": format!("{:x}", Sha256::digest(bytes))});
        p.write_manifest(&m);
        p
    }
}
impl Drop for Package {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

#[test]
fn verified_snapshots_survive_mutation_and_identity_does_not_use_install_path() {
    let p = Package::team();
    let copy = Package::new();
    for name in ["manifest.json", "neutral.glb"] {
        std::fs::copy(p.0.join(name), copy.0.join(name)).unwrap();
    }
    let a = RobotDescriptor::load(&p.0).unwrap();
    let b = RobotDescriptor::load(&copy.0).unwrap();
    assert_eq!(a.identity_sha256(), b.identity_sha256());
    assert_eq!(
        a.configuration_origin(),
        ConfigurationOrigin::SimulatorFittedPreset
    );
    assert_eq!(a.chassis(), &ChassisConfig::team_cad());
    assert_eq!(a.sources().len(), 2);
    std::fs::write(p.0.join("neutral.glb"), b"replaced after verification").unwrap();
    assert!(RobotDescriptor::load(&p.0).is_err());
    assert_eq!(a.visual().bytes(), b.visual().bytes());
    assert!(Arc::ptr_eq(&a.visual.bytes, &a.clone().visual.bytes));
    std::fs::write(copy.0.join("native-properties.json"), b"untrusted sidecar").unwrap();
    assert_eq!(
        b.identity_sha256(),
        RobotDescriptor::load(&copy.0).unwrap().identity_sha256()
    );
}

#[test]
fn source_configuration_stays_raw_and_tuning_requires_an_explicit_clone() {
    let p = Package::source();
    let model = RobotDescriptor::load(&p.0).unwrap();
    assert_eq!(
        model.configuration_origin(),
        ConfigurationOrigin::SourceFittedConfiguration
    );
    assert_eq!(
        model.chassis().articulation.as_ref().unwrap().speed_rad_s,
        0.25
    );
    let original: ChassisConfig =
        serde_json::from_slice(model.source_configuration().unwrap().bytes()).unwrap();
    assert_eq!(model.chassis(), &original);
    assert_eq!(source_chassis(&p.0).unwrap(), Some(original.clone()));
    let mut tuned = original.clone();
    tuned.set_transform_duration_s(0.5).unwrap();
    assert_eq!(tuned.articulation.as_ref().unwrap().speed_rad_s, 2.);
    assert_eq!(model.chassis(), &original);
    assert_eq!(model.armor(), ArmorIdentity::Overlays);
}

#[test]
fn trusted_profile_checksums_and_local_references_are_independent_guards() {
    let p = Package::source();
    let valid = p.manifest();
    let mut m = valid.clone();
    m["source"]["cad_sha256"] = "wrong".into();
    p.write_manifest(&m);
    assert!(
        RobotDescriptor::load(&p.0)
            .unwrap_err()
            .to_string()
            .contains("profile")
    );
    // The narrow helper intentionally does not certify original CAD provenance.
    assert!(source_chassis(&p.0).unwrap().is_some());
    for reference in [
        "../neutral.glb",
        "/neutral.glb",
        "nested/neutral.glb",
        "C:\\neutral.glb",
        ".",
        "",
    ] {
        let mut m = valid.clone();
        m["files"]["preview"] = serde_json::json!({"file": reference, "sha256": "0".repeat(64)});
        p.write_manifest(&m);
        assert!(RobotDescriptor::load(&p.0).is_err(), "accepted {reference}");
    }
    let mut m = valid.clone();
    m["files"]["neutral"]["file"] = "other.glb".into();
    p.write_manifest(&m);
    assert!(RobotDescriptor::load(&p.0).is_err());
    p.write_manifest(&valid);
    std::fs::write(p.0.join("chassis.json"), b"changed").unwrap();
    assert!(
        RobotDescriptor::load(&p.0)
            .unwrap_err()
            .to_string()
            .contains("checksum")
    );
}

#[test]
fn matching_checksum_cannot_admit_invalid_config_or_wrong_articulation_profile() {
    let p = Package::source();
    let valid = std::fs::read(p.0.join("chassis.json")).unwrap();
    for (field, value) in [
        ("profile", serde_json::json!("wrong-source-profile")),
        ("speed_rad_s", serde_json::json!(-1.)),
    ] {
        let mut c: Value = serde_json::from_slice(&valid).unwrap();
        c["articulation"][field] = value;
        let bytes = serde_json::to_vec(&c).unwrap();
        std::fs::write(p.0.join("chassis.json"), &bytes).unwrap();
        let mut m = p.manifest();
        m["files"]["chassis"]["sha256"] = format!("{:x}", Sha256::digest(bytes)).into();
        p.write_manifest(&m);
        assert!(RobotDescriptor::load(&p.0).is_err());
    }
}

#[test]
fn replacement_mounts_and_legacy_helper_scope_are_preserved() {
    let p = Package::team();
    let mut m = p.manifest();
    let config = ChassisConfig::team_cad();
    m["armor"] = serde_json::json!({"schema":"robot-armor-v1", "runtime_replacement":true,
        "modules":config.armor_faces().into_iter().enumerate().map(|(id, pose)|serde_json::json!({
            "id":id,"kind":"AM02","body_face_pose_flu":pose,"housing_half_m":config.armor_housing_half_m()
        })).collect::<Vec<_>>()});
    p.write_manifest(&m);
    assert_eq!(
        RobotDescriptor::load(&p.0).unwrap().armor(),
        ArmorIdentity::Replacement {
            kind: ArmorKind::Am02
        }
    );
    std::fs::remove_file(p.0.join("neutral.glb")).unwrap();
    assert!(replacement_armor(&p.0).unwrap());
    assert!(RobotDescriptor::load(&p.0).is_err());
    m["armor"]["modules"][0]["body_face_pose_flu"]["translation_m"][0] = 12.into();
    p.write_manifest(&m);
    assert!(replacement_armor(&p.0).is_err());
}

#[cfg(unix)]
#[test]
fn consumed_files_cannot_be_symlink_references() {
    let p = Package::team();
    let outside = Package::team();
    std::fs::remove_file(p.0.join("neutral.glb")).unwrap();
    std::os::unix::fs::symlink(outside.0.join("neutral.glb"), p.0.join("neutral.glb")).unwrap();
    assert!(RobotDescriptor::load(&p.0).is_err());
}

#[test]
#[ignore = "requires RM_ROBOT_PACKAGES with external checked package paths"]
fn actual_supported_packages_preserve_raw_configuration_and_content_identity() {
    let paths = std::env::var_os("RM_ROBOT_PACKAGES")
        .expect("set RM_ROBOT_PACKAGES to package paths separated by the platform path separator");
    let mut count = 0;
    for root in std::env::split_paths(&paths) {
        let descriptor = RobotDescriptor::load(&root).unwrap();
        assert_eq!(
            verify(&root).unwrap(),
            descriptor.profile() == RobotProfile::ArtisansFoldingHero
        );
        assert_eq!(
            replacement_armor(&root).unwrap(),
            matches!(descriptor.armor(), ArmorIdentity::Replacement { .. })
        );
        let relocated = Package::new();
        for file in std::iter::once(descriptor.manifest())
            .chain(std::iter::once(descriptor.visual()))
            .chain(descriptor.source_configuration())
        {
            std::fs::write(relocated.0.join(file.name()), file.bytes()).unwrap();
        }
        assert_eq!(
            descriptor.identity_sha256(),
            RobotDescriptor::load(&relocated.0)
                .unwrap()
                .identity_sha256()
        );
        if let Some(bytes) = descriptor.source_configuration() {
            let raw: ChassisConfig = serde_json::from_slice(bytes.bytes()).unwrap();
            assert_eq!(descriptor.chassis(), &raw);
            let mut tuned = raw.clone();
            tuned.set_transform_duration_s(0.5).unwrap();
            tuned.validate().unwrap();
            assert_eq!(source_chassis(&root).unwrap(), Some(raw));
        }
        count += 1;
    }
    assert!(count > 0, "no package paths supplied");
}
