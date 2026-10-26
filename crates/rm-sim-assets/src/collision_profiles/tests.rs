// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
use super::*;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::path::PathBuf;
struct Fixture {
    root: PathBuf,
    manifest: Value,
}
impl Fixture {
    fn new() -> Self {
        static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let root = std::env::temp_dir().join(format!(
            "rm-profiles-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
        ));
        std::fs::create_dir_all(&root).unwrap();
        let mut bin = Vec::new();
        for value in [2_f32, -1., -1., 2., 1., -1., 2., 0., 1.] {
            bin.extend(value.to_le_bytes());
        }
        for index in [0_u32, 1, 2] {
            bin.extend(index.to_le_bytes());
        }
        let doc = json!({"asset":{"version":"2.0"},"buffers":[{"byteLength":bin.len()}],"bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":36},{"buffer":0,"byteOffset":36,"byteLength":12}],"accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"},{"bufferView":1,"componentType":5125,"count":3,"type":"SCALAR"}],"meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1}]}],"nodes":[{"mesh":0}],"scenes":[{"nodes":[0]}],"scene":0});
        let mut text = serde_json::to_vec(&doc).unwrap();
        while !text.len().is_multiple_of(4) {
            text.push(b' ');
        }
        let mut glb = Vec::new();
        for x in [
            0x46546c67_u32,
            2,
            (12 + 8 + text.len() + 8 + bin.len()) as u32,
            text.len() as u32,
            0x4e4f534a,
        ] {
            glb.extend(x.to_le_bytes());
        }
        glb.extend(text);
        glb.extend((bin.len() as u32).to_le_bytes());
        glb.extend(0x004e4942_u32.to_le_bytes());
        glb.extend(bin);
        std::fs::write(root.join("mesh.glb"), &glb).unwrap();
        let native = json!({"units":"metres","shape_axis":"+Y","quaternion_order":"wxyz","coordinate_frame":"same part-local frame as reviewed.glb","source_sha256":"a".repeat(64),"children":[{"name":"shaft","kind":"cylinder","radius_m":0.3,"half_height_m":0.5,"translation_m":[3.,0.,0.],"rotation_wxyz":[1.,0.,0.,0.]}]});
        let nb = serde_json::to_vec(&native).unwrap();
        std::fs::write(root.join("native.json"), &nb).unwrap();
        let mesh = json!({"id":"mesh","kind":"triangle-mesh-v1","priority":0,"requires":["triangle-mesh-v1"],"approval":"reviewed","files":[{"path":"mesh.glb","sha256":format!("{:x}",Sha256::digest(&glb))}]});
        let primitive = json!({"id":"native","kind":"convex-primitives-v1","priority":100,"requires":["compound-convex-v1","cylinder-y-v1"],"approval":"experimental","files":[{"path":"native.json","sha256":format!("{:x}",Sha256::digest(&nb))}]});
        let manifest = json!({"schema":"reviewed-field-collision-choices-v1","world_frame":"right-handed glTF Y-up, metre; source CAD world rotated -90 degrees about X","parts":[{"id":"unit","profiled_rigid_part":true,"source_sha256":"a".repeat(64),"placements_yup_m":[[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]],"qualities":{"reviewed":[primitive.clone(),mesh],"coarse":[]}}]});
        let mut manifest = manifest;
        manifest["parts"][0]["qualities"]["coarse"] = json!([primitive]);
        manifest["parts"][0]["qualities"]["coarse"][0]["allowed_purposes"] =
            json!(["projectile", "lidar"]);
        Self { root, manifest }
    }
    fn load(&self, requests: &[ProfileRequest]) -> Result<PreparedProfiles> {
        let bytes = serde_json::to_vec(&self.manifest)?;
        std::fs::write(self.root.join("collision-choices.json"), &bytes)?;
        load_part_profiles(
            &self.root,
            &format!("{:x}", Sha256::digest(bytes)),
            "unit",
            requests,
            0.4,
            0.45,
        )
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.root);
    }
}
fn requests() -> Vec<ProfileRequest> {
    [
        CollisionPurpose::RigidBody,
        CollisionPurpose::Projectile,
        CollisionPurpose::Lidar,
    ]
    .into_iter()
    .map(|purpose| ProfileRequest {
        purpose,
        quality: "reviewed".into(),
        representation: if purpose == CollisionPurpose::RigidBody {
            Representation::Mesh
        } else {
            Representation::NativePreferred
        },
        allow_experimental: purpose != CollisionPurpose::RigidBody,
    })
    .collect()
}
#[test]
fn profiles_keep_distinct_queries_and_immutable_cooked_bytes() {
    let f = Fixture::new();
    let prepared = f.load(&requests()).unwrap();
    assert_eq!(prepared.selected, ["mesh", "native", "native"]);
    std::fs::write(f.root.join("native.json"), b"mutated after loading").unwrap();
    std::fs::write(f.root.join("mesh.glb"), b"mutated after loading").unwrap();
    let mut world = rm_sim_physics::WorldPhysics::new(&[], 0.);
    world
        .install_body(
            rm_sim_physics::runtime::BodyKey(1),
            &prepared.model,
            rm_sim_physics::runtime::BodyState::at(Pose::default()),
        )
        .unwrap();
    let capture = world.static_geometry_snapshot(true);
    let rigid = capture
        .for_collision_purpose(CollisionPurpose::RigidBody)
        .cast_ray([0.; 3], [1., 0., 0.], 5.)
        .unwrap()
        .unwrap();
    let lidar = capture
        .for_collision_purpose(CollisionPurpose::Lidar)
        .cast_ray([0.; 3], [1., 0., 0.], 5.)
        .unwrap()
        .unwrap();
    assert!((rigid.distance_m - 2.).abs() < 1e-9);
    assert!((lidar.distance_m - 2.7).abs() < 1e-9);
}
#[test]
fn coarse_geometry_cannot_replace_rigid_contact_geometry() {
    let f = Fixture::new();
    let mut r = requests();
    r[0].quality = "coarse".into();
    r[0].allow_experimental = true;
    r[0].representation = Representation::NativePreferred;
    assert!(f.load(&r).err().unwrap().to_string().contains("purpose"));
}
#[test]
fn unsupported_native_capability_falls_back_without_reading_it() {
    let mut f = Fixture::new();
    f.manifest["parts"][0]["qualities"]["reviewed"][0]["requires"] =
        json!(["future-unavailable-shape"]);
    std::fs::write(f.root.join("native.json"), b"corrupt").unwrap();
    assert_eq!(
        f.load(&requests()).unwrap().selected,
        ["mesh", "mesh", "mesh"]
    );
}
#[test]
fn corrupt_supported_native_never_falls_back() {
    let f = Fixture::new();
    std::fs::write(f.root.join("native.json"), b"corrupt").unwrap();
    assert!(f.load(&requests()).is_err());
}
#[test]
fn unknown_quality_and_nonrigid_assembly_are_rejected() {
    let mut f = Fixture::new();
    let mut r = requests();
    r[1].quality = "missing".into();
    assert!(f.load(&r).is_err());
    f.manifest["parts"][0]["profiled_rigid_part"] = json!(false);
    assert!(f.load(&requests()).is_err());
}
