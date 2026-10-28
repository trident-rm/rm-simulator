// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Opt-in portable static terrain packages. This does not replace installed CAD.
use anyhow::{Context, Result, bail, ensure};
use glam::{DMat3, DMat4, DQuat, DVec3};
use rm_sim_physics::{
    Pose, WorldPhysics,
    cooking::CookedRigidBody,
    runtime::{ColliderBinding, PhysicalRuntime},
};
use serde::Deserialize;
#[cfg(test)]
use sha2::{Digest, Sha256};
use sim_engine::model::{
    BodyKind, ColliderDescription, ContactMaterial, HeightField, RigidBodyDescription,
    ShapeDescription,
};
use std::{collections::HashSet, path::Path};

/// Immutable verified terrain manifest selected by a field package.
#[derive(Clone, Debug, PartialEq)]
pub struct VerifiedTerrain {
    root: std::path::PathBuf,
    manifest: std::sync::Arc<[u8]>,
}
impl VerifiedTerrain {
    pub(crate) fn read(
        root: &Path,
        file: &str,
        sha256: &str,
        expected_source: &str,
    ) -> Result<Self> {
        let path = crate::collision_shapes::local_file(root, file)?;
        let manifest = crate::cad_assets::read_verified_bytes(&path, sha256)?;
        let parsed: Manifest = serde_json::from_slice(&manifest)?;
        ensure!(
            parsed.source_sha256 == expected_source,
            "terrain source differs from field"
        );
        Ok(Self {
            root: path
                .parent()
                .context("terrain manifest directory")?
                .to_path_buf(),
            manifest: manifest.into(),
        })
    }
    /// Prepare the explicitly selected balanced native terrain from the verified snapshot.
    /// Payload checksums are checked when cooking. No fallback changes quality.
    pub fn prepare(
        &self,
        world_from_yup: Pose,
        material: ContactMaterial,
    ) -> Result<TerrainPackage> {
        TerrainPackage::load_snapshot(
            &self.root,
            &self.manifest,
            Quality::Balanced,
            Representation::NativePreferred,
            world_from_yup,
            material,
        )
    }
}

/// An explicit fidelity choice; capability fallback never changes this tier.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quality {
    /// 64 wave segments per period.
    Precise,
    /// 32 wave segments per period.
    Balanced,
    /// 16 wave segments per period.
    Fast,
    /// User-approved eight wave segments per period.
    Coarse,
}
impl Quality {
    /// Manifest key.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Precise => "precise",
            Self::Balanced => "balanced",
            Self::Fast => "fast",
            Self::Coarse => "coarse",
        }
    }
}
/// Representation preference at the selected quality.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Representation {
    /// Always use the complete same-quality mesh.
    Mesh,
    /// Prefer native grids; unsupported variants fall back to a mesh.
    NativePreferred,
}
#[derive(Deserialize)]
struct Manifest {
    schema: String,
    source_sha256: String,
    world_frame: String,
    parts: Vec<Part>,
}
#[derive(Deserialize)]
struct Part {
    id: String,
    motion: String,
    placements_yup_m: Vec<[[f64; 4]; 4]>,
    qualities: std::collections::HashMap<String, Vec<Variant>>,
}
#[derive(Deserialize)]
struct Variant {
    id: String,
    priority: i64,
    kind: String,
    requires: Vec<String>,
    files: Vec<FileHash>,
}
#[derive(Deserialize)]
struct FileHash {
    path: String,
    sha256: String,
}
#[derive(Deserialize)]
struct GridFile {
    schema: String,
    component_id: String,
    source_mesh_remainder_required: bool,
    heightfield: Grid,
}
#[derive(Deserialize)]
struct Grid {
    schema: String,
    units: String,
    local_up_axis: String,
    sample_order: String,
    heights_m: Vec<Vec<f64>>,
    size_x_m: f64,
    size_z_m: f64,
    pose_yup_m: [[f64; 4]; 4],
    cell_diagonal: String,
    finite_footprint: bool,
}

/// Cooked immutable fixed work part; placements are ordinary FLU poses.
#[derive(Clone)]
pub struct TerrainPart {
    /// Stable source work-part identity.
    pub id: String,
    /// Chosen manifest representation identifier.
    pub representation_id: String,
    /// Shared collider resources, cooked exactly once per definition.
    pub model: CookedRigidBody,
    /// Original occurrence placements converted at the explicit package boundary.
    pub placements: Vec<Pose>,
}
/// Complete verified/cooked package, independent of match or rendering state.
#[derive(Clone)]
pub struct TerrainPackage {
    /// Original CAD source checksum, retained as provenance.
    pub source_sha256: String,
    /// Static terrain definitions and their placements.
    pub parts: Vec<TerrainPart>,
    /// Number of native grids in definitions, before occurrence multiplication.
    pub native_heightfields: usize,
}
impl TerrainPackage {
    /// Verify selected file hashes and coordinate contracts, then cook all parts.
    /// `world_from_yup` is an ordinary rigid transform from exported Y-up metres
    /// into the caller's FLU world. Materials and filters are explicit caller policy.
    /// Corrupt selected native payloads fail; only unsupported kinds fall back.
    pub fn load(
        root: &Path,
        quality: Quality,
        representation: Representation,
        world_from_yup: Pose,
        material: ContactMaterial,
    ) -> Result<Self> {
        let root = crate::package_io::canonicalize(root)?;
        let bytes = crate::package_io::read(root.join("manifest.json"))?;
        Self::load_snapshot(
            &root,
            &bytes,
            quality,
            representation,
            world_from_yup,
            material,
        )
    }
    fn load_snapshot(
        root: &Path,
        bytes: &[u8],
        quality: Quality,
        representation: Representation,
        world_from_yup: Pose,
        material: ContactMaterial,
    ) -> Result<Self> {
        let root = crate::package_io::canonicalize(root)?;
        let manifest: Manifest = serde_json::from_slice(bytes)?;
        ensure!(
            manifest.schema == "terrain-collision-candidates-v1",
            "unsupported terrain schema"
        );
        ensure!(
            manifest.source_sha256.len() == 64
                && manifest
                    .source_sha256
                    .bytes()
                    .all(|b| b.is_ascii_hexdigit()),
            "invalid source checksum"
        );
        ensure!(
            manifest.world_frame
                == "right-handed glTF Y-up, metre; source CAD world rotated -90 degrees about X",
            "unknown terrain coordinate frame"
        );
        let root_pose = pose_matrix(world_from_yup)?;
        let mut ids = HashSet::new();
        let mut parts = Vec::new();
        let mut native_heightfields = 0;
        for part in manifest.parts {
            ensure!(
                !part.id.is_empty() && ids.insert(part.id.clone()) && part.motion == "static",
                "duplicate or nonstatic terrain work part"
            );
            ensure!(
                !part.placements_yup_m.is_empty(),
                "terrain work part has no placements"
            );
            let choices = part
                .qualities
                .get(quality.as_str())
                .context("missing selected terrain quality")?;
            let supported = |v: &&Variant| {
                let allowed = match v.kind.as_str() {
                    "triangle-mesh-v1" => true,
                    "heightfield-grid-v1" | "hybrid-heightfield-v1" => {
                        representation == Representation::NativePreferred
                    }
                    _ => false,
                };
                allowed
                    && v.requires.iter().all(|c| {
                        c == "triangle-mesh-v1"
                            || (c == "heightfield-grid-v1"
                                && representation == Representation::NativePreferred)
                    })
            };
            let chosen = choices
                .iter()
                .filter(supported)
                .max_by(|a, b| a.priority.cmp(&b.priority).then_with(|| b.id.cmp(&a.id)))
                .context("no supported same-quality representation")?;
            let paths = chosen
                .files
                .iter()
                .map(|f| verify_file(&root, f))
                .collect::<Result<Vec<_>>>()?;
            let mut colliders = Vec::new();
            match chosen.kind.as_str() {
                "triangle-mesh-v1" => {
                    ensure!(
                        paths.len() == 1,
                        "mesh representation requires exactly one file"
                    );
                    colliders.push(mesh_collider(&paths[0], material)?);
                }
                "heightfield-grid-v1" | "hybrid-heightfield-v1" => {
                    let hybrid = chosen.kind == "hybrid-heightfield-v1";
                    ensure!(
                        paths.len() == if hybrid { 2 } else { 1 },
                        "native representation is incomplete"
                    );
                    let file: GridFile = serde_json::from_slice(&paths[0])?;
                    ensure!(
                        file.schema == "collision-heightfield-component-v1"
                            && file.component_id == part.id
                            && file.source_mesh_remainder_required == hybrid,
                        "heightfield ownership/remainder mismatch"
                    );
                    let g = file.heightfield;
                    ensure!(
                        g.schema == "heightfield-grid-v1"
                            && g.units == "metre"
                            && g.local_up_axis == "+Y"
                            && g.finite_footprint,
                        "unsupported heightfield contract"
                    );
                    ensure!(
                        g.sample_order
                            == "heights[row_z][column_x]; rows run from -Z to +Z; columns -X to +X"
                            && g.cell_diagonal == "(row,col) to (row+1,col+1)",
                        "unsupported grid ordering/diagonal"
                    );
                    let grid = HeightField::new(g.heights_m, g.size_x_m, g.size_z_m)
                        .map_err(anyhow::Error::msg)?;
                    colliders.push(
                        ColliderDescription::new(
                            ShapeDescription::HeightField(grid),
                            matrix_pose(rigid(g.pose_yup_m)?),
                            material,
                        )
                        .map_err(anyhow::Error::msg)?,
                    );
                    if hybrid {
                        colliders.push(mesh_collider(&paths[1], material)?);
                    }
                    native_heightfields += 1;
                }
                _ => bail!("unsupported terrain representation"),
            }
            let model = CookedRigidBody::new(
                RigidBodyDescription::new(BodyKind::Fixed, colliders)
                    .map_err(anyhow::Error::msg)?,
            )
            .map_err(anyhow::Error::msg)?;
            let placements = part
                .placements_yup_m
                .into_iter()
                .map(|p| Ok(matrix_pose(root_pose * rigid(p)?)))
                .collect::<Result<Vec<_>>>()?;
            parts.push(TerrainPart {
                id: part.id,
                representation_id: chosen.id.clone(),
                model,
                placements,
            });
        }
        ensure!(!parts.is_empty(), "empty terrain package");
        Ok(Self {
            source_sha256: manifest.source_sha256,
            parts,
            native_heightfields,
        })
    }
    /// Add the verified fixed colliders to a caller-owned bare runtime.
    /// Do not also install the complete mesh fallbacks or legacy terrain there.
    pub fn install(&self, runtime: &mut PhysicalRuntime) -> Result<Vec<ColliderBinding>> {
        let mut bindings = Vec::new();
        for p in &self.parts {
            for pose in &p.placements {
                bindings.extend(
                    runtime
                        .install_fixed_model(&p.model, *pose)
                        .map_err(anyhow::Error::msg)?,
                );
            }
        }
        Ok(bindings)
    }
    /// Add native scenery through the existing chassis/projectile facade.
    /// The caller retains responsibility for removing its catch plane/old terrain.
    pub fn install_in_world(&self, world: &mut WorldPhysics) -> Result<Vec<ColliderBinding>> {
        let mut bindings = Vec::new();
        for p in &self.parts {
            for pose in &p.placements {
                bindings.extend(
                    world
                        .add_fixed_model(&p.model, *pose)
                        .map_err(anyhow::Error::msg)?,
                );
            }
        }
        Ok(bindings)
    }
}
fn verify_file(root: &Path, file: &FileHash) -> Result<Vec<u8>> {
    ensure!(
        !Path::new(&file.path).is_absolute(),
        "absolute terrain payload path"
    );
    let path = crate::verified::confined_path(root, Path::new(&file.path))
        .context("missing or outside-root terrain payload")?;
    let snapshot = crate::verified::Snapshot::read(&path)?;
    ensure!(
        snapshot.sha256 == file.sha256,
        "terrain payload checksum mismatch: {}",
        file.path
    );
    Ok(snapshot.bytes)
}
fn mesh_collider(bytes: &[u8], material: ContactMaterial) -> Result<ColliderDescription> {
    let mesh = crate::collision_mesh::load_glb_bytes(bytes)?
        .into_description()
        .map_err(anyhow::Error::msg)?;
    ColliderDescription::new(
        ShapeDescription::TriangleMesh(mesh),
        Pose::default(),
        material,
    )
    .map_err(anyhow::Error::msg)
}
pub(crate) fn rigid(matrix: [[f64; 4]; 4]) -> Result<DMat4> {
    ensure!(
        matrix.iter().flatten().all(|v| v.is_finite()) && matrix[3] == [0., 0., 0., 1.],
        "nonfinite/nonaffine terrain pose"
    );
    let m = DMat4::from_cols_array(&std::array::from_fn::<_, 16, _>(|i| matrix[i % 4][i / 4]));
    let r = DMat3::from_mat4(m);
    let gram = r.transpose() * r;
    ensure!(
        (r.determinant() - 1.).abs() < 1e-7
            && gram
                .to_cols_array()
                .iter()
                .zip(DMat3::IDENTITY.to_cols_array())
                .all(|(a, b)| (a - b).abs() < 1e-7),
        "terrain pose must be a proper rigid transform"
    );
    Ok(m)
}
fn pose_matrix(p: Pose) -> Result<DMat4> {
    ensure!(
        p.translation_m
            .iter()
            .chain(p.rotation_wxyz.iter())
            .all(|x| x.is_finite()),
        "invalid world pose"
    );
    let [w, x, y, z] = p.rotation_wxyz;
    let q = DQuat::from_xyzw(x, y, z, w);
    ensure!((q.length_squared() - 1.).abs() < 1e-7, "nonunit world pose");
    Ok(DMat4::from_rotation_translation(
        q,
        DVec3::from_array(p.translation_m),
    ))
}
fn matrix_pose(m: DMat4) -> Pose {
    let q = DQuat::from_mat3(&DMat3::from_mat4(m)).normalize();
    let [x, y, z, w] = q.to_array();
    Pose {
        translation_m: m.w_axis.truncate().to_array(),
        rotation_wxyz: [w, x, y, z],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn field_manifest_snapshot_and_source_pin_survive_deferred_cooking() {
        let p = fixture();
        let mut manifest: serde_json::Value =
            serde_json::from_slice(&std::fs::read(p.join("manifest.json")).unwrap()).unwrap();
        let variants = manifest["parts"][0]["qualities"]["coarse"].clone();
        manifest["parts"][0]["qualities"]["balanced"] = variants;
        let bytes = serde_json::to_vec(&manifest).unwrap();
        std::fs::write(p.join("manifest.json"), &bytes).unwrap();
        let hash = format!("{:x}", Sha256::digest(&bytes));
        assert!(VerifiedTerrain::read(&p, "manifest.json", &hash, &"b".repeat(64)).is_err());
        let verified = VerifiedTerrain::read(&p, "manifest.json", &hash, &"a".repeat(64)).unwrap();
        std::fs::write(p.join("manifest.json"), b"changed after verification").unwrap();
        assert_eq!(
            verified
                .prepare(Pose::default(), material())
                .unwrap()
                .native_heightfields,
            1
        );
        std::fs::write(p.join("shape.json"), b"tampered payload").unwrap();
        assert!(verified.prepare(Pose::default(), material()).is_err());
        std::fs::remove_dir_all(p).unwrap();
    }
    pub(super) fn fixture() -> std::path::PathBuf {
        static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let p = std::env::temp_dir().join(format!(
            "rm-terrain-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
        ));
        std::fs::create_dir_all(&p).unwrap();
        let shape = serde_json::json!({"schema":"collision-heightfield-component-v1","component_id":"floor","source_mesh_remainder_required":false,"heightfield":{
            "schema":"heightfield-grid-v1","units":"metre","local_up_axis":"+Y","finite_footprint":true,
            "sample_order":"heights[row_z][column_x]; rows run from -Z to +Z; columns -X to +X",
            "cell_diagonal":"(row,col) to (row+1,col+1)","heights_m":[[0.,0.],[0.2,0.2]],"size_x_m":2.,"size_z_m":2.,
            "pose_yup_m":[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]}});
        let bytes = serde_json::to_vec(&shape).unwrap();
        std::fs::write(p.join("shape.json"), &bytes).unwrap();
        let variant = serde_json::json!({"id":"native","priority":100,"kind":"heightfield-grid-v1","requires":["heightfield-grid-v1"],"files":[{"path":"shape.json","sha256":format!("{:x}",Sha256::digest(&bytes))}]});
        let manifest = serde_json::json!({"schema":"terrain-collision-candidates-v1","source_sha256":"a".repeat(64),"world_frame":"right-handed glTF Y-up, metre; source CAD world rotated -90 degrees about X","parts":[{"id":"floor","motion":"static","placements_yup_m":[[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]],"qualities":{"coarse":[variant]}}]});
        std::fs::write(
            p.join("manifest.json"),
            serde_json::to_vec(&manifest).unwrap(),
        )
        .unwrap();
        p
    }
    fn material() -> ContactMaterial {
        ContactMaterial {
            friction: 0.5,
            restitution: 0.,
            memberships: u32::MAX,
            filter: u32::MAX,
        }
    }
    #[test]
    fn verifies_native_contract_and_explicit_quality_before_install() {
        let p = fixture();
        let package = TerrainPackage::load(
            &p,
            Quality::Coarse,
            Representation::NativePreferred,
            Pose::default(),
            material(),
        )
        .unwrap();
        assert_eq!(package.native_heightfields, 1);
        let mut runtime = PhysicalRuntime::new(rm_sim_physics::StepDuration::HZ_1000);
        assert_eq!(package.install(&mut runtime).unwrap().len(), 1);
        assert!(
            TerrainPackage::load(
                &p,
                Quality::Fast,
                Representation::NativePreferred,
                Pose::default(),
                material()
            )
            .is_err()
        );
        assert!(
            TerrainPackage::load(
                &p,
                Quality::Coarse,
                Representation::Mesh,
                Pose::default(),
                material()
            )
            .is_err()
        );
        std::fs::write(p.join("shape.json"), b"tampered").unwrap();
        assert!(
            TerrainPackage::load(
                &p,
                Quality::Coarse,
                Representation::NativePreferred,
                Pose::default(),
                material()
            )
            .is_err()
        );
        std::fs::remove_dir_all(p).unwrap();
    }
    #[test]
    fn rejects_nonrigid_and_reflected_frames() {
        assert!(
            rigid([
                [1., 0., 0., 0.],
                [0., 1., 0., 0.],
                [0., 0., 1., 0.],
                [0., 0., 0., 1.]
            ])
            .is_ok()
        );
        for scale in [-1., 2., f64::NAN] {
            assert!(
                rigid([
                    [scale, 0., 0., 0.],
                    [0., 1., 0., 0.],
                    [0., 0., 1., 0.],
                    [0., 0., 0., 1.]
                ])
                .is_err()
            );
        }
    }
}

#[cfg(test)]
mod hostile_payload_tests {
    use super::*;
    fn material() -> ContactMaterial {
        ContactMaterial {
            friction: 0.5,
            restitution: 0.,
            memberships: u32::MAX,
            filter: u32::MAX,
        }
    }
    fn update_shape(root: &Path, change: impl FnOnce(&mut serde_json::Value)) {
        let path = root.join("shape.json");
        let mut data: serde_json::Value =
            serde_json::from_slice(&std::fs::read(&path).unwrap()).unwrap();
        change(&mut data);
        let bytes = serde_json::to_vec(&data).unwrap();
        std::fs::write(path, &bytes).unwrap();
        let path = root.join("manifest.json");
        let mut manifest: serde_json::Value =
            serde_json::from_slice(&std::fs::read(&path).unwrap()).unwrap();
        manifest["parts"][0]["qualities"]["coarse"][0]["files"][0]["sha256"] =
            format!("{:x}", Sha256::digest(bytes)).into();
        std::fs::write(path, serde_json::to_vec(&manifest).unwrap()).unwrap();
    }
    #[test]
    fn matching_hash_does_not_admit_wrong_ownership_or_diagonal() {
        for field in ["owner", "diagonal"] {
            let root = super::tests::fixture();
            update_shape(&root, |data| {
                if field == "owner" {
                    data["component_id"] = "other".into();
                } else {
                    data["heightfield"]["cell_diagonal"] = "other".into();
                }
            });
            assert!(
                TerrainPackage::load(
                    &root,
                    Quality::Coarse,
                    Representation::NativePreferred,
                    Pose::default(),
                    material()
                )
                .is_err()
            );
            std::fs::remove_dir_all(root).unwrap();
        }
    }
    #[test]
    fn prepared_native_geometry_survives_removal_of_source_payloads() {
        let root = super::tests::fixture();
        let model = TerrainPackage::load(
            &root,
            Quality::Coarse,
            Representation::NativePreferred,
            Pose::default(),
            material(),
        )
        .unwrap();
        std::fs::remove_dir_all(root).unwrap();
        let mut world = WorldPhysics::new(&[], -10.);
        world.remove_catch_floor();
        assert_eq!(model.install_in_world(&mut world).unwrap().len(), 1);
        assert_eq!(
            world
                .static_geometry_snapshot(true)
                .into_triangles()
                .1
                .len(),
            2
        );
    }
    #[test]
    fn checksummed_payload_cannot_escape_the_package() {
        let root = super::tests::fixture();
        let external = root.with_extension("external");
        std::fs::write(&external, b"anything").unwrap();
        let f = FileHash {
            path: external.to_string_lossy().into_owned(),
            sha256: format!("{:x}", Sha256::digest(b"anything")),
        };
        assert!(verify_file(&root, &f).is_err());
        let f = FileHash {
            path: format!("../{}", external.file_name().unwrap().to_string_lossy()),
            ..f
        };
        assert!(verify_file(&root, &f).is_err());
        std::fs::remove_dir_all(root).unwrap();
        std::fs::remove_file(external).unwrap();
    }
}
