// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Explicit per-purpose choices for one rigid source part; no match defaults change.
use crate::{cad_assets::read_verified_bytes, collision_shapes::local_file};
use anyhow::{Context, Result, ensure};
use rm_sim_physics::{Pose, collision_profiles::CollisionPurpose, cooking::CookedRigidBody};
use serde::Deserialize;
use sim_engine::model::{
    BodyKind, ColliderDescription, CompoundShape, ConvexHull, RigidBodyDescription,
    ShapeDescription,
};
use std::{
    collections::{BTreeMap, BTreeSet},
    path::Path,
};

/// Representation preference at an explicitly selected geometry fidelity.
#[derive(Clone, Copy, Debug)]
pub enum Representation {
    /// Require the selected quality's portable triangle mesh.
    Mesh,
    /// Prefer a retained-mesh/native hybrid, with same-quality mesh fallback.
    Hybrid,
    /// Prefer a native compound, then hybrid, then same-quality mesh.
    NativePreferred,
}
/// Explicit request for one purpose. No unsupported fidelity is silently replaced.
#[derive(Clone, Debug)]
pub struct ProfileRequest {
    /// Rigid contacts, projectiles, or read-only LiDAR queries.
    pub purpose: CollisionPurpose,
    /// Exact exported fidelity key, for example `reviewed` or `projectile-balanced`.
    pub quality: String,
    /// Capability preference within that fidelity.
    pub representation: Representation,
    /// Admit candidates whose contact/trajectory behavior remains experimental.
    pub allow_experimental: bool,
}
/// Prepared fixed geometry for a single rigid source part, with three masked profiles.
/// Callers retain source motion/ownership; this does not flatten a complete field
/// into static scenery or install/replace a match package. Use explicit actor masks
/// from [`CollisionPurpose::actor_material`] in the receiving opt-in world.
#[derive(Clone)]
pub struct PreparedProfiles {
    /// Stable source definition identity.
    pub part_id: String,
    /// Selected representation IDs in request order.
    pub selected: Vec<String>,
    /// Cooked fixed model; all profiles share one caller-owned placement/pose.
    pub model: CookedRigidBody,
    /// Original rigid source occurrences. Convert Y-up to the caller's world once.
    pub placements_yup_m: Vec<[[f64; 4]; 4]>,
}
#[derive(Deserialize)]
struct Manifest {
    schema: String,
    world_frame: String,
    parts: Vec<Part>,
}
#[derive(Deserialize)]
struct Part {
    id: String,
    #[serde(default)]
    profiled_rigid_part: bool,
    source_sha256: String,
    placements_yup_m: Vec<[[f64; 4]; 4]>,
    qualities: BTreeMap<String, Vec<Variant>>,
}
#[derive(Deserialize)]
struct Variant {
    id: String,
    kind: String,
    priority: i64,
    requires: Vec<String>,
    approval: String,
    #[serde(default)]
    allowed_purposes: Vec<String>,
    files: Vec<FileHash>,
}
#[derive(Deserialize)]
struct FileHash {
    path: String,
    sha256: String,
}
#[derive(Deserialize)]
struct NativeRecipe {
    units: String,
    shape_axis: String,
    quaternion_order: String,
    coordinate_frame: String,
    source_sha256: String,
    #[serde(default)]
    allowed_purposes: Vec<String>,
    children: Vec<Child>,
}
#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Child {
    Cylinder {
        name: String,
        radius_m: f64,
        half_height_m: f64,
        translation_m: [f64; 3],
        rotation_wxyz: [f64; 4],
    },
    ConvexHull {
        name: String,
        vertices_m: Vec<[f64; 3]>,
    },
    Box {
        name: String,
        half_extents_m: [f64; 3],
        translation_m: [f64; 3],
        rotation_wxyz: [f64; 4],
    },
}
fn allowed(list: &[String], purpose: CollisionPurpose) -> bool {
    list.is_empty() || list.iter().any(|x| x == purpose.as_str())
}
fn shape(child: Child) -> Result<(String, Pose, ShapeDescription)> {
    Ok(match child {
        Child::Cylinder {
            name,
            radius_m,
            half_height_m,
            translation_m,
            rotation_wxyz,
        } => (
            name,
            Pose {
                translation_m,
                rotation_wxyz,
            },
            ShapeDescription::Cylinder {
                radius_m,
                half_height_m,
            },
        ),
        Child::Box {
            name,
            half_extents_m,
            translation_m,
            rotation_wxyz,
        } => (
            name,
            Pose {
                translation_m,
                rotation_wxyz,
            },
            ShapeDescription::Cuboid { half_extents_m },
        ),
        Child::ConvexHull { name, vertices_m } => (
            name,
            Pose::default(),
            ShapeDescription::ConvexHull(ConvexHull::new(vertices_m).map_err(anyhow::Error::msg)?),
        ),
    })
}
fn native(bytes: &[u8], source_hash: &str, purpose: CollisionPurpose) -> Result<ShapeDescription> {
    let r: NativeRecipe = serde_json::from_slice(bytes).context("native profile recipe")?;
    ensure!(
        r.units == "metres"
            && r.shape_axis == "+Y"
            && r.quaternion_order == "wxyz"
            && r.coordinate_frame == "same part-local frame as reviewed.glb",
        "unsupported native profile frame"
    );
    ensure!(
        r.source_sha256 == source_hash,
        "native profile source differs from selected part"
    );
    ensure!(
        allowed(&r.allowed_purposes, purpose),
        "native geometry prohibited for selected purpose"
    );
    let mut names = BTreeSet::new();
    let mut children = Vec::new();
    for c in r.children {
        let (name, pose, shape) = shape(c)?;
        ensure!(
            !name.is_empty() && names.insert(name),
            "duplicate or empty native child identity"
        );
        children.push((pose, shape));
    }
    Ok(ShapeDescription::Compound(
        CompoundShape::new(children).map_err(anyhow::Error::msg)?,
    ))
}
fn supports(v: &Variant, preference: Representation) -> bool {
    let kind = match preference {
        Representation::Mesh => v.kind == "triangle-mesh-v1",
        Representation::Hybrid => matches!(
            v.kind.as_str(),
            "triangle-mesh-v1" | "hybrid-convex-primitives-v1"
        ),
        Representation::NativePreferred => matches!(
            v.kind.as_str(),
            "triangle-mesh-v1" | "hybrid-convex-primitives-v1" | "convex-primitives-v1"
        ),
    };
    kind && v.requires.iter().all(|x| {
        matches!(
            x.as_str(),
            "triangle-mesh-v1"
                | "compound-convex-v1"
                | "convex-hull-v1"
                | "cylinder-y-v1"
                | "box-v1"
        )
    })
}
/// Verify the manifest snapshot and prepare exactly one choice for each purpose.
/// Selected files are verified and parsed from the same immutable bytes. Missing
/// capability may fall back within the requested quality; corrupt selected files,
/// unknown qualities, prohibited purposes and invalid shapes fail before mutation.
/// This API prepares one rigid source part only. Its adapter must preserve any
/// articulated owner boundaries instead of submitting a whole mixed assembly here.
pub fn load_part_profiles(
    root: &Path,
    manifest_sha256: &str,
    part_id: &str,
    requests: &[ProfileRequest],
    friction: f64,
    restitution: f64,
) -> Result<PreparedProfiles> {
    ensure!(
        requests.len() == 3,
        "require rigid-body, projectile and LiDAR requests"
    );
    let mut purposes = BTreeSet::new();
    for r in requests {
        ensure!(
            purposes.insert(r.purpose.as_str()),
            "duplicate collision purpose"
        );
    }
    let root = root.canonicalize()?;
    let bytes = read_verified_bytes(&root.join("collision-choices.json"), manifest_sha256)?;
    let manifest: Manifest = serde_json::from_slice(&bytes)?;
    ensure!(
        manifest.schema == "reviewed-field-collision-choices-v1"
            && manifest.world_frame
                == "right-handed glTF Y-up, metre; source CAD world rotated -90 degrees about X",
        "unsupported collision choices schema/frame"
    );
    let mut ids = BTreeSet::new();
    for p in &manifest.parts {
        ensure!(
            ids.insert(p.id.as_str()),
            "duplicate collision part identity"
        );
    }
    let part = manifest
        .parts
        .iter()
        .find(|p| p.id == part_id)
        .context("missing collision part")?;
    ensure!(
        part.profiled_rigid_part,
        "profile installation requires an explicitly rigid part; preserve articulated owner boundaries in its adapter"
    );
    ensure!(
        part.source_sha256.len() == 64 && part.source_sha256.bytes().all(|c| c.is_ascii_hexdigit()),
        "invalid part source checksum"
    );
    ensure!(
        !part.placements_yup_m.is_empty(),
        "missing collision part placement"
    );
    for matrix in &part.placements_yup_m {
        crate::terrain_collision::rigid(*matrix)?;
    }
    let mut colliders = Vec::new();
    let mut selected = Vec::new();
    for request in requests {
        let variants = part
            .qualities
            .get(&request.quality)
            .context("unknown quality; no implicit downgrade")?;
        let mut variant_ids = BTreeSet::new();
        for v in variants {
            ensure!(
                variant_ids.insert(v.id.as_str()),
                "duplicate collision variant identity"
            );
        }
        let chosen = variants
            .iter()
            .filter(|v| {
                supports(v, request.representation)
                    && allowed(&v.allowed_purposes, request.purpose)
                    && (v.approval == "reviewed"
                        || (v.approval == "experimental" && request.allow_experimental))
            })
            .max_by(|a, b| a.priority.cmp(&b.priority).then_with(|| b.id.cmp(&a.id)))
            .context("no supported/admitted representation for selected purpose and quality")?;
        let mut mesh_count = 0;
        let mut native_count = 0;
        for file in &chosen.files {
            let path = local_file(&root, &file.path)?;
            let payload = read_verified_bytes(&path, &file.sha256)?;
            let shape = if file.path.ends_with(".glb") {
                mesh_count += 1;
                ShapeDescription::TriangleMesh(
                    crate::collision_mesh::load_glb_bytes(&payload)?
                        .into_description()
                        .map_err(anyhow::Error::msg)?,
                )
            } else if file.path.ends_with(".json") {
                native_count += 1;
                native(&payload, &part.source_sha256, request.purpose)?
            } else {
                anyhow::bail!("unsupported collision profile payload")
            };
            colliders.push(
                ColliderDescription::new(
                    shape,
                    Pose::default(),
                    request.purpose.scenery_material(friction, restitution),
                )
                .map_err(anyhow::Error::msg)?,
            );
        }
        let expected = match chosen.kind.as_str() {
            "triangle-mesh-v1" => (1, 0),
            "convex-primitives-v1" => (0, 1),
            "hybrid-convex-primitives-v1" => (1, 1),
            _ => unreachable!(),
        };
        ensure!(
            (mesh_count, native_count) == expected,
            "collision profile payload set is incomplete or duplicated"
        );
        selected.push(chosen.id.clone());
    }
    let source =
        RigidBodyDescription::new(BodyKind::Fixed, colliders).map_err(anyhow::Error::msg)?;
    let model = CookedRigidBody::new(source).map_err(anyhow::Error::msg)?;
    Ok(PreparedProfiles {
        part_id: part.id.clone(),
        selected,
        model,
        placements_yup_m: part.placements_yup_m.clone(),
    })
}

#[cfg(test)]
#[path = "collision_profiles/tests.rs"]
mod tests;
