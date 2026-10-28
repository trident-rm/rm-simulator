// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Checked robot packages without an app, match, renderer or dynamics backend.
//!
//! [`RobotDescriptor`] snapshots the bytes it consumes and validates the supported
//! source profile, checksums, physical configuration and armor mounts together.
//! The free functions retain the older, narrower app-helper contracts; they do
//! not certify an entire package. Native CAD property caches are not consumed or
//! validated here. These fitted models do not establish source link inertias or
//! a torque-driven articulation plant.
use rm_sim_physics::{Pose, chassis::ChassisConfig};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{io, path::Path, sync::Arc};

/// Supported source-bound robot identities; not competition admission or roles.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RobotProfile {
    /// Paired team chassis/gimbal STEP sources with the existing fitted model.
    TeamCad,
    /// Artisans folding source with the existing fitted Hero model.
    ArtisansFoldingHero,
    /// Dreamchaser stair source with the existing fitted climbing model.
    DreamchaserStairHeavy,
    /// PIE tunnel source and checksummed physical/articulation configuration.
    PieTunnelHeavy,
    /// Future tunnel source and checksummed physical/articulation configuration.
    FutureTunnelInfantry,
    /// EGA tunnel source and checksummed physical/articulation configuration.
    EgaTunnelSentry,
}

/// Origin of the physical configuration, independent of CAD provenance.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigurationOrigin {
    /// Schema 1–3 uses the simulator's existing fitted preset, not source inertias.
    SimulatorFittedPreset,
    /// Schema 4 supplies validated, checksummed `chassis.json`, without tuning.
    SourceFittedConfiguration,
}

/// Replacement module identity in the existing armor package contract.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ArmorKind {
    /// Small AM02 module.
    Am02,
    /// Large AM12 module.
    Am12,
}

/// How source surfaces relate to the fitted collision armor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ArmorIdentity {
    /// Legacy source surfaces are retained with artwork/light overlays.
    Overlays,
    /// Four validated `robot-armor-v1` mounts replace source armor surfaces.
    Replacement {
        /// Module kind shared by all four addressed mounts.
        kind: ArmorKind,
    },
}

/// Source CAD identity checked against the supported profile allowlist.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SourceProvenance {
    /// Manifest source role (`chassis`, `gimbal` or `source`).
    pub role: String,
    /// SHA-256 of the original CAD, as asserted by the supported package profile.
    /// The original STEP is not required or re-read by this loader.
    pub cad_sha256: String,
}

/// Immutable bytes and checksum of one consumed package file.
#[derive(Clone)]
pub struct VerifiedRobotFile {
    name: String,
    sha256: String,
    bytes: Arc<[u8]>,
}
impl std::fmt::Debug for VerifiedRobotFile {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("VerifiedRobotFile")
            .field("name", &self.name)
            .field("sha256", &self.sha256)
            .field("byte_len", &self.bytes.len())
            .finish()
    }
}
impl VerifiedRobotFile {
    /// Package-local filename, never an installation path.
    pub fn name(&self) -> &str {
        &self.name
    }
    /// SHA-256 computed from the snapshotted bytes.
    pub fn sha256(&self) -> &str {
        &self.sha256
    }
    /// Verified bytes, shared on descriptor clone; no later filesystem read.
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }
}

/// Immutable catalogue version 1 of the existing supported robot packages.
///
/// This version describes the Rust catalogue contract, not a new manifest schema.
/// Physical data contains no backend handles, HP, team or admission authority.
/// Clone once at construction, or borrow/share the descriptor during stepping;
/// loading does not cook shapes or copy geometry into per-tick state.
#[derive(Clone, Debug)]
pub struct RobotDescriptor {
    profile: RobotProfile,
    manifest_schema: u64,
    origin: ConfigurationOrigin,
    sources: Vec<SourceProvenance>,
    chassis: ChassisConfig,
    armor: ArmorIdentity,
    manifest: VerifiedRobotFile,
    visual: VerifiedRobotFile,
    source_configuration: Option<VerifiedRobotFile>,
    identity_sha256: String,
}
impl RobotDescriptor {
    /// Catalogue contract version; independent of the external manifest schema.
    pub const VERSION: u32 = 1;

    /// Load one checked package, snapshotting manifest, neutral GLB and optional
    /// chassis JSON before parsing. Every declared file reference must be a plain
    /// local filename with a SHA-256; only consumed files are read and verified.
    /// Profile trust checks known original CAD digests, not CAD geometry/inertias.
    /// Unconsumed preview files and native property sidecars are not validated.
    pub fn load(root: &Path) -> io::Result<Self> {
        let manifest_bytes = read_local(root, "manifest.json")?;
        let m: Value = serde_json::from_slice(&manifest_bytes)?;
        let (profile, sources) = profile(&m)?;
        validate_references(&m)?;
        let visual = checked_file(root, &m, "neutral", "neutral.glb")?;
        let (source, source_configuration) = read_source_configuration(root, &m)?;
        let origin = if source.is_some() {
            ConfigurationOrigin::SourceFittedConfiguration
        } else {
            ConfigurationOrigin::SimulatorFittedPreset
        };
        let chassis = source.unwrap_or_else(|| fitted(profile));
        chassis.validate().map_err(io::Error::other)?;
        let armor = validate_armor(&m, &chassis)?;
        let manifest = snapshot("manifest.json", manifest_bytes);
        let mut identity = Sha256::new();
        identity.update(b"rm-sim-robot-descriptor-v1\0");
        for file in std::iter::once(&manifest)
            .chain(std::iter::once(&visual))
            .chain(source_configuration.iter())
        {
            identity.update((file.name.len() as u64).to_le_bytes());
            identity.update(file.name.as_bytes());
            identity.update(file.sha256.as_bytes());
        }
        // Include the resolved raw model as well: legacy presets live in code,
        // and may change without modifying any external package bytes.
        let raw_model = serde_json::to_vec(&chassis)?;
        identity.update((raw_model.len() as u64).to_le_bytes());
        identity.update(raw_model);
        Ok(Self {
            profile,
            manifest_schema: m["schema"].as_u64().expect("checked profile schema"),
            origin,
            sources,
            chassis,
            armor,
            manifest,
            visual,
            source_configuration,
            identity_sha256: format!("{:x}", identity.finalize()),
        })
    }
    /// Supported source/profile identity.
    pub fn profile(&self) -> RobotProfile {
        self.profile
    }
    /// Original package schema (1–4), not the catalogue contract version.
    pub fn manifest_schema(&self) -> u64 {
        self.manifest_schema
    }
    /// Distinguish source-fitted JSON from simulator-fitted fallback parameters.
    pub fn configuration_origin(&self) -> ConfigurationOrigin {
        self.origin
    }
    /// Verified raw physical configuration, with original actuator rates intact.
    /// Caller tuning can be applied explicitly to a clone with
    /// [`ChassisConfig::set_transform_duration_s`].
    pub fn chassis(&self) -> &ChassisConfig {
        &self.chassis
    }
    /// Original CAD digest assertions checked against the profile allowlist.
    pub fn sources(&self) -> &[SourceProvenance] {
        &self.sources
    }
    /// Validated armor replacement/overlay identity; physical mount poses remain
    /// available from [`Self::chassis`].
    pub fn armor(&self) -> ArmorIdentity {
        self.armor
    }
    /// Complete original manifest bytes, preserving source notices and visual
    /// link metadata without claiming validation of arbitrary metadata fields.
    pub fn manifest(&self) -> &VerifiedRobotFile {
        &self.manifest
    }
    /// Verified neutral GLB bytes; visual hierarchy is not rewritten or cooked.
    pub fn visual(&self) -> &VerifiedRobotFile {
        &self.visual
    }
    /// Original checksummed schema-4 configuration bytes, before caller tuning.
    pub fn source_configuration(&self) -> Option<&VerifiedRobotFile> {
        self.source_configuration.as_ref()
    }
    /// Descriptor identity over catalogue version, manifest, consumed file hashes
    /// and serialized raw physical configuration (including fitted presets).
    /// Installation paths never participate. Manifest byte changes intentionally
    /// change identity. Caller tuning is not included; record the effective tuned
    /// config separately. This is not solver/backend compatibility certification.
    pub fn identity_sha256(&self) -> &str {
        &self.identity_sha256
    }
}

fn fitted(profile: RobotProfile) -> ChassisConfig {
    match profile {
        RobotProfile::ArtisansFoldingHero => ChassisConfig::folding_hero(),
        RobotProfile::DreamchaserStairHeavy => ChassisConfig::stair_heavy(),
        _ => ChassisConfig::team_cad(),
    }
}

fn profile(m: &Value) -> io::Result<(RobotProfile, Vec<SourceProvenance>)> {
    let invalid = || io::Error::other("robot package does not match the fitted team CAD profile");
    let (profile, expected) = match (m["schema"].as_u64(), m["profile"].as_str()) {
        (Some(1), _) => {
            let expected = [
                (
                    "chassis",
                    "b5a603acd8222295096184b68824eacf7302c7955e67943568dbc702a0611357",
                ),
                (
                    "gimbal",
                    "47e6aab908c3fbb443b9f3d950e77144ea2bc7baf6e6aa9d8a754174e822ed7f",
                ),
            ];
            let mut sources = Vec::new();
            for (role, hash) in expected {
                if m["sources"][role]["cad_sha256"] != hash {
                    return Err(invalid());
                }
                sources.push(SourceProvenance {
                    role: role.into(),
                    cad_sha256: hash.into(),
                });
            }
            return Ok((RobotProfile::TeamCad, sources));
        }
        (Some(2), Some("artisans-folding-hero-v1")) => (
            RobotProfile::ArtisansFoldingHero,
            "71d6ddb07672028613c03ab2abca508cd616f93a3e2b5481b6696a063b86dd6e",
        ),
        (Some(3), Some("dreamchaser-stair-heavy-v1")) => (
            RobotProfile::DreamchaserStairHeavy,
            "dd98239dce94e35cb2ccc43171f4edc6e35c77f3f0659f91890750b98bad0e01",
        ),
        (Some(4), Some("pie-tunnel-heavy-v1")) => (
            RobotProfile::PieTunnelHeavy,
            "bfb91b499f17488ed1deb523568869f2fcb29436a53cbc39ec5e64ac4a822c47",
        ),
        (Some(4), Some("future-tunnel-infantry-v1")) => (
            RobotProfile::FutureTunnelInfantry,
            "d82b2c6950ec29d1d9609e1b5973d7981500be95bfb5f22b41b5eb68ced4bbbd",
        ),
        (Some(4), Some("ega-tunnel-sentry-v1")) => (
            RobotProfile::EgaTunnelSentry,
            "c3aafbe2b06bd678a66cb769f2fcc825a851c56b4644d6b1d84a6389ad73b65b",
        ),
        _ => return Err(invalid()),
    };
    if m["source"]["cad_sha256"] != expected {
        return Err(invalid());
    }
    Ok((
        profile,
        vec![SourceProvenance {
            role: "source".into(),
            cad_sha256: expected.into(),
        }],
    ))
}

fn local_name(name: &str) -> bool {
    !name.is_empty() && name != "." && name != ".." && !name.contains(['/', '\\', ':', '\0'])
}
fn validate_references(m: &Value) -> io::Result<()> {
    let files = m["files"]
        .as_object()
        .ok_or_else(|| io::Error::other("missing robot file references"))?;
    for file in files.values() {
        let name = file["file"]
            .as_str()
            .ok_or_else(|| io::Error::other("missing robot file reference"))?;
        let hash = file["sha256"].as_str().unwrap_or("");
        if !local_name(name)
            || hash.len() != 64
            || !hash
                .bytes()
                .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
        {
            return Err(io::Error::other(
                "malformed robot file reference or checksum",
            ));
        }
    }
    Ok(())
}
fn read_local(root: &Path, name: &str) -> io::Result<Vec<u8>> {
    if !local_name(name) || !std::fs::symlink_metadata(root.join(name))?.is_file() {
        return Err(io::Error::other(
            "robot reference must be a local regular file",
        ));
    }
    crate::package_io::read(root.join(name))
}
fn snapshot(name: &str, bytes: Vec<u8>) -> VerifiedRobotFile {
    let snapshot = crate::verified::Snapshot::new(bytes);
    VerifiedRobotFile {
        name: name.into(),
        sha256: snapshot.sha256,
        bytes: snapshot.bytes.into(),
    }
}
fn checked_file(root: &Path, m: &Value, key: &str, name: &str) -> io::Result<VerifiedRobotFile> {
    if m["files"][key]["file"] != name {
        return Err(io::Error::other("robot file reference mismatch"));
    }
    let file = snapshot(name, read_local(root, name)?);
    if m["files"][key]["sha256"] != file.sha256 {
        return Err(io::Error::other(format!("robot {name} checksum mismatch")));
    }
    Ok(file)
}
fn read_manifest(root: &Path) -> io::Result<Value> {
    Ok(serde_json::from_slice(&read_local(root, "manifest.json")?)?)
}
fn read_source_configuration(
    root: &Path,
    m: &Value,
) -> io::Result<(Option<ChassisConfig>, Option<VerifiedRobotFile>)> {
    if m["schema"] != 4 {
        return Ok((None, None));
    }
    let file = checked_file(root, m, "chassis", "chassis.json")?;
    let c: ChassisConfig = serde_json::from_slice(file.bytes())?;
    c.validate().map_err(io::Error::other)?;
    if c.articulation
        .as_ref()
        .is_none_or(|a| Some(a.profile.as_str()) != m["profile"].as_str())
    {
        return Err(io::Error::other("source articulation profile mismatch"));
    }
    Ok((Some(c), Some(file)))
}

/// Fully verify a supported package; return whether it selects the folding Hero.
/// Prefer retaining [`RobotDescriptor`] when more package data will be needed.
pub fn verify(root: &Path) -> io::Result<bool> {
    Ok(RobotDescriptor::load(root)?.profile() == RobotProfile::ArtisansFoldingHero)
}

/// Read only the profile label, matching the legacy app helper. This does not
/// validate CAD provenance, geometry or checksums; use [`RobotDescriptor::load`].
pub fn is_stair_heavy(root: &Path) -> io::Result<bool> {
    Ok(read_manifest(root)?["profile"] == "dreamchaser-stair-heavy-v1")
}

/// Validate only optional armor mounts and schema-4 physical configuration.
/// Legacy fixtures need no neutral GLB or trusted source digest for this helper.
/// It does not certify source provenance; use [`RobotDescriptor::load`] for that.
pub fn replacement_armor(root: &Path) -> io::Result<bool> {
    let m = read_manifest(root)?;
    // Preserve legacy behavior: absent armor requires no physical configuration.
    if m.get("armor").is_none() {
        return Ok(false);
    }
    let source = read_source_configuration(root, &m)?.0;
    let fallback = if m["profile"] == "artisans-folding-hero-v1" {
        RobotProfile::ArtisansFoldingHero
    } else if m["profile"] == "dreamchaser-stair-heavy-v1" {
        RobotProfile::DreamchaserStairHeavy
    } else {
        RobotProfile::TeamCad
    };
    Ok(matches!(
        validate_armor(&m, &source.unwrap_or_else(|| fitted(fallback)))?,
        ArmorIdentity::Replacement { .. }
    ))
}

/// Read only the checksummed schema-4 configuration and articulation profile.
/// Returns unmodified actuator rates; no simulator tuning or CAD provenance
/// certification is implied. Schemas 1–3 return `None` for compatibility.
pub fn source_chassis(root: &Path) -> io::Result<Option<ChassisConfig>> {
    Ok(read_source_configuration(root, &read_manifest(root)?)?.0)
}

fn validate_armor(manifest: &Value, config: &ChassisConfig) -> io::Result<ArmorIdentity> {
    let Some(armor) = manifest.get("armor") else {
        return Ok(ArmorIdentity::Overlays);
    };
    let invalid =
        || std::io::Error::other("robot armor bindings do not match the fitted collision profile");
    if armor["schema"] != "robot-armor-v1" || armor["runtime_replacement"] != true {
        return Err(invalid());
    }
    let hero = manifest["profile"] == "artisans-folding-hero-v1";
    let modules = armor["modules"].as_array().ok_or_else(invalid)?;
    if modules.len() != 4 {
        return Err(invalid());
    }
    for ((index, module), face) in modules.iter().enumerate().zip(config.armor_faces()) {
        if module["id"] != index || module["kind"] != if hero { "AM12" } else { "AM02" } {
            return Err(invalid());
        }
        let pose: Pose =
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
    Ok(ArmorIdentity::Replacement {
        kind: if hero {
            ArmorKind::Am12
        } else {
            ArmorKind::Am02
        },
    })
}

#[cfg(test)]
mod tests;
