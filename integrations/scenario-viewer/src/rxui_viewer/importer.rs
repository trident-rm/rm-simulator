// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Offline reviewed import model. No native CAD parser or automatic mate solver.
use bevy::math::{DMat3, DQuat, DVec3};
use nalgebra::{Matrix3, SymmetricEigen};
use rm_sim_engine::{Pose, model::*};
use rm_sim_physics::{
    StepDuration,
    cooking::CookedAssembly,
    runtime::{AssemblyKeys, BodyKey, JointKey, PhysicalRuntime},
};
use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::Path,
    sync::Arc,
};
type Result<T> = std::result::Result<T, String>;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Part {
    pub id: u64,
    pub name: String,
    pub occurrence: String,
    pub source_sha256: String,
    pub pose: Pose,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mesh: Option<String>,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub configuration: String,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub vertices_m: Vec<[f64; 3]>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub triangles: Vec<[u32; 3]>,
    pub group: u64,
    pub native_cache: Option<serde_json::Value>,
    pub override_mass: Option<Mass>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshDefinition {
    pub source_sha256: String,
    pub configuration: String,
    pub mapping_evidence: String,
    pub vertices_m: Arc<[[f64; 3]]>,
    pub triangles: Arc<[[u32; 3]]>,
}
#[derive(Clone, Copy)]
pub struct MeshView<'a> {
    pub vertices: &'a [[f64; 3]],
    pub triangles: &'a [[u32; 3]],
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Mass {
    pub mass_kg: f64,
    pub com_m: [f64; 3],
    /// Symmetric central tensor in the part/group frame, kg m²; not products of inertia.
    pub inertia_kg_m2: [[f64; 3]; 3],
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Proxy {
    PerPartBoxes,
    Box,
    Sphere,
    Hull,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum MassMode {
    Unspecified,
    ReviewedCache { products_sign: f64 },
    EstimatedBoxDensity { density_kg_m3: f64 },
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Group {
    pub id: u64,
    pub name: String,
    pub pose: Pose,
    pub fixed: bool,
    pub proxy: Proxy,
    pub mass_mode: MassMode,
    pub joint: Option<Joint>,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Motion {
    Fixed,
    Revolute,
    Prismatic,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Joint {
    pub parent: u64,
    pub motion: Motion,
    pub pivot_m: [f64; 3],
    pub axis: [f64; 3],
    pub limits: Option<[f64; 2]>,
    pub reviewed: bool,
    pub evidence: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Project {
    pub schema: String,
    pub source: String,
    #[serde(default)]
    pub provenance: std::collections::BTreeMap<String, String>,
    pub source_frames_and_geometry_reviewed: bool,
    pub diagnostics: Vec<String>,
    #[serde(default)]
    pub omitted_occurrences: Vec<String>,
    pub parts: Vec<Part>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub meshes: BTreeMap<String, MeshDefinition>,
    pub groups: Vec<Group>,
}
#[derive(Serialize)]
pub struct GroupReport {
    pub group: u64,
    pub mass: Mass,
    pub principal_inertia_kg_m2: [f64; 3],
    pub principal_frame: Pose,
    pub colliders: usize,
    pub vertices: usize,
    pub mass_basis: String,
}

pub fn quat(p: Pose) -> DQuat {
    let [w, x, y, z] = p.rotation_wxyz;
    DQuat::from_xyzw(x, y, z, w)
}
pub fn transform(p: Pose, v: [f64; 3]) -> DVec3 {
    quat(p) * DVec3::from_array(v) + DVec3::from_array(p.translation_m)
}
fn local(p: Pose, v: DVec3) -> DVec3 {
    quat(p).conjugate() * (v - DVec3::from_array(p.translation_m))
}
fn compose(a: Pose, b: Pose) -> Pose {
    let q = quat(a) * quat(b);
    Pose {
        translation_m: transform(a, b.translation_m).to_array(),
        rotation_wxyz: [q.w, q.x, q.y, q.z],
    }
}
fn inverse(p: Pose) -> Pose {
    let q = quat(p).conjugate();
    Pose {
        translation_m: (q * -DVec3::from_array(p.translation_m)).to_array(),
        rotation_wxyz: [q.w, q.x, q.y, q.z],
    }
}
fn mat(a: [[f64; 3]; 3]) -> DMat3 {
    DMat3::from_cols_array(&[
        a[0][0], a[1][0], a[2][0], a[0][1], a[1][1], a[2][1], a[0][2], a[1][2], a[2][2],
    ])
}
fn array(a: DMat3) -> [[f64; 3]; 3] {
    let c = a.to_cols_array_2d();
    std::array::from_fn(|i| std::array::from_fn(|j| c[j][i]))
}
fn outer(v: DVec3) -> DMat3 {
    DMat3::from_cols(v * v.x, v * v.y, v * v.z)
}

impl Project {
    pub fn set_mass_override(&mut self, part: usize, mass: Mass) -> Result<()> {
        principal(&mass)?;
        self.parts
            .get_mut(part)
            .ok_or("missing selected part")?
            .override_mass = Some(mass);
        Ok(())
    }
    /// Explicitly omit unresolved source edges. No existing groups/joints are
    /// redirected: missing occurrences were never promoted to physical bodies.
    pub fn omit_missing_references(&mut self) -> usize {
        let before = self.omitted_occurrences.len();
        self.diagnostics.retain(|d| {
            if d.ends_with(": unresolved active reference") {
                self.omitted_occurrences.push(d.clone());
                false
            } else {
                true
            }
        });
        self.omitted_occurrences.len() - before
    }
    /// Kinematic review only: rotate/slide one subtree without altering the recipe.
    pub fn preview(&self, group: u64, coordinate: f64) -> Result<Self> {
        let mut result = self.clone();
        if coordinate == 0. {
            return Ok(result);
        }
        let j = self
            .groups
            .iter()
            .find(|g| g.id == group)
            .and_then(|g| g.joint.as_ref())
            .ok_or("selected root has no joint")?;
        if !coordinate.is_finite()
            || j.limits
                .is_some_and(|[lo, hi]| coordinate < lo || coordinate > hi)
        {
            return Err("preview outside joint limits".into());
        }
        let axis = DVec3::from_array(j.axis);
        if !axis.is_finite() || !axis.length().is_finite() || axis.length() < 1e-10 {
            return Err("invalid preview axis".into());
        }
        let delta = match j.motion {
            Motion::Fixed => return Err("fixed joint cannot move".into()),
            Motion::Prismatic => Pose::at((axis.normalize() * coordinate).to_array()),
            Motion::Revolute => {
                let q = DQuat::from_axis_angle(axis.normalize(), coordinate);
                let pivot = DVec3::from_array(j.pivot_m);
                Pose {
                    translation_m: (pivot - q * pivot).to_array(),
                    rotation_wxyz: [q.w, q.x, q.y, q.z],
                }
            }
        };
        let mut subtree = BTreeSet::from([group]);
        loop {
            let n = subtree.len();
            for g in &self.groups {
                if g.joint
                    .as_ref()
                    .is_some_and(|j| subtree.contains(&j.parent))
                {
                    subtree.insert(g.id);
                }
            }
            if n == subtree.len() {
                break;
            }
        }
        for p in &mut result.parts {
            if subtree.contains(&p.group) {
                p.pose = compose(delta, p.pose);
            }
        }
        for g in &mut result.groups {
            if subtree.contains(&g.id) {
                g.pose = compose(delta, g.pose);
            }
        }
        Ok(result)
    }
    pub fn load(path: &Path) -> Result<Self> {
        use std::io::Read;
        let file = fs::File::open(path).map_err(|e| e.to_string())?;
        let mut data = Vec::new();
        file.take(256 * 1024 * 1024 + 1)
            .read_to_end(&mut data)
            .map_err(|e| e.to_string())?;
        if data.len() > 256 * 1024 * 1024 {
            return Err("import project exceeds 256 MiB".into());
        }
        #[derive(Deserialize)]
        struct Envelope {
            schema: String,
            project: Option<Project>,
        }
        let envelope: Envelope = serde_json::from_slice(&data).map_err(|e| e.to_string())?;
        let project: Self = if matches!(
            envelope.schema.as_str(),
            "rm-reviewed-solidworks-recipe-v1" | "rm-reviewed-solidworks-recipe-v2"
        ) {
            envelope
                .project
                .ok_or("recipe has no reconstruction project")?
        } else {
            serde_json::from_slice(&data).map_err(|e| e.to_string())?
        };
        project.validate()?;
        Ok(project)
    }
    pub fn validate(&self) -> Result<()> {
        if !matches!(
            self.schema.as_str(),
            "rm-solidworks-project-v1" | "rm-solidworks-project-v2"
        ) || self.parts.is_empty()
            || self.parts.len() > 4096
            || self.groups.is_empty()
            || self.groups.len() > 256
        {
            return Err("invalid project schema/part/group limits".into());
        }
        let mut ids = BTreeSet::new();
        for g in &self.groups {
            if !ids.insert(g.id) {
                return Err("duplicate group ID".into());
            }
            BodyState::at(g.pose).validate().map_err(str::to_owned)?;
            match g.mass_mode {
                MassMode::ReviewedCache { products_sign }
                    if products_sign != -1. && products_sign != 1. =>
                {
                    return Err("invalid product convention".into());
                }
                MassMode::EstimatedBoxDensity { density_kg_m3 }
                    if !density_kg_m3.is_finite() || density_kg_m3 <= 0. =>
                {
                    return Err("invalid density".into());
                }
                _ => (),
            }
            if let Some(j) = &g.joint
                && (j
                    .axis
                    .iter()
                    .chain(j.pivot_m.iter())
                    .any(|x| !x.is_finite())
                    || j.limits
                        .is_some_and(|v| v.iter().any(|x| !x.is_finite()) || v[0] > v[1]))
            {
                return Err("invalid joint quantities".into());
            }
        }
        let v2 = self.schema == "rm-solidworks-project-v2";
        if self.meshes.len() > 4096 || (!v2 && !self.meshes.is_empty()) {
            return Err("invalid mesh library/schema".into());
        }
        let mut vertices = 0;
        for mesh in self.meshes.values() {
            if mesh.source_sha256.len() != 64
                || !mesh.source_sha256.bytes().all(|b| b.is_ascii_hexdigit())
                || mesh.configuration.is_empty()
                || mesh.mapping_evidence.is_empty()
            {
                return Err("invalid shared mesh provenance".into());
            }
            validate_mesh(&mesh.vertices_m, &mesh.triangles)?;
            vertices += mesh.vertices_m.len();
        }
        let mut parts = BTreeSet::new();
        let mut placed = 0;
        for p in &self.parts {
            if !parts.insert(p.id)
                || !ids.contains(&p.group)
                || p.source_sha256.len() != 64
                || !p.source_sha256.bytes().all(|b| b.is_ascii_hexdigit())
            {
                return Err("invalid part identity/group/provenance".into());
            }
            BodyState::at(p.pose).validate().map_err(str::to_owned)?;
            if let Some(m) = &p.override_mass {
                principal(m)?;
            }
            if v2 {
                if p.mesh.is_none()
                    || p.configuration.is_empty()
                    || !p.vertices_m.is_empty()
                    || !p.triangles.is_empty()
                {
                    return Err("v2 occurrences require only a shared geometry reference".into());
                }
            } else if p.mesh.is_some() {
                return Err("v1 occurrences require inline geometry".into());
            }
            let mesh = self.mesh(p)?;
            if p.mesh.is_none() {
                validate_mesh(mesh.vertices, mesh.triangles)?;
                vertices += mesh.vertices.len();
            }
            placed += mesh.vertices.len();
        }
        if vertices > 2_000_000 || placed > 8_000_000 {
            return Err("project exceeds unique/placed vertex budget".into());
        }
        Ok(())
    }
    pub fn save(&self, path: &Path) -> Result<()> {
        self.validate()?;
        write_json(path, self)
    }
    pub fn mesh<'a>(&'a self, p: &'a Part) -> Result<MeshView<'a>> {
        if let Some(key) = &p.mesh {
            let mesh = self
                .meshes
                .get(key)
                .ok_or("missing shared mesh definition")?;
            if mesh.source_sha256 != p.source_sha256 || mesh.configuration != p.configuration {
                return Err("shared mesh source/configuration mismatch".into());
            }
            Ok(MeshView {
                vertices: &mesh.vertices_m,
                triangles: &mesh.triangles,
            })
        } else {
            Ok(MeshView {
                vertices: &p.vertices_m,
                triangles: &p.triangles,
            })
        }
    }
    pub fn points(&self, g: &Group, p: &Part) -> Result<Vec<[f64; 3]>> {
        Ok(self
            .mesh(p)?
            .vertices
            .iter()
            .map(|&v| local(g.pose, transform(p.pose, v)).to_array())
            .collect())
    }
    pub fn proxies(&self, g: &Group) -> Result<Vec<(Pose, ShapeDescription)>> {
        let pieces: Vec<_> = self
            .parts
            .iter()
            .filter(|p| p.group == g.id)
            .map(|p| self.points(g, p))
            .collect::<Result<Vec<_>>>()?;
        if pieces.is_empty() {
            return Err(format!("group {} is empty", g.id));
        }
        if g.proxy == Proxy::PerPartBoxes {
            return pieces.iter().map(|p| box_shape(p)).collect();
        }
        let points: Vec<_> = pieces.into_iter().flatten().collect();
        match g.proxy {
            Proxy::Box => Ok(vec![box_shape(&points)?]),
            Proxy::Sphere => {
                let (lo, hi) = bounds(&points)?;
                let center = (lo + hi) * 0.5;
                let radius = points
                    .iter()
                    .map(|p| (DVec3::from_array(*p) - center).length())
                    .fold(0., f64::max)
                    .max(1e-5);
                Ok(vec![(
                    Pose::at(center.to_array()),
                    ShapeDescription::Ball { radius_m: radius },
                )])
            }
            Proxy::Hull => Ok(vec![(
                Pose::default(),
                ShapeDescription::ConvexHull(ConvexHull::new(points).map_err(str::to_owned)?),
            )]),
            Proxy::PerPartBoxes => unreachable!(),
        }
    }
    pub fn mass(&self, g: &Group) -> Result<(Mass, Inertial)> {
        let mut entries = Vec::new();
        let mut total = 0.;
        let mut center = DVec3::ZERO;
        for p in self.parts.iter().filter(|p| p.group == g.id) {
            let m = if let Some(m) = &p.override_mass {
                m.clone()
            } else {
                part_mass(p, &g.mass_mode, self.mesh(p)?.vertices)?
            };
            principal(&m)?;
            let r = DMat3::from_quat(quat(g.pose).conjugate() * quat(p.pose));
            let c = local(g.pose, transform(p.pose, m.com_m));
            total += m.mass_kg;
            center += c * m.mass_kg;
            entries.push((m.mass_kg, c, r * mat(m.inertia_kg_m2) * r.transpose()));
        }
        if total <= 0. || !total.is_finite() {
            return Err("missing/overflowing group mass".into());
        }
        center /= total;
        let mut inertia = DMat3::ZERO;
        for (mass, c, tensor) in entries {
            let d = c - center;
            inertia += tensor + (DMat3::IDENTITY * d.length_squared() - outer(d)) * mass;
        }
        let m = Mass {
            mass_kg: total,
            com_m: center.to_array(),
            inertia_kg_m2: array(inertia),
        };
        let inertial = principal(&m)?;
        Ok((m, inertial))
    }
    pub fn assembly(&self) -> Result<(CookedAssembly, Vec<GroupReport>)> {
        self.validate()?;
        if !self.source_frames_and_geometry_reviewed {
            return Err("review native frames, configuration geometry and completeness before physical export".into());
        }
        if !self.diagnostics.is_empty() {
            return Err(
                "unresolved source diagnostics block physical export; repair the input".into(),
            );
        }
        let roots: Vec<_> = self.groups.iter().filter(|g| g.joint.is_none()).collect();
        if roots.len() != 1 {
            return Err("physical model requires one explicit root".into());
        }
        let mut links = Vec::new();
        let mut joints = Vec::new();
        let mut report = Vec::new();
        for g in &self.groups {
            let (mass, inertial) = self.mass(g)?;
            let proxies = self.proxies(g)?;
            report.push(GroupReport {
                group: g.id,
                mass,
                principal_inertia_kg_m2: inertial.principal_inertia_kg_m2(),
                principal_frame: inertial.frame(),
                colliders: proxies.len(),
                vertices: self
                    .parts
                    .iter()
                    .filter(|p| p.group == g.id)
                    .map(|p| self.mesh(p).map(|m| m.vertices.len()))
                    .collect::<Result<Vec<_>>>()?
                    .into_iter()
                    .sum(),
                mass_basis: format!(
                    "{:?}; explicit per-part overrides take precedence",
                    g.mass_mode
                ),
            });
            let colliders = proxies
                .into_iter()
                .map(|(pose, shape)| {
                    ColliderDescription::new(
                        shape,
                        pose,
                        ContactMaterial {
                            friction: 0.5,
                            restitution: 0.,
                            memberships: u32::MAX,
                            filter: u32::MAX,
                        },
                    )
                })
                .collect::<std::result::Result<Vec<_>, _>>()
                .map_err(str::to_owned)?;
            let body = RigidBodyDescription::new(
                if g.fixed {
                    BodyKind::Fixed
                } else {
                    BodyKind::Dynamic(inertial)
                },
                colliders,
            )
            .map_err(str::to_owned)?;
            links.push(
                AssemblyLink::new(AssemblyLinkId(g.id), body, BodyState::at(g.pose))
                    .map_err(str::to_owned)?,
            );
            if let Some(j) = &g.joint {
                if !j.reviewed || j.evidence.trim().is_empty() {
                    return Err("joint requires reviewed evidence".into());
                }
                let parent = self
                    .groups
                    .iter()
                    .find(|p| p.id == j.parent)
                    .ok_or("missing joint parent")?;
                let axis = DVec3::from_array(j.axis);
                if !axis.is_finite()
                    || axis.length() < 1e-10
                    || !DVec3::from_array(j.pivot_m).is_finite()
                {
                    return Err("invalid joint axis/pivot".into());
                }
                let q = DQuat::from_rotation_arc(DVec3::X, axis.normalize());
                let frame = Pose {
                    translation_m: j.pivot_m,
                    rotation_wxyz: [q.w, q.x, q.y, q.z],
                };
                let kind = match j.motion {
                    Motion::Fixed => JointKind::Fixed,
                    Motion::Revolute => JointKind::Revolute,
                    Motion::Prismatic => JointKind::Prismatic,
                };
                let limits = j
                    .limits
                    .map(|[a, b]| JointLimits::new(a, b))
                    .transpose()
                    .map_err(str::to_owned)?;
                let joint = JointDescription::new(
                    kind,
                    compose(inverse(parent.pose), frame),
                    compose(inverse(g.pose), frame),
                    limits,
                    false,
                )
                .map_err(str::to_owned)?;
                joints.push(AssemblyJoint::new(
                    AssemblyJointId(g.id),
                    AssemblyLinkId(parent.id),
                    AssemblyLinkId(g.id),
                    joint,
                ));
            }
        }
        let source = AssemblyDescription::new(AssemblyLinkId(roots[0].id), links, joints)
            .map_err(str::to_owned)?;
        Ok((CookedAssembly::new(source).map_err(str::to_owned)?, report))
    }
    pub fn check_runtime(&self) -> Result<Vec<GroupReport>> {
        let (model, report) = self.assembly()?;
        let keys = AssemblyKeys {
            links: model
                .source()
                .links()
                .iter()
                .map(|l| (l.id(), BodyKey(l.id().0)))
                .collect(),
            joints: model
                .source()
                .joints()
                .iter()
                .map(|j| (j.id(), JointKey(j.id().0)))
                .collect(),
        };
        let mut runtime = PhysicalRuntime::new(StepDuration::HZ_128);
        runtime
            .install_assembly(&model, Pose::default(), &keys)
            .map_err(str::to_owned)?;
        Ok(report)
    }
    pub fn export(&self, path: &Path) -> Result<()> {
        let report = self.check_runtime()?;
        write_json(
            path,
            &serde_json::json!({"schema":if self.schema=="rm-solidworks-project-v2" {"rm-reviewed-solidworks-recipe-v2"}else{"rm-reviewed-solidworks-recipe-v1"},"project":self,"groups":report,"continuation":"recipe reconstruction only; no native checkpoint","collision":"explicit approximations independent of mass"}),
        )
    }
}

fn validate_mesh(vertices: &[[f64; 3]], triangles: &[[u32; 3]]) -> Result<()> {
    if vertices.is_empty()
        || vertices.len() > 250_000
        || triangles.len() > 500_000
        || vertices.iter().flatten().any(|v| !v.is_finite())
        || triangles
            .iter()
            .flatten()
            .any(|&i| i as usize >= vertices.len())
    {
        return Err("invalid/beyond-limit mesh definition".into());
    }
    Ok(())
}

fn write_json(path: &Path, value: &impl Serialize) -> Result<()> {
    // Serialize completely before replacing the destination. Never write native CAD.
    if path.extension().is_none_or(|x| x != "json") {
        return Err("output must be a JSON sidecar".into());
    }
    let bytes = serde_json::to_vec_pretty(value).map_err(|e| e.to_string())?;
    let temp = tempfile_path(path);
    let mut f = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&temp)
        .map_err(|e| e.to_string())?;
    let result = (|| {
        use std::io::Write;
        f.write_all(&bytes).map_err(|e| e.to_string())?;
        drop(f);
        fs::rename(&temp, path).map_err(|e| e.to_string())
    })();
    if result.is_err() {
        let _ = fs::remove_file(&temp);
    }
    result
}
fn tempfile_path(path: &Path) -> std::path::PathBuf {
    let mut p = path.as_os_str().to_owned();
    p.push(format!(".{}.tmp", std::process::id()));
    p.into()
}
fn bounds(points: &[[f64; 3]]) -> Result<(DVec3, DVec3)> {
    let mut lo = DVec3::splat(f64::INFINITY);
    let mut hi = -lo;
    for &p in points {
        let p = DVec3::from_array(p);
        lo = lo.min(p);
        hi = hi.max(p);
    }
    if !lo.is_finite() || !hi.is_finite() {
        return Err("invalid geometry bounds".into());
    }
    Ok((lo, hi))
}
fn box_shape(points: &[[f64; 3]]) -> Result<(Pose, ShapeDescription)> {
    let (lo, hi) = bounds(points)?;
    Ok((
        Pose::at(((lo + hi) * 0.5).to_array()),
        ShapeDescription::Cuboid {
            half_extents_m: ((hi - lo) * 0.5).max(DVec3::splat(1e-5)).to_array(),
        },
    ))
}
fn part_mass(p: &Part, mode: &MassMode, vertices: &[[f64; 3]]) -> Result<Mass> {
    match mode {
        MassMode::Unspecified => Err(format!(
            "{}: choose a mass basis or explicit override",
            p.name
        )),
        MassMode::EstimatedBoxDensity { density_kg_m3 } => {
            if !density_kg_m3.is_finite() || *density_kg_m3 <= 0. {
                return Err("invalid estimated density".into());
            }
            let (lo, hi) = bounds(vertices)?;
            let d = hi - lo;
            let mass = d.x * d.y * d.z * density_kg_m3;
            Ok(Mass {
                mass_kg: mass,
                com_m: ((lo + hi) * 0.5).to_array(),
                inertia_kg_m2: [
                    [mass * (d.y * d.y + d.z * d.z) / 12., 0., 0.],
                    [0., mass * (d.x * d.x + d.z * d.z) / 12., 0.],
                    [0., 0., mass * (d.x * d.x + d.y * d.y) / 12.],
                ],
            })
        }
        MassMode::ReviewedCache { products_sign } => {
            if *products_sign != -1. && *products_sign != 1. {
                return Err("choose an explicit cached product-of-inertia sign convention".into());
            }
            let cache = p
                .native_cache
                .as_ref()
                .ok_or_else(|| format!("{}: no native cached mass", p.name))?;
            let number = |v: &serde_json::Value| {
                v.as_f64()
                    .ok_or_else(|| "missing native cached quantity".to_owned())
            };
            let m = number(&cache["mass"])?;
            let com = std::array::from_fn(|i| {
                number(&cache["center_of_gravity"][["x", "y", "z"][i]]).unwrap_or(f64::NAN)
            });
            let diagonal = std::array::from_fn::<_, 3, _>(|i| {
                number(&cache["moments_of_inertia"][i]).unwrap_or(f64::NAN)
            });
            let product = std::array::from_fn::<_, 3, _>(|i| {
                number(&cache["products_of_inertia"][i]).unwrap_or(f64::NAN) * products_sign
            });
            Ok(Mass {
                mass_kg: m,
                com_m: com,
                inertia_kg_m2: [
                    [diagonal[0], product[0], product[1]],
                    [product[0], diagonal[1], product[2]],
                    [product[1], product[2], diagonal[2]],
                ],
            })
        }
    }
}
fn principal(m: &Mass) -> Result<Inertial> {
    if !m.mass_kg.is_finite()
        || m.mass_kg <= 0.
        || m.com_m.iter().any(|x| !x.is_finite())
        || m.inertia_kg_m2.iter().flatten().any(|x| !x.is_finite())
    {
        return Err("nonfinite/nonpositive mass or tensor".into());
    }
    let scale = m
        .inertia_kg_m2
        .iter()
        .flatten()
        .map(|x| x.abs())
        .fold(0., f64::max);
    if scale <= 0. {
        return Err("zero inertia".into());
    }
    for i in 0..3 {
        for j in 0..3 {
            if (m.inertia_kg_m2[i][j] - m.inertia_kg_m2[j][i]).abs() > scale * 1e-10 {
                return Err("nonsymmetric inertia".into());
            }
        }
    }
    let normalized = mat(m.inertia_kg_m2) / scale;
    let eigen = SymmetricEigen::new(Matrix3::from_column_slice(&normalized.to_cols_array()));
    let mut basis = DMat3::from_cols_array(eigen.eigenvectors.as_slice().try_into().unwrap());
    if basis.determinant() < 0. {
        basis.x_axis = -basis.x_axis;
    }
    let q = DQuat::from_mat3(&basis).normalize();
    Inertial::new(
        m.mass_kg,
        [
            eigen.eigenvalues[0] * scale,
            eigen.eigenvalues[1] * scale,
            eigen.eigenvalues[2] * scale,
        ],
        Pose {
            translation_m: m.com_m,
            rotation_wxyz: [q.w, q.x, q.y, q.z],
        },
    )
    .map_err(str::to_owned)
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;
    pub fn fixture() -> Project {
        let points = vec![
            [-0.5, -0.5, -0.5],
            [0.5, -0.5, -0.5],
            [-0.5, 0.5, -0.5],
            [-0.5, -0.5, 0.5],
            [0.5, 0.5, 0.5],
        ];
        let part = |id, x| Part {
            id,
            name: format!("part {id}"),
            occurrence: format!("root/{id}"),
            source_sha256: "0".repeat(64),
            pose: Pose::at([x, 0., 0.]),
            mesh: None,
            configuration: String::new(),
            vertices_m: points.clone(),
            triangles: vec![[0, 1, 2], [0, 1, 3]],
            group: 1,
            native_cache: None,
            override_mass: Some(Mass {
                mass_kg: 1.,
                com_m: [0.; 3],
                inertia_kg_m2: [[1. / 6., 0., 0.], [0., 1. / 6., 0.], [0., 0., 1. / 6.]],
            }),
        };
        Project {
            schema: "rm-solidworks-project-v1".into(),
            source: "synthetic".into(),
            provenance: Default::default(),
            source_frames_and_geometry_reviewed: true,
            diagnostics: vec![],
            omitted_occurrences: vec![],
            parts: vec![part(1, -1.), part(2, 1.)],
            meshes: Default::default(),
            groups: vec![Group {
                id: 1,
                name: "base".into(),
                pose: Pose::default(),
                fixed: false,
                proxy: Proxy::PerPartBoxes,
                mass_mode: MassMode::Unspecified,
                joint: None,
            }],
        }
    }
    pub fn shared_fixture() -> Project {
        let mut p = fixture();
        p.schema = "rm-solidworks-project-v2".into();
        let mesh = MeshDefinition {
            source_sha256: p.parts[0].source_sha256.clone(),
            configuration: "default".into(),
            mapping_evidence: "synthetic fixture".into(),
            vertices_m: Arc::from(p.parts[0].vertices_m.clone()),
            triangles: Arc::from(p.parts[0].triangles.clone()),
        };
        p.meshes.insert("shared".into(), mesh);
        for part in &mut p.parts {
            part.mesh = Some("shared".into());
            part.configuration = "default".into();
            part.vertices_m.clear();
            part.triangles.clear();
        }
        p
    }

    #[test]
    fn shared_definitions_retain_legacy_geometry_and_mass_without_occurrence_copies() {
        let old = fixture();
        let shared = shared_fixture();
        shared.validate().unwrap();
        for (a, b) in old.parts.iter().zip(&shared.parts) {
            assert_eq!(a.pose, b.pose);
            assert_eq!(
                old.mesh(a).unwrap().vertices,
                shared.mesh(b).unwrap().vertices
            );
            assert_eq!(
                old.mesh(a).unwrap().triangles,
                shared.mesh(b).unwrap().triangles
            );
        }
        let preview = shared.preview(1, 0.).unwrap();
        assert!(Arc::ptr_eq(
            &shared.meshes["shared"].vertices_m,
            &preview.meshes["shared"].vertices_m
        ));
        assert!(std::ptr::eq(
            shared.mesh(&shared.parts[0]).unwrap().vertices,
            shared.mesh(&shared.parts[1]).unwrap().vertices
        ));
        assert_eq!(
            old.mass(&old.groups[0]).unwrap().0.inertia_kg_m2,
            shared.mass(&shared.groups[0]).unwrap().0.inertia_kg_m2
        );
        shared.check_runtime().unwrap();
        let mut weighted = shared.clone();
        weighted.parts[1].override_mass.as_mut().unwrap().mass_kg = 2.;
        let (mass, _) = weighted.mass(&weighted.groups[0]).unwrap();
        assert_eq!(mass.mass_kg, 3.);
        assert!((mass.com_m[0] - 1. / 3.).abs() < 1e-12);
        assert!(Arc::ptr_eq(
            &weighted.meshes["shared"].vertices_m,
            &shared.meshes["shared"].vertices_m
        ));
        let mut bad = shared.clone();
        bad.parts[1].configuration = "other length".into();
        assert!(bad.validate().is_err());
        bad = shared.clone();
        bad.parts[1].mesh = Some("missing".into());
        assert!(bad.validate().is_err());
        bad = shared;
        bad.parts[1].vertices_m.push([0.; 3]);
        assert!(bad.validate().is_err());
    }
    #[test]
    fn aggregate_preserves_mass_com_and_parallel_axis_tensor() {
        let p = fixture();
        let (m, i) = p.mass(&p.groups[0]).unwrap();
        assert_eq!(m.mass_kg, 2.);
        assert_eq!(m.com_m, [0.; 3]);
        assert!((m.inertia_kg_m2[0][0] - 1. / 3.).abs() < 1e-12);
        assert!((m.inertia_kg_m2[1][1] - 7. / 3.).abs() < 1e-12);
        let rebuilt = DMat3::from_quat(quat(i.frame()))
            * DMat3::from_diagonal(DVec3::from_array(i.principal_inertia_kg_m2()))
            * DMat3::from_quat(quat(i.frame())).transpose();
        assert!(
            (rebuilt - mat(m.inertia_kg_m2))
                .to_cols_array()
                .iter()
                .all(|x| x.abs() < 1e-12)
        );
    }
    #[test]
    fn tensor_rotates_before_parallel_axis_and_proxy_never_changes_mass() {
        let mut p = fixture();
        p.parts.truncate(1);
        p.parts[0].pose = Pose {
            translation_m: [0.; 3],
            rotation_wxyz: [
                std::f64::consts::FRAC_1_SQRT_2,
                0.,
                0.,
                std::f64::consts::FRAC_1_SQRT_2,
            ],
        };
        p.parts[0].override_mass.as_mut().unwrap().inertia_kg_m2 =
            [[1., 0., 0.], [0., 2., 0.], [0., 0., 3.]];
        let (m, _) = p.mass(&p.groups[0]).unwrap();
        assert!((m.inertia_kg_m2[0][0] - 2.).abs() < 1e-12);
        for proxy in [Proxy::Box, Proxy::Sphere, Proxy::Hull, Proxy::PerPartBoxes] {
            p.groups[0].proxy = proxy;
            let (same, _) = p.mass(&p.groups[0]).unwrap();
            assert_eq!(same.inertia_kg_m2, m.inertia_kg_m2);
            p.check_runtime().unwrap();
        }
    }
    #[test]
    fn explicit_joint_installs_with_coincident_anchors_and_source_poses() {
        let mut p = fixture();
        p.parts[1].group = 2;
        let mut g = p.groups[0].clone();
        g.id = 2;
        g.name = "moving".into();
        g.pose = p.parts[1].pose;
        g.joint = Some(Joint {
            parent: 1,
            motion: Motion::Revolute,
            pivot_m: [0.5, 0., 0.],
            axis: [0., 0., 1.],
            limits: Some([-0.8, 0.8]),
            reviewed: true,
            evidence: "manual fixture".into(),
        });
        p.groups.push(g);
        let (c, _) = p.assembly().unwrap();
        assert_eq!(c.source().links()[1].initial().pose, p.parts[1].pose);
        p.check_runtime().unwrap();
        p.groups[1].joint.as_mut().unwrap().reviewed = false;
        assert!(p.check_runtime().is_err());
        p.groups[1].joint.as_mut().unwrap().reviewed = true;
        p.groups[1].joint.as_mut().unwrap().parent = 2;
        assert!(p.check_runtime().is_err());
    }
    #[test]
    fn missing_mass_unreviewed_frames_bad_tensor_and_diagnostics_fail_closed() {
        let mut p = fixture();
        p.source_frames_and_geometry_reviewed = false;
        assert!(p.check_runtime().is_err());
        p.source_frames_and_geometry_reviewed = true;
        p.diagnostics.push("missing active part".into());
        assert!(p.check_runtime().is_err());
        p.diagnostics.clear();
        p.parts[0].override_mass = None;
        assert!(p.check_runtime().is_err());
        p.groups[0].mass_mode = MassMode::EstimatedBoxDensity {
            density_kg_m3: 1000.,
        };
        assert!(p.check_runtime().is_ok());
        p.parts[1].override_mass.as_mut().unwrap().inertia_kg_m2[2][2] = 100.;
        assert!(p.check_runtime().is_err());
    }

    #[test]
    fn omission_is_explicit_and_does_not_hide_bad_geometry() {
        let mut p = fixture();
        p.diagnostics = vec![
            "root/occurrence:1: unresolved active reference".into(),
            "root: unchecked occurrence transform".into(),
        ];
        assert_eq!(p.omit_missing_references(), 1);
        assert_eq!(p.omitted_occurrences.len(), 1);
        assert_eq!(p.diagnostics.len(), 1);
        assert!(p.check_runtime().is_err());
        p.diagnostics.clear();
        assert!(p.check_runtime().is_ok());
    }

    #[test]
    fn exported_recipe_reconstructs_and_failed_export_preserves_destination() {
        let mut n = 0;
        let dir = loop {
            let p = std::env::temp_dir()
                .join(format!("rm-solidworks-import-{}-{n}", std::process::id()));
            match fs::create_dir(&p) {
                Ok(()) => break p,
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => n += 1,
                Err(e) => panic!("{e}"),
            }
        };
        let path = dir.join("model.json");
        for mut p in [fixture(), shared_fixture()] {
            p.export(&path).unwrap();
            let original = fs::read(&path).unwrap();
            let restored = Project::load(&path).unwrap();
            assert_eq!(restored.check_runtime().unwrap()[0].mass.mass_kg, 2.);
            p.source_frames_and_geometry_reviewed = false;
            assert!(p.export(&path).is_err());
            assert_eq!(fs::read(&path).unwrap(), original);
            assert!(p.save(&dir.join("original.SLDASM")).is_err());
        }
        fs::remove_file(path).unwrap();
        fs::remove_dir(dir).unwrap();
    }

    #[test]
    fn hinge_preview_preserves_local_shapes_without_mutating_source_and_motor_steps() {
        let mut p = fixture();
        p.parts[1].group = 2;
        p.groups[0].fixed = true;
        let mut child = p.groups[0].clone();
        child.id = 2;
        child.fixed = false;
        child.pose = p.parts[1].pose;
        child.joint = Some(Joint {
            parent: 1,
            motion: Motion::Revolute,
            pivot_m: [0.5, 0., 0.],
            axis: [0., 0., 1.],
            limits: Some([-1., 1.]),
            reviewed: true,
            evidence: "explicit fixture".into(),
        });
        p.groups.push(child);
        let before = p.parts[1].pose;
        let shown = p.preview(2, 0.4).unwrap();
        assert_eq!(p.parts[1].pose, before);
        assert!(
            (local(
                shown.groups[1].pose,
                transform(shown.parts[1].pose, [0.; 3])
            ) - local(p.groups[1].pose, transform(p.parts[1].pose, [0.; 3])))
            .length()
                < 1e-12
        );
        assert!(p.preview(2, 1.1).is_err());
        let (model, _) = p.assembly().unwrap();
        let keys = AssemblyKeys {
            links: vec![
                (AssemblyLinkId(1), BodyKey(1)),
                (AssemblyLinkId(2), BodyKey(2)),
            ],
            joints: vec![(AssemblyJointId(2), JointKey(2))],
        };
        let mut runtime = PhysicalRuntime::new(StepDuration::HZ_128);
        runtime
            .install_assembly(&model, Pose::default(), &keys)
            .unwrap();
        let binding = runtime.joint_binding(JointKey(2)).unwrap();
        runtime
            .set_joint_motor(
                binding,
                Some(JointMotor::new(0.2, 0., 40., 5., 5.).unwrap()),
            )
            .unwrap();
        for i in 0..128 {
            runtime.step(i * 7_812_500).unwrap();
        }
        assert!((runtime.joint_state(binding).unwrap().position - 0.2).abs() < 0.03);
    }
}
