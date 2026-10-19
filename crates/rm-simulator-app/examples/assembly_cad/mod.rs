// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Source-qualified CAD adapter for the standalone assembly workbench only.
use bevy::{math::DQuat, prelude::*};
use rm_simulator_physics::{Pose, manipulator::SerialArm};
use rm_simulator_render::{PoseFlu, apply_pose};
use rm_simulator_world::gameplay::rmuc_2027::assembly_pose::TargetPose;
use serde::Deserialize;
use sha2::{Digest, Sha256};
// Fixed cylinder grasp clocking; the unit gets the inverse offset at rendering.
const GRASP_ROLL_RAD: f64 = -std::f64::consts::FRAC_PI_2;
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};
#[derive(Clone, Deserialize)]
struct Jaw {
    name: String,
    origin_m: [f64; 3],
    axis: [f64; 3],
    open_rad: f64,
    closed_rad: f64,
}
#[derive(Clone, Deserialize)]
struct Config {
    schema: u32,
    assembly_frame: Pose,
    core_envelope_m: Vec<[f64; 3]>,
    engineer: SerialArm,
    core: SerialArm,
    engineer_jaws: Vec<Jaw>,
    core_p_origin_m: [f64; 3],
    core_p_axis: [f64; 3],
    core_q_origin_m: [f64; 3],
    core_q_axis: [f64; 3],
    core_lift_axis: [f64; 3],
}
#[derive(Clone)]
pub struct Rig {
    config: Config,
    engineer_q: [f64; 6],
    core_q: [f64; 6],
    initial_engineer_q: [f64; 6],
    pub engineer_body: Pose,
    pub tool: Pose,
    pub socket: Pose,
    pub requested_pose: Option<TargetPose>,
    pub position_error_m: f64,
    pub angle_error_rad: f64,
    peak_joint_step_rad: f64,
    lift_m: f64,
    p_rad: f64,
    q_rad: f64,
    jaw_angles: Vec<f64>,
    gripped: bool,
    grip_requested: bool,
    pub unit_pose: Pose,
}
fn quaternion(p: Pose) -> DQuat {
    let [w, x, y, z] = p.rotation_wxyz;
    DQuat::from_xyzw(x, y, z, w)
}
fn position(p: Pose) -> bevy::math::DVec3 {
    bevy::math::DVec3::from_array(p.translation_m)
}
fn pose(p: bevy::math::DVec3, q: DQuat) -> Pose {
    Pose {
        translation_m: p.to_array(),
        rotation_wxyz: [q.w, q.x, q.y, q.z],
    }
}
fn compose(a: Pose, b: Pose) -> Pose {
    let q = quaternion(a);
    pose(position(a) + q * position(b), q * quaternion(b))
}
fn inverse(a: Pose) -> Pose {
    let q = quaternion(a).inverse();
    pose(-(q * position(a)), q)
}
fn delta(origin: [f64; 3], axis: [f64; 3], angle: f64) -> Pose {
    let origin = bevy::math::DVec3::from_array(origin);
    let q = DQuat::from_axis_angle(bevy::math::DVec3::from_array(axis), angle);
    pose(origin - q * origin, q)
}
fn translation(v: [f64; 3]) -> Pose {
    Pose::at(v)
}
fn solve_initial(arm: &SerialArm, target: Pose) -> Result<[f64; 6], &'static str> {
    for seed in [
        [0.; 6],
        [0., 1., -1., 0., -1.5, 0.],
        [0., 1., -1., -1., -1.5, 0.],
        [0., 0.8, -1.8, 0., 1., 0.],
        [0., 1.6, 2.2, 0., -1.6, 0.],
        [0., -1.6, 2.2, 0., 1.6, 0.],
        [std::f64::consts::PI, 0.8, -1.8, 0., 1., 0.],
        [-std::f64::consts::PI, 0.8, -1.8, 0., 1., 0.],
    ] {
        if let Ok(q) = arm.solve(target, seed) {
            return Ok(q);
        }
    }
    Err("source CAD arm cannot reach the workbench's initial tool pose")
}
impl Rig {
    pub fn report(&self) -> serde_json::Value {
        serde_json::json!({"engineer_q_rad":self.engineer_q,"core_q_rad":self.core_q,"tool":self.tool,
            "position_error_m":self.position_error_m,"angle_error_rad":self.angle_error_rad,
            "peak_joint_step_rad":self.peak_joint_step_rad,"requested_pose":self.requested_pose,
            "gripped":self.gripped,"grip_requested":self.grip_requested,
            "jaw_angles_rad":self.jaw_angles,"unit_pose":self.unit_pose,
            "p_origin_world_m":compose(self.core_delta(), translation(self.config.core_p_origin_m)).translation_m,
            "post_tip_world_m":compose(self.head_delta(self.lift_m,self.p_rad,self.q_rad), self.config.core.tool).translation_m})
    }
    pub fn load(root: &Path) -> Result<Self, Box<dyn std::error::Error>> {
        Self::load_impl(root, None)
    }
    pub fn load_target(
        root: &Path,
        requested_pose: TargetPose,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        Self::load_impl(root, Some(requested_pose))
    }
    fn load_impl(
        root: &Path,
        requested_pose: Option<TargetPose>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let m: serde_json::Value =
            serde_json::from_slice(&std::fs::read(root.join("manifest.json"))?)?;
        if m["schema"] != 1
            || m["profile"] != "pie-rmuc2027-assembly-workbench-v4"
            || m["source_step_sha256"]
                != "bfb91b499f17488ed1deb523568869f2fcb29436a53cbc39ec5e64ac4a822c47"
            || m["source_field_sha256"]
                != "a5ad3ede3b7674fc98f2e73b71717216383cb2b4b5dcdbe900c2038918bea8e0"
        {
            return Err("unsupported assembly CAD profile or source release".into());
        }
        for (key, file) in [
            ("config", "assembly.json"),
            ("engineer", "engineer.glb"),
            ("core-base", "core-base.glb"),
            ("core-arm", "core-arm.glb"),
            ("core-head", "core-head.glb"),
            ("unit", "unit.glb"),
        ] {
            let hash = format!("{:x}", Sha256::digest(std::fs::read(root.join(file))?));
            if m["files"][key]["file"] != file || m["files"][key]["sha256"] != hash {
                return Err(format!("assembly CAD checksum mismatch: {file}").into());
            }
        }
        let config: Config = serde_json::from_slice(&std::fs::read(root.join("assembly.json"))?)?;
        config.engineer.validate()?;
        config.core.validate()?;
        if config.schema != 4 || config.engineer_jaws.len() != 2 {
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
        let goal = if let Some(requested) = requested_pose {
            let r = requested.rotation_matrix();
            let rotation = bevy::math::DMat3::from_cols_array(&[
                r[0][0], r[1][0], r[2][0], r[0][1], r[1][1], r[2][1], r[0][2], r[1][2], r[2][2],
            ]);
            let goal = compose(
                config.assembly_frame,
                pose(
                    bevy::math::DVec3::from_array(requested.position_m),
                    DQuat::from_mat3(&rotation),
                ),
            );
            // Table 5-10 is a bounding domain, not a guarantee that a pose is legal.
            let placement = compose(
                inverse(config.assembly_frame),
                compose(goal, inverse(config.core.tool)),
            );
            if config.core_envelope_m.is_empty()
                || config.core_envelope_m.iter().any(|p| {
                    let x = position(placement)
                        + quaternion(placement) * bevy::math::DVec3::from_array(*p);
                    !x.is_finite() || x.x > 1e-6
                })
            {
                return Err(
                    "sample violates the Core head's pedestal-front plane; no resampling performed"
                        .into(),
                );
            }
            goal
        } else {
            // Preserve the reviewed fixed fixture for before/after mechanism comparisons.
            let mut goal = config.core.tool;
            goal.translation_m[2] = 0.6;
            goal
        };
        let core_q =
            solve_initial(&config.core, goal).map_err(|e| format!("Core positioning: {e}"))?;
        let socket = config.core.tool_pose(core_q);
        let body = if let Some(requested) = requested_pose {
            compose(
                config.assembly_frame,
                Pose::yawed([0.45, requested.position_m[1], 0.132], std::f64::consts::PI),
            )
        } else {
            Pose::yawed([0.95, socket.translation_m[1], 0.132], std::f64::consts::PI)
        };
        let mut rig = Self {
            jaw_angles: config.engineer_jaws.iter().map(|j| j.open_rad).collect(),
            config,
            unit_pose: Pose::default(),
            gripped: false,
            grip_requested: false,
            engineer_q: [0.; 6],
            initial_engineer_q: [0.; 6],
            core_q,
            engineer_body: body,
            tool: Pose::default(),
            socket,
            requested_pose,
            position_error_m: 0.,
            angle_error_rad: 0.,
            peak_joint_step_rad: 0.,
            lift_m: 0.,
            p_rad: 0.,
            q_rad: 0.,
        };
        let target = rig.target([0.09, 0.2, 0.6], [0.; 3]);
        rig.engineer_q = solve_initial(&rig.config.engineer, compose(inverse(body), target))
            .map_err(|e| format!("Engineer positioning: {e}"))?;
        rig.tool = compose(body, rig.config.engineer.tool_pose(rig.engineer_q));
        rig.initial_engineer_q = rig.engineer_q;
        rig.unit_pose = rig.tool_unit_pose();
        Ok(rig)
    }
    fn tool_unit_pose(&self) -> Pose {
        pose(
            position(self.tool),
            quaternion(self.tool) * DQuat::from_rotation_z(-GRASP_ROLL_RAD),
        )
    }
    pub fn gripped(&self) -> bool {
        self.gripped
    }
    /// Prescribed jaw actuator and proximity-gated attachment, advanced on explicit ticks.
    /// A released unit stays at its last pose; this fixture does not simulate a falling load.
    pub fn step_gripper(&mut self, close: bool, dt_s: f64) {
        const JAW_RATE_RAD_S: f64 = 1.5; // prototype servo policy, not a hardware measurement
        self.grip_requested = close;
        if !close {
            self.gripped = false;
        }
        for (angle, jaw) in self.jaw_angles.iter_mut().zip(&self.config.engineer_jaws) {
            let target = if close { jaw.closed_rad } else { jaw.open_rad };
            let travel = JAW_RATE_RAD_S * dt_s;
            let remaining = target - *angle;
            *angle = if remaining.abs() <= travel {
                target
            } else {
                *angle + remaining.signum() * travel
            };
        }
        let closed = close
            && !self.jaw_angles.is_empty()
            && self
                .jaw_angles
                .iter()
                .zip(&self.config.engineer_jaws)
                .all(|(a, j)| (a - j.closed_rad).abs() < 1e-10);
        let target = self.tool_unit_pose();
        if closed
            && position(target).distance(position(self.unit_pose)) <= 0.005
            && quaternion(target).angle_between(quaternion(self.unit_pose)) <= 5f64.to_radians()
        {
            self.gripped = true;
            self.unit_pose = target;
        }
    }
    pub fn restart(&mut self) {
        self.gripped = false;
        self.grip_requested = false;
        self.jaw_angles = self
            .config
            .engineer_jaws
            .iter()
            .map(|j| j.open_rad)
            .collect();
        self.engineer_q = self.initial_engineer_q;
        self.lift_m = 0.;
        self.p_rad = 0.;
        self.q_rad = 0.;
        self.position_error_m = 0.;
        self.angle_error_rad = 0.;
        self.peak_joint_step_rad = 0.;
        self.tool = compose(
            self.engineer_body,
            self.config.engineer.tool_pose(self.engineer_q),
        );
        self.unit_pose = self.tool_unit_pose();
    }
    fn core_delta(&self) -> Pose {
        compose(
            self.config.core.frames(self.core_q)[5],
            inverse(translation(self.config.core.axes[5].origin_m)),
        )
    }
    fn head_delta(&self, lift: f64, p: f64, q: f64) -> Pose {
        let l = translation(self.config.core_lift_axis.map(|v| v * lift));
        compose(
            self.core_delta(),
            compose(
                delta(self.config.core_p_origin_m, self.config.core_p_axis, p),
                compose(
                    l,
                    delta(self.config.core_q_origin_m, self.config.core_q_axis, q),
                ),
            ),
        )
    }
    fn target(&self, logical: [f64; 3], angles: [f64; 3]) -> Pose {
        let insertion = super::ENTRANCE_X - logical[0];
        let center = compose(
            self.config.core.tool,
            Pose::at([0.075 - insertion, logical[1] - 0.2, 0.]),
        );
        let rotation = pose(
            bevy::math::DVec3::ZERO,
            DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2)
                * DQuat::from_rotation_z(GRASP_ROLL_RAD)
                * DQuat::from_rotation_x(angles[2]),
        );
        compose(
            self.head_delta(logical[2] - 0.6, angles[0], angles[1]),
            compose(center, rotation),
        )
    }
    pub fn update(&mut self, logical: [f64; 3], angles: [f64; 3]) -> Result<(), &'static str> {
        let target = self.target(logical, angles);
        let q = self.config.engineer.solve(
            compose(inverse(self.engineer_body), target),
            self.engineer_q,
        )?;
        let actual = compose(self.engineer_body, self.config.engineer.tool_pose(q));
        self.position_error_m = position(actual).distance(position(target));
        self.angle_error_rad = quaternion(actual).angle_between(quaternion(target));
        self.peak_joint_step_rad = self.peak_joint_step_rad.max(
            q.into_iter()
                .zip(self.engineer_q)
                .map(|(a, b)| (a - b).abs())
                .fold(0., f64::max),
        );
        self.engineer_q = q;
        self.tool = actual;
        if self.gripped {
            self.unit_pose = self.tool_unit_pose();
            self.lift_m = logical[2] - 0.6;
            self.p_rad = angles[0];
            self.q_rad = angles[1];
        }
        Ok(())
    }
    /// Absolute source link frames, shared by offline inspection and GPU presentation.
    pub fn world_frames(&self) -> BTreeMap<String, Pose> {
        let mut frames = self.frames();
        for (name, frame) in &mut frames {
            if name.starts_with("engineer/") {
                *frame = compose(self.engineer_body, *frame);
            }
        }
        frames.insert("unit/unit".into(), self.unit_pose);
        frames
    }
    pub fn frames(&self) -> BTreeMap<String, Pose> {
        let mut result = BTreeMap::new();
        result.insert("engineer/body".into(), Pose::default());
        result.insert("core/body".into(), Pose::default());
        for (prefix, arm, q) in [
            ("engineer", &self.config.engineer, self.engineer_q),
            ("core", &self.config.core, self.core_q),
        ] {
            for (i, p) in arm.frames(q).into_iter().enumerate() {
                result.insert(format!("{prefix}/j{}", i + 1), p);
            }
        }
        let d = compose(
            self.config.engineer.frames(self.engineer_q)[5],
            inverse(translation(self.config.engineer.axes[5].origin_m)),
        );
        for (jaw, angle) in self.config.engineer_jaws.iter().zip(&self.jaw_angles) {
            result.insert(
                format!("engineer/{}", jaw.name),
                compose(
                    d,
                    compose(
                        delta(jaw.origin_m, jaw.axis, *angle),
                        translation(jaw.origin_m),
                    ),
                ),
            );
        }
        let l = translation(self.config.core_lift_axis.map(|v| v * self.lift_m));
        result.insert(
            "core/lift".into(),
            compose(
                self.core_delta(),
                compose(l, translation(self.config.core_q_origin_m)),
            ),
        );
        result.insert(
            "core/p".into(),
            compose(
                self.head_delta(self.lift_m, self.p_rad, 0.),
                translation(self.config.core_p_origin_m),
            ),
        );
        result.insert(
            "core/q".into(),
            compose(
                self.head_delta(self.lift_m, self.p_rad, self.q_rad),
                translation(self.config.core_q_origin_m),
            ),
        );
        result
    }
}
#[derive(Component)]
pub(super) struct UnitRoot;
#[derive(Resource, Default)]
pub(super) struct BoundGroups(BTreeMap<String, Entity>);
impl BoundGroups {
    pub fn ready(&self) -> bool {
        let count = |name: &str| {
            self.0
                .keys()
                .filter(|key| key.split('@').next() == Some(name))
                .count()
        };
        [
            "engineer/body",
            "engineer/jaw-left",
            "engineer/jaw-right",
            "unit/unit",
        ]
        .iter()
        .all(|name| count(name) == 1)
            && (1..=6)
                .all(|i| count(&format!("engineer/j{i}")) == 1 && count(&format!("core/j{i}")) == 3)
            && ["core/body", "core/lift", "core/p", "core/q"]
                .iter()
                .all(|name| count(name) == 3)
    }
}
/// Register this asset source before DefaultPlugins.
pub fn register(app: &mut App, root: PathBuf) {
    use bevy::asset::io::{AssetSourceBuilder, file::FileAssetReader};
    app.register_asset_source(
        "assembly",
        AssetSourceBuilder::new(move || Box::new(FileAssetReader::new(root.clone()))),
    );
    app.init_resource::<BoundGroups>();
}
pub fn spawn(commands: &mut Commands, server: &AssetServer, rig: &Rig) {
    for (file, body) in [
        ("engineer.glb", rig.engineer_body),
        ("core-base.glb", Pose::default()),
        ("core-arm.glb", Pose::default()),
        ("core-head.glb", Pose::default()),
    ] {
        let mut t = Transform::default();
        apply_pose(
            &mut t,
            PoseFlu {
                translation_m: body.translation_m,
                rotation_wxyz: body.rotation_wxyz,
            },
        );
        t.rotation *= Quat::from_rotation_y(std::f32::consts::PI);
        commands.spawn((
            WorldAssetRoot(
                server.load(GltfAssetLabel::Scene(0).from_asset(format!("assembly://{file}"))),
            ),
            t,
        ));
    }
    commands.spawn((
        UnitRoot,
        WorldAssetRoot(server.load(GltfAssetLabel::Scene(0).from_asset("assembly://unit.glb"))),
        Transform::default(),
    ));
}
pub fn sync(
    w: Res<super::Workbench>,
    mut groups: ResMut<BoundGroups>,
    names: Query<(Entity, &Name), Added<Name>>,
    mut transforms: Query<&mut Transform>,
    unit: Query<Entity, With<UnitRoot>>,
) {
    let Some(rig) = &w.cad else {
        return;
    };
    // A frame may occur in several component GLBs, so bind by entity, not just name.
    for (entity, name) in &names {
        if let Some(key) = name.as_str().strip_prefix("link/") {
            groups
                .0
                .insert(format!("{}@{}", key, entity.to_bits()), entity);
        }
    }
    let frames = rig.frames();
    for (name, entity) in &groups.0 {
        let key = name.split('@').next().unwrap();
        let Some(p) = frames.get(key) else {
            continue;
        };
        if let Ok(mut t) = transforms.get_mut(*entity) {
            let next = native_transform(*p);
            if *t != next {
                *t = next;
            }
        }
    }
    for entity in &unit {
        if let Ok(mut t) = transforms.get_mut(entity) {
            let mut next = Transform::default();
            apply_pose(
                &mut next,
                PoseFlu {
                    translation_m: rig.unit_pose.translation_m,
                    rotation_wxyz: rig.unit_pose.rotation_wxyz,
                },
            );
            next.rotation *= Quat::from_rotation_y(std::f32::consts::PI);
            if *t != next {
                *t = next;
            }
        }
    }
}

fn native_transform(p: Pose) -> Transform {
    let [w, x, y, z] = p.rotation_wxyz;
    let q = Quat::from_xyzw(x as f32, y as f32, z as f32, w as f32);
    // Native glTF is left/up/forward; the scene root supplies its half-turn.
    let b = Quat::from_mat3(&Mat3::from_cols(Vec3::Z, Vec3::X, Vec3::Y));
    Transform::from_translation(Vec3::new(
        p.translation_m[1] as f32,
        p.translation_m[2] as f32,
        p.translation_m[0] as f32,
    ))
    .with_rotation(b * q * b.inverse())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn mechanism_fixture() -> Rig {
        use rm_simulator_physics::manipulator::RevoluteAxis;
        let arm = SerialArm {
            axes: [RevoluteAxis {
                origin_m: [0.; 3],
                axis: [0., 0., 1.],
                range_rad: [-3.2, 3.2],
            }; 6],
            tool: Pose::default(),
        };
        Rig {
            config: Config {
                schema: 4,
                assembly_frame: Pose::default(),
                core_envelope_m: vec![],
                engineer: arm.clone(),
                core: arm,
                engineer_jaws: vec![],
                core_p_origin_m: [0., 0., 0.1],
                core_p_axis: [0., -1., 0.],
                core_q_origin_m: [0.; 3],
                core_q_axis: [0., 0., 1.],
                core_lift_axis: [0., 0., 1.],
            },
            engineer_q: [0.; 6],
            core_q: [0.; 6],
            initial_engineer_q: [0.; 6],
            engineer_body: Pose::default(),
            tool: Pose::default(),
            socket: Pose::default(),
            requested_pose: None,
            position_error_m: 0.,
            angle_error_rad: 0.,
            peak_joint_step_rad: 0.,
            lift_m: 0.,
            p_rad: 0.,
            q_rad: 0.,
            jaw_angles: vec![],
            gripped: false,
            grip_requested: false,
            unit_pose: Pose::default(),
        }
    }
    #[test]
    fn flip_uses_fixed_hinge_and_q_tilts_the_raised_unit() {
        use std::f64::consts::{FRAC_PI_2, FRAC_PI_6};
        let rig = mechanism_fixture();
        let raised = rig.head_delta(0.1, FRAC_PI_2, 0.);
        // A carriage point lifted into P stays on that hinge during the flip.
        assert!(position(raised).distance(bevy::math::DVec3::new(0., 0., 0.1)) < 1e-12);
        let center = position(raised) + quaternion(raised) * bevy::math::DVec3::new(0.075, 0., 0.);
        assert!(center.distance(bevy::math::DVec3::new(0., 0., 0.175)) < 1e-12);
        let up = quaternion(raised) * bevy::math::DVec3::X;
        assert!(up.distance(bevy::math::DVec3::Z) < 1e-12);
        let tilted = rig.head_delta(0.1, FRAC_PI_2, FRAC_PI_6);
        let normal = quaternion(tilted) * bevy::math::DVec3::X;
        // Fig 5-15: Q changes the shaft normal; it does not roll around it.
        assert!(normal.distance(bevy::math::DVec3::new(0., 0.5, 3f64.sqrt() / 2.)) < 1e-12);
    }
    #[test]
    fn rulebook_side_view_matches_208_169_54_and_15_mm_dimensions() {
        let mut rig = mechanism_fixture();
        // 2026 EN V2.1.0 Fig 5-7, printed p82; same dimensioned drawing in
        // RMUC2027 Fig 5-7. Source panel face is x=0, initial post is z=0.
        rig.config.core_p_origin_m = [0.015, 0., 0.208];
        let lifted = rig.head_delta(0.1, 0., 0.);
        let flipped = rig.head_delta(0.1, std::f64::consts::FRAC_PI_2, 0.);
        let point =
            |d: Pose, p: [f64; 3]| position(d) + quaternion(d) * bevy::math::DVec3::from_array(p);
        assert!(
            point(lifted, [0.169, 0., 0.]).distance(bevy::math::DVec3::new(0.169, 0., 0.1)) < 1e-12
        );
        let tip = point(flipped, [0.169, 0., 0.]);
        assert!(tip.distance(bevy::math::DVec3::new(0.123, 0., 0.362)) < 1e-12);
        let front = point(flipped, [0.069, 0., 0.]); // 169 mm reach minus 100 mm insertion
        assert!((front.z - 0.208 - 0.054).abs() < 1e-12);
        let back = point(flipped, [0., 0., 0.]);
        assert!((0.208 - back.z - 0.015).abs() < 1e-12);
        assert!((front.x - 0.123).abs() < 1e-12);
    }
    #[test]
    fn jaw_actuation_requires_alignment_then_releases_and_regrips() {
        let mut rig = mechanism_fixture();
        rig.config.engineer_jaws = vec![
            Jaw {
                name: "jaw-left".into(),
                origin_m: [0.; 3],
                axis: [0., 0., 1.],
                open_rad: -0.5,
                closed_rad: -0.1,
            },
            Jaw {
                name: "jaw-right".into(),
                origin_m: [0.; 3],
                axis: [0., 0., 1.],
                open_rad: 0.6,
                closed_rad: 0.15,
            },
        ];
        rig.jaw_angles = vec![-0.5, 0.6];
        rig.unit_pose = rig.tool_unit_pose();
        rig.step_gripper(true, 0.01);
        assert!(!rig.gripped());
        assert!((rig.jaw_angles[0] + 0.485).abs() < 1e-12);
        rig.step_gripper(true, 1.);
        assert!(rig.gripped());
        assert_eq!(rig.jaw_angles, vec![-0.1, 0.15]);
        let held = rig.unit_pose;
        rig.step_gripper(false, 0.01);
        assert!(!rig.gripped());
        rig.tool.translation_m[0] += 0.02;
        rig.step_gripper(false, 1.);
        assert_eq!(rig.unit_pose, held); // released unit does not follow an empty gripper
        assert_eq!(rig.jaw_angles, vec![-0.5, 0.6]);
        rig.step_gripper(true, 1.);
        assert!(!rig.gripped()); // closing remotely cannot claim attachment
        rig.tool.translation_m[0] -= 0.02;
        rig.tool.rotation_wxyz = [0., 1., 0., 0.];
        rig.step_gripper(true, 1.);
        assert!(!rig.gripped()); // proximity alone does not align the shaft
        rig.tool.rotation_wxyz = [1., 0., 0., 0.];
        rig.step_gripper(true, 1.);
        assert!(rig.gripped());
        assert_eq!(rig.unit_pose, held);
        rig.restart();
        assert!(!rig.gripped());
        assert_eq!(rig.jaw_angles, vec![-0.5, 0.6]);
    }
    #[test]
    fn source_group_hierarchy_matches_independent_world_flu_coordinates() {
        let body = pose(
            bevy::math::DVec3::new(1.2, -0.7, 0.132),
            DQuat::from_euler(EulerRot::ZYX, 0.4, 0.2, -0.1),
        );
        let frame = pose(
            bevy::math::DVec3::new(0.3, 0.1, 0.7),
            DQuat::from_euler(EulerRot::ZYX, -0.7, 0.9, 0.6),
        );
        let mut parent = Transform::default();
        apply_pose(
            &mut parent,
            PoseFlu {
                translation_m: body.translation_m,
                rotation_wxyz: body.rotation_wxyz,
            },
        );
        parent.rotation *= Quat::from_rotation_y(std::f32::consts::PI);
        let child = native_transform(frame);
        for native in [Vec3::new(0.1, 0.2, 0.3), Vec3::new(-0.4, 0.3, -0.2)] {
            let world = parent.transform_point(child.transform_point(native));
            let local = bevy::math::DVec3::new(native.z as f64, native.x as f64, native.y as f64);
            let expected =
                position(body) + quaternion(body) * (position(frame) + quaternion(frame) * local);
            assert!(world.distance(rm_simulator_render::flu_position(expected.to_array())) < 1e-6);
        }
    }
}
