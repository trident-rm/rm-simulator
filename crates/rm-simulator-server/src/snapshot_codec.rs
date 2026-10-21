// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Binary message encodings. Every server message a peer receives and every
//! client message a host receives is a magic-tagged value in the positional
//! [`bitpack`] format; a snapshot travels as the compact independent player
//! checkpoint. No encoding here depends on an earlier transmitted frame; the
//! periodic delta lane in [`crate::udp_snapshot`] builds on
//! [`checkpoint_node`] and [`decode_checkpoint`].
use crate::binary_snapshot::{
    bitpack,
    bitpack::{Aligned, Node},
    fixed_point,
};
use crate::protocol::{ClientMessage, ServerMessage};
use crate::simulation::SimulationState;
use serde::{Deserialize, Serialize};
use std::io;

/// First four bytes of every encoded [`ServerMessage`].
pub const SERVER_MAGIC: &[u8; 4] = b"RMM1";
/// First four bytes of every encoded [`ClientMessage`].
pub const CLIENT_MAGIC: &[u8; 4] = b"RMQ1";

/// Projectile caliber as one bit on the compact checkpoint: `false` is
/// 17 mm, `true` is 42 mm (rulebook section 1.4 projectile sizes). A third
/// caliber would need a protocol bump; the bit order is fixed on the wire.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CaliberBit(bool);
impl Serialize for CaliberBit {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_bool(self.0)
    }
}
impl<'de> Deserialize<'de> for CaliberBit {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        Ok(Self(bool::deserialize(deserializer)?))
    }
}
impl From<rm_simulator_world::Caliber> for CaliberBit {
    fn from(caliber: rm_simulator_world::Caliber) -> Self {
        Self(matches!(caliber, rm_simulator_world::Caliber::Mm42))
    }
}
impl From<CaliberBit> for rm_simulator_world::Caliber {
    fn from(bit: CaliberBit) -> Self {
        if bit.0 { Self::Mm42 } else { Self::Mm17 }
    }
}
/// Independent UDP player checkpoint. Physics restore values remain f64;
/// ball position and velocity carry `f32` precision, well below 0.1 mm on this
/// field, unless a value does not survive `f32`. No earlier packet or
/// acknowledgement is needed to decode it.
///
/// Nothing a decoder can derive travels. The field clock is `tick` times
/// `tick_ns()`; the native presentation view is rebuilt from authority
/// in the restore (see [`DerivedRules`]); wheel hubs and tyre targets are
/// rebuilt from the pose, configuration and command (see [`ChassisMotion`]).
/// Every absolute timestamp rides as a [`StampCodes`] tick code, with any
/// sub-tick remainders listed once in `sub_tick_ns`, in visit order.
///
/// The fields are laid out by change rate. The slow section comes first, each
/// record [`Aligned`] to a byte boundary so an unchanged record has the same
/// bytes in every independent frame and the dictionary matches it; the fast
/// section (chassis motion and projectiles) follows as dense bits that no
/// dictionary would match anyway. Values fixed for a placement travel as a
/// preset index when they equal one ([`ConfigWire`], [`PolicyWire`]), so a
/// checkpoint stays decodable on its own.
///
/// Field names never reach the wire; they select the fixed-point rounding in
/// [`fixed_point::Fine`].
#[derive(Serialize, Deserialize)]
struct PlayerSnapshot {
    header: Aligned<Header>,
    shot_results: Aligned<Vec<Aligned<crate::protocol::ShotResult>>>,
    /// Registered hits only; hit times are stamp codes.
    hits: Aligned<Vec<Aligned<rm_simulator_world::ArmorHit>>>,
    bases: Aligned<Vec<Aligned<rm_simulator_world::BaseSnapshot>>>,
    rules: Aligned<DerivedRules>,
    chassis: Aligned<Vec<Aligned<ChassisRecord>>>,
    /// One per `chassis` record, in the same order.
    motion: Vec<ChassisMotion>,
    projectiles: Vec<ProjectileWire>,
}
/// The scalar part of [`SimulationState`] and its field, without the clock.
#[derive(Serialize, Deserialize)]
struct Header {
    snapshot_id: u64,
    input_epoch: u64,
    tick: u64,
    paused: bool,
    shots_fired: u64,
    hits_detected: u64,
    bots: Vec<u32>,
    navigation: Vec<crate::navigation::View>,
    sentry_tracking: Vec<crate::sentry_controller::View>,
    /// Nonzero remainders of the stamps whose code marks one, in the order
    /// [`PlayerSnapshot::for_each_field_stamp`] visits them.
    sub_tick_ns: Vec<u32>,
}
// Native authority is encoded once in DerivedRules. Historical presentation
// arrays must be empty; they cannot supply rule state or restore an old match.
// A trajectory is immutable bulk data plus a small changing cursor. Packing it
// separately keeps a complete restore within the pacer's 64 KiB world budget,
// including on the raw loopback carrier. Source calibration stays exact.
struct AssemblyWire(rm_simulator_world::assembly::Run);
#[derive(Serialize, Deserialize)]
struct AssemblyData {
    header: rm_simulator_world::assembly::Run,
    columns: [Vec<i32>; 21],
    flags: Vec<u8>,
    derived_units: bool,
}
fn carried_frame_pose(
    run: &rm_simulator_world::assembly::Run,
    joints: [f64; 6],
) -> Option<rm_simulator_world::Pose> {
    run.carried_unit_pose(joints)
        .map(|p| rm_simulator_world::Pose {
            translation_m: p.translation_m.map(|v| (v * 1e6).round() / 1e6),
            rotation_wxyz: p.rotation_wxyz.map(|v| (v * 1e7).round() / 1e7),
        })
}
impl AssemblyData {
    fn new(run: &rm_simulator_world::assembly::Run) -> Self {
        let mut held_pose = None;
        let derived_units = run.unit_grasp.is_some()
            && run.frames.iter().all(|f| {
                if f.gripped {
                    held_pose = carried_frame_pose(run, f.joints_rad);
                }
                held_pose == Some(f.unit)
            });
        let mut header = run.clone();
        let frames = std::mem::take(&mut header.frames);
        let mut columns: [Vec<i32>; 21] = std::array::from_fn(|_| Vec::with_capacity(frames.len()));
        let mut flags = Vec::with_capacity(frames.len());
        let mut previous = [0_i32; 21];
        for f in frames {
            for (j, (column, value)) in columns
                .iter_mut()
                .zip(
                    f.joints_rad
                        .into_iter()
                        .chain(f.jaws_rad)
                        .chain(f.unit.translation_m)
                        .chain(f.unit.rotation_wxyz)
                        .chain(f.guide_m)
                        .chain(f.angles_rad),
                )
                .enumerate()
            {
                if derived_units && (8..15).contains(&j) {
                    continue;
                }
                let scale = if (11..15).contains(&j) { 1e7 } else { 1e6 };
                let value = (value * scale).round() as i32;
                column.push(value - previous[j]);
                previous[j] = value;
            }
            flags.push(f.stage | (u8::from(f.gripped) << 3));
        }
        Self {
            header,
            columns,
            flags,
            derived_units,
        }
    }
    fn into_run(mut self) -> Result<rm_simulator_world::assembly::Run, &'static str> {
        if self.flags.len() > 4096
            || self.columns.iter().enumerate().any(|(i, v)| {
                v.len()
                    != if self.derived_units && (8..15).contains(&i) {
                        0
                    } else {
                        self.flags.len()
                    }
            })
            || self.derived_units
                && (self.header.unit_grasp.is_none()
                    || self.flags.first().is_some_and(|v| v & 8 == 0))
            || self.flags.iter().any(|v| *v > 15)
            || !self.header.frames.is_empty()
        {
            return Err("invalid packed assembly trajectory");
        }
        self.header.model.validate()?;
        let valid = |p: rm_simulator_world::Pose| {
            p.translation_m
                .into_iter()
                .chain(p.rotation_wxyz)
                .all(f64::is_finite)
                && (p.rotation_wxyz.into_iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-6
        };
        if !valid(self.header.pose.engineer_body)
            || self.header.unit_grasp.is_some_and(|g| !valid(g))
        {
            return Err("invalid carried Unit transform");
        }
        let mut previous = [0_i64; 21];
        let mut held_pose = None;
        self.header.frames = (0..self.flags.len())
            .map(|i| {
                let v: [f64; 21] = std::array::from_fn(|j| {
                    if self.derived_units && (8..15).contains(&j) {
                        return 0.;
                    }
                    previous[j] += i64::from(self.columns[j][i]);
                    previous[j] as f64 / if (11..15).contains(&j) { 1e7 } else { 1e6 }
                });
                rm_simulator_world::assembly::Frame {
                    joints_rad: std::array::from_fn(|j| v[j]),
                    jaws_rad: [v[6], v[7]],
                    unit: if self.derived_units {
                        if self.flags[i] & 8 != 0 {
                            held_pose =
                                carried_frame_pose(&self.header, std::array::from_fn(|j| v[j]));
                        }
                        held_pose.unwrap()
                    } else {
                        rm_simulator_world::Pose {
                            translation_m: [v[8], v[9], v[10]],
                            rotation_wxyz: [v[11], v[12], v[13], v[14]],
                        }
                    },
                    guide_m: [v[15], v[16], v[17]],
                    angles_rad: [v[18], v[19], v[20]],
                    gripped: self.flags[i] & 8 != 0,
                    stage: self.flags[i] & 7,
                }
            })
            .collect();
        self.header.validate()?;
        Ok(self.header)
    }
}
impl Serialize for AssemblyWire {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::Error;
        let packed = bitpack::to_bytes(&AssemblyData::new(&self.0)).map_err(S::Error::custom)?;
        let bytes = zstd::bulk::compress(&packed, 3).map_err(S::Error::custom)?;
        serializer.serialize_bytes(&bytes)
    }
}
impl<'de> Deserialize<'de> for AssemblyWire {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct Visitor;
        impl<'de> serde::de::Visitor<'de> for Visitor {
            type Value = AssemblyWire;
            fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                f.write_str("a bounded packed assembly trajectory")
            }
            fn visit_bytes<E: serde::de::Error>(self, bytes: &[u8]) -> Result<Self::Value, E> {
                let packed = zstd::bulk::decompress(bytes, crate::protocol::MAX_LINE_BYTES)
                    .map_err(E::custom)?;
                let data: AssemblyData = bitpack::from_bytes(&packed).map_err(E::custom)?;
                let run = data.into_run().map_err(E::custom)?;
                Ok(AssemblyWire(run))
            }
            fn visit_byte_buf<E: serde::de::Error>(self, bytes: Vec<u8>) -> Result<Self::Value, E> {
                self.visit_bytes(&bytes)
            }
        }
        deserializer.deserialize_bytes(Visitor)
    }
}
// Immutable payload hulls travel separately from changing poses, using exact
// delta-coded micrometre coordinates. Host proxy generation uses the same grid.
#[derive(Serialize, Deserialize)]
struct EnergyUnitWire {
    header: Aligned<rm_simulator_world::energy_unit::EnergyUnit>,
    geometry: UnitGeometryWire,
}
struct UnitGeometryWire(Vec<Vec<[f64; 3]>>);
#[derive(Serialize, Deserialize)]
struct UnitGeometryData {
    lengths: Vec<u16>,
    columns: [Vec<i32>; 3],
}
impl EnergyUnitWire {
    fn new(unit: &rm_simulator_world::energy_unit::EnergyUnit) -> Self {
        let mut header = unit.clone();
        let geometry = UnitGeometryWire(std::mem::take(&mut header.physical.shape.hulls_m));
        Self {
            header: Aligned(header),
            geometry,
        }
    }
    fn into_unit(mut self) -> rm_simulator_world::energy_unit::EnergyUnit {
        self.header.0.physical.shape.hulls_m = self.geometry.0;
        self.header.0
    }
}
impl Serialize for UnitGeometryWire {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::Error;
        let mut data = UnitGeometryData {
            lengths: Vec::new(),
            columns: std::array::from_fn(|_| Vec::new()),
        };
        if self.0.len() > 32 || self.0.iter().map(Vec::len).sum::<usize>() > 8192 {
            return Err(S::Error::custom("oversized Unit geometry"));
        }
        for hull in &self.0 {
            data.lengths.push(hull.len() as u16);
            let mut previous = [0i32; 3];
            for p in hull {
                for i in 0..3 {
                    let scaled = p[i] * 1e6;
                    if !scaled.is_finite()
                        || scaled < i32::MIN as f64
                        || scaled > i32::MAX as f64
                        || scaled.round() / 1e6 != p[i]
                    {
                        return Err(S::Error::custom(
                            "Unit geometry is not on its canonical grid",
                        ));
                    }
                    let value = scaled.round() as i32;
                    data.columns[i].push(
                        value
                            .checked_sub(previous[i])
                            .ok_or_else(|| S::Error::custom("Unit geometry delta overflow"))?,
                    );
                    previous[i] = value;
                }
            }
        }
        let packed = bitpack::to_bytes(&data).map_err(S::Error::custom)?;
        let bytes = zstd::bulk::compress(&packed, 3).map_err(S::Error::custom)?;
        serializer.serialize_bytes(&bytes)
    }
}
impl<'de> Deserialize<'de> for UnitGeometryWire {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct Visitor;
        impl<'de> serde::de::Visitor<'de> for Visitor {
            type Value = UnitGeometryWire;
            fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                f.write_str("bounded canonical Unit collision hulls")
            }
            fn visit_bytes<E: serde::de::Error>(self, bytes: &[u8]) -> Result<Self::Value, E> {
                let packed = zstd::bulk::decompress(bytes, 256 * 1024).map_err(E::custom)?;
                let data: UnitGeometryData = bitpack::from_bytes(&packed).map_err(E::custom)?;
                let count = data.lengths.iter().map(|v| usize::from(*v)).sum::<usize>();
                if data.lengths.is_empty()
                    || data.lengths.len() > 32
                    || count > 8192
                    || data.lengths.iter().any(|v| *v < 4)
                    || data.columns.iter().any(|v| v.len() != count)
                {
                    return Err(E::custom("invalid Unit hull columns"));
                }
                let mut offset = 0;
                let mut hulls = Vec::new();
                for length in data.lengths {
                    let mut previous = [0i64; 3];
                    let mut hull = Vec::new();
                    for j in offset..offset + usize::from(length) {
                        let mut p = [0.; 3];
                        for i in 0..3 {
                            previous[i] += i64::from(data.columns[i][j]);
                            if i32::try_from(previous[i]).is_err() {
                                return Err(E::custom("Unit vertex overflow"));
                            }
                            p[i] = previous[i] as f64 / 1e6;
                        }
                        hull.push(p);
                    }
                    offset += usize::from(length);
                    hulls.push(hull);
                }
                Ok(UnitGeometryWire(hulls))
            }
            fn visit_byte_buf<E: serde::de::Error>(self, bytes: Vec<u8>) -> Result<Self::Value, E> {
                self.visit_bytes(&bytes)
            }
        }
        deserializer.deserialize_bytes(Visitor)
    }
}
#[derive(Serialize, Deserialize)]
struct DerivedRules {
    step_duration: rm_simulator_world::StepDuration,
    assemblies: Vec<AssemblyWire>,
    arms: Vec<rm_simulator_world::assembly::ArmState>,
    energy_units: Vec<EnergyUnitWire>,
    rmuc_2027: Option<rm_simulator_world::rmuc_2027::State>,
    next_chassis_id: u32,
    last_detection_ns: Vec<(rm_simulator_world::ArmorTarget, u64)>,
    projectile_policy: PolicyWire,
}
impl DerivedRules {
    fn new(restore: &rm_simulator_world::FieldRestore) -> Self {
        Self {
            step_duration: restore.step_duration,
            arms: restore.arms.clone(),
            energy_units: restore
                .energy_units
                .iter()
                .map(EnergyUnitWire::new)
                .collect(),
            assemblies: restore
                .assemblies
                .iter()
                .cloned()
                .map(AssemblyWire)
                .collect(),
            rmuc_2027: restore.rmuc_2027.clone(),
            next_chassis_id: restore.next_chassis_id,
            last_detection_ns: restore.last_detection_ns.clone(),
            projectile_policy: PolicyWire::new(restore.projectile_policy),
        }
    }
    fn into_restore(self) -> rm_simulator_world::FieldRestore {
        rm_simulator_world::FieldRestore {
            step_duration: self.step_duration,
            assemblies: self.assemblies.into_iter().map(|a| a.0).collect(),
            arms: self.arms,
            energy_units: self
                .energy_units
                .into_iter()
                .map(EnergyUnitWire::into_unit)
                .collect(),
            rmuc_2027: self.rmuc_2027,
            next_chassis_id: self.next_chassis_id,
            last_detection_ns: self.last_detection_ns,
            projectile_policy: self.projectile_policy.into_policy(),
        }
    }
}
/// A projectile policy: the default in one gamma bit, anything else exactly.
#[derive(Serialize, Deserialize)]
enum PolicyWire {
    /// [`rm_simulator_world::projectile::ProjectilePolicy::default`].
    Default,
    /// Any other policy.
    Exact(rm_simulator_world::projectile::ProjectilePolicy),
}
impl PolicyWire {
    fn new(policy: rm_simulator_world::projectile::ProjectilePolicy) -> Self {
        if policy == rm_simulator_world::projectile::ProjectilePolicy::default() {
            Self::Default
        } else {
            Self::Exact(policy)
        }
    }
    fn into_policy(self) -> rm_simulator_world::projectile::ProjectilePolicy {
        match self {
            Self::Default => rm_simulator_world::projectile::ProjectilePolicy::default(),
            Self::Exact(policy) => policy,
        }
    }
}
/// A chassis configuration. The two presets in
/// `rm_simulator_physics::chassis` cost one and three bits instead of the
/// roughly 123 packed bytes of an exact configuration; a configuration equal
/// to neither travels exactly, so the checkpoint still needs no earlier frame.
#[derive(Serialize, Deserialize)]
enum ConfigWire {
    /// [`rm_simulator_world::ChassisConfig::default`], the omni Infantry.
    Infantry,
    /// [`rm_simulator_world::ChassisConfig::hero`], the mecanum Hero.
    Hero,
    /// Any other configuration.
    Exact(Box<rm_simulator_world::ChassisConfig>),
}
impl ConfigWire {
    fn new(config: &rm_simulator_world::ChassisConfig) -> Self {
        if *config == rm_simulator_world::ChassisConfig::default() {
            Self::Infantry
        } else if *config == rm_simulator_world::ChassisConfig::hero() {
            Self::Hero
        } else {
            Self::Exact(Box::new(config.clone()))
        }
    }
    fn into_config(self) -> rm_simulator_world::ChassisConfig {
        match self {
            Self::Infantry => rm_simulator_world::ChassisConfig::default(),
            Self::Hero => rm_simulator_world::ChassisConfig::hero(),
            Self::Exact(config) => *config,
        }
    }
}
/// The slowly changing part of a [`rm_simulator_world::ChassisSnapshot`]:
/// identity, configuration, command and whether it is defeated.
#[derive(Serialize, Deserialize)]
struct ChassisRecord {
    placement_revision: u64,
    id: u32,
    team: rm_simulator_world::Team,
    config: ConfigWire,
    command: rm_simulator_world::ChassisCommand,
    drive_control: rm_simulator_world::DriveControl,
    jump_held: bool,
    defeated: bool,
}
/// The dynamic part of a [`rm_simulator_world::ChassisSnapshot`], with each
/// wheel reduced to spin, axle speed and spring compression. Contacts are
/// diagnostics; hubs and tyre targets
/// are rebuilt with
/// [`rm_simulator_world::ChassisSnapshot::derive_wheel_kinematics`] after the
/// pose is normalized.
#[derive(Serialize, Deserialize)]
struct ChassisMotion {
    fold_angle_rad: f64,
    pose: rm_simulator_world::Pose,
    turret: rm_simulator_world::Pose,
    velocity_m_s: [f64; 3],
    angular_velocity_rad_s: [f64; 3],
    held_aim_rad: [f64; 2],
    gimbal_velocity_rad_s: [f64; 2],
    wheel_spin_rad: Vec<f64>,
    wheel_angular_velocity_rad_s: Vec<f64>,
    wheel_suspension_compression_m: Vec<f64>,
}
/// Stable identity and shooter, caliber, a position and velocity vector, then
/// the retirement bookkeeping. Spin is not carried: it moves no ball a client
/// can see. The launch, first-contact and dwell times are stamp codes, so a
/// predicting client retires the same ball on the same tick as the host does.
#[derive(Serialize, Deserialize)]
struct ProjectileWire {
    id: u64,
    caliber: CaliberBit,
    launched_ns: u64,
    position_m: [f64; 3],
    velocity_m_s: [f64; 3],
    shooter: Option<u32>,
    first_contact_ns: Option<u64>,
    dwell_since_ns: Option<u64>,
}

/// Rewrites absolute nanosecond timestamps as small tick codes and back.
///
/// Code 0 is the clock's current time, so a value that tracks the clock
/// (a rune's or the referee's own time, the economy's round time) costs one
/// byte and never changes between frames. Any other stamp `t` splits into
/// whole ticks `t / tick_ns()` and a remainder; the code is
/// `1 + 2 * ticks + (remainder != 0)`, and a nonzero remainder is appended to
/// the checkpoint's sub-tick list. Rule stamps land on the 128 Hz grid except
/// where the rules add a nanosecond (a hit window "exactly at 2.5 s is valid")
/// or a referee skip moves the round clock by an arbitrary amount, so the
/// remainder keeps every value exact. Whole ticks since the clock's zero are
/// used rather than ages against the checkpoint tick: an unchanged stamp then
/// has the same code in every frame and costs one bit in a delta.
struct StampCodes<'a> {
    now_ns: u64,
    sub_tick_ns: &'a mut Vec<u32>,
    /// Next remainder to read; decoding only.
    next: usize,
    error: bool,
}
impl<'a> StampCodes<'a> {
    fn new(now_ns: u64, sub_tick_ns: &'a mut Vec<u32>) -> Self {
        Self {
            now_ns,
            sub_tick_ns,
            next: 0,
            error: false,
        }
    }
    fn encode(&mut self, stamp: &mut u64) {
        let tick = rm_simulator_world::tick_ns();
        if *stamp == self.now_ns {
            *stamp = 0;
            return;
        }
        let remainder = *stamp % tick;
        if remainder != 0 {
            self.sub_tick_ns
                .push(u32::try_from(remainder).expect("a tick is shorter than 4 s"));
        }
        *stamp = 1 + 2 * (*stamp / tick) + u64::from(remainder != 0);
    }
    fn decode(&mut self, stamp: &mut u64) {
        if *stamp == 0 {
            *stamp = self.now_ns;
            return;
        }
        let code = *stamp - 1;
        let remainder = if code & 1 == 1 {
            let Some(&remainder) = self.sub_tick_ns.get(self.next) else {
                self.error = true;
                return;
            };
            self.next += 1;
            u64::from(remainder)
        } else {
            0
        };
        match (code >> 1)
            .checked_mul(rm_simulator_world::tick_ns())
            .and_then(|ns| ns.checked_add(remainder))
        {
            Some(ns) => *stamp = ns,
            None => self.error = true,
        }
    }
}

impl PlayerSnapshot {
    /// Visit every field-clock stamp in the checkpoint: shot results, hits,
    /// projectiles and physical detection times. Native authority keeps exact
    /// nanosecond durations in its own serialized state.
    /// The visit order is the wire order of `sub_tick_ns`.
    fn for_each_field_stamp(&mut self, visit: &mut dyn FnMut(&mut u64)) {
        for result in self.shot_results.iter_mut() {
            if let Some(time) = &mut result.executed_time_ns {
                visit(time);
            }
        }
        for hit in self.hits.iter_mut() {
            visit(&mut hit.time_ns);
        }
        for ball in &mut self.projectiles {
            visit(&mut ball.launched_ns);
            if let Some(time) = &mut ball.first_contact_ns {
                visit(time);
            }
            if let Some(time) = &mut ball.dwell_since_ns {
                visit(time);
            }
        }
        let restore = &mut *self.rules;
        for (_, time) in &mut restore.last_detection_ns {
            visit(time);
        }
    }

    /// The checkpoint for `state`. Refuses a state whose field has no restore,
    /// whose clock is off the tick grid, or whose rune, outpost or referee views
    /// differ from what the restore gives: the checkpoint carries only the
    /// restore, so such views could not be reproduced.
    fn from_state(state: &SimulationState) -> io::Result<Self> {
        let field = &state.field;
        let now_ns = field.tick.checked_mul(rm_simulator_world::tick_ns());
        let rules = match &field.restore {
            Some(restore)
                if restore.step_duration == rm_simulator_world::StepDuration::default()
                    && now_ns == Some(field.time_ns)
                    && field.runes.is_empty()
                    && field.outposts.is_empty()
                    && field.bases.is_empty()
                    && field.referee.is_none()
                    && field.rmuc_2027 == restore.rmuc_2027 =>
            {
                DerivedRules::new(restore)
            }
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "checkpoint state needs a restore that matches its views",
                ));
            }
        };
        let chassis = field
            .chassis
            .iter()
            .map(|chassis| {
                Aligned(ChassisRecord {
                    placement_revision: chassis.placement_revision,
                    id: chassis.id,
                    team: chassis.team,
                    config: ConfigWire::new(&chassis.config),
                    command: chassis.command,
                    drive_control: chassis.drive_control.clone(),
                    jump_held: chassis.jump_held,
                    defeated: chassis.defeated,
                })
            })
            .collect();
        let motion = field
            .chassis
            .iter()
            .map(|chassis| ChassisMotion {
                fold_angle_rad: chassis.fold_angle_rad,
                pose: chassis.pose,
                turret: chassis.turret,
                velocity_m_s: chassis.velocity_m_s,
                angular_velocity_rad_s: chassis.angular_velocity_rad_s,
                held_aim_rad: chassis.held_aim_rad,
                gimbal_velocity_rad_s: chassis.gimbal_velocity_rad_s,
                wheel_spin_rad: chassis.wheels.iter().map(|wheel| wheel.spin_rad).collect(),
                wheel_angular_velocity_rad_s: chassis
                    .wheels
                    .iter()
                    .map(|wheel| wheel.angular_velocity_rad_s)
                    .collect(),
                wheel_suspension_compression_m: chassis
                    .wheels
                    .iter()
                    .map(|wheel| wheel.suspension_compression_m)
                    .collect(),
            })
            .collect();
        let projectiles = field
            .projectiles
            .iter()
            .map(|ball| ProjectileWire {
                id: ball.id,
                caliber: CaliberBit::from(ball.caliber),
                launched_ns: ball.launched_ns,
                position_m: ball.position_m.map(single_precision),
                velocity_m_s: ball.velocity_m_s.map(single_precision),
                shooter: ball.shooter,
                first_contact_ns: ball.first_contact_ns,
                dwell_since_ns: ball.dwell_since_ns,
            })
            .collect();
        let mut snapshot = Self {
            header: Aligned(Header {
                snapshot_id: state.snapshot_id,
                input_epoch: state.input_epoch,
                tick: field.tick,
                paused: state.paused,
                shots_fired: field.shots_fired,
                hits_detected: field.hits_detected,
                bots: state.bots.clone(),
                navigation: state.navigation.clone(),
                sentry_tracking: state.sentry_tracking.clone(),
                sub_tick_ns: Vec::new(),
            }),
            shot_results: Aligned(state.shot_results.iter().cloned().map(Aligned).collect()),
            // Failed-contact diagnostics only fed the removed impact markers.
            // Keep genuine registered hits, damage and rune outcomes for armor
            // feedback.
            hits: Aligned(
                field
                    .hits
                    .iter()
                    .filter(|hit| hit.detected)
                    .cloned()
                    .map(Aligned)
                    .collect(),
            ),
            bases: Aligned(field.bases.iter().cloned().map(Aligned).collect()),
            rules: Aligned(rules),
            chassis: Aligned(chassis),
            motion,
            projectiles,
        };
        let mut sub_tick_ns = Vec::new();
        let mut codes = StampCodes::new(field.time_ns, &mut sub_tick_ns);
        snapshot.for_each_field_stamp(&mut |stamp| codes.encode(stamp));
        snapshot.header.sub_tick_ns = sub_tick_ns;
        Ok(snapshot)
    }

    /// The simulation state this checkpoint carries. With `normalize`,
    /// quantized chassis rotations are normalized first (see
    /// [`fixed_point::normalize_pose`]), so the rebuilt wheel hubs follow the pose
    /// physics will use.
    fn into_state(mut self, normalize: bool) -> io::Result<SimulationState> {
        let invalid = |message| io::Error::new(io::ErrorKind::InvalidData, message);
        if self.rules.step_duration != rm_simulator_world::StepDuration::default() {
            return Err(invalid(
                "network checkpoint requires the default 128 Hz field step",
            ));
        }
        let mut sub_tick_ns = std::mem::take(&mut self.header.sub_tick_ns);
        let time_ns = self
            .header
            .tick
            .checked_mul(rm_simulator_world::tick_ns())
            .ok_or_else(|| invalid("checkpoint tick overflows the clock"))?;
        let mut codes = StampCodes::new(time_ns, &mut sub_tick_ns);
        self.for_each_field_stamp(&mut |stamp| codes.decode(stamp));
        let (next, error) = (codes.next, codes.error);
        if error || next != sub_tick_ns.len() {
            return Err(invalid(
                "checkpoint timestamps do not match their remainders",
            ));
        }
        let PlayerSnapshot {
            header,
            shot_results,
            hits,
            bases,
            rules,
            chassis: records,
            motion,
            projectiles,
        } = self;
        let restore = rules.0.into_restore();
        let runes = Vec::new();
        let outposts = Vec::new();
        let referee = None;
        let restore = Some(restore);
        if records.len() != motion.len() {
            return Err(invalid(
                "checkpoint chassis motion does not match its records",
            ));
        }
        let mut chassis = Vec::with_capacity(records.len());
        for (Aligned(record), motion) in records.0.into_iter().zip(motion) {
            let config = record.config.into_config();
            if motion.wheel_spin_rad.len() != config.wheel_hubs_m.len()
                || motion.wheel_angular_velocity_rad_s.len() != config.wheel_hubs_m.len()
                || motion.wheel_suspension_compression_m.len() != config.wheel_hubs_m.len()
            {
                return Err(invalid("checkpoint wheel count does not match the chassis"));
            }
            record
                .drive_control
                .validate(config.wheel_hubs_m.len())
                .map_err(invalid)?;
            let mut snapshot = rm_simulator_world::ChassisSnapshot {
                placement_revision: record.placement_revision,
                id: record.id,
                team: record.team,
                config,
                fold_angle_rad: motion.fold_angle_rad,
                pose: motion.pose,
                turret: motion.turret,
                velocity_m_s: motion.velocity_m_s,
                angular_velocity_rad_s: motion.angular_velocity_rad_s,
                command: record.command,
                drive_control: record.drive_control,
                jump_held: record.jump_held,
                held_aim_rad: motion.held_aim_rad,
                gimbal_velocity_rad_s: motion.gimbal_velocity_rad_s,
                wheels: motion
                    .wheel_spin_rad
                    .into_iter()
                    .zip(motion.wheel_angular_velocity_rad_s)
                    .zip(motion.wheel_suspension_compression_m)
                    .map(
                        |((spin_rad, angular_velocity_rad_s), suspension_compression_m)| {
                            rm_simulator_world::WheelSnapshot {
                                spin_rad,
                                angular_velocity_rad_s,
                                suspension_compression_m,
                                ..Default::default()
                            }
                        },
                    )
                    .collect(),
                defeated: record.defeated,
            };
            if normalize {
                for pose in [&mut snapshot.pose, &mut snapshot.turret] {
                    fixed_point::normalize_pose(pose)?;
                }
            }
            snapshot.derive_wheel_kinematics();
            chassis.push(snapshot);
        }
        let projectiles = projectiles
            .into_iter()
            .map(|ball| rm_simulator_world::ProjectileSnapshot {
                id: ball.id,
                caliber: rm_simulator_world::Caliber::from(ball.caliber),
                launched_ns: ball.launched_ns,
                position_m: ball.position_m,
                velocity_m_s: ball.velocity_m_s,
                angular_velocity_rad_s: [0.; 3],
                shooter: ball.shooter,
                first_contact_ns: ball.first_contact_ns,
                dwell_since_ns: ball.dwell_since_ns,
            })
            .collect();
        let header = header.0;
        if header.sentry_tracking.len() > 32
            || header.sentry_tracking.iter().any(|v| {
                v.detections.len() > 32
                    || v.detections
                        .iter()
                        .any(|d| d.position_m.iter().any(|p| !p.is_finite()))
            })
        {
            return Err(invalid("invalid Sentry tracking diagnostics"));
        }
        if header.navigation.len() > 32 || header.navigation.iter().any(|view| !view.valid()) {
            return Err(invalid("invalid navigation diagnostics"));
        }
        Ok(SimulationState {
            bots: header.bots,
            navigation: header.navigation,
            sentry_tracking: header.sentry_tracking,
            snapshot_id: header.snapshot_id,
            input_epoch: header.input_epoch,
            shot_results: shot_results.0.into_iter().map(|result| result.0).collect(),
            paused: header.paused,
            field: rm_simulator_world::FieldSnapshot {
                bases: bases.0.into_iter().map(|base| base.0).collect(),
                tick: header.tick,
                time_ns,
                runes,
                outposts,
                projectiles,
                chassis,
                hits: hits.0.into_iter().map(|hit| hit.0).collect(),
                shots_fired: header.shots_fired,
                hits_detected: header.hits_detected,
                referee,
                rmuc_2027: restore.as_ref().and_then(|rules| rules.rmuc_2027.clone()),
                restore,
            },
        })
    }
}
/// `x` at `f32` precision when `f32` holds it finitely, otherwise exact, so an
/// extreme diagnostic coordinate is never turned into infinity.
fn single_precision(x: f64) -> f64 {
    let single = x as f32;
    if single.is_finite() {
        f64::from(single)
    } else {
        x
    }
}
/// Server message wire form; the snapshot arm carries its compact view.
#[derive(Serialize)]
enum WireRef<'a> {
    Snapshot(Box<PlayerSnapshot>),
    Message(&'a ServerMessage),
}
/// Decoded [`WireRef`]; boxing keeps the arms comparable in size.
#[derive(Deserialize)]
enum Wire {
    Snapshot(Box<PlayerSnapshot>),
    Message(ServerMessage),
}

/// Encodes a server message for a peer: a snapshot becomes an independent
/// compact checkpoint that needs no earlier frame, and every other message its
/// positional binary form, both behind [`SERVER_MAGIC`].
///
/// # Panics
///
/// On a snapshot [`checkpoint_node`] refuses (no restore, or views that differ
/// from it). Every host snapshot comes from `Field::snapshot`, which always
/// carries a matching restore, so this is a caller bug in the same class as a
/// message the positional format cannot hold.
///
/// ```
/// use rm_simulator_server::protocol::ServerMessage;
/// use rm_simulator_server::simulation::Simulation;
/// use rm_simulator_server::snapshot_codec::{decode_player_message, encode_player_message};
/// use rm_simulator_world::{Field, FieldConfig};
///
/// let simulation = Simulation::new(Field::new(&FieldConfig::default()).unwrap(), false);
/// let mut state = simulation.state();
/// state.snapshot_id = 9;
/// let tick = state.field.tick;
/// let message = ServerMessage::Snapshot(Box::new(state));
/// let compact = encode_player_message(&message);
/// let ServerMessage::Snapshot(restored) = decode_player_message(&compact).unwrap() else {
///     panic!("a snapshot must decode to a snapshot");
/// };
/// assert_eq!(restored.snapshot_id, 9);
/// assert_eq!(restored.field.tick, tick);
/// ```
pub fn encode_player_message(message: &ServerMessage) -> Vec<u8> {
    let wire = match message {
        ServerMessage::Snapshot(state) => WireRef::Snapshot(Box::new(
            PlayerSnapshot::from_state(state).expect("a snapshot carries its matching restore"),
        )),
        other => WireRef::Message(other),
    };
    let mut bytes = SERVER_MAGIC.to_vec();
    bytes.extend(bitpack::to_bytes(&wire).expect("protocol messages are positional"));
    bytes
}
/// Decodes [`encode_player_message`] output. Errors on malformed bytes rather
/// than returning a partial state, because a decoder that drops rule state
/// gives up restoring, not drawing.
pub fn decode_player_message(bytes: &[u8]) -> io::Result<ServerMessage> {
    let body = bytes
        .strip_prefix(SERVER_MAGIC)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "not a server message"))?;
    Ok(match bitpack::from_bytes::<Wire>(body)? {
        Wire::Snapshot(snapshot) => ServerMessage::Snapshot(Box::new(snapshot.into_state(false)?)),
        Wire::Message(message) => message,
    })
}

/// Encodes a client message behind [`CLIENT_MAGIC`].
///
/// ```
/// use rm_simulator_server::protocol::ClientMessage;
/// use rm_simulator_server::snapshot_codec::{decode_client_message, encode_client_message};
///
/// let ping = ClientMessage::Ping { nonce: 7 };
/// let bytes = encode_client_message(&ping);
/// assert_eq!(bytes.len(), 6);
/// assert_eq!(decode_client_message(&bytes).unwrap(), ping);
/// ```
pub fn encode_client_message(message: &ClientMessage) -> Vec<u8> {
    let mut bytes = CLIENT_MAGIC.to_vec();
    bytes.extend(bitpack::to_bytes(message).expect("protocol messages are positional"));
    bytes
}
/// Decodes [`encode_client_message`] output.
pub fn decode_client_message(bytes: &[u8]) -> io::Result<ClientMessage> {
    let body = bytes
        .strip_prefix(CLIENT_MAGIC)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "not a client message"))?;
    bitpack::from_bytes(body)
}

/// The fine fixed-point checkpoint tree for `state`: the tree the periodic
/// delta lane packs and pins as a baseline. Errors with
/// [`io::ErrorKind::InvalidInput`] when the field has no restore, its clock is
/// off the tick grid, contains historical presentation arrays, or its native
/// authority disagrees with the restore.
pub fn checkpoint_node(state: &SimulationState) -> io::Result<Node> {
    Ok(bitpack::to_node_with(
        &PlayerSnapshot::from_state(state)?,
        fixed_point::Fine::Root,
    )?)
}

// Dead reckoning of delta baselines (protocol 42).
//
// A delta checkpoint is differenced against its pinned baseline carried forward
// to the frame's tick, so a steadily moving chassis or a ball in flight costs
// the residual of a prediction instead of its whole displacement. Both ends
// compute the prediction from the baseline tree alone.

/// Nanoseconds per second, for integer dead reckoning on grid steps.
const NS_PER_S: i128 = 1_000_000_000;
/// The largest baseline age, in ticks either way, whose motion is predicted:
/// 32 s at 128 Hz. An older baseline is differenced as it is, which also
/// bounds the integer arithmetic below for a hostile lead.
const MAX_PREDICTED_TICKS: u64 = 1 << 12;
/// Rotation integration substeps at most; one per tick for a shorter lead.
const MAX_ROTATION_SUBSTEPS: u64 = 32;
/// Positions of the fast and tick fields in the packed [`PlayerSnapshot`]
/// tuple and its header, checked by `baseline_prediction_patches_the_typed_tree`.
const HEADER_FIELD: usize = 0;
const HEADER_TICK_FIELD: usize = 2;
const CHASSIS_FIELD: usize = 5;
const MOTION_FIELD: usize = 6;
const PROJECTILES_FIELD: usize = 7;

/// `n / d` rounded half away from zero, for `d > 0`.
fn div_round(n: i128, d: i128) -> i128 {
    if n >= 0 {
        (n + d / 2) / d
    } else {
        -((-n + d / 2) / d)
    }
}
/// The step count of `value` on a `scale` grid, when it lies exactly on it.
fn grid_steps(value: f64, scale: f64) -> Option<i128> {
    let q = (value * scale).round();
    (q.abs() < (1_u64 << 52) as f64 && (q / scale).to_bits() == value.to_bits())
        .then_some(q as i128)
}
/// `value` advanced by `rate` for `dt_ns`, in whole steps of `scale`, both
/// inputs on their grids. Off-grid inputs leave the value unchanged.
fn advance(value: &mut f64, scale: f64, rate: f64, rate_scale: f64, dt_ns: i128) {
    let (Some(steps), Some(rate)) = (grid_steps(*value, scale), grid_steps(rate, rate_scale))
    else {
        return;
    };
    let per_rate_step = (scale / rate_scale) as i128;
    *value = (steps + div_round(rate * per_rate_step * dt_ns, NS_PER_S)) as f64 / scale + 0.0;
}
fn quat_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}
fn quat_normalize(q: [f64; 4]) -> [f64; 4] {
    let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if norm.is_finite() && norm > 0. {
        q.map(|v| v / norm)
    } else {
        q
    }
}
/// The rotation by half-angle vector `half`, first order and normalized. Only
/// `+ - * / sqrt` are used, which IEEE 754 rounds the same on every platform;
/// `sin` and `cos` are not guaranteed to.
fn small_rotation(half: [f64; 3]) -> [f64; 4] {
    quat_normalize([1., half[0], half[1], half[2]])
}
fn wrap_angle(angle: f64) -> f64 {
    use std::f64::consts::{PI, TAU};
    (angle + PI).rem_euclid(TAU) - PI
}
/// A value rounded onto the 0.1 mrad angle grid, as the serializer would.
fn angle_grid(angle: f64) -> f64 {
    bitpack::Grid {
        scale: fixed_point::ANGLE_SCALE,
        width: 24,
        k: 0,
    }
    .round(angle)
}

/// Carries chassis motion `lead_ticks` forward. See [`predict_baseline`].
fn predict_motion(_records: &[Aligned<ChassisRecord>], motion: &mut [ChassisMotion], lead: i64) {
    use fixed_point::{ANGLE_SCALE, RATE_SCALE, TRANSLATION_SCALE, VELOCITY_SCALE};
    let dt_ns = i128::from(lead) * i128::from(rm_simulator_world::tick_ns());
    let dt_s = dt_ns as f64 / NS_PER_S as f64;
    let substeps = lead.unsigned_abs().min(MAX_ROTATION_SUBSTEPS);
    let h = dt_s / substeps as f64;
    for motion in motion {
        for axis in 0..3 {
            let v = motion.velocity_m_s[axis];
            for translation in [&mut motion.pose, &mut motion.turret].map(|p| &mut p.translation_m)
            {
                advance(
                    &mut translation[axis],
                    TRANSLATION_SCALE,
                    v,
                    VELOCITY_SCALE,
                    dt_ns,
                );
            }
        }
        let w = motion.angular_velocity_rad_s;
        if w != [0.; 3] && w.iter().all(|v| grid_steps(*v, RATE_SCALE).is_some()) {
            let step = small_rotation(w.map(|v| v * h * 0.5));
            for _ in 0..substeps {
                motion.pose.rotation_wxyz =
                    quat_normalize(quat_mul(step, motion.pose.rotation_wxyz));
            }
        }
        // The turret is world-stabilised: yaw turns it about world up, pitch
        // about its own left axis (negative pitch about +y, as the physics aim
        // rotation builds it).
        let [yaw_rate, pitch_rate] = motion.gimbal_velocity_rad_s;
        if grid_steps(yaw_rate, RATE_SCALE).is_some()
            && grid_steps(pitch_rate, RATE_SCALE).is_some()
        {
            if (yaw_rate, pitch_rate) != (0., 0.) {
                let yaw = small_rotation([0., 0., yaw_rate * h * 0.5]);
                let pitch = small_rotation([0., -pitch_rate * h * 0.5, 0.]);
                for _ in 0..substeps {
                    motion.turret.rotation_wxyz =
                        quat_normalize(quat_mul(quat_mul(yaw, motion.turret.rotation_wxyz), pitch));
                }
            }
            let [held_yaw, held_pitch] = &mut motion.held_aim_rad;
            if grid_steps(*held_yaw, ANGLE_SCALE).is_some() {
                advance(held_yaw, ANGLE_SCALE, yaw_rate, RATE_SCALE, dt_ns);
                *held_yaw = angle_grid(wrap_angle(*held_yaw));
            }
            advance(held_pitch, ANGLE_SCALE, pitch_rate, RATE_SCALE, dt_ns);
        }
        // Baseline predictor only: extrapolate measured axle speed. Full world
        // prediction uses the restored motor, inertia and suspension state.
        for (spin, rate) in motion
            .wheel_spin_rad
            .iter_mut()
            .zip(&motion.wheel_angular_velocity_rad_s)
        {
            let next = wrap_angle(*spin + rate * dt_s);
            if grid_steps(*spin, ANGLE_SCALE).is_some() && next.is_finite() {
                *spin = angle_grid(next);
            }
        }
    }
}

/// Carries projectiles `lead` ticks forward ballistically. See
/// [`predict_baseline`].
fn predict_projectiles(projectiles: &mut [ProjectileWire], lead: i64) {
    use fixed_point::PROJECTILE_SCALE;
    let dt_ns = i128::from(lead) * i128::from(rm_simulator_world::tick_ns());
    let g = (rm_simulator_world::projectile::GRAVITY_M_S2 * PROJECTILE_SCALE).round() as i128;
    for ball in projectiles {
        // A ball that has touched something is usually rolling or resting,
        // where free fall predicts worse than no acceleration at all.
        let g = if ball.first_contact_ns.is_none() {
            g
        } else {
            0
        };
        for axis in 0..3 {
            let (Some(p), Some(v)) = (
                grid_steps(ball.position_m[axis], PROJECTILE_SCALE),
                grid_steps(ball.velocity_m_s[axis], PROJECTILE_SCALE),
            ) else {
                continue;
            };
            let g = if axis == 2 { g } else { 0 };
            // p + v dt - g dt^2 / 2 and v - g dt, in whole steps.
            let p = p + div_round(
                2 * v * dt_ns * NS_PER_S - g * dt_ns * dt_ns,
                2 * NS_PER_S * NS_PER_S,
            );
            let v = v - div_round(g * dt_ns, NS_PER_S);
            ball.position_m[axis] = p as f64 / PROJECTILE_SCALE + 0.0;
            ball.velocity_m_s[axis] = v as f64 / PROJECTILE_SCALE + 0.0;
        }
    }
}

fn not_checkpoint() -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, "not a checkpoint tree")
}
/// The field at `index` of a checkpoint tree's top-level tuple.
fn field(node: &Node, index: usize) -> io::Result<&Node> {
    match node {
        Node::Tuple(fields) => fields.get(index).ok_or_else(not_checkpoint),
        _ => Err(not_checkpoint()),
    }
}
fn field_mut(node: &mut Node, index: usize) -> io::Result<&mut Node> {
    match node {
        Node::Tuple(fields) => fields.get_mut(index).ok_or_else(not_checkpoint),
        _ => Err(not_checkpoint()),
    }
}
/// The tick in a checkpoint tree's header.
fn tick(node: &Node) -> io::Result<u64> {
    let Node::Aligned(header) = field(node, HEADER_FIELD)? else {
        return Err(not_checkpoint());
    };
    match field(header, HEADER_TICK_FIELD)? {
        Node::Uint(tick) => Ok(*tick),
        _ => Err(not_checkpoint()),
    }
}
fn tick_mut(node: &mut Node) -> io::Result<&mut u64> {
    let Node::Aligned(header) = field_mut(node, HEADER_FIELD)? else {
        return Err(not_checkpoint());
    };
    match field_mut(header, HEADER_TICK_FIELD)? {
        Node::Uint(tick) => Ok(tick),
        _ => Err(not_checkpoint()),
    }
}

/// The baseline tree `baseline` dead-reckoned `lead` ticks forward (backward
/// when negative): the tree a delta checkpoint is differenced against.
///
/// Only the tick and the fast section change; the slow records are kept as
/// they are. The tick advances by `lead`. Each chassis and turret translation
/// advances by the body velocity; the body rotation integrates its angular
/// velocity; the turret rotation and held aim integrate the gimbal rates; each
/// wheel spins at its measured axle speed. A projectile moves at its
/// velocity, falling under [`rm_simulator_world::projectile::GRAVITY_M_S2`]
/// until its first contact. Velocities, drag, contacts, spawns and retirement
/// are not predicted: this is only a predictor, and a wrong guess costs bits,
/// never correctness.
///
/// Translations, velocities, aims and projectiles use integer arithmetic on
/// grid steps, rotations and wheel spin only IEEE 754 basic operations, and
/// every result is re-quantized onto its grid, so an encoder and a decoder
/// holding the same tree get the same prediction bit for bit on every
/// platform. A value off its grid is not predicted. Past
/// `MAX_PREDICTED_TICKS` only the tick moves. Errors when the tree is not a
/// checkpoint.
///
/// ```
/// use rm_simulator_server::simulation::Simulation;
/// use rm_simulator_server::snapshot_codec::{checkpoint_node, predict_baseline};
/// use rm_simulator_world::{Field, FieldConfig};
///
/// let simulation = Simulation::new(Field::new(&FieldConfig::default()).unwrap(), false);
/// let baseline = checkpoint_node(&simulation.state()).unwrap();
/// // No lead predicts nothing; a lead moves at least the tick.
/// assert_eq!(predict_baseline(&baseline, 0).unwrap(), baseline);
/// assert_ne!(predict_baseline(&baseline, 4).unwrap(), baseline);
/// ```
pub fn predict_baseline(baseline: &Node, lead: i64) -> io::Result<Node> {
    let mut prediction = Prediction::default();
    prediction.predict(baseline, (0, 0), lead)?;
    Ok(prediction.tree)
}

/// A reusable dead-reckoned copy of one pinned baseline. The slow records of a
/// baseline never change, so they are cloned once per baseline and only the
/// tick, chassis motion and projectiles are rewritten for each frame; that
/// halves the prediction cost at twelve chassis.
#[derive(Default)]
pub struct Prediction {
    /// Epoch and id of the baseline `tree` was cloned from.
    key: Option<(u64, u64)>,
    tree: Node,
}
impl Prediction {
    /// `baseline`, pinned as `key` (epoch and id), dead-reckoned `lead` ticks;
    /// see [`predict_baseline`]. A key is never reused for a different
    /// baseline, which the delta lane guarantees.
    fn predict(&mut self, baseline: &Node, key: (u64, u64), lead: i64) -> io::Result<&Node> {
        if self.key != Some(key) {
            self.key = None;
            self.tree = baseline.clone();
            self.key = Some(key);
        }
        let result = self.rewrite(baseline, lead);
        if result.is_err() {
            self.key = None;
        }
        result?;
        Ok(&self.tree)
    }
    fn rewrite(&mut self, baseline: &Node, lead: i64) -> io::Result<()> {
        let base_tick = tick(baseline)?;
        *tick_mut(&mut self.tree)? = base_tick.checked_add_signed(lead).unwrap_or(base_tick);
        let predicted = lead != 0 && lead.unsigned_abs() <= MAX_PREDICTED_TICKS;
        let motion = field(baseline, MOTION_FIELD)?;
        *field_mut(&mut self.tree, MOTION_FIELD)? = if predicted && !is_empty_seq(motion) {
            let Node::Aligned(records) = field(baseline, CHASSIS_FIELD)? else {
                return Err(not_checkpoint());
            };
            let records: Vec<Aligned<ChassisRecord>> = bitpack::from_node(records)?;
            let mut typed: Vec<ChassisMotion> = bitpack::from_node(motion)?;
            predict_motion(&records, &mut typed, lead);
            bitpack::to_node_with(&typed, fixed_point::Fine::Chassis)?
        } else {
            motion.clone()
        };
        let projectiles = field(baseline, PROJECTILES_FIELD)?;
        *field_mut(&mut self.tree, PROJECTILES_FIELD)? = if predicted && !is_empty_seq(projectiles)
        {
            let mut typed: Vec<ProjectileWire> = bitpack::from_node(projectiles)?;
            predict_projectiles(&mut typed, lead);
            bitpack::to_node_with(&typed, fixed_point::Fine::Projectiles)?
        } else {
            projectiles.clone()
        };
        Ok(())
    }
}
fn is_empty_seq(node: &Node) -> bool {
    matches!(node, Node::Seq(items) if items.is_empty())
}

fn zigzag(n: i64) -> u64 {
    ((n << 1) ^ (n >> 63)) as u64
}
fn unzigzag(n: u64) -> i64 {
    ((n >> 1) as i64) ^ -((n & 1) as i64)
}

/// Packs checkpoint tree `node`, whose tick is `tick`, for the periodic lane:
/// independent without a baseline, otherwise a delta against the pinned
/// `baseline` (with its epoch-unique id) dead-reckoned to `tick` by
/// [`predict_baseline`], reusing `prediction` across frames. The tick lead
/// travels zigzag-coded as the frame's [`bitpack::encode_hinted`] hint, so
/// [`decode_checkpoint`] rebuilds the same prediction before it reads the body.
pub fn encode_checkpoint(
    node: &Node,
    tick: u64,
    baseline: Option<(&Node, u64)>,
    epoch: u64,
    prediction: &mut Prediction,
) -> io::Result<Vec<u8>> {
    let Some((base, id)) = baseline else {
        return bitpack::encode(node, None, epoch, 0);
    };
    let lead = tick.wrapping_sub(self::tick(base)?) as i64;
    let predicted = prediction.predict(base, (epoch, id), lead)?;
    bitpack::encode_hinted(node, Some(predicted), epoch, id, zigzag(lead))
}

/// Decodes one packed checkpoint frame against its pinned baseline, checks its
/// input epoch and normalizes its quantized rotations. With `keep_node` the
/// decoded wire-grid tree comes back too, for pinning as a baseline; the
/// delivered state is normalized, the tree is not, so later deltas use the
/// exact values the encoder holds.
pub fn decode_checkpoint(
    bytes: &[u8],
    baseline: Option<(&Node, u64)>,
    epoch: u64,
    keep_node: bool,
) -> io::Result<(ServerMessage, Option<Node>)> {
    decode_checkpoint_with(
        bytes,
        baseline,
        epoch,
        keep_node,
        &mut Prediction::default(),
    )
}

/// [`decode_checkpoint`] reusing `prediction` for a delta's dead-reckoned
/// baseline, as a decoder that receives many deltas against one pinned
/// baseline should.
pub fn decode_checkpoint_with(
    bytes: &[u8],
    baseline: Option<(&Node, u64)>,
    epoch: u64,
    keep_node: bool,
    prediction: &mut Prediction,
) -> io::Result<(ServerMessage, Option<Node>)> {
    let predicted = match baseline {
        Some((base, id)) if bitpack::header(bytes)?.0 => {
            let lead = unzigzag(bitpack::hint(bytes)?);
            Some((prediction.predict(base, (epoch, id), lead)?, id))
        }
        _ => None,
    };
    let (snapshot, _) = bitpack::decode_with::<PlayerSnapshot, _>(
        bytes,
        predicted,
        epoch,
        fixed_point::Fine::Root,
    )?;
    let node = keep_node
        .then(|| bitpack::to_node_with(&snapshot, fixed_point::Fine::Root))
        .transpose()?;
    if snapshot.header.input_epoch != epoch {
        return Err(io::Error::other("invalid baseline state/epoch"));
    }
    let message = ServerMessage::Snapshot(Box::new(snapshot.into_state(true)?));
    Ok((message, node))
}

#[cfg(test)]
mod tests {
    #[test]
    fn duration_authority_is_explicit_on_wire_and_network_rejects_custom_rates() {
        use rm_simulator_world::{Field, FieldConfig, StepDuration};
        for invalid in [0, 7_812_501, u64::MAX] {
            assert!(StepDuration::from_ns(invalid).is_err());
            assert!(serde_json::from_str::<StepDuration>(&invalid.to_string()).is_err());
        }
        for valid in [1, 999_999, 1_000_000, 7_812_500] {
            let duration = StepDuration::from_ns(valid).unwrap();
            assert_eq!(
                serde_json::from_str::<StepDuration>(&valid.to_string()).unwrap(),
                duration
            );
        }
        let field = Field::new(&FieldConfig::default()).unwrap();
        let rules = field.snapshot().restore.unwrap();
        let bytes = bitpack::to_bytes(&DerivedRules::new(&rules)).unwrap();
        let decoded: DerivedRules = bitpack::from_bytes(&bytes).unwrap();
        assert_eq!(decoded.into_restore(), rules);
        let mut custom_rules = rules;
        custom_rules.step_duration = StepDuration::HZ_1000;
        let bytes = bitpack::to_bytes(&DerivedRules::new(&custom_rules)).unwrap();
        let decoded: DerivedRules = bitpack::from_bytes(&bytes).unwrap();
        assert_eq!(decoded.into_restore(), custom_rules);
        let mut state = crate::simulation::Simulation::new(field, true).state();
        state.field.restore.as_mut().unwrap().step_duration = StepDuration::HZ_1000;
        assert!(PlayerSnapshot::from_state(&state).is_err());
        state.field.restore.as_mut().unwrap().step_duration = StepDuration::default();
        let mut encoded = PlayerSnapshot::from_state(&state).unwrap();
        encoded.rules.step_duration = StepDuration::HZ_1000;
        assert!(encoded.into_state(true).is_err());
        let custom = Field::new(&FieldConfig {
            step_duration: StepDuration::HZ_1000,
            ..Default::default()
        })
        .unwrap();
        assert!(crate::simulation::Simulation::try_new(custom, true).is_err());
    }
    #[test]
    fn direct_wheel_torques_survive_player_checkpoint_and_restore() {
        use rm_simulator_world::{DriveControl, WheelTorques};
        let (mut simulation, chassis) = crate::workload::simulation(1);
        let torques = WheelTorques::new(vec![0.123456789012345, -0.7, 0., 1.]).unwrap();
        simulation
            .test_field_mut()
            .command_chassis_wheel_torques(chassis[0], torques.clone())
            .unwrap();
        let mut state = self::simulation();
        state.snapshot_id = 1;
        state.field = simulation.field().snapshot();
        let message = ServerMessage::Snapshot(Box::new(state));
        let ServerMessage::Snapshot(decoded) =
            decode_player_message(&encode_player_message(&message)).unwrap()
        else {
            panic!("expected checkpoint");
        };
        assert_eq!(
            decoded.field.chassis[0].drive_control,
            DriveControl::WheelTorques(torques)
        );
        let restored = rm_simulator_world::Field::restore(
            &decoded.field,
            &simulation.field().static_geometry_snapshot(),
            simulation.field().floor_height_m(),
        )
        .unwrap();
        assert_eq!(
            restored.snapshot().chassis[0].drive_control,
            decoded.field.chassis[0].drive_control
        );
    }

    #[test]
    fn canonical_unit_geometry_roundtrips_and_does_not_change_with_motion() {
        use rm_simulator_world::{
            Pose,
            energy_unit::{Attachment, EnergyUnit},
            projectile::{PayloadShape, PayloadSnapshot},
        };
        let shape = PayloadShape::from_parts(
            &[
                rm_simulator_physics::manipulator_collision::CollisionPart::Box {
                    half_m: [0.03; 3],
                },
            ],
            0.25,
        )
        .unwrap();
        let unit = EnergyUnit {
            physical: PayloadSnapshot {
                id: 3,
                shape,
                pose: Pose::at([1., 2., 0.5]),
                velocity_m_s: [0.; 3],
                angular_velocity_rad_s: [0.; 3],
                prescribed: true,
            },
            attachment: Attachment::Held(3),
            spent: false,
            grasp: Pose::default(),
            seated_generation: 0,
        };
        let wire = EnergyUnitWire::new(&unit);
        let bytes = bitpack::to_bytes(&wire).unwrap();
        let decoded: EnergyUnitWire = bitpack::from_bytes(&bytes).unwrap();
        assert_eq!(decoded.into_unit(), unit);
        let mut moved = unit.clone();
        moved.physical.pose.translation_m[0] += 0.5;
        let moved = EnergyUnitWire::new(&moved);
        assert_eq!(
            bitpack::to_bytes(&wire.geometry).unwrap(),
            bitpack::to_bytes(&moved.geometry).unwrap()
        );
        let mut invalid = unit;
        invalid.physical.shape.hulls_m[0][0][0] += 1e-7;
        assert!(bitpack::to_bytes(&EnergyUnitWire::new(&invalid)).is_err());
    }
    #[test]
    fn live_2027_checkpoint_keeps_capture_expiry_ownership_and_source_bindings() {
        use rm_simulator_world::{
            FieldConfig, Pose, RefereeCommand,
            rmuc_2027::{Config, Detector, Tower},
        };
        let d = Detector {
            pose: Pose::at([3., 0., 1.]),
            half_m: [0.045, 0.044],
        };
        let mut field = Field::new(&FieldConfig {
            rmuc_2027: Some(Config {
                bases: [vec![d.clone()], vec![d.clone()]],
                outposts: [
                    Tower {
                        pivot_m: [3., 0., 1.],
                        axis: [0., 0., 1.],
                        detectors: vec![d.clone(); 3],
                    },
                    Tower {
                        pivot_m: [-3., 0., 1.],
                        axis: [0., 0., 1.],
                        detectors: vec![d; 3],
                    },
                ],
                zones: vec![],
                supply_zones: vec![],
                vantage_zones: vec![],
                terrain_zones: vec![],
                runes: vec![],
                rotor_direction: -1,
            }),
            runes: vec![],
            outposts: vec![],
            ..Default::default()
        })
        .unwrap();
        let robot = field
            .add_chassis(&rm_simulator_world::ChassisPlacement {
                config: Default::default(),
                spawn: Pose::at([0., 0., 0.16]),
                team: rm_simulator_world::Team::Blue,
                kind: rm_simulator_world::RobotKind::Infantry,
                performance: None,
            })
            .unwrap();
        field.referee_command(RefereeCommand::StartMatch).unwrap();
        field
            .referee_command(RefereeCommand::SetBaseHp {
                team: rm_simulator_world::Team::Blue,
                hp: 4000,
                shield_hp: 0,
            })
            .unwrap();
        field
            .referee_command(RefereeCommand::SetBaseHp {
                team: rm_simulator_world::Team::Blue,
                hp: 10000,
                shield_hp: 0,
            })
            .unwrap();
        field
            .step(6_000_000_000 / rm_simulator_world::tick_ns())
            .unwrap();
        field
            .referee_command(RefereeCommand::StartAssembly {
                team: rm_simulator_world::Team::Red,
                difficulty: 1,
            })
            .unwrap();
        field
            .referee_command(RefereeCommand::FinishAssembly {
                team: rm_simulator_world::Team::Red,
            })
            .unwrap();
        field
            .referee_command(RefereeCommand::StartAssembly {
                team: rm_simulator_world::Team::Blue,
                difficulty: 1,
            })
            .unwrap();
        field
            .buy_ammo(robot, rm_simulator_world::Caliber::Mm17)
            .unwrap();
        field
            .referee_command(RefereeCommand::SetAllowance {
                robot,
                allowance: [20, 0],
            })
            .unwrap();
        field
            .fire(
                Pose::yawed([3.15, 0., 1.], std::f64::consts::PI),
                rm_simulator_world::Shot::at_limit(rm_simulator_world::Caliber::Mm17),
                Some(robot),
            )
            .unwrap();
        // Retire the structure-contact flight before replay, whose solver and
        // f32 trajectory need not reproduce micrometre hit positions. Keep
        // pending logistics, and a new unobstructed flight to carry live heat.
        field
            .step(4_000_000_000 / rm_simulator_world::tick_ns())
            .unwrap();
        field
            .fire(
                Pose::at([0., 0., 2.]),
                rm_simulator_world::Shot::at_limit(rm_simulator_world::Caliber::Mm17),
                Some(robot),
            )
            .unwrap();
        field
            .referee_command(RefereeCommand::SetOutpostHp { outpost: 0, hp: 0 })
            .unwrap();
        assert_rules_round_trip(&mut field, 700);
        let mut state = SimulationState {
            bots: vec![],
            navigation: Vec::new(),
            sentry_tracking: Vec::new(),
            input_epoch: 0,
            shot_results: vec![],
            snapshot_id: 0,
            paused: false,
            field: field.snapshot(),
        };
        // Frozen selections and historic classifications travel through the same
        // complete checkpoint as active assembly and source-bound mechanism state.
        {
            use rm_simulator_world::gameplay::{
                Target, Team,
                rmuc_2027::adaptive::{Impact, Region},
            };
            let game = &mut state.field.rmuc_2027.as_mut().unwrap().game;
            game.advance_ns(60_000_000_000);
            game.adaptive.launches = [20; 2];
            for team in Team::BOTH {
                for i in 0..10 {
                    game.adaptive.impacts.push(Impact {
                        time_ns: game.elapsed_ns(),
                        attacker: Some(team),
                        victim: team.other(),
                        target: Target::Base(team.other()),
                        plate: 0,
                        position_um: [10_000 + i % 2 * 20_000, 20_000],
                        size_um: [90_000, 88_000],
                        region: Region::Default,
                    });
                }
                game.select_reinforcement(team, [45_000, 44_000]).unwrap();
            }
            game.validate().unwrap();
        }
        state.field.restore.as_mut().unwrap().rmuc_2027 = state.field.rmuc_2027.clone();
        let message = ServerMessage::Snapshot(Box::new(state.clone()));
        let decoded = decode_player_message(&encode_player_message(&message)).unwrap();
        let ServerMessage::Snapshot(decoded) = decoded else {
            panic!("expected snapshot");
        };
        assert_eq!(decoded.field.rmuc_2027, state.field.rmuc_2027);
        assert_eq!(decoded.field.restore, state.field.restore);
        assert!(checkpoint_node(&state).is_ok());
        state.field.rmuc_2027 = None;
        assert!(checkpoint_node(&state).is_err());
        let simulation = crate::simulation::Simulation::new(field, true);
        assert_eq!(simulation.weapon_for(robot).shot.speed_m_s, 25.);
        assert_eq!(simulation.weapon_limits_for(robot).max_speed_m_s, 25.);
    }

    use super::*;
    use rm_simulator_world::{Field, FieldConfig};
    fn simulation() -> SimulationState {
        SimulationState {
            bots: vec![],
            navigation: Vec::new(),
            sentry_tracking: Vec::new(),
            input_epoch: 0,
            shot_results: Vec::new(),
            snapshot_id: 0,
            paused: false,
            field: Field::new(&FieldConfig::default()).unwrap().snapshot(),
        }
    }
    fn equivalent(a: &serde_json::Value, b: &serde_json::Value) {
        use serde_json::Value;
        match (a, b) {
            (Value::Number(x), Value::Number(y)) if x.is_f64() || y.is_f64() => {
                let (x, y) = (x.as_f64().unwrap(), y.as_f64().unwrap());
                assert!((x - y).abs() < 1e-10 * x.abs().max(1.), "{x} != {y}");
            }
            (Value::Object(x), Value::Object(y)) => {
                assert!(x.keys().eq(y.keys()));
                for (key, value) in x {
                    equivalent(value, &y[key]);
                }
            }
            (Value::Array(x), Value::Array(y)) => {
                assert_eq!(x.len(), y.len());
                for (a, b) in x.iter().zip(y) {
                    equivalent(a, b);
                }
            }
            _ => assert_eq!(a, b),
        }
    }
    /// Steps `field` through 40 published frames, checking each checkpoint
    /// decodes exactly through the independent codec and through the
    /// acknowledged delta lane, then restores the last decoded checkpoint and
    /// replays `replay_ticks` against the host.
    fn assert_rules_round_trip(field: &mut Field, replay_ticks: u64) {
        use crate::udp_snapshot::{Decoder, Encoder, parse};
        let geometry = field.static_geometry_snapshot();
        let mut encoder = Encoder::default();
        let mut decoder = Decoder::default();
        let mut last = None;
        for frame in 0..40 {
            field.step(1).unwrap();
            let mut state = simulation();
            state.snapshot_id = frame + 1;
            state.field = field.snapshot();
            assert!(
                PlayerSnapshot::from_state(&state).is_ok(),
                "a field's own snapshot must travel as its restore alone"
            );
            let message = ServerMessage::Snapshot(Box::new(state.clone()));
            let decoded = decode_player_message(&encode_player_message(&message)).unwrap();
            if state.field.rmuc_2027.is_some()
                && let ServerMessage::Snapshot(decoded) = &decoded
            {
                // Hot body/projectile/hit views deliberately use f32 on this
                // lane (covered by the precision tests below). Rules, including
                // all µm adaptive coordinates, must still be bit-exact.
                assert_eq!(decoded.field.restore, state.field.restore);
                assert_eq!(decoded.field.rmuc_2027, state.field.rmuc_2027);
                state.field.chassis = decoded.field.chassis.clone();
                state.field.projectiles = decoded.field.projectiles.clone();
                state.field.hits = decoded.field.hits.clone();
            }
            assert_eq!(decoded, ServerMessage::Snapshot(Box::new(state.clone())));
            let bytes = encoder.snapshot(0, &state).unwrap();
            let wire = parse(&crate::compression::decompress(&bytes, 4 << 20).unwrap())
                .unwrap()
                .unwrap();
            let (received, feedback) = decoder.receive(wire).unwrap();
            let Some(ServerMessage::Snapshot(received)) = received else {
                panic!("every frame must deliver a snapshot");
            };
            if state.field.rmuc_2027.is_some() {
                let expected: PlayerSnapshot =
                    bitpack::from_node(&checkpoint_node(&state).unwrap()).unwrap();
                assert_eq!(received.field, expected.into_state(true).unwrap().field);
                assert_eq!(received.field.restore, state.field.restore);
                assert_eq!(received.field.rmuc_2027, state.field.rmuc_2027);
            } else {
                assert_eq!(received.field, state.field);
            }
            if let Some(feedback) = feedback
                && let Some(retire) = encoder.feedback(feedback)
                && let (_, Some(retired)) = decoder.receive(retire).unwrap()
            {
                encoder.feedback(retired);
            }
            last = Some(received.field);
        }
        assert!(
            encoder.deltas > 0,
            "the delta lane must have been exercised"
        );
        let mut restored = Field::restore(&last.unwrap(), &geometry, 0.).unwrap();
        field.step(replay_ticks).unwrap();
        restored.step(replay_ticks).unwrap();
        let (host, replay) = (field.snapshot(), restored.snapshot());
        assert_eq!(replay.runes, host.runes);
        assert_eq!(replay.referee, host.referee);
        assert_eq!(replay.restore, host.restore);
    }

    /// A small deterministic stream for the property tests.
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> u64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            self.0 >> 11
        }
        fn below(&mut self, n: u64) -> u64 {
            self.next() % n
        }
        /// A grid value: usually small, sometimes at the edge of `width` bits,
        /// sometimes off the grid or past its range so it escapes exactly.
        fn value(&mut self, scale: f64, width: u32) -> f64 {
            let edge = (1_i64 << (width - 1)) as f64;
            match self.below(8) {
                0 => (edge - 1. - self.below(3) as f64) / scale,
                1 => -(edge - self.below(3) as f64) / scale,
                2 => (edge + self.below(4) as f64) / scale,
                3 => 0.123_456_789_123 + self.below(100) as f64,
                _ => (self.below(20_000) as f64 - 10_000.) / scale,
            }
        }
    }

    /// A checkpoint with `chassis` moving robots and `balls` projectiles, their
    /// fast values randomized, as the typed snapshot the serializer packs.
    fn random_snapshot(rng: &mut Lcg, chassis: usize, balls: usize) -> PlayerSnapshot {
        use crate::binary_snapshot::fixed_point::{
            ANGLE_SCALE, PROJECTILE_SCALE, RATE_SCALE, TRANSLATION_SCALE, VELOCITY_SCALE,
        };
        let (mut simulation, _) = crate::workload::simulation(chassis);
        simulation.step(3).unwrap();
        let mut snapshot = PlayerSnapshot::from_state(&simulation.state()).unwrap();
        let unit = |rng: &mut Lcg| {
            let q = [0; 4].map(|_| rng.below(2001) as f64 / 1000. - 1.);
            super::quat_normalize(q)
        };
        for (record, motion) in snapshot.chassis.iter_mut().zip(&mut snapshot.motion) {
            record.command.forward_m_s = rng.below(40) as f64 / 10. - 2.;
            record.command.yaw_rate_rad_s = rng.below(40) as f64 / 10. - 2.;
            record.defeated = rng.below(4) == 0;
            if rng.below(3) == 0 {
                record.config = ConfigWire::Hero;
                motion.wheel_spin_rad.truncate(4);
            }
            for v in motion
                .pose
                .translation_m
                .iter_mut()
                .chain(&mut motion.turret.translation_m)
            {
                *v = rng.value(TRANSLATION_SCALE, 18);
            }
            for v in &mut motion.velocity_m_s {
                *v = rng.value(VELOCITY_SCALE, 16);
            }
            for v in motion
                .angular_velocity_rad_s
                .iter_mut()
                .chain(&mut motion.gimbal_velocity_rad_s)
            {
                *v = rng.value(RATE_SCALE, 18);
            }
            for v in motion
                .held_aim_rad
                .iter_mut()
                .chain(&mut motion.wheel_spin_rad)
            {
                *v = rng.value(ANGLE_SCALE, 24);
            }
            motion.pose.rotation_wxyz = unit(rng);
            motion.turret.rotation_wxyz = if rng.below(8) == 0 {
                [0.5, 0.5, 0.5, 0.500_000_1]
            } else {
                unit(rng)
            };
        }
        snapshot.projectiles = (0..balls)
            .map(|index| ProjectileWire {
                id: 100 + rng.below(4) + index as u64,
                caliber: CaliberBit(rng.below(2) == 0),
                launched_ns: 1,
                position_m: [0; 3].map(|_| rng.value(PROJECTILE_SCALE, 18)),
                velocity_m_s: [0; 3].map(|_| rng.value(PROJECTILE_SCALE, 18)),
                shooter: None,
                first_contact_ns: (rng.below(2) == 0).then_some(1),
                dwell_since_ns: None,
            })
            .collect();
        snapshot
    }

    /// The encoder predicts from the tree it built and the decoder from the
    /// tree it rebuilt from the wire; the two predictions must agree bit for
    /// bit for any baseline, any lead and any later frame, including values at
    /// grid edges, escaped values, chassis joining or leaving and projectiles
    /// spawning or retiring between the baseline and the frame.
    #[test]
    fn baseline_prediction_is_bit_exact_between_encoder_and_decoder() {
        use crate::binary_snapshot::bitpack;
        let mut rng = Lcg(42);
        let mut encoder_prediction = Prediction::default();
        let mut decoder_prediction = Prediction::default();
        for case in 0..120_u64 {
            let epoch = case / 40;
            let chassis = rng.below(4) as usize;
            let balls = rng.below(6) as usize;
            let mut base = random_snapshot(&mut rng, chassis, balls);
            base.header.input_epoch = epoch;
            base.header.tick = 10_000 + rng.below(1000);
            let base_node = bitpack::to_node_with(&base, fixed_point::Fine::Root).unwrap();
            // The decoder pins what a proposal frame decodes to.
            let proposal = bitpack::encode(&base_node, None, epoch, case + 1).unwrap();
            let (_, pinned) = decode_checkpoint(&proposal, None, epoch, true).unwrap();
            let pinned = pinned.unwrap();
            assert_eq!(pinned, base_node);
            for _ in 0..3 {
                let lead = match rng.below(6) {
                    0 => 0,
                    1 => -(rng.below(64) as i64),
                    2 => MAX_PREDICTED_TICKS as i64 + rng.below(3) as i64,
                    _ => rng.below(300) as i64,
                };
                let chassis = (chassis + rng.below(3) as usize).saturating_sub(1);
                let balls = rng.below(6) as usize;
                let mut next = random_snapshot(&mut rng, chassis, balls);
                next.header.input_epoch = epoch;
                next.header.tick = base.header.tick.wrapping_add_signed(lead);
                let node = bitpack::to_node_with(&next, fixed_point::Fine::Root).unwrap();
                let key = (epoch, case + 1);
                let encoded = encoder_prediction
                    .predict(&base_node, key, lead)
                    .unwrap()
                    .clone();
                let decoded = decoder_prediction.predict(&pinned, key, lead).unwrap();
                assert_eq!(&encoded, decoded, "case {case} lead {lead}");
                assert_eq!(encoded, predict_baseline(&pinned, lead).unwrap());
                let bytes = encode_checkpoint(
                    &node,
                    next.header.tick,
                    Some((&base_node, case + 1)),
                    epoch,
                    &mut encoder_prediction,
                )
                .unwrap();
                let (_, delivered) = decode_checkpoint_with(
                    &bytes,
                    Some((&pinned, case + 1)),
                    epoch,
                    true,
                    &mut decoder_prediction,
                )
                .unwrap();
                assert_eq!(delivered.unwrap(), node, "case {case} lead {lead}");
            }
        }
    }

    /// The tree patch follows the typed layout: predicting the typed snapshot
    /// and packing it gives the patched tree.
    #[test]
    fn baseline_prediction_patches_the_typed_tree() {
        use crate::binary_snapshot::bitpack;
        let mut rng = Lcg(7);
        for _ in 0..20 {
            let snapshot = random_snapshot(&mut rng, 3, 4);
            let node = bitpack::to_node_with(&snapshot, fixed_point::Fine::Root).unwrap();
            let lead = rng.below(40) as i64 + 1;
            let mut typed: PlayerSnapshot = bitpack::from_node(&node).unwrap();
            typed.header.tick += lead as u64;
            predict_motion(&typed.chassis, &mut typed.motion, lead);
            predict_projectiles(&mut typed.projectiles, lead);
            assert_eq!(
                predict_baseline(&node, lead).unwrap(),
                bitpack::to_node_with(&typed, fixed_point::Fine::Root).unwrap()
            );
        }
    }

    /// A chassis driving straight is predicted to within a few steps and a ball
    /// in flight far closer than its baseline position, so residuals are small.
    #[test]
    fn dead_reckoning_tracks_steady_motion() {
        let (mut simulation, chassis) = crate::workload::simulation(1);
        simulation
            .apply(&crate::protocol::Command::Chassis {
                chassis: chassis[0],
                command: rm_simulator_world::ChassisCommand {
                    forward_m_s: 1.0,
                    ..Default::default()
                },
            })
            .unwrap();
        simulation.step(400).unwrap();
        simulation
            .apply(&crate::protocol::Command::Fire {
                shooter: chassis[0],
            })
            .unwrap();
        simulation.step(2).unwrap();
        let before = checkpoint_node(&simulation.state()).unwrap();
        simulation.step(8).unwrap();
        let after = PlayerSnapshot::from_state(&simulation.state()).unwrap();
        let predicted: PlayerSnapshot =
            crate::binary_snapshot::bitpack::from_node(&predict_baseline(&before, 8).unwrap())
                .unwrap();
        let unpredicted: PlayerSnapshot =
            crate::binary_snapshot::bitpack::from_node(&before).unwrap();
        let error = |a: [f64; 3], b: [f64; 3]| {
            a.iter()
                .zip(b)
                .map(|(a, b)| (a - b).abs())
                .fold(0., f64::max)
        };
        let (m, p, u) = (
            &after.motion[0],
            &predicted.motion[0],
            &unpredicted.motion[0],
        );
        assert!(error(m.pose.translation_m, p.pose.translation_m) < 0.003);
        assert!(error(m.pose.translation_m, u.pose.translation_m) > 0.05);
        let ball = after.projectiles.first().expect("a ball in flight");
        let guess = &predicted.projectiles[0];
        // Drag is not modelled, so the ball runs a little ahead of its guess.
        let guessed = error(ball.position_m, guess.position_m);
        let held = error(ball.position_m, unpredicted.projectiles[0].position_m);
        assert!(guessed * 20. < held, "{guessed} m against {held} m");
    }

    #[test]
    fn stamp_codes_are_exact_and_refuse_missing_remainders() {
        let tick = rm_simulator_world::tick_ns();
        let now_ns = 1_000 * tick;
        let stamps = [
            0,
            now_ns,
            now_ns + 1,
            7 * tick,
            7 * tick + tick - 1,
            u64::MAX,
        ];
        let mut sub_tick_ns = Vec::new();
        let mut codes = stamps;
        let mut encoder = StampCodes::new(now_ns, &mut sub_tick_ns);
        codes.iter_mut().for_each(|stamp| encoder.encode(stamp));
        assert_eq!(codes[1], 0, "the current time is code zero");
        assert_eq!(codes[3], 15, "an on-grid stamp is one plus twice its tick");
        assert_eq!(sub_tick_ns.len(), 3);
        let mut decoded = codes;
        let mut decoder = StampCodes::new(now_ns, &mut sub_tick_ns);
        decoded.iter_mut().for_each(|stamp| decoder.decode(stamp));
        assert!(!decoder.error && decoder.next == 3);
        assert_eq!(decoded, stamps);
        let mut none = Vec::new();
        let mut decoder = StampCodes::new(now_ns, &mut none);
        decoder.decode(&mut codes[2].clone());
        assert!(decoder.error);
    }

    #[test]
    fn extreme_diagnostic_coordinates_keep_full_precision() {
        let mut state = simulation();
        state
            .field
            .projectiles
            .push(rm_simulator_world::ProjectileSnapshot {
                id: 1,
                caliber: rm_simulator_world::Caliber::Mm17,
                launched_ns: 0,
                position_m: [1e60, 0., 0.],
                velocity_m_s: [1., 0., 0.],
                angular_velocity_rad_s: [2.; 3],
                shooter: None,
                first_contact_ns: None,
                dwell_since_ns: None,
            });
        let message = ServerMessage::Snapshot(Box::new(state.clone()));
        let decoded = decode_player_message(&encode_player_message(&message)).unwrap();
        // Spin is never carried; the out-of-range coordinate stays exact.
        state.field.projectiles[0].angular_velocity_rad_s = [0.; 3];
        assert_eq!(decoded, ServerMessage::Snapshot(Box::new(state)));
    }

    #[test]
    fn independent_player_checkpoints_reduce_bytes_and_preserve_reconciliation() {
        use rm_simulator_world::{Caliber, ChassisConfig, ChassisPlacement, Pose, Shot, Team};
        let mut config = FieldConfig::default();
        config.chassis.push(ChassisPlacement {
            team: Team::Red,
            kind: rm_simulator_world::RobotKind::Infantry,
            performance: None,
            config: ChassisConfig::default(),
            spawn: Pose::at([0., 0., 0.2]),
        });
        let mut field = Field::new(&config).unwrap();
        let mut full_bytes = 0;
        let mut compact_bytes = 0;
        for index in 0..80 {
            if index % 4 == 0 {
                field
                    .fire(Pose::at([0., 0., 5.]), Shot::at_limit(Caliber::Mm17), None)
                    .unwrap();
            }
            field.step(16).unwrap();
            let mut state = simulation();
            state.snapshot_id = index + 1;
            state.field = field.snapshot();
            let message = ServerMessage::Snapshot(Box::new(state.clone()));
            // The JSON form is only a size reference here.
            let full = serde_json::to_vec(&message).unwrap();
            let compact = encode_player_message(&message);
            full_bytes += crate::compression::compress(&full).len();
            compact_bytes += crate::compression::compress(&compact).len();
            // Decode each packet in isolation; no earlier frame or delta chain.
            let ServerMessage::Snapshot(actual) = decode_player_message(&compact).unwrap() else {
                panic!("not a snapshot")
            };
            let mut expected = state.clone();
            expected.field.hits.retain(|hit| hit.detected);
            for chassis in &mut expected.field.chassis {
                for wheel in &mut chassis.wheels {
                    wheel.contact = None;
                }
            }
            for ball in &mut expected.field.projectiles {
                ball.angular_velocity_rad_s = [0.; 3];
                let position = ball.position_m.map(|x| f64::from(x as f32));
                assert!(
                    position
                        .into_iter()
                        .zip(ball.position_m)
                        .all(|(a, b)| (a - b).abs() < 0.0001)
                );
                ball.position_m = position;
                ball.velocity_m_s = ball.velocity_m_s.map(|x| f64::from(x as f32));
            }
            equivalent(
                &serde_json::to_value(expected).unwrap(),
                &serde_json::to_value(actual).unwrap(),
            );
            assert_eq!(
                message,
                ServerMessage::Snapshot(Box::new(state)),
                "encoding must not mutate host state"
            );
        }
        eprintln!("UDP full/compact compressed bytes: {full_bytes}/{compact_bytes}");
        assert!(
            compact_bytes * 4 < full_bytes * 3,
            "expect at least 25% less compressed traffic"
        );
    }

    #[test]
    fn chassis_presets_travel_as_indexes_and_other_values_exactly() {
        use rm_simulator_world::{ChassisConfig, ChassisPlacement, Pose, Team};
        let custom = ChassisConfig {
            mass_kg: 17.25,
            ..Default::default()
        };
        let mut config = FieldConfig::default();
        for (index, chassis) in [
            ChassisConfig::default(),
            ChassisConfig::hero(),
            custom,
            ChassisConfig::team_cad(),
        ]
        .into_iter()
        .enumerate()
        {
            config.chassis.push(ChassisPlacement {
                team: Team::Red,
                kind: rm_simulator_world::RobotKind::Infantry,
                performance: None,
                spawn: Pose::at([index as f64, 0., chassis.rest_height_m()]),
                config: chassis,
            });
        }
        let field = Field::new(&config).unwrap();
        let mut state = simulation();
        state.field = field.snapshot();
        let wire = PlayerSnapshot::from_state(&state).unwrap();
        assert!(matches!(
            wire.chassis
                .iter()
                .map(|record| &record.config)
                .collect::<Vec<_>>()[..],
            [
                ConfigWire::Infantry,
                ConfigWire::Hero,
                ConfigWire::Exact(_),
                ConfigWire::Exact(_)
            ]
        ));
        assert!(matches!(wire.rules.projectile_policy, PolicyWire::Default));
        // A policy off the preset travels exactly too.
        if let Some(restore) = &mut state.field.restore {
            restore.projectile_policy = restore.projectile_policy.without_retirement();
        }
        let message = ServerMessage::Snapshot(Box::new(state));
        let decoded = decode_player_message(&encode_player_message(&message)).unwrap();
        let (ServerMessage::Snapshot(expected), ServerMessage::Snapshot(actual)) =
            (&message, &decoded)
        else {
            panic!("a snapshot must decode to a snapshot");
        };
        let configs = |state: &SimulationState| {
            state
                .field
                .chassis
                .iter()
                .map(|chassis| chassis.config.clone())
                .collect::<Vec<_>>()
        };
        assert_eq!(configs(actual), configs(expected));
        assert_eq!(
            actual.field.restore.as_ref().unwrap().projectile_policy,
            expected.field.restore.as_ref().unwrap().projectile_policy
        );
    }

    #[test]
    fn projectile_timestamp_ages_reconstruct_exactly() {
        use rm_simulator_world::{Caliber, ProjectileSnapshot};
        fn ball(
            id: u64,
            caliber: Caliber,
            launched_ns: u64,
            first_contact_ns: Option<u64>,
            dwell_since_ns: Option<u64>,
        ) -> ProjectileSnapshot {
            ProjectileSnapshot {
                id,
                caliber,
                launched_ns,
                position_m: [1., 2., 3.],
                velocity_m_s: [25., 0., 0.],
                angular_velocity_rad_s: [0.; 3],
                shooter: Some(7),
                first_contact_ns,
                dwell_since_ns,
            }
        }
        // The clock rides as a tick, so reach 90 s by stepping the field.
        let mut field = Field::new(&FieldConfig::default()).unwrap();
        field
            .step(90_000_000_000 / rm_simulator_world::tick_ns())
            .unwrap();
        let mut state = simulation();
        state.field = field.snapshot();
        assert_eq!(state.field.time_ns, 90_000_000_000);
        state.field.projectiles = vec![
            ball(1, Caliber::Mm17, 89_000_000_000, None, None),
            ball(2, Caliber::Mm42, 88_000_000_000, Some(88_500_000_000), None),
            ball(
                3,
                Caliber::Mm17,
                87_000_000_000,
                Some(87_100_000_000),
                Some(87_150_000_000),
            ),
        ];
        let message = ServerMessage::Snapshot(Box::new(state.clone()));
        let ServerMessage::Snapshot(actual) =
            decode_player_message(&encode_player_message(&message)).unwrap()
        else {
            panic!("a snapshot must decode to a snapshot");
        };
        assert_eq!(actual.field.projectiles, state.field.projectiles);
    }

    #[test]
    fn coarsened_projectile_checkpoints_replay_within_centimetres() {
        use crate::protocol::Command;
        // Balls in flight, cut through the compact checkpoint, then replayed
        // from the restore against the exact state: the 1 mm / 1 mm/s
        // projectile quantization must not move any ball by armor scale.
        let (mut simulation, chassis) = crate::workload::simulation(1);
        let shooter = chassis[0];
        for frame in 0..40 {
            if frame % 4 == 0 {
                let _ = simulation.apply(&Command::Fire { shooter });
            }
            simulation.step(4).unwrap();
        }
        let state = simulation.state();
        assert!(
            !state.field.projectiles.is_empty(),
            "the workload must hold balls in flight"
        );
        // The periodic lane's quantized checkpoint, not the f32 confirmation.
        let bytes = crate::binary_snapshot::bitpack::encode(
            &checkpoint_node(&state).unwrap(),
            None,
            state.input_epoch,
            0,
        )
        .unwrap();
        let (ServerMessage::Snapshot(decoded), _) =
            decode_checkpoint(&bytes, None, state.input_epoch, false).unwrap()
        else {
            panic!("a snapshot must decode to a snapshot");
        };
        let geometry = simulation.field().static_geometry_snapshot();
        let mut exact = Field::restore(&state.field, &geometry, 0.).unwrap();
        let mut replay = Field::restore(&decoded.field, &geometry, 0.).unwrap();
        exact.step(512).unwrap();
        replay.step(512).unwrap();
        let replayed = replay.snapshot().projectiles;
        for ball in exact.snapshot().projectiles {
            let other = replayed
                .iter()
                .find(|other| other.id == ball.id)
                .unwrap_or_else(|| panic!("ball {} must survive the replay", ball.id));
            let drift_m = ball
                .position_m
                .into_iter()
                .zip(other.position_m)
                .map(|(a, b)| (a - b).powi(2))
                .sum::<f64>()
                .sqrt();
            assert!(
                drift_m < 0.05,
                "ball {} drifted {drift_m:.4} m over four seconds",
                ball.id
            );
        }
    }
}
