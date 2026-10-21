// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Explicit own-robot telemetry from native 2027 snapshots into the older schema.
//! This is privileged caller access; it does not broadcast opponent truth or
//! execute commands. Unsupported fields remain absent rather than mocked as live.
use super::client;
use rm_simulator_world::{
    FieldSnapshot, Team,
    rmuc_2027::match_rules::{GroundRole, Phase},
};
use serde::{Deserialize, Serialize};

/// Calibrated position transform; no referee origin or true-north convention is assumed.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PositionFrame {
    /// Arena origin expressed in protocol metres.
    pub arena_origin_protocol_m: [f64; 3],
    /// Counterclockwise rotation of arena +x in the protocol XY plane, radians.
    pub arena_yaw_protocol_rad: f64,
}
impl PositionFrame {
    /// Transform arena FLU metres, rejecting nonfinite/unrepresentable float32 results.
    pub fn position(self, arena_m: [f64; 3]) -> Result<[f32; 3], &'static str> {
        if !self
            .arena_origin_protocol_m
            .into_iter()
            .chain([self.arena_yaw_protocol_rad])
            .chain(arena_m)
            .all(f64::is_finite)
        {
            return Err("position transform must be finite");
        }
        let (s, c) = self.arena_yaw_protocol_rad.sin_cos();
        let [x, y, z] = arena_m;
        let p = [
            c * x - s * y + self.arena_origin_protocol_m[0],
            s * x + c * y + self.arena_origin_protocol_m[1],
            z + self.arena_origin_protocol_m[2],
        ];
        let p = p.map(|v| v as f32);
        if p.into_iter().all(f32::is_finite) {
            Ok(p)
        } else {
            Err("protocol position exceeds float32 range")
        }
    }
}
/// Own-robot telemetry assembled at one explicit simulation instant.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OwnTelemetry {
    /// Source world time, in nanoseconds; not a UNIX/NTP timestamp.
    pub time_ns: u64,
    /// Physical chassis identity, never a referee ID.
    pub chassis_id: u32,
    /// Caller-mapped official robot ID.
    pub referee_id: u8,
    /// Match stage/clock; series results remain unspecified.
    pub game: client::GameStatus,
    /// Native HP, level and limits with a compatibility role code.
    pub robot_static: client::RobotStaticStatus,
    /// Native HP, heat, allowance, experience, buffer and out-of-combat state.
    pub robot_dynamic: client::RobotDynamicStatus,
    /// Whether a native respawn is pending; unsupported pricing/progress fields absent.
    pub respawn: client::RobotRespawnStatus,
    /// Position only when a caller supplies frame calibration; yaw stays absent.
    pub position: Option<client::RobotPosition>,
}
/// Project a single native robot without supplying a transport, scheduler or enemy truth.
///
/// `referee_id` must match the team's official ID range and compatibility role:
/// Heavy 1/2, Infantry 3/4/5, Sentry 7. Heavy retains the Engineer type code 2.
/// A caller selects its numbering explicitly; internal chassis IDs are not converted.
/// Buffer microjoules and experience tenths round down to protocol whole units.
///
/// ```
/// use rm_simulator_server::{workload, referee_protocol::projection::own_robot};
/// let (simulation, ids) = workload::simulation(1);
/// let snapshot = simulation.snapshot();
/// let telemetry = own_robot(&snapshot, false, ids[0], 3, None)?;
/// assert_eq!(telemetry.robot_static.robot_id, Some(3));
/// assert!(telemetry.robot_static.max_health.is_some());
/// assert!(telemetry.robot_dynamic.last_projectile_firing_rate.is_none());
/// assert!(telemetry.position.is_none());
/// # Ok::<(), &'static str>(())
/// ```
pub fn own_robot(
    field: &FieldSnapshot,
    paused: bool,
    chassis_id: u32,
    referee_id: u8,
    frame: Option<PositionFrame>,
) -> Result<OwnTelemetry, &'static str> {
    let state = field
        .rmuc_2027
        .as_ref()
        .ok_or("native RMUC 2027 authority required")?;
    let game = &state.game;
    let robot = game
        .robots
        .iter()
        .find(|r| r.id == chassis_id)
        .ok_or("native robot missing")?;
    let body = field
        .chassis
        .iter()
        .find(|r| r.id == chassis_id)
        .ok_or("physical robot missing")?;
    let blue = body.team == Team::Blue;
    let number = if blue {
        referee_id
            .checked_sub(100)
            .ok_or("referee ID has wrong team")?
    } else {
        referee_id
    };
    let valid = match robot.role {
        GroundRole::Heavy => matches!(number, 1 | 2),
        GroundRole::Infantry => matches!(number, 3..=5),
        GroundRole::Sentry => number == 7,
    };
    if !valid {
        return Err("referee ID does not match robot team/role");
    }
    let now = game.elapsed_ns();
    let stats = robot.stats(now);
    let seconds = |ns: u64| i32::try_from(ns / 1_000_000_000).unwrap_or(i32::MAX);
    let (stage, countdown, elapsed) = match game.phase {
        Phase::Idle => (0, None, None),
        Phase::Countdown => (3, Some(seconds(game.countdown_ns)), None),
        Phase::Running => (
            4,
            Some(seconds(
                rm_simulator_world::gameplay::rmuc_2027::ROUND_NS.saturating_sub(now),
            )),
            Some(seconds(now)),
        ),
        Phase::Finished => (5, None, None),
    };
    let position = frame
        .map(|frame| -> Result<client::RobotPosition, &'static str> {
            let [x, y, z] = frame.position(body.pose.translation_m)?;
            Ok(client::RobotPosition {
                x: Some(x),
                y: Some(y),
                z: Some(z),
                ..Default::default()
            })
        })
        .transpose()?;
    Ok(OwnTelemetry {
        time_ns: field.time_ns,
        chassis_id,
        referee_id,
        game: client::GameStatus {
            current_round: (game.phase != Phase::Idle).then_some(1),
            current_stage: Some(stage),
            stage_countdown_sec: countdown,
            stage_elapsed_sec: elapsed,
            is_paused: Some(paused),
            ..Default::default()
        },
        robot_static: client::RobotStaticStatus {
            connection_state: Some(u32::from(robot.online)),
            alive_state: Some(if robot.hp > 0 { 1 } else { 2 }),
            robot_id: Some(u32::from(referee_id)),
            robot_type: Some(match robot.role {
                GroundRole::Heavy => 2,
                GroundRole::Infantry => 3,
                GroundRole::Sentry => 5,
            }),
            level: Some(u32::from(robot.combat.level)),
            max_health: Some(robot.max_hp),
            max_heat: Some(stats.heat_limit),
            heat_cooldown_rate: Some(stats.cooling_per_s as f32),
            max_power: Some(stats.chassis_power_w),
            ..Default::default()
        },
        robot_dynamic: client::RobotDynamicStatus {
            current_health: Some(robot.hp),
            current_heat: Some(robot.combat.heat_tenths as f32 / 10.),
            current_experience: Some(
                u32::try_from(robot.combat.experience_tenths / 10).unwrap_or(u32::MAX),
            ),
            current_buffer_energy: Some(
                u32::try_from(robot.combat.power.buffer_uj / 1_000_000).unwrap_or(u32::MAX),
            ),
            remaining_ammo: Some(robot.combat.allowance),
            is_out_of_combat: Some(robot.out_of_combat(now)),
            ..Default::default()
        },
        respawn: client::RobotRespawnStatus {
            is_pending_revive: Some(robot.combat.respawn.is_some()),
            ..Default::default()
        },
        position,
    })
}

impl OwnTelemetry {
    /// Collect the supported topics for a caller-owned encoder/transport.
    /// Keep [`Self::time_ns`] separately if acquisition stamping is required;
    /// these official message payloads do not all carry timestamps.
    pub fn into_messages(self) -> Vec<client::Message> {
        let mut messages = vec![
            client::Message::GameStatus(self.game),
            client::Message::RobotStaticStatus(self.robot_static),
            client::Message::RobotDynamicStatus(self.robot_dynamic),
            client::Message::RobotRespawnStatus(self.respawn),
        ];
        if let Some(position) = self.position {
            messages.push(client::Message::RobotPosition(position));
        }
        messages
    }
}
