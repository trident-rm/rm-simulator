// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! All custom-client topics, Communication Protocol V2.0.0 §§2.1–2.2.36.
//! Optional scalars preserve absence; numeric enumerations retain unknown codes.
//! Field descriptors preserve Protobuf tags without adding a codec dependency.
use serde::{Deserialize, Serialize};

/// One logical Protobuf field; encoding remains the caller's responsibility.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Field {
    /// Logical Rust field name; reserved words use Rust raw identifiers.
    pub name: &'static str,
    /// Protobuf field number from the numbered table (including documented corrections).
    pub tag: u32,
    /// Scalar/message type and cardinality, using Protobuf terminology.
    pub proto_type: &'static str,
}
/// One custom-client topic from Table 2-1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Descriptor {
    /// Exact MQTT topic/message name; no broker is created here.
    pub topic: &'static str,
    /// Source section in the supplied V2.0.0 manual.
    pub source: &'static str,
    /// Field numbers/types for a caller-provided codec.
    pub fields: &'static [Field],
}
macro_rules! messages {
    ($($name:ident, $source:literal { $( $field:ident : $ty:ty => ($tag:literal, $proto:literal, $doc:literal)),* $(,)? })*) => {
        $(#[doc = concat!("Custom-client ", stringify!($name), "; V2.0.0 §", $source, ".")]
        #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
        pub struct $name { $(#[doc = $doc] pub $field: $ty,)* }
        impl $name {
            /// Protocol topic and field descriptor; not a wire encoder.
            pub const DESCRIPTOR: Descriptor = Descriptor { topic: stringify!($name), source: $source,
                fields: &[$(Field {name: stringify!($field), tag: $tag, proto_type: $proto},)*] };
        })*
        /// Complete custom-client message/command family, including caller-originated commands.
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        pub enum Message { $(#[doc = concat!("V2.0.0 §", $source, ".")] $name($name),)* }
        impl Message {
            /// Topic, source and field tags for this payload.
            pub fn descriptor(&self) -> Descriptor { match self { $(Self::$name(_) => $name::DESCRIPTOR,)* } }
            /// One synthetic template per topic, with absent optional fields.
            pub fn templates() -> Vec<Self> { vec![$(Self::$name($name::default()),)*] }
        }
        /// All top-level custom-client topic descriptors, in manual order.
        pub const CATALOG: &[Descriptor] = &[$($name::DESCRIPTOR,)*];
    };
}
/// One Radar entry in opponent 1/2/3/4/6/7 then own 1/2/3/4/6/7 order (§2.2.19).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RadarSingleRobotInfo {
    /// Protocol-frame x in centimetres (tag 1).
    pub target_pos_x: Option<u32>,
    /// Protocol-frame y in centimetres (tag 2).
    pub target_pos_y: Option<u32>,
    /// Identification status 0/1/2; retain unknown codes (tag 3).
    pub is_high_light: Option<u32>,
}
messages! {
    KeyboardMouseControl, "2.2.1" {
        mouse_x: Option<i32> => (1, "optional int32", "Horizontal mouse movement speed; negative is left."),
        mouse_y: Option<i32> => (2, "optional int32", "Vertical mouse movement speed; negative is down."),
        mouse_z: Option<i32> => (3, "optional int32", "Wheel speed; negative is backward."),
        left_button_down: Option<bool> => (4, "optional bool", "Left button down; V2.0.0 §2.2.1, field 4 (raw protocol value)."),
        right_button_down: Option<bool> => (5, "optional bool", "Right button down; V2.0.0 §2.2.1, field 5 (raw protocol value)."),
        keyboard_value: Option<u32> => (6, "optional uint32", "Keys W/S/A/D/Shift/Ctrl/Q/E/R/F/G/Z/X/C/V/B in bits 0–15."),
        mid_button_down: Option<bool> => (7, "optional bool", "Mid button down; V2.0.0 §2.2.1, field 7 (raw protocol value)."),
    }
    CustomControl, "2.2.2" {
        data: Option<Vec<u8>> => (1, "optional bytes", "Data; V2.0.0 §2.2.2, field 1 (raw protocol value)."),
    }
    GameStatus, "2.2.3" {
        current_round: Option<u32> => (1, "optional uint32", "Current round; V2.0.0 §2.2.3, field 1 (raw protocol value)."),
        total_rounds: Option<u32> => (2, "optional uint32", "Total rounds; V2.0.0 §2.2.3, field 2 (raw protocol value)."),
        red_score: Option<u32> => (3, "optional uint32", "Red score; V2.0.0 §2.2.3, field 3 (raw protocol value)."),
        blue_score: Option<u32> => (4, "optional uint32", "Blue score; V2.0.0 §2.2.3, field 4 (raw protocol value)."),
        current_stage: Option<u32> => (5, "optional uint32", "Current stage; V2.0.0 §2.2.3, field 5 (raw protocol value)."),
        stage_countdown_sec: Option<i32> => (6, "optional int32", "Stage countdown sec; V2.0.0 §2.2.3, field 6 (raw protocol value)."),
        stage_elapsed_sec: Option<i32> => (7, "optional int32", "Stage elapsed sec; V2.0.0 §2.2.3, field 7 (raw protocol value)."),
        is_paused: Option<bool> => (8, "optional bool", "Is paused; V2.0.0 §2.2.3, field 8 (raw protocol value)."),
        game_result: Option<u32> => (9, "optional uint32", "Game result; V2.0.0 §2.2.3, field 9 (raw protocol value)."),
        end_reason: Option<u32> => (10, "optional uint32", "End reason; V2.0.0 §2.2.3, field 10 (raw protocol value)."),
    }
    GlobalUnitStatus, "2.2.4" {
        base_health: Option<u32> => (1, "optional uint32", "Base health; V2.0.0 §2.2.4, field 1 (raw protocol value)."),
        base_status: Option<u32> => (2, "optional uint32", "Base status; V2.0.0 §2.2.4, field 2 (raw protocol value)."),
        base_shield: Option<u32> => (3, "optional uint32", "Base shield; V2.0.0 §2.2.4, field 3 (raw protocol value)."),
        outpost_health: Option<u32> => (4, "optional uint32", "Outpost health; V2.0.0 §2.2.4, field 4 (raw protocol value)."),
        outpost_status: Option<u32> => (5, "optional uint32", "Outpost status; V2.0.0 §2.2.4, field 5 (raw protocol value)."),
        enemy_base_health: Option<u32> => (6, "optional uint32", "Enemy base health; V2.0.0 §2.2.4, field 6 (raw protocol value)."),
        enemy_base_status: Option<u32> => (7, "optional uint32", "Enemy base status; V2.0.0 §2.2.4, field 7 (raw protocol value)."),
        enemy_base_shield: Option<u32> => (8, "optional uint32", "Enemy base shield; V2.0.0 §2.2.4, field 8 (raw protocol value)."),
        enemy_outpost_health: Option<u32> => (9, "optional uint32", "Enemy outpost health; V2.0.0 §2.2.4, field 9 (raw protocol value)."),
        enemy_outpost_status: Option<u32> => (10, "optional uint32", "Enemy outpost status; V2.0.0 §2.2.4, field 10 (raw protocol value)."),
        robot_health: Vec<u32> => (11, "repeated uint32", "Robot health; V2.0.0 §2.2.4, field 11 (raw protocol value)."),
        robot_bullets: Vec<i32> => (12, "repeated int32", "Robot bullets; V2.0.0 §2.2.4, field 12 (raw protocol value)."),
        total_damage_ally: Option<u32> => (13, "optional uint32", "Total damage ally; V2.0.0 §2.2.4, field 13 (raw protocol value)."),
        total_damage_enemy: Option<u32> => (14, "optional uint32", "Total damage enemy; V2.0.0 §2.2.4, field 14 (raw protocol value)."),
    }
    GlobalLogisticsStatus, "2.2.5" {
        remaining_economy: Option<u32> => (1, "optional uint32", "Remaining economy; V2.0.0 §2.2.5, field 1 (raw protocol value)."),
        total_economy_obtained: Option<u64> => (2, "optional uint64", "Total economy obtained; V2.0.0 §2.2.5, field 2 (raw protocol value)."),
        own_side_level: Option<u32> => (3, "optional uint32", "Own side level; V2.0.0 §2.2.5, field 3 (raw protocol value)."),
        encryption_level: Option<u32> => (4, "optional uint32", "Encryption level; V2.0.0 §2.2.5, field 4 (raw protocol value)."),
    }
    GlobalSpecialMechanism, "2.2.6" {
        mechanism_id: Vec<u32> => (1, "repeated uint32", "Mechanism IDs paired by index with mechanism_time_sec; malformed optional repeated normalized."),
        mechanism_time_sec: Vec<i32> => (2, "repeated int32", "Mechanism time in seconds, paired by index with mechanism_id."),
    }
    Event, "2.2.7" {
        event_id: Option<i32> => (1, "optional int32", "Event id; V2.0.0 §2.2.7, field 1 (raw protocol value)."),
        param: Option<String> => (2, "optional string", "Param; V2.0.0 §2.2.7, field 2 (raw protocol value)."),
    }
    RobotInjuryStat, "2.2.8" {
        total_damage: Option<u32> => (1, "optional uint32", "Total damage; V2.0.0 §2.2.8, field 1 (raw protocol value)."),
        collision_damage: Option<u32> => (2, "optional uint32", "Collision HP loss; table tag 2 omitted from printed declaration."),
        small_projectile_damage: Option<u32> => (3, "optional uint32", "Small projectile damage; V2.0.0 §2.2.8, field 3 (raw protocol value)."),
        large_projectile_damage: Option<u32> => (4, "optional uint32", "Large projectile damage; V2.0.0 §2.2.8, field 4 (raw protocol value)."),
        dart_splash_damage: Option<u32> => (5, "optional uint32", "Dart splash damage; V2.0.0 §2.2.8, field 5 (raw protocol value)."),
        module_offline_damage: Option<u32> => (6, "optional uint32", "Module offline damage; V2.0.0 §2.2.8, field 6 (raw protocol value)."),
        offline_damage: Option<u32> => (7, "optional uint32", "Offline damage; V2.0.0 §2.2.8, field 7 (raw protocol value)."),
        penalty_damage: Option<u32> => (8, "optional uint32", "Penalty damage; V2.0.0 §2.2.8, field 8 (raw protocol value)."),
        server_kill_damage: Option<u32> => (9, "optional uint32", "Server kill damage; V2.0.0 §2.2.8, field 9 (raw protocol value)."),
        killer_id: Option<u32> => (10, "optional uint32", "Killer id; V2.0.0 §2.2.8, field 10 (raw protocol value)."),
    }
    RobotRespawnStatus, "2.2.9" {
        is_pending_revive: Option<bool> => (1, "optional bool", "Is pending revive; V2.0.0 §2.2.9, field 1 (raw protocol value)."),
        total_revive_progress: Option<u32> => (2, "optional uint32", "Total revive progress; V2.0.0 §2.2.9, field 2 (raw protocol value)."),
        current_revive_progress: Option<u32> => (3, "optional uint32", "Current revive progress; V2.0.0 §2.2.9, field 3 (raw protocol value)."),
        can_revive_for_free: Option<bool> => (4, "optional bool", "Can revive for free; V2.0.0 §2.2.9, field 4 (raw protocol value)."),
        gold_coin_cost_for_revive: Option<u32> => (5, "optional uint32", "Gold coin cost for revive; V2.0.0 §2.2.9, field 5 (raw protocol value)."),
        can_pay_for_revive: Option<bool> => (6, "optional bool", "Can pay for revive; V2.0.0 §2.2.9, field 6 (raw protocol value)."),
    }
    RobotStaticStatus, "2.2.10" {
        connection_state: Option<u32> => (1, "optional uint32", "Connection state; V2.0.0 §2.2.10, field 1 (raw protocol value)."),
        field_state: Option<u32> => (2, "optional uint32", "Field state; V2.0.0 §2.2.10, field 2 (raw protocol value)."),
        alive_state: Option<u32> => (3, "optional uint32", "Alive state; V2.0.0 §2.2.10, field 3 (raw protocol value)."),
        robot_id: Option<u32> => (4, "optional uint32", "Robot id; V2.0.0 §2.2.10, field 4 (raw protocol value)."),
        robot_type: Option<u32> => (5, "optional uint32", "Robot type; V2.0.0 §2.2.10, field 5 (raw protocol value)."),
        performance_system_shooter: Option<u32> => (6, "optional uint32", "Performance system shooter; V2.0.0 §2.2.10, field 6 (raw protocol value)."),
        performance_system_chassis: Option<u32> => (7, "optional uint32", "Performance system chassis; V2.0.0 §2.2.10, field 7 (raw protocol value)."),
        level: Option<u32> => (8, "optional uint32", "Level; V2.0.0 §2.2.10, field 8 (raw protocol value)."),
        max_health: Option<u32> => (9, "optional uint32", "Max health; V2.0.0 §2.2.10, field 9 (raw protocol value)."),
        max_heat: Option<u32> => (10, "optional uint32", "Max heat; V2.0.0 §2.2.10, field 10 (raw protocol value)."),
        heat_cooldown_rate: Option<f32> => (11, "optional float", "Heat cooldown rate; V2.0.0 §2.2.10, field 11 (raw protocol value)."),
        max_power: Option<u32> => (12, "optional uint32", "Max power; V2.0.0 §2.2.10, field 12 (raw protocol value)."),
        max_buffer_energy: Option<u32> => (13, "optional uint32", "Max buffer energy; V2.0.0 §2.2.10, field 13 (raw protocol value)."),
        max_chassis_energy: Option<u32> => (14, "optional uint32", "Max chassis energy; V2.0.0 §2.2.10, field 14 (raw protocol value)."),
    }
    RobotDynamicStatus, "2.2.11" {
        current_health: Option<u32> => (1, "optional uint32", "Current health; V2.0.0 §2.2.11, field 1 (raw protocol value)."),
        current_heat: Option<f32> => (2, "optional float", "Current heat; V2.0.0 §2.2.11, field 2 (raw protocol value)."),
        last_projectile_firing_rate: Option<f32> => (3, "optional float", "Previous projectile speed in m/s per table §2.2.11; prose identifier is misleading."),
        current_chassis_energy: Option<u32> => (4, "optional uint32", "Current chassis energy; V2.0.0 §2.2.11, field 4 (raw protocol value)."),
        current_buffer_energy: Option<u32> => (5, "optional uint32", "Current buffer energy; V2.0.0 §2.2.11, field 5 (raw protocol value)."),
        current_experience: Option<u32> => (6, "optional uint32", "Current experience; V2.0.0 §2.2.11, field 6 (raw protocol value)."),
        experience_needed_for_upgrade: Option<u32> => (7, "optional uint32", "Experience needed for upgrade; V2.0.0 §2.2.11, field 7 (raw protocol value)."),
        total_projectiles_fired: Option<u32> => (8, "optional uint32", "Total projectiles fired; V2.0.0 §2.2.11, field 8 (raw protocol value)."),
        remaining_ammo: Option<u32> => (9, "optional uint32", "Remaining ammo; V2.0.0 §2.2.11, field 9 (raw protocol value)."),
        is_out_of_combat: Option<bool> => (10, "optional bool", "Is out of combat; V2.0.0 §2.2.11, field 10 (raw protocol value)."),
        out_of_combat_countdown: Option<u32> => (11, "optional uint32", "Out of combat countdown; V2.0.0 §2.2.11, field 11 (raw protocol value)."),
        can_be_repaired_remotely: Option<bool> => (12, "optional bool", "Can be repaired remotely; V2.0.0 §2.2.11, field 12 (raw protocol value)."),
        can_remote_ammo: Option<bool> => (13, "optional bool", "Can remote ammo; V2.0.0 §2.2.11, field 13 (raw protocol value)."),
    }
    RobotModuleStatus, "2.2.12" {
        power_manager: Option<u32> => (1, "optional uint32", "Power manager; V2.0.0 §2.2.12, field 1 (raw protocol value)."),
        rfid: Option<u32> => (2, "optional uint32", "Rfid; V2.0.0 §2.2.12, field 2 (raw protocol value)."),
        light_strip: Option<u32> => (3, "optional uint32", "Light strip; V2.0.0 §2.2.12, field 3 (raw protocol value)."),
        small_shooter: Option<u32> => (4, "optional uint32", "Small shooter; V2.0.0 §2.2.12, field 4 (raw protocol value)."),
        big_shooter: Option<u32> => (5, "optional uint32", "Big shooter; V2.0.0 §2.2.12, field 5 (raw protocol value)."),
        uwb: Option<u32> => (6, "optional uint32", "Uwb; V2.0.0 §2.2.12, field 6 (raw protocol value)."),
        armor: Option<u32> => (7, "optional uint32", "Armor; V2.0.0 §2.2.12, field 7 (raw protocol value)."),
        video_transmission: Option<u32> => (8, "optional uint32", "Video transmission; V2.0.0 §2.2.12, field 8 (raw protocol value)."),
        capacitor: Option<u32> => (9, "optional uint32", "Capacitor; V2.0.0 §2.2.12, field 9 (raw protocol value)."),
        main_controller: Option<u32> => (10, "optional uint32", "Main controller; V2.0.0 §2.2.12, field 10 (raw protocol value)."),
        laser_detection_module: Option<u32> => (11, "optional uint32", "Laser detection module; V2.0.0 §2.2.12, field 11 (raw protocol value)."),
    }
    RobotPosition, "2.2.13" {
        x: Option<f32> => (1, "optional float", "x coordinate in protocol-frame metres; caller supplies frame calibration."),
        y: Option<f32> => (2, "optional float", "y coordinate in protocol-frame metres; caller supplies frame calibration."),
        z: Option<f32> => (3, "optional float", "z coordinate in protocol-frame metres; caller supplies frame calibration."),
        yaw: Option<f32> => (4, "optional float", "Speed Monitor heading in degrees, zero true north; requires explicit calibration."),
        recipient_id: Option<u32> => (5, "optional uint32", "Recipient 6/106 (Drone) or 7/107 (Sentry); table tag 5 omitted from printed declaration."),
    }
    Buff, "2.2.14" {
        robot_id: Option<u32> => (1, "optional uint32", "Robot id; V2.0.0 §2.2.14, field 1 (raw protocol value)."),
        buff_type: Option<u32> => (2, "optional uint32", "Buff type; V2.0.0 §2.2.14, field 2 (raw protocol value)."),
        buff_level: Option<i32> => (3, "optional int32", "Buff level; V2.0.0 §2.2.14, field 3 (raw protocol value)."),
        buff_max_time: Option<u32> => (4, "optional uint32", "Buff max time; V2.0.0 §2.2.14, field 4 (raw protocol value)."),
        buff_left_time: Option<u32> => (5, "optional uint32", "Buff left time; V2.0.0 §2.2.14, field 5 (raw protocol value)."),
    }
    PenaltyInfo, "2.2.15" {
        penalty_type: Option<u32> => (1, "optional uint32", "Penalty type; V2.0.0 §2.2.15, field 1 (raw protocol value)."),
        penalty_effect_sec: Option<u32> => (2, "optional uint32", "Penalty effect sec; V2.0.0 §2.2.15, field 2 (raw protocol value)."),
        total_penalty_num: Option<u32> => (3, "optional uint32", "Total penalty num; V2.0.0 §2.2.15, field 3 (raw protocol value)."),
    }
    RobotPathPlanInfo, "2.2.16" {
        intention: Option<u32> => (1, "optional uint32", "Intention; V2.0.0 §2.2.16, field 1 (raw protocol value)."),
        start_pos_x: Option<u32> => (2, "optional uint32", "start pos x in decimetres."),
        start_pos_y: Option<u32> => (3, "optional uint32", "start pos y in decimetres."),
        offset_x: Vec<i32> => (4, "repeated int32", "49 signed x increments in dm, each -128 through 127."),
        offset_y: Vec<i32> => (5, "repeated int32", "49 signed y increments in dm, each -128 through 127."),
        sender_id: Option<u32> => (6, "optional uint32", "Sender id; V2.0.0 §2.2.16, field 6 (raw protocol value)."),
    }
    MapClickInfo, "2.2.17" {
        is_send_all: Option<u32> => (1, "optional uint32", "Is send all; V2.0.0 §2.2.17, field 1 (raw protocol value)."),
        robot_id: Option<Vec<u8>> => (2, "optional bytes", "Exactly seven recipient-ID bytes when present; zero pads unused entries."),
        mode: Option<u32> => (3, "optional uint32", "Mode; V2.0.0 §2.2.17, field 3 (raw protocol value)."),
        enemy_id: Option<u32> => (4, "optional uint32", "Enemy id; V2.0.0 §2.2.17, field 4 (raw protocol value)."),
        ascii: Option<u32> => (5, "optional uint32", "Ascii; V2.0.0 §2.2.17, field 5 (raw protocol value)."),
        r#type: Option<u32> => (6, "optional uint32", "Type; V2.0.0 §2.2.17, field 6 (raw protocol value)."),
        map_x: Option<f32> => (7, "optional float", "map_x coordinate in protocol-frame metres; caller supplies frame calibration."),
        map_y: Option<f32> => (8, "optional float", "map_y coordinate in protocol-frame metres; caller supplies frame calibration."),
    }
    MapClickCmd, "2.2.18" {
        is_send_all: Option<u32> => (1, "optional uint32", "Is send all; V2.0.0 §2.2.18, field 1 (raw protocol value)."),
        robot_id: Option<Vec<u8>> => (2, "optional bytes", "Exactly seven recipient-ID bytes when present; zero pads unused entries."),
        mode: Option<u32> => (3, "optional uint32", "Mode; V2.0.0 §2.2.18, field 3 (raw protocol value)."),
        enemy_id: Option<u32> => (4, "optional uint32", "Enemy id; V2.0.0 §2.2.18, field 4 (raw protocol value)."),
        ascii: Option<u32> => (5, "optional uint32", "Ascii; V2.0.0 §2.2.18, field 5 (raw protocol value)."),
        r#type: Option<u32> => (6, "optional uint32", "Type; V2.0.0 §2.2.18, field 6 (raw protocol value)."),
        map_x: Option<f32> => (7, "optional float", "map_x coordinate in protocol-frame metres; caller supplies frame calibration."),
        map_y: Option<f32> => (8, "optional float", "map_y coordinate in protocol-frame metres; caller supplies frame calibration."),
    }
    RadarInfoToClient, "2.2.19" {
        radar_info: Vec<RadarSingleRobotInfo> => (1, "repeated RadarSingleRobotInfo", "Twelve nested Radar entries, table field 1; printed repeated uint32 declaration is malformed."),
    }
    CustomByteBlock, "2.2.20" {
        data: Vec<u8> => (1, "bytes", "Up to 300 custom bytes, proto3 bytes field 1 (§2.2.20)."),
    }
    AssemblyCommand, "2.2.21" {
        operation: Option<u32> => (1, "optional uint32", "Operation; V2.0.0 §2.2.21, field 1 (raw protocol value)."),
        difficulty: Option<u32> => (2, "optional uint32", "Difficulty; V2.0.0 §2.2.21, field 2 (raw protocol value)."),
    }
    TechCoreMotionStateSync, "2.2.22" {
        maximum_difficulty_level: Option<u32> => (1, "optional uint32", "Maximum difficulty level; V2.0.0 §2.2.22, field 1 (raw protocol value)."),
        basic_state: Option<u32> => (2, "optional uint32", "Basic state; V2.0.0 §2.2.22, field 2 (raw protocol value)."),
        putin_state: Option<u32> => (3, "optional uint32", "Unit insertion state; V2.0.0 §2.2.22, field 3 (raw protocol value)."),
        move_state: Option<u32> => (4, "optional uint32", "Move state; V2.0.0 §2.2.22, field 4 (raw protocol value)."),
        rotate_state: Option<u32> => (5, "optional uint32", "Rotate state; V2.0.0 §2.2.22, field 5 (raw protocol value)."),
        enemy_core_status: Option<u32> => (6, "optional uint32", "Enemy core status; V2.0.0 §2.2.22, field 6 (raw protocol value)."),
        remain_time_all: Option<u32> => (7, "optional uint32", "Remain time all; V2.0.0 §2.2.22, field 7 (raw protocol value)."),
        remain_time_step: Option<u32> => (8, "optional uint32", "Remain time step; V2.0.0 §2.2.22, field 8 (raw protocol value)."),
    }
    RobotPerformanceSelectionCommand, "2.2.23" {
        shooter: Option<u32> => (1, "optional uint32", "Shooter; V2.0.0 §2.2.23, field 1 (raw protocol value)."),
        chassis: Option<u32> => (2, "optional uint32", "Chassis; V2.0.0 §2.2.23, field 2 (raw protocol value)."),
        sentry_control: Option<u32> => (3, "optional uint32", "Sentry control; V2.0.0 §2.2.23, field 3 (raw protocol value)."),
    }
    RobotPerformanceSelectionSync, "2.2.24" {
        shooter: Option<u32> => (1, "optional uint32", "Shooter; V2.0.0 §2.2.24, field 1 (raw protocol value)."),
        chassis: Option<u32> => (2, "optional uint32", "Chassis; V2.0.0 §2.2.24, field 2 (raw protocol value)."),
        sentry_control: Option<u32> => (3, "optional uint32", "Sentry control; V2.0.0 §2.2.24, field 3 (raw protocol value)."),
    }
    CommonCommand, "2.2.25" {
        cmd_type: Option<u32> => (1, "optional uint32", "Cmd type; V2.0.0 §2.2.25, field 1 (raw protocol value)."),
        param: Option<u32> => (2, "optional uint32", "Param; V2.0.0 §2.2.25, field 2 (raw protocol value)."),
    }
    HeroDeployModeEventCommand, "2.2.26" {
        mode: Option<u32> => (1, "optional uint32", "Mode; V2.0.0 §2.2.26, field 1 (raw protocol value)."),
    }
    DeployModeStatusSync, "2.2.27" {
        status: Option<u32> => (1, "optional uint32", "Status; V2.0.0 §2.2.27, field 1 (raw protocol value)."),
    }
    RuneActivateCommand, "2.2.28" {
        activate: Option<u32> => (1, "optional uint32", "Activate; V2.0.0 §2.2.28, field 1 (raw protocol value)."),
    }
    RuneStatusSync, "2.2.29" {
        rune_status: Option<u32> => (1, "optional uint32", "Rune status; V2.0.0 §2.2.29, field 1 (raw protocol value)."),
        activated_arms: Option<u32> => (2, "optional uint32", "Activated arms; V2.0.0 §2.2.29, field 2 (raw protocol value)."),
        average_rings: Option<f32> => (3, "optional float", "Average rings; V2.0.0 §2.2.29, field 3 (raw protocol value)."),
    }
    SentryStatusSync, "2.2.30" {
        posture_id: Option<u32> => (1, "optional uint32", "Posture id; V2.0.0 §2.2.30, field 1 (raw protocol value)."),
        is_weakened: Option<bool> => (2, "optional bool", "Is weakened; V2.0.0 §2.2.30, field 2 (raw protocol value)."),
        is_powered: Option<bool> => (3, "optional bool", "Is powered; V2.0.0 §2.2.30, field 3 (raw protocol value)."),
    }
    DartCommand, "2.2.31" {
        target_id: Option<u32> => (1, "optional uint32", "Target id; V2.0.0 §2.2.31, field 1 (raw protocol value)."),
        open: Option<bool> => (2, "optional bool", "Open; V2.0.0 §2.2.31, field 2 (raw protocol value)."),
        launch_confirm: Option<bool> => (3, "optional bool", "Launch confirm; V2.0.0 §2.2.31, field 3 (raw protocol value)."),
    }
    DartSelectTargetStatusSync, "2.2.32" {
        target_id: Option<u32> => (1, "optional uint32", "Target id; V2.0.0 §2.2.32, field 1 (raw protocol value)."),
        open: Option<u32> => (2, "optional uint32", "Gate status 0 closed, 1 opening, 2 open; table uint32 replaces invalid uint."),
    }
    SentryCtrlCommand, "2.2.33" {
        command_id: Option<u32> => (1, "optional uint32", "Command id; V2.0.0 §2.2.33, field 1 (raw protocol value)."),
    }
    SentryCtrlResult, "2.2.34" {
        command_id: Option<u32> => (1, "optional uint32", "Command id; V2.0.0 §2.2.34, field 1 (raw protocol value)."),
        result_code: Option<u32> => (2, "optional uint32", "Result code; V2.0.0 §2.2.34, field 2 (raw protocol value)."),
    }
    AirSupportCommand, "2.2.35" {
        command_id: Option<u32> => (1, "optional uint32", "Command id; V2.0.0 §2.2.35, field 1 (raw protocol value)."),
    }
    AirSupportStatusSync, "2.2.36" {
        airsupport_status: Option<u32> => (1, "optional uint32", "Airsupport status; V2.0.0 §2.2.36, field 1 (raw protocol value)."),
        left_time: Option<u32> => (2, "optional uint32", "Left time; V2.0.0 §2.2.36, field 2 (raw protocol value)."),
        cost_gold_coin: Option<u32> => (3, "optional uint32", "Cost gold coin; V2.0.0 §2.2.36, field 3 (raw protocol value)."),
        is_being_targeted: Option<u32> => (4, "optional uint32", "Is being targeted; V2.0.0 §2.2.36, field 4 (raw protocol value)."),
        shooter_status: Option<u32> => (5, "optional uint32", "Shooter status; V2.0.0 §2.2.36, field 5 (raw protocol value)."),
    }
}

/// Printed §2.2.1 declaration name; the Table 2-1 topic is KeyboardMouseControl.
pub type RemoteControl = KeyboardMouseControl;
/// Printed §2.2.32 declaration name; Table 2-1 calls the topic DartSelectTargetStatusSync.
pub type DartSelectControlStatusSync = DartSelectTargetStatusSync;

impl Message {
    /// Check documented bounded arrays/byte blocks and finite floating-point
    /// measurements. Unknown numeric enum values remain available for fault tests.
    /// Empty repeated fields represent unconfigured templates, not complete reports.
    pub fn validate(&self) -> Result<(), &'static str> {
        match self {
            Self::CustomControl(v) if v.data.as_ref().is_some_and(|b| b.len() > 30) => {
                Err("custom control exceeds 30 bytes")
            }
            Self::CustomByteBlock(v) if v.data.len() > 300 => {
                Err("custom byte block exceeds 300 bytes")
            }
            Self::GlobalSpecialMechanism(v)
                if v.mechanism_id.len() != v.mechanism_time_sec.len() =>
            {
                Err("mechanism IDs/times must pair")
            }
            Self::GlobalUnitStatus(v)
                if !v.robot_health.is_empty() && v.robot_health.len() != 10 =>
            {
                Err("health array requires ten entries")
            }
            Self::RadarInfoToClient(v) if !v.radar_info.is_empty() && v.radar_info.len() != 12 => {
                Err("radar array requires twelve entries")
            }
            Self::RobotPathPlanInfo(v)
                if (!v.offset_x.is_empty() || !v.offset_y.is_empty())
                    && (v.offset_x.len() != 49
                        || v.offset_y.len() != 49
                        || v.offset_x
                            .iter()
                            .chain(&v.offset_y)
                            .any(|x| !(-128..=127).contains(x))) =>
            {
                Err("path requires 49 paired int8 increments")
            }
            Self::MapClickInfo(v) => validate_map(v.robot_id.as_deref(), v.map_x, v.map_y),
            Self::MapClickCmd(v) => validate_map(v.robot_id.as_deref(), v.map_x, v.map_y),
            Self::RobotPosition(v) => finite(&[v.x, v.y, v.z, v.yaw]),
            Self::RobotStaticStatus(v) => finite(&[v.heat_cooldown_rate]),
            Self::RobotDynamicStatus(v) => finite(&[v.current_heat, v.last_projectile_firing_rate]),
            Self::RuneStatusSync(v) => finite(&[v.average_rings]),
            _ => Ok(()),
        }
    }
}
fn finite(values: &[Option<f32>]) -> Result<(), &'static str> {
    if values.iter().flatten().all(|x| x.is_finite()) {
        Ok(())
    } else {
        Err("nonfinite custom-client measurement")
    }
}
fn validate_map(ids: Option<&[u8]>, x: Option<f32>, y: Option<f32>) -> Result<(), &'static str> {
    if ids.is_some_and(|ids| ids.len() != 7) {
        return Err("map recipients require seven bytes");
    }
    finite(&[x, y])
}
