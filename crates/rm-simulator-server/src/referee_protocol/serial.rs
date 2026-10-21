// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Logical serial payloads, V2.0.0 Tables 1-6–1-49. Bitfields retain their raw
//! words, including reserved bits. No packed layout, byte codec or CRC is implied.
use serde::{Deserialize, Serialize};

/// Serial framing fields, V2.0.0 Tables 1-2–1-4; CRC is caller supplied.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FrameHeader {
    /// Start byte, specified as 0xA5.
    pub sof: u8,
    /// Payload length in bytes, excluding command ID and framing.
    pub data_length: u16,
    /// Wrapping frame sequence.
    pub sequence: u8,
    /// Caller-computed header CRC8.
    pub crc8: u8,
}
impl Default for FrameHeader {
    fn default() -> Self {
        Self {
            sof: 0xa5,
            data_length: 0,
            sequence: 0,
            crc8: 0,
        }
    }
}
/// Serial command metadata; overview lengths are not silently treated as normative.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Descriptor {
    /// Top-level UART command ID.
    pub command_id: u16,
    /// Logical payload name.
    pub name: &'static str,
    /// Detailed table in the supplied manual.
    pub source: &'static str,
    /// Length printed in overview Table 1-5; can conflict with the detailed table.
    pub overview_len: usize,
    /// Detailed field-table length, or `None` for variable-length interaction data.
    pub modeled_len: Option<usize>,
}
macro_rules! payloads {
    ($($name:ident, $id:literal, $overview:literal, $length:expr, $source:literal {
        $($field:ident: $ty:ty = $default:expr => $doc:literal),* $(,)?
    })*) => {
        $(#[doc = concat!("Logical payload for ", stringify!($id), ", V2.0.0 Table ", $source, ".")]
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        pub struct $name { $(#[doc = $doc] pub $field: $ty,)* }
        impl Default for $name { fn default() -> Self { Self { $($field: $default,)* } } }
        impl $name {
            /// Source identity and lengths, independent of Rust memory layout.
            pub const DESCRIPTOR: Descriptor = Descriptor { command_id: $id, name: stringify!($name), source: $source,
                overview_len: $overview, modeled_len: $length };
        })*
        /// Every top-level serial command in V2.0.0 Table 1-5.
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        pub enum Message { $(#[doc = concat!("Table ", $source, ".")] $name($name),)* }
        impl Message {
            /// Descriptor of the contained payload.
            pub fn descriptor(&self) -> Descriptor { match self { $(Self::$name(_) => $name::DESCRIPTOR,)* } }
            /// Complete synthetic catalog, using zero-valued fixed-size payloads.
            pub fn templates() -> Vec<Self> { vec![$(Self::$name($name::default()),)*] }
        }
        /// Complete serial command catalog, ordered as Table 1-5.
        pub const CATALOG: &[Descriptor] = &[$($name::DESCRIPTOR,)*];
    };
}

/// One centimetre-resolution map position, Tables 1-36/1-44.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PositionCm {
    /// Protocol x coordinate in centimetres.
    pub x_cm: u16,
    /// Protocol y coordinate in centimetres.
    pub y_cm: u16,
}
/// Buff record shared by the wireless report (7 bytes each), Table 1-48.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct WirelessBuff {
    /// Maximum-HP recovery percentage per second.
    pub recovery_percent_per_s: u8,
    /// Added heat cooling per second.
    pub cooling_per_s: u16,
    /// Defense buff in percent.
    pub defense_percent: u8,
    /// Vulnerability in percent.
    pub vulnerability_percent: u8,
    /// Attack buff in percent.
    pub attack_percent: u16,
}
payloads! {
    GameStatus, 0x0001, 11, Some(11), "1-6" {
        kind_and_progress: u8 = 0 => "Competition type in bits 0–3; phase in bits 4–7.",
        stage_remain_time_s: u16 = 0 => "Remaining phase duration in seconds.",
        sync_timestamp: u64 = 0 => "Raw NTP/UNIX synchronization timestamp; epoch and units must be supplied by the adapter."
    }
    GameResult, 0x0002, 1, Some(1), "1-7" {
        winner: u8 = 0 => "Winner code: 0 draw, 1 red, 2 blue."
    }
    RobotHp, 0x0003, 20, Some(20), "1-8" {
        ally_1_hp: u16 = 0 => "Own Hero HP (compatibility robot number 1).",
        ally_2_hp: u16 = 0 => "Own Engineer HP (compatibility robot number 2).",
        ally_3_hp: u16 = 0 => "Own Infantry 3 HP.",
        ally_4_hp: u16 = 0 => "Own Infantry 4 HP.",
        damage_difference: i16 = 0 => "Signed damage difference; normalized from the malformed C identifier.",
        ally_7_hp: u16 = 0 => "Own Sentry HP.",
        ally_outpost_hp: u16 = 0 => "Own Outpost HP.",
        ally_base_hp: u16 = 0 => "Own Base HP.",
        enemy_outpost_hp: u16 = 0 => "Opponent Outpost HP.",
        enemy_base_hp: u16 = 0 => "Opponent Base HP."
    }
    EventData, 0x0101, 4, Some(4), "1-9" {
        flags: u32 = 0 => "Raw battlefield event/occupation flags; Table 1-9 defines each bit."
    }
    RefereeWarning, 0x0104, 3, Some(3), "1-10" {
        level: u8 = 0 => "Penalty level code.",
        offending_robot_id: u8 = 0 => "Penalized referee robot ID.",
        count: u8 = 0 => "Penalty count."
    }
    DartInfo, 0x0105, 3, Some(3), "1-11" {
        remaining_time_s: u8 = 0 => "Dart launch remaining time in seconds.",
        flags: u16 = 0 => "Raw Dart information bitfield, Table 1-11."
    }
    RobotStatus, 0x0201, 17, Some(17), "1-12" {
        robot_id: u8 = 0 => "Official referee ID, not a simulator chassis ID.",
        robot_level: u8 = 0 => "Performance level.",
        current_hp: u16 = 0 => "Current HP.",
        maximum_hp: u16 = 0 => "Maximum HP.",
        shooter_barrel_cooling_per_s: u16 = 0 => "Barrel heat cooling per second.",
        shooter_barrel_heat_limit: u16 = 0 => "Barrel heat limit.",
        chassis_power_limit_w: u16 = 0 => "Chassis power limit in watts.",
        bullet_speed_limit_m_s: f32 = 0. => "Maximum projectile launching speed in m/s.",
        power_outputs: u8 = 0 => "Bits 0/1/2 enable gimbal/chassis/shooter; reserved bits retained."
    }
    PowerHeat, 0x0202, 14, Some(14), "1-13" {
        reserved_0: u16 = 0 => "Reserved bytes 0–1.",
        reserved_2: u16 = 0 => "Reserved bytes 2–3.",
        reserved_4_bits: u32 = 0 => "Reserved bytes 4–7 as raw bits, not a fabricated power measurement.",
        buffer_energy_j: u16 = 0 => "Remaining buffer energy in joules.",
        shooter_17mm_heat: u16 = 0 => "17 mm barrel heat.",
        shooter_42mm_heat: u16 = 0 => "42 mm barrel heat."
    }
    RobotPosition, 0x0203, 16, Some(12), "1-14" {
        x_m: f32 = 0. => "Protocol-frame x in metres.",
        y_m: f32 = 0. => "Protocol-frame y in metres.",
        angle_deg: f32 = 0. => "Speed Monitor heading in degrees, zero true north; no invented fourth float."
    }
    Buff, 0x0204, 8, Some(8), "1-15" {
        recovery_percent_per_s: u8 = 0 => "Maximum-HP recovery percentage per second.",
        cooling_per_s: u16 = 0 => "Added heat cooling per second.",
        defense_percent: u8 = 0 => "Defense buff in percent.",
        vulnerability_percent: u8 = 0 => "Vulnerability in percent.",
        attack_percent: u16 = 0 => "Attack buff in percent.",
        remaining_energy: u8 = 0 => "Protocol chassis-energy value; retain the Table 1-15 encoding."
    }
    Hurt, 0x0206, 1, Some(1), "1-16" {
        armor_and_reason: u8 = 0 => "Armor ID in bits 0–3; HP-deduction reason in bits 4–7."
    }
    Shoot, 0x0207, 7, Some(7), "1-17" {
        projectile_type: u8 = 0 => "Projectile class code.",
        shooter_number: u8 = 0 => "Shooter identity code.",
        launching_frequency_hz: u8 = 0 => "Projectile launching rate in Hz.",
        projectile_speed_m_s: f32 = 0. => "Measured projectile speed in m/s."
    }
    ProjectileAllowance, 0x0208, 8, Some(8), "1-18" {
        rounds_17mm: u16 = 0 => "Available 17 mm projectiles.",
        rounds_42mm: u16 = 0 => "Available 42 mm projectiles.",
        remaining_gold: u16 = 0 => "Remaining team gold coins.",
        fortress_rounds: u16 = 0 => "Fortress reserve 17 mm allowance."
    }
    RfidStatus, 0x0209, 5, Some(5), "1-19" {
        flags: u32 = 0 => "RFID status bits 0–31; exact definitions in Table 1-19.",
        flags_2: u8 = 0 => "Additional RFID bits, Table 1-19."
    }
    DartClientCommand, 0x020a, 6, Some(6), "1-20" {
        opening_status: u8 = 0 => "Dart launch opening status code.",
        reserved: u8 = 0 => "Reserved byte.",
        target_change_time_s: u16 = 0 => "Latest target change time in remaining-match seconds.",
        latest_launch_cmd_time_s: u16 = 0 => "Latest launch command time in remaining-match seconds."
    }
    GroundRobotPositions, 0x020b, 40, Some(40), "1-21" {
        positions_m: [[f32; 2]; 4] = [[0.; 2]; 4] => "Hero, Engineer, Infantry 3 and Infantry 4 x/y coordinates in metres.",
        reserved: [u32; 2] = [0; 2] => "Two reserved 4-byte words; raw bits retained."
    }
    RadarMark, 0x020c, 2, Some(2), "1-22" {
        tracking_progress: u16 = 0 => "Packed tracking progress/status bitfield, Table 1-22."
    }
    SentryInfo, 0x020d, 14, Some(14), "1-23" {
        flags: u32 = 0 => "Primary Sentry decision bitfield.",
        flags_2: u16 = 0 => "Secondary Sentry decision bitfield.",
        flags_3: u64 = 0 => "Extended Sentry decision bitfield."
    }
    RadarInfo, 0x020e, 1, Some(1), "1-24" {
        flags: u8 = 0 => "Radar decision bitfield."
    }
    RobotInteraction, 0x0301, 118, None, "1-25" {
        sender_id: u16 = 0 => "Official sender ID.",
        receiver_id: u16 = 0 => "Official receiver ID.",
        data: InteractionData = InteractionData::default() => "Typed interaction content, including its subcommand ID."
    }
    CustomControllerRobot, 0x0302, 30, Some(30), "1-39" {
        data: [u8; 30] = [0; 30] => "Thirty custom-controller bytes."
    }
    MapCommand, 0x0303, 15, Some(12), "1-35" {
        target_x_m: f32 = 0. => "Selected protocol-frame location x in metres.",
        target_y_m: f32 = 0. => "Selected protocol-frame location y in metres.",
        keyboard: u8 = 0 => "General keyboard code, zero when absent.",
        enemy_robot_id: u8 = 0 => "Selected opponent robot ID, zero for location commands.",
        source_id: u16 = 0 => "Official information-source ID; no invented trailing three bytes."
    }
    MapRobotData, 0x0305, 48, Some(48), "1-36" {
        positions: [PositionCm; 12] = [PositionCm::default(); 12] => "Opponent 1/2/3/4/6/7 then own 1/2/3/4/6/7, in centimetres."
    }
    CustomClientInput, 0x0306, 8, Some(8), "1-43" {
        keys: u16 = 0 => "Two general key codes in low/high bytes.",
        x_and_left: u16 = 0 => "Mouse x pixels in bits 0–11; left-button state in bits 12–15.",
        y_and_right: u16 = 0 => "Mouse y pixels in bits 0–11; right-button state in bits 12–15.",
        reserved: u16 = 0 => "Reserved final word."
    }
    MapPath, 0x0307, 103, Some(105), "1-37" {
        intention: u8 = 0 => "Intention code: 1 attack, 2 defend, 3 move.",
        start_x_dm: u16 = 0 => "Starting protocol-frame x in decimetres.",
        start_y_dm: u16 = 0 => "Starting protocol-frame y in decimetres.",
        delta_x_dm: Vec<i8> = vec![0; 49] => "Exactly 49 successive signed x increments in decimetres.",
        delta_y_dm: Vec<i8> = vec![0; 49] => "Exactly 49 successive signed y increments in decimetres.",
        sender_id: u16 = 0 => "Official sender ID; detailed table includes these final two bytes."
    }
    CustomInfo, 0x0308, 34, Some(34), "1-38" {
        sender_id: u16 = 0 => "Official robot sender ID.",
        receiver_id: u16 = 0 => "Official client receiver ID.",
        data: [u8; 30] = [0; 30] => "Thirty custom display bytes."
    }
    RobotCustomData, 0x0309, 30, Some(30), "1-40" {
        data: [u8; 30] = [0; 30] => "Thirty custom robot-to-controller bytes."
    }
    RobotCustomData2, 0x0310, 300, Some(300), "1-41" {
        data: Vec<u8> = vec![0; 300] => "Exactly 300 robot-to-custom-client bytes."
    }
    RobotCustomData3, 0x0311, 30, Some(30), "1-42" {
        data: [u8; 30] = [0; 30] => "Thirty custom-client-to-robot bytes."
    }
    WirelessPositions, 0x0a01, 24, Some(24), "1-44" {
        positions: [PositionCm; 6] = [PositionCm::default(); 6] => "Opponent 1/2/3/4/6/7 positions in centimetres."
    }
    WirelessHp, 0x0a02, 12, Some(12), "1-45" {
        hp: [u16; 6] = [0; 6] => "Opponent 1/2/3/4, reserved, 7 HP."
    }
    WirelessAllowance, 0x0a03, 10, Some(10), "1-46" {
        rounds: [u16; 5] = [0; 5] => "Opponent 1/3/4/6/7 projectile allowance."
    }
    WirelessTeamStatus, 0x0a04, 8, Some(8), "1-47" {
        remaining_gold: u16 = 0 => "Opponent remaining gold.",
        accumulated_gold: u16 = 0 => "Opponent accumulated gold.",
        flags: u32 = 0 => "Opponent occupation and traversal flags."
    }
    WirelessBuffs, 0x0a05, 41, Some(41), "1-48" {
        buffs: [WirelessBuff; 5] = [WirelessBuff::default(); 5] => "Opponent 1/2/3/4/7 buffs, seven bytes per record.",
        sentry_posture: u8 = 0 => "Opponent Sentry posture/strengthening code.",
        robot_status: [u8; 5] = [0; 5] => "Opponent 1/2/3/4/7 alive/defeated/invincibility status codes."
    }
    WirelessKey, 0x0a06, 6, Some(6), "1-49" {
        ascii: [u8; 6] = [0; 6] => "Six ASCII letters/digits of the interference-wave key; zero template is unconfigured."
    }
}

/// One drawing figure, Table 1-27. Fields are logical values, not C bitfield layout.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Figure {
    /// Three-byte figure name.
    pub name: [u8; 3],
    /// Operation, 3 bits.
    pub operate_type: u8,
    /// Figure type, 3 bits.
    pub figure_type: u8,
    /// Layer, 4 bits.
    pub layer: u8,
    /// Color, 4 bits.
    pub color: u8,
    /// Type-dependent detail A, 9 bits.
    pub details_a: u16,
    /// Type-dependent detail B, 9 bits.
    pub details_b: u16,
    /// Line width in pixels, 10 bits.
    pub width: u16,
    /// Starting x in pixels, 11 bits.
    pub start_x: u16,
    /// Starting y in pixels, 11 bits.
    pub start_y: u16,
    /// Type-dependent detail C, 10 bits.
    pub details_c: u16,
    /// Type-dependent detail D, 11 bits.
    pub details_d: u16,
    /// Type-dependent detail E, 11 bits.
    pub details_e: u16,
}
impl Figure {
    /// Reject values that cannot fit the Table 1-27 bitfields.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.operate_type > 7
            || self.figure_type > 7
            || self.layer > 15
            || self.color > 15
            || self.details_a > 511
            || self.details_b > 511
            || self.width > 1023
            || self.start_x > 2047
            || self.start_y > 2047
            || self.details_c > 1023
            || self.details_d > 2047
            || self.details_e > 2047
        {
            Err("figure value exceeds protocol bit width")
        } else {
            Ok(())
        }
    }
}
/// Every robot-interaction subcontent family, Table 1-25 and Tables 1-26–1-34.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum InteractionData {
    /// 0x0100, Table 1-26.
    LayerDelete {
        /// Delete operation code.
        delete_type: u8,
        /// Layer to delete.
        layer: u8,
    },
    /// 0x0101, Table 1-27.
    Figure(Figure),
    /// 0x0102, Table 1-29.
    Figures2([Figure; 2]),
    /// 0x0103, Table 1-30.
    Figures5([Figure; 5]),
    /// 0x0104, Table 1-31.
    Figures7([Figure; 7]),
    /// 0x0110, Table 1-32.
    Character {
        /// Character figure parameters.
        figure: Figure,
        /// Thirty raw character bytes; encoding is caller-owned.
        data: [u8; 30],
    },
    /// 0x0120, Table 1-33.
    SentryCommand {
        /// Complete decision bitfield; reserved bits retained.
        flags: u32,
    },
    /// 0x0121, Table 1-34.
    RadarCommand {
        /// Radar decision byte.
        command: u8,
        /// Password control byte.
        password_command: u8,
        /// Six password bytes.
        password: [u8; 6],
    },
    /// 0x0200–0x02ff, Table 1-25, up to 112 custom bytes.
    RobotData {
        /// Application-defined subcontent ID in 0x0200–0x02ff.
        content_id: u16,
        /// Up to 112 custom bytes, excluding the six-byte interaction header.
        data: Vec<u8>,
    },
}
impl Default for InteractionData {
    fn default() -> Self {
        Self::RobotData {
            content_id: 0x0200,
            data: vec![],
        }
    }
}
impl InteractionData {
    /// Subcontent ID for the interaction header.
    pub fn content_id(&self) -> u16 {
        match self {
            Self::LayerDelete { .. } => 0x0100,
            Self::Figure(_) => 0x0101,
            Self::Figures2(_) => 0x0102,
            Self::Figures5(_) => 0x0103,
            Self::Figures7(_) => 0x0104,
            Self::Character { .. } => 0x0110,
            Self::SentryCommand { .. } => 0x0120,
            Self::RadarCommand { .. } => 0x0121,
            Self::RobotData { content_id, .. } => *content_id,
        }
    }
    /// Content length in bytes, excluding the six-byte interaction header.
    pub fn data_len(&self) -> usize {
        match self {
            Self::LayerDelete { .. } => 2,
            Self::Figure(_) => 15,
            Self::Figures2(_) => 30,
            Self::Figures5(_) => 75,
            Self::Figures7(_) => 105,
            Self::Character { .. } => 45,
            Self::SentryCommand { .. } => 4,
            Self::RadarCommand { .. } => 8,
            Self::RobotData { data, .. } => data.len(),
        }
    }
    /// Check bounded custom data, user subcontent IDs and figure bit widths.
    pub fn validate(&self) -> Result<(), &'static str> {
        let figures: &[Figure] = match self {
            Self::Figure(f) | Self::Character { figure: f, .. } => std::slice::from_ref(f),
            Self::Figures2(f) => f,
            Self::Figures5(f) => f,
            Self::Figures7(f) => f,
            Self::RobotData { content_id, data } => {
                if !(0x0200..=0x02ff).contains(content_id) || data.len() > 112 {
                    return Err("invalid inter-robot subcontent ID or payload length");
                }
                &[]
            }
            _ => &[],
        };
        for figure in figures {
            figure.validate()?;
        }
        Ok(())
    }
    /// One synthetic template per interaction family (not all 256 application IDs).
    pub fn templates() -> Vec<Self> {
        vec![
            Self::LayerDelete {
                delete_type: 0,
                layer: 0,
            },
            Self::Figure(Figure::default()),
            Self::Figures2([Figure::default(); 2]),
            Self::Figures5([Figure::default(); 5]),
            Self::Figures7([Figure::default(); 7]),
            Self::Character {
                figure: Figure::default(),
                data: [0; 30],
            },
            Self::SentryCommand { flags: 0 },
            Self::RadarCommand {
                command: 0,
                password_command: 0,
                password: [0; 6],
            },
            Self::default(),
        ]
    }
}
impl Message {
    /// Check lengths, finite measurements and interaction bit widths. This is
    /// structural validation, not robot permission, ID, cadence or gameplay admission.
    pub fn validate(&self) -> Result<(), &'static str> {
        let finite: &[f32] = match self {
            Self::RobotStatus(v) => std::slice::from_ref(&v.bullet_speed_limit_m_s),
            Self::Shoot(v) => std::slice::from_ref(&v.projectile_speed_m_s),
            Self::GroundRobotPositions(v) => v.positions_m.as_flattened(),
            _ => &[],
        };
        if !finite.iter().all(|x| x.is_finite()) {
            return Err("nonfinite referee measurement");
        }
        match self {
            Self::RobotPosition(v)
                if ![v.x_m, v.y_m, v.angle_deg].into_iter().all(f32::is_finite) =>
            {
                Err("nonfinite position")
            }
            Self::MapCommand(v)
                if ![v.target_x_m, v.target_y_m].into_iter().all(f32::is_finite) =>
            {
                Err("nonfinite map command")
            }
            Self::MapPath(v) if v.delta_x_dm.len() != 49 || v.delta_y_dm.len() != 49 => {
                Err("path requires 49 paired increments")
            }
            Self::RobotCustomData2(v) if v.data.len() != 300 => Err("0x0310 requires 300 bytes"),
            Self::RobotInteraction(v) => v.data.validate(),
            _ => Ok(()),
        }
    }
}

/// Read a raw protocol bitfield without assuming a compiler's C bitfield layout.
///
/// ```
/// use rm_simulator_server::referee_protocol::serial::bits;
/// assert_eq!(bits(0x42, 4, 4)?, 4);
/// # Ok::<(), &'static str>(())
/// ```
pub fn bits(word: u64, offset: u8, width: u8) -> Result<u64, &'static str> {
    if width == 0 || width > 64 || u16::from(offset) + u16::from(width) > 64 {
        return Err("invalid bit range");
    }
    let mask = u64::MAX >> (64 - width);
    Ok((word >> offset) & mask)
}
/// Replace only the selected bits, rejecting overflow and retaining reserved bits.
pub fn with_bits(word: u64, offset: u8, width: u8, value: u64) -> Result<u64, &'static str> {
    bits(word, offset, width)?;
    let mask = u64::MAX >> (64 - width);
    if value > mask {
        return Err("bitfield value exceeds width");
    }
    Ok((word & !(mask << offset)) | (value << offset))
}
