// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Transport-independent referee data from the supplied 2026 University Series
//! Communication Protocol V2.0.0 (2026-06-30). This is a compatibility schema,
//! not a 2026 gameplay engine. See `docs/referee-link.md` for source conflicts.
//!
//! Rust layout and serde JSON are **not** the UART or Protobuf wire format.
//! Callers own encoding, framing, CRC, transport, cadence and visibility policy.
//! Templates are synthetic; live projections expose only documented own-robot
//! telemetry and leave unsupported custom-client fields absent.
pub mod client;
pub mod projection;
pub mod serial;

/// Exact communication schema identity, independent of the live game's season.
pub const SCHEMA: &str = "RoboMaster University Series Communication Protocol V2.0.0 (2026-06-30)";

/// A caller-configurable collection covering every top-level message and UI submessage.
/// No template implies that the corresponding gameplay mechanism is simulated.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MockData {
    /// One zero-valued, structurally valid payload per serial command ID.
    pub serial: Vec<serial::Message>,
    /// One custom-client message per topic; optional fields start absent.
    pub client: Vec<client::Message>,
    /// Every 0x0301 interaction subtype, including bounded inter-robot data.
    pub interaction: Vec<serial::InteractionData>,
}
impl Default for MockData {
    fn default() -> Self {
        Self {
            serial: serial::Message::templates(),
            client: client::Message::templates(),
            interaction: serial::InteractionData::templates(),
        }
    }
}

impl MockData {
    /// Check payload bounds before handing a configured dataset to a transport.
    /// No rate scheduling, command execution or semantic admission is performed.
    ///
    /// ```
    /// use rm_sim_server::referee_protocol::{MockData, client};
    /// let mut mock = MockData::default();
    /// for message in &mut mock.client {
    ///     if let client::Message::RobotDynamicStatus(status) = message {
    ///         status.current_health = Some(200);
    ///     }
    /// }
    /// mock.validate()?;
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn validate(&self) -> Result<(), &'static str> {
        for data in &self.serial {
            data.validate()?;
        }
        for data in &self.client {
            data.validate()?;
        }
        for data in &self.interaction {
            data.validate()?;
        }
        Ok(())
    }
}
/// Custom-client HEVC fragment header, §2 (eight wire bytes).
/// This logical DTO does not fragment, encode video or guarantee Rust layout.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct VideoHeader {
    /// Wrapping video frame number.
    pub frame_number: u16,
    /// Fragment index within this frame.
    pub fragment_index: u16,
    /// Total encoded frame length in bytes.
    pub total_frame_bytes: u32,
}

#[cfg(test)]
mod tests;
