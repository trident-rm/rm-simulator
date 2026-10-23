// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Typed map intents corresponding to 2026 communication §1.3.1/Table 1-35 (0x0303).
//! Also models MapClickCmd/MapClickInfo (§§2.2.17–2.2.18) marker and recipient selection.
//! Positions remain simulator world FLU metres; this is not a referee serial frame.
use rm_sim_world::Team;
use serde::{Deserialize, Serialize};

/// Logical destination; a Sentry need not be deployed to record an intent.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Recipient {
    /// The sender's team Sentry, including a future autonomous controller.
    Sentry,
    /// One currently deployed own-team chassis id.
    Robot(u32),
    /// Up to seven explicit own-team chassis ids (MapClickCmd's fixed recipient list).
    Selected([Option<u32>; 7]),
    /// Broadcast, matching MapClickCmd is_send_all = 1 or 2.
    Team {
        /// Whether Sentry is included; false corresponds to is_send_all = 1.
        include_sentry: bool,
    },
}
/// Mutually exclusive target alternatives from Table 1-35.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Target {
    /// Position in simulator world FLU metres, not referee-corner coordinates.
    Position([f64; 2]),
    /// An opponent's stable simulator chassis id; resolved to robot id by the host.
    Opponent(u32),
}
/// Untrusted client request. Sender, team, source and event id come from the host.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Interaction {
    /// Logical own-team destination.
    pub recipient: Recipient,
    /// Coordinates or opponent id, never both.
    pub target: Target,
    /// Attack/defend/alert/custom marker, communication §§2.2.17–2.2.18.
    pub kind: crate::protocol::MapMarkerKind,
    /// Protocol general keyboard byte, zero for no key; interpreted by a future controller.
    pub key_code: u8,
}
impl Interaction {
    /// Reject non-finite/off-field positions before consulting the host roster.
    ///
    /// ```
    /// use rm_sim_server::map_interaction::{Interaction, Recipient, Target};
    /// assert!(Interaction { recipient: Recipient::Sentry,
    ///     target: Target::Position([2., -1.]), kind: rm_sim_server::protocol::MapMarkerKind::Attack, key_code: 0 }.validate().is_ok());
    /// ```
    pub fn validate(self) -> Result<(), &'static str> {
        if !self.kind.is_valid() {
            return Err("invalid interaction marker");
        }
        if let crate::protocol::MapMarkerKind::Custom(c) = self.kind
            && !matches!(c, b'C'..=b'L' | b'N' | b'O' | b'Q'..=b'Z')
        {
            return Err("custom marker is not allowed by MapClickCmd");
        }
        if let Recipient::Selected(ids) = self.recipient {
            let mut unique = std::collections::BTreeSet::new();
            if ids.iter().flatten().any(|id| !unique.insert(*id)) || unique.is_empty() {
                return Err("selected recipient list is empty or duplicated");
            }
        }
        if let Target::Position(position) = self.target {
            crate::protocol::check_map_marker(crate::protocol::MapMarkerKind::Alert, position)?;
        }
        Ok(())
    }
}
/// Distinguishes the two source paths in communication §1.3.1.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Source {
    /// Team command interface; simulator policy grants this to other team pilots.
    TeamOperator,
    /// Operator of the corresponding semi-automatic Sentry.
    SentryOperator,
}
impl Source {
    /// Minimum transmission interval in host nanoseconds: §1.3.1, 0.5 s or 3 s.
    pub const fn interval_ns(self) -> u64 {
        match self {
            Self::TeamOperator => 500_000_000,
            Self::SentryOperator => 3_000_000_000,
        }
    }
}
/// Accepted intent, not an acknowledgement that a robot executed it.
/// Event ids permit consumers to ignore duplicate transport deliveries.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Event {
    /// Monotonically increasing identity for this host lifetime, starting at one.
    pub id: u64,
    /// Authenticated sender's simulator client id.
    pub client_id: u32,
    /// Authenticated sender's team; never supplied by the client.
    pub team: Team,
    /// Host-derived information source category.
    pub source: Source,
    /// 2026 Appendix II player-client id when defined; absent for the simulator Sentry seat and 2027 identities.
    pub source_id: Option<u16>,
    /// Original request in simulator coordinates/identities.
    pub interaction: Interaction,
    /// Own-team chassis ids resolved when accepted; empty for a not-yet-deployed Sentry.
    pub recipients: Vec<u32>,
    /// Opponent's Appendix II robot id (1–9 or 101–109), absent for coordinate targets or an unbound 2027 wire identity.
    pub opponent_robot_id: Option<u8>,
    /// Simulation time of acceptance in nanoseconds; cooldowns use the host transport clock.
    pub time_ns: u64,
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn map_click_custom_letter_and_selected_recipient_contract() {
        let mut interaction = Interaction {
            recipient: Recipient::Sentry,
            target: Target::Position([0., 0.]),
            kind: crate::protocol::MapMarkerKind::Custom(b'Q'),
            key_code: 0,
        };
        assert!(interaction.validate().is_ok());
        interaction.kind = crate::protocol::MapMarkerKind::Custom(b'P');
        assert!(interaction.validate().is_err());
        interaction.kind = crate::protocol::MapMarkerKind::Attack;
        interaction.recipient = Recipient::Selected([None; 7]);
        assert!(interaction.validate().is_err());
        interaction.recipient =
            Recipient::Selected([Some(1), Some(1), None, None, None, None, None]);
        assert!(interaction.validate().is_err());
        interaction.recipient =
            Recipient::Selected([Some(1), Some(2), None, None, None, None, None]);
        assert!(interaction.validate().is_ok());
    }
}
