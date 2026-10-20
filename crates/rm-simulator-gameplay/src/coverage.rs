// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared observation identifiers retained by serialized diagnostic records.
//! `External` requires an authoritative caller input. `Tracked` records state
//! only. Neither label claims that the interactive simulator enforces the rule.
use serde::{Deserialize, Serialize};

/// Mechanism groups the coverage register accounts for (sections 5.1 to 5.8
/// and sections 6 to 9).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Mechanic {
    /// Setup, initialization, countdown, round, confirmation and match result
    /// (sections 6.3 to 6.9).
    MatchLifecycle,
    /// Round comparison and match progression (section 5.8).
    Victory,
    /// Detected raw damage, attribution, defenses, HP and attack totals
    /// (sections 5.1.1 and 5.5.3.1).
    Damage,
    /// Measured launch speed against the configured limit and its launch locks
    /// (section 5.1.2, Table 5-3).
    LaunchSpeed,
    /// Heat per shot, 10 Hz cooling and temporary and permanent launch locks
    /// (section 5.1.3, Figure 5-1).
    Heat,
    /// Power telemetry. Buffer energy and chassis power-off are not enforced
    /// (section 5.1.4).
    Power,
    /// Irregular disconnection, which blocks recovery and launch
    /// (sections 5.1.5.1 to 5.1.5.5).
    Disconnection,
    /// Explicit penalty damage and ejection. Card escalation is not automatic
    /// (section 5.1.6, sections 7.1 to 7.3).
    Penalties,
    /// Resupply healing and remote HP purchase (section 5.2.1).
    Recovery,
    /// Respawn timers, accelerated progress, paid respawn, weakness and
    /// invincibility (section 5.2.2).
    Respawn,
    /// Scheduled gold, spending and refunds (section 5.3.1, Tables 5-5, 5-6).
    Economy,
    /// Initial allowance, exchange limits and delays, consumption and sentry
    /// resupply (section 5.3.2, Tables 5-7 to 5-9).
    Allowance,
    /// Certified assembly completions and their rewards (section 5.3.3).
    Assembly,
    /// Explicit and shot experience awards with capped level thresholds
    /// (section 5.4.1, Table 5-11).
    Experience,
    /// Caller-supplied HP, heat and cooling. Level-up does not select
    /// performance tables (section 5.4.2, Tables 5-12 to 5-15).
    Performance,
    /// Base HP, shield, outpost protection, cumulative loss and rebuild
    /// opportunities (section 5.5.1).
    Base,
    /// Outpost damage and RFID rebuilding (section 5.5.1).
    Outpost,
    /// Rune activation and opportunities, owned by the world referee
    /// (section 5.5.2).
    Rune,
    /// Own-base defense, exchange and weakened-state removal (section 5.5.3.2).
    BaseZone,
    /// Central highland occupation (section 5.5.3.3).
    CentralHighland,
    /// Trapezoid highland occupation (section 5.5.3.4).
    TrapezoidHighland,
    /// Road, elevated ground, ramp and tunnel contacts (section 5.5.3.5).
    TerrainCrossing,
    /// Own-outpost exchange and weakness removal, plus the rebuild scan
    /// (section 5.5.3.6).
    OutpostZone,
    /// Assembly zone occupation (section 5.5.3.7).
    AssemblyZone,
    /// Healing, exchanges, sentry allowance and respawn acceleration at
    /// resupply (section 5.5.3.8).
    ResupplyZone,
    /// Fortress occupation (section 5.5.3.9).
    Fortress,
    /// Hero deployment and its speed report thresholds (section 5.6.1).
    HeroDeployment,
    /// Engineer special state (section 5.6.2).
    Engineer,
    /// Drone air-support time, grants, paid continuation and firing gate
    /// (section 5.6.3).
    Drone,
    /// Sentry pose state (section 5.6.4).
    Sentry,
    /// Dart launch windows, target selection and hits (section 5.6.5).
    Dart,
    /// Marks, vulnerability, drone counter and decoding (section 5.6.6,
    /// Appendix 1).
    Radar,
    /// Chassis energy expenditure and recharge in joules (section 5.6.7).
    Energy,
    /// Inspection and staging decisions (sections 6.1 and 6.2).
    Inspection,
    /// Timeout decisions (sections 6.3.1 and 6.3.2).
    Timeouts,
    /// Referee irregularity decisions (section 8).
    Irregularities,
    /// Appeal status and decisions (section 9).
    Appeals,
    /// Library snapshots and commands for custom clients (section 5.7).
    CustomClient,
}
