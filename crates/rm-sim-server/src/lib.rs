// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared client wire components and an optional native simulation host.
//!
//! The default `native-host` feature provides sockets, host workers and file-based
//! field construction. Disable default features for a caller-driven client codec
//! without those services. This is a library boundary, not a browser application.
//!
//! Native hosting loads verified CAD into a `Field`, paces a `Simulation` on a
//! host clock, carries the protocol over GNS UDP and serves a referee HTTP panel.
//! Nothing here draws; the `rm-simulator` app is one client of this crate,
//! whether it hosts the simulation in-process or connects to
//! `rm-sim-server`.
#![deny(missing_docs)]

extern crate self as rm_sim_server;
#[cfg(feature = "native-host")]
mod assembly;
#[cfg(feature = "native-host")]
mod assembly_rig;
#[cfg(test)]
#[cfg(feature = "native-host")]
mod bandwidth_probe;
pub mod binary_snapshot;
pub mod cad_assets;
pub mod clock;
pub use rm_sim_assets::collision_mesh;
pub mod compression;
pub mod deployment;
pub mod external_control;
#[cfg(feature = "native-host")]
pub mod host;
pub mod host_args;
#[cfg(feature = "native-host")]
pub mod http;
pub mod layout;
#[cfg(feature = "native-host")]
mod lifecycle;
pub mod map_interaction;
pub use rm_sim_assets::math;
#[cfg(feature = "native-host")]
pub mod navigation;
/// Caller-visible navigation diagnostics without native planning workers.
#[cfg(not(feature = "native-host"))]
pub mod navigation {
    pub use rm_sim_control::navigation::{Bake, CELL_M, Phase, RADIUS_M, View};
}
#[cfg(feature = "native-host")]
pub mod net;
pub mod protocol;
#[cfg(feature = "native-host")]
pub mod referee_protocol;
pub mod rmuc_2027;
pub mod roster;
#[cfg(feature = "native-host")]
pub mod sensor_stream;
#[cfg(feature = "native-host")]
pub mod simulation;
pub mod snapshot_codec;
pub mod zone_catalog;

pub use rm_sim_assets::semantics;

pub mod prediction;

pub mod view;

#[cfg(feature = "native-host")]
pub mod workload;

pub mod input_stream;
pub mod intelligence;
pub mod sentry_controller;

pub mod network_stats;

pub mod network_trace;

pub mod owner_stream;

pub mod pacing;

pub mod scripted_link;

pub mod udp_codec;

pub mod udp_snapshot;

pub mod lobby;

pub mod competition;
pub mod sentry_view;
/// Published state and control identities, without a simulation owner.
pub mod state;

#[cfg(feature = "native-host")]
pub mod websocket;

#[cfg(not(feature = "native-host"))]
#[path = "presentation_clock.rs"]
pub mod presentation_clock;
