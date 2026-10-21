// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Compare independent and shared checkpoint preparation for 1, 2 and 12 peers.
//! Release CPU-side encoding times include preparation once per broadcast and
//! exclude stepping, decoding and feedback. Every output byte and counter must
//! match across moving/firing states, staggered feedback and an epoch change.
//! No CAD assets, sockets, renderer or wall-clock pacing are involved.
use rm_simulator_server::{
    compression,
    protocol::Command,
    simulation::{SimulationState, TickSchedule},
    udp_snapshot::{Decoder, Encoder, PreparedSnapshot, envelope_bytes, parse},
    workload,
};
use std::time::Instant;

fn samples() -> Vec<SimulationState> {
    let (mut simulation, ids) = workload::simulation(12);
    for id in ids {
        simulation
            .apply(&Command::Chassis {
                chassis: id,
                command: workload::constant_drive(),
            })
            .unwrap();
    }
    let mut schedule = TickSchedule::default();
    (0..240)
        .map(|frame| {
            simulation.step(schedule.advance(32_000_000)).unwrap();
            if frame % 8 == 0 {
                let _ = simulation.apply(&Command::Fire { shooter: 0 });
            }
            let mut state = simulation.state();
            state.snapshot_id = frame + 1;
            state.input_epoch = u64::from(frame >= 120);
            state
        })
        .collect()
}

fn measure(states: &[SimulationState], raw: bool, peers: usize) {
    let encoder = || {
        if raw {
            Encoder::new_raw()
        } else {
            Encoder::default()
        }
    };
    let mut direct: Vec<_> = (0..peers).map(|_| encoder()).collect();
    let mut shared: Vec<_> = (0..peers).map(|_| encoder()).collect();
    let mut decoders: Vec<_> = (0..peers).map(|_| Decoder::default()).collect();
    let (mut direct_ns, mut shared_ns, mut bytes) = (0, 0, 0);
    for (frame, state) in states.iter().enumerate() {
        let started = Instant::now();
        let prepared = PreparedSnapshot::new(state.input_epoch, state).unwrap();
        shared_ns += started.elapsed().as_nanos();
        for peer in 0..peers {
            let started = Instant::now();
            let a = direct[peer].snapshot(state.input_epoch, state).unwrap();
            direct_ns += started.elapsed().as_nanos();
            let started = Instant::now();
            let b = shared[peer].snapshot_prepared(&prepared).unwrap();
            shared_ns += started.elapsed().as_nanos();
            assert_eq!(a, b, "raw={raw} peer={peer} frame={frame}");
            bytes += a.len();
            let wire = parse(&compression::decompress(&b, 4 << 20).unwrap())
                .unwrap()
                .unwrap();
            let (_, ack) = decoders[peer].receive(wire).unwrap();
            // Different lost acknowledgements keep per-peer baseline histories
            // independent, even though their current preparation is identical.
            if let Some(ack) = ack
                && !(frame + peer).is_multiple_of(5)
            {
                let a = direct[peer].feedback(ack);
                let b = shared[peer].feedback(ack);
                assert_eq!(
                    a.as_ref().map(envelope_bytes),
                    b.as_ref().map(envelope_bytes)
                );
                if let Some(retire) = a {
                    let (_, ack) = decoders[peer].receive(retire).unwrap();
                    if let Some(ack) = ack {
                        direct[peer].feedback(ack);
                        shared[peer].feedback(ack);
                    }
                }
            }
            let counters = |e: &Encoder| {
                (
                    e.full_bytes,
                    e.sent_bytes,
                    e.deltas,
                    e.last_packed_bytes,
                    e.raw_fallbacks(),
                    e.skipped_deltas(),
                )
            };
            assert_eq!(counters(&direct[peer]), counters(&shared[peer]));
        }
    }
    println!(
        "raw={raw} peers={peers} frames={} direct_us_per_broadcast={:.2} shared_us_per_broadcast={:.2} speedup={:.2}x identical_bytes={bytes}",
        states.len(),
        direct_ns as f64 / states.len() as f64 / 1000.,
        shared_ns as f64 / states.len() as f64 / 1000.,
        direct_ns as f64 / shared_ns as f64,
    );
}
fn main() {
    let states = samples();
    for raw in [false, true] {
        for peers in [1, 2, 12] {
            measure(&states, raw, peers);
        }
    }
}
