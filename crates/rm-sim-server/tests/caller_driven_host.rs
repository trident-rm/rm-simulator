// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-driven scheduling retains the ordinary codec and host authority path.
#![cfg(feature = "host-runtime")]
use rm_sim_server::{
    host::CallerDrivenHost,
    protocol::{Command, Role, ServerMessage},
    simulation::Simulation,
    udp_codec::{ClientCodec, ClientEvent, HostPeer, Instant, QueuedCommand},
};
use rm_sim_world::{Field, FieldConfig, Team};
use std::time::Duration;

fn simulation(paused: bool) -> Simulation {
    Simulation::new(Field::new(&FieldConfig::default()).unwrap(), paused)
}
fn exchange(peer: &mut HostPeer, codec: &mut ClientCodec, now: Instant) -> Vec<ClientEvent> {
    codec.acknowledge(now).unwrap();
    while let Some(packet) = codec.next(now).unwrap() {
        peer.deliver(&packet.bytes, now).unwrap();
    }
    peer.pump(now, 0, 32, 256).unwrap();
    peer.take_outgoing()
        .into_iter()
        .filter_map(|packet| codec.receive(&packet.bytes, now).unwrap())
        .collect()
}
fn admitted(host: &CallerDrivenHost, owner: bool) -> (HostPeer, ClientCodec, Instant) {
    let now = Instant::now();
    let mut peer = HostPeer::new(host.handle(), now, 1_000_000, true);
    if owner {
        peer = peer.with_owner_spawn([0., 0., 1.], 0.);
    }
    peer.deliver(
        &ClientCodec::hello("caller", Some(Team::Red), Role::Spectator).unwrap(),
        now,
    )
    .unwrap();
    let mut codec = ClientCodec::new(now, 1_000_000, 12, true);
    let mut welcomed = false;
    for i in 0..100 {
        welcomed |= exchange(&mut peer, &mut codec, now + Duration::from_millis(i))
            .into_iter()
            .any(|event| matches!(event, ClientEvent::Welcome(_)));
        if welcomed {
            break;
        }
    }
    assert!(welcomed, "the ordinary codec must complete admission");
    (peer, codec, now + Duration::from_millis(100))
}
#[test]
fn caller_budget_carries_bounded_debt_and_preserves_startup_hold() {
    let mut host = CallerDrivenHost::new(simulation(false), false);
    let handle = host.handle();
    host.advance(Duration::from_secs(10)).unwrap();
    assert_eq!(
        handle.snapshot().unwrap().tick,
        0,
        "startup cannot charge loading time"
    );
    handle.ready();
    host.advance(Duration::ZERO).unwrap();
    assert_eq!(
        host.stats().ticks,
        0,
        "first ready interval releases the hold"
    );
    for _ in 0..4 {
        host.advance(Duration::ZERO).unwrap();
        assert!(
            host.stats().ticks <= 8,
            "one caller frame exceeds its tick budget"
        );
    }
    assert_eq!(host.stats().backlog_ns, 0);
    let before = handle.snapshot().unwrap().tick;
    host.advance(Duration::from_secs(10)).unwrap();
    assert_eq!(host.stats().ticks, 8);
    assert_eq!(
        host.stats().backlog_ns,
        250_000_000 - 8 * rm_sim_world::tick_ns()
    );
    for _ in 0..3 {
        host.advance(Duration::ZERO).unwrap();
        assert!(host.stats().ticks <= 8);
    }
    assert_eq!(
        handle.snapshot().unwrap().tick - before,
        32,
        "suspension catch-up is capped to 250 ms"
    );
}
#[test]
fn owner_step_confirmation_precedes_pong_through_shared_codec() {
    let mut host = CallerDrivenHost::new(simulation(true), true);
    let (mut peer, mut codec, mut now) = admitted(&host, true);
    codec
        .submit(
            QueuedCommand {
                command: Some(Command::Step { ticks: 3 }),
                confirmation: Some(7),
                time_probe: None,
            },
            now,
        )
        .unwrap();
    let mut covered = false;
    let mut confirmed = false;
    for _ in 0..100 {
        now += Duration::from_millis(1);
        host.advance(Duration::from_millis(1)).unwrap();
        for event in exchange(&mut peer, &mut codec, now) {
            match event {
                ClientEvent::Message(ServerMessage::Snapshot(state)) if state.field.tick == 3 => {
                    covered = true
                }
                ClientEvent::Message(ServerMessage::Pong { nonce: 7 }) => {
                    assert!(covered, "confirmation must cover the preceding step");
                    confirmed = true;
                }
                _ => {}
            }
        }
        if confirmed {
            break;
        }
    }
    assert!(confirmed);
    assert_eq!(
        host.handle().snapshot().unwrap().tick,
        3,
        "paused manual step must execute exactly once"
    );
}
#[test]
fn non_owner_cannot_pause_the_caller_driven_host() {
    let host = CallerDrivenHost::new(simulation(false), true);
    let (mut peer, mut codec, mut now) = admitted(&host, false);
    codec
        .submit(
            QueuedCommand {
                command: Some(Command::Pause { paused: true }),
                confirmation: Some(11),
                time_probe: None,
            },
            now,
        )
        .unwrap();
    let mut rejected = false;
    for _ in 0..100 {
        now += Duration::from_millis(1);
        rejected |= exchange(&mut peer, &mut codec, now)
            .into_iter()
            .any(|event| matches!(event, ClientEvent::Message(ServerMessage::Rejected { .. })));
        if rejected {
            break;
        }
    }
    assert!(
        rejected,
        "caller-driven scheduling cannot grant a socket peer owner authority"
    );
    assert!(!host.handle().state().unwrap().paused);
}

#[test]
#[ignore = "requires hydrated official RMUL CAD packages"]
fn official_rmul_hosts_advertise_verified_prediction_geometry() {
    use rm_sim_server::{cad_assets, competition::Competition};
    for (competition, package) in [
        (Competition::RmulInfantry2027, "field-rmul-2027-1v1"),
        (Competition::Rmul2027, "field-rmul-2027-3v3"),
    ] {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join(package);
        let cad = cad_assets::load(&path).unwrap();
        let simulation = Simulation::rmul_from_cad(&cad, competition, true, None, false).unwrap();
        let scene = simulation
            .prediction_scene()
            .expect("official RMUL scene supports prediction");
        assert!(scene.terrain);
        assert!(
            scene.matches_package(&cad),
            "prediction identity must use the served package manifests"
        );
        let (geometry, _) = scene.geometry(&cad).unwrap();
        assert!(
            !geometry.into_triangles().1.is_empty(),
            "prediction uses the actual official field"
        );
        let snapshot = simulation.snapshot();
        assert_eq!(
            snapshot.rmul_2027.as_ref().unwrap().format,
            competition.rmul_format().unwrap()
        );
        assert!(
            snapshot.rmuc_2027.is_none(),
            "prediction admission must preserve the independent RMUL authority"
        );
    }
}
