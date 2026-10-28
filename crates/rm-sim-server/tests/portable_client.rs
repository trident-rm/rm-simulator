// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Carrier-independent client behavior; also runs with native hosting disabled.
use rm_sim_server::{
    competition::Competition,
    compression,
    protocol::{ClientMessage, Command, PROTOCOL_VERSION, Role, ServerMessage, Welcome},
    snapshot_codec,
    state::SimulationState,
    udp_codec::{ClientCodec, ClientEvent, Instant, QueuedCommand},
};
use std::time::Duration;

// RMG1 is a transport-independent wire contract. Feed independently framed
// messages through the public client API without constructing a host or socket.
fn receive(
    codec: &mut ClientCodec,
    revision: u64,
    raw: bool,
    message: &ServerMessage,
    now: Instant,
) -> Option<ClientEvent> {
    let bytes = compression::encode(raw, &snapshot_codec::encode_player_message(message));
    let mut result = None;
    for (index, chunk) in bytes.chunks(1000).enumerate().rev() {
        let mut packet = b"RMG1".to_vec();
        packet.push(1);
        packet.extend(revision.to_le_bytes());
        packet.extend((bytes.len() as u32).to_le_bytes());
        packet.extend((index as u32).to_le_bytes());
        packet.extend(chunk);
        if let Some(event) = codec.receive(&packet, now).unwrap() {
            assert!(result.is_none());
            result = Some(event);
        }
    }
    result
}
fn welcome() -> ServerMessage {
    ServerMessage::Welcome(Box::new(Welcome {
        competition: Competition::Rmuc2027,
        prediction_scene: None,
        protocol: PROTOCOL_VERSION,
        client_id: 1,
        team: None,
        role: Role::Spectator,
        chassis: None,
        weapon: Default::default(),
        weapon_limits: Default::default(),
    }))
}

#[test]
fn command_probe_and_barrier_keep_order_without_native_workers() {
    let epoch = Instant::now();
    let mut codec = ClientCodec::new(epoch, 1_000_000, 12, false);
    codec
        .submit(
            QueuedCommand {
                command: Some(Command::Pause { paused: true }),
                time_probe: Some(17),
                confirmation: Some(18),
            },
            epoch,
        )
        .unwrap();
    let now = epoch + Duration::from_millis(32);
    let mut messages = Vec::new();
    while let Some(packet) = codec.next(now).unwrap() {
        assert!(packet.reliable);
        messages.push(snapshot_codec::decode_client_message(&packet.bytes).unwrap());
    }
    assert_eq!(
        messages,
        vec![
            ClientMessage::Command(Command::Pause { paused: true }),
            ClientMessage::TimeProbe { nonce: 17 },
            ClientMessage::Ping { nonce: 18 },
        ]
    );
}

#[test]
fn reversed_fragments_decode_snapshots_with_both_compression_modes() {
    let mut field = rm_sim_world::Field::new(&Default::default()).unwrap();
    add_robot(&mut field);
    let state = SimulationState {
        bots: vec![],
        navigation: vec![],
        sentry_tracking: vec![],
        snapshot_id: 4,
        input_epoch: 2,
        shot_results: vec![],
        paused: true,
        field: field.snapshot(),
    };
    for raw in [false, true] {
        let now = Instant::now();
        let mut codec = ClientCodec::new(now, 1_000_000, 12, raw);
        assert!(matches!(
            receive(&mut codec, 1, raw, &welcome(), now),
            Some(ClientEvent::Welcome(_))
        ));
        let snapshot = ServerMessage::Snapshot(Box::new(state.clone()));
        let Some(ClientEvent::Message(decoded)) = receive(&mut codec, 2, raw, &snapshot, now)
        else {
            panic!("the complete snapshot must be delivered");
        };
        assert_eq!(decoded, snapshot);
        assert!(receive(&mut codec, 3, raw, &snapshot, now).is_none());
        assert_eq!(codec.stats().stale_updates, 1);
    }
}

#[test]
fn unknown_owner_configuration_requests_recovery_without_failing() {
    use rm_sim_server::owner_stream::{OwnerAnchor, OwnerConfig};
    let mut field = rm_sim_world::Field::new(&Default::default()).unwrap();
    let chassis = add_robot(&mut field);
    let state = SimulationState {
        bots: vec![],
        navigation: vec![],
        sentry_tracking: vec![],
        snapshot_id: 1,
        input_epoch: 0,
        shot_results: vec![],
        paused: false,
        field: field.snapshot(),
    };
    let anchor = OwnerAnchor::from_state(&state, chassis).unwrap();
    let now = Instant::now();
    let mut codec = ClientCodec::new(now, 1_000_000, 12, false);
    receive(&mut codec, 1, false, &welcome(), now).unwrap();
    assert!(
        codec
            .receive(&anchor.encode().unwrap(), now)
            .unwrap()
            .is_none()
    );
    codec.acknowledge(now).unwrap();
    let request = codec
        .next(now + Duration::from_millis(32))
        .unwrap()
        .unwrap();
    assert_eq!(
        rm_sim_server::udp_snapshot::Feedback::decode(&request.bytes).unwrap(),
        rm_sim_server::udp_snapshot::Feedback::ConfigMissing {
            revision: anchor.config_revision().raw()
        }
    );
    let config =
        ServerMessage::OwnerConfig(Box::new(OwnerConfig::new(anchor.owner.config.clone())));
    assert!(receive(&mut codec, 2, false, &config, now).is_none());
    assert!(matches!(
        codec.receive(&anchor.encode().unwrap(), now).unwrap(),
        Some(ClientEvent::Anchor(_))
    ));
}

fn add_robot(field: &mut rm_sim_world::Field) -> u32 {
    field
        .add_chassis(&rm_sim_world::ChassisPlacement {
            team: rm_sim_world::Team::Red,
            kind: rm_sim_world::RobotKind::Infantry,
            performance: None,
            spawn: rm_sim_world::Pose::at([0., 0., 0.3]),
            config: Default::default(),
        })
        .unwrap()
}
