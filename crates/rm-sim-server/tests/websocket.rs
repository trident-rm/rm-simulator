// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Real socket admission and codec exchange through the browser carrier.
#![cfg(feature = "native-host")]
use rm_sim_server::{
    host::Host,
    protocol::Role,
    simulation::Simulation,
    udp_codec::{ClientCodec, ClientEvent, Instant},
    websocket::WebSocketServer,
};
use rm_sim_world::{Field, FieldConfig};
use std::{net::TcpStream, time::Duration};
use tungstenite::{Message, client::IntoClientRequest};

#[test]
fn allowed_origin_receives_welcome_and_disconnect_releases_seat() {
    let host = Host::new(
        Simulation::new(Field::new(&FieldConfig::default()).unwrap(), true),
        true,
    )
    .unwrap();
    let server = WebSocketServer::bind(
        "127.0.0.1:0",
        host.handle(),
        vec!["http://localhost:8080".into()],
    )
    .unwrap();
    let address = format!("ws://{}/", server.local_addr());
    let mut request = address.as_str().into_client_request().unwrap();
    request
        .headers_mut()
        .insert("Origin", "http://localhost:8080".parse().unwrap());
    let stream = TcpStream::connect(server.local_addr()).unwrap();
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    let (mut socket, _) = tungstenite::client(request, stream).unwrap();
    socket
        .send(Message::Binary(
            ClientCodec::hello("browser", None, Role::Spectator)
                .unwrap()
                .into(),
        ))
        .unwrap();
    let mut codec = ClientCodec::new(Instant::now(), 1_000_000, 12, false);
    loop {
        let Message::Binary(bytes) = socket.read().unwrap() else {
            continue;
        };
        if let Some(ClientEvent::Welcome(welcome)) = codec.receive(&bytes, Instant::now()).unwrap()
        {
            assert_eq!(welcome.role, Role::Spectator);
            break;
        }
    }
    assert_eq!(host.handle().peer_count().unwrap(), 1);
    drop(socket);
    drop(server);
    assert_eq!(host.handle().peer_count().unwrap(), 0);
}

#[test]
fn missing_or_unapproved_origin_cannot_join() {
    let host = Host::new(
        Simulation::new(Field::new(&FieldConfig::default()).unwrap(), true),
        true,
    )
    .unwrap();
    let server = WebSocketServer::bind(
        "127.0.0.1:0",
        host.handle(),
        vec!["http://localhost:8080".into()],
    )
    .unwrap();
    for origin in [None, Some("http://evil.example")] {
        let address = format!("ws://{}/", server.local_addr());
        let mut request = address.as_str().into_client_request().unwrap();
        if let Some(origin) = origin {
            request
                .headers_mut()
                .insert("Origin", origin.parse().unwrap());
        }
        assert!(
            tungstenite::client(request, TcpStream::connect(server.local_addr()).unwrap()).is_err()
        );
    }
    assert_eq!(host.handle().peer_count().unwrap(), 0);
}

#[test]
fn referee_hello_cannot_claim_a_privileged_seat() {
    let host = Host::new(
        Simulation::new(Field::new(&FieldConfig::default()).unwrap(), true),
        true,
    )
    .unwrap();
    let server = WebSocketServer::bind(
        "127.0.0.1:0",
        host.handle(),
        vec!["http://localhost:8080".into()],
    )
    .unwrap();
    let address = format!("ws://{}/", server.local_addr());
    let mut request = address.as_str().into_client_request().unwrap();
    request
        .headers_mut()
        .insert("Origin", "http://localhost:8080".parse().unwrap());
    let stream = TcpStream::connect(server.local_addr()).unwrap();
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    let (mut socket, _) = tungstenite::client(request, stream).unwrap();
    socket
        .send(Message::Binary(
            ClientCodec::hello("browser", None, Role::Referee)
                .unwrap()
                .into(),
        ))
        .unwrap();
    assert!(
        socket.read().is_err(),
        "privileged hello must close the connection"
    );
    assert_eq!(host.handle().peer_count().unwrap(), 0);
}

#[test]
fn shutdown_interrupts_an_incomplete_upgrade() {
    let host = Host::new(
        Simulation::new(Field::new(&FieldConfig::default()).unwrap(), true),
        true,
    )
    .unwrap();
    let server = WebSocketServer::bind(
        "127.0.0.1:0",
        host.handle(),
        vec!["http://localhost:8080".into()],
    )
    .unwrap();
    let mut stream = TcpStream::connect(server.local_addr()).unwrap();
    use std::io::Write;
    stream.write_all(b"GET / HTTP/1.1\r\n").unwrap();
    let start = Instant::now();
    drop(server);
    assert!(
        start.elapsed() < Duration::from_secs(2),
        "shutdown waited for an incomplete upgrade"
    );
}
