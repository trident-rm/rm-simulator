// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Bounded browser carrier. Each binary WebSocket message carries one codec datagram.
use crate::{
    host::HostHandle,
    lifecycle::Stop,
    protocol::{ClientMessage, Role},
    snapshot_codec,
    udp_codec::HostPeer,
};
use std::{
    io,
    net::{SocketAddr, TcpListener, TcpStream, ToSocketAddrs},
    thread,
    time::{Duration, Instant},
};
use tungstenite::{
    Error, Message, accept_hdr_with_config,
    handshake::server::{Request, Response},
    protocol::WebSocketConfig,
};

const POLL: Duration = Duration::from_millis(2);
const TIMEOUT: Duration = Duration::from_secs(5);
const MAX_PEERS: usize = 16;
const MAX_PACKET: usize = 64 * 1024;
const MAX_PENDING: usize = 512 * 1024;

/// Optional WebSocket listener over the ordinary unprivileged host admission path.
///
/// This listener serves plain WebSockets. Public deployments must terminate TLS
/// at a reverse proxy. Origins are exact matches; absent origins are rejected.
/// Dropping the listener stops and joins its bounded set of connection workers.
pub struct WebSocketServer {
    address: SocketAddr,
    stop: Stop,
    worker: Option<thread::JoinHandle<()>>,
}
impl WebSocketServer {
    /// Bind a browser carrier restricted to the supplied exact HTTP(S) origins.
    ///
    /// # Errors
    /// Returns an error for an empty origin allowlist or a failed socket/thread setup.
    pub fn bind(
        address: impl ToSocketAddrs,
        handle: HostHandle,
        origins: Vec<String>,
    ) -> io::Result<Self> {
        if origins.is_empty()
            || origins
                .iter()
                .any(|origin| origin.is_empty() || origin == "null" || origin.contains('*'))
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "explicit browser origins required",
            ));
        }
        let listener = TcpListener::bind(address)?;
        let address = listener.local_addr()?;
        listener.set_nonblocking(true)?;
        let stop = Stop::default();
        let stopping = stop.clone();
        let worker = thread::Builder::new()
            .name("rm-websocket".into())
            .spawn(move || {
                let mut peers: Vec<thread::JoinHandle<()>> = Vec::new();
                while !stopping.wait(POLL) {
                    let mut index = 0;
                    while index < peers.len() {
                        if peers[index].is_finished() {
                            let _ = peers.swap_remove(index).join();
                        } else {
                            index += 1;
                        }
                    }
                    match listener.accept() {
                        Ok((stream, _)) if peers.len() < MAX_PEERS => {
                            let handle = handle.clone();
                            let origins = origins.clone();
                            let stop = stopping.clone();
                            if let Ok(peer) = thread::Builder::new()
                                .name("rm-websocket-peer".into())
                                .spawn(move || {
                                    let _ = serve(stream, handle, &origins, &stop);
                                })
                            {
                                peers.push(peer);
                            }
                        }
                        Ok(_) => {}
                        Err(error) if error.kind() == io::ErrorKind::WouldBlock => {}
                        Err(_) => break,
                    }
                }
                for peer in peers {
                    let _ = peer.join();
                }
            })?;
        Ok(Self {
            address,
            stop,
            worker: Some(worker),
        })
    }
    /// The bound TCP address, including the chosen port for a port-zero bind.
    pub fn local_addr(&self) -> SocketAddr {
        self.address
    }
}
impl Drop for WebSocketServer {
    fn drop(&mut self) {
        self.stop.request();
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}
fn pending(error: Error) -> Result<(), Error> {
    match error {
        Error::Io(error) if error.kind() == io::ErrorKind::WouldBlock => Ok(()),
        error => Err(error),
    }
}
fn serve(
    stream: TcpStream,
    handle: HostHandle,
    origins: &[String],
    stop: &Stop,
) -> Result<(), Box<dyn std::error::Error>> {
    stream.set_read_timeout(Some(TIMEOUT))?;
    stream.set_write_timeout(Some(TIMEOUT))?;
    stream.set_nodelay(true)?;
    let origins = origins.to_vec();
    #[expect(
        clippy::result_large_err,
        reason = "tungstenite requires an unboxed HTTP error response in its callback"
    )]
    let callback = move |request: &Request, response: Response| {
        if request
            .headers()
            .get("origin")
            .and_then(|value| value.to_str().ok())
            .is_some_and(|origin| origins.iter().any(|allowed| allowed == origin))
        {
            Ok(response)
        } else {
            let mut denied = tungstenite::http::Response::new(Some("Origin denied".into()));
            *denied.status_mut() = tungstenite::http::StatusCode::FORBIDDEN;
            Err(denied)
        }
    };
    let config = WebSocketConfig::default()
        .write_buffer_size(0)
        .max_write_buffer_size(MAX_PENDING)
        .max_message_size(Some(MAX_PACKET))
        .max_frame_size(Some(MAX_PACKET));
    // Tungstenite caps handshake input at 64 KiB and 512 reads.
    // Nonblocking resumption gives the entire HTTP upgrade one deadline.
    stream.set_nonblocking(true)?;
    let deadline = Instant::now() + TIMEOUT;
    let mut handshake = accept_hdr_with_config(stream, callback, Some(config));
    let mut socket = loop {
        match handshake {
            Ok(socket) => break socket,
            Err(tungstenite::HandshakeError::Interrupted(interrupted)) => {
                if Instant::now() >= deadline || stop.wait(POLL) {
                    return Err("WebSocket handshake timeout".into());
                }
                handshake = interrupted.handshake();
            }
            Err(tungstenite::HandshakeError::Failure(error)) => return Err(error.into()),
        }
    };
    let mut peer = HostPeer::new(handle, Instant::now(), 2 * 1024 * 1024, false);
    let mut stalled = None;
    let mut last_received = Instant::now();
    let mut last_ping = Instant::now();
    while !stop.wait(POLL) && !peer.closed() {
        if peer.client_id().is_none() && peer.admitted().elapsed() > TIMEOUT {
            break;
        }
        if last_received.elapsed() > Duration::from_secs(30) {
            break;
        }
        if last_ping.elapsed() >= TIMEOUT {
            if let Err(error) = socket.write(Message::Ping(Vec::new().into())) {
                pending(error)?;
                stalled.get_or_insert_with(Instant::now);
            }
            last_ping = Instant::now();
        }
        for _ in 0..32 {
            match socket.read() {
                Ok(Message::Binary(bytes)) => {
                    last_received = Instant::now();
                    if peer.client_id().is_none() {
                        // Browser connections never gain the embedded owner or referee seat.
                        match snapshot_codec::decode_client_message(&bytes)? {
                            ClientMessage::Hello {
                                role: Role::Pilot | Role::Spectator,
                                ..
                            } => {}
                            _ => return Err("expected unprivileged hello".into()),
                        }
                    }
                    peer.deliver(&bytes, Instant::now())?;
                }
                Ok(Message::Close(_)) => return Ok(()),
                Ok(Message::Ping(_) | Message::Pong(_)) => {
                    last_received = Instant::now();
                }
                Ok(_) => return Err("binary codec messages required".into()),
                Err(Error::Io(error)) if error.kind() == io::ErrorKind::WouldBlock => break,
                Err(error) => return Err(error.into()),
            }
        }
        match socket.flush() {
            Ok(()) => stalled = None,
            Err(error) => {
                pending(error)?;
                stalled.get_or_insert_with(Instant::now);
            }
        }
        if stalled.is_some_and(|since| since.elapsed() > TIMEOUT) {
            break;
        }
        if stalled.is_none() {
            peer.pump(Instant::now(), 0, 8, 32)?;
            for packet in peer.take_outgoing() {
                if let Err(error) = socket.write(Message::Binary(packet.bytes.into())) {
                    pending(error)?;
                    stalled.get_or_insert_with(Instant::now);
                }
            }
        }
    }
    Ok(())
}
