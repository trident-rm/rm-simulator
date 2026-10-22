// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! A small HTTP/1.1 front for the referee: `GET /` serves the control panel,
//! `GET /api/state` the current state (`paused` and the field snapshot),
//! `GET /api/clock` compact clock metadata and `GET /api/control/ID` a robot life identity,
//! `POST /api/referee` a `RefereeCommand` and `POST /api/command` any
//! [`Command`] as JSON. Requests are handled one per connection on their
//! own thread, up to 32 simultaneous handlers with a shared 10 s request budget;
//! the reply always closes the connection. No dependencies
//! beyond the standard library and `serde_json`.
use crate::host::HostHandle;
use crate::lifecycle::Stop;
use crate::protocol::Command;
use rm_simulator_world::RefereeCommand;
use std::io::{self, BufRead, BufReader, Read, Write};
use std::net::{Shutdown, SocketAddr, TcpListener, TcpStream, ToSocketAddrs};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

/// The referee panel page.
pub const PANEL_HTML: &str = include_str!("panel.html");
/// Largest body accepted, 64 KiB. A request is refused before the body is read.
const MAX_BODY_BYTES: usize = 64 << 10;
/// Longest request line or header line, and the most headers, accepted.
const MAX_HEADER_LINE_BYTES: usize = 8 << 10;
/// Header count past which a request head is refused.
const MAX_HEADERS: usize = 64;
/// Hard ceiling on active connection threads and tracked sockets.
const MAX_CONNECTIONS: usize = 32;
/// Total socket and host-mailbox budget, shared by reads and writes.
const REQUEST_LIFETIME: Duration = Duration::from_secs(10);

/// A bound referee HTTP server. Dropping it closes the listener.
pub struct HttpServer {
    listener: HttpListener,
}

/// A bound listener and the worker that accepts for it. Dropping it stops and
/// joins the worker. The referee panel is the only remaining TCP server, so its
/// accept loop lives here rather than in a shared transport module.
struct HttpListener {
    /// Address the socket is actually bound to.
    local_addr: SocketAddr,
    stop: Stop,
    worker: Option<JoinHandle<()>>,
}
impl HttpListener {
    /// Start the accepting worker under the given thread name.
    ///
    /// The worker polls the listener in nonblocking mode, moves each accepted
    /// stream back to blocking mode and serves it on its own thread. `serve`
    /// runs per connection and must not use the listener. Completed connection
    /// workers are reaped as the loop runs; excess connections are closed.
    /// Shutdown stops accepting within one
    /// 10 ms poll, shuts every live stream down and joins all of them,
    /// including connections still waiting for a handshake.
    fn start(
        listener: TcpListener,
        stop: Stop,
        name: &'static str,
        serve: impl Fn(TcpStream) + Send + Sync + 'static,
    ) -> io::Result<Self> {
        let local_addr = listener.local_addr()?;
        listener.set_nonblocking(true)?;
        let stopping = stop.clone();
        let serve = Arc::new(serve);
        let worker = thread::Builder::new().name(name.into()).spawn(move || {
            let mut connections: Vec<(TcpStream, JoinHandle<()>)> = Vec::new();
            while !stopping.wait(Duration::ZERO) {
                // Reap completed workers so a long-running server keeps only
                // active connections. Shutdown also includes incomplete hellos.
                let mut index = 0;
                while index < connections.len() {
                    if connections[index].1.is_finished() {
                        let (_, worker) = connections.swap_remove(index);
                        let _ = worker.join();
                    } else {
                        index += 1;
                    }
                }
                match listener.accept() {
                    Ok((stream, _)) => {
                        if connections.len() >= MAX_CONNECTIONS {
                            let _ = stream.shutdown(Shutdown::Both);
                            continue;
                        }
                        // Accepted sockets inherit nonblocking mode on some
                        // platforms; only the listener uses polling.
                        if let Err(error) = stream.set_nonblocking(false) {
                            eprintln!("{name}: configuring connection: {error}");
                            continue;
                        }
                        let tracked = match stream.try_clone() {
                            Ok(tracked) => tracked,
                            Err(error) => {
                                eprintln!("{name}: tracking connection: {error}");
                                continue;
                            }
                        };
                        let serve = serve.clone();
                        match thread::Builder::new()
                            .name(format!("{name}-peer"))
                            .spawn(move || serve(stream))
                        {
                            Ok(worker) => connections.push((tracked, worker)),
                            Err(error) => eprintln!("{name}: starting connection: {error}"),
                        }
                    }
                    Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                        stopping.wait(Duration::from_millis(10));
                    }
                    Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
                    Err(error) => {
                        eprintln!("{name}: accept failed: {error}");
                        stopping.request();
                    }
                }
            }
            // Release the port before waiting for handlers to finish.
            drop(listener);
            for (stream, _) in &connections {
                let _ = stream.shutdown(Shutdown::Both);
            }
            for (_, worker) in connections {
                let _ = worker.join();
            }
        })?;
        Ok(Self {
            local_addr,
            stop,
            worker: Some(worker),
        })
    }

    /// Stop accepting, close live connections and join the worker. Idempotent.
    fn shutdown(&mut self) {
        self.stop.request();
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}
impl Drop for HttpListener {
    /// Stop accepting and join the worker.
    fn drop(&mut self) {
        self.shutdown();
    }
}

impl HttpServer {
    /// Bind `addr` and serve the panel on its own thread. Every action the page
    /// sends becomes a typed request on the same host queue as the transports.
    pub fn bind(addr: impl ToSocketAddrs, host: HostHandle) -> io::Result<HttpServer> {
        let listener = TcpListener::bind(addr)?;
        let stop = Stop::default();
        let connection_stop = stop.clone();
        let listener = HttpListener::start(listener, stop, "rm-http", move |stream| {
            if let Err(error) = handle(stream, &host, connection_stop.clone(), REQUEST_LIFETIME) {
                eprintln!("http: {error}");
            }
        })?;
        Ok(HttpServer { listener })
    }
    /// The address the socket is actually bound to, with the port the OS chose.
    pub fn local_addr(&self) -> SocketAddr {
        self.listener.local_addr
    }
    /// Close the listener and active requests, then wait for request workers.
    pub fn shutdown(&mut self) {
        self.listener.shutdown();
    }
}

/// A parsed request: method, path (without query) and body.
pub struct Request {
    /// Request method, verbatim and case sensitive.
    pub method: String,
    /// Target path with the query string removed.
    pub path: String,
    /// Body of exactly the declared `Content-Length` bytes.
    pub body: Vec<u8>,
}

/// A reply: status, content type and body.
pub struct Response {
    /// HTTP status code, which also selects the reason phrase.
    pub status: u16,
    /// MIME type written as the `Content-Type` header.
    pub content_type: &'static str,
    /// Body bytes, one image of the length written as `Content-Length`.
    pub body: Vec<u8>,
}
impl Response {
    /// A JSON reply. A serialization failure yields an empty body and no error
    /// signal, which no handled variant can produce.
    fn json(status: u16, value: &impl serde::Serialize) -> Response {
        Response {
            status,
            content_type: "application/json",
            body: serde_json::to_vec(value).unwrap_or_default(),
        }
    }
    /// A JSON reply of `{"error": message}`.
    fn error(status: u16, message: impl Into<String>) -> Response {
        Response::json(status, &serde_json::json!({ "error": message.into() }))
    }
    /// The reason phrase for this status, defaulting to `Internal Server Error`.
    fn reason(&self) -> &'static str {
        match self.status {
            200 => "OK",
            400 => "Bad Request",
            404 => "Not Found",
            405 => "Method Not Allowed",
            413 => "Payload Too Large",
            503 => "Service Unavailable",
            _ => "Internal Server Error",
        }
    }
}

/// One line of the request head, capped at [`MAX_HEADER_LINE_BYTES`].
fn read_head_line<R: BufRead>(reader: &mut R) -> io::Result<String> {
    let mut line = String::new();
    reader
        .by_ref()
        .take(MAX_HEADER_LINE_BYTES as u64 + 1)
        .read_line(&mut line)?;
    if line.len() > MAX_HEADER_LINE_BYTES {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "header line too long",
        ));
    }
    Ok(line)
}

/// Read one request head and its body. `Ok(None)` means the peer closed before
/// sending anything; an oversized line, too many headers or a body past
/// `MAX_BODY_BYTES` is an error. The body is read to the declared length, so a
/// short body blocks until the read timeout.
pub fn read_request<R: BufRead>(reader: &mut R) -> io::Result<Option<Request>> {
    let line = read_head_line(reader)?;
    if line.is_empty() {
        return Ok(None);
    }
    let mut words = line.split_whitespace();
    let (Some(method), Some(target)) = (words.next(), words.next()) else {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "bad request line",
        ));
    };
    let path = target.split('?').next().unwrap_or("/").to_string();
    let method = method.to_string();
    let mut content_length = 0usize;
    for count in 0.. {
        if count == MAX_HEADERS {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "too many headers",
            ));
        }
        let header = read_head_line(reader)?;
        if header.is_empty() {
            break;
        }
        let header = header.trim_end();
        if header.is_empty() {
            break;
        }
        if let Some((name, value)) = header.split_once(':')
            && name.eq_ignore_ascii_case("content-length")
        {
            content_length = value
                .trim()
                .parse()
                .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "bad content length"))?;
        }
    }
    if content_length > MAX_BODY_BYTES {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "body too large"));
    }
    let mut body = vec![0; content_length];
    reader.read_exact(&mut body)?;
    Ok(Some(Request { method, path, body }))
}

/// Route one request against the simulation.
pub fn respond(request: &Request, host: &HostHandle) -> Response {
    match (request.method.as_str(), request.path.as_str()) {
        ("GET", "/" | "/index.html") => Response {
            status: 200,
            content_type: "text/html; charset=utf-8",
            body: PANEL_HTML.as_bytes().to_vec(),
        },
        ("GET", "/api/fire-records") => match host.fire_records() {
            Ok(records) => Response::json(200, &records),
            Err(reason) => Response::error(503, reason),
        },
        ("GET", "/api/clock") => match host.control_status(None) {
            Ok(status) => Response::json(200, &status),
            Err(reason) => Response::error(503, reason),
        },
        ("GET", path) if path.starts_with("/api/control/") => {
            match path.trim_start_matches("/api/control/").parse::<u32>() {
                Ok(id) => match host.control_status(Some(id)) {
                    Ok(status) => Response::json(200, &status),
                    Err(reason) => {
                        Response::error(if host.is_stopped() { 503 } else { 404 }, reason)
                    }
                },
                Err(_) => Response::error(400, "invalid chassis ID"),
            }
        }
        ("POST", "/api/sensors/attach") => match serde_json::from_slice::<
            rm_simulator_physics::sensors::synchronized::Config,
        >(&request.body)
        {
            Ok(config) => match host.attach_sensors(config) {
                Ok(id) => Response::json(200, &serde_json::json!({"id": id})),
                Err(e) => Response::error(400, e),
            },
            Err(e) => Response::error(400, e.to_string()),
        },
        ("POST", "/api/sensors/poll") => match serde_json::from_slice::<u64>(&request.body) {
            Ok(id) => match host.poll_sensors(id) {
                Ok(batch) => Response::json(200, &batch),
                Err(e) => Response::error(404, e),
            },
            Err(e) => Response::error(400, e.to_string()),
        },
        ("POST", "/api/sensors/detach") => match serde_json::from_slice::<u64>(&request.body) {
            Ok(id) => match host.detach_sensors(id) {
                Ok(()) => Response::json(200, &serde_json::json!({"ok": true})),
                Err(e) => Response::error(503, e),
            },
            Err(e) => Response::error(400, e.to_string()),
        },
        ("GET", "/api/state") => match host.state() {
            Ok(state) => Response::json(200, &state),
            Err(reason) => Response::error(503, reason),
        },
        ("POST", "/api/command") => match serde_json::from_slice::<Command>(&request.body) {
            Ok(Command::PlaceChassis { .. }) => {
                Response::error(403, "only the embedded owner places a chassis")
            }
            Ok(command) => apply(host, &command),
            Err(error) => Response::error(400, format!("not a command: {error}")),
        },
        ("POST", "/api/referee") => match serde_json::from_slice::<RefereeCommand>(&request.body) {
            Ok(command) => apply(host, &Command::Referee(command)),
            Err(error) => Response::error(400, format!("not a referee command: {error}")),
        },
        ("GET", "/api/command" | "/api/referee") => Response::error(405, "POST a JSON command"),
        _ => Response::error(404, "no such page"),
    }
}

/// Submit one command through the host queue and turn its result into a reply:
/// 200 on acceptance, 400 on rejection and 503 once the host has stopped.
fn apply(host: &HostHandle, command: &Command) -> Response {
    if host.is_stopped() {
        return Response::error(503, "host is stopped");
    }
    match host.apply(command) {
        Ok(_) => Response::json(200, &serde_json::json!({ "ok": true })),
        Err(reason) => Response::error(if host.is_stopped() { 503 } else { 400 }, reason),
    }
}

/// A socket whose individual operations share one absolute deadline.
struct DeadlineStream {
    stream: TcpStream,
    deadline: Instant,
}
impl DeadlineStream {
    fn remaining(&self) -> io::Result<Duration> {
        self.deadline
            .checked_duration_since(Instant::now())
            .filter(|remaining| !remaining.is_zero())
            .ok_or_else(|| {
                io::Error::new(io::ErrorKind::TimedOut, "HTTP request deadline exceeded")
            })
    }
}
impl Read for DeadlineStream {
    fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        self.stream.set_read_timeout(Some(self.remaining()?))?;
        self.stream.read(buffer)
    }
}
impl Write for DeadlineStream {
    fn write(&mut self, buffer: &[u8]) -> io::Result<usize> {
        self.stream.set_write_timeout(Some(self.remaining()?))?;
        self.stream.write(buffer)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.stream.set_write_timeout(Some(self.remaining()?))?;
        self.stream.flush()
    }
}

/// Serve one connection within a total budget, including mailbox waits and writes.
fn handle(stream: TcpStream, host: &HostHandle, stop: Stop, lifetime: Duration) -> io::Result<()> {
    let deadline = Instant::now() + lifetime;
    let host = host.for_http(deadline, stop);
    let mut reader = BufReader::new(DeadlineStream { stream, deadline });
    let response = match read_request(&mut reader) {
        Ok(Some(request)) => respond(&request, &host),
        Ok(None) => return Ok(()),
        Err(error) => Response::error(400, error.to_string()),
    };
    write_response(reader.into_inner(), &response)
}

/// Write a complete response and close the connection. `Connection: close` is
/// always sent, so a caller serves at most one request per accepted socket.
pub fn write_response<W: Write>(mut writer: W, response: &Response) -> io::Result<()> {
    write!(
        writer,
        "HTTP/1.1 {} {}\r\nContent-Type: {}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
        response.status,
        response.reason(),
        response.content_type,
        response.body.len()
    )?;
    writer.write_all(&response.body)?;
    writer.flush()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::host::Host;
    use crate::simulation::Simulation;
    use rm_simulator_world::{Field, FieldConfig};
    use std::io::Read;

    fn simulation() -> (Host, HostHandle) {
        let config = crate::workload::field_config();
        let host = Host::new(Simulation::new(Field::new(&config).unwrap(), true), true).unwrap();
        let handle = host.handle();
        (host, handle)
    }
    fn request(text: &str) -> Request {
        read_request(&mut BufReader::new(text.as_bytes()))
            .unwrap()
            .unwrap()
    }
    fn socket_pair() -> (TcpStream, TcpStream) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let client = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        let (server, _) = listener.accept().unwrap();
        (client, server)
    }

    #[test]
    fn trickling_headers_and_bodies_cannot_extend_the_deadline() {
        let (_host, host) = simulation();
        for prefix in [
            &b"GET / HTTP/1.1\r\nX-Test: "[..],
            &b"POST /api/command HTTP/1.1\r\nContent-Length: 1000\r\n\r\n"[..],
        ] {
            let (mut client, server) = socket_pair();
            client.write_all(prefix).unwrap();
            let (done, result) = std::sync::mpsc::channel();
            let host = host.clone();
            let worker = thread::spawn(move || {
                done.send(handle(
                    server,
                    &host,
                    Stop::default(),
                    Duration::from_millis(150),
                ))
                .unwrap();
            });
            let trickle = thread::spawn(move || {
                for _ in 0..30 {
                    if client.write_all(b"a").is_err() {
                        break;
                    }
                    thread::sleep(Duration::from_millis(20));
                }
            });
            assert!(
                result
                    .recv_timeout(Duration::from_secs(2))
                    .unwrap()
                    .is_err()
            );
            worker.join().unwrap();
            trickle.join().unwrap();
        }
    }

    #[test]
    fn nonreading_clients_cannot_hold_response_writes_forever() {
        let (_client, server) = socket_pair();
        let (done, result) = std::sync::mpsc::channel();
        let worker = thread::spawn(move || {
            let socket = DeadlineStream {
                stream: server,
                deadline: Instant::now() + Duration::from_millis(150),
            };
            let response = Response {
                status: 200,
                content_type: "application/octet-stream",
                body: vec![0; 32 << 20],
            };
            done.send(write_response(socket, &response)).unwrap();
        });
        assert!(
            result
                .recv_timeout(Duration::from_secs(2))
                .unwrap()
                .is_err()
        );
        worker.join().unwrap();
    }

    #[test]
    fn listener_caps_handlers_and_reclaims_slots() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let (entered, workers) = std::sync::mpsc::channel();
        let server = HttpListener::start(
            listener,
            Stop::default(),
            "http-cap-test",
            move |mut stream| {
                entered.send(()).unwrap();
                let _ = stream.read(&mut [0]);
            },
        )
        .unwrap();
        let mut clients = Vec::new();
        for _ in 0..MAX_CONNECTIONS {
            clients.push(TcpStream::connect(server.local_addr).unwrap());
            workers.recv_timeout(Duration::from_secs(2)).unwrap();
        }
        let mut excess = TcpStream::connect(server.local_addr).unwrap();
        excess
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        assert_eq!(excess.read(&mut [0]).unwrap(), 0);
        assert!(workers.try_recv().is_err());
        clients.pop().unwrap().shutdown(Shutdown::Both).unwrap();
        // Probe until the accept loop has reaped the released handler.
        let end = Instant::now() + Duration::from_secs(2);
        loop {
            let probe = TcpStream::connect(server.local_addr).unwrap();
            if workers.recv_timeout(Duration::from_millis(20)).is_ok() {
                clients.push(probe);
                break;
            }
            assert!(Instant::now() < end);
        }
        drop(server); // Must also join all handlers still waiting on reads.
    }

    #[test]
    fn request_deadline_bounds_host_submission_and_reply_waits() {
        for fill_mailbox in [false, true] {
            let (_host, host) = simulation();
            let release = host.stall();
            if fill_mailbox {
                while host.broadcast_snapshot(false).is_ok() {}
            }
            let (mut client, server) = socket_pair();
            client
                .write_all(b"GET /api/state HTTP/1.1\r\n\r\n")
                .unwrap();
            let (done, result) = std::sync::mpsc::channel();
            let worker = thread::spawn(move || {
                done.send(handle(
                    server,
                    &host,
                    Stop::default(),
                    Duration::from_millis(150),
                ))
                .unwrap();
            });
            let finished = result.recv_timeout(Duration::from_secs(2));
            drop(release);
            worker.join().unwrap();
            assert!(finished.unwrap().is_err());
            client
                .set_read_timeout(Some(Duration::from_secs(1)))
                .unwrap();
            assert_eq!(client.read(&mut [0]).unwrap(), 0);
        }
    }

    #[test]
    fn shutdown_interrupts_http_waits_on_a_full_host_mailbox() {
        let (_host, host) = simulation();
        let release = host.stall();
        while host.broadcast_snapshot(false).is_ok() {}
        let server = HttpServer::bind("127.0.0.1:0", host).unwrap();
        let address = server.local_addr();
        let mut client = TcpStream::connect(address).unwrap();
        client
            .write_all(b"GET /api/state HTTP/1.1\r\n\r\n")
            .unwrap();
        thread::sleep(Duration::from_millis(50));
        let (done, result) = std::sync::mpsc::channel();
        let worker = thread::spawn(move || {
            drop(server);
            done.send(()).unwrap();
        });
        let stopped = result.recv_timeout(Duration::from_secs(2));
        drop(release);
        worker.join().unwrap();
        stopped.unwrap();
        let _rebound = TcpListener::bind(address).unwrap();
    }

    #[test]
    fn http_cannot_place_a_pilots_chassis() {
        let host = Host::new(
            Simulation::new(Field::new(&FieldConfig::default()).unwrap(), true),
            true,
        )
        .unwrap();
        let handle = host.handle();
        let response = respond(
            &Request {
                method: "POST".into(),
                path: "/api/command".into(),
                body: serde_json::to_vec(&Command::PlaceChassis {
                    chassis: 0,
                    position_m: [0.0; 3],
                    yaw_deg: 0.0,
                })
                .unwrap(),
            },
            &handle,
        );
        assert_eq!(response.status, 403);
    }
    #[test]
    fn compact_clock_and_control_endpoints_follow_worker_order() {
        let (simulation, ids) = crate::workload::simulation(1);
        let host = Host::new(simulation, true).unwrap();
        let handle = host.handle();
        let get = |path: &str| {
            respond(
                &Request {
                    method: "GET".into(),
                    path: path.into(),
                    body: vec![],
                },
                &handle,
            )
        };
        let clock = get("/api/clock");
        assert_eq!(clock.status, 200);
        assert!(clock.body.len() < 256);
        let clock: crate::simulation::ControlStatus = serde_json::from_slice(&clock.body).unwrap();
        assert_eq!(clock.time_ns, 0);
        assert!(clock.chassis.is_none());
        let robot = get(&format!("/api/control/{}", ids[0]));
        assert_eq!(robot.status, 200);
        let robot: crate::simulation::ControlStatus = serde_json::from_slice(&robot.body).unwrap();
        assert_eq!(robot.chassis.unwrap().id, ids[0]);
        assert_eq!(get("/api/control/999").status, 404);
        assert_eq!(get("/api/control/invalid").status, 400);
        handle.apply(&Command::Pause { paused: true }).unwrap();
        handle.apply(&Command::Step { ticks: 3 }).unwrap();
        let clock: crate::simulation::ControlStatus =
            serde_json::from_slice(&get("/api/clock").body).unwrap();
        assert!(clock.paused);
        assert_eq!(clock.input_epoch, 1);
        assert_eq!(clock.time_ns, 3 * rm_simulator_world::tick_ns());
    }
    #[test]
    fn requests_parse_and_route() {
        let (_host, simulation) = simulation();
        let page = respond(
            &request("GET /?x=1 HTTP/1.1\r\nHost: a\r\n\r\n"),
            &simulation,
        );
        assert_eq!(page.status, 200);
        assert!(std::str::from_utf8(&page.body).unwrap().contains("<title>"));
        let state = respond(&request("GET /api/state HTTP/1.1\r\n\r\n"), &simulation);
        let json: serde_json::Value = serde_json::from_slice(&state.body).unwrap();
        assert_eq!(json["paused"], true);
        assert_eq!(json["field"]["rmuc_2027"]["game"]["phase"], "Idle");
        let journal = respond(
            &request("GET /api/fire-records HTTP/1.1\r\n\r\n"),
            &simulation,
        );
        assert_eq!(journal.status, 200);
        assert_eq!(journal.body, b"[]");
        let body = r#"{"Referee":"StartMatch"}"#;
        let post = format!(
            "POST /api/command HTTP/1.1\r\nContent-Type: application/json\r\nContent-Length: {}\r\n\r\n{body}",
            body.len()
        );
        assert_eq!(respond(&request(&post), &simulation).status, 200);
        assert_eq!(
            simulation
                .state()
                .unwrap()
                .field
                .rmuc_2027
                .unwrap()
                .game
                .phase,
            rm_simulator_world::rmuc_2027::match_rules::Phase::Countdown
        );
        // A second start is refused with the referee's reason.
        let again = respond(&request(&post), &simulation);
        assert_eq!(again.status, 400);
        assert!(std::str::from_utf8(&again.body).unwrap().contains("reset"));
        let body = r#"{"Step":{"ticks":5500}}"#;
        let step = format!(
            "POST /api/command HTTP/1.1\r\nContent-Length: {}\r\n\r\n{body}",
            body.len()
        );
        assert_eq!(respond(&request(&step), &simulation).status, 200);
        let body = r#"{"ActivateRune":{"team":"Red"}}"#;
        let referee = format!(
            "POST /api/referee HTTP/1.1\r\nContent-Length: {}\r\n\r\n{body}",
            body.len()
        );
        assert_eq!(respond(&request(&referee), &simulation).status, 200);
        assert_eq!(
            respond(
                &request("POST /api/command HTTP/1.1\r\nContent-Length: 2\r\n\r\n{}"),
                &simulation
            )
            .status,
            400
        );
        assert_eq!(
            respond(&request("GET /nope HTTP/1.1\r\n\r\n"), &simulation).status,
            404
        );
        assert_eq!(
            respond(&request("GET /api/command HTTP/1.1\r\n\r\n"), &simulation).status,
            405
        );
        assert!(
            read_request(&mut BufReader::new(&b""[..]))
                .unwrap()
                .is_none()
        );
        assert!(read_request(&mut BufReader::new(&b"GET\r\n\r\n"[..])).is_err());
        let mut out = Vec::new();
        write_response(&mut out, &Response::error(404, "x")).unwrap();
        let text = String::from_utf8(out).unwrap();
        assert!(text.starts_with("HTTP/1.1 404 Not Found\r\n"));
        assert!(text.ends_with("\r\n\r\n{\"error\":\"x\"}"));
    }
    #[test]
    fn the_bound_server_answers_over_tcp() {
        let (_host, simulation) = simulation();
        let server = HttpServer::bind("127.0.0.1:0", simulation).unwrap();
        let mut stream = TcpStream::connect(server.local_addr()).unwrap();
        stream
            .write_all(b"GET /api/state HTTP/1.1\r\nHost: x\r\n\r\n")
            .unwrap();
        let mut text = String::new();
        stream.read_to_string(&mut text).unwrap();
        assert!(text.starts_with("HTTP/1.1 200 OK"), "{text}");
        let body = text.split("\r\n\r\n").nth(1).unwrap();
        let json: serde_json::Value = serde_json::from_str(body).unwrap();
        assert_eq!(json["field"]["tick"], 0);
    }

    #[test]
    fn dropping_http_releases_listener() {
        let (mut host, simulation) = simulation();
        let server = HttpServer::bind("127.0.0.1:0", simulation.clone()).unwrap();
        let address = server.local_addr();
        let mut request = TcpStream::connect(address).unwrap();
        request.write_all(b"GET / HTTP/1.1\r\n").unwrap();
        drop(server);
        let _rebound = TcpListener::bind(address).unwrap();
        host.shutdown();
        assert!(simulation.apply(&Command::Pause { paused: true }).is_err());
        let response = respond(
            &Request {
                method: "POST".into(),
                path: "/api/command".into(),
                body: serde_json::to_vec(&Command::Pause { paused: true }).unwrap(),
            },
            &simulation,
        );
        assert_eq!(response.status, 503);
        assert_eq!(simulation.state().unwrap().field.tick, 0);
    }

    #[test]
    fn hostile_request_heads_are_refused() {
        let flood = format!("GET / HTTP/1.1\r\n{}", "X-A: b\r\n".repeat(MAX_HEADERS + 1));
        assert!(read_request(&mut BufReader::new(flood.as_bytes())).is_err());
        let long = format!(
            "GET /{} HTTP/1.1\r\n\r\n",
            "a".repeat(MAX_HEADER_LINE_BYTES)
        );
        assert!(read_request(&mut BufReader::new(long.as_bytes())).is_err());
        assert!(read_request(&mut BufReader::new(io::repeat(b'a'))).is_err());
        let mut endless_headers =
            BufReader::new((&b"GET / HTTP/1.1\r\n"[..]).chain(io::repeat(b'a')));
        assert!(read_request(&mut endless_headers).is_err());
        // The panel is same-origin; other pages get no cross-origin grant.
        let mut out = Vec::new();
        write_response(&mut out, &Response::error(404, "x")).unwrap();
        assert!(!String::from_utf8(out).unwrap().contains("Access-Control"));
    }
    #[test]
    fn http_edits_live_resources_equipment_and_rejects_invalid_purchases() {
        let config = crate::workload::field_config();
        let mut field = Field::new(&config).unwrap();
        let id = field
            .add_chassis(&rm_simulator_world::ChassisPlacement {
                config: Default::default(),
                spawn: rm_simulator_world::Pose::at([0.0, 0.0, 1.0]),
                team: rm_simulator_world::Team::Red,
                kind: rm_simulator_world::RobotKind::Infantry,
                performance: None,
            })
            .unwrap();
        let host = Host::new(Simulation::new(field, true), true).unwrap();
        let simulation = host.handle();
        let post = |value: serde_json::Value| {
            respond(
                &Request {
                    method: "POST".into(),
                    path: "/api/referee".into(),
                    body: serde_json::to_vec(&value).unwrap(),
                },
                &simulation,
            )
        };
        assert_eq!(post(serde_json::json!({"SetPolicy":{"enforce_allowance":true,"exchange_requires_zone":false}})).status, 400);
        assert_eq!(post(serde_json::json!("StartMatch")).status, 200);
        simulation.apply(&Command::Step { ticks: 6000 }).unwrap();
        let game = || simulation.state().unwrap().field.rmuc_2027.unwrap().game;
        assert_eq!(
            post(serde_json::json!({"SetGold":{"team":"Red","gold":1000}})).status,
            200
        );
        let gold = game().gold[0];
        assert_eq!(
            post(serde_json::json!({"BuyAmmo":{"robot":id,"caliber":"Mm17","amount":100}})).status,
            200
        );
        simulation.apply(&Command::Step { ticks: 768 }).unwrap();
        assert_eq!(game().gold[0], gold - 150);
        assert_eq!(game().robots[0].combat.allowance, 100);
        let before = simulation.state().unwrap();
        assert_eq!(
            post(serde_json::json!({"BuyAmmo":{"robot":id,"caliber":"Mm42","amount":2}})).status,
            400
        );
        assert_eq!(simulation.state().unwrap(), before);
        assert_eq!(
            post(serde_json::json!({"SetAllowance":{"robot":id,"allowance":[50,0]}})).status,
            200
        );
        assert_eq!(
            post(serde_json::json!({"SetGold":{"team":"Blue","gold":123}})).status,
            200
        );
        assert_eq!(
            post(serde_json::json!({"SetOutpostHp":{"outpost":0,"hp":750}})).status,
            200
        );
        assert_eq!(
            post(serde_json::json!({"SetRuneOpportunities":{"team":"Red","opportunities":3}}))
                .status,
            400
        );
        assert_eq!(post(serde_json::json!({"SetRuneBuff":{"team":"Red","defense_pct":100,"attack_pct":100,"cooling_multiplier":1,"duration_ns":1000000000_u64}})).status, 400);
        assert_eq!(
            post(serde_json::json!({"SetBaseOpen":{"team":"Red","open":true}})).status,
            200
        );
        assert_eq!(
            post(serde_json::json!({"SetDartDoorOpen":{"team":"Blue","open":false}})).status,
            400
        );
        assert_eq!(
            post(serde_json::json!({"SetBaseOpen":{"team":"Green","open":true}})).status,
            400
        );
        let state = respond(
            &Request {
                method: "GET".into(),
                path: "/api/state".into(),
                body: Vec::new(),
            },
            &simulation,
        );
        let json: serde_json::Value = serde_json::from_slice(&state.body).unwrap();
        let game = &json["field"]["rmuc_2027"]["game"];
        assert_eq!(game["gold"][1], 123);
        assert_eq!(game["robots"][0]["combat"]["allowance"], 50);
        assert_eq!(game["outpost_hp"][0], 750);
        assert_eq!(game["base_open"], serde_json::json!([true, false]));
        let before = simulation.state().unwrap();
        assert_eq!(post(serde_json::json!({"SetRuneBuff":{"team":"Red","defense_pct":101,"attack_pct":100,"cooling_multiplier":1,"duration_ns":1000000000_u64}})).status, 400);
        assert_eq!(simulation.state().unwrap(), before);
    }
}
