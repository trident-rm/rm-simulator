// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-polled WebSocket client for the ordinary application session.
use rm_sim_server::{
    clock::{Instant, TimeSource},
    protocol::{Command, PlayerInfo, Role, ServerMessage, Welcome},
    state::SimulationState,
    udp_codec::{ClientCodec, ClientEvent, QueuedCommand},
};
use std::{
    cell::RefCell,
    collections::{HashMap, VecDeque},
    io,
    sync::atomic::{AtomicU64, Ordering},
    time::Duration,
};
use wasm_bindgen::{JsCast, closure::Closure};

struct Carrier {
    socket: web_sys::WebSocket,
    incoming: std::rc::Rc<RefCell<VecDeque<Vec<u8>>>>,
    _message: Closure<dyn FnMut(web_sys::MessageEvent)>,
}
impl Drop for Carrier {
    fn drop(&mut self) {
        self.socket.set_onmessage(None);
        let _ = self.socket.close();
    }
}
thread_local! { static CARRIERS: RefCell<HashMap<u64, Carrier>> = RefCell::new(HashMap::new()); }
static NEXT_CARRIER: AtomicU64 = AtomicU64::new(1);

/// The session owns portable protocol state; browser objects stay on their JS thread.
pub(crate) struct Client {
    id: u64,
    codec: ClientCodec,
    welcome: Option<Welcome>,
    latest: Option<SimulationState>,
    roster: Vec<PlayerInfo>,
    disconnected: Option<String>,
    hello: Option<Vec<u8>>,
    time: TimeSource,
    epoch: Instant,
    clock: rm_sim_server::presentation_clock::ClockEstimate,
    lead: rm_sim_server::presentation_clock::InputLead,
    last_packet: Instant,
    blocked: Option<Instant>,
    sent_confirmation: u64,
    acknowledged: u64,
    pending_probe: Option<(u64, Instant)>,
    last_probe: Instant,
    rtts: VecDeque<u64>,
    rtt_samples: rm_sim_server::network_trace::EventSamples,
    checkpoint_samples: rm_sim_server::network_trace::EventSamples,
    last_checkpoint: Option<Instant>,
    delivery: Option<rm_sim_server::pacing::QueueStats>,
    telemetry: Option<rm_sim_server::network_stats::HostTelemetry>,
    owner: Option<rm_sim_server::owner_stream::OwnerAnchor>,
    hits: Vec<(u64, u64, rm_sim_world::ArmorHit)>,
    scheduled: Vec<(u32, u64)>,
    results: Vec<rm_sim_server::protocol::ShotResult>,
    map: Vec<rm_sim_server::map_interaction::Event>,
    notices: Vec<String>,
}
impl Drop for Client {
    fn drop(&mut self) {
        CARRIERS.with(|carriers| {
            carriers.borrow_mut().remove(&self.id);
        });
    }
}
impl Client {
    /// Complete admission and obtain the first state without blocking browser events.
    pub async fn connect(args: &crate::args::Args) -> anyhow::Result<Self> {
        let address = args
            .connect
            .as_deref()
            .ok_or_else(|| anyhow::anyhow!("browser sessions require a WebSocket server"))?;
        anyhow::ensure!(
            address.starts_with("ws://") || address.starts_with("wss://"),
            "browser server must use ws:// or wss://"
        );
        let role = if args.staging && !args.referee {
            Role::Spectator
        } else {
            args.role()
        };
        anyhow::ensure!(
            role != Role::Referee,
            "this browser endpoint admits pilots and spectators; referee authority requires the native operator connection"
        );
        let hello = ClientCodec::hello_with_chassis(
            &args.name,
            Some(args.team.into()),
            role,
            args.robot,
            args.chassis,
            &args.password,
        )?;
        let socket = web_sys::WebSocket::new(address)
            .map_err(|e| anyhow::anyhow!("opening WebSocket: {e:?}"))?;
        socket.set_binary_type(web_sys::BinaryType::Arraybuffer);
        let incoming = std::rc::Rc::new(RefCell::new(VecDeque::<Vec<u8>>::new()));
        let queue = std::rc::Rc::clone(&incoming);
        let close = socket.clone();
        let message = Closure::wrap(Box::new(move |event: web_sys::MessageEvent| {
            let Ok(buffer) = event.data().dyn_into::<js_sys::ArrayBuffer>() else {
                let _ = close.close();
                return;
            };
            let bytes = js_sys::Uint8Array::new(&buffer);
            let mut queue = queue.borrow_mut();
            if queue.len() >= 4096
                || queue.iter().map(Vec::len).sum::<usize>() + bytes.length() as usize
                    > 4 * 1024 * 1024
            {
                let _ = close.close();
                return;
            }
            queue.push_back(bytes.to_vec());
        }) as Box<dyn FnMut(_)>);
        socket.set_onmessage(Some(message.as_ref().unchecked_ref()));
        let id = NEXT_CARRIER.fetch_add(1, Ordering::Relaxed);
        CARRIERS.with(|carriers| {
            carriers.borrow_mut().insert(
                id,
                Carrier {
                    socket,
                    incoming,
                    _message: message,
                },
            )
        });
        let time = TimeSource::system();
        let now = time.now();
        let mut client = Self {
            id,
            codec: ClientCodec::new(now, 1_000_000, 12, false),
            welcome: None,
            latest: None,
            roster: Vec::new(),
            disconnected: None,
            hello: Some(hello),
            time,
            epoch: now,
            clock: Default::default(),
            lead: Default::default(),
            last_packet: now,
            blocked: None,
            sent_confirmation: 0,
            acknowledged: 0,
            pending_probe: None,
            last_probe: now,
            rtts: VecDeque::new(),
            rtt_samples: Default::default(),
            checkpoint_samples: Default::default(),
            last_checkpoint: None,
            delivery: None,
            telemetry: None,
            owner: None,
            hits: Vec::new(),
            scheduled: Vec::new(),
            results: Vec::new(),
            map: Vec::new(),
            notices: Vec::new(),
        };
        while client.welcome.is_none() || client.latest.is_none() {
            client.poll();
            if let Some(reason) = &client.disconnected {
                anyhow::bail!("{reason}");
            }
            anyhow::ensure!(
                now.elapsed() < Duration::from_secs(15),
                "server admission timed out"
            );
            let promise = js_sys::Promise::new(&mut |resolve, _| {
                if let Some(window) = web_sys::window() {
                    let _ =
                        window.set_timeout_with_callback_and_timeout_and_arguments_0(&resolve, 10);
                }
            });
            wasm_bindgen_futures::JsFuture::from(promise)
                .await
                .map_err(|e| anyhow::anyhow!("browser timer: {e:?}"))?;
        }
        Ok(client)
    }
    /// The admitted host seat; construction completes only after this exists.
    pub fn welcome(&self) -> &Welcome {
        self.welcome.as_ref().expect("connect completed admission")
    }
    /// Use one caller clock for probes, packet expiry and presentation.
    pub fn set_time_source(&mut self, time: TimeSource) {
        self.time = time;
        self.epoch = self.time.now();
        self.clock = Default::default();
    }
    fn elapsed_ns(&self) -> u64 {
        self.time.since(self.epoch).as_nanos().min(u64::MAX as u128) as u64
    }
    /// Queue one command through the ordinary ordered client codec.
    pub fn send(&mut self, command: Command) -> io::Result<()> {
        self.queue(QueuedCommand {
            command: Some(command),
            confirmation: None,
            time_probe: None,
        })
    }
    /// Queue a command and request the host state barrier following it.
    pub fn send_confirmed(&mut self, command: Command) -> io::Result<()> {
        self.barrier(Some(command))
    }
    fn barrier(&mut self, command: Option<Command>) -> io::Result<()> {
        let nonce = self
            .sent_confirmation
            .checked_add(1)
            .ok_or_else(|| io::Error::other("confirmation overflow"))?;
        self.queue(QueuedCommand {
            command,
            confirmation: Some(nonce),
            time_probe: None,
        })?;
        self.sent_confirmation = nonce;
        Ok(())
    }
    fn queue(&mut self, queued: QueuedCommand) -> io::Result<()> {
        if let Some(reason) = &self.disconnected {
            return Err(io::Error::new(io::ErrorKind::NotConnected, reason.clone()));
        }
        let result = self.codec.submit(queued, self.time.now());
        if let Err(error) = &result {
            self.disconnected = Some(error.to_string());
            CARRIERS.with(|carriers| {
                if let Some(carrier) = carriers.borrow().get(&self.id) {
                    let _ = carrier.socket.close();
                }
            });
        }
        result
    }
    /// Whether the host answered the newest queued state barrier.
    pub fn commands_confirmed(&self) -> bool {
        self.acknowledged >= self.sent_confirmation
    }
    /// Drain bounded browser packets and pump outgoing codec work without blocking.
    pub fn poll(&mut self) {
        if self.disconnected.is_none()
            && let Err(error) = self.pump()
        {
            self.disconnected = Some(error.to_string());
            CARRIERS.with(|carriers| {
                if let Some(carrier) = carriers.borrow().get(&self.id) {
                    let _ = carrier.socket.close();
                }
            });
        }
    }
    fn pump(&mut self) -> io::Result<()> {
        let now = self.time.now();
        let (socket, incoming) = CARRIERS.with(|carriers| {
            let carriers = carriers.borrow();
            let carrier = carriers
                .get(&self.id)
                .ok_or_else(|| io::Error::other("browser carrier removed"))?;
            Ok::<_, io::Error>((
                carrier.socket.clone(),
                std::rc::Rc::clone(&carrier.incoming),
            ))
        })?;
        if socket.ready_state() >= web_sys::WebSocket::CLOSING {
            return Err(io::Error::other("server disconnected"));
        }
        if self.time.since(self.last_packet) > Duration::from_secs(15) {
            return Err(io::Error::new(
                io::ErrorKind::TimedOut,
                "server stopped responding",
            ));
        }
        if socket.ready_state() != web_sys::WebSocket::OPEN {
            return Ok(());
        }
        let send = |bytes: &[u8]| {
            socket
                .send_with_u8_array(bytes)
                .map_err(|e| io::Error::other(format!("WebSocket send: {e:?}")))
        };
        if let Some(hello) = self.hello.take() {
            send(&hello)?;
        }
        for packet in incoming.borrow_mut().drain(..) {
            self.last_packet = now;
            match self.codec.receive(&packet, now)? {
                Some(ClientEvent::Welcome(welcome)) => {
                    self.welcome = Some(*welcome);
                }
                Some(ClientEvent::Anchor(anchor)) => {
                    self.clock.observe(
                        anchor.input_epoch,
                        self.elapsed_ns(),
                        anchor.time_ns,
                        anchor.paused,
                    );
                    self.owner = Some(*anchor);
                }
                Some(ClientEvent::Message(message)) => self.receive(message, now)?,
                None => {}
            }
        }
        self.codec.acknowledge(now)?;
        if socket.buffered_amount() >= 256 * 1024 {
            let since = *self.blocked.get_or_insert(now);
            if self.time.since(since) >= Duration::from_secs(2) {
                return Err(io::Error::other("WebSocket send queue stalled"));
            }
        } else {
            self.blocked = None;
            while socket.buffered_amount() < 256 * 1024 {
                let Some(packet) = self.codec.next(now)? else {
                    break;
                };
                send(&packet.bytes)?;
            }
        }
        Ok(())
    }
    fn receive(&mut self, message: ServerMessage, now: Instant) -> io::Result<()> {
        let message = match message {
            ServerMessage::Rejected { reason } => {
                ServerMessage::Notice(format!("rejected: {reason}"))
            }
            message => message,
        };
        match message {
            ServerMessage::Snapshot(state) => {
                if let Some(last) = self.last_checkpoint {
                    self.checkpoint_samples
                        .record(self.time.since(last).as_secs_f64() * 1000.);
                }
                self.last_checkpoint = Some(now);
                self.clock.observe(
                    state.input_epoch,
                    self.elapsed_ns(),
                    state.field.time_ns,
                    state.paused,
                );
                self.latest = Some(*state);
            }
            ServerMessage::Roster(roster) => self.roster = roster,
            ServerMessage::Pong { nonce } => self.acknowledged = self.acknowledged.max(nonce),
            ServerMessage::Hit {
                epoch,
                event_id,
                hit,
            } => {
                if self.hits.len() >= 4096 {
                    return Err(io::Error::other("unread contact event queue full"));
                }
                self.hits.push((epoch, event_id, hit));
            }
            ServerMessage::ShotScheduled { shooter, shot_id } => {
                if self.scheduled.len() >= 4096 {
                    return Err(io::Error::other("unread shot queue full"));
                }
                self.scheduled.push((shooter, shot_id));
            }
            ServerMessage::ShotResult(result) => {
                if self.results.len() >= 4096 {
                    return Err(io::Error::other("unread result queue full"));
                }
                self.results.push(result);
            }
            ServerMessage::TimeSample {
                nonce,
                time_ns,
                paused,
            } => {
                if let Some((pending, sent)) = self.pending_probe
                    && pending == nonce
                {
                    let rtt = self.time.since(sent).as_nanos().min(u64::MAX as u128) as u64;
                    self.rtts.push_back(rtt);
                    if self.rtts.len() > 8 {
                        self.rtts.pop_front();
                    }
                    self.rtt_samples.record(rtt as f64 / 1e6);
                    if rtt
                        <= self
                            .rtts
                            .iter()
                            .copied()
                            .min()
                            .unwrap_or(rtt)
                            .saturating_add(5_000_000)
                    {
                        self.clock.synchronize(
                            self.elapsed_ns(),
                            time_ns.saturating_add(rtt / 2),
                            paused,
                        );
                    }
                    self.pending_probe = None;
                }
            }
            ServerMessage::DeliveryStats(stats) => self.delivery = Some(stats),
            ServerMessage::Telemetry(stats) => {
                self.lead.observe(stats, self.initial_lead());
                self.telemetry = Some(stats);
            }
            ServerMessage::MapInteraction(event) => {
                if self.map.len() == 64 {
                    self.map.remove(0);
                }
                self.map.push(event);
            }
            ServerMessage::Deployed(receipt) => {
                if let Some(welcome) = &mut self.welcome {
                    welcome.team = Some(receipt.team);
                    welcome.role = Role::Pilot;
                    welcome.chassis = Some(receipt.chassis);
                    welcome.weapon = receipt.weapon;
                    welcome.weapon_limits = receipt.weapon_limits;
                }
            }
            ServerMessage::WeaponChanged { weapon, limits } => {
                if let Some(welcome) = &mut self.welcome {
                    welcome.weapon = weapon;
                    welcome.weapon_limits = limits;
                }
            }
            ServerMessage::Notice(text) | ServerMessage::Rejected { reason: text } => {
                if self.notices.len() >= 256
                    || self.notices.iter().map(String::len).sum::<usize>() + text.len()
                        > 1024 * 1024
                {
                    return Err(io::Error::other("unread notice queue full"));
                }
                self.notices.push(text);
            }
            ServerMessage::Welcome(_) | ServerMessage::OwnerConfig(_) => {}
        }
        Ok(())
    }
    /// The latest complete authoritative state retained for startup.
    pub fn state(&self) -> Option<&SimulationState> {
        self.latest.as_ref()
    }
    /// Take the newest complete authoritative state once.
    pub fn take_state(&mut self) -> Option<SimulationState> {
        self.latest.take()
    }
    /// The newest complete roster published by the host.
    pub fn roster(&self) -> &[PlayerInfo] {
        &self.roster
    }
    /// Drain ordered notices and rejected-command reasons.
    pub fn take_notices(&mut self) -> Vec<String> {
        std::mem::take(&mut self.notices)
    }
    /// The actual carrier or codec failure, once detected.
    pub fn disconnected(&self) -> Option<&str> {
        self.disconnected.as_deref()
    }
    /// Drain the newest bounded team map intents in host order.
    pub fn take_map_interactions(&mut self) -> Vec<rm_sim_server::map_interaction::Event> {
        std::mem::take(&mut self.map)
    }
    /// Drain admitted shot identities; these are not execution confirmations.
    pub fn take_scheduled_shots(&mut self) -> Vec<(u32, u64)> {
        std::mem::take(&mut self.scheduled)
    }
    /// Drain reliable scored contacts in host order.
    pub fn take_hits(&mut self) -> Vec<(u64, u64, rm_sim_world::ArmorHit)> {
        std::mem::take(&mut self.hits)
    }
    /// Drain authoritative shot execution or rejection results.
    pub fn take_shot_results(&mut self) -> Vec<rm_sim_server::protocol::ShotResult> {
        std::mem::take(&mut self.results)
    }
    /// Take the newest own-chassis anchor once.
    pub fn take_owner_anchor(&mut self) -> Option<rm_sim_server::owner_stream::OwnerAnchor> {
        self.owner.take()
    }
    /// The latest host downstream queue counters, if published.
    pub fn delivery_stats(&self) -> Option<&rm_sim_server::pacing::QueueStats> {
        self.delivery.as_ref()
    }
    /// No native disk trace exists for the browser carrier.
    pub fn trace_report(&self) -> Option<rm_sim_server::network_trace::Report> {
        None
    }
    /// The latest actual host input-arrival sample, if published.
    pub fn host_telemetry(&self) -> Option<rm_sim_server::network_stats::HostTelemetry> {
        self.telemetry
    }
    /// The latest measured timing-probe round trip, in nanoseconds.
    pub fn round_trip_ns(&self) -> Option<u64> {
        self.rtts.back().copied()
    }
    /// Bounded measured timing-probe round trips, in milliseconds.
    pub fn round_trip_samples(&self) -> &rm_sim_server::network_trace::EventSamples {
        &self.rtt_samples
    }
    /// Bounded intervals between decoded complete checkpoints, in milliseconds.
    pub fn checkpoint_interval_samples(
        &self,
    ) -> Option<rm_sim_server::network_trace::EventSamples> {
        Some(self.checkpoint_samples.clone())
    }
    /// The latest probe RTT or outstanding probe age, in nanoseconds.
    pub fn response_delay_ns(&self) -> Option<u64> {
        self.round_trip_ns()
            .into_iter()
            .chain(
                self.pending_probe
                    .map(|(_, at)| self.time.since(at).as_nanos().min(u64::MAX as u128) as u64),
            )
            .max()
    }
    /// Rebase presentation after loading without advancing host physics.
    pub fn reset_presentation_clock(&mut self, time_ns: u64, paused: bool) {
        self.clock = Default::default();
        self.clock.observe(
            self.latest.as_ref().map_or(0, |s| s.input_epoch),
            self.elapsed_ns(),
            time_ns,
            paused,
        );
    }
    /// Reset input-arrival feedback across a host epoch or chassis placement.
    pub fn reset_input_timing(&mut self, epoch: u64, revision: u64) {
        self.lead.reset(epoch, revision);
    }
    fn initial_lead(&self) -> u64 {
        (self.rtts.iter().copied().min().unwrap_or(0) / 2 + 32_000_000).min(150_000_000)
    }
    /// The shared bounded control lead from actual host feedback, in nanoseconds.
    pub fn input_lead_ns(&self) -> u64 {
        self.lead.get(self.initial_lead())
    }
    /// The bounded client estimate of authoritative simulation time, in nanoseconds.
    pub fn presentation_time_ns(&self) -> u64 {
        self.clock.at(self.elapsed_ns())
    }
    /// Queue at most one timing probe per second with a bounded outstanding age.
    pub fn synchronize_clock(&mut self) -> io::Result<()> {
        let now = self.time.now();
        if self.time.since(self.last_probe) < Duration::from_secs(1)
            || self
                .pending_probe
                .is_some_and(|(_, at)| self.time.since(at) < Duration::from_secs(3))
        {
            return Ok(());
        }
        let nonce = self.elapsed_ns();
        self.queue(QueuedCommand {
            command: None,
            confirmation: None,
            time_probe: Some(nonce),
        })?;
        self.pending_probe = Some((nonce, now));
        self.last_probe = now;
        Ok(())
    }
    /// Report actual codec counters and probes; unavailable socket measurements stay absent.
    pub fn network_stats(&self) -> rm_sim_server::network_stats::NetworkStats {
        rm_sim_server::network_stats::NetworkStats {
            schema_version: 1,
            connection_generation: u64::from(self.welcome().client_id),
            transport: "websocket".into(),
            sampled_elapsed_ms: self.elapsed_ns() as f64 / 1e6,
            stale: self.disconnected.is_some(),
            native_sample_age_ms: None,
            native: Some(self.codec.stats()),
            native_rate_source: "browser codec counters; socket delivery is ordered",
            loss_percent: None,
            loss_unavailable_reason: "WebSocket does not expose datagram loss",
            app_rtt_ms: self.round_trip_ns().map(|ns| ns as f64 / 1e6),
            pending_response_age_ms: self
                .pending_probe
                .map(|(_, at)| self.time.since(at).as_secs_f64() * 1000.),
        }
    }
}
