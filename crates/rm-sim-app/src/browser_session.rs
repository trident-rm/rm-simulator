// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Browser-owned transport and authoritative snapshot presentation, without prediction.
use bevy::prelude::*;
use rm_sim_render::sync::{SceneInput, SceneSyncSet};
use rm_sim_server::{
    protocol::{Command, Role, ServerMessage},
    udp_codec::{ClientCodec, ClientEvent, Instant, QueuedCommand},
};
use std::{cell::RefCell, collections::VecDeque, rc::Rc, time::Duration};
use wasm_bindgen::{JsCast, closure::Closure};

const MAX_BUFFERED: u32 = 256 * 1024;
const MAX_INBOX_BYTES: usize = 4 * 1024 * 1024;

pub(super) struct BrowserSession {
    socket: web_sys::WebSocket,
    inbox: Rc<RefCell<VecDeque<Vec<u8>>>>,
    _message: Closure<dyn FnMut(web_sys::MessageEvent)>,
    codec: ClientCodec,
    hello: Option<Vec<u8>>,
    expected: rm_sim_server::competition::Competition,
    chassis: Option<u32>,
    aim: [f64; 2],
    next_input: Instant,
    next_fire: Instant,
    failed: bool,
    last_receive: Instant,
    blocked_since: Option<Instant>,
}
impl Drop for BrowserSession {
    fn drop(&mut self) {
        self.socket.set_onmessage(None);
        let _ = self.socket.close();
    }
}
pub(super) fn install(
    app: &mut App,
    url: &str,
    query: &web_sys::UrlSearchParams,
) -> anyhow::Result<()> {
    use rm_sim_server::{competition::Competition, protocol::Robot};
    anyhow::ensure!(
        url.starts_with("ws://") || url.starts_with("wss://"),
        "server must use ws:// or wss://"
    );
    let expected = match query.get("field").as_deref().unwrap_or("1v1") {
        "1v1" => Competition::RmulInfantry2027,
        "3v3" => Competition::Rmul2027,
        _ => Competition::Rmuc2027,
    };
    let team = match query.get("team").as_deref() {
        Some("blue") => rm_sim_world::Team::Blue,
        _ => rm_sim_world::Team::Red,
    };
    let socket =
        web_sys::WebSocket::new(url).map_err(|e| anyhow::anyhow!("opening server: {e:?}"))?;
    socket.set_binary_type(web_sys::BinaryType::Arraybuffer);
    let inbox = Rc::new(RefCell::new(VecDeque::new()));
    let queue = Rc::clone(&inbox);
    let close = socket.clone();
    let message = Closure::wrap(Box::new(move |event: web_sys::MessageEvent| {
        let Ok(buffer) = event.data().dyn_into::<js_sys::ArrayBuffer>() else {
            let _ = close.close();
            return;
        };
        let array = js_sys::Uint8Array::new(&buffer);
        let mut queue = queue.borrow_mut();
        let bytes: usize = queue.iter().map(Vec::len).sum();
        if bytes + array.length() as usize > MAX_INBOX_BYTES {
            let _ = close.close();
            return;
        }
        queue.push_back(array.to_vec());
    }) as Box<dyn FnMut(_)>);
    socket.set_onmessage(Some(message.as_ref().unchecked_ref()));
    let now = Instant::now();
    let hello = ClientCodec::hello_with_chassis(
        &query.get("name").unwrap_or_else(|| "Browser pilot".into()),
        Some(team),
        Role::Pilot,
        Robot::Infantry3,
        rm_sim_server::protocol::Chassis::Auto,
        &query.get("password").unwrap_or_default(),
    )?;
    app.insert_non_send(BrowserSession {
        socket,
        inbox,
        _message: message,
        codec: ClientCodec::new(now, 1_000_000, 12, false),
        hello: Some(hello),
        expected,
        chassis: None,
        aim: [
            if team == rm_sim_world::Team::Red {
                std::f64::consts::PI
            } else {
                0.
            },
            0.,
        ],
        next_input: now,
        next_fire: now,
        failed: false,
        last_receive: now,
        blocked_since: None,
    });
    app.add_plugins((
        rm_sim_render::chassis::ChassisVisualsPlugin {
            rendering: Default::default(),
            armor: rm_sim_presentation::armor_optics(),
        },
        rm_sim_render::projectile::ProjectileVisualsPlugin {
            rendering: Default::default(),
        },
    ))
    .add_systems(Update, update.before(SceneSyncSet));
    super::status("Connecting to server…", "connecting");
    Ok(())
}
fn update(
    mut session: NonSendMut<BrowserSession>,
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut scene: ResMut<SceneInput>,
) {
    if session.failed {
        return;
    }
    if let Err(error) = advance(&mut session, &keys, time.delta_secs_f64(), &mut scene) {
        session.failed = true;
        let _ = session.socket.close();
        super::status(&format!("Connection failed: {error:#}"), "error");
    }
}
fn advance(
    session: &mut BrowserSession,
    keys: &ButtonInput<KeyCode>,
    dt: f64,
    scene: &mut SceneInput,
) -> anyhow::Result<()> {
    let now = Instant::now();
    anyhow::ensure!(
        now.saturating_duration_since(session.last_receive) < Duration::from_secs(15),
        "server did not respond within 15 seconds"
    );
    anyhow::ensure!(
        session.socket.ready_state() != web_sys::WebSocket::CLOSED,
        "server disconnected"
    );
    if session.socket.ready_state() != web_sys::WebSocket::OPEN {
        return Ok(());
    }
    if let Some(hello) = session.hello.take() {
        send(&session.socket, &hello)?;
    }
    let packets = std::mem::take(&mut *session.inbox.borrow_mut());
    for packet in packets {
        session.last_receive = now;
        match session.codec.receive(&packet, now)? {
            Some(ClientEvent::Welcome(welcome)) => {
                anyhow::ensure!(
                    welcome.competition == session.expected,
                    "server competition differs from selected field"
                );
                session.chassis = welcome.chassis.map(|c| c.id);
                anyhow::ensure!(
                    session.chassis.is_some(),
                    "host did not grant a pilot chassis"
                );
            }
            Some(ClientEvent::Message(ServerMessage::Snapshot(state))) => {
                let flash = rm_sim_presentation::FlashSettings {
                    hit_flash_ns: 50_000_000,
                    rune_flash_hz: 0.,
                    rune_flashes: 3,
                };
                let mut next = rm_sim_presentation::scene_state_with_hits(
                    &state.field,
                    flash,
                    state.field.time_ns,
                    &[],
                );
                next.chassis.extend(
                    state
                        .field
                        .chassis
                        .iter()
                        .map(|c| rm_sim_presentation::chassis_appearance(c, &[], None)),
                );
                if let Some(own) = state
                    .field
                    .chassis
                    .iter()
                    .find(|c| Some(c.id) == session.chassis)
                {
                    let phase = state
                        .field
                        .rmul_2027
                        .as_ref()
                        .map(|game| format!("{:?}", game.phase))
                        .or_else(|| {
                            state
                                .field
                                .rmuc_2027
                                .as_ref()
                                .map(|state| format!("{:?}", state.game.phase))
                        });
                    if state.paused {
                        super::status("Connected · Simulation paused by host", "waiting");
                    } else if let Some(phase) = phase.as_deref().filter(|phase| *phase != "Running")
                    {
                        super::status(
                            &format!(
                                "Connected · Match {phase} · Waiting for host to start the round"
                            ),
                            "waiting",
                        );
                    } else {
                        super::status("WASD drive · Q/E turn · Arrows aim · Space fire", "playing");
                    }
                    if let Some(element) = web_sys::window()
                        .and_then(|w| w.document())
                        .and_then(|d| d.get_element_by_id("status"))
                    {
                        for (key, value) in [
                            ("data-chassis", own.id.to_string()),
                            ("data-team", format!("{:?}", own.team)),
                            (
                                "data-position",
                                serde_json::to_string(&own.pose.translation_m)?,
                            ),
                            ("data-snapshot", state.snapshot_id.to_string()),
                            (
                                "data-match-phase",
                                phase.unwrap_or_else(|| "Practice".into()),
                            ),
                            (
                                "data-projectiles",
                                state.field.projectiles.len().to_string(),
                            ),
                            ("data-shots", state.shot_results.len().to_string()),
                        ] {
                            let _ = element.set_attribute(key, &value);
                        }
                    }
                }
                scene.0 = Some(next);
            }
            Some(ClientEvent::Message(ServerMessage::Rejected { reason: error })) => {
                anyhow::bail!("{error}")
            }
            _ => {}
        }
    }
    session.codec.acknowledge(now)?;
    if let Some(chassis) = session.chassis {
        let axis = |positive, negative| {
            f64::from(keys.pressed(positive)) - f64::from(keys.pressed(negative))
        };
        session.aim[0] += axis(KeyCode::ArrowLeft, KeyCode::ArrowRight) * dt.min(0.1);
        session.aim[1] = (session.aim[1]
            + axis(KeyCode::ArrowUp, KeyCode::ArrowDown) * dt.min(0.1))
        .clamp(-0.3, 0.7);
        if now >= session.next_input {
            let command = rm_sim_world::ChassisCommand {
                forward_m_s: axis(KeyCode::KeyW, KeyCode::KeyS) * 2.,
                left_m_s: axis(KeyCode::KeyA, KeyCode::KeyD) * 2.,
                yaw_rate_rad_s: axis(KeyCode::KeyQ, KeyCode::KeyE),
                aim_yaw_rad: session.aim[0],
                aim_pitch_rad: session.aim[1],
                ..Default::default()
            };
            session.codec.submit(
                QueuedCommand {
                    command: Some(Command::Chassis { chassis, command }),
                    time_probe: None,
                    confirmation: None,
                },
                now,
            )?;
            session.next_input = now + Duration::from_millis(32);
        }
        if keys.pressed(KeyCode::Space) && now >= session.next_fire {
            session.codec.submit(
                QueuedCommand {
                    command: Some(Command::Fire { shooter: chassis }),
                    time_probe: None,
                    confirmation: None,
                },
                now,
            )?;
            session.next_fire = now + Duration::from_millis(100);
        }
    }
    if session.socket.buffered_amount() >= MAX_BUFFERED {
        let since = *session.blocked_since.get_or_insert(now);
        anyhow::ensure!(
            now.saturating_duration_since(since) < Duration::from_secs(2),
            "server send queue stalled"
        );
    } else {
        session.blocked_since = None;
    }
    while session.socket.buffered_amount() < MAX_BUFFERED {
        let Some(packet) = session.codec.next(now)? else {
            break;
        };
        send(&session.socket, &packet.bytes)?;
    }
    Ok(())
}
fn send(socket: &web_sys::WebSocket, bytes: &[u8]) -> anyhow::Result<()> {
    socket
        .send_with_u8_array(bytes)
        .map_err(|e| anyhow::anyhow!("sending to server: {e:?}"))
}
