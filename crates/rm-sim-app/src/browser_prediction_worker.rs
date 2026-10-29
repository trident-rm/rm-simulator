// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Persistent isolated prediction realms with one replaceable unsent request.
use crate::{prediction, projectile_prediction};
use rm_sim_server::prediction::{PredictionScene, Replay};
use serde::{Deserialize, Serialize};
use std::{
    cell::RefCell,
    collections::BTreeMap,
    path::PathBuf,
    rc::Rc,
    sync::atomic::{AtomicU64, Ordering},
};
use wasm_bindgen::{JsCast, closure::Closure};

const MAX_MESSAGE_BYTES: usize = 16 * 1024 * 1024;
const MAX_WORKERS: usize = 2;
static NEXT_ID: AtomicU64 = AtomicU64::new(1);
#[derive(Clone, Serialize, Deserialize)]
struct Source {
    root: PathBuf,
    scene: PredictionScene,
}
thread_local! {
    static SOURCE: RefCell<Option<Source>> = const { RefCell::new(None) };
    static WORKERS: RefCell<BTreeMap<u64, Carrier>> = const { RefCell::new(BTreeMap::new()) };
}
/// Pin workers to the same verified package/scene used by this session's replay geometry.
pub(crate) fn set_source(root: PathBuf, scene: PredictionScene) {
    SOURCE.with(|source| *source.borrow_mut() = Some(Source { root, scene }));
}
#[derive(Serialize, Deserialize)]
pub(crate) enum Request {
    Chassis {
        epoch: u64,
        replay: Replay,
        shots: Vec<prediction::ShotSample>,
    },
    Projectile(Box<projectile_prediction::Request>),
    ClearProjectile {
        epoch: u64,
    },
}
#[derive(Serialize, Deserialize)]
pub(crate) enum Response {
    Chassis(Box<prediction::Completion>),
    Projectile {
        output: projectile_prediction::Output,
        ticks: u64,
    },
    ClearedProjectile,
}
#[derive(Serialize, Deserialize)]
enum Message {
    Initialize(Source),
    Replay(Box<Request>),
}
#[derive(Serialize, Deserialize)]
enum Reply {
    Booted,
    Ready,
    Completed(Box<Response>),
    Failed(String),
}
type Completion = Box<dyn FnMut(Result<Response, String>)>;
struct State {
    source: Option<Source>,
    last_progress: rm_sim_server::clock::Instant,
    ready: bool,
    busy: bool,
    failed: Option<String>,
    pending: Option<Request>,
    completed: u64,
    replaced: u64,
    callback: Completion,
}
struct Carrier {
    worker: web_sys::Worker,
    state: Rc<RefCell<State>>,
    _message: Closure<dyn FnMut(web_sys::MessageEvent)>,
    _error: Closure<dyn FnMut(web_sys::ErrorEvent)>,
    _deadline: Closure<dyn FnMut()>,
    deadline_id: i32,
}
impl Drop for Carrier {
    fn drop(&mut self) {
        self.worker.set_onmessage(None);
        self.worker.set_onerror(None);
        self.worker.terminate();
        if let Some(window) = web_sys::window() {
            window.clear_interval_with_handle(self.deadline_id);
        }
    }
}
/// Send-safe numeric token; browser objects remain confined to their owning realm.
pub(crate) struct Handle(u64);
impl Handle {
    pub(crate) fn new(callback: Completion) -> std::io::Result<Self> {
        create(callback).map_err(std::io::Error::other)
    }
    pub(crate) fn post(&self, request: Request) -> Result<(), String> {
        WORKERS.with(|workers| {
            let workers = workers.borrow();
            let carrier = workers
                .get(&self.0)
                .ok_or("prediction worker was dropped")?;
            let mut state = carrier.state.borrow_mut();
            if let Some(error) = &state.failed {
                return Err(error.clone());
            }
            if state.pending.replace(request).is_some() {
                state.replaced += 1;
            }
            flush(&carrier.worker, &mut state)
        })
    }
    pub(crate) fn diagnostics(&self) -> serde_json::Value {
        WORKERS.with(|workers| {
            let workers=workers.borrow();
            let Some(carrier)=workers.get(&self.0) else { return serde_json::json!({"dropped":true}); };
            let state=carrier.state.borrow();
            serde_json::json!({"ready":state.ready,"inflight":state.busy,"queued":state.pending.is_some(),"completed":state.completed,"replaced":state.replaced,"error":state.failed})
        })
    }
}
impl Drop for Handle {
    fn drop(&mut self) {
        WORKERS.with(|workers| {
            workers.borrow_mut().remove(&self.0);
        });
    }
}
fn encode<T: Serialize>(value: &T) -> Result<String, String> {
    let json = serde_json::to_string(value).map_err(|e| e.to_string())?;
    if json.len() > MAX_MESSAGE_BYTES {
        return Err("prediction worker message exceeds 16 MiB".into());
    }
    Ok(json)
}
fn post(worker: &web_sys::Worker, message: &Message) -> Result<(), String> {
    worker
        .post_message(&encode(message)?.into())
        .map_err(|e| format!("prediction worker send: {e:?}"))
}
fn fail(worker: &web_sys::Worker, state: &mut State, error: String) {
    state.failed = Some(error.clone());
    state.pending = None;
    state.busy = false;
    worker.terminate();
    (state.callback)(Err(error));
}
fn flush(worker: &web_sys::Worker, state: &mut State) -> Result<(), String> {
    if !state.ready || state.busy {
        return Ok(());
    }
    let Some(request) = state.pending.take() else {
        return Ok(());
    };
    if let Err(error) = post(worker, &Message::Replay(Box::new(request))) {
        fail(worker, state, error.clone());
        return Err(error);
    }
    state.busy = true;
    state.last_progress = rm_sim_server::clock::Instant::now();
    Ok(())
}
fn create(callback: Completion) -> Result<Handle, String> {
    if WORKERS.with(|workers| workers.borrow().len()) >= MAX_WORKERS {
        return Err("prediction worker limit reached".into());
    }
    let source = SOURCE
        .with(|s| s.borrow().clone())
        .ok_or("prediction source is not installed")?;
    let options = web_sys::WorkerOptions::new();
    options.set_type(web_sys::WorkerType::Module);
    options.set_name("rm-prediction");
    let worker = web_sys::Worker::new_with_options("prediction-worker.js", &options)
        .map_err(|e| format!("starting prediction worker: {e:?}"))?;
    let state = Rc::new(RefCell::new(State {
        source: Some(source),
        last_progress: rm_sim_server::clock::Instant::now(),
        ready: false,
        busy: false,
        failed: None,
        pending: None,
        completed: 0,
        replaced: 0,
        callback,
    }));
    let socket = worker.clone();
    let output = Rc::clone(&state);
    let message = Closure::wrap(Box::new(move |event: web_sys::MessageEvent| {
        let mut state = output.borrow_mut();
        if state.failed.is_some() {
            return;
        }
        let result = event
            .data()
            .as_string()
            .ok_or_else(|| "prediction worker sent a non-string message".to_string())
            .and_then(|json| {
                if json.len() > MAX_MESSAGE_BYTES {
                    return Err("prediction worker reply exceeds 16 MiB".into());
                }
                serde_json::from_str::<Reply>(&json).map_err(|e| e.to_string())
            });
        state.last_progress = rm_sim_server::clock::Instant::now();
        match result {
            Ok(Reply::Booted) => {
                if let Some(source) = state.source.take()
                    && let Err(error) = post(&socket, &Message::Initialize(source))
                {
                    fail(&socket, &mut state, error);
                    return;
                }
            }
            Ok(Reply::Ready) => {
                state.ready = true;
            }
            Ok(Reply::Completed(reply)) => {
                state.busy = false;
                state.completed += 1;
                (state.callback)(Ok(*reply));
            }
            Ok(Reply::Failed(error)) | Err(error) => {
                fail(&socket, &mut state, error);
                return;
            }
        }
        let _ = flush(&socket, &mut state);
    }) as Box<dyn FnMut(_)>);
    worker.set_onmessage(Some(message.as_ref().unchecked_ref()));
    let socket = worker.clone();
    let output = Rc::clone(&state);
    let error = Closure::wrap(Box::new(move |event: web_sys::ErrorEvent| {
        event.prevent_default();
        let mut state = output.borrow_mut();
        if state.failed.is_none() {
            fail(
                &socket,
                &mut state,
                format!("prediction worker failed: {}", event.message()),
            );
        }
    }) as Box<dyn FnMut(_)>);
    worker.set_onerror(Some(error.as_ref().unchecked_ref()));
    let socket = worker.clone();
    let output = Rc::clone(&state);
    let deadline = Closure::wrap(Box::new(move || {
        let mut state = output.borrow_mut();
        if state.failed.is_none()
            && (!state.ready || state.busy)
            && state.last_progress.elapsed().as_secs() >= 30
        {
            fail(
                &socket,
                &mut state,
                "prediction worker made no progress for 30 seconds".into(),
            );
        }
    }) as Box<dyn FnMut()>);
    let window = web_sys::window().ok_or("prediction worker requires a browser window")?;
    let deadline_id = window
        .set_interval_with_callback_and_timeout_and_arguments_0(
            deadline.as_ref().unchecked_ref(),
            1000,
        )
        .map_err(|e| format!("prediction deadline: {e:?}"))?;
    let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
    WORKERS.with(|workers| {
        workers.borrow_mut().insert(
            id,
            Carrier {
                worker,
                state,
                _message: message,
                _error: error,
                _deadline: deadline,
                deadline_id,
            },
        )
    });
    Ok(Handle(id))
}
type Listener = Closure<dyn FnMut(web_sys::MessageEvent)>;
type Drivers = (
    prediction::ReplayDriver,
    projectile_prediction::ProjectileDriver,
);
thread_local! {
    static LISTENER: RefCell<Option<Listener>> = const {RefCell::new(None)};
    static DRIVERS: RefCell<Option<Drivers>> = const {RefCell::new(None)};
}
fn reply(scope: &web_sys::DedicatedWorkerGlobalScope, reply: Reply) {
    let json = encode(&reply).unwrap_or_else(|error| {
        serde_json::to_string(&Reply::Failed(error)).expect("static worker failure encodes")
    });
    let _ = scope.post_message(&json.into());
}
/// Claim only prediction realms, before the assembly worker's general realm detector.
pub(crate) fn start_if_worker() -> bool {
    let Ok(scope) = js_sys::global().dyn_into::<web_sys::DedicatedWorkerGlobalScope>() else {
        return false;
    };
    if js_sys::Reflect::get(scope.as_ref(), &"name".into())
        .ok()
        .and_then(|v| v.as_string())
        .as_deref()
        != Some("rm-prediction")
    {
        return false;
    }
    let output = scope.clone();
    let listener = Closure::wrap(Box::new(move |event: web_sys::MessageEvent| {
        let parsed = event
            .data()
            .as_string()
            .ok_or_else(|| "prediction request is not a string".to_string())
            .and_then(|json| {
                if json.len() > MAX_MESSAGE_BYTES {
                    return Err("prediction request exceeds 16 MiB".into());
                }
                serde_json::from_str::<Message>(&json).map_err(|e| e.to_string())
            });
        match parsed {
            Err(error) => reply(&output, Reply::Failed(error)),
            Ok(Message::Initialize(source)) => {
                let output = output.clone();
                wasm_bindgen_futures::spawn_local(async move {
                    let result = initialize(source).await;
                    match result {
                        Ok(drivers) => {
                            DRIVERS.with(|d| *d.borrow_mut() = Some(drivers));
                            reply(&output, Reply::Ready);
                        }
                        Err(error) => reply(&output, Reply::Failed(error)),
                    }
                });
            }
            Ok(Message::Replay(request)) => {
                let result: Result<Response, &'static str> = DRIVERS.with(|drivers| {
                    let mut drivers = drivers.borrow_mut();
                    let (chassis, projectile) = drivers
                        .as_mut()
                        .ok_or("prediction worker is not initialized")?;
                    Ok(match *request {
                        Request::Chassis {
                            epoch,
                            replay,
                            shots,
                        } => Response::Chassis(Box::new(chassis.run(epoch, &replay, shots, 8))),
                        Request::Projectile(request) => {
                            let output = projectile.run(&request, 4);
                            Response::Projectile {
                                output,
                                ticks: projectile.last_replay_ticks,
                            }
                        }
                        Request::ClearProjectile { epoch: _ } => {
                            projectile.clear();
                            Response::ClearedProjectile
                        }
                    })
                });
                reply(
                    &output,
                    match result {
                        Ok(response) => Reply::Completed(Box::new(response)),
                        Err(error) => Reply::Failed(error.into()),
                    },
                );
            }
        }
    }) as Box<dyn FnMut(_)>);
    scope.set_onmessage(Some(listener.as_ref().unchecked_ref()));
    LISTENER.with(|l| *l.borrow_mut() = Some(listener));
    reply(&scope, Reply::Booted);
    true
}
async fn initialize(source: Source) -> Result<Drivers, String> {
    use clap::Parser;
    let root = source
        .root
        .to_str()
        .ok_or("invalid prediction package path")?;
    let args = crate::args::Args::try_parse_from(["prediction-worker", "--cad-assets", root])
        .map_err(|e| e.to_string())?;
    let cad = crate::browser_assets::load_prediction(&args)
        .await
        .map_err(|e| e.to_string())?;
    let (geometry, floor) = source.scene.geometry(&cad)?;
    Ok((
        prediction::ReplayDriver::new(geometry.clone(), floor),
        projectile_prediction::ProjectileDriver::new(geometry, floor),
    ))
}
