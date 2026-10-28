// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Isolated Web Worker carrier for the shared native assembly planner.
use rm_sim_server::assembly_worker::{Request, Response, Task};
use std::{
    cell::RefCell,
    collections::BTreeMap,
    rc::Rc,
    sync::{
        atomic::{AtomicU64, Ordering},
        mpsc,
    },
};
use wasm_bindgen::{JsCast, closure::Closure};

const MAX_MESSAGE_BYTES: usize = 8 * 1024 * 1024;
const MAX_SOURCE_BYTES: usize = 256 * 1024 * 1024;
const MAX_WORKERS: usize = 2;
static NEXT_WORKER: AtomicU64 = AtomicU64::new(1);
struct Carrier {
    worker: web_sys::Worker,
    _message: Closure<dyn FnMut(web_sys::MessageEvent)>,
    _error: Closure<dyn FnMut(web_sys::ErrorEvent)>,
    _timeout: Closure<dyn FnMut()>,
    timeout_id: i32,
}
impl Drop for Carrier {
    fn drop(&mut self) {
        self.worker.set_onmessage(None);
        self.worker.set_onerror(None);
        self.worker.terminate();
        if let Some(window) = web_sys::window() {
            window.clear_timeout_with_handle(self.timeout_id);
        }
    }
}
thread_local! {static WORKERS: RefCell<BTreeMap<u64,Carrier>>=const {RefCell::new(BTreeMap::new())};}
struct Handle(u64);
impl Task for Handle {}
impl Drop for Handle {
    fn drop(&mut self) {
        WORKERS.with(|workers| {
            workers.borrow_mut().remove(&self.0);
        });
    }
}
/// Install platform dispatch before an offline host can request assembly planning.
pub(crate) fn install() {
    let _ = rm_sim_server::assembly_worker::install(dispatch);
}
fn set(object: &js_sys::Object, key: &str, value: &wasm_bindgen::JsValue) -> Result<(), String> {
    js_sys::Reflect::set(object, &key.into(), value)
        .map(|_| ())
        .map_err(|error| format!("worker message: {error:?}"))
}
fn dispatch(
    request: Request,
    sender: mpsc::SyncSender<Result<Response, String>>,
) -> Result<Box<dyn Task>, String> {
    if WORKERS.with(|workers| workers.borrow().len()) >= MAX_WORKERS {
        return Err("assembly worker limit reached".into());
    }
    let metadata = serde_json::to_string(&request).map_err(|e| e.to_string())?;
    if metadata.len() > MAX_MESSAGE_BYTES {
        return Err("assembly worker metadata exceeds capacity".into());
    }
    let payload = js_sys::Object::new();
    let files = js_sys::Array::new();
    let transfer = js_sys::Array::new();
    set(&payload, "job", &metadata.into())?;
    for (path, bytes) in request.recipe.source_files()? {
        let file = js_sys::Object::new();
        let array = js_sys::Uint8Array::from(bytes.as_slice());
        let buffer = array.buffer();
        set(&file, "path", &path.into())?;
        set(&file, "bytes", buffer.as_ref())?;
        files.push(file.as_ref());
        transfer.push(buffer.as_ref());
    }
    set(&payload, "files", files.as_ref())?;
    let options = web_sys::WorkerOptions::new();
    options.set_type(web_sys::WorkerType::Module);
    let worker = web_sys::Worker::new_with_options("assembly-worker.js", &options)
        .map_err(|e| format!("starting assembly worker: {e:?}"))?;
    let pending = Rc::new(RefCell::new(Some((payload, transfer))));
    let result = Rc::new(RefCell::new(Some(sender)));
    let socket = worker.clone();
    let output = Rc::clone(&result);
    let message = Closure::wrap(Box::new(move |event: web_sys::MessageEvent| {
        let kind = js_sys::Reflect::get(&event.data(), &"type".into())
            .ok()
            .and_then(|v| v.as_string());
        if kind.as_deref() == Some("ready") {
            if let Some((payload, transfer)) = pending.borrow_mut().take()
                && let Err(error) = socket.post_message_with_transfer(payload.as_ref(), &transfer)
                && let Some(sender) = output.borrow_mut().take()
            {
                let _ = sender.try_send(Err(format!("sending assembly worker request: {error:?}")));
                socket.terminate();
            }
            return;
        }
        let parsed = js_sys::Reflect::get(&event.data(), &"result".into())
            .ok()
            .and_then(|v| v.as_string())
            .ok_or_else(|| "invalid assembly worker response".to_string())
            .and_then(|json| {
                if json.len() > MAX_MESSAGE_BYTES {
                    return Err("assembly worker response exceeds capacity".into());
                }
                serde_json::from_str::<Result<Response, String>>(&json)
                    .map_err(|e| format!("invalid assembly worker result: {e}"))
                    .and_then(|result| result)
            });
        if let Some(sender) = output.borrow_mut().take() {
            let _ = sender.try_send(parsed);
        }
        socket.terminate();
    }) as Box<dyn FnMut(_)>);
    worker.set_onmessage(Some(message.as_ref().unchecked_ref()));
    let socket = worker.clone();
    let output = Rc::clone(&result);
    let error = Closure::wrap(Box::new(move |event: web_sys::ErrorEvent| {
        event.prevent_default();
        if let Some(sender) = output.borrow_mut().take() {
            let _ = sender.try_send(Err(format!("assembly worker failed: {}", event.message())));
        }
        socket.terminate();
    }) as Box<dyn FnMut(_)>);
    worker.set_onerror(Some(error.as_ref().unchecked_ref()));
    let socket = worker.clone();
    let output = Rc::clone(&result);
    let timeout = Closure::wrap(Box::new(move || {
        if let Some(sender) = output.borrow_mut().take() {
            let _ = sender.try_send(Err("assembly worker exceeded its 60-second lifetime".into()));
        }
        socket.terminate();
    }) as Box<dyn FnMut()>);
    let window = web_sys::window().ok_or("browser window unavailable")?;
    let timeout_id = window
        .set_timeout_with_callback_and_timeout_and_arguments_0(
            timeout.as_ref().unchecked_ref(),
            60_000,
        )
        .map_err(|e| format!("assembly worker deadline: {e:?}"))?;
    let id = NEXT_WORKER.fetch_add(1, Ordering::Relaxed);
    WORKERS.with(|workers| {
        workers.borrow_mut().insert(
            id,
            Carrier {
                worker,
                _message: message,
                _error: error,
                _timeout: timeout,
                timeout_id,
            },
        )
    });
    Ok(Box::new(Handle(id)))
}
type MessageListener = Closure<dyn FnMut(web_sys::MessageEvent)>;
thread_local! {static LISTENER: RefCell<Option<MessageListener>> = const {RefCell::new(None)};}
/// Detect the isolated realm and install its solver handler without creating an App.
pub(crate) fn start_if_worker() -> bool {
    let Ok(scope) = js_sys::global().dyn_into::<web_sys::DedicatedWorkerGlobalScope>() else {
        return false;
    };
    let output = scope.clone();
    let listener = Closure::wrap(Box::new(move |event: web_sys::MessageEvent| {
        let result = run(event.data());
        let json = serde_json::to_string(&result)
            .unwrap_or_else(|_| "{\"Err\":\"cannot encode assembly worker result\"}".into());
        let response = js_sys::Object::new();
        let _ = set(&response, "type", &"result".into());
        let _ = set(&response, "result", &json.into());
        let _ = output.post_message(response.as_ref());
    }) as Box<dyn FnMut(_)>);
    scope.set_onmessage(Some(listener.as_ref().unchecked_ref()));
    LISTENER.with(|slot| *slot.borrow_mut() = Some(listener));
    true
}
fn run(data: wasm_bindgen::JsValue) -> Result<Response, String> {
    let json = js_sys::Reflect::get(&data, &"job".into())
        .map_err(|_| "missing worker job")?
        .as_string()
        .ok_or("missing worker metadata")?;
    if json.len() > MAX_MESSAGE_BYTES {
        return Err("assembly worker request exceeds capacity".into());
    }
    let request: Request = serde_json::from_str(&json).map_err(|e| e.to_string())?;
    let files = js_sys::Reflect::get(&data, &"files".into())
        .map_err(|_| "missing worker source files")?
        .dyn_into::<js_sys::Array>()
        .map_err(|_| "invalid worker source files")?;
    if files.length() > 16 {
        return Err("assembly worker file count exceeds capacity".into());
    }
    let mut total = 0usize;
    for file in files.iter() {
        let path = js_sys::Reflect::get(&file, &"path".into())
            .map_err(|_| "missing worker file path")?
            .as_string()
            .ok_or("invalid worker file path")?;
        let buffer = js_sys::Reflect::get(&file, &"bytes".into())
            .map_err(|_| "missing worker file bytes")?
            .dyn_into::<js_sys::ArrayBuffer>()
            .map_err(|_| "invalid worker file bytes")?;
        let bytes = js_sys::Uint8Array::new(&buffer);
        total = total
            .checked_add(bytes.length() as usize)
            .ok_or("worker source byte overflow")?;
        if total > MAX_SOURCE_BYTES {
            return Err("assembly worker source exceeds capacity".into());
        }
        rm_sim_assets::package_io::install(path, bytes.to_vec()).map_err(|e| e.to_string())?;
    }
    rm_sim_server::assembly_worker::solve(request)
}
