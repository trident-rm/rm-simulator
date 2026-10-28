<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Platform boundaries

Native and browser launches use the same app wiring, `Session`, `Screen`,
controls and HUD. Browser platform adapters provide asynchronous WebSocket
admission, HTTP field packages and localStorage preferences. They do not define
another HUD or match lifecycle.

## Browser application

Install the Rust `wasm32-unknown-unknown` target and a `wasm-bindgen-cli` matching
the locked version (currently 0.2.129). Hydrate the field packages with Git LFS:

```sh
just browser-check
just browser-build
python3 -m http.server 8080 --bind 127.0.0.1 --directory local-assets/browser
```

Open `http://127.0.0.1:8080/`. The existing title screen, settings, multiplayer
form, loading splash, deployment view and gameplay HUD all render in Bevy.
There is no HTML pilot card or crosshair. WebGPU and a secure context (HTTPS or
localhost) are required. The HTML page only owns the canvas and fatal startup
errors. Existing [controls and HUD documentation](app-options.md) applies.

The native HUD keeps its scoreboard, robot and ammunition cards, heat ring,
reticle, team status, controls help and map. Settings retain the same validated
JSON format in browser localStorage; passwords are not saved. The browser starts
with the low graphics preset and bloom disabled; settings can change these.

Start a practice host with an explicit WebSocket endpoint and allowed origin:

```sh
cargo run -p rm-sim-server --bin rm-sim-server -- \
  --competition rmul-infantry-2027 --no-referee --no-lobby \
  --websocket 127.0.0.1:5002 --websocket-origin http://127.0.0.1:8080
```

For a refereed match, omit `--no-referee` and start the round through the private
native referee panel. Pre-match rules still gate controls. Enter the endpoint
in the Multiplayer form. A launch URL can prefill and join it directly:
`http://127.0.0.1:8080/?field=1v1&server=ws%3A%2F%2F127.0.0.1%3A5002%2F`.
Use `field=3v3` or `field=rmuc` for another competition. The selected field must
match the host. Admission starts as a spectator, then the native deployment UI
requests a side, robot and position. The server validates the deployment.

The bundle includes approved field files and a checksum inventory. HTTP bytes
enter the same package loader that native uses, including collision recipes,
source bindings and minimap validation. Each fetch has a 64 MiB ceiling; a
package has a 512 MiB byte ceiling and at most 256 files. Paths remain confined
to the package root. Checksums validate the served inventory and manifests;
serve these from a trusted origin. Verified visuals are supplied through Bevy's
memory asset reader, without an unchecked second HTTP download.

Browser sessions retain verified collision geometry for staging heights, aim
queries and debug drawing. Robot interpolation and shot feedback use the existing
Session. Local physics prediction remains disabled because its worker requires a
browser scheduling implementation. Simulation and match authority stay on the
native host. Native hosting, UDP LAN discovery, the TCP automation console and
FFmpeg recording are native platform facilities. Optional external robot and
assembly CAD packages still need browser loading support.

Each binary WebSocket message carries one existing codec datagram. Host command
ownership, admission, weapon cadence, confirmation ordering and snapshots retain
the same path. The endpoint accepts exact configured Origins and pilot/spectator
admission only. Connections, handshake time, message size and pending writes are
bounded. Public hosting uses HTTPS with a WSS reverse proxy to the listener;
the listener itself provides no TLS. Keep the privileged referee panel private.
No public deployment is performed by the build command.

The optional browser regression test uses Chromium and a running practice host:

```sh
uv run --with playwright playwright install chromium
uv run --with playwright python scripts/smoke-browser.py \
  --root local-assets/browser --server ws://127.0.0.1:5002/
```

The test owns a temporary HTTP server on port 8080. `--executable` selects an
existing Chromium; `--port` and the host's Origin allowlist must agree. Tests use
`inspect=1`, which exports actual app screen, session and laid-out UI observations
to a hidden diagnostic element. It adds no controls or alternate HUD. Ordinary
CI compiles and lints Wasm; it does not claim a GPU browser run. Browser frame-time
and memory budgets remain unmeasured.

## Shared client library

Disable the default `native-host` feature when consuming the library from a
caller-owned transport:

```toml
rm-sim-server = { path = "../rm-sim-server", default-features = false }
```

`state` owns published snapshots and control identities. `competition` owns
competition identity. `sentry_view` owns published camera diagnostics. Native
`simulation`, `host_args` and `sentry_controller` re-export those same types.
Moving them does not change field order, protocol version or serialization.

`udp_codec::ClientCodec` accepts caller-provided datagrams and monotonic time.
`submit` queues a `QueuedCommand`; `next` yields a `pacing::Datagram` when its
byte budget allows it. `receive` yields a `ClientEvent` after fragment and
baseline processing. `acknowledge` queues baseline/configuration feedback.
A carrier must preserve the ordered reliable lane and the replaceable unreliable
lane. The codec starts no worker and opens no socket. Its `Instant` is backed
by `web-time` on browsers and by `std::time` on native targets.

Compression, snapshot packing, input history, owner anchors, interpolation and
bounded prediction replay remain shared. File-based prediction package loading
and native host construction require `native-host`. The client library still
depends on the shared world, assets, control and Rapier stack for snapshots and
replay; it is not a renderer-only wire DTO package.

`native-host` remains enabled by default. It provides the authoritative host,
GNS sockets, loopback workers, HTTP panel, LAN discovery, sensor service and
planning workers. The server binary requires that feature. Cargo features are
additive: a workspace build containing the app enables native hosting again.
Run the portable checks against the server package alone.

## Checks

```sh
just client-test
just client-wasm-check
```

The first command runs strict Clippy, library tests, the public carrier-independent
client tests and doctests with native hosting disabled. The second compiles the
library for Wasm; install the Rust `wasm32-unknown-unknown` target first. The Nix
shell uses `mkShellNoCC`, includes native Clang explicitly and selects unwrapped
Clang/LLVM ar for Wasm C dependencies. Native SDK and hardening flags stay on
the native compiler path. Full validation includes a separate
portable CI job and `just verify` includes the host-free tests. The dependency
boundary script resolves the Wasm client separately and rejects GNS and interface
discovery dependencies.

The client tests exercise command/probe/barrier ordering, fragmented snapshot
receipt in raw and compressed modes, stale publication rejection and recovery
when an owner anchor references an unknown configuration. Native tests still
exercise the same client codec through GNS and loopback.

## Remaining deployment work

Browser scheduling for local physics prediction, optional robot/assembly CAD
loading and performance measurements remain. Offline browser hosting needs a
caller-driven host or Web Worker, rather than native socket/thread services.
