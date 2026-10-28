<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Platform boundaries

The native app supports simulation and multiplayer. The Wasm entry point loads
source-backed 2027 CAD over HTTP. Without a server it is a static field viewer.
With a WebSocket server it sends pilot commands and draws authoritative snapshots
through the shared renderer. Physics and match rules run on the native host.

## Browser viewer

The browser entry point uses a canvas and asynchronous Fetch requests. It reads
the arena and equipment manifests through the same byte parser and identity
checks as native loading. The visual plan shares native placement conversion.
Each GLB is SHA-256 checked before it enters Bevy's in-memory asset source. The
renderer receives those exact bytes, without a second HTTP read. External image
and buffer URIs are rejected; the current packages embed their dependencies.
This is visual validation, not collision or rules admission. SHA-256 is checked
against the served manifests; serve the bundle from an origin you trust.

Install the Rust `wasm32-unknown-unknown` target and a `wasm-bindgen-cli` matching
the locked `wasm-bindgen` version (currently 0.2.129). Hydrate the field packages
with Git LFS, then run:

```sh
just browser-check
just browser-build
python3 -m http.server 8080 --bind 127.0.0.1 --directory local-assets/browser
```

Open `http://127.0.0.1:8080/`. Choose RMUL 1v1, RMUL 3v3 or RMUC, then drag
to orbit and scroll to zoom. WebGPU and a secure context (HTTPS or localhost)
are required. The page reports missing HTTP files, hash mismatches, renderer
loading failures and unavailable WebGPU. The default field is 1v1. Scenery is
static; rule-driven lights, moving equipment and robots are not installed.
The browser uses the native lit materials and venue lights/skybox, with bloom,
shadows and MSAA disabled. It fetches each distinct visual file once,
sequentially, with a 64 MiB per-file and 256 MiB package byte ceiling.

`just browser-build <new-output-directory> [profile]` uses the size-oriented
`web` profile by default and runs `wasm-bindgen` to produce a static bundle.
The output directory must not exist. The bundle contains the viewer, manifests,
verified visual GLBs and ownership/license notices. It includes the checksummed RMUC source-motion and optical-node bindings.
It excludes collision sidecars. Native release settings are unchanged. A static HTTP server is
enough; no Python process runs inside the browser.

The optional real-browser regression test starts and stops its own loopback
HTTP server. Install Playwright's Chromium, or pass `--executable` for an
existing Chromium build:

```sh
uv run --with playwright playwright install chromium
uv run --with playwright python scripts/smoke-browser.py --root local-assets/browser
```

It tests all three fields, camera dragging, missing manifests, checksum failures
and the no-WebGPU message. Ordinary CI compiles and lints the Wasm entry point;
it does not claim a GPU browser run. Browser performance and memory budgets
have not been measured.

## Browser gameplay

Start a native 1v1 host with an explicit browser endpoint and allowed origin:

```sh
cargo run -p rm-sim-server --bin rm-sim-server -- \
  --competition rmul-infantry-2027 --no-referee --no-lobby \
  --websocket 127.0.0.1:5002 --websocket-origin http://127.0.0.1:8080
```

Build and serve the static bundle as above, then open
`http://127.0.0.1:8080/?field=1v1&server=ws%3A%2F%2F127.0.0.1%3A5002%2F&team=red`.
This starts practice mode. For a refereed match, omit `--no-referee` and start
the match through the private native referee panel; pre-match rules gate controls.
A second pilot can choose `team=blue`. The host grants the pilot's robot through
ordinary admission. The pilot starts in first person; V switches to the trailing
camera. Click the canvas to capture the pointer, move the mouse to aim, and
click again to fire. The capture click never fires. Escape releases capture.
WASD drives, Q/E turns, arrow keys aim and Space also fires. Editing the
connection form suspends game input. The HUD displays authoritative HP, rounds,
heat, speed and defeat state, with a centre crosshair. The selected field must
match the server competition. The browser reports mismatches and disconnects.

Each binary WebSocket message contains one existing codec datagram. Command
ownership, admission, weapon cadence and snapshots use the existing host path.
The endpoint rejects referee admission and accepts only exact configured Origins.
Connections, handshake time, message size and pending writes are bounded.
WebSocket provides ordered delivery; congestion can still delay newer snapshots.
The client interpolates robot poses using the shared bounded history with a
64 ms presentation delay. Extrapolation stops after 100 ms; pause and placement
changes reset history. It does not run local physics prediction. Projectiles use
authoritative snapshot positions. RMUC source joints, target lights, HP and
strike flashes use the native presentation and passive renderer.

For public hosting, serve the bundle over HTTPS and proxy a WSS endpoint to the
native listener. Configure the exact frontend origin with `--websocket-origin`.
The listener provides no TLS itself. Keep the privileged referee HTTP panel
private. No public deployment is performed by the build command.

Run the existing viewer checks plus host-backed movement and firing checks:

```sh
uv run --with playwright python scripts/smoke-browser.py \
  --root local-assets/browser --server ws://127.0.0.1:5002/
```

The test owns a temporary HTTP server on port 8080; start only the practice simulation
server beforehand. Pass `--port` and update the host's allowed Origin together
if 8080 is occupied. These checks use authoritative position and shot/projectile
observations, rather than assuming key events reached the simulation. Camera
mode changes, pointer capture, capture-click suppression, mouse aim and form
focus are also checked. Pass `--rmuc-server ws://127.0.0.1:5003/` to check a
separate source-bound RMUC host; source joints and target lights must be present.
A running RMUC round also checks Rune motion. Pilot tests wait for all CAD
instances to finish spawning before exercising controls. A tampered source
binding JSON must fail loading.

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

Offline browser simulation still needs an HTTP-backed verified physics package
and caller-driven task scheduling or Web Workers in place of native host threads.
The remote browser client does not predict or restore a local physical world.
Native staging, the full HUD, local prediction and robot CAD-package loading
remain to be shared with the browser entry point. Browser performance and memory budgets remain unmeasured.
