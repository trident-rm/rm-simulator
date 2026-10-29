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
queries and debug drawing. Chassis and provisional projectile prediction use the
same whole-world replay as native, with at most eight chassis ticks and four
projectile ticks per exchange. Checkpoint restoration remains indivisible.
`prediction=0` disables local prediction.

Single Player runs locally through the same host authority and codecs, without
a native process or network socket. Each host pump advances at most eight ticks
and retains at most 250 ms of catch-up time. F5 starts an idle RMUC or RMUL round;
F6 pauses/resumes and F7 advances one manual step while paused. `play=1` starts
Single Player directly. Navigation runs one queued bake/search per poll. Native
multiplayer listening, UDP LAN discovery, the TCP automation console and FFmpeg
recording remain native platform facilities.

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
CI compiles and lints Wasm; it does not claim a GPU browser run.

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

## Optional CAD packages

Optional robot, folding-Hero, assembly and area packages use the same descriptor,
armor, collision and frame validators as native. Browser launch parameters map
to the existing options: `robot_assets`, `hero_assets`, `assembly_assets`,
`assembly_area` and `assembly_hints`. `robot=engineer` chooses the Heavy role.
Paths are relative to the served application. Invalid packages fail startup.

After compiling the Wasm binary, stage explicitly supplied packages alongside
it. Each package must retain its upstream `CAD-NOTICE.md`:

```sh
python3 scripts/stage-browser.py --out local-assets/browser-with-cad \
  --extra-package robot=/path/to/checked-robot \
  --extra-package hero=/path/to/checked-hero \
  --extra-package assembly=/path/to/checked-assembly \
  --extra-package area=/path/to/matching-area
```

Use `--extra-file hints.json=/path/to/recording.json` for optional planning hints.
A launch URL can include
`?field=rmuc&play=1&robot=engineer&robot_assets=robot&hero_assets=hero&assembly_assets=assembly&assembly_area=area`.
Robot descriptors are resolved before startup, so the local host and passive
renderer consume the same checked physical configuration.

Automatic assembly planning runs the existing solver in isolated Web Workers.
The host sends a source-pinned reconstruction recipe and bounded rig state;
the worker receives no match or actuator authority. The host retains generation,
parking, actual-start and cancellation checks before installing a result.
At most two workers run, and cancelling or dropping a task terminates its worker.
Native builds keep their planning threads. Worker source transfer is capped at
256 MiB, metadata/results at 8 MiB, with a 60-second deadline.

## Browser validation and measurements

Pass `--offline` to the smoke test to exercise local admission, rule start,
movement, firing, pause, exact manual stepping and bounded prediction. Opt-in
`inspect=1` diagnostics include measured host/replay work, recent frame interval
percentiles and Wasm linear-memory capacity. Linear memory is not browser-process
RSS. Measurements include rendering and inspection overhead; they do not establish
a hardware-independent budget. Heavy checkpoint restoration, map construction
and a single navigation search are still indivisible operations.

A local macOS Chromium run at 1600 × 1000, with one robot in the offline 1v1
arena, recorded a 16.7 ms median frame interval and an 18.7 ms p95 over the latest
180 samples. Wasm linear-memory capacity was 164,167,680 bytes. These are one-run
observations, not peak process memory or startup measurements. The source-backed
assembly worker matched a native 1,053-frame plan with 854 checks while the page
rendered 277 frames during the worker request. Use `--cad` and
`--worker-fixture PATH` to opt into optional CAD and native-fixture parity checks.

For repeatable startup and sampled Chromium process-memory measurements, see
[browser performance](browser-performance.md).

## GitHub Pages

The manual `Browser Single Player Pages` workflow builds the Wasm app on `main`,
hydrates the approved field packages through Git LFS and uploads the staged static
bundle. It deploys through the `github-pages` environment. No native server or
optional development CAD package is deployed. Single Player runs in the visitor's
browser. It still requires WebGPU; loading the page once does not install it for
use without an Internet connection.

After authorizing publication, set the repository's Pages source to GitHub
Actions, push the reviewed commits and dispatch `.github/workflows/pages.yml`
on `main`. Deployment is manual; ordinary pushes do not publish the app.
The expected project URL is `https://trident-rm.github.io/rm-simulator/`.
The title screen offers Single Player; `?field=1v1&play=1` launches it directly.
All runtime URLs stay relative to the page, including field bytes and workers.
The build uses the existing field checksum inventories and retains their CAD
notices, workspace licenses and third-party notices.

The default staged bundle is about 111 MiB. The workflow does not rely on custom
HTTP headers or precompressed file negotiation. GitHub Pages serves the static
application; multiplayer hosting remains a separate native service.
