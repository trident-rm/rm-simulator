<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Platform boundaries

The interactive app and headless host are native applications. The shared client
library in `rm-sim-server` also compiles for `wasm32-unknown-unknown`. There is
no browser app, browser asset loader or browser server endpoint yet. Compilation
does not establish browser runtime correctness, frame rate or memory use.

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

A browser app needs HTTP-backed verified asset bytes, asynchronous startup and
a browser rendering configuration. Embedded simulation needs caller-driven task
scheduling or Web Workers in place of native host threads. Browser multiplayer
needs a server endpoint for a browser-supported carrier; the native GNS socket
is not that endpoint. Keep host permission checks and the existing wire ordering
when adding it. A Wasm compile check does not substitute for a browser playtest.
