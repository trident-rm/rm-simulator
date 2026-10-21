<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# rm-simulator

A first-person RMUC 2027 field simulator written in Rust with Bevy. It loads
our reviewed CAD field, runs native combat, territory control, Rune and assembly
mechanics on an explicit 128 Hz multiplayer clock, and supports local and network play.
Reusable in-process sessions support configurable stepping, including 1000 Hz,
without a server or renderer; see [simulation embedding](docs/simulation-embedding.md).
Drive a chassis over the terrain with a mouse-aimed gun, or explore with `--fly`.
The final 2026-capable source is preserved at Git tag `rm2026-final`.

Physics and motion, gameplay rules, Bevy rendering, the server and the
interactive application are separate Rust crates. Rune rules, outpost geometry,
the CAD scene handling, and the armor artwork masks were reused from the sibling
`../Vision/rm-vision-sim` repository; see `NOTICE.md`. The
[documentation index](docs/README.md) lists the current guides.

The [2027 rules reference](docs/rules-2027.md) interprets the supplied manuals;
[gameplay coverage](docs/rmuc2027-field.md) distinguishes supported mechanics
from remaining gaps. Builds target RMUC 2027 without season feature switches.
See [the archived runtime guide](docs/seasons.md) for 2026 and shared internals.

## Contents

- [Quick start](#quick-start)
- [Controls and options](#controls-and-options)
- [Title screen and LAN lobbies](#title-screen-and-lan-lobbies)
- [Crate layout and dependencies](#crate-layout-and-dependencies)
- [Field assets](#field-assets)
- [Running a server and joining](#running-a-server-and-joining)
- [Match rules and scoring](#match-rules-and-scoring)
- [Robot equipment](#robot-equipment)
- [Performance](#performance)
- [Distributing builds](#distributing-builds)
- [Development](#development)
- [License](#license)

## Quick start

Download a platform ZIP from GitHub Releases and extract it completely. Each ZIP
includes the app, headless server, native runtime libraries, and the field package.
Run `bin/rm-simulator` or `bin/rm-simulator.exe`.

With Nix on Apple Silicon macOS or Linux, use the pinned project environment:

```sh
direnv allow  # once, after reviewing .envrc and flake.nix
# Or enter explicitly without direnv:
nix develop
just server
```

The shell supplies CMake, Clang/libclang, pkg-config, Protobuf 21 and OpenSSL.
Rust remains managed by rustup and `rust-toolchain.toml`. Nix builds use
`target/nix/`, keeping objects linked against Homebrew out of this build.
Commit `flake.nix`, `flake.lock` and `.envrc`; `.direnv/` stays local. Update
native dependencies with `nix flake update`, then rebuild and test before
committing the new lockfile. No global Homebrew dependencies are needed in this shell.

Without Nix, install the Rust toolchain from `rust-toolchain.toml`. GameNetworkingSockets
also needs CMake, Clang/libclang, pkg-config, Protobuf and OpenSSL to build.
On Ubuntu, install `cmake libclang-dev pkg-config protobuf-compiler libprotobuf-dev libssl-dev`.
On macOS:

```sh
brew install cmake pkg-config protobuf@21 openssl@3
export CMAKE_PREFIX_PATH="$(brew --prefix protobuf@21):$(brew --prefix openssl@3)"
export PKG_CONFIG_PATH="$(brew --prefix protobuf@21)/lib/pkgconfig:$(brew --prefix openssl@3)/lib/pkgconfig"
export PATH="$(brew --prefix protobuf@21)/bin:$PATH"
```

Windows builds need the MSVC C++ tools, CMake, Git and LLVM/libclang. The native
crate builds Protobuf through vcpkg. Use a short checkout/build path to avoid
Windows path-length limits.

Then:

```sh
just run
just run --play --big-rune --outpost-speed-rad-s 3.0
```

## Controls and options

The essentials, for driving and for match control:

| Input | Action |
|---|---|
| Mouse | Look |
| Left button (held, once captured) | Fire from the barrel (driving) or just ahead of and below the eye (flying) |
| Right mouse (hold) | Auto Aim + Auto Fire; separately rebindable actions in Controls |
| G | Switch the auto-aim target between armor and rune, when Controls sets the auto-aim target to Manual |
| W A S D | Drive forward/back and strafe left/right relative to the aim; the chassis heading follows the aim (driving), fly forward/back and sideways in the body frame (Drone) or move on the horizontal plane (free camera) |
| Q / E | Rotate the Drone's body left/right; the mouse aims its gimbal separately |
| Left Ctrl | Fast: 5 m/s drive command, 8 m/s flight |
| R | Toggle chassis spin (6 rad/s) while driving |
| T | Toggle supported chassis fold, stair linkage or tunnel stow (500 ms clear stroke) |
| J | Open Heavy assembly controls and unlocked Hero conversion (2027) |
| V | Toggle first- and third-person view while driving |
| C | Cycle physics colliders: Off / Overlay / Only |
| F3 | Open / close debug panel: collider Off / Overlay / Only, visual wireframe, buff/RFID zones and labels, rendering statistics, remote motion buffering, reset own robot to spawn |
| Space / Left Shift | Move up / down (flying); Space jumps with balance Infantry |
| F5 | Start the match (or reset a finished one) (local world or referee) |
| F6 | Pause or resume the world clock (local world or referee) |
| F7 | Step the world one manual step (three 128 Hz ticks, 23.4375 ms) while paused (local world or referee) |
| F | Activate the rune for your team when it has an opportunity (local world or referee) |
| Tab (hold) | Show team robot status |
| P | Toggle settings; 1 toggles reticle, 2 toggles minimap, - / = adjusts mouse sensitivity |
| M | Open the team map; click for a local marker or Sentry/team interaction. A / B / I choose attack / defend / alert |
| Left Alt (hold) | Free the cursor; clicking the small map opens the interaction choice |
| F12 (hold) | Show controls |
| Escape | Close the current panel or open Pause; in staging, go back a step or leave; in Multiplayer, return to the main menu; otherwise open or cancel quit confirmation |

The complete command-line reference for both binaries, the weapon settings, the
HUD and interface behavior, the graphics presets and the full controls table are
in the [app options reference](docs/app-options.md). Settings, weapon preferences
and graphics choices are edited from **Settings** or the toolbar and saved in
`settings.json`.

## Title screen and LAN lobbies

A bare launch opens a vertical main menu: Single Player, Multiplayer, Settings
and Quit. Single Player starts a local practice field and never asks for a
name. Multiplayer asks for the player name, then lists LAN lobbies on the left
and creates a named, optionally
password-protected lobby on the right. Select a listing, enter its password if needed, and press Join lobby / address.
The menu preloads the field in the background, with a thin progress bar along
the bottom. Its live arena backdrop follows the pointer gently; verified assets
and scenery are reused when joining.

Joining opens a connected 3D staging view of the real field. Hover over a half
of the arena to highlight it and click to choose your side. Choose a robot and
supported chassis, then click a starting position in the outlined ring around
your base. The camera switches to a top-down view for
placement. Deploy creates your robot only after the host accepts the position;
occupied points are refused. Drone uses its designated aerial pad. You can also
enter as a spectator without a robot. Escape goes back a step or leaves staging.
Host weapon limits and defaults are available on the Multiplayer page before
creating a LAN lobby. In-game Settings > Weapon adjusts the player weapon. Solo uses the same embedded host and client as LAN hosting, without
a network listener. Existing direct-play CLI commands still enter immediately;
add `--staging` to use the connected setup flow.
You can also enter a direct address. The page and lobby list scroll with the
mouse wheel, trackpad or scrollbar; narrow windows stack the two columns. Public is greyed out pending public connectivity support.
The firewall tip recommends allowing the app on private networks. Lobby names
and addresses are remembered, but passwords are never saved. The fields live in
`$XDG_CONFIG_HOME/rm-simulator/title.json` (`~/.config` by default,
`%APPDATA%` on Windows). Losing the
connection, a host failure or a bad address ends the match and returns to the
title screen with the reason; the
window stays open. The CAD package is verified once per process, so a second
join is quick. The title background is a darkened in-game screenshot without
the gameplay HUD. The field sits in a dark stadium, with visible wire fencing
aligned to its two-metre collision boundary at the edge of the grounded CAD
scenery. The slab apron lies outside the fence. Distant seating, walls and ceiling
lights use an indoor cubemap instead of stadium meshes; they have no parallax.

### LAN lobbies

Create a lobby in Multiplayer, then click Refresh LAN on another computer on
that network. Select the lobby and press Join lobby / address. The creator runs the match and must keep it open. Discovery uses
UDP port 7792; gameplay uses GNS over UDP 7700 by default. Allow the app
through the firewall on private networks.
Guest Wi-Fi isolation and separate subnets can prevent discovery. Direct
connections remain available when broadcast discovery is blocked.

LAN discovery needs no directory server, internet access, router registration
or port forwarding. One advertised lobby can run per computer, since the LAN
responder owns a fixed port. Leaving the match closes its listing. Refresh
removes departed hosts from the displayed list. Passwords apply to pilots and
spectators, including direct connections. The current text widget displays
passwords visibly; they are never saved to `title.json`. The game handshake
is unencrypted, so use a lobby-specific password.

Public hosting and joining from the menu are disabled. The directory code and
Docker setup are in [`services/lobby`](services/lobby/README.md), ready for a
later deployment. No directory is required or running for this feature.

## Crate layout and dependencies

| Crate | Responsibility |
|---|---|
| [`rm-simulator-gameplay`](crates/rm-simulator-gameplay/README.md) | Standalone match lifecycle and gameplay state, deterministic commands, rule coverage and unsupported-element observations. No physics or renderer; its live resource component is driven by the world referee. |
| [`rm-simulator-physics`](crates/rm-simulator-physics/README.md) | Reusable Rapier dynamics, chassis, projectiles, raw armor contacts, shared geometry and prescribed armor motion. No gameplay, Bevy, server or CAD-loader dependency. |
| [`rm-simulator-world`](crates/rm-simulator-world/README.md) | Complete `Field` facade: explicit ticks, activation, detection, damage, referee integration and restore. Coordinates the physics library and preserves existing public world imports. |
| [`rm-simulator-render`](crates/rm-simulator-render/README.md) | Bevy CAD scenery, lighting, rune and outpost light overlays, projectile spheres, chassis visuals, and pose/visibility/strike synchronization from caller-owned scene state. No world dependency. |
| [`rm-simulator-server`](crates/rm-simulator-server/README.md) | Bevy-free glue: CAD loading and checksums, collision triangles, field layout, the `Simulation` wrapper, a `Host` worker that owns simulation and command ordering, the GNS UDP transport, the embedded owner's loopback codec link, the HTTP referee panel, and the headless binary. |
| [`rm-simulator-bench`](crates/rm-simulator-bench/README.md) | Fixed-camera CAD renderer benchmark, independent of physics, world, server and gameplay. |
| [`rm-simulator-app`](crates/rm-simulator-app/README.md) | The `rm-simulator` binary: window, chassis driving and gimbal camera (or fly camera), gun, HUD, world-to-scene adaptation, and the local or remote session that turns inputs into protocol commands. |

Use the physical library without a match or server:

```sh
cargo run -p rm-simulator-physics --example moving_armor --locked
```

An optional Bevy composition example draws moving armor and projectiles without
using `Field`, the referee or server APIs:

```sh
cargo run -p rm-simulator-app --example physical_scene --locked
```

The example advances 16 simulation milliseconds per rendered frame. Production
applications supply their own pacing. See [physics reuse](docs/physics-reuse.md)
and the [refactor contract](docs/architecture-refactor.md).

The native RMUC 2027 subset includes terrain traversal buffs and Small/Big Rune
activation, rings, motion and team benefits. Use `just control-test` for its
headless rule tests or the [physical console scenarios](docs/console.md#native-terrain-and-rune-scenarios)
for the loaded field. See [2027 coverage](docs/rmuc2027-field.md#native-terrain-traversal-and-rune-2026-10-18)
for approximations and remaining equipment.

RMUC 2027 also supports virtual assembly through the `--http` referee panel:
start a difficulty and press **Finish assembly** before its 45-second prototype
deadline. That referee mode requires no manipulation. The source PIE Heavy also
supports [in-game automatic L1–L3 assembly](docs/assembly-workbench.md#in-game-automatic-assembly):
park manually, press J, and choose a level. The host solves asynchronously and
first moves the Tech Core into position, then solves and plays the articulated
sequence, with separate positioning, solving and actuation progress bars.
For L3, the final Q angle is revealed by the LEDs only after insertion, lift and
flip; the arm holds its grasp while a second solve plans the observed twist. The arm
starts neutral with a persistent Unit already held in its closed jaws. It carries
that same object through a collision-checked approach and inserts it before lifting or flipping.
The J panel provides grip/release and 1 cm / 5° wrist jog controls. Released Units
fall and collide with robots and terrain; pickup requires nearby aligned pads.
Starting assembly requires a held Unit and never creates a replacement.
After L3 completion or the native launcher catch-up timer, **J → Convert to Hero**
switches a stopped Heavy to the folding Hero approximation with a 42 mm gun.
Use `--hero-assets PATH` for its verified CAD package; sibling
`bound-hero-armored/` is discovered automatically. Buy shells with the existing
42 mm purchase control; conversion does not grant ammunition. See
[the approximation and remaining limits](docs/assembly-workbench.md#temporary-hero-conversion-for-heavy).
It holds its last pose when finished. T folds it for tunnels or returns it toward neutral;
joint and jaw motion is limited to 1 rad/s.

Native 2027 adaptive armor can be selected from the referee panel after ten enemy
hits and an earned opportunity. Its center and selection-time area stay fixed
until selected again. See [adaptive armor](docs/gameplay.md#rmuc-2027-adaptive-armor).

Coordinates: the world uses metres in forward/left/up (FLU) with wxyz quaternions.
The renderer converts to Bevy's right/up/back frame at the scene boundary.
Crate ownership, tick order, restore and ECS boundaries are documented in
[simulation architecture](docs/architecture-refactor.md).

## Field assets

[RMUC 2027 map support](docs/rmuc2027-field.md) describes the bound `field-2027/`
package. Its checksummed bindings enable native rules, source motion and detector
contacts. The package is versioned through Git LFS:

```sh
git lfs install
git lfs pull --include='field-2027/**'
```

`--cad-assets` overrides discovery; otherwise the loader checks `field-2027/`
beside an installed `bin/` directory and then in the checkout. Application
sessions reject 2026 packages. Live matches require native bindings; unbound
2027 geometry can be inspected with `--no-referee`. The loader verifies listed
assets against their recorded SHA-256 checksums.

Historical 2026 assets are preserved at Git tag `rm2026-final`; they are no
longer part of the active checkout or release pipeline.

The default field sits in a dark indoor hall. Distant scenery is baked into a
cubemap; nearby boundaries and floor retain geometry. Source materials remain
intact, and source-bound equipment lights and motion follow native rules.

Generate the simplified map from your field package with the sibling
`rm-map-tools` checkout. This is an offline tool, not a runtime dependency:

```sh
uv run --with numpy --with pillow python scripts/generate-minimap.py
```

An optional source-derived team robot loads with `--robot-assets DIR`; see
[hardware models](docs/hardware-models.md) for the external package, credits and
physics approximations. The EGA Sentry uses `--robot sentry` with its
[bound package](docs/hardware-models.md#ega-tunnel-sentry-runtime-package) and
`--chassis tunnel-omni`; T folds its head. Sentry deployment also retains the
ordinary Omni option. Host-owned EGA navigation bots fold for low-clearance paths.
A headless host selects its fitted dimensions with `--team-cad`.
The Sentry panel offers **Auto aim + sweep** / **Manual aim**. This optional host
controller scans with its gimbal, rotates the chassis where clear and tracks
visible enemy armor; firing remains manual. Detected enemies gain map/scene
labels. Enemy HP/ammunition are shown only when decoded information supplies them.


Use `--cad-assets PATH` and `--map-tools PATH` for other locations. The generator
writes `minimap.png` and `minimap.json` beside the external field manifest.
The app checks the image and package hashes, preserves the map aspect ratio,
and uses the recorded FLU bounds for marker placement. Regenerate after changing
the package. Missing or stale artwork falls back to a position-only schematic.
The map is a simplified top-down projection; it does not show underpasses.

Package composition, deployment, native-texture patches, `static_assets`,
collision contracts, terrain, placement frames, arena colours and loading detail
are in [the field package guide](docs/field-package.md).

## Running a server and joining

The world can run headless:

```sh
just server --big-rune            # UDP on 0.0.0.0:7700, panel on http://127.0.0.1:7780
just run --connect 127.0.0.1:7700 --team blue --name alice
```

`rm-simulator-server` loads the CAD, builds the same field as the app, steps
it in real time and serves gameplay over Valve GameNetworkingSockets UDP and an
HTTP referee panel. Open UDP port 7700 for guests. It advertises itself as a
LAN lobby, so Refresh LAN on the Multiplayer page lists it; `--lobby-name NAME`
names it (default `RM Simulator server`) and `--no-lobby` stays unlisted.

### Transports and roles

Every client sends `Hello` with its protocol version,
name, preferred team and role (`Pilot`, `Spectator` or `Referee`), then
commands (`Chassis`, `Fire`, `Referee`, `Pause`, `Step`), and receives `Welcome`, `Snapshot` (the full field snapshot, coalesced
to about 31 Hz), `Rejected`, `Pong`, `Notice` and `Roster` messages. Every
pilot gets its own chassis, spawned in the next free slot beside its team's
spawn point and removed when the client leaves; a chassis answers only to
the client it was assigned to, and a `Fire` must name it as the shooter.
The host derives its muzzle from the held turret aim and enforces its configured
caliber, speed and firing interval using simulation time. `Welcome` announces
that weapon configuration to every client. Spectators get no chassis and cannot
fire. The embedded free camera uses a separate `SpawnProjectile` command;
remote peers cannot submit it.

GNS sends commands and confirmation replies reliably in order. Periodic snapshots
are independently compressed and fragmented into small unreliable messages. Losing
a fragment drops that snapshot; the next complete snapshot needs no baseline.
Sequence numbers prevent late state from moving the client backwards. GNS handles
congestion control and encrypted transport. This standalone library needs no Steam
account or internet service; Steam lobbies and Steam Datagram Relay are not integrated.

The referee is a spectator on no team who alone may send the `Referee`,
`Pause` and `Step` commands; from anyone else they are rejected (the host's
own HTTP panel is not restricted). The roster (name, team, role and chassis
of every client) is sent whenever it changes; hold Tab to see team players.
`--no-chassis` on the server gives pilots no chassis. The window's
`--listen` hosts the same server from the app's own world so a second window
can `--connect` to it, and `--http` opens the panel next to it. The panel is
a single page that polls `/api/state`, posts `/api/command` and
`/api/referee`, and shows the clock, teams, robots, outposts and event log.

Standalone play starts an embedded server and a private in-process loopback that
runs the same framed codec as a network peer, without a gameplay socket or
compression. It works offline without a separate server process or Steam. Its clock runs on a worker, held until scenery finishes loading,
and preserves `--start-paused`. The local owner retains match controls and
free-camera firing; these privileges cannot be requested in a network hello.
`--listen` exposes the same world to other players. Its local player rides the
same framed loopback codec link, so local and remote peers share one wire;
remote peers reach it over GNS UDP instead of the in-process channels.

### Host authority

The hosted simulation and roster belong to one worker. Gameplay transports and HTTP
handlers submit bounded mailbox requests; joins, departures, commands and clock
advances execute in the worker's order. A confirmation queues its resulting
snapshot before its Pong, and socket writers encode snapshots after capture.
Each peer's bounded outbox replaces unsent periodic snapshots before enqueueing.
Confirmation snapshots and Pongs remain ordered and cannot be coalesced away;
a peer that exhausts the reliable backlog is disconnected.
Command arrival order still depends on the transports; the host establishes a
single application order, not a reproducible ordering of simultaneous arrivals.

For Rust callers, `Server::bind` and `bind_suspended` take ownership of a
`Simulation` and bind the GNS UDP listener. `Server::in_process` and `Host::new`
support a host without a socket.
Use `server.handle()` for operator commands and state queries, and
pass that handle to `HttpServer::bind`. Request handling starts immediately; `spawn_clock` or `run_clock` enables
real-time pacing. Operator command receipts contain their application sequence
and pre-command tick. State queries wait for earlier mailbox requests; after
shutdown they return the final captured state. Debug capture requests keep the
app's frame thread independent of the simulation worker.

Host handles own their listeners and workers. Shutdown closes active connections,
including incomplete handshakes and HTTP requests, and waits for workers to finish.
Collider drawing expands client-loaded CAD shapes once on a worker and follows
the rendered scene transforms each frame. It makes no host debug requests.

### Snapshots and prediction

Local snapshots target a
4 ms interval, while remote snapshots use a 32 ms interval; physics load
can reduce either rate. Clients reconstruct outpost and rune motion between
snapshots using an estimated host simulation clock. Remote chassis use automatic interpolation buffering, starting at 64 ms and
adapting within 32–250 ms, with at most 100 ms of extrapolation on underrun.
F3 offers a manual 0–250 ms delay, reset to automatic defaults, and effective
delay, view-age and underrun readouts. These settings last for the session and
affect remote chassis only. Local
movement, gimbal and projectile prediction are enabled by default for matching
field packages. Physics persists between frames and continues for up to one second
without a checkpoint. A new checkpoint restores authoritative state and replays
only control transitions whose scheduled ticks remain unprocessed. Missing past
inputs are discarded. Small camera corrections blend over about 100 ms where
geometry allows; large corrections snap. Completed predictions remain usable when
a newer ordinary snapshot arrives, provided their robot generation is current
and neither predicted time nor the correction baseline moves backward.

Small toasts distinguish measured high latency, interrupted updates and a confirmed
disconnect. Silence alone is labeled "Connection interrupted". After one second
without a checkpoint, predicted movement and new shots stop; camera look remains
available. A closed transport keeps the window open with "Disconnected". Joining
again creates a new session and robot; it does not replay old inputs.

`--no-prediction` selects host-driven movement, aim and projectile presentation for
comparison. Firing always uses independent server physics. Local offline play
uses the same ownership and command path. Severe packet loss still limits useful
combat even when local controls remain responsive.

UDP player checkpoints use compact projectile position/velocity vectors with
identity, caliber and launch time. Vectors use 32-bit floats; server physics stays
64-bit. The server retains projectile spin, while client ball prediction starts
with zero spin at each checkpoint and may correct bounced trajectories later.
Clients reconstruct rune/outpost armor poses and omit wheel-contact and failed-hit
diagnostics. HP, registered hits, command/life state and chassis reconciliation
values are preserved. HTTP referee diagnostics keep full precision and detail.

Remote pose history holds at most 32 samples. It interpolates translation,
shortest-path rotation and wheel motion, holds the nearest known pose on underrun,
and discards stale history on placement, defeat, despawn and pause changes.
Screenshots bypass interpolation, keeping confirmation-dependent captures exact.

Known mechanism motion advances locally during up to one second without a newly
received checkpoint. Pauses and screenshots use exact authoritative poses. Clock probes
run at most once per second with one outstanding; their replies never confirm
commands. The estimate assumes roughly symmetric latency and is intended for
low-latency links. The acknowledged-baseline delta encoder is driven only by the
GNS per-peer codec. The UDP encoder rotates its acknowledged baselines every 32 encoded
frames. Result snapshots
before Pong are always full. Readers reconstruct deltas before inbox coalescing;
a missing/invalid baseline closes the connection rather than exposing partial state.

The snapshot remains authoritative for rules and chassis physics. A
chassis command carries the pilot's gun aim (heading and elevation in the
world) along with the body velocity, and the snapshot reports each turret's
pose, so other players' guns are drawn where their pilots point them; your
own turret predicts the same motor response locally, ahead of the round trip.
The mouse sets an aim target; camera, barrel and muzzle follow actual motor aim.
The assumed gimbal response is 25 ms with 12 rad/s speed and 240 rad/s²
acceleration limits, evaluated on the same clock as the field.

### Input cadence

The server schedules future shots at their intended movement tick, applying controls
before deriving the muzzle with the shot's exact aim, then advancing physics.
Eligible late shots launch from the current authoritative position. Each pilot
has at most 32 queued intents. A scheduling receipt stops retries but does not
confirm a bullet; terminal results carry the actual execution tick. Cooldown
rejections are final for that shot ID, and pause, defeat or a new placement cancels
queued shots. The client predicts legal firing cadence and the same launch tick. Local projectiles and
contacts are provisional. A visible local hit may become a server miss, or the
server may register a contact that the client did not predict. HP, confirmed hit
flashes, ammunition and match results remain authoritative. Shot IDs prevent
retries from duplicating projectiles or damage. Expired shots are rejected instead
of creating a burst when a connection recovers. No per-frame shooter-view evidence
is sent by normal clients.

Input lead adapts to host-observed arrival margins within 32–150 ms. UDP repeats
the newest four samples plus useful older movement transitions, up to twelve
samples in one packet. The lead always adapts to host feedback; there is no
fixed or shortened-history override. F3 provides automatic or manual remote
interpolation buffering; it does not alter local input lead. Automatic buffering
releases excess delay at up to 50 ms per second after the two-second jitter
window improves.

RMI5 input batches share identity fields and encode exact value changes without
reducing input redundancy. Placement revisions prevent old input from
crossing robot lives. `Ping` remains a command barrier: the server queues a
resulting snapshot before its `Pong`. Screenshot capture waits for confirmation of
queued discrete commands and for the scene to apply that state.

Client socket reads and writes run on background workers. Outgoing commands
keep their order in a queue of up to 256 pending commands. The client keeps
only the latest unread snapshot and roster, plus ordered notices and
rejections capped at 256 messages or 1 MiB. Exceeding either queue limit
disconnects with an error shown in the HUD. Successfully queueing a command
does not acknowledge its execution by the host.

### Protocol versions

| Protocol | Behavior |
|---|---|
| 27 | Sends authoritative armor contacts reliably, independently of world snapshots. Recent contacts recovered from snapshots also display once; repeated snapshots cannot restart their flash. Feedback more than 250 ms old is discarded instead of replayed after a long interruption. Damage and HP remain host-owned. Both host and clients must use matching protocol builds. |
| 29 | Combines referenced owner configurations with lossless RMI3 input batches; experimental version 28 builds carried only one of the two changes. |
| 32 | Defaults periodic UDP world checkpoints to bitpacked fine fixed point with a separately trained, embedded ZSTD dictionary. Both peers must run this build. Owner anchors and full confirmations retain their existing precision. |
| 33 | Lets every pilot name the robot it drives in `Hello`; the chassis assignment and the roster repeat it, and the gun caliber follows the robot instead of one host setting. |
| 34 | Removes the `ShotFinished` message, which no host ever produced: a shot's end was only ever reported as a `ShotResult`. |
| 35 | Makes packed checkpoints the only periodic snapshot encoding and ZSTD the only wire codec, removing the JSON checkpoint path and the DEFLATE codec. |
| 36 | Freezes the simulation at the 128 Hz tick and drops the rate from the handshake. Every build, host and client now runs the fixed 128 Hz tick; `Hello` and `Welcome` no longer carry a tick length. |
| 37 | Carries projectile timestamps as checkpoint-relative ages and quantizes the `RMO5` owner anchor. |
| 38 | Removes JSON from the gameplay wire: checkpoints (`RMB1`) and every other message use a positional, type-driven bitpack. |
| 39 | Drops derived checkpoint data (field clock, rune/outpost/referee views, wheel hubs and tyre targets) and sends timestamps as tick codes; runes, including Big Runes, travel as their full restore state. |
| 40 | Implies fixed-point grids by field path, codes delta differences as exponential-Golomb and sends rotations smallest-three, including the `RMO6` owner anchor. |
| 41 | Lays checkpoints out by change rate with chassis configuration presets; deltas under 128 bytes skip compression. |
| 42 | Dead-reckons delta baselines to the frame's tick, realigns changed sequences by id and rotates baselines after 12 frames. |
| 43 | Removes client fire timing, `ShotScheduled`'s intended time, the explicit-rules checkpoint fallback and `RMI2` input batches; `RMO6` command and tyre speeds use the 1 cm/s velocity scale. |
| 44 | Carries the gameplay engine's state as the referee's `game`, outpost rotor start and homing, the match rule commands and `SetPerformance`. |
| 45 | Adds buff point contacts, terrain crossing and Fortress state, the `TerrainCrossing` and `BaseArmorExpanded` events and the field's `zones`. |
| 46 | Adds expanded base armor and the Dart Detection Module sweep state. |
| 47 | Adds robot/chassis selection, Engineer and Sentry pilots, balance assist, jump inputs (`RMI4`) and the jump latch in owner anchors (`RMO7`). |
| 48 | Adds fixed-altitude Drone and adjustable balance stabilization (`RMI5` inputs, `RMO8` anchors). |
| 49 | Arms the Drone and carries the live prototype air-support policy. |
| 50 | Adds the Drone's Aerial Safety Rope to the chassis configuration and the `MapMarker` command. |
| 52 | Adds connected deployment and the reliable ownership receipt. |

The console can spawn a host-owned navigation Sentry and send it map destinations.
See [navigation commands and prototype limits](docs/app-options.md#console-spawned-navigation-sentry).

The current protocol version is 79, defined by `PROTOCOL_VERSION` in
`crates/rm-simulator-server/src/protocol.rs`. GNS sends redundant controls
and retried shot intents unreliably; scheduling receipts and terminal shot results
remain reliable. There is no shooter-view fire path or input-acknowledgement
stream. UDP deltas use acknowledged baselines. Server and client must use
matching protocol versions.

### Compression and environment variables

The UDP transport sends independent owner corrections and uses acknowledged
baselines for world deltas. Owner anchors (`RMO8`) are quantized to
millimetre positions, 1 cm/s velocities, 1 mrad/s rates, 0.1 mrad aims and
smallest-three rotations; input state keeps f64 precision, while shot intents
use bounded compression. Owner anchors reference
a configuration delivered reliably and explicitly acknowledged by the client.
World and owner publication cadence is unchanged.

Application pacing defaults
to 64 KiB/s upstream and 512 KiB/s downstream per peer with the default LAN
profile. Native congestion control remains active. These application budgets
exclude native retransmissions and framing; proxy rates remain the wire-capacity
reference.

| Variable | Effect |
|---|---|
| `RM_NET_UP_KIB_S` / `RM_NET_DOWN_KIB_S` | Integer values from 4 to 2048 that override the application pacing for a trial. Invalid overrides use the defaults. |

Measure snapshot bytes and codec CPU with
`cargo run -p rm-simulator-server --example network_bandwidth --locked`.
Periodic checkpoints are packed fine fixed point compressed with the embedded
checkpoint dictionary at ZSTD level 6, except deltas under 128 bytes, which
travel uncompressed; every other wire message is plain ZSTD at level 3. There is no codec selector and no dictionaryless or DEFLATE path.

### Diagnostics

Set `--network-stats off|compact|detailed` or use the Network stats button in P/F3
to show local diagnostics. Native rates and ping are GNS estimates; loss stays
unavailable when the wrapper cannot identify packet loss. The display adds no
server requests. The console reports host downstream queue bytes, age and service
for control, owner and world traffic.

`GET /api/fire-records` on the host HTTP endpoint returns the latest 256 accepted
pilot shots, oldest first. Records include the shooter, launch speed,
authoritative muzzle pose, acceptance time and projectile ID. These diagnostics
never backdate shots or change hit detection. The journal is in memory; export it before host shutdown
if needed. Rejected attempts do not create shot records.

Run `just network-test` to verify the harness without building Rust. With matching
binaries built, `just network-trial scripts/network-scenarios/smoke-single.json
--output /tmp/rm-network-smoke` launches an isolated bandwidth/loss/latency trial
and writes reports. The output directory must be new. A scenario's `expectations`
block asserts named thresholds on the summary, `--baseline PREVIOUS/summary.json`
fails regressions beyond a relative tolerance, and the run reports `passed`,
`failed` or `completed` with a matching exit code.

## Match rules and scoring

The bound field runs the native RMUC 2027 V1.0.0 authority. It covers territory
capture and scoring, mutable Outpost ownership, Base/Outpost damage, combat heat
and allowance, healing and respawn, performance and experience, field buffs,
terrain traversal, Rune opportunities, virtual assembly and adaptive armor.
Source-bound structures and lights follow that authority. All world state and
actuation advance on the explicit 128 Hz clock and travel through authoritative
snapshots and restore.

The Heavy has an experimental physical assembly path: manual parking, Cartesian
controls, collision-aware automatic L1–L3 planning, rate-limited arm playback,
Unit release and recovery. Successful virtual assembly can unlock the current
Hero approximation for Heavy combat. These simulator approximations and remaining
rules gaps are documented in the [field guide](docs/rmuc2027-field.md),
[assembly workbench](docs/assembly-workbench.md) and
[2027 interpretation](docs/rules-2027.md). RMUL formats are not implemented.

Use `--start-paused`, referee controls and the [automation console](docs/console.md)
to inspect a scenario. Unbound 2027 CAD exports support rest-pose practice with
`--no-referee`. They cannot start a match using a legacy referee.

The 2026 runtime and historical rule guide are archived at Git tag
`rm2026-final`. Main retains shared types, robot models and physics, with no
2026 engine or season feature switch.

## Robot equipment

The standalone [Cartesian assembly workbench](docs/assembly-workbench.md) lets
you practice insertion, lifting and rotation with source-bound Engineer and
Tech Core CAD, or a generic arm when no CAD package is supplied. It can generate
seeded rulebook pose candidates and record offline IK animations; source CAD
reachability is checked separately. It is separate from live match assembly progression.
Its offline planner can twist the Unit around the rod across all moving stages;
explicit Core poses, fixed chassis parking and per-stage twist inputs support
repeatable source-CAD scenario tests.
`--fast-collisions` uses convex proxies for the larger Engineer parts while
retaining source geometry at the gripper and fixture; visuals stay unchanged.

`just run --robot hero --third-person` drives the mecanum Hero; omit `--robot`
for the omni Infantry 3, or choose robot then chassis on the title screen.
`--chassis balance` selects the two-wheel Infantry mock with balance assist and
Space to jump (no strafing). `--robot sentry` drives the Sentry like Infantry;
`--robot engineer` drives the Engineer with a fixed arm and no launcher.
`--robot drone` flies a guarded quadcopter on a horizontal plane 2.0 m above
its team's landing pad: W/S and A/D fly in the body frame, Q/E rotate the body,
the mouse aims the gimbal, left click fires its underslung 17 mm launcher, and
V switches view. It has no altitude controls, and a 2.4 m tether on the
section 4.5 Aerial Safety Rope limits where it can go. Balance's serial legs use a four-bar knee-drive
linkage; **P > Controls > Balance stabilization** adjusts the reduced-model
LQR assist from 0% (off) to 100% (full). The setting is remembered. The Sentry
radar follows its gimbal above and behind the camera.
These are procedural approximations of the supplied reference models; no
URDF or STEP meshes are loaded. Hero retains its existing model.
`--performance melee` (Hero) or `--performance power-burst` (Infantry) picks a
section 5.4.2 performance type; see [app options](docs/app-options.md). Both carry approximate AM02 armor, LI01 HP lights,
FI02 underbody RFID hardware, VT03 camera and a muzzle speed monitor. Hero uses
an SM11-sized housing; Infantry uses SM01. HP segments and defeat state follow
the referee. FI02 detection and speed-monitor LED sequences are not simulated.

Every pilot names its robot when it joins, and the host builds that chassis
and fixes its caliber: the Hero fires 42 mm, and the Drone, Sentry and Infantry 3, 4 and 5
fire 17 mm. Muzzle speed and firing rate defaults come from the host. The
robot does not change HP, heat or power policies, and two pilots may pick the
same robot. The visible equipment is decorative;
armor scoring keeps its existing dimensions. [Reference measurements and
limitations](docs/robot-equipment.md) distinguish modeled details from assumptions.

The default omni chassis is a generic 22 kg infantry-sized box (520 × 520 × 100 mm) on
four 153 mm omni wheels centered on the four edges, with a 120 mm turret cube 150 mm
above the body (its top is about 0.42 m above the ground), four small armor
modules on its sides (the outpost's 128 × 113 mm module, leaning back 15°,
scoring on the 101 × 94 mm area of Figure 5-16), 80 mm of unloaded
spring/damper extension with progressive bump stops beyond 120 mm compression, motor-limited
drive (60 N stall, 3.8 m/s no-load wheel speed, so below 3.8 m/s
straight-line top speed) and Coulomb-limited ideal omni tyres. These figures
are assumptions, not rulebook values. A shared 120 W infantry or 160 W hero
drivetrain budget includes mechanical work and torque-dependent motor loss.
Climbing and rotating compete for that budget and available tyre grip.
Robot contacts use low restitution; projectiles touching or leaving the arena
perimeter are absorbed while the boundary still contains robots.
Wheels are ray casts against the ground
mesh, so the chassis climbs the plateau ramps, feels the undulating road
through its wheels, takes the ramps and drops into the
lower floor sections; the body and the turret collide with walls and roofs,
so a covered passage lower than the turret top stops the chassis. The HUD
shows its speed and follow/spin mode. The gun pivot is stabilised above the chassis, so aiming does not
follow the body's pitch and roll. Projectiles that strike an armor module
above the detection speed damage the robot (20 HP per 17 mm, 200 HP per
42 mm, Table 5-2; a module driven into scenery or another robot loses 2 HP) and light it grey for the flash; the module's light bars go dark when
the robot is defeated. Strikes elsewhere on the chassis bounce off without
scoring. The visuals show the omni wheels with their rollers, a two-axis
gimbal bolted to the body (a turntable and fork that turn about the body's
up axis, a cradle between the arms that pitches with the barrel, feeder and
camera; the two angles are solved so the barrel points along the stabilised
aim even on a tilted chassis) and the armor light bars in the team colour.

Review both models without loading field CAD:

```sh
cargo run -p rm-simulator-render --example robots
cargo run -p rm-simulator-render --example robots -- /tmp/robots.png healthy
cargo run -p rm-simulator-render --example robots -- /tmp/robots-hit.png hit
cargo run -p rm-simulator-render --example robots -- /tmp/robots-rear.png healthy rear
cargo run -p rm-simulator-render --example robots -- /tmp/robots-defeated.png defeated
```

The comparison stages Hero HP at 60%. `hit` freezes a front armor flash;
`defeated` switches module LEDs off. These are static visual fixtures.

### Armor artwork

Chassis and outpost armor use the shared [SVG-derived sprite atlas](assets/armor-atlas/README.md).
It includes numbers 1–5, guard, outpost and base identifiers with small/large
variants. Hero uses 1, Infantry 3, 4 and 5 use 3, 4 and 5, and outpost uses O.
Textures preserve the source aspect ratio and face outward. Patterns are passive
white printing; only the light bars stop emitting with disabled armor and remain
white plastic. The external base package uses the base sprite on its moving dart
plate, three upper armor modules and three lower modules exposed by opening the
shields.

## Performance

Use the `inspect_assets` example to measure the selected field package's visual
and collision geometry. Rebuilding, validation and rollback are described in
[field detail](docs/field-detail.md). Runtime level of detail switching and mesh
chunking are not available; GPU occlusion culling is an optional Graphics
setting and is off by default.

Use [the loaded-match CPU probe](docs/performance.md) to measure median and tail
costs with several matches running at once. Test client frame times separately
on the target GPU; a server probe does not establish rendering frame rate.
Engine microbenchmarks and their limits are described in
[performance checks](docs/performance.md).

The separate `rm-simulator-bench` executable loads only visual field assets and
measures rendering with fixed cameras, GPU timestamps and configurable JSON/raw
output. Use `scripts/benchmark-render.py` for repeated settings sweeps. See
[the benchmark guide](docs/render-benchmark.md) for build commands, Vulkan setup,
Metal CPU-only checks, cases and output definitions.

## Distributing builds

Windows release ZIPs include `Join Red.cmd` and `Join Blue.cmd`. Extract the
whole ZIP, double-click the desired team script, and enter the server IP or
hostname. Port 7700 is the default. Both scripts remember the last address in
`server.txt` beside them; press Enter to reuse it. Player names combine the Windows computer
name with a new random suffix for each launch. The scripts resolve assets from
the extracted folder, so no working-directory setup is needed.

Local play in a distributed Windows package uses `Play Local.cmd`. It launches
`bin/rm-simulator.exe --cad-assets field-2027` without `--connect`. Keep the complete
`field-2027/` directory beside `bin/`; the executable also discovers that layout when
launched directly. The embedded host starts automatically.

GameNetworkingSockets itself is linked into the executable. Release packagers must
also include required platform runtime libraries. Windows uses the MSVC runtime;
macOS builds link Protobuf and OpenSSL dynamically, so bundle their dylibs and fix
their install names before distributing. Check the final package on a machine
without Rust, Homebrew or build tools. A Windows package has not been validated
by the macOS test run.

## Development

Install the pinned tools once, then run the full check:

```sh
cargo install prek --version 0.4.14 --locked
cargo install cargo-deny --version 0.20.2 --locked
prek install
just verify
```

`just verify` runs the hooks, formatting, check, Clippy with denied warnings,
tests, the module dependency check, the MPL-2.0 notice check, and `cargo deny`.
The [development guide](docs/development.md) documents the full target list and
where the tests live.

For automation, launch with `just run --console --window-mode headless`, then send
runtime spawn, camera, input and screenshot commands through the
[app console](docs/console.md). `normal` and `unfocused` modes also support it.

Build, CI and release workflow is in [CI and releases](docs/releases.md):
lightweight PR checks, manual full validation, branch protection setup, and the
stable/alpha/beta/RC release workflow with dry runs. Download and platform notes
for a published build are in [release notes](docs/release-notes.md). An optional
`steam` Cargo feature initializes the Steam client and pumps callbacks.
Ordinary builds remain standalone. The [Steam setup guide](docs/steam.md) explains
App IDs, matched SDK/runtime staging and the local SDK's version mismatch.
Steam Input, lobbies and portable distribution packaging remain future work.

## License

`rm-simulator` is dual-licensed under your choice of either the
[MIT License](LICENSE-MIT) or the [Apache License, Version 2.0](LICENSE-APACHE),
at your option. Every source file carries its SPDX identifier and the copyright
line `Copyright (c) 2026 hxyulin <hxyulin@proton.me>`.

Third-party material keeps its own terms. In particular the competitor UI pulls
in five unmodified MPL-2.0 components through Bevy Flair; their versions,
sources and the MPL-2.0 source offer are recorded in [NOTICE.md](NOTICE.md) with
the full license text at [LICENSES/MPL-2.0.txt](LICENSES/MPL-2.0.txt). Run
`just mpl` to check that notice against the resolved dependency graph.

Networking traces can be recorded with `RM_NET_TRACE_DIR=/tmp/rm-network-traces`.
The detailed network overlay reports recording status; console `state` includes
message counters, encoding costs and trace status. See
[network tracing](docs/network-tracing.md) for byte definitions, bounded recording,
and the summary command. Keep traces outside Git.

Owned Sentries offer Automatic/Semi-auto selection and normal/enhanced posture
controls; the HUD shows referee power limits, buffer and cuts. See the
[Sentry controls](docs/app-options.md#sentry-rule-controls) and
[native rules](docs/gameplay.md#sentry-modes-postures-and-interventions).
