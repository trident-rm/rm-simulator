<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# rm-simulator

A first-person RoboMaster field simulator in Rust with Bevy. It loads the
RMUC 2027 field from the extracted competition CAD, runs native gameplay and
projectile rules on an explicit clock (128 Hz in multiplayer), referees a match on top of them, and
lets you drive an omni chassis over the terrain (or fly a free camera) and shoot
at armor. Workspace crates keep gameplay rules, physics, rendering, the headless
server, the interactive application and the rendering benchmark apart.

## Documentation map

Read the shortest document that answers the question before opening a PDF or a
source file.

| Question | Read |
|---|---|
| What the simulator is, install, controls, options | `README.md` |
| Complete CLI reference, bindings, HUD, presets | `docs/app-options.md` |
| RMUC 2027 runtime adapter and candidate inspection | `docs/rmuc2027-field.md` |
| Field package layout, discovery, composition, collision | `docs/field-package.md` |
| Build, test, release and commit workflow | `docs/development.md`, `CONTRIBUTING.md` |
| Current rule sources and historical archive | `docs/referee-rules.md` |
| RM2027 manuals, interpretation and unresolved requirements | `docs/rules-2027.md` |
| Current competitions, archived seasons and admission | `docs/seasons.md` |
| Standalone gameplay engine and its live integration | `docs/gameplay.md` |
| Crate ownership, ticks, restore, ECS | `docs/architecture-refactor.md` |
| Physics reuse without a match or server | `docs/physics-reuse.md` |
| In-process sessions, configurable timing and adapter boundaries | `docs/simulation-embedding.md` |
| Nearest-hit queries and ideal LiDAR | `docs/navigation-sensors.md` |
| Optional ROS 2 / Livox publishing and Pixi environment | `integrations/ros2/README.md` |
| Referee serial/custom-client DTOs, mocks and compatibility link | `docs/referee-link.md` |
| Licensing and third-party provenance | `NOTICE.md` |
| Every guide, indexed | `docs/README.md` |

The `README.md` is user-facing; this file is the engineering contract. Where the
two disagree, this file wins.

## Context

- **Season scope.** The app and headless host support RMUC 2027 V1.0.0 using the bound
  repaired Low `field-2027/`, plus distinct RMUL 2027 V1.0.0 3v3 and
  Infantry 1v1 authorities on explicitly approximate practice layouts.
  RMUL never loads or relabels RMUC CAD; official RMUL geometry is unavailable. There are no season
  Cargo features. The final dual-season revision is tagged `rm2026-final`.
  The 2026 runtime engines and field package are removed; shared data types,
  model descriptors and physics remain. Never reintroduce a 2026 execution path. The supplied 2027 manuals
  (2026-10-15) are interpreted in `docs/rules-2027.md`; RMUC and RMUL require
  distinct competition/revision identities. See `docs/seasons.md` for runtime
  admission, archive instructions and retained internal boundaries.
- **Rules source.** Current rules use the supplied RMUC 2027 V1.0.0 manuals
  (2026-10-15), interpreted in `docs/rules-2027.md`. Historical physical model
  dimensions retain their original 2026 citations. Cite section, table or figure
  numbers for rule constants; identify dimensions read only from drawings.
- **Sibling repositories.** `../Vision/rm-vision-sim` (computer vision
  simulator) and the Embedded simulator share the same rules and the same
  field package; `../rm-map-tools` builds that package from the DJI STEP
  releases (`prototype/export_field_package.py`). Rune rules,
  outpost geometry, CAD scene handling and armor artwork masks were copied
  from `rm-vision-sim` with their provenance notices. Never add a path or git
  dependency on those crates; copy code and keep its provenance notice.
- **CAD colours.** The default package's arena comes from DJI's V1.2.0
  STEP, which styles every arena face (white surfaces, dark grey slab and
  plate tops, beige plate sides, red and blue marking sheets); those colours
  are glTF materials and must be shown as they are. The V2.0.0 STEP styles
  every arena solid and face in the exporter's unpainted yellow-tinted
  cream, with flush cream markings; the renderer's scenery override
  recolours that cream only (`is_unpainted`), never white or painted
  materials. A re-export of V2.0.0 cannot add colour the STEP lacks.
- **Chassis per player, a referee, two teams.** Pilots own one addressed chassis;
  spectators and referees own none. The host enforces ownership, weapon cadence
  and authoritative muzzle poses. IDs are never reused. The native 2027 Match
  owns HP, heat, economy, respawn, control, Rune, assembly and adaptive armor.
  `gameplay::rmul_2027` owns the separate RMUL 3v3 and Infantry authorities;
  it does not execute RMUC control, assembly, adaptive armor or Rune rules.
  `world::referee` retains shared commands and presentation records, not a second
  scheduler. `FieldConfig::default()` is a bare physical world; old rule configs
  are rejected. App inputs always travel through protocol Commands.

## Assets

Four asset locations exist and are deliberately separate. Never merge them; the
loader and the licensing rules depend on the split.

| Path | Tracked | Role |
|---|---|---|
| `assets/` | yes | Build input embedded with `include_bytes!`: armor-atlas artwork masks, the outpost mask and their sources. Small and CAD-free. |
| `crates/rm-simulator-server/assets/` | yes | Build input embedded with `include_bytes!`: the protocol 32 checkpoint ZSTD dictionary. |
| `local-assets/` | no | Gitignored development scratch: extracted `field` package, dated `field.before-*` backups, coarse/preview exports, harness reports. Never source; not referenced from committed code. |
| `~/dev/RM/assets/` | outside the repo | Historical home-directory field installs for offline tools; no runtime fallback. Not this repository's `assets/`. |

The runtime field is versioned in `field-2027/` through Git LFS with its
upstream ownership notice retained. `--cad-assets` overrides discovery;
otherwise the loader checks `field-2027/` beside `bin/` and in the checkout.
App/server sessions reject 2026 CAD. Historical `field/` is archived at `rm2026-final` and removed from main. Files are
verified against SHA-256 in both manifests. The manifest's
`floor_top_source_z_m` is the slab top in the arena frame. Keep original STEP
files and intermediate exports outside Git; only approved packages belong in
LFS. Record runtime package updates in `scripts/release-field-2027.json` and
verify with `inspect_assets`.
Never bake geometry into code beyond the fitted physics and world constants.
See `docs/field-package.md` for composition, collision contracts and deployment.

The RMUC 2027 sandbox displays the complete equipment roster through
`server::roster`; Infantry 3/4, the manual Sentry and assembly-only Heavy have practice models.
`server::zone_catalog` contains diagram footprints. Bound RMUC 2027 V1.0.0
packages use approximate antenna polygons for control, supply, Vantage and
terrain traversal; other unsupported areas remain inspection-only. Use only native 2027 bindings for match observations. `gameplay::rmuc_2027::match_rules` owns the live 2027
combat/control subset, including native heat, allowance, healing, respawn and
combat XP and continuous Base/Vantage/Highland/Fortress field benefits. Virtual
assembly supports referee Start/Finish/Cancel, automatic 45-second timeouts,
shared-core exclusion, income, Infantry caps and permanent defense; Heavy can use an explicitly approximate folding-Hero conversion after launcher
permission unlocks; direct 42 mm combat is supported, suppression remains deferred. Source PIE Heavy automatic L1–L3
manipulation carries a persistent deployment-preloaded Unit from the actual gripper through
approach and insertion, with manually parked chassis and bounded restorable Core positioning
before host worker planning, and restorable 128 Hz playback from persistent actual joints with 1 rad/s joint/jaw limits and
T-selectable tunnel/neutral postures; L3 reveals its Q target only after the P
flip and solves the attached twist separately and follows a validated jaw-release/neutral-return suffix after confirmation, then prescribed Core discharge over the source collection bay and reset; successful Units are spent and cannot be regripped (R60) (`world::assembly`, `server::assembly_rig`,
`app::assembly`); persistent Energy Unit bodies, proximity grip/release and bounded
Cartesian jogs live in `world::energy_unit` and `physics::projectile` (`payload.rs`).
Held/socket-coupled Units remain prescribed; loose Units use source convex proxies
in the shared Rapier scene. See
`docs/assembly-workbench.md`. Adaptive reinforcement
uses frozen selection-time areas, inclusive circle boundaries and clipping without
resizing; undefined selection-time statistics freeze default armor until reselection.
Referee commands stand in for Radar adaptive selection and radio decoding; native Radar marks, localization-module faults and independently sourced enemy contacts are supported. Native Road/Highland/
Tunnel traversal and Small/Big Rune effects are supported. Sentry mode/posture
budgets and paid commander operations are native; referee power uses measured
wheel motor load, a separate 60 J buffer and 2 Hz module-fault sampling. See
`docs/gameplay.md` for timing policies and `docs/tunnel-rfid-calibration.md` for
CAD-aligned tunnel antenna approximations. Source Rune arm
subtrees and detector banks are bound by `scripts/bind-rmuc2027-rune.py` without
remeshing. Adaptive observations/statistics live in `gameplay::rmuc_2027::adaptive`;
Figure 5-1's inclusive permanent heat-lock boundary is the adopted user policy.
`world::rmuc_2027` adapts contacts and source detector
geometry and preserves the entire authority in `FieldRestore`. Source-body
bindings are built by `scripts/bind-rmuc2027-structures.py`; the shared tower
frame stays fixed, detector masks are source-envelope approximations and base
shield travel is illustrative. See `docs/rmuc2027-field.md` before extending it.

## Layout

`integrations/ros2/` is an opt-in, separate Cargo workspace for rclrs publishing
and a Pixi/RoboStack environment. It consumes measured rolling LiDAR frames and
publishes Livox Driver 2 messages plus `/clock`; it adds no ROS dependency to core
crates. Its standalone probe owns a trusted `Field`, not a multiplayer host.
Native ROS checks run separately; keep the ordinary workspace ROS-free.

| Path | Contents |
|---|---|
| `crates/rm-simulator-assets` | Verified manifests, collision GLBs, source binding records, articulation semantics and FLU math. Depends on physics/gameplay types, never world, server, rendering or networking. Server compatibility imports preserve native runtime validation. |
| `crates/rm-simulator-gameplay` | Deterministic match engine the world referee runs as its live rules authority (re-exported as `rm_simulator_world::gameplay`). No physics, rendering or other simulator-crate dependency. |
| `crates/rm-simulator-physics` | Bevy-free, gameplay-free Rapier library. `chassis.rs` owns wheel/body dynamics, `mechanism.rs` owns prescribed robot folding geometry, `projectile.rs` owns stepping and raw armor contacts, `geometry.rs` owns shared collision captures and clearance queries, and `motion.rs` owns prescribed rotor/rail motion and fitted armor geometry. `control.rs` owns direct wheel torque request types; `sensors.rs` owns passive caller-stamped ideal LiDAR over geometry captures; `sensors/imu.rs` owns mounted IMU measurements over opt-in substep carrier traces. No other simulator crate dependency. |
| `crates/rm-simulator-world` | Complete `Field` facade, rune activation, outpost/base HP, referee integration and `scoring.rs` detection/damage. Coordinates physics on explicit ticks and restores the whole world. `session.rs` exposes trusted `MatchSession` stepping, control, truth, match checkpoints and seeded resets (`Session` remains an alias). Re-exports existing physical types through compatibility modules. |
| `crates/rm-simulator-render` | Bevy scene. `cad.rs` (glTF scenery and roles), `rune.rs` and `outpost.rs` (light overlays with lit and struck materials), `projectile.rs` (pooled projectile spheres), `chassis.rs` (body, armor lights, omni wheels, two-axis gimbal), `sync.rs` (`SceneState` in, transforms and materials out), `lighting.rs`. Never depends on the world crate. |
| `crates/rm-simulator-server` | Bevy-free glue and the `rm-simulator-server` binary. asset compatibility imports (verified CAD loading and collision geometry supplied by `rm-simulator-assets`), `compression.rs` (the self-identifying wire framing for non-checkpoint messages: ZSTD or, for the loopback transport, raw), `math.rs` (wxyz quaternion and column-major matrix helpers, the glTF root pose convention), `layout.rs` (rune hubs, outpost origins, terrain into the field, team spawn slots and the `ChassisSpawner`, `FieldConfig` from options), `simulation.rs` (`Simulation`: paused flag, bounded real-time advance, command application, chassis spawning per player), `protocol.rs` (`ClientMessage`/`ServerMessage`, roles), `host.rs` (single-owner simulation worker, roster and command authority), `net.rs` (the GNS UDP sockets, the in-process owner's loopback codec link, peer delivery and `Client`), `network_trace.rs` (bounded metadata tracing and local counters), `udp_codec.rs` (the per-peer UDP codec with no socket in it: fragment framing, reassembly, input batches, delta coding and pacing, all on an explicit `now`), `gns_transport.rs` (the GNS sockets that drive that codec), `scripted_link.rs` (a deterministic datagram link with scripted loss, reordering, duplication, delay and blackouts, for tests), `host_args.rs` (the field, rune, weapon and chassis options a host accepts, shared with the app's command line), `workload.rs` (the deterministic no-CAD field the bandwidth probe and the measurement examples drive), `http.rs` and `panel.html` (minimal HTTP/1.1 server and the referee page), `main.rs` (headless binary). Depends on the world crate; the only place besides the app that reads host time. |
| `crates/rm-simulator-app` | The `rm-simulator` binary. `main.rs` (app wiring), `args.rs` (clap arguments), `loading.rs` (the `Screen` state and the match lifecycle: entering `Screen::Loading` with a `JoinRequest` prepares a session on a worker behind a splash, `Screen::InMatch` unlocks gameplay, a `LeaveRequest` or any failure tears the match down to `Screen::Title`), `staging.rs` (connected side/robot/position selection over the real scene; deployment uses the host protocol), `title.rs` (the title screen, its `Page` sub-state and the remembered fields; every choice becomes the arguments a command line would have given), `session.rs` (`Session` over a `Client` for both embedded and remote hosts, prediction state and match keys), `controls.rs` (gimbal camera, drive and fly, gun, mouse capture), `scene.rs` (CAD instances, overlay spawning, world-to-scene adaptation and flashes), `hud.rs` (overlay text), `debug.rs` (collision wireframe view), `screenshot.rs` (`--screenshot`), `frames.rs` (FLU-to-Bevy conversions). |
| `crates/rm-simulator-bench` | Standalone fixed-camera visual CAD benchmark. Shares renderer and graphics presets; never depends on physics, world, server or gameplay. GPU timestamp readbacks, CPU frame distributions, settings cases and optional raw output. See `docs/render-benchmark.md`. |
| `scripts/check-module-dependencies.py` | Asserts the crate boundaries above. |
| `scripts/check-mpl-compliance.py` | Asserts every resolved MPL-2.0 dependency is allowlisted in `deny.toml`, named at its locked version in `NOTICE.md`, and unmodified. |
| `LICENSE-MIT`, `LICENSE-APACHE`, `LICENSES/MPL-2.0.txt` | The workspace's dual license and the vendored MPL-2.0 text. |
| `README.md`, `CHANGELOG.md`, `NOTICE.md` | User docs, user-visible changes, licensing and provenance of reused code. |
| `docs/referee-rules.md` | Digest of the rulebook clauses the referee implements (clock, rune opportunities, buffs, outposts), with section and table numbers; read it before the PDF and update it when a rule changes. |

## Architecture rules

These define the shape of the system. Changing one is an architectural change,
not a refactor.

- **Crate boundaries.** Physical dynamics and prescribed motion in physics,
  live rules in world, drawing in render, Bevy-free glue
  and networking in server, the window in app. The world and server crates
  never depend on Bevy; the render crate never depends on the world or
  server crate, or physics crate. Physics never depends on world, gameplay,
  server or Bevy. The app converts world snapshots into render `SceneState`
  and sends every input as a protocol `Command`, whether the simulation is
  local or remote, so multiplayer needs no new input path.
- **Hosted ownership.** `host.rs` owns each live `Simulation` and its roster on
  one worker. Local loopback, GNS UDP and HTTP submit typed requests through `HostHandle`; never
  restore shared mutable simulation access in a transport or app system.
  The embedded owner is not a separate typed path: `Server::connect_owner` runs
  the same `udp_codec` per-peer codec a UDP client runs over two in-process
  datagram channels, with transport compression skipped (`raw`; the bounded
  assembly trajectory payload is independently packed/compressed), so bitpacking, baselines,
  input batches, owner anchors and pacing are shared and only the socket and
  ZSTD differ.
  Queue confirmation snapshots and Pongs in the worker's application order;
  encode broadcasts on socket writers. Replace only unsent periodic snapshots
  in the bounded peer outbox, never confirmation snapshots or reliable messages.
  Encode snapshots only after dequeuing the selected frame; decode before client
  inbox coalescing. Addressed local pilots use 4 ms owner-only anchors and 32 ms
  world publications; owner-only tails never replace a waiting world checkpoint.
  World frames lazily share quantization, independent packing and compression on
  delivery workers, retaining per-peer baseline state and logical counters. Confirmation snapshots stay full and precede Pong. Only the
  acknowledged UDP baselines carry deltas, and their retained-baseline scheme is
  not a revision chain.
  `Server::bind` consumes the simulation.
  Per-peer codec state is transport-agnostic and clock-injected: `udp_codec.rs`
  takes an explicit `now` and never touches a socket or the wall clock, so the
  socket loops stay thin and the same codec runs under a scripted link in tests.
  Frame-facing debug requests must remain nonblocking and keep at most one
  dynamic geometry capture pending.
- **One world.** A client never builds a reduced physics world of its own.
  `Field::restore` rebuilds a whole `Field` from a `FieldSnapshot` plus the
  shared `StaticGeometry` and floor height, and prediction replays it through
  the ordinary `Field::step`, `command_chassis` and `fire` paths. Snapshots
  therefore carry the rules' hidden state (`FieldSnapshot::restore`); a decoder
  that drops it gives up restoring, not drawing. What a restore cannot carry is
  solver state: contact manifolds and warm starts are rebuilt. Rebuild only when
  the checkpoint changes, never once per frame.
- **One screen at a time.** The app's `Screen` state (`loading.rs`) is the
  authority for the title screen, the loading splash, connected deployment
  (`Screen::Staging`) and a running match, and
  the title screen's pages are a `Page` sub-state of `Screen::Title`, so leaving
  the title screen removes them and returning opens the main menu. Screens spawn
  on `OnEnter` and despawn on `OnExit`; gameplay systems run under
  `in_state(Screen::InMatch)`. Never reintroduce a resource whose presence means
  "which screen is up". The HUD is not a screen: its panels and display
  preferences are independent flags on `HudState`, and its entities are spawned
  once and mutated from simulation data.
- **Map intent boundary.** `server::map_interaction` models 2026 §1.3.1 and
  MapClickCmd recipient/marker intent. The host derives source/team and routes
  bounded numbered events; this is not a Sentry controller or world rule state.
  Transport-clock rate limits continue while paused. World FLU/chassis identities
  require explicit conversion before a future referee UART/MQTT bridge.
  `server::navigation` optionally consumes location intents for operator-spawned
  Sentries only: one bounded map/search worker, ordinary chassis commands on
  explicit ticks, path diagnostics through snapshots, and braking on obstruction
  or stalled progress, with at most three recovery replans per goal. Inflated
  terrain checks, explicit 2027 Road exclusions and bounded per-Sentry learned
  failure regions constrain navigation only. Manual Sentry pilots default to manual control and may opt into host optical aim/sweep; operator navigation Sentries default to that controller. It yields to tunnel/stow alignment and never fires.
  The CAD bake is an in-memory host cache; controller state is external input,
  not a second physical world or an addition to the referee rules.
- **Embedding.** `physics::session::RobotSession` owns a bare `WorldPhysics`
  with explicit time and scenario-recipe reset, without match authority or arbitrary
  checkpoint continuation. `world::session::MatchSession` (`Session` compatibility
  alias) is a trusted caller-owned facade over the same
  `Field` control and step paths. It adds no transport authority or wall-clock pacing.
  Ground-truth access is explicitly privileged and is not a simulated sensor.
  Passive `physics::sensors` acquisitions take explicit geometry, pose and time;
  their caller owns scheduling. Mounted rolling scans retain self-occlusion and
  interpolate coherent per-step robot/collider captures; they reject topology
  changes and robot placement discontinuities. Optional seeded range effects
  are indexed by global emission number, independent of batching. Sensor output
  contains local points and timestamps, not privileged per-point world poses.
  Caller adapters own sampling schedules, random generators and task/reward state.
  Direct wheel torques bypass velocity/balance assistance but preserve physical
  motor, power, traction and defeat limits; gimbal/folding remain prescribed.
- **Renderer is passive.** It applies caller-owned `SceneState` and never
  advances rules or reads the physics/world crates. Chassis ingestion resolves
  caller state into pose and light components before applying changes. Keep
  material swaps and transform writes conditional on changed values.

## Simulation rules

- **Coordinates.** The world uses metres in forward/left/up (FLU) with wxyz
  quaternions (`Pose`). Bevy is right/up/back. Convert only at the renderer
  boundary with `apply_pose`, `flu_position`, `flu_vector` and
  `pose_from_transform`. The playing floor top is at height zero. Red's
  half is +x and blue's is −x (the V1.2.0 CAD paints its red markings
  there); `layout::side_team`, `rune_team` and `outpost_team` derive
  ownership from that, and the render `TeamColor` follows it.
- **Time.** World time advances only through explicit ticks (`tick_ns()`,
  defaulting to 128 Hz (7.8125 ms per tick). In-process physics/world sessions
  may select a validated immutable `StepDuration`, including 1000 Hz. The duration
  travels in checkpoints and controls both integration and field time. Multiplayer
  remains 128 Hz and rejects custom-rate admission until peer timing is negotiated;
  there is no environment override or CLI rate flag). Rule durations
  stay in
  nanoseconds, never tick counts. Never read
  host time in physics, world or gameplay code. Stepping must be deterministic
  and independent of how ticks are partitioned; there is a test for this.
  With no projectiles, chassis or native authority, `Field::step` jumps
  directly to the target tick. A referee keeps the tick loop active even when
  physics is idle; active bodies also advance tick by tick.
- **Physics.** Projectiles are rapier dynamic balls with CCD, kept until the
  four-second flight limit, the arena bounds or the 64-ball cap removes them,
  or until `ProjectilePolicy` retires a spent one that has rested at or below 2 m/s
  on stationary scenery for 50 ms; armor housings
  are kinematic cuboids that follow the rule poses each tick; the ground is
  a static trimesh supplied by the server layout adapter from the manifest's
  visual GLBs, including sheets and markings. The rune and outpost contribute
  only their `static` nodes; moving targets remain rule-driven bodies.
  Bases, energy cores and extra scenery also use their visual triangles
  unless the asset explicitly declares `collision_method: source-tessellation-v1`.
  That contract selects a checksummed, independently tessellated mesh of the same
  source faces. Legacy unmarked proxies remain ignored. User-approved approximate collision
  meshes may replace exact tessellations when generated reproducibly by scripts
  here or in rm-map-tools. Preserve ramps, clearances and scoring behavior, and
  record provenance, deviation checks and placed triangle counts. A bare
  physical world has a catch half-space at `floor_height_m`, zero by default.
  The loaded CAD layout removes that plane and adds a perimeter boundary.
  Mesh contacts are two-sided.
  `Field::static_geometry` returns what the physics holds, for the app's
  wireframe. The chassis is a
  dynamic body with an octagonal deck and compact lower bumper and wheel guards for omni
  Infantry/Sentry (box for other presets), a turret cuboid and four massless
  armor housings. Ray-cast wheels have independent spring/damper suspension,
  lumped rotational inertia, motor torque and slip-limited omni tyres;
  projectile colliders never support suspension rays. The chassis steps in the
  same world as projectiles, which score on armor and bounce off the rest. `C` in the app draws the physics geometry as a
  wireframe of the client's presented scene. The debug view uses verified local
  geometry and sends no host capture requests. Raw contacts leave physics in
  order and are detected/scored by the world at the end of each tick.
  Scoring faces are `TargetFace` poses with +x the outward normal, +y width,
  +z height.
- **Ground.** The default (V1.2.0) floor slab is flat at height zero with
  the terrain plates standing on it. The V2.0.0 slab is crowned: about
  0.11 m below the reference height along the centre line, falling to about
  0.32 m below at the side walls, with height zero at the small level pads
  near (±12.8, ±3.0) m. Never assume a floor height; ask the collision mesh
  (`CollisionMesh::ground_height_below`). Both releases put a hex-marked
  truncated pyramid on the centre line at x ≈ ±6.3..8.5 m (1.5 × 1.0 m
  top, 0.15 m high, 17° ramps; the default spawn faces the red-side one).
  The 起伏路段 undulating road, a 2.4 × 2.1 m strip of waves about 70 mm
  high on the 0.2 m deck near each side wall (x ≈ 6.3..8.7, y ≈ 5.4..7.5 m
  and its point mirror), is modelled only in the V2.0.0 STEP; the default
  package carries those two V2.0.0 solids grafted onto its flat deck (the
  manifest's `grafted` entry), so both maps have it. The passages under the
  base highland decks have 0.65 m of clearance in both.

## Code rules

- **Units and naming.** Standard Rust naming with explicit physical units on
  public numeric fields and constants (`_m`, `_rad`, `_ns`, `_rad_s`,
  `_m_s`). Rulebook constants get a doc comment with the source.
- **Documentation.** `gameplay`, `physics`, `world`, `render` and `server` deny
  `missing_docs`, so every new public item there, including struct fields and
  enum variants, needs a doc comment. Say what the item does, its units and the
  invariant it keeps; cite the rulebook section or table a constant comes from.
  Add a runnable doctest when a caller can exercise the item without a GPU, a
  socket, a file or the wall clock, and hide setup behind `# ` lines. The `app`
  and `bench` binaries are not linted, but their module interfaces are
  documented the same way.
- **Licensing.** The workspace is dual-licensed `MIT OR Apache-2.0`. Every file
  starts with its SPDX identifier and the line
  `Copyright (c) 2026 hxyulin <hxyulin@proton.me>`. Keep provenance notices on
  copied code, and never assert this copyright over third-party artwork,
  recorded measurements or license texts. Dependencies must satisfy
  `deny.toml` (permissive licenses, plus the explicitly approved MPL-2.0
  exceptions for Flair's CSS dependencies). The MPL-2.0 source offer lives in
  `NOTICE.md` and is enforced by `scripts/check-mpl-compliance.py`.
- **Keep it small.** Avoid unsafe code and new dependencies unless the task
  needs them. Prefer extending an existing module over adding a crate.

## Workflow

- Follow the commit and PR convention in `CONTRIBUTING.md`: `type(scope): summary`,
  at most 72 characters, optional short body. Squash by default; rebase only
  independently passing commits. Keep main linear.
- When local CPU contention makes heavy checks slow, prefer an isolated source
  copy on Talos over waiting for shared local build capacity. Use the fleet
  access instructions, preserve remote work, and record the tested source and logs.
- Run `just verify` before opening a PR. It runs the hooks, formatting, check,
  clippy with warnings denied, all tests, the crate-boundary script, the MPL
  notice check and `cargo deny`. `just run <args>` runs the app; `just server
  <args>` the headless server; `just world-test` the fast rule tests while
  iterating. See `docs/development.md` for the full target list.
- `--screenshot PATH` renders the loaded scene to a PNG and exits; use it to
  check visuals without a display session. `--start-paused` and `F7` step the
  world by one manual step — three 128 Hz ticks, 23.4375 ms — for inspection.
- Record user-visible changes under `Unreleased` in `CHANGELOG.md` and keep
  the README option and control tables current.
- Tests live beside the code. Run `cargo test -p rm-simulator-physics --locked`
  for dynamics, geometry and raw-contact tests. World tests build a `Field` from a
  `FieldConfig` and step it; render tests run a headless `App` with the sync
  plugin and inspect components; server tests exercise the protocol, the UDP
  client flows and the HTTP routes; app tests cover argument
  parsing, frame conversions, the flashes, the HUD line and a deterministic
  end-to-end network trace (`net_harness.rs`) that runs a real host and a real
  session over `scripted_link.rs` on a hand-advanced clock.
