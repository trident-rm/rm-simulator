<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# App options, controls and interface

The complete command-line reference for `rm-simulator` and
`rm-simulator-server`, the input bindings, weapon settings and the graphics
presets. For what the simulator does, see the [project README](../README.md);
for automation, see [console commands](console.md).

## Command-line options

`rm-simulator` accepts every option below. `rm-simulator-server` accepts the
field and weapon options that a host owns; a client
joined with `--connect` takes those from the server.

| Option | Purpose |
|---|---|
| `--play` | Skip the title screen and start a local practice match at once |
| `--staging` | Connect first, then choose side, robot and position over the real field; works with local hosting and `--connect` |
| `--console [ADDR]` | App automation console on localhost, default `127.0.0.1:7790`; see [console commands](console.md) |
| `--referee-link [ADDR]` | With the `referee-link` feature, serve the custom-client referee protocol over MQTT, default `127.0.0.1:3333`; see [referee link](referee-link.md) |
| `--referee-video [ADDR]` | With the `referee-link` feature, send the pilot's view as HEVC over UDP, default `127.0.0.1:3334`; needs `ffmpeg` with libx265; see [referee link](referee-link.md) |
| `--window-mode normal\|unfocused\|headless` | Normal visible window, visible without requesting focus, or GPU rendering without an OS window |
| `--robot hero\|engineer\|infantry-3\|infantry-4\|infantry-5\|sentry\|drone` | Robot you drive on any host. Hero fires 42 mm; Infantry (default), Sentry and Drone fire 17 mm. Engineer has a fixed decorative arm; Drone flies at fixed altitude. Engineer has no launcher |
| `--chassis auto\|omni\|balance\|mecanum\|flight\|tunnel-omni` | Default `auto`: Infantry/Sentry use omni, Hero/Engineer use mecanum; Drone uses fixed-altitude flight. Infantry also offers balance (two wheels, mock balance assist and jump). Sentry also offers `tunnel-omni` (verified EGA package required); explicit `omni` retains the standard model. Incompatible choices are rejected |
| `--performance TYPE` | Section 5.4.2 performance type requested after joining: `long-range` or `melee` for the Hero, `hp-cooling`, `hp-burst`, `power-cooling` or `power-burst` for an infantry. Unset keeps the rulebook default; a host refuses a type for the other class or during a running round |
| `--assembly-assets DIR` / `--assembly-area DIR` | Verified in-game Engineer/Core rig and matching station terrain; see [assembly workbench](assembly-workbench.md#in-game-automatic-assembly) |
| `--assembly-hints FILE` | Optional completed source recording for warm IK seeds; every new path is validated |
| `--robot-assets DIR` | Optional verified source-derived Omni, folding Hero, stair Heavy or tunnel Infantry/Heavy/EGA Sentry package; selects its fitted local physics and validated replacement armor when supplied |
| `--stair-heavy` | Host selects the DreamChaser stair linkage; pilot with `--robot engineer` |
| `--folding-hero` | Host selects the reference folding Hero preset; live 2027 deployment uses Heavy |
| `--team-cad` | Host option selecting the fitted team Omni dimensions without requiring visual assets |
| `--cad-assets DIR` | Extracted RMUC CAD directory; relative paths start at the working directory for both manifests and meshes |
| `--big-rune` / `--no-rune` | Big Rune motion and two-target groups for training (a match always starts with the Small Rune and converts at 3:00), or no rune rules |
| `--outpost-speed-rad-s R` | Armor ring rotation rate |
| `--spawn X,Y,Z` | Start position in FLU metres; driving, the chassis is set down on the highest ground below Z (default: your team's half, before the centre-line plateau: red at x = +9, blue at x = −9) |
| `--spawn-yaw-deg D` / `--spawn-pitch-deg D` | Start heading and pitch (default heading: facing the opponent's half) |
| `--fly` | Free-flying camera without a chassis; on a remote host, join as a spectator (no chassis, no firing) |
| `--referee` | Join as the referee: the free camera, no chassis, and the match keys (`F5`, `F6`, `F7`, `F`), which a remote host grants to nobody else |
| `--third-person` | Start driving in the third-person view (`V` toggles) |
| `--collision-view hidden\|overlay\|alone` | Start with the physics geometry drawn as a green wireframe over the scenery or on its own (F3 panel; built in the background on first use; default hidden) |
| `--start-paused` | Open with the world clock paused |
| `--screenshot PATH` | Capture the window or headless camera to a PNG once loaded and settled, then exit |
| `--muzzle-speed-m-s V` | Starting muzzle speed for 17 mm robots, default 25 m/s (the caliber follows each pilot's robot; a Hero uses the `--hero-*` weapon below); unavailable with `--connect` |
| `--fire-rate-hz R` | Starting firing rate in simulation time, default 20 Hz; unavailable with `--connect` |
| `--max-fire-rate-hz R` | Host firing-rate cap, default 30 Hz; unavailable with `--connect` |
| `--max-muzzle-speed-m-s V` | Host actual launch-speed cap, default 30 m/s; unavailable with `--connect` |
| `--muzzle-speed-variation-m-s V` | Host default Gaussian speed variation, 0..1 m/s maximum offset, default 0.3; unavailable with `--connect` |
| `--spread-deg D` | Host default spread half-angle, 0..90 degrees; default 0.3; zero means perfect accuracy; unavailable with `--connect` |
| `--spread-distribution uniform\|gaussian` | Host default distribution: Gaussian angular offsets truncated at three sigma, or a uniform solid-angle cone; unavailable with `--connect` |
| `--spread-seed N` | Repeatable spread seed, default 0; unavailable with `--connect` |
| `--hero-muzzle-speed-m-s V` / `--hero-fire-rate-hz R` | A Hero's own starting 42 mm muzzle speed and firing rate, default 12 m/s and 1 Hz; unavailable with `--connect` |
| `--hero-max-muzzle-speed-m-s V` / `--hero-max-fire-rate-hz R` | Host caps for a Hero's weapon, default 16 m/s and 2 Hz; unavailable with `--connect` |
| `--hero-muzzle-speed-variation-m-s V` / `--hero-spread-deg D` | A Hero's default speed variation and spread, default 0.1 m/s and 0.1°; the spread distribution and seed are shared; unavailable with `--connect` |
| `--debug-panel` | Open the F3 debug panel at startup |
| `--network-stats MODE` | Show local network diagnostics; also selectable in P/F3 |
| `--render-stats` | Show frame and renderer CPU statistics at startup |
| `--zones` / `--rfid-zones` / `--zone-labels` | Show seasonal buff volumes, functional RFID footprints and names/traversal order; also selectable in F3. 2027 estimates are debug-only. |
| `--wireframe` | Draw visual mesh wireframe lines if supported by the GPU |
| `--no-field-collision` | Skip the CAD collision meshes; only a flat floor at height zero and armor collide |
| `--no-referee` | Run the field without a referee: the rune is active from the start and there is no match |
| `--no-projectile-retirement` | Keep spent projectiles until the four-second flight limit; by default a ball resting on scenery at or below 2 m/s for 50 ms is removed |
| `--no-prediction` | Use host-driven movement, aim and firing for comparison |
| `--connect HOST:PORT` | Join a running server instead of simulating locally (field options come from the server) |
| `--lobby-name NAME` | Advertise a LAN lobby while using `--listen` |
| `--password TEXT` | Password for hosting or joining; not saved by the menu |
| `--lobby-host HOST:PORT` | Future public directory, default `127.0.0.1:7791`; LAN never contacts it |
| `--public-lobby` | Reserved public listing option; currently rejected |
| `--advertise-address IP:PORT` | Reserved public game address override |
| `--listen [ADDR]` | Host the local world for other clients (default `0.0.0.0:7700`) |
| `--http [ADDR]` | Serve the HTTP referee panel (default `127.0.0.1:7780`), including native 2027 assembly and adaptive-armor selection controls |
| `--team red\|blue` / `--name NAME` | Team and name announced to the server (default red, `pilot`); a client that names no team is put on the team with fewer pilots. The referee joins on neither team but `F` still spends this team's opportunity |
| `--rune-flash-hz F` / `--rune-flashes N` | Blink rate and count of an activated rune's arms (default 2 Hz, 3 blinks, then lit; rate 0 never blinks, count 0 blinks until the state changes) |
| `--hit-flash-ms T` | How long a struck armor module shows grey (default 50) |

`--play`, `--connect`, `--listen`, `--screenshot` and `--console` skip the title
screen and enter a match at once; the remaining options are the defaults every
join from the title screen starts from.

## Menu feedback and navigation

The main menu is one vertical stack: Single Player, Multiplayer, Settings and
Quit. The player name is asked for on the Multiplayer page, above the lobby
columns, because only joining and hosting send it; a local practice match uses
`--name` or `pilot`. An empty name is reported beside the Join or Create lobby
action that needed it.

In Multiplayer, Enter submits the form containing the focused input: lobby
creation fields create a lobby, while the address and join-password fields join
one. Passwords start masked; each field has its own Show password checkbox.
Passwords are never remembered. Search progress appears beside Refresh LAN;
validation and lobby-selection messages appear beside the relevant action.
LAN help opens troubleshooting on request; Escape dismisses it. Menu pages
start at the top and clear focus from inputs on the previous page.
Loading can be cancelled with Escape, including while scenery and the first
frame are being prepared.

Settings keeps its tabs and Reset / Close buttons visible while the selected
page scrolls. The active tab is underlined; switching tabs starts at the top of
that page.

## Connected deployment lobby

Single Player, Join lobby / address and Create lobby connect first and open the
same staging screen over the real 3D field. Solo uses the embedded host and
client without a network listener. No robot exists for this player yet.

1. Hover over a half of the arena to highlight it, then click to join that side.
   The arena faces you with its long edge horizontal and the camera tilted down.
   Keyboard shortcuts: 1 chooses the left (blue) half, 2 the right (red) half.
   Watch as spectator skips robot deployment.
2. Choose Hero, Engineer, Infantry 3/4/5, Sentry or Drone and a supported chassis.
   The preview uses the normal robot renderer.
3. In the top-down base view, click inside the outlined ring, outside the base.
   Arrow keys adjust the selected point by 0.1 m; Enter or Deploy submits it.

The host checks the team ring, drivetrain and occupancy, then assigns a chassis
on the existing connection. A refusal leaves the form open so you can retry.
Drone retains its designated aerial pad and flight boundary. The outlined ring
is this simulator's deployment policy, based on its Base-area footprint and the
approved minimap prototype, not a claim of official competition starting rules.
Multiple players can still choose the same robot class; no exclusive roster
slots are enforced. Spectators and referees have no robot. `--referee --staging`
opens a referee-only entry rather than offering pilot deployment.

Escape goes back one step and leaves the lobby from the side-selection step.
Deployment remembers the chosen team, robot and chassis. Host weapon limits
and defaults are on the Multiplayer page; remote joins use the host's configuration.
Direct-play CLI commands preserve their immediate spawn behavior. Add
`--staging` (alone, with `--connect`, or with `--listen`) to use this flow.

## Balance stabilization and Drone flight

**P > Controls > Balance stabilization** sets 0–100% assistance while in a
match and is remembered between launches. Zero disables stabilizing torque;
100% applies the full reduced-model LQR. Intermediate values scale the torque
and damping, so low settings can let the robot tip. The slider affects only
Balance, and travels to the host through normal pilot inputs. Reset all settings
restores 100%. Space still requests one jump per grounded press.

Drone offers the Flight chassis. Flying and aiming are separate: W/S fly
forward and back and A/D sideways in the body frame, Q/E rotate the body
(1.5 rad/s, 3 rad/s with Left Ctrl), and the mouse aims the gimbal, which can
pitch down to about 69°. Flight is 2 m/s, 4 m/s with Left Ctrl. V changes
viewpoint. The drone starts 2.0 m above its team's landing pad; Space/Shift do
not change altitude. A 2.4 m tether on the Aerial Safety Rope (rulebook
section 4.5) runs from the team's edge to the centre line along the pad's
side; the drone brakes at its reach and is pulled back inside, and the HUD
robot card shows the remaining slack. The rope height is not in the rulebook;
the simulator assumes 3.6 m, which leaves about 1.8 m of sideways reach. Left click fires an
underslung 17 mm gimbal launcher, including when aiming downward. Live matches
do not require activating air support for this prototype; normal cadence, heat
and configured ammunition checks still apply.

## Weapon settings

Weapon settings can be changed during a match under **Settings > Weapon**.
Starting values are 20 Hz, 25 m/s, +/-0.3 m/s Gaussian speed variation and
0.3 degrees of Gaussian angular spread. Host caps default to 30 Hz and 30 m/s.
The bullet-speed HUD shows the actual last confirmed launch speed to two decimal
places, or `-- m/s` before the first shot. Rejected shots and settings changes
do not alter the reading.
Each player chooses a fire rate and muzzle speed within the host's limits,
and speed variation, a spread angle, uniform or Gaussian distribution, and seed.
The in-game spread slider covers 0..2 degrees in 0.01-degree steps; 2 degrees
gives a group about 35 cm across at 5 m. Zero means perfect accuracy. Host CLI
defaults can still specify larger cones up to 90 degrees. The angle is the
maximum deviation from the muzzle axis, or three standard deviations for
Gaussian spread. Gaussian and normal mean the same distribution; Gaussian is
the angular default. Muzzle-speed variation is always Gaussian: 0..1 m/s sets
the maximum +/- offset, with standard deviation one third of that value.
Samples stay positive and within the host's speed cap. Thus 30 m/s with +/-1
at a 30 m/s cap yields 29..30 m/s, while 29 m/s with +/-1 yields 28..30 m/s.
The host retains each pilot's settings and applies them at
launch, including to queued fire requests. Already flying bullets are unchanged.
The client sends one reliable update when settings change and waits for its
confirmation before sending new shots. A seed, chassis id and intended launch
time determine the spread sample, so prediction can reproduce it.

Expand **Host weapon limits and defaults** on the Multiplayer page to set the rate and speed
limits and default spread before creating a lobby. These
fields are remembered. The caliber is not a setting: each pilot's robot fixes
it, 42 mm for the Hero and 17 mm for the infantries, and the host refuses a
weapon update that names another. Remote joins use the host's defaults; in-game adjustments
last for the current session. The server binary accepts the same weapon flags.

## Interface and HUD

The title screen verifies and loads the field in the background. A bottom bar
reports loading stages; the live arena backdrop gently follows the pointer.
Joining reuses the verified assets and imported scenery, including when selected
before background loading finishes. Match physics and connections still start
on join. The active graphics preset also applies to the menu backdrop.

The HUD follows the July 2026 RMUC competitor client manual: red/blue team
status and clock across the top, robot HP below left, a separate ammunition readout along the bottom, and a top-down teammate map below right. The ring around the
reticle fills clockwise with barrel heat, including shots not yet confirmed
by the host, turning amber past 60 % and red past 85 % of the heat limit. Teal marks your robot;
defeated robots fade. Referees see both teams on the map. The map uses optional
CAD-derived artwork and live positions, without radar
detection. The local status readout includes base HP and shield; ammunition and team gold
come from the match engine, with the robot's level, heat, weakened and
invincible states, respawn progress and the round result. Hardware-module
status and a full competition purchase interface are not modeled by the HUD. The overlay uses native Bevy UI with a responsive
Flair stylesheet; Feathers supplies mouse-driven settings and toolbar buttons.
Controls and graphics preferences are saved in `settings.json` beside the
remembered `title.json`. Open Settings from the title screen or toolbar to rebind
keyboard/mouse actions, invert mouse Y, adjust sensitivity, and select Low,
Medium, High or Ultra graphics with individual overrides. Help labels follow
saved bindings. Graphics changes apply live and do not change simulation physics.
Hover a graphics option or focus it with the keyboard to read its visual effect
and performance tradeoff.

The HUD stylesheet is embedded from `crates/rm-simulator-app/src/hud.css`, so
UI assets work with any `--cad-assets` location. It follows the July 2026 student
manual's main interface and panels 1, 3, 5 and 6. Mirrored robot slots across the
top show each team's individual robot HP. Empty slots are dimmed; defeated
robots show DOWN. Duplicate types get extra slots so every robot remains visible.
Slot numbers follow the manual; simulator chassis ids remain in the Team panel.
Base HP is not represented by an aggregate robot health bar. The shared
`ui_theme.rs` palette and sans-serif typography apply to menus, Feathers controls
and contextual panels; the HUD uses the matching embedded Flair stylesheet.
Notices flow above the robot card. The normal 2027 HUD shows a quiet territory
summary; Team and F3 views expand capture, RFID and Rune diagnostics. XP and
purchase costs appear in the robot card while using the cursor.

Settings keep navigation and footer actions visible while the selected page
scrolls. A sidebar becomes horizontal tabs below 900 logical pixels. The main
menu uses a compact card; multiplayer forms keep the wider responsive layout.

Press Escape to open the Pause menu with Resume, Settings, and Exit Match.
Singleplayer pauses until you resume; multiplayer keeps running, including when
you host. Settings opened from Pause return to that menu. A match that was
already paused stays paused when you resume. On the title screen, Escape opens
a quit confirmation with Cancel and Quit.
Clicking the large map (M) opens a choice at the selected point; hold Left
Alt to free the cursor and click the small map the same way (it expands first).
Choose **Place local marker**, **Send to Sentry**, **Send to team (except Sentry)**,
or **Send to team + Sentry**. Cancel or Escape sends nothing. Optional key data
starts at zero; **Use selected map letter** includes the current A/B/I/custom
letter as an uninterpreted byte. A, B and I select
the attack, defend and alert markers while the large map is open, and other
letters a custom lettered marker, following pages 24–25 of the competitor client
manual. A local marker is drawn and recorded at the host. Accepted interaction events
are delivered only to selected own-team clients (and acknowledged to the sender);
the latest location intent is drawn in cyan. Spectators and referees cannot send.
Only a console-spawned navigation Sentry consumes location intents; manual
Sentry pilots remain manual. No intent automatically aims or fires.
Clicking the small minimap without Left Alt still opens its expanded view. Settings, the expanded map, and toolbar-opened panels stop driving,
aiming and firing and release the cursor. Hold Tab for both teams' players and HP;
hold F12 for controls. Scroll the wheel to see longer lists while holding the key.
These hold-to-peek panels do not pause gameplay. Close a panel, then click the field
to resume control. The collision wireframe remains an optional developer view.

## Graphics presets

| Preset | Shadow map | Cascades | Shadow distance | MSAA | Bloom |
|---|---:|---:|---:|---:|---|
| Low | Off | — | — | 1× | On |
| Medium | 1024 px | 1 | 40 m | 2× | On |
| High | 2048 px | 1 | 40 m | 4× | On |
| Ultra | 4096 px | 2 | 60 m | 4× | On |

**Warning:** Changing Light emission can alter armor plate and target colors or
hide gameplay indicators, without a meaningful performance saving. Every preset,
including Low, retains the normal emission strength of 12000 and enables bloom.
Disabling bloom can also change the apparent colors of bright armor and target
lights; the settings panel warns when it is disabled.

VSync is on by default; depth prepass and occlusion culling are opt-in on every
preset. All options remain customizable. Shadow cascade choices were measured on
an NVIDIA laptop GPU; Mac performance still needs separate measurements.

## Controls

| Input | Action |
|---|---|
| Left click | Operate visible controls; click the field to capture the mouse |
| Right mouse (hold) | Auto Aim + Auto Fire; separately rebindable actions in Controls |
| G | Switch the auto-aim target between armor and rune, when Controls sets the auto-aim target to Manual |
| Mouse | Look |
| Left button (held, once captured) | Fire from the barrel (driving) or just ahead of and below the eye (flying) |
| W A S D | Drive forward/back and strafe left/right relative to the aim; the chassis heading follows the aim (driving), fly forward/back and sideways in the body frame (Drone) or move on the horizontal plane (free camera) |
| Q / E | Rotate the Drone's body left/right; the mouse aims its gimbal separately |
| Left Ctrl | Fast: 5 m/s drive command, 8 m/s flight |
| R | Toggle chassis spin (6 rad/s) while driving |
| T | Toggle mechanism: the active robot’s supported fold, stair linkage or tunnel stow |
| J | Open automatic L1–L3 assembly controls for the source PIE Heavy (manual parking) |
| V | Toggle first- and third-person view while driving |
| C | Cycle physics colliders: Off / Overlay / Only |
| F3 | Open / close debug panel: collider Off / Overlay / Only, visual wireframe, buff/RFID zones and labels, rendering statistics, remote motion buffering, reset own robot to spawn |
| Space / Left Shift | Move up / down (flying); Space jumps once per grounded press on balance Infantry |
| F6 | Pause or resume the world clock (local world or referee) |
| F7 | Step the world one manual step (three 128 Hz ticks, 23.4375 ms) while paused (local world or referee) |
| O / I | Buy one 17 mm / 42 mm round using team gold, at default prices of 1 / 10 gold during a running match |
| F5 | Start the match (or reset a finished one) (local world or referee) |
| F | Activate your team’s available Rune opportunity (Infantry/Sentry pilot or referee) |
| Tab (hold) | Show team robot status |
| P | Toggle settings; 1 toggles reticle, 2 toggles minimap, - / = adjusts mouse sensitivity |
| M | Toggle the team map; click for marker/Sentry/team choices. A / B / I pick attack / defend / alert |
| Left Alt (hold) | Free the cursor, for example to click a marker on the small map |
| F12 (hold) | Show controls |
| Toolbar | Mouse-driven Settings, Map, Team, Help, Close and Leave match; appears with the cursor released |
| Escape | Close the current panel or open Pause; in staging, go back a step or leave; in Multiplayer, return to the main menu; otherwise open or cancel quit confirmation |

On a bound RMUC 2027 match, O buys a zone unit of ten 17 mm rounds, or a remote
unit of 100 while out of combat (six-second delivery). Release the mouse to use
the combat toolbar's ammunition, remote HP and paid-respawn controls. Gold and
purchase costs appear with HP; heat, XP, level, weakness, invincibility and
respawn progress reflect native rules. A heat video cut covers only the
first-person scene, preserving HUD and minimap. See [the lifecycle guide](rmuc2027-field.md#combat-lifecycle-and-adaptive-armor-infrastructure).

`--hero-assets PATH` supplies an additional verified folding Hero CAD package for
2027 Heavy conversion, alongside `--robot-assets PATH`. When present beside the
primary package, `bound-hero-armored/` is discovered automatically. The J panel
shows **Convert to Hero**; the host requires launcher permission, a stopped live
Heavy and finished assembly recovery. This keeps the Heavy slot and swaps its
model, physics and launcher. See [the approximation policy](assembly-workbench.md#temporary-hero-conversion-for-heavy).

## Map interaction controller interface

Source: the supplied *RoboMaster 2026 University Series Communication Protocol
V2.0.0 (2026-06-30)*, §1.3.1/Table 1-35 (`0x0303`), §§2.2.17–2.2.18
(`MapClickInfo`/`MapClickCmd`), and Appendix II robot/client identities.

`Command::MapInteraction(Interaction)` carries a logical recipient, attack/defend/
alert/custom marker, a position **or** opponent chassis id, and an optional key
byte. Recipient choices are Sentry, one chassis, up to seven selected chassis ids,
or team broadcast including/excluding Sentry. Empty/duplicate selected lists,
non-finite/out-of-field positions, opposing recipients and non-opponent targets
are rejected. The custom marker subset follows MapClickCmd (C–L, N, O, Q–Z,
with A/B/I represented by their predefined kinds); general key bytes are data,
not commands that the simulator interprets. Opponent-id targeting is available
through the typed API/console; this first map card selects locations.

The host authenticates sender/team and creates a monotonic event id, simulation
acceptance timestamp, source category and resolved recipients. A logical Sentry
intent is accepted even before a Sentry deploys; an empty receiver list means
recorded intent, not execution or guaranteed replay to a later deployment.
The current simulator has no separate Gimbal Operator seat: other team pilots
may use its team-command path (simulator policy, 500 ms minimum interval).
A Sentry pilot uses the corresponding-operator path, may address only itself,
and has a 3 s interval. Both are enforced on the injected **host transport clock**,
so pausing the world does not freeze the communication cooldown.

A future controller hosted beside the simulation can read
`HostHandle::map_interactions()`; a client/controller can drain
`Client::take_map_interactions()`. These are bounded to the newest 64 events;
consumers must track event ids, tolerate duplicates, and decide when an intent
is stale. The app keeps a bounded received history exposed as `map_interactions`
in console state; `pending_map` exposes an unsent choice. Controllers still send
ordinary owned chassis commands through the host. The inbox itself does not execute actions. The optional navigation prototype
below consumes location intents; Sentry combat modes remain unimplemented.

This implements semantic simulator messages, not UART frames or MQTT topics.
Coordinates stay in world FLU metres, and chassis ids stay simulator ids.
Legacy source/opponent wire ids are supplied only when the 2026 mapping is known;
Sentry client ids absent from Appendix II and 2027 mappings remain unspecified.
A future hardware bridge must perform explicit referee-frame/identity conversion.
The protocol's initial five serial copies at 100 ms intervals and subsequent
1 Hz refresh are not generated over the simulator's reliable transport; event
identity separates an intent from transport repeats. The host journal and its
rate-limit state last for the host session, not a world checkpoint.

## Console-spawned navigation Sentry

Start the usual bound 2027 map with `--console --start-paused`. Submit these
JSON console commands (through `scripts/console.py`, or `ConsoleClient`):

```json
{"type":"world","command":{"SpawnSentry":{"team":"Red","position_m":[4,-3,1],"yaw_deg":0}}}
```

The ordinary terrain lookup resolves spawn height. Spawn rejects occupied or
obstructed positions and existing team Sentries. It creates a host-owned Sentry
under the normal Sentry rule/HP record, with the current omni navigation profile.
It is a training bot for lifecycle management, without a peer or fake pilot.
`navigation` in console state reports its id, bake summary, phase, target,
waypoints and message; source rules identify it as Sentry 7. Operators may use
`PlaceBot`, `RemoveBot` and `ClearBots`; placement stops its current route.

After spawning, **M → click → Send to Sentry** sends a destination. The exact
location is retained; blocked targets are rejected by the planner rather than
silently moved across a wall. Optional marker/key data stays uninterpreted by
this movement-only controller. Its route appears as a green ground line and map
waypoints; blocked routes turn orange. Phase changes produce HUD notices.
The following operator-only commands permit repeatable console tests:

```json
{"type":"world","command":{"NavigateSentry":{"chassis":1,"position_m":[10,0]}}}
{"type":"world","command":{"StopSentry":{"chassis":1}}}
```

Use the actual id returned in `navigation`, rather than assuming 1. A remote
ordinary pilot cannot spawn, directly steer or remove these bots; it may submit
own-team map intents. A manually piloted Sentry is never taken over. With pause
active, planning can finish without movement; `step` advances its ordinary wheel
physics, or unpause to drive in real time. Example verified on the loaded 2027
map: `[4,-3] → [10,0]` completes through three smoothed path points. This is a
fixture location, not a route hard-coded into the planner.

The first spawn starts one background bake shared by the host's Sentries. It
uses the loaded, verified CAD collision geometry and indexed ground, sampled on
a 25 cm XY grid with height candidates through 1.45 m. Unsupported footprints,
missing support, abrupt surface changes, steep slopes and obstructed headroom
are excluded. Edges sweep a vertical cylinder against mesh BVHs; A* uses travel
and elevation costs, followed by support- and collision-checked shortcuts.
The navigation profile assumes a 37 cm radius and adds 12 cm for tracking during
planning. Its body clearance band is 16–64 cm above support, with a conservative 12°
slope limit and 4 mm seam tolerance. Ground height is sampled every 3 cm along
nine tracks across the inflated footprint, including diagonal wheel locations.
The 2027 Road card envelope, enlarged by 15 cm for diagram uncertainty and then
by the robot planning radius, is explicitly unavailable: this omni profile does
not attempt the bumpy road. This approximate navigation mask is not a rule zone.
These are simulator settings, not rulebook constants or a full swept source-CAD
robot model.

With the EGA package supplied through `--robot-assets`, Sentry deployment offers
**EGA tunnel Sentry** (`--chassis tunnel-omni`), alongside ordinary **Omni**.
`Auto` and console-spawned Sentries use EGA when that package is loaded; explicit
`omni` keeps the standard model. A missing EGA package causes a clear host rejection.
Manual EGA pilots still use T; only host-owned navigation bots fold automatically.

The bake also retains low-clearance edges, distinct from ordinary driving.
It samples additional negative height bands for the crowned floor and uses only
upward-facing CAD support faces, preventing slab bottoms and roof undersides
from becoming floors. These are geometric navigation links, not RFID traversal
identities: diagram locations alone cannot authorize a passage.
Only the fitted EGA controller may use them. Its stowed sweep is an oriented
700 × 700 mm upper-body box, from 140 to 248 mm above support; this approximates the loaded
source envelope with margin. The box follows the sampled support-plane tilt, so
the crowned floor does not consume headroom in a roof with the same slope.
Ordinary navigation retains its larger inflated
cylinder. Support and live obstacles apply to both kinds of edge. Tunnel actions admit
approaches up to 16° (the manual's §4.4 diagram includes a 15° ramp) and inflate
the Road exclusion by the oriented box extent, instead of the large ordinary
circle. The lower body still uses source collision physics and the nine support
tracks reject curbs and support discontinuities; this is a navigation approximation.
A* charges for folding/slow travel and path shortcuts preserve the low-clearance
action. Six off-grid centreline fits use 50 mm lateral intervals: the 250 mm
grid alone can miss a narrow opening. The Road seeds come from the diagram;
the adjacent near-/far-highland lanes use CAD survey hints at FLU Y ≈ 5.93 and
6.93 m, X ≈ ±1.45 m, plus their point mirrors (UC §4.4, Figures 4-4 and 4-35–38).
The old inferred Highland RFID pad pairs run diagonally and are unsuitable as
navigation axes. These hints never authorize a passage: every fit checks the
source support profile, headroom and connections to full-height staging nodes.
The connectors also use checked stowed edges, allowing a narrow approach
without requiring the ordinary 980 mm footprint inside an 800 mm lane.
`tunnel_links` counts low edges; `passages` counts these accepted off-grid fits.
The source-axis body box also identifies chassis forward independently of its
resting barrel, so EGA's X-wheel stance approaches the ramp correctly.
Snapshots expose `tunnel_segments` aligned with `path_m` and the bake's
low-edge and passage counts; world overlays highlight selected tunnel segments blue.

At the safe approach the controller folds to its real actuator endpoint,
levels the firing pitch and aligns the chassis to the longest segment in the
contiguous stowed section. Bounded, collision-checked position corrections hold
that staging point against drift while preparing; forward travel waits for the
fold and alignment to finish. It holds that heading through short lateral exit
connectors, using omni strafing instead of trying to turn inside the lane.
It uses a nominal 0.30 m/s crossing speed with bounded lateral centering,
checking clearance before movement and small yaw changes, and unfolds after
its whole footprint clears the roof. Preparation has a ten-second timeout.
Stop, defeat and replanning retain the current fold target; they cannot request
an unsafe opening. Goals inside low-clearance interiors are currently rejected;
choose a supported exit-side destination. Search is conservative and does not yet
carry orientation as an A* state, so some physically possible curved passages
may be rejected or stop for recovery.

Current geometry and other robot footprints are checked during search and ahead
of motion. The follower requests at most 0.65 m/s through ordinary chassis
commands at 16 Hz, inside the explicit 128 Hz simulation loop. Defeat stops the
controller. Obstruction or a three-second stall brakes the wheels and triggers a worker
replan from the actual position, up to three recovery attempts per goal. A stall
adds an inflated avoidance region 75 cm ahead of the failed approach. Each
Sentry remembers at most 64 such regions for its lifetime (including across new
goals); removing the bot clears them. These are conservative learned exclusions,
not proof that the underlying terrain is impossible. Transient robot blockages
are replanned from current positions without learning a permanent region. If no
safe route exists or recovery is exhausted, it stays stopped; a new goal resets
the recovery budget. No teleport or prescribed chassis movement is used.
Low bumpers, curbs, wheel slip and complex terrain may still invalidate an
approximate route. Stair mechanisms, target pursuit, aiming, firing,
strategic decisions and coordinated traffic recovery are not included.

Arena regression checks now complete `[4,-3] → [12,3]`, `[4,-3] → [10,3]`
and the mirrored `[-4,3] → [-12,-3]`, in addition to the open route and its
reverse. The curb destination `[10,-3]`, cross-arena `[10,0] → [-10,0]`
and sampled perimeter crossings are rejected by this conservative profile;
full-arena connectivity is not yet supported.

The headless `sentry_navigation` example exercises the same host controller:

```sh
nix develop -c cargo run -p rm-simulator-server --example sentry_navigation --locked -- \
  fixture local-assets/robot-cad-survey/bound-tunnel-sentry/chassis.json -1 0 3 0
nix develop -c cargo run -p rm-simulator-server --example sentry_navigation --locked -- \
  field-2027 local-assets/robot-cad-survey/bound-tunnel-sentry/chassis.json 4.7 3.5 4.5 6.8
# Adjacent far-highland and ramp-side near-highland passages:
nix develop -c cargo run -p rm-simulator-server --example sentry_navigation --locked -- \
  field-2027 local-assets/robot-cad-survey/bound-tunnel-sentry/chassis.json 2.5 6.93 -2.5 6.93
nix develop -c cargo run -p rm-simulator-server --example sentry_navigation --locked -- \
  field-2027 local-assets/robot-cad-survey/bound-tunnel-sentry/chassis.json 3.5 5.7 -3.5 5.7
```

The fixture forces a 250 mm passage. The loaded-map cases exercise Road,
far-highland and near-highland tunnels. Reverse the endpoints or negate all four
coordinates for the opposite direction or team-side mirror. The current bake
admits all six explicit passages; this is a geometric fit, not a guarantee of
successful driving from arbitrary poses or a surveyed RFID traversal binding.

A continuous three-type tour is available through the live app console:

```sh
# Uses the built app, one freshly spawned Sentry and ordinary host goals:
python3 scripts/render-sentry-navigation-video.py --check-only
# Rendering additionally needs Pillow and FFmpeg (the map-tools venv has Pillow):
../rm-map-tools/ocpenv/bin/python scripts/render-sentry-navigation-video.py \
  --out local-assets/sentry-three-tunnel-tour.mp4
```

The runner owns and closes its app process, uses an ephemeral loopback console,
checks Road/near-/far-highland crossings from actual folded chassis poses under
source roofs and opposite-side exits, and rejects repositioning or a failed leg.
It spawns once and never injects poses or joint playback. The chosen tour starts
at the Road entrance, crosses the far-highland lane, then lets the planner return
through that lane to approach the near-highland lane from the road deck. This is
four physical crossings across three tunnel types; the backtrack avoids a slope
outside the configured vehicle limit. Slope admission uses the source support
face's full gradient, so diagonal A* edges cannot disguise an excessive slope.

A JSON audit beside the video records the initial CAD bake, every command-to-path
wall latency, paths/replans, arrivals and sampled poses. Warm query latency
includes console transport/polling and is an upper bound on search time. The
default query deadline is five seconds, separate from the initial bake/model
readiness deadline. Video uses real in-game scenery/path overlays and 128 Hz
frame-stepped physics at 1× playback, including observed planning delays. The
headless check still starts the real app but skips screenshots and encoding.

The bake is cached in memory for the current loaded scene; no generated field
asset is committed or written. Map processing/search run on one bounded worker,
with latest-goal generations discarding stale results. Pose, command and path
presentation travel through the existing multiplayer codec (protocol 76).
Controller state is host-side external input, like a remote pilot; `Field::restore`
continues to restore the complete physical/rule world, while a new host must
recreate the controller and bake. There is no portable on-disk bake yet.
