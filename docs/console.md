<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# App automation console

The app automation console drives one local `rm-simulator` process over a
loopback TCP endpoint: controls, clock, camera, inspection options and
screenshot paths. It is separate from the gameplay transport. The typed
`ConsoleCommand` handler runs on the app thread independently of socket
parsing, so a future in-game console can translate text into these commands and
use the same completion handling. For the interactive flags and bindings the
console stands in for, see [app options](app-options.md).

Start the app with a local console. Window mode is chosen at launch.

~~~sh
just run --console --window-mode headless
~~~

## Window modes

| Mode | Behavior |
|---|---|
| `normal` | Visible window with normal focus behavior; the default |
| `unfocused` | Visible window that requests no focus on creation |
| `headless` | No OS window or window event loop; the gameplay camera renders to an image |

All modes render and process commands while unfocused. Headless rendering still
requires a working GPU backend. The default headless image is 1280 × 720. Use
`{"type":"resize","width":960,"height":540}` to resize the native window or
headless render target for layout checks. Sizes are bounded to 640–3840 ×
360–2160 logical pixels. Resize works before joining; screenshot and UI key,
cursor and mouse-button commands also work on the title screen. `ready` still
waits for a running match.
Whether a visible window can start unfocused depends on the operating system and
window manager. `--window-mode unfocused` sets the native window's initial focus
request; it does not keep forcing focus away later.

## Endpoint

`--console` defaults to `127.0.0.1:7790`. Use `--console 127.0.0.1:0` for an
allocated port; the app prints the actual address. Only loopback addresses are
accepted. This is an app automation endpoint, separate from the gameplay UDP
transport. It has access to the local app's controls and screenshot filesystem paths.
One controller connects at a time; additional connections are closed.

Hold a single connection open for multi-command input sequences. Disconnecting
releases console-held keys and mouse buttons and releases mouse capture.
Discrete commands already sent to the simulation are not rolled back.

Existing launch flags such as `--spawn`, `--third-person`, `--start-paused`,
`--collision-view`, and `--screenshot` remain available as shortcuts. The
screenshot flag still captures once and exits; the console command supports
repeated captures without exiting. Both use the window in visible modes and
the camera image in headless mode.

## Send commands

The bundled Python client waits for the scene to be ready, runs commands in
order, prints each reply, and stops with a nonzero exit status on errors.

~~~sh
python3 scripts/console.py \
  '{"type":"pause","paused":true}' \
  '{"type":"spawn","position_m":[9,0,2],"yaw_deg":180}' \
  '{"type":"camera","pitch_deg":10}' \
  '{"type":"key","key":"w","pressed":true}' \
  '{"type":"step","ticks":100}' \
  '{"type":"release_inputs"}' \
  '{"type":"screenshot","path":"/tmp/field.png"}'
~~~

The app stays open. Send `{"type":"quit"}` to close it. The client also reads
command objects, one per line, from stdin. Use `--host` and `--port` to select
another local endpoint and `--no-wait` for queries or shutdown during loading.

## Wire format and completion

Send one JSON request per line:

~~~json
{"id":1,"command":{"type":"step","ticks":16}}
~~~

Replies echo the unsigned integer ID:

~~~json
{"id":1,"ok":true,"result":{"tick":16,"paused":true}}
{"id":2,"ok":false,"error":"pause the simulation before stepping"}
~~~

Malformed requests get an error; unparsable IDs are returned as `null`. Unknown
fields and commands are errors.

Requests are executed sequentially, including pipelined requests. There is at
most one command in progress. The input buffer is limited to 64 KiB, including
pipelined requests. Exceeding that limit closes the connection and releases held
inputs. A command waiting on readiness, a snapshot or a screenshot times out
after 120 seconds. Socket reads and writes are nonblocking and bounded per frame.

Successful input and world-command replies wait for the app to process the
input and receive the server's resulting snapshot. Host rejections are errors,
not successful acknowledgements. This preserves existing pilot/referee
permissions. A screenshot succeeds only after the scene, pipelines, requested
collision geometry and command snapshots are ready, and the PNG has been saved.
Blank frames are retried. Save errors are returned without closing the app.
A timeout does not cancel a simulation command already submitted.

## Commands

Positions are world forward/left/up metres. Yaw is counter-clockwise from +x;
positive pitch looks up. Camera pitch follows the existing gameplay limits.

| Type | Fields | Effect |
|---|---|---|
| `help` | None | State and supported command names; available during loading |
| `ready` | None | Wait for loaded scenery, compiled pipelines and confirmed commands |
| `geometry` | None | Loaded `Mesh3d` entity counts, unique mesh handles/triangles and instanced triangles, split into CAD and other scene meshes. Includes hidden entities; excludes culling and render-pass multipliers. Available during loading; wait for `ready` for complete counts |
| `state` | None | Current readiness, tick, estimated `presentation_time_ns`, pause, role, own/predicted chassis, camera, UI, connection toast and network diagnostics |
| `camera` | Optional `position_m: [x,y,z]`, `yaw_deg`, `pitch_deg`, `third_person` | Set free-camera position or pilot aim; third person requires a pilot |
| `spawn` | `position_m: [x,y,z]`, optional `yaw_deg` default 0 | Place the existing local pilot using the ground below z, or move a free camera. Resets pitch, body velocity and wheel contacts; preserves ID, HP and defeat state. Remote pilot placement is rejected |
| `pause` | `paused: bool` | Set the clock's paused state |
| `step` | `ticks: 1..60000` | Advance a paused world by this many physics ticks (7.8125 ms per tick at the fixed 128 Hz rate) |
| `screenshot` | `path: "file.png"` | Save the current rendered view and HUD; relative paths use the app's working directory; parent directory must exist |
| `record_video` | `path: "new.mp4"`, optional `fps: 24/30/60` (30) | Continuously capture the view/HUD; requires FFmpeg with libx264; new files only |
| `stop_video` | None | Drain GPU readbacks and finalize MP4 asynchronously; returns frame/drop counters and source timestamps |
| `key` | `key: string`, `pressed: bool` | Hold or release a game key |
| `mouse_button` | `button: "left"`, `"right"` or `"middle"`, `pressed: bool` | Press or release a button in gameplay and UI picking |
| `mouse_motion` | `dx`, `dy` | Relative mouse motion in pixels; aiming requires capture and no blocking panel |
| `cursor` | `x`, `y` | Set pointer position in logical pixels from the upper-left corner, for UI picking |
| `capture` | `captured: bool` | Set gameplay mouse capture for automation without grabbing the OS cursor; explicit capture bypasses OS focus blocking until release or disconnect |
| `release_inputs` | None | Release all console-held keys/buttons and gameplay capture |
| `inspect` | Optional `debug_panel`, `wireframe`, `render_stats`, `zones`, `rfid_zones`, `zone_labels` booleans, `collision_view: "hidden"`, `"overlay"` or `"alone"` | Set runtime inspection options |
| `world` | `command`: multiplayer protocol `Command` object | Submit a world command through the existing session, with its normal host permissions |
| `quit` | None | Reply, then exit |

Continuous video bypasses screenshot warmup, PNG compression and per-frame console
round trips. Six GPU readbacks and three encoder inputs bound pending work; a slow
encoder drops frames rather than blocking the app. Wall-time slots preserve
playback speed by repeating the last frame across gaps. The stop report separates
`unique_frames`, `duplicated_frames`, `skipped_capture_slots`, `queue_drops` and
`readback_limit_skips`; `unique_frames / elapsed_s` is the actual capture rate.
`fps` is the container rate, not a claim of that many distinct captures.
`samples` holds `[output_slot, presentation_time_ns]` for accepted source frames,
so diagnostics can follow the renderer's presentation time, including held frames.
Recordings stop automatically at ten minutes or when leaving the match. Keep the
resolution fixed while recording and call `stop_video` before quitting. Raw
capture includes the HUD; annotation/composition can run offline. Readback pixel
buffers transfer directly to the encoder worker; channel conversion runs there,
and FFmpeg is limited to two encoding threads to reduce contention with rendering.

`state.performance` exposes the existing Bevy diagnostic averages (including `fps`
and `frame_time` in milliseconds; render pass measurements appear when available).
Compare settled recording-on/off windows in the same scene: capture rate alone
cannot tell whether rendering, readback or unrelated machine load limited a run.

A 2026-10-22 check alternated two recording periods with three idle periods in a
fixed 1280 × 720 headless RMUC scene (no ROS/Nav2 workers). Excluding the initial
settling period, diagnostic mean frame times were 20.85 ms idle / 21.29 ms recording
before the buffer change, and 26.67 / 27.06 ms afterward. The machine's CPU load
varied substantially; these runs establish neither a speedup nor a regression.
Distinct capture rates were 29.2–29.5 fps before and 24.4–26.0 fps afterward, with
zero encoder-queue or readback-limit drops. The similar small recording-on/off
separation points to background contention as a contributor, rather than a fixed
22.5 fps capture ceiling. This isolated scene does not measure live Nav2 overhead.

## Keys

Keys include case-insensitive letters `a`–`z`, digits `0`–`9`, `f1`–`f12`,
`space`, `escape`, `enter`, `tab`, `backspace`, `arrow_up`, `arrow_down`,
`arrow_left`, `arrow_right`, `shift`, `control`, `alt`, `shift_left`,
`shift_right`, `control_left`, `control_right`, and `alt_left`.
These are physical game controls, not text entry. Panels block automation input
the same way they block physical input. For example, press and release `f3` to
open the debug panel.

## World commands

The `world` command uses the existing externally tagged protocol shape:

~~~json
{"type":"world","command":{"Referee":"StartMatch"}}
{"type":"world","command":{"Pause":{"paused":true}}}
~~~

`PlaceChassis` is reserved for the embedded owner and their own chassis. The
console does not grant remote referee privileges and never creates a robot for
a spectator or referee.

## Reply fields

A reply is `{"id":<id>,"ok":true,"result":{...}}` or
`{"id":<id>,"ok":false,"error":"..."}`. The `state` result carries these
top-level fields:

| Field | Meaning |
|---|---|
| `ready` | Whether loading has completed. A `state` call with no session returns only `{"ready":false}` |
| `tick` | Current world tick |
| `paused` | Clock paused state |
| `presentation_time_ns` | Estimated presentation time |
| `role` | Client role |
| `chassis_id` | Own chassis id |
| `chassis` | Own confirmed chassis state |
| `chassis_all` | All confirmed chassis poses, teams and ids, including training bots |
| `rmuc_2027_config` | Bound native detector and zone geometry, or null |
| `control_diagnostics` | Each native robot's point eligibility/blocking reason, RFID time remaining and effective field benefits |
| `presented_chassis` | Presentation-only chassis state |
| `pending_shots` | Pending launch count |
| `auto_aim` | Selected target/status, execution time, observation age and fire permission |
| `connection_toast` | Connection notice |
| `network` | Network diagnostics, below |
| `shots_fired` | Global accepted launch counter |
| `hits_detected` | Global detected-hit counter |
| `notices` | Pending notices |
| `camera` | `position_m`, `yaw_deg`, `pitch_deg`, `captured` and `third_person` |
| `ui` | `debug_panel`, `settings`, `blocks_input`, `unfocused` and `consumed` |
| `referee_link` | With `--referee-link`: `controlling`, `controls_received`, and the held `keys` and `buttons` from the custom client; otherwise null |
| `commands` | Supported command names |

`ui.unfocused` and `ui.consumed` identify focus and one-frame input blocking.
`network.confirmed_launches` counts this client's accepted launches once, unlike
the global `shots_fired` counter. `network.downstream_queues` reports control,
owner and world queue bytes, oldest age in ms and cumulative bytes sent.
`network.hit_feedback` reports event count, impact tick, receipt age and the last
receipt-to-scene interval in ms. Scene submission does not measure GPU scanout.

The `state` reply includes `network.trace`: cumulative stage/class counters,
trace path, dropped observations, contention and writer errors. Host-produced
snapshot/owner byte counters are under `network.downstream_queues.encoding`.
See [network tracing](network-tracing.md) for recording and accounting details.

## Deterministic tick scripts

For repeatable world motion, pause, press the controls, step an exact number of
ticks, release the controls, then capture. Each tick is 7.8125 ms at the fixed
128 Hz rate. Free-camera keyboard
movement and held firing use app frames and are not deterministic tick scripts;
use explicit camera poses or the `world` command for those cases.

The Python client also accepts `--script FILE`, a JSON array containing ordinary
commands and two client-side directives:

~~~json
[
  {"type":"pause","paused":true},
  {"repeat":5,"steps":[{"type":"step","ticks":128}]},
  {"assert":{"/paused":true,"/rmuc_2027/phase":"Running"}}
]
~~~

`assert` sends a fresh `state` request and compares each JSON pointer against
its expected value. Array indexes use `/0`, `/1`, etc.; `~1` escapes a slash and
`~0` a tilde in an object key. Values compare exactly, without tolerances or
implicit waiting; missing paths fail even when expecting null. Booleans differ
from numbers. Failures identify the expanded step, path, expected and actual
value, stop the script and exit nonzero. Every reply remains JSON on stdout;
failure messages go to stderr. Assertion and repeat directives are interpreted
by the Python client and are not TCP console commands.

`repeat` preserves one connection and expands its `steps` in order. Scripts are
limited to 10,000 expanded steps and eight nested repeat levels. The client
validates their structure before connecting. Ordinary commands still use the
app's normal validation and authority checks. Each assertion queries confirmed
state after the preceding command's existing completion barrier.

For an end-to-end 2027 control smoke test, launch a **local referee** with the
bound package and no pilot (so no RFID contacts affect the scenario):

~~~sh
just run --console --window-mode headless --referee --start-paused --cad-assets field-2027
# In another terminal:
python3 scripts/console.py --script scripts/scenarios/control-2027.json
~~~

The scenario starts a countdown, destroys Blue's outpost, verifies the five-second
transfer and 500 HP rebuild, checks temporary/full scoring, transfers it back,
checks the rotor never restarts, and starts a fresh countdown to verify reset.
It leaves the app paused in that new countdown. It changes match state and
requires local referee authority; do not run it against a match to preserve.
This smoke test uses real host commands, snapshots and console replies. Physical
contact, contest, departure grace and restore are tested without rendering by
`just control-test`; the scenario does not inject RFID observations.

For a full-field **two-robot** scenario, use the same paused referee launch and run:

~~~sh
python3 scripts/control-field-scenarios.py --port 7790
# Optional saved views of each scenario:
python3 scripts/control-field-scenarios.py --screenshots local-assets/control-views
~~~

This runner resets the match and clears training bots, discovers new chassis ids
from confirmed state, and samples separated positions inside the host's actual
zone polygons. It uses `world` **`PlaceBot`** commands (the same ground lookup as
pilot placement), real physics ticks and real RFID observations. Both field sides
are checked for Base contest, two-second departure grace, capture and recovery;
Central Highland checks the opposing-team latch, handover and physical Outpost
transfer. Fortress checks historical-loss defense/cooling/reserve after Base
repair, the three-minute enemy unlock, vulnerability and ten-second armor opening.
Fortress samples favor its flat center so parked chassis do not slide off ramps.
No control flags, progress or RFID expiries are injected. Success clears
the bots and returns to paused Idle; failure leaves the scene available to inspect.
It requires the bound, hydrated 2027 package and a working renderer.

`PlaceBot { chassis, position_m, yaw_deg }` requires referee or embedded-owner
authority and refuses pilot chassis. Spawn/clear/remove retain their existing
authority checks. These training commands do not grant control of another pilot.
The reusable Python `console.ConsoleClient.call()` keeps one connection and
monotonic request ids for state-driven runners. Its result is the confirmed reply
object, so scripts can discover ids and geometry without assuming an initial id.

Enable `inspect` with `zones` and `zone_labels` to see live point captions. They
show the territory's capture/recovery/rollback state, each detected robot's id,
team, RFID time remaining, eligibility or blocking reason, defense/vulnerability,
effective cooling, reserve and enemy-Fortress armor-opening progress. Occupied
control zones take precedence when labels overlap. Console diagnostics include
all eight points, even when a robot has no contact or a label is off-screen.

For a bound 2027 match, `state.rmuc_2027` exposes the native ruleset, phase,
control, per-robot RFID expiries, structure HP/owners, rotor stops and result.
The object is read-only; changes still use authorized `world` commands.

For a bound RMUC 2027 match, `world` accepts pilot `BuyHp { chassis }` and
`InstantRespawn { chassis }` commands through the same owner checks as `BuyAmmo`.
The `state.rmuc_2027` game includes `robots[].combat`, `gold`, `purchased_rounds`,
`level_caps` and `adaptive` observations. Referee `BuyHp { robot }`, `SetGold`,
`SetAllowance` and `InstantRespawn` are also available. Practice recovery cannot
bypass a running 2027 match. The mouse-release combat toolbar sends pilot commands.

Known confirmation limitation: the console fence currently counts any rejection
received in its window. Late `PilotInput` packets from a previous robot life can
therefore make a respawn command report an error even though its authoritative
HP/gold transition succeeded. Inspect `state` before retrying; the host still
rejects those stale inputs and enforces ownership. This pre-existing fence issue
also affects placement/recovery automation and is separate from the native rules.

## Native terrain and Rune scenarios

```sh
just run --referee --start-paused --console --window-mode headless
python3 scripts/terrain-rune-scenarios.py
# Optional isolation:
python3 scripts/terrain-rune-scenarios.py --only terrain
python3 scripts/terrain-rune-scenarios.py --only rune
```

The installed native Rune-bound 2027 package is required. The runner resets the
match, places a training bot through physical Road/Highland/Tunnel cards, checks
benefits/cooldown/interruption/death, then shoots actual projectiles into both
Small faces and the ten-hit Big Rune. It injects no RFID flags or Rune progress.
It leaves paused Idle on success and inspectable paused state on failure.
`state` includes `rmuc_2027_config`, raw `terrain` diagnostics, `armor_hits` and
`projectiles` for reproducing detector and motion failures. Running the full
scenario requires a GPU even in headless window mode; `just control-test` runs
engine/adapter/console tests without a GPU.

## Virtual 2027 assembly

On a native 2027 match, the HTTP referee panel (`--http`) has **Start assembly**,
**Finish assembly**, and **Cancel** buttons with team/difficulty selectors and
per-team deadline, outcome and income. The console exposes the same authorized
referee actions:

```json
{"type":"world","command":{"Referee":{"StartAssembly":{"team":"Red","difficulty":1}}}}
{"type":"world","command":{"Referee":{"FinishAssembly":{"team":"Red"}}}}
{"type":"world","command":{"Referee":{"CancelAssembly":{"team":"Red"}}}}
```

Finish and Cancel are alternatives for one active attempt. Start requires a
running round; the prototype needs no robot or contact. Inspect
`/rmuc_2027/assembly/0` (Red) or `/rmuc_2027/assembly/1` (Blue) for `outcome`,
`difficulty`, `started_ns`, `deadline_ns`, `completions` and `income_per_10s`.
Use `step` while paused to test timeout exactly: 5,760 ticks equal 45 seconds.
The deadline is exclusive; a Finish at that timestamp fails. See
[virtual assembly](gameplay.md#rmuc-2027-virtual-assembly) for policies and scope.

## Adaptive armor selection

The native 2027 HTTP referee panel has team and center selectors under
**Adaptive armor**. Enter millimetres from the detector's bottom-left corner,
then press **Select reinforcement**. This proxy Radar action is referee-only.
One accumulated opportunity and at least ten identified enemy hits are required.
Console equivalent, with integer micrometres:

```json
{"type":"world","command":{"Referee":{"SelectReinforcement":{"team":"Red","center_um":[45000,44000]}}}}
```

`/rmuc_2027/adaptive/selected/0` stores Red's frozen `center_um`,
`area_share_ppb` and `selected_ns`; index 1 is Blue. A null selection means none
chosen. A selected null area means undefined statistics froze default armor.
`opportunities` reports the remaining allowance; `selections` counts spending.
Each `impacts` entry includes historical `region`: `Default`, `Reinforced` or
`Unreinforced`. Selection never changes when subsequent statistics change;
spend another opportunity to update it. See [adaptive armor](gameplay.md#rmuc-2027-adaptive-armor).

## Responsive UI review

Start a fresh paused match with `--console --start-paused --window-mode headless`,
then capture the real GPU-rendered HUD and settings at four window sizes:

~~~sh
python3 scripts/render-ui-review.py --output local-assets/ui-review
~~~

The output directory must be new. `--port` selects another console;
`--sizes 1280x720 960x540` selects sizes. The script checks PNG dimensions and
settings opening/closing, leaves the match paused and returns to 1280 × 720.
Review the images for clipping, contrast and overlap. Headless rendering uses
scale 1; native high-DPI captures have physical rather than logical dimensions.
