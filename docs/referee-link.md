<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Referee data and client link

## Transport-independent Rust data

`rm_simulator_server::referee_protocol` models the supplied **2026 University
Series Communication Protocol V2.0.0 (2026-06-30)**. This communication schema
is independent of the live RMUC 2027 gameplay identity. No 2026 rule engine or
season execution path is added.

The module provides ordinary, documented Rust structs with serde support:

| API | Coverage |
|---|---|
| `serial` | All **35** top-level command IDs from Table 1-5: standard, VTM, non-data and Radar wireless links. |
| `serial::InteractionData` | All **9** Table 1-25 subcontent families: layer deletion, 1/2/5/7 figures, character figures, Sentry and Radar decisions, and application IDs 0x0200–0x02ff. |
| `client` | All **36** §2.2 topics/commands, plus the nested Radar record, with source sections and field-number/type descriptors. |
| `serial::FrameHeader`, `VideoHeader` | Logical UART header and §2 HEVC fragment header fields. |
| `MockData` | Complete, mutable synthetic templates, with structural validation. |
| `projection::own_robot` | Selected native 2027 own-robot telemetry at one snapshot instant. |

Payloads cover both directions, including controller commands. Constructing one
never executes it. The library adds no UART/MQTT/ROS transport, byte decoder,
CRC implementation, Protobuf codec, emission schedule or simulated packet loss.
**Rust memory layout and serde JSON are not the official wire format.** There is
no `repr(packed)`, transmute or unsafe access. Your adapter owns field-by-field
encoding, endianness, framing, visibility, rates, queues and transport.

Serial bitfields retain raw unsigned words, including reserved bits; `bits` and
`with_bits` read/write bounded bit ranges. Figures use separate logical fields
with checks against the documented bit widths. Fixed protocol blocks above 32
elements use length-checked vectors; their defaults have the required lengths.
Other fixed arrays remain arrays. Reserved 4-byte values retain raw bits rather
than inventing a physical measurement or losing NaN payloads through JSON.

Custom-client optional scalars use `Option<T>` so **absent** differs from a
reported zero. Repeated fields use vectors; raw numeric enum/status values permit
unknown codes for compatibility and fault tests. Unknown fields in an incoming
Protobuf stream cannot be preserved by these DTOs alone; that belongs to the
caller's codec. Validation checks bounded bytes, paired arrays and finite
measurements, not command authority or every semantic enum restriction. Empty
repeated fields and zero IDs are unconfigured mock templates, not complete
operational messages.

### Configure synthetic data

```rust
use rm_simulator_server::referee_protocol::{MockData, client, serial};

let mut mock = MockData::default();
for message in &mut mock.serial {
    if let serial::Message::RobotStatus(status) = message {
        status.robot_id = 3;
        status.current_hp = 200;
        status.maximum_hp = 200;
    }
}
for message in &mut mock.client {
    if let client::Message::RobotDynamicStatus(status) = message {
        status.current_health = Some(200);
    }
}
mock.validate()?;
// Your adapter selects messages, encodes them and sends at its chosen schedule.
# Ok::<(), &'static str>(())
```

Every serial message exposes `descriptor()` with command ID, source table,
overview length and detailed-table length. `client::Message::descriptor()` gives
topic, source section and field tags/types. `CATALOG` enumerates each family.
Templates include custom commands and wireless reports; they are deliberately
synthetic and do not assert that every mechanism or sensor is simulated.

### Use native own-robot values

```rust
use rm_simulator_server::{workload, referee_protocol::projection};

let (simulation, ids) = workload::simulation(1); // No-CAD example only.
let snapshot = simulation.snapshot(); // A live adapter can use HostHandle::state().
let telemetry = projection::own_robot(&snapshot, false, ids[0], 3, None)?;
let time_ns = telemetry.time_ns;
for payload in telemetry.into_messages() {
    let topic = payload.descriptor().topic;
    // Your encoder/transport consumes (time_ns, topic, payload).
}
# Ok::<(), &'static str>(())
```

`own_robot` reads one immutable snapshot and never advances time. In a running
game obtain `state = host.state()?` and pass `&state.field` and `state.paused`.
The projection emits stage/clock/pause, own robot connection/HP/level/limits,
heat/allowance/experience/buffer energy/out-of-combat state, and whether respawn
is pending. Whole experience and buffer joules round down. Costs, launch counters,
series scores, unspecified module telemetry and unsupported attributes stay
absent. It does not automatically mix synthetic templates into live reports.
Synthetic overlays are an explicit caller choice.

Supply an official referee ID explicitly, distinct from the simulator chassis
ID. The function checks team and compatibility number: Heavy 1/2, Infantry 3/4/5,
Sentry 7, with +100 for blue. Heavy uses the Engineer type code 2. Opponent Radar
positions, wireless observations, global roster slots and damage events need
explicit visibility/roster/event inputs; the projection does not leak them from
privileged snapshots or synthesize event history from current HP.

Position is omitted unless you supply `projection::PositionFrame`, specifying
the arena origin in protocol metres and rotation about up. The resulting position
is a **chassis-origin truth proxy**, not a noisy/calibrated UWB measurement. Speed
Monitor heading remains absent because true north and its mounting reference
need separate calibration. No historical corner-origin assumption is silently
reused. Simulation time is separate from the serial synchronization UNIX/NTP
field; the adapter must provide any required absolute clock mapping.

### Manual inconsistencies and normalization

The source is the supplied English PDF, filename
`UTF-8__RoboMaster 2026 University Series Communication Protocol V2.0.0（20260630）.pdf`,
SHA-256 `156aa1311d9a5475095ff9c1a07aa0e67813411ec75df24b2970252ad9b89f70`.
Detailed numbered tables take precedence over conflicting overview lengths or
malformed example declarations. No missing bytes are silently padded:

| Location | Model policy |
|---|---|
| 0x0203: Table 1-5 says 16 bytes; Table 1-14 and C example contain 12 | Model x/y/heading (12); retain overview 16 in metadata. |
| 0x0303: overview says 15; Table 1-35 and C example contain 12 | Model the specified 12 bytes; retain overview 15. |
| 0x0307: overview says 103; Table 1-37 includes sender at offset 103 | Model 105 bytes including sender; retain overview 103. |
| §2.2.1 topic KeyboardMouseControl but declaration RemoteControl | Use topic name; retain `RemoteControl` as a Rust alias. |
| §2.2.6 invalid `optional repeated` | Normalize to repeated fields. |
| §2.2.8 declaration omits table field 2 | Include `collision_damage`, tag 2. |
| §2.2.13 declaration omits table field 5 | Include `recipient_id`, tag 5. |
| §2.2.19 malformed repeated scalar Radar declaration | Use repeated nested Radar records at table field 1, opponent then own 1/2/3/4/6/7. |
| Table 2-1 DartSelectTargetStatusSync vs §2.2.32 DartSelectControlStatusSync | Use Table 2-1 topic; retain the detailed name as a Rust alias. Normalize invalid `uint` to table uint32. |
| Invalid identifiers with spaces, duplicate reserved names, HP C typo | Normalize to legal Rust names; preserve field numbers/types and reserved words as raw bits. |
| §2.2.11 field 3 named like a firing rate but table describes velocity | Preserve normalized identifier; document its m/s meaning. |

These policies are explicit compatibility interpretations, not confirmation of
actual firmware serialization. A byte codec should resolve conflicts against
its target hardware revision before asserting wire compatibility. Removed serial
0x0304 is intentionally absent.

Run `cargo test -p rm-simulator-server referee_protocol --locked` for full
catalog inventory, template/JSON round trips, malformed-length/bitfield checks,
optional-vs-zero behavior and native snapshot projection tests. The existing
MQTT app link below remains a separate older-schema implementation; it is not
automatically switched to these full V2.0.0 DTOs.

## Existing MQTT custom-client link

The bridge projects native RMUC 2027 state onto the existing custom-client
Protobuf schema. Heavy maps to its Engineer type; unsupported energy, series and
launch-counter telemetry stays absent, and shield fields are absent because
2027 has no virtual Base shield. This compatibility interface does not run a
2026 referee.


The referee link lets a RoboMaster custom client, such as trident-rm
`custom-client-27`, run against the simulator instead of the referee system.
The app embeds an MQTT broker and publishes the official custom-client state
topics for the robot it pilots, as raw Protobuf, one topic per message name,
drives that robot from the client's `KeyboardMouseControl`, and with
`--referee-video` sends the pilot's view as the client's video stream.

It is opt-in twice: build with the `referee-link` feature (`just run-referee`
does), then pass `--referee-link`.

~~~sh
just run-referee --play --robot hero --referee-link --referee-video
# The custom client connects as it would to the referee system:
./build/rm-client --broker tcp://127.0.0.1:3333 --client-id 1
~~~

`--referee-link` listens on `127.0.0.1:3333`. Pass an address, for example
`--referee-link 0.0.0.0:3333`, to accept a client on another machine; the
broker has no authentication. `--client-id` on the client must be this robot's
referee ID (red 1-7, blue 101-107) so it shows the right side.

## Topics

| Topic | Rate | Content |
|---|---|---|
| `GameStatus` | 5 Hz | Stage from the referee phase, stage clock, round, round wins as scores, pause |
| `GlobalUnitStatus` | 1 Hz | Base, shield and outpost HP of both sides; robot HP by number; damage dealt |
| `RobotStaticStatus` | 1 Hz | Referee ID, type, level, maximum HP, heat limit and cooling, chassis power |
| `RobotDynamicStatus` | 10 Hz | HP, heat, allowance for the robot's caliber, chassis energy, experience, shots, out of combat |
| `RobotPosition` | 1 Hz | Own chassis position |
| `RadarInfoToClient` | 1 Hz | Every numbered robot's position, opponent first |

Fields the simulation does not model stay absent rather than zero: series
length, base and outpost status codes, buffer energy, last launch speed and
yaw. Radar positions go to every client at 1 Hz, which is more than a real
client receives.

Referee IDs come from the roster's robot (Infantry 3, 4 or 5), or from the kind
where only one number fits. Robot numbers with no robot on the field report 0
HP and radar position 0, 0.

## Control

The link subscribes to `KeyboardMouseControl` and plays each message as local
input, the way the [app console](console.md) injects keys: the sixteen protocol
keys (W, S, A, D, Shift, Ctrl, Q, E, R, F, G, Z, X, C, V, B) are held as the
same keyboard keys, the three buttons as the mouse buttons, and mouse motion
aims (protocol y is up). The app's own bindings then apply, so W drives, Ctrl
is fast, R toggles spin and the left button fires unless they were rebound in
the settings. The wheel is ignored.

While messages arrive the link holds mouse capture, so the app aims and fires
even when its window is not focused; an open panel still blocks input. When no
message arrives for 200 ms, or the match ends, every held key and button is
released and capture ends. A client in menu mode sends neutral input, which
keeps capture but moves nothing.

The console's `state` reply shows the link under `referee_link`:
`controlling`, `controls_received` and the held `keys` and `buttons`.

## Video

`--referee-video` sends the pilot's view to `127.0.0.1:3334`, the custom
client's video port, or to the address given. A second camera follows the
gameplay camera without the HUD, including third person if selected, and
renders 1280 x 720 at 30 frames per second while a match runs. Frames are read
back from the GPU and encoded by an `ffmpeg` child process with libx265
(ultrafast, zero latency, 3 Mbit/s, a keyframe every second); `ffmpeg` must be
on `PATH`. The rendering costs a second view of the scene.

Each access unit is one UDP frame in the official format: an 8-byte big-endian
header (frame number, fragment index, access unit size) and at most 1400
payload bytes per packet. Access units are split at the encoder's delimiters,
so a frame leaves once the next one is encoded, adding one frame of latency.
Frames are dropped, not queued, while the encoder is behind.

## Coordinates

The protocol world frame is assumed to have its origin at the red-side corner
with x towards blue, in metres; radar positions are in centimetres. The
simulator puts red on +x with the origin at the field centre, so
`to_protocol_frame` turns positions a half turn about the centre. The custom
client uses the same assumption; it has not been confirmed against the V2.0.0
protocol.
