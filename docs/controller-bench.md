<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# CPU controller bench

The first generic consumer is a headless XY positioning robot. It runs on one
CPU thread, with no CAD, renderer, match, transport or background worker. It uses
`PhysicalRuntime`, the same concrete Rapier scene composed by RoboMaster's
`WorldPhysics`. No new dependency or solver interface is required.

The resource baseline is the development Mac, CPU only, with one robot. Measure
the built executable directly; Cargo compilation is a separate cost. This small
fixture establishes a reproducible starting point, not the cost of a full arena,
sensors, imported robots or the interactive game.

## Run

```sh
cargo build -p rm-sim-physics --release --example controller_bench --locked
```

In the repository's Nix development shell, run:

```sh
/usr/bin/time -l target/nix/release/examples/controller_bench 1000 10000
/usr/bin/time -l target/nix/release/examples/controller_bench 128 1280
```

Outside that shell, use your Cargo target path (normally `target/release`).
The example accepts a rate of 128 or 1000 Hz and an optional positive step count;
the default is ten simulated seconds. It prints setup/reset/loop wall time,
mean microseconds per physical tick, simulation-to-wall-time ratio, final target
and position error. The loop includes feedback reads, force computation, command
application and physical integration. A 128 Hz tick has multiple native slices;
compare equal simulated durations as well as per-tick costs.

`time -l` on macOS reports whole-process elapsed time and maximum resident bytes.
It includes startup, preparation, 128 warmup ticks, one reset and output. The
internal setup timer includes cooking and installation; the loop timer excludes
setup, warmup, reset and printing. Repeat the executable after compilation has
finished and report ranges. There is no sleep or real-time pacing.

## Model and control

The fixture has a fixed support, a 2 kg X carriage and a 1 kg Y slide, connected
by prismatic joints. Both stages move within ±0.5 m. Box dimensions and
uniform-solid inertias are explicit illustrative choices. Gravity loads the
constraints. The X joint moves both stages, so its moving mass is 3 kg.

A caller-side PD controller reads position/velocity truth before every tick and
applies a held joint force. Gains are 120 N/m and 30 N s/m, capped at ±20 N.
Targets alternate between (0.25, −0.20) m and (−0.15, 0.30) m every two seconds
of simulation time. This is perfect physical feedback, without encoder errors,
electrical dynamics or hardware validation. The controller is ordinary Rust code
in the example, rather than another scheduler or engine component interface.

## Session lifecycle

`runtime::session::PhysicalSession::new(recipe)` owns one bare runtime and a
typed recipe returning its initial scene. The recipe captures shared prepared
bodies; reset reinstalls them without recooking. A recipe can select a native
solver budget through the existing runtime constructor.

Use `runtime()` for compact truth and `runtime_mut()` for ordinary commands.
Call `step(1)` between feedback updates. The session owns exact completed ticks
and nanoseconds, using the same `engine::time::Stamp` exposed by the existing
robot and match sessions. Overflow is rejected before advancing any requested
steps. The session adds no observation history or per-tick command buffer.
Native solver allocations remain possible; no allocation-free claim is made.

`reset()` constructs a replacement before swapping live state. Setup failure or
a changed integration duration preserves the live world and clock. Successful
reset starts time at zero, clears held commands by reconstruction and invalidates
old body/joint bindings. Reacquire bindings by logical keys and separately reset
controller, task and sensor histories. Recipes must return an unstepped scene
from repeatable inputs. External recipe side effects cannot be rolled back.

This is initial-scenario reset. It does not add arbitrary solver checkpoints or
host admission for generic robots. Existing RoboMaster sessions keep their
participant APIs and rule/physics ordering.

## Validation and checkpoint boundary

The integration tests exercise both 128 and 1000 Hz. They check two-target
tracking within 1 mm after each two-second phase, capped effort and exact repeated
episode results on the same build. Session tests compare direct runtime stepping,
uneven batch partitioning, overflow rejection, stale bindings and failed reset
with preserved live commands. These are software regressions, not calibration.

This checkpoint ends at construct, command, step, observe and reset for one robot
unrelated to RoboMaster. The existing game uses the same runtime body construction
and integration primitives. Migrating game gimbals/arms from prescribed motion to
dynamic joints is a separate model change. Choose the next concrete consumer
before extending this lifecycle, adding a backend or changing package boundaries.
