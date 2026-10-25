<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Pendulum extension example

`sim-pendulum-example` is an independent consumer of `sim-engine` and
`sim-physics`. It adds a motor model, encoder, controller and episode reset without
editing generic crate source or enabling native backend access. Its default build
has no renderer, CAD, RoboMaster, transport, worker or wall-clock pacing.

Run the CPU-only example from the repository root:

```sh
cargo run -p sim-pendulum-example --release --locked
cargo run -p sim-pendulum-example --release --locked -- --seconds 12 --csv /tmp/pendulum.csv
cargo test -p sim-pendulum-example --no-default-features --locked
```

The default episode lasts eight simulated seconds. CSV output is optional and
includes separate acquisition/completed timestamps, quantized encoder counts,
actuation and explicitly labelled physical truth. Loop timing includes CSV I/O
when enabled. Build first and run the executable directly for resource measurements.

## Model and clocks

A 0.5 kg uniform box rod, 0.6 m long and 0.05 m wide/deep, hangs from a fixed
support at 1.1 m. Its mass and inertia are explicit uniform-solid fixture choices.
The hinge rotates about world +Y within ±1.2 rad. It starts at 0.55 rad from the
downward vertical; gravity creates a real restoring moment. There is no
prescribed pendulum motion or built-in joint servo.

The example owns three rates on one explicit simulation clock:

- At 1000 Hz, read plant speed, compute motor torque, apply held joint effort
  and step the ordinary `PhysicalSession` once.
- At 200 Hz, quantize actual joint angle to 4096 counts/revolution. Hold both
  value and acquisition timestamp between samples.
- At 100 Hz, update voltage feedback using only the latest encoder sample.
  Estimate velocity from successive measured angles with a 30 ms filter.

The target alternates between ±0.35 rad every four simulated seconds. Feedback
gains are 20 V/rad, 4 V s/rad and 24 V/(rad s), with conditional integral growth
during voltage saturation. The controller receives no exact position, velocity,
motor current or runtime reference. `Experiment::truth()` is a separate privileged
evaluation API. The motor's use of actual velocity models back EMF inside the
plant, rather than supplying perfect feedback to the controller.

The illustrative motor uses R=2 Ω, Kt=0.4 N m/A, Ke=0.4 V s/rad, a ±12 V ideal
bridge, a ±3 A ideal current limit and viscous loss of 0.03 N m s/rad:

```text
voltage = clamp(request, -12, 12)
current = clamp((voltage - 0.4 * speed) / 2, -3, 3)
torque  = 0.4 * current - 0.03 * speed
```

This algebraic model omits inductance, bus energy, gearing, backlash, temperature,
communication delay and calibrated hardware properties. The ideal bridge accepts
braking current. Sensor error here is sample/hold and quantization, without a
noise generator or artificial latency queue.

## Reset and ownership

`Experiment` owns a generic physical session and separate motor, sensor and
controller state. One boxed recipe retains the prepared assembly and is called
only during construction/reset. Feedback and physical stepping use typed calls,
with no callback bus or per-tick observation history.

Reset reconstructs the physical scene from shared cooked resources, reacquires
the hinge binding and clears encoder phase/hold, controller phase/previous sample,
velocity filter, integral, held voltage and motor output. Old body/joint bindings
become invalid. This is initial-episode reconstruction, not arbitrary continuation.

The tests check motor bounds and passive braking, sensor hold/quantization and
exact sample counts, both gravity-loaded targets within 0.005 rad after each
four-second phase, reset between sample boundaries with identical replay,
caller partitioning and rejection of overflowing requests before mutation.
These are software checks of this fixture, not hardware validation.

## Viewer

The existing Bevy viewer selects this same `Experiment`, including its model,
motor, measurements, controller and reset. It publishes the actual body poses
and uses the model's visual dimensions. It constructs no second physical world.

```sh
cargo run --manifest-path integrations/scenario-viewer/Cargo.toml --locked -- --pendulum
cargo run --manifest-path integrations/scenario-viewer/Cargo.toml --locked -- --pendulum --headless --frames 2000
cargo run --manifest-path integrations/scenario-viewer/Cargo.toml --locked -- --pendulum --frames 300 --screenshot /tmp/pendulum.png
```

Space pauses, N steps one physical millisecond, and R resets while preserving
pause and visual assets. Finite runs advance four physical ticks per presentation;
2000 presentations therefore run eight simulated seconds. The optional RXUI
binary retains its existing fixture; `--pendulum` selects the default Bevy viewer.

The viewer test compares its physical poses/time against an independent headless
episode and exercises reset. The dependency guard rejects RM/render dependencies
from the example. Isolated validation also compiles it with native extension
access disabled, so workspace feature unification cannot hide an API dependency.
