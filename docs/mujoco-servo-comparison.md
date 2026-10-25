<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Servo response: Rapier and MuJoCo

On 2026-10-24 the standalone rotor was compared with MuJoCo 3.17.0 on
Apple M5 Max. This checks a simple mechanical model and its controller, not a
particular servo's hardware fidelity. It is not a throughput benchmark.

The simulator already uses Rapier's solver. The new code supplies source models,
control laws and adapters, rather than implementing rigid-body dynamics from
scratch. MuJoCo is itself a C/C++ physics engine with a C API, generalized-coordinate
articulated dynamics and its own constraint solvers; its Python package wraps
that engine. There is no underlying third-party solver to adopt instead of
MuJoCo. See the official [overview](https://mujoco.readthedocs.io/en/stable/overview.html)
and [computation description](https://mujoco.readthedocs.io/en/stable/computation/).

## Matched experiment

Both models use a centered, contact-free 5 kg rotor with explicit diagonal
inertia `[0.02; 3]` kg m², an X-axis hinge, standard gravity, zero damping,
zero dry friction and no armature inertia. The actuator is direct output-shaft
torque with unit transmission. Both run the same explicit PD law at every
outer solver slice: `clamp(4*(target-position) - 0.4*velocity, -0.5, 0.5)` N m.
The target changes from +0.3 to −0.2 rad at 1.5 s; an external −0.2 N m load
acts from 1 to 2 s. The reference CSV contains completed-tick states and
last-slice actuator output, not an average torque.

At 128 Hz, both take eight 0.0009765625 s slices per tick. At 1000 Hz, both
take one 0.001 s slice. MuJoCo uses Euler integration, Newton constraint solving,
100 iterations and tolerance 1e-12. Rapier retains its existing generic-runtime
integration and default four solver iterations. These are different algorithms;
iteration counts are not equivalent accuracy budgets. The MuJoCo built-in
position actuator shortcut is deliberately not used, to avoid changing the
controller's implicit/explicit integration semantics.

| Caller rate | Position RMS difference | Maximum position difference | Velocity RMS difference | Actuator torque RMS difference |
|---|---:|---:|---:|---:|
| 128 Hz | 0.000392 rad (0.0225°) | 0.001569 rad (0.0899°) | 0.002999 rad/s | 0.000980 N m |
| 1000 Hz | 0.000401 rad (0.0230°) | 0.001606 rad (0.0920°) | 0.003068 rad/s | 0.000975 N m |

These are errors between two numerical models, not errors against ground truth.
The small differences do not indicate a hardware accuracy ranking. The full
three-second trajectory, including load changes and torque saturation, is used.

Separately, both engines pass the constant-acceleration check
`velocity = (actuator torque + external torque) * time / inertia` with absolute
velocity error below 1e-8 rad/s for external loads 0, −0.1 and −0.2 N m. This
has an analytical reference rather than treating either engine as truth.

A MuJoCo default soft limit under +0.4 N m settles at 0.500570 rad for a
+0.5 rad stop: about 0.000570 rad of compliance. Its actuator output is +0.4 N m
and generalized constraint torque about −0.4 N m. Rapier's existing fixture tests
also verify stop holding, but its stops use a different numerical model and
were not tuned to match this compliance. `qfrc_constraint` is a generalized
constraint torque, not a full six-axis joint-wrench sensor.

## Reproduce

The optional Python script adds no MuJoCo dependency to the Rust workspace.
Its inline script metadata pins MuJoCo 3.17.0. Use an ignored output directory:

```sh
mkdir -p local-assets/mujoco-servo
cargo run -p sim-physics --example servo_response --locked -- \
  local-assets/mujoco-servo/rapier-128.csv 128
cargo run -p sim-physics --example servo_response --locked -- \
  local-assets/mujoco-servo/rapier-1000.csv 1000
uv run --script scripts/compare-servo-mujoco.py \
  --rapier-128 local-assets/mujoco-servo/rapier-128.csv \
  --rapier-1000 local-assets/mujoco-servo/rapier-1000.csv \
  --output local-assets/mujoco-servo
```

The script checks reference row counts, command schedules and timestamps, writes
MuJoCo CSVs and `results.json`, and asserts its analytical velocity checks.
Research artifacts, logs and recorded Python dependency versions are retained in
ignored `local-assets/mujoco-servo-20261024/`; the task-created Python environment
was removed after validation. No temporary examples or files in `/tmp` are needed.

## Backend decision

The current mechanical joint model is suitable for initial torque-controller
experiments. Its missing realism is mainly uncalibrated inertia, friction,
actuator dynamics, gear compliance/backlash and measurement/timing effects.
Changing the solver does not identify those quantities. MuJoCo supplies richer
actuation facilities, including a documented
[DC motor actuator](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-dcmotor),
but those also require measured parameters.

A separate optional in-process MuJoCo runtime is a credible next prototype for
articulated robotics and training. It can consume the same source descriptions,
commands and passive render outputs. Use the native C library through a small
isolated binding layer; Python is only the comparison harness here. Keep one
solver owner per interacting scene; do not simulate a robot in MuJoCo and its
contacting arena in Rapier without a separate, validated coupling design.

Before selecting a backend, compare a gravity-loaded arm, the source closed-loop
suspension and the four-wheel contact fixture (including the unresolved steep
transition). Measure torque response, energy, constraint residuals, support loads,
contact behavior and CPU cost at matched controller cadence. MuJoCo's tree joints
avoid numerical enforcement of tree connectivity; closed loops still require
constraints. Rapier also provides reduced-coordinate multibody joints, whereas
our current generic bindings use impulse joints; that is another potential
increment before a complete backend replacement. See Rapier's
[joint representation guide](https://rapier.rs/docs/user_guides/rust/joint_constraints/).

This run establishes no speed advantage, complete-robot contact accuracy,
checkpoint compatibility or working MuJoCo backend integration. It supports
reusing an established dynamics engine and retaining our importer, sensors,
controllers, gameplay and rendering around that engine.
