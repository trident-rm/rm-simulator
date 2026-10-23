<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Capability overview

Status as of 2026-10-24. The repository contains both a playable RoboMaster
simulator and reusable in-process simulation components. An available API or
standalone experiment does not mean every feature is enabled in multiplayer.
The guides linked below record exact contracts and measured limitations.

## Layers and ownership

- **Engine (`rm-sim-engine`):** solver-free physical descriptions, FLU poses,
  immutable timing, surfaces, primitive/convex/compound shapes, explicit inertia,
  joints, assemblies, mounts and scalar wheel laws.
- **Physical runtime (`rm-sim-physics`):** one Rapier scene for cooked resources,
  body/joint commands, queries, physical contacts and passive sensor acquisition.
  RoboMaster-specific chassis, projectile, payload and target adapters share it.
- **Gameplay and world:** separate deterministic competition authorities, with
  `Field` and `MatchSession` composing rules and physics. `RobotSession` supports
  standalone physical scenarios without match authority.
- **Control:** caller-driven navigation and assembly algorithms, without owning
  a simulation clock, host worker or transport.
- **Presentation and rendering:** borrowed game state becomes passive render
  records; generic GPU visuals can be used without gameplay. The optional RXUI
  viewer supplies panels around the shared GPU viewport.
- **App, server and optional integrations:** interactive play, authoritative
  multiplayer, ROS adapters and developer tools remain outside the engine.

See [architecture](engine-architecture.md), [physical runtime](physical-runtime.md),
[embedding](simulation-embedding.md) and [presentation/render split](presentation-render-split.md).

## Available capabilities

| Area | Implemented | Qualification |
|---|---|---|
| Physical models | Fixed, dynamic and kinematic bodies; primitive, convex, compound and static triangle geometry; explicit mass, COM and inertia; reusable cooked bodies/assemblies | Dynamic triangle surfaces and unsupported compound combinations reject explicitly; cooking is backend-specific |
| Articulations | Physical fixed/revolute/prismatic joints, local anchors, limits, held effort and capped PD motors; independent placed assembly instances | Generic described assemblies are trees; direct installation can close loops; rotary observations use principal angles, not an unwrapped encoder |
| Commands and feedback | Body wrenches; joint position/velocity and last-slice actuator effort; ordinary chassis velocity and direct wheel-torque modes | Actuator effort is not net torque, constraint reaction, electrical current or a tick average |
| Lifecycles | Branded bindings, remove/reinstall checks, prepared resource sharing and recipe resets | Standalone recipe reconstruction does not promise arbitrary solver checkpoint continuation |
| Equipment/scenarios | Standalone equipment presets, composable terrain/robots/targets/RFID and independent motion identities | Scenario metadata does not create a second match authority |
| Sensors | Nearest-hit rays, ideal scans, mounted rolling MID-360 acquisition, dynamic geometry/self-occlusion, configurable errors and synchronized IMU | Laser/material response is approximate; truth observations are privileged; discontinuous capture intervals reject interpolation |
| ROS/navigation hooks | Optional rclrs/Livox and synchronized sensor publishing; leased external velocity/heading commands; built-in CAD-aware routes with tunnel transitions | Optional workspace and setup; no claim that every external nav stack or the new articulated chassis is integrated automatically |
| Planning | Reusable inflated-map routing/following and tunnel posture logic; collision-aware staged Engineer IK/assembly planning | Current assembly task is RoboMaster-specific; game arm motion remains prescribed playback rather than a torque-driven arm |
| Rollouts | Headless stepping, retained buffers, batches, selective resets and bounded CPU parallel stepping | No generic GPU physics backend; throughput depends on world, sensors, integration settings and batching |
| Visual tools | Bevy game rendering, graphics presets, generic visual API, standalone GPU viewer, optional RXUI controls and import review | Optional integrations have separate workspaces; not a complete RViz replacement |
| CAD import | Verified robot packages, STEP-derived visuals and native SolidWorks metadata/mesh preparation; group/joint/mass/collision review with physical previews | Missing/stale native references and cache properties need review; no blanket certification of recovered mates, mass or inertia |
| Game | RMUC 2027; separate RMUL 2027 3v3 and Infantry confrontation; combat, referee control, equipment, assembly and multiplayer | Official RMUL geometry is absent; approximate practice layouts are labelled; remaining rule gaps stay documented |

Detailed guides: [jointed mechanisms](jointed-mechanisms.md),
[scenario composition](composable-scenarios.md), [controllers](controllers.md),
[synchronized sensors](synchronized-sensors.md), [ROS](../integrations/ros2/README.md),
[SolidWorks review](solidworks-importer.md), [gameplay](gameplay.md) and
[season coverage](seasons.md).

## Recent mechanical and servo additions

1. **Recovered suspension experiment.** A source-derived four-bar/damper fixture
   exercises closed-loop joints and provisional spring/damping laws. The selected
   44 g damper is user-approved; stroke and force-law parameters remain assumptions.
2. **Physical wheel contact.** A driven whole-tyre convex collider loads the actual
   suspension. The tyre does not model individual roller bodies.
3. **Directional contact approximation.** Opt-in tread/roller friction on fixed
   terrain, coupled through native constraints, with passive lateral resistance
   and load diagnostics. Default game contact behavior is unchanged.
4. **Free four-wheel plant.** Four suspension/wheel modules share one free
   six-DOF chassis. Tests cover translation, yaw, combined commands, braking/coast,
   COM load transfer, small asymmetric tracks and deterministic reset at 128/1000 Hz.
   Root mass/inertia, mounting arrangement and upper-robot geometry are illustrative.
5. **Torque and servo bench.** Direct joint effort and capped PD control now expose
   the last applied actuator effort. Known-inertia acceleration, opposing loads,
   saturation, stop holding and stale bindings are tested at both rates.
6. **Optional MuJoCo cross-check.** A pinned Python harness runs the same rotor
   and controller in MuJoCo 3.17.0. Position RMS disagreement is about 0.023° and
   peak disagreement below 0.093° for this three-second test. Both engines pass
   the analytical acceleration check. This is not hardware validation or a speed
   comparison, and MuJoCo is not yet an integrated simulator backend.

See [suspension evidence](suspension-recovery.md), [wheel contact](wheel-contact.md),
[whole chassis](whole-chassis.md), [servo controls](jointed-mechanisms.md#torque-commands-and-servo-response)
and [MuJoCo comparison](mujoco-servo-comparison.md) for reproduction.

## Use-case readiness

**Embedded controls:** a standalone mechanical plant can take output-shaft torque
commands and return joint/body truth on an explicit 1000 Hz clock. Hardware-specific
current/voltage loops, torque-speed envelopes, calibrated losses, encoder effects
and MCU timing must be supplied. Current tests are software simulation, not an
MCU hardware-in-the-loop or hard-real-time qualification.

**Navigation:** arena collisions, rolling LiDAR/IMU and external command hooks
already provide the principal simulation interfaces. The new articulated chassis
still needs its own sensor/ROS/host wiring before it can replace the existing
navigation robot model. Ground-truth CAD routing is distinct from localization
or SLAM driven only by simulated measurements.

**Vision:** the passive GPU renderer, CAD/material assets and robot control
interfaces support visual experiments. Rendering a detailed scene does not
establish photometric camera or material calibration.

**RL/training:** headless sessions and independent CPU batches provide stepping
and reset primitives. Task observations, rewards, policies and a training framework
remain caller responsibilities. There is no claim of a complete PPO lab or GPU
physics integration.

**Driving practice:** the interactive app and multiplayer host are available now.
The articulated suspension/servo experiments do not silently replace the existing
faster game chassis or prescribed gimbal/folding/assembly mechanisms.

## Important remaining limits

- The original isolated directional tyre still stalls on a 5 mm rise over a
  20 mm ramp; passing a gentler whole-chassis track does not resolve that case.
- Directional contact uses prior-slice normal budgets and an approximate patch;
  dynamic opposite bodies use native isotropic friction. It is not an individual
  roller/bearing model or calibrated omni tyre.
- Native solver contact caches/warm starts are reconstructed on match restore.
  Generic standalone assemblies are not admitted as arbitrary hosted checkpoint
  content. Keep recipe reset and match continuation distinct.
- MuJoCo or another solver must be evaluated on coupled arms, suspension loops,
  wheel/terrain contact and matched performance before selecting a new default.
  We reuse Rapier today; the project is not implementing its own rigid-body solver.
