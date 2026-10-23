<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Free four-wheel chassis experiment

`whole_chassis` is a standalone mechanical prototype in one `PhysicalRuntime`.
It attaches four copies of the [source-derived suspension](suspension-recovery.md)
and [directional wheel fixture](wheel-contact.md#directional-contact-experiment)
to one **free six-DOF chassis body**. There are no guide locks or prescribed
chassis poses/velocities. Inputs set axle motor targets; normal support, motion,
pitch, roll and load transfer arise from the shared native scene.

This is **not the complete imported EGA robot**. The module linkage dimensions
and rocker inertia retain their existing source checks. Robot layout, sprung
mass, COM, inertia, mounting/up orientation and drivetrain remain explicit test
assumptions. No playable chassis, CAD package, gameplay or network behavior is
replaced.

## Construction and ownership

- One assumed 20 kg sprung body represents the rigid frame/upper mass. Its
  collision box is 480 × 360 × 70 mm, initially centered 230 mm above the floor.
  Explicit box principal inertia and caller-selected local COM are approximations.
- Wheel-center XY coordinates are `[±220, ±200]` mm. Tangential rolling angles
  are 135°, 45°, 225° and 315° for FL, FR, RL and RR. This symmetric layout is
  illustrative, not recovered from the complete assembly.
- Each module adds two checked 14.193 g rockers, an assumed 450 g carrier/motor,
  the provisional 44 g damper and a 250 g, 75 mm radius wheel. The modeled total
  is approximately **23.090 kg**, not 20 kg including modules.
- All four recovered mount frames use the preceding bench's 47° installation
  choice, rotated around the chassis. Native relative mount dimensions are
  retained; complete-robot mounting/up transforms remain unverified.
- The 25 dynamic bodies and 32 mechanical joints share one world and clock.
  Directional contacts add up to four two-axis velocity constraints and a single
  fixed, collider-free ground anchor. Linkage colliders remain disabled; tyres
  and the chassis box contact terrain, with self-collisions filtered out.
- `wheel_suspension::Linkage` retains bindings into the caller's runtime.
  `wheel_contact::RollingWheel` installs on its recovered carrier hub. The old
  isolated fixtures use these same installers; complete before/after CSV traces
  match byte for byte. These are reusable example helpers, not new stable import
  APIs or transactional installation contracts.

Commands specify geometric-origin body-frame translation and yaw rate. The
nominal wheel Jacobian maps these to held axle speeds. Each axle retains the
assumed 0.5 N m s/rad gain and 1.2 N m torque cap. There is **no chassis velocity
feedback**, motor electrical model or power budget. Real tracking errors under
load should therefore be reported rather than hidden by setting body velocity.
A zero speed target actively brakes; `coast()` clears motor torque control.
The recovered mounts connect to the upper body through assumed rigid offsets,
not a recovered complete frame/strut assembly.

The native numerical budget remains 32 solver iterations. Standalone timing is
selectable, including 128 and 1000 Hz; no multiplayer timing policy changes.
Fixture reset reconstructs the same recipe, retaining the held drive/coast mode
and invalidating old tokens. It does not promise arbitrary checkpoint continuation.

## Run

```sh
# output, mode, Hz, optional one-track bump height (m), optional COM X offset (m)
mbx run -p rm-sim-physics --example whole_chassis --locked -- \
  /path/to/forward.csv forward 1000
mbx run -p rm-sim-physics --example whole_chassis --locked -- \
  /path/to/sideways.csv sideways 128
mbx run -p rm-sim-physics --example whole_chassis --locked -- \
  /path/to/turn.csv combined 1000
mbx run -p rm-sim-physics --example whole_chassis --locked -- \
  /path/to/bump.csv forward 1000 0.002
mbx run -p rm-sim-physics --example whole_chassis --locked -- \
  /path/to/com.csv settle 1000 0 0.05
```

Modes are `settle`, `forward`, `sideways`, `diagonal`, `spin`, `combined` and
`coast`. Each run settles for two seconds, drives until eight seconds, then
brakes through ten seconds. `coast` releases motors at eight seconds instead.
CSV generation and its wall-clock reporting are example-owned; the runtime
never reads host time. CSV timing includes I/O and is not a solver benchmark.

The optional narrow left track spans Y = 100..300 mm. It rises over 80 mm from
X = 650..730 mm, has an 80 mm plateau, and descends over 80 mm through X = 890 mm.
It excites the left wheels at different times, producing pitch/roll and load
transfer. These are generated test dimensions, not competition terrain.

CSV records actual chassis pose, COM velocities in the chassis frame, four wheel
slip proxies, spring lengths, loop closure and contact diagnostics. Per-wheel
`directional_normal_load_n` is privileged **preceding-slice normal-impulse
magnitude divided by slice duration**, the same budget used by the directional
law. It is not a new load sensor, independently measured ground truth or vertical
force on a slope. `None` means no supported directional patch. Do not infer no
physical contact from `None`. Summing these magnitudes checks weight balance on
flat horizontal support only.

## Validation and scope

The six chassis regressions exercise both 128 and 1000 Hz:

- Four-wheel settling and flat support balance against the approximately 226.51 N
  modeled weight, with no contact-observation drops.
- Forward, sideways, diagonal, yaw and combined commands through axle motors;
  finite poses, bounded attitude and loop errors.
- Active braking versus motor-free coasting and subsequent passive stability.
- A 2 mm asymmetric track: both left wheels lift and pass; chassis height and
  attitude respond naturally.
- A +50 mm upper COM X offset increases front support and changes pitch.
- Exact multi-module reset/replay, fresh bindings and rejected setup/commands.

Representative commands are 0.25 m/s along either body axis, `[0.18, 0.18]` m/s
for diagonal motion, 0.6 rad/s spin, and `[0.15, 0.10, 0.3]` for combined motion.
Both rates produce approximately 0.225–0.227 m/s forward/sideways and
approximately 0.60 rad/s mean spin; instantaneous rates oscillate around the mean. The finite motor gain and roller drag cause tracking
error. In the +50 mm COM case, front/rear load sums are about 134/92 N instead of
the centered approximately 113/113 N. These are model self-consistency results,
not comparisons to hardware.

The previous single-wheel **5 mm / 20 mm-rise transition stall remains unresolved**.
The new track has an 80 mm rise and different whole-chassis loading. Success on
that gentler track does not resolve the earlier contact-model difference or
certify robot passability. Individual roller bodies, rubber compliance, accurate
upper-robot inertia, native whole-robot mounting transforms, real damper/motor
identification and collision geometry of the rest of the robot remain outside
this prototype.

Ignored evidence under `local-assets/whole-chassis-20261024/` retains both-rate
CSV runs, source/trace hashes, regression/check logs and bounded CPU measurements.
There is no new dependency or private CAD required to run this example.

## Bounded CPU measurement

On Apple M5 Max, development profile with optimization and 32 solver iterations,
five alternating trials measured the following medians for `Fixture::step()`
including its fixed-size truth sample, without CSV, rendering, setup or reset:

| Outer rate | CPU time per outer tick | Simulated time / wall time |
|---|---:|---:|
| 128 Hz | 2.137 ms | 3.66× real time |
| 1000 Hz | 0.268 ms | 3.73× real time |

Both rates use native integration slices no longer than 1 ms; the 128 Hz outer
tick contains eight slices. Each trial used two seconds of settling, 128 extra
warmup ticks after a combined-motion command, then 512 timed ticks. Sampling
uses stack arrays; this is not an instrumented allocation count. The trials are
unpinned and do not establish batch/PPO scaling or performance with a renderer,
sensors or complete source collision geometry. The temporary probe source and
raw timings are preserved with the ignored evidence.

Across the recorded default runs, the peak hinge gap is approximately 0.031 mm
at 1000 Hz and 0.0064 mm at 128 Hz. The shifted-COM run reaches approximately
0.033 mm at 1000 Hz. Cached contact separations reach approximately −1.24 mm on
the 5 mm gentle track. Small joint gaps do not certify accurate tyre contact:
these separations retain the native cached-contact semantics described in the
wheel-contact guide. No observed contact buffer overflow occurred.
