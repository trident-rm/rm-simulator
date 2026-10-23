<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Guided wheel contact and obstacle fixture

`wheel_contact` advances the source-derived suspension linkage, a driven rolling
wheel and terrain in one `PhysicalRuntime`. It is the next experiment after the
[closed-loop bench](suspension-recovery.md), not a replacement for playable robot
physics. The previous investigation is committed at `161bed6`.

## Physical setup

The chassis-side frame is now a **dynamic 5 kg mass**. A test-stand guide locks
its lateral translation and all rotation, leaving forward X and vertical Z free.
No chassis pose, height or forward velocity is prescribed after installation.
A 0.45 kg lumped carrier/motor equivalent and separate 0.25 kg wheel replace the
previous bench's moving 5 kg load. The two rockers retain their checked native
mass, COM and principal moments; other body mass distributions remain estimated.

The same seven suspension joints close both loops, and a new eighth revolute
joint attaches the driven wheel to the carrier. All seven bodies share the floor,
obstacle, integration clock and ordinary native contact solver. The 44 g damper,
30 mm provisional software stroke, 3000 N/m spring and 50 N s/m damping remain.
This standalone fixture uses 32 solver iterations; game defaults are unchanged.

The source linkage plane is rotated onto the guide using an explicit 47-degree
bench orientation and a half-turn about X, so the wheel supports a loaded frame
with useful bump travel. This rotates all recovered frames together. It is a
controlled test orientation, not a recovered whole-robot installation/up frame.
The guide supplies lateral and rotational reactions, so this is not a free
quarter-car or a complete vehicle stability test.

### Wheel, drive and ground choices

- Paired native inner plates locate the hub at `[25, 34.55, -25.25]` mm in the
  original carrier bench frame. The wheel is attached there, not at a rocker pivot.
- Display geometry across the 35 physical wheel occurrences has approximately
  74.984 mm outer radius and 38.994 mm axial width. The named swept-clearance
  volume is excluded. The physical envelope uses rounded 75 mm radius and 40 mm
  width, with 64 circumferential segments and explicit uniform-cylinder inertia.
- The single convex envelope is an isotropic rigid rolling proxy. It does not
  simulate separate omni rollers, tyre deformation, lateral omni friction or
  measured rolling resistance. Polygon faceting can cause small contact ripple.
- Tyre and ground use an assumed Coulomb coefficient of 0.9 and zero restitution.
  Linkage-body self/ground contacts remain disabled; only the wheel interacts
  with the floor and obstacle in this experiment.
- The axle receives a held velocity target, starting at 0.25 m/s tread speed after
  two seconds of gravity settling. Its illustrative derivative gain is
  0.5 N m s/rad with a 1.2 N m torque cap. This is not an identified M3508/C620
  electrical, gear-loss or power-budget model. There is no forward frame actuator.
- Ground is a fixed half-space at z=0. The generated obstacle starts at x=0.35 m:
  a 20 mm up-ramp, 40 mm plateau and 20 mm down-ramp, 5 mm high and 300 mm wide.
  These are test dimensions, not official arena terrain.

## Reproduce

```sh
# Optional args: output CSV, bump height (m), friction, tick frequency (Hz).
mbx run -p rm-sim-physics --example wheel_contact --locked -- \
  /path/to/bump.csv 0.005 0.9 1000

mbx run -p rm-sim-physics --example wheel_contact --locked -- \
  /path/to/flat.csv 0 0.9 1000

# Control case: the axle spins but cannot drive the frame through friction.
mbx run -p rm-sim-physics --example wheel_contact --locked -- \
  /path/to/frictionless.csv 0 0 1000
```

Each trace covers eight seconds, including an installed time-zero pose and all
seven body frames. `chassis_support_n` is absent in a pose-only sample, including
the initial CSV row. For completed ticks it is a privileged **estimate** from the
frame's COM momentum balance, `m * (delta_v_z/dt + 9.81)`. It is not a direct
solver contact impulse, simulated load sensor or total tyre normal force. The
other moving masses also contribute to tyre load. Contact records retain their
ordinary cached-geometry semantics; reported signed separation is not a freshly
recomputed exact source-mesh penetration depth.

The observer uses a bounded 64-sample buffer. All measured cases dropped zero
samples; there is no unbounded contact collection or second hidden plant.

## Observed behavior

Local 1000 Hz runs on the pinned backend gave:

| Case | Final frame X | Final forward speed | Late slip RMS |
|---|---:|---:|---:|
| Flat, friction 0.9 | 1.466 m | 0.247 m/s | 0.00025 m/s |
| 5 mm obstacle, friction 0.9 | 1.451 m | 0.247 m/s | 0.00025 m/s |
| Flat, friction zero | 0.00031 m | approximately zero | 0.248 m/s |

Before drive, the inferred frame support averages approximately 49.05 N, matching
the 5 kg sprung test load after settling. The obstacle raises the wheel by about
5.09 mm relative to the flat control; dynamic frame lift peaks at about 6.58 mm.
It passes the obstacle, then returns close to the flat-run height. This validates
contact-coupled behavior for this small generated ramp, not larger competition
terrain or the whole robot's passability.

Peak suspension hinge gap is about 0.031 mm across the gravity drop and passage.
Deepest reported cached separation is approximately −0.69 mm in the default case
and −0.72 mm in the zero-friction control, including initial touchdown from 2 mm
above support. This is finite contact compliance/numerical behavior, not calibrated
tyre elasticity. The larger errors are not hidden with prescribed body motion.

Five regression tests cover both 128/1000 Hz loading, traction versus the
zero-friction control, obstacle contact/lift/recovery, exact reset/replay with
fresh tokens and rejected setup/drive requests. The preceding seven closed-loop
bench regressions remain in place. Full vehicle pitch/roll, omni-roller forces,
real mass distribution, damper calibration and contact-geometry fidelity remain
separate follow-on work before adopting this as game physics.

## CAD-backed inspection

Read-only probes bind the previous 19 suspension render pieces plus all 35
physical native wheel occurrences to recorded body poses. The complete wheel
subtree rotates as rigid visual geometry; individual rollers are not animated as
independent dynamics. Generated terrain accompanies the before/on/after-obstacle
views. Native mesh coordinates are preserved; the coil remains omitted and the
previous review-only fused-shaft split remains explicit. These partial inspection
projects are unapproved and retain mass/frame gates.

At 100 Hz samples, the selected non-tyre CAD surfaces stay at least 13.5 mm above
z=0 in the default trace, above the generated 5 mm obstacle. This conservative
vertex/plane bound concerns this selected linkage subset; it is not a continuous
whole-robot clearance certificate or an admission of contacts for those bodies.

Ignored evidence under `local-assets/wheel-contact-20261023/` includes the three
CSV runs, `source-wheel.json`, `results.json`, the passive `bind_cad.py` probe,
GPU captures, source/output hashes and validation logs. None is a runtime asset
or a new dependency of core simulator crates.

## Directional contact experiment

The standalone fixture now accepts an **opt-in directional law**. Existing default
runs still use the original native isotropic cylinder and lateral guide. Supplying
`roller_friction` releases the frame's Y guide while retaining its rotation locks;
Y motion is then actual dynamics rather than a commanded sideways pose. The CAD
wheel remains a single visual subtree, with no individual roller bodies.

`runtime::DirectionalFriction` selects a unit collider-local roller axis and
independent dimensionless tread/roller coefficients. The fixture uses local Y,
0.9 tread grip and an assumed 0.02 roller coefficient. Its default tangential
damping is an assumed **500 N s/m**, configurable with `with_slip_damping`.
These are test choices, not recovered rubber, bearing or hardware measurements.
The explicit directional coefficients override the combined native material
friction on supported contacts; the opposite terrain material does not rescale them.

The implementation uses a regularized friction patch:

1. Keep ordinary native normal contacts. For each selected collider contacting
   fixed scenery, collect the preceding integration slice's solved normal impulses
   and pressure-weighted contact position/normal into one patch.
2. Project the collider's actual roller axis into that contact plane. The
   perpendicular tangent is the tread direction; orientation follows the wheel.
3. Implicit zero-velocity damping along both tangents supplies resistance up to
   independent `mu * normal_impulse / slice_dt` force limits. These constraints
   solve **alongside** the suspension joints and normal contacts. No chassis
   speed, position or sprung height is prescribed.

The rectangular friction bound differs from native isotropic Coulomb friction's
circular bound. Below saturation, finite damping permits slip. There is one
integration slice of load lag (at most 1 ms at the tested rates), so first impact
has no directional friction until a normal load is available. There is no
individual roller inertia, distributed twisting resistance or tangent warm start.
A persistent inertialess fixed body anchors the passive tangential constraints;
it has no collider and does not create another plant. Patch/joint storage is
reused for stable contacts. No directional registration retains the ordinary
native stepping path and allocates no directional storage.

This first version supports **fixed terrain only**. Contacts with dynamic or
kinematic bodies retain native material friction. A roller axis nearly parallel
to the contact normal falls back to native isotropic tread friction. Registration
uses runtime-branded collider tokens, rejects stale/foreign/fixed bodies, and must
be reapplied by reset recipes. It is not a serialized match/checkpoint feature.

### Reproduce and interpret

Optional arguments after the original output/height/friction/rate arguments are
`roller_friction`, `lateral_force_n` and `drive_speed_m_s`. The two commands below
compare low/high lateral resistance under the same 5 N force, applied at the
sprung-frame COM after two seconds of settling, with the axle commanded to zero:

```sh
mbx run -p rm-sim-physics --example wheel_contact --locked -- \
  /path/to/roller-low.csv 0 0.9 1000 0.02 5 0
mbx run -p rm-sim-physics --example wheel_contact --locked -- \
  /path/to/roller-high.csv 0 0.9 1000 0.9 5 0

# Forward drive over a smaller 2 mm obstacle.
mbx run -p rm-sim-physics --example wheel_contact --locked -- \
  /path/to/directional-ramp.csv 0.002 0.9 1000 0.02
```

CSV output appends `chassis_y_m` and `chassis_vy_m_s`; existing body-pose columns
retain their names. At both 128 and 1000 Hz, after one second of the lateral push,
the low-drag fixture reaches approximately **0.67 m/s**, while the high-grip
control reaches approximately **0.010 m/s**, consistent with the assumed
500 N s/m damping under a 5 N push. An independently sliding one-kilogram body verifies
rotation of the friction axes, monotonic horizontal energy decay, free sideways
coasting with zero roller coefficient and approximately `mu * 9.81` lateral
slowing with finite roller coefficient. These checks test the approximation,
not agreement with measured omni-wheel hardware.

Forward traction and **2 mm ramp passage** pass at both rates, as do exact recipe
replay, fresh reset tokens, airborne motion and coefficient/admission checks.
The original isotropic 5 mm ramp tests remain passing.

**Known limitation:** the directional model stalls at the original 5 mm
floor/ramp transition with the unchanged 1.2 N m drive cap. The isotropic control
passes it. Changing solve order and using a pressure-centered patch did not
resolve this. It remains an unresolved contact/constraint-model difference,
not evidence that the real wheel cannot climb 5 mm, and the motor limit was not
raised to conceal it. Detailed roller geometry or better coupled contact
constraints remain necessary before using this approximation for game terrain
or robot passability claims.

Validation and CPU-only fixture measurements are retained under
`local-assets/directional-contact-20261024/`. No measured CAD mass, real spring
stiffness or real lateral wheel data is inferred from this experiment.

A local CPU-only probe used the same free-lateral, flat, seven-body fixture in
both modes (native isotropic friction versus directional registration). Five
alternating trials, each after 2000 settling ticks and timing 2048 driven ticks,
gave median **82.29 µs/tick native / 83.32 µs/tick directional** at 1000 Hz and
32 solver iterations on Apple M5 Max, development profile with optimization.
That is approximately 1.3% additional fixture cost in this unpinned run, not a
whole-game or statistically established throughput result. It includes stepping
and truth sampling, excludes CSV/rendering/setup, and both cases finish near
0.247 m/s. The small test does not measure scaling to many registered wheels.
