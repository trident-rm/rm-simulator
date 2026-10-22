<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Navigation sensor foundation

`rm-simulator-physics::sensors` provides ideal instantaneous LiDAR over a
caller-owned `StaticGeometry` capture and mounted rolling scans over timestamped
physics captures, with optional seeded range effects. It needs no match, CAD
loader, renderer, networking, host clock or MuJoCo. It does not perform
localization or planning.

## Nearest-hit geometry

`StaticGeometry::cast_ray(origin_m, direction, max_distance_m)` validates inputs
and returns `Result<Option<RayHit>, &'static str>`. Direction is normalized
internally, so distances remain metres even when its magnitude is not one.
Non-finite origins/directions, zero directions and nonpositive/non-finite maximum
ranges are errors. A valid miss is `Ok(None)`.

`RayHit` contains metric range, world FLU hit point and normal, and the shape
ordinal in that capture. Ordinals are not durable entity identities or semantic
part labels. The query uses cached per-shape AABBs and existing triangle BVHs,
checking every candidate that could improve the closest result. It preserves
mesh gaps rather than convexifying the imported geometry. Equal-range hits keep
the first capture-local shape.

Solid shapes containing the origin occlude at zero range; their normal may be
zero or unreliable. Avoid mounting a scanner inside collision geometry. Triangle
normals follow captured geometry and the underlying two-sided ray intersection;
they are geometric outputs, not a calibrated intensity model.

## Scan API

`LidarConfig` defines horizontal count, first angle/increment, elevation angles
and minimum/maximum range. `LidarConfig::planar` covers 360 degrees without
duplicating the endpoint. Negative azimuth increments support clockwise patterns.
`Lidar::new` validates the bounded pattern and precomputes local unit directions;
the config is immutable after construction.

```rust
use rm_simulator_physics::{Pose, WorldPhysics, sensors::{Lidar, LidarConfig}};

let mut world = WorldPhysics::new(&[], 0.);
world.add_static_box(Pose::at([2., 0., 1.]), [1.; 3]);
let geometry = world.static_geometry_snapshot(true);
let sensor = Lidar::new(LidarConfig::planar(1080, 0.05, 30.))?;
let frame = sensor.sample(&geometry, Pose::at([0., 0., 1.]), 1_000_000)?;
assert_eq!(frame.time_ns, 1_000_000);
assert_eq!(frame.returns[0].unwrap().distance_m, 1.5);
# Ok::<(), &'static str>(())
```

`sample_into` reuses a frame's return allocation. Invalid sensor pose leaves the
previous frame intact. Sensor pose uses a finite FLU position and unit wxyz
quaternion, including the caller-composed robot-to-sensor mounting transform.
Returns are elevation-major, then azimuth-major. `None` means no reported return:
either a miss or a nearest surface below minimum range. A close occluder never
allows seeing a farther surface through it. Maximum range is inclusive.

All rays in one frame share the supplied acquisition timestamp and scene pose.
This is an instantaneous scan; no spinning-scanner motion distortion, per-ray
timing, noise, dropout, intensity, beam divergence or multi-return model is
implemented. `world::session::SampleSchedule` can schedule acquisitions at a
rate independent of integration. Drain it at the correct simulation step;
sampling an old due timestamp against the present world does not reconstruct a
historical scan.

## Captures and remaining work

The same scanner can query fixed, articulated or dynamic captures. Captures
retain immutable shape poses: moving a live robot does not mutate an old scan
scene. Explicitly capture the new scene to observe it at a later time. The current
API operates on one supplied capture and assumes the caller has already excluded
unwanted geometry. It has no durable entity filter or automatic self-robot
exclusion yet.

The bare world's catch half-space is excluded from geometry captures by design.
Use an explicit ground mesh/box for ground returns in bare tests. The loaded CAD
field supplies its ground triangles normally.

Mounted rolling scans provide coherent fixed/moving captures for caller-owned
acquisitions. Scan overlays, noisy odometry and estimated localization remain
separate work. The built-in Sentry planner consumes ground-truth geometry.
The experimental live-game ROS/PolarBear adapter has been removed; the existing
optional standalone ROS publisher remains available.

Run the CAD query/scan benchmark with:

```sh
cargo run --release -p rm-simulator-server --example navigation_query_cost -- field-2027
```

It reports boolean early-out visibility separately from nearest-hit scans, over
three clear sensor origins with 1080 planar or 28,800 multi-elevation rays.
Compare the nearest-hit output when budgeting a sensor. The earlier visibility
proxy measurements in [the baseline](performance-20261021.md) are not equivalent.

## Current MID-360 profile

The runtime pattern now delegates to dependency-free
[mid360-sim](https://github.com/hxyulin/mid360-sim) at pinned revision
`c832a45bbde4d6cc5ef1e09afb0102b699fdc8d6`, using its hardware-derived nominal
profile and representative 18.2-hour cycle. The copied 12-second fit described
below is historical. Capture interpolation, mounted geometry and range effects
remain native physics responsibilities. Optional emitted-ray angular noise uses
the shared `effects::AngularNoise` model; enabling this dependency does not add ROS.

## Historical MID-360 pattern investigation

The following records the original angle-table investigation and copied fit;
it is retained as provenance, not the current runtime scan law.


Investigated on 2026-10-21. The [manufacturer specifications](https://www.livoxtech.com/mid-360/specs)
give 360-degree horizontal coverage, -7 to +52 degrees vertical coverage,
200,000 first-return points/s and a typical 10 Hz frame rate. A 100 ms output
window therefore contains nominally 20,000 samples; it is not necessarily a
complete repeating optical cycle.

The [Livox simulator source](https://github.com/Livox-SDK/livox_laser_simulation/tree/1cce1073633a062b92e30243a4c2920e45551bb5)
contains a CSV replay implementation, not an optical pattern generator:

- `src/livox_points_plugin.cpp::convertDataToRotateInfo` reads three CSV columns
  and converts angles from degrees. `Load` constructs rays from those angles.
- `InitializeRays` advances `currStartIndex` by `samplesStep`, indexing the table
  with `k % maxPointSize`. It does not restart at the first row each output frame,
  but does repeat when the table ends. The parsed time column does not drive ray
  timing in this implementation.
- `src/csv_main.cpp` is only a CSV reader example. The repository tree contains
  no source that generates the MID-360 angle table or models its rotating optics.
- The pinned `scan_mode/mid360.csv` has 800,000 rows, with first-column integers
  1 through 800,000 despite the `Time/s` heading. At the nominal manufacturer
  point rate, sequential playback would last four seconds before wrapping.
  Its SHA-256 is
  `aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a`.
- The file is 16,809,997 bytes (16.81 MB), not 160 MB. The last-to-first
  spherical ray separation is 67.47 degrees. Comparing each of the four
  interleaved sample lanes across the wrap gives 68.89, 70.79, 72.60 and
  74.32 degrees, versus ordinary same-lane steps of about 0.71 to 1.41 degrees.
  Thus this table does not close continuously; its length is not evidence of a
  full optical repeat cycle. A hardware repeat period cannot be inferred from
  this finite sequence alone.
- Its zenith range is 37.836 to 97.2123 degrees. The upstream Euler pitch
  convention makes elevation `90 - zenith` degrees (roughly -7.21 to +52.16).
  These are not elevation angles to use directly in our FLU rays.

The file is useful as a reference sequence for checking a future procedural
model, but finite looping playback must not be described as indefinitely
non-repeating generation. The dataset was inspected in ignored development
scratch; neither its data nor upstream code is embedded in the runtime.

An additional source audit checked the upstream branches, history and MID-360
proposal. The `main` revision remains the pinned commit above; `gazebo7` has no
MID-360 table. In `InitializeRays`, the loop count affects only table indexing:
there is no quarter-cycle reconstruction, mirroring or added optical phase.
The sensor mounting rotation does not supply such a phase. The published
[MID-360 xacro](https://github.com/Livox-SDK/livox_laser_simulation/blob/1cce1073633a062b92e30243a4c2920e45551bb5/urdf/livox_mid360.xacro)
actually defaults to 24,000 samples at 10 Hz (240,000 points/s without
downsampling), so its configured playback traverses the table in about 3.33
seconds. Four seconds describes the table at the hardware's nominal point rate,
not a duration enforced by the plugin.

The closed, unmerged [CPU/GPU proposal #14](https://github.com/Livox-SDK/livox_laser_simulation/pull/14)
also retains modulo CSV playback in its CPU path. Its GPU plugin consumes
scan angles from Gazebo rather than defining an analytical optical generator
in the plugin. The [SDK2 scan-pattern API](https://github.com/Livox-SDK/Livox-SDK2/blob/master/sdk_core/livox_lidar_sdk.cpp)
forwards a device command, not a trajectory calculation. These inspected public
sources do not supply the optical generator; this does not establish what
unpublished firmware or internal tools contain.

The similarly named `LivoxGenerator` in
[MuJoCo-LiDAR](https://github.com/discoverse-dev/MuJoCo-LiDAR/blob/main/src/mujoco_lidar/scan_gen.py)
also loads a stored angle array and wraps it; the class name does not establish
that it synthesizes the optical trajectory.

For occupancy and basic path planning, coverage, point budget and vertical blind
spots are useful first approximations. For short-window detection, LiDAR odometry
and deskewing, spatial ordering and per-point acquisition times also matter.
A procedural approximation should preserve continuous phase across frame
boundaries, expose per-point times and remain explicitly distinct from calibrated
MID-360 emulation. The current instantaneous grid scanner implements neither
MID-360 replay or rolling optical motion. The procedural approximation below
adds time-varying directions while retaining the frozen-capture query model.

## Fitted procedural MID-360 approximation

`sensors::mid360::Mid360Pattern` evaluates a compact empirical model; it is not
an identified optical design or calibrated hardware emulator. Four interleaved
lanes have a fast phase `s = 2π × (2189/12) × t` and a slow phase
`v = 2π × 10 × t`. Azimuth is `-s` plus a Fourier correction, while elevation
is a Fourier series. Each series uses 22 sine/cosine pairs at combinations
`j*s + k*v`, with `j=0, k=1..7` and `j=1, k=-7..7`.
The directions are unit FLU vectors from the resulting azimuth/elevation.

Frequency fitting initially estimated 182.416664 and 9.999998 Hz. Replacing
these with the simple rational hypotheses above preserves held-out accuracy
and avoids unbounded linear drift. These rates assume the specified 200,000
points/s and four interleaved lanes: the CSV itself supplies no usable clock.
There is no claim that those rational rates exactly match the physical motors.

One vertical sweep takes 100 ms. During it the fast phase makes 18 + 29/120
turns, so successive sweeps are shifted by **87 degrees clockwise**. This
explains coverage growth without injecting a separate per-frame offset. After
120 sweeps (12 seconds), the rational model repeats. Real optical clocks need
not have this exact ratio: incommensurate ideal frequencies have no finite
exact repeat, and physical drift/jitter can prevent exact retracing. Livox's
[description of non-repetitive scanning](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/introduction/livox_scanning_pattern.rst)
describes increasing coverage with integration, not a guarantee of a particular
long-period recurrence for this device.

The fit uses 10,812 individual points from the first two seconds. Validation
against **all 400,000 points in the remaining two seconds** gives per-lane
spherical angular RMS errors of 0.05594, 0.01312, 0.01312 and 0.05595 degrees.
The largest held-out error is 0.15549 degrees. This is approximation error against
the published rounded table, separate from hardware measurement noise.
The model stores 360 f64 coefficients (2,880 bytes), plus 22 integer frequency
pairs; it does not store or loop the 16.8 MB table.

`direction(global_sample_index)` preserves phase across frame boundaries and
works beyond four seconds. `point_offset_ns` supplies a nominal uniform 5 µs
cadence with checked overflow; exact inter-lane firing timing is unverified.
`sample_into` accepts a `PatternWindow` with explicit first index, count and
stride, preserves its ordering and reuses `LidarFrame` storage. It queries a
single frozen scene/pose: a caller must separately supply historical poses and
captures to synthesize motion distortion. Decimation retains source phase;
strides divisible by four repeatedly select the same lane and should be avoided
when representative angular coverage is desired.

Reproduce the fit with NumPy installed in a development environment (no new
runtime dependency):

```sh
curl -fL https://raw.githubusercontent.com/Livox-SDK/livox_laser_simulation/1cce1073633a062b92e30243a4c2920e45551bb5/scan_mode/mid360.csv -o local-assets/mid360.csv
python scripts/fit-mid360-pattern.py local-assets/mid360.csv --rust-output local-assets/mid360_coefficients.rs --report local-assets/mid360-fit-report.json
```

The fitter verifies the source hash and fails if any training-interval or
held-out direction differs by more than 0.2 degrees. Generated coefficients
retain the source notice; see [NOTICE.md](../NOTICE.md).

Evaluate coverage with NumPy and SciPy installed:

```sh
python scripts/evaluate-mid360-coverage.py local-assets/mid360-fit-report.json local-assets/mid360.csv --output local-assets/mid360-coverage.json
```

## Angular coverage evaluation (2026-10-21)

Coverage below means the fraction of **360 × 59 one-degree azimuth/elevation
cells** visited inside the declared rectangular FOV (-7 to +52 degrees), with
the sensor fixed and no obstacles. This resolution-dependent occupancy metric
is not a manufacturer beam-footprint coverage measurement. Gap estimates query
the nearest sampled direction at 0.25-degree grid centers, not a proven continuous
worst-case bound. No range noise, motion, reflectivity or scene occlusion is used.

| Accumulation | Points | Model cells visited | Original CSV cells visited | 95th percentile nearest-direction gap |
|---|---:|---:|---:|---:|
| 100 ms | 20,000 | 63.22% | 63.01% | 1.26° |
| 200 ms | 40,000 | 82.24% | 82.15% | 0.83° |
| 500 ms | 100,000 | 96.41% | 96.30% | 0.48° |
| 1 s | 200,000 | 97.11% | 97.02% | 0.40° |
| 4 s | 800,000 | 97.16% | 97.09% | 0.19° |
| 12 s (extrapolated) | 2,400,000 | 97.17% | unavailable | 0.09° |

Individual, non-accumulated 100 ms windows over the inferred 12-second period
visit about 62.0–66.2% of cells. Accumulation mainly fills gaps in the first
500 ms, then densifies already visited areas. The remaining missing cells are
concentrated at the lower and upper envelope. In the complete model cycle, all
one-degree cells from -5 to +48 degrees elevation are visited; the top 51–52
degree band visits only 44.4% of azimuth cells. The largest sampled residual
gap is approximately 4.11 degrees near elevation +51.875 degrees and azimuth
0/180 degrees. This appears to be the pattern's nonrectangular angular envelope,
not a gap solved by waiting longer; the published CSV also stops near 97.1%
under the same rectangular-cell metric. Do not artificially fill those boundary
regions when emulating this source pattern.

For navigation this is a useful coverage approximation: a single 100 ms frame
is sparse, while a rolling 200–500 ms local map is substantially denser. A moving
robot requires time-aware accumulation/deskewing; simply merging world points
from frozen scans is not sufficient. Twelve-second extrapolation is a model
prediction only. Testing a recorded real sensor sequence longer than four
seconds is necessary to validate the inferred long-period behavior.

## Mounted rolling acquisitions

`physics::sensors::rolling::MountedLidar` mounts a `Mid360Pattern` in chassis FLU
coordinates. `Field::lidar_scene_capture(robot)` captures enabled fixed and moving
colliders, the actual robot pose, simulation time and robot placement revisions.
The lower-level `WorldPhysics::lidar_scene_capture(robot, time_ns)` accepts an
explicit caller-owned clock. Neither advances time or creates a second world.
Shapes are shared, while transforms are frozen.

Capture **before and after each integration step**, then select emissions with
`window_between(before.time_ns(), after.time_ns(), stride)` and call
`sample_into`. Each point uses its own nominal emission time. Chassis and collider
translations interpolate linearly and rotations take the shortest quaternion
arc. Fixed shapes use cached AABBs and mesh BVHs; moving shapes have conservative
swept bounds that include intermediate rotated corners. This models motion
within a step approximately, not true continuous acceleration or deforming CAD.
Do not bracket a large movement or a rotation exceeding π with just two captures.

The configured `robot_from_sensor` pose includes the mounting offset and tilt.
**Own-robot colliders remain present.** A sensor inside a solid sees a zero-range
occluder and produces no return when below its minimum range. Robot-mounted
shadows remain even when a more distant wall is visible. Collision geometry is
used as the optical surface proxy, including simplified chassis hulls and guards;
small visual-only details are not yet separate sensor geometry. Unlike the older
`StaticGeometry` snapshots, these scene captures include the enabled bare-world
catch half-space, so the no-CAD example can see the ground.

`RollingFrame` stores `TimedReturn` entries in emission order, each with its global
sample index, absolute simulation timestamp, optional measured range and optional
point in **that emission's sensor frame**. Misses preserve timestamps. These are
raw points, not automatically deskewed points in one reference frame. No perfect
per-point world pose or hidden robot identity is included in measured output.
Use an independently supplied motion estimate to deskew them for mapping.

`RangeEffects` explicitly configures range bias, Gaussian range standard deviation
and independent return dropout. Defaults are noiseless. This is a generic model,
not a calibrated MID-360 material/reflectivity response. Noise is keyed by device
seed and global sample index: batching or decimation does not consume a mutable
RNG or alter retained measurements. Nearby occluders are traced before effects;
noise does not see through a below-minimum surface. Noisy out-of-range/non-finite
measurements are suppressed, not clamped into artificial returns.

`MountedLidar::with_angular_noise(standard_deviation_rad)` opts into independent
tangent-axis perturbations before intersection, using the same device seed and
global emission index. It reuses `mid360-sim::effects::AngularNoise`; the existing
RM range/dropout random streams are unchanged. Zero is the default and restores
the original unperturbed directions. Standard deviation must be finite and in
`[0, pi]` radians. This is a generic stress model, not calibrated encoder noise.

```rust
let sensor = MountedLidar::new(pattern, mount, epoch_ns, range_effects)?
    .with_angular_noise(0.001)?;
```

The emitted direction is transformed with the interpolated carrier pose and
fixed mount at each emission time; the same direction is used for local XYZ.
Noise can change which side of a foreground edge is hit. A nearest surface
inside the minimum range still blocks the background, even if no point is
reported. Reconstruct the sensor with the same angular setting and seed after
restore; there is no additional RNG state to checkpoint.

Window timing is half-open `[before, after)`. Global stride alignment prevents
missing/duplicate emissions when a 128 Hz tick falls between nominal 5 µs firing
times. At full rate, one simulated second emits exactly 200,000 points.
Prefer odd decimation strides to retain all four pattern lanes. The sensor is
immutable; checkpoint the caller's acquisition epoch, global index/schedule,
mount, effects and seed, then take fresh captures after restore.

Both captures must retain the same enabled collider topology and shared shapes.
Changing fixed geometry, adding/removing colliders, or teleporting/reviving any
robot makes the bracket invalid. A fixed shape's pose must also remain fixed.
Invalid inputs leave the output frame unchanged. Capture on either side of such
an event and restart the bracket; do not interpolate across it. If a folding
mechanism replaces a collider shape rather than moving a rigid source shape,
its replacement also requires a new bracket. There is no automatic host service
that silently discards these errors or substitutes a frozen scan.

Run the no-CAD moving-chassis example, optionally writing raw local-frame points
and explicit misses to CSV:

```sh
cargo run --release -p rm-simulator-world --example rolling_lidar -- local-assets/rolling-lidar.csv
```

Run the loaded CAD probe with one driven chassis:

```sh
cargo run --release -p rm-simulator-server --example mounted_lidar_cost -- field-2027
```

An initial release probe on the M5 Max (2026-10-21, current working tree) produced
200,000 points / 94,704 returns in one simulated second, with 212.994 ms acquisition,
0.173 ms capture construction and 47.323 ms physics. The mount is 0.4 m above the
chassis origin; effects use 20 mm range standard deviation and 1% dropout. The
arena has 447,885 collision triangles. This is one workload/sample, excludes
asset loading and rendering, and does not predict multiplayer performance.
The independent bare-world probe used 58.401 ms acquisition for the same point
budget. Mesh queries dominate the loaded-world difference. Reports remain in
ignored `local-assets/rolling-*-example.log` files.

An [optional rclrs adapter](../integrations/ros2/README.md) publishes measured
rolling frames as Livox Driver 2 CustomMsg and simulation `/clock`. Its live-host
mode also exports synchronized mounted IMU measurements from the same world;
see [synchronized sensors](synchronized-sensors.md). Its native ROS checks use
Pixi/RoboStack; no ROS dependency enters this core.

## Mounted inertial measurements

`physics::sensors::imu::MountedImu` measures angular rate and specific force at
a point rigidly attached to the chassis body or to the actual gimbal frame. It
consumes an opt-in trace (`Field::set_carrier_trace`, `drain_carrier_trace`) of
every chassis' body and gimbal pose at the end of every solver substep (at most
1 ms), so it resolves motion finer than the 128 Hz tick and does not depend on
how ticks are partitioned.

- **Gyroscope:** the relative rotation between consecutive mounted poses over
  the substep. On a spinning chassis it reads the spin rate. A gimbal-mounted
  sensor reads the actual motor-limited yaw/pitch motion; the world-stabilised
  gimbal does not spin with the chassis.
- **Accelerometer:** the second difference of the mounted point's world
  position, plus the solver's gravity (`GRAVITY_M_S2`, 9.81 m/s², not
  9.80665), rotated into the sensor frame. Centripetal ω²r and tangential α×r
  terms of an off-centre mount are therefore inherent, not a separate model.
  At 15 rad/s a 0.2 m offset reads about 45 m/s² radially.
- **Output:** each sample is the time-overlap mean over the preceding period,
  stamped at its end, in sensor FLU, rad/s and m/s². That boxcar lags the motion
  by half a period: a modelling choice, not a measured filter delay. The
  centred accelerometer difference delays completion by half a substep.
- **Continuity:** placement revisions, trace gaps and invalid poses restart
  differentiation and drop the affected periods; no motion is invented across a
  teleport, revival or restore. `ImuState` restores an identical continuation;
  the trace itself is observation, not checkpointed world state.
- **Effects:** seeded white noise, constant bias, bias random walk and
  saturation, indexed by output sample number, all off by default. They are not
  calibrated to any device; take values from a datasheet or Allan-variance
  measurement and record their source. Scale-factor, misalignment, temperature
  and quantisation errors are not modelled.

For an IMU inside a LiDAR, compose the LiDAR mount with the device's documented
LiDAR-to-IMU extrinsic, and use one clock for both. Livox consumers commonly
expect acceleration in g; convert exactly once at the bridge. Rapier contact
jitter appears in the accelerometer as deterministic numerical noise. It is not
a vibration model, so check a stationary and a spinning robot before tuning an
estimator's noise parameters.

The [live synchronized stream](synchronized-sensors.md) pairs these measurements
with rolling LiDAR, using bounded workers outside the host owner and optional
ROS SI publication. It publishes no perfect orientation or odometry.

App overlays, noisy wheel odometry, estimated-pose
SLAM, incidence/material response and angular measurement noise remain future
work. The existing Sentry planner still consumes ground-truth geometry; these
sensor APIs do not silently replace it with a localization stack.

## Standalone sensor project

The sibling `mid360-sim` repository contains a standalone, simulator-independent
pattern/acquisition core and rclrs publisher, with Pixi/RoboStack setup, copied
Livox interface notices and a moving-sensor room demo. It adds optional angular
ray perturbations, now shared by this mounted scanner, and an explicitly
uncalibrated diffuse dropout stress model that is not enabled in RM acquisition.
The existing physics implementation and ROS/control bridge remain compatible;
the pattern core is now a pinned Git dependency. Its other simulator backends remain independent. Its `docs/realism.md` prioritizes optical geometry,
IMU synchronization, material calibration and long real-sensor recordings.
