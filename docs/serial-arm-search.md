<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Faster calibrated serial-arm planning

The existing PIE source-arm planner uses the actual imported bearings, tool and
joint intervals. TongJi remains a separate benchmark model. No ideal wrist,
replacement robot, Python worker or sibling dependency is introduced.

## Implemented, from simpler to larger changes

1. **Analytical Jacobian.** Serial IK computes spatial revolute derivatives from
   the reused FK prefixes and applies the inverse left Jacobian of the SO(3)
   logarithmic residual. It retains joint stops and winding, the 120-iteration
   limit, damping, line search and 10 µm / 0.1 mrad endpoint tolerances. This is
   numerical IK with analytical derivatives, not closed-form IK. Numerical
   branches can change; bit-identical coordinates are no longer promised.
2. **Exact bounded reuse.** Each search keeps up to 32,768 individual IK results,
   including failures, keyed by every model, target and seed floating-point bit.
   Warm seed ordering and root deduplication remain unchanged. Collision scenes
   retain 32 exact endpoint results per body pair, keyed by both poses and
   tolerance. Allowed contacts are checked before cache lookup; proxy scenes
   have independent caches. No nearby pose is treated as already solved/clear.
3. **Collision broad phase.** Precomputed exclusion indices, whole-body bounds,
   first-contact queries and an immutable BVH across source chunks avoid repeated
   allocations and exhaustive chunk-pair scans. Candidate indices are sorted
   back into original order. Swept queries expand the BVH search by the moving
   body's maximum translational/angular travel; narrow-phase CCD remains active.
4. **Separate path keyframes from playback.** IK continuation retains its fine
   cadence to preserve local branches. Live
   collision/path keyframes span up to eight ticks (62.5 ms), then expand into
   the ordinary 128 Hz command stream. Blocked chords retry their fine paths.
   Retiming preserves the represented duration and the existing 1 rad/s joint/
   jaw limits. A carried Unit is reconstructed from actual FK and its fixed
   grasp, including intermediate playback samples. Offline workbench keyframes
   retain their original one-tick cadence. Warm-start samples are spaced by
   playback time, preserving their density when keyframes are reduced.

Live articulated sweeps use **0.025 rad / 5 mm** subdivisions, with continuous
rigid-frame CCD inside each subdivision. This is an approximate articulated
collision policy for the simulator, not a hardware clearance certificate.
Offline inspection retains **0.01 rad / 2 mm**. Grouped neutral-return sweeps
fall back to checking the original rounded playback samples when a coarse
sweep rejects a narrow escape. Clearance still requires collision validation;
endpoint reachability alone never accepts an edge.

Manual parking, persistent starting joints, the deployment-held Unit, immutable
scene ownership, cancellation and bounded search remain in place. The first L3
job plans insertion/P with Q physically zero; only the post-P LED reveal starts
the separate Q job. Both jobs retain the fixed grasp. The existing Q/clock graph
and route search remain the fallbacks. Neutral return allows 256 joint-search
samples within its unchanged 2,500 collision-check budget to retain difficult
post-flip exits. A new full-sequence lazy graph and
joint-plus-free-clock projector are deferred because the tested live phases
already meet the requested 1–5 second target.

## Validation and timing scope

On the two existing manual-parking L3 source-CAD fixtures (upright and settled),
ordinary development builds solve insertion/P in approximately **3.4–3.6 s** and
successful revealed-Q/return cases in approximately **1.9–2.9 s**. L1/L2 jobs,
including return motion, take approximately **3.1–3.8 s**. The -32.47° Q fixture remains a
bounded unresolved continuation in both the original and optimized planners;
it is never counted as completed. Measure each phase separately: Q is deliberately unknown during the first job. These are warm local
fixtures, not random-pose coverage or worst-case bounds. Core positioning,
physical actuation time and initial package loading are outside worker latency.
The source test also exercises actual preload/grasp, restore, checkpoint size,
128 Hz playback, release/return and subsequent Core recovery.

Tests compare analytical derivatives against central differences near zero and
large rotation errors, verify exact-cache isolation, BVH candidate completeness
for sampled moving bounds, first-contact ordering, bounded histories, keyframe
duration and invalid-duration rejection. Existing thin-wall/swept-rotation tests
remain active. The optional source test requires the locally generated package.

The IK benchmark compares with the frozen pre-optimization implementation at
`c642c0cb9ed5e894e1b9a64c40055ae6e4366aaa`. It exercises 768 deterministic
single-seed queries repeated five times, plus 32 multi-seed targets repeated
three times, rotating backend order. Every returned optimized root is checked
against actual FK and original joint bounds. Success-mask differences are
reported, rather than asserting identical numerical roots. Component speedups
must not be presented as full collision-checked planning speedups.

## Reproduce

```sh
nix develop -c cargo test -p rm-simulator-physics --locked
nix develop -c cargo test -p rm-simulator-server --lib --locked assembly_rig::planner::tests
nix develop -c cargo test -p rm-simulator-server --lib --locked source_live_manual_parking_all_levels -- --ignored --nocapture
nix develop -c cargo build -p rm-simulator-physics --example serial_ik_bench --release --locked
target/nix/release/examples/serial_ik_bench /path/to/arm-bearings.txt
```

Set `RM_ASSEMBLY_TEST_LEVEL=3` to restrict the source test to L3. The full test
covers L1–L3 at both parking poses. Its source paths are the existing ignored
`local-assets/assembly-cad/package`, `area` and qualified hint recording.

The benchmark input is 55 whitespace-separated numbers: six joint records of
origin xyz metres, unit axis xyz, minimum/maximum radians; then tool xyz metres
and wxyz quaternion. There are no built-in CAD paths or dimensions. The frozen
reference under `examples/support/serial_ik_reference.rs` is benchmark-only.
