<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Server and sensor copy costs: 2026-10-23

Scope: HEAD `a721350` plus the preserved uncommitted RMUL work and concurrent
performance changes. No protocol, tick, physical model or sensor fidelity changes.
No commit or staging was performed. The graphics investigation's overloaded
full-app observations do not identify a server or physics bottleneck.

## Removed work

The high-rate owner delivery path previously cloned a complete addressed
`ChassisSnapshot` with `OwnerAnchor::from_state`, then hashed the configuration
for the handshake and hashed it again inside `encode`. It now borrows the
snapshot and passes the already computed revision to the internal encoder.
The public owned-anchor constructor/encoder remains compatible. Contacts are
still omitted from the wire; the borrowed source snapshot is never mutated.

For the default four-wheel chassis, each owner opportunity now avoids **two
nonempty Vec clones** (four wheel-hub positions and four wheel states), including
when the configuration handshake prevents sending an anchor. A successfully
encoded anchor also avoids **one entire configuration serialization/tree build
and hash**. Source articulation/climbing/folding buffers and direct-torque arrays
previously contributed additional recursive clones. These are exact source-level
operation counts, not allocator-interposition measurements or full-app speedups.
The output datagram allocation and the first content-derived revision calculation
remain. Revision invalidation behavior is unchanged.

Mounted LiDAR capture previously requested a full chassis snapshot only to read
its pose and placement revision. It now uses the existing allocation-free
`chassis_control_state`. Default captures avoid the same two nonempty Vec
allocations and all wheel/turret snapshot calculations. At before/after 128 Hz
captures that saves 512 such vector constructions per simulated second per
sensor schedule; retaining the previous after-capture as the next before-capture
saves 256. More sensors may share a capture of the same robot/time at the caller.
Collider buffers and shared shape references are still captured independently;
no stale geometry cache or topology shortcut was added.

## Validation and reproduction

Scoped correctness checks cover borrowed/owned wire equality, direct torques,
Blue team, pause state, quantized decode, omitted wheel contacts, missing chassis,
non-finite motion rejection, and live sensor pose/revision after placement.
The existing owner codec tests exercise configuration acknowledgment and epoch
recovery. Existing rolling sensor tests retain occlusion, bracket discontinuity,
noise determinism, topology checks and point timing.

A short ignored `owner_stream::tests::borrowed_owner_encoding_cost` test compares
12,000 old-path encodes and 12,000 borrowed-path encodes, alternating order in six
rounds. Build first, then run only that probe with one test thread:

```sh
nix develop -c cargo test -p rm-simulator-server --lib --locked -j 2 --no-run
nix develop -c cargo test -p rm-simulator-server --lib --locked -j 2 borrowed_owner_encoding_cost -- --ignored --nocapture --test-threads=1
```

The old arm intentionally includes clone, handshake hash and encode hash, matching
the former delivery path; the new arm includes one hash and borrowed encoding.
This is a no-CAD microbenchmark, excluding physics, queueing, socket transport,
world checkpoint construction and rendering. Timing under concurrent builds is
indicative only; use the structural copy reductions as the contention-independent
result. Do not run a broad network/graphics sweep while compiling.

The release test binary from the shared build was used after confirming it
contained the new borrowed-anchor regression test. SHA-256:
`98e9d3594c5bd08e3be960f8472da4a43c821d500e201699e838d36cc576caef`.
Nine owner-stream tests and 23 UDP-codec tests passed with one test thread.
The ignored probe completed 12,000 encodes per path: old **4,931.3 ns/anchor**,
borrowed **3,134.0 ns/anchor**. It took 0.10 seconds total. Load average was
94.51 on this 18-logical-CPU workstation with concurrent unrelated work and
builds: this is a reproduction check, not a controlled application speedup.
No repeated sweep was run. Formatting and scoped checks are recorded below.

Completed checks:

- Release owner-stream tests: 9 passed, benchmark ignored in the ordinary run.
- Release UDP-codec tests: 23 passed.
- `nix develop -c cargo test --release -p rm-simulator-physics --lib --locked -j 2 sensors::rolling -- --test-threads=1`: 8 passed.
- `nix develop -c cargo clippy --release -p rm-simulator-server -p rm-simulator-physics --lib --tests --locked -j 2 -- -D warnings`: passed.
- `nix develop -c cargo fmt -p rm-simulator-server -p rm-simulator-physics -- --check`: passed.
- Scoped `git diff --check`: passed.

The initially queued debug server build was cancelled while waiting for the
renderer build lock; current release binaries from the shared build supplied
validation and the microbenchmark instead. No simultaneous expensive sweep was
started. The full workspace gate remains the coordinating task's responsibility.

## Ranked remaining opportunities

1. Profile owner-only host state construction separately from encoding.
   `host::Worker::publish_snapshot` currently calls `simulation.state()` even for
   owner-only publications. The borrowed encoder does not remove that upstream
   complete state build.
   Preserve confirmation ordering, restore completeness and world publication
   priority when considering a smaller preparation path.
2. Measure configuration revision hashing with source articulation. The remaining
   content serialization builds a Node tree each owner opportunity. Cache only
   behind an explicit immutable configuration identity/invalidation boundary;
   pointer identity alone cannot prove configuration equality after mutation.
3. Reuse fixed sensor capture data, AABBs and validation across ticks under an
   explicit geometry/topology revision. Mesh buffers/BVHs already use SharedShape;
   fixed shape/pose vectors and lazy AABBs are reconstructed per capture. Detect
   enable/disable, insertion/removal and shape replacement before sharing them.
   Fixed bracket validation also searches the other fixed list for every entry;
   an ordered fast path with a compatibility fallback is a possible small change.
4. `snapshot_codec::EnergyUnitWire` already separates immutable hull coordinates
   from the changing header for compact wire deltas. Its constructor still clones
   the Unit before moving cloned hulls into the geometry record. Likewise assembly
   trajectories are independently packed. Profile these copies with an actual
   assembly/Unit workload before replacing serialization with borrowed views;
   preserve exact restore data and the bounded packed payload format.
5. Profile per-peer baseline tree cloning/prediction and decoding under real
   acknowledged/dropped baseline patterns before changing retention ownership.
   Existing world broadcasts already share lazy quantization, independent packing
   and compression; repeating that optimization offers no gain.

## GPU nearest-hit and LiDAR decision

No GPU implementation is justified by these copy reductions. The current CPU
path shares immutable mesh BVHs and uses cached AABBs to reject shapes before
nearest-hit traversal. A useful GPU prototype would upload verified static
triangles plus its acceleration structure once, retain them for the geometry
lifetime, and update only dynamic transforms/instances. Re-uploading the entire
arena or rebuilding its acceleration data per scan would discard that reuse.

Measure a quiet release-build loaded-field acquisition first, separating capture,
bracket validation, direction generation, traversal and output assembly. Then
compare CPU with GPU at 1,080, 20,000 and 200,000 rays per batch and multiple
sensors/environments. Include warmup, BVH build/upload, per-step instance updates,
submission/synchronization and readback in wall time. At 128 Hz a full-rate
200 kHz sensor has about 1,562 emissions per tick, so dispatch/readback latency
may dominate these smaller rolling brackets; batching across sensors/environments
is the stronger candidate. Preserve emission-time interpolation, self-occlusion,
nearest-hit tie behavior, misses, min-range blocking and deterministic indexed
noise. Do not introduce latency or batch over a placement/topology discontinuity.

If PPO consumes observations on the GPU, retaining output in device memory can
avoid readback; the current Rust/CPU consumer requires it. First measure the
actual point/range payload and copies rather than assuming unified memory makes
synchronization free. Static GPU geometry can be shared across independent PPO
environments via instance transforms, but dynamic authority and explicit ticks
remain CPU-owned unless a separately validated design changes that boundary.
