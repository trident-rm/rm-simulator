<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Portable terrain collisions and native heightfields

`rm_sim_assets::terrain_collision::TerrainPackage` loads the candidate package
exported by rm-map-tools. Fidelity (`precise`, `balanced`, `fast`, `coarse`) is
explicit and separate from representation (`Mesh` or `NativePreferred`).
This is an opt-in standalone/scenario API. It does not change default match
loading, replace installed `field-2027/`, or accept this manifest as `--cad-assets`.
The current candidate covers ground and 16 static terrain families, not the
complete field/equipment roster. A match integration must deliberately compose
its missing equipment/boundary and avoid installing the old terrain twice.

## Geometry contract

- `triangle-mesh-v1` consumes the same-quality complete GLB surface.
- `heightfield-grid-v1` consumes finite regular local X/Z samples with local +Y
  elevations in metres. Rows advance Z, columns X; the diagonal runs from
  `(row,col)` to `(row+1,col+1)`. No implicit transposition or normal reversal.
- `hybrid-heightfield-v1` requires both the grid and its checksummed remainder
  mesh. They are one atomic alternative to the complete mesh, not extra geometry
  on top of it. The exporter currently qualifies the floor and two road shells;
  irregular border strips retain the mesh alternative.

The loader checks source/schema/frame identity, unique static work-part IDs,
rigid non-reflected poses, grid dimensions/order, component ownership, complete
payload counts, and confined canonical paths. Selected SHA-256-verified bytes
are parsed from the same immutable snapshot. Unsupported kinds/capabilities may
fall back within the chosen quality; selected corruption never silently omits
collision or changes fidelity. No engine-cooked blob is used as interchange.

`sim_engine::model::HeightField` holds the solver-free validated source.
Cylinder descriptions also live in `sim-engine`; their native cooking and shared
boundary tessellation live in `sim-physics`. RM capture/motion integration uses
the opt-in `sim-physics/rapier-extension` API. Package verification and field
ownership remain in `rm-sim-assets`, without a new package abstraction.
Physics cooks a native Parry/Rapier heightfield with explicit cell subdivision;
fixed installations retain native grids. Capture, ray queries, wireframe triangle
export and `WorldPhysics::try_insert_geometry` preserve native shapes. Dynamic
heightfields and heightfield children inside compounds are rejected. Remainders
are separate colliders of the same fixed model. Source groups/materials are
retained during model installation. Existing legacy capture reconstruction still
uses the facade's historical material defaults; reconstruct caller-selected
materials from the prepared package recipe instead of inferring them from a
wireframe capture.

Both above/below sphere contacts and ray hits are tested. The standalone query
comparison has zero hit/miss disagreements in 100,000 rays, maximum distance
change about 0.000414 mm, and maximum normal change about 0.000676 degrees versus
the exported coarse GLB. These finite tests are not a complete contact proof.

## API

Load with an explicit world transform and contact policy:

```rust,ignore
use rm_sim_assets::terrain_collision::{TerrainPackage, Quality, Representation};
use sim_engine::model::ContactMaterial;
use rm_sim_physics::PhysicalRuntime;

let prepared = TerrainPackage::load(
    package_root,
    Quality::Coarse,
    Representation::NativePreferred,
    world_from_export_yup,
    ContactMaterial {
        friction: 0.4, restitution: 0.45,
        memberships: u32::MAX, filter: u32::MAX,
    },
)?;
let bindings = prepared.install(&mut runtime)?;
```

`world_from_export_yup` is an ordinary rigid `Pose`, not a render
`gltf_root_pose`. Its translation chooses floor reference/centering; its rotation
maps export Y-up into the caller's FLU world. The benchmark documents its source
fixture calibration. Use `install_in_world` for the existing chassis/projectile
facade, after deliberately removing its catch plane and avoiding other terrain.
Preparation verifies/cooks all parts before installation; repeated installations
share immutable source/cooked buffers. Caller-created runtimes still own time,
participant identity, contacts, and complete reconstruction recipes.

## Performance and controls

The release example runs 50,000-ray batches, a 32-ball scene for 1,500 × 1 ms
steps, and a driven chassis for 3,000 × 1 ms steps. It reports query agreement,
final chassis positions and timing, not just triangle counts. Whole-field capture
queries, AABB-cached static capture queries and isolated-road queries are separate.
Native and fallback use the same coarse quality and caller material.

```sh
cargo run -p rm-sim-assets --example terrain_collision_bench --release --locked -- \
  /absolute/path/to/terrain-collision-package coarse 50000
# Alternate measurement order to expose ordering/cache effects:
cargo run -p rm-sim-assets --example terrain_collision_bench --release --locked -- \
  /absolute/path/to/terrain-collision-package coarse 50000 native-first
```

An important control converts the native grids to **f64 triangle meshes**,
preserving the grid poses, collider partition and remainder meshes. Additional
split-float32 and combined-float64 controls diagnose source quantization and
collider grouping. These do not change the shipped representation policy.

Do not attribute all differences against the original GLB fallback to native
heightfields. The original complete float32 GLB and split native package can
produce different trajectories in this faceted drive fixture, despite extremely
close ray geometry. The exact-grid mesh control matches the native final chassis
position to nanometres and has similar drive-step time. Native heightfields do
reduce isolated grid query cost, but that does not establish a large whole-world
solver or FPS improvement. Choose representations at preparation/reset boundaries,
not by swapping geometry midway through a live simulation.

Raw local timing/control results are in ignored
`local-assets/terrain-heightfield-bench/`; no benchmark dump or CAD is committed.
The implementation uses the current pinned Rapier 0.35.3 and compiler toolchain;
no Rust version or third-party version was changed.

### Local release measurements (2026-10-24)

Apple M5 Max, Rust 1.99.1, release build, five process runs alternating which
representation runs first. Ray figures are medians of warmed batches; sphere
and original drive figures are medians of three fresh scenes per process.
The matched exact-grid drive control is one fresh scene per process. The
microbench clock is 1000 Hz; it is not the default 128 Hz match benchmark.

| Workload | Exact-grid split f64 mesh | Native heightfield | Interpretation |
| --- | ---: | ---: | --- |
| 50,000 isolated road capture rays | 8.017 ms | 6.267 ms | 21.8% less query time |
| 50,000 AABB-cached whole-scene rays aimed at road | 14.328 ms | 12.586 ms | 12.2% less query time |
| 32 balls, 1,500 solver ticks | 60.536 ms | 61.800 ms | about 2% slower; no solver win |
| One driven chassis, 3,000 ticks | 46.369 ms | 45.163 ms | small difference, not a 2× native gain |

Whole-field AABB-cached capture queries are approximately unchanged versus the
original portable GLB path. Physical captures currently visit each captured
collider; changing three shapes does not eliminate that whole-scene overhead.

The original portable GLB drive fixture takes about 98 ms and finishes about
0.208 m away in X from the native run. Combining the exact f64 grid and remainder
back into one collider also recovers that slower/different behavior; quantizing
split meshes changes it too. This makes the 98 → 45 ms comparison confounded by
packing/precision/contact dynamics. The exact-grid split control finishes within
about 2 nm of native, isolating native shape choice more cleanly. Do not claim a
54% full-simulator or pure-heightfield improvement from the original drive pair.

## Consolidated implementation

The native-cylinder and portable-terrain implementations now share the current
`sim-engine` / `sim-physics` core. The existing `primitive-shapes-v1` field recipe
and standalone terrain API remain distinct opt-in inputs. Neither changes the
installed `field-2027/` package or default match loading. Cylinder recipes accept
only cylinders; box/hull field recipes are not added by this consolidation.
The terrain benchmark retains its exact-grid split-f64 mesh control, float32
control, combined-mesh control and alternating measurement order. The historical
timings above have not been rerun on the consolidated source and are not a new
performance claim.

## Collision purpose profiles

`rm_sim_assets::collision_profiles::load_part_profiles` prepares three explicit
profiles of one declared rigid source part from the map-tool export inventory.
Fidelity and representation remain separate. Every request names its quality,
representation preference and experimental admission. Mesh fallback stays within
that quality. A selected corrupt payload fails. File hashes and parsing use the
same immutable bytes, including the manifest snapshot supplied by the caller's
expected SHA-256. Selected hybrid payloads are atomic.

The first exported rigid part is the Energy Unit. Rigid-body contacts keep the
reviewed 48-sided mesh, bore and bars. Projectile and LiDAR candidates fill the
socket and approximate its caps/bars. Coarse files declare that they are forbidden
for rigid-body use; both selection metadata and native recipes enforce this.
Other assemblies are not accepted as rigid profile installations merely because
they contain a mesh. Their articulated owner boundaries require an adapter.

`rm_sim_physics::collision_profiles::CollisionPurpose` supplies opt-in actor and
scenery material masks. Rigid actors use bit 0, projectiles bit 1, detailed scenery
bit 3, projectile scenery bit 4 and LiDAR-only scenery bit 5. Each actor's filter
admits one scenery set. LiDAR geometry has a zero physical-contact filter.
`StaticGeometry::for_collision_purpose` selects one query representation and
retains common unprofiled geometry, source transforms and motion channels. Moving
copies must remain attached to the same rigid owner. All profile colliders in a
prepared fixed model share one body's pose, so they cannot drift independently.

Use the supplied actor material policy in the receiving opt-in scene. Default
ALL actor memberships would admit multiple surfaces. These reserved bits and
selection policies are not applied to existing match bodies or the installed
field. The match loader, loose-Unit assembly proxies, scoring colliders and live
LiDAR call sites retain their current defaults. This is a tested standalone
loader/policy API, not a silent global migration. A future match integration must
classify its actors and route each query purpose before installing alternatives.

The example validates detailed and filled socket rays on the actual export:

```sh
cargo run --locked -p rm-sim-assets --example collision_profiles -- \
  EXPORT_ROOT EXPECTED_MANIFEST_SHA256
```

It displays one product-local Unit. It retains all source occurrence matrices but
does not implicitly convert Y-up placement matrices to the caller's FLU frame or
install twelve source copies. The caller owns that explicit boundary conversion.


The example now selects `lidar-coarse` for non-contact rays and visibility. Add
`--benchmark` to compare the actual loader-built three-profile scene against one
detailed mesh per Unit. It measures nearest rays, visibility, projectile stepping
and rigid-body stepping, and checks unchanged rigid query results and trajectories.
Preparation, installation and capture are excluded from the timed queries. The
benchmark covers twelve synthetic Energy placements, not the complete match.

The map tools can now reconcile the existing balanced road grids with the current
reviewed terrain. Its separate 30-definition package retains current road
remainders and uses the finite four-triangle floor. The `reviewed_terrain`
example compares the source mesh, reconciled native grids and floor support,
including finite edges and a chassis traversal across the floor seam:

```sh
cargo run --locked -p rm-sim-assets --example reviewed_terrain -- \
  PACKAGE EXPECTED_MANIFEST_SHA256
```

`terrain_collision_bench` also accepts this package with `balanced`; floor bounds
can come from its finite mesh. Compare `ExactGridMeshControl` when attributing
representation speedups. Road chassis paths can differ even when ray heights
agree. These examples do not deploy the terrain or replace match defaults.

## Bundled native road deployment

The bundled field selects `terrain/manifest.json` through the top-level verified
`terrain_collision` reference. `native-terrain-remainder-v1` requires that reference
and makes the arena collision file the complementary mesh remainder. The manifest
snapshot is verified and retained before deferred cooking. All selected payloads
are verified when cooked. The terrain CAD checksum must match the field source.

The package contains 29 current fixed terrain definitions, including two native
balanced wave grids and their current retained side/cap meshes. The separately
loaded floor uses four collision triangles. Selected terrain owners are removed
from the arena contact mesh, so surfaces do not produce duplicate contacts.
Original ground geometry stays available for support and zone-height lookups.
Both the match adapter and generic arena sessions install the same cooked models;
geometry captures and session reset preserve native heightfields.

Visual assets, optical selectors and motion bindings stay unchanged. Both grids
keep the current 32 segments per wave. This deployment does not select the coarse
query candidates or change the engine from f64. The field contains 206,181 mesh
collision triangles plus two heightfields equivalent to 1,280 grid triangles,
excluding the 48-triangle runtime boundary.
