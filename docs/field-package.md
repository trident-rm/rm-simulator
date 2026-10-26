<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Field package

The application and headless host use only the reviewed, bound `field-2027/`
package. Discovery checks beside installed `bin/` and then the checkout;
`--cad-assets` overrides that location. Live matches require native bindings.
See [2027 integration](rmuc2027-field.md#default-bound-repaired-low-package).

The directory and collision contracts below are shared with offline tools.
Historical 2026 composition and deployment instructions are archived at
`rm2026-final`. Its `field/` package and inventory have been removed from main.
The active pipeline pins `scripts/release-field-2027.json`; `just field-check`
verifies that single package.

## Textures and semantic assets

Text and paint can be exported as native glTF texture patches. Their PNGs,
UVs and alpha-mask materials are embedded in the GLBs; no texture sidecar loader
is needed. Textured CAD materials retain their exported appearance. Package composition and sidecar bindings are documented in
[semantic assets](semantic-assets.md). See [field detail](field-detail.md) for
rebuilding a simplified package; use loader counts and manifest hashes when
comparing packages.

The simulator supports rm-map-tools semantic packages: exported joint frames,
stable IDs and fixed geometry drive animation and collision selection. See
[Semantic assets](semantic-assets.md) for package composition, running this
version, geometry counts and remaining fitted detector data.

## Independent equipment instances

`rm_sim_assets::components::load` composes one verified exported asset into
an asset-local `ComponentModel`. Native body partitions and semantic joint
subtrees reuse the existing package metadata; visual materials and geometry are
not regenerated. Apply one exported root placement with `placed`, then consume
the `PlacedComponent` into `WorldPhysics` or a `RobotScenario` setup recipe.
The same builder supplies live server terrain, preserving source triangle and
mechanism order. See [standalone components](physics-reuse.md#standalone-arena-components)
for the source-Outpost example, coordinate convention and rule/geometry boundary.

## Deployment

The approved native 2027 source adapter and reproduction steps are documented in
[the field guide](rmuc2027-field.md). Staging verifies the pinned LFS files and
refuses to overwrite a destination:

```sh
python3 scripts/stage-release-field.py --verify-only
python3 scripts/stage-release-field.py
```

The staged package is `dist/field-2027/`. Development overrides use
`--cad-assets PATH`; restart the app/server after replacing a package.

## Collision contracts

`reviewed-cad-proxy-v1` selects independently reviewed, checksummed approximate
CAD collision geometry. It allows a low-detail visual export to retain the
preceding collision meshes without claiming that either is an exact source
tessellation or inventing simplifier error metadata. The package's
`runtime-build.json` records source/input hashes, component coverage and placed
counts. The RMUC 2027 repaired Low export uses this contract; visual-only marking
definitions remain excluded from collision.

Static collision loads each asset's declared, checksummed collision geometry,
falling back to visual triangles for packages without a supported collision
contract. Reproducible approximate meshes are permitted to reduce CPU and memory
cost, provided ramps, clearance, traversable openings and scoring behavior are
verified. Rune and outpost moving targets remain rule-driven bodies. CAD source
materials and semantic joints remain intact.

The explicit `opaque-panel-reconstruction-v1` collision contract selects a
checksummed derivative containing intentionally solid panels in place of source
openings. The RMUC 2027 center-platform wall builder uses it for the four
user-approved vent closures; `solid-wall-build.json` records selected nodes,
input provenance and matching visual/collision panel geometry. This is an
intentional topology change, not a measured source-tessellation approximation.

New exports may declare `collision_method: "source-tessellation-v1"` per asset.
The simulator then uses that asset's checksummed collision GLB, tessellated from
the same source faces with separate tolerances. Unmarked legacy proxies remain
ignored and those assets use their visual geometry. This does not change
rule-driven scoring shapes or joint motion.

Ground-height queries for chassis spawning use an index of triangle footprints
built during terrain loading. The index retains the original triangles and
height tolerances, including stacked surfaces. Wheel suspension continues to
use Rapier's collision queries.

## Parameterized collision shapes

`collision_method: "primitive-shapes-v1"` explicitly selects a hybrid recipe.
The asset keeps its original `collision` GLB and `collision_sha256` as the
provenance source and adds two fields, relative to the containing manifest:

```json
{
  "collision_method": "primitive-shapes-v1",
  "collision_shapes": "primitive-collision/tech-core/collision-shapes.json",
  "collision_shapes_sha256": "<sha256 of the sidecar>"
}
```

The initial adapter accepts `rm-map-tools.collision-shapes.prototype-1`: metres,
`source_glb_active_scene` coordinates, xyzw poses and local +Z solid cylinders
with full lengths. It verifies source/remainder hashes, active-scene node names,
indices, ancestry and component ownership, exact selected mesh removal, unchanged
retained geometry/metadata, and positive finite dimensions. Axis vectors must
agree with the pose. It rejects malformed recognized recipes and unsupported
shape kinds rather than silently falling back to a visual mesh. Other contracts
retain their existing behavior. Producer prototype/review notes are informational;
using the contract is an explicit package selection, not contact-fidelity approval.

The loader reads each recipe, source GLB and remainder once, then hashes and
parses those same bytes. `CadAsset::physics_bytes` and `physics_mesh` retain the
verified remainder through deferred component loading and reset. `physics_file`
is a provenance path for hybrid assets, not permission to reopen mutable input.
Changing or deleting the files after preparation cannot change retained collision.

`remainder.glb` retains all mesh occurrences except the selected replacements.
Multiple cylinder parts may replace one selected node, but that source node is
removed exactly once. The original source collision remains available for
provenance and binding validation; it is not also installed as a collider.
Remainder references must stay within the sidecar directory. All runtime paths
are package-relative; no sibling exporter or CAD source is a runtime dependency.

At loading, +Z/full length/xyzw becomes native +Y/half-height/wxyz. Source GLB
node transforms are already baked into exported cylinder poses and are not
applied again. The usual checked asset root placement and floor conversion then
place both meshes and primitives in world FLU. Source body bindings and semantic
joint ancestry assign each native part to one fixed or moving owner; ambiguous
motion ownership fails. Native-only moving parts are retained even when their
mesh becomes empty. Child poses are baked into source rest coordinates before
prescribed translation/rotation, matching the existing mesh motion contract.

Native children share compounds of at most 255 parts per source placement or
motion owner. Retained triangle surfaces remain separate colliders; they are
not nested into primitive compounds. The solver, direct rays, sweeps, LiDAR
captures, prepared scenes, composed instances and resets keep native shapes.
Collision wireframes alone use deterministic 32-segment cylinder tessellation,
including compound child transforms. Extracted moving captures retain collision
filter masks. This does not guarantee a speedup; benchmark the actual workload.

The floor and `arena-static` remain mesh-backed in this first field adapter,
since chassis spawn/support-height queries use their triangle footprints. Native
recipes there are rejected explicitly. Fixed equipment, additional placed
scenery and supported moving equipment channels may use cylinders. Existing
visual files, detector definitions and installed field manifests are unchanged.

Stage a candidate in a **new** directory, then inspect it:

```sh
python3 scripts/stage-primitive-collision.py field-2027 \
  --bundle /path/to/exported/native-cylinder-bundle \
  --out local-assets/native-core-trial
mbx run -p rm-sim-assets --example inspect_primitive_shapes --locked -- \
  local-assets/native-core-trial
just run --cad-assets local-assets/native-core-trial
```

The staging helper copies the source package, checks the bundle source/remainder
hashes, updates only the chosen asset's collision descriptor, and refuses an
existing output directory. The simulator loader performs full shape and retained
geometry verification. The inspector exercises terrain preparation and each
placed cylinder's side/cap ray queries; diagnostic triangle export also includes
the new shapes. No installed package is replaced. The current three-cylinder
Tech Core joint trial loads **539,225 asset mesh triangles plus six native
cylinders in two placement groups**, excluding the separate 48-triangle boundary.
That is 6,374 fewer mesh triangles than the installed 545,599 reference; collision
shape adoption and performance measurements remain separate work.

The exporter currently writes JSON/GLB and an SDF rest-pose model. SDF is an
interchange artifact, not an input to this runtime loader. The JSON prototype's
other future primitive kinds are not accepted implicitly; cylinders are the
supported analytic shape in this initial field contract.

## Terrain

The V1.2.0 floor slab is flat at height zero and the
terrain plates (the centre-line plateaus at x ≈ ±6.3..8.5 m, the 起伏路段
undulating roads near the side walls, ramps, the central highland, covered
passages) stand on it. The V2.0.0 slab is crowned
instead: about 0.11 m below the reference height along the centre line,
falling to about 0.32 m below at the side walls, with height zero at its
small level pads. Invisible walls, 2 m high, surround the loaded floor bounds
to keep robots inside the arena. The CAD floor remains the driving surface;
there is no invisible catch plane beneath a loaded field.

Both CAD releases put a hex-marked truncated pyramid on the centre line at
x ≈ ±6.3..8.5 m: a 1.5 × 1.0 m flat top 0.15 m above the floor with 17°
ramps on all four sides, which the chassis crosses (the default spawn faces
the red-side one). The 起伏路段 undulating road, a 2.4 × 2.1 m strip of
waves about 70 mm high on the 0.2 m deck near each side wall (x ≈ 6.3..8.7,
y ≈ 5.4..7.5 m and its point mirror), exists only in the V2.0.0 STEP; the
default package carries the two V2.0.0 solids grafted onto its flat deck in
V1.2.0's plate colours (listed under `grafted` in the manifest), so both
maps have it. The covered passages under the base highland decks (around
x = ±12.5, y = ±5.5 m) have 0.65 m of clearance in both.

## Placement frames

Placements come from the manifests' `placements_in_source_arena_frame`. The CAD
arena frame is forward/left/up in metres; the simulator centres the field on
the origin and puts the top of the playing floor at height zero. Rune hubs are
derived from the `face_0`/`face_1` pivots inside `rune.glb` (targets on the
blade front plane, 698.5 mm orbit) and outpost bases from the outpost
placements, including the CAD's slight lean, so the emissive rune targets and
outpost armor overlays coincide with the imported geometry.

## Arena colours

The arena's colours are the CAD's. The V1.2.0 STEP styles every arena face
(white for most surfaces, dark grey slab and plate tops, beige plate sides,
red and blue 1 mm marking sheets), and the package keeps them as glTF
materials. The V2.0.0 STEP styles every arena solid in the exporter's
unpainted cream and its markings are flush cream solids; the simulator
recolours only that yellow-tinted cream (floor dark, arena light grey), so
painted materials, white line markings included, are left alone.

The default field sits in a dark indoor hall with neutral lighting. Distant
seating, walls and roof lights are baked into a cubemap; the nearby fence and
floor apron retain geometry. Equipment
paint follows team ownership: red occupies +x and blue occupies -x. Destroying
an outpost opens its team's base shields. Friendly-fire rules are unchanged.

Scene reference fixes and reproducible rune-state captures were audited against
the rule manual. Rune arrows follow simulation
time, including pause and step. Neutral lamps preserve the imported field paint;
only rune optics receive the state-dependent material treatment.

## Loading, splash and the collision view

A splash screen shows stage progress and the current
loading activity. Verification, terrain loading, physics construction and
connection setup run in the background. Visual CAD instances load across
frames before local gameplay starts. The bar measures stages and completed
instances, not elapsed time. Press `Esc` to cancel a join and return to the
title screen. GPU uploads
and individual scene instantiations can still briefly stall the window. The
startup log reports triangle counts; the F3 debug panel shows the geometry held by the physics.
Press C to cycle Off / Overlay / Only. The green wireframe is built once from
client-loaded, verified collision assets and reused. It makes no debug requests
to the server. Unknown or mismatched packages remain unavailable.
Amber chassis, armor, projectile and articulated scenery outlines use the client's
predicted and interpolated content. Moving CAD wire meshes are built once in the background; all collider transforms
then follow the same scene poses as the visuals every frame. No physics world is
created and gameplay poses are never replaced. This is a client inspection view,
not a capture of current server physics.
Scenery stays visible until the fixed wireframe is ready.
The panel offers Off / Overlay / Only radio buttons, a visual mesh wireframe
checkbox and a persistent rendering statistics overlay with FPS, frame time,
visible mesh instance counts and render pass CPU times. Visual wireframe is
disabled when the GPU lacks support.
Screenshot mode waits for a requested wireframe and reports an error if it is
unavailable or the build fails. Capture renders into a dedicated image at the
window’s physical resolution. It waits for GPU pipeline compilation and
three settled frames, retries blank readbacks up to three captures, and fails
if capture has not finished within two minutes of gameplay readiness.
GPU mesh uploads still happen through Bevy.

## Opt-in portable terrain collisions

The separate [terrain collision API](terrain-collisions.md) supports explicit
quality tiers, native finite heightfields and atomic mesh remainders. It does
not replace this installed field manifest or default loading path.
