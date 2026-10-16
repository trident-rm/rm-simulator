<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# RMUC 2027 runtime field

The RMUC 2027 V1.0.0 CAD can be loaded using the simulator's existing runtime
package layout: `manifest.json`, floor and arena GLBs, rune and outpost GLBs,
and `equipment/manifest.json` with base and tech-core GLBs. The 2026 package
remains separate and stays the default.

The initial package is a **CAD rest-pose sandbox** with source-colored scenery,
static collision and chassis exploration. A separately generated, checksummed
structure binding enables the **RMUC 2027 V1.0.0 control/structure match subset**
described below. The bundled 2026 field remains unchanged; no 2026 referee,
armor damage table or rune is enabled on the 2027 field.

## Build from rm-map-tools

Use the optimized `rmuc2027` component package produced by rm-map-tools' 2027
export workflow. The adapter uses its existing Python environment and GLB
helpers; no Rust path/git dependency on that repository is added.

```sh
../rm-map-tools/ocpenv/bin/python scripts/build-rmuc2027-runtime.py \
  --source ../rm-map-tools/out/rm2027/optimized/rmuc2027 \
  --map-tools ../rm-map-tools \
  --out local-assets/field-2027
```

The output must not already exist. The build is staged in a temporary directory
and installed only after verification. Choose a new output directory to rebuild
without overwriting a candidate. Original CAD and generated candidates remain
outside Git; keep the upstream `CAD-NOTICE.md` with the runtime files.
The currently bundled LFS `field/` and `scripts/release-field.json` still
identify the 2026 package. A candidate does not replace that release inventory.

The adapter requires the source hash in
[the partition recipe](rmuc2027-runtime.json):
`a5ad3ede3b7674fc98f2e73b71717216383cb2b4b5dcdbe900c2038918bea8e0`.
This is the source STEP hash, not a hash of the generated runtime directory.
It verifies each input GLB against the source manifest, rejects missing source
coverage, and accepts only the existing locked-border meshopt input policy.

## Source part assignments

The source exporter groups several mechanisms inside `centre-platform.glb`.
The adapter separates exact source product IDs, preserving their local geometry
and composing the source node and arena placement once. No 2026 equipment,
reconstructed interiors, artwork masks or terrain grafts are copied.

| Runtime asset | Source selection | Placed copies | Visual triangles per copy | Collision triangles per copy |
|---|---|---:|---:|---:|
| `floor.glb` | Source floor, baked in arena coordinates | 1 | 3,236 | 3,236 |
| `arena-static.glb` | Remaining source scenery and central-platform products, baked in arena coordinates | 1 | 758,706 | 596,498 |
| `rune.glb` | Central product 1354629 | 1 | 491,419 | 491,419 |
| `outpost.glb` | Central product 2603236 | 2 | 98,282 | 98,282 |
| `equipment/tech-core.glb` | Central product 1751926 | 2 | 84,147 | 84,147 |
| `equipment/base.glb` | Source base product 4774244 | 2 | 110,761 | 79,210 |

These are source-rest-pose partitions for visual review, not reviewed rigid-link
or target-face selections. The whole rune and each whole outpost are stationary
collision geometry under a `static` node. Their internal moving/fixed parts
must be separated before enabling physical animation.

`runtime-build.json` records all 68 placed source-product occurrences, their
runtime assignments, input/recipe/helper/script hashes, original simplification
records, expected bounds and measured runtime bounds. Both visual and collision
coverage must be complete. Repeated instances must have identical canonical
geometry and colors before the adapter shares a mesh.

The generated package retains all **1,839,741 placed visual triangles** and
**1,614,431 placed collision triangles** from the optimized upstream full map.
The upstream `full-map.glb` is a duplicate composition and is not loaded again.
Output GLBs are read back to verify placed triangle counts and bounds within
10 micrometres of the inputs. These checks establish composition fidelity;
they do not certify the upstream mesh simplification against CAD.

## Coordinates and collision

The output retains the source's Z-up arena frame, uses metres and records
explicit xyzw placement quaternions. Existing loader conversions put this frame
into world FLU and renderer coordinates. The floor reference is the highest
vertex of the source floor mesh, approximately -1.640343666 m in source Z.
This selects a vertical origin; it does not flatten the crowned floor. Spawning
and wheel contacts continue to query the loaded terrain.

The collider files preserve the upstream independent, optimized collision
geometry. Manifest simplification metadata records the worst producer error
across each partition's contributing source assets; the build report retains
those individual records. Static floor/scenery are transformed into arena
coordinates; shared equipment stays local and uses explicit placements.

`inspect_assets` loads 599,734 ground triangles, 688,031 fixture triangles and
326,714 equipment triangles for this candidate. Their sum includes the
simulator's 48 perimeter triangles in addition to the source collision count.

## Run and inspect

Loading 2027 CAD requires `season-2027`. The unbound rest-pose package still uses
the legacy practice host and needs a combined build. A bound package selects
its explicit 2027 ruleset and supports a 2027-only or combined build.

```sh
nix develop -c cargo run --locked -p rm-simulator-server \
  --features season-2027 --example inspect_assets -- local-assets/field-2027

nix develop -c cargo run --locked -p rm-simulator-app --features season-2027 -- \
  --cad-assets local-assets/field-2027 --fly --start-paused
```

Omit `--fly` to explore using the existing training chassis. These models are
physics tools, not a claim of 2027 robot construction compliance. Match controls
cannot start a match on an unbound rest-pose package. Imported colors are kept without
team recoloring or the legacy scenery override. Prediction uses the normal
matching-package hashes and the same collision geometry.

For reproducible captures, add `--window-mode headless --console`, then use
[the app console](console.md) to set the camera and request screenshots. `C`
(or the console's inspection command) can overlay the actual collision meshes.
The HUD uses the complete 2027 equipment roster; unsupported systems remain
labelled `NO SIM`.

Before promoting this candidate to the bundled field or binding animation,
visually check the full arena, central rune/core separation, both outposts,
bases, tunnels, ramps, and team-side colors. Review source-derived detector
planes and individual moving parts separately when those are added.

## Roster and zone inspection

The map selects the complete [2027 equipment roster](robot-equipment.md#seasonal-rmuc-rosters).
Run the sandbox with inspection overlays:

```sh
cargo run -p rm-simulator-app --features season-2027 -- --cad-assets local-assets/field-2027 --fly --zones --rfid-zones --zone-labels
```

F3 exposes **Buff-zone volumes**, **RFID detection areas**, and **Zone
names / traversal order**. The console `inspect` command accepts `zones`,
`rfid_zones` and `zone_labels` booleans. Overlays work in both seasons and are
client-only: they send no referee commands.

The 2027 catalog covers supply, base, Fortress, vantage, assembly, outpost,
central highland, lower highland, both road pads and all nine tunnel areas per
side. Central-highland RFID also acts as the upper highland traversal pad.
Labels identify low/high or end/interior/end order and the 3 s / 5 s windows.
Green outlines identify functional RFID regions; volumes add a 0.45 m inspection
height band. X-ray drawing keeps under-deck areas visible. The outpost mechanism
vicinity has a separate colored outline; its control activation comes from the
connected highland region, not a fabricated extra RFID card.

Footprints were digitized from UC V1.0.0 **Figure 5-23**, registered to the CAD's
bases and outposts, with roughly ±0.10 m horizontal precision and approximate
surface heights. Three hidden tunnel interiors per side are inferred between
visible ends and explicitly labelled. **Figure 5-24** specifies embedded module
card spacing and dead zones, not a surveyed detection map. The overlay shows
functional areas, not every embedded card or a physical RF field. On an unbound package these are inspection-only shapes. On a bound package,
base, Fortress, central-highland and outpost footprints supply approximate
body/antenna contacts to the native 2027 adapter. They never populate the 2026
`FieldConfig::zones`. Terrain traversal, supply, assembly and vantage overlays
remain inspection-only; embedded-card dead zones and RF propagation are not
simulated.

## Enable the live control/structure subset

Generate a separate candidate from the pinned split source. Keep the original
sandbox intact; the command refuses an existing output directory:

```sh
../rm-map-tools/ocpenv/bin/python scripts/bind-rmuc2027-structures.py \
  --package local-assets/field-2027 \
  --split ../rm-map-tools/out/rm2027/RMUC2027_V1.0.0/split \
  --map-tools ../rm-map-tools --out local-assets/field-2027-live

nix develop -c cargo run --locked -p rm-simulator-app --features season-2027 -- \
  --cad-assets local-assets/field-2027-live --zones --rfid-zones
```

F5 starts the simulator's five-second countdown, followed by the seven-minute
running phase. F6 pauses, F7 steps. Remote match controls retain the normal
referee permission checks. Start again after Finish resets the round. Operator
`ReviveRobot`/`SetRobotHp` remain available; normal respawn now runs automatically.
Practice recovery/reset commands cannot bypass a running round's respawn rules. Use `--fly` for a referee camera or omit it to drive Infantry 3/4
or the manual Sentry. Both hosts and clients must use the same bound package.
A 2027-only build uses `--no-default-features --features season-2027` before
Cargo's `--` separator.

The binding sidecar is named and SHA-256-pinned by `manifest.json`; it identifies
**RMUC 2027 V1.0.0**, not RMUL or an unspecified season. Loading validates its
source hash, detector poses, dimensions, axis vectors and joint contracts.
Prediction uses the same package fingerprint and authoritative restored rules.

### RFID and control

The world samples chassis body centres at 128 Hz against the functional polygon
and its 0.45 m height band. This is an explicit antenna approximation, using the
reviewed diagram footprints. Band floors are calibrated from the verified ground
at footprint vertices, edge midpoints and centre, selecting plausible surfaces
near the diagram's level and adding 10 mm tolerance. This retains sloped edges
without selecting a tunnel below an elevated point. A deck above/below a
point cannot activate it merely by sharing x/y. Each eligible ground robot has
its own two-second expiry for each point; pause freezes that clock. Death,
offline status and departure remove eligibility immediately.

Central-highland activation is mutually exclusive between teams. The first
eligible side keeps its latch through expiry; simultaneous first detections use
ascending chassis id as a deterministic simulation tie-break. Different points
in one territory can still contest. Outpost RFID requires full control of its
territory. Own Fortress requires loss of own outpost-territory control, allows
only the first own robot, and opposing Fortress also requires three minutes and
control of that side's outpost territory (§5.6.4). Opposing Fortress occupation
opens base protective armor after ten seconds, retaining paused progress for
three seconds after death/expiry.

The existing five-second capture/rollback engine settles control before a
coincident whole-second award. **Temporary control earns points**, as requested:
both outposts give 1/s, both bases 3/s. The HUD shows points, each territory's
full/temporary controller, progress, activation flags, contests and Fortress
progress. RFID outlines turn red/blue while active and amber when contested.

### Structures, damage and motion

- UC §5.6.1 starts bases at 10,000 HP without the 2026 virtual shield and
  outposts at 1,000 HP. Full control of one's original outpost territory gives
  the corresponding base 100% defense. Temporary control does not.
- Calibrated 17 mm projectile contacts apply **20 HP**, including the centre and
  every bound base plate (UC Table 5-2). The 2026 centre bonus and upper-front
  5 HP exception are not used. Detection retains UC §5.1.1's greater-than-12 m/s
  normal speed and 50 ms per-module interval.
- Destruction reports to the control engine. A completed physical ownership
  transfer rebuilds the outpost at **500 HP**; owner, HP, color and damage
  attribution remain distinct from original field side and temporary control.
- Both rotors use one host-randomized direction per round and a five-second
  ramp to 0.8π rad/s. First destruction, opposing current owner's base armor
  opening or three minutes permanently stops that rotor. Rebuild never restarts
  it. The source towers tilt about 1.5°; drawing, detector faces and collider
  motion share the fitted three-dimensional axis.
- Base destruction compares remaining base HP immediately. At timeout compare
  accumulated control, attack damage actually applied to enemy HP, then remaining
  robot HP, then draw (UC §5.8). Physical contacts are applied in their stable
  per-tick contact order; simultaneous destruction is not batched.

The exporter identifies every source solid by a unique geometry/face-count
match and preserves the remaining surface model with its exact face count.
It tessellates all source faces at 1 mm / 0.35 rad and records source-part and
script hashes. All 43 outpost bodies and 27 base bodies are retained. Fifteen
outpost armor-module bodies rotate; the shared tower frame stays fixed. The
three base protective-armor bodies translate, with matching collision geometry.
No old reconstructed armor or artwork is copied.

**Binding approximations requiring visual review:** the active dot-matrix mask
is not supplied, so detector rectangles use the source modules' projected
front envelopes, offset 1.5 mm outside the skin. Only the three source-present
base armor modules are bound; missing lower modules and dart detectors are not
invented. Base shield travel is an **illustrative 0.25 m** displacement, without
a claimed official end stop or travel duration. Plain colored detector panels
are an inspection visual, not final 2027 artwork. The regenerated source assets
have 405,623 triangles per outpost and 487,164 per base before instancing;
`structure-binding-build.json` records the new counts separately from the
original sandbox adapter's report.

### Simplified bound field

Use the existing rm-map-tools meshoptimizer pipeline after source-body binding:

```sh
../rm-map-tools/ocpenv/bin/python scripts/optimize-field.py \
  --map-tools ../rm-map-tools \
  --input local-assets/field-2027-live \
  --output local-assets/field-2027-optimized \
  --settings scripts/field-detail-rmuc2027.json

nix develop -c cargo run --locked -p rm-simulator-server \
  --features season-2027 --example inspect_assets -- local-assets/field-2027-optimized
nix develop -c cargo run --locked -p rm-simulator-server \
  --features season-2027 --example compare_asset_ground -- \
  local-assets/field-2027-live local-assets/field-2027-optimized
```

The input must be the unsimplified bound checkpoint; after installing a
derivative, use the retained backup as input to reproduce it. Output must be a
new directory. The settings enable only base, outpost and arena-static. Floor,
rune and tech cores remain byte-identical. All moving bodies and detector
modules are protected in both visual and collision GLBs. The builder validates
binding checksums and complete source-body coverage, checks retained primitives'
positions, normals and indices exactly, and preserves node hierarchy, materials,
joint metadata, detector poses and placements. The custom 2027 sidecar remains
byte-identical; it does not use 2026 articulation-role protection.

Fixed geometry uses 3 mm visual / 5 mm collision error settings and 4 mm / 7 mm
sampled-deviation limits against the input checkpoint. Borders may collapse,
using the existing connected-component retention/refinement policy and 1,024
triangle samples per direction per primitive. These sampled checks are not
certified maximum errors against CAD. Upstream approximation records remain in
`field-detail-build.json`, alongside input manifests, settings, helper hashes,
protected selectors and per-primitive results. No surfaces are classified as
decoration and removed. The minimap is regenerated from the derivative.

Measured results for the bound V1.0.0 checkpoint (triangles per copy):

| Asset | Copies | Visual before → after | Collision before → after |
|---|---:|---:|---:|
| Base | 2 | 487,164 → 133,960 | 487,164 → 116,932 |
| Outpost | 2 | 405,623 → 87,522 | 405,623 → 76,971 |
| Arena-static | 1 | 758,706 → 264,522 | 596,498 → 195,165 |

The complete placed package falls from **3,207,229 to 1,370,435 visual**
triangles (57% reduction) and **3,045,021 to 1,245,920 collision** triangles
(59% reduction). These inventories include moving bodies. The native loader's
static geometry falls from 2,769,093 to 969,992 triangles, including its perimeter.
Its resulting ground/fixture/equipment counts are 198,401 / 529,957 / 241,634.
The ground comparison probes both directions on a 0.2 m grid at five height
bands, retaining lower tunnel floors and upper decks as separate queries. All
111,980 comparisons pass a 10 mm tolerance; the maximum observed height change
is 5.05 mm. This checks sampled ground continuity, not every possible clearance.
Review base/outpost silhouettes, corner dart-station detail and ramp/tunnel
clearances before promoting a derivative into a shared runtime release.

### Source-aware geometry and paint pipeline

The broader pass separates visual precision, collision geometry and artwork.
It starts from the original bound checkpoint, not from the derivative above:

```sh
../rm-map-tools/ocpenv/bin/python scripts/build-rmuc2027-detail.py \
  --map-tools ../rm-map-tools \
  --input local-assets/field-2027-live.before-20261016-geometry \
  --output local-assets/field-2027-detail-ready \
  --recipe scripts/rmuc2027-detail.json
```

Choose a new output directory when repeating the build. The recipe pins both
input manifests and source-painted regions, and fails on a different checkpoint
instead of guessing equivalent selections. The builder retains input manifests,
upstream approximation records, dependency versions and script/helper hashes in
`source-detail-build.json`. These local packages are not the bundled LFS release.

- **Exact representation:** byte-identical complete vertex records are indexed
  together, preserving expanded positions, normals, UVs, normalized attributes,
  triangle order and winding. Hard normal and UV seams are retained. This saves
  storage and vertex work; it is not a claim of lossless triangle decimation.
  A zero-error decimation trial produced negligible triangle savings.
- **Independent parts:** each disconnected component is simplified and sampled
  independently, then reassembled into its original material primitive. Small
  fittings no longer force their entire assembly to use the finest tolerance,
  and component splitting does not multiply render primitives. Every component
  survives; small parts are not removed by a size threshold.
- **Collision surfaces:** source-body nodes and motion hierarchies remain
  separate, but their material partitions are merged before simplification.
  Physics has no paint boundaries. Both fixed and moving housing geometry can
  simplify; detector modules remain exact in the visual mesh, and the complete
  rule/detector/motion sidecar stays byte-identical. Collision housings are
  approximate, independently of those scoring definitions.
- **Shape fitting:** connected components at most 50 mm across may become
  oriented boxes only after bidirectional surface checks stay within 0.5 mm.
  Source sampling covers every triangle; candidate box faces also use a 2 mm
  sampling grid. Open frames and large housings do not qualify. Accepted fits
  and their measurements are individually recorded; they currently provide
  much less reduction than component-wise mesh processing.
- **Paint:** the recipe identifies planar white lettering/logos and red/blue
  pad/boundary regions by source node and material, rather than interpreting
  arbitrary white or colored mechanical surfaces as decoration. Original caps
  remain as solid structural backing with the reviewed background material.
  Detailed text/logos are baked with rm-map-tools at 1,024 pixels/m into embedded
  transparent atlases. Sparse colored lines retain their original few polygons,
  avoiding large transparent texture allocations and retaining their exact
  shape. Oversized atlas regions are clipped into tiles without filling holes;
  source plane thickness may be up to 1.5 mm. Fixed artwork is batched by material
  under a separate `rmuc2027.decoration` node, offset
  0.5 mm, and never added to collision GLBs. Unselected curved artwork and
  mechanical colors remain source geometry. The tactical minimap is baked from
  source-painted geometry before conversion, with its input hashes and shared
  coordinate frame retained, so transparent atlas conversion cannot erase map
  markings.

Structural error settings are 2 mm visual / 3 mm collision, with 3 mm / 4.5 mm
sampled limits per component. Box fitting precedes collision simplification;
its separate 0.5 mm allowance must also be considered. These are sampled checks
against the bound input, not certified maximum deviations from the original CAD.
Two source-pinned thin-frame components in the opposite dart stations remain
exact: their steep faces amplified small remeshing errors in vertical ground
queries. Missing or changed precision components fail the build.

The reviewed recipe covers 24 painted source regions: 14 atlas selections and
10 geometry selections. It produces three 2,048² atlas pages and five batched
decoration primitives in total, rather than one draw primitive per patch.
The complete placed package contains **755,677 visual / 501,398 collision
triangles**: 76% / 84% below the original bound source, and 45% / 60% below
the first derivative above. Its static loader counts are 287,488 ground,
118,924 fixture and 82,134 equipment triangles. All 111,980 ground probes pass
the unchanged 10 mm threshold; maximum observed height difference is 6.72 mm.
The tighter per-component limits can retain more scenery triangles than the
first pass: arena-static has 319,691 visual / 284,252 collision triangles.
Most additional savings come from the bases, moving housings, rune and cores,
with a finer structural error policy than the first pass. Vertex packing saves
additional bytes without claiming further triangle reductions.

Validate and install with a rollback copy:

```sh
nix develop -c cargo run --locked -p rm-simulator-server \
  --features season-2027 --example inspect_assets -- local-assets/field-2027-detail-ready
nix develop -c cargo run --locked -p rm-simulator-server \
  --features season-2027 --example compare_asset_ground -- \
  local-assets/field-2027-live.before-20261016-geometry local-assets/field-2027-detail-ready
python3 scripts/install-field-detail.py \
  --candidate local-assets/field-2027-detail-ready \
  --destination local-assets/field-2027-live \
  --backup local-assets/field-2027-live.before-source-detail-20261016 \
  --build-report source-detail-build.json \
  --inspector target/nix/debug/examples/inspect_assets
```


### Read-only base visibility audit

Before deleting internal geometry, audit the approved high-fidelity base rather
than increasing its simplification tolerance. This command reads the package
and writes measurements and triangle masks to a new directory; it never changes
runtime assets:

```sh
../rm-map-tools/ocpenv/bin/python scripts/analyze-structure-visibility.py \
  --cad-assets local-assets/field-2027-live \
  --map-tools ../rm-map-tools \
  --output local-assets/rmuc2027-base-visibility
```

The audit treats surfaces as opaque and two-sided. It starts with triangle
centroids, both face-normal directions and 32 spherical directions. Candidates
are retested at four barycentric points with 128 spherical directions. Fixed
bodies are tested with all moving shields omitted as occluders; moving bodies
use self-occlusion only. Thus closed shields cannot authorize deleting fixed
detail behind them, and untested combinations of shield positions are not
assumed safe. The floor, arena and other equipment are omitted, deliberately
retaining even underside surfaces that an in-game camera normally cannot see.

For the restored source-aware package, the sampled counts **per base** are:

| Region | Hidden visual / total | Hidden collision / total |
|---|---:|---:|
| Three detector bodies (`4099408`, `4121375`, `4143351`) | 15,011 / 21,420 | 1,043 / 1,608 |
| Top cover (`4268930`) | 5,134 / 11,410 | 2,783 / 5,888 |
| Surface-detail assembly (`4774151`) | 5,210 / 9,820 | 3,723 / 6,631 |
| Main lower housing (`4527988`) | 430 / 15,611 | 1,123 / 6,814 |
| Moving shields (`4314593`, `4353403`, `4392221`) | 508 / 10,270 | 488 / 4,814 |
| Other fittings | 472 / 2,544 | 379 / 2,334 |
| **Total** | **26,765 / 71,075 (37.7%)** | **9,539 / 28,089 (34.0%)** |

These are candidates for inspection, **not validated deletion counts**. The
coarse centroid test alone classified 34,818 visual / 12,925 collision triangles
as hidden; denser sampling exposed many of them. Tiny openings, triangle edges,
curved projectile paths and internal bounce contacts still require validation.
A hidden visual surface is not automatically a removable collider. The hidden
triangles represent about 7.2% of surface area in either mesh, consistent with
many small internal details rather than most of the visible outer envelope.

At the assembled rest pose, the hidden counts increase to 28,698 visual / 10,190
collision triangles. The extra 1,933 / 651 are excluded from the conservative
candidate totals because their occlusion may depend on neighboring moving parts.
The detector candidates include hidden metallic and colored internal faces as
well as dark housings; material color alone is not a valid removal selector.
Scoring definitions and exposed detector optics must remain intact.

A separate source-CAD topology check found ten closed inner shells in body
`4268930` (124 CAD faces). Their bounds match ten closed components in the
current meshes within 0.63 mm: 250 visual / 200 collision triangles, all hidden
in the ray audit. These sealed-cavity components are a stronger initial target
than the broader sampled set, but account for only a small part of it. Another
859 collision triangles are exact coincident duplicates within their source
bodies, ignoring winding because contacts are two-sided. Of those, 715 already
belong to the hidden set; the counts must not be added without accounting for
overlap. These findings were measured against the approved checkpoint; the
interior-removal trial below applies a reviewed subset to a separate derivative.

The output `report.json` records input checksums, source-body counts, areas and
sampling settings. `visual-triangles.npz` and `collision-triangles.npz` retain
triangle positions, body names and masks for diagnostic views. Future removal
should start with source-confirmed enclosed surfaces and exact collision
duplicates, then validate occluded detail in the top cover and surface assembly.
Keep exterior vertices and silhouettes at the approved precision.


### Source armor lighting

The fitted detector rectangles are scoring envelopes, not visible optical masks.
Drawing them as solid emissive panels produced broad red wedges where the
rectangles intersected the source housings, including on the unsimplified CAD.
The renderer now binds the exact detector-body nodes from the source report and
animates only their optical material: source-red for base optics and source-cyan
for outpost optics. Materials are cloned per primitive/instance so team ownership,
strike flashes and disabled state remain independent. Other materials and all
source vertex positions stay unchanged; imported joints carry the lights with
their modules. A source binding suppresses the procedural rectangular panel,
including when the CAD arrives after that fallback was drawn. Rectangular scoring
envelopes and physics are unchanged. Plain panels remain available for targets
without bound CAD surfaces.

### Interior-removal trial

The interior builder starts from the approved source-aware checkpoint, keeps its
exterior precision, and deletes selected triangles without fitting or moving
any retained vertex. It requires the exact reviewed visibility report and masks;
the recipe pins their checksums and both input manifests. Sealed-cavity component
signatures retain their source-CAD matching measurements and source-part hash.

```sh
../rm-map-tools/ocpenv/bin/python scripts/build-rmuc2027-interior.py \
  --input local-assets/field-2027-detail-ready \
  --output local-assets/field-2027-interior-ready \
  --audit local-assets/visibility-review/base \
  --map-tools ../rm-map-tools
```

Use new output/backup names for another build or installation. The reviewed
masks are build inputs from the audit above; a different report, mask or approved
checkpoint fails rather than silently reusing different surface selections.

Visual pruning is restricted to the top cover, surface-detail assembly and dark
detector housings. The main lower housing and moving shields are untouched;
silver/colored detector primitives remain complete. One vertex-adjacent guard
ring protects retained boundaries. A further exterior guard traces 96 spherical
views at 192 × 192 samples, omitting movable occluders. It restored ten triangles
that the earlier audit had missed. Every original first-hit triangle is retained
for these 3,538,944 test rays. This is a sampled safeguard, not a mathematical
visibility proof; before/after camera inspection is still required.

Collision pruning does **not** use the broad visual hidden mask. It removes the
200 triangles in the ten pinned source cavities and 859 exact two-sided duplicate
triangles. These sets do not overlap in this checkpoint. Other hidden collision
surfaces remain until projectile access and contact behavior can be established.
All retained expanded vertex attributes, triangle winding, materials, source-body
names, primitive slots, joint hierarchy and scoring authority stay unchanged.
Unused vertex records are dropped and index widths remain compact. No further
simplification tolerance is introduced.

| Geometry | Approved checkpoint | Interior trial | Reduction |
|---|---:|---:|---:|
| Visual, per base | 71,075 | 60,268 | 15.2% |
| Collision, per base | 28,089 | 27,030 | 3.8% |
| Complete placed visual | 755,677 | 734,063 | 2.9% |
| Complete placed collision | 501,398 | 499,280 | 0.4% |

`interior-build.json` retains input identities, guard-ray counts, restored and
removed triangles, per-primitive results and inherited build provenance. The
asset's `mesh_cleanup` record describes the subset operation separately from
its inherited `mesh_simplification` records, which describe how the retained
source surfaces were tessellated/decimated. Removed inaccessible surfaces are
not claimed to satisfy a full-source bidirectional distance bound.

Native loading succeeds and all 111,980 ground probes against the approved
checkpoint have exactly zero height difference. Validate and install with a
rollback copy:

```sh
target/nix/debug/examples/inspect_assets local-assets/field-2027-interior-ready
target/nix/debug/examples/compare_asset_ground \
  local-assets/field-2027-detail-ready local-assets/field-2027-interior-ready
python3 scripts/install-field-detail.py \
  --candidate local-assets/field-2027-interior-ready \
  --destination local-assets/field-2027-live \
  --backup local-assets/field-2027-live.before-interior-20261016 \
  --build-report interior-build.json \
  --inspector target/nix/debug/examples/inspect_assets
```

This is a playable, single-round RMUC combat/control subset. Heavy, assembly,
ordinary field defense/terrain buffs, rune, aerial, dart and Radar equipment
remain unavailable. 42 mm launches remain refused until Heavy suppression
observations are implemented. Adaptive armor remains in its default damage state.

## Remaining geometry opportunities (2026-10-16)

The approved interior-cleanup package still contains **734,063 placed visual**
and **499,280 packaged collision triangles**. The app confirms the visual total
across 462 CAD mesh entities sharing 324 mesh assets (563,265 unique triangles).
These are full-scene counts before culling and render passes. Sharing meshes
reduces storage; it does not remove triangles from each visible instance.

The read-only complexity inventory verifies manifest checksums and ranks source
nodes, material primitives and connected fragments, including placement counts:

```sh
../rm-map-tools/ocpenv/bin/python scripts/analyze-field-complexity.py \
  --cad-assets local-assets/field-2027-live \
  --map-tools ../rm-map-tools \
  --output local-assets/geometry-opportunities/complexity-review.json \
  --probe outpost visual source_body_2200540 4 \
  --probe outpost visual source_body_2302471 4 \
  --probe outpost visual source_body_2404428 4
```

The output must be new and outside the package. No GLB or manifest is written.
A probe measures a possible derivative; it does not install or approve one.
Selectors refer to this checkpoint's source nodes and primitive order.

| Source selection, all placed copies | Visual triangles | Collision triangles | Next investigation |
|---|---:|---:|---|
| Two dart-station `source_3749800_7` nodes | 136,182 | 163,144 | Reconstruct panels/extrusions selectively; inspect internal assemblies and fasteners. Preserve openings and the precision-pinned ground-contact components. |
| Ten `source_2717102_*` fan-housing nodes | 93,686 | 63,816 | Author simple housing panels, retain functional openings; inspect grille/blade and internal hardware detail separately. |
| Six outpost detector bodies (`2200540`, `2302471`, `2404428`, two outposts) | 92,556 | 2,236 | Narrow exact visual preservation to optical surfaces and required boundaries; simplify mechanical backing independently. |

These three non-overlapping selections account for 43.9% of visual triangles.
The first two account for 45.5% of packaged collision triangles. Their counts
are costs, **not predicted savings**. The first two include assemblies whose
visible form is much simpler than their detailed CAD construction.

The outpost visual preservation policy currently retains entire detector bodies,
including their default-material mechanical parts. Primitive 4 alone accounts
for 77,172 placed triangles across the six bodies. Component-retaining dry runs
on that primitive leave all other primitives, including the source optical
materials, untouched:

| Requested meshoptimizer error | Placed triangles after | Reduction | Worst sampled surface deviation |
|---|---:|---:|---:|
| 0.10 mm | 38,820 | 38,352 | 0.434 mm |
| 0.25 mm | 21,034 | 56,138 | 0.779 mm |
| 0.50 mm | 10,394 | 66,778 | 1.409 mm |

Distances use 2,048 sampled triangles per component in each direction against
this runtime mesh, not the original STEP. They are not certified maximum errors
or visual approval: normal/shading changes, thin walls and all detector poses
still require checks. The 0.25 mm probe alone suggests a 7.6% whole-field visual
reduction without changing the optical primitives, pending that review.

A near-zero-error probe of the red dart-frame dark primitive reduced only
62,229 to 62,155 triangles, yet sampled deviation reached 7.29 mm; its collision
probe reduced 81,769 to 81,721 with 8.29 mm sampled deviation. A small simplifier
metric does not establish lossless geometry or justify another broad tolerance
increase. Targeted source-part cleanup is the stronger approach there.

The inventory also found 8,240 placed coincident collision-face candidates
(5,066 arena, 1,980 rune, 862 outpost, 332 tech core). These match exact vertex
positions within each primitive irrespective of winding; contact behavior and
source semantics must be checked before removal. They represent only 1.65% of
collision geometry and cannot solve the larger cost alone.

Connected material fragments no wider than 10 cm carry 192,564 visual triangles,
but this is not a count of disposable small objects: material boundaries split
otherwise connected solids, optical parts are included, and these fragments
overlap the node selections above. Hidden-geometry sampling and STEP topology
should guide further selection rather than deleting by size.

### Groundwork-only investigation

`arena-static` mixes terrain with mechanisms and resource objects. Counting the
whole file as groundwork overstates the terrain cost. The current floor,
road/deck/ramp surfaces, central terrain/grates, Fortress pads and base foundations
contain **46,536 visual / 26,630 collision triangles**:

| Groundwork selection | Visual | Collision |
|---|---:|---:|
| Floor slab and its markings | 3,236 | 3,236 |
| Two road assemblies, excluding fans | 23,159 | 11,228 |
| Central terrain and grates | 12,612 | 8,244 |
| Side ramps/platforms, excluding dart mechanisms, fans and detector panels | 5,735 | 3,124 |
| Fortress pads | 224 | 128 |
| Base foundations | 1,570 | 670 |

This excludes resource machinery/objects (45,723 visual / 33,402 collision),
dart mechanisms/panels and fans/supports. It also excludes the already separated
240-triangle arena paint node, whose patches span several source selections.
Equipment and resource-object geometry remain unchanged.

The strongest measured groundwork opportunity is the pair of wavy-road faces.
Source products **3797438** and **3813260** each have an imported face **289**
whose surface is a BSpline profile swept along a constant extrusion direction.
At 2 mm linear / 0.5 rad angular settings, general OCCT tessellation produces
34,128 and 34,052 triangles on these single faces. A profile strip needs just
**506 triangles per face** at a 0.25 mm point-to-segment selection tolerance.
An independent 32,770-point, same-parameter interpolation check measures about
**0.327 mm** maximum deviation from each source profile, preserving its waves
and cross-slope. This is sampled source-CAD evidence, not a certified error bound.

Repeat the read-only source probe (the blue source part has SHA-256
`211703e82bfaa9423d6f664bed65f029c8fa265730945fd17d0efe94a5cb81a2`):

```sh
../rm-map-tools/ocpenv/bin/python scripts/probe-groundwork-extrusion.py \
  --source-part ../rm-map-tools/out/rm2027/RMUC2027_V1.0.0/split/parts/0011-3797438.stp \
  --source-sha256 89efb6f0aee8c4518739c112798426b155b28a495892f07d8919476189a2991d \
  --face-index 289 --map-tools ../rm-map-tools \
  --output local-assets/groundwork-review/source-profile-review.json
```

The tool validates the source hash, face type, four-edge trim and sampled trim
interior. It emits measurements, never a replacement mesh. Rectangular trimming,
edge stitching, adjacent walls, caps, winding and material boundaries still need
exporter validation before adopting a strip.

A provisional runtime-face attribution (source parameter range, centroid distance
within 5 mm and matching normal direction) finds 18,736 visual / 8,917 collision
triangles near those two surfaces. Replacing that region with two 506-triangle
strips suggests **about 17,700 visual / 7,900 collision triangles saved**. That
would bring groundwork to **roughly 29,000 visual / 19,000 collision triangles**,
about 38% / 30% lower. Attribution is not an approved deletion mask, and stitching
may add triangles. Ground-height, wheel traversal, clearance and visual checks
remain required. The installed field has not been modified.

Because groundwork is only 6.3% of the field's visual geometry, this initial
terrain-only reduction would lower the whole-field visual count by about 2.4%.
Further opportunities include floor artwork conversion, source-guided removal
of sealed undersides, and more economical grate bars while retaining the actual
openings. Their additional savings are not yet measured. The broader equipment
costs above remain a separate future pass.

### Analytic bump fitting and uniform fidelity controls

The supplied CAD does **not** match seated semicircular cylinders at the
measured pitch. In a local plane perpendicular to the extrusion, its ten bumps
fit `h(x) = H * (1 - cos(2*pi*x/P)) / 2` with **H = 69.99959 mm** and
**P = 240.64227 mm**. The fit includes baseline, phase and slope terms;
maximum sampled residual is about **0.00021 mm**. A repeated semicircle fit
with the same measured pitch and baseline has radius **71.34 mm** and maximum
sampled residual **24.92 mm** (8.43 mm RMS). These measurements describe this
CAD release, not a ruling on the intended physical road profile. Changing to
semicylinders would change the obstacle, rather than faithfully remeshing it.

The source probe now compares both models and generates a uniform local sine
surface in memory, without exporting or installing a field. Its tolerance is
an analytic interpolation bound: for height H and n equally spaced cells per
period, height error is at most `H*pi^2/(4*n^2)`. Cells are rounded up to a
multiple of four to retain the exact peaks, valleys and quarter-period points.
The source-fit residual remains a separate sampled error.

| Analytic error limit | Cells per bump | Triangles per ten-bump surface, one width strip |
|---|---:|---:|
| 0.10 mm | 44 | 880 |
| 0.25 mm | 28 | 560 |
| 0.50 mm | 20 | 400 |
| 1.00 mm | 16 | 320 |

At 0.25 mm the analytic bound is 0.221 mm. Across-width subdivision is a separate
quality/aspect-ratio control: `--max-strip-width-mm 500` uses five uniform strips
across the 2,084.5 mm width, increasing the surface to 2,800 triangles. The
extrusion is linear across this axis, so these subdivisions do not reduce shape
error; their cost must be chosen independently from profile accuracy. Omitting
the option uses one strip. Both roads have the same fitted parameters within
the sampled residuals. The generator supplies upward-wound triangles in a local
millimetre frame; runtime binding, boundaries and adjoining faces remain future
exporter work.

## Combat lifecycle and adaptive-armor infrastructure

Native 2027 tables supply Infantry HP, power reporting, heat limit and cooling
for levels 1–10; the active cap starts at **five**. Assembly-driven cap increases
remain separate. Defaults are HP chassis and cooling launcher. Existing performance
selection commands use the shared choice names with the 2027 tables; selecting
one changed component during Running locks that component, leaving the other
available. Confirming both unchanged choices locks both. Combat launches,
actual damage and kills grant XP, with Table 5-14 distribution for unidentified,
friendly or ineligible sources. Level-up adds the maximum-HP increase immediately;
a capped Infantry stops collecting XP. Terrain/rune/dart XP awaits those systems.
Semi-automatic Sentry retains its Table 3-6 preset; full Sentry posture rules
are not enabled. Chassis referee power limits and the 60 J buffer remain outside
this change; reported performance does not alter the separate drivetrain budget.

The native host advertises a 25 m/s 17 mm default/cap (Tables 3-5–3-6),
including random speed variation, instead of the legacy 30 m/s configurable
cap. Each accepted launch consumes allowance and adds heat. Cooling settles at 10 Hz.
Exceeding the ordinary heat limit locks the launcher and blacks out the
first-person scene until heat reaches zero; HUD, minimap and inspection camera
remain available. **Permanent heat lock follows Figure 5-1's inclusive `>=`
threshold**, as requested by the user, rather than the conflicting prose `>`.
Heat, temporary/permanent overspeed and speed-module connectivity locks remain
independent. Death resets heat, but never removes a round-long lock. Host cadence,
weapon configuration and ownership checks still precede these rules.

Infantry starts with zero ammunition allowance; Sentry starts with 300 and can
collect accumulated 100-round grants for each completed minute at own supply.
**O / Buy ammo** buys ten rounds immediately at an eligible own service/vantage
point, or buys 100 rounds remotely for 150 gold while out of combat, delivered
after six seconds. Both use the team's 1,000-round gold-purchase cap. Zone units
cost ten gold. While the mouse is released, the combat toolbar also provides
**Remote HP** and **Paid respawn**; current gold and costs appear in the HP panel.
These submit ordinary pilot commands, with the host checking chassis ownership.
Remote HP delivers 60% of the contemporary maximum HP after six seconds;
death cancels that HP purchase without refund. Pending ammunition survives death.
Operator allowance/gold overrides remain explicit referee commands.

The initial 800 gold is credited **on entering Running**; subsequent 100-gold
awards occur at elapsed 60, 120, 180, 240 and 300 seconds, and 300 gold at 360
seconds. This is an explicit simulator interpretation of Table 5-6's displayed
countdown labels, not a silently inherited 2026 schedule. Assembly income is absent.
Out of combat requires six live seconds without launch or actual HP loss; round
start defaults to out of combat (Table 2-1).

Own supply and vantage footprints now use the same reviewed, height-calibrated
antenna approximation as control cards. Supply healing is 10% maximum HP/s,
or 25% after four minutes while out of combat. Fully controlled outpost healing
is half that contemporary rate. The base logistics coefficient applies **once**
to both paths and to remote HP recovery. Healing settles at 10 Hz with integer
fractional carry and cumulative nearest-HP rounding. Visual-tag supply activation,
RF propagation, other buff stacking and logistics-induced image degradation are
not implemented. Normal respawn retains XP/allowance, restores 10% HP and gives
weakness plus 30-second invincibility. Service-card recovery clears weakness
but preserves at least ten seconds of that invincibility. Paid respawn restores
full HP, gives three seconds of weakness/invincibility, and reports the four-second
power bonus. Supply or base HP below 4,000 accelerates progress; base damage also
increases its requirement. Offline state suspends progress. A penalty override
can still damage an invincible robot; ordinary hits and collisions cannot.

`gameplay::rmuc_2027::adaptive` retains accepted launch totals and detected armor
contacts, including contacts dealing zero damage. Observations identify the
attacker when known, victim ownership at the event, target/plate, and detector
bottom-left/right/up coordinates and dimensions quantized to **1 µm**. This uses
fitted source detector envelopes for structures and the current Infantry/Sentry
prototype rectangle; final 2027 detector masks remain pending. Friendly and
unattributed contacts stay in history but do not invent enemy-hit attribution.
Snapshots/checkpoints preserve these records, cooling state, choices, XP,
respawn progress, delivery deadlines and per-robot service expiry (**protocol 54**).

Adaptive statistics expose reproducible coordinate medians, median spread,
accuracy and formula-defined area share when both sides are non-singular. Each
minute grants one accumulating reinforcement opportunity; the ten-hit prerequisite
is observable. No Radar selection or reinforced/unreinforced damage is enabled.
No-launch, no-hit and zero-spread cases return an explicit undefined result;
clipping, circle-boundary and area-update policies await a later implementation.
