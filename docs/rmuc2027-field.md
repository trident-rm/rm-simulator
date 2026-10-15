<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# RMUC 2027 runtime field

The RMUC 2027 V1.0.0 CAD can be loaded using the simulator's existing runtime
package layout: `manifest.json`, floor and arena GLBs, rune and outpost GLBs,
and `equipment/manifest.json` with base and tech-core GLBs. The 2026 package
remains separate and stays the default.

The initial 2027 package supports source-colored scenery, complete static
collision, free-camera inspection and chassis exploration. It is explicitly a
**CAD rest-pose sandbox**. It has no live referee, scoring targets, moving
mechanisms or 2026 buff zones. The independent 2027 control component is not yet
connected to the field. This keeps source geometry available for inspection
before animation and detector bindings are verified.

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

Use a combined build: the current session/physics host uses `season-2026`, and
loading the new CAD requires `season-2027`. This does not run a 2026 referee on
the new map. A 2027-only build still cannot open a full application session.

```sh
nix develop -c cargo run --locked -p rm-simulator-server \
  --features season-2027 --example inspect_assets -- local-assets/field-2027

nix develop -c cargo run --locked -p rm-simulator-app --features season-2027 -- \
  --cad-assets local-assets/field-2027 --fly --start-paused
```

Omit `--fly` to explore using the existing training chassis. These models are
physics tools, not a claim of 2027 robot construction compliance. Match controls
cannot start a referee on a rest-pose package. Imported colors are kept without
team recoloring or the legacy scenery override. Prediction uses the normal
matching-package hashes and the same collision geometry.

For reproducible captures, add `--window-mode headless --console`, then use
[the app console](console.md) to set the camera and request screenshots. `C`
(or the console's inspection command) can overlay the actual collision meshes.
The existing practice HUD still uses its 2026 roster labels; it does not
represent a 2027 match roster.

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
functional areas, not every embedded card or a physical RF field. These debug
shapes deliberately do not populate `FieldConfig::zones` or drive capture,
traversal, buffs or RFID expiry. Those require reviewed detector bindings.
