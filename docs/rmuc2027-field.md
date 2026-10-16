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

This is a playable, single-round RMUC combat/control subset. Heavy, assembly,
ordinary field defense/terrain buffs, rune, aerial, dart and Radar equipment
remain unavailable. 42 mm launches remain refused until Heavy suppression
observations are implemented. Adaptive armor remains in its default damage state.

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
