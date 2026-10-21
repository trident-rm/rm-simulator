<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# RoboMaster 2027 rules reference

Engineering interpretation of the three Chinese V1.0.0 manuals released on
2026-10-15. This is a reference for implementation, not a claim that the simulator
implements the full 2027 season. The [control component](gameplay.md#rmuc-2027-control-component) now drives a
[source-bound RMUC 2027 control/structure subset](rmuc2027-field.md#enable-the-live-controlstructure-subset)
in every build. Historical 2026 kernels are documented in [referee rules](referee-rules.md) and [gameplay](gameplay.md); they do not select the active runtime.
[Runtime admission](seasons.md) describes the 2027 application and 2026 archive.

The largest changes are the merger of Hero and Engineer into 重装机器人,
adaptive armor, outpost ownership and battlefield control, assembly-driven
progression, and a replacement aerial system. A season flag alone cannot express
the rules: RMUC, RMUL 3v3 and RMUL Infantry have different rosters, fields,
resource systems and victory conditions.

## Contents

- [Sources and citation convention](#sources-and-citation-convention)
- [Existing 2026 references and comparison scope](#existing-2026-references-and-comparison-scope)
- [Separate competition profiles](#separate-competition-profiles)
- [UC roster and field](#uc-roster-and-field)
- [UC damage and equipment limits](#uc-damage-and-equipment-limits)
- [UC adaptive armor](#uc-adaptive-armor)
- [UC Heavy and assembly progression](#uc-heavy-and-assembly-progression)
- [UC Infantry experience and performance](#uc-infantry-experience-and-performance)
- [UC healing, respawn and economy](#uc-healing-respawn-and-economy)
- [UC territory, logistics and victory](#uc-territory-logistics-and-victory)
- [UC rune and field benefits](#uc-rune-and-field-benefits)
- [UC Sentry, aerial, dart and Radar systems](#uc-sentry-aerial-dart-and-radar-systems)
- [UL 3v3 rules](#ul-3v3-rules)
- [UL Infantry 1v1 rules](#ul-infantry-1v1-rules)
- [Robot construction and visual implications](#robot-construction-and-visual-implications)
- [Decisions and missing inputs before implementation](#decisions-and-missing-inputs-before-implementation)
- [Implementation boundaries and migration order](#implementation-boundaries-and-migration-order)

## Sources and citation convention

| Key | Supplied manual | Revision | PDF pages |
|---|---|---|---|
| UC | RoboMaster 2027 机甲大师超级对抗赛比赛规则手册 | V1.0.0, 20261015 | 144 |
| UL | RoboMaster 2027 机甲大师高校联盟赛比赛规则手册 | V1.0.0, 20261015 | 68 |
| ROB | RoboMaster 2027 机甲大师高校系列赛机器人制作规范手册 | V1.0.0, 20261015 | 91 |

Citations below use the manual's **printed page number**; PDF viewer page is
printed page + 1 in all three supplied files. Section and table numbers are
scoped to the source key. Retain the Chinese term when an English translation
could accidentally identify an old robot or mechanism. “Heavy” below means
重装机器人 and is a working translation, not an official English class name.

Exact source fingerprints (SHA-256):

```text
UC   9e2eed9295536c6fd547b62d634df1d5e70a2e1e2250e5dff3521d557e5d54be
UL   a8fd7b7f6ea5212b9e4dfa590b478827992598d91912ed671653c8bdd9727fd6
ROB  3287fe5f521f8801ce69632032ea17a9c8d8a099091558f58618cd7721154a3c
```

The supplied PDFs remain external source material, under DJI's notices; they
are not relicensed or embedded here. Local text extracts and inspection images
belong in ignored `local-assets/rm2027-rules/`. Obtain the matching source files
before relying on figure dimensions; the filenames, versions and fingerprints
above distinguish these releases from subsequent corrections. Text extraction
can corrupt mathematical notation and omits dimension drawings. In particular,
the adaptive-armor area formula below was checked against the rendered page.

This digest prioritizes simulation, equipment and client integration. Inspection,
conduct, penalties, appeals and event administration remain governed by the
manuals, not exhaustively restated here. Later publications and official answers
must be incorporated explicitly; do not silently replace this baseline. UC/UL
§§1.3–1.4 describe amendments and the authority of official answers.

## Existing 2026 references and comparison scope

The repository already has a useful pattern:

- [Referee rules](referee-rules.md): implemented live rules, source citations,
  simulator overrides and assumptions. Baseline: RMUC 2026 V2.1.0, with a limited
  V2.2.0 update for base damage.
- [Gameplay](gameplay.md): standalone engine coverage, integration boundaries,
  and ambiguities. It must not become a mixture of 2026 and 2027 requirements.
- [Robot equipment](robot-equipment.md): current modeled robot geometry and
  artwork assumptions, rather than an inspection certificate.

The comparison below is against those documented simulator rules, not an
exhaustive diff of every edition of the 2026 manuals. Changes expressly announced
by the 2027 manuals are cited to UC/UL §1.2.

| Area | Current documented 2026 behavior | 2027 requirement | Source |
|---|---|---|---|
| Ground roster | Hero, Engineer, Infantry 3/4/5 and Sentry records/prototypes | UC: Heavy 1, Infantry 3/4, Sentry 7 | UC §3, Table 3-1, p.20 |
| Hero and Engineer | Separate classes | Merge into Heavy; manipulation and detachable 42 mm launcher | UC §1.2, §3.1.1, §5.7.1 |
| Base and outpost | Base 5,000 HP plus 150 shield; fixed team; outpost destruction/protection | Base 10,000 HP; outpost 1,000 HP, capture and rebuild for the other team | UC §5.6.1, pp.94–96 |
| Timeout result | Base HP, outposts, attack damage, remaining robot HP | Accumulated control, attack damage, remaining robot HP | UC §5.8, pp.114–115 |
| Armor | Fixed detection faces and damage paths | Hit coordinates, team accuracy statistics and selectable reinforcement circle | UC §5.2, pp.72–73 |
| Progression | Hero and Infantry performance/experience | Only Infantry earns experience; Heavy progresses through assembly; team level caps unlock | UC §§5.4.3, 5.5 |
| Field | 2026 CAD and buff areas | New tunnels, vantage point, hangar platforms, workbench and revised field layout | UC §1.2, chapter 4 |
| Aerial robot | Manually driven, tethered 17 mm prototype | Autonomous miniature aerial robots, hangar, impact attack; no launcher | UC §3.2.1; ROB §2.4.4 |
| Client equipment | Existing custom-client/referee-link integration | Custom controller and client combined as 自定义终端 | UC §3.3.1; ROB §2.1.7 |

## Separate competition profiles

| Property | UC | UL 3v3 | UL Infantry 1v1 |
|---|---|---|---|
| Round duration | 420 s | 300 s | 120 s |
| Nominal field | 28 × 15 m | 12 × 8 m | 5 × 5 m |
| Main ground roster | Heavy 1, Infantry 3/4, Sentry 7 | Heavy 1, Infantry 3, Sentry 7 | Infantry 3 |
| Other systems | Aerial 6, Dart 8, Radar 9 | None in the listed roster | None |
| Main objective | Destroy base; otherwise accumulated control | Reduce opponent's 600 control to zero | Defeat opponent; otherwise HP |
| Progression | Infantry experience; assembly | Fixed class configurations | Fixed configuration |
| Economy | Periodic and assembly income; ammunition, HP and respawn purchases | Periodic/catch-up income; ammunition only | Fixed 200 rounds, no replenishment |

Sources: UC Table 3-1, §4.1, §5.8, §6.6; UL Table 3-1, §3.2.1,
§§3.3.3–3.3.7, §§4.1–4.4. UL §1.2 removes the Engineer challenge event.
Neither UL mode should inherit UC base, rune, assembly or adaptive-armor rules
merely because both use the 2027 season.

## UC roster and field

UC Table 3-1 (p.20) lists one Heavy, two Infantry and one Sentry; four aerial
robots in one hangar; four darts with one launcher; and one Radar system with
compute and sensor components. The minimum first-round lineup is three ground
robots (§3, p.21). Registration and reserve limits are competition administration,
not a reason to change the simulator's connection capacity.

Ground pilots operate Heavy and the two Infantry. A commander operates the
optional aerial/dart systems and Sentry interventions; an aerial safety operator
has restricted emergency control. The former gimbal operator and pilot roles
are removed (UC §1.2, Table 3-2). These competition roles are distinct from the
simulator's host-authorized pilot, spectator and referee network roles.

The field remains centrally symmetric, with a 28 × 15 m footprint and nominal
2 mm PVC surface (UC §4.1, p.29). The manual changes terrain and equipment;
existing 2026 CAD, collision triangles, ownership mapping and buff rectangles
are not a 2027 field package. Principal geometry references are:

| Element | Reference |
|---|---|
| Six base armor modules, dart module, closed/open base armor | UC §4.2.2, Figures 4-10–4-13 |
| Vantage point reached by three steps | UC §4.2.4, Figures 4-17–4-18 |
| Supply zone, energy-unit source and workbench | UC §4.2.6, Figures 4-20–4-24 |
| Central highland and assembly zone | UC §§4.3.2–4.3.2.1, Figures 4-27–4-29 |
| Hangar platforms | UC §4.3.2.2, Figure 4-30 |
| Rune | UC §4.3.2.3, Figures 4-31–4-33 |
| Outpost with three rotating middle armor modules and dart module | UC §4.3.2.4, Figure 4-34 |
| Road, undulations and tunnels | UC §4.4, Figures 4-35–4-38 |
| Energy unit | UC §4.5.1, Figure 4-39 |
| Buff card coverage | UC §5.6.4.1, Figures 5-23–5-24 |

The rune's physical circular target is at most 308 mm across; its effective
detection diameter is 300 mm (UC §4.3.2.3, p.55). An energy unit is an asymmetric
hollow body of approximately 400 ± 50 g (UC §4.5.1). Detailed shape and placement
must come from verified drawings/CAD; do not convert perspective illustrations
into exact collision geometry.

## UC damage and equipment limits

UC §5.1.1 and Tables 5-1–5-2 (pp.64–65): detection uses the **normal component**
of impact velocity, strictly greater than 12 m/s for 17 mm and 8 m/s for 42 mm.
Minimum detection intervals are 50 ms and 200 ms respectively. Rune detection
accepts only 17 mm, above 12 m/s.

| Target | 17 mm | 42 mm | Aerial impact | Collision |
|---|---:|---:|---:|---:|
| Ground robot armor | 20 | 200 | — | 2 |
| Base armor | 20 | 200 | 200 | — |
| Outpost middle armor | 20 | 200 | 200 | — |

Base dart damage is 200/300/800/1,250 for fixed/random-fixed/random-moving/
terminal-moving targets. Outpost dart damage is 1,000. Do not carry the 2026
base upper-front 5 HP value or center-square bonus into this table without a
2027 source. Aerial-impact contact time, force threshold and integrated impulse
criteria are explicitly pending in Table 5-1, so a projectile-like speed
threshold would be an invented rule.

Heavy's 42 mm maximum launch speed is 16.5 m/s; Infantry and Sentry 17 mm is
25 m/s (UC Tables 3-3, 3-5, 3-6). After Heavy has not launched for four seconds,
the opposing robot/base/outpost armor suppresses 42 mm damage until a launch
occurs (§5.1.1). Additional suppression applies after death/offline, overfiring
or launcher-link loss (§5.4.2); this is independent of flight integration.

UC Table 4-1 (p.61) gives projectile physical specifications: 42.5 ± 0.5 mm,
44.5 ± 0.5 g for the nominal 42 mm projectile; 16.8 ± 0.2 mm, 3.2 ± 0.1 g
for nominal 17 mm. These are source tolerances, not a demand to randomize physics.

### Heat, speed and power penalties

UC §5.1.2, Table 5-3: 17 mm overspeed by less than 5 m/s locks for 15 s;
5 to less than 10 m/s locks for 20 s; 10 m/s or more locks for the round.
For 42 mm, above its limit through 18 m/s locks for 15 s; above 18 m/s locks
for the round. Lock causes remain independent.

UC §5.1.3: each 17 mm launch adds 10 heat and each 42 mm launch 100; cooling
settles at 10 Hz. Exceeding the normal heat limit locks the launcher **and cuts
first-person video until heat reaches zero**. The permanent threshold is the
normal limit +100 (17 mm) or +200 (42 mm). The text uses `>` whereas Figure 5-1
uses `>=` at the permanent threshold: the simulator adopts Figure 5-1's inclusive `>=` boundary by user instruction.
Minimap and custom UI remain available during the heat-related video cut.

UC §5.1.4, Figure 5-2: at 10 Hz update the 60 J buffer by
`Z = Z - (P_actual - P_limit) × 0.1 s`, capped at 60 J. Exhaustion causes a
five-second chassis power cut. This referee buffer is separate from the
simulator's drivetrain power budget and the hardware supercapacitor energy.

UC §5.1.5: module connectivity is checked at 2 Hz. Main-controller offline cuts
actuation, damages at 5% maximum HP/s, disables field interaction and
communication, suspends respawn progress, and stops ordinary hit/module damage;
Infantry experience/levels continue. Each offline armor or supercapacitor
management module costs 4 HP per 500 ms. A disconnected speed module locks its
launcher; reconnecting only removes that lock cause. Missing localization makes
enemy Radar observations count as accurate and friendly ones as wrong.

## UC adaptive armor

UC §5.2, Table 5-5 (pp.72–73) applies a team-selected reinforcement region to
robot, base and outpost armor. All plates start in the default state. Each team
gains an accumulating reinforcement opportunity every minute. After the team
has at least ten cumulative projectile hits, its Radar can spend one opportunity
to select or change a common center point.

| Region | Multiplier on raw projectile damage |
|---|---:|
| Default, before reinforcement selection | 1.0 |
| Inside reinforced circle | 0.5 |
| Outside reinforced circle | 1.5 |

This multiplier multiplies attack and defense effects; it is not an alternative
buff to discard under the strongest-buff rule. The plate coordinate system has
origin at the **bottom left of the detection face**, horizontal right as x and
up as y. It is not the simulator's FLU world frame or centered target face frame.

The source defines:

```text
η = opposing armor projectile hits / own projectiles launched
(x0, y0) = coordinate-wise medians of hit positions over all armor plates
σ = median distance of hit positions from (x0, y0)
Λ = η / σ
ρ = Λ_own / Λ_opponent
A = plate_area × ρ² / (2 + ρ²)
```

The common center and area define a circular reinforced region on each plate.
Hit coordinates and region classification are delivered to the victim over the
ordinary referee link and to the attacker over the Radar wireless link.

The native simulator now supports selection and damage classification. Adopted
user policies (not official clarifications): the circle boundary is reinforced;
the circle may extend beyond a plate without resizing; **area freezes when the
center is selected**, until another opportunity is spent. Both the center and
area fraction persist through death, ownership transfers and checkpoints. Each
plate's own detection area determines the circle's radius from that fixed share.

If either team's selection-time accuracy is undefined (no launches, no hits or
zero spread), selection consumes its opportunity and freezes default 1× armor
until reselection. Later statistics neither resize the circle nor turn an
undefined selection into a finite one. Area fraction is rounded once to parts
per billion for exact checkpoint equality; hit positions/centers use micrometres.
Centers use nonnegative bottom-left coordinates; a center may lie outside a
smaller plate. No circle renormalization is performed at the edge.

A referee/console action stands in for Radar selection; playable Radar and
2027 hardware-message compatibility remain unavailable. Protected/immune accepted
hits still enter statistics and retain their historical region classification;
unknown and friendly attackers do not contribute to an opposing-hit count.

## UC Heavy and assembly progression

Heavy is not the old Hero with a renamed label. It starts without permission to
power a detachable 42 mm launcher and combines manipulation, economy and later
combat (UC §§3.1.1, 5.4.3, 5.7.1).

Assembly difficulty unlocks at elapsed 0/60/120/180 s for levels 1/2/3/4, and
the previous difficulty must have been completed before selecting the next
(UC Table 5-11, pp.82–83). Both sides have matching poses at the same difficulty.
Difficulty 4 uses both cores, has a 45 s completion limit after positioning,
and excludes competing assembly attempts under the priority rules in §5.4.3.

| Difficulty | First completion | Repeat completion |
|---|---|---|
| 1 | Heavy level 5; income 40 gold/10 s | Additional 4 gold/10 s |
| 2 | Heavy level 7; team level cap 5 → 7; additional 20 gold/10 s | Additional 8 gold/10 s |
| 3 | Heavy level 10; team level cap → 10; team robots/base/outposts 25% defense through round end; unlock launcher; additional 20 gold/10 s | Additional 12 gold/10 s |
| 4 | Team robots/base/outposts 50% defense through round end; additional 40 gold/10 s | Only once |

Source: UC Table 5-12. Income additions accumulate; the manual's example
1→2→3→2→1 yields 92 gold/10 s. Robots at their current level cap stop acquiring
experience (p.84). The 25% and 50% defense effects follow the strongest-effect
rule, not 75% summed defense.

UC Table 5-10 and Figures 5-6–5-15 define core coordinates and manipulation.
The assembly frame is equipment-local and must be transformed into FLU.
Its initial pose is `(-200, 200, 800) mm`, `(θ, φ, α) = (0, 90, 0)°`.
Assembly positions use x ∈ [-100,0], y ∈ [100,300], z ∈ [500,700] mm.
Difficulty 1 has fixed orientation; levels 2–4 allow θ ∈ [-90,90],
φ ∈ [0,90], α ∈ [-45,45] degrees. Do not substitute these angles directly for
world yaw/pitch/roll. The standalone workbench now generates seeded candidates
with normal `(sinφ cosθ, sinφ sinθ, cosφ)` and an explicit roll-zero convention;
see `docs/assembly-workbench.md` for sampling policies and CAD/IK rejection.
Figures 5-7/5-15 distinguish the insertion post from Q: Q tilts the raised unit
sideways. Figure 5-7 places P 208 mm above the original post axis and 15 mm ahead of the
panel plane (post reach 169 mm); lifting must not translate P. The resulting
flip sweeps forward/up and reproduces the drawing's 54/15 mm final offsets.
These source bindings remain workbench calibration, not live match authority.

The required actions are selecting difficulty while activating the assembly
zone, inserting 100 mm in local −x, translating 100 mm in +z, rotating upward
90° about P, rotating to the target about Q, and confirming. Difficulty 1 uses
steps 1/2/6; level 2 adds step 3; level 3 uses all six. Level 4 requires both
cores to complete corresponding steps within 5 s of each other, except steps 1
and 4. Q alignment tolerance is 5°; the correct-position indication requires
0.3–0.5 s. Failed confirmation ignores new confirmation for one second.

UC §5.4.3 also defines removal/failure, loss of assembly-zone detection for
15 s, recovery, and Heavy death while attached. A Heavy defeated during assembly
retains chassis/gimbal power for up to 15 s to disengage; its launcher is off.
This is a specific exception to ordinary defeated-robot actuation.

### Tech Core indicators

UC §5.4.3, Figure 5-11 and Table 5-13 (printed pp.84–85) specify three
independent groups: the radial angular strips (1), the two vertical side bars
(2), and the small indicators around the insertion mechanism (3). The supplied
2026 English V2.1.0 manual has the same patterns in Figure 5-11 and Table 5-10
(printed pp.88–90).

| Physical state | Radial group 1 | Side group 2 | Insertion group 3 |
|---|---|---|---|
| No assembly in progress | Off | Steady team color | Steady team color |
| Difficulty selected, Core arm positioning | Flowing lights | Steady team color | Steady team color |
| Core positioned, ready to insert | White, 1 Hz | Team color, 1 Hz | Team color, 1 Hz |
| Step 2 complete: inserted | White, 1 Hz | Team color, 1 Hz | Team color, 3 Hz |
| Step 3 complete: lifted | White, 1 Hz | Team color, 3 Hz | Team color, 3 Hz |
| Step 4 complete: raised 90° about P | White, 1 Hz | Steady team color | Steady team color |
| Step 5: rotating Q | Target angle in team color; current angle in white | Steady team color | Steady team color |
| Step 5 complete: alignment accepted | Target angle in green | Steady team color | Steady team color |
| Confirmed, Unit recovery | White, 3 Hz | Team color, 3 Hz | Team color, 3 Hz |

The manual does not define flash duty cycle, phase origin, luminous intensity,
flowing-light speed or overlapping angular-indicator priority. Simulator display
policies use a 50% duty cycle on explicit field time, nearest source strip per
angular marker, and team-target priority if current and target choose the same
strip. A paused simulation freezes the lights. The live prototype samples light angles
and accepted motion from the same presentation clock. Recovery flashing ends when automatic Core discharge and reset finish; that
lifecycle boundary is simulator policy. No special failure pattern is
specified; cancelled/failed attempts return to idle indication.

Live source-CAD assembly binds these effects to accepted manipulation steps,
not planner progress or elapsed animation percentage. A restorable positioning
phase first moves the Core from its previous joint/lift/P/Q state (the source
CAD rest state on first use) into its selected pose with flowing radial lights.
Only arrival releases the robot planner; ready-to-insert lights remain active
while that planner searches. The selected Core positioning branch stays fixed.
The native 45-second attempt begins at arrival. For live L3, the final
Q target remains unavailable through insertion/lift/P flip. The world samples
and reveals it only at the completed flip; target LEDs then guide a separate
attached solve. The initial path never sees that target. Failed late planning
does not resample it or retroactively change the grasp. Confirmed indication is shown after acceptance and throughout the live Core
recovery described below. These effects currently apply to
the articulated source Core loaded for a live assembly, not the untouched
static arena Core or the standalone workbench renderer.

### Successful Core recovery and spent Units

UC §5.4.3 step 6 requires recovery after accepted assembly and grants the
corresponding reward. It does not specify the recovery trajectory, discharge
location or timings. R60 prohibits obtaining, carrying or reusing an already
successfully assembled Unit. The manual's initial Core pose is
OXYZ (−200, 200, 800) mm, θ=0°, φ=90°, α=0° (§5.4.3, after Figure 5-10).
Figure 4-39 describes the Unit as 400±50 g; free-body dynamics use 400 g.

The live simulator waits for the Engineer's validated jaw-release and neutral
return before moving the Core. It retracts to that initial pose, then positions
the entire Core wrist above the recessed collection opening behind the curved
front wall, with the shaft pointing down. The fitted discharge post centre is
OXYZ (−450, 0, 650) mm in the verified Figure 4-29 arena adapter. This is a
source-layout fit and simulator policy, outside the assembly-pose selection
domain, not an official recovery coordinate. The central bay floor is 250 mm
below the standing deck in the source crop. The P hinge is reset to zero;
discharge turns the six-axis Core wrist rather than reversing the P hinge.

A 250 ms stationary hold removes wrist momentum before release. The **same**
Unit then becomes a free Rapier body and falls through the opening under gravity.
After a 500 ms clearance dwell, the Core returns to its initial pose and idle
lights. These delays and the 1 rad/s Core rotary / 0.1 m/s carriage limits are
simulator defaults. Motion, attachment, spent state and progress survive restore.
The Engineer can drive away or disconnect during Core recovery without stopping
it. Recovery reserves the station and does not re-award success or consume the
attempt deadline. Spent Units remain physical scenery and cannot be gripped;
R60's match forfeiture penalty is not automated. Base inventory and recovery of
failed/interrupted attempts remain separate future work. Core travel remains
prescribed, without obstacle/overload response, as documented for positioning.

### Virtual assembly prototype

The live simulator supports referee Start/Finish/Cancel rather than physical
assembly. Start represents positioned cores; Finish certifies the complete
attempt, including both cores at difficulty 4. All difficulties have a 45-second
simulation-time limit in this prototype (only difficulty 4 has this limit in
the manual). Timeout is settled before an action at the exact deadline; failure
and cancellation grant no rewards and free the cores immediately. No physical
recovery delay, contact loss, attached-Heavy death or zone invincibility is
modeled. Income is paid on round-aligned ten-second boundaries before 7:00;
a completion submitted on a boundary affects later payments. UC unlocks,
prerequisites, shared-core priority, one-time level 4, income, Infantry caps and
strongest permanent defense are implemented. Full Heavy level/HP and launcher
installation are deferred. See [gameplay](gameplay.md#rmuc-2027-virtual-assembly).

### Launcher installation and Heavy properties

UC §5.7.1: launcher permission unlocks by difficulty 3 **or** after elapsed time

```text
300 - M / 50 - 2N / 3 seconds
M = maximum observed (opponent base HP - own base HP) during the round
N = maximum observed (opponent control - own control) during the round
```

These are historical maxima, not current deficits. The manual gives no explicit
clamp for a negative threshold. Installation/removal and splitting/recombining
subsystems are restricted to the workbench. Installation is confirmed by a
detected 42 mm launch; that confirmation shot does not count heat, speed or
damage. The launcher stays unpowered before permission unlocks.

| State | Maximum HP | Power limit W | Heat limit | Cooling /s | Speed m/s |
|---|---:|---:|---:|---:|---:|
| Initial | 400 | 50 | — | — | — |
| First difficulty 1 complete | 450 | 50 | — | — | — |
| First difficulty 2 complete | 500 | 50 | — | — | — |
| Difficulty 3 complete or launcher successfully installed | 600 | 50 | 300 | 100 | 16.5 |

Source: UC Table 5-16, pp.91–92. During the first three minutes, before successful
installation, Heavy has 50% defense (§5.7.1). A Heavy activating its supply zone
is invincible (§5.6.4.8). Assembly-zone invincibility has a separate cumulative
45 s allowance refreshed when the highest completed assembly level increases
(§5.6.4.7).

## UC Infantry experience and performance

Only Infantry earns experience (UC §5.5.1). Launching earns 1 per projectile;
robot damage earns 4 per HP, outpost armor damage 2 per HP and base armor damage
1 per 2 HP, rounded up for odd damage. Kill rewards depend on victim level and
positive level difference; unidentified/ineligible sources use the distribution
rules in Table 5-14. First acquisition of each listed road/highland/tunnel/
vantage-point benefit earns 300 experience. Rune and dart awards are separate.

Table 5-15 cumulative thresholds for levels 1–10 are:
`0, 550, 1100, 1650, 2200, 2750, 3300, 3850, 4400, 5000`.
Assembly controls the team's level cap. The Infantry chooses chassis and launcher
performance independently; defaults are HP priority and cooling priority.
Choices lock once selected after the round starts; leveling immediately adds the
increase in maximum HP to current HP (§5.5.2).

| Level | Power chassis HP/W | HP chassis HP/W | Burst heat/cooling per s | Cooling heat/cooling per s |
|---|---|---|---|---|
| 1 | 150/60 | 200/45 | 170/5 | 40/12 |
| 2 | 175/65 | 225/50 | 180/7 | 48/14 |
| 3 | 200/70 | 250/55 | 190/9 | 56/16 |
| 4 | 225/75 | 275/60 | 200/11 | 64/18 |
| 5 | 250/80 | 300/65 | 210/12 | 72/20 |
| 6 | 275/85 | 325/70 | 220/13 | 80/22 |
| 7 | 300/90 | 350/75 | 230/14 | 88/24 |
| 8 | 325/95 | 375/80 | 240/16 | 96/26 |
| 9 | 350/100 | 400/90 | 250/18 | 104/28 |
| 10 | 400/100 | 400/100 | 260/20 | 120/30 |

Source: UC Tables 5-17–5-18, pp.92–94. These tables are reference data, not changes
to the existing 2026 presets.

## UC healing, respawn and economy

UC Table 2-1 defines out of combat as alive with no launch and no HP loss for
six continuous seconds; robots start out of combat. This gates remote purchases.

Supply-zone healing is 10% maximum HP/s, increasing to 25%/s after four minutes
while out of combat; leaving that state immediately removes the stronger rate.
An outpost zone in fully controlled territory heals at half the contemporary
supply rate. Remote HP purchase delivers 60% maximum HP after 6 s; death during
the delay invalidates it without refund (UC §5.3.1).

Death resets heat to zero and the power buffer to full. Normal respawn progress
length is `round(10 + elapsed_seconds / 10 + 20 × prior_paid_respawns)`.
It advances at 1/s, or 4/s in the supply zone or while own base HP is below 4,000.
Normal respawn restores 10% HP and grants weakness plus 30 s invincibility.
Detecting an eligible outpost/base/supply card clears weakness, but respawn
invincibility lasts at least 10 s. Paid respawn restores full HP, grants 3 s
weakness/invincibility and doubles the power limit for 4 s, capped at 200 W.
Weakness locks the launcher and prevents ordinary buff benefits (UC §5.3.2).
The supply/weakness interaction and exact timing are worth testing explicitly.

UC Tables 5-6–5-9:

| Transaction | Cost and delivery | Team purchase cap |
|---|---|---|
| Periodic income | Countdown 6:59 → 800; 5:59, 4:59, 3:59, 2:59, 1:59 → 100 each; 0:59 → 300 | — |
| Zone 17 mm | 10 gold / 10 rounds | 1,000 rounds |
| Zone 42 mm | 10 gold / 1 round | 100 rounds |
| Remote 17 mm | 150 gold / 100 rounds; 6 s delay | Same purchase cap |
| Remote 42 mm | 150 gold / 10 rounds; 6 s delay | Same purchase cap |
| Remote HP | `50 + ceil(elapsed_seconds / 60 × 20)` gold | No count limit |
| Instant respawn | `ceil(elapsed_seconds / 60) × 80 + robot_level × 20` gold | No count limit |

The table preserves displayed countdown labels. UC Table 2-1's time convention
(p.19) means a displayed 6:59 initial award must not be silently changed to the
old engine's elapsed-one-second schedule without deciding clock boundaries.
Initial allowance is zero for Heavy and Infantry; Sentry starts with 300.
Each elapsed minute adds 100 collectable Sentry rounds at supply, accumulating
until collected (UC Tables 3-3, 3-5, 3-6; §5.4.2).

42 mm damage suppression also starts three seconds after Heavy dies/goes
offline, on overfiring at zero allowance, after the third post-death launch,
or at launcher/controller packet loss ≥5%; the specified recovery requires
Heavy alive with positive allowance (§5.4.2). Do not apply it to 17 mm damage.

## UC territory, logistics and victory

UC §5.6.1 sets base HP to 10,000 and outpost HP to 1,000. Fully controlling one's
own outpost territory grants one's base 100% defense; otherwise that protection
is lost. An outpost destroyed to change full control rebuilds for its new owner
with 500 HP. Ownership is mutable; array index or arena half cannot remain the
only source of its team color, damage attribution or benefits.

There are two territories per initial side (UC §5.6.2, Tables 5-20–5-21):

- Outpost territory comprises its outpost zone and central-highland zone and
  supports temporary and full control.
- Base territory comprises its base zone and Fortress zone; only its initial
  owner can fully control it.

Initially each side fully controls its own two territories. Continuously
activating either constituent zone for 5 s converts opponent full control to
one's temporary control. With no opposing activation for 5 s, an opponent's
temporary control converts back to one's full control. Incomplete progress
recedes at the same speed instead of resetting, and opposing simultaneous
activation pauses progress. Destroying an outpost converts an opponent's full
outpost territory to their temporary control, or one's temporary outpost
territory to one's full control, according to Table 5-21. That distinction must
survive snapshots rather than be reduced to a single owner bit.

Controlling both outpost territories adds 1 control/s; controlling both base
territories adds 3 control/s (p.97). The sentence says “control” without
qualifying temporary/full. The implemented component counts both, per the user
instruction; this is the adopted interpretation rather than a later official
clarification. Base protection separately requires full control.

Base damage also reduces logistics. With `Δ = lost base HP`,
`λ = 1 - Δ / 20000` (UC Table 5-19):

| Property | Adjusted value |
|---|---|
| Required respawn progress | Original / λ |
| Supply healing rate | Original × λ |
| Remote HP recovery fraction | Original × λ |
| Robot interaction bandwidth | Original × λ |
| Video resolution by pixel area | Original 1920 × 1080 area × λ² |

The specified image process is area-average downsampling followed by bilinear
upscaling; player UI clarity is unchanged. Whether the outpost's derived healing
rate receives the logistics coefficient once or twice must be made explicit.

Outpost rotors reach 0.8π rad/s within 5 s, with one shared random direction
for both initial outposts. They stop permanently for the round on first
destruction, opposing base armor opening, or after 3 min (UC §5.6.1). Rebuilding
under a new owner must not restart the rotor.

UC §5.8 decides a round in order: on base destruction, higher remaining base HP;
at timeout with both bases surviving, higher accumulated control; then team
attack damage; then total remaining robot HP; otherwise draw. Surviving base HP
and surviving outpost count are no longer timeout tiebreakers. UC uses BO2/BO3
groups and BO3/BO5 eliminations; tied BO3/BO5 rounds require another round.
The phase durations remain 180 s preparation, 15 s self-check, 5 s countdown,
420 s running (UC §§6.3–6.6).

## UC rune and field benefits

UC §5.6.3 retains the small stage until 3 min and big stage thereafter.
Small opportunities arrive at 0 and 90 s; big opportunities at 180, 255 and
330 s. Opportunities accumulate. Activation attempts last 20 s. Small and
nonactivating big runes turn at π/3 rad/s; activating big uses
`a sin(ωt) + b`, `a ∈ [0.780,1.045]`, `ω ∈ [1.884,2.000]`, `b = 2.090 - a`,
with parameters/time reset for each attempt and up to 500 ms target-speed error.

Small activation chooses one lit target at a time. Big chooses two; hit the
first within 2.5 s, then optionally the second within 1 s, for five groups.
Wrong-target hits or missing the 2.5 s deadline reset activation progress.
Between groups the lamps may go dark for up to 200 ms (§5.6.3.1).

Small activation grants 25% defense for 45 s to robots, base and outposts;
Infantry receives 100% extra experience during it, capped at 1,200 extra per
team per activation. Big activation divides 750 experience among surviving
Infantry and supplies the following effects (Tables 5-22–5-24):

| Mean ring | Attack multiplier | Defense | Cooling multiplier |
|---|---:|---:|---:|
| [1,3] | 1.5 | 25% | None |
| (3,7] | 1.5 | 25% | 2 |
| (7,8] | 2 | 25% | 2 |
| (8,9] | 2 | 25% | 3 |
| (9,10] | 3 | 50% | 5 |

Five/six/seven/eight/nine/ten lit arms give 30/35/40/45/50/60 s.
After the first successful big activation, only rings 4–10 detect; after the
second, only rings 7–10. The manual names four states, including activation
failure; do not assume the existing three-state representation expresses every
2027 visual transition. It still does not define the simulator's hit-flash time.

Native Rune implementation policy: first-hit and optional-hit deadlines are
inclusive; the overall 20 s attempt endpoint is exclusive. If the fifth first
hit has landed when the attempt ends, it finalizes a five-group success without
waiting past that endpoint for its bonus. At 180 s, an unfinished Small attempt
ends without refund; an already awarded Small benefit keeps its original expiry.
Motion has zero motor lag; group transitions use zero blackout and failed
progress uses a one-nanosecond state marker. Ring radius rounds to nearest
millimetre before inward-inclusive 15 mm bands. These tie/rounding policies make
unspecified boundaries deterministic and are separate from manual constants.

UC §§5.6.4.1–5.6.4.9 define a 2 s activation expiry delay and distinguish
continuous field, timed terrain and non-field benefits. Terrain benefits clear
on death; non-field sources survive a recipient's death and apply to a newly
rebuilt outpost. Same-type attack, defense, vulnerability, healing and cooling
use the strongest effect; defense and vulnerability combine as described in
§5.6.4.1. Final noninteger damage/healing rounds to nearest integer.

| Zone or traversal | Principal effect and conditions | UC section |
|---|---|---|
| Own base | 50% defense, ammunition exchange, clear weakness | 5.6.4.2 |
| Own vantage point | 50% defense, zone ammunition exchange | 5.6.4.3 |
| Central highland | 25% defense; opposite teams cannot activate the same connected zone simultaneously | 5.6.4.4 |
| Highland, low then high in 5 s | 25% defense for 30 s | 5.6.4.5 |
| Road, low then high in 3 s | 25% defense for 5 s; 15 s repeat cooldown | 5.6.4.5 |
| Across tunnel, end then end in 3 s | 25% defense for 30 s | Table 5-27 |
| Through tunnel, end/interior/end in 3 s | 1.5× cooling for 120 s | Table 5-27 |
| Fully controlled outpost territory | Exchange, healing, clear weakness and associated invincibility subject to respawn timing | 5.6.4.6 |
| Assembly | Own Heavy only; active assembly gives invincibility and disables launcher, with cumulative 45 s limit | 5.6.4.7 |
| Own supply | Exchange, healing/respawn acceleration; Heavy invincible | 5.6.4.8 |
| Own Fortress | Enabled when own outpost territory is not controlled; first own robot gets 50% defense, extra cooling and reserve ammunition | 5.6.4.9 |
| Opponent Fortress | After 3 min and control of opponent outpost territory; 100% vulnerability until base armor opens after 10 s activation | 5.6.4.9 |

Terrain sequences must not detect another card along the way. Combining
different terrain benefits grants 50% defense for the maximum of old remaining
and new initial duration. UC lists nine tunnel cards per side, not the old
simulator's six-pad tunnel assumption.

Fortress uses the maximum base HP loss reached this round:
`Δ = max_base_HP - lowest_base_HP`. Cooling is `floor(Δ / 80)`, capped at +75/s;
reserve ammunition is `100 + floor(Δ / 15)`, capped at 500. Each 17 mm launch
consumes 1 reserve, each 42 mm launch 10, before personal allowance. Opposing
Fortress activation progress pauses and remains for 3 s after death/expiry;
base armor opening ends the vulnerability. UC examples on pp.104–105 use an
older 75% Fortress vulnerability value in one example, inconsistent with the
100% mechanism on p.107; retain this conflict rather than importing 75%.

The native continuous-field implementation follows §5.6.4.9's **100%** enemy
Fortress vulnerability, combined additively with the strongest defense under
§5.6.4.1. Fortress reserve is a persistent **per-side, per-round pool**: its
capacity grows with historical Base HP loss; consumption survives handover,
leaving, death and Base repairs. Re-entry does not refill it. This reserve
lifetime is an explicit simulator policy because the manual specifies capacity
and spending priority but no replenishment schedule. Only a new round resets it.

## UC Sentry, aerial, dart and Radar systems

### Sentry

UC Table 3-6 and §5.7.2 distinguish automatic Sentry (400 HP, 260 heat, 30/s
cooling, 100 W) and semi-automatic Sentry (200 HP, 100 heat, 10/s, 60 W).
Choose before the 15 s self-check; automatic is the default. A commander
intervention for automatic Sentry costs 50 gold; the corresponding intervention
for semi-automatic Sentry has no intervention fee. Purchase prices still need
their own accounting.

Sentry starts in movement posture. Switching has a 5 s cooldown. Each posture
weakens after more than 3 min cumulative use. Each enhanced posture has an
independent 15 s cumulative allowance (UC Table 5-28).

| Posture | Normal | Weakened | Enhanced |
|---|---|---|---|
| Attack | 3× cooling; half power; 25% vulnerability | 2× cooling; same penalties | Heat stays zero; same penalties |
| Defense | 50% defense; half power; one-third cooling | 25% defense; same penalties | 99% defense; same penalties |
| Movement | 1.5× power; 25% vulnerability; one-third cooling | 1.2× power; same penalties | 200 W limit; same other penalties |

Power/cooling reductions multiply the value after other benefits; Table 5-28
specifies integer values. A robot death must not erase cumulative posture use. The native implementation
and explicit timing/selection policies are described in [gameplay](gameplay.md#sentry-modes-postures-and-interventions).

### Aerial system

UC §3.2.1 and ROB §2.4.4 replace the old shooting drone: autonomous aircraft
communicate only indirectly via the hangar, carry no launcher, and use an
emergency-stop remote. A hangar holds up to four aircraft. Impact attacks can
score on base/outpost armor, but detection thresholds are pending (UC Table 5-1).
No 2026 aerial support timer, tether or 17 mm launcher can be assumed.

### Darts

UC §5.7.3 grants four darts and gate opportunities at 30 s and 240 s; unused
opportunities accumulate. Gate opening takes about 7 s; the fully open launch
period lasts 30 s, target detection 40 s, and the first closure starts a 15 s
cooldown. Launch is disabled during gate motion. A hit closes that target's
detection window for 2 s.

Targets include outpost and four base modes. A moving base target travels
within ±280 mm along the field-width direction. Random-moving mode relocates
within 600 ms after launch detection; terminal-moving waits 1.2 s before that
relocation. Hits in these modes require the target to be stationary. Details of
reset/restart and the one outpost-to-base switch are in pp.110–111.

Beyond raw damage, fixed/random-fixed hits obscure opposing operator UI,
minimap robot positions and custom UI for 10/5/3/2 s on successive hits; base
hits distribute 200/600 experience to surviving Infantry. Random-fixed base
hits suspend current terrain/rune benefits for that duration while timers run.
Moving/terminal-moving base hits obscure for 10 s, award 2,500 shared experience,
invalidate current terrain/rune benefits and remove 20%/50% maximum HP from all
surviving opposing ground robots without damage/kill experience. Repeated
obscurations add. Four fixed/random-fixed base hits or any moving-mode base hit
open base armor. A hit disables the corresponding base/outpost zone for 30 s,
refreshed on another hit.

The manual allows a map-based fallback control interface during obscuration
with commands at least 3 s apart, and says custom terminals still receive video
data. Model visibility restrictions separately from network delivery.

### Radar

UC §5.7.4 replaces a simple mark flag with progress `P ∈ [0,150]` and history
variable `x`. Accurate position error is <0.8 m, semi-accurate is [0.8,1.6) m,
and wrong is ≥1.6 m. Accurate/semi-accurate samples add 1/0.5 to `x` after a
previous nonwrong sample, otherwise reset it to 1/0.5. Wrong samples subtract
0.8 after a previous wrong/interrupted sample, otherwise reset to −0.8.
Then `P = clamp(P + x, 0, 150)`. Every 0.5 s without data produces a wrong sample
(Table 5-29).

Opponent marks give 15% vulnerability at `P ∈ [100,120)`, 20% at `[120,150]`,
and authoritative positions from 100. Friendly marks reveal authoritative
positions from 50 (Table 5-30). Respawn retains P but resets x to zero.
Each accumulated minute with at least one opposing robot vulnerable earns a
double-vulnerability opportunity; overlapping robots do not multiply time.
Activation doubles Radar vulnerability for 30 s, at most twice per round.

The new information-wave mechanism exposes opponent state through an official
radio source with three interference difficulty levels. Decoding the opponent's
six-character alphanumeric key raises that interference difficulty; while the
opponent's difficulty is higher than one's own, their accurate marks cannot
cause vulnerability or build double-vulnerability progress. Physical signal
parameters are to be published later. Exact packets require the separately
referenced 2027 communication protocol, not supplied with these three PDFs.

## UL 3v3 rules

UL chapter 3 defines its own game; UC economy, assembly unlocks, adaptive armor,
base HP and control accumulation do not apply automatically.

| Robot | HP | Power W | Heat | Cooling /s | Initial allowance |
|---|---:|---:|---:|---:|---:|
| Heavy before installation | 100 | 50 | — | — | 0 |
| Heavy after installation | 450 | 50 | 250 | 66 | 0 |
| Infantry power chassis | 300 | 90 | Depends on launcher | Depends on launcher | 0 |
| Infantry HP chassis | 350 | 75 | Depends on launcher | Depends on launcher | 0 |
| Sentry | 400 | 100 | 260 | 30 | 750, no additions |

Infantry burst launcher is 230 heat / 14 per s; cooling launcher 88 / 24 per s.
Defaults are HP/cooling priority. Heavy maximum speed is 16.5 m/s after launcher
installation, Infantry/Sentry 25 m/s (UL Tables 3-2–3-5). UL Sentry must be
fully automatic and does not use multi-robot communication (§3.1.3).
Installation at the workbench and a detected 42 mm shot unlock Heavy's improved
properties; invincibility in own supply applies only before successful
installation (§3.3.6). UC's assembly/time/deficit permission formula is not stated.

UL §§3.3.1–3.3.3 keep normal impact-speed thresholds and 20/200/2 damage, heat
increments/cooling cadence, a 60 J power buffer and five-second cut, and 42 mm
suppression gates. However **42 mm overspeed differs from UC**: through 1.1×
limit → 15 s; through 1.2× → 20 s; above → round lock (Table 3-9).

Supply heals at 25% maximum HP/s. Respawn progress is 5 for the first death,
then increases by 5 per death; automatic progress is 1/s. Respawn restores 20%
HP, grants 30 s invincibility and weakness (launcher locked, cannot activate
control zones). Own supply-card detection clears weakness and invincibility
(§3.3.2). Death resets heat and buffer.

Income at countdown 4:59/3:59/2:59/1:59/0:59 is 200/200/200/300/300 gold.
Only ammunition is purchasable, at 10 gold per ten 17 mm or one 42 mm round.
At the first 200- and 400-point control gaps, the trailing team earns 200 gold,
at most 400 per team via catch-up (§3.3.3). Heavy and Infantry buy at supply;
Sentry's 750 allowance cannot increase.

### Control and victory

UL §3.3.4 starts both teams at 600 control and the three zones (red highland,
blue highland, center) neutral. Each team's progress per zone ranges from 0–5.
One team alone gains 1/s if opposing progress is zero; otherwise it removes
1/s from opposing progress first. Five grants control; zero removes it.
Both teams activating pauses progress. With nobody activating a nonneutral
zone, progress decays 1/s. Do not reuse UC temporary/full territory states.

Each controlled zone drains the opponent by 1 control/s. Each robot death costs
its team 10 control. A trailing team at ≥200 gap needs 33% less respawn progress
and heals at 33%/s; at ≥400 gap, 50% less progress and 50%/s healing. These are
active catch-up conditions, distinct from the one-time gold awards.

Highland activation gives 25% defense. Detecting the two own tunnel cards within
3 s, with no intervening other card, gives 1.5× cooling for 60 s. Buff activation
expires 2 s after losing card detection (§3.3.5).

Zero control ends the round immediately. Otherwise at 300 s compare control,
team attack damage, remaining team robot HP, then draw. Groups use BO2 and
eliminations BO3, with another round for an elimination draw (§3.3.7).

## UL Infantry 1v1 rules

UL chapter 4 uses Infantry 3 with 200 HP, 120 W, 80 heat, 12/s cooling, 25 m/s
maximum speed and 200 initial rounds without replenishment (Table 4-2).
Preparation and running each last 2 min. There is no ordinary respawn system.
Despite Table 4-2 saying healing/respawn is inapplicable, §4.3.1 explicitly adds
**healing equal to 50% of projectile damage dealt**. This specific new mechanism
must not be lost when translating the table.

UL §4.4 resolves a round by defeat, then at timeout higher remaining HP, then
later first HP-loss time when nonzero HP is tied. When both finish full HP,
compare total rounds fired (higher wins), then earlier first launch, then draw.
With lifesteal, both robots can have been damaged and finish full: the priority
between the first-damage clause and full-HP clauses needs an explicit decision.
Group format is BO2 and elimination BO3, with special incomplete/tied series
resolution in §4.4.1; do not blindly reuse UC overtime behavior.

## Robot construction and visual implications

ROB is a hardware/inspection specification. The values below constrain future
presets and visuals; they are not a request to add automated inspection rules.

| Equipment | Construction envelope or limit | Source |
|---|---|---|
| Heavy | 36 kg excluding launcher; 41 kg including; stows in 600 mm cube; 300 Wh, 30 V | ROB Table 2-5, §§2.4.1, S67–S68 |
| Heavy extension | Either 1200 × 1200 × 1100 mm or the alternative locked-chassis envelope test | ROB Table 2-5, pp.27–28 |
| Modular 42 mm launcher | 6 kg including referee hardware, 400 mm stowed cube; no battery; internal voltage limit 140 V | ROB Table 2-6, S69–S70 |
| Infantry and Sentry | 28 kg; 600 mm stowed cube; 800 mm maximum cube; 300 Wh, 30 V | ROB Tables 2-7–2-8 |
| Aerial robot | 4 Wh, 30 V; no launcher; weight/dimension relation below | ROB Table 2-9, S75–S81 |
| Hangar | Up to 4 aircraft; 300 × 300 × 200 mm stowed, 600 × 300 × 400 mm extended; 300 Wh, 30 V | ROB Table 2-10, S82–S83 |
| Radar compute | 750 W; 600 × 350 × 600 mm | ROB Table 2-13 |
| Custom terminal | 600 mm stowed cube; 30 V; wired interface | ROB Table 2-4, S46–S50 |

Aerial weight `x ∈ [0.15,0.249] kg` sets maximum cube side
`round((0.3225 - x) / 0.001225) mm`; below 0.15 kg the limit is 141 mm
(ROB Table 2-9). S78 requires full propeller enclosure that blocks a 17 mm ball;
S79 requires simultaneous emergency stop and stop on loss of remote signal.
S80 specifies a top circular team-color lamp of seven LEDs within a 24 mm
outer envelope; self-test details are pending.

Armor is especially consequential for rendering and scoring:

- ROB §2.1.4 (p.15) says the 2027 detection face is a **dot-matrix screen**
  displaying robot numbers/patterns, with exact artwork to follow. Current
  artwork masks and digit atlases cannot be claimed as final 2027 artwork.
- ROB §3.6.1, S120–S125 requires four armor modules facing the body ±X/±Y,
  rigidly mounted with the specified 75° normal/up relationship and installation
  tolerances. Armor bottom edges are ≤400 mm above the supporting surface and
  may not touch it; pairwise bottom-edge height difference is ≤100 mm, except
  transiently while crossing terrain. The ROB body frame uses **Z downward**;
  convert explicitly rather than copying it into FLU.
- ROB §3.6.3, S129–S130 sets occlusion limits: lower edge 105°, upper 120°,
  sides 145°. Heavy may conditionally obscure at most one plate within the
  specified 150/170 mm clearances. The drawings are needed for precise cones.
- ROB §2.1.8 requires 17 mm fluorescent-projectile charging equipment, except
  in UL; detailed brightness self-test is still pending. This affects projectile
  appearance without defining a new damage multiplier.

ROB §2.1.7 specifies UDP video and MQTT + Protobuf match data for custom
terminals. These transport names do not establish compatibility of existing
2026 message schemas with the unavailable 2027 communication protocol.
ROB S5–S8 distinguishes hardware supercapacitors (2,000 J nominal, 2,200 J
measured maximum) from the 60 J referee buffer. Physical inspection limits are
not interchangeable with the drivetrain's assumed shared power budget.

## Decisions and missing inputs before implementation

| Issue | Evidence | Required follow-up |
|---|---|---|
| 2027 field/collision geometry | UC chapter 4; checksum-pinned RMUC source package now adapted by [the runtime builder](rmuc2027-field.md) | Rest-pose composition reviewed; live source-bound outpost rotation, base motion and fitted detector envelopes available. Final masks and base travel remain approximate; see runtime guide. UL remains separate |
| Armor detection geometry/artwork | ROB §2.1.4 promises pattern details later | Obtain module/drawing assets; do not carry old 101 × 94 mm detection area forward without validation |
| Aerial-impact detection | UC Table 5-1 explicitly pending | Await force/time/area-impulse thresholds; do not approximate as a projectile rule |
| Armor statistics singularities | UC §5.2 divides by σ and opposing accuracy | Adopted user policy: undefined selection-time accuracy freezes default 1× armor until reselection; opportunity is consumed |
| Reinforcement clipping and updates | UC §5.2 specifies circle and area but not edge treatment, exact boundary or update cadence | Adopted user policy: freeze area at selection, boundary inside, clip without resizing; common nonnegative coordinates may lie outside smaller plates |
| Permanent heat-lock equality | UC §5.1.3 text `>` vs Figure 5-1 `>=`; same UL conflict | Adopted by user instruction: Figure 5-1 inclusive `>=`; not an official clarification |
| Control scoring and contested timing | UC Table 5-21 and p.97 | Adopted: temporary control scores; capture before coincident whole-second scoring; fresh progress after transitions. The native adapter resolves per-robot 2 s expiry. Physical destruction follows stable contact order |
| Heavy unlock and upgrades | UC §5.7.1, Table 5-16 | Resolve negative unlock times, HP adjustment on Heavy upgrades, confirmation-shot allowance accounting and removal/reinstallation |
| Assembly splitting | ROB S59 generally forbids splitting, UC §5.7.1 and UL §3.3.6 permit it at workbench | Apply/document the specific exception without broadly permitting disassembly |
| UL launcher installation phase | ROB Table 2-5 says seven-minute phase, UL has five-minute rounds | Treat as a cross-manual wording conflict; obtain clarification before enforcement |
| Fortress example conflict | UC p.105 example says 75% vulnerability; §5.6.4.9 says 100% | Prefer no silent inherited constant; record the selected clause if implementing |
| Healing and logistics | UC §§5.3.1, 5.6.1 | Adopted in the native lifecycle: apply logistics once to outpost healing, fixed-point 10 Hz healing, continuous respawn progress. Other buff stacking remains pending |
| Time and event ordering | UC Table 2-1, countdown-based income tables | Specify tick boundary, settlement order, capture/death/purchase ties; keep 128 Hz world time |
| Radar information-wave details | UC §5.7.4 explicitly defers signal parameters | Obtain later release and 2027 communication protocol; clarify difficulty ceiling/key failures |
| UL full-HP tiebreak | UL §4.4 includes first damage and full-HP launch comparisons | Clarify precedence after both heal back to full |
| UL lifesteal rounding/cap | UL §4.3.1 gives 50% but not all settlement cases | Specify overkill basis, rounding, HP cap and simultaneous lethal hits |

## Implementation boundaries and migration order

The following is engineering guidance, not additional competition rules:

1. Preserve the complete 2026 implementation and its source baseline. Additive
   build features should control inclusion; an explicit ruleset identifies the
   behavior of a match. Unknown/unimplemented selections must fail clearly.
2. Identify a ruleset by season, competition mode and manual revision. Treat
   field package identity separately; a 2027 label on a 2026 field is insufficient.
3. Add independent 2027 rule data and state machines in gameplay/world according
   to existing crate boundaries. Shared physical integration remains in physics,
   with variant equipment geometry where needed; rendering stays passive.
4. Carry ruleset identity and every authoritative new state through admission,
   snapshots, restore and prediction before exposing a running 2027 match.
   This includes mutable ownership, capture progress, control totals, historical
   deficits, armor hit statistics/selection, assembly/income state, buff sources
   and Sentry/Radar timers. Reject incompatible peers rather than infer season
   from the local binary default.
5. Validate isolated mechanics with explicit clocks, then integrate host commands,
   presentation and field assets. Keep uncertain/pending requirements unavailable
   or explicitly marked assumptions; do not add unrelated enforcement.

A future behavior test matrix should include 2026 regression traces, each
competition's timeout order, contested capture and decay, outpost ownership
transfer without rotor restart, adaptive-armor boundary/statistics cases,
assembly prerequisites and recurring income, historical-deficit unlocks,
independent launcher lock causes, logistics degradation, weak respawn benefits,
Sentry posture budgets, Radar mark history and UL lifesteal ties. Partitioning
elapsed time into different tick batches must continue to produce identical
results. Build availability alone is not evidence of any of those rules passing.

## Temporary Hero conversion for Heavy

In an RMUC 2027 running round, **J → Convert to Hero** replaces the parked
Engineer's chassis and visual model with the source folding Hero. The player
keeps its id, team and Heavy roster slot. Launcher permission follows UC §5.7.1:
L3 completion or `300 - M/50 - 2N/3` elapsed seconds, with historical maxima
of opponent-minus-own Base HP and control. Negative thresholds unlock immediately
(simulator policy); the exact fractional boundary is inclusive. No conversion
is allowed during an unfinished assembly/Core recovery, while moving, defeated
or offline. Conversion releases existing held cargo rather than deleting it.

This is an explicit gameplay approximation: conversion is available wherever
parked, without workbench installation or the confirmation shot. It installs a
600-HP Heavy (existing damage preserved), 300 heat limit, 100/s cooling, a
100-heat 42 mm launch, and the Hero's configured weapon speed/rate within host
caps. Initial ammunition remains zero. Buy 42 mm ammunition through the existing
controls: one shell for 10 gold at a service zone, or ten for 150 gold remotely
with six-second delivery; the team cap is 100. Direct hits use the native
200-HP damage table; **42 mm idle/death/link suppression remains deferred**.
Adaptive accuracy statistics continue to count only 17 mm launches/hits.

The swap is immediate, with collision geometry rebuilt and placement revision
incremented so prediction does not interpolate between shapes. Source arm meshes
are removed; the Unit remains a physical object. There is no reverse conversion.
Starting a new round retains the chosen Hero shape but clears installation,
ammunition and unlock history; use the same button after unlocking again.
