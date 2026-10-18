<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Standalone gameplay engine

`rm-simulator-gameplay` runs a match without Bevy, Rapier, CAD, networking or
host time. It is pinned to the English RoboMaster 2026 University Championship
Rule Manual V2.1.0, dated 2026-07-17. It does not silently adopt later releases.
The [2027 rules reference](rules-2027.md) is a separate implementation baseline.
The [live 2027 control/structure subset](rmuc2027-field.md#enable-the-live-controlstructure-subset)
uses `rmuc_2027::match_rules::Match`, not the 2026 `Game`; [season builds](seasons.md) explains feature availability and admission.

```sh
just gameplay-test
just gameplay-demo
```

The demo completes a BO3 match, including setup, initialization, countdown,
base destruction, result confirmation and the next round. Tests cover timed
purchases, income, respawns, rebuilding, heat, eligibility and tick partitioning.

Read this guide before changing either gameplay integration.

## Contents

- [RMUC 2027 control component](#rmuc-2027-control-component)
- [Live integration and ownership](#live-integration-and-ownership)
- [API and clock](#api-and-clock)
- [Rule coverage and unsupported elements](#rule-coverage-and-unsupported-elements)
- [Implemented consequences](#implemented-consequences)
- [Result comparison](#result-comparison)
- [Ambiguities and application choices](#ambiguities-and-application-choices)

## RMUC 2027 control component

`rm_simulator_gameplay::rmuc_2027::Control` implements the UC V1.0.0 territory
state machine separately from the legacy `Game`. The native 2027 match adapter
now consumes it; live world admission requires `season-2027` and source bindings. The 2026 rule
inventory and engine below retain their existing meaning.

The component implements UC §5.6.2, Tables 5-20–5-21, and the ownership-transfer
part of §5.6.1:

- Four territories start fully controlled by their original teams. Each combines
  its two constituent buff points; team presence at either point counts.
- Five seconds of uncontested opposing activation changes full control to
  temporary control for the opposing team. Incomplete progress rolls back at the
  same rate when occupation ends; opposing presence anywhere in the same
  territory pauses progress.
- Five seconds without the temporary controller's activation gives full control
  to the other team, even if no robot is present. The temporary controller's
  return rolls incomplete recovery back. Only the initial owner can fully
  control a base territory.
- Outpost destruction changes full control to temporary control for the same
  team, or temporary control to full control for its current controller. Physical
  outpost ownership is separate. A completed full-control transfer rebuilds the
  outpost for its new owner and emits a 500 HP rebuild event.
- **Temporary and full control both earn points**, per the user-selected
  interpretation of the manual's unqualified wording. Both outpost territories
  award 1 point/s; both base territories award 3 points/s. Base protection still
  requires full control of that team's original outpost territory.

`set_activation(side, point, [red_present, blue_present])` replaces resolved team
presence at one point. The caller must enforce robot eligibility, zone disabling
and the two-second activation expiry before submitting it. Changing points within
one territory preserves aggregate occupation continuity. Robot count does not
multiply capture speed. `outpost_destroyed(side)` observes an authoritative
physical destruction; it does not simulate damage or launch a projectile.

`advance_ns(duration_ns)` advances only running-round time, clamped at 420 s.
Scores settle at each elapsed whole second. Capture/recovery completes before a
score award at the same timestamp, so a capture at exactly 5 s can earn that
second's points. Observations applied after advancing affect future time only.
Each control-state transition starts fresh progress, including transitions due
to destruction; partial progress is not carried into a different transition.
These settlement and progress-reset choices are explicit simulation policies,
not additional timing precision specified by the manual.

The component owns no base HP, robot HP, rotor, buffs, winner or match lifecycle.
Callers consume `base_protected` and rebuild events and use cumulative points
in the native 2027 result comparison. In particular, an outpost rebuild event is
not an instruction to restart its rotor. The enclosing match/world adapters
carry these values through checkpoints and the network; the component itself
still has no geometry or physics dependencies.

`Control::snapshot()` includes activations, partial progress, scores, physical
owners and elapsed nanoseconds. Serialize this snapshot and validate it with
`Control::restore` to resume the same computation. This is a component snapshot,
not a full `Game` or world checkpoint. Event sequences and scores are independent
of how elapsed time is partitioned, including 128 Hz caller steps.

```sh
# Component unit tests and examples without enabling the 2026 engine.
cargo test --locked -p rm-simulator-gameplay --no-default-features --features season-2027 --lib rmuc_2027
cargo test --locked -p rm-simulator-gameplay --no-default-features --features season-2027 --doc rmuc_2027
```

`just control-test` runs the native 2027 unit suite, the additional
`tests/control_2027.rs` scenarios, physical world-adapter tests and Python console
tests without a GPU. The additional scenarios enumerate all 16 two-point/team
presence combinations for all 12 valid territory/control-state pairs (192
cases), exercise both sides and every ground role, and compare explicit time
partitions across JSON restore. Exact-boundary regressions cover two-second
RFID expiry, five-second capture, the three-minute Fortress unlock, ten-second
armor opening and three-second retained progress. Captures split the enclosing
match advance immediately so new eligibility applies at fractional timestamps;
retention expires at its endpoint without requiring another call.

Physical tests use small synthetic zones and real chassis to check contention,
leaving a zone and the same expiry tick after restore. They do not certify the
diagram-derived antenna polygons against hardware. The optional
[console smoke scenario](console.md#deterministic-tick-scripts) checks actual
host commands and authoritative replies with the bound field. These checks
cover native control, continuous field benefits, terrain traversal and Rune.
Virtual assembly and adaptive armor are supported as described below; physical
manipulation and playable Radar remain outside its scope. `just control-test` also
runs `adaptive_2027`, `assembly_2027`, `terrain_2027` and `rune_2027`; the latter enumerates both teams, all six
Big arm counts and each ring-table tier, including exact target deadlines and
restore across pending bonus hits.

Continuous native field benefits now include own Base/Vantage 50% defense,
exclusive Central Highland 25% defense, own Fortress 50% defense plus historical
Base-deficit cooling/reserve, and enemy Fortress 100% vulnerability until armor
opens. Same-type defense uses the strongest value; defense and vulnerability
combine before nearest-integer damage rounding (§5.6.4.1). Ordinary projectile
and armor-collision HP loss use these effects; operator penalties bypass them.
Weak, defeated and offline robots receive no continuous benefits. Native launch
checks, client pending-ammunition accounting and the ammo HUD include the reserve.

`Match::control_diagnostics()` explains all eight point states without modifying
authority. `Match::field_effects()` reports defense, vulnerability, cooling and
usable reserve after the latest activation resolution. Historical Base HP minima
and reserve spending are checkpoint authority, not recomputed from current HP.
See [the Fortress reserve policy](rules-2027.md#uc-territory-logistics-and-victory)
for its explicit per-side/per-round lifetime.

## Live integration and ownership

The world referee runs one `Game` as the live match authority for both the app
and the server. It steps the engine to the round time at the start of every
128 Hz world tick, so the engine's 1 ms clock never runs ahead of the world.

| Owner | Responsibility |
|---|---|
| World referee | Phase changes, round clock, rune schedule and activation; translating engine events |
| `Game` | HP, damage, heat, allowance, gold, experience, levels, performance, respawn, weakness, invincibility, buffs, base shield and outpost protection, rebuild opportunities, round result |
| World `Field` | Detection (speed, area, interval) and scoring geometry; mirrors base and outpost HP into the physical objects; stops rotors; reports buff point contacts from the Figure 5-24 areas (`zones.rs`) and armor collisions |
| `rm-simulator-physics` | Dynamics, raw contacts, armor collision speeds and prescribed rotor motion |

`StartMatch` resets the engine and begins its countdown; `ResetMatch` returns it
to Idle. Idle is free practice. The live policy defaults to
`enforce_allowance: false, exchange_requires_zone: false`, and survives resets.
A field without a referee keeps the physical damage path and no engine.

`RefereeSnapshot.game` carries the engine snapshot over GNS UDP and
`GET /api/state`. The HTTP page sends referee commands through
`POST /api/command` or `POST /api/referee`. Clients and hosts must use matching
builds; `PROTOCOL_VERSION` in the server's `protocol.rs` is authoritative.
Pilots and spectators cannot submit referee commands; a pilot may only buy
ammunition, pick its own performance type and fire.

| Live control | Behavior |
|---|---|
| Policy | Toggle allowance enforcement and whether exchanges need a service zone. |
| Team gold | Set either team's balance. |
| Robot | Set allowance, performance type (not while Running), clear weakness, paid instant respawn, revive, set HP or apply penalty damage. |
| Exchange | Buy Table 5-6 units for a robot from team gold. |
| Result | `Adjudicate` a finished round the comparison could not decide. |
| Outpost and base HP | Set through the engine; zero outpost HP counts as a destruction. |
| Rune opportunities and buff | Set a team's opportunity count; override or clear its rune buff. |

The selectable Drone is a fixed-altitude flight prototype with a 17 mm launcher,
tethered to its section 4.5 Aerial Safety Rope.
The live referee starts with `enforce_air_support: false`, so the prototype
can fire during a round without unsupported activation controls. Heat, caliber,
allowance policy and firing cadence still apply. The standalone game defaults
to requiring air support under section 5.6.3; callers can retain that policy.
Drone air-support controls and mission mechanics remain outside the live integration.

### Outside the live integration

The Assembly Zone, remote exchanges and HP purchases, power enforcement,
assembly, drone, dart, radar, engineer and sentry equipment, penalties,
disconnection and multi-round series are not connected. The engine still
implements several of them for headless scenarios.

## API and clock

Construct `Game::new(Config)` with a fixed roster and explicit performance
parameters. Robot ids must be unique and HP positive. The roster supports
Hero, Engineer, Infantry, Drone, Sentry, Dart and Radar. These records do not
create chassis, so unsupported equipment can exist in a headless scenario.
The app also offers manually driven Engineer and Sentry chassis prototypes;
the Engineer arm and Sentry radar are decorative and their equipment rules
remain outside the live integration.
There is no referee robot. `AddRobot` and `RemoveRobot` admit and drop robots
at any phase; the live referee uses them as pilots join and leave.

`Game::command(Command)` validates an input and commits it atomically. An error
leaves the game unchanged, including pending deliveries and event ids.
`Game::can_launch` includes match phase as well as robot permissions.
`Game::step(ticks)` advances an explicit number of 1 ms ticks. Withholding ticks
pauses the game. Splitting the same elapsed ticks cannot change the result.
`BeginRound` runs 180 s setup, 15 s initialization and 5 s countdown before the
420 s round. `BeginCountdown` is an explicit practice shortcut. `EndRound`
ends a running round early; `ConfirmResult` records its result once.

A BO2 ends after two confirmed rounds. A BO3 or BO5 ends after two or three
wins respectively; draws require another round. `BeginRound` after confirmation
clears round resources, timers and observations while retaining result history.
`ResetMatch` clears that history and restores the configured roster. The global
simulation tick never goes backwards. Event ids restart on match reset.

Snapshots expose teams, robots, pending purchases, buffs, observations, round
results and the latest 512 events. Event retention is an application policy;
record input commands and their tick positions if you need a complete replay.
Snapshots and commands serialize through serde. Snapshots are read-only views,
not a trusted save-game loading API; clone a `Game` to branch a scenario.

`Game::step` skips directly across setup, initialization, countdown, and phases
whose state is frozen. Running gameplay retains exact 1 ms processing. Pending
deliveries remain ordered by due tick, so the running loop only removes the due
prefix instead of rebuilding the complete queue each millisecond.

`Game::command` validates and commits combat-state observations,
disconnections, validated launches, and observed launches without cloning the
whole game. These paths have no fallible operation after their validation.
Commands with several dependent writes retain the clone-and-commit transaction,
which keeps rejection atomic without a general rollback system.

Run the standard-library microbenchmark with:

```sh
cargo run -p rm-simulator-gameplay --release --example benchmark
```

On the development machine on 2026-09-11, three runs put one coarse 200,000-tick
setup-to-running advance at 93-104 us, versus 631-666 us for 200,000 one-tick
calls. Twenty thousand direct combat-state commands took 5.54-5.62 ms; wrapping
each in the former full-clone transaction pattern took 21.9-23.8 ms with a full
512-event history. This is a reproducible microbenchmark rather than a stable
performance target; hardware and allocator results will differ.

## Rule coverage and unsupported elements

[`coverage::RULES`](../crates/rm-simulator-gameplay/src/coverage.rs) is the
complete inventory of gameplay mechanism groups in sections 5.1 through 5.8,
plus the competition process and human adjudication in sections 6 through 9.
It names each source section, implementation status and remaining work.

| Status | Meaning |
|---|---|
| `Implemented` | The engine executes the listed state transitions. The entry also names exclusions. |
| `External` | An authoritative caller must supply the specified observation; the engine applies the listed consequences. |
| `Tracked` | State can be recorded but the mechanic's consequences are not automatically executed. |

`Observe` records a mechanic, optional team and robot, a state name and named
measurements. Include units in measurement keys. The engine stamps the current
round tick and replaces the previous observation for the same mechanic/team/
robot. Missing observations mean **unknown**, not inactive or unsupported
hardware absent. Consult the coverage register separately to determine support.
Observations never grant rewards, inflict damage or change permissions.

All field buff point types have structured zone contacts, including base,
resupply, outpost, central and trapezoid highlands, road, elevated crossing,
launch ramp, tunnel, assembly and fortress. Radar decoding, dart windows,
engineer assemblies, sentry poses, hero deployment, drone counters, module
faults, power telemetry, inspections, timeouts, penalties and appeals can be
recorded even when there is no simulated equipment or sensor for them.

Mechanics outside the live integration are named under
[Outside the live integration](#outside-the-live-integration).

## Implemented consequences

- Income follows Table 5-5 at elapsed 1, 61, 121, 181, 241, 301 and 361 seconds.
  Table 5-6 purchases enforce costs and per-team ammo limits. Remote ammo and
  HP arrive six seconds later. Defeat cancels pending HP without refund.
- Sections 5.2.1 and 5.2.2 provide resupply healing, normal and paid respawn,
  accelerated respawn progress, weakened launch restrictions and invincibility.
  The caller supplies out-of-combat and irregular-disconnection observations.
- Section 5.3.2 supplies initial ammo, launch consumption and accumulated sentry
  resupply. The Hero's special 42 mm immunity mechanism is not yet enforced.
- Section 5.3.3 certified assembly completions provide recurring income,
  level-cap increases, defense, base HP and overflow shield. Availability times,
  preceding level completion and the single level-4 completion are checked.
  Robotic arm poses, shared-core exclusion and assembly failure penalties are
  still external; a player must not be allowed to certify their own completion.
- Section 5.4.1 and Table 5-11 apply launch, damage and kill experience,
  shared unattributed awards and rune rewards up to the current cap. Section
  5.4.2 and Tables 5-12 to 5-14 select HP, heat limit and cooling by Hero or
  Infantry performance type and level; other robots use fixed values.
- Section 5.5.1 supplies base HP, virtual shield, outpost protection, destruction,
  cumulative base-loss rebuild opportunities and individual uninterrupted
  rebuild scans, with a strict five-minute cutoff. Rotation remains outside
  this crate; the world starts and stops the rotor.
- Section 5.5.3.1 applies the strongest attack, defense and vulnerability
  independently. `ProjectileHit` computes Table 5-2 damage, the centre-square
  bonus, attack and defense; `Damage` takes an amount that already includes
  attacker effects. Penalties, disconnection and darts bypass defense.
- Section 5.1.3 adds heat per detected launch, cools at 10 Hz and maintains
  independent temporary and permanent launch locks. Table 5-3 speed observations
  set independent launch locks. `Launch` is a validated action; `ObserveLaunch`
  accounts for an actual shot even if it happened while locked or over allowance.
  Both update launch counters. An adapter must report each shot only once.
- Section 5.6.3 supplies drone air-support time, minute grants, paid continuation
  and a launch gate. Landing-pad checks, laser and radar countermeasures remain
  external. Section 5.6.7 tracks chassis energy and resupply recharge, but does
  not alter physical motor limits. Buffer power and instant-respawn power boosts
  remain unsupported.

## Result comparison

The section 5.8 result comparison excludes virtual shields from base HP,
counts effective attack damage including shield loss and penalties, and
excludes collision/disconnection damage from the attack total. Confirming an
ambiguous result requires `Adjudicate`; the engine never guesses a winner.

## Ambiguities and application choices

- The printed section 5.8 only compares unequal base HP when a base is destroyed.
  It does not state how unequal surviving bases decide a round. It also does
  not clarify comparisons after destroyed outposts have been rebuilt. These
  cases return `NeedsRefereeDecision` rather than inventing a corrected rule.
- Section 5.1.3's prose says `Q1 > Q2`, while Figure 5-1 says `Q1 >= Q2`.
  The engine follows the figure for the permanent heat lock.
- Assembly income settles on round-aligned ten-second boundaries. The manual
  specifies the interval but not its phase relative to each assembly completion.
- Respawn progress accrues continuously on the world clock. The remaining-time
  input to its formula is rounded to seconds as section 5.2.2 specifies;
  fractional timer units are retained. Low maximum HP configurations respawn
  with at least one HP so a positive recovery cannot leave them defeated.
- Resupply recovery uses round-aligned one-second boundaries. Cooling uses
  round-aligned 100 ms boundaries. Expirations are processed before other timed
  effects, and round end takes precedence over a delivery at exactly 7:00.
- A destroyed base ends the round immediately after that damage command.
  Simultaneous physical hits must be ordered by the caller; batching simultaneous
  destruction is not implemented.
- Outpost rebuild requires continuous detected RFID contact. The two-second
  occupation grace period does not count as continued scanning.

These choices are documented and tested where they affect observable state.

## Live RMUC 2027 control and structures

`rmuc_2027::match_rules::Match` owns the explicit RMUC 2027 V1.0.0 subset:
countdown/round clock, per-robot RFID expiry, eligibility and mutual exclusion,
Fortress timers, control, base/outpost HP, physical ownership and rebuilds,
permanent rotor stops, attack totals and single-round results. Native combat
adds 10 Hz heat/cooling, launch locks and allowance, delayed logistics, healing,
respawn/weakness/invincibility, combat XP and Infantry performance. The initial
level cap starts at five and virtual assembly raises it to seven/ten. Adaptive
observations and statistics are carried but reinforcement effects are inactive.
The user-selected permanent heat-lock boundary is Figure 5-1's `>=`.
It receives
physical contact observations and explicit nanosecond durations only.
`rm-simulator-world::rmuc_2027::State` supplies source detector geometry and
height-aware polygon contacts, with the whole state in `FieldRestore`.

The [field guide](rmuc2027-field.md#enable-the-live-controlstructure-subset)
documents startup, approximations, tie ordering and intentionally unsupported
mechanics. Existing 2026 `Game` admission, balance tables and referee behavior
remain unchanged. 2027 component types always compile for a stable protocol
schema; the world checks `season-2027` before creation or restore.

The native `FieldSnapshot.rmuc_2027` presentation state remains available when a
viewer drops `restore`. Checkpoints encode the authority once and reconstruct
this view; restore and checkpoint encoding reject mismatched copies.

## RMUC 2027 virtual assembly

`rmuc_2027::assembly` implements the progression state machine from UC §5.4.3
and Tables 5-11/12 without robot manipulation. A referee starts an attempt with
team and difficulty, then presses **Finish assembly** before the deadline or
lets it time out. Cancel also releases the core without rewards. Repeated
Finish is refused. Attempts, outcomes, deadlines, completion counts and rewards
are checkpoint authority, including through UDP baselines and prediction.

Difficulties unlock at 0/60/120/180 seconds and require the preceding difficulty
completed. Levels 1–3 can overlap across teams. Level 4 reserves both cores,
requires the other team idle, and can succeed only once per team. Failed attempts
may be retried. Match end cancels active attempts; Reset clears all progression.

The prototype treats Start as instantaneous core positioning and Finish as all
required manipulation, including both level-four cores. **Every difficulty uses
45 seconds**, with completion strictly before the deadline. The manual specifies
45 seconds only for level four; the lower-difficulty limit is a simulator policy.
Pause freezes the timer; SkipTo ages it normally. There is no physical Heavy,
RFID, attachment, paired-step, recovery or invincibility requirement in this mode.

Successful attempts grant Table 5-12 recurring income, Infantry caps and strongest
25%/50% permanent robot/Base/owned-Outpost defense. Income settles on round-aligned
ten-second boundaries, with no terminal 7:00 payment. Completion submitted at a
settlement timestamp affects future payments. Heavy level/HP changes and launcher
permission remain deferred with Heavy equipment support. Assembly can be exercised
without any deployed robots; buttons remain referee-only rather than granting
pilots a way to certify real physical completion.

## RMUC 2027 adaptive armor

Native physical armor contacts use `Match::armor_hit` to record observations,
resolve current victim ownership, classify the selected circle, and multiply
raw damage by Table 5-5's 0.5/1.0/1.5 before attack and defense. Protected and
invincible hits still enter statistics. Each impact retains its classification
at contact time; later selections never rewrite history. The coordinate-free
`Match::hit` remains an explicitly default-armor API for synthetic callers.

One opportunity accumulates each elapsed minute. After ten identified hits on
enemy armor, `SelectReinforcement` spends one and selects the common bottom-left
center for that team's robot/Base/currently owned Outpost plates. This is a
referee proxy for the unavailable playable Radar. Each plate uses its own area;
a common center can lie outside a smaller plate. Selection persists through
respawn and Outpost transfers, with ownership selecting the applicable team's
region. Reset clears it.

**Area is frozen at selection time**, not recomputed on hits or launches.
Reselecting spends another opportunity and snapshots new accuracy. The area
fraction is rounded once to parts per billion; positions use integer micrometres.
Undefined accuracy freezes default armor until another selection, consuming the
opportunity. Circle boundaries are reinforced and circles crossing plate edges
are clipped without resizing. These are adopted user policies, not official
rule clarifications. See [the rules reference](rules-2027.md#uc-adaptive-armor).

The `adaptive_2027` suite covers timing/prerequisites, frozen/undefined areas,
reselection, clipping/boundaries, damage/defense, respawn, Outpost transfer and
malformed restore. A physical world test shoots through native source detectors
inside/outside the selected region, then repeats after restore; the wire codec
checks frozen authority without floating-point checkpoint fields.
