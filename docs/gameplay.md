<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Native 2027 competition gameplay

`rm-sim-gameplay` runs 2027 V1.0.0 rules without Bevy, Rapier, CAD,
networking or host time. `rmuc_2027::match_rules::Match` owns RMUC; the separate
`rmul_2027::Match` owns RMUL 3v3 and Infantry 1v1. See
[RMUL support and limitations](rmul-2027.md). The 2026 engine is archived at `rm2026-final`.
See [manual interpretations](rules-2027.md) and [runtime admission](seasons.md).

```sh
just gameplay-test
just gameplay-demo
just control-test
```

## Integration and clock

The engine accepts explicit nanosecond durations and external observations.
The world adapter runs it alongside shared physics at a fixed 128 Hz. Never read
a host clock in either crate. Snapshots preserve hidden rule state so restore and
prediction follow the same command/step paths as the host.

`FieldConfig::default()` is a bare physical world. Native source bindings are
required for RMUC match construction. RMUL opts into separate authority on
explicitly approximate practice geometry; historical rule configurations are rejected.
Shared identity, command and presentation records remain data, not a second
engine. Remaining unsupported rules are listed in the
[field guide](rmuc2027-field.md) and [rules reference](rules-2027.md).

## RMUC 2027 control component

`rm_sim_gameplay::rmuc_2027::Control` implements the UC V1.0.0 territory
state machine. The native match adapter consumes it; live world admission requires
native source bindings.

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
cargo test --locked -p rm-sim-gameplay --lib rmuc_2027
cargo test --locked -p rm-sim-gameplay --doc rmuc_2027
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

## Live RMUC 2027 control and structures

`rmuc_2027::match_rules::Match` owns the explicit RMUC 2027 V1.0.0 subset:
countdown/round clock, per-robot RFID expiry, eligibility and mutual exclusion,
Fortress timers, control, base/outpost HP, physical ownership and rebuilds,
permanent rotor stops, attack totals and single-round results. Native combat
adds 10 Hz heat/cooling, launch locks and allowance, delayed logistics, healing,
respawn/weakness/invincibility, combat XP and Infantry performance. The initial
level cap starts at five and virtual assembly raises it to seven/ten. Adaptive
reinforcement uses frozen selection-time areas and native observations/statistics.
The user-selected permanent heat-lock boundary is Figure 5-1's `>=`.
It receives
physical contact observations and explicit nanosecond durations only.
`rm-sim-world::rmuc_2027::State` supplies source detector geometry and
height-aware polygon contacts, with the whole state in `FieldRestore`.

The [field guide](rmuc2027-field.md#enable-the-live-controlstructure-subset)
documents startup, approximations, tie ordering and intentionally unsupported
mechanics. Shared serialized records and physical model descriptors remain available;
the 2026 rule engine and referee are archived at `rm2026-final`. Native creation
and restore validate the complete 2027 authority.

The native `FieldSnapshot.rmuc_2027` presentation state remains available when a
viewer drops `restore`. Checkpoints encode the authority once and reconstruct
this view; restore and checkpoint encoding reject mismatched copies.

## RMUC 2027 virtual assembly

The separate [Cartesian assembly workbench](assembly-workbench.md) now evaluates
mechanism sensors and ordered manipulation. It does not yet replace the live
referee-button prototype below or grant its own match rewards.

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
settlement timestamp affects future payments. Full Heavy upgrades and physical launcher installation remain deferred; the
[temporary Hero conversion](#temporary-hero-conversion-for-heavy) supports unlocked combat. Assembly can be exercised
without any deployed robots; buttons remain referee-only rather than granting
pilots a way to certify real physical completion. The separate source-bound
pilot prototype now certifies L1–L3 through the manipulation evaluator; it starts
and settles these same native attempts, preserving match unlocks and rewards.

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

## Sentry modes, postures and interventions

UC Table 3-6/§5.7.2 run through `rmuc_2027::sentry`. New Sentries default to
Automatic (400 HP, 260 heat, 30/s cooling, 100 W before posture); SemiAutomatic
uses 200 HP, 100 heat, 10/s and 60 W. Select with `SetSentryMode` while Idle;
the simulator's countdown is the pre-self-check selection cutoff. Model choice
and the navigation controller do not choose this rule identity. Mode selection
survives starting another round; posture usage resets each round.

Movement is the initial posture. Attack/Defense/Movement implement Table 5-28,
including five-second selection cooldowns, strict `>180 s` cumulative weakening,
independent 15 s enhanced budgets and automatic enhanced expiry. Enhanced time
also counts toward ordinary posture use. Simulator policy: usage pauses while
defeated or main-controller offline; neither state erases existing usage or
cooldown deadlines. Voluntary ordinary/enhanced changes share one cooldown;
automatic expiry never waits for or starts a cooldown. The first switch is
available immediately. Cooling reductions apply after other benefits and round
down to whole heat units per second. Attack cooling competes with other cooling
benefits; enhanced Attack keeps heat zero without clearing independent permanent
launcher locks. `Match::power_limit_w` includes posture; `Robot::stats` retains
base performance and the paid-respawn modifier.

Owned `Command::SentryAction` requests are Sentry-originated operations.
Privileged `RefereeCommand::SentryAction { commander: true, ... }` represents the
commander and charges Automatic Sentries 50 gold per accepted operation.
Posture, ammunition, delayed HP, immediate respawn and normal-respawn confirmation
use the same atomic transaction; purchase prices are additional. Rejected
operations spend nothing. Sentries wait at a completed normal respawn bar for
`ConfirmRespawn`; this interprets §5.7.2's confirmation operation as the final
revival gate, while the general §5.3.2 progress starts immediately on death.
Other ground roles retain automatic normal revival.

Minimap routing uses the team operator as a commander proxy: forwarding an
accepted marker to Automatic Sentry costs 50 gold; its own operator and
SemiAutomatic Sentry incur no intervention fee. All recipients and total costs
are validated before committing an event. Offline main controllers cannot send
or receive these communications. This does not add autonomous combat.

The in-game Sentry panel offers pre-round modes, ordinary/enhanced postures; the defeat dialog offers
normal-respawn confirmation and paid revival. The HTTP referee panel offers paid commander posture
requests. The HUD shows mode/posture, native power limit, buffer and drive cuts.

## Referee power and module faults

`rmuc_2027::power` implements UC §5.1.4/Figure 5-2's 10 Hz 60 J referee buffer
and five-second drive cut. It is separate from hardware supercapacitor energy.
The physical wheel model reports nonnegative electrical load (positive mechanical
work plus copper loss, after torque scaling, without regeneration credit).
Simulator metering policy: the last completed physics slice is held as the
10 Hz meter sample. Integer milliwatts/microjoules and explicit round boundaries
make buffer settlement restorable and independent of step partitioning.

The world's next motor budget is bounded by both configured hardware power and
`native limit + remaining buffer / 0.1 s`. This permits buffered bursts rather
than silently hard-clamping every request to the steady-state rule limit. Actual
buffer exhaustion cuts motor torque; an over-power cut leaves the gimbal powered.
The buffer refills below the rule limit, including during a cut. Death restores
60 J. No rule enforcement is added to bare physical worlds or Idle practice.

`SetRobotModules` supplies pending main/speed/armor-count/supercapacitor-manager
connectivity; the engine samples it at 2 Hz (§5.1.5). Each disconnected armor or
supercapacitor manager costs 4 HP per sample. Main-controller offline instead
costs 2.5% maximum HP per sample (fractional carry retained), cuts drive/gimbal/
fold/arm actuation, suppresses ordinary hits and other module damage, disables
RFID and map communication and suspends respawn. Infantry XP/levels still accrue.
Speed-monitor reconnection clears only that module's launcher lock. Localization
accuracy consequences remain deferred with the Radar observation system.

### Native Radar and team intelligence

UC §5.7.4 Tables 5-29–30 are native authority in `rmuc_2027::radar`:
position reports update integer-tenth progress/history, interrupted streams
produce wrong samples every 500 ms, and opposing/friendly localization thresholds
are 100/50. Enemy vulnerability is 15% at 100 and 20% at 120, combined with
other same-kind vulnerability by taking the strongest source. Each union minute
with at least one living opposing robot vulnerable earns one double opportunity;
up to two 30-second activations may be spent per round. Simultaneous vulnerable
robots never multiply elapsed time. Interference difficulty suppresses the
handicapped observer's vulnerability and opportunity accrual while preserving
marking progress and localization. Revival preserves progress and resets history;
reports and missing-data sampling continue while defeated.

The world evaluates report error against authoritative positions, quantized to
millimetres before the strict 800/1600 mm comparisons. Localization-module
connectivity is independently sampled at 2 Hz; missing localization forces
opposing reports accurate and friendly reports wrong (§5.1.5). Main-controller
loss is a separate condition. All marks, observations, timers and pending module
inputs survive complete restore.

`server::intelligence::Contact` separates position provenance from optional HP,
HP ceiling, ammunition and defense. Radar position alone never discloses those
fields. Independent decoded information is accepted through a typed referee
injection, with a one-second simulator freshness policy. Unlocalized last reports
fade after 500 ms and disappear after two seconds; Table 5-30 localization remains
available while progress meets its threshold. Enemy health in the scoreboard and
roster also follows these observed fields. Full prediction checkpoints still
contain authority: this is a presentation boundary, not an anti-cheat security
boundary. The unspecified radio waveform, key decoder and unavailable 2027
communication packet layout are not implemented; interference can be set by the
referee as a decoder stand-in.

### Optional Sentry scanning and tracking

`server::sentry_controller` runs host-side optical acquisition at 16 Hz and applies
its aim/rotation outputs after ordinary pilot inputs on every 128 Hz tick. Owned
Sentries opt in through **Auto aim + sweep** and opt out through **Manual aim**;
operator-spawned navigation Sentries opt in by default. Disabling it immediately
returns aim and chassis rotation to normal input. **StopSentry** stops navigation
and disables the sweep; a later navigation goal does not implicitly re-enable it.

The simulator camera acquires opposing, living armor faces within 20 m,
±0.70 rad yaw and ±0.5 rad pitch of the actual gimbal. Scenery rays, front-facing
armor and approximate intervening robot bodies reject occluded targets. It keeps
its visible target, otherwise selects the nearest visible enemy. Detection reports
position and identity only, with a 250 ms freshness limit; it does not grant Radar
progress, infer health or use the decoded/global position list to aim through
walls. This models ideal optical recognition, not a camera-image detector.

An unobstructed chassis sweeps at 0.8 rad/s. Without a visible target the gimbal
sweeps independently in yaw and oscillates in pitch; with a target it tracks the
visible armor center using the existing physical motor limits. This first pass
is geometric tracking, without projectile lead/drag solving or a trigger policy.
The controller never fires. Pilot movement and firing remain ordinary protocol
inputs. Source stow and tunnel navigation take priority over aiming and rotation;
passage yaw remains locked until open clearance, nearby geometry/robots prevent
free rotation, and countdown/finished phases suspend scanning. The controller is
host input, not another world or rule scheduler; diagnostics travel in protocol
78 snapshots and physical commands remain in normal checkpoints.
