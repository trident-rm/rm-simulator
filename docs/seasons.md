<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Season builds

RMUC 2026 retains its documented match engine. RM2027 adds an independent
[control component](gameplay.md#rmuc-2027-control-component) and a playable
[RMUC 2027 V1.0.0 control/structure subset](rmuc2027-field.md#enable-the-live-controlstructure-subset)
through an explicitly bound field package. The standalone legacy `Game` does
not run 2027; no 2026 rules are silently reused. The [2027 rules reference](rules-2027.md) records the new manuals;
[gameplay coverage](gameplay.md) describes the existing engine.

## Cargo features

`rm-simulator-gameplay`, `rm-simulator-world`, `rm-simulator-server` and
`rm-simulator-app` expose additive `season-2026` and `season-2027` features.
The app and server default to both seasons; the standalone gameplay/world
libraries retain their 2026 default. Each layer forwards its enabled seasons and disables
dependency defaults, so a 2027-only app build does not silently re-enable 2026
through its server or world dependency. Physics and rendering have no season
features: they stay reusable and receive explicit state from their callers.

| Selection | Build status | Sessions available today |
|---|---|---|
| `season-2026` without defaults | Supported | Existing RMUC 2026 subset |
| `season-2027` without defaults | Supported | Bound RMUC 2027 control/structure subset |
| Both (normal app/server default) | Supported | Both subsets; bound `field-2027/` selected by default |
| Neither | Infrastructure only | None; creation returns an error |

Build a binary with the desired feature set:

```sh
# Explicit RM2026-only build.
cargo build --locked -p rm-simulator-app --no-default-features --features season-2026
# RM2027 control/structure subset; requires a bound 2027 field package.
cargo build --locked -p rm-simulator-app --no-default-features --features season-2027
# Both subsets in one binary; existing RM2026 play stays available.
cargo build --locked -p rm-simulator-app --no-default-features --features season-2026,season-2027
```

Replace `rm-simulator-app` with `rm-simulator-server` for the headless binary.
Features select availability, not optimization or removal of every legacy
algorithm: the existing modules still compile in the infrastructure build.
The verified package and explicit ruleset binding select the world; there is
no independent `--season` switch that could mislabel an incompatible map.
The rendering benchmark has no match or season selection.

Cargo unifies features across all selected workspace packages. To obtain a
2027-only build, select the intended package with `-p` and disable its defaults;
adding another package with its defaults can enable 2026 again. `--all-features`
intentionally enables both seasons. Optional features such as `steam` and
`referee-link` remain independent.

## RMUC 2027 map exploration

A combined build can also load the [2027 runtime field](rmuc2027-field.md) as a
rest-pose physics sandbox. Its manifest disables legacy targets and rules.
Adding the pinned structure binding enables the separate 2027 authority, with
its own source detector geometry and RFID contacts. A 2027-only build supports
that bound package; the unbound legacy practice sandbox needs a combined build.

## Admission and compatibility

`rm_simulator_gameplay::season::Season` distinguishes the stable serialized
identities `"2026"` and `"2027"`, even when one is disabled. `Season::ENABLED`
reports enabled identities. `require_playable()` separately checks feature
availability and implemented gameplay, returning `Disabled` or `Unimplemented`.
It never falls back to a different year's mechanics.

`Game::new_for_season` still rejects 2027: that API owns the legacy complete
2026 engine. `Season::has_gameplay` and `require_playable` describe that engine.
The world instead admits 2027 only with `FieldConfig::rmuc_2027`, an explicit
source-bound RMUC V1.0.0 config, and the `season-2027` feature. An ordinary 2026
config labelled 2027 is rejected. Default `Field::new` uses its explicit config;
restore selects admission from the native authority carried in `FieldRestore`.
Mixed-season geometry/referee state is rejected.

Protocol **53** carries the complete optional 2027 authority in confirmations,
checkpoints and deltas. Types remain available in every build so the wire shape
is independent of Cargo feature unification; live admission and restore still
check the feature. The existing checkpoint ZSTD dictionary is retained unchanged.
The 2027 sidecar pins competition format, manual revision, source identity and
package hashes. Clients reject a different or unbound local 2027 package.

## Extending RM2027 coverage

The year alone is insufficient to identify a ruleset. The supplied 2027 manuals
describe distinct RMUC, RMUL 3v3 and RMUL Infantry 1v1 formats, with different
round lengths, winning conditions and equipment restrictions. Before admitting
RM2027 gameplay, add a ruleset identity containing competition format and manual
revision, and route implementations through that identity. Do not treat the
RMUL manual as an override of RMUC or copy a changed global constant over 2026.

The control/structure subset now carries ruleset, geometry, robot eligibility,
RFID expiry, contested capture, progress, scores, HP, mutable ownership, result
and permanent rotor stops through snapshots and restore. Remaining subsystems
must extend that native authority rather than enabling the old referee. Keep the
explicit 128 Hz world clock; season durations remain nanoseconds. New protocol
fields need normal versioning and dictionary review.

## Verification

`just season-test` runs admission tests in gameplay and world separately with
neither feature, each individual feature and both features. These checks prove
that disabled and unimplemented seasons are rejected, that stable identities
survive serialization, and that legacy construction and restore cannot bypass
the 2026 feature. It also checks Cargo's resolved app dependency graph to catch
accidental season activation through dependency defaults. The normal
default/all-features suites continue exercising
the complete existing RM2026 implementation. Running the legacy gameplay suite
with 2026 disabled is not an RM2027 rules test: those scenarios require 2026.

When 2027 is enabled, the matrix also runs control/match and live-world adapter
tests, including a 2027-only build without the 2026 engine.

The season matrix is included in `just verify` and Linux native validation.
It complements workspace tests for native restore, network checkpoint codecs,
physics motion and client presentation. Coverage limitations are listed in the
2027 field guide.

Native 2027 combat now includes heat/launch locks, allowance, purchases/healing,
normal/paid respawn and combat XP/performance with the initial level cap of five.
Adaptive launch/hit statistics and opportunities are carried in protocol 55;
reinforcement selection/effects and assembly progression remain unavailable.
See [the lifecycle boundary](rmuc2027-field.md#combat-lifecycle-and-adaptive-armor-infrastructure).

Protocol 55 also carries historical Base HP minima and per-side Fortress reserve
spending. Continuous Base/Vantage/Highland/Fortress field effects are native 2027;
terrain traversal sequences and rune/reinforcement effects remain unavailable.
