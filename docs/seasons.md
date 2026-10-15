<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Season builds

The current engine, field package and network schema implement the documented
RMUC 2026 subset. RM2027 has an explicit identity and feature so its implementation
can develop alongside RM2026. The feature includes an isolated
[RMUC 2027 control component](gameplay.md#rmuc-2027-control-component), but does
not enable playable RM2027 matches. The [2027 rules reference](rules-2027.md) records the new manuals;
[gameplay coverage](gameplay.md) describes the existing engine.

## Cargo features

`rm-simulator-gameplay`, `rm-simulator-world`, `rm-simulator-server` and
`rm-simulator-app` expose additive `season-2026` and `season-2027` features.
`season-2026` is the default. Each layer forwards its enabled seasons and disables
dependency defaults, so a 2027-only app build does not silently re-enable 2026
through its server or world dependency. Physics and rendering have no season
features: they stay reusable and receive explicit state from their callers.

| Selection | Build status | Sessions available today |
|---|---|---|
| Default or `season-2026` | Supported | Existing RMUC 2026 subset |
| `season-2027` without defaults | Isolated control component | No full matches; creation returns an error |
| Both | Supported | Existing RMUC 2026 subset; RM2027 creation returns an error |
| Neither | Infrastructure only | None; creation returns an error |

Build a binary with the desired feature set:

```sh
# RM2026, also the normal default build.
cargo build --locked -p rm-simulator-app --no-default-features --features season-2026
# RM2027 infrastructure; this cannot start or join a match yet.
cargo build --locked -p rm-simulator-app --no-default-features --features season-2027
# Both identities in one binary; existing RM2026 play stays available.
cargo build --locked -p rm-simulator-app --no-default-features --features season-2026,season-2027
```

Replace `rm-simulator-app` with `rm-simulator-server` for the headless binary.
Features select availability, not optimization or removal of every legacy
algorithm: the existing modules still compile in the infrastructure build.
There is no `--season` switch yet because there is only one playable season.
The rendering benchmark has no match or season selection.

Cargo unifies features across all selected workspace packages. To obtain a
2027-only build, select the intended package with `-p` and disable its defaults;
adding another package with its defaults can enable 2026 again. `--all-features`
intentionally enables both seasons. Optional features such as `steam` and
`referee-link` remain independent.

## RMUC 2027 map exploration

A combined build can also load the [2027 runtime field](rmuc2027-field.md) as a
rest-pose physics sandbox. Its manifest identifies the season and disables all
legacy match targets, zones and the referee. This is separate from full 2027
match admission; the existing host/session path still requires `season-2026`.

## Admission and compatibility

`rm_simulator_gameplay::season::Season` distinguishes the stable serialized
identities `"2026"` and `"2027"`, even when one is disabled. `Season::ENABLED`
reports enabled identities. `require_playable()` separately checks feature
availability and implemented gameplay, returning `Disabled` or `Unimplemented`.
It never falls back to a different year's mechanics.

`Game::new_for_season` and `Field::new_for_season` validate this before creating
state. Existing `Game::new`, `Field::new` and `Field::restore` explicitly require
RM2026. The dedicated host, CAD simulation builder and app session entry also
check RM2026 before starting their existing session paths. A build without that
feature reports an error instead of starting a 2026 match under a 2027 label.

This foundation does not change the protocol version, checkpoint schema,
compression dictionary or field manifest. Existing checkpoints remain RM2026
only; the serialized `Season` type is not yet included in them. Combined builds
do not currently negotiate different seasons over the network.

## Adding playable RM2027

The year alone is insufficient to identify a ruleset. The supplied 2027 manuals
describe distinct RMUC, RMUL 3v3 and RMUL Infantry 1v1 formats, with different
round lengths, winning conditions and equipment restrictions. Before admitting
RM2027 gameplay, add a ruleset identity containing competition format and manual
revision, and route implementations through that identity. Do not treat the
RMUL manual as an override of RMUC or copy a changed global constant over 2026.

The remaining integration needs to select rules and field geometry together,
carry the ruleset through authoritative configuration, checkpoints and restore,
validate it during network admission, and expose the selection in the CLI and
title screen. Protocol/checkpoint changes need their normal versioning and
dictionary review. Keep the explicit 128 Hz world clock and crate boundaries;
season-specific rule durations remain durations rather than tick-rate changes.

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

When 2027 is enabled, the matrix also runs the isolated control tests without
requiring the 2026 engine.

The season matrix is included in `just verify` and Linux native validation.
It complements the normal workspace checks; it does not claim app startup,
network negotiation or implemented RM2027 rule coverage.
