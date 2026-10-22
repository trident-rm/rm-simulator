<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Active runtime and archived seasons

The application and headless CAD host now target **RMUC 2027 V1.0.0**. There
are no season Cargo features, combined-season binaries, or season build matrix.
Normal builds and `--no-default-features` builds include the same native rules;
`steam` and `referee-link` remain independent optional app integrations.

```sh
cargo build --locked -p rm-simulator-app
cargo build --locked -p rm-simulator-server
just runtime-test
```

## Preserved 2026 runtime

Annotated tag **`rm2026-final`** points to `89ebbdd`, the final revision supporting
2026 sessions. It includes the latest shared physics, robot models, networking
and UI as well as both season implementations. It is a source archive, not a
published binary release. Use a separate checkout to run it:

```sh
git worktree add ../rm-simulator-2026 rm2026-final
```

The tag is published on origin. Existing 2026 releases are unchanged.
The old `season-2026` and `season-2027` arguments are no longer accepted on main.

## Runtime admission and assets

Discovery checks only `field-2027/` beside an installed `bin/` directory, then
`field-2027/` in the checkout. `--cad-assets` selects an explicit package, but
2026 packages are rejected before session construction, with the archive tag in
the error. There is no fallback to a historical home-directory installation.
Release archives contain only the 2027 runtime field.

Live matches require checksummed native structure/rule bindings. Unbound 2027
rest-pose exports may be inspected in practice with `--no-referee`; they do not
acquire a legacy referee. The [2027 field guide](rmuc2027-field.md) documents the
bound package and supported mechanics. The [rules reference](rules-2027.md)
records the manual revision and adopted policies. RMUL remains unsupported.

## Shared types and physics

Main no longer compiles the 2026 gameplay engine, referee scheduler, Rune state
machines, Outpost HP controller, zone effects or base damage tables. Native
`rmuc_2027::match_rules::Match` is the only match authority. `Field::new` defaults
to a bare physical world and rejects historical rule configurations. Source
identities, serialized presentation/command records, robot descriptors, armor
geometry and reusable physics remain shared. Historical records do not execute
rules and cannot restore an old match.

The checkpoint schema is protocol **82**. Both peers must use the same version;
old checkpoints containing removed rule state cannot be replayed on main.

The historical `field/` package and its inventory have been removed from main;
retrieve them from `rm2026-final` when needed. The local cleanup preserves a copy
under ignored `local-assets/field-2026-archive/`. Optional historical terrain
physics tests accept external fixture paths through
`RM_SIMULATOR_CAD_PHYSICS_FIXTURES` (platform path-list syntax); no runtime or
test loader discovers that scratch directory automatically.
Only `field-2027/` is tracked, verified and staged by the active pipeline.

## Verification

`just verify` covers the normal workspace and optional integrations.
`just runtime-test` also checks that Cargo metadata exposes no season switches,
then runs native gameplay/world and server admission tests with default
features disabled. The workspace suite also covers application admission. Regression tests verify rejection before terrain processing,
bound-match requirements, rest-pose practice, native authority restore, and
installed package discovery without a 2026 fallback.

Shared mechanics still run on the explicit 128 Hz world clock. This change does
not fill remaining gameplay gaps. Future rules work
extends the native 2027 authority rather than introducing another season flag.
