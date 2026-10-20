<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# rm-simulator-gameplay

Deterministic RMUC 2027 V1.0.0 rules without physics, CAD, networking, rendering
or host time. `rmuc_2027::match_rules::Match` owns the running authority; callers
supply observations and explicit nanosecond durations.

| Module | Responsibility |
|---|---|
| `rmuc_2027` | Territory control, combat, economy, field benefits, traversal, Rune, assembly and adaptive armor |
| `state`, `performance` | Shared identities, serialized records and physical model descriptors |
| `season` | Source identities and current-runtime admission |
| `coverage` | Shared mechanic identifiers |

The 2026 engine and policy implementations are archived at `rm2026-final`.
Shared data records do not provide a runnable 2026 game.

Run `just gameplay-test` for native rules and partitioning tests, and
`just gameplay-demo` for a round with capture, scoring and Base destruction.
`cargo run -p rm-simulator-gameplay --release --example benchmark --locked`
measures native round stepping without external assets.

See [gameplay](../../docs/gameplay.md), [rules](../../docs/rules-2027.md) and
[archive instructions](../../docs/seasons.md). Dependencies remain `serde` and
`thiserror`; no other simulator crate is permitted.
