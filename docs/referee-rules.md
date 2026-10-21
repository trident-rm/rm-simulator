<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Referee rule sources

The active runtime implements the documented **RMUC 2027 V1.0.0** subset.
See [rules and adopted policies](rules-2027.md), [gameplay](gameplay.md) and
[source geometry integration](rmuc2027-field.md). RMUL is not admitted as an RMUC
match. Cite the applicable manual section, table or figure when adding constants.

The 2026 University Championship V2.1.0 digest, its limited V2.2.0 Base damage
exception, gameplay engine and referee scheduler are preserved in the published
`rm2026-final` tag. [Archive instructions](seasons.md) explain how to run it.
Main retains shared physical dimensions and serialized records, but cannot run or
restore a 2026 match.

The native Radar implementation follows UC §5.7.4 Tables 5-29–30: integer-tenth
mark progress/history, 500 ms interruption samples, 100/50 opposing/friendly
localization thresholds, 15%/20% vulnerability, union-minute opportunity accrual
and two possible 30-second double activations. Localization-link faults are
sampled independently at 2 Hz (§5.1.5). Decoded information-wave observations use
an explicit simulator receiver proxy; see the [freshness and presentation
policies](gameplay.md#native-radar-and-team-intelligence). Sentry optical tracking
is host control input and never grants Radar progress or triggers a shot.
