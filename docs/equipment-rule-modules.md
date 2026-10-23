<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Standalone native equipment rules

`rm_sim_gameplay::rmuc_2027::equipment` exposes borrowed `Outposts` and
`Bases` views used by the live `match_rules::Match`. They operate on the same
caller-owned state fields and the existing `Control` territory authority.
They introduce no serialized records, clock, physics, transport or dependencies.
For a complete match, continue to use `Match` or a trusted world session.

```sh
nix develop -c env CARGO_TARGET_DIR=/tmp/rm-equipment-target \
  cargo run --locked -p rm-sim-gameplay --example equipment
nix develop -c env CARGO_TARGET_DIR=/tmp/rm-equipment-target \
  cargo test --locked -p rm-sim-gameplay --test equipment_2027
```

`Outposts` borrows both HP values, permanent rotor-stop timestamps and mutable
`Control`. Damage executes the native UC §5.6.1 / Table 5-21 destruction
transition, including an immediate 500-HP ownership-transfer rebuild when
appropriate. Operator HP overrides cannot resurrect destroyed outposts.
Advance `Control` explicitly in nanoseconds and pass its returned events to
`Outposts::apply_events` immediately, before further observations. Call
`settle_rotors` after control events and armor-opening updates. Split explicit
advances at three minutes and settle there to preserve the exact stop timestamp;
also settle each newly observed armor opening at its observation time. A rebuild
preserves the first stop timestamp. The three-minute stop and the opposing
Base armor condition use current physical ownership, as in the live match.

`Bases` borrows HP, historical minimum HP and immutable `Control`. Its damage
method checks native outpost-territory protection, caps loss at remaining HP
and retains the minimum through repairs. `set_hp` is an operator override
without attack credit. Armor-opening progress, Heavy deficit history, rewards,
logistics effects and victory settlement remain responsibilities of `Match`.

These are trusted rule adapters. Supply coherent state from a fresh native
configuration or its checkpoint, and submit only eligible, already-adjusted
damage in HP. `Match` still checks phase, launcher permissions, shooter team,
adaptive armor, Rune and assembly modifiers before invoking the same adapter.
It then credits damage and experience, resolves activation revocation and
settles a lethal Base result in its original order. Standalone callers own
these surrounding policies; the borrowed views do not reproduce them.

The existing `rmuc_2027::rune::Rune` is already the pure native equipment
authority. Reuse `new`, `activate`, `hit`, `boundary` and `settle` directly.
Settle at zero to grant the first opportunity. To advance through several
deadlines, repeatedly advance by the smaller of `boundary(now_ns)` and the
remaining duration, then call `settle` at that timestamp. Advancing directly
past intermediate boundaries can change transition epochs. Callers gate
eligible 17 mm ground-robot detections before `hit`. The enclosing match owns
team experience awards and application of benefits to other equipment.

The standalone example exercises Outpost destruction/rebuild, Base protection
loss and a Small Rune activation using explicit time. Parity tests compare
borrowed adapters against live match transitions, including overkill,
ownership-dependent credit, permanent stops, HP minima, JSON continuation and
Small/Big Rune benefits and hidden timers. They complement existing full-match
tests for combat rewards, permissions and timing partition invariance.

No rule constants or interpretations change. Sources and adopted policies are
in [the rules guide](rules-2027.md); the full native integration is documented
in [gameplay](gameplay.md).
