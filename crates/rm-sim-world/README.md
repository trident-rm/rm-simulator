<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# rm-sim-world

The complete `Field` facade combines native 2027 competition rules with shared chassis
and projectile physics on an explicit clock (**128 Hz** by default, configurable
for in-process callers through `FieldConfig::step_duration`). It never reads host time
or depends on Bevy, rendering, transport or background workers.

`FieldConfig::default()` creates a bare physical world. A match requires native
`rmuc_2027::Config` detector/zone bindings for RMUC, or an explicit
`rmul_2027` format for the approximate RMUL practice world. The two are
mutually exclusive in configuration and checkpoints. Historical rule configurations are
rejected; use `rm2026-final` to run a 2026 match.

| Module | Responsibility |
|---|---|
| `session` | Caller-owned stepping, addressed controls, privileged truth, shared-geometry checkpoints and reset |
| `lib` | Field construction, tick ordering, commands, snapshots and complete restore |
| `rmuc_2027` | Source detector/zone adapter and native RMUC match authority |
| `rmul_2027` | Distinct RMUL authority and explicit practice RFID/installation adapter |
| `assembly`, `energy_unit` | Persistent articulated manipulation and physical Units |
| `scoring` | Contact detection and native rule consequences |
| `referee` | Shared commands and serialized presentation records; no historical scheduler |
| `rune`, `outpost`, `base`, `zones` | Shared records and pure geometric helpers |
| `chassis`, `projectile` | Physical exports and serialized contact reports |

`Field::restore` reconstructs the whole field using its checkpoint plus shared
`StaticGeometry` and floor height. Solver contact manifolds and warm starts are
rebuilt. With no native authority or physical bodies, stepping jumps directly to
the requested tick; active matches and bodies step tick by tick.

Run `just world-test` for native scoring, restore, partitioning, manipulation and
shared physical integration tests. `examples/step_cost.rs` measures stepping.
See [architecture](../../docs/architecture-refactor.md),
[gameplay](../../docs/gameplay.md), [physics reuse](../../docs/physics-reuse.md),
and [embedding](../../docs/simulation-embedding.md).
