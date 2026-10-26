<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# sim-physics

Generic concrete Rapier physics over solver-free `sim-engine` descriptions.
There are no RoboMaster participants, rules, assets, sensor schedules, networking,
renderer or host clock, including in the crate's tests and examples.

- `cooking`: shared prepared mesh, body and assembly resources.
- `runtime`: body/joint installation, branded bindings, held commands, exact
  integration slices, read-only geometry and optional bounded physical contacts.
- `runtime::session::PhysicalSession`: caller-owned time and transactional
  initial-scene reconstruction with a typed recipe.
- `sensors::carrier`: copied carrier/mount poses and explicit continuity evidence,
  without a measured sensor, acquisition schedule or live world ownership.
- `backend`: explicitly enabled Rapier access for trusted participant adapters.

The scene starts empty, without a floor. Bodies that interact share one runtime.
Prepare immutable resources once and install them into independent runtimes or
recipe resets. Reset invalidates bindings and caller interpolation history; it
does not continue arbitrary solver state.

The default build has no features. Ordinary consumers use opaque model/body/joint
bindings, commands, compact state and captures. The `controller_bench` example
constructs an XY positioning robot, runs external force feedback and resets:

```sh
cargo run -p sim-physics --release --example controller_bench --locked -- 1000
cargo test -p sim-physics --no-default-features --locked
```

## Native physical extensions

The opt-in `rapier-extension` feature exposes concrete native access and slice
phases through `backend::RuntimeExt`, along with cooked-resource and capture
access traits. `rm-sim-physics` uses it for chassis wheel forces, prescribed
equipment motion and pre-impact contact sampling. Both generic and RM paths use
the same scene, resource builders and integration functions; no engine is copied.

This is a trusted backend-specific API. Native adapters own their handles and
must preserve integration timing and the generic endpoint, wrench, joint and
contact phases. They must not remove generic bound bodies via native handles.
The extension does not grant remote command authority or provide a new solver
interface, plugin loader, callback scheduler or arbitrary checkpoint API.

The separate `test-support` feature permits explicit epoch-exhaustion fault
injection for downstream regression tests. It is not part of normal operation.

`rm-sim-physics` keeps its existing public paths as re-exports where applicable.
Generic applications should depend on this crate directly. See
[physical runtime](../../docs/physical-runtime.md) for ownership and
[controller bench](../../docs/controller-bench.md) for the first measured consumer.

Cooking returns `CookError`; runtime commands return `RuntimeError`, including
foreign/stale/duplicate/exhausted `BindingError` categories. Carrier observations
return `CarrierError`, and physical sessions return `SessionError` with nested
causes. Reconstruction recipes return `RuntimeError`. See the
[API migration notes](../../docs/engine-architecture.md#typed-errors-and-pose-admission).
