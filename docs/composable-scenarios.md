<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Composable physical scenarios

`rm_simulator_assets::scenarios::composition` combines existing checked equipment,
static terrain, robot spawns and explicit antenna metadata into an immutable
`ComposedScenario`. It creates ordinary `RobotSession`s, with no match authority,
network protocol, implicit controller or automatic mechanism scheduler. Existing
[`Scenario` presets](standalone-scenarios.md) and their session API stay available.

Run the example without CAD, or compose two complete source Outposts:

```sh
cargo run -p rm-simulator-assets --example composed_scenario --locked
cargo run -p rm-simulator-assets --example composed_scenario --locked -- field-2027
```

The example reports preparation, initial session creation and the mean of 100
recipe resets. Preparation timing excludes package verification and source preset
loading. It demonstrates explicit robot/contact IDs and RFID metadata; it does
not implement a traversal reward or Outpost rules.

## Prepare once

Build a `Composition`, then call `prepare()`. The preparation validates unique
instance IDs, finite unit rigid poses, target mapping cardinality and uniqueness,
source detector metadata, mechanism channel independence, RFID polygon/prism
metadata and physical robot configurations. Errors return no prepared definition.
Self-intersecting/degenerate RFID polygons and repeated vertices are rejected.
A `ComponentDefinition` comes from `Scenario::load(...).into()`,
`Scenario::tunnel(...).into()`, or `ComponentDefinition::new` with an existing
`PlacedComponent`, explicit detector definitions and an optional checked visual.
The latter validates physical metadata; its caller obtains source geometry and
file provenance through the existing verified package loader.

```rust
use rm_simulator_assets::scenarios::{Scenario, composition::*};
use rm_simulator_physics::{Pose, StepDuration, arena::TunnelModel};

let tunnel = Scenario::tunnel(TunnelModel {
    pose: Pose::default(),
    opening_m: [2., 1., 0.65],
    wall_thickness_m: 0.1,
})?;
let definition = Composition {
    components: vec![ComponentInstance {
        id: InstanceId(10),
        source: tunnel.into(),
        placement: Pose::yawed([3., 0., 0.], 0.4),
        targets: vec![],
    }],
    catch_floor_m: Some(0.),
    ..Default::default()
}.prepare()?;
let mut session = definition.session(StepDuration::HZ_1000, Default::default())?;
session.step(100).map_err(anyhow::Error::msg)?;
session.reset().map_err(anyhow::Error::msg)?;
assert_eq!(session.stamp().time_ns, 0);
# Ok::<(), anyhow::Error>(())
```

`TerrainInstance` accepts a `StaticGeometry` capture containing explicit static
triangles/boxes and rejects moving channels. Source catch planes, projectile
bounds and collision-group policy are not imported. Composition cooks its shapes
with the ordinary static-mesh collision policy. `catch_floor_m: Some(z)` chooses
an explicit flat catch plane; `None` removes it and requires caller terrain to
provide any desired ground. No default terrain, boundary or floor-height inference
is added. The illustrative tunnel remains an approximate three-box fixture.

Preparation expands source shapes into triangles, applies placement and cooks new
BVHs once. This temporarily needs source and transformed triangle buffers, and it
has a measurable cost even when the source was already cooked. Source meshes keep
their surfaces; boxes become triangle surfaces. Prepared sessions and resets clone
shared immutable shape storage, rebuild world bodies and broad-phase state, and
do not repeat expansion, CAD parsing or cooking. They retain independent mutable
robot/projectile/contact state. Preparing a new definition is required to change
component topology or placements; changing a definition never mutates an existing
session. There is no arbitrary running-world physical checkpoint API.

## Placement and identity

`ComponentInstance::placement` is an **additional FLU rigid transform**, applied
after its checked source placement. For a source-world point `p`, the result is
`t + R p`. Arbitrary pitch/roll/yaw are supported. Fixed and moving vertices,
detector frames, rotor pivots and axes, translation offset vectors and visual
roots are transformed consistently. Translation offsets receive `R`; pivots and
vertices receive `t + R p`. Visual roots retain the original glTF basis exactly
once. A plain `Pose::at` is correct for this additional FLU transform; it is not
a replacement for the original exported glTF root.

`InstanceId(u64)` is caller-owned and unique across components, terrain, robots
and RFID volumes. It does not encode a team, competition role or insertion order.
Prepared component/terrain placement records retain geometry-only instance IDs.
Robot IDs map explicitly to list-allocated physical chassis IDs through
`robot_identities()`; those physical IDs repeat after recipe reset.

Each component supplies one explicit `ArmorTarget` per original source target,
in source order. Mappings preserve variant and local plate/face/blade index while
allowing the parent index to be renamed. Every physical target identity must be
unique across the composition. Component targets cannot use `Chassis` identities,
which belong to spawned robots. `target_identities()` and `source_target()` retain
the exact source identity and owning instance for interpreting raw contacts.
Renaming an Outpost index to 101 does not make it a native match team. The caller
must explicitly map task/gameplay policy through the source/instance metadata;
passing renamed physical labels directly to a native match adapter is incorrect.

## Prescribed motion limits

The current physics `MechanismState` addresses fixed `(kind, side)` channels:
two Outpost angles, two Rune angles, two Base fractions, two DartDoor booleans,
and one DartTarget fraction shared across sides. Composition preserves the source
channels and **rejects overlaps between independent component instances**. It does
not silently couple repeated equipment or allocate new physics motion channels.
Several parts within one source component may legitimately share a channel.

Thus one red and one blue source Outpost can coexist, but two red source Outposts
are rejected even with distinct target IDs. One complete Rune occupies both Rune
channels, so a second complete Rune is rejected. DartTarget overlap is rejected
even when two components use different team labels. Relocating equipment across
the field does not change its source channel or infer new ownership. Supporting
more independently driven repetitions needs a future per-instance physics motion
interface; composition does not claim that capability.

Resolve targets and scenery from the same validated coordinates before stepping:

```rust,ignore
definition.targets_at(&state, &mut target_scratch)?;
session.physics_mut().sync_mechanisms(&state);
let contacts = session.step_with_targets(&target_scratch)?;
```

Target output validation precedes mutation. Session creation/reset retains its
initial mechanism coordinates and topology. Controllers, RNG, sensor interpolation
and acquisition schedules, task rewards and rule state remain caller-owned.

## Explicit RFID metadata

`RfidVolume` is an opaque label and a local polygon prism with a rigid FLU pose
and inclusive local z range. `contains(world_position_m)` transforms the query
into the prism frame and uses the existing `ZoneFootprint` polygon edge policy;
polygon edges are not universally inclusive. Arbitrary prism tilt is supported.
This is an ideal geometric antenna approximation, not an RF propagation model.

Presets never generate RFID automatically, and a tunnel mesh does not imply a
traversal card. The caller supplies all antenna metadata and associates IDs/labels
with its own existing rule adapter explicitly. There is no competing ownership,
team, buff, HP, grace-period or reward scheduler in this layer.

## Validation

CAD-free tests cover two tunnel instances plus terrain/robot/RFID, stable identity
and topology after reset, arbitrary rotated rotor/slide/detector/visual frames,
invalid poses/configuration/mapping cardinality, duplicate target/instance IDs,
transactional target-output rejection and mechanism overlap rejection. The source
check is explicit because ordinary tests must work without local CAD:

```sh
cargo test -p rm-simulator-assets --locked
cargo test -p rm-simulator-assets --locked checked_source_outposts -- --ignored --nocapture
```

The source test verifies two approved Outposts with six uniquely remapped targets,
two visuals and reset. It does not add field assets or change native match rules.
