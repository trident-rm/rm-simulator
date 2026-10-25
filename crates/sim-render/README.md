<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# sim-render

Passive Bevy visuals with caller-owned identities, FLU poses and shared assets.
`Visuals` installs meshes/scenes and buffers sparse pose/visibility changes.
Camera appearance and generated emissive material registration are explicit.
The renderer owns no physics, rules, host clock, scenario loader or player input.

There are no RoboMaster types, plugins or default game features. `rm-sim-render`
depends on this crate for its armor, chassis, equipment and match-scene visuals.
Its existing generic imports re-export these same types for compatibility.

```sh
cargo test -p sim-render --locked
```
