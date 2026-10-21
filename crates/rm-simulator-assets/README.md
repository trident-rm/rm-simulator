<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# rm-simulator-assets

Verified field-package import without a server, match runtime or renderer.
Physics supplies FLU poses and gameplay supplies season/rule identities; this
crate never depends on world, Bevy, GNS or the server.

- `cad_assets`: manifest discovery, placements, SHA-256 verification and loading.
- `collision_mesh`: indexed GLB triangles, node transforms and ground lookup.
- `math`: quaternion/matrix helpers for source frames.
- `semantics`: checked source-part and articulation metadata.
- `rmuc_2027`: source binding records and geometry validation.

`cad_assets::load` validates the standalone source schema. The server keeps its
existing import paths and supplies native runtime validation through
`load_with_bindings`; antenna calibration and mechanism creation remain there.
The package format and original ownership notices are unchanged.

Run `cargo test -p rm-simulator-assets --locked`. See
[embedding](../../docs/simulation-embedding.md) and
[field packages](../../docs/field-package.md) for composition and asset contracts.
