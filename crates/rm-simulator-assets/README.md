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

## Checked robot packages

`robots::RobotDescriptor::load` prepares the existing six supported source
profiles without app, world, server or Bevy. Catalogue `VERSION = 1` describes
this Rust interface; original manifest schemas 1–4 remain unchanged. The
immutable descriptor retains source CAD digest assertions, the original manifest
(including notices and visual link metadata), verified neutral GLB bytes, armor
replacement identity and physical `ChassisConfig`. Configuration provenance
explicitly distinguishes schema 1–3 simulator-fitted fallback presets from
schema 4 checksummed source-fitted JSON. The existing articulation configuration
retains source group names and mounts; no universal joint/inertia import is implied.

The descriptor identity hashes the catalogue version, manifest bytes, consumed
file checksums and serialized raw physical configuration, with no installation
path. Including the physical configuration detects code changes to legacy fitted
presets. Caller tuning is separate and must be recorded with the effective
configuration; this identity alone does not certify backend/solver compatibility. Clones share source byte buffers.
Every declared file reference must name a local file and a SHA-256; only neutral
GLB and optional chassis JSON are consumed and checksum-verified. Unconsumed
preview exports and native CAD property caches are outside this trust boundary.
A changed manifest intentionally changes identity even if its semantics match.
Original STEP hashes are checked against the supported profile allowlist; this
neither reopens STEP nor validates source inertias, joints or motor constants.

The descriptor keeps original actuator rates. Callers may clone `chassis()` and
explicitly call `set_transform_duration_s`; app compatibility still chooses
0.5 seconds. App argument resolution retains descriptors locally and sends only
its existing physical configuration through host arguments. The renderer's
existing AssetServer path remains a separate filesystem import; immutable GPU
asset ingestion is not established by this loader.

Run the headless physical example against an installed supported package:

```sh
cargo run -p rm-simulator-assets --example robot_package -- PACKAGE_DIRECTORY
# Explicit simulator actuator tuning, without changing the checked descriptor:
cargo run -p rm-simulator-assets --example robot_package -- PACKAGE_DIRECTORY 0.5
cargo test -p rm-simulator-assets --locked
# External source fixtures are intentionally not embedded or relicensed:
RM_ROBOT_PACKAGES=/path/to/package:/path/to/other-package \
  cargo test -p rm-simulator-assets --locked \
  actual_supported_packages_preserve_raw_configuration_and_content_identity -- --ignored
```

The free `source_chassis`, `replacement_armor` and `is_stair_heavy` functions
preserve narrow legacy helper contracts; they do not certify a complete package.
Use and retain `RobotDescriptor` for complete source/profile/checksum/mount
validation. All physical fitting assumptions remain those of the existing models.
