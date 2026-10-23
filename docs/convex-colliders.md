<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Explicit convex collision sources

`engine::model::ConvexHull` owns immutable local FLU points in metres. Construction
requires at least four finite points, retaining their order, duplicates and
interior points. Clones share the point allocation. `ShapeDescription::ConvexHull`
explicitly requests the convex envelope; it fills concavities and openings. It
must not be substituted silently for an arbitrary source triangle surface.

`CookedRigidBody` computes the Rapier hull once from those exact points. It checks
normalization range and affine rank before native construction, then rejects
invalid/flat hulls, nonfinite normals or nonpositive/unrepresentable volume.
Backend tolerances can reject very thin or numerically difficult inputs even
when the neutral source constructor accepts them. There is no box, point
perturbation, decimation or adjusted-vertex fallback. Native hull construction
can remove interior/duplicate vertices from its representation; the retained
source points remain unchanged.

Dynamic hull bodies require explicit mass, COM and principal inertia like other
dynamic bodies. The unit-density volume calculation during cooking is only a
geometry validity check. It is not imported into body mass or inertia: installed
colliders retain zero density. This adds no convex decomposition or CAD inertia
estimation and continues to reject dynamic nonconvex triangle surfaces.

## Reuse and inspection

One body can compose multiple local colliders, each with its own pose and contact
parameters. Convex hulls use that existing contract. `CookedAssembly` shares these
cooked shapes across instances and recipe resets without repeating hull creation.
Body/joint bindings and source identities retain their existing lifetime rules.

Static and dynamic geometry captures retain native convex shapes for clearance
and nearest-hit queries. Triangle expansion now includes their boundaries, so
collision debug drawing/export does not omit convex parts such as decks and wheel
guards. Rolling LiDAR already queried these native shapes directly; its acquisition
and motion interpolation are unchanged. Expanding debug triangles does not change
the actual solid collider or alter source geometry.

```sh
nix develop -c mbx run -p rm-simulator-physics --locked --example convex_body
nix develop -c mbx test -p rm-simulator-physics --locked --test convex_bodies
```

The example supplies a cube's explicit uniform-box mass and inertia, constructs a
one-link prepared assembly, drops it onto the catch floor, then reconstructs its
recipe and reacquires the binding. It requires no CAD or renderer.

## What the tests establish

The parity fixture reuses all 57 exact hull point sets from the default omni
chassis deck and bumpers. It compares the described/cooked path against direct
`SharedShape::convex_hull` construction, at identity and at combined local/body
rotations/translations. Triangles, bounds, volume and per-face ray hits/normals
match within the declared numerical tolerances. Other tests exercise dynamic
settling, projectile momentum transfer, independent worlds, reset replay, source
retention and shared cooked resources. Degenerate and selected unrepresentable
inputs fail cooking rather than selecting another shape.

These checks establish parity with the existing native hull path, not independent
proof of the backend's hull algorithm or robustness for every finite point cloud.
They do not migrate chassis construction, wheel forces, material combination
rules, body mass policy or competition rules. In particular the existing chassis
derives mass distribution from its collider setup; the generic model still needs
an explicit equivalent policy before a whole-chassis migration. CCD/damping,
sensor/visual mounts and participant/contact separation remain further work in
the [physics migration plan](physics-migration.md).
