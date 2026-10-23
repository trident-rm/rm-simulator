<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Native SolidWorks import with sw-rs

The offline SolidWorks workbench accepts the independent `sw-rs` CLI for both
metadata and cached Part meshes. Build the reader separately, then pass its path
to the two preparation scripts. It adds no dependency to simulator crates and
never changes the original CAD or the live match runtime.

## Build and prepare

From the `sw-rs` checkout, build `sw-dump` with `mbx build --release --locked`
(or Cargo if mbx is unavailable). From rm-simulator:

```sh
python3 scripts/extract-solidworks.py /path/to/native/robot.SLDASM \
  --search-root /path/to/native \
  --sw-rs /path/to/sw-rs/target/release/sw-dump \
  --output /path/to/robot.catalogue.json

python3 scripts/prepare-solidworks-project.py /path/to/robot.catalogue.json \
  --source-root /path/to/native \
  --document robot.SLDASM --configuration '默认' \
  --sw-rs /path/to/sw-rs/target/release/sw-dump \
  --output /path/to/robot.project.json
```

`--document` must exactly match the catalogue's relative source path. The source
reader is selected explicitly. Optional `--reader-revision` on extraction records
a declared sw-rs revision alongside the binary SHA-256. The project records a
reader identity and binary hash under `provenance.geometry_reader`; it does not
claim the binary is openswx or sldkit. Legacy `--swx-dump` and `--sldkit` options
remain available for comparison and unsupported files. They are mutually
exclusive with `--sw-rs` at each stage. A sw-rs selection requires the versioned
`sw-rs-metadata-v1` / `sw-rs-geometry-v1` interchange envelope.

Continue through the [reviewed import workbench](solidworks-importer.md).
Reference omissions require its existing explicit `--omit-missing-references`
policy. Groups, source-frame approval, mass review, collision proxies and joint
review remain required before physical export. Existing project/recipe v1 and v2
formats are unchanged, and reimports retain the same review-merging rules.

## Supported profile and validation

Metadata includes native configurations/properties, cached mass tokens, assembly
occurrences and saved transforms, plus cut lists and basic drawing references.
COM is serialized as an x/y/z object for the reviewed-cache path. Previews remain
accessible through the Rust library and are omitted from normal JSON/catalogues.
Reader diagnostics are carried into catalogue diagnostics. Missing/malformed
transforms stay unknown, and unverified cache quantities are not labelled SI.

The initial Part mesh profile reads validated saved DisplayLists descriptors,
including compact/extended headers, reused face classes, normals and strip
winding. It accepts a single native configuration and a single mesh carrier;
ambiguous configurations, missing caches and unsupported formats are errors.
It does not regenerate geometry, evaluate mates, or calculate physical mass.
Source float coordinates follow the audited carrier's metre-to-millimetre
conversion, then the preparer converts millimetres to metres. Source frames and
cache freshness still require independent review.

On the private 340-document EGA corpus, sw-rs preserves all consumed catalogue
source hashes, configurations, reference resolutions, occurrence data and cached
values. Across 295 Parts and 1,373 requested configurations, all 243 meshes
accepted by the old preparation workflow match exactly after reindexing,
including triangle order and signed-zero handling. Four additional Parts yield
valid saved meshes despite unrelated B-Rep byte-ledger failures in the old
reader. These remain display-cache recoveries, not certified B-Rep imports.

The small armor-module assembly prepares with three Part occurrences and two
shared meshes identical to the old workflow, with no project diagnostics.
Real Part geometry matches between the native and WASM entry points. No CAD or
private corpus output is committed. See sw-rs's `VALIDATION.md` and `PORTING.md`
for evidence and the remaining full-reader roadmap.
