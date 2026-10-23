<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Development

The developer workflow reference: environment setup, `just` targets, tests,
commit conventions and headless inspection. See [Contributing](../CONTRIBUTING.md)
and [CI and releases](releases.md); the [project README](../README.md) covers play.

## Environment

`rust-toolchain.toml` pins stable Rust with the `rustfmt` and `clippy`
components; Rust stays under rustup even inside the Nix shell.

`.envrc` runs `use flake`, so `direnv allow` loads the shell from `flake.nix`;
`nix develop` enters it explicitly. The shell provides rustup, CMake, Ninja,
pkg-config, Clang/libclang, Protobuf 21, OpenSSL, `just`, Python 3, `cargo-deny`
and `prek`, plus the ALSA, udev, Vulkan, X11 and Wayland libraries on Linux, and
sets `LIBCLANG_PATH`.

The shell sets `CARGO_TARGET_DIR="$PWD/target/nix"`, so Nix builds land in
`target/nix/`, apart from objects linked against Homebrew. `target/` and
`.direnv/` are gitignored; commit `flake.nix`, `flake.lock` and `.envrc`. Run
`nix flake update`, rebuild and test before committing a new lockfile.

## Shared compilation cache

When `mbx` is on `PATH`, `just` compilation/test/run recipes and the runtime
validation script use it automatically; otherwise they use Cargo. Toolchain pins,
features, profiles, targets and warning flags are unchanged. Non-compiling
commands (`cargo fmt`, metadata and dependency policy checks) remain Cargo.
Explicit one-off builds can use `mbx test`, `mbx check`, `mbx clippy` or `mbx run`
with the same arguments as Cargo. No global Cargo shim is required.

To bring an existing checkout under managed storage without deleting outputs:

```sh
mbx adopt --dry-run .
mbx adopt .
mbx gc --dry-run
```

Adoption moves build storage and leaves a `target` symlink. The Nix shell's
explicit `target/nix` subdirectory remains unchanged; each checkout/worktree
retains its own target. Optional integration workspaces have independent targets
and can be adopted explicitly. Do not share one `CARGO_TARGET_DIR` across them.
Keep the shared default mbx action cache and existing compiler-wrapper settings.
Native-link caching stays disabled; no remote cache is required.

Inspect with `mbx doctor`, `mbx explain --last` and `mbx cache stats`. Cache hits
are measured compiler reuse, not a Cargo no-op. `mbx gc --dry-run` previews global
collection, so review its scope before collecting unrelated workspaces. Old
checkout-specific diagnostic targets/examples/incremental outputs can be removed
when not in use; there is no need to clean useful dependencies before adoption.

## `just` targets

`just` alone lists every target. Compile commands below show the Cargo-equivalent
arguments; the recipes select `mbx` automatically when available.

| Target | Action |
|---|---|
| `just fmt` | `cargo fmt --all` |
| `just check` | `cargo check --workspace --all-targets --all-features --locked` |
| `just lint` | Clippy over the workspace with `-D warnings` |
| `just test` | `cargo test --workspace --all-features --locked` |
| `just deny` | `cargo deny check advisories bans licenses sources` |
| `just hooks` | Run every `prek` hook over all files |
| `just module-deps` | Crate boundaries via `scripts/check-module-dependencies.py` |
| `just mpl` | MPL-2.0 notice via `scripts/check-mpl-compliance.py` |
| `just field-check` | Verify the tracked field package inventory and LFS pointers |
| `just verify` | The full pre-PR gate; see below |
| `just run <args>` / `just server <args>` | Interactive app / headless server |
| `just world-test` | Renderer-independent world tests |
| `just runtime-test` | Native 2027 rules and admission without optional integrations |
| `just gameplay-test` / `just gameplay-demo` | Gameplay tests / headless `match` example |
| `just control-test` | Native 2027 control, physical RFID and console automation tests, without rendering |
| `just network-test` / `just network-trial <scenario>` | Harness tests with real UDP / one scenario against built binaries |
| `just bench-build` / `just bench-render <config>` | Build the benchmark / run a sweep without recompiling |

The `prek` hook set in `prek.toml` formats the Rust workspace and checks
whitespace, end-of-file, merge conflicts, YAML/JSON/TOML validity, line endings,
large files, `typos`, `actionlint` and commit messages. It excludes `target/`
and the LFS `field-2027/` package. Clippy remains in `just verify` and CI's full workspace gate.

## `just verify`

`just verify` runs the pre-PR gate in this order:

1. `prek run --all-files --show-diff-on-failure`
2. `cargo fmt --all -- --check`
3. `cargo check --workspace --all-targets --all-features --locked`
4. `cargo clippy --workspace --all-targets --all-features --locked -- -D warnings`
5. `cargo test --workspace --all-features --locked`
6. `python3 scripts/check-runtime.py`, current runtime admission and native rules
7. `python3 scripts/check-module-dependencies.py`
8. `python3 scripts/check-mpl-compliance.py`
9. `python3 scripts/stage-release-field.py --verify-only --allow-lfs-pointers`, the field inventory CI checks (also `just field-check`)
10. `python3 -m unittest discover -s scripts/tests`, every Python regression test
11. `python3 -m doctest scripts/check-mpl-compliance.py`
12. `cargo deny check advisories bans licenses sources`

Steps 6-12 are the same commands CI runs, so a green `just verify` should mean a
green CI job. If the two lists ever disagree, fix the target rather than adding a
second command line.

Run it before opening a pull request, after compilation has finished.

## Tests

Tests live beside the code. `just test` runs the whole workspace; these commands
narrow to one crate.

- `rm-simulator-physics`: `cargo test -p rm-simulator-physics --locked` for
  dynamics, geometry and raw contacts.
- `rm-simulator-world`: `just world-test` = `cargo test -p rm-simulator-world
  --locked`, building a `Field` from a `FieldConfig` and stepping it.
- `rm-simulator-gameplay`: `just gameplay-test`, with no physics, CAD, server or
  renderer; `just gameplay-demo` runs a complete scenario.
- `rm-simulator-render`: a headless `App` with the sync plugin, inspecting components.
- `rm-simulator-server`: the protocol, the UDP client flows and the HTTP routes.
- `rm-simulator-app`: argument parsing, frame conversions, flashes, the HUD line
  and an end-to-end trace (`net_harness.rs`) over `scripted_link.rs`.

Python suites live in `scripts/tests/`; `just network-test` runs the network
harness tests verbosely, including real UDP and process cleanup.

## Commits and pull requests

Use `type(scope): summary` for commit subjects and PR titles, with an optional
short body. Keep the complete subject within 72 characters, imperative and
without a trailing period; add `!` before the colon for a breaking change. Types
are `feat`, `fix`, `docs`, `style`, `refactor`, `perf`, `test`, `build`, `ci`,
`chore` and `revert`. Squash merge ordinary PRs; rebase when the retained commits
pass independently. Main stays linear; check a branch with:

```sh
python3 scripts/check-commit-message.py --range origin/main..HEAD
```

`prek` installs the same check as a hook:

```sh
cargo install prek --version 0.4.14 --locked
prek install --hook-type pre-commit --hook-type commit-msg
```

## Headless inspection

`--screenshot PATH` captures the window or headless camera to a PNG once the
scene has loaded and settled, then exits; it checks visuals without a display
session. `--start-paused` opens with the world clock paused, and `F7` steps the
world one manual step (three 128 Hz ticks, 23.4375 ms) while paused in a local
world or as referee; `F6` pauses and resumes. The app
automation console (`--console`) drives the same controls; see
[console commands](console.md) and [app options](app-options.md).

## Changelog

Record every user-visible change under `Unreleased` in
[CHANGELOG.md](../CHANGELOG.md), and keep the README option and control tables
current with the code. Keep dated measurements tied to their recorded revision
and asset hashes.

## Dependency updates

Use `cargo update --dry-run` to inspect compatible lockfile updates, then run
`just verify` after applying them. Keep Bevy and its UI/render integrations on
compatible release lines; a Bevy migration is separate from a lockfile refresh.
The current compatible set is Bevy 0.20, Flair 0.9 and wgpu 30. The benchmark
uses the same wgpu major as Bevy; a second wgpu major cannot share its GPU handles.
Custom UI shaders use WESL imports and the `.wesl` extension.

The workspace expands Bevy 0.20's `3d`, `3d_bevy_render` and `3d_api` feature
bundles to select `zstd_c` for KTX2 textures. This reuses the native Zstd backend
already used for network compression and preserves the other rendering features.
Those bundles force `zstd_rust` and vulnerable `ruzstd` 0.8
([RUSTSEC-2026-0344](https://rustsec.org/advisories/RUSTSEC-2026-0344.html));
adding `zstd_c` alongside them does not remove the Rust backend. When changing
Bevy features, inspect `cargo tree --workspace --all-features -i ruzstd` and the
resolved feature graph, and smoke-test the renderer's compressed lookup textures.
