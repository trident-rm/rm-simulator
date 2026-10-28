<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Development

The developer workflow reference: environment setup, `just` targets, tests,
commit conventions and headless inspection. See [Contributing](../CONTRIBUTING.md)
and [CI and releases](releases.md); the [project README](../README.md) covers play.

## Environment

`rust-toolchain.toml` pins stable Rust with the `rustfmt`, `clippy` and `rust-analyzer`
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
| `just test-crate <package> [filter]` | One crate, all features, with an optional test-name substring |
| `just affected-test [base] [--list]` | Changed crates and their workspace consumers; defaults to `HEAD` |
| `just test-nextest` | Optional nextest run, Cargo doctests and isolated generic feature tests |
| `just python-test [pattern] [--name substring]` | Python tooling tests with per-test timing |
| `just python-fast-test` | Python tests except the real-time network subprocess trial class |
| `just deny` | `cargo deny check advisories bans licenses sources` |
| `just hooks` | Run every `prek` hook over all files |
| `just module-deps` | Crate boundaries via `scripts/check-module-dependencies.py` |
| `just mpl` | MPL-2.0 notice via `scripts/check-mpl-compliance.py` |
| `just field-check` | Verify the tracked field package inventory and LFS pointers |
| `just verify` | The full pre-PR gate; see below |
| `just run <args>` / `just server <args>` | Interactive app / headless server |
| `just world-test` | Renderer-independent world tests |
| `just runtime-test [profile]` | Native 2027 rules and admission without optional integrations; default `test` |
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
4. `cargo clippy --workspace --all-targets --all-features --locked -- -D warnings`,
   plus isolated `sim-physics` and `sim-pendulum-example --no-default-features` Clippy
5. `cargo test --workspace --all-features --locked`, including `sim-render`, plus
   isolated generic physics/example and compatibility renderer tests with
   `--no-default-features`
6. `python3 scripts/check-runtime.py`, current runtime admission and native rules
7. `python3 scripts/check-module-dependencies.py`
8. `python3 scripts/check-mpl-compliance.py`
9. `python3 scripts/stage-release-field.py --verify-only --allow-lfs-pointers`, the field inventory CI checks (also `just field-check`)
10. `python3 scripts/test-python.py`, every Python regression test with slow-test reporting
11. `python3 -m doctest scripts/check-mpl-compliance.py`
12. `cargo deny check advisories bans licenses sources`

Steps 6-12 are the same commands CI runs, so a green `just verify` should mean a
green CI job. If the two lists ever disagree, fix the target rather than adding a
second command line.

Run it before opening a pull request, after compilation has finished.

## Crate naming

Generic packages and directories use `sim-*`; Rust imports use `sim_*`.
The solver-free `sim-engine`, concrete `sim-physics` and passive `sim-render`
are generic crates. Packages containing RoboMaster code keep `rm-sim-*` /
`rm_sim_*`, including the physical/render extensions, assets and controllers.
The main `rm-simulator` executable and GitHub repository keep their names;
suffixed executables use `rm-sim-server`, `rm-sim-bench` and `rm-sim-ros2`.
The independent `crates/rm-sim` registry placeholder remains separate from the
unpublished runtime workspace.

Dated performance reports preserve the package names used for their recorded
revisions. Substitute the new prefix when running their commands on current
source.

## Tests

Tests live beside the code. `just test` runs the whole workspace; these commands
narrow to one crate.

- `rm-sim-physics`: `cargo test -p rm-sim-physics --locked` for
  dynamics, geometry and raw contacts.
- `rm-sim-world`: `just world-test` = `cargo test -p rm-sim-world
  --locked`, building a `Field` from a `FieldConfig` and stepping it.
- `rm-sim-gameplay`: `just gameplay-test`, with no physics, CAD, server or
  renderer; `just gameplay-demo` runs a complete scenario.
- `rm-sim-render`: a headless `App` with the sync plugin, inspecting components.
- `rm-sim-server`: the protocol, the UDP client flows and the HTTP routes.
- `rm-sim-app`: argument parsing, frame conversions, flashes, the HUD line
  and an end-to-end trace (`net_harness.rs`) over `scripted_link.rs`.

Python suites live in `scripts/tests/`; `just network-test` runs the network
harness tests verbosely, including real UDP and process cleanup.

### Targeted iteration

Use the smallest relevant suite while editing. Keep `just verify` as the full
pre-PR gate; targeted runs do not replace feature-isolation, ignored CAD checks
or repository policy checks.

```sh
just test-crate rm-sim-world energy_unit
just test-crate rm-sim-control assembly
just affected-test HEAD --list
just affected-test origin/main
just python-test test_console.py
just python-test test_network_harness.py --name RunnerProcessTests
just python-fast-test
```

`affected-test` reads Cargo metadata and Git changes, including staged, unstaged,
deleted and untracked files. It selects changed packages and their transitive
workspace consumers, including optional, build and dev dependency edges. It
selects the whole workspace for global configuration, unknown paths and tooling
changes. Root Markdown and `docs/` changes select no Rust packages. It does not
infer individual test coverage or select Python tests. A name filter narrows
execution, but Cargo still compiles the selected test targets.

To select several crates explicitly or choose a build profile:

```sh
python3 scripts/test-rust.py -p rm-sim-world -p rm-sim-server --filter energy_unit
python3 scripts/test-rust.py --changed-since HEAD --profile release --list
```

The default runner is Cargo. If `cargo-nextest` is installed, `just test-nextest`
uses its cross-binary scheduling, then runs doctests through Cargo. For a targeted
nextest run, append `--runner nextest` to `just affected-test`, or pass it to
`scripts/test-rust.py`. A failing nextest run stops before doctests. No retries
are enabled. Both runners use the same selected packages and all features.

`check-runtime.py --profile release` keeps native CI's isolated runtime checks
on the release build profile. Local runtime checks default to `test`. The
separate feature configurations remain intentional boundary checks.

### Test cost and Python ownership

Python runs offline asset preparation and developer tools: field/CAD conversion
and binding, console scenario automation, network impairment trials, benchmark
reporting, release packaging and repository/CI checks. The optional ROS bridge
also has Python control-I/O tests. Rust owns simulation stepping, physical
dynamics, match rules and the production server/app.

On the local Mac, a warm run on 2026-10-27 listed 1,543 Rust tests: 1,280 unit
and integration tests (18 ignored), plus 263 doctests. The full workspace run
passed in 39.2 seconds. Nextest ran the same 1,262 enabled unit/integration tests
in 17.8 seconds including command overhead, without doctests. These are single
samples; compilation, doctests and the extra feature configurations must be
included when comparing full gates.
A later nextest-plus-doctest run passed in 63.9 seconds while other builds and
browser processes competed for CPU. Nextest remains opt-in; these samples do
not establish a fixed speedup for the full gate.
The six whole-chassis tests took 12.0 seconds in the workspace run. A separate
level-2 optimization trial for both physics crates took 13.3 seconds of test
execution, so the checked-in optimization settings remain unchanged.

The original 198-test Python run took about 26 seconds with 39 skipped. Per-test
timing attributed 24.3 seconds to `test_network_harness`, including 22.6 seconds
in `RunnerProcessTests`. Those tests start real fixture processes and wait for
sampling windows, rate pacing, console stalls and readiness timeouts. The
baseline comparison test runs two trials and took 5.2 seconds. These waits test
the tooling's time behavior; reducing them can weaken coverage or cause flakes.
`python-fast-test` excludes only that process-test class and prints the exclusion.
With the workflow regression tests added, that command passed 194 discovered
tests in 3.3 seconds (39 skipped).
Full local and CI validation still run it. The Python runner reports setup,
execution and teardown time for the ten slowest tests; `--durations 0` disables
the report. Skipped CAD tests still require their external tools/assets.

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

The October 2026 refresh uses Rapier 0.36.1 / Parry 0.31.1 and application
Zstd 0.14.3. Bevy 0.20 still requires Zstd 0.13.3 for its C texture decoder,
so its transitive Zstd/Zstd-safe advisories remain until that dependency moves.
Zstd-sys 2.1.2 fixes the Wasm allocator advisory. No advisory exceptions were
added for these versions.

Keep Glam on Bevy's 0.33 release line to share vector types. SHA-2 0.11 removes
hex formatting on digest outputs; migrate checksum formatting separately before
raising its requirement. Native and web feature selection is a separate change
from this dependency refresh.
