# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
set shell := ["sh", "-cu"]
set quiet
set minimum-version := "1.55.0"

# Keep CI usable without mbx; compilation prefers the shared cache when installed.
rust := `if command -v mbx >/dev/null 2>&1; then printf mbx; else printf cargo; fi`

[default]
help:
    just --list

fmt:
    cargo fmt --all

check:
    {{rust}} check --workspace --all-targets --all-features --locked

lint:
    {{rust}} clippy --workspace --all-targets --all-features --locked -- -D warnings

test:
    {{rust}} test --workspace --all-features --locked

deny:
    cargo deny check advisories bans licenses sources

hooks:
    prek run --all-files --show-diff-on-failure

module-deps:
    python3 scripts/check-module-dependencies.py

# Current runtime admission and native rules without optional app integrations.
runtime-test:
    python3 scripts/check-runtime.py

mpl:
    python3 scripts/check-mpl-compliance.py

verify:
    prek run --all-files --show-diff-on-failure
    cargo fmt --all -- --check
    {{rust}} check --workspace --all-targets --all-features --locked
    {{rust}} clippy --workspace --all-targets --all-features --locked -- -D warnings
    {{rust}} test --workspace --all-features --locked
    python3 scripts/check-runtime.py
    python3 scripts/check-module-dependencies.py
    python3 scripts/check-mpl-compliance.py
    python3 scripts/stage-release-field.py --verify-only --allow-lfs-pointers
    python3 -m unittest discover -s scripts/tests
    python3 -m doctest scripts/check-mpl-compliance.py
    cargo deny check advisories bans licenses sources

# The field package inventory CI checks, without staging a release.
field-check:
    python3 scripts/stage-release-field.py --verify-only --allow-lfs-pointers

# Standalone network harness tests, including real UDP and process cleanup.
network-test:
    python3 -m unittest discover -s scripts/tests -p 'test_network*.py' -v

# Run an isolated scenario using explicitly built server/app binaries.
network-trial SCENARIO *ARGS:
    python3 scripts/network-harness.py {{quote(SCENARIO)}} {{ARGS}}

# Gameplay tests without physics, CAD, a server or a renderer.
gameplay-test:
    {{rust}} test -p rm-simulator-gameplay --locked

# Native 2027 control rules, physical RFID and console scenario tooling, without a GPU.
control-test:
    {{rust}} test -p rm-simulator-gameplay --locked --lib rmuc_2027::
    {{rust}} test -p rm-simulator-gameplay --locked --test control_2027
    {{rust}} test -p rm-simulator-gameplay --locked --test field_buffs_2027
    {{rust}} test -p rm-simulator-gameplay --locked --test terrain_2027 --test rune_2027 --test assembly_2027 --test adaptive_2027
    {{rust}} test -p rm-simulator-world --locked --lib rmuc_2027::tests
    python3 -m unittest discover -s scripts/tests -p test_console.py -v
    python3 -m unittest discover -s scripts/tests -p test_control_field_scenarios.py -v
    python3 -m unittest discover -s scripts/tests -p test_terrain_rune_scenarios.py -v

# A native RMUC 2027 capture/scoring/result scenario.
gameplay-demo:
    {{rust}} run -p rm-simulator-gameplay --example match --locked

# Renderer-independent world tests only.
world-test:
    {{rust}} test -p rm-simulator-world --locked

# Interactive first-person scene.
run *ARGS:
    {{rust}} run -p rm-simulator-app --locked -- {{ARGS}}

# The app with the opt-in custom-client referee link (see docs/referee-link.md).
run-referee *ARGS:
    {{rust}} run -p rm-simulator-app --locked --features referee-link -- {{ARGS}}

# Headless simulation server (GNS UDP plus the HTTP referee panel).
server *ARGS:
    {{rust}} run -p rm-simulator-server --locked -- {{ARGS}}

# Build once, then run rendering sweeps without compiling during capture.
bench-build:
    {{rust}} build -p rm-simulator-bench --locked

bench-render CONFIG *ARGS:
    python3 scripts/benchmark-render.py {{quote(CONFIG)}} {{ARGS}}
