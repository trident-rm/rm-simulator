<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# rm-sim-presentation

RoboMaster snapshot-to-appearance adaptation. Caller-owned time, borrowed world
records and detected hits become renderer scene/target records; retained-output
APIs let callers reuse their buffers. This crate owns no host, network transport,
session, input, window or simulation clock.

The app still chooses predicted/interpolated robot states, live visual projectiles
and presentation time. The renderer applies resolved values and never reads the
world. Generic viewers can bypass this game adapter entirely and use the
renderer with its default `robomaster` feature disabled.

See [presentation and rendering boundaries](../../docs/presentation-render-split.md)
for ownership, copying costs and verification, and crate API docs for arguments
and units. Run `mbx test -p rm-sim-presentation --locked` for pure adaptation
fixtures without a window or GPU.
