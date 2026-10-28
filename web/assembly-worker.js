// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
// The same Wasm module installs only the assembly message handler in this realm.
import init from './pkg/rm_simulator.js';
await init();
self.postMessage({type: 'ready'});
