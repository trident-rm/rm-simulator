// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared immutable file snapshots; each format retains its own path/hash policy.
use sha2::{Digest, Sha256};
use std::{
    io,
    path::{Path, PathBuf},
};

/// The bytes and digest always describe the same read, including after file edits.
pub(crate) struct Snapshot {
    pub(crate) bytes: Vec<u8>,
    pub(crate) sha256: String,
}
impl Snapshot {
    pub(crate) fn new(bytes: Vec<u8>) -> Self {
        let sha256 = format!("{:x}", Sha256::digest(&bytes));
        Self { bytes, sha256 }
    }
    pub(crate) fn read(path: &Path) -> io::Result<Self> {
        crate::package_io::read(path).map(Self::new)
    }
}

/// Check canonical containment without changing a format's lexical path rules.
pub(crate) fn confined_path(root: &Path, relative: &Path) -> io::Result<PathBuf> {
    let path = crate::package_io::canonicalize(root.join(relative))?;
    if !path.starts_with(crate::package_io::canonicalize(root)?) {
        return Err(io::Error::other("asset payload escapes package root"));
    }
    Ok(path)
}
