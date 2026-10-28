// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Package file access: native files or caller-supplied browser bytes.
use std::{
    io,
    path::{Path, PathBuf},
};

/// Read a package payload. Browser callers must install the bytes first.
pub fn read(path: impl AsRef<Path>) -> io::Result<Vec<u8>> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        std::fs::read(path)
    }
    #[cfg(target_arch = "wasm32")]
    {
        let path = canonicalize(path)?;
        FILES
            .with(|files| files.borrow().get(&path).cloned())
            .ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::NotFound,
                    format!("package file not fetched: {}", path.display()),
                )
            })
    }
}
/// Resolve package containment using native canonical paths or browser logical paths.
pub(crate) fn canonicalize(path: impl AsRef<Path>) -> io::Result<PathBuf> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        path.as_ref().canonicalize()
    }
    #[cfg(target_arch = "wasm32")]
    {
        use std::path::Component;
        let mut result = PathBuf::new();
        for component in path.as_ref().components() {
            match component {
                Component::Normal(value) => result.push(value),
                Component::CurDir => {}
                _ => {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidInput,
                        "browser package path must be relative and contained",
                    ));
                }
            }
        }
        Ok(result)
    }
}
#[cfg(target_arch = "wasm32")]
thread_local! { static FILES: std::cell::RefCell<std::collections::BTreeMap<PathBuf, Vec<u8>>> = const { std::cell::RefCell::new(std::collections::BTreeMap::new()) }; }
/// Install a fetched browser payload for the ordinary checksum-verifying loader.
/// Installation does not validate a file; package loading still checks its manifest.
#[cfg(target_arch = "wasm32")]
pub fn install(path: impl AsRef<Path>, bytes: Vec<u8>) -> io::Result<()> {
    let path = canonicalize(path)?;
    FILES.with(|files| files.borrow_mut().insert(path, bytes));
    Ok(())
}
