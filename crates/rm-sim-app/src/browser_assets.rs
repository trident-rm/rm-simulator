// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! HTTP packages for the ordinary native manifest and collision loaders.
use bevy::asset::io::memory::MemoryAssetReader;
use rm_sim_server::cad_assets::CadAssets;
use std::{
    path::Path,
    sync::{Arc, OnceLock},
};
use wasm_bindgen::JsCast;
use wasm_bindgen_futures::JsFuture;

pub(crate) fn reader() -> MemoryAssetReader {
    static READER: OnceLock<MemoryAssetReader> = OnceLock::new();
    READER.get_or_init(MemoryAssetReader::default).clone()
}
async fn fetch(path: &str, limit: usize) -> anyhow::Result<Vec<u8>> {
    let window = web_sys::window().ok_or_else(|| anyhow::anyhow!("missing browser window"))?;
    let response: web_sys::Response = JsFuture::from(window.fetch_with_str(path))
        .await
        .map_err(|e| anyhow::anyhow!("fetch {path}: {e:?}"))?
        .dyn_into()
        .map_err(|_| anyhow::anyhow!("invalid response: {path}"))?;
    anyhow::ensure!(response.ok(), "{path}: HTTP {}", response.status());
    if let Some(length) = response
        .headers()
        .get("Content-Length")
        .ok()
        .flatten()
        .and_then(|v| v.parse::<usize>().ok())
    {
        anyhow::ensure!(length <= limit, "{path}: response exceeds byte limit");
    }
    let bytes = JsFuture::from(
        response
            .array_buffer()
            .map_err(|e| anyhow::anyhow!("{path}: {e:?}"))?,
    )
    .await
    .map_err(|e| anyhow::anyhow!("reading {path}: {e:?}"))?;
    let array = js_sys::Uint8Array::new(&bytes);
    anyhow::ensure!(
        array.length() as usize <= limit,
        "{path}: response exceeds byte limit"
    );
    Ok(array.to_vec())
}
/// Fetch every approved package file, then run the unchanged admission loader.
pub(crate) async fn load(args: &crate::args::Args) -> anyhow::Result<Arc<CadAssets>> {
    use sha2::{Digest, Sha256};
    let root = args.host.cad_assets_path();
    let root = root
        .to_str()
        .ok_or_else(|| anyhow::anyhow!("invalid package path"))?;
    anyhow::ensure!(
        !root.starts_with('/')
            && !root.contains("..")
            && !root.contains(':')
            && !root.contains('\\'),
        "browser package must use a relative HTTP path"
    );
    let index: std::collections::BTreeMap<String, String> =
        serde_json::from_slice(&fetch(&format!("{root}/package-files.json"), 1024 * 1024).await?)?;
    anyhow::ensure!(index.len() <= 256, "package file count exceeds limit");
    let memory = reader();
    let mut total = 0;
    for (file, digest) in index {
        anyhow::ensure!(
            !file.starts_with('/')
                && !file.contains("..")
                && !file.contains([':', '\\', '?', '#', '%']),
            "invalid package filename"
        );
        let path = format!("{root}/{file}");
        let bytes = fetch(&path, (64 * 1024 * 1024).min(512 * 1024 * 1024 - total)).await?;
        total += bytes.len();
        anyhow::ensure!(
            format!("{:x}", Sha256::digest(&bytes)) == digest,
            "{path}: package checksum mismatch"
        );
        rm_sim_assets::package_io::install(&path, bytes.clone())?;
        memory.root.insert_asset(Path::new(&path), bytes);
    }
    let cad = rm_sim_server::cad_assets::load(Path::new(root))?;
    Ok(Arc::new(cad))
}
