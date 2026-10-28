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

fn readers()
-> &'static std::sync::Mutex<std::collections::BTreeMap<std::path::PathBuf, MemoryAssetReader>> {
    static READERS: OnceLock<
        std::sync::Mutex<std::collections::BTreeMap<std::path::PathBuf, MemoryAssetReader>>,
    > = OnceLock::new();
    READERS.get_or_init(Default::default)
}
/// Memory source rooted at an optional checked robot or assembly package.
pub(crate) fn reader_at(root: &Path) -> MemoryAssetReader {
    readers()
        .lock()
        .expect("browser package reader lock poisoned")
        .entry(root.to_path_buf())
        .or_default()
        .clone()
}
fn relative_root(root: &Path) -> anyhow::Result<&str> {
    let root = root
        .to_str()
        .ok_or_else(|| anyhow::anyhow!("invalid package path"))?;
    anyhow::ensure!(
        !root.is_empty()
            && !root.starts_with('/')
            && !root.contains("..")
            && !root.contains([':', '\\', '?', '#', '%']),
        "browser package must use a relative HTTP path"
    );
    Ok(root)
}
fn local_filename(file: &str) -> bool {
    !file.is_empty()
        && !file.contains(['/', '\\', ':', '?', '#', '%'])
        && file != "."
        && file != ".."
}
fn verify_embedded_glb(bytes: &[u8]) -> anyhow::Result<()> {
    anyhow::ensure!(
        bytes.len() >= 20 && &bytes[..4] == b"glTF",
        "expected a self-contained GLB"
    );
    let word = |index| {
        u32::from_le_bytes(
            bytes[index..index + 4]
                .try_into()
                .expect("checked GLB header"),
        )
    };
    anyhow::ensure!(
        word(4) == 2 && word(8) as usize == bytes.len() && &bytes[16..20] == b"JSON",
        "invalid GLB header"
    );
    let end = 20usize
        .checked_add(word(12) as usize)
        .ok_or_else(|| anyhow::anyhow!("GLB JSON overflow"))?;
    let json: serde_json::Value = serde_json::from_slice(
        bytes
            .get(20..end)
            .ok_or_else(|| anyhow::anyhow!("truncated GLB JSON"))?,
    )?;
    for kind in ["buffers", "images"] {
        anyhow::ensure!(
            json[kind]
                .as_array()
                .is_none_or(|values| values.iter().all(|value| value.get("uri").is_none())),
            "external GLB {kind} are not verified"
        );
    }
    Ok(())
}
async fn install_optional(root: &Path, assembly: bool) -> anyhow::Result<()> {
    use sha2::{Digest, Sha256};
    let url = relative_root(root)?;
    let manifest_bytes = fetch(&format!("{url}/manifest.json"), 1024 * 1024).await?;
    let manifest: serde_json::Value = serde_json::from_slice(&manifest_bytes)?;
    let files = manifest["files"]
        .as_object()
        .ok_or_else(|| anyhow::anyhow!("missing package file references"))?;
    anyhow::ensure!(
        files.len() <= 32,
        "optional package file count exceeds limit"
    );
    let mut total = manifest_bytes.len();
    let memory = reader_at(root);
    rm_sim_assets::package_io::install(root.join("manifest.json"), manifest_bytes.clone())?;
    memory
        .root
        .insert_asset(Path::new("manifest.json"), manifest_bytes);
    for (key, file) in files {
        // The descriptor consumes the neutral pose and optional physical configuration.
        // Preview poses remain unconsumed, just as they do in the native loader.
        if !assembly && !matches!(key.as_str(), "neutral" | "chassis") {
            continue;
        }
        let name = file["file"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("missing package filename"))?;
        let expected = file["sha256"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("missing package checksum"))?;
        anyhow::ensure!(local_filename(name), "invalid package filename: {name}");
        let bytes = fetch(
            &format!("{url}/{name}"),
            (64 * 1024 * 1024).min(256 * 1024 * 1024 - total),
        )
        .await?;
        total += bytes.len();
        anyhow::ensure!(
            format!("{:x}", Sha256::digest(&bytes)) == expected,
            "{url}/{name}: package checksum mismatch"
        );
        if name.ends_with(".glb") {
            verify_embedded_glb(&bytes)?;
        }
        rm_sim_assets::package_io::install(root.join(name), bytes.clone())?;
        memory.root.insert_asset(Path::new(name), bytes);
    }
    Ok(())
}
/// Fetch optional CAD packages before the ordinary app registers their asset sources.
/// Source identity, physical models, armor and assembly guards use native loaders.
pub(crate) async fn prepare_optional(args: &mut crate::args::Args) -> anyhow::Result<()> {
    use sha2::Digest;
    for root in [args.robot_assets.as_ref(), args.hero_assets.as_ref()]
        .into_iter()
        .flatten()
    {
        install_optional(root, false).await?;
    }
    args.resolve_loaded_robot_assets()?;
    if let Some(root) = &args.host.assembly_assets {
        install_optional(root, true).await?;
        let source = rm_sim_assets::semantics::assembly::SourcePackage::load(root)
            .map_err(|error| anyhow::anyhow!("{error}"))?;
        source.model().map_err(|error| anyhow::anyhow!("{error}"))?;
        source
            .collision_scene()
            .map_err(|error| anyhow::anyhow!("{error}"))?;
        let area = args
            .host
            .assembly_area
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("assembly package requires an area frame"))?;
        let url = relative_root(area)?;
        for (file, limit) in [
            ("area.json", 1024 * 1024),
            ("area.glb", 64 * 1024 * 1024),
            ("terrain.json", 64 * 1024 * 1024),
        ] {
            let bytes = fetch(&format!("{url}/{file}"), limit).await?;
            if file.ends_with(".glb") {
                verify_embedded_glb(&bytes)?;
            }
            rm_sim_assets::package_io::install(area.join(file), bytes)?;
        }
        let evidence: serde_json::Value =
            serde_json::from_slice(&rm_sim_assets::package_io::read(area.join("area.json"))?)?;
        for (file, key) in [
            ("area.glb", "visual_sha256"),
            ("terrain.json", "collision_sha256"),
        ] {
            let digest = format!(
                "{:x}",
                sha2::Sha256::digest(rm_sim_assets::package_io::read(area.join(file))?)
            );
            anyhow::ensure!(
                evidence[key].as_str() == Some(&digest),
                "assembly area checksum mismatch: {file}"
            );
        }
        source
            .area_frame(area)
            .map_err(|error| anyhow::anyhow!("{error}"))?;
    }
    if let Some(hints) = &args.host.assembly_hints {
        let url = relative_root(hints)?;
        let bytes = fetch(url, 16 * 1024 * 1024).await?;
        rm_sim_assets::package_io::install(hints, bytes)?;
    }
    Ok(())
}
