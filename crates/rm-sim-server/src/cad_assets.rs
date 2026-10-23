// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Compatibility facade for verified CAD packages used by the server runtime.
pub use rm_sim_assets::cad_assets::{
    CENTER_CAD_Y_M, CadAsset, FLOOR_TOP_CAD_M, arena_pose, cad_point_to_flu, default_cad_assets,
};

/// Verified CAD package with the server's native 2027 runtime bindings.
pub type CadAssets = rm_sim_assets::cad_assets::CadAssets<crate::rmuc_2027::Bindings>;

/// Read and verify a package, retaining native runtime geometry validation.
pub fn load(root: &std::path::Path) -> anyhow::Result<CadAssets> {
    rm_sim_assets::cad_assets::load_with_bindings(root, |bytes| {
        let bindings: crate::rmuc_2027::Bindings = serde_json::from_slice(bytes)?;
        bindings.validate()?;
        Ok(bindings)
    })
}
