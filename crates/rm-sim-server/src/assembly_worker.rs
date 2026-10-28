// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Domain messages for isolated assembly planning; the host retains installation authority.
use rm_sim_control::assembly::{Rig, RigState, Signal};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    path::PathBuf,
    sync::{Arc, OnceLock, mpsc},
};

/// Verified source recipe used to rebuild collision acceleration in a worker.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SourceRecipe {
    /// Package root in the caller's source provider.
    pub package: PathBuf,
    /// Matching source-bound area root.
    pub area: PathBuf,
    /// Initial target used to construct the mechanical model.
    pub target: rm_sim_world::gameplay::rmuc_2027::assembly_pose::TargetPose,
    manifest_sha256: String,
    area_sha256: String,
}
impl SourceRecipe {
    /// Snapshot source identity for later worker reconstruction.
    pub fn new(
        package: PathBuf,
        area: PathBuf,
        target: rm_sim_world::gameplay::rmuc_2027::assembly_pose::TargetPose,
    ) -> Result<Self, String> {
        let digest = |path| {
            rm_sim_assets::package_io::read(path)
                .map(|bytes| format!("{:x}", Sha256::digest(bytes)))
                .map_err(|e| e.to_string())
        };
        Ok(Self {
            manifest_sha256: digest(package.join("manifest.json"))?,
            area_sha256: digest(area.join("area.json"))?,
            package,
            area,
            target,
        })
    }
    /// Copy only verified recipe inputs for transfer to an isolated file provider.
    pub fn source_files(&self) -> Result<Vec<(String, Vec<u8>)>, String> {
        let mut files = Vec::new();
        let mut total = 0usize;
        for path in [
            "manifest.json",
            "assembly.json",
            "collision.json",
            "engineer.glb",
            "core-base.glb",
            "core-arm.glb",
            "core-head.glb",
            "unit.glb",
        ]
        .into_iter()
        .map(|f| self.package.join(f))
        .chain(
            ["area.json", "area.glb", "terrain.json"]
                .into_iter()
                .map(|f| self.area.join(f)),
        ) {
            let bytes = rm_sim_assets::package_io::read(&path).map_err(|e| e.to_string())?;
            total = total
                .checked_add(bytes.len())
                .ok_or("assembly worker input overflow")?;
            if total > 256 * 1024 * 1024 {
                return Err("assembly worker source exceeds 256 MiB".into());
            }
            files.push((
                path.to_str()
                    .ok_or("non-UTF8 assembly source path")?
                    .to_owned(),
                bytes,
            ));
        }
        Ok(files)
    }
    fn rig(&self) -> Result<Rig, String> {
        for (path, expected) in [
            (self.package.join("manifest.json"), &self.manifest_sha256),
            (self.area.join("area.json"), &self.area_sha256),
        ] {
            let bytes = rm_sim_assets::package_io::read(path).map_err(|e| e.to_string())?;
            if format!("{:x}", Sha256::digest(bytes)) != *expected {
                return Err("assembly worker source identity changed".into());
            }
        }
        let mut rig =
            Rig::load_in_area(&self.package, self.target, &self.area).map_err(|e| e.to_string())?;
        rig.use_live_collisions();
        rig.select_live_core_branch().map_err(str::to_string)?;
        Ok(rig)
    }
}
/// One generation-owned planning request, without match or actuator authority.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Request {
    /// Checked immutable source reconstruction recipe.
    pub recipe: SourceRecipe,
    /// Mutable joints, grasp and bounded hints of the actual requested rig.
    pub state: RigState,
    /// Assembly difficulty, 1 through 3.
    pub difficulty: u8,
    /// Revealed Q twist, radians; absent for the initial segment.
    pub twist_rad: Option<f64>,
}
/// Worker result; host generation, parking and actual-start checks still apply.
#[derive(Serialize, Deserialize)]
pub struct Response {
    /// Validated start state, retaining the held Unit's real grasp.
    pub start: RigState,
    /// Validated endpoint and reusable bounded planner hints.
    pub end: RigState,
    /// Bounded 128 Hz actuator samples.
    pub frames: Vec<rm_sim_world::assembly::Frame>,
    /// Measured planner checks completed by this request.
    pub checks: u32,
}
/// Solve with the native algorithm after rebuilding verified source geometry.
/// This is intended for a worker, never the browser UI event loop.
pub fn solve(request: Request) -> Result<Response, String> {
    if !(1..=3).contains(&request.difficulty) || request.twist_rad.is_some_and(|v| !v.is_finite()) {
        return Err("invalid assembly worker request".into());
    }
    let mut rig = request.recipe.rig()?;
    rig.restore_transfer_state(request.state)?;
    let signal = Arc::new(Signal::default());
    let solved = crate::assembly::solve_segment(
        rig,
        request.difficulty,
        request.twist_rad,
        Arc::clone(&signal),
    )?;
    Ok(Response {
        start: solved.start.transfer_state().map_err(|e| e.to_string())?,
        end: solved.end.transfer_state().map_err(|e| e.to_string())?,
        frames: solved.frames,
        checks: signal.checks.load(std::sync::atomic::Ordering::Relaxed) as u32,
    })
}
/// Owned worker lifetime. Dropping it must cancel the isolated task.
pub trait Task: Send {}
/// Platform dispatcher, receiving one request and a bounded result channel.
pub type Dispatcher =
    fn(Request, mpsc::SyncSender<Result<Response, String>>) -> Result<Box<dyn Task>, String>;
static DISPATCHER: OnceLock<Dispatcher> = OnceLock::new();
/// Install the browser's isolated-worker dispatcher before constructing a host.
pub fn install(dispatcher: Dispatcher) -> Result<(), &'static str> {
    DISPATCHER
        .set(dispatcher)
        .map_err(|_| "assembly worker dispatcher already installed")
}
#[cfg(target_arch = "wasm32")]
pub(crate) fn dispatch(
    request: Request,
    sender: mpsc::SyncSender<Result<Response, String>>,
) -> Result<Box<dyn Task>, String> {
    DISPATCHER
        .get()
        .ok_or("isolated assembly worker dispatcher unavailable")?(request, sender)
}

#[cfg(all(test, feature = "native-host"))]
mod tests {
    use super::*;
    #[test]
    #[ignore = "requires checked assembly CAD and RM_ASSEMBLY_WORKER_FIXTURE_OUT"]
    fn source_worker_request_exports_browser_fixture_and_native_result() {
        let source = PathBuf::from(
            std::env::var_os("RM_ASSEMBLY_PACKAGE").expect("set RM_ASSEMBLY_PACKAGE"),
        );
        let area =
            PathBuf::from(std::env::var_os("RM_ASSEMBLY_AREA").expect("set RM_ASSEMBLY_AREA"));
        let output = PathBuf::from(
            std::env::var_os("RM_ASSEMBLY_WORKER_FIXTURE_OUT")
                .expect("set RM_ASSEMBLY_WORKER_FIXTURE_OUT"),
        );
        assert!(!output.exists(), "fixture output must be new");
        let target = rm_sim_world::gameplay::rmuc_2027::assembly_pose::TargetPose {
            position_m: [-0.05, 0.2, 0.5],
            theta_rad: 0.,
            phi_rad: std::f64::consts::FRAC_PI_2,
            alpha_rad: 0.,
        };
        let recipe = SourceRecipe::new(source.clone(), area.clone(), target).unwrap();
        let mut rig = recipe.rig().unwrap();
        if let Some(hints) = std::env::var_os("RM_ASSEMBLY_HINTS") {
            rig.load_ik_hints(std::path::Path::new(&hints)).unwrap();
        }
        rig.set_live_parking(rm_sim_world::Pose::yawed(
            [1.278000049957, -0.547914948432, 0.383150523114],
            195f64.to_radians(),
        ))
        .unwrap();
        let arm = rm_sim_control::assembly::ArmPose::neutral(&rig.runtime_model());
        rig.set_live_entry(&arm);
        let request = Request {
            recipe,
            state: rig.transfer_state().unwrap(),
            difficulty: 1,
            twist_rad: None,
        };
        let native = solve(request.clone());
        std::fs::create_dir_all(&output).unwrap();
        std::fs::write(
            output.join("expected.json"),
            serde_json::to_vec(&native).unwrap(),
        )
        .unwrap();
        let mut browser = request.clone();
        browser.recipe.package = "assembly".into();
        browser.recipe.area = "area".into();
        std::fs::write(
            output.join("request.json"),
            serde_json::to_vec(&browser).unwrap(),
        )
        .unwrap();
        let mut paths = Vec::new();
        for (path, bytes) in request.recipe.source_files().unwrap() {
            let path = PathBuf::from(path);
            let virtual_path = if let Ok(relative) = path.strip_prefix(&source) {
                PathBuf::from("assembly").join(relative)
            } else {
                PathBuf::from("area").join(path.strip_prefix(&area).unwrap())
            };
            let destination = output.join("files").join(&virtual_path);
            std::fs::create_dir_all(destination.parent().unwrap()).unwrap();
            std::fs::write(destination, bytes).unwrap();
            paths.push(virtual_path.to_str().unwrap().to_owned());
        }
        std::fs::write(
            output.join("files.json"),
            serde_json::to_vec(&paths).unwrap(),
        )
        .unwrap();
        match native {
            Ok(response) => eprintln!(
                "native worker fixture solved {} frames with {} checks",
                response.frames.len(),
                response.checks
            ),
            Err(error) => eprintln!("native worker fixture rejected: {error}"),
        }
    }
}
