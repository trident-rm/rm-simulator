// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Browser launch options for the ordinary application lifecycle.
use clap::Parser;
pub(crate) fn args() -> anyhow::Result<crate::args::Args> {
    let query = web_sys::window()
        .and_then(|w| w.location().search().ok())
        .and_then(|s| web_sys::UrlSearchParams::new_with_str(&s).ok());
    let mut values = vec![
        "rm-simulator".to_string(),
        "--graphics-preset".into(),
        "low".into(),
        "--no-bloom".into(),
    ];
    if let Some(query) = query {
        if let Some(server) = query.get("server").filter(|s| !s.is_empty()) {
            values.extend(["--connect".into(), server, "--staging".into()]);
        }
        if query.get("play").as_deref() == Some("1") {
            values.push("--play".into());
        }
        if query.get("prediction").as_deref() == Some("0") {
            values.push("--no-prediction".into());
        }
        let competition = match query.get("field").as_deref() {
            Some("3v3") => "rmul-2027",
            Some("rmuc") => "rmuc-2027",
            _ => "rmul-infantry-2027",
        };
        values.extend(["--competition".into(), competition.into()]);
        for (key, flag) in [
            ("team", "--team"),
            ("name", "--name"),
            ("robot", "--robot"),
            ("password", "--password"),
            ("robot_assets", "--robot-assets"),
            ("hero_assets", "--hero-assets"),
            ("assembly_assets", "--assembly-assets"),
            ("assembly_area", "--assembly-area"),
            ("assembly_hints", "--assembly-hints"),
        ] {
            if let Some(value) = query.get(key).filter(|s| !s.is_empty()) {
                values.extend([flag.into(), value]);
            }
        }
    }
    crate::args::Args::try_parse_from(values).map_err(Into::into)
}
pub(crate) fn start() {
    if crate::browser_assembly_worker::start_if_worker() {
        return;
    }
    wasm_bindgen_futures::spawn_local(async {
        let result = async {
            let mut args = args()?;
            crate::browser_assets::prepare_optional(&mut args).await?;
            let _ = crate::native::run_with_args(args);
            Ok::<_, anyhow::Error>(())
        }
        .await;
        if let Err(error) = result
            && let Some(element) = web_sys::window()
                .and_then(|w| w.document())
                .and_then(|d| d.get_element_by_id("status"))
        {
            let _ = element.remove_attribute("hidden");
            let _ = element.set_attribute("data-state", "error");
            element.set_text_content(Some(&format!("Cannot start simulator: {error:#}")));
        }
    });
}

pub(crate) fn install(app: &mut bevy::prelude::App) {
    crate::browser_assembly_worker::install();
    use bevy::prelude::*;
    let inspect = web_sys::window()
        .and_then(|w| w.location().search().ok())
        .is_some_and(|q| q.contains("inspect=1"));
    if inspect {
        app.init_resource::<FrameSamples>()
            .add_systems(PostUpdate, inspect_ui.after(bevy::ui::UiSystems::Layout));
    }
}
// Opt-in test observations come from actual application resources and laid-out UI nodes.
fn inspect_ui(world: &mut bevy::prelude::World) {
    use bevy::prelude::*;
    let Some(element) = web_sys::window()
        .and_then(|w| w.document())
        .and_then(|d| d.get_element_by_id("status"))
    else {
        return;
    };
    let state = world
        .get_resource::<State<crate::loading::Screen>>()
        .map(|s| format!("{:?}", s.get()))
        .unwrap_or_default();
    let frame_ms = world.resource::<Time<Real>>().delta_secs_f64() * 1000.;
    let frames = {
        let mut samples = world.resource_mut::<FrameSamples>();
        samples.record(&state, frame_ms)
    };
    let sample_network = frames.is_some();
    let mut values = vec![("data-screen", state)];
    if let Some(frames) = frames {
        values.push(("data-frame-timing", frames));
    }
    {
        use wasm_bindgen::JsCast;
        let memory = wasm_bindgen::memory().unchecked_into::<js_sys::WebAssembly::Memory>();
        let buffer = memory.buffer().unchecked_into::<js_sys::ArrayBuffer>();
        values.push(("data-wasm-memory-bytes", buffer.byte_length().to_string()));
    }
    values.push(("data-in-match", crate::loading::in_match(world).to_string()));
    values.push((
        "data-loading-error",
        crate::loading::failure(world).unwrap_or_default(),
    ));
    let ready = world
        .get_resource::<crate::screenshot::CaptureReadiness>()
        .is_some_and(|r| r.ready());
    values.push(("data-pipelines-ready", ready.to_string()));
    if let Some(base) = world.get_resource::<crate::title::BaseArgs>() {
        values.push((
            "data-robot-package",
            (base.0.robot_descriptor.is_some() || base.0.hero_descriptor.is_some()).to_string(),
        ));
    }
    if let Some(ui) = world.get_resource::<crate::hud::HudState>() {
        values.push(("data-input-blocked", ui.blocks_input().to_string()));
    }
    if let Some(keys) = world.get_resource::<ButtonInput<KeyCode>>() {
        values.push(("data-key-w", keys.pressed(KeyCode::KeyW).to_string()));
    }
    if let Some(session) = world.get_resource::<crate::session::Session>() {
        values.push(("data-local-aim", session.local_aim.to_string()));
        values.push(("data-paused", session.paused.to_string()));
        values.push((
            "data-phase",
            session
                .snapshot
                .rmul_2027
                .as_ref()
                .map(|g| format!("{:?}", g.phase))
                .or_else(|| {
                    session
                        .snapshot
                        .rmuc_2027
                        .as_ref()
                        .map(|s| format!("{:?}", s.game.phase))
                })
                .unwrap_or_else(|| "Practice".into()),
        ));
        values.push(("data-offline", (!session.is_remote()).to_string()));
        if let Some(work) = session.local_host_work() {
            values.push((
                "data-host-work",
                serde_json::to_string(&work).unwrap_or_default(),
            ));
        }
        if sample_network {
            values.push(("data-network", session.network_diagnostics().to_string()));
        }
        values.push((
            "data-input-frame",
            serde_json::to_string(&session.last_input_frame).unwrap_or_default(),
        ));
        values.push(("data-rejections", session.rejection_count.to_string()));
        values.push((
            "data-last-rejection",
            session.last_rejection.clone().unwrap_or_default(),
        ));
        values.push((
            "data-owner-command",
            session
                .own_chassis()
                .map(|c| serde_json::to_string(&c.command).unwrap_or_default())
                .unwrap_or_default(),
        ));
        values.push(("data-time", session.snapshot.time_ns.to_string()));
        values.push((
            "data-chassis",
            session
                .chassis_id
                .map_or_else(String::new, |id| id.to_string()),
        ));
        if let Some(own) = session.own_chassis() {
            values.push((
                "data-position",
                serde_json::to_string(&own.pose.translation_m).unwrap_or_default(),
            ));
            values.push(("data-team", format!("{:?}", own.team)));
        }
        values.push((
            "data-projectiles",
            session.snapshot.projectiles.len().to_string(),
        ));
        let previous = element
            .get_attribute("data-last-projectile")
            .and_then(|s| s.parse::<i128>().ok())
            .unwrap_or(-1);
        let newest = session
            .snapshot
            .projectiles
            .iter()
            .filter(|p| p.shooter == session.chassis_id)
            .map(|p| i128::from(p.id))
            .max()
            .unwrap_or(-1);
        values.push(("data-last-projectile", previous.max(newest).to_string()));
    }
    let mut texts = Vec::new();
    let mut query = world.query::<(
        &Text,
        &ComputedNode,
        &UiGlobalTransform,
        Option<&InheritedVisibility>,
    )>();
    for (text, node, transform, visibility) in query.iter(world) {
        if text.0.is_empty()
            || node.size().min_element() <= 0.
            || visibility.is_some_and(|v| !v.get())
        {
            continue;
        }
        let center = transform.translation * node.inverse_scale_factor;
        let size = node.size() * node.inverse_scale_factor;
        texts.push(serde_json::json!({"text":text.0,"x":center.x,"y":center.y,"width":size.x,"height":size.y}));
    }
    values.push(("data-ui", serde_json::to_string(&texts).unwrap_or_default()));
    for (key, value) in values {
        if element.get_attribute(key).as_deref() != Some(&value) {
            let _ = element.set_attribute(key, &value);
        }
    }
}

#[derive(bevy::prelude::Resource, Default)]
struct FrameSamples {
    screen: String,
    frames: u64,
    intervals_ms: std::collections::VecDeque<f64>,
}
impl FrameSamples {
    fn record(&mut self, screen: &str, interval_ms: f64) -> Option<String> {
        if self.screen != screen {
            self.screen = screen.into();
            self.frames = 0;
            self.intervals_ms.clear();
        }
        self.frames += 1;
        if interval_ms.is_finite() && interval_ms > 0. {
            self.intervals_ms.push_back(interval_ms);
        }
        if self.intervals_ms.len() > 180 {
            self.intervals_ms.pop_front();
        }
        if !self.frames.is_multiple_of(30) || self.intervals_ms.is_empty() {
            return None;
        }
        let mut sorted: Vec<_> = self.intervals_ms.iter().copied().collect();
        sorted.sort_by(f64::total_cmp);
        Some(serde_json::json!({"screen":screen,"frames":self.frames,"samples":sorted.len(),"median_ms":sorted[sorted.len()/2],"p95_ms":sorted[(sorted.len()-1)*95/100]}).to_string())
    }
}
