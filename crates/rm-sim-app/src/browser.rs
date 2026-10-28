// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Browser launch options for the ordinary application lifecycle.
use clap::Parser;
pub(crate) fn args() -> crate::args::Args {
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
        let competition = match query.get("field").as_deref() {
            Some("3v3") => "rmul-2027",
            Some("rmuc") => "rmuc-2027",
            _ => "rmul-infantry-2027",
        };
        values.extend(["--competition".into(), competition.into()]);
        for (key, flag) in [
            ("team", "--team"),
            ("name", "--name"),
            ("password", "--password"),
        ] {
            if let Some(value) = query.get(key).filter(|s| !s.is_empty()) {
                values.extend([flag.into(), value]);
            }
        }
    }
    crate::args::Args::parse_from(values)
}
pub(crate) fn start() {
    let _ = crate::native::run();
}

pub(crate) fn install(app: &mut bevy::prelude::App) {
    use bevy::prelude::*;
    let inspect = web_sys::window()
        .and_then(|w| w.location().search().ok())
        .is_some_and(|q| q.contains("inspect=1"));
    if inspect {
        app.add_systems(PostUpdate, inspect_ui.after(bevy::ui::UiSystems::Layout));
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
    let mut values = vec![("data-screen", state)];
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
