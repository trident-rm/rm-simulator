// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Pilot camera, browser pointer capture and authoritative match HUD.
use bevy::{input::mouse::AccumulatedMouseMotion, prelude::*};
use rm_sim_render::{apply_pose, flu_position, sync::SceneSyncSet};
use rm_sim_server::state::SimulationState;
use rm_sim_world::ChassisSnapshot;

/// Input is sampled before protocol commands and camera presentation.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(super) enum BrowserViewSet {
    Input,
}

#[derive(Resource, Default)]
pub(super) struct PilotView {
    pub input_active: bool,
    pub fire: bool,
    /// Captured relative pointer motion in CSS pixels for this frame.
    pub mouse_delta: Vec2,
    cameras: Option<[Transform; 2]>,
    third_person: bool,
    hud: String,
    captured: bool,
    mouse_armed: bool,
}

pub(super) fn install(app: &mut App) {
    app.init_resource::<PilotView>()
        .add_systems(Update, input.in_set(BrowserViewSet::Input))
        .add_systems(Update, camera.after(SceneSyncSet));
    if let Some(document) = document() {
        if let Some(element) = document.get_element_by_id("pilot-hud") {
            let _ = element.remove_attribute("hidden");
        }
        if let Some(element) = document.get_element_by_id("crosshair") {
            let _ = element.remove_attribute("hidden");
        }
    }
}

fn document() -> Option<web_sys::Document> {
    web_sys::window()?.document()
}

fn input(
    mut view: ResMut<PilotView>,
    keys: Res<ButtonInput<KeyCode>>,
    buttons: Res<ButtonInput<MouseButton>>,
    mouse: Res<AccumulatedMouseMotion>,
) {
    let Some(document) = document() else {
        return;
    };
    let active = document.active_element();
    let editing = active.as_ref().is_some_and(|element| {
        matches!(
            element.tag_name().as_str(),
            "INPUT" | "SELECT" | "BUTTON" | "TEXTAREA"
        )
    });
    let captured = document
        .pointer_lock_element()
        .is_some_and(|element| element.id() == "sim");
    let previously_captured = view.captured;
    view.captured = captured;
    view.input_active = !editing && document.has_focus().unwrap_or(false);
    if captured != previously_captured || !view.input_active {
        view.mouse_armed = false;
    }
    if captured && !buttons.pressed(MouseButton::Left) {
        view.mouse_armed = true;
    }
    view.fire = view.input_active
        && (keys.pressed(KeyCode::Space)
            || (captured && view.mouse_armed && buttons.pressed(MouseButton::Left)));
    view.mouse_delta = if view.input_active && captured && previously_captured {
        mouse.delta
    } else {
        Vec2::ZERO
    };
    if view.input_active && keys.just_pressed(KeyCode::KeyV) {
        view.third_person = !view.third_person;
    }
    if view.input_active && !captured && buttons.just_pressed(MouseButton::Left) {
        // Only a click that targeted the canvas captures. HTML controls remain usable.
        if active.is_some_and(|element| element.id() == "sim")
            && let Some(canvas) = document.get_element_by_id("sim")
        {
            canvas.request_pointer_lock();
        }
    }
    if keys.just_pressed(KeyCode::Escape) {
        document.exit_pointer_lock();
    }
    if captured != previously_captured
        && let Some(element) = document.get_element_by_id("crosshair")
    {
        let _ = element.set_attribute("data-captured", if captured { "true" } else { "false" });
    }
}

/// Accept a caller-presented pose and authority-owned rule values.
pub(super) fn observe(view: &mut PilotView, state: &SimulationState, own: &ChassisSnapshot) {
    view.cameras = Some(camera_poses(own));
    let robot = state
        .field
        .rmul_2027
        .as_ref()
        .and_then(|game| game.robots.iter().find(|robot| robot.id == own.id))
        .map(|robot| {
            format!(
                "{} / {} HP   ·   {} rounds   ·   Heat {:.0}",
                robot.hp,
                robot.max_hp,
                robot.allowance,
                robot.heat_tenths as f64 / 10.
            )
        })
        .or_else(|| {
            state
                .field
                .rmuc_2027
                .as_ref()
                .and_then(|state| state.game.robots.iter().find(|robot| robot.id == own.id))
                .map(|robot| {
                    format!(
                        "{} / {} HP   ·   {} rounds   ·   Heat {:.0}",
                        robot.hp,
                        robot.max_hp,
                        robot.combat.allowance,
                        robot.combat.heat_tenths as f64 / 10.
                    )
                })
        })
        .unwrap_or_else(|| "Practice · Unlimited ammunition".into());
    let speed_m_s = own.velocity_m_s[0].hypot(own.velocity_m_s[1]);
    let text = format!(
        "{:?} #{}   ·   {:.1} m/s\n{}{}",
        own.team,
        own.id,
        speed_m_s,
        robot,
        if own.defeated { "   ·   DEFEATED" } else { "" }
    );
    if text != view.hud {
        view.hud = text;
        if let Some(element) =
            document().and_then(|document| document.get_element_by_id("pilot-hud"))
        {
            element.set_text_content(Some(&view.hud));
        }
    }
}

fn camera_poses(own: &ChassisSnapshot) -> [Transform; 2] {
    let cradle = rm_sim_presentation::chassis_appearance(own, &[], None).turret;
    let mut mount = Transform::default();
    apply_pose(&mut mount, cradle);
    let third_person = {
        let heading = Quat::from_rotation_y(mount.rotation.to_euler(EulerRot::YXZ).0);
        let mut eye = mount.translation
            + mount.rotation * Vec3::Z * 2.4
            + heading * Vec3::X * 0.45
            + Vec3::Y * 0.9;
        eye.y = eye.y.max(mount.translation.y);
        Transform::from_translation(eye).with_rotation(mount.rotation)
    };
    let first_person = {
        let offset = if let Some(articulation) = &own.config.articulation {
            flu_position(articulation.camera_offset_m)
        } else if let Some(folding) = &own.config.folding {
            flu_position(folding.camera_offset_m)
        } else if own.config.team_cad {
            rm_sim_render::team_robot::camera_eye()
        } else {
            rm_sim_render::chassis::camera_eye(own.config.turret_half_m.map(|value| value as f32))
        };
        Transform::from_translation(mount.translation + mount.rotation * offset)
            .with_rotation(mount.rotation)
    };
    [first_person, third_person]
}

fn camera(view: Res<PilotView>, mut cameras: Query<&mut Transform, With<super::FieldCamera>>) {
    let Some(poses) = &view.cameras else {
        return;
    };
    let next = poses[usize::from(view.third_person)];
    for mut camera in &mut cameras {
        if *camera != next {
            *camera = next;
        }
    }
    if let Some(element) = document().and_then(|document| document.get_element_by_id("status")) {
        let mode = if view.third_person {
            "third-person"
        } else {
            "first-person"
        };
        if element.get_attribute("data-camera").as_deref() != Some(mode) {
            let _ = element.set_attribute("data-camera", mode);
        }
    }
}
