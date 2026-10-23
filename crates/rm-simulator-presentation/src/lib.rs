// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Pure RoboMaster snapshot-to-renderer adaptation, with caller-owned time and outputs.
//! No host, transport, UI, or session clock is required.
#![deny(missing_docs)]
use glam::{DQuat, DVec3};
use rm_simulator_render::{
    PoseFlu, TeamColor,
    armor::ArmorPattern,
    chassis::ArmorOptics,
    sync::{
        ArmorAppearance, ChassisAppearance, OutpostAppearance, ProjectileAppearance,
        RuneAppearance, SceneState, SourceTime, WheelAppearance,
    },
};
use rm_simulator_world::{
    ArmorHit, ArmorTarget, ChassisSnapshot, FieldSnapshot, Pose, RuneKind, RuneState,
    SMALL_ARMOR_HOUSING_HALF_M, Team, outpost as outpost_rules,
};
use std::collections::HashMap;
/// Visual settings for the world-to-scene adaptation.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FlashSettings {
    /// How long a detected armor hit stays lit, in ns. An app setting: the
    /// rulebook gives no flash duration.
    pub hit_flash_ns: u64,
    /// Blink rate of a newly activated rune, in Hz. The rulebook fixes only
    /// that the arms light up, so the rate is an app setting.
    pub rune_flash_hz: f64,
    /// Number of blinks a newly activated rune shows. An app setting.
    pub rune_flashes: u32,
}
/// Caller-owned robot identity used only to choose its visual model and armor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Robot {
    /// Hero, with armor number one.
    Hero,
    /// Engineer, with number two (one for a folding Hero conversion).
    Engineer,
    /// Infantry number three.
    Infantry3,
    /// Infantry number four.
    Infantry4,
    /// Infantry number five.
    Infantry5,
    /// Sentry, with the guard artwork.
    Sentry,
    /// Drone, with number three.
    Drone,
}
/// The world's `Pose` as the renderer's `PoseFlu`, again without converting axes.
fn pose_flu(value: Pose) -> PoseFlu {
    PoseFlu {
        translation_m: value.translation_m,
        rotation_wxyz: value.rotation_wxyz,
    }
}
/// A world wxyz quaternion as Bevy's xyzw `DQuat`. The world orders the scalar
/// component first and Bevy orders it last.
fn dquat(rotation_wxyz: [f64; 4]) -> DQuat {
    let [w, x, y, z] = rotation_wxyz;
    DQuat::from_xyzw(x, y, z, w)
}
/// A Bevy `DQuat` as the world's wxyz quaternion array, the inverse of `dquat`.
fn wxyz(q: DQuat) -> [f64; 4] {
    [q.w, q.x, q.y, q.z]
}
/// The two stages of a gimbal bolted to the body, from the body pose and the
/// aim the pilot holds in the world: the yaw stage turns about the body's
/// own up axis and the pitch stage about the yaw stage's left axis, by the
/// angles that point the barrel (+x) along the aim. A two-axis mount cannot
/// roll, so on a tilted chassis the barrel rolls with the body while its
/// direction stays where the pilot points it.
fn gimbal_poses(body: PoseFlu, aim: PoseFlu) -> (PoseFlu, PoseFlu) {
    let body_rotation = dquat(body.rotation_wxyz);
    let direction = body_rotation.inverse() * (dquat(aim.rotation_wxyz) * DVec3::X);
    let yaw = direction.y.atan2(direction.x);
    let pitch = direction.z.clamp(-1.0, 1.0).asin();
    let yaw_stage = body_rotation * DQuat::from_rotation_z(yaw);
    let turret = yaw_stage * DQuat::from_rotation_y(-pitch);
    (
        PoseFlu {
            translation_m: aim.translation_m,
            rotation_wxyz: wxyz(yaw_stage.normalize()),
        },
        PoseFlu {
            translation_m: aim.translation_m,
            rotation_wxyz: wxyz(turret.normalize()),
        },
    )
}
/// Wheel visual pose: hub in the world, local +y along the axle, spun about it.
fn wheel_pose(
    chassis: &ChassisSnapshot,
    hub_m: [f64; 2],
    wheel: &rm_simulator_world::WheelSnapshot,
) -> PoseFlu {
    let axle_yaw = if chassis.config.mecanum {
        0.0
    } else {
        hub_m[1].atan2(hub_m[0]) - std::f64::consts::FRAC_PI_2
    };
    let rotation = dquat(chassis.pose.rotation_wxyz)
        * DQuat::from_rotation_z(axle_yaw)
        * DQuat::from_rotation_y(-wheel.spin_rad);
    let centre = wheel.hub_m;
    PoseFlu {
        translation_m: centre,
        rotation_wxyz: wxyz(rotation.normalize()),
    }
}
/// The small armor module geometry the renderer draws on every chassis,
/// from the rules: the same module as the outpost's middle armor.
pub fn armor_optics() -> ArmorOptics {
    ArmorOptics {
        housing_half_m: SMALL_ARMOR_HOUSING_HALF_M.map(|v| v as f32),
        face_size_m: [
            outpost_rules::FACE_WIDTH_M as f32,
            outpost_rules::FACE_HEIGHT_M as f32,
        ],
        light_span_m: outpost_rules::LIGHT_SPAN_M as f32,
        light_length_m: outpost_rules::LIGHT_LENGTH_M as f32,
    }
}
/// The number painted on a chassis: the pilot's robot when the roster names
/// it, else 1 for a mecanum Hero and 3 for any other (a bot, or a chassis the
/// roster has not described yet).
fn armor_pattern(chassis: &ChassisSnapshot, robot: Option<Robot>) -> ArmorPattern {
    match robot {
        Some(Robot::Hero) => ArmorPattern::One,
        Some(Robot::Engineer) if chassis.config.folding.is_some() => ArmorPattern::One,
        Some(Robot::Engineer) => ArmorPattern::Two,
        Some(Robot::Sentry) => ArmorPattern::GuardSmall,
        Some(Robot::Drone) => ArmorPattern::Three,
        Some(Robot::Infantry3) => ArmorPattern::Three,
        Some(Robot::Infantry4) => ArmorPattern::Four,
        Some(Robot::Infantry5) => ArmorPattern::Five,
        None if chassis.config.mecanum => ArmorPattern::One,
        None => ArmorPattern::Three,
    }
}
/// One chassis for the renderer, using the actual host or predicted motor pose. `flash`
/// lists the armor plates flashing after a strike; `robot` is what the roster
/// says its pilot drives.
///
/// The inline hint preserves caller optimization of this per-frame path across
/// crate boundaries.
#[inline]
pub fn chassis_appearance(
    chassis: &ChassisSnapshot,
    flash: &[bool],
    robot: Option<Robot>,
) -> ChassisAppearance {
    let config = &chassis.config;
    let aim = pose_flu(chassis.turret);
    let (yaw_stage, turret) = gimbal_poses(pose_flu(chassis.pose), aim);
    ChassisAppearance {
        source_joints: if let Some(a) = &config.articulation {
            let q = dquat(chassis.pose.rotation_wxyz);
            let aimq = dquat(chassis.turret.rotation_wxyz);
            let d = q.inverse() * (aimq * DVec3::X);
            a.joints
                .iter()
                .zip(a.poses(
                    chassis.fold_angle_rad,
                    [d.y.atan2(d.x), d.z.clamp(-1., 1.).asin()],
                ))
                .map(|(j, p)| (j.name.clone(), pose_flu(p)))
                .collect()
        } else {
            config
                .climbing
                .as_ref()
                .map(|c| {
                    let p = c.poses(chassis.fold_angle_rad);
                    [
                        ("front", p.front),
                        ("coupler", p.coupler),
                        ("rear", p.rear),
                        ("guide", p.guide),
                        ("slider", p.slider),
                    ]
                    .into_iter()
                    .map(|(name, p)| (name.to_string(), pose_flu(p)))
                    .collect()
                })
                .unwrap_or_default()
        },
        fold_angle_rad: chassis.fold_angle_rad as f32,
        fold_collision_parts: {
            let mut parts = config.transform_collision_parts(
                chassis.pose,
                chassis.held_aim_rad,
                chassis.fold_angle_rad,
            );
            if config.climbing.is_some() {
                // Include the fixed upper proxy and rear shelf in debug/fallback drawings.
                let q = dquat(chassis.pose.rotation_wxyz);
                for (center, half) in [
                    (config.turret_center_m, config.turret_half_m),
                    ([-0.075, 0., -0.16], [0.05, 0.26, 0.006]),
                ] {
                    let p = DVec3::from_array(chassis.pose.translation_m)
                        + q * DVec3::from_array(center);
                    parts.push((
                        rm_simulator_world::Pose {
                            translation_m: p.to_array(),
                            rotation_wxyz: chassis.pose.rotation_wxyz,
                        },
                        half,
                    ));
                }
            }
            parts
                .into_iter()
                .map(|(p, h)| (pose_flu(p), h.map(|v| v as f32)))
                .collect()
        },
        model: {
            use rm_simulator_render::chassis::RobotModel;
            if config.articulation.is_some() {
                RobotModel::Articulated
            } else if config.climbing.is_some() {
                RobotModel::StairHeavy
            } else if config.folding.is_some() {
                RobotModel::FoldingHero
            } else if config.team_cad {
                RobotModel::TeamCad
            } else {
                match robot {
                    Some(Robot::Engineer) => RobotModel::Engineer,
                    Some(Robot::Drone) => RobotModel::Drone,
                    Some(Robot::Sentry) => RobotModel::Sentry,
                    Some(Robot::Hero) => RobotModel::Hero,
                    _ if config.balance_assist => RobotModel::Balance,
                    None if config.mecanum => RobotModel::Hero,
                    _ => RobotModel::Omni,
                }
            }
        },
        armor_pattern: armor_pattern(chassis, robot),
        mecanum: config.mecanum,
        hp_fraction: if chassis.defeated { 0.0 } else { 1.0 },
        id: chassis.id,
        team: team_color(chassis.team),
        pose: pose_flu(chassis.pose),
        body_half_m: config.body_half_m.map(|v| v as f32),
        body_shell_visible: true,
        frame_beam_step_m: config.frame_beam_step_m() as f32,
        body_outline_m: config
            .body_outline_m()
            .iter()
            .map(|p| p.map(|v| v as f32))
            .collect(),
        bumper_base_prisms: if config.omni_frame.is_some() {
            config.body_outline_m().len()
        } else {
            0
        },
        bumper_prisms_m: config
            .bumper_prisms_m()
            .iter()
            .map(|p| p.map(|p| p.map(|v| v as f32)))
            .collect(),
        wheels: chassis
            .wheels
            .iter()
            .zip(&config.wheel_hubs_m)
            .enumerate()
            .map(|(index, (wheel, hub))| WheelAppearance {
                pose: wheel_pose(chassis, *hub, wheel),
                suspension_anchor_m: {
                    let mounts = config
                        .suspension_mounts_at_m(
                            index,
                            wheel.suspension_compression_m,
                            chassis.fold_angle_rad,
                        )
                        .unwrap();
                    (DVec3::from_array(chassis.pose.translation_m)
                        + dquat(chassis.pose.rotation_wxyz) * DVec3::from_array(mounts[0]))
                    .to_array()
                },
                suspension_lower_m: {
                    let mounts = config
                        .suspension_mounts_at_m(
                            index,
                            wheel.suspension_compression_m,
                            chassis.fold_angle_rad,
                        )
                        .unwrap();
                    (DVec3::from_array(chassis.pose.translation_m)
                        + dquat(chassis.pose.rotation_wxyz) * DVec3::from_array(mounts[1]))
                    .to_array()
                },
                radius_m: config.wheel_radius_m as f32,
                width_m: config.wheel_width_m as f32,
            })
            .collect(),
        armor: config
            .armor_faces()
            .iter()
            .enumerate()
            .map(|(plate, face)| ArmorAppearance {
                optics: Some(ArmorOptics {
                    housing_half_m: config.armor_housing_half_m().map(|v| v as f32),
                    face_size_m: [
                        if config.folding.is_some() {
                            0.223
                        } else {
                            0.128
                        },
                        0.113,
                    ],
                    light_span_m: if config.folding.is_some() {
                        0.225
                    } else {
                        0.130
                    },
                    light_length_m: 0.056,
                }),
                plate_visible: true,
                bracket_prisms_m: config
                    .armor_bracket_prisms_m(plate)
                    .iter()
                    .map(|p| p.map(|p| p.map(|v| v as f32)))
                    .collect(),
                local: pose_flu(*face),
                hit_flash: flash.get(plate).copied().unwrap_or(false),
            })
            .collect(),
        yaw_stage,
        turret,
        turret_half_m: config.turret_half_m.map(|v| v as f32),
        pivot_above_body_m: (config.turret_center_m[2] - config.body_half_m[2]) as f32,
        barrel_length_m: config.muzzle_offset_m()[0] as f32,
        defeated: chassis.defeated,
    }
}

/// Detected strikes still within the flash window.
pub fn flashing(snapshot: &FieldSnapshot, hit_flash_ns: u64) -> impl Iterator<Item = &ArmorHit> {
    snapshot.hits.iter().filter(move |hit| {
        hit.detected && snapshot.time_ns.saturating_sub(hit.time_ns) < hit_flash_ns
    })
}

/// Blink an activated rune: `flashes` on/off cycles at `flash_hz`, then lit.
/// A zero rate never blinks; a zero count blinks until the state changes.
fn rune_lit(since_ns: u64, now_ns: u64, flash_hz: f64, flashes: u32) -> bool {
    if flash_hz <= 0.0 {
        return true;
    }
    let phase = now_ns.saturating_sub(since_ns) as f64 * 1e-9 * flash_hz;
    if flashes > 0 && phase >= f64::from(flashes) {
        return true;
    }
    phase.fract() < 0.5
}

fn team_color(team: Team) -> TeamColor {
    match team {
        Team::Red => TeamColor::Red,
        Team::Blue => TeamColor::Blue,
    }
}

#[cfg(test)]
fn scene_state(snapshot: &FieldSnapshot, flash: FlashSettings) -> SceneState {
    scene_state_at(snapshot, flash, snapshot.time_ns)
}

#[cfg(test)]
fn scene_state_at(
    snapshot: &FieldSnapshot,
    flash: FlashSettings,
    presentation_ns: u64,
) -> SceneState {
    let hits: Vec<_> = flashing(snapshot, flash.hit_flash_ns).collect();
    scene_state_with_hits(snapshot, flash, presentation_ns, &hits)
}

/// Build rule appearances and snapshot projectiles at caller-supplied time.
/// Chassis selection is left to the caller; use [`chassis_appearance`].
pub fn scene_state_with_hits(
    snapshot: &FieldSnapshot,
    flash: FlashSettings,
    presentation_ns: u64,
    hits: &[&ArmorHit],
) -> SceneState {
    let mut scene = SceneState::default();
    scene_state_into(snapshot, flash, presentation_ns, hits, &mut scene);
    scene.projectiles.extend(
        snapshot
            .projectiles
            .iter()
            .map(|projectile| ProjectileAppearance {
                position_m: projectile.position_m,
                radius_m: (projectile.caliber.diameter_m() / 2.0) as f32,
            }),
    );
    scene
}

/// Update rule appearances, retaining output vector capacities.
/// Leaves projectiles and chassis untouched so the caller can fill its selected
/// predicted/interpolated appearances directly, without discarded intermediates.
///
/// ```
/// use rm_simulator_presentation::{FlashSettings, scene_state_into};
/// use rm_simulator_render::sync::SceneState;
/// # let snapshot = rm_simulator_world::Field::new(&Default::default()).unwrap().snapshot();
/// let mut scene = SceneState::default();
/// let flash = FlashSettings { hit_flash_ns: 50_000_000, rune_flash_hz: 0., rune_flashes: 3 };
/// scene_state_into(&snapshot, flash, snapshot.time_ns, &[], &mut scene);
/// assert_eq!(scene.source.time_ns, snapshot.time_ns);
/// ```
pub fn scene_state_into(
    snapshot: &FieldSnapshot,
    flash: FlashSettings,
    presentation_ns: u64,
    hits: &[&ArmorHit],
    scene: &mut SceneState,
) {
    let mut rune_flash: HashMap<u32, [bool; 5]> = HashMap::new();
    let mut outpost_flash: HashMap<u32, [bool; rm_simulator_render::outpost::FACE_COUNT]> =
        HashMap::new();
    for hit in hits {
        match hit.target {
            ArmorTarget::Rune { rune, blade } => {
                if let Some(flag) = rune_flash.entry(rune).or_default().get_mut(blade as usize) {
                    *flag = true;
                }
            }
            ArmorTarget::Outpost { outpost, face } => {
                if let Some(flag) = outpost_flash
                    .entry(outpost)
                    .or_default()
                    .get_mut(face as usize)
                {
                    *flag = true;
                }
            }
            // Chassis flashes are gathered with the chassis in `publish_scene`.
            ArmorTarget::Chassis { .. } | ArmorTarget::Base { .. } => {}
        }
    }
    let dart_position = rm_simulator_world::referee::DART_TARGET_REST;
    scene.bases.clear();
    scene
        .bases
        .extend(snapshot.bases.iter().enumerate().map(|(index, base)| {
            rm_simulator_render::sync::BaseAppearance {
                disabled: base.hp == 0,
                plates: std::array::from_fn(|plate| pose_flu(base.pose(plate, dart_position))),
                hit_flash: std::array::from_fn(|plate| {
                    hits.iter().any(|hit| {
                        hit.target
                            == ArmorTarget::Base {
                                base: index as u32,
                                plate: plate as u32,
                            }
                    })
                }),
            }
        }));
    scene.dart_target_fraction = dart_position as f32;
    scene.base_open_fraction = [0.; 2];
    scene.dart_door_open = [true; 2];
    scene.source = SourceTime {
        tick: snapshot.tick,
        time_ns: snapshot.time_ns,
    };
    scene.runes.clear();
    scene
        .runes
        .extend(snapshot.runes.iter().enumerate().map(|(index, rune)| {
            let big = rune.kind == RuneKind::Big;
            let activated = rune.state == RuneState::Activated;
            // An activated rune blinks its arms (a visual aid, see
            // `--rune-flash-hz`); the rule text shows them lit.
            let lit = !activated
                || rune_lit(
                    rune.state_since_ns,
                    presentation_ns,
                    flash.rune_flash_hz,
                    flash.rune_flashes,
                );
            RuneAppearance {
                hub_pose: pose_flu(rune.hub_pose),
                angle_rad: rune.presentation_angle_rad(presentation_ns),
                active_blades: rune.active_blades,
                // Big Rune shows hit blades only once the whole rune is activated.
                activated_blades: rune.activated.map(|hit| hit && lit && (!big || activated)),
                fully_activated: activated && lit,
                progress_stages: std::array::from_fn(|stage| {
                    big && lit && (stage as u32) < rune.completed_groups
                }),
                hit_flash: rune_flash.get(&(index as u32)).copied().unwrap_or_default(),
            }
        }));
    scene.outposts.clear();
    scene.outposts.extend(
        snapshot
            .outposts
            .iter()
            .enumerate()
            .map(|(index, outpost)| {
                let outpost = outpost.presentation_at(presentation_ns);
                OutpostAppearance {
                    disabled: outpost.destroyed,
                    origin: pose_flu(outpost.origin),
                    angle_rad: outpost.angle_rad,
                    armors: std::array::from_fn(|face| {
                        pose_flu(
                            outpost
                                .armors
                                .iter()
                                .find(|armor| armor.id == face as u32)
                                .expect("outpost snapshot has three faces")
                                .pose,
                        )
                    }),
                    hit_flash: outpost_flash
                        .get(&(index as u32))
                        .copied()
                        .unwrap_or_default(),
                }
            }),
    );
}

/// Apply native RMUC rules, target lights and HP overrides to caller-owned outputs.
/// `presentation_ns` is chosen by the caller; this function never reads a clock.
pub fn native_scene_into(
    snapshot: &FieldSnapshot,
    presentation_ns: u64,
    paused: bool,
    hits: &[&ArmorHit],
    scene: &mut SceneState,
    targets: &mut Vec<rm_simulator_render::targets::TargetAppearance>,
) {
    let time_ns = presentation_ns;
    targets.clear();
    if let Some(state) = snapshot.rmuc_2027.as_ref() {
        let round_ns = state.game.elapsed_ns()
            + if !paused
                && state.game.phase == rm_simulator_world::rmuc_2027::match_rules::Phase::Running
            {
                time_ns.saturating_sub(snapshot.time_ns)
            } else {
                0
            };
        scene.base_open_fraction = state.game.base_open.map(|v| if v { 1. } else { 0. });
        scene.outposts.clear();
        scene.outposts.extend(Team::BOTH.map(|side| {
            let tower = &state.config.outposts[side.index()];
            OutpostAppearance {
                disabled: state.game.outpost_hp[side.index()] == 0,
                origin: pose_flu(rm_simulator_world::Pose::at(tower.pivot_m)),
                angle_rad: state.angle_at(side, round_ns),
                armors: std::array::from_fn(|i| {
                    pose_flu(state.rotated_at(side, tower.detectors[i].pose, round_ns))
                }),
                hit_flash: [false; 3],
            }
        }));
        scene.runes.clear();
        scene
            .runes
            .extend(state.config.runes.iter().enumerate().map(|(i, geometry)| {
                let r = &state.game.runes[i];
                RuneAppearance {
                    hub_pose: pose_flu(rm_simulator_world::Pose::at(geometry.pivot_m)),
                    angle_rad: r.angle_at(round_ns),
                    active_blades: r.targets,
                    activated_blades: r.completed_blades,
                    progress_stages: std::array::from_fn(|n| n < usize::from(r.groups)),
                    fully_activated: r.phase
                        == rm_simulator_world::gameplay::rmuc_2027::rune::Phase::Activated,
                    hit_flash: std::array::from_fn(|blade| {
                        hits.iter().any(|h| {
                            h.target
                                == ArmorTarget::Rune {
                                    rune: i as u32,
                                    blade: blade as u32,
                                }
                        })
                    }),
                }
            }));
        for side in Team::BOTH {
            let i = side.index();
            if let Some(geometry) = state.config.runes.get(i) {
                let r = &state.game.runes[i];
                for (blade, rest) in geometry.targets.iter().enumerate() {
                    targets.push(rm_simulator_render::targets::TargetAppearance {
                        id: (512 + i * 5 + blade) as u64,
                        pose: pose_flu(state.rune_pose_at(side, *rest, round_ns)),
                        size_m: [0.3; 2],
                        team: team_color(side),
                        disabled: !r.targets[blade] && !r.completed_blades[blade],
                        flash: hits.iter().any(|h| {
                            h.target
                                == ArmorTarget::Rune {
                                    rune: i as u32,
                                    blade: blade as u32,
                                }
                        }),
                    });
                }
            }
            let owner = match state.game.control.snapshot().outposts[i].owner {
                rm_simulator_world::gameplay::Team::Red => Team::Red,
                rm_simulator_world::gameplay::Team::Blue => Team::Blue,
            };
            for (index, d) in state.config.bases[i].iter().enumerate() {
                let target = ArmorTarget::Base {
                    base: i as u32,
                    plate: index as u32,
                };
                targets.push(rm_simulator_render::targets::TargetAppearance {
                    id: (i * 32 + index) as u64,
                    pose: pose_flu(d.pose),
                    size_m: d.half_m.map(|v| (v * 2.) as f32),
                    team: team_color(side),
                    disabled: state.game.base_hp[i] == 0,
                    flash: hits.iter().any(|h| h.target == target),
                });
            }
            for (index, d) in state.config.outposts[i].detectors.iter().enumerate() {
                let target = ArmorTarget::Outpost {
                    outpost: i as u32,
                    face: index as u32,
                };
                targets.push(rm_simulator_render::targets::TargetAppearance {
                    id: (128 + i * 32 + index) as u64,
                    pose: pose_flu(state.rotated_at(side, d.pose, round_ns)),
                    size_m: d.half_m.map(|v| (v * 2.) as f32),
                    team: team_color(owner),
                    disabled: state.game.outpost_hp[i] == 0,
                    flash: hits.iter().any(|h| h.target == target),
                });
            }
        }
        for c in &mut scene.chassis {
            if let Some(r) = state.game.robots.iter().find(|r| r.id == c.id) {
                c.defeated = r.hp == 0;
                c.hp_fraction = r.hp as f32 / r.max_hp as f32;
            }
        }
    }
}

#[cfg(test)]
mod fixtures {
    use rm_simulator_world::{OutpostSnapshot, Pose, RuneKind, RuneSnapshot, RuneState};

    pub fn rune(hub_pose: Pose) -> RuneSnapshot {
        RuneSnapshot {
            motion: None,
            kind: RuneKind::Small,
            active_blades: [true, false, false, false, false],
            completed_groups: 0,
            angular_speed_rad_s: std::f64::consts::PI / 3.,
            time_ns: 0,
            hub_pose,
            target_radius_m: 0.7,
            angle_rad: 0.,
            state: RuneState::Activating,
            active_blade: Some(0),
            activated: [false; 5],
            target_poses: rm_simulator_world::rune::target_poses(hub_pose, 0.7, 0.),
            state_since_ns: 0,
        }
    }
    pub fn outpost(origin: Pose) -> OutpostSnapshot {
        let mut snapshot = OutpostSnapshot {
            speed_rad_s: rm_simulator_world::outpost::DEFAULT_SPEED_RAD_S,
            pivot_cad_m: rm_simulator_world::outpost::PIVOT_CAD_M,
            time_ns: 0,
            origin,
            angle_rad: 0.,
            armors: vec![],
            hp: 1000,
            destroyed: false,
            started_ns: None,
            stopped_ns: None,
            homing: false,
        };
        snapshot.rebuild_armors();
        snapshot
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DVec3, Quat, Vec3};
    use rm_simulator_world::{Caliber, ChassisConfig, Field, Pose, tick_ns};

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        a.iter().zip(&b).all(|(a, b)| (a - b).abs() < 1e-4)
    }
    fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
        let [w, x, y, z] = q.map(|v| v as f32);
        let out = Quat::from_xyzw(x, y, z, w) * Vec3::new(v[0] as f32, v[1] as f32, v[2] as f32);
        [out.x as f64, out.y as f64, out.z as f64]
    }
    #[test]
    fn presentation_reconstructs_motion_without_changing_source_or_authority() {
        let mut checkpoint = Field::new(&Default::default()).unwrap().snapshot();
        checkpoint
            .runes
            .push(fixtures::rune(Pose::at([6., 0., 1.6])));
        checkpoint
            .outposts
            .push(fixtures::outpost(Pose::at([7., 0., 0.])));
        let flash = FlashSettings {
            hit_flash_ns: 50_000_000,
            rune_flash_hz: 2.,
            rune_flashes: 3,
        };
        let at = 123_000_000 / tick_ns() * tick_ns();
        let presented = scene_state_at(&checkpoint, flash, at);
        assert!(
            (presented.runes[0].angle_rad - checkpoint.runes[0].presentation_angle_rad(at)).abs()
                < 1e-12
        );
        assert_eq!(presented.source.tick, checkpoint.tick);
        assert_eq!(presented.source.time_ns, checkpoint.time_ns);
        let exact = scene_state_at(&checkpoint, flash, checkpoint.time_ns);
        assert_eq!(
            exact.outposts[0].angle_rad,
            checkpoint.outposts[0].angle_rad
        );
    }

    #[test]
    fn rune_state_masks_clear_on_failure_deactivation_and_big_completion_blinks() {
        let mut snapshot = Field::new(&Default::default()).unwrap().snapshot();
        snapshot.time_ns = 3_000_000_000;
        snapshot.runes.push(fixtures::rune(Pose::at([6., 0., 1.6])));
        let flash = FlashSettings {
            hit_flash_ns: 50_000_000,
            rune_flash_hz: 2.,
            rune_flashes: 3,
        };
        snapshot.runes[0].activated[0] = true;
        assert!(scene_state(&snapshot, flash).runes[0].activated_blades[0]);
        snapshot.runes[0].activated = [false; 5];
        assert_eq!(
            scene_state(&snapshot, flash).runes[0].activated_blades,
            [false; 5]
        );
        let rune = &mut snapshot.runes[0];
        rune.kind = RuneKind::Big;
        rune.state = RuneState::Activated;
        rune.activated = [true; 5];
        rune.completed_groups = 5;
        rune.state_since_ns = snapshot.time_ns - 300_000_000;
        let dark = scene_state(&snapshot, flash);
        assert_eq!(dark.runes[0].activated_blades, [false; 5]);
        assert_eq!(dark.runes[0].progress_stages, [false; 5]);
        assert!(!dark.runes[0].fully_activated);
        snapshot.time_ns += 300_000_000;
        let lit = scene_state(&snapshot, flash);
        assert_eq!(lit.runes[0].progress_stages, [true; 5]);
        assert!(lit.runes[0].fully_activated);
    }

    #[test]
    fn activated_runes_blink_at_the_configured_rate() {
        // 2 Hz: lit for the first quarter second of each half second.
        assert!(rune_lit(1_000_000_000, 1_000_000_000, 2.0, 3));
        assert!(rune_lit(1_000_000_000, 1_249_000_000, 2.0, 3));
        assert!(!rune_lit(1_000_000_000, 1_250_000_000, 2.0, 3));
        assert!(!rune_lit(1_000_000_000, 1_499_000_000, 2.0, 3));
        assert!(rune_lit(1_000_000_000, 1_500_000_000, 2.0, 3));
        // Three flashes take 1.5 s; the dark half of the fourth cycle stays lit.
        assert!(!rune_lit(1_000_000_000, 2_300_000_000, 2.0, 3));
        assert!(rune_lit(1_000_000_000, 2_800_000_000, 2.0, 3));
        assert!(rune_lit(1_000_000_000, 9_800_000_000, 2.0, 3));
        // A zero count blinks forever; a zero rate disables the blink;
        // a clock behind the activation stays lit.
        assert!(!rune_lit(1_000_000_000, 9_800_000_000, 2.0, 0));
        assert!(rune_lit(1_000_000_000, 1_300_000_000, 0.0, 3));
        assert!(rune_lit(5_000_000_000, 1_000_000_000, 2.0, 3));
        // The scene adaptation applies it to the activated arms only.
        let mut snapshot = Field::new(&Default::default()).unwrap().snapshot();
        let mut rune = fixtures::rune(Pose::at([6., 0., 1.6]));
        rune.state = RuneState::Activated;
        rune.activated = [true; 5];
        rune.active_blades = [false; 5];
        snapshot.runes.push(rune);
        let since = snapshot.runes[0].state_since_ns;
        let flash = |hz| FlashSettings {
            hit_flash_ns: 50_000_000,
            rune_flash_hz: hz,
            rune_flashes: 3,
        };
        let lit = scene_state(&snapshot, flash(0.0));
        assert_eq!(lit.runes[0].activated_blades, [true; 5]);
        // Step into the dark half of a 2 Hz blink: the first dark moment at
        // least one step past the snapshot, wrapped forward by blink periods.
        let period = 500_000_000;
        let mut dark_at = since + 300_000_000;
        dark_at += (snapshot.time_ns.saturating_sub(dark_at) / period + 1) * period;
        snapshot.time_ns = dark_at;
        let dark = scene_state(&snapshot, flash(2.0));
        assert_eq!(dark.runes[0].activated_blades, [false; 5]);
        let still_lit = scene_state(&snapshot, flash(0.0));
        assert_eq!(still_lit.runes[0].activated_blades, [true; 5]);
    }
    #[test]
    fn hit_flash_window_follows_the_setting() {
        let mut snapshot = Field::new(&Default::default()).unwrap().snapshot();
        snapshot.time_ns = 300_000_000;
        snapshot
            .outposts
            .push(fixtures::outpost(Pose::at([4., 0., 0.])));
        snapshot.hits.push(rm_simulator_world::ArmorHit {
            time_ns: 100_000_000,
            projectile: 0,
            shooter: None,
            caliber: Caliber::Mm17,
            target: ArmorTarget::Outpost {
                outpost: 0,
                face: 0,
            },
            position_m: [0.; 3],
            local_offset_m: [0.; 2],
            normal_speed_m_s: 25.,
            detected: true,
            rejection: None,
            rune_outcome: None,
            damage: 0,
        });
        let hit = &snapshot.hits[0];
        let age_ns = snapshot.time_ns - hit.time_ns;
        assert!(age_ns > 50_000_000, "{age_ns}");
        let short = FlashSettings {
            hit_flash_ns: 50_000_000,
            rune_flash_hz: 0.0,
            rune_flashes: 3,
        };
        let long = FlashSettings {
            hit_flash_ns: age_ns + 1,
            rune_flash_hz: 0.0,
            rune_flashes: 3,
        };
        assert_eq!(
            scene_state(&snapshot, short).outposts[0].hit_flash,
            [false; 3]
        );
        assert!(scene_state(&snapshot, long).outposts[0].hit_flash[0]);
    }
    /// On a chassis rolled 20° the yaw stage stays bolted to the body and
    /// the pitch stage still points the barrel along the world aim.
    #[test]
    fn gimbal_stages_follow_the_body_and_point_the_barrel_at_the_aim() {
        let roll = DQuat::from_rotation_x(20_f64.to_radians());
        let body = PoseFlu {
            translation_m: [0.0, 0.0, 0.2],
            rotation_wxyz: wxyz(roll),
        };
        let aim_rotation = DQuat::from_rotation_z(0.7) * DQuat::from_rotation_y(-0.3);
        let aim = PoseFlu {
            translation_m: [0.0, 0.0, 0.4],
            rotation_wxyz: wxyz(aim_rotation),
        };
        let (stage, turret) = gimbal_poses(body, aim);
        let stage_up = rotate(stage.rotation_wxyz, [0.0, 0.0, 1.0]);
        let body_up = roll * DVec3::Z;
        assert!(close(stage_up, body_up.to_array()), "{stage_up:?}");
        let barrel = rotate(turret.rotation_wxyz, [1.0, 0.0, 0.0]);
        let wanted = aim_rotation * DVec3::X;
        assert!(close(barrel, wanted.to_array()), "{barrel:?} vs {wanted:?}");
        // The pitch axis is the yaw stage's left axis: the turret's left
        // stays in the stage's plane.
        let turret_left = rotate(turret.rotation_wxyz, [0.0, 1.0, 0.0]);
        let stage_left = rotate(stage.rotation_wxyz, [0.0, 1.0, 0.0]);
        assert!(
            close(turret_left, stage_left),
            "{turret_left:?} vs {stage_left:?}"
        );
    }
    #[test]
    fn wheel_visuals_put_the_axle_on_the_hub_radius_and_the_turret_above_the_body() {
        let config = ChassisConfig::default();
        let yaw = std::f64::consts::FRAC_PI_2;
        let mut field = Field::new(&rm_simulator_world::FieldConfig {
            chassis: vec![rm_simulator_world::ChassisPlacement {
                config: config.clone(),
                spawn: Pose::yawed([1.0, 0.0, config.rest_height_m()], yaw),
                team: Team::Blue,
                kind: rm_simulator_world::RobotKind::Infantry,
                performance: None,
            }],
            ..Default::default()
        })
        .unwrap();
        field.step(1).unwrap();
        let chassis = field.snapshot().chassis.remove(0);
        for (wheel, hub) in chassis.wheels.iter().zip(&config.wheel_hubs_m) {
            let pose = wheel_pose(&chassis, *hub, wheel);
            let axle = rotate(pose.rotation_wxyz, [0.0, 1.0, 0.0]);
            // The axle is the radial direction of the hub, turned with the body.
            let radius = (hub[0].powi(2) + hub[1].powi(2)).sqrt();
            let expected = [-hub[1] / radius, hub[0] / radius, 0.0];
            assert!(close(axle, expected), "{axle:?} vs {expected:?}");
        }
        let pivot = chassis.turret.translation_m;
        assert!(
            (pivot[2] - chassis.pose.translation_m[2] - config.turret_center_m[2]).abs() < 1e-6
        );
        assert!((pivot[0] - 1.0).abs() < 1e-3 && pivot[1].abs() < 1e-3);
        // Another player's turret is drawn pointing where the world holds
        // its aim, on a gimbal that turns with the body.
        let appearance = chassis_appearance(&chassis, &[false, true], None);
        assert_eq!(appearance.turret.translation_m, pivot);
        assert_eq!(appearance.team, TeamColor::Blue);
        assert_eq!(appearance.yaw_stage.translation_m, pivot);
        let barrel = rotate(appearance.turret.rotation_wxyz, [1.0, 0.0, 0.0]);
        let aim = rotate(chassis.turret.rotation_wxyz, [1.0, 0.0, 0.0]);
        assert!(close(barrel, aim), "{barrel:?} vs {aim:?}");
        let stage_up = rotate(appearance.yaw_stage.rotation_wxyz, [0.0, 0.0, 1.0]);
        let body_up = rotate(chassis.pose.rotation_wxyz, [0.0, 0.0, 1.0]);
        assert!(close(stage_up, body_up), "{stage_up:?} vs {body_up:?}");
        assert_eq!(appearance.armor.len(), 4);
        assert!(!appearance.armor[0].hit_flash && appearance.armor[1].hit_flash);
        assert!(!appearance.defeated);
        assert!((appearance.pivot_above_body_m - 0.15).abs() < 1e-6);
    }
}

#[cfg(test)]
mod extraction_tests {
    use super::*;
    use rm_simulator_world::{ChassisConfig, ChassisPlacement, Field, FieldConfig, RobotKind};

    fn chassis(config: ChassisConfig) -> ChassisSnapshot {
        let spawn = Pose::at([0., 0., config.rest_height_m()]);
        Field::new(&FieldConfig {
            chassis: vec![ChassisPlacement {
                config,
                spawn,
                team: Team::Red,
                kind: RobotKind::Infantry,
                performance: None,
            }],
            ..Default::default()
        })
        .unwrap()
        .snapshot()
        .chassis
        .remove(0)
    }

    #[test]
    fn roster_numbers_and_model_precedence_are_preserved() {
        use rm_simulator_render::chassis::RobotModel;
        let chassis = chassis(ChassisConfig::default());
        for (robot, pattern, model) in [
            (Robot::Hero, ArmorPattern::One, RobotModel::Hero),
            (Robot::Engineer, ArmorPattern::Two, RobotModel::Engineer),
            (Robot::Drone, ArmorPattern::Three, RobotModel::Drone),
            (Robot::Sentry, ArmorPattern::GuardSmall, RobotModel::Sentry),
            (Robot::Infantry3, ArmorPattern::Three, RobotModel::Omni),
            (Robot::Infantry4, ArmorPattern::Four, RobotModel::Omni),
            (Robot::Infantry5, ArmorPattern::Five, RobotModel::Omni),
        ] {
            let appearance = chassis_appearance(&chassis, &[], Some(robot));
            assert_eq!(appearance.armor_pattern, pattern);
            assert_eq!(appearance.model, model);
        }
        assert_eq!(
            chassis_appearance(&chassis, &[], None).armor_pattern,
            ArmorPattern::Three
        );
        let mut hero = chassis.clone();
        hero.config = ChassisConfig::hero();
        assert_eq!(armor_pattern(&hero, None), ArmorPattern::One);
        let mut folding = chassis;
        folding.config = ChassisConfig::folding_hero();
        assert_eq!(
            armor_pattern(&folding, Some(Robot::Engineer)),
            ArmorPattern::One
        );
        assert_eq!(
            chassis_appearance(&folding, &[], Some(Robot::Engineer)).model,
            RobotModel::FoldingHero
        );
    }

    #[test]
    fn retained_output_preserves_selected_bodies_and_reuses_rule_buffers() {
        let mut snapshot = Field::new(&FieldConfig::default()).unwrap().snapshot();
        snapshot
            .runes
            .push(crate::fixtures::rune(Pose::at([6., 0., 1.6])));
        snapshot
            .outposts
            .push(crate::fixtures::outpost(Pose::at([7., 0., 0.])));
        let flash = FlashSettings {
            hit_flash_ns: 50_000_000,
            rune_flash_hz: 2.,
            rune_flashes: 3,
        };
        let mut scene = scene_state_with_hits(&snapshot, flash, 123_000_000, &[]);
        scene.projectiles.push(ProjectileAppearance {
            position_m: [1., 2., 3.],
            radius_m: 0.01,
        });
        scene.chassis.push(chassis_appearance(
            &chassis(ChassisConfig::default()),
            &[],
            None,
        ));
        let selected = (scene.projectiles.clone(), scene.chassis.clone());
        let pointers = (scene.runes.as_ptr(), scene.outposts.as_ptr());
        for time_ns in [0, 123_000_000, 999_000_000] {
            scene_state_into(&snapshot, flash, time_ns, &[], &mut scene);
            let expected = scene_state_with_hits(&snapshot, flash, time_ns, &[]);
            assert_eq!(scene.runes, expected.runes);
            assert_eq!(scene.outposts, expected.outposts);
            assert_eq!((scene.runes.as_ptr(), scene.outposts.as_ptr()), pointers);
            assert_eq!(
                (&scene.projectiles, &scene.chassis),
                (&selected.0, &selected.1)
            );
        }
        snapshot.runes.clear();
        snapshot.outposts.clear();
        scene_state_into(&snapshot, flash, 0, &[], &mut scene);
        assert!(scene.runes.is_empty() && scene.outposts.is_empty());
    }

    #[test]
    fn flash_settings_round_trip_keeps_the_existing_field_schema() {
        let json = r#"{"hit_flash_ns":50000000,"rune_flash_hz":2.0,"rune_flashes":3}"#;
        let flash: FlashSettings = serde_json::from_str(json).unwrap();
        assert_eq!(flash.hit_flash_ns, 50_000_000);
        assert_eq!(serde_json::to_string(&flash).unwrap(), json);
    }
}
