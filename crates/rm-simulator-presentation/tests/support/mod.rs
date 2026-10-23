// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Deterministic bounded fixtures shared by regression checks and the CPU probe.
use rm_simulator_world::{
    ArmorHit, ArmorTarget, Caliber, ChassisConfig, ChassisPlacement, Field, FieldConfig,
    FieldSnapshot, OutpostSnapshot, Pose, RobotKind, RuneKind, RuneSnapshot, RuneState, Team,
    rmuc_2027::{Config, Detector, RuneGeometry, Tower},
};

pub fn fixture(native: bool, flashing: bool) -> FieldSnapshot {
    let detector = |x, y| Detector {
        pose: Pose::at([x, y, 1.]),
        half_m: [0.045, 0.044],
    };
    let native_config = Config {
        bases: [vec![detector(7., 0.)], vec![detector(-7., 0.)]],
        outposts: [3., -3.].map(|x| Tower {
            pivot_m: [x, 0., 1.],
            axis: [0., 0., 1.],
            detectors: (0..3).map(|i| detector(x, i as f64)).collect(),
        }),
        zones: vec![],
        supply_zones: vec![],
        vantage_zones: vec![],
        terrain_zones: vec![],
        runes: [6., -6.]
            .map(|x| RuneGeometry {
                pivot_m: [x, 0., 1.6],
                axis: [1., 0., 0.],
                targets: std::array::from_fn(|i| Pose::at([x, i as f64 * 0.2, 2.])),
            })
            .to_vec(),
        rotor_direction: -1,
    };
    let mut snapshot = Field::new(&FieldConfig {
        rmuc_2027: native.then_some(native_config),
        chassis: (0..16)
            .map(|i| {
                let config = match i % 4 {
                    0 => ChassisConfig::hero(),
                    1 => ChassisConfig::folding_hero(),
                    _ => ChassisConfig::default(),
                };
                let spawn =
                    Pose::yawed([i as f64 - 8., 2., config.rest_height_m()], i as f64 * 0.1);
                ChassisPlacement {
                    config,
                    spawn,
                    team: Team::BOTH[i % 2],
                    kind: RobotKind::Infantry,
                    performance: None,
                }
            })
            .collect(),
        ..Default::default()
    })
    .unwrap()
    .snapshot();
    snapshot
        .projectiles
        .extend((0..32).map(|i| rm_simulator_world::ProjectileSnapshot {
            id: i,
            caliber: Caliber::Mm17,
            launched_ns: 0,
            position_m: [i as f64 * 0.1, 0., 1.],
            velocity_m_s: [25., 0., 0.],
            angular_velocity_rad_s: [0.; 3],
            shooter: None,
            first_contact_ns: None,
            dwell_since_ns: None,
        }));
    if !native {
        snapshot.runes.push(rune(Pose::at([6., 0., 1.6])));
        snapshot.outposts.push(outpost(Pose::at([3., 0., 0.])));
        snapshot.outposts.push(outpost(Pose::at([-3., 0., 0.])));
    }
    snapshot.time_ns = 300_000_000;
    snapshot.tick = 38;
    if let Some(state) = &mut snapshot.rmuc_2027 {
        state.game.phase = rm_simulator_world::rmuc_2027::match_rules::Phase::Running;
        state.game.base_open = [true, false];
        state.game.outpost_hp[1] = 0;
        state.game.base_hp[1] = 0;
        state.game.runes[0].targets = [true, false, true, false, true];
        state.game.runes[1].completed_blades = [true; 5];
        for r in &mut state.game.robots {
            r.hp = if r.id % 2 == 0 { r.max_hp / 2 } else { 0 };
        }
    }
    if flashing {
        for target in [
            ArmorTarget::Rune { rune: 0, blade: 2 },
            ArmorTarget::Outpost {
                outpost: 1,
                face: 1,
            },
            ArmorTarget::Base { base: 0, plate: 0 },
            ArmorTarget::Chassis {
                chassis: snapshot.chassis[0].id,
                plate: 1,
            },
        ] {
            snapshot.hits.push(ArmorHit {
                time_ns: 299_000_000,
                projectile: 0,
                shooter: None,
                caliber: Caliber::Mm17,
                target,
                position_m: [0.; 3],
                local_offset_m: [0.; 2],
                normal_speed_m_s: 25.,
                detected: true,
                rejection: None,
                rune_outcome: None,
                damage: 0,
            });
        }
    }
    snapshot
}

pub fn hash(bytes: impl AsRef<[u8]>) -> String {
    use sha2::{Digest, Sha256};
    format!("{:x}", Sha256::digest(bytes.as_ref()))
}

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

use rm_simulator_presentation as adapter;
use rm_simulator_render::{
    sync::{ProjectileAppearance, SceneState},
    targets::TargetAppearance,
};
pub fn adapt(
    snapshot: &FieldSnapshot,
    retained: bool,
    scene: &mut SceneState,
    targets: &mut Vec<TargetAppearance>,
) {
    let flash = adapter::FlashSettings {
        hit_flash_ns: 50_000_000,
        rune_flash_hz: 2.,
        rune_flashes: 3,
    };
    let hits: Vec<_> = adapter::flashing(snapshot, flash.hit_flash_ns).collect();
    let presentation_ns = snapshot.time_ns + 123_000_000;
    if retained {
        adapter::scene_state_into(snapshot, flash, presentation_ns, &hits, scene);
    } else {
        *scene = adapter::scene_state_with_hits(snapshot, flash, presentation_ns, &hits);
    }
    scene.projectiles.clear();
    scene
        .projectiles
        .extend(snapshot.projectiles.iter().map(|p| ProjectileAppearance {
            position_m: p.position_m,
            radius_m: (p.caliber.diameter_m() / 2.) as f32,
        }));
    scene.chassis.clear();
    scene.chassis.extend(snapshot.chassis.iter().map(|c| {
        let armor_flash: [bool; 4] = std::array::from_fn(|plate| {
            hits.iter().any(|h| {
                h.target
                    == ArmorTarget::Chassis {
                        chassis: c.id,
                        plate: plate as u32,
                    }
            })
        });
        adapter::chassis_appearance(c, &armor_flash, Some(adapter::Robot::Engineer))
    }));
    adapter::native_scene_into(snapshot, presentation_ns, false, &hits, scene, targets);
}
