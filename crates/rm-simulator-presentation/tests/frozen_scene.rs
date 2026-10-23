// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Frozen outputs captured from the original app scene functions on 2026-10-23.
//! Source: `b00cdcd4f398ebad33109420432c493ac7f1f0f5` app `src/scene.rs`.
//! SHA-256: `470b68254945c9c7f18aaf00ecb55f48cc299e7ed585802c9d0becf31e404a3d`.
//! The baseline copied the original pure functions plus native publication block;
//! roster selection and wall-clock/interpolation input were fixed by each fixture.
//! Covers the complete appearance, including chassis geometry, target IDs, flashes and HP.
//! Exact old/new parity was measured on aarch64-apple-darwin and x86_64-unknown-linux-gnu.
//! Unmeasured platforms retain all numerical/semantic and fresh/retained checks;
//! they do not assume either platform's exact floating-point byte representation.
use rm_simulator_render::sync::SceneState;
mod support;

// Exact hashes are independently measured from the original adapter on these
// targets. Floating-point fixture construction and Debug bytes are not promised
// bitwise portable. Other targets still run every fresh/retained equality and
// semantic assertion below, plus the numerical geometry/native tests.
fn platform_goldens() -> Option<[(&'static str, &'static str); 3]> {
    if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
        Some([
            (
                "75f461d024c99468610f092832ae05e4f5fdd15c438e4df9bbf5bf664997ecb8",
                "b72fcc75f57fd8cda901db2c9491a3f8c93e484650498a001a0345bf41b81774",
            ),
            (
                "fe4bd1e10bc42c13ef076cd242435dbf79fa686812a910598809a71ae101899f",
                "448879ed822fba73388393bae7dca8fcc6932b84d10aeb3216ae6bfff1d022cd",
            ),
            (
                "86c21a2379a66f32bf248421053d843c191e0c74097c0d1fedba7aca0fc92400",
                "77fc20ccd21b3b8b7464a60fa3753407554c742144d3ea7176586d608cbb6ddb",
            ),
        ])
    } else if cfg!(all(
        target_os = "linux",
        target_arch = "x86_64",
        target_env = "gnu"
    )) {
        Some([
            (
                "177097081a5a44f330f19f6b5dc7be63395d74bc6ae1b84755bdbfbe041eb55b",
                "84fb7199ee7b33655a83366c313688965ab0f7f905c2490ec45a3fbd0bd00c58",
            ),
            (
                "b4c743565ecc211d6d10f2f16b33f8516a659e4de24b50c25e4c7ef8ff06e30f",
                "08459d5179f848d2470287ec88601bfb35f27011b9777badfe6764380d7275ad",
            ),
            (
                "43c650d7e5ab9b1accda4a9ba9e56ee32756254e1182bc3d39614fcbbb82b7c1",
                "737405a9e6671effc48e1bb2f3830647596933e7531822719558e7843e80896e",
            ),
        ])
    } else {
        None
    }
}

#[test]
fn retained_output_matches_fresh_and_validated_original_goldens() {
    let goldens = platform_goldens();
    for (index, (native, flashing)) in [(false, false), (false, true), (true, true)]
        .into_iter()
        .enumerate()
    {
        let snapshot = support::fixture(native, flashing);
        let fixture_hash = support::hash(serde_json::to_vec(&snapshot).unwrap());
        if let Some(goldens) = goldens {
            assert_eq!(fixture_hash, goldens[index].0);
        }
        let mut fresh = SceneState::default();
        let mut fresh_targets = Vec::new();
        support::adapt(&snapshot, false, &mut fresh, &mut fresh_targets);
        let mut scene = SceneState::default();
        let mut targets = Vec::new();
        for _ in 0..2 {
            support::adapt(&snapshot, true, &mut scene, &mut targets);
            assert_eq!(scene, fresh);
            assert_eq!(targets, fresh_targets);
            if let Some(goldens) = goldens {
                assert_eq!(
                    support::hash(format!("{scene:?}{targets:?}")),
                    goldens[index].1
                );
            }
        }
        assert_eq!(scene.source.time_ns, snapshot.time_ns);
        assert_eq!(scene.source.tick, snapshot.tick);
        assert_eq!(scene.chassis.len(), snapshot.chassis.len());
        assert_eq!(scene.projectiles.len(), snapshot.projectiles.len());
        assert_eq!(scene.chassis[0].armor[1].hit_flash, flashing);
        assert_eq!(
            support::hash(serde_json::to_vec(&snapshot).unwrap()),
            fixture_hash
        );
        if native {
            assert_eq!(targets.len(), 18);
            assert!(targets.iter().any(|t| t.id == 514 && t.flash));
            assert_eq!(scene.base_open_fraction, [1., 0.]);
            assert!(
                scene
                    .chassis
                    .iter()
                    .any(|c| c.defeated && c.hp_fraction == 0.)
            );
            assert!(
                scene
                    .chassis
                    .iter()
                    .any(|c| c.hp_fraction > 0. && c.hp_fraction < 1.)
            );
            assert!(targets.iter().any(|t| t.disabled));
        }
    }
}

#[test]
fn native_time_is_explicit_pause_is_respected_and_targets_clear() {
    use rm_simulator_presentation::{FlashSettings, native_scene_into, scene_state_into};
    let snapshot = support::fixture(true, false);
    let flash = FlashSettings {
        hit_flash_ns: 50_000_000,
        rune_flash_hz: 2.,
        rune_flashes: 3,
    };
    let mut scene = SceneState::default();
    let mut targets = Vec::new();
    scene_state_into(&snapshot, flash, snapshot.time_ns, &[], &mut scene);
    native_scene_into(
        &snapshot,
        snapshot.time_ns,
        true,
        &[],
        &mut scene,
        &mut targets,
    );
    let paused_angle = scene.outposts[0].angle_rad;
    native_scene_into(
        &snapshot,
        snapshot.time_ns + 123_000_000,
        true,
        &[],
        &mut scene,
        &mut targets,
    );
    assert_eq!(scene.outposts[0].angle_rad, paused_angle);
    native_scene_into(
        &snapshot,
        snapshot.time_ns + 123_000_000,
        false,
        &[],
        &mut scene,
        &mut targets,
    );
    assert_ne!(scene.outposts[0].angle_rad, paused_angle);
    assert_eq!(scene.source.time_ns, snapshot.time_ns);
    let bare = support::fixture(false, false);
    scene_state_into(&bare, flash, bare.time_ns, &[], &mut scene);
    native_scene_into(&bare, bare.time_ns, false, &[], &mut scene, &mut targets);
    assert!(targets.is_empty());
}

#[test]
fn native_outpost_lights_follow_captured_owner_and_bases_keep_their_side() {
    use rm_simulator_presentation::native_scene_into;
    use rm_simulator_render::TeamColor;
    use rm_simulator_world::gameplay::{
        Team,
        rmuc_2027::{Control, ControlState, Territory},
    };
    let mut snapshot = support::fixture(true, false);
    let state = snapshot.rmuc_2027.as_mut().unwrap();
    let mut control = state.game.control.snapshot().clone();
    control.outposts[0].owner = Team::Blue;
    control.territories[Territory::RedOutpost.index()].control = ControlState::Full(Team::Blue);
    state.game.control = Control::restore(control).unwrap();
    let mut scene = SceneState::default();
    let mut targets = Vec::new();
    native_scene_into(
        &snapshot,
        snapshot.time_ns,
        true,
        &[],
        &mut scene,
        &mut targets,
    );
    assert_eq!(
        targets.iter().find(|t| t.id == 128).unwrap().team,
        TeamColor::Blue
    );
    assert_eq!(
        targets.iter().find(|t| t.id == 0).unwrap().team,
        TeamColor::Red
    );
}
