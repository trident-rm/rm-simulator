// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! CPU-only sparse publication versus guarded direct ECS fixture, without a GPU.
use bevy::prelude::*;
use rm_simulator_render::{PoseFlu, apply_pose};
use std::{hint::black_box, time::Instant};

fn main() {
    for count in [4, 128, 1024] {
        for changed in [false, true] {
            pipeline_fixture(count, changed, false);
            pipeline_fixture(count, changed, true);
        }
    }
}

#[derive(Resource, Default)]
struct DirectPoses(Vec<(Entity, PoseFlu)>);

fn guarded_direct_sync(poses: Res<DirectPoses>, mut transforms: Query<&mut Transform>) {
    for &(entity, pose) in &poses.0 {
        if let Ok(mut current) = transforms.get_mut(entity) {
            let mut next = *current;
            apply_pose(&mut next, pose);
            if *current != next {
                *current = next;
            }
        }
    }
}

fn pipeline_fixture(count: usize, changed: bool, sparse: bool) {
    use rm_simulator_render::generic::{VisualId, VisualPlugin, Visuals};
    const FRAMES: usize = 1024;
    let mut app = App::new();
    if sparse {
        app.add_plugins(VisualPlugin);
        app.world_mut()
            .resource_scope(|world, mut visuals: Mut<Visuals>| {
                visuals.reserve(count, count);
                for id in 0..count {
                    visuals.spawn_mesh(
                        &mut world.commands(),
                        VisualId(id as u64),
                        Handle::default(),
                        Handle::default(),
                        PoseFlu::default(),
                        Vec3::ONE,
                    );
                }
            });
        app.world_mut().flush();
    } else {
        let mut poses = DirectPoses::default();
        for _ in 0..count {
            poses.0.push((
                app.world_mut()
                    .spawn((
                        Mesh3d(Handle::default()),
                        MeshMaterial3d::<StandardMaterial>(Handle::default()),
                        Transform::default(),
                        Visibility::Inherited,
                    ))
                    .id(),
                PoseFlu::default(),
            ));
        }
        app.insert_resource(poses)
            .add_systems(PostUpdate, guarded_direct_sync);
    }
    app.update();
    for trial in 0..5 {
        let start = Instant::now();
        for frame in 0..FRAMES {
            let pose = PoseFlu {
                translation_m: [
                    if changed {
                        (frame % 128) as f64 * 0.001
                    } else {
                        0.0
                    },
                    0.0,
                    0.0,
                ],
                ..default()
            };
            if sparse {
                let mut visuals = app.world_mut().resource_mut::<Visuals>();
                for id in 0..count {
                    visuals.set_pose(VisualId(id as u64), black_box(pose));
                }
            } else {
                for (_, current) in &mut app.world_mut().resource_mut::<DirectPoses>().0 {
                    *current = black_box(pose);
                }
            }
            app.update();
        }
        let elapsed = start.elapsed();
        let capacity = if sparse {
            app.world().resource::<Visuals>().update_capacity()
        } else {
            0
        };
        println!(
            "pipeline {}: visuals={count}, changed={changed}, frames={FRAMES}, trial={trial}, ns/frame={}, retained_updates={capacity}",
            if sparse { "sparse" } else { "direct" },
            elapsed.as_nanos() / FRAMES as u128
        );
    }
}
