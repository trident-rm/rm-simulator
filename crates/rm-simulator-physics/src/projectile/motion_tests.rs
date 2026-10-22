// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
use super::*;
use crate::motion::{MechanismState, MotionChannel, MotionFrame, MotionId, MotionUpdate};

fn id(instance: u64) -> MotionId {
    MotionId {
        instance,
        channel: 0,
    }
}
fn wall() -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
    (
        vec![
            [2., -0.5, 0.5],
            [2., 0.5, 0.5],
            [2., 0.5, 1.5],
            [2., -0.5, 1.5],
        ],
        vec![[0, 1, 2], [0, 2, 3]],
    )
}
fn update(instance: u64, coordinate: f64) -> MotionUpdate {
    MotionUpdate {
        id: id(instance),
        coordinate,
    }
}

#[test]
fn explicit_rotor_capture_preserves_identity_rest_frame_and_shared_channel_shapes() {
    let mut world = WorldPhysics::new(&[], 0.);
    let frame = MotionFrame::Rotation {
        pivot_m: [1., 0., 1.],
        axis: [0., 0., 1.],
    };
    for _ in 0..2 {
        let (vertices, triangles) = wall();
        world
            .add_motion_mesh(vertices, triangles, id(40), frame)
            .unwrap();
    }
    let rest = world.static_geometry_snapshot(true);
    let angle = std::f64::consts::FRAC_PI_2;
    world.sync_motions(&[update(40, angle)]).unwrap();
    let moved = world.static_geometry_snapshot(true);
    assert_eq!(
        moved.motion_channels().collect::<Vec<_>>(),
        vec![MotionChannel::Instance(id(40)); 2]
    );
    let hit = moved
        .cast_ray([1., -2., 1.], [0., 1., 0.], 5.)
        .unwrap()
        .unwrap();
    assert!((hit.distance_m - 3.).abs() < 1e-9);
    assert!(
        moved
            .cast_ray([0., 0., 1.], [1., 0., 0.], 5.)
            .unwrap()
            .is_none()
    );
    let mut restored = WorldPhysics::new(&[], 0.);
    restored.try_insert_geometry(&moved).unwrap();
    assert_eq!(
        restored.static_geometry_snapshot(true).into_triangles(),
        moved.clone().into_triangles()
    );
    restored.sync_motions(&[update(40, angle)]).unwrap();
    assert_eq!(
        restored.static_geometry_snapshot(true).into_triangles(),
        moved.clone().into_triangles()
    );
    restored.sync_motions(&[update(40, 0.)]).unwrap();
    assert_eq!(
        restored.static_geometry_snapshot(true).into_triangles(),
        rest.clone().into_triangles()
    );
    assert_eq!(
        moved
            .at_motions(&[update(40, 0.)])
            .unwrap()
            .into_triangles(),
        rest.into_triangles()
    );
    restored.sync_mechanisms(&MechanismState {
        outpost_angle_rad: [1.2; 2],
        ..Default::default()
    });
    assert_eq!(
        restored.static_geometry_snapshot(true).into_triangles(),
        moved
            .at_motions(&[update(40, 0.)])
            .unwrap()
            .into_triangles()
    );
    let before = restored.static_geometry_snapshot(true).into_triangles();
    assert!(restored.try_insert_geometry(&moved).is_err());
    assert_eq!(
        before,
        restored.static_geometry_snapshot(true).into_triangles()
    );
    assert_eq!(
        restored
            .static_geometry_snapshot(true)
            .motion_channels()
            .count(),
        2
    );
    // Another declared namespace can be installed without affecting the first.
    let mut other = WorldPhysics::new(&[], 0.);
    let (vertices, triangles) = wall();
    other
        .add_motion_mesh(vertices, triangles, id(41), frame)
        .unwrap();
    restored
        .try_insert_geometry(&other.static_geometry_snapshot(true))
        .unwrap();
    assert_eq!(
        restored
            .static_geometry_snapshot(true)
            .motion_channels()
            .count(),
        3
    );
}

#[test]
fn partial_motion_updates_reject_malformed_batches_without_any_movement() {
    let mut world = WorldPhysics::new(&[], 0.);
    for instance in [1, 2] {
        let (vertices, triangles) = wall();
        world
            .add_motion_mesh(
                vertices,
                triangles,
                id(instance),
                MotionFrame::Translation {
                    offsets_m: [[0.; 3], [5., 0., 0.]],
                },
            )
            .unwrap();
    }
    world.sync_motions(&[update(1, 0.5)]).unwrap();
    let initial = world.static_geometry_snapshot(true);
    for bad in [
        update(1, 0.1),
        update(99, 0.1),
        update(2, f64::NAN),
        update(2, f64::INFINITY),
        update(2, 1.1),
    ] {
        let updates = [update(1, 0.1), bad];
        assert!(world.sync_motions(&updates).is_err());
        assert!(initial.at_motions(&updates).is_err());
        assert_eq!(
            world.static_geometry_snapshot(true).into_triangles(),
            initial.clone().into_triangles()
        );
    }
    // Only the first wall moved; the second still occludes at its initial x=2.
    let hit = initial
        .cast_ray([0., 0., 1.], [1., 0., 0.], 10.)
        .unwrap()
        .unwrap();
    assert!((hit.distance_m - 2.).abs() < 1e-9);
    world.sync_motions(&[update(2, 1.)]).unwrap();
    let hit = world
        .static_geometry_snapshot(true)
        .cast_ray([0., 0., 1.], [1., 0., 0.], 10.)
        .unwrap()
        .unwrap();
    assert!((hit.distance_m - 4.5).abs() < 1e-9);
}

#[test]
fn geometry_interpolation_rejects_motion_identity_swaps() {
    let mut world = WorldPhysics::new(&[], 0.);
    let (vertices, triangles) = wall();
    world
        .add_motion_mesh(
            vertices,
            triangles,
            id(1),
            MotionFrame::Rotation {
                pivot_m: [0.; 3],
                axis: [0., 0., 1.],
            },
        )
        .unwrap();
    let before = world.static_geometry_snapshot(true);
    let mut after = before.clone();
    after.mechanisms[0].1 = MotionChannel::Instance(id(2));
    assert!(before.interpolate(&after, 0.5).is_err());
}
