// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
use super::*;

fn face(target: ArmorTarget, x: f64, yaw_rad: f64) -> TargetFace {
    TargetFace {
        target,
        pose: Pose::yawed([x, 0., 1.], yaw_rad),
    }
}

#[test]
fn contact_storage_preserves_order_latches_and_recontact() {
    fn fixture() -> WorldPhysics {
        let target = face(
            ArmorTarget::Outpost {
                outpost: 0,
                face: 0,
            },
            1.,
            std::f64::consts::PI,
        );
        let mut world = WorldPhysics::new(&[target], 0.);
        world
            .set_projectile_policy(ProjectilePolicy::default().without_retirement())
            .unwrap();
        world.add_static_box(Pose::at([0.7, 0., 0.3]), [0.1, 4., 0.6]);
        for y in [-1., 1.] {
            let config = ChassisConfig::default();
            let height = config.rest_height_m();
            let id = world
                .add_chassis(Team::Red, config, Pose::at([0., y, height]))
                .unwrap();
            let armor = world.chassis[id as usize].armor_pose(&world.world, 1);
            world.add_static_box(Pose::at(armor.translation.to_array()), [0.04; 3]);
            world
                .command_chassis(
                    id,
                    ChassisCommand {
                        forward_m_s: 2.,
                        ..Default::default()
                    },
                )
                .unwrap();
        }
        world
    }
    let mut retained = fixture();
    let mut fresh = fixture();
    let frames = TargetFrames::new(vec![face(
        ArmorTarget::Outpost {
            outpost: 0,
            face: 0,
        },
        1.,
        std::f64::consts::PI,
    )]);
    let mut hits = Vec::new();
    let mut armor_touches = 0;
    let mut armor_recontacts = 0;
    let mut separated = false;
    for tick in 0..192 {
        for world in [&mut retained, &mut fresh] {
            if tick == 64 || tick == 96 {
                for id in 0..2 {
                    let x = if tick == 64 { -2. } else { 0. };
                    let height = ChassisConfig::default().rest_height_m();
                    world
                        .place_chassis(id, Pose::at([x, id as f64 * 2. - 1., height]))
                        .unwrap();
                }
            }
            if tick % 32 == 0 {
                for y in [-0.03, 0.03] {
                    world
                        .fire(
                            tick * tick_ns(),
                            Pose::at([0.8, y, 1.]),
                            Shot::at_limit(Caliber::Mm17),
                            None,
                        )
                        .unwrap();
                }
            }
        }
        // Exercise both empty scratch and stale handles from unrelated owners.
        // The reference still retains the preceding latches, as fresh allocation
        // in the original collectors did.
        fresh.contact_touching = Vec::new();
        retained
            .contact_touching
            .extend(retained.projectiles.iter().map(|p| p.collider));
        let expected = fresh.step(tick * tick_ns(), &frames).unwrap();
        let actual = retained.step(tick * tick_ns(), &frames).unwrap();
        assert_eq!(actual, expected, "contact order at tick {tick}");
        hits.extend(actual);
        assert_eq!(retained.snapshot(), fresh.snapshot());
        assert_eq!(retained.chassis_snapshots(), fresh.chassis_snapshots());
        assert_eq!(
            retained.take_armor_collisions(),
            fresh.take_armor_collisions()
        );
        let projectiles = &retained.projectiles;
        for (a, b) in retained.chassis.iter().zip(&fresh.chassis) {
            assert_eq!(a.armor_touching, b.armor_touching);
            let expected: Vec<_> = a
                .armor_colliders()
                .iter()
                .flat_map(|&armor| {
                    retained
                        .world
                        .contact_pairs_with(armor)
                        .filter_map(move |pair| {
                            let other = if pair.collider1 == armor {
                                pair.collider2
                            } else {
                                pair.collider1
                            };
                            (pair.has_any_active_contact()
                                && !projectiles.iter().any(|p| p.collider == other))
                            .then_some(other)
                        })
                })
                .collect();
            assert_eq!(a.armor_touching, expected);
            armor_touches += expected.len();
            if tick >= 96 {
                armor_recontacts += expected.len();
            }
            if tick == 65 && expected.is_empty() {
                separated = true;
            }
        }
        for (a, b) in retained.projectiles.iter().zip(&fresh.projectiles) {
            assert_eq!(a.touching, b.touching);
        }
    }
    assert!(armor_touches > 0);
    assert!(armor_recontacts > 0);
    assert!(separated);
    assert!(
        hits.len() >= 4,
        "multiple ordered hits and later contacts: {hits:?}"
    );
    assert!(hits.windows(2).any(|p| p[0].projectile < p[1].projectile));

    // Repeated collection over unchanged pairs must recycle the same pool.
    let buffers = |world: &WorldPhysics| {
        let mut buffers: Vec<_> = std::iter::once(&world.contact_touching)
            .chain(world.projectiles.iter().map(|p| &p.touching))
            .chain(world.chassis.iter().map(|c| &c.armor_touching))
            .filter(|v| v.capacity() > 0)
            .map(|v| (v.as_ptr(), v.capacity()))
            .collect();
        buffers.sort_unstable();
        buffers
    };
    let mut output = Vec::with_capacity(64);
    for _ in 0..2 {
        retained.capture_contacts(
            &frames,
            0.,
            1.,
            tick_ns() as f64 * 1e-9,
            192 * tick_ns(),
            &mut output,
        );
        retained.capture_armor_collisions();
    }
    let before = buffers(&retained);
    assert!(!before.is_empty());
    for _ in 0..16 {
        retained.capture_contacts(
            &frames,
            0.,
            1.,
            tick_ns() as f64 * 1e-9,
            192 * tick_ns(),
            &mut output,
        );
        retained.capture_armor_collisions();
        assert_eq!(buffers(&retained), before);
    }
    assert!(output.is_empty(), "persistent contacts stay latched");
}
