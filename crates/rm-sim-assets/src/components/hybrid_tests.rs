// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Verify native-only motion ownership and exact primitive placement.
use super::*;
use crate::collision_shapes::{CollisionShapes, CylinderPart};
use std::path::PathBuf;

fn cylinder(name: &str) -> CylinderPart {
    CylinderPart {
        source_node_index: 1,
        source_node_name: name.into(),
        ancestor_nodes_root_first: vec![1],
        component_id: Some("test/arm".into()),
        pose: Pose::at([2., 0., 1.]),
        radius_m: 0.2,
        half_height_m: 0.5,
    }
}
fn source_mesh() -> PathBuf {
    use std::sync::atomic::{AtomicUsize, Ordering};
    static NEXT: AtomicUsize = AtomicUsize::new(0);
    let path = std::env::temp_dir().join(format!(
        "native-component-{}-{}.glb",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    let mut document=serde_json::to_vec(&serde_json::json!({"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0,1]}],"nodes":[{"name":"retained","mesh":0},{"name":"moving"}],"meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1}]}],"buffers":[{"byteLength":48}],"bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":36},{"buffer":0,"byteOffset":36,"byteLength":12}],"accessors":[{"bufferView":0,"componentType":5126,"type":"VEC3","count":3},{"bufferView":1,"componentType":5125,"type":"SCALAR","count":3}]})).unwrap();
    while !document.len().is_multiple_of(4) {
        document.push(b' ');
    }
    let mut bin = Vec::new();
    for v in [10f32, 10., 0., 11., 10., 0., 10., 11., 0.] {
        bin.extend(v.to_le_bytes());
    }
    for i in [0u32, 1, 2] {
        bin.extend(i.to_le_bytes());
    }
    let mut bytes = Vec::new();
    for v in [
        0x46546c67u32,
        2,
        (12 + 8 + document.len() + 8 + bin.len()) as u32,
        document.len() as u32,
        0x4e4f534a,
    ] {
        bytes.extend(v.to_le_bytes());
    }
    bytes.extend(document);
    bytes.extend((bin.len() as u32).to_le_bytes());
    bytes.extend(0x004e4942u32.to_le_bytes());
    bytes.extend(bin);
    std::fs::write(&path, bytes).unwrap();
    path
}
fn cad(path: PathBuf, part: CylinderPart) -> CadAssets {
    let asset = CadAsset {
        file: "unused.glb".into(),
        semantics: None,
        placements: vec![],
        collision: Some(path.clone()),
        collision_shapes: Some(CollisionShapes {
            query_mesh: None,
            remainder_bytes: std::fs::read(&path).unwrap().into(),
            remainder_file: path,
            parts: vec![part],
        }),
        source_tessellated_collision: true,
        bake_motion_endpoints: false,
    };
    CadAssets {
        terrain_collision: None,
        rmuc_2027: None,
        season: rm_sim_gameplay::season::Season::Rm2027,
        rest_pose_only: false,
        root: PathBuf::new(),
        floor_top_cad_m: 0.,
        collision_solids: false,
        floor: asset.clone(),
        arena_static: asset.clone(),
        rune: asset.clone(),
        outpost: asset.clone(),
        base: asset.clone(),
        tech_core: asset,
        static_assets: vec![],
    }
}
#[test]
fn native_only_bound_body_keeps_motion_and_places_once() {
    let path = source_mesh();
    let cad = cad(path.clone(), cylinder("moving"));
    let motions = [Motion {
        id: "slide".into(),
        bodies: vec!["moving".into()],
        pivot_m: [0.; 3],
        axis: [1., 0., 0.],
        limits: Some([0., 1.]),
    }];
    let model = load(
        &cad,
        &cad.tech_core,
        ComponentRole::Equipment,
        Some(&motions),
    )
    .unwrap();
    assert!(model.native_shapes.is_empty());
    assert_eq!(model.moving[0].native_shapes.len(), 1);
    assert!(model.moving[0].mesh.triangles.is_empty());
    let root = math::gltf_root_pose([3., -2., 1.], math::quat_axis_angle([0., 0., 1.], 0.7));
    let placed = model.placed(&root);
    let rest = placed.moving[0].native_shapes[0].0;
    let expected = math::gltf_point(&root, [2., 0., 1.]);
    assert!(
        rest.translation_m
            .iter()
            .zip(expected)
            .all(|(a, b)| (a - b).abs() < 1e-12)
    );
    let mut physics = WorldPhysics::new(&[], 0.);
    placed.install(&mut physics).unwrap();
    let state = rm_sim_physics::motion::MechanismState {
        base_open_fraction: [0.5; 2],
        ..Default::default()
    };
    physics.sync_mechanisms(&state);
    let offset = math::rotate(math::gltf_rotation(&root), [0.5, 0., 0.]);
    let center = math::add(expected, offset);
    let radial = math::rotate(rest.rotation_wxyz, [1., 0., 0.]);
    let origin = math::add(center, radial.map(|v| v * 0.3));
    let hit = physics
        .static_geometry_snapshot(true)
        .cast_ray(origin, radial.map(|v| -v), 0.2)
        .unwrap()
        .unwrap();
    assert!((hit.distance_m - 0.1).abs() < 1e-9);
    assert!(
        physics
            .static_geometry_snapshot(false)
            .cast_ray(origin, radial.map(|v| -v), 0.2)
            .unwrap()
            .is_none()
    );
    std::fs::remove_file(path).unwrap();
}
#[test]
fn hybrid_component_uses_retained_snapshot_after_remainder_is_deleted() {
    let path = source_mesh();
    let cad = cad(path.clone(), cylinder("moving"));
    let first = load(&cad, &cad.tech_core, ComponentRole::Equipment, None).unwrap();
    std::fs::remove_file(&path).unwrap();
    let second = load(&cad, &cad.tech_core, ComponentRole::Equipment, None).unwrap();
    assert_eq!(first.fixed, second.fixed);
    assert_eq!(first.native_shapes, second.native_shapes);
    assert_eq!(second.fixed.triangles.len(), 1);
    assert_eq!(second.native_shapes.len(), 1);
}
#[test]
fn ambiguous_native_motion_owner_fails_instead_of_duplicating_collision() {
    let path = source_mesh();
    let cad = cad(path.clone(), cylinder("moving"));
    let motion = Motion {
        id: "slide".into(),
        bodies: vec!["moving".into()],
        pivot_m: [0.; 3],
        axis: [1., 0., 0.],
        limits: Some([0., 1.]),
    };
    assert!(
        load(
            &cad,
            &cad.tech_core,
            ComponentRole::Equipment,
            Some(&[motion.clone(), motion])
        )
        .is_err()
    );
    std::fs::remove_file(path).unwrap();
}

#[test]
fn native_only_semantic_joint_is_not_skipped_with_empty_mesh() {
    let path = source_mesh();
    let mut cad = cad(path.clone(), cylinder("moving"));
    let file:crate::semantics::FileSemantics=serde_json::from_value(serde_json::json!({"file":"unused.glb","sha256":"unused","nodes":[],"joints":[{"id":"base.shield.0.slide","type":"prismatic","parent":"parent","children":["child"],"origin_m":[0,0,0],"axis":[1,0,0],"limits":[0,1],"origin_node":1,"motion_node":1,"verification":null}]})).unwrap();
    cad.tech_core.semantics = Some(crate::semantics::AssetSemantics {
        files: std::collections::BTreeMap::from([("collision".into(), file)]),
    });
    let model = load(&cad, &cad.tech_core, ComponentRole::Equipment, None).unwrap();
    assert_eq!(model.moving.len(), 1);
    assert!(model.moving[0].mesh.triangles.is_empty());
    assert_eq!(model.moving[0].native_shapes.len(), 1);
    assert!(model.native_shapes.is_empty());
    let mut world = WorldPhysics::new(&[], 0.);
    model
        .placed(&math::gltf_root_pose([0.; 3], [1., 0., 0., 0.]))
        .install(&mut world)
        .unwrap();
    assert!(world.has_mechanisms());
    std::fs::remove_file(path).unwrap();
}

#[test]
fn fixed_query_mesh_is_placed_and_installed_but_moving_owners_are_rejected() {
    let path = source_mesh();
    let mut cad = cad(path.clone(), cylinder("moving"));
    let query = CollisionMesh {
        vertices_m: vec![[1., -3., -2.], [1., 3., -2.], [1., 0., 4.]],
        triangles: vec![[0, 1, 2]],
    };
    cad.tech_core.collision_shapes.as_mut().unwrap().query_mesh = Some(std::sync::Arc::new(query));
    let model = load(&cad, &cad.tech_core, ComponentRole::Equipment, Some(&[])).unwrap();
    let placed = model.placed(&math::gltf_root_pose([0.; 3], [1., 0., 0., 0.]));
    assert!(placed.query_mesh.is_some());
    let mut world = WorldPhysics::new(&[], 0.);
    world.remove_catch_floor();
    placed.install(&mut world).unwrap();
    let geometry = world.static_geometry_snapshot(true);
    assert!(
        (geometry
            .cast_ray([0., 0., 1.], [1., 0., 0.], 5.)
            .unwrap()
            .unwrap()
            .distance_m
            - 1.)
            .abs()
            < 1e-10
    );
    let physical = geometry
        .for_collision_purpose(rm_sim_physics::collision_profiles::CollisionPurpose::RigidBody);
    assert!(
        (physical
            .cast_ray([0., 0., 1.], [1., 0., 0.], 5.)
            .unwrap()
            .unwrap()
            .distance_m
            - 1.8)
            .abs()
            < 1e-8
    );
    let moving = [Motion {
        id: "slide".into(),
        bodies: vec!["moving".into()],
        pivot_m: [0.; 3],
        axis: [1., 0., 0.],
        limits: Some([0., 1.]),
    }];
    assert!(
        load(
            &cad,
            &cad.tech_core,
            ComponentRole::Equipment,
            Some(&moving)
        )
        .is_err()
    );
    std::fs::remove_file(path).unwrap();
}
