// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Export deployed collision source descriptions for an isolated precision audit.
use anyhow::{Result, ensure};
use rm_sim_assets::{
    arena, cad_assets,
    collision_mesh::CollisionMesh,
    math::{quat_multiply, rotate},
};
use rm_sim_physics::{Caliber, Pose, model::ShapeDescription, motion::MotionFrame};
use serde_json::{Value, json};

fn transform(pose: Pose, p: [f64; 3]) -> [f64; 3] {
    let q = rotate(pose.rotation_wxyz, p);
    std::array::from_fn(|i| q[i] + pose.translation_m[i])
}
fn mesh(owner: &str, m: &CollisionMesh, pose: Pose, projectile: bool) -> Value {
    let mut used: Vec<_> = m.triangles.iter().flatten().copied().collect();
    used.sort_unstable();
    used.dedup();
    let mut remap = vec![0u32; m.vertices_m.len()];
    for (i, index) in used.iter().enumerate() {
        remap[*index as usize] = i as u32;
    }
    json!({"kind":"mesh","owner":owner,"projectile":projectile,
        "vertices":used.iter().map(|i|transform(pose,m.vertices_m[*i as usize])).collect::<Vec<_>>(),
        "triangles":m.triangles.iter().map(|t|t.map(|i|remap[i as usize])).collect::<Vec<_>>()})
}
fn compose(a: Pose, b: Pose) -> Pose {
    Pose {
        translation_m: transform(a, b.translation_m),
        rotation_wxyz: quat_multiply(a.rotation_wxyz, b.rotation_wxyz),
    }
}
fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    ensure!(
        args.len() == 3,
        "usage: export_precision_field FIELD OUTPUT_JSON"
    );
    let root = std::path::Path::new(&args[1]);
    let cad = cad_assets::load(root)?;
    let terrain = arena::load_terrain(&cad)?;
    ensure!(
        terrain.native_shapes.is_empty(),
        "exporter must explicitly handle every native primitive"
    );
    let mut common = vec![
        mesh(
            "ground-remainder",
            terrain.collision_ground(),
            Pose::default(),
            true,
        ),
        mesh(
            "fixed-rune-outpost",
            &terrain.fixtures,
            Pose::default(),
            true,
        ),
        mesh(
            "robot-only-boundary",
            &terrain.boundary,
            Pose::default(),
            false,
        ),
    ];
    for (i, m) in terrain.equipment.iter().enumerate() {
        common.push(mesh(&format!("equipment-{i}"), m, Pose::default(), true));
    }
    if let Some(package) = &terrain.native_terrain {
        for part in &package.parts {
            for (occurrence, placement) in part.placements.iter().enumerate() {
                for (i, collider) in part.model.source().colliders().iter().enumerate() {
                    let owner = format!("{}/{occurrence}/{i}", part.id);
                    let pose = compose(*placement, collider.pose());
                    match collider.shape() {
                        ShapeDescription::TriangleMesh(m) => common.push(mesh(
                            &owner,
                            &CollisionMesh {
                                vertices_m: m.vertices_m().to_vec(),
                                triangles: m.triangles().to_vec(),
                            },
                            pose,
                            true,
                        )),
                        ShapeDescription::HeightField(g) => {
                            let [w, x, y, z] = pose.rotation_wxyz;
                            common.push(json!({"kind":"grid","owner":owner,"projectile":true,
                                "heights":g.heights_m(),"size":[g.size_x_m(),g.size_z_m()],
                                "translation":pose.translation_m,"rotation_xyzw":[x,y,z,w]}));
                        }
                        _ => anyhow::bail!("unsupported deployed shape in precision export"),
                    }
                }
            }
        }
    }
    let mut scenes = Vec::new();
    for (name, fraction, angle) in [
        ("rest", 0., 0.),
        ("half-open-45deg", 0.5, std::f64::consts::FRAC_PI_4),
        ("open-90deg", 1., std::f64::consts::FRAC_PI_2),
    ] {
        let mut shapes = Vec::new();
        for (i, part) in terrain.mechanisms.iter().enumerate() {
            ensure!(
                part.native_shapes.is_empty(),
                "moving native export not implemented"
            );
            let pose = part.motion.pose(match part.motion {
                MotionFrame::Translation { .. } => fraction,
                MotionFrame::Rotation { .. } => angle,
            });
            shapes.push(mesh(
                &format!("mechanism-{i}-{:?}-{:?}", part.mechanism, part.team),
                &part.mesh,
                pose,
                true,
            ));
        }
        scenes.push(json!({"name":name,"shapes":shapes}));
    }
    let output = std::fs::File::create(&args[2])?;
    serde_json::to_writer(
        output,
        &json!({"schema":"precision-field-audit-1","common_shapes":common,"scenes":scenes,
        "projectile_bounds_m":terrain.projectile_bounds_m,
        "calibers":([Caliber::Mm17,Caliber::Mm42].map(|c|json!({"radius_m":c.diameter_m()/2.,"mass_kg":c.mass_kg(),"speed_m_s":c.launch_speed_limit_m_s()}))),
        "scope":"Actual deployed terrain/scenery, three frozen mechanism poses. Robot-only boundary excluded from projectile queries. No dynamic robots, loose Units or scoring policy bodies.","source_field":root.join("manifest.json").file_name().unwrap().to_string_lossy()}),
    )?;
    Ok(())
}
