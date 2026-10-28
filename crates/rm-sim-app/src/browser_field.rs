// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Passive browser CAD instances from verified visual and source-binding bytes.
use bevy::prelude::*;
use rm_sim_assets::{cad_assets::visual::VisualPackage, rmuc_2027::Bindings};
use rm_sim_render::{
    PoseFlu, TeamColor, apply_pose,
    cad::{CadInstance, CadJoint, CadRole, CadTargetLight, JointDriver},
};

/// Resolve the same source joints and exact optical nodes used by native RMUC.
pub(super) fn instances(package: &VisualPackage, bindings: Option<&Bindings>) -> Vec<CadInstance> {
    package
        .assets
        .iter()
        .flat_map(|asset| {
            asset.placements.iter().map(move |pose| {
                let mut transform = Transform::default();
                apply_pose(
                    &mut transform,
                    PoseFlu {
                        translation_m: pose.translation_m,
                        rotation_wxyz: pose.rotation_wxyz,
                    },
                );
                // The native adapter assigns source-side equipment using FLU x, with Red +x.
                let side = usize::from(pose.translation_m[0] < 0.);
                let mut role = CadRole::Static { base_color: None };
                let mut team = None;
                if let Some(bindings) = bindings {
                    let motions: &[rm_sim_assets::rmuc_2027::Motion] = match asset.name.as_str() {
                        "rune" => &bindings.rune_motion,
                        "outpost" => &bindings.outpost_motion,
                        "base" => &bindings.base_motion,
                        _ => &[][..],
                    };
                    let joints: Vec<_> = motions
                        .iter()
                        .enumerate()
                        .map(|(index, motion)| CadJoint {
                            id: motion.id.clone(),
                            axis: Vec3::from_array(motion.axis.map(|v| v as f32)),
                            prismatic: motion.limits.is_some(),
                            limits: motion.limits.map(|v| v.map(|v| v as f32)),
                            driver: Some(if asset.name == "rune" {
                                JointDriver::Rune {
                                    index: index as u32,
                                    sign: 1.,
                                }
                            } else if motion.limits.is_some() {
                                JointDriver::BaseOpening { team: side }
                            } else {
                                JointDriver::Outpost { index: side as u32 }
                            }),
                        })
                        .collect();
                    let target_lights = if asset.name == "rune" {
                        bindings
                            .rune_arms
                            .iter()
                            .enumerate()
                            .flat_map(|(index, arms)| {
                                arms.iter()
                                    .enumerate()
                                    .map(move |(blade, node)| CadTargetLight {
                                        node: node.clone(),
                                        target: (512 + index * 5 + blade) as u64,
                                        source_color: if index == 0 {
                                            Color::srgb(1., 0., 0.)
                                        } else {
                                            Color::srgb(0., 0., 1.)
                                        },
                                    })
                            })
                            .collect()
                    } else if matches!(asset.name.as_str(), "base" | "outpost") {
                        let offset = if asset.name == "base" { 0 } else { 128 };
                        let source_color = if asset.name == "base" {
                            Color::srgb(1., 0., 0.)
                        } else {
                            Color::srgb(0., 1., 1.)
                        };
                        bindings
                            .report
                            .get(&asset.name)
                            .map(|report| {
                                report
                                    .detector_bodies
                                    .iter()
                                    .enumerate()
                                    .map(|(index, body)| CadTargetLight {
                                        node: format!("source_body_{body}"),
                                        target: offset + (side * 32 + index) as u64,
                                        source_color,
                                    })
                                    .collect()
                            })
                            .unwrap_or_default()
                    } else {
                        Vec::new()
                    };
                    if !target_lights.is_empty() {
                        team = Some(if side == 0 {
                            TeamColor::Red
                        } else {
                            TeamColor::Blue
                        });
                    }
                    if !joints.is_empty() {
                        role = CadRole::Semantic {
                            scene: 0,
                            joints,
                            hidden_ids: Vec::new(),
                            rune_faces: Vec::new(),
                            target_lights,
                        };
                    }
                }
                CadInstance {
                    file: format!("verified://{}", asset.file),
                    transform,
                    team,
                    role,
                }
            })
        })
        .collect()
}
