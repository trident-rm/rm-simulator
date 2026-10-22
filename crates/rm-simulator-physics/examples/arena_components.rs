// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Independent tunnel, occupancy and moving Outpost detectors, without Field or CAD.
use rm_simulator_physics::{
    ArmorTarget, Pose, StepDuration, TargetFace,
    arena::{Detector, OutpostModel, TunnelModel, ZoneFootprint},
    session::{RobotScenario, RobotSession},
};

fn main() -> Result<(), &'static str> {
    // Illustrative fixture dimensions, not a claim about official arena geometry.
    let tunnel = TunnelModel {
        pose: Pose::at([0., 0., 0.]),
        opening_m: [2., 1., 0.25],
        wall_thickness_m: 0.1,
    };
    let outpost = OutpostModel {
        pivot_m: [3., 0., 1.],
        axis: [0., 0., 1.],
        detectors: (0..3)
            .map(|i| Detector {
                pose: Pose::yawed([3.3, 0., 1. + 0.1 * f64::from(i)], 0.),
                half_m: [0.0675, 0.0275],
            })
            .collect(),
    };
    outpost.validate()?;
    let targets = |angle| {
        (0..3)
            .map(|face| TargetFace {
                target: ArmorTarget::Outpost {
                    outpost: 0,
                    face: face as u32,
                },
                pose: outpost.detector_pose(face, angle).unwrap(),
            })
            .collect()
    };
    let scenario = RobotScenario::new(targets(0.), 0., StepDuration::HZ_1000, move |physics| {
        tunnel.install(physics)
    })?;
    let mut session = RobotSession::new(scenario)?;
    for i in 1..=1000 {
        // The caller supplies angle endpoints; the physical session interpolates.
        session.step_with_targets(&targets(f64::from(i) * 0.001))?;
    }
    let footprint = ZoneFootprint {
        polygon_m: &[[-1., -0.5], [1., -0.5], [1., 0.5], [-1., 0.5]],
        floor_m: 0.,
        height_m: 0.25,
    };
    assert!(footprint.contains([0., 0., 0.1]));
    assert!(
        session
            .ground_truth()
            .static_geometry_snapshot(true)
            .segment_clear([-2., 0., 0.1], [2., 0., 0.1])
    );
    println!(
        "Standalone tunnel, occupancy and Outpost: {} ns",
        session.stamp().time_ns
    );
    Ok(())
}
