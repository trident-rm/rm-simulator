// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Official source geometry, competition admission and terrain continuity.
use rm_sim_server::{cad_assets, host_args::Competition, layout, simulation::Simulation};

#[test]
fn official_fields_preserve_source_geometry_and_reject_cross_competition() {
    for (competition, triangles, extent) in [
        (Competition::Rmul2027, 3811, 6.3),
        (Competition::RmulInfantry2027, 2124, 2.8),
    ] {
        let format = competition.rmul_format().unwrap();
        let cad = cad_assets::load(&cad_assets::default_rmul_assets(format)).unwrap();
        assert_eq!(cad.rmul_2027, Some(format));
        assert!(cad.rmuc_2027.is_none());
        let terrain = layout::load_terrain(&cad).unwrap();
        assert_eq!(terrain.ground.triangles.len(), triangles);
        assert!(
            terrain
                .ground
                .vertices_m
                .iter()
                .any(|p| (p[0] - extent).abs() < 1e-5)
        );
        assert!(
            terrain
                .ground
                .vertices_m
                .iter()
                .any(|p| (p[2] - 0.42).abs() < 1e-5)
        );
        let mut simulation =
            Simulation::rmul_from_cad(&cad, competition, true, Some(Default::default()), true)
                .unwrap();
        assert_eq!(simulation.competition(), competition);
        let id = simulation
            .spawn_robot(
                rm_sim_world::Team::Red,
                rm_sim_server::protocol::Robot::Infantry3,
            )
            .unwrap();
        simulation.step(256).unwrap();
        let height = simulation
            .field()
            .chassis_snapshot(id)
            .unwrap()
            .pose
            .translation_m[2];
        assert!(
            (0.05..0.5).contains(&height),
            "source floor must support the robot, got {height}"
        );
        let other = if competition == Competition::Rmul2027 {
            Competition::RmulInfantry2027
        } else {
            Competition::Rmul2027
        };
        assert!(Simulation::rmul_from_cad(&cad, other, true, None, true).is_err());
        assert!(Simulation::rmul_from_cad(&cad, Competition::Rmuc2027, true, None, true).is_err());
        assert!(Simulation::from_cad(&cad, &Default::default(), None, true, |_| {}).is_err());
    }
}
