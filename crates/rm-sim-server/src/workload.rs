// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! The deterministic field the bandwidth probe and the measurement examples drive.
//!
//! Bandwidth comparisons and training workloads behind the compressed-dictionary
//! checkpoints must start from the same field, or the numbers stop being
//! comparable. The builder lives in the library rather than in each harness so
//! that "the same workload" is a
//! compiler-checked fact instead of a copied block: the two-rune, two-team
//! referee layout with no CAD, and `players` chassis that alternate teams in
//! spawn order.
//!
//! This is measurement scaffolding, not gameplay: nothing in the running host
//! calls it, and it reads no assets.
use crate::layout::ChassisSpawner;
use crate::simulation::Simulation;
use rm_sim_world::{ChassisCommand, ChassisConfig, Field, FieldConfig, Team};

/// Build one measurement field and its chassis.
///
/// No CAD is read: the implicit floor and the rule layout are enough for chassis
/// motion, projectiles and the referee, so a workload needs no assets and runs
/// the same everywhere. Chassis are spawned in index order and alternate teams,
/// so index parity alone fixes every team assignment. Returns the simulation and
/// the chassis ids in spawn order.
///
/// ```
/// use rm_sim_server::workload;
///
/// let (simulation, chassis) = workload::simulation(2);
/// assert_eq!(chassis.len(), 2);
/// assert_eq!(simulation.state().field.chassis.len(), 2);
/// ```
pub fn simulation(players: usize) -> (Simulation, Vec<u32>) {
    let config = field_config();
    let mut simulation = Simulation::new(
        Field::new(&config).expect("the measurement field configuration is valid"),
        false,
    )
    .with_spawner(ChassisSpawner {
        config: ChassisConfig::default(),
        terrain: None,
    });
    let chassis = (0..players)
        .map(|index| {
            let team = if index.is_multiple_of(2) {
                Team::Red
            } else {
                Team::Blue
            };
            simulation
                .spawn_chassis(team)
                .expect("the measurement field offers a chassis per player")
        })
        .collect();
    (simulation, chassis)
}

/// Synthetic native field for repeatable no-CAD measurements and protocol fixtures.
/// Geometry is deliberately artificial and must never substitute for a verified
/// arena package. Coordinates are metres in FLU; the clock remains 128 Hz.
///
/// ```
/// let config = rm_sim_server::workload::field_config();
/// assert!(config.rmuc_2027.is_some());
/// rm_sim_world::Field::new(&config).unwrap();
/// ```
pub fn field_config() -> FieldConfig {
    use rm_sim_world::{
        Pose,
        rmuc_2027::{Config, Detector, RuneGeometry, Tower},
    };
    let detector = |x, y| Detector {
        pose: Pose::at([x, y, 1.0]),
        half_m: [0.045, 0.044],
    };
    FieldConfig {
        rmuc_2027: Some(Config {
            bases: [vec![detector(12., 0.)], vec![detector(-12., 0.)]],
            outposts: [1., -1.].map(|side| Tower {
                pivot_m: [side * 3., 0., 1.],
                axis: [0., 0., 1.],
                detectors: (0..3)
                    .map(|i| detector(side * 3., i as f64 * 0.3))
                    .collect(),
            }),
            zones: vec![],
            supply_zones: vec![],
            vantage_zones: vec![],
            terrain_zones: vec![],
            runes: [1., -1.]
                .map(|side| RuneGeometry {
                    pivot_m: [0., side * 6., 1.],
                    axis: [0., 1., 0.],
                    targets: std::array::from_fn(|i| {
                        let a = i as f64 * std::f64::consts::TAU / 5.;
                        Pose::at([a.cos() * 0.7, side * 6., 1. + a.sin() * 0.7])
                    }),
                })
                .to_vec(),
            rotor_direction: 1,
        }),
        ..FieldConfig::default()
    }
}

/// The command the constant-drive measurement workloads apply to every chassis:
/// a steady forward speed with a small yaw rate, so a run carries motion without
/// depending on how many frames have elapsed.
pub fn constant_drive() -> ChassisCommand {
    ChassisCommand {
        forward_m_s: 1.0,
        yaw_rate_rad_s: 0.4,
        ..Default::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn players_alternate_teams_in_spawn_order() {
        let (simulation, chassis) = simulation(4);
        assert_eq!(chassis.len(), 4);
        let state = simulation.state();
        let teams: Vec<Team> = state
            .field
            .chassis
            .iter()
            .map(|chassis| chassis.team)
            .collect();
        assert_eq!(teams, vec![Team::Red, Team::Blue, Team::Red, Team::Blue]);
    }

    #[test]
    fn an_empty_workload_needs_no_chassis() {
        let (_, chassis) = simulation(0);
        assert!(chassis.is_empty());
    }

    #[test]
    fn the_field_carries_two_native_runes_and_authority() {
        let (simulation, _) = simulation(1);
        let state = simulation.state();
        assert!(state.field.runes.is_empty());
        assert_eq!(
            state.field.rmuc_2027.as_ref().unwrap().config.runes.len(),
            2
        );
        assert!(state.field.referee.is_none());
    }
}
