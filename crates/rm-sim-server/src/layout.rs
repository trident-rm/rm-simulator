// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! From a verified CAD package to a `FieldConfig`: rune hubs and outpost
//! origins from the placements, the visual meshes as static trimeshes
//! (ground, rune frame and outpost towers, base and tech core), and the
//! chassis set down on the terrain.
use crate::cad_assets::CadAssets;
#[cfg(test)]
use crate::collision_mesh::CollisionMesh;
use crate::math::{gltf_child, quat_axis_angle, rotate};
use rm_sim_world::{
    ChassisConfig, ChassisPlacement, Field, FieldConfig, Pose, RobotKind, RuneKind, Team,
    chassis::Tether,
};

/// Rune face pivots inside the extracted rune asset (glTF axes, metres).
pub const RUNE_FACE_PIVOTS_M: [[f64; 3]; 2] = [
    [-0.035_830_23, 0.093, -0.270_318_74],
    [-0.035_832_06, 0.093, 0.367_281_26],
];
/// Blade fronts sit 12.2 mm ahead of the face pivot; targets live on that plane.
pub const RUNE_FACE_FRONT_M: f64 = 0.0122;
/// Start position and heading (degrees, counter-clockwise from +x) for a
/// team's pilot: on its own half just before the centre-line plateau,
/// looking down the field towards the opponent.
pub fn default_spawn(team: Team) -> ([f64; 3], f64) {
    match team {
        Team::Red => ([9.0, 0.0, 1.0], 180.0),
        Team::Blue => ([-9.0, 0.0, 1.0], 0.0),
    }
}

/// Team whose half of the field a point lies in. The V1.2.0 CAD paints its
/// red marking sheets on the +x half and the blue ones on the -x half
/// (the rulebook's field figures place one team per end and name no
/// axis); the centre line goes to red.
pub fn side_team(x_m: f64) -> Team {
    if x_m >= 0.0 { Team::Red } else { Team::Blue }
}

/// Owner of a rune face: the team whose half its front normal faces
/// (section 4.3.2.2: one side of the rune is red's, the other blue's). The
/// hub pose's forward points into the wheel, away from that team.
pub fn rune_team(hub_pose: &Pose) -> Team {
    side_team(-rotate(hub_pose.rotation_wxyz, [1.0, 0.0, 0.0])[0])
}

/// Owner of an outpost: the team on whose half it stands (section 5.5.1).
pub fn outpost_team(origin: &Pose) -> Team {
    side_team(origin.translation_m[0])
}
/// Compatibility alias for the physical arena catch-floor setting.
pub use rm_sim_assets::arena::CATCH_FLOOR_M;

/// Rune hub pose for one face of the placed CAD rune. The rear face (index 0)
/// is turned about the asset's up axis so its front normal faces its own
/// viewer.
///
/// Panics if `face` is neither 0 nor 1.
pub fn cad_rune_hub(rune_root: &Pose, face: usize) -> Pose {
    rune_hub_at(rune_root, face, RUNE_FACE_PIVOTS_M[face])
}

/// Rune hub pose at an explicit face pivot in the placed rune's glTF frame.
/// Face 0 is turned a half turn about the asset's up axis; both hubs then sit
/// `RUNE_FACE_FRONT_M` forward of their pivot.
fn rune_hub_at(rune_root: &Pose, face: usize, origin_m: [f64; 3]) -> Pose {
    let flip = if face == 0 {
        quat_axis_angle([0.0, 1.0, 0.0], std::f64::consts::PI)
    } else {
        [1.0, 0.0, 0.0, 0.0]
    };
    let pivot = gltf_child(rune_root, origin_m, flip);
    gltf_child(&pivot, [0.0, 0.0, RUNE_FACE_FRONT_M], [1.0, 0.0, 0.0, 0.0])
}

/// Tower base pose of a placed CAD outpost: the asset's +x forward and +y up
/// become the tower's FLU forward and up.
pub fn cad_outpost_origin(outpost_root: &Pose) -> Pose {
    gltf_child(
        outpost_root,
        [0.0; 3],
        quat_axis_angle([0.0, 1.0, 0.0], -std::f64::consts::FRAC_PI_2),
    )
}

#[cfg(test)]
use rm_sim_assets::arena::arena_boundary;
/// Compatibility exports of backend-independent arena preparation.
pub use rm_sim_assets::arena::{Terrain, arena_boundary_bounds};
pub use rm_sim_assets::components::{MechanismMesh, mechanism_joint};
/// Build terrain through shared assets while retaining native checked bindings.
pub fn load_terrain(cad: &CadAssets) -> anyhow::Result<Terrain> {
    rm_sim_assets::arena::load_terrain_for_supply_with_motions(cad, |cad, asset| {
        cad.rmuc_2027
            .as_ref()
            .map(|bindings| bindings.motions(cad, asset))
    })
}

/// Hand the terrain to the field: the ground, fixtures and equipment as
/// the same triangles used to draw the CAD.
pub fn add_terrain(field: &mut Field, terrain: &Terrain) -> anyhow::Result<()> {
    let geometry =
        rm_sim_assets::arena::prepare_terrain_geometry(terrain).map_err(anyhow::Error::msg)?;
    field.add_prepared_geometry(&geometry)?;
    for (station, units) in terrain.supply_units.as_chunks::<6>().0.iter().enumerate() {
        let mounts = std::array::from_fn(|i| {
            let mut mount = rm_sim_world::energy_unit::supply::SupplyMount::new(units[i].pose);
            mount.visual_node = Some(units[i].visual_node.clone());
            (u32::MAX - (station * 6 + i) as u32, mount)
        });
        field.add_supply_station(mounts, units[0].shape.clone())?;
    }
    Ok(())
}

/// Set a chassis down on the highest ground below the requested point (the
/// flat floor without terrain), heading `yaw_deg` from FLU forward. A flying
/// chassis instead holds `DRONE_FLIGHT_HEIGHT_M` above the playing floor and
/// is tied to its team's [`drone_tether`].
pub fn chassis_placement(
    mut config: ChassisConfig,
    kind: RobotKind,
    terrain: Option<&Terrain>,
    team: Team,
    spawn: [f64; 3],
    yaw_deg: f64,
) -> ChassisPlacement {
    let ground = terrain
        .and_then(|terrain| {
            terrain
                .ground
                .ground_height_below(spawn[0], spawn[1], spawn[2])
        })
        .unwrap_or(0.0);
    let height = if config.planar_flight {
        config.tether = Some(drone_tether(team));
        DRONE_FLIGHT_HEIGHT_M
    } else {
        ground + config.rest_height_m() + 0.01
    };
    let spawn = Pose::yawed([spawn[0], spawn[1], height], yaw_deg.to_radians());
    ChassisPlacement {
        config,
        spawn,
        team,
        kind,
        performance: None,
    }
}

/// Red's Landing Pad centre in world FLU metres (Figure 4-4 item 9, Figure
/// 4-17). Read off the default package's minimap artwork, not a drawing
/// dimension; blue's pad is the point mirror.
pub const RED_LANDING_PAD_M: [f64; 2] = [12.6, -5.8];
/// Height of a Drone's body centre above the playing floor, in metres. R51
/// keeps the drone's lowest point at least 1.5 m above the ground and its
/// launcher's speed monitor below the 2.4 m Flight Zone perimeter wall
/// (Figure 7-1); the prototype's lowest point sits about 1.8 m up and its
/// muzzle near 1.86 m. The height itself is a design choice.
pub const DRONE_FLIGHT_HEIGHT_M: f64 = 2.0;
/// Length of the Aerial Safety Rope's soft tether, section 4.5.
pub const DRONE_TETHER_LENGTH_M: f64 = 2.4;
/// Height of the wire rope the tether box rides, in metres. Assumed: the
/// rulebook gives no height, only that the rope hangs above the Pilot
/// Operation Room. At `DRONE_FLIGHT_HEIGHT_M` it leaves about 1.8 m of
/// sideways reach.
pub const DRONE_ROPE_HEIGHT_M: f64 = 3.6;
/// Distance of the Snap Ring from a team's own short edge of the field,
/// section 4.5 ("approximately 14 m"). On the 28 m field this is the centre
/// line.
pub const DRONE_SNAP_RING_FROM_EDGE_M: f64 = 14.0;
/// Half the field's length along x, in metres (Figure 4-5, 28 m overall).
pub const FIELD_HALF_LENGTH_M: f64 = 14.0;

/// The Aerial Safety Rope of `team`'s Drone (section 4.5): a rope running
/// along x over the team's Landing Pad from its own field edge to the Snap
/// Ring, with a 2.4 m tether below it. Red's rope is on its +x half.
///
/// ```
/// use rm_sim_server::layout::drone_tether;
/// use rm_sim_world::Team;
///
/// let red = drone_tether(Team::Red);
/// assert_eq!(red.rope_end_m[0], 0.0); // the Snap Ring is on the centre line
/// let blue = drone_tether(Team::Blue);
/// assert_eq!(blue.rope_start_m[0], -red.rope_start_m[0]);
/// assert_eq!(blue.rope_start_m[1], -red.rope_start_m[1]);
/// ```
pub fn drone_tether(team: Team) -> Tether {
    let side = match team {
        Team::Red => 1.0,
        Team::Blue => -1.0,
    };
    let y = side * RED_LANDING_PAD_M[1];
    Tether {
        rope_start_m: [side * FIELD_HALF_LENGTH_M, y, DRONE_ROPE_HEIGHT_M],
        rope_end_m: [
            side * (FIELD_HALF_LENGTH_M - DRONE_SNAP_RING_FROM_EDGE_M),
            y,
            DRONE_ROPE_HEIGHT_M,
        ],
        length_m: DRONE_TETHER_LENGTH_M,
    }
}
/// Spawn point and heading for a team's `slot`th Drone: above its Landing
/// Pad at the flight height, facing down the field, later slots stepped
/// towards the centre line so bodies do not overlap.
pub fn drone_slot(team: Team, slot: usize) -> ([f64; 3], f64) {
    let (side, yaw_deg) = match team {
        Team::Red => (1.0, 180.0),
        Team::Blue => (-1.0, 0.0),
    };
    let [x, y] = RED_LANDING_PAD_M;
    (
        [
            side * (x - slot as f64 * SPAWN_SLOT_SPACING_M),
            side * y,
            DRONE_FLIGHT_HEIGHT_M,
        ],
        yaw_deg,
    )
}

/// Sideways spacing between a team's spawn slots.
pub const SPAWN_SLOT_SPACING_M: f64 = 0.9;
/// Spawn point and heading for the `slot`th chassis of a team: the team's
/// default spawn, with later slots stepped left and right of it in turn so
/// bodies do not overlap.
pub fn spawn_slot(team: Team, slot: usize) -> ([f64; 3], f64) {
    let ([x, y, z], yaw_deg) = default_spawn(team);
    let step = slot.div_ceil(2) as f64 * SPAWN_SLOT_SPACING_M;
    let offset = if slot.is_multiple_of(2) { -step } else { step };
    ([x, y + offset, z], yaw_deg)
}

/// Places chassis for players as they arrive: one configuration, the
/// ground to set them down on, and the team spawn slots.
pub struct ChassisSpawner {
    /// Chassis configuration a chassis gets when no robot is named for it:
    /// training bots and the plain `slot` and `at` placements.
    pub config: ChassisConfig,
    /// Ground to set chassis down on; `None` uses the flat floor at height
    /// zero.
    pub terrain: Option<Terrain>,
}
impl ChassisSpawner {
    /// The placement for a team's `slot`th chassis, in the default
    /// configuration as an infantry.
    pub fn slot(&self, team: Team, slot: usize) -> ChassisPlacement {
        self.slot_with(self.config.clone(), RobotKind::Infantry, team, slot)
    }
    /// A placement at an explicit point, set down on the ground below it, in
    /// the default configuration as an infantry.
    pub fn at(&self, team: Team, spawn: [f64; 3], yaw_deg: f64) -> ChassisPlacement {
        self.at_with(
            self.config.clone(),
            RobotKind::Infantry,
            team,
            spawn,
            yaw_deg,
        )
    }
    /// The placement for a team's `slot`th chassis with an explicit
    /// configuration and robot class, for a pilot who chose a robot.
    pub fn slot_with(
        &self,
        config: ChassisConfig,
        kind: RobotKind,
        team: Team,
        slot: usize,
    ) -> ChassisPlacement {
        let (spawn, yaw_deg) = if config.planar_flight {
            drone_slot(team, slot)
        } else {
            spawn_slot(team, slot)
        };
        self.at_with(config, kind, team, spawn, yaw_deg)
    }
    /// A placement at an explicit point with an explicit configuration and
    /// robot class, set down on the ground below it.
    pub fn at_with(
        &self,
        config: ChassisConfig,
        kind: RobotKind,
        team: Team,
        spawn: [f64; 3],
        yaw_deg: f64,
    ) -> ChassisPlacement {
        chassis_placement(config, kind, self.terrain.as_ref(), team, spawn, yaw_deg)
    }
}

/// What to put on the CAD field besides its fixed geometry.
#[derive(Clone, Debug, PartialEq)]
pub struct LayoutOptions {
    /// Rune rules on both CAD faces, or none (the CAD rune stays static).
    pub rune: Option<RuneKind>,
    /// Outpost tower spin rate in radians per second.
    pub outpost_speed_rad_s: f64,
    /// The CAD visual triangles are in the world (lowers the catch floor).
    pub terrain: bool,
    /// A referee owning the runes and outposts by field side (`rune_team`,
    /// `outpost_team`).
    pub referee: bool,
    /// Ball restitution, hard flight limit and low-speed retirement rule the
    /// hosted field runs. Defaults to the simulator's control behaviour.
    pub projectile_policy: rm_sim_world::projectile::ProjectilePolicy,
}
impl Default for LayoutOptions {
    /// The shipped configuration: no rune, a still outpost, CAD terrain, a
    /// referee and spent-ball retirement.
    fn default() -> Self {
        Self {
            rune: None,
            outpost_speed_rad_s: 0.0,
            terrain: true,
            referee: true,
            projectile_policy: Default::default(),
        }
    }
}

/// The field configuration for a CAD package.
///
/// The simulation always runs on the fixed 128 Hz tick from
/// [`rm_sim_world::tick_ns`], so there is no rate to freeze here.
///
/// Panics if the rune asset has no placement.
pub fn field_config(cad: &CadAssets, options: &LayoutOptions) -> FieldConfig {
    FieldConfig {
        rmuc_2027: if options.referee {
            cad.rmuc_2027.as_ref().map(|b| b.config())
        } else {
            None
        },
        floor_height_m: if options.terrain { CATCH_FLOOR_M } else { 0.0 },
        projectile_policy: options.projectile_policy,
        ..FieldConfig::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore = "requires the verified field-2027 CAD package"]
    fn standalone_arena_matches_native_terrain_installation() {
        let path = std::env::var_os("RM_ARENA_PACKAGE")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|| std::path::PathBuf::from("../../field-2027"));
        let native = crate::cad_assets::load(&path).unwrap();
        let shared = rm_sim_assets::cad_assets::load(&path).unwrap();
        let terrain = load_terrain(&native).unwrap();
        let mut field = Field::new(&Default::default()).unwrap();
        add_terrain(&mut field, &terrain).unwrap();
        let arena = rm_sim_assets::arena::PreparedArena::from_terrain(
            rm_sim_assets::arena::load_terrain_for_supply_with_motions(&shared, |cad, asset| {
                cad.rmuc_2027.as_ref().map(|b| b.motions(cad, asset))
            })
            .unwrap(),
        )
        .unwrap();
        assert_eq!(terrain.describe(), arena.terrain().describe());
        assert_eq!(
            field.static_geometry_snapshot().into_triangles(),
            arena
                .geometry()
                .at_mechanisms(&rm_sim_world::referee::mechanism_view(None, 0))
                .into_triangles()
        );
        let recipe = arena
            .scenario(vec![], rm_sim_world::StepDuration::HZ_128, |_| Ok(()))
            .unwrap();
        let mut session = rm_sim_physics::session::RobotSession::new(recipe).unwrap();
        session.step(128).unwrap();
        session.reset().unwrap();
        assert_eq!(session.stamp().tick, 0);
    }

    #[test]
    fn arena_boundary_follows_grounded_scenery_instead_of_the_slab_apron() {
        let floor = CollisionMesh {
            vertices_m: vec![[-14.5, -8.0, -0.2], [14.5, 8.0, 0.0]],
            ..Default::default()
        };
        let scenery = CollisionMesh {
            vertices_m: vec![
                [-14.0, -7.5, 0.0],
                [14.0, 7.5, 0.0],
                [-15.0, -8.5, 1.5],
                [15.0, 8.5, 1.5],
            ],
            ..Default::default()
        };
        let bounds = arena_boundary_bounds(&floor, &scenery).unwrap();
        assert_eq!(bounds, [[-14.0, -7.5, -0.2], [14.0, 7.5, 0.0]]);
        let walls = arena_boundary(bounds);
        assert!(
            walls
                .vertices_m
                .iter()
                .all(|p| p[0].abs() <= 14.1 && p[1].abs() <= 7.6)
        );
        assert_eq!(
            arena_boundary_bounds(&floor, &CollisionMesh::default()).unwrap(),
            [[-14.5, -8.0, -0.2], [14.5, 8.0, 0.0]]
        );
    }

    #[test]
    fn arena_walls_contain_driving_robots_and_do_not_add_an_infinite_floor() {
        use rm_sim_world::ChassisCommand;
        let floor = CollisionMesh {
            vertices_m: vec![
                [-2.0, -1.0, 0.0],
                [2.0, -1.0, 0.0],
                [2.0, 1.0, 0.0],
                [-2.0, 1.0, 0.0],
            ],
            triangles: vec![[0, 1, 2], [0, 2, 3]],
        };
        let boundary =
            arena_boundary(arena_boundary_bounds(&floor, &CollisionMesh::default()).unwrap());
        assert_eq!(boundary.triangles.len(), 48);
        for (start, command) in [
            (
                [1.4, 0.0],
                ChassisCommand {
                    forward_m_s: 5.0,
                    ..Default::default()
                },
            ),
            (
                [-1.4, 0.0],
                ChassisCommand {
                    forward_m_s: -5.0,
                    ..Default::default()
                },
            ),
            (
                [0.0, 0.4],
                ChassisCommand {
                    left_m_s: 5.0,
                    ..Default::default()
                },
            ),
            (
                [0.0, -0.4],
                ChassisCommand {
                    left_m_s: -5.0,
                    ..Default::default()
                },
            ),
        ] {
            let mut field = Field::new(&FieldConfig {
                runes: vec![],
                outposts: vec![],
                ..Default::default()
            })
            .unwrap();
            field.remove_catch_floor();
            field
                .add_static_mesh(floor.vertices_m.clone(), floor.triangles.clone())
                .unwrap();
            field
                .add_static_mesh(boundary.vertices_m.clone(), boundary.triangles.clone())
                .unwrap();
            let config = ChassisConfig::default();
            let id = field
                .add_chassis(&ChassisPlacement {
                    team: Team::Red,
                    kind: rm_sim_world::RobotKind::Infantry,
                    performance: None,
                    spawn: Pose::at([start[0], start[1], config.rest_height_m()]),
                    config,
                })
                .unwrap();
            field.command_chassis(id, command).unwrap();
            field.step(1500).unwrap();
            let p = field.snapshot().chassis[0].pose.translation_m;
            assert!(
                p[0].abs() < 2.0 && p[1].abs() < 1.0 && p[2] > 0.0,
                "escaped: {p:?}"
            );
            field.place_chassis(id, Pose::at([4.0, 0.0, 1.0])).unwrap();
            field.step(1000).unwrap();
            assert!(field.snapshot().chassis[0].pose.translation_m[2] < -2.0);
        }
    }

    use crate::cad_assets::CadAsset;
    use crate::math::{RENDER_BASIS_WXYZ, gltf_root_pose, quat_conjugate, quat_multiply};

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        a.iter().zip(&b).all(|(a, b)| (a - b).abs() < 1e-4)
    }
    #[test]
    fn cad_rune_faces_get_opposing_front_normals_ahead_of_their_pivots() {
        // Rune asset placed with its front (+z) toward FLU forward: under
        // the renderer's basis glTF +z is FLU backward, so the placement is
        // a half turn about the renderer's up axis, read back through that
        // basis.
        let root = gltf_root_pose(
            [5.0, 0.0, 2.0],
            quat_multiply(
                quat_conjugate(RENDER_BASIS_WXYZ),
                quat_axis_angle([0.0, 1.0, 0.0], std::f64::consts::PI),
            ),
        );
        let front = cad_rune_hub(&root, 1);
        let rear = cad_rune_hub(&root, 0);
        // The hub pose's FLU forward is the direction into the wheel (opposite the front normal).
        assert!(close(
            rotate(front.rotation_wxyz, [1.0, 0.0, 0.0]),
            [-1.0, 0.0, 0.0]
        ));
        assert!(close(
            rotate(rear.rotation_wxyz, [1.0, 0.0, 0.0]),
            [1.0, 0.0, 0.0]
        ));
        // Face 1 sits ahead of the pivot along its own normal; face 0 the other way.
        let z1 = RUNE_FACE_PIVOTS_M[1][2] + RUNE_FACE_FRONT_M;
        let z0 = RUNE_FACE_PIVOTS_M[0][2] - RUNE_FACE_FRONT_M;
        assert!(close(
            front.translation_m,
            [5.0 + z1, RUNE_FACE_PIVOTS_M[1][0], 2.093]
        ));
        assert!(close(
            rear.translation_m,
            [5.0 + z0, RUNE_FACE_PIVOTS_M[0][0], 2.093]
        ));
        // Both hubs keep up as up.
        assert!(close(
            rotate(front.rotation_wxyz, [0.0, 0.0, 1.0]),
            [0.0, 0.0, 1.0]
        ));
        assert!(close(
            rotate(rear.rotation_wxyz, [0.0, 0.0, 1.0]),
            [0.0, 0.0, 1.0]
        ));
    }
    #[test]
    fn cad_outpost_forward_follows_the_asset_x_axis() {
        // The blue outpost placement (see cad_assets tests).
        let n = (0.716_301_943_f64.powi(2) + 0.697_790_46_f64.powi(2)).sqrt();
        let root = gltf_root_pose(
            [3.0917, 3.8674, 0.1907],
            [0.0, 0.0, 0.716_301_943 / n, 0.697_790_46 / n],
        );
        let origin = cad_outpost_origin(&root);
        assert!(close(origin.translation_m, [3.0917, 3.8674, 0.1907]));
        assert!(close(
            rotate(origin.rotation_wxyz, [1.0, 0.0, 0.0]),
            [-1.0, 0.0, 0.0]
        ));
        let up = rotate(origin.rotation_wxyz, [0.0, 0.0, 1.0]);
        assert!(up[2] > 0.999, "{up:?}");
    }
    #[test]
    fn chassis_is_set_down_on_the_ground_below_the_spawn_point() {
        let config = ChassisConfig::default();
        let terrain = Terrain {
            supply_units: vec![],
            native_terrain: None,
            ground_collision: None,
            boundary: CollisionMesh::default(),
            projectile_bounds_m: None,
            mechanisms: Vec::new(),
            ground: CollisionMesh {
                vertices_m: vec![
                    [-5.0, -5.0, 0.3],
                    [5.0, -5.0, 0.3],
                    [5.0, 5.0, 0.3],
                    [-5.0, 5.0, 0.3],
                ],
                triangles: vec![[0, 1, 2], [0, 2, 3]],
            }
            .into(),
            fixtures: CollisionMesh::default(),
            equipment: Vec::new(),
            endpoint_bakes: vec![],
            native_shapes: Vec::new(),
            native_query_meshes: Vec::new(),
        };
        let placement = chassis_placement(
            config.clone(),
            RobotKind::Hero,
            Some(&terrain),
            Team::Red,
            [1.0, 2.0, 1.0],
            90.0,
        );
        assert_eq!(placement.kind, RobotKind::Hero);
        assert!(close(
            placement.spawn.translation_m,
            [1.0, 2.0, 0.3 + config.rest_height_m() + 0.01]
        ));
        assert!(close(
            rotate(placement.spawn.rotation_wxyz, [1.0, 0.0, 0.0]),
            [0.0, 1.0, 0.0]
        ));
        // Without terrain the flat floor at zero is the ground.
        let flat = chassis_placement(
            config.clone(),
            RobotKind::Infantry,
            None,
            Team::Red,
            [1.0, 2.0, 1.0],
            0.0,
        );
        assert!(close(
            flat.spawn.translation_m,
            [1.0, 2.0, config.rest_height_m() + 0.01]
        ));
        // Above the mesh footprint there is no ground: the flat floor applies.
        let outside = chassis_placement(
            config.clone(),
            RobotKind::Infantry,
            Some(&terrain),
            Team::Blue,
            [9.0, 9.0, 1.0],
            0.0,
        );
        assert!(close(
            outside.spawn.translation_m,
            [9.0, 9.0, config.rest_height_m() + 0.01]
        ));
        assert_eq!(outside.team, Team::Blue);
        // Spawn slots step sideways from the team's default point.
        assert_eq!(spawn_slot(Team::Red, 0), default_spawn(Team::Red));
        assert_eq!(spawn_slot(Team::Blue, 1).0[1], SPAWN_SLOT_SPACING_M);
        assert_eq!(spawn_slot(Team::Blue, 2).0[1], -SPAWN_SLOT_SPACING_M);
        assert_eq!(spawn_slot(Team::Blue, 3).0[1], 2.0 * SPAWN_SLOT_SPACING_M);
        let spawner = ChassisSpawner {
            config: config.clone(),
            terrain: Some(terrain),
        };
        let second = spawner.slot(Team::Red, 1);
        assert!(close(
            second.spawn.translation_m,
            [9.0, SPAWN_SLOT_SPACING_M, config.rest_height_m() + 0.01]
        ));
    }
    #[test]
    fn layout_options_place_runes_outposts_and_the_referee() {
        let cad = CadAssets {
            terrain_collision: None,
            rmul_2027: None,
            rmuc_2027: None,
            season: rm_sim_world::season::Season::Rm2026,
            rest_pose_only: false,
            root: std::path::PathBuf::new(),
            floor_top_cad_m: crate::cad_assets::FLOOR_TOP_CAD_M,
            collision_solids: true,
            floor: asset(0),
            arena_static: asset(0),
            rune: asset(1),
            outpost: asset(2),
            base: asset(2),
            tech_core: asset(2),
            static_assets: Vec::new(),
        };
        let mut cad = cad;
        cad.rune.placements[0] = gltf_root_pose(
            [5.0, 0.0, 2.0],
            quat_multiply(
                quat_conjugate(RENDER_BASIS_WXYZ),
                quat_axis_angle([0.0, 1.0, 0.0], std::f64::consts::PI),
            ),
        );
        cad.outpost.placements = vec![
            gltf_root_pose([3.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
            gltf_root_pose([-3.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
        ];
        let options = LayoutOptions {
            rune: Some(RuneKind::Big),
            outpost_speed_rad_s: 1.0,
            terrain: true,
            referee: true,
            projectile_policy: Default::default(),
        };
        let config = field_config(&cad, &options);
        assert!(config.runes.is_empty() && config.outposts.is_empty());
        assert!(config.referee.is_none() && config.rmuc_2027.is_none());
        assert_eq!(config.floor_height_m, CATCH_FLOOR_M);
        assert_eq!(side_team(0.0), Team::Red);
        assert_eq!(side_team(-0.1), Team::Blue);
        assert_eq!(default_spawn(Team::Red).0[0], 9.0);
        assert_eq!(default_spawn(Team::Blue), ([-9.0, 0.0, 1.0], 0.0));
        assert!(Field::new(&field_config(&cad, &options)).is_ok());
        let plain = field_config(
            &cad,
            &LayoutOptions {
                rune: None,
                terrain: false,
                referee: false,
                projectile_policy: Default::default(),
                ..options
            },
        );
        assert!(plain.runes.is_empty());
        assert!(plain.referee.is_none());
        assert_eq!(plain.floor_height_m, 0.0);
    }

    fn asset(placements: usize) -> CadAsset {
        CadAsset {
            collision_shapes: None,
            semantics: None,
            file: "x.glb".into(),
            placements: (0..placements)
                .map(|i| gltf_root_pose([i as f64, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]))
                .collect(),
            collision: None,
            source_tessellated_collision: false,
            bake_motion_endpoints: false,
        }
    }
}
