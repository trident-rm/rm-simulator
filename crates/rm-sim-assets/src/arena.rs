// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Complete verified arena collision preparation without a match, server or renderer.
use crate::{
    cad_assets::CadAssets,
    collision_mesh::{CollisionMesh, GroundMesh},
};
use anyhow::ensure;
use rm_sim_physics::{
    StaticGeometry, StepDuration, TargetFace, WorldPhysics, session::RobotScenario,
};
/// Catch plane used while constructing CAD scenes; installed arenas remove it.
/// This simulator setting is below the source crowned slab.
pub const CATCH_FLOOR_M: f64 = -0.45;
const FIXED_NODE: &str = "static";
/// Everything rigid the chassis and projectiles collide with, in FLU metres.
pub struct Terrain {
    /// Separate native terrain installation, replacing matching ground owners.
    pub native_terrain: Option<crate::terrain_collision::TerrainPackage>,
    /// Physical mesh remainder. The full ground is retained for support lookups.
    pub ground_collision: Option<CollisionMesh>,
    /// The perimeter walls that keep chassis inside the arena, in FLU metres.
    /// Projectile collision ignores them.
    pub boundary: CollisionMesh,
    /// Projectile bounds in FLU metres as `[low, high]`; a shot touching or
    /// crossing this XY perimeter is absorbed. `None` leaves the field without
    /// projectile bounds.
    pub projectile_bounds_m: Option<[[f64; 3]; 2]>,
    /// Moving mechanisms, one entry per placed translation or rotor joint.
    pub mechanisms: Vec<MechanismMesh>,
    /// The floor and arena visual meshes and the
    /// placed static scenery, as one triangle soup; also the ground the
    /// chassis is set down on.
    pub ground: GroundMesh,
    /// The rune frame and the outpost towers at their placements: the
    /// `static` node of each visual mesh. The faces, hubs and rotors are
    /// rule-driven bodies the field owns, so their nodes are left out.
    pub fixtures: CollisionMesh,
    /// The bases and tech cores at their placements, from their
    /// visual meshes, kept as trimeshes.
    pub equipment: Vec<CollisionMesh>,
    /// Opt-in whole-Base endpoint caches as equipment mesh index and placed team.
    pub endpoint_bakes: Vec<(usize, rm_sim_physics::Team)>,
    /// Fixed native children grouped per source placement, preserving local
    /// broad-phase bounds. Poses use world FLU metres and wxyz quaternions.
    pub native_shapes: Vec<Vec<crate::components::NativeShape>>,
    /// Query-only mesh by fixed native group index. These surfaces never add contact forces.
    pub native_query_meshes: Vec<(usize, CollisionMesh)>,
}
/// Shared equipment composition records and semantic channel classification.
pub use crate::components::{MechanismMesh, mechanism_joint};

impl Terrain {
    /// Mesh installed for physical contacts, excluding separately installed terrain.
    pub fn collision_ground(&self) -> &CollisionMesh {
        self.ground_collision.as_ref().unwrap_or(&self.ground)
    }
    /// One line for the start-up log.
    pub fn describe(&self) -> String {
        format!(
            "field collision: {} ground triangles, {} fixture triangles, {} equipment triangles, {} native primitives, {} query-only triangles (source tessellations, reviewed meshes or declared shapes)",
            self.collision_ground().triangles.len(),
            self.fixtures.triangles.len() + self.boundary.triangles.len(),
            self.equipment
                .iter()
                .map(|m| m.triangles.len())
                .sum::<usize>(),
            self.native_shapes.iter().map(Vec::len).sum::<usize>()
                + self
                    .mechanisms
                    .iter()
                    .map(|p| p.native_shapes.len())
                    .sum::<usize>(),
            self.native_query_meshes
                .iter()
                .map(|(_, mesh)| mesh.triangles.len())
                .sum::<usize>()
        )
    }
}

/// Build the collision terrain for a verified package.
///
/// The floor and arena scenery become the ground soup, with the placed static
/// scenery appended. The rune and outpost contribute only their `static` node;
/// the bases, tech cores and static scenery contribute their collision
/// primitives, and the prismatic joints the referee drives are pulled out into
/// `mechanisms` instead of staying in those static meshes. Everything is
/// placed in world FLU metres with the playing floor top at height zero.
pub fn load_terrain(cad: &CadAssets) -> anyhow::Result<Terrain> {
    load_terrain_with_motions(cad, |cad, asset| {
        cad.rmuc_2027.as_ref().map(|b| b.motions(cad, asset))
    })
}

/// Prepare physical terrain using caller-supplied source motion partitions.
/// This supports checked packages with runtime-specific binding DTOs without
/// importing their match configuration into assets. `None` retains semantic
/// fallback; `Some(&[])` explicitly selects an empty source motion partition.
pub fn load_terrain_with_motions<B>(
    cad: &CadAssets<B>,
    source_motions: impl for<'a> Fn(
        &'a CadAssets<B>,
        &'a crate::cad_assets::CadAsset,
    ) -> Option<&'a [crate::rmuc_2027::Motion]>,
) -> anyhow::Result<Terrain> {
    let load_component = |cad: &CadAssets<B>, asset: &crate::cad_assets::CadAsset, role| {
        crate::components::load(cad, asset, role, source_motions(cad, asset))
    };
    ensure!(
        cad.floor.collision_shapes.is_none() && cad.arena_static.collision_shapes.is_none(),
        "floor and arena ground require mesh collision for ground-height queries"
    );
    let mut native_shapes = Vec::new();
    let mut native_query_meshes = Vec::new();
    let mut mechanisms = Vec::new();
    let mut ground = cad
        .floor
        .physics_mesh(&cad.root)?
        .into_flu(cad.floor_top_cad_m);
    let scenery = cad
        .arena_static
        .physics_mesh(&cad.root)?
        .into_flu(cad.floor_top_cad_m);
    let source_scenery = if cad.terrain_collision.is_some() {
        crate::collision_mesh::load_glb(&cad.root.join(&cad.arena_static.file))?
            .into_flu(cad.floor_top_cad_m)
    } else {
        scenery.clone()
    };
    let projectile_bounds_m = arena_boundary_bounds(&ground, &source_scenery)?;
    let boundary = arena_boundary(projectile_bounds_m);
    let mut ground_collision = cad.terrain_collision.as_ref().map(|_| {
        let mut g = ground.clone();
        g.append(scenery);
        g
    });
    ground.append(source_scenery);
    let native_terrain = cad
        .terrain_collision
        .as_ref()
        .map(|source| {
            source.prepare(
                rm_sim_physics::Pose {
                    translation_m: [0., -crate::cad_assets::CENTER_CAD_Y_M, -cad.floor_top_cad_m],
                    rotation_wxyz: crate::math::quat_axis_angle(
                        [1., 0., 0.],
                        std::f64::consts::FRAC_PI_2,
                    ),
                },
                sim_engine::model::ContactMaterial {
                    friction: 0.4,
                    restitution: 0.45,
                    memberships: u32::MAX,
                    filter: u32::MAX,
                },
            )
        })
        .transpose()?;
    if let Some(package) = &native_terrain {
        ensure!(
            !package.parts.iter().any(|p| p.id == "floor/two-quads"),
            "native terrain must not duplicate the separately loaded floor"
        );
    }
    for asset in &cad.static_assets {
        let model = load_component(cad, asset, crate::components::ComponentRole::Equipment)?;
        mechanisms.extend(model.mechanisms(&asset.placements));
        for placement in &asset.placements {
            ground.append(model.fixed.placed(placement));
            if let Some(g) = &mut ground_collision {
                g.append(model.fixed.placed(placement));
            }
            let shapes = crate::components::place_shapes(&model.native_shapes, placement);
            if !shapes.is_empty() {
                if let Some(query) = model.query_mesh.as_ref() {
                    native_query_meshes.push((native_shapes.len(), query.placed(placement)));
                }
                native_shapes.push(shapes);
            }
        }
    }
    let mut fixtures = CollisionMesh::default();
    for (name, asset) in [("rune", &cad.rune), ("outpost", &cad.outpost)] {
        let role = if name == "rune" {
            crate::components::ComponentRole::Rune
        } else {
            crate::components::ComponentRole::Outpost
        };
        let model = load_component(cad, asset, role)?;
        ensure!(
            !model.fixed.triangles.is_empty() || !model.native_shapes.is_empty(),
            "{name} visual mesh has no `{FIXED_NODE}` node"
        );
        mechanisms.extend(model.mechanisms(&asset.placements));
        for placement in &asset.placements {
            fixtures.append(model.fixed.placed(placement));
            let shapes = crate::components::place_shapes(&model.native_shapes, placement);
            if !shapes.is_empty() {
                if let Some(query) = model.query_mesh.as_ref() {
                    native_query_meshes.push((native_shapes.len(), query.placed(placement)));
                }
                native_shapes.push(shapes);
            }
        }
    }
    let mut equipment = Vec::new();
    let mut endpoint_bakes = Vec::new();
    for asset in [&cad.base, &cad.tech_core] {
        let model = load_component(cad, asset, crate::components::ComponentRole::Equipment)?;
        mechanisms.extend(model.mechanisms(&asset.placements));
        for placement in &asset.placements {
            if asset.bake_motion_endpoints {
                ensure!(
                    asset.file == cad.base.file && !model.fixed.triangles.is_empty(),
                    "endpoint baking requires a nonempty Base fixed body"
                );
                endpoint_bakes.push((
                    equipment.len(),
                    if placement.translation_m[0] >= 0. {
                        rm_sim_physics::Team::Red
                    } else {
                        rm_sim_physics::Team::Blue
                    },
                ));
            }
            equipment.push(model.fixed.placed(placement));
            let shapes = crate::components::place_shapes(&model.native_shapes, placement);
            if !shapes.is_empty() {
                if let Some(query) = model.query_mesh.as_ref() {
                    native_query_meshes.push((native_shapes.len(), query.placed(placement)));
                }
                native_shapes.push(shapes);
            }
        }
    }
    Ok(Terrain {
        native_terrain,
        ground_collision,
        boundary,
        projectile_bounds_m: Some(projectile_bounds_m),
        mechanisms,
        ground: ground.into(),
        fixtures,
        equipment,
        endpoint_bakes,
        native_shapes,
        native_query_meshes,
    })
}

/// Playing boundary in FLU metres, shared by the fence and its colliders.
/// The slab includes an exterior apron. Fit XY to scenery vertices at slab
/// elevation, excluding elevated equipment overhangs. The 10 mm tolerance is
/// an asset-fitting setting, not a rulebook dimension. Empty scenery falls
/// back to the slab bounds for synthetic fields.
pub fn arena_boundary_bounds(
    floor: &CollisionMesh,
    scenery: &CollisionMesh,
) -> anyhow::Result<[[f64; 3]; 2]> {
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    for point in &floor.vertices_m {
        for i in 0..3 {
            low[i] = low[i].min(point[i]);
            high[i] = high[i].max(point[i]);
        }
    }
    ensure!(
        low.iter().chain(&high).all(|x| x.is_finite()) && high[0] > low[0] && high[1] > low[1],
        "floor has no finite arena boundary"
    );
    let mut edge_low = [f64::INFINITY; 2];
    let mut edge_high = [f64::NEG_INFINITY; 2];
    for point in &scenery.vertices_m {
        if point[2] >= low[2] - 0.01 && point[2] <= high[2] + 0.01 {
            for axis in 0..2 {
                edge_low[axis] = edge_low[axis].min(point[axis]);
                edge_high[axis] = edge_high[axis].max(point[axis]);
            }
        }
    }
    // Only use a footprint spanning the arena, not an isolated fixture.
    if (0..2).all(|axis| edge_high[axis] - edge_low[axis] > (high[axis] - low[axis]) * 0.5) {
        for axis in 0..2 {
            low[axis] = low[axis].max(edge_low[axis]);
            high[axis] = high[axis].min(edge_high[axis]);
        }
    }
    Ok([low, high])
}

/// Build robot-only containment walls from FLU bounds in metres.
/// Two metres high and 100 mm thick are simulator settings, not CAD dimensions.
pub fn arena_boundary([low, high]: [[f64; 3]; 2]) -> CollisionMesh {
    let mut mesh = CollisionMesh::default();
    for (a, b) in [
        (
            [low[0] - 0.1, low[1] - 0.1, low[2]],
            [low[0], high[1] + 0.1, high[2] + 2.0],
        ),
        (
            [high[0], low[1] - 0.1, low[2]],
            [high[0] + 0.1, high[1] + 0.1, high[2] + 2.0],
        ),
        (
            [low[0], low[1] - 0.1, low[2]],
            [high[0], low[1], high[2] + 2.0],
        ),
        (
            [low[0], high[1], low[2]],
            [high[0], high[1] + 0.1, high[2] + 2.0],
        ),
    ] {
        let base = mesh.vertices_m.len() as u32;
        for bits in 0..8 {
            mesh.vertices_m.push(std::array::from_fn(|axis| {
                if bits & (1 << axis) == 0 {
                    a[axis]
                } else {
                    b[axis]
                }
            }));
        }
        mesh.triangles.extend(
            [
                [0, 2, 1],
                [1, 2, 3],
                [4, 5, 6],
                [5, 7, 6],
                [0, 1, 4],
                [1, 5, 4],
                [2, 6, 3],
                [3, 6, 7],
                [0, 4, 2],
                [2, 4, 6],
                [1, 3, 5],
                [3, 7, 5],
            ]
            .map(|t| t.map(|i| i + base)),
        );
    }
    mesh
}

/// Hand the terrain to the field: the ground, fixtures and equipment as
/// the same triangles used to draw the CAD.
pub fn install_terrain(field: &mut WorldPhysics, terrain: &Terrain) -> Result<(), &'static str> {
    let mut query_groups = std::collections::BTreeSet::new();
    for (index, query) in &terrain.native_query_meshes {
        if !query_groups.insert(*index)
            || terrain.native_shapes.get(*index).is_none_or(Vec::is_empty)
            || query.triangles.is_empty()
        {
            return Err("query mesh must reference one nonempty fixed native group exactly once");
        }
    }
    if let Some(bounds) = terrain.projectile_bounds_m
        && (!bounds.iter().flatten().all(|v| v.is_finite())
            || (0..2).any(|i| bounds[0][i] >= bounds[1][i]))
    {
        return Err("invalid projectile bounds");
    }
    field.remove_catch_floor();
    if let Some(bounds) = terrain.projectile_bounds_m {
        field.projectile_bounds_m = Some(bounds);
    }
    if !terrain.boundary.triangles.is_empty() {
        field.add_boundary_mesh(
            terrain.boundary.vertices_m.clone(),
            terrain.boundary.triangles.clone(),
        )?;
    }
    let mut equipment_handles = vec![None; terrain.equipment.len()];
    for (index, mesh) in std::iter::once(terrain.collision_ground())
        .chain(std::iter::once(&terrain.fixtures))
        .chain(&terrain.equipment)
        .enumerate()
    {
        if mesh.triangles.is_empty() {
            continue;
        }
        let handle = field.add_static_mesh(mesh.vertices_m.clone(), mesh.triangles.clone())?;
        if index >= 2 {
            equipment_handles[index - 2] = Some(handle);
        }
    }
    for (index, shapes) in terrain.native_shapes.iter().enumerate() {
        if let Some((_, query)) = terrain
            .native_query_meshes
            .iter()
            .find(|(i, _)| *i == index)
        {
            field.add_static_shapes_with_queries(
                shapes,
                query.vertices_m.clone(),
                query.triangles.clone(),
            )?;
        } else {
            field.add_static_shapes(shapes)?;
        }
    }
    if let Some(package) = &terrain.native_terrain {
        package
            .install_in_world(field)
            .map_err(|_| "native terrain installation failed")?;
    }
    for part in &terrain.mechanisms {
        part.install(field)?;
    }
    for (index, team) in &terrain.endpoint_bakes {
        let handle = equipment_handles
            .get(*index)
            .copied()
            .flatten()
            .ok_or("missing endpoint fixed body")?;
        field.bake_translation_endpoints(handle, rm_sim_physics::motion::Mechanism::Base, *team)?;
    }
    Ok(())
}

/// Cook terrain through the shared installation path, retaining source motion
/// channels, native shapes and opt-in endpoint caches for insertion or reset.
pub fn prepare_terrain_geometry(terrain: &Terrain) -> Result<StaticGeometry, &'static str> {
    let mut physics = WorldPhysics::new(&[], CATCH_FLOOR_M);
    install_terrain(&mut physics, terrain)?;
    Ok(physics.static_geometry_snapshot(true))
}

/// Immutable cooked arena shared by independent physical sessions and resets.
///
/// Contains source scenery and legacy mechanism channels, but no robot, detector
/// target bank, team ownership policy, match authority or controller. Shapes
/// start in their source rest frame; callers supply targets and resolve mechanism
/// coordinates explicitly in the setup recipe. Preparation cooks meshes once.
pub struct PreparedArena {
    terrain: Terrain,
    geometry: StaticGeometry,
}

#[cfg(test)]
mod whole_bake_package_tests {
    use super::*;
    use crate::{cad_assets, collision_mesh, components};
    use rm_sim_physics::{Team, motion::MechanismState};

    #[test]
    #[ignore = "requires materialized bundled field GLBs"]
    fn bundled_core_native_contacts_and_mesh_queries_survive_restore() {
        let cad = cad_assets::load(&cad_assets::default_cad_assets()).unwrap();
        let terrain = load_terrain(&cad).unwrap();
        assert_eq!(
            terrain.native_shapes.iter().map(Vec::len).sum::<usize>(),
            54
        );
        assert_eq!(terrain.native_query_meshes.len(), 2);
        let model = components::load(
            &cad,
            &cad.tech_core,
            components::ComponentRole::Equipment,
            Some(&[]),
        )
        .unwrap();
        let mut physics = WorldPhysics::new(&[], 0.);
        physics.remove_catch_floor();
        let mut reference = WorldPhysics::new(&[], 0.);
        reference.remove_catch_floor();
        for placement in &cad.tech_core.placements {
            model.placed(placement).install(&mut physics).unwrap();
            let mut mesh = model.fixed.clone();
            mesh.append(model.query_mesh.as_ref().unwrap().clone());
            let mesh = mesh.placed(placement);
            reference
                .add_static_mesh(mesh.vertices_m, mesh.triangles)
                .unwrap();
        }
        let capture = physics.static_geometry_snapshot(true);
        let reference = reference.static_geometry_snapshot(true);
        let mut restored = WorldPhysics::new(&[], 0.);
        restored.try_insert_geometry(&capture).unwrap();
        let restored = restored.static_geometry_snapshot(true);
        let mut checked = 0;
        for placement in &cad.tech_core.placements {
            for (pose, shape) in model.placed(placement).native_shapes {
                let rm_sim_physics::model::ShapeDescription::Cylinder {
                    radius_m,
                    half_height_m,
                } = shape
                else {
                    panic!("expected native cylinder")
                };
                for (point, direction, length) in [
                    ([radius_m + 0.1, 0., 0.], [-1., 0., 0.], 2. * radius_m + 0.2),
                    (
                        [0., half_height_m + 0.1, 0.],
                        [0., -1., 0.],
                        2. * half_height_m + 0.2,
                    ),
                ] {
                    let origin = crate::math::add(
                        pose.translation_m,
                        crate::math::rotate(pose.rotation_wxyz, point),
                    );
                    let direction = crate::math::rotate(pose.rotation_wxyz, direction);
                    let expected = reference
                        .cast_ray(origin, direction, length)
                        .unwrap()
                        .map(|h| h.distance_m);
                    for scene in [&capture, &restored] {
                        let hit = scene
                            .cast_ray(origin, direction, length)
                            .unwrap()
                            .map(|h| h.distance_m);
                        assert_eq!(hit.is_some(), expected.is_some());
                        if let (Some(hit), Some(expected)) = (hit, expected) {
                            assert!(
                                (hit - expected).abs() < 1e-10,
                                "mesh lidar range changed: {hit} vs {expected}"
                            );
                        }
                    }
                    checked += 1;
                }
            }
        }
        assert_eq!(checked, 108);
    }

    #[test]
    #[ignore = "requires materialized bundled field GLBs"]
    fn bundled_whole_base_endpoints_match_reference_and_survive_restore() {
        let cad = cad_assets::load(&cad_assets::default_cad_assets()).unwrap();
        assert!(cad.base.bake_motion_endpoints);
        let terrain = load_terrain(&cad).unwrap();
        assert_eq!(terrain.endpoint_bakes.len(), 2);
        let prepared = PreparedArena::from_terrain(terrain).unwrap();
        // Cooking drops two degenerate faces; heightfield surfaces and boundary are included.
        assert_eq!(
            prepared.geometry().clone().into_triangles().1.len(),
            121_701
        );
        let motions = cad.rmuc_2027.as_ref().unwrap().motions(&cad, &cad.base);
        let model = components::load(
            &cad,
            &cad.base,
            components::ComponentRole::Equipment,
            Some(motions),
        )
        .unwrap();
        for placement in &cad.base.placements {
            let mut physics = WorldPhysics::new(&[], CATCH_FLOOR_M);
            physics.remove_catch_floor();
            let fixed = model.fixed.placed(placement);
            let handle = physics
                .add_static_mesh(fixed.vertices_m, fixed.triangles)
                .unwrap();
            let team = if placement.translation_m[0] >= 0. {
                Team::Red
            } else {
                Team::Blue
            };
            for part in model.mechanisms(&[*placement]) {
                part.install(&mut physics).unwrap();
            }
            physics
                .bake_translation_endpoints(handle, rm_sim_physics::motion::Mechanism::Base, team)
                .unwrap();
            for (name, fraction) in [("closed", 0.), ("open", 1.)] {
                let reference = collision_mesh::load_glb(
                    &cad.root.join(format!("collision-bakes/base-{name}.glb")),
                )
                .unwrap()
                .placed(placement);
                assert_eq!(reference.triangles.len(), 7_096);
                let mut state = MechanismState::default();
                state.base_open_fraction[team.index()] = fraction;
                physics.sync_mechanisms(&state);
                let captured = physics.static_geometry_snapshot(true);
                let captured_triangle_count = captured.clone().into_triangles().1.len();
                let mut restored = WorldPhysics::new(&[], CATCH_FLOOR_M);
                restored.try_insert_geometry(&captured).unwrap();
                restored.sync_mechanisms(&state);
                let restored = restored.static_geometry_snapshot(true);
                let mut reference_world = WorldPhysics::new(&[], CATCH_FLOOR_M);
                reference_world.remove_catch_floor();
                let mut low = [f64::INFINITY; 3];
                let mut high = [f64::NEG_INFINITY; 3];
                for point in &reference.vertices_m {
                    for i in 0..3 {
                        low[i] = low[i].min(point[i]);
                        high[i] = high[i].max(point[i]);
                    }
                }
                reference_world
                    .add_static_mesh(reference.vertices_m, reference.triangles)
                    .unwrap();
                let reference = reference_world.static_geometry_snapshot(true);
                assert_eq!(
                    captured_triangle_count,
                    reference.clone().into_triangles().1.len()
                );
                let center = std::array::from_fn::<_, 3, _>(|i| (low[i] + high[i]) / 2.);
                let radius = (0..3)
                    .map(|i| (high[i] - low[i]).powi(2))
                    .sum::<f64>()
                    .sqrt()
                    / 2.
                    + 0.1;
                for index in 0..96 {
                    let y = 1. - 2. * (index as f64 + 0.5) / 96.;
                    let theta = index as f64 * 2.399_963_229_728_653;
                    let direction = [
                        (1. - y * y).sqrt() * theta.cos(),
                        y,
                        (1. - y * y).sqrt() * theta.sin(),
                    ];
                    let origin = std::array::from_fn(|i| center[i] + direction[i] * radius);
                    let direction = direction.map(|x| -x);
                    let expected = reference.cast_ray(origin, direction, radius * 2.).unwrap();
                    for actual in [&captured, &restored] {
                        let hit = actual.cast_ray(origin, direction, radius * 2.).unwrap();
                        assert_eq!(hit.is_some(), expected.is_some());
                        if let (Some(a), Some(b)) = (hit, expected) {
                            assert!((a.distance_m - b.distance_m).abs() < 1e-5);
                        }
                    }
                }
            }
        }
    }
}
impl PreparedArena {
    /// Load verified package collision meshes and prepare their shared shapes.
    pub fn load(cad: &CadAssets) -> anyhow::Result<Self> {
        Self::from_terrain(load_terrain(cad)?).map_err(anyhow::Error::msg)
    }

    /// Cook caller-owned FLU terrain once, preserving mesh and insertion order.
    ///
    /// ```
    /// use rm_sim_assets::{arena::{PreparedArena, Terrain}, collision_mesh::CollisionMesh};
    /// use rm_sim_physics::{StepDuration, session::RobotSession};
    /// let terrain = Terrain {
    ///     boundary: Default::default(), projectile_bounds_m: None, mechanisms: vec![],
    ///     ground: CollisionMesh::default().into(), fixtures: Default::default(), equipment: vec![], endpoint_bakes: vec![], native_shapes: vec![], native_query_meshes: vec![], native_terrain: None, ground_collision: None,
    /// };
    /// let arena = PreparedArena::from_terrain(terrain)?;
    /// let recipe = arena.scenario(vec![], StepDuration::HZ_1000, |_| Ok(()))?;
    /// let mut session = RobotSession::new(recipe)?;
    /// session.step(10)?;
    /// session.reset()?;
    /// assert_eq!(session.stamp().tick, 0);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn from_terrain(terrain: Terrain) -> Result<Self, &'static str> {
        let geometry = prepare_terrain_geometry(&terrain)?;
        Ok(Self { terrain, geometry })
    }

    /// Source collision meshes, including the ground used for placement queries.
    pub fn terrain(&self) -> &Terrain {
        &self.terrain
    }

    /// Shared cooked geometry; cloning this record shares immutable shape buffers.
    pub fn geometry(&self) -> &StaticGeometry {
        &self.geometry
    }

    /// Define a repeatable standalone world with explicit targets and timing.
    ///
    /// The setup runs after arena insertion at construction and every reset.
    /// Add robots, initial mechanism coordinates and physical policies there.
    /// No rules or source detector targets are inferred from the arena scenery.
    pub fn scenario(
        &self,
        targets: Vec<TargetFace>,
        step_duration: StepDuration,
        setup: impl Fn(&mut WorldPhysics) -> Result<(), &'static str> + Send + Sync + 'static,
    ) -> Result<RobotScenario, &'static str> {
        let geometry = self.geometry.clone();
        RobotScenario::new(targets, CATCH_FLOOR_M, step_duration, move |physics| {
            physics.remove_catch_floor();
            physics.try_insert_geometry(&geometry)?;
            setup(physics)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rm_sim_physics::{Pose, Team, chassis::ChassisConfig, session::RobotSession};

    #[test]
    fn prepared_arena_preserves_geometry_bounds_and_reset() {
        let floor = CollisionMesh {
            vertices_m: vec![[-2., -1., 0.], [2., -1., 0.], [2., 1., 0.], [-2., 1., 0.]],
            triangles: vec![[0, 1, 2], [0, 2, 3]],
        };
        let bounds = arena_boundary_bounds(&floor, &CollisionMesh::default()).unwrap();
        let terrain = Terrain {
            native_terrain: None,
            ground_collision: None,
            boundary: arena_boundary(bounds),
            projectile_bounds_m: Some(bounds),
            mechanisms: vec![],
            ground: floor.into(),
            fixtures: CollisionMesh::default(),
            equipment: vec![],
            endpoint_bakes: vec![],
            native_shapes: vec![],
            native_query_meshes: vec![],
        };
        let mut direct = WorldPhysics::new(&[], CATCH_FLOOR_M);
        install_terrain(&mut direct, &terrain).unwrap();
        let prepared = PreparedArena::from_terrain(terrain).unwrap();
        assert_eq!(
            direct.static_geometry_snapshot(true).into_triangles(),
            prepared.geometry().clone().into_triangles()
        );
        let recipe = prepared
            .scenario(vec![], StepDuration::HZ_1000, |world| {
                world.add_chassis(Team::Red, ChassisConfig::default(), Pose::at([0., 0., 0.3]))?;
                Ok(())
            })
            .unwrap();
        let mut session = RobotSession::new(recipe).unwrap();
        session.step(128).unwrap();
        let first = session.robot_truth(0).unwrap();
        assert_eq!(session.ground_truth().projectile_bounds_m, Some(bounds));
        session.reset().unwrap();
        session.step(128).unwrap();
        assert_eq!(first, session.robot_truth(0).unwrap());
    }
}
