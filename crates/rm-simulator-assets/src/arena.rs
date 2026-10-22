// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Complete verified arena collision preparation without a match, server or renderer.
use crate::{
    cad_assets::CadAssets,
    collision_mesh::{self, CollisionMesh, GroundMesh},
};
use anyhow::ensure;
use rm_simulator_physics::{
    StaticGeometry, StepDuration, TargetFace, WorldPhysics, session::RobotScenario,
};
/// Catch plane used while constructing CAD scenes; installed arenas remove it.
/// This simulator setting is below the source crowned slab.
pub const CATCH_FLOOR_M: f64 = -0.45;
const FIXED_NODE: &str = "static";
/// Everything rigid the chassis and projectiles collide with, in FLU metres.
pub struct Terrain {
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
}
/// Shared equipment composition records and semantic channel classification.
pub use crate::components::{MechanismMesh, mechanism_joint};

impl Terrain {
    /// One line for the start-up log.
    pub fn describe(&self) -> String {
        format!(
            "field collision: {} ground triangles, {} fixture triangles, {} equipment triangles (source tessellations or CAD visual meshes)",
            self.ground.triangles.len(),
            self.fixtures.triangles.len() + self.boundary.triangles.len(),
            self.equipment
                .iter()
                .map(|m| m.triangles.len())
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
    let mut mechanisms = Vec::new();
    let mut ground =
        collision_mesh::load_glb(&cad.floor.physics_file(&cad.root))?.into_flu(cad.floor_top_cad_m);
    let scenery = collision_mesh::load_glb(&cad.arena_static.physics_file(&cad.root))?
        .into_flu(cad.floor_top_cad_m);
    let projectile_bounds_m = arena_boundary_bounds(&ground, &scenery)?;
    let boundary = arena_boundary(projectile_bounds_m);
    ground.append(scenery);
    for asset in &cad.static_assets {
        let model = load_component(cad, asset, crate::components::ComponentRole::Equipment)?;
        mechanisms.extend(model.mechanisms(&asset.placements));
        for placement in &asset.placements {
            ground.append(model.fixed.placed(placement));
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
            !model.fixed.triangles.is_empty(),
            "{name} visual mesh has no `{FIXED_NODE}` node"
        );
        mechanisms.extend(model.mechanisms(&asset.placements));
        for placement in &asset.placements {
            fixtures.append(model.fixed.placed(placement));
        }
    }
    let mut equipment = Vec::new();
    for asset in [&cad.base, &cad.tech_core] {
        let model = load_component(cad, asset, crate::components::ComponentRole::Equipment)?;
        mechanisms.extend(model.mechanisms(&asset.placements));
        for placement in &asset.placements {
            equipment.push(model.fixed.placed(placement));
        }
    }
    Ok(Terrain {
        boundary,
        projectile_bounds_m: Some(projectile_bounds_m),
        mechanisms,
        ground: ground.into(),
        fixtures,
        equipment,
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
    for mesh in std::iter::once(&*terrain.ground)
        .chain(std::iter::once(&terrain.fixtures))
        .chain(&terrain.equipment)
        .filter(|mesh| !mesh.triangles.is_empty())
    {
        field.add_static_mesh(mesh.vertices_m.clone(), mesh.triangles.clone())?;
    }
    for part in &terrain.mechanisms {
        match part.motion {
            rm_simulator_physics::motion::MotionFrame::Translation { offsets_m } => field
                .add_mechanism_mesh(
                    part.mesh.vertices_m.clone(),
                    part.mesh.triangles.clone(),
                    part.mechanism,
                    part.team,
                    offsets_m,
                )?,
            rm_simulator_physics::motion::MotionFrame::Rotation { pivot_m, axis } => field
                .add_rotating_mesh(
                    part.mesh.vertices_m.clone(),
                    part.mesh.triangles.clone(),
                    part.mechanism,
                    part.team,
                    pivot_m,
                    axis,
                )?,
        }
    }
    Ok(())
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
impl PreparedArena {
    /// Load verified package collision meshes and prepare their shared shapes.
    pub fn load(cad: &CadAssets) -> anyhow::Result<Self> {
        Self::from_terrain(load_terrain(cad)?).map_err(anyhow::Error::msg)
    }

    /// Cook caller-owned FLU terrain once, preserving mesh and insertion order.
    ///
    /// ```
    /// use rm_simulator_assets::{arena::{PreparedArena, Terrain}, collision_mesh::CollisionMesh};
    /// use rm_simulator_physics::{StepDuration, session::RobotSession};
    /// let terrain = Terrain {
    ///     boundary: Default::default(), projectile_bounds_m: None, mechanisms: vec![],
    ///     ground: CollisionMesh::default().into(), fixtures: Default::default(), equipment: vec![],
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
        let mut physics = WorldPhysics::new(&[], CATCH_FLOOR_M);
        install_terrain(&mut physics, &terrain)?;
        let geometry = physics.static_geometry_snapshot(true);
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
    use rm_simulator_physics::{Pose, Team, chassis::ChassisConfig, session::RobotSession};

    #[test]
    fn prepared_arena_preserves_geometry_bounds_and_reset() {
        let floor = CollisionMesh {
            vertices_m: vec![[-2., -1., 0.], [2., -1., 0.], [2., 1., 0.], [-2., 1., 0.]],
            triangles: vec![[0, 1, 2], [0, 2, 3]],
        };
        let bounds = arena_boundary_bounds(&floor, &CollisionMesh::default()).unwrap();
        let terrain = Terrain {
            boundary: arena_boundary(bounds),
            projectile_bounds_m: Some(bounds),
            mechanisms: vec![],
            ground: floor.into(),
            fixtures: CollisionMesh::default(),
            equipment: vec![],
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
