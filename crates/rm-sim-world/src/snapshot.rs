// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Observer and continuation records, independent of live field coordination.
use crate::{
    ArmorHit, ArmorTarget, BaseSnapshot, ChassisSnapshot, OutpostSnapshot, Pose,
    ProjectileSnapshot, RefereeSnapshot, RuneSnapshot, StepDuration, assembly, energy_unit,
    gameplay, projectile, rmuc_2027,
};
use crate::{Field, FieldError, StaticGeometry, TargetFrames, WorldPhysics, base, referee};
use serde::{Deserialize, Serialize};

/// One armor module pose on a rotating mechanism, keyed by face index.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ArmorSnapshot {
    /// Face index in the mechanism's own order.
    pub id: u32,
    /// Face centre in world FLU metres; +x is the outward scoring normal.
    pub pose: Pose,
}

/// The rules' own bookkeeping, beyond what an observer needs: rune activation
/// progress and its seeded stream, outpost rotor stops, the referee's schedule
/// and configuration, and the physics identity counters. [`crate::Field::restore`]
/// needs it; a snapshot that lost it can still be drawn.
///
/// What it deliberately cannot carry is solver state. Rapier's contact
/// manifolds, warm starts and each ball's touched-armor set are rebuilt from
/// the restored bodies, so a restored field's first ticks differ from the
/// original's by the solver's warm-start residual, and a ball already resting
/// against armor registers that touch once more.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FieldRestore {
    /// Exact simulation step, preserved when restoring the authoritative clock.
    #[serde(default)]
    pub step_duration: StepDuration,
    /// Restorable automatic assembly jobs and validated remaining trajectories.
    #[serde(default)]
    pub assemblies: Vec<assembly::Run>,
    /// Persistent articulated arm authority for deployed source Heavy robots.
    #[serde(default)]
    pub arms: Vec<assembly::ArmState>,
    /// Persistent physical Energy Units, including loose and socket-seated objects.
    #[serde(default)]
    pub energy_units: Vec<energy_unit::EnergyUnit>,
    /// Complete RMUC 2027 authority and geometry, absent for bare physics.
    pub rmuc_2027: Option<rmuc_2027::State>,
    /// Complete distinct RMUL 2027 authority; never combined with RMUC.
    pub rmul_2027: Option<gameplay::rmul_2027::Match>,
    /// Next id [`crate::Field::add_chassis`] would hand out; ids are never reused.
    pub next_chassis_id: u32,
    /// Per-module time of the last detected strike, oldest kept, in target
    /// order, so the detection intervals of section 5.1.1 survive a restore.
    pub last_detection_ns: Vec<(ArmorTarget, u64)>,
    /// The projectile restitution, flight limit and retirement rule the field
    /// runs, so a restored field spends and retires balls exactly as the field
    /// it was captured from does.
    pub projectile_policy: projectile::ProjectilePolicy,
}

/// One authoritative tick of the whole field, as an observer or a peer
/// receives it. `Field::restore` rebuilds a field from one, so it carries the
/// rules' hidden state as well as what a viewer draws.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FieldSnapshot {
    /// Native 2027 view, retained even when a drawing-only decoder drops restore.
    /// The checkpoint codec derives this from the matching authority record.
    pub rmuc_2027: Option<rmuc_2027::State>,
    /// Complete distinct RMUL 2027 authority; never combined with RMUC.
    pub rmul_2027: Option<gameplay::rmul_2027::Match>,
    /// Every base's fitted geometry, HP and shield.
    pub bases: Vec<BaseSnapshot>,
    /// Ticks elapsed since the field was built.
    pub tick: u64,
    /// `tick` times the restore's step duration, the field clock in nanoseconds.
    pub time_ns: u64,
    /// One frame per rune, in referee index order.
    pub runes: Vec<RuneSnapshot>,
    /// One frame per outpost, in referee index order.
    pub outposts: Vec<OutpostSnapshot>,
    /// Balls still in flight at this tick.
    pub projectiles: Vec<ProjectileSnapshot>,
    /// Every chassis on the field, in id order.
    pub chassis: Vec<ChassisSnapshot>,
    /// Armor contacts from the last second, oldest first.
    pub hits: Vec<ArmorHit>,
    /// Balls launched since the field was built.
    pub shots_fired: u64,
    /// Contacts that passed every detection condition, rejected ones excluded.
    pub hits_detected: u64,
    /// The match state, when the field has a referee.
    pub referee: Option<RefereeSnapshot>,
    /// Hidden rule state, for [`crate::Field::restore`]. Present on every snapshot a
    /// field takes; a decoder that drops it gives up restoring, not drawing.
    pub restore: Option<FieldRestore>,
}

impl Field {
    /// A frame of everything an observer needs at the current tick: bases,
    /// runes, outposts, projectiles, chassis, recent hits and the referee.
    /// Taking one never advances the clock, and it always carries the hidden
    /// rule state a `restore` needs.
    ///
    /// ```rust
    /// use rm_sim_world::{Field, FieldConfig, tick_ns};
    ///
    /// let mut field = Field::new(&FieldConfig::default()).unwrap();
    /// field.step(250).unwrap();
    /// let snapshot = field.snapshot();
    /// assert_eq!(snapshot.tick, 250);
    /// assert_eq!(snapshot.time_ns, 250 * tick_ns());
    /// assert!(snapshot.runes.is_empty());
    /// assert!(snapshot.restore.is_some());
    /// ```
    pub fn snapshot(&self) -> FieldSnapshot {
        let time_ns = self.time_ns();
        FieldSnapshot {
            rmuc_2027: self.rmuc_2027.clone(),
            rmul_2027: self.rmul_2027.clone(),
            bases: Vec::new(),
            tick: self.tick,
            time_ns,
            runes: Vec::new(),
            outposts: Vec::new(),
            projectiles: self.physics.snapshot(),
            chassis: self.physics.chassis_snapshots(),
            hits: self.hits.clone(),
            shots_fired: self.physics.launched(),
            hits_detected: self.hits_detected,
            referee: None,
            restore: Some(FieldRestore {
                step_duration: self.step_duration(),
                assemblies: self.assemblies.clone(),
                arms: self.arms.clone(),
                energy_units: self.energy_units(),
                rmuc_2027: self.rmuc_2027.clone(),
                rmul_2027: self.rmul_2027.clone(),
                next_chassis_id: self.physics.next_chassis_id(),
                projectile_policy: self.physics.projectile_policy(),
                last_detection_ns: {
                    // A hash map has no order; sort so equal fields snapshot equal.
                    let mut detections: Vec<_> = self
                        .last_detection_ns
                        .iter()
                        .map(|(target, time_ns)| (*target, *time_ns))
                        .collect();
                    detections.sort_unstable();
                    detections
                },
            }),
        }
    }
    /// Rebuild the field the snapshot was taken from, sharing `geometry`'s
    /// collision shapes instead of reloading any CAD, and standing at the
    /// snapshot's own tick. Stepping the result applies the same rules as the
    /// original: this is one field, not a reduced presentation world.
    ///
    /// What is restored: every rune, outpost and the referee exactly (from
    /// [`FieldRestore`]); every chassis with its pose, velocities, held aim,
    /// command, defeat and wheel spin; every projectile with its identity,
    /// age, position, velocity and spin; the armor detection intervals, the
    /// recent hits and the shot and detection counters.
    ///
    /// What cannot be restored is solver state, as [`FieldRestore`] records:
    /// contact manifolds and warm starts are rebuilt, and a projectile's set
    /// of already-touched armor colliders starts empty. Wheel contacts and
    /// tyre targets are recomputed on the first tick, so they need no
    /// checkpoint.
    ///
    /// ```rust
    /// use rm_sim_world::{Field, FieldConfig};
    ///
    /// let mut field = Field::new(&FieldConfig::default()).unwrap();
    /// field.step(500).unwrap();
    /// let checkpoint = field.snapshot();
    /// let geometry = field.static_geometry_snapshot();
    ///
    /// let mut restored = Field::restore(&checkpoint, &geometry, 0.0).unwrap();
    /// assert_eq!(restored.tick(), checkpoint.tick);
    /// // The restored field keeps stepping under the same rules as the original.
    /// field.step(10).unwrap();
    /// restored.step(10).unwrap();
    /// assert_eq!(restored.snapshot().runes, field.snapshot().runes);
    /// ```
    pub fn restore(
        snapshot: &FieldSnapshot,
        geometry: &StaticGeometry,
        floor_height_m: f64,
    ) -> Result<Self, FieldError> {
        // The explicit native authority chooses restore admission; never infer
        // a season from a team, HP value or CAD position.
        if snapshot.referee.is_some()
            || !snapshot.runes.is_empty()
            || !snapshot.outposts.is_empty()
            || !snapshot.bases.is_empty()
        {
            return Err(FieldError::Restore(
                "2026 runtime/checkpoints removed; use rm2026-final",
            ));
        }
        let incoming_2027 = snapshot.restore.as_ref().and_then(|r| r.rmuc_2027.as_ref());
        if let Some(state) = incoming_2027 {
            state.config.validate().map_err(FieldError::Restore)?;
            state.game.validate().map_err(FieldError::Restore)?;
            if !snapshot.bases.is_empty()
                || !snapshot.outposts.is_empty()
                || !snapshot.runes.is_empty()
                || snapshot.referee.is_some()
            {
                return Err(FieldError::Restore("mixed season checkpoint"));
            }
        }
        let incoming_ul = snapshot.restore.as_ref().and_then(|r| r.rmul_2027.as_ref());
        if incoming_ul.is_some() && incoming_2027.is_some() {
            return Err(FieldError::Restore("mixed competition checkpoint"));
        }
        if snapshot.rmul_2027.as_ref() != incoming_ul {
            return Err(FieldError::Restore("RMUL view differs from its authority"));
        }
        if let Some(game) = incoming_ul {
            game.validate().map_err(FieldError::Restore)?;
            if !snapshot.chassis.iter().all(|c| {
                game.robots
                    .iter()
                    .any(|r| r.id == c.id && r.team == referee::game_team(c.team))
            }) || game.robots.len() != snapshot.chassis.len()
            {
                return Err(FieldError::Restore(
                    "RMUL roster differs from physical chassis",
                ));
            }
        }
        if snapshot.rmuc_2027.as_ref() != incoming_2027 {
            return Err(FieldError::Restore("2027 view differs from its authority"));
        }
        let rules = snapshot
            .restore
            .as_ref()
            .ok_or(FieldError::Restore("the snapshot carries no rule state"))?;
        if incoming_ul.is_some()
            && (!rules.assemblies.is_empty()
                || !rules.arms.is_empty()
                || !rules.energy_units.is_empty())
        {
            return Err(FieldError::Restore(
                "RMUL checkpoint contains RMUC assembly state",
            ));
        }
        if rules.arms.len() > 64 {
            return Err(FieldError::Restore("too many arm owners"));
        }
        let mut arm_ids = std::collections::BTreeSet::new();
        for arm in &rules.arms {
            arm.validate().map_err(FieldError::Restore)?;
            if !arm_ids.insert(arm.owner) || !snapshot.chassis.iter().any(|c| c.id == arm.owner) {
                return Err(FieldError::Restore("missing or duplicate arm owner"));
            }
        }
        if rules.assemblies.len() > 2 {
            return Err(FieldError::Restore("too many assembly jobs"));
        }
        if rules.assemblies.len() == 2
            && (rules.assemblies[0].owner == rules.assemblies[1].owner
                || rules.assemblies[0].team == rules.assemblies[1].team)
        {
            return Err(FieldError::Restore("duplicate assembly owner or station"));
        }
        for run in &rules.assemblies {
            run.validate_with_step_duration(rules.step_duration)
                .map_err(FieldError::Restore)?;
            if run.active()
                && !snapshot.chassis.iter().any(|c| {
                    c.id == run.owner
                        && c.team == run.team
                        && c.placement_revision == run.placement_revision
                })
            {
                return Err(FieldError::Restore("missing assembly owner"));
            }
        }
        if !floor_height_m.is_finite() {
            return Err(FieldError::Restore("the floor height must be finite"));
        }
        if snapshot.tick.checked_mul(rules.step_duration.as_ns()) != Some(snapshot.time_ns) {
            return Err(FieldError::Restore("the snapshot tick and time disagree"));
        }
        for base in &snapshot.bases {
            base.config.validate().map_err(FieldError::Restore)?;
            if base.hp > base::INITIAL_HP || base.shield_hp > base::INITIAL_HP {
                return Err(FieldError::Restore("base HP or shield out of range"));
            }
        }
        let time_ns = snapshot.time_ns;
        let mut faces = Vec::new();
        Self::write_target_faces(rules.rmuc_2027.as_ref(), &mut faces);
        let mut physics =
            WorldPhysics::with_step_duration(&faces, floor_height_m, rules.step_duration);
        physics
            .set_projectile_policy(rules.projectile_policy)
            .map_err(FieldError::Restore)?;
        physics.insert_geometry(geometry);
        let mut field = Self {
            assemblies: rules.assemblies.clone(),
            arms: rules.arms.clone(),
            energy_units: rules.energy_units.clone(),
            rmuc_2027: rules.rmuc_2027.clone(),
            rmul_2027: rules.rmul_2027.clone(),
            floor_height_m,
            tick: snapshot.tick,
            physics,
            rmuc_terrain_cards: Vec::new(),
            target_frames: TargetFrames::new(faces),
            hits: snapshot.hits.clone(),
            hits_detected: snapshot.hits_detected,
            last_detection_ns: rules.last_detection_ns.iter().copied().collect(),
        };
        for chassis in &snapshot.chassis {
            field
                .physics
                .restore_chassis(chassis)
                .map_err(FieldError::Chassis)?;
        }
        field.restore_energy_units()?;
        for projectile in &snapshot.projectiles {
            field
                .physics
                .restore_projectile(projectile)
                .map_err(FieldError::Shot)?;
        }
        field
            .physics
            .restore_identity(rules.next_chassis_id, snapshot.shots_fired);
        field.physics.mark_synced(time_ns);
        field.sync_mechanisms();
        field.sync_defeats();
        Ok(field)
    }
}
