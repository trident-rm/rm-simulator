// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Contact detection and gameplay consequences, applied in physical contact order.
use super::*;

impl Field {
    fn detect(&self, time_ns: u64, contact: &Contact) -> Result<[f64; 2], Rejection> {
        crate::rule_adapters::DetectionContext {
            detector: self
                .rmuc_2027
                .as_ref()
                .and_then(|s| s.detector(contact.target)),
            minimum_rune_ring: if let (ArmorTarget::Rune { rune, .. }, Some(native)) =
                (contact.target, &self.rmuc_2027)
            {
                native
                    .game
                    .runes
                    .get(rune as usize)
                    .map(|r| r.minimum_ring())
            } else {
                None
            },
            last_detection_ns: self.last_detection_ns.get(&contact.target).copied(),
        }
        .detect(time_ns, contact)
    }
    /// Apply the rule manual's detection conditions to one raw contact.
    pub(super) fn score(&mut self, time_ns: u64, contact: Contact) -> Result<ArmorHit, FieldError> {
        let mut hit = ArmorHit {
            time_ns,
            projectile: contact.projectile,
            shooter: contact.shooter,
            caliber: contact.caliber,
            target: contact.target,
            position_m: contact.position_m,
            local_offset_m: [contact.local_m[1], contact.local_m[2]],
            normal_speed_m_s: contact.normal_speed_m_s,
            detected: false,
            rejection: None,
            rune_outcome: None,
            damage: 0,
        };
        let offset = match self.detect(time_ns, &contact) {
            Err(reason) => {
                hit.rejection = Some(reason);
                return Ok(hit);
            }
            Ok(offset_m) => offset_m,
        };
        hit.local_offset_m = offset;
        self.last_detection_ns.insert(contact.target, time_ns);
        hit.detected = true;
        self.hits_detected += 1;
        if let Some(state) = &mut self.rmuc_2027 {
            if let ArmorTarget::Rune { rune, blade } = contact.target {
                let ring =
                    gameplay::rmuc_2027::rune::Rune::ring(offset[0].hypot(offset[1])).unwrap_or(1);
                let previous = &state.game.runes[rune as usize];
                let expected = previous.targets.iter().position(|&v| v).unwrap_or(0) as u32;
                let group = u32::from(previous.groups);
                let bonus = previous.bonus;
                let big = previous.stage == gameplay::rmuc_2027::rune::Stage::Big;
                let outcome =
                    state
                        .game
                        .hit_rune(gameplay::Team::BOTH[rune as usize], blade as u8, ring);
                use gameplay::rmuc_2027::rune::Hit;
                hit.rune_outcome = Some(match outcome {
                    Hit::Ignored => HitOutcome::Inactive,
                    Hit::Reset => HitOutcome::WrongBlade { blade, expected },
                    Hit::Completed => HitOutcome::Activated { blade },
                    Hit::Accepted if big => HitOutcome::GroupHit {
                        blade,
                        group,
                        bonus,
                    },
                    Hit::Accepted => HitOutcome::Accepted {
                        blade,
                        next_blade: state.game.runes[rune as usize]
                            .targets
                            .iter()
                            .position(|&v| v)
                            .unwrap_or(0) as u32,
                    },
                });
            }
            let target = crate::rule_adapters::combat_target(contact.target);
            if let Some(target) = target
                && state.game.phase == rmuc_2027::match_rules::Phase::Running
            {
                let victim = match target {
                    gameplay::Target::Base(t) => Some(t),
                    gameplay::Target::Outpost(t) => {
                        Some(state.game.control.snapshot().outposts[t.index()].owner)
                    }
                    gameplay::Target::Robot(id) => state
                        .game
                        .robots
                        .iter()
                        .find(|r| r.id == id)
                        .map(|r| r.team),
                };
                if let Some(victim) = victim {
                    let half = state
                        .detector(contact.target)
                        .map_or(projectile::OUTPOST_TARGET_HALF_M, |d| d.half_m);
                    let size_um = half.map(|v| (v * 2_000_000.).round() as u32);
                    let position_um = std::array::from_fn(|i| {
                        ((offset[i] + half[i]) * 1_000_000.)
                            .round()
                            .clamp(0., f64::from(size_um[i])) as u32
                    });
                    let attacker = contact.shooter.and_then(|id| {
                        state
                            .game
                            .robots
                            .iter()
                            .find(|r| r.id == id)
                            .map(|r| r.team)
                    });
                    let plate = match contact.target {
                        ArmorTarget::Base { plate, .. } | ArmorTarget::Chassis { plate, .. } => {
                            plate
                        }
                        ArmorTarget::Outpost { face, .. } => face,
                        _ => 0,
                    };
                    hit.damage = state.game.armor_hit(
                        gameplay::rmuc_2027::adaptive::Impact {
                            time_ns: state.game.elapsed_ns(),
                            attacker,
                            victim,
                            target,
                            plate,
                            position_um,
                            size_um,
                            region: gameplay::rmuc_2027::adaptive::Region::Default,
                        },
                        contact.shooter,
                        referee::game_caliber(contact.caliber),
                    );
                }
            }
            self.sync_defeats();
            return Ok(hit);
        }
        if let (Some(game), ArmorTarget::Chassis { chassis, .. }) =
            (&mut self.rmul_2027, contact.target)
        {
            hit.damage = game.damage(contact.shooter, chassis, contact.caliber == Caliber::Mm42);
            self.sync_defeats();
        }
        // Bare physical worlds report contacts without a season damage engine.
        Ok(hit)
    }
}
