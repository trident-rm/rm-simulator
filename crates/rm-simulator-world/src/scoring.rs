// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Contact detection and gameplay consequences, applied in physical contact order.
use super::*;

/// A contact either passes every detection condition or has one rejection reason.
enum DetectionOutcome {
    Rejected(Rejection),
    Detected { offset_m: [f64; 2] },
}

impl Field {
    fn detect(&self, time_ns: u64, contact: &Contact) -> DetectionOutcome {
        let is_rune = matches!(contact.target, ArmorTarget::Rune { .. });
        let offset = if let Some(detector) = self
            .rmuc_2027
            .as_ref()
            .and_then(|s| s.detector(contact.target))
        {
            let [x, y, z] = contact.local_m;
            ((-0.004..=0.012).contains(&x)
                && y.abs() <= detector.half_m[0]
                && z.abs() <= detector.half_m[1])
                .then_some([y, z])
        } else {
            projectile::scoring_offset(contact.target, contact.local_m)
        };
        let Some(offset) = offset else {
            return DetectionOutcome::Rejected(Rejection::OutsideTarget);
        };
        if is_rune && contact.caliber != Caliber::Mm17 {
            return DetectionOutcome::Rejected(Rejection::Caliber);
        }
        if let (ArmorTarget::Rune { rune, .. }, Some(native)) = (contact.target, &self.rmuc_2027) {
            let ring = gameplay::rmuc_2027::rune::Rune::ring(offset[0].hypot(offset[1]));
            if ring.is_none_or(|r| r < native.game.runes[rune as usize].minimum_ring()) {
                return DetectionOutcome::Rejected(Rejection::DisabledRing);
            }
        }
        if contact.normal_speed_m_s <= contact.caliber.armor_detection_speed_m_s() {
            return DetectionOutcome::Rejected(Rejection::NormalSpeed);
        }
        if let Some(last) = self.last_detection_ns.get(&contact.target)
            && time_ns.saturating_sub(*last) < contact.caliber.detection_interval_ns()
        {
            return DetectionOutcome::Rejected(Rejection::DetectionInterval);
        }
        if let (ArmorTarget::Rune { rune, .. }, Some(referee)) = (contact.target, &self.referee)
            && referee::ring_of(offset) < referee.rune_min_ring(rune)
        {
            return DetectionOutcome::Rejected(Rejection::DisabledRing);
        }
        DetectionOutcome::Detected { offset_m: offset }
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
            DetectionOutcome::Rejected(reason) => {
                hit.rejection = Some(reason);
                return Ok(hit);
            }
            DetectionOutcome::Detected { offset_m } => offset_m,
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
            let target = match contact.target {
                ArmorTarget::Base { base, .. } => {
                    Some(gameplay::Target::Base(gameplay::Team::BOTH[base as usize]))
                }
                ArmorTarget::Outpost { outpost, .. } => Some(gameplay::Target::Outpost(
                    gameplay::Team::BOTH[outpost as usize],
                )),
                ArmorTarget::Chassis { chassis, .. } => Some(gameplay::Target::Robot(chassis)),
                _ => None,
            };
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
        match contact.target {
            ArmorTarget::Base { base, plate } => {
                let team = self.bases[base as usize].config.team;
                hit.damage = match &mut self.referee {
                    // The gameplay engine applies outpost protection, buffs,
                    // the Table 5-2 values and the section 5.5.1 centre square.
                    // The seventh (dart) plate accepting projectiles is a
                    // training override without the centre bonus.
                    Some(referee) => {
                        let applied =
                            referee.projectile_hit(rm_simulator_gameplay::ProjectileHit {
                                target: rm_simulator_gameplay::Target::Base(referee::game_team(
                                    team,
                                )),
                                caliber: referee::game_caliber(contact.caliber),
                                shooter: contact.shooter,
                                upper_front: plate == 3,
                                critical: plate != 6 && projectile::in_centre_square(offset),
                            });
                        applied.hp + applied.shield
                    }
                    None => self.bases[base as usize].damage(base::damage(
                        contact.caliber,
                        plate,
                        offset,
                    )),
                };
                self.sync_structures(time_ns);
            }
            ArmorTarget::Outpost { outpost, .. } => {
                hit.damage = match &mut self.referee {
                    Some(referee) => {
                        let team = referee.config().outpost_teams[outpost as usize];
                        referee
                            .projectile_hit(rm_simulator_gameplay::ProjectileHit {
                                target: rm_simulator_gameplay::Target::Outpost(referee::game_team(
                                    team,
                                )),
                                caliber: referee::game_caliber(contact.caliber),
                                shooter: contact.shooter,
                                upper_front: false,
                                critical: projectile::in_centre_square(offset),
                            })
                            .hp
                    }
                    None => self.outposts[outpost as usize]
                        .damage(time_ns, projectile::outpost_damage(contact.caliber, offset)),
                };
                self.sync_structures(time_ns);
            }
            ArmorTarget::Rune { rune, blade } => {
                let outcome: HitOutcome = self.runes[rune as usize].hit(time_ns, blade)?;
                hit.rune_outcome = Some(outcome);
            }
            ArmorTarget::Chassis { chassis, .. } => {
                // Table 5-2 robot damage; the gameplay engine keeps the HP, so
                // without a referee the strike is only reported.
                hit.damage = match &mut self.referee {
                    Some(referee) => {
                        referee
                            .projectile_hit(rm_simulator_gameplay::ProjectileHit {
                                target: rm_simulator_gameplay::Target::Robot(chassis),
                                caliber: referee::game_caliber(contact.caliber),
                                shooter: contact.shooter,
                                upper_front: false,
                                critical: false,
                            })
                            .hp
                    }
                    None => contact.caliber.robot_damage(),
                };
            }
        }
        Ok(hit)
    }
}
