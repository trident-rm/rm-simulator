// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Geometry-to-rule adapters shared by Field and standalone experiments.
//! These borrow layouts and authority; they own no clock, HP or second scheduler.
use crate::{
    ArmorTarget, Caliber, Rejection, gameplay,
    projectile::{self, Contact},
    rmuc_2027::{Config, Detector, Point, TerrainZone, Zone},
};
use gameplay::rmuc_2027::{
    match_rules::Match,
    terrain::{Card, Course},
};

/// Borrowed RFID layout, which may contain only the features under test.
/// A full pair of Bases/Outposts is not required to construct this adapter.
///
/// ```
/// use rm_simulator_world::{rule_adapters::RfidLayout,
///     gameplay::rmuc_2027::terrain::Terrain};
/// let layout = RfidLayout::default();
/// let mut cards = Vec::new();
/// layout.terrain_cards(None, &mut cards);
/// let mut terrain = Terrain::default();
/// assert_eq!(terrain.observe(0, &cards), None);
/// ```
#[derive(Clone, Copy, Debug, Default)]
pub struct RfidLayout<'a> {
    /// Control-point antenna footprints.
    pub control: &'a [Zone],
    /// Team supply antenna footprints.
    pub supply: &'a [Zone],
    /// Team Vantage service antenna footprints.
    pub vantage: &'a [Zone],
    /// Raw traversal-card antenna footprints, without grace or buff state.
    pub terrain: &'a [TerrainZone],
}
impl<'a> From<&'a Config> for RfidLayout<'a> {
    fn from(config: &'a Config) -> Self {
        Self {
            control: &config.zones,
            supply: &config.supply_zones,
            vantage: &config.vantage_zones,
            terrain: &config.terrain_zones,
        }
    }
}
impl RfidLayout<'_> {
    /// Replace reusable raw card storage from one antenna/body centre in FLU metres.
    /// `None` reports no contact. Coalesces overlapping upper Highland and control
    /// roles exactly as the live adapter does. No gameplay state is modified.
    /// Feed this to the pure `terrain::Terrain::observe` for a traversal-only test.
    pub fn terrain_cards(&self, position_m: Option<[f64; 3]>, cards: &mut Vec<Card>) {
        cards.clear();
        cards.extend(
            self.terrain
                .iter()
                .filter(|z| position_m.is_some_and(|p| z.zone.contains(p)))
                .map(|z| z.card),
        );
        for z in self.control {
            let shared = matches!(z.point, Point::CentralHighland | Point::Outpost)
                && cards.iter().any(|c| {
                    c.side == crate::referee::game_team(z.side)
                        && c.course == Some(Course::Highland)
                        && c.pad == 1
                });
            if !shared && position_m.is_some_and(|p| z.contains(p)) {
                cards.push(Card {
                    side: crate::referee::game_team(z.side),
                    course: None,
                    index: match z.point {
                        Point::Base => 0,
                        Point::Fortress => 1,
                        Point::Outpost => 2,
                        Point::CentralHighland => 3,
                    },
                    pad: 0,
                });
            }
        }
        for (index, zones) in [(4, self.supply), (5, self.vantage)] {
            for z in zones {
                if position_m.is_some_and(|p| z.contains(p)) {
                    cards.push(Card {
                        side: crate::referee::game_team(z.side),
                        course: None,
                        index,
                        pad: 0,
                    });
                }
            }
        }
    }

    /// Submit observations to an existing native authority without advancing it.
    /// Preserves live ordering: all traversal evidence, then all control contacts,
    /// then own-team services. The callback must return coherent positions throughout
    /// the call. Storage is reused; roster membership must not change during calls.
    pub fn observe(
        &self,
        game: &mut Match,
        mut position: impl FnMut(u32) -> Option<[f64; 3]>,
        cards: &mut Vec<Card>,
    ) {
        for index in 0..game.robots.len() {
            let id = game.robots[index].id;
            self.terrain_cards(position(id), cards);
            game.observe_terrain(id, cards);
        }
        for index in 0..game.robots.len() {
            let id = game.robots[index].id;
            let p = position(id);
            for z in self.control {
                game.observe(
                    id,
                    crate::referee::game_team(z.side),
                    z.point,
                    p.is_some_and(|p| z.contains(p)),
                );
            }
        }
        for index in 0..game.robots.len() {
            let (id, team) = {
                let r = &game.robots[index];
                (r.id, r.team)
            };
            let p = position(id);
            for (supply, zones) in [(true, self.supply), (false, self.vantage)] {
                let present = zones.iter().any(|z| {
                    crate::referee::game_team(z.side) == team && p.is_some_and(|p| z.contains(p))
                });
                game.observe_service(id, supply, present);
            }
        }
    }
}

/// Caller-owned evidence needed to detect one raw armor contact.
/// Cadence memory remains owned/checkpointed by the caller; a successful result
/// alone does not update that memory or apply damage.
///
/// ```
/// use rm_simulator_world::{ArmorTarget, Caliber, rule_adapters::DetectionContext};
/// use rm_simulator_physics::Contact;
/// let hit = Contact { projectile: 0, shooter: None, caliber: Caliber::Mm17,
///     target: ArmorTarget::Outpost { outpost: 0, face: 0 },
///     position_m: [0.; 3], local_m: [0.; 3], normal_speed_m_s: 30. };
/// assert_eq!(DetectionContext::default().detect(0, &hit), Ok([0., 0.]));
/// ```
#[derive(Clone, Copy, Debug, Default)]
pub struct DetectionContext<'a> {
    /// Optional source-fitted rectangular detector; otherwise use existing artwork.
    pub detector: Option<&'a Detector>,
    /// Current minimum native Rune ring, if native Rune rules are enabled.
    pub minimum_rune_ring: Option<u8>,
    /// Latest accepted detection on this target, in simulation nanoseconds.
    pub last_detection_ns: Option<u64>,
}
impl DetectionContext<'_> {
    /// Apply existing surface, caliber, Rune ring, normal-speed and cadence gates
    /// in their live order. Returns plate-local y/z offsets in metres.
    pub fn detect(&self, time_ns: u64, contact: &Contact) -> Result<[f64; 2], Rejection> {
        let offset = if let Some(detector) = self.detector {
            let [x, y, z] = contact.local_m;
            ((-0.004..=0.012).contains(&x)
                && y.abs() <= detector.half_m[0]
                && z.abs() <= detector.half_m[1])
                .then_some([y, z])
        } else {
            projectile::scoring_offset(contact.target, contact.local_m)
        }
        .ok_or(Rejection::OutsideTarget)?;
        let is_rune = matches!(contact.target, ArmorTarget::Rune { .. });
        if is_rune && contact.caliber != Caliber::Mm17 {
            return Err(Rejection::Caliber);
        }
        if is_rune && let Some(minimum) = self.minimum_rune_ring {
            let ring = gameplay::rmuc_2027::rune::Rune::ring(offset[0].hypot(offset[1]));
            if ring.is_none_or(|r| r < minimum) {
                return Err(Rejection::DisabledRing);
            }
        }
        if contact.normal_speed_m_s <= contact.caliber.armor_detection_speed_m_s() {
            return Err(Rejection::NormalSpeed);
        }
        if self.last_detection_ns.is_some_and(|last| {
            time_ns.saturating_sub(last) < contact.caliber.detection_interval_ns()
        }) {
            return Err(Rejection::DetectionInterval);
        }
        Ok(offset)
    }
}

/// Resolve native combat identities from physical labels; Rune activation is
/// submitted separately. Invalid side indices return `None` rather than aliasing.
pub fn combat_target(target: ArmorTarget) -> Option<gameplay::Target> {
    match target {
        ArmorTarget::Base { base, .. } => gameplay::Team::BOTH
            .get(base as usize)
            .copied()
            .map(gameplay::Target::Base),
        ArmorTarget::Outpost { outpost, .. } => gameplay::Team::BOTH
            .get(outpost as usize)
            .copied()
            .map(gameplay::Target::Outpost),
        ArmorTarget::Chassis { chassis, .. } => Some(gameplay::Target::Robot(chassis)),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Pose, Team};
    use gameplay::rmuc_2027::{
        match_rules::{COUNTDOWN_NS, GroundRole},
        terrain::{Benefit, Terrain},
    };
    fn zone(x: f64) -> Zone {
        Zone {
            side: Team::Red,
            point: Point::Base,
            floor_m: 0.,
            height_m: 0.4,
            polygon_m: vec![[x, -0.5], [x + 0.5, -0.5], [x + 0.5, 0.5], [x, 0.5]],
        }
    }
    #[test]
    fn isolated_tunnel_adapter_drives_existing_terrain_and_match_authorities() {
        let zones: Vec<_> = (0..3)
            .map(|pad| TerrainZone {
                card: Card {
                    side: gameplay::Team::Red,
                    course: Some(Course::Tunnel),
                    index: 0,
                    pad,
                },
                zone: zone(f64::from(pad)),
            })
            .collect();
        let adapter = RfidLayout {
            terrain: &zones,
            ..Default::default()
        };
        let mut terrain = Terrain::default();
        let mut cards = Vec::with_capacity(8);
        let ptr = cards.as_ptr();
        let mut game = Match::new(1);
        game.add_robot(7, gameplay::Team::Red, GroundRole::Infantry);
        game.start(1);
        game.advance_ns(COUNTDOWN_NS);
        for pad in 0..3 {
            let p = Some([f64::from(pad) + 0.25, 0., 0.2]);
            adapter.terrain_cards(p, &mut cards);
            let now = game.elapsed_ns();
            let result = terrain.observe(now, &cards);
            assert_eq!(result, (pad == 2).then_some(Benefit::ThroughTunnel));
            adapter.observe(&mut game, |_| p, &mut cards);
            assert_eq!(game.elapsed_ns(), now);
            assert_eq!(game.robots[0].terrain.until_ns, terrain.until_ns);
            if pad < 2 {
                game.advance_ns(1_000_000_000);
            }
        }
        assert_eq!(cards.as_ptr(), ptr);
        assert_eq!(terrain.until_ns[3], 122_000_000_000);
        adapter.terrain_cards(None, &mut cards);
        assert!(cards.is_empty());
    }
    #[test]
    fn shared_upper_highland_is_one_card_and_other_overlap_still_interrupts() {
        let mut control = zone(0.);
        control.point = Point::Outpost;
        let terrain = [TerrainZone {
            zone: control.clone(),
            card: Card {
                side: gameplay::Team::Red,
                course: Some(Course::Highland),
                index: 0,
                pad: 1,
            },
        }];
        let controls = [control];
        let supplies = [zone(0.)];
        let layout = RfidLayout {
            control: &controls,
            terrain: &terrain,
            ..Default::default()
        };
        let mut cards = Vec::new();
        layout.terrain_cards(Some([0.25, 0., 0.2]), &mut cards);
        assert_eq!(cards, [terrain[0].card]);
        RfidLayout {
            supply: &supplies,
            ..layout
        }
        .terrain_cards(Some([0.25, 0., 0.2]), &mut cards);
        assert_eq!(cards.len(), 2);
        let mut state = Terrain::default();
        state.observe(
            0,
            &[Card {
                pad: 0,
                ..terrain[0].card
            }],
        );
        state.observe(1, &cards);
        assert!(state.progress.is_none());
    }
    fn contact() -> Contact {
        Contact {
            projectile: 1,
            shooter: Some(2),
            caliber: Caliber::Mm17,
            target: ArmorTarget::Outpost {
                outpost: 0,
                face: 0,
            },
            position_m: [0.; 3],
            local_m: [0.; 3],
            normal_speed_m_s: 30.,
        }
    }
    #[test]
    fn source_detection_boundaries_and_cadence_are_caller_owned() {
        let d = Detector {
            pose: Pose::at([0.; 3]),
            half_m: [0.067, 0.05],
        };
        let mut context = DetectionContext {
            detector: Some(&d),
            ..Default::default()
        };
        let mut hit = contact();
        hit.local_m = [-0.004, 0.067, -0.05];
        assert_eq!(context.detect(0, &hit), Ok([0.067, -0.05]));
        context.last_detection_ns = Some(0);
        let duration = hit.caliber.detection_interval_ns();
        assert_eq!(
            context.detect(duration - 1, &hit),
            Err(Rejection::DetectionInterval)
        );
        assert!(context.detect(duration, &hit).is_ok());
        assert_eq!(context.last_detection_ns, Some(0));
        hit.normal_speed_m_s = hit.caliber.armor_detection_speed_m_s();
        assert_eq!(context.detect(duration, &hit), Err(Rejection::NormalSpeed));
        hit.local_m[1] += 1e-6;
        assert_eq!(
            context.detect(duration, &hit),
            Err(Rejection::OutsideTarget)
        );
    }
    #[test]
    fn rune_rejections_keep_precedence_and_target_mapping_is_bounded() {
        let context = DetectionContext {
            minimum_rune_ring: Some(7),
            ..Default::default()
        };
        let mut hit = contact();
        hit.target = ArmorTarget::Rune { rune: 0, blade: 0 };
        hit.local_m = [0., 0.14, 0.];
        hit.caliber = Caliber::Mm42;
        assert_eq!(context.detect(0, &hit), Err(Rejection::Caliber));
        hit.caliber = Caliber::Mm17;
        assert_eq!(context.detect(0, &hit), Err(Rejection::DisabledRing));
        hit.local_m = [0.; 3];
        assert!(context.detect(0, &hit).is_ok());
        assert_eq!(
            combat_target(ArmorTarget::Outpost {
                outpost: 1,
                face: 0
            }),
            Some(gameplay::Target::Outpost(gameplay::Team::Blue))
        );
        assert_eq!(combat_target(ArmorTarget::Base { base: 2, plate: 0 }), None);
        assert_eq!(combat_target(hit.target), None);
    }
}
