// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! RFID-to-native-terrain adapter, without a Field, referee or full match.
use rm_sim_world::{
    Team,
    gameplay::{
        self,
        rmuc_2027::terrain::{Benefit, Card, Course, Terrain},
    },
    rmuc_2027::{Point, TerrainZone, Zone},
    rule_adapters::RfidLayout,
};
fn main() {
    // Illustrative local RFID bands, independently authored from passage geometry.
    let zones: Vec<_> = (0..3)
        .map(|pad| {
            let x = f64::from(pad);
            TerrainZone {
                card: Card {
                    side: gameplay::Team::Red,
                    course: Some(Course::Tunnel),
                    index: 0,
                    pad,
                },
                zone: Zone {
                    side: Team::Red,
                    point: Point::Base,
                    floor_m: 0.,
                    height_m: 0.4,
                    polygon_m: vec![[x, -0.5], [x + 0.5, -0.5], [x + 0.5, 0.5], [x, 0.5]],
                },
            }
        })
        .collect();
    let layout = RfidLayout {
        terrain: &zones,
        ..Default::default()
    };
    let mut cards = Vec::new();
    let mut authority = Terrain::default();
    let mut benefit = None;
    for pad in 0..3 {
        let now_ns = pad * 1_000_000_000;
        layout.terrain_cards(Some([pad as f64 + 0.25, 0., 0.2]), &mut cards);
        benefit = authority.observe(now_ns, &cards);
    }
    assert_eq!(benefit, Some(Benefit::ThroughTunnel));
    println!(
        "Standalone terrain authority: {benefit:?}; cooling benefit until {} ns",
        authority.until_ns[3]
    );
}
