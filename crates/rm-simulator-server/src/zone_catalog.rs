// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Inspection footprints, independent of referee activation and buff enforcement.
//! 2027 coordinates are read from UC V1.0.0 Figure 5-23, registered by the bases
//! (±11.586 m) and outposts (±3.091 m, ±3.865 m), about ±0.10 m horizontally.
//! Figure 5-24 shows embedded card spacing, not an exact per-card survey. These
//! outlines show functional RFID areas, including inferred hidden tunnel pads;
//! they must not be used as an authoritative antenna detection model.
use rm_simulator_world::{Team, season::Season};

/// A named area for developer visualization; all coordinates are world FLU metres.
#[derive(Clone, Debug)]
pub struct Area {
    /// Stable side-qualified identifier.
    pub id: String,
    /// Effect or traversal role, including sequence and timing.
    pub label: String,
    /// Side on which this area lies, not a restriction on who can activate it.
    pub side: Team,
    /// Polygon corners in FLU x/y metres.
    pub polygon_m: Vec<[f64; 2]>,
    /// Approximate surface height in metres, used to distinguish decks and tunnels.
    pub floor_m: f64,
    /// True for an RFID detection area; false for a larger mechanism footprint.
    pub rfid: bool,
}
impl Area {
    /// Physical traversal identity for a named native card footprint, if any.
    pub fn terrain_card(&self) -> Option<rm_simulator_world::gameplay::rmuc_2027::terrain::Card> {
        use rm_simulator_world::gameplay::{
            Team as T,
            rmuc_2027::terrain::{Card, Course},
        };
        let name = self
            .id
            .strip_prefix("red-")
            .or_else(|| self.id.strip_prefix("blue-"))?;
        let side = if self.side == Team::Red {
            T::Red
        } else {
            T::Blue
        };
        if let Some(pad) = name.strip_prefix("road-") {
            return Some(Card {
                side,
                course: Some(Course::Road),
                index: 0,
                pad: match pad {
                    "low" => 0,
                    "high" => 1,
                    _ => return None,
                },
            });
        }
        if let Some(rest) = name.strip_prefix("tunnel-") {
            let (index, pad) = rest.split_once('-')?;
            return Some(Card {
                side,
                course: Some(Course::Tunnel),
                index: index.parse::<u8>().ok()?.checked_sub(1)?,
                pad: pad.parse().ok()?,
            });
        }
        if let Some(rest) = name.strip_prefix("highland-card-") {
            let (index, pad) = rest.split_once('-')?;
            return Some(Card {
                side,
                course: Some(Course::Highland),
                index: index.parse().ok()?,
                pad: pad.parse().ok()?,
            });
        }
        None
    }
    /// Constituent control point for 2027 capture, or `None` for inspection-only
    /// terrain, supply, assembly and vantage footprints.
    pub fn control_point(&self) -> Option<rm_simulator_world::rmuc_2027::Point> {
        use rm_simulator_world::rmuc_2027::Point;
        if self.id.ends_with("central-highland") {
            Some(Point::CentralHighland)
        } else if self.id.ends_with("outpost") {
            Some(Point::Outpost)
        } else if self.id.ends_with("base") {
            Some(Point::Base)
        } else if self.id.ends_with("fortress") {
            Some(Point::Fortress)
        } else {
            None
        }
    }
}

/// Native traversal footprints: two Highland courses, one Road and three tunnels per side.
/// Higher Highland cards share the connected control polygon. Its halves and the
/// low footprint are split at y=0; these are diagram approximations, not a card survey.
pub fn terrain_areas() -> Vec<Area> {
    let all = areas(Season::Rm2027);
    let mut result = all
        .iter()
        .filter(|a| a.terrain_card().is_some())
        .cloned()
        .collect::<Vec<_>>();
    for side in [Team::Red, Team::Blue] {
        for course in 0..2 {
            for (pad, suffix) in [(0, "highland-low"), (1, "central-highland")] {
                let source = all
                    .iter()
                    .find(|a| a.side == side && a.id.ends_with(suffix))
                    .expect("named UC polygon");
                let sign = if (side == Team::Red) == (course == 1) {
                    1.
                } else {
                    -1.
                };
                let mut polygon = vec![];
                for (a, b) in source
                    .polygon_m
                    .iter()
                    .zip(source.polygon_m.iter().cycle().skip(1))
                {
                    if a[1] * sign >= 0. {
                        polygon.push(*a);
                    }
                    if (a[1] * sign >= 0.) != (b[1] * sign >= 0.) {
                        let t = -a[1] / (b[1] - a[1]);
                        polygon.push([a[0] + t * (b[0] - a[0]), 0.]);
                    }
                }
                result.push(Area {
                    id: format!(
                        "{}-highland-card-{course}-{pad}",
                        if side == Team::Red { "red" } else { "blue" }
                    ),
                    label: format!(
                        "Highland {} {} / LOW -> HIGH in 5s",
                        course + 1,
                        if pad == 0 { "LOW" } else { "HIGH" }
                    ),
                    polygon_m: polygon,
                    ..source.clone()
                });
            }
        }
    }
    result
}
/// Build the selected season's inspection areas without enabling gameplay rules.
/// 2026 uses the existing referee footprints; 2027 uses Figure 5-23 and §5.6.4.
pub fn areas(season: Season) -> Vec<Area> {
    if season == Season::Rm2026 {
        return rm_simulator_world::zones::rmuc_2026()
            .into_iter()
            .enumerate()
            .map(|(i, a)| Area {
                id: format!("{}-{i}", a.owner.name()),
                label: format!("{:?} pad {}", a.kind, a.pad),
                side: a.owner,
                polygon_m: a.polygon_m,
                floor_m: a.floor_m,
                rfid: true,
            })
            .collect();
    }
    let mut red = Vec::new();
    // Digitized in the displayed figure's pixel frame; keep that frame here so
    // each vertex can be reviewed against the published diagram.
    let mut add = |id: &str, label: &str, floor_m, rfid, pixels: &[[f64; 2]]| {
        red.push(Area {
            id: format!("red-{id}"),
            label: label.into(),
            side: Team::Red,
            polygon_m: pixels
                .iter()
                .map(|[u, v]| [(639.0 - u) / 33.86, (v - 704.0) / 33.86])
                .collect(),
            floor_m,
            rfid,
        });
    };
    add(
        "supply",
        "Supply / own team",
        0.0,
        true,
        &[
            [166., 852.],
            [239., 852.],
            [285., 879.],
            [285., 956.],
            [166., 956.],
        ],
    );
    add(
        "base",
        "Base / control",
        0.20,
        true,
        &[
            [191., 671.],
            [248., 637.],
            [307., 672.],
            [307., 737.],
            [248., 772.],
            [191., 737.],
        ],
    );
    add(
        "fortress",
        "Fortress / base control",
        0.0,
        true,
        &[
            [350., 704.],
            [368., 669.],
            [407., 669.],
            [427., 704.],
            [407., 738.],
            [368., 738.],
        ],
    );
    add(
        "vantage",
        "Vantage / own team",
        0.60,
        true,
        &[
            [177., 525.],
            [190., 511.],
            [235., 511.],
            [249., 526.],
            [249., 569.],
            [235., 582.],
            [190., 582.],
            [177., 568.],
        ],
    );
    add(
        "highland-low",
        "Highland LOW -> HIGH / 5 s",
        0.0,
        true,
        &[[448., 668.], [475., 657.], [475., 752.], [448., 752.]],
    );
    add(
        "central-highland",
        "Central highland + Highland HIGH / outpost control",
        0.40,
        true,
        &[
            [516., 638.],
            [581., 544.],
            [592., 539.],
            [592., 576.],
            [543., 649.],
            [543., 779.],
            [566., 835.],
            [553., 866.],
            [523., 866.],
            [535., 835.],
            [516., 787.],
        ],
    );
    add(
        "assembly",
        "Assembly / Heavy only",
        0.20,
        true,
        &[
            [574., 675.],
            [611., 656.],
            [651., 674.],
            [611., 714.],
            [574., 697.],
        ],
    );
    // Outpost area is continuous with the central-highland card region in the
    // figure. This outline identifies the mechanism vicinity, not a new card.
    add(
        "outpost",
        "Outpost / control (connected highland RFID)",
        0.40,
        false,
        &[
            [514., 807.],
            [554., 807.],
            [565., 840.],
            [551., 867.],
            [523., 867.],
            [532., 840.],
        ],
    );
    // Nine tunnel pads per side (§5.6.4.5): end/interior/end for three tunnels.
    // Interior footprints are inferred between visible ends, hidden by decks.
    for (course, ends) in [
        [[480., 862.], [480., 905.]],
        [[650., 905.], [620., 938.]],
        [[690., 905.], [660., 938.]],
    ]
    .iter()
    .enumerate()
    {
        for pad in 0..3 {
            let t = pad as f64 / 2.;
            let [u, v] = [
                ends[0][0] * (1. - t) + ends[1][0] * t,
                ends[0][1] * (1. - t) + ends[1][1] * t,
            ];
            add(
                &format!("tunnel-{}-{pad}", course + 1),
                &format!(
                    "Tunnel {} {} / 3 s{}",
                    course + 1,
                    ["END A", "INTERIOR", "END B"][pad],
                    if pad == 1 { " (inferred)" } else { "" }
                ),
                0.0,
                true,
                &[
                    [u - 13., v - 12.],
                    [u + 13., v - 12.],
                    [u + 13., v + 12.],
                    [u - 13., v + 12.],
                ],
            );
        }
    }
    // Source CAD Road card bodies 3791032 / 3791514 are on crowned surfaces
    // near -0.13 / +0.05 m. Their height order reverses the old 2026 deck assumption.
    for (id, v, z) in [("road-low", 875., -0.13), ("road-high", 905., 0.05)] {
        add(
            id,
            &format!(
                "Road {} / LOW -> HIGH / 3 s",
                if id.ends_with("low") { "LOW" } else { "HIGH" }
            ),
            z,
            true,
            &[
                [428., v - 9.],
                [461., v - 9.],
                [461., v + 9.],
                [428., v + 9.],
            ],
        );
    }
    let blue: Vec<_> = red
        .iter()
        .map(|a| Area {
            id: a.id.replacen("red-", "blue-", 1),
            label: a.label.clone(),
            side: Team::Blue,
            polygon_m: a.polygon_m.iter().map(|[x, y]| [-x, -y]).collect(),
            floor_m: a.floor_m,
            rfid: a.rfid,
        })
        .collect();
    red.extend(blue);
    red
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn every_2027_zone_and_nine_tunnel_pads_per_side() {
        let areas = areas(Season::Rm2027);
        assert_eq!(areas.len(), 38);
        for side in [Team::Red, Team::Blue] {
            assert_eq!(
                areas
                    .iter()
                    .filter(|a| a.side == side && a.id.contains("tunnel-"))
                    .count(),
                9
            );
            for name in [
                "supply",
                "base",
                "fortress",
                "vantage",
                "highland-low",
                "central-highland",
                "assembly",
                "outpost",
                "road-low",
                "road-high",
            ] {
                assert!(
                    areas
                        .iter()
                        .any(|a| a.id == format!("{}-{name}", side.name()))
                );
            }
        }
        for pair in areas[..19].iter().zip(&areas[19..]) {
            for (a, b) in pair.0.polygon_m.iter().zip(&pair.1.polygon_m) {
                assert_eq!(*a, [-b[0], -b[1]]);
                assert!(a.iter().all(|v| v.is_finite() && v.abs() < 15.));
            }
        }
    }
}
