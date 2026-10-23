// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Inspection footprints, independent of referee activation and buff enforcement.
//! 2027 coordinates are read from UC V1.0.0 Figure 5-23, registered by the bases
//! (±11.586 m) and outposts (±3.091 m, ±3.865 m), about ±0.10 m horizontally.
//! Figure 5-24 shows embedded card spacing, not an exact per-card survey. These
//! outlines show functional RFID areas, including inferred hidden tunnel pads;
//! they must not be used as an authoritative antenna detection model.
use rm_sim_world::Team;

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
    /// Antenna/body-center height band, metres. Tunnel cards exclude travel on roofs.
    pub fn height_m(&self) -> f64 {
        if self.id.contains("-tunnel-") {
            0.26
        } else {
            0.45
        }
    }

    /// Physical traversal identity for a named native card footprint, if any.
    pub fn terrain_card(&self) -> Option<rm_sim_world::gameplay::rmuc_2027::terrain::Card> {
        use rm_sim_world::gameplay::{
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
    pub fn control_point(&self) -> Option<rm_sim_world::rmuc_2027::Point> {
        use rm_sim_world::rmuc_2027::Point;
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
    let all = areas();
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
/// RMUC 2027 inspection areas from Figure 5-23 and §5.6.4.
/// These diagram footprints alone do not enable gameplay rules.
pub fn areas() -> Vec<Area> {
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
    // UC §5.6.4.5/Table 5-27: nine tunnel pads per side. Endpoint centers
    // follow 250 mm CAD plates, not the old diagonal Figure 5-23 digitization.
    // See scripts/survey-tunnel-rfid.py and docs/tunnel-rfid-calibration.md.
    // Interior centers are inferred below each surveyed roof. The 600 mm
    // transverse antenna footprint is an explicit simulator approximation.
    for (course, centers, floors, along_x) in [
        (
            0,
            [[4.6871, 4.87535], [4.6868, 5.3452], [4.68645, 5.81504]],
            [-0.10, 0.01, 0.047],
            false,
        ),
        (
            1,
            [
                [-1.35594, 5.95629],
                [-0.92627, 5.95629],
                [-0.49659, 5.95629],
            ],
            [-0.1563; 3],
            true,
        ),
        (
            2,
            [[-0.40594, 6.92491], [0.02373, 6.92491], [0.45341, 6.92491]],
            [0.0176; 3],
            true,
        ),
    ] {
        for (pad, [x, y]) in centers.into_iter().enumerate() {
            let longitudinal = if pad == 1 { 0.08 } else { 0.125 };
            let [dx, dy] = if along_x {
                [longitudinal, 0.30]
            } else {
                [0.30, longitudinal]
            };
            let pixels = [
                [x - dx, y - dy],
                [x + dx, y - dy],
                [x + dx, y + dy],
                [x - dx, y + dy],
            ]
            .map(|[x, y]| [639. - x * 33.86, 704. + y * 33.86]);
            add(
                &format!("tunnel-{}-{pad}", course + 1),
                &format!(
                    "Tunnel {} {} / 3 s (CAD aligned{})",
                    course + 1,
                    ["END A", "INTERIOR", "END B"][pad],
                    if pad == 1 { "; inferred" } else { "" }
                ),
                floors[pad],
                true,
                &pixels,
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
    fn calibrated_tunnel_routes_reward_both_directions_and_mirrors_but_not_roof_travel() {
        use rm_sim_world::{
            gameplay::rmuc_2027::{
                match_rules::{COUNTDOWN_NS, GroundRole, Match},
                terrain::Course,
            },
            rmuc_2027::Zone,
        };
        let areas = terrain_areas();
        for side in [Team::Red, Team::Blue] {
            for index in 0..3 {
                let course = areas
                    .iter()
                    .filter(|a| {
                        a.side == side
                            && a.terrain_card().is_some_and(|c| {
                                c.course == Some(Course::Tunnel) && c.index == index
                            })
                    })
                    .collect::<Vec<_>>();
                let center = |a: &Area| {
                    [
                        a.polygon_m.iter().map(|p| p[0]).sum::<f64>() / 4.,
                        a.polygon_m.iter().map(|p| p[1]).sum::<f64>() / 4.,
                        a.floor_m + 0.18,
                    ]
                };
                let a = center(
                    course
                        .iter()
                        .find(|a| a.terrain_card().unwrap().pad == 0)
                        .unwrap(),
                );
                let b = center(
                    course
                        .iter()
                        .find(|a| a.terrain_card().unwrap().pad == 2)
                        .unwrap(),
                );
                for reverse in [false, true] {
                    for (duration_ns, above_roof, expected) in [
                        (2_500_000_000u64, false, true),
                        (8_000_000_000, false, false),
                        (2_500_000_000, true, false),
                    ] {
                        let mut g = Match::new(1);
                        g.add_robot(7, rm_sim_world::gameplay::Team::Red, GroundRole::Sentry);
                        g.start(1);
                        g.advance_ns(COUNTDOWN_NS);
                        for step in 0..=500 {
                            let t = if reverse {
                                1. - f64::from(step) / 500.
                            } else {
                                f64::from(step) / 500.
                            };
                            let mut p = std::array::from_fn(|i| a[i] * (1. - t) + b[i] * t);
                            if above_roof {
                                p[2] += 0.5;
                            }
                            let cards = areas
                                .iter()
                                .filter(|a| {
                                    Zone {
                                        side: a.side,
                                        point: rm_sim_world::rmuc_2027::Point::Base,
                                        floor_m: a.floor_m,
                                        height_m: a.height_m(),
                                        polygon_m: a.polygon_m.clone(),
                                    }
                                    .contains(p)
                                })
                                .filter_map(|a| a.terrain_card())
                                .collect::<Vec<_>>();
                            g.observe_terrain(7, &cards);
                            if step < 500 {
                                g.advance_ns(duration_ns / 500);
                            }
                        }
                        assert_eq!(
                            g.robots[0].terrain.until_ns[3] > g.elapsed_ns(),
                            expected,
                            "side={side:?} course={index} reverse={reverse} roof={above_roof} duration={duration_ns}"
                        );
                    }
                }
            }
        }
    }
    #[test]
    fn every_2027_zone_and_nine_tunnel_pads_per_side() {
        let areas = areas();
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
