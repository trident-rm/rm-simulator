// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Official RMUC equipment slots, separate from the simulator's drivable prototypes.
use crate::protocol::Robot;
use rm_simulator_world::season::Season;

/// One official per-team slot; counts exclude reserves (UC 2027 Table 3-1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Slot {
    /// Official equipment number, not a chassis id.
    pub number: u8,
    /// Competition role shown in the roster.
    pub name: &'static str,
    /// Equipment included in this slot, including subordinate aircraft or darts.
    pub equipment: &'static str,
    /// Available practice prototype, or `None` when the equipment is not implemented.
    pub prototype: Option<Robot>,
}
const fn slot(
    number: u8,
    name: &'static str,
    equipment: &'static str,
    prototype: Option<Robot>,
) -> Slot {
    Slot {
        number,
        name,
        equipment,
        prototype,
    }
}
const RM2026: [Slot; 9] = [
    slot(1, "Hero", "1 robot", Some(Robot::Hero)),
    slot(2, "Engineer", "1 robot", Some(Robot::Engineer)),
    slot(3, "Infantry 3", "1 robot", Some(Robot::Infantry3)),
    slot(4, "Infantry 4", "1 robot", Some(Robot::Infantry4)),
    slot(5, "Infantry 5", "1 robot", Some(Robot::Infantry5)),
    slot(6, "Drone", "1 robot", Some(Robot::Drone)),
    slot(7, "Sentry", "1 robot", Some(Robot::Sentry)),
    slot(8, "Dart system", "1 launcher + 4 darts", None),
    slot(9, "Radar", "1 system", None),
];
const RM2027: [Slot; 7] = [
    slot(1, "Heavy", "1 robot", None),
    slot(3, "Infantry 3", "1 robot", Some(Robot::Infantry3)),
    slot(4, "Infantry 4", "1 robot", Some(Robot::Infantry4)),
    slot(6, "Aerial system", "1 hangar + 4 aircraft", None),
    slot(7, "Sentry", "1 robot", Some(Robot::Sentry)),
    slot(8, "Dart system", "1 launcher + 4 darts", None),
    slot(
        9,
        "Radar system",
        "1 compute module + 1 sensor module",
        None,
    ),
];
/// Complete official slots, including equipment without a simulation model.
/// A prototype's presence means practice support, not complete rule enforcement.
///
/// ```
/// use rm_simulator_server::roster::slots;
/// use rm_simulator_world::season::Season;
/// assert_eq!(slots(Season::Rm2027)[0].name, "Heavy");
/// assert_eq!(slots(Season::Rm2027)[3].equipment, "1 hangar + 4 aircraft");
/// ```
pub fn slots(season: Season) -> &'static [Slot] {
    match season {
        Season::Rm2026 => &RM2026,
        Season::Rm2027 => &RM2027,
    }
}
/// Whether a prototype belongs to this season's official roster.
pub fn supports(season: Season, robot: Robot) -> bool {
    slots(season)
        .iter()
        .any(|slot| slot.prototype == Some(robot))
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn complete_numbered_rosters_and_subordinate_equipment() {
        assert_eq!(
            slots(Season::Rm2026)
                .iter()
                .map(|s| s.number)
                .collect::<Vec<_>>(),
            [1, 2, 3, 4, 5, 6, 7, 8, 9]
        );
        assert_eq!(
            slots(Season::Rm2027)
                .iter()
                .map(|s| s.number)
                .collect::<Vec<_>>(),
            [1, 3, 4, 6, 7, 8, 9]
        );
        for robot in [Robot::Hero, Robot::Engineer, Robot::Infantry5, Robot::Drone] {
            assert!(!supports(Season::Rm2027, robot));
            assert!(supports(Season::Rm2026, robot));
        }
        assert_eq!(RM2027.iter().filter(|s| s.prototype.is_some()).count(), 3);
        assert!(RM2027[5].equipment.contains("4 darts"));
    }
}
