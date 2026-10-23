// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Official RMUC equipment slots, separate from the simulator's drivable prototypes.
use crate::protocol::Robot;

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
const RM2027: [Slot; 7] = [
    slot(1, "Heavy", "1 robot", Some(Robot::Engineer)),
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
/// Complete RMUC 2027 official slots, including equipment without a simulation model.
/// A prototype's presence means practice support, not complete rule enforcement.
///
/// ```
/// use rm_sim_server::roster::slots;
/// assert_eq!(slots()[0].name, "Heavy");
/// assert_eq!(slots()[3].equipment, "1 hangar + 4 aircraft");
/// ```
pub fn slots() -> &'static [Slot] {
    &RM2027
}
/// Official selectable equipment for a competition. UL Table 3-1 separates the
/// three-versus-three Heavy/Infantry/Sentry roster from Infantry confrontation.
pub fn slots_for(competition: crate::host_args::Competition) -> &'static [Slot] {
    use crate::host_args::Competition;
    const UL: [Slot; 3] = [
        slot(1, "Heavy", "1 robot", Some(Robot::Engineer)),
        slot(3, "Infantry", "1 robot", Some(Robot::Infantry3)),
        slot(7, "Sentry", "1 robot", Some(Robot::Sentry)),
    ];
    const INFANTRY: [Slot; 1] = [slot(3, "Infantry", "1 robot", Some(Robot::Infantry3))];
    match competition {
        Competition::Rmuc2027 => slots(),
        Competition::Rmul2027 => &UL,
        Competition::RmulInfantry2027 => &INFANTRY,
    }
}
/// Whether a named robot is admitted by the selected competition.
pub fn supports_for(competition: crate::host_args::Competition, robot: Robot) -> bool {
    slots_for(competition)
        .iter()
        .any(|slot| slot.prototype == Some(robot))
}
/// Whether a prototype belongs to the RMUC 2027 official roster.
/// Reference Hero/Drone presets are not independent deployable roles.
pub fn supports(robot: Robot) -> bool {
    slots().iter().any(|slot| slot.prototype == Some(robot))
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rmul_rosters_have_distinct_admission() {
        use crate::host_args::Competition::*;
        assert_eq!(slots_for(Rmul2027).len(), 3);
        assert_eq!(slots_for(RmulInfantry2027).len(), 1);
        for robot in [Robot::Engineer, Robot::Infantry3, Robot::Sentry] {
            assert!(supports_for(Rmul2027, robot));
        }
        for robot in [
            Robot::Hero,
            Robot::Infantry4,
            Robot::Infantry5,
            Robot::Drone,
        ] {
            assert!(!supports_for(Rmul2027, robot));
        }
        assert!(!supports_for(RmulInfantry2027, Robot::Sentry));
    }
    #[test]
    fn complete_numbered_rosters_and_subordinate_equipment() {
        assert_eq!(
            slots().iter().map(|s| s.number).collect::<Vec<_>>(),
            [1, 3, 4, 6, 7, 8, 9]
        );
        for robot in [Robot::Hero, Robot::Infantry5, Robot::Drone] {
            assert!(!supports(robot));
        }
        assert_eq!(RM2027.iter().filter(|s| s.prototype.is_some()).count(), 4);
        assert!(RM2027[5].equipment.contains("4 darts"));
    }
}
