// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! RMUC 2027 V1.0.0 battlefield control (§5.6.2, Tables 5-20–5-21).
//!
//! This independent component does not enable a complete 2027 [`crate::Game`].
//! Callers report eligible, already resolved zone activations, including §5.6.4's
//! two-second expiry; this component does not infer contacts or robot eligibility.
//! Only advance running-round time. Durations are nanoseconds, independent of
//! the caller's tick rate. Capture transitions settle before score awards at the
//! same timestamp; observations submitted afterward affect only future time.
//!
//! Table 5-21's temporary-control recovery runs when its controller is absent,
//! even if neither team activates a point. Both teams present pause all progress.
//! An outpost's physical owner remains distinct from its territory controller.
//! Scoring counts temporary and full control, the user-selected interpretation
//! of §5.6.2. Base protection still requires full control (§5.6.1).

/// Live RMUC 2027 control/structure match subset.
pub mod adaptive;
/// Button-driven assembly progression and shared-core arbitration.
pub mod assembly;
/// Seeded equipment-local assembly poses and spherical orientation (Table 5-10).
pub mod assembly_pose;
pub mod combat;
/// Sensor-driven assembly manipulation workbench; separate from virtual match rewards.
pub mod manipulation;
pub mod match_rules;
/// Radar progress, information observations and vulnerability, UC §5.7.4.
pub mod radar;
/// Native 2027 Rune activation, motion, rings and non-field benefits.
pub mod rune;
/// Native physical terrain-card sequences and timed benefits (§5.6.4.5).
pub mod terrain;

use crate::Team;
use serde::{Deserialize, Serialize};

/// UC §5.6.2, Table 5-21: occupation or absence needed for a transition, 5 s.
pub const CAPTURE_NS: u64 = 5_000_000_000;
/// UC §5.6.2: control points settle once per second.
pub const SCORE_PERIOD_NS: u64 = 1_000_000_000;
/// UC §6.6: running round duration, 7 min.
pub const ROUND_NS: u64 = 420_000_000_000;
/// UC §5.6.1: a transferred outpost rebuilds with 500 HP.
pub const REBUILT_OUTPOST_HP: u32 = 500;

/// Territory identity is its original field location, never its current owner.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Territory {
    /// Outpost and central-highland points initially on Red's side.
    RedOutpost,
    /// Outpost and central-highland points initially on Blue's side.
    BlueOutpost,
    /// Base and Fortress points initially on Red's side.
    RedBase,
    /// Base and Fortress points initially on Blue's side.
    BlueBase,
}

impl Territory {
    /// Snapshot slot order: red/blue outposts followed by red/blue bases.
    pub const ALL: [Self; 4] = [
        Self::RedOutpost,
        Self::BlueOutpost,
        Self::RedBase,
        Self::BlueBase,
    ];

    /// Slot in [`Snapshot::territories`].
    pub const fn index(self) -> usize {
        match self {
            Self::RedOutpost => 0,
            Self::BlueOutpost => 1,
            Self::RedBase => 2,
            Self::BlueBase => 3,
        }
    }

    /// Initial owner; only this team may fully control a base territory.
    pub const fn initial_team(self) -> Team {
        match self {
            Self::RedOutpost | Self::RedBase => Team::Red,
            Self::BlueOutpost | Self::BlueBase => Team::Blue,
        }
    }

    fn is_outpost(self) -> bool {
        matches!(self, Self::RedOutpost | Self::BlueOutpost)
    }
}

/// Constituent buff point, scoped to its original field side (UC Table 5-20).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Point {
    /// Outpost buff point, first slot of an outpost territory.
    Outpost,
    /// Central-highland point, second slot of an outpost territory.
    CentralHighland,
    /// Base buff point, first slot of a base territory.
    Base,
    /// Fortress point, second slot of a base territory.
    Fortress,
}

impl Point {
    fn location(self, side: Team) -> (Territory, usize) {
        match (self, side) {
            (Self::Outpost, Team::Red) => (Territory::RedOutpost, 0),
            (Self::Outpost, Team::Blue) => (Territory::BlueOutpost, 0),
            (Self::CentralHighland, Team::Red) => (Territory::RedOutpost, 1),
            (Self::CentralHighland, Team::Blue) => (Territory::BlueOutpost, 1),
            (Self::Base, Team::Red) => (Territory::RedBase, 0),
            (Self::Base, Team::Blue) => (Territory::BlueBase, 0),
            (Self::Fortress, Team::Red) => (Territory::RedBase, 1),
            (Self::Fortress, Team::Blue) => (Territory::BlueBase, 1),
        }
    }
}

/// The controller and strength of a territory (UC Table 5-21).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ControlState {
    /// Full control; either team for an outpost, initial owner only for a base.
    Full(Team),
    /// Temporary control, which can revert after the controller's absence.
    Temporary(Team),
}

impl ControlState {
    /// Team currently controlling the territory, regardless of strength.
    pub const fn team(self) -> Team {
        match self {
            Self::Full(team) | Self::Temporary(team) => team,
        }
    }
}

/// State of one territory; progress belongs to its current transition.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TerritoryState {
    /// Current controller and strength.
    pub control: ControlState,
    /// Nanoseconds toward capture while full, or recovery while temporary.
    /// Always less than [`CAPTURE_NS`] in a valid snapshot.
    pub progress_ns: u64,
    /// Eligible activations by constituent point, then team (Red, Blue).
    /// Slots are Outpost/CentralHighland or Base/Fortress per territory kind.
    pub activations: [[bool; 2]; 2],
}

impl TerritoryState {
    fn active(&self, team: Team) -> bool {
        self.activations.iter().any(|point| point[team.index()])
    }

    // +1 advances the pending transition, -1 rolls it back, 0 contests/holds.
    fn direction(&self) -> i8 {
        let controller = self.control.team();
        let own = self.active(controller);
        let other = self.active(controller.other());
        if own && other {
            0
        } else {
            match self.control {
                ControlState::Full(_) if other => 1,
                ControlState::Temporary(_) if !own => 1,
                _ => -1,
            }
        }
    }
}

/// Physical outpost state, separate from temporary territory control.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct OutpostState {
    /// Team owning the physical outpost and its damage attribution.
    pub owner: Team,
    /// Whether another destruction observation may be reported.
    pub alive: bool,
}

/// Complete restorable state; the component starts at running-round time zero.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Snapshot {
    /// Elapsed running time in nanoseconds, at most [`ROUND_NS`].
    pub elapsed_ns: u64,
    /// Cumulative integer control points, Red then Blue.
    pub points: [u32; 2],
    /// Territories in [`Territory::ALL`] order, including partial progress/input.
    pub territories: [TerritoryState; 4],
    /// Outposts by original side, Red then Blue; owners may change.
    pub outposts: [OutpostState; 2],
}

/// One change and its exact elapsed running-round timestamp.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Event {
    /// Nanoseconds since the running round began.
    pub at_ns: u64,
    /// Authoritative state change at this time.
    pub kind: EventKind,
}

/// Changes consumed by the native match adapter, in deterministic order.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EventKind {
    /// A territory completed an occupation, absence or destruction transition.
    ControlChanged {
        /// Original field location.
        territory: Territory,
        /// State before the transition.
        previous: ControlState,
        /// State after the transition.
        current: ControlState,
    },
    /// A fully transferred outpost revives; applying buffs/HP is the caller's job.
    OutpostRebuilt {
        /// Original side locating the outpost.
        side: Team,
        /// New physical owner.
        owner: Team,
        /// UC §5.6.1 rebuild HP, always [`REBUILT_OUTPOST_HP`].
        hp: u32,
    },
    /// Whole-second settlement awarded control points.
    PointsAwarded {
        /// Team receiving the award.
        team: Team,
        /// UC §5.6.2: 1 for both outpost territories, 3 for both base territories.
        amount: u32,
    },
}

/// Rejected observations/restores leave the component unchanged.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    /// Inputs cannot change a round at or after its seven-minute endpoint.
    #[error("the RMUC 2027 control round has ended")]
    RoundEnded,
    /// A physical outpost must rebuild before it can be destroyed again.
    #[error("the outpost is already destroyed")]
    AlreadyDestroyed,
    /// Snapshot time, progress, score or ownership violates component invariants.
    #[error("invalid RMUC 2027 control snapshot")]
    InvalidSnapshot,
}

/// Deterministic UC territory state machine, independent of the 2026 engine.
///
/// ```
/// use rm_simulator_gameplay::{Team, rmuc_2027::{Control, Point, CAPTURE_NS}};
/// let mut control = Control::new();
/// control.set_activation(Team::Blue, Point::Outpost, [true, false])?;
/// control.advance_ns(CAPTURE_NS);
/// assert!(!control.base_protected(Team::Blue));
/// assert_eq!(control.snapshot().points, [1, 0]);
/// # Ok::<(), rm_simulator_gameplay::rmuc_2027::Error>(())
/// ```
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Control {
    state: Snapshot,
}

impl Default for Control {
    fn default() -> Self {
        Self::new()
    }
}

impl Control {
    /// Start UC §5.6.2 with each team fully controlling its original territories.
    /// No zones are activated and both cumulative scores are zero.
    pub fn new() -> Self {
        Self {
            state: Snapshot {
                elapsed_ns: 0,
                points: [0; 2],
                territories: Territory::ALL.map(|territory| TerritoryState {
                    control: ControlState::Full(territory.initial_team()),
                    progress_ns: 0,
                    activations: [[false; 2]; 2],
                }),
                outposts: Team::BOTH.map(|owner| OutpostState { owner, alive: true }),
            },
        }
    }

    /// Borrow all state needed to resume the same control computation.
    pub fn snapshot(&self) -> &Snapshot {
        &self.state
    }

    /// Restore validated state, including fractional time, progress and activations.
    /// This is a control-component snapshot, not a full match/physics checkpoint.
    ///
    /// ```
    /// use rm_simulator_gameplay::rmuc_2027::Control;
    /// let control = Control::new();
    /// assert_eq!(Control::restore(control.snapshot().clone())?, control);
    /// # Ok::<(), rm_simulator_gameplay::rmuc_2027::Error>(())
    /// ```
    pub fn restore(state: Snapshot) -> Result<Self, Error> {
        if state.elapsed_ns > ROUND_NS
            || state
                .points
                .iter()
                .map(|&points| u64::from(points))
                .sum::<u64>()
                > 4 * (state.elapsed_ns / SCORE_PERIOD_NS)
        {
            return Err(Error::InvalidSnapshot);
        }
        for territory in Territory::ALL {
            let entry = &state.territories[territory.index()];
            if entry.progress_ns >= CAPTURE_NS
                || (!territory.is_outpost()
                    && entry.control != ControlState::Full(territory.initial_team())
                    && entry.control != ControlState::Temporary(territory.initial_team().other()))
            {
                return Err(Error::InvalidSnapshot);
            }
            if territory.is_outpost() {
                let outpost = state.outposts[territory.initial_team().index()];
                let valid = match entry.control {
                    ControlState::Full(team) => outpost.owner == team && outpost.alive,
                    ControlState::Temporary(team) => outpost.alive == (outpost.owner != team),
                };
                if !valid {
                    return Err(Error::InvalidSnapshot);
                }
            }
        }
        Ok(Self { state })
    }

    /// Replace both teams' resolved activation at one point at the current time.
    /// The aggregate is team presence, not robot count. The caller handles alive,
    /// weak, disabled-zone and contact-expiry eligibility before reporting it.
    pub fn set_activation(
        &mut self,
        side: Team,
        point: Point,
        teams: [bool; 2],
    ) -> Result<(), Error> {
        self.require_running()?;
        let (territory, slot) = point.location(side);
        self.state.territories[territory.index()].activations[slot] = teams;
        Ok(())
    }

    /// Report one authoritative physical outpost destruction (UC Table 5-21).
    /// Full control becomes temporary for the same team; temporary control
    /// becomes full for its controller. A transfer rebuilds for the new team.
    /// This observes destruction; projectile damage and HP are not simulated here.
    ///
    /// ```
    /// use rm_simulator_gameplay::{Team, rmuc_2027::Control};
    /// let mut control = Control::new();
    /// control.outpost_destroyed(Team::Red)?;
    /// assert!(!control.base_protected(Team::Red));
    /// assert!(!control.snapshot().outposts[0].alive);
    /// # Ok::<(), rm_simulator_gameplay::rmuc_2027::Error>(())
    /// ```
    pub fn outpost_destroyed(&mut self, side: Team) -> Result<Vec<Event>, Error> {
        self.require_running()?;
        if !self.state.outposts[side.index()].alive {
            return Err(Error::AlreadyDestroyed);
        }
        self.state.outposts[side.index()].alive = false;
        let territory = match side {
            Team::Red => Territory::RedOutpost,
            Team::Blue => Territory::BlueOutpost,
        };
        let current = match self.state.territories[territory.index()].control {
            ControlState::Full(team) => ControlState::Temporary(team),
            ControlState::Temporary(team) => ControlState::Full(team),
        };
        let mut events = Vec::new();
        self.transition(territory, current, &mut events);
        Ok(events)
    }

    /// UC §5.6.1 base protection: full control of that base's original outpost
    /// territory grants 100% defense. Temporary control does not qualify.
    pub fn base_protected(&self, team: Team) -> bool {
        self.state.territories[team.index()].control == ControlState::Full(team)
    }

    /// Advance running time explicitly, clamped at the seven-minute round end.
    /// Capture completes before a coincident whole-second score settlement.
    /// Each settlement awards 1 point for both outposts and 3 for both bases.
    /// Returned events are ordered by time, territory order, then team order.
    /// Zero elapsed time performs no transition or score settlement.
    pub fn advance_ns(&mut self, duration_ns: u64) -> Vec<Event> {
        let target = self.state.elapsed_ns + duration_ns.min(ROUND_NS - self.state.elapsed_ns);
        let mut events = Vec::new();
        while self.state.elapsed_ns < target {
            let next_score = (self.state.elapsed_ns / SCORE_PERIOD_NS + 1) * SCORE_PERIOD_NS;
            let mut delta =
                (target - self.state.elapsed_ns).min(next_score - self.state.elapsed_ns);
            for territory in &self.state.territories {
                if territory.direction() == 1 {
                    delta = delta.min(CAPTURE_NS - territory.progress_ns);
                }
            }
            for territory in &mut self.state.territories {
                territory.progress_ns = match territory.direction() {
                    1 => territory.progress_ns + delta,
                    -1 => territory.progress_ns.saturating_sub(delta),
                    _ => territory.progress_ns,
                };
            }
            self.state.elapsed_ns += delta;
            for territory in Territory::ALL {
                let entry = &self.state.territories[territory.index()];
                if entry.progress_ns == CAPTURE_NS {
                    let current = match entry.control {
                        ControlState::Full(team) => ControlState::Temporary(team.other()),
                        ControlState::Temporary(team) => ControlState::Full(team.other()),
                    };
                    self.transition(territory, current, &mut events);
                }
            }
            if self.state.elapsed_ns == next_score {
                for team in Team::BOTH {
                    let amount = self.score_rate(team);
                    if amount != 0 {
                        self.state.points[team.index()] += amount;
                        events.push(Event {
                            at_ns: self.state.elapsed_ns,
                            kind: EventKind::PointsAwarded { team, amount },
                        });
                    }
                }
            }
        }
        events
    }

    fn require_running(&self) -> Result<(), Error> {
        if self.state.elapsed_ns == ROUND_NS {
            Err(Error::RoundEnded)
        } else {
            Ok(())
        }
    }

    fn score_rate(&self, team: Team) -> u32 {
        let controls =
            |territory: Territory| match self.state.territories[territory.index()].control {
                ControlState::Full(owner) => owner == team,
                ControlState::Temporary(owner) => owner == team,
            };
        u32::from(controls(Territory::RedOutpost) && controls(Territory::BlueOutpost))
            + 3 * u32::from(controls(Territory::RedBase) && controls(Territory::BlueBase))
    }

    fn transition(&mut self, territory: Territory, current: ControlState, events: &mut Vec<Event>) {
        let entry = &mut self.state.territories[territory.index()];
        let previous = entry.control;
        entry.control = current;
        entry.progress_ns = 0;
        events.push(Event {
            at_ns: self.state.elapsed_ns,
            kind: EventKind::ControlChanged {
                territory,
                previous,
                current,
            },
        });
        if territory.is_outpost()
            && let ControlState::Full(team) = current
        {
            let outpost = &mut self.state.outposts[territory.initial_team().index()];
            if outpost.owner != team {
                *outpost = OutpostState {
                    owner: team,
                    alive: true,
                };
                events.push(Event {
                    at_ns: self.state.elapsed_ns,
                    kind: EventKind::OutpostRebuilt {
                        side: territory.initial_team(),
                        owner: team,
                        hp: REBUILT_OUTPOST_HP,
                    },
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state(control: &Control, territory: Territory) -> &TerritoryState {
        &control.snapshot().territories[territory.index()]
    }

    #[test]
    fn initial_ownership_protects_bases_without_passive_points() {
        let mut control = Control::new();
        for territory in Territory::ALL {
            assert_eq!(
                state(&control, territory).control,
                ControlState::Full(territory.initial_team())
            );
        }
        assert!(control.base_protected(Team::Red));
        assert!(control.base_protected(Team::Blue));
        assert!(control.advance_ns(10 * SCORE_PERIOD_NS).is_empty());
        assert_eq!(control.snapshot().points, [0, 0]);
    }

    #[test]
    fn either_constituent_point_captures_at_exactly_five_seconds() {
        for point in [
            Point::Outpost,
            Point::CentralHighland,
            Point::Base,
            Point::Fortress,
        ] {
            let mut control = Control::new();
            control
                .set_activation(Team::Blue, point, [true, false])
                .unwrap();
            let territory = point.location(Team::Blue).0;
            control.advance_ns(CAPTURE_NS - 1);
            assert_eq!(
                state(&control, territory).control,
                ControlState::Full(Team::Blue)
            );
            let events = control.advance_ns(1);
            assert_eq!(
                state(&control, territory).control,
                ControlState::Temporary(Team::Red)
            );
            assert_eq!(events[0].at_ns, CAPTURE_NS);
            assert_eq!(state(&control, territory).progress_ns, 0);
            if territory.is_outpost() {
                assert!(!control.base_protected(Team::Blue));
                // Occupying territory does not transfer the physical outpost.
                assert_eq!(
                    control.snapshot().outposts[1],
                    OutpostState {
                        owner: Team::Blue,
                        alive: true
                    }
                );
            }
        }
    }

    #[test]
    fn interrupted_capture_rolls_back_instead_of_resetting() {
        let mut control = Control::new();
        control
            .set_activation(Team::Blue, Point::Outpost, [true, false])
            .unwrap();
        control.advance_ns(3 * SCORE_PERIOD_NS);
        control
            .set_activation(Team::Blue, Point::Outpost, [false, false])
            .unwrap();
        control.advance_ns(SCORE_PERIOD_NS);
        assert_eq!(
            state(&control, Territory::BlueOutpost).progress_ns,
            2 * SCORE_PERIOD_NS
        );
        control
            .set_activation(Team::Blue, Point::CentralHighland, [true, false])
            .unwrap();
        control.advance_ns(3 * SCORE_PERIOD_NS);
        assert_eq!(
            state(&control, Territory::BlueOutpost).control,
            ControlState::Temporary(Team::Red)
        );
    }

    #[test]
    fn contest_across_different_points_pauses_capture_and_recovery() {
        let mut control = Control::new();
        control
            .set_activation(Team::Blue, Point::Outpost, [true, false])
            .unwrap();
        control.advance_ns(2 * SCORE_PERIOD_NS);
        control
            .set_activation(Team::Blue, Point::CentralHighland, [false, true])
            .unwrap();
        control.advance_ns(8 * SCORE_PERIOD_NS);
        assert_eq!(
            state(&control, Territory::BlueOutpost).progress_ns,
            2 * SCORE_PERIOD_NS
        );
        control
            .set_activation(Team::Blue, Point::CentralHighland, [false, false])
            .unwrap();
        control.advance_ns(3 * SCORE_PERIOD_NS);
        control
            .set_activation(Team::Blue, Point::Outpost, [false, false])
            .unwrap();
        control.advance_ns(2 * SCORE_PERIOD_NS);
        control
            .set_activation(Team::Blue, Point::Outpost, [true, false])
            .unwrap();
        control
            .set_activation(Team::Blue, Point::CentralHighland, [false, true])
            .unwrap();
        control.advance_ns(8 * SCORE_PERIOD_NS);
        assert_eq!(
            state(&control, Territory::BlueOutpost).progress_ns,
            2 * SCORE_PERIOD_NS
        );
        assert_eq!(
            state(&control, Territory::BlueOutpost).control,
            ControlState::Temporary(Team::Red)
        );
    }

    #[test]
    fn temporary_absence_recovers_with_no_robot_present_and_can_roll_back() {
        let mut control = Control::new();
        control
            .set_activation(Team::Blue, Point::Base, [true, false])
            .unwrap();
        control.advance_ns(CAPTURE_NS);
        control
            .set_activation(Team::Blue, Point::Base, [false, false])
            .unwrap();
        control.advance_ns(3 * SCORE_PERIOD_NS);
        control
            .set_activation(Team::Blue, Point::Fortress, [true, false])
            .unwrap();
        control.advance_ns(SCORE_PERIOD_NS);
        assert_eq!(
            state(&control, Territory::BlueBase).progress_ns,
            2 * SCORE_PERIOD_NS
        );
        control
            .set_activation(Team::Blue, Point::Fortress, [false, false])
            .unwrap();
        control.advance_ns(3 * SCORE_PERIOD_NS - 1);
        assert_eq!(
            state(&control, Territory::BlueBase).control,
            ControlState::Temporary(Team::Red)
        );
        control.advance_ns(1);
        assert_eq!(
            state(&control, Territory::BlueBase).control,
            ControlState::Full(Team::Blue)
        );
    }

    #[test]
    fn temporary_control_scores_both_pairs_and_both_teams_symmetrically() {
        for team in Team::BOTH {
            let mut control = Control::new();
            let mut active = [false; 2];
            active[team.index()] = true;
            control
                .set_activation(team.other(), Point::Outpost, active)
                .unwrap();
            control
                .set_activation(team.other(), Point::Base, active)
                .unwrap();
            let events = control.advance_ns(CAPTURE_NS);
            assert_eq!(control.snapshot().points[team.index()], 4);
            assert_eq!(control.snapshot().points[team.other().index()], 0);
            // Capture settles before scoring at the same timestamp.
            assert_eq!(
                events.last().unwrap().kind,
                EventKind::PointsAwarded { team, amount: 4 }
            );
            control.advance_ns(SCORE_PERIOD_NS);
            assert_eq!(control.snapshot().points[team.index()], 8);
        }
    }

    #[test]
    fn fully_owned_outpost_destruction_demotes_then_transfers_on_absence() {
        let mut control = Control::new();
        let events = control.outpost_destroyed(Team::Blue).unwrap();
        assert_eq!(events.len(), 1);
        assert_eq!(
            state(&control, Territory::BlueOutpost).control,
            ControlState::Temporary(Team::Blue)
        );
        assert!(!control.base_protected(Team::Blue));
        assert!(!control.snapshot().outposts[1].alive);
        let before = control.clone();
        assert_eq!(
            control.outpost_destroyed(Team::Blue),
            Err(Error::AlreadyDestroyed)
        );
        assert_eq!(control, before);
        let events = control.advance_ns(CAPTURE_NS);
        assert_eq!(
            state(&control, Territory::BlueOutpost).control,
            ControlState::Full(Team::Red)
        );
        assert_eq!(
            control.snapshot().outposts[1],
            OutpostState {
                owner: Team::Red,
                alive: true
            }
        );
        assert!(events.iter().any(|event| event.kind
            == EventKind::OutpostRebuilt {
                side: Team::Blue,
                owner: Team::Red,
                hp: 500,
            }));
    }

    #[test]
    fn destruction_while_temporarily_captured_secures_full_control_and_rebuilds() {
        let mut control = Control::new();
        control
            .set_activation(Team::Blue, Point::Outpost, [true, false])
            .unwrap();
        control.advance_ns(CAPTURE_NS);
        let events = control.outpost_destroyed(Team::Blue).unwrap();
        assert_eq!(events.len(), 2);
        assert_eq!(events[0].at_ns, CAPTURE_NS);
        assert_eq!(
            events[1].kind,
            EventKind::OutpostRebuilt {
                side: Team::Blue,
                owner: Team::Red,
                hp: 500
            }
        );
        assert_eq!(
            state(&control, Territory::BlueOutpost).control,
            ControlState::Full(Team::Red)
        );
        // An uncontested captured full territory does not decay on its own.
        control
            .set_activation(Team::Blue, Point::Outpost, [false, false])
            .unwrap();
        control.advance_ns(20 * SCORE_PERIOD_NS);
        assert_eq!(
            state(&control, Territory::BlueOutpost).control,
            ControlState::Full(Team::Red)
        );
        assert!(!control.base_protected(Team::Blue));
        // Blue can capture it back and recover its own base protection.
        control
            .set_activation(Team::Blue, Point::CentralHighland, [false, true])
            .unwrap();
        control.advance_ns(CAPTURE_NS);
        control.outpost_destroyed(Team::Blue).unwrap();
        assert!(control.base_protected(Team::Blue));
        assert_eq!(
            control.snapshot().outposts[1],
            OutpostState {
                owner: Team::Blue,
                alive: true
            }
        );
    }

    #[test]
    fn occupation_then_retreat_does_not_rebuild_an_undestroyed_outpost() {
        let mut control = Control::new();
        control
            .set_activation(Team::Blue, Point::Outpost, [true, false])
            .unwrap();
        control.advance_ns(CAPTURE_NS);
        control
            .set_activation(Team::Blue, Point::Outpost, [false, false])
            .unwrap();
        let events = control.advance_ns(CAPTURE_NS);
        assert!(control.base_protected(Team::Blue));
        assert!(
            !events
                .iter()
                .any(|event| matches!(event.kind, EventKind::OutpostRebuilt { .. }))
        );
    }

    #[test]
    fn step_partition_and_serialized_restore_preserve_state_and_event_order() {
        let mut coarse = Control::new();
        coarse
            .set_activation(Team::Blue, Point::Outpost, [true, false])
            .unwrap();
        coarse
            .set_activation(Team::Red, Point::Fortress, [false, true])
            .unwrap();
        let mut fine = coarse.clone();
        let expected = coarse.advance_ns(13_123_456_789);
        let mut actual = fine.advance_ns(2_123_456_789);
        let json = serde_json::to_string(fine.snapshot()).unwrap();
        fine = Control::restore(serde_json::from_str(&json).unwrap()).unwrap();
        for _ in 0..1408 {
            actual.extend(fine.advance_ns(7_812_500)); // One 128 Hz world tick.
        }
        assert_eq!(coarse, fine);
        assert_eq!(expected, actual);
        // Repeat across capture, destruction, rebuild and absence recovery.
        coarse.outpost_destroyed(Team::Blue).unwrap();
        fine.outpost_destroyed(Team::Blue).unwrap();
        coarse
            .set_activation(Team::Red, Point::Fortress, [false, false])
            .unwrap();
        fine.set_activation(Team::Red, Point::Fortress, [false, false])
            .unwrap();
        let expected = coarse.advance_ns(12 * SCORE_PERIOD_NS);
        let mut actual = Vec::new();
        for _ in 0..1536 {
            actual.extend(fine.advance_ns(7_812_500));
        }
        assert_eq!(coarse, fine);
        assert_eq!(expected, actual);
    }

    #[test]
    fn opposite_pair_awards_stop_when_temporary_control_recovers() {
        let mut control = Control::new();
        control
            .set_activation(Team::Blue, Point::CentralHighland, [true, false])
            .unwrap();
        control
            .set_activation(Team::Red, Point::Fortress, [false, true])
            .unwrap();
        control.advance_ns(CAPTURE_NS + SCORE_PERIOD_NS);
        assert_eq!(control.snapshot().points, [2, 6]);
        control
            .set_activation(Team::Blue, Point::CentralHighland, [false, false])
            .unwrap();
        control
            .set_activation(Team::Red, Point::Fortress, [false, false])
            .unwrap();
        control.advance_ns(CAPTURE_NS);
        assert_eq!(control.snapshot().points, [6, 18]);
        control.advance_ns(SCORE_PERIOD_NS);
        assert_eq!(control.snapshot().points, [6, 18]);
        assert!(control.base_protected(Team::Blue));
    }

    #[test]
    fn round_endpoint_clamps_time_and_rejects_late_observations_atomically() {
        let mut control = Control::new();
        control
            .set_activation(Team::Blue, Point::Outpost, [true, false])
            .unwrap();
        control.advance_ns(u64::MAX);
        assert_eq!(control.snapshot().elapsed_ns, ROUND_NS);
        assert_eq!(control.snapshot().points, [416, 0]); // Seconds 5 through 420 inclusive.
        let before = control.clone();
        assert!(control.advance_ns(u64::MAX).is_empty());
        assert_eq!(
            control.set_activation(Team::Blue, Point::Outpost, [false, false]),
            Err(Error::RoundEnded)
        );
        assert_eq!(
            control.outpost_destroyed(Team::Blue),
            Err(Error::RoundEnded)
        );
        assert_eq!(control, before);
    }

    #[test]
    fn invalid_restores_cannot_grant_enemy_full_base_control_or_break_progress() {
        let initial = Control::new().snapshot().clone();
        for invalid in 0..7 {
            let mut snapshot = initial.clone();
            match invalid {
                0 => snapshot.elapsed_ns = ROUND_NS + 1,
                1 => snapshot.territories[0].progress_ns = CAPTURE_NS,
                2 => snapshot.territories[2].control = ControlState::Full(Team::Blue),
                3 => snapshot.outposts[0].alive = false,
                4 => snapshot.territories[0].control = ControlState::Temporary(Team::Red),
                5 => {
                    snapshot.territories[0].control = ControlState::Temporary(Team::Blue);
                    snapshot.outposts[0].alive = false;
                }
                _ => snapshot.points = [1, 0],
            }
            assert_eq!(Control::restore(snapshot), Err(Error::InvalidSnapshot));
        }
    }
}

/// UC §5.7.2 Sentry operation modes and cumulative posture budgets.
pub mod sentry;

/// UC §§5.1.4–5 referee power buffer and module connectivity.
pub mod power;
