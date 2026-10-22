// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Borrowed UC §5.6.1 structure transitions used by the native match.
//!
//! Views borrow caller-owned HP, hidden rotor/history state and the existing
//! [`Control`]. They add neither serialized state nor a clock. Advance control
//! with explicit nanoseconds and apply its events before the next observation.
//! Callers resolve projectile eligibility and damage modifiers before calling
//! `damage`; the enclosing match retains rewards, permissions and victory.
//! Rune experiments use [`super::rune::Rune`] directly.
//!
//! ```
//! use rm_simulator_gameplay::{Team, rmuc_2027::{Control, equipment::Outposts}};
//! let mut control = Control::new();
//! let mut hp = [1000; 2];
//! let mut stopped_ns = [None; 2];
//! control.advance_ns(1_000_000_000);
//! let mut outposts = Outposts::new(&mut control, &mut hp, &mut stopped_ns);
//! assert_eq!(outposts.damage(Team::Red, 1000), 1000);
//! assert_eq!(hp[0], 0);
//! assert_eq!(stopped_ns[0], Some(1_000_000_000));
//! assert!(!control.base_protected(Team::Red));
//! ```

use super::{
    Control, Event, EventKind,
    match_rules::{BASE_HP, OUTPOST_HP, THREE_MIN_NS},
};
use crate::Team;

/// A temporary view of both physical outposts, indexed by original field side.
/// Mutable ownership and territory transitions remain in the borrowed control.
pub struct Outposts<'a> {
    control: &'a mut Control,
    hp: &'a mut [u32; 2],
    rotor_stopped_ns: &'a mut [Option<u64>; 2],
}

impl<'a> Outposts<'a> {
    /// Borrow coherent native state: HP agrees with control's alive flags and
    /// stop timestamps are running-round nanoseconds, retained across rebuilds.
    pub fn new(
        control: &'a mut Control,
        hp: &'a mut [u32; 2],
        rotor_stopped_ns: &'a mut [Option<u64>; 2],
    ) -> Self {
        Self {
            control,
            hp,
            rotor_stopped_ns,
        }
    }

    /// Apply already-adjusted damage in HP, returning actual HP lost (no overkill).
    /// A lethal hit stops the rotor and executes Table 5-21's control transition,
    /// including an immediate ownership-transfer rebuild when applicable.
    pub fn damage(&mut self, side: Team, damage_hp: u32) -> u32 {
        let i = side.index();
        let applied = self.hp[i].min(damage_hp);
        self.hp[i] -= applied;
        if self.hp[i] == 0 && applied > 0 {
            self.destroyed(side);
        }
        applied
    }

    /// Override HP without attack credit, clamped to initial HP (§5.6.1).
    /// Only control events can rebuild a destroyed outpost; a positive override
    /// on a destroyed outpost is ignored. Zero still executes destruction.
    pub fn set_hp(&mut self, side: Team, hp: u32) {
        let i = side.index();
        if self.hp[i] == 0 && hp > 0 {
            return;
        }
        let destroyed = self.hp[i] > 0 && hp == 0;
        self.hp[i] = hp.min(OUTPOST_HP);
        if destroyed {
            self.destroyed(side);
        }
    }

    fn destroyed(&mut self, side: Team) {
        self.rotor_stopped_ns[side.index()].get_or_insert(self.control.snapshot().elapsed_ns);
        if let Ok(events) = self.control.outpost_destroyed(side) {
            self.apply_events(&events);
        }
    }

    /// Consume this control's ordered events once, immediately after advancing
    /// it. Rebuild HP comes from control; permanent rotor stops are preserved.
    pub fn apply_events(&mut self, events: &[Event]) {
        for event in events {
            if let EventKind::OutpostRebuilt { side, hp, .. } = event.kind {
                self.hp[side.index()] = hp;
            }
        }
    }

    /// Settle §5.6.1's permanent rotor stops at the control clock's current time:
    /// three minutes or open opposing Base armor relative to the current owner.
    /// Call after control events, matching the native match's tick ordering.
    /// The caller must visit the three-minute boundary exactly and settle a new
    /// armor opening at its observation time; this view does not partition time.
    pub fn settle_rotors(&mut self, base_open: [bool; 2]) {
        let snapshot = self.control.snapshot();
        for side in Team::BOTH {
            let i = side.index();
            let owner = snapshot.outposts[i].owner;
            if snapshot.elapsed_ns >= THREE_MIN_NS || base_open[owner.other().index()] {
                self.rotor_stopped_ns[i].get_or_insert(snapshot.elapsed_ns);
            }
        }
    }
}

/// A temporary view of Base HP and historical minimum, indexed Red then Blue.
/// Armor presentation/opening and match victory remain with the enclosing match.
pub struct Bases<'a> {
    control: &'a Control,
    hp: &'a mut [u32; 2],
    lowest_hp: &'a mut [u32; 2],
}

impl<'a> Bases<'a> {
    /// Borrow coherent native state: HP is at most initial HP and each historical
    /// minimum is at most current HP. Control supplies §5.6.1 protection.
    pub fn new(control: &'a Control, hp: &'a mut [u32; 2], lowest_hp: &'a mut [u32; 2]) -> Self {
        Self {
            control,
            hp,
            lowest_hp,
        }
    }

    /// Apply already-adjusted damage in HP, unless own outpost territory grants
    /// full protection (§5.6.1). Returns actual HP lost and updates the historical HP minimum.
    ///
    /// ```
    /// use rm_simulator_gameplay::{Team, rmuc_2027::{Control, equipment::Bases}};
    /// let mut control = Control::new();
    /// let mut hp = [10_000; 2];
    /// let mut lowest = hp;
    /// assert_eq!(Bases::new(&control, &mut hp, &mut lowest).damage(Team::Red, 20), 0);
    /// control.outpost_destroyed(Team::Red)?;
    /// assert_eq!(Bases::new(&control, &mut hp, &mut lowest).damage(Team::Red, 20), 20);
    /// assert_eq!(lowest[0], 9980);
    /// # Ok::<(), rm_simulator_gameplay::rmuc_2027::Error>(())
    /// ```
    pub fn damage(&mut self, team: Team, damage_hp: u32) -> u32 {
        if self.control.base_protected(team) {
            return 0;
        }
        let i = team.index();
        let applied = self.hp[i].min(damage_hp);
        self.hp[i] -= applied;
        self.lowest_hp[i] = self.lowest_hp[i].min(self.hp[i]);
        applied
    }

    /// Override HP without attack credit, clamped to initial HP (§5.6.1).
    /// Healing does not erase the historical minimum used by Fortress reserves.
    /// The enclosing match handles a zero-HP result immediately.
    pub fn set_hp(&mut self, team: Team, hp: u32) {
        let i = team.index();
        self.hp[i] = hp.min(BASE_HP);
        self.lowest_hp[i] = self.lowest_hp[i].min(self.hp[i]);
    }
}
