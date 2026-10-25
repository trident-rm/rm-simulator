// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-owned time and reconstruction for a generic physical scene.

use super::PhysicalRuntime;
use sim_engine::{StepDuration, time::Stamp};

/// One generic physical runtime, its completed time and a repeatable initial recipe.
///
/// The recipe returns a fresh runtime and can retain shared cooked resources.
/// It runs only at construction and reset; stepping performs no recipe dispatch,
/// creates no worker and captures no geometry or checkpoint. Commands and compact
/// truth reads use the ordinary runtime APIs. There is no implicit floor.
///
/// Reset reconstructs initial state, not arbitrary solver continuation. Reacquire
/// bindings and reset caller controller, task and sensor history after success.
/// The recipe must retain repeatable inputs and return an unstepped scene at the
/// same integration duration. Its external side effects cannot be rolled back.
///
/// ```
/// use rm_sim_physics::{PhysicalRuntime, StepDuration,
///     runtime::session::PhysicalSession};
/// let mut session = PhysicalSession::new(|| {
///     Ok(PhysicalRuntime::new(StepDuration::HZ_1000))
/// })?;
/// session.step(1000)?;
/// assert_eq!(session.stamp().time_ns, 1_000_000_000);
/// session.reset()?;
/// assert_eq!(session.stamp().tick, 0);
/// # Ok::<(), &'static str>(())
/// ```
pub struct PhysicalSession<F> {
    runtime: PhysicalRuntime,
    recipe: F,
    stamp: Stamp,
}

impl<F> PhysicalSession<F>
where
    F: Fn() -> Result<PhysicalRuntime, &'static str>,
{
    /// Construct the initial scene; setup errors return without a partial session.
    pub fn new(recipe: F) -> Result<Self, &'static str> {
        let runtime = recipe()?;
        Ok(Self {
            runtime,
            recipe,
            stamp: Stamp {
                tick: 0,
                time_ns: 0,
            },
        })
    }

    /// Completed steps and elapsed nanoseconds, without copying physical state.
    pub fn stamp(&self) -> Stamp {
        self.stamp
    }

    /// Immutable physical integration duration for this session, including resets.
    pub fn step_duration(&self) -> StepDuration {
        self.runtime.step_duration()
    }

    /// Borrow privileged physical truth, queries and the latest tick's contacts.
    /// These observations are not simulated sensor measurements.
    pub fn runtime(&self) -> &PhysicalRuntime {
        &self.runtime
    }

    /// Apply ordinary commands or edit physical installations without advancing time.
    /// Do not step or replace the runtime through this accessor: the session owns
    /// time. Edits do not change the reset recipe. Placement/topology changes may
    /// invalidate caller sensor interpolation and cached bindings.
    pub fn runtime_mut(&mut self) -> &mut PhysicalRuntime {
        &mut self.runtime
    }

    /// Advance explicit physical steps, rejecting clock overflow before mutation.
    /// Completed steps remain on an integration error. Physical contact output is
    /// bounded and replaced each tick; step one tick at a time to consume every
    /// tick's contacts or run feedback control. No observation history is allocated.
    pub fn step(&mut self, steps: u64) -> Result<(), &'static str> {
        self.stamp
            .tick
            .checked_add(steps)
            .ok_or("physics tick overflow")?;
        let duration_ns = self.step_duration().as_ns();
        steps
            .checked_mul(duration_ns)
            .and_then(|elapsed| self.stamp.time_ns.checked_add(elapsed))
            .ok_or("physics time overflow")?;
        for _ in 0..steps {
            self.runtime.step(self.stamp.time_ns)?;
            self.stamp.tick += 1;
            self.stamp.time_ns += duration_ns;
        }
        Ok(())
    }

    /// Prepare a fresh scene before replacing live state and restarting time.
    /// Recipe errors or a changed integration duration leave the live runtime,
    /// held commands, bindings and time untouched. Success invalidates old bindings.
    pub fn reset(&mut self) -> Result<(), &'static str> {
        let runtime = (self.recipe)()?;
        if runtime.step_duration() != self.step_duration() {
            return Err("physical session reset changed step duration");
        }
        self.runtime = runtime;
        self.stamp = Stamp {
            tick: 0,
            time_ns: 0,
        };
        Ok(())
    }
}
