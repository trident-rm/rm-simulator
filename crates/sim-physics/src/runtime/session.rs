// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-owned time and reconstruction for a generic physical scene.

use super::PhysicalRuntime;
use sim_engine::{StepDuration, time::Stamp};

/// A physical session could not construct, step or reset its scene.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SessionError {
    /// The caller-owned reconstruction recipe rejected its inputs.
    Recipe(super::RuntimeError),
    /// Completed step count would overflow.
    TickOverflow,
    /// Elapsed nanoseconds would overflow.
    TimeOverflow,
    /// The runtime could not complete the next tick.
    Integration(super::RuntimeError),
    /// A reset recipe changed the immutable integration period.
    ChangedStepDuration,
}
impl SessionError {
    /// Original diagnostic for compatibility callers.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Recipe(e) => e.as_str(),
            Self::TickOverflow => "physics tick overflow",
            Self::TimeOverflow => "physics time overflow",
            Self::Integration(e) => e.as_str(),
            Self::ChangedStepDuration => "physical session reset changed step duration",
        }
    }
}
impl std::fmt::Display for SessionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}
impl std::error::Error for SessionError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Integration(e) | Self::Recipe(e) => Some(e),
            _ => None,
        }
    }
}
impl From<SessionError> for &'static str {
    fn from(error: SessionError) -> Self {
        error.as_str()
    }
}

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
/// use sim_physics::{PhysicalRuntime, StepDuration,
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
    F: Fn() -> Result<PhysicalRuntime, super::RuntimeError>,
{
    /// Construct the initial scene; setup errors return without a partial session.
    ///
    /// # Errors
    /// Returns the caller recipe failure without constructing a partial session.
    pub fn new(recipe: F) -> Result<Self, SessionError> {
        let runtime = recipe().map_err(SessionError::Recipe)?;
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
    ///
    /// # Errors
    /// Rejects nanosecond overflow before integration; session clocks retain only completed ticks.
    pub fn step(&mut self, steps: u64) -> Result<(), SessionError> {
        self.stamp
            .tick
            .checked_add(steps)
            .ok_or(SessionError::TickOverflow)?;
        let duration_ns = self.step_duration().as_ns();
        steps
            .checked_mul(duration_ns)
            .and_then(|elapsed| self.stamp.time_ns.checked_add(elapsed))
            .ok_or(SessionError::TimeOverflow)?;
        for _ in 0..steps {
            self.runtime
                .step(self.stamp.time_ns)
                .map_err(SessionError::Integration)?;
            self.stamp.tick += 1;
            self.stamp.time_ns += duration_ns;
        }
        Ok(())
    }

    /// Prepare a fresh scene before replacing live state and restarting time.
    /// Recipe errors or a changed integration duration leave the live runtime,
    /// held commands, bindings and time untouched. Success invalidates old bindings.
    ///
    /// # Errors
    /// Returns recipe/integration-period errors without replacing the live scene or clock.
    pub fn reset(&mut self) -> Result<(), SessionError> {
        let runtime = (self.recipe)().map_err(SessionError::Recipe)?;
        if runtime.step_duration() != self.step_duration() {
            return Err(SessionError::ChangedStepDuration);
        }
        self.runtime = runtime;
        self.stamp = Stamp {
            tick: 0,
            time_ns: 0,
        };
        Ok(())
    }
}
