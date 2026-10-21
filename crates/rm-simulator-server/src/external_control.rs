// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Leased external navigation inputs, resolved on authoritative simulation ticks.
use rm_simulator_world::{ChassisCommand, Pose};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Maximum accepted planar request magnitude in m/s; an adapter policy, not a rule.
pub const MAX_SPEED_M_S: f64 = 6.;
/// Maximum accepted yaw rate in rad/s; heading feedback is limited to this rate.
pub const MAX_YAW_RATE_RAD_S: f64 = 4.;
/// Stale-command timeout in advancing simulation nanoseconds.
pub const LEASE_NS: u64 = 250_000_000;

/// External planar motion. All world vectors use arena forward/left/up axes.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Motion {
    /// Body forward/left velocity with independent counterclockwise yaw rate.
    BodyVelocity {
        /// Body forward/left velocity in m/s, magnitude at most [`MAX_SPEED_M_S`].
        velocity_m_s: [f64; 2],
        /// Counterclockwise yaw rate in rad/s, bounded by [`MAX_YAW_RATE_RAD_S`].
        yaw_rate_rad_s: f64,
    },
    /// Arena-frame velocity with independent counterclockwise yaw rate.
    WorldVelocity {
        /// Arena x/y velocity in m/s, magnitude at most [`MAX_SPEED_M_S`].
        velocity_m_s: [f64; 2],
        /// Counterclockwise yaw rate in rad/s, bounded by [`MAX_YAW_RATE_RAD_S`].
        yaw_rate_rad_s: f64,
    },
    /// Translate along a world vector while turning to face it. Zero speed turns in place.
    HeadingVector {
        /// Finite nonzero arena x/y direction; magnitude is ignored.
        direction_world: [f64; 2],
        /// Signed speed along the vector in m/s, absolute value at most [`MAX_SPEED_M_S`].
        speed_m_s: f64,
    },
}
impl Motion {
    /// Validate finite values and adapter limits without modifying a robot.
    pub fn validate(self) -> Result<(), &'static str> {
        match self {
            Self::BodyVelocity {
                velocity_m_s,
                yaw_rate_rad_s,
            }
            | Self::WorldVelocity {
                velocity_m_s,
                yaw_rate_rad_s,
            } => {
                if !velocity_m_s.into_iter().all(f64::is_finite)
                    || velocity_m_s[0].hypot(velocity_m_s[1]) > MAX_SPEED_M_S
                    || !yaw_rate_rad_s.is_finite()
                    || yaw_rate_rad_s.abs() > MAX_YAW_RATE_RAD_S
                {
                    return Err("external velocity outside finite adapter limits");
                }
            }
            Self::HeadingVector {
                direction_world,
                speed_m_s,
            } => {
                if !direction_world.into_iter().all(f64::is_finite)
                    || direction_world == [0.; 2]
                    || !speed_m_s.is_finite()
                    || speed_m_s.abs() > MAX_SPEED_M_S
                {
                    return Err("invalid external heading vector or speed");
                }
            }
        }
        Ok(())
    }
    /// Resolve a validated request at the current chassis pose, preserving non-drive channels.
    ///
    /// Heading uses proportional yaw feedback (4/s) with a bounded rate. This is
    /// velocity assistance, not collision avoidance or a path planner.
    ///
    /// ```
    /// use rm_simulator_server::external_control::Motion;
    /// use rm_simulator_world::{ChassisCommand, Pose};
    /// let request = Motion::HeadingVector { direction_world: [1., 0.], speed_m_s: 1. };
    /// let command = request.resolve(Pose::at([0.; 3]), ChassisCommand::default())?;
    /// assert_eq!(command.forward_m_s, 1.);
    /// assert_eq!(command.yaw_rate_rad_s, 0.);
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn resolve(
        self,
        pose: Pose,
        mut held: ChassisCommand,
    ) -> Result<ChassisCommand, &'static str> {
        self.validate()?;
        let norm = pose.rotation_wxyz.iter().map(|x| x * x).sum::<f64>();
        if !pose
            .translation_m
            .into_iter()
            .chain(pose.rotation_wxyz)
            .all(f64::is_finite)
            || (norm - 1.).abs() > 1e-6
        {
            return Err("external motion requires a finite unit pose");
        }
        let [w, x, y, z] = pose.rotation_wxyz;
        let yaw = (2. * (w * z + x * y)).atan2(1. - 2. * (y * y + z * z));
        let (velocity, rate, world) = match self {
            Self::BodyVelocity {
                velocity_m_s,
                yaw_rate_rad_s,
            } => (velocity_m_s, yaw_rate_rad_s, false),
            Self::WorldVelocity {
                velocity_m_s,
                yaw_rate_rad_s,
            } => (velocity_m_s, yaw_rate_rad_s, true),
            Self::HeadingVector {
                direction_world: [dx, dy],
                speed_m_s,
            } => {
                let scale = dx.abs().max(dy.abs());
                let norm = (dx / scale).hypot(dy / scale);
                let error = dy.atan2(dx) - yaw;
                let error = error.sin().atan2(error.cos());
                (
                    [dx / scale / norm * speed_m_s, dy / scale / norm * speed_m_s],
                    (4. * error).clamp(-MAX_YAW_RATE_RAD_S, MAX_YAW_RATE_RAD_S),
                    true,
                )
            }
        };
        let [vx, vy] = velocity;
        let (s, c) = yaw.sin_cos();
        [held.forward_m_s, held.left_m_s] = if world {
            [c * vx + s * vy, -s * vx + c * vy]
        } else {
            velocity
        };
        held.yaw_rate_rad_s = rate;
        held.jump = false;
        Ok(held)
    }
}

/// One external drive update, admitted only through operator-authorized commands.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Input {
    /// Caller-chosen nonzero monotonically increasing lease identity per chassis.
    /// Revoked/expired identities cannot be reused; choose a larger value to reacquire.
    pub lease_id: u64,
    /// Nonzero strictly increasing sequence within the lease. Duplicates cannot refresh it.
    pub sequence: u64,
    /// Simulation timestamp in ns. Commands at least 250 ms old or over one tick ahead are refused.
    pub sampled_time_ns: u64,
    /// Current host pause epoch, read from `/api/state`.
    pub input_epoch: u64,
    /// Current chassis placement revision, read from `/api/state`.
    pub placement_revision: u64,
    /// Requested planar movement, recomputed on each physical tick.
    pub motion: Motion,
}
#[derive(Clone, Copy)]
struct Active {
    input: Input,
    refreshed_ns: u64,
}
#[derive(Default)]
struct Entry {
    last_lease: u64,
    active: Option<Active>,
}
#[derive(Default)]
pub(crate) struct Service {
    entries: BTreeMap<u32, Entry>,
}
impl Service {
    pub(crate) fn remove(&mut self, id: u32) {
        self.entries.remove(&id);
    }
    pub(crate) fn active(&self, id: u32) -> bool {
        self.entries.get(&id).is_some_and(|e| e.active.is_some())
    }
    pub(crate) fn any_active(&self) -> bool {
        self.entries.values().any(|e| e.active.is_some())
    }
    pub(crate) fn receive(
        &mut self,
        id: u32,
        input: Input,
        now_ns: u64,
    ) -> Result<(), &'static str> {
        input.motion.validate()?;
        if now_ns.saturating_sub(input.sampled_time_ns) >= LEASE_NS
            || input.sampled_time_ns > now_ns.saturating_add(rm_simulator_world::tick_ns())
        {
            return Err("external sample timestamp outside lease window");
        }
        if input.lease_id == 0 || input.sequence == 0 {
            return Err("external lease and sequence must be nonzero");
        }
        let entry = self.entries.entry(id).or_default();
        let expired = entry
            .active
            .is_none_or(|active| now_ns.saturating_sub(active.refreshed_ns) >= LEASE_NS);
        if input.lease_id < entry.last_lease || (input.lease_id == entry.last_lease && expired) {
            return Err("external lease revoked or expired; choose a newer lease");
        }
        if let Some(active) = entry.active
            && input.lease_id == active.input.lease_id
            && input.sequence <= active.input.sequence
        {
            return Err("external sequence must increase");
        }
        entry.last_lease = input.lease_id;
        entry.active = Some(Active {
            input,
            refreshed_ns: now_ns.min(input.sampled_time_ns),
        });
        Ok(())
    }
    pub(crate) fn revoke(&mut self, id: u32) -> bool {
        self.entries
            .get_mut(&id)
            .is_some_and(|e| e.active.take().is_some())
    }
    pub(crate) fn release(&mut self, id: u32, lease_id: u64) -> Result<(), &'static str> {
        if !self
            .entries
            .get(&id)
            .is_some_and(|e| e.last_lease == lease_id && e.active.is_some())
        {
            return Err("external lease is not active");
        }
        self.revoke(id);
        Ok(())
    }
    pub(crate) fn tick(&mut self, field: &mut rm_simulator_world::Field) {
        let now = field.time_ns();
        for (&id, entry) in &mut self.entries {
            let Some(active) = entry.active else { continue };
            let body = field.chassis_snapshot(id);
            if now.saturating_sub(active.refreshed_ns) >= LEASE_NS
                || body
                    .as_ref()
                    .is_none_or(|b| b.placement_revision != active.input.placement_revision)
                || field.chassis_defeated(id) != Some(false)
            {
                entry.active = None;
                let _ = crate::navigation::brake(field, id);
            } else if let Some(body) = body {
                let command = active
                    .input
                    .motion
                    .resolve(body.pose, body.command)
                    .expect("validated external motion");
                let _ = field.command_chassis(id, command);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn input(lease_id: u64, sequence: u64, sampled_time_ns: u64) -> Input {
        Input {
            lease_id,
            sequence,
            sampled_time_ns,
            input_epoch: 0,
            placement_revision: 0,
            motion: Motion::HeadingVector {
                direction_world: [0., 1.],
                speed_m_s: 1.,
            },
        }
    }
    #[test]
    fn frames_heading_wrap_and_channel_preservation() {
        let angle = std::f64::consts::FRAC_PI_2;
        let pose = Pose {
            translation_m: [0.; 3],
            rotation_wxyz: [(angle / 2.).cos(), 0., 0., (angle / 2.).sin()],
        };
        let held = ChassisCommand {
            folded: true,
            aim_yaw_rad: 1.2,
            aim_pitch_rad: 0.3,
            balance_control: 23,
            jump: true,
            ..Default::default()
        };
        let world = Motion::WorldVelocity {
            velocity_m_s: [1., 0.],
            yaw_rate_rad_s: 0.7,
        }
        .resolve(pose, held)
        .unwrap();
        assert!(world.forward_m_s.abs() < 1e-12);
        assert!((world.left_m_s + 1.).abs() < 1e-12);
        assert_eq!(world.yaw_rate_rad_s, 0.7);
        assert!(world.folded && !world.jump);
        assert_eq!(
            (
                world.aim_yaw_rad,
                world.aim_pitch_rad,
                world.balance_control
            ),
            (1.2, 0.3, 23)
        );
        let body = Motion::BodyVelocity {
            velocity_m_s: [1., 0.],
            yaw_rate_rad_s: -0.2,
        }
        .resolve(pose, held)
        .unwrap();
        assert_eq!((body.forward_m_s, body.left_m_s), (1., 0.));
        let heading = input(1, 1, 0).motion.resolve(pose, held).unwrap();
        assert!((heading.forward_m_s - 1.).abs() < 1e-12);
        assert!(heading.yaw_rate_rad_s.abs() < 1e-12);
        let yaw = -std::f64::consts::PI + 0.01;
        let pose = Pose {
            rotation_wxyz: [(yaw / 2.).cos(), 0., 0., (yaw / 2.).sin()],
            ..pose
        };
        let heading = Motion::HeadingVector {
            direction_world: [-1., 0.],
            speed_m_s: 0.,
        }
        .resolve(pose, held)
        .unwrap();
        assert!((heading.yaw_rate_rad_s + 0.04).abs() < 1e-10);
        assert_eq!(heading.forward_m_s, 0.);
    }
    #[test]
    fn extreme_vectors_and_invalid_requests() {
        for scale in [1e-300, 1e300] {
            let c = Motion::HeadingVector {
                direction_world: [scale, scale],
                speed_m_s: -2.,
            }
            .resolve(Pose::at([0.; 3]), ChassisCommand::default())
            .unwrap();
            assert!((c.forward_m_s + 2_f64.sqrt()).abs() < 1e-12);
            assert!((c.left_m_s + 2_f64.sqrt()).abs() < 1e-12);
        }
        for motion in [
            Motion::HeadingVector {
                direction_world: [0.; 2],
                speed_m_s: 0.,
            },
            Motion::HeadingVector {
                direction_world: [f64::INFINITY, 1.],
                speed_m_s: 1.,
            },
            Motion::WorldVelocity {
                velocity_m_s: [5., 5.],
                yaw_rate_rad_s: 0.,
            },
            Motion::BodyVelocity {
                velocity_m_s: [0.; 2],
                yaw_rate_rad_s: f64::NAN,
            },
        ] {
            assert!(motion.validate().is_err());
        }
    }
    #[test]
    fn sequence_and_lease_replays_do_not_refresh_and_stale_release_is_rejected() {
        let mut service = Service::default();
        service.receive(0, input(1, 1, 0), 0).unwrap();
        assert!(service.receive(0, input(1, 1, 0), LEASE_NS - 1).is_err());
        assert!(service.receive(0, input(1, 2, LEASE_NS), LEASE_NS).is_err());
        service.receive(0, input(2, 1, LEASE_NS), LEASE_NS).unwrap();
        assert!(service.release(0, 1).is_err());
        assert!(service.active(0));
        service.release(0, 2).unwrap();
        assert!(service.receive(0, input(2, 2, LEASE_NS), LEASE_NS).is_err());
        service.receive(0, input(3, 1, LEASE_NS), LEASE_NS).unwrap();
        assert!(service.receive(0, input(4, 1, 0), LEASE_NS).is_err());
        assert!(service.active(0));
    }
}
