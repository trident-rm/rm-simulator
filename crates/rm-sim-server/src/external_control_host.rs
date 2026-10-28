// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native lease ownership and authoritative application.
use super::*;

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
            || input.sampled_time_ns > now_ns.saturating_add(rm_sim_world::tick_ns())
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
    pub(crate) fn tick(&mut self, field: &mut rm_sim_world::Field) {
        let now = field.time_ns();
        for (&id, entry) in &mut self.entries {
            let Some(active) = entry.active else { continue };
            let body = field.chassis_control_state(id);
            if now.saturating_sub(active.refreshed_ns) >= LEASE_NS
                || body
                    .as_ref()
                    .is_none_or(|b| b.placement_revision != active.input.placement_revision)
                || body.as_ref().is_none_or(|b| b.defeated)
            {
                entry.active = None;
                let _ = crate::navigation::brake(field, id);
            } else if let Some(body) = body {
                let mut command = active
                    .input
                    .motion
                    .resolve(body.pose, body.command)
                    .expect("validated external motion");
                if let Motion::Patrol {
                    head_yaw_rad,
                    head_yaw_rate_rad_s,
                    ..
                } = active.input.motion
                {
                    let elapsed = now.saturating_sub(active.input.sampled_time_ns) as f64 * 1e-9;
                    if !command.folded {
                        command.aim_yaw_rad = head_yaw_rad + head_yaw_rate_rad_s * elapsed;
                    }
                }
                let _ = field.command_chassis(id, command);
            }
        }
    }
}
