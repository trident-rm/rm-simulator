// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Read-only live sensor attachments. Capture on the owner; acquire on bounded workers.
use rm_simulator_physics::sensors::{
    imu::{CarrierFrame, CarrierSample},
    rolling::SceneCapture,
    synchronized::{Config, Frame, Synchronized},
};
use rm_simulator_world::Field;
use serde::Serialize;
use std::{
    collections::BTreeMap,
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
        mpsc::{self, Receiver, SyncSender, TrySendError},
    },
    thread,
    time::{Duration, Instant},
};

struct Input {
    before: SceneCapture,
    after: SceneCapture,
    trace: Vec<CarrierSample>,
}
struct Attachment {
    config: Config,
    before: Option<SceneCapture>,
    input: SyncSender<Input>,
    output: Receiver<Frame>,
    drops: Arc<AtomicU64>,
    last_poll: Instant,
}
/// Bounded measured frames and explicit cumulative capture/output loss.
#[derive(Serialize)]
pub struct Poll {
    /// Current host simulation time, nanoseconds; unchanged while paused.
    pub time_ns: u64,
    /// At most two completed packets, in acquisition order.
    pub frames: Vec<Frame>,
    /// Cumulative full-queue losses or invalid capture intervals for this attachment.
    pub dropped_intervals: u64,
}
#[derive(Default)]
pub(crate) struct Service {
    next_id: u64,
    attachments: BTreeMap<u64, Attachment>,
    trace: Vec<CarrierSample>,
}
impl Service {
    pub(crate) fn attach(&mut self, field: &mut Field, config: Config) -> Result<u64, String> {
        self.prune(field);
        if self.attachments.len() >= 4 {
            return Err("at most four sensor attachments".into());
        }
        if field.chassis_defeated(config.robot) != Some(false) {
            return Err("sensor chassis missing or defeated".into());
        }
        let mut processor = Synchronized::new(config, field.time_ns()).map_err(str::to_owned)?;
        let before = capture(field, config).map_err(str::to_owned)?;
        let (tx, rx) = mpsc::sync_channel::<Input>(4);
        let (output, received) = mpsc::sync_channel(2);
        let drops = Arc::new(AtomicU64::new(0));
        let count = drops.clone();
        thread::Builder::new()
            .name("rm-sensors".into())
            .spawn(move || {
                for input in rx {
                    let mut frames = Vec::new();
                    if processor
                        .ingest(&input.before, &input.after, &input.trace, &mut frames)
                        .is_err()
                    {
                        count.fetch_add(1, Ordering::Relaxed);
                    }
                    for frame in frames {
                        match output.try_send(frame) {
                            Ok(()) => {}
                            Err(TrySendError::Full(_)) => {
                                count.fetch_add(1, Ordering::Relaxed);
                            }
                            Err(TrySendError::Disconnected(_)) => return,
                        }
                    }
                }
            })
            .map_err(|e| e.to_string())?;
        let id = self.next_id;
        self.next_id = self.next_id.checked_add(1).ok_or("sensor ID overflow")?;
        if self.attachments.is_empty() {
            field.set_carrier_trace(true);
        }
        self.attachments.insert(
            id,
            Attachment {
                config,
                before: Some(before),
                input: tx,
                output: received,
                drops,
                last_poll: Instant::now(),
            },
        );
        Ok(id)
    }
    pub(crate) fn prune(&mut self, field: &mut Field) {
        let had = !self.attachments.is_empty();
        self.attachments.retain(|_, a| {
            a.last_poll.elapsed() < Duration::from_secs(5)
                && field.chassis_defeated(a.config.robot) == Some(false)
        });
        if had && self.attachments.is_empty() {
            field.set_carrier_trace(false);
        }
    }
    pub(crate) fn active(&self) -> bool {
        !self.attachments.is_empty()
    }
    pub(crate) fn poll(&mut self, field: &mut Field, id: u64) -> Result<Poll, String> {
        self.prune(field);
        let a = self
            .attachments
            .get_mut(&id)
            .ok_or("sensor attachment expired or missing")?;
        a.last_poll = Instant::now();
        Ok(Poll {
            time_ns: field.time_ns(),
            frames: a.output.try_iter().take(2).collect(),
            dropped_intervals: a.drops.load(Ordering::Relaxed),
        })
    }
    pub(crate) fn detach(&mut self, field: &mut Field, id: u64) {
        self.attachments.remove(&id);
        if self.attachments.is_empty() {
            field.set_carrier_trace(false);
        }
    }
    pub(crate) fn capture(&mut self, field: &mut Field) {
        if !self.active() {
            return;
        }
        self.trace.clear();
        field.drain_carrier_trace(&mut self.trace);
        for a in self.attachments.values_mut() {
            let after = capture(field, a.config).ok();
            let before = std::mem::replace(&mut a.before, after.clone());
            if let (Some(before), Some(after)) = (before, after) {
                let trace = self
                    .trace
                    .iter()
                    .filter(|s| s.robot == a.config.robot)
                    .copied()
                    .collect();
                if a.input
                    .try_send(Input {
                        before,
                        after,
                        trace,
                    })
                    .is_err()
                {
                    a.drops.fetch_add(1, Ordering::Relaxed);
                }
            } else {
                a.drops.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}
fn capture(field: &Field, config: Config) -> Result<SceneCapture, &'static str> {
    let capture = field.lidar_scene_capture(config.robot)?;
    match config.carrier {
        CarrierFrame::Chassis => Ok(capture),
        CarrierFrame::Gimbal => capture.with_mount_frame(
            field
                .chassis_snapshot(config.robot)
                .ok_or("missing carrier")?
                .turret,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rm_simulator_world::{ChassisConfig, ChassisPlacement, FieldConfig, Pose, RobotKind, Team};
    fn field() -> Field {
        let mut field = Field::new(&FieldConfig {
            chassis: vec![ChassisPlacement {
                config: ChassisConfig::default(),
                spawn: Pose::at([0., 0., 0.4]),
                team: Team::Red,
                kind: RobotKind::Infantry,
                performance: None,
            }],
            ..Default::default()
        })
        .unwrap();
        field.step(128).unwrap();
        field
    }
    #[test]
    fn bounded_worker_emits_coherent_live_measurements_and_detaches() {
        let mut field = field();
        let mut service = Service::default();
        let epoch = field.time_ns();
        let id = service
            .attach(
                &mut field,
                Config {
                    stride: 3,
                    ..Default::default()
                },
            )
            .unwrap();
        let mut received = Vec::new();
        for _ in 0..45 {
            field.step(1).unwrap();
            service.capture(&mut field);
            std::thread::sleep(Duration::from_millis(2));
            received.extend(service.poll(&mut field, id).unwrap().frames);
        }
        let deadline = Instant::now() + Duration::from_secs(2);
        while received.len() < 2 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(2));
            received.extend(service.poll(&mut field, id).unwrap().frames);
        }
        assert_eq!(received.len(), 2);
        assert_eq!(received[0].start_ns, epoch + 100_000_000);
        assert_eq!(received[0].imu.len(), 20);
        assert_eq!(received[0].imu.last().unwrap().time_ns, received[0].end_ns);
        assert!(
            received[0]
                .lidar
                .points
                .iter()
                .all(|p| p.sample_index % 3 == 0)
        );
        service.detach(&mut field, id);
        assert!(!service.active());
        assert!(service.poll(&mut field, id).is_err());
    }
    #[test]
    fn admission_and_expiry_bound_sensor_lifetimes() {
        let mut field = field();
        let mut service = Service::default();
        assert!(
            service
                .attach(
                    &mut field,
                    Config {
                        robot: 999,
                        ..Default::default()
                    }
                )
                .is_err()
        );
        let id = service.attach(&mut field, Config::default()).unwrap();
        service.attachments.get_mut(&id).unwrap().last_poll =
            Instant::now() - Duration::from_secs(6);
        assert!(service.poll(&mut field, id).is_err());
        assert!(!service.active());
    }
    #[test]
    fn slow_consumer_drops_output_without_blocking_the_world() {
        let mut field = field();
        let mut service = Service::default();
        let id = service
            .attach(
                &mut field,
                Config {
                    stride: 31,
                    ..Default::default()
                },
            )
            .unwrap();
        for _ in 0..110 {
            field.step(1).unwrap();
            service.capture(&mut field);
            std::thread::sleep(Duration::from_millis(2));
        }
        let deadline = Instant::now() + Duration::from_secs(2);
        while service.attachments[&id].drops.load(Ordering::Relaxed) == 0
            && Instant::now() < deadline
        {
            std::thread::sleep(Duration::from_millis(2));
        }
        let batch = service.poll(&mut field, id).unwrap();
        assert_eq!(batch.frames.len(), 2);
        assert!(batch.dropped_intervals > 0);
        assert_eq!(batch.time_ns, 238 * rm_simulator_world::tick_ns());
    }
}
