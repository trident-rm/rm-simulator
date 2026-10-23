// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Read-only live sensor attachments. Capture on the owner; acquire on bounded workers.
//! Optional host-duration telemetry never enters physics or changes sensor stamps.
use rm_sim_physics::sensors::{
    imu::{CarrierFrame, CarrierSample},
    rolling::SceneCapture,
    synchronized::{Config, Frame, Synchronized},
};
use rm_sim_world::Field;
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

/// Host loss counters. Units differ: capture brackets, ingest calls and full bundles.
/// Counters are sampled once per poll; concurrent increments may arrive next time.
#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct LossCounts {
    /// Input capture brackets rejected by a full/closed worker queue or capture failure.
    pub capture_intervals: u64,
    /// Acquisition ingest calls that returned an error.
    pub ingest_errors: u64,
    /// Completed LiDAR/IMU bundles rejected by the full output queue.
    pub output_frames: u64,
}
impl LossCounts {
    fn total(self) -> u64 {
        self.capture_intervals
            .saturating_add(self.ingest_errors)
            .saturating_add(self.output_frames)
    }
}
#[derive(Default)]
struct LossCounters {
    capture: AtomicU64,
    ingest: AtomicU64,
    output: AtomicU64,
}
impl LossCounters {
    fn snapshot(&self) -> LossCounts {
        LossCounts {
            capture_intervals: self.capture.load(Ordering::Relaxed),
            ingest_errors: self.ingest.load(Ordering::Relaxed),
            output_frames: self.output.load(Ordering::Relaxed),
        }
    }
}
/// Work accumulated since the previous ingest call that completed any bundles.
/// Ingest calls can cross a packet boundary; these are compute windows, not an
/// exact partition of per-ray work or simulated acquisition time.
#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct AcquisitionWork {
    /// Attachment-local work-batch ordinal. Deduplicate when several frames share it.
    pub batch: u64,
    /// Number of bundles completed by this batch; some may be dropped at delivery.
    pub completed_frames: u32,
    /// Number of input brackets ingested, including warmup/discarded partial work.
    pub input_brackets: u64,
    /// Sum of geometry-capture/trace-selection wall durations on the owner, ns.
    pub capture_wall_ns: u64,
    /// Sum of input queue residence durations across the ingested brackets, ns.
    pub input_queue_wait_ns: u64,
    /// Maximum individual input queue residence duration in this work batch, ns.
    pub input_queue_wait_max_ns: u64,
    /// Sum of acquisition ingest wall durations on the worker, ns; not CPU time.
    pub ingest_wall_ns: u64,
}
/// A completed bundle's host-duration telemetry, separate from measured values.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct FrameTelemetry {
    /// Original simulation packet sequence; identifies the corresponding measured frame.
    pub sequence: u64,
    /// Work that completed this frame. Several frames may share one work-batch ID.
    pub work: AcquisitionWork,
    /// Wall duration from ingest completion until the owner dequeues this bundle, ns.
    pub output_queue_wait_ns: u64,
}
/// Bounded measured frames and explicit cumulative capture/output loss.
#[derive(Serialize)]
pub struct Poll {
    /// Current host simulation time, nanoseconds; unchanged while paused.
    pub time_ns: u64,
    /// At most two completed packets, in acquisition order.
    pub frames: Vec<Frame>,
    /// Sum of the sampled loss counters, retaining the original heterogeneous units.
    pub dropped_intervals: u64,
}
/// Opt-in host timings for one poll. Durations cannot be used as sensor stamps.
#[derive(Serialize)]
pub struct PollTelemetry {
    /// Submission/admission and mailbox wait until dispatch on the owner, ns.
    /// `None` for direct Simulation calls that bypass the HostHandle mailbox.
    pub owner_request_wait_ns: Option<u64>,
    /// Owner time in the profiled service poll (pruning, draining and metadata), ns.
    pub owner_poll_wall_ns: u64,
    /// Aligned one-to-one with measured frames, bounded to two records.
    pub frames: Vec<FrameTelemetry>,
    /// Stage loss counts sampled together with the ordinary total.
    pub losses: LossCounts,
}
/// Original measured poll fields plus a separate opt-in telemetry object.
#[derive(Serialize)]
pub struct ProfiledPoll {
    /// Unchanged sensor contract, flattened on the JSON wire.
    #[serde(flatten)]
    pub measurements: Poll,
    /// Host wall durations and stage loss counters; no world poses or wall stamps.
    pub telemetry: PollTelemetry,
}
struct InputTiming {
    queued_at: Instant,
    capture_wall_ns: u64,
}
struct Input {
    before: SceneCapture,
    after: SceneCapture,
    trace: Vec<CarrierSample>,
    timing: Option<InputTiming>,
}
struct Completed {
    frame: Frame,
    timing: Option<(Instant, AcquisitionWork)>,
}
struct Attachment {
    config: Config,
    before: Option<SceneCapture>,
    input: SyncSender<Input>,
    output: Receiver<Completed>,
    drops: Arc<LossCounters>,
    last_poll: Instant,
    profiled: bool,
}
#[derive(Default)]
pub(crate) struct Service {
    next_id: u64,
    attachments: BTreeMap<u64, Attachment>,
    trace: Vec<CarrierSample>,
}
/// Convert a host interval to saturating integer nanoseconds.
pub(crate) fn duration_ns(duration: Duration) -> u64 {
    u64::try_from(duration.as_nanos()).unwrap_or(u64::MAX)
}
impl Service {
    pub(crate) fn attach(&mut self, field: &mut Field, config: Config) -> Result<u64, String> {
        self.attach_inner(field, config, false)
    }
    pub(crate) fn attach_profiled(
        &mut self,
        field: &mut Field,
        config: Config,
    ) -> Result<u64, String> {
        self.attach_inner(field, config, true)
    }
    fn attach_inner(
        &mut self,
        field: &mut Field,
        config: Config,
        profiled: bool,
    ) -> Result<u64, String> {
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
        let drops = Arc::new(LossCounters::default());
        let count = drops.clone();
        thread::Builder::new()
            .name("rm-sensors".into())
            .spawn(move || {
                let mut work = AcquisitionWork::default();
                for input in rx {
                    let begin = input.timing.as_ref().map(|_| Instant::now());
                    if let (Some(timing), Some(begin)) = (&input.timing, begin) {
                        let wait = duration_ns(begin.saturating_duration_since(timing.queued_at));
                        work.input_brackets = work.input_brackets.saturating_add(1);
                        work.capture_wall_ns =
                            work.capture_wall_ns.saturating_add(timing.capture_wall_ns);
                        work.input_queue_wait_ns = work.input_queue_wait_ns.saturating_add(wait);
                        work.input_queue_wait_max_ns = work.input_queue_wait_max_ns.max(wait);
                    }
                    let mut frames = Vec::new();
                    if processor
                        .ingest(&input.before, &input.after, &input.trace, &mut frames)
                        .is_err()
                    {
                        count.ingest.fetch_add(1, Ordering::Relaxed);
                    }
                    let completed_at = begin.map(|_| Instant::now());
                    if let (Some(begin), Some(end)) = (begin, completed_at) {
                        work.ingest_wall_ns = work
                            .ingest_wall_ns
                            .saturating_add(duration_ns(end.saturating_duration_since(begin)));
                    }
                    if !frames.is_empty() {
                        work.completed_frames =
                            frames.len().try_into().expect("bounded acquisition frames");
                        for frame in frames {
                            let completed = Completed {
                                frame,
                                timing: completed_at.map(|ready| (ready, work)),
                            };
                            match output.try_send(completed) {
                                Ok(()) => {}
                                Err(TrySendError::Full(_)) => {
                                    count.output.fetch_add(1, Ordering::Relaxed);
                                }
                                Err(TrySendError::Disconnected(_)) => return,
                            }
                        }
                        work = AcquisitionWork {
                            batch: work.batch.saturating_add(1),
                            ..Default::default()
                        };
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
                profiled,
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
            frames: a.output.try_iter().take(2).map(|c| c.frame).collect(),
            dropped_intervals: a.drops.snapshot().total(),
        })
    }
    pub(crate) fn poll_profiled(
        &mut self,
        field: &mut Field,
        id: u64,
    ) -> Result<ProfiledPoll, String> {
        let begin = Instant::now();
        self.prune(field);
        let a = self
            .attachments
            .get_mut(&id)
            .ok_or("sensor attachment expired or missing")?;
        if !a.profiled {
            return Err("sensor telemetry was not enabled at attachment".into());
        }
        a.last_poll = begin;
        let mut frames = Vec::with_capacity(2);
        let mut timings = Vec::with_capacity(2);
        for c in a.output.try_iter().take(2) {
            let (ready, work) = c.timing.expect("profiled attachment has profiled output");
            timings.push(FrameTelemetry {
                sequence: c.frame.sequence,
                work,
                output_queue_wait_ns: duration_ns(ready.elapsed()),
            });
            frames.push(c.frame);
        }
        let losses = a.drops.snapshot();
        Ok(ProfiledPoll {
            measurements: Poll {
                time_ns: field.time_ns(),
                frames,
                dropped_intervals: losses.total(),
            },
            telemetry: PollTelemetry {
                owner_request_wait_ns: None,
                owner_poll_wall_ns: duration_ns(begin.elapsed()),
                frames: timings,
                losses,
            },
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
            let begin = a.profiled.then(Instant::now);
            let after = capture(field, a.config).ok();
            let before = std::mem::replace(&mut a.before, after.clone());
            if let (Some(before), Some(after)) = (before, after) {
                let trace = self
                    .trace
                    .iter()
                    .filter(|s| s.robot == a.config.robot)
                    .copied()
                    .collect();
                let timing = begin.map(|start| {
                    let queued_at = Instant::now();
                    InputTiming {
                        queued_at,
                        capture_wall_ns: duration_ns(queued_at.saturating_duration_since(start)),
                    }
                });
                if a.input
                    .try_send(Input {
                        before,
                        after,
                        trace,
                        timing,
                    })
                    .is_err()
                {
                    a.drops.capture.fetch_add(1, Ordering::Relaxed);
                }
            } else {
                a.drops.capture.fetch_add(1, Ordering::Relaxed);
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
    use rm_sim_world::{ChassisConfig, ChassisPlacement, FieldConfig, Pose, RobotKind, Team};
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
        while service.attachments[&id].drops.snapshot().total() == 0 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(2));
        }
        let batch = service.poll(&mut field, id).unwrap();
        assert_eq!(batch.frames.len(), 2);
        assert!(batch.dropped_intervals > 0);
        assert_eq!(batch.time_ns, 238 * rm_sim_world::tick_ns());
    }
    #[test]
    fn profiling_keeps_measured_values_identical_to_plain_acquisition() {
        let mut field = field();
        let mut service = Service::default();
        let config = Config {
            stride: 31,
            ..Default::default()
        };
        let plain = service.attach(&mut field, config).unwrap();
        let profiled = service.attach_profiled(&mut field, config).unwrap();
        assert!(service.poll_profiled(&mut field, plain).is_err());
        let mut expected = Vec::new();
        let mut actual = Vec::new();
        let mut timings = Vec::new();
        for _ in 0..45 {
            field.step(1).unwrap();
            service.capture(&mut field);
            std::thread::sleep(Duration::from_millis(2));
            expected.extend(service.poll(&mut field, plain).unwrap().frames);
            let poll = service.poll_profiled(&mut field, profiled).unwrap();
            assert!(poll.telemetry.owner_request_wait_ns.is_none());
            assert_eq!(poll.telemetry.frames.len(), poll.measurements.frames.len());
            for (frame, timing) in poll.measurements.frames.iter().zip(&poll.telemetry.frames) {
                assert_eq!(frame.sequence, timing.sequence);
            }
            actual.extend(poll.measurements.frames);
            timings.extend(poll.telemetry.frames);
        }
        let end = Instant::now() + Duration::from_secs(2);
        while (expected.len() < 2 || actual.len() < 2) && Instant::now() < end {
            std::thread::sleep(Duration::from_millis(2));
            expected.extend(service.poll(&mut field, plain).unwrap().frames);
            let poll = service.poll_profiled(&mut field, profiled).unwrap();
            actual.extend(poll.measurements.frames);
            timings.extend(poll.telemetry.frames);
        }
        assert_eq!(expected.len(), 2);
        assert_eq!(expected, actual, "telemetry changed simulated measurements");
        assert_eq!(timings.len(), 2);
        assert!(timings.iter().all(|t| t.work.input_brackets > 0
            && t.work.ingest_wall_ns > 0
            && t.work.input_queue_wait_ns >= t.work.input_queue_wait_max_ns));
        assert!(timings[1].work.batch > timings[0].work.batch);
    }
    #[test]
    fn profiled_poll_reports_queue_residence_and_stage_losses_without_draining_plain() {
        let mut field = field();
        let mut service = Service::default();
        let id = service
            .attach_profiled(&mut field, Config::default())
            .unwrap();
        let frame = Frame {
            sequence: 17,
            start_ns: 1_000_000_123,
            end_ns: 1_100_000_123,
            lidar: Default::default(),
            imu: vec![],
            discontinuities: 2,
        };
        let (send, receive) = mpsc::sync_channel(2);
        let a = service.attachments.get_mut(&id).unwrap();
        a.output = receive;
        a.drops.capture.store(2, Ordering::Relaxed);
        a.drops.ingest.store(3, Ordering::Relaxed);
        a.drops.output.store(4, Ordering::Relaxed);
        let ready = Instant::now() - Duration::from_millis(10);
        let work = AcquisitionWork {
            batch: 7,
            completed_frames: 2,
            input_brackets: 13,
            ..Default::default()
        };
        send.send(Completed {
            frame: frame.clone(),
            timing: Some((ready, work)),
        })
        .unwrap();
        send.send(Completed {
            frame: Frame {
                sequence: 18,
                ..frame.clone()
            },
            timing: Some((ready, work)),
        })
        .unwrap();
        let poll = service.poll_profiled(&mut field, id).unwrap();
        assert_eq!(poll.measurements.frames[0], frame);
        assert_eq!(poll.measurements.dropped_intervals, 9);
        assert_eq!(poll.telemetry.losses.capture_intervals, 2);
        assert_eq!(poll.telemetry.losses.ingest_errors, 3);
        assert_eq!(poll.telemetry.losses.output_frames, 4);
        assert_eq!(poll.telemetry.frames.len(), 2);
        assert!(
            poll.telemetry
                .frames
                .iter()
                .all(|t| t.output_queue_wait_ns >= 10_000_000 && t.work.batch == 7)
        );
        assert!(
            service
                .poll_profiled(&mut field, id)
                .unwrap()
                .measurements
                .frames
                .is_empty()
        );
        let plain = service.attach(&mut field, Config::default()).unwrap();
        let (send, receive) = mpsc::sync_channel(2);
        service.attachments.get_mut(&plain).unwrap().output = receive;
        send.send(Completed {
            frame: frame.clone(),
            timing: None,
        })
        .unwrap();
        assert!(service.poll_profiled(&mut field, plain).is_err());
        let poll = service.poll(&mut field, plain).unwrap();
        assert_eq!(poll.frames, vec![frame]);
        let json = serde_json::to_value(poll).unwrap();
        assert!(json.get("telemetry").is_none());
    }
}
