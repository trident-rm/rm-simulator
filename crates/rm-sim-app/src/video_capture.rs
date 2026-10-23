// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Continuous console recording: bounded asynchronous readbacks and an encoder
//! worker, with wall-time resampling and explicit duplicate/drop counters.
use bevy::{
    prelude::*,
    render::view::screenshot::{Screenshot, ScreenshotCaptured},
};
use serde::Serialize;
use std::{
    io::Write,
    path::PathBuf,
    process::{Command, Stdio},
    sync::{
        Arc, Mutex,
        atomic::{AtomicU64, Ordering},
        mpsc::{self, Receiver, SyncSender},
    },
    time::Instant,
};

const IN_FLIGHT: usize = 6;
const QUEUE: usize = 3;
const MAX_DURATION_S: u64 = 600;

/// Outcome of a finalized MP4, distinguishing real captures from held frames.
#[derive(Clone, Debug, Serialize)]
pub struct Report {
    path: PathBuf,
    fps: u32,
    width: u32,
    height: u32,
    elapsed_s: f64,
    unique_frames: u64,
    encoded_frames: u64,
    duplicated_frames: u64,
    skipped_capture_slots: u64,
    queue_drops: u64,
    readback_limit_skips: u64,
    out_of_order_frames: u64,
    // Accepted source slots and renderer presentation timestamps. Bounded by
    // the ten-minute recording limit (36k entries at 60 fps).
    samples: Vec<(u64, Option<u64>)>,
}
struct Frame {
    slot: u64,
    presentation_time_ns: Option<u64>,
    pixels: Vec<u8>,
    bgra: bool,
}
struct Active {
    started: Instant,
    next_slot: u64,
    pending: usize,
    stopping: bool,
    sender: Option<SyncSender<Frame>>,
    result: Mutex<Receiver<Result<Report, String>>>,
    end_slot: Arc<AtomicU64>,
    report: Report,
    failure: Option<String>,
}
/// One recording at a time. Encoding and finalization never run on the app thread.
#[derive(Resource, Default)]
pub struct Recorder {
    active: Option<Active>,
    /// Final success report or encoder failure, retained until the next start.
    pub result: Option<Result<Report, String>>,
}
impl Recorder {
    /// Start a new file at an immutable even resolution and 24/30/60 fps.
    pub fn start(
        &mut self,
        path: PathBuf,
        fps: u32,
        width: u32,
        height: u32,
    ) -> Result<(), String> {
        if self.active.is_some() {
            return Err("a video is already recording or finalizing".into());
        }
        if !matches!(fps, 24 | 30 | 60)
            || width == 0
            || height == 0
            || !width.is_multiple_of(2)
            || !height.is_multiple_of(2)
        {
            return Err("video requires 24/30/60 fps and positive even pixel dimensions".into());
        }
        if path.extension().and_then(|x| x.to_str()) != Some("mp4") || path.exists() {
            return Err("video needs a new .mp4 path (existing files are preserved)".into());
        }
        let mut child = Command::new("ffmpeg")
            .args([
                "-n",
                "-v",
                "error",
                "-f",
                "rawvideo",
                "-pixel_format",
                "rgba",
                "-video_size",
            ])
            .arg(format!("{width}x{height}"))
            .args([
                "-framerate",
                &fps.to_string(),
                "-i",
                "pipe:0",
                "-an",
                "-c:v",
                "libx264",
                "-threads",
                "2",
                "-preset",
                "fast",
                "-crf",
                "20",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
            ])
            .arg(&path)
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .spawn()
            .map_err(|e| format!("starting video encoder: {e}"))?;
        let stdin = child.stdin.take().expect("piped stdin");
        let report = Report {
            path,
            fps,
            width,
            height,
            elapsed_s: 0.,
            unique_frames: 0,
            encoded_frames: 0,
            duplicated_frames: 0,
            skipped_capture_slots: 0,
            queue_drops: 0,
            readback_limit_skips: 0,
            out_of_order_frames: 0,
            samples: Vec::new(),
        };
        let mut output = report.clone();
        let (sender, frames) = mpsc::sync_channel::<Frame>(QUEUE);
        let (reply, result) = mpsc::channel();
        let end_slot = Arc::new(AtomicU64::new(0));
        let end = end_slot.clone();
        std::thread::spawn(move || {
            let encode = || -> Result<Report, String> {
                let mut timeline = Timeline::new(stdin);
                for mut frame in frames {
                    // Keep pixel conversion off Bevy's frame thread.
                    if frame.bgra {
                        for pixel in frame.pixels.as_chunks_mut::<4>().0 {
                            pixel.swap(0, 2);
                        }
                    }
                    let accepted = frame.slot >= timeline.next;
                    timeline
                        .push(frame.slot, frame.pixels)
                        .map_err(|e| e.to_string())?;
                    if accepted {
                        output
                            .samples
                            .push((frame.slot, frame.presentation_time_ns));
                    }
                }
                timeline
                    .finish(end.load(Ordering::Acquire))
                    .map_err(|e| e.to_string())?;
                output.unique_frames = timeline.unique;
                output.encoded_frames = timeline.next;
                output.duplicated_frames = timeline.next - timeline.unique;
                output.out_of_order_frames = timeline.out_of_order;
                drop(timeline); // close stdin before waiting for MP4 finalization
                Ok(output)
            };
            let mut outcome = encode();
            if outcome.is_err() {
                let _ = child.kill();
            }
            match child.wait() {
                Ok(status) if status.success() => {}
                status => outcome = Err(format!("video encoder failed: {status:?}")),
            }
            let _ = reply.send(outcome);
        });
        self.result = None;
        self.active = Some(Active {
            started: Instant::now(),
            next_slot: 0,
            pending: 0,
            stopping: false,
            sender: Some(sender),
            result: Mutex::new(result),
            end_slot,
            report,
            failure: None,
        });
        Ok(())
    }
    /// Stop submissions; pending readbacks and MP4 finalization drain asynchronously.
    pub fn stop(&mut self) -> Result<(), String> {
        let a = self.active.as_mut().ok_or("no video is recording")?;
        if !a.stopping {
            a.stopping = true;
            a.report.elapsed_s = a.started.elapsed().as_secs_f64().min(MAX_DURATION_S as f64);
            a.end_slot.store(
                (a.report.elapsed_s * a.report.fps as f64).ceil() as u64,
                Ordering::Release,
            );
        }
        Ok(())
    }
    /// Compact console diagnostics; actual encoded/unique counts arrive on stop.
    pub fn status(&self) -> serde_json::Value {
        self.active.as_ref().map_or_else(|| serde_json::json!({"active":false, "result":self.result}), |a| serde_json::json!({"active":true,"finalizing":a.stopping,"fps":a.report.fps,"pending_readbacks":a.pending,"skipped_capture_slots":a.report.skipped_capture_slots,"queue_drops":a.report.queue_drops}))
    }
}

/// Resamples request timestamps to constant-rate output without hiding held frames.
struct Timeline<W> {
    writer: W,
    last: Option<Vec<u8>>,
    next: u64,
    unique: u64,
    out_of_order: u64,
}
impl<W: Write> Timeline<W> {
    fn new(writer: W) -> Self {
        Self {
            writer,
            last: None,
            next: 0,
            unique: 0,
            out_of_order: 0,
        }
    }
    fn push(&mut self, slot: u64, pixels: Vec<u8>) -> std::io::Result<()> {
        if slot < self.next {
            self.out_of_order += 1;
            return Ok(());
        }
        let fill = self.last.as_deref().unwrap_or(&pixels);
        while self.next < slot {
            self.writer.write_all(fill)?;
            self.next += 1;
        }
        self.writer.write_all(&pixels)?;
        self.next += 1;
        self.unique += 1;
        self.last = Some(pixels);
        Ok(())
    }
    fn finish(&mut self, end: u64) -> std::io::Result<()> {
        let Some(last) = &self.last else {
            return Err(std::io::Error::other("video received no frames"));
        };
        while self.next < end {
            self.writer.write_all(last)?;
            self.next += 1;
        }
        self.writer.flush()
    }
}

/// Capture the current view without PNG encoding or console round trips per frame.
pub struct VideoCapturePlugin;
impl Plugin for VideoCapturePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Recorder>()
            .add_systems(PostUpdate, capture);
    }
}
fn capture(
    mut commands: Commands,
    mut recorder: ResMut<Recorder>,
    target: Res<crate::presentation::CaptureTarget>,
    screen: Res<State<crate::loading::Screen>>,
    session: Option<Res<crate::session::Session>>,
) {
    if recorder.active.as_ref().is_some_and(|a| {
        a.started.elapsed().as_secs() >= MAX_DURATION_S
            || *screen.get() != crate::loading::Screen::InMatch
    }) {
        let _ = recorder.stop();
    }
    let Some(a) = &mut recorder.active else {
        return;
    };
    if a.stopping && a.pending == 0 {
        a.sender.take();
    }
    let result = a
        .result
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .try_recv();
    match result {
        Ok(result) => {
            let result = result.and_then(|mut report| {
                report.elapsed_s = a.report.elapsed_s;
                report.skipped_capture_slots = a.report.skipped_capture_slots;
                report.queue_drops = a.report.queue_drops;
                report.readback_limit_skips = a.report.readback_limit_skips;
                if let Some(e) = a.failure.take() {
                    Err(e)
                } else {
                    Ok(report)
                }
            });
            recorder.result = Some(result);
            recorder.active = None;
            return;
        }
        Err(mpsc::TryRecvError::Disconnected) => {
            recorder.result = Some(Err("video worker disconnected".into()));
            recorder.active = None;
            return;
        }
        Err(mpsc::TryRecvError::Empty) => {}
    }
    if a.stopping {
        return;
    }
    let slot = (a.started.elapsed().as_secs_f64() * a.report.fps as f64).floor() as u64;
    if slot < a.next_slot {
        return;
    }
    a.report.skipped_capture_slots += slot - a.next_slot;
    a.next_slot = slot + 1;
    if a.pending >= IN_FLIGHT {
        a.report.readback_limit_skips += 1;
        a.report.skipped_capture_slots += 1;
        return;
    }
    a.pending += 1;
    let shot = target
        .0
        .as_ref()
        .map_or_else(Screenshot::primary_window, |h| Screenshot::image(h.clone()));
    let presentation_time_ns = session.as_ref().map(|s| s.presentation_time_ns());
    commands.spawn(shot).observe(
        move |mut frame: On<ScreenshotCaptured>, mut recorder: ResMut<Recorder>| {
            let Some(a) = &mut recorder.active else {
                return;
            };
            a.pending = a.pending.saturating_sub(1);
            let image = &mut frame.image;
            let size = image.texture_descriptor.size;
            let bytes = image.data.as_ref();
            if size.width != a.report.width
                || size.height != a.report.height
                || bytes.is_none_or(|v| {
                    v.len() != a.report.width as usize * a.report.height as usize * 4
                })
            {
                a.failure =
                    Some("capture size/format changed; stop recording before resizing".into());
                a.stopping = true;
                return;
            }
            use bevy::render::render_resource::TextureFormat;
            let bgra = match image.texture_descriptor.format {
                TextureFormat::Rgba8Unorm | TextureFormat::Rgba8UnormSrgb => false,
                TextureFormat::Bgra8Unorm | TextureFormat::Bgra8UnormSrgb => true,
                _ => {
                    a.failure = Some("video requires an RGBA8/BGRA8 target".into());
                    a.stopping = true;
                    return;
                }
            };
            // This private screenshot entity has this single observer. Transfer
            // the readback allocation rather than copying another full frame.
            let pixels = image.data.take().expect("validated screenshot bytes");
            if let Some(sender) = &a.sender {
                match sender.try_send(Frame {
                    slot,
                    presentation_time_ns,
                    pixels,
                    bgra,
                }) {
                    Ok(()) => {}
                    Err(mpsc::TrySendError::Full(_)) => a.report.queue_drops += 1,
                    Err(mpsc::TrySendError::Disconnected(_)) => {
                        a.failure = Some("video encoder stopped".into());
                        a.stopping = true;
                    }
                }
            }
        },
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn missing_and_late_frames_keep_duration_and_report_duplicates() {
        let mut t = Timeline::new(Vec::new());
        t.push(1, vec![1]).unwrap();
        t.push(4, vec![2]).unwrap();
        t.push(3, vec![9]).unwrap();
        t.finish(7).unwrap();
        assert_eq!(t.writer, [1, 1, 1, 1, 2, 2, 2]);
        assert_eq!((t.next, t.unique, t.out_of_order), (7, 2, 1));
    }
    #[test]
    fn empty_capture_fails_and_encoder_errors_propagate() {
        assert!(Timeline::new(Vec::new()).finish(1).is_err());
        let mut bytes = [0u8; 1];
        let mut t = Timeline::new(std::io::Cursor::new(&mut bytes[..]));
        assert!(t.push(0, vec![1, 2]).is_err());
    }
}
