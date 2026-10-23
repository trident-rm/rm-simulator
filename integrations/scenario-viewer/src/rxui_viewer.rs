// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! In-process RXUI shell over a shared-device Bevy viewport.
#[path = "rxui_viewer/import_ui.rs"]
mod import_ui;
#[path = "rxui_viewer/importer.rs"]
mod importer;
#[path = "simulation.rs"]
mod simulation;
#[path = "rxui_viewer/viewport.rs"]
mod viewport;

#[path = "rxui_viewer/recovery.rs"]
mod recovery;

#[path = "rxui_viewer/mechanism.rs"]
mod mechanism;

use clap::Parser;
use rxui::{
    CommandId, UiPainter, WindowInput,
    astrelis::{Frame, FramebufferOptions, GraphicsContext, wgpu},
    astrelis_winit::{
        AppContext, Handler, PrepareAction, Runner, SurfaceSettings, WindowInfo,
        winit::{
            dpi::LogicalSize,
            event::WindowEvent,
            window::{Window, WindowId},
        },
    },
    prelude::*,
    standard_commands,
};
use simulation::Simulation;
use std::{
    error::Error,
    path::PathBuf,
    time::{Duration, Instant},
};
use viewport::Viewport;
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
#[command(
    about = "RXUI panels and a shared-device GPU physical viewport; no match or ROS required"
)]
struct Options {
    /// Review a prepared native SolidWorks occurrence project.
    #[arg(long)]
    solidworks_project: Option<PathBuf>,
    /// Source/target end centers and a transverse pin axis for one rigid group.
    #[arg(
        long,
        requires = "solidworks_project",
        conflicts_with = "joint_observations"
    )]
    mount_observation: Option<PathBuf>,
    /// Save an unreviewed rigid mount-fit proposal without a GPU or geometry scaling.
    #[arg(long, requires="mount_observation", conflicts_with_all=["recover_joints","export_solidworks","simulate_joints","mechanism_report","screenshot","frames"])]
    fit_mounts: Option<PathBuf>,
    /// Two saved parent/child poses per proposed joint, with explicit provenance.
    #[arg(long, requires = "solidworks_project")]
    joint_observations: Option<PathBuf>,
    /// Save inferred, unreviewed joints without a GPU. Does not guess travel stops.
    #[arg(long, requires="joint_observations", conflicts_with_all=["export_solidworks","screenshot","frames","simulate_joints","mechanism_report"])]
    recover_joints: Option<PathBuf>,
    /// Run reviewed joints with diagnostic motors; no gravity or contacts.
    #[arg(
        long,
        requires = "solidworks_project",
        conflicts_with = "export_solidworks"
    )]
    simulate_joints: bool,
    /// Headless explicit-tick motor test report, using --frames (four ticks each).
    #[arg(long, requires = "simulate_joints", conflicts_with_all = ["screenshot","paused","export_solidworks"])]
    mechanism_report: Option<PathBuf>,
    /// Initially show collision proxies instead of native display surfaces.
    #[arg(long, requires = "solidworks_project")]
    import_proxies: bool,
    /// Save reviewed source/group edits here (JSON sidecar, never native CAD).
    #[arg(long, requires = "solidworks_project")]
    save_project: Option<PathBuf>,
    /// Validate/cook/install and export a reviewed recipe without opening a GPU.
    #[arg(long, requires = "solidworks_project", conflicts_with_all = ["screenshot", "frames"])]
    export_solidworks: Option<PathBuf>,
    /// Save the complete GPU-rendered UI offscreen and exit.
    #[arg(long)]
    screenshot: Option<PathBuf>,
    /// Stop after this many presentations (four ticks each unless paused).
    #[arg(long, value_parser = clap::value_parser!(u32).range(1..=10000))]
    frames: Option<u32>,
    /// Start with automatic physics advance disabled.
    #[arg(long)]
    paused: bool,
}

struct Panel {
    import: Option<import_ui::Review>,
    plant: Simulation,
    image: Option<Image>,
    floor: bool,
    yaw: f32,
    error: Option<String>,
}
impl Panel {
    fn reset(&mut self) {
        self.error = self.plant.reset().err().map(str::to_owned);
    }
    fn step(&mut self) {
        self.plant.paused = true;
        self.error = self.plant.step().err().map(str::to_owned);
    }
}
impl View for Panel {
    fn view(&self, cx: &mut ViewContext<'_, Self>) -> impl IntoElement {
        if self.import.is_some() {
            return self.import_view(cx);
        }
        let cube = self.plant.pose(simulation::CUBE).translation_m;
        let sidebar = column()
            .width(248.)
            .fill_height()
            .scroll_y()
            .gap(16.)
            .padding(16.)
            .child(label("DISPLAYS").font_size(13.))
            .child(label(
                "Physical bodies · 3\nSource: local runtime\nCoordinates: FLU metres",
            ))
            .child(
                button(if self.floor {
                    "Hide floor"
                } else {
                    "Show floor"
                })
                .key("floor")
                .on_click(cx.listener(|s, _, _| s.floor = !s.floor)),
            )
            .child(label("INSPECTOR").font_size(13.))
            .child(label(format!(
                "Falling cube\nx  {:.3} m\ny  {:.3} m\nz  {:.3} m",
                cube[0], cube[1], cube[2]
            )))
            .child(label(
                "Privileged simulation truth\nNo sensor measurements yet",
            ))
            .child(label("RENDERING").font_size(13.))
            .child(label(
                "Shared wgpu device\nGPU texture → RXUI\nNo per-frame readback",
            ));
        let mut scene = column()
            .flex_grow(1.)
            .flex_basis(0.)
            .min_width(0.)
            .min_height(0.)
            .fill_height()
            .gap(8.)
            .child(
                row()
                    .gap(8.)
                    .child(
                        button("Orbit left")
                            .key("left")
                            .on_click(cx.listener(|s, _, _| s.yaw -= 0.2)),
                    )
                    .child(
                        button("Orbit right")
                            .key("right")
                            .on_click(cx.listener(|s, _, _| s.yaw += 0.2)),
                    ),
            );
        if let Some(source) = &self.image {
            scene = scene.child(
                image(source.clone())
                    .key("viewport")
                    .fill_width()
                    .flex_grow(1.)
                    .flex_basis(0.)
                    .min_width(0.)
                    .min_height(0.)
                    .fit(ImageFit::Contain),
            );
        }
        column()
            .fill_width()
            .fill_height()
            .padding(12.)
            .gap(10.)
            .child(
                row()
                    .gap(12.)
                    .child(label("rm-sim / Physical viewer").font_size(22.))
                    .child(
                        button(if self.plant.paused { "Resume" } else { "Pause" })
                            .key("play")
                            .on_click(cx.listener(|s, _, _| s.plant.paused = !s.plant.paused)),
                    )
                    .child(
                        button("Step")
                            .key("step")
                            .on_click(cx.listener(|s, _, _| s.step())),
                    )
                    .child(
                        button("Reset")
                            .key("reset")
                            .on_click(cx.listener(|s, _, _| s.reset())),
                    ),
            )
            .child(label(format!(
                "{}  ·  {:.3} s  ·  {} ticks  ·  128 Hz",
                if self.plant.paused {
                    "PAUSED"
                } else {
                    "RUNNING"
                },
                self.plant.tick as f64 / 128.,
                self.plant.tick
            )))
            .child(
                row()
                    .fill_width()
                    .flex_grow(1.)
                    .flex_basis(0.)
                    .min_height(0.)
                    .gap(12.)
                    .child(sidebar)
                    .child(scene),
            )
            .child(label(self.error.clone().unwrap_or_else(|| {
                "In process · generic physics + renderer · RXUI controls".into()
            })))
    }
}

/// Host-owned pacing; UI refresh never advances a second physics world.
#[derive(Default)]
struct Clock {
    pending: Duration,
}
impl Clock {
    fn ticks(&mut self, elapsed: Duration, paused: bool) -> u32 {
        if paused {
            self.pending = Duration::ZERO;
            return 0;
        }
        let tick = Duration::from_nanos(7_812_500);
        self.pending += elapsed.min(Duration::from_millis(250));
        let count = (self.pending.as_nanos() / tick.as_nanos()).min(32) as u32;
        self.pending -= tick * count;
        count
    }
}

struct Host {
    runtime: Runtime,
    ui: Ui<Panel>,
    painter: UiPainter,
    viewport: Viewport,
    input: WindowInput,
    window: Option<WindowId>,
    clock: Clock,
    last: Instant,
    last_tick: u64,
    last_paused: bool,
    frames: u32,
    limit: Option<u32>,
    size: [u32; 2],
    import_revision: Option<u64>,
}
impl Host {
    fn new(paused: bool, limit: Option<u32>) -> Result<(Self, GraphicsContext)> {
        let plant = Simulation::new(paused)?;
        let viewport = Viewport::new(&plant);
        let graphics = viewport.graphics();
        let mut painter = UiPainter::new(&graphics);
        if painter.fonts_mut().load_system_fonts().is_empty() {
            return Err("no system fonts available".into());
        }
        let mut runtime = Runtime::new();
        let root = runtime.update(|cx| {
            cx.new(|_| Panel {
                import: None,
                plant,
                image: None,
                floor: true,
                yaw: 0.65,
                error: None,
            })
        });
        let ui = Ui::new(&mut runtime, root)?;
        Ok((
            Self {
                runtime,
                ui,
                painter,
                viewport,
                input: WindowInput::new(),
                window: None,
                clock: Clock::default(),
                last: Instant::now(),
                last_tick: 0,
                last_paused: paused,
                frames: 0,
                limit,
                size: [0; 2],
                import_revision: None,
            },
            graphics,
        ))
    }

    fn prepare_ui(
        &mut self,
        logical: [f32; 2],
        scale: f32,
        format: &rxui::astrelis::RenderFormat,
    ) -> Result<()> {
        let elapsed = self.last.elapsed();
        self.last = Instant::now();
        let root = self.ui.entity().clone();
        let size = viewport::backing_size(logical, scale);
        let viewport = &mut self.viewport;
        let clock = &mut self.clock;
        let last_tick = self.last_tick;
        let last_paused = self.last_paused;
        let finite = self.limit.is_some();
        let revision = &mut self.import_revision;
        let mut result = Ok(());
        self.runtime.update(|cx| {
            root.update(cx, |s, _| {
                if let Some(review) = &mut s.import {
                    let reset = std::mem::take(&mut review.clock_reset);
                    if reset {
                        clock.pending = Duration::ZERO;
                    }
                    let count = if finite {
                        if review.physical_running { 4 } else { 0 }
                    } else {
                        clock.ticks(
                            if reset { Duration::ZERO } else { elapsed },
                            !review.physical_running,
                        )
                    };
                    if let Err(e) = review.advance_physics(count) {
                        review.physical_running = false;
                        s.error = Some(e);
                        return;
                    }
                    if *revision != Some(review.revision) {
                        match review
                            .project
                            .preview(
                                review.selected_group,
                                if review.playback.is_some() {
                                    0.
                                } else {
                                    review.preview
                                },
                            )
                            .and_then(|p| {
                                viewport.import_scene(&p, review.selected_group, review.proxies)
                            }) {
                            Ok(()) => {
                                *revision = Some(review.revision);
                                s.error = None;
                            }
                            Err(e) => {
                                *revision = Some(review.revision);
                                s.error = Some(e);
                                return;
                            }
                        }
                    }
                    if let Some(playback) = &review.playback
                        && let Err(e) = viewport.update_import_poses(&playback.poses)
                    {
                        s.error = Some(e);
                        return;
                    }
                    viewport.draw(&s.plant, size, s.yaw, false);
                    // Reuse the RXUI image unless Bevy replaces its native view.
                    match viewport.image() {
                        Ok(image) => s.image = Some(image),
                        Err(e) => result = Err(e),
                    }
                    return;
                }
                // Reset, explicit step and pause transitions discard accumulated time.
                let changed = s.plant.tick != last_tick || s.plant.paused != last_paused;
                if changed {
                    clock.pending = Duration::ZERO;
                }
                let count = if finite {
                    if s.plant.paused { 0 } else { 4 }
                } else {
                    clock.ticks(
                        if changed { Duration::ZERO } else { elapsed },
                        s.plant.paused,
                    )
                };
                if let Err(e) = s.plant.advance(count) {
                    s.error = Some(e.into());
                    s.plant.paused = true;
                }
                viewport.draw(&s.plant, size, s.yaw, s.floor);
                match viewport.image() {
                    Ok(image) => s.image = Some(image),
                    Err(e) => result = Err(e),
                }
                self.last_tick = s.plant.tick;
                self.last_paused = s.plant.paused;
            })
        });
        result?;
        self.size = size;
        self.runtime.flush()?;
        self.ui
            .prepare(&mut self.runtime, logical, &mut self.painter)?;
        self.painter.prepare(&self.ui, format, scale)?;
        Ok(())
    }

    fn paint(&mut self, frame: &mut Frame<'_, 'static>, scale: f32) -> Result<()> {
        self.painter.compose(&self.ui, frame, scale, |frame, ui| {
            let mut pass = frame
                .render_pass()
                .clear_color(wgpu::Color {
                    r: 0.025,
                    g: 0.035,
                    b: 0.055,
                    a: 1.,
                })
                .begin()?;
            ui.paint(&mut pass)
        })?;
        Ok(())
    }
}
impl Handler for Host {
    type Message = ();
    type Error = Box<dyn Error>;
    fn resumed(&mut self, cx: &mut AppContext<'_, ()>) -> Result<()> {
        if self.window.is_some() {
            return Ok(());
        }
        self.window = Some(
            cx.create_window(
                Window::default_attributes()
                    .with_title("rm-sim / RXUI viewer")
                    .with_inner_size(LogicalSize::new(1100., 760.)),
                SurfaceSettings::new(),
            )?,
        );
        Ok(())
    }
    fn window_event(
        &mut self,
        cx: &mut AppContext<'_, ()>,
        id: WindowId,
        event: WindowEvent,
    ) -> Result<()> {
        let window = cx.window(id).ok_or("missing native window")?;
        let result = self.input.handle(
            &mut self.runtime,
            &mut self.ui,
            &mut self.painter,
            window.metrics(),
            &event,
        )?;
        self.input
            .sync_window(&mut self.ui, &mut self.painter, window.window())?;
        if result.command == Some(CommandId::of::<standard_commands::CloseWindow>()) {
            cx.close_window(id)?;
        }
        Ok(())
    }
    fn prepare(&mut self, cx: &mut AppContext<'_, ()>, id: WindowId) -> Result<PrepareAction> {
        if self.limit.is_some_and(|limit| self.frames >= limit) {
            cx.close_window(id)?;
            return Ok(PrepareAction::Skip);
        }
        let window = cx.window(id).ok_or("missing native window")?;
        let size = window.metrics().logical_size();
        self.prepare_ui(
            [size.width as f32, size.height as f32],
            window.metrics().scale_factor() as f32,
            window.render_format().ok_or("missing surface format")?,
        )?;
        self.frames += 1;
        cx.request_redraw_at(id, Instant::now() + Duration::from_millis(16))?;
        Ok(PrepareAction::Render)
    }
    fn render(&mut self, window: WindowInfo<'_>, frame: &mut Frame<'_, 'static>) -> Result<()> {
        self.paint(frame, window.metrics().scale_factor() as f32)
    }
}

fn main() -> Result<()> {
    let options = Options::parse();
    let mut project = options
        .solidworks_project
        .as_ref()
        .map(|p| importer::Project::load(p))
        .transpose()?;
    if let Some(path) = &options.mount_observation {
        let input = recovery::mounts::Observation::load(path)?;
        let p = project.as_mut().unwrap();
        let fit = recovery::mounts::Alignment::fit(p, &input)?;
        fit.apply(p)?;
        println!(
            "Fitted group {}; closure {} m. Source-frame and attached-joint review invalidated.",
            fit.group, fit.center_error_m
        );
    }
    if let Some(output) = &options.fit_mounts {
        project.as_ref().unwrap().save(output)?;
        return Ok(());
    }
    if let Some(path) = &options.joint_observations {
        let input = recovery::Observations::load(path)?;
        let p = project.as_mut().unwrap();
        let report = recovery::Report::recover(p, &input)?;
        report.apply(p)?;
        println!(
            "Recovered {} proposals; inferred joints remain unreviewed",
            report.candidates.len()
        );
    }
    if let Some(output) = &options.recover_joints {
        project.as_ref().unwrap().save(output)?;
        return Ok(());
    }
    if let Some(output) = &options.mechanism_report {
        let p = project.as_ref().unwrap();
        let mut review = import_ui::Review::new(p.clone(), output.with_extension("reviewed.json"));
        review.start_physics()?;
        let sim = review.playback.as_mut().unwrap();
        let mut history = Vec::new();
        let frames = options.frames.unwrap_or(256);
        for frame in 0..frames {
            sim.advance(4)?;
            if frame % 8 == 0 || frame + 1 == frames {
                history.push(serde_json::json!({"tick":sim.tick,"joints":sim.readings()?,"body_poses":sim.poses}));
            }
        }
        importer::write_json(
            output,
            &serde_json::json!({"schema":"rm-constraint-playback-report-v1","no_gravity":true,"no_contacts":true,"motor_gains":"inertia-scaled diagnostic gains, not robot calibration","frames":frames,"step_duration_ns":rm_sim_physics::StepDuration::HZ_128.as_ns(),"history":history}),
        )?;
        return Ok(());
    }
    if let Some(output) = &options.export_solidworks {
        project.as_ref().unwrap().export(output)?;
        println!(
            "Reviewed assembly cooked and installed; recipe: {}",
            output.display()
        );
        return Ok(());
    }
    let count = options.frames.or(options.screenshot.as_ref().map(|_| 60));
    let (mut host, graphics) = Host::new(options.paused, count)?;
    if let Some(project) = project {
        let save = options.save_project.unwrap_or_else(|| {
            options
                .solidworks_project
                .as_ref()
                .unwrap()
                .with_extension("reviewed.json")
        });
        let mut review = import_ui::Review::new(project, save);
        review.proxies = options.import_proxies;
        if options.simulate_joints {
            review.start_physics()?;
            review.physical_running = !options.paused;
        }
        let root = host.ui.entity().clone();
        host.runtime.update(|cx| {
            root.update(cx, |s, _| {
                s.plant.paused = true;
                s.import = Some(review);
            })
        });
    }
    if let Some(path) = options.screenshot {
        capture(&mut host, &graphics, path, count.unwrap())
    } else {
        Runner::new()?.with_graphics(graphics).run(&mut host)?;
        Ok(())
    }
}

fn capture(host: &mut Host, graphics: &GraphicsContext, path: PathBuf, count: u32) -> Result<()> {
    let mut target = graphics.create_framebuffer(
        FramebufferOptions::new(1100, 760)
            .usage(wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC),
    )?;
    // GPU preparation must not advance the requested physical trajectory.
    let root = host.ui.entity().clone();
    let paused = host.runtime.update(|cx| {
        root.update(cx, |s, _| {
            let paused = s.plant.paused;
            s.plant.paused = true;
            let joints = s.import.as_ref().is_some_and(|r| r.physical_running);
            if let Some(r) = &mut s.import {
                r.physical_running = false;
            }
            (paused, joints)
        })
    });
    let start = Instant::now();
    let mut stable = 0;
    while stable < 8 {
        host.prepare_ui([1100., 760.], 1., &target.render_format())?;
        let error = host.runtime.update(|cx| root.read(cx).error.clone());
        if let Some(error) = error {
            return Err(error.into());
        }
        let mut frame = target.begin_frame()?;
        host.paint(&mut frame, 1.)?;
        frame.finish()?;
        stable = if host.viewport.ready() { stable + 1 } else { 0 };
        if start.elapsed() > Duration::from_secs(30) {
            return Err("viewport GPU preparation timed out".into());
        }
    }
    host.runtime.update(|cx| {
        root.update(cx, |s, _| {
            s.plant.paused = paused.0;
            if let Some(r) = &mut s.import {
                r.physical_running = paused.1;
            }
        })
    });
    for _ in 0..count {
        host.prepare_ui([1100., 760.], 1., &target.render_format())?;
        let mut frame = target.begin_frame()?;
        host.paint(&mut frame, 1.)?;
        frame.finish()?;
    }
    // Read back only the final complete UI capture, never the live viewport.
    let stride = (1100_u32 * 4).div_ceil(256) * 256;
    let buffer = graphics.device().create_buffer(&wgpu::BufferDescriptor {
        label: Some("one-shot viewer screenshot"),
        size: (stride * 760).into(),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let texture = target.color_texture()?.clone();
    let mut frame = target.begin_frame()?;
    frame.encoder().copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture: &texture,
            mip_level: 0,
            origin: Default::default(),
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(stride),
                rows_per_image: Some(760),
            },
        },
        wgpu::Extent3d {
            width: 1100,
            height: 760,
            depth_or_array_layers: 1,
        },
    );
    let submission = frame.finish()?;
    let (tx, rx) = std::sync::mpsc::channel();
    buffer.slice(..).map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    graphics.device().poll(wgpu::PollType::Wait {
        submission_index: Some(submission),
        timeout: Some(Duration::from_secs(30)),
    })?;
    rx.recv_timeout(Duration::from_secs(30))??;
    let bytes = buffer.slice(..).get_mapped_range()?;
    let pixels: Vec<u8> = bytes
        .chunks_exact(stride as usize)
        .flat_map(|row| row[..4400].iter().copied())
        .collect();
    let bounds = host
        .ui
        .elements()
        .find(|e| e.key == Some(&rxui::Key::from("viewport")))
        .ok_or("missing viewport in capture")?
        .bounds;
    if !visible_viewport(
        &pixels,
        [1100, 760],
        [bounds.x, bounds.y, bounds.width, bounds.height],
        host.size,
    ) {
        return Err("viewport capture is blank; refusing to save an empty preview".into());
    }
    let mut encoder = png::Encoder::new(std::fs::File::create(path)?, 1100, 760);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(&pixels)?;
    drop(bytes);
    buffer.unmap();
    println!(
        "Captured RXUI + Bevy on one GPU device; {} presentations; one final readback",
        count
    );
    Ok(())
}

/// Check the contained image, excluding UI gutters and letterboxing.
fn visible_viewport(pixels: &[u8], output: [u32; 2], bounds: [f32; 4], image: [u32; 2]) -> bool {
    let fit = (bounds[2] / image[0] as f32).min(bounds[3] / image[1] as f32);
    let width = image[0] as f32 * fit;
    let height = image[1] as f32 * fit;
    let x0 = bounds[0] + (bounds[2] - width) * 0.5;
    let y0 = bounds[1] + (bounds[3] - height) * 0.5;
    let xs = (x0.ceil() as usize + 2)
        ..((x0 + width).floor() as usize)
            .saturating_sub(2)
            .min(output[0] as usize);
    let ys = (y0.ceil() as usize + 2)
        ..((y0 + height).floor() as usize)
            .saturating_sub(2)
            .min(output[1] as usize);
    ys.into_iter().any(|y| {
        xs.clone().any(|x| {
            let offset = (y * output[0] as usize + x) * 4;
            pixels[offset..offset + 3].iter().any(|v| *v > 32)
        })
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn blank_capture_check_excludes_bright_ui_and_letterboxing() {
        let mut pixels = vec![255; 100 * 100 * 4];
        for y in 35..65 {
            for x in 20..80 {
                pixels[(y * 100 + x) * 4..(y * 100 + x) * 4 + 3].fill(0);
            }
        }
        assert!(!visible_viewport(
            &pixels,
            [100, 100],
            [20., 20., 60., 60.],
            [120, 60]
        ));
        pixels[(50 * 100 + 50) * 4] = 180;
        assert!(visible_viewport(
            &pixels,
            [100, 100],
            [20., 20., 60., 60.],
            [120, 60]
        ));
    }

    #[test]
    fn pacing_and_backing_sizes_are_bounded() {
        let mut clock = Clock::default();
        assert_eq!(clock.ticks(Duration::from_secs(1), false), 32);
        assert_eq!(clock.ticks(Duration::from_secs(1), true), 0);
        assert_eq!(clock.ticks(Duration::from_nanos(7_812_500), false), 1);
        assert_eq!(viewport::backing_size([0., 0.], 2.), [64, 64]);
        assert_eq!(viewport::backing_size([1100., 760.], 1.), [812, 620]);
        assert_eq!(viewport::backing_size([10000., 10000.], 2.), [2048, 2048]);
    }

    #[test]
    fn controls_share_the_same_plant_and_reset_preserves_pause() {
        let mut panel = Panel {
            import: None,
            plant: Simulation::new(false).unwrap(),
            image: None,
            floor: true,
            yaw: 0.65,
            error: None,
        };
        panel.step();
        assert!(panel.plant.paused);
        assert_eq!(panel.plant.tick, 1);
        panel.reset();
        assert!(panel.plant.paused);
        assert_eq!(panel.plant.tick, 0);
        assert!(panel.error.is_none());
    }

    #[test]
    #[ignore = "requires a native GPU and system fonts"]
    fn physical_import_keeps_gpu_resources_and_capture_preserves_tick_budget() {
        use bevy::prelude::{Mesh3d, MeshMaterial3d, StandardMaterial, Transform};
        use rm_sim_render::generic::{VisualId, Visuals};
        let (mut host, graphics) = Host::new(true, Some(2)).unwrap();
        let root = host.ui.entity().clone();
        let mut r = import_ui::Review::new(mechanism::tests::source(), "unused.json".into());
        r.start_physics().unwrap();
        r.playback.as_mut().unwrap().target(2, 0.4).unwrap();
        host.runtime
            .update(|cx| root.update(cx, |s, _| s.import = Some(r)));
        let target = graphics
            .create_framebuffer(FramebufferOptions::new(1100, 760))
            .unwrap();
        host.prepare_ui([1100., 760.], 1., &target.render_format())
            .unwrap();
        let world = host.viewport.app.world();
        let e = world.resource::<Visuals>().entity(VisualId(1001)).unwrap();
        let mesh = world.get::<Mesh3d>(e).unwrap().0.clone();
        let material = world
            .get::<MeshMaterial3d<StandardMaterial>>(e)
            .unwrap()
            .0
            .clone();
        let before = *world.get::<Transform>(e).unwrap();
        for _ in 0..64 {
            host.prepare_ui([1100., 760.], 1., &target.render_format())
                .unwrap();
        }
        let world = host.viewport.app.world();
        assert_eq!(world.resource::<Visuals>().entity(VisualId(1001)), Some(e));
        assert_eq!(world.get::<Mesh3d>(e).unwrap().0, mesh);
        assert_eq!(
            world.get::<MeshMaterial3d<StandardMaterial>>(e).unwrap().0,
            material
        );
        assert_ne!(*world.get::<Transform>(e).unwrap(), before);
        let tick = host.runtime.update(|cx| {
            root.read(cx)
                .import
                .as_ref()
                .unwrap()
                .playback
                .as_ref()
                .unwrap()
                .tick
        });
        let path =
            std::env::temp_dir().join(format!("rm-joints-capture-{}.png", std::process::id()));
        capture(&mut host, &graphics, path.clone(), 2).unwrap();
        host.runtime.update(|cx| {
            assert_eq!(
                root.read(cx)
                    .import
                    .as_ref()
                    .unwrap()
                    .playback
                    .as_ref()
                    .unwrap()
                    .tick,
                tick + 8
            )
        });
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    #[ignore = "requires a native GPU and system fonts"]
    fn shared_gpu_resize_retains_assets_and_control_state() {
        let (mut host, graphics) = Host::new(false, Some(8)).unwrap();
        let target = graphics
            .create_framebuffer(FramebufferOptions::new(1100, 760))
            .unwrap();
        let root = host.ui.entity().clone();
        host.prepare_ui([1100., 760.], 1., &target.render_format())
            .unwrap();
        let bounds = host
            .ui
            .elements()
            .find(|e| e.key == Some(&rxui::Key::from("viewport")))
            .unwrap()
            .bounds;
        assert!(bounds.x + bounds.width <= 1100. && bounds.y + bounds.height <= 760.);
        let image = host
            .runtime
            .update(|cx| root.read(cx).image.clone().unwrap());
        let mesh_count = host
            .viewport
            .app
            .world()
            .resource::<bevy::prelude::Assets<bevy::prelude::Mesh>>()
            .len();
        host.prepare_ui([1100., 760.], 1., &target.render_format())
            .unwrap();
        host.runtime.update(|cx| {
            assert_eq!(root.read(cx).image.as_ref().unwrap().id(), image.id());
            assert_eq!(root.read(cx).plant.tick, 8);
            root.update(cx, |s, _| s.plant.paused = true);
        });
        host.prepare_ui([900., 640.], 1., &target.render_format())
            .unwrap();
        let bounds = host
            .ui
            .elements()
            .find(|e| e.key == Some(&rxui::Key::from("viewport")))
            .unwrap()
            .bounds;
        assert!(bounds.x + bounds.width <= 900. && bounds.y + bounds.height <= 640.);
        host.runtime.update(|cx| {
            let state = root.read(cx);
            assert_eq!(state.plant.tick, 8);
            assert_eq!(state.image.as_ref().unwrap().pixel_size(), Some([612, 500]));
            assert_ne!(state.image.as_ref().unwrap().id(), image.id());
        });
        // Replace the render asset without changing the requested size. RXUI must
        // rebind the new native view, rather than keeping the stale sampled image.
        let resized_image = host
            .runtime
            .update(|cx| root.read(cx).image.clone().unwrap());
        host.viewport.replace_target_for_test();
        for _ in 0..3 {
            host.prepare_ui([900., 640.], 1., &target.render_format())
                .unwrap();
        }
        host.runtime.update(|cx| {
            assert_ne!(
                root.read(cx).image.as_ref().unwrap().id(),
                resized_image.id()
            );
            assert_eq!(root.read(cx).plant.tick, 8);
        });
        assert!(graphics.same_device(&host.viewport.graphics()));
        assert_eq!(
            host.viewport
                .app
                .world()
                .resource::<bevy::prelude::Assets<bevy::prelude::Mesh>>()
                .len(),
            mesh_count
        );
        host.runtime.update(|cx| {
            root.update(cx, |s, _| {
                s.step();
                s.reset();
            })
        });
        host.prepare_ui([900., 640.], 1., &target.render_format())
            .unwrap();
        host.runtime
            .update(|cx| assert_eq!(root.read(cx).plant.tick, 0));
        host.runtime
            .update(|cx| root.update(cx, |s, _| s.plant.paused = false));
        let capture_path =
            std::env::temp_dir().join(format!("rm-sw2-test-capture-{}.png", std::process::id()));
        capture(&mut host, &graphics, capture_path.clone(), 2).unwrap();
        host.runtime
            .update(|cx| assert_eq!(root.read(cx).plant.tick, 8));
        assert!(std::fs::metadata(&capture_path).unwrap().len() > 1000);
        std::fs::remove_file(capture_path).unwrap();
    }

    #[test]
    #[ignore = "requires a native GPU and system fonts"]
    fn imported_occurrences_share_gpu_meshes_without_sharing_pose_or_color() {
        use bevy::prelude::{Mesh3d, MeshMaterial3d, StandardMaterial, Transform};
        use rm_sim_render::generic::{VisualId, Visuals};
        let (mut host, _) = Host::new(true, None).unwrap();
        let mut project = importer::tests::shared_fixture();
        let mut group = project.groups[0].clone();
        group.id = 2;
        project.groups.push(group);
        project.parts[1].group = 2;
        project.validate().unwrap();
        host.viewport.import_scene(&project, 1, false).unwrap();
        let world = host.viewport.app.world();
        let visuals = world.resource::<Visuals>();
        let first = visuals.entity(VisualId(1000)).unwrap();
        let second = visuals.entity(VisualId(1001)).unwrap();
        assert_ne!(first, second);
        assert_eq!(
            world.get::<Mesh3d>(first).unwrap().0,
            world.get::<Mesh3d>(second).unwrap().0
        );
        assert_ne!(
            world
                .get::<MeshMaterial3d<StandardMaterial>>(first)
                .unwrap()
                .0,
            world
                .get::<MeshMaterial3d<StandardMaterial>>(second)
                .unwrap()
                .0
        );
        assert_ne!(
            world.get::<Transform>(first).unwrap().translation,
            world.get::<Transform>(second).unwrap().translation
        );
    }
}
