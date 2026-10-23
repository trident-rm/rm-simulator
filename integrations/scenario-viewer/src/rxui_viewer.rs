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
        let replace = size != self.size;
        let revision = &mut self.import_revision;
        let mut result = Ok(());
        self.runtime.update(|cx| {
            root.update(cx, |s, _| {
                if let Some(review) = &s.import {
                    if *revision != Some(review.revision) {
                        match review
                            .project
                            .preview(review.selected_group, review.preview)
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
                    viewport.draw(&s.plant, size, s.yaw, false);
                    // Group edits keep the camera target; resizing still replaces
                    // the exact GPU image view through the common path below.
                    if replace || s.image.is_none() {
                        match viewport.image() {
                            Ok(image) => s.image = Some(image),
                            Err(e) => result = Err(e),
                        }
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
                if replace || s.image.is_none() {
                    match viewport.image() {
                        Ok(image) => s.image = Some(image),
                        Err(e) => result = Err(e),
                    }
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
    let project = options
        .solidworks_project
        .as_ref()
        .map(|p| importer::Project::load(p))
        .transpose()?;
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
        let root = host.ui.entity().clone();
        host.runtime.update(|cx| {
            root.update(cx, |s, _| {
                s.plant.paused = true;
                let mut review = import_ui::Review::new(project, save);
                review.proxies = options.import_proxies;
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

#[cfg(test)]
mod tests {
    use super::*;
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
    }
}
