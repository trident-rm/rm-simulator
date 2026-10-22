// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! RXUI adapter: controls mutate the backend; rendering receives only pixels.
use crate::{
    model::Inspector,
    raster::{self, Options, Plane},
};
use rxui::prelude::*;
use std::time::Duration;

struct App {
    model: Inspector,
    options: Options,
    image: Image,
    error: Option<String>,
    timer: Option<Task>,
    loading: Option<Task>,
}
impl App {
    fn refresh(&mut self) {
        match Image::from_rgba8(
            raster::WIDTH,
            raster::HEIGHT,
            raster::render(&self.model, &self.options),
        ) {
            Ok(image) => self.image = image,
            Err(e) => self.error = Some(e.to_string()),
        }
    }
    fn apply(&mut self, action: impl FnOnce(&mut Inspector) -> anyhow::Result<()>) {
        self.error = action(&mut self.model).err().map(|e| e.to_string());
        self.refresh();
    }
    fn schedule(&mut self, cx: &mut Context<'_, Self>) {
        if !self.model.timeline.playing {
            self.timer = None;
            return;
        }
        self.timer = Some(cx.spawn(
            async {
                rxui::sleep(Duration::from_millis(125)).await;
            },
            |this, outcome, cx| {
                this.timer = None;
                if outcome.is_ok() && this.model.timeline.playing {
                    this.apply(|m| m.pulse());
                    if this.error.is_some() {
                        this.model.timeline.playing = false;
                    }
                    this.schedule(cx);
                }
            },
        ));
    }
    fn choose(&mut self, index: usize, placement: usize, cx: &mut Context<'_, Self>) {
        self.model.timeline.playing = false;
        self.timer = None;
        let overlay = self.model.overlay_zones.clone();
        let overlay_source = self.model.overlay_source.clone();
        let root = self.model.cad.root.clone();
        let name = self.model.names[index].clone();
        // Loading/verification and mesh parsing do not block native event routing.
        self.loading = Some(cx.spawn_blocking(
            move || {
                let mut model = Inspector::load(&root, &name, placement)?;
                model.overlay_zones = overlay;
                model.overlay_source = overlay_source;
                Ok::<_, anyhow::Error>(model)
            },
            |this, result, _| {
                this.loading = None;
                match result {
                    Ok(Ok(model)) => {
                        this.model = model;
                        this.error = None;
                        this.refresh();
                    }
                    Ok(Err(e)) => this.error = Some(e.to_string()),
                    Err(e) => this.error = Some(e.to_string()),
                }
            },
        ));
    }
}
impl View for App {
    fn view(&self, cx: &mut ViewContext<'_, Self>) -> impl IntoElement {
        let mut assets = row().gap(6.).fill_width().height(44.).scroll_x();
        for (index, name) in self.model.names.iter().enumerate() {
            assets = assets.child(
                button(name.clone())
                    .key(format!("asset-{index}"))
                    .disabled(self.loading.is_some())
                    .on_click(cx.listener(move |s, _, cx| s.choose(index, 0, cx))),
            );
        }
        let mut placements = row().gap(6.);
        for i in 0..self.model.asset().placements.len().max(1) {
            placements = placements.child(
                button(format!("Placement {i}"))
                    .key(format!("place-{i}"))
                    .disabled(self.loading.is_some())
                    .on_click(cx.listener(move |s, _, cx| s.choose(s.model.selected, i, cx))),
            );
        }
        let mut channels = column().gap(8.);
        for (i, part) in self.model.placed.moving.iter().enumerate() {
            let rotational = matches!(
                part.motion,
                rm_simulator_physics::motion::MotionFrame::Rotation { .. }
            );
            let unit = if rotational { "rad" } else { "fraction" };
            channels = channels
                .child(label(format!(
                    "Channel {i}: {:?} / {:?} = {:.4} {unit}",
                    part.mechanism,
                    part.team,
                    self.model.coordinate(i)
                )))
                .child(label(format!("World motion frame: {:?}", part.motion)))
                .child(
                    slider(self.model.coordinates[i] as f32)
                        .key(format!("coordinate-{i}"))
                        .range(
                            if rotational {
                                -std::f32::consts::PI
                            } else {
                                0.
                            },
                            if rotational { std::f32::consts::PI } else { 1. },
                        )
                        .width(380.)
                        .accessibility_label(format!("Channel {i} t=0 coordinate {unit}"))
                        .on_change(cx.listener(move |s, e: &SliderEvent, _| {
                            s.timer = None;
                            s.apply(|m| m.set_coordinate(i, e.value as f64));
                        })),
                );
        }
        let mut projections = row().gap(8.);
        for plane in [Plane::Top, Plane::Front, Plane::Side] {
            projections = projections.child(button(plane.label()).key(plane.label()).on_click(
                cx.listener(move |s, _, _| {
                    s.options.plane = plane;
                    s.refresh();
                }),
            ));
        }
        column().padding(20.).gap(10.).fill_width().fill_height().scroll_y()
            .child(label("Component / scenario inspector").font_size(26.))
            .child(label("Verified CAD collider projection • RXUI • world FLU metres").color(ThemeColor::TextMuted))
            .child(assets).child(placements)
            .child(row().gap(8.)
                .child(button(if self.model.timeline.playing {"Pause"} else {"Play preview"}).key("play")
                    .disabled(self.loading.is_some())
                    .on_click(cx.listener(|s,_,cx| { s.model.timeline.playing = !s.model.timeline.playing; s.schedule(cx); })))
                .child(button("Step 1 tick").key("step").on_click(cx.listener(|s,_,_| { s.model.timeline.playing=false; s.timer=None; s.apply(|m|m.step(1)); })))
                .child(button("Reset").key("reset").on_click(cx.listener(|s,_,_| { s.timer=None; s.apply(|m| m.reset()); })))
                .child(label(format!("{} ns / {:.6} s",self.model.timeline.time_ns,self.model.timeline.time_ns as f64/1e9))))
            .child(label("Inspection time: 7.8125 ms/tick. Play adds 16 ticks/pulse. Illustrative motion: 0.5 rad/s or 0.1 fraction/s."))
            .child(projections)
            .child(row().gap(12.)
                .child(checkbox(self.options.fixed,"Fixed colliders").key("fixed").on_change(cx.listener(|s,e: &ToggleEvent,_| {s.options.fixed=e.checked; s.refresh();})))
                .child(checkbox(self.options.moving,"Moving colliders").key("moving").on_change(cx.listener(|s,e: &ToggleEvent,_| {s.options.moving=e.checked; s.refresh();})))
                .child(checkbox(self.options.detectors,"Detector frames").key("detectors").on_change(cx.listener(|s,e: &ToggleEvent,_| {s.options.detectors=e.checked; s.refresh();})))
                .child(checkbox(self.options.rfid,"RFID bands").key("rfid").on_change(cx.listener(|s,e: &ToggleEvent,_| {s.options.rfid=e.checked; s.refresh();})))
                .child(checkbox(self.options.field,"Fit field footprints").key("field").on_change(cx.listener(|s,e: &ToggleEvent,_| {s.options.field=e.checked; s.refresh();}))))
            .child(row().gap(12.).child(label("Zoom"))
                .child(slider(self.options.zoom as f32).key("zoom").range(0.5,4.).width(300.).accessibility_label("Projection zoom")
                    .on_change(cx.listener(|s,e: &SliderEvent,_| {s.options.zoom=e.value as f64; s.refresh();}))))
            .child(image(self.image.clone()).key("projection").width(960.).height(640.).fit(ImageFit::Contain)
                .accessibility_label("Orthographic collider wire projection, fixed teal, moving amber, RFID purple; detector axes red green blue"))
            .child(label(format!("{} • teal fixed {} triangles • amber moving {} triangles / {} channels • {} detector frames • {} RFID footprints",
                self.options.plane.label(),self.model.placed.fixed.triangles.len(),self.model.placed.moving.iter().map(|p|p.mesh.triangles.len()).sum::<usize>(),self.model.placed.moving.len(),self.model.detectors().len(),self.model.zones().len())))
            .child(label(if self.model.zones().is_empty() {"RFID unavailable in this source package: import caller geometry with --rfid ZONES_JSON."} else {"Purple: RFID footprint and vertical band; caller overlays are unverified inspection geometry."}))
            .child(channels)
            .child(label("Sliders restart at t=0 with the chosen channel offset and pause. Reset clears time and all offsets. Fit follows each pose; there is no perspective or occlusion."))
            .child(label(format!("Visual source: {}\nCollider source: {}\nPlacement root (basis included): {:?}",self.model.cad.root.join(&self.model.asset().file).display(),self.model.asset().physics_file(&self.model.cad.root).display(),self.model.root())))
            .child(label(format!("Source STEP SHA-256: {}",self.model.cad.rmuc_2027.as_ref().map(|b|b.source_sha256.as_str()).unwrap_or("no native binding"))))
            .child(label(if self.loading.is_some() {"Loading verified geometry…".into()} else {self.error.clone().unwrap_or_default()}))
            .child(label("Wireframe only: source materials/textures are not rendered. Source presets step an empty RobotSession. No robot properties, RF simulation or match authority.").color(ThemeColor::TextMuted))
    }
}
pub fn run(model: Inspector, options: Options) -> anyhow::Result<()> {
    let image = Image::from_rgba8(
        raster::WIDTH,
        raster::HEIGHT,
        raster::render(&model, &options),
    )
    .map_err(|e| anyhow::anyhow!(e.to_string()))?;
    Application::new()
        .run(move |cx| {
            let state = cx.new(|_| App {
                model,
                options,
                image,
                error: None,
                timer: None,
                loading: None,
            });
            cx.open_window(
                WindowOptions::new()
                    .title("RM scenario inspector — RXUI")
                    .size(1120., 1000.),
                state,
            )?;
            Ok(())
        })
        .map_err(|e| anyhow::anyhow!(e.to_string()))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    struct Measure;
    impl rxui::TextMeasure for Measure {
        fn measure(
            &mut self,
            _: rxui::ElementId,
            text: rxui::TextRequest<'_>,
        ) -> Result<[f32; 2], rxui::UiError> {
            let intrinsic = text.text.chars().count() as f32 * text.font_size / 2.;
            let width = match text.width {
                rxui::TextWidth::Available(w) => intrinsic.min(w),
                rxui::TextWidth::MinContent => text.font_size / 2.,
                rxui::TextWidth::MaxContent => intrinsic,
            };
            Ok([
                width,
                (intrinsic / width.max(1.)).ceil().max(1.) * text.font_size,
            ])
        }
    }
    #[test]
    #[ignore = "requires the verified field-2027 LFS package"]
    fn native_view_prepares_all_controls_without_a_display() {
        let model = Inspector::load(
            &rm_simulator_assets::cad_assets::default_cad_assets(),
            "rune",
            0,
        )
        .unwrap();
        let options = Options::default();
        let image = Image::from_rgba8(
            raster::WIDTH,
            raster::HEIGHT,
            raster::render(&model, &options),
        )
        .unwrap();
        let mut runtime = rxui::Runtime::new();
        let entity = runtime.update(|cx| {
            cx.new(|_| App {
                model,
                options,
                image,
                error: None,
                timer: None,
                loading: None,
            })
        });
        let mut ui = rxui::Ui::new(&mut runtime, entity.clone()).unwrap();
        ui.prepare(&mut runtime, [1120., 1000.], &mut Measure)
            .unwrap();
        assert!(ui.elements().any(|e| e.key == Some(&"step".into())));
        let bounds = ui
            .elements()
            .find(|e| e.key == Some(&"step".into()))
            .unwrap()
            .bounds;
        let point = [bounds.x + bounds.width / 2., bounds.y + bounds.height / 2.];
        ui.pointer(&mut runtime, rxui::PointerEvent::Pressed(point))
            .unwrap();
        ui.pointer(&mut runtime, rxui::PointerEvent::Released(point))
            .unwrap();
        assert_eq!(
            runtime.update(|cx| entity.read(cx).model.timeline.time_ns),
            crate::model::TICK_NS
        );
        ui.prepare(&mut runtime, [1120., 1000.], &mut Measure)
            .unwrap();

        assert!(ui.elements().any(|e| e.key == Some(&"coordinate-1".into())));
    }
}
