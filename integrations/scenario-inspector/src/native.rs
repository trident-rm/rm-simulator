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
    frame: Option<rm_scenario_inspector::viewport::Frame>,
    picked: String,
    error: Option<String>,
    timer: Option<Task>,
    loading: Option<Task>,
}
impl App {
    fn refresh(&mut self) {
        self.picked.clear();
        let pixels = if self.options.orbit {
            match rm_scenario_inspector::viewport::render(&self.model, &self.options) {
                Ok(frame) => {
                    let pixels = frame.pixels.clone();
                    self.frame = Some(frame);
                    pixels
                }
                Err(e) => {
                    self.error = Some(format!("{e:#}"));
                    self.frame = None;
                    return;
                }
            }
        } else {
            self.frame = None;
            raster::render(&self.model, &self.options)
        };
        match Image::from_rgba8(raster::WIDTH, raster::HEIGHT, pixels) {
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
                rm_sim_physics::motion::MotionFrame::Rotation { .. }
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
                    s.options.orbit = false;
                    s.options.plane = plane;
                    s.refresh();
                }),
            ));
        }
        projections = projections.child(button("3D CPU orbit").key("orbit").on_click(cx.listener(
            |s, _, _| {
                s.options.orbit = true;
                s.refresh();
            },
        )));
        let mut orbit_controls = column().gap(6.);
        if self.options.orbit {
            orbit_controls=orbit_controls.child(label("CPU depth-rasterized orthographic diagnostic 3D • source color/emission factors • no textures/PBR/alpha • diagnostic overlays are x-ray. Click surface to pick."));
            for (axis, name, value, low, high) in [
                (
                    0,
                    "Orbit yaw (rad)",
                    self.options.yaw_rad,
                    -std::f64::consts::PI,
                    std::f64::consts::PI,
                ),
                (
                    1,
                    "Orbit elevation (rad)",
                    self.options.elevation_rad,
                    -1.55,
                    1.55,
                ),
            ] {
                orbit_controls = orbit_controls
                    .child(label(format!("{name}: {value:.3}")))
                    .child(
                        slider(value as f32)
                            .key(format!("orbit-{axis}"))
                            .range(low as f32, high as f32)
                            .width(380.)
                            .accessibility_label(name)
                            .on_change(cx.listener(move |s, e: &SliderEvent, _| {
                                if axis == 0 {
                                    s.options.yaw_rad = e.value as f64;
                                } else {
                                    s.options.elevation_rad = e.value as f64;
                                }
                                s.refresh();
                            })),
                    );
            }
            orbit_controls = orbit_controls.child(
                checkbox(self.options.collision_overlay, "X-ray collision edges")
                    .key("collision-overlay")
                    .on_change(cx.listener(|s, e: &ToggleEvent, _| {
                        s.options.collision_overlay = e.checked;
                        s.refresh();
                    })),
            );
            if let Some(frame) = &self.frame {
                orbit_controls=orbit_controls.child(label(format!("CPU viewport: {:.2} ms • {} triangles • excludes parsing, physics and GPU upload",frame.elapsed_ms,frame.triangles)));
            }
            orbit_controls = orbit_controls.child(label(self.picked.clone()));
        }
        let mut robot_controls = column().gap(6.).child(label(
            "Physical robot • privileged truth, not simulated sensors",
        ));
        if let Some(spawn) = self.model.robot_spawn() {
            robot_controls = robot_controls.child(label("Spawn edits restart the episode; flat isolated catch floor at z=0. Cyan boxes show profile bounds, not CAD."));
            let q = spawn.pose.rotation_wxyz;
            let yaw = 2. * q[3].atan2(q[0]);
            for (axis, name, value, low, high) in [
                (0, "Spawn x (m)", spawn.pose.translation_m[0], -30., 30.),
                (1, "Spawn y (m)", spawn.pose.translation_m[1], -30., 30.),
                (2, "Spawn z (m)", spawn.pose.translation_m[2], 0., 5.),
                (
                    3,
                    "Spawn yaw (rad)",
                    yaw,
                    -std::f64::consts::PI,
                    std::f64::consts::PI,
                ),
            ] {
                robot_controls = robot_controls
                    .child(label(format!("{name}: {value:.3}")))
                    .child(
                        slider(value as f32)
                            .key(format!("spawn-{axis}"))
                            .range(low as f32, high as f32)
                            .width(380.)
                            .accessibility_label(name)
                            .on_change(cx.listener(move |s, e: &SliderEvent, _| {
                                s.timer = None;
                                s.apply(|m| {
                                    let mut spawn = m
                                        .robot_spawn()
                                        .cloned()
                                        .ok_or_else(|| anyhow::anyhow!("no robot"))?;
                                    if axis < 3 {
                                        spawn.pose.translation_m[axis] = e.value as f64;
                                    } else {
                                        let half = e.value as f64 / 2.;
                                        spawn.pose.rotation_wxyz = [half.cos(), 0., 0., half.sin()];
                                    }
                                    m.configure_robot(Some(spawn))
                                });
                            })),
                    );
            }
            robot_controls = robot_controls.child(
                button(format!("Team {:?}: swap label", spawn.team))
                    .key("robot-team")
                    .on_click(cx.listener(|s, _, _| {
                        s.apply(|m| {
                            let mut spawn = m.robot_spawn().cloned().unwrap();
                            spawn.team = if spawn.team == rm_sim_physics::Team::Red {
                                rm_sim_physics::Team::Blue
                            } else {
                                rm_sim_physics::Team::Red
                            };
                            m.configure_robot(Some(spawn))
                        })
                    })),
            );
        }
        if let Some(robot) = self.model.robot_truth() {
            for (axis, name, value, limit) in [
                (0, "Forward (m/s)", robot.command.forward_m_s, 3.),
                (1, "Left (m/s)", robot.command.left_m_s, 3.),
                (2, "Yaw rate (rad/s)", robot.command.yaw_rate_rad_s, 3.),
                (
                    3,
                    "Aim world yaw (rad)",
                    robot.command.aim_yaw_rad,
                    std::f64::consts::PI,
                ),
                (4, "Aim elevation (rad)", robot.command.aim_pitch_rad, 1.2),
            ] {
                robot_controls = robot_controls
                    .child(label(format!("{name}: {value:.3}")))
                    .child(
                        slider(value as f32)
                            .key(format!("robot-control-{axis}"))
                            .range(-limit as f32, limit as f32)
                            .width(380.)
                            .accessibility_label(name)
                            .on_change(cx.listener(move |s, e: &SliderEvent, _| {
                                s.apply(|m| {
                                    let mut command = m.robot_truth().unwrap().command;
                                    match axis {
                                        0 => command.forward_m_s = e.value as f64,
                                        1 => command.left_m_s = e.value as f64,
                                        2 => command.yaw_rate_rad_s = e.value as f64,
                                        3 => return m.aim(e.value as f64, command.aim_pitch_rad),
                                        _ => return m.aim(command.aim_yaw_rad, e.value as f64),
                                    }
                                    m.drive(command)
                                })
                            })),
                    );
            }
            robot_controls=robot_controls.child(label("Direct torques replace body velocity control; aim preserves the drive mode. Zero torque free-rolls."));
            let torques = match robot.drive_control {
                rm_sim_physics::control::DriveControl::WheelTorques(t) => t.as_nm().to_vec(),
                _ => vec![0.; 4],
            };
            for (i, value) in torques.into_iter().enumerate() {
                robot_controls = robot_controls
                    .child(label(format!("Wheel {i} torque: {value:.3} N m")))
                    .child(
                        slider(value as f32)
                            .key(format!("torque-{i}"))
                            .range(-3., 3.)
                            .width(380.)
                            .accessibility_label(format!("Wheel {i} torque N m"))
                            .on_change(cx.listener(move |s, e: &SliderEvent, _| {
                                s.apply(|m| {
                                    let robot = m.robot_truth().unwrap();
                                    let mut torques = match robot.drive_control {
                                        rm_sim_physics::control::DriveControl::WheelTorques(t) => {
                                            t.as_nm().to_vec()
                                        }
                                        _ => vec![0.; 4],
                                    };
                                    torques[i] = e.value as f64;
                                    m.wheel_torques(torques)
                                })
                            })),
                    );
            }
            robot_controls =
                robot_controls.child(
                    row()
                        .gap(8.)
                        .child(button("Brake / velocity zero").key("robot-stop").on_click(
                            cx.listener(|s, _, _| {
                                s.apply(|m| {
                                    let command = m.robot_truth().unwrap().command;
                                    m.drive(rm_sim_physics::chassis::ChassisCommand {
                                        forward_m_s: 0.,
                                        left_m_s: 0.,
                                        yaw_rate_rad_s: 0.,
                                        ..command
                                    })
                                })
                            }),
                        ))
                        .child(
                            button("Fire 17 mm / 25 m/s")
                                .key("fire-17")
                                .on_click(cx.listener(|s, _, _| {
                                    s.apply(|m| m.fire(rm_sim_physics::Caliber::Mm17, 25.))
                                })),
                        )
                        .child(
                            button("Fire 42 mm / 12 m/s")
                                .key("fire-42")
                                .on_click(cx.listener(|s, _, _| {
                                    s.apply(|m| m.fire(rm_sim_physics::Caliber::Mm42, 12.))
                                })),
                        ),
                );
            robot_controls=robot_controls.child(label(format!("Truth pose {:?} m • velocity {:?} m/s • actual aim {:?} rad • {} projectiles • {} retained contacts",
                robot.pose.translation_m,robot.velocity_m_s,robot.held_aim_rad,self.model.projectiles().len(),self.model.contacts().len())));
        } else {
            robot_controls = robot_controls.child(label(
                "This selection has no robot. Outpost/Base/Rune support physical commands.",
            ));
        }
        for contact in self.model.contacts().iter().rev().take(8) {
            robot_controls = robot_controls.child(label(format!(
                "{} ns: {} {} projectile {:?} at {:?} m; local {:?} m; normal {:.3} m/s",
                contact.time_ns,
                contact.kind,
                contact.target,
                contact.projectile,
                contact.position_m,
                contact.local_m,
                contact.normal_speed_m_s
            )));
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
            .child(projections).child(orbit_controls)
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
                .on_pointer_down(cx.listener(|s,e: &PointerInput,_| {
                    if let Some(frame)=&s.frame {
                        let scale=(e.bounds.width/raster::WIDTH as f32).min(e.bounds.height/raster::HEIGHT as f32);
                        let x=(e.local_position[0]-(e.bounds.width-raster::WIDTH as f32*scale)*0.5)/scale;
                        let y=(e.local_position[1]-(e.bounds.height-raster::HEIGHT as f32*scale)*0.5)/scale;
                        s.picked=if x>=0. && y>=0. {frame.pick(&s.model,x as u32,y as u32).map(|p|p.to_string()).unwrap_or_else(||"No source surface at pixel".into())} else {"Outside image".into()};
                    }
                }))
                .accessibility_label("Orthographic diagnostic projection; fixed teal, moving amber, RFID purple, robot cyan, projectile yellow, contact pink; detector axes red green blue"))
            .child(label(format!("{} • teal fixed {} triangles • amber moving {} triangles / {} channels • {} detector frames • {} RFID footprints",
                if self.options.orbit {"CPU orthographic orbit 3D"} else {self.options.plane.label()},self.model.placed.fixed.triangles.len(),self.model.placed.moving.iter().map(|p|p.mesh.triangles.len()).sum::<usize>(),self.model.placed.moving.len(),self.model.detectors().len(),self.model.zones().len())))
            .child(label(if self.model.zones().is_empty() {"RFID unavailable in this source package: import caller geometry with --rfid ZONES_JSON."} else {"Purple: RFID footprint and vertical band; caller overlays are unverified inspection geometry."}))
            .child(robot_controls)
            .child(channels)
            .child(label("Sliders restart at t=0 with the chosen channel offset and pause. Reset clears time and all offsets. Fit follows each pose. 3D uses orthographic depth occlusion; wire planes show all edges."))
            .child(label(format!("Visual source: {}\nCollider source: {}\nPlacement root (basis included): {:?}",self.model.cad.root.join(&self.model.asset().file).display(),self.model.asset().physics_file(&self.model.cad.root).display(),self.model.root())))
            .child(label(format!("Source STEP SHA-256: {}",self.model.cad.rmuc_2027.as_ref().map(|b|b.source_sha256.as_str()).unwrap_or("no native binding"))))
            .child(label(if self.loading.is_some() {"Loading verified geometry…".into()} else {self.error.clone().unwrap_or_default()}))
            .child(label("Wire planes or CPU diagnostic 3D with source color/emission factors. Textures, alpha and PBR are unsupported. Source presets use RobotSession with the existing omni profile. Cyan robot, yellow projectile, pink raw armor contact. No RF simulation or match authority.").color(ThemeColor::TextMuted))
    }
}
pub fn run(model: Inspector, options: Options) -> anyhow::Result<()> {
    let frame = if options.orbit {
        Some(rm_scenario_inspector::viewport::render(&model, &options)?)
    } else {
        None
    };
    let pixels = frame
        .as_ref()
        .map(|f| f.pixels.clone())
        .unwrap_or_else(|| raster::render(&model, &options));
    let image = Image::from_rgba8(raster::WIDTH, raster::HEIGHT, pixels)
        .map_err(|e| anyhow::anyhow!(e.to_string()))?;
    Application::new()
        .run(move |cx| {
            let state = cx.new(|_| App {
                model,
                options,
                image,
                frame,
                picked: String::new(),
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
        let model =
            Inspector::load(&rm_sim_assets::cad_assets::default_cad_assets(), "rune", 0).unwrap();
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
                frame: None,
                picked: String::new(),
                error: None,
                timer: None,
                loading: None,
            })
        });
        let mut ui = rxui::Ui::new(&mut runtime, entity.clone()).unwrap();
        ui.prepare(&mut runtime, [1120., 1000.], &mut Measure)
            .unwrap();
        for key in [
            "step",
            "spawn-0",
            "robot-control-3",
            "torque-0",
            "fire-17",
            "fire-42",
            "robot-stop",
        ] {
            assert!(
                ui.elements().any(|e| e.key == Some(&key.into())),
                "missing {key}"
            );
        }
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
        // Exercise the actual image pointer route in 3D, without a window server.
        runtime.update(|cx| {
            entity.update(cx, |app, _| {
                app.options.orbit = true;
                app.refresh();
            })
        });
        ui.prepare(&mut runtime, [1120., 5000.], &mut Measure)
            .unwrap();
        for key in ["orbit-0", "orbit-1", "collision-overlay"] {
            assert!(ui.elements().any(|e| e.key == Some(&key.into())));
        }
        let bounds = ui
            .elements()
            .find(|e| e.key == Some(&"projection".into()))
            .unwrap()
            .bounds;
        let pixel = runtime.update(|cx| {
            let app = entity.read(cx);
            let frame = app.frame.as_ref().unwrap();
            let i = frame.owners.iter().position(|&o| o != usize::MAX).unwrap();
            [
                (i % raster::WIDTH as usize) as f32 + 0.5,
                (i / raster::WIDTH as usize) as f32 + 0.5,
            ]
        });
        let scale =
            (bounds.width / raster::WIDTH as f32).min(bounds.height / raster::HEIGHT as f32);
        let point = [
            bounds.x + (bounds.width - raster::WIDTH as f32 * scale) * 0.5 + pixel[0] * scale,
            bounds.y + (bounds.height - raster::HEIGHT as f32 * scale) * 0.5 + pixel[1] * scale,
        ];
        ui.pointer(&mut runtime, rxui::PointerEvent::Pressed(point))
            .unwrap();
        ui.pointer(&mut runtime, rxui::PointerEvent::Released(point))
            .unwrap();
        assert!(runtime.update(|cx| entity.read(cx).picked.contains("primitive")));
    }
}
