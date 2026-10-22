// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Small orthographic wire projection, shared by the native image and PNG probe.
use crate::model::Inspector;
use rm_simulator_assets::math;

pub const WIDTH: u32 = 960;
pub const HEIGHT: u32 = 640;
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub enum Plane {
    #[default]
    Top,
    Front,
    Side,
}
impl Plane {
    pub fn project(self, p: [f64; 3]) -> [f64; 2] {
        match self {
            Self::Top => [p[0], p[1]],
            Self::Front => [p[0], p[2]],
            Self::Side => [p[1], p[2]],
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Top => "Top: +x right / +y up",
            Self::Front => "Front: +x right / +z up",
            Self::Side => "Side: +y right / +z up",
        }
    }
}
#[derive(Clone)]
pub struct Options {
    pub plane: Plane,
    pub fixed: bool,
    pub moving: bool,
    pub detectors: bool,
    pub rfid: bool,
    pub field: bool,
    pub zoom: f64,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            plane: Plane::Top,
            fixed: true,
            moving: true,
            detectors: true,
            rfid: true,
            field: false,
            zoom: 1.,
        }
    }
}
struct Canvas {
    pixels: Vec<u8>,
    center: [f64; 2],
    scale: f64,
    plane: Plane,
}
impl Canvas {
    fn new(min: [f64; 2], max: [f64; 2], options: &Options) -> Self {
        let scale = ((WIDTH as f64 - 80.) / (max[0] - min[0]).max(0.2))
            .min((HEIGHT as f64 - 80.) / (max[1] - min[1]).max(0.2))
            * options.zoom;
        Self {
            pixels: [13, 20, 30, 255].repeat((WIDTH * HEIGHT) as usize),
            center: [(min[0] + max[0]) / 2., (min[1] + max[1]) / 2.],
            scale,
            plane: options.plane,
        }
    }
    fn pixel(&self, p: [f64; 3]) -> [f64; 2] {
        let p = self.plane.project(p);
        [
            WIDTH as f64 / 2. + (p[0] - self.center[0]) * self.scale,
            HEIGHT as f64 / 2. - (p[1] - self.center[1]) * self.scale,
        ]
    }
    fn line(&mut self, a: [f64; 3], b: [f64; 3], color: [u8; 4]) {
        self.pixel_line(self.pixel(a), self.pixel(b), color);
    }
    fn pixel_line(&mut self, a: [f64; 2], b: [f64; 2], color: [u8; 4]) {
        let Some((a, b)) = clip(a, b) else {
            return;
        };
        let steps = (b[0] - a[0]).abs().max((b[1] - a[1]).abs()).ceil() as usize;
        for i in 0..=steps {
            let t = i as f64 / steps.max(1) as f64;
            let x = (a[0] + t * (b[0] - a[0]))
                .round()
                .clamp(0., (WIDTH - 1) as f64) as usize;
            let y = (a[1] + t * (b[1] - a[1]))
                .round()
                .clamp(0., (HEIGHT - 1) as f64) as usize;
            self.pixels[(y * WIDTH as usize + x) * 4..(y * WIDTH as usize + x) * 4 + 4]
                .copy_from_slice(&color);
        }
    }
}
// Clip before walking pixels, including distant field RFID in a close component view.
fn clip(a: [f64; 2], b: [f64; 2]) -> Option<([f64; 2], [f64; 2])> {
    if !a.into_iter().chain(b).all(f64::is_finite) {
        return None;
    }
    let d = [b[0] - a[0], b[1] - a[1]];
    let mut lo: f64 = 0.;
    let mut hi: f64 = 1.;
    for (p, q) in [
        (-d[0], a[0]),
        (d[0], WIDTH as f64 - 1. - a[0]),
        (-d[1], a[1]),
        (d[1], HEIGHT as f64 - 1. - a[1]),
    ] {
        if p == 0. {
            if q < 0. {
                return None;
            }
        } else {
            let t = q / p;
            if p < 0. {
                lo = lo.max(t);
            } else {
                hi = hi.min(t);
            }
            if lo > hi {
                return None;
            }
        }
    }
    Some((
        std::array::from_fn(|i| a[i] + lo * d[i]),
        std::array::from_fn(|i| a[i] + hi * d[i]),
    ))
}

pub fn render(model: &Inspector, options: &Options) -> Vec<u8> {
    let mut points = model.placed.fixed.vertices_m.clone();
    for (i, part) in model.placed.moving.iter().enumerate() {
        points.extend(
            part.mesh
                .vertices_m
                .iter()
                .map(|p| model.moved_point(i, *p)),
        );
    }
    if let Some(robot) = model.robot_truth() {
        points.extend(box_corners(robot.pose, robot.config.body_half_m));
        points.push(robot.turret.translation_m);
    }
    points.extend(model.projectiles().iter().map(|p| p.position_m));
    points.extend(model.contacts().iter().map(|p| p.position_m));
    if options.field {
        for zone in model.zones() {
            points.extend(zone.polygon_m.iter().map(|p| [p[0], p[1], zone.floor_m]));
        }
    }
    let mut min = [f64::INFINITY; 2];
    let mut max = [f64::NEG_INFINITY; 2];
    for p in points {
        let p = options.plane.project(p);
        for i in 0..2 {
            min[i] = min[i].min(p[i]);
            max[i] = max[i].max(p[i]);
        }
    }
    if !min.into_iter().chain(max).all(f64::is_finite) {
        min = [-1.; 2];
        max = [1.; 2];
    }
    let mut c = Canvas::new(min, max, options);
    // One metre grid; cap iteration for pathological packages.
    for axis in 0..2 {
        let start = min[axis].floor().max(-100.);
        let end = max[axis].ceil().min(100.);
        for v in start as i32..=end as i32 {
            let mut a = min;
            let mut b = max;
            a[axis] = v as f64;
            b[axis] = v as f64;
            let to3 = |p: [f64; 2]| match options.plane {
                Plane::Top => [p[0], p[1], 0.],
                Plane::Front => [p[0], 0., p[1]],
                Plane::Side => [0., p[0], p[1]],
            };
            c.line(to3(a), to3(b), [29, 41, 55, 255]);
        }
    }
    let mut mesh =
        |vertices: &[[f64; 3]], triangles: &[[u32; 3]], channel: Option<usize>, color| {
            for t in triangles {
                let p = t.map(|i| {
                    let p = vertices[i as usize];
                    channel.map(|i| model.moved_point(i, p)).unwrap_or(p)
                });
                for (a, b) in [(0, 1), (1, 2), (2, 0)] {
                    c.line(p[a], p[b], color);
                }
            }
        };
    if options.fixed {
        mesh(
            &model.placed.fixed.vertices_m,
            &model.placed.fixed.triangles,
            None,
            [74, 148, 135, 255],
        );
    }
    if options.moving {
        for (i, p) in model.placed.moving.iter().enumerate() {
            mesh(
                &p.mesh.vertices_m,
                &p.mesh.triangles,
                Some(i),
                [231, 169, 81, 255],
            );
        }
    }
    if options.rfid {
        for z in model.zones() {
            for (a, b) in z.polygon_m.iter().zip(z.polygon_m.iter().cycle().skip(1)) {
                for height in [z.floor_m, z.floor_m + z.height_m] {
                    c.line(
                        [a[0], a[1], height],
                        [b[0], b[1], height],
                        [174, 120, 243, 255],
                    );
                }
                c.line(
                    [a[0], a[1], z.floor_m],
                    [a[0], a[1], z.floor_m + z.height_m],
                    [174, 120, 243, 255],
                );
            }
        }
    }
    if options.detectors {
        for pose in model.detectors() {
            let p = pose.translation_m;
            for (axis, color) in [
                ([0.20, 0., 0.], [255, 100, 108, 255]),
                ([0., 0.15, 0.], [119, 239, 160, 255]),
                ([0., 0., 0.15], [111, 177, 255, 255]),
            ] {
                c.line(
                    p,
                    math::add(p, math::rotate(pose.rotation_wxyz, axis)),
                    color,
                );
            }
            let pixel = c.pixel(p);
            c.pixel_line(
                [pixel[0] - 3., pixel[1]],
                [pixel[0] + 3., pixel[1]],
                [255; 4],
            );
            c.pixel_line(
                [pixel[0], pixel[1] - 3.],
                [pixel[0], pixel[1] + 3.],
                [255; 4],
            );
        }
    }
    // Cyan bounds use actual physical poses/configuration (not generated CAD).
    if let Some(robot) = model.robot_truth() {
        for (pose, half) in [
            (robot.pose, robot.config.body_half_m),
            (robot.turret, robot.config.turret_half_m),
        ] {
            let corners = box_corners(pose, half);
            for i in 0..8 {
                for axis in [1, 2, 4] {
                    if i & axis == 0 {
                        c.line(corners[i], corners[i | axis], [80, 225, 244, 255]);
                    }
                }
            }
        }
        for wheel in robot.wheels {
            cross(&mut c, wheel.hub_m, [80, 225, 244, 255], 3.);
            if let Some(contact) = wheel.contact {
                cross(&mut c, contact.point_m, [119, 239, 160, 255], 2.);
            }
        }
        c.line(
            robot.turret.translation_m,
            math::add(
                robot.turret.translation_m,
                math::rotate(robot.turret.rotation_wxyz, [1., 0., 0.]),
            ),
            [80, 225, 244, 255],
        );
    }
    for projectile in model.projectiles() {
        cross(&mut c, projectile.position_m, [255, 230, 90, 255], 4.);
    }
    for contact in model.contacts() {
        cross(&mut c, contact.position_m, [255, 80, 180, 255], 6.);
    }
    // Always visible orientation glyph, same world axes as detector frames.
    let origin = [38., HEIGHT as f64 - 38.];
    c.pixel_line(origin, [origin[0] + 26., origin[1]], [255, 100, 108, 255]);
    c.pixel_line(origin, [origin[0], origin[1] - 26.], [119, 239, 160, 255]);
    c.pixels
}

fn box_corners(pose: rm_simulator_physics::Pose, half: [f64; 3]) -> [[f64; 3]; 8] {
    std::array::from_fn(|i| {
        math::add(
            pose.translation_m,
            math::rotate(
                pose.rotation_wxyz,
                std::array::from_fn(|axis| {
                    if i & (1 << axis) == 0 {
                        -half[axis]
                    } else {
                        half[axis]
                    }
                }),
            ),
        )
    })
}
fn cross(c: &mut Canvas, p: [f64; 3], color: [u8; 4], radius: f64) {
    let p = c.pixel(p);
    c.pixel_line([p[0] - radius, p[1]], [p[0] + radius, p[1]], color);
    c.pixel_line([p[0], p[1] - radius], [p[0], p[1] + radius], color);
}

pub fn write_png(path: &std::path::Path, pixels: &[u8]) -> anyhow::Result<()> {
    let mut encoder = png::Encoder::new(
        std::io::BufWriter::new(std::fs::File::create(path)?),
        WIDTH,
        HEIGHT,
    );
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(pixels)?;
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn clipping_bounds_work_and_reject_nonfinite_or_external_lines() {
        assert!(clip([-10., -10.], [-1., -1.]).is_none());
        assert!(clip([f64::NAN, 0.], [1., 1.]).is_none());
        let (a, b) = clip([-100000., 20.], [100000., 20.]).unwrap();
        assert!(a[0].abs() < 1e-8);
        assert!((b[0] - (WIDTH - 1) as f64).abs() < 1e-8);
    }
    #[test]
    fn projection_preserves_flu_axes() {
        assert_eq!(Plane::Top.project([1., 2., 3.]), [1., 2.]);
        assert_eq!(Plane::Front.project([1., 2., 3.]), [1., 3.]);
        assert_eq!(Plane::Side.project([1., 2., 3.]), [2., 3.]);
    }
}
