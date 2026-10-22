// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Optional CPU diagnostic viewport. Opaque, two-sided, flat factor lighting;
//! textures, transparency, metallic/roughness and PBR are intentionally unsupported.
use crate::{
    model::Inspector,
    raster::{self, HEIGHT, Options, WIDTH},
};
use rm_simulator_assets::collision_mesh::VisualPart;
use serde_json::{Value, json};

/// One cached world-placed visual primitive and its native motion channel.
pub struct Visual {
    /// Source identity/material factors with triangles placed once in world FLU.
    pub part: VisualPart,
    /// Index in the inspector's native moving channels, or fixed geometry.
    pub channel: Option<usize>,
}
/// Orthographic orbit camera, world FLU metres. Smaller depth is nearer.
#[derive(Clone, Copy, Debug)]
pub struct Camera {
    center: [f64; 3],
    right: [f64; 3],
    up: [f64; 3],
    toward: [f64; 3],
    scale: f64,
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter().zip(b).map(|(a, b)| a * b).sum()
}
fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
impl Camera {
    /// Fit projected extents, preserving the supplied yaw/elevation and zoom.
    pub fn fit(points: &[[f64; 3]], options: &Options) -> Self {
        let (sy, cy) = options.yaw_rad.sin_cos();
        let (se, ce) = options.elevation_rad.sin_cos();
        let toward = [ce * cy, ce * sy, se];
        let right = [-sy, cy, 0.];
        let up = cross(toward, right);
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for &p in points {
            for (i, axis) in [right, up, toward].into_iter().enumerate() {
                let v = dot(p, axis);
                min[i] = min[i].min(v);
                max[i] = max[i].max(v);
            }
        }
        if !min.into_iter().chain(max).all(f64::is_finite) {
            min = [-1.; 3];
            max = [1.; 3];
        }
        let mid: [f64; 3] = std::array::from_fn(|i| (min[i] + max[i]) * 0.5);
        let center =
            std::array::from_fn(|i| right[i] * mid[0] + up[i] * mid[1] + toward[i] * mid[2]);
        let scale = ((WIDTH as f64 - 80.) / (max[0] - min[0]).max(0.2))
            .min((HEIGHT as f64 - 80.) / (max[1] - min[1]).max(0.2))
            * options.zoom;
        Self {
            center,
            right,
            up,
            toward,
            scale,
        }
    }
    /// Pixel x/y and signed linear depth for a world point.
    pub fn project(&self, point: [f64; 3]) -> [f64; 3] {
        let p = sub(point, self.center);
        [
            WIDTH as f64 * 0.5 + dot(p, self.right) * self.scale,
            HEIGHT as f64 * 0.5 - dot(p, self.up) * self.scale,
            -dot(p, self.toward),
        ]
    }
    /// World point on the orthographic pixel ray at a signed linear depth.
    pub fn unproject(&self, pixel: [f64; 2], depth: f64) -> [f64; 3] {
        let x = (pixel[0] - WIDTH as f64 * 0.5) / self.scale;
        let y = -(pixel[1] - HEIGHT as f64 * 0.5) / self.scale;
        std::array::from_fn(|i| {
            self.center[i] + x * self.right[i] + y * self.up[i] - depth * self.toward[i]
        })
    }
}
/// Completed surface pass. Source ownership/depth excludes x-ray overlays.
pub struct Frame {
    /// RGBA8 sRGB pixels, including the requested x-ray diagnostics.
    pub pixels: Vec<u8>,
    /// Nearest source depth in metres per pixel; infinity means background.
    pub depth: Vec<f64>,
    /// Cached visual-part index per source pixel; usize::MAX means background.
    pub owners: Vec<usize>,
    /// Camera shared by surfaces, x-ray diagnostics and picking.
    pub camera: Camera,
    /// Number of submitted source visual triangles, including clipped triangles.
    pub triangles: usize,
    /// CPU viewport time in milliseconds, excluding source loading and presentation.
    pub elapsed_ms: f64,
}
impl Frame {
    fn empty(camera: Camera) -> Self {
        Self {
            pixels: [13, 20, 30, 255].repeat((WIDTH * HEIGHT) as usize),
            depth: vec![f64::INFINITY; (WIDTH * HEIGHT) as usize],
            owners: vec![usize::MAX; (WIDTH * HEIGHT) as usize],
            camera,
            triangles: 0,
            elapsed_ms: 0.,
        }
    }
    /// Pick the nearest source surface at the pixel centre, independent of overlays.
    pub fn pick(&self, model: &Inspector, x: u32, y: u32) -> Option<Value> {
        if x >= WIDTH || y >= HEIGHT {
            return None;
        }
        let index = (y * WIDTH + x) as usize;
        let visual = model.visuals().ok()?.get(self.owners[index])?;
        Some(
            json!({"node":visual.part.node,"primitive":visual.part.primitive,"name":visual.part.name,"material":visual.part.material,"base_color_linear":visual.part.base_color_linear,"emissive_linear":visual.part.emissive_linear,"unsupported_texture":visual.part.textured,"motion_channel":visual.channel,"world_m":self.camera.unproject([x as f64+0.5,y as f64+0.5],self.depth[index]),"depth_m":self.depth[index],"selection":"nearest opaque source triangle at pixel centre; diagnostic overlays excluded"}),
        )
    }
    fn triangle(&mut self, points: [[f64; 3]; 3], color: [u8; 4], owner: usize) {
        self.triangles += 1;
        let p = points.map(|p| self.camera.project(p));
        if !p.iter().flatten().all(|v| v.is_finite()) {
            return;
        }
        let edge = |a: [f64; 3], b: [f64; 3], x: f64, y: f64| {
            (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0])
        };
        let area = edge(p[0], p[1], p[2][0], p[2][1]);
        if area.abs() < 1e-10 {
            return;
        }
        let minx = p
            .iter()
            .map(|p| p[0])
            .fold(f64::INFINITY, f64::min)
            .floor()
            .max(0.) as u32;
        let maxx = p
            .iter()
            .map(|p| p[0])
            .fold(f64::NEG_INFINITY, f64::max)
            .ceil()
            .min(WIDTH as f64) as u32;
        let miny = p
            .iter()
            .map(|p| p[1])
            .fold(f64::INFINITY, f64::min)
            .floor()
            .max(0.) as u32;
        let maxy = p
            .iter()
            .map(|p| p[1])
            .fold(f64::NEG_INFINITY, f64::max)
            .ceil()
            .min(HEIGHT as f64) as u32;
        for y in miny..maxy {
            for x in minx..maxx {
                let weights = [
                    edge(p[1], p[2], x as f64 + 0.5, y as f64 + 0.5) / area,
                    edge(p[2], p[0], x as f64 + 0.5, y as f64 + 0.5) / area,
                    edge(p[0], p[1], x as f64 + 0.5, y as f64 + 0.5) / area,
                ];
                if weights.iter().any(|w| *w < -1e-10) {
                    continue;
                }
                let depth = weights.iter().zip(p).map(|(w, p)| w * p[2]).sum();
                let i = (y * WIDTH + x) as usize;
                if depth < self.depth[i] {
                    self.depth[i] = depth;
                    self.owners[i] = owner;
                    self.pixels[i * 4..i * 4 + 4].copy_from_slice(&color);
                }
            }
        }
    }
}
fn shade(part: &VisualPart, triangle: [[f64; 3]; 3]) -> [u8; 4] {
    let n = cross(sub(triangle[1], triangle[0]), sub(triangle[2], triangle[0]));
    let length = dot(n, n).sqrt();
    let light = 0.35
        + 0.65
            * (dot(n, [0.3, 0.4, 0.8660254]) / length.max(1e-15))
                .abs()
                .min(1.);
    let mut color = [255; 4];
    for (i, c) in color.iter_mut().enumerate().take(3) {
        let linear = (part.base_color_linear[i] * light + part.emissive_linear[i]).clamp(0., 1.);
        let srgb = if linear <= 0.0031308 {
            12.92 * linear
        } else {
            1.055 * linear.powf(1. / 2.4) - 0.055
        };
        *c = (srgb * 255.).round() as u8;
    }
    color
}
/// Render only the viewport; timings exclude GLB parsing, physics and PNG encoding.
/// Source alpha is treated as opaque; diagnostic lines are deliberately x-ray.
pub fn render(model: &Inspector, options: &Options) -> anyhow::Result<Frame> {
    let visuals = model.visuals()?;
    let started = std::time::Instant::now();
    let vertices: Vec<Vec<[f64; 3]>> = visuals
        .iter()
        .map(|v| {
            v.part
                .mesh
                .vertices_m
                .iter()
                .map(|&p| v.channel.map(|i| model.moved_point(i, p)).unwrap_or(p))
                .collect()
        })
        .collect();
    let mut fit: Vec<_> = vertices.iter().flatten().copied().collect();
    if let Some(robot) = model.robot_truth() {
        fit.extend(crate::raster::box_corners(
            robot.pose,
            robot.config.body_half_m,
        ));
    }
    fit.extend(model.projectiles().iter().map(|p| p.position_m));
    fit.extend(model.contacts().iter().map(|p| p.position_m));
    if options.field {
        for z in model.zones() {
            for p in &z.polygon_m {
                fit.push([p[0], p[1], z.floor_m]);
                fit.push([p[0], p[1], z.floor_m + z.height_m]);
            }
        }
    }
    let mut frame = Frame::empty(Camera::fit(&fit, options));
    for (owner, v) in visuals.iter().enumerate() {
        for t in &v.part.mesh.triangles {
            let p = t.map(|i| vertices[owner][i as usize]);
            frame.triangle(p, shade(&v.part, p), owner);
        }
    }
    frame.pixels = raster::orbit_overlays(model, options, frame.camera, frame.pixels);
    frame.elapsed_ms = started.elapsed().as_secs_f64() * 1000.;
    Ok(frame)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rm_simulator_assets::collision_mesh::CollisionMesh;
    fn camera() -> Camera {
        Camera::fit(
            &[[-1.; 3], [1.; 3]],
            &Options {
                yaw_rad: 0.,
                elevation_rad: 0.,
                ..Options::default()
            },
        )
    }
    #[test]
    fn orbit_projection_round_trips_zoom_and_flu_axes() {
        let camera = camera();
        let p = [0.3, 0.4, 0.6];
        let q = camera.project(p);
        assert!(q[0] > WIDTH as f64 / 2. && q[1] < HEIGHT as f64 / 2.);
        assert!(q[2] < 0.);
        assert!(
            sub(camera.unproject([q[0], q[1]], q[2]), p)
                .iter()
                .all(|v| v.abs() < 1e-12)
        );
        let zoom = Camera::fit(
            &[[-1.; 3], [1.; 3]],
            &Options {
                yaw_rad: 0.,
                elevation_rad: 0.,
                zoom: 2.,
                ..Options::default()
            },
        );
        assert!((zoom.project(p)[0] - 480. - 2. * (q[0] - 480.)).abs() < 1e-10);
    }
    #[test]
    fn depth_order_ownership_and_ray_point_are_draw_order_independent() {
        let far = [[-0.5, -0.8, -0.8], [-0.5, 0.8, -0.8], [-0.5, 0., 0.8]];
        let near = far.map(|p| [0.5, p[1], p[2]]);
        let mut a = Frame::empty(camera());
        let mut b = Frame::empty(camera());
        a.triangle(far, [0, 0, 255, 255], 1);
        a.triangle(near, [255, 0, 0, 255], 2);
        b.triangle(near, [255, 0, 0, 255], 2);
        b.triangle(far, [0, 0, 255, 255], 1);
        assert_eq!(a.pixels, b.pixels);
        assert_eq!(a.owners, b.owners);
        let i = (320 * WIDTH + 480) as usize;
        assert_eq!(a.owners[i], 2);
        let world = a.camera.unproject([480.5, 320.5], a.depth[i]);
        assert!((world[0] - 0.5).abs() < 1e-10);
        assert_eq!(a.owners[0], usize::MAX);
        // Degenerate and fully clipped triangles cannot replace a visible surface.
        a.triangle([[0.; 3]; 3], [0; 4], 3);
        a.triangle(
            [[0., 100., 100.], [0., 101., 100.], [0., 100., 101.]],
            [0; 4],
            3,
        );
        assert_eq!(a.owners, b.owners);
    }
    #[test]
    fn unpainted_default_white_and_emission_use_linear_factors() {
        let mut part = VisualPart {
            node: 0,
            primitive: 0,
            name: String::new(),
            material: String::new(),
            base_color_linear: [1.; 4],
            emissive_linear: [0.; 3],
            textured: false,
            mesh: CollisionMesh::default(),
        };
        let triangle = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]];
        let white = shade(&part, triangle);
        assert_eq!(white[0], white[1]);
        assert_eq!(white[1], white[2]);
        assert!(white[0] > 200);
        part.base_color_linear = [0., 0., 0., 1.];
        part.emissive_linear = [1., 0., 0.];
        assert_eq!(shade(&part, triangle), [255, 0, 0, 255]);
    }
    // Independent ray/triangle intersection verifies the surface ownership buffer.
    fn ray_hit(origin: [f64; 3], dir: [f64; 3], p: [[f64; 3]; 3]) -> Option<f64> {
        let e1 = sub(p[1], p[0]);
        let e2 = sub(p[2], p[0]);
        let h = cross(dir, e2);
        let det = dot(e1, h);
        if det.abs() < 1e-12 {
            return None;
        }
        let s = sub(origin, p[0]);
        let u = dot(s, h) / det;
        if !(0. ..=1.).contains(&u) {
            return None;
        }
        let q = cross(s, e1);
        let v = dot(dir, q) / det;
        if v < 0. || u + v > 1. {
            return None;
        }
        let t = dot(e2, q) / det;
        (t >= 0.).then_some(t)
    }
    #[test]
    #[ignore = "requires the verified field-2027 LFS package"]
    fn real_source_previews_pick_nearest_ray_and_follow_native_motion() {
        let root = std::env::var_os("RM_INSPECTOR_TEST_ASSETS")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(rm_simulator_assets::cad_assets::default_cad_assets);
        for name in ["outpost", "base", "rune"] {
            let mut model = Inspector::load(&root, name, 0).unwrap();
            model.configure_robot(None).unwrap();
            let options = Options {
                orbit: true,
                detectors: false,
                rfid: false,
                ..Options::default()
            };
            let frame = render(&model, &options).unwrap();
            assert!(frame.triangles > 0);
            let i = frame
                .owners
                .iter()
                .enumerate()
                .filter(|(_, o)| **o != usize::MAX)
                .min_by_key(|(i, _)| {
                    let x = *i % WIDTH as usize;
                    let y = *i / WIDTH as usize;
                    x.abs_diff(480) + y.abs_diff(320)
                })
                .unwrap()
                .0;
            let pixel = [
                (i % WIDTH as usize) as f64 + 0.5,
                (i / WIDTH as usize) as f64 + 0.5,
            ];
            let origin = frame.camera.unproject(pixel, -10000.);
            let dir = frame.camera.toward.map(|v| -v);
            let mut nearest = (f64::INFINITY, usize::MAX);
            for (owner, v) in model.visuals().unwrap().iter().enumerate() {
                for tri in &v.part.mesh.triangles {
                    let p = tri.map(|i| {
                        let p = v.part.mesh.vertices_m[i as usize];
                        v.channel.map(|c| model.moved_point(c, p)).unwrap_or(p)
                    });
                    if let Some(t) = ray_hit(origin, dir, p)
                        && t < nearest.0
                    {
                        nearest = (t, owner);
                    }
                }
            }
            assert!((nearest.0 - 10000. - frame.depth[i]).abs() < 1e-7, "{name}");
            // Coplanar primitives can tie; compare depth rather than arbitrary tie IDs.
            let picked = frame
                .pick(
                    &model,
                    (i % WIDTH as usize) as u32,
                    (i / WIDTH as usize) as u32,
                )
                .unwrap();
            assert_eq!(
                picked["node"],
                model.visuals().unwrap()[frame.owners[i]].part.node
            );
            let ptr = model.visuals().unwrap().as_ptr();
            assert_eq!(ptr, model.visuals().unwrap().as_ptr());
            let v = model
                .visuals()
                .unwrap()
                .iter()
                .find(|v| v.channel.is_some())
                .expect("native moving source visual");
            let channel = v.channel.unwrap();
            assert!(channel < model.placed.moving.len());
            let point = v
                .part
                .mesh
                .vertices_m
                .iter()
                .copied()
                .max_by(|a, b| dot(*a, *a).total_cmp(&dot(*b, *b)))
                .unwrap();
            let before = model.moved_point(channel, point);
            model.set_coordinate(channel, 0.7).unwrap();
            assert!(
                dot(
                    sub(before, model.moved_point(channel, point)),
                    sub(before, model.moved_point(channel, point))
                ) > 1e-9
            );
            model.step(16).unwrap();
            let moved = render(&model, &options).unwrap();
            assert_ne!(frame.pixels, moved.pixels);
            assert_eq!(ptr, model.visuals().unwrap().as_ptr());
            model.reset().unwrap();
            assert_eq!(frame.pixels, render(&model, &options).unwrap().pixels);
            let old_node_count = model.visuals().unwrap().len();
            assert!(model.select(usize::MAX, 0).is_err());
            assert_eq!(old_node_count, model.visuals().unwrap().len());
            if model.asset().placements.len() > 1 {
                let old = model.visuals().unwrap()[0].part.mesh.vertices_m[0];
                model.select(model.selected, 1).unwrap();
                let new = model.visuals().unwrap()[0].part.mesh.vertices_m[0];
                assert!(
                    dot(sub(old, new), sub(old, new)) > 1e-8,
                    "placement cache invalidation"
                );
            }
        }
    }
}
