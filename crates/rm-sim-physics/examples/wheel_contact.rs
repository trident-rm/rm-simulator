// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Guided single-module contact test: same native scene for suspension, tyre and terrain.
//! Provisional drivetrain/tyre assumptions; not a complete vehicle or game replacement.
#[allow(dead_code)]
#[path = "wheel_suspension.rs"]
mod suspension;
use glam::{DQuat, DVec3};
use rm_sim_engine::{
    Pose,
    model::{
        BodyKind, BodyState, ColliderDescription, ContactMaterial, ConvexHull, Inertial,
        JointDescription, JointKind, JointMotor, RigidBodyDescription, ShapeDescription,
        TriangleMesh,
    },
};
use rm_sim_physics::{
    StepDuration,
    cooking::{CookedRigidBody, CookedTriangleMesh},
    runtime::{BodyBinding, BodyKey, ColliderBinding, JointBinding, JointKey},
};

/// Explicit test parameters, not recovered manufacturer properties.
#[derive(Clone, Copy, Debug)]
pub struct Parameters {
    /// Sprung frame mass, kg. Represents one nominal quarter of a 20 kg robot.
    pub sprung_mass_kg: f64,
    /// Carrier/motor equivalent mass, kg; not native 5 kg bench load.
    pub carrier_mass_kg: f64,
    /// Tyre/wheel mass, kg, with explicit uniform-cylinder inertia.
    pub wheel_mass_kg: f64,
    /// Radius, m; rounded 150 mm source-derived outer envelope.
    pub radius_m: f64,
    /// Width, m, rounded from the native approximately 39 mm axial envelope.
    pub width_m: f64,
    /// Coulomb coefficient on both tyre and ground; zero is a traction control case.
    pub friction: f64,
    /// Optional ramp/plateau obstacle height, m; zero installs only the flat floor.
    pub bump_height_m: f64,
}
impl Default for Parameters {
    fn default() -> Self {
        Self {
            sprung_mass_kg: 5.,
            carrier_mass_kg: 0.45,
            wheel_mass_kg: 0.25,
            radius_m: 0.075,
            width_m: 0.040,
            friction: 0.9,
            bump_height_m: 0.005,
        }
    }
}
fn pose(p: DVec3, q: DQuat) -> Pose {
    Pose {
        translation_m: p.to_array(),
        rotation_wxyz: [q.w, q.x, q.y, q.z],
    }
}
fn rotation(p: Pose) -> DQuat {
    let [w, x, y, z] = p.rotation_wxyz;
    DQuat::from_xyzw(x, y, z, w)
}
fn frame(p: Pose, anchor: DVec3, q: DQuat) -> Pose {
    let r = rotation(p).conjugate();
    pose(r * (anchor - DVec3::from_array(p.translation_m)), r * q)
}
fn material(friction: f64, memberships: u32, filter: u32) -> ContactMaterial {
    ContactMaterial {
        friction,
        restitution: 0.,
        memberships,
        filter,
    }
}
/// Start of the 20 mm up-ramp, m. The 40 mm plateau ends at x=0.41 m.
pub const BUMP_START_M: f64 = 0.35;
/// End of the symmetric down-ramp, m.
pub const BUMP_END_M: f64 = 0.43;
/// Source wheel center relative to the carrier's original bench frame, m.
/// From the midpoint of the two native inner plates; independent of suspension
/// hinge locations. The 34.55 mm outboard offset is retained.
pub const HUB_CARRIER_M: [f64; 3] = [0.025, 0.03455, -0.02525];

/// Actual tick-end dynamics and privileged contact observations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// Shared suspension truth, including its six solved body frames.
    pub suspension: suspension::Sample,
    /// Driven tyre body pose.
    pub wheel: BodyState,
    /// Sprung frame linear velocity, world m/s.
    pub chassis_velocity_m_s: [f64; 3],
    /// Wheel tread speed from spin, m/s; forward +X is positive.
    pub rolling_speed_m_s: f64,
    /// Longitudinal wheel-COM speed minus angular tread speed, m/s.
    pub slip_m_s: f64,
    /// Sprung-frame vertical support estimate from COM momentum balance, N.
    /// This is m*(delta_v_z/dt + g), not a measured contact impulse or sensor.
    /// None in a pose-only sample without a preceding completed tick interval.
    pub chassis_support_n: Option<f64>,
    /// At least one wheel/floor contact sampled during this tick.
    pub floor_contact: bool,
    /// At least one wheel/obstacle contact sampled during this tick.
    pub bump_contact: bool,
    /// Deepest reported separation, m; negative means penetration.
    pub minimum_separation_m: f64,
    /// Contact samples lost to the explicitly bounded buffer.
    pub dropped_contacts: usize,
}
/// Dynamic guided frame, actual rolling body and source-derived linkage. All
/// interactions run inside the suspension fixture's single PhysicalRuntime.
pub struct Fixture {
    /// Reused linkage; this is also the only world owner and explicit clock.
    pub suspension: suspension::Fixture,
    /// Test choices retained for reconstruction.
    pub parameters: Parameters,
    duration: StepDuration,
    wheel: BodyBinding,
    axle: JointBinding,
    floor: ColliderBinding,
    bump: Option<ColliderBinding>,
    speed_m_s: f64,
}
impl Fixture {
    /// Build an unpowered module 2 mm above its flat support, with no prescribed
    /// kinematic endpoint. Frame guide locks lateral motion and rotation only.
    pub fn new(parameters: Parameters, duration: StepDuration) -> Result<Self, &'static str> {
        if ![
            parameters.sprung_mass_kg,
            parameters.carrier_mass_kg,
            parameters.wheel_mass_kg,
            parameters.radius_m,
            parameters.width_m,
            parameters.friction,
            parameters.bump_height_m,
        ]
        .into_iter()
        .all(f64::is_finite)
            || parameters.sprung_mass_kg <= 0.
            || parameters.carrier_mass_kg <= 0.
            || parameters.wheel_mass_kg <= 0.
            || !(0.01..=0.2).contains(&parameters.radius_m)
            || !(0.001..=0.1).contains(&parameters.width_m)
            || parameters.friction < 0.
            || !(0.0..=0.02).contains(&parameters.bump_height_m)
        {
            return Err("invalid wheel-contact parameters");
        }
        // Source-up remains unverified: choose a documented test orientation with
        // rising rockers and adequate bump travel. All recovered relative frames
        // rotate together; no link lengths or mounting offsets are changed.
        let q = DQuat::from_rotation_y(47_f64.to_radians())
            * DQuat::from_rotation_x(std::f64::consts::PI);
        let arm = DVec3::new(0.011754609365650348, 0., -0.022064205371165435);
        let root = DVec3::new(
            0.,
            0.,
            parameters.radius_m + 0.002 - (q * (arm + DVec3::from_array(HUB_CARRIER_M))).z,
        );
        let placement = pose(root - q * DVec3::new(0., 0., 0.3), q);
        let mut module = suspension::Fixture::guided_mount(
            suspension::Parameters {
                load_kg: parameters.carrier_mass_kg,
                ..Default::default()
            },
            duration,
            32,
            placement,
            parameters.sprung_mass_kg,
        )?;
        let runtime = &mut module.runtime;
        let carrier = runtime.body_binding(BodyKey(4)).ok_or("missing carrier")?;
        let carrier_state = runtime.body_state(carrier)?;
        let center = DVec3::from_array(carrier_state.pose.translation_m)
            + rotation(carrier_state.pose) * DVec3::from_array(HUB_CARRIER_M);
        let mut points = Vec::with_capacity(128);
        for i in 0..64 {
            let t = i as f64 * std::f64::consts::TAU / 64.;
            for side in [-1., 1.] {
                points.push([
                    parameters.radius_m * t.cos(),
                    side * parameters.width_m / 2.,
                    parameters.radius_m * t.sin(),
                ]);
            }
        }
        let r = parameters.radius_m;
        let w = parameters.width_m;
        let m = parameters.wheel_mass_kg;
        let transverse = m * (3. * r * r + w * w) / 12.;
        let axial = m * r * r / 2.;
        let source = RigidBodyDescription::new(
            BodyKind::Dynamic(Inertial::new(
                m,
                [transverse, axial, transverse],
                Pose::default(),
            )?),
            vec![ColliderDescription::new(
                ShapeDescription::ConvexHull(ConvexHull::new(points)?),
                Pose::default(),
                material(parameters.friction, 1, 2),
            )?],
        )?;
        let wheel_pose = Pose::at(center.to_array());
        let wheel = runtime.install_body(
            BodyKey(7),
            &CookedRigidBody::new(source)?,
            BodyState::at(wheel_pose),
        )?;
        let hq = DQuat::from_rotation_arc(DVec3::X, DVec3::Y);
        let axle = runtime.install_joint(
            JointKey(8),
            carrier,
            wheel,
            &JointDescription::new(
                JointKind::Revolute,
                frame(carrier_state.pose, center, hq),
                frame(wheel_pose, center, hq),
                None,
                false,
            )?,
        )?;
        let floor = runtime.install_catch_floor(0., material(parameters.friction, 2, 1))?;
        let bump = if parameters.bump_height_m > 0. {
            let mut v = Vec::new();
            for (x, z) in [
                (BUMP_START_M, 0.),
                (0.37, parameters.bump_height_m),
                (0.41, parameters.bump_height_m),
                (BUMP_END_M, 0.),
            ] {
                v.push([x, -0.15, z]);
                v.push([x, 0.15, z]);
            }
            let mut triangles = Vec::new();
            for i in 0..3_u32 {
                let k = 2 * i;
                triangles.push([k, k + 2, k + 1]);
                triangles.push([k + 1, k + 2, k + 3]);
            }
            Some(runtime.install_fixed_mesh(
                &CookedTriangleMesh::new(TriangleMesh::new(v, triangles)?)?,
                Pose::default(),
                material(parameters.friction, 2, 1),
            )?)
        } else {
            None
        };
        runtime.enable_contact_observations(64)?;
        let mut f = Self {
            suspension: module,
            parameters,
            duration,
            wheel,
            axle,
            floor,
            bump,
            speed_m_s: 0.,
        };
        f.drive(0.)?;
        Ok(f)
    }
    /// Hold a bounded axle velocity command. The frame is never driven directly.
    /// ±1 m/s request bound; derivative gain 0.5 N m s/rad and 1.2 N m cap are
    /// explicit fixture assumptions, not recovered motor/power/gearbox data.
    pub fn drive(&mut self, speed_m_s: f64) -> Result<(), &'static str> {
        if !speed_m_s.is_finite() || speed_m_s.abs() > 1. {
            return Err("invalid fixture drive speed");
        }
        self.suspension.runtime.set_joint_motor(
            self.axle,
            Some(JointMotor::new(
                0.,
                speed_m_s / self.parameters.radius_m,
                0.,
                0.5,
                1.2,
            )?),
        )?;
        self.speed_m_s = speed_m_s;
        Ok(())
    }
    /// Rebuild the same ground, body and drive recipe with fresh runtime tokens.
    pub fn reset(&mut self) -> Result<(), &'static str> {
        let mut replacement = Self::new(self.parameters, self.duration)?;
        replacement.drive(self.speed_m_s)?;
        *self = replacement;
        Ok(())
    }
    /// Wheel binding for explicit ground-truth/stale-token checks.
    pub fn wheel_binding(&self) -> BodyBinding {
        self.wheel
    }
    /// One shared tick, with no second plant, clock or prescribed frame pose.
    pub fn step(&mut self) -> Result<Sample, &'static str> {
        let base = self
            .suspension
            .runtime
            .body_binding(BodyKey(1))
            .ok_or("missing frame")?;
        let before = self.suspension.runtime.body_state(base)?.velocity_m_s[2];
        let s = self.suspension.step()?;
        let velocity = self.suspension.runtime.body_state(base)?.velocity_m_s[2];
        let support = self.parameters.sprung_mass_kg
            * ((velocity - before) / self.duration.as_secs_f64() + 9.81);
        self.observe(s, Some(support))
    }
    /// Read actual poses without advancing. Acceleration-derived support is absent.
    pub fn sample(&self) -> Result<Sample, &'static str> {
        self.observe(self.suspension.sample()?, None)
    }
    fn observe(&self, s: suspension::Sample, support: Option<f64>) -> Result<Sample, &'static str> {
        let runtime = &self.suspension.runtime;
        let base = runtime.body_state(runtime.body_binding(BodyKey(1)).ok_or("missing frame")?)?;
        let wheel = runtime.body_state(self.wheel)?;
        let speed = wheel.angular_velocity_rad_s[1] * self.parameters.radius_m;
        let mut floor_contact = false;
        let mut bump_contact = false;
        let mut minimum = 0_f64;
        for c in runtime.physical_contacts() {
            floor_contact |= c.side1.collider == self.floor || c.side2.collider == self.floor;
            bump_contact |= self
                .bump
                .is_some_and(|b| c.side1.collider == b || c.side2.collider == b);
            minimum = minimum.min(c.geometry.separation_m);
        }
        Ok(Sample {
            suspension: s,
            wheel,
            chassis_velocity_m_s: base.velocity_m_s,
            rolling_speed_m_s: speed,
            slip_m_s: wheel.velocity_m_s[0] - speed,
            chassis_support_n: support,
            floor_contact,
            bump_contact,
            minimum_separation_m: minimum,
            dropped_contacts: runtime.physical_contacts_dropped(),
        })
    }
}
fn write_sample(csv: &mut impl std::io::Write, s: Sample) -> std::io::Result<()> {
    let base = s.suspension.body_poses[0];
    let state = s
        .suspension
        .body_poses
        .into_iter()
        .chain([s.wheel.pose])
        .flat_map(|p| p.translation_m.into_iter().chain(p.rotation_wxyz))
        .map(|v| format!(",{v}"))
        .collect::<String>();
    writeln!(
        csv,
        "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}{}",
        s.suspension.time_s,
        base.translation_m[0],
        base.translation_m[2],
        s.chassis_velocity_m_s[0],
        s.wheel.pose.translation_m[0],
        s.wheel.pose.translation_m[2],
        s.rolling_speed_m_s,
        s.slip_m_s,
        s.suspension.length_m,
        s.suspension.closure_error_m,
        s.chassis_support_n
            .map(|v| v.to_string())
            .unwrap_or_default(),
        u8::from(s.floor_contact),
        u8::from(s.bump_contact),
        s.minimum_separation_m,
        s.dropped_contacts,
        state
    )?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let out = args.next().unwrap_or("wheel-contact.csv".into());
    let mut p = Parameters::default();
    if let Some(v) = args.next() {
        p.bump_height_m = v.parse()?;
    }
    if let Some(v) = args.next() {
        p.friction = v.parse()?;
    }
    let hz = args.next().map(|v| v.parse()).transpose()?.unwrap_or(1000);
    let dt = StepDuration::from_hz(hz)?;
    let mut f = Fixture::new(p, dt)?;
    use std::io::Write;
    let mut csv = std::io::BufWriter::new(std::fs::File::create(out)?);
    let poses = (1..=7)
        .flat_map(|i| ["x", "y", "z", "qw", "qx", "qy", "qz"].map(move |k| format!(",body{i}_{k}")))
        .collect::<String>();
    writeln!(
        csv,
        "time_s,chassis_x_m,chassis_z_m,chassis_vx_m_s,wheel_x_m,wheel_z_m,rolling_m_s,slip_m_s,damper_length_m,loop_error_m,chassis_support_n,floor_contact,bump_contact,separation_m,dropped{poses}"
    )?;
    write_sample(&mut csv, f.sample()?)?;
    let ticks = 8_000_000_000 / dt.as_ns();
    let start = 2_000_000_000 / dt.as_ns();
    let mut last = None;
    for tick in 0..ticks {
        if tick == start {
            f.drive(0.25)?;
        }
        let s = f.step()?;
        write_sample(&mut csv, s)?;
        last = Some(s);
    }
    eprintln!("{p:?}; final {:?}", last.ok_or("no samples")?);
    Ok(())
}
