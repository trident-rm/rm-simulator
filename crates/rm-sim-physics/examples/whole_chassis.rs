// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Free six-DOF chassis with four source-derived closed-loop wheel modules.
//! Flat-floor dynamics prototype; dimensions, upper mass/COM and motors are assumed.
#[allow(dead_code)]
#[path = "wheel_contact.rs"]
mod module;
use glam::{DQuat, DVec3};
use module::{
    RollingWheel,
    suspension::{self, Linkage},
};
use rm_sim_physics::{
    StepDuration,
    cooking::{CookedRigidBody, CookedTriangleMesh},
    runtime::{BodyBinding, BodyKey, PhysicalRuntime},
};
use sim_engine::{
    Pose,
    model::{
        BodyKind, BodyState, ColliderDescription, ContactMaterial, Inertial, RigidBodyDescription,
        ShapeDescription, TriangleMesh,
    },
};

/// Explicit prototype choices, not recovered complete-robot properties.
#[derive(Clone, Copy, Debug)]
pub struct Parameters {
    /// Rigid sprung frame/upper robot mass, kg. Modules add their own mass.
    pub sprung_mass_kg: f64,
    /// Assumed upper mass COM relative to the chassis geometric origin, metres.
    pub com_offset_m: [f64; 3],
    /// Generated one-track ramp height, m; zero is flat terrain.
    pub bump_height_m: f64,
    /// Assumed roller grip; tread grip remains 0.9.
    pub roller_friction: f64,
    /// Explicit native solver iteration budget, inclusive 1..=64.
    pub solver_iterations: usize,
}
impl Default for Parameters {
    fn default() -> Self {
        Self {
            sprung_mass_kg: 20.,
            com_offset_m: [0.; 3],
            bump_height_m: 0.,
            roller_friction: 0.02,
            solver_iterations: 32,
        }
    }
}
/// Assumed wheel centers in chassis XY, FL, FR, RL, RR order, metres.
pub const CENTERS_M: [[f64; 2]; 4] = [[0.22, 0.20], [0.22, -0.20], [-0.22, 0.20], [-0.22, -0.20]];
/// Tangential wheel rolling angles in the chassis XY plane, radians.
pub const ROLLING_ANGLES_RAD: [f64; 4] = [
    3. * std::f64::consts::FRAC_PI_4,
    std::f64::consts::FRAC_PI_4,
    5. * std::f64::consts::FRAC_PI_4,
    7. * std::f64::consts::FRAC_PI_4,
];
fn q(p: Pose) -> DQuat {
    let [w, x, y, z] = p.rotation_wxyz;
    DQuat::from_xyzw(x, y, z, w)
}
fn pose(p: DVec3, q: DQuat) -> Pose {
    Pose {
        translation_m: p.to_array(),
        rotation_wxyz: [q.w, q.x, q.y, q.z],
    }
}
fn material(memberships: u32, filter: u32) -> ContactMaterial {
    ContactMaterial {
        friction: 0.9,
        restitution: 0.,
        memberships,
        filter,
    }
}
/// Actual plant truth after an explicit tick; no estimator or reward state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// Completed time, seconds.
    pub time_s: f64,
    /// Actual free chassis body pose and COM velocities.
    pub body: BodyState,
    /// Linear COM velocity expressed in the current chassis frame, m/s.
    pub velocity_body_m_s: [f64; 3],
    /// Angular velocity expressed in the current chassis frame, rad/s.
    pub angular_body_rad_s: [f64; 3],
    /// Actual FL/FR/RL/RR wheel poses and velocities.
    pub wheels: [BodyState; 4],
    /// Per-wheel longitudinal contact slip proxy, m/s (COM tread projection minus spin*R).
    pub slip_m_s: [f64; 4],
    /// Preceding-slice native normal load magnitudes used by the friction law, N.
    pub loads_n: [Option<f64>; 4],
    /// Actual spring slider lengths, metres.
    pub lengths_m: [f64; 4],
    /// Largest suspension hinge anchor gap, metres.
    pub closure_error_m: f64,
    /// Deepest cached native contact separation, metres.
    pub minimum_separation_m: f64,
    /// Contact observation buffer overflow count.
    pub dropped_contacts: usize,
}
/// All 25 dynamic bodies and 32 mechanical joints share one native runtime.
/// Directional patches add up to four two-axis constraints and one fixed anchor.
/// No chassis degree of freedom is locked and no body pose is prescribed.
pub struct Fixture {
    /// Sole world owner; explicit controller/observer access for experiments.
    pub runtime: PhysicalRuntime,
    /// Assumptions retained for exact recipe reset.
    pub parameters: Parameters,
    duration: StepDuration,
    tick: u64,
    body: BodyBinding,
    modules: [Linkage; 4],
    wheels: [RollingWheel; 4],
    command: Option<[f64; 3]>,
}
impl Fixture {
    /// Install a free chassis, four closed-loop modules, tyre contacts and floor.
    pub fn new(parameters: Parameters, duration: StepDuration) -> Result<Self, &'static str> {
        if !parameters.sprung_mass_kg.is_finite()
            || !(1. ..=50.).contains(&parameters.sprung_mass_kg)
            || !parameters
                .com_offset_m
                .into_iter()
                .all(|v| v.is_finite() && v.abs() <= 0.1)
            || !parameters.bump_height_m.is_finite()
            || !(0. ..=0.02).contains(&parameters.bump_height_m)
        {
            return Err("invalid whole-chassis parameters");
        }
        rm_sim_physics::runtime::DirectionalFriction::new(
            [0., 1., 0.],
            0.9,
            parameters.roller_friction,
        )?;
        let mut runtime =
            PhysicalRuntime::with_solver_iterations(duration, parameters.solver_iterations)?;
        let half = [0.24, 0.18, 0.035];
        let [x, y, z] = half;
        let m = parameters.sprung_mass_kg;
        let source = RigidBodyDescription::new(
            BodyKind::Dynamic(Inertial::new(
                m,
                [
                    m * (y * y + z * z) / 3.,
                    m * (x * x + z * z) / 3.,
                    m * (x * x + y * y) / 3.,
                ],
                Pose::at(parameters.com_offset_m),
            )?),
            vec![ColliderDescription::new(
                ShapeDescription::Cuboid {
                    half_extents_m: half,
                },
                Pose::default(),
                material(1, 2),
            )?],
        )?;
        let body = runtime.install_body(
            BodyKey(1),
            &CookedRigidBody::new(source)?,
            BodyState::at(Pose::at([0., 0., 0.23])),
        )?;
        let mut modules = Vec::new();
        let mut wheels = Vec::new();
        let options = module::Parameters {
            roller_friction: Some(parameters.roller_friction),
            bump_height_m: 0.,
            ..Default::default()
        };
        let bench = DQuat::from_rotation_y(47_f64.to_radians())
            * DQuat::from_rotation_x(std::f64::consts::PI);
        let arm = DVec3::new(0.011754609365650348, 0., -0.022064205371165435);
        for i in 0..4 {
            let yaw = DQuat::from_rotation_z(ROLLING_ANGLES_RAD[i]);
            let rotation = yaw * bench;
            let center = DVec3::new(CENTERS_M[i][0], CENTERS_M[i][1], options.radius_m + 0.002);
            let root = center - rotation * (arm + DVec3::from_array(module::HUB_CARRIER_M));
            let placement = pose(root - rotation * DVec3::new(0., 0., 0.3), rotation);
            let namespace = (i as u64 + 1) * 100;
            let linkage = Linkage::attach(
                &mut runtime,
                suspension::Parameters {
                    load_kg: options.carrier_mass_kg,
                    ..Default::default()
                },
                placement,
                body,
                namespace,
            )?;
            let wheel =
                RollingWheel::install(&mut runtime, linkage.carrier(), options, namespace, yaw)?;
            modules.push(linkage);
            wheels.push(wheel);
        }
        runtime.install_catch_floor(0., material(2, 1))?;
        if parameters.bump_height_m > 0. {
            // Narrow left-side track, from x=0.65 to 0.89; asymmetric wheel input.
            let mut v = Vec::new();
            for (x, z) in [
                (0.65, 0.),
                (0.73, parameters.bump_height_m),
                (0.81, parameters.bump_height_m),
                (0.89, 0.),
            ] {
                v.push([x, 0.10, z]);
                v.push([x, 0.30, z]);
            }
            let mut t = Vec::new();
            for i in 0..3u32 {
                let k = 2 * i;
                t.push([k, k + 2, k + 1]);
                t.push([k + 1, k + 2, k + 3]);
            }
            runtime.install_fixed_mesh(
                &CookedTriangleMesh::new(TriangleMesh::new(v, t)?)?,
                Pose::default(),
                material(2, 1),
            )?;
        }
        runtime.enable_contact_observations(256)?;
        let mut f = Self {
            runtime,
            parameters,
            duration,
            tick: 0,
            body,
            modules: modules.try_into().ok().expect("four modules"),
            wheels: wheels.try_into().ok().expect("four wheels"),
            command: Some([0.; 3]),
        };
        f.command(0., 0., 0.)?;
        Ok(f)
    }
    /// Apply geometric-origin body-frame vx/vy (m/s) and yaw (rad/s) through axle motors only.
    /// Reject nonfinite/out-of-range wheel targets before changing any motor.
    pub fn command(&mut self, vx: f64, vy: f64, yaw: f64) -> Result<(), &'static str> {
        let targets = std::array::from_fn::<_, 4, _>(|i| {
            let [x, y] = CENTERS_M[i];
            let (s, c) = ROLLING_ANGLES_RAD[i].sin_cos();
            (vx - yaw * y) * c + (vy + yaw * x) * s
        });
        if ![vx, vy, yaw].into_iter().all(f64::is_finite)
            || !targets.into_iter().all(|v| v.is_finite() && v.abs() <= 1.)
        {
            return Err("invalid chassis wheel targets");
        }
        for (wheel, target) in self.wheels.iter().zip(targets) {
            wheel.drive(&mut self.runtime, target)?;
        }
        self.command = Some([vx, vy, yaw]);
        Ok(())
    }
    /// Clear all axle motors, retaining passive ground and suspension dynamics.
    pub fn coast(&mut self) -> Result<(), &'static str> {
        for wheel in &self.wheels {
            wheel.coast(&mut self.runtime)?;
        }
        self.command = None;
        Ok(())
    }
    /// Command-owned root binding for privileged truth and stale-token checks.
    pub fn body_binding(&self) -> BodyBinding {
        self.body
    }
    /// Modeled total mass including all four modules, kg; not a CAD mass claim.
    pub fn total_mass_kg(&self) -> f64 {
        let wheel = module::Parameters::default();
        self.parameters.sprung_mass_kg
            + 4. * (wheel.carrier_mass_kg
                + wheel.wheel_mass_kg
                + suspension::Parameters::default().damper_mass_kg
                + 2. * suspension::ARM_MASS_KG)
    }
    /// Reconstruct the initial plant and current held command with fresh tokens.
    pub fn reset(&mut self) -> Result<(), &'static str> {
        let mut new = Self::new(self.parameters, self.duration)?;
        if let Some([vx, vy, yaw]) = self.command {
            new.command(vx, vy, yaw)?;
        } else {
            new.coast()?;
        }
        *self = new;
        Ok(())
    }
    /// Advance all modules and chassis through one shared explicit tick.
    pub fn step(&mut self) -> Result<Sample, &'static str> {
        let t = self
            .tick
            .checked_mul(self.duration.as_ns())
            .ok_or("fixture time overflow")?;
        self.runtime.step(t)?;
        self.tick += 1;
        self.sample()
    }
    /// Read actual frames, contact budgets and loop diagnostics without stepping.
    pub fn sample(&self) -> Result<Sample, &'static str> {
        let body = self.runtime.body_state(self.body)?;
        let inverse = q(body.pose).conjugate();
        let mut states = [BodyState::at(Pose::default()); 4];
        let mut slip = [0.; 4];
        let mut loads = [None; 4];
        let mut lengths = [0.; 4];
        let mut gap = 0_f64;
        for i in 0..4 {
            let state = self.runtime.body_state(self.wheels[i].body())?;
            let rotation = q(state.pose);
            let axle = rotation * DVec3::Y;
            let tread = axle.cross(q(body.pose) * DVec3::Z).normalize_or_zero();
            slip[i] = DVec3::from_array(state.velocity_m_s).dot(tread)
                - DVec3::from_array(state.angular_velocity_rad_s).dot(axle)
                    * self.wheels[i].radius_m();
            loads[i] = self
                .runtime
                .directional_normal_load_n(self.wheels[i].collider())?;
            let module = self.modules[i].sample(
                &self.runtime,
                self.tick as f64 * self.duration.as_secs_f64(),
            )?;
            lengths[i] = module.length_m;
            gap = gap.max(module.closure_error_m);
            states[i] = state;
        }
        Ok(Sample {
            time_s: self.tick as f64 * self.duration.as_secs_f64(),
            body,
            velocity_body_m_s: (inverse * DVec3::from_array(body.velocity_m_s)).to_array(),
            angular_body_rad_s: (inverse * DVec3::from_array(body.angular_velocity_rad_s))
                .to_array(),
            wheels: states,
            slip_m_s: slip,
            loads_n: loads,
            lengths_m: lengths,
            closure_error_m: gap,
            minimum_separation_m: self
                .runtime
                .physical_contacts()
                .iter()
                .map(|c| c.geometry.separation_m)
                .fold(0., f64::min),
            dropped_contacts: self.runtime.physical_contacts_dropped(),
        })
    }
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args.next().unwrap_or("whole-chassis.csv".into());
    let mode = args.next().unwrap_or("forward".into());
    let hz = args.next().map(|v| v.parse()).transpose()?.unwrap_or(1000);
    let bump = args.next().map(|v| v.parse()).transpose()?.unwrap_or(0.);
    let com_x = args.next().map(|v| v.parse()).transpose()?.unwrap_or(0.);
    let target = match mode.as_str() {
        "forward" | "coast" => [0.25, 0., 0.],
        "sideways" => [0., 0.25, 0.],
        "diagonal" => [0.18, 0.18, 0.],
        "spin" => [0., 0., 0.6],
        "combined" => [0.15, 0.10, 0.3],
        "settle" => [0.; 3],
        _ => return Err("unknown fixture mode".into()),
    };
    let dt = StepDuration::from_hz(hz)?;
    let mut f = Fixture::new(
        Parameters {
            bump_height_m: bump,
            com_offset_m: [com_x, 0., 0.],
            ..Default::default()
        },
        dt,
    )?;
    let mut csv = std::io::BufWriter::new(std::fs::File::create(path)?);
    use std::io::Write;
    writeln!(
        csv,
        "time_s,x_m,y_m,z_m,vx_m_s,vy_m_s,wz_rad_s,qw,qx,qy,qz,load_fl_n,load_fr_n,load_rl_n,load_rr_n,length_fl_m,length_fr_m,length_rl_m,length_rr_m,loop_error_m,separation_m,dropped,slip_fl_m_s,slip_fr_m_s,slip_rl_m_s,slip_rr_m_s"
    )?;
    let start = std::time::Instant::now();
    let mut last = f.sample()?;
    for tick in 0..10_000_000_000 / dt.as_ns() {
        if tick == 2_000_000_000 / dt.as_ns() {
            f.command(target[0], target[1], target[2])?;
        }
        if tick == 8_000_000_000 / dt.as_ns() {
            if mode == "coast" {
                f.coast()?;
            } else {
                f.command(0., 0., 0.)?;
            }
        }
        let s = f.step()?;
        let p = s.body.pose;
        let mut row = vec![
            s.time_s,
            p.translation_m[0],
            p.translation_m[1],
            p.translation_m[2],
            s.velocity_body_m_s[0],
            s.velocity_body_m_s[1],
            s.angular_body_rad_s[2],
        ];
        row.extend(p.rotation_wxyz);
        let mut fields = row.into_iter().map(|v| v.to_string()).collect::<Vec<_>>();
        fields.extend(
            s.loads_n
                .map(|v| v.map(|v| v.to_string()).unwrap_or_default()),
        );
        fields.extend(s.lengths_m.map(|v| v.to_string()));
        fields.extend([
            s.closure_error_m.to_string(),
            s.minimum_separation_m.to_string(),
            s.dropped_contacts.to_string(),
        ]);
        fields.extend(s.slip_m_s.map(|v| v.to_string()));
        writeln!(csv, "{}", fields.join(","))?;
        last = s;
    }
    eprintln!(
        "elapsed including CSV {:?}; modeled mass {} kg; final {last:?}",
        start.elapsed(),
        f.total_mass_kg()
    );
    Ok(())
}
